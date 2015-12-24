#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "conf_general.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "spi_sw.h"
#include "rf.h"
#include "rfhelp.h"
#include "buffer.h"

// Settings
#define ADC_GRP1_NUM_CHANNELS   10
#define ADC_GRP1_BUF_DEPTH      1

#define ADC_IND_VREF			0
#define ADC_IND_TEMP			1
#define ADC_IND_VBAT_A			8
#define ADC_IND_VBAT_B			9

#define ALIVE_TIMEOUT_MS		10000

// Macros
#define ACTIVATE_BUTTONS()		palClearPad(CHUK_P1_GND_PORT, CHUK_P1_GND_PIN); \
		palClearPad(CHUK_P2_GND_PORT, CHUK_P2_GND_PIN)

#define DEACTIVATE_BUTTONS()	palSetPad(CHUK_P1_GND_PORT, CHUK_P1_GND_PIN); \
		palSetPad(CHUK_P2_GND_PORT, CHUK_P2_GND_PIN)

#define READ_BT_Z()				(!palReadPad(CHUK_BT_Z_PORT, CHUK_BT_Z_PIN))
#define READ_BT_C()				(!palReadPad(CHUK_BT_C_PORT, CHUK_BT_C_PIN))
#define READ_BT_PUSH()			(!palReadPad(CHUK_BT_PUSH_PORT, CHUK_BT_PUSH_PIN))
#define SHOW_VBAT_NOW()			(READ_BT_PUSH() || (READ_BT_C() && READ_BT_Z()))

#define READ_ADDR()				((!!palReadPad(ADDR0_PORT, ADDR0_PIN)) | \
		(!!palReadPad(ADDR1_PORT, ADDR1_PIN) << 1) | \
		(!!palReadPad(ADDR2_PORT, ADDR2_PIN) << 2) | \
		(!!palReadPad(ADDR3_PORT, ADDR3_PIN) << 3))

#define LED_RED_ON()			palSetPad(LED_RED_PORT, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_PORT, LED_RED_PIN)
#define LED_RED_TOGGLE()		palTogglePad(LED_RED_PORT, LED_RED_PIN)
#define LED_GREEN_ON()			palSetPad(LED_GREEN_PORT, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_PORT, LED_GREEN_PIN)
#define LED_GREEN_TOGGLE()		palTogglePad(LED_GREEN_PORT, LED_GREEN_PIN)

#define IS_ALIVE()				(chTimeElapsedSince(alive_timestamp) < MS2ST(ALIVE_TIMEOUT_MS))

#define CR_DS_MASK				((uint32_t)0xFFFFFFFC)
#define PWR_Regulator_ON		((uint32_t)0x00000000)
#define PWR_Regulator_LowPower	((uint32_t)0x00000001)

// Datatypes
typedef struct {
	uint8_t js_x;
	uint8_t js_y;
	bool bt_c;
	bool bt_z;
	bool bt_push;
	float vin;
	float vbat;
} mote_state;

typedef enum {
	MOTE_PACKET_BATT_LEVEL = 0,
	MOTE_PACKET_BUTTONS,
	MOTE_PACKET_ALIVE
} MOTE_PACKET;

// Variables
static Mutex print_mutex;
static Mutex read_mutex;
static WORKING_AREA(rx_thread_wa, 512);
static WORKING_AREA(tx_thread_wa, 512);
static int address = 0;
static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static systime_t alive_timestamp = 0;
static Thread *rx_tp;
static Thread *tx_tp;
static bool rx_thd_is_waiting = false;
static bool tx_thd_is_waiting = false;
static bool rx_now = false;

// Functions
void printf_thd(const char* format, ...);
void print_rf_status(void);
static msg_t tx_thread(void *arg);
static msg_t rx_thread(void *arg);
static void read_mote_state(mote_state *data);
static void show_vbat(void);

const SerialConfig sc1_config =
{
		115200,     // baud rate
		0,          // CR1 register
		0,          // CR2 register
		0           // CR3 register
};

/*
 * ADC conversion group.
 * Mode:        Continuous, 4 samples of 4 channels, SW triggered.
 * Channels:    IN0, IN1, Sensor, VRef.
 */
static const ADCConversionGroup adcgrpcfg = {
		FALSE,
		ADC_GRP1_NUM_CHANNELS,
		0,
		0,
		0, ADC_CR2_TSVREFE,           /* CR1, CR2 */
		ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_239P5) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_239P5),
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_239P5) |
		ADC_SMPR2_SMP_AN2(ADC_SAMPLE_239P5),
		ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
		ADC_SQR2_SQ10_N(ADC_CHANNEL_IN2) | ADC_SQR2_SQ9_N(ADC_CHANNEL_IN2) |
		ADC_SQR2_SQ8_N(ADC_CHANNEL_IN1) | ADC_SQR2_SQ7_N(ADC_CHANNEL_IN0),
		ADC_SQR3_SQ6_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN0) |
		ADC_SQR3_SQ4_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN0) |
		ADC_SQR3_SQ2_N(ADC_CHANNEL_SENSOR) | ADC_SQR3_SQ1_N(ADC_CHANNEL_VREFINT)
};

void printf_thd(const char* format, ...) {
	va_list arg;
	va_start(arg, format);

	chMtxLock(&print_mutex);
	chvprintf((BaseSequentialStream*)&SD1, format, arg);
	chMtxUnlock();
}

void print_rf_status(void) {
	int s = rf_status();

	printf_thd("RF Status Register\r\n");
	printf_thd("RX_DR  TX_DS  MAX_RT  RX_P_NO  TX_FULL\r\n");
	printf_thd("%i      %i      %i       %i        %i\r\n",
			NRF_STATUS_GET_RX_DR(s), NRF_STATUS_GET_TX_DS(s), NRF_STATUS_GET_MAX_RT(s),
			NRF_STATUS_GET_RX_P_NO(s), NRF_STATUS_GET_TX_FULL(s));
}

static void read_mote_state(mote_state *data) {
	chMtxLock(&read_mutex);
	ACTIVATE_BUTTONS();
	VBAT_ON();

	chThdSleepMilliseconds(1);

	adcConvert(&ADCD1, &adcgrpcfg, adc_samples, ADC_GRP1_BUF_DEPTH);

	for (int i = 0;i < 2;i++) {
		adc_samples[ADC_IND_CHUK_PX] += adc_samples[ADC_IND_CHUK_PX + 2 * (i + 1)];
		adc_samples[ADC_IND_CHUK_PY] += adc_samples[ADC_IND_CHUK_PY + 2 * (i + 1)];
	}

	adc_samples[ADC_IND_CHUK_PX] /= 3;
	adc_samples[ADC_IND_CHUK_PY] /= 3;

#if ADC_INVERT_PX
	adc_samples[ADC_IND_CHUK_PX] = 4095 - adc_samples[ADC_IND_CHUK_PX];
#endif
#if ADC_INVERT_PY
	adc_samples[ADC_IND_CHUK_PY] = 4095 - adc_samples[ADC_IND_CHUK_PY];
#endif

	data->js_x = adc_samples[ADC_IND_CHUK_PX] >> 4;
	data->js_y = adc_samples[ADC_IND_CHUK_PY] >> 4;
	data->bt_c = READ_BT_C();
	data->bt_z = READ_BT_Z();
	data->bt_push = READ_BT_PUSH();
	data->vin = 1.2 / ((float)adc_samples[ADC_IND_VREF] / 4095.0);

#ifdef HAS_VBAT_EXTRA
	float vbat = (float)(adc_samples[ADC_IND_VBAT_A] + adc_samples[ADC_IND_VBAT_B]);
	vbat /= 2.0;
	vbat /= 4095.0;
	vbat *= 3.3;
	data->vbat = vbat * 2.0;
#else
	data->vbat = data->vin;
#endif

	VBAT_OFF();
	DEACTIVATE_BUTTONS();
	chMtxUnlock();
}

static void show_vbat(void) {
	mote_state state;
	read_mote_state(&state);

	LED_RED_OFF();
	LED_GREEN_OFF();

	float v1, v2, v3;

#if HAS_VBAT_EXTRA
	v1 = 4.0;
	v2 = 3.6;
	v3 = 3.2;
#else
	v1 = 2.9;
	v2 = 2.5;
	v3 = 2.3;
#endif

	chThdSleepMilliseconds(1000);

	const float v = state.vbat;
	if (v > v1) {
		LED_GREEN_ON();
		chThdSleepMilliseconds(3000);
	} else if (v > v2) {
		for (int i = 0;i < 3;i++) {
			LED_GREEN_TOGGLE();
			chThdSleepMilliseconds(500);
			LED_GREEN_TOGGLE();
			chThdSleepMilliseconds(500);
		}
	} else if (v > v3) {
		for (int i = 0;i < 3;i++) {
			LED_RED_TOGGLE();
			chThdSleepMilliseconds(500);
			LED_RED_TOGGLE();
			chThdSleepMilliseconds(500);
		}
	} else {
		LED_RED_ON();
		chThdSleepMilliseconds(3000);
	}

	LED_RED_OFF();
	LED_GREEN_OFF();

	chThdSleepMilliseconds(1000);
}

static msg_t tx_thread(void *arg) {
	(void)arg;

	tx_tp = chThdSelf();
	chRegSetThreadName("TX");

	for(;;) {
		if (!IS_ALIVE()) {
			tx_thd_is_waiting = true;
			chEvtWaitAny((eventmask_t) 1);
			tx_thd_is_waiting = false;
		}

		mote_state state;
		read_mote_state(&state);

		uint8_t pl[6];
		int32_t index = 0;
		pl[index++] = MOTE_PACKET_BUTTONS;
		pl[index++] = state.js_x;
		pl[index++] = state.js_y;
		pl[index++] = state.bt_c | (state.bt_z << 1) | (state.bt_push << 2);
		buffer_append_int16(pl, (int16_t)(state.vbat * 1000.0), &index);

		rfhelp_power_up();
		chThdSleepMilliseconds(2);
		rfhelp_send_data_crc((char*)pl, index);

		// Turn on RX for 5 of 30 iterations
		static int rx_cnt = 0;
		rx_cnt++;
		if (rx_cnt >= 30) {
			rx_cnt = 0;
		}

		if (rx_cnt < 25) {
			rx_now = false;
			rfhelp_power_down();
		} else {
			rx_now = true;
		}

		chThdSleepMilliseconds(20);
	}

	return 0;
}

static msg_t rx_thread(void *arg) {
	(void)arg;

	rx_tp = chThdSelf();
	chRegSetThreadName("RX");

	int retry_cnt = 0;

	for(;;) {
		if (!IS_ALIVE()) {
			if (retry_cnt < 20) {
				retry_cnt++;
			} else {
				rx_thd_is_waiting = true;
				chEvtWaitAny((eventmask_t) 1);
				rx_thd_is_waiting = false;
				retry_cnt = 0;
			}
		}

		if (!rx_now) {
			chThdSleepMilliseconds(10);
			continue;
		}

		char buf[32];
		int len;
		int pipe;

		for(;;) {
			int res = rfhelp_read_rx_data_crc(buf, &len, &pipe);

			// If something was read
			if (res >= 0) {
				MOTE_PACKET packet = buf[0];

				switch (packet) {
				case MOTE_PACKET_BATT_LEVEL:
					// TODO!
					break;

				case MOTE_PACKET_ALIVE:
					alive_timestamp = chTimeNow();
					break;

				default:
					break;
				}
			}

			// Stop when there is no more data to read.
			if (res <= 0) {
				break;
			}  else {
				// Sleep a bit to prevent locking the other threads.
				chThdSleepMilliseconds(5);
			}
		}

		chThdSleepMilliseconds(10);
	}

	return 0;
}

int main(void) {
	halInit();
	chSysInit();

	// Disable JTAG and SWD
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;

	// Disable some clocks
	RCC->AHBENR &= ~(RCC_AHBENR_CRCEN | RCC_AHBENR_DMA1EN);
	RCC->APB2ENR &= ~(RCC_APB2ENR_IOPEEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPCEN);

	// Event on buttons for wakeup
#ifdef HW_KAMA
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PA;
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PB | AFIO_EXTICR2_EXTI7_PB;
	EXTI->EMR |= EXTI_EMR_MR2 | EXTI_EMR_MR5 | EXTI_EMR_MR7;
	EXTI->FTSR |= EXTI_FTSR_TR2 | EXTI_FTSR_TR5 | EXTI_FTSR_TR7;
#elif defined HW_JACOB
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PA;
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PB | AFIO_EXTICR2_EXTI7_PB;
	EXTI->EMR |= EXTI_EMR_MR2 | EXTI_EMR_MR5 | EXTI_EMR_MR7;
	EXTI->FTSR |= EXTI_FTSR_TR2 | EXTI_FTSR_TR5 | EXTI_FTSR_TR7;
#elif defined HW_V5
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PB | AFIO_EXTICR3_EXTI11_PB;
	EXTI->EMR |= EXTI_EMR_MR10 | EXTI_EMR_MR11;
	EXTI->FTSR |= EXTI_FTSR_TR10 | EXTI_FTSR_TR11;
#endif

	// LED Pins
	palSetPadMode(LED_RED_PORT, LED_RED_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(LED_GREEN_PORT, LED_GREEN_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	// Initialize serial port driver
	sdStart(&SD1, &sc1_config);

	// Serial pins
	palSetPadMode(GPIOA, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(GPIOA, 10, PAL_MODE_INPUT);

	// Initialize mutexes
	chMtxInit(&print_mutex);
	chMtxInit(&read_mutex);

	// NRF
	rf_init();
	rfhelp_init();

	print_rf_status();

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);

		// Vbat transistor
#if HAS_VBAT_EXTRA
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(VBAT_PORT, VBAT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	VBAT_OFF();
#endif

	// ADC
	adcStart(&ADCD1, NULL);

	// IO Pins
	palSetPadMode(CHUK_BT_Z_PORT, CHUK_BT_Z_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(CHUK_BT_C_PORT, CHUK_BT_C_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(CHUK_BT_PUSH_PORT, CHUK_BT_PUSH_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(CHUK_P1_GND_PORT, CHUK_P1_GND_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(CHUK_P2_GND_PORT, CHUK_P2_GND_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	// Read ADDR Pins
	palSetPadMode(ADDR0_PORT, ADDR0_PIN, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(ADDR1_PORT, ADDR1_PIN, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(ADDR2_PORT, ADDR2_PIN, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(ADDR3_PORT, ADDR3_PIN, PAL_MODE_INPUT_PULLDOWN);

	chThdSleepMilliseconds(5);
	address = READ_ADDR();

	// Remove pulldowns to save power
	palSetPadMode(ADDR0_PORT, ADDR0_PIN, PAL_MODE_INPUT);
	palSetPadMode(ADDR1_PORT, ADDR1_PIN, PAL_MODE_INPUT);
	palSetPadMode(ADDR2_PORT, ADDR2_PIN, PAL_MODE_INPUT);
	palSetPadMode(ADDR3_PORT, ADDR3_PIN, PAL_MODE_INPUT);

	// Set RF address
	char addr[5] = {0xC4, 0xC5, 0xC6, 0xC7, 0x00};
	addr[4] = address;
	rfhelp_set_rx_addr(0, addr, 5);
	rfhelp_set_tx_addr(addr, 5);

	// Start threads
	chThdCreateStatic(rx_thread_wa, sizeof(rx_thread_wa), NORMALPRIO, rx_thread, NULL);
	chThdCreateStatic(tx_thread_wa, sizeof(tx_thread_wa), NORMALPRIO, tx_thread, NULL);

	LED_RED_OFF();
	LED_GREEN_OFF();

	alive_timestamp = chTimeNow();

	for(;;) {
		static bool was_alive = true;

		if (IS_ALIVE()) {
			if (!was_alive) {
				// Power everything up if we weren't alive previously.
				chEvtSignal(rx_tp, (eventmask_t) 1);
				chEvtSignal(tx_tp, (eventmask_t) 1);
			}

			was_alive = true;
			LED_GREEN_ON();

			// Reset nrf just in case
			static int restart_cnt = 0;
			restart_cnt++;
			if (restart_cnt >= 10) {
				restart_cnt = 0;

				print_rf_status();
				rfhelp_power_up();
				rfhelp_restart();
				print_rf_status();

				mote_state state;
				read_mote_state(&state);

				printf_thd(
						"BT_C: %d\r\n"
						"BT_Z: %d\r\n"
						"BT_P: %d\r\n"
						"PX:   %d\r\n"
						"PY:   %d\r\n"
						"Vbat: %f\r\n"
						"Vin:  %f\r\n\r\n",
						state.bt_c, state.bt_z, state.bt_push,
						state.js_x, state.js_y, state.vbat, state.vin);
			}

			if (SHOW_VBAT_NOW()) {
				show_vbat();
			}

			chThdSleepMilliseconds(50);
		} else {
			// Wait for threads (TODO: use proper ChibiOS style...)
			while (!(rx_thd_is_waiting && tx_thd_is_waiting)) {
				chThdSleepMilliseconds(1);
			}

			// Power everything down
			was_alive = false;
			LED_GREEN_OFF();

			rfhelp_restart();
			rfhelp_power_down();
			spi_sw_write_ce(0);

			// Enter low power deepsleep
			PWR->CR |= PWR_CR_LPDS;
			SCB->SCR |= SCB_SCR_SLEEPDEEP;
			__WFE();
			SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);

			// Power up the NRF and see if there are alive packets
			spi_sw_write_ce(1);
			rfhelp_restart();
			rx_now = true;
			chEvtSignal(rx_tp, (eventmask_t) 1);

			// Wait for the thread (TODO: use proper ChibiOS style...)
			chThdSleepMilliseconds(5);
			while (!rx_thd_is_waiting && !IS_ALIVE()) {
				chThdSleepMilliseconds(5);
			}

			rx_now = false;

			if (SHOW_VBAT_NOW() && !IS_ALIVE()) {
				show_vbat();
			}

			while (SHOW_VBAT_NOW()) {
				chThdSleepMilliseconds(10);
			}
		}
	}

	for(;;) {
		LED_RED_TOGGLE();
		chThdSleepMilliseconds(50);

		mote_state state;
		read_mote_state(&state);

		if (state.bt_push) {
			LED_GREEN_ON();
		} else {
			LED_GREEN_OFF();
		}

		printf_thd(
				"BT_C: %d\r\n"
				"BT_Z: %d\r\n"
				"BT_P: %d\r\n"
				"PX:   %d\r\n"
				"PY:   %d\r\n"
				"Vbat: %f\r\n"
				"Vin:  %f\r\n\r\n",
				state.bt_c, state.bt_z, state.bt_push,
				state.js_x, state.js_y, state.vbat, state.vin);
	}

	return 0;
}
