#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "spi_sw.h"
#include "rf.h"
#include "rfhelp.h"
#include "buffer.h"

// Settings
#define ADC_GRP1_NUM_CHANNELS   8
#define ADC_GRP1_BUF_DEPTH      1

#define ADC_IND_CHUK_PX			3
#define ADC_IND_CHUK_PY			2
#define ADC_IND_VREF			0
#define ADC_IND_TEMP			1

#define CHUK_P1_GND_PORT		GPIOB
#define CHUK_P1_GND_PIN			6
#define CHUK_P2_GND_PORT		GPIOA
#define CHUK_P2_GND_PIN			11

#define CHUK_BT_Z_PORT			GPIOB
#define CHUK_BT_Z_PIN			5
#define CHUK_BT_C_PORT			GPIOB
#define CHUK_BT_C_PIN			7
#define CHUK_BT_PUSH_PORT		GPIOA
#define CHUK_BT_PUSH_PIN		2

#define ADDR0_PORT				GPIOA
#define ADDR0_PIN				14
#define ADDR1_PORT				GPIOA
#define ADDR1_PIN				15
#define ADDR2_PORT				GPIOB
#define ADDR2_PIN				3
#define ADDR3_PORT				GPIOB
#define ADDR3_PIN				4

#define LED_RED_PORT			GPIOB
#define LED_RED_PIN				13
#define LED_GREEN_PORT			GPIOB
#define LED_GREEN_PIN			14

// Macros
#define ACTIVATE_BUTTONS()		palClearPad(CHUK_P1_GND_PORT, CHUK_P1_GND_PIN); \
								palClearPad(CHUK_P2_GND_PORT, CHUK_P2_GND_PIN)

#define DEACTIVATE_BUTTONS()	palSetPad(CHUK_P1_GND_PORT, CHUK_P1_GND_PIN); \
								palSetPad(CHUK_P2_GND_PORT, CHUK_P2_GND_PIN)

#define READ_BT_Z()				(!palReadPad(CHUK_BT_Z_PORT, CHUK_BT_Z_PIN))
#define READ_BT_C()				(!palReadPad(CHUK_BT_C_PORT, CHUK_BT_C_PIN))
#define READ_BT_PUSH()			(!palReadPad(CHUK_BT_PUSH_PORT, CHUK_BT_PUSH_PIN))

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

// Datatypes
typedef struct {
	uint8_t js_x;
	uint8_t js_y;
	bool bt_c;
	bool bt_z;
	bool bt_push;
	float vbat;
} mote_state;

typedef enum {
	MOTE_PACKET_BATT_LEVEL = 0,
	MOTE_PACKET_BUTTONS
} MOTE_PACKET;

// Variables
static Mutex print_mutex;
static Mutex read_mutex;
static WORKING_AREA(rx_thread_wa, 512);
static WORKING_AREA(tx_thread_wa, 512);
static int address = 0;
static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
void read_mote_state(mote_state *data);

// Functions
void printf_thd(const char* format, ...);
void print_rf_status(void);
static msg_t tx_thread(void *arg);
static msg_t rx_thread(void *arg);

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
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_239P5),
		ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
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

void read_mote_state(mote_state *data) {
	chMtxLock(&read_mutex);
	ACTIVATE_BUTTONS();
	chThdSleepMilliseconds(1);

	adcConvert(&ADCD1, &adcgrpcfg, adc_samples, ADC_GRP1_BUF_DEPTH);

	for (int i = 0;i < 2;i++) {
		adc_samples[ADC_IND_CHUK_PX] += adc_samples[ADC_IND_CHUK_PX + 2 * (i + 1)];
		adc_samples[ADC_IND_CHUK_PY] += adc_samples[ADC_IND_CHUK_PY + 2 * (i + 1)];
	}

	adc_samples[ADC_IND_CHUK_PX] /= 3;
	adc_samples[ADC_IND_CHUK_PY] /= 3;

	data->js_x = adc_samples[ADC_IND_CHUK_PX] >> 4;
	data->js_y = adc_samples[ADC_IND_CHUK_PY] >> 4;
	data->bt_c = READ_BT_C();
	data->bt_z = READ_BT_Z();
	data->bt_push = READ_BT_PUSH();
	data->vbat = 1.2 / ((float)adc_samples[ADC_IND_VREF] / 4095.0);

	DEACTIVATE_BUTTONS();
	chMtxUnlock();
}

static msg_t tx_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("TX");

	for(;;) {
		mote_state state;
		read_mote_state(&state);

		uint8_t pl[6];
		int32_t index = 0;
		pl[index++] = MOTE_PACKET_BUTTONS;
		pl[index++] = state.js_x;
		pl[index++] = state.js_y;
		pl[index++] = state.bt_c | (state.bt_z << 1) | (state.bt_push << 2);
		buffer_append_int16(pl, (int16_t)(state.vbat * 1000.0), &index);
		rfhelp_send_data((char*)pl, index);

		chThdSleepMilliseconds(20);
	}

	return 0;
}

static msg_t rx_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("RX");

	for(;;) {
		char buf[32];
		int len;
		int pipe;

		for(;;) {
			int res = rfhelp_read_rx_data(buf, &len, &pipe);

			// If something was read
			if (res >= 0) {
				MOTE_PACKET packet = buf[0];

				switch (packet) {
				case MOTE_PACKET_BATT_LEVEL:
					// TODO!
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

	// ADC
	adcStart(&ADCD1, NULL);

	// IO Pins
	palSetPadMode(CHUK_BT_Z_PORT, CHUK_BT_Z_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(CHUK_BT_C_PORT, CHUK_BT_C_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(CHUK_BT_PUSH_PORT, CHUK_BT_PUSH_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(CHUK_P1_GND_PORT, CHUK_P1_GND_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(CHUK_P2_GND_PORT, CHUK_P2_GND_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	// Read ADDR Pins
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE; // Disable jtag
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

	for(;;) {
		chThdSleepMilliseconds(50);

		// Reset nrf just in case
		rfhelp_restart();

		if (READ_BT_PUSH()) {
			mote_state state;
			read_mote_state(&state);

			LED_RED_OFF();
			LED_GREEN_OFF();

			const float v = state.vbat;
			if (v > 2.9) {
				LED_GREEN_ON();
				chThdSleepMilliseconds(3000);
			} else if (v > 2.5) {
				for (int i = 0;i < 3;i++) {
					LED_GREEN_TOGGLE();
					chThdSleepMilliseconds(500);
					LED_GREEN_TOGGLE();
					chThdSleepMilliseconds(500);
				}
			} else if (v > 2.3) {
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
		} else {
			LED_RED_OFF();
			LED_GREEN_OFF();
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
				"Vin:  %f\r\n\r\n",
				state.bt_c, state.bt_z, state.bt_push,
				state.js_x, state.js_y, state.vbat);
	}

	return 0;
}
