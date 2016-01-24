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
#include "packet.h"
#include "crc.h"
#include "memstreams.h"

// Settings
#define ADC_GRP1_NUM_CHANNELS   10
#define ADC_GRP1_BUF_DEPTH      1

#define ADC_IND_VREF			0
#define ADC_IND_TEMP			1
#define ADC_IND_VBAT_A			8
#define ADC_IND_VBAT_B			9

#define ALIVE_TIMEOUT_MS		10000

#define MAX_PL_LEN				25
#define RX_BUFFER_SIZE			PACKET_MAX_PL_LEN
#define SERIAL_RX_BUFFER_SIZE	1024

#define USE_PRINTF				0    // 0 = off, 1 = direct, 2 = BLDC Tool
#define PRINT_MAIN				0    // Print some stats
#define PRINT_NRF_STATS			1    // Print NRF packet stats
#define TX_DISABLE_TIME			200  // Disable the chuck packets for this time when the uart bridge is used
#define RX_ENABLE_TIME			200  // Keep RX on for at least this time when the uart bridge is used
#define NRF_RESTART_TIMEOUT		500  // Restart the NRF if nothing has been received or acked for this time

// Don't use ack on tx (requires the same setting on the vesc too)
#define NOACK					0
#define TX_RESENDS				1

// Hardcoded address (instead of using the resistors)
#define USE_HARD_ADDR			0
#define HARD_ADDR				5

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

#define IS_ALIVE()				(chVTTimeElapsedSinceX(alive_timestamp) < MS2ST(ALIVE_TIMEOUT_MS))

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

typedef struct {
	uint32_t req_values;
	uint32_t res_values;
	uint32_t tx_ok;
	uint32_t tx_max_rt;
	uint32_t tx_timeout;
} nrf_stats_t;

typedef enum {
	MOTE_PACKET_BATT_LEVEL = 0,
	MOTE_PACKET_BUTTONS,
	MOTE_PACKET_ALIVE,
	MOTE_PACKET_FILL_RX_BUFFER,
	MOTE_PACKET_FILL_RX_BUFFER_LONG,
	MOTE_PACKET_PROCESS_RX_BUFFER,
	MOTE_PACKET_PROCESS_SHORT_BUFFER,
} MOTE_PACKET;

// Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA
} COMM_PACKET_ID;

// Variables
static mutex_t print_mutex;
static mutex_t read_mutex;
static THD_WORKING_AREA(rx_thread_wa, 256);
static THD_WORKING_AREA(tx_thread_wa, 256);
static THD_WORKING_AREA(packet_process_thread_wa, 256);
static int address = 0;
static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static systime_t alive_timestamp = 0;
static thread_t *rx_tp;
static thread_t *tx_tp;
static thread_t *process_tp;
static bool rx_thd_is_waiting = false;
static bool tx_thd_is_waiting = false;
static bool rx_now = false;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static int tx_disable_time = 0;
static int rx_enable_time = 0;
static nrf_stats_t nrf_stats;
static int nrf_restart_rx_time = 0;
static int nrf_restart_tx_time = 0;

// Functions
void printf_thd(const char* format, ...);
static void print_rf_status(void);
static int rf_tx_wrapper(char *data, int len);
static THD_FUNCTION(tx_thread, arg);
static THD_FUNCTION(rx_thread, arg);
static THD_FUNCTION(packet_process_thread, arg);
static void read_mote_state(mote_state *data);
static void show_vbat(void);
static void send_buffer_nrf(unsigned char *data, unsigned int len);

static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chSysLockFromISR();
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		0,
		0,
		0,
		rxchar,
		0,
		115200,
		0,
		USART_CR2_LINEN,
		0
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
#if USE_PRINTF
	chMtxLock(&print_mutex);

	// Wait for the previous transmission to finish.
	while (UARTD1.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	static uint8_t str[200];

	va_list ap;
	MemoryStream ms;
	BaseSequentialStream *chp;

	/* Memory stream object to be used as a string writer.*/
	msObjectInit(&ms, str + 1, sizeof(str) - 1, 0);

	/* Performing the print operation using the common code.*/
	chp = (BaseSequentialStream *)&ms;
	va_start(ap, format);
	chvprintf(chp, format, ap);
	va_end(ap);

	/* Final zero and size return.*/
	chSequentialStreamPut(chp, 0);

#if USE_PRINTF == 1
	uartStartSend(&UARTD1, ms.eos - 1, str + 1);
#elif USE_PRINTF == 2
	str[0] = COMM_PRINT;
	packet_send_packet(str, ms.eos, 0);
#endif

	chMtxUnlock(&print_mutex);
#else
	(void)format;
#endif
}

static void print_rf_status(void) {
	int s = rf_status();

	printf_thd("RF Status Register\r\n");
	printf_thd("RX_DR  TX_DS  MAX_RT  RX_P_NO  TX_FULL\r\n");
	printf_thd("%i      %i      %i       %i        %i\r\n",
			NRF_STATUS_GET_RX_DR(s), NRF_STATUS_GET_TX_DS(s), NRF_STATUS_GET_MAX_RT(s),
			NRF_STATUS_GET_RX_P_NO(s), NRF_STATUS_GET_TX_FULL(s));
}

static int rf_tx_wrapper(char *data, int len) {
#if NOACK
	int res = rfhelp_send_data_crc_noack(data, len, TX_RESENDS);
#else
	int res = rfhelp_send_data_crc(data, len);
#endif

	if (res == 0) {
		nrf_stats.tx_ok++;
		nrf_restart_tx_time = NRF_RESTART_TIMEOUT;
	} else if (res == -1) {
		nrf_stats.tx_max_rt++;
	} else if (res == -2) {
		nrf_stats.tx_timeout++;
	}

	return res;
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
	chMtxUnlock(&read_mutex);
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

static void send_buffer_nrf(unsigned char *data, unsigned int len) {
	uint8_t send_buffer[MAX_PL_LEN];

	tx_disable_time = TX_DISABLE_TIME;
	rx_enable_time = RX_ENABLE_TIME;
	alive_timestamp = chVTGetSystemTime();

	if (data[0] == COMM_GET_VALUES) {
		nrf_stats.req_values++;
	}

	if (len <= (MAX_PL_LEN - 1)) {
		uint32_t ind = 0;
		send_buffer[ind++] = MOTE_PACKET_PROCESS_SHORT_BUFFER;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		rf_tx_wrapper((char*)send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		unsigned int len2 = len - (MAX_PL_LEN - 5);

		for (unsigned int i = 0;i < len2;i += (MAX_PL_LEN - 2)) {
			if (i > 255) {
				break;
			}

			end_a = i + (MAX_PL_LEN - 2);

			uint8_t send_len = (MAX_PL_LEN - 2);
			send_buffer[0] = MOTE_PACKET_FILL_RX_BUFFER;
			send_buffer[1] = i;

			if ((i + (MAX_PL_LEN - 2)) <= len2) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len2 - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			rf_tx_wrapper((char*)send_buffer, send_len + 2);
		}

		for (unsigned int i = end_a;i < len2;i += (MAX_PL_LEN - 3)) {
			uint8_t send_len = (MAX_PL_LEN - 3);
			send_buffer[0] = MOTE_PACKET_FILL_RX_BUFFER_LONG;
			send_buffer[1] = i >> 8;
			send_buffer[2] = i & 0xFF;

			if ((i + (MAX_PL_LEN - 3)) <= len2) {
				memcpy(send_buffer + 3, data + i, send_len);
			} else {
				send_len = len2 - i;
				memcpy(send_buffer + 3, data + i, send_len);
			}

			rf_tx_wrapper((char*)send_buffer, send_len + 3);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = MOTE_PACKET_PROCESS_RX_BUFFER;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);
		memcpy(send_buffer + 5, data + len2, len - len2);
		ind += len - len2;

		rf_tx_wrapper((char*)send_buffer, ind);
	}
}

static void send_packet(unsigned char *data, unsigned int len) {
	// Wait for the previous transmission to finish.
	while (UARTD1.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	uartStartSend(&UARTD1, len, data);
}

static THD_FUNCTION(packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("UART");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {
			packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], 0);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}

static THD_FUNCTION(tx_thread, arg) {
	(void)arg;

	tx_tp = chThdGetSelfX();
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

		if (tx_disable_time > 0) {
			tx_disable_time -= 20;
		} else {
			rfhelp_power_up();
			chThdSleepMilliseconds(2);
			rf_tx_wrapper((char*)pl, index);
		}

		// Turn on RX for 5 of 30 iterations
		if (rx_enable_time > 0) {
			rx_enable_time -= 20;
			rx_now = true;
		} else {
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
		}

		chThdSleepMilliseconds(20);
	}
}

static THD_FUNCTION(rx_thread, arg) {
	(void)arg;

	rx_tp = chThdGetSelfX();
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
		int p;
		int ind;

		for(;;) {
#if NOACK
			int res = rfhelp_read_rx_data_crc_noack(buf, &len, &p);
#else
			int res = rfhelp_read_rx_data_crc(buf, &len, &p);
#endif

			// If something was read
			if (res >= 0) {
				MOTE_PACKET packet = buf[0];
				alive_timestamp = chVTGetSystemTime();
				nrf_restart_rx_time = NRF_RESTART_TIMEOUT;

//				LED_RED_TOGGLE();

				switch (packet) {
				case MOTE_PACKET_BATT_LEVEL:
					// TODO!
					break;

				case MOTE_PACKET_ALIVE:
					// Everything triggers alive...
					break;

				case MOTE_PACKET_FILL_RX_BUFFER:
					memcpy(rx_buffer + buf[1], buf + 2, len - 2);
					break;

				case MOTE_PACKET_FILL_RX_BUFFER_LONG: {
					int rxbuf_ind = (unsigned int)buf[1] << 8;
					rxbuf_ind |= buf[2];
					if (rxbuf_ind < RX_BUFFER_SIZE) {
						memcpy(rx_buffer + rxbuf_ind, buf + 3, len - 3);
					}
				}
				break;

				case MOTE_PACKET_PROCESS_RX_BUFFER: {
					ind = 1;
					int rxbuf_len = (unsigned int)buf[ind++] << 8;
					rxbuf_len |= (unsigned int)buf[ind++];

					if (rxbuf_len > RX_BUFFER_SIZE) {
						break;
					}

					uint8_t crc_high = buf[ind++];
					uint8_t crc_low = buf[ind++];

					memcpy(rx_buffer + rxbuf_len - (len - ind), buf + ind, len - ind);

					if (crc16(rx_buffer, rxbuf_len)
							== ((unsigned short) crc_high << 8
									| (unsigned short) crc_low)) {

						if (rx_buffer[0] == COMM_GET_VALUES) {
							nrf_stats.res_values++;
						}

						// Wait a bit in case retries are still made
						chThdSleepMilliseconds(2);

#if USE_PRINTF == 2
						chMtxLock(&print_mutex);
#endif
						packet_send_packet(rx_buffer, rxbuf_len, 0);
#if USE_PRINTF == 2
						chMtxUnlock(&print_mutex);
#endif
					}
				}
				break;

				case MOTE_PACKET_PROCESS_SHORT_BUFFER:
					// Wait a bit in case retries are still made
					chThdSleepMilliseconds(2);

#if USE_PRINTF == 2
					chMtxLock(&print_mutex);
#endif
					packet_send_packet((unsigned char*)buf + 1, len - 1, 0);
#if USE_PRINTF == 2
					chMtxUnlock(&print_mutex);
#endif
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
				chThdSleepMilliseconds(1);
			}
		}

		chThdSleepMilliseconds(5);
	}
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
	uartStart(&UARTD1, &uart_cfg);

	// Packet interface
	packet_init(send_packet, send_buffer_nrf, 0);

	// Serial pins
	palSetPadMode(GPIOA, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(GPIOA, 10, PAL_MODE_INPUT);

	// Initialize mutexes
	chMtxObjectInit(&print_mutex);
	chMtxObjectInit(&read_mutex);

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

#if USE_HARD_ADDR
	address = HARD_ADDR;
	printf_thd("Using hardcoded address\r\n");
#else
	chThdSleepMilliseconds(5);
	address = READ_ADDR();
	printf_thd("Using address from resistor setting\r\n");
#endif

	printf_thd("Address: %d\r\n", address);

	// Remove pulldowns to save power
	palSetPadMode(ADDR0_PORT, ADDR0_PIN, PAL_MODE_INPUT);
	palSetPadMode(ADDR1_PORT, ADDR1_PIN, PAL_MODE_INPUT);
	palSetPadMode(ADDR2_PORT, ADDR2_PIN, PAL_MODE_INPUT);
	palSetPadMode(ADDR3_PORT, ADDR3_PIN, PAL_MODE_INPUT);

	// Set RF address
	const char addr[3] = {0xC6, 0xC7, address};
	rfhelp_set_rx_addr(0, addr, 3);
	rfhelp_set_tx_addr(addr, 3);

	alive_timestamp = chVTGetSystemTime();

	// Start threads
	chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa), NORMALPRIO, packet_process_thread, NULL);
	chThdCreateStatic(rx_thread_wa, sizeof(rx_thread_wa), NORMALPRIO + 1, rx_thread, NULL);
	chThdCreateStatic(tx_thread_wa, sizeof(tx_thread_wa), NORMALPRIO, tx_thread, NULL);

	LED_RED_OFF();
	LED_GREEN_OFF();

	// Restart radio
	chThdSleepMilliseconds(5);
	rfhelp_power_up();
	chThdSleepMilliseconds(1);
	rfhelp_restart();

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

			// Reset nrf and print some data
			static int print_cnt = 0;
			print_cnt++;
			if (print_cnt >= 50) {
				print_cnt = 0;

#if PRINT_MAIN
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
#endif
#if PRINT_NRF_STATS
				printf_thd(
						"Val req:  %d\r\n"
						"Val rec:  %d\r\n"
						"Val succ: %.2f %%\r\n"
						"TX OK:    %d\r\n"
						"TX M_RT:  %d\r\n"
						"TX T_O:   %d\r\n"
						"TX succ:  %.2f %%\r\n",
						nrf_stats.req_values, nrf_stats.res_values,
						((float)nrf_stats.res_values / (float)nrf_stats.req_values) * 100.0,
						nrf_stats.tx_ok, nrf_stats.tx_max_rt, nrf_stats.tx_timeout,
						((float)nrf_stats.tx_ok / (float)(nrf_stats.tx_ok + nrf_stats.tx_max_rt + nrf_stats.tx_timeout)) * 100.0);
#endif

			}

			if (nrf_restart_rx_time > 0 && nrf_restart_tx_time > 0) {
				nrf_restart_rx_time -= 10;
				nrf_restart_tx_time -= 10;
			} else {
#if PRINT_MAIN
				print_rf_status();
#endif
				rfhelp_power_up();
				rfhelp_restart();
#if PRINT_MAIN
				print_rf_status();
#endif
				nrf_restart_rx_time = NRF_RESTART_TIMEOUT;
				nrf_restart_tx_time = NRF_RESTART_TIMEOUT;
			}

			if (SHOW_VBAT_NOW()) {
				show_vbat();
			}

			chThdSleepMilliseconds(10);
			packet_timerfunc();
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
