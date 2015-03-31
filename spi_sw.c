/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "spi_sw.h"
#include "ch.h"
#include "hal.h"
#include <stdbool.h>

// Private functions
static void spi_sw_delay(void);

// Private variables
static bool init_done = false;

void spi_sw_init(void) {
	if (!init_done) {
		palSetPadMode(PORT_MISO, PIN_MISO, PAL_MODE_INPUT);
		palSetPadMode(PORT_CE, PIN_CE, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(PORT_CSN, PIN_CSN, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(PORT_SCK, PIN_SCK, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(PORT_MOSI, PIN_MOSI, PAL_MODE_OUTPUT_PUSHPULL);

		palSetPad(PORT_CSN, PIN_CSN);
		palClearPad(PORT_SCK, PIN_SCK);
		init_done = true;
	}
}

void spi_sw_transfer(char *in_buf, const char *out_buf, int length) {
	palClearPad(PORT_SCK, PIN_SCK);

	for (int i = 0;i < length;i++) {
		unsigned char send = out_buf ? out_buf[i] : 0;
		unsigned char recieve = 0;

		for (int bit=0;bit < 8;bit++) {
			palWritePad(PORT_MOSI, PIN_MOSI, send >> 7);
			send <<= 1;

			spi_sw_delay();

			recieve <<= 1;
			if (palReadPad(PORT_MISO, PIN_MISO)) {
				recieve |= 0x1;
			}

			palSetPad(PORT_SCK, PIN_SCK);
			spi_sw_delay();
			palClearPad(PORT_SCK, PIN_SCK);
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

void spi_sw_begin(void) {
	palClearPad(PORT_CSN, PIN_CSN);
}

void spi_sw_end(void) {
	palSetPad(PORT_CSN, PIN_CSN);
}

void spi_sw_write_ce(int state) {
	palWritePad(PORT_CE, PIN_CE, state);
}

static void spi_sw_delay(void) {
	__NOP();
	// Possibly more NOPs on a faster MCU
}
