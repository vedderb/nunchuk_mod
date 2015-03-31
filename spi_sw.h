/*
 * spi_sw.h
 *
 *  Created on: 10 mar 2015
 *      Author: benjamin
 */

#ifndef SPI_SW_H_
#define SPI_SW_H_

// Pin map
#define PORT_CE		GPIOA
#define PIN_CE		3
#define PORT_CSN	GPIOA
#define PIN_CSN		4
#define PORT_SCK	GPIOA
#define PIN_SCK		5
#define PORT_MOSI	GPIOA
#define PIN_MOSI	6
#define PORT_MISO	GPIOA
#define PIN_MISO	7

// Functions
void spi_sw_init(void);
void spi_sw_transfer(char *in_buf, const char *out_buf, int length);
void spi_sw_begin(void);
void spi_sw_end(void);
void spi_sw_write_ce(int state);

#endif /* SPI_SW_H_ */
