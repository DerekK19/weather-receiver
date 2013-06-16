 /** @file rfm12b.h
 *   
 * @author Peter Emmerling 
 * 
 * @date 31.01.2013 
 * 
 * @version 0.7 
 * basic defines, data structures,and function prototypes
 * used from both, transmitter and receiver
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#ifndef _STDBOOL_H
#define _STDBOOL_H
#define bool _Bool //Bool als Datentyp hinzufügen
#define true 1
#define false 0
#define __bool_true_false_are_defined 1

#endif

#define TIMEOUT_PACKET 10000 		// Timeout for waiting for a packet to arrive (* .001 secs)

#define RFM_IRQ RPI_GPIO_P1_15		// IRQ pin. Corresponds to GPIO22 - Pin 15
#define RFM_SEL RPI_GPIO_P1_24  	// SPI chip select. Corresponds to CE0 - Pin 24
#define RFM_SDI RPI_GPIO_P1_19		// SDI pin. Corresponds to GPIO MOSI - Pin 19
#define RFM_SDO RPI_GPIO_P1_21		// SDO pin. Corresponds to GPIO MISO - Pin 21
#define RFM_SCK RPI_GPIO_P1_23		// SCK clock pin, Corresponds to GPIO SCLK - Pin 23
#define RFM_CS BCM2835_SPI_CS0  	// SPI chip select pin

#define REALTIME        1
#define STANDARD        0

//! Initialise rfm module
void rf_initialize(uint16_t band, uint16_t freq, uint16_t rate, uint16_t bw, uint16_t lna, uint16_t rssi);
//! Low level function for communication with the rfm12b via SPI of the RPi
uint16_t rf_xfer (uint16_t cmd);
//! Low level function for communication with the rfm12b via SPI of the RPi
void rf_set_register(uint16_t cmd);
//! Receiving 16 data bytes with or without timeout functionality
uint16_t rf_rxdata(unsigned char*, unsigned char);
//! Initialize SPI of the Raspberry Pi using bcm2835-library
void spi_init (void);
//! Wait microseconds using the usleep-function
void waitus (unsigned int);
//! Control FIFO (receiver) of rfm12b
void activate_receiver( void);
//! Control FIFO (receiver) of rfm12b
void deactivate_receiver( void);

void scheduler(uint8_t mode);
