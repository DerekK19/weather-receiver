/** @file rfm12b.c
*
* @author Peter Emmerling
*
* @date 31.01.2013
*
* @version 0.7
* basic functions like init_device_list, rfm_initialize, calc_crc, char_to_int, int_to_char,
* rf_xfer, rf_rxdata, receive_data, spi_init, waitus,
* activate_receiver, deactivate_receiver.
*
*/

#include "rfm.h"
#include "rfm_commands.h"
#include <stdio.h>
#include <stdlib.h>
#include <bcm2835.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>

#if 0
#define DEBUG_PRINTF printf
#define DEBUG_FFLUSH fflush
#else
#define DEBUG_PRINTF do {} while(0);
#define DEBUG_FFLUSH do {} while(0);
#endif

uint16_t cmd_reset;
uint16_t cmd_status;
uint16_t cmd_drate;
uint16_t cmd_freq;
uint16_t cmd_wakeup;
uint16_t cmd_afc;
uint16_t cmd_dcycle;
uint16_t cmd_fifo_c;
uint16_t cmd_fifo_e;
uint16_t cmd_config;
uint16_t cmd_rcon;
uint16_t cmd_rcon_on;
uint16_t cmd_dfilter;
uint16_t cmd_lowbatt;
#ifdef RFM12B
uint16_t cmd_power;
uint16_t cmd_power_n;
uint16_t cmd_power_y;
uint16_t cmd_sync;
uint16_t cmd_pll;
#endif

/*****************************************************************************
* The RX union is used in the FIFO tranfer from the RFM01 module. You can read
* the FIFO one byte at a time by sending a status command (0x00) to the RFM01.
* After the execution, the first two bytes of the buffer (rx.status) contains
* status information, the 3rd byte (rx.fifo) contains the FIFO data.
* See: "RFM01 Universal ISM Band FSK Receiver" pg. 17, Status Read Command
* (http://www.hoperf.com/upload/rf/RFM01.pdf)
******************************************************************************/
typedef union
{
    char buffer[3];
    struct
    {
		uint16_t status;
        uint8_t  fifo;
    };
} RX;

#define recordsMAX      1   // loop counter for bytesMAX bytes to read
#define bytesMAX        10  // bytes to read

RX			rx[recordsMAX][bytesMAX];

/**
   * create delay in microseconds
   */
void waitus (unsigned int delaytime)
{
    usleep(delaytime);
}

/**
   * enables FIFO fill of rfm12b after synchron pattern reception,\n
   * receiver activated either since initialization or previous transmission event
   */
void activate_receiver( void)
{
    DEBUG_PRINTF("\033[32mactive \033[37m\n");
    rf_xfer(cmd_fifo_e);
#ifdef RFM12B
//	rf_xfer(cmd_power_y);
    rf_xfer(0);
#endif
}

/**
   * disables FIFO fill of rfm12b, receiver keeps active
   */
void deactivate_receiver( void)
{
#ifdef RFM12B
//	rf_xfer(cmd_power_n);
#endif
    rf_xfer(cmd_fifo_c);
    DEBUG_PRINTF("\033[35minactive \033[37m\n");
}

/**
   * Initialization of RPi's SPI-Bus, bcm2835-library used,\n
   * GPIO 7 (Pin 26) used for Chip Enable and GPIO-Pin 22 (Pin 15) for nIRQ
   */
void spi_init()
{
    if (!bcm2835_init())
        exit (1);
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_chipSelect(RFM_CS);
    bcm2835_spi_setChipSelectPolarity(RFM_CS, LOW);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default ()

    //Set IRQ pin details
    bcm2835_gpio_fsel(RFM_IRQ, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(RFM_IRQ, BCM2835_GPIO_PUD_UP);
}

/*****************************************************************************
* Function:	scheduler
*
* Overview:	This function switches the RasPi niceness of processes to REALTIME
*			mode (high priority) or STANDARD mode (normal priority)
*			Function taken from Kevin Sangeelees' WH1080 project
*
******************************************************************************/
void scheduler(uint8_t mode)
{
    struct sched_param p;

    if(mode==REALTIME)
    {
        p.__sched_priority = sched_get_priority_max(SCHED_RR);
        if(sched_setscheduler(0, SCHED_RR, &p)==-1)
            printf("Failed to switch to realtime scheduler.\n");
    }
    else
    {
        p.__sched_priority = 0;
        if(sched_setscheduler(0, SCHED_OTHER, &p)==-1 )
            printf("Failed to switch to normal scheduler.\n");
    }
}

/*****************************************************************************
* Function:	rf_set_register
*
* Overview:	This function sends configuration commands to the RFM01 module
*
******************************************************************************/
void rf_set_register(uint16_t cmd)
{
    char buffer[2];
    buffer[0] = cmd >> 8;
    buffer[1] = cmd;
    bcm2835_spi_transfern(buffer,2);
}

/*****************************************************************************
* Function:	rf_xfer
*
* Overview:	This function sends configuration commands to the RFM01 module
*			and gets a reply, which it returns
*
******************************************************************************/
uint16_t rf_xfer(uint16_t cmd)
{
    char buffer[2];
    uint16_t reply;
    buffer[0] = cmd >> 8;
    buffer[1] = cmd;
	DEBUG_PRINTF("SPI %02x%02x", buffer[0], buffer[1]);
    bcm2835_spi_transfern(buffer,2);
    reply = buffer[0] << 8;
    reply |= buffer[1];
   	DEBUG_PRINTF("\t-> %04x\n", reply);

    return reply;
}

/*****************************************************************************
* Function:	rf_receive_record
*
* Overview:	This function monitors the RFM01/RFM12B interrupt signal. When an
*			interrupt is received, the FIFO register is read out and savede
*			The transmitter sends new data every 48 seconds, so after
*			processing the data this function goes into sleep mode for 40 seconds
*			(to be on the safe side) before waiting for the next interrupt.
*
******************************************************************************/
uint8_t rf_receive_record(uint8_t *rData, uint8_t mData)
{
    int8_t record, byte;

#ifdef RFM01
    const uint16_t clearfifo  = CMD_FIFO | 8<<4 | 1<<2 | 0<<1 | 0;
    const uint16_t enablefifo = CMD_FIFO | 8<<4 | 1<<2 | 1<<1 | 1;
#endif

    while(1)
    {
        scheduler(REALTIME);												// set the scheduler policy to realtime
        for(record=0; record<recordsMAX; record++)
        {
#ifdef RFM01
            rf_set_register(clearfifo);										// disable (clear) FIFO
            rf_set_register(enablefifo);									// re-enable FIFO
#endif
            for(byte=0; byte<mData; byte++)									// we only read the first byteMAX bytes
            {
                rx[record][byte].status = 0;								// set command to 0 (read status, FIFO)
                while(bcm2835_gpio_lev(RFM_IRQ));							// wait for interrupt
				bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128); // set SPI speed to 2MHz for FIFO read
#ifdef RFM01
                bcm2835_spi_transfern(rx[record][byte].buffer, 3);			// read FIFO and store in rx
#endif
#ifdef RFM12B
				while (!(rf_xfer(CMD_STATUS) & 0x8000));
				rx[record][byte].fifo=rf_xfer(CMD_FIFO_RD);
#endif
				bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);  // reset SPI speed to 8MHz
                if(rx[record][0].fifo==0) byte--;							// ignore occasional leading 0
				else if((rx[record][0].fifo>>4)!=0x0a &&
						(rx[record][0].fifo>>4)!=0x0b)						// high nibble of first byte must be 1010b or 1011b
				{
					record--;											   	// ignore if record is not weather data
					break;													// continue record loop
				}
				else rData[byte] = rx[record][byte].fifo;					// store received data in received array
            }
			if(byte==0) continue;											// this is necessary because of the previous break!
			break;//if(rx[record][9].fifo==crc8(received, 9)) break;		// if the CRC is OK, don't read any further
        }
        return mData;														// idle loop until next possible transmission
   }
}

void rf_initialize(uint16_t band, uint16_t freq, uint16_t rate, uint16_t bw, uint16_t lna, uint16_t rssi)
{
	//CMD_DRATE: Expected bit rate:
	//     13 = 17241
	//     95 = 1959
	//     99 = 1700
	//     9c = 1500
	//     a1 = 1268
	//     aa = 1000
	//     b8 = 756
	//     d5 = 500
	
#ifdef RFM01
	cmd_reset   = CMD_RESET;
	cmd_status  = CMD_STATUS;
	cmd_drate   = CMD_DRATE|rate;
	cmd_freq    = CMD_FREQ|freq;
	cmd_wakeup  = CMD_WAKEUP|1<<8|0x05;
	cmd_afc     = CMD_AFC|AFC_ON|AFC_OUT_ON|AFC_VDI|AFC_FINE|AFC_RL_15|AFC_STROBE;
	cmd_dcycle  = CMD_LOWDUTY|0x0E;
	cmd_fifo_c  = CMD_FIFO|RFM01_FIFO_8_SYNC_STOP_CLEAR;
	cmd_fifo_e  = CMD_FIFO|RFM01_FIFO_8_SYNC_FILL_ENABL;
	cmd_config  = CMD_CONFIG|band|LOWBATT_EN|CRYSTAL_EN|LOAD_CAP_12C5|bw;
	cmd_rcon    = CMD_RCON|VDI_CR_LOCK|VDI_DRSSI|lna|rssi;
	cmd_rcon_on = cmd_rcon|RX_EN;
	cmd_dfilter = CMD_DFILTER|CR_LOCK_FAST|FILTER_DIGITAL|DQD_2;
	cmd_lowbatt = CMD_LOWBATT|0x06;
#endif

#ifdef RFM12B
	cmd_reset   = CMD_RESET;
	cmd_status  = CMD_STATUS;
	cmd_drate   = CMD_DRATE|rate;
	cmd_config  = CMD_CONFIG|CONFIG_EF|band|LOAD_CAP_12C5;
	cmd_freq    = CMD_FREQ|freq;
	cmd_wakeup  = CMD_WAKEUP|0x00;
	cmd_afc     = CMD_AFC|0x83;
	cmd_dcycle  = CMD_LOWDUTY;
	cmd_fifo_c  = CMD_FIFO|RFM01_FIFO_8_VDI_STOP_ENABL;
	cmd_fifo_e  = CMD_FIFO|RFM01_FIFO_8_VDI_FILL_ENABL;
	cmd_rcon    = CMD_RCON|P16|VDI_FAST|lna|rssi|bw;
	cmd_rcon_on = cmd_rcon;
	cmd_dfilter = CMD_DFILTER|0x84;
	cmd_lowbatt = CMD_LOWBATT|0xE0;
	cmd_power   = CMD_PM|0xD9;
	cmd_power_n = CMD_PM|0x09;	//B2.3 !er , !ebb , !Et , !Es , Ex , !eb , !ew , dc
	cmd_power_y = CMD_PM|0xc9;  //B2.3 er, ebb, !et, !es, ex, !eb, !ew, dc
	cmd_sync    = CMD_SYNCRON|0xD4;
	cmd_pll     = CMD_PLL|0x77;
#endif

#if 0
    rf_xfer(0x80d7); //B2.2 el , ef , 433band, 12.5pf (x1-x4 ggf. anpassen bei externem Quarz)
    rf_xfer(0x82d9); //B2.3 er , ebb , !Et , Es , Ex , !eb , !ew , dc
    rf_xfer(0xa620); //DK//B2.4 434,15 MHz (*2)
    rf_xfer(0xc613); //DK//B2.5 4.8kbps
    rf_xfer(0x94a4); //B2.6 Vdi Fast , 134 kHz Bandbreite, 0db Abschwächung , -79dbm Schwellwert
    rf_xfer(0xc2ac); //B2.7 Al , !ml , Dig , Dqd4
    rf_xfer(0xca81); //B2.8 Fifo8 , Sync , !ff , Dr
    //Synchron pattern Command:
    rf_xfer(0xced4); // SYNC=2DD4ï¼.
    rf_xfer(0xc483); //B2.10 Keep Offset , No Rstric , !st , !fi , Oe , En
    rf_xfer(0x9850); //B2.11 90 kHz , power - 0 dB
    //PLL Setting Command:
    rf_xfer(0xcc77); // OB1,OB0,LPX,!ddy,DDIT,BW0
    rf_xfer(0xe000); //B2.13 no wakeup
    rf_xfer(0xc800); //B2.14 !en
    rf_xfer(0xc0E0); //B2.15 10 MHz , 2.2V Schwellwert Low Battery Detector
#else
	rf_xfer(cmd_status);
	rf_xfer(cmd_config);
	rf_xfer(cmd_freq);
	rf_xfer(cmd_wakeup);
	rf_xfer(cmd_dcycle);
	rf_xfer(cmd_afc);
	rf_xfer(cmd_dfilter);
	rf_xfer(cmd_drate);
	rf_xfer(cmd_lowbatt);
	rf_xfer(cmd_rcon);
	rf_xfer(cmd_fifo_c);
#ifdef RFM01
	rf_xfer(cmd_fifo_e);
	rf_xfer(cmd_rcon_on);
#endif
#ifdef RFM12B
	rf_xfer(cmd_power);
	rf_xfer(cmd_sync);
	rf_xfer(cmd_pll);
#endif
#endif
	usleep(5000);	// Allow crystal oscillator to start

	// Don't know if we should do this again...	
//	rf_xfer(cmd_config);
//	rf_xfer(cmd_rcon_on);
//	usleep(1000);
}

/**
   * \param number number data bytes to receive
   * \return 0 if data received, 1 if timed out
   * 
   * receiving data bytes
   */
uint16_t rf_rxdata(unsigned char* rData, unsigned char mData)
{
	uint8_t nData;

    activate_receiver();

	// Wait for the RFM_IRQ pin to go low.
	while(bcm2835_gpio_lev(RFM_IRQ) == HIGH);

	nData = rf_receive_record(rData, mData);

	DEBUG_PRINTF("\t-> %d\n", nData);

    deactivate_receiver();

    return nData;
}
