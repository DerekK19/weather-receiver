/*
* Maplin N96GY (Fine Offset WH1080/WH1081) RF receiver using a
* Raspberry Pi and an  or RFM12b transceiver module. I switched
* to an RFM01 module after frying the RFM12b; turns out it works *far*
* better anyway, so it was something of a blessing in disguise.
* 
* The code here is really just experimental, and is not intended to be used
* beyond a learning excercise. It conveys the basics of what's required
* to get the Raspberry Pi receiving sensor data, but that's about it!
*
* I can't be sure it still works with an RFM12b, but it shouldn't be far off
* the mark if not - a bit of debugging may be required, but I no longer
* have a working module to test.
*
* This program configures an RFM01 to receive RF transmissions from the
* weather station's sensors, and reads them directly from the receiver's
* demodulator via the DATA pin, in to a GPIO pin on the Raspberry Pi. The
* pulse widths are used to derive the data-packet that was transmitted.
*
* The process switches to SCHED_RR for realtime latency while it waits
* for a packet. It returns to SCHED_OTHER when a packet is received.
* This ensures that bit transitions aren't missed, and also allows very
* heavy loads to run on the Pi while maintaining reliable reads. Optionally,
* the command 'sysctl kernel.sched_wakeup_granularity_ns=100000' may
* further improve latency, though it seems to work with Raspbian defaults
* regardless.
*
* Includes Luc Small's version of CRC8 from the OneWire Arduino library
* adapted for Fine Offset's calculations that also happen to work for this
* weather station. The SPI code was derived from the driver example at 
* kernel.org.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation as version 2 of the License.
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <rest/rest-proxy.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>

#include "ws1093_rf.h"
#include "bcm2835.h"
#include "rfm.h"
#include "rfm_commands.h"

// The time to expect the whole packet to arrive in (initially this was set to 5000)
// Since the ARM time is running at 1Mhz, this is millionths of a second (microseconds)
#define TIMEOUT_ARM_COUNT 50000

#define BUFFER_SIZE 10

struct weather_s
{
	float temperature;			// -127 - +127 degrees centigrade
	uint8_t humidity;			// 0 - 100 percent
	uint8_t wind_quadrant;		// 0=north, 1=north north east, ... 15=north north west
	float wind_speed;			// metres per second
	float gust_speed;			// metres per second
	float rain;					// millimetres since the last reading
};
typedef struct weather_s weather_t;

unsigned char buffer[BUFFER_SIZE];

unsigned int f;
unsigned int band;
unsigned int lna;
unsigned int bw;
unsigned int rssi;
unsigned int rate;
unsigned int sample;
unsigned int rev;
unsigned int fsk;

uint16_t arg_band;
uint16_t arg_freq;
uint16_t arg_lna;
uint16_t arg_bw;
uint16_t arg_rssi;
uint16_t arg_rate;
uint16_t arg_rev;
uint16_t arg_fsk;

float rain_at_start_of_day = -1;
float last_rain = 0.0;
int CRC_fails = 0;

#define LED_RED RPI_V2_GPIO_P1_11
#define LED_AMBER RPI_V2_GPIO_P1_12
#define LED_GREEN RPI_V2_GPIO_P1_13

char *direction_name[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

extern int read_bmp085(float altitude);

/*************** Forward references ***************/

void led_off(uint8_t pin);
void led_on(uint8_t pin);
void get_args(int argc, char *argv[]);
bool calculate_values(unsigned char *buf, weather_t *weather);
bool save_values(weather_t *weather);
void dump_buffer(unsigned char *buffer);
void log_current(char *format, ...);
uint8_t _crc8( uint8_t *addr, uint8_t len);
int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y);

/*************** Main entry point ***************/

int main(int argc, char *argv[])
{
	if(map_peripheral(&gpio) == -1 || map_peripheral(&timer_arm) == -1) {
		printf("Failed to map the GPIO or TIMER registers into the virtual memory space.\n");
		return -1;
	}

#if 0
	{
		//Unit test for date interpreter
		uint8_t test_crc;
		unsigned char sample_bytes1[] = {0xa1,0x82,0x0a,0x59,0x03,0x06,0x00,0x4e,0x06,0xc8};
		calculate_values(sample_bytes1);
		unsigned char sample_bytes2[] = {0xa4,0xf0,0x27,0x47,0x00,0x00,0x03,0xc6,0x0c,0xfe};
		calculate_values(sample_bytes2);
		test_crc = _crc8(sample_bytes2, 9);
		if (sample_bytes2[9] != test_crc2)
			printf("test CRC failed (%02x != %02x)", sample_bytes2[9], test_crc2);
		unsigned char sample_bytes3[] = {0xa1,0x31,0xd1,0x59,0x00,0x00,0x09,0xdd,0x02,0x68};
		calculate_values(sample_bytes3);
		test_crc = _crc8(sample_bytes3, 9);
		if (sample_bytes3[9] != test_crc)
			printf("test CRC failed (%02x != %02x)\n", sample_bytes3[9], test_crc);
		unsigned char sample_bytes4[] = {0xa4,0xd0,0x64,0x63,0x00,0x00,0x01,0x16,0x0e,0xe5};
		calculate_values(sample_bytes4);
		test_crc = _crc8(sample_bytes4, 9);
		if (sample_bytes4[9] != test_crc)
			printf("test CRC failed (%02x != %02x)\n", sample_bytes4[9], test_crc);
	}
#endif
#if 0
	{
		printf("blink led\n");
	    	spi_init();
 	   	bcm2835_gpio_fsel(LED_RED, BCM2835_GPIO_FSEL_OUTP);
  	  	bcm2835_gpio_fsel(LED_AMBER, BCM2835_GPIO_FSEL_OUTP);
   	 	bcm2835_gpio_fsel(LED_GREEN, BCM2835_GPIO_FSEL_OUTP);
		int i;
		for (i=0; i < 20; i++)
		{
			printf("blink red\n");
			led_on(LED_RED);
			sleep(1);
			led_off(LED_RED);
			sleep(1);
			printf("blink amber\n");
			led_on(LED_AMBER);
			sleep(1);
			led_off(LED_AMBER);
			sleep(1);
			printf("blink green\n");
			led_on(LED_GREEN);
			sleep(1);
			led_off(LED_GREEN);
			sleep(1);
		}
		return -1;
	}
#endif
	
	get_args(argc, argv);

	printf("pi: %d - frequency: %d - bw: %d - rssi: %d - lna: %d - rate: %d\n",rev,f,bw,rssi,lna,rate);

	// 0xF90200; // run at 1MHz
	TIMER_ARM_CONTROL = TIMER_ARM_C_DISABLE |
						TIMER_ARM_C_FREE_EN |
						TIMER_ARM_C_16BIT |
						TIMER_ARM_C_PS1 |
						TIMER_ARM_C_FPS(0xf9);
	
	uint16_t station_code = 0l;
    struct timeval current_time;
    struct timeval previous_time;
    struct timeval delta_time;
	time_t time_now;
	struct tm *tm_now;
	char timestamp[30]; // The timestamp is actually 20 characters
	weather_t weather;
	unsigned int i;
    
    previous_time.tv_sec = 0;
	
	printf("\nCtrl+C to exit\n\n");

	// Switch to realtime scheduler
	scheduler(REALTIME);
	uint16_t count;
	do
	{
    	    spi_init();
    	    bcm2835_gpio_fsel(LED_RED, BCM2835_GPIO_FSEL_OUTP);
    	    bcm2835_gpio_fsel(LED_AMBER, BCM2835_GPIO_FSEL_OUTP);
    	    bcm2835_gpio_fsel(LED_GREEN, BCM2835_GPIO_FSEL_OUTP);
	    led_off(LED_RED);
	    led_off(LED_AMBER);
	    led_off(LED_GREEN);
	    rf_initialize(arg_band, arg_freq, arg_rate, arg_bw, arg_lna, arg_rssi);

		for (i = 0; i < BUFFER_SIZE; i++) buffer[i] = '\0';
		count = rf_rxdata(buffer, BUFFER_SIZE);
		if (count > 0)
		{

			// Weather station packets always start with a 0xa
			// The Fine Offset device can have a 0x0b packet,
			// but the WS1093 doesn't seem to use that
			if ((buffer[0] & 0xf0) != 0xa0) continue;

#if 1
			dump_buffer(buffer);
#endif
			// Does the packet have a valid CRC?
			uint8_t packet_crc = _crc8(buffer, 9);
			if (buffer[9] != packet_crc)
			{
				led_on(LED_AMBER);
				printf("\033[31mCRC failed (%02x != %02x)\033[37m\n", buffer[9], packet_crc);
				sleep(2);
				led_off(LED_AMBER);
				sleep(8); // Give the receiver time to re-cycle
				if (CRC_fails++ > 5) return 1; // We are getting too many errors
				continue;
			}
			
			// Get the current time, and record how long it was since we last had a reading
			gettimeofday(&current_time, NULL);			
			if (previous_time.tv_sec == 0) {previous_time = current_time;}
			timeval_subtract(&delta_time, &current_time, &previous_time);
			previous_time = current_time;
			time_now = time(0);
			tm_now = localtime(&time_now);
			strftime(timestamp, 30, "%Y/%m/%d %H:%M:%S", tm_now);
			printf ("\033[33m\tTime %s\t\033[37m\033[33mDelta %ld.%02ld\033[37m\n", timestamp, delta_time.tv_sec, delta_time.tv_usec);

			// Is this a packet from the weather station? It will start with the 12-bit station code
			// So if the first 12 bits are not our weather station id, wait for another packet
			if (station_code != 0 &&
				((station_code >> 8) != buffer[0] || (station_code & 0x00f0) != (buffer[1] & 0xf0)))
				continue;
			
			led_on(LED_GREEN);

			// If we haven't saved a station id yet, do it now
			if (station_code == 0)
			{
				station_code = buffer[0] << 8 | (buffer[1] & 0xf0);
				printf("Station Id: %04X\n", station_code >> 4);
			}
	
			// Reset the daily rain counter if we have past midnight
			if (rain_at_start_of_day > 0 && tm_now->tm_hour == 0) rain_at_start_of_day = -1; //DK. NOPE, need a new flag here...

			// at this point, we can do other stuff that requires the RT scheduler
			
			#ifdef USE_BMP085
			read_bmp085(ALTITUDE_M);	// read pressure, calculate for the given altitude
			#endif
			
			// Decode a weather packet
			if ((buffer[0] & 0xf0) == 0xa0)
			{
				if (!calculate_values(buffer, &weather))
				{
					led_on(LED_RED);
					printf("\033[31mUnlikely packet!\033[37m\n");
					sleep(2);
					led_off(LED_RED);
					return 2;
				}
				CRC_fails = 0;
				save_values(&weather);
				printf("\n");
			}

			led_off(LED_GREEN);
			
			scheduler(STANDARD);

			sleep(35); // sleep 35 seconds, since weather station signals only come in every 48 seconds

			scheduler(REALTIME);
		} // Handle a packet
		
	} while (1); // Ctrl+C to exit for now...

	unmap_peripheral(&gpio);
	unmap_peripheral(&timer_arm);
	
	return 0;
}

/*************** Private functions ***************/

void led_off(uint8_t pin)
{
	bcm2835_gpio_clr(pin);
}

void led_on(uint8_t pin)
{
	bcm2835_gpio_set(pin);
}

/*****************************************************************************
* Function:   	get_args
*
* Overview:   	This function processes possible command line parameters
* Input:
* Output:
*
******************************************************************************/
void get_args(int argc, char *argv[])
{
    int opt;

    // set default values
    f = 433;
    lna = 6;
    bw = 200;
    rssi = 85;
    rev = 2;
    rate = 17241;
    sample = 0;
    fsk = 1;

	// process all passed options
    while ((opt = getopt(argc, argv, "f:l:b:r:v:d:x:s")) != -1) {
        switch (opt) {
        case 'f':
        	band = atoi(optarg);
            break;
        case 'l':
        	lna = atoi(optarg);
            break;
        case 'b':
        	bw = atoi(optarg);
            break;
        case 'r':
        	rssi = atoi(optarg);
	    	break;
        case 'v':
        	rev = atoi(optarg);
            break;
        case 'd':
        	rate = atoi(optarg);
            break;
		case 's':
			sample = 1;
	    	break;
        case 'h':
        default:
            printf("Usage: wh1080_rf [OPTIONS]\n");
            printf("  -f   Frequency\n");
            printf("       315  \n");
            printf("       433  (default)\n");
            printf("       868  \n");
            printf("       915  \n");
            printf("  -l  low noice amplifier\n");
            printf("       0  \n");
            printf("       6  (default)\n");
            printf("       14  \n");
            printf("       20  \n");
            printf("  -b  band width\n");
            printf("       67  \n");
            printf("       134  (default)\n");
            printf("       200  \n");
            printf("       270  \n");
            printf("       340  \n");
            printf("       400  \n");
            printf("  -r  Received signal strength indication\n");
            printf("       73  \n");
            printf("       79  (default)\n");
            printf("       85  \n");
            printf("       91  \n");
            printf("       97  \n");
            printf("       103  \n");
            printf("  -d  Expected data bit rate\n");
            printf("  -v  Raspberry PI revision\n");
            printf("       1  (default)\n");
            printf("       2  \n");
	    printf("  -s  RSSI Sample Mode\n");
	    printf("  -h  Display this information\n");
            exit(1);
        }
    }

	switch (f) {
		case 315:
			arg_freq = 0x0620;
			arg_band = BAND_315;
			break;
		case 433:
			arg_freq = 0x0620 ;
			arg_band = BAND_433;
			break;
		case 868:
			arg_freq = 0x067c;
			arg_band = BAND_868;
			break;
		case 915:
			arg_freq = 0x07d0;
			arg_band = BAND_915;
			break;
	}

	switch (lna) {
		case 0:
#ifdef RFM01
			arg_lna = LNA_0;
#endif
#ifdef RFM12B
			arg_lna = LNA_MAX;
#endif
			break;
		case 6:
#ifdef RFM01
			arg_lna = LNA_6;
#endif
#ifdef RFM12B
			arg_lna = LNA_HIGH;
#endif
			break;
		case 14:
#ifdef RFM01
			arg_lna = LNA_14;
#endif
#ifdef RFM12B
			arg_lna = LNA_MEDIUM;
#endif
			break;
		case 20:
#ifdef RFM01
			arg_lna = LNA_20;
#endif
#ifdef RFM12B
			arg_lna = LNA_LOW;
#endif
			break;
	}

	switch (bw) {
		case 67:
			arg_bw = BW_67;
			break;
		case 134:
			arg_bw = BW_134;
			break;
		case 200:
			arg_bw = BW_200;
			break;
		case 270:
			arg_bw = BW_270;
			break;
		case 340:
			arg_bw = BW_340;
			break;
		case 400:
			arg_bw = BW_400;
			break;
	}

	switch (rssi) {
		case 73:
			arg_rssi = RSSI_73;
			break;
		case 79:
			arg_rssi = RSSI_79;
			break;
		case 85:
			arg_rssi = RSSI_85;
			break;
		case 91:
			arg_rssi = RSSI_91;
			break;
		case 97:
			arg_rssi = RSSI_97;
			break;
		case 103:
			arg_rssi = RSSI_103;
			break;
	}
	switch (rate) {
		case 17241:
			arg_rate = 0x13;
			break;
		case 1959:
			arg_rate = 0x95;
			break;
		case 1700:
			arg_rate = 0x99;
			break;
		case 1500:
			arg_rate = 0x9c;
			break;
		case 1268:
			arg_rate = 0xa1;
			break;
		case 1000:
			arg_rate = 0xaa;
			break;
		case 756:
			arg_rate = 0xb8;
			break;
		case 500:
			arg_rate = 0xd5;
			break;
		default:
			arg_rate = 10000000.0/29.0/((float)rate+1.0);
			break;
	}
	switch (rev) {
		case 1:
			arg_rev = 1;
			break;
		case 2:
			arg_rev = 2;
			break;
	}
	switch (fsk)
	{
		case 0:
			arg_fsk = 0;
			break;
		case 1:
			arg_fsk = 1;
			break;
	}
}

bool calculate_values(unsigned char *buf, weather_t *weather)
{
	
	unsigned short device_id = ((unsigned short)buf[0] << 4) | (buf[1] >> 4);
	unsigned short temp_raw = (((unsigned short)buf[1] & 0x0f) << 8) | buf[2];
	int humidity = buf[3];
	unsigned short wind_avg_raw = (unsigned short)buf[4];
	unsigned short wind_gust_raw = (unsigned short)buf[5];
	unsigned short rain_raw = (((unsigned short)buf[6] & 0x0f) << 8) | buf[7];
	int direction = buf[8] & 0x0f;
					
	float temperature_c = (float)temp_raw / 10;
	float wind_avg_ms = roundf((float)wind_avg_raw * 34.0f) / 100;
	float wind_avg_mph = wind_avg_ms * 2.23693629f;
	float wind_avg_knot = wind_avg_ms * 1.943844;
	float wind_gust_ms = roundf((float)wind_gust_raw * 34.0f) / 100;
	float wind_gust_mph = wind_gust_ms * 2.23693629f;
	float wind_gust_knot = wind_gust_ms * 1.943844;
	float rain_mm = (float)rain_raw * 0.3f;
	char *direction_str = direction_name[direction];

	if (rain_at_start_of_day < 0) {printf("Rain Reset\n"); rain_at_start_of_day = rain_mm; last_rain = rain_raw;}

	float this_rain = (rain_raw - last_rain) * 0.3f;
	last_rain = rain_raw;

	log_current("Station Id: %04X\n"
	"Temperature: %0.1fc, Humidity: %d%%\n"
	"Wind speed: %0.2f m/s, Gust Speed %0.2f m/s, %s\n"
	"Wind speed: %0.1f mph, Gust Speed %0.1f mph, %s\n"
	"Wind speed: %0.1f knots, Gust Speed %0.1f knots, %s\n"
	"Rain: %0.1f mm\n",
	 device_id,
	 temperature_c, humidity,
	 wind_avg_ms, wind_gust_ms, direction_str,
	 wind_avg_mph, wind_gust_mph, direction_str,
	 wind_avg_knot, wind_gust_knot, direction_str,
	 this_rain);
	
	printf("Station Id: %04X\n", device_id);
	printf("Temperature: \033[32m%0.1fc\033[37m, Humidity: \033[32m%d%%\033[37m\n", temperature_c, humidity);
	printf("Wind speed: \033[32m%0.2f m/s\033[37m, Gust Speed \033[32m%0.2f m/s\033[37m, \033[32m%s\033[37m\n", wind_avg_ms, wind_gust_ms, direction_str);
	printf("Wind speed: %0.1f mph, Gust Speed %0.1f mph, %s\n", wind_avg_mph, wind_gust_mph, direction_str);
	printf("Wind speed: %0.1f knots, Gust Speed %0.1f knots, %s\n", wind_avg_knot, wind_gust_knot, direction_str);
	printf("Rain: \033[32m%0.1f mm\033[37m\n", this_rain);
	
	// Do some checking, since we can get unreasonable values even with a good CRC
	if (this_rain > 10.0 ||					// 10 mm of rain in 48 seconds is very unlikely
		humidity < 20.0 ||					// Humidity is never this low (26% is low enough)
		humidity > 100.0 ||					// And humidity is never > 100%
		temperature_c > 40.0 ||				// Temperatures here are between 2 and 40 degrees c
		temperature_c < 2) return false;
	
	weather->temperature = temperature_c;
	weather->humidity = humidity;
	weather->wind_quadrant = direction;
	weather->wind_speed = wind_avg_ms;
	weather->gust_speed = wind_gust_ms;
	weather->rain = this_rain;
	
	return true;
}

bool save_values(weather_t *weather)
{
	const char *url = "http://192.168.0.9/weather/service";
	char string_temperature[10];
	char string_humidity[10];
	char string_direction[10];
	char string_speed[10];
	char string_gust[10];
	char string_rain[10];

	RestProxy *proxy;
	RestProxyCall *call;
	GError *error = NULL;
	
	FILE *fp = fopen("/tmp/current.json","w");
	fprintf(fp, "{\"Temperature\":\"%2.2f\",\"Humidity\":\"%2.2f\",\"Rainfall\":\"%2.2f\",\"Wind\":\"%2.2f\",\"Gust\":\"%2.2f\",\"Quadrant\":\"%1d\"}",
				 weather->temperature,
				 (float)weather->humidity,
				 weather->rain,
				 weather->wind_speed,
				 weather->gust_speed,
				 weather->wind_quadrant);
	fclose(fp);
	
	sprintf(string_temperature, "%1.1f", weather->temperature);
	sprintf(string_humidity, "%1d", weather->humidity);
	sprintf(string_direction, "%1d", weather->wind_quadrant);
	sprintf(string_speed, "%1.2f", weather->wind_speed);
	sprintf(string_gust, "%1.2f", weather->gust_speed);
	sprintf(string_rain, "%1.1f", weather->rain);

	g_type_init ();

	proxy = rest_proxy_new (url, FALSE);
	call = rest_proxy_new_call (proxy);
	rest_proxy_call_set_function (call, "data/add");
	rest_proxy_call_set_method (call, "GET");
	rest_proxy_call_add_params (call,
								"sensor", "1",
							  	"temperature", string_temperature,
								"humidity", string_humidity,
								"direction", string_direction,
								"speed", string_speed,
								"gust", string_gust,
								"rain", string_rain,
							  NULL);
							  
	bool status = rest_proxy_call_run (call, NULL, &error);
	
	g_object_unref (call);
	g_object_unref (proxy);

	if (!status)
  	{
		g_error ("Cannot make call: %s", error->message);
		return false;
	}
	
	printf("Sent\n");

	return true;
}

void log_current(char *format, ...)
{
	va_list arg_list;

	va_start(arg_list, format);

	FILE *fp = fopen("/tmp/current.txt","w");
	time_t time_now = time(0);
	struct tm *tm_now = localtime(&time_now);
	char timestamp[30];
	strftime(timestamp, 30, "%d %b %Y %H:%M:%S", tm_now);
	fprintf(fp, "%s\n", timestamp);
	vfprintf(fp, format, arg_list);
	fclose(fp);

	va_end(arg_list);
}

void dump_buffer(unsigned char *buffer)
{
	unsigned char nibbles[BUFFER_SIZE*2];
	unsigned int j = 0;
	unsigned int i;

	for (i = 0; i < BUFFER_SIZE; i++)
	{
		nibbles[j++] = (buffer[i] & 0xf0) >> 4;
		nibbles[j++] = buffer[i] & 0x0f;

	}
	// We are only interested in data and time packets
	// TODO Report incorrect packets, they might be important
	//if (nibbles[0] != 0x0a && nibbles[0] != 0x0b) continue;
	int count = 0;			
	printf("\033[34m");
	for (i = 0; i < BUFFER_SIZE*2; i++)
	{
		printf("%1x", nibbles[i]);
		if (i == 19) printf("\033[37m");
		if (nibbles[0] == 0x0a &&
			(i == 0 ||
			 i == 2 ||
			 i == 5 ||
			 i == 7 ||
			 i == 9 ||
			 i == 11 ||
			 i == 12 ||
			 i == 15 ||
			 i == 16 ||
			 i == 17 )) printf("-");
		if (nibbles[0] == 0x0b &&
			(i == 0 ||
			 i == 2 ||
			 i == 3 ||
			 i == 5 ||
			 i == 7 ||
			 i == 9 ||
			 i == 11 ||
			 i == 13 ||
			 i == 15 ||
			 i == 17 ||
			 i == 19)) printf("-");
		count++;
	}
	printf(" (%d)\n", count);
}

/*
* Function taken from Luc Small (http://lucsmall.com), itself
* derived from the OneWire Arduino library. Modifications to
* the polynomial according to Fine Offset's CRC8 calulations.
*/
uint8_t _crc8( uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	// Indicated changes are from reference CRC-8 function in OneWire library
	while (len--) {
		uint8_t inbyte = *addr++;
		uint8_t i;
		for (i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
			crc <<= 1; // changed from right shift
			if (mix) crc ^= 0x31;// changed from 0x8C;
			inbyte <<= 1; // changed from right shift
		}
	}
	return crc;
}

int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y)
{
   /* Perform the carry for the later subtraction by updating y. */
	if (x->tv_usec < y->tv_usec) {
		int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
		y->tv_usec -= 1000000 * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > 1000000) {
		int nsec = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * nsec;
		y->tv_sec -= nsec;
	}

	/* Compute the time remaining to wait.
	tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
 }
