#ifndef _NMEA_H_
#define _NMEA_H_

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define NMEA_GPRMC_STR 		"$GPRMC"
#define NMEA_GNRMC_STR 		"$GNRMC"
#define NMEA_GPGGA_STR 		"$GPGGA"
#define NMEA_GNGGA_STR 		"$GNGGA"

#define NMEA_UNKNOWN 		0x00
#define NMEA_GPRMC 		0x01
#define NMEA_GNRMC		0x02
#define NMEA_GPGGA 		0x03
#define NMEA_GNGGA		0x04

#define NMEA_GxGGA		0x05
#define NMEA_GxRMC		0x06

#define _EMPTY 			0x00
#define _COMPLETED 		0x03

#define NMEA_CHECKSUM_ERR 	0x80
#define NMEA_MESSAGE_ERR 	0xC0

struct gx_gga {
    // Latitude eg: 4124.8963 (XXYY.ZZKK.. DEG, MIN, SEC.SS)
    double latitude;
    // Latitude eg: N
    char lat;
    // Longitude eg: 08151.6838 (XXXYY.ZZKK.. DEG, MIN, SEC.SS)
    double longitude;
    // Longitude eg: W
    char lon;
    // Quality 0, 1, 2
    uint8_t quality;
    // Number of satellites: 1,2,3,4,5...
    uint8_t satellites;
    // Altitude eg: 280.2 (Meters above mean sea level)
    double altitude;
};
typedef struct gx_gga gx_gga_t;

struct gx_rmc { 
	double latitude; 
	char lat; 
	double longitude; 
	char lon; 
	double speed; 
	double course;
};
typedef struct gx_rmc gx_rmc_t;


struct location {
	double latitude;
	double longitude;
	double speed;
	double altitude; 
	double course;
};

typedef struct location loc_t;

// Get the actual location
void gps_location(loc_t *);

// convert deg to decimal deg latitude, (N/S), longitude, (W/E)
void gps_convert_deg_to_dec(double *, char, double *, char);
double gps_deg_dec(double);

uint8_t nmea_get_message_type(const char *);
uint8_t nmea_valid_checksum(const char *);
void nmea_parse_gx_gga(char *, gx_gga_t *);
void nmea_parse_gx_rmc(char *, gx_rmc_t *);

#endif

