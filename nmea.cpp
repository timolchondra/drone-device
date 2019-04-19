#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "nmea.h"

void nmea_parse_gx_gga(char *nmea, gx_gga_t *loc)
{
    char *p = nmea;

    p = strchr(p, ',')+1; //skip time

    p = strchr(p, ',')+1;
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->quality = (uint8_t)atoi(p);

    p = strchr(p, ',')+1;
    loc->satellites = (uint8_t)atoi(p);

    p = strchr(p, ',')+1;

    p = strchr(p, ',')+1;
    loc->altitude = atof(p);
}

void nmea_parse_gx_rmc(char *nmea, gx_rmc_t *loc)
{
    char *p = nmea;

    p = strchr(p, ',')+1; //skip time
    p = strchr(p, ',')+1; //skip status

    p = strchr(p, ',')+1;
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->speed = atof(p);

    p = strchr(p, ',')+1;
    loc->course = atof(p);
}

/**
 * Get the message type (GPGGA, GPRMC, etc..)
 *
 * This function filters out also wrong packages (invalid checksum)
 *
 * @param message The NMEA message
 * @return The type of message if it is valid
 */
uint8_t nmea_get_message_type(const char *message)
{ 
	uint8_t checksum = 0; 

	if ((checksum = nmea_valid_checksum(message)) != _EMPTY) {
        	return checksum;
    	}
	if (strstr(message, NMEA_GPGGA_STR) != NULL || strstr(message, NMEA_GNGGA_STR) != NULL) { 
		return NMEA_GxGGA; 
	}

	if (strstr(message, NMEA_GPRMC_STR) != NULL || strstr(message, NMEA_GNRMC_STR) != NULL) {
        	return NMEA_GxRMC;
    	}

    	return NMEA_UNKNOWN;
}

uint8_t nmea_valid_checksum(const char *message) 
{
	uint8_t checksum = (uint8_t)strtol(strchr(message, '*')+1, NULL, 16); 
	uint8_t sum = 0; 
	char p; 

	++message;

	while ((p = *message++) != '*') 
        	sum ^= p; 

	if (sum != checksum) 
		return NMEA_CHECKSUM_ERR; 
	
	return _EMPTY;
}


/* Convert lat/lon to decimal (from degrees) */
void gps_convert_deg_to_dec(double *latitude, char ns,  double *longitude, char we)
{
	double lat = (ns == 'N') ? *latitude : -1 * (*latitude); 
	double lon = (we == 'E') ? *longitude : -1 * (*longitude); 
	
	*latitude = gps_deg_dec(lat); 
	*longitude = gps_deg_dec(lon);
}

double gps_deg_dec(double deg_point)
{ 
	double ddeg; 
	double sec = modf(deg_point, &ddeg)*60; 
	int deg = (int)(ddeg/100); 
	int min = (int)(deg_point-(deg*100));

	double absdlat = round(deg * 1000000.); 
	double absmlat = round(min * 1000000.); 
	double absslat = round(sec * 1000000.);

	return round(absdlat + (absmlat/60) + (absslat/3600)) /1000000;
}


