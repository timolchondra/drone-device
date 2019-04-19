#ifndef _GPS_H_
#define _GPS_H_

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nmea.h"


#include "esp_err.h"

/* Configurables that must be tuned based on board/platform connectivity */


/* This should be tuned at some point to make it of minimal size */
#define GPS_STACK_SIZE		3072

/* GPS device specifics */
#define MT333x_ADDR 		0x10
#define GPS_I2C_ADDR		MT333x_ADDR
#define MAX_PACKET_SIZE		255
#define I2C_SPEED_STANDARD	100000
#define I2C_SPEED_FAST		400000

#define I2C_MASTER_NUM		I2C_NUM_1
#define WRITE_BIT		I2C_MASTER_WRITE
#define READ_BIT		I2C_MASTER_READ
#define ACK_CHECK_EN		0x1
#define ACK_CHECK_DIS		0x0
#define ACK_VAL			0x0
#define NACK_VAL		0x1

#define I2C_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_SDA_PIN	    14	
#define I2C_SCL_PIN		13
struct gps_position { 
	int gps_present;
	unsigned long time; 		/* matt - Not currently accounting for time yet */
	double lat; 
	double lon; 
	double alt; 
	double speed; 
	double course;
	uint8_t quality;
	uint8_t nr_satellites;

	SemaphoreHandle_t lock;
};


/* main entry point to the GPS driver */
esp_err_t init_gps(struct gps_position *);


#endif

