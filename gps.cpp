/* 
 * Copyright (c) 2018 Matthew E. Tolentino, UW, metolent@uw.edu
 * Intelligent Platforms and Architecture (IPA) Lab, UW-Tacoma, USA
 *
 * <NEED LICENSE INFO HERE> 
 *
 */
#include <stdio.h>
#include <string.h>

#include "esp_task.h"
#include "driver/i2c.h"
#include "gps.h"
#include "nmea.h"
#include "nmea.cpp"

/* 
 * Probe the GPS by writing to the device and then trying to read from it. If
 * we get a response, then we can safely asssume that we can access the device. 
 * Come to think of it, I dont' see why would need the WRITE at all.  
 */
static esp_err_t probe_gps(i2c_port_t i2c_num)
{
	uint8_t dummy_buffer = 0;
    	int ret;

    	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    	i2c_master_start(cmd);
    	i2c_master_write_byte(cmd, GPS_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    	i2c_master_stop(cmd);
    	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    	i2c_cmd_link_delete(cmd);
    	if (ret != ESP_OK) {
        	return ret;
    	}
    	vTaskDelay(30 / portTICK_RATE_MS);
    	cmd = i2c_cmd_link_create();
    	i2c_master_start(cmd);
    	i2c_master_write_byte(cmd, GPS_I2C_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    	i2c_master_read_byte(cmd, &dummy_buffer, I2C_MASTER_NACK);
    	i2c_master_stop(cmd);
    	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    	i2c_cmd_link_delete(cmd);

    	return ret;
}

/* 
 * READ a data blob of NMEA sentence information from the GPS.  This is pretty simple. 
 * We will always try to read 255 bytes from the GPS buffer.  This 255 data blob is 
 * not ordered.  It may start in the middle of a NMEA sentence, at the beginning, or
 * have nothing in it.  We'll have to parse it after we get the blob.  Also note 
 * that based on the esp-idf, we actually need to read 254 bytes first that will be
 * ACK'd by the I2C driver. The last byte (e.g. 255th), we read in a separate step
 * that will NOT require an ACK (e.g. NACK). Also note the pause afterwards can 
 * likely be shortened from 1s to something more reasonable.  
 */
static esp_err_t read_from_gps(i2c_port_t i2c_num, uint8_t *sentence)
{
	i2c_cmd_handle_t cmd; 
    	int ret;

    	vTaskDelay(30 / portTICK_RATE_MS);
    	cmd = i2c_cmd_link_create();
    	i2c_master_start(cmd);
    	i2c_master_write_byte(cmd, GPS_I2C_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    	i2c_master_read(cmd, sentence, MAX_PACKET_SIZE-1, I2C_MASTER_ACK);
    	i2c_master_read_byte(cmd, sentence+254, I2C_MASTER_NACK);
    	i2c_master_stop(cmd);
    	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    	i2c_cmd_link_delete(cmd);

    	return ret;
}

/* 
 * Initialize the I2C driver stack to enable us to start talking to the 
 * GPS.  Use the SDA and SCL pin assignments in the defines at the start
 * of this file. 
 */
static void init_i2c()
{
    	i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    	i2c_config_t conf;

    	conf.mode = I2C_MODE_MASTER;
    	conf.sda_io_num = (gpio_num_t) I2C_SDA_PIN;
    	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    	conf.scl_io_num = (gpio_num_t) I2C_SCL_PIN;
    	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    	conf.master.clk_speed = I2C_SPEED_STANDARD;

    	i2c_param_config((i2c_port_t) i2c_master_port, &conf);

    	i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

/* 
 * We need to start by searching for the start of a NMEA sentence, 
 * which is a $. Then starting form that point, search for the marker
 * of the end of a sentence, which is a *.  The next number after the 
 * end marker (e.g. *) is the checksum for the sentence.  Use that 
 * to verify the validity of the sentence.  
 * Note that strchr returns a valid pointer to the symbol on success, 
 * and NULL otherwise
 */
static esp_err_t parse_gps_data(char *gps_data, struct gps_position *current_position)
{
	char *start = NULL, *end = NULL, *data = gps_data;
	int keep_looking = 1;

	while (keep_looking) { 
		/* find the first sentence start - if nothing, bail out */
		start = strchr(data, '$');
		if (!start) {
			keep_looking = 0;
			continue;
		} 
		/* cool - we have the start of a sentence, look for the end - if nothing, bail */
		end = strchr(start, '*');
		if (!end) {
			keep_looking = 0;
			continue;
		}
		/* Ok, we have the end of a sentence too! */
		size_t sentence_length;
		char *sentence; 
		gx_gga_t gx_gga;
		gx_rmc_t gx_rmc;

		/* 
		 * The end pointer is pointing at the * character, so 
		 * move it to the start of the next sentence so this
		 * message contains the checksum. 
		 */
		end = strchr(end, '$');
		if (!end) { 
			/* 
			 * If the start of the next sentence isn't available, 
			 * we probably ran out of content in this data blob.
			 * This isn't exactly ideal as we clearly saw the end
			 * of a sentence (via * detection above), BUT we 
			 * can't verify the checksum. Fuck it for now... 
			 */
			keep_looking = 0;
			continue;
		}

		sentence_length = end - start;

		sentence = (char *)malloc(sentence_length);
		if (!sentence) {
			printf("parse_gps_data: Failed to allocate memory ....\n");
			return ESP_ERR_NO_MEM;
		}
		memset(sentence, 0, sentence_length);
		memcpy(sentence, start, sentence_length);

		/* Ok, got just the sentence, do the rest */
		switch (nmea_get_message_type(sentence)) {
			case NMEA_GxGGA: 
				memset(&gx_gga, 0, sizeof(gx_gga_t));
				nmea_parse_gx_gga(sentence, &gx_gga);
				gps_convert_deg_to_dec(&gx_gga.latitude, gx_gga.lat, 
						&gx_gga.longitude, gx_gga.lon);
				 
				//printf("GGA: (%f, %f) Alt: %f Quality: %d Satellites: %d\n", 
				//		gx_gga.latitude, gx_gga.longitude, 
			//			gx_gga.altitude, gx_gga.quality, gx_gga.satellites);
				
				xSemaphoreTake(current_position->lock, portMAX_DELAY);

				current_position->lat = gx_gga.latitude;
				current_position->lon = gx_gga.longitude; 
				current_position->alt = gx_gga.altitude;
				current_position->quality = gx_gga.quality;
				current_position->nr_satellites = gx_gga.satellites;

				xSemaphoreGive(current_position->lock);
				break;
			case NMEA_GxRMC:
				memset(&gx_gga, 0, sizeof(gx_gga_t));
				nmea_parse_gx_rmc(sentence, &gx_rmc);
				//printf("RMC: Speed: %f, Course: %f\n", gx_rmc.speed, gx_rmc.course);
				break;
		}
		/* make sure to free the allocated memory! */
		free(sentence);
		
		/* update the pointer for the next pass through this blob */
		data = end; 
	}	
	return ESP_OK;
}

/* 
 * Main thread for the GPS monitoring driver. 
 *
 * Get the GPS data blob from the GPS.  This reads and stores 255 bytes from the GPS via I2C, 
 * then calls parse_gps_data() to take the raw data blob from the GPS and turn them into 
 * valid NMEA sentences that have data we can use. This function is set to act as the main 
 * thread for the GPS driver and is expected to be spawned by a caller to execute stand-alone. 
 * That's why it's in an infinite loop with a call to vTaskDelete() at the end to kill it off
 * should something nefarious happen that breaks it out of the infinite loop.  
 */
void get_gps_data(struct gps_position *current_position)
{
	uint8_t gps_data[MAX_PACKET_SIZE];
	esp_err_t status; 
		
	/* Ensure we have a pointer to the global addresses of our current position */	
	/* TODO - Fix error handling here.  */
	//current_position = (struct gps_position *)arg; 

		/* zero out our data read blob buffer */
		memset(gps_data, 0, MAX_PACKET_SIZE);

		/* try to read a data blob rom the GPS */
		status = read_from_gps(I2C_MASTER_NUM, gps_data);
		if (status == ESP_OK) { 
			if (gps_data[0] == 0x0A) 
				return;
			/* 
			 * Ok, so we have a data blob that may have some sentences, so we 
			 * need to go through the data blob and parse out sentences. 
			 */
			parse_gps_data((char *)gps_data, current_position);

			vTaskDelay(2 / portTICK_RATE_MS);

		} else  { 
			/* fuck - i2c failed to read.  well, try again. probably need a timeout  */
			printf("I2C READ FAILED...%d \n", status);
			vTaskDelay(100 / portTICK_RATE_MS);
		} 
}

/* 
 * This function initializes and installs the I2C driver to enable access to the GPS 
 * controller over I2C.  It will probe the GPS to determine if a GPS is present 
 * before either bailing (e.g. No GPS found) or spawning a thread to continuously read
 * values from the GPS. It is expected this is the function an application would call to 
 * kick off the GPS monitoring capability.  
 *
 * Note that the SDA and SCL pin settings for I2C are defined in the gps.h header.  You
 * must ensure those are correctly connected to your esp32.  
 */
esp_err_t init_gps(struct gps_position *current_position)
{
	init_i2c();
	esp_err_t status; 

	status = probe_gps(I2C_MASTER_NUM);
	if (status != ESP_OK) { 
		printf("GPS: No GPS detected. \n");
		return ESP_ERR_NOT_FOUND; 
	}

	printf("GPS Found and Initialized!  ...\n"); 

	//memset(current_position, 0, sizeof(struct gps_position));
	current_position->lock = xSemaphoreCreateMutex();

	//xTaskCreate(get_gps_data, "gpsd", GPS_STACK_SIZE, current_position, 5, NULL);
	return ESP_OK;
}

