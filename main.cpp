 
/* Timothy Olchondra
 * Code to run ESP32 to gather both sensor data
 *
 * Gases read by grove sensor:
 *   Carbon monoxide CO 1 – 1000ppm
 *   Nitrogen dioxide NO2 0.05 – 10ppm
 *   Ethanol C2H5OH 10 – 500ppm
 *   Hydrogen H2 1 – 1000ppm
 *   Ammonia NH3 1 – 500ppm
 *   Methane CH4 >1000ppm
 *   Propane C3H8 >1000ppm
 *   Iso-butane C4H10 >1000ppm
 *
 *
 */ 


#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_client.h"

#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"

#include "rest.h"
#include "rest.cpp"
#include "ccs811.h"
#include "gps.h"
#include "gps.cpp"

#include "app_wifi.h"
#include "driver/i2c.h"
#include "nmea.h"
#ifdef __cplusplus
}
#endif

#include "Arduino.h"
#include "MutichannelGasSensor.h"
#include "MutichannelGasSensor.cpp"
#include <queue>


#define TASK_STACK_DEPTH 2048

#define I2C_FREQ I2C_FREQ_100K
#define I2C_BUS 1
#define i2c_scl_pin     22
#define i2c_sda_pin     21


using namespace std; 
struct gps_position myGPS; 
static ccs811_sensor_t* sensor;
SemaphoreHandle_t xSemaphore;

//const char *server_ip_address = "10.0.0.229";
//const char *server_ip_address           = "192.168.1.154";

//Fablab IP from my laptop
//const char *server_ip_address = "192.168.1.117";

//Matt's IP address for Drone
const char *server_ip_address = "192.168.42.33";

//my IP address for Drone
//const char *server_ip_address = "192.168.42.77";
const int   server_webserver_port       = 8080;

int deviceID = 1;


struct ccs811Data {
    int TVOC;
    int CO2;
};

/*
struct groveData {
    float co;
    float nh3;
    float no2;
    float c3h8;
    float c4h10;
    float ch4;
    float h2;
    float c2h5oh;


};
*/

struct gpsData {
    double lat;
    double lon;
};

queue<ccs811Data> ccs811Queue;
queue<gpsData> gpsQueue;
//queue<groveData> groveQueue;


void post_task(void *args) {
 //   groveData Grove;
    gpsData GPS;
    ccs811Data CCS811;

    char *msg;
    int firstPost = 0;

    while(1) {
        if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
           if(!ccs811Queue.empty() && !gpsQueue.empty()) {
                //Grove = groveQueue.front();
               CCS811 = ccs811Queue.front(); 
               GPS = gpsQueue.front();
                
                //groveQueue.pop();
                
                ccs811Queue.pop();
                gpsQueue.pop();

                msg = form_update_message(deviceID, CCS811.CO2, CCS811.TVOC, GPS.lat, GPS.lon);

                //printf("%s\n", msg);
               

                //first post always has to send the message to /register to register the drone

                if(firstPost == 0) {
                    post_to_server(deviceID, msg, firstPost);
                    firstPost = 1;
                    free(msg);
                    msg = NULL;
                } else {

                post_to_server(deviceID, msg, firstPost);
                free(msg);
                msg = NULL;

                }
            }

           
            xSemaphoreGive( xSemaphore);
           vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }
}

void CCS811_task(void *args) {
    uint16_t tvoc;
    uint16_t eco2;
    ccs811Data readCCS811;
    
    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        if(xSemaphoreTake( xSemaphore, ( TickType_t ) 10 )==pdTRUE) {
            if (ccs811_get_results (sensor, &tvoc, &eco2, 0, 0)) {
                printf("%.3f CCS811 Sensor periodic: TVOC %d ppb, eCO2 %d ppm\n", (double)sdk_system_get_time()*1e-3, tvoc, eco2);
            
                readCCS811.TVOC = tvoc;
                readCCS811.CO2 = eco2;

                ccs811Queue.push(readCCS811);

            }
             xSemaphoreGive( xSemaphore);
             vTaskDelay(100/portTICK_PERIOD_MS);
        }
        // passive waiting until 1 second is over
    }
}
/*
void grove_task(void *args) {
   groveData readGrove;

   while(1) {
    readGrove.nh3 =gas.measure_NH3();
    //printf("The concentration of NH3 is ");
    //if(c>=0) printf("%f",c);
    //else printf("invalid");
    //printf(" ppm\n");
    vTaskDelay(1);

    readGrove.co = gas.measure_CO();
    vTaskDelay(1);
    
    //    printf("The concetration of CO is ");
 //   if(readGrove.co>=0) printf("%f", readGrove.co);
 //   else printf("invalid");
 //   printf(" ppm\n");

    readGrove.no2 = gas.measure_NO2();
    
    vTaskDelay(1);
    readGrove.c3h8 = gas.measure_C3H8();
    vTaskDelay(1);
    readGrove.c4h10 = gas.measure_C4H10();
    vTaskDelay(1);

    readGrove.ch4 = gas.measure_CH4();
    vTaskDelay(1);
    readGrove.h2 = gas.measure_H2();
    vTaskDelay(1);
    readGrove.c2h5oh = gas.measure_C2H5OH();
    vTaskDelay(1);
    groveQueue.push(readGrove);
    xSemaphoreGive(xSemaphore);
    vTaskDelay(100/portTICK_PERIOD_MS);
  } 


}
*/
void gps_task(void *args) {
    gpsData readGPS;
    TickType_t last_wakeup = xTaskGetTickCount();

    while(1) {
       if(xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE) {
            get_gps_data(&myGPS);
            printf("%lf, %lf\n",myGPS.lat, myGPS.lon );
      
            readGPS.lat = myGPS.lat;
            readGPS.lon = myGPS.lon;
            gpsQueue.push(readGPS);
            xSemaphoreGive( xSemaphore);
            vTaskDelay(100/portTICK_PERIOD_MS);

       }

     }
}

extern "C" void app_main(void) {
     initArduino();
     
     esp_err_t ret = nvs_flash_init();
      if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
      }
     ESP_ERROR_CHECK(ret);
    
     ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  
     app_wifi_initialise();
     app_wifi_wait_connected();
    
    vTaskDelay(1);
   
    esp_err_t gpsStatus = init_gps(&myGPS);

    i2c_init(I2C_NUM_0, (gpio_num_t) i2c_scl_pin, (gpio_num_t) i2c_sda_pin, I2C_FREQ);
    sensor = ccs811_init_sensor (I2C_NUM_0, CCS811_I2C_ADDRESS_2);
    
    if(sensor) {
         printf("CCS811 sensor has been initialized!\n");
    } else {
        printf("failed to init CCS811\n");
    }



    //gas.begin(0x04);
    //gas.powerOn();
    vTaskDelay(10);
    if(gpsStatus == ESP_OK && sensor){
      printf("Successfully initialized both sensors!\n"); 
     
      ccs811_set_mode (sensor, ccs811_mode_1s);
      xSemaphore = xSemaphoreCreateMutex();
      printf("ESP ID: %d\n", deviceID);
        

     // xTaskCreatePinnedToCore(grove_task, "grove_task", TASK_STACK_DEPTH, NULL, 2, NULL, 1);
      xTaskCreatePinnedToCore(CCS811_task,"CCS811_task", TASK_STACK_DEPTH, NULL, 2, NULL,1);
      xTaskCreatePinnedToCore(gps_task, "gps_task",TASK_STACK_DEPTH, NULL, 2, NULL,1);
      xTaskCreatePinnedToCore(post_task, "post_task", 4042, NULL, 2, NULL,0);       //need larger stack for task to post data



    } else {
      printf("ESP ID: %d\n", deviceID);
      pinMode(5,OUTPUT);
      digitalWrite(5,HIGH);

    }
}
