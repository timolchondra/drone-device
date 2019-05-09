 
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

//#include "Arduino.h"
#include "MultiChannelGasSensor.h"
#include <stack>


#define TASK_STACK_DEPTH 2048

#define I2C_FREQ I2C_FREQ_100K
#define I2C_BUS 1
#define i2c_scl_pin     22
#define i2c_sda_pin     21

using namespace std;
MultiChannelGasSensor gas;
struct gps_position myGPS; 
static ccs811_sensor_t* sensor;
SemaphoreHandle_t xSemaphore;

//const char *server_ip_address = "10.0.0.229";
//const char *server_ip_address           = "192.168.1.154";

//Fablab IP from my laptop
//const char *server_ip_address = "192.168.1.117";

//Matt's IP address for Drone
//const char *server_ip_address = "192.168.42.33";

//My IP address through the Drone
const char *server_ip_address = "192.168.42.77";
const int   server_webserver_port       = 8080;

int deviceID = 1;


struct ccs811Data {
    int TVOC;
    int CO2;
};


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


struct gpsData {
    double lat;
    double lon;
};

//queue<ccs811Data> ccs811Queue;
stack<gpsData> gpsStack;
stack<groveData> groveStack;

void post_task(void *args) {
    groveData Grove;
    gpsData GPS;
//    ccs811Data CCS811;

    char *msg;
    int firstPost = 0;

    while(1) {
        if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
           if(!groveStack.empty() && !gpsStack.empty()) {
                Grove = groveStack.top();
                GPS = gpsStack.top();

                //printf("%s\n", msg);
               
                groveStack.pop();
                gpsStack.top();
          
                //first post always has to send the message to /register to register the drone

                if(firstPost == 0) {
                    post_to_server(deviceID, msg, firstPost);
                    firstPost = 1;
                    free(msg);
                } else {

                post_to_server(deviceID, msg, firstPost);
                free(msg);

                }
            }

           
            xSemaphoreGive( xSemaphore);
           vTaskDelay(200/portTICK_PERIOD_MS);
        }
    }
}
/*
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
             vTaskDelay(200/portTICK_PERIOD_MS);
        }
        // passive waiting until 1 second is over
    }
}
*/
void grove_task(void *args) {
   groveData readGrove;

   while(1) {
    if(xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE) {
        readGrove.nh3 =gas.measure_NH3();
    //printf("The concentration of NH3 is ");
    //if(c>=0) printf("%f",c);
    //else printf("invalid");
    //printf(" ppm\n");

        readGrove.co = gas.measure_CO();
    
     //   printf("The concetration of CO is ");
       // if(readGrove.co>=0) printf("%f", readGrove.co);
       // else printf("invalid");
       // printf(" ppm\n");

        readGrove.no2 = gas.measure_NO2();
    
        readGrove.c3h8 = gas.measure_C3H8();
        readGrove.c4h10 = gas.measure_C4H10();

        readGrove.ch4 = gas.measure_CH4();
        readGrove.h2 = gas.measure_H2();
        readGrove.c2h5oh = gas.measure_C2H5OH();
        groveStack.push(readGrove);
        xSemaphoreGive(xSemaphore);
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
  } 


}

void gps_task(void *args) {
    gpsData readGPS;
    TickType_t last_wakeup = xTaskGetTickCount();

    while(1) {
       if(xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE) {
            get_gps_data(&myGPS);
      //      printf("%lf, %lf\n",myGPS.lat, myGPS.lon );
      
            readGPS.lat = myGPS.lat;
            readGPS.lon = myGPS.lon;
            gpsStack.push(readGPS);
            xSemaphoreGive( xSemaphore);
            vTaskDelay(200/portTICK_PERIOD_MS);

       }

     }
}
void connectToWifi() {
    esp_err_t ret = nvs_flash_init();
      if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    //ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  
    app_wifi_initialise();
    app_wifi_wait_connected();
    
}
extern "C" void app_main(void) {
    //initArduino();

    connectToWifi();  
    
    vTaskDelay(1);
   
    esp_err_t gpsStatus = init_gps(&myGPS);

   // sensor = ccs811_init_sensor (I2C_MASTER_NUM, CCS811_I2C_ADDRESS_2);
   // ccs811_set_mode (sensor, ccs811_mode_1s);

    gas.begin(0x04);
    gas.powerOn();
    
    vTaskDelay(1); 
    if(gpsStatus == ESP_OK){
      
      xSemaphore = xSemaphoreCreateMutex();
      printf("ESP ID: %d\n", deviceID);
      //xTaskCreatePinnedToCore(CCS811_task,"CCS811_task", TASK_STACK_DEPTH, NULL, 2, NULL,0);

        xTaskCreatePinnedToCore(grove_task, "grove_task", TASK_STACK_DEPTH, NULL, 2, NULL, 1);
        xTaskCreatePinnedToCore(gps_task, "gps_task",TASK_STACK_DEPTH, NULL, 2, NULL,1);
        xTaskCreatePinnedToCore(post_task, "post_task", 4042, NULL, 2, NULL,0);       //need larger stack for task to post data



    } else {
      printf("fail to init sensor\n");
      printf("ESP ID: %d\n", deviceID);

    }
}
