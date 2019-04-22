/* 
 * Copyright (c) 2018 Matthew E. Tolentino, UW, metolent@uw.edu
 * Intelligent Platforms and Architecture (IPA) Lab, UW-Tacoma, USA
 *
 * <NEED LICENSE INFO HERE> 
 *
 */

#include <string.h>

#include "rest.h"

/* This is for the buffer size when forming destination URL strings */
#define URL_SIZE	100

/* matt - using this one to test out the gps+mesh+serverpush integration */

struct http_response_data rxd_http_response;


esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
	switch (evt->event_id) {
		case HTTP_EVENT_ERROR:
			//printf("HTTP_EVENT_ERROR\n");
			break;
		case HTTP_EVENT_ON_CONNECTED:
			//printf("HTTP_EVENT_ON_CONNECTED\n");
			break;
		case HTTP_EVENT_HEADER_SENT:
			//printf("HTTP_EVENT_HEADER_SENT\n");
			break;
		case HTTP_EVENT_ON_HEADER:
			//printf("HTTP_EVENT_ON_HEADER, key=%s, value=%s\n", evt->header_key, evt->header_value);
			break;
		case HTTP_EVENT_ON_DATA:
			//printf("HTTP_EVENT_ON_DATA, len=%d\n", evt->data_len);
			/* save off the data received to our global buffer */
			memset(&rxd_http_response, 0, sizeof(struct http_response_data));
			rxd_http_response.size = evt->data_len;
			memcpy(rxd_http_response.data, evt->data, evt->data_len);
			//rxd_http_response.data = (char *)evt->data;
			//if (!esp_http_client_is_chunked_response(evt->client)) { 
			//}
			break;
		case HTTP_EVENT_ON_FINISH:
			//printf("HTTP_EVENT_ON_FINISH\n");
			break;
		case HTTP_EVENT_DISCONNECTED:
			//printf("HTTP_EVENT_DISCONNECTED\n");
			break;
		default: 
			break;
	}
	return ESP_OK;
}

char *form_update_message(int id,float co, float nh3, float no2, float c3h8, float c4h10, float ch4, float h2, float c2h5oh, double lat, double lon )
{
	cJSON *root = cJSON_CreateObject();
	char *message; 


	/* create the message to the server */
    cJSON_AddNumberToObject(root, "id", id);
    cJSON_AddNumberToObject(root, "type", 3);
    cJSON_AddNumberToObject(root, "lat", lat);
    cJSON_AddNumberToObject(root, "lon", lon);
    cJSON_AddNumberToObject(root, "co", co);
    cJSON_AddNumberToObject(root, "nh3", nh3);
    cJSON_AddNumberToObject(root, "no2", no2);
    cJSON_AddNumberToObject(root, "c3h8", c3h8);
    cJSON_AddNumberToObject(root, "c4h10", c4h10);
    cJSON_AddNumberToObject(root, "ch4", ch4);
    cJSON_AddNumberToObject(root, "h2", h2);
    cJSON_AddNumberToObject(root, "c2h5oh", c2h5oh);
    
	message = cJSON_Print(root);
	cJSON_Delete(root);

	return message;
}
// 
// 
// /* 
//  * matt - This currently establishes a new connection to the server on every call. 
//  * This should probably be updated to just keep a connection open for the calls
//  * that come in from beacons/FF devices AND reduce latency. 
//  * I'm leaving it as-is right now, but it's a TODO to fix this before scalability 
//  * tests start.  
//  */
void get_from_server() 
{
	esp_err_t err; 
	char url[60];

	memset(url, 0, 60);
	sprintf(url, "http://%s:%d%s", 
			server_ip_address, server_webserver_port, "/page1");
	
	esp_http_client_config_t rest_server_config;
	memset(&rest_server_config, 0, sizeof(esp_http_client_config_t));
	rest_server_config.url = url;
	rest_server_config.event_handler = http_event_handler;

	/* Fire it up */
	esp_http_client_handle_t client = esp_http_client_init(&rest_server_config);

	/* Go for it!  Note that without an additional arg, this does a GET */
	err = esp_http_client_perform(client);
	if (err == ESP_OK) { 
		printf("HTTP GET Status = %d, content_length = %d\n", 
				esp_http_client_get_status_code(client),
				esp_http_client_get_content_length(client));
	} else 
		printf("HTTP GET request failed: %s\n", esp_err_to_name(err));

	//printf("Cleaning up http....\n");
	esp_http_client_cleanup(client);

	/* look at the data received */
	//printf("REST API response: %s\n", rxd_http_response.data);
}

char *post_to_server(int deviceID, char *post_data)
{
	esp_http_client_config_t rest_server_config;
	char url[URL_SIZE];
	const char *post_function;
	esp_err_t err;
   
    post_function = "/update";


	memset(&rest_server_config, 0, sizeof(esp_http_client_config_t));
	memset(url, 0, URL_SIZE);

	sprintf(url, "http://%s:%d%s", server_ip_address, server_webserver_port, post_function);

	rest_server_config.url = url;
	rest_server_config.event_handler = http_event_handler;

	/* Fire it up */
	esp_http_client_handle_t client = esp_http_client_init(&rest_server_config);

	esp_http_client_set_method(client, HTTP_METHOD_POST);
	esp_http_client_set_post_field(client, post_data, strlen(post_data));

	/* Go for it!  Note that without an additional arg, this does a GET */
	err = esp_http_client_perform(client);
	if (err != ESP_OK) { 
	//	printf("HTTP POST to %s failed: %s\n", post_function, esp_err_to_name(err));
		/* 
		 * uhhhhh - how about we DO something if this failed... 
		 * oh wait, what if the server just isn't up yet... or went down? 
		 * Ok, fine we at least need to pass this err back to the caller. 
		 */
		return NULL;
	}

	esp_http_client_cleanup(client);

	/* We should have a response by now.... this feels sketchy. FIXME Need certainly here */
	//printf("REST API response: 0x%x\n", atoi(rxd_http_response.data));

	return rxd_http_response.data;
}
