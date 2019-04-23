#ifndef _REST_H_
#define _REST_H_

#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_http_client.h"
#include "esp_mesh.h"
#include "esp_err.h"
#include "cJSON.h"

extern const char *server_ip_address;
extern const int server_webserver_port;

#define REST_POST_REGISTRATION			1
#define REST_POST_UPDATE			2
#define REST_POST_REGISTRATION_RESPONSE		3

/* Interface to REST server */
struct http_response_data { 
	int size;
	char data[DEFAULT_HTTP_BUF_SIZE]; 	/* This is currently set to 512 in esp_http_client.h */
};


void get_from_server(void);
char *post_to_server(int deviceID, char *post_data, int firstPost);

char *form_update_message(int id, int co2, int tvoc, double lat, double lon);
#endif
