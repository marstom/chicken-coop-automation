#pragma once

#define SSID_OFFICE 
#define SSID_KITCHEN 
#define SSID_GARDEN 
#define WIFI_SSID
#define WIFI_PASS 

#define MQTT_HOST
#define MQTT_PORT 1883
#define MQTT_USER "admin"
#define MQTT_PASS

// "unset" disables the door TCP endpoint (GET /open?token=...)
#define DOOR_TOKEN
// "unset" disables opening the door over BLE
#define BLE_DOOR_PASS

// TODO
#define URL_BASEMENT "basement.local"
#define URL_COOP "coop.local"
