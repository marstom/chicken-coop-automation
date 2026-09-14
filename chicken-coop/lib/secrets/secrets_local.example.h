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

/*
SECRETS for main_s3_ventilation


this is the main page on AWS:
https://us-east-1.console.aws.amazon.com/iot/home?region=us-east-1#/connectdevice
https://us-east-1.console.aws.amazon.com/iot/home?region=us-east-1#/thing/ventilation_temperature_sensors
*/

#define AWS_IOT_ENDPOINT ""
#define AWS_THINGNAME ""

const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...
-----END CERTIFICATE-----
)EOF";

const char DEVICE_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...
-----END CERTIFICATE-----
)EOF";

const char DEVICE_PRIVATE_KEY[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
...
-----END RSA PRIVATE KEY-----
)EOF";