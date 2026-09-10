#pragma once
/*

How to?

create file in root
private.ini

[secret]
build_flags =
  -DWIFI_SSID=\"mySSID\"
  -DWIFI_PASS=\"pass\"
*/

#ifndef WIFI_SSID
#define WIFI_SSID "unset"
#endif
#ifndef SSID_OFFICE
#define SSID_OFFICE "unset"
#endif
#ifndef SSID_KITCHEN
#define SSID_KITCHEN "unset"
#endif
#ifndef SSID_GARDEN
#define SSID_GARDEN "unset"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "unset"
#endif
#ifndef MQTT_HOST
#define MQTT_HOST "raspberrypi.local"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER "admin"
#endif
#ifndef MQTT_PASS
#define MQTT_PASS "admin"
#endif
// Token required by the door TCP endpoint (GET /open?token=...).
// "unset" disables the endpoint entirely.
#ifndef DOOR_TOKEN
#define DOOR_TOKEN "paulina"
#endif
// Password required over BLE to open the door. "unset" disables BLE open.
#ifndef BLE_DOOR_PASS
#define BLE_DOOR_PASS "paulina"
#endif
