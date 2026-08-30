#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <PubSubClient.h>
#include <HTTPClient.h>

namespace relay_controller
{
    extern WiFiClient net;

    extern PubSubClient client;
    // GATT objects
    extern BLEService &deviceService;

    // phone writes
    extern BLEStringCharacteristic &deviceRequestCharacteristic;
    // phone reads / notify phone
    extern BLEStringCharacteristic &deviceResponseCharacteristic;

    void taskRelay(void *pvParameters);     // sole owner of RELAY_PIN
    void taskTcpServer(void *pvParameters); // direct connection
    void taskBLE(void *pvParameters);
}
