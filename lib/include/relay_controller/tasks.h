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

    /// make mqtt thread safe
    void taskMQTT(void *pvParameters); // Spin all the time and keep receiving the messages!
    // void taskReadBME280(void *pvParameters);
    void taskStackMonitor(void *pvParameters); // debug stack monitor for memory usage
    void taskRelay(void *pvParameters);
    void taskTcpServer(void *pvParameters); // direct connection

    void taskBLE(void *pvParameters);

}
