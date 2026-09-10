#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>

namespace common::ble
{
    class Ble
    {
    public:
        Ble(
            const char *serviceUuid,
            const char *requestCharacteristicUuid,
            const char *responseCharacteristicUuid,
            const char *requestDescriptorLabel,
            const char *responseDescriptorLabel,
            int requestMaxLength = 32,
            int responseMaxLength = 32);
        ~Ble();

        bool begin(const char *deviceName, const char *localName);

        BLEService &service();
        BLEStringCharacteristic &requestCharacteristic();
        BLEStringCharacteristic &responseCharacteristic();

    private:
        BLEService deviceService;
        BLEStringCharacteristic deviceRequestCharacteristic;
        BLEStringCharacteristic deviceResponseCharacteristic;
        BLEDescriptor requestName;
        BLEDescriptor responseName;
    };
}
