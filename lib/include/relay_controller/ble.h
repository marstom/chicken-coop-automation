#pragma once
#include <ArduinoBLE.h>

namespace relay_controller
{
    // GATT objects

    extern BLEService deviceService;
    extern BLEStringCharacteristic deviceRequestCharacteristic;
    extern BLEStringCharacteristic deviceResponseCharacteristic;
    extern BLEDescriptor reqName;
    extern BLEDescriptor respName;


    void setupBLE(const char *deviceName, const char *localName);

}