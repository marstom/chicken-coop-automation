#include "common/ble.h"

namespace common::ble
{
    Ble::Ble(
        const char *serviceUuid,
        const char *requestCharacteristicUuid,
        const char *responseCharacteristicUuid,
        const char *requestDescriptorLabel,
        const char *responseDescriptorLabel,
        int requestMaxLength,
        int responseMaxLength)
        : deviceService(serviceUuid),
          deviceRequestCharacteristic(requestCharacteristicUuid, BLEWrite, requestMaxLength),
          deviceResponseCharacteristic(responseCharacteristicUuid, BLERead | BLENotify, responseMaxLength),
          requestName("2901", requestDescriptorLabel),
          responseName("2901", responseDescriptorLabel)
    {
    }

    Ble::~Ble()
    {
    }

    bool Ble::begin(const char *deviceName, const char *localName)
    {
        if (!::BLE.begin())
        {
            return false;
        }

        ::BLE.setLocalName(localName);
        ::BLE.setDeviceName(deviceName);

        deviceService.addCharacteristic(deviceRequestCharacteristic);
        deviceService.addCharacteristic(deviceResponseCharacteristic);

        deviceRequestCharacteristic.addDescriptor(requestName);
        deviceResponseCharacteristic.addDescriptor(responseName);
        deviceResponseCharacteristic.setValue("");

        ::BLE.setAdvertisedService(deviceService);
        ::BLE.addService(deviceService);
        ::BLE.advertise();

        return true;
    }

    BLEService &Ble::service()
    {
        return deviceService;
    }

    BLEStringCharacteristic &Ble::requestCharacteristic()
    {
        return deviceRequestCharacteristic;
    }

    BLEStringCharacteristic &Ble::responseCharacteristic()
    {
        return deviceResponseCharacteristic;
    }
}
