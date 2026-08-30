#include "relay_controller/ble.h"

#include "common/ble.h"
#include "relay_controller/constants.h"
#include "relay_controller/tasks.h"

namespace relay_controller
{
    namespace
    {
        common::ble::Ble bleDevice(
            deviceServiceUuid,
            deviceServiceRequestCharacteristicUuid,
            deviceServiceResponseCharacteristicUuid,
            "Phone to ESP request",
            "ESP to Phone response");
    }

    BLEService &deviceService = bleDevice.service();
    BLEStringCharacteristic &deviceRequestCharacteristic = bleDevice.requestCharacteristic();
    BLEStringCharacteristic &deviceResponseCharacteristic = bleDevice.responseCharacteristic();

    void setupBLE(const char *deviceName, const char *localName)
    {
        if constexpr (relay_controller::BLE_ENABLED)
        {
            if (!bleDevice.begin(deviceName, localName))
            {
                Serial.println("BLE.begin() failed");
                for (;;)
                    delay(1000);
            }

            Serial.println("BLE initialized, advertising...");
            xTaskCreate(relay_controller::taskBLE, "taskBLE", 4096, NULL, 1, NULL);
        }
    }
}
