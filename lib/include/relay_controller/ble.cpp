#include "tasks.h"
#include "constants.h"

namespace relay_controller
{

    void setupBLE(const char *deviceName, const char *localName)
    {
        if constexpr (relay_controller::BLE_ENABLED)
        {
            // 1) MUST start BLE before using any other BLE APIs
            if (!BLE.begin())
            {
                Serial.println("BLE.begin() failed");
                for (;;)
                    delay(1000);
            }

            BLE.setLocalName(localName);
            BLE.setDeviceName(deviceName);

            // TODO move to ble.h
            // build GATT
            relay_controller::deviceService.addCharacteristic(relay_controller::deviceRequestCharacteristic);
            relay_controller::deviceService.addCharacteristic(relay_controller::deviceResponseCharacteristic);

            // add descriptors
            relay_controller::deviceRequestCharacteristic.addDescriptor(relay_controller::reqName);
            relay_controller::deviceResponseCharacteristic.addDescriptor(relay_controller::respName);

            relay_controller::deviceResponseCharacteristic.setValue(""); // initial value

            BLE.setAdvertisedService(relay_controller::deviceService);
            BLE.addService(relay_controller::deviceService);
            BLE.advertise();

            Serial.println("BLE initialized, advertising...");
            xTaskCreate(relay_controller::taskBLE, "taskBLE", 4096, NULL, 1, NULL);
        }
    }

}