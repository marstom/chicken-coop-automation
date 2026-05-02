
#pragma once
#include <Arduino.h>

// #define I2C_SDA D4
// #define I2C_SCL D5

namespace relay_controller
{
    // Hardware feature toggles.
    inline constexpr bool BLE_ENABLED = true;
    inline constexpr bool ENABLE_MONITORING = false;
    inline constexpr int WDT_TIMEOUT = 30;

    inline constexpr uint8_t RELAY_PIN = D0;

    // MQTT stuff.
    inline constexpr const char *THINGNAME = "esp32-c3-basement-fhs232y3a43";
    inline constexpr const char *PREFIX = "basement/";
    inline constexpr const char *MQTT_LOG_TOPIC = "basement/log/mydebug";
    inline constexpr const char *STATUS_TOPIC = "basement/status/read";
    inline constexpr const char *RELAY_1_SET_TOPIC = "basement/relay/1/set";
    // Bluetooth
    inline constexpr const char *deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
    inline constexpr const char *deviceServiceRequestCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
    inline constexpr const char *deviceServiceResponseCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
    // Web
    inline constexpr const char *MDNS_HOSTNAME = "relay";
}
