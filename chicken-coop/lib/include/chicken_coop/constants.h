#pragma once

#include <Arduino.h>

namespace chicken_coop
{
    // Hardware feature toggles.
    inline constexpr bool DEVICE_BME_280_ENABLED = true;
    inline constexpr bool ENABLE_MONITORING = false;

    // Addressable RGB LED, driven by GPIO48.
    inline constexpr uint8_t LED_PIN = 48;
    inline constexpr int WDT_TIMEOUT = 30;

    inline constexpr uint8_t RELAY_PIN = D0;

    // BME temperature and humidity sensor, connected to i2c bus in current setup.
    inline constexpr uint8_t BME_SCK = D8;
    inline constexpr uint8_t BME_MISO = D9;
    inline constexpr uint8_t BME_MOSI = D10;
    inline constexpr uint8_t BME_CS = D7;

    inline constexpr uint8_t I2C_SDA = D4;
    inline constexpr uint8_t I2C_SCL = D5;
    inline constexpr uint8_t AMONIA_SENSOR_PIN = A0;
    inline constexpr uint32_t AMONIA_SENSOR_READ_INTERVAL_MS = 1000;

    inline constexpr float SEALEVELPRESSURE_HPA = 1013.25F;

    // MQTT stuff.
    inline constexpr const char *MDNS_HOSTNAME = "chicken";
    inline constexpr const char *THINGNAME = "esp32-c3-coop-temp-amonia-sensor";
    inline constexpr const char *PREFIX = "coop/";
    inline constexpr const char *BME_TEMPERATURE_TOPIC = "coop/bme280/temperature";
    inline constexpr const char *BME_PRESSURE_TOPIC = "coop/bme280/pressure";
    inline constexpr const char *BME_HUMIDITY_TOPIC = "coop/bme280/humidity";
    inline constexpr const char *BME_ALTITUDE_TOPIC = "coop/bme280/altitude";
    inline constexpr const char *AMONIA_SENSOR_TOPIC = "coop/amonia/raw";
    inline constexpr const char *MQTT_LOG_TOPIC = "coop/log/mydebug";
    inline constexpr const char *STATUS_TOPIC = "coop/status/read";
    inline constexpr const char *RELAY_1_SET_TOPIC = "coop/relay/1/set";
}
