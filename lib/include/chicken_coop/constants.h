#pragma once
// Hardware feature toggle, comment out hardware which you don't need
#define DEVICE_BME_280_ENABLED // enable bme280 temp/humidity sensor
// #define ENABLE_MONITORING

// Addressable RGB LED, driven by GPIO48.
#define LED_PIN 48
#define WDT_TIMEOUT 30

#define RELAY_PIN D0

// BME temperature and humidity sensor, connected to i2c bus in current setup
#define BME_SCK D8
#define BME_MISO D9
#define BME_MOSI D10
#define BME_CS D7

#define I2C_SDA D4
#define I2C_SCL D5
#define AMONIA_SENSOR_PIN A0
#define AMONIA_SENSOR_READ_INTERVAL_MS 1000

#define SEALEVELPRESSURE_HPA (1013.25)

// MQTT stuff
#define MDNS_HOSTNAME "chicken"

#define THINGNAME "esp32-c3-coop-temp-amonia-sensor" // thing name for MQTT broker
#define PREFIX "coop/"
#define BME_TEMPERATURE_TOPIC PREFIX "bme280/temperature"
#define BME_TEMPERATURE_TOPIC PREFIX "bme280/temperature"
#define BME_PRESSURE_TOPIC PREFIX "bme280/pressure"
#define BME_HUMIDITY_TOPIC PREFIX "bme280/humidity"
#define BME_ALTITUDE_TOPIC PREFIX "bme280/altitude"
#define AMONIA_SENSOR_TOPIC PREFIX "amonia/raw"
#define MQTT_LOG_TOPIC PREFIX "log/mydebug"
#define STATUS_TOPIC PREFIX "status/read"
#define RELAY_1_SET_TOPIC PREFIX "relay/1/set"
