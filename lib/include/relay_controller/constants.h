
#pragma once
// Hardware feature toggle, comment out hardware which you don't need
#define DEVICE_RELAY_ENABLED // enable relay controll
#define BLE_ENABLED          // enable ble communication
// #define ENABLE_MONITORING
#define WDT_TIMEOUT 30

#define RELAY_PIN D0

// MQTT stuff
#define THINGNAME "esp32-c3-basement-fhs232y3a43"
#define PREFIX "basement/"
#define MQTT_LOG_TOPIC PREFIX "log/mydebug"
#define STATUS_TOPIC PREFIX "status/read"
#define RELAY_1_SET_TOPIC PREFIX "relay/1/set"



// #define I2C_SDA D4
// #define I2C_SCL D5