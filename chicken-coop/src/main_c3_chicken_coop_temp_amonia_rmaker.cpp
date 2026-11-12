
/*

This is chicken coop monitor.

List of all mdns devices is:
dns-sd -B _http._tcp

*/

#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

// my libs
#include "secrets.h"
// #include "common/wifi_conn/wifi_conn.h"
#include "common/mqtt.h"
#include "common/ota.h"
// #include "common/web.h"
#include "mqtt_comm.h"
#include "debug_tools/debug_tools.h"
#include "chicken_coop/tasks.h"
#include "chicken_coop/constants.h"
#include "chicken_coop/web.h"
#include "wifi_conn_soft_ap.h"

// MQTT broker settings
const char *mqttHost = MQTT_HOST;
const uint16_t mqttPort = MQTT_PORT;

// --- Task handles (needed for stack monitoring) ---
TaskHandle_t hMQTTTask = NULL;
TaskHandle_t hBME280Task = NULL;
TaskHandle_t hStackMonTask = NULL;
TaskHandle_t hWebServerTask = NULL;

// MQTT service task configuration; static lifetime, the task keeps a pointer.
// This board only publishes sensor readings - no subscriptions.
static common::mqtt::TaskConfig mqttConfig = {
    &chicken_coop::client,
    MQTT_USER,
    MQTT_PASS,
    chicken_coop::THINGNAME,
    nullptr, // no subscription
    chicken_coop::STATUS_TOPIC,
    "{\"message\": \"Initialized the connection from chicken coop sensor\"}",
};

// Tasks watched by the stack monitor (terminated by {nullptr, nullptr}).
static debug_tools::MonitoredTask monitoredTasks[] = {
    {"MQTTTask", &hMQTTTask},
    {"BME280Task", &hBME280Task},
    {"WebServerTask", &hWebServerTask},
    {nullptr, nullptr},
};

void setup()
{
    Serial.begin(9600);

    // Setup logging
    debug_tools::logOptions.printToSerial = true;
    debug_tools::logOptions.logToMqtt = true;
    debug_tools::logPrefix = chicken_coop::PREFIX;

    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb
    wifi::connectToWifiWithSoftAPProvisioning(chicken_coop::MDNS_HOSTNAME, "kurnikService", POP_CODE, true);

    chicken_coop::client.setServer(mqttHost, mqttPort);

    debug_tools::logMessage("Initialize watchdog");
    esp_task_wdt_init(chicken_coop::WDT_TIMEOUT, true);

    // Init wireless updates
    common::ota::setup(chicken_coop::MDNS_HOSTNAME); // must match upload_port in platformio.ini

    chicken_coop::simpleWebPage(); // run server with web page at 80
    Serial.println("WebServer started");
    communication::initQueue();

    // main mqtt task
    xTaskCreatePinnedToCore(common::mqtt::taskMQTT, "taskMQTT", 2048 * 4, &mqttConfig, 1, &hMQTTTask, 0);
    xTaskCreate(chicken_coop::taskReadBME280, "taskReadBME280", 2048 * 4, NULL, 1, &hBME280Task); // temperature & pressure sensor
    xTaskCreate(chicken_coop::taskWebServer, "taskWebServer", 4096 * 2, NULL, 1, &hWebServerTask);
    xTaskCreate(chicken_coop::taskAmoniaSensor, "taskAmoniaSensor", 2048 * 4, NULL, 1, NULL);
    // monitoring tasks
    if constexpr (chicken_coop::ENABLE_MONITORING)
    {
        xTaskCreate(debug_tools::taskStackMonitor, "taskStackMonitor", 4096, monitoredTasks, 1, &hStackMonTask); // task monitor
    }
    debug_tools::logMessage("Tasks created, watchdog armed!");
}

void loop()
{
    // Keep alive OTA wireless update process.
    common::ota::handle();
    delay(2);
}
