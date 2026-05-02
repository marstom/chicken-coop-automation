
/*

This is door lock in my basement. BLE controlled plus WiFi controlled
*/

#include <stdarg.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include <WiFi.h>

#include <HTTPClient.h>
#include <PubSubClient.h>

// heap monitoring
#include "esp_heap_caps.h"

// BLE support
#include <ArduinoBLE.h>

// my libs
#include "common/wifi_conn/wifi_conn.h"
#include "mqtt_comm.h"
#include "debug_tools/debug_tools.h"
#include "secrets.h"

#include "relay_controller/constants.h"
#include "relay_controller/tasks.h"
#include "relay_controller/mqtt.h"
#include "relay_controller/ble.h"
#include "common/mqtt.h"
#include "common/ota.h"
#include "common/web.h"


// MQTT broker settings
const char *host = MQTT_HOST;
const uint16_t mqttPort = MQTT_PORT;

// PubSubClient client(net);

// --- Task handles (needed for stack monitoring) ---
TaskHandle_t hMQTTTask = NULL;
TaskHandle_t hBME280Task = NULL;
TaskHandle_t hStackMonTask = NULL;
TaskHandle_t hRelayTask = NULL;

/// make mqtt thread safe
void mycallback(char *topic, byte *message, unsigned int length);

// void taskBLE(void *pvParameters);
bool connectToMqttBroker();
// void onWiFiEvent(WiFiEvent_t event);

void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb
    common::connect_to_wifi_with_wait(SSID_OFFICE, WIFI_PASS, "basement");
    common::wifi_event::g_instance_name = "Relay Controller";
    common::wifi_event::g_mdns_hostname = "relay";
    WiFi.onEvent(common::wifi_event::onWiFiEvent);
    // ~chicken_coop~::setupMDNS(chicken_coop::MDNS_HOSTNAME); // 2modules needs this, then create common module for it
    common::setupMDNS(relay_controller::MDNS_HOSTNAME, "Relay Controller");
    debug_tools::logPrefix = relay_controller::PREFIX;

    pinMode(relay_controller::RELAY_PIN, OUTPUT); // RELAY_PIN as output
    digitalWrite(relay_controller::RELAY_PIN, HIGH);
    common::mqtt::setupAndConnect(
        relay_controller::client,
        host,
        mqttPort,
        mycallback,
        connectToMqttBroker,
        relay_controller::RELAY_1_SET_TOPIC,
        relay_controller::STATUS_TOPIC,
        "{\"message\": \"Initialized the connection from c3 relay controller\"}");

    debug_tools::logMessage("Initialize watchdog");
    esp_task_wdt_init(relay_controller::WDT_TIMEOUT, true);

    common::ota::setup("esp32c3"); // must match upload_port in platformio.ini

    communication::initQueue();
    // main mqtt task
    xTaskCreatePinnedToCore(relay_controller::taskMQTT, "taskMQTT", 2048 * 4, NULL, 1, &hMQTTTask, 0);
    // hardware sensors tasks
    xTaskCreate(relay_controller::taskRelay, "taskRelay", 4096, NULL, 1, &hRelayTask);

    // TODO move ble to ble.h
    relay_controller::setupBLE("Piwnica", "Drzwi w piwnicy");

    xTaskCreate(relay_controller::taskTcpServer, "taskTcpServer", 4096, NULL, 1, NULL);

    // monitoring tasks
    xTaskCreate(relay_controller::taskStackMonitor, "taskStackMonitor", 4096, NULL, 1, &hStackMonTask);
    debug_tools::logMessage("Tasks created, watchdog armed!");
}


void loop()
{
    // Keep alive OTA wireless update process.
    common::ota::handle();
}



bool connectToMqttBroker()
{
    if (MQTT_USER[0] != '\0')
    {
        return relay_controller::client.connect(relay_controller::THINGNAME, MQTT_USER, MQTT_PASS);
    }

    return relay_controller::client.connect(relay_controller::THINGNAME);
}

// callback triggers when a message is received
// Triggers on topic: "coop/relay/1/set"
// because it subscribes RELAY_1_SET_TOPIC
void mycallback(char *topic, byte *message, unsigned int length)
{
    String msgTemp = "";
    for (unsigned int i = 0; i < length; i++)
    {
        msgTemp += (char)message[i];
    }

    communication::RelayCommand cmd;
    cmd.on = (msgTemp == "ON");
    xQueueSend(communication::relayQueue, &cmd, 0); // queue is thread safe
}
