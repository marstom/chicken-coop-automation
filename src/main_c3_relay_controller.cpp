
/*

This is door lock in my basement. BLE controlled plus WiFi controlled
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include <WiFi.h>

// my libs
#include "common/wifi_conn/wifi_conn.h"
#include "mqtt_comm.h"
#include "debug_tools/debug_tools.h"
#include "secrets.h"

#include "relay_controller/constants.h"
#include "relay_controller/tasks.h"
#include "relay_controller/ble.h"
#include "common/mqtt.h"
#include "common/ota.h"
#include "common/web.h"

// MQTT broker settings
const char *host = MQTT_HOST;
const uint16_t mqttPort = MQTT_PORT;

// --- Task handles (needed for stack monitoring) ---
TaskHandle_t hMQTTTask = NULL;
TaskHandle_t hStackMonTask = NULL;
TaskHandle_t hRelayTask = NULL;

/// make mqtt thread safe
void mycallback(char *topic, byte *message, unsigned int length);

// MQTT service task configuration; static lifetime, the task keeps a pointer.
static common::mqtt::TaskConfig mqttConfig = {
    &relay_controller::client,
    MQTT_USER,
    MQTT_PASS,
    relay_controller::THINGNAME,
    relay_controller::RELAY_1_SET_TOPIC,
    relay_controller::STATUS_TOPIC,
    "{\"message\": \"Initialized the connection from c3 relay controller\"}",
};

// Tasks watched by the stack monitor (terminated by {nullptr, nullptr}).
static debug_tools::MonitoredTask monitoredTasks[] = {
    {"MQTTTask", &hMQTTTask},
    {"RelayTask", &hRelayTask},
    {nullptr, nullptr},
};

void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb
    debug_tools::logPrefix = relay_controller::PREFIX;

    // Modem sleep must stay enabled: this target runs BLE alongside WiFi and
    // the C3 coexistence layer aborts at BLE.begin() when sleep is disabled.
    common::connectToWifiWithWait(SSID_OFFICE, WIFI_PASS, "basement", /*disableModemSleep=*/false);
    common::wifi_event::g_instance_name = "Relay Controller";
    common::wifi_event::g_mdns_hostname = relay_controller::MDNS_HOSTNAME;
    // Register AFTER the initial connect: failed connect attempts during boot
    // also fire DISCONNECTED events.
    WiFi.onEvent(common::wifi_event::onWiFiEvent);
    common::setupMDNS(relay_controller::MDNS_HOSTNAME, "Relay Controller");

    pinMode(relay_controller::RELAY_PIN, OUTPUT); // RELAY_PIN as output
    digitalWrite(relay_controller::RELAY_PIN, HIGH);

    // Broker connection is handled entirely by the MQTT task, so a dead
    // broker at boot no longer blocks OTA / BLE / the door.
    relay_controller::client.setServer(host, mqttPort);
    relay_controller::client.setCallback(mycallback);

    debug_tools::logMessage("Initialize watchdog");
    esp_task_wdt_init(relay_controller::WDT_TIMEOUT, true);

    common::ota::setup(relay_controller::MDNS_HOSTNAME); // must match upload_port in platformio.ini

    communication::initQueue();
    // main mqtt task
    xTaskCreatePinnedToCore(common::mqtt::taskMQTT, "taskMQTT", 2048 * 4, &mqttConfig, 1, &hMQTTTask, 0);
    // hardware sensors tasks
    xTaskCreate(relay_controller::taskRelay, "taskRelay", 4096, NULL, 1, &hRelayTask);

    relay_controller::setupBLE("Piwnica", "Drzwi w piwnicy");

    xTaskCreate(relay_controller::taskTcpServer, "taskTcpServer", 4096, NULL, 1, NULL);

    // monitoring tasks
    if constexpr (relay_controller::ENABLE_MONITORING)
    {
        xTaskCreate(debug_tools::taskStackMonitor, "taskStackMonitor", 4096, monitoredTasks, 1, &hStackMonTask);
    }
    debug_tools::logMessage("Tasks created, watchdog armed!");
}


void loop()
{
    // Keep alive OTA wireless update process.
    common::ota::handle();
}


///////mq callbacks
// callback triggers when a message is received
// Triggers on topic: "basement/relay/1/set"
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
