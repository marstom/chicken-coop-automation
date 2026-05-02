
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

// Wireless code update and monitoring! Cool thing!
#include <ArduinoOTA.h>
#include <WiFiUdp.h>

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
    // TODO move mqtt to separate file common/mqtt.h / .cpp
    relay_controller::client.setServer(host, mqttPort);
    relay_controller::client.setCallback(mycallback);

    
    while (!relay_controller::client.connected())
    {
        if (connectToMqttBroker())
        {
            debug_tools::logMessage("☑ Connected to MQTT broker!");
            // Subscriptions here
            relay_controller::client.subscribe(relay_controller::RELAY_1_SET_TOPIC);
            ////////////
            communication::MqttMessage msg;
            msg.setContent(relay_controller::STATUS_TOPIC, "{\"message\": \"Initialized the connection from c3 relay controller\"}");
            msg.sendToQueue();
        }
        else
        {
            debug_tools::logMessage("✖ Failed to connect, try again in 2 seconds, rc=");
            debug_tools::logMessage("%d", relay_controller::client.state());
            delay(2000);
        }
    }
    /////////////////////////////
    debug_tools::logMessage("Initialize watchdog");
    esp_task_wdt_init(relay_controller::WDT_TIMEOUT, true);

    // TODO move mqtt to separate file
    // Init wireless updates
    debug_tools::logMessage("Initialize OTA updates via Wireless");
    debug_tools::logMessage("UPDATE VIA OTA");
    ////////move to ota file common/ota.h ota.cpp
    ArduinoOTA.setHostname("esp32c3"); // must match upload_port in platformio.ini
    ArduinoOTA
        .onStart([]()
                 { debug_tools::logMessage("OTA update start"); })
        .onEnd([]()
               { debug_tools::logMessage("\nOTA update end"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { debug_tools::logMessage("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 { debug_tools::logMessage("Error[%u]: ", error); });

    ArduinoOTA.begin();
    /////////////
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
    ArduinoOTA.handle();
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
