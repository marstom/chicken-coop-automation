
/*

This is chicken coop monitor.

List of all mdns devices is:
dns-sd -B _http._tcp

*/

#include <stdarg.h>
#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include <HTTPClient.h>

#include <WebServer.h>

// heap monitoring
#include "esp_heap_caps.h"

// Wireless code update and monitoring! Cool thing!
#include <ArduinoOTA.h>
#include <WiFiUdp.h>

// BLE support
#include <ArduinoBLE.h>

// my libs
#include "secrets.h"
#include "wifi_conn.h"
#include "mqtt_comm.h"
#include "debug_tools.h"
#include "chicken_coop/tasks.h"
#include "chicken_coop/constants.h"
#include "chicken_coop/web.h"

// MQTT broker settings
const char *host = MQTT_HOST;
const uint16_t mqttPort = MQTT_PORT;
// TODO change WIFI access point outside !

// --- Task handles (needed for stack monitoring) ---
TaskHandle_t hMQTTTask = NULL;
TaskHandle_t hBME280Task = NULL;
TaskHandle_t hStackMonTask = NULL;
TaskHandle_t hWebServerTask = NULL;

/// make mqtt thread safe
void mycallback(char *topic, byte *message, unsigned int length);

void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb
    WiFi.onEvent(chicken_coop::onWiFiEvent);
    my::connect_to_wifi_with_wait(SSID_OFFICE, WIFI_PASS, "coop-automation"); // TODO later change to garden SSID_OFFICE SSID_GARDEN
    delay(1000);
    chicken_coop::setupMDNS(chicken_coop::MDNS_HOSTNAME);
    Serial.print("Adres to: http://");
    Serial.print(chicken_coop::MDNS_HOSTNAME);
    Serial.println(".local");
    debug_tools::logPrefix = chicken_coop::PREFIX;

    chicken_coop::client.setServer(host, mqttPort);
    chicken_coop::client.setCallback(mycallback);

    while (!chicken_coop::client.connected())
    {
        if (chicken_coop::connectToMqttBroker())
        {
            debug_tools::logMessage("☑ Connected to MQTT broker!");
            // Subscriptions here
            chicken_coop::client.subscribe(chicken_coop::RELAY_1_SET_TOPIC);
            ////////////
            communication::MqttMessage msg;
            msg.setContent(chicken_coop::STATUS_TOPIC, "{\"message\": \"Initialized the connection from c3 relay controller\"}");
            msg.sendToQueue();
        }
        else
        {
            debug_tools::logMessage("✖ Failed to connect, try again in 2 seconds, rc=");
            debug_tools::logMessage("%d", chicken_coop::client.state());
            delay(2000);
        }
    }

    debug_tools::logMessage("Initialize watchdog");
    esp_task_wdt_init(chicken_coop::WDT_TIMEOUT, true);

    // Init wireless updates
    debug_tools::logMessage("Initialize OTA updates via Wireless");
    debug_tools::logMessage("UPDATE VIA OTA");
    ArduinoOTA.setHostname("chicken"); // must match upload_port in platformio.ini
    // remote code upload
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
    chicken_coop::simpleWebPage(); // run server with web page at 80
    Serial.println("WebServer started");
    communication::initQueue();

    // main mqtt task
    xTaskCreatePinnedToCore(chicken_coop::taskMQTT, "taskMQTT", 2048 * 4, NULL, 1, &hMQTTTask, 0);
    xTaskCreate(chicken_coop::taskReadBME280, "taskReadBME280", 2048 * 4, NULL, 1, &hBME280Task); // temperature & pressure sensor
    xTaskCreate(chicken_coop::taskWebServer, "taskWebServer", 4096 * 2, NULL, 1, &hWebServerTask);
    xTaskCreate(chicken_coop::taskAmoniaSensor, "taskAmoniaSensor", 2048 * 4, NULL, 1, NULL);
    // monitoring tasks
    if constexpr (chicken_coop::ENABLE_MONITORING)
    {
        xTaskCreate(chicken_coop::taskStackMonitor, "taskStackMonitor", 4096, NULL, 1, &hStackMonTask); // task monitor
    }
    debug_tools::logMessage("Tasks created, watchdog armed!");
}

void loop()
{
    // Keep alive OTA wireless update process.
    ArduinoOTA.handle();
    // webServer.handleClient();
    delay(2);
}

///////////////////// Tasks


// callback triggers when a message is received
// Triggers on topic: "coop/relay/1/set"
// because it subscribes RELAY_1_SET_TOPIC
void mycallback(char *topic, byte *message, unsigned int length)
{

    Serial.println("MQTT Message arrived");
    Serial.println("Message arrived to MQTT topic: " + String(topic));
    String msgTemp = "";
    for (unsigned int i = 0; i < length; i++)
    {
        msgTemp += (char)message[i];
    }

    communication::RelayCommand cmd;
    cmd.on = (msgTemp == "ON");
    xQueueSend(communication::relayQueue, &cmd, 0); // queue is thread safe
}

//////// utils functions
