
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

// mDNS support
#include <ESPmDNS.h>

// my libs
#include "secrets.h"
#include "wifi_conn.h"
#include "mqtt_comm.h"
#include "debug_tools.h"
#include "gauge_page_chicken_coop.h"
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

void setupMDNS(const char *hostname);
void simpleWebPage();                   // for demo purposes that mDNS works
void taskWebServer(void *pvParameters); // simple web page for demo
void handleRootPage();                  // handle the page for above task
void handleJsonAPI();
void onWiFiEvent(WiFiEvent_t event);
bool connectToMqttBroker();

void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb
    WiFi.onEvent(onWiFiEvent);
    my::connect_to_wifi_with_wait(SSID_OFFICE, WIFI_PASS, "coop-automation"); // TODO later change to garden
    delay(1000);
    setupMDNS(MDNS_HOSTNAME);
    Serial.print("Adres to: http://");
    Serial.print(MDNS_HOSTNAME);
    Serial.println(".local");
    debug_tools::logPrefix = PREFIX;

    client.setServer(host, mqttPort);
    client.setCallback(mycallback);

    while (!client.connected())
    {
        if (connectToMqttBroker())
        {
            debug_tools::logMessage("☑ Connected to MQTT broker!");
            // Subscriptions here
            client.subscribe(RELAY_1_SET_TOPIC);
            ////////////
            communication::MqttMessage msg;
            msg.setContent(STATUS_TOPIC, "{\"message\": \"Initialized the connection from c3 relay controller\"}");
            msg.sendToQueue();
        }
        else
        {
            debug_tools::logMessage("✖ Failed to connect, try again in 2 seconds, rc=");
            debug_tools::logMessage("%d", client.state());
            delay(2000);
        }
    }

    debug_tools::logMessage("Initialize watchdog");
    esp_task_wdt_init(WDT_TIMEOUT, true);

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
    simpleWebPage(); // run server with web page at 80
    Serial.println("WebServer started");
    communication::initQueue();

    // main mqtt task
    xTaskCreatePinnedToCore(taskMQTT, "taskMQTT", 2048 * 4, NULL, 1, &hMQTTTask, 0);
    xTaskCreate(taskReadBME280, "taskReadBME280", 2048 * 4, NULL, 1, &hBME280Task); // temperature & pressure sensor
    xTaskCreate(taskWebServer, "taskWebServer", 4096 * 2, NULL, 1, &hWebServerTask);
// xTaskCreate(taskAmoniaSensor, "taskAmoniaSensor", 2048 * 4, NULL, 1, NULL);
// monitoring tasks
#ifdef ENABLE_MONITORING
    xTaskCreate(taskStackMonitor, "taskStackMonitor", 4096, NULL, 1, &hStackMonTask); // task monitor
#endif
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

bool connectToMqttBroker()
{
    if (MQTT_USER[0] != '\0')
    {
        return client.connect(THINGNAME, MQTT_USER, MQTT_PASS);
    }

    return client.connect(THINGNAME);
}

//////// utils functions

// set hostname for my chickenCOOP server
// https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32c3/api-reference/protocols/mdns.html
void setupMDNS(const char *hostname)
{

    // initialize mDNS service
    esp_err_t err = mdns_init();
    if (err)
    {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    // set hostname
    mdns_hostname_set(hostname);
    // set default instance
    mdns_instance_name_set("Chicken Coop");

    Serial.print("mDNS started: http://");
    Serial.print(hostname);
    Serial.println(".local");

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // add our services
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_add(NULL, "_ota", "_tcp", 3232, NULL, 0);
    mdns_service_add(NULL, "mqtt", "_tcp", 1883, NULL, 0);
}

void onWiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("WiFi connected, IP: ");
        Serial.println(WiFi.localIP());
        setupMDNS(MDNS_HOSTNAME);
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("WiFi disconnected, waiting for reconnect...");
        break;
    default:
        break;
    }
}

void simpleWebPage()
{
    webServer.on("/", handleRootPage);
    webServer.on("/api/v1/", handleJsonAPI);
    webServer.on("/health", []()
                 { webServer.send(200, "application/json", "{\"status\":\"ok\"}"); });
    webServer.on("/favicon.ico", []()
                 { webServer.send(200, "image/x-icon", ""); });
    webServer.onNotFound([]()
                         { webServer.send(404, "text/plain", "404: Not Found"); });
    webServer.begin();
}

String temp = "";
String press = "";
String hum = "";
String alt = "";

void readSensorsToStrings()
{
    char *webBuff = NULL;
    char *type = NULL;

    communication::WebMessage webMsg;
    while (xQueueReceive(communication::webQueue, &webMsg, 0) != pdFALSE)
    {
        webBuff = webMsg.getBuffer();
        type = webMsg.getMessageType();
        // String tt = String(type);
        bool exists = type != NULL && webBuff != NULL;
        if (exists && strcmp(type, communication::WebMessage::temperature) == 0)
        {
            temp = String(webBuff);
        }
        else if (exists && strcmp(type, communication::WebMessage::pressure) == 0)
        {
            press = String(webBuff);
        }
        else if (exists && strcmp(type, communication::WebMessage::altitude) == 0)
        {
            alt = String(webBuff);
        }

        else if (exists && strcmp(type, communication::WebMessage::humidity) == 0)
        {
            hum = String(webBuff);
        }
    }
}

void handleRootPage()
{
    readSensorsToStrings();

    webServer.send(200, "text/html", INDEX_HTML);
}

void handleJsonAPI()
{
    readSensorsToStrings();
    String json = "{";
    json += "\"temperature\":\"" + temp + "\",";
    json += "\"pressure\":\"" + press + "\",";
    json += "\"humidity\":\"" + hum + "\",";
    json += "\"altitude\":\"" + alt + "\"";
    json += "}";

    webServer.send(200, "application/json", json);
}
