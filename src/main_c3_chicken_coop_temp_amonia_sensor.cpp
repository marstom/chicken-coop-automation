
/*

This is chicken coop monitor.
*/

#include <stdarg.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include <HTTPClient.h>
#include <PubSubClient.h>

// for BME
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

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

// Hardware feature toggle, comment out hardware which you don't need
#define DEVICE_BME_280_ENABLED       // enable bme280 temp/humidity sensor
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

// MQTT stuff
#define THINGNAME "esp32-c3-coop-temp-amonia-sensor-k31hfaie"
#define PREFIX "coop/"
#define BME_TEMPERATURE_TOPIC PREFIX "bme280/temperature"
#define BME_TEMPERATURE_TOPIC PREFIX "bme280/temperature"
#define BME_PRESSURE_TOPIC PREFIX "bme280/pressure"
#define BME_HUMIDITY_TOPIC PREFIX "bme280/humidity"
#define BME_ALTITUDE_TOPIC PREFIX "bme280/altitude"
#define MQTT_LOG_TOPIC PREFIX "log/mydebug"
#define STATUS_TOPIC PREFIX "status/read"
#define RELAY_1_SET_TOPIC PREFIX "relay/1/set"

#define I2C_SDA D4
#define I2C_SCL D5

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// My rasberry pi server name
const char *host = "raspberrypi.local";
//TODO change WIFI access point outside !
WiFiClient net;

PubSubClient client(net);

// --- Task handles (needed for stack monitoring) ---
TaskHandle_t hMQTTTask = NULL;
TaskHandle_t hBME280Task = NULL;
TaskHandle_t hStackMonTask = NULL;

/// make mqtt thread safe
void mycallback(char *topic, byte *message, unsigned int length);
void taskMQTT(void *pvParameters); // Spin all the time and keep receiving the messages!
void taskReadBME280(void *pvParameters);
void taskStackMonitor(void *pvParameters); // debug stack monitor for memory usage
// void taskRelay(void *pvParameters);
void tcpServerTask(void *pvParameters); // direct connection


void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb
    my::connect_to_wifi_with_wait(SSID_OFFICE, WIFI_PASS, "coop-automation"); // TODO later change to garden
    debug_tools::logPrefix = PREFIX;

    client.setServer(host, 1883); // rpi server
    client.setCallback(mycallback);

    while (!client.connected())
    {
        if (client.connect(THINGNAME))
        {
            debug_tools::logMessage("☑ Connected to RPI Broker!");
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
    ArduinoOTA.setHostname("esp32c3"); // must match upload_port in platformio.ini
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

    communication::initQueue();
    // main mqtt task
    xTaskCreatePinnedToCore(taskMQTT, "taskMQTT", 2048 * 4, NULL, 1, &hMQTTTask, 0);
    xTaskCreate(taskReadBME280, "taskReadBME280", 2048 * 4, NULL, 1, &hBME280Task);
    // monitoring tasks
    xTaskCreate(taskStackMonitor, "taskStackMonitor", 4096, NULL, 1, &hStackMonTask);
    debug_tools::logMessage("Tasks created, watchdog armed!");
}

void loop()
{
    // Keep alive OTA wireless update process.
    ArduinoOTA.handle();
}

// MQTT loop task
void taskMQTT(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    for (;;)
    {
        client.loop(); // <--- processes incoming MQTT messages
        communication::MqttMessage msg;
        if (xQueueReceive(communication::mqttQueue, &msg, 0))
        {
            client.publish(msg.topic, msg.payload);
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


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

/// @brief Temperature sensor
/// @param pvParameters 
void taskReadBME280(void *pvParameters)
{
    esp_task_wdt_add(NULL);

    char buf[16];
    Wire.begin(D4, D5); // 6 7
    communication::MqttMessage msg;

    if (!bme.begin(0x76, &Wire))
    {
        if (!bme.begin(0x77, &Wire))
        {
            debug_tools::logMessage("Could not find a valid BME280 sensor, check wiring!");
            vTaskDelete(NULL); // delete task immediately if fails
        }
    }
    for (;;)
    {
        snprintf(buf, sizeof(buf), "%.2f", bme.readTemperature());
        msg.setContent(BME_TEMPERATURE_TOPIC, buf);
        msg.sendToQueue();

        snprintf(buf, sizeof(buf), "%.2f", bme.readPressure());
        msg.setContent(BME_PRESSURE_TOPIC, buf);
        msg.sendToQueue();

        snprintf(buf, sizeof(buf), "%.2f", bme.readAltitude(SEALEVELPRESSURE_HPA));
        msg.setContent(BME_ALTITUDE_TOPIC, buf);
        msg.sendToQueue();

        snprintf(buf, sizeof(buf), "%.2f", bme.readHumidity());
        msg.setContent(BME_HUMIDITY_TOPIC, buf);
        msg.sendToQueue();

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




// --- New task: device monitoring, for troubleshooting ---
void taskStackMonitor(void *pvParameters)
{
    for (;;)
    {
#ifdef ENABLE_MONITORING
        debug_tools::printStackInfo("MQTTTask", hMQTTTask);
        debug_tools::printStackInfo("BME280Task", hBME280Task);
        debug_tools::printHeap(); // DEBUG memory leaks
#endif
        vTaskDelay(pdMS_TO_TICKS(5000)); // print every 10s
    }
}
