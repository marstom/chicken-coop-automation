
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

// my libs
#include "wifi_conn.h"
#include "mqtt_comm.h"
#include "debug_tools.h"


// Hardware feature toggle, comment out hardware which you don't need
// #define DEVICE_BME_280_ENABLED // enable temp & humidity & altitude & pressure sensor
#define DEVICE_RELAY_ENABLED // enable relay controll

// Addressable RGB LED, driven by GPIO48.
#define LED_PIN 48
#define WDT_TIMEOUT 10 // 10 seconds


#define RELAY_PIN D0

// BME temperature and humidity sensor, connected to i2c bus in current setup
#define BME_SCK D8
#define BME_MISO D9
#define BME_MOSI D10
#define BME_CS D7

// MQTT stuff
#define THINGNAME "esp32-c3-basement-fhs232y3a43"
#define PREFIX "basement/"
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
WiFiClient net;
PubSubClient client(net);

// telnet for print messages via wifi minitor
// WiFiServer telnetServer(23);
// WiFiClient telnetClient;

// --- Task handles (needed for stack monitoring) ---
TaskHandle_t hMQTTTask = NULL;
TaskHandle_t hBME280Task = NULL;
TaskHandle_t hStackMonTask = NULL;
TaskHandle_t hRelayTask = NULL;

/// make mqtt thread safe

void mycallback(char *topic, byte *message, unsigned int length);
void taskMQTT(void *pvParameters); // Spin all the time and keep receiving the messages!
void taskReadBME280(void *pvParameters);
void taskStackMonitor(void *pvParameters); // debug stack monitor for memory usage
void taskRelay(void *pvParameters);

void setup()
{
    Serial.begin(9600);
    my::connect_to_wifi_with_wait();
    debug_tools::logPrefix  = PREFIX;

    pinMode(RELAY_PIN, OUTPUT); // RELAY_PIN as output
    digitalWrite(RELAY_PIN, HIGH);

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
    /// telnet
    // telnetServer.begin();
    // telnetServer.setNoDelay(true);
    ////
    debug_tools::logMessage("Initialize watchdog");
    esp_task_wdt_init(WDT_TIMEOUT, true);

    // Init wireless updates
    debug_tools::logMessage("Initialize OTA updates via Wireless");
    debug_tools::logMessage("UPDATE VIA OTA");
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

    // xTaskCreatePinnedToCore(taskReceiveRelayCommand, "taskReceiveRelayCommand", 4096 * 4, NULL, 1, &hRelayTask, 0);
    communication::initQueue();
    // main mqtt task
    xTaskCreatePinnedToCore(taskMQTT, "taskMQTT", 2048 * 4, NULL, 1, &hMQTTTask, 0);
    #ifdef DEVICE_BME_280_ENABLED
    xTaskCreate(taskReadBME280, "taskReadBME280", 2048 * 4, NULL, 1, &hBME280Task);
    #endif
    // hardware sensors tasks
    #ifdef DEVICE_RELAY_ENABLED
    xTaskCreate(taskRelay, "taskRelay", 4096, NULL, 1, &hRelayTask);
    #endif
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

void taskRelay(void *pv)
{
    communication::RelayCommand cmd;
    for (;;)
    {
        if (xQueueReceive(communication::relayQueue, &cmd, portMAX_DELAY))
        {
            if (cmd.on)
            {
                digitalWrite(RELAY_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(6000));
                digitalWrite(RELAY_PIN, HIGH);
            }
            else
            {
                digitalWrite(RELAY_PIN, HIGH);
            }
        }
        // vTaskDelay(pdMS_TO_TICKS(100));
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

        debug_tools::printStackInfo("MQTTTask", hMQTTTask);
        debug_tools::printStackInfo("BME280Task", hBME280Task);
        debug_tools::printStackInfo("RelayTask", hRelayTask);
        debug_tools::printHeap(); // DEBUG memory leaks
        vTaskDelay(pdMS_TO_TICKS(5000)); // print every 10s
    }
}
