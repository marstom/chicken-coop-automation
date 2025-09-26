
#include <Arduino.h>
// #include <WiFi.h>
// #include <FreeRTOS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include <HTTPClient.h>
#include <PubSubClient.h>

// for BME
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "wifi_conn.h"
// #include "c3utils.h"

// Constants

// Addressable RGB LED, driven by GPIO48.
#define LED_PIN 48

// MQTT stuff
#define THINGNAME "esp32"
#define STATUS_TOPIC "status/read"
#define RELAY_1_SET_TOPIC "relay/1/set"
#define WDT_TIMEOUT 10 // 10 seconds

// BME temperature and humidity sensor, connected to i2c bus in current setup
#define BME_SCK D8
#define BME_MISO D9
#define BME_MOSI D10
#define BME_CS D7

#define BME_TEMPERATURE_TOPIC "bme280/temperature"
#define BME_PRESSURE_TOPIC "bme280/pressure"
#define BME_HUMIDITY_TOPIC "bme280/humidity"
#define BME_ALTITUDE_TOPIC "bme280/altitude"

#define I2C_SDA D4
#define I2C_SCL D5

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// My rasberry pi server name
const char *host = "raspberrypi.local";
WiFiClient net;
PubSubClient client(net);

// --- Task handles (needed for stack monitoring) ---
// TaskHandle_t hRelayTask = NULL;
TaskHandle_t hMQTTTask = NULL;
TaskHandle_t hBME280Task = NULL;
TaskHandle_t hStackMonTask = NULL;
TaskHandle_t hRelayTask = NULL;

// put function declarations here:
// void taskReceiveRelayCommand(void *pvParameters);
void mycallback(char *topic, byte *message, unsigned int length);
void taskMQTT(void *pvParameters); // Spin all the time and keep receiving the messages!
void taskReadBME280(void *pvParameters);
void taskStackMonitor(void *pvParameters); // debug stack monitor for memory usage
void relayActionTask(void *pv);

void setup()
{
    Serial.begin(9600);
    my::connect_to_wifi_with_wait();
    pinMode(D0, OUTPUT); // D0 as output
    digitalWrite(D0, HIGH);

    client.setServer(host, 1883); // rpi server
    client.setCallback(mycallback);

    while (!client.connected())
    {
        if (client.connect(THINGNAME))
        {
            Serial.println("☑ Connected to RPI Broker!");
            // Subscriptions here
            client.subscribe(RELAY_1_SET_TOPIC);
            ////////////
            client.publish(STATUS_TOPIC, "{\"message\": \"Initialized the connection from c3 relay controller\"}");
        }
        else
        {
            Serial.print("✖ Failed to connect, try again in 2 seconds, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }
    Serial.println("Initialize watchdog");
    esp_task_wdt_init(WDT_TIMEOUT, true);

    // xTaskCreatePinnedToCore(taskReceiveRelayCommand, "taskReceiveRelayCommand", 4096 * 4, NULL, 1, &hRelayTask, 0);
    xTaskCreatePinnedToCore(taskMQTT, "taskMQTT", 2048 * 1, NULL, 1, &hMQTTTask, 0);
    xTaskCreate(taskReadBME280, "taskReadBME280", 2048 * 2, NULL, 1, &hBME280Task);
    xTaskCreate(taskStackMonitor, "taskStackMonitor", 4096, NULL, 1, &hStackMonTask);
    Serial.println("Tasks created, watchdog armed!");
}

void loop() { /* Empty loop */ }

// MQTT loop task
void taskMQTT(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    for (;;)
    {
        client.loop(); // <--- processes incoming MQTT messages
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void relayActionTask(void *pv)
{
    // esp_task_wdt_add(NULL);
    String *msg = (String *)pv;
    Serial.printf("Async relay action: %s\n", msg->c_str());

    if (*msg == "ON")
    {
        digitalWrite(D0, LOW);
        // vTaskDelay(pdMS_TO_TICKS(6000));
        for (int i = 0; i < 60; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_task_wdt_reset();
        }
    }
    digitalWrite(D0, HIGH);
    // esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    delete msg;        // free heap
    vTaskDelete(NULL); // destroy task after finish
}

// the mqtt callback:
void mycallback(char *topic, byte *message, unsigned int length)
{
    // String messageTemp;
    // for (int i = 0; i < length; i++)
    // {
    //     messageTemp += (char)message[i];
    // }
    // Serial.println(messageTemp);

    // if (String(topic) == RELAY_1_SET_TOPIC)
    // {
    //     // in case of relay, the high state is OFF
    //     if (messageTemp == "ON")
    //     {
    //         digitalWrite(D0, HIGH);
    //     }
    //     else
    //     {
    //         digitalWrite(D0, LOW);
    //     }
    // }

    String *msg = new String();
    for (unsigned int i = 0; i < length; i++)
    {
        *msg += (char)message[i];
    }

    if (hRelayTask == NULL)
    {
        xTaskCreate(relayActionTask, "RelayAction", 4096, msg, 1, &hRelayTask);
    }
    else
    {
     Serial.println("Relay task already exists, skipping creation");   
    }
}

void taskReadBME280(void *pvParameters)
{
    esp_task_wdt_add(NULL);

    char buf[16];
    Wire.begin(D4, D5); // 6 7

    if (!bme.begin(0x76, &Wire))
    {
        if (!bme.begin(0x77, &Wire))
        {
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
            vTaskDelete(NULL); // delete task immediately if fails
        }
    }
    for (;;)
    {
        snprintf(buf, sizeof(buf), "%.2f", bme.readTemperature());
        client.publish(BME_TEMPERATURE_TOPIC, buf, true);

        snprintf(buf, sizeof(buf), "%.2f", bme.readPressure());
        client.publish(BME_PRESSURE_TOPIC, buf);

        snprintf(buf, sizeof(buf), "%.2f", bme.readAltitude(SEALEVELPRESSURE_HPA));
        client.publish(BME_ALTITUDE_TOPIC, buf);

        snprintf(buf, sizeof(buf), "%.2f", bme.readHumidity());
        client.publish(BME_HUMIDITY_TOPIC, buf); // add ,true to retain

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- New task: monitor stack usage ---
// debug monitoring
void taskStackMonitor(void *pvParameters)
{
    for (;;)
    {
        if (hMQTTTask)
        {
            Serial.printf("MQTTTask stack free: %u words (%u bytes)\n",
                          uxTaskGetStackHighWaterMark(hMQTTTask),
                          uxTaskGetStackHighWaterMark(hMQTTTask) * 4);
        }
        if (hBME280Task)
        {
            Serial.printf("BME280Task stack free: %u words (%u bytes)\n",
                          uxTaskGetStackHighWaterMark(hBME280Task),
                          uxTaskGetStackHighWaterMark(hBME280Task) * 4);
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // print every 10s
    }
}

/*
Results with print serial
RelayTask stack free: 16180 words (64720 bytes)
MQTTTask stack free: 15476 words (61904 bytes)
BME280Task stack free: 14576 words (58304 bytes)

MQTTTask stack free: 15476 words (61904 bytes)
BME280Task stack free: 14576 words (58304 bytes)
*/