#include <mqtt_comm.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <WiFi.h>
#include "debug_tools/debug_tools.h"

#include "chicken_coop/tasks.h"
#include "chicken_coop/constants.h"
#include "chicken_coop/protocols.h"
#include "secrets.h"

extern TaskHandle_t hMQTTTask;
extern TaskHandle_t hBME280Task;

namespace chicken_coop
{

PubSubClient client(chicken_coop::net);

// MQTT loop task
void taskMQTT(void *pvParameters)
{
    esp_task_wdt_add(NULL); // watchdog
    TickType_t lastReconnectAttempt = 0;
    TickType_t lastWifiOk = xTaskGetTickCount();

    for (;;)
    {
        const TickType_t now = xTaskGetTickCount();
        if (WiFi.status() == WL_CONNECTED)
        {
            lastWifiOk = now;
            if (!client.connected())
            {
                if (now - lastReconnectAttempt >= pdMS_TO_TICKS(2000))
                {
                    lastReconnectAttempt = now;
                    if (chicken_coop::connectToMqttBroker())
                    {
                        client.subscribe(RELAY_1_SET_TOPIC);
                        debug_tools::logMessage("MQTT reconnected");
                    }
                }
            }
            else
            {
                client.loop(); // <--- processes incoming MQTT messages
            }
        }
        else if (now - lastWifiOk >= pdMS_TO_TICKS(30000))
        {
            // safety net: WiFi stuck down and no DISCONNECTED event recovered it
            debug_tools::logMessage("WiFi down >30s, forcing reconnect");
            WiFi.reconnect();
            lastWifiOk = now;
        }

        communication::MqttMessage msg;
        if (xQueueReceive(communication::mqttQueue, &msg, 0))
        {
            client.publish(msg.topic, msg.payload);
        }
        esp_task_wdt_reset(); // reset watchdog
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/// @brief Temperature sensor
/// @param pvParameters
void taskReadBME280(void *pvParameters)
{
    esp_task_wdt_add(NULL);

    char buf[16];
    Wire.begin(I2C_SDA, I2C_SCL); // 6 7
    communication::MqttMessage msg;
    communication::WebMessage webMsg;

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
        debug_tools::logMessage("This is the buf content: %s", buf);
        Serial.println(buf);
        msg.setContent(BME_TEMPERATURE_TOPIC, buf);
        msg.sendToQueue();
        webMsg.setContent(communication::WebMessage::temperature, buf);
        webMsg.sendToQueue();

        snprintf(buf, sizeof(buf), "%.2f", bme.readPressure());
        Serial.println(buf);
        msg.setContent(BME_PRESSURE_TOPIC, buf);
        // webMsg.setContent(BME_PRESSURE_TOPIC, buf);
        msg.sendToQueue();
        webMsg.setContent(communication::WebMessage::pressure, buf);
        webMsg.sendToQueue();

        snprintf(buf, sizeof(buf), "%.2f", bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(buf);
        msg.setContent(BME_ALTITUDE_TOPIC, buf);
        msg.sendToQueue();
        webMsg.setContent(communication::WebMessage::altitude, buf);
        webMsg.sendToQueue();

        snprintf(buf, sizeof(buf), "%.2f", bme.readHumidity());
        Serial.println(buf);
        msg.setContent(BME_HUMIDITY_TOPIC, buf);
        msg.sendToQueue();
        webMsg.setContent(communication::WebMessage::humidity, buf);
        webMsg.sendToQueue();

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskAmoniaSensor(void *pvParameters){
    esp_task_wdt_add(NULL);

    char buf[16];
    communication::MqttMessage msg;

    pinMode(AMONIA_SENSOR_PIN, INPUT);

    for (;;)
    {
        const int raw = analogRead(AMONIA_SENSOR_PIN);
        snprintf(buf, sizeof(buf), "%d", raw);
        msg.setContent(AMONIA_SENSOR_TOPIC, buf);
        msg.sendToQueue();

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(AMONIA_SENSOR_READ_INTERVAL_MS));
    }
}


// --- New task: device monitoring, for troubleshooting ---
void taskStackMonitor(void *pvParameters)
{
    for (;;)
    {
        if constexpr (ENABLE_MONITORING)
        {
            debug_tools::printStackInfo("MQTTTask", ::hMQTTTask);
            debug_tools::printStackInfo("BME280Task", ::hBME280Task);
            debug_tools::printHeap(); // DEBUG memory leaks
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // print every 10s
    }
}

void taskWebServer(void *pvParameters)
{
    for (;;)
    {
        chicken_coop::webServer.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool connectToMqttBroker()
{
    if (MQTT_USER[0] != '\0')
    {
        return client.connect(THINGNAME, MQTT_USER, MQTT_PASS);
    }

    return client.connect(THINGNAME);
}
}
