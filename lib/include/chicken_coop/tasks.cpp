#include <mqtt_comm.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <debug_tools.h>

#include "chicken_coop/tasks.h"
#include "chicken_coop/constants.h"
#include "chicken_coop/protocols.h"
#include "secrets.h"

PubSubClient client(net);

// MQTT loop task
void taskMQTT(void *pvParameters)
{
    esp_task_wdt_add(NULL); // watchdog
    for (;;)
    {
        client.loop(); // <--- processes incoming MQTT messages
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
    Wire.begin(D4, D5); // 6 7
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

void taskWebServer(void *pvParameters)
{
    for (;;)
    {
        webServer.handleClient();
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