#include <mqtt_comm.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include "debug_tools/debug_tools.h"
#include "chicken_coop/sensor_listener.h"

#include "chicken_coop/tasks.h"
#include "chicken_coop/constants.h"
#include "chicken_coop/protocols.h"
#include <RMaker.h>

// using TemperatureCallback = void (*)(float);

namespace chicken_coop
{


PubSubClient client(chicken_coop::net);

/// @brief Temperature sensor
/// @param pvParameters
void taskReadBME280(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    auto* listener = static_cast<chicken_coop::SensorListener*>(pvParameters);
    // auto onTemp = reinterpret_cast<TemperatureCallback>(pvParameters);  // może być nullptr

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
        float t = bme.readTemperature();
        listener->onTemperature(t);
        // if (onTemp) onTemp(t); // if function exists, update rainMaker
        snprintf(buf, sizeof(buf), "%.2f", t);
        debug_tools::logMessage("This is the buf content: %s", buf);
        Serial.println(buf);
        msg.setContent(BME_TEMPERATURE_TOPIC, buf);
        msg.sendToQueue();
        webMsg.setContent(communication::WebMessage::temperature, buf);
        webMsg.sendToQueue();


        float p = bme.readPressure();
        snprintf(buf, sizeof(buf), "%.2f", p);
        listener->onPressure(p);
        Serial.println(buf);
        msg.setContent(BME_PRESSURE_TOPIC, buf);
        // webMsg.setContent(BME_PRESSURE_TOPIC, buf);
        msg.sendToQueue();
        webMsg.setContent(communication::WebMessage::pressure, buf);
        webMsg.sendToQueue();


        float h = bme.readHumidity();
        snprintf(buf, sizeof(buf), "%.2f", h);
        listener->onHumidity(h);
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


void taskWebServer(void *pvParameters)
{
    for (;;)
    {
        chicken_coop::webServer.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}
