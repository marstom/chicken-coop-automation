#include "chicken_coop/sensor_listener.h"
#include "debug_tools/debug_tools.h"
#include <Wire.h>
#include <esp_task_wdt.h>
#include <mqtt_comm.h>

#include "chicken_coop/constants.h"
#include "chicken_coop/protocols.h"
#include "chicken_coop/tasks.h"
#include <RMaker.h>

// using TemperatureCallback = void (*)(float);

namespace chicken_coop
{

  PubSubClient client(chicken_coop::net);

  /// @brief Temperature sensor
  /// @param pvParameters
  void taskReadBME280(void* pvParameters)
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

  void taskAmoniaSensor(void* pvParameters)
  {
    esp_task_wdt_add(NULL);
    auto* listener = static_cast<chicken_coop::SensorListener*>(pvParameters);

    char buf[16];
    communication::MqttMessage msg;

    pinMode(AMONIA_SENSOR_PIN, INPUT);
    constexpr float VCC_MV = 3300.0f;
    constexpr float RL_KOHM = 4.7f;
    constexpr float R0_KOHM = 0.57f; // wartość R0 w kΩ TODO
    for (;;)
    {
      uint32_t sum = 0;
      for (int i = 0; i < 16; i++)
      {
        sum += analogReadMilliVolts(AMONIA_SENSOR_PIN);
        delay(5);
      }
      float mv = sum / 16.0f;
      float rs = RL_KOHM * (VCC_MV - mv) / mv; // kΩ, maleje gdy NH3 rośnie
      float ratio = R0_KOHM / rs;
      Serial.printf("NH3 sensor: mv=%.2f - zapisz RS R0_KOHM rs=%.2f\n", mv, rs, ratio);
      // ratio < 1.5 → OK
      // 1.5–2.5 → do obserwacji, przewietrzyć
      // > 2.5 → wysokie, wymienić ściółkę
      listener->onAmoniaSensor(ratio);
      // TODO calacualte amonia volume
      snprintf(buf, sizeof(buf), "%d", mv);
      msg.setContent(AMONIA_SENSOR_TOPIC, buf);
      msg.sendToQueue();

      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(AMONIA_SENSOR_READ_INTERVAL_MS));
    }
  }

  void taskWebServer(void* pvParameters)
  {
    for (;;)
    {
      chicken_coop::webServer.handleClient();
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
} // namespace chicken_coop
