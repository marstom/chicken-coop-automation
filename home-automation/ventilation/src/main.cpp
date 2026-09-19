#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// local libs
#include <my_am_2320_tca9548a.h>
#include <uart_utils.h>
#include <wifi_conn.h>
#include <wifi_mdns.h>

#include "secrets/secrets_local.h"

WebServer server(80);

// Written by taskTemperature, read by the /api handler. Aligned 32-bit floats,
// so a reader sees either the old or the new value, never a torn one.
float temperature_intake = NAN;
float humidity_intake = NAN;
float temperature_exhaust = NAN;
float humidity_exhaust = NAN;
float temperature_room = NAN;
float humidity_room = NAN;

void taskTemperature(void* pvParameters);

String buildJsonResponse();

void setup()
{
  common::connectToUartWithWait();
  wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation", true);

  my_am2320::init_wire(GPIO_NUM_1, GPIO_NUM_2);
  delay(1000);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Intake);
  delay(1000);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Exhaust);
  delay(1000);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Room);
  delay(1000);
  Serial.println("Ventilation controller started");

  wifi::setupMdns("ventilation");

  server.on("/", []() { server.send(200, "text/plain", "Hello"); });
  server.on("/api", []() { server.send(200, "application/json", buildJsonResponse()); });
  server.begin();

  xTaskCreate(taskTemperature, "Temperature Task", 2048 * 4, NULL, 1, NULL);
}

void loop()
{
  delay(10);
  server.handleClient();
}

void taskTemperature(void* pvParameters)
{
  while (true)
  {
    temperature_intake = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Intake);
    humidity_intake = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Intake);
    Serial.printf("T Intake: %.2f C, H Intake: %.2f %%\n", temperature_intake, humidity_intake);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    temperature_exhaust =
        my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Exhaust);
    humidity_exhaust = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Exhaust);
    Serial.printf("T Exhaust: %.2f C, H Exhaust: %.2f %%\n", temperature_exhaust, humidity_exhaust);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    temperature_room = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Room);
    humidity_room = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Room);
    Serial.printf("T Room: %.2f C, H Room: %.2f %%\n", temperature_room, humidity_room);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

// A failed sensor read is NAN, which is not valid JSON, so it goes out as null.
static void appendReading(String& json, float value)
{
  if (isnan(value))
  {
    json += "null";
  }
  else
  {
    json += String(value, 2);
  }
}

/*
{
  "temperature": { "intake": 23.10, "exhaust": null, "room": null },
  "humidity":    { "intake": 65.70, "exhaust": null, "room": null }
}
*/
String buildJsonResponse()
{
  String json = "{\"temperature\":{\"intake\":";
  appendReading(json, temperature_intake);
  json += ",\"exhaust\":";
  appendReading(json, temperature_exhaust);
  json += ",\"room\":";
  appendReading(json, temperature_room);
  json += "},\"humidity\":{\"intake\":";
  appendReading(json, humidity_intake);
  json += ",\"exhaust\":";
  appendReading(json, humidity_exhaust);
  json += ",\"room\":";
  appendReading(json, humidity_room);
  json += "}}";
  return json;
}
