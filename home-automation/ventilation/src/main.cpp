#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// local libs
#include <wifi_conn.h>
#include <wifi_mdns.h>
#include <uart_utils.h>
#include <my_am_2320_tca9548a.h>

#include "secrets/secrets_local.h"

WebServer server(80);

void taskTemperature(void *pvParameters);

void setup()
{
  common::connectToUartWithWait();
  wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation", true);

  my_am2320::init_wire(GPIO_NUM_10, GPIO_NUM_11);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Intake);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Exhaust);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Room);
  Serial.println("Ventilation controller started");

  wifi::setupMdns("ventilation");
  // if (MDNS.begin("ventilation"))
  // {
  //   Serial.println("mDNS: ventilation.local");
  // }
  // MDNS.addService("http", "tcp", 80);
  // MDNS.addService("mqtt", "tcp", 1883);
  // MDNS.addService("ota", "tcp", 3232);

  server.on("/", []() {
      server.send(200, "text/plain", "Hello");
  });
  server.begin();

  xTaskCreate(taskTemperature, "Temperature Task", 2048*4, NULL, 1, NULL);

}


void loop()
{
  delay(10);
  server.handleClient();
}


void taskTemperature(void *pvParameters)
{
  float temperature_intake = 0.0;
  float humidity_intake = 0.0;
  float temperature_exhaust = 0.0;
  float humidity_exhaust = 0.0;
  float temperature_room = 0.0;
  float humidity_room = 0.0;

  while (true)
  {
    temperature_intake = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Intake);
    humidity_intake = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Intake);
    Serial.printf("T Intake: %.2f C, H Intake: %.2f %%\n", temperature_intake, humidity_intake);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    temperature_intake = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Exhaust);
    humidity_exhaust = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Exhaust);
    Serial.printf("T Exhaust: %.2f C, H Exhaust: %.2f %%\n", temperature_exhaust, humidity_exhaust);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    temperature_intake = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Room);
    humidity_room = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Room);
    Serial.printf("T Room: %.2f C, H Room: %.2f %%\n", temperature_room, humidity_room);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // 
    /*
    Build JSON response:
    {
      "temperature": {
        "intake": temperature,
        "exhaust": temperature,
        "room": temperature
      },
      "humidity": {
        "intake": humidity,
        "exhaust": humidity,
        "room": humidity
    }
    */
    String jsonResponse = "{\"temperature\":{\"intake\":";
    jsonResponse += temperature_intake;
    jsonResponse += ",\"exhaust\":";
    jsonResponse += temperature_intake;
    jsonResponse += ",\"room\":";
    jsonResponse += temperature_intake;
    jsonResponse += "},\"humidity\":{\"intake\":";
    jsonResponse += humidity_intake;
    jsonResponse += ",\"exhaust\":";
    jsonResponse += humidity_exhaust;
    jsonResponse += ",\"room\":";
    jsonResponse += humidity_room;
    jsonResponse += "}}";
    server.send(200, "application/json", jsonResponse);
}


}



String buildJsonResponse(float temperature_intake, float humidity_intake, float temperature_exhaust, float humidity_exhaust, float temperature_room, float humidity_room)
{
  String jsonResponse = "{\"temperature\":{\"intake\":";
  jsonResponse += temperature_intake;
  jsonResponse += ",\"exhaust\":";
  jsonResponse += temperature_intake;
  jsonResponse += ",\"room\":";
  jsonResponse += temperature_intake;
  jsonResponse += "},\"humidity\":{\"intake\":";
  jsonResponse += humidity_intake;
  jsonResponse += ",\"exhaust\":";
  jsonResponse += humidity_exhaust;
  jsonResponse += ",\"room\":";
  jsonResponse += humidity_room;
  jsonResponse += "}}";
  return jsonResponse;
}