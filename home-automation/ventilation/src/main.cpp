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

  my_am2320::init_tca9548a(GPIO_NUM_1, GPIO_NUM_2);
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
  float temperature = 0.0;
  float humidity = 0.0;

  while (true)
  {
    temperature = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Intake);
    humidity = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Intake);
    Serial.printf("T Intake: %.2f C, H Intake: %.2f %%\n", temperature, humidity);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    temperature = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Exhaust);
    humidity = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Exhaust);
    Serial.printf("T Exhaust: %.2f C, H Exhaust: %.2f %%\n", temperature, humidity);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    temperature = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Room);
    humidity = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Room);
    Serial.printf("T Room: %.2f C, H Room: %.2f %%\n", temperature, humidity);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}