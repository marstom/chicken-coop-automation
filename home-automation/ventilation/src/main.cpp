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
#include <my_am2320.h>

#include "secrets/secrets_local.h"

WebServer server(80);

void taskTemperature(void *pvParameters)
{
  while (true)
  {
    float temperature = my_am2320::measure_temperature();
    float humidity = my_am2320::measure_humidity();
    Serial.printf("Temperature: %.2f C, Humidity: %.2f %%\n", temperature, humidity);
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  common::connectToUartWithWait();
  wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation", true);

  my_am2320::init(GPIO_NUM_1, GPIO_NUM_2);
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

  xTaskCreate(taskTemperature, "Temperature Task", 2048, NULL, 1, NULL);

}


void loop()
{
  delay(10);
  server.handleClient();
}
