#include <Arduino.h>
// #include <WiFi.h>
// #include <FreeRTOS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Local
#include "wifi_conn.h"

#include "secrets.h"

// WiFiClient net;


void taskHello(void *pvParameters);

void setup()
{
  Serial.begin(9600);

  // setup WIFI
  my::connect_to_wifi_with_wait(SSID_OFFICE, WIFI_PASS);
  xTaskCreate(taskHello, "taskHello", 4096, NULL, 1, NULL);
}

void loop() {}


void taskHello(void *pvParameters)
{
  while (1)
  {
    printf("This is sid and password: %s %s .\n", SSID_OFFICE, WIFI_PASS);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}