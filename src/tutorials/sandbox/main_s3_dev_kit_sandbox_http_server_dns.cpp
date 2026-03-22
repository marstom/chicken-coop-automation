#include <Arduino.h>
// #include <WiFi.h>
// #include <FreeRTOS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WebServer.h>

// Local
#include "wifi_conn.h"

#include "secrets.h"

// WiFiClient net;

WebServer server(80);
void taskHello(void *pvParameters);

void setup()
{
  Serial.begin(9600);
  // setup WIFI
  // http://tomroom/  < - it's working with my router!
  my::connect_to_wifi_with_wait(SSID_OFFICE, WIFI_PASS, "tomroom");

  server.on("/", []() {
    server.send(200, "text/plain", "<p>Hello from ESP32-S3 Dev Kit!</p><p>Go to <a href=\"/hello\">/hello</a> to see the message from FreeRTOS task.</p>");
  });
  server.begin();

  // xTaskCreate(taskHello, "taskHello", 4096, NULL, 1, NULL);
}

void loop() {
    server.handleClient();
}


void taskHello(void *pvParameters)
{
  while (1)
  {
    printf("This is sid and password: %s %s .\n", SSID_OFFICE, WIFI_PASS);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}