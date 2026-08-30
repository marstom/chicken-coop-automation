#include <Arduino.h>
// #include <WiFi.h>
// #include <FreeRTOS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WebServer.h>

//TODO USEMNDS
#include <ESPmDNS.h>

// Local
#include "wifi_conn.h"

#include "secrets.h"

// WiFiClient net;

WebServer server(80);
void taskHello(void *pvParameters);

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<link rel=\"icon\" href=\"data:,\">";
  html += "<style>";
  html += "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: white; color: #333; margin: 20px; }";
  html += "h1 { color: #4a5568; font-weight: 300; }";
  html += "</style></head>";
  html += "<body>";
  html += "<h1>Hello World</h1>";
  html += "<p>ESP32 mDNS Demo.</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}


// snippent
void setupMDNS(const char* hostname) {
    if (!MDNS.begin(hostname)) {
        Serial.println("mDNS failed to start");
        while(1) {
            delay(1000);
        }
    }
}

void setup()
{
  Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb
  Serial.println("Serial started");

  // setup WIFI
  // http://tomroom/  < - it's working with my router!
  my::connect_to_wifi_with_wait(SSID_OFFICE, WIFI_PASS, "tomroom");

  ///https://randomnerdtutorials.com/esp32-mdns-arduino/
  // mdns service
  Serial.println("Setting up MDNS responder...");
  setupMDNS("dom");
  MDNS.addService("_http", "_tcp", 80);
  MDNS.addService("ota", "tcp", 3232); // OTA updates

  server.on("/", handleRoot);
  server.begin();
  Serial.println("Server started");

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