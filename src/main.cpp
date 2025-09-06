#include <Arduino.h>
#include <WiFi.h>
// #include <FreeRTOS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

#include <HTTPClient.h>
#include <PubSubClient.h>

#include <Adafruit_NeoPixel.h>

#include "wifi_conn.h"

// Constants

// Addressable RGB LED, driven by GPIO48.
#define LED_PIN 48

// MQTT stuff
#define THINGNAME "esp32"
#define TEMPERATURE_TOPIC "temperature/read"

// My rasberry pi server name
const char *host = "raspberrypi.local";
WiFiClient net;
PubSubClient client(net);

// LED stuff
#define PIN 48
#define NUMPIXELS 1
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint32_t ledColor = 0;

// put function declarations here:
void taskHandleRGBLed(void *pvParameters);
void taskReadTemperature(void *pvParameters);

void setup()
{
  Serial.begin(9600);
  // Setup LED
  strip.begin();
  strip.setBrightness(50); // 0–255
  strip.show();

  // setup WIFI
  my::connect_to_wifi_with_wait();

  client.setServer(host, 1883);
  while (!client.connected())
  {
    if (client.connect(THINGNAME))
    {
      Serial.println("☑ Connected to AWS IoT");

      Serial.println("☑ Connected!");
      client.publish(TEMPERATURE_TOPIC, "{\"message\": \"Initialized the connection\"}");
      // Create a message handler
      // client.setCallback(messageHandler);
      // client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
      // start task freeRTOS
      xTaskCreate(taskReadTemperature, "taskReadTemperature", 2048 * 4, NULL, 1, NULL);
    }
    else
    {
      Serial.print("✖ AWS connection failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
  xTaskCreatePinnedToCore(taskHandleRGBLed, "taskHandleRGBLed", 4096 * 4, NULL, 1, NULL, 0);
}

void loop() {}

void taskHandleRGBLed(void *pvParameters)
{
  uint16_t hue = 0;          // 0–65535 in Adafruit helper
  const uint16_t step = 256; // how fast to move (≈1°/frame)
  for (;;)
  {
    // For a single LED
    uint32_t color = strip.gamma32(strip.ColorHSV(hue, 255, 255));
    strip.setPixelColor(0, color);

    strip.show();
    hue += step;                   // wrap automatically on overflow
    vTaskDelay(pdMS_TO_TICKS(20)); // ~50 FPS
  }
}

void taskReadTemperature(void *pvParameters)
{
  while (1)
  {
    float cpuTemperature = temperatureRead();
    printf("CPU temperature: %f\n", cpuTemperature);
    String payload = "{\"message\": \"" + String(cpuTemperature) + "\"}";
    client.publish(TEMPERATURE_TOPIC, payload.c_str());

    vTaskDelay(pdMS_TO_TICKS(1200));
  }
}