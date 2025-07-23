#include <Arduino.h>
#include <WiFi.h>
#include <FreeRTOS.h>

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
  while (1)
  {
    // float cpuTemperature = temperatureRead();
    // printf("CPU temperature: %f\n", cpuTemperature);

    strip.setPixelColor(0, strip.Color(ledColor % 255, 56, ledColor % 255)); // green
    strip.show();
    printf("Led color: %d\n", ledColor);
    ledColor++;
    // delay(1000);

    vTaskDelay(pdMS_TO_TICKS(100));
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