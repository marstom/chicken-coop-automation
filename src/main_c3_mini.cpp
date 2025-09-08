#include <Arduino.h>
#include <WiFi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <HTTPClient.h>
#include <PubSubClient.h>


#include "wifi_conn.h"

// Constants

// Addressable RGB LED, driven by GPIO48.
#define LED_PIN 48

// MQTT stuff
#define THINGNAME "esp32"
#define TEMPERATURE_TOPIC "temperature/read"

// My rasberry pi server name
const char *host = "raspberr ypi.local";
WiFiClient net;
PubSubClient client(net);


// put function declarations here:
void taskHandleRGBLed(void *pvParameters);
void taskReadTemperature(void *pvParameters);

void setup()
{
  Serial.begin(9600);

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
}

void loop() {}

void taskReadTemperature(void *pvParameters)
{
  while (1)
  {
    float cpuTemperature = temperatureRead();
    printf("CPU temperature HIH: %f\n", cpuTemperature);
    String payload = "{\"message\": \"" + String(cpuTemperature) + "\"}";
    client.publish(TEMPERATURE_TOPIC, payload.c_str());

    vTaskDelay(pdMS_TO_TICKS(1200));
  }
}