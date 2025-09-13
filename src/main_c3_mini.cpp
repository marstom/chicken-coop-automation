#include <Arduino.h>
#include <WiFi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <HTTPClient.h>
#include <PubSubClient.h>

#include "wifi_conn.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


// Constants

// TODO: move to my library BME280
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
unsigned long delayTime;


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
void readTemperatureFromBME280();

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


void readTemperatureFromBME280(){
      if (! bme.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

  Serial.print("Temp: ");
  Serial.print(bme.readTemperature());
  Serial.print(" °C  Hum: ");
  Serial.print(bme.readHumidity());
  Serial.print(" %  Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
}