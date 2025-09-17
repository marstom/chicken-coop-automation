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
#define BME_280_TEMPERATURE_TOPIC "bme280/temperature/read"
#define BME_280_PRESSURE_TOPIC "bme280/pressure/read"
#define BME_280_HUMIDITY_TOPIC "bme280/humidity/read"
#define RELAY_1_TOPIC_R "relay/1/read"
#define RELAY_2_TOPIC_R "relay/2/read"
#define RELAY_1_TOPIC_W "relay/3/write"
#define RELAY_2_TOPIC_W "relay/4/write"

// My rasberry pi server name
const char *host = "raspberr ypi.local";
WiFiClient net;
PubSubClient client(net);


// put function declarations here:
void taskHandleRGBLed(void *pvParameters);
void taskReadTemperature(void *pvParameters);
void readTemperatureFromBME280(void *pvParameters);
void relaysControl(void *pvParameters);

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
      // client.publish(TEMPERATURE_TOPIC, "{\"message\": \"Initialized the connection\"}");

      xTaskCreate(taskReadTemperature, "taskReadTemperature", 2048 * 4, NULL, 1, NULL);
      xTaskCreate(readTemperatureFromBME280, "readTemperatureFromBME280", 2048 * 4, NULL, 1, NULL);
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


void readTemperatureFromBME280(void *pvParameters){
    if (! bme.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }


  while(1){
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;
    String payload_temp = "{\"message\": \"" + String(temperature) + "\"}";
    String payload_hum = "{\"message\": \"" + String(humidity) + "\"}";
    String payload_press = "{\"message\": \"" + String(pressure) + "\"}";


    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print(" °C  Hum: ");
    Serial.print(humidity);
    Serial.print(" %  Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");


    client.publish(BME_280_TEMPERATURE_TOPIC, payload_temp.c_str());
    client.publish(BME_280_HUMIDITY_TOPIC, payload_hum.c_str());
    client.publish(BME_280_PRESSURE_TOPIC, payload_press.c_str());
    vTaskDelay(pdMS_TO_TICKS(1200));
  }
}



void relaysControl(void *pvParameters){
  //control the relays 1 and 2
  pinMode(26, OUTPUT); // TODO check pins
  pinMode(27, OUTPUT);

  client.subscribe(RELAY_1_TOPIC_W);
  client.subscribe(RELAY_2_TOPIC_W);
  while(1){
    
    vTaskDelay(pdMS_TO_TICKS(1200));

    
  }
}