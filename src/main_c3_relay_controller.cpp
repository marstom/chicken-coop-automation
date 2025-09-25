
#include <Arduino.h>
// #include <WiFi.h>
// #include <FreeRTOS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <HTTPClient.h>
#include <PubSubClient.h>

// for BME
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "wifi_conn.h"
// #include "c3utils.h"

// Constants

// Addressable RGB LED, driven by GPIO48.
#define LED_PIN 48

// MQTT stuff
#define THINGNAME "esp32"
#define STATUS_TOPIC "status/read"
#define RELAY_1_SET_TOPIC "relay/1/set"

// BME temperature and humidity sensor, connected to i2c bus in current setup
#define BME_SCK my::D8
#define BME_MISO my::D9
#define BME_MOSI my::D10
#define BME_CS my::D7

#define BME_TEMPERATURE_TOPIC "bme280/temperature"
#define BME_PRESSURE_TOPIC "bme280/pressure"
#define BME_HUMIDITY_TOPIC "bme280/humidity"
#define BME_ALTITUDE_TOPIC "bme280/altitude"

#define I2C_SDA my::D4
#define I2C_SCL my::D5

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// My rasberry pi server name
const char *host = "raspberrypi.local";
WiFiClient net;
PubSubClient client(net);

// put function declarations here:
void taskReceiveRelayCommand(void *pvParameters);
void mycallback(char *topic, byte *message, unsigned int length);
void taskMQTT(void *pvParameters); // Spin all the time and keep receiving the messages!
void taskReadBME280(void *pvParameters);

void setup()
{
    Serial.begin(9600);
    my::connect_to_wifi_with_wait();
    pinMode(my::D0, OUTPUT); // D0 as output

    client.setServer(host, 1883); // rpi server
    client.setCallback(mycallback);

    while (!client.connected())
    {
        if (client.connect(THINGNAME))
        {
            Serial.println("☑ Connected to RPI Broker!");
            // Subscriptions here
            client.subscribe(RELAY_1_SET_TOPIC);
            ////////////
            client.publish(STATUS_TOPIC, "{\"message\": \"Initialized the connection from c3 relay controller\"}");
        }
        else
        {
            Serial.print("✖ Failed to connect, try again in 2 seconds, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }

    xTaskCreatePinnedToCore(taskReceiveRelayCommand, "taskReceiveRelayCommand", 4096 * 4, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskMQTT, "taskMQTT", 4096 * 4, NULL, 1, NULL, 0);
    xTaskCreate(taskReadBME280, "taskReadBME280", 4096 * 4, NULL, 1, NULL);
}

void loop() { /* Empty loop */ }

void taskReceiveRelayCommand(void *pvParameters)
{
    // Probably here i want to receive mqtt message, but is it right place?

    for (;;)
    {

        // Serial.println("Waiting for relay command");
        // digitalWrite(D0, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        // digitalWrite(D0, LOW);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// MQTT loop task
void taskMQTT(void *pvParameters)
{
    for (;;)
    {
        client.loop(); // <--- processes incoming MQTT messages
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// the mqtt callback:

void mycallback(char *topic, byte *message, unsigned int length)
{
    // Here is messsages arrival
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;
    for (int i = 0; i < length; i++)
    {
        messageTemp += (char)message[i];
    }
    Serial.println(messageTemp);

    if (String(topic) == RELAY_1_SET_TOPIC)
    {
        // in case of relay, the high state is OFF
        if (messageTemp == "ON")
        {
            digitalWrite(my::D0, HIGH);
        }
        else
        {
            digitalWrite(my::D0, LOW);
        }
    }
}

void taskReadBME280(void *pvParameters)
{
    bool status;
    String payload;
    //   Wire.begin(4, 5);
    //   while (!status) {
    //     status = bme.begin(0x77, &Wire);   // I2C or 0x77
    //     Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //     vTaskDelay(pdMS_TO_TICKS(4000));
    //   }

    
    Wire.begin(4, 5);
    if (!bme.begin(0x76, &Wire))
    {
        if (!bme.begin(0x77, &Wire))
        {
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
            while (1)
                delay(10);
        }
    }
    for (;;)
    {
        // Serial.print("Temperature = ");
        // Serial.print(bme.readTemperature());
        String t = String(bme.readTemperature());
        Serial.print(t);
        client.publish(BME_TEMPERATURE_TOPIC, t.c_str(), true);
        Serial.println(" *C");

        // Serial.print("Pressure = ");
        // Serial.print(bme.readPressure() / 100.0F);
        String p = String(bme.readPressure());
        client.publish(BME_PRESSURE_TOPIC, p.c_str());
        // Serial.println(" hPa");

        // Serial.print("Approx. Altitude = ");
        // Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        String a = String(bme.readAltitude(SEALEVELPRESSURE_HPA));
        client.publish(BME_ALTITUDE_TOPIC, a.c_str());
        // Serial.println(" m");

        // Serial.print("Humidity = ");
        // Serial.print(bme.readHumidity());
        String h = String(bme.readHumidity());
        client.publish(BME_HUMIDITY_TOPIC, h.c_str()); // add ,true to retain
        // Serial.println(" %");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}