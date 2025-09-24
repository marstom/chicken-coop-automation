
#include <Arduino.h>
// #include <WiFi.h>
// #include <FreeRTOS.h>
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
#define STATUS_TOPIC "status/read"
#define RELAY_1_SET_TOPIC "relay/1/set"

// PIN definitions
#define D0 2
#define D1 3
#define D2 28
#define D3 29
#define D4 4
#define D5 5
#define D6 11
#define D7 12
#define D8 13
#define D9 14
#define D10 15

// My rasberry pi server name
const char *host = "raspberrypi.local";
WiFiClient net;
PubSubClient client(net);

// put function declarations here:
void taskReceiveRelayCommand(void *pvParameters);
void mycallback(char *topic, byte *message, unsigned int length);
void taskMQTT(void *pvParameters); // Spin all the time and keep receiving the messages!

void setup()
{
    Serial.begin(9600);
    my::connect_to_wifi_with_wait();
    pinMode(D0, OUTPUT); // D0 as output

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


    if (String(topic) == RELAY_1_SET_TOPIC) {
        // in case of relay, the high state is OFF
        if (messageTemp == "ON") {
            digitalWrite(D0, HIGH); 
        } else {
            digitalWrite(D0, LOW);
        }
    }
}