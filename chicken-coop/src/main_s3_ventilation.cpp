#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <PubSubClient.h>

#include "common/web.h"
#include "common/wifi_conn/wifi_conn.h"
#include "secrets.h"
#include <WiFiClientSecure.h>

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

void setup_aws_iot()
{
    secureClient.setCACert(AWS_ROOT_CA);
    secureClient.setCertificate(DEVICE_CERT);
    secureClient.setPrivateKey(DEVICE_PRIVATE_KEY);

    mqttClient.setServer(AWS_ENDPOINT, AWS_PORT);
}

bool connect_aws_iot()
{
    if (mqttClient.connected()) {
        return true;
    }

    Serial.println("Connecting to AWS IoT...");

    bool connected = mqttClient.connect(
        "office-vantilation-esp32"
    );

    if (connected) {
        Serial.println("Connected to AWS IoT");
    } else {
        Serial.printf(
            "MQTT error: %d\n",
            mqttClient.state()
        );
    }

    return connected;
}

/*
publish whole batch because it's cheaper
*/
bool publish_batch(const String& payload)
{
    if (!connect_aws_iot()) {
        return false;
    }

    return mqttClient.publish(
        "chicken-coop/telemetry",
        payload.c_str()
    );
}

void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb

    common::connectToWifiWithWait(SSID_OFFICE, WIFI_PASS, "vantilation", /*disableModemSleep=*/false);

    Serial.println("Setup aws iot....");
    setup_aws_iot();
    Serial.println("....SUCCESS!");
    xTaskCreate(taskReadTemperature, "Read Temperature", 8192, NULL, 1, NULL);
}

void loop()
{
    mqttClient.loop(); // refresh mqtt client
    delay(10); // wait for a second
}



// TODO add policy in AWS
void taskReadTemperature(void *pvParameters)
{
    while (1)
    {

        String payload = R"({
            "device_id": "office-vantilation-esp32-1",
            "measurements": [
                {
                    "timestamp": 1789392000,
                    "temperature": 20.4,
                    "humidity": 68.1
                }
            ]
        })";

        publish_batch(payload);
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}