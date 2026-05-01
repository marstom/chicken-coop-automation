#include "relay_controller/tasks.h"
#include "relay_controller/constants.h"
#include <stdarg.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include "esp_task_wdt.h"
#include "mqtt_comm.h"
#include <ArduinoBLE.h>

#include "relay_controller/constants.h"

namespace relay_controller
{

    // ------------- BLE support -------------
    // UUIDs
    WiFiClient net;


    PubSubClient client(net);
    // GATT objects
    BLEService deviceService(deviceServiceUuid);

    // phone writes
    BLEStringCharacteristic deviceRequestCharacteristic(deviceServiceRequestCharacteristicUuid, BLEWrite, 32);
    // phone reads / notify phone
    BLEStringCharacteristic deviceResponseCharacteristic(deviceServiceResponseCharacteristicUuid, BLERead | BLENotify, 32);

    BLEDescriptor reqName("2901", "Phone → ESP request");
    BLEDescriptor respName("2901", "ESP → Phone response");

    // direct tcp
    void taskTcpServer(void *pvParameters)
    {
        WiFiServer server(80); // http
        server.begin();

        for (;;)
        {
            WiFiClient client = server.available();
            if (client)
            {
                esp_task_wdt_add(NULL);
                String req = client.readStringUntil('\n');
                Serial.println(req);

                // on receive GET requeset
                digitalWrite(RELAY_PIN, LOW);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: text/plain");
                client.println("Connection: close");
                client.println("");
                client.println("Open, closing in 6s...");
                client.stop();
                vTaskDelay(pdMS_TO_TICKS(6000));
                digitalWrite(RELAY_PIN, HIGH);
                esp_task_wdt_reset();
            }
            vTaskDelay(pdMS_TO_TICKS(250)); // yield
        }
    }

    void taskBLE(void *pvParameters)
    {
        Serial.println("Starting BLE work!");
        while (1)
        {
            BLEDevice central = BLE.central();
            if (central)
            {
                // poll BLE radio events and handle them
                while (central.connected())
                {
                    BLE.poll();

                    if (deviceRequestCharacteristic.written())
                    {
                        String receivedData = deviceRequestCharacteristic.value();
                        String pass = "paulina";
                        Serial.println("RCV data:" + receivedData);
                        Serial.println(receivedData == pass);
                        if (receivedData == pass)
                        {
                            Serial.println("OPEN BLE");
                            String resp = "Status: The door has been opened!";
                            deviceResponseCharacteristic.setValue(resp);
                            digitalWrite(RELAY_PIN, LOW);
                            vTaskDelay(pdMS_TO_TICKS(6000));
                            digitalWrite(RELAY_PIN, HIGH);
                        }
                        else
                        {
                            Serial.println("DENY BLE");
                            // Send notification back
                            String resp = "Status: acces denied...";
                            deviceResponseCharacteristic.setValue(resp);
                            digitalWrite(RELAY_PIN, HIGH);
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                Serial.println("Disconnected");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // --- New task: device monitoring, for troubleshooting ---
    void taskStackMonitor(void *pvParameters)
    {
        for (;;)
        {
#ifdef ENABLE_MONITORING
            debug_tools::printStackInfo("MQTTTask", hMQTTTask);
            debug_tools::printStackInfo("BME280Task", hBME280Task);
            debug_tools::printStackInfo("RelayTask", hRelayTask);
            debug_tools::printHeap(); // DEBUG memory leaks
#endif
            vTaskDelay(pdMS_TO_TICKS(5000)); // print every 10s
        }
    }

    // MQTT loop task
    void taskMQTT(void *pvParameters)
    {
        esp_task_wdt_add(NULL);
        for (;;)
        {
            client.loop(); // <--- processes incoming MQTT messages
            communication::MqttMessage msg;
            if (xQueueReceive(communication::mqttQueue, &msg, 0))
            {
                client.publish(msg.topic, msg.payload);
            }
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    void taskRelay(void *pv)
    {
        communication::RelayCommand cmd;
        for (;;)
        {
            if (xQueueReceive(communication::relayQueue, &cmd, portMAX_DELAY))
            {
                if (cmd.on)
                {
                    digitalWrite(RELAY_PIN, LOW);
                    vTaskDelay(pdMS_TO_TICKS(6000));
                    digitalWrite(RELAY_PIN, HIGH);
                }
                else
                {
                    digitalWrite(RELAY_PIN, HIGH);
                }
            }
        }
    }
}
