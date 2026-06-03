#include "relay_controller/tasks.h"
#include "relay_controller/constants.h"
#include <stdarg.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_task_wdt.h"
#include "mqtt_comm.h"
#include "secrets.h"
#include "debug_tools/debug_tools.h"
#include "ble.h"
#include <ArduinoBLE.h>

#include "relay_controller/constants.h"

extern TaskHandle_t hMQTTTask;
extern TaskHandle_t hBME280Task;
extern TaskHandle_t hRelayTask;

namespace relay_controller
{

    // ------------- BLE support -------------
    // UUIDs
    WiFiClient net;

    PubSubClient client(net);

    // direct tcp
    void taskTcpServer(void *pvParameters)
    {
        WiFiServer server(80); // http
        server.begin();
        esp_task_wdt_add(NULL); // subscribe once; must be fed every iteration, not only on requests

        for (;;)
        {
            WiFiClient client = server.available();
            if (client)
            {
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
            }
            esp_task_wdt_reset();
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
                            Serial.println("Open the door BLE ...");
                            String resp = "Status: The door has been opened!";
                            deviceResponseCharacteristic.setValue(resp);
                            digitalWrite(RELAY_PIN, LOW);
                            vTaskDelay(pdMS_TO_TICKS(6000));
                            digitalWrite(RELAY_PIN, HIGH);
                            Serial.println("Door closed.");
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
            if constexpr (ENABLE_MONITORING)
            {
                debug_tools::printStackInfo("MQTTTask", ::hMQTTTask);
                debug_tools::printStackInfo("BME280Task", ::hBME280Task);
                debug_tools::printStackInfo("RelayTask", ::hRelayTask);
                debug_tools::printHeap(); // DEBUG memory leaks
            }
            vTaskDelay(pdMS_TO_TICKS(5000)); // print every 10s
        }
    }

    static bool connectToMqttBroker()
    {
        if (MQTT_USER[0] != '\0')
        {
            return client.connect(THINGNAME, MQTT_USER, MQTT_PASS);
        }
        return client.connect(THINGNAME);
    }

    // MQTT loop task
    void taskMQTT(void *pvParameters)
    {
        esp_task_wdt_add(NULL);
        TickType_t lastReconnectAttempt = 0;
        TickType_t lastWifiOk = xTaskGetTickCount();

        for (;;)
        {
            const TickType_t now = xTaskGetTickCount();
            if (WiFi.status() == WL_CONNECTED)
            {
                lastWifiOk = now;
                if (!client.connected())
                {
                    // WiFi drop kills the broker socket; reconnect and re-subscribe
                    if (now - lastReconnectAttempt >= pdMS_TO_TICKS(2000))
                    {
                        lastReconnectAttempt = now;
                        if (connectToMqttBroker())
                        {
                            client.subscribe(RELAY_1_SET_TOPIC);
                            debug_tools::logMessage("MQTT reconnected");
                        }
                    }
                }
                else
                {
                    client.loop(); // <--- processes incoming MQTT messages
                }
            }
            else if (now - lastWifiOk >= pdMS_TO_TICKS(30000))
            {
                // safety net: WiFi stuck down and event-driven reconnect didn't recover
                debug_tools::logMessage("WiFi down >30s, forcing reconnect");
                WiFi.reconnect();
                lastWifiOk = now;
            }

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
                    Serial.println("Door opened ...");
                    digitalWrite(RELAY_PIN, LOW);
                    vTaskDelay(pdMS_TO_TICKS(6000));
                    digitalWrite(RELAY_PIN, HIGH);
                    Serial.println("Door CLOSED");
                }
                else
                {
                    digitalWrite(RELAY_PIN, HIGH);
                }
            }
        }
    }
}
