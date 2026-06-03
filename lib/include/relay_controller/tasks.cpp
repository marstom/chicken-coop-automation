#include "relay_controller/tasks.h"
#include "relay_controller/constants.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include "esp_task_wdt.h"
#include "mqtt_comm.h"
#include "secrets.h"
#include "debug_tools/debug_tools.h"
#include "ble.h"
#include <ArduinoBLE.h>

namespace relay_controller
{

    WiFiClient net;

    PubSubClient client(net);

    /// Ask taskRelay (the only owner of RELAY_PIN) to open the door.
    static void requestDoorOpen()
    {
        communication::RelayCommand cmd;
        cmd.on = true;
        xQueueSend(communication::relayQueue, &cmd, 0); // queue is thread safe
    }

    /// Door TCP endpoint only answers "GET /open?token=<DOOR_TOKEN>".
    /// A token left at "unset" disables the endpoint.
    static bool isAuthorizedDoorRequest(const String &requestLine)
    {
        const char *prefix = "GET /open?token=";
        if (strcmp(DOOR_TOKEN, "unset") == 0 || !requestLine.startsWith(prefix))
        {
            return false;
        }
        int tokenEnd = requestLine.indexOf(' ', strlen(prefix));
        if (tokenEnd < 0)
        {
            tokenEnd = requestLine.length();
        }
        String token = requestLine.substring(strlen(prefix), tokenEnd);
        token.trim();
        return token == DOOR_TOKEN;
    }

    // direct tcp
    void taskTcpServer(void *pvParameters)
    {
        WiFiServer server(80); // http
        server.begin();
        esp_task_wdt_add(NULL); // subscribe once; must be fed every iteration, not only on requests

        for (;;)
        {
            WiFiClient httpClient = server.available();
            if (httpClient)
            {
                String req = httpClient.readStringUntil('\n');
                Serial.println(req);

                if (isAuthorizedDoorRequest(req))
                {
                    requestDoorOpen();
                    httpClient.println("HTTP/1.1 200 OK");
                    httpClient.println("Content-Type: text/plain");
                    httpClient.println("Connection: close");
                    httpClient.println("");
                    httpClient.println("Open, closing in 6s...");
                }
                else
                {
                    httpClient.println("HTTP/1.1 403 Forbidden");
                    httpClient.println("Content-Type: text/plain");
                    httpClient.println("Connection: close");
                    httpClient.println("");
                    httpClient.println("Forbidden");
                }
                httpClient.stop();
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
                        const bool authorized =
                            strcmp(BLE_DOOR_PASS, "unset") != 0 && receivedData == BLE_DOOR_PASS;
                        if (authorized)
                        {
                            Serial.println("Open the door BLE ...");
                            String resp = "Status: The door has been opened!";
                            deviceResponseCharacteristic.setValue(resp);
                            requestDoorOpen();
                        }
                        else
                        {
                            Serial.println("DENY BLE");
                            // Send notification back
                            String resp = "Status: acces denied...";
                            deviceResponseCharacteristic.setValue(resp);
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                Serial.println("Disconnected");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
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
                    vTaskDelay(pdMS_TO_TICKS(DOOR_OPEN_MS));
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
