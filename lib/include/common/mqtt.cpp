#include "common/mqtt.h"

#include <WiFi.h>
#include "esp_task_wdt.h"

#include "debug_tools/debug_tools.h"
#include "mqtt_comm.h"

namespace common::mqtt
{
    bool connectToBroker(PubSubClient &client, const char *user, const char *pass, const char *thingName)
    {
        if (user != nullptr && user[0] != '\0')
        {
            return client.connect(thingName, user, pass);
        }
        return client.connect(thingName);
    }

    static void onConnected(const TaskConfig &cfg)
    {
        debug_tools::logMessage("☑ Connected to MQTT broker!");
        if (cfg.subscribeTopic != nullptr)
        {
            cfg.client->subscribe(cfg.subscribeTopic);
        }
        if (cfg.statusTopic != nullptr)
        {
            cfg.client->publish(cfg.statusTopic, cfg.statusPayload);
        }
    }

    void taskMQTT(void *pvParameters)
    {
        const TaskConfig &cfg = *static_cast<const TaskConfig *>(pvParameters);
        PubSubClient &client = *cfg.client;

        esp_task_wdt_add(NULL); // watchdog
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
                        if (connectToBroker(client, cfg.user, cfg.pass, cfg.thingName))
                        {
                            onConnected(cfg);
                        }
                        else
                        {
                            debug_tools::logMessage("✖ MQTT connect failed, rc=%d", client.state());
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
            if (client.connected() && xQueueReceive(communication::mqttQueue, &msg, 0))
            {
                client.publish(msg.topic, msg.payload);
            }
            esp_task_wdt_reset(); // reset watchdog
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
