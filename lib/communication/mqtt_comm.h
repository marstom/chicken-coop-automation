
#pragma once

#include "mqtt_client.h"

namespace communication
{
    QueueHandle_t mqttQueue;
    QueueHandle_t relayQueue;

    struct RelayCommand
    {
        bool on;
    };


    struct MqttMessage
    {
        char topic[32];
        char payload[64];

        MqttMessage() {
        }


        ~MqttMessage() {

        }

        void setContent(const char *t, const char *msg)
        {
            strncpy(topic, t, sizeof(topic));
            topic[sizeof(topic) - 1] = '\0';
            strncpy(payload, msg, sizeof(payload));
            payload[sizeof(payload) - 1] = '\0';
        }

        void sendToQueue(uint32_t timeout_ms = 10)
        {
            if (!mqttQueue)
                return; // guard
            xQueueSend(mqttQueue, this, portMAX_DELAY);
        }
    };

    void initQueue(){
            mqttQueue = xQueueCreate(200, sizeof(MqttMessage));
            relayQueue = xQueueCreate(2, sizeof(RelayCommand));

    }
}