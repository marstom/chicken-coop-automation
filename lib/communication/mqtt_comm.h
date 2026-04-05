
#pragma once

#include "mqtt_client.h"

namespace communication
{
    QueueHandle_t mqttQueue;
    QueueHandle_t relayQueue;
    QueueHandle_t webQueue;

    struct RelayCommand
    {
        bool on;
    };

    /**
     * Small POD-style message passed between tasks through `mqttQueue`.
     *
     * Purpose:
     * - lets producer tasks prepare MQTT data without touching `PubSubClient`
     * - keeps cross-task communication simple by copying fixed-size buffers
     * - is consumed later by `taskMQTT()`, which publishes `topic` + `payload`
     *
     * Notes:
     * - `topic` is limited to 31 characters plus the null terminator
     * - `payload` is limited to 63 characters plus the null terminator
     * - `setContent()` truncates long strings safely
     * - `sendToQueue()` sends a copy of this struct to the queue, so the caller
     *   can reuse the same local variable immediately after calling it
     */
    struct MqttMessage
    {
        char topic[32];
        char payload[64];

        MqttMessage()
        {
        }

        ~MqttMessage()
        {
        }
        /// Copy topic and payload into the internal fixed-size buffers.
        void setContent(const char *t, const char *msg)
        {
            strncpy(topic, t, sizeof(topic));
            topic[sizeof(topic) - 1] = '\0';
            strncpy(payload, msg, sizeof(payload));
            payload[sizeof(payload) - 1] = '\0';
        }

        /// Queue this message for the MQTT task to publish later.
        /// `timeout_ms` is currently unused; this call blocks with `portMAX_DELAY`.
        void sendToQueue(uint32_t timeout_ms = 10)
        {
            if (!mqttQueue)
                return; // guard
            xQueueSend(mqttQueue, this, portMAX_DELAY);
        }
    };

    /**
     * This is queue for a webpage
     * it has reads of temperature etc. and
     * I can push data to queue
     * and receive it in different task
     *
     */
    class WebMessage
    {
    public:
        static constexpr const char *temperature = "temperature";
        static constexpr const char *humidity = "humidity";
        static constexpr const char *pressure = "pressure";

        void setContent(const char *messageType, const char *msg)
        {
            strncpy(buffer, msg, sizeof(buffer));
            buffer[sizeof(buffer) - 1] = '\0';

            strncpy(msgType, messageType, sizeof(msgType));
        }

        char *getBuffer()
        {
            return buffer;
        }

        char *getMessageType()
        {
            return msgType;
        }

        void sendToQueue()
        {
            if (!webQueue)
                return;
            if (xQueueSend(webQueue, this, 0) != pdTRUE)
            {                                     // when queue is full, drop sth
                xQueueReceive(webQueue, this, 0); // remove oldest
                xQueueSend(webQueue, this, 0);    // try again
            }
        }

    private:
        char buffer[64];
        char msgType[12];
    };

    void initQueue()
    {
        mqttQueue = xQueueCreate(200, sizeof(MqttMessage));
        relayQueue = xQueueCreate(2, sizeof(RelayCommand));
        webQueue = xQueueCreate(12, sizeof(WebMessage));
    }
}
