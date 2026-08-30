
#pragma once

#include "mqtt_client.h"
#include "bounded_copy.h"

namespace communication
{
    extern QueueHandle_t mqttQueue;
    extern QueueHandle_t relayQueue;
    extern QueueHandle_t webQueue;

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
            logic::copyBounded(topic, sizeof(topic), t);
            logic::copyBounded(payload, sizeof(payload), msg);
        }

        /// Queue this message for the MQTT task to publish later.
        /// Waits at most `timeout_ms`; the message is dropped if the queue
        /// stays full (never block a sensor task on a slow/dead broker).
        void sendToQueue(uint32_t timeout_ms = 10)
        {
            if (!mqttQueue)
                return; // guard
            xQueueSend(mqttQueue, this, pdMS_TO_TICKS(timeout_ms));
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
        static constexpr const char *altitude = "altitude";

        void setContent(const char *messageType, const char *msg)
        {
            logic::copyBounded(buffer, sizeof(buffer), msg);
            logic::copyBounded(msgType, sizeof(msgType), messageType);
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

    void initQueue();
}
