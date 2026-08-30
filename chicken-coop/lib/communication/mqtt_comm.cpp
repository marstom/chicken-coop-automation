#include "mqtt_comm.h"

namespace communication
{
    QueueHandle_t mqttQueue = nullptr;
    QueueHandle_t relayQueue = nullptr;
    QueueHandle_t webQueue = nullptr;

    void initQueue()
    {
        // ~96 B per message; 30 entries is plenty and saves ~16 KB of RAM vs 200
        mqttQueue = xQueueCreate(30, sizeof(MqttMessage));
        relayQueue = xQueueCreate(2, sizeof(RelayCommand));
        webQueue = xQueueCreate(12, sizeof(WebMessage));
    }
}
