#include "mqtt_comm.h"

namespace communication
{
    QueueHandle_t mqttQueue = nullptr;
    QueueHandle_t relayQueue = nullptr;
    QueueHandle_t webQueue = nullptr;

    void initQueue()
    {
        mqttQueue = xQueueCreate(200, sizeof(MqttMessage));
        relayQueue = xQueueCreate(2, sizeof(RelayCommand));
        webQueue = xQueueCreate(12, sizeof(WebMessage));
    }
}
