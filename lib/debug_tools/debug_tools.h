#pragma once

#include <Arduino.h>
#include "freertos/task.h"
#include <mqtt_comm.h>



namespace debug_tools{


String logPrefix = "";

void logMessage(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.println(buf);              // local USB log
    communication::MqttMessage msg;
    String topic = logPrefix + "log/mydebug";
    msg.setContent(topic.c_str(), buf);
    msg.sendToQueue();
}

void printStackInfo(const char *taskName, TaskHandle_t mqttTaskHandler){
        if (mqttTaskHandler)
        {
            logMessage("MQTTTask stack free: %u words (%u bytes)\n",
                       uxTaskGetStackHighWaterMark(mqttTaskHandler),
                       uxTaskGetStackHighWaterMark(mqttTaskHandler) * 4);
        }
}

// debug monitoring
void printHeap()
{
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    debug_tools::logMessage("Free: %u, Min free: %u, Largest free block: %u\n",
                  info.total_free_bytes,
                  info.minimum_free_bytes,
                  info.largest_free_block);
}


}