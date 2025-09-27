#pragma once

#include <Arduino.h>
#include "freertos/task.h"
#include <mqtt_comm.h>



namespace debug_tools{


void logMessage(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    /// TODO add variadic funciont *fmt , .....
    Serial.println(buf);              // local USB log

    communication::MqttMessage msg;
    msg.setContent("log/mydebug", buf);
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



}