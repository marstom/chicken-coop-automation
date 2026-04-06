#pragma once

#include <Arduino.h>
#include "freertos/task.h"
#include <mqtt_comm.h>

namespace debug_tools{
extern String logPrefix;

void logMessage(const char *fmt, ...);
void printStackInfo(const char *taskName, TaskHandle_t mqttTaskHandler);
void printHeap();

}
