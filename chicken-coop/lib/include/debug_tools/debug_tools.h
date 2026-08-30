#pragma once

#include <Arduino.h>
#include "freertos/task.h"
#include <mqtt_comm.h>

namespace debug_tools{
extern String logPrefix;

struct LogOptions {
    bool printToSerial = true;
    bool logToMqtt = true;
};

extern LogOptions logOptions;

void logMessage(const char *fmt, ...);
void logMessage(const LogOptions &options, const char *fmt, ...);
void printStackInfo(const char *taskName, TaskHandle_t taskHandle);
void printHeap();

/// One entry of the stack-monitor watch list. `handle` is a pointer because
/// task handles are filled in by xTaskCreate after the list is defined.
struct MonitoredTask {
    const char *name;
    TaskHandle_t *handle;
};

/// Shared stack/heap monitor task. `pvParameters` must point to a
/// MonitoredTask array with static lifetime, terminated by {nullptr, nullptr}.
void taskStackMonitor(void *pvParameters);

}
