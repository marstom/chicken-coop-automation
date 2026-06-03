#include "debug_tools.h"

#include <cstdarg>
#include <cstdio>

namespace debug_tools{

String logPrefix = "";
LogOptions logOptions;

static void logFormattedMessage(const LogOptions &options, const char *fmt, va_list args)
{
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);

    if (options.printToSerial) {
        Serial.println(buf);
    }

    if (options.logToMqtt) {
        communication::MqttMessage msg;
        String topic = logPrefix + "log/mydebug";
        msg.setContent(topic.c_str(), buf);
        msg.sendToQueue();
    }
}

void logMessage(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    // LogOptions logOptions;
    // logOptions.printToSerial = true;
    // logOptions.logToMqtt = true;
    logFormattedMessage(logOptions, fmt, args);
    va_end(args);
}

void logMessage(const LogOptions &options, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    logFormattedMessage(options, fmt, args);
    va_end(args);
}

void printStackInfo(const char *taskName, TaskHandle_t taskHandle)
{
    if (taskHandle)
    {
        logMessage("%s stack free: %u words (%u bytes)\n",
                   taskName,
                   uxTaskGetStackHighWaterMark(taskHandle),
                   uxTaskGetStackHighWaterMark(taskHandle) * 4);
    }
}

void printHeap()
{
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    debug_tools::logMessage("Free: %u, Min free: %u, Largest free block: %u\n",
                            info.total_free_bytes,
                            info.minimum_free_bytes,
                            info.largest_free_block);
}

void taskStackMonitor(void *pvParameters)
{
    const MonitoredTask *tasks = static_cast<const MonitoredTask *>(pvParameters);
    for (;;)
    {
        for (const MonitoredTask *t = tasks; t->name != nullptr; ++t)
        {
            printStackInfo(t->name, *t->handle);
        }
        printHeap(); // DEBUG memory leaks
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

}
