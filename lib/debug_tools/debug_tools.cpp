#include "debug_tools.h"

#include <cstdarg>
#include <cstdio>

namespace debug_tools{

String logPrefix = "";

/*
Usage

debug_tools::LogOptions{.printToSerial = false, .logToMqtt = true}

---
debug_tools::LogOptions opts;
opts.printToSerial = false;
opts.logToMqtt = true;

debug_tools::logMessage(opts, "MQTT only: %d", value);


*/
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
    logFormattedMessage(LogOptions{}, fmt, args);
    va_end(args);
}

void logMessage(const LogOptions &options, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    logFormattedMessage(options, fmt, args);
    va_end(args);
}

void printStackInfo(const char *taskName, TaskHandle_t mqttTaskHandler)
{
    if (mqttTaskHandler)
    {
        logMessage("MQTTTask stack free: %u words (%u bytes)\n",
                   uxTaskGetStackHighWaterMark(mqttTaskHandler),
                   uxTaskGetStackHighWaterMark(mqttTaskHandler) * 4);
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

}
