#include "common/ota.h"

#include <ArduinoOTA.h>

#include "debug_tools/debug_tools.h"

namespace common::ota
{
    void setup(const char *hostname)
    {
        debug_tools::logMessage("Initialize OTA updates via Wireless");
        debug_tools::logMessage("UPDATE VIA OTA");

        ArduinoOTA.setHostname(hostname);
        ArduinoOTA
            .onStart([]()
                     { debug_tools::logMessage("OTA update start"); })
            .onEnd([]()
                   { debug_tools::logMessage("\nOTA update end"); })
            .onProgress([](unsigned int progress, unsigned int total)
                        { debug_tools::logMessage("Progress: %u%%\r", (progress / (total / 100))); })
            .onError([](ota_error_t error)
                     { debug_tools::logMessage("Error[%u]: ", error); });

        ArduinoOTA.begin();
    }

    void handle()
    {
        ArduinoOTA.handle();
    }
}
