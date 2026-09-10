
#pragma once
#include <WiFi.h>

namespace common
{
    void setupMDNS(const char *hostname, const char *instance_name);

    namespace wifi_event
    {
        // Configure these before registering onWiFiEvent with WiFi.onEvent().
        extern const char *g_mdns_hostname; // extern - it's defined somewhere else
        extern const char *g_instance_name;
        // 0 = keep reconnecting forever; N > 0 = ESP.restart() after N
        // consecutive failed reconnects (counter resets on GOT_IP).
        extern int g_restart_after_failures;

        void onWiFiEvent(WiFiEvent_t event);
    }
}
