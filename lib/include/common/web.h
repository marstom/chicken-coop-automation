
#pragma once
#include <WiFi.h>

namespace common
{
    void setupMDNS(const char *hostname, const char *instance_name);
    namespace wifi_event
    {
        extern const char *g_mdns_hostname; // extern - it's defined somewhere else
        extern const char *g_instance_name;
        void onWiFiEvent(WiFiEvent_t event);

    }
}
