#include "wifi_mdns.h"
#include <ESPmDNS.h>

namespace wifi
{
    void setupMdns(const char *hostname)
    {
        if (MDNS.begin(hostname))
        {
            Serial.println("mDNS: " + String(hostname) + ".local");
        }
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("mqtt", "tcp", 1883);
        MDNS.addService("ota", "tcp", 3232);
    }

}