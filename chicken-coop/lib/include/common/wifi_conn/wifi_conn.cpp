#include <WiFi.h>

#include "wifi_conn.h"

namespace common
{

    void connectToWifiWithWait(const char *ssid, const char *pass, const char *hostname, bool disableModemSleep)
    {
        Serial.println("Connecting to WiFi");
        Serial.println(ssid);
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(hostname);
        if (disableModemSleep)
        {
            WiFi.setSleep(false); // modem sleep causes random disconnects / MQTT keepalive timeouts
        }
        else
        {
            // WIFI_PS_MIN_MODEM: BLE+WiFi coexistence on ESP32-C3 requires
            // modem sleep, otherwise coex_core_enable() calls abort() at boot
            WiFi.setSleep(true);
        }
        WiFi.setAutoReconnect(true); // let the stack retry on its own after drops
        WiFi.begin(ssid, pass);
        unsigned long lastBegin = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
            // status can get stuck in WL_CONNECT_FAILED / WL_NO_SSID_AVAIL
            // (e.g. AP still booting after a power cut) - retry begin()
            if (millis() - lastBegin >= 15000)
            {
                Serial.println();
                Serial.println("Still not connected, retrying WiFi.begin()...");
                WiFi.disconnect();
                delay(100);
                WiFi.begin(ssid, pass);
                lastBegin = millis();
            }
        }
        Serial.println("Connected to WiFi");
    }

}