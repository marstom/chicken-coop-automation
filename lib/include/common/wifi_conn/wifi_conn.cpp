#include <WiFi.h>

#include "wifi_conn.h"

namespace common
{

    void connect_to_wifi_with_wait(const char *ssid, const char *pass, const char *hostname)
    {
        Serial.println("Connecting to WiFi");
        Serial.println(ssid);
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(hostname);
        WiFi.setSleep(false);        // modem sleep causes random disconnects / MQTT keepalive timeouts
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