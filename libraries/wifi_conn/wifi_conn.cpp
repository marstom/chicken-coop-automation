#include <WiFi.h>

#include "wifi_conn.h"

namespace wifi
{

    /* Retry until success, it blocks whole proggramm */
    void connectToWifiWithWait(const char *ssid, const char *pass, const char *hostname, bool disableModemSleep)
    {
        Serial.println("Connecting to WiFi");
        Serial.println(ssid);
        WiFi.setHostname(hostname);
        WiFi.mode(WIFI_STA);
        if (disableModemSleep)
        {
            // connection stability is better
            WiFi.setSleep(false);
        }
        else
        {
            // it's good idea when you use thing on battery :)
            WiFi.setSleep(true);
        }
        WiFi.setAutoReconnect(true);
        WiFi.begin(ssid, pass);
        unsigned long lastBegin = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
            // on timeout try again
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