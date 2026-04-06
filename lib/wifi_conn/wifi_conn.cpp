#include <WiFi.h>

#include "wifi_conn.h"

namespace my
{

    void connect_to_wifi_with_wait(const char *ssid, const char *pass, const char *hostname)
    {
        Serial.println("Connecting to WiFi");
        Serial.println(ssid);
        Serial.println(pass);
        WiFi.setHostname(hostname);
        WiFi.begin(ssid, pass);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
        Serial.println("Connected to WiFi");
    }

}