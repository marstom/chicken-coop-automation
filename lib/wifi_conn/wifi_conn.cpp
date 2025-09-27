#include <WiFi.h>
#include "secrets.h"

#include "wifi_conn.h"

namespace my{

void connect_to_wifi_with_wait(){
    Serial.println("Connecting to WiFi");
    Serial.println(secrets::wifiPass);
    Serial.println(secrets::wifiSsid);
    WiFi.begin(secrets::wifiSsid, secrets::wifiPass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
}

}