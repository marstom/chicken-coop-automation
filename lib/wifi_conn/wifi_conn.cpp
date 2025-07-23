#include <WiFi.h>

#include "wifi_conn.h"

namespace my{

void connect_to_wifi_with_wait(){
    WiFi.begin("T-Mobile-Tom", "7FQR242857");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
}

}