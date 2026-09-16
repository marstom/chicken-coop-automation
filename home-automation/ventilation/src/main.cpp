#include <Arduino.h>
#include <wifi_conn.h>

void setup() {
  Serial.begin(9600);
  Serial.println("Ventilation controller started");

  common::connectToWifiWithWait("YOUR_SSID", "YOUR_PASS", "ventilation");
}

void loop() {
    delay(1000);
    Serial.println("Ventilation controller running");
  // put your main code here, to run repeatedly:
}
