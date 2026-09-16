#include <Arduino.h>
#include <wifi_conn.h>
#include "secrets/secrets_local.h"


void setup() {
  Serial.begin(9600);
  Serial.println("Ventilation controller started");

  common::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation");
}

void loop() {
    delay(1000);
    Serial.println("Ventilation controller running");
  // put your main code here, to run repeatedly:
}
