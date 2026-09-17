#include <Arduino.h>
#include <ESPmDNS.h>
#include <wifi_conn.h>
#include <uart_utils.h>
#include "secrets/secrets_local.h"

void setup()
{
  common::connectToUartWithWait();

  wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation", true);
  Serial.println("Ventilation controller started");
  if (MDNS.begin("ventilation"))
  {
    Serial.println("mDNS: ventilation.local");
  }
}

void loop()
{
  delay(1000);
  Serial.println("Ventilation controller running");
  // put your main code here, to run repeatedly:
}
