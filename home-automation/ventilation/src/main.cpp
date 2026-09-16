#include <Arduino.h>
#include <wifi_conn.h>
#include <uart_utils.h>
#include "secrets/secrets_local.h"

void setup()
{
  common::connectToUartWithWait();

  wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation");
  Serial.println("Ventilation controller started");

}

void loop()
{
  delay(1000);
  Serial.println("Ventilation controller running");
  // put your main code here, to run repeatedly:
}
