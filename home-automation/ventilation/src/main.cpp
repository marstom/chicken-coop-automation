#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>

#include <wifi_conn.h>
#include <wifi_mdns.h>
#include <uart_utils.h>
#include "secrets/secrets_local.h"

WebServer server(80);

void setup()
{
  common::connectToUartWithWait();

  wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation", true);
  Serial.println("Ventilation controller started");

  wifi::setupMdns("ventilation");
  // if (MDNS.begin("ventilation"))
  // {
  //   Serial.println("mDNS: ventilation.local");
  // }
  // MDNS.addService("http", "tcp", 80);
  // MDNS.addService("mqtt", "tcp", 1883);
  // MDNS.addService("ota", "tcp", 3232);

  server.on("/", []() {
      server.send(200, "text/plain", "Hello");
  });
  server.begin();

}


void loop()
{
  delay(10);
  server.handleClient();
}
