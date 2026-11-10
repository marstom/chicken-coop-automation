#pragma once
// WIFI operations
namespace wifi
{
  // disableModemSleep = true avoids random disconnects / MQTT keepalive
  // timeouts, but MUST be false on targets that also use BLE: the ESP32-C3
  // coexistence layer aborts unless WiFi modem sleep stays enabled.
  void connectToWifiWithWait(const char* ssid, const char* pass, const char* hostname,
                             bool disableModemSleep = true);

  void connectToWifiWithProvisioning(const char* hostname, const char* service_name,
                                     const char* pop, bool autoReconnect=true);
} // namespace wifi
