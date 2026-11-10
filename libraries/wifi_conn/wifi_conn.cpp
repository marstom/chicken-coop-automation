#include <WiFi.h>
#include <WiFiProv.h>

#include "wifi_conn.h"

namespace wifi
{

  /// connect to wifi directly, no provisioning, no QR code, no BLE, just connect to wifi
  void connectToWifiWithWait(const char* ssid, const char* pass, const char* hostname,
                             bool disableModemSleep)
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

  //// connect to wifi using wifi provisioning, you will use BT
  // Suitable for ESP BLE Prov or RainMaker BLE Prov
  // It prints QR code that allows to connect, or you can scan BT and provide POP
  void connectToWifiWithProvisioning(const char* hostname, const char* service_name,
                                     const char* pop, bool autoReconnect = true)
  {
    Serial.println("Connecting to WiFi using provisioning");
    WiFi.setHostname(hostname);
    WiFi.setAutoReconnect(autoReconnect);
    WiFi.onEvent(_sysProvEvent);

    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                            WIFI_PROV_SECURITY_1, pop, service_name);
    Serial.println("Provisioning started. Scan QR code or use BT to provide WiFi credentials.");

    WiFiProv.printQR(service_name, pop, "ble");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to WiFi");
  }

  void _sysProvEvent(arduino_event_t* e)
  {
    switch (e->event_id)
    {
    case ARDUINO_EVENT_PROV_START:
      Serial.println("[prov] start, czekam na telefon");
      WiFiProv.printQR(service_name, pop, "ble");
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.printf("[prov] dostałem SSID: %s\n", (const char*)e->event_info.prov_cred_recv.ssid);
      break;
    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.printf("[prov] błąd: %s\n", e->event_info.prov_fail_reason == WIFI_PROV_STA_AUTH_ERROR
                                             ? "złe hasło"
                                             : "nie znaleziono sieci");
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("[prov] dane poprawne");
      break;
    case ARDUINO_EVENT_PROV_END:
      Serial.println("[prov] koniec, BLE wyłączone");

      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("[wifi] IP: ");
      Serial.println(WiFi.localIP());
      break;
    default:
      break;
    }
  }

} // namespace wifi