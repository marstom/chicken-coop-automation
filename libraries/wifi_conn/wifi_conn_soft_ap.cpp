#include "wifi_conn_soft_ap.h"
#include "qrcode.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiGeneric.h>
#include <WiFiProv.h>
#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>
bool wifiLowLevelInit(bool persistent);   // <-- tutaj, w przestrzeni globalnej

namespace
{ // this class is private
  class WiFiProvConnection
  {
  public:
    WiFiProvConnection(const char* service_name, const char* pop)
    {
      WiFiProvConnection::service_name = service_name;
      WiFiProvConnection::pop = pop;
    }

    void connect(const char* hostname, bool autoReconnect)
    {
      WiFi.setHostname(hostname);
      WiFi.setAutoReconnect(autoReconnect);
      WiFi.onEvent(sysProvEvent); // eventy ARDUINO_EVENT_PROV_* działają jak dotąd

      wifi_prov_mgr_config_t config = {};
      config.scheme = wifi_prov_scheme_softap; // tylko SoftAP, bez BLE
      config.scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE;

      wifiLowLevelInit(true);
      if (wifi_prov_mgr_init(config) != ESP_OK)
      {
        Serial.println("[prov] init failed");
        return;
      }

      bool provisioned = false;
      wifi_prov_mgr_is_provisioned(&provisioned);

      if (!provisioned)
      {
        // service_name = nazwa hotspotu ESP (np. "kurnikService")
        // service_key  = hasło do hotspotu (min. 8 znaków) albo nullptr dla otwartej sieci
        wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1, pop, service_name, nullptr);
      }
      else
      {
        esp_wifi_start();
        wifi_prov_mgr_deinit();
        WiFi.begin(); // łączy się z zapisanymi w NVS danymi
      }
    }

  private:
    static inline const char* service_name = nullptr;
    static inline const char* pop = nullptr;

    static void sysProvEvent(arduino_event_t* e)
    {
      switch (e->event_id)
      {
      case ARDUINO_EVENT_PROV_START:
      {
        Serial.println("[prov] start, połącz się z hotspotem ESP");
        char payload[150];
        snprintf(payload, sizeof(payload),
                 "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"softap\"}",
                 service_name, pop);
        esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
        esp_qrcode_generate(&cfg, payload);
        break;
      }

      case ARDUINO_EVENT_PROV_CRED_RECV:
        Serial.printf("[prov] dostałem SSID: %s\n", (const char*)e->event_info.prov_cred_recv.ssid);
        break;
      case ARDUINO_EVENT_PROV_CRED_FAIL:
        Serial.printf("[prov] błąd: %s\n",
                      e->event_info.prov_fail_reason == WIFI_PROV_STA_AUTH_ERROR
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
  };
}; // namespace

namespace wifi
{

  void connectToWifiWithSoftAPProvisioning(const char* hostname, const char* service_name,
                                     const char* pop, bool autoReconnect)
  {
    static WiFiProvConnection conn(service_name, pop);
    conn.connect(hostname, autoReconnect);
  }
} // namespace wifi
