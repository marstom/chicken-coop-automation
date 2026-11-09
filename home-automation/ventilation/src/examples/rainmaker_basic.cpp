#include <Arduino.h>
#include <WiFi.h>
#include <WiFiProv.h>
#include "secrets/secrets_local.h"

// local libs
#include "uart_utils.h"

static const char* service_name = "PROV_biuro";
static const char* pop = "abcd1234"; // proof of possession, shown in the QR code

void taskDeviceControl(void* pvParameters);
void sysProvEvent(arduino_event_t* sys_event);
void waitWiFiConnected();

void setup()
{
  common::connectToUartWithWait();
  // wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "biuro", /*disableModemSleep=*/false);

  WiFi.setHostname("biuro");
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);
  waitWiFiConnected();

  xTaskCreate(taskDeviceControl, "taskDeviceControl", 2048, NULL, 1, NULL);
}

void loop() {}

void taskDeviceControl(void* pvParameters)
{
  for (;;)
  {
    Serial.println("Device Control task started....");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/// @brief
/// Display qr on the screen
/// @param sys_event
void sysProvEvent(arduino_event_t* e)
{
  switch (e->event_id)
  {
  case ARDUINO_EVENT_PROV_START:
    Serial.println("[prov] start, czekam na telefon");
    WiFiProv.printQR(service_name, pop, "ble");
    break;
  case ARDUINO_EVENT_PROV_CRED_RECV:
    Serial.printf("[prov] dostałem SSID: %s\n",
                  (const char*)e->event_info.prov_cred_recv.ssid);
    break;
  case ARDUINO_EVENT_PROV_CRED_FAIL:
    Serial.printf("[prov] błąd: %s\n",
                  e->event_info.prov_fail_reason == WIFI_PROV_STA_AUTH_ERROR
                      ? "złe hasło" : "nie znaleziono sieci");
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


void waitWiFiConnected()
{
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}