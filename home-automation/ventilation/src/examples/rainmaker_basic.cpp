#include <Arduino.h>
#include <RMaker.h>
#include <WiFi.h>
#include <WiFiProv.h>

#include "secrets/secrets_local.h"

// local libs
#include "uart_utils.h"

static const char* service_name = "PROV_biuro";
static const char* pop = "abcd1234"; // proof of possession, shown in the QR code

static Device* led_device;
static bool diode_on = false;

void taskDeviceControl(void* pvParameters);
void sysProvEvent(arduino_event_t* sys_event);
void waitWiFiConnected();

void write_callback(Device* device, Param* param, const param_val_t val, void* priv_data,
                    write_ctx_t* ctx);
void lightShortTheDiode();

void setup()
{
  common::connectToUartWithWait();

  // setup rMaker - must be before provision wifi
  Node node = RMaker.initNode("Biuro Kontroler");

  RMaker.start();

  // provision wifi
  WiFi.setHostname("biuro");
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);
  waitWiFiConnected();

  // setup tasks
  xTaskCreate(taskDeviceControl, "taskDeviceControl", 2048, NULL, 1, NULL);
}

void loop() {}

void taskDeviceControl(void* pvParameters)
{
  for (;;)
  {
    Serial.println("Device Control task started....");
    if (diode_on)
    {
      diode_on = false;
      lightShortTheDiode();
    }
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

void waitWiFiConnected()
{
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

static void makeLedButtonDevice(Node& node)
{
  led_device = new Device("LedButton", "esp.device.other");
  // led_button->addNameParam(); // bez-nazwa z kodu

  Param toggle("Zapal", "custom.param.toggle", value(false), PROP_FLAG_WRITE);
  toggle.addUIType(ESP_RMAKER_UI_TRIGGER); // tylko pisanie
  led_device->addParam(toggle);

  Param state("LED Stan", "custom.param.state", value(false), PROP_FLAG_READ);
  state.addUIType(ESP_RMAKER_UI_TOGGLE); // tylko odczyt
  led_device->addParam(state);

  Param click("Click", "custom.param.click", value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
  // click.addUIType(ESP_RMAKER_UI_BUTTON);
  // led_button->addParam(click);

  led_device->addCb(write_callback);
  node.addDevice(*led_device);
}

void write_callback(Device* device, Param* param, const param_val_t val, void* priv_data,
                    write_ctx_t* ctx)
{
  if (strcmp(param->getParamName(), "Zapal") == 0 && val.val.b)
  {
    Serial.println("[rmaker] Zapalam diode!");
    diode_on = true;
    // tu akcja
  }
}

void lightShortTheDiode()
{
  Serial.println("[rmaker] Zapalam diode na 1s!");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("[rmaker] Gasze diode!");
}