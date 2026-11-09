#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <RMaker.h>
#include <WiFi.h>
#include <WiFiProv.h>

#include "secrets/secrets_local.h"

// local libs
#include "uart_utils.h"
// dioda
static const uint8_t RGB_LED_PIN = 48; // WS2812 on ESP32-S3-DevKitM-1
static bool rgb_on = false;
static int hue = 0;          // 0..360
static int saturation = 100; // 0..100
static int brightness = 30;  // 0..100

static const char* service_name = "PROV_biuro";
static const char* pop = "abcd1234"; // proof of possession, shown in the QR code

static Device* led_device;
static bool diode_on = false;

void taskDeviceControl(void* pvParameters);
void sysProvEvent(arduino_event_t* sys_event);
static void makeLedButtonDevice(Node& node);
void waitWiFiConnected();
static void applyRgb();

void write_callback(Device* device, Param* param, const param_val_t val, void* priv_data,
                    write_ctx_t* ctx);
// void lightShortTheDiode();

void setup()
{
  common::connectToUartWithWait();

  // setup rMaker - must be before provision wifi
  Node node = RMaker.initNode("Biuro Kontroler");
  makeLedButtonDevice(node);

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
      // lightShortTheDiode();
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

  // Param toggle("Zapal", "custom.param.toggle", value(false), PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
  // toggle.addUIType(ESP_RMAKER_UI_TOGGLE); 
  // led_device->addParam(toggle);
  // led_device->addCb(write_callback);
  led_device->addPowerParam(false, "Zapal");

  // galeczka mala

  Param hue_p("Hue", ESP_RMAKER_PARAM_HUE, value(hue), PROP_FLAG_READ | PROP_FLAG_WRITE);
  hue_p.addUIType(ESP_RMAKER_UI_HUE_CIRCLE); // koło zamiast suwaka
  hue_p.addBounds(value(0), value(360), value(1));

  led_device->addParam(hue_p);
  led_device->addSaturationParam(saturation); // suwak 0..100
  led_device->addBrightnessParam(brightness); // suwak 0..100
  led_device->assignPrimaryParam(led_device->getParamByName("Zapal"));
  led_device->addCb(write_callback);

  node.addDevice(*led_device);
}

void write_callback(Device* device, Param* param, const param_val_t val, void* priv_data,
                    write_ctx_t* ctx)
{
  const char* name = param->getParamName();

  if (strcmp(name, "Zapal") == 0)
    rgb_on = val.val.b;
  else if (strcmp(name, "Hue") == 0)
    hue = val.val.i;
  else if (strcmp(name, "Saturation") == 0)
    saturation = val.val.i;
  else if (strcmp(name, "Brightness") == 0)
    brightness = val.val.i;

  applyRgb();
  param->updateAndReport(val); // confirm back to the app
}

// void lightShortTheDiode()
// {
//   Serial.println("[rmaker] Zapalam diode na 5s!");
//   led_device->updateAndReportParam("LED Stan", true);
//   neopixelWrite(48, 0, 0, 40);

//   vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);
//   led_device->updateAndReportParam("LED Stan", false);
//   neopixelWrite(48, 0, 0, 0);

//   Serial.println("[rmaker] Gasze diode!");
// }

static void applyRgb()
{
  if (!rgb_on)
  {
    neopixelWrite(48, 0, 0, 0);
    return;
  }
  float h = hue / 60.0f;
  float s = saturation / 100.0f;
  float v = brightness / 100.0f;
  float c = v * s;
  float x = c * (1 - fabsf(fmodf(h, 2) - 1));
  float m = v - c;
  float r = 0, g = 0, b = 0;
  switch ((int)h % 6)
  {
  case 0:
    r = c;
    g = x;
    break;
  case 1:
    r = x;
    g = c;
    break;
  case 2:
    g = c;
    b = x;
    break;
  case 3:
    g = x;
    b = c;
    break;
  case 4:
    r = x;
    b = c;
    break;
  case 5:
    r = c;
    b = x;
    break;
  }
  neopixelWrite(48, (r + m) * 255, (g + m) * 255, (b + m) * 255);
}