#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <RMaker.h>
// #include <WiFi.h>
// #include <WiFiProv.h>

#include "secrets/secrets_local.h"

// local libs
#include "uart_utils.h"
#include "wifi_conn.h"

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
static void makeLedButtonDevice(Node& node);
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

  wifi::connectToWifiWithProvisioning("biuro", service_name, pop, true);
  // provision wifi
  // WiFi.setHostname("biuro");
  // WiFi.setAutoReconnect(true);
  // WiFi.onEvent(sysProvEvent);
  // WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
  //                         WIFI_PROV_SECURITY_1, pop, service_name);
  // waitWiFiConnected();

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