
#include <RMaker.h>
#include <WiFi.h>
#include <WiFiProv.h>

static const char* service_name = "PROV_ventilation";
static const char* pop = "abcd1234";  // proof of possession, shown in the QR code

static TemperatureSensor* sensor_intake;
static TemperatureSensor* sensor_exhaust;
static TemperatureSensor* sensor_room;

static TemperatureSensor* makeSensor(Node& node, const char* name)
{
  auto* s = new TemperatureSensor(name);
  Param humidity("Humidity", "custom.param.humidity", value(0.0f), PROP_FLAG_READ);
  s->addParam(humidity);
  node.addDevice(*s);
  return s;
}

void sysProvEvent(arduino_event_t* sys_event)
{
  switch (sys_event->event_id)
  {
    case ARDUINO_EVENT_PROV_START:
      WiFiProv.printQR(service_name, pop, "ble");  // scan this with the RainMaker app
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifi::setupMdns("ventilation");  // start mDNS only after the network is up
      break;
    default:
      break;
  }
}

void setup()
{
  common::connectToUartWithWait();
  // ... sensors + display init as before ...

  Node node = RMaker.initNode("Ventilation");
  sensor_intake  = makeSensor(node, "Intake");
  sensor_exhaust = makeSensor(node, "Exhaust");
  sensor_room    = makeSensor(node, "Room");

  RMaker.enableOTA(OTA_USING_TOPICS);  // optional: OTA updates from the RainMaker dashboard
  RMaker.enableTZService();
  RMaker.start();

  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);

  server.on(...);  // WebServer unchanged
  server.begin();
  xTaskCreate(taskTemperature, ...);
}
