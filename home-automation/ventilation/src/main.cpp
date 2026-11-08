#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <RMaker.h>
#include <WiFiProv.h>

// local libs
#include <my_am_2320_tca9548a.h>
#include <tiny_yellow_blue_display.h>
#include <uart_utils.h>
#include <wifi_conn.h>
// #include <wifi_mdns.h>

#include "secrets/secrets_local.h"

WebServer server(80);

// Written by taskTemperature, read by the /api handler. Aligned 32-bit floats,
// so a reader sees either the old or the new value, never a torn one.
float temperature_intake = NAN;
float humidity_intake = NAN;
float temperature_exhaust = NAN;
float humidity_exhaust = NAN;
float temperature_room = NAN;
float humidity_room = NAN;

// Application RainMaker for android, allows us go with home automation!

static const char* service_name = "PROV_ventilation";
static const char* pop = "abcd1234";  // proof of possession, shown in the QR code

static TemperatureSensor* sensor_intake;

void taskTemperature(void* pvParameters);
//
static TemperatureSensor* makeSensor(Node& node, const char* name);
void sysProvEvent(arduino_event_t* sys_event);
//

String buildJsonResponse();

void displayAllValuedOnDisplay(float temperature_intake, float humidity_intake,
                               float temperature_exhaust, float humidity_exhaust,
                               float temperature_room, float humidity_room);

void setup()
{
  common::connectToUartWithWait();
  // wifi::connectToWifiWithWait(WIFI_SSID, WIFI_PASS, "ventilation", true);

  my_am2320::init_wire(GPIO_NUM_1, GPIO_NUM_2);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Intake);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Exhaust);
  my_am2320::init_tca9548a_sensor(my_am2320::SensorId::Room);
  Serial.println("Ventilation controller started");

  // wifi::setupMdns("ventilation");

  tiny_yellow_blue_display::init();
  tiny_yellow_blue_display::clearDisplay();
  tiny_yellow_blue_display::displayText("Ventilation", 0, 0);
  tiny_yellow_blue_display::displayText("Temperature", 0, 12);
  tiny_yellow_blue_display::show();

  server.on("/", []() { server.send(200, "text/plain", "Hello"); });
  server.on("/api", []() { server.send(200, "application/json", buildJsonResponse()); });

  // Setup rainmaker node
  Node node =  RMaker.initNode("Ventilation Controller");
  sensor_intake = makeSensor(node, "Temperature Sensor");

  RMaker.enableOTA(OTA_USING_TOPICS);
  RMaker.enableTZService();
  RMaker.start();

  WiFi.setHostname("ventilation");
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  server.begin();

  xTaskCreate(taskTemperature, "Temperature Task", 2048 * 4, NULL, 1, NULL);
}

void loop()
{
  delay(10);
  server.handleClient();
}

void taskTemperature(void* pvParameters)
{

  while (true)
  {
    Serial.println("Measuring temperature and humidity from sensors...");
    temperature_intake = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Intake);
    humidity_intake = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Intake);
    Serial.printf("T Intake: %.2f C, H Intake: %.2f %%\n", temperature_intake, humidity_intake);
    sensor_intake->updateAndReportParam("Temperature", temperature_intake);
    sensor_intake->updateAndReportParam("Humidity", humidity_intake);

    temperature_exhaust =
        my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Exhaust);
    humidity_exhaust = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Exhaust);
    Serial.printf("T Exhaust: %.2f C, H Exhaust: %.2f %%\n", temperature_exhaust, humidity_exhaust);

    temperature_room = my_am2320::measure_temperature_from_sensor_x(my_am2320::SensorId::Room);
    humidity_room = my_am2320::measure_humidity_from_sensor_x(my_am2320::SensorId::Room);
    Serial.printf("T Room: %.2f C, H Room: %.2f %%\n", temperature_room, humidity_room);

    displayAllValuedOnDisplay(temperature_intake, humidity_intake, temperature_exhaust, humidity_exhaust, temperature_room, humidity_room);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

// A failed sensor read is NAN, which is not valid JSON, so it goes out as null.
static void appendReading(String& json, float value)
{
  if (isnan(value))
  {
    json += "null";
  }
  else
  {
    json += String(value, 2);
  }
}

/*
{
  "temperature": { "intake": 23.10, "exhaust": null, "room": null },
  "humidity":    { "intake": 65.70, "exhaust": null, "room": null }
}
*/
String buildJsonResponse()
{
  String json = "{\"temperature\":{\"intake\":";
  appendReading(json, temperature_intake);
  json += ",\"exhaust\":";
  appendReading(json, temperature_exhaust);
  json += ",\"room\":";
  appendReading(json, temperature_room);
  json += "},\"humidity\":{\"intake\":";
  appendReading(json, humidity_intake);
  json += ",\"exhaust\":";
  appendReading(json, humidity_exhaust);
  json += ",\"room\":";
  appendReading(json, humidity_room);
  json += "}}";
  return json;
}

void displayAllValuedOnDisplay(float temperature_intake, float humidity_intake,
                               float temperature_exhaust, float humidity_exhaust,
                               float temperature_room, float humidity_room)
{
  // ROUND all floats to 2 decimal places
  temperature_intake = round(temperature_intake * 100) / 100;
  humidity_intake = round(humidity_intake * 100) / 100;
  temperature_exhaust = round(temperature_exhaust * 100) / 100;
  humidity_exhaust = round(humidity_exhaust * 100) / 100;
  temperature_room = round(temperature_room * 100) / 100;
  humidity_room = round(humidity_room * 100) / 100;

  tiny_yellow_blue_display::clearDisplay();
  tiny_yellow_blue_display::displayText("Powietrze Tomek Pokoj", 0, 0);
  String line = "i " + String(temperature_intake) + "C";
  tiny_yellow_blue_display::displayText(line.c_str(), 0, 24);
  line = "" + String(humidity_intake) + "%";
  tiny_yellow_blue_display::displayText(line.c_str(), 54, 24);
  line = "e " + String(temperature_exhaust) + "C";
  tiny_yellow_blue_display::displayText(line.c_str(), 0, 12+24);
  line = "" + String(humidity_exhaust) + "%";
  tiny_yellow_blue_display::displayText(line.c_str(), 54, 12+24);
  line = "r " + String(temperature_room) + "C";
  tiny_yellow_blue_display::displayText(line.c_str(), 0, 24+24);
  line = "" + String(humidity_room) + "%";
  tiny_yellow_blue_display::displayText(line.c_str(), 54, 24+24);
  tiny_yellow_blue_display::show();
}




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
      Serial.println("Provisioning started with BLE");
      WiFiProv.printQR(service_name, pop, "ble");  // scan this with the RainMaker app
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // wifi::setupMdns("ventilation");  // start mDNS only after the network is up
      Serial.println("WiFi connected, provisioning done");
      break;
    default:
      break;
  }
}
