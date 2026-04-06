// mDNS support
#include <ESPmDNS.h>

#include "chicken_coop/web.h"
#include "chicken_coop/constants.h"
#include "chicken_coop/web.h"
#include "mqtt_comm.h"
#include "chicken_coop/gauge_page_chicken_coop.h"

WebServer webServer(80);
WiFiClient net;
String temp = "";
String press = "";
String hum = "";
String alt = "";

// set hostname for my chickenCOOP server
// https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32c3/api-reference/protocols/mdns.html
void setupMDNS(const char *hostname)
{

    // initialize mDNS service
    esp_err_t err = mdns_init();
    if (err)
    {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    // set hostname
    mdns_hostname_set(hostname);
    // set default instance
    mdns_instance_name_set("Chicken Coop");

    Serial.print("mDNS started: http://");
    Serial.print(hostname);
    Serial.println(".local");

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // add our services
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    mdns_service_add(NULL, "_ota", "_tcp", 3232, NULL, 0);
    mdns_service_add(NULL, "mqtt", "_tcp", 1883, NULL, 0);
}

void onWiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("WiFi connected, IP: ");
        Serial.println(WiFi.localIP());
        setupMDNS(MDNS_HOSTNAME);
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("WiFi disconnected, waiting for reconnect...");
        break;
    default:
        break;
    }
}



void simpleWebPage()
{
    webServer.on("/", handleRootPage);
    webServer.on("/api/v1/", handleJsonAPI);
    webServer.on("/health", []()
                 { webServer.send(200, "application/json", "{\"status\":\"ok\"}"); });
    webServer.on("/favicon.ico", []()
                 { webServer.send(200, "image/x-icon", ""); });
    webServer.onNotFound([]()
                         { webServer.send(404, "text/plain", "404: Not Found"); });
    webServer.begin();
}
void readSensorsToStrings()
{
    char *webBuff = NULL;
    char *type = NULL;

    communication::WebMessage webMsg;
    while (xQueueReceive(communication::webQueue, &webMsg, 0) != pdFALSE)
    {
        webBuff = webMsg.getBuffer();
        type = webMsg.getMessageType();
        // String tt = String(type);
        bool exists = type != NULL && webBuff != NULL;
        if (exists && strcmp(type, communication::WebMessage::temperature) == 0)
        {
            temp = String(webBuff);
        }
        else if (exists && strcmp(type, communication::WebMessage::pressure) == 0)
        {
            press = String(webBuff);
        }
        else if (exists && strcmp(type, communication::WebMessage::altitude) == 0)
        {
            alt = String(webBuff);
        }

        else if (exists && strcmp(type, communication::WebMessage::humidity) == 0)
        {
            hum = String(webBuff);
        }
    }
}


void handleRootPage()
{
    readSensorsToStrings();

    webServer.send(200, "text/html", INDEX_HTML);
}

void handleJsonAPI()
{
    readSensorsToStrings();
    String json = "{";
    json += "\"temperature\":\"" + temp + "\",";
    json += "\"pressure\":\"" + press + "\",";
    json += "\"humidity\":\"" + hum + "\",";
    json += "\"altitude\":\"" + alt + "\"";
    json += "}";

    webServer.send(200, "application/json", json);
}



