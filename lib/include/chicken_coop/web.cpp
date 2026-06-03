#include "chicken_coop/web.h"
#include "chicken_coop/constants.h"
#include "mqtt_comm.h"
#include "chicken_coop/gauge_page_chicken_coop.h"

namespace chicken_coop
{

WebServer webServer(80);
WiFiClient net;
String temp = "";
String press = "";
String hum = "";
String alt = "";

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

} // namespace chicken_coop
