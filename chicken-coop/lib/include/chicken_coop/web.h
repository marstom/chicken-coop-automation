#pragma once

#include <HTTPClient.h>

#include <WebServer.h>
namespace chicken_coop
{
    extern WiFiClient net;
    extern WebServer webServer;

    void readSensorsToStrings();
    void simpleWebPage();                   // for demo purposes that mDNS works
    void taskWebServer(void *pvParameters); // simple web page for demo
    void handleRootPage();                  // handle the page for above task
    void handleJsonAPI();
}
