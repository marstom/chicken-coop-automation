#pragma once

#include <HTTPClient.h>

#include <WebServer.h>

extern WiFiClient net;
extern WebServer webServer;

void setupMDNS(const char *hostname);
void onWiFiEvent(WiFiEvent_t event);


void readSensorsToStrings();
void simpleWebPage();                   // for demo purposes that mDNS works
void taskWebServer(void *pvParameters); // simple web page for demo
void handleRootPage();                  // handle the page for above task
void handleJsonAPI();