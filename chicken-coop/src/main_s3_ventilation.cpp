#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



#include "common/web.h"


void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 3000)
    {
    } // wait a moment for usb

    WiFi.onEvent(common::wifi_event::onWiFiEvent);
}

void loop()
{
}