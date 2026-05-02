#pragma once
#include <Adafruit_NeoPixel.h>

namespace mcu::s3 {
    int pin = 1;
    void setupRGBLedOnBoard(int num_leds, int pin);
}