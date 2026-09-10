#include "hardware.h"
#include <Adafruit_NeoPixel.h>
namespace mcu::s3
{
    
    Adafruit_NeoPixel *strip = nullptr;
    
    void setupRGBLedOnBoard(int num_leds, int pin){
  // Setup LED
        strip = new Adafruit_NeoPixel(num_leds, pin, NEO_GRB + NEO_KHZ800);
        strip->begin();
        strip->setBrightness(50); // 0–255
        strip->show();
    }

    // From 0-255 range
    void changeBrightness(int brightness){
        strip->setBrightness(brightness);
        strip->show();
    }

    /**
     * Sets the color of the LED strip.
     *
     * @param color The color to set the LED strip to. It should be a value
     *              that can be passed to the `setPixelColor` function of the
     *              `Adafruit_NeoPixel` class.
     *
     * @return None.
     *
     * @throws None.
     */
    void setLedColor(uint32_t color){
        strip->setPixelColor(0, color);
        strip->show();
    }
}