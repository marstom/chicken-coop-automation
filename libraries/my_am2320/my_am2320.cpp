
#include "my_am2320.h"
#include <Arduino.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
// #include <driver/gpio.h>

Adafruit_AM2320 am2320 = Adafruit_AM2320();

namespace my_am2320
{
    void init(const int sdaPin, const int sclPin)
    {
        Wire.begin(sdaPin, sclPin);

        if (!am2320.begin())
        {
            Serial.println("AM2320 not found");
        }
    }

    float measure_temperature()
    {
        return am2320.readTemperature();
    }

    float measure_humidity()
    {
        return am2320.readHumidity();
    }
}