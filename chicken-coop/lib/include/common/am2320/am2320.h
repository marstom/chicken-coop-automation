#pragma once

#include <Arduino.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#include <driver/gpio.h>

Adafruit_AM2320 am2320 = Adafruit_AM2320();

// PINS for s3
// TODO use constant
#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2

// c3
// #define SDA_PIN xx
// #define SCL_PIN xx
namespace my_am2320
{
    void init()
    {
        Wire.begin(SDA_PIN, SCL_PIN);

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