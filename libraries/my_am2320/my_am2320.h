#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

// PINS for s3
// TODO use constant
#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2

// c3
// #define SDA_PIN xx
// #define SCL_PIN xx
namespace my_am2320
{
  void init(const int sdaPin = SDA_PIN, const int sclPin = SCL_PIN);
  float measure_temperature();
  float measure_humidity();
} // namespace my_am2320
