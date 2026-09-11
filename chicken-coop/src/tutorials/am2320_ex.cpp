/*
Exampe usage of temperature and humidity sensor AM2320
*/
#include "common/am2320/am2320.h"
#include "driver/gpio.h"
// PINS for i2c

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10); // hang out until serial port opens
  }

  Serial.println("Adafruit AM2320 Basic Test");
  my_am2320::init();
  int p = GPIO_NUM_2;
}

void loop()
{
  Serial.print("Temp: ");
  Serial.println(my_am2320::measure_temperature());
  Serial.print("Hum: ");
  Serial.println(my_am2320::measure_humidity());

  delay(2000);
}