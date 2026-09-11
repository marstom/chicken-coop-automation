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

  Serial.println(".....");
  delay(2000);
  Serial.println("Adafruit AM2320 Basic Test");
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    Serial.println("We have s3 with debugger");
    my_am2320::init(GPIO_NUM_1, GPIO_NUM_2);
  #elif defined(CONFIG_IDF_TARGET_ESP32C3)
    Serial.println("We have c3");
    my_am2320::init(GPIO_NUM_6, GPIO_NUM_7);
  #endif
}

void loop()
{
  Serial.print("Temp: ");
  Serial.println(my_am2320::measure_temperature());
  Serial.print("Hum: ");
  Serial.println(my_am2320::measure_humidity());

  delay(2000);
}