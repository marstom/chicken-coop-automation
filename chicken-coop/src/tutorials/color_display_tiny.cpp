#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <driver/gpio.h>

#include "common/am2320/am2320.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int positionX = 0;
int positionY = 0;

void taskDisplay(void* pvParameters);

void setup()
{
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED not found");
    while (true)
      ;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.display();
  my_am2320::init(SDA_PIN, SCL_PIN);
  xTaskCreate(taskDisplay, "taskDisplay", 4096, NULL, 1, NULL);
}

void loop() {}

void taskDisplay(void* pvParameters)
{
  float temperature = 0.0;
  float humidity = 0.0;
  while (true)
  {
    Serial.println(my_am2320::measure_temperature());
    display.setCursor(positionX, positionY);
    temperature = my_am2320::measure_temperature();
    humidity = my_am2320::measure_humidity();
    display.println("Temp: ");
    display.println(temperature);
    display.println("Hum: ");
    display.println(humidity);
    display.display();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    display.clearDisplay();
  }
}