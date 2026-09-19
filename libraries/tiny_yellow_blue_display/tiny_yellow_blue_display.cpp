/// TODO display driver...

#include "tiny_yellow_blue_display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <driver/gpio.h>


#define SDA_PIN GPIO_NUM_1
#define SCL_PIN GPIO_NUM_2

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int positionX = 0;
int positionY = 0;

namespace tiny_yellow_blue_display
{

  void init_wire(const int sdaPin, const int sclPin)
  {
    Wire.begin(sdaPin, sclPin);
  }

  void init()
  {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
      Serial.println("OLED not found");
      while (true)
      {
      }
    }
  }

  void displayText(const char* text, int x, int y)
  {
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x, y);
    display.println(text);
  }


  void displayBitmap(const uint8_t* bitmap, int x, int y)
  {
    display.drawBitmap(x, y, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  }

  void displayVerticalBar(const int height, int x, int y)
  {
    display.drawRect(x, y, 10, height, SSD1306_WHITE);
  }

  void clearDisplay()
  {
    display.clearDisplay();
  }


} // namespace tiny_yellow_blue_display

// void taskDisplay(void* pvParameters);

// void setup()
// {
//   Serial.begin(115200);

//   Wire.begin(SDA_PIN, SCL_PIN);

//   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
//   {
//     Serial.println("OLED not found");
//     while (true)
//       ;
//   }

//   display.clearDisplay();

//   display.setTextColor(SSD1306_WHITE);
//   display.setTextSize(2);
//   // display.println("Hello!");
//   // display.setCursor(1, 15);
//   // display.println("WORLD!");
//   // display.setCursor(1, 45);
//   // display.println("Hej!");
//   display.display();

//   // temperature sensor

//   my_am2320::init(SDA_PIN, SCL_PIN);

//   xTaskCreate(taskDisplay, "taskDisplay", 4096, NULL, 1, NULL);
// }

// void loop()
// {
//   // display.clearDisplay();
//   // positionX += 1;
//   // positionY += 1;
//   // display.setCursor(positionX, positionY);
//   // display.println("Tomek");
//   // display.display();
//   // // delay(80);

//   // if (positionX > SCREEN_WIDTH) {
//   //     positionX = 0;
//   // }
//   // if (positionY > SCREEN_HEIGHT) {
//   //     positionY = 0;
//   // }
// }

// void taskDisplay(void* pvParameters)
// {
//   float temperature = 0.0;
//   float humidity = 0.0;
//   while (true)
//   {
//     //     display.clearDisplay();
//     //     positionX += 1;
//     //     positionY += 1;
//     //     display.setCursor(positionX, positionY);
//     //     display.println("Tomek");
//     //     display.display();
//     //     delay(50);

//     //     if (positionX > SCREEN_WIDTH) {
//     //         positionX = 0;
//     //     }
//     //     if (positionY > SCREEN_HEIGHT) {
//     //         positionY = 0;
//     Serial.println(my_am2320::measure_temperature());
//     //     }

//     display.setCursor(positionX, positionY);
//     temperature = my_am2320::measure_temperature();
//     humidity = my_am2320::measure_humidity();
//     display.println("Temp: ");
//     display.println(temperature);
//     display.println("Hum: ");
//     display.println(humidity);
//     display.display();
//     vTaskDelay(500 / portTICK_PERIOD_MS);
//     display.clearDisplay();
//   }
// }