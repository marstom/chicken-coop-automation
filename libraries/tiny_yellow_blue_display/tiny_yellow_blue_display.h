#include <Arduino.h>



namespace tiny_yellow_blue_display
{
    void init();
    void displayText(const char *text, int x, int y);
    void displayBitmap(const uint8_t *bitmap, int x, int y);
    void displayVerticalBar(const int height, int x, int y);
    void clearDisplay();
}