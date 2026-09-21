#pragma once

#include <Arduino.h>

namespace tiny_yellow_blue_display
{
  void init_wire();
  void init();
  void displayText(const char* text, int x, int y);
  void displayTextBig(const char* text, int x, int y);
  void displayBitmap(const uint8_t* bitmap, int x, int y);
  void displayVerticalBar(const int height, int x, int y);
  void clearDisplay();
  // Drawing only updates the RAM buffer; this pushes it to the panel.
  void show();
} // namespace tiny_yellow_blue_display