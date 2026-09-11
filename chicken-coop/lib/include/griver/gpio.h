#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

namespace driver::gpio
{
    // Stable GPIO numbers that do not depend on board-specific Arduino
    // variants exposing compile-time aliases such as D0/D4.
    static constexpr uint8_t D0 = 2;
    static constexpr uint8_t D1 = 3;
    static constexpr uint8_t D2 = 4;
    static constexpr uint8_t D3 = 5;
    static constexpr uint8_t D4 = 6;
    static constexpr uint8_t D5 = 7;
    static constexpr uint8_t D6 = 8;
    static constexpr uint8_t D7 = 20;
    static constexpr uint8_t D8 = 8;
    static constexpr uint8_t D9 = 9;
    static constexpr uint8_t D10 = 10;

    static constexpr uint8_t SDA = 6;
    static constexpr uint8_t SCL = 7;

    // Re-export the ESP-IDF GPIO enum constants as driver::gpio names.
    static constexpr gpio_num_t GPIO_NUM_1 = gpio_num_t::GPIO_NUM_1;
    static constexpr gpio_num_t GPIO_NUM_2 = gpio_num_t::GPIO_NUM_2;
}
