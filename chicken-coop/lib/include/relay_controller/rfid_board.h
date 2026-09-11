#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

namespace relay_controller
{
    inline constexpr uint8_t RFID_DEFAULT_SDA = GPIO_NUM_6; // XIAO ESP32-C3: D4
    inline constexpr uint8_t RFID_DEFAULT_SCL = GPIO_NUM_7; // XIAO ESP32-C3: D5
    inline constexpr uint8_t PN532_IRQ_PIN = GPIO_NUM_2;
    inline constexpr uint8_t PN532_RESET_PIN = GPIO_NUM_3;

    /// Initialize PN532 and start the RFID reader task.
    /// Returns false when the reader is not detected, leaving the door usable
    /// through the other control paths.
    bool setupRfid(
        uint8_t sdaPin = RFID_DEFAULT_SDA,
        uint8_t sclPin = RFID_DEFAULT_SCL);

    void rfidTask(void *pvParameters);
}
