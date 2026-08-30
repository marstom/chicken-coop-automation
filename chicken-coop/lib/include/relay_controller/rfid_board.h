#pragma once

#include <Arduino.h>

namespace relay_controller
{
    inline constexpr uint8_t RFID_DEFAULT_SDA = SDA; // XIAO ESP32-C3: D4 / GPIO6
    inline constexpr uint8_t RFID_DEFAULT_SCL = SCL; // XIAO ESP32-C3: D5 / GPIO7
    inline constexpr uint8_t PN532_IRQ_PIN = 2;
    inline constexpr uint8_t PN532_RESET_PIN = 3;

    /// Initialize PN532 and start the RFID reader task.
    /// Returns false when the reader is not detected, leaving the door usable
    /// through the other control paths.
    bool setupRfid(
        uint8_t sdaPin = RFID_DEFAULT_SDA,
        uint8_t sclPin = RFID_DEFAULT_SCL);

    void rfidTask(void *pvParameters);
}
