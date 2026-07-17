#pragma once

#include <Arduino.h>

namespace relay_controller
{
    inline constexpr int8_t RFID_DEFAULT_SDA = -1;
    inline constexpr int8_t RFID_DEFAULT_SCL = -1;
    inline constexpr uint8_t PN532_IRQ_PIN = 2;
    inline constexpr uint8_t PN532_RESET_PIN = 3;

    /// Initialize PN532 and start the RFID reader task.
    /// Returns false when the reader is not detected, leaving the door usable
    /// through the other control paths.
    bool setupRfid(
        int8_t sdaPin = RFID_DEFAULT_SDA,
        int8_t sclPin = RFID_DEFAULT_SCL);

    void rfidTask(void *pvParameters);
}
