// PN532 I2C RFID reader support for the relay controller.

#include "relay_controller/rfid_board.h"

#include <Adafruit_PN532.h>
#include <Wire.h>

#include "mqtt_comm.h"

namespace relay_controller
{
    namespace
    {
        Adafruit_PN532 nfc(PN532_IRQ_PIN, PN532_RESET_PIN);

        constexpr const char *AUTHORIZED_UIDS[] = {
            "6deb45",
            "544b21",
            "252b9e7800",
            "198e93",
            "146c9f",
            "42f8d12e95b80",
        };

        String uidToString(const uint8_t *uid, uint8_t uidLength)
        {
            String uidString;
            uidString.reserve(uidLength * 2);
            for (uint8_t i = 0; i < uidLength; i++)
            {
                if (uid[i] < 0x10)
                {
                    uidString += '0';
                }
                uidString += String(uid[i], HEX);
            }
            return uidString;
        }

        bool isAuthorizedUid(const String &uid)
        {
            for (const char *authorizedUid : AUTHORIZED_UIDS)
            {
                if (uid == authorizedUid)
                {
                    return true;
                }
            }
            return false;
        }

        void requestDoorOpen()
        {
            if (!communication::relayQueue)
            {
                Serial.println("RFID access granted but relay queue is not initialized");
                return;
            }

            communication::RelayCommand cmd;
            cmd.on = true;
            xQueueSend(communication::relayQueue, &cmd, 0);
        }
    }

    bool setupRfid(int8_t sdaPin, int8_t sclPin)
    {
        Serial.println("PN532 NFC/RFID Reader - I2C setup");

        if (sdaPin >= 0 && sclPin >= 0)
        {
            Wire.begin(sdaPin, sclPin);
        }
        else
        {
            Wire.begin();
        }

        nfc.begin();
        const uint32_t versiondata = nfc.getFirmwareVersion();
        if (!versiondata)
        {
            Serial.println("Didn't find PN532 - RFID door access disabled");
            return false;
        }

        Serial.print("Found PN532 with firmware version: ");
        Serial.print((versiondata >> 24) & 0xFF, DEC);
        Serial.print('.');
        Serial.println((versiondata >> 16) & 0xFF, DEC);

        nfc.SAMConfig();
        Serial.println("Waiting for an ISO14443A Card (MIFARE, NFC tag)...");

        xTaskCreate(rfidTask, "rfidTask", 4096, NULL, 1, NULL);
        return true;
    }

    void rfidTask(void *pvParameters)
    {
        (void)pvParameters;

        uint8_t uid[7];
        uint8_t uidLength = 0;

        for (;;)
        {
            const bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
            if (success)
            {
                const String uidString = uidToString(uid, uidLength);

                Serial.print("Found RFID card, UID: ");
                Serial.println(uidString);

                if (isAuthorizedUid(uidString))
                {
                    Serial.println("RFID access granted - opening door");
                    requestDoorOpen();
                }
                else
                {
                    Serial.println("RFID access denied");
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}
