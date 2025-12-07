// This is RFID module hello worl#include <Arduino.h>
#include <WiFi.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>

// #define SDA_PIN 8
// #define SCL_PIN 9

#define PN532_ADDR 0x24


void sendCommand(uint8_t *cmd, size_t len) {
  Wire.beginTransmission(PN532_ADDR);
  Wire.write(cmd, len);
  Wire.endTransmission();
}

void waitReady(const char *phase) {
  uint8_t status = 0;
  unsigned long start = millis();
  do {
    Wire.requestFrom(PN532_ADDR, 1);
    if (Wire.available()) status = Wire.read();
    if (millis() - start > 1000) {
      Serial.printf("Timeout waiting for %s\n", phase);
      return;
    }
    delay(5);
  } while (status != 0x01);
}


void justTest()
{
    Wire.begin(8,9);
    delay(1000);

    Serial.println("🔍 Sending GetFirmwareVersion command to PN532...");

    {
        // Build command frame (GetFirmwareVersion = 0x02)
        uint8_t cmd[] = {
            0x00, 0x00, 0xFF, // preamble + start
            0x02, 0xFE,       // LEN + LCS
            0xD4, 0x02,       // TFI + command
            0x2A, 0x00        // DCS + postamble (checksum 0x2A)
        };

        // Send frame
        Wire.beginTransmission(PN532_ADDR);
        Wire.write(cmd, sizeof(cmd));
        Wire.endTransmission();

        delay(100); // wait for chip to respond

        // Request 24 bytes of response
        Wire.requestFrom(PN532_ADDR, 24);
        Serial.print("📡 Response: ");
        while (Wire.available())
        {
            Serial.printf("0x%02X ", Wire.read());
        }
        Serial.println();
    }

    delay(1000);
    /// read
    {
        uint8_t cmd[] = {
            0x00, 0x00, 0xFF,
            0x04, 0xFC,
            0xD4, 0x4A, 0x01, 0x00, // TFI + Command + MaxTg + BrTy
            0x29, 0x00              // DCS + Postamble
        };

        // Send frame
        Wire.beginTransmission(PN532_ADDR);
        Wire.write(cmd, sizeof(cmd));

        Wire.endTransmission();
        delay(400); // wait for chip to respond
        // Request 24 bytes of response
        Wire.requestFrom(PN532_ADDR, 24);
        Serial.print("📡 Response: ");
        while (Wire.available())
        {
            Serial.printf("0x%02X ", Wire.read());
        }
        Serial.println();
    }

    // read card
    {
        // 00 00 FF 04 FC D4 4A 01 00 29 00
        uint8_t cmd[] = {
            0x00, 0x00, 0xFF,
            0x04, 0xFC,
            0xD4, 0x4A, 0x01, 0x00, // command
            0x29, 0x00};
        // Send frame
        Wire.beginTransmission(PN532_ADDR);
        Wire.write(cmd, sizeof(cmd));

        Wire.endTransmission();
        delay(400); // wait for chip to respond
        // Request 24 bytes of response
        Wire.requestFrom(PN532_ADDR, 32);
        Serial.print("📡 Response: ");
        uint8_t buffer[32];
        int i = 0;
        Serial.println("REading card....");

        while (Wire.available() && i < 32)
        {
            buffer[i] = Wire.read();
            Serial.printf("%02X ", buffer[i]);
            i++;
        }
        Serial.println();
        // Step 3: look for UID
        for (int j = 0; j < i - 7; j++)
        {
            // heuristic: response should contain D5 4B (PN532→host + InListPassiveTarget response)
            if (buffer[j] == 0xD5 && buffer[j + 1] == 0x4B)
            {
                uint8_t uidLen = buffer[j + 6];
                Serial.print("✅ UID: ");
                for (uint8_t k = 0; k < uidLen; k++)
                {
                    Serial.printf("%02X ", buffer[j + 7 + k]);
                }

                Serial.println();
            }
        }
    }
    Serial.println("END");
}

void setup() {
  Serial.begin(9600);
  delay(500);
//   Wire.begin(10,11);  // slower 100kHz is more stable for PN532
  delay(100);
  Serial.println("🔌 PN532 raw I2C test");
    justTest();
  // 1️⃣ Wake up PN532
//   uint8_t wakeup[] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
//   Serial.println("🌙 Sending wakeup...");
//   sendCommand(wakeup, sizeof(wakeup));
  delay(100);

  // 2️⃣ Send "InListPassiveTarget" command
//   Serial.println("🔍 Sending InListPassiveTarget...");
//   uint8_t detectCardCmd[] = {
//     0x00, 0x00, 0xFF,
//     0x04, 0xFC,
//     0xD4, 0x4A, 0x01, 0x00,
//     0x29, 0x00
//   };
//   sendCommand(detectCardCmd, sizeof(detectCardCmd));

//   // 3️⃣ Wait for ACK
//   waitReady("ACK");
//   Wire.requestFrom(PN532_ADDR, 6);
//   Serial.print("ACK: ");
//   while (Wire.available()) Serial.printf("%02X ", Wire.read());
//   Serial.println();

//   // 4️⃣ Wait for response
//   waitReady("Response");
//   Wire.requestFrom(PN532_ADDR, 32);
//   Serial.print("Response: ");
//   uint8_t buffer[32];
//   int i = 0;
//   while (Wire.available() && i < 32) buffer[i++] = Wire.read(), Serial.printf("%02X ", buffer[i-1]);
//   Serial.println();

//   // 5️⃣ Extract UID
//   for (int j = 0; j < i - 7; j++) {
//     if (buffer[j] == 0xD5 && buffer[j + 1] == 0x4B) {
//       uint8_t uidLen = buffer[j + 6];
//       Serial.print("✅ UID: ");
//       for (uint8_t k = 0; k < uidLen; k++) Serial.printf("%02X ", buffer[j + 7 + k]);
//       Serial.println();
//     }
//   }
}



void loop()
{
    //   Serial.println("Hello World");
    // scanning
    // scanI2CDevices();
    delay(1000);
}

