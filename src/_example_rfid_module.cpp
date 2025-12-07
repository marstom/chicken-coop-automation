// This is RFID module hello worl#include <Arduino.h>
#include <WiFi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>

#define I2C_SDA 8
#define I2C_SCL 9

#define RFID_DEVICE_ADDR 0x24

void taskRFID(void *pvParameters);
void scanI2CDevices();

void sendCommand(uint8_t *cmd, size_t len) {
  Wire.beginTransmission(RFID_DEVICE_ADDR);
  Wire.write(cmd, len);
  Wire.endTransmission();
}

void waitReady(const char *phase) {
  uint8_t status = 0;
  unsigned long start = millis();
  do {
    Wire.requestFrom(RFID_DEVICE_ADDR, 1);
    if (Wire.available()) status = Wire.read();
    if (millis() - start > 1000) {
      Serial.printf("Timeout waiting for %s\n", phase);
      return;
    }
    delay(5);
  } while (status != 0x01);
}


// void justTest()
// {
//     Wire.begin(I2C_SDA, I2C_SCL);
//     delay(100);

//     Serial.println("🔍 Sending GetFirmwareVersion command to PN532...");

//     {
//         // Build command frame (GetFirmwareVersion = 0x02)
//         uint8_t cmd[] = {
//             0x00, 0x00, 0xFF, // preamble + start
//             0x02, 0xFE,       // LEN + LCS
//             0xD4, 0x02,       // TFI + command
//             0x2A, 0x00        // DCS + postamble (checksum 0x2A)
//         };

//         // Send frame
//         Wire.beginTransmission(RFID_DEVICE_ADDR);
//         Wire.write(cmd, sizeof(cmd));
//         Wire.endTransmission();

//         delay(10); // wait for chip to respond

//         // Request 24 bytes of response
//         Wire.requestFrom(RFID_DEVICE_ADDR, 24);
//         Serial.print("📡 Response: ");
//         while (Wire.available())
//         {
//             Serial.printf("0x%02X ", Wire.read());
//         }
//         Serial.println();
//     }

//     delay(1000);
//     /// read
//     {
//         uint8_t cmd[] = {
//             0x00, 0x00, 0xFF,
//             0x04, 0xFC,
//             0xD4, 0x4A, 0x01, 0x00, // TFI + Command + MaxTg + BrTy
//             0x29, 0x00              // DCS + Postamble
//         };

//         // Send frame
//         Wire.beginTransmission(RFID_DEVICE_ADDR);
//         Wire.write(cmd, sizeof(cmd));

//         Wire.endTransmission();
//         delay(400); // wait for chip to respond
//         // Request 24 bytes of response
//         Wire.requestFrom(RFID_DEVICE_ADDR, 24);
//         Serial.print("📡 Response: ");
//         while (Wire.available())
//         {
//             Serial.printf("0x%02X ", Wire.read());
//         }
//         Serial.println();
//     }

//     // read card
//     {
//         // 00 00 FF 04 FC D4 4A 01 00 29 00
//         uint8_t cmd[] = {
//             0x00, 0x00, 0xFF,
//             0x04, 0xFC,
//             0xD4, 0x4A, 0x01, 0x00, // command
//             0x29, 0x00};
//         // Send frame
//         Wire.beginTransmission(RFID_DEVICE_ADDR);
//         Wire.write(cmd, sizeof(cmd));

//         Wire.endTransmission();
//         delay(400); // wait for chip to respond
//         // Request 24 bytes of response
//         Wire.requestFrom(RFID_DEVICE_ADDR, 32);
//         Serial.print("📡 Response: ");
//         uint8_t buffer[32];
//         int i = 0;
//         Serial.println("REading card....");

//         while (Wire.available() && i < 32)
//         {
//             buffer[i] = Wire.read();
//             Serial.printf("%02X ", buffer[i]);
//             i++;
//         }
//         Serial.println();
//         // Step 3: look for UID
//         for (int j = 0; j < i - 7; j++)
//         {
//             // heuristic: response should contain D5 4B (PN532→host + InListPassiveTarget response)
//             if (buffer[j] == 0xD5 && buffer[j + 1] == 0x4B)
//             {
//                 uint8_t uidLen = buffer[j + 6];
//                 Serial.print("✅ UID: ");
//                 for (uint8_t k = 0; k < uidLen; k++)
//                 {
//                     Serial.printf("%02X ", buffer[j + 7 + k]);
//                 }

//                 Serial.println();
//             }
//         }
//     }
//     Serial.println("END");
// }

void setup() {
  Serial.begin(9600);
      while (!Serial)
        ; // wait for serial
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  Serial.println("🔍 Sending InListPassiveTarget...");

  uint8_t detectCardCmd[] = {
    0x00, 0x00, 0xFF,
    0x04, 0xFC,
    0xD4, 0x4A, 0x01, 0x00, // command
    0x29, 0x00
  };

  sendCommand(detectCardCmd, sizeof(detectCardCmd));

  // Step 1: wait for ACK
  waitReady("ACK");
  Wire.requestFrom(PN532_I2C_ADDRESS, 6);
  Serial.print("ACK: ");
  while (Wire.available()) Serial.printf("%02X ", Wire.read());
  Serial.println();

  // Step 2: wait for actual data
  waitReady("Response");

  Wire.requestFrom(PN532_I2C_ADDRESS, 32);  // read full response
  Serial.print("Response: ");
  uint8_t buffer[32];
  int i = 0;
  while (Wire.available() && i < 32) {
    buffer[i] = Wire.read();
    Serial.printf("%02X ", buffer[i]);
    i++;
  }
  Serial.println();

  // Step 3: look for UID
  for (int j = 0; j < i - 7; j++) {
    // heuristic: response should contain D5 4B (PN532→host + InListPassiveTarget response)
    if (buffer[j] == 0xD5 && buffer[j + 1] == 0x4B) {
      uint8_t uidLen = buffer[j + 6];
      Serial.print("✅ UID: ");
      for (uint8_t k = 0; k < uidLen; k++) {
        Serial.printf("%02X ", buffer[j + 7 + k]);
      }
      Serial.println();
    }
  }
}

void loop()
{
    //   Serial.println("Hello World");
    // scanning
    // scanI2CDevices();
    delay(1000);
}

void setupI2C()
{
}

void taskRFID(void *pvParameters)
{
    while (1)
    {

        Wire.beginTransmission(RFID_DEVICE_ADDR);
        Wire.write(0x00);            // optional: register address (0x00 for many devices)
        Wire.endTransmission(false); // false = keep connection for read

        Wire.requestFrom(RFID_DEVICE_ADDR, 8); // read 8 bytes (change as needed)
        Serial.print("Data: ");
        while (Wire.available())
        {
            byte b = Wire.read();
            Serial.printf("0x%02X ", b);
        }
        Serial.println();

        delay(1000);
    }
}

void scanI2CDevices()
{
    //   Serial.begin(115200);
    Wire.begin(8, 9);
    Serial.println("Scanning I2C...");
    for (byte i = 1; i < 127; i++)
    {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf("Found device at 0x%02X\n", i);
        }
    }
}