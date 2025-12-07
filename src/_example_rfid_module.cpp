// piny 8,9

#include <Wire.h>
#include <Adafruit_PN532.h>

// Use this for I2C:
#define PN532_IRQ   (2)   // Can be any digital pin (optional, used for IRQ)
#define PN532_RESET (3)   // Can be any digital pin, or 0 if not used
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port on some boards
  }
  Serial.println("PN532 NFC/RFID Reader - I2C Test");
  // Initialize I2C
  Wire.begin();
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("Didn't find PN532 - check wiring and I2C mode!");
    while (1);
  }
  Serial.print("Found PN532 with firmware version: ");
  Serial.print((versiondata >> 24) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 16) & 0xFF, DEC);
  // Configure board to read RFID tags
  nfc.SAMConfig();
  Serial.println("Waiting for an ISO14443A Card (MIFARE, NFC tag)...");
}
void loop() {
  boolean success;
  uint8_t uid[7];  // Buffer to store the returned UID
  uint8_t uidLength;
  // Check for NFC/RFID card
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  if (success) {
    Serial.print("Found a card! UID Length: ");
    Serial.print(uidLength);
    Serial.println(" bytes");
    Serial.print("UID Value: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      Serial.print(" 0x");
      if (uid[i] < 0x10) Serial.print('0');
      Serial.print(uid[i], HEX);
    }
    Serial.println("");
    // Simple delay so you don't spam the serial monitor
    delay(1000);
  }
  // Small delay between scans
  delay(100);
}