// piny 8,9

#include <Wire.h>
#include <Adafruit_PN532.h>
#include <Adafruit_NeoPixel.h>
// Addressable RGB LED, driven by GPIO48.
#define LED_PIN 48
// Use this for I2C:
#define PN532_IRQ   (2)   // Can be any digital pin (optional, used for IRQ)
#define PN532_RESET (3)   // Can be any digital pin, or 0 if not used
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

Adafruit_NeoPixel strip(1, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    Serial.begin(9600);
    
    
    strip.begin();
    strip.setBrightness(50); // 0–255
    strip.show();
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    
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
        String uidString = "";
        
        Serial.print("Found a card! UID Length: ");
        Serial.print(uidLength);
        Serial.println(" bytes");
        Serial.print("UID Value: ");
        for (uint8_t i = 0; i < uidLength; i++) {
            //   Serial.print(" 0x");
            //   if (uid[i] < 0x10) Serial.print('0');
            //   Serial.print(uid[i], HEX);
            uidString += String(uid[i], HEX);
            
        }
        Serial.println(uidString);
        
        if(uidString == "6deb45"){
            Serial.println("Access Granted");
            strip.setPixelColor(0, strip.Color(255, 0, 0));
            strip.show();
        }else if(uidString == "544b21"){
            Serial.println("Access Denied");
            strip.setPixelColor(0, strip.Color(0, 255, 0));
            strip.show();
        }else if(uidString == "252b9e7800"){
            Serial.println("Access Denied");
            strip.setPixelColor(0, strip.Color(0, 0, 255));
            strip.show();
        }else if(uidString == "198e93"){
            Serial.println("Access Denied");
            strip.setPixelColor(0, strip.Color(255, 255, 0));
            strip.show();
        }else if(uidString == "146c9f"){
            Serial.println("Access Denied");
            strip.setPixelColor(0, strip.Color(0, 255, 255));
            strip.show();
        }else if(uidString == "42f8d12e95b80"){
            Serial.println("Access Denied");
            strip.setPixelColor(0, strip.Color(255, 255, 255));
            strip.show();
        }else{
            Serial.println("Unknown Card");
            strip.setPixelColor(0, strip.Color(0, 0, 0));
            strip.show();
        }
        
        Serial.println("");
        // Simple delay so you don't spam the serial monitor
        delay(1000);
    }
    // Small delay between scans
    delay(100);
}