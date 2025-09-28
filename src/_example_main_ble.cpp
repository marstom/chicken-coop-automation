#include <Arduino.h>
#include <ArduinoBLE.h>

// Define UUIDs for your service and characteristics
// Replace these with your actual UUIDs
const char* serviceUUID = "12345678-1234-1234-1234-123456789ABC";
const char* dataCharacteristicUUID = "12345678-1234-1234-1234-123456789ABD";

BLEService dataService(serviceUUID);
BLEStringCharacteristic dataCharacteristic(dataCharacteristicUUID, BLERead | BLEWrite, 20); // 20 bytes max

void setup() {
  Serial.begin(9600);
  while (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    delay(1000);
  }

  // Set local name and advertise
  BLE.setLocalName("My ESP32-S3");
  BLE.setAdvertisedService(dataService);

  // Add the service and characteristic
  dataService.addCharacteristic(dataCharacteristic);
  BLE.addService(dataService);

  // Start advertising
  BLE.advertise();
  Serial.println("ESP32-S3 advertising as a BLE server!");
}

void loop() {
  // Your main loop logic
  BLEDevice central = BLE.central(); // Check for central connections

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      // You can read/write characteristics here
      if (dataCharacteristic.written()) {
        String receivedValue = dataCharacteristic.value();
        Serial.print("Received: ");
        Serial.println(receivedValue);
        dataCharacteristic.writeValue("Received your data"); // Acknowledge
      }
      delay(100);
    }
    Serial.println("Disconnected from central");
  }
}
