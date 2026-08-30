#include <Arduino.h>
#include <hello_world.h>

void setup() {
  Serial.begin(115200);
  Serial.println(hello_world::message());
}

void loop() {}
