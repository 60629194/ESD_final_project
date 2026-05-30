#include <Wire.h>

void setup() {
  // Setup Hardware I2C (Wire)
  Wire.begin(21, 22);
  Wire.setClock(400000); // Fast Mode (400 kHz)
}

void loop() {
  // Continuously blast I2C packets to Due (Address 8)
  Wire.beginTransmission(8);
  for(int i = 0; i < 32; i++) {
    // 32 bytes is the standard Arduino Wire buffer limit
    Wire.write(i); 
  }
  Wire.endTransmission();
}
