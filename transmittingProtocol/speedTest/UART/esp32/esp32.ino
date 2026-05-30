#include <Arduino.h>

void setup() {
  // Hardware UART on ESP32, high baud rate for stress testing
  Serial2.begin(500000, SERIAL_8N1, 3, 1);
}

void loop() {
  uint8_t buffer[64];
  for(int i = 0; i < 64; i++) {
    buffer[i] = i;
  }
  
  // Continuously blast data
  Serial2.write(buffer, 64);
}
