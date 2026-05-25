#include <SPI.h>

const int SS_PIN = 5;
const int READY_PIN = 4;

// Test 20MHz or push it to 40MHz!
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

#define BUFFER_SIZE 1024
// Force 32-bit hardware alignment for the ESP32 DMA engine
uint32_t dataBuffer32[BUFFER_SIZE / 4];
uint8_t *dataBuffer = (uint8_t *)dataBuffer32;

void setup() {
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  pinMode(READY_PIN, INPUT);

  SPI.begin();

  // Create the predetermined verification content: 0, 1, 2, ... 255, 0, 1...
  for (int i = 0; i < BUFFER_SIZE; i++) {
    dataBuffer[i] = (uint8_t)((i % 255) + 1); // Never sends a 0x00!
  }
}

void loop() {
  // 1. Wait until Due says "Go!"
  while (digitalRead(READY_PIN) == LOW) {
    // Rest
  }

  // 2. Fire the packet immediately
  digitalWrite(SS_PIN, LOW);
  delayMicroseconds(5); // Brief setup time
  SPI.beginTransaction(spiSettings);

  SPI.transferBytes(dataBuffer, NULL, BUFFER_SIZE);

  SPI.endTransaction();
  digitalWrite(SS_PIN, HIGH);

  // 3. Wait for the Due to drop the Ready pin back to LOW
  // This ensures the ESP32 doesn't accidentally double-trigger a loop
  while (digitalRead(READY_PIN) == HIGH) {
    // Wait for Ack acknowledgement
  }
}