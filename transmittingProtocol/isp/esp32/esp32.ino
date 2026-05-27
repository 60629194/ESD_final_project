#include <SPI.h>

const int SS_PIN = 5;
const int READY_PIN = 4;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);

#define BUFFER_SIZE 1024
uint32_t dataBuffer32[BUFFER_SIZE / 4];
uint8_t *dataBuffer = (uint8_t *)dataBuffer32;

void setup() {
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  pinMode(READY_PIN, INPUT);

  SPI.begin(18, 19, 23, -1); 

  for (int i = 0; i < BUFFER_SIZE; i++) {
    dataBuffer[i] = (uint8_t)((i % 255) + 1); 
  }
}

void loop() {
  // 1. LOCK THE HARDWARE STATE FIRST
  // This prevents the setup glitch from hitting an armed Due
  SPI.beginTransaction(spiSettings);
  
  // 2. NOW wait for the Due to arm its DMA and signal it is ready
  while (digitalRead(READY_PIN) == LOW) {
    // Wait
  }

  // 3. Fire the payload on a perfectly quiet bus
  digitalWrite(SS_PIN, LOW);
  delayMicroseconds(5); 

  SPI.transferBytes(dataBuffer, NULL, BUFFER_SIZE);

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();

  // 4. Wait for Due to process and drop the line
  while (digitalRead(READY_PIN) == HIGH) {
    // Wait
  }
}