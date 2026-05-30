#include <Arduino.h>

uint32_t bytesReceived = 0;
uint32_t lastTime = 0;

void setup() {
  Serial.begin(115200); // Monitor port
  
  // Use Due's Hardware Serial1 for receiving
  // Make sure ESP32 TX (Pin 17) is connected to Due RX1 (Pin 19)
  Serial1.begin(500000); 
  
  while(!Serial);
  Serial.println("Due Hardware UART Receiver Ready.");
  Serial.println("Waiting for continuous stream...");
  lastTime = millis();
}

void loop() {
  // Read incoming bytes from Hardware Buffer
  while (Serial1.available()) {
    Serial1.read();
    bytesReceived++;
  }
  
  // Calculate throughput every 1 second
  uint32_t currentTime = millis();
  if (currentTime - lastTime >= 1000) {
    float kbps = (bytesReceived * 8.0) / 1000.0;
    
    Serial.print("--- Continuous UART Throughput --- | Speed: "); 
    Serial.print(kbps, 2); 
    Serial.println(" Kbps");
    
    bytesReceived = 0;
    lastTime = currentTime;
  }
}
