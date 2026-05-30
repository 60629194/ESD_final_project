#include <Wire.h>

volatile uint32_t bytesReceived = 0;
uint32_t lastTime = 0;

void receiveEvent(int howMany) {
  while (Wire.available()) {
    Wire.read();
    bytesReceived++;
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  // Initialize Hardware I2C as Slave at address 8
  // Make sure ESP32 SDA (Pin 21) -> Due SDA (Pin 20)
  // Make sure ESP32 SCL (Pin 22) -> Due SCL (Pin 21)
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  
  Serial.println("Due Hardware I2C Slave Ready.");
  Serial.println("Waiting for continuous stream...");
  lastTime = millis();
}

void loop() {
  uint32_t currentTime = millis();
  
  // Calculate throughput every 1 second
  if (currentTime - lastTime >= 1000) {
    // Safely copy volatile variable
    noInterrupts();
    uint32_t count = bytesReceived;
    bytesReceived = 0;
    interrupts();
    
    float kbps = (count * 8.0) / 1000.0;
    
    Serial.print("--- Continuous I2C Throughput --- | Speed: "); 
    Serial.print(kbps, 2); 
    Serial.println(" Kbps");
    
    lastTime = currentTime;
  }
}
