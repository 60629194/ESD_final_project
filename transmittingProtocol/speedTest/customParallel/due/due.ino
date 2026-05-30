#include <Arduino.h>

const int CLOCK_PIN = 2;
volatile uint32_t bytesReceived = 0;
uint32_t lastTime = 0;

void onClockPulse() {
  // We simply count the bytes received per second
  bytesReceived++;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  REG_PIOC_ODR = 0xFFFFFFFF; // Set Port C as inputs
  pinMode(CLOCK_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), onClockPulse, RISING);
  
  Serial.println("Due customParallel Receiver Ready.");
  Serial.println("Waiting for continuous stream...");
  lastTime = millis();
}

void loop() {
  uint32_t currentTime = millis();
  
  // Calculate throughput every 1 second
  if (currentTime - lastTime >= 1000) {
    noInterrupts();
    uint32_t count = bytesReceived;
    bytesReceived = 0;
    interrupts();
    
    float mbps = (count * 8.0) / 1000000.0;
    
    Serial.print("--- Continuous customParallel Throughput --- | Speed: "); 
    Serial.print(mbps, 4); 
    Serial.println(" Mbps");
    
    lastTime = currentTime;
  }
}
