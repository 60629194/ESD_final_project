// ESP32-S Transmitter
const int pins[8] = {16, 17, 18, 19, 21, 22, 23, 25};
const int CLOCK_PIN = 4;

void setup() {
  for(int i=0; i<8; i++) {
    pinMode(pins[i], OUTPUT);
  }
  pinMode(CLOCK_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, LOW);
}

void sendByte(uint8_t data) {
  // 1. First, place ALL data onto the wires
  for (int i = 0; i < 8; i++) {
    digitalWrite(pins[i], (data >> i) & 0x01);
  }
  
  // 2. WAIT HERE: Let all 8 data lines completely settle and stop moving
  delayMicroseconds(20); 
  
  // 3. Now that data is frozen on the lines, pulse the clock cleanly
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(10); // Hold it high so Due catches it reliably
  digitalWrite(CLOCK_PIN, LOW);
  
  // 4. Wait for the clock line trail noise to clear before exiting
  delayMicroseconds(20); 
}

int count = 0;

void loop() {
  if(count == 0){
    for(uint8_t i = 0; i < 255; i++) {
      sendByte(i);
      delay(10); // Send a new number every 100ms for testing
    }
  }
  count = count+1;
}