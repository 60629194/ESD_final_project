// Arduino Due Receiver (Final 8-Bit Calibration)
const int CLOCK_PIN = 2; 
volatile uint8_t receivedData = 0;
volatile bool newData = false;

// Track the last raw value to prevent double-printing noise
volatile uint32_t lastRawPort = 0xFFFFFFFF; 

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  // Set Port C pins as input (Disables outputs on the whole register)
  REG_PIOC_ODR = 0xFFFFFFFF; 
  
  pinMode(CLOCK_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), onClockPulse, RISING); 
  
  Serial.println("Parallel Receiver Ready. Full 8-Bit Capture Active...");
}

void onClockPulse() {
  // Grab the register data immediately 
  uint32_t active_pins = REG_PIOC_PDSR & 0x1FE; 
  
  uint8_t corrected = 0;
  
  // The PERFECT 1-to-1 Physical Mapping
  if (active_pins & (1 << 8)) corrected |= (1 << 0); // Pin 40 (PC8) -> Bit 0
  if (active_pins & (1 << 7)) corrected |= (1 << 1); // Pin 39 (PC7) -> Bit 1
  if (active_pins & (1 << 6)) corrected |= (1 << 2); // Pin 38 (PC6) -> Bit 2
  if (active_pins & (1 << 5)) corrected |= (1 << 3); // Pin 37 (PC5) -> Bit 3
  if (active_pins & (1 << 4)) corrected |= (1 << 4); // Pin 36 (PC4) -> Bit 4
  if (active_pins & (1 << 3)) corrected |= (1 << 5); // Pin 35 (PC3) -> Bit 5
  if (active_pins & (1 << 2)) corrected |= (1 << 6); // Pin 34 (PC2) -> Bit 6
  if (active_pins & (1 << 1)) corrected |= (1 << 7); // Pin 33 (PC1) -> Bit 7

  receivedData = corrected;
  newData = true;
}

void loop() {
  if (newData) {
    Serial.print("Received: ");
    Serial.println(receivedData);
    newData = false;
  }
}