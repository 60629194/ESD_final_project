#include "RS_FEC.h"

const int CLOCK_PIN = 2;
volatile uint8_t receivedData = 0;
volatile bool newData = false;

// Reed-Solomon Configuration matching the ESP32
const int msglen = 245;
const int ECC_LEN = 10;
uint8_t rx_buffer[msglen + ECC_LEN];
uint8_t repaired_msg[msglen];

RS::ReedSolomon<msglen, ECC_LEN> rs;

uint32_t packetIndex = 0;
uint32_t startTime = 0;
bool isTimingBurst = false;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  REG_PIOC_ODR = 0xFFFFFFFF; // Set Port C as inputs
  pinMode(CLOCK_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), onClockPulse, RISING);
  Serial.println("Due Reed-Solomon Receiver Ready. Measuring Restored Data...");
}

void onClockPulse() {
  uint32_t active_pins = REG_PIOC_PDSR & 0x1FE;
  uint8_t corrected = 0;
  if (active_pins & (1 << 8))
    corrected |= (1 << 0);
  if (active_pins & (1 << 7))
    corrected |= (1 << 1);
  if (active_pins & (1 << 6))
    corrected |= (1 << 2);
  if (active_pins & (1 << 5))
    corrected |= (1 << 3);
  if (active_pins & (1 << 4))
    corrected |= (1 << 4);
  if (active_pins & (1 << 3))
    corrected |= (1 << 5);
  if (active_pins & (1 << 2))
    corrected |= (1 << 6);
  if (active_pins & (1 << 1))
    corrected |= (1 << 7);

  receivedData = corrected;
  newData = true;
}

void loop() {
  if (newData) {
    newData = false;
    if (!isTimingBurst) {
      startTime = micros();
      isTimingBurst = true;
      packetIndex = 0;
    }

    if (isTimingBurst) {
      rx_buffer[packetIndex] = receivedData;
      packetIndex++;
      // Update the timestamp of the last successfully received byte
      uint32_t lastByteTime = micros();

      // IF WE HIT THE PACKET TARGET OR A TIMEOUT HAPPENS
      // (Bails out if wires go silent for more than 5000 microseconds)
      while (packetIndex < (msglen + ECC_LEN)) {
        if (newData) {
          newData = false;
          rx_buffer[packetIndex] = receivedData;
          packetIndex++;
          lastByteTime = micros(); // Reset timeout clock
        }
        // TIMEOUT CHECK: Did the transmitter stop sending?
        if (micros() - lastByteTime > 5000) {
          break; // Break out of the collection loop early!
        }
      }

      // Process the burst (whether it finished naturally or timed out)
      uint32_t endTime = micros();
      isTimingBurst = false;
      uint32_t durationUs = endTime - startTime;

      int repairStatus = rs.Decode(rx_buffer, repaired_msg);

      float totalBits = msglen * 8.0;
      float seconds = durationUs / 1000000.0;
      float mbps = (totalBits / 1000000.0) / seconds;

      Serial.print("--- BURST COMPLETE ---");
      Serial.print(" | Time: ");
      Serial.print(durationUs);
      Serial.print(" us");
      Serial.print(" | Speed: ");
      Serial.print(mbps, 4);
      Serial.print(" Mbps");
      if (repairStatus == 0 && packetIndex == (msglen + ECC_LEN)) {
        Serial.println(" | Status: PERFECT (0 Errors)");
      } else if (repairStatus >= 0) {
        Serial.print(" | Status: REPAIRED! Fixed ");
        Serial.print(repairStatus);
        Serial.println(" missing/broken bytes.");
      } else {
        Serial.println(" | Status: UNRECOVERABLE CORRUPTION");
      }
    }
  }
}