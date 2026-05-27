#include <Arduino.h>

#define BUFFER_SIZE 1024
uint8_t dmaBuffer[BUFFER_SIZE];
const int READY_PIN = 22; 

volatile bool packetReceived = false;
volatile uint32_t burstDurationUs = 0;
uint32_t startTimeUs = 0;

unsigned long totalPackets = 0;
unsigned long corruptPackets = 0;

typedef struct {
  __IO uint32_t PERIPH_RPR;       
  __IO uint32_t PERIPH_RCR;       
  __IO uint32_t PERIPH_TPR;       
  __IO uint32_t PERIPH_TCR;       
  __IO uint32_t PERIPH_RNPR;      
  __IO uint32_t PERIPH_RNCR;      
  __IO uint32_t PERIPH_TNPR;      
  __IO uint32_t PERIPH_TNCR;      
  __O  uint32_t PERIPH_PTCR;      
  __I  uint32_t PERIPH_PTSR;      
} PdcChannel;

volatile PdcChannel *spiDma = (volatile PdcChannel *)((uint32_t)SPI0 + 0x100);

#define Bit_SPI_SR_ENDRX   (1 << 3)
#define Bit_SPI_SR_RDRF    (1 << 0)
#define Bit_PTCR_RXTEN     (1 << 0)
#define Bit_PTCR_RXTDIS    (1 << 1)

void flushSPIHardware() {
  spiDma->PERIPH_PTCR = Bit_PTCR_RXTDIS;
  uint32_t dummyStatus = SPI0->SPI_SR;
  while (SPI0->SPI_SR & Bit_SPI_SR_RDRF) {
    uint32_t dummyRead = SPI0->SPI_RDR;
  }
  NVIC_ClearPendingIRQ(SPI0_IRQn);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(READY_PIN, OUTPUT);
  digitalWrite(READY_PIN, LOW);

  pmc_enable_periph_clk(ID_SPI0);
  PIO_Configure(PIOA, PIO_PERIPH_A,
                PIO_PA25 | PIO_PA26 | PIO_PA27 | PIO_PA28,
                PIO_DEFAULT);

  SPI0->SPI_CR  = SPI_CR_SPIDIS;
  SPI0->SPI_CR  = SPI_CR_SWRST;
  SPI0->SPI_CR  = SPI_CR_SWRST;

  SPI0->SPI_MR  = SPI_MR_MODFDIS;
  SPI0->SPI_CSR[0] = 0; // Mode 1

  // ← 先別急著 enable SPI，讓 ESP32 還沒開始送
  delay(2000); // 等 ESP32 完全開機並停在 while(READY==LOW)
  Serial.println("Due SPI Slave Ready. Waiting for trigger...");

  // 清除任何啟動期間的雜訊
  flushSPIHardware();

  // 設定 DMA
  spiDma->PERIPH_RPR = (uint32_t)dmaBuffer;
  spiDma->PERIPH_RCR = BUFFER_SIZE;

  SPI0->SPI_IER = Bit_SPI_SR_ENDRX;
  NVIC_EnableIRQ(SPI0_IRQn);

  // 最後才 enable SPI
  SPI0->SPI_CR = SPI_CR_SPIEN;

  // 再等一下確保穩定，然後才告訴 ESP32 可以開始
  delay(100);
  spiDma->PERIPH_PTCR = Bit_PTCR_RXTEN;
  startTimeUs = micros();
  digitalWrite(READY_PIN, HIGH);
}

void SPI0_Handler() {
  uint32_t status = SPI0->SPI_SR;
  if (status & Bit_SPI_SR_ENDRX) {
    burstDurationUs = micros() - startTimeUs;
    digitalWrite(READY_PIN, LOW);
    packetReceived = true;
  }
}

void loop() {
  if (packetReceived) {
    totalPackets++;
    bool isCorrupt = false;
    int firstBadIndex = -1;

    for (int i = 0; i < BUFFER_SIZE; i++) {
      uint8_t expected = (uint8_t)((i % 255) + 1);
      if (dmaBuffer[i] != expected) {
        isCorrupt = true;
        firstBadIndex = i;
        break; 
      }
    }

    if (isCorrupt) corruptPackets++;

    float megabitsPerSecond = (float)(BUFFER_SIZE * 8) / burstDurationUs;
    Serial.print("Packet #"); Serial.print(totalPackets);
    Serial.print(" | Wire Speed: "); Serial.print(megabitsPerSecond, 2); Serial.print(" Mbps");
    
    if (isCorrupt) {
      Serial.println(" | Status: CORRUPTED");
      Serial.println("\n--- DEBUG: DATA MISMATCH DETECTED ---");
      Serial.print("First error at index: "); Serial.println(firstBadIndex);
      Serial.println("INDEX\t| EXPECTED\t| ACTUAL\t| BINARY");
      Serial.println("-----------------------------------------------------------------");
      int startPrint = max(0, firstBadIndex - 4);
      for (int g = startPrint; g < startPrint + 16 && g < BUFFER_SIZE; g++) {
        uint8_t exp = (uint8_t)((g % 255) + 1);
        uint8_t act = dmaBuffer[g];
        Serial.print(g); Serial.print("\t| 0x");
        if (exp < 0x10) Serial.print("0"); Serial.print(exp, HEX);
        Serial.print("\t\t| 0x");
        if (act < 0x10) Serial.print("0"); Serial.print(act, HEX);
        Serial.print("\t\t| B");
        for (int b = 7; b >= 0; b--) Serial.print((act >> b) & 1);
        Serial.println();
      }
      Serial.println("-----------------------------------------------------------------");
      Serial.println("Benchmark halted. Check logs.");
      while(1);
    } else {
      Serial.println(" | Status: PERFECT");
    }

    delay(500); 
    packetReceived = false;
    memset(dmaBuffer, 0, BUFFER_SIZE);
    flushSPIHardware(); 
    spiDma->PERIPH_RPR = (uint32_t)dmaBuffer;
    spiDma->PERIPH_RCR = BUFFER_SIZE;
    spiDma->PERIPH_PTCR = Bit_PTCR_RXTEN; 
    startTimeUs = micros();         
    digitalWrite(READY_PIN, HIGH); 
  }
}