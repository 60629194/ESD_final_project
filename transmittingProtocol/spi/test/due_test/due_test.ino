// 完全不用 DMA、不用中斷、不用 READY 握手
// 只做一件事：等 CS 拉低，然後讀一個 byte，印出來

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pmc_enable_periph_clk(ID_SPI0);

  // PA25=MISO, PA26=MOSI, PA27=SCK, PA28=CS0
  PIO_Configure(PIOA, PIO_PERIPH_A,
                PIO_PA25 | PIO_PA26 | PIO_PA27 | PIO_PA28,
                PIO_DEFAULT);

  SPI0->SPI_CR  = SPI_CR_SPIDIS;
  SPI0->SPI_CR  = SPI_CR_SWRST;
  SPI0->SPI_CR  = SPI_CR_SWRST;

  // Slave 模式，關閉 mode fault 偵測
  SPI0->SPI_MR  = SPI_MR_MODFDIS;

  // Mode 1：NCPHA=0, CPOL=0，8-bit
  SPI0->SPI_CSR[0] = SPI_CSR_NCPHA; 

  SPI0->SPI_CR  = SPI_CR_SPIEN;

  Serial.println("Due Test Slave Ready.");
  Serial.println("Waiting for SPI bytes (polling)...");
}

void loop() {
  // 等到有資料才處理，不會漏掉
  while (!(SPI0->SPI_SR & SPI_SR_RDRF)) { }
  
  uint8_t received = (uint8_t)(SPI0->SPI_RDR & 0xFF);
  Serial.print("Received: 0x");
  if (received < 0x10) Serial.print("0");
  Serial.println(received, HEX);

  if (received == 0xAB) {
    Serial.println(">>> MATCH!");
  } else {
    Serial.print(">>> MISMATCH! got 0x");
    if (received < 0x10) Serial.print("0");
    Serial.println(received, HEX);
  }
}