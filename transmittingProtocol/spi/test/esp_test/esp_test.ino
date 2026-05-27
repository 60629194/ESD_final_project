#include <SPI.h>

const int SS_PIN  = 5;
const int LED_PIN = 2; // 板上 LED，方便觀察

SPISettings spiSettings(500000, MSBFIRST, SPI_MODE0);

void setup() {
  Serial.begin(115200);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  SPI.begin();
  Serial.println("ESP32 Test Master Ready.");
  Serial.println("Sending 0xAB every 2 seconds...");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);

  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);
  delayMicroseconds(10);          // 給 Due 一點準備時間

  SPI.transfer(0xAB);             // 送一個固定 byte，容易辨認

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();

  digitalWrite(LED_PIN, LOW);

  Serial.println("Sent: 0xAB");
  delay(2000);
}