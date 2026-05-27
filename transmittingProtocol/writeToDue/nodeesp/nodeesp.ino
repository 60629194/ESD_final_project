#include <WiFi.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include "RS_FEC.h"
#include "soc/gpio_reg.h"

// ============================================================
// 並列傳輸 (Parallel) 腳位與變數設定
// ============================================================
const int pins[8] = {16, 17, 18, 19, 21, 22, 23, 25};
const int CLOCK_PIN = 4;

uint32_t gpio_lookup[256];
const uint32_t DATA_MASK = (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19) |
                           (1 << 21) | (1 << 22) | (1 << 23) | (1 << 25);
const uint32_t CLOCK_MASK = (1 << 4);

// 調整大小：98 Bytes (圖片) + 4 Bytes (C3 Latency) + 4 Bytes (Node Latency)
const int msglen = 106;
const int ECC_LEN = 10;
uint8_t raw_msg[msglen];
uint8_t encoded_msg[msglen + ECC_LEN];

RS::ReedSolomon<msglen, ECC_LEN> rs;

volatile bool hasNewData = false;
volatile uint32_t t_node_start = 0;

// ============================================================
// ESP-NOW 接收回呼函式
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len == 102) {
    t_node_start = micros();
    // 複製資料並立起旗標，交由 loop() 處理以避免在 ISR 中卡住
    memcpy((void*)raw_msg, incomingData, 102);
    hasNewData = true;
  }
}

// ============================================================
// 並列傳輸底層函式
// ============================================================
#define SETTLE_DELAY()                                                         \
  {                                                                            \
    for (volatile int x = 0; x < 20; x++)                                      \
      __asm__ __volatile__("nop");                                             \
  }

void sendByteFast(uint8_t data) {
  REG_WRITE(GPIO_OUT_W1TC_REG, DATA_MASK);
  REG_WRITE(GPIO_OUT_W1TS_REG, gpio_lookup[data]);
  SETTLE_DELAY();
  REG_WRITE(GPIO_OUT_W1TS_REG, CLOCK_MASK);
  SETTLE_DELAY();
  REG_WRITE(GPIO_OUT_W1TC_REG, CLOCK_MASK);
  SETTLE_DELAY();
}

void setup() {
  Serial.begin(115200);

  // 1. 初始化 Parallel 腳位
  for (int i = 0; i < 8; i++) pinMode(pins[i], OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, LOW);

  // 建立 GPIO 快速查表
  for (int i = 0; i < 256; i++) {
    uint32_t val = 0;
    if (i & (1 << 0)) val |= (1 << 16);
    if (i & (1 << 1)) val |= (1 << 17);
    if (i & (1 << 2)) val |= (1 << 18);
    if (i & (1 << 3)) val |= (1 << 19);
    if (i & (1 << 4)) val |= (1 << 21);
    if (i & (1 << 5)) val |= (1 << 22);
    if (i & (1 << 6)) val |= (1 << 23);
    if (i & (1 << 7)) val |= (1 << 25);
    gpio_lookup[i] = val;
  }

  // 2. 初始化 ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW 初始化失敗");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("✅ NodeMCU 中繼端已啟動");
  Serial.println("等待 ESP32-C3 透過 ESP-NOW 傳輸資料...");
}

void loop() {
  if (hasNewData) {
    hasNewData = false;
    Serial.println("\n[NodeMCU] 收到 ESP-NOW 資料，準備轉發至 Due...");

    uint32_t t_node = micros() - t_node_start;
    memcpy(&raw_msg[102], &t_node, 4); // 將 NodeMCU 中繼等待時間寫入

    // 進行 Reed-Solomon 編碼 (106 bytes -> 116 bytes)
    rs.Encode(raw_msg, encoded_msg);

    // 透過 Parallel Bus 傳送給 Due
    uint32_t t1 = micros();
    for (int i = 0; i < (msglen + ECC_LEN); i++) {
      sendByteFast(encoded_msg[i]);
    }
    uint32_t t2 = micros();

    Serial.printf("[Latency] NodeMCU 處理與轉發耗時: %u us\n", (uint32_t)(t2 - t1) + t_node);
  }
}
