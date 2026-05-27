/*
 * ============================================================
 *  ESP-NOW 傳輸速度測試 (Throughput Benchmark)
 * ============================================================
 *  使用說明：
 *  1. 將此程式燒錄至第一塊板子（發送端），並將下方的 IS_SENDER 設為 true
 *  2. 將此程式燒錄至第二塊板子（接收端），並將下方的 IS_SENDER 設為 false
 *  3. 開啟 Serial Monitor (115200 baud) 即可查看每秒傳輸速度
 * ============================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_arduino_version.h>

// 🔴 請在這裡切換發送端與接收端 (true = 發送端, false = 接收端)
#define IS_SENDER false

// 廣播 MAC
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW 每次最大只能傳 250 Bytes
const int PAYLOAD_SIZE = 250;
uint8_t payload[PAYLOAD_SIZE];

// 統計變數
volatile unsigned long totalBytes = 0;
volatile unsigned long successPackets = 0;
volatile unsigned long failPackets = 0;
unsigned long lastTime = 0;

// ============================================================
// 發送端回呼
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataSent(const wifi_tx_info_t *pi, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    successPackets++;
    totalBytes += PAYLOAD_SIZE;
  } else {
    failPackets++;
  }
}

// ============================================================
// 接收端回呼
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  totalBytes += len;
  successPackets++;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW 初始化失敗");
    return;
  }

  if (IS_SENDER) {
    Serial.println("🚀 啟動為【發送端】(Sender)...");
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("❌ 無法新增 Peer");
      return;
    }
    
    // 初始化測試資料
    for (int i = 0; i < PAYLOAD_SIZE; i++) payload[i] = (uint8_t)i;
    
  } else {
    Serial.println("📥 啟動為【接收端】(Receiver)...");
    esp_now_register_recv_cb(OnDataRecv);
  }

  lastTime = millis();
}

// ============================================================
// Loop
// ============================================================
void loop() {
  unsigned long now = millis();

  // 若為發送端，全力發送封包
  if (IS_SENDER) {
    esp_err_t result = esp_now_send(broadcastAddress, payload, PAYLOAD_SIZE);
    if (result != ESP_OK) {
      // 緩衝區滿了或發送太快，稍微讓出 CPU 時間給底層 WiFi 堆疊處理
      delay(1); 
    }
  }

  // 每秒印出一次傳輸速度
  if (now - lastTime >= 1000) {
    // 停用中斷或鎖定變數避免讀取時被更改
    unsigned long bytes = totalBytes;
    unsigned long succ = successPackets;
    unsigned long fail = failPackets;
    
    totalBytes = 0;
    successPackets = 0;
    failPackets = 0;
    lastTime = now;

    // 計算速度
    float kbps = (bytes / 1024.0f);            // KB/s
    float mbps = (bytes * 8.0f) / 1000000.0f;  // Mbps (Megabits per second)

    Serial.println("--------------------------------------------------");
    if (IS_SENDER) {
      Serial.printf("📤 發送速度: %.2f KB/s (%.2f Mbps) | 成功: %lu 封包 | 失敗: %lu 封包\n", kbps, mbps, succ, fail);
    } else {
      Serial.printf("📥 接收速度: %.2f KB/s (%.2f Mbps) | 收到: %lu 封包\n", kbps, mbps, succ);
    }
  }
}
