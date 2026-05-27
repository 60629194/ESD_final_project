/*
 * ============================================================
 *  NodeMCU-32S ESP-NOW 接收端
 * ============================================================
 */
#include <WiFi.h>
#include <esp_now.h>

// 定義接收回呼函式
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len != 98) {
    Serial.print("收到未知的封包，長度: ");
    Serial.println(len);
    return;
  }
  
  Serial.println("\n\n╔══════════════════════════════════════════╗");
  Serial.println("║  收到無線傳輸圖形 (28x28)                ║");
  Serial.println("╠══════════════════════════════════════════╣");
  
  for (int y = 0; y < 28; y++) {
    Serial.print("║");
    for (int x = 0; x < 28; x++) {
      int bitIndex = y * 28 + x;
      // 檢查對應的 bit 是否為 1 (ink)
      bool isInk = (incomingData[bitIndex / 8] & (1 << (7 - (bitIndex % 8)))) != 0;
      if (isInk) {
        Serial.print("██");
      } else {
        Serial.print("  ");
      }
    }
    Serial.println("║");
  }
  Serial.println("╚══════════════════════════════════════════╝");
}

void setup() {
  Serial.begin(115200);
  
  // 必須先設定 WiFi 為 Station 模式
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW 初始化失敗");
    return;
  }
  
  // 註冊接收回呼函數
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("✅ ESP-NOW 接收端已啟動，等待接收手寫資料...");
}

void loop() {
  // ESP-NOW 是事件驅動 (中斷回呼)，迴圈不需做特別處理
  delay(1000);
}
