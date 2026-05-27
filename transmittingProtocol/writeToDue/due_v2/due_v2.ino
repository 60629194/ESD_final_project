#include "RS_FEC.h"

const int CLOCK_PIN = 2;
volatile uint8_t receivedData = 0;
volatile bool newData = false;

// 調整大小：98 Bytes (圖片) + 4 Bytes (C3 Latency) + 4 Bytes (Node Latency)
const int msglen = 106;
const int ECC_LEN = 10;
uint8_t rx_buffer[msglen + ECC_LEN];
uint8_t repaired_msg[msglen];

RS::ReedSolomon<msglen, ECC_LEN> rs;

uint32_t packetIndex = 0;
uint32_t startTime = 0;
bool isTimingBurst = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  REG_PIOC_ODR = 0xFFFFFFFF; // Set Port C as inputs
  pinMode(CLOCK_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), onClockPulse, RISING);
  
  Serial.println("✅ Arduino Due 接收端已啟動");
  Serial.println("等待來自 NodeMCU 的 Parallel 傳輸資料...");
}

void onClockPulse() {
  uint32_t active_pins = REG_PIOC_PDSR & 0x1FE;
  uint8_t corrected = 0;
  if (active_pins & (1 << 8)) corrected |= (1 << 0);
  if (active_pins & (1 << 7)) corrected |= (1 << 1);
  if (active_pins & (1 << 6)) corrected |= (1 << 2);
  if (active_pins & (1 << 5)) corrected |= (1 << 3);
  if (active_pins & (1 << 4)) corrected |= (1 << 4);
  if (active_pins & (1 << 3)) corrected |= (1 << 5);
  if (active_pins & (1 << 2)) corrected |= (1 << 6);
  if (active_pins & (1 << 1)) corrected |= (1 << 7);

  receivedData = corrected;
  newData = true;
}

void printGrid(uint8_t* data) {
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  收到手寫辨識圖形 (28x28)                ║");
  Serial.println("╠══════════════════════════════════════════╣");
  
  for (int y = 0; y < 28; y++) {
    Serial.print("║");
    for (int x = 0; x < 28; x++) {
      int bitIndex = y * 28 + x;
      // 檢查對應的 bit 是否為 1 (ink)
      bool isInk = (data[bitIndex / 8] & (1 << (7 - (bitIndex % 8)))) != 0;
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
      uint32_t lastByteTime = micros();

      while (packetIndex < (msglen + ECC_LEN)) {
        if (newData) {
          newData = false;
          rx_buffer[packetIndex] = receivedData;
          packetIndex++;
          lastByteTime = micros(); 
        }
        if (micros() - lastByteTime > 5000) {
          break;
        }
      }

      uint32_t endTime = micros();
      isTimingBurst = false;
      uint32_t durationUs = endTime - startTime;

      int repairStatus = rs.Decode(rx_buffer, repaired_msg);

      Serial.println("\n--- 傳輸完成 ---");
      Serial.print("[Latency] Due 接收與解碼耗時: "); Serial.print(durationUs); Serial.println(" us");
      
      if (repairStatus == 0 && packetIndex == (msglen + ECC_LEN)) {
        Serial.println(" | 狀態: 完美 (0 Errors)");
        printGrid(repaired_msg);
      } else if (repairStatus > 0) {
        Serial.print(" | 狀態: 修復成功! (修正了 ");
        Serial.print(repairStatus);
        Serial.println(" 個錯誤 bytes)");
        printGrid(repaired_msg);
      } else {
        Serial.println(" | 狀態: ❌ 無法修復的嚴重資料損毀");
      }

      if (repairStatus >= 0) {
        uint32_t t_c3 = 0, t_node = 0;
        memcpy(&t_c3, &repaired_msg[98], 4);
        memcpy(&t_node, &repaired_msg[102], 4);
        uint32_t total_latency = t_c3 + 2000 + t_node + durationUs; // 假設 ESP-NOW 空中傳輸耗時約 2ms (2000 us)
        Serial.println("\n📊 === 總延遲分析 (Latency Breakdown) ===");
        Serial.print(" 1. ESP32-C3 構圖處理 : "); Serial.print(t_c3); Serial.println(" us");
        Serial.println(" 2. ESP-NOW 無線傳輸  : ~2000 us (預估)");
        Serial.print(" 3. NodeMCU 中繼處理  : "); Serial.print(t_node); Serial.println(" us");
        Serial.print(" 4. Due 接收與解碼    : "); Serial.print(durationUs); Serial.println(" us");
        Serial.println(" --------------------------------------");
        Serial.print(" 🌟 總耗時 (從按下送出到完成): ~"); Serial.print(total_latency); Serial.print(" us (");
        Serial.print((float)total_latency / 1000.0, 2); Serial.println(" ms)");
        Serial.println("=========================================");
      }
    }
  }
}
