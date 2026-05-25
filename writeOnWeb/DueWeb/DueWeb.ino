// --- Arduino Due 接收端程式碼 ---

const int GRID_SIZE = 28;
const int TOTAL_PIXELS = GRID_SIZE * GRID_SIZE;
uint8_t imageBuffer[TOTAL_PIXELS];

void setup() {
  // Serial 是 Due 的 USB 接頭，用來連接電腦的 Serial Monitor (鮑率 115200)
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Serial1 是 Due 的實體腳位 Pin 19(RX) 和 Pin 18(TX)
  // 必須和 ESP32-C3 設定成一模一樣的 500000 狂暴速度！
  Serial1.begin(500000); 

  Serial.println("🐾 Due 已經張開嘴巴，等待 ESP32-C3 的影像餵食...");
}

void loop() {
  // 尋找封包頭 0xAA 0xBB
  if (Serial1.available() >= 2) {
    uint8_t header1 = Serial1.read();
    if (header1 == 0xAA) {
      // 稍微等一下下，確保下一個 byte 進來
      unsigned long waitStart = millis();
      while (Serial1.available() < 1) {
        if (millis() - waitStart > 10) return; // 超時放棄
      }
      
      uint8_t header2 = Serial1.read();
      if (header2 == 0xBB) {
        // 密碼正確！開始接收 784 個 Bytes
        int bytesRead = 0;
        unsigned long timeout = millis();
        
        while (bytesRead < TOTAL_PIXELS) {
          if (Serial1.available() > 0) {
            imageBuffer[bytesRead] = Serial1.read();
            bytesRead++;
          }
          // 安全機制：如果傳到一半斷線，超過 500 毫秒就放棄，防止 Due 死機
          if (millis() - timeout > 500) {
            Serial.println("⚠️ 喵唔！接收超時，封包破碎了！");
            return; 
          }
        }
        
        // 接收完成！把這張圖畫在 Due 的監控視窗上
        printImageToSerial();
        
        // 💡 主人可以在這裡把 imageBuffer 餵給你的 TinyML 辨識模型囉！
        // runInference(imageBuffer);
      }
    }
  }
}

// 在 Due 的監控視窗畫出 28x28 圖像
void printImageToSerial() {
  Serial.println("\n--- 🎯 Arduino Due 成功接收！影像重現 ---");
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      int index = y * GRID_SIZE + x;
      // 0 是黑色的字跡，255 是白色的背景
      if (imageBuffer[index] == 0) {
        Serial.print("██");
      } else {
        Serial.print("  ");
      }
    }
    Serial.println(); // 換行
  }
  Serial.println("----------------------------------------\n");
}