#include "RS_FEC.h"
#include "weights.h"

// ---------------------------------------------------------
// CNN Inference definitions
// ---------------------------------------------------------
// Slice Configuration structure
struct SliceConfig {
  int I;
  int D;
  int J;
};

// Slices configurations matching run_model.py
const SliceConfig SLICES_CONFIG[9] = {
  {1, 576, 24},
  {2, 288, 12},
  {4, 144, 6},
  {8, 72, 3},
  {24, 24, 1},
  {48, 12, 1},
  {96, 6, 1},
  {192, 3, 1},
  {576, 1, 1}
};

// Fixed 3x3 kernels for fixed edge-detectors
const int8_t SOBEL_X[3][3] = {
  {-1,  0,  1},
  {-2,  0,  2},
  {-1,  0,  1}
};

const int8_t SOBEL_Y[3][3] = {
  {-1, -2, -1},
  { 0,  0,  0},
  { 1,  2,  1}
};

const int8_t SOBEL_DIAG1[3][3] = {
  {-2, -1,  0},
  {-1,  0,  1},
  { 0,  1,  2}
};

const int8_t SOBEL_DIAG2[3][3] = {
  { 0, -1, -2},
  { 1,  0, -1},
  { 2,  1,  0}
};

// Globally allocated buffers to avoid stack overflow (Due SRAM is 96 KB)
static float input_image[28][28];
static float c1[4][26][26];
static float c2[4][24][24];
static float fixed_conv_out[4][26][26];
static float o_trainable[36][12];
static float o_fixed[4][12];
static float o_flat[480];
static float h2[128];
static float final_out[47];

struct InfResult {
  int digit;
  float score;
};

extern volatile uint32_t packetIndex;

bool isValidClass(int classIdx, uint8_t strokes) {
  char c = EMNIST_CLASSES[classIdx];
  if (strokes == 1) {
    const char* s1 = "CLOSUVWZ1236890abcdeghlmnopqrsuvwz";
    for (int i = 0; s1[i] != '\0'; i++) if (s1[i] == c) return true;
  } else if (strokes == 2) {
    const char* s2 = "ABDGJKMNPQRTXY457fijktxy";
    for (int i = 0; s2[i] != '\0'; i++) if (s2[i] == c) return true;
  } else if (strokes == 3) {
    const char* s3 = "EFHI";
    for (int i = 0; s3[i] != '\0'; i++) if (s3[i] == c) return true;
  }
  return false;
}

InfResult run_inference(bool printDetails = true, uint8_t expectedStrokes = 0) {
  uint32_t t_start = millis();

  // ==========================================
  // STREAM 1: Trainable Conv Stream
  // ==========================================
  
  // 1. conv1: Standard 2D convolution (input 1x28x28 -> output 4x26x26)
  for (int c = 0; c < 4; c++) {
    if (packetIndex > 0) return {-1, 0.0f};
    for (int y = 0; y < 26; y++) {
      for (int x = 0; x < 26; x++) {
        float sum = 0.0f;
        for (int ky = 0; ky < 3; ky++) {
          for (int kx = 0; kx < 3; kx++) {
            sum += input_image[y + ky][x + kx] * conv1_weights[c][0][ky][kx];
          }
        }
        c1[c][y][x] = sum * conv1_scale;
      }
    }
  }

  // 2. conv2: Depthwise Conv2d (groups = 4, output 4x24x24)
  for (int c = 0; c < 4; c++) {
    if (packetIndex > 0) return {-1, 0.0f};
    for (int y = 0; y < 24; y++) {
      for (int x = 0; x < 24; x++) {
        float sum = 0.0f;
        for (int ky = 0; ky < 3; ky++) {
          for (int kx = 0; kx < 3; kx++) {
            sum += c1[c][y + ky][x + kx] * conv2_weights[c][0][ky][kx];
          }
        }
        c2[c][y][x] = sum * conv2_scale;
      }
    }
  }

  // 3. Slicing & Trainable Projection H1_trainable
  for (int ch = 0; ch < 36; ch++) {
    if (packetIndex > 0) return {-1, 0.0f};
    int s = ch / 4;
    int c = ch % 4;
    int I_val = SLICES_CONFIG[s].I;
    int D_val = SLICES_CONFIG[s].D;
    for (int z = 0; z < 12; z++) {
      float sum = 0.0f;
      for (int idx = 0; idx < 576; idx++) {
        int v = idx / I_val;
        int u = idx % I_val;
        int k = u * D_val + v;
        int y = k / 24;
        int x = k % 24;
        
        sum += c2[c][y][x] * H1_trainable_weights[ch][idx][z];
      }
      if (sum < 0.0f) sum = 0.0f;
      o_trainable[ch][z] = sum * H1_trainable_scale;
    }
  }

  // ==========================================
  // STREAM 2: Fixed Conv Edge Detection
  // ==========================================
  
  // 1. Sobel convolution over input_image[28][28] -> output fixed_conv_out[4][26][26]
  for (int y = 0; y < 26; y++) {
    if (packetIndex > 0) return {-1, 0.0f};
    for (int x = 0; x < 26; x++) {
      float sum_x = 0.0f;
      float sum_y = 0.0f;
      float sum_d1 = 0.0f;
      float sum_d2 = 0.0f;
      for (int ky = 0; ky < 3; ky++) {
        for (int kx = 0; kx < 3; kx++) {
          float val = input_image[y + ky][x + kx];
          sum_x += val * SOBEL_X[ky][kx];
          sum_y += val * SOBEL_Y[ky][kx];
          sum_d1 += val * SOBEL_DIAG1[ky][kx];
          sum_d2 += val * SOBEL_DIAG2[ky][kx];
        }
      }
      fixed_conv_out[0][y][x] = sum_x;
      fixed_conv_out[1][y][x] = sum_y;
      fixed_conv_out[2][y][x] = sum_d1;
      fixed_conv_out[3][y][x] = sum_d2;
    }
  }

  // 2. o_fixed = einsum('bcxy,cyz->bcz', fixed_conv_out, H1_fixed)
  for (int c = 0; c < 4; c++) {
    if (packetIndex > 0) return {-1, 0.0f};
    float S[26];
    for (int y = 0; y < 26; y++) {
      float sum_x = 0.0f;
      for (int x = 0; x < 26; x++) {
        sum_x += fixed_conv_out[c][x][y];
      }
      S[y] = sum_x;
    }
    
    for (int z = 0; z < 12; z++) {
      float sum_y = 0.0f;
      for (int y = 0; y < 26; y++) {
        sum_y += S[y] * H1_fixed_weights[c][y][z];
      }
      if (sum_y < 0.0f) sum_y = 0.0f;
      o_fixed[c][z] = sum_y * H1_fixed_scale;
    }
  }

  // ==========================================
  // CONCATENATE & FLATTEN
  // ==========================================
  for (int ch = 0; ch < 36; ch++) {
    for (int z = 0; z < 12; z++) {
      o_flat[ch * 12 + z] = o_trainable[ch][z];
    }
  }
  for (int ch = 0; ch < 4; ch++) {
    for (int z = 0; z < 12; z++) {
      o_flat[432 + ch * 12 + z] = o_fixed[ch][z];
    }
  }

  // ==========================================
  // CLASSIFIER DENSE LAYERS
  // ==========================================
  for (int j = 0; j < 128; j++) {
    float sum = 0.0f;
    for (int i = 0; i < 480; i++) {
      sum += o_flat[i] * H2_weights[i][j];
    }
    if (sum < 0.0f) sum = 0.0f;
    h2[j] = sum * H2_scale;
  }

  for (int k = 0; k < 47; k++) {
    float sum = 0.0f;
    for (int j = 0; j < 128; j++) {
      sum += h2[j] * H3_weights[j][k];
    }
    final_out[k] = sum * H3_scale;
  }

  uint32_t t_inference = millis() - t_start;

  // ==========================================
  // ARGMAX PREDICTION (WITH STROKE FILTERING)
  // ==========================================
  int predicted_digit = -1;
  float max_score = -999999.0f;
  for (int k = 0; k < 47; k++) {
    if (expectedStrokes > 0) {
      if (!isValidClass(k, expectedStrokes)) {
        continue;
      }
    }
    if (final_out[k] > max_score) {
      max_score = final_out[k];
      predicted_digit = k;
    }
  }

  if (predicted_digit == -1) {
    predicted_digit = 0;
    max_score = final_out[0];
    for (int k = 1; k < 47; k++) {
      if (final_out[k] > max_score) {
        max_score = final_out[k];
        predicted_digit = k;
      }
    }
  }

  if (printDetails) {
    // Print results
    Serial.println("================================================");
    Serial.print("Predicted Class Index: ");
    Serial.println(predicted_digit);
    Serial.print("Predicted Character:   ");
    Serial.println(EMNIST_CLASSES[predicted_digit]);
    Serial.print("Confidence Score:      ");
    Serial.println(max_score, 4);
    Serial.print("Inference Time:        ");
    Serial.print(t_inference);
    Serial.println(" ms");
    Serial.println("================================================");
  }
  return {predicted_digit, max_score};
}

void populate_input_image(uint8_t* data) {
  for (int y = 0; y < 28; y++) {
    for (int x = 0; x < 28; x++) {
      int bitIndex = y * 28 + x;
      // 檢查對應的 bit 是否為 1 (ink)
      bool isInk = (data[bitIndex / 8] & (1 << (7 - (bitIndex % 8)))) != 0;
      // 轉置給模型用 (把 (y, x) 存入 (x, y))
      input_image[x][y] = isInk ? 1.0f : 0.0f;
    }
  }
}

// ---------------------------------------------------------
// Parallel Receiving and Decoding
// ---------------------------------------------------------
const int CLOCK_PIN = 2;

// 調整大小：98 Bytes (圖片) + 4 Bytes (C3 Latency) + 4 Bytes (Node Latency) + 1 Byte (Strokes)
const int msglen = 107;
const int ECC_LEN = 10;
const int FRAME_SIZE = msglen + ECC_LEN; // 117
const int NUM_FRAMES = 5;

volatile uint8_t rx_buffer[2][NUM_FRAMES * FRAME_SIZE];
volatile uint8_t currentBuffer = 0;
uint8_t repaired_msg[msglen];

RS::ReedSolomon<msglen, ECC_LEN> rs;

volatile uint32_t packetIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  REG_PIOC_ODR = 0xFFFFFFFF; // Set Port C as inputs
  pinMode(CLOCK_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), onClockPulse, RISING);
  
  Serial.println("啟動");
  Serial.println("等待資料...");
}

void onClockPulse() {
  if (packetIndex < (NUM_FRAMES * FRAME_SIZE)) {
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

    rx_buffer[currentBuffer][packetIndex++] = corrected;
  }
}

void printGrid(uint8_t* data) {
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  收到手寫辨識圖形 (28x28)                ║");
  Serial.println("╠══════════════════════════════════════════╣");
  
  for (int y = 0; y < 28; y++) {
    Serial.print("║");
    for (int x = 0; x < 28; x++) {
      int bitIndex = y * 28 + x;
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
  static uint32_t lastIdx = 0;
  static uint32_t lastTime = 0;

  if (packetIndex != lastIdx) {
    lastIdx = packetIndex;
    lastTime = micros();
  }

  // 偵測到有資料，且超過 50ms 沒變化，或是緩衝區已滿，視為一次 Burst 完成
  if (packetIndex > 0 && (micros() - lastTime > 50000 || packetIndex >= NUM_FRAMES * FRAME_SIZE)) {
      
      noInterrupts();
      uint32_t receivedBytes = packetIndex;
      uint8_t procBuf = currentBuffer;
      currentBuffer = 1 - currentBuffer; // 交換緩衝區
      packetIndex = 0;                   // 重置計數，讓 ISR 可隨時接收下一次中斷
      interrupts();
      
      lastIdx = 0;
      uint32_t durationUs = micros() - lastTime; 
      
      int frames_received = receivedBytes / FRAME_SIZE;

      if (frames_received > 0) {
        Serial.println("\n--- 傳輸完成 ---");
        Serial.print("Received bytes: "); Serial.println(receivedBytes);
        Serial.print("Frames complete: "); Serial.println(frames_received);

        int repairStatus = rs.Decode((uint8_t*)&rx_buffer[procBuf][0], repaired_msg);
        
        if (repairStatus >= 0) {
            uint8_t receivedStrokes = repaired_msg[106];
            printGrid(repaired_msg);
            populate_input_image(repaired_msg);
            
            if (packetIndex > 0) return; // 如果在印出時有新資料進來，提早中斷
            
            Serial.print("\n[原始圖片推論] 收到筆畫數: "); Serial.println(receivedStrokes);
            InfResult res = run_inference(true, receivedStrokes);
            if (res.digit == -1) return; // 推論被新資料打斷
            
            int best_digit = res.digit;
            float best_score = res.score;
            
            if (res.score < 12.0 && frames_received > 1) {
                Serial.println("\n--- 分數低於 12.0，啟動 WASD 微調評估 ---");
                const char* dirs[4] = {"W", "A", "S", "D"};
                for (int i = 1; i < min(5, frames_received); i++) {
                    if (packetIndex > 0) return; // 如果被新資料打斷
                    
                    int status = rs.Decode((uint8_t*)&rx_buffer[procBuf][i * FRAME_SIZE], repaired_msg);
                    if (status >= 0) {
                        populate_input_image(repaired_msg);
                        InfResult adj_res = run_inference(false, receivedStrokes); 
                        if (adj_res.digit == -1) return;
                        
                        Serial.print("[微調 "); Serial.print(dirs[i-1]); Serial.print("] Character: ");
                        Serial.print(EMNIST_CLASSES[adj_res.digit]);
                        Serial.print(", Score: "); Serial.println(adj_res.score, 4);
                        
                        if (adj_res.score > best_score) {
                            best_score = adj_res.score;
                            best_digit = adj_res.digit;
                        }
                    } else {
                        Serial.print("[微調 "); Serial.print(dirs[i-1]); Serial.println("] Decode failed.");
                    }
                }
            }
            
            if (packetIndex > 0) return;
            
            Serial.println("\n================================================");
            Serial.print("FINAL Predicted Character:   ");
            Serial.println(EMNIST_CLASSES[best_digit]);
            Serial.print("FINAL Confidence Score:      ");
            Serial.println(best_score, 4);
            Serial.println("================================================");
            
        } else {
            Serial.println(" | 狀態:資料損毀");
        }
      }
  }
}
