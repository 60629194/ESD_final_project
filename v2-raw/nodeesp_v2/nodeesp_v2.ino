#include <WiFi.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include "RS_FEC.h"
#include "soc/gpio_reg.h"
#include <math.h>

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

// ============================================================
// V2 封包定義與緩衝
// ============================================================
enum PktType { PKT_DATA = 0, PKT_DONE = 1 };

#pragma pack(push, 1)
struct V2Packet {
  uint8_t type; 
  float ax;
  float ay;
  float dt;
  bool ink;
  bool stationary;
  uint32_t t_c3;
};
#pragma pack(pop)

const int MAX_POINTS = 1200;
struct Sample {
  float ax, ay;
  float dt;
  bool  stationary;
  bool  ink;
};
Sample  buf[MAX_POINTS];
int     pointCount = 0;

float   rawPathX[MAX_POINTS], rawPathY[MAX_POINTS];
float   pathX[MAX_POINTS],    pathY[MAX_POINTS];
uint8_t grid[28][28];

volatile bool hasDone = false;
volatile uint32_t t_c3_rx = 0;
volatile uint32_t t_node_start = 0;

// ============================================================
// ESP-NOW 接收回呼函式
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(V2Packet)) {
    V2Packet* pkt = (V2Packet*)incomingData;
    if (pkt->type == PKT_DATA) {
      if (pointCount < MAX_POINTS) {
        buf[pointCount].ax = pkt->ax;
        buf[pointCount].ay = pkt->ay;
        buf[pointCount].dt = pkt->dt;
        buf[pointCount].ink = pkt->ink;
        buf[pointCount].stationary = pkt->stationary;
        pointCount++;
      }
    } else if (pkt->type == PKT_DONE) {
      t_node_start = micros();
      t_c3_rx = pkt->t_c3;
      hasDone = true;
    }
  }
}

// ============================================================
// 構圖與軌跡演算法 (移植自 V1 ESP32-C3)
// ============================================================
void generateTrajectory() {
  static float svX[MAX_POINTS], svY[MAX_POINTS];
  rawPathX[0] = rawPathY[0] = 0;

  int seg = 0;
  while (seg < pointCount) {
    int segEnd = seg;
    while (segEnd < pointCount-1 && buf[segEnd+1].ink == buf[seg].ink)
      segEnd++;
    int len = segEnd - seg + 1;

    svX[0] = svY[0] = 0;
    for (int i = 1; i < len; i++) {
      int idx = seg + i;
      if (buf[idx].stationary) {
        svX[i] = svY[i] = 0;
      } else {
        svX[i] = svX[i-1] + buf[idx].ax * buf[idx].dt;
        svY[i] = svY[i-1] + buf[idx].ay * buf[idx].dt;
      }
    }

    float errVx = svX[len-1], errVy = svY[len-1];
    float segTime = 0;
    for (int i = 0; i < len; i++) segTime += buf[seg+i].dt;
    if (segTime < 0.001f) segTime = 0.001f;

    float elapsed = 0;
    for (int i = 1; i < len; i++) {
      int idx = seg + i;
      elapsed += buf[idx].dt;
      float ratio = elapsed / segTime;
      float cvX = svX[i] - errVx * ratio;
      float cvY = svY[i] - errVy * ratio;
      rawPathX[idx] = rawPathX[idx-1] + cvX * buf[idx].dt;
      rawPathY[idx] = rawPathY[idx-1] + cvY * buf[idx].dt;
    }

    seg = segEnd + 1;
    if (seg < pointCount) {
      rawPathX[seg] = rawPathX[seg-1];
      rawPathY[seg] = rawPathY[seg-1];
    }
  }
}

void drawLine(int x0, int y0, int x1, int y1, uint8_t val) {
  int dx=abs(x1-x0), sx=x0<x1?1:-1;
  int dy=-abs(y1-y0), sy=y0<y1?1:-1;
  int err=dx+dy;
  while (true) {
    if (x0>=0&&x0<28&&y0>=0&&y0<28) {
      if (grid[y0][x0] == 255 || val == 0)
        grid[y0][x0] = val;
    }
    if (x0==x1&&y0==y1) break;
    int e2=2*err;
    if (e2>=dy){err+=dy;x0+=sx;}
    if (e2<=dx){err+=dx;y0+=sy;}
  }
}

// ============================================================
// 並列傳輸底層函式
// ============================================================
#define SETTLE_DELAY() { for (volatile int x = 0; x < 20; x++) __asm__ __volatile__("nop"); }

void sendByteFast(uint8_t data) {
  REG_WRITE(GPIO_OUT_W1TC_REG, DATA_MASK);
  REG_WRITE(GPIO_OUT_W1TS_REG, gpio_lookup[data]);
  SETTLE_DELAY();
  REG_WRITE(GPIO_OUT_W1TS_REG, CLOCK_MASK);
  SETTLE_DELAY();
  REG_WRITE(GPIO_OUT_W1TC_REG, CLOCK_MASK);
  SETTLE_DELAY();
}

// ============================================================
// 處理並發送
// ============================================================
void processAndSendToDue() {
  if (pointCount < 5) return;
  generateTrajectory();
  
  // Check original ink count first
  int origInkCnt = 0;
  for (int i = 0; i < pointCount; i++) {
    if (buf[i].ink) origInkCnt++;
  }
  if (origInkCnt < 2) return;

  float tweaks[5][2] = {
    {0.0, 0.0},
    {0.0, -0.001}, // w
    {-0.001, 0.0}, // a
    {0.0, 0.001},  // s
    {0.001, 0.0}   // d
  };

  uint32_t t1 = micros();
  uint32_t total_t_node = 0;

  for (int step = 0; step < 5; step++) {
    float tweak_dx = tweaks[step][0];
    float tweak_dy = tweaks[step][1];

    for (int i = 0; i < pointCount; i++) {
      pathX[i] = rawPathX[i] + (float)i * tweak_dx;
      pathY[i] = rawPathY[i] + (float)i * tweak_dy;
    }

    float minX=99999,maxX=-99999,minY=99999,maxY=-99999;
    int inkCnt = 0;
    for (int i = 0; i < pointCount; i++) {
      if (!buf[i].ink) continue;
      if (pathX[i]<minX) minX=pathX[i]; if (pathX[i]>maxX) maxX=pathX[i];
      if (pathY[i]<minY) minY=pathY[i]; if (pathY[i]>maxY) maxY=pathY[i];
      inkCnt++;
    }
    
    // Prevent div by zero, should be rare if origInkCnt >= 2
    if (inkCnt < 2) continue;

    float w=maxX-minX; if(w<0.001f)w=0.001f;
    float h=maxY-minY; if(h<0.001f)h=0.001f;
    float scale = min(20.0f/w, 20.0f/h);
    float cx=(minX+maxX)*0.5f, cy=(minY+maxY)*0.5f;

    for (int y=0;y<28;y++) for (int x=0;x<28;x++) grid[y][x]=255;

    int lastGX=-1, lastGY=-1;
    bool lastWasAny = false;
    bool lastWasInk = false;

    for (int i = 0; i < pointCount; i++) {
      int gx = constrain((int)roundf(14.0f - (pathX[i]-cx)*scale), 0, 27);
      int gy = constrain((int)roundf(14.0f + (pathY[i]-cy)*scale), 0, 27);
      uint8_t val = buf[i].ink ? 0 : 128;

      if (lastWasAny) {
        if (buf[i].ink == lastWasInk) {
          drawLine(lastGX, lastGY, gx, gy, val);
        } else {
          if (gx>=0&&gx<28&&gy>=0&&gy<28) {
            if (grid[gy][gx]==255 || val==0) grid[gy][gx]=val;
          }
        }
      } else {
        if (gx>=0&&gx<28&&gy>=0&&gy<28) {
          if (grid[gy][gx]==255 || val==0) grid[gy][gx]=val;
        }
      }
      lastGX=gx; lastGY=gy;
      lastWasAny=true;
      lastWasInk=buf[i].ink;
    }

    // === 加粗 (Thicken) ===
    uint8_t thick_grid[28][28];
    for (int y = 0; y < 28; y++) {
      for (int x = 0; x < 28; x++) {
        thick_grid[y][x] = 255;
      }
    }
    for (int y = 0; y < 28; y++) {
      for (int x = 0; x < 28; x++) {
        if (grid[y][x] == 0) {
          // 3x3 矩陣加粗
          for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
              int ny = y + dy, nx = x + dx;
              if (ny >= 0 && ny < 28 && nx >= 0 && nx < 28) {
                thick_grid[ny][nx] = 0;
              }
            }
          }
        }
      }
    }

    // === 壓縮 784 bits 成 98 bytes (維持正向) ===
    memset(raw_msg, 0, msglen);
    for (int y = 0; y < 28; y++) {
      for (int x = 0; x < 28; x++) {
        int bitIndex = y * 28 + x;
        if (thick_grid[y][x] == 0) { // 若為實心墨水點
          raw_msg[bitIndex / 8] |= (1 << (7 - (bitIndex % 8)));
        }
      }
    }

    // 測量 NodeMCU 在 RS-FEC 之前的構圖處理時間
    uint32_t t_node = micros() - t_node_start; // Cumulative time
    total_t_node = t_node;
    uint32_t local_t_c3 = t_c3_rx;
    
    memcpy(&raw_msg[98], &local_t_c3, 4);
    memcpy(&raw_msg[102], &t_node, 4);

    // RS-FEC 編碼 (106 bytes -> 116 bytes)
    rs.Encode(raw_msg, encoded_msg);

    // 透過 Parallel Bus 傳送給 Due
    for (int i = 0; i < (msglen + ECC_LEN); i++) {
      sendByteFast(encoded_msg[i]);
    }
  }

  uint32_t t2 = micros();

  Serial.println("\n[NodeMCU V2] 5張圖片(原圖+WASD微調)構圖與轉發完成！");
  Serial.printf("[Latency] NodeMCU 總構圖耗時: %u us\n", total_t_node);
  Serial.printf("[Latency] NodeMCU 總發送耗時: %u us\n", (uint32_t)(t2 - t1));
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

  Serial.println("✅ NodeMCU 中繼端已啟動 (V2 串流構圖版 + WASD微調)");
  Serial.println("等待 ESP32-C3 即時傳輸感測資料...");
}

void loop() {
  if (hasDone) {
    hasDone = false;
    processAndSendToDue();
    pointCount = 0; // 重置以準備下一個字
  }
}
