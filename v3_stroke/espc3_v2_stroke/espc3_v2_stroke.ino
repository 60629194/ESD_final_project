/*
 * ============================================================
 *  ESP32-C3 SuperMini + MPU6050  手寫輸入器 (WiFi / ESP-NOW V2)
 *  負責即時讀取 MPU6050，將 raw data 傳送給 NodeMCU。
 * ============================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

Adafruit_MPU6050 mpu;

// ─── 廣播 MAC ────────────────────────────────────────────────
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataSent(const wifi_tx_info_t *pi, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  // 為了效能，即時傳輸時不印出狀態
}

// ─── 腳位 ────────────────────────────────────────────────────
const int BUTTON_DRAW_PIN = 2;
const int BUTTON_SEND_PIN = 1;

// ─── 靜止偵測門檻 ────────────────────────────────────────────
const float GYRO_STILL_THR  = 0.12f;   // rad/s
const float ACCEL_STILL_THR = 0.25f;   // m/s²
const int   STILL_CONFIRM_MS = 60;     // ms

// ─── Mahony 參數（僅用於校準期間收斂四元數）────────────────
const float MAHONY_KP = 2.0f;
const float MAHONY_KI = 0.005f;

// ─── V2 封包定義 ────────────────────────────────────────────
enum PktType { PKT_DATA = 0, PKT_DONE = 1 };

#pragma pack(push, 1)
struct V2Packet {
  uint8_t type; 
  // 若為 PKT_DATA
  float ax;
  float ay;
  float dt;
  bool ink;
  bool stationary;
  // 若為 PKT_DONE
  uint32_t t_c3;
  uint8_t strokes;
};
#pragma pack(pop)

// ─── 狀態機 ──────────────────────────────────────────────────
enum State { CALIB_WAIT, CALIB_REC, IDLE, RECORDING_INK, RECORDING_AIR };
State currentState = CALIB_WAIT;
uint8_t currentStrokes = 0;

// ─── 加速度計校準（零點 bias） ────────────────────────────────
float bias_ax = 0, bias_ay = 0, bias_az = 0;
unsigned long calibStartMs = 0;
float sum_ax = 0, sum_ay = 0, sum_az = 0;
int   calibCount = 0;

// ─── 四元數（只在校準期間用，校準後凍結投影矩陣）────────────
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float iFBx = 0, iFBy = 0, iFBz = 0;

// ─── 凍結投影向量（感測器座標系下的平面基向量）──────────────
float sXx, sXy, sXz;
float sYx, sYy, sYz;
bool  projFrozen = false;

// ─── 按鈕 ────────────────────────────────────────────────────
bool lastSendState   = HIGH;
bool lastDrawState   = HIGH;
bool bothKeysWasHeld = false;

// ─── 靜止確認 ────────────────────────────────────────────────
unsigned long stillSinceMs = 0;
bool previouslyStill = false;

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm > 0.001f) {
    ax /= norm; ay /= norm; az /= norm;
    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;
    iFBx += MAHONY_KI*ex*dt; iFBy += MAHONY_KI*ey*dt; iFBz += MAHONY_KI*ez*dt;
    gx += MAHONY_KP*ex + iFBx;
    gy += MAHONY_KP*ey + iFBy;
    gz += MAHONY_KP*ez + iFBz;
  }
  float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz)*dt;
  float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy)*dt;
  float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx)*dt;
  float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx)*dt;
  q0+=dq0; q1+=dq1; q2+=dq2; q3+=dq3;
  norm = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
  q0/=norm; q1/=norm; q2/=norm; q3/=norm;
}

void freezeProjectionMatrix() {
  float nx = bias_ax, ny = bias_ay, nz = bias_az;
  float nn = sqrtf(nx*nx+ny*ny+nz*nz);
  if (nn < 0.001f) return;
  nx/=nn; ny/=nn; nz/=nn;

  float ax, ay, az;
  if (fabsf(nz) < 0.9f) { ax=0; ay=0; az=1; }
  else                   { ax=1; ay=0; az=0; }
  float wx = ny*az - nz*ay;
  float wy = nz*ax - nx*az;
  float wz = nx*ay - ny*ax;
  float wn = sqrtf(wx*wx+wy*wy+wz*wz);
  wx/=wn; wy/=wn; wz/=wn;

  float ux = ny*wz - nz*wy;
  float uy = nz*wx - nx*wz;
  float uz = nx*wy - ny*wx;

  sXx = (1-2*(q2*q2+q3*q3))*wx + 2*(q1*q2+q0*q3)*wy + 2*(q1*q3-q0*q2)*wz;
  sXy = 2*(q1*q2-q0*q3)*wx + (1-2*(q1*q1+q3*q3))*wy + 2*(q2*q3+q0*q1)*wz;
  sXz = 2*(q1*q3+q0*q2)*wx + 2*(q2*q3-q0*q1)*wy + (1-2*(q1*q1+q2*q2))*wz;

  sYx = (1-2*(q2*q2+q3*q3))*ux + 2*(q1*q2+q0*q3)*uy + 2*(q1*q3-q0*q2)*uz;
  sYy = 2*(q1*q2-q0*q3)*ux + (1-2*(q1*q1+q3*q3))*uy + 2*(q2*q3+q0*q1)*uz;
  sYz = 2*(q1*q3+q0*q2)*ux + 2*(q2*q3-q0*q1)*uy + (1-2*(q1*q1+q2*q2))*uz;

  projFrozen = true;

  float dot = sXx*sYx + sXy*sYy + sXz*sYz;
  Serial.printf("   sX=(%.3f,%.3f,%.3f)\n", sXx, sXy, sXz);
  Serial.printf("   sY=(%.3f,%.3f,%.3f)\n", sYx, sYy, sYz);
  Serial.printf("   正交誤差=%.4f\n", fabsf(dot));
}

inline void project(float ax, float ay, float az, float &px, float &py) {
  px = ax*sXx + ay*sXy + az*sXz;
  py = ax*sYx + ay*sYy + az*sYz;
}

void checkSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length()==0) return;
  char first = toLowerCase(cmd.charAt(0));
  if (first=='c') {
    currentState=CALIB_WAIT;
    projFrozen=false;
    q0=1;q1=q2=q3=0;iFBx=iFBy=iFBz=0;
    Serial.println("🔄 重新校準 → 握好筆靜止，按住 GPIO2 三秒");
    return;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_DRAW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SEND_PIN, INPUT_PULLUP);
  
  // === 初始化 WiFi 與 ESP-NOW ===
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW 初始化失敗");
    while(1) delay(10);
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ 無法新增 ESP-NOW Peer");
    while(1) delay(10);
  }
  
  Wire.begin(8, 9);
  if (!mpu.begin()) { Serial.println("❌ MPU6050 未找到"); while(1)delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║ ESP32-C3 手寫輸入器 v2.2 (V2串流版)    ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║  此版本將感測資料即時串流給 NodeMCU    ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("\n[校準] 握好筆保持靜止，按住 GPIO2 三秒...");
}

unsigned long last_micros = 0;

void loop() {
  checkSerialCommands();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now_us = micros();
  float dt = (now_us - last_micros) / 1000000.0f;
  last_micros = now_us;
  if (dt<=0||dt>0.1f) return;

  bool isDrawPressed  = (digitalRead(BUTTON_DRAW_PIN)==LOW);
  bool isSendPressed  = (digitalRead(BUTTON_SEND_PIN)==LOW);
  bool isSendTriggered = (isSendPressed && lastSendState==HIGH);
  bool isDrawReleased  = (!isDrawPressed && lastDrawState==LOW);
  lastSendState = isSendPressed ? LOW : HIGH;
  lastDrawState = isDrawPressed ? LOW : HIGH;

  float gx=g.gyro.x, gy=g.gyro.y, gz=g.gyro.z;
  float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
  float ax_net=ax-bias_ax, ay_net=ay-bias_ay, az_net=az-bias_az;

  float gyro_mag  = sqrtf(gx*gx+gy*gy+gz*gz);
  float accel_mag = sqrtf(ax_net*ax_net+ay_net*ay_net+az_net*az_net);
  bool instantStill = (gyro_mag<GYRO_STILL_THR) && (accel_mag<ACCEL_STILL_THR);

  unsigned long nowMs = millis();
  if (instantStill) {
    if (!previouslyStill) stillSinceMs=nowMs;
    previouslyStill=true;
  } else {
    previouslyStill=false;
  }
  bool confirmedStill = instantStill && ((nowMs-stillSinceMs)>=STILL_CONFIRM_MS);

  bool bothHeld = isDrawPressed && isSendPressed;
  if (bothHeld && !bothKeysWasHeld &&
      projFrozen &&
      (currentState==IDLE || currentState==RECORDING_AIR)) {
    q0=1;q1=0;q2=0;q3=0;iFBx=iFBy=iFBz=0;
    for (int w=0;w<400;w++) mahonyUpdate(0,0,0,ax,ay,az,0.01f);
    freezeProjectionMatrix();
    Serial.println("🔄 雙鍵重置完成！投影方向已重新對齊");
  }
  bothKeysWasHeld = bothHeld;

  switch (currentState) {
    case CALIB_WAIT:
      if (isDrawPressed && !isSendPressed) {
        calibStartMs=millis();
        sum_ax=sum_ay=sum_az=0; calibCount=0;
        currentState=CALIB_REC;
        Serial.println("⏳ 校準中（3秒），保持靜止...");
      }
      break;

    case CALIB_REC:
      if (!isDrawPressed) {
        currentState=CALIB_WAIT;
        Serial.println("❌ 中斷，請重試");
        break;
      }
      mahonyUpdate(gx,gy,gz,ax,ay,az,dt);
      sum_ax+=ax; sum_ay+=ay; sum_az+=az; calibCount++;

      if (millis()-calibStartMs >= 3000) {
        bias_ax=sum_ax/calibCount;
        bias_ay=sum_ay/calibCount;
        bias_az=sum_az/calibCount;

        q0=1;q1=0;q2=0;q3=0;iFBx=iFBy=iFBz=0;
        for (int w=0;w<400;w++) mahonyUpdate(0,0,0,bias_ax,bias_ay,bias_az,0.01f);
        freezeProjectionMatrix();

        Serial.println("✅ 校準完成！");
        Serial.println("📝 按住 GPIO2 落筆，放開抬筆，GPIO1 送出");
        currentState=IDLE;
      }
      break;

    case IDLE:
      if (isDrawPressed && !isSendPressed) {
        currentState=RECORDING_INK;
        currentStrokes = 1;
        Serial.println("🖊️  記錄與串流中...");
      }
      break;

    case RECORDING_INK:
    case RECORDING_AIR:
      if (currentState==RECORDING_INK && !isDrawPressed)
        currentState=RECORDING_AIR;
      else if (currentState==RECORDING_AIR && isDrawPressed && !isSendPressed) {
        currentState=RECORDING_INK;
        currentStrokes++;
      }

      if (isSendTriggered && !isDrawPressed) {
        unsigned long sendStartUs = micros();
        Serial.println("📤 發送結束訊號 [DONE]...");
        
        V2Packet pkt;
        pkt.type = PKT_DONE;
        pkt.t_c3 = micros() - sendStartUs; // C3 幾乎沒有處理，延遲極低
        pkt.strokes = currentStrokes;
        esp_now_send(broadcastAddress, (uint8_t*)&pkt, sizeof(pkt));
        
        currentState=IDLE;
        break;
      }

      // 即時串流資料點
      float px, py;
      project(ax_net, ay_net, az_net, px, py);
      
      V2Packet pkt;
      pkt.type = PKT_DATA;
      pkt.ax = px;
      pkt.ay = py;
      pkt.dt = dt;
      pkt.stationary = confirmedStill;
      pkt.ink = (currentState==RECORDING_INK);
      
      esp_now_send(broadcastAddress, (uint8_t*)&pkt, sizeof(pkt));
      break;
  }

  delay(10);
}
