/*
 * ============================================================
 *  ESP32-C3 SuperMini + MPU6050  手寫輸入器  v2.2
 * ============================================================
 *  v2.2 核心改動：
 *    ① 凍結投影矩陣（徹底解決逆時針漂移）
 *       校準完成後把平面基向量轉換到感測器座標系，
 *       寫字時直接對原始加速度做點積，完全不用四元數，
 *       四元數漂移不再影響投影方向。
 *    ② 統一單筆畫積分：全程（含 air 段）連續積分，
 *       渲染時 ink=實心██，air=虛線░░，方便判斷對準效果。
 *    ③ 雙鍵重置：同按 GPIO1+GPIO2 重新對齊凍結矩陣到當前姿態。
 *    ④ 左右翻轉輸出（-X）。
 *
 *  硬體接線：
 *    MPU6050 SDA → GPIO 8
 *    MPU6050 SCL → GPIO 9
 *    DRAW 按鈕   → GPIO 2（按住=落筆，放開=抬筆）
 *    SEND 按鈕   → GPIO 1（單擊=送出）
 *
 *  Serial 指令（115200 baud）：
 *    w/s/a/d + 數值 → 微調 (e.g. "d0.02")
 *    r              → 重置微調
 *    c              → 重新校準（重開校準流程）
 * ============================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

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

// ─── 資料緩衝 ────────────────────────────────────────────────
const int MAX_POINTS = 1200;

struct Sample {
  float ax, ay;   // 凍結投影後的平面加速度
  float dt;
  bool  stationary;
  bool  ink;
};
Sample  buf[MAX_POINTS];
int     pointCount = 0;

float   rawPathX[MAX_POINTS], rawPathY[MAX_POINTS];
float   pathX[MAX_POINTS],    pathY[MAX_POINTS];

// 網格：0=ink，128=air，255=空白
uint8_t grid[28][28];

// ─── 狀態機 ──────────────────────────────────────────────────
enum State { CALIB_WAIT, CALIB_REC, IDLE, RECORDING_INK, RECORDING_AIR };
State currentState = CALIB_WAIT;

// ─── 加速度計校準（零點 bias） ────────────────────────────────
float bias_ax = 0, bias_ay = 0, bias_az = 0;
unsigned long calibStartMs = 0;
float sum_ax = 0, sum_ay = 0, sum_az = 0;
int   calibCount = 0;

// ─── 四元數（只在校準期間用，校準後凍結投影矩陣）────────────
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float iFBx = 0, iFBy = 0, iFBz = 0;

// ─── 凍結投影向量（感測器座標系下的平面基向量）──────────────
// project: px = dot(a_net, sX),  py = dot(a_net, sY)
// 校準完成後計算一次，之後永遠不變
float sXx, sXy, sXz;   // 右方向在感測器座標
float sYx, sYy, sYz;   // 下方向在感測器座標
bool  projFrozen = false;

// ─── 按鈕 ────────────────────────────────────────────────────
bool lastSendState   = HIGH;
bool lastDrawState   = HIGH;
bool bothKeysWasHeld = false;

// ─── 靜止確認 ────────────────────────────────────────────────
unsigned long stillSinceMs = 0;
bool previouslyStill = false;

// ─── 微調 ────────────────────────────────────────────────────
float tweak_dx = 0, tweak_dy = 0;

// ============================================================
//  Mahony 互補濾波（只在校準收斂用）
// ============================================================
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

// ============================================================
//  凍結投影矩陣
//  把世界座標的平面基向量用 R^T（當前四元數的逆）轉到感測器座標，
//  之後 project() 只做點積，完全不碰四元數。
// ============================================================
void freezeProjectionMatrix() {
  // 重力方向（世界座標：校準時靜止，重力 = 負 Z）
  // 用叉積建立兩個平面基向量（世界座標）
  // 法向量 N = 歸一化(bias)（指向重力方向）
  float nx = bias_ax, ny = bias_ay, nz = bias_az;
  float nn = sqrtf(nx*nx+ny*ny+nz*nz);
  if (nn < 0.001f) return;
  nx/=nn; ny/=nn; nz/=nn;

  // 平面 X（任意正交方向，可能與真實「右」不同，
  //         但每次重開校準結果一致 → 方向固定）
  float ax, ay, az;
  if (fabsf(nz) < 0.9f) { ax=0; ay=0; az=1; }
  else                   { ax=1; ay=0; az=0; }
  float wx = ny*az - nz*ay;
  float wy = nz*ax - nx*az;
  float wz = nx*ay - ny*ax;
  float wn = sqrtf(wx*wx+wy*wy+wz*wz);
  wx/=wn; wy/=wn; wz/=wn;

  // 平面 Y = N × planeX
  float ux = ny*wz - nz*wy;
  float uy = nz*wx - nx*wz;
  float uz = nx*wy - ny*wx;

  // 把世界座標的 (wx,wy,wz) 和 (ux,uy,uz) 轉到感測器座標
  // R^T * v：R^T 的列 = R 的行，從四元數展開轉置
  // R^T = R 的逆（正交矩陣），行向量 = 感測器軸在世界的方向
  // sX = R^T * worldX
  sXx = (1-2*(q2*q2+q3*q3))*wx + 2*(q1*q2+q0*q3)*wy + 2*(q1*q3-q0*q2)*wz;
  sXy = 2*(q1*q2-q0*q3)*wx + (1-2*(q1*q1+q3*q3))*wy + 2*(q2*q3+q0*q1)*wz;
  sXz = 2*(q1*q3+q0*q2)*wx + 2*(q2*q3-q0*q1)*wy + (1-2*(q1*q1+q2*q2))*wz;

  sYx = (1-2*(q2*q2+q3*q3))*ux + 2*(q1*q2+q0*q3)*uy + 2*(q1*q3-q0*q2)*uz;
  sYy = 2*(q1*q2-q0*q3)*ux + (1-2*(q1*q1+q3*q3))*uy + 2*(q2*q3+q0*q1)*uz;
  sYz = 2*(q1*q3+q0*q2)*ux + 2*(q2*q3-q0*q1)*uy + (1-2*(q1*q1+q2*q2))*uz;

  projFrozen = true;

  // 驗證正交
  float dot = sXx*sYx + sXy*sYy + sXz*sYz;
  Serial.printf("   sX=(%.3f,%.3f,%.3f)\n", sXx, sXy, sXz);
  Serial.printf("   sY=(%.3f,%.3f,%.3f)\n", sYx, sYy, sYz);
  Serial.printf("   正交誤差=%.4f\n", fabsf(dot));
}

// ============================================================
//  投影：感測器去 bias 加速度 → 寫字平面 (px, py)
//  完全不用四元數，只做兩個點積
// ============================================================
inline void project(float ax, float ay, float az,
                    float &px, float &py) {
  px = ax*sXx + ay*sXy + az*sXz;
  py = ax*sYx + ay*sYy + az*sYz;
}

// ============================================================
//  統一單筆畫積分
//  全程（含 air 段）連續積分，保持座標系連續。
//  子段（ink/air 連續段）各自做線性殘差修正。
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

    // 正向積分，靜止強制歸零
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

    // 線性殘差修正（假設子段首尾速度=0）
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

// ============================================================
//  Bresenham 畫線（value=0 ink，128 air）
// ============================================================
void drawLine(int x0, int y0, int x1, int y1, uint8_t val) {
  int dx=abs(x1-x0), sx=x0<x1?1:-1;
  int dy=-abs(y1-y0), sy=y0<y1?1:-1;
  int err=dx+dy;
  while (true) {
    if (x0>=0&&x0<28&&y0>=0&&y0<28) {
      // ink 可以蓋掉 air，但 air 不能蓋 ink
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
//  渲染並輸出
//  ink  段：實心 ██（黑）
//  air  段：虛線 ░░（灰，顯示空中路徑）
//  空白：      　　（空）
// ============================================================
void processFinalWordAndOutput() {
  if (pointCount < 5) return;
  generateTrajectory();

  for (int i = 0; i < pointCount; i++) {
    pathX[i] = rawPathX[i] + (float)i * tweak_dx;
    pathY[i] = rawPathY[i] + (float)i * tweak_dy;
  }

  // 邊界（只用 ink 點）
  float minX=99999,maxX=-99999,minY=99999,maxY=-99999;
  int inkCnt = 0;
  for (int i = 0; i < pointCount; i++) {
    if (!buf[i].ink) continue;
    if (pathX[i]<minX) minX=pathX[i]; if (pathX[i]>maxX) maxX=pathX[i];
    if (pathY[i]<minY) minY=pathY[i]; if (pathY[i]>maxY) maxY=pathY[i];
    inkCnt++;
  }
  if (inkCnt < 2) return;

  float w=maxX-minX; if(w<0.001f)w=0.001f;
  float h=maxY-minY; if(h<0.001f)h=0.001f;
  float scale = min(20.0f/w, 20.0f/h);
  float cx=(minX+maxX)*0.5f, cy=(minY+maxY)*0.5f;

  // 清空
  for (int y=0;y<28;y++) for (int x=0;x<28;x++) grid[y][x]=255;

  int lastGX=-1, lastGY=-1;
  bool lastWasAny = false;
  bool lastWasInk = false;

  for (int i = 0; i < pointCount; i++) {
    // 左右翻轉：用 14 - ...
    int gx = constrain((int)roundf(14.0f - (pathX[i]-cx)*scale), 0, 27);
    int gy = constrain((int)roundf(14.0f + (pathY[i]-cy)*scale), 0, 27);
    uint8_t val = buf[i].ink ? 0 : 128;

    if (lastWasAny) {
      // 只有同類型才連線（ink-ink 或 air-air）
      // ink→air 或 air→ink 的跨段，只畫點
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

  // 輸出
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  手寫輸入器 v2.2  ██=落筆 ░░=抬筆路徑  ║");
  Serial.println("╠══════════════════════════════════════════╣");
  for (int y=0;y<28;y++) {
    Serial.print("║");
    for (int x=0;x<28;x++) {
      if      (grid[y][x]==0)   Serial.print("██");
      else if (grid[y][x]==128) Serial.print("░░");
      else                      Serial.print("  ");
    }
    Serial.println("║");
  }
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.printf("  ink點=%d  總點=%d\n", inkCnt, pointCount);
}

// ============================================================
//  Serial 指令
// ============================================================
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
  if (cmd.length()>=2) {
    float val=cmd.substring(1).toFloat();
    if      (first=='w') tweak_dy-=val;
    else if (first=='s') tweak_dy+=val;
    else if (first=='a') tweak_dx-=val;
    else if (first=='d') tweak_dx+=val;
    else if (first=='r') { tweak_dx=0;tweak_dy=0;Serial.println("🔄 微調重置"); }
    if (pointCount>0) processFinalWordAndOutput();
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_DRAW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SEND_PIN, INPUT_PULLUP);
  Wire.begin(8, 9);
  if (!mpu.begin()) { Serial.println("❌ MPU6050 未找到"); while(1)delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   ESP32-C3 手寫輸入器 v2.2 啟動       ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║  1. 握好筆靜止，按住 GPIO2 滿 3 秒     ║");
  Serial.println("║  2. 按住 GPIO2 落筆，放開=抬筆         ║");
  Serial.println("║  3. 按 GPIO1 送出                      ║");
  Serial.println("║  4. 同按 GPIO1+GPIO2 重置姿態零點      ║");
  Serial.println("║  Serial: c=重新校準 r=重置微調         ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("\n[校準] 握好筆保持靜止，按住 GPIO2 三秒...");
}

// ============================================================
//  主迴圈
// ============================================================
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

  // ─── 雙鍵重置（IDLE 或 AIR 中才允許）────────────────────
  bool bothHeld = isDrawPressed && isSendPressed;
  if (bothHeld && !bothKeysWasHeld &&
      projFrozen &&
      (currentState==IDLE || currentState==RECORDING_AIR)) {
    // 用當前加速度重新收斂四元數，再重新凍結投影矩陣
    q0=1;q1=0;q2=0;q3=0;iFBx=iFBy=iFBz=0;
    for (int w=0;w<400;w++) mahonyUpdate(0,0,0,ax,ay,az,0.01f);
    // 用當前重力重新計算 bias（保留原有的因為還在同一握筆姿勢）
    freezeProjectionMatrix();
    Serial.println("🔄 雙鍵重置完成！投影方向已重新對齊");
  }
  bothKeysWasHeld = bothHeld;

  // ─── 狀態機 ──────────────────────────────────────────────
  switch (currentState) {

    // ── 等待開始校準 ─────────────────────────────────────
    case CALIB_WAIT:
      if (isDrawPressed && !isSendPressed) {
        calibStartMs=millis();
        sum_ax=sum_ay=sum_az=0; calibCount=0;
        currentState=CALIB_REC;
        Serial.println("⏳ 校準中（3秒），保持靜止...");
      }
      break;

    // ── 記錄靜止重力 ─────────────────────────────────────
    case CALIB_REC:
      if (!isDrawPressed) {
        currentState=CALIB_WAIT;
        Serial.println("❌ 中斷，請重試");
        break;
      }
      // 校準期間持續用 Mahony 收斂四元數
      mahonyUpdate(gx,gy,gz,ax,ay,az,dt);
      sum_ax+=ax; sum_ay+=ay; sum_az+=az; calibCount++;

      if (millis()-calibStartMs >= 3000) {
        bias_ax=sum_ax/calibCount;
        bias_ay=sum_ay/calibCount;
        bias_az=sum_az/calibCount;

        // 讓四元數對齊重力後凍結投影矩陣
        q0=1;q1=0;q2=0;q3=0;iFBx=iFBy=iFBz=0;
        for (int w=0;w<400;w++) mahonyUpdate(0,0,0,bias_ax,bias_ay,bias_az,0.01f);
        freezeProjectionMatrix();

        Serial.println("✅ 校準完成！凍結投影矩陣，方向固定。");
        Serial.printf("   bias: ax=%.3f ay=%.3f az=%.3f\n", bias_ax,bias_ay,bias_az);
        Serial.println("📝 按住 GPIO2 落筆，放開抬筆，GPIO1 送出");
        pointCount=0;
        currentState=IDLE;
      }
      break;

    // ── 就緒 ─────────────────────────────────────────────
    case IDLE:
      if (isDrawPressed && !isSendPressed) {
        pointCount=0;
        currentState=RECORDING_INK;
        Serial.println("🖊️  記錄中...");
      }
      break;

    // ── 落筆 / 抬筆 ──────────────────────────────────────
    case RECORDING_INK:
    case RECORDING_AIR:
      if (currentState==RECORDING_INK && !isDrawPressed)
        currentState=RECORDING_AIR;
      else if (currentState==RECORDING_AIR && isDrawPressed && !isSendPressed)
        currentState=RECORDING_INK;

      if (isSendTriggered && !isDrawPressed) {
        Serial.println("📤 處理中...");
        processFinalWordAndOutput();
        currentState=IDLE;
        break;
      }

      if (pointCount < MAX_POINTS) {
        float px, py;
        project(ax_net, ay_net, az_net, px, py);
        buf[pointCount].ax         = px;
        buf[pointCount].ay         = py;
        buf[pointCount].dt         = dt;
        buf[pointCount].stationary = confirmedStill;
        buf[pointCount].ink        = (currentState==RECORDING_INK);
        pointCount++;
      } else {
        Serial.println("⚠️  緩衝已滿，請送出");
      }
      break;
  }

  delay(10);
}
