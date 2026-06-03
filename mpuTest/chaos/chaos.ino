#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const int DELAY_MS = 10;
const int CALIB_SAMPLES = 200; // 2秒校準
const int TEST_TIME_MS = 3000; // 3秒的動態測試時間

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin(8, 9);
  
  if (!mpu.begin()) {
    Serial.println("❌ 找不到 MPU6050...");
    while (1) delay(10);
  }

  // 與你原本專案相同的設定
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  Serial.println("\n=======================================");
  Serial.println("🐾 [準備] 請將感測器平放靜置，開始校準環境味道...");
  delay(1000);
  
  float bias_ax = 0, bias_ay = 0, bias_az = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    bias_ax += a.acceleration.x;
    bias_ay += a.acceleration.y;
    bias_az += a.acceleration.z;
    delay(DELAY_MS);
  }
  bias_ax /= CALIB_SAMPLES;
  bias_ay /= CALIB_SAMPLES;
  bias_az /= CALIB_SAMPLES;

  Serial.println("✅ 校準完成！");
  Serial.println("⚠️ 注意！倒數 3 秒後，你有 3 秒鐘可以大動作揮舞！");
  Serial.println("⚠️ 任務：在 3 秒結束時，必須回到『現在的精準位置與角度』並靜止！");
  
  for(int i=3; i>0; i--) {
    Serial.printf("⏳ %d...\n", i);
    delay(1000);
  }

  Serial.println("💥 跑！！！ (請大動作揮舞並折返)");

  float vx = 0, vy = 0, vz = 0;
  float px = 0, py = 0, pz = 0;
  float dt = DELAY_MS / 1000.0f;
  
  unsigned long startTime = millis();
  
  // 記錄 3 秒鐘的動態軌跡
  while (millis() - startTime < TEST_TIME_MS) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 扣除靜態零偏
    float ax_net = a.acceleration.x - bias_ax;
    float ay_net = a.acceleration.y - bias_ay;
    float az_net = a.acceleration.z - bias_az;

    // 簡單尤拉積分
    vx += ax_net * dt;
    vy += ay_net * dt;
    vz += az_net * dt;

    px += vx * dt;
    py += vy * dt;
    pz += vz * dt;

    delay(DELAY_MS);
  }

  Serial.println("🛑 停！時間到！");
  
  // 顯示結果
  Serial.println("\n[ 🐾 動態折返跑 誤差報告 ]");
  Serial.println("如果你的手真的回到了原點，以下數字應該要接近 0。");
  Serial.printf("  X軸 動態殘留誤差: %.2f mm\n", px * 1000.0f);
  Serial.printf("  Y軸 動態殘留誤差: %.2f mm\n", py * 1000.0f);
  Serial.printf("  Z軸 動態殘留誤差: %.2f mm\n", pz * 1000.0f);
  
  float total_drift = sqrt(px*px + py*py + pz*pz) * 1000.0f;
  Serial.printf("  🎯 總計動態漂移量: %.2f mm\n", total_drift);

  if (total_drift > 20.0f) {
    Serial.println("  => ⚠️ 糟糕！誤差超過 2 公分！你的『手腕傾斜』讓重力嚴重干擾了軌跡！");
    Serial.println("  => 解法：寫字時手腕必須盡量『平移』，或是要在主程式加入姿態即時更新！");
  } else {
    Serial.println("  => ✅ 唔嗯～還算不錯！你的動作控制得很穩定喔。");
  }

  Serial.println("=======================================");
  delay(4000); // 休息一下再測下一次
}