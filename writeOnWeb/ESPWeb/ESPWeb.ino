#include <WiFi.h>
#include <WebServer.h>

HardwareSerial DueSerial(1);

// --- WiFi 基地台設定 ---
const char* ssid = "ESP32_Canvas_Magic"; // 主人可以在手機 WiFi 裡找這個名字！
const char* password = "";               // 留空代表不需要密碼，直接連線

WebServer server(80);

// --- 28x28 網格與軌跡快取記憶體 ---
const int MAX_POINTS = 800;
float pathX[MAX_POINTS];
float pathY[MAX_POINTS];
int pointCount = 0;

uint8_t grid[28][28];

// 🎨 網頁原始碼 (HTML + CSS + JavaScript)
// 這裡用 R"rawliteral(...)rawliteral" 來包裝，這樣就能直接把網頁寫在 C++ 裡囉！
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=0">
  <title>獸性少女的魔法畫布</title>
  <style>
    body { text-align: center; font-family: sans-serif; background-color: #2c3e50; color: white; margin: 0; padding: 10px; }
    h2 { margin-top: 10px; margin-bottom: 10px; }
    canvas { 
      border: 3px solid #ecf0f1; 
      background-color: #ffffff; 
      touch-action: none; /* 阻止手機滑動畫面的預設行為 */
      border-radius: 12px; 
      box-shadow: 0px 4px 10px rgba(0,0,0,0.5);
    }
    .btn-container { margin-top: 20px; }
    .btn { 
      padding: 15px 30px; font-size: 20px; margin: 0 10px; 
      border-radius: 10px; border: none; font-weight: bold; cursor: pointer;
    }
    .btn-clear { background-color: #95a5a6; color: white; }
    .btn-send { background-color: #e74c3c; color: white; }
  </style>
</head>
<body>
  <h2>🐾 魔法觸控畫布 🐾</h2>
  <canvas id="c"></canvas>
  <div class="btn-container">
    <button class="btn btn-clear" onclick="clearCanvas()">重寫</button>
    <button class="btn btn-send" onclick="sendData()">發送字跡</button>
  </div>

  <script>
    var canvas = document.getElementById('c');
    var ctx = canvas.getContext('2d');
    var drawing = false;
    var points = [];

    // 讓畫布自動適應手機螢幕寬度
    var size = Math.min(window.innerWidth - 40, 400);
    canvas.width = size;
    canvas.height = size;
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.strokeStyle = '#2c3e50';

    function getPos(e) {
      var rect = canvas.getBoundingClientRect();
      var x = e.clientX || (e.touches && e.touches[0].clientX);
      var y = e.clientY || (e.touches && e.touches[0].clientY);
      return { x: x - rect.left, y: y - rect.top };
    }

    function start(e) { 
      e.preventDefault(); 
      drawing = true; 
      addPoint(e); 
      var p = getPos(e);
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
    }
    
    function move(e) {
      e.preventDefault();
      if(!drawing) return;
      var p = getPos(e);
      ctx.lineTo(p.x, p.y);
      ctx.stroke();
      addPoint(e);
    }
    
    function end(e) { 
      e.preventDefault(); 
      if(drawing) {
        drawing = false; 
        points.push('-1,-1'); // 插入斷筆標記，讓 C++ 知道手抬起來了喵！
      }
    }
    
    function addPoint(e) {
      var p = getPos(e);
      points.push(Math.round(p.x) + ',' + Math.round(p.y));
    }

    // 綁定滑鼠與觸控事件
    canvas.addEventListener('mousedown', start);
    canvas.addEventListener('mousemove', move);
    canvas.addEventListener('mouseup', end);
    canvas.addEventListener('touchstart', start, {passive: false});
    canvas.addEventListener('touchmove', move, {passive: false});
    canvas.addEventListener('touchend', end);

    function clearCanvas() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      points = [];
    }

    function sendData() {
      if(points.length < 5) {
        alert("喵唔？點太少啦，多畫一點吧！");
        return;
      }
      var payload = points.join(';');
      fetch('/upload', { 
        method: 'POST', 
        body: payload 
      })
      .then(response => { 
        alert('✨ 發送成功！快看序列埠！'); 
        clearCanvas(); 
      })
      .catch(error => { 
        alert('發送失敗，檢查連線喵...'); 
      });
    }
  </script>
</body>
</html>
)rawliteral";

// 布雷森漢姆直線演算法
void drawLine(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;
  while (true) {
    if (x0 >= 0 && x0 < 28 && y0 >= 0 && y0 < 28) grid[y0][x0] = 0;
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// 將收集到的座標轉換為 28x28 影像
void processGrid() {
  for (int y = 0; y < 28; y++) {
    for (int x = 0; x < 28; x++) grid[y][x] = 255;
  }

  // 1. 尋找 Bounding Box (跳過 -1,-1 斷筆標記)
  float minX = 9999, maxX = -9999, minY = 9999, maxY = -9999;
  int validPoints = 0;
  for (int i = 0; i < pointCount; i++) {
    if (pathX[i] == -1 && pathY[i] == -1) continue; // 忽略斷筆標記
    if (pathX[i] < minX) minX = pathX[i];
    if (pathX[i] > maxX) maxX = pathX[i];
    if (pathY[i] < minY) minY = pathY[i];
    if (pathY[i] > maxY) maxY = pathY[i];
    validPoints++;
  }

  if (validPoints < 2) return;

  // 2. 等比例縮放居中 (縮至 20x20 核心區)
  float w = maxX - minX; if (w < 1) w = 1;
  float h = maxY - minY; if (h < 1) h = 1;
  float scale = (20.0 / w < 20.0 / h) ? 20.0 / w : 20.0 / h;

  float avgX = (minX + maxX) / 2.0;
  float avgY = (minY + maxY) / 2.0;

  // 3. 網格化與連線處理 (加入斷筆邏輯)
  int lastGX = -1, lastGY = -1;
  bool isLifted = true; // 追蹤筆是否抬起

  for (int i = 0; i < pointCount; i++) {
    if (pathX[i] == -1 && pathY[i] == -1) {
      isLifted = true; // 偵測到斷筆，下一個點不要連線
      continue;
    }

    int gx = round(14.0 + (pathX[i] - avgX) * scale);
    int gy = round(14.0 + (pathY[i] - avgY) * scale);
    if (gx < 0) gx = 0; if (gx > 27) gx = 27;
    if (gy < 0) gy = 0; if (gy > 27) gy = 27;

    if (!isLifted) {
      drawLine(lastGX, lastGY, gx, gy);
    } else {
      grid[gy][gx] = 0; // 新筆畫的第一個點
      isLifted = false;
    }
    lastGX = gx; lastGY = gy;
  }

  // 4. 輸出預覽與 PBM
  Serial.println("\n--- 📱 Web 畫布：28x28 完美字跡預覽 ---");
  for (int y = 0; y < 28; y++) {
    for (int x = 0; x < 28; x++) Serial.print(grid[y][x] == 0 ? "██" : "  ");
    Serial.println();
  }
  
  Serial.println("\n--- 💾 PBM 圖片文字 ---");
  Serial.println("P1\n28 28");
  for (int y = 0; y < 28; y++) {
    for (int x = 0; x < 28; x++) Serial.print(grid[y][x] == 0 ? "1 " : "0 ");
    Serial.println();
  }
  Serial.println("--------------------------------------------------\n");
}

// 處理前端發過來的 POST 座標資料
void handleUpload() {
  String payload = server.arg("plain");
  pointCount = 0;
  
  // 解析用分號隔離的資料： "x1,y1;x2,y2;-1,-1;x3,y3..."
  int startIdx = 0;
  while (startIdx < payload.length() && pointCount < MAX_POINTS) {
    int sepIdx = payload.indexOf(';', startIdx);
    if (sepIdx == -1) sepIdx = payload.length();
    
    String pairStr = payload.substring(startIdx, sepIdx);
    int commaIdx = pairStr.indexOf(',');
    if (commaIdx != -1) {
      pathX[pointCount] = pairStr.substring(0, commaIdx).toFloat();
      pathY[pointCount] = pairStr.substring(commaIdx + 1).toFloat();
      pointCount++;
    }
    startIdx = sepIdx + 1;
  }

  server.send(200, "text/plain", "OK");
  Serial.println("📥 [收到手機軌跡資料] 正在處理...");
  processGrid();
  
  Serial.println("--------------------------------------------------\n");

  // 🚀 開始將資料高速發送給 Due！
  Serial.println("⚡ 正在用 500000 bps 狂飆傳送給 Due...");
  
  // 傳送「封包頭」，讓 Due 知道資料要來了 (用 0xAA 和 0xBB 當作開門密碼)
  DueSerial.write(0xAA);
  DueSerial.write(0xBB);
  
  // 依序傳送 28x28 = 784 個 Byte
  for (int y = 0; y < 28; y++) {
    for (int x = 0; x < 28; x++) {
      DueSerial.write(grid[y][x]);
    }
  }
}

void setup() {
  Serial.begin(115200);

  DueSerial.begin(500000, SERIAL_8N1, 0, 1);
  
  // 啟動 WiFi 基地台模式
  Serial.println("🐾 正在張開魔法陣 (WiFi AP)...");
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("✅ 基地台啟動成功！請用手機連線至 WiFi: ");
  Serial.println(ssid);
  Serial.print("🌐 然後打開瀏覽器，輸入網址: http://");
  Serial.println(IP);

  // 設定伺服器路由
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });
  
  server.on("/upload", HTTP_POST, handleUpload);

  server.begin();
  Serial.println("🚀 Web 伺服器待命喵！");
}

void loop() {
  server.handleClient(); // 不斷監聽手機傳來的網頁請求
  delay(2); 
}