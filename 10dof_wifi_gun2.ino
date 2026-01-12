#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <ILI9488.h>
#include "ICM_20948.h"
#include <WiFiS3.h>

// ================== WIFI ==================
const char* WIFI_SSID = "JetsonAP";
const char* WIFI_PASS = "12345678";
constexpr uint16_t SERVER_PORT = 3333;

WiFiServer server(SERVER_PORT);
WiFiClient client;

// ================== DISPLAY ==================
constexpr uint8_t TFT_CS   = 10;
constexpr uint8_t TFT_DC   = 9;
constexpr uint8_t TFT_RST  = 8;
constexpr uint8_t TFT_LED  = A0;

constexpr uint8_t TFT_ROTATION = 1;   // 480x320 landscape
constexpr int16_t SCREEN_W = 480;
constexpr int16_t SCREEN_H = 320;
constexpr int16_t CX = SCREEN_W / 2;
constexpr int16_t CY = SCREEN_H / 2;

ILI9488 tft(TFT_CS, TFT_DC, TFT_RST);

// ================== UI ==================
constexpr float   PX_PER_DEG = 3.0f;   // px per degree
constexpr int16_t BOX_SIZE   = 50;
constexpr int16_t ANGLE_STEP_DEG = 2;
constexpr int16_t TARGET_TOL_DEG = 6;

constexpr uint8_t TEXT_SIZE = 1;

// HUD positions
constexpr int16_t TXT_X_LABEL = 10;
constexpr int16_t TXT_X_VAL   = 80;
constexpr int16_t TXT_Y_ROLL  = 10;
constexpr int16_t TXT_Y_PITCH = 28;
constexpr int16_t TXT_Y_YAW   = 46;
constexpr int16_t TXT_Y_MSG   = 64;
constexpr int16_t TXT_Y_IP    = 82;

// HUD auto refresh
constexpr uint32_t HUD_REFRESH_MS = 10000;

// edge marker visible thickness (px)
constexpr int16_t EDGE_VISIBLE_PX = 5;

// Values on screen
int16_t lastDispRoll  = 32767;
int16_t lastDispPitch = 32767;
int16_t lastDispYaw   = 32767;

char lastMsg[48] = "-";
char lastIP[20]  = "0.0.0.0";

uint32_t lastHudRefreshMs = 0;

// ================== SPAWN (from Jetson) ==================
int16_t spawnPitchQ = 0;
int16_t spawnYawQ   = 0;
bool    spawnSet    = false;

// ================== IMU ==================
ICM_20948_I2C imu;

// zero calibration
bool  zeroSet    = false;
float zeroRoll   = 0.0f;
float zeroPitch  = 0.0f;
float zeroYaw    = 0.0f;

// Anti-drift yaw
float lastYaw          = 0.0f;
unsigned long lastMove = 0;
const float yawDriftLimitDeg   = 0.05f;
const unsigned long yawHoldTime = 250;

// Freeze yaw near gimbal lock
float yawStable      = 0.0f;
bool  yawStableInit  = false;
const float pitchLockThreshold = 80.0f;

// ================== helpers ==================
static inline int16_t quantizeDeg2(float a) {
  float q = (float)ANGLE_STEP_DEG * roundf(a / (float)ANGLE_STEP_DEG);
  return (int16_t)q;
}

// ---------- safe clip helpers ----------
static inline bool clipRect(int32_t x, int32_t y, int32_t w, int32_t h,
                            int16_t &ox, int16_t &oy, int16_t &ow, int16_t &oh) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }

  if (x + w > SCREEN_W) w = SCREEN_W - x;
  if (y + h > SCREEN_H) h = SCREEN_H - y;

  if (w <= 0 || h <= 0) return false;

  ox = (int16_t)x; oy = (int16_t)y; ow = (int16_t)w; oh = (int16_t)h;
  return true;
}

// ================== Cross / HUD ==================
void drawCrossFull() {
  tft.drawLine(CX, 0,      CX, SCREEN_H, ILI9488_DARKGREY);
  tft.drawLine(0,  CY, SCREEN_W, CY,     ILI9488_DARKGREY);
}

// крест только внутри прямоугольника (поверх всего)
void drawCrossInRect(int16_t x, int16_t y, int16_t w, int16_t h) {
  // vertical x=CX
  if (CX >= x && CX < (x + w)) {
    int16_t y0 = y;
    int16_t y1 = y + h - 1;
    if (y0 < 0) y0 = 0;
    if (y1 > SCREEN_H - 1) y1 = SCREEN_H - 1;
    tft.drawLine(CX, y0, CX, y1, ILI9488_DARKGREY);
  }
  // horizontal y=CY
  if (CY >= y && CY < (y + h)) {
    int16_t x0 = x;
    int16_t x1 = x + w - 1;
    if (x0 < 0) x0 = 0;
    if (x1 > SCREEN_W - 1) x1 = SCREEN_W - 1;
    tft.drawLine(x0, CY, x1, CY, ILI9488_DARKGREY);
  }
}

void drawStaticTextLabels() {
  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);

  tft.setCursor(TXT_X_LABEL, TXT_Y_ROLL);  tft.print(F("Roll ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_PITCH); tft.print(F("Pitch ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_YAW);   tft.print(F("Yaw ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_MSG);   tft.print(F("Msg ="));
  tft.setCursor(TXT_X_LABEL, TXT_Y_IP);    tft.print(F("IP ="));
}

void drawValueIfChanged(int16_t x, int16_t y, int16_t v, int16_t &lastV) {
  if (v == lastV) return;

  char buf[12];
  snprintf(buf, sizeof(buf), "%+5d", (int)v);

  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);

  lastV = v;
}

void drawStringIfChanged(int16_t x, int16_t y, const char* s, char* last, size_t lastSz, int padWidth) {
  if (strncmp(s, last, lastSz) == 0) return;

  char buf[80];
  snprintf(buf, sizeof(buf), "%-*s", padWidth, s);

  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
  tft.setCursor(x, y);
  tft.print(buf);

  strncpy(last, s, lastSz - 1);
  last[lastSz - 1] = '\0';
}

// Полный рефреш HUD (раз в 10 сек)
void refreshHUDForce(int16_t rollQ, int16_t pitchQ, int16_t yawQ) {
  // Перепечатываем всё. Текст с bg=BLACK сам "очищает" область.
  drawStaticTextLabels();

  char buf[16];
  tft.setTextSize(TEXT_SIZE);
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);

  snprintf(buf, sizeof(buf), "%+5d", (int)rollQ);
  tft.setCursor(TXT_X_VAL, TXT_Y_ROLL);  tft.print(buf);

  snprintf(buf, sizeof(buf), "%+5d", (int)pitchQ);
  tft.setCursor(TXT_X_VAL, TXT_Y_PITCH); tft.print(buf);

  snprintf(buf, sizeof(buf), "%+5d", (int)yawQ);
  tft.setCursor(TXT_X_VAL, TXT_Y_YAW);   tft.print(buf);

  char m[80];  snprintf(m, sizeof(m),  "%-30s", lastMsg);
  tft.setCursor(TXT_X_VAL, TXT_Y_MSG); tft.print(m);

  char ip[80]; snprintf(ip, sizeof(ip), "%-18s", lastIP);
  tft.setCursor(TXT_X_VAL, TXT_Y_IP);  tft.print(ip);

  // IMPORTANT: обновляем lastDisp*, чтобы drawValueIfChanged работал корректно
  lastDispRoll  = rollQ;
  lastDispPitch = pitchQ;
  lastDispYaw   = yawQ;

  lastHudRefreshMs = millis();
}

// ================== IMU init/read ==================
bool startIMU() {
  imu.begin(Wire, 0);
  if (imu.status != ICM_20948_Stat_Ok) return false;

  if (imu.initializeDMP() != ICM_20948_Stat_Ok) return false;
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) return false;

  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) != ICM_20948_Stat_Ok) return false;
  if (imu.enableFIFO() != ICM_20948_Stat_Ok) return false;
  if (imu.enableDMP()  != ICM_20948_Stat_Ok) return false;
  if (imu.resetDMP()   != ICM_20948_Stat_Ok) return false;
  if (imu.resetFIFO()  != ICM_20948_Stat_Ok) return false;

  return true;
}

bool readAnglesOnce(float &outRoll, float &outPitch, float &outYaw) {
  icm_20948_DMP_data_t d;
  ICM_20948_Status_e s = imu.readDMPdataFromFIFO(&d);

  if (s == ICM_20948_Stat_FIFONoDataAvail) return false;

  if (s != ICM_20948_Stat_Ok && s != ICM_20948_Stat_FIFOMoreDataAvail) {
    imu.resetFIFO();
    imu.resetDMP();
    return false;
  }

  if (!(d.header & DMP_header_bitmap_Quat6)) return false;

  float q1 = (float)d.Quat6.Data.Q1 / 1073741824.0f;
  float q2 = (float)d.Quat6.Data.Q2 / 1073741824.0f;
  float q3 = (float)d.Quat6.Data.Q3 / 1073741824.0f;

  float sum = q1*q1 + q2*q2 + q3*q3;
  if (sum > 1.0f) sum = 1.0f;
  if (sum < 0.0f) sum = 0.0f;
  float q0 = sqrtf(1.0f - sum);

  float roll  = atan2f(2.0f * (q0*q1 + q2*q3),
                       1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / PI;

  float pitch = asinf (2.0f * (q0*q2 - q1*q3)) * 180.0f / PI;

  float yaw   = atan2f(2.0f * (q0*q3 + q1*q2),
                       1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / PI;

  if (!zeroSet) {
    zeroRoll  = roll;
    zeroPitch = pitch;
    zeroYaw   = yaw;
    zeroSet   = true;
  }

  roll  -= zeroRoll;
  pitch -= zeroPitch;
  yaw   -= zeroYaw;

  if (fabs(yaw - lastYaw) > yawDriftLimitDeg) {
    lastMove = millis();
  } else {
    if (millis() - lastMove > yawHoldTime) yaw = lastYaw;
  }
  lastYaw = yaw;

  if (!yawStableInit) {
    yawStable = yaw;
    yawStableInit = true;
  }
  if (fabs(pitch) < pitchLockThreshold) yawStable = yaw;

  outRoll  = roll;
  outPitch = pitch;
  outYaw   = yawStable;
  return true;
}

// ================== NET parsing ==================
static String readLine(WiFiClient& c) {
  String line;
  while (c.available()) {
    char ch = (char)c.read();
    if (ch == '\r') continue;
    if (ch == '\n') break;
    line += ch;
    if (line.length() > 256) break;
  }
  return line;
}

// X/Y only digits (no sign) as in your Tkinter
static bool parsePacket(const String& s, String& msg, long& x, long& y) {
  int iMsg = s.indexOf("MSG:");
  int iX   = s.indexOf(";X:");
  int iY   = s.indexOf(";Y:");
  if (iMsg != 0 || iX < 0 || iY < 0) return false;

  msg = s.substring(4, iX);
  String sx = s.substring(iX + 3, iY);
  String sy = s.substring(iY + 3);

  sx.trim(); sy.trim();
  for (size_t i = 0; i < sx.length(); i++) if (!isDigit(sx[i])) return false;
  for (size_t i = 0; i < sy.length(); i++) if (!isDigit(sy[i])) return false;

  x = sx.toInt();
  y = sy.toInt();
  return true;
}

void wifiConnectAndStartServer() {
  Serial.print("Connecting WiFi to: ");
  Serial.println(WIFI_SSID);

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(800);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected.");

  IPAddress ip = WiFi.localIP();
  unsigned long t0 = millis();
  while (ip[0] == 0 && (millis() - t0) < 15000) {
    delay(250);
    ip = WiFi.localIP();
  }

  snprintf(lastIP, sizeof(lastIP), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

  server.begin();
  Serial.print("TCP server started on port ");
  Serial.println(SERVER_PORT);
}

// ================== BOX (single, safe, edge visible) ==================
bool     lastBoxValid = false;
int16_t  lastBoxX = 0, lastBoxY = 0, lastBoxW = 0, lastBoxH = 0;
uint16_t lastBoxColor = 0xFFFF;

void eraseOldBox() {
  if (!lastBoxValid) return;
  tft.fillRect(lastBoxX, lastBoxY, lastBoxW, lastBoxH, ILI9488_BLACK);
  drawCrossInRect(lastBoxX, lastBoxY, lastBoxW, lastBoxH);
  lastBoxValid = false;
}

void drawNewBox(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  tft.fillRect(x, y, w, h, color);
  // крест поверх квадрата
  drawCrossInRect(x, y, w, h);

  lastBoxValid = true;
  lastBoxX = x; lastBoxY = y; lastBoxW = w; lastBoxH = h;
  lastBoxColor = color;
}

void updateBox(int16_t pitchRelQ, int16_t yawRelQ, bool onTarget) {
  int32_t centerX = (int32_t)lroundf((float)CX + (float)yawRelQ * PX_PER_DEG);
  int32_t centerY = (int32_t)lroundf((float)CY + (float)pitchRelQ * PX_PER_DEG);

  int32_t boxX = centerX - BOX_SIZE / 2;
  int32_t boxY = centerY - BOX_SIZE / 2;

  // ---- edge-visible behavior: keep at least EDGE_VISIBLE_PX on screen ----
  // Right: show only EDGE_VISIBLE_PX pixels -> x = SCREEN_W - EDGE_VISIBLE_PX
  if (boxX > (int32_t)SCREEN_W - EDGE_VISIBLE_PX) boxX = (int32_t)SCREEN_W - EDGE_VISIBLE_PX;
  // Left: show only EDGE_VISIBLE_PX pixels -> x = -BOX_SIZE + EDGE_VISIBLE_PX
  if (boxX < (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX)) boxX = (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX);

  // Bottom
  if (boxY > (int32_t)SCREEN_H - EDGE_VISIBLE_PX) boxY = (int32_t)SCREEN_H - EDGE_VISIBLE_PX;
  // Top
  if (boxY < (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX)) boxY = (int32_t)(-BOX_SIZE + EDGE_VISIBLE_PX);

  int16_t nx, ny, nw, nh;
  bool ok = clipRect(boxX, boxY, BOX_SIZE, BOX_SIZE, nx, ny, nw, nh);

  if (!ok) {
    // по идее не должно случаться из-за edge-clamp, но оставим
    eraseOldBox();
    return;
  }

  uint16_t color = onTarget ? ILI9488_GREEN : ILI9488_RED;

  // тот же прямоугольник?
  if (lastBoxValid &&
      nx == lastBoxX && ny == lastBoxY && nw == lastBoxW && nh == lastBoxH) {

    // только цвет поменялся
    if (color != lastBoxColor) {
      tft.fillRect(nx, ny, nw, nh, color);
      drawCrossInRect(nx, ny, nw, nh);
      lastBoxColor = color;
    } else {
      // подстрахуем крест поверх
      drawCrossInRect(nx, ny, nw, nh);
    }
    return;
  }

  // новый прямоугольник -> стереть старый и нарисовать новый
  eraseOldBox();
  drawNewBox(nx, ny, nw, nh, color);
}

// ================== Arduino ==================
void setup() {
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(TFT_ROTATION);

  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  tft.fillScreen(ILI9488_BLACK);
  drawCrossFull();
  drawStaticTextLabels();

  Wire.begin();
  Wire.setClock(400000);

  tft.setTextSize(2);
  tft.setTextColor(ILI9488_YELLOW, ILI9488_BLACK);
  tft.setCursor(10, 110);
  tft.print(F("Init IMU..."));

  while (!startIMU()) {
    tft.setTextColor(ILI9488_RED, ILI9488_BLACK);
    tft.setCursor(10, 110);
    tft.print(F("IMU init fail, retry... "));
    delay(200);
  }

  tft.setTextColor(ILI9488_GREEN, ILI9488_BLACK);
  tft.setCursor(10, 110);
  tft.print(F("IMU OK                "));
  delay(150);

  tft.setTextColor(ILI9488_BLACK, ILI9488_BLACK);
  tft.setCursor(10, 110);
  tft.print(F("                      "));

  wifiConnectAndStartServer();

  // первичный вывод IP/Msg
  tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
  tft.setTextSize(TEXT_SIZE);
  tft.setCursor(TXT_X_VAL, TXT_Y_IP);  tft.print(lastIP);
  tft.setCursor(TXT_X_VAL, TXT_Y_MSG); tft.print(lastMsg);

  lastHudRefreshMs = millis();
}

void loop() {
  float roll = 0, pitch = 0, yaw = 0;
  bool got = false;

  uint32_t t0 = micros();
  while (true) {
    float r, p, y;
    if (!readAnglesOnce(r, p, y)) break;
    roll = r; pitch = p; yaw = y;
    got = true;
    if (micros() - t0 > 2500) break;
  }
  if (!got) return;

  // swap roll/pitch
  float tmp = roll; roll = pitch; pitch = tmp;

  int16_t rQ = quantizeDeg2(roll);
  int16_t pQ = quantizeDeg2(pitch);
  int16_t yQ = quantizeDeg2(yaw);

  // обновляем числа (быстро)
  drawValueIfChanged(TXT_X_VAL, TXT_Y_ROLL,  rQ, lastDispRoll);
  drawValueIfChanged(TXT_X_VAL, TXT_Y_PITCH, pQ, lastDispPitch);
  drawValueIfChanged(TXT_X_VAL, TXT_Y_YAW,   yQ, lastDispYaw);

  // ===== NET accept =====
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      client.setTimeout(5);
      client.println("HELLO from UNO R4 WiFi");
      Serial.println("Client connected.");
    }
  }

  // ===== NET read =====
  if (client && client.connected() && client.available()) {
    String line = readLine(client);
    line.trim();
    if (line.length() > 0) {
      String msg;
      long x = 0, y = 0;

      if (parsePacket(line, msg, x, y)) {
        // spawn (deg, step 2)
        spawnPitchQ = quantizeDeg2((float)x);
        spawnYawQ   = quantizeDeg2((float)y);
        spawnSet    = true;

        // Msg на HUD (может быть затёрт квадратом — ок, раз в 10 сек восстановим)
        char msgBuf[48];
        msg.toCharArray(msgBuf, sizeof(msgBuf));
        msgBuf[30] = '\0';
        drawStringIfChanged(TXT_X_VAL, TXT_Y_MSG, msgBuf, lastMsg, sizeof(lastMsg), 30);

        // ACK
        client.print("ACK;MSG:");
        client.print(msg);
        client.print(";X:");
        client.print(x);
        client.print(";Y:");
        client.println(y);
      } else {
        client.print("ERR;BAD_PACKET;");
        client.println(line);
      }
    }
  }

  // ===== BOX draw =====
  int16_t pitchRelQ = pQ - (spawnSet ? spawnPitchQ : 0);
  int16_t yawRelQ   = yQ - (spawnSet ? spawnYawQ   : 0);

  bool onTarget = (abs((int)pitchRelQ) <= TARGET_TOL_DEG) &&
                  (abs((int)yawRelQ)   <= TARGET_TOL_DEG);

  updateBox(pitchRelQ, yawRelQ, onTarget);

  // ===== HUD refresh every 10s =====
  if (millis() - lastHudRefreshMs >= HUD_REFRESH_MS) {
    refreshHUDForce(rQ, pQ, yQ);
  }
}
