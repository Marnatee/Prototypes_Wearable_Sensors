#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <BluetoothSerial.h>

#include <Arduino_GFX_Library.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <XPT2046_Touchscreen.h>

#include <math.h>

// =====================================================
// OPTIONAL WIFI CREDENTIALS
// Leave blank if you only want network scanning.
// =====================================================
const char *WIFI_SSID = "";
const char *WIFI_PASSWORD = "";

// =====================================================
// COLORS
// =====================================================
#define BLACK      0x0000
#define WHITE      0xFFFF
#define GREEN      0x07E0
#define RED        0xF800
#define BLUE       0x001F
#define CYAN       0x07FF
#define YELLOW     0xFFE0
#define MAGENTA    0xF81F
#define DARKGREY   0x7BEF
#define ORANGE     0xFD20

// =====================================================
// DISPLAY
// =====================================================
#define TFT_DC    2
#define TFT_CS    15
#define TFT_SCK   14
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_BL    21
#define TFT_RST   -1

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  TFT_DC,
  TFT_CS,
  TFT_SCK,
  TFT_MOSI,
  TFT_MISO
);

Arduino_GFX *gfx = new Arduino_ILI9341(
  bus,
  TFT_RST,
  1,
  false
);

// =====================================================
// TOUCH
// =====================================================
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define TOUCH_CLK  25
#define TOUCH_MOSI 32
#define TOUCH_MISO 39

SPIClass touchSPI(VSPI);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// Approximate calibration.
// We can tune these later if touch coordinates are off.
#define TOUCH_MIN_X 200
#define TOUCH_MAX_X 3800
#define TOUCH_MIN_Y 200
#define TOUCH_MAX_Y 3800

// =====================================================
// MPU-6050
// =====================================================
#define MPU_SDA 23
#define MPU_SCL 18

Adafruit_MPU6050 mpu;

// =====================================================
// BLUETOOTH
// =====================================================
BluetoothSerial SerialBT;

// =====================================================
// SAMPLING
// =====================================================
const uint32_t TARGET_HZ = 100;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / TARGET_HZ;

uint32_t lastSampleUs = 0;

uint32_t totalSamples = 0;
uint32_t lateEvents = 0;
uint32_t missedSamples = 0;
uint32_t worstLateUs = 0;

uint32_t samplesThisWindow = 0;
uint32_t hzWindowStartMs = 0;
float actualHz = 0.0f;

bool samplingPaused = false;

// Intentional WiFi pauses
uint32_t wifiScanCount = 0;
uint32_t lastScanDurationMs = 0;

// =====================================================
// SENSOR VALUES
// =====================================================
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float tempC = 0;

float accelMag = 0;
float gyroMag = 0;

// =====================================================
// ORIENTATION
// =====================================================
float rollDeg = 0;
float pitchDeg = 0;
float yawDeg = 0;

uint32_t lastOrientationUs = 0;

// =====================================================
// PAGES
// =====================================================
enum Page
{
  PAGE_SENSOR,
  PAGE_ORIENTATION,
  PAGE_WIRELESS
};

Page currentPage = PAGE_SENSOR;
Page previousPage = PAGE_WIRELESS;

// =====================================================
// UI TIMING
// =====================================================
uint32_t lastScreenUpdateMs = 0;

const uint32_t SENSOR_SCREEN_MS = 200;       // 5 Hz
const uint32_t ORIENTATION_SCREEN_MS = 50;   // 20 Hz
const uint32_t WIRELESS_SCREEN_MS = 500;     // 2 Hz

// =====================================================
// WIFI SCAN RESULTS
// =====================================================
int networkCount = 0;

String strongestSSID = "";
int strongestRSSI = -999;

String secondSSID = "";
int secondRSSI = -999;

String thirdSSID = "";
int thirdRSSI = -999;

// =====================================================
// TOUCH
// =====================================================
bool readTouch(int &x, int &y)
{
  if (!ts.touched())
    return false;

  TS_Point p = ts.getPoint();

  x = map(p.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, 320);
  y = map(p.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, 240);

  x = constrain(x, 0, 319);
  y = constrain(y, 0, 239);

  return true;
}

// =====================================================
// WIFI QUALITY
// =====================================================
const char *wifiQuality(int rssi)
{
  if (rssi >= -50) return "Excellent";
  if (rssi >= -60) return "Good";
  if (rssi >= -70) return "Fair";
  return "Weak";
}

// =====================================================
// COMMON UI
// =====================================================
void drawHeader()
{
  gfx->fillRect(0, 0, 320, 32, DARKGREY);

  gfx->setTextSize(1);

  gfx->setTextColor(
    currentPage == PAGE_SENSOR ? YELLOW : WHITE
  );
  gfx->setCursor(15, 12);
  gfx->print("SENSOR");

  gfx->setTextColor(
    currentPage == PAGE_ORIENTATION ? YELLOW : WHITE
  );
  gfx->setCursor(105, 12);
  gfx->print("ORIENTATION");

  gfx->setTextColor(
    currentPage == PAGE_WIRELESS ? YELLOW : WHITE
  );
  gfx->setCursor(245, 12);
  gfx->print("WIRELESS");
}

void drawFooter()
{
  gfx->fillRect(0, 221, 320, 19, DARKGREY);

  gfx->setTextSize(1);

  gfx->setTextColor(GREEN);
  gfx->setCursor(5, 228);
  gfx->print("IMU");

  gfx->setTextColor(
    WiFi.status() == WL_CONNECTED ? GREEN : ORANGE
  );
  gfx->setCursor(60, 228);
  gfx->print("WiFi");

  gfx->setTextColor(
    SerialBT.hasClient() ? GREEN : ORANGE
  );
  gfx->setCursor(125, 228);
  gfx->print("BT");

  gfx->setTextColor(WHITE);
  gfx->setCursor(195, 228);

  if (samplingPaused)
    gfx->print("PAUSED");
  else
    gfx->printf("%.1f Hz", actualHz);
}

// =====================================================
// PAGE STATIC DRAWING
// =====================================================
void drawSensorStatic()
{
  gfx->fillRect(0, 33, 320, 188, BLACK);

  gfx->setTextSize(1);

  gfx->setTextColor(CYAN);
  gfx->setCursor(8, 43);
  gfx->print("ACCELERATION (m/s^2)");

  gfx->setTextColor(MAGENTA);
  gfx->setCursor(8, 96);
  gfx->print("GYROSCOPE (deg/s)");

  gfx->setTextColor(YELLOW);
  gfx->setCursor(8, 151);
  gfx->print("SAMPLING HEALTH");
}

void drawOrientationStatic()
{
  gfx->fillRect(0, 33, 320, 188, BLACK);

  gfx->setTextSize(1);

  gfx->setTextColor(CYAN);
  gfx->setCursor(8, 42);
  gfx->print("LIVE IMU ORIENTATION");

  gfx->setTextColor(ORANGE);
  gfx->setCursor(190, 201);
  gfx->print("Yaw drifts");
}

void drawWirelessStatic()
{
  gfx->fillRect(0, 33, 320, 188, BLACK);

  gfx->setTextSize(1);

  gfx->setTextColor(CYAN);
  gfx->setCursor(8, 43);
  gfx->print("WIFI");

  gfx->setTextColor(MAGENTA);
  gfx->setCursor(8, 133);
  gfx->print("BLUETOOTH");

  // SCAN button
  gfx->fillRect(205, 40, 100, 28, BLUE);
  gfx->drawRect(205, 40, 100, 28, WHITE);

  gfx->setTextColor(WHITE);
  gfx->setCursor(236, 50);
  gfx->print("SCAN");
}

void drawCurrentPageStatic()
{
  drawHeader();

  switch (currentPage)
  {
    case PAGE_SENSOR:
      drawSensorStatic();
      break;

    case PAGE_ORIENTATION:
      drawOrientationStatic();
      break;

    case PAGE_WIRELESS:
      drawWirelessStatic();
      break;
  }

  drawFooter();

  previousPage = currentPage;
}

// =====================================================
// IMU SAMPLING
// =====================================================
void sampleIMU()
{
  if (samplingPaused)
    return;

  uint32_t nowUs = micros();
  uint32_t elapsed = nowUs - lastSampleUs;

  if (elapsed < SAMPLE_PERIOD_US)
    return;

  // Number of complete sampling periods since last read
  uint32_t periodsElapsed = elapsed / SAMPLE_PERIOD_US;

  if (periodsElapsed > 1)
  {
    missedSamples += periodsElapsed - 1;
  }

  uint32_t lateness =
    elapsed - SAMPLE_PERIOD_US;

  if (lateness > 1000)
  {
    lateEvents++;

    if (lateness > worstLateUs)
      worstLateUs = lateness;
  }

  // Move schedule forward by ALL elapsed periods.
  // This prevents counting the same stall repeatedly.
  lastSampleUs +=
    periodsElapsed * SAMPLE_PERIOD_US;

  sensors_event_t a, g, t;

  mpu.getEvent(&a, &g, &t);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x * 57.2957795f;
  gy = g.gyro.y * 57.2957795f;
  gz = g.gyro.z * 57.2957795f;

  tempC = t.temperature;

  accelMag =
    sqrtf(ax * ax + ay * ay + az * az);

  gyroMag =
    sqrtf(gx * gx + gy * gy + gz * gz);

  totalSamples++;
  samplesThisWindow++;

  // -----------------------------
  // Actual Hz calculation
  // -----------------------------
  uint32_t nowMs = millis();

  if (nowMs - hzWindowStartMs >= 1000)
  {
    uint32_t delta =
      nowMs - hzWindowStartMs;

    actualHz =
      samplesThisWindow * 1000.0f / delta;

    samplesThisWindow = 0;
    hzWindowStartMs = nowMs;
  }

  // -----------------------------
  // Orientation filter
  // -----------------------------
  float accelRoll =
    atan2f(ay, az) * 57.2957795f;

  float accelPitch =
    atan2f(
      -ax,
      sqrtf(ay * ay + az * az)
    ) * 57.2957795f;

  uint32_t orientationNow =
    micros();

  float dt =
    (orientationNow - lastOrientationUs)
    / 1000000.0f;

  lastOrientationUs =
    orientationNow;

  if (dt > 0.0f && dt < 0.1f)
  {
    rollDeg =
      0.98f * (rollDeg + gx * dt)
      + 0.02f * accelRoll;

    pitchDeg =
      0.98f * (pitchDeg + gy * dt)
      + 0.02f * accelPitch;

    // No magnetometer, so yaw is relative
    yawDeg += gz * dt;
  }
}

// =====================================================
// SENSOR PAGE DYNAMIC
// =====================================================
void updateSensorPage()
{
  // Only erase changing value regions
  gfx->fillRect(0, 55, 320, 38, BLACK);
  gfx->fillRect(0, 108, 320, 38, BLACK);
  gfx->fillRect(0, 163, 320, 56, BLACK);

  gfx->setTextSize(1);
  gfx->setTextColor(WHITE);

  // accel
  gfx->setCursor(8, 59);
  gfx->printf(
    "X:%7.2f  Y:%7.2f  Z:%7.2f",
    ax, ay, az
  );

  gfx->setCursor(8, 77);
  gfx->printf(
    "Magnitude: %7.2f",
    accelMag
  );

  // gyro
  gfx->setCursor(8, 112);
  gfx->printf(
    "X:%7.1f  Y:%7.1f  Z:%7.1f",
    gx, gy, gz
  );

  gfx->setCursor(8, 130);
  gfx->printf(
    "Magnitude: %7.1f",
    gyroMag
  );

  // health
  gfx->setCursor(8, 166);
  gfx->printf(
    "Hz: %.1f / %lu",
    actualHz,
    TARGET_HZ
  );

  gfx->setCursor(115, 166);
  gfx->printf(
    "Temp: %.1f C",
    tempC
  );

  gfx->setCursor(8, 182);
  gfx->printf(
    "Late: %lu",
    lateEvents
  );

  gfx->setCursor(105, 182);
  gfx->printf(
    "Missed: %lu",
    missedSamples
  );

  gfx->setCursor(8, 198);
  gfx->printf(
    "Worst late: %.2f ms",
    worstLateUs / 1000.0f
  );

  gfx->setCursor(175, 198);
  gfx->printf(
    "N:%lu",
    totalSamples
  );
}

// =====================================================
// ORIENTATION MATH
// =====================================================
void rotateVector(
  float x,
  float y,
  float z,
  float roll,
  float pitch,
  float yaw,
  float &ox,
  float &oy,
  float &oz)
{
  float r = roll * DEG_TO_RAD;
  float p = pitch * DEG_TO_RAD;
  float yw = yaw * DEG_TO_RAD;

  float cr = cosf(r);
  float sr = sinf(r);

  float cp = cosf(p);
  float sp = sinf(p);

  float cy = cosf(yw);
  float sy = sinf(yw);

  ox =
    cy * cp * x
    + (cy * sp * sr - sy * cr) * y
    + (cy * sp * cr + sy * sr) * z;

  oy =
    sy * cp * x
    + (sy * sp * sr + cy * cr) * y
    + (sy * sp * cr - cy * sr) * z;

  oz =
    -sp * x
    + cp * sr * y
    + cp * cr * z;
}

void drawArrow(
  int x1,
  int y1,
  int x2,
  int y2,
  uint16_t color)
{
  gfx->drawLine(
    x1, y1,
    x2, y2,
    color
  );

  float angle =
    atan2f(
      y2 - y1,
      x2 - x1
    );

  const float arrowAngle =
    2.6f;

  const int arrowLen =
    10;

  int hx1 =
    x2 + cosf(angle + arrowAngle)
    * arrowLen;

  int hy1 =
    y2 + sinf(angle + arrowAngle)
    * arrowLen;

  int hx2 =
    x2 + cosf(angle - arrowAngle)
    * arrowLen;

  int hy2 =
    y2 + sinf(angle - arrowAngle)
    * arrowLen;

  gfx->drawLine(
    x2, y2,
    hx1, hy1,
    color
  );

  gfx->drawLine(
    x2, y2,
    hx2, hy2,
    color
  );
}

// =====================================================
// ORIENTATION PAGE DYNAMIC
// =====================================================
void updateOrientationPage()
{
  // Only clear the visualization region,
  // not the whole screen.
  gfx->fillRect(
    45, 55,
    230, 115,
    BLACK
  );

  int cx = 160;
  int cy = 115;

  // reference axes
  gfx->drawLine(
    cx, cy,
    cx + 55, cy,
    DARKGREY
  );

  gfx->drawLine(
    cx, cy,
    cx - 35, cy + 35,
    DARKGREY
  );

  gfx->drawLine(
    cx, cy,
    cx, cy - 50,
    DARKGREY
  );

  gfx->setTextSize(1);

  gfx->setTextColor(RED);
  gfx->setCursor(cx + 58, cy - 3);
  gfx->print("X");

  gfx->setTextColor(GREEN);
  gfx->setCursor(cx - 45, cy + 38);
  gfx->print("Y");

  gfx->setTextColor(BLUE);
  gfx->setCursor(cx + 4, cy - 57);
  gfx->print("Z");

  // IMU's local +X direction
  float vx, vy, vz;

  rotateVector(
    1, 0, 0,
    rollDeg,
    pitchDeg,
    yawDeg,
    vx, vy, vz
  );

  // Isometric-like projection.
  // Better visually than the previous depth divide.
  float screenX =
    vx * 65.0f
    - vy * 35.0f;

  float screenY =
    -vz * 60.0f
    + vy * 30.0f;

  int arrowX =
    cx + (int)screenX;

  int arrowY =
    cy + (int)screenY;

  // central sphere
  gfx->fillCircle(
    cx, cy,
    9,
    WHITE
  );

  gfx->drawCircle(
    cx, cy,
    12,
    DARKGREY
  );

  // CURRENT ORIENTATION ARROW
  gfx->drawLine(
    cx, cy,
    arrowX, arrowY,
    RED
  );

  drawArrow(
    cx, cy,
    arrowX, arrowY,
    RED
  );

  // numeric area
  gfx->fillRect(
    0, 171,
    320, 45,
    BLACK
  );

  gfx->setTextColor(WHITE);

  gfx->setCursor(8, 176);
  gfx->printf(
    "Roll: %7.2f",
    rollDeg
  );

  gfx->setCursor(110, 176);
  gfx->printf(
    "Pitch: %7.2f",
    pitchDeg
  );

  gfx->setCursor(8, 194);
  gfx->printf(
    "Yaw: %7.2f",
    yawDeg
  );

  gfx->setTextColor(ORANGE);
  gfx->setCursor(150, 194);
  gfx->print(
    "relative / drift"
  );
}

// =====================================================
// WIFI SCAN
// =====================================================
void performWifiScan()
{
  samplingPaused = true;

  // Show scan state immediately
  gfx->fillRect(
    5, 72,
    310, 50,
    BLACK
  );

  gfx->setTextColor(YELLOW);
  gfx->setTextSize(1);
  gfx->setCursor(8, 80);
  gfx->print(
    "SCANNING WIFI..."
  );

  gfx->setCursor(8, 98);
  gfx->print(
    "IMU sampling intentionally paused"
  );

  drawFooter();

  uint32_t startMs =
    millis();

  networkCount =
    WiFi.scanNetworks(
      false,
      true
    );

  lastScanDurationMs =
    millis() - startMs;

  wifiScanCount++;

  strongestSSID = "";
  secondSSID = "";
  thirdSSID = "";

  strongestRSSI = -999;
  secondRSSI = -999;
  thirdRSSI = -999;

  for (int i = 0; i < networkCount; i++)
  {
    int rssi =
      WiFi.RSSI(i);

    String ssid =
      WiFi.SSID(i);

    if (rssi > strongestRSSI)
    {
      thirdRSSI =
        secondRSSI;

      thirdSSID =
        secondSSID;

      secondRSSI =
        strongestRSSI;

      secondSSID =
        strongestSSID;

      strongestRSSI =
        rssi;

      strongestSSID =
        ssid;
    }
    else if (rssi > secondRSSI)
    {
      thirdRSSI =
        secondRSSI;

      thirdSSID =
        secondSSID;

      secondRSSI =
        rssi;

      secondSSID =
        ssid;
    }
    else if (rssi > thirdRSSI)
    {
      thirdRSSI =
        rssi;

      thirdSSID =
        ssid;
    }
  }

  WiFi.scanDelete();

  // ---------------------------------------------------
  // Restart sensor timing AFTER scan.
  // Intentional scan time is NOT counted as missed.
  // ---------------------------------------------------
  lastSampleUs =
    micros();

  lastOrientationUs =
    micros();

  samplesThisWindow = 0;

  hzWindowStartMs =
    millis();

  samplingPaused =
    false;

  // redraw Wireless page cleanly
  drawWirelessStatic();
}

// =====================================================
// WIRELESS PAGE DYNAMIC
// =====================================================
void updateWirelessPage()
{
  // Do not erase SCAN button
  gfx->fillRect(
    0, 70,
    320, 60,
    BLACK
  );

  gfx->fillRect(
    0, 145,
    320, 74,
    BLACK
  );

  gfx->setTextSize(1);
  gfx->setTextColor(WHITE);

  // WiFi connection
  gfx->setCursor(8, 74);

  if (WiFi.status() == WL_CONNECTED)
  {
    gfx->printf(
      "Connected: %s",
      WiFi.SSID().c_str()
    );

    gfx->setCursor(8, 90);
    gfx->printf(
      "RSSI: %d dBm  %s",
      WiFi.RSSI(),
      wifiQuality(
        WiFi.RSSI()
      )
    );

    gfx->setCursor(8, 106);
    gfx->printf(
      "IP: %s",
      WiFi.localIP()
      .toString()
      .c_str()
    );
  }
  else
  {
    gfx->print(
      "Connected: NO"
    );

    gfx->setCursor(
      8, 90
    );

    gfx->printf(
      "Networks found: %d",
      networkCount
    );

    if (
      strongestSSID.length()
    )
    {
      gfx->setCursor(
        8, 106
      );

      gfx->printf(
        "Best: %s %d dBm",
        strongestSSID.c_str(),
        strongestRSSI
      );
    }
  }

  gfx->setCursor(
    8, 120
  );

  gfx->printf(
    "Last scan: %lu ms  scans:%lu",
    lastScanDurationMs,
    wifiScanCount
  );

  // Bluetooth
  gfx->setCursor(
    8, 149
  );

  gfx->printf(
    "Device: IMU-Logger"
  );

  gfx->setCursor(
    8, 165
  );

  gfx->printf(
    "Client: %s",
    SerialBT.hasClient()
      ? "CONNECTED"
      : "none"
  );

  gfx->setCursor(
    8, 181
  );

  gfx->printf(
    "RX waiting: %d bytes",
    SerialBT.available()
  );

  gfx->setCursor(
    8, 197
  );

  gfx->printf(
    "Free heap: %u bytes",
    ESP.getFreeHeap()
  );
}

// =====================================================
// TOUCH HANDLER
// =====================================================
void handleTouch()
{
  static uint32_t lastTouchMs = 0;

  if (
    millis() - lastTouchMs
    < 250
  )
    return;

  int x, y;

  if (!readTouch(x, y))
    return;

  lastTouchMs =
    millis();

  // Page tabs
  if (y < 40)
  {
    if (x < 95)
      currentPage =
        PAGE_SENSOR;

    else if (x < 225)
      currentPage =
        PAGE_ORIENTATION;

    else
      currentPage =
        PAGE_WIRELESS;

    if (
      currentPage
      != previousPage
    )
    {
      drawCurrentPageStatic();
    }

    return;
  }

  // WiFi SCAN button
  if (
    currentPage
    == PAGE_WIRELESS
  )
  {
    if (
      x >= 195 &&
      x <= 315 &&
      y >= 35 &&
      y <= 75
    )
    {
      performWifiScan();
      return;
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  pinMode(
    TFT_BL,
    OUTPUT
  );

  digitalWrite(
    TFT_BL,
    HIGH
  );

  gfx->begin();

  gfx->fillScreen(
    BLACK
  );

  // -------------------------------
  // TOUCH
  // -------------------------------
  touchSPI.begin(
    TOUCH_CLK,
    TOUCH_MISO,
    TOUCH_MOSI,
    TOUCH_CS
  );

  ts.begin(
    touchSPI
  );

  ts.setRotation(1);

  // -------------------------------
  // MPU
  // -------------------------------
  Wire.begin(
    MPU_SDA,
    MPU_SCL
  );

  Wire.setClock(
    400000
  );

  delay(150);

  if (
    !mpu.begin(
      0x68,
      &Wire
    )
  )
  {
    gfx->setTextColor(
      RED
    );

    gfx->setTextSize(
      2
    );

    gfx->setCursor(
      45,
      100
    );

    gfx->print(
      "MPU NOT FOUND"
    );

    while (true)
      delay(1000);
  }

  mpu.setAccelerometerRange(
    MPU6050_RANGE_4_G
  );

  mpu.setGyroRange(
    MPU6050_RANGE_500_DEG
  );

  mpu.setFilterBandwidth(
    MPU6050_BAND_44_HZ
  );

  // -------------------------------
  // BLUETOOTH
  // -------------------------------
  SerialBT.begin(
    "IMU-Logger"
  );

  // -------------------------------
  // WIFI
  // -------------------------------
  WiFi.mode(
    WIFI_STA
  );

  // Optional nonblocking connect
  if (
    strlen(WIFI_SSID) > 0
  )
  {
    WiFi.begin(
      WIFI_SSID,
      WIFI_PASSWORD
    );
  }

  // -------------------------------
  // TIMERS
  // -------------------------------
  lastSampleUs =
    micros();

  lastOrientationUs =
    micros();

  hzWindowStartMs =
    millis();

  drawCurrentPageStatic();
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  // Highest priority
  sampleIMU();

  handleTouch();

  uint32_t nowMs =
    millis();

  uint32_t interval;

  switch (currentPage)
  {
    case PAGE_ORIENTATION:
      interval =
        ORIENTATION_SCREEN_MS;
      break;

    case PAGE_WIRELESS:
      interval =
        WIRELESS_SCREEN_MS;
      break;

    default:
      interval =
        SENSOR_SCREEN_MS;
      break;
  }

  if (
    nowMs - lastScreenUpdateMs
    >= interval
  )
  {
    lastScreenUpdateMs =
      nowMs;

    switch (currentPage)
    {
      case PAGE_SENSOR:
        updateSensorPage();
        break;

      case PAGE_ORIENTATION:
        updateOrientationPage();
        break;

      case PAGE_WIRELESS:
        updateWirelessPage();
        break;
    }

    drawFooter();
  }
}