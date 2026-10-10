// M5StickT thermal camera (M5StickC + FLIR Lepton 3.x, 160x120).
// Shows the live image and battery level on the LCD, and serves a web viewer
// on the Wi-Fi network configured in secrets.ini (DHCP or a static address).

// The web server headers must come before M5StickT.h, which defines a min()
// macro that breaks <chrono>.
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <M5StickT.h>
#include <SPI.h>
#include <esp32/rom/crc.h>
#include <ping/ping_sock.h>

static_assert(sizeof(WIFI_SSID) > 1, "Set wifi_ssid in secrets.ini");
static_assert(sizeof(WIFI_SSID) - 1 <= 32, "wifi_ssid must be at most 32 characters");
static_assert(sizeof(WIFI_PASSWORD) - 1 <= 63, "wifi_password must be at most 63 characters");

// ---- Hardware (from M5Stack's M5-StickT example) ---------------------------

constexpr int kLeptonCsPin = 0;
constexpr int kLeptonVsyncPin = 38;
constexpr int kLeptonResetPin = 26;
constexpr int kLeptonSckPin = 2;
constexpr int kLeptonMisoPin = 25;
// The Lepton has no MOSI. (Arduino core 2.x substituted GPIO13, the LCD's
// clock, for -1; core 3.x leaves it unattached.)
constexpr int kLeptonMosiPin = -1;
constexpr uint32_t kLeptonSpiHz = 20000000;
constexpr uint8_t kLeptonI2cAddr = 0x2A;  // CCI, on Wire1 with the AXP192

constexpr int kFrameW = 160;
constexpr int kFrameH = 120;
constexpr int kFramePixels = kFrameW * kFrameH;

// ---- Display layout (landscape 240x135) ------------------------------------

constexpr uint8_t kScreenRotation = 1;
constexpr int kScreenW = 240;
constexpr int kScreenH = 135;
constexpr int kImgX = 4;
constexpr int kImgY = (kScreenH - kFrameH) / 2;

// ---- Shared state ---------------------------------------------------------

// Wire1 is shared by the Lepton CCI (sensor task) and the AXP192 (loop).
static SemaphoreHandle_t wire1Mutex;

// Double buffer: the sensor task fills `captureBuf`; a finished frame is
// swapped into `readyBuf` unless loop() is still using it, in which case the
// frame is dropped.
// Allocated in setup(): two 38 KB buffers don't fit in static DRAM.
static uint16_t *captureBuf;
static uint16_t *readyBuf;
static portMUX_TYPE frameMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t readySeq = 0;
static bool readyInUse = false;

// Pixel temperature in degrees C = raw * tempScale + tempOffset.
static volatile float tempScale = 0;
static volatile float tempOffset = 0;
static volatile bool sensorOk = false;
// A snapshot line is being written to serial; routine log lines wait.
static volatile bool serialBusy = false;

// ---- Lepton CCI (I2C command interface) -----------------------------------

namespace cci {

constexpr uint16_t kRegStatus = 0x0002;
constexpr uint16_t kRegCommand = 0x0004;
constexpr uint16_t kRegDataLen = 0x0006;
constexpr uint16_t kRegData = 0x0008;

constexpr uint16_t kStatusBusy = 1 << 0;

constexpr uint16_t kTypeGet = 0;
constexpr uint16_t kTypeSet = 1;

// Command IDs (Lepton Software IDD).
constexpr uint16_t kAgcEnable = 0x0100;
constexpr uint16_t kSysFpaTempKelvin = 0x0214;
constexpr uint16_t kOemPartNumber = 0x481C;
constexpr uint16_t kOemGpioMode = 0x4854;
constexpr uint16_t kOemVsyncDelay = 0x4858;
constexpr uint16_t kRadTLinearEnable = 0x4EC0;
constexpr uint16_t kRadTLinearResolution = 0x4EC4;

constexpr int kErrTimeout = -1000;

static bool writeWords(uint16_t reg, const uint16_t *words, int n) {
  Wire1.beginTransmission(kLeptonI2cAddr);
  Wire1.write(reg >> 8);
  Wire1.write(reg & 0xFF);
  for (int i = 0; i < n; i++) {
    Wire1.write(words[i] >> 8);
    Wire1.write(words[i] & 0xFF);
  }
  return Wire1.endTransmission() == 0;
}

static bool readWords(uint16_t reg, uint16_t *words, int n) {
  Wire1.beginTransmission(kLeptonI2cAddr);
  Wire1.write(reg >> 8);
  Wire1.write(reg & 0xFF);
  if (Wire1.endTransmission() != 0) return false;
  if (Wire1.requestFrom(kLeptonI2cAddr, (uint8_t)(n * 2)) != n * 2) return false;
  for (int i = 0; i < n; i++) {
    uint16_t hi = Wire1.read();
    words[i] = (hi << 8) | Wire1.read();
  }
  return true;
}

// Waits for the camera to be idle; returns the LEP_RESULT code (0 = OK).
static int waitIdle(uint32_t timeoutMs = 5000) {
  uint32_t start = millis();
  uint16_t status = 0xFFFF;
  bool ok;
  do {
    ok = readWords(kRegStatus, &status, 1);
    if (ok && !(status & kStatusBusy)) return (int8_t)(status >> 8);
    vTaskDelay(1);
  } while (millis() - start < timeoutMs);
  Serial.printf("Lepton: busy timeout (read %s, status 0x%04X)\n", ok ? "ok" : "failed", status);
  return kErrTimeout;
}


static int command(uint16_t id, uint16_t type, uint16_t *data, int words) {
  xSemaphoreTake(wire1Mutex, portMAX_DELAY);
  int err = waitIdle();
  if (err != kErrTimeout) {
    if (type == kTypeSet) {
      // Data length is followed by the data registers, so write them together.
      uint16_t buf[17] = {(uint16_t)words};
      memcpy(buf + 1, data, words * 2);
      writeWords(kRegDataLen, buf, words + 1);
    } else {
      uint16_t len = words;
      writeWords(kRegDataLen, &len, 1);
    }
    uint16_t cmd = id | type;
    writeWords(kRegCommand, &cmd, 1);
    err = waitIdle();
    if (err == 0 && type == kTypeGet && !readWords(kRegData, data, words)) err = kErrTimeout;
  }
  xSemaphoreGive(wire1Mutex);
  return err;
}

static int get(uint16_t id, uint16_t *data, int words) { return command(id, kTypeGet, data, words); }
static int set(uint16_t id, uint16_t *data, int words) { return command(id, kTypeSet, data, words); }

// 32-bit enum/value, sent least-significant word first.
static int set32(uint16_t id, uint32_t v) {
  uint16_t d[2] = {(uint16_t)(v & 0xFFFF), (uint16_t)(v >> 16)};
  return set(id, d, 2);
}

}  // namespace cci

// ---- Lepton VoSPI (video over SPI) ----------------------------------------

static SPIClass leptonSpi(HSPI);
static SemaphoreHandle_t vsyncSem;
static bool radiometric = false;

static void IRAM_ATTR onVsync() {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(vsyncSem, &woken);
  if (woken) portYIELD_FROM_ISR();
}

static void updateFpaOffset() {
  uint16_t fpa;
  if (cci::get(cci::kSysFpaTempKelvin, &fpa, 1) == 0) {
    // M5Stack's empirical conversion for non-radiometric Leptons.
    tempOffset = fpa / 100.0f - 273.15f - 177.77f;
  }
}

static bool initLepton() {
  // Same reset and start-up sequence as M5Stack's StickT example, which is
  // known to work on this hardware. The Lepton won't boot if there is other
  // traffic on its I2C bus (here, AXP192 polling) while it starts, so hold
  // the bus until it's up.
  xSemaphoreTake(wire1Mutex, portMAX_DELAY);
  digitalWrite(kLeptonCsPin, HIGH);
  digitalWrite(kLeptonResetPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(kLeptonResetPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(300));
  digitalWrite(kLeptonResetPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Only start the SPI bus and VSYNC interrupt after reset, as the stock
  // firmware does.
  static bool spiStarted = false;
  if (!spiStarted) {
    spiStarted = true;
    pinMode(kLeptonVsyncPin, INPUT);
    attachInterrupt(kLeptonVsyncPin, onVsync, RISING);
    leptonSpi.begin(kLeptonSckPin, kLeptonMisoPin, kLeptonMosiPin, -1);
    leptonSpi.beginTransaction(SPISettings(kLeptonSpiHz, MSBFIRST, SPI_MODE3));
  }
  digitalWrite(kLeptonCsPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(1000));
  xSemaphoreGive(wire1Mutex);

  // VSYNC pulse on the Lepton's GPIO3 (the board's G38), as the stock
  // firmware configures it (one data word, as there).
  uint16_t v = 5;
  int err = cci::set(cci::kOemGpioMode, &v, 1);
  if (err) {
    Serial.printf("Lepton: VSYNC enable failed (%d)\n", err);
    return false;
  }
  v = 3;
  err = cci::set(cci::kOemVsyncDelay, &v, 1);
  if (err) Serial.printf("Lepton: VSYNC delay failed (%d)\n", err);
  Serial.println("Lepton: VSYNC configured");

  uint16_t part[16] = {};
  int partErr = cci::get(cci::kOemPartNumber, part, 16);
  if (partErr != 0) Serial.printf("Lepton: part number read failed (%d)\n", partErr);
  if (partErr == 0) {
    char s[33] = {};
    for (int i = 0; i < 16; i++) {  // Bytes are packed little-endian per word.
      s[i * 2] = part[i] & 0xFF;
      s[i * 2 + 1] = part[i] >> 8;
    }
    Serial.printf("Lepton part number: %s\n", s);
  }

  err = cci::set32(cci::kAgcEnable, 0);
  if (err) Serial.printf("Lepton: AGC disable failed (%d)\n", err);

  // Lepton 3.5 is radiometric: TLinear makes each pixel an absolute
  // temperature in kelvin. Earlier models reject the command.
  uint16_t res[2] = {};
  radiometric = cci::set32(cci::kRadTLinearEnable, 1) == 0 &&
                cci::get(cci::kRadTLinearResolution, res, 2) == 0;
  if (radiometric) {
    tempScale = res[0] == 0 ? 0.1f : 0.01f;
    tempOffset = -273.15f;
    Serial.printf("Lepton: radiometric (TLinear, %.2f K/count)\n", tempScale);
  } else {
    tempScale = 0.0217f;
    updateFpaOffset();
    Serial.println("Lepton: non-radiometric; using FPA-based estimate");
  }

  xSemaphoreTake(vsyncSem, 0);
  return true;
}

// Deassert CS long enough for the Lepton to drop sync (>185 ms), then resume.
static void resyncLepton() {
  digitalWrite(kLeptonCsPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(200));
  digitalWrite(kLeptonCsPin, LOW);
  xSemaphoreTake(vsyncSem, 0);
}

static uint16_t segmentBuf[60 * 80];

// Reads one segment (60 packets of 80 pixels). Returns the segment number
// 1-4, 0 for a segment the Lepton marks invalid, or -1 on loss of sync.
static int readSegment() {
  uint8_t pkt[164];
  int segment = -1;
  int discards = 0;
  for (int line = 0; line < 60;) {
    leptonSpi.transferBytes(nullptr, pkt, sizeof(pkt));
    if ((pkt[0] & 0x0F) == 0x0F) {  // Discard packet.
      if (line > 0 || ++discards > 120) return -1;
      continue;
    }
    int number = ((pkt[0] & 0x0F) << 8) | pkt[1];
    if (number != line) return -1;
    if (line == 20) {
      segment = (pkt[0] >> 4) & 0x07;
      if (segment == 0) return 0;  // The Lepton repeats frames; skip.
      if (segment > 4) return -1;
    }
    uint16_t *dst = segmentBuf + line * 80;
    for (int i = 0; i < 80; i++) dst[i] = (pkt[4 + i * 2] << 8) | pkt[5 + i * 2];
    line++;
  }
  return segment;
}

static void placeSegment(uint16_t *frame, int segment) {
  // Each segment is 30 rows; each row is two packets (left and right halves).
  for (int line = 0; line < 60; line++) {
    int row = (segment - 1) * 30 + line / 2;
    int col = (line & 1) * 80;
    memcpy(frame + row * kFrameW + col, segmentBuf + line * 80, 80 * sizeof(uint16_t));
  }
}

static void publishFrame() {
  portENTER_CRITICAL(&frameMux);
  if (!readyInUse) {
    std::swap(captureBuf, readyBuf);
    readySeq++;
  }
  portEXIT_CRITICAL(&frameMux);
}

static void sensorTask(void *) {
  bool ready = false;
  int expected = 1;
  int errors = 0;
  int vsyncTimeouts = 0;
  uint32_t lastFpaMs = 0;
  uint32_t statsMs = millis(), frames = 0, badSegments = 0, resyncs = 0;

  for (;;) {
    if (!ready) {
      sensorOk = false;
      ready = initLepton();
      if (!ready) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      expected = 1;
      errors = vsyncTimeouts = 0;
    }

    if (xSemaphoreTake(vsyncSem, pdMS_TO_TICKS(200)) != pdTRUE) {
      if (++vsyncTimeouts >= 10) {
        Serial.println("Lepton: no VSYNC, reinitialising");
        sensorOk = false;
        ready = false;
      }
      continue;
    }
    vsyncTimeouts = 0;

    if (millis() - statsMs >= 10000 && !serialBusy) {
      Serial.printf("Lepton: %.1f fps, %u bad segments, %u resyncs\n",
                    frames * 1000.0f / (millis() - statsMs), badSegments, resyncs);
      statsMs = millis();
      frames = badSegments = resyncs = 0;
    }

    int segment = readSegment();
    if (segment < 0) {
      expected = 1;
      badSegments++;
      if (++errors >= 30) {
        resyncLepton();
        resyncs++;
        errors = 0;
      }
      continue;
    }
    if (segment == 0) continue;
    errors = 0;

    if (segment == expected || segment == 1) {
      placeSegment(captureBuf, segment);
      expected = segment + 1;
      if (segment == 4) {
        sensorOk = true;
        frames++;
        publishFrame();
        expected = 1;
        if (!radiometric && millis() - lastFpaMs > 1000) {
          lastFpaMs = millis();
          updateFpaOffset();
        }
      }
    } else {
      expected = 1;
    }
  }
}

// ---- Battery and power button ---------------------------------------------

struct Battery {
  uint16_t mv = 0;
  uint8_t pct = 0;
  bool charging = false;
  bool usb = false;
};
static Battery battery;

static uint8_t batteryPercent(float v) {
  // Rough single-cell LiPo discharge curve.
  static const float pts[][2] = {{3.30f, 0},  {3.60f, 10}, {3.70f, 25}, {3.75f, 40},
                                 {3.85f, 60}, {4.00f, 80}, {4.15f, 100}};
  if (v <= pts[0][0]) return 0;
  for (int i = 1; i < 7; i++) {
    if (v <= pts[i][0]) {
      float f = (v - pts[i - 1][0]) / (pts[i][0] - pts[i - 1][0]);
      return pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * f;
    }
  }
  return 100;
}

static void readBattery() {
  xSemaphoreTake(wire1Mutex, portMAX_DELAY);
  float v = M5.Axp.GetBatVoltage();
  // AXP192 reg 0x01 bit 6: charging; reg 0x00 bit 5: VBUS (USB) present.
  battery.charging = M5.Axp.Read8bit(0x01) & 0x40;
  battery.usb = M5.Axp.Read8bit(0x00) & 0x20;
  xSemaphoreGive(wire1Mutex);
  battery.mv = v * 1000;
  battery.pct = batteryPercent(v);
}

// Power off on a long press of the power button, as the stock firmware does.
static void checkPowerButton() {
  xSemaphoreTake(wire1Mutex, portMAX_DELAY);
  uint8_t press = M5.Axp.GetBtnPress();
  if (press == 0x01) M5.Axp.Write1Byte(0x32, 0x80);
  xSemaphoreGive(wire1Mutex);
}

// Thumbwheel: an encoder at I2C 0x30 on the internal bus, read the way the
// stock firmware does: register 0x00 is the signed rotation since the last
// clear, and writing 0xFF to 0x20 clears it.
constexpr uint8_t kEncoderAddr = 0x30;

struct EncoderState {
  bool ok = false;      // The encoder answered.
  bool turned = false;  // Rotated since the last read...
  int steps = 0;        // ...by this much (signed: the direction).
};

static EncoderState readEncoder() {
  EncoderState st;
  xSemaphoreTake(wire1Mutex, portMAX_DELAY);
  Wire1.beginTransmission(kEncoderAddr);
  Wire1.write(0x00);
  if (Wire1.endTransmission() == 0 && Wire1.requestFrom(kEncoderAddr, (uint8_t)2) == 2) {
    st.ok = true;
    int16_t steps = (Wire1.read() << 8) | Wire1.read();
    if (steps != 0) {
      st.turned = true;
      st.steps = steps;
      Wire1.beginTransmission(kEncoderAddr);
      Wire1.write(0x20);
      Wire1.write(0xFF);
      Wire1.endTransmission();
    }
  }
  xSemaphoreGive(wire1Mutex);
  return st;
}

// Freeze frame: the display and web viewers keep showing the current frame.
static bool frozen = false;

// ---- Display --------------------------------------------------------------

constexpr uint16_t kHotColor = 0xF8A2;   // red
constexpr uint16_t kColdColor = 0x3D9F;  // blue

// Side panel layout (panel-local coordinates).
constexpr int kPanelX = kImgX + kFrameW + 4;
constexpr int kPanelW = kScreenW - kPanelX;
constexpr int kMaxLabelY = 22;
constexpr int kHistY = 33;
constexpr int kHistH = 86;
constexpr int kMinLabelY = kHistY + kHistH + 4;
constexpr int kScaleX = 5, kScaleW = 5;  // colour scale strip
constexpr int kBarsX = kScaleX + kScaleW + 2;

static uint16_t palette[256];         // RGB565, for drawing primitives
static uint16_t paletteSwapped[256];  // byte-swapped, for raw pushImage()
// The panel is drawn in strips through a small sprite: a full-height one
// costs 19 KB, which the Wi-Fi stack needs to stream frames.
constexpr int kPanelStripH = 27;
static_assert(kScreenH % kPanelStripH == 0, "strips must tile the panel");
static TFT_eSprite panel(&M5.Lcd);

// "Ironbow" palette; keep in sync with the web viewer.
static void buildPalette() {
  struct Stop { float pos; uint8_t r, g, b; };
  static const Stop stops[] = {
      {0.00f, 0, 0, 10},     {0.15f, 45, 0, 120},   {0.32f, 140, 0, 150},
      {0.50f, 215, 40, 70},  {0.65f, 245, 110, 0},  {0.82f, 255, 190, 0},
      {1.00f, 255, 255, 230},
  };
  for (int i = 0; i < 256; i++) {
    float t = i / 255.0f;
    int s = 0;
    while (s < 5 && t > stops[s + 1].pos) s++;
    const Stop &a = stops[s], &b = stops[s + 1];
    float f = (t - a.pos) / (b.pos - a.pos);
    palette[i] = M5.Lcd.color565(a.r + (b.r - a.r) * f, a.g + (b.g - a.g) * f,
                                 a.b + (b.b - a.b) * f);
    paletteSwapped[i] = (palette[i] >> 8) | (palette[i] << 8);
  }
}

static float toCelsius(float raw) { return raw * tempScale + tempOffset; }

// Draws into an 8-row chunk of byte-swapped pixels covering image rows
// [y0, y0 + 8); anything outside the chunk or image is clipped.
struct Chunk {
  uint16_t *px;
  int y0;
  void plot(int x, int y, uint16_t color) {
    if (x < 0 || x >= kFrameW || y < y0 || y >= y0 + 8 || y >= kFrameH) return;
    px[(y - y0) * kFrameW + x] = (color >> 8) | (color << 8);
  }
  void hline(int x0, int x1, int y, uint16_t color) {
    for (int x = x0; x <= x1; x++) plot(x, y, color);
  }
  void vline(int x, int y0_, int y1, uint16_t color) {
    for (int y = y0_; y <= y1; y++) plot(x, y, color);
  }
  // Triangle centred on (x, y), pointing up or down, with a black outline.
  void triangle(int x, int y, bool up, uint16_t color) {
    for (int pass = 0; pass < 2; pass++) {
      int grow = pass == 0 ? 1 : 0;
      for (int i = -grow; i <= 5 + grow; i++) {
        int half = (i + grow) * 4 / 6 + grow;
        int row = up ? y - 3 + i : y + 3 - i;
        hline(x - half, x + half, row, pass == 0 ? TFT_BLACK : color);
      }
    }
  }
  void crosshair(int x, int y) {
    for (int pass = 0; pass < 2; pass++) {
      uint16_t c = pass == 0 ? TFT_BLACK : TFT_WHITE;
      int o = pass == 0 ? 1 : 0;
      for (int d = -o; d <= o; d++) {
        hline(x - 8 - o, x - 3 + o, y + d, c);
        hline(x + 3 - o, x + 8 + o, y + d, c);
        vline(x + d, y - 8 - o, y - 3 + o, c);
        vline(x + d, y + 3 - o, y + 8 + o, c);
      }
    }
  }
};

static void drawDegrees(TFT_eSprite &s, int x, int y, uint16_t color) {
  s.drawCircle(x + 1, y + 1, 1, color);
}

// Temperature as "-12.3" plus a degree mark; returns the width drawn.
static int drawTemp(TFT_eSprite &s, float c, int x, int y, int font, uint16_t color) {
  char buf[12];
  snprintf(buf, sizeof(buf), "%.1f", c);
  s.setTextColor(color);
  int w = s.drawString(buf, x, y, font);
  drawDegrees(s, x + w + 1, y, color);
  return w + 4;
}

// Draws on the LCD or a sprite (a template, so each uses its own drawing
// functions rather than relying on which are virtual).
template <typename Canvas>
static void drawBatteryIcon(Canvas &s, int x, int y) {
  const int w = 15, h = 7;
  uint16_t outline = TFT_LIGHTGREY;
  s.drawRect(x, y, w, h, outline);
  s.fillRect(x + w, y + 2, 2, h - 4, outline);
  uint16_t color = battery.pct > 50 ? TFT_GREEN : battery.pct > 20 ? TFT_YELLOW : TFT_RED;
  int fill = (w - 4) * battery.pct / 100;
  if (battery.pct > 0 && fill < 1) fill = 1;
  s.fillRect(x + 2, y + 2, fill, h - 4, color);
  if (battery.usb) {
    // External power: a bolt over the battery, yellow while charging and
    // white once full. Outlined in black so it reads over the fill.
    static const char *const kBolt[] = {"...##", "..##.", ".##..", "#####", "..##.", ".##..", "##..."};
    const int bx = x + 5;
    auto lit = [](int c, int r) { return r >= 0 && r < 7 && c >= 0 && c < 5 && kBolt[r][c] == '#'; };
    for (int r = -1; r <= 7; r++) {
      for (int c = -1; c <= 5; c++) {
        if (lit(c, r)) continue;
        bool edge = false;
        for (int dr = -1; dr <= 1; dr++) for (int dc = -1; dc <= 1; dc++) edge |= lit(c + dc, r + dr);
        if (edge) s.drawPixel(bx + c, y + r, TFT_BLACK);
      }
    }
    uint16_t boltColor = battery.charging ? TFT_YELLOW : TFT_WHITE;
    for (int r = 0; r < 7; r++)
      for (int c = 0; c < 5; c++)
        if (lit(c, r)) s.drawPixel(bx + c, y + r, boltColor);
  }
}

static void drawCrosshairIcon(TFT_eSprite &s, int cx, int cy) {
  s.drawFastHLine(cx - 4, cy, 3, TFT_WHITE);
  s.drawFastHLine(cx + 2, cy, 3, TFT_WHITE);
  s.drawFastVLine(cx, cy - 4, 3, TFT_WHITE);
  s.drawFastVLine(cx, cy + 2, 3, TFT_WHITE);
}

static void drawPanelTriangle(TFT_eSprite &s, int x, int y, bool up, uint16_t color) {
  if (up) s.fillTriangle(x, y - 3, x - 3, y + 2, x + 3, y + 2, color);
  else s.fillTriangle(x, y + 3, x - 3, y - 2, x + 3, y - 2, color);
}

struct FrameStats {
  float lo, hi;            // colour range (raw counts)
  uint16_t minRaw, maxRaw;
  int minX, minY, maxX, maxY;
  float centreRaw;
  uint16_t hist[kHistH];   // row 0 = hottest
};

// Draws the panel shifted up by `oy` rows, so it can be rendered one strip
// at a time into a small sprite.
static void drawPanelContent(const FrameStats *st, int oy) {
  drawBatteryIcon(panel, kPanelW - 18, 4 - oy);
  if (frozen) {
    // Pause icon below the battery.
    panel.fillRect(kPanelW - 15, 13 - oy, 3, 8, TFT_WHITE);
    panel.fillRect(kPanelW - 9, 13 - oy, 3, 8, TFT_WHITE);
  }
  drawCrosshairIcon(panel, 5, 8 - oy);
  if (!st) {
    panel.setTextColor(TFT_DARKGREY);
    panel.drawString("--", 13, 1 - oy, 2);
    return;
  }

  drawTemp(panel, toCelsius(st->centreRaw), 13, 1 - oy, 2, TFT_WHITE);

  drawPanelTriangle(panel, 7, kMaxLabelY + 3 - oy, true, kHotColor);
  drawTemp(panel, toCelsius(st->maxRaw), 13, kMaxLabelY - oy, 1, TFT_WHITE);
  drawPanelTriangle(panel, 7, kMinLabelY + 3 - oy, false, kColdColor);
  drawTemp(panel, toCelsius(st->minRaw), 13, kMinLabelY - oy, 1, TFT_WHITE);

  // Colour scale with the histogram beside it, hottest at the top.
  uint16_t peak = 1;
  for (int r = 0; r < kHistH; r++) peak = max(peak, st->hist[r]);
  const int barMax = kPanelW - kBarsX - 1;
  for (int r = 0; r < kHistH; r++) {
    int y = kHistY + r - oy;
    if (y < 0 || y >= kPanelStripH) continue;
    uint16_t c = palette[255 - r * 255 / (kHistH - 1)];
    panel.drawFastHLine(kScaleX, y, kScaleW, c);
    int len = (st->hist[r] * barMax + peak - 1) / peak;
    if (len) panel.drawFastHLine(kBarsX, y, len, c);
  }

  // Pointer at the centre temperature on the scale.
  float f = (st->hi - st->centreRaw) / (st->hi - st->lo);
  int py = kHistY + constrain((int)(f * (kHistH - 1)), 0, kHistH - 1) - oy;
  panel.fillTriangle(kScaleX - 1, py, kScaleX - 5, py - 3, kScaleX - 5, py + 3, TFT_WHITE);
}

static void drawPanel(const FrameStats *st) {
  for (int oy = 0; oy < kScreenH; oy += kPanelStripH) {
    panel.fillSprite(TFT_BLACK);
    drawPanelContent(st, oy);
    panel.pushSprite(kPanelX, oy);
  }
}

static void drawFrame(const uint16_t *raw) {
  static FrameStats st;
  st.minRaw = UINT16_MAX;
  st.maxRaw = 0;
  for (int y = 0; y < kFrameH; y++) {
    for (int x = 0; x < kFrameW; x++) {
      uint16_t v = raw[y * kFrameW + x];
      if (v < st.minRaw) { st.minRaw = v; st.minX = x; st.minY = y; }
      if (v > st.maxRaw) { st.maxRaw = v; st.maxX = x; st.maxY = y; }
    }
  }
  const int cx = kFrameW / 2, cy = kFrameH / 2;
  st.centreRaw = (raw[(cy - 1) * kFrameW + cx - 1] + raw[(cy - 1) * kFrameW + cx] +
                  raw[cy * kFrameW + cx - 1] + raw[cy * kFrameW + cx]) / 4.0f;

  // Colour range follows the coldest and hottest pixels, smoothed so it
  // doesn't flicker, and at least 2 degrees C wide.
  static float lo = NAN, hi = NAN;
  if (isnan(lo)) {
    lo = st.minRaw;
    hi = st.maxRaw;
  } else {
    lo += (st.minRaw - lo) * 0.3f;
    hi += (st.maxRaw - hi) * 0.3f;
  }
  float minSpan = tempScale > 0 ? 2.0f / tempScale : 100.0f;
  float mid = (lo + hi) / 2;
  st.lo = min(lo, mid - minSpan / 2);
  st.hi = max(hi, mid + minSpan / 2);
  const float scale = 255.0f / (st.hi - st.lo);
  const float histScale = kHistH / (st.hi - st.lo);

  memset(st.hist, 0, sizeof(st.hist));
  static uint16_t lines[kFrameW * 8];
  Chunk chunk{lines, 0};
  for (int y = 0; y < kFrameH; y += 8) {
    for (int i = 0; i < kFrameW * 8; i++) {
      float d = raw[y * kFrameW + i] - st.lo;
      lines[i] = paletteSwapped[constrain((int)(d * scale), 0, 255)];
      st.hist[kHistH - 1 - constrain((int)(d * histScale), 0, kHistH - 1)]++;
    }
    chunk.y0 = y;
    chunk.triangle(st.minX, st.minY, false, kColdColor);
    chunk.triangle(st.maxX, st.maxY, true, kHotColor);
    chunk.crosshair(cx, cy);
    M5.Lcd.pushImage(kImgX, kImgY + y, kFrameW, 8, lines);
  }
  drawPanel(&st);
}

static void drawNoSensor() {
  M5.Lcd.fillRect(kImgX, kImgY, kFrameW, kFrameH, TFT_BLACK);
  M5.Lcd.drawRect(kImgX, kImgY, kFrameW, kFrameH, TFT_DARKGREY);
  int cx = kImgX + kFrameW / 2, cy = kImgY + kFrameH / 2;
  for (int d = -1; d <= 1; d++) {
    M5.Lcd.drawLine(cx - 25 + d, cy - 25, cx + 25 + d, cy + 25, TFT_RED);
    M5.Lcd.drawLine(cx + 25 + d, cy - 25, cx - 25 + d, cy + 25, TFT_RED);
  }
  drawPanel(nullptr);
}

// ---- Web ------------------------------------------------------------------

extern const uint8_t indexHtmlStart[] asm("_binary_web_index_html_start");
extern const uint8_t indexHtmlEnd[] asm("_binary_web_index_html_end");

static AsyncWebServer server(80);
static volatile int wifiDisconnectReason = 0;  // Last reason since losing the connection.
static volatile bool gotIp = false;  // Set by the Wi-Fi event; mDNS starts from loop().
static volatile bool wifiChanged = false;  // Print a status line over serial.
static AsyncWebSocket ws("/ws");

// Wire format (little-endian), see README.
enum : uint8_t {
  kFlagSensorOk = 1 << 0,
  kFlagCharging = 1 << 1,
  kFlagUsbPower = 1 << 2,
  kFlagFrozen = 1 << 3,
};

struct __attribute__((packed)) FrameHeader {
  uint8_t version;  // 2: raw pixels; 3: delta-encoded pixels
  uint8_t flags;
  uint8_t batteryPct;
  uint8_t reserved;
  uint16_t batteryMv;
  uint16_t seq;
  uint16_t width;
  uint16_t height;
  float scale;   // degrees C = raw * scale + offset
  float offset;
};

// Static addressing if STATIC_IP is set in secrets.ini, otherwise DHCP.
static bool configureAddress() {
  IPAddress ip, gateway, subnet;
  if (!ip.fromString(STATIC_IP)) {
    if (strlen(STATIC_IP)) Serial.printf("Wi-Fi: invalid static_ip \"%s\"; using DHCP\n", STATIC_IP);
    return false;
  }
  if (!gateway.fromString(GATEWAY) || !subnet.fromString(SUBNET)) {
    Serial.println("Wi-Fi: static_ip needs gateway and subnet; using DHCP");
    return false;
  }
  WiFi.config(ip, gateway, subnet);  // No DNS: the camera never looks up names.
  return true;
}

static void startNetwork() {
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
      Serial.printf("Wi-Fi: joined \"%s\" (channel %d, RSSI %d dBm)\n", WIFI_SSID,
                    info.wifi_sta_connected.channel, WiFi.RSSI());
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      wifiDisconnectReason = 0;
      gotIp = true;
      wifiChanged = true;
      Serial.printf("Wi-Fi: viewer at http://%s/ (or http://m5-thermal.local/)\n",
                    IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
    } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      // Reason codes: wifi_err_reason_t, e.g. 15 = handshake timeout (wrong
      // password), 201 = network not found. The driver retries by itself.
      // The driver retries every few seconds; only report a new reason.
      if (wifiDisconnectReason != info.wifi_sta_disconnected.reason) wifiChanged = true;
      wifiDisconnectReason = info.wifi_sta_disconnected.reason;
      Serial.printf("Wi-Fi: disconnected (reason %d)\n", info.wifi_sta_disconnected.reason);
    }
  });
  WiFi.persistent(false);  // Don't keep credentials in flash; they're built in.
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("m5-thermal");
  // An IPv6 link-local address lets mDNS answer IPv6 lookups for
  // m5-thermal.local; without one, clients wait 5 s for them to time out.
  WiFi.enableIPv6();
  bool isStatic = configureAddress();
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Modem sleep delays incoming packets by up to a beacon interval, which
  // makes the stream stutter.
  WiFi.setSleep(false);
  Serial.printf("Wi-Fi: joining \"%s\" (%s)\n", WIFI_SSID, isStatic ? "static " STATIC_IP : "DHCP");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    // embed_txtfiles appends a NUL terminator; don't send it.
    req->send(200, "text/html", indexHtmlStart, indexHtmlEnd - indexHtmlStart - 1);
  });
  server.onNotFound([](AsyncWebServerRequest *req) { req->redirect("/"); });
  ws.onEvent([](AsyncWebSocket *, AsyncWebSocketClient *client, AwsEventType type, void *,
                uint8_t *, size_t) {
    if (type == WS_EVT_CONNECT) client->setCloseClientOnQueueFull(true);
  });
  server.addHandler(&ws);
  server.begin();
}

// Frames go out from one buffer reserved at boot: allocating one per frame
// fails once Wi-Fi clients have fragmented the heap. Delta-encoded frames are
// typically ~19-20 KB; 28 KB leaves room for detailed scenes without the
// 38 KB raw size, which there isn't memory for on core 3.x. A frame that
// doesn't fit is skipped.
static AsyncWebSocketSharedBuffer frameMsg;
constexpr size_t kFrameMsgCapacity = sizeof(FrameHeader) + 28 * 1024;

// Delta-encodes pixels against their left neighbour (or the pixel above, at
// the start of a row). Zigzag-encoded deltas below 128 take one byte; others
// take two, with the top bit of the first byte set. Thermal images are smooth,
// so frames shrink to roughly half. Returns 0 if a delta doesn't fit or the
// result would exceed `maxLen`.
static size_t encodeFrame(const uint16_t *px, uint8_t *out, size_t maxLen) {
  uint8_t *o = out;
  uint8_t *const end = out + maxLen - 1;  // Room for a two-byte delta.
  for (int y = 0; y < kFrameH; y++) {
    for (int x = 0; x < kFrameW; x++) {
      int i = y * kFrameW + x;
      int pred = x > 0 ? px[i - 1] : y > 0 ? px[i - kFrameW] : 0;
      int32_t d = (int32_t)px[i] - pred;
      uint32_t z = ((uint32_t)d << 1) ^ (uint32_t)(d >> 31);
      if (o >= end) return 0;
      if (z < 0x80) {
        *o++ = z;
      } else if (z < 0x8000) {
        *o++ = 0x80 | (z >> 8);
        *o++ = z & 0xFF;
      } else {
        return 0;
      }
    }
  }
  return o - out;
}
static uint32_t framesSent = 0, framesSkipped = 0, bytesSent = 0;
static uint32_t frameMsgBusySince = 0;
// A viewer stalled: have the Wi-Fi watchdog check the link now.
static bool gatewayCheckRequested = false;
// Watchdog reconnects since boot, and the uptime (s) of the latest; reported
// in the status line to correlate freezes with what was going on.
static uint32_t watchdogReconnects = 0, lastWatchdogReconnectS = 0;
// A serial snapshot borrows frameMsg (holding a reference) while it's sent.
static AsyncWebSocketSharedBuffer snapMsg;
static bool snapRequested = false;  // A snapshot is waiting for frameMsg...
static uint32_t snapRequestedMs = 0;  // ...since this time.

// Header for a frame message (version 2; callers set 3 once delta-encoded).
static FrameHeader makeHeader(bool hasPixels, uint32_t seq) {
  FrameHeader h = {};
  h.version = 2;
  h.flags = (sensorOk ? kFlagSensorOk : 0) | (battery.charging ? kFlagCharging : 0) |
            (battery.usb ? kFlagUsbPower : 0) | (frozen ? kFlagFrozen : 0);
  h.batteryPct = battery.pct;
  h.batteryMv = battery.mv;
  h.seq = seq;
  h.width = hasPixels ? kFrameW : 0;
  h.height = hasPixels ? kFrameH : 0;
  h.scale = tempScale;
  h.offset = tempOffset;
  return h;
}

// Header plus delta-encoded pixels, into `out` (kFrameMsgCapacity bytes).
// Returns the size, or 0 if the frame doesn't fit.
static size_t buildFrameMessage(const uint16_t *pixels, uint32_t seq, uint8_t *out) {
  FrameHeader h = makeHeader(true, seq);
  h.version = 3;
  size_t payload = encodeFrame(pixels, out + sizeof(h), kFrameMsgCapacity - sizeof(h));
  if (!payload) return 0;
  memcpy(out, &h, sizeof(h));
  return sizeof(h) + payload;
}

static void sendFrame(const uint16_t *pixels, uint32_t seq) {
  if (ws.count() == 0) return;

  AsyncWebSocketSharedBuffer buf;
  if (pixels) {
    // Still referenced by a viewer's send queue: that viewer is behind, so
    // skip this frame rather than overwrite the one being sent.
    // Lent to (or being drained for) a serial snapshot; viewers resume after.
    if (snapMsg || snapRequested) {
      framesSkipped++;
      return;
    }
    if (frameMsg.use_count() > 1) {
      framesSkipped++;
      // A viewer that stops reading (e.g. a backgrounded browser tab) never
      // finishes, which would stall every viewer since they share frameMsg.
      // After a while, send a tiny status message: viewers with an empty
      // queue just get it, and the stuck one has a full queue so the library
      // closes it (setCloseClientOnQueueFull). Its page reconnects by itself.
      uint32_t now = millis();
      if (!frameMsgBusySince) {
        frameMsgBusySince = now;
      } else if (now - frameMsgBusySince > 3000) {
        Serial.println("Web: viewer stalled; closing it");
        gatewayCheckRequested = true;  // Often the first sign of a wedged link.
        frameMsgBusySince = 0;
        sendFrame(nullptr, seq);
      }
      return;
    }
    frameMsgBusySince = 0;
    buf = frameMsg;
  } else {
    buf = std::make_shared<std::vector<uint8_t>>(sizeof(FrameHeader));
  }

  FrameHeader h = makeHeader(pixels != nullptr, seq);
  if (pixels) {
    buf->resize(kFrameMsgCapacity);  // Within capacity: never reallocates.
    size_t n = buildFrameMessage(pixels, seq, buf->data());
    if (!n) {  // Too detailed to fit; skip this one.
      framesSkipped++;
      return;
    }
    buf->resize(n);
    framesSent++;
    bytesSent += buf->size();
  } else {
    memcpy(buf->data(), &h, sizeof(h));
  }
  // Viewers still sending the previous message skip this one.
  ws.binaryAll(buf);
}

// ---- Network status page (shown when the thumbwheel is turned) ------------

constexpr uint32_t kStatusShowMs = 15000;
// Ignore a reverse turn this soon after opening (the wheel can bounce back).
constexpr uint32_t kStatusReverseGuardMs = 400;
constexpr uint16_t kLabelColor = 0x8C71;  // grey

static const char *disconnectText(int reason) {
  switch (reason) {
    case WIFI_REASON_NO_AP_FOUND: return "network not found";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "wrong password?";
    case WIFI_REASON_ASSOC_FAIL: return "association failed";
    case 0: return "joining...";
    default: return "disconnected";
  }
}

// Draws the page; with `full`, clears the screen and draws the labels too.
// Values are drawn with padding so they update in place without flicker.
static void drawStatus(bool full) {
  auto &lcd = M5.Lcd;
  if (full) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextDatum(TL_DATUM);
    lcd.setTextPadding(0);
    lcd.setTextColor(kLabelColor, TFT_BLACK);
    lcd.drawString("NETWORK", 8, 6, 1);
    lcd.drawString("IP ADDRESS", 8, 40, 1);
    lcd.drawString("SIGNAL", 8, 74, 1);
    lcd.drawString("VIEWERS", 88, 74, 1);
    lcd.drawString("BATTERY", 156, 74, 1);
    lcd.drawString("m5-thermal.local", 8, 112, 2);
  }

  const bool connected = WiFi.status() == WL_CONNECTED;
  char buf[48];

  lcd.setTextPadding(kScreenW - 16);
  if (connected) {
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%s", WIFI_SSID);
  } else {
    lcd.setTextColor(TFT_RED, TFT_BLACK);
    snprintf(buf, sizeof(buf), "Not connected: %s", disconnectText(wifiDisconnectReason));
  }
  // Shorten long network names to fit.
  while (strlen(buf) > 4 && lcd.textWidth(buf, 2) > kScreenW - 16) strcpy(buf + strlen(buf) - 4, "...");
  lcd.drawString(buf, 8, 16, 2);

  lcd.setTextColor(connected ? TFT_WHITE : kLabelColor, TFT_BLACK);
  lcd.drawString(connected ? WiFi.localIP().toString().c_str() : "none", 8, 50, 2);

  lcd.setTextPadding(76);
  if (connected) snprintf(buf, sizeof(buf), "%d dBm", WiFi.RSSI());
  else snprintf(buf, sizeof(buf), "-");
  lcd.drawString(buf, 8, 84, 2);
  lcd.setTextPadding(64);
  snprintf(buf, sizeof(buf), "%u", (unsigned)ws.count());
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(buf, 88, 84, 2);

  // Battery: the same icon as the main screen (with the external-power bolt)
  // and the percentage.
  lcd.fillRect(156, 85, 20, 13, TFT_BLACK);
  drawBatteryIcon(lcd, 157, 89);
  lcd.setTextPadding(kScreenW - 180);
  snprintf(buf, sizeof(buf), "%u%%", battery.pct);
  lcd.drawString(buf, 180, 84, 2);
  lcd.setTextPadding(0);
}

// ---- Serial console --------------------------------------------------------

// Machine-readable status for the WebSerial page (index.html at the repo
// root): one "@status {json}" line, printed when "status" is received and
// whenever the Wi-Fi connection changes.
static void printJson(const char *s) {
  Serial.print('"');
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') Serial.print('\\');
    if ((uint8_t)*s >= 0x20) Serial.print(*s);
  }
  Serial.print('"');
}

static void printStatusLine() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  Serial.print("@status {\"device\":\"m5-thermal\",\"ssid\":");
  printJson(WIFI_SSID);
  Serial.printf(",\"connected\":%s,\"ip\":", connected ? "true" : "false");
  if (connected) printJson(WiFi.localIP().toString().c_str());
  else Serial.print("null");
  Serial.printf(",\"rssi\":%d,\"reason\":", connected ? WiFi.RSSI() : 0);
  if (connected) Serial.print("null");
  else printJson(disconnectText(wifiDisconnectReason));
  Serial.printf(",\"viewers\":%u,\"frozen\":%s,\"sensor\":%s,\"battery\":%u,\"uptime\":%lu,\"heap\":%u,\"heapLowest\":%u,"
                "\"watchdogReconnects\":%u,\"lastWatchdogReconnect\":%u}\n",
                (unsigned)ws.count(), frozen ? "true" : "false", sensorOk ? "true" : "false", battery.pct,
                millis() / 1000, ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                (unsigned)watchdogReconnects, (unsigned)lastWatchdogReconnectS);
}

// Single frame over serial ("snapshot"): one line, "@frame <bytes> <crc32>
// <base64>", carrying the same message as the WebSocket stream. It's ~26 KB
// of text, ~2-3 s at 115200 baud, so it's streamed from loop() in pieces.
// There isn't memory for its own 38 KB buffer, so it borrows frameMsg
// (pausing web viewers meanwhile). Other log lines can still land inside it
// occasionally; the CRC lets the receiver detect that.
static size_t snapLen = 0, snapPos = 0;

static void requestSnapshot() {
  if (snapMsg || snapRequested) return;  // One at a time.
  if (!sensorOk) {
    Serial.println("@frame-error no image yet");
    return;
  }
  // The Lepton's first frames after a reset are unsettled (wild values).
  if (readySeq < 20) {
    Serial.println("@frame-error camera warming up");
    return;
  }
  snapRequested = true;
  snapRequestedMs = millis();
}

static void startSnapshotIfReady() {
  if (!snapRequested) return;
  if (frameMsg.use_count() > 1) {  // A viewer's frame is still being sent.
    if (millis() - snapRequestedMs > 3000) {
      snapRequested = false;
      Serial.println("@frame-error busy");
    }
    return;
  }
  snapRequested = false;
  snapMsg = frameMsg;
  // The frame on screen: readyBuf. Hold it (as when frozen) while encoding.
  portENTER_CRITICAL(&frameMux);
  bool wasHeld = readyInUse;
  readyInUse = true;
  uint32_t seq = readySeq;
  portEXIT_CRITICAL(&frameMux);
  snapMsg->resize(kFrameMsgCapacity);
  snapLen = buildFrameMessage(readyBuf, seq, snapMsg->data());
  if (!wasHeld) {
    portENTER_CRITICAL(&frameMux);
    readyInUse = false;
    portEXIT_CRITICAL(&frameMux);
  }
  if (!snapLen) {
    snapMsg.reset();
    Serial.println("@frame-error frame too detailed to encode");
    return;
  }
  snapPos = 0;
  serialBusy = true;
  Serial.printf("@frame %u %08x ", (unsigned)snapLen, (unsigned)crc32_le(0, snapMsg->data(), snapLen));
}

static void continueSnapshot() {
  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  char out[256];
  while (snapMsg && Serial.availableForWrite() >= (int)sizeof(out)) {
    const uint8_t *src = snapMsg->data();
    size_t n = 0;
    while (n + 4 <= sizeof(out) && snapPos < snapLen) {
      uint32_t v = src[snapPos] << 16;
      size_t k = min((size_t)3, snapLen - snapPos);
      if (k > 1) v |= src[snapPos + 1] << 8;
      if (k > 2) v |= src[snapPos + 2];
      out[n++] = b64[(v >> 18) & 63];
      out[n++] = b64[(v >> 12) & 63];
      out[n++] = k > 1 ? b64[(v >> 6) & 63] : '=';
      out[n++] = k > 2 ? b64[v & 63] : '=';
      snapPos += k;
    }
    Serial.write((const uint8_t *)out, n);
    if (snapPos >= snapLen) {
      Serial.write('\n');
      snapMsg.reset();
      serialBusy = false;
    }
  }
}

static void pollSerial() {
  static char line[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[len] = 0;
      // Status lines wait until a snapshot line is finished.
      if (!strcmp(line, "status")) wifiChanged = true;
      else if (!strcmp(line, "snapshot")) requestSnapshot();
      else if (!strcmp(line, "reconnect")) {
        Serial.println("Wi-Fi: reconnecting (requested)");
        WiFi.reconnect();
      }
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  startSnapshotIfReady();
  if (wifiChanged && !snapMsg) {  // Not in the middle of a snapshot line.
    wifiChanged = false;
    printStatusLine();
  }
  continueSnapshot();
}

// ---- Connectivity watchdog --------------------------------------------------

// The ESP32 can end up "connected" while no traffic gets through: seen here,
// the camera kept its address but didn't answer ARP or ping, and its own
// pings failed to send, so its Wi-Fi transmit path was wedged. Every 10 s
// (or right away when a web viewer stalls), ping the router three times; if
// none get a reply, reconnect. Armed only once the router has answered, so
// one that ignores pings can't cause reconnects.
constexpr uint32_t kGatewayCheckMs = 10000;
constexpr int kGatewayFailuresBeforeReconnect = 1;

static esp_ping_handle_t pingSession = nullptr;
static volatile bool pingDone = false;
static volatile uint32_t pingReplies = 0;

static void onPingEnd(esp_ping_handle_t hdl, void *) {
  uint32_t replies = 0;
  esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &replies, sizeof(replies));
  pingReplies = replies;
  pingDone = true;  // The session is deleted from loop(), not from here.
}

static void checkGateway(uint32_t now) {
  static uint32_t lastCheckMs = 0;
  static bool armed = false;
  static int failures = 0;

  if (pingSession && pingDone) {
    esp_ping_delete_session(pingSession);
    pingSession = nullptr;
    if (pingReplies) {
      if (!armed) Serial.printf("Wi-Fi: watchdog armed (router %s answers)\n", WiFi.gatewayIP().toString().c_str());
      armed = true;
      failures = 0;
    } else if (armed && ++failures >= kGatewayFailuresBeforeReconnect) {
      Serial.printf("Wi-Fi: router %s not answering (RSSI %d dBm); reconnecting\n",
                    WiFi.gatewayIP().toString().c_str(), WiFi.RSSI());
      failures = 0;
      armed = false;
      watchdogReconnects++;
      lastWatchdogReconnectS = millis() / 1000;
      WiFi.reconnect();
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    failures = 0;
    return;
  }
  if (pingSession) return;
  if (now - lastCheckMs < kGatewayCheckMs && !gatewayCheckRequested) return;
  gatewayCheckRequested = false;
  lastCheckMs = now;

  IPAddress gw = WiFi.gatewayIP();
  if (gw == IPAddress(0, 0, 0, 0)) return;
  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  IP_ADDR4(&cfg.target_addr, gw[0], gw[1], gw[2], gw[3]);
  cfg.count = 3;
  cfg.interval_ms = 300;
  esp_ping_callbacks_t cbs = {};
  cbs.on_ping_end = onPingEnd;
  pingDone = false;
  if (esp_ping_new_session(&cfg, &cbs, &pingSession) == ESP_OK) {
    esp_ping_start(pingSession);
  } else {
    pingSession = nullptr;
  }
}

// ---- Main -----------------------------------------------------------------

void setup() {
  // A transmit buffer lets snapshots stream without blocking loop(); it must
  // be set before M5.begin() starts the port.
  Serial.setTxBufferSize(4096);
  M5.begin();
  wire1Mutex = xSemaphoreCreateMutex();
  captureBuf = (uint16_t *)calloc(kFramePixels, sizeof(uint16_t));
  readyBuf = (uint16_t *)calloc(kFramePixels, sizeof(uint16_t));
  if (!captureBuf || !readyBuf) {
    Serial.println("Out of memory for frame buffers");
    for (;;) delay(1000);
  }
  vsyncSem = xSemaphoreCreateBinary();
  frameMsg = std::make_shared<std::vector<uint8_t>>(kFrameMsgCapacity);

  M5.Lcd.setRotation(kScreenRotation);
  M5.Lcd.fillScreen(TFT_BLACK);
  buildPalette();
  panel.createSprite(kPanelW, kPanelStripH);
  readBattery();
  drawNoSensor();

  pinMode(kLeptonCsPin, OUTPUT);
  digitalWrite(kLeptonCsPin, HIGH);
  pinMode(kLeptonResetPin, OUTPUT);
  digitalWrite(kLeptonResetPin, HIGH);

  startNetwork();
  Serial.printf("Heap free %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  xTaskCreatePinnedToCore(sensorTask, "lepton", 6144, nullptr, 5, nullptr, 1);
}

void loop() {
  static uint32_t lastSeq = 0;
  static uint32_t lastBatteryMs = 0, lastButtonMs = 0, lastCleanupMs = 0, lastStatusMs = 0;
  static uint32_t lastWebStatsMs = 0, lastEncoderMs = 0, statusShownMs = 0, lastStatusDrawMs = 0;
  static bool showingStatus = false, closeStatus = false;
  static int statusDir = 0;  // Direction of the turn that opened the status page.
  static uint32_t statusOpenedMs = 0;
  static uint8_t encoderFailures = 0;
  static uint32_t lastFrozenSendMs = 0, frozenSeq = 0;
  static bool shownNoSensor = true;
  uint32_t now = millis();

  if (now - lastButtonMs >= 100) {
    lastButtonMs = now;
    checkPowerButton();
  }

  // The button on top of the case is the StickC's button B: freeze frame.
  pollSerial();
  checkGateway(now);

  // mDNS (m5-thermal.local) only announces reliably if started once we have
  // an address.
  static bool mdnsStarted = false;
  if (gotIp && !mdnsStarted) {
    mdnsStarted = true;
    if (MDNS.begin("m5-thermal")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("mDNS: m5-thermal.local");
    }
  }

  M5.update();
  const bool freezePressed = M5.BtnB.wasPressed();

  // Thumbwheel: turning it shows the network status page. Poll less often if
  // the encoder isn't answering, so it can't slow the shared I2C bus.
  if (now - lastEncoderMs >= (encoderFailures >= 3 ? 5000u : 100u)) {
    lastEncoderMs = now;
    EncoderState enc = readEncoder();
    encoderFailures = enc.ok ? 0 : min(encoderFailures + 1, 3);
    // The first turn opens the status page; turning the same way keeps it
    // open, and the other way closes it.
    if (enc.turned) {
      const int dir = enc.steps > 0 ? 1 : -1;
      if (!showingStatus) {
        drawStatus(true);
        showingStatus = true;
        statusDir = dir;
        statusOpenedMs = statusShownMs = lastStatusDrawMs = now;
      } else if (dir == statusDir) {
        statusShownMs = now;
      } else if (now - statusOpenedMs >= kStatusReverseGuardMs) {
        closeStatus = true;
      }
    }
  }

  if (freezePressed) {
    Serial.printf("Freeze %s\n", frozen ? "off" : "on");
    portENTER_CRITICAL(&frameMux);
    // Holding readyBuf stops the sensor task replacing it; frames keep being
    // captured in the background and are dropped until we let go.
    frozen = readyInUse = !frozen;
    frozenSeq = readySeq;
    portEXIT_CRITICAL(&frameMux);
    if (frozen && sensorOk) {
      if (!showingStatus) drawFrame(readyBuf);  // Adds the pause icon.
      sendFrame(readyBuf, frozenSeq);
      lastFrozenSendMs = now;
    }
  }

  if (showingStatus) {
    if (closeStatus || now - statusShownMs >= kStatusShowMs) {
      showingStatus = closeStatus = false;
      M5.Lcd.fillScreen(TFT_BLACK);
      if (!sensorOk) drawNoSensor();
      else if (frozen) drawFrame(readyBuf);
      // Otherwise the next frame redraws everything.
    } else if (now - lastStatusDrawMs >= 500) {
      lastStatusDrawMs = now;
      drawStatus(false);
    }
  }

  if (now - lastBatteryMs >= 2000) {
    lastBatteryMs = now;
    Battery prev = battery;
    readBattery();
    // With frames arriving the panel redraws anyway; otherwise refresh it here.
    if (!sensorOk && !showingStatus && (prev.pct != battery.pct || prev.charging != battery.charging)) {
      drawPanel(nullptr);
    }
  }

  const uint16_t *frame = nullptr;
  uint32_t seq;
  portENTER_CRITICAL(&frameMux);
  seq = readySeq;
  if (!frozen && seq != lastSeq) {
    readyInUse = true;
    frame = readyBuf;
  }
  portEXIT_CRITICAL(&frameMux);

  if (frozen) {
    // Keep viewers (including newly connected ones) showing the frozen frame.
    if (sensorOk && now - lastFrozenSendMs >= 1000) {
      lastFrozenSendMs = now;
      sendFrame(readyBuf, frozenSeq);
    }
  } else if (frame) {
    lastSeq = seq;
    shownNoSensor = false;
    if (!showingStatus) drawFrame(frame);
    sendFrame(frame, seq);
    portENTER_CRITICAL(&frameMux);
    readyInUse = false;
    portEXIT_CRITICAL(&frameMux);
  } else if (!sensorOk && !shownNoSensor && !showingStatus) {
    shownNoSensor = true;
    drawNoSensor();
  }

  // Keep web clients informed (battery, sensor state) even with no frames.
  if (!sensorOk && now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    sendFrame(nullptr, seq);
  }

  if (now - lastWebStatsMs >= 10000 && !serialBusy) {
    lastWebStatsMs = now;
    if (ws.count()) {
      Serial.printf("Web: %u viewers, %u frames sent (avg %u B), %u skipped; heap %u free, %u largest, %u lowest\n",
                    ws.count(), framesSent, framesSent ? bytesSent / framesSent : 0, framesSkipped,
                    ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
    }
    framesSent = framesSkipped = bytesSent = 0;
  }

  if (now - lastCleanupMs >= 1000) {
    lastCleanupMs = now;
    ws.cleanupClients();
  }

  delay(2);
}
