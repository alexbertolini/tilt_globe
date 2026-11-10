/*
  Tilt Globe - Waveshare ESP32-S3-LCD-1.28 (GC9A01 round LCD + QMI8658 IMU)

  A shaded, textured planet Earth rendered in software. Tilt the board and the world spins
  in that direction (it is treated as a free ball, the planet's axis does not matter).
  The view starts centred on a random place on Earth.

  - BOOT button: short press = jump to a new random place, hold ~1 s = recalibrate "level"

  Files in this sketch folder:
    tilt_globe.ino   this file (hardware, input, main loop)
    globe_render.h   the renderer
    earth_texture.h  512x256 Earth map (RGB565)

  Library needed: "GFX Library for Arduino" (moononournation)
  Board: ESP32S3 Dev Module, USB CDC On Boot = Disabled (this board uses a CH343 USB-serial chip),
         Flash 16MB, PSRAM = QSPI PSRAM (recommended), Partition scheme: any with >= 1.5 MB app is fine
         (the default works too).
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "earth_texture.h"
#include "globe_render.h"

// Newer Arduino_GFX versions removed the short colour names, so define the ones we use
#ifndef BLACK
#define BLACK  0x0000
#define WHITE  0xFFFF
#define RED    0xF800
#endif

// ---------------- Pins (non-touch ESP32-S3-LCD-1.28) ----------------
#define PIN_SCK   10
#define PIN_MOSI  11
#define PIN_CS     9
#define PIN_DC     8
#define PIN_RST   12
#define PIN_BL    40
#define PIN_SDA    6
#define PIN_SCL    7
#define PIN_BOOT   0

// ---------------- Tilt mapping (same meaning as in the maze sketch) ----------------
#define SWAP_XY    0
#define INVERT_X   1
#define INVERT_Y   1
#define DEBUG_FPS  1   // 1 = print frames per second on Serial

// ---------------- Feel ----------------
const float SPIN_GAIN = 2.2f;    // rad/s of surface speed per g of tilt
const float MAX_SPIN  = 2.6f;    // rad/s limit
const float DEADZONE  = 0.04f;   // g
const float SPIN_TAU  = 0.35f;   // seconds, smoothing: higher = heavier, coasts longer

// ---------------- Display ----------------
Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, PIN_RST, 0 /*rotation*/, true /*IPS*/);

// ---------------- IMU (QMI8658) ----------------
#define QMI_WHO_AM_I 0x00
#define QMI_CTRL1    0x02
#define QMI_CTRL2    0x03
#define QMI_CTRL7    0x08
#define QMI_AX_L     0x35
#define QMI_RESET    0x60

uint8_t imuAddr = 0;
float ax0 = 0, ay0 = 0;

bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t reg, uint8_t *buf, int n) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)imuAddr, (int)n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  const uint8_t addrs[2] = {0x6B, 0x6A};
  bool found = false;
  for (int i = 0; i < 2 && !found; i++) {
    imuAddr = addrs[i];
    uint8_t id = 0;
    if (i2cRead(QMI_WHO_AM_I, &id, 1) && id == 0x05) found = true;
  }
  if (!found) return false;
  i2cWrite(QMI_RESET, 0xB0);   // soft reset
  delay(30);
  i2cWrite(QMI_CTRL1, 0x40);   // address auto-increment
  i2cWrite(QMI_CTRL2, 0x05);   // accel +-2g, 250 Hz
  i2cWrite(QMI_CTRL7, 0x01);   // enable accelerometer
  delay(20);
  return true;
}

bool readAccel(float &ax, float &ay, float &az) {
  uint8_t b[6];
  if (!i2cRead(QMI_AX_L, b, 6)) return false;
  int16_t rx = (int16_t)(b[0] | (b[1] << 8));
  int16_t ry = (int16_t)(b[2] | (b[3] << 8));
  int16_t rz = (int16_t)(b[4] | (b[5] << 8));
  ax = rx / 16384.0f;
  ay = ry / 16384.0f;
  az = rz / 16384.0f;
  return true;
}

void centerText(const char *txt, int y, int size, uint16_t color) {
  int w = strlen(txt) * 6 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(FB_CX - w / 2, y);
  gfx->print(txt);
}

void calibrateLevel() {
  gfx->fillScreen(BLACK);
  centerText("Hold level", 100, 2, WHITE);
  delay(1500);
  float sx = 0, sy = 0, az;
  int n = 0;
  for (int i = 0; i < 50; i++) {
    float x, y;
    if (readAccel(x, y, az)) { sx += x; sy += y; n++; }
    delay(10);
  }
  if (n) { ax0 = sx / n; ay0 = sy / n; }
}

// ---------------- Rendering (two cores) ----------------
uint16_t *fb = nullptr;
SemaphoreHandle_t semStart, semDone;

void renderWorker(void *arg) {
  for (;;) {
    xSemaphoreTake(semStart, portMAX_DELAY);
    globe_render_rows(fb, 0, FB_H / 2);      // top half on core 0
    xSemaphoreGive(semDone);
  }
}

// ---------------- Globe state ----------------
float G[3][3];          // world -> view rotation
float spinVx = 0, spinVy = 0;   // current surface speed on screen (rad/s), x right / y down

void randomView() {
  float lon = (float)random(-1800, 1800) / 10.0f;
  float lat = (float)random(-650, 650) / 10.0f;
  gl_from_lonlat(G, lon, lat);
  spinVx = spinVy = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  if (!gfx->begin()) {
    Serial.println("Display init failed");
    while (true) delay(1000);
  }
  gfx->fillScreen(BLACK);
  centerText("Loading Earth", 108, 2, WHITE);

  if (!imuInit()) {
    gfx->fillScreen(BLACK);
    centerText("IMU not found", 110, 2, RED);
    Serial.println("QMI8658 not found");
    while (true) delay(1000);
  }

  // Frame buffer in fast internal RAM (falls back to PSRAM)
  fb = (uint16_t *)heap_caps_malloc(FB_W * FB_H * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!fb) fb = (uint16_t *)heap_caps_malloc(FB_W * FB_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  // Texture level 0 (copy to PSRAM if there is any, otherwise read straight from flash) and mip levels
  uint16_t *lvl0 = nullptr, *lvl1 = nullptr, *lvl2 = nullptr;
  if (psramFound()) {
    lvl0 = (uint16_t *)ps_malloc(TEX_W * TEX_H * 2);
    lvl1 = (uint16_t *)ps_malloc(TEX_W * TEX_H / 2);
    lvl2 = (uint16_t *)ps_malloc(TEX_W * TEX_H / 8);
    if (lvl0) memcpy(lvl0, earth_tex, TEX_W * TEX_H * 2);
  }
  if (!lvl0) lvl0 = (uint16_t *)earth_tex;               // read-only, used directly from flash
  if (!lvl1) lvl1 = (uint16_t *)malloc(TEX_W * TEX_H / 2);
  if (!lvl2) lvl2 = (uint16_t *)malloc(TEX_W * TEX_H / 8);
  if (!fb || !lvl1 || !lvl2) {
    gfx->fillScreen(BLACK);
    centerText("Out of memory", 110, 2, RED);
    Serial.println("Allocation failed (enable PSRAM in Tools menu)");
    while (true) delay(1000);
  }
  Serial.printf("PSRAM: %s, free heap: %u\n", psramFound() ? "yes" : "no", (unsigned)ESP.getFreeHeap());

  globe_init(lvl0, lvl1, lvl2);

  semStart = xSemaphoreCreateBinary();
  semDone = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(renderWorker, "render", 6144, NULL, 1, NULL, 0);

  randomSeed(esp_random());
  calibrateLevel();
  randomView();
  gfx->fillScreen(BLACK);
}

uint32_t lastUs = 0, fpsMs = 0, pressStart = 0;
int frames = 0;
bool btnWasDown = false;

void loop() {
  // --- BOOT button: short = new random place, long = recalibrate ---
  bool down = (digitalRead(PIN_BOOT) == LOW);
  if (down && !btnWasDown) pressStart = millis();
  if (!down && btnWasDown) {
    if (millis() - pressStart > 1000) calibrateLevel();
    randomView();
    lastUs = micros();
  }
  btnWasDown = down;

  // --- time step ---
  uint32_t now = micros();
  float dt = (now - lastUs) / 1e6f;
  lastUs = now;
  if (dt > 0.1f) dt = 0.1f;

  // --- tilt -> target spin ---
  float tx = 0, ty = 0;
  float ax, ay, az;
  if (readAccel(ax, ay, az)) {
    float gx = ax - ax0, gy = ay - ay0;
#if SWAP_XY
    float t = gx; gx = gy; gy = t;
#endif
#if INVERT_X
    gx = -gx;
#endif
#if INVERT_Y
    gy = -gy;
#endif
    if (fabsf(gx) < DEADZONE) gx = 0;
    if (fabsf(gy) < DEADZONE) gy = 0;
    tx = gx * SPIN_GAIN;
    ty = gy * SPIN_GAIN;
    float sp = sqrtf(tx * tx + ty * ty);
    if (sp > MAX_SPIN) { tx *= MAX_SPIN / sp; ty *= MAX_SPIN / sp; }
  }
  float k = 1.0f - expf(-dt / SPIN_TAU);                 // smooth approach to the target speed
  spinVx += (tx - spinVx) * k;
  spinVy += (ty - spinVy) * k;

  // --- rotate the planet ---
  gl_spin(G, spinVx, spinVy, dt);
  globe_set_view(G);

  // --- render: stars + top half on core 0, bottom half here ---
  globe_draw_stars(fb);
  xSemaphoreGive(semStart);
  globe_render_rows(fb, FB_H / 2, FB_H);
  xSemaphoreTake(semDone, portMAX_DELAY);

  // --- push to the display ---
  gfx->draw16bitRGBBitmap(0, 0, fb, FB_W, FB_H);

#if DEBUG_FPS
  frames++;
  if (millis() - fpsMs >= 2000) {
    Serial.printf("%.1f fps\n", frames * 1000.0f / (millis() - fpsMs));
    frames = 0;
    fpsMs = millis();
  }
#endif
}
