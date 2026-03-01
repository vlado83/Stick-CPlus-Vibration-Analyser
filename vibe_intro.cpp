// Tilt-biased random walkers (N walkers, OLD colors, rotation(1), dissolve fade)
// M5StickC Plus + M5Unified
//
// Your requests implemented:
// - M5.Display.setRotation(1); (IMU mapping assumed correct for this)
// - OLD colormap (black -> purple -> red -> orange/yellow -> white)
// - Optional fading trails via "dissolve": M random black dots per frame (no scanline)
// - Constant speed (STEP_PX, FPS_MS fixed)
// - Label drawn ONCE (never redrawn)
// - All walkers same personality (same bias k)
//
// Controls:
// - BtnA / BtnB: exit intro
// - Shake: clear + reseed (label stays)

#include "vibe_intro.h"
#include <M5Unified.h>
#include <math.h>

// ---------- user options ----------
static constexpr int   N_WALKERS     = 64;    // set N here
static constexpr int   STEP_PX       = 5;
static constexpr int   FPS_MS        = 30;    // constant stepping rate
static constexpr int   BAR_H         = 18;

// Fade option: dissolve with random black dots
static constexpr bool  FADE_TRAILS   = true;  // false => permanent trails
static constexpr int   DISSOLVE_DOTS = 140;   // M dots per frame (40..300 typical)
static constexpr int   DOT_SIZE      = 1;     // 1 or 2 (2 erases faster)

// IMU smoothing
static constexpr float LPF_A         = 0.15f;

// Strong bias: weights = exp(k * dot)
static constexpr float K_MIN         = 0.2f;
static constexpr float K_MAX         = 10.0f; // stronger follow with tilt (try 18..22 if desired)
static constexpr float EPS_W         = 1e-4f;

// ---------- types ----------
struct Walker { int16_t x, y; uint16_t col; };
static Walker w[N_WALKERS];

// ---------- RNG (xorshift32) ----------
static uint32_t rngState = 0xA5F01EEDu;
static inline uint32_t xrnd() {
  uint32_t x = rngState;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  rngState = x;
  return x;
}
static inline int32_t irand(int32_t lo, int32_t hi) { return lo + (int32_t)(xrnd() % (uint32_t)(hi - lo)); }
static inline float frand01() { return (float)(xrnd() & 0xFFFF) / 65535.0f; }

// 8 directions
static constexpr int8_t DIRS[8][2] = {
  { 1, 0}, {-1, 0}, { 0, 1}, { 0,-1},
  { 1, 1}, { 1,-1}, {-1, 1}, {-1,-1}
};

// ---------- colormap helpers ----------
static inline void lerpRGB(float t,
                           uint8_t r0, uint8_t g0, uint8_t b0,
                           uint8_t r1, uint8_t g1, uint8_t b1,
                           uint8_t &r, uint8_t &g, uint8_t &b) {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  r = (uint8_t)(r0 + (r1 - r0) * t + 0.5f);
  g = (uint8_t)(g0 + (g1 - g0) * t + 0.5f);
  b = (uint8_t)(b0 + (b1 - b0) * t + 0.5f);
}

// OLD palette you preferred:
static uint16_t ironhot565(float t) {
  // black -> deep purple -> red -> orange/yellow -> white
  const float  T0 = 0.00f; const uint8_t R0=0,   G0=0,   B0=0;
  const float  T1 = 0.25f; const uint8_t R1=70,  G1=0,   B1=120;   // purple
  const float  T2 = 0.50f; const uint8_t R2=220, G2=0,   B2=20;    // red
  const float  T3 = 0.75f; const uint8_t R3=255, G3=160, B3=0;     // orange/yellow
  const float  T4 = 1.00f; const uint8_t R4=255, G4=255, B4=255;   // white

  uint8_t r,g,b;
  if (t <= T1) {
    lerpRGB((t - T0) / (T1 - T0), R0,G0,B0, R1,G1,B1, r,g,b);
  } else if (t <= T2) {
    lerpRGB((t - T1) / (T2 - T1), R1,G1,B1, R2,G2,B2, r,g,b);
  } else if (t <= T3) {
    lerpRGB((t - T2) / (T3 - T2), R2,G2,B2, R3,G3,B3, r,g,b);
  } else {
    lerpRGB((t - T3) / (T4 - T3), R3,G3,B3, R4,G4,B4, r,g,b);
  }
  return M5.Display.color565(r, g, b);
}

// ---------- UI ----------
static void drawLabelOnce(int W) {
  M5.Display.fillRect(0, 0, W, BAR_H, TFT_BLACK);
  M5.Display.setCursor(6, 2);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.print("VibeSens 1.15");
}

// ---------- init walkers ----------
static void initWalkers(int W, int H, int yMin, int yMax) {
  for (int i = 0; i < N_WALKERS; ++i) {
    float t = (N_WALKERS <= 1) ? 1.0f : (float)i / (float)(N_WALKERS - 1);
    w[i].col = ironhot565(t);

    w[i].x = (int16_t)irand(0, W);
    w[i].y = (int16_t)irand(yMin, yMax + 1);
    M5.Display.fillCircle(w[i].x, w[i].y, 1, w[i].col);
  }
}

// ---------- direction picker ----------
static int pickDirExp(float bx, float by, float k) {
  float ww[8];
  float sum = 0.0f;

  for (int i = 0; i < 8; ++i) {
    float dx = (float)DIRS[i][0];
    float dy = (float)DIRS[i][1];
    if (dx != 0.0f && dy != 0.0f) { dx *= 0.70710678f; dy *= 0.70710678f; } // normalize diagonals

    float dot = dx * bx + dy * by; // ~[-1..1]
    float wi  = expf(k * dot) + EPS_W;
    ww[i] = wi;
    sum += wi;
  }

  float r = frand01() * sum;
  for (int i = 0; i < 8; ++i) {
    r -= ww[i];
    if (r <= 0.0f) return i;
  }
  return 0;
}

// ---------- dissolve fade ----------
static inline void dissolveDots(int W, int yMin, int yMax) {
  if (!FADE_TRAILS) return;

  // draw M random black dots (or small squares) in the drawing area only
  for (int i = 0; i < DISSOLVE_DOTS; ++i) {
    int x = (int)(xrnd() % (uint32_t)W);
    int y = yMin + (int)(xrnd() % (uint32_t)(yMax - yMin + 1));
    if ((xrnd() & 3) != 0) {
      M5.Display.drawPixel(x, y, TFT_BLACK);
    } else {
      // 2x2 erase block (clamped)
      int w = (x < W - 1) ? 2 : 1;
      int h = (y < yMax) ? 2 : 1;
      M5.Display.fillRect(x, y, w, h, TFT_BLACK);
    }
  }
}

// ---------- intro ----------
void runTiltWalkIntro(uint32_t maxMs) {
  const uint32_t t0 = millis();
  uint32_t last = 0;

  const int W = M5.Display.width();
  const int H = M5.Display.height();
  const int yMin = BAR_H;
  const int yMax = H - 1;

  M5.Display.fillScreen(TFT_BLACK);
  drawLabelOnce(W);
  initWalkers(W, H, yMin, yMax);

  float fax = 0.0f, fay = 0.0f, faz = 1.0f;

  while (true) {
    M5.update();
    if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed()) break;
    if (maxMs && (millis() - t0) > maxMs) break;

    const uint32_t now = millis();
    if (now - last < (uint32_t)FPS_MS) continue;
    last = now;

    // IMU (g)
    float ax, ay, az;
    M5.Imu.getAccelData(&ax, &ay, &az);

    // low-pass
    fax = fax * (1.0f - LPF_A) + ax * LPF_A;
    fay = fay * (1.0f - LPF_A) + ay * LPF_A;
    faz = faz * (1.0f - LPF_A) + az * LPF_A;

    // Shake -> clear + reseed (label stays)
    const float mag2 = ax*ax + ay*ay + az*az;
    if (mag2 > 2.8f) {
      M5.Display.fillRect(0, yMin, W, (yMax - yMin + 1), TFT_BLACK);
      initWalkers(W, H, yMin, yMax);
      continue;
    }

    // Your confirmed mapping for rotation(1):
    float bx =  fay;  // right tilt -> +x bias
    float by =  fax;  // down tilt  -> +y bias

    // normalize bias + magnitude
    float mag = sqrtf(bx*bx + by*by);
    if (mag > 1e-3f) { bx /= mag; by /= mag; }
    if (mag > 1.0f) mag = 1.0f;

    float k = K_MIN + (K_MAX - K_MIN) * mag*mag;

    M5.Display.startWrite();

    // dissolve old trails a bit
    dissolveDots(W, yMin, yMax);

    // step walkers (constant step)
    for (int i = 0; i < N_WALKERS; ++i) {
      int16_t ox = w[i].x;
      int16_t oy = w[i].y;

      int di = pickDirExp(bx, by, k);

      int16_t nx = (int16_t)(ox + DIRS[di][0] * STEP_PX);
      int16_t ny = (int16_t)(oy + DIRS[di][1] * STEP_PX);

      // NO WRAP: reflect if out of bounds
      if (nx < 0 || nx >= W) nx = (int16_t)(ox - DIRS[di][0] * STEP_PX);
      if (ny < yMin || ny > yMax) ny = (int16_t)(oy - DIRS[di][1] * STEP_PX);

      // clamp safety
      if (nx < 0) nx = 0;
      if (nx >= W) nx = (int16_t)(W - 1);
      if (ny < yMin) ny = (int16_t)yMin;
      if (ny > yMax) ny = (int16_t)yMax;

      M5.Display.drawLine(ox, oy, nx, ny, w[i].col);
      M5.Display.fillCircle(nx, ny, 1, w[i].col);

      w[i].x = nx;
      w[i].y = ny;
    }

    M5.Display.endWrite();
  }

  M5.Display.fillScreen(TFT_BLACK);
}

