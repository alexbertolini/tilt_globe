// globe_render.h - software renderer for a shaded, textured Earth (pure C++, no Arduino dependencies)
//
// Pipeline per pixel: sphere normal -> rotate into world -> lon/lat -> mip-mapped bilinear
// texture fetch -> sun lighting (day/night/sunset tint) -> ocean sun-glint -> atmosphere haze
// -> ordered dither to RGB565.
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Optimise this file for speed even if the Arduino core defaults to size optimisation
#pragma GCC push_options
#pragma GCC optimize ("O3")

#define FB_W        240
#define FB_H        240
#define FB_CX       120
#define FB_CY       120
#define GLOBE_R     100                     // globe radius in pixels
#define GLOW_W      12                      // atmosphere halo width in pixels
#define GLOBE_RG    (GLOBE_R + GLOW_W)
#define TEX_W       512                     // level-0 texture size (equirectangular)
#define TEX_H       256
#define MIP_LEVELS  3

// ---------------------------------------------------------------- tables
static float    gl_nzT[GLOBE_R * GLOBE_R + 1];            // sqrt(1 - r^2) by squared pixel radius
static uint8_t  gl_atmT[GLOBE_RG * GLOBE_RG + 1];         // atmosphere alpha (limb haze inside, glow outside)
static uint8_t  gl_mipT[GLOBE_R * GLOBE_R + 1];           // mip level by squared pixel radius
static uint16_t gl_vLUT[4096];                            // asin(y) -> texture row (1/32 texel)
static uint16_t gl_lightR[256], gl_lightG[256], gl_lightB[256];   // lighting by n.L
static uint8_t  gl_specT[256];                            // ocean glint by n.H
static int16_t  gl_halfW[FB_H];                           // half row width of glow disk
static const uint16_t *gl_tex[MIP_LEVELS];
static float    gl_kU[MIP_LEVELS];

static const uint8_t gl_bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

// light direction in view space (x right, y up, z toward viewer) and half vector for glint
static const float GL_LX = -0.50f, GL_LY = 0.42f, GL_LZ = 0.76f;
static float gl_Hx, gl_Hy, gl_Hz;

// atmosphere colour
#define ATM_R 92
#define ATM_G 158
#define ATM_B 255

// ---------------------------------------------------------------- per-frame data
static float gl_G[3][3];                                  // world -> view rotation (rows = view axes)
static float gl_ax0[FB_W], gl_ax1[FB_W], gl_ax2[FB_W];    // G[0][i] * nx
static float gl_lxx[FB_W], gl_hxx[FB_W];                  // nx * Lx, nx * Hx

// ---------------------------------------------------------------- small helpers
static inline float gl_clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
static inline float gl_smooth(float a, float b, float x) {
  float t = gl_clampf((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}
static inline int gl_clampi(int v, int a, int b) { return v < a ? a : (v > b ? b : v); }

// atan2 with ~1e-4 rad error
static inline float gl_atan2(float y, float x) {
  float ax = fabsf(x), ay = fabsf(y);
  float mx = ax > ay ? ax : ay, mn = ax > ay ? ay : ax;
  float a = mn / (mx + 1e-20f);
  float s = a * a;
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  if (ay > ax) r = 1.57079637f - r;
  if (x < 0) r = 3.14159274f - r;
  if (y < 0) r = -r;
  return r;
}

// 565 colour spread so that three channels can be blended with one multiply (weights 0..32)
#define GL_MASK 0x07E0F81FUL
static inline uint32_t gl_spread(uint16_t c) { return ((uint32_t)c | ((uint32_t)c << 16)) & GL_MASK; }
static inline uint32_t gl_lerp(uint32_t a, uint32_t b, uint32_t w) {
  return ((a * (32 - w) + b * w) >> 5) & GL_MASK;
}
static inline uint16_t gl_pack(int r, int g, int b, int t) {
  r += t >> 1; g += t >> 2; b += t >> 1;
  r = gl_clampi(r, 0, 255); g = gl_clampi(g, 0, 255); b = gl_clampi(b, 0, 255);
  return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}
static inline void gl_unpack(uint16_t c, int &r, int &g, int &b) {
  int r5 = (c >> 11) & 31, g6 = (c >> 5) & 63, b5 = c & 31;
  r = (r5 << 3) | (r5 >> 2); g = (g6 << 2) | (g6 >> 4); b = (b5 << 3) | (b5 >> 2);
}

// ---------------------------------------------------------------- rotation helpers
// G rows are the view axes (right, up, toward viewer) expressed in world coordinates.
// World: Y = north pole, lon = atan2(x, z), lat = asin(y).
static void gl_from_lonlat(float G[3][3], float lonDeg, float latDeg) {
  float lo = lonDeg * 0.017453292f, la = latDeg * 0.017453292f;
  float sl = sinf(lo), cl = cosf(lo), sp = sinf(la), cp = cosf(la);
  G[0][0] = cl;        G[0][1] = 0;   G[0][2] = -sl;           // east  -> screen right
  G[1][0] = -sp * sl;  G[1][1] = cp;  G[1][2] = -sp * cl;      // north -> screen up
  G[2][0] = cp * sl;   G[2][1] = sp;  G[2][2] = cp * cl;       // point -> toward viewer
}

static void gl_orthonormalize(float G[3][3]) {
  float *a = G[0], *b = G[1], *c = G[2];
  float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  a[0] /= n; a[1] /= n; a[2] /= n;
  float d = b[0] * a[0] + b[1] * a[1] + b[2] * a[2];
  b[0] -= d * a[0]; b[1] -= d * a[1]; b[2] -= d * a[2];
  n = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  b[0] /= n; b[1] /= n; b[2] /= n;
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

// Spin the globe so its visible surface moves with velocity (vx right, vy down) rad/s for dt seconds.
static void gl_spin(float G[3][3], float vx, float vy, float dt) {
  float wx = vy, wy = vx;                                  // angular velocity axis (view space)
  float w = sqrtf(wx * wx + wy * wy);
  float ang = w * dt;
  if (ang < 1e-6f) return;
  float kx = wx / w, ky = wy / w;                          // kz = 0
  float c = cosf(ang), s = sinf(ang), t = 1.0f - c;
  float Q[3][3] = {
    {c + kx * kx * t,  kx * ky * t,       ky * s},
    {kx * ky * t,      c + ky * ky * t,  -kx * s},
    {-ky * s,          kx * s,            c}
  };
  float N[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      N[i][j] = Q[i][0] * G[0][j] + Q[i][1] * G[1][j] + Q[i][2] * G[2][j];
  memcpy(G, N, sizeof(N));
  gl_orthonormalize(G);
}

// ---------------------------------------------------------------- initialisation
// lvl0 must already hold the TEX_W x TEX_H RGB565 texture; lvl1 / lvl2 receive the mip levels.
static void globe_init(uint16_t *lvl0, uint16_t *lvl1, uint16_t *lvl2) {
  // mip chain (2x2 box filter)
  uint16_t *src = lvl0, *dst[2] = {lvl1, lvl2};
  int w = TEX_W, h = TEX_H;
  for (int l = 0; l < 2; l++) {
    int nw = w / 2, nh = h / 2;
    for (int y = 0; y < nh; y++)
      for (int x = 0; x < nw; x++) {
        int r = 0, g = 0, b = 0;
        for (int dy = 0; dy < 2; dy++)
          for (int dx = 0; dx < 2; dx++) {
            uint16_t c = src[(2 * y + dy) * w + 2 * x + dx];
            r += (c >> 11) & 31; g += (c >> 5) & 63; b += c & 31;
          }
        dst[l][y * nw + x] = (uint16_t)(((r + 2) >> 2 << 11) | ((g + 2) >> 2 << 5) | ((b + 2) >> 2));
      }
    src = dst[l]; w = nw; h = nh;
  }
  gl_tex[0] = lvl0; gl_tex[1] = lvl1; gl_tex[2] = lvl2;
  for (int l = 0; l < MIP_LEVELS; l++) gl_kU[l] = (float)(TEX_W >> l) / 6.28318531f;

  // sphere depth, mip level and atmosphere by squared radius
  for (int r2 = 0; r2 <= GLOBE_R * GLOBE_R; r2++) {
    float rr = (float)r2 / (float)(GLOBE_R * GLOBE_R);
    float nz = sqrtf(fmaxf(0.0f, 1.0f - rr));
    gl_nzT[r2] = nz;
    gl_mipT[r2] = nz > 0.55f ? 0 : (nz > 0.22f ? 1 : 2);
    float a = 0.80f * powf(1.0f - nz, 2.6f) + 0.02f;
    gl_atmT[r2] = (uint8_t)(gl_clampf(a, 0.0f, 1.0f) * 255.0f);
  }
  for (int r2 = GLOBE_R * GLOBE_R + 1; r2 <= GLOBE_RG * GLOBE_RG; r2++) {
    float t = (sqrtf((float)r2) - GLOBE_R) / (float)GLOW_W;
    float a = 0.65f * expf(-4.0f * t) * (1.0f - t);
    gl_atmT[r2] = (uint8_t)(gl_clampf(a, 0.0f, 1.0f) * 255.0f);
  }
  for (int dy = 0; dy < FB_H; dy++) {
    int d = dy - FB_CY;
    int v = GLOBE_RG * GLOBE_RG - d * d;
    gl_halfW[dy] = v > 0 ? (int16_t)sqrtf((float)v) : -1;
  }

  // latitude lookup: y = sin(lat) -> row coordinate in 1/32 texel units (level 0)
  for (int i = 0; i < 4096; i++) {
    float y = ((float)i + 0.5f) / 2048.0f - 1.0f;
    float lat = asinf(gl_clampf(y, -1.0f, 1.0f));
    float row = (0.5f - lat / 3.14159265f) * TEX_H - 0.5f;
    row = gl_clampf(row, 0.0f, (float)(TEX_H - 1));
    gl_vLUT[i] = (uint16_t)(row * 32.0f);
  }

  // lighting: night / day / sunset tint
  for (int i = 0; i < 256; i++) {
    float d = (float)i / 127.5f - 1.0f;
    float f = gl_smooth(-0.10f, 0.34f, d);
    float sun = expf(-powf((d - 0.03f) / 0.11f, 2.0f)) * 0.32f;
    float r = 0.15f * (1 - f) + 1.06f * f + sun * 0.50f;
    float g = 0.21f * (1 - f) + 1.03f * f + sun * 0.16f;
    float b = 0.34f * (1 - f) + 0.98f * f - sun * 0.10f;
    gl_lightR[i] = (uint16_t)(gl_clampf(r, 0.0f, 1.4f) * 256.0f);
    gl_lightG[i] = (uint16_t)(gl_clampf(g, 0.0f, 1.4f) * 256.0f);
    gl_lightB[i] = (uint16_t)(gl_clampf(b, 0.0f, 1.4f) * 256.0f);
  }

  // glint on the ocean
  float hx = GL_LX, hy = GL_LY, hz = GL_LZ + 1.0f;
  float hn = sqrtf(hx * hx + hy * hy + hz * hz);
  gl_Hx = hx / hn; gl_Hy = hy / hn; gl_Hz = hz / hn;
  for (int i = 0; i < 256; i++) {                      // n.H from 0.80 to 1.0 in fine steps
    float h = 0.80f + (float)i / 1275.0f;
    float s = 0.60f * powf(h, 140.0f) + 0.06f * powf(h, 16.0f);
    gl_specT[i] = (uint8_t)(gl_clampf(s, 0.0f, 1.0f) * 255.0f);
  }
}

// Set the orientation for the next frame and precompute the per-column terms.
static void globe_set_view(const float G[3][3]) {
  memcpy(gl_G, G, sizeof(gl_G));
  const float invR = 1.0f / (float)GLOBE_R;
  for (int x = 0; x < FB_W; x++) {
    float nx = (float)(x - FB_CX) * invR;
    gl_ax0[x] = G[0][0] * nx;
    gl_ax1[x] = G[0][1] * nx;
    gl_ax2[x] = G[0][2] * nx;
    gl_lxx[x] = nx * GL_LX;
    gl_hxx[x] = nx * gl_Hx;
  }
}

// ---------------------------------------------------------------- renderer
// Renders rows [y0, y1) of the globe into fb (FB_W x FB_H, RGB565). Safe to call from two cores
// on disjoint row ranges.
static void globe_render_rows(uint16_t *fb, int y0, int y1) {
  const float invR = 1.0f / (float)GLOBE_R;
  const float PI_F = 3.14159265f;
  const float g20 = gl_G[2][0], g21 = gl_G[2][1], g22 = gl_G[2][2];
  const int R2 = GLOBE_R * GLOBE_R;

  for (int y = y0; y < y1; y++) {
    int dy = y - FB_CY;
    int hw = gl_halfW[y];
    if (hw < 0) continue;
    int dy2 = dy * dy;
    float ny = -(float)dy * invR;                           // view-space up
    float b0 = gl_G[1][0] * ny, b1 = gl_G[1][1] * ny, b2 = gl_G[1][2] * ny;
    float lyy = ny * GL_LY, hyy = ny * gl_Hy;
    uint16_t *row = fb + y * FB_W;
    const uint8_t *bay = gl_bayer[y & 3];

    for (int x = FB_CX - hw; x <= FB_CX + hw; x++) {
      int dx = x - FB_CX;
      int r2 = dx * dx + dy2;
      int t = bay[x & 3];
      int a = gl_atmT[r2];

      if (r2 > R2) {                                        // atmosphere glow over the background
        int br, bg, bb;
        gl_unpack(row[x], br, bg, bb);
        float s = gl_lxx[x] + lyy;                          // n.L without the z part
        float day = gl_clampf(0.52f + 0.95f * s, 0.12f, 1.0f);
        int da = (int)(a * day);
        int r = br + (((ATM_R - br) * da) >> 8);
        int g = bg + (((ATM_G - bg) * da) >> 8);
        int b = bb + (((ATM_B - bb) * da) >> 8);
        row[x] = gl_pack(r, g, b, t);
        continue;
      }

      float nz = gl_nzT[r2];
      float p0 = gl_ax0[x] + b0 + g20 * nz;                 // world x
      float p1 = gl_ax1[x] + b1 + g21 * nz;                 // world y (north)
      float p2 = gl_ax2[x] + b2 + g22 * nz;                 // world z

      // texture coordinates
      int lvl = gl_mipT[r2];
      float ay = fabsf(p1);                                 // blur more near the poles
      if (ay > 0.93f) { int pl = ay > 0.985f ? 2 : 1; if (pl > lvl) lvl = pl; }
      float lon = gl_atan2(p0, p2);
      float t1 = (lon + PI_F) * gl_kU[lvl] + 0.5f;          // (u - 0.5) + 1
      int ti = (int)t1;
      int fu = (int)((t1 - (float)ti) * 32.0f);
      int iu = ti - 1;
      int vi = (int)((p1 + 1.0f) * 2047.5f);
      int vv = gl_vLUT[gl_clampi(vi, 0, 4095)] >> lvl;
      int iv = vv >> 5, fv = vv & 31;
      int hmax = (TEX_H >> lvl) - 1;
      int iv1 = iv < hmax ? iv + 1 : hmax;
      int wmask = (TEX_W >> lvl) - 1;
      int shw = 9 - lvl;
      const uint16_t *T = gl_tex[lvl];
      int i0 = iu & wmask, i1 = (iu + 1) & wmask;
      const uint16_t *ra = T + (iv << shw), *rb = T + (iv1 << shw);
      uint32_t top = gl_lerp(gl_spread(ra[i0]), gl_spread(ra[i1]), fu);
      uint32_t bot = gl_lerp(gl_spread(rb[i0]), gl_spread(rb[i1]), fu);
      uint32_t c = gl_lerp(top, bot, fv);
      int r5 = (c >> 11) & 31, g6 = (c >> 21) & 63, b5 = c & 31;
      int r = (r5 << 3) | (r5 >> 2), g = (g6 << 2) | (g6 >> 4), b = (b5 << 3) | (b5 >> 2);

      // sun lighting
      float d = gl_lxx[x] + lyy + nz * GL_LZ;
      int di = gl_clampi((int)((d + 1.0f) * 127.5f), 0, 255);
      int dayAmt = gl_lightG[di];
      r = (r * gl_lightR[di]) >> 8;
      g = (g * dayAmt) >> 8;
      b = (b * gl_lightB[di]) >> 8;

      // ocean glint (water is blue-dominant in the texture)
      int oc = b5 - r5 - 3;
      if (oc > 0) {
        float h = gl_hxx[x] + hyy + nz * gl_Hz;
        if (h > 0.80f) {
          int sp = gl_specT[gl_clampi((int)((h - 0.80f) * 1275.0f), 0, 255)];
          if (oc > 8) oc = 8;
          sp = (sp * oc) >> 3;
          r += sp; g += sp; b += (sp * 7) >> 3;
        }
      }

      // limb haze
      if (a > 3) {
        int ar = (ATM_R * dayAmt) >> 8, ag = (ATM_G * dayAmt) >> 8, ab = (ATM_B * dayAmt) >> 8;
        r += ((ar - r) * a) >> 8;
        g += ((ag - g) * a) >> 8;
        b += ((ab - b) * a) >> 8;
      }
      row[x] = gl_pack(r, g, b, t);
    }
  }
}

// ---------------------------------------------------------------- background stars
static void globe_draw_stars(uint16_t *fb) {
  memset(fb, 0, FB_W * FB_H * sizeof(uint16_t));
  uint32_t s = 12345u;
  int placed = 0;
  while (placed < 110) {
    s = s * 1664525u + 1013904223u;
    int x = (int)((s >> 8) % FB_W);
    s = s * 1664525u + 1013904223u;
    int y = (int)((s >> 8) % FB_H);
    s = s * 1664525u + 1013904223u;
    int br = 70 + (int)((s >> 10) % 186);
    int dx = x - FB_CX, dy = y - FB_CY;
    int r2 = dx * dx + dy * dy;
    if (r2 < (GLOBE_R + 4) * (GLOBE_R + 4) || r2 > 118 * 118) continue;
    int tint = (int)((s >> 4) & 3);
    int r = br - (tint == 0 ? 25 : 0), g = br - (tint == 1 ? 10 : 0), b = br + (tint == 2 ? 0 : -8);
    uint16_t c = gl_pack(r, g, b, 0);
    fb[y * FB_W + x] = c;
    if (br > 215 && x + 1 < FB_W) fb[y * FB_W + x + 1] = gl_pack(r / 2, g / 2, b / 2, 0);
    placed++;
  }
}

#pragma GCC pop_options
