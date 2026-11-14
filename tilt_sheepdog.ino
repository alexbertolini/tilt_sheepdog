/*
  Sheepdog Trials - Waveshare ESP32-S3-LCD-1.28 (GC9A01 round LCD + QMI8658 IMU)

  Seen from above: a green field, a small wooden pen with one open side (the gate), and ten sheep.
  Two of the sheep are marked with a red patch. Steer the border collie by tilting the board and
  drive the TWO RED-MARKED sheep into the pen to win. The white sheep may wander in too, it does not matter.

  The sheep run away from the dog, but they are a flock: they are drawn toward each other (a sort of
  gravity), follow the ones around them and keep a small gap. Far from the dog they graze quietly.

  BOOT button: short press = new game, hold ~1 s = recalibrate "level"
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "esp_heap_caps.h"

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
#define DEBUG_SERIAL 1

// ---------------- Dog ----------------
const float DOG_SPEED   = 80.0f;   // px/s at full tilt
const float TILT_FULL_G = 0.35f;   // tilt (g) that gives full speed
const float DEADZONE    = 0.03f;   // g
const float DOG_TAU     = 0.10f;   // s, how quickly the dog changes speed
const float DOG_R       = 3.5f;    // collision radius

// ---------------- Sheep ----------------
#define NS 10                       // number of sheep (the first two are the marked ones)
const float SHEEP_R     = 3.0f;
const float FLEE_R      = 46.0f;    // the dog starts to worry them from this distance
const float PANIC_R     = 16.0f;    // ... and they really run from this distance
const float F_FLEE      = 380.0f;   // flee acceleration (px/s^2) at point-blank range
const float NEIGH_R     = 30.0f;    // sheep within this radius count as neighbours
const float SEP_R       = 9.0f;     // minimum comfortable gap between sheep
const float K_COH       = 0.5f;     // gravity toward the neighbours' centre (1/s^2)
const float K_ALIGN     = 0.8f;     // how strongly they copy their neighbours' motion
const float K_SEP       = 160.0f;   // push apart when too close
const float V_FLEE      = 58.0f;    // top speed when running
const float V_CALM      = 13.0f;    // top speed when grazing

// ---------------- Field and pen ----------------
const float FIELD_R     = 112.0f;   // sheep and dog stay inside this radius (the hedge is just beyond)
const float PEN_HALF    = 14.0f;    // half size of the pen interior
const float PEN_WALL    = 2.0f;     // wall thickness
const float GATE_HALF   = 8.0f;     // half width of the opening

#define C565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
const uint16_t COL_SHEEP   = C565(250, 250, 245);
const uint16_t COL_SHEEP_S = C565(214, 216, 210);   // shaded underside
const uint16_t COL_HEAD    = C565(52, 46, 46);
const uint16_t COL_MARK    = C565(214, 28, 28);
const uint16_t COL_DOG     = C565(22, 22, 26);
const uint16_t COL_DOGW    = C565(248, 248, 248);
const uint16_t COL_STRAW   = C565(206, 178, 100);
const uint16_t COL_WALL    = C565(112, 72, 40);
const uint16_t COL_WALL_L  = C565(150, 104, 62);
const uint16_t COL_POST    = C565(70, 44, 24);

// ---------------- Display ----------------
Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, PIN_RST, 0 /*rotation*/, true /*IPS*/);

const int CX = 120, CY = 120;

// ---------------- Boot-screen logo (webwarrior.org, 86x13 pixel art, RGB565), drawn unscaled ----------------
#define LOGO_W 86
#define LOGO_H 13
const uint16_t logo_ww[LOGO_W * LOGO_H] PROGMEM = {
0x5B4B,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,
0x636C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xF800,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF800,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF800,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xF800,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0xFFFF,0xFFFF,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8C6F,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0x8C6F,0xFFFF,0xF800,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF800,0xFFFF,0xF800,0xF800,0xF800,0xFFFF,0xF800,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0x8C6F,0xFFFF,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xF800,0xFFFF,0x636C,
0x636C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x636C,
0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C,0x636C
};

// Draws the logo at its native size, centred horizontally on cx with its top edge at y.
// Runs of equal colour are drawn as one rectangle, so it only takes a few milliseconds.
void drawLogo(int cx, int y) {
  int x0 = cx - LOGO_W / 2;
  for (int j = 0; j < LOGO_H; j++) {
    int i = 0;
    while (i < LOGO_W) {
      uint16_t c = logo_ww[j * LOGO_W + i];
      int k = i + 1;
      while (k < LOGO_W && logo_ww[j * LOGO_W + k] == c) k++;
      gfx->fillRect(x0 + i, y + j, k - i, 1, c);
      i = k;
    }
  }
}


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
  gfx->setCursor(CX - w / 2, y);
  gfx->print(txt);
}

void calibrateLevel() {
  gfx->fillScreen(BLACK);
  drawLogo(CX, 98);                        // logo at its native size, above the message
  centerText("Hold level", 124, 2, WHITE);
  delay(1500);
  float sx_ = 0, sy_ = 0, az;
  int n = 0;
  for (int i = 0; i < 50; i++) {
    float x, y;
    if (readAccel(x, y, az)) { sx_ += x; sy_ += y; n++; }
    delay(10);
  }
  if (n) { ax0 = sx_ / n; ay0 = sy_ / n; }
}

// ======================= game state =======================
uint16_t *fb = nullptr;
uint16_t grassTile[64 * 64];
uint16_t ringIdx[3200];
int ringCount = 0;

float dogx, dogy, dogvx, dogvy;
int   dogfx = 1, dogfy = 0;               // axis the dog is facing
float sx[NS], sy[NS], svx[NS], svy[NS];
float swx[NS], swy[NS], swt[NS];           // grazing wander target velocity and timer
int8_t shx[NS], shy[NS];                   // heading (for the head pixel)
float penX, penY;
int   gateDir;                             // 0 = north, 1 = east, 2 = south, 3 = west
float wall[6][4];                          // wall rectangles x0, y0, x1, y1
int   nwall = 0;
int   state = 0;                           // 0 = playing, 1 = won
uint32_t startMs = 0, winMs = 0;
float inPenTime = 0;
uint32_t lastUs = 0;
uint32_t pressStart = 0;
bool btnWasDown = false;
uint32_t fpsMs = 0;
int frames = 0;

float frand() { return (float)(esp_random() & 0xFFFFFF) / 16777216.0f; }
float frange(float a, float b) { return a + (b - a) * frand(); }

// ---------------- small drawing helpers (into the frame buffer) ----------------
inline void putPx(int x, int y, uint16_t c) {
  if ((unsigned)x < 240u && (unsigned)y < 240u) fb[y * 240 + x] = c;
}
void fillR(int x0, int y0, int w, int h, uint16_t c) {
  for (int y = y0; y < y0 + h; y++)
    for (int x = x0; x < x0 + w; x++) putPx(x, y, c);
}

void drawSheep(int i) {
  int cx = (int)lroundf(sx[i]), cy = (int)lroundf(sy[i]);
  // body: a 5x5 blob with rounded corners
  for (int dy = -2; dy <= 2; dy++) {
    int hw = (dy == -2 || dy == 2) ? 1 : 2;
    uint16_t c = (dy == 2) ? COL_SHEEP_S : COL_SHEEP;
    for (int dx = -hw; dx <= hw; dx++) putPx(cx + dx, cy + dy, c);
  }
  if (i < 2) fillR(cx - 1, cy - 1, 3, 3, COL_MARK);                  // the red mark
  // head: 2x2 in the direction of travel
  int hx = shx[i] > 0 ? cx + 3 : (shx[i] < 0 ? cx - 4 : cx - 1);
  int hy = shy[i] > 0 ? cy + 3 : (shy[i] < 0 ? cy - 4 : cy - 1);
  fillR(hx, hy, 2, 2, COL_HEAD);
}

void drawDog() {
  int cx = (int)lroundf(dogx), cy = (int)lroundf(dogy);
  int fx = dogfx, fy = dogfy, px = -dogfy, py = dogfx;               // forward and sideways unit vectors
  // body (black), white collar, head with a white blaze, white tail tip
  for (int f = -4; f <= -1; f++) for (int s = -1; s <= 1; s++) putPx(cx + f * fx + s * px, cy + f * fy + s * py, COL_DOG);
  putPx(cx - 2 * fx, cy - 2 * fy, COL_DOGW);                          // white back patch
  for (int s = -1; s <= 1; s++) putPx(cx + s * px, cy + s * py, COL_DOGW);                      // collar
  for (int f = 1; f <= 3; f++) for (int s = -1; s <= 1; s++) putPx(cx + f * fx + s * px, cy + f * fy + s * py, COL_DOG);
  putPx(cx + 1 * fx + 2 * px, cy + 1 * fy + 2 * py, COL_DOG);         // ears
  putPx(cx + 1 * fx - 2 * px, cy + 1 * fy - 2 * py, COL_DOG);
  putPx(cx + 2 * fx, cy + 2 * fy, COL_DOGW);                          // blaze
  putPx(cx - 5 * fx, cy - 5 * fy, COL_DOGW);                          // tail tip
  putPx(cx - 6 * fx, cy - 6 * fy, COL_DOGW);
}

void drawPen() {
  int x0 = (int)lroundf(penX - PEN_HALF), y0 = (int)lroundf(penY - PEN_HALF);
  int n = (int)(PEN_HALF * 2);
  fillR(x0, y0, n, n, COL_STRAW);                                     // straw floor
  for (int w = 0; w < nwall; w++) {
    int a = (int)lroundf(wall[w][0]), b = (int)lroundf(wall[w][1]);
    int c = (int)lroundf(wall[w][2]), d = (int)lroundf(wall[w][3]);
    fillR(a, b, c - a, d - b, COL_WALL);
    if (c - a > d - b) fillR(a, b, c - a, 1, COL_WALL_L);            // light edge for a little depth
    else fillR(a, b, 1, d - b, COL_WALL_L);
  }
  // gate posts
  float gx = penX, gy = penY;
  if (gateDir == 0) gy -= PEN_HALF + PEN_WALL * 0.5f;
  if (gateDir == 2) gy += PEN_HALF + PEN_WALL * 0.5f;
  if (gateDir == 1) gx += PEN_HALF + PEN_WALL * 0.5f;
  if (gateDir == 3) gx -= PEN_HALF + PEN_WALL * 0.5f;
  float ex = (gateDir == 0 || gateDir == 2) ? GATE_HALF : 0, ey = (gateDir == 1 || gateDir == 3) ? GATE_HALF : 0;
  fillR((int)lroundf(gx - ex) - 1, (int)lroundf(gy - ey) - 1, 3, 3, COL_POST);
  fillR((int)lroundf(gx + ex) - 1, (int)lroundf(gy + ey) - 1, 3, 3, COL_POST);
}

bool inPen(float x, float y) { return fabsf(x - penX) < PEN_HALF - 1.0f && fabsf(y - penY) < PEN_HALF - 1.0f; }

void drawHud() {
  for (int k = 0; k < 2; k++) {                                       // one box per marked sheep: filled when it is in the pen
    int x = 111 + k * 12, y = 5;
    bool in = inPen(sx[k], sy[k]);
    fillR(x, y, 7, 7, C565(20, 60, 25));
    fillR(x + 1, y + 1, 5, 5, in ? COL_MARK : C565(60, 110, 62));
    if (!in) fillR(x + 2, y + 2, 3, 3, C565(30, 80, 36));
  }
}

void drawField() {
  for (int y = 0; y < 240; y++) {
    uint16_t *row = fb + y * 240;
    const uint16_t *t = grassTile + (y & 63) * 64;
    for (int x = 0; x < 240; x += 64) memcpy(row + x, t, (x + 64 <= 240 ? 64 : 240 - x) * 2);
  }
  for (int i = 0; i < ringCount; i++) {                               // hedge around the field
    int idx = ringIdx[i];
    fb[idx] = (((idx % 240) + (idx / 240)) & 1) ? C565(24, 78, 34) : C565(38, 100, 46);
  }
}

// ---------------- collisions: circle against axis-aligned wall rectangles ----------------
void collideWalls(float &x, float &y, float &vx, float &vy, float r) {
  for (int w = 0; w < nwall; w++) {
    float x0 = wall[w][0], y0 = wall[w][1], x1 = wall[w][2], y1 = wall[w][3];
    float cx = x < x0 ? x0 : (x > x1 ? x1 : x), cy = y < y0 ? y0 : (y > y1 ? y1 : y);
    float dx = x - cx, dy = y - cy, d2 = dx * dx + dy * dy;
    if (d2 >= r * r) continue;
    float nx, ny, pen;
    if (d2 > 1e-6f) { float d = sqrtf(d2); nx = dx / d; ny = dy / d; pen = r - d; }
    else {                                                            // centre is inside the rectangle: leave by the nearest face
      float l = x - x0, rr = x1 - x, t = y - y0, b = y1 - y;
      float m = fminf(fminf(l, rr), fminf(t, b));
      nx = ny = 0;
      if (m == l) nx = -1; else if (m == rr) nx = 1; else if (m == t) ny = -1; else ny = 1;
      pen = m + r;
    }
    x += nx * pen; y += ny * pen;
    float vn = vx * nx + vy * ny;
    if (vn < 0) { vx -= vn * nx; vy -= vn * ny; }                    // slide along the wall
  }
}

void collideField(float &x, float &y, float &vx, float &vy) {
  float dx = x - CX, dy = y - CY, r = sqrtf(dx * dx + dy * dy);
  if (r > FIELD_R) {
    float nx = dx / r, ny = dy / r;
    x = CX + nx * FIELD_R; y = CY + ny * FIELD_R;
    float vn = vx * nx + vy * ny;
    if (vn > 0) { vx -= vn * nx; vy -= vn * ny; }
  }
}

// ---------------- setting up a game ----------------
void buildWalls() {
  nwall = 0;
  float o = PEN_HALF + PEN_WALL;                                      // outer half size
  float L = penX - o, R = penX + o, T = penY - o, B = penY + o;
  float il = penX - PEN_HALF, ir = penX + PEN_HALF, it = penY - PEN_HALF, ib = penY + PEN_HALF;
  // north side
  if (gateDir == 0) { wall[nwall][0] = L; wall[nwall][1] = T; wall[nwall][2] = penX - GATE_HALF; wall[nwall][3] = it; nwall++;
                      wall[nwall][0] = penX + GATE_HALF; wall[nwall][1] = T; wall[nwall][2] = R; wall[nwall][3] = it; nwall++; }
  else { wall[nwall][0] = L; wall[nwall][1] = T; wall[nwall][2] = R; wall[nwall][3] = it; nwall++; }
  // south side
  if (gateDir == 2) { wall[nwall][0] = L; wall[nwall][1] = ib; wall[nwall][2] = penX - GATE_HALF; wall[nwall][3] = B; nwall++;
                      wall[nwall][0] = penX + GATE_HALF; wall[nwall][1] = ib; wall[nwall][2] = R; wall[nwall][3] = B; nwall++; }
  else { wall[nwall][0] = L; wall[nwall][1] = ib; wall[nwall][2] = R; wall[nwall][3] = B; nwall++; }
  // west and east sides (between the north and south walls)
  if (gateDir == 3) { wall[nwall][0] = L; wall[nwall][1] = it; wall[nwall][2] = il; wall[nwall][3] = penY - GATE_HALF; nwall++;
                      wall[nwall][0] = L; wall[nwall][1] = penY + GATE_HALF; wall[nwall][2] = il; wall[nwall][3] = ib; nwall++; }
  else { wall[nwall][0] = L; wall[nwall][1] = it; wall[nwall][2] = il; wall[nwall][3] = ib; nwall++; }
  if (gateDir == 1) { wall[nwall][0] = ir; wall[nwall][1] = it; wall[nwall][2] = R; wall[nwall][3] = penY - GATE_HALF; nwall++;
                      wall[nwall][0] = ir; wall[nwall][1] = penY + GATE_HALF; wall[nwall][2] = R; wall[nwall][3] = ib; nwall++; }
  else { wall[nwall][0] = ir; wall[nwall][1] = it; wall[nwall][2] = R; wall[nwall][3] = ib; nwall++; }
}

bool nearPen(float x, float y, float margin) {
  float o = PEN_HALF + PEN_WALL + margin;
  return fabsf(x - penX) < o && fabsf(y - penY) < o;
}

void newGame() {
  // pen: random place with room in front of the gate
  for (int tries = 0; tries < 200; tries++) {
    float a = frange(0, 6.2831853f), r = frange(28.0f, 76.0f);
    penX = CX + cosf(a) * r; penY = CY + sinf(a) * r;
    gateDir = (int)(frand() * 4.0f); if (gateDir > 3) gateDir = 3;
    float gx = penX + (gateDir == 1 ? 1 : (gateDir == 3 ? -1 : 0)) * (PEN_HALF + 20.0f);
    float gy = penY + (gateDir == 2 ? 1 : (gateDir == 0 ? -1 : 0)) * (PEN_HALF + 20.0f);
    float rg = sqrtf((gx - CX) * (gx - CX) + (gy - CY) * (gy - CY));
    if (rg < FIELD_R - 6.0f) break;
  }
  buildWalls();

  // flock: somewhere away from the pen
  float fx_ = CX, fy_ = CY;
  for (int tries = 0; tries < 200; tries++) {
    float a = frange(0, 6.2831853f), r = frange(10.0f, 70.0f);
    fx_ = CX + cosf(a) * r; fy_ = CY + sinf(a) * r;
    float d = sqrtf((fx_ - penX) * (fx_ - penX) + (fy_ - penY) * (fy_ - penY));
    if (d > 62.0f) break;
  }
  for (int i = 0; i < NS; i++) {
    for (int tries = 0; tries < 100; tries++) {
      float a = frange(0, 6.2831853f), r = frange(0, 20.0f);
      float x = fx_ + cosf(a) * r, y = fy_ + sinf(a) * r;
      float rr = sqrtf((x - CX) * (x - CX) + (y - CY) * (y - CY));
      if (rr > FIELD_R - 8.0f || nearPen(x, y, 6.0f)) continue;
      bool ok = true;
      for (int j = 0; j < i; j++) if ((sx[j] - x) * (sx[j] - x) + (sy[j] - y) * (sy[j] - y) < 49.0f) ok = false;
      if (!ok) continue;
      sx[i] = x; sy[i] = y; break;
    }
    svx[i] = svy[i] = 0; swx[i] = swy[i] = 0; swt[i] = frange(0.2f, 2.5f);
    shx[i] = (frand() < 0.5f) ? -1 : 1; shy[i] = 0;
  }
  // dog: far from the flock and the pen
  for (int tries = 0; tries < 200; tries++) {
    float a = frange(0, 6.2831853f), r = frange(20.0f, 90.0f);
    dogx = CX + cosf(a) * r; dogy = CY + sinf(a) * r;
    float d1 = sqrtf((dogx - fx_) * (dogx - fx_) + (dogy - fy_) * (dogy - fy_));
    if (d1 > 75.0f && !nearPen(dogx, dogy, 8.0f)) break;
  }
  dogvx = dogvy = 0; dogfx = 1; dogfy = 0;
  inPenTime = 0; state = 0; startMs = millis();
}

// ---------------- the flock ----------------
void updateSheep(float dt) {
  for (int i = 0; i < NS; i++) {
    float ax = 0, ay = 0;
    // run away from the dog
    float ddx = sx[i] - dogx, ddy = sy[i] - dogy;
    float dd = sqrtf(ddx * ddx + ddy * ddy) + 0.001f;
    bool threatened = dd < FLEE_R;
    if (threatened) {
      float f = 1.0f - dd / FLEE_R;
      f = f * f * (dd < PANIC_R ? 2.5f : 1.0f);
      ax += ddx / dd * F_FLEE * f;
      ay += ddy / dd * F_FLEE * f;
    }
    // the flock: gravity toward neighbours, copy their motion, keep a small gap
    float cxs = 0, cys = 0, vxs = 0, vys = 0; int cnt = 0;
    for (int j = 0; j < NS; j++) {
      if (j == i) continue;
      float dx = sx[j] - sx[i], dy = sy[j] - sy[i];
      float d2 = dx * dx + dy * dy;
      if (d2 > NEIGH_R * NEIGH_R) continue;
      cnt++; cxs += sx[j]; cys += sy[j]; vxs += svx[j]; vys += svy[j];
      float d = sqrtf(d2) + 0.001f;
      if (d < SEP_R) {
        float push = (SEP_R - d) / SEP_R;
        ax -= dx / d * K_SEP * push;
        ay -= dy / d * K_SEP * push;
      }
    }
    if (cnt) {
      float inv = 1.0f / cnt;
      ax += (cxs * inv - sx[i]) * K_COH;
      ay += (cys * inv - sy[i]) * K_COH;
      ax += (vxs * inv - svx[i]) * K_ALIGN;
      ay += (vys * inv - svy[i]) * K_ALIGN;
    }
    // grazing: mostly standing, sometimes strolling a few steps
    if (!threatened) {
      swt[i] -= dt;
      if (swt[i] <= 0) {
        swt[i] = frange(1.5f, 4.5f);
        if (frand() < 0.55f) { swx[i] = swy[i] = 0; }
        else { float a = frange(0, 6.2831853f), sp = frange(5.0f, 11.0f); swx[i] = cosf(a) * sp; swy[i] = sinf(a) * sp; }
      }
      ax += (swx[i] - svx[i]) * 2.0f;
      ay += (swy[i] - svy[i]) * 2.0f;
    }
    // stay away from the hedge
    float ex = sx[i] - CX, ey = sy[i] - CY, er = sqrtf(ex * ex + ey * ey) + 0.001f;
    if (er > FIELD_R - 14.0f) {
      float f = (er - (FIELD_R - 14.0f)) / 14.0f;
      ax -= ex / er * 90.0f * f; ay -= ey / er * 90.0f * f;
    }
    svx[i] += ax * dt; svy[i] += ay * dt;
    float drag = expf(-(threatened ? 0.5f : 2.2f) * dt);
    svx[i] *= drag; svy[i] *= drag;
    float sp = sqrtf(svx[i] * svx[i] + svy[i] * svy[i]);
    float vmax = threatened ? V_FLEE : V_CALM;
    if (sp > vmax) { svx[i] *= vmax / sp; svy[i] *= vmax / sp; sp = vmax; }
    sx[i] += svx[i] * dt; sy[i] += svy[i] * dt;
    collideField(sx[i], sy[i], svx[i], svy[i]);
    collideWalls(sx[i], sy[i], svx[i], svy[i], SHEEP_R);
    if (sp > 4.0f) {                                                  // heading for the head pixel (8 directions)
      shx[i] = svx[i] > 0.38f * sp ? 1 : (svx[i] < -0.38f * sp ? -1 : 0);
      shy[i] = svy[i] > 0.38f * sp ? 1 : (svy[i] < -0.38f * sp ? -1 : 0);
      if (shx[i] == 0 && shy[i] == 0) shx[i] = 1;
    }
  }
}

// ---------------- setup / loop ----------------
void showWin() {
  uint32_t secs = (winMs - startMs) / 1000;
  char buf[24];
  gfx->fillRect(30, 86, 180, 70, BLACK);
  centerText("WELL DONE!", 94, 2, WHITE);
  snprintf(buf, sizeof(buf), "%lu s", (unsigned long)secs);
  centerText(buf, 118, 2, C565(120, 230, 120));
  centerText("BOOT = play again", 142, 1, C565(180, 180, 180));
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

  if (!imuInit()) {
    centerText("IMU not found", 110, 2, RED);
    Serial.println("QMI8658 not found");
    while (true) delay(1000);
  }

  fb = (uint16_t *)heap_caps_malloc(240 * 240 * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!fb) fb = (uint16_t *)heap_caps_malloc(240 * 240 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fb) {
    centerText("Out of memory", 110, 2, RED);
    while (true) delay(1000);
  }

  // grass tile: plain green with a few darker and lighter blades
  uint32_t s = 12345;
  for (int i = 0; i < 64 * 64; i++) {
    s = s * 1664525u + 1013904223u;
    int v = (s >> 16) & 255;
    uint16_t c = C565(58, 142, 62);
    if (v < 14) c = C565(46, 120, 52);
    else if (v < 22) c = C565(72, 160, 72);
    grassTile[i] = c;
  }
  // hedge ring (pixels between radius 117 and 120)
  for (int y = 0; y < 240; y++)
    for (int x = 0; x < 240; x++) {
      float dx = x + 0.5f - CX, dy = y + 0.5f - CY, r = sqrtf(dx * dx + dy * dy);
      if (r >= 116.5f && r < 120.0f && ringCount < 3200) ringIdx[ringCount++] = (uint16_t)(y * 240 + x);
    }

  randomSeed(esp_random());
  calibrateLevel();
  newGame();
  lastUs = micros();
}

void loop() {
  // --- BOOT button: short = new game, long = recalibrate ---
  bool down = (digitalRead(PIN_BOOT) == LOW);
  if (down && !btnWasDown) pressStart = millis();
  if (!down && btnWasDown) {
    if (millis() - pressStart > 1000) calibrateLevel();
    newGame();
    lastUs = micros();
  }
  btnWasDown = down;

  if (state == 1) {                                                    // won: wait for BOOT or a few seconds
    if (millis() - winMs > 6000) { newGame(); lastUs = micros(); }
    delay(20);
    return;
  }

  uint32_t now = micros();
  float dt = (now - lastUs) / 1e6f;
  lastUs = now;
  if (dt > 0.05f) dt = 0.05f;

  // --- dog: tilt -> velocity ---
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
    tx = gx / TILT_FULL_G; ty = gy / TILT_FULL_G;
    float m = sqrtf(tx * tx + ty * ty);
    if (m > 1.0f) { tx /= m; ty /= m; }
    tx *= DOG_SPEED; ty *= DOG_SPEED;
  }
  float k = 1.0f - expf(-dt / DOG_TAU);
  dogvx += (tx - dogvx) * k;
  dogvy += (ty - dogvy) * k;
  dogx += dogvx * dt; dogy += dogvy * dt;
  collideField(dogx, dogy, dogvx, dogvy);
  collideWalls(dogx, dogy, dogvx, dogvy, DOG_R);
  float dsp = sqrtf(dogvx * dogvx + dogvy * dogvy);
  if (dsp > 8.0f) {
    if (fabsf(dogvx) > fabsf(dogvy)) { dogfx = dogvx > 0 ? 1 : -1; dogfy = 0; }
    else { dogfy = dogvy > 0 ? 1 : -1; dogfx = 0; }
  }

  updateSheep(dt);

  // --- win: both marked sheep in the pen for a moment ---
  if (inPen(sx[0], sy[0]) && inPen(sx[1], sy[1])) inPenTime += dt; else inPenTime = 0;
  bool won = inPenTime > 0.7f;

  // --- draw ---
  drawField();
  drawPen();
  for (int i = NS - 1; i >= 0; i--) drawSheep(i);                      // marked sheep (0, 1) on top
  drawDog();
  drawHud();
  gfx->draw16bitRGBBitmap(0, 0, fb, 240, 240);

  if (won) {
    state = 1; winMs = millis();
    showWin();
#if DEBUG_SERIAL
    Serial.printf("won in %lu s\n", (unsigned long)((winMs - startMs) / 1000));
#endif
  }

#if DEBUG_SERIAL
  frames++;
  if (millis() - fpsMs >= 2000) {
    Serial.printf("%.1f fps\n", frames * 1000.0f / (millis() - fpsMs));
    frames = 0;
    fpsMs = millis();
  }
#endif
}
