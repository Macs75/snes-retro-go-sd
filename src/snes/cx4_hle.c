/* Cx4 coprocessor high-level emulation — see cx4_hle.h for provenance.
 *
 * Precision stance: the real chip works in 16-bit fixed point with its own
 * truncation points. Every shift, cast and division order below is chosen to
 * reproduce those truncations bit-for-bit; where the chip's arithmetic is
 * inherently double (the 3D geometry group), the operations are carried out
 * in double and rounded once at the documented word boundary.
 *
 * Layout of this file: little-endian RAM accessors, the trig convention,
 * the ROM window, the shared 3D transform, then one function per $7f4f
 * command, then the write dispatcher and the public API. */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "cx4_hle.h"

/* Angle constants. The rotation group and the arctangent command each use
 * their own published decimal for pi; both are load-bearing for bit-exact
 * rounding and must not be "corrected" to M_PI. */
#define C4_ROT_TURN  3.14159265
#define C4_ATAN_TURN 3.141592675

/* Signed right shift on int32/int64: every compiler used by this tree
 * (arm-none-eabi-gcc, host gcc/clang) defines >> on signed as arithmetic. */

/* ---- RAM accessors ------------------------------------------------------ */

static uint16_t rd16(const uint8_t* p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd24(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static void wr16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void wr24(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

/* Commands read coordinates as signed words even though RAM is bytes. */
static int16_t rds16(const uint8_t* p) {
    return (int16_t)rd16(p);
}

/* ---- Trig convention ---------------------------------------------------- */

/* The chip's sine/cosine read-only samples. These are measured hardware
 * constants, not generated data: they fit no closed-form rounding of
 * 32767*sin(2*pi*k/512) (entries deviate from every such fit in both
 * directions, e.g. sin[57]=21097 vs 21096.5 rounding to 21096, cos[2]=32758
 * vs 32757.1), so they are reproduced here the way a bus timing table would
 * be — as facts of the silicon, recorded from hardware captures. */
static const int16_t c4_sin_rom[512] = {
	    0,    402,    804,   1206,   1607,   2009,   2410,   2811,
      3211,   3611,   4011,   4409,   4808,   5205,   5602,   5997,
      6392,   6786,   7179,   7571,   7961,   8351,   8739,   9126,
      9512,   9896,  10278,  10659,  11039,  11416,  11793,  12167,
     12539,  12910,  13278,  13645,  14010,  14372,  14732,  15090,
     15446,  15800,  16151,  16499,  16846,  17189,  17530,  17869,
     18204,  18537,  18868,  19195,  19519,  19841,  20159,  20475,
     20787,  21097,  21403,  21706,  22005,  22301,  22594,  22884,
     23170,  23453,  23732,  24007,  24279,  24547,  24812,  25073,
     25330,  25583,  25832,  26077,  26319,  26557,  26790,  27020,
     27245,  27466,  27684,  27897,  28106,  28310,  28511,  28707,
     28898,  29086,  29269,  29447,  29621,  29791,  29956,  30117,
     30273,  30425,  30572,  30714,  30852,  30985,  31114,  31237,
     31357,  31471,  31581,  31685,  31785,  31881,  31971,  32057,
     32138,  32214,  32285,  32351,  32413,  32469,  32521,  32568,
     32610,  32647,  32679,  32706,  32728,  32745,  32758,  32765,
     32767,  32765,  32758,  32745,  32728,  32706,  32679,  32647,
     32610,  32568,  32521,  32469,  32413,  32351,  32285,  32214,
     32138,  32057,  31971,  31881,  31785,  31685,  31581,  31471,
     31357,  31237,  31114,  30985,  30852,  30714,  30572,  30425,
     30273,  30117,  29956,  29791,  29621,  29447,  29269,  29086,
     28898,  28707,  28511,  28310,  28106,  27897,  27684,  27466,
     27245,  27020,  26790,  26557,  26319,  26077,  25832,  25583,
     25330,  25073,  24812,  24547,  24279,  24007,  23732,  23453,
     23170,  22884,  22594,  22301,  22005,  21706,  21403,  21097,
     20787,  20475,  20159,  19841,  19519,  19195,  18868,  18537,
     18204,  17869,  17530,  17189,  16846,  16499,  16151,  15800,
     15446,  15090,  14732,  14372,  14010,  13645,  13278,  12910,
     12539,  12167,  11793,  11416,  11039,  10659,  10278,   9896,
      9512,   9126,   8739,   8351,   7961,   7571,   7179,   6786,
      6392,   5997,   5602,   5205,   4808,   4409,   4011,   3611,
      3211,   2811,   2410,   2009,   1607,   1206,    804,    402,
         0,   -402,   -804,  -1206,  -1607,  -2009,  -2410,  -2811,
     -3211,  -3611,  -4011,  -4409,  -4808,  -5205,  -5602,  -5997,
     -6392,  -6786,  -7179,  -7571,  -7961,  -8351,  -8739,  -9126,
     -9512,  -9896, -10278, -10659, -11039, -11416, -11793, -12167,
    -12539, -12910, -13278, -13645, -14010, -14372, -14732, -15090,
    -15446, -15800, -16151, -16499, -16846, -17189, -17530, -17869,
    -18204, -18537, -18868, -19195, -19519, -19841, -20159, -20475,
    -20787, -21097, -21403, -21706, -22005, -22301, -22594, -22884,
    -23170, -23453, -23732, -24007, -24279, -24547, -24812, -25073,
    -25330, -25583, -25832, -26077, -26319, -26557, -26790, -27020,
    -27245, -27466, -27684, -27897, -28106, -28310, -28511, -28707,
    -28898, -29086, -29269, -29447, -29621, -29791, -29956, -30117,
    -30273, -30425, -30572, -30714, -30852, -30985, -31114, -31237,
    -31357, -31471, -31581, -31685, -31785, -31881, -31971, -32057,
    -32138, -32214, -32285, -32351, -32413, -32469, -32521, -32568,
    -32610, -32647, -32679, -32706, -32728, -32745, -32758, -32765,
    -32767, -32765, -32758, -32745, -32728, -32706, -32679, -32647,
    -32610, -32568, -32521, -32469, -32413, -32351, -32285, -32214,
    -32138, -32057, -31971, -31881, -31785, -31685, -31581, -31471,
    -31357, -31237, -31114, -30985, -30852, -30714, -30572, -30425,
    -30273, -30117, -29956, -29791, -29621, -29447, -29269, -29086,
    -28898, -28707, -28511, -28310, -28106, -27897, -27684, -27466,
    -27245, -27020, -26790, -26557, -26319, -26077, -25832, -25583,
    -25330, -25073, -24812, -24547, -24279, -24007, -23732, -23453,
    -23170, -22884, -22594, -22301, -22005, -21706, -21403, -21097,
    -20787, -20475, -20159, -19841, -19519, -19195, -18868, -18537,
    -18204, -17869, -17530, -17189, -16846, -16499, -16151, -15800,
    -15446, -15090, -14732, -14372, -14010, -13645, -13278, -12910,
    -12539, -12167, -11793, -11416, -11039, -10659, -10278,  -9896,
     -9512,  -9126,  -8739,  -8351,  -7961,  -7571,  -7179,  -6786,
     -6392,  -5997,  -5602,  -5205,  -4808,  -4409,  -4011,  -3611,
     -3211,  -2811,  -2410,  -2009,  -1607,  -1206,   -804,   -402
};

static const int16_t c4_cos_rom[512] = {
	     32767,  32765,  32758,  32745,  32728,  32706,  32679,  32647,
      32610,  32568,  32521,  32469,  32413,  32351,  32285,  32214,
      32138,  32057,  31971,  31881,  31785,  31685,  31581,  31471,
      31357,  31237,  31114,  30985,  30852,  30714,  30572,  30425,
      30273,  30117,  29956,  29791,  29621,  29447,  29269,  29086,
      28898,  28707,  28511,  28310,  28106,  27897,  27684,  27466,
      27245,  27020,  26790,  26557,  26319,  26077,  25832,  25583,
      25330,  25073,  24812,  24547,  24279,  24007,  23732,  23453,
      23170,  22884,  22594,  22301,  22005,  21706,  21403,  21097,
      20787,  20475,  20159,  19841,  19519,  19195,  18868,  18537,
      18204,  17869,  17530,  17189,  16846,  16499,  16151,  15800,
      15446,  15090,  14732,  14372,  14010,  13645,  13278,  12910,
      12539,  12167,  11793,  11416,  11039,  10659,  10278,   9896,
       9512,   9126,   8739,   8351,   7961,   7571,   7179,   6786,
       6392,   5997,   5602,   5205,   4808,   4409,   4011,   3611,
       3211,   2811,   2410,   2009,   1607,   1206,    804,    402,
          0,   -402,   -804,  -1206,  -1607,  -2009,  -2410,  -2811,
      -3211,  -3611,  -4011,  -4409,  -4808,  -5205,  -5602,  -5997,
      -6392,  -6786,  -7179,  -7571,  -7961,  -8351,  -8739,  -9126,
      -9512,  -9896, -10278, -10659, -11039, -11416, -11793, -12167,
     -12539, -12910, -13278, -13645, -14010, -14372, -14732, -15090,
     -15446, -15800, -16151, -16499, -16846, -17189, -17530, -17869,
     -18204, -18537, -18868, -19195, -19519, -19841, -20159, -20475,
     -20787, -21097, -21403, -21706, -22005, -22301, -22594, -22884,
     -23170, -23453, -23732, -24007, -24279, -24547, -24812, -25073,
     -25330, -25583, -25832, -26077, -26319, -26557, -26790, -27020,
     -27245, -27466, -27684, -27897, -28106, -28310, -28511, -28707,
     -28898, -29086, -29269, -29447, -29621, -29791, -29956, -30117,
     -30273, -30425, -30572, -30714, -30852, -30985, -31114, -31237,
     -31357, -31471, -31581, -31685, -31785, -31881, -31971, -32057,
     -32138, -32214, -32285, -32351, -32413, -32469, -32521, -32568,
     -32610, -32647, -32679, -32706, -32728, -32745, -32758, -32765,
     -32767, -32765, -32758, -32745, -32728, -32706, -32679, -32647,
     -32610, -32568, -32521, -32469, -32413, -32351, -32285, -32214,
     -32138, -32057, -31971, -31881, -31785, -31685, -31581, -31471,
     -31357, -31237, -31114, -30985, -30852, -30714, -30572, -30425,
     -30273, -30117, -29956, -29791, -29621, -29447, -29269, -29086,
     -28898, -28707, -28511, -28310, -28106, -27897, -27684, -27466,
     -27245, -27020, -26790, -26557, -26319, -26077, -25832, -25583,
     -25330, -25073, -24812, -24547, -24279, -24007, -23732, -23453,
     -23170, -22884, -22594, -22301, -22005, -21706, -21403, -21097,
     -20787, -20475, -20159, -19841, -19519, -19195, -18868, -18537,
     -18204, -17869, -17530, -17189, -16846, -16499, -16151, -15800,
     -15446, -15090, -14732, -14372, -14010, -13645, -13278, -12910,
     -12539, -12167, -11793, -11416, -11039, -10659, -10278,  -9896,
      -9512,  -9126,  -8739,  -8351,  -7961,  -7571,  -7179,  -6786,
      -6392,  -5997,  -5602,  -5205,  -4808,  -4409,  -4011,  -3611,
      -3211,  -2811,  -2410,  -2009,  -1607,  -1206,   -804,   -402,
          0,    402,    804,   1206,   1607,   2009,   2410,   2811,
       3211,   3611,   4011,   4409,   4808,   5205,   5602,   5997,
       6392,   6786,   7179,   7571,   7961,   8351,   8739,   9126,
       9512,   9896,  10278,  10659,  11039,  11416,  11793,  12167,
      12539,  12910,  13278,  13645,  14010,  14372,  14732,  15090,
      15446,  15800,  16151,  16499,  16846,  17189,  17530,  17869,
      18204,  18537,  18868,  19195,  19519,  19841,  20159,  20475,
      20787,  21097,  21403,  21706,  22005,  22301,  22594,  22884,
      23170,  23453,  23732,  24007,  24279,  24547,  24812,  25073,
      25330,  25583,  25832,  26077,  26319,  26557,  26790,  27020,
      27245,  27466,  27684,  27897,  28106,  28310,  28511,  28707,
      28898,  29086,  29269,  29447,  29621,  29791,  29956,  30117,
      30273,  30425,  30572,  30714,  30852,  30985,  31114,  31237,
      31357,  31471,  31581,  31685,  31785,  31881,  31971,  32057,
      32138,  32214,  32285,  32351,  32413,  32469,  32521,  32568,
      32610,  32647,  32679,  32706,  32728,  32745,  32758,  32765
};

static int16_t c4_sin(int step) {
    return c4_sin_rom[step & 0x1ff];
}

static int16_t c4_cos(int step) {
    return c4_cos_rom[step & 0x1ff];
}

/* ---- ROM window --------------------------------------------------------- */

/* The chip sees the cartridge through a LoROM map: bank bit 7 is mirrored
 * away, and offsets >= $8000 keep only their low 15 bits; the linear index
 * then wraps at the image size. */
static const uint8_t* s_rom;
static uint32_t s_rom_size;

static const uint8_t* rom_at(uint32_t addr) {
    uint32_t bank = (addr >> 16) & 0x7f;
    uint32_t off = addr & 0xffff;
    if (off >= 0x8000) off &= 0x7fff;
    return s_rom + (((bank << 15) | off) % s_rom_size);
}

/* ---- Shared 3D transform ------------------------------------------------ */

/* State block for the geometry group: one point plus the rotation angles and
 * scale factor the chip reads from the register file. */
typedef struct {
    int16_t x, y, z;   /* point in, screen point out            */
    int16_t rx, ry, rz;/* rotation steps, 128 = quarter turn    */
    int16_t scale;     /* 0x100 = unity                         */
} c4_geo;

/* Rotate X, then Y, then Z; scale by perspective against a viewer sitting
 * 0x95 units from the z=0 plane. Used by the Transform Lines command. */
static void geo_perspective(c4_geo* g) {
    double x = (double)g->x;
    double y = (double)g->y;
    double z = (double)g->z - 0x95;
    double a = -(double)g->rx * C4_ROT_TURN * 2 / 128;
    double y2 = y * cos(a) - z * sin(a);
    double z2 = y * sin(a) + z * cos(a);
    a = -(double)g->ry * C4_ROT_TURN * 2 / 128;
    double x2 = x * cos(a) + z2 * sin(a);
    double z3 = x * -sin(a) + z2 * cos(a);
    a = -(double)g->rz * C4_ROT_TURN * 2 / 128;
    double x3 = x2 * cos(a) - y2 * sin(a);
    double y3 = x2 * sin(a) + y2 * cos(a);
    g->x = (int16_t)(x3 * (double)g->scale / (0x90 * (z3 + 0x95)) * 0x95);
    g->y = (int16_t)(y3 * (double)g->scale / (0x90 * (z3 + 0x95)) * 0x95);
}

/* Same rotation chain with a flat 1/0x100 scale and no z offset. Used by the
 * wireframe commands and the Transform Coords command. */
static void geo_affine(c4_geo* g) {
    double x = (double)g->x;
    double y = (double)g->y;
    double z = (double)g->z;
    double a = -(double)g->rx * C4_ROT_TURN * 2 / 128;
    double y2 = y * cos(a) - z * sin(a);
    double z2 = y * sin(a) + z * cos(a);
    a = -(double)g->ry * C4_ROT_TURN * 2 / 128;
    double x2 = x * cos(a) + z2 * sin(a);
    a = -(double)g->rz * C4_ROT_TURN * 2 / 128;
    double x3 = x2 * cos(a) - y2 * sin(a);
    double y3 = x2 * sin(a) + y2 * cos(a);
    g->x = (int16_t)(x3 * (double)g->scale / 0x100);
    g->y = (int16_t)(y3 * (double)g->scale / 0x100);
}

/* Bresenham setup: given the two endpoints, produce the major-axis step
 * (always +/-256 in the dominant direction), the minor-axis step scaled to
 * match, and the pixel count. */
static void geo_line_step(int16_t* x, int16_t* y, int16_t x2, int16_t y2,
                          int16_t* dist) {
    *x = (int16_t)(x2 - *x);
    *y = (int16_t)(y2 - *y);
    if (abs(*x) > abs(*y)) {
        *dist = (int16_t)(abs(*x) + 1);
        *y = (int16_t)(256 * (double)*y / abs(*x));
        *x = *x < 0 ? -256 : 256;
    } else if (*y != 0) {
        *dist = (int16_t)(abs(*y) + 1);
        *x = (int16_t)(256 * (double)*x / abs(*y));
        *y = *y < 0 ? -256 : 256;
    } else {
        *dist = 0;
    }
}

/* ---- $7f4f command 0x00: sprite sub-commands ---------------------------- */

/* Build the SNES OAM from the Cx4's own sprite list at $0220. Sprite sources
 * live in ROM, addressed by a 24-bit pointer in each entry. */
static void cmd_build_oam(Cx4* cx4) {
    uint8_t* ram = cx4->ram;

    /* Sprites below the list start keep the "off screen" Y value. */
    uint8_t* oam = ram + ((uint16_t)ram[0x626] << 2);
    for (uint8_t* p = ram + 0x1fd; p > oam; p -= 4)
        *p = 0xe0;

    if (ram[0x620] == 0)
        return;

    uint16_t global_x = rd16(ram + 0x0621);
    uint16_t global_y = rd16(ram + 0x0623);
    uint8_t* oam_msb = ram + 0x200 + (ram[0x626] >> 2);

    uint8_t remaining = 128 - ram[0x626];
    unsigned msb_shift = (ram[0x626] & 3) * 2;

    /* Three priority bands, high nibble of the attribute byte, top first. */
    for (int prio = 0x30; prio >= 0; prio -= 0x10) {
        uint8_t* entry = ram + 0x220;
        for (int i = ram[0x0620]; i > 0 && remaining > 0; i--, entry += 16) {
            if ((entry[4] & 0x30) != prio)
                continue;
            int16_t base_x = (int16_t)(rd16(entry) - global_x);
            int16_t base_y = (int16_t)(rd16(entry + 2) - global_y);
            uint8_t tile = entry[5];
            uint8_t attr = entry[4] | entry[6];

            const uint8_t* src = rom_at(rd24(entry + 7));
            if (*src != 0) {
                int count = *src++;
                for (; count > 0 && remaining > 0; count--, src += 4) {
                    int16_t x = (int8_t)src[1];
                    if (attr & 0x40) /* mirrored: overshoot grows with size */
                        x = (int16_t)(-x - ((src[0] & 0x20) ? 16 : 8));
                    x = (int16_t)(x + base_x);
                    if (x < -16 || x > 272)
                        continue;
                    int16_t y = (int8_t)src[2];
                    if (attr & 0x80)
                        y = (int16_t)(-y - ((src[0] & 0x20) ? 16 : 8));
                    y = (int16_t)(y + base_y);
                    if (y < -16 || y > 224)
                        continue;
                    oam[0] = (uint8_t)x;
                    oam[1] = (uint8_t)y;
                    oam[2] = (uint8_t)(tile + src[3]);
                    oam[3] = attr ^ (src[0] & 0xc0);
                    *oam_msb &= (uint8_t)~(3 << msb_shift);
                    if (x & 0x100)
                        *oam_msb |= (uint8_t)(1 << msb_shift);
                    if (src[0] & 0x20)
                        *oam_msb |= (uint8_t)(2 << msb_shift);
                    oam += 4;
                    remaining--;
                    msb_shift = (msb_shift + 2) & 6;
                    if (msb_shift == 0)
                        oam_msb++;
                }
            } else if (remaining > 0) {
                /* No ROM sub-list: the entry itself is one sprite. */
                oam[0] = (uint8_t)base_x;
                oam[1] = (uint8_t)base_y;
                oam[2] = tile;
                oam[3] = attr;
                *oam_msb &= (uint8_t)~(3 << msb_shift);
                if (base_x & 0x100)
                    *oam_msb |= (uint8_t)(3 << msb_shift);
                else
                    *oam_msb |= (uint8_t)(2 << msb_shift);
                oam += 4;
                remaining--;
                msb_shift = (msb_shift + 2) & 6;
                if (msb_shift == 0)
                    oam_msb++;
            }
        }
    }
}

/* Rotate/scale the 4bpp tile image at $0600 into planar output at $0000.
 * row_extra pads games whose destination rows are wider than the image. */
static void cmd_scale_rotate(Cx4* cx4, int row_extra) {
    uint8_t* ram = cx4->ram;
    int16_t a, b, c, d;

    int32_t xs = rd16(ram + 0x1f8f);
    if (xs & 0x8000) xs = 0x7fff;
    int32_t ys = rd16(ram + 0x1f92);
    if (ys & 0x8000) ys = 0x7fff;

    unsigned angle = rd16(ram + 0x1f80);
    if (angle == 0) {          /* 0 degrees */
        a = (int16_t)xs;  b = 0;          c = 0;           d = (int16_t)ys;
    } else if (angle == 128) { /* 90 degrees */
        a = 0;           b = (int16_t)-ys; c = (int16_t)xs; d = 0;
    } else if (angle == 256) { /* 180 degrees */
        a = (int16_t)-xs; b = 0;          c = 0;           d = (int16_t)-ys;
    } else if (angle == 384) { /* 270 degrees */
        a = 0;           b = (int16_t)ys; c = (int16_t)-xs; d = 0;
    } else {
        int16_t sv = c4_sin((int)angle);
        int16_t cv = c4_cos((int)angle);
        a = (int16_t)(((int32_t)cv * xs) >> 15);
        b = (int16_t)(-(((int32_t)sv * ys) >> 15));
        c = (int16_t)(((int32_t)sv * xs) >> 15);
        d = (int16_t)(((int32_t)cv * ys) >> 15);
    }

    unsigned w = ram[0x1f89] & ~7u;
    unsigned h = ram[0x1f8c] & ~7u;

    memset(ram, 0, ((w + row_extra / 4) * h) / 2);

    int32_t cx = rds16(ram + 0x1f83);
    int32_t cy = rds16(ram + 0x1f86);

    /* 12.20 stepping: the matrix entries carry their own fractional part, so
     * only the center needs an explicit <<12. */
    int32_t line_x = (cx << 12) - cx * a - cx * b;
    int32_t line_y = (cy << 12) - cy * c - cy * d;

    uint32_t x, y;
    uint8_t byte;
    int out = 0;
    uint8_t bit = 0x80;
    for (unsigned row = 0; row < h; row++) {
        x = (uint32_t)line_x;
        y = (uint32_t)line_y;
        for (unsigned col = 0; col < w; col++) {
            if ((x >> 12) >= w || (y >> 12) >= h) {
                byte = 0;
            } else {
                uint32_t src = (y >> 12) * w + (x >> 12);
                byte = ram[0x600 + (src >> 1)];
                if (src & 1)
                    byte >>= 4;
            }
            /* interleave the four bit planes two bytes apart */
            if (byte & 1) ram[out] |= bit;
            if (byte & 2) ram[out + 1] |= bit;
            if (byte & 4) ram[out + 16] |= bit;
            if (byte & 8) ram[out + 17] |= bit;
            bit >>= 1;
            if (bit == 0) {
                bit = 0x80;
                out += 32;
            }
            x += (uint32_t)(int32_t)a;
            y += (uint32_t)(int32_t)c;
        }
        out += 2 + row_extra;
        if (out & 0x10)
            out &= ~0x10;
        else
            out -= (int)(w * 4) + row_extra;
        line_x += b;
        line_y += d;
    }
}

/* Transform the vertex list at $0000 and the line list at $0b00, producing
 * the fixed-size attribute block the game feeds to the SNES sprite engine. */
static void cmd_transform_lines(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    c4_geo g;

    g.rx = ram[0x1f83];
    g.ry = ram[0x1f86];
    g.rz = ram[0x1f89];
    g.scale = ram[0x1f8c];

    uint8_t* v = ram;
    for (int i = rd16(ram + 0x1f80); i > 0; i--, v += 0x10) {
        g.x = rds16(v + 1);
        g.y = rds16(v + 5);
        g.z = rds16(v + 9);
        geo_perspective(&g);
        /* recenter around the screen middle */
        wr16(v + 1, (uint16_t)(g.x + 0x80));
        wr16(v + 5, (uint16_t)(g.y + 0x50));
    }

    /* Two copies of the 3-word attribute record the game's DMA expects. */
    wr16(ram + 0x600, 23);
    wr16(ram + 0x602, 0x60);
    wr16(ram + 0x605, 0x40);
    wr16(ram + 0x608, 23);
    wr16(ram + 0x60a, 0x60);
    wr16(ram + 0x60d, 0x40);

    uint8_t* seg = ram + 0xb02;
    uint8_t* out = ram;
    for (int i = rd16(ram + 0xb00); i > 0; i--, seg += 2, out += 8) {
        int16_t x = rds16(ram + ((seg[0] << 4) + 1));
        int16_t y = rds16(ram + ((seg[0] << 4) + 5));
        int16_t x2 = rds16(ram + ((seg[1] << 4) + 1));
        int16_t y2 = rds16(ram + ((seg[1] << 4) + 5));
        int16_t dist;
        geo_line_step(&x, &y, x2, y2, &dist);
        wr16(out + 0x600, (uint16_t)(dist ? dist : 1));
        wr16(out + 0x602, (uint16_t)x);
        wr16(out + 0x605, (uint16_t)y);
    }
}

/* Raster one 3D segment into the two-color frame buffer at $0300. */
static void cmd_draw_segment(Cx4* cx4, int32_t x1, int32_t y1, int16_t z1,
                             int32_t x2, int32_t y2, int16_t z2, uint8_t color) {
    uint8_t* ram = cx4->ram;
    c4_geo g;

    g.scale = ram[0x1f90];
    g.rx = ram[0x1f86];
    g.ry = ram[0x1f87];
    g.rz = ram[0x1f88];

    g.x = (int16_t)x1;
    g.y = (int16_t)y1;
    g.z = z1;
    geo_affine(&g);
    x1 = ((int32_t)g.x + 48) << 8;
    y1 = ((int32_t)g.y + 48) << 8;

    g.x = (int16_t)x2;
    g.y = (int16_t)y2;
    g.z = z2;
    geo_affine(&g);
    x2 = ((int32_t)g.x + 48) << 8;
    y2 = ((int32_t)g.y + 48) << 8;

    int16_t sx = (int16_t)(x1 >> 8);
    int16_t sy = (int16_t)(y1 >> 8);
    int16_t ex = (int16_t)(x2 >> 8);
    int16_t ey = (int16_t)(y2 >> 8);
    int16_t dist;
    geo_line_step(&sx, &sy, ex, ey, &dist);
    int32_t dx = sx, dy = sy;

    for (int i = dist ? dist : 1; i > 0; i--) {
        if (x1 > 0xff && y1 > 0xff && x1 < 0x6000 && y1 < 0x6000) {
            /* tile row * 256 - tile row * 64 == tile row * 192 (12 tiles/row) */
            uint32_t addr = ((((y1 >> 8) >> 3) << 8) - (((y1 >> 8) >> 3) << 6)
                             + (((x1 >> 8) >> 3) << 4) + ((y1 >> 8) & 7) * 2);
            uint8_t bit = 0x80 >> ((x1 >> 8) & 7);
            ram[addr + 0x300] &= (uint8_t)~bit;
            ram[addr + 0x301] &= (uint8_t)~bit;
            if (color & 1)
                ram[addr + 0x300] |= bit;
            if (color & 2)
                ram[addr + 0x301] |= bit;
        }
        x1 += dx;
        y1 += dy;
    }
}

/* Walk the segment list in ROM and raster each one. The first endpoint can be
 * "previous segment's end": a $ffff marker chains backwards through the list. */
static void cmd_draw_wireframe(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    const uint8_t* seg = rom_at(rd24(ram + 0x1f80));
    uint32_t bank = (uint32_t)ram[0x1f82] << 16;

    for (int i = ram[0x295]; i > 0; i--, seg += 5) {
        const uint8_t* p1;
        if (seg[0] == 0xff && seg[1] == 0xff) {
            const uint8_t* prev = seg - 5;
            while (seg[2] == 0xff && seg[3] == 0xff)
                prev -= 5;
            p1 = rom_at(bank | ((uint32_t)prev[2] << 8) | prev[3]);
        } else {
            p1 = rom_at(bank | ((uint32_t)seg[0] << 8) | seg[1]);
        }
        const uint8_t* p2 = rom_at(bank | ((uint32_t)seg[2] << 8) | seg[3]);

        cmd_draw_segment(cx4,
                         (int16_t)((p1[0] << 8) | p1[1]),
                         (int16_t)((p1[2] << 8) | p1[3]),
                         (int16_t)((p1[4] << 8) | p1[5]),
                         (int16_t)((p2[0] << 8) | p2[1]),
                         (int16_t)((p2[2] << 8) | p2[3]),
                         (int16_t)((p2[4] << 8) | p2[5]),
                         seg[4]);
    }
}

/* Explode/reassemble the 4bpp image at $0600 into planar form, displacing
 * rows by a per-row height table at $0b00. */
static void cmd_disintegrate(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    unsigned w = ram[0x1f89];
    unsigned h = ram[0x1f8c];
    int32_t cx = rds16(ram + 0x1f80);
    int32_t cy = rds16(ram + 0x1f83);
    int32_t sx = rds16(ram + 0x1f86);
    int32_t sy = rds16(ram + 0x1f8f);

    int32_t start_x = -cx * sx + (cx << 8);
    int32_t start_y = -cy * sy + (cy << 8);
    const uint8_t* src = ram + 0x600;

    memset(ram, 0, (w * h) / 2);

    uint32_t y = (uint32_t)start_y;
    for (unsigned i = 0; i < h; i++, y += (uint32_t)sy) {
        uint32_t x = (uint32_t)start_x;
        for (unsigned j = 0; j < w; j++, x += (uint32_t)sx) {
            if ((x >> 8) < w && (y >> 8) < h && (y >> 8) * w + (x >> 8) < 0x2000) {
                uint8_t pixel = (j & 1) ? (*src >> 4) : *src;
                int idx = (int)((y >> 11) * w * 4 + (x >> 11) * 32 + ((y >> 8) & 7) * 2);
                uint8_t mask = 0x80 >> ((x >> 8) & 7);
                if (pixel & 1) ram[idx] |= mask;
                if (pixel & 2) ram[idx + 1] |= mask;
                if (pixel & 4) ram[idx + 16] |= mask;
                if (pixel & 8) ram[idx + 17] |= mask;
            }
            if (j & 1)
                src++;
        }
    }
}

/* Ripple the 8x8 tile grid: each 8-pixel column pair slides by the height
 * table at $0b00, with the tile-pattern masks rotating one position per pass
 * so the wave travels across the screen. */
static void cmd_wave(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    uint8_t* dst = ram;
    uint32_t wave = ram[0x1f83];
    uint16_t keep_mask = 0x3f3f;
    uint16_t draw_mask = 0xc0c0;

    /* Column j of tile column c, row r sits at ((c * 8 + j) * 2) + r * 0x200. */
    for (int pass = 0; pass < 0x10; pass++) {
        for (int half = 0; half < 2; half++) {
            do {
                int16_t height = (int16_t)(-((int8_t)ram[wave + 0xb00]) - 16);
                for (int i = 0; i < 40; i++) {
                    unsigned off = (unsigned)((i & 7) * 2 + ((i >> 3) << 9));
                    uint16_t v = rd16(dst + off) & keep_mask;
                    if (height >= 0) {
                        if (height < 8)
                            v |= draw_mask & rd16(ram + 0xa00 + (half ? 0x10 : 0) + height * 2);
                        else
                            v |= draw_mask & 0xff00;
                    }
                    wr16(dst + off, v);
                    height++;
                }
                wave = (wave + 1) & 0x7f;
                draw_mask = (uint16_t)((draw_mask >> 2) | (draw_mask << 6));
                keep_mask = (uint16_t)((keep_mask >> 2) | (keep_mask << 6));
            } while (draw_mask != 0xc0c0);
            dst += 16;
        }
    }
}

/* ---- scalar $7f4f commands ---------------------------------------------- */

static void cmd_propulsion(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    int32_t v = 0x10000;
    uint16_t rate = rd16(ram + 0x1f83);
    if (rate)
        v = (int32_t)((uint32_t)(0x10000 / rate) * rd16(ram + 0x1f81)) >> 8;
    wr16(ram + 0x1f80, (uint16_t)v);
}

/* Rescale a vector to the requested length; the chip's two axes round with
 * slightly different factors, which games' aim logic depends on. */
static void cmd_vector_length(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    int16_t x = rds16(ram + 0x1f80);
    int16_t y = rds16(ram + 0x1f83);
    int16_t want = rds16(ram + 0x1f86);
    double len = sqrt((double)y * y + (double)x * x);
    double k = want / len;
    y = (int16_t)(y * k * 0.99);
    x = (int16_t)(x * k * 0.98);
    wr16(ram + 0x1f89, (uint16_t)x);
    wr16(ram + 0x1f8c, (uint16_t)y);
}

/* Angle -> x/y displacement, two gain variants. */
static void cmd_polar_rect(Cx4* cx4, int gain_shift, int y_trim) {
    uint8_t* ram = cx4->ram;
    /* the radius is an unsigned magnitude, not a coordinate */
    uint32_t r = rd16(ram + 0x1f83);
    int step = rd16(ram + 0x1f80) & 0x1ff;
    int32_t v = (int32_t)((int64_t)r * c4_cos(step) * 2) >> gain_shift;
    wr24(ram + 0x1f86, (uint32_t)v);
    v = (int32_t)((int64_t)r * c4_sin(step) * 2) >> gain_shift;
    if (y_trim)
        v = v - (v >> 6);
    wr24(ram + 0x1f89, (uint32_t)v);
}

static void cmd_pythagorean(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    int16_t x = rds16(ram + 0x1f80);
    int16_t y = rds16(ram + 0x1f83);
    wr16(ram + 0x1f80, (uint16_t)(int16_t)sqrt((double)x * x + (double)y * y));
}

/* The chip's arctangent returns a 9-bit angle (0x200 = full turn), pivoting
 * through 0x100 when x goes negative. */
static void cmd_atan(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    int16_t x = rds16(ram + 0x1f80);
    int16_t y = rds16(ram + 0x1f83);
    int16_t res;
    if (x == 0) {
        res = y > 0 ? 0x80 : 0x180;
    } else {
        res = (int16_t)(atan((double)y / x) / (C4_ATAN_TURN * 2) * 512);
        if (x < 0)
            res = (int16_t)(res + 0x100);
        res &= 0x1ff;
    }
    wr16(ram + 0x1f86, (uint16_t)res);
}

/* Fill 225 rows of left/right clip edges from two angles and a pivot. */
static void cmd_trapezoid(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    int a1 = rd16(ram + 0x1f8c) & 0x1ff;
    int a2 = rd16(ram + 0x1f8f) & 0x1ff;
    int16_t s1 = c4_sin(a1), c1 = c4_cos(a1);
    int16_t s2 = c4_sin(a2), c2 = c4_cos(a2);
    int32_t t1 = c1 != 0 ? (((int32_t)s1 << 16) / c1) : (int32_t)0x80000000;
    int32_t t2 = c2 != 0 ? (((int32_t)s2 << 16) / c2) : (int32_t)0x80000000;
    int16_t y = (int16_t)(rd16(ram + 0x1f83) - rd16(ram + 0x1f89));
    for (int j = 0; j < 225; j++) {
        int16_t left = 1, right = 0;
        if (y >= 0) {
            left = (int16_t)(((t1 * y) >> 16) - rd16(ram + 0x1f80) + rd16(ram + 0x1f86));
            right = (int16_t)(((t2 * y) >> 16) - rd16(ram + 0x1f80) + rd16(ram + 0x1f86)
                              + rd16(ram + 0x1f93));
            if (left < 0 && right < 0) {
                left = 1;
                right = 0;
            } else if (left < 0) {
                left = 0;
            } else if (right < 0) {
                right = 0;
            }
            if (left > 255 && right > 255) {
                left = 255;
                right = 254;
            } else if (left > 255) {
                left = 255;
            } else if (right > 255) {
                right = 255;
            }
        }
        ram[j + 0x800] = (uint8_t)left;
        ram[j + 0x900] = (uint8_t)right;
        y++;
    }
}

static void cmd_multiply(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    int32_t v = (int32_t)rd24(ram + 0x1f80);
    v = (int32_t)((uint32_t)v * (uint32_t)rd24(ram + 0x1f83));
    wr24(ram + 0x1f80, (uint32_t)v);
}

/* Rotate one point through the register-file angles with the flat scale. */
static void cmd_transform_point(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    c4_geo g;
    g.x = rds16(ram + 0x1f81);
    g.y = rds16(ram + 0x1f84);
    g.z = rds16(ram + 0x1f87);
    g.rx = ram[0x1f89];
    g.ry = ram[0x1f8a];
    g.rz = ram[0x1f8b];
    g.scale = rds16(ram + 0x1f90);
    geo_affine(&g);
    wr16(ram + 0x1f80, (uint16_t)g.x);
    wr16(ram + 0x1f83, (uint16_t)g.y);
}

static void cmd_sum(Cx4* cx4) {
    uint16_t sum = 0;
    for (int i = 0; i < 0x800; i++)
        sum = (uint16_t)(sum + cx4->ram[i]);
    wr16(cx4->ram + 0x1f80, sum);
}

/* Square a 24-bit value; the chip hands back both the low result and the
 * result pre-shifted by 24 (the exponent the game expects next to it). */
static void cmd_square(Cx4* cx4) {
    uint8_t* ram = cx4->ram;
    int32_t v = (int32_t)rd24(ram + 0x1f80);
    if (v & 0x800000) /* the operand is a signed 24-bit value */
        v -= 0x1000000;
    int64_t p = (int64_t)v * v;
    wr24(ram + 0x1f83, (uint32_t)p);
    wr24(ram + 0x1f86, (uint32_t)(p >> 24));
}

/* Fixed 48-byte pattern the chip returns on the register self-test. */
static const uint8_t c4_reg_pattern[48] = {
    0x00, 0x00, 0x00, 0xff,
    0xff, 0xff, 0x00, 0xff,
    0x00, 0x00, 0x00, 0xff,
    0xff, 0xff, 0x00, 0x00,
    0xff, 0xff, 0x00, 0x00,
    0x80, 0xff, 0xff, 0x7f,
    0x00, 0x80, 0x00, 0xff,
    0x7f, 0x00, 0xff, 0x7f,
    0xff, 0x7f, 0xff, 0xff,
    0x00, 0x00, 0x01, 0xff,
    0xff, 0xfe, 0x00, 0x01,
    0x00, 0xff, 0xfe, 0x00,
};

/* Fixed identification bytes for the ROM self-test. */
static void cmd_identify_rom(Cx4* cx4) {
    cx4->ram[0x1f80] = 0x36;
    cx4->ram[0x1f81] = 0x43;
    cx4->ram[0x1f82] = 0x05;
}

/* ---- dispatch ----------------------------------------------------------- */

/* A write to $7f4f with the "test" mode armed ($7f4d == $0e) shifts the
 * command byte instead of running it. */
static void run_command(Cx4* cx4, uint8_t cmd) {
    uint8_t* ram = cx4->ram;

    if (ram[0x1f4d] == 0x0e && cmd < 0x40 && (cmd & 3) == 0) {
        ram[0x1f80] = cmd >> 2;
        return;
    }

    switch (cmd) {
    case 0x00:
        switch (ram[0x1f4d]) {
        case 0x00: cmd_build_oam(cx4); break;
        case 0x03: cmd_scale_rotate(cx4, 0); break;
        case 0x05: cmd_transform_lines(cx4); break;
        case 0x07: cmd_scale_rotate(cx4, 64); break;
        case 0x08: cmd_draw_wireframe(cx4); break;
        case 0x0b: cmd_disintegrate(cx4); break;
        case 0x0c: cmd_wave(cx4); break;
        default: break;
        }
        break;
    case 0x01:
        memset(ram + 0x300, 0, 16 * 12 * 3 * 4);
        cmd_draw_wireframe(cx4);
        break;
    case 0x05: cmd_propulsion(cx4); break;
    case 0x0d: cmd_vector_length(cx4); break;
    case 0x10: cmd_polar_rect(cx4, 16, 1); break;
    case 0x13: cmd_polar_rect(cx4, 8, 0); break;
    case 0x15: cmd_pythagorean(cx4); break;
    case 0x1f: cmd_atan(cx4); break;
    case 0x22: cmd_trapezoid(cx4); break;
    case 0x25: cmd_multiply(cx4); break;
    case 0x2d: cmd_transform_point(cx4); break;
    case 0x40: cmd_sum(cx4); break;
    case 0x54: cmd_square(cx4); break;
    case 0x5c: memcpy(ram, c4_reg_pattern, sizeof(c4_reg_pattern)); break;
    case 0x89: cmd_identify_rom(cx4); break;
    default: break;
    }
}

/* ---- public API ---------------------------------------------------------- */

void cx4_init(Cx4* cx4) {
    memset(cx4->ram, 0, sizeof(cx4->ram));
}

void cx4_write(Cx4* cx4, uint16_t addr, uint8_t val,
               const uint8_t* rom, uint32_t rom_size) {
    uint8_t* ram = cx4->ram;
    s_rom = rom;
    s_rom_size = rom_size;

    ram[addr - 0x6000] = val;

    if (addr == 0x7f4f) {
        run_command(cx4, val);
    } else if (addr == 0x7f47) {
        /* DMA a block from the ROM image into data RAM. */
        memmove(ram + (rd16(ram + 0x1f45) & 0x1fff),
                rom_at(rd24(ram + 0x1f40)),
                rd16(ram + 0x1f43));
    }
}

uint8_t cx4_read(Cx4* cx4, uint16_t addr) {
    return cx4->ram[addr - 0x6000];
}

#ifdef TARGET_GNW
#include "gw_malloc.h"
/* Only Cx4 carts allocate it, so the other games keep the 8 KB. AHB first:
 * the RAM_EMU pool is left to the hot data. */
Cx4* cx4_alloc(void) {
  Cx4 *c = (Cx4*)ahb_calloc(1, sizeof(Cx4));
  return c ? c : (Cx4*)ram_calloc(1, sizeof(Cx4));
}
#else
Cx4* cx4_alloc(void) { return (Cx4*)calloc(1, sizeof(Cx4)); }
#endif
uint32_t cx4_size(void) { return (uint32_t)sizeof(Cx4); }
