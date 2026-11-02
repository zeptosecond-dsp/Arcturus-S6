/* SPDX-License-Identifier: GPL-3.0-only */
/* Cellular additive: test build 0.2. The sister of Arcturus S6 (the main branch), on Dinghy (zvenson), on the platform
 * of Felucca (Hügelton Instruments) as SLOOP (isod89) carries it.
 *
 * The engine as Arcturus S6 1.7 has it: 22.05 kHz (each sample held twice, no smoothing: the images stay) and 12 bit
 * words on the bus, through S6's CRT (the bulge, the halo, the scanlines).
 *
 * What it is for: to measure what 64 sine partials cost on the FM-1 before anything else goes in. A 16-cell
 * automaton (an elementary rule, wrapping edges, mutation, a rescue when it dies, fills or loops) switches the
 * harmonics; each cell owns 1, 2 or 4 of them (16 / 32 / 64 partials). One screen:
 *
 *   K1 rule  K2 automaton clock  K3 harmonic slew  K4 tilt   ALGORITHM mutation   PRESETS stretch
 *   SELECT partials (16 / 32 / 64)   keys: the root (OCT- / OCT+ its octave)
 *   PLAY  stress test: every partial on, the worst case for the CPU (PLAY lit)
 *   SEL   reseed the cells        REC  anchor the fundamental on / off
 *   the keys' LEDs: the 16 cells
 *
 * The CPU line is the audio interrupt's own time, as a share of its 5.8 ms: AVG over half a second, PEAK the worst
 * render in that half second, MAX the worst since the last change of partials or of the stress test.
 *
 * Rules of the road (the chip has no FPU and no libgcc): integers only, no float, no 64-bit division.
 * tests/host.c runs this file on a computer: the screen as a PNG, the sound as a WAV. */
#define APP_NAME "CELLULAR"
#define CA_VERSION "0.2"
#define CV_MAX (240u * 218u)                    /* the page body's canvas */
#define STORE_MAGIC 0x41434D46u                 /* "FMCA": never Arcturus's flash data */
#include "platform.c"
#include "font_chunky.h"
#define FONT_S FONT_C
#include "luts.h"
#include "ca_tables.h"

static int32_t cl(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }

/* ================================================================== the engine (audio interrupt) */
#define NP 64u                                  /* partials, at most */
#define NC 16u                                  /* cells */
#define SUB 32u                                 /* frames between the partials' level updates (1378 Hz) */
#define SIN_BITS 12u
static int16_t SINT[1u << SIN_BITS];            /* one cycle, built at boot (8 KB of RAM: faster than flash) */

typedef struct { uint32_t ph; int32_t env, amp; } part_t;   /* env: the slewed level, Q23; amp: Q15, as played */
static part_t pt[NP];
static volatile int32_t tgt_v[NP];              /* each partial's level to slew to, Q15 (the main loop writes) */
static volatile uint32_t inc_v[NP], kh_v[NP];   /* its phase step; its slew step per block, Q16 */
static volatile uint32_t us_sum, us_n, us_peak; /* the interrupt's time, for the CPU line (the main loop resets) */
static volatile uint32_t live_v;                /* partials sounding in the last render */
static volatile int32_t eng_mask = ~15;         /* the word length: 12 bit (as S6) */
static volatile uint8_t eng_half = 1;           /* 22.05 kHz: each sample held twice (as S6) */

static int32_t sine(uint32_t ph)                /* a full cycle over 2^32 -> -32767..32767 (a refined parabola) */
{
    int32_t x = (int32_t)ph >> 16, ax = x < 0 ? -x : x, y, ay;
    y = (x * (32768 - ax)) >> 13;
    ay = y < 0 ? -y : y;
    y += ((((y * ay) >> 15) - y) * 7373) >> 15;
    return y > 32767 ? 32767 : y < -32767 ? -32767 : y;
}

static const uint32_t INC_C9[12] = {            /* MIDI 120..131 (C9..B9); lower octaves shift right */
    815363807u, 863847862u, 915214929u, 969636441u, 1027294024u, 1088380105u,
    1153098554u, 1221665363u, 1294309365u, 1371273005u, 1452813141u, 1539201906u};
#define Q_TOP (124 * 256)                       /* E9, 10.5 kHz: the highest partial, under 22.05 kHz's Nyquist */
#define Q_FADE (117 * 256)                      /* (A8, 7 kHz: from here the partials fade out toward Q_TOP) */
static uint32_t pitch_inc(int32_t q)            /* 1/256 semitone (MIDI 0 .. 131) -> phase step */
{
    uint32_t s, f, inc;
    q = cl(q, 0, Q_TOP);
    s = (uint32_t)q >> 8;
    f = (uint32_t)q & 255u;
    inc = INC_C9[s % 12u] >> (10u - s / 12u);
    return (uint32_t)(((uint64_t)inc * LUT_SEMIFRAC[f]) >> 16);
}

static void render(int32_t *out)                /* SUB frames: odd harmonics lean left, even ones right */
{
    const uint32_t half = eng_half, nf = half ? SUB / 2u : SUB, sh = half ? 4u : 5u;   /* frames computed; log2 */
    const int32_t qm = eng_mask, qh = (int32_t)(((uint32_t)~qm + 1u) >> 1);
    int32_t ba[SUB], bb[SUB];
    uint32_t h, i, live = 0;
    for (i = 0; i < nf; i++) ba[i] = bb[i] = 0;
    for (h = 0; h < NP; h++) {
        part_t *p = &pt[h];
        int32_t t = tgt_v[h], a, a1, da, *buf;
        uint32_t ph, inc;
        if (t == 0 && p->amp == 0 && p->env < 512) { p->env = 0; continue; }   /* silent: costs nothing */
        p->env += (int32_t)(((int64_t)((t << 8) - p->env) * (int32_t)kh_v[h]) >> 16);
        if (t == 0 && p->env < 512) p->env = 0;
        a1 = p->env >> 8;
        a = p->amp;
        da = (a1 - a) >> sh;                    /* a straight ramp across the block */
        inc = inc_v[h] << half;                 /* (at 22 kHz: twice the step, every other frame) */
        ph = p->ph;
        buf = (h & 1u) ? bb : ba;               /* (h = 0 is harmonic 1: odd) */
        for (i = 0; i < nf; i++) {
            ph += inc;
            buf[i] += (SINT[ph >> (32u - SIN_BITS)] * a) >> 15;
            a += da;
        }
        p->ph = ph;
        p->amp = a1;
        live++;
    }
    for (i = 0; i < nf; i++) {                  /* to the bus, in 12 bit words; at 22 kHz each frame twice */
        int32_t l = (ba[i] + (bb[i] >> 1) + qh) & qm, r = (bb[i] + (ba[i] >> 1) + qh) & qm;
        if (half) {
            out[4u * i] += l; out[4u * i + 1u] += r;
            out[4u * i + 2u] += l; out[4u * i + 3u] += r;
        } else {
            out[2u * i] += l; out[2u * i + 1u] += r;
        }
    }
    live_v = live;
}

static void app_audio(int32_t *out, uint32_t n)
{
    uint32_t b;
#ifndef DINGHY_HOST
    uint32_t t0 = fm1_ticks(), us;
#endif
    for (b = 0; b + SUB <= n; b += SUB)
        render(out + 2u * b);
#ifndef DINGHY_HOST
    us = (fm1_ticks() - t0) / FM1_TICKS_PER_US;
    us_sum += us;
    us_n++;
    if (us > us_peak) us_peak = us;
#endif
}

/* ================================================================== the automaton (main loop) */
static uint32_t rnd_s = 0x9E3779B9u;
static uint32_t rnd(uint32_t n) { rnd_s ^= rnd_s << 13; rnd_s ^= rnd_s >> 17; rnd_s ^= rnd_s << 5; return n ? rnd_s % n : 0u; }

#define HIST 30u                                /* rows kept for the screen */
#define SEEN 32u                                /* rows remembered to spot a loop */
static uint16_t row, hist[HIST], seen[SEEN];
static uint32_t hhead, nseen, rescue_t, gens;
static const char *rescue_why = "";

static void ca_seed(void)
{
    uint32_t c;
    row = 0;
    for (c = 0; c < NC; c++) if (rnd(100) < 30u) row |= (uint16_t)(1u << c);
    if (!row) row = 1;
    nseen = 0;
}
static uint32_t popcount16(uint32_t v) { uint32_t n = 0; while (v) { v &= v - 1u; n++; } return n; }
static void ca_step(uint32_t rule, uint32_t mut)   /* mut: per cent of cells flipped at random */
{
    uint32_t c, n = 0, k;
    for (c = 0; c < NC; c++) {
        uint32_t l = (row >> ((c + NC - 1u) % NC)) & 1u, m = (row >> c) & 1u, r = (row >> ((c + 1u) % NC)) & 1u;
        n |= ((rule >> (l << 2 | m << 1 | r)) & 1u) << c;
    }
    for (c = 0; c < NC; c++) if (rnd(100) < mut) n ^= 1u << c;
    row = (uint16_t)n;
    {                                           /* the rescue: died, filled up, or a row seen in the last 32 */
        uint32_t p = popcount16(row), looped = 0;
        for (k = 0; k < nseen; k++) if (seen[k] == row) looped = 1;
        rescue_why = p == 0u ? "DIED" : p == NC ? "FILLED" : looped ? "LOOPED" : "";
        if (*rescue_why) {
            for (k = 0; k < 4u; k++) row ^= (uint16_t)(1u << rnd(NC));
            rescue_t = fm1_ms;
            nseen = 0;
        }
    }
    if (nseen < SEEN) seen[nseen++] = row;
    else { for (k = 1; k < SEEN; k++) seen[k - 1u] = seen[k]; seen[SEEN - 1u] = row; }
    hist[hhead] = row;
    hhead = (hhead + 1u) % HIST;
    gens++;
}

/* ================================================================== the patch -> the partials */
static uint32_t p_rule = 90, p_rate = 25, p_slew = 32, p_tilt = 100, p_mut = 3, p_np = 64, p_root = 33;
static int32_t p_str;                           /* stretch, per mille of the exponent: -30 .. +60 */
static uint32_t p_anchor = 1, p_stress;
static int32_t shown_lvl[NP];                   /* the levels drawn: the target, for the screen */

static void push_partials(void)
{
    uint32_t h, per = p_np / NC, kg = CA_SLEW_K[p_slew];
    uint32_t w[NP], sum = 0;
    int32_t q[NP];
    for (h = 1; h <= NP; h++) {
        uint32_t on, cell = (h - 1u) / per, wt = 0;
        int32_t st = (int32_t)(CA_L2[h - 1u] * 3u / 4u);   /* 12 * 256 * log2 h: the harmonic's interval */
        q[h - 1u] = (int32_t)p_root * 256 + st + st * p_str / 1000;
        on = h <= p_np && (p_stress || ((row >> cell) & 1u) || (h == 1u && p_anchor));
        if (on && q[h - 1u] <= Q_TOP) {
            uint32_t e = p_tilt * CA_L2[h - 1u] / 100u, ip = e >> 12, fp = (e & 4095u) >> 4;   /* h ^ -tilt */
            wt = ip >= 31u ? 0u : (2147483648u >> ip) / LUT_OCTFRAC[fp];                    /* Q15 */
            if (q[h - 1u] > Q_FADE) wt = wt * (uint32_t)(Q_TOP - q[h - 1u]) / (uint32_t)(Q_TOP - Q_FADE);   /* fade the top */
        }
        w[h - 1u] = wt;
        sum += wt;
    }
    for (h = 0; h < NP; h++) {                  /* the level: the sum held to 22000 at most (all in phase: no clip) */
        int32_t s = (int32_t)(sum < 40000u ? 40000u : sum), lvl = (int32_t)(w[h] * 22000u / (uint32_t)s);   /* (w <= 32768: fits 32 bits) */
        uint32_t semi = (uint32_t)cl(q[h], 0, Q_TOP) >> 8, k = CA_KF[semi < 132u ? semi : 131u];
        inc_v[h] = pitch_inc(q[h]);
        kh_v[h] = kg < k ? kg : k;              /* never faster than ~3 of its own cycles: no thump */
        tgt_v[h] = lvl;
        shown_lvl[h] = lvl;
    }
}

/* ================================================================== the screen: green phosphor */
#define PH_HI RGB(150, 255, 160)
#define PH_MID RGB(70, 200, 95)
#define PH_LO RGB(28, 105, 48)
#define PH_SEL RGB(8, 48, 20)
#define PH_BG RGB(0, 0, 0)
#define BODY_Y 22
#define BODY_H 218

/* ---- the CRT: every canvas is drawn as phosphor light (its green is the intensity), then cv_out() dims the light
 * toward the edges and corners (the vignette, radial), adds a soft halo around what is bright, darkens every other
 * line (the scanlines) and maps it all through a phosphor ramp, the brightest a little white. The background stays
 * black: a gradient there bands on the panel's 16-bit colour. */
static uint8_t crt_scr[240u * 240u] __attribute__((section(".pool")));   /* the whole screen's intensity, as drawn */
static uint8_t crt_cmb[240u * 240u] __attribute__((section(".pool")));   /* and with its halo: what the tube shows */
static uint8_t CRT_D2[240];                     /* (distance from the middle)^2: 0 .. 128 along an axis */
#define CRT_BULGE 30                            /* the tube's curve: ~6 px in at the corners, ~3 at the edges */
#define CRT_M 8                                 /* (the most a row moves: the rows around a strip redrawn too) */
static int32_t CRT_K2[240];                     /* BULGE x (distance from the middle)^2, per row / column */
static uint16_t CRT_LUT[2][256];                /* intensity -> the panel's pixel, even and odd lines */

/* the VDU settings (page 9, OCT): the phosphor and the brightness. Kept in the settings sector, not in a patch. */
#define NPHOS 4u
static const uint8_t PHOS_K[NPHOS][7][4] = {    /* intensity, r, g, b: each phosphor's ramp, its top a little white */
    {{0, 0, 0, 0}, {20, 12, 18, 14}, {60, 16, 62, 28}, {110, 28, 114, 50}, {170, 56, 192, 86}, {230, 122, 246, 142},
     {255, 196, 255, 204}},
    {{0, 0, 0, 0}, {20, 8, 18, 20}, {60, 10, 56, 64}, {110, 20, 108, 120}, {170, 44, 182, 198}, {230, 116, 236, 250},
     {255, 196, 250, 255}},
    {{0, 0, 0, 0}, {20, 20, 11, 3}, {60, 66, 34, 6}, {110, 124, 64, 10}, {170, 198, 110, 22}, {230, 250, 170, 66},
     {255, 255, 214, 140}},
    {{0, 0, 0, 0}, {20, 15, 16, 19}, {60, 54, 57, 64}, {110, 106, 110, 120}, {170, 176, 180, 192}, {230, 234, 238, 246},
     {255, 255, 255, 255}},
};
typedef struct { uint32_t boots; uint8_t phos, bright, eng, pad; } settings_t;   /* eng: bit 0 8-bit, bit 1 22 kHz */
static settings_t gset = {0, 0, 7, 0, 0};
static uint16_t CRT_TF[257];                    /* the light's gain over the radius: 256 in the middle .. 0 the corners */

static void crt_build(void)                     /* the ramp and the vignette from the VDU settings */
{
    const uint8_t (*K)[4] = PHOS_K[gset.phos < NPHOS ? gset.phos : 0u];
    uint32_t i, k, br = gset.bright > 10u ? 10u : gset.bright;
    uint32_t gc = 116u + br * 20u, ge = gc * br * br * 90u / 10000u;   /* the middle's gain; the corners' (dims first) */
    for (i = 0; i <= 256u; i++) CRT_TF[i] = (uint16_t)(ge + (gc - ge) * i / 256u);
    for (i = 0; i < 256u; i++) {
        uint32_t r, g, b, lr, lg, lb;
        for (k = 1; K[k][0] < i; k++) {}
        {
            uint32_t a = K[k - 1][0], z = K[k][0], f = z > a ? (i - a) * 256u / (z - a) : 0u;
            r = (K[k - 1][1] * (256u - f) + K[k][1] * f) >> 8;
            g = (K[k - 1][2] * (256u - f) + K[k][2] * f) >> 8;
            b = (K[k - 1][3] * (256u - f) + K[k][3] * f) >> 8;
        }
        lr = r * 70u / 100u; lg = g * 70u / 100u; lb = b * 70u / 100u;   /* the scanline: 70 % */
        CRT_LUT[0][i] = swap16(RGB(r, g, b));
        CRT_LUT[1][i] = swap16(RGB(lr, lg, lb));
    }
}
static void crt_init(void)
{
    uint32_t i;
    for (i = 0; i < 240u; i++) {
        int32_t d = 2 * (int32_t)i + 1 - 240;  /* -239 .. 239 */
        CRT_D2[i] = (uint8_t)((d * d * 128) / (239 * 239));
        CRT_K2[i] = CRT_BULGE * ((int32_t)i - 120) * ((int32_t)i - 120);
    }
    crt_build();
}

static void cv_out(uint32_t y0)                 /* the canvas (240 wide) through the CRT, to the screen at y0 */
{
    uint32_t w = cv_w, h = cv_h, x, y, r0, r1, oy, o0, o1;
    uint8_t hb[3][240];                         /* the halo's rows (a 3 x 3 box, rows first), rolling */
    if (w != 240u || y0 >= 240u) { cv_blit(0, y0); return; }
    if (y0 + h > 240u) h = 240u - y0;
    for (y = 0; y < h; y++) {                   /* 1: the strip into the screen's picture */
        uint8_t *a = &crt_scr[(y0 + y) * 240u];
        const uint16_t *px = &cv_px[y * w];
        for (x = 0; x < 240u; x++) a[x] = (uint8_t)(((swap16(px[x]) >> 5) & 63u) << 2);
    }
    r0 = y0 ? y0 - 1u : 0u;                     /* 2: its halo, and one row each side (their halo changed) */
    r1 = y0 + h < 240u ? y0 + h : 239u;
    for (y = (r0 ? r0 - 1u : 0u); y <= r1 + 1u && y < 240u; y++) {
        const uint8_t *a = &crt_scr[y * 240u];
        uint8_t *b = hb[y % 3u];
        b[0] = (uint8_t)((a[0] * 2u + a[1]) * 85u >> 8);
        for (x = 1; x + 1u < 240u; x++) b[x] = (uint8_t)(((uint32_t)a[x - 1] + a[x] + a[x + 1]) * 85u >> 8);
        b[239] = (uint8_t)((a[238] + a[239] * 2u) * 85u >> 8);
        {                                       /* the row before this one now has its three halo rows */
            uint32_t c = y - 1u;
            if (y >= 1u && c >= r0 && c <= r1) {
                const uint8_t *bu = hb[(c ? c - 1u : 0u) % 3u], *bm = hb[c % 3u], *bd = hb[y % 3u];
                const uint8_t *s0 = &crt_scr[c * 240u];
                uint8_t *m = &crt_cmb[c * 240u];
                for (x = 0; x < 240u; x++) {
                    uint32_t v = s0[x] + (((uint32_t)bu[x] + bm[x] + bd[x]) * 30u >> 8);
                    m[x] = (uint8_t)(v > 255u ? 255u : v);
                }
            }
        }
    }
    if (r1 == 239u) {                           /* (the last row: no row below it) */
        const uint8_t *bu = hb[238u % 3u], *bm = hb[239u % 3u];
        const uint8_t *s0 = &crt_scr[239u * 240u];
        uint8_t *m = &crt_cmb[239u * 240u];
        for (x = 0; x < 240u; x++) {
            uint32_t v = s0[x] + (((uint32_t)bu[x] + 2u * bm[x]) * 30u >> 8);
            m[x] = (uint8_t)(v > 255u ? 255u : v);
        }
    }
    /* 3: through the tube: each pixel looks a little further out (the bulge: the picture curves in at the edges),
     * read between pixels (no lost strokes), dimmed toward the corners, the scanlines straight across the glass */
    o0 = y0 > CRT_M + 1u ? y0 - CRT_M - 1u : 0u;
    o1 = y0 + h + CRT_M + 1u < 240u ? y0 + h + CRT_M + 1u : 240u;
    for (oy = o0; oy < o1;) {
        uint32_t n = o1 - oy > CV_MAX / 240u ? CV_MAX / 240u : o1 - oy, j;
        lcd_sync();                             /* (the strip before may still be going out from this buffer) */
        for (j = 0; j < n; j++, oy++) {
            const uint16_t *lut = CRT_LUT[oy & 1u];
            uint16_t *px = &cv_px[j * 240u];
            int32_t dy = (int32_t)oy - 120, ky = CRT_K2[oy];
            uint32_t ddy = CRT_D2[oy];
            for (x = 0; x < 240u; x++) {
                int32_t k2 = CRT_K2[x] + ky, dx = (int32_t)x - 120;
                int32_t sx = ((int32_t)x << 4) + ((dx * k2) >> 20), sy = ((int32_t)oy << 4) + ((dy * k2) >> 20);   /* Q4 */
                int32_t ix = sx >> 4, iy = sy >> 4;
                uint32_t v = 0;
                if (ix >= 0 && iy >= 0 && ix < 239 && iy < 239) {
                    const uint8_t *m = &crt_cmb[(uint32_t)iy * 240u + (uint32_t)ix];
                    uint32_t fx, fy, t, b;
                    if (!(m[0] | m[1] | m[240] | m[241])) { px[x] = lut[0]; continue; }   /* (dark glass: most of it) */
                    fx = (uint32_t)sx & 15u; fy = (uint32_t)sy & 15u;
                    t = m[0] * (16u - fx) + m[1] * fx; b = m[240] * (16u - fx) + m[241] * fx;
                    v = (t * (16u - fy) + b * fy) >> 8;
                    v = v * CRT_TF[256u - CRT_D2[x] - ddy] >> 8;   /* the light: dimmer toward the corners */
                }
                px[x] = lut[v > 255u ? 255u : v];
            }
        }
        cv_w = 240u; cv_h = n;
        cv_blit(0, oy - n);
    }
}


static void s_cat(char *s, const char *t) { while (*s) s++; while (*t) *s++ = *t++; *s = 0; }
static void s_int(char *s, int32_t v)
{
    char b[12];
    int i = 0;
    uint32_t u;
    while (*s) s++;
    if (v < 0) { *s++ = '-'; u = (uint32_t)-v; } else u = (uint32_t)v;
    do { b[i++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (i) *s++ = b[--i];
    *s = 0;
}
static void s_dec(char *s, uint32_t v, uint32_t div, uint32_t places)   /* v / div with places decimals */
{
    uint32_t p = 1, k;
    for (k = 0; k < places; k++) p *= 10u;
    s_int(s, (int32_t)(v / div));
    if (places) {
        uint32_t f = (v % div) * p / div;
        char b[6];
        s_cat(s, ".");
        for (k = places; k; k--) { b[k - 1u] = (char)('0' + f % 10u); f /= 10u; }
        b[places] = 0;
        s_cat(s, b);
    }
}
static void s_note(char *s, uint32_t m)
{
    static const char *const NN[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    s_cat(s, NN[m % 12u]);
    s_int(s, (int32_t)(m / 12u) - 1);
}

static uint32_t isqrt32(uint32_t v) { uint32_t r = 0, b = 1u << 30; while (b > v) b >>= 2; while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; } return r; }
static uint32_t cpu_avg, cpu_peak, cpu_max;     /* in 1/10 per cent of the 5.8 ms */
#define BUDGET_US 5805u                         /* 256 frames at 44.1 kHz */

static void draw_header(void)
{
    char s[40] = "CELLULAR TEST " CA_VERSION;
    cv_begin(240, BODY_Y, PH_BG);
    cv_text(4, 2, &FONT_S, s, PH_HI);
    cv_rect(0, 19, 240, 1, PH_MID);
    cv_out(0);
}
static void cpu_text(char *s, const char *lab, uint32_t v)
{
    s_cat(s, lab);
    s_dec(s, v, 10u, 1u);
    s_cat(s, " ");
}
static void draw_body(void)
{
    uint32_t y, c, h;
    char s[48];
    cv_begin(240, BODY_H, PH_BG);
    for (y = 0; y < HIST; y++) {                /* the cells: oldest at the top */
        uint32_t r = hist[(hhead + y) % HIST], age = HIST - 1u - y;
        uint16_t col = age == 0u ? PH_HI : age < 10u ? PH_MID : PH_LO;
        for (c = 0; c < NC; c++)
            if ((r >> c) & 1u) cv_rect(4 + (int32_t)c * 6, 4 + (int32_t)y * 4, 5, 3, col);
    }
    cv_rect(102, 4, 1, 120, PH_LO);
    {                                           /* the partials: 64 bars, 2 px each, scaled to the loudest */
        int32_t mx = 1;
        for (h = 0; h < NP; h++) if (shown_lvl[h] > mx) mx = shown_lvl[h];
        for (h = 0; h < NP; h++) {
            int32_t hgt = (int32_t)isqrt32((uint32_t)(shown_lvl[h] * 1000 / mx) * 14u), x = 108 + (int32_t)h * 2;   /* (a square root: the quiet ones show) */
            if (h >= p_np) { cv_rect(x, 122, 1, 2, PH_LO); continue; }
            if (hgt > 0) cv_rect(x, 124 - hgt, 1, hgt, h == 0u ? PH_HI : PH_MID);
            else cv_rect(x, 123, 1, 1, PH_LO);
        }
    }
    s[0] = 0; s_cat(s, "RULE "); s_int(s, (int32_t)p_rule); s_cat(s, "   MUT "); s_int(s, (int32_t)p_mut); s_cat(s, "%");
    cv_text(4, 130, &FONT_S, s, PH_MID);
    s[0] = 0; s_cat(s, "CLK "); s_dec(s, 100000u / CA_RATE_MS[p_rate], 100u, 2u); s_cat(s, "/S SLEW ");
    s_dec(s, CA_SLEW_MS[p_slew], 1000u, CA_SLEW_MS[p_slew] < 10000u ? 2u : 1u); s_cat(s, "S");
    cv_text(4, 146, &FONT_S, s, PH_MID);
    s[0] = 0; s_cat(s, "TILT "); s_dec(s, p_tilt, 100u, 2u); s_cat(s, "  STR ");
    s_cat(s, p_str < 0 ? "-" : "+"); s_dec(s, (uint32_t)(p_str < 0 ? -p_str : p_str), 1000u, 3u);
    cv_text(4, 162, &FONT_S, s, PH_MID);
    s[0] = 0; s_cat(s, "ROOT "); s_note(s, p_root); s_cat(s, "  "); s_int(s, (int32_t)p_np); s_cat(s, " PARTIALS");
    cv_text(4, 178, &FONT_S, s, PH_MID);
    if (p_stress) cv_text(4, 114, &FONT_S, "STRESS", PH_HI);
    else if (fm1_ms - rescue_t < 800u && rescue_t) { s[0] = 0; s_cat(s, "RESCUE "); s_cat(s, rescue_why); cv_text(4, 114, &FONT_S, s, PH_HI); }
    if (!p_anchor) cv_text(180, 178, &FONT_S, "NO F0", PH_LO);
    s[0] = 0; cpu_text(s, "CPU% ", cpu_avg); cpu_text(s, "PK ", cpu_peak); cpu_text(s, "MX ", cpu_max);
    cv_text(4, 196, &FONT_S, s, PH_HI);
    {                                           /* the bar: the average solid, the peak a tick */
        int32_t wa = (int32_t)(cpu_avg > 1000u ? 1000u : cpu_avg) * 232 / 1000, wp = (int32_t)(cpu_peak > 1000u ? 1000u : cpu_peak) * 232 / 1000;
        cv_rect(4, 214, 232, 1, PH_LO);
        cv_rect(4, 212, wa, 4, PH_MID);
        cv_rect(4 + wp, 210, 2, 8, PH_HI);
        cv_rect(4 + 116, 210, 1, 8, PH_LO);     /* (half the budget: the line to stay under) */
    }
    cv_out(BODY_Y);
}

/* ---- the boot screen */
static void big_text(int32_t x, int32_t y, const char *s, int32_t sx, int32_t sy, uint16_t c)
{
    for (; *s; s++, x += 8 * sx) {
        uint32_t ch = (uint8_t)*s, r, k;
        if (ch < 32u || ch > 126u) ch = '?';
        for (r = 0; r < 8u; r++) {
            uint32_t bits = FONT8_RAW[(ch - 32u) * 8u + r];
            for (k = 0; k < 8u; k++)
                if (bits >> k & 1u) cv_rect(x + (int32_t)k * sx, y + (int32_t)r * sy, sx, sy, c);
        }
    }
}
static void boot_line(int32_t y, const char *t, uint16_t c) { cv_text((240 - text_w(&FONT_S, t)) / 2, y, &FONT_S, t, c); }
static void draw_boot(void)
{
    cv_begin(240, BODY_Y, PH_BG);
    cv_out(0);
    cv_begin(240, BODY_H, PH_BG);
    big_text(24, 20, "CELLULAR", 3, 4, PH_HI);
    cv_rect(16, 60, 208, 2, PH_MID);
    boot_line(70, "Additive engine test " CA_VERSION, PH_MID);
    boot_line(88, "by zeptosecond DSP", PH_MID);
    boot_line(112, "Written by machine", PH_LO);
    boot_line(128, "with human guidance", PH_LO);
    boot_line(170, "PARTIALS ........  64", PH_MID);
    boot_line(186, "CELLS ...........  16", PH_MID);
    boot_line(202, "22 KHZ  12 BIT", PH_LO);
    cv_out(BODY_Y);
}

/* ================================================================== the main loop */
static void app_main(void)
{
    uint32_t prev_keys = 0, t_ca = 0, t_draw = 0, t_cpu = 0, i, b, rel, dirty = 1, mute_until = 0;
    int32_t oct = -2, e;

    for (i = 0; i < (1u << SIN_BITS); i++) SINT[i] = (int16_t)sine(i << (32u - SIN_BITS));
    crt_init();
    draw_boot();
#ifndef DINGHY_HOST
    rnd_s ^= fm1_ticks() * 0xC2B2AE35u;
    if (!rnd_s) rnd_s = 1u;
#endif
    ca_seed();
    for (i = 0; i < HIST; i++) ca_step(p_rule, p_mut);
    { uint32_t t0 = fm1_ms; while (fm1_ms - t0 < 1200u) platform_poll(); }
    draw_header();
    push_partials();

    for (;;) {
        uint32_t keys, down;
        platform_poll();
        { uint8_t st, d1, d2; while (midi_get(&st, &d1, &d2)) {} }   /* (nothing listens yet: keep the queue empty) */

        b = fm1_input_edges(&rel);
        if (b & (1u << BTN_PLAY)) { p_stress ^= 1u; cpu_max = 0; mute_until = fm1_ms + 300u; dirty = 1; }
        if (b & (1u << BTN_SEL)) { ca_seed(); dirty = 1; }
        if (b & (1u << BTN_REC)) { p_anchor ^= 1u; dirty = 1; }
        if (b & (1u << BTN_OCT_DOWN) && oct > -3) oct--;
        if (b & (1u << BTN_OCT_UP) && oct < 1) oct++;
        if ((e = fm1_enc_take(ENC_KNOB1)) != 0) { p_rule = (uint32_t)cl((int32_t)p_rule + e, 0, 255); dirty = 1; }
        if ((e = fm1_enc_take(ENC_KNOB2)) != 0) { p_rate = (uint32_t)cl((int32_t)p_rate + e, 0, 63); dirty = 1; }
        if ((e = fm1_enc_take(ENC_KNOB3)) != 0) { p_slew = (uint32_t)cl((int32_t)p_slew + e, 0, 63); dirty = 1; }
        if ((e = fm1_enc_take(ENC_KNOB4)) != 0) { p_tilt = (uint32_t)cl((int32_t)p_tilt + 5 * e, 0, 250); dirty = 1; }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) { p_mut = (uint32_t)cl((int32_t)p_mut + e, 0, 30); dirty = 1; }
        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) { p_str = cl(p_str + 2 * e, -30, 60); dirty = 1; }
        if ((e = fm1_enc_take(ENC_SELECT)) != 0) {
            p_np = e > 0 ? (p_np < 64u ? p_np * 2u : 64u) : (p_np > 16u ? p_np / 2u : 16u);
            cpu_max = 0; dirty = 1;
        }
        keys = fm1_in.notes;
        down = keys & ~prev_keys;
        prev_keys = keys;
        if (down) {                             /* the lowest key pressed: the root */
            for (i = 0; i < 27u; i++) if ((down >> i) & 1u) break;
            p_root = (uint32_t)cl(53 + 12 * oct + (int32_t)i, 21, 72);
            dirty = 1;
        }

        if (fm1_ms - t_ca >= CA_RATE_MS[p_rate]) {   /* the automaton's next row */
            t_ca = fm1_ms;
            ca_step(p_rule, p_mut);
            dirty = 1;
        }
        if (dirty) { push_partials(); dirty = 0; }

        if (fm1_ms - t_cpu >= 500u) {           /* the CPU line: this half second */
            uint32_t n = us_n, sum = us_sum, pk = us_peak;
            t_cpu = fm1_ms;
            us_sum = 0; us_n = 0; us_peak = 0;
            if (n && (int32_t)(fm1_ms - mute_until) > 0) {
                cpu_avg = sum / n * 1000u / BUDGET_US;
                cpu_peak = pk * 1000u / BUDGET_US;
                if (cpu_peak > cpu_max) cpu_max = cpu_peak;
            }
        }
        if (fm1_ms - t_draw >= 66u) {           /* ~15 frames a second */
            t_draw = fm1_ms;
            draw_body();
            for (i = 0; i < 16u; i++) fm1_led_key(KEY_LED(i), (row >> i) & 1u);
            fm1_led_key(BTN_PLAY, (int)p_stress);
            fm1_led_key(BTN_REC, (int)p_anchor);
        }
    }
}
