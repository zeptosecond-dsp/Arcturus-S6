/* SPDX-License-Identifier: GPL-3.0-only */
/* FM-1 wavetable synth: test build 0.2, on Dinghy (zvenson), on the platform of Felucca (Hügelton Instruments)
 * as SLOOP (isod89) carries it.
 *
 * Four PPG-style wavetable oscillators (1 and 2 phase modulate each other, A and B have wavefolders), six
 * Serge-style function generators, and a mod matrix from the FGs to the oscillators and the FGs' own times.
 * Pages in the manner of the Fairlight CMI, green phosphor on black:
 *
 *   HOME   PAGE 1 INDEX         SELECT: choose a page, SEL: open it
 *   EDIT   PAGE 2 OSCILLATORS   SELECT osc, ALGORITHM pitch (SEL: fine), K1 wave, K2 PM / fold (SEL on A, B:
 *                               threshold), K3 hi cut, K4 level, PRESETS pan; keys: the selected osc takes the note
 *   SEL+EDIT PAGE 3 WAVEFORM    the selected osc's table in 3D, its wave now in front; SELECT osc, K1 wave
 *   ENV    PAGE 4 FUNCTION GEN  SELECT FG, K1 rise, K2 fall, K3 curve, K4 mode; PLAY held: the FG's gate
 *   ARP    PAGE 7 MOD MATRIX    K1 source, K2 destination, K3 depth (turning an empty cell makes a pin),
 *                               PLAY: pin on / off
 *   the keys' LEDs follow the page: the pitch stack, the FG's slope, the source's level
 *
 * Rules of the road (the chip has no FPU and no libgcc): integers only, no float, no 64-bit division.
 * tests/host.c runs this file on a computer: the screen as a PNG, the sound as a WAV. */
#define APP_NAME "FM1 WAVETABLE"
#define CV_MAX (240u * 218u)                    /* the page body's canvas */
#include "platform.c"
#include "wt_tables.h"
#include "luts.h"

static int32_t cl(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }

/* ================================================================== the engine (audio interrupt) */
#define NO 4u                                   /* oscillators */
#define NFG 6u                                  /* function generators */
#define NSRC NFG                                /* mod sources (more to come: SSGs, the quad LFO) */
#define SUB 32u                                 /* frames between modulation updates (1378 Hz) */
enum { P_PITCH, P_WAVE, P_MOD, P_THR, P_HICUT, P_LEVEL, P_PAN, NP };
#define D_FG(k, fall) (NO * NP + 2u * (k) + (fall))   /* an FG's rise / fall time as a destination */
#define NDST (NO * NP + 2u * NFG)

typedef struct {                                /* the knobs, set by the main loop */
    volatile int32_t pitch_q8;                  /* note * 256 + fine, 1/256 semitone */
    volatile uint8_t pos, mod, thr, hicut, level, pan;
} osc_par_t;
typedef struct {                                /* owned by the interrupt */
    uint32_t ph;
    volatile int32_t posq;                      /* wave position now, Q8 waves (the waveform page reads it) */
    int32_t amp, lp1, lp2, last;
} osc_st_t;
static osc_par_t opar[NO];
static osc_st_t ost[NO];

enum { FG_IDLE, FG_RISE, FG_HOLD, FG_FALL };
enum { M_CYCLE, M_GATE, M_ONCE };
enum { CV_LIN, CV_EXP, CV_LOG };
#define FG_TOP (1 << 24)
typedef struct { volatile uint8_t rise, fall, curve, mode, gate; } fg_par_t;
typedef struct { int32_t x; uint8_t stage, gate_prev, eoc; } fg_st_t;
static fg_par_t fgp[NFG];
static fg_st_t fgs[NFG];
static volatile int32_t fg_out[NFG];            /* 0..32767 */
static volatile uint8_t fg_stage_v[NFG], fg_eoc_v[NFG];

static volatile int8_t depth[NSRC][NDST];       /* the mod matrix: 0 = no pin */
static volatile uint32_t audio_us, audio_us_max;

static const uint32_t INC_C9[12] = {            /* MIDI 120..131 (C9..B9); lower octaves shift right */
    815363807u, 863847862u, 915214929u, 969636441u, 1027294024u, 1088380105u,
    1153098554u, 1221665363u, 1294309365u, 1371273005u, 1452813141u, 1539201906u};

static uint32_t pitch_inc(int32_t q)            /* 1/256 semitone -> phase step */
{
    uint32_t s, f, inc;
    q = cl(q, 12 * 256, 119 * 256 + 255);
    s = (uint32_t)q >> 8;
    f = (uint32_t)q & 255u;
    inc = INC_C9[s % 12u] >> (10u - s / 12u);
    return (uint32_t)(((uint64_t)inc * LUT_SEMIFRAC[f]) >> 16);
}

static int32_t fg_shape(int32_t x, uint32_t curve)   /* position Q24 -> level 0..32767 */
{
    uint32_t i = (uint32_t)x >> 16, f = ((uint32_t)x >> 8) & 255u;
    const uint16_t *t;
    if (curve == CV_LIN) return x >= FG_TOP ? 32767 : x >> 9;
    t = curve == CV_EXP ? LUT_SHAPE_EXP : LUT_SHAPE_LOG;
    if (i >= 256u) return t[256];
    return (int32_t)t[i] + ((((int32_t)t[i + 1] - (int32_t)t[i]) * (int32_t)f) >> 8);
}

/* one FG, one step. The position runs 0..TOP and back; the level is shape(position), so a slope that turns
 * early (a gate let go mid-rise) turns from where it is, as a Serge slope does. */
static void fg_step(uint32_t k, uint32_t rise_k, uint32_t fall_k)
{
    fg_st_t *s = &fgs[k];
    uint32_t mode = fgp[k].mode, g = fgp[k].gate;
    if (s->eoc) s->eoc--;
    if (mode == M_CYCLE) {
        if (s->stage == FG_IDLE || s->stage == FG_HOLD) s->stage = FG_RISE;
    } else if (mode == M_GATE) {
        if (g && (s->stage == FG_IDLE || s->stage == FG_FALL)) s->stage = FG_RISE;
        else if (!g && (s->stage == FG_RISE || s->stage == FG_HOLD)) s->stage = FG_FALL;
    } else {                                    /* ONCE: a gate's start fires a whole rise and fall */
        if (g && !s->gate_prev) s->stage = FG_RISE;
        if (s->stage == FG_HOLD) s->stage = FG_FALL;
    }
    s->gate_prev = (uint8_t)g;
    if (s->stage == FG_RISE) {
        s->x += (int32_t)LUT_FGINC[rise_k];
        if (s->x >= FG_TOP) { s->x = FG_TOP; s->stage = mode == M_GATE ? FG_HOLD : FG_FALL; }
    } else if (s->stage == FG_FALL) {
        s->x -= (int32_t)LUT_FGINC[fall_k];
        if (s->x <= 0) { s->x = 0; s->eoc = 7; s->stage = mode == M_CYCLE ? FG_RISE : FG_IDLE; }   /* EOC ~5 ms */
    }
    fg_out[k] = fg_shape(s->x, fgp[k].curve);
    fg_stage_v[k] = s->stage;
    fg_eoc_v[k] = s->eoc != 0;
}

static void render(int32_t *out)                /* SUB frames */
{
    int32_t acc[NDST], src[NSRC];
    uint32_t o, k, d, i;
    int32_t pm[NO], fg_g[NO], fg_t[NO], fg_mk[NO], hc[NO], amp_t[NO], amp_d[NO], pl[NO], pr[NO];
    uint32_t inc[NO];
    const int8_t *tab = &WT[0][0][0];

    for (d = 0; d < NDST; d++) acc[d] = 0;      /* the matrix: sum of source x depth, per destination */
    for (k = 0; k < NSRC; k++) {
        int32_t v = fg_out[k];
        src[k] = v;
        if (!v) continue;
        for (d = 0; d < NDST; d++) {
            int32_t dep = depth[k][d];
            if (dep) acc[d] += v * dep;
        }
    }
    for (d = 0; d < NDST; d++) acc[d] >>= 6;    /* -> knob units, Q8: a full source at depth 63 is ~ a full knob */
    (void)src;

    for (k = 0; k < NFG; k++)
        fg_step(k, (uint32_t)cl(fgp[k].rise + (acc[D_FG(k, 0)] >> 8), 0, 127),
                   (uint32_t)cl(fgp[k].fall + (acc[D_FG(k, 1)] >> 8), 0, 127));

    for (o = 0; o < NO; o++) {
        const osc_par_t *p = &opar[o];
        const int32_t *a = &acc[o * NP];
        int32_t q, target, dpos;
        inc[o] = pitch_inc(p->pitch_q8 + ((a[P_PITCH] * 3) >> 3));          /* full depth: ~4 octaves */
        target = (cl(p->pos * 256 + a[P_WAVE], 0, 127 * 256) * 127) >> 8;   /* -> Q8 waves, 0..63 */
        dpos = target - ost[o].posq;
        ost[o].posq += dpos > 128 ? 128 : dpos < -128 ? -128 : dpos;
        q = cl(p->mod * 256 + a[P_MOD], 0, 127 * 256);
        pm[o] = q >> 4;                                                     /* PM: 0..2032 */
        fg_g[o] = 256 + ((q * 14) >> 8);                                    /* fold drive x1..x8, Q8 */
        fg_t[o] = 4096 + ((cl(p->thr * 256 + a[P_THR], 0, 127 * 256) * 224) >> 8);   /* threshold, Q15 */
        fg_mk[o] = (int32_t)((32767u << 12) / (uint32_t)fg_t[o]);         /* make-up gain, Q12 */
        hc[o] = cl(p->hicut + (a[P_HICUT] >> 8), 0, 127);
        amp_t[o] = LUT_LEVEL[cl(p->level + (a[P_LEVEL] >> 8), 0, 127)];
        amp_d[o] = (amp_t[o] - ost[o].amp) >> 5;                            /* a ramp over the SUB frames */
        k = (uint32_t)cl(p->pan + (a[P_PAN] >> 8), 0, 127);
        pl[o] = LUT_PAN_L[k];
        pr[o] = LUT_PAN_R[k];
    }

    for (i = 0; i < SUB; i++) {
        int32_t l = 0, r = 0, prev0 = ost[0].last, prev1 = ost[1].last;    /* 1 and 2 hear each other a sample late */
        for (o = 0; o < NO; o++) {
            osc_st_t *s = &ost[o];
            uint32_t ph, idx, w, frac;
            int32_t x, v;
            const int8_t *w0, *w1;
            s->ph += inc[o];
            ph = s->ph;
            if (o == 0u) ph += (uint32_t)(prev1 * pm[0]) << 6;           /* phase modulation (wraps: fine) */
            else if (o == 1u) ph += (uint32_t)(prev0 * pm[1]) << 6;
            idx = ph >> 25;                      /* 128 samples a wave, no interpolation: PPG grit */
            w = (uint32_t)s->posq >> 8;
            frac = (uint32_t)s->posq & 255u;
            w0 = tab + w * WT_SAMPLES;
            w1 = w + 1u < WT_WAVES ? w0 + WT_SAMPLES : w0;
            x = (int32_t)w0[idx] * (int32_t)(256u - frac) + (int32_t)w1[idx] * (int32_t)frac;   /* +-32512 */
            if (o >= 2u) {                       /* the wavefolder */
                int32_t y = (x * fg_g[o]) >> 8, t = fg_t[o], n;
                for (n = 0; n < 24 && (y > t || y < -t); n++)
                    y = y > t ? 2 * t - y : -2 * t - y;
                x = (y * fg_mk[o]) >> 12;
            }
            s->last = x;
            if (hc[o] < 127) {                   /* the hi cut: two one-poles, 12 dB */
                int32_t c = LUT_HICUT[hc[o]];
                s->lp1 += ((x - s->lp1) * c) >> 15;
                s->lp2 += ((s->lp1 - s->lp2) * c) >> 15;
                x = s->lp2;
            }
            s->amp += amp_d[o];
            v = (x * s->amp) >> 15;              /* <= 10000 */
            l += (v * pl[o]) >> 15;
            r += (v * pr[o]) >> 15;
        }
        out[2u * i] += l;
        out[2u * i + 1u] += r;
    }
    for (o = 0; o < NO; o++) ost[o].amp = amp_t[o];
}

static void app_audio(int32_t *out, uint32_t n)
{
    uint32_t b;
#ifndef DINGHY_HOST
    uint32_t t0 = fm1_ticks();
#endif
    for (b = 0; b + SUB <= n; b += SUB)
        render(out + 2u * b);
#ifndef DINGHY_HOST
    audio_us = (fm1_ticks() - t0) / FM1_TICKS_PER_US;
    if (audio_us > audio_us_max) audio_us_max = audio_us;
#endif
}

/* ================================================================== the patch (main loop) */
typedef struct { uint8_t note; int8_t fine; uint8_t pos, mod, thr, hicut, level, pan; } patch_t;
static patch_t opatch[NO] = {                   /* a drone to start from: C2, G2, C3, G3 */
    {36,  0, 20, 20,   0,  90, 100,  40},
    {43,  4, 50, 10,   0,  80,  85,  88},
    {48, -3, 10, 30,  90,  70,  80,  20},
    {55,  2, 64, 15, 110,  60,  70, 107},
};
typedef struct { uint8_t rise, fall, curve, mode; } fgpatch_t;
static fgpatch_t fgpatch[NFG] = {               /* four cycling, two waiting for a gate */
    {70, 80, CV_EXP, M_CYCLE}, {85, 60, CV_LOG, M_CYCLE}, {100, 100, CV_LIN, M_CYCLE},
    {45, 55, CV_EXP, M_CYCLE}, {20, 70, CV_EXP, M_GATE}, {10, 50, CV_LOG, M_ONCE},
};

static void push_osc(uint32_t o)
{
    patch_t *q = &opatch[o];
    opar[o].pitch_q8 = (int32_t)q->note * 256 + (int32_t)q->fine * 256 / 100;
    opar[o].pos = q->pos;
    opar[o].mod = q->mod;
    opar[o].thr = q->thr;
    opar[o].hicut = q->hicut;
    opar[o].level = q->level;
    opar[o].pan = q->pan;
}
static void push_fg(uint32_t k)
{
    fgp[k].rise = fgpatch[k].rise;
    fgp[k].fall = fgpatch[k].fall;
    fgp[k].curve = fgpatch[k].curve;
    fgp[k].mode = fgpatch[k].mode;
}

/* the matrix's destinations as the page lists them (no threshold on 1 and 2: they have no folder) */
static const uint8_t DLIST[] = {
    0 * NP + P_PITCH, 0 * NP + P_WAVE, 0 * NP + P_MOD, 0 * NP + P_HICUT, 0 * NP + P_LEVEL, 0 * NP + P_PAN,
    1 * NP + P_PITCH, 1 * NP + P_WAVE, 1 * NP + P_MOD, 1 * NP + P_HICUT, 1 * NP + P_LEVEL, 1 * NP + P_PAN,
    2 * NP + P_PITCH, 2 * NP + P_WAVE, 2 * NP + P_MOD, 2 * NP + P_THR, 2 * NP + P_HICUT, 2 * NP + P_LEVEL, 2 * NP + P_PAN,
    3 * NP + P_PITCH, 3 * NP + P_WAVE, 3 * NP + P_MOD, 3 * NP + P_THR, 3 * NP + P_HICUT, 3 * NP + P_LEVEL, 3 * NP + P_PAN,
    D_FG(0, 0), D_FG(0, 1), D_FG(1, 0), D_FG(1, 1), D_FG(2, 0), D_FG(2, 1),
    D_FG(3, 0), D_FG(3, 1), D_FG(4, 0), D_FG(4, 1), D_FG(5, 0), D_FG(5, 1),
};
#define NDLIST (sizeof DLIST / sizeof DLIST[0])

/* ================================================================== the screen: green phosphor */
#define PH_HI RGB(150, 255, 160)
#define PH_MID RGB(70, 200, 95)
#define PH_LO RGB(28, 105, 48)
#define PH_SEL RGB(8, 48, 20)
#define PH_BG RGB(0, 0, 0)
#define BODY_Y 22
#define BODY_H 218
static const char *const ONAME[NO] = {"1", "2", "A", "B"};

static void s_cat(char *s, const char *t) { while (*s) s++; while (*t) *s++ = *t++; *s = 0; }
static void s_int(char *s, int32_t v, int sign)
{
    char t[8];
    uint32_t i = 0, u = (uint32_t)(v < 0 ? -v : v);
    while (*s) s++;
    if (v < 0) *s++ = '-'; else if (sign && v > 0) *s++ = '+';
    do { t[i++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (i) *s++ = t[--i];
    *s = 0;
}
static void s_int3(char *s, int32_t v)          /* right-aligned in 3 */
{
    char t[8] = "";
    uint32_t n;
    s_int(t, v, 0);
    for (n = 0; t[n]; n++) {}
    while (n++ < 3u) s_cat(s, " ");
    s_cat(s, t);
}
static void s_note(char *s, uint32_t note)
{
    static const char *const NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    s_cat(s, NAME[note % 12u]);
    s_int(s, (int32_t)(note / 12u) - 1, 0);
}
static void dst_name(char *s, uint32_t d)
{
    static const char *const PN[NP] = {"PITCH", "WAVE", "PM", "THRESH", "HI CUT", "LEVEL", "PAN"};
    s[0] = 0;
    if (d < NO * NP) {
        s_cat(s, ONAME[d / NP]);
        s_cat(s, " ");
        s_cat(s, d % NP == P_MOD && d / NP >= 2u ? "FOLD" : PN[d % NP]);
    } else {
        s_cat(s, "FG");
        s_int(s, (int32_t)(d - NO * NP) / 2 + 1, 0);
        s_cat(s, (d - NO * NP) & 1u ? " FALL" : " RISE");
    }
}

enum { PG_INDEX, PG_OSC, PG_WAVE, PG_FG, PG_SH, PG_GATE, PG_MOD, PG_GMAT, PG_GLOBAL, PG_PATCH, NPG };
static const char *const PG_NUM[NPG] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "P"};
static const char *const PG_NAME[NPG] = {"INDEX", "OSCILLATORS", "WAVEFORM", "FUNCTION GEN", "S&H / QUAD",
                                         "GATE GEN", "MOD MATRIX", "GATE MATRIX", "GLOBAL", "PATCHES"};
static const uint8_t PG_READY[NPG] = {1, 1, 1, 1, 0, 0, 1, 0, 0, 0};

static void draw_header(uint32_t pg, uint32_t cpu_pct)
{
    char s[40] = "PAGE ";
    s_cat(s, PG_NUM[pg]);
    s_cat(s, "  ");
    s_cat(s, PG_NAME[pg]);
    cv_begin(240, BODY_Y, PH_BG);
    cv_text(4, 2, &FONT_S, s, PH_HI);
    if (cpu_pct) {
        char c[12] = "";
        s_int(c, (int32_t)cpu_pct, 0);
        s_cat(c, "%");
        cv_text(236 - text_w(&FONT_S, c), 2, &FONT_S, c, PH_LO);
    }
    cv_rect(0, 19, 240, 1, PH_MID);
    cv_blit(0, 0);
}

static void draw_index(uint32_t cur)
{
    uint32_t i;
    cv_begin(240, BODY_H, PH_BG);
    for (i = 0; i < NPG; i++) {
        int32_t y = 6 + (int32_t)i * 18;
        char s[32] = "";
        uint16_t c = PG_READY[i] ? PH_MID : PH_LO;
        if (i == cur) { cv_rect(0, y - 1, 240, 18, PH_SEL); c = PH_HI; cv_text(4, y, &FONT_S, ">", PH_HI); }
        s_cat(s, PG_NUM[i]);
        s_cat(s, "   ");
        s_cat(s, PG_NAME[i]);
        cv_text(20, y, &FONT_S, s, c);
        if (!PG_READY[i]) cv_text(196, y, &FONT_S, "--", PH_LO);
    }
    cv_text(4, 196, &FONT_S, "SELECT: CHOOSE  SEL: OPEN", PH_LO);
    cv_blit(0, BODY_Y);
}

static void draw_soon(uint32_t pg)
{
    cv_begin(240, BODY_H, PH_BG);
    cv_text(4, 60, &FONT_S, PG_NAME[pg], PH_MID);
    cv_text(4, 84, &FONT_S, "NOT IN THIS BUILD YET", PH_LO);
    cv_text(4, 196, &FONT_S, "HOME: INDEX", PH_LO);
    cv_blit(0, BODY_Y);
}

static void draw_osc(uint32_t sel, int shift)
{
    static const char *const ROW[8] = {"NOTE", "FINE", "WAVE", "PM/FLD", "THRESH", "HI CUT", "LEVEL", "PAN"};
    uint32_t o, r;
    cv_begin(240, BODY_H, PH_BG);
    for (o = 0; o < NO; o++) {
        int32_t x = 64 + (int32_t)o * 44;
        if (o == sel) cv_rect(x - 2, 4, 42, 168, PH_SEL);
        cv_text(x + 14, 6, &FONT_S, ONAME[o], o == sel ? PH_HI : PH_LO);
    }
    for (r = 0; r < 8u; r++) {
        int32_t y = 28 + (int32_t)r * 18;
        int hot = shift ? (r == 1u || r == 4u) : (r != 1u && r != 4u);   /* the rows the knobs reach now */
        cv_text(2, y, &FONT_S, ROW[r], hot ? PH_MID : PH_LO);
        for (o = 0; o < NO; o++) {
            patch_t *q = &opatch[o];
            char s[12] = "";
            switch (r) {
            case 0: s_note(s, q->note); break;
            case 1: s_int(s, q->fine, 1); break;
            case 2: s_int(s, q->pos, 0); break;
            case 3: s_int(s, q->mod, 0); break;
            case 4: if (o < 2u) s_cat(s, "-"); else s_int(s, q->thr, 0); break;
            case 5: if (q->hicut >= 127u) s_cat(s, "OPN"); else s_int(s, q->hicut, 0); break;
            case 6: s_int(s, q->level, 0); break;
            default: s_int(s, (int32_t)q->pan - 64, 1); break;
            }
            cv_text(64 + (int32_t)o * 44, y, &FONT_S, s, o == sel ? PH_HI : PH_MID);
        }
    }
    cv_text(4, 196, &FONT_S, shift ? "SEL: FINE, THRESH" : "KEYS: SET NOTE  SEL: MORE", PH_LO);
    cv_blit(0, BODY_Y);
}

static int32_t wave_at(uint32_t w, uint32_t i) { return WT[0][w][i]; }

static void draw_wave(uint32_t sel, int32_t posq)
{
    int32_t w, i, f = posq & 255, wc = posq >> 8;
    char s[40] = "OSC ";
    cv_begin(240, BODY_H, PH_BG);
    for (w = 60; w >= 0; w -= 4) {              /* the table in 3D, back to front, as the Waveterm drew it */
        int32_t x0 = 8 + (w * 3) / 2, y0 = 150 - (w * 9) / 5, py = 0;
        for (i = 0; i < 128; i++) {
            int32_t y = y0 - ((wave_at((uint32_t)w, (uint32_t)i) * 28) >> 7);
            if (i) cv_line(x0 + i - 1, py, x0 + i, y, PH_LO);
            py = y;
        }
    }
    {                                           /* the wave playing now, bright, in its place */
        int32_t x0 = 8 + (wc * 3) / 2, y0 = 150 - (wc * 9) / 5, py = 0;
        uint32_t w1 = wc + 1 < (int32_t)WT_WAVES ? (uint32_t)wc + 1u : (uint32_t)wc;
        for (i = 0; i < 128; i++) {
            int32_t v = (wave_at((uint32_t)wc, (uint32_t)i) * (256 - f) + wave_at(w1, (uint32_t)i) * f) >> 8;
            int32_t y = y0 - ((v * 28) >> 7);
            if (i) { cv_line(x0 + i - 1, py, x0 + i, y, PH_HI); cv_line(x0 + i - 1, py + 1, x0 + i, y + 1, PH_HI); }
            py = y;
        }
    }
    s_cat(s, ONAME[sel]);
    s_cat(s, "   ");
    s_cat(s, WT_NAME[0]);
    s_cat(s, "   WAVE ");
    s_int(s, (posq * 127) / (63 * 256), 0);
    cv_text(4, 180, &FONT_S, s, PH_MID);
    cv_text(4, 198, &FONT_S, "SELECT: OSC  K1: WAVE", PH_LO);
    cv_blit(0, BODY_Y);
}

static void draw_fg(uint32_t sel)
{
    static const char *const CV[3] = {"LIN", "EXP", "LOG"}, *const MD[3] = {"CYCLE", "GATE ", "ONCE "};
    uint32_t k;
    fgpatch_t *q;
    int32_t x0 = 8, rw, hw, fw, top = 116, h = 64, i, px = 0, py = 0, dot_x, lvl, st;
    cv_begin(240, BODY_H, PH_BG);
    for (k = 0; k < NFG; k++) {
        int32_t y = 4 + (int32_t)k * 18, bar = (fg_out[k] * 54) >> 15;
        char s[40] = "";
        q = &fgpatch[k];
        if (k == sel) cv_rect(0, y - 1, 240, 18, PH_SEL);
        s_int(s, (int32_t)k + 1, 0);
        s_cat(s, " R");
        s_int3(s, q->rise);
        s_cat(s, " F");
        s_int3(s, q->fall);
        s_cat(s, " ");
        s_cat(s, CV[q->curve]);
        s_cat(s, " ");
        s_cat(s, MD[q->mode]);
        cv_text(2, y, &FONT_S, s, k == sel ? PH_HI : PH_MID);
        cv_rect(182, y + 4, 56, 8, PH_LO);
        cv_rect(183, y + 5, bar, 6, fg_eoc_v[k] ? PH_HI : PH_MID);
    }
    q = &fgpatch[sel];                          /* the selected slope, drawn: widths follow the time knobs */
    rw = 10 + (q->rise * 90) / 127;
    hw = q->mode == M_GATE ? 24 : 0;
    fw = 10 + (q->fall * 90) / 127;
    cv_rect(x0, top + h, 224, 1, PH_LO);
    for (i = 0; i <= rw + hw + fw; i++) {
        int32_t y, xx = x0 + i;
        if (i <= rw) y = fg_shape((i << 24) / rw, q->curve);
        else if (i <= rw + hw) y = 32767;
        else y = fg_shape(((rw + hw + fw - i) << 24) / fw, q->curve);
        y = top + h - ((y * h) >> 15);
        if (i) cv_line(px, py, xx, y, PH_MID);
        px = xx;
        py = y;
    }
    st = fg_stage_v[sel];                       /* where the FG is now */
    lvl = fg_out[sel];
    {
        int32_t xq = fgs[sel].x >> 12;          /* 0..4096 */
        dot_x = st == FG_RISE ? x0 + (rw * xq >> 12)
              : st == FG_HOLD ? x0 + rw + hw / 2
              : st == FG_FALL ? x0 + rw + hw + (fw * (4096 - xq) >> 12) : x0;
    }
    cv_rect(dot_x - 2, top + h - ((lvl * h) >> 15) - 2, 5, 5, PH_HI);
    if (fg_eoc_v[sel]) cv_text(200, top, &FONT_S, "EOC", PH_HI);
    cv_text(4, 198, &FONT_S, "K1 RISE K2 FALL K3 CRV K4 MOD", PH_LO);
    cv_blit(0, BODY_Y);
}

#define MROWS 9
static void draw_mod(uint32_t cs, uint32_t cr, uint32_t top)
{
    uint32_t c, r;
    char s[40] = "";
    cv_begin(240, BODY_H, PH_BG);
    for (c = 0; c < NSRC; c++) {
        char t[4] = "F";
        s_int(t, (int32_t)c + 1, 0);
        cv_text(98 + (int32_t)c * 24, 2, &FONT_S, t, c == cs ? PH_HI : PH_LO);
    }
    cv_rect(0, 19, 240, 1, PH_LO);
    for (r = 0; r < MROWS && top + r < NDLIST; r++) {
        uint32_t d = DLIST[top + r];
        int32_t y = 24 + (int32_t)r * 17;
        char n[16];
        dst_name(n, d);
        if (top + r == cr) cv_rect(0, y - 1, 92, 17, PH_SEL);
        cv_text(2, y, &FONT_S, n, top + r == cr ? PH_HI : PH_MID);
        for (c = 0; c < NSRC; c++) {
            int32_t x = 98 + (int32_t)c * 24, dep = depth[c][d];
            if (dep) {
                uint16_t col = fg_out[c] > 16384 ? PH_HI : PH_MID;   /* the pin glows with its source */
                if (dep > 0) cv_rect(x, y + 4, 9, 9, col);
                else { cv_rect(x, y + 4, 9, 1, col); cv_rect(x, y + 12, 9, 1, col); cv_rect(x, y + 4, 1, 9, col); cv_rect(x + 8, y + 4, 1, 9, col); }
            } else
                cv_rect(x + 4, y + 8, 1, 1, PH_LO);
            if (c == cs && top + r == cr) {
                cv_rect(x - 4, y, 17, 1, PH_HI); cv_rect(x - 4, y + 16, 17, 1, PH_HI);
                cv_rect(x - 4, y, 1, 17, PH_HI); cv_rect(x + 12, y, 1, 17, PH_HI);
            }
        }
    }
    s_cat(s, "FG");
    s_int(s, (int32_t)cs + 1, 0);
    s_cat(s, " > ");
    {
        char n[16];
        dst_name(n, DLIST[cr]);
        s_cat(s, n);
    }
    s_cat(s, "  ");
    if (depth[cs][DLIST[cr]]) s_int(s, depth[cs][DLIST[cr]], 1); else s_cat(s, "NO PIN");
    cv_text(4, 180, &FONT_S, s, PH_HI);
    cv_text(4, 198, &FONT_S, "K1 SRC K2 DST K3 AMT PLAY PIN", PH_LO);
    cv_blit(0, BODY_Y);
}

/* ================================================================== the main loop */
static void app_main(void)
{
    uint32_t o, k, sel = 0, fsel = 0, prev = 0, page = PG_OSC, icur = PG_OSC, mcs = 0, mcr = 0, mtop = 0;
    uint32_t dirty = 1, hdr_dirty = 1, cpu_pct = 0, t_cpu = 0, t_live = 0;
    int32_t oct = -1, e, shift_drawn = -1;

    for (o = 0; o < NO; o++) push_osc(o);
    for (k = 0; k < NFG; k++) push_fg(k);
    depth[0][0 * NP + P_WAVE] = 40;             /* a few pins to hear the FGs at once */
    depth[1][2 * NP + P_MOD] = 30;
    depth[2][3 * NP + P_PAN] = 45;
    depth[2][1 * NP + P_MOD] = 20;
    depth[3][D_FG(0, 0)] = 20;                  /* FG4 bends FG1's rise: feedback */
    lcd_fill(0, 0, 240, 240, PH_BG);

    for (;;) {
        uint32_t rel, b, base, newpage;
        int shift, play;
        platform_poll();

        b = fm1_input_edges(&rel);
        shift = (fm1_in.buttons >> BTN_SEL) & 1u;
        play = (fm1_in.buttons >> BTN_PLAY) & 1u;
        newpage = page;
        if (b & (1u << BTN_HOME)) { newpage = PG_INDEX; icur = page; }
        if (b & (1u << BTN_EDIT)) newpage = shift ? PG_WAVE : PG_OSC;
        if (b & (1u << BTN_ENV)) newpage = shift ? PG_SH : PG_FG;
        if (b & (1u << BTN_LFO)) newpage = PG_GATE;
        if (b & (1u << BTN_ARP)) newpage = PG_MOD;
        if (b & (1u << BTN_SEQ)) newpage = PG_GMAT;
        if (b & (1u << BTN_GLO)) newpage = PG_GLOBAL;
        if (b & (1u << BTN_SAVE)) newpage = PG_PATCH;
        if (b & (1u << BTN_OCT_DOWN) && oct > -3) oct--;
        if (b & (1u << BTN_OCT_UP) && oct < 2) oct++;
        base = (uint32_t)(53 + 12 * oct);       /* key 0 = F3, shifted */

        if (page == PG_INDEX) {
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) { icur = (uint32_t)cl((int32_t)icur + e, 0, NPG - 1); dirty = 1; }
            if (b & (1u << BTN_SEL)) newpage = icur;
        }
        if (newpage != page) { page = newpage; dirty = hdr_dirty = 1; }

        for (k = 0; k < NFG; k++)               /* PLAY held on the FG page: the selected FG's gate */
            fgp[k].gate = (uint8_t)(page == PG_FG && k == fsel && play);

        if (page == PG_OSC || page == PG_WAVE) {
            patch_t *q;
            int ch = 0;
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) { sel = (uint32_t)cl((int32_t)sel + e, 0, NO - 1); dirty = 1; }
            q = &opatch[sel];
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) { q->pos = (uint8_t)cl(q->pos + 2 * e, 0, 127); ch = 1; }
            if (page == PG_OSC) {
                uint32_t keys = fm1_in.notes, down = keys & ~prev;
                prev = keys;
                if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {
                    if (shift) q->fine = (int8_t)cl(q->fine + e, -50, 50);
                    else q->note = (uint8_t)cl(q->note + e, 12, 108);
                    ch = 1;
                }
                if ((e = fm1_enc_take(ENC_KNOB2)) != 0) {
                    if (shift && sel >= 2u) q->thr = (uint8_t)cl(q->thr + 2 * e, 0, 127);
                    else q->mod = (uint8_t)cl(q->mod + 2 * e, 0, 127);
                    ch = 1;
                }
                if ((e = fm1_enc_take(ENC_KNOB3)) != 0) { q->hicut = (uint8_t)cl(q->hicut + 2 * e, 0, 127); ch = 1; }
                if ((e = fm1_enc_take(ENC_KNOB4)) != 0) { q->level = (uint8_t)cl(q->level + 2 * e, 0, 127); ch = 1; }
                if ((e = fm1_enc_take(ENC_PRESETS)) != 0) { q->pan = (uint8_t)cl(q->pan + 2 * e, 0, 127); ch = 1; }
                for (k = 0; k < 27u; k++)
                    if (down >> k & 1u) { q->note = (uint8_t)cl((int32_t)(base + k), 12, 108); ch = 1; }
            }
            if (ch) { push_osc(sel); dirty = 1; }
        } else if (page == PG_FG) {
            fgpatch_t *q;
            int ch = 0;
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) { fsel = (uint32_t)cl((int32_t)fsel + e, 0, NFG - 1); dirty = 1; }
            q = &fgpatch[fsel];
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) { q->rise = (uint8_t)cl(q->rise + e, 0, 127); ch = 1; }
            if ((e = fm1_enc_take(ENC_KNOB2)) != 0) { q->fall = (uint8_t)cl(q->fall + e, 0, 127); ch = 1; }
            if ((e = fm1_enc_take(ENC_KNOB3)) != 0) { q->curve = (uint8_t)cl(q->curve + (e > 0 ? 1 : -1), 0, 2); ch = 1; }
            if ((e = fm1_enc_take(ENC_KNOB4)) != 0) { q->mode = (uint8_t)cl(q->mode + (e > 0 ? 1 : -1), 0, 2); ch = 1; }
            if (ch) { push_fg(fsel); dirty = 1; }
        } else if (page == PG_MOD) {
            uint32_t d;
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) { mcs = (uint32_t)cl((int32_t)mcs + e, 0, NSRC - 1); dirty = 1; }
            if ((e = fm1_enc_take(ENC_KNOB2)) != 0 || (e = fm1_enc_take(ENC_SELECT)) != 0) {
                mcr = (uint32_t)cl((int32_t)mcr + e, 0, (int32_t)NDLIST - 1);
                if (mcr < mtop) mtop = mcr;
                if (mcr >= mtop + MROWS) mtop = mcr - MROWS + 1u;
                dirty = 1;
            }
            d = DLIST[mcr];
            if ((e = fm1_enc_take(ENC_KNOB3)) != 0) { depth[mcs][d] = (int8_t)cl(depth[mcs][d] + e, -64, 63); dirty = 1; }
            if (b & (1u << BTN_PLAY)) { depth[mcs][d] = depth[mcs][d] ? 0 : 32; dirty = 1; }
        }
        if (page != PG_OSC) prev = fm1_in.notes;
        for (k = 0; k < 7u; k++) (void)fm1_enc_take(k);   /* turns a page does not use go nowhere */

        /* the keys' LEDs follow the page */
        for (k = 0; k < 27u; k++) {
            uint32_t on = 0, glow = 0;
            if (page == PG_OSC) {
                for (o = 0; o < NO; o++)
                    if (opatch[o].note == base + k) { if (o == sel) on = 1; else glow = 1; }
            } else if (page == PG_FG || page == PG_MOD) {
                uint32_t pos = (uint32_t)(fg_out[page == PG_FG ? fsel : mcs] * 26) >> 15;
                on = k == pos;
                glow = k < pos;
            }
            fm1_led_key(KEY_LED(k), on);
            if (!on) fm1_led_key_glow(KEY_LED(k), glow);
        }
        fm1_led_key(BTN_OCT_DOWN, oct < 0);
        fm1_led_key(BTN_OCT_UP, oct > 0);
        {
            uint32_t blink = (fm1_ms / 250u) & 1u;   /* a flashing button: "press me here" */
            fm1_led_key(BTN_SEL, shift || (page == PG_INDEX && blink));
            fm1_led_key(BTN_PLAY, (page == PG_FG && play)
                                  || (page == PG_MOD && (depth[mcs][DLIST[mcr]] || blink)));   /* solid: a pin here */
        }
        fm1_led_key(BTN_HOME, page == PG_INDEX);
        fm1_led_key(BTN_EDIT, page == PG_OSC || page == PG_WAVE);
        fm1_led_key(BTN_ENV, page == PG_FG);
        fm1_led_key(BTN_ARP, page == PG_MOD);

        if (fm1_ms - t_cpu > 500u) {            /* the worst render of the last half second, in the header */
            uint32_t us = audio_us_max, pct;
            audio_us_max = 0;
            t_cpu = fm1_ms;
            pct = us * 100u / 5805u;            /* 256 frames at 44.1 kHz = 5805 us */
            if (us && pct != cpu_pct) { cpu_pct = pct; hdr_dirty = 1; }
        }
        if (page == PG_OSC && shift != shift_drawn) { shift_drawn = shift; dirty = 1; }
        if ((page == PG_FG || page == PG_MOD || page == PG_WAVE) && fm1_ms - t_live > 50u) { t_live = fm1_ms; dirty = 1; }

        if (hdr_dirty) { hdr_dirty = 0; draw_header(page, cpu_pct); }
        if (dirty) {
            dirty = 0;
            switch (page) {
            case PG_INDEX: draw_index(icur); break;
            case PG_OSC: draw_osc(sel, shift); break;
            case PG_WAVE: draw_wave(sel, ost[sel].posq); break;
            case PG_FG: draw_fg(fsel); break;
            case PG_MOD: draw_mod(mcs, mcr, mtop); break;
            default: draw_soon(page); break;
            }
        }
    }
}
