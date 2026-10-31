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
 *   SAVE   PAGE P PATCHES      SELECT: a slot; SEL, SEL: load; SAVE, SAVE: save (an empty slot: one SAVE)
 *   ARP    PAGE 7 MOD MATRIX    K1 source, K2 destination, K3 depth (turning an empty cell makes a pin),
 *                               PLAY: pin on / off
 *   the keys' LEDs follow the page: the pitch stack, the FG's slope, the source's level
 *
 * Rules of the road (the chip has no FPU and no libgcc): integers only, no float, no 64-bit division.
 * tests/host.c runs this file on a computer: the screen as a PNG, the sound as a WAV. */
#define APP_NAME "FM1 WAVETABLE"
static void app_idle(void);
#define LCD_IDLE app_idle                       /* while the screen's DMA runs: read MIDI (the clock) */
#define CV_MAX (240u * 218u)                    /* the page body's canvas */
#define STORE_MAGIC 0x54574D46u                 /* "FMWT": our own flash data, never another Dinghy fork's */
#include "platform.c"
#include "wt_tables.h"
#include "luts.h"
#include "tunings.h"

static int32_t cl(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }

/* ================================================================== the engine (audio interrupt) */
#define NO 4u                                   /* oscillators */
#define NFG 6u                                  /* function generators */
#define NSSG 2u                                 /* sample-and-holds (Serge SSG style) */
/* mod sources: FG1..6 (0..+), S1, S2 (+-), the quad LFO's SIN+ SIN- COS+ COS- (+-) */
#define S_SSG(k) (NFG + (k))
#define S_QUAD(k) (NFG + NSSG + (k))
#define NSRC (NFG + NSSG + 4u)
#define SUB 32u                                 /* frames between modulation updates (1378 Hz) */
enum { P_PITCH, P_WAVE, P_MOD, P_THR, P_HICUT, P_LEVEL, P_PAN, NP };
#define NGG 4u                                  /* gate generators */
#define D_FG(k, fall) (NO * NP + 2u * (k) + (fall))   /* an FG's rise / fall time as a destination */
#define D_GG(g, l, duty) (NO * NP + 2u * NFG + 4u * (g) + 2u * (l) + (duty))   /* a gate LFO's rate / duty */
#define D_SSG(k) (NO * NP + 2u * NFG + 4u * NGG + (k))   /* an SSG's slew */
#define D_QRATE (NO * NP + 2u * NFG + 4u * NGG + NSSG)    /* the quad LFO's rate */
#define NDST (NO * NP + 2u * NFG + 4u * NGG + NSSG + 1u)
/* gate sources: per generator A B AND NAND OR NOR XOR XNOR NOT, then the FGs' EOCs.
 * gate destinations: the FGs' gates, then each generator's two logic inputs (pinned: they replace A, B there) */
#define GS_PER 9u
#define GS_Q(o) (NGG * GS_PER + NFG + (o))     /* ... then the quantisers' note-change gates */
#define NGSRC (NGG * GS_PER + NFG + NO)
#define NGDST (NFG + 2u * NGG + NSSG)              /* ... then the SSGs' sample inputs */
#define G_SSG(k) (NFG + 2u * NGG + (k))

typedef struct {                                /* the knobs, set by the main loop */
    volatile int32_t pitch_q8;                  /* note * 256, 1/256 semitone */
    volatile int32_t fine_q8;                   /* (after the quantiser) */
    volatile uint8_t qon, qglide;
    volatile uint8_t pos, mod, thr, hicut, level, pan;
    volatile uint8_t table, scan;               /* scan: 0 smooth (crossfade), 1 stepped (PPG) */
} osc_par_t;
typedef struct {                                /* owned by the interrupt */
    uint32_t ph;
    volatile int32_t posq;                      /* wave position now, Q8 waves (the waveform page reads it) */
    int32_t amp, lp1, lp2, last;
} osc_st_t;
static osc_par_t opar[NO];
static volatile uint8_t g_tun, g_scl, g_root;   /* the global tuning, scale (an index into SCL_*), root (0..11) */
static int32_t q_held[NO], q_cur[NO];           /* the quantiser: the note held, the pitch gliding to it */
static int32_t q_id[NO] = {-1, -1, -1, -1};
static uint8_t qg_cnt[NO];                      /* the note-change gate, ~5 ms */
static volatile int32_t q_step_v[NO], q_pitch_v[NO];   /* for the screen: the step, the pitch now */
#define T_GEN WT_TABLES                         /* the generative table, built here from a seed */
#define NTAB (WT_TABLES + 1u)
static int8_t gen_tab[2][WT_WAVES][WT_SAMPLES]; /* two: one plays while the other is built */
static volatile uint32_t gen_cur;
static const int8_t *table_of(uint32_t t)
{
    return t >= T_GEN ? &gen_tab[gen_cur & 1u][0][0] : &WT[t][0][0];
}
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

enum { LM_FREE, LM_CLK };
typedef struct { volatile uint8_t mode[2], rate[2], duty[2], notsrc; } gg_par_t;
static gg_par_t ggp[NGG];
static uint32_t gg_ph[NGG][2];
static volatile uint16_t gg_bits[NGG];          /* bit i: the generator's source i now */
static volatile uint32_t gmat[NGSRC];           /* the gate matrix: bit j = a pin to gate destination j */
static volatile uint32_t clk_ticks = 0xFFFFFFFFu;   /* MIDI clocks (24 a quarter), counted by the main loop */
static volatile uint8_t fg_manual[NFG];         /* PLAY held on the FG page */
static volatile uint8_t fg_gate_v[NFG];
static const uint16_t CLK_DIV[10] = {6, 8, 12, 16, 24, 48, 96, 192, 384, 768};   /* 1/16 .. 8 bars */

typedef struct { volatile uint8_t input, slew; } ssg_par_t;   /* input 0 = noise, 1.. = mod source input - 1 */
static ssg_par_t ssgp[NSSG];
static volatile int32_t ssg_out[NSSG], ssg_held[NSSG];   /* +-32767 */
static uint8_t ssg_gate_prev[NSSG];
static volatile uint8_t ssg_tap[NSSG];          /* PLAY (and SEL + PLAY) on page 5: a count of taps, by hand */
static uint8_t ssg_tap_seen[NSSG];
static volatile uint8_t ssg_fired[NSSG];        /* set at each sample, cleared by the screen */
static uint32_t noise_s = 0x6D2B79F5u;
static volatile uint8_t q_rate;
static uint32_t q_ph;
static volatile int32_t q_out[4];               /* SIN+ SIN- COS+ COS- */
static volatile int32_t src_v[NSRC];            /* every source now (the screens read it) */

static int32_t sine(uint32_t ph)                /* a full cycle over 2^32 -> -32767..32767 (a refined parabola) */
{
    int32_t x = (int32_t)ph >> 16, ax = x < 0 ? -x : x, y, ay;
    y = (x * (32768 - ax)) >> 13;
    ay = y < 0 ? -y : y;
    y += ((((y * ay) >> 15) - y) * 7373) >> 15;
    return y > 32767 ? 32767 : y < -32767 ? -32767 : y;
}
static volatile uint32_t audio_us, audio_us_max;
static volatile uint32_t mute;                  /* 1: fade the output out (a flash save stalls the sound) */
static volatile int32_t out_gain = 32767;

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
static void fg_step(uint32_t k, uint32_t rise_k, uint32_t fall_k, uint32_t g)
{
    fg_st_t *s = &fgs[k];
    uint32_t mode = fgp[k].mode;
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
    fg_gate_v[k] = (uint8_t)g;
    fg_eoc_v[k] = s->eoc != 0;
}

/* the nearest allowed step of the global tuning and scale to pitch q (1/256 semitone); its id (period * n + step)
 * and its step number come back too */
static int32_t quantise(int32_t q, int32_t *id, int32_t *stepn)
{
    uint32_t t = g_tun < NTUN ? g_tun : 0u, n = TUN_N[t], j, sc = g_scl;
    int32_t per = TUN_PERIOD[t], c, k, rem, best = 0x7FFFFFFF, bv = 0, bj = 0, bk = 0;
    const uint16_t *st = &TUN_STEPS[TUN_OFF[t]];
    uint32_t lo, hi;
    if (sc < TUN_SCL0[t] || sc >= TUN_SCL0[t] + TUN_NSCL[t]) sc = TUN_SCL0[t];
    lo = SCL_LO[sc]; hi = SCL_HI[sc];
    if (!lo && !hi) lo = 1u;
    c = q - (int32_t)g_root * 256;
    k = c >= 0 ? c / per : -((-c + per - 1) / per);
    rem = c - k * per;
    for (j = 0; j < n; j++) {                   /* this period's steps, the next one's first, the last one's last */
        int32_t v, d, kk;
        if (!(j < 32u ? lo >> j & 1u : hi >> (j - 32u) & 1u)) continue;
        for (kk = -1; kk <= 1; kk++) {
            v = (int32_t)st[j] + kk * per;
            d = rem - v;
            if (d < 0) d = -d;
            if (d < best) { best = d; bv = v; bj = (int32_t)j; bk = k + kk; }
        }
    }
    *id = bk * (int32_t)n + bj;
    *stepn = bj;
    return (int32_t)g_root * 256 + k * per + bv;
}

static void render(int32_t *out)                /* SUB frames */
{
    int32_t acc[NDST], src[NSRC];
    uint32_t o, k, d, i;
    int32_t pm[NO], fg_g[NO], fg_t[NO], fg_mk[NO], hc[NO], amp_t[NO], amp_d[NO], pl[NO], pr[NO];
    uint32_t inc[NO];
    const int8_t *tab[NO];
    uint32_t step[NO];

    for (k = 0; k < NFG; k++) src[k] = fg_out[k];   /* every source as it stood a step ago */
    for (k = 0; k < NSSG; k++) src[S_SSG(k)] = ssg_out[k];
    for (k = 0; k < 4u; k++) src[S_QUAD(k)] = q_out[k];
    for (k = 0; k < NSRC; k++) src_v[k] = src[k];
    for (d = 0; d < NDST; d++) acc[d] = 0;      /* the matrix: sum of source x depth, per destination */
    for (k = 0; k < NSRC; k++) {
        int32_t v = src[k];
        if (!v) continue;
        for (d = 0; d < NDST; d++) {
            int32_t dep = depth[k][d];
            if (dep) acc[d] += v * dep;
        }
    }
    for (d = 0; d < NDST; d++) acc[d] >>= 6;    /* -> knob units, Q8: a full source at depth 63 is ~ a full knob */

    {   /* the gates: every source as it stood a step ago, so feedback loops never lock up */
        uint32_t gdst = 0, gpin = 0, g, l, i;
        for (i = 0; i < NGSRC; i++) {
            uint32_t m = gmat[i], on;
            if (!m) continue;
            gpin |= m;
            on = i < NGG * GS_PER ? (uint32_t)gg_bits[i / GS_PER] >> (i % GS_PER) & 1u
               : i < GS_Q(0) ? fg_eoc_v[i - NGG * GS_PER] : qg_cnt[i - GS_Q(0)] != 0u;
            if (on) gdst |= m;
        }
        for (g = 0; g < NGG; g++) {
            uint32_t lv[2], a, bb, bits;
            for (l = 0; l < 2u; l++) {
                uint32_t rk = (uint32_t)cl(ggp[g].rate[l] + (acc[D_GG(g, l, 0)] >> 8), 0, 127);
                uint32_t dk = (uint32_t)cl(ggp[g].duty[l] + (acc[D_GG(g, l, 1)] >> 8), 0, 127);
                uint32_t thr = (((dk * 250u) >> 7) + 3u) << 24, ph;   /* duty ~1 .. 98 % */
                if (ggp[g].mode[l] == LM_CLK) {
                    uint32_t div = CLK_DIV[(rk * 10u) >> 7], t = clk_ticks % div;
                    ph = ((t * 65536u) / div) << 16;
                } else
                    ph = gg_ph[g][l] += LUT_GGINC[rk];
                lv[l] = ph < thr;
            }
            a = gpin >> (NFG + 2u * g) & 1u ? gdst >> (NFG + 2u * g) & 1u : lv[0];
            bb = gpin >> (NFG + 2u * g + 1u) & 1u ? gdst >> (NFG + 2u * g + 1u) & 1u : lv[1];
            bits = lv[0] | lv[1] << 1 | (a & bb) << 2 | (~(a & bb) & 1u) << 3 | (a | bb) << 4 | (~(a | bb) & 1u) << 5
                 | (a ^ bb) << 6 | (~(a ^ bb) & 1u) << 7;
            bits |= (~(bits >> (ggp[g].notsrc & 7u)) & 1u) << 8;   /* NOT: one of the eight, inverted */
            gg_bits[g] = (uint16_t)bits;
        }
        for (k = 0; k < NFG; k++)
            fg_step(k, (uint32_t)cl(fgp[k].rise + (acc[D_FG(k, 0)] >> 8), 0, 127),
                       (uint32_t)cl(fgp[k].fall + (acc[D_FG(k, 1)] >> 8), 0, 127),
                       fg_manual[k] | (gdst >> k & 1u));
        for (k = 0; k < NSSG; k++) {            /* the SSGs: sample on a gate's start, then slew to it */
            uint32_t gt = gdst >> G_SSG(k) & 1u, sk, tap = ssg_tap[k] != ssg_tap_seen[k];
            int32_t o = ssg_out[k], t;
            ssg_tap_seen[k] = ssg_tap[k];       /* (a tap is never missed, however short) */
            if ((gt && !ssg_gate_prev[k]) || tap) {
                uint32_t in = ssgp[k].input;
                if (in == 0u || in > NSRC) {
                    noise_s ^= noise_s << 13; noise_s ^= noise_s >> 17; noise_s ^= noise_s << 5;
                    ssg_held[k] = (int32_t)(noise_s >> 16) - 32768;
                } else
                    ssg_held[k] = src[in - 1u];
                ssg_fired[k] = 1;
            }
            ssg_gate_prev[k] = (uint8_t)gt;
            t = ssg_held[k];
            sk = (uint32_t)cl(ssgp[k].slew + (acc[D_SSG(k)] >> 8), 0, 127);
            if (sk == 0u) o = t;                /* no slew: a hard step */
            else {
                int32_t step = (int32_t)(LUT_FGINC[sk] >> 8);   /* full scale (64K) in the slew time */
                o = t > o ? (t - o > step ? o + step : t) : (o - t > step ? o - step : t);
            }
            ssg_out[k] = cl(o, -32767, 32767);
        }
        q_ph += LUT_QINC[cl(q_rate + (acc[D_QRATE] >> 8), 0, 127)];   /* the quadrature LFO */
        {
            int32_t sn = sine(q_ph), cs = sine(q_ph + 0x40000000u);
            q_out[0] = sn; q_out[1] = -sn; q_out[2] = cs; q_out[3] = -cs;
        }
    }

    for (o = 0; o < NO; o++) {
        const osc_par_t *p = &opar[o];
        const int32_t *a = &acc[o * NP];
        tab[o] = table_of(p->table);
        step[o] = p->scan;
        int32_t q, target, dpos;
        {
            int32_t pq = p->pitch_q8 + ((a[P_PITCH] * 3) >> 3);           /* full depth: ~4 octaves */
            if (p->qon) {                       /* the quantiser, with a little hysteresis (~8 cents) */
                int32_t id, sn, cand = quantise(pq, &id, &sn), dc = pq - cand, dh = pq - q_held[o];
                if (dc < 0) dc = -dc;
                if (dh < 0) dh = -dh;
                if (q_id[o] < 0 || (id != q_id[o] && dc + 20 < dh)) {
                    if (q_id[o] >= 0) qg_cnt[o] = 7;   /* a new note: a gate */
                    q_id[o] = id; q_held[o] = cand; q_step_v[o] = sn;
                }
                if (!p->qglide) q_cur[o] = q_held[o];
                else {
                    int32_t g = (int32_t)(((uint64_t)LUT_FGINC[p->qglide] * 3072u) >> 24) + 1, d = q_held[o] - q_cur[o];
                    q_cur[o] += d > g ? g : d < -g ? -g : d;   /* (an octave in the glide time) */
                }
                pq = q_cur[o];
            } else {
                q_id[o] = -1;
                q_cur[o] = pq;
            }
            if (qg_cnt[o]) qg_cnt[o]--;
            q_pitch_v[o] = pq;
            inc[o] = pitch_inc(pq + p->fine_q8);
        }
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
            if (step[o]) { w = ((uint32_t)s->posq + 128u) >> 8; frac = 0; }   /* stepped: one wave, then the next */
            else { w = (uint32_t)s->posq >> 8; frac = (uint32_t)s->posq & 255u; }
            w0 = tab[o] + w * WT_SAMPLES;
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
    if (mute || out_gain < 32767) {             /* a short fade, out for a save and back in */
        int32_t g = out_gain;
        for (b = 0; b < 2u * n; b++) {
            if (!(b & 1u)) g = mute ? (g > 128 ? g - 128 : 0) : (g < 32767 - 128 ? g + 128 : 32767);
            out[b] = (out[b] * g) >> 15;
        }
        out_gain = g;
    }
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

static uint8_t otab[NO], oscan[NO];             /* (kept apart from patch_t: older patches keep their layout) */
typedef struct { uint8_t on, glide; } oq_t;
static oq_t oq[NO];                             /* each oscillator's quantiser */
typedef struct { uint8_t tun, scl, root, pad; } tunpatch_t;
static tunpatch_t tunpatch = {0, 1, 0, 0};      /* 12-TET, MAJOR, C */
static void push_tun(void)
{
    uint32_t t = tunpatch.tun < NTUN ? tunpatch.tun : 0u;
    if (tunpatch.scl < TUN_SCL0[t] || tunpatch.scl >= TUN_SCL0[t] + TUN_NSCL[t]) tunpatch.scl = TUN_SCL0[t];
    g_tun = (uint8_t)t;
    g_scl = tunpatch.scl;
    g_root = (uint8_t)(tunpatch.root % 12u);
}
static uint32_t gen_seed = 1u, gen_built;

/* the generative table: a sine, partials from the seed creeping in (the higher, the later), bite in the last
 * third. Built in the spare buffer, then swapped in. */
static void gen_build(uint32_t seed)
{
    enum { NPART = 12 };
    uint32_t r = seed ? seed : 1u, i, w, n;
    uint32_t hk[NPART], ph[NPART];
    int32_t amp[NPART], start[NPART];
    int8_t (*dst)[WT_SAMPLES] = gen_tab[(gen_cur & 1u) ^ 1u];
#define GRND() (r ^= r << 13, r ^= r >> 17, r ^= r << 5, r)
    hk[0] = 1; amp[0] = 32767; start[0] = -8; ph[0] = 0;
    for (i = 1; i < NPART; i++) {
        uint32_t a = GRND() % 31u, b = GRND() % 31u;
        hk[i] = 2u + (a * b) / 31u;             /* 2..32, the low ones likelier */
        amp[i] = (int32_t)(8000u + GRND() % 20000u) / (int32_t)(1u + hk[i] / 6u);
        start[i] = cl((int32_t)(hk[i] * 44u / 32u) + (int32_t)(GRND() % 17u) - 8, 2, 58);
        ph[i] = GRND();
    }
#undef GRND
    for (w = 0; w < WT_WAVES; w++) {
        int32_t buf[WT_SAMPLES], peak = 1, g[NPART];
        for (i = 0; i < NPART; i++) {
            int32_t x = cl(((int32_t)w - start[i]) * 512, 0, 4096);   /* fades in over 8 waves, Q12 */
            int32_t sm = (x * x >> 12) * (3 * 4096 - 2 * x) >> 12;
            if (hk[i] >= 8u && w > 42u) sm += sm * ((int32_t)w - 42) / 21;   /* the last third: the high ones bite */
            g[i] = (amp[i] * sm) >> 12;
        }
        for (n = 0; n < WT_SAMPLES; n++) {
            int32_t v = 0;
            for (i = 0; i < NPART; i++)
                if (g[i]) v += (sine(hk[i] * n * (1u << 25) + ph[i]) * g[i]) >> 15;
            buf[n] = v;
            if (v > peak) peak = v; else if (-v > peak) peak = -v;
        }
        {
            int32_t sc = (int32_t)((127u << 16) / (uint32_t)peak);
            for (n = 0; n < WT_SAMPLES; n++) dst[w][n] = (int8_t)cl((int32_t)(((int64_t)buf[n] * sc) >> 16), -127, 127);
        }
    }
    gen_cur ^= 1u;                              /* (the engine reads the new one from its next step) */
    gen_built = seed;
}

static void push_osc(uint32_t o)
{
    patch_t *q = &opatch[o];
    opar[o].pitch_q8 = (int32_t)q->note * 256;
    opar[o].fine_q8 = (int32_t)q->fine * 256 / 100;
    opar[o].qon = oq[o].on;
    opar[o].qglide = oq[o].glide;
    opar[o].pos = q->pos;
    opar[o].mod = q->mod;
    opar[o].thr = q->thr;
    opar[o].hicut = q->hicut;
    opar[o].level = q->level;
    opar[o].pan = q->pan;
    opar[o].table = otab[o];
    opar[o].scan = oscan[o];
}
typedef struct { uint8_t mode[2], rate[2], duty[2], notsrc, pad; } ggpatch_t;
static const ggpatch_t GG_DEFAULT[NGG] = {      /* free running; 3's two LFOs a hair apart: a slowly turning XOR */
    {{LM_FREE, LM_FREE}, {60, 72}, {64, 38}, 0, 0}, {{LM_FREE, LM_FREE}, {40, 52}, {12, 76}, 0, 0},
    {{LM_FREE, LM_FREE}, {80, 81}, {64, 64}, 0, 0}, {{LM_FREE, LM_FREE}, {20, 33}, {6, 6}, 0, 0},
};
static ggpatch_t ggpatch[NGG];
static uint32_t gpatch[NGSRC];                  /* the gate matrix as the patch keeps it */

static void push_gg(uint32_t g)
{
    uint32_t l;
    for (l = 0; l < 2u; l++) {
        ggp[g].mode[l] = ggpatch[g].mode[l];
        ggp[g].rate[l] = ggpatch[g].rate[l];
        ggp[g].duty[l] = ggpatch[g].duty[l];
    }
    ggp[g].notsrc = ggpatch[g].notsrc;
}
typedef struct { uint8_t input[NSSG], slew[NSSG], qrate, pad[3]; } shpatch_t;
static const shpatch_t SH_DEFAULT = {{0, 0}, {0, 40}, 70, {0, 0, 0}};   /* noise into both, S2 slewed */
static shpatch_t shpatch;
static void push_sh(void)
{
    uint32_t k;
    for (k = 0; k < NSSG; k++) { ssgp[k].input = shpatch.input[k]; ssgp[k].slew = shpatch.slew[k]; }
    q_rate = shpatch.qrate;
}
static void push_gmat(void)
{
    uint32_t i;
    for (i = 0; i < NGSRC; i++) gmat[i] = gpatch[i];
}
static void push_fg(uint32_t k)
{
    fgp[k].rise = fgpatch[k].rise;
    fgp[k].fall = fgpatch[k].fall;
    fgp[k].curve = fgpatch[k].curve;
    fgp[k].mode = fgpatch[k].mode;
}

/* ---- a random patch: at power-on, and RANDOM on page P. Happy accidents, within reason: mostly consonant
 * intervals with the odd stray one, slow cycling FGs, a handful of pins, small pitch depths, something audible */
static uint32_t rnd_s = 0x9E3779B9u;
static uint32_t rnd(uint32_t n)                 /* 0..n-1 (xorshift32) */
{
    rnd_s ^= rnd_s << 13;
    rnd_s ^= rnd_s >> 17;
    rnd_s ^= rnd_s << 5;
    return n ? rnd_s % n : 0u;
}
static int32_t rnd_in(int32_t lo, int32_t hi) { return lo + (int32_t)rnd((uint32_t)(hi - lo + 1)); }
static void randomize(void);

/* ---- patches in flash: 16 slots, two to a 4 KB blob, each a fixed 2040 bytes with room to grow
 * (later versions add their parts in the spare room; a patch saved now still loads) */
#define NPATCH 16u
#define PATCH_TAG 0x31505457u                   /* "WTP1" */
typedef struct {
    uint32_t tag;
    uint16_t ver, used;
    patch_t osc[NO];
    fgpatch_t fg[8];                            /* (room for 8 FGs) */
    int8_t dep[16][64];                         /* (room for 16 sources x 64 destinations) */
    ggpatch_t gg[NGG];                          /* version 2 on */
    uint32_t gm[48];                            /* (room for 48 gate sources x 32 destinations) */
    shpatch_t sh;                               /* version 3 on */
    uint8_t otab[NO], oscan[NO];                /* version 4 on */
    uint32_t gseed;
    oq_t oq[NO];                                /* version 5 on */
    tunpatch_t tun;
} pdata_t;
typedef union { pdata_t d; uint8_t raw[2040]; } pslot_t;
typedef char pdata_fits_its_slot[sizeof(pdata_t) <= 2040u ? 1 : -1];   /* (the build fails if it ever grows past) */
static pslot_t pbank[2];                        /* one blob's worth */
static uint8_t pused[NPATCH + 2];               /* slot 0 = the built-in default, NPATCH + 1 = RANDOM */
#define P_RANDOM (NPATCH + 1u)
static patch_t def_osc[NO];
static fgpatch_t def_fg[NFG];
static int8_t def_dep[NSRC][NDST];
static uint32_t def_gm[NGSRC];

static int bank_read(uint32_t blob)
{
    uint8_t *p = (uint8_t *)pbank;
    uint32_t i;
    if (blob_load(blob, pbank, sizeof pbank) == 0) return 0;
    for (i = 0; i < sizeof pbank; i++) p[i] = 0;
    return -1;
}
static void patches_scan(void)
{
    uint32_t b, j;
    pused[0] = pused[P_RANDOM] = 1;
    for (b = 0; b < NPATCH / 2u; b++) {
        bank_read(b);
        for (j = 0; j < 2u; j++)
            pused[1u + 2u * b + j] = pbank[j].d.tag == PATCH_TAG && pbank[j].d.used;
    }
}
static int patch_save(uint32_t slot)            /* 1..16 */
{
    uint32_t b = (slot - 1u) / 2u, j = (slot - 1u) & 1u, o, k, d, t0;
    pdata_t *q;
    int r;
    bank_read(b);
    q = &pbank[j].d;
    {
        uint8_t *raw = pbank[j].raw;
        for (k = 0; k < sizeof pbank[j].raw; k++) raw[k] = 0;
    }
    q->tag = PATCH_TAG;
    q->ver = 5;
    q->used = 1;
    for (o = 0; o < NO; o++) q->osc[o] = opatch[o];
    for (k = 0; k < NFG; k++) q->fg[k] = fgpatch[k];
    for (k = 0; k < NSRC; k++)
        for (d = 0; d < NDST; d++) q->dep[k][d] = depth[k][d];
    for (k = 0; k < NGG; k++) q->gg[k] = ggpatch[k];
    for (k = 0; k < NGSRC; k++) q->gm[k] = gpatch[k];
    q->sh = shpatch;
    for (o = 0; o < NO; o++) { q->otab[o] = otab[o]; q->oscan[o] = oscan[o]; }
    q->gseed = gen_seed;
    for (o = 0; o < NO; o++) q->oq[o] = oq[o];
    q->tun = tunpatch;
    mute = 1;                                   /* fade out, save, fade in: the erase stops the sound ~50 ms */
    t0 = fm1_ms;
    while (out_gain > 0 && fm1_ms - t0 < 100u) platform_poll();
    r = blob_save(b, pbank, sizeof pbank);
    mute = 0;
    if (r == 0) pused[slot] = 1;
    return r;
}
static int patch_load(uint32_t slot)            /* 0 = the default */
{
    uint32_t o, k, d;
    const pdata_t *q;
    if (slot == P_RANDOM) { randomize(); return 0; }
    if (slot == 0u) {
        for (o = 0; o < NO; o++) opatch[o] = def_osc[o];
        for (k = 0; k < NFG; k++) fgpatch[k] = def_fg[k];
        for (k = 0; k < NSRC; k++)
            for (d = 0; d < NDST; d++) depth[k][d] = def_dep[k][d];
        for (k = 0; k < NGG; k++) ggpatch[k] = GG_DEFAULT[k];
        for (k = 0; k < NGSRC; k++) gpatch[k] = def_gm[k];
        shpatch = SH_DEFAULT;
        for (o = 0; o < NO; o++) { otab[o] = oscan[o] = 0; oq[o].on = oq[o].glide = 0; }
        tunpatch.tun = 0; tunpatch.scl = 1; tunpatch.root = 0;
    } else {
        if (bank_read((slot - 1u) / 2u)) return -1;
        q = &pbank[(slot - 1u) & 1u].d;
        if (q->tag != PATCH_TAG || !q->used) return -1;
        for (o = 0; o < NO; o++) opatch[o] = q->osc[o];
        for (k = 0; k < NFG; k++) fgpatch[k] = q->fg[k];
        for (k = 0; k < NSRC; k++)
            for (d = 0; d < NDST; d++) depth[k][d] = q->dep[k][d];
        for (k = 0; k < NGG; k++) ggpatch[k] = q->ver >= 2u ? q->gg[k] : GG_DEFAULT[k];   /* (version 1: none) */
        for (k = 0; k < NGSRC; k++) gpatch[k] = q->ver >= 2u ? q->gm[k] & ((1u << NGDST) - 1u) : 0u;
        shpatch = q->ver >= 3u ? q->sh : SH_DEFAULT;   /* (older: the default SSGs and quad) */
        for (o = 0; o < NO; o++) {              /* (older: table 1, smooth) */
            otab[o] = q->ver >= 4u ? q->otab[o] : 0u;
            oscan[o] = q->ver >= 4u ? q->oscan[o] : 0u;
        }
        if (q->ver >= 4u && q->gseed) gen_seed = q->gseed;
        for (o = 0; o < NO; o++) {              /* (older: no quantising, 12-TET major on C) */
            oq[o].on = q->ver >= 5u ? q->oq[o].on : 0u;
            oq[o].glide = q->ver >= 5u ? q->oq[o].glide : 0u;
        }
        if (q->ver >= 5u) tunpatch = q->tun; else { tunpatch.tun = 0; tunpatch.scl = 1; tunpatch.root = 0; }
    }
    if (gen_seed != gen_built) gen_build(gen_seed);
    for (o = 0; o < NO; o++) {                  /* (a damaged value never reaches the engine) */
        patch_t *p = &opatch[o];
        if (otab[o] >= NTAB) otab[o] = 0;
        oscan[o] &= 1u;
        oq[o].on &= 1u; oq[o].glide &= 127u;
        p->note = (uint8_t)cl(p->note, 12, 108); p->fine = (int8_t)cl(p->fine, -50, 50);
        p->pos &= 127u; p->mod &= 127u; p->thr &= 127u; p->hicut &= 127u; p->level &= 127u; p->pan &= 127u;
        push_osc(o);
    }
    for (k = 0; k < NFG; k++) {
        fgpatch_t *f = &fgpatch[k];
        f->rise &= 127u; f->fall &= 127u; f->curve = (uint8_t)cl(f->curve, 0, 2); f->mode = (uint8_t)cl(f->mode, 0, 2);
        push_fg(k);
    }
    for (k = 0; k < NSRC; k++)
        for (d = 0; d < NDST; d++) depth[k][d] = (int8_t)cl(depth[k][d], -64, 63);
    for (k = 0; k < NGG; k++) {
        ggpatch_t *g = &ggpatch[k];
        for (d = 0; d < 2u; d++) { g->mode[d] = (uint8_t)(g->mode[d] ? LM_CLK : LM_FREE); g->rate[d] &= 127u; g->duty[d] &= 127u; }
        g->notsrc &= 7u;
        push_gg(k);
    }
    push_tun();
    for (k = 0; k < NSSG; k++) { shpatch.input[k] = (uint8_t)cl(shpatch.input[k], 0, NSRC); shpatch.slew[k] &= 127u; }
    shpatch.qrate &= 127u;
    push_sh();
    push_gmat();
    return 0;
}

/* the matrix's destinations as the page lists them (no threshold on 1 and 2: they have no folder) */
static const uint8_t DLIST[] = {
    0 * NP + P_PITCH, 0 * NP + P_WAVE, 0 * NP + P_MOD, 0 * NP + P_HICUT, 0 * NP + P_LEVEL, 0 * NP + P_PAN,
    1 * NP + P_PITCH, 1 * NP + P_WAVE, 1 * NP + P_MOD, 1 * NP + P_HICUT, 1 * NP + P_LEVEL, 1 * NP + P_PAN,
    2 * NP + P_PITCH, 2 * NP + P_WAVE, 2 * NP + P_MOD, 2 * NP + P_THR, 2 * NP + P_HICUT, 2 * NP + P_LEVEL, 2 * NP + P_PAN,
    3 * NP + P_PITCH, 3 * NP + P_WAVE, 3 * NP + P_MOD, 3 * NP + P_THR, 3 * NP + P_HICUT, 3 * NP + P_LEVEL, 3 * NP + P_PAN,
    D_FG(0, 0), D_FG(0, 1), D_FG(1, 0), D_FG(1, 1), D_FG(2, 0), D_FG(2, 1),
    D_FG(3, 0), D_FG(3, 1), D_FG(4, 0), D_FG(4, 1), D_FG(5, 0), D_FG(5, 1),
    D_GG(0, 0, 0), D_GG(0, 0, 1), D_GG(0, 1, 0), D_GG(0, 1, 1), D_GG(1, 0, 0), D_GG(1, 0, 1), D_GG(1, 1, 0), D_GG(1, 1, 1),
    D_GG(2, 0, 0), D_GG(2, 0, 1), D_GG(2, 1, 0), D_GG(2, 1, 1), D_GG(3, 0, 0), D_GG(3, 0, 1), D_GG(3, 1, 0), D_GG(3, 1, 1),
    D_SSG(0), D_SSG(1), D_QRATE,
};
#define NDLIST (sizeof DLIST / sizeof DLIST[0])

static void randomize(void)
{
    static const uint8_t NICE[] = {0, 5, 7, 12, 12, 17, 19, 24};   /* unisons, fourths, fifths, octaves */
    uint32_t o, k, d, n, loud = 0;
    int32_t root = rnd_in(33, 48);              /* A1 .. C3 */
    for (o = 0; o < NO; o++) {
        patch_t *p = &opatch[o];
        int32_t iv = o == 0u ? 0 : rnd(4) ? NICE[rnd(sizeof NICE)] : rnd_in(1, 24);   /* now and then, a stray */
        p->note = (uint8_t)cl(root + iv, 12, 108);
        p->fine = (int8_t)rnd_in(-12, 12);
        p->pos = (uint8_t)rnd(128);
        n = rnd(128);
        p->mod = (uint8_t)((n * n) >> 7);       /* PM / fold: mostly gentle */
        p->thr = (uint8_t)rnd(128);
        p->hicut = (uint8_t)rnd_in(50, 127);
        p->level = (uint8_t)rnd_in(55, 110);
        p->pan = (uint8_t)rnd(128);
        if (p->level >= 85u) loud = 1;
    }
    if (!loud) opatch[rnd(NO)].level = 100;
    for (o = 0; o < NO; o++) {
        otab[o] = (uint8_t)rnd(NTAB);           /* any table, the generative one too */
        oscan[o] = rnd(4) == 0u;                /* now and then: stepped */
    }
    gen_seed = rnd(0xFFFFFFFFu) | 1u;
    gen_build(gen_seed);
    tunpatch.tun = (uint8_t)(rnd(2) ? rnd(4) : rnd(NTUN));   /* half the time a 12-step tuning, else anything */
    tunpatch.scl = (uint8_t)(TUN_SCL0[tunpatch.tun] + rnd(TUN_NSCL[tunpatch.tun]));
    tunpatch.root = (uint8_t)rnd(12);
    push_tun();
    for (o = 0; o < NO; o++) {
        oq[o].on = rnd(2) == 0u;
        oq[o].glide = rnd(3) ? 0u : (uint8_t)rnd_in(20, 80);
    }
    for (k = 0; k < NFG; k++) {
        fgpatch_t *f = &fgpatch[k];
        f->rise = (uint8_t)rnd_in(30, 115);
        f->fall = (uint8_t)rnd_in(30, 115);
        f->curve = (uint8_t)rnd(3);
        f->mode = rnd(5) < 3u ? M_CYCLE : (uint8_t)(1u + rnd(2));   /* some wait for a gate (below) */
    }
    for (k = 0; k < NGG; k++) {
        ggpatch_t *g = &ggpatch[k];
        for (d = 0; d < 2u; d++) {
            g->mode[d] = LM_FREE;               /* (no clock may be coming) */
            g->rate[d] = (uint8_t)rnd_in(25, 100);
            g->duty[d] = (uint8_t)rnd_in(6, 120);
        }
        g->notsrc = (uint8_t)rnd(8);
    }
    for (k = 0; k < NGSRC; k++) gpatch[k] = 0;
    for (k = 0; k < NFG; k++)                   /* every FG that waits gets a gate */
        if (fgpatch[k].mode != M_CYCLE) {
            uint32_t src = rnd(5) ? rnd(NGG * GS_PER) : NGG * GS_PER + rnd(NFG);   /* now and then: an EOC */
            gpatch[src] |= 1u << k;
        }
    if (!rnd(6)) gpatch[NGG * GS_PER + rnd(NFG)] |= 1u << (NFG + rnd(2u * NGG));   /* rarely: an EOC into the logic */
    for (k = 0; k < NSSG; k++) {
        shpatch.input[k] = rnd(3) ? 0u : (uint8_t)(1u + rnd(NSRC));   /* mostly noise */
        shpatch.slew[k] = rnd(2) ? 0u : (uint8_t)rnd_in(20, 90);
        if (rnd(2)) gpatch[rnd(NGSRC)] |= 1u << G_SSG(k);   /* half the time it gets a clock */
    }
    shpatch.qrate = (uint8_t)rnd_in(20, 90);
    for (k = 0; k < NSRC; k++)
        for (d = 0; d < NDST; d++) depth[k][d] = 0;
    for (n = (uint32_t)rnd_in(3, 8); n; n--) {  /* 3..8 pins */
        uint32_t src = rnd(NSRC), dst = DLIST[rnd(NDLIST)];
        int32_t a = dst < NO * NP && dst % NP == P_PITCH ? rnd_in(2, 8) : rnd_in(10, 45);
        depth[src][dst] = (int8_t)(rnd(10) < 7u ? a : -a);
    }
    for (o = 0; o < NO; o++) push_osc(o);
    for (k = 0; k < NFG; k++) push_fg(k);
    for (k = 0; k < NGG; k++) push_gg(k);
    push_sh();
    push_gmat();
}

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
    } else if (d < D_GG(0, 0, 0)) {
        s_cat(s, "FG");
        s_int(s, (int32_t)(d - NO * NP) / 2 + 1, 0);
        s_cat(s, (d - NO * NP) & 1u ? " FALL" : " RISE");
    } else if (d >= D_SSG(0)) {
        if (d == D_QRATE) s_cat(s, "QUAD RATE");
        else { s_cat(s, "S"); s_int(s, (int32_t)(d - D_SSG(0)) + 1, 0); s_cat(s, " SLEW"); }
    } else {
        uint32_t g = d - D_GG(0, 0, 0);
        s_cat(s, "G");
        s_int(s, (int32_t)(g / 4u) + 1, 0);
        s_cat(s, g & 2u ? "B" : "A");
        s_cat(s, g & 1u ? " DUTY" : " RATE");
    }
}

enum { PG_INDEX, PG_OSC, PG_WAVE, PG_FG, PG_SH, PG_GATE, PG_MOD, PG_GMAT, PG_GLOBAL, PG_PATCH, NPG };
static const char *const PG_NUM[NPG] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "P"};
static const char *const PG_NAME[NPG] = {"INDEX", "OSCILLATORS", "WAVEFORM", "FUNCTION GEN", "S&H / QUAD",
                                         "GATE GEN", "MOD MATRIX", "GATE MATRIX", "GLOBAL", "PATCHES"};
static const uint8_t PG_READY[NPG] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

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
    cv_text(4, 60, &FONT_S, PG_NAME[pg < NPG ? pg : 0], PH_MID);
    cv_text(4, 84, &FONT_S, "NOT IN THIS BUILD YET", PH_LO);
    cv_text(4, 196, &FONT_S, "HOME: INDEX", PH_LO);
    cv_blit(0, BODY_Y);
}

static const char *const TAB_SHORT[NTAB] = {"S>O", "T>S", "ORG1", "ORG2", "BEL1", "BEL2", "VOW1", "VOW2",
                                             "FORM", "PWM", "SYNC", "GRIT", "GEN"};
static const char *tab_name(uint32_t t) { return t >= T_GEN ? "GENERATIVE" : WT_NAME[t]; }

static void draw_osc(uint32_t sel, int shift)
{
    static const char *const ROW[9] = {"NOTE", "FINE", "TABLE", "WAVE", "PM/FLD", "THRESH", "HI CUT", "LEVEL", "PAN"};
    uint32_t o, r;
    cv_begin(240, BODY_H, PH_BG);
    for (o = 0; o < NO; o++) {
        int32_t x = 64 + (int32_t)o * 44;
        if (o == sel) cv_rect(x - 2, 4, 42, 186, PH_SEL);
        cv_text(x + 14, 6, &FONT_S, ONAME[o], o == sel ? PH_HI : PH_LO);
    }
    for (r = 0; r < 9u; r++) {
        int32_t y = 28 + (int32_t)r * 18;
        int hot = shift ? (r == 1u || r == 2u || r == 5u) : (r != 1u && r != 2u && r != 5u);   /* what the knobs reach now */
        cv_text(2, y, &FONT_S, ROW[r], hot ? PH_MID : PH_LO);
        for (o = 0; o < NO; o++) {
            patch_t *q = &opatch[o];
            char s[12] = "";
            switch (r) {
            case 0: s_note(s, q->note); break;
            case 1: s_int(s, q->fine, 1); break;
            case 2: s_cat(s, TAB_SHORT[otab[o] < NTAB ? otab[o] : 0]); break;
            case 3: s_int(s, q->pos, 0); break;
            case 4: s_int(s, q->mod, 0); break;
            case 5: if (o < 2u) s_cat(s, "-"); else s_int(s, q->thr, 0); break;
            case 6: if (q->hicut >= 127u) s_cat(s, "OPN"); else s_int(s, q->hicut, 0); break;
            case 7: s_int(s, q->level, 0); break;
            default: s_int(s, (int32_t)q->pan - 64, 1); break;
            }
            cv_text(64 + (int32_t)o * 44, y, &FONT_S, s, o == sel ? PH_HI : PH_MID);
        }
    }
    cv_text(4, 198, &FONT_S, shift ? "SEL: FINE, TABLE, THRESH" : "KEYS: SET NOTE  SEL: MORE", PH_LO);
    cv_blit(0, BODY_Y);
}

static const int8_t *wave_tab;
static int32_t wave_at(uint32_t w, uint32_t i) { return wave_tab[w * WT_SAMPLES + i]; }

static void draw_wave(uint32_t sel, int32_t posq)
{
    int32_t w, i, f = posq & 255, wc = posq >> 8;
    char s[40] = "OSC ";
    wave_tab = table_of(otab[sel]);
    if (oscan[sel]) { wc = (posq + 128) >> 8; f = 0; }   /* stepped: the wave it is on */
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
    s_cat(s, "  ");
    s_cat(s, tab_name(otab[sel]));
    cv_text(4, 162, &FONT_S, s, PH_HI);
    s[0] = 0;
    s_cat(s, "WAVE ");
    s_int(s, (posq * 127) / (63 * 256), 0);
    s_cat(s, oscan[sel] ? "   STEPPED" : "   SMOOTH");
    cv_text(4, 180, &FONT_S, s, PH_MID);
    cv_text(4, 198, &FONT_S, otab[sel] >= T_GEN ? "K1 WAV K2 TAB K3 SCN PLAY:NEW" : "K1 WAVE K2 TABLE K3 SCAN", PH_LO);
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
        if (fg_gate_v[k]) cv_rect(172, y + 4, 4, 8, PH_HI);   /* the FG's gate is high */
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
#define MCOLS 7u
static const char *const SRCN[NSRC] = {"F1", "F2", "F3", "F4", "F5", "F6", "S1", "S2", "Q+", "Q-", "C+", "C-"};
static const char *const SRCL[NSRC] = {"FG1", "FG2", "FG3", "FG4", "FG5", "FG6", "SSG1", "SSG2",
                                       "SIN+", "SIN-", "COS+", "COS-"};
static uint32_t src_disp(uint32_t k)            /* a source as 0..32767 (the bipolar ones lifted), for lights */
{
    int32_t v = src_v[k];
    return (uint32_t)(k < NFG ? v : (v + 32767) >> 1);
}
static void draw_mod(uint32_t cs, uint32_t cr, uint32_t top, uint32_t left)
{
    uint32_t c, r;
    char s[40] = "";
    cv_begin(240, BODY_H, PH_BG);
    for (c = left; c < left + MCOLS && c < NSRC; c++)
        cv_text(96 + (int32_t)(c - left) * 20, 2, &FONT_S, SRCN[c], c == cs ? PH_HI : PH_LO);
    cv_rect(0, 19, 240, 1, PH_LO);
    for (r = 0; r < MROWS && top + r < NDLIST; r++) {
        uint32_t d = DLIST[top + r];
        int32_t y = 24 + (int32_t)r * 17;
        char n[16];
        dst_name(n, d);
        if (top + r == cr) cv_rect(0, y - 1, 92, 17, PH_SEL);
        cv_text(2, y, &FONT_S, n, top + r == cr ? PH_HI : PH_MID);
        for (c = left; c < left + MCOLS && c < NSRC; c++) {
            int32_t x = 100 + (int32_t)(c - left) * 20, dep = depth[c][d], v = src_v[c];
            if (dep) {
                uint16_t col = (v < 0 ? -v : v) > 16384 ? PH_HI : PH_MID;   /* the pin glows with its source */
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
    s_cat(s, SRCL[cs]);
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

/* ---- the gate pages */
#define HIST 196u                               /* the logic analyser: 196 samples, one every 10 ms */
static uint16_t ghist[NGG][HIST];
static uint32_t ghead;
static const char *const GSN[GS_PER] = {"A", "B", "AND", "NAND", "OR", "NOR", "XOR", "XNOR", "NOT"};
static const char *const CLKN[10] = {"1/16", "1/8T", "1/8", "1/4T", "1/4", "1/2", "1 BAR", "2 BAR", "4 BAR", "8 BAR"};

static uint32_t gsrc_on(uint32_t i)
{
    return i < NGG * GS_PER ? (uint32_t)gg_bits[i / GS_PER] >> (i % GS_PER) & 1u
         : i < GS_Q(0) ? fg_eoc_v[i - NGG * GS_PER] : qg_cnt[i - GS_Q(0)] != 0u;
}
static void gsrc_name(char *s, uint32_t i)
{
    s[0] = 0;
    if (i < NGG * GS_PER) { s_cat(s, "G"); s_int(s, (int32_t)(i / GS_PER) + 1, 0); s_cat(s, " "); s_cat(s, GSN[i % GS_PER]); }
    else if (i < GS_Q(0)) { s_cat(s, "FG"); s_int(s, (int32_t)(i - NGG * GS_PER) + 1, 0); s_cat(s, " EOC"); }
    else { s_cat(s, "Q"); s_cat(s, ONAME[i - GS_Q(0)]); s_cat(s, " NOTE"); }
}
static void gdst_name(char *s, uint32_t j, int full)
{
    s[0] = 0;
    if (j < NFG) { s_cat(s, full ? "FG" : "F"); s_int(s, (int32_t)j + 1, 0); if (full) s_cat(s, " GATE"); }
    else if (j >= G_SSG(0)) { s_cat(s, "S"); s_int(s, (int32_t)(j - G_SSG(0)) + 1, 0); if (full) s_cat(s, " SAMPLE"); }
    else {
        uint32_t g = (j - NFG) / 2u, l = (j - NFG) & 1u;
        if (full) { s_cat(s, "G"); s_int(s, (int32_t)g + 1, 0); s_cat(s, l ? " IN B" : " IN A"); }
        else { s_int(s, (int32_t)g + 1, 0); s_cat(s, l ? "b" : "a"); }
    }
}

static void draw_gg(uint32_t g)
{
    const ggpatch_t *q = &ggpatch[g];
    uint32_t l, r, i;
    char s[40] = "GEN ";
    cv_begin(240, BODY_H, PH_BG);
    s_int(s, (int32_t)g + 1, 0);
    cv_text(4, 2, &FONT_S, s, PH_HI);
    s[0] = 0;
    s_cat(s, "NOT: ");
    s_cat(s, GSN[q->notsrc & 7u]);
    cv_text(236 - text_w(&FONT_S, s), 2, &FONT_S, s, PH_MID);
    for (l = 0; l < 2u; l++) {
        uint32_t pct = (((q->duty[l] * 250u) >> 7) + 3u) * 100u >> 8;
        s[0] = 0;
        s_cat(s, l ? "B " : "A ");
        if (q->mode[l] == LM_CLK) { s_cat(s, "CLK  "); s_cat(s, CLKN[(q->rate[l] * 10u) >> 7]); }
        else {
            uint32_t hz = (uint32_t)(((uint64_t)LUT_GGINC[q->rate[l]] * 137813u) >> 32);   /* 1/100 Hz */
            s_cat(s, "FREE ");
            s_int(s, (int32_t)(hz / 100u), 0);
            s_cat(s, hz % 100u < 10u ? ".0" : ".");
            s_int(s, (int32_t)(hz % 100u), 0);
            s_cat(s, "HZ");
        }
        cv_text(4, 22 + (int32_t)l * 18, &FONT_S, s, PH_MID);
        s[0] = 0;
        s_cat(s, "DUTY ");
        s_int(s, (int32_t)pct, 0);
        s_cat(s, "%");
        cv_text(236 - text_w(&FONT_S, s), 22 + (int32_t)l * 18, &FONT_S, s, PH_MID);
    }
    cv_rect(0, 59, 240, 1, PH_LO);
    for (r = 0; r < GS_PER; r++) {              /* the logic analyser: oldest on the left */
        int32_t yb = 62 + (int32_t)r * 15, plv = -1;
        uint32_t on = (uint32_t)gg_bits[g] >> r & 1u;
        cv_text(2, yb - 1, &FONT_S, GSN[r], on ? PH_HI : PH_LO);
        for (i = 0; i < HIST; i++) {
            int32_t lv = ghist[g][(ghead + i) % HIST] >> r & 1u, x = 40 + (int32_t)i;
            cv_pset(x, lv ? yb + 2 : yb + 11, i + 1u == HIST && on ? PH_HI : PH_MID);
            if (plv >= 0 && lv != plv) cv_rect(x, yb + 2, 1, 10, PH_MID);
            plv = lv;
        }
    }
    cv_text(2, 200, &FONT_S, "K1/2 A K3/4 B SEL:MODE ALG:NOT", PH_LO);
    cv_blit(0, BODY_Y);
}

#define GCOLS 8u
static void draw_gmat(uint32_t cc, uint32_t cr, uint32_t top, uint32_t left)
{
    uint32_t c, r;
    char s[40], n[20];
    cv_begin(240, BODY_H, PH_BG);
    for (c = left; c < left + GCOLS && c < NGDST; c++) {
        gdst_name(n, c, 0);
        cv_text(76 + (int32_t)(c - left) * 20, 2, &FONT_S, n, c == cc ? PH_HI : PH_LO);
    }
    cv_rect(0, 19, 240, 1, PH_LO);
    for (r = 0; r < MROWS && top + r < NGSRC; r++) {
        uint32_t i = top + r, on = gsrc_on(i);
        int32_t y = 24 + (int32_t)r * 17;
        gsrc_name(n, i);
        if (i == cr) cv_rect(0, y - 1, 72, 17, PH_SEL);
        cv_text(2, y, &FONT_S, n, i == cr ? PH_HI : PH_MID);
        for (c = left; c < left + GCOLS && c < NGDST; c++) {
            int32_t x = 80 + (int32_t)(c - left) * 20;
            if (gpatch[i] >> c & 1u) cv_rect(x, y + 4, 9, 9, on ? PH_HI : PH_LO);   /* a pin, lit by its gate */
            else cv_rect(x + 4, y + 8, 1, 1, PH_LO);
            if (c == cc && i == cr) {
                cv_rect(x - 4, y, 17, 1, PH_HI); cv_rect(x - 4, y + 16, 17, 1, PH_HI);
                cv_rect(x - 4, y, 1, 17, PH_HI); cv_rect(x + 12, y, 1, 17, PH_HI);
            }
        }
    }
    gsrc_name(s, cr);
    s_cat(s, " > ");
    gdst_name(n, cc, 1);
    s_cat(s, n);
    s_cat(s, gpatch[cr] >> cc & 1u ? "  PIN" : "");
    cv_text(4, 180, &FONT_S, s, PH_HI);
    cv_text(4, 198, &FONT_S, "K1 DST K2 SRC PLAY PIN", PH_LO);
    cv_blit(0, BODY_Y);
}

/* ---- page 5: the SSGs and the quad LFO */
static int16_t shist[NSSG][HIST];               /* the SSGs' outputs, one every 20 ms */
static uint32_t shead;
static void s_secs(char *s, uint32_t cs)        /* 1/100 s -> "12.5S" or "120S" */
{
    if (cs >= 1000u) s_int(s, (int32_t)((cs + 50u) / 100u), 0);
    else { s_int(s, (int32_t)(cs / 100u), 0); s_cat(s, "."); s_int(s, (int32_t)(cs / 10u % 10u), 0); }
    s_cat(s, "S");
}
static void draw_sh(void)
{
    uint32_t k, i;
    char s[32];
    cv_begin(240, BODY_H, PH_BG);
    for (k = 0; k < NSSG; k++) {
        int32_t y0 = (int32_t)k * 62, mid = y0 + 40, py = 0;
        uint32_t in = shpatch.input[k];
        s[0] = 0;
        s_cat(s, "S");
        s_int(s, (int32_t)k + 1, 0);
        s_cat(s, " IN ");
        s_cat(s, in == 0u || in > NSRC ? "NOISE" : SRCL[in - 1u]);
        cv_text(4, y0 + 2, &FONT_S, s, ssg_fired[k] ? PH_HI : PH_MID);
        ssg_fired[k] = 0;
        s[0] = 0;
        s_cat(s, "SLEW ");
        if (shpatch.slew[k]) s_int(s, shpatch.slew[k], 0); else s_cat(s, "OFF");
        cv_text(236 - text_w(&FONT_S, s), y0 + 2, &FONT_S, s, PH_MID);
        cv_rect(40, mid, 196, 1, PH_LO);
        for (i = 0; i < HIST; i++) {            /* the output, oldest on the left */
            int32_t y = mid - ((int32_t)shist[k][(shead + i) % HIST] * 19 >> 15);
            if (i) cv_line(40 + (int32_t)i - 1, py, 40 + (int32_t)i, y, i + 1u == HIST ? PH_HI : PH_MID);
            py = y;
        }
    }
    cv_rect(0, 126, 240, 1, PH_LO);
    s[0] = 0;
    s_cat(s, "QUAD ");
    s_secs(s, LUT_QPER[shpatch.qrate & 127u]);
    cv_text(4, 130, &FONT_S, s, PH_MID);
    {
        int32_t cx = 40, cy = 174, r = 24;
        for (i = 0; i < 64u; i++)
            cv_pset(cx + ((sine((i << 26) + 0x40000000u) * r) >> 15), cy - ((sine(i << 26) * r) >> 15), PH_LO);
        cv_rect(cx + ((q_out[2] * r) >> 15) - 2, cy - ((q_out[0] * r) >> 15) - 2, 5, 5, PH_HI);
    }
    for (k = 0; k < 4u; k++) {                  /* SIN+ SIN- COS+ COS-, bipolar bars */
        int32_t y = 150 + (int32_t)k * 13, v = (q_out[k] * 56) >> 15;
        cv_text(84, y - 4, &FONT_S, SRCN[S_QUAD(k)], PH_LO);
        cv_rect(178, y, 1, 7, PH_LO);
        if (v >= 0) cv_rect(179, y + 1, v, 5, PH_MID); else cv_rect(178 + v, y + 1, -v, 5, PH_MID);
    }
    cv_text(2, 202, &FONT_S, "K1/2 S1 K3/4 S2 ALG:RATE PLAY", PH_LO);
    cv_blit(0, BODY_Y);
}

/* ---- page 9: the tuning, the scale, the quantisers */
static const char *const NOTEN[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
static void draw_global(uint32_t qsel)
{
    uint32_t t = g_tun, n = TUN_N[t], j, o;
    int32_t per = TUN_PERIOD[t], cx = 56, cy = 122, r = 46;
    const uint16_t *st = &TUN_STEPS[TUN_OFF[t]];
    uint32_t lo = SCL_LO[g_scl], hi = SCL_HI[g_scl], turn = 0xFFFFFFFFu / (uint32_t)per;   /* a pitch -> an angle */
    char s[32];
    cv_begin(240, BODY_H, PH_BG);
    cv_text(4, 2, &FONT_S, "TUNING", PH_LO);  cv_text(68, 2, &FONT_S, TUN_NAME[t], PH_HI);
    cv_text(4, 20, &FONT_S, "SCALE", PH_LO);  cv_text(68, 20, &FONT_S, SCL_NAME[g_scl], PH_HI);
    cv_text(4, 38, &FONT_S, "ROOT", PH_LO);   cv_text(68, 38, &FONT_S, NOTEN[g_root % 12u], PH_HI);
    cv_rect(0, 58, 240, 1, PH_LO);
    for (j = 0; j < n; j++) {                   /* the period as a wheel: a tick a step, the scale's long and bright */
        uint32_t a = st[j] * turn;              /* (32-bit only: the chip has no 64-bit division) */
        int32_t sx = sine(a), cy0 = sine(a + 0x40000000u), in = (j < 32u ? lo >> j & 1u : hi >> (j - 32u) & 1u);
        int32_t r0 = in ? r - 10 : r - 4;
        cv_line(cx + (sx * r0 >> 15), cy - (cy0 * r0 >> 15), cx + (sx * r >> 15), cy - (cy0 * r >> 15), in ? PH_HI : PH_LO);
    }
    for (o = 0; o < NO; o++) {                  /* each oscillator's pitch on the wheel, the outer ring 1, inner B */
        int32_t c = q_pitch_v[o] - (int32_t)g_root * 256, rem = c % per, rr = r - 16 - (int32_t)o * 7;
        uint32_t a;
        if (rem < 0) rem += per;
        a = (uint32_t)rem * turn;
        cv_rect(cx + (sine(a) * rr >> 15) - 2, cy - (sine(a + 0x40000000u) * rr >> 15) - 2, 5, 5,
                oq[o].on ? PH_HI : PH_LO);
    }
    for (o = 0; o < NO; o++) {
        int32_t y = 66 + (int32_t)o * 20;
        s[0] = 0;
        s_cat(s, ONAME[o]);
        s_cat(s, oq[o].on ? " ON  " : " OFF ");
        s_cat(s, "G");
        s_int(s, oq[o].glide, 0);
        if (oq[o].on) {
            s_cat(s, oq[o].glide < 10u ? "   " : oq[o].glide < 100u ? "  " : " ");
            if (n == 12u) {                     /* a 12-step tuning: the note; else the step */
                int32_t nt = (q_pitch_v[o] + 128) >> 8;
                s_cat(s, NOTEN[nt % 12]);
                s_int(s, nt / 12 - 1, 0);
            } else { s_cat(s, "#"); s_int(s, q_step_v[o], 0); }
        }
        if (o == qsel) cv_rect(112, y - 1, 128, 18, PH_SEL);
        cv_text(116, y, &FONT_S, s, o == qsel ? PH_HI : oq[o].on ? PH_MID : PH_LO);
    }
    cv_text(4, 182, &FONT_S, "K1 TUNE K2 SCALE K3 ROOT", PH_LO);
    cv_text(4, 200, &FONT_S, "SELECT OSC K4 GLIDE PLAY Q", PH_LO);
    cv_blit(0, BODY_Y);
}

static uint32_t idle_ok;
static void midi_poll(void)                     /* the clock, START: the clocked LFOs follow them */
{
    uint8_t st, d1, d2;
    while (midi_get(&st, &d1, &d2)) {
        if (st == 0xF8u) clk_ticks++;
        else if (st == 0xFAu) clk_ticks = 0xFFFFFFFFu;   /* the next clock is beat 1 */
    }
}
static void app_idle(void) { if (idle_ok) midi_poll(); }

enum { PC_NONE, PC_SAVE, PC_LOAD };
static void draw_patch(uint32_t cur, uint32_t confirm, const char *msg)
{
    uint32_t i;
    cv_begin(240, BODY_H, PH_BG);
    for (i = 0; i <= P_RANDOM; i++) {
        int32_t col = i == 0u ? 0 : i == P_RANDOM ? 1 : (int32_t)((i - 1u) / 8u);
        int32_t row = i == 0u || i == P_RANDOM ? 0 : (int32_t)((i - 1u) % 8u) + 1;
        int32_t x = 4 + col * 120, y = 4 + row * 18;
        char s[24] = "";
        if (i == 0u) s_cat(s, "DEFAULT");
        else if (i == P_RANDOM) s_cat(s, "RANDOM");
        else {
            if (i < 10u) s_cat(s, "0");
            s_int(s, (int32_t)i, 0);
            s_cat(s, pused[i] ? "  SAVED" : "  --");
        }
        if (i == cur) cv_rect(x - 4, y - 1, 116, 18, PH_SEL);
        cv_text(x, y, &FONT_S, s, i == cur ? PH_HI : pused[i] ? PH_MID : PH_LO);
    }
    if (msg && *msg) cv_text(4, 172, &FONT_S, msg, PH_HI);
    else if (confirm == PC_SAVE) cv_text(4, 172, &FONT_S, "SAVE AGAIN: OVERWRITE", PH_HI);
    else if (confirm == PC_LOAD) cv_text(4, 172, &FONT_S, cur == P_RANDOM ? "SEL AGAIN: ROLL THE DICE" : "SEL AGAIN: LOAD IT", PH_HI);
    cv_text(4, 198, &FONT_S, "SEL: LOAD  SAVE: SAVE", PH_LO);
    cv_blit(0, BODY_Y);
}

/* ================================================================== the main loop */
static void app_main(void)
{
    uint32_t o, k, sel = 0, fsel = 0, prev = 0, page = PG_INDEX, icur = PG_OSC, mcs = 0, mcr = 0, mtop = 0, mleft = 0;
    uint32_t dirty = 1, hdr_dirty = 1, cpu_pct = 0, t_cpu = 0, t_live = 0, pcur = 1, pconf = PC_NONE, t_msg = 0;
    const char *pmsg = "";
    uint32_t gsel = 0, gcc = 0, gcr = 0, gtop = 0, gleft = 0, t_hist = 0, qsel = 0;
    int32_t oct = -1, e, shift_drawn = -1;

    for (k = 0; k < NGG; k++) ggpatch[k] = GG_DEFAULT[k];
    shpatch = SH_DEFAULT;

    depth[0][0 * NP + P_WAVE] = 40;             /* a few pins to hear the FGs at once */
    depth[1][2 * NP + P_MOD] = 30;
    depth[2][3 * NP + P_PAN] = 45;
    depth[2][1 * NP + P_MOD] = 20;
    depth[3][D_FG(0, 0)] = 20;                  /* FG4 bends FG1's rise: feedback */
    for (o = 0; o < NO; o++) def_osc[o] = opatch[o];   /* the built-in default, for slot 0 */
    for (k = 0; k < NFG; k++) def_fg[k] = fgpatch[k];
    for (k = 0; k < NSRC; k++)
        for (o = 0; o < NDST; o++) def_dep[k][o] = depth[k][o];
    patches_scan();
    {                                           /* every power-on a new patch: a boot count in flash, the timing */
        struct { uint32_t boots; } set = {0};
        settings_load(&set, sizeof set);
        set.boots++;
        settings_save(&set, sizeof set);        /* (silent yet: nothing has been pushed to the engine) */
        rnd_s ^= set.boots * 0x85EBCA6Bu;
#ifndef DINGHY_HOST
        rnd_s ^= fm1_ticks() * 0xC2B2AE35u;
#endif
        if (!rnd_s) rnd_s = 1u;
        rnd(1); rnd(1);
    }
    randomize();
    lcd_fill(0, 0, 240, 240, PH_BG);

    for (;;) {
        uint32_t rel, b, base, newpage;
        int shift, play;
        platform_poll();
        midi_poll();
        if (fm1_ms - t_hist >= 10u) {           /* the logic analyser's next sample */
            t_hist = fm1_ms;
            for (k = 0; k < NGG; k++) ghist[k][ghead] = gg_bits[k];
            ghead = (ghead + 1u) % HIST;
            if (ghead & 1u) {                   /* the SSGs' trace: every 20 ms */
                for (k = 0; k < NSSG; k++) shist[k][shead] = (int16_t)ssg_out[k];
                shead = (shead + 1u) % HIST;
            }
        }

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
        if (page == PG_PATCH) {
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) {
                static const uint8_t ORDER[NPATCH + 2] = {0, P_RANDOM, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
                uint32_t at = 0;
                while (ORDER[at] != pcur) at++;
                pcur = ORDER[cl((int32_t)at + e, 0, NPATCH + 1)];
                pconf = PC_NONE; pmsg = ""; dirty = 1;
            }
            if (b & (1u << BTN_SAVE)) {
                newpage = PG_PATCH;
                if (pcur == 0u || pcur == P_RANDOM) { pmsg = "CANNOT SAVE HERE"; t_msg = fm1_ms; }
                else if (pconf == PC_SAVE || !pused[pcur]) {
                    pmsg = patch_save(pcur) == 0 ? "SAVED" : "SAVE FAILED";
                    pconf = PC_NONE; t_msg = fm1_ms;
                } else pconf = PC_SAVE;
                dirty = 1;
            }
            if (b & (1u << BTN_SEL)) {
                if (!pused[pcur]) { pmsg = "EMPTY SLOT"; t_msg = fm1_ms; }
                else if (pconf == PC_LOAD) {
                    pmsg = patch_load(pcur) != 0 ? "LOAD FAILED" : pcur == P_RANDOM ? "ROLLED" : "LOADED";
                    pconf = PC_NONE; t_msg = fm1_ms;
                } else pconf = PC_LOAD;
                dirty = 1;
            }
            if (*pmsg && fm1_ms - t_msg > 2000u) { pmsg = ""; dirty = 1; }
        }
        if (newpage != page) { page = newpage; pconf = PC_NONE; pmsg = ""; dirty = hdr_dirty = 1; }

        for (k = 0; k < NFG; k++)               /* PLAY held on the FG page: the selected FG's gate */
            fg_manual[k] = (uint8_t)(page == PG_FG && k == fsel && play);
        if (page == PG_SH && b >> BTN_PLAY & 1u) ssg_tap[shift ? 1 : 0]++;   /* PLAY: S1, SEL + PLAY: S2 */

        if (page == PG_OSC || page == PG_WAVE) {
            patch_t *q;
            int ch = 0;
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) { sel = (uint32_t)cl((int32_t)sel + e, 0, NO - 1); dirty = 1; }
            q = &opatch[sel];
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) {
                if (shift && page == PG_OSC) otab[sel] = (uint8_t)cl(otab[sel] + (e > 0 ? 1 : -1), 0, NTAB - 1);   /* SEL + K1: the table */
                else q->pos = (uint8_t)cl(q->pos + 2 * e, 0, 127);
                ch = 1;
            }
            if (page == PG_WAVE) {
                if ((e = fm1_enc_take(ENC_KNOB2)) != 0) { otab[sel] = (uint8_t)cl(otab[sel] + (e > 0 ? 1 : -1), 0, NTAB - 1); ch = 1; }
                if ((e = fm1_enc_take(ENC_KNOB3)) != 0) { oscan[sel] = e > 0; ch = 1; }
                if (b >> BTN_PLAY & 1u && otab[sel] >= T_GEN) {   /* a new generative table, from a new seed */
                    gen_seed = rnd(0xFFFFFFFFu) | 1u;
                    gen_build(gen_seed);
                    ch = 1;
                }
            }
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
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) {
                mcs = (uint32_t)cl((int32_t)mcs + e, 0, NSRC - 1);
                if (mcs < mleft) mleft = mcs;
                if (mcs >= mleft + MCOLS) mleft = mcs - MCOLS + 1u;
                dirty = 1;
            }
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
        else if (page == PG_GLOBAL) {
            int ch = 0;
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) {   /* a new tuning keeps the scale's place where it can */
                uint32_t rel = (uint32_t)(tunpatch.scl - TUN_SCL0[tunpatch.tun]), nt;
                nt = (uint32_t)cl((int32_t)tunpatch.tun + e, 0, NTUN - 1);
                if (rel >= TUN_NSCL[nt]) rel = TUN_NSCL[nt] - 1u;
                tunpatch.tun = (uint8_t)nt;
                tunpatch.scl = (uint8_t)(TUN_SCL0[nt] + rel);
                ch = 1;
            }
            if ((e = fm1_enc_take(ENC_KNOB2)) != 0) {
                tunpatch.scl = (uint8_t)cl(tunpatch.scl + e, TUN_SCL0[tunpatch.tun], TUN_SCL0[tunpatch.tun] + TUN_NSCL[tunpatch.tun] - 1);
                ch = 1;
            }
            if ((e = fm1_enc_take(ENC_KNOB3)) != 0) { tunpatch.root = (uint8_t)((tunpatch.root + 12 + (e % 12)) % 12); ch = 1; }
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) { qsel = (uint32_t)cl((int32_t)qsel + e, 0, NO - 1); dirty = 1; }
            if ((e = fm1_enc_take(ENC_KNOB4)) != 0) { oq[qsel].glide = (uint8_t)cl(oq[qsel].glide + e, 0, 127); push_osc(qsel); dirty = 1; }
            if (b & (1u << BTN_PLAY)) { oq[qsel].on ^= 1u; push_osc(qsel); dirty = 1; }
            if (ch) { push_tun(); dirty = 1; }
        } else if (page == PG_SH) {
            int ch = 0;
            uint32_t l;
            for (l = 0; l < NSSG; l++) {
                if ((e = fm1_enc_take(l ? ENC_KNOB3 : ENC_KNOB1)) != 0) { shpatch.input[l] = (uint8_t)cl(shpatch.input[l] + e, 0, NSRC); ch = 1; }
                if ((e = fm1_enc_take(l ? ENC_KNOB4 : ENC_KNOB2)) != 0) { shpatch.slew[l] = (uint8_t)cl(shpatch.slew[l] + e, 0, 127); ch = 1; }
            }
            if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) { shpatch.qrate = (uint8_t)cl(shpatch.qrate + e, 0, 127); ch = 1; }
            if (ch) { push_sh(); dirty = 1; }
        } else if (page == PG_GATE) {
            ggpatch_t *q;
            int ch = 0;
            uint32_t l;
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) { gsel = (uint32_t)cl((int32_t)gsel + e, 0, NGG - 1); dirty = 1; }
            q = &ggpatch[gsel];
            if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) { q->notsrc = (uint8_t)cl(q->notsrc + e, 0, 7); ch = 1; }
            for (l = 0; l < 2u; l++) {
                if ((e = fm1_enc_take(l ? ENC_KNOB3 : ENC_KNOB1)) != 0) {
                    if (shift) q->mode[l] = e > 0 ? LM_CLK : LM_FREE;
                    else q->rate[l] = (uint8_t)cl(q->rate[l] + e, 0, 127);
                    ch = 1;
                }
                if ((e = fm1_enc_take(l ? ENC_KNOB4 : ENC_KNOB2)) != 0) { q->duty[l] = (uint8_t)cl(q->duty[l] + e, 0, 127); ch = 1; }
            }
            if (ch) { push_gg(gsel); dirty = 1; }
        } else if (page == PG_GMAT) {
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) {
                gcc = (uint32_t)cl((int32_t)gcc + e, 0, NGDST - 1);
                if (gcc < gleft) gleft = gcc;
                if (gcc >= gleft + GCOLS) gleft = gcc - GCOLS + 1u;
                dirty = 1;
            }
            if ((e = fm1_enc_take(ENC_KNOB2)) != 0 || (e = fm1_enc_take(ENC_SELECT)) != 0) {
                gcr = (uint32_t)cl((int32_t)gcr + e, 0, NGSRC - 1);
                if (gcr < gtop) gtop = gcr;
                if (gcr >= gtop + MROWS) gtop = gcr - MROWS + 1u;
                dirty = 1;
            }
            if (b & (1u << BTN_PLAY)) { gpatch[gcr] ^= 1u << gcc; push_gmat(); dirty = 1; }
        }
        if (page != PG_OSC) prev = fm1_in.notes;
        for (k = 0; k < 7u; k++) (void)fm1_enc_take(k);   /* turns a page does not use go nowhere */

        /* the keys' LEDs follow the page */
        for (k = 0; k < 27u; k++) {
            uint32_t on = 0, glow = 0;
            if (page == PG_OSC) {
                for (o = 0; o < NO; o++)
                    if (opatch[o].note == base + k) { if (o == sel) on = 1; else glow = 1; }
            } else if (page == PG_GLOBAL) {        /* the oscillators' notes lit; a 12-step scale glows */
                uint32_t note = base + k;
                for (o = 0; o < NO; o++)
                    if ((uint32_t)((q_pitch_v[o] + 128) >> 8) == note) on = 1;
                if (TUN_N[g_tun] == 12u) {
                    uint32_t pc = (note + 12u - g_root) % 12u;
                    glow = SCL_LO[g_scl] >> pc & 1u;
                }
            } else if (page == PG_SH) {            /* S1 on the lower keys, S2 on the upper */
                uint32_t p1 = (src_disp(S_SSG(0)) * 12u) >> 15, p2 = 14u + ((src_disp(S_SSG(1)) * 12u) >> 15);
                on = k == p1 || k == p2;
            } else if (page == PG_GATE) {          /* the nine outputs, every third key */
                on = k % 3u == 0u && (uint32_t)gg_bits[gsel] >> (k / 3u) & 1u;
            } else if (page == PG_GMAT) {          /* the whole keyboard flashes with the source */
                on = gsrc_on(gcr);
            } else if (page == PG_FG || page == PG_MOD) {
                uint32_t pos = ((page == PG_FG ? (uint32_t)fg_out[fsel] : src_disp(mcs)) * 26u) >> 15;
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
            fm1_led_key(BTN_SEL, shift || ((page == PG_INDEX || page == PG_PATCH) && blink));
            fm1_led_key(BTN_HOME, page != PG_INDEX && blink);   /* away from the index: HOME leads back */
            fm1_led_key(BTN_SAVE, page == PG_PATCH && (pconf == PC_SAVE || blink));
            fm1_led_key(BTN_PLAY, ((page == PG_FG || page == PG_SH) && (play || blink))   /* a manual gate / sample */
                                  || (page == PG_WAVE && otab[sel] >= T_GEN && blink)   /* a new generative table */
                                  || (page == PG_GLOBAL && (oq[qsel].on || blink))   /* solid: this one quantises */
                                  || (page == PG_MOD && (depth[mcs][DLIST[mcr]] || blink))   /* solid: a pin here */
                                  || (page == PG_GMAT && (gpatch[gcr] >> gcc & 1u || blink)));
        }
        fm1_led_key(BTN_EDIT, page == PG_OSC || page == PG_WAVE);
        fm1_led_key(BTN_ENV, page == PG_FG || page == PG_SH);
        fm1_led_key(BTN_ARP, page == PG_MOD);
        fm1_led_key(BTN_LFO, page == PG_GATE);
        fm1_led_key(BTN_SEQ, page == PG_GMAT);
        fm1_led_key(BTN_GLO, page == PG_GLOBAL);

        if (fm1_ms - t_cpu > 500u) {            /* the worst render of the last half second, in the header */
            uint32_t us = audio_us_max, pct;
            audio_us_max = 0;
            t_cpu = fm1_ms;
            pct = us * 100u / 5805u;            /* 256 frames at 44.1 kHz = 5805 us */
            if (us && pct != cpu_pct) { cpu_pct = pct; hdr_dirty = 1; }
        }
        if (page == PG_OSC && shift != shift_drawn) { shift_drawn = shift; dirty = 1; }
        if ((page == PG_FG || page == PG_MOD || page == PG_WAVE || page == PG_GATE || page == PG_GMAT || page == PG_SH
             || page == PG_GLOBAL)
            && fm1_ms - t_live > 50u) { t_live = fm1_ms; dirty = 1; }

        idle_ok = 1;                            /* MIDI in while the screen draws */
        if (hdr_dirty) { hdr_dirty = 0; draw_header(page, cpu_pct); }
        if (dirty) {
            dirty = 0;
            switch (page) {
            case PG_INDEX: draw_index(icur); break;
            case PG_OSC: draw_osc(sel, shift); break;
            case PG_WAVE: draw_wave(sel, ost[sel].posq); break;
            case PG_FG: draw_fg(fsel); break;
            case PG_MOD: draw_mod(mcs, mcr, mtop, mleft); break;
            case PG_PATCH: draw_patch(pcur, pconf, pmsg); break;
            case PG_GATE: draw_gg(gsel); break;
            case PG_SH: draw_sh(); break;
            case PG_GLOBAL: draw_global(qsel); break;
            case PG_GMAT: draw_gmat(gcc, gcr, gtop, gleft); break;
            default: draw_soon(page); break;
            }
        }
        idle_ok = 0;
    }
}
