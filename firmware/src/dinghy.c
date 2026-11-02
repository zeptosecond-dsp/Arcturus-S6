/* SPDX-License-Identifier: GPL-3.0-only */
/* QUORUM Cellular Synthesis System: build 0.1 (stage 1: the engine and its pages). The sister of Arcturus S6 (the
 * main branch), on Dinghy (zvenson), on the platform of Felucca (Hügelton Instruments) as SLOOP (isod89) carries it.
 *
 * 64 sine partials at 22.05 kHz (each sample held twice) and 12 bit, as S6. Two 32-cell automata switch them; two
 * strikes ring a partial's overtone series (Strike, Mallet, Bow, Breath); every partial is split into path A or B as
 * it is made, and each path has its own filter (vowel, or a 12 / 24 dB state-variable band pass), level, pan and
 * pitch offset. Logic blocks on the clock and the cells fire the strikes.
 *
 *   HOME   1 INDEX      SELECT: a page, SEL: open it
 *   EDIT   2 VOICE      K1 slew, K2 tilt, K3 stretch, K4 level; ALGORITHM root; PRESETS clock; SEL anchor
 *   LFO    3 GRIDS      OCT: grid A / B; K1-K4 the bank shown, SEL (flashing) the other bank; PLAY reseeds
 *   ENV    4 STRIKE     OCT: strike 1 / 2; K1-K4, SEL: three banks; ALGORITHM sum / filter; PRESETS routing; PLAY fires
 *   SEL+EDIT 5 SPLIT    OCT: SPLIT / PATH A / PATH B; K1-K4, SEL: two banks on the paths; ALGORITHM N
 *   GLO    G GLOBAL     the CPU; K1 clock source, K2 tempo, K3 phosphor, K4 brightness
 *   the keys: the root (F1 .. G3); their LEDs show the cells
 *
 * Rules of the road (the chip has no FPU and no libgcc): integers only, no float, no 64-bit division.
 * tests/host.c runs this file on a computer: the screen as a PNG, the sound as a WAV. */
#define APP_NAME "QUORUM"
#define Q_VERSION "0.2"
static void app_idle(void);
#define LCD_IDLE app_idle                       /* while the screen's DMA runs: the clock and MIDI keep going */
#define CV_MAX (240u * 218u)                    /* the page body's canvas */
#define STORE_MAGIC 0x4D525551u                 /* "QURM": never Arcturus's flash data */
#include "platform.c"
#include "font_chunky.h"
#define FONT_S FONT_C
#include "luts.h"
#include "ca_tables.h"
#include "q_tables.h"

static int32_t cl(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }
static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

/* ================================================================== the engine (audio interrupt) */
#define NP 64u                                  /* partials */
#define NC 32u                                  /* cells a grid */
#define SUB 32u                                 /* frames a block: levels move once a block (1378 Hz) */
#define SIN_BITS 12u
#define ONE 32768                               /* 1.0 in the weights (Q15) */
static int16_t SINT[1u << SIN_BITS];

/* per partial: the phase, the grid's slewed level (Q23), the levels played last block into path A and B */
static uint32_t ph[NP];
static int32_t genv[NP], pamp[2][NP];
static volatile int32_t gtgt_v[NP];             /* the grid's level to slew to, Q15 */
static volatile uint32_t inc_v[NP], kh_v[NP];   /* the phase step (at 44.1 kHz); the grid slew step a block, Q16 */
/* the sends, Q15 (with the paths' vowel and the strikes' levels in them): 0 grid->A, 1 grid->B, 2 strike 1->A,
 * 3 strike 1->B, 4 strike 2->A, 5 strike 2->B. The main loop sets the target, the interrupt glides to it (~30 ms). */
static volatile int32_t wt_v[6][NP];
static int32_t ws[6][NP];

/* the strikes: per strike and partial, the envelope (Q23); an attack in blocks still to run and its step */
static int32_t ke[2][NP], katd[2][NP];
static uint16_t katn[2][NP];
static volatile int32_t kadd_v[2][NP];          /* a strike's kick (Q15), read when kseq moves on */
static volatile uint16_t katt_v[2][NP];         /* its attack, blocks */
static volatile int32_t ktgt_v[2][NP];          /* Bow / Breath: the level held while the gate is high, Q15 */
static volatile uint32_t kku_v[2][NP], kkd_v[2][NP];   /* the steps up (Bow / Breath) and down, a block, Q16 */
static volatile uint16_t kfl_v[2][NP];          /* the flicker: Bow's flutter, Breath's air (Q15 depth) */
static volatile uint32_t kseq_v[2], kimp_v[2], kheld_v[2];   /* a new kick; Strike / Mallet (1) or Bow / Breath; gate */
static uint32_t kseen[2];
static uint32_t frnd = 0x2545F491u;

/* the paths: mode (0 off, 1 vowel, 2 band pass 12 dB, 3 band pass 24 dB), the SVFs' f and damping (Q14), makeup
 * (Q12), the gains to left and right with the level in them (Q15) */
static volatile uint8_t pm_v[2];
static volatile int32_t pf_v[2][2], pq_v[2], pmk_v[2], pgl_v[2], pgr_v[2];
typedef struct { int32_t lo, bp; } svf_t;
static svf_t svf[2][2];

static volatile uint32_t us_sum, us_n, us_peak; /* the interrupt's time, for the CPU line */
static volatile uint32_t bits_v;                /* the engine: 0 smooth (16 bit inside, 12 at the output), 1 12 bit, 2 8 bit */
static volatile uint32_t live_v;
static volatile int32_t kshow_v[2][NP];         /* each strike's level now (for the screens) */

static int32_t sine(uint32_t p)                 /* a full cycle over 2^32 -> -32767..32767 (a refined parabola) */
{
    int32_t x = (int32_t)p >> 16, ax = x < 0 ? -x : x, y, ay;
    y = (x * (32768 - ax)) >> 13;
    ay = y < 0 ? -y : y;
    y += ((((y * ay) >> 15) - y) * 7373) >> 15;
    return y > 32767 ? 32767 : y < -32767 ? -32767 : y;
}

static const uint32_t INC_C9[12] = {            /* MIDI 120..131 (C9..B9); lower octaves shift right */
    815363807u, 863847862u, 915214929u, 969636441u, 1027294024u, 1088380105u,
    1153098554u, 1221665363u, 1294309365u, 1371273005u, 1452813141u, 1539201906u};
#define Q_TOP (124 * 256)                       /* E9, 10.5 kHz: the highest partial, under 22.05 kHz's Nyquist */
#define Q_FADE (117 * 256)                      /* A8, 7 kHz: the partials fade out from here */
static uint32_t pitch_inc(int32_t q)            /* 1/256 semitone (MIDI 0 .. 131) -> phase step at 44.1 kHz */
{
    uint32_t s, f, inc;
    q = cl(q, 0, 131 * 256 + 255);
    s = (uint32_t)q >> 8;
    f = (uint32_t)q & 255u;
    inc = INC_C9[s % 12u] >> (10u - s / 12u);
    return (uint32_t)(((uint64_t)inc * LUT_SEMIFRAC[f]) >> 16);
}
static uint32_t q_hz(int32_t q) { return (uint32_t)(((uint64_t)pitch_inc(q) * 44100u) >> 32); }

static int32_t knee(int32_t x)                  /* a soft ceiling for stacked strikes: linear to 8000, then to 16000 */
{
    if (x <= 8000) return x;
    return 8000 + (x - 8000) * 8000 / x;
}

static int32_t svf_bp(svf_t *s, int32_t x, int32_t f, int32_t q)   /* a Chamberlin band pass, run twice (44.1 kHz) */
{
    uint32_t k;
    for (k = 0; k < 2u; k++) {
        int32_t hp = x - s->lo - (int32_t)(((int64_t)q * s->bp) >> 14);
        s->bp += (int32_t)(((int64_t)f * hp) >> 14);
        s->bp = cl(s->bp, -3000000, 3000000);
        s->lo += (int32_t)(((int64_t)f * s->bp) >> 14);
        s->lo = cl(s->lo, -3000000, 3000000);
    }
    return (int32_t)(((int64_t)s->bp * q) >> 14);   /* the peak at 0 dB, whatever the resonance */
}

static int32_t soft_out(int32_t x)              /* linear to 20000, then bending toward 32000 */
{
    int32_t a = iabs(x), y;
    if (a <= 20000) return x;
    a -= 20000;
    if (a > 150000) a = 150000;                 /* (a x 12000 stays in 32 bits) */
    y = 20000 + a * 12000 / (a + 12000);
    return x < 0 ? -y : y;
}
static void render(int32_t *out)                /* SUB frames at 44.1 kHz: 16 computed, each held twice */
{
    const uint32_t nf = SUB / 2u, bq = bits_v;
    const int32_t qm = bq == 2u ? ~255 : ~15, qh = bq == 2u ? 128 : 8;   /* the bus's word: 8 bit, else 12 */
    const int32_t lm = bq == 2u ? ~63 : ~3, lh = bq == 2u ? 32 : 2;      /* a level's: 256 / 4096 steps of 16384 */
    int32_t ba[SUB / 2], bb[SUB / 2];
    uint32_t h, i, s, live = 0;
    for (i = 0; i < nf; i++) ba[i] = bb[i] = 0;
    for (s = 0; s < 2u; s++)
        if (kseq_v[s] != kseen[s]) {            /* a strike: kick its members (the main loop wrote them, then kseq) */
            kseen[s] = kseq_v[s];
            if (kimp_v[s])
                for (h = 0; h < NP; h++) {
                    int32_t add = kadd_v[s][h], pk, n;
                    if (add <= 0) continue;
                    pk = knee((ke[s][h] >> 8) + add);
                    n = katt_v[s][h] ? katt_v[s][h] : 1;
                    katn[s][h] = (uint16_t)n;
                    katd[s][h] = ((pk << 8) - ke[s][h]) / n;
                }
        }
    for (h = 0; h < NP; h++) {
        int32_t t = gtgt_v[h], gl, kl[2], a1, b1, a, b, da, db, j;
        uint32_t p, inc;
        /* the grid's level, slewed */
        genv[h] += (int32_t)(((int64_t)((t << 8) - genv[h]) * (int32_t)kh_v[h]) >> 16);
        if (t == 0 && genv[h] < 512) genv[h] = 0;
        gl = genv[h] >> 8;
        /* the strikes */
        for (s = 0; s < 2u; s++) {
            int32_t e = ke[s][h];
            if (katn[s][h]) { e += katd[s][h]; katn[s][h]--; }
            else if (e || (!kimp_v[s] && kheld_v[s])) {
                int32_t tt = (kimp_v[s] || !kheld_v[s]) ? 0 : ktgt_v[s][h] << 8;
                int32_t k = (int32_t)(e < tt ? kku_v[s][h] : kkd_v[s][h]);
                e += (int32_t)(((int64_t)(tt - e) * k) >> 16);
                if (tt == 0 && e < 256) e = 0;
            }
            if (e < 0) e = 0;
            ke[s][h] = e;
            kl[s] = e >> 8;
            if (kl[s] && kfl_v[s][h]) {         /* Bow's flutter, Breath's air: the level shaken a little each block */
                uint32_t r;
                frnd ^= frnd << 13; frnd ^= frnd >> 17; frnd ^= frnd << 5;
                r = frnd >> 17;                 /* 0..32767 */
                kl[s] -= (kl[s] * (int32_t)((kfl_v[s][h] * r) >> 15)) >> 15;
            }
            kshow_v[s][h] = kl[s];
        }
        /* the sends, gliding */
        for (j = 0; j < 6; j++) ws[j][h] += ((wt_v[j][h] - ws[j][h]) * 1585) >> 16;
        a1 = ((gl * ws[0][h]) >> 15) + ((kl[0] * ws[2][h]) >> 15) + ((kl[1] * ws[4][h]) >> 15);
        b1 = ((gl * ws[1][h]) >> 15) + ((kl[0] * ws[3][h]) >> 15) + ((kl[1] * ws[5][h]) >> 15);
        a = pamp[0][h];
        b = pamp[1][h];
        if (!a1 && !b1 && !a && !b) continue;   /* silent: costs nothing */
        inc = inc_v[h] << 1;                    /* (22 kHz: twice the step) */
        p = ph[h];
        if (!bq) {                              /* smooth: a straight ramp across the block */
            da = (a1 - a) >> 4;
            db = (b1 - b) >> 4;
            for (i = 0; i < nf; i++) {
                int32_t sv;
                p += inc;
                sv = SINT[p >> (32u - SIN_BITS)];
                ba[i] += (sv * a) >> 15;
                bb[i] += (sv * b) >> 15;
                a += da;
                b += db;
            }
        } else {                                /* 12 / 8 bit: the level held for the block (it steps), in 4096 / 256
                                                 * steps of the loudest a partial gets; the sine and every product in
                                                 * words of that length too (rounded) */
            a1 = (cl(a1, 0, 16383) + lh) & lm;
            b1 = (cl(b1, 0, 16383) + lh) & lm;
            for (i = 0; i < nf; i++) {
                int32_t sv;
                p += inc;
                sv = (SINT[p >> (32u - SIN_BITS)] + qh) & qm;
                ba[i] += (((sv * a1) >> 13) + qh) & qm;   /* (x 4 here, not at the output: the words are the DAC's) */
                bb[i] += (((sv * b1) >> 13) + qh) & qm;
            }
        }
        ph[h] = p;
        pamp[0][h] = a1;
        pamp[1][h] = b1;
        live++;
    }
    for (i = 0; i < nf; i++) {                  /* the paths: filter, makeup, level and pan; then 12 bit, each twice */
        int32_t x[2], l = 0, r = 0, k;
        x[0] = ba[i]; x[1] = bb[i];
        for (k = 0; k < 2; k++) {
            uint32_t m = pm_v[k];
            int32_t y = x[k];
            if (m >= 2u) {
                y = svf_bp(&svf[k][0], y, pf_v[k][0], pq_v[k]);
                if (m == 3u) y = svf_bp(&svf[k][1], y, pf_v[k][1], pq_v[k]);
                y = (y * pmk_v[k]) >> 12;
                if (bq) y = (y + qh) & qm;
            }
            l += (y * pgl_v[k]) >> 15;
            r += (y * pgr_v[k]) >> 15;
        }
        l = (soft_out(bq ? l : l * 4) + qh) & qm;
        r = (soft_out(bq ? r : r * 4) + qh) & qm;
        out[4u * i] += l; out[4u * i + 1u] += r;
        out[4u * i + 2u] += l; out[4u * i + 3u] += r;
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

/* ================================================================== the patch */
enum { SR_CUT, SR_BAND, SR_ODD, SR_NTH, SR_GRID, SR_SRC, NSR };
enum { GM_LOWHIGH, GM_ODDEVEN, GM_AND, GM_OR, GM_XOR, NGM };
enum { KM_STRIKE, KM_MALLET, KM_BOW, KM_BREATH, NKM };
enum { SE_ALL, SE_ODD, SE_EVEN };
typedef struct { uint8_t rule, div, mut, rescue, dens, shift; } gpar_t;
typedef struct { uint8_t mode, part, vel, level, shape, decay, fall, series, a, op, b, filt; } kpar_t;
typedef struct { uint8_t mode, freq, res, bw, level, pan; int8_t coarse, fine; } ppar_t;
static struct {
    uint8_t root, slew, tilt, level, anchor, tempo, clk_midi, gmode, bits;
    int8_t stretch;                             /* per mille of the exponent: -30 .. +60 */
    gpar_t g[2];
    kpar_t k[2];
    uint8_t route_pre;                          /* 0: strike 1 -> A, strike 2 -> B; 1: through the split */
    uint8_t srule, spoint, swidth, ssoft, sn;
    ppar_t pth[2];
} P = {
    33, 32, 100, 100, 1, 64, 0, GM_LOWHIGH, 0, 0,
    {{90, 4, 3, 1, 30, 0}, {30, 2, 4, 1, 30, 0}},
    {{KM_STRIKE, 3, 89, 100, 64, 70, 76, SE_ALL, 15, 0, 3, 0},       /* A cell 5 AND clock /3 */
     {KM_MALLET, 2, 76, 100, 44, 82, 51, SE_ALL, 46, 0, 5, 0}},      /* B cell 3 AND clock /5 */
    0,
    SR_CUT, 64, 15, 12, 3,
    {{1, 0, 40, 12, 90, 37, 0, 0}, {3, 79, 30, 12, 100, 91, 0, 0}},
};
static const char *const GM_NAME[NGM] = {"LO/HI", "ODD/EVN", "A AND B", "A OR B", "A XOR B"};
static const char *const KM_NAME[NKM] = {"STRIKE", "MALLET", "BOW", "BREATH"};
static const char *const KSH_NAME[NKM] = {"HARD", "SOFT", "PRESS", "AIR"};
static const char *const SE_NAME[3] = {"ALL", "ODD", "EVEN"};
static const char *const SR_NAME[NSR] = {"CUTOFF", "BAND", "ODD/EVN", "EVERY N", "GRID AB", "SOURCE"};
static const char *const PM_NAME[4] = {"OFF", "VOWEL", "BP 12DB", "BP 24DB"};
static const char *const OP_NAME[6] = {"AND", "OR", "XOR", "NAND", "NOR", "XNOR"};

/* ================================================================== the grids (main loop) */
static uint32_t rnd_s = 0x9E3779B9u;
static uint32_t rnd(uint32_t n) { rnd_s ^= rnd_s << 13; rnd_s ^= rnd_s >> 17; rnd_s ^= rnd_s << 5; return n ? rnd_s % n : 0u; }
#define GHIST 28u
#define SEEN 32u
typedef struct { uint32_t row, hist[GHIST], seen[SEEN], nseen, head, count, rescue_t; const char *why; } grid_t;
static grid_t G[2];
static uint32_t popc(uint32_t v) { uint32_t n = 0; while (v) { v &= v - 1u; n++; } return n; }
static uint32_t rotl1(uint32_t v) { return v << 1 | v >> 31; }
static uint32_t rotr1(uint32_t v) { return v >> 1 | v << 31; }
static void grid_seed(uint32_t i)
{
    uint32_t c, r = 0;
    for (c = 0; c < NC; c++) if (rnd(100) < P.g[i].dens) r |= 1u << c;
    if (!r) r = 1u;
    G[i].row = r;
    G[i].nseen = 0;
}
static void grid_step(uint32_t i)
{
    grid_t *g = &G[i];
    uint32_t c, n = 0, rule = P.g[i].rule, l = rotl1(g->row), r = rotr1(g->row), k;
    for (c = 0; c < NC; c++) {                  /* cell c: its left neighbour is c - 1 (wrapping) */
        uint32_t idx = ((l >> c) & 1u) << 2 | ((g->row >> c) & 1u) << 1 | ((r >> c) & 1u);
        n |= ((rule >> idx) & 1u) << c;
    }
    for (c = 0; c < NC; c++) if (rnd(100) < P.g[i].mut) n ^= 1u << c;
    g->row = n;
    if (P.g[i].rescue) {                        /* died, filled up, or a row seen in the last 32: a few cells flipped */
        uint32_t p = popc(n), looped = 0;
        for (k = 0; k < g->nseen; k++) if (g->seen[k] == n) looped = 1;
        g->why = p == 0u ? "DIED" : p == NC ? "FILLED" : looped ? "LOOPED" : "";
        if (*g->why) {
            for (k = 0; k < 5u; k++) g->row ^= 1u << rnd(NC);
            g->rescue_t = fm1_ms ? fm1_ms : 1u;
            g->nseen = 0;
        }
    }
    if (g->nseen < SEEN) g->seen[g->nseen++] = g->row;
    else { for (k = 1; k < SEEN; k++) g->seen[k - 1u] = g->seen[k]; g->seen[SEEN - 1u] = g->row; }
    g->hist[g->head] = g->row;
    g->head = (g->head + 1u) % GHIST;
}
static uint32_t cell(uint32_t i, int32_t c) { return G[i].row >> (uint32_t)(((c % 32) + 32) % 32) & 1u; }
static uint32_t cell_on(uint32_t h)             /* partial h (1..64) per the grids and how they share it */
{
    int32_t s = P.g[0].shift;
    switch (P.gmode) {
    case GM_LOWHIGH: return h <= 32u ? cell(0, (int32_t)h - 1 + s) : cell(1, (int32_t)h - 33 + P.g[1].shift);
    case GM_ODDEVEN: return h & 1u ? cell(0, (int32_t)(h - 1u) / 2 + s) : cell(1, (int32_t)h / 2 - 1 + P.g[1].shift);
    default: {
        uint32_t a = cell(0, (int32_t)((h - 1u) >> 1) + s), b = cell(1, (int32_t)((h - 1u) >> 1) + P.g[1].shift);
        return P.gmode == GM_AND ? a & b : P.gmode == GM_OR ? a | b : a ^ b;
    }
    }
}
static uint32_t owner_a(uint32_t h)
{
    if (P.gmode == GM_LOWHIGH) return h <= 32u;
    if (P.gmode == GM_ODDEVEN) return h & 1u;
    return cell(0, (int32_t)((h - 1u) >> 1) + P.g[0].shift);
}

/* ================================================================== the partials: pitch, levels, sends */
static int32_t pq_base[NP], pq_now[NP];         /* each partial's pitch, 1/256 semitone: before and after the offsets */
static int32_t wsplit[NP];                      /* each partial's share of path A, Q15 */
static int32_t gshow[NP];                       /* the grid's level for the screens */
static int32_t vow_show[2][NP];

static uint32_t pow_neg(uint32_t h, uint32_t e_q8)   /* h ^ -e (h 1..64, e in 1/256), Q15 */
{
    uint32_t e = e_q8 * CA_L2[h - 1u] >> 8, ip = e >> 12, fp = (e & 4095u) >> 4;
    return ip >= 31u ? 0u : (2147483648u >> ip) / LUT_OCTFRAC[fp];
}
static int32_t vowel_w(uint32_t hz, uint32_t pos, uint32_t res)   /* a path's vowel weight at hz, Q15 */
{
    uint32_t v = pos * 4u * 256u / 127u, i = v >> 8, t = v & 255u, k;   /* 0 .. 4: A E I O U, blended */
    int32_t w = 1638, bwk = 1311 * 60 / (36 + (int32_t)Q_RES_X10[res]);   /* 0.32 / (0.6 + Q / 6), Q12 */
    if (i >= 4u) { i = 3u; t = 256u; }
    for (k = 0; k < 3u; k++) {
        int32_t c = (int32_t)((Q_FORM_HZ[i * 3u + k] * (256u - t) + Q_FORM_HZ[(i + 1u) * 3u + k] * t) >> 8);
        int32_t lv = (int32_t)((Q_FORM_LV[i * 3u + k] * (256u - t) + Q_FORM_LV[(i + 1u) * 3u + k] * t) >> 8);
        int32_t bwhz = c * bwk / 4096 + 1, x = iabs((int32_t)hz - c) * 16 / bwhz;
        if (x < 64) w += (lv * Q_GAUSS[x]) >> 15;
    }
    if (w > 39321) w = 39321;
    return 3932 + ((w * 28836) >> 15);          /* 0.12 + 0.88 x the bells */
}
static uint32_t split_w(uint32_t h, int32_t qb)
{
    int32_t qc = 8858 + (int32_t)P.spoint * (30720 - 8858) / 127, soft = (int32_t)P.ssoft * 256, x;
    switch (P.srule) {
    case SR_ODD: return h & 1u ? ONE : 0;
    case SR_NTH: return h % (P.sn < 2u ? 2u : P.sn) == 0u ? ONE : 0;
    case SR_GRID: return owner_a(h) ? ONE : 0;
    case SR_SRC: return ONE;
    case SR_CUT: x = qb - qc; break;
    default: x = iabs(qb - qc) - (int32_t)P.swidth * 256 * 12 / 20; break;   /* width in 1/10 octave: half of it each side */
    }
    if (soft <= 0) return x < 0 ? ONE : 0;
    x = cl(x, -60000, 60000);                   /* (x * 16384 stays in 32 bits: no 64-bit division on this chip) */
    return (uint32_t)cl(16384 - x * 16384 / soft * 2, 0, ONE);
}
static void path_push(void)
{
    uint32_t k;
    for (k = 0; k < 2u; k++) {
        const ppar_t *q = &P.pth[k];
        int32_t sp = (int32_t)q->bw * 19 / 20;   /* half the bandwidth in knob steps (19.2 a octave; bw in 1/20 oct) */
        int32_t lv = (int32_t)q->level * 258;   /* 0..127 -> ~1.0 at 127 */
        pf_v[k][0] = Q_SVF_F[q->mode == 3u ? cl((int32_t)q->freq - sp, 0, 127) : q->freq];
        pf_v[k][1] = Q_SVF_F[cl((int32_t)q->freq + sp, 0, 127)];
        pq_v[k] = Q_RES_D[q->res];
        pmk_v[k] = q->mode == 3u ? 10650 : 6554;
        pgl_v[k] = (int32_t)LUT_PAN_L[q->pan & 127u] * lv >> 15;
        pgr_v[k] = (int32_t)LUT_PAN_R[q->pan & 127u] * lv >> 15;
        pm_v[k] = q->mode;
    }
}
static void push_partials(void)
{
    uint32_t h, sum = 0, w[NP], kg = CA_SLEW_K[P.slew];
    int32_t offA = (int32_t)P.pth[0].coarse * 256 + (int32_t)P.pth[0].fine * 256 / 100;
    int32_t offB = (int32_t)P.pth[1].coarse * 256 + (int32_t)P.pth[1].fine * 256 / 100;
    int32_t klev[2];
    klev[0] = P.k[0].level * 258; klev[1] = P.k[1].level * 258;   /* 0..127 -> Q15 (127: ~1.0) */
    for (h = 1; h <= NP; h++) {
        int32_t st = (int32_t)(CA_L2[h - 1u] * 3u / 4u), qb = (int32_t)P.root * 256 + st + st * P.stretch / 1000, q;
        uint32_t on, wt = 0, sw = split_w(h, qb);
        q = qb + (int32_t)(((int64_t)sw * offA + (int64_t)(ONE - (int32_t)sw) * offB) >> 15);
        pq_base[h - 1u] = qb; pq_now[h - 1u] = q; wsplit[h - 1u] = (int32_t)sw;
        on = cell_on(h) || (h == 1u && P.anchor);
        if (on && q <= Q_TOP) {
            wt = pow_neg(h, P.tilt * 256u / 100u);
            if (q > Q_FADE) wt = wt * (uint32_t)(Q_TOP - q) / (uint32_t)(Q_TOP - Q_FADE);
        }
        w[h - 1u] = wt;
        sum += wt;
    }
    for (h = 0; h < NP; h++) {
        uint32_t semi = (uint32_t)cl(pq_now[h], 0, 131 * 256) >> 8, kf = CA_KF[semi < 132u ? semi : 131u];
        uint32_t s = sum < 40000u ? 40000u : sum, hz, k;
        int32_t lvl = (int32_t)(w[h] * 16000u / s), top = ONE, va, vb, gw, kw;
        inc_v[h] = pitch_inc(pq_now[h]);
        kh_v[h] = kg < kf ? kg : kf;
        gtgt_v[h] = lvl;
        gshow[h] = lvl;
        hz = q_hz(pq_now[h]);
        if (pq_now[h] > Q_TOP) top = 0;
        else if (pq_now[h] > Q_FADE) top = (Q_TOP - pq_now[h]) * ONE / (Q_TOP - Q_FADE);
        va = P.pth[0].mode == 1u ? vowel_w(hz, P.pth[0].freq, P.pth[0].res) : ONE;
        vb = P.pth[1].mode == 1u ? vowel_w(hz, P.pth[1].freq, P.pth[1].res) : ONE;
        vow_show[0][h] = va; vow_show[1][h] = vb;
        va = (va * top) >> 15; vb = (vb * top) >> 15;
        gw = P.srule == SR_SRC ? ONE : wsplit[h];
        wt_v[0][h] = (gw * va) >> 15;
        wt_v[1][h] = ((ONE - gw) * vb) >> 15;
        for (k = 0; k < 2u; k++) {
            kw = !P.route_pre ? (k == 0u ? ONE : 0) : P.srule == SR_SRC ? 0 : wsplit[h];
            wt_v[2u + 2u * k][h] = (((kw * va) >> 15) * klev[k]) >> 15;
            wt_v[3u + 2u * k][h] = ((((ONE - kw) * vb) >> 15) * klev[k]) >> 15;
        }
    }
    path_push();
    bits_v = P.bits;
}

/* ================================================================== the strikes (main loop) */
static uint32_t kflash[2], kman[2];             /* a strike just fired (for the lamps); held by hand until */
static uint8_t kmember[2][NP];                  /* the partials the last strike rang */
static uint32_t tau_k(uint32_t tau_ms)          /* a one-pole's step a block for a time constant, Q16 */
{
    uint32_t b = tau_ms * 1378u / 1000u;
    return b < 1u ? 65536u : 65536u / b;
}
static void strike_fire(uint32_t s)             /* a rising gate (or PLAY) */
{
    const kpar_t *k = &P.k[s];
    uint32_t h, m, n = 0, list_h[NP], list_m[NP], list_i[NP], sust = k->mode >= KM_BOW;
    int32_t vel = (int32_t)k->vel * 258, shp = k->shape;   /* Q15; 0..127 */
    for (h = 0; h < NP; h++) { kadd_v[s][h] = 0; kmember[s][h] = 0; if (sust) ktgt_v[s][h] = 0; }
    if (k->mode == KM_MALLET) {                 /* the bar's ratios: 1, 4, 10 */
        static const uint8_t R[3] = {1, 4, 10};
        for (m = 0; m < 3u; m++) if ((uint32_t)k->part * R[m] <= NP) { list_h[n] = k->part * R[m]; list_m[n] = R[m]; list_i[n] = m; n++; }
    } else
        for (m = 1; (uint32_t)k->part * m <= NP; m++) {
            if (m > 1u && k->series == SE_ODD && !(m & 1u)) continue;
            if (m > 1u && k->series == SE_EVEN && (m & 1u)) continue;
            list_h[n] = k->part * m; list_m[n] = m; list_i[n] = m - 1u; n++;
        }
    for (m = 0; m < n; m++) {
        uint32_t hh = list_h[m], p = hh - 1u, mm = list_m[m], hz = q_hz(pq_now[p]), dec, fallq;
        int32_t a;
        if (pq_now[p] > Q_TOP) continue;
        if (k->filt && !cell_on(hh)) continue;
        if (hz < 20u) hz = 20u;
        fallq = 508u + (k->fall * 5u * (mm - 1u)) / 2u;   /* 4 x 127 + fall x 2.5 x (m - 1) */
        dec = Q_DEC_MS[k->decay] * 4u * 127u / (3u * fallq);   /* higher partials die faster (the LPG) */
        if (dec < 30u) dec = 30u;
        kkd_v[s][p] = tau_k(dec);
        kmember[s][p] = 1;
        if (k->mode == KM_STRIKE || k->mode == KM_MALLET) {
            uint32_t att_us;
            if (k->mode == KM_STRIKE) {
                uint32_t e_q8 = 435u - 307u * (uint32_t)shp / 127u, reach = 1u + (uint32_t)(vel * (3 + shp * 22 / 127)) / 32768u;
                a = (int32_t)(((int64_t)((vel * 6554) >> 15) * pow_neg(mm, e_q8)) >> 15);
                if (mm > reach) a >>= (mm - reach) > 15u ? 15u : mm - reach;
                att_us = 1500000u / hz; if (att_us < 1500u) att_us = 1500u;
            } else {
                static const int32_t F0[3] = {32768, 14746, 8192};   /* 1, 0.45, 0.25 */
                int32_t f = F0[list_i[m]];
                if (list_i[m] == 1u) f = (int32_t)(((int64_t)f * (32768 - 19661 * shp / 127) >> 15) * (13107 + vel) >> 15);
                if (list_i[m] == 2u) f = (int32_t)(((int64_t)f * (32768 - 27853 * shp / 127) >> 15) * vel >> 15);
                a = (((vel * 7209) >> 15) * f) >> 15;   /* 0.22 x vel */
                att_us = 4000u + 30000u * (uint32_t)shp / 127u;
                if (att_us < 2000000u / hz) att_us = 2000000u / hz;
            }
            if (a < 3) { kmember[s][p] = 0; continue; }
            kadd_v[s][p] = a;
            katt_v[s][p] = (uint16_t)cl((int32_t)(att_us * 1378u / 1000000u), 2, 4000);
            kfl_v[s][p] = 0;
        } else {                                /* Bow / Breath: a level to hold, an attack, a flicker */
            uint32_t tau;
            if (k->mode == KM_BOW) {
                a = (int32_t)(((int64_t)((vel * 4915) >> 15) * pow_neg(mm, 256)) >> 15);   /* 0.15 x vel / m */
                if ((int32_t)mm * 32768 > 32768 + vel * 18) a >>= 2;
                tau = (30u + 900u * (127u - (uint32_t)shp) / 127u) * (100u + 35u * (mm - 1u)) / 100u;
                kfl_v[s][p] = 2300;
            } else {
                a = (int32_t)(((int64_t)((vel * 3932) >> 15) * pow_neg(mm, 205)) >> 15);   /* 0.12 x vel / m^0.8 */
                tau = 60u * (100u + 15u * (mm - 1u)) / 100u;
                kfl_v[s][p] = (uint16_t)((uint32_t)shp * 258u * (11469u + 21299u * (mm < 12u ? mm : 12u) / 12u) >> 15);
            }
            ktgt_v[s][p] = knee(a);
            kku_v[s][p] = tau_k(tau);
        }
    }
    kimp_v[s] = !sust;
    if (sust) kheld_v[s] = 1;
    else kseq_v[s]++;
    kflash[s] = fm1_ms ? fm1_ms : 1u;
}
static void strike_release(uint32_t s) { kheld_v[s] = 0; }
static void strike_mode_changed(uint32_t s)
{
    uint32_t h;
    kheld_v[s] = 0;
    for (h = 0; h < NP; h++) { ktgt_v[s][h] = 0; kfl_v[s][h] = 0; }
    kimp_v[s] = P.k[s].mode < KM_BOW;
}

/* ================================================================== the clock and the logic */
/* logic sources: 0 off; 1..11 the clock and its divisions; 12..43 grid A's cells; 44..75 grid B's; 76, 77 pop > half */
#define NLS 78u
static const uint8_t DIVS[11] = {1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 32};
static uint32_t half_n;                         /* half pulses since start */
static uint32_t lsrc(uint32_t id)
{
    if (id == 0u) return 0u;
    if (id <= 11u) return (half_n / DIVS[id - 1u]) % 2u == 0u;
    if (id <= 43u) return cell(0, (int32_t)id - 12);
    if (id <= 75u) return cell(1, (int32_t)id - 44);
    return popc(G[id - 76u].row) > NC / 2u;
}
static void lsrc_name(char *s, uint32_t id);
static uint32_t lop(uint32_t a, uint32_t b, uint32_t op)
{
    switch (op) { case 0: return a & b; case 1: return a | b; case 2: return a ^ b; case 3: return !(a & b); case 4: return !(a | b); default: return !(a ^ b); }
}
static uint32_t kla[2], klb[2], klo[2], kprev[2];
static uint32_t gcount[2], t_half, midi_clk;
static volatile uint32_t dirty_partials = 1;
static void half_tick(void)
{
    uint32_t s, i, rise;
    half_n++;
    rise = !(half_n & 1u);
    if (rise)
        for (i = 0; i < 2u; i++)
            if (++gcount[i] >= P.g[i].div) { gcount[i] = 0; grid_step(i); dirty_partials = 1; }
    for (s = 0; s < 2u; s++) {
        uint32_t o;
        kla[s] = lsrc(P.k[s].a); klb[s] = lsrc(P.k[s].b);
        o = lop(kla[s], klb[s], P.k[s].op);
        if (o && !kprev[s]) strike_fire(s);
        if (!o && kprev[s] && (int32_t)(fm1_ms - kman[s]) > 0) strike_release(s);
        kprev[s] = o; klo[s] = o;
    }
}
static void clock_poll(void)                    /* the clock: internal, or 3 MIDI clocks a half pulse (16ths) */
{
    uint8_t st, d1, d2;
    uint32_t s;
    while (midi_get(&st, &d1, &d2)) {
        if (!P.clk_midi) continue;
        if (st == 0xF8u) { if (++midi_clk >= 3u) { midi_clk = 0; half_tick(); } }
        else if (st == 0xFAu) { midi_clk = 0; half_n = 0xFFFFFFFFu; gcount[0] = gcount[1] = 0; }
    }
    if (!P.clk_midi && fm1_ms - t_half >= Q_TEMPO_HMS[P.tempo]) { t_half += Q_TEMPO_HMS[P.tempo]; if (fm1_ms - t_half > 1000u) t_half = fm1_ms; half_tick(); }
    for (s = 0; s < 2u; s++)                    /* a hand-held Bow / Breath lets go when its time is up and the gate is low */
        if (kman[s] && (int32_t)(fm1_ms - kman[s]) > 0) { kman[s] = 0; if (!klo[s]) strike_release(s); }
}
static uint32_t idle_ok;
static void app_idle(void) { if (idle_ok) clock_poll(); }


/* ================================================================== text */
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
    uint32_t p = 1, k, f;
    char b[6];
    for (k = 0; k < places; k++) p *= 10u;
    s_int(s, (int32_t)(v / div));
    if (!places) return;
    f = (v % div) * p / div;
    s_cat(s, ".");
    for (k = places; k; k--) { b[k - 1u] = (char)('0' + f % 10u); f /= 10u; }
    b[places] = 0;
    s_cat(s, b);
}
static void s_sdec(char *s, int32_t v, uint32_t div, uint32_t places)   /* signed, always with its sign */
{
    s_cat(s, v < 0 ? "-" : "+");
    s_dec(s, (uint32_t)iabs(v), div, places);
}
static void s_note(char *s, uint32_t m)
{
    static const char *const NN[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    s_cat(s, NN[m % 12u]);
    s_int(s, (int32_t)(m / 12u) - 1);
}
static void s_hz(char *s, uint32_t hz)
{
    if (hz >= 1000u) { s_dec(s, hz / 10u, 100u, 1u); s_cat(s, "K"); } else s_int(s, (int32_t)hz);
}
static void lsrc_name(char *s, uint32_t id)
{
    if (id == 0u) { s_cat(s, "OFF"); return; }
    if (id == 1u) { s_cat(s, "CLK"); return; }
    if (id <= 11u) { s_cat(s, "/"); s_int(s, DIVS[id - 1u]); return; }
    if (id <= 43u) { s_cat(s, "A"); s_int(s, (int32_t)id - 11); return; }
    if (id <= 75u) { s_cat(s, "B"); s_int(s, (int32_t)id - 43); return; }
    s_cat(s, id == 76u ? "POP A" : "POP B");
}
static uint32_t isqrt32(uint32_t v) { uint32_t r = 0, b = 1u << 30; while (b > v) b >>= 2; while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; } return r; }

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




/* ---- the battery (as S6) */
#define BATT_EMPTY 515
#define BATT_FULL 615
static int32_t batt_avg = -1;
static uint32_t batt_raw(void)
{
#ifdef DINGHY_HOST
    return 600u;
#else
    int32_t a = fm1_adc_read(FM1_ADC_BATT);
    return a < 0 ? 0u : (uint32_t)a;
#endif
}
static void batt_poll(void)
{
    uint32_t r = batt_raw();
    if (!r) return;
    if (batt_avg < 0) batt_avg = (int32_t)r * 16;
    else batt_avg += ((int32_t)r * 16 - batt_avg) / 8;
}
static int batt_usb(void)
{
#ifdef DINGHY_HOST
    return 0;
#else
    return usb.config && !usb.suspended;
#endif
}
static uint32_t batt_anim;
static uint32_t batt_bars(void)
{
    int32_t b;
    if (batt_usb()) return batt_anim & 3u;
    if (batt_avg < 0) return 3u;
    b = cl(((batt_avg / 16) - BATT_EMPTY) * 100 / (BATT_FULL - BATT_EMPTY), 0, 100);
    return b >= 67 ? 3u : b >= 34 ? 2u : b >= 10 ? 1u : 0u;
}

/* ---- the pages */
enum { PG_INDEX, PG_VOICE, PG_GRID, PG_STRIKE, PG_SPLIT, PG_LOGIC, PG_SEQ, PG_FG, PG_MOD, PG_FX, PG_GLOBAL, PG_PATCH, NPG };
static const char *const PG_NUM[NPG] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "F", "G", "P"};
static const char *const PG_NAME[NPG] = {"INDEX", "VOICE", "GRIDS", "STRIKE", "SPLIT + FILTERS", "LOGIC", "SEQUENCERS",
                                         "FUNCTION GEN", "MOD MATRIX", "FX", "GLOBAL", "PATCHES"};
static const uint8_t PG_READY[NPG] = {1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0};
static uint32_t page = PG_INDEX, sub, bank;     /* sub: OCT's choice (grid, strike, split part); bank: SEL's */
static uint32_t cpu_avg, cpu_peak, cpu_max;     /* 1/10 per cent of the 5.8 ms */
#define BUDGET_US 5805u

static uint32_t page_subs(uint32_t pg) { return pg == PG_GRID || pg == PG_STRIKE ? 2u : pg == PG_SPLIT ? 3u : 1u; }
static uint32_t page_banks(uint32_t pg, uint32_t sb)
{
    if (pg == PG_GRID) return 2u;
    if (pg == PG_STRIKE) return 3u;
    if (pg == PG_SPLIT) return sb ? 2u : 1u;
    if (pg == PG_GLOBAL) return 2u;
    return 1u;
}

/* ---- the knobs: what K1..K4 turn on each page and bank */
enum { F_INT, F_SLEW, F_TILT, F_STR, F_PCT, F_GMODE, F_DIV, F_MUT, F_ONOFF, F_SHIFT, F_KMODE, F_PART, F_VEL, F_SHAPE,
       F_DEC, F_FALL, F_SER, F_LSRC, F_OP, F_ROUTE, F_SRULE, F_POINT, F_WIDTH, F_SOFT, F_PMODE, F_FREQ, F_RES, F_BW,
       F_PAN, F_SEMI, F_CENT, F_CLK, F_TEMPO, F_PHOS, F_BRIGHT, F_BITS, F_NONE };
typedef struct { const char *name; uint8_t *p; int16_t lo, hi; uint8_t fmt, step; } kn_t;
static kn_t KN[4];
static void kn_set(uint32_t i, const char *name, void *p, int32_t lo, int32_t hi, uint32_t fmt, uint32_t step)
{
    KN[i].name = name; KN[i].p = (uint8_t *)p; KN[i].lo = (int16_t)lo; KN[i].hi = (int16_t)hi; KN[i].fmt = (uint8_t)fmt; KN[i].step = (uint8_t)step;
}
static uint8_t dummy;
static void knobs_for(uint32_t pg, uint32_t sb, uint32_t bk)
{
    uint32_t i;
    for (i = 0; i < 4u; i++) kn_set(i, "", &dummy, 0, 0, F_NONE, 1);
    if (pg == PG_VOICE) {
        kn_set(0, "SLEW", &P.slew, 0, 63, F_SLEW, 1); kn_set(1, "TILT", &P.tilt, 0, 250, F_TILT, 5);
        kn_set(2, "STRETCH", &P.stretch, -30, 60, F_STR, 1); kn_set(3, "LEVEL", &P.level, 0, 127, F_PCT, 1);
    } else if (pg == PG_GRID) {
        gpar_t *g = &P.g[sb & 1u];
        if (!bk) { kn_set(0, "RULE", &g->rule, 0, 255, F_INT, 1); kn_set(1, "STEP", &g->div, 1, 16, F_DIV, 1);
                   kn_set(2, "MUTATE", &g->mut, 0, 30, F_MUT, 1); kn_set(3, "SHARE", &P.gmode, 0, NGM - 1, F_GMODE, 1); }
        else { kn_set(0, "RESCUE", &g->rescue, 0, 1, F_ONOFF, 1); kn_set(1, "SEED", &g->dens, 5, 95, F_MUT, 5);
               kn_set(2, "SHIFT", &g->shift, 0, 31, F_SHIFT, 1); }
    } else if (pg == PG_STRIKE) {
        kpar_t *k = &P.k[sb & 1u];
        if (bk == 0u) { kn_set(0, "MODE", &k->mode, 0, NKM - 1, F_KMODE, 1); kn_set(1, "PARTIAL", &k->part, 1, 32, F_PART, 1);
                        kn_set(2, "VEL", &k->vel, 0, 127, F_VEL, 1); kn_set(3, "LEVEL", &k->level, 0, 127, F_PCT, 1); }
        else if (bk == 1u) { kn_set(0, KSH_NAME[k->mode & 3u], &k->shape, 0, 127, F_SHAPE, 1); kn_set(1, "DECAY", &k->decay, 0, 127, F_DEC, 1);
                             kn_set(2, "FALL", &k->fall, 0, 127, F_FALL, 1); kn_set(3, "SERIES", &k->series, 0, 2, F_SER, 1); }
        else { kn_set(0, "INPUT A", &k->a, 0, NLS - 1, F_LSRC, 1); kn_set(1, "LOGIC", &k->op, 0, 5, F_OP, 1);
               kn_set(2, "INPUT B", &k->b, 0, NLS - 1, F_LSRC, 1); kn_set(3, "ROUTE", &P.route_pre, 0, 1, F_ROUTE, 1); }
    } else if (pg == PG_SPLIT) {
        if (sb == 0u) { kn_set(0, "RULE", &P.srule, 0, NSR - 1, F_SRULE, 1); kn_set(1, "POINT", &P.spoint, 0, 127, F_POINT, 1);
                        kn_set(2, "WIDTH", &P.swidth, 2, 50, F_WIDTH, 1); kn_set(3, "SOFT", &P.ssoft, 0, 36, F_SOFT, 1); }
        else {
            ppar_t *q = &P.pth[sb - 1u];
            if (!bk) { kn_set(0, "FILTER", &q->mode, 0, 3, F_PMODE, 1); kn_set(1, q->mode == 1u ? "VOWEL" : "FREQ", &q->freq, 0, 127, F_FREQ, 4);
                       kn_set(2, "RESO", &q->res, 0, 127, F_RES, 4); kn_set(3, "BW", &q->bw, 0, 40, F_BW, 2); }
            else { kn_set(0, "LEVEL", &q->level, 0, 127, F_PCT, 1); kn_set(1, "PAN", &q->pan, 0, 127, F_PAN, 1);
                   kn_set(2, "PITCH", &q->coarse, -24, 24, F_SEMI, 1); kn_set(3, "FINE", &q->fine, -50, 50, F_CENT, 1); }
        }
    } else if (pg == PG_GLOBAL) {
        if (!bk) { kn_set(0, "CLOCK", &P.clk_midi, 0, 1, F_CLK, 1); kn_set(1, "TEMPO", &P.tempo, 0, 127, F_TEMPO, 1);
                   kn_set(2, "ENGINE", &P.bits, 0, 2, F_BITS, 1); }
        else { kn_set(0, "PHOS", &gset.phos, 0, NPHOS - 1, F_PHOS, 1); kn_set(1, "BRIGHT", &gset.bright, 0, 10, F_BRIGHT, 1); }
    }
}
static int32_t kn_get(const kn_t *k) { return k->lo < 0 ? (int32_t)*(int8_t *)k->p : (int32_t)*k->p; }
static void kn_put(const kn_t *k, int32_t v)
{
    v = cl(v, k->lo, k->hi);
    if (k->lo < 0) *(int8_t *)k->p = (int8_t)v; else *k->p = (uint8_t)v;
}
static void kn_text(char *s, const kn_t *k, uint32_t pathk)
{
    int32_t v = kn_get(k);
    s[0] = 0;
    switch (k->fmt) {
    case F_INT: s_int(s, v); break;
    case F_SLEW: s_dec(s, CA_SLEW_MS[v], 1000u, CA_SLEW_MS[v] < 10000u ? 2u : 1u); s_cat(s, "S"); break;
    case F_TILT: s_dec(s, (uint32_t)v, 100u, 2u); break;
    case F_STR: s_sdec(s, v, 1000u, 3u); break;
    case F_PCT: s_int(s, v * 100 / 127); s_cat(s, "%"); break;
    case F_GMODE: s_cat(s, GM_NAME[v % NGM]); break;
    case F_DIV: s_cat(s, "/"); s_int(s, v); break;
    case F_MUT: s_int(s, v); s_cat(s, "%"); break;
    case F_ONOFF: s_cat(s, v ? "ON" : "OFF"); break;
    case F_SHIFT: s_sdec(s, v, 1u, 0u); break;
    case F_KMODE: s_cat(s, KM_NAME[v & 3]); break;
    case F_PART: s_cat(s, "H"); s_int(s, v); break;
    case F_VEL: case F_SHAPE: case F_FALL: s_dec(s, (uint32_t)v * 100u / 127u, 100u, 2u); break;
    case F_DEC: s_dec(s, Q_DEC_MS[v] / 10u, 100u, Q_DEC_MS[v] < 10000u ? 2u : 1u); s_cat(s, "S"); break;
    case F_SER: s_cat(s, SE_NAME[v % 3]); break;
    case F_LSRC: lsrc_name(s, (uint32_t)v); break;
    case F_OP: s_cat(s, OP_NAME[v % 6]); break;
    case F_ROUTE: s_cat(s, v ? "PRE" : "POST"); break;
    case F_SRULE: s_cat(s, SR_NAME[v % NSR]); break;
    case F_POINT: s_hz(s, q_hz(8858 + v * (30720 - 8858) / 127)); break;
    case F_WIDTH: s_dec(s, (uint32_t)v, 10u, 1u); s_cat(s, "OCT"); break;
    case F_SOFT: if (!v) s_cat(s, "HARD"); else { s_int(s, v); s_cat(s, "ST"); } break;
    case F_PMODE: s_cat(s, PM_NAME[v & 3]); break;
    case F_FREQ:
        if (P.pth[pathk].mode == 1u) {
            static const char *const VW[5] = {"A", "E", "I", "O", "U"};
            uint32_t x = (uint32_t)v * 4u * 256u / 127u, i = x >> 8, t = x & 255u;
            if (i >= 4u) s_cat(s, "U");
            else { s_cat(s, VW[i]); if (t > 20u && t < 236u) { s_cat(s, ">"); s_cat(s, VW[i + 1u]); } else if (t >= 236u) { s[0] = 0; s_cat(s, VW[i + 1u]); } }
        } else s_hz(s, Q_FHZ[v]);
        break;
    case F_RES: s_dec(s, Q_RES_X10[v], 10u, 1u); break;
    case F_BW: s_dec(s, (uint32_t)v * 5u, 100u, 2u); break;
    case F_PAN: if (v == 64) s_cat(s, "C"); else { s_cat(s, v < 64 ? "L" : "R"); s_int(s, iabs(v - 64) * 100 / 63); } break;
    case F_SEMI: s_sdec(s, v, 1u, 0u); s_cat(s, "ST"); break;
    case F_CENT: s_sdec(s, v, 1u, 0u); s_cat(s, "C"); break;
    case F_CLK: s_cat(s, v ? "MIDI" : "INT"); break;
    case F_TEMPO: s_dec(s, Q_TEMPO_X100[v], 100u, 2u); break;
    case F_PHOS: { static const char *const PN[NPHOS] = {"GREEN", "AQUA", "AMBER", "WHITE"}; s_cat(s, PN[v % NPHOS]); } break;
    case F_BRIGHT: s_int(s, v); break;
    case F_BITS: { static const char *const BN[3] = {"SMOOTH", "12 BIT", "8 BIT"}; s_cat(s, BN[v % 3]); } break;
    default: break;
    }
}

static const char *hmsg = "";
static void draw_header(void)
{
    char s[40] = "PAGE ";
    s_cat(s, PG_NUM[page]);
    s_cat(s, " ");
    if (page == PG_GRID) s_cat(s, sub ? "GRID B" : "GRID A");
    else if (page == PG_STRIKE) s_cat(s, sub ? "STRIKE 2" : "STRIKE 1");
    else if (page == PG_SPLIT) s_cat(s, sub == 0u ? "SPLIT" : sub == 1u ? "PATH A" : "PATH B");
    else s_cat(s, PG_NAME[page]);
    if (page_banks(page, sub) > 1u) { s_cat(s, "  BANK "); s_int(s, (int32_t)bank + 1); }
    if (*hmsg) { s[0] = 0; s_cat(s, hmsg); }
    cv_begin(240, BODY_Y, PH_BG);
    cv_text(4, 2, &FONT_S, s, PH_HI);
    {
        uint32_t n = batt_bars(), k;
        uint16_t c = !batt_usb() && n == 0u ? PH_HI : PH_MID;
        cv_rect(208, 4, 22, 1, c); cv_rect(208, 14, 22, 1, c);
        cv_rect(208, 4, 1, 11, c); cv_rect(229, 4, 1, 11, c);
        cv_rect(230, 7, 2, 5, c);
        for (k = 0; k < n; k++) cv_rect(211 + (int32_t)k * 6, 7, 5, 5, PH_HI);
    }
    cv_rect(0, 19, 240, 1, PH_MID);
    cv_out(0);
}

static void knob_row(uint32_t pathk)            /* K1..K4 along the bottom: the name, the value */
{
    uint32_t i;
    char s[16];
    cv_rect(0, 161, 240, 1, PH_LO);
    for (i = 0; i < 4u; i++) {
        int32_t x = 2 + (int32_t)i * 60;
        if (KN[i].fmt == F_NONE) continue;
        cv_text(x, 164, &FONT_S, KN[i].name, PH_LO);
        kn_text(s, &KN[i], pathk);
        cv_text(x, 181, &FONT_S, s, PH_HI);
    }
}
static void footer(const char *t) { cv_text(2, 201, &FONT_S, t, PH_LO); }

static void draw_index(uint32_t cur)
{
    uint32_t i;
    cv_begin(240, BODY_H, PH_BG);
    for (i = 0; i < NPG; i++) {
        int32_t y = 2 + (int32_t)i * 16;
        char s[32] = "";
        uint16_t c = PG_READY[i] ? PH_MID : PH_LO;
        if (i == 0u) continue;
        y -= 16;
        if (i == cur) { cv_rect(0, y - 1, 240, 16, PH_SEL); c = PH_HI; cv_text(4, y, &FONT_S, ">", PH_HI); }
        s_cat(s, PG_NUM[i]); s_cat(s, "   "); s_cat(s, PG_NAME[i]);
        cv_text(20, y, &FONT_S, s, c);
        if (i == cur && (fm1_ms / 400u) & 1u) cv_rect(20 + text_w(&FONT_S, s) + 4, y + 1, 8, 14, PH_HI);
        if (!PG_READY[i]) cv_text(196, y, &FONT_S, "--", PH_LO);
    }
    cv_text(4, 196, &FONT_S, "SELECT: CHOOSE  SEL: OPEN", PH_LO);
    cv_out(BODY_Y);
}
static void draw_soon(void)
{
    cv_begin(240, BODY_H, PH_BG);
    cv_text(4, 60, &FONT_S, PG_NAME[page], PH_MID);
    cv_text(4, 84, &FONT_S, "NEXT BUILD (STAGE 2)", PH_LO);
    cv_text(4, 196, &FONT_S, "HOME: INDEX", PH_LO);
    cv_out(BODY_Y);
}

/* the partials as bars: 64 x 3 px from x = 24, path A solid, path B as every other line */
static void bars(int32_t y0, int32_t hgt, uint32_t which)   /* which: 0 both, 1 A only, 2 B only */
{
    uint32_t h;
    int32_t tot[NP], ta[NP], mx = 1;
    for (h = 0; h < NP; h++) {
        int32_t g = gshow[h], k0 = kshow_v[0][h], k1 = kshow_v[1][h], wa = wsplit[h], ka0, ka1;
        int32_t gw = P.srule == SR_SRC ? ONE : wa;
        ka0 = !P.route_pre ? ONE : P.srule == SR_SRC ? 0 : wa;
        ka1 = !P.route_pre ? 0 : P.srule == SR_SRC ? 0 : wa;
        tot[h] = g + k0 * P.k[0].level / 127 + k1 * P.k[1].level / 127;
        ta[h] = ((g * gw) >> 15) + ((k0 * P.k[0].level / 127 * ka0) >> 15) + ((k1 * P.k[1].level / 127 * ka1) >> 15);
        if (which == 1u) tot[h] = ta[h];
        if (which == 2u) { tot[h] -= ta[h]; ta[h] = 0; }
        if (tot[h] > mx) mx = tot[h];
    }
    for (h = 0; h < NP; h++) {
        int32_t x = 24 + (int32_t)h * 3, ht, ha, y;
        if (tot[h] <= 0) { cv_rect(x, y0 + hgt - 1, 2, 1, PH_LO); continue; }
        ht = (int32_t)isqrt32((uint32_t)(tot[h] * 1000 / mx) * (uint32_t)(hgt * hgt) / 1000u);
        ha = ht * ta[h] / tot[h];
        if (ha > 0) cv_rect(x, y0 + hgt - ha, 2, ha, PH_HI);
        for (y = y0 + hgt - ht; y < y0 + hgt - ha; y += 2) cv_rect(x, y, 2, 1, PH_MID);
    }
}

static void draw_voice(void)
{
    char s[40];
    cv_begin(240, BODY_H, PH_BG);
    bars(4, 116, 0);
    cv_text(4, 4, &FONT_S, "H1", PH_LO);
    cv_text(236 - 24, 4, &FONT_S, "H64", PH_LO);
    s[0] = 0; s_cat(s, "ROOT "); s_note(s, P.root); s_cat(s, P.anchor ? "  F0 ON" : "  F0 OFF");
    s_cat(s, "  CLK "); if (P.clk_midi) s_cat(s, "MIDI"); else s_dec(s, Q_TEMPO_X100[P.tempo], 100u, 1u);
    cv_text(4, 126, &FONT_S, s, PH_MID);
    s[0] = 0; s_cat(s, "LIVE "); s_int(s, (int32_t)live_v); s_cat(s, "  CPU "); s_dec(s, cpu_avg, 10u, 1u); s_cat(s, "%");
    s_cat(s, "  A/B");
    cv_text(4, 142, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("ALG ROOT  PRE CLOCK  SEL F0");
    cv_out(BODY_Y);
}

static void draw_grid(void)
{
    const grid_t *g = &G[sub & 1u];
    uint32_t y, c;
    char s[40];
    cv_begin(240, BODY_H, PH_BG);
    for (y = 0; y < GHIST; y++) {               /* oldest at the top */
        uint32_t r = g->hist[(g->head + y) % GHIST], age = GHIST - 1u - y;
        uint16_t col = age == 0u ? PH_HI : age < 10u ? PH_MID : PH_LO;
        for (c = 0; c < NC; c++)
            if ((r >> c) & 1u) cv_rect(8 + (int32_t)c * 7, 2 + (int32_t)y * 5, 6, 4, col);
    }
    s[0] = 0; s_cat(s, sub ? "B" : "A"); s_cat(s, " RULE "); s_int(s, P.g[sub & 1u].rule);
    s_cat(s, "  POP "); s_int(s, (int32_t)popc(g->row)); s_cat(s, "/32");
    cv_text(4, 146, &FONT_S, s, PH_MID);
    if (g->rescue_t && fm1_ms - g->rescue_t < 900u) { s[0] = 0; s_cat(s, "RESCUE "); s_cat(s, g->why); cv_text(236 - text_w(&FONT_S, s), 146, &FONT_S, s, PH_HI); }
    knob_row(0);
    footer("OCT A/B  SEL BANK  PLAY SEED");
    cv_out(BODY_Y);
}

static void draw_strike(void)
{
    uint32_t s_ = sub & 1u, h;
    const kpar_t *k = &P.k[s_];
    int32_t mx = 1;
    char s[40];
    cv_begin(240, BODY_H, PH_BG);
    for (h = 0; h < NP; h++) if (kshow_v[s_][h] > mx) mx = kshow_v[s_][h];
    if (mx < 2000) mx = 2000;
    for (h = 0; h < NP; h++) {                  /* this strike's partials, ringing now; its members marked below */
        int32_t x = 24 + (int32_t)h * 3, v = kshow_v[s_][h], ht = (int32_t)isqrt32((uint32_t)(v * 1000 / mx) * 8100u / 1000u);
        if (ht > 0) cv_rect(x, 94 - ht, 2, ht, PH_HI);
        cv_rect(x, 98, 2, 2, kmember[s_][h] ? PH_MID : PH_SEL);
    }
    cv_text(4, 4, &FONT_S, KM_NAME[k->mode & 3u], PH_MID);
    s[0] = 0; s_cat(s, "H"); s_int(s, k->part); cv_text(236 - text_w(&FONT_S, s), 4, &FONT_S, s, PH_MID);
    {                                           /* the logic: A, B, the gate */
        int32_t x = 4;
        const char *L[3] = {"A ", "B ", "GATE"};
        uint32_t on[3], i;
        on[0] = kla[s_]; on[1] = klb[s_]; on[2] = klo[s_] || (kflash[s_] && fm1_ms - kflash[s_] < 120u);
        for (i = 0; i < 3u; i++) {
            if (on[i]) cv_rect(x, 110, 10, 10, PH_HI); else { cv_rect(x, 110, 10, 1, PH_MID); cv_rect(x, 119, 10, 1, PH_MID); cv_rect(x, 110, 1, 10, PH_MID); cv_rect(x + 9, 110, 1, 10, PH_MID); }
            x += 14; x = cv_text(x, 107, &FONT_S, L[i], on[i] ? PH_HI : PH_LO) + 8;
        }
        s[0] = 0; lsrc_name(s, k->a); s_cat(s, " "); s_cat(s, OP_NAME[k->op % 6u]); s_cat(s, " "); lsrc_name(s, k->b);
        cv_text(4, 128, &FONT_S, s, PH_MID);
        s[0] = 0; s_cat(s, P.route_pre ? "PRE-SPLIT" : s_ ? "POST > B" : "POST > A"); s_cat(s, k->filt ? " FILT" : " SUM");
        cv_text(236 - text_w(&FONT_S, s), 128, &FONT_S, s, PH_LO);
    }
    knob_row(0);
    footer("OCT 1/2 SEL BANK PLAY FIRE");
    cv_out(BODY_Y);
}

static void draw_split(void)
{
    char s[40];
    cv_begin(240, BODY_H, PH_BG);
    if (sub == 0u) {
        int32_t qc = 8858 + (int32_t)P.spoint * (30720 - 8858) / 127, h, x;
        bars(4, 116, 0);
        if (P.srule == SR_CUT || P.srule == SR_BAND) {   /* where the split point falls among the partials */
            for (h = 0; h < (int32_t)NP - 1 && pq_base[h + 1] < qc; h++) {}
            x = 24 + h * 3 + 2;
            for (h = 4; h < 120; h += 4) cv_rect(x, h, 1, 2, PH_HI);
        }
        s[0] = 0; s_cat(s, SR_NAME[P.srule % NSR]); if (P.srule == SR_NTH) { s_cat(s, " N="); s_int(s, P.sn); }
        cv_text(4, 126, &FONT_S, s, PH_MID);
        cv_text(4, 142, &FONT_S, "SOLID: PATH A  LINES: PATH B", PH_LO);
        knob_row(0);
        footer("OCT PART  ALG N");
    } else {
        uint32_t k = sub - 1u, h;
        const ppar_t *q = &P.pth[k];
        bars(4, 116, k + 1u);
        if (q->mode == 1u)                      /* the vowel's shape over the partials */
            for (h = 0; h < NP; h++) cv_rect(24 + (int32_t)h * 3, 120 - vow_show[k][h] * 110 / 39321, 2, 2, PH_MID);
        else if (q->mode >= 2u) {               /* the band pass, roughly: 1 / (1 + (Q x octaves)^2) */
            int32_t qf = (int32_t)(Q_FHZ[q->freq]), qx = Q_RES_X10[q->res];
            for (h = 0; h < NP; h++) {
                int32_t hz = (int32_t)q_hz(pq_now[h]), d, r;
                d = hz > qf ? (hz - qf) * 256 / qf : (qf - hz) * 256 / (hz + 1);   /* ~ distance, Q8 */
                d = d * qx / 10 / (q->mode == 3u ? 1 : 2);
                r = 65536 / (256 + (d * d >> 8));
                cv_rect(24 + (int32_t)h * 3, 120 - r * 110 / 256, 2, 2, PH_MID);
            }
        }
        s[0] = 0; s_cat(s, PM_NAME[q->mode & 3u]); s_cat(s, "  PITCH "); s_sdec(s, q->coarse, 1u, 0u); s_cat(s, "ST ");
        s_sdec(s, q->fine, 1u, 0u); s_cat(s, "C");
        cv_text(4, 126, &FONT_S, s, PH_MID);
        knob_row(k);
        footer("OCT PART  SEL BANK");
    }
    cv_out(BODY_Y);
}

static void draw_global(void)
{
    char s[40];
    int32_t wa, wp;
    cv_begin(240, BODY_H, PH_BG);
    cv_text(4, 4, &FONT_S, "AUDIO INTERRUPT", PH_MID);
    s[0] = 0; s_cat(s, "AVG "); s_dec(s, cpu_avg, 10u, 1u); s_cat(s, "%  PEAK "); s_dec(s, cpu_peak, 10u, 1u); s_cat(s, "%");
    cv_text(4, 26, &FONT_S, s, PH_HI);
    s[0] = 0; s_cat(s, "MAX "); s_dec(s, cpu_max, 10u, 1u); s_cat(s, "%  LIVE "); s_int(s, (int32_t)live_v); s_cat(s, "/64");
    cv_text(4, 44, &FONT_S, s, PH_MID);
    wa = (int32_t)(cpu_avg > 1000u ? 1000u : cpu_avg) * 232 / 1000;
    wp = (int32_t)(cpu_peak > 1000u ? 1000u : cpu_peak) * 232 / 1000;
    cv_rect(4, 70, 232, 1, PH_LO);
    cv_rect(4, 66, wa, 8, PH_MID);
    cv_rect(4 + wp, 62, 2, 16, PH_HI);
    cv_rect(120, 62, 1, 16, PH_LO);
    cv_text(4, 84, &FONT_S, "THE LINE: HALF THE BUDGET", PH_LO);
    s[0] = 0; s_cat(s, "ENGINE "); s_cat(s, P.bits == 2u ? "8 BIT: 256 LEVELS" : P.bits == 1u ? "12 BIT: 4096 LEVELS" : "SMOOTH");
    cv_text(4, 108, &FONT_S, s, PH_MID);
    s[0] = 0; s_cat(s, "QUORUM "); s_cat(s, Q_VERSION); s_cat(s, " (STAGE 1)");
    cv_text(4, 130, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("SEL BANK  PLAY: RESET MAX");
    cv_out(BODY_Y);
}

/* ---- the boot screen: a petri dish, colonies growing */
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
static void circle(int32_t cx, int32_t cy, int32_t r, uint16_t c)
{
    int32_t x = r, y = 0, e = 1 - r;
    while (x >= y) {
        cv_pset(cx + x, cy + y, c); cv_pset(cx - x, cy + y, c); cv_pset(cx + x, cy - y, c); cv_pset(cx - x, cy - y, c);
        cv_pset(cx + y, cy + x, c); cv_pset(cx - y, cy + x, c); cv_pset(cx + y, cy - x, c); cv_pset(cx - y, cy - x, c);
        y++;
        if (e < 0) e += 2 * y + 1; else { x--; e += 2 * (y - x) + 1; }
    }
}
static void disc(int32_t cx, int32_t cy, int32_t r, uint16_t c)
{
    int32_t y;
    for (y = -r; y <= r; y++) { int32_t w = (int32_t)isqrt32((uint32_t)(r * r - y * y)); cv_rect(cx - w, cy + y, 2 * w + 1, 1, c); }
}
static void draw_boot(uint32_t step)            /* step 0..6: the colonies grow, then the name */
{
    const int32_t cx = 120, cy = 62, R = 56;
    uint32_t seed = 0x51C0FFEEu, i, n = 6u + step * 7u;
    cv_begin(240, BODY_Y, PH_BG);
    cv_out(0);
    cv_begin(240, BODY_H, PH_BG);
    circle(cx, cy, R, PH_MID);                  /* the dish, its rim */
    circle(cx, cy, R - 3, PH_LO);
    for (i = 0; i < n && i < 48u; i++) {        /* the bacteria: rods, cocci in clusters, a few colonies */
        int32_t x, y, kind, d2;
        seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
        x = (int32_t)(seed % 96u) - 48; y = (int32_t)((seed >> 8) % 96u) - 48; kind = (int32_t)((seed >> 16) % 7u);
        d2 = x * x + y * y;
        if (d2 > 44 * 44) { x = x * 3 / 4; y = y * 3 / 4; }
        x += cx; y += cy;
        if (kind < 3) {                         /* a rod: a capsule, one of four angles */
            int32_t a = (int32_t)((seed >> 20) & 3u), k;
            for (k = -3; k <= 3; k++) {
                int32_t px = a == 0 ? x + k : a == 1 ? x : x + k, py = a == 0 ? y : a == 1 ? y + k : a == 2 ? y + k : y - k;
                cv_rect(px, py, 2, 2, PH_HI);
            }
        } else if (kind < 6) {                  /* cocci: a little cluster */
            cv_rect(x, y, 2, 2, PH_MID); cv_rect(x + 3, y + 1, 2, 2, PH_MID); cv_rect(x + 1, y + 3, 2, 2, PH_MID);
            if (kind == 5) cv_rect(x + 4, y + 4, 2, 2, PH_MID);
        } else if (step > 2u) {                 /* a colony: a disc in a halo, once there are enough of them */
            circle(x, y, 7, PH_LO);
            disc(x, y, 4, PH_HI);
        }
    }
    if (step >= 3u) {
        big_text(48, 124, "QUORUM", 3, 3, PH_HI);
        boot_line(152, "Cellular Synthesis System", PH_MID);
        boot_line(168, "by zeptosecond DSP  " Q_VERSION, PH_MID);
        boot_line(186, "Written by machine", PH_LO);
        boot_line(200, "with human guidance", PH_LO);
    }
    cv_out(BODY_Y);
}
static int boot_wait(uint32_t ms)
{
    uint32_t t0 = fm1_ms, rel;
    while (fm1_ms - t0 < ms) {
        platform_poll();
        if (fm1_input_edges(&rel)) return 1;
    }
    return 0;
}

/* ================================================================== the main loop */
static void after_change(uint32_t pg, uint32_t sb, uint32_t knob, uint32_t bk)
{
    dirty_partials = 1;
    if (pg == PG_STRIKE && bk == 0u && knob == 0u) strike_mode_changed(sb & 1u);
    if (pg == PG_GLOBAL && bk == 1u) crt_build();
    if (pg == PG_GLOBAL && bk == 0u && knob == 1u) t_half = fm1_ms;
    if (pg == PG_GLOBAL && bk == 0u && knob == 2u) cpu_max = 0;   /* (a new engine: a new worst case) */
}
static void app_main(void)
{
    uint32_t i, b, rel, icur = PG_VOICE, t_draw = 0, t_cpu = 0, t_batt = 0, t_push = 0, prev_keys = 0, hdr = 1;
    int32_t e;
    int skip;

    for (i = 0; i < (1u << SIN_BITS); i++) SINT[i] = (int16_t)sine(i << (32u - SIN_BITS));
    crt_init();
    batt_poll();
    draw_boot(0);
#ifndef DINGHY_HOST
    rnd_s ^= fm1_ticks() * 0xC2B2AE35u;
    if (!rnd_s) rnd_s = 1u;
#endif
    for (i = 0; i < 2u; i++) { grid_seed(i); for (b = 0; b < GHIST; b++) grid_step(i); G[i].rescue_t = 0; }
    kimp_v[0] = P.k[0].mode < KM_BOW; kimp_v[1] = P.k[1].mode < KM_BOW;
    skip = boot_wait(250);
    for (i = 1; i <= 6u && !skip; i++) { draw_boot(i); skip = boot_wait(i == 6u ? 900u : 250u); }
    push_partials();
    t_half = fm1_ms;
    idle_ok = 1;

    for (;;) {
        uint32_t keys, down, nb, nsub;
        int shift;
        platform_poll();
        clock_poll();

        b = fm1_input_edges(&rel);
        shift = (fm1_in.buttons >> BTN_SEL) & 1u;
        {                                       /* page buttons, as Arcturus */
            uint32_t np = page;
            if (b & (1u << BTN_HOME)) { if (page != PG_INDEX) icur = page; np = PG_INDEX; }
            if (b & (1u << BTN_EDIT)) np = shift ? PG_SPLIT : PG_VOICE;
            if (b & (1u << BTN_LFO)) np = PG_GRID;
            if (b & (1u << BTN_ENV)) np = shift ? PG_FG : PG_STRIKE;
            if (b & (1u << BTN_SEQ)) np = PG_SEQ;
            if (b & (1u << BTN_ARP)) np = PG_MOD;
            if (b & (1u << BTN_FX)) np = PG_FX;
            if (b & (1u << BTN_GLO)) np = PG_GLOBAL;
            if (b & (1u << BTN_SAVE)) np = PG_PATCH;
            if (np != page) { page = np; sub = 0; bank = 0; hdr = 1; b &= ~(1u << BTN_SEL); }
        }
        nsub = page_subs(page);
        if (nsub > 1u) {                        /* OCT: the next / previous thing on this page */
            if (b & (1u << BTN_OCT_UP)) { sub = (sub + 1u) % nsub; bank = 0; hdr = 1; }
            if (b & (1u << BTN_OCT_DOWN)) { sub = (sub + nsub - 1u) % nsub; bank = 0; hdr = 1; }
        }
        nb = page_banks(page, sub);
        if (page == PG_INDEX) {
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) icur = (uint32_t)cl((int32_t)icur + e, 1, NPG - 1);
            if (b & (1u << BTN_SEL)) { page = icur; sub = 0; bank = 0; hdr = 1; }
        } else if (b & (1u << BTN_SEL)) {
            if (nb > 1u) { bank = (bank + 1u) % nb; hdr = 1; }
            else if (page == PG_VOICE) { P.anchor ^= 1u; dirty_partials = 1; }

        }
        if (b & (1u << BTN_PLAY)) {
            if (page == PG_GRID) { grid_seed(sub & 1u); dirty_partials = 1; }
            if (page == PG_GLOBAL) cpu_max = 0;
            if (page == PG_STRIKE) {
                uint32_t s_ = sub & 1u;
                strike_fire(s_);
                if (P.k[s_].mode >= KM_BOW) kman[s_] = fm1_ms + 1200u;
            }
        }
        knobs_for(page, sub, bank);
        for (i = 0; i < 4u; i++)
            if ((e = fm1_enc_take(ENC_KNOB1 + i)) != 0 && KN[i].fmt != F_NONE) {
                kn_put(&KN[i], kn_get(&KN[i]) + e * KN[i].step);
                after_change(page, sub, i, bank);
                knobs_for(page, sub, bank);     /* (a mode's change renames a knob) */
            }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {
            if (page == PG_VOICE) P.root = (uint8_t)cl((int32_t)P.root + e, 21, 72);
            if (page == PG_STRIKE) P.k[sub & 1u].filt = e > 0;
            if (page == PG_SPLIT) P.sn = (uint8_t)cl((int32_t)P.sn + e, 2, 8);
            dirty_partials = 1;
        }
        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) {
            if (page == PG_VOICE) { P.tempo = (uint8_t)cl((int32_t)P.tempo + e, 0, 127); }
            if (page == PG_STRIKE) { P.route_pre = e > 0; dirty_partials = 1; }
        }
        if (page != PG_INDEX) (void)fm1_enc_take(ENC_SELECT);
        keys = fm1_in.notes;
        down = keys & ~prev_keys;
        prev_keys = keys;
        if (down) {                             /* the lowest key pressed: the root, F1 .. G3 */
            for (i = 0; i < 27u; i++) if ((down >> i) & 1u) break;
            P.root = (uint8_t)(29u + i);
            dirty_partials = 1;
        }

        if (dirty_partials && fm1_ms - t_push >= 8u) { dirty_partials = 0; t_push = fm1_ms; push_partials(); }
        if (fm1_ms - t_cpu >= 500u) {
            uint32_t n = us_n, sum = us_sum, pk = us_peak;
            t_cpu = fm1_ms;
            us_sum = 0; us_n = 0; us_peak = 0;
            if (n) {
                cpu_avg = sum / n * 1000u / BUDGET_US;
                cpu_peak = pk * 1000u / BUDGET_US;
                if (cpu_peak > cpu_max) cpu_max = cpu_peak;
            }
        }
        if (fm1_ms - t_batt >= 2000u) { t_batt = fm1_ms; batt_poll(); batt_anim++; hdr = 1; }
        if (fm1_ms - t_draw >= 66u) {
            uint32_t blink = (fm1_ms / 250u) & 1u, row;
            t_draw = fm1_ms;
            if (hdr) { hdr = 0; draw_header(); }
            switch (page) {
            case PG_INDEX: draw_index(icur); break;
            case PG_VOICE: draw_voice(); break;
            case PG_GRID: draw_grid(); break;
            case PG_STRIKE: draw_strike(); break;
            case PG_SPLIT: draw_split(); break;
            case PG_GLOBAL: draw_global(); break;
            default: draw_soon(); break;
            }
            row = G[page == PG_GRID ? sub & 1u : 0u].row;   /* the keys: the cells (grid A, or the grid shown) */
            for (i = 0; i < 27u; i++) fm1_led_key(KEY_LED(i), (row >> i) & 1u);
            fm1_led_key(BTN_SEL, shift || (page == PG_INDEX && blink) || (page_banks(page, sub) > 1u && blink));
            fm1_led_key(BTN_OCT_DOWN, page_subs(page) > 1u && blink);
            fm1_led_key(BTN_OCT_UP, page_subs(page) > 1u && !blink);
            fm1_led_key(BTN_HOME, page != PG_INDEX && blink);
            fm1_led_key(BTN_PLAY, (page == PG_STRIKE && (klo[sub & 1u] || blink)) || ((page == PG_GRID || page == PG_GLOBAL) && blink));
        }
    }
}
