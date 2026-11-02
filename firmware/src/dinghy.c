/* SPDX-License-Identifier: GPL-3.0-only */
/* QUORUM Cellular Synthesis System: build 0.4 (the engine, its logic, its modulation, its sequencers). The sister of Arcturus S6 (the
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
 *   ENV    4 STRIKE     OCT: strike 1 / 2; SEL: SOUND, ENVELOPE, TRILL; ALGORITHM sum / filter; PRESETS routing; PLAY fires
 *   SEL+EDIT 5 SPLIT    OCT: SPLIT / PATH A / PATH B; K1-K4, SEL: two banks on the paths; ALGORITHM N
 *   SEL+LFO 6 LOGIC     OCT: generator 1-4: an input AND/OR/... a square LFO (free or clocked); ALGORITHM free / clock
 *   SEL+ENV 8 FUNC GEN  OCT: FG 1-4, S&H 1-2 (their gates come from the gate matrix), the quadrature LFO
 *   ARP    9 MOD MATRIX K1 / SELECT the source, K2 / PRESETS the destination, K3 the depth, PLAY a pin in / out
 *   SEQ    7 SEQUENCERS OCT: 1-3; bank 1 STEP (cursor) VALUE GATE LENGTH, bank 2 DIR SLEW ADDRESS (CLOCK, or a matrix
 *                       source sweeping the steps, Serge style) CHAIN; PLAY: the step's gate
 *   SEL+ARP X GATE MATRIX  24 gate sources x 16 destinations (the strikes, FG triggers, S&H clocks, grid steps,
 *                       the sequencers' advance and reset), OR'd
 *   GLO    G GLOBAL     OCT: TUNING (SELECT a path, PLAY quantise, K4 glide), VDU, ENGINE (version, clock, CPU)
 *   the keys: the root (F1 .. F3); their LEDs show the cells. The top two keys (lit): strike 1 and 2 by hand, anywhere
 *
 * Rules of the road (the chip has no FPU and no libgcc): integers only, no float, no 64-bit division.
 * tests/host.c runs this file on a computer: the screen as a PNG, the sound as a WAV. */
#define APP_NAME "QUORUM"
#define Q_VERSION "0.4"
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
#include "tunings.h"

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
static volatile uint16_t katt_v[2][NP];         /* a kick's attack, blocks */
static int32_t kvel_hold[2];                    /* Bow / Breath: the velocity of the gate that is held */
static volatile uint32_t kku_v[2][NP], kkd_v[2][NP];   /* the steps up (Bow / Breath) and down, a block, Q16 */
static volatile uint16_t kfl_v[2][NP];          /* the flicker: Bow's flutter, Breath's air (Q15 depth) */
static volatile uint32_t kimp_v[2], kheld_v[2];   /* Strike / Mallet (1) or Bow / Breath; Bow / Breath's gate */
static uint32_t frnd = 0x2545F491u;

/* the paths: mode (0 off, 1 vowel, 2 band pass 12 dB, 3 band pass 24 dB), the SVFs' f and damping (Q14), makeup
 * (Q12), the gains to left and right with the level in them (Q15) */
static volatile uint8_t pm_v[2];
static volatile int32_t pf_v[2][2], pq_v[2], pmk_v[2], pgl_v[2], pgr_v[2];
typedef struct { int32_t lo, bp; } svf_t;
static svf_t svf[2][2];

static volatile uint32_t us_sum, us_n, us_peak; /* the interrupt's time, for the CPU line */
static volatile uint32_t bits_v = 1;            /* the engine's word: 1 12 bit (ver 2), 2 8 bit (ver 1); (0 smooth: unused) */
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

/* ================================================================== the patch */
enum { SR_CUT, SR_BAND, SR_ODD, SR_NTH, SR_GRID, SR_SRC, NSR };
enum { GM_LOWHIGH, GM_ODDEVEN, GM_AND, GM_OR, GM_XOR, NGM };
enum { KM_STRIKE, KM_MALLET, KM_BOW, KM_BREATH, NKM };
enum { SE_ALL, SE_ODD, SE_EVEN };
enum { TP_RISE, TP_FALL, TP_HUMP };
enum { CV_LOG, CV_LIN, CV_EXP };
enum { FM_CYCLE, FM_ONCE, FM_GATE };
#define NGEN 4u                                 /* logic generators */
#define NFG 4u                                  /* function generators */
#define NSSG 2u                                 /* sample-and-holds */
/* the matrix's sources: FG 1-4 (0..+), SSG 1-2 (+-), each grid's population (0..+), the generators (0 / +) */
#define S_FG(k) (k)
#define S_SSG(k) (NFG + (k))
#define S_POP(i) (NFG + NSSG + (i))
#define S_GEN(g) (NFG + NSSG + 2u + (g))
#define S_QUAD(k) (NFG + NSSG + 2u + NGEN + (k))   /* the quadrature LFO: SIN+ SIN- COS+ COS- (+-) */
#define NSEQ 3u                                 /* slew sequencers */
#define S_SEQ(n) (NFG + NSSG + 2u + NGEN + 4u + (n))
#define NSRC (NFG + NSSG + 2u + NGEN + 4u + NSEQ)
enum { D_TILT, D_STR, D_LVL, D_SPLIT, D_FILTA, D_FILTB, D_PANA, D_PANB, D_PITA, D_PITB,
       D_K1PART, D_K1VEL, D_K1SHP, D_K1LVL, D_K2PART, D_K2VEL, D_K2SHP, D_K2LVL,
       D_MUTA, D_MUTB, D_SPDA, D_SPDB, D_FG1, D_FG2, D_FG3, D_FG4, D_QRATE, NDST };
/* the gate matrix: rows (sources) and columns (destinations); pins OR'd into each column */
enum { GS_GEN = 0, GS_LFO = 4, GS_EOC = 8, GS_QNT = 12, GS_CLK = 14, GS_POP = 19, GS_SEQ = 21, NGS = 24 };
enum { GD_K1, GD_K2, GD_FG1, GD_FG2, GD_FG3, GD_FG4, GD_SH1, GD_SH2, GD_GA, GD_GB,
       GD_SQA1, GD_SQA2, GD_SQA3, GD_SQR1, GD_SQR2, GD_SQR3, NGD };   /* (16: one uint16 a row) */
enum { SD_FWD, SD_PEND, SD_RND };
typedef struct { uint8_t val[8], gate[8], len, dir, slew, addr, chain; } seq_t;   /* addr 0: clocked; 1..: swept by a
                                                                                      matrix source (addr - 1) */
typedef struct { uint8_t rule, rate, mut, rescue, dens, shift; } gpar_t;   /* rate: the grid's own clock (0..127) */
typedef struct { uint8_t mode, part, vel, level, shape, decay, fall, series, trig, filt; } kpar_t;
typedef struct { uint8_t mode, freq, res, bw, level, pan; int8_t coarse, fine; } ppar_t;
typedef struct { uint8_t in, op, mode, rate, duty; } gen_t;   /* op: AND OR XOR NAND NOR XNOR, IN, LFO; mode 0 free 1 clock */
typedef struct { uint8_t rise, fall, curve, mode, trig; } fgp_t;
typedef struct { uint8_t input, clock, slew; } ssgp_t;      /* input 0 noise, 1.. a matrix source - 1 */
typedef struct { uint8_t count, space, pattern, max; } trill_t;   /* space in 10 ms */
typedef struct {
    uint8_t root, slew, tilt, level, anchor, tempo, clk_midi, gmode, engver;   /* engver: 1 (8 bit), 2 (12 bit) */
    int8_t stretch;                             /* per mille of the exponent: -30 .. +60 */
    gpar_t g[2];
    kpar_t k[2];
    uint8_t route_pre;                          /* 0: strike 1 -> A, strike 2 -> B; 1: through the split */
    uint8_t srule, spoint, swidth, ssoft, sn;
    ppar_t pth[2];
    uint8_t tun, scl, sroot, qon[2], glide[2];  /* the global tuning, its scale (index within it), root (0..11);
                                                 * per path: quantise on / off, glide */
    gen_t gen[NGEN];
    fgp_t fg[NFG];
    ssgp_t ssg[NSSG];
    trill_t tr[2];
    int8_t dep[NSRC][NDST];                     /* the mod matrix: 0 = no pin, +-63 */
    uint16_t gmat[NGS];                         /* the gate matrix: bit d = a pin to destination d */
    uint8_t qrate;                              /* the quadrature LFO's rate (as S6: FG speeds) */
    uint8_t lock;                               /* 1: grid B runs on grid A's clock (lockstep) */
    seq_t seq[NSEQ];
} patch_t;
/* logic sources: 0 off; 1..11 the clock and its divisions; 12..43 grid A's cells; 44..75 grid B's; 76, 77 pop > half;
 * 78..81 the generators; 82..85 the FGs' end of cycle */
#define LS_GEN(g) (78u + (g))
#define LS_EOC(k) (82u + (k))
#define NLS 86u
static patch_t P = {
    33, 32, 100, 100, 1, 64, 0, GM_LOWHIGH, 2, 0,
    {{90, 69, 3, 1, 30, 0}, {30, 85, 4, 1, 30, 0}},   /* grid A 1 step a second, grid B 2 */
    {{KM_STRIKE, 3, 89, 100, 64, 70, 76, SE_ALL, LS_GEN(0), 0},
     {KM_MALLET, 2, 76, 100, 44, 82, 51, SE_ALL, LS_GEN(1), 0}},
    0,
    SR_CUT, 64, 15, 12, 3,
    {{1, 0, 40, 12, 90, 37, 0, 0}, {3, 79, 30, 12, 100, 91, 0, 0}},
    0, 1, 9, {0, 1}, {0, 24},                   /* 12-TET, major on A; path B quantised, a little glide */
    {{16, 0, 1, 24, 64},                         /* G1: A cell 5 AND clock /3 (clocked: rate 24 = the third division) */
     {46, 0, 1, 47, 64},                         /* G2: B cell 3 AND clock /5 */
     {0, 7, 0, 70, 50},                          /* G3: a free LFO alone */
     {78, 0, 0, 96, 60}},                        /* G4: G1 AND a fast LFO: rolls */
    {{90, 96, CV_LIN, FM_CYCLE, 0}, {30, 70, CV_EXP, FM_ONCE, 79}, {70, 80, CV_LOG, FM_CYCLE, 0}, {40, 60, CV_LIN, FM_GATE, 80}},
    {{0, 78, 0}, {0, 10, 40}},
    {{1, 8, TP_HUMP, 120}, {1, 6, TP_FALL, 110}},
    {{0}},
    {(1u << GD_K1) | (1u << GD_SH1), (1u << GD_K2) | (1u << GD_FG2), 1u << GD_FG4, 0,   /* G1: strike 1, S&H 1; G2: strike 2, FG2; G3: FG4 */
     0, 0, 0, 0,  0, 0, 0, 0,  0, 0,  0, 0, 1u << GD_SQA2, 0, 1u << GD_SH2,  0, 0,  0, 0, 0},
    /* (G3, a free LFO, also advances SEQ 1 - below; clock /4 advances SEQ 2; clock /16 clocks S&H 2) */
    50, 0,
    {{{20, 90, 45, 110, 30, 75, 60, 127}, {1, 0, 1, 0, 1, 1, 0, 1}, 8, SD_FWD, 40, 0, 0},
     {{0, 40, 80, 127, 90, 50, 0, 0}, {1, 0, 0, 1, 0, 0, 0, 0}, 5, SD_PEND, 90, 0, 0},
     {{10, 60, 30, 100, 120, 70, 0, 0}, {1, 1, 1, 1, 1, 1, 0, 0}, 6, SD_FWD, 20, 1, 0}},   /* SEQ 3 swept by FG 1 */
};
static patch_t E;                               /* the patch as played: P with the matrix's modulation on it */
static const char *const GM_NAME[NGM] = {"LO/HI", "ODD/EVN", "A AND B", "A OR B", "A XOR B"};
static const char *const KM_NAME[NKM] = {"STRIKE", "MALLET", "BOW", "BREATH"};
static const char *const KSH_NAME[NKM] = {"HARD", "SOFT", "PRESS", "AIR"};
static const char *const SE_NAME[3] = {"ALL", "ODD", "EVEN"};
static const char *const SR_NAME[NSR] = {"CUTOFF", "BAND", "ODD/EVN", "EVERY N", "GRID AB", "SOURCE"};
static const char *const PM_NAME[4] = {"OFF", "VOWEL", "BP 12DB", "BP 24DB"};
static const char *const OP_NAME[8] = {"AND", "OR", "XOR", "NAND", "NOR", "XNOR", "IN", "LFO"};
static const char *const TP_NAME[3] = {"RISE", "FALL", "HUMP"};
static const char *const CV_NAME[3] = {"LOG", "LIN", "EXP"};
static const char *const FM_NAME[3] = {"CYCLE", "ONCE", "GATE"};
static void default_pins(void)                  /* a few pins to hear the matrix at once */
{
    P.gmat[GS_GEN + 2] |= 1u << GD_SQA1;        /* G3 advances SEQ 1 */
    P.dep[S_SEQ(0)][D_FILTA] = 28;              /* SEQ 1 walks path A's vowel */
    P.dep[S_SEQ(1)][D_STR] = 10;                /* SEQ 2, slewed: a slow stretch */
    P.dep[S_FG(0)][D_SPLIT] = 24;               /* FG1's slow cycle moves the split */
    P.dep[S_FG(1)][D_TILT] = -20;               /* FG2 (on G2) darkens, then brightens */
    P.dep[S_SSG(0)][D_PITB] = 30;               /* SSG1 (noise, on G1): a melody on path B, quantised */
    P.dep[S_FG(2)][D_FILTB] = 20;               /* FG3 sweeps path B's band pass */
    P.dep[S_POP(1)][D_K1PART] = 12;             /* grid B's population moves strike 1's partial */
}

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
    for (c = 0; c < NC; c++) if (rnd(100) < E.g[i].mut) n ^= 1u << c;
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
    int32_t s = E.g[0].shift;
    switch (E.gmode) {
    case GM_LOWHIGH: return h <= 32u ? cell(0, (int32_t)h - 1 + s) : cell(1, (int32_t)h - 33 + E.g[1].shift);
    case GM_ODDEVEN: return h & 1u ? cell(0, (int32_t)(h - 1u) / 2 + s) : cell(1, (int32_t)h / 2 - 1 + E.g[1].shift);
    default: {
        uint32_t a = cell(0, (int32_t)((h - 1u) >> 1) + s), b = cell(1, (int32_t)((h - 1u) >> 1) + E.g[1].shift);
        return E.gmode == GM_AND ? a & b : E.gmode == GM_OR ? a | b : a ^ b;
    }
    }
}
static uint32_t owner_a(uint32_t h)
{
    if (E.gmode == GM_LOWHIGH) return h <= 32u;
    if (E.gmode == GM_ODDEVEN) return h & 1u;
    return cell(0, (int32_t)((h - 1u) >> 1) + E.g[0].shift);
}

/* ================================================================== the quantiser (as Arcturus S6) */
static int32_t quantise(int32_t q, int32_t *stepn)   /* a pitch (1/256 semitone) to the nearest step of the scale */
{
    uint32_t t = P.tun < NTUN ? P.tun : 0u, n = TUN_N[t], j, sc = TUN_SCL0[t] + P.scl;
    int32_t per = TUN_PERIOD[t], c, k, rem, best = 0x7FFFFFFF, bv = 0, bj = 0;
    const uint16_t *st = &TUN_STEPS[TUN_OFF[t]];
    uint32_t lo, hi;
    if (sc < TUN_SCL0[t] || sc >= TUN_SCL0[t] + TUN_NSCL[t]) sc = TUN_SCL0[t];
    lo = SCL_LO[sc]; hi = SCL_HI[sc];
    if (!lo && !hi) lo = 1u;
    c = q - (int32_t)P.sroot * 256;
    k = c >= 0 ? c / per : -((-c + per - 1) / per);
    rem = c - k * per;
    for (j = 0; j < n; j++) {                   /* this period's steps, the next one's first, the last one's last */
        int32_t v, d, kk;
        if (!(j < 32u ? lo >> j & 1u : hi >> (j - 32u) & 1u)) continue;
        for (kk = -1; kk <= 1; kk++) {
            v = (int32_t)st[j] + kk * per;
            d = rem - v;
            if (d < 0) d = -d;
            if (d < best) { best = d; bv = v; bj = (int32_t)j; }
        }
    }
    *stepn = bj;
    return (int32_t)P.sroot * 256 + k * per + bv;
}
static volatile uint32_t qpulse_v[2];           /* a quantiser's note-change gate: blocks still high */
static int32_t off_tgt[2], off_cur[2], off_step[2], pitch_mod[2];   /* (pitch_mod: the matrix, 1/256 semitone) */   /* each path's pitch offset (1/256 semitone): where it goes, where it is */
static uint32_t t_glide;
static uint32_t offsets_update(void)            /* 1: an offset moved (the partials need pushing) */
{
    uint32_t k, moved = 0, dt = fm1_ms - t_glide;
    t_glide = fm1_ms;
    if (dt > 100u) dt = 100u;
    for (k = 0; k < 2u; k++) {
        int32_t o = (int32_t)P.pth[k].coarse * 256 + (int32_t)P.pth[k].fine * 256 / 100 + pitch_mod[k], prev = off_cur[k];
        if (P.qon[k]) {
            int32_t q = quantise((int32_t)P.root * 256 + o, &off_step[k]) - (int32_t)P.root * 256;
            if (q != off_tgt[k]) qpulse_v[k] = 7;   /* a new note: a ~5 ms gate */
            o = q;
        }
        off_tgt[k] = o;
        if (!P.glide[k] || iabs(o - off_cur[k]) < 2) off_cur[k] = o;
        else {                                  /* a glide: a one-pole, glide x 20 ms */
            int32_t tau = (int32_t)P.glide[k] * 20;
            off_cur[k] += (o - off_cur[k]) * (int32_t)dt / (tau + (int32_t)dt);
        }
        if (off_cur[k] != prev) moved = 1;
    }
    return moved;
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
    int32_t qc = 8858 + (int32_t)E.spoint * (30720 - 8858) / 127, soft = (int32_t)E.ssoft * 256, x;
    switch (E.srule) {
    case SR_ODD: return h & 1u ? ONE : 0;
    case SR_NTH: return h % (E.sn < 2u ? 2u : E.sn) == 0u ? ONE : 0;
    case SR_GRID: return owner_a(h) ? ONE : 0;
    case SR_SRC: return ONE;
    case SR_CUT: x = qb - qc; break;
    default: x = iabs(qb - qc) - (int32_t)E.swidth * 256 * 12 / 20; break;   /* width in 1/10 octave: half of it each side */
    }
    if (soft <= 0) return x < 0 ? ONE : 0;
    x = cl(x, -60000, 60000);                   /* (x * 16384 stays in 32 bits: no 64-bit division on this chip) */
    return (uint32_t)cl(16384 - x * 16384 / soft * 2, 0, ONE);
}
static void path_push(void)
{
    uint32_t k;
    for (k = 0; k < 2u; k++) {
        const ppar_t *q = &E.pth[k];
        int32_t sp = (int32_t)q->bw * 19 / 20;   /* half the bandwidth in knob steps (19.2 a octave; bw in 1/20 oct) */
        int32_t lv = (int32_t)q->level * 258 * E.level / 127;   /* 0..127 -> ~1.0 at 127; x the voice's level */
        pf_v[k][0] = Q_SVF_F[q->mode == 3u ? cl((int32_t)q->freq - sp, 0, 127) : q->freq];
        pf_v[k][1] = Q_SVF_F[cl((int32_t)q->freq + sp, 0, 127)];
        pq_v[k] = Q_RES_D[q->res];
        pmk_v[k] = q->mode == 3u ? 10650 : 6554;
        pgl_v[k] = (int32_t)LUT_PAN_L[q->pan & 127u] * lv >> 15;
        pgr_v[k] = (int32_t)LUT_PAN_R[q->pan & 127u] * lv >> 15;
        pm_v[k] = q->mode;
    }
}
static void strike_prepare(uint32_t s);
static volatile int32_t src_v[NSRC];            /* every matrix source now (Q15): the interrupt keeps them */
static volatile int8_t fgmod_v[NFG];            /* the matrix on each FG's rate, knob steps */
static volatile int8_t qmod_v;                  /* the matrix on the quad LFO's rate, knob steps */
static volatile int32_t sq_k_v[3];              /* the sequencers' slew steps a block, Q16 */
static int32_t macc[NDST];                      /* each destination's modulation, knob steps x 256 */
static uint32_t any_pin;
static uint8_t m8(uint8_t v, int32_t d, int32_t lo, int32_t hi) { return (uint8_t)cl((int32_t)v + d, lo, hi); }
static void mod_apply(void)                     /* E = P, moved by the matrix (a full source at depth 63 ~ a whole knob) */
{
    uint32_t k, d, n = 0;
    int32_t src[NSRC];
    for (k = 0; k < NSRC; k++) src[k] = src_v[k];
    for (d = 0; d < NDST; d++) macc[d] = 0;
    for (k = 0; k < NSRC; k++)
        for (d = 0; d < NDST; d++) {
            int32_t dep = P.dep[k][d];
            if (dep) { macc[d] += src[k] * dep >> 6; n++; }
        }
    any_pin = n != 0u;
    E = P;
#define MK(d) (macc[d] >> 8)
    E.tilt = m8(P.tilt, MK(D_TILT) * 2, 0, 250);
    E.stretch = (int8_t)cl((int32_t)P.stretch + MK(D_STR) / 2, -60, 90);
    E.level = m8(P.level, MK(D_LVL), 0, 127);
    E.spoint = m8(P.spoint, MK(D_SPLIT), 0, 127);
    E.pth[0].freq = m8(P.pth[0].freq, MK(D_FILTA), 0, 127);
    E.pth[1].freq = m8(P.pth[1].freq, MK(D_FILTB), 0, 127);
    E.pth[0].pan = m8(P.pth[0].pan, MK(D_PANA), 0, 127);
    E.pth[1].pan = m8(P.pth[1].pan, MK(D_PANB), 0, 127);
    pitch_mod[0] = macc[D_PITA] * 3 / 32;       /* +-12 semitones at full, 1/256 semitone */
    pitch_mod[1] = macc[D_PITB] * 3 / 32;
    for (k = 0; k < 2u; k++) {
        uint32_t b = k ? D_K2PART : D_K1PART;
        E.k[k].part = m8(P.k[k].part, MK(b) / 4, 1, 32);
        E.k[k].vel = m8(P.k[k].vel, MK(b + 1u), 0, 127);
        E.k[k].shape = m8(P.k[k].shape, MK(b + 2u), 0, 127);
        E.k[k].level = m8(P.k[k].level, MK(b + 3u), 0, 127);
        E.g[k].mut = m8(P.g[k].mut, MK(k ? D_MUTB : D_MUTA) / 4, 0, 50);
        E.g[k].rate = m8(P.g[k].rate, MK(k ? D_SPDB : D_SPDA), 0, 127);   /* (up: faster) */
    }
    for (k = 0; k < NFG; k++) fgmod_v[k] = (int8_t)cl(-MK(D_FG1 + k), -127, 127);   /* (up: faster) */
    qmod_v = (int8_t)cl(MK(D_QRATE), -127, 127);
    for (k = 0; k < NSEQ; k++) { int32_t sl = P.seq[k].slew; sq_k_v[k] = 65536 / (1 + sl * sl / 2); }   /* slew 127: ~6 s */
#undef MK
}
static void push_partials(void)
{
    uint32_t h, sum = 0, w[NP], kg = CA_SLEW_K[E.slew];
    int32_t offA = off_cur[0], offB = off_cur[1];
    uint32_t snap = E.qon[0] || E.qon[1];       /* quantising: each partial takes its main path's pitch (on the scale) */
    int32_t klev[2];
    klev[0] = E.k[0].level * 258; klev[1] = E.k[1].level * 258;   /* 0..127 -> Q15 (127: ~1.0) */
    for (h = 1; h <= NP; h++) {
        int32_t st = (int32_t)(CA_L2[h - 1u] * 3u / 4u), qb = (int32_t)E.root * 256 + st + st * E.stretch / 1000, q;
        uint32_t on, wt = 0, sw = split_w(h, qb);
        q = snap ? qb + (sw >= 16384u ? offA : offB)
                 : qb + (int32_t)(((int64_t)sw * offA + (int64_t)(ONE - (int32_t)sw) * offB) >> 15);
        pq_base[h - 1u] = qb; pq_now[h - 1u] = q; wsplit[h - 1u] = (int32_t)sw;
        on = cell_on(h) || (h == 1u && E.anchor);
        if (on && q <= Q_TOP) {
            wt = pow_neg(h, E.tilt * 256u / 100u);
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
        va = E.pth[0].mode == 1u ? vowel_w(hz, E.pth[0].freq, E.pth[0].res) : ONE;
        vb = E.pth[1].mode == 1u ? vowel_w(hz, E.pth[1].freq, E.pth[1].res) : ONE;
        vow_show[0][h] = va; vow_show[1][h] = vb;
        va = (va * top) >> 15; vb = (vb * top) >> 15;
        gw = E.srule == SR_SRC ? ONE : wsplit[h];
        wt_v[0][h] = (gw * va) >> 15;
        wt_v[1][h] = ((ONE - gw) * vb) >> 15;
        for (k = 0; k < 2u; k++) {
            kw = !E.route_pre ? (k == 0u ? ONE : 0) : E.srule == SR_SRC ? 0 : wsplit[h];
            wt_v[2u + 2u * k][h] = (((kw * va) >> 15) * klev[k]) >> 15;
            wt_v[3u + 2u * k][h] = ((((ONE - kw) * vb) >> 15) * klev[k]) >> 15;
        }
    }
    path_push();
    bits_v = E.engver == 1u ? 2u : 1u;          /* ver 1: the 8 bit machine; ver 2: the 12 bit revision */
    strike_prepare(0); strike_prepare(1);
}

/* ================================================================== the strikes: their templates (main loop) */
/* A strike's partials, worked out ahead (on any change, and as the matrix moves them): each member's kick at full
 * velocity, its attack and decay; Bow / Breath: the level held at full velocity, attack, flicker. The interrupt fires
 * them, scaled by the hit's velocity, the moment the gate rises: the trills stay in time whatever the screen does. */
static volatile int32_t kamp_t[2][NP];          /* the kick (Strike / Mallet) or the level held (Bow / Breath) at velocity 1 */
static volatile int32_t kvel_v[2], kmax_v[2];   /* the strike's velocity and the trill's peak, Q15 */
static volatile uint32_t kfire_v[2], kgate_v[2], kman_req_v[2], kman_hold_v[2];
static uint8_t kmember[2][NP];
static uint32_t kman_until[2];
static uint32_t tau_k(uint32_t tau_ms)          /* a one-pole's step a block for a time constant, Q16 */
{
    uint32_t b = tau_ms * 1378u / 1000u;
    return b < 1u ? 65536u : 65536u / b;
}
static void strike_prepare(uint32_t s)
{
    const kpar_t *k = &E.k[s];
    uint32_t h, m, n = 0, list_h[NP], list_m[NP], list_i[NP], sust = k->mode >= KM_BOW, part = k->part ? k->part : 1u;
    int32_t vel = (int32_t)k->vel * 258, shp = k->shape;   /* Q15; 0..127 */
    int32_t amp[NP];
    for (h = 0; h < NP; h++) { amp[h] = 0; kmember[s][h] = 0; }
    if (k->mode == KM_MALLET) {                 /* the bar's ratios: 1, 4, 10 */
        static const uint8_t R[3] = {1, 4, 10};
        for (m = 0; m < 3u; m++) if (part * R[m] <= NP) { list_h[n] = part * R[m]; list_m[n] = R[m]; list_i[n] = m; n++; }
    } else
        for (m = 1; part * m <= NP; m++) {
            if (m > 1u && k->series == SE_ODD && !(m & 1u)) continue;
            if (m > 1u && k->series == SE_EVEN && (m & 1u)) continue;
            list_h[n] = part * m; list_m[n] = m; list_i[n] = m - 1u; n++;
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
        if (k->mode == KM_STRIKE || k->mode == KM_MALLET) {
            uint32_t att_us;
            if (k->mode == KM_STRIKE) {         /* (the reach and brightness from the strike's velocity) */
                uint32_t e_q8 = 435u - 307u * (uint32_t)shp / 127u, reach = 1u + (uint32_t)(vel * (3 + shp * 22 / 127)) / 32768u;
                a = (int32_t)(((int64_t)6554 * pow_neg(mm, e_q8)) >> 15);   /* 0.2 / m^e */
                if (mm > reach) a >>= (mm - reach) > 15u ? 15u : mm - reach;
                att_us = 1500000u / hz; if (att_us < 1500u) att_us = 1500u;
            } else {
                static const int32_t F0[3] = {32768, 14746, 8192};   /* 1, 0.45, 0.25 */
                int32_t f = F0[list_i[m]];
                if (list_i[m] == 1u) f = (int32_t)(((int64_t)f * (32768 - 19661 * shp / 127) >> 15) * (13107 + vel) >> 15);
                if (list_i[m] == 2u) f = (int32_t)(((int64_t)f * (32768 - 27853 * shp / 127) >> 15) * vel >> 15);
                a = (7209 * f) >> 15;           /* 0.22 */
                att_us = 4000u + 30000u * (uint32_t)shp / 127u;
                if (att_us < 2000000u / hz) att_us = 2000000u / hz;
            }
            if (a < 3) continue;
            katt_v[s][p] = (uint16_t)cl((int32_t)(att_us * 1378u / 1000000u), 2, 4000);
            kfl_v[s][p] = 0;
        } else {                                /* Bow / Breath: a level to hold, an attack, a flicker */
            uint32_t tau;
            if (k->mode == KM_BOW) {
                a = (int32_t)(((int64_t)4915 * pow_neg(mm, 256)) >> 15);   /* 0.15 / m */
                if ((int32_t)mm * 32768 > 32768 + vel * 18) a >>= 2;
                tau = (30u + 900u * (127u - (uint32_t)shp) / 127u) * (100u + 35u * (mm - 1u)) / 100u;
                kfl_v[s][p] = 2300;
            } else {
                a = (int32_t)(((int64_t)3932 * pow_neg(mm, 205)) >> 15);   /* 0.12 / m^0.8 */
                tau = 60u * (100u + 15u * (mm - 1u)) / 100u;
                kfl_v[s][p] = (uint16_t)((uint32_t)shp * 258u * (11469u + 21299u * (mm < 12u ? mm : 12u) / 12u) >> 15);
            }
            kku_v[s][p] = tau_k(tau);
        }
        kkd_v[s][p] = tau_k(dec);
        amp[p] = a;
        kmember[s][p] = 1;
    }
    for (h = 0; h < NP; h++) kamp_t[s][h] = amp[h];
    kvel_v[s] = vel;
    kmax_v[s] = (int32_t)E.tr[s].max * 258;
    if (kimp_v[s] != !sust) { kimp_v[s] = !sust; kheld_v[s] = 0; }
}

/* ================================================================== the clock (main loop) */
static const uint8_t DIVS[11] = {1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 32};
static volatile uint32_t half_n;                /* half pulses since start (the interrupt's logic reads it) */
static uint32_t t_half, t_halfb, half_b, midi_clk, gstep_seen[2];
static volatile uint32_t gstep_req_v[2];        /* a grid step asked for by the gate matrix */
static volatile uint32_t dirty_partials = 1;
static const uint16_t MIDI_DIV[8] = {3, 6, 12, 24, 48, 96, 192, 384};   /* MIDI clocks a step: 1/32 .. 4 bars */
static const char *const MIDI_DIVN[8] = {"1/32", "1/16", "1/8", "1/4", "1/2", "1 BAR", "2 BAR", "4 BAR"};
static uint32_t gated(uint32_t i) { uint32_t r; for (r = 0; r < NGS; r++) if (P.gmat[r] >> (GD_GA + i) & 1u) return 1u; return 0u; }
static void half_tick(void)                     /* the master clock (grid A's), half a step: the CLK rows' squares */
{
    half_n++;
    if (!(half_n & 1u)) {
        if (!gated(0)) { grid_step(0); dirty_partials = 1; }
        if (P.lock && !gated(1)) { grid_step(1); dirty_partials = 1; }
    }
}
static void clock_poll(void)                    /* grid A's clock: internal, or MIDI; grid B's: its own, or A's */
{
    uint8_t st, d1, d2;
    uint32_t s;
    while (midi_get(&st, &d1, &d2)) {
        if (!P.clk_midi) continue;
        if (st == 0xF8u) { if (++midi_clk >= MIDI_DIV[E.g[0].rate >> 4] / 2u) { midi_clk = 0; half_tick(); } }
        else if (st == 0xFAu) { midi_clk = 0; half_n = 0xFFFFFFFFu; }
    }
    if (!P.clk_midi && fm1_ms - t_half >= Q_GRID_HMS[E.g[0].rate]) {
        t_half += Q_GRID_HMS[E.g[0].rate];
        if (fm1_ms - t_half > 2000u) t_half = fm1_ms;
        half_tick();
    }
    if (!P.lock && fm1_ms - t_halfb >= Q_GRID_HMS[E.g[1].rate]) {   /* grid B on its own clock */
        t_halfb += Q_GRID_HMS[E.g[1].rate];
        if (fm1_ms - t_halfb > 2000u) t_halfb = fm1_ms;
        if (!(++half_b & 1u) && !gated(1)) { grid_step(1); dirty_partials = 1; }
    }
    for (s = 0; s < 2u; s++)                    /* a gate from the matrix steps a grid */
        if (gstep_req_v[s] != gstep_seen[s]) { gstep_seen[s] = gstep_req_v[s]; grid_step(s); dirty_partials = 1; }
    for (s = 0; s < 2u; s++)                    /* a hand-held Bow / Breath lets go when its time is up */
        if (kman_hold_v[s] && (int32_t)(fm1_ms - kman_until[s]) > 0) kman_hold_v[s] = 0;
}
static uint32_t idle_ok;
static void app_idle(void) { if (idle_ok) clock_poll(); }

/* ================================================================== the logic, the FGs, the SSGs (audio interrupt, a block) */
static volatile uint32_t gen_out_v, gen_in_v, gen_lfo_v;   /* bit g: generator g's output, input, LFO */
static volatile uint32_t gsrc_v, gdst_v;        /* the gate matrix now: bit r a source high, bit d a destination high */
static uint32_t lfo_ph[NGEN];
#define FG_TOP (1 << 24)
enum { FG_IDLE, FG_RISE, FG_HOLD, FG_FALL };
typedef struct { int32_t x; uint8_t stage, gate_prev, eoc; } fg_st_t;
static fg_st_t fgs[NFG];
static int32_t ssg_held[NSSG], ssg_out[NSSG];
static uint32_t q_ph;                           /* the quadrature LFO's phase */

static uint8_t ssg_prev[NSSG];
static volatile uint32_t ssg_fired_v[NSSG];
static uint32_t isr_rnd = 0x1234567u;
static volatile uint32_t sq_gate_v;
static uint32_t lsrc(uint32_t id)               /* a logic source now (also read by the main loop, for the lamps) */
{
    if (id == 0u) return 0u;
    if (id <= 11u) return (half_n / DIVS[id - 1u]) % 2u == 0u;
    if (id <= 43u) return cell(0, (int32_t)id - 12);
    if (id <= 75u) return cell(1, (int32_t)id - 44);
    if (id <= 77u) return popc(G[id - 76u].row) > NC / 2u;
    if (id <= 81u) return gen_out_v >> (id - 78u) & 1u;
    if (id <= 85u) return fgs[id - 82u].eoc != 0;   /* (a generator's input can read these; the rest is the matrix's) */
    return 0u;
}
static uint32_t lop(uint32_t a, uint32_t b, uint32_t op)
{
    switch (op) { case 0: return a & b; case 1: return a | b; case 2: return a ^ b; case 3: return !(a & b);
                  case 4: return !(a | b); case 5: return !(a ^ b); case 6: return a; default: return b; }
}
static int32_t fg_shape(int32_t x, uint32_t curve)   /* position Q24 -> level 0..32767 (as S6) */
{
    uint32_t i = (uint32_t)x >> 16, f = ((uint32_t)x >> 8) & 255u;
    const uint16_t *t;
    if (curve == CV_LIN) return x >= FG_TOP ? 32767 : x >> 9;
    t = curve == CV_EXP ? LUT_SHAPE_EXP : LUT_SHAPE_LOG;
    if (i >= 256u) return t[256];
    return (int32_t)t[i] + ((((int32_t)t[i + 1] - (int32_t)t[i]) * (int32_t)f) >> 8);
}
static void fg_step(uint32_t k, uint32_t g)     /* one FG, one block: S6's Serge slope */
{
    fg_st_t *s = &fgs[k];
    uint32_t mode = P.fg[k].mode, rk = (uint32_t)cl((int32_t)P.fg[k].rise + fgmod_v[k], 0, 127), fk = (uint32_t)cl((int32_t)P.fg[k].fall + fgmod_v[k], 0, 127);
    if (s->eoc) s->eoc--;
    if (mode == FM_CYCLE) { if (s->stage == FG_IDLE || s->stage == FG_HOLD) s->stage = FG_RISE; if (g && !s->gate_prev) s->stage = FG_RISE; }
    else if (mode == FM_GATE) {
        if (g && (s->stage == FG_IDLE || s->stage == FG_FALL)) s->stage = FG_RISE;
        else if (!g && (s->stage == FG_RISE || s->stage == FG_HOLD)) s->stage = FG_FALL;
    } else { if (g && !s->gate_prev) s->stage = FG_RISE; if (s->stage == FG_HOLD) s->stage = FG_FALL; }
    s->gate_prev = (uint8_t)g;
    if (s->stage == FG_RISE) {
        s->x += (int32_t)LUT_FGINC[rk];
        if (s->x >= FG_TOP) { s->x = FG_TOP; s->stage = mode == FM_GATE ? FG_HOLD : FG_FALL; }
    } else if (s->stage == FG_FALL) {
        s->x -= (int32_t)LUT_FGINC[fk];
        if (s->x <= 0) { s->x = 0; s->eoc = 7; s->stage = mode == FM_CYCLE ? FG_RISE : FG_IDLE; }   /* EOC ~5 ms */
    }
    src_v[S_FG(k)] = fg_shape(s->x, P.fg[k].curve);
}
/* the strikes in the interrupt: a gate's rise starts a trill (one hit, or COUNT hits SPACE apart with a velocity
 * pattern); Bow / Breath hold while the gate is high */
static uint32_t kgprev[2], kman_seen[2], tr_left[2], tr_i[2], tr_wait[2];
static void strike_hit(uint32_t s, int32_t v)   /* kick every member, v = the hit's velocity (Q15) */
{
    uint32_t h;
    for (h = 0; h < NP; h++) {
        int32_t add = (int32_t)(((int64_t)kamp_t[s][h] * v) >> 15), pk, n;
        if (add <= 0) continue;
        pk = knee((ke[s][h] >> 8) + add);
        n = katt_v[s][h] ? katt_v[s][h] : 1;
        katn[s][h] = (uint16_t)n;
        katd[s][h] = ((pk << 8) - ke[s][h]) / n;
    }
    kfire_v[s]++;
}
static int32_t trill_vel(uint32_t s, uint32_t i, uint32_t n)   /* hit i of n: between the strike's velocity and the peak */
{
    int32_t lo = kvel_v[s], hi = kmax_v[s], f;   /* f: 0..256 along the pattern */
    if (n < 2u) return lo;
    switch (P.tr[s].pattern) {
    case TP_RISE: f = (int32_t)(i * 256u / (n - 1u)); break;
    case TP_FALL: f = 256 - (int32_t)(i * 256u / (n - 1u)); break;
    default: f = 256 - iabs((int32_t)(i * 512u / (n - 1u)) - 256); break;
    }
    return lo + (hi - lo) * f / 256;
}
/* the slew sequencers: CLOCKED (an advance gate from the matrix, or the one before wrapping when chained; a reset gate
 * back to step 1) or SWEPT (a matrix source picks the step: low = step 1, high = the last, as a Serge sequencer under
 * an FG). Entering a step whose gate is on fires a short gate (to the gate matrix). The output glides at SLEW. */
static uint32_t sq_step[NSEQ], sq_dir[NSEQ], sq_adv_prev, sq_rst_prev, sq_wrap, sq_pulse[NSEQ];
static int32_t sq_out[NSEQ];
static volatile uint32_t sq_step_v[NSEQ];       /* for the screens */

static void seq_enter(uint32_t n, uint32_t st)
{
    if (P.seq[n].gate[st & 7u]) sq_pulse[n] = 7;   /* (~5 ms) */
    sq_step[n] = st & 7u;
}
static void seq_block(void)
{
    uint32_t n, wraps = 0, adv = gdst_v >> GD_SQA1 & 7u, rst = gdst_v >> GD_SQR1 & 7u, gates = 0;
    for (n = 0; n < NSEQ; n++) {
        const seq_t *q = &P.seq[n];
        uint32_t len = q->len < 1u ? 1u : q->len > 8u ? 8u : q->len, cur = sq_step[n] >= len ? 0u : sq_step[n];
        if (q->addr) {                          /* swept: the source's level picks the step */
            uint32_t k = q->addr - 1u, st;
            int32_t v = k < NSRC ? src_v[k] : 0;
            if (k >= NFG && k < NFG + NSSG) v = (v + 32768) >> 1;   /* (the bipolar ones lifted: S&H, quad) */
            if (k >= S_QUAD(0) && k < S_QUAD(0) + 4u) v = (v + 32768) >> 1;
            st = (uint32_t)cl(v, 0, 32767) * len >> 15;
            if (st >= len) st = len - 1u;
            if (st != cur) seq_enter(n, st);
            else sq_step[n] = cur;
        } else {
            uint32_t go = ((adv >> n & 1u) && !(sq_adv_prev >> n & 1u)) || (q->chain && n && (sq_wrap >> (n - 1u) & 1u));
            if ((rst >> n & 1u) && !(sq_rst_prev >> n & 1u)) seq_enter(n, 0);
            else if (go) {
                uint32_t nx;
                if (q->dir == SD_RND) { isr_rnd ^= isr_rnd << 13; isr_rnd ^= isr_rnd >> 17; isr_rnd ^= isr_rnd << 5; nx = (isr_rnd >> 16) % len; }
                else if (q->dir == SD_PEND) {
                    if (len < 2u) nx = 0;
                    else { if (sq_dir[n]) { nx = cur ? cur - 1u : 1u; if (!nx) sq_dir[n] = 0; } else { nx = cur + 1u >= len ? len - 2u : cur + 1u; if (nx == len - 1u) sq_dir[n] = 1; } }
                    if (nx == 0u) wraps |= 1u << n;
                } else { nx = cur + 1u >= len ? 0u : cur + 1u; if (!nx) wraps |= 1u << n; }
                seq_enter(n, nx);
            }
        }
        sq_out[n] += (int32_t)(((int64_t)((int32_t)P.seq[n].val[sq_step[n]] * 258 - sq_out[n]) * sq_k_v[n]) >> 16);
        src_v[S_SEQ(n)] = sq_out[n];
        if (sq_pulse[n]) { sq_pulse[n]--; gates |= 1u << n; }
        sq_step_v[n] = sq_step[n];
    }
    sq_adv_prev = adv; sq_rst_prev = rst; sq_wrap = wraps; sq_gate_v = gates;
}
static void logic_block(void)                   /* once a block (1378 Hz): generators, FGs, SSGs, strikes */
{
    uint32_t g, k, s, outs = 0, ins = 0, lfos = 0;
    for (g = 0; g < NGEN; g++) {                /* (the others' outputs as they were a block ago) */
        const gen_t *q = &P.gen[g];
        uint32_t in = lsrc(q->in), l;
        if (q->mode) l = (half_n / DIVS[q->rate * 11u / 128u]) % 2u == 0u;   /* clocked: a division of the clock */
        else { lfo_ph[g] += LUT_GGINC[q->rate & 127u]; l = lfo_ph[g] < ((uint32_t)cl(q->duty, 1, 126) << 25); }
        if (lop(in, l, q->op)) outs |= 1u << g;
        if (in) ins |= 1u << g;
        if (l) lfos |= 1u << g;
    }
    gen_out_v = outs; gen_in_v = ins; gen_lfo_v = lfos;
    {                                           /* the gate matrix: each source's gate, OR'd into its pinned columns */
        static const uint8_t CD[5] = {1, 2, 4, 8, 16};
        uint32_t r, gs = 0, gd = 0;
        for (g = 0; g < NGEN; g++) { gs |= (outs >> g & 1u) << (GS_GEN + g); gs |= (lfos >> g & 1u) << (GS_LFO + g); }
        for (k = 0; k < NFG; k++) gs |= (uint32_t)(fgs[k].eoc != 0) << (GS_EOC + k);
        for (k = 0; k < 2u; k++) { if (qpulse_v[k]) { qpulse_v[k]--; gs |= 1u << (GS_QNT + k); } }
        for (k = 0; k < 5u; k++) gs |= (uint32_t)((half_n / CD[k]) % 2u == 0u) << (GS_CLK + k);
        for (k = 0; k < 2u; k++) gs |= (uint32_t)(popc(G[k].row) > NC / 2u) << (GS_POP + k);
        gs |= sq_gate_v << GS_SEQ;              /* (the sequencers' step gates, as they were a block ago) */
        for (r = 0; r < NGS; r++) if (gs >> r & 1u) gd |= P.gmat[r];
        for (k = 0; k < 2u; k++) if ((gd >> (GD_GA + k) & 1u) && !(gdst_v >> (GD_GA + k) & 1u)) gstep_req_v[k]++;
        gsrc_v = gs; gdst_v = gd;
    }
    for (k = 0; k < NFG; k++) fg_step(k, gdst_v >> (GD_FG1 + k) & 1u);
    seq_block();
    for (k = 0; k < NSSG; k++) {                /* sample on the clock's rise; glide to it at SLEW */
        uint32_t c = gdst_v >> (GD_SH1 + k) & 1u;
        if (c && !ssg_prev[k]) {
            uint32_t in = P.ssg[k].input;
            isr_rnd ^= isr_rnd << 13; isr_rnd ^= isr_rnd >> 17; isr_rnd ^= isr_rnd << 5;
            ssg_held[k] = in == 0u || in > NSRC ? (int32_t)(isr_rnd >> 16) - 32768 : src_v[in - 1u];
            ssg_fired_v[k]++;
        }
        ssg_prev[k] = (uint8_t)c;
        if (!P.ssg[k].slew) ssg_out[k] = ssg_held[k];
        else ssg_out[k] += (ssg_held[k] - ssg_out[k]) * 64 / (64 + (int32_t)P.ssg[k].slew * P.ssg[k].slew / 4);
        src_v[S_SSG(k)] = ssg_out[k];
    }
    {                                           /* the quadrature LFO (as S6) */
        int32_t sn, cs;
        q_ph += LUT_QINC[cl((int32_t)P.qrate + qmod_v, 0, 127)];
        sn = sine(q_ph); cs = sine(q_ph + 0x40000000u);
        src_v[S_QUAD(0)] = sn; src_v[S_QUAD(1)] = -sn; src_v[S_QUAD(2)] = cs; src_v[S_QUAD(3)] = -cs;
    }
    src_v[S_POP(0)] = (int32_t)popc(G[0].row) * 1024;
    src_v[S_POP(1)] = (int32_t)popc(G[1].row) * 1024;
    for (g = 0; g < NGEN; g++) src_v[S_GEN(g)] = outs >> g & 1u ? 32767 : 0;
    for (s = 0; s < 2u; s++) {
        uint32_t gt = (gdst_v >> (GD_K1 + s) & 1u) || kman_hold_v[s], rise = gt && !kgprev[s], fall = !gt && kgprev[s];
        if (kman_req_v[s] != kman_seen[s]) { kman_seen[s] = kman_req_v[s]; rise = 1; }
        kgprev[s] = gt;
        kgate_v[s] = gt;
        if (kimp_v[s]) {
            if (rise) { tr_left[s] = P.tr[s].count ? P.tr[s].count : 1u; tr_i[s] = 0; tr_wait[s] = 0; }
            if (tr_left[s]) {
                if (tr_wait[s]) tr_wait[s]--;
                else {
                    strike_hit(s, trill_vel(s, tr_i[s], P.tr[s].count));
                    tr_i[s]++; tr_left[s]--;
                    tr_wait[s] = (uint32_t)P.tr[s].space * 1378u / 100u;   /* (10 ms a step) */
                    if (tr_wait[s]) tr_wait[s]--;
                }
            }
        } else {
            if (rise) { kheld_v[s] = 1; kvel_hold[s] = kvel_v[s]; kfire_v[s]++; }
            if (fall) kheld_v[s] = 0;
        }
    }
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
    logic_block();
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
                int32_t tt = (kimp_v[s] || !kheld_v[s]) ? 0 : (int32_t)(((int64_t)kamp_t[s][h] * kvel_hold[s]) >> 15) << 8;
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
static void src_name(char *s, uint32_t k);
static void lsrc_name(char *s, uint32_t id)
{
    if (id == 0u) { s_cat(s, "OFF"); return; }
    if (id == 1u) { s_cat(s, "CLK"); return; }
    if (id <= 11u) { s_cat(s, "/"); s_int(s, DIVS[id - 1u]); return; }
    if (id <= 43u) { s_cat(s, "A"); s_int(s, (int32_t)id - 11); return; }
    if (id <= 75u) { s_cat(s, "B"); s_int(s, (int32_t)id - 43); return; }
    if (id <= 77u) { s_cat(s, id == 76u ? "POP A" : "POP B"); return; }
    if (id <= 81u) { s_cat(s, "GEN"); s_int(s, (int32_t)id - 77); return; }
    s_cat(s, "EOC"); s_int(s, (int32_t)id - 81);
}
static void src_name(char *s, uint32_t k)       /* a matrix source */
{
    if (k < NFG) { s_cat(s, "FG"); s_int(s, (int32_t)k + 1); }
    else if (k < NFG + NSSG) { s_cat(s, "SSG"); s_int(s, (int32_t)(k - NFG) + 1); }
    else if (k < NFG + NSSG + 2u) s_cat(s, k == S_POP(0) ? "POP A" : "POP B");
    else if (k < S_QUAD(0)) { s_cat(s, "GEN"); s_int(s, (int32_t)(k - S_GEN(0)) + 1); }
    else if (k < S_SEQ(0)) { static const char *const QN[4] = {"SIN+", "SIN-", "COS+", "COS-"}; s_cat(s, QN[(k - S_QUAD(0)) & 3u]); }
    else { s_cat(s, "SEQ"); s_int(s, (int32_t)(k - S_SEQ(0)) + 1); }
}
static const char *const GS_NAME[NGS] = {"GEN1", "GEN2", "GEN3", "GEN4", "LFO1", "LFO2", "LFO3", "LFO4", "EOC1", "EOC2",
    "EOC3", "EOC4", "QNT A", "QNT B", "CLK", "CLK/2", "CLK/4", "CLK/8", "CLK16", "POP A", "POP B", "SEQ1", "SEQ2", "SEQ3"};
static const char *const GD_SHORT[NGD] = {"K1", "K2", "F1", "F2", "F3", "F4", "S1", "S2", "GA", "GB", "Q1", "Q2", "Q3", "R1", "R2", "R3"};
static const char *const GD_NAME[NGD] = {"STRIKE 1", "STRIKE 2", "FG1 TRIGGER", "FG2 TRIGGER", "FG3 TRIGGER", "FG4 TRIGGER",
    "S&H 1 CLOCK", "S&H 2 CLOCK", "GRID A STEP", "GRID B STEP", "SEQ 1 ADVANCE", "SEQ 2 ADVANCE", "SEQ 3 ADVANCE",
    "SEQ 1 RESET", "SEQ 2 RESET", "SEQ 3 RESET"};
static void gins(char *s, uint32_t d)           /* what is pinned into destination d */
{
    uint32_t r, n = 0;
    for (r = 0; r < NGS && n < 4u; r++) if (P.gmat[r] >> d & 1u) { if (n) s_cat(s, " "); s_cat(s, GS_NAME[r]); n++; }
    if (!n) s_cat(s, "-");
}
static const char *const DST_SHORT[NDST] = {"TLT", "STR", "LVL", "SPL", "FLA", "FLB", "PNA", "PNB", "PTA", "PTB",
    "1PT", "1VL", "1SH", "1LV", "2PT", "2VL", "2SH", "2LV", "MTA", "MTB", "SPA", "SPB", "F1R", "F2R", "F3R", "F4R", "QRT"};
static const char *const DST_NAME[NDST] = {"TILT", "STRETCH", "OUT LEVEL", "SPLIT POINT", "FILTER A", "FILTER B",
    "PAN A", "PAN B", "PITCH A", "PITCH B", "STRIKE 1 PARTIAL", "STRIKE 1 VELOCITY", "STRIKE 1 SHAPE", "STRIKE 1 LEVEL",
    "STRIKE 2 PARTIAL", "STRIKE 2 VELOCITY", "STRIKE 2 SHAPE", "STRIKE 2 LEVEL", "GRID A MUTATION", "GRID B MUTATION",
    "GRID A SPEED", "GRID B SPEED", "FG1 RATE", "FG2 RATE", "FG3 RATE", "FG4 RATE", "QUAD RATE"};
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
enum { PG_INDEX, PG_VOICE, PG_GRID, PG_STRIKE, PG_SPLIT, PG_LOGIC, PG_SEQ, PG_FG, PG_MOD, PG_GMAT, PG_FX, PG_GLOBAL, PG_PATCH, NPG };
static const char *const PG_NUM[NPG] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "X", "F", "G", "D"};
static const char *const PG_NAME[NPG] = {"INDEX", "VOICE", "GRIDS", "STRIKE", "SPLIT + FILTERS", "LOGIC", "SEQUENCERS",
                                         "FUNCTION GEN", "MOD MATRIX", "GATE MATRIX", "FX", "GLOBAL", "DISK"};
static const uint8_t PG_READY[NPG] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0};
static uint32_t page = PG_INDEX, sub, bank;     /* sub: OCT's choice (grid, strike, split part); bank: SEL's */
static uint32_t cpu_avg, cpu_peak, cpu_max;     /* 1/10 per cent of the 5.8 ms */
#define BUDGET_US 5805u

static uint32_t page_subs(uint32_t pg)
{
    if (pg == PG_GRID) return 2u;
    if (pg == PG_STRIKE) return 2u;
    if (pg == PG_LOGIC) return 4u;
    if (pg == PG_SEQ) return NSEQ;
    if (pg == PG_SPLIT || pg == PG_GLOBAL) return 3u;
    if (pg == PG_FG) return NFG + NSSG + 1u;    /* FG 1-4, S&H 1-2, the quad LFO */
    return 1u;
}
static uint32_t mrow, mcol, mtop, mleft;
static uint8_t scur[NSEQ];                      /* each sequencer's step under the cursor (page 7) */        /* the matrix's cursor (source, destination) and its window */
static uint32_t qsel;                           /* the global page's TUNING: the path SELECT picks */
static uint32_t page_banks(uint32_t pg, uint32_t sb)
{
    if (pg == PG_GRID) return 2u;
    if (pg == PG_STRIKE) return 3u;             /* SOUND, ENVELOPE, TRILL */
    if (pg == PG_SEQ) return 2u;                /* STEPS, PLAY */
    if (pg == PG_SPLIT) return sb ? 2u : 1u;
    return 1u;
}

/* ---- the knobs: what K1..K4 turn on each page and bank */
enum { F_INT, F_SLEW, F_TILT, F_STR, F_PCT, F_GMODE, F_GRATE, F_MUT, F_ONOFF, F_SHIFT, F_KMODE, F_PART, F_VEL, F_SHAPE,
       F_DEC, F_FALL, F_SER, F_LSRC, F_OP, F_ROUTE, F_SRULE, F_POINT, F_WIDTH, F_SOFT, F_PMODE, F_FREQ, F_RES, F_BW,
       F_PAN, F_SEMI, F_CENT, F_CLK, F_TEMPO, F_PHOS, F_BRIGHT, F_BITS, F_TUN, F_SCL, F_SROOT, F_COUNT, F_SPACE, F_TPAT,
       F_GOP, F_GMODE2, F_RATE, F_DUTY, F_FGT, F_CURVE, F_FMODE, F_SSGIN, F_DEPTH, F_QRATE, F_STEPN, F_SDIR, F_SADDR, F_NONE };
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
        if (!bk) { kn_set(0, "RULE", &g->rule, 0, 255, F_INT, 1);
                   kn_set(1, "RATE", (sb & 1u) && P.lock ? &P.g[0].rate : &g->rate, 0, 127, F_GRATE,
                          (!(sb & 1u) || P.lock) && P.clk_midi ? 16u : 2u);   /* (lockstep: B's turns A's; MIDI: a division a step) */
                   kn_set(2, "MUTATE", &g->mut, 0, 30, F_MUT, 1); kn_set(3, "SHARE", &P.gmode, 0, NGM - 1, F_GMODE, 1); }
        else { kn_set(0, "RESCUE", &g->rescue, 0, 1, F_ONOFF, 1); kn_set(1, "SEED", &g->dens, 5, 95, F_MUT, 5);
               kn_set(2, "SHIFT", &g->shift, 0, 31, F_SHIFT, 1); kn_set(3, "LOCK", &P.lock, 0, 1, F_ONOFF, 1); }
    } else if (pg == PG_STRIKE) {
        kpar_t *k = &P.k[sb & 1u];
        trill_t *t = &P.tr[sb & 1u];
        if (bk == 0u) { kn_set(0, "MODE", &k->mode, 0, NKM - 1, F_KMODE, 1); kn_set(1, "PARTIAL", &k->part, 1, 32, F_PART, 1);
                        kn_set(2, "VEL", &k->vel, 0, 127, F_VEL, 1); kn_set(3, "LEVEL", &k->level, 0, 127, F_PCT, 1); }
        else if (bk == 1u) { kn_set(0, KSH_NAME[k->mode & 3u], &k->shape, 0, 127, F_SHAPE, 1); kn_set(1, "DECAY", &k->decay, 0, 127, F_DEC, 1);
                             kn_set(2, "FALL", &k->fall, 0, 127, F_FALL, 1); kn_set(3, "SERIES", &k->series, 0, 2, F_SER, 1); }
        else { kn_set(0, "COUNT", &t->count, 1, 16, F_COUNT, 1); kn_set(1, "SPACE", &t->space, 1, 50, F_SPACE, 1);
               kn_set(2, "PATTERN", &t->pattern, 0, 2, F_TPAT, 1); kn_set(3, "MAX VEL", &t->max, 0, 127, F_VEL, 4); }
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
    } else if (pg == PG_LOGIC) {
        gen_t *q = &P.gen[sb & 3u];
        kn_set(0, "INPUT", &q->in, 0, NLS - 1, F_LSRC, 1); kn_set(1, "LOGIC", &q->op, 0, 7, F_GOP, 1);
        kn_set(2, "RATE", &q->rate, 0, 127, F_RATE, q->mode ? 12 : 2); kn_set(3, q->mode ? "LFO" : "DUTY", &q->duty, 1, 126, q->mode ? F_NONE : F_DUTY, 4);
        if (q->mode) kn_set(3, "", &dummy, 0, 0, F_NONE, 1);
    } else if (pg == PG_FG) {
        if (sb < NFG) { fgp_t *f = &P.fg[sb];
            kn_set(0, "RISE", &f->rise, 0, 127, F_FGT, 2); kn_set(1, "FALL", &f->fall, 0, 127, F_FGT, 2);
            kn_set(2, "CURVE", &f->curve, 0, 2, F_CURVE, 1); kn_set(3, "MODE", &f->mode, 0, 2, F_FMODE, 1); }
        else if (sb >= NFG + NSSG) kn_set(0, "RATE", &P.qrate, 0, 127, F_QRATE, 2);
        else { ssgp_t *q = &P.ssg[sb - NFG];
            kn_set(0, "INPUT", &q->input, 0, NSRC, F_SSGIN, 1); kn_set(1, "SLEW", &q->slew, 0, 127, F_INT, 4); }
    } else if (pg == PG_SEQ) {
        seq_t *q = &P.seq[sb % NSEQ];
        uint8_t *c = &scur[sb % NSEQ];
        if (!bk) { kn_set(0, "STEP", c, 0, 7, F_STEPN, 1); kn_set(1, "VALUE", &q->val[*c & 7u], 0, 127, F_VEL, 2);
                   kn_set(2, "GATE", &q->gate[*c & 7u], 0, 1, F_ONOFF, 1); kn_set(3, "LENGTH", &q->len, 1, 8, F_INT, 1); }
        else { kn_set(0, "DIR", &q->dir, 0, 2, F_SDIR, 1); kn_set(1, "SLEW", &q->slew, 0, 127, F_INT, 2);
               kn_set(2, "ADDR", &q->addr, 0, NSRC, F_SADDR, 1); kn_set(3, "CHAIN", &q->chain, 0, 1, F_ONOFF, 1); }
    } else if (pg == PG_MOD) {
        kn_set(2, "DEPTH", &P.dep[mrow][mcol], -63, 63, F_DEPTH, 1);
    } else if (pg == PG_GLOBAL) {
        if (sb == 0u) { kn_set(0, "TUNING", &P.tun, 0, NTUN - 1, F_TUN, 1); kn_set(1, "SCALE", &P.scl, 0, TUN_NSCL[P.tun % NTUN] - 1, F_SCL, 1);
                        kn_set(2, "ROOT", &P.sroot, 0, 11, F_SROOT, 1); kn_set(3, "GLIDE", &P.glide[qsel & 1u], 0, 127, F_INT, 4); }
        else if (sb == 1u) { kn_set(0, "PHOS", &gset.phos, 0, NPHOS - 1, F_PHOS, 1); kn_set(1, "BRIGHT", &gset.bright, 0, 10, F_BRIGHT, 1); }
        else { kn_set(0, "ENG VER", &P.engver, 1, 2, F_BITS, 1); kn_set(1, "CLOCK", &P.clk_midi, 0, 1, F_CLK, 1);
               }
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
    case F_GRATE: {                             /* a grid's clock: steps a second, a MIDI division, or GATE when pinned */
        uint32_t gi = sub & 1u, onA = !gi || P.lock;
        if (gated(gi)) s_cat(s, "GATE");
        else if (onA && P.clk_midi) s_cat(s, MIDI_DIVN[(v >> 4) & 7]);
        else { s_dec(s, Q_GRID_X100[v & 127], 100u, Q_GRID_X100[v & 127] < 1000u ? 2u : 1u); s_cat(s, "/S"); }
    } break;
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
    case F_ONOFF + 100: s_cat(s, v ? "FILTER" : "SUM"); break;
    case F_COUNT: if (v <= 1) s_cat(s, "OFF"); else { s_int(s, v); s_cat(s, "X"); } break;
    case F_SPACE: s_int(s, v * 10); s_cat(s, "MS"); break;
    case F_TPAT: s_cat(s, TP_NAME[v % 3]); break;
    case F_GOP: s_cat(s, OP_NAME[v & 7]); break;
    case F_RATE: {
        const gen_t *q = &P.gen[sub & 3u];
        if (q->mode) { s_cat(s, "/"); s_int(s, DIVS[v * 11 / 128]); }
        else { uint32_t hz = (uint32_t)(((uint64_t)LUT_GGINC[v & 127] * 137813u) >> 32); s_dec(s, hz, 100u, hz < 1000u ? 2u : 1u); s_cat(s, "HZ"); }
    } break;
    case F_DUTY: s_int(s, v * 100 / 128); s_cat(s, "%"); break;
    case F_FGT: { uint32_t ms = (16777216u / LUT_FGINC[v & 127]) * 1000u / 1378u;   /* (the slope's length) */
        if (ms < 1000u) { s_int(s, (int32_t)ms); s_cat(s, "MS"); } else { s_dec(s, ms / 10u, 100u, ms < 10000u ? 2u : 1u); s_cat(s, "S"); } } break;
    case F_CURVE: s_cat(s, CV_NAME[v % 3]); break;
    case F_STEPN: s_int(s, v + 1); break;
    case F_SDIR: { static const char *const DN[3] = {"FWD", "PEND", "RAND"}; s_cat(s, DN[v % 3]); } break;
    case F_SADDR: if (!v) s_cat(s, "CLOCK"); else src_name(s, (uint32_t)v - 1u); break;
    case F_QRATE: { uint32_t cs = LUT_QPER[v & 127];   /* a cycle, 1/100 s (as S6) */
        if (cs >= 1000u) s_int(s, (int32_t)((cs + 50u) / 100u)); else { s_int(s, (int32_t)(cs / 100u)); s_cat(s, "."); s_int(s, (int32_t)(cs / 10u % 10u)); }
        s_cat(s, "S"); } break;
    case F_FMODE: s_cat(s, FM_NAME[v % 3]); break;
    case F_SSGIN: if (!v) s_cat(s, "NOISE"); else src_name(s, (uint32_t)v - 1u); break;
    case F_DEPTH: if (!v) s_cat(s, "--"); else s_sdec(s, v, 1u, 0u); break;
    case F_BITS: s_cat(s, v == 1 ? "V1 8B" : "V2 12B"); break;
    case F_TUN: s_cat(s, TUN_NAME[v % NTUN]); break;
    case F_SCL: s_cat(s, SCL_NAME[TUN_SCL0[P.tun % NTUN] + (uint32_t)v]); break;
    case F_SROOT: { static const char *const NN[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"}; s_cat(s, NN[v % 12]); } break;
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
    else if (page == PG_STRIKE) { s_cat(s, "STRIKE "); s_int(s, (int32_t)sub + 1); }
    else if (page == PG_LOGIC) { s_cat(s, "LOGIC GEN "); s_int(s, (int32_t)sub + 1); }
    else if (page == PG_SEQ) { s_cat(s, "SEQ "); s_int(s, (int32_t)sub + 1); }
    else if (page == PG_FG) { if (sub < NFG) { s_cat(s, "FUNC GEN "); s_int(s, (int32_t)sub + 1); }
                              else if (sub < NFG + NSSG) { s_cat(s, "S&H "); s_int(s, (int32_t)(sub - NFG) + 1); }
                              else s_cat(s, "QUAD LFO"); }
    else if (page == PG_SPLIT) s_cat(s, sub == 0u ? "SPLIT" : sub == 1u ? "PATH A" : "PATH B");
    else if (page == PG_GLOBAL) s_cat(s, sub == 0u ? "GLOBAL: TUNING" : sub == 1u ? "GLOBAL: VDU" : "GLOBAL: ENGINE");
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
    s_cat(s, "  CLK "); if (P.clk_midi) s_cat(s, "MIDI"); else { s_dec(s, Q_GRID_X100[E.g[0].rate], 100u, 2u); s_cat(s, "/S"); }
    cv_text(4, 126, &FONT_S, s, PH_MID);
    s[0] = 0; s_cat(s, "LIVE "); s_int(s, (int32_t)live_v); s_cat(s, "  CPU "); s_dec(s, cpu_avg, 10u, 1u); s_cat(s, "%");
    s_cat(s, "  A/B");
    cv_text(4, 142, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("ALG ROOT  SEL F0  (CLOCK: GRIDS)");
    cv_out(BODY_Y);
}

static void lamp(int32_t x, int32_t y, uint32_t on);
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
    s[0] = 0; s_cat(s, sub ? "B" : "A"); s_cat(s, " POP "); s_int(s, (int32_t)popc(g->row)); s_cat(s, "/32");
    cv_text(4, 144, &FONT_S, s, PH_MID);
    if (g->rescue_t && fm1_ms - g->rescue_t < 900u) { s[0] = 0; s_cat(s, "RESCUE "); s_cat(s, g->why); cv_text(236 - text_w(&FONT_S, s), 144, &FONT_S, s, PH_HI); }
    else {                                      /* the clock: LOCK lit in lockstep; GATE when the gate matrix drives it */
        int32_t x = 236 - 4 * 8 - 14;
        if (gated(sub & 1u)) { x = 236 - 4 * 8 - 14 - 64; lamp(x + 64, 147, 1); cv_text(x + 78, 144, &FONT_S, "GATE", PH_HI); }
        lamp(x, 147, P.lock); cv_text(x + 14, 144, &FONT_S, "LOCK", P.lock ? PH_HI : PH_LO);
    }
    knob_row(0);
    footer("OCT A/B  SEL BANK  PLAY SEED");
    cv_out(BODY_Y);
}

/* ---- histories for the screens: one sample every 10 ms (the logic analyser, the FG and S&H traces) */
#define HISTN 200u
static uint8_t ghist[HISTN];                    /* bit g: generator g's output; bits 4..7: inputs; (LFOs kept apart) */
static uint8_t lhist[HISTN];                    /* bit g: generator g's LFO */
static int16_t fhist[NFG + NSSG][HISTN];
static uint8_t khist[2][HISTN];                 /* each strike's gate */
static uint32_t hhead_s;
static void hist_sample(void)
{
    uint32_t k, o = gen_out_v, in = gen_in_v;
    ghist[hhead_s] = (uint8_t)(o | in << 4);
    lhist[hhead_s] = (uint8_t)gen_lfo_v;
    for (k = 0; k < NFG + NSSG; k++) fhist[k][hhead_s] = (int16_t)(src_v[k] >> 1);
    for (k = 0; k < 2u; k++) khist[k][hhead_s] = (uint8_t)kgate_v[k];
    hhead_s = (hhead_s + 1u) % HISTN;
}
static void lamp(int32_t x, int32_t y, uint32_t on)
{
    if (on) cv_rect(x, y, 10, 10, PH_HI);
    else { cv_rect(x, y, 10, 1, PH_MID); cv_rect(x, y + 9, 10, 1, PH_MID); cv_rect(x, y, 1, 10, PH_MID); cv_rect(x + 9, y, 1, 10, PH_MID); }
}
static void trace_bits(int32_t x0, int32_t y, const uint8_t *h, uint32_t bit)   /* a logic line, oldest on the left */
{
    uint32_t i;
    int32_t plv = -1;
    for (i = 0; i < HISTN; i++) {
        int32_t lv = h[(hhead_s + i) % HISTN] >> bit & 1u, x = x0 + (int32_t)i;
        cv_pset(x, lv ? y : y + 9, i + 1u == HISTN ? PH_HI : PH_MID);
        if (plv >= 0 && lv != plv) cv_rect(x, y, 1, 10, PH_MID);
        plv = lv;
    }
}

static void trill_bars(uint32_t s_, int32_t y0, int32_t hgt)   /* COUNT hits SPACE apart, as tall as their velocities */
{
    uint32_t i, n = P.tr[s_].count < 1u ? 1u : P.tr[s_].count;
    int32_t span = (int32_t)(n - 1u) * P.tr[s_].space, lo = kvel_v[s_], hi = kmax_v[s_];
    cv_rect(8, y0 + hgt, 224, 1, PH_LO);
    for (i = 0; i < n; i++) {
        int32_t f, v, x, ht;
        if (n < 2u) f = 0;
        else if (P.tr[s_].pattern == TP_RISE) f = (int32_t)(i * 256u / (n - 1u));
        else if (P.tr[s_].pattern == TP_FALL) f = 256 - (int32_t)(i * 256u / (n - 1u));
        else f = 256 - iabs((int32_t)(i * 512u / (n - 1u)) - 256);
        v = lo + (hi - lo) * f / 256;
        x = n < 2u ? 118 : 12 + (int32_t)i * P.tr[s_].space * 212 / (span > 1 ? span : 1);
        ht = v * hgt / 32768;
        cv_rect(x, y0 + hgt - ht, 4, ht, PH_HI);
    }
}
static void draw_strike(void)
{
    uint32_t s_ = sub & 1u, h;
    const kpar_t *k = &P.k[s_];
    int32_t mx = 1;
    char s[40];
    cv_begin(240, BODY_H, PH_BG);
    if (bank == 2u) {                           /* TRILL: the hits as bars */
        uint32_t n = P.tr[s_].count < 1u ? 1u : P.tr[s_].count;
        s[0] = 0;
        if (n < 2u) s_cat(s, "TRILL OFF: ONE HIT A GATE");
        else { s_int(s, (int32_t)n); s_cat(s, " HITS IN "); s_int(s, (int32_t)(n - 1u) * P.tr[s_].space * 10); s_cat(s, " MS"); }
        cv_text(4, 4, &FONT_S, s, PH_MID);
        trill_bars(s_, 26, 64);
        cv_text(4, 96, &FONT_S, k->mode >= KM_BOW ? "BOW/BREATH: NO TRILL" : "FROM VEL UP TO MAX VEL", PH_LO);
    } else {
        for (h = 0; h < NP; h++) if (kshow_v[s_][h] > mx) mx = kshow_v[s_][h];
        if (mx < 2000) mx = 2000;
        for (h = 0; h < NP; h++) {              /* this strike's partials, ringing now; its members marked below */
            int32_t x = 24 + (int32_t)h * 3, v = kshow_v[s_][h], ht = (int32_t)isqrt32((uint32_t)(v * 1000 / mx) * 4900u / 1000u);
            if (ht > 0) cv_rect(x, 92 - ht, 2, ht, PH_HI);
            cv_rect(x, 96, 2, 2, kmember[s_][h] ? PH_MID : PH_SEL);
        }
        cv_text(4, 4, &FONT_S, KM_NAME[k->mode & 3u], PH_MID);
        s[0] = 0; s_cat(s, "H"); s_int(s, E.k[s_].part); cv_text(236 - text_w(&FONT_S, s), 4, &FONT_S, s, PH_MID);
    }
    lamp(4, 109, kgate_v[s_]);                  /* its gate (from the gate matrix): a lamp, the last 2 s */
    s[0] = 0; s_cat(s, "IN "); gins(s, GD_K1 + s_);
    cv_text(18, 106, &FONT_S, s, PH_MID);
    trace_bits(20, 126, khist[s_], 0);
    s[0] = 0; s_cat(s, P.route_pre ? "PRE" : s_ ? "POST>B" : "POST>A"); s_cat(s, k->filt ? " FILT" : " SUM");
    cv_text(236 - text_w(&FONT_S, s), 140, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("OCT 1/2 SEL BANK PLAY FIRE");
    cv_out(BODY_Y);
}

static void draw_logic(void)                    /* a generator: its input, its LFO, its output, as a logic analyser */
{
    uint32_t g = sub & 3u, in, l, o;
    const gen_t *q = &P.gen[g];
    char s[40];
    in = gen_in_v >> g & 1u; l = gen_lfo_v >> g & 1u; o = gen_out_v >> g & 1u;
    cv_begin(240, BODY_H, PH_BG);
    s[0] = 0; s_cat(s, "IN "); lsrc_name(s, q->in); s_cat(s, " "); s_cat(s, OP_NAME[q->op & 7u]); s_cat(s, " LFO");
    cv_text(4, 4, &FONT_S, s, PH_MID);
    s[0] = 0; s_cat(s, q->mode ? "CLOCKED" : "FREE");
    cv_text(236 - text_w(&FONT_S, s), 4, &FONT_S, s, PH_LO);
    cv_rect(0, 24, 240, 1, PH_LO);
    lamp(4, 38, in);  cv_text(18, 35, &FONT_S, "IN", in ? PH_HI : PH_LO);   trace_bits(36, 38, ghist, 4u + g);
    lamp(4, 70, l);   cv_text(18, 67, &FONT_S, "LF", l ? PH_HI : PH_LO);    trace_bits(36, 70, lhist, g);
    lamp(4, 102, o);  cv_text(18, 99, &FONT_S, "OU", o ? PH_HI : PH_LO);    trace_bits(36, 102, ghist, g);
    s[0] = 0; s_cat(s, "PINNED TO:");
    {                                           /* where the gate matrix sends this generator (and its LFO) */
        uint32_t d, any = 0;
        for (d = 0; d < NGD; d++) if ((P.gmat[GS_GEN + g] | P.gmat[GS_LFO + g]) >> d & 1u) { s_cat(s, " "); s_cat(s, GD_SHORT[d]); any = 1; }
        if (!any) s_cat(s, " -");
    }
    cv_text(4, 130, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("OCT GEN  ALG FREE/CLOCK");
    cv_out(BODY_Y);
}

static void draw_quad(void)                     /* the quadrature LFO: a dot round a circle, four bipolar bars */
{
    uint32_t i, k, d, n;
    int32_t cx = 60, cy = 74, r = 48;
    char s[40];
    static const char *const QN[4] = {"SIN+", "SIN-", "COS+", "COS-"};
    cv_begin(240, BODY_H, PH_BG);
    for (i = 0; i < 96u; i++)
        cv_pset(cx + ((sine(i * 0x02AAAAABu + 0x40000000u) * r) >> 15), cy - ((sine(i * 0x02AAAAABu) * r) >> 15), PH_LO);
    cv_rect(cx - r, cy, 2 * r + 1, 1, PH_SEL); cv_rect(cx, cy - r, 1, 2 * r + 1, PH_SEL);
    cv_rect(cx + ((src_v[S_QUAD(2)] * r) >> 15) - 3, cy - ((src_v[S_QUAD(0)] * r) >> 15) - 3, 7, 7, PH_HI);
    for (k = 0; k < 4u; k++) {                  /* SIN+ SIN- COS+ COS-, bipolar bars, and where each goes */
        int32_t y = 30 + (int32_t)k * 24, v = (src_v[S_QUAD(k)] * 42) >> 15;
        cv_text(116, y - 4, &FONT_S, QN[k], PH_MID);
        cv_rect(193, y - 2, 1, 11, PH_LO);
        if (v >= 0) cv_rect(194, y, v, 7, PH_MID); else cv_rect(193 + v, y, -v, 7, PH_MID);
    }
    s[0] = 0; s_cat(s, "TO:");
    for (k = 0, n = 0; k < 4u; k++)
        for (d = 0; d < NDST && n < 6u; d++) if (P.dep[S_QUAD(k)][d]) { s_cat(s, " "); s_cat(s, DST_SHORT[d]); n++; }
    if (!n) s_cat(s, " -");
    cv_text(4, 134, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("OCT FG/S&H/QUAD");
    cv_out(BODY_Y);
}
static void draw_fg(void)                       /* a function generator or an S&H: its trace, its settings */
{
    uint32_t k = sub, i;
    char s[40];
    int32_t bip = k >= NFG;                     /* the S&Hs are bipolar */
    cv_begin(240, BODY_H, PH_BG);
    s[0] = 0;
    if (!bip) { s_cat(s, FM_NAME[P.fg[k].mode % 3u]); s_cat(s, " < "); gins(s, GD_FG1 + k); }
    else { s_cat(s, "CLK < "); gins(s, GD_SH1 + (k - NFG)); }
    cv_text(4, 4, &FONT_S, s, PH_MID);
    cv_rect(20, 24, 1, 112, PH_LO);
    if (bip) for (i = 0; i < 200u; i += 4u) cv_pset(20 + (int32_t)i, 80, PH_LO);
    else cv_rect(20, 135, 200, 1, PH_LO);
    for (i = 0; i < HISTN; i++) {               /* the trace, oldest on the left */
        int32_t v = fhist[k][(hhead_s + i) % HISTN], y = bip ? 80 - v * 54 / 16384 : 134 - v * 108 / 16384;
        cv_rect(20 + (int32_t)i, y, 1, 2, i + 1u == HISTN ? PH_HI : PH_MID);
    }
    {                                           /* who it modulates: its pins */
        uint32_t d, n = 0;
        s[0] = 0; s_cat(s, "TO:");
        for (d = 0; d < NDST && n < 5u; d++) if (P.dep[k][d]) { s_cat(s, " "); s_cat(s, DST_SHORT[d]); n++; }
        if (!n) s_cat(s, " -");
        cv_text(4, 142, &FONT_S, s, PH_LO);
    }
    knob_row(0);
    footer("OCT FG/S&H  GATES: GATE MATRIX");
    cv_out(BODY_Y);
}

static void draw_seq(void)                      /* a sequencer: its steps as bars, the one playing lit, the output's glide */
{
    uint32_t n = sub % NSEQ, i, len = P.seq[n].len < 1u ? 1u : P.seq[n].len, st = sq_step_v[n];
    const seq_t *q = &P.seq[n];
    char s[40];
    int32_t out = src_v[S_SEQ(n)];
    cv_begin(240, BODY_H, PH_BG);
    for (i = 0; i < 8u; i++) {
        int32_t x = 8 + (int32_t)i * 24, ht = q->val[i] * 88 / 127, in = i < len;
        if (i == scur[n]) cv_rect(x - 3, 4, 24, 116, PH_SEL);            /* the cursor */
        if (in) cv_rect(x, 104 - ht, 18, ht + 1, i == st ? PH_HI : PH_MID);
        else { int32_t y; for (y = 104 - ht; y <= 104; y += 3) cv_rect(x, y, 18, 1, PH_LO); }
        if (q->gate[i]) cv_rect(x + 5, 110, 8, 6, in ? (i == st && (sq_gate_v >> n & 1u) ? PH_HI : PH_MID) : PH_LO);
        else cv_rect(x + 8, 112, 2, 2, PH_LO);
    }
    cv_rect(206, 16, 1, 89, PH_LO);             /* the output, gliding */
    cv_rect(212, 104 - out * 88 / 32768, 18, 3, PH_HI);
    s[0] = 0;
    if (q->addr) { s_cat(s, "SWEPT BY "); src_name(s, q->addr - 1u); }
    else { s_cat(s, "ADV < "); gins(s, GD_SQA1 + n); }
    cv_text(4, 122, &FONT_S, s, PH_MID);
    s[0] = 0;
    if (!q->addr) { s_cat(s, "RST < "); gins(s, GD_SQR1 + n); }
    if (q->chain && n) { s_cat(s, q->addr ? "" : "  "); s_cat(s, "CHAIN"); }
    cv_text(4, 140, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("OCT SEQ SEL BANK PLAY GATE");
    cv_out(BODY_Y);
}
static uint32_t grow, gcol, gtop, gleft;       /* the gate matrix's cursor and window */
#define GCOLS 10u
static void draw_gmat(void)                     /* sources down, destinations across; a pin lights while its gate is high */
{
    uint32_t r, c, gs = gsrc_v, gd = gdst_v;
    char s[40];
    if (grow < gtop) gtop = grow;
    if (grow >= gtop + 8u) gtop = grow - 7u;
    if (gcol < gleft) gleft = gcol;
    if (gcol >= gleft + GCOLS) gleft = gcol - GCOLS + 1u;
    cv_begin(240, BODY_H, PH_BG);
    for (c = gleft; c < gleft + GCOLS && c < NGD; c++) {   /* the column names, two lines: K / 1, F / 3, G / A ... */
        int32_t x = 64 + (int32_t)(c - gleft) * 17;
        char t[2] = {0, 0};
        uint16_t col = c == gcol ? PH_HI : (gd >> c & 1u) ? PH_MID : PH_LO;
        t[0] = GD_SHORT[c][0]; cv_text(x, 0, &FONT_S, t, col);
        t[0] = GD_SHORT[c][1]; cv_text(x, 15, &FONT_S, t, col);
    }
    cv_rect(0, 33, 240, 1, PH_LO);
    for (r = 0; r < 8u && gtop + r < NGS; r++) {
        uint32_t i = gtop + r, on = gs >> i & 1u;
        int32_t y = 36 + (int32_t)r * 15;
        cv_text(2, y, &FONT_S, GS_NAME[i], i == grow ? PH_HI : on ? PH_MID : PH_LO);
        for (c = gleft; c < gleft + GCOLS && c < NGD; c++) {
            int32_t x = 62 + (int32_t)(c - gleft) * 17;
            uint32_t pin = P.gmat[i] >> c & 1u, cur = i == grow && c == gcol;
            if (cur) cv_rect(x, y - 1, 15, 15, PH_SEL);
            if (pin) cv_rect(x + 3, y + 2, 9, 9, on ? PH_HI : PH_MID);
            else cv_rect(x + 7, y + 6, 2, 2, cur ? PH_HI : PH_LO);
        }
    }
    s[0] = 0; s_cat(s, GS_NAME[grow]); s_cat(s, " > "); s_cat(s, GD_NAME[gcol]);
    cv_text(2, 162, &FONT_S, s, PH_HI);
    s[0] = 0; s_cat(s, P.gmat[grow] >> gcol & 1u ? "PIN IN" : "NO PIN"); s_cat(s, "  ("); s_int(s, (int32_t)grow + 1); s_cat(s, "/24)");
    cv_text(2, 180, &FONT_S, s, PH_MID);
    footer("K1 SRC K2 DST PLAY PIN  OR'D");
    cv_out(BODY_Y);
}

#define MROWS 8u
#define MCOLS 6u
static void draw_mod(void)                      /* the pin grid, EMS style: sources down, destinations across */
{
    uint32_t r, c;
    char s[40];
    if (mrow < mtop) mtop = mrow;
    if (mrow >= mtop + MROWS) mtop = mrow - MROWS + 1u;
    if (mcol < mleft) mleft = mcol;
    if (mcol >= mleft + MCOLS) mleft = mcol - MCOLS + 1u;
    cv_begin(240, BODY_H, PH_BG);
    for (c = 0; c < MCOLS && mleft + c < NDST; c++)
        cv_text(64 + (int32_t)c * 29, 2, &FONT_S, DST_SHORT[mleft + c], mleft + c == mcol ? PH_HI : PH_LO);
    cv_rect(0, 19, 240, 1, PH_LO);
    for (r = 0; r < MROWS && mtop + r < NSRC; r++) {
        uint32_t k = mtop + r;
        int32_t y = 22 + (int32_t)r * 17, lv = src_v[k] < 0 ? -src_v[k] : src_v[k];
        s[0] = 0; src_name(s, k);
        cv_text(2, y, &FONT_S, s, k == mrow ? PH_HI : PH_MID);
        cv_rect(54, y + 14 - lv * 13 / 32768, 3, lv * 13 / 32768 + 1, PH_MID);   /* the source's level, live */
        for (c = 0; c < MCOLS && mleft + c < NDST; c++) {
            int32_t x = 64 + (int32_t)c * 29, d = P.dep[k][mleft + c];
            uint32_t cur = k == mrow && mleft + c == mcol;
            if (cur) cv_rect(x - 2, y - 1, 28, 16, PH_SEL);
            if (d) { s[0] = 0; s_int(s, d); cv_text(x + (d < 0 ? 0 : 4), y, &FONT_S, s, cur ? PH_HI : PH_MID); }
            else cv_rect(x + 10, y + 7, 3, 2, cur ? PH_HI : PH_LO);
        }
    }
    s[0] = 0; src_name(s, mrow); s_cat(s, " > "); s_cat(s, DST_NAME[mcol]);
    cv_text(2, 160, &FONT_S, s, PH_HI);
    s[0] = 0; s_cat(s, "DEPTH "); if (P.dep[mrow][mcol]) s_sdec(s, P.dep[mrow][mcol], 1u, 0u); else s_cat(s, "--");
    cv_text(2, 178, &FONT_S, s, PH_MID);
    footer("K1 SRC K2 DST K3 AMT PLAY PIN");
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

static const char *const PHOS_NAME[NPHOS] = {"GREEN P1", "AQUA", "AMBER P3", "WHITE P4"};
static void draw_tuning(void)                   /* GLOBAL: TUNING, as Arcturus S6: the period as a wheel, the paths on it */
{
    static const char *const NOTEN[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    uint32_t t = P.tun % NTUN, n = TUN_N[t], j, o, sc = TUN_SCL0[t] + P.scl;
    int32_t per = TUN_PERIOD[t], cx = 56, cy = 112, r = 46;
    const uint16_t *st = &TUN_STEPS[TUN_OFF[t]];
    uint32_t lo = SCL_LO[sc], hi = SCL_HI[sc], turn = 0xFFFFFFFFu / (uint32_t)per;
    char s[32];
    cv_begin(240, BODY_H, PH_BG);
    cv_text(4, 2, &FONT_S, "TUNING", PH_LO);  cv_text(68, 2, &FONT_S, TUN_NAME[t], PH_HI);
    cv_text(4, 20, &FONT_S, "SCALE", PH_LO);  cv_text(68, 20, &FONT_S, SCL_NAME[sc], PH_HI);
    cv_text(4, 38, &FONT_S, "ROOT", PH_LO);   cv_text(68, 38, &FONT_S, NOTEN[P.sroot % 12u], PH_HI);
    cv_rect(0, 58, 240, 1, PH_LO);
    for (j = 0; j < n; j++) {                   /* a tick a step, the scale's long and bright */
        uint32_t a = st[j] * turn;
        int32_t sx = sine(a), cy0 = sine(a + 0x40000000u), in = (j < 32u ? lo >> j & 1u : hi >> (j - 32u) & 1u);
        int32_t r0 = in ? r - 10 : r - 4;
        cv_line(cx + (sx * r0 >> 15), cy - (cy0 * r0 >> 15), cx + (sx * r >> 15), cy - (cy0 * r >> 15), in ? PH_HI : PH_LO);
    }
    for (o = 0; o < 2u; o++) {                  /* each path's pitch on the wheel: A the outer ring, B the inner */
        int32_t c = (int32_t)P.root * 256 + off_cur[o] - (int32_t)P.sroot * 256, rem = c % per, rr = r - 16 - (int32_t)o * 9;
        uint32_t a;
        if (rem < 0) rem += per;
        a = (uint32_t)rem * turn;
        cv_rect(cx + (sine(a) * rr >> 15) - 2, cy - (sine(a + 0x40000000u) * rr >> 15) - 2, 5, 5, P.qon[o] ? PH_HI : PH_LO);
    }
    for (o = 0; o < 2u; o++) {
        int32_t y = 70 + (int32_t)o * 22;
        s[0] = 0;
        s_cat(s, o ? "PATH B" : "PATH A");
        s_cat(s, P.qon[o] ? " ON " : " OFF");
        if (P.qon[o]) {
            s_cat(s, " ");
            if (n == 12u) { int32_t nt = ((int32_t)P.root * 256 + off_cur[o] + 128) >> 8; s_cat(s, NOTEN[nt % 12]); s_int(s, nt / 12 - 1); }
            else { s_cat(s, "#"); s_int(s, off_step[o]); }
        }
        if (o == qsel) cv_rect(108, y - 1, 132, 18, PH_SEL);
        cv_text(112, y, &FONT_S, s, o == qsel ? PH_HI : P.qon[o] ? PH_MID : PH_LO);
    }
    s[0] = 0; s_cat(s, "GLIDE "); s_int(s, P.glide[qsel & 1u]);
    cv_text(112, 120, &FONT_S, s, PH_MID);
    cv_text(4, 182, &FONT_S, "K1 TUNE K2 SCALE K3 ROOT", PH_LO);
    cv_text(4, 200, &FONT_S, "SELECT PATH K4 GLIDE PLAY Q", PH_LO);
    cv_out(BODY_Y);
}
static void draw_vdu(void)                      /* GLOBAL: VDU, as Arcturus S6: phosphor, brightness, a test card */
{
    char s[24] = "";
    uint32_t i;
    cv_begin(240, BODY_H, PH_BG);
    cv_text(4, 2, &FONT_S, "PHOSPHOR", PH_LO);
    cv_text(108, 2, &FONT_S, PHOS_NAME[gset.phos % NPHOS], PH_HI);
    cv_text(4, 20, &FONT_S, "BRIGHTNESS", PH_LO);
    s_int(s, gset.bright);
    cv_text(108, 20, &FONT_S, s, PH_HI);
    for (i = 0; i <= 10u; i++) cv_rect(140 + (int32_t)i * 9, 24, 6, 8, i <= gset.bright ? PH_MID : PH_LO);
    cv_rect(0, 39, 240, 1, PH_LO);
    cv_rect(6, 46, 228, 1, PH_MID); cv_rect(6, 194, 228, 1, PH_MID);
    cv_rect(6, 46, 1, 149, PH_MID); cv_rect(233, 46, 1, 149, PH_MID);
    for (i = 1; i < 8u; i++) {
        uint32_t k;
        for (k = 48; k < 194u; k += 3u) cv_pset(6 + (int32_t)i * 29, (int32_t)k, PH_LO);
        for (k = 8; k < 233u; k += 3u) if (i < 6u) cv_pset((int32_t)k, 46 + (int32_t)i * 25, PH_LO);
    }
    for (i = 0; i < 128u; i++)
        cv_pset(120 + ((sine(i << 25) * 52) >> 15), 120 - ((sine((i << 25) + 0x40000000u) * 52) >> 15), PH_HI);
    for (i = 0; i < 8u; i++) cv_rect(24 + (int32_t)i * 24, 104, 20, 32, RGB(0, 32 + i * 31, 0));
    for (i = 0; i < 192u; i++) cv_pset(24 + (int32_t)i, 160 - ((sine(i * 0x05555555u) * 14) >> 15), PH_HI);
    cv_text(2, 200, &FONT_S, "K1 PHOSPHOR  K2 BRIGHTNESS", PH_LO);
    cv_out(BODY_Y);
}
static void draw_engine(void)                   /* GLOBAL: ENGINE: the hardware revision, the clock, the CPU */
{
    char s[40];
    int32_t wa, wp;
    cv_begin(240, BODY_H, PH_BG);
    s[0] = 0; s_cat(s, "ENGINE VER: "); s_cat(s, P.engver == 1u ? "V1 (8 BIT)" : "V2 (12 BIT)");
    cv_text(4, 2, &FONT_S, s, PH_HI);
    cv_text(4, 20, &FONT_S, P.engver == 1u ? "256 LEVELS, 22.05 KHZ" : "4096 LEVELS, 22.05 KHZ", PH_LO);
    cv_rect(0, 40, 240, 1, PH_LO);
    cv_text(4, 46, &FONT_S, "AUDIO INTERRUPT", PH_MID);
    s[0] = 0; s_cat(s, "AVG "); s_dec(s, cpu_avg, 10u, 1u); s_cat(s, "%  PEAK "); s_dec(s, cpu_peak, 10u, 1u); s_cat(s, "%");
    cv_text(4, 64, &FONT_S, s, PH_HI);
    s[0] = 0; s_cat(s, "MAX "); s_dec(s, cpu_max, 10u, 1u); s_cat(s, "%  LIVE "); s_int(s, (int32_t)live_v); s_cat(s, "/64");
    cv_text(4, 82, &FONT_S, s, PH_MID);
    wa = (int32_t)(cpu_avg > 1000u ? 1000u : cpu_avg) * 232 / 1000;
    wp = (int32_t)(cpu_peak > 1000u ? 1000u : cpu_peak) * 232 / 1000;
    cv_rect(4, 108, 232, 1, PH_LO);
    cv_rect(4, 104, wa, 8, PH_MID);
    cv_rect(4 + wp, 100, 2, 16, PH_HI);
    cv_rect(120, 100, 1, 16, PH_LO);
    cv_text(4, 122, &FONT_S, "THE LINE: HALF THE BUDGET", PH_LO);
    s[0] = 0; s_cat(s, "QUORUM "); s_cat(s, Q_VERSION); s_cat(s, " (STAGE 1)");
    cv_text(4, 140, &FONT_S, s, PH_LO);
    knob_row(0);
    footer("OCT PART  PLAY: RESET MAX");
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
static uint32_t hdr_force;                      /* the header must be redrawn (a new phosphor or brightness) */
static void after_change(uint32_t pg, uint32_t sb, uint32_t knob, uint32_t bk)
{
    dirty_partials = 1;
    (void)bk;
    if (pg == PG_GLOBAL && sb == 1u) { crt_build(); hdr_force = 1; }   /* (the bulge redraws the rows above the body,
                                                                           not the header's top: redraw it, all of it) */
    if (pg == PG_GLOBAL && sb == 2u && knob == 0u) cpu_max = 0;   /* (a new engine: a new worst case) */
    if (pg == PG_GLOBAL && sb == 2u && knob >= 1u) t_half = fm1_ms;
    if (pg == PG_GRID && knob == 3u && bk == 1u) { t_halfb = t_half; half_b = half_n; }   /* (lockstep on or off: B in phase with A) */
    if (pg == PG_GLOBAL && sb == 0u && knob == 0u) P.scl = 0;    /* (a new tuning: its first scale) */
}
static void app_main(void)
{
    uint32_t i, b, rel, icur = PG_VOICE, t_draw = 0, t_cpu = 0, t_batt = 0, t_push = 0, t_hist = 0, prev_keys = 0, prev_up = 0, hdr = 1;
    hdr_force = 0;
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
    default_pins();
    E = P;
    skip = boot_wait(250);
    for (i = 1; i <= 6u && !skip; i++) { draw_boot(i); skip = boot_wait(i == 6u ? 900u : 250u); }
    mod_apply();
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
            if (b & (1u << BTN_LFO)) np = shift ? PG_LOGIC : PG_GRID;
            if (b & (1u << BTN_ENV)) np = shift ? PG_FG : PG_STRIKE;
            if (b & (1u << BTN_SEQ)) np = PG_SEQ;
            if (b & (1u << BTN_ARP)) np = shift ? PG_GMAT : PG_MOD;
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
            if (page == PG_GLOBAL && sub == 2u) cpu_max = 0;
            if (page == PG_GLOBAL && sub == 0u) { P.qon[qsel & 1u] ^= 1u; dirty_partials = 1; }
            if (page == PG_GMAT) P.gmat[grow] ^= (uint16_t)(1u << gcol);   /* a pin in / out */
            if (page == PG_SEQ) { uint32_t n = sub % NSEQ; P.seq[n].gate[scur[n] & 7u] ^= 1u; }   /* the step's gate */
            if (page == PG_STRIKE) {            /* fire it by hand (Bow / Breath: held 1.2 s) */
                uint32_t s_ = sub & 1u;
                if (P.k[s_].mode >= KM_BOW) { kman_until[s_] = fm1_ms + 1200u; kman_hold_v[s_] = 1; }
                else kman_req_v[s_]++;
            }
            if (page == PG_MOD) { P.dep[mrow][mcol] = P.dep[mrow][mcol] ? 0 : 32; }   /* a pin in / out */
        }
        if (page == PG_GMAT) {                  /* K1 the source, K2 the destination */
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) grow = (uint32_t)cl((int32_t)grow + e, 0, NGS - 1);
            if ((e = fm1_enc_take(ENC_KNOB2)) != 0) gcol = (uint32_t)cl((int32_t)gcol + e, 0, NGD - 1);
            if ((e = fm1_enc_take(ENC_SELECT)) != 0) grow = (uint32_t)cl((int32_t)grow + e, 0, NGS - 1);
            if ((e = fm1_enc_take(ENC_PRESETS)) != 0) gcol = (uint32_t)cl((int32_t)gcol + e, 0, NGD - 1);
        }
        if (page == PG_MOD) {                   /* K1 the source, K2 the destination; K3 the depth */
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) mrow = (uint32_t)cl((int32_t)mrow + e, 0, NSRC - 1);
            if ((e = fm1_enc_take(ENC_KNOB2)) != 0) mcol = (uint32_t)cl((int32_t)mcol + e, 0, NDST - 1);
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
            if (page == PG_LOGIC) P.gen[sub & 3u].mode = e > 0;
            if (page == PG_STRIKE) P.k[sub & 1u].filt = e > 0;   /* with the grids: sum / filter */
            if (page == PG_SPLIT) P.sn = (uint8_t)cl((int32_t)P.sn + e, 2, 8);
            dirty_partials = 1;
        }
        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) {
            (void)0;
            if (page == PG_STRIKE) { P.route_pre = e > 0; dirty_partials = 1; }
            if (page == PG_MOD) mcol = (uint32_t)cl((int32_t)mcol + e, 0, NDST - 1);
        }
        if (page == PG_GLOBAL && sub == 0u) { if ((e = fm1_enc_take(ENC_SELECT)) != 0) qsel = e > 0 ? 1u : 0u; }
        else if (page == PG_MOD) { if ((e = fm1_enc_take(ENC_SELECT)) != 0) mrow = (uint32_t)cl((int32_t)mrow + e, 0, NSRC - 1); }
        else if (page != PG_INDEX) (void)fm1_enc_take(ENC_SELECT);
        keys = fm1_in.notes;
        down = keys & ~prev_keys;
        prev_keys = keys;
        for (i = 0; i < 2u; i++) {              /* the top two keys: strike 1 and 2 by hand, on any page */
            uint32_t bit = 1u << (25u + i);
            if (down & bit) {
                if (P.k[i].mode >= KM_BOW) { kman_hold_v[i] = 1; kman_until[i] = fm1_ms + 0x7FFFFFFFu; }   /* held while the key is */
                else kman_req_v[i]++;
            }
            if ((prev_up & bit) && (keys & bit) == 0u && P.k[i].mode >= KM_BOW) kman_hold_v[i] = 0;
        }
        prev_up = keys;
        down &= (1u << 25u) - 1u;
        if (down) {                             /* the lowest key pressed (below the top two): the root, F1 .. F3 */
            for (i = 0; i < 25u; i++) if ((down >> i) & 1u) break;
            P.root = (uint8_t)(29u + i);
            dirty_partials = 1;
        }

        if (offsets_update()) dirty_partials = 1;
        if ((dirty_partials || any_pin) && fm1_ms - t_push >= 8u) { dirty_partials = 0; t_push = fm1_ms; mod_apply(); push_partials(); }
        if (fm1_ms - t_hist >= 10u) { t_hist = fm1_ms; hist_sample(); }
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
            if (hdr || hdr_force) { hdr = 0; hdr_force = 0; draw_header(); }   /* (a new phosphor: the header too) */
            switch (page) {
            case PG_INDEX: draw_index(icur); break;
            case PG_VOICE: draw_voice(); break;
            case PG_GRID: draw_grid(); break;
            case PG_STRIKE: draw_strike(); break;
            case PG_GMAT: draw_gmat(); break;
            case PG_SEQ: draw_seq(); break;
            case PG_LOGIC: draw_logic(); break;
            case PG_FG: if (sub >= NFG + NSSG) draw_quad(); else draw_fg(); break;
            case PG_MOD: draw_mod(); break;
            case PG_SPLIT: draw_split(); break;
            case PG_GLOBAL: if (sub == 0u) draw_tuning(); else if (sub == 1u) draw_vdu(); else draw_engine(); break;
            default: draw_soon(); break;
            }
            row = G[page == PG_GRID ? sub & 1u : 0u].row;   /* the keys: the cells (grid A, or the grid shown) */
            for (i = 0; i < 25u; i++) fm1_led_key(KEY_LED(i), (row >> i) & 1u);
            for (i = 0; i < 2u; i++) {          /* the strike keys: a glow at rest, full while the strike's gate is high */
                uint32_t on = kgate_v[i] || ((fm1_in.notes >> (25u + i)) & 1u);
                fm1_led_key(KEY_LED(25u + i), (int)on);
                if (!on) fm1_led_key_glow(KEY_LED(25u + i), 1);
            }
            fm1_led_key(BTN_SEL, shift || (page == PG_INDEX && blink) || (page_banks(page, sub) > 1u && blink));
            fm1_led_key(BTN_OCT_DOWN, page_subs(page) > 1u && blink);
            fm1_led_key(BTN_OCT_UP, page_subs(page) > 1u && !blink);
            fm1_led_key(BTN_HOME, page != PG_INDEX && blink);
            fm1_led_key(BTN_PLAY, (page == PG_STRIKE && (kgate_v[sub & 1u] || blink)) || (page == PG_GRID && blink)
                                  || (page == PG_GMAT && ((P.gmat[grow] >> gcol & 1u) || blink))
                                  || (page == PG_SEQ && (P.seq[sub % NSEQ].gate[scur[sub % NSEQ] & 7u] || blink))
                                  || (page == PG_MOD && (P.dep[mrow][mcol] || blink))
                                  || (page == PG_GLOBAL && sub == 0u && (P.qon[qsel & 1u] || blink)) || (page == PG_GLOBAL && sub == 2u && blink));
        }
    }
}
