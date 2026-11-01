/* SPDX-License-Identifier: GPL-3.0-only */
/* The reverb: a plate in the manner of Jon Dattorro's ("Effect Design, Part 1", JAES 1997), at a low sample rate,
 * in integers, early-digital style: 22 kHz or 11 kHz, the output held between its samples (the grit), the tank's
 * words optionally crushed to fewer bits. Its own output can be fed back into its input through a soft saturator:
 * at most it rings, as a Midiverb pushed to about two o'clock, never more.
 *
 * Included by dinghy.c after sine() and cl(). Runs in the audio interrupt: verb_run() takes a block of mono input
 * (the oscillators' sends) and adds its stereo return to the output. */

#define V_NAP 4                                 /* the input diffusers */
static const uint16_t V_AP_LEN[V_NAP] = {142, 107, 379, 277};
static const int16_t V_AP_G[V_NAP] = {24576, 24576, 20480, 20480};   /* 0.75, 0.75, 0.625, 0.625 */
/* the tank, at Dattorro's 29761 Hz lengths (here they run slower, so the plate is bigger) */
#define V_MAPL 672u
#define V_MAPR 908u
#define V_EXC 16u                               /* the modulated allpasses' excursion */
#define V_D1L 4453u
#define V_APL 1800u
#define V_D2L 3720u
#define V_D1R 4217u
#define V_APR 2656u
#define V_D2R 3163u
#define V_PRE 4096u                             /* the pre-delay, up to ~190 ms at 22 kHz */

typedef struct { int16_t *b; uint32_t len, w; } vline_t;

static int16_t v_mem[142 + 107 + 379 + 277 + (V_MAPL + V_EXC + 2) + (V_MAPR + V_EXC + 2) + V_D1L + V_APL + V_D2L
                     + V_D1R + V_APR + V_D2R + V_PRE] __attribute__((section(".pool")));
static vline_t v_ap[V_NAP], v_mapl, v_mapr, v_d1l, v_apl, v_d2l, v_d1r, v_apr, v_d2r, v_pre;

typedef struct {                                /* set by the main loop (knob units 0..127) */
    volatile uint8_t size, decay, damp, pre, mod, crush, ret, fb, rate, freeze;
} verb_par_t;
static verb_par_t vp;
static volatile int32_t v_level;                /* the return's level now, for the screen */
static volatile uint32_t v_gate_freeze;         /* the gate matrix's FREEZE */
static volatile int32_t eng_mask = ~15;         /* the engine's word length: ~15 = 12 bits, ~255 = 8 (on 16) */
static volatile uint32_t v_ready;               /* set by verb_init: the audio interrupt runs before app_main does */
static int32_t v_damp_l, v_damp_r, v_bw, v_last_l, v_last_r, v_out_l, v_out_r, v_fb;
static uint32_t v_lfo, v_phase;

static void vline_init(vline_t *l, int16_t **mem, uint32_t len)
{
    l->b = *mem;
    l->len = len;
    l->w = 0;
    *mem += len;
}
static void verb_init(void)
{
    int16_t *m = v_mem;
    uint32_t i;
    for (i = 0; i < V_NAP; i++) vline_init(&v_ap[i], &m, V_AP_LEN[i]);
    vline_init(&v_mapl, &m, V_MAPL + V_EXC + 2u);
    vline_init(&v_mapr, &m, V_MAPR + V_EXC + 2u);
    vline_init(&v_d1l, &m, V_D1L); vline_init(&v_apl, &m, V_APL); vline_init(&v_d2l, &m, V_D2L);
    vline_init(&v_d1r, &m, V_D1R); vline_init(&v_apr, &m, V_APR); vline_init(&v_d2r, &m, V_D2R);
    vline_init(&v_pre, &m, V_PRE);
    v_ready = 1;                                /* (last: the lines are all in place) */
}

static inline __attribute__((always_inline)) int16_t sat16(int32_t v) { return (int16_t)(v > 32767 ? 32767 : v < -32767 ? -32767 : v); }
static inline __attribute__((always_inline)) int32_t vread(const vline_t *l, uint32_t d)      /* d samples ago, 1..len */
{
    uint32_t i = l->w + l->len - d;
    if (i >= l->len) i -= l->len;
    return l->b[i];
}
static inline __attribute__((always_inline)) int16_t qr16(int32_t v, int32_t mask)   /* to 16 bits, then to the mask's */
{
    int32_t h = (int32_t)(((uint32_t)~mask + 1u) >> 1);   /* half a step: rounded, not chopped (no DC creeping in) */
    return (int16_t)(sat16(v + h) & mask);
}
static inline __attribute__((always_inline)) void vwrite(vline_t *l, int32_t v, int32_t mask)
{
    l->b[l->w] = qr16(v, mask);                 /* (the crush: to fewer bits, rounded) */
    if (++l->w >= l->len) l->w = 0;
}
static inline __attribute__((always_inline)) int32_t vallpass(vline_t *l, uint32_t d, int32_t g, int32_t x, int32_t mask)   /* g Q15 */
{
    int32_t del = vread(l, d), u = x - ((del * g) >> 15);
    vwrite(l, u, mask);
    return del + ((u * g) >> 15);
}
static inline __attribute__((always_inline)) int32_t vallpass_frac(vline_t *l, uint32_t d_q8, int32_t g, int32_t x, int32_t mask)   /* a moving length */
{
    uint32_t d = d_q8 >> 8, f = d_q8 & 255u;
    int32_t a = vread(l, d), b = vread(l, d + 1u), del = (a * (int32_t)(256u - f) + b * (int32_t)f) >> 8;
    int32_t u = x - ((del * g) >> 15);
    vwrite(l, u, mask);
    return del + ((u * g) >> 15);
}
static inline __attribute__((always_inline)) int32_t softclip(int32_t x)              /* x - x^3 / 3 on the clamped input: never more than 2/3 full scale */
{
    int32_t t;
    x = x > 32767 ? 32767 : x < -32767 ? -32767 : x;
    t = (x * x) >> 15;
    return x - ((x * t) >> 15) / 3;
}

/* the knobs as they stand this block, after the mod matrix (Q8 knob offsets in m[]) */
typedef struct { int32_t size, decay, damp, ret, fb; } verb_mod_t;

#ifndef FB_MAX
#define FB_MAX 15000                            /* the feedback at full: calibrated (host tests) to just ring at a medium decay */
#endif
#ifndef FB_LIM
#define FB_LIM 6000                             /* the level the feedback backs off toward: a ring, not a scream */
#endif
static int32_t v_fbenv;

static void verb_run(const int32_t *in, int32_t *out, uint32_t n, const verb_mod_t *m, int32_t *wet)
{
    uint32_t i, div = vp.rate ? 4u : 2u, k, frz = vp.freeze | v_gate_freeze;
    int32_t size_k = cl(vp.size + (m->size >> 8), 0, 127), dec_k = cl(vp.decay + (m->decay >> 8), 0, 127);
    int32_t damp_k = cl(vp.damp + (m->damp >> 8), 0, 127), ret_k = cl(vp.ret + (m->ret >> 8), 0, 127);
    int32_t fb_k = cl(vp.fb + (m->fb >> 8), 0, 127);
    int32_t sz = 9830 + size_k * 179;           /* the lengths x 0.3 .. 1.0, Q15 */
    int32_t decay = frz ? 32767 : 6554 + dec_k * 199;   /* 0.2 .. 0.97 */
    int32_t damp = frz ? 0 : damp_k * 200;              /* the share of the old value kept: 0 .. 0.78 */
    int32_t ret = ret_k * ret_k * 2;            /* the return, squared: 0 .. ~1.0 */
    int32_t fbg = frz ? 0 : fb_k * (FB_MAX / 127);   /* linear: the tail thickens, then (near the top) rings */
    int32_t ingain = frz ? 0 : 32767;
    int32_t mask = ~((1 << (vp.crush > 10u ? 10u : vp.crush)) - 1) & eng_mask;   /* crush 0 .. 10 bits off */
    uint32_t exc = (uint32_t)vp.mod * V_EXC * 2u;   /* the excursion, Q8 samples (0 .. 16) */
    uint32_t pre = 1u + (uint32_t)vp.pre * (V_PRE - 2u) / 127u;
    uint32_t dmapl = (V_MAPL * (uint32_t)sz >> 15), dmapr = (V_MAPR * (uint32_t)sz >> 15);
    uint32_t d1l = V_D1L * (uint32_t)sz >> 15, apl = V_APL * (uint32_t)sz >> 15, d2l = V_D2L * (uint32_t)sz >> 15;
    uint32_t d1r = V_D1R * (uint32_t)sz >> 15, apr = V_APR * (uint32_t)sz >> 15, d2r = V_D2R * (uint32_t)sz >> 15;
#define TAP(len) (((len) * (uint32_t)sz >> 15) + 1u)
    int32_t lvl = 0;
    if (!v_ready) { for (i = 0; i < n; i++) wet[i] = 0; return; }   /* (not set up yet: the sound starts first) */

    for (i = 0; i < n; i++) {
        if ((v_phase++ & (div - 1u)) == 0u) {          /* a reverb sample: the input averaged over div frames */
            int32_t x = 0, y, lm, rm;
            for (k = 0; k < div && i + k < n; k++) x += in[i + k];
            x = (x / (int32_t)div) >> 1;
            x = softclip(((x * ingain) >> 15) + v_fb);
            vwrite(&v_pre, x, -1);
            x = vread(&v_pre, pre);
            v_bw += ((x - v_bw) * 24576) >> 15;    /* (the input band: a gentle low-pass) */
            x = v_bw;
            for (k = 0; k < V_NAP; k++) x = vallpass(&v_ap[k], V_AP_LEN[k], V_AP_G[k], x, -1);
            v_lfo += 0x00A00000u;               /* the tank's slow wobble (~0.6 Hz at 22 kHz) */
            lm = (int32_t)((((uint32_t)(sine(v_lfo) + 32768) * exc) >> 16));
            rm = (int32_t)((((uint32_t)(sine(v_lfo + 0x40000000u) + 32768) * exc) >> 16));
            /* the left half of the tank */
            y = softclip(x + ((v_last_r * decay) >> 15));
            y = vallpass_frac(&v_mapl, (dmapl << 8) + (uint32_t)lm + 256u, -22938, y, mask);   /* -0.7 */
            vwrite(&v_d1l, y, mask);
            y = vread(&v_d1l, d1l);
            v_damp_l = y + (((v_damp_l - y) * damp) >> 15);
            y = (v_damp_l * decay) >> 15;
            y = vallpass(&v_apl, apl, 16384, y, mask);                          /* 0.5 */
            vwrite(&v_d2l, y, mask);
            v_last_l = vread(&v_d2l, d2l);
            /* the right half */
            y = softclip(x + ((v_last_l * decay) >> 15));
            y = vallpass_frac(&v_mapr, (dmapr << 8) + (uint32_t)rm + 256u, -22938, y, mask);
            vwrite(&v_d1r, y, mask);
            y = vread(&v_d1r, d1r);
            v_damp_r = y + (((v_damp_r - y) * damp) >> 15);
            y = (v_damp_r * decay) >> 15;
            y = vallpass(&v_apr, apr, 16384, y, mask);
            vwrite(&v_d2r, y, mask);
            v_last_r = vread(&v_d2r, d2r);
            /* the output taps (Dattorro's) */
            v_out_l = (vread(&v_d1r, TAP(266)) + vread(&v_d1r, TAP(2974)) - vread(&v_apr, TAP(1913))
                       + vread(&v_d2r, TAP(1996)) - vread(&v_d1l, TAP(1990)) - vread(&v_apl, TAP(187))
                       - vread(&v_d2l, TAP(1066))) * 3 >> 3;
            v_out_r = (vread(&v_d1l, TAP(353)) + vread(&v_d1l, TAP(3627)) - vread(&v_apl, TAP(1228))
                       + vread(&v_d2l, TAP(2673)) - vread(&v_d1r, TAP(2111)) - vread(&v_apr, TAP(335))
                       - vread(&v_d2r, TAP(121))) * 3 >> 3;
            {                                   /* back into the input next time, less as it rings louder */
                int32_t f = (v_out_l + v_out_r) >> 1, a = f < 0 ? -f : f, room;
                if (a > v_fbenv) v_fbenv += (a - v_fbenv) >> 4; else v_fbenv -= (v_fbenv - a) >> 11;
                room = cl(FB_LIM - v_fbenv, 0, FB_LIM);
                v_fb = (((f * fbg) >> 15) * ((room << 15) / FB_LIM)) >> 15;
            }
        }
        out[2u * i] += (v_out_l * ret) >> 15;   /* held between the reverb's samples: the grit */
        out[2u * i + 1u] += (v_out_r * ret) >> 15;
        wet[i] = (v_out_l + v_out_r) >> 1;      /* (for the next in the chain) */
        {
            int32_t a = v_out_l < 0 ? -v_out_l : v_out_l;
            if (a > lvl) lvl = a;
        }
    }
#undef TAP
    v_level = lvl;
}

/* ---- the delay: a shift register of DL_N cells, clocked: the clock sets the time, so a turned TIME bends the
 * repeats' pitch, and a slow clock is a low sample rate (darker, grainier). The input is sampled at each tick,
 * the output held between ticks and smoothed a little (a cutoff that follows the clock, as a BBD's filters do).
 * Safe from the first interrupt: a static buffer, no setup. */
#define DL_N 2048u
static int16_t dl_buf[DL_N] __attribute__((section(".pool")));
typedef struct { volatile uint8_t time, fb, crush, ret, toverb; } dl_par_t;
static dl_par_t dlp;
static uint32_t dl_ph, dl_w;
static int32_t dl_hold, dl_lp, dl_in;
static volatile uint32_t dl_w_v;                /* the write head, for the screen */
#define DL_FB_MAX 29500                         /* ~0.9: long, never runaway (and the loop's saturated) */

/* one frame: in = the sends' sum; returns the delayed signal (before the return level) */
static inline __attribute__((always_inline)) int32_t dl_frame(int32_t in, uint32_t inc, int32_t fbg, int32_t mask,
                                                             int32_t lpc)
{
    uint32_t ph = dl_ph + inc;
    dl_in = in;
    if (ph < dl_ph || inc == 0xFFFFFFFFu) {     /* a clock tick: the oldest cell out, the input (and feedback) in */
        int32_t old = dl_buf[dl_w];
        dl_buf[dl_w] = qr16(softclip((dl_in >> 1) + ((old * fbg) >> 15)), mask);
        if (++dl_w >= DL_N) dl_w = 0;
        dl_hold = old;
    }
    dl_ph = ph;
    dl_lp += ((dl_hold - dl_lp) * lpc) >> 15;
    return dl_lp;
}

/* ---- the flanger: a short delay swept by its own triangle LFO, at 22 or 11 kHz (11: twice the time, grittier),
 * held between its samples as the others. Two taps swept in opposite directions: stereo. Feedback -..+ through the
 * saturator (- hollow and metallic, + the jet), at most ~0.9. Safe from the first interrupt: a static buffer. */
#define FL_N 1024u
static int16_t fl_buf[2][FL_N] __attribute__((section(".pool")));   /* left and right: each its own comb */
typedef struct { volatile uint8_t rate, depth, manual, fb, ret, slow; } fl_par_t;
static fl_par_t flp;
static uint32_t fl_w, fl_lfo, fl_phase;
static int32_t fl_hold_l, fl_hold_r;
static volatile int32_t fl_lfo_v;               /* the LFO's position, -32768..32767, for the screen */

static inline __attribute__((always_inline)) int32_t fl_read(const int16_t *b, uint32_t d_q8)   /* d samples ago, Q8 */
{
    uint32_t d = d_q8 >> 8, f = d_q8 & 255u, i0 = fl_w + FL_N - d, i1 = i0 + FL_N - 1u;
    if (i0 >= FL_N) i0 -= FL_N;
    if (i1 >= 2u * FL_N) i1 -= 2u * FL_N;
    if (i1 >= FL_N) i1 -= FL_N;
    return (b[i0] * (int32_t)(256u - f) + b[i1] * (int32_t)f) >> 8;
}
static int32_t fl_oct(int32_t base_q8, int32_t oct_q8)   /* base x 2^(oct / 256): the sweep, even in pitch */
{
    int32_t ip = oct_q8 >> 8;                   /* (an arithmetic shift: floor) */
    uint32_t m = ((uint32_t)base_q8 * (LUT_OCTFRAC[oct_q8 & 255] >> 4)) >> 12;   /* 32-bit: base < 2^18 */
    if (ip > 12) ip = 12;
    if (ip < -31) return 0;
    return ip >= 0 ? (int32_t)(m << ip) : (int32_t)(m >> -ip);
}
static int32_t fl_manual(uint32_t k)            /* MANUAL 0..127 -> the centre, Q8 samples: 1 .. ~390, even in pitch */
{
    return fl_oct(256, (int32_t)k * 2200 / 127);
}

/* in: the block's input (sends + the chain); wet: its output, mono, for the next in the chain; out: + the return */
static void fl_run(const int32_t *in, int32_t *wet, int32_t *out, uint32_t n, const uint32_t *lut_rate)
{
    uint32_t i, k, div = flp.slow ? 4u : 2u;
    uint32_t inc = lut_rate[flp.rate & 127u] >> (flp.slow ? 3u : 4u);   /* the LUT is a step per 32 frames */
    int32_t man = fl_manual(flp.manual);
    int32_t span = (int32_t)flp.depth * 512 / 127;                     /* the sweep: up to +-2 octaves, Q8 */
    int32_t fbg = ((int32_t)flp.fb - 64) * (32100 / 63), ret = (int32_t)flp.ret * flp.ret * 2;
    const int32_t lo = 256, hi = (int32_t)(FL_N - 4u) << 8;
    for (i = 0; i < n; i++) {
        if ((fl_phase++ & (div - 1u)) == 0u) {
            int32_t x = 0, t, tri, e, yl, yr;
            for (k = 0; k < div && i + k < n; k++) x += in[i + k];
            x /= (int32_t)div;
            fl_lfo += inc;
            t = (int32_t)(fl_lfo >> 16);
            tri = (t < 32768 ? t : 65535 - t) * 2 - 32768;   /* a triangle, -32768 .. 32766 */
            e = (tri * span) >> 15;
            yl = fl_read(fl_buf[0], (uint32_t)cl(fl_oct(man, e), lo, hi));   /* the two taps swept apart */
            yr = fl_read(fl_buf[1], (uint32_t)cl(fl_oct(man, -e), lo, hi));
            fl_buf[0][fl_w] = qr16(softclip((x >> 2) + ((yl * fbg) >> 15)), eng_mask);   /* each fed back on its own: it rings */
            fl_buf[1][fl_w] = qr16(softclip((x >> 2) + ((yr * fbg) >> 15)), eng_mask);   /* (in at a quarter: room to ring) */
            if (++fl_w >= FL_N) fl_w = 0;
            fl_hold_l = softclip(4 * yl);         /* the swept echo alone: it combs against the dry voices */
            fl_hold_r = softclip(4 * yr);         /* in the main mix, so the notches go all the way down */
            fl_lfo_v = tri;
        }
        out[2u * i] += (fl_hold_l * ret) >> 15;
        out[2u * i + 1u] += (fl_hold_r * ret) >> 15;
        wet[i] = (fl_hold_l + fl_hold_r) >> 1;
    }
}
