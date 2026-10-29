/* SPDX-License-Identifier: GPL-3.0-only */
/* FM-1 wavetable synth: test build 0.1, on Dinghy (zvenson), on the platform of Felucca (Hügelton Instruments)
 * as SLOOP (isod89) carries it.
 *
 * Four PPG-style wavetable oscillators, droning: 1 and 2 phase modulate each other, A and B have wavefolders.
 * Each: pitch (semitone + fine), wave position, PM depth or fold depth + threshold, hi cut (12 dB), level, pan.
 * One wavetable for now (SINE>ODD). No modulation, gates or matrices yet: this build proves the voice.
 *
 *   SELECT knob     choose the oscillator: 1, 2, A, B
 *   ALGORITHM       pitch in semitones          (hold SEL: fine tune, cents)
 *   KNOB 1          wave position
 *   KNOB 2          PM depth (1, 2) / fold depth (A, B)   (hold SEL on A, B: fold threshold)
 *   KNOB 3          hi cut (fully clockwise: open)
 *   KNOB 4          level
 *   PRESETS knob    pan
 *   keys            tap a key: the selected oscillator jumps to that note (build chord stacks fast)
 *   OCT- OCT+       the keyboard's octave
 *   MASTER          volume
 *
 * Rules of the road (the chip has no FPU and no libgcc): integers only, no float, no 64-bit division.
 * tests/host.c runs this file on a computer: the screen as a PNG, the sound as a WAV. */
#define APP_NAME "FM1 WAVETABLE"
#define CV_MAX (240u * 150u)                    /* the parameter grid's canvas */
#include "platform.c"
#include "wt_tables.h"
#include "luts.h"

/* ---- the oscillators */
#define NO 4u
typedef struct {                                /* set by the main loop, read by the audio interrupt */
    volatile uint32_t inc;                      /* phase step per frame: f * 2^32 / 44100 */
    volatile uint8_t table, pos, mod, thr, hicut, level, pan;
} osc_par_t;
typedef struct {                                /* owned by the audio interrupt */
    uint32_t ph;
    int32_t posq;                               /* wave position now, Q8 waves: glides to the knob */
    int32_t amp;                                /* level now, glides to the knob: no zipper, no clicks */
    int32_t lp1, lp2;                           /* the hi cut's two poles */
    int32_t last;                               /* the last output before the filter (the other's PM source) */
} osc_st_t;
static osc_par_t opar[NO];
static osc_st_t ost[NO];
static volatile uint32_t audio_us, audio_us_max;   /* how long a render took (the device only) */

static void app_audio(int32_t *out, uint32_t n)
{
    uint32_t o, i;
    int32_t posq_t[NO], fold_g[NO], fold_t[NO], fold_mk[NO], amp_t[NO], hc[NO], pl[NO], pr[NO];
    const int8_t *tab[NO];
    uint32_t inc[NO], pm[NO];
#ifndef DINGHY_HOST
    uint32_t t0 = fm1_ticks();
#endif
    for (o = 0; o < NO; o++) {                  /* once a block: the knobs into coefficients */
        tab[o] = &WT[opar[o].table % WT_TABLES][0][0];
        inc[o] = opar[o].inc;
        posq_t[o] = (int32_t)opar[o].pos * (int32_t)((WT_WAVES - 1u) * 256u) / 127;
        pm[o] = opar[o].mod;                     /* 1, 2: PM depth 0..127 */
        fold_g[o] = 256 + (int32_t)opar[o].mod * 14;          /* A, B: drive x1..x8, Q8 */
        fold_t[o] = 4096 + (int32_t)opar[o].thr * 224;        /* threshold, Q15 */
        fold_mk[o] = (int32_t)((32767u << 12) / (uint32_t)fold_t[o]);   /* make-up gain, Q12 (divisor >= 4096) */
        hc[o] = opar[o].hicut;
        amp_t[o] = LUT_LEVEL[opar[o].level & 127u];
        pl[o] = LUT_PAN_L[opar[o].pan & 127u];
        pr[o] = LUT_PAN_R[opar[o].pan & 127u];
    }
    for (o = 0; o < NO; o++) {                  /* the wave position glides a step per block */
        int32_t d = posq_t[o] - ost[o].posq;
        ost[o].posq += d > 192 ? 192 : d < -192 ? -192 : d;
    }
    for (i = 0; i < n; i++) {
        int32_t l = 0, r = 0;
        int32_t prev0 = ost[0].last, prev1 = ost[1].last;      /* 1 and 2 hear each other one sample late */
        for (o = 0; o < NO; o++) {
            osc_st_t *s = &ost[o];
            uint32_t p, idx, w, frac;
            int32_t x, v;
            const int8_t *w0, *w1;
            s->ph += inc[o];
            p = s->ph;
            if (o == 0u) p += (uint32_t)(prev1 * (int32_t)pm[0]) << 10;   /* phase modulation: up to ~1 cycle */
            else if (o == 1u) p += (uint32_t)(prev0 * (int32_t)pm[1]) << 10;
            idx = p >> 25;                       /* 128 samples a wave, no interpolation: PPG grit */
            w = (uint32_t)s->posq >> 8;
            frac = (uint32_t)s->posq & 255u;
            w0 = tab[o] + w * WT_SAMPLES;
            w1 = w + 1u < WT_WAVES ? w0 + WT_SAMPLES : w0;
            x = (int32_t)w0[idx] * (int32_t)(256u - frac) + (int32_t)w1[idx] * (int32_t)frac;   /* +-32512 */
            if (o >= 2u) {                       /* the wavefolder */
                int32_t y = (x * fold_g[o]) >> 8, t = fold_t[o], k;
                for (k = 0; k < 24 && (y > t || y < -t); k++)
                    y = y > t ? 2 * t - y : -2 * t - y;
                x = (y * fold_mk[o]) >> 12;
            }
            s->last = x;
            if (hc[o] < 127) {                   /* the hi cut: two one-poles, 12 dB */
                int32_t a = LUT_HICUT[hc[o]];
                s->lp1 += ((x - s->lp1) * a) >> 15;
                s->lp2 += ((s->lp1 - s->lp2) * a) >> 15;
                x = s->lp2;
            }
            if (s->amp < amp_t[o]) { s->amp += 16; if (s->amp > amp_t[o]) s->amp = amp_t[o]; }
            else if (s->amp > amp_t[o]) { s->amp -= 16; if (s->amp < amp_t[o]) s->amp = amp_t[o]; }
            v = (x * s->amp) >> 15;              /* <= 10000 */
            l += (v * pl[o]) >> 15;
            r += (v * pr[o]) >> 15;
        }
        out[2u * i] += l;
        out[2u * i + 1u] += r;
    }
#ifndef DINGHY_HOST
    audio_us = (fm1_ticks() - t0) / FM1_TICKS_PER_US;
    if (audio_us > audio_us_max) audio_us_max = audio_us;
#endif
}

/* ---- the patch (main loop) */
typedef struct { uint8_t note; int8_t fine; uint8_t pos, mod, thr, hicut, level, pan; } patch_t;
static patch_t opatch[NO] = {                       /* a fixed drone to start from: C2, G2, C3, G3 */
    {36,  0, 20, 20,  0,  90, 100,  40},
    {43,  4, 50, 10,  0,  80,  85,  88},
    {48, -3, 10, 30,  90,  70,  80,  20},
    {55,  2, 64, 15, 110,  60,  70, 107},
};
static const uint32_t INC_C9[12] = {            /* MIDI 120..131 (C9..B9); lower octaves shift right */
    815363807u, 863847862u, 915214929u, 969636441u, 1027294024u, 1088380105u,
    1153098554u, 1221665363u, 1294309365u, 1371273005u, 1452813141u, 1539201906u};

static void push(uint32_t o)                    /* the patch into what the interrupt reads */
{
    patch_t *q = &opatch[o];
    uint32_t inc = INC_C9[q->note % 12u] >> (10u - q->note / 12u);
    opar[o].inc = (uint32_t)(((uint64_t)inc * LUT_FINE[q->fine + 50]) >> 16);
    opar[o].table = 0;
    opar[o].pos = q->pos;
    opar[o].mod = q->mod;
    opar[o].thr = q->thr;
    opar[o].hicut = q->hicut;
    opar[o].level = q->level;
    opar[o].pan = q->pan;
}

static int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : v > hi ? hi : v; }

/* ---- the screen */
#define WT_ACC RGB(120, 220, 200)
#define WT_DIM RGB(110, 110, 110)
#define WT_TXT RGB(220, 220, 220)
static const char *const ONAME[NO] = {"1", "2", "A", "B"};

static void wt_fmt_note(char *s, uint32_t note)
{
    static const char *const NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    const char *p = NAME[note % 12u];
    uint32_t i = 0;
    while (*p) s[i++] = *p++;
    if (note >= 12u) s[i++] = (char)('0' + note / 12u - 1u);
    s[i] = 0;
}
static void wt_fmt_int(char *s, int32_t v, int sign)
{
    char t[8];
    uint32_t i = 0, j = 0, u = (uint32_t)(v < 0 ? -v : v);
    if (v < 0) s[j++] = '-'; else if (sign && v > 0) s[j++] = '+';
    do { t[i++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (i) s[j++] = t[--i];
    s[j] = 0;
}

static void draw_grid(uint32_t sel, int shift)
{
    static const char *const ROW[8] = {"NOTE", "FINE", "WAVE", "PM/FLD", "THRESH", "HI CUT", "LEVEL", "PAN"};
    uint32_t o, r;
    char s[12];
    cv_begin(240, 150, C_BLACK);
    for (o = 0; o < NO; o++) {
        int32_t x = 64 + (int32_t)o * 44;
        if (o == sel) cv_rect(x - 2, 0, 42, 150, RGB(30, 60, 56));
        cv_text(x + 14, 2, &FONT_S, ONAME[o], o == sel ? WT_ACC : WT_DIM);
    }
    for (r = 0; r < 8u; r++) {
        int32_t y = 20 + (int32_t)r * 16;
        int hot = shift ? (r == 1u || r == 4u) : (r != 1u && r != 4u);   /* the rows the knobs reach now */
        cv_text(2, y, &FONT_S, ROW[r], hot ? WT_TXT : WT_DIM);
        for (o = 0; o < NO; o++) {
            patch_t *q = &opatch[o];
            int32_t x = 64 + (int32_t)o * 44;
            switch (r) {
            case 0: wt_fmt_note(s, q->note); break;
            case 1: wt_fmt_int(s, q->fine, 1); break;
            case 2: wt_fmt_int(s, q->pos, 0); break;
            case 3: wt_fmt_int(s, q->mod, 0); break;
            case 4: if (o < 2u) { s[0] = '-'; s[1] = 0; } else wt_fmt_int(s, q->thr, 0); break;
            case 5: if (q->hicut >= 127u) { s[0] = 'O'; s[1] = 'P'; s[2] = 'N'; s[3] = 0; } else wt_fmt_int(s, q->hicut, 0); break;
            case 6: wt_fmt_int(s, q->level, 0); break;
            default: wt_fmt_int(s, (int32_t)q->pan - 64, 1); break;
            }
            cv_text(x, y, &FONT_S, s, o == sel ? WT_TXT : WT_DIM);
        }
    }
    cv_blit(0, 48);
}

static void draw_cpu(uint32_t us)
{
    char s[24] = "CPU ", n[8];
    uint32_t pct = us * 100u / 5805u, i = 4, j = 0;   /* 256 frames at 44.1 kHz = 5805 us */
    wt_fmt_int(n, (int32_t)pct, 0);
    while (n[j]) s[i++] = n[j++];
    s[i++] = '%';
    s[i] = 0;
    draw_text_box(0, 212, 240, &FONT_S, s, WT_DIM, 1);
}

/* ---- the main loop */
static void app_main(void)
{
    uint32_t o, k, sel = 0, prev = 0, dirty = 1, cpu_drawn = ~0u, t_cpu = 0;
    int32_t oct = -1, e, shift_drawn = -1;
    for (o = 0; o < NO; o++) push(o);

    lcd_fill(0, 0, 240, 240, C_BLACK);
    draw_text_box(0, 8, 240, &FONT_L, "WAVETABLE", WT_ACC, 1);
    draw_text_box(0, 212, 240, &FONT_S, "test build " APP_VER, WT_DIM, 1);

    for (;;) {
        uint32_t keys, down, rel, b, base;
        int shift;
        platform_poll();

        b = fm1_input_edges(&rel);
        shift = (fm1_in.buttons >> BTN_SEL) & 1u;
        if (b & (1u << BTN_OCT_DOWN) && oct > -3) oct--;
        if (b & (1u << BTN_OCT_UP) && oct < 2) oct++;
        base = (uint32_t)(53 + 12 * oct);       /* key 0 = F3, shifted */

        if ((e = fm1_enc_take(ENC_SELECT)) != 0) { sel = (uint32_t)clampi((int32_t)sel + e, 0, NO - 1); dirty = 1; }
        {
            patch_t *q = &opatch[sel];
            int ch = 0;
            if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {
                if (shift) q->fine = (int8_t)clampi(q->fine + e, -50, 50);
                else q->note = (uint8_t)clampi(q->note + e, 12, 108);
                ch = 1;
            }
            if ((e = fm1_enc_take(ENC_KNOB1)) != 0) { q->pos = (uint8_t)clampi(q->pos + 2 * e, 0, 127); ch = 1; }
            if ((e = fm1_enc_take(ENC_KNOB2)) != 0) {
                if (shift && sel >= 2u) q->thr = (uint8_t)clampi(q->thr + 2 * e, 0, 127);
                else q->mod = (uint8_t)clampi(q->mod + 2 * e, 0, 127);
                ch = 1;
            }
            if ((e = fm1_enc_take(ENC_KNOB3)) != 0) { q->hicut = (uint8_t)clampi(q->hicut + 2 * e, 0, 127); ch = 1; }
            if ((e = fm1_enc_take(ENC_KNOB4)) != 0) { q->level = (uint8_t)clampi(q->level + 2 * e, 0, 127); ch = 1; }
            if ((e = fm1_enc_take(ENC_PRESETS)) != 0) { q->pan = (uint8_t)clampi(q->pan + 2 * e, 0, 127); ch = 1; }

            keys = fm1_in.notes;                /* a key tapped: the selected oscillator takes its note */
            down = keys & ~prev;
            prev = keys;
            for (k = 0; k < 27u; k++)
                if (down >> k & 1u) { q->note = (uint8_t)clampi((int32_t)(base + k), 12, 108); ch = 1; }
            if (ch) { push(sel); dirty = 1; }
        }

        for (k = 0; k < 27u; k++) {             /* the keys show the stack: the selected bright, the others dim */
            uint32_t note = base + k, on = 0, glow = 0;
            for (o = 0; o < NO; o++)
                if (opatch[o].note == note) { if (o == sel) on = 1; else glow = 1; }
            fm1_led_key(KEY_LED(k), on);
            if (!on) fm1_led_key_glow(KEY_LED(k), glow);
        }
        fm1_led_key(BTN_OCT_DOWN, oct < 0);
        fm1_led_key(BTN_OCT_UP, oct > 0);
        fm1_led_key(BTN_SEL, shift);

        if (dirty || shift != shift_drawn) { dirty = 0; shift_drawn = shift; draw_grid(sel, shift); }
        if (fm1_ms - t_cpu > 500u) {            /* the worst render of the last half second */
            uint32_t us = audio_us_max;
            audio_us_max = 0;
            t_cpu = fm1_ms;
            if (us && us != cpu_drawn) { cpu_drawn = us; draw_cpu(us); }
        }
    }
}
