#!/usr/bin/env python3
"""Generate the wavetables as a C header (firmware/src/wt_tables.h).

PPG / early Waldorf style: 64 waves per table, 128 samples per wave, 8-bit signed.
Each table is described by a recipe: a function from position t (0..1 across the table)
to a list of (harmonic number, amplitude, phase). The waves are built by additive
synthesis here on the computer, normalised, quantised to 8 bits and written out as
const data, so they live in flash and cost no RAM on the FM-1.

Usage: python3 tools/gen_tables.py   (run from the project folder)
"""
import math
from pathlib import Path

WAVES = 64
SAMPLES = 128


def smooth(x):
    """0 below 0, 1 above 1, an S-curve between."""
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def reach(t, top):
    """How far up the harmonic series a wave reaches at position t.
    Slow at the start, quick in the last third: the 'bite late' curve."""
    return 1.0 + (t ** 2.2) * (top - 1.0)


# ---- the recipes ------------------------------------------------------------
# A recipe maps a position t (0..1 across the table) to a wave: a list of partials (harmonic, amplitude, phase)
# or, for the time-domain ones, ("wave", 128 samples). All start soft (a sine or a triangle: they suit PM and the
# folders) and bite in the last third. A table normalises each wave to full scale, except PWM (the table as a whole,
# so its first wave stays the silence it is).

def fade(h, k, width=2.0):
    """how far partial k is in, while the reach h passes it"""
    return smooth((h - k) / width + 1.0)


def sine_to_odd(t):
    """1: a sine gaining odd harmonics, ending hollow and square-ish."""
    h = reach(t, 31)
    out = [(1, 1.0, 0.0)]
    for k in range(3, 32, 2):
        g = smooth(h - k + 2)          # each odd harmonic fades in as the reach passes it
        if g > 0:
            out.append((k, g / k, 0.0))
    return out


def tri_to_saw(t):
    """2: a triangle (odd, 1/k^2) turning into a saw (every harmonic, 1/k), reaching higher as it goes."""
    h, m = reach(t, 40), smooth(t * 1.3)
    out = []
    for k in range(1, 41):
        tri = (8 / math.pi ** 2) * ((-1) ** ((k - 1) // 2)) / k ** 2 if k % 2 else 0.0
        saw = 2 / math.pi / k
        a = ((1 - m) * tri + m * saw) * (1.0 if k == 1 else fade(h, k, 3.0))
        if a:
            out.append((k, a, 0.0))
    return out


def organ_drawbar(t):
    """3: drawbars pulled one by one: 8', 4', 2 2/3', 2', 1 1/3', 1', 1 3/5', then the bright upper ones."""
    order = [(1, 1.0), (2, 0.8), (3, 0.7), (4, 0.6), (6, 0.5), (8, 0.45), (5, 0.35), (10, 0.35), (12, 0.3), (16, 0.3)]
    out = []
    for i, (k, a) in enumerate(order):
        start = i / len(order) * (0.75 if i < 7 else 1.0) if i else -1.0
        g = smooth((t - start) * 5.0) if i else 1.0
        if i >= 7:                      # the last third: the high bars come in strong
            g *= 0.5 + 1.5 * smooth((t - 0.66) * 3.0)
        if g > 0:
            out.append((k, a * g, 0.0))
    return out


def organ_reed(t):
    """4: reedy and nasal, clarinet-ish (odd first), the even ones creeping in, honky in the last third."""
    h = reach(t, 24)
    out = [(1, 1.0, 0.0)]
    for k in range(2, 25):
        base = (0.9 / k) if k % 2 else (0.25 / k) * smooth(t * 2.0)
        honk = 1.0 + 4.0 * smooth((t - 0.6) * 2.5) * math.exp(-((k - 8) / 2.5) ** 2)   # a nasal bump around 6..10
        a = base * honk * fade(h, k, 2.5)
        if a > 1e-4:
            out.append((k, a, 0.0))
    return out


def bell_harmonic(t):
    """5: a PPG-style harmonic bell: sparse high partials glinting in over a fading fundamental."""
    parts = [(3, 0.5), (5, 0.45), (9, 0.4), (11, 0.35), (17, 0.35), (23, 0.3), (27, 0.3), (31, 0.28), (37, 0.25)]
    out = [(1, 1.0 - 0.45 * smooth(t * 1.5), 0.0)]
    for i, (k, a) in enumerate(parts):
        start = (i / len(parts)) ** 1.3 * 0.95
        g = smooth((t - start) * 6.0)
        if i >= 6:
            g *= 0.6 + 1.4 * smooth((t - 0.66) * 3.0)
        if g > 0:
            out.append((k, a * g, 0.3 * i))
    return out


def bell_cluster(t):
    """6: clangier: clusters of neighbouring upper partials, rising and widening: metallic by the end."""
    out = [(1, 1.0 - 0.5 * smooth(t), 0.0)]
    centre, width = 6 + 22 * t ** 1.5, 0.6 + 2.6 * t ** 2
    level = 1.05 * smooth(t * 1.4)        # (nothing at the start: a sine)
    for k in range(2, 50):
        a = level * math.exp(-((k - centre) / width) ** 2)
        a += 0.6 * smooth((t - 0.6) * 2.5) * math.exp(-((k - (centre * 1.6)) / (width * 0.8)) ** 2)   # a second, higher cluster
        if a > 2e-3:
            out.append((k, a, 1.1 * k))
    return out


F0 = 130.0                              # the voice the vowels are tuned for (they shift with pitch, as on the PPG)


def formants(fs, k):
    """the level of harmonic k under formants fs = [(Hz, bandwidth Hz, gain), ...]"""
    f = k * F0
    return sum(g / (1 + ((f - c) / (bw / 2)) ** 2) for c, bw, g in fs)


def lerp_formants(a, b, x):
    return [tuple(p + (q - p) * x for p, q in zip(fa, fb)) for fa, fb in zip(a, b)]


OO = [(300, 90, 1.0), (870, 120, 0.3), (2240, 200, 0.02)]
OH = [(500, 100, 1.0), (900, 120, 0.6), (2400, 200, 0.05)]
AH = [(730, 120, 1.0), (1090, 130, 0.7), (2440, 180, 0.25)]
EE = [(270, 80, 1.0), (2290, 160, 0.3), (3010, 220, 0.2)]
EH = [(530, 100, 1.0), (1840, 150, 0.5), (2480, 200, 0.2)]


def vowel_set(seq, t, bite):
    seg = t * (len(seq) - 1)
    i = min(int(seg), len(seq) - 2)
    fs = lerp_formants(seq[i], seq[i + 1], seg - i)
    fs = fs + [(3300, 300, bite * smooth((t - 0.6) * 2.5))]   # the last third: an edge on top
    start = 1.0 - smooth(t * 2.0)       # the first part leans to a plain sine: delicate first
    out = []
    for k in range(1, 41):
        a = formants(fs, k) * (1 - start) + (start if k == 1 else 0.0)
        if a > 1e-3:
            out.append((k, a, 0.0))
    return out


def vowel_1(t):
    """7: oo -> oh -> ah."""
    return vowel_set([OO, OH, AH], t, 0.35)


def vowel_2(t):
    """8: ee -> eh -> ah, more nasal (a low nasal murmur)."""
    out = vowel_set([EE, EH, AH], t, 0.4)
    return out + [(2, 0.35 * smooth(t * 2.0), 0.0)]


def formant_sweep(t):
    """9: a resonant hump climbing the harmonics, narrowing and growing as it goes."""
    centre, q = 2 + 28 * t ** 1.4, 2.5 - 1.4 * t
    gain = 1.2 * smooth(t * 2.0) + 1.4 * smooth((t - 0.5) * 2.0)   # (nothing at the start: a sine)
    out = [(1, 1.0, 0.0)]
    for k in range(2, 48):
        a = gain * math.exp(-((k - centre) / q) ** 2) + 0.25 / k * smooth(t * 2)
        if a > 1e-3:
            out.append((k, a, 0.0))
    return out


def pwm(t):
    """10: a pulse from 0 % (silence) to 50 % (a square), soft at first, reaching higher late."""
    w, h = 0.5 * t, reach(t, 48)
    out = []
    for k in range(1, 49):
        a = 2 / (k * math.pi) * math.sin(k * math.pi * w) * (1.0 if k <= 2 else fade(h, k, 4.0))
        if abs(a) > 1e-5:
            out.append((k, a, math.pi / 2))
    return out


def band_limit(wave, top):
    """keep the harmonics up to top (soft-edged): the time-domain recipes stay soft until they are meant to bite"""
    import numpy as np
    sp = np.fft.rfft(np.asarray(wave, dtype=float))
    k = np.arange(len(sp))
    sp *= np.clip((top + 2 - k) / 3.0, 0, 1)
    sp[0] = 0
    return list(np.fft.irfft(sp, len(wave)))


def soft_sync(t):
    """11: a soft-synced sine: the slave climbs from unison, the reset softened by a window that hardens late:
    tearing in the last third."""
    r = 1 + 5.5 * t ** 1.3
    p = 3.0 * smooth(t * 5.0) - 2.8 * smooth((t - 0.55) * 2.2)   # the window: none at first (a sine), soft, then hard
    wave = [math.sin(2 * math.pi * r * (n / SAMPLES)) * math.sin(math.pi * n / SAMPLES) ** p for n in range(SAMPLES)]
    return ("wave", band_limit(wave, reach(t, 56)))


def grit(t):
    """12: a sine picking up digital artefacts: fewer bits, then fewer samples: crunchy at the end."""
    bits = 8 - 5 * smooth((t - 0.3) * 1.5)              # 8 .. 3 bits
    hold = 1 + int(round(15 * smooth((t - 0.55) * 2.2)))   # 1 .. 16 samples held
    wave = []
    for n in range(SAMPLES):
        m = (n // hold) * hold
        v = math.sin(2 * math.pi * m / SAMPLES)
        q = 2 ** (bits - 1)
        wave.append(round(v * q) / q)
    return ("wave", wave)


TABLES = [                              # (name, recipe, normalise each wave)
    ("SINE>ODD", sine_to_odd, True),
    ("TRI>SAW", tri_to_saw, True),
    ("ORGAN I", organ_drawbar, True),
    ("ORGAN II", organ_reed, True),
    ("BELL I", bell_harmonic, True),
    ("BELL II", bell_cluster, True),
    ("VOWEL I", vowel_1, True),
    ("VOWEL II", vowel_2, True),
    ("FORMANT", formant_sweep, True),
    ("PWM", pwm, False),
    ("SOFTSYNC", soft_sync, True),
    ("GRIT", grit, True),
]


# ---- building ---------------------------------------------------------------

def raw_wave(w):
    if isinstance(w, tuple) and w[0] == "wave":
        return list(w[1])
    s = []
    for n in range(SAMPLES):
        x = 2 * math.pi * n / SAMPLES
        s.append(sum(a * math.sin(k * x + p) for k, a, p in w))
    return s


def build_table(recipe, per_wave):
    raws = [raw_wave(recipe(w / (WAVES - 1))) for w in range(WAVES)]
    top = max(max(abs(v) for v in r) for r in raws) or 1.0
    out = []
    for r in raws:
        peak = (max(abs(v) for v in r) or 1.0) if per_wave else top
        out.append([max(-127, min(127, round(v / peak * 127))) for v in r])
    return out


def write_luts(root):
    """Lookup tables for the engine (the chip has no floating point)."""
    fs = 44100.0
    hicut = []                       # knob 0..127 -> one-pole coefficient, Q15; 127 = open (bypass)
    for k in range(128):
        if k == 127:
            hicut.append(32767)
            continue
        fc = 60.0 * (9000.0 / 60.0) ** (k / 126.0)
        hicut.append(round((1 - math.exp(-2 * math.pi * fc / fs)) * 32767))
    pan_l, pan_r = [], []            # knob 0..127 -> equal-power gains, Q15
    for k in range(128):
        a = k / 127 * math.pi / 2
        pan_l.append(round(math.cos(a) * 32767))
        pan_r.append(round(math.sin(a) * 32767))
    fine = [round(2 ** (c / 1200) * 65536) for c in range(-50, 51)]   # cents -50..+50 -> ratio, Q16
    level = [round((k / 127) ** 2 * 10000) for k in range(128)]        # knob -> amplitude, a little under a third of full scale
    # function generators, updated every 32 frames: time knob 0..127 -> 2 ms .. 60 s, as a step of a Q24 position
    sub = fs / 32
    fginc = [max(1, round((1 << 24) / ((0.002 * 30000 ** (k / 127)) * sub))) for k in range(128)]
    # the slope shapes over the position 0..1 (257 points, Q15): EXP accelerates, LOG decelerates
    a = 4.0
    exp_s = [round((math.exp(a * i / 256) - 1) / (math.exp(a) - 1) * 32767) for i in range(257)]
    log_s = [32767 - exp_s[256 - i] for i in range(257)]
    # gate generator LFOs, also every 32 frames: rate knob 0..127 -> 0.02 .. 30 Hz, as a step of a 32-bit phase
    gginc = [round(0.02 * 1500 ** (k / 127) / sub * 2 ** 32) for k in range(128)]
    # the quadrature LFO, every 32 frames: rate knob 0..127 -> a period of 120 s .. 0.1 s
    qper = [120.0 / 1200 ** (k / 127) for k in range(128)]
    qinc = [round(1 / t / sub * 2 ** 32) for t in qper]
    qcs = [round(t * 100) for t in qper]          # the period in 1/100 s, for the screen
    # the shift-register delay: 2048 cells; time knob 0..127 -> 46 ms .. 2 s, as the register clock's phase step per frame
    dcells = 2048
    tmin = dcells / fs
    dinc = [min(0xFFFFFFFF, round(dcells / (tmin * (2.0 / tmin) ** (k / 127)) / fs * 2 ** 32)) for k in range(128)]
    dms = [round(tmin * (2.0 / tmin) ** (k / 127) * 1000) for k in range(128)]
    semifrac = [round(2 ** (f / 256 / 12) * 65536) for f in range(256)]   # 1/256 semitone -> ratio, Q16
    octfrac = [round(2 ** (f / 256) * 65536) for f in range(256)]   # 1/256 octave -> ratio, Q16 (the flanger's sweep)
    arr = lambda name, typ, v: f"static const {typ} {name}[{len(v)}] = {{" + ",".join(map(str, v)) + "};"
    lines = [
        arr("LUT_FGINC", "uint32_t", fginc),
        arr("LUT_GGINC", "uint32_t", gginc),
        arr("LUT_QINC", "uint32_t", qinc),
        arr("LUT_QPER", "uint32_t", qcs),
        arr("LUT_DINC", "uint32_t", dinc),
        arr("LUT_DMS", "uint16_t", dms),
        arr("LUT_SHAPE_EXP", "uint16_t", exp_s),
        arr("LUT_SHAPE_LOG", "uint16_t", log_s),
        arr("LUT_SEMIFRAC", "uint32_t", semifrac),
        arr("LUT_OCTFRAC", "uint32_t", octfrac),
        "/* SPDX-License-Identifier: GPL-3.0-only */",
        "/* Generated by tools/gen_tables.py: do not edit by hand. */",
        arr("LUT_HICUT", "uint16_t", hicut),
        arr("LUT_PAN_L", "uint16_t", pan_l),
        arr("LUT_PAN_R", "uint16_t", pan_r),
        arr("LUT_FINE", "uint32_t", fine),
        arr("LUT_LEVEL", "uint16_t", level),
    ]
    (root / "firmware" / "src" / "luts.h").write_text("\n".join(lines) + "\n")


def main():
    root = Path(__file__).resolve().parent.parent
    write_luts(root)
    lines = [
        "/* SPDX-License-Identifier: GPL-3.0-only */",
        "/* Generated by tools/gen_tables.py: do not edit by hand. */",
        f"#define WT_TABLES {len(TABLES)}u",
        f"#define WT_WAVES {WAVES}u",
        f"#define WT_SAMPLES {SAMPLES}u",
        "static const char *const WT_NAME[WT_TABLES] = {"
        + ", ".join(f'"{name}"' for name, _, _ in TABLES) + "};",
        "static const int8_t WT[WT_TABLES][WT_WAVES][WT_SAMPLES] = {",
    ]
    for name, recipe, per_wave in TABLES:
        lines.append(f"  {{ /* {name} */")
        for wave in build_table(recipe, per_wave):
            lines.append("    {" + ",".join(str(v) for v in wave) + "},")
        lines.append("  },")
    lines.append("};")
    out = root / "firmware" / "src" / "wt_tables.h"
    out.write_text("\n".join(lines) + "\n")
    print(f"{out}: {len(TABLES)} table(s), {len(TABLES) * WAVES * SAMPLES} bytes")



# ---- tunings and scales (firmware/src/tunings.h) -------------------------------------------------------------
# A tuning: its steps in cents within one period (usually the octave), and the scales over it (which steps are
# allowed). Pitches are 1/256 semitone in the firmware: a cent is 2.56 of them.

def cents(r):
    return 1200 * math.log2(r)


def ratios(*rs):
    return [cents(eval(r)) for r in rs]


def edo(n, period=1200.0):
    return [period * i / n for i in range(n)]


S12 = [("CHROMATIC", range(12)), ("MAJOR", [0, 2, 4, 5, 7, 9, 11]), ("MINOR", [0, 2, 3, 5, 7, 8, 10]),
       ("DORIAN", [0, 2, 3, 5, 7, 9, 10]), ("PHRYGIAN", [0, 1, 3, 5, 7, 8, 10]), ("LYDIAN", [0, 2, 4, 6, 7, 9, 11]),
       ("MIXOLYDIAN", [0, 2, 4, 5, 7, 9, 10]), ("LOCRIAN", [0, 1, 3, 5, 6, 8, 10]), ("HARM MINOR", [0, 2, 3, 5, 7, 8, 11]),
       ("MAJ PENTA", [0, 2, 4, 7, 9]), ("MIN PENTA", [0, 3, 5, 7, 10]), ("WHOLE TONE", [0, 2, 4, 6, 8, 10]),
       ("4TH + 5TH", [0, 5, 7]), ("FIFTHS", [0, 7]), ("OCTAVES", [0])]
ALL = lambda n: [("ALL STEPS", range(n))]

TUNINGS = [   # (name, period in cents, steps in cents, scales)
    ("12-TET", 1200, edo(12), S12),
    ("JUST 5-LIMIT", 1200, ratios("1", "16/15", "9/8", "6/5", "5/4", "4/3", "45/32", "3/2", "8/5", "5/3", "9/5", "15/8"), S12),
    ("PYTHAGOREAN", 1200, ratios("1", "256/243", "9/8", "32/27", "81/64", "4/3", "729/512", "3/2", "128/81", "27/16",
                                 "16/9", "243/128"), S12),
    ("MEANTONE 1/4", 1200, [0, 76.0, 193.2, 310.3, 386.3, 503.4, 579.5, 696.6, 772.6, 889.7, 1006.8, 1082.9], S12),
    ("SHRUTI 22", 1200, ratios("1", "256/243", "16/15", "10/9", "9/8", "32/27", "6/5", "5/4", "81/64", "4/3", "27/20",
                               "45/32", "729/512", "3/2", "128/81", "8/5", "5/3", "27/16", "16/9", "9/5", "15/8", "243/128"),
     ALL(22) + [("BILAWAL", [0, 4, 7, 9, 13, 16, 20]), ("BHAIRAV", [0, 2, 7, 9, 13, 15, 20]),
                ("YAMAN", [0, 4, 7, 11, 13, 16, 20]), ("KAFI", [0, 4, 6, 9, 13, 16, 19]),
                ("BHAIRAVI", [0, 2, 6, 9, 13, 15, 19]), ("TODI", [0, 1, 5, 12, 13, 14, 20])]),
    ("5-EDO", 1200, edo(5), ALL(5)),
    ("7-EDO", 1200, edo(7), ALL(7)),
    ("17-EDO", 1200, edo(17), ALL(17) + [("DIATONIC", [0, 3, 6, 7, 10, 13, 16])]),
    ("19-EDO", 1200, edo(19), ALL(19) + [("DIATONIC", [0, 3, 6, 8, 11, 14, 17]), ("PENTA", [0, 3, 6, 11, 14])]),
    ("22-EDO", 1200, edo(22), ALL(22) + [("SUPERPYTH", [0, 4, 8, 9, 13, 17, 21])]),
    ("24-EDO", 1200, edo(24), ALL(24) + [("MAQAM RAST", [0, 4, 7, 10, 14, 18, 21]),
                                         ("MAQAM BAYATI", [0, 3, 6, 10, 14, 16, 20])]),
    ("31-EDO", 1200, edo(31), ALL(31) + [("DIATONIC", [0, 5, 10, 13, 18, 23, 28])]),
    ("53-EDO", 1200, edo(53), ALL(53) + [("JUST MAJOR", [0, 9, 17, 22, 31, 39, 48])]),
    ("HARMONIC 8", 1200, [cents(k / 8) for k in range(8, 16)], ALL(8) + [("ODD ONLY", [0, 1, 3, 5, 7])]),
    ("HARMONIC 16", 1200, [cents(k / 16) for k in range(16, 32)],
     ALL(16) + [("PRIMES", [0, 1, 3, 7, 13, 15])]),
    ("SLENDRO", 1200, [0, 228, 484, 728, 960], ALL(5)),
    ("PELOG", 1200, [0, 120, 270, 540, 670, 785, 950], ALL(7) + [("BEM", [0, 1, 2, 4, 5]), ("BARANG", [1, 2, 4, 5, 6])]),
    ("BOHLEN-PIERCE", cents(3), edo(13, cents(3)), ALL(13) + [("LAMBDA", [0, 2, 3, 4, 6, 7, 9, 10, 12])]),
    ("CARLOS ALPHA", 78.0, [0], ALL(1)),
    ("CARLOS BETA", 63.8, [0], ALL(1)),
    ("CARLOS GAMMA", 35.1, [0], ALL(1)),
]


def write_tunings(root):
    q8 = lambda c: round(c * 2.56)
    steps, off, scl_name, lo, hi, scl0, nscl = [], [], [], [], [], [], []
    for name, period, st, scales in TUNINGS:
        assert len(st) <= 53 and st[0] == 0
        off.append(len(steps))
        steps += [q8(c) for c in st]
        scl0.append(len(scl_name))
        nscl.append(len(scales))
        for sname, sel in scales:
            m = 0
            for i in sel:
                m |= 1 << i
            scl_name.append(sname)
            lo.append(m & 0xFFFFFFFF)
            hi.append(m >> 32)
    arr = lambda name, typ, v: f"static const {typ} {name}[{len(v)}] = {{" + ",".join(map(str, v)) + "};"
    strs = lambda name, v: f"static const char *const {name}[{len(v)}] = {{" + ", ".join(f'"{x}"' for x in v) + "};"
    lines = [
        "/* SPDX-License-Identifier: GPL-3.0-only */",
        "/* Generated by tools/gen_tables.py: do not edit by hand. Pitches in 1/256 semitone. */",
        f"#define NTUN {len(TUNINGS)}u",
        f"#define NSCL {len(scl_name)}u",
        strs("TUN_NAME", [t[0] for t in TUNINGS]),
        arr("TUN_PERIOD", "uint16_t", [q8(t[1]) for t in TUNINGS]),
        arr("TUN_N", "uint8_t", [len(t[2]) for t in TUNINGS]),
        arr("TUN_OFF", "uint16_t", off),
        arr("TUN_STEPS", "uint16_t", steps),
        arr("TUN_SCL0", "uint8_t", scl0),
        arr("TUN_NSCL", "uint8_t", nscl),
        strs("SCL_NAME", scl_name),
        arr("SCL_LO", "uint32_t", lo),
        arr("SCL_HI", "uint32_t", hi),
    ]
    (root / "firmware" / "src" / "tunings.h").write_text("\n".join(lines) + "\n")
    print(f"tunings.h: {len(TUNINGS)} tunings, {len(scl_name)} scales")


if __name__ == "__main__":
    main()
    write_tunings(Path(__file__).resolve().parent.parent)
