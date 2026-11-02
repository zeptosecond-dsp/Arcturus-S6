# Arcturus S6 Quick Start Guide

Arcturus S6 turns the M-VAVE FM-1 into a four-oscillator wavetable instrument that plays itself: patch modulators into each other, set it going, and let it evolve. 

## What it is

Arcturus S6 is an experimental system for generative, evolving soundscapes, not a keyboard synth. There is no sequencer and no filter section: sound comes from four wavetable oscillators, and movement comes from modulators patched into the oscillators, into each other and into themselves.

**The Serge idea.** Serge synthesizers are built from small, general-purpose functions instead of fixed voices. A function generator can be an envelope, an LFO or a slew limiter depending on how it's patched; a gate can come from an LFO, a logic gate or the end of another envelope. The power is in the feedback: when modulators control each other, a few simple parts make patterns that never quite repeat. Arcturus S6 works the same way, with six Serge-style function generators, two sample-and-holds, a quadrature LFO, four gate generators with logic, and two patch matrices to connect them.

**The sound.** The engine runs natively at 12 bit and 22.05kHz, with 8-bit wavetables and no smoothing, so it aliases and steps the way early digital instruments did. Two machines are its direct influences:

- **PPG Wave & Waveterm** (1981, 1983): gritty, stepped wavetables you sweep through, rather than clean modern ones.
- **Fairlight CMI** (1979): low sample rates and short words, plus the look. The green phosphor screen, numbered pages and central index are taken from the Fairlight's monitor.

Expect drones, pulses and slowly shifting textures with a distinct early digital edge, and expect surprises: it's an experimental instrument, written by machine with human guidance.

## Getting around

The rule of thumb: **a flashing button does something on this page.** If it's flashing, try it.

- **The index.** The system boots to PAGE 1 INDEX. Turn **SELECT** to move the blinking cursor, press the flashing **SEL** to open a page.
- **HOME** flashes whenever you're away from the index. Press it to go back; the cursor remembers where you were.
- **Shortcut buttons** jump straight to a page (EDIT, ENV, LFO, ARP, SEQ, GLO, FX, SAVE); hold **SEL** with EDIT or ENV for their second page. The table below lists them.
- **The knobs** edit what's on screen; hints along the bottom say which knob does what. **Hold SEL** for a second layer of controls on pages that have one.
- **OCT− / OCT+** flash in turn on pages with more than one of something. They step to the previous or next one (FG, S&H, gate generator, oscillator, sub-page), wrapping round. On page 2 they shift the keyboard's octave instead.
- **PLAY** flashes where it has a job: a manual gate or sample, setting a pin, freezing the reverb, naming a patch.
- **The keys are a display.** On most pages the key LEDs follow what's on screen: an FG's slope travelling along the keys, an S&H value, gate states, an LFO dot. On page 2 the keys set oscillator pitches.
- **INIT and RANDOM.** The top two keys flash on every page except page 2. **G5** (far right) rolls a new random patch and **F#5** loads INIT (four sines in fifths, nothing patched). Each needs a second press to confirm; anything else cancels.
- **Power-on** always rolls a gentle random drone. RANDOM from the keys or the DISK page rolls busier, more heavily patched sounds.

## Page by page

Eleven pages, each reachable from the index or its shortcut button.

| Page | Button | What's there |
| --- | --- | --- |
| 1 INDEX | HOME | The central menu. SELECT moves, SEL opens. |
| 2 OSCILLATORS | EDIT | The four oscillators: 1 and 2 phase-modulate each other, A and B have wavefolders. Note, fine, table, wave, PM/fold, threshold, hi cut, level, pan. SELECT picks the oscillator, keys set its note, SEL for more. |
| 3 WAVETABLE | SEL + EDIT | The selected oscillator's table in 3D, the current wave lit. K1 wave, K2 table (13, the last generative: PLAY rerolls it), K3 smooth or stepped scanning. |
| 4 FUNCTION GEN | ENV | Six Serge-style FGs: rise, fall, curve (lin / exp / log) and mode (cycle / gate / once). Each sends an end-of-cycle gate. PLAY is a manual gate. |
| 5 S&H / QUAD | SEL + ENV | Two sample-and-holds (input: noise or any modulator, slew) with gate and sample lamps, and the quadrature LFO (SIN+, SIN−, COS+, COS−). PLAY samples by hand. |
| 6 GATE GEN | LFO | Four gate generators: two square LFOs each (free or MIDI clock, rate, duty) into AND, NAND, OR, NOR, XOR, XNOR, plus a NOT. Shown as a live logic analyser. |
| 7 MOD MATRIX | ARP | The EMS-style pin grid: 12 sources by every destination. K1 source, K2 destination, K3 depth (±), PLAY sets or pulls a pin. |
| 8 GATE MATRIX | SEQ | Gate sources (logic outputs, EOCs, quantiser note gates) to gate destinations (FGs, S&Hs, logic inputs, reverb freeze). Pins light with live gate state. |
| 9 GLOBAL | GLO | TUNING: 21 tunings and their scales, root, per-oscillator quantise and glide. OCT for VDU: phosphor colour, CRT brightness, a test card. |
| R FX | FX | Four halves, stepped with FX or OCT: REVERB (a low-rate plate, freeze), DELAY (a shift register), FLANGER (its own LFO, 22 / 11k) and ROUTING (the order of the three, and links between them). Each effect has its own send per oscillator. |
| D DISK | SAVE | A:\SYSTEM holds INIT and RANDOM, B:\PATCHES your 16 slots. SEL loads, SAVE saves (twice to overwrite), PLAY names a patch. |

## Patch ideas

Start from INIT (F#5, twice) and build up. These are loose starting points; the fun is in what goes wrong. Head over to the Disk page and try rolling a few random patches too, just to get a sense for how the system sounds and what its capabilities are. 

**Krell.** The classic self-varying patch: every event sets up the next one.

1. Set FG1 to ONCE with a short rise and a longer fall, and pin FG1 to an oscillator's LEVEL and HI CUT (page 7).
2. In the gate matrix (page 8), send FG1's EOC to S1's sample input and to FG1's own gate, so it retriggers itself.
3. Pin S1 to FG1's RISE and FALL, and to the oscillator's PITCH with quantise on (page 9). Each note now gets a new pitch, length and shape.

**A basic generative melody.**

1. On a gate generator, run its two LFOs at unrelated rates and send the XOR output to S1's sample input.
2. Pin S1 (input: noise, or a quad phase for something smoother) to an oscillator's PITCH, and turn its quantiser on with a scale you like.
3. Send that oscillator's quantiser note gate (Q1 NOTE etc.) to an FG in ONCE mode, and pin the FG to the oscillator's LEVEL. Each new note now plucks itself.

**Rhythms from logic.**

- Two square LFOs at related rates (try MIDI clock divisions) make polyrhythms through AND, OR and XOR; NOT turns a short pulse into a long one.
- Send different logic outputs to FGs in ONCE mode, each opening a different oscillator, for interlocking parts.
- Feed an FG's EOC back into a logic input, or pin the quad LFO to a gate LFO's RATE, for rhythms that break and drift.

**Slow drone motion.**

- Pin SIN+ and COS+ to the WAVE of oscillators 1 and 2: their tables sweep a quarter cycle apart.
- Slewed noise from S2 into PM/FOLD gives slow, unpredictable bite.
- A long, slow gate on the reverb's FREEZE (page 8) catches and holds moments of the texture.
