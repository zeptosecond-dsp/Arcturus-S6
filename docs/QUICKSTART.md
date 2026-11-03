# Quorum Quick Start Guide

Quorum turns the M-VAVE FM-1 into a 64-partial additive instrument that plays itself: two rows of cellular automata
switch the harmonics on and off, logic and modulators patched into each other keep it moving, and you steer.

## What it is

Quorum is an experimental system for generative, evolving soundscapes, not a keyboard synth. It's the sister of
Arcturus S6: where Arcturus sweeps wavetables, Quorum builds every sound from sine waves and lets simple rules decide
which ones sound.

**The additive idea.** Any sound can be built from sine waves at whole-number multiples of a fundamental: its
partials. Quorum has 64 of them, as the Kawai K5 did. Their levels follow a slope (TILT), their spacing can be
stretched out of tune for bells and metal (STRETCH), and each one fades in and out at the SLEW rate rather than
clicking.

**The cellular idea.** Each grid is a row of 32 cells, on or off. At every clock step each cell looks at itself and its
two neighbours and follows a rule (one of 256, Wolfram's elementary automata) to decide its next state. Some rules
settle, some repeat, some never do (rule 30, rule 110). A cell that's on lets its partial sound, so the rule plays the
harmonic series. Two grids share the 64 partials (low / high, odd / even, or combined with AND, OR, XOR), each on its
own clock or in lockstep. The name comes from quorum sensing, how bacteria act together once enough of them are
present, hence the petri dish on the boot screen.

**The Serge idea.** As in Arcturus, the movement comes from small, general-purpose parts patched together. Logic
generators turn cells and clocks into gates; function generators, sample-and-holds, a quadrature LFO and slew
sequencers make control voltages; two matrices connect everything to everything, the grids' own rules included. When
modulators control each other, a few simple parts make patterns that never quite repeat.

**The sound.** The engine runs at 22.05kHz. ENGINE VER V2 (the default) works in 12 bit; V1 works in 8 bit throughout,
levels and all, closer to the early Fairlight CMI. Two machines are its direct influences:

- **Kawai K5** (1987): additive synthesis with banks of sine partials, here put under the cells' control.
- **Fairlight CMI** (1979): short words and low rates in the sound, and the look. The phosphor screen, numbered pages
  and central index are taken from the Fairlight's monitor.

Expect drones, shimmering spectra, plucked and bowed harmonics and slowly shifting textures, and expect surprises:
it's an experimental instrument, written by machine with human guidance.

## Getting around

The rule of thumb: **a flashing button does something on this page.** If it's flashing, try it.

- **The index.** The system boots to PAGE 1 INDEX. Turn **SELECT** to move the blinking cursor, press the flashing
  **SEL** to open a page.
- **HOME** flashes whenever you're away from the index. Press it to go back; the cursor remembers where you were.
- **Shortcut buttons** jump straight to a page (EDIT, LFO, ENV, SEQ, ARP, FX, GLO, SAVE); hold **SEL** with EDIT,
  LFO, ENV or ARP for their second page. The table below lists them.
- **The knobs** K1 to K4 edit what's along the bottom of the screen. Where **SEL** flashes, it switches to the next bank
  of four.
- **OCT− / OCT+** flash in turn on pages with more than one of something. They step to the previous or next one (grid,
  strike, generator, FG, sequencer, path, effect), wrapping round.
- **PLAY** flashes where it has a job: reseeding a grid, firing a strike, setting a pin, a sequencer step's gate,
  freezing the reverb, naming a patch. Where it's solid, the thing it toggles is on.
- **The keys.** The lowest key you press (F1 to F3) sets the root for the paths that track the keyboard. The key LEDs
  show grid A's cells (on the GRIDS page, the grid shown).
- **The top two keys** are lit on every page: they fire strike 1 and strike 2 by hand. Bow and Breath sound for as long
  as you hold the key.
- **Power-on** always rolls a new slow drone. RANDOM on the DISK page rolls a busy, heavily patched patch (now and then
  it's just noise: roll again), and INIT gives you a plain one to build from. Your phosphor, brightness, engine version
  and clock source are remembered.

## Page by page

Thirteen pages, each reachable from the index or its shortcut button.

| Page | Button | What's there |
| --- | --- | --- |
| 1 INDEX | HOME | The central menu. SELECT moves, SEL opens. |
| 2 VOICE | EDIT | The 64 partials as bars (path A solid, path B lined). K1 SLEW (how fast partials fade in and out), K2 TILT (the spectrum's slope), K3 STRETCH (out of harmonic: bells), K4 LEVEL. ALGORITHM sets the root; SEL keeps the fundamental always on (F0). |
| 3 GRIDS | LFO | Grid A or B (OCT), its last 28 rows scrolling. Bank 1: RULE (shown as 45>110 when the matrix moves it), RATE (steps a second, a MIDI division, or GATE when the gate matrix steps it), MUTATE, SHARE (how the grids split the partials). Bank 2: RESCUE (flips a few cells when a grid dies, fills or loops), SEED (density for reseeding), SHIFT, LOCK (grid B on grid A's clock). PLAY reseeds. |
| 4 STRIKE | ENV | Strike 1 or 2. Bank 1: MODE (Strike, Mallet with marimba bar ratios, Bow, Breath), PARTIAL, VEL, LEVEL. Bank 2: the attack's shape, DECAY, FALL (higher partials die sooner, as a low-pass gate), SERIES (all, odd, even). Bank 3: the trill, COUNT, SPACE, PATTERN (rise, fall, hump), MAX VEL. ALGORITHM: summed with the grids or filtered by them; PRESETS: into its own path (strike 1 to A, 2 to B) or through the split. PLAY fires it. Its gate comes from the gate matrix. |
| 5 SPLIT + FILTERS | SEL + EDIT | SPLIT: RULE (cutoff, band, odd / even, every Nth, grid A / B, source), POINT, WIDTH, SOFT; ALGORITHM sets N. PATH A and PATH B: bank 1 FILTER (off, vowel, 12dB or 24dB band pass), FREQ or VOWEL, RESO, BW; bank 2 LEVEL, PAN, PITCH, FINE. PLAY: the path on or off the keys (off: it holds its note). |
| 6 LOGIC | SEL + LFO | Four generators drawn as a logic analyser: INPUT (a clock division, any cell of either grid, a grid's population, another generator, an FG's end of cycle), LOGIC (AND, OR, XOR, NAND, NOR, XNOR, or the input or LFO alone), RATE and DUTY of its square LFO. ALGORITHM: the LFO free or on the clock. |
| 7 SEQUENCERS | SEQ | SEQ 1 to 3: eight steps each. Bank 1 STEP (the cursor), VALUE, GATE, LENGTH; bank 2 DIR (forward, pendulum, random), SLEW, CLOCK, CHAIN. PLAY sets the step's gate. A pin into SEQ ADDRESS on the mod matrix sweeps the steps, Serge style; otherwise ADVANCE gates move it, otherwise the master clock. KEYS: the keyboard sequencer (below). |
| 8 FUNCTION GEN | SEL + ENV | FG 1 to 4 (RISE, FALL, CURVE lin / exp / log, MODE cycle / once / gate; each sends an end-of-cycle gate), S&H 1 and 2 (INPUT: noise or any modulator, SLEW; clocked from the gate matrix), and the QUAD LFO (SIN+, SIN−, COS+, COS−). |
| 9 MOD MATRIX | ARP | The EMS-style pin grid: 19 sources by 38 destinations (tilt, stretch, split, the filters, pans and pitches, the strikes, the grids' rules, mutation and speed, the FGs' rates, the sequencers' addresses, the effects). K1 or SELECT the source, K2 or PRESETS the destination, K3 the depth (±), PLAY sets or pulls a pin. |
| X GATE MATRIX | SEL + ARP | 25 gate sources (generators, their LFOs, EOCs, quantiser note gates, clock divisions, the grids' populations, the sequencers' step gates, KEYS) to 16 destinations (the strikes, FG triggers, S&H clocks, grid steps, sequencer advance and reset). Pins light with the live gate. |
| F FX | FX | Four parts, stepped with FX or OCT. PHASER: stereo, 2 to 16 stages, its sides swept a quarter cycle apart by its LFO, or by the mod matrix (PHASER LEFT / RIGHT); PRESETS 22 / 11k, PLAY zeroes the feedback. DELAY: two shift registers, left and right, CROSS for ping-pong; PLAY clears. REVERB: Arcturus's plate; PRESETS 22 / 11k, PLAY freezes. ROUTING: the order of the three and the links between them. Each effect has a send from each path (SEND A, SEND B). |
| G GLOBAL | GLO | TUNING: 21 tunings and their scales, the root; SELECT picks a path, PLAY quantises it, K4 its glide. VDU: phosphor colour (green, aqua, amber, white) and brightness. ENGINE: ENG VER (V1 8 bit, V2 12 bit), CLOCK (internal or MIDI) and the CPU meter. |
| D DISK | SAVE | A:\SYSTEM holds INIT and RANDOM, B:\PATCHES your 16 slots. SEL loads (press twice), SAVE saves (twice to overwrite), PLAY names a patch. |

### The keyboard sequencer

On the SEQUENCERS page, OCT past SEQ 3 to KEYS: a piano roll of up to 32 steps, one note at a time.

- **Placing notes.** PLAY brings up a note bar, flashing. PRESETS moves it up and down, ALGORITHM along, SELECT makes it
  longer (clockwise) or shorter. PLAY again puts it down. PLAY on a note picks it up to move it; SEL drops the bar
  (which deletes a note you picked up). Hold SEL and press PLAY to clear the roll.
- **Knobs.** STEPS (1 to 32), DIR, CLOCK (turn left to follow the master clock, /32 to /1; turn right for its own,
  0.05 to 12 steps a second), RUN.
- **Recording.** REC records from the keys as it runs (and starts it): notes land on the nearest step, and a held key
  makes a longer note.
- **Playing.** Its notes move the paths that track the keyboard. While it runs, a key transposes it. Each note sends a
  gate (KEYS) to the gate matrix: pin it to a strike to hear each note struck. The paths have no envelope of their
  own, so empty steps hold the last pitch; the strikes and FGs give the notes their shape.

## Patch ideas

Start from INIT (DISK page, SELECT up to INIT, SEL twice) and build up. These are loose starting points; the fun is in
what goes wrong. Roll a few RANDOM patches first, just to get a sense for how the system sounds and what it can do.

**A drone with a line over it.**

1. On the SPLIT page, OCT to PATH A and press PLAY: path A comes off the keys and holds its note.
2. On SEQUENCERS, OCT to KEYS. Place a few notes, turn CLOCK to the right for a rate of its own and RUN on.
3. In the gate matrix, pin KEYS to STRIKE 2 and set strike 2 to Mallet: each note is struck on path B.
4. Give path B some glide (GLOBAL, TUNING, K4) and press keys while it runs: the line moves, the drone stays.

**A self-playing melody.**

1. On LOGIC, set generator 1's input to one of grid A's cells, its logic to AND, its LFO on the clock.
2. In the gate matrix, pin GEN1 to S&H 1 CLOCK.
3. In the mod matrix, pin SSG1 to PITCH B. On GLOBAL, TUNING, SELECT path B and press PLAY to quantise it; pick a
   scale you like.
4. Pin QNT B to STRIKE 2 in the gate matrix: each new note strikes itself.

**Rhythms from the cells.**

- Generators can take any single cell as their input: pick cells from both grids, gate them with clocked LFOs through
  AND or XOR, and send them to the strikes and FG triggers.
- Turn LOCK off and run grid B at a different rate from grid A for patterns that drift against each other.
- A trill on strike 1 (bank 3) turns single hits into rolls; pin a generator to GRID A STEP to clock a grid from logic.

**Rules that evolve.**

- Pin a slow FG, or a slewed sequencer, to GRID A RULE: the grid passes through other rules as it moves, from order to
  chaos and back. The RULE knob shows where it is (45>110).
- A little MUTATE and RESCUE keep a grid alive; pin POP A (grid A's population) back into TILT or a strike's partial
  to close the loop.

**Vowel sequences, Serge style.**

1. Set path A's filter to VOWEL and pin SEQ 1 to FILTER A.
2. Pin FG1 to SEQ 1 ADDRESS and let FG1 cycle: it sweeps the vowel steps, speeding up and slowing down with its curve.

**Slow motion and big spaces.**

- Pin SIN+ and COS+ to PAN A and PAN B: the paths circle each other.
- Send path A to the phaser, 16 stages, feedback well up and a slow rate, for the whoosh; pin two unrelated modulators
  to PHASER LEFT and RIGHT to sweep the sides apart.
- PLAY on the reverb freezes it and holds a moment of the texture while the grids move on.
