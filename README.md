<p align="center"><img src="assets/card.png" width="240" alt="Quorum's boot screen: a petri dish of growing colonies"></p>

# Quorum

**A Cellular Synthesis System for the M-VAVE FM-1.** An experimental, self-playing additive instrument for
generative, evolving soundscapes, where cellular automata decide which harmonics sound, with the sound and look of
early digital machines.

Quorum is the sister of [Arcturus S6](../../tree/main). Arcturus sweeps wavetables; Quorum builds its sound from 64
sine partials in the manner of the Kawai K5, and lets two rows of cellular automata switch them on and off. Strikes
ring parts of the overtone series by hand or by logic, every partial is split into two paths with their own filters,
and Serge-style modulators, sequencers and two patch matrices keep the whole thing moving. The engine runs at 22.05kHz
in 12 bit (or 8 bit, after the Fairlight CMI's first models), and the screen is a phosphor CRT with Fairlight-style
pages.

<p align="center"><img src="docs/screens.png" width="720" alt="The boot screen, a grid, a strike, a path's filter, the keyboard sequencer, the mod matrix, the phaser and the disk page"></p>

- **64 sine partials** with slew, tilt, stretch (bells and inharmonic spectra) and an always-on fundamental.
- **2 cellular automata grids** of 32 cells: any of the 256 elementary rules, mutation, rescue when a grid dies or
  loops, five ways to share the partials between them, each on its own clock, in lockstep, or on MIDI clock.
- **2 strikes:** Strike, Mallet (marimba bar ratios), Bow and Breath ring a partial's overtone series (all, odd or
  even), with low-pass-gate fall-off and trills that rise, fall or hump.
- **A two-path split:** by cutoff, band, odd / even, every Nth, grid A / B or source, with soft edges. Each path has a
  vowel filter or a 12 / 24dB band pass, level, pan, pitch offset and its own keyboard tracking (hold one as a drone
  and play the other).
- **4 logic generators:** a cell, a clock division or another generator into AND, OR, XOR, NAND, NOR or XNOR with a
  square LFO, free or clocked.
- **Modulation:** 4 Serge-style function generators, 2 sample-and-holds, a quadrature LFO, 3 slew sequencers (swept
  Serge-style by any modulator) and a **keyboard sequencer** with a piano roll, step and live recording.
- **An EMS-style mod matrix** (19 sources, 38 destinations, the grids' rules among them) and a **gate matrix**.
- **21 tunings** (just intonation, shruti, Bohlen-Pierce, the Carlos scales and more) with per-path quantisers.
- **Effects:** a stereo phaser (2 to 16 stages, 22 or 11kHz), two shift-register delays, Arcturus's plate reverb with
  freeze, in any order, with a send from each path.
- **16 named patches** on a DOS-style disk page, INIT and RANDOM, a new slow drone at every power-on.

**[Quick start guide](docs/QUICKSTART.md)**: getting around, page by page, and patch ideas (a drone with a melody
over it, self-playing melodies, rhythms from the cells, rules that evolve).

## Install

The easy way: the [FM-1 Firmware Hub](https://thefirmwarehub.com), in Chrome or Edge, installs it over USB with
one click, and takes you back to the official firmware or any other the same way.

By hand: download `quorum-X.Y.fwsc` from the [releases](../../releases), then

```sh
pip3 install mido python-rtmidi                # (Windows: not needed, see below)
python3 tools/fm1_install.py quorum-X.Y.fwsc
```

On Windows, put the `.fwsc`, `tools/fm1_install.py`, `tools/fm1_winmidi.py` and `tools/windows/install.bat` in one
folder, rename the firmware `quorum.fwsc` and double-click `install.bat` (it needs Python 3).

**If it won't start:** hold **OCT−** while switching on. That's the USB rescue: no sound, only the updater, so you
can install it again or install another firmware from the hub. Installing firmware is at your own risk.

## Status

Tested on one FM-1. It's an experimental instrument written by machine (Claude, by Anthropic) with human guidance:
expect rough edges, and tell us about them in the [issues](../../issues).

## Build it yourself

Quorum is built on [Dinghy](https://github.com/zvenson/dinghy), so it builds the same way: Linux on x86-64
(or Docker), Python 3 with Pillow, the JieLi toolchain and three files of the JieLi AC79 SDK.

```sh
pip3 install Pillow mido python-rtmidi
tools/get_toolchain.sh
git clone --depth 1 --branch AC79NN_SDK_V1.2.1_2023-12-13 \
    https://gitee.com/Jieli-Tech/fw-AC79_AIoT_SDK.git ~/fw-AC79_AIoT_SDK
./build.sh                                     # -> build/quorum.fwsc
tests/run_host.sh tests/scripts/quorum10.txt   # the firmware on your computer: build/host/*.wav and screen PNGs
```

Quorum lives on the `cellular-s6` branch. GitHub builds every push to it, and a tag `quorum-vX.Y` builds, tests and
publishes a release with a build attestation. The firmware itself is [firmware/src/dinghy.c](firmware/src/dinghy.c)
and [firmware/src/verb.h](firmware/src/verb.h) (the reverb, from Arcturus S6); the lookup tables come from
[tools/gen_ca_tables.py](tools/gen_ca_tables.py). Dinghy's own docs, for the platform under it:
[docs/API.md](docs/API.md), [docs/RULES.md](docs/RULES.md), [docs/MAKE-IT-YOURS.md](docs/MAKE-IT-YOURS.md).

## Credits and licence

GPL-3.0-only: you may change and share it, and whoever gets your firmware gets its source. Details in
[LICENSING.md](LICENSING.md).

Quorum by Zeptosecond DSP. Built on [Dinghy](https://github.com/zvenson/dinghy) by zvenson, on the platform of
[Felucca](https://github.com/hugelton/Felucca) by Leo Kuroshita (Hügelton Instruments) as
[SLOOP](https://github.com/isod89/sloop-fm1) (isod89) carries it, with the SMK-37 layer by M. Geisler (genkigenki)
and the loader's screen after [Skiff](https://github.com/illuminus5/skiff) (illuminus5). The screen font is
font8x8 by Daniel Hepper (public domain). Not made or endorsed by M-VAVE.
