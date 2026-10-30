#!/usr/bin/env python3
"""Preview the wavetables on the computer: firmware/src/wt_tables.h ->
   build/preview/tables.wav  (each table swept from its first wave to its last, as the engine plays it)
   build/preview/tables.png  (each table drawn in 3D, as page 3 draws it)

Usage: python3 tools/preview_tables.py [seconds per table] [MIDI note]
"""
import re
import sys
import wave
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

root = Path(__file__).resolve().parent.parent
src = (root / "firmware" / "src" / "wt_tables.h").read_text()
names = re.findall(r'"([^"]+)"', src.split("WT_NAME")[1].split(";")[0])
nums = [int(v) for v in re.findall(r"-?\d+", src.split("WT_SAMPLES] = {", 1)[1])]
W, S = 64, 128
tables = np.array(nums[: len(names) * W * S], dtype=float).reshape(len(names), W, S)

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
note = int(sys.argv[2]) if len(sys.argv) > 2 else 48
fs, f0 = 44100, 440.0 * 2 ** ((note - 69) / 12)
out = root / "build" / "preview"
out.mkdir(parents=True, exist_ok=True)

# the sound: no sample interpolation (the engine's PPG grit), a crossfade between neighbouring waves
audio = []
for t in tables:
    n = int(secs * fs)
    ph = (np.arange(n) * f0 / fs) % 1.0
    idx = (ph * S).astype(int)
    pos = np.clip(np.arange(n) / n * 1.25 - 0.1, 0, 1) * (W - 1)   # hold at both ends a moment
    w0 = pos.astype(int)
    w1 = np.minimum(w0 + 1, W - 1)
    fr = pos - w0
    x = t[w0, idx] * (1 - fr) + t[w1, idx] * fr
    env = np.minimum(1, np.minimum(np.arange(n), n - np.arange(n)) / (0.02 * fs))
    audio.append(x / 127 * 0.5 * env)
    audio.append(np.zeros(int(0.4 * fs)))
a = (np.concatenate(audio) * 32767).astype(np.int16)
with wave.open(str(out / "tables.wav"), "wb") as f:
    f.setnchannels(1)
    f.setsampwidth(2)
    f.setframerate(fs)
    f.writeframes(a.tobytes())

# the picture: 4 x 3 panels in green phosphor, back to front
PW, PH = 240, 200
img = Image.new("RGB", (4 * PW, 3 * PH), (0, 0, 0))
d = ImageDraw.Draw(img)
for i, (name, t) in enumerate(zip(names, tables)):
    ox, oy = (i % 4) * PW, (i // 4) * PH
    for w in range(60, -1, -4):
        x0, y0 = ox + 8 + w * 3 // 2, oy + 150 - w * 9 // 5
        pts = [(x0 + k, y0 - t[w, k] * 28 / 127) for k in range(S)]
        d.line(pts, fill=(28, 105, 48) if w else (150, 255, 160), width=1 if w else 2)
    d.text((ox + 6, oy + 182), f"{i + 1}  {name}", fill=(70, 200, 95))
img.save(out / "tables.png")
print(out / "tables.wav", out / "tables.png", f"{len(names)} tables, {secs:g} s each")
