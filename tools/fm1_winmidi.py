"""Windows' own MIDI (winmm.dll) for fm1_install.py: no mido, no python-rtmidi, nothing to pip install.

The same interface as fm1_install.py's MidoBackend / MidoLink:
  backend.input_names(), backend.output_names(), backend.open(in_name, out_name) -> link
  link.send(sysex_bytes) -> bool, link.read(timeout) -> sysex bytes or None, link.drain(), link.close(), link.lost

SysEx in: winmm fills buffers we hand it (midiInAddBuffer); a thread here collects the full ones in order,
gives them back, and cuts the stream into F0 ... F7 messages. SysEx out: midiOutLongMsg, waiting until it is sent.
"""
import ctypes
import queue
import sys
import threading
import time
from collections import deque

if sys.platform != "win32":
    raise ImportError("fm1_winmidi works on Windows only")

from ctypes import wintypes as W

winmm = ctypes.WinDLL("winmm")

MHDR_DONE = 0x1
MIDIERR_STILLPLAYING = 65
CALLBACK_NULL = 0
NBUF, BUFSZ = 16, 4096


class MIDIHDR(ctypes.Structure):
    _fields_ = [("lpData", ctypes.c_void_p), ("dwBufferLength", W.DWORD), ("dwBytesRecorded", W.DWORD),
                ("dwUser", ctypes.c_size_t), ("dwFlags", W.DWORD), ("lpNext", ctypes.c_void_p),
                ("reserved", ctypes.c_size_t), ("dwOffset", W.DWORD), ("dwReserved", ctypes.c_size_t * 8)]


class MIDIINCAPSW(ctypes.Structure):
    _fields_ = [("wMid", W.WORD), ("wPid", W.WORD), ("vDriverVersion", W.UINT),
                ("szPname", W.WCHAR * 32), ("dwSupport", W.DWORD)]


class MIDIOUTCAPSW(ctypes.Structure):
    _fields_ = [("wMid", W.WORD), ("wPid", W.WORD), ("vDriverVersion", W.UINT), ("szPname", W.WCHAR * 32),
                ("wTechnology", W.WORD), ("wVoices", W.WORD), ("wNotes", W.WORD), ("wChannelMask", W.WORD),
                ("dwSupport", W.DWORD)]


H = ctypes.c_void_p
PH = ctypes.POINTER(H)
PHDR = ctypes.POINTER(MIDIHDR)
U = W.UINT
for name, args in {
    "midiInGetNumDevs": [], "midiOutGetNumDevs": [],
    "midiInGetDevCapsW": [ctypes.c_size_t, ctypes.POINTER(MIDIINCAPSW), U],
    "midiOutGetDevCapsW": [ctypes.c_size_t, ctypes.POINTER(MIDIOUTCAPSW), U],
    "midiInOpen": [PH, U, ctypes.c_size_t, ctypes.c_size_t, W.DWORD],
    "midiOutOpen": [PH, U, ctypes.c_size_t, ctypes.c_size_t, W.DWORD],
    "midiInPrepareHeader": [H, PHDR, U], "midiInUnprepareHeader": [H, PHDR, U], "midiInAddBuffer": [H, PHDR, U],
    "midiInStart": [H], "midiInStop": [H], "midiInReset": [H], "midiInClose": [H],
    "midiOutPrepareHeader": [H, PHDR, U], "midiOutUnprepareHeader": [H, PHDR, U], "midiOutLongMsg": [H, PHDR, U],
    "midiOutReset": [H], "midiOutClose": [H],
}.items():
    f = getattr(winmm, name)
    f.argtypes, f.restype = args, U

HDRSZ = ctypes.sizeof(MIDIHDR)


def unique(names):
    """Windows can show two ports by one name: the second becomes 'NAME 2' (as rtmidi does; pair_output strips it)"""
    seen, out = {}, []
    for n in names:
        seen[n] = seen.get(n, 0) + 1
        out.append(n if seen[n] == 1 else f"{n} {seen[n]}")
    return out


def in_ports():
    names = []
    for i in range(winmm.midiInGetNumDevs()):
        c = MIDIINCAPSW()
        names.append(c.szPname if winmm.midiInGetDevCapsW(i, ctypes.byref(c), ctypes.sizeof(c)) == 0 else f"?{i}")
    return unique(names)


def out_ports():
    names = []
    for i in range(winmm.midiOutGetNumDevs()):
        c = MIDIOUTCAPSW()
        names.append(c.szPname if winmm.midiOutGetDevCapsW(i, ctypes.byref(c), ctypes.sizeof(c)) == 0 else f"?{i}")
    return unique(names)


def split_sysex(buf):
    """bytes received so far -> (complete F0..F7 messages, the rest to keep)"""
    msgs = []
    while True:
        s = buf.find(0xF0)
        if s < 0:
            return msgs, bytearray()
        e = buf.find(0xF7, s + 1)
        if e < 0:
            return msgs, buf[s:]
        nxt = buf.find(0xF0, s + 1)
        if 0 <= nxt < e:                # a message cut short: start again at the next F0
            buf = buf[nxt:]
            continue
        msgs.append(bytes(buf[s:e + 1]))
        buf = buf[e + 1:]


class WinLink:
    def __init__(self, backend, in_name, out_name):
        self.backend, self.name = backend, in_name
        self.q = queue.Queue()
        ins, outs = in_ports(), out_ports()
        if in_name not in ins or out_name not in outs:
            raise OSError("MIDI port not found")
        self.hin, self.hout = H(), H()
        r = winmm.midiInOpen(ctypes.byref(self.hin), ins.index(in_name), 0, 0, CALLBACK_NULL)
        if r:
            raise OSError(f"midiInOpen failed ({r}): is another program using the FM-1?")
        r = winmm.midiOutOpen(ctypes.byref(self.hout), outs.index(out_name), 0, 0, CALLBACK_NULL)
        if r:
            winmm.midiInClose(self.hin)
            raise OSError(f"midiOutOpen failed ({r}): is another program using the FM-1?")
        self.bufs, self.hdrs = [], deque()
        for _ in range(NBUF):
            b = ctypes.create_string_buffer(BUFSZ)
            h = MIDIHDR()
            h.lpData, h.dwBufferLength = ctypes.cast(b, ctypes.c_void_p), BUFSZ
            winmm.midiInPrepareHeader(self.hin, ctypes.byref(h), HDRSZ)
            winmm.midiInAddBuffer(self.hin, ctypes.byref(h), HDRSZ)
            self.bufs.append((b, h))
            self.hdrs.append((b, h))
        winmm.midiInStart(self.hin)
        self.running = True
        self.th = threading.Thread(target=self._pump, daemon=True)
        self.th.start()
        time.sleep(0.3)

    def _pump(self):
        pending = bytearray()
        while self.running:
            got = False
            while self.hdrs and self.hdrs[0][1].dwFlags & MHDR_DONE:   # winmm fills the buffers in order
                b, h = self.hdrs.popleft()
                pending += ctypes.string_at(b, h.dwBytesRecorded)
                h.dwBytesRecorded = 0
                h.dwFlags &= ~MHDR_DONE
                if self.running:
                    winmm.midiInAddBuffer(self.hin, ctypes.byref(h), HDRSZ)
                self.hdrs.append((b, h))
                got = True
            if got:
                msgs, pending = split_sysex(pending)
                for m in msgs:
                    self.q.put(m)
            else:
                time.sleep(0.001)

    @property
    def lost(self):
        return self.name not in self.backend.input_names()

    def send(self, pkt):
        data = ctypes.create_string_buffer(bytes(pkt), len(pkt))
        h = MIDIHDR()
        h.lpData, h.dwBufferLength = ctypes.cast(data, ctypes.c_void_p), len(pkt)
        if winmm.midiOutPrepareHeader(self.hout, ctypes.byref(h), HDRSZ):
            return False
        ok = winmm.midiOutLongMsg(self.hout, ctypes.byref(h), HDRSZ) == 0
        end = time.monotonic() + 3.0
        while ok and not h.dwFlags & MHDR_DONE:
            if time.monotonic() > end:
                winmm.midiOutReset(self.hout)
                ok = False
                break
            time.sleep(0.0005)
        for _ in range(2000):
            if winmm.midiOutUnprepareHeader(self.hout, ctypes.byref(h), HDRSZ) != MIDIERR_STILLPLAYING:
                break
            time.sleep(0.001)
        return ok

    def read(self, timeout):
        try:
            return self.q.get(timeout=max(timeout, 0))
        except queue.Empty:
            return None

    def drain(self):
        while not self.q.empty():
            self.q.get_nowait()

    def close(self):
        if not getattr(self, "running", False):
            return
        self.running = False
        self.th.join(1.0)
        try:
            winmm.midiInStop(self.hin)
            winmm.midiInReset(self.hin)          # gives back the buffers still queued
            for b, h in self.bufs:
                winmm.midiInUnprepareHeader(self.hin, ctypes.byref(h), HDRSZ)
            winmm.midiInClose(self.hin)
        finally:
            winmm.midiOutReset(self.hout)
            winmm.midiOutClose(self.hout)


class WinBackend:
    def input_names(self):
        return in_ports()

    def output_names(self):
        return out_ports()

    def open(self, in_name, out_name):
        return WinLink(self, in_name, out_name)
