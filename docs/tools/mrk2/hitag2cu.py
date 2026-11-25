#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Part of KAT (https://github.com/KaraZajac/KAT). See https://kat.karazajac.io/mrk2.html
"""The PCF79xx Calculation Unit in HITAG2 mode, as the PCF7922 data sheet describes it (functions 2, 4 and 6),
with HITAG2's filter and feedback taken from KAT's src/protocols/hitag2.rs (filter48 / feedback48).

Every access to CRYP1 clocks the 48-bit shift register once; it shifts towards the higher bits:
  Load0-HITAG2 (CRM = 2): bit 0 <- the bit written to CRIO;            CRIO <- filter(new state)
  LF-HITAG2    (CRM = 4): bit 0 <- the linear feedback;                CRIO <- filter(new state)
  OWF-HITAG2   (CRM = 6): bit 0 <- the bit written ^ filter(state);    CRIO <- filter(new state)
A read returns CRIO as it stands, then clocks. The Load16 and Enhanced (64-bit) modes are not modelled.
"""
MASK48 = (1 << 48) - 1
FEEDBACK_TAPS = 0xB380_8322_0073


def _group(truth, taps, shift, n):
    t = []
    for w in range(n):
        idx = (((w >> taps[0]) & 1) << 3) | (((w >> taps[1]) & 1) << 2) | (((w >> taps[2]) & 1) << 1) | ((w >> taps[3]) & 1)
        t.append(((truth >> idx) & 1) << shift)
    return t


_G0 = _group(0x2C79, (0, 1, 3, 4), 0, 32)
_G1 = _group(0x6671, (0, 1, 3, 7), 1, 256)
_G2 = _group(0x6671, (0, 3, 5, 9), 2, 1024)
_G3 = _group(0x6671, (0, 2, 4, 5), 3, 64)
_G4 = _group(0x2C79, (0, 2, 3, 12), 4, 8192)


def filter48(s: int) -> int:
    i = _G0[(s >> 41) & 0x1F] | _G1[(s >> 32) & 0xFF] | _G2[(s >> 21) & 0x3FF] | _G3[(s >> 14) & 0x3F] | _G4[(s >> 1) & 0x1FFF]
    return (0x7907287B >> i) & 1


def feedback48(s: int) -> int:
    return bin(s & FEEDBACK_TAPS).count("1") & 1


def run48(state: int, key32: int, iv: int) -> int:
    """HITAG2 as KAT computes it (hitag2::run48): from `state` = uid(32) ‖ key16, 32 rounds shifting in
    (key32 ^ iv, most significant bit first) ^ filter, then 32 output bits, the first one the most significant."""
    s = state; inj = key32 ^ iv
    for i in range(31, -1, -1):
        s = ((s << 1) | (((inj >> i) & 1) ^ filter48(s))) & MASK48
    out = 0
    for _ in range(32):
        out = (out << 1) | filter48(s)
        s = ((s << 1) | feedback48(s)) & MASK48
    return out


class CalcUnit:
    """Attach to mrk2emu.CPU as `cpu.cu = CalcUnit()`."""

    def __init__(self):
        self.sr = 0
        self.crio = 0
        self.trace = []

    def clock(self, mode: int, written):
        out = self.crio
        if mode == 2:
            self.sr = ((self.sr << 1) | (written or 0)) & MASK48
        elif mode == 6:
            self.sr = ((self.sr << 1) | ((written or 0) ^ filter48(self.sr))) & MASK48
        elif mode == 4:
            self.sr = ((self.sr << 1) | feedback48(self.sr)) & MASK48
        else:
            raise NotImplementedError(f"calculation unit mode {mode} (Load16 or Enhanced) is not modelled")
        self.crio = filter48(self.sr)
        self.trace.append((mode, written, out))
        return out
