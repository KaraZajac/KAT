#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Part of KAT (https://github.com/KaraZajac/KAT). See https://kat.karazajac.io/mrk2.html
"""An instruction-level emulator for NXP's MRK II core (PCF79xx key-fob chips), built on the decoding in mrk2.py.

It runs a fob's own firmware far enough to watch it build and send a frame: buttons, EEPROM, the HITAG2
Calculation Unit (hitag2cu.py), timers, interrupts and the modulator. It is a model, not the chip: the mask
ROM is not modelled (a `SYS` call is only logged), Timer 1 ticks are counted, not timed, and a busy-wait on a
hardware flag is let through after a few turns. An instruction the decoding does not know stops the run with
UnknownOp instead of doing something invented. koleos_demo.py shows a complete run.

Hardware model:
  - data memory 256 bytes; R0..R7 = 00..07; PSW 12h (C = bit 7)
  - indirect ports: 08 @R0, 09 @R1, 0A @R2, 0B @R3, 0C @R0+, 0D @R1+, 0E @-R0, 0F @-R1,
    10 @dbp+ (the next bit at SPTR/SBIT, bit 0 of a byte first = its most significant), 11 @dbp
  - EEPROM 512 bytes: EECON 2Dh (page bits 6..3), EEADR 2Eh (page 2..0, byte, bit); EEIO = EEDAT.7 reads the
    addressed bit (bit 0 = most significant) and moves to the next one
  - CRYP1.7 (CRIO) / CRYP2 (mode): handed to `cu` (hitag2cu.CalcUnit) when one is attached
  - VCON.7 (comparator): 1 while VCON[3:0] <= `battery`
  - interrupts: Timer 1 compare (vector word 4) when EA, ET1O and T1RUN are set, else Timer 0 (word 3) when
    EA, ET0 and TRS0 are set; delivered every `irq_every` instructions and on SETB IDLE
  - `inputs`: {SFR address: value} forced before every instruction (e.g. a held button on P1INS)
  - `tx`: (MDB, T1CMP) after each Timer-1 interrupt = one transmitted chip and the ticks it lasts
"""
import pathlib
import random

from mrk2 import native, RET, RETI, SYS


class UnknownOp(Exception):
    pass


class Halt(Exception):
    """The firmware powered down (SETB POFF), reset itself (SETB RST) or stopped in a JMP-to-itself."""


class CPU:
    def __init__(self, words, eeprom=b"", base=0, msb_first=True, seed=1):
        self.w = words; self.base = base
        self.codeb = b"".join(x.to_bytes(2, "little") for x in words)
        self.m = bytearray(256)
        self.ee = bytearray(bytes(eeprom).ljust(512, b"\0")[:512])
        self.pc = 0; self.stack = []; self.n = 0; self.skip = False
        self.msb_first = msb_first
        self.mdb_log = []; self.crio_in = []; self.crio_out = []; self.sys_log = []; self.ee_reads = []
        self.rng = random.Random(seed)
        self.spin = {}; self.in_isr = 0; self.irq_every = 300; self.isr_log = []
        self.cu = None
        self.inputs = {}
        self.tx = []
        self.battery = 8

    @classmethod
    def from_files(cls, erom_path, eeprom_path=None, **kw):
        ee = pathlib.Path(eeprom_path).read_bytes() if eeprom_path else b""
        return cls(native(pathlib.Path(erom_path).read_bytes()), ee, **kw)

    # ---- memory and ports ----
    def ea(self, a):
        """Data address of a 7-bit operand, applying a port's pointer update once."""
        m = self.m
        if a == 0x08: return m[0]
        if a == 0x09: return m[1]
        if a == 0x0A: return m[2]
        if a == 0x0B: return m[3]
        if a == 0x0C: r = m[0]; m[0] = (r + 1) & 0xFF; return r
        if a == 0x0D: r = m[1]; m[1] = (r + 1) & 0xFF; return r
        if a == 0x0E: m[0] = (m[0] - 1) & 0xFF; return m[0]
        if a == 0x0F: m[1] = (m[1] - 1) & 0xFF; return m[1]
        if 0x10 <= a <= 0x11: raise UnknownOp(f"byte access to the bit port {a:02X}h at {self.pc:03X}")
        return a

    def rd(self, addr):
        if addr == 0x2C:
            return self.ee_bit() << 7
        if addr == 0x2F:
            v = self.m[0x2F]
            return (v & 0x7F) | ((1 if (v & 0x0F) <= self.battery else 0) << 7)
        return self.m[addr]

    def wr(self, addr, v):
        v &= 0xFF
        if addr == 0x1D and ((self.m[0x1D] ^ v) & 0x80):
            self.mdb_log.append((self.n, v >> 7, self.in_isr))
        self.m[addr] = v
        if addr == 0x19:
            self.m[0x18] = v                       # a reload loads the stopped timer 0

    # ---- bit pointer, EEPROM, calculation unit ----
    def dbp_bit(self, write=None, inc=True):
        sptr, idx = self.m[0x15], self.m[0x14] & 7
        pos = 7 - idx if self.msb_first else idx
        if write is None:
            b = (self.m[sptr] >> pos) & 1
        else:
            self.m[sptr] = (self.m[sptr] & ~(1 << pos)) | ((write & 1) << pos); b = write
        if inc:
            idx += 1
            if idx == 8:
                idx = 0; self.m[0x15] = (sptr + 1) & 0xFF
            self.m[0x14] = (self.m[0x14] & 0xF8) | idx
        return b

    def ee_addr(self):
        page = ((self.m[0x2D] & 0x0F) << 3) | (self.m[0x2E] >> 5)
        return page * 32 + ((self.m[0x2E] >> 3) & 3) * 8 + (self.m[0x2E] & 7)

    def ee_bit(self):
        p = self.ee_addr() & 0xFFF
        b = (self.ee[p >> 3] >> (7 - (p & 7))) & 1
        self.ee_reads.append(p)
        p = (p + 1) & 0xFFF
        self.m[0x2D] = (self.m[0x2D] & 0xF0) | (p >> 8)
        self.m[0x2E] = ((p >> 5) & 7) << 5 | (p & 0x1F)
        return b

    def crio(self, write=None):
        mode = self.m[0x1F] & 7
        out = self.cu.clock(mode, write) if self.cu is not None else self.rng.getrandbits(1)
        (self.crio_out if write is None else self.crio_in).append((mode, out if write is None else write))
        return out

    def bit_get(self, a, b):
        if a == 0x10: return self.dbp_bit()
        if a == 0x11: return self.dbp_bit(inc=False)
        if a == 0x1E and b == 7: return self.crio()
        if a == 0x2C and b == 7: return self.ee_bit()
        addr = self.ea(a)
        return (self.rd(addr) >> b) & 1 if addr == 0x2F else (self.m[addr] >> b) & 1

    def bit_set(self, a, b, v):
        if a == 0x10: self.dbp_bit(write=v); return
        if a == 0x11: self.dbp_bit(write=v, inc=False); return
        if a == 0x1E and b == 7: self.crio(write=v); return
        addr = self.ea(a)
        self.wr(addr, (self.m[addr] & ~(1 << b)) | ((v & 1) << b))

    @property
    def C(self): return self.m[0x12] >> 7

    @C.setter
    def C(self, v): self.m[0x12] = (self.m[0x12] & 0x7F) | ((v & 1) << 7)

    # ---- interrupts ----
    def irq(self, vec):
        self.stack.append(self.pc); self.pc = vec; self.in_isr += 1
        self.isr_log.append((self.n, vec))

    def maybe_irq(self):
        if self.in_isr or not (self.m[0x16] & 0x80):
            return
        if (self.m[0x16] & 0x08) and (self.m[0x28] & 0x80):
            self.m[0x17] |= 0x08; self.irq(4); return
        if (self.m[0x16] & 0x04) and (self.m[0x1A] & 0x01):
            self.m[0x17] |= 0x04; self.irq(3)

    # ---- one instruction ----
    def step(self):
        for a, v in self.inputs.items():
            self.m[a] = v
        x = self.w[self.pc - self.base]; pc0 = self.pc; self.pc += 1; self.n += 1
        if self.skip:
            self.skip = False; return
        t, hi, lo = x >> 12, x >> 8, x & 0xFF
        if x == RET:
            self.pc = self.stack.pop(); return
        if x == RETI:
            self.pc = self.stack.pop(); self.in_isr = max(0, self.in_isr - 1)
            if self.isr_log and self.isr_log[-1][1] == 4:
                self.tx.append((self.m[0x1D] >> 7, self.m[0x2B]))
            return
        if x == SYS:
            self.sys_log.append((self.n, self.m[4])); return
        if x == 0xFFFF or hi == 0xE1:
            return
        if t == 8:
            tgt = x & 0xFFF
            if tgt == pc0: raise Halt(f"JMP to itself at {pc0:03X}")
            self.pc = tgt; return
        if t == 0xA:
            self.stack.append(self.pc); self.pc = x & 0xFFF; return
        if t <= 7:
            k = (x >> 8) & 7
            if (x >> 11) & 1:
                addr, to_mem, operand = None, False, lo
            else:
                addr = self.ea(lo & 0x7F); to_mem = bool(lo & 0x80)
                operand = None if (t == 7 and to_mem) else self.rd(addr)
            r = self.m[k]
            if t == 7:
                if (x >> 11) & 1: self.m[k] = lo
                elif to_mem: self.wr(addr, r)
                else: self.m[k] = operand
                return
            if t == 3:                                    # SKEQ: C = (Rk < operand), skip when equal
                self.C = 1 if r < operand else 0
                if r == operand: self.skip = True
                return
            a_, b_ = (operand, r) if to_mem else (r, operand)
            if t == 0: v = a_ + b_; self.C = v > 0xFF
            elif t == 1: v = a_ + b_ + self.C; self.C = v > 0xFF
            elif t == 2: v = a_ - b_; self.C = v < 0     # C = borrow (not yet proved)
            elif t == 4: v = a_ & b_
            elif t == 5: v = a_ | b_
            else: v = a_ ^ b_
            v &= 0xFF
            if to_mem: self.wr(addr, v)
            else: self.m[k] = v
            return
        if 0xC0 <= hi <= 0xDF and not 0xCC <= hi <= 0xCF:
            b, a = (x >> 7) & 7, x & 0x7F; op = (x >> 10) & 3
            if t == 0xC:
                if op == 0: self.bit_set(a, b, 0)
                elif op == 1:
                    self.bit_set(a, b, 1)
                    if (a, b) == (0x26, 0): self.maybe_irq()                     # SETB IDLE: wake on an interrupt
                    if (a, b) in ((0x26, 1), (0x26, 3)): raise Halt(f"{'POFF' if b == 1 else 'RST'} at {pc0:03X}")
                else: self.bit_set(a, b, self.bit_get(a, b) ^ 1)
                return
            if op == 0: self.C = self.bit_get(a, b); return
            if op == 1: self.bit_set(a, b, self.C); return
            val = self.bit_get(a, b); want = 0 if op == 2 else 1
            nxt = self.w[self.pc - self.base] if self.pc - self.base < len(self.w) else 0
            if val != want and 0x12 <= a <= 0x3F and nxt >> 12 == 8 and (nxt & 0xFFF) == pc0:
                c = self.spin.get(pc0, 0) + 1; self.spin[pc0] = c        # busy-wait on hardware: let it through
                if c > 3:
                    self.m[a] = (self.m[a] & ~(1 << b)) | (want << b); val = want; self.spin[pc0] = 0
            if val == want: self.skip = True
            return
        if 0xE0 <= hi <= 0xFE:
            a = lo & 0x7F
            if hi == 0xF7: self.m[0x17] &= lo; return                         # INTA #mask
            if hi == 0xF1:                                                     # MOVC: code byte at R5:R4
                addr = self.ea(a); self.wr(addr, self.codeb[((self.m[5] << 8) | self.m[4]) % len(self.codeb)]); return
            addr = self.ea(a); v = self.rd(addr)
            if hi == 0xE0: self.wr(addr, v + 1)
            elif hi == 0xE8: self.wr(addr, v - 1)
            elif hi == 0xE2: v = (v + 1) & 0xFF; self.wr(addr, v); self.skip = v == 0
            elif hi == 0xEA: v = (v - 1) & 0xFF; self.wr(addr, v); self.skip = v == 0
            elif hi == 0xE4: v = (v + 1) & 0xFF; self.wr(addr, v); self.skip = v != 0
            elif hi == 0xEC: v = (v - 1) & 0xFF; self.wr(addr, v); self.skip = v != 0
            elif hi == 0xE6: self.skip = v == 0
            elif hi == 0xEE: self.skip = v != 0
            elif hi == 0xF0: self.C = v & 1; self.wr(addr, v >> 1)
            elif hi == 0xF2: self.C = v >> 7; self.wr(addr, v << 1)
            elif hi == 0xF4: c = self.C; self.C = v & 1; self.wr(addr, (v >> 1) | (c << 7))
            elif hi == 0xF6: c = self.C; self.C = v >> 7; self.wr(addr, (v << 1) | c)
            elif hi == 0xF8: self.wr(addr, ((v << 4) | (v >> 4)) & 0xFF)
            elif hi == 0xFA: self.wr(addr, 0xFF)                               # not yet proved
            elif hi == 0xFC: self.wr(addr, 0)
            elif hi == 0xFE: self.wr(addr, ~v)
            else: raise UnknownOp(f"{x:04X} at {pc0:03X}")
            return
        raise UnknownOp(f"{x:04X} at {pc0:03X}")

    def run(self, max_steps=200000):
        for _ in range(max_steps):
            if self.irq_every and self.n % self.irq_every == 0 and not self.skip:
                self.maybe_irq()
            self.step()
        return "max"
