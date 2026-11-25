#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Part of KAT (https://github.com/KaraZajac/KAT). See https://kat.karazajac.io/mrk2.html
"""Disassembler for NXP's MRK II core, the 8-bit RISC in PCF79xx key-fob chips (PCF7941, PCF7952, PCF7961, ...).

NXP never published the instruction set; this decoding was worked out from 81 firmware images and checked
against what NXP's public documents do give (see mrk2.html). Usage:

    python3 mrk2.py erom.bin                 # listing of the code reached from the reset/interrupt vectors
    python3 mrk2.py erom.bin --all           # every word, code or not
    python3 mrk2.py erom.bin --base 0x800    # for images whose E-ROM sits at word 0x800 (vectors point above 0x800)

Encoding facts used here:
  * A dump as programmers read it (`*EROM.bin`) holds every byte bit-reversed: reverse each byte's bits, then
    read 16-bit little-endian words. Program addresses in JMP/CALL are word addresses.
  * Byte instructions: [op:4][x:1][k:3][operand:8]. x=1: `op Rk,#imm8`. x=0: operand = [dir:1][addr:7];
    dir=0 `Rk = Rk op [addr]`, dir=1 `[addr] = [addr] op Rk`. op 0-7 = ADD ADDC SUB SKEQ AND OR XOR MOV
    (SKEQ compares, sets C when Rk is lower, and skips the next instruction when equal).
  * Bit instructions: [op:4][x:1][s:1][bit:3][addr:7]. C0-C3 CLR, C4-C7 SETB, C8-CB CPL, D0-D3 MOV C,bit,
    D4-D7 MOV bit,C, D8-DB skip if the bit is 0, DC-DF skip if it is 1.
  * Direct addresses are 7 bits: 00-07 R0..R7, 08-11 indirect "ports" (@R0, @R1+, the bit pointer, ...),
    12-3F special function registers, 40-7F RAM. RAM above 7F is reached through the pointers only.
  * Conditionals are skips: an instruction skips the next one, which is usually a JMP.
"""
import argparse
import pathlib

_REV = bytes(int(f"{i:08b}"[::-1], 2) for i in range(256))
RET, RETI, SYS, TRAP = 0xE500, 0xED00, 0xF500, 0xF580

# Special function registers as the PCF7922 data sheet lists them (MRK II, 2nd generation); the PCF7941-class
# parts agree wherever the firmware lets us check. 08-11 are the indirect "ports" (see mrk2.html).
SFR = {0x08: "@R0", 0x09: "@R1", 0x0A: "@R2", 0x0B: "@R3", 0x0C: "@R0+", 0x0D: "@R1+", 0x0E: "@-R0", 0x0F: "@-R1",
       0x10: "@dbp+", 0x11: "@dbp",
       0x12: "PSW", 0x13: "SP", 0x14: "SBIT", 0x15: "SPTR", 0x16: "IE", 0x17: "IFF", 0x18: "T0", 0x19: "TR0", 0x1A: "TCON",
       0x1B: "WTCON", 0x1D: "MODCON", 0x1E: "CRYP1", 0x1F: "CRYP2", 0x20: "P1OUT", 0x21: "P1INS", 0x22: "P1DIR", 0x23: "P2OUT",
       0x24: "P2INS", 0x25: "P2DIR", 0x26: "PCON", 0x27: "SCSL", 0x28: "T1CON1", 0x29: "T1CON2", 0x2A: "T1CAP", 0x2B: "T1CMP",
       0x2C: "EEDAT", 0x2D: "EECON", 0x2E: "EEADR", 0x2F: "VCON", 0x30: "XFCON", 0x31: "PACON", 0x32: "TXCON1", 0x33: "TXCON2"}

# Named bits (PCF7922 bit tables). (0x10, 0) is the data bit pointer's port: the next bit at SPTR/SBIT.
BITS = {(0x12, 7): "C", (0x14, 7): "SBIT.MAP", (0x16, 7): "EA", (0x16, 5): "EE", (0x16, 3): "ET1O", (0x16, 2): "ET0",
        (0x16, 1): "EP", (0x1A, 0): "TRS0", (0x1B, 0): "WCLR", (0x1D, 7): "MDB", (0x1D, 6): "EFM", (0x1D, 5): "EAM",
        (0x1E, 7): "CRIO", (0x26, 0): "IDLE", (0x26, 1): "POFF", (0x26, 3): "RST", (0x26, 4): "PMODE", (0x28, 7): "T1RUN",
        (0x2C, 7): "EEIO", (0x2D, 7): "EE_BUSY", (0x2F, 7): "VCMP", (0x10, 0): "@dbp+", (0x11, 0): "@dbp"}

OPS = ["ADD", "ADDC", "SUB", "SKEQ", "AND", "OR", "XOR", "MOV"]
UNARY = {0xE0: "INC", 0xE2: "INCSZ", 0xE4: "INCSNZ", 0xE6: "SKIPZ", 0xE8: "DEC", 0xEA: "DECSZ", 0xEC: "DECSNZ",
         0xEE: "SKIPNZ", 0xF0: "SHR", 0xF2: "SHL", 0xF4: "RRC", 0xF6: "RLC", 0xF8: "SWAP", 0xFC: "CLR", 0xFE: "CPL"}
SKIPPING = {0xE2, 0xE4, 0xE6, 0xEA, 0xEC, 0xEE}


def native(raw: bytes) -> list:
    """Words of a dump in the CPU's own order (bit-reverse each byte, 16-bit little-endian)."""
    nb = raw.translate(_REV)
    return [int.from_bytes(nb[i:i + 2], "little") for i in range(0, len(nb) - 1, 2)]


def load(path) -> list:
    return native(pathlib.Path(path).read_bytes())


def is_skip(x: int) -> bool:
    """Instructions that may skip the next one."""
    hi = x >> 8
    return x >> 12 == 3 or 0xD8 <= hi <= 0xDF or hi in SKIPPING


def traverse(words, base=0):
    """Code reached from the reset and interrupt vectors (words 0..10), following JMP, CALL and skips.
    Returns (code word indexes, CALL targets, JMP targets); targets are addresses (index + base)."""
    n = len(words)
    roots = [0] + [i for i in range(1, 11) if i < n and (words[i] >> 12 == 8 or words[i] == RETI)]
    code, calls, jumps = set(), set(), set()
    todo = list(roots)
    while todo:
        i = todo.pop()
        while 0 <= i < n and i not in code:
            x = words[i]; code.add(i); t = x >> 12
            if x in (RET, RETI):
                break
            if t == 8:
                tgt = x & 0xFFF; jumps.add(tgt)
                if 0 <= tgt - base < n: todo.append(tgt - base)
                break
            if t == 0xA:
                tgt = x & 0xFFF; calls.add(tgt)
                if 0 <= tgt - base < n: todo.append(tgt - base)
            elif is_skip(x):
                todo.append(i + 2)
            i += 1
    return code, calls, jumps


def name(a: int) -> str:
    return f"R{a}" if a < 8 else SFR.get(a, f"{a:02X}h")


def dis(x: int) -> str:
    """One instruction word as text. Unknown words come out as `?hh  ll`."""
    if x == RET: return "RET"
    if x == RETI: return "RETI"
    if x == SYS: return "SYS              ; ROM library call, function number in R4"
    if x == TRAP: return "TRAP"
    if x == 0xFFFF or x >> 8 == 0xE1: return "NOP?"
    t, hi, lo = x >> 12, x >> 8, x & 0xFF
    if t == 8: return f"JMP   {x & 0xFFF:03X}"
    if t == 0xA: return f"CALL  {x & 0xFFF:03X}" + ("      ; mask ROM (4 KB parts)" if (x & 0xFFF) >= 0x800 else "")
    if t <= 7:
        k = (x >> 8) & 7; op = OPS[t]
        if (x >> 11) & 1:
            return f"{op:5} R{k},#{lo:02X}h"
        a = name(lo & 0x7F)
        if op == "SKEQ":
            return f"SKEQ  R{k},{a}"
        return f"{op:5} {a},R{k}" if lo & 0x80 else f"{op:5} R{k},{a}"
    if 0xC0 <= hi <= 0xDF and not 0xCC <= hi <= 0xCF:
        b, a = (x >> 7) & 7, x & 0x7F
        bit = BITS.get((a, b)) or f"{name(a)}.{b}"
        op = (x >> 10) & 3
        if t == 0xC:
            return f"{('CLR', 'SETB', 'CPL')[op]:5} {bit}"
        return (f"MOV   C,{bit}", f"MOV   {bit},C", f"SKIP  if {bit}=0", f"SKIP  if {bit}=1")[op]
    if hi in UNARY: return f"{UNARY[hi]:5} {name(lo & 0x7F)}"
    if hi == 0xF1: return f"MOVC  {name(lo & 0x7F)},@R5R4  ; code byte at R5:R4"
    if hi == 0xF7: return f"INTA  #{lo:02X}h         ; clear the IFF bits that are 0"
    if hi == 0xFA: return f"SETFF? {name(lo & 0x7F)}"
    return f"?{hi:02X}   {lo:02X}h"


def listing(words, base=0, everything=False) -> str:
    code, calls, jumps = traverse(words, base)
    out = []
    for i, x in enumerate(words):
        if not everything and i not in code:
            continue
        a = i + base
        if a in calls: out.append(f"\n; ---- function {a:03X} ----")
        elif a in jumps: out.append(f"L{a:03X}:")
        out.append(f"   {a:03X}: {x:04X}  {dis(x)}" + ("" if i in code else "      ; (not reached)"))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="Disassemble an NXP MRK II (PCF79xx) E-ROM image.")
    ap.add_argument("erom", help="E-ROM dump as read by a programmer (*EROM.bin)")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=None,
                    help="word address of the image's first word (default: 0, or 0x800 when the vectors point there)")
    ap.add_argument("--all", action="store_true", help="list every word, not only reached code")
    args = ap.parse_args()
    words = load(args.erom)
    base = args.base
    if base is None:
        base = 0x800 if len(words) == 2048 and words[0] >> 12 == 8 and (words[0] & 0xFFF) >= 0x800 else 0
    print(listing(words, base, args.all))


if __name__ == "__main__":
    main()
