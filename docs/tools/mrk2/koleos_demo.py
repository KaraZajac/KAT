#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Part of KAT (https://github.com/KaraZajac/KAT). See https://kat.karazajac.io/renault_koleos.html
"""Run the Renault Koleos key's firmware (part 285970036R) in the MRK II emulator and print what it transmits.

    python3 koleos_demo.py                      # the firmware files KAT publishes, next to this folder on the site
    python3 koleos_demo.py EROM.bin EEPROM.bin  # or explicit paths

The published EEPROM is a factory template: no ID, the default key. The demo writes a made-up identity into a
copy (UID 1A2B3C4D, key 5A6B + C0FFEE12, not any real key's), holds each of the four buttons in turn, decodes
the frames the firmware sends and checks their keystream against HITAG2 computed independently (hitag2cu.run48).
"""
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True                              # no __pycache__ beside the published files
import hitag2cu  # noqa: E402
import mrk2emu  # noqa: E402

FW = HERE.parent.parent / "firmware" / "renault" / "tp-285970036r"
UID, KEY16, KEY32 = 0x1A2B3C4D, 0x5A6B, 0xC0FFEE12        # made up
HITAG2_RET = 0x388                                          # RET of the firmware's HITAG2 routine (function 374)


def frames(tx):
    """Manchester frames from the (MDB, ticks) list: 43 ticks per half bit, LOW then HIGH = 1 (the firmware's own
    polarity), split at LOW gaps, each returned as the 104 bits after the sync (a run of 0 bits, then 0 1)."""
    runs = []                                     # (level, half bits)
    for level, ticks in tx:
        n = round(ticks / 43)
        if runs and runs[-1][0] == level:
            runs[-1][1] += n
        else:
            runs.append([level, n])
    segments, cur = [], []
    for level, n in runs:
        if level == 0 and n >= 20:                # a gap between frames
            if cur: segments.append(cur)
            cur = []
        else:
            cur += [level] * n
    if cur: segments.append(cur)
    out = []
    for body in segments:
        if len(body) % 2: body = body + [0]       # the last LOW half bit runs into the gap
        best = min(([(body[i], body[i + 1]) for i in range(s, len(body) - 1, 2)] for s in (0, 1)),
                   key=lambda pairs: sum(a == b for a, b in pairs))
        if sum(a == b for a, b in best) > 1:
            continue
        bits = "".join("1" if (a, b) == (0, 1) else "0" for a, b in best)
        k = bits.find("01", 8)                    # end of the sync: a run of 0 bits, then 0 1
        if k > 0 and len(bits) >= k + 2 + 104:
            out.append(bits[k + 2:k + 2 + 104])
    return out


def run(erom, eeprom, button):
    ee = bytearray(pathlib.Path(eeprom).read_bytes())
    ee[0:4] = UID.to_bytes(4, "big")              # page 0: the chip's ID
    ee[16:20] = KEY32.to_bytes(4, "big")          # page 4: key, 32 bits
    ee[22:24] = KEY16.to_bytes(2, "big")          # page 5, bytes 2-3: key, 16 bits
    cpu = mrk2emu.CPU.from_files(erom)
    cpu.ee[:] = ee
    cpu.m[0x13] = 0xC0                            # what the ROM's boot leaves in SP before it starts the application
    cpu.inputs = {0x21: 0x0F & ~(1 << button), 0x24: 0x07}      # the button held down (inputs are active low)
    cpu.cu = hitag2cu.CalcUnit()
    iv = None
    try:
        for _ in range(1_500_000):
            if cpu.n % cpu.irq_every == 0 and not cpu.skip:
                cpu.maybe_irq()
            pc = cpu.pc
            cpu.step()
            if pc == HITAG2_RET and iv is None:
                iv = int.from_bytes(cpu.m[0x80:0x84], "big")
    except (mrk2emu.Halt, mrk2emu.UnknownOp) as e:
        print(f"  (stopped: {e})")
    return frames(cpu.tx), iv


def main():
    erom, eeprom = (sys.argv[1], sys.argv[2]) if len(sys.argv) > 2 else (FW / "erom.bin", FW / "eeprom.bin")
    print(f"Renault Koleos key firmware 285970036R; test identity UID {UID:08X}, key {KEY16:04X} {KEY32:08X} (made up)")
    for button in range(4):
        fs, iv = run(erom, eeprom, button)
        print(f"\nbutton P1.{button}: {len(fs)} frames")
        for bits in fs[:3]:
            b = bytes(int(bits[i:i + 8], 2) for i in range(0, 104, 8))
            x = 0
            for v in b[:12]:
                x ^= v
            print(f"  {b[0]:02X} {b[1:5].hex().upper()} {b[5]:02X} {b[6]:02X} {b[7]:02X} {b[8:12].hex().upper()} {b[12]:02X}"
                  f"   command {b[5]:02X}, repeat {b[6] >> 2}, counter {((b[6] & 3) << 8) | b[7]:03X}, check {'ok' if x == b[12] else 'BAD'}")
        if fs and iv is not None:
            ks = int(fs[0][64:96], 2)
            want = hitag2cu.run48((UID << 16) | KEY16, KEY32, iv)
            print(f"  HITAG2(UID, key, IV {iv:08X}) = {want:08X}: {'matches' if want == ks else 'DOES NOT match'} the keystream sent")


if __name__ == "__main__":
    main()
