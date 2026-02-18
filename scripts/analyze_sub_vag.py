#!/usr/bin/env python3
from __future__ import annotations

"""
Analyze a Flipper .sub file against the VAG decoder logic (vag.rs) to determine
why the signal does not decode as VAG (or even as Unknown).

Usage:
  python scripts/analyze_sub_vag.py path/to/file.sub
  python scripts/analyze_sub_vag.py IMPORTS/Test_55_unlock_and_55_lock_suran.sub

Reads the .sub file (Frequency + RAW_Data: positive=HIGH, negative=LOW, µs),
then:
  1. Checks if frequency is in VAG's supported range (433.92 / 434.42 MHz, 2% tolerance).
  2. Simulates the VAG decoder state machine (Reset -> Preamble1/2 -> ...) and reports
     the first pulse index where the decoder fails and why.
  3. Scans for the first occurrence of a VAG-like preamble (HIGH 300±79 or 500±79 µs)
     so you can see if the frame appears later in the stream.
  4. Tries both normal and inverted polarity (KAT tries inverted if normal yields no decode).
"""

import re
import sys
from pathlib import Path

# --- Timing constants from vag.rs (Type 1/2 vs Type 3/4) ---
TE_SHORT_12 = 300   # Type 1/2 short
TE_LONG_12 = 600    # Type 1/2 long
TE_SHORT = 500      # Type 3/4 short
TE_LONG = 1000      # Type 3/4 long
REF_RESET_DELTA = 79
REF_PREAMBLE_SYNC = 80
REF_GAP1_DELTA = 79

# VAG supported frequencies (Hz) from vag.rs
VAG_FREQS = [433_920_000, 434_420_000]

# KAT frequency tolerance: diff < (f / 50) => 2%
def frequency_supported(file_hz: int) -> tuple[bool, str]:
    for f in VAG_FREQS:
        diff = abs(f - file_hz)
        if diff < (f / 50):
            return True, f"Frequency {file_hz} Hz is within 2% of VAG supported {f} Hz"
    return False, f"Frequency {file_hz} Hz is NOT in VAG supported list {VAG_FREQS} (2% tolerance)"


def parse_sub(path: Path) -> tuple[int, list[tuple[bool, int]]]:
    """Parse .sub file; return (frequency_hz, list of (level, duration_us))."""
    text = path.read_text()
    frequency_hz = 433_920_000
    raw_data: list[int] = []

    for line in text.splitlines():
        line = line.strip()
        if line.startswith("Frequency:"):
            frequency_hz = int(line.split(":", 1)[1].strip())
        elif line.startswith("RAW_Data:"):
            rest = line.split(":", 1)[1].strip()
            for word in rest.split():
                raw_data.append(int(word))

    # positive => HIGH (True), negative => LOW (False); duration in µs
    pairs: list[tuple[bool, int]] = []
    for v in raw_data:
        duration_us = abs(v)
        level = v >= 0
        pairs.append((level, duration_us))

    return frequency_hz, pairs


def analyze_reset_and_first_steps(pairs: list[tuple[bool, int]], invert: bool) -> list[str]:
    """
    Simulate VAG decoder from Reset: what does it expect, and at which index do we fail?
    Returns list of report lines.
    """
    lines: list[str] = []
    count = 0
    for i, (level, duration) in enumerate(pairs):
        if invert:
            level = not level
        if count >= 20:
            break
        if i == 0:
            lines.append("")
            lines.append("--- Step-by-step from start (first 20 pulses) ---")
            lines.append("Reset expects: first pulse must be HIGH, duration 300±79 µs (Type1/2) or 500±79 µs (Type3/4).")
        # Reset: only look at HIGH pulses
        if not level:
            lines.append(f"  [{i}] LOW  {duration:5} µs  -> Reset ignores LOW (stays in Reset)")
            count += 1
            continue
        # HIGH pulse
        count += 1
        if duration < TE_SHORT_12:
            diff = TE_SHORT_12 - duration
            if diff <= REF_RESET_DELTA:
                lines.append(f"  [{i}] HIGH {duration:5} µs  -> 300-duration={diff}<=79 -> would enter Preamble1 (Type1/2)")
            else:
                lines.append(f"  [{i}] HIGH {duration:5} µs  -> 300-duration={diff}>79 -> REJECT (stays Reset)")
        elif duration - TE_SHORT_12 <= REF_RESET_DELTA:
            lines.append(f"  [{i}] HIGH {duration:5} µs  -> duration-300<={duration - TE_SHORT_12}<=79 -> would enter Preamble1 (Type1/2)")
        else:
            if TE_SHORT - REF_RESET_DELTA <= duration <= TE_SHORT + REF_RESET_DELTA:
                lines.append(f"  [{i}] HIGH {duration:5} µs  -> 500±79 -> would enter Preamble2 (Type3/4)")
            else:
                lines.append(f"  [{i}] HIGH {duration:5} µs  -> not 300±79 nor 500±79 -> REJECT (stays Reset)")
    return lines


def find_first_vag_like_pulse(pairs: list[tuple[bool, int]], invert: bool) -> list[str]:
    """Find first HIGH pulse that looks like VAG preamble (300±79 or 500±79)."""
    lines: list[str] = []
    for i, (level, duration) in enumerate(pairs):
        if invert:
            level = not level
        if not level:
            continue
        ok_300 = (TE_SHORT_12 - REF_RESET_DELTA <= duration <= TE_SHORT_12 + REF_RESET_DELTA)
        ok_500 = (TE_SHORT - REF_RESET_DELTA <= duration <= TE_SHORT + REF_RESET_DELTA)
        if ok_300 or ok_500:
            kind = "300±79 (Type1/2)" if ok_300 else "500±79 (Type3/4)"
            lines.append(f"First VAG-like HIGH pulse at index {i}: {duration} µs ({kind})")
            return lines
    lines.append("No HIGH pulse in the entire file matches 300±79 or 500±79 µs.")
    return lines


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python analyze_sub_vag.py <path/to/file.sub>")
        sys.exit(1)
    path = Path(sys.argv[1])
    if not path.exists():
        print(f"File not found: {path}")
        sys.exit(1)

    print("=" * 60)
    print("VAG decoder analysis for:", path)
    print("=" * 60)

    freq_hz, pairs = parse_sub(path)
    print(f"\nParsed: frequency = {freq_hz} Hz ({freq_hz/1e6:.2f} MHz), {len(pairs)} level/duration pairs.")

    ok, msg = frequency_supported(freq_hz)
    print(f"\nFrequency check: {msg}")
    if not ok:
        print("  -> VAG decoder is never tried for this file (frequency filter in KAT).")

    # First few pulses
    print("\nFirst 10 pulses (level, duration_us):")
    for i, (level, dur) in enumerate(pairs[:10]):
        lvl = "HIGH " if level else "LOW  "
        print(f"  [{i}] {lvl} {dur} µs")

    # Why decoder fails from start
    report = analyze_reset_and_first_steps(pairs, invert=False)
    for line in report:
        print(line)

    # Find first VAG-like pulse in stream
    print("\n--- Search for VAG preamble in full stream ---")
    for line in find_first_vag_like_pulse(pairs, invert=False):
        print(line)
    print("\nWith INVERTED polarity (KAT tries this if normal fails):")
    for line in find_first_vag_like_pulse(pairs, invert=True):
        print(line)

    # Summary
    print("\n--- Summary (why no decode / no Unknown) ---")
    if not ok:
        print("1. Frequency is outside VAG supported range -> VAG decoder is skipped.")
    else:
        first_high = next((i for i, (l, d) in enumerate(pairs) if l), None)
        if first_high is None:
            print("1. No HIGH pulse in file (invalid or empty?).")
        else:
            _, first_high_dur = pairs[first_high]
            if first_high_dur < TE_SHORT_12 - REF_RESET_DELTA or (
                first_high_dur > TE_SHORT_12 + REF_RESET_DELTA
                and not (TE_SHORT - REF_RESET_DELTA <= first_high_dur <= TE_SHORT + REF_RESET_DELTA)
            ):
                print(f"1. First HIGH pulse is at index {first_high} with duration {first_high_dur} µs.")
                print("   VAG Reset requires first HIGH to be 300±79 µs (Type1/2) or 500±79 µs (Type3/4).")
                print("   So the decoder never leaves Reset and never produces a decode (or Unknown).")
        print("2. KAT only creates a capture (including Unknown) when a decoder consumes the stream")
        print("   and returns a result. VAG never returns because it stays in Reset.")
        print("3. If the VAG frame appears later in the file, the decoder would need to be run from")
        print("   that position (sliding window). Currently KAT feeds the stream from the start only.")


if __name__ == "__main__":
    main()
