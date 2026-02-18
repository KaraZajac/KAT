# Scripts

## analyze_sub_vag.py

Analyzes a Flipper `.sub` file against the VAG decoder logic in `src/protocols/vag.rs` to see why a signal does not decode as VAG (or even appear as Unknown).

**Usage:**
```bash
python scripts/analyze_sub_vag.py path/to/file.sub
python scripts/analyze_sub_vag.py "IMPORTS/VOLKSWAGEN AUDI/Test_55_unlock_and_55_lock_suran.sub"
```

**What it does:**
- Parses the .sub file (Frequency + RAW_Data: positive = HIGH, negative = LOW, duration in µs).
- Checks whether the file frequency is within VAG’s supported range (433.92 / 434.42 MHz, 2% tolerance as in KAT).
- Simulates the VAG decoder’s **Reset** step: the first HIGH pulse must be 300±79 µs (Type 1/2) or 500±79 µs (Type 3/4). Reports the first few pulses and why they pass or fail.
- Scans the full stream for the first HIGH pulse that matches 300±79 or 500±79 (VAG-like preamble), for both normal and inverted polarity.

**Why a file might not show up at all:**  
KAT only creates a capture (including “Unknown”) when some decoder returns a result. The VAG decoder only leaves the Reset state when it sees a HIGH pulse of 300±79 or 500±79 µs. If the stream starts with other data (e.g. 133 µs HIGH), the decoder never leaves Reset and never emits a decode, so no capture is created for that stream.
