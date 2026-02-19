# KAT scripts

## demod_sub_to_bits.py

Demodulate Flipper SubGhz RAW `.sub` files into a stream of `1` and `0` bits using duration-based on-off keying (OOK). The script parses level+duration pulses, classifies each as **short** or **long** against configurable timing, then outputs bits according to the chosen encoding.

### Input format

The script reads Flipper-style `.sub` files:

- **RAW_Data:** space-separated signed integers (one per pulse).
- **Positive value** = HIGH level, **negative** = LOW; **magnitude** = duration in microseconds.
- Optional **Frequency:** line (Hz). If missing, 433 920 000 is assumed.

Same convention as KAT’s `.sub` import and ProtoPirate’s raw file reader.

### Encoding modes

| Mode         | Description |
|-------------|-------------|
| **pwm**     | One bit per pulse: short duration → `0`, long duration → `1`. Unmatched durations are skipped. Default. |
| **manchester** | Short/long pulses fed into a Manchester state machine (Ford V0–style). Outputs decoded data bits. Long gaps reset state. |
| **raw**     | No duration decoding: one character per pulse, `1` = HIGH, `0` = LOW. |

### Timing parameters

Pulses are classified using nominal short/long durations and a tolerance:

- **--te-short** — Nominal short duration (µs). Default: 250.
- **--te-long**  — Nominal long duration (µs). Default: 500.
- **--te-delta** — Tolerance (µs): a pulse is “short” if `|duration - te_short| ≤ te_delta`, “long” if `|duration - te_long| ≤ te_delta`. Default: 100.

Examples:

- Ford V0–style: `--te-short 250 --te-long 500 --te-delta 100`
- VAG/VW 500 µs: `--te-short 500 --te-long 1000 --te-delta 120`

### Usage

```bash
# PWM decode (short→0, long→1), default 250/500 µs
python3 scripts/demod_sub_to_bits.py path/to/file.sub

# PWM with VAG-like timing
python3 scripts/demod_sub_to_bits.py path/to/file.sub --te-short 500 --te-long 1000 --te-delta 120

# Manchester decode (e.g. Ford)
python3 scripts/demod_sub_to_bits.py path/to/file.sub --encoding manchester --te-short 250 --te-long 500

# Wrap output to 80 characters per line
python3 scripts/demod_sub_to_bits.py path/to/file.sub --encoding pwm --wrap 80

# Only decode HIGH pulses (or --pulse-level low)
python3 scripts/demod_sub_to_bits.py path/to/file.sub --encoding pwm --pulse-level high

# Raw level stream (no duration decoding)
python3 scripts/demod_sub_to_bits.py path/to/file.sub --encoding raw

# Print level:duration for each pulse (no bits)
python3 scripts/demod_sub_to_bits.py path/to/file.sub --with-durations
```

### Options summary

| Option | Default | Description |
|--------|---------|-------------|
| `--encoding` | pwm | `raw`, `pwm`, or `manchester` |
| `--te-short` | 250 | Nominal short pulse (µs) |
| `--te-long` | 500 | Nominal long pulse (µs) |
| `--te-delta` | 100 | Timing tolerance (µs) |
| `--pulse-level` | both | For PWM: `high`, `low`, or `both` |
| `--gap-us` | 10000 | Manchester: gap (µs) that resets state |
| `--wrap` | 0 | Wrap bit string every N characters (0 = no wrap) |
| `--with-durations` | off | Print `level:duration` instead of bits |

### Output

- Decoded bit string is printed to **stdout** (e.g. `1011001...` or wrapped lines).
- A single comment line (pulse count, frequency, encoding, timing) is printed to **stderr**.

### Requirements

- Python 3.9+ (for `list[tuple[...]]` type hints; can be relaxed if needed).
- No extra dependencies; uses only the standard library.

### Inspecting raw pulses (e.g. for VAG)

To see the first pulses of a `.sub` file (level and duration in µs) without decoding:

```bash
python3 scripts/demod_sub_to_bits.py path/to/file.sub --encoding raw --with-durations
```

VAG Type 3/4 expects: first pulse **HIGH** ~500 µs, then LOW ~500 µs, repeated (preamble); after ≥41 such pairs, HIGH ~1000 µs then LOW ~500 µs (sync), then 3×750 µs, then data. If the first pulse is LOW or durations are outside 500±79/80, the decoder will not lock.
