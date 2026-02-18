# Sub Decode: ProtoPirate vs KAT

Comparison of how **ProtoPirate** (REFERENCES/protopirate) and **KAT** decode Flipper `.sub` (RAW) files.

## File format

Both use the same Flipper SubGhz RAW format:

- **Filetype:** `Flipper SubGhz RAW File`
- **Protocol:** `RAW`
- **Frequency:** one value (Hz)
- **RAW_Data:** space-separated **int32** values:
  - **Positive** = HIGH level
  - **Negative** = LOW level  
  - **Magnitude** = duration in **microseconds**

So parsing and (level, duration) stream content are the same.

---

## ProtoPirate sub decode (REFERENCES/protopirate)

### Where it lives

- **Scene:** `scenes/protopirate_scene_sub_decode.c`
- **Raw reader:** `helpers/raw_file_reader.c` / `raw_file_reader.h`

### Flow

1. **Open file**  
   `raw_file_reader_open()` uses FlipperFormat to open the file, checks header "Flipper SubGhz RAW File" and Protocol "RAW". Does **not** read Frequency in the reader; the scene reads that separately.

2. **Read metadata (scene)**  
   In `DecodeStateReadHeader` / `DecodeStateStartingWorker` the scene opens the file again with FlipperFormat and reads:
   - **Frequency** (default 433920000 if missing)
   - **Preset** (e.g. AM650, FM238) and maps it to the SubGhz preset used for the receiver.

3. **Feed stream**  
   In `DecodeStateDecodingRaw`:
   - Loop: `raw_file_reader_get_next(ctx->raw_reader, &level, &duration)` to get the next (level, duration).
   - For each pair: **`subghz_receiver_decode(app->txrx->receiver, level, duration)`**.
   - Reads in chunks of **128** samples per tick (`SAMPLES_TO_READ_PER_TICK`) for UI responsiveness.

4. **On decode**  
   When the Flipper receiver reports a decode (`protopirate_sub_decode_receiver_callback`):
   - Add the decode to history.
   - **`subghz_receiver_reset(receiver)`** so all decoders are reset.
   - Continue feeding the **same** file from the next sample.

So: one continuous stream from the file, (level, duration) in order; on each decode → record → reset receiver → keep going. **No** polarity inversion in this path. **No** sliding window or multiple start positions.

### Raw file reader details

- **`raw_file_reader_get_next()`** (raw_file_reader.c):  
  Reads next int32 from buffer; if buffer is empty, loads next chunk via `flipper_format_read_int32(..., "RAW_Data", ...)`.  
  `level = (value >= 0)`, `duration = abs(value)`. Same convention as KAT.

---

## KAT sub import (src/app.rs + protocols/mod.rs + export/flipper.rs)

### Where it lives

- **Import:** `src/export/flipper.rs` → `import_sub_raw(path)` → returns `(frequency, Vec<StoredLevelDuration>)`.
- **Decode:** `src/protocols/mod.rs` → `process_signal_stream()` / `process_signal_stream_inner()`.
- **Use:** `src/app.rs` → when loading a .sub file, calls `import_sub_raw` then `protocols.process_signal_stream(&pairs, frequency)`.

### Flow

1. **Parse file**  
   `import_sub_raw()`:
   - Reads whole file as text.
   - Parses **Frequency** (default 433_920_000 if missing).
   - Parses all **RAW_Data** lines into one list of (level, duration) with the same rule: positive ⇒ HIGH, negative ⇒ LOW, duration = abs(value) µs.

2. **Decode stream**  
   `process_signal_stream(pairs, frequency)`:
   - Tries **normal polarity** first: `process_signal_stream_inner(pairs, frequency, false)`.
   - If that returns **no** decodes, tries **inverted polarity**: `process_signal_stream_inner(pairs, frequency, true)` (flip level for every pair).
   - Inner loop: for each (level, duration) in order, for each decoder that supports the file frequency, call **`decoder.feed(level, duration_us)`**. If any decoder returns a decode:
     - Record (protocol name, decoded signal, segment of pairs).
     - **Reset all decoders** and set `segment_start` to next index.
     - Continue from the next pair.

So: one pass over the in-memory stream; on each decode → record → reset all decoders → continue. **Difference:** KAT also runs a **second** pass with **inverted polarity** if the first pass finds nothing.

---

## Differences summary

| Aspect | ProtoPirate | KAT |
|--------|-------------|-----|
| **File format** | Same (Flipper RAW, positive=HIGH, negative=LOW, µs) | Same |
| **Stream order** | Same (sequential level/duration from file) | Same |
| **Frequency** | Read from file, used for preset/receiver | Read from file, used for decoder filter (2% tolerance) |
| **Decode loop** | One (level, duration) at a time → `subghz_receiver_decode()` | One (level, duration) at a time → `decoder.feed()` for each decoder |
| **On decode** | Add to history, **subghz_receiver_reset()**, continue same file | Append to list, **reset all decoders**, continue same stream |
| **Polarity** | Single polarity (as in file) | Tries **normal**, then **inverted** if no decodes |
| **Sliding window / multiple starts** | No | No |
| **“No protocol” result** | Shows “No ProtoPirate protocol detected” (no “Unknown” capture) | No capture if no decoder ever returns (same idea) |

So the **sub decode strategy is the same**: single stream, reset-after-each-decode, no sliding window. The only functional difference is **KAT’s extra inverted-polarity pass** when the normal pass finds no decodes.

For a file like the VAG Suran .sub: the first HIGH pulse is 133 µs, so VAG’s Reset condition (300±79 or 500±79 µs) is never met. In both codebases the decoder would stay in Reset and never emit a decode; neither would create a capture from that stream. ProtoPirate would show “No ProtoPirate protocol detected”; KAT would add no capture. So for that case the behavior is aligned; fixing it would require something like trying decode from multiple start indices (sliding window) or trimming to the first VAG-like preamble, in either codebase.
