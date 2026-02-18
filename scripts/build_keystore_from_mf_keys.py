#!/usr/bin/env python3
"""Generate src/keystore/embedded.rs and KEY_ENTRY_NAMES for mod.rs from REFERENCES/mf_keys.txt"""
import re

MF_KEYS_PATH = "REFERENCES/mf_keys.txt"
# Star Line uses same key as SL_A2-A4; we add type 20 so KeyStore.star_line_mf_key is set
STAR_LINE_KEY_HEX = "9BF7F89BF8FE78DA"

def parse_mf_keys(path: str):
    entries = []  # (type_id, key_u64, name)
    vag_bytes = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # VAG block: "0000: xx xx ..." or "0020: xx xx ..."
            if re.match(r"^[0-9A-Fa-f]{4}:\s", line):
                hex_part = line.split(":", 1)[1].strip()
                for h in hex_part.split():
                    vag_bytes.append(int(h, 16))
                continue
            # KEY:TYPE:NAME
            parts = line.split(":", 2)
            if len(parts) != 3:
                continue
            key_hex, type_str, name = parts
            key_hex = key_hex.strip()
            type_id = int(type_str.strip())
            name = name.strip()
            key_u64 = int(key_hex, 16)
            entries.append((type_id, key_u64, name))
    # Append Star Line (type 20) so load_kia_keys sets star_line_mf_key
    entries.append((20, int(STAR_LINE_KEY_HEX, 16), "Star Line"))
    return entries, vag_bytes

def u64_to_le_bytes(x: int) -> bytes:
    return x.to_bytes(8, "little")

def build_blob(entries, vag_bytes):
    blob = bytearray()
    blob.extend(b"KATK")
    blob.extend((len(entries)).to_bytes(2, "little"))
    for type_id, key_u64, _ in entries:
        blob.extend(type_id.to_bytes(4, "little"))
        blob.extend(u64_to_le_bytes(key_u64))
    blob.extend(b"VAG ")
    assert len(vag_bytes) == 64, f"expected 64 VAG bytes, got {len(vag_bytes)}"
    blob.extend(bytes(vag_bytes))
    return bytes(blob)

def rust_byte_literal(blob: bytes, indent="    ") -> str:
    lines = []
    for i in range(0, len(blob), 12):
        chunk = blob[i : i + 12]
        hexes = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(indent + hexes + ",")
    return "\n".join(lines)

def main():
    entries, vag_bytes = parse_mf_keys(MF_KEYS_PATH)
    blob = build_blob(entries, vag_bytes)

    # embedded.rs
    embedded_rs = '''//! Embedded keystore blob (standard encrypted + VAG raw).
//! Data is stored as binary to avoid plain-text keys in config.
//! Format: "KATK" magic, n_entries (u16 LE), then per entry: type_id (u32 LE), key (u64 LE), then "VAG " + 64 bytes.
//! Key bytes are little-endian (LSB first); the resulting u64 matches reference hex (MSB-first notation).
//! Generated from REFERENCES/mf_keys.txt by scripts/build_keystore_from_mf_keys.py

/// Blob: KATK + ''' + str(len(entries)) + ''' entries + VAG 64 bytes.
#[rustfmt::skip]
pub const KEYSTORE_BLOB: &[u8] = &[
'''
    # Split blob for readable comments: magic + n, then entries (each 12 bytes), then VAG tag + 64
    off = 0
    embedded_rs += "    // magic + n_entries\n"
    embedded_rs += rust_byte_literal(blob[0:6], "    ") + "\n"
    off = 6
    entry_size = 12
    for i, (ty, key, name) in enumerate(entries):
        if i % 8 == 0:
            embedded_rs += f"    // entries {i}..\n"
        embedded_rs += rust_byte_literal(blob[off : off + entry_size], "    ") + "\n"
        off += entry_size
    embedded_rs += "    // VAG tag + 64 bytes\n"
    embedded_rs += rust_byte_literal(blob[off:], "    ") + "\n"
    embedded_rs += "];\n"

    with open("src/keystore/embedded.rs", "w") as f:
        f.write(embedded_rs)

    # KEY_ENTRY_NAMES for mod.rs (same order as blob entries)
    names = [e[2] for e in entries]

    # Update mod.rs: replace KEY_ENTRY_NAMES
    mod_rs_path = "src/keystore/mod.rs"
    with open(mod_rs_path) as f:
        mod_content = f.read()
    escaped = [n.replace("\\", "\\\\").replace('"', '\\"') for n in names]
    new_block = "const KEY_ENTRY_NAMES: &[&str] = &[\n    " + ",\n    ".join(f'"{s}"' for s in escaped) + ",\n];"
    pattern = r"const KEY_ENTRY_NAMES: &\[&str\] = &\[\n.*?\n\];"
    mod_content = re.sub(pattern, new_block, mod_content, flags=re.DOTALL)
    with open(mod_rs_path, "w") as f:
        f.write(mod_content)

    print(f"Wrote embedded blob with {len(entries)} entries, {len(vag_bytes)} VAG bytes")
    print(f"Updated KEY_ENTRY_NAMES with {len(names)} names")

if __name__ == "__main__":
    main()
