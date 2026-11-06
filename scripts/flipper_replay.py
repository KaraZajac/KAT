#!/usr/bin/env python3
"""Replay test signals from a Flipper Zero one file at a time, to watch them arrive in KAT.

The Flipper transmits over its USB CLI (`subghz tx_from_file`): each `.sub` file is uploaded,
announced, sent once, and followed by a pause (or a wait for Enter with --step), so the capture
that pops up in KAT can be matched to the file that produced it.

    scripts/flipper_replay.py                    every test file KAT hears when tuned to 433.92 MHz
    scripts/flipper_replay.py --freq 315         the files KAT hears at 315 MHz
    scripts/flipper_replay.py --step             wait for Enter after each file
    scripts/flipper_replay.py --only princeton   only files whose path contains "princeton"
    scripts/flipper_replay.py --start 42         resume at file 42 (the numbering of --list)
    scripts/flipper_replay.py --list             print the sequence and exit
    scripts/flipper_replay.py IMPORTS/KIA        a folder or file of your own instead of the defaults

Defaults: the files under IMPORTS/ and the Flipper unit-test corpus under REFERENCES/, limited to
the band KAT's receiver covers at --freq (1.15 MHz below to 0.65 MHz above it at 2 MS/s): RAW
recordings, plus key files of the protocols the Flipper firmware has an encoder for. Key files of
ProtoPirate-only protocols (`Fiat type 0`, `KIA/HYU V5`, `VAG`...) crash Momentum's tx_from_file
and are skipped unless --keys all is given.

Standard library only. The port is found under /dev/serial/by-id (or set FZ_PORT / --port); the
user must be allowed to open it (group `dialout` on most distributions).
"""
import argparse
import glob
import os
import re
import select
import sys
import termios
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
UNIT_TESTS = os.path.join(REPO, 'REFERENCES', 'Flipper-ARF', 'applications', 'debug', 'unit_tests',
                          'resources', 'unit_tests', 'subghz')
GOLDEN = os.path.join(REPO, 'testdata', 'imports_decodes.txt')
PROMPT = b'\r\n>: '
# Protocols every Flipper firmware encodes (the unit-test corpus's key files); refreshed from the
# corpus when it is present.
NATIVE_KEY_PROTOCOLS = {
    'Ansonic', 'BETT', 'CAME', 'CAME TWEE', 'Clemsa', 'Dickert_MAHS', 'Doitrand', 'Dooya', 'Feron',
    'GangQi', 'GateTX', 'Hollarm', 'Holtek', 'Holtek_HT12X', 'Honeywell', 'Intertechno_V3', 'KeeLoq',
    'Legrand', 'Linear', 'LinearDelta3', 'Magellan', 'Marantec', 'Marantec24', 'Mastercode',
    'MegaCode', 'Nice FLO', 'Phoenix_V2', 'Power Smart', 'Princeton', 'Revers_RB2', 'Roger',
    'SMC5326', 'Security+ 1.0', 'Security+ 2.0',
}


# ---- Flipper CLI over the USB serial port ------------------------------------------------------

def find_port(explicit):
    if explicit:
        return explicit
    env = os.environ.get('FZ_PORT')
    if env:
        return env
    ports = sorted(glob.glob('/dev/serial/by-id/usb-Flipper_Devices_Inc._*'))
    if not ports:
        sys.exit('no Flipper on /dev/serial/by-id (is it plugged in and the CLI free?); or set FZ_PORT')
    return ports[0]


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0                                             # raw input
    a[1] = 0                                             # raw output
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1, no modem lines
    a[3] = 0                                             # no echo, non-canonical
    a[4] = a[5] = termios.B115200
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def read_until(fd, token, timeout):
    buf, end = b'', time.time() + timeout
    while time.time() < end:
        ready, _, _ = select.select([fd], [], [], 0.05)
        if ready:
            try:
                buf += os.read(fd, 65536)
            except BlockingIOError:
                continue
            if token in buf:
                return buf, True
    return buf, False


def sync(fd):
    """Reach a clean prompt (past the banner, or whatever an earlier session left behind)."""
    os.write(fd, b'\r')
    read_until(fd, PROMPT, 3.0)
    time.sleep(0.1)
    termios.tcflush(fd, termios.TCIFLUSH)


def run(fd, line, timeout=10.0):
    """One CLI command; returns (its output, whether the prompt came back in time)."""
    os.write(fd, line.encode() + b'\r')
    out, ok = read_until(fd, PROMPT, timeout)
    text = out.decode(errors='replace')
    if text.startswith(line):
        text = text[len(line):]
    return text.replace('\r\n>: ', '').strip('\r\n'), ok


def upload(fd, local, remote):
    data = open(local, 'rb').read()
    run(fd, f'storage remove {remote}', 5.0)
    for i in range(0, len(data), 512):
        chunk = data[i:i + 512]
        os.write(fd, f'storage write_chunk {remote} {len(chunk)}\r'.encode())
        out, ok = read_until(fd, b'Ready\r\n', 5.0)
        if not ok:
            raise RuntimeError(f'upload: no Ready at byte {i}: {out[-80:]!r}')
        os.write(fd, chunk)
        out, ok = read_until(fd, PROMPT, 10.0)
        if not ok or b'rror' in out:
            raise RuntimeError(f'upload failed at byte {i}: {out[-80:]!r}')


def remove_tree(fd, path):
    """storage remove refuses a folder with files in it: empty it first."""
    text, _ = run(fd, f'storage list {path}', 15.0)
    for line in text.splitlines():
        m = re.match(r'\s*\[(F|D)\]\s+(\S.*?)(?:\s+\d+b)?\s*$', line)
        if not m:
            continue
        if m.group(1) == 'D':
            remove_tree(fd, f'{path}/{m.group(2).strip()}')
        else:
            run(fd, f'storage remove {path}/{m.group(2).strip()}', 15.0)
    run(fd, f'storage remove {path}', 15.0)


# ---- the files ----------------------------------------------------------------------------------

def header(path):
    head = open(path, errors='replace').read(600)
    freq = re.search(r'Frequency:\s*(\d+)', head)
    preset = re.search(r'Preset:\s*(\S+)', head)
    proto = re.search(r'Protocol:\s*(.+)', head)
    return (int(freq.group(1)) if freq else 0,
            preset.group(1).replace('FuriHalSubGhzPreset', '') if preset else '?',
            proto.group(1).strip() if proto else '?')


def collect(sources):
    out = []
    for src in sources:
        if os.path.isfile(src):
            out.append(src)
            continue
        for dirpath, _, names in sorted(os.walk(src)):
            out.extend(os.path.join(dirpath, n) for n in sorted(names) if n.lower().endswith('.sub'))
    return out


def golden():
    """IMPORTS file (relative to IMPORTS/) -> the protocols KAT decodes from it (testdata golden)."""
    table = {}
    if os.path.exists(GOLDEN):
        for line in open(GOLDEN, errors='replace'):
            parts = [p.strip() for p in line.split('|')]
            if len(parts) >= 3 and parts[2] not in ('', '(none)'):
                table.setdefault(parts[0], set()).add(parts[2])
    return table


def expectation(path, proto, table):
    if proto != 'RAW':
        return proto
    rel = os.path.relpath(path, os.path.join(REPO, 'IMPORTS'))
    if rel in table:
        return ', '.join(sorted(table[rel]))
    if os.path.relpath(path, REPO).startswith('IMPORTS'):
        return 'nothing (no decoder)'
    name = os.path.basename(path)
    return re.sub(r'(_raw)?\.sub$', '', name)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split('\n', 1)[1])
    ap.add_argument('sources', nargs='*', help='.sub files or folders (default: IMPORTS and the Flipper unit-test corpus)')
    ap.add_argument('--freq', type=float, default=433.92, help='MHz KAT is tuned to (default 433.92): files outside its band are skipped')
    ap.add_argument('--band', help='explicit band in MHz, e.g. 432.77-434.57 (overrides --freq)')
    ap.add_argument('--all', action='store_true', help='no band filter')
    ap.add_argument('--keys', choices=['native', 'all', 'none'], default='native', help='which key files to send (default: native protocols only)')
    ap.add_argument('--only', help='only files whose path contains this text (case-insensitive)')
    ap.add_argument('--start', type=int, default=1, help='first file number to send (the numbering of --list)')
    ap.add_argument('--list', action='store_true', help='print the sequence and exit')
    ap.add_argument('--step', action='store_true', help='wait for Enter after each file instead of pausing')
    ap.add_argument('--pause', type=float, default=3.0, help='seconds between files (default 3)')
    ap.add_argument('--repeat', type=int, default=1, help='tx_from_file repeat count (default 1; RAW files only)')
    ap.add_argument('--dir', default='/ext/subghz/kat_replay', help='folder on the Flipper for the uploads')
    ap.add_argument('--no-upload', action='store_true', help='the files are already on the Flipper from an earlier run (same numbering)')
    ap.add_argument('--clean', action='store_true', help='delete the uploaded files from the Flipper at the end')
    ap.add_argument('--port', help='serial device (default: the Flipper under /dev/serial/by-id, or $FZ_PORT)')
    args = ap.parse_args()

    sources = args.sources or [os.path.join(REPO, 'IMPORTS'), UNIT_TESTS]
    if args.band:
        lo, hi = (float(x) for x in args.band.split('-'))
    else:
        lo, hi = args.freq - 1.15, args.freq + 0.65
    native = set(NATIVE_KEY_PROTOCOLS)
    if os.path.isdir(UNIT_TESTS):
        for name in os.listdir(UNIT_TESTS):
            if name.endswith('.sub'):
                proto = header(os.path.join(UNIT_TESTS, name))[2]
                if proto != 'RAW':
                    native.add(proto)
    table = golden()

    files = []
    for path in collect(sources):
        freq, preset, proto = header(path)
        if not args.all and not (lo * 1e6 <= freq <= hi * 1e6):
            continue
        if proto != 'RAW' and (args.keys == 'none' or (args.keys == 'native' and proto not in native)):
            continue
        if args.only and args.only.lower() not in path.lower():
            continue
        files.append((path, freq, preset, proto))
    if not files:
        sys.exit('no files match')
    width = len(str(len(files)))

    def describe(n, path, freq, preset, proto):
        rel = os.path.relpath(path, REPO) if path.startswith(REPO) else path
        return f'[{n:{width}d}/{len(files)}] {rel}  {freq / 1e6:.3f} MHz  {preset}  -> {expectation(path, proto, table)}'

    if args.list:
        for n, (path, freq, preset, proto) in enumerate(files, 1):
            print(describe(n, path, freq, preset, proto))
        return

    fd = open_port(find_port(args.port))
    n = args.start
    try:
        sync(fd)
        run(fd, f'storage mkdir {args.dir}', 10.0)
        for n, (path, freq, preset, proto) in enumerate(files, 1):
            if n < args.start:
                continue
            remote = f'{args.dir}/{n:0{width}d}_{re.sub(r"[^A-Za-z0-9._-]+", "_", os.path.basename(path))[:60]}'
            print(time.strftime('%H:%M:%S '), describe(n, path, freq, preset, proto), flush=True)
            if not args.no_upload:
                upload(fd, path, remote)
            text, ok = run(fd, f'subghz tx_from_file {remote} {args.repeat} 0', 180.0)
            if not ok:
                os.write(fd, b'\x03')
                time.sleep(0.5)
                print('          timed out (sent Ctrl-C to the Flipper)')
            elif re.search(r'rror|restricted|not supported|not found', text, re.I):
                print('          Flipper said:', ' '.join(text.split())[:140])
            else:
                print('          sent')
            if args.step:
                input('          Enter for the next file... ')
            else:
                time.sleep(args.pause)
        if args.clean:
            remove_tree(fd, args.dir)
            print('uploads removed from the Flipper')
    except KeyboardInterrupt:
        try:
            os.write(fd, b'\x03')
        except OSError:
            pass
        print(f'\nstopped at file {n}; resume with --start {n}')
        sys.exit(130)
    except OSError as e:
        print(f'\nlost the Flipper at file {n} ({e}); it may have rebooted — resume with --start {n}')
        sys.exit(2)
    finally:
        try:
            os.close(fd)
        except OSError:
            pass


if __name__ == '__main__':
    main()
