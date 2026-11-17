#!/usr/bin/env python3
"""Writes the data kat.karazajac.io's pages are built from, into docs/data/.

  protocols.json  KAT's own facts about every protocol (name, make, coding, RF, timing, bits,
                  encoder). A cargo test writes it from the program: src/protocols/site_data.rs.
  captures.json   The recordings in IMPORTS: what each file is (type, frequency, preset, length)
                  and what KAT reads from it. From the files' headers and the two pinned lists
                  testdata/imports_decodes.txt and testdata/imports_more_decodes.txt.

Run it from the repository root on the tree that is published, after the pinned lists were
regenerated:

    python3 scripts/site_data.py              # both files
    python3 scripts/site_data.py --captures   # captures.json only (no cargo)

docs/build.mjs reads only docs/, so the site builds without the rest of the repository.
"""
import collections, json, os, re, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'docs', 'data')
IMPORTS = os.path.join(ROOT, 'IMPORTS')

PRESETS = {
    'FuriHalSubGhzPresetOok650Async': 'AM650',
    'FuriHalSubGhzPresetOok270Async': 'AM270',
    'FuriHalSubGhzPreset2FSKDev238Async': 'FM238',
    'FuriHalSubGhzPreset2FSKDev476Async': 'FM476',
    'FuriHalSubGhzPresetCustom': 'Custom',
}
XTAL = 26_000_000  # the CC1101's crystal on a Flipper Zero


def custom_preset(data):
    """What a `Custom_preset_data` register list sets: keying, deviation, data rate."""
    try:
        b = [int(x, 16) for x in data.split()]
    except ValueError:
        return None
    reg = {}
    for i in range(0, len(b) - 1, 2):
        if b[i] == 0 and b[i + 1] == 0:
            break
        reg[b[i]] = b[i + 1]
    out = {}
    if 0x12 in reg:
        out['keying'] = {0: '2-FSK', 1: 'GFSK', 3: 'OOK', 4: '4-FSK', 7: 'MSK'}.get((reg[0x12] >> 4) & 7, '?')
    if 0x15 in reg and out.get('keying') not in ('OOK', None):
        e, m = (reg[0x15] >> 4) & 7, reg[0x15] & 7
        out['deviation_hz'] = round(XTAL / 2 ** 17 * (8 + m) * 2 ** e)
    if 0x10 in reg and 0x11 in reg:
        out['baud'] = round((256 + reg[0x11]) * 2 ** (reg[0x10] & 15) / 2 ** 28 * XTAL)
        # channel filter bandwidth
        e, m = (reg[0x10] >> 6) & 3, (reg[0x10] >> 4) & 3
        out['bandwidth_hz'] = round(XTAL / (8 * (4 + m) * 2 ** e))
    return out or None


def header(path):
    """The fields of a .sub file's head, and the length of its RAW data."""
    h, us, pulses = {}, 0, 0
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            k, sep, v = line.partition(':')
            if not sep:
                continue
            k, v = k.strip(), v.strip()
            if k == 'RAW_Data':
                for tok in v.split():
                    try:
                        us += abs(int(tok)); pulses += 1
                    except ValueError:
                        pass
            elif k not in h and len(h) < 40:
                h[k] = v
    return h, us, pulses


def pinned():
    """What KAT reads from each file: {path: {protocol: {...}}} from the two pinned lists."""
    reads = collections.defaultdict(dict)
    seen = set()
    notes = {}

    def entry(path, proto):
        return reads[path].setdefault(proto, {'protocol': proto, 'frames': 0, 'codes': set(), 'serials': collections.Counter(),
                                              'buttons': collections.Counter(), 'crc_ok': 0, 'encodable': 0})

    for name in ('imports_decodes.txt', 'imports_more_decodes.txt'):
        for line in open(os.path.join(ROOT, 'testdata', name), encoding='utf-8'):
            f = [x.strip() for x in line.rstrip('\n').split(' | ')]
            if len(f) < 2:
                continue
            path = f[0]; seen.add(path)
            rest = f[2:] if re.fullmatch(r'\d*', f[1]) or f[1] == '-' else f[1:]
            if rest and rest[0] == 'key':
                rest = rest[1:]
            if not rest or rest[0] == '(none)':
                continue
            if rest[0].startswith(('ERR', 'refused:')):                  # not a decode: why there is none
                notes[path] = rest[0][4:] if rest[0].startswith('ERR ') else rest[0][len('refused: '):]
                continue
            e = entry(path, rest[0])
            if len(rest) >= 3 and re.fullmatch(r'x\d+', rest[1]):       # "x50 | 20 distinct | fnv=…"
                e['frames'] += int(rest[1][1:])
                m = re.match(r'(\d+) distinct', rest[2])
                e['distinct'] = int(m.group(1)) if m else None
            else:                                                        # one line per frame
                e['frames'] += 1
                d = rest[1] if len(rest) > 1 else ''
                m = re.search(r'data=(\S+)', d)
                if m: e['codes'].add((m.group(1), (re.search(r'extra=(\S+)', d) or [None, ''])[1]))
                m = re.search(r'\bsn=(\S+)', d)
                if m and m.group(1) != '-': e['serials'][m.group(1)] += 1
                m = re.search(r'\bbtn=(\S+)', d)
                if m and m.group(1) != '-': e['buttons'][m.group(1)] += 1
                if re.search(r'\bcrc=1\b', d): e['crc_ok'] += 1
                if re.search(r'\btx=1\b', d): e['encodable'] += 1
    out = {}
    for path, by in reads.items():
        rows = []
        for e in by.values():
            row = {'protocol': e['protocol'], 'frames': e['frames']}
            if e.get('distinct') is not None:
                row['distinct'] = e['distinct']
            elif e['codes']:
                row['distinct'] = len(e['codes'])
            if e['serials']:
                row['serials'] = len(e['serials'])
                row['serial'] = e['serials'].most_common(1)[0][0]
            if e['buttons']:
                row['buttons'] = sorted(e['buttons'], key=lambda b: (len(b), b))[:8]
            if e['crc_ok']:
                row['crc_ok'] = e['crc_ok']
            if e['encodable']:
                row['encodable'] = True
            rows.append(row)
        out[path] = sorted(rows, key=lambda r: -r['frames'])
    return out, seen, notes


def captures(key_names=None):
    git = subprocess.run(['git', '-C', ROOT, 'ls-files', '-z', 'IMPORTS'], capture_output=True)
    if git.returncode == 0 and git.stdout:
        files = git.stdout.decode().split('\0')        # only what is in the repository
    else:                                              # an export without .git: every file is
        files = [os.path.relpath(os.path.join(d, f), ROOT).replace(os.sep, '/') for d, _, fs in os.walk(IMPORTS) for f in fs]
    reads, pinned_paths, notes = pinned()
    rows = []
    for rel in sorted(f for f in files if f.lower().endswith('.sub')):
        path = rel[len('IMPORTS/'):]
        h, us, pulses = header(os.path.join(ROOT, rel))
        kind = 'raw' if 'RAW' in h.get('Filetype', '') or h.get('Protocol') == 'RAW' else ('key' if 'Key' in h.get('Filetype', '') else 'other')
        if not h.get('Filetype') and pulses:
            kind = 'raw'
        row = {'path': path, 'folder': path.split('/')[0], 'more': '/more/' in '/' + path, 'bytes': os.path.getsize(os.path.join(ROOT, rel)), 'type': kind}
        try:
            row['frequency'] = int(h['Frequency'])
        except (KeyError, ValueError):
            pass
        p = h.get('Preset')
        if p:
            row['preset'] = PRESETS.get(p, p)
            if row['preset'] == 'Custom':
                c = custom_preset(h.get('Custom_preset_data', ''))
                if c: row['custom'] = c
        if kind == 'key' and h.get('Protocol'):
            row['key_protocol'] = h['Protocol']
            if h.get('Bit'): row['key_bits'] = h['Bit']
        if pulses:
            row['pulses'] = pulses
            row['seconds'] = round(us / 1e6, 2)
        row['reads'] = reads.get(path, [])
        if path in notes:
            row['note'] = notes[path]
        # The first pinned list is of RAW files; a key file is read by the decoder its name loads with.
        if kind == 'key' and not row['reads'] and row.get('note', '').startswith('No RAW_Data'):
            del row['note']
            if key_names and row.get('key_protocol') in key_names:
                row['reads'] = [{'protocol': key_names[row['key_protocol']], 'frames': 1, 'key': True}]
        row['pinned'] = path in pinned_paths
        rows.append(row)
    missing = [r['path'] for r in rows if not r['pinned']]
    if missing:
        print('not in the pinned lists (%d):' % len(missing), *missing[:8], sep='\n  ')
    doc = {'repo': 'https://github.com/KaraZajac/KAT', 'branch': 'main', 'folder': 'IMPORTS', 'captures': rows}
    os.makedirs(DATA, exist_ok=True)
    with open(os.path.join(DATA, 'captures.json'), 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, indent=1, sort_keys=True); f.write('\n')
    read = sum(1 for r in rows if r['reads'])
    print('captures.json: %d files, %d read, %d protocols seen, %.1f MB' % (
        len(rows), read, len({x['protocol'] for r in rows for x in r['reads']}), sum(r['bytes'] for r in rows) / 1e6))


def modules():
    """The module of each entry of `info::ALL`, in its order, and the docs page of each module."""
    info = open(os.path.join(ROOT, 'src', 'protocols', 'info.rs'), encoding='utf-8').read()
    a = info.index('pub static ALL')
    order = [m.replace('::', '/') for m in re.findall(r'&super::([\w:]+)::\w+,', info[a:info.index('];', a)])]
    pages = {}
    docs = os.path.join(ROOT, 'docs')
    for f in sorted(os.listdir(docs)):
        if not f.endswith('.md'):
            continue
        head = open(os.path.join(docs, f), encoding='utf-8').read(3000)
        m = re.search(r'\*\*Rust module[^\n]*', head)
        for mod in re.findall(r'src/protocols/([\w/]+)\.rs', m.group(0) if m else ''):
            pages.setdefault(mod, f[:-3])          # the first page that names a module is its page
    return order, pages


def protocols():
    out = os.path.join(DATA, 'protocols.json')
    os.makedirs(DATA, exist_ok=True)
    env = dict(os.environ, KAT_SITE_DATA=out)
    subprocess.run(['cargo', 'test', '--locked', '--bin', 'kat', 'site_data', '--', '--ignored', '--exact',
                    'protocols::site_data::site_data'], cwd=ROOT, env=env, check=True)
    d = json.load(open(out, encoding='utf-8'))
    order, pages = modules()
    assert len(order) == len(d['protocols']), (len(order), len(d['protocols']))
    docs = os.path.join(ROOT, 'docs')
    nopage = []
    for p, mod in zip(d['protocols'], order):
        p['module'] = mod
        base = mod.split('/')[-1]
        p['page'] = pages.get(mod) or pages.get(base) or (base if os.path.isfile(os.path.join(docs, base + '.md')) else None)
        if not p['page']:
            nopage.append(p['name'])
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=1, sort_keys=True); f.write('\n')
    print('protocols.json: KAT %s, %d entries, %d registered decoders; without a page: %s' % (
        d['version'], len(d['protocols']), d['decoders'], nopage or 'none'))
    return d


if __name__ == '__main__':
    if '--captures' in sys.argv[1:]:
        names = json.load(open(os.path.join(DATA, 'protocols.json'), encoding='utf-8')).get('key_names')
    else:
        names = protocols().get('key_names')
    captures(names)
