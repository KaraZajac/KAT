#!/usr/bin/env python3
"""Writes docs/data/frames.json: example frames per protocol, and where their fields lie.

KAT shows a decode as a data word (up to 64 bits, `data`, and what is left of a longer frame in `extra`) with
the serial, button and counter it read from it. This script takes every decode of the recordings in IMPORTS
(the output of `kat --decode IMPORTS --all`, one line per frame) and looks, per protocol, for the bits of the
data word that hold the serial, the button and the counter: a field is listed only when the same bits give
KAT's value in every frame of the protocol in the library, and those values are not all one value that could
sit there by chance. A field that is not found is one the decoder works out (a decrypted counter, a serial
spread over the frame).

    kat --decode IMPORTS --all > decodes.txt        (the release binary of the tree that is published)
    python3 scripts/site_frames.py decodes.txt      (reads docs/data/protocols.json, writes docs/data/frames.json)

docs/build.mjs draws the examples on each protocol's page.
"""
import collections, json, os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'docs', 'data')
EXAMPLES = 4
LINE = re.compile(r'data=([0-9A-F]+)/(\d+)(?: extra=([0-9A-F]+))?(?: sn=([0-9A-F]+|-))?(?: btn=([0-9A-F]+|-))?(?: cnt=([0-9A-F]+|-))?(?: crc=(\d))?')


def rows(path):
    """Decodes by display name: (file, data, bits, extra, serial, button, counter)."""
    out = collections.defaultdict(list)
    for line in open(path, encoding='utf-8', errors='replace'):
        parts = [p.strip() for p in line.rstrip('\n').split(' | ')]
        if len(parts) < 4 or not parts[-1].startswith('data='):
            continue
        name = next((p for p in parts[1:-1] if p not in ('raw', 'key', '-') and not p.isdigit() and not re.fullmatch(r'x\d+', p)), None)
        m = LINE.match(parts[-1])
        if not name or not m:
            continue
        val = lambda s: None if s in (None, '-') else int(s, 16)
        file = parts[0]
        file = file[file.index('IMPORTS/') + 8:] if 'IMPORTS/' in file else file
        out[name].append((file, int(m.group(1), 16), int(m.group(2)), val(m.group(3)), val(m.group(4)), val(m.group(5)), val(m.group(6))))
    return out


def rev(v, w):
    return int(format(v, '0%db' % w)[::-1], 2)


def find(frames, pick, word, kind):
    """Where the values `pick(frame)` lie in `word(frame)` (64 bits): (offset from the low end, width, how), or None.
    The widest window that fits every frame; None when the values could sit in several places."""
    vals = [(word(f), pick(f)) for f in frames if pick(f) is not None and word(f) is not None]
    if len(vals) < 2:
        return None
    distinct = len({v for _, v in vals})
    need = max(v.bit_length() for _, v in vals) or 1
    if distinct < 2 and need < 12:
        return None          # one small value: it could be anywhere
    widths = sorted({need, *(w for w in (4, 8, 12, 16, 20, 24, 28, 32) if w >= need and w - need < 4)}, reverse=True)
    for how, f in (('', lambda x, w: x), ('reversed', rev), ('inverted', lambda x, w: x ^ ((1 << w) - 1))):
        for w in widths:
            if how and w != need and kind != 'serial':
                continue
            hits = [off for off in range(0, 64 - w + 1) if all(f((d >> off) & ((1 << w) - 1), w) == v for d, v in vals)]
            if len(hits) == 1:
                return {'offset': hits[0], 'width': w, 'how': how, 'frames': len(vals), 'values': distinct}
            if len(hits) > 1 and w == need:
                break            # ambiguous at the exact width: give up on this reading
    return None


def main():
    src = sys.argv[1]
    proto = json.load(open(os.path.join(DATA, 'protocols.json')))
    names = {p['name']: p for p in proto['protocols']}
    display = proto['display_names']
    by_entry = collections.defaultdict(list)
    for name, frames in rows(src).items():
        base = re.sub(r' \[[^\]]*\]$', '', name)
        entry = display.get(name) or display.get(base) or (name if name in names else base if base in names else None)
        if entry is None:
            # a display name of a family ("KeeLoq (DoorHan)"): the prefix of an entry
            entry = next((p['name'] for p in proto['protocols'] if p.get('prefix') and name.lower().startswith(p['prefix'].lower())), None)
        if entry:
            by_entry[entry] += [(name,) + f for f in frames]
    out = {}
    for entry, frames in sorted(by_entry.items()):
        fr = [f[1:] for f in frames]
        fields = {}
        for kind, pick in (('serial', lambda f: f[4]), ('button', lambda f: f[5]), ('counter', lambda f: f[6])):
            got = find(fr, pick, lambda f: f[1], kind)
            where = 'data'
            if got is None:
                got = find(fr, pick, lambda f: f[3], kind)
                where = 'extra'
            if got:
                fields[kind] = dict(got, word=where)
        # fields that overlap each other are not both right: keep the one with more distinct values
        ks = sorted(fields, key=lambda k: -fields[k]['values'])
        keep = []
        for k in ks:
            a = fields[k]
            if all(a['word'] != fields[o]['word'] or a['offset'] + a['width'] <= fields[o]['offset'] or fields[o]['offset'] + fields[o]['width'] <= a['offset'] for o in keep):
                keep.append(k)
        fields = {k: fields[k] for k in keep}
        # examples: one per button first, from different files, most common bit count
        bits = collections.Counter(f[2] for f in fr).most_common(1)[0][0]
        seen, ex = set(), []
        for pass_ in (0, 1):
            for disp, file, data, nbits, extra, sn, btn, cnt in frames:
                key = (btn,) if pass_ == 0 else (file,)
                if nbits != bits or key in seen or any(e['data'] == '%X' % data for e in ex) or len(ex) >= EXAMPLES:
                    continue
                seen.add(key); seen.add((btn,))
                ex.append({'file': file, 'name': disp, 'data': '%X' % data, 'bits': nbits, 'extra': None if extra is None else '%X' % extra,
                           'serial': None if sn is None else '%X' % sn, 'button': btn, 'counter': None if cnt is None else '%X' % cnt})
        out[entry] = {'frames': len(fr), 'files': len({f[0] for f in fr}), 'bits': bits, 'fields': fields, 'examples': ex}
    text = json.dumps({'source': 'scripts/site_frames.py from the decodes of the recordings in IMPORTS', 'protocols': out}, indent=1, sort_keys=True)
    open(os.path.join(DATA, 'frames.json'), 'w').write(text + '\n')
    print('%d protocols; fields found: %s' % (len(out), dict(collections.Counter(k for v in out.values() for k in v['fields']))))


if __name__ == '__main__':
    main()
