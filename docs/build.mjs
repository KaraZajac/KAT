// build.mjs: builds kat.karazajac.io from this folder.
//
// index.html is written by hand. Every Markdown file here becomes a page at
// the address Jekyll gave it (radio.md becomes radio.html), so links into the
// docs keep working; heading anchors follow the rule GitHub Pages used
// (kramdown's GFM ids). Around the Markdown, this script builds what makes the
// site a reference work rather than a folder of notes:
//
//   - a box of facts beside every protocol's page, the recordings KAT reads
//     with it, the FCC IDs tied to it and the pages next to it;
//   - the tables: protocols.html, makes.html and a page per make,
//     captures.html, fcc.html, coverage.html, guides.html, changelog.html;
//   - the hardware: remotes.html and a page per remote (pictures of the
//     remote and its board, what its FCC filing says, the chips on it),
//     chips.html and a page per chip;
//   - example frames on every protocol's page, with the digits that hold
//     the serial, the button and the counter marked;
//   - search.json, which site.js searches.
//
// The facts come from data/: protocols.json is written by the program itself
// (src/protocols/site_data.rs), captures.json from the recordings and the
// pinned decode lists (scripts/site_data.py), fcc.json and coverage.json from
// the FCC filings that were read, taxonomy.json by hand; remotes_fcc.json (the
// filings' pictures and notes), remotes.json and chips.json (by hand: remotes
// whose owners sent pictures, and the chips), frames.json (example frames,
// scripts/site_frames.py) and frames_hand.json (field maps by hand). Nothing
// outside this folder is needed but ../CHANGELOG.md, and the site builds
// without it.
//
// The web server runs this on every deploy (npm ci, then npm run build), so it
// stays small: one pinned dependency and no network access.
//
//   node build.mjs                      writes the site to dist/
//   node build.mjs --redirect <origin>  writes, for GitHub Pages, a page at
//                                       every old address that forwards the
//                                       reader to the same page on <origin>
import { readFileSync, writeFileSync, readdirSync, mkdirSync, rmSync, cpSync, existsSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { Marked } from 'marked';

const HERE = dirname(fileURLToPath(import.meta.url));
const OUT = join(HERE, 'dist');
const ORIGIN = 'https://kat.karazajac.io';
const REPO = 'https://github.com/KaraZajac/KAT';

// What in this folder is not part of the published site.
const SKIP = new Set(['build.mjs', 'package.json', 'package-lock.json', 'node_modules', 'dist',
  '_config.yml', 'og-card.html', 'README.md']);
// Pages that explain how to use KAT, in the order the guide list shows them.
const GUIDES = ['recording', 'radio', 'flipper_radio', 'transmit', 'session', 'analyze', 'timing_analysis', 'compare',
  'add_manually', 'flipper_key_files', 'hitag2_hell', 'unidentified_captures'];
// Pages about the reference work itself.
const REFERENCE = ['glossary', 'about'];

const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
const plural = (n, one, many = `${one}s`) => `${n.toLocaleString('en-US')} ${n === 1 ? one : many}`;

// ---------------------------------------------------------------------------
// Data

const data = (f) => JSON.parse(readFileSync(join(HERE, 'data', f), 'utf8'));
const PROTO = data('protocols.json');
const CAPS = data('captures.json');
const FCC = data('fcc.json');
const COV = data('coverage.json');
const TAX = data('taxonomy.json');
const RFCC = data('remotes_fcc.json');
const ROWN = data('remotes.json');
const CHIPS = data('chips.json');
const FRAMES = data('frames.json');
const FHAND = data('frames_hand.json');

const CODING = { Pwm: 'PWM', Manchester: 'Manchester', DifferentialManchester: 'Differential Manchester', Nrz: 'NRZ', Miller: 'Miller' };
const RF = { AM: 'AM (on-off keying)', FM: 'FM (2-FSK)', Both: 'AM and FM' };
const RF_SHORT = { AM: 'AM', FM: 'FM', Both: 'AM, FM' };
const KIND = { Vehicle: 'Car key fob or alarm remote', Gate: 'Gate, garage or home remote' };

/** An address for a file in the repository, each part of the path escaped. */
const repoPath = (path) => path.split('/').map(encodeURIComponent).join('/');
const blob = (path) => `${REPO}/blob/${CAPS.branch}/${repoPath(path)}`;
const raw = (path) => `${REPO}/raw/${CAPS.branch}/${repoPath(path)}`;
const anchor = (s) => s.toLowerCase().normalize('NFD').replace(/[̀-ͯ]/g, '').replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');

// The entries of a page, and the page of a name.
const entries = PROTO.protocols.filter((p) => p.page);
const byPage = new Map();
for (const p of entries) (byPage.get(p.page) ?? byPage.set(p.page, []).get(p.page)).push(p);
const byName = new Map(entries.map((p) => [p.name, p]));
/** The entry a display name in a recording's decode belongs to ("KeeLoq (DoorHan)" is KeeLoq's). */
const resolve = (display) => byName.get(PROTO.display_names[display] ?? display) ?? byName.get(display);

// Recordings.
for (const c of CAPS.captures) {
  c.id = `rec-${anchor(c.path)}`;
  c.name = c.path.split('/').pop();
  c.repo = `${CAPS.folder}/${c.path}`;
  c.pages = new Set();
  for (const r of c.reads) {
    r.entry = resolve(r.protocol);
    if (r.entry) c.pages.add(r.entry.page);
  }
}
const capsOfPage = new Map();
for (const c of CAPS.captures) for (const p of c.pages) (capsOfPage.get(p) ?? capsOfPage.set(p, []).get(p)).push(c);
const capByRepoPath = new Map(CAPS.captures.map((c) => [c.repo, c]));

// Makes.
const tokens = (make) => make.split('/').map((t) => t.trim()).filter(Boolean);
for (const g of TAX.groups) {
  g.entries = entries.filter((p) => tokens(p.make).some((t) => g.tokens.includes(t)));
  g.pages = [...new Set(g.entries.map((p) => p.page))];
  g.captures = CAPS.captures.filter((c) => g.folders.includes(c.folder));
  g.fcc = FCC.ids.filter((i) => i.makes.some((m) => g.catalogue.includes(m)));
  g.href = `make-${g.slug}.html`;
}
const groupsOfPage = (page) => TAX.groups.filter((g) => g.pages.includes(page));
const groupOfFolder = (folder) => TAX.groups.find((g) => g.folders.includes(folder)) ?? TAX.captures_only.find((g) => g.folders.includes(folder));

// FCC IDs.
const famByKey = new Map(FCC.families.map((f) => [f.key, f]));
const fccOfPage = (page) => {
  const names = new Set((byPage.get(page) ?? []).flatMap((p) => [p.name, p.decoder]));
  return FCC.ids.filter((i) => i.protocols.some((n) => names.has(n)));
};
const EVIDENCE = {
  recording: ['recording', 'KAT reads a recording of this remote'],
  reference: ['reference', "The protocol's source names this FCC ID"],
  signal: ['signal match', "The filing's figures match the protocol; no recording confirms it"],
  assumed: ['assumed', 'Same maker, band and years as a filing that was read'],
};
const evidence = (e) => {
  const key = Object.keys(EVIDENCE).find((k) => e.startsWith(k)) ?? 'assumed';
  const [label, tip] = EVIDENCE[key];
  const auto = /only remote listed/.test(e);
  return `<span class="tag tag-${key === 'recording' && !auto ? 'ok' : key === 'reference' ? 'ok' : 'dim'}" title="${esc(auto ? 'KAT reads a recording of a car for which parts catalogues list this remote alone' : tip)}">${auto ? 'catalogue match' : label}</span>`;
};
const STATUS = {
  read: ['read', 'ok'], likely: ['likely read', 'warn'], known: ['signal known', 'warn'], blind: ['recorded, no data', 'bad'],
  unread: ['recorded, not read', 'bad'], filed: ['filing read', 'dim'], maybe: ['maybe', 'dim'],
};
/** A status tag. `row`: the FCC ID it stands beside. "Filing read" is a family's status; beside an ID whose
 *  own filing was not read it says so. */
const status = (s, row) => (s === 'filed' && row && !row.filing_read
  ? '<span class="tag tag-dim" title="The filing of another remote of this family was read; this one’s was not">family’s filing read</span>'
  : `<span class="tag tag-${STATUS[s]?.[1] ?? 'dim'}">${STATUS[s]?.[0] ?? esc(s)}</span>`);
/** Names joined by slashes ("Chevrolet/Buick/GMC/Holden") may break after one. Chromium takes such a string for
 *  one word, and one long make label set the width of its whole column. */
const slashes = (s) => esc(s).replace(/\//g, '/<wbr>');
// File paths in a fact box break after a slash ("src/protocols/" + "princeton.rs"), not inside a
// name as overflow-wrap alone broke them ("princeto" + "n.rs"). Only the text of <code> changes,
// never a link's address.
const codeBreaks = (html) => html.replace(/<code>([^<]*)<\/code>/g, (_, t) => `<code>${t.replace(/\//g, '/<wbr>')}</code>`);
/** A cell's text, or a dash where there is none. */
const orDash = (s) => (s ? esc(s) : '<span class="dim">–</span>');
/** The cell of an FCC ID: a link to its filing. The ID never breaks across lines. */
const fccCell = (id) => (RFCC_IDS.has(id)
  ? `<td class="fccid"><a href="remote-${anchor(id)}.html" title="The remote's page">${esc(id)}</a></td>`
  : `<td class="fccid"><a href="https://fccid.io/${encodeURIComponent(id)}" rel="noopener" title="The filing at fccid.io">${esc(id)}</a></td>`);
const RFCC_IDS = new Set(FCC.ids.map((i) => i.id));

// The numbers the pages quote. A decoder counts once, whatever names it shows; an entry that only
// replays saved frames (MARELLI) is no decoder.
const decoders = (category) => new Set(entries.filter((p) => p.registered && p.category === category && !/no decoder/i.test(p.encryption)).map((p) => p.decoder)).size;
const counts = {
  car: decoders('Vehicle'), gate: decoders('Gate'), decoders: decoders('Vehicle') + decoders('Gate'),
  recordings: CAPS.captures.length, read: CAPS.captures.filter((c) => c.reads.length).length,
  fcc: FCC.ids.length, makes: TAX.groups.length,
  remotes: FCC.ids.length + ROWN.remotes.length,
  pictured: FCC.ids.filter((i) => RFCC.remotes[i.id]?.photos.length).length + ROWN.remotes.length,
};

// ---------------------------------------------------------------------------
// Small pieces of HTML

const mhz = (hz) => (hz / 1e6).toFixed(3).replace(/\.?0+$/, '');
const size = (b) => (b < 1024 ? `${b} B` : b < 1048576 ? `${Math.round(b / 1024)} kB` : `${(b / 1048576).toFixed(1)} MB`);
const pageLink = (p, text) => `<a href="${p.page}.html">${esc(text ?? p.name)}</a>`;

function presetCell(c) {
  if (!c.preset) return '<span class="dim">–</span>';
  if (c.preset !== 'Custom' || !c.custom) return `<code>${esc(c.preset)}</code>`;
  const k = c.custom;
  const parts = [k.keying, k.deviation_hz ? `±${(k.deviation_hz / 1000).toFixed(1)} kHz` : null, k.baud ? `${(k.baud / 1000).toFixed(2)} kBd` : null].filter(Boolean);
  return `<code title="${esc(parts.join(', '))}">Custom</code> <span class="dim">${esc(parts.slice(0, 2).join(' '))}</span>`;
}

function readsCell(c, own) {
  if (!c.reads.length) return c.note ? `<span class="dim" title="${esc(c.note)}">not loaded</span>` : '<span class="dim">nothing read</span>';
  return c.reads.filter((r) => !own || own.has(r.entry?.page)).map((r) => {
    const name = r.entry ? `<a href="${r.entry.page}.html">${esc(r.protocol)}</a>` : esc(r.protocol);
    const n = r.distinct != null && r.distinct !== r.frames ? `${r.frames} frames, ${r.distinct} different` : plural(r.frames, 'frame');
    return `${name} <span class="dim">${n}</span>`;
  }).join('<br>');
}

/** A table of recordings. `own`: only the decodes of these pages are named. */
function captureTable(list, { own = null, folder = false, ids = false } = {}) {
  const rows = list.map((c) => `<tr${ids ? ` id="${c.id}"` : ''} data-q="${esc(`${c.path} ${c.reads.map((r) => r.protocol).join(' ')} ${c.reads.length ? 'read' : 'unread'} ${c.type}`.toLowerCase())}">
<td class="file"><a href="${blob(c.repo)}" title="${esc(c.repo)}">${esc(c.name)}</a><a class="dl" href="${raw(c.repo)}" title="Download ${esc(c.name)}" aria-label="Download ${esc(c.name)}">↓</a></td>
${folder ? `<td>${esc(c.path.split('/').slice(0, -1).join('/'))}</td>` : ''}<td class="num" data-v="${c.frequency ?? 0}">${c.frequency ? mhz(c.frequency) : '–'}</td>
<td>${presetCell(c)}</td>
<td>${c.type === 'raw' ? 'RAW' : c.type === 'key' ? `key file${c.key_protocol ? ` <span class="dim">${esc(c.key_protocol)}</span>` : ''}` : esc(c.type)}</td>
<td class="num" data-v="${c.seconds ?? 0}">${c.seconds ? `${c.seconds.toFixed(1)} s` : '–'}</td>
<td class="num" data-v="${c.bytes}">${size(c.bytes)}</td>
<td>${readsCell(c, own)}</td></tr>`).join('\n');
  return `<div class="table-wrap"><table class="data sortable">
<thead><tr><th>File</th>${folder ? '<th>Folder</th>' : ''}<th class="num">MHz</th><th>Preset</th><th>Type</th><th class="num">Length</th><th class="num">Size</th><th>KAT reads</th></tr></thead>
<tbody>
${rows}
</tbody></table></div>`;
}

function supportCell(p) {
  if (!p.registered) return '<span class="tag tag-dim" title="Ported and kept in the source, but not in the decoder list">not registered</span>';
  const tags = ['<span class="tag tag-ok">decode</span>'];
  if (p.encoder) tags.push('<span class="tag tag-ok">encode</span>');
  if (p.rtl433) tags.push('<span class="tag tag-dim" title="A port of rtl_433\'s decoder: receive only">rtl_433</span>');
  return tags.join(' ');
}

const timing = (p) => (p.te_short ? `${p.te_short} / ${p.te_long}` : '');

/** A table of protocol entries. */
function protocolTable(list, { make = true } = {}) {
  const rows = list.map((p) => {
    const caps = (capsOfPage.get(p.page) ?? []).filter((c) => c.reads.some((r) => r.entry === p)).length;
    return `<tr data-q="${esc(`${p.name} ${p.aliases.join(' ')} ${p.make} ${CODING[p.coding] ?? p.coding} ${RF_SHORT[p.rf]} ${p.encryption} ${p.category === 'Vehicle' ? 'car vehicle' : 'gate garage'} ${p.encoder ? 'encode' : ''} ${p.rtl433 ? 'rtl_433' : ''}`.toLowerCase())}">
<td>${pageLink(p)}</td>
${make ? `<td>${slashes(p.make)}</td>` : ''}<td>${p.category === 'Vehicle' ? 'Car' : 'Gate'}</td>
<td>${esc(CODING[p.coding] ?? p.coding)}</td>
<td class="nw">${RF_SHORT[p.rf] ?? esc(p.rf)}</td>
<td class="num" data-v="${p.te_short ?? 0}">${timing(p)}</td>
<td class="num" data-v="${p.bits ?? 0}">${p.bits ?? ''}</td>
<td>${esc(p.encryption)}</td>
<td class="nw">${supportCell(p)}</td>
<td class="num" data-v="${caps}">${caps ? `<a href="${p.page}.html#recordings">${caps}</a>` : ''}</td></tr>`;
  }).join('\n');
  return `<div class="table-wrap"><table class="data sortable">
<thead><tr><th>Protocol</th>${make ? '<th>Make</th>' : ''}<th>Kind</th><th>Line coding</th><th>RF</th><th class="num" title="Short and long pulse, µs">Pulses µs</th><th class="num" title="The shortest frame the decoder takes">Bits</th><th>Code</th><th>KAT</th><th class="num" title="Recordings in the library KAT reads with it">Rec.</th></tr></thead>
<tbody>
${rows}
</tbody></table></div>`;
}

function fccTable(list, { cars = true } = {}) {
  const rows = list.map((i) => {
    const fam = famByKey.get(i.family);
    const protos = i.protocols.map((n) => byName.get(n)).filter(Boolean);
    return `<tr id="fcc-${anchor(i.id)}" data-q="${esc(`${i.id} ${i.maker} ${i.makes.join(' ')} ${i.models} ${i.protocols.join(' ')} ${fam?.title ?? ''}`.toLowerCase())}">
${fccCell(i.id)}
<td>${orDash(titleCase(i.maker))}</td>
<td class="num" data-v="${i.granted.slice(0, 4) || 0}">${orDash(i.granted.slice(0, 4))}</td>
<td>${orDash(i.frequency)}</td>
${cars ? `<td title="${esc(i.models)}">${i.makes.length ? `${esc(i.makes.join(', '))}${i.years ? ` <span class="dim">${esc(i.years)}</span>` : ''}` : '<span class="dim">–</span>'}</td>` : ''}<td>${fam ? `<a href="fcc.html#fam-${fam.key}">${esc(fam.title)}</a>` : '<span class="dim">–</span>'}</td>
<td>${protos.length ? protos.map((p) => pageLink(p)).join(', ') : '<span class="dim">–</span>'}</td>
<td>${status(i.status, i)} ${i.status === 'read' || i.status === 'likely' ? evidence(i.evidence) : ''}</td></tr>`;
  }).join('\n');
  return `<div class="table-wrap"><table class="data sortable">
<thead><tr><th>FCC ID</th><th>Maker</th><th class="num">Year</th><th>Band</th>${cars ? '<th>Cars</th>' : ''}<th>Remote family</th><th>Protocol</th><th>Status</th></tr></thead>
<tbody>
${rows}
</tbody></table></div>`;
}

const titleCase = (s) => s.replace(/\b([A-Z])([A-Z]+)\b/g, (m, a, b) => (m.length <= 3 || /^(USA|LLC|GMBH)$/.test(m) ? m : a + b.toLowerCase()));

const filterBox = (label, target) => `<div class="table-find" data-filter="${target}">
<input type="search" placeholder="${esc(label)}" aria-label="${esc(label)}"><output aria-live="polite"></output>
</div>`;

// ---------------------------------------------------------------------------
// Remotes, chips, example frames

/** A line of the data files, which is Markdown without blocks: code, emphasis, links to pages. */
const inlineMd = new Marked({
  gfm: true,
  renderer: {
    link({ href, tokens: toks }) {
      const to = pageHref(href);
      return `<a href="${esc(to)}"${/^https?:/.test(to ?? '') ? ' rel="noopener"' : ''}>${this.parser.parseInline(toks)}</a>`;
    },
    html({ text }) {
      return esc(text);
    },
  },
});
const inline = (s) => inlineMd.parseInline(String(s));

const chipByKey = new Map(CHIPS.chips.map((c) => [c.key, c]));
const capsWith = (prefixes = []) => CAPS.captures.filter((c) => prefixes.some((p) => c.path.startsWith(p)));
const ownPhoto = (p) => ({ ...p, cap: inline(p.caption), by: esc(p.credit ?? '') });
const filedPhoto = (id) => (p) => ({
  ...p, cap: `${esc(p.exhibit)}${p.date ? `, ${esc(p.date.slice(0, 4))}` : ''}`,
  by: `<a href="${esc(p.source)}" rel="noopener">FCC filing of ${esc(id)}</a>`,
});

/** Every remote the site has a page for: one per FCC ID, and the remotes whose owners sent pictures. */
const remotes = [];
for (const i of FCC.ids) {
  const r = RFCC.remotes[i.id] ?? { photos: [], notes: [], chips: [] };
  const x = ROWN.fcc_extra[i.id] ?? {};
  const named = r.chips.filter((k) => !(x.chips ?? []).some((c) => c.chip === k)).map((k) => ({ chip: k, note: 'named in the filing' }));
  remotes.push({
    slug: `remote-${anchor(i.id)}`, fcc: i, title: i.id, short: i.id, maker: titleCase(i.maker),
    groups: TAX.groups.filter((g) => i.makes.some((m) => g.catalogue.includes(m))),
    entries: [...new Set([...i.protocols, ...(x.protocols ?? [])])].map((n) => byName.get(n)).filter(Boolean),
    photos: [...(x.photos ?? []).map(ownPhoto), ...r.photos.map(filedPhoto(i.id))],
    notes: r.notes, chips: [...(x.chips ?? []), ...named], facts: [], cars: null,
    captures: capsWith(x.recordings), note: x.note ?? null, summary: null,
  });
}
for (const o of ROWN.remotes) {
  remotes.push({
    slug: `remote-${o.slug}`, fcc: null, title: o.title, short: o.short, maker: o.maker,
    groups: TAX.groups.filter((g) => g.slug === o.group),
    entries: o.protocols.map((n) => byName.get(n)).filter(Boolean),
    photos: o.photos.map(ownPhoto), notes: [], chips: o.chips ?? [], facts: o.facts ?? [], cars: o.cars ?? null,
    captures: capsWith(o.recordings), note: null, summary: o.summary ?? null,
  });
}
for (const r of remotes) for (const c of r.chips) if (!chipByKey.has(c.chip)) throw new Error(`${r.slug}: chip ${c.chip} is not in data/chips.json`);
const remotesOfPage = (page) => remotes.filter((r) => r.entries.some((e) => e.page === page));
const remotesOfChip = (key) => remotes.filter((r) => r.chips.some((c) => c.chip === key));
/** Remotes with pictures first, then those KAT reads, then by ID. */
const byInterest = (a, b) => (b.photos.length > 0) - (a.photos.length > 0) || (b.fcc == null) - (a.fcc == null)
  || (b.entries.length > 0) - (a.entries.length > 0) || a.short.localeCompare(b.short);

const thumbSize = (p, max = 320) => { const k = Math.min(1, max / Math.max(p.w, p.h)); return `width="${Math.round(p.w * k)}" height="${Math.round(p.h * k)}"`; };
const figure = (p, alt) => `<figure class="shot"><a href="${esc(p.src)}"><img src="${esc(p.thumb)}" ${thumbSize(p)} loading="lazy" alt="${esc(alt)}"></a>
<figcaption>${p.cap}${p.by ? ` <span class="by">${p.by}</span>` : ''}</figcaption></figure>`;

function remoteCard(r) {
  const i = r.fcc;
  const sub = [r.maker, i?.frequency, i?.granted?.slice(0, 4)].filter(Boolean).join(' · ');
  const cars = i?.makes.length ? `${i.makes.join(', ')}${i.years ? ` ${i.years}` : ''}` : r.groups.map((g) => g.short).join(', ');
  const q = `${r.title} ${r.short} ${r.maker} ${i?.models ?? ''} ${cars} ${r.entries.map((e) => e.name).join(' ')} ${r.chips.map((c) => chipByKey.get(c.chip).name).join(' ')} ${r.photos.length ? 'picture' : ''}`.toLowerCase();
  const p = r.photos[0];
  return `<a class="fob-card" href="${r.slug}.html" data-q="${esc(q)}">
<span class="fob-img">${p ? `<img src="${esc(p.thumb)}" ${thumbSize(p)} loading="lazy" alt="">` : '<span class="fob-none">no picture</span>'}</span>
<span class="fob-name">${esc(r.short)}</span>
<span class="fob-sub">${esc(sub)}</span>
${cars ? `<span class="fob-cars">${esc(clip(cars, 70))}</span>` : ''}
${r.entries.length ? `<span class="fob-tags">${r.entries.slice(0, 2).map((e) => `<span class="tag tag-ok">${esc(e.name)}</span>`).join(' ')}</span>` : ''}
</a>`;
}
const remoteGrid = (list) => `<div class="fob-grid">\n${list.map(remoteCard).join('\n')}\n</div>`;

// --- example frames

const FIELD = { serial: 'Serial', button: 'Button', counter: 'Counter', check: 'Check', code: 'Code', fixed: 'Fixed' };
/** Protocols whose button names depend on the kind of frame: an example shows the code alone. */
const NAMES_BY_FRAME = new Set(['PSA']);
const GENERIC = { 1: 'Lock', 2: 'Unlock', 4: 'Trunk', 8: 'Panic' };
const buttonName = (entry, code) => (code == null || NAMES_BY_FRAME.has(entry.name) ? ''
  : entry.buttons?.find(([c]) => c === code)?.[1] ?? (entry.generic_buttons ? GENERIC[code] ?? '' : ''));

/** The example frames of one entry: a table of data words with the fields' digits marked, and where each field lies. */
function frameBlock(entry, many) {
  const f = FRAMES.protocols[entry.name];
  if (!f?.examples.length) return '';
  const hand = FHAND.protocols[entry.name];
  const fields = hand ? hand.fields : Object.entries(f.fields).map(([kind, x]) => ({ kind, name: FIELD[kind], ...x }));
  const showExtra = !hand?.hide_extra && f.examples.some((e) => e.extra != null);
  const digits = (word) => Math.max(1, ...f.examples.map((e) => (e[word] ?? '').length), ...fields.filter((x) => x.word === word).map((x) => Math.ceil((x.offset + x.width) / 4)));
  const hex = (value, word) => {
    if (value == null) return '<span class="dim">–</span>';
    const n = digits(word);
    return `<code class="frame-hex">${[...value.padStart(n, '0')].map((ch, k) => {
      const lo = 4 * (n - 1 - k);
      const hits = fields.filter((x) => x.word === word && x.offset < lo + 4 && x.offset + x.width > lo);
      const whole = hits.length === 1 && hits[0].offset <= lo && hits[0].offset + hits[0].width >= lo + 4;
      return `<span${hits.length ? ` class="${whole ? `f-${hits[0].kind}` : 'f-mixed'}" title="${esc(hits.map((x) => x.name).join(' / '))}"` : ''}>${ch}</span>`;
    }).join('')}</code>`;
  };
  const rows = f.examples.map((e) => {
    const cap = capByRepoPath.get(`${CAPS.folder}/${e.file}`);
    const name = buttonName(entry, e.button);
    return `<tr><td>${hex(e.data, 'data')}</td>${showExtra ? `<td>${hex(e.extra, 'extra')}</td>` : ''}
<td class="mono">${e.serial ?? '<span class="dim">–</span>'}</td>
<td>${e.button == null ? '<span class="dim">–</span>' : `<span class="mono">${e.button.toString(16).toUpperCase()}</span>${name ? ` ${esc(name)}` : ''}`}</td>
<td class="mono">${e.counter ?? '<span class="dim">–</span>'}</td>
<td class="file">${cap ? `<a href="${blob(cap.repo)}" title="${esc(cap.repo)}">${esc(cap.name)}</a>` : esc(e.file.split('/').pop())}${e.name !== entry.name ? ` <span class="dim">${esc(e.name)}</span>` : ''}</td></tr>`;
  }).join('\n');
  const where = (x) => {
    const hi = x.offset + x.width - 1;
    const bits = x.width === 1 ? `bit ${x.offset}` : `bits ${hi} to ${x.offset}`;
    const how = x.how === 'reversed' ? ', read backwards' : x.how === 'inverted' ? ', inverted' : '';
    const seen = x.frames ? ` <span class="dim">(${x.values === 1 ? 'one value' : `${x.values.toLocaleString('en-US')} different values`} in ${plural(x.frames, 'frame')})</span>` : '';
    return `<li><span class="f-key f-${x.kind}"></span><strong>${esc(x.name)}</strong>: ${bits} of the ${x.word === 'extra' ? 'extra' : 'data'} word${how}${x.note ? `; ${inline(x.note)}` : ''}${seen}</li>`;
  };
  const missing = hand ? [] : ['serial', 'button', 'counter'].filter((k) => !(k in f.fields) && f.examples.some((e) => e[k] != null));
  return `${many ? `<h3>${esc(entry.name)}</h3>\n` : ''}<div class="table-wrap"><table class="data frames">
<thead><tr><th title="The frame as KAT shows it: up to 64 bits, in hexadecimal">Data word</th>${showExtra ? '<th title="What is left of a frame longer than 64 bits">Extra</th>' : ''}<th>Serial</th><th>Button</th><th>Counter</th><th>Recording</th></tr></thead>
<tbody>
${rows}
</tbody></table></div>
${fields.length ? `<ul class="frame-fields">\n${fields.slice().sort((a, b) => (a.word === b.word ? b.offset - a.offset : a.word === 'data' ? -1 : 1)).map(where).join('\n')}\n</ul>` : ''}
${hand?.note ? `<p class="note">${inline(hand.note)}</p>` : ''}${missing.length ? `<p class="note">The ${missing.join(' and the ')} ${missing.length > 1 ? 'are' : 'is'} not marked: no run of bits of the data word gives ${missing.length > 1 ? 'them' : 'it'} in every frame, so KAT works ${missing.length > 1 ? 'them' : 'it'} out (by decrypting, or from bits that are spread over the frame).</p>` : ''}`;
}

function frameSection(page) {
  const list = (byPage.get(page) ?? []).filter((p) => FRAMES.protocols[p.name]?.examples.length);
  if (!list.length) return '';
  const n = list.reduce((m, p) => m + FRAMES.protocols[p.name].frames, 0);
  return `<h2 id="example-frames">Example frames<a class="h-anchor" href="#example-frames" aria-label="Link to this section">#</a></h2>
<p>Frames KAT reads from recordings in the <a href="captures.html">library</a>, as KAT shows them: the data word in hexadecimal (bit 0 is its last bit), and the serial, button and counter KAT gives for it. A digit is marked where its bits hold a field: a field is listed only when the same bits give KAT's value in every frame of the protocol in the library (${plural(n, 'frame')} here). The digits left unmarked are the rest of the frame: fixed bits, a checksum, an encrypted part; the sections above say which.</p>
${list.map((p) => frameBlock(p, list.length > 1)).join('\n')}`;
}

// --- a remote's page

function remotePage(r, parts) {
  const i = r.fcc;
  const toc = [];
  const h2 = (id, text) => { toc.push({ depth: 2, id, text }); return `<h2 id="${id}">${text}<a class="h-anchor" href="#${id}" aria-label="Link to this section">#</a></h2>`; };
  const row = (k, v) => (v ? `<div><dt>${k}</dt><dd>${v}</dd></div>` : '');
  const fam = i ? famByKey.get(i.family) : null;
  const hero = r.photos[0];
  const box = `<aside class="infobox" aria-label="Facts about ${esc(r.short)}">
${hero ? `<a class="infobox-img" href="${esc(hero.src)}"><img src="${esc(hero.thumb)}" ${thumbSize(hero)} alt="${esc(r.short)}"></a>` : ''}
<div class="infobox-title">${esc(r.short)}</div>
<div class="infobox-sub">${i ? esc(titleCase(i.product) || 'Remote') : 'Remote'}</div>
<dl>
${row('FCC ID', i ? `<a href="https://fccid.io/${encodeURIComponent(i.id)}" rel="noopener" title="The filing at fccid.io"><code>${esc(i.id)}</code></a>` : '<span class="dim">none: not a US remote</span>')}
${row('Maker', esc(r.maker))}
${row('Granted', i?.granted ? esc(i.granted) : '')}
${row('Band', i?.frequency ? esc(i.frequency) : '')}
${row('Makes', r.groups.length ? r.groups.map((g) => `<a href="${g.href}">${esc(g.short)}</a>`).join(', ') : i?.makes.length ? esc(i.makes.join(', ')) : '')}
${row('Years', i?.years ? `${esc(i.years)} <span class="dim">catalogues</span>` : '')}
${row('Family', fam ? `<a href="fcc.html#fam-${fam.key}">${esc(fam.title)}</a>` : '')}
${row('Protocol', r.entries.length ? r.entries.map((e) => pageLink(e)).join(', ') : i ? '<span class="dim">none tied yet</span>' : '')}
${row('Status', i ? `${status(i.status, i)} ${i.status === 'read' || i.status === 'likely' ? evidence(i.evidence) : ''}` : '')}
${row('Chips', r.chips.map((c) => `<a href="chip-${c.chip}.html">${inline(chipByKey.get(c.chip).name)}</a>`).join(', '))}
${row('Recordings', r.captures.length ? `<a href="#recordings">${plural(r.captures.length, 'file')}</a>` : '')}
${r.facts.map(([k, v]) => row(esc(k), inline(v))).join('\n')}
</dl>
</aside>
`;
  const lead = i
    ? `<p class="lead"><code>${esc(i.id)}</code> is the FCC ID of ${i.product ? `a remote its filing calls “${esc(titleCase(i.product))}”` : 'a car remote'}${r.maker ? `, by ${esc(r.maker)}` : ''}${i.granted ? `, granted in ${esc(i.granted.slice(0, 4))}` : ''}.${i.models ? ` Parts catalogues list it for the ${esc(i.models)}${i.years ? ` (${esc(i.years)})` : ''}.` : ''}</p>${r.note ? `\n<p>${inline(r.note)}</p>` : ''}`
    : `<p class="lead">${inline(r.summary ?? '')}</p>`;
  const pictures = r.photos.length ? `${h2('pictures', 'Pictures')}
<div class="shots">
${r.photos.map((p) => figure(p, `${r.short}: ${unentity(p.cap.replace(/<[^>]+>/g, ''))}`)).join('\n')}
</div>
${i && r.photos.some((p) => p.exhibit) ? `<p class="note">Pictures marked “FCC filing” are from the filing's exhibit of internal photos; a picture's source line opens the exhibit.</p>` : ''}` : '';
  const filing = r.notes.length ? `${h2('filing', 'What the filing says')}
<p>Notes taken from the exhibits of the <a href="https://fccid.io/${encodeURIComponent(i.id)}" rel="noopener">filing</a>. Text in quotation marks is the filing's own. A note marked <span class="tag tag-ok">quoted</span> has every one of its quotations found again in the text of the exhibits; the others were read from scanned pages and figures, where a search cannot check them.</p>
<dl class="filing">
${r.notes.map((n) => `<div><dt>${esc(n.label)}${n.quoted ? ' <span class="tag tag-ok" title="Every quotation in this note is in the text of the exhibits">quoted</span>' : ''}</dt>
<dd>${n.lines.length === 1 ? inline(n.lines[0]) : `${n.lines[0].endsWith(':') || n.lines.length > 1 ? `<ul>${n.lines.map((l) => `<li>${inline(l)}</li>`).join('')}</ul>` : inline(n.lines[0])}`}</dd></div>`).join('\n')}
</dl>` : '';
  const cars = i?.models ? `${h2('cars', 'Cars')}
<p>North American parts catalogues list this remote for: ${esc(i.models)}${i.years ? `, ${esc(i.years)}` : ''} (${plural(i.model_years, 'model-year')}). The lists come from shops' catalogues and carry their mistakes; years between the first and last a catalogue lists are filled in.</p>`
    : r.cars ? `${h2('cars', 'Cars')}\n<p>${inline(r.cars)}</p>` : '';
  const proto = r.entries.length ? `${h2('protocol', 'Protocol')}
<p>${i ? `KAT ties this remote to ${r.entries.map((e) => pageLink(e)).join(', ')}: ${status(i.status, i)} ${i.status === 'read' || i.status === 'likely' ? evidence(i.evidence) : ''}${fam ? ` It belongs to the family “<a href="fcc.html#fam-${fam.key}">${esc(fam.title)}</a>”: ${esc(fam.signal)}.` : ''}`
    : `KAT reads this remote as ${r.entries.map((e) => pageLink(e)).join(', ')}.`}</p>
<ul>
${r.entries.map((e) => `<li>${pageLink(e)} <span class="dim">${esc([CODING[e.coding] ?? e.coding, RF_SHORT[e.rf] ?? e.rf, e.te_short ? `${e.te_short} / ${e.te_long} µs` : null, e.bits ? `${e.bits} bits` : null, e.encryption].filter(Boolean).join(' · '))}</span> ${supportCell(e)}</li>`).join('\n')}
</ul>` : i && fam ? `${h2('protocol', 'Protocol')}
<p>No KAT protocol is tied to this remote yet. It belongs to the family “<a href="fcc.html#fam-${fam.key}">${esc(fam.title)}</a>”: ${esc(fam.signal)}. ${status(i.status, i)}</p>` : '';
  const chips = r.chips.length ? `${h2('chips', 'Chips')}
<ul>
${r.chips.map((c) => `<li><a href="chip-${c.chip}.html">${inline(chipByKey.get(c.chip).name)}</a> <span class="dim">${esc(chipByKey.get(c.chip).kind)}</span>: ${inline(c.note)}.</li>`).join('\n')}
</ul>` : '';
  const recs = r.captures.length ? `${h2('recordings', 'Recordings')}
<p>${plural(r.captures.length, 'recording')} of this remote in the <a href="captures.html">library</a>.</p>
${captureTable(r.captures.slice().sort((a, b) => a.path.localeCompare(b.path)), { folder: true })}` : '';
  const frames = r.entries.length && !i ? r.entries.map((e) => frameSection(e.page)).join('\n') : '';
  if (frames) toc.push({ depth: 2, id: 'example-frames', text: 'Example frames' });
  const g = r.groups[0];
  const near = remotes.filter((x) => x !== r && g && x.groups.includes(g)).sort(byInterest).slice(0, 30);
  return wikiPage({
    file: `${r.slug}.html`, title: r.title, wide: false,
    description: clip(i ? `${i.id}: ${titleCase(i.product) || 'car remote'} by ${r.maker}${i.models ? `, for the ${i.models}` : ''}. Pictures of the board, what the FCC filing says and the KAT protocol that reads it.` : unentity((r.summary ?? '').replace(/[`*\[\]]|\([^)]*\.md[^)]*\)/g, ''))),
    trail: [['./', 'KAT'], ['remotes.html', 'Remotes'], ...(g ? [[g.href, g.short]] : [])],
    side: tocList(toc) + sideList(g ? `${g.short} remotes` : '', near.map((x) => [`${x.slug}.html`, x.short]), `${r.slug}.html`),
    lead: box, body: `${lead}\n${pictures}\n${filing}\n${cars}\n${proto}\n${frames}\n${chips}\n${recs}\n`,
    foot: `      <div class="doc-foot"><a href="remotes.html">← All remotes</a><span>${i ? `The filing: <a href="https://fccid.io/${encodeURIComponent(i.id)}" rel="noopener">fccid.io/${esc(i.id)}</a>, a mirror of the FCC's database.` : 'The pictures were sent in by the remote’s owner.'}</span></div>`,
  }, parts);
}

function chipPage(c, parts) {
  const list = remotesOfChip(c.key).sort(byInterest);
  const pagesOf = (c.pages ?? []).filter((p) => byPage.has(p));
  const box = `<aside class="infobox" aria-label="Facts about ${esc(c.name)}">
<div class="infobox-title">${inline(c.name)}</div>
<div class="infobox-sub">${esc(c.kind)}</div>
<dl>
${c.maker ? `<div><dt>Maker</dt><dd>${esc(c.maker)}</dd></div>` : ''}
<div><dt>Data sheet</dt><dd>${c.datasheet ? `<a href="${esc(c.datasheet.url)}" rel="noopener">${esc(c.datasheet.title)}</a>` : '<span class="dim">none found in public</span>'}</dd></div>
<div><dt>Remotes</dt><dd>${list.length || '<span class="dim">none pictured</span>'}</dd></div>
${pagesOf.length ? `<div><dt>Protocols</dt><dd>${pagesOf.map((p) => `<a href="${p}.html">${esc(titles.get(p) ?? p)}</a>`).join(', ')}</dd></div>` : ''}
</dl>
</aside>
`;
  return wikiPage({
    file: `chip-${c.key}.html`, title: unentity(inline(c.name).replace(/<[^>]+>/g, '')),
    description: clip(`${c.name.replace(/`/g, '')}${c.maker ? ` (${c.maker})` : ''}: ${c.kind.toLowerCase()}. ${unentity(c.text.replace(/[`*\[\]]|\([^)]*\)/g, ''))}`),
    trail: [['./', 'KAT'], ['remotes.html', 'Remotes'], ['chips.html', 'Chips']],
    side: sideList('Chips', CHIPS.chips.map((x) => [`chip-${x.key}.html`, x.name.replace(/`/g, '')]), `chip-${c.key}.html`),
    lead: box,
    body: `<p class="lead">${inline(c.text)}</p>
${list.length ? `<h2 id="remotes">Remotes<a class="h-anchor" href="#remotes" aria-label="Link to this section">#</a></h2>
<ul>
${list.map((r) => `<li><a href="${r.slug}.html">${esc(r.short)}</a>${r.maker ? ` <span class="dim">${esc(r.maker)}</span>` : ''}: ${inline(r.chips.find((x) => x.chip === c.key).note)}.</li>`).join('\n')}
</ul>
${remoteGrid(list.filter((r) => r.photos.length))}` : ''}`,
    foot: '      <div class="doc-foot"><a href="chips.html">← All chips</a><span>A chip is listed where a picture of a board shows it or an FCC filing names it.</span></div>',
  }, parts);
}

// ---------------------------------------------------------------------------
// Markdown to HTML

/** Jekyll front matter (`---` … `---`) at the top of a file, removed. */
const stripFrontMatter = (src) => src.replace(/^---\r?\n[\s\S]*?\r?\n---\r?\n/, '');

/** The plain text of inline tokens: what a reader sees, without markup. */
function plain(toks = []) {
  return toks.map((t) => (t.tokens ? plain(t.tokens) : t.type === 'br' ? ' ' : t.text ?? '')).join('');
}
const unentity = (s) => s.replace(/&(amp|lt|gt|quot|#39);/g, (_, e) => ({ amp: '&', lt: '<', gt: '>', quot: '"', '#39': "'" })[e]);

// kramdown's GFM header ids: lowercase the heading's source text, drop all but
// letters, digits, underscores, hyphens and spaces, make each space a hyphen,
// and number repeats -1, -2 …
function slugger() {
  const seen = new Map();
  return (rawText) => {
    const base = rawText.toLowerCase().replace(/[^\p{L}\p{M}\p{N}\p{Pc}\- \t]/gu, '').replace(/[ \t]/g, '-');
    const n = seen.has(base) ? seen.get(base) + 1 : 0;
    seen.set(base, n);
    return n ? `${base}-${n}` : base;
  };
}

/** A link to a sibling page's .md, pointed at its .html. Other links stay. */
function pageHref(href) {
  const m = /^(?:docs\/)?([A-Za-z0-9_.-]+)\.md(#.*)?$/.exec(href ?? '');
  return m ? `${m[1]}.html${m[2] ?? ''}` : href;
}

/** Cut a description to about `max` characters, at a word. */
function clip(s, max = 200) {
  if (s.length <= max) return s;
  return s.slice(0, s.lastIndexOf(' ', max)).replace(/[\s,;:.(–—-]+$/, '') + '…';
}

/** Where a path named in the text can be opened: a recording, a folder of them, a source file. */
function pathHref(text) {
  const t = text.replace(/[.,;:]+$/, '');
  if (capByRepoPath.has(t)) return blob(t);
  const dir = t.replace(/\/$/, '');
  if (dir === CAPS.folder) return `${REPO}/tree/${CAPS.branch}/${CAPS.folder}`;
  if (/^IMPORTS\/[^*…{<]+$/.test(dir) && CAPS.captures.some((c) => c.repo.startsWith(`${dir}/`))) return `${REPO}/tree/${CAPS.branch}/${repoPath(dir)}`;
  if (/^(src|scripts|testdata)\/[\w./-]+\.(rs|py|txt)$/.test(t)) return blob(t);
  if (/^docs\/[\w-]+\.md$/.test(t)) return pageHref(t);
  return null;
}

/**
 * A Markdown page as parts: its title, the facts under the title ("Rust
 * module:", "Reference:"), its sections for the list beside it, and its body
 * without the title.
 */
function render(name, src) {
  const slug = slugger();
  let title = null, section = '', overview = null, first = null;
  const meta = [], toc = [];
  const marked = new Marked({
    gfm: true,
    renderer: {
      heading({ tokens: toks, depth, text }) {
        const id = slug(text);
        const words = unentity(plain(toks)).trim();
        section = words.toLowerCase();
        if (depth === 1 && title == null) {
          title = words;
          return '';
        }
        if (depth === 2 || depth === 3) toc.push({ depth, id, text: words });
        return `<h${depth} id="${esc(id)}">${this.parser.parseInline(toks)}<a class="h-anchor" href="#${esc(id)}" aria-label="Link to this section">#</a></h${depth}>\n`;
      },
      paragraph({ tokens: toks }) {
        // The label lines under the title go into the box of facts.
        const label = toks[0]?.type === 'strong' && plain(toks[0].tokens).trim().endsWith(':');
        if (label && !toc.length && title != null && section === title.toLowerCase()) {
          let cur = null;
          for (const t of toks) {
            if (t.type === 'strong' && plain(t.tokens).trim().endsWith(':')) {
              cur = { label: plain(t.tokens).trim().slice(0, -1), tokens: [] };
              meta.push(cur);
            } else if (cur && t.type !== 'br') cur.tokens.push(t);
          }
          for (const m of meta) if (m.tokens) { m.html = this.parser.parseInline(m.tokens).trim(); delete m.tokens; }
          return '';
        }
        // The page's description: the first paragraph under "Overview", or
        // else the first one that isn't a label line.
        const text = unentity(plain(toks)).replace(/\s+/g, ' ').trim();
        if (overview == null && section === 'overview' && text) overview = text;
        if (first == null && !label && text) first = text;
        return false; // marked's own <p>
      },
      // The docs carry no HTML on purpose, so anything that looks like a tag is
      // text: radio.md's "Receiver stopped: <reason>" must not vanish.
      html({ text }) {
        return esc(text);
      },
      link({ href, title: t, tokens: toks }) {
        // A link to this site's own address (the pages this script writes have no .md) stays on the site.
        const own = href === ORIGIN ? '' : href?.startsWith(`${ORIGIN}/`) ? href.slice(ORIGIN.length + 1) : null;
        const to = pageHref(own == null ? href : own === '' || own.startsWith('#') ? `./${own}` : own);
        const out = /^https?:/.test(to ?? '') ? ' rel="noopener"' : '';
        return `<a href="${esc(to)}"${t ? ` title="${esc(t)}"` : ''}${out}>${this.parser.parseInline(toks)}</a>`;
      },
      // A recording or a source file named in the text opens in the repository.
      codespan({ text }) {
        const shown = unentity(text);
        const href = pathHref(shown);
        const code = `<code>${esc(shown)}</code>`;
        return href ? `<a class="path" href="${esc(href)}">${code}</a>` : code;
      },
    },
  });
  // Wide tables scroll sideways inside their own box on a phone.
  const body = marked.parse(stripFrontMatter(src))
    .replace(/<table>/g, '<div class="table-wrap"><table>')
    .replace(/<\/table>/g, '</table></div>');
  return { title: title ?? name, description: clip(overview ?? first ?? ''), body, meta, toc };
}

// ---------------------------------------------------------------------------
// Page shell

/** ?v=<content hash> on every local asset a page names, so a changed file is
 *  never served from a stale cache under its old name. */
function versioned(html, hashes) {
  return html.replace(/(href|src|content|data-search)="((?:https:\/\/kat\.karazajac\.io\/)?)(style\.css|app\.js|site\.js|search\.json|logo\.svg|og\.png|img\/[^"?#]+)"/g,
    (m, attr, origin, path) => (hashes.has(path) ? `${attr}="${origin}${path}?v=${hashes.get(path)}"` : m));
}

/** index.html's own header or footer, with its #section links pointed back at
 *  the landing page. */
function borrow(index, re) {
  const m = re.exec(index);
  if (!m) throw new Error(`index.html: no match for ${re}`);
  return m[0].replace(/href="#"/g, 'href="./"').replace(/href="#([^"]+)"/g, 'href="./#$1"');
}

const crumbs = (trail) => `<nav class="crumbs" aria-label="Breadcrumb">${trail.map(([href, text], i) =>
  `${i ? '<span aria-hidden="true">›</span>' : ''}${href ? `<a href="${href}">${esc(text)}</a>` : `<span>${esc(text)}</span>`}`).join('')}</nav>`;

function shell({ file, title, description, main, type = 'article' }, parts) {
  const url = `${ORIGIN}/${file}`;
  return `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>${esc(title)} · KAT</title>
<meta name="description" content="${esc(description)}">
<link rel="canonical" href="${url}">
<meta property="og:title" content="${esc(title)}">
<meta property="og:description" content="${esc(description)}">
<meta property="og:type" content="${type}">
<meta property="og:url" content="${url}">
<meta property="og:site_name" content="KAT — Keyfob Analysis Toolkit">
<meta property="og:image" content="${ORIGIN}/og.png">
<meta property="og:image:width" content="1200">
<meta property="og:image:height" content="630">
<meta name="twitter:card" content="summary_large_image">
<meta name="twitter:image" content="${ORIGIN}/og.png">
<meta name="theme-color" content="#07070d">
<link rel="icon" type="image/svg+xml" href="logo.svg">
<link rel="stylesheet" href="style.css">
<script src="site.js" data-search="search.json" defer></script>
</head>
<body>

${parts.mesh}

${parts.nav}

${main}

${parts.footer}

</body>
</html>
`;
}

const sideList = (title, items, here) => (items.length ? `<div class="side-block"><h2>${esc(title)}</h2><ul>${items.map(([href, text]) =>
  `<li><a href="${href}"${href === here ? ' aria-current="page"' : ''}>${esc(text)}</a></li>`).join('')}</ul></div>` : '');

const tocList = (toc) => (toc.length > 1 ? `<div class="side-block side-toc"><h2>On this page</h2><ul>${toc.map((t) =>
  `<li class="d${t.depth}"><a href="#${esc(t.id)}">${esc(t.text)}</a></li>`).join('')}</ul></div>` : '');

/** A page of the reference work: the list beside it, the trail above it, the article. */
function wikiPage({ file, title, description, trail, side = '', lead = '', body, foot = '', wide = false }, parts) {
  const main = `<main class="doc wiki${wide ? ' wiki-wide' : ''}">
  <div class="container wiki-grid">
    <aside class="wiki-side" aria-label="Contents">
${side}
    </aside>
    <div class="wiki-main">
      ${crumbs(trail)}
      <article class="prose">
<h1>${esc(title)}</h1>
${lead}${body}      </article>
${foot}
    </div>
  </div>
</main>`;
  return shell({ file, title, description, main }, parts);
}

// ---------------------------------------------------------------------------
// A protocol's page

function infobox(page, r) {
  const list = byPage.get(page) ?? [];
  if (!list.length) return '';
  const main = list.find((p) => p.registered) ?? list[0];
  const others = list.filter((p) => p !== main);
  const uniq = (f) => [...new Set(list.map(f).filter(Boolean))];
  const groups = groupsOfPage(page);
  const caps = capsOfPage.get(page) ?? [];
  const bands = [...new Set(caps.filter((c) => c.frequency).map((c) => mhz(c.frequency)))].sort((a, b) => a - b);
  const fcc = fccOfPage(page);
  const row = (k, v, tip) => (v ? `<div><dt${tip ? ` title="${esc(tip)}"` : ''}>${k}</dt><dd>${v}</dd></div>` : '');
  const code = r.meta.find((m) => /^rust module/i.test(m.label));
  const ref = r.meta.filter((m) => !/^rust module/i.test(m.label));
  const presets = uniq((p) => p.presets.join(' '))[0]?.split(' ') ?? [];
  const pulses = uniq((p) => (p.te_short ? `${p.te_short} / ${p.te_long} µs` : null));
  const bits = uniq((p) => p.bits);
  const pictured = remotesOfPage(page).filter((x) => x.photos.length).sort(byInterest)[0];
  return `<aside class="infobox" aria-label="Facts about ${esc(main.name)}">
${pictured ? `<a class="infobox-img" href="${pictured.slug}.html" title="${esc(pictured.title)}"><img src="${esc(pictured.photos[0].thumb)}" ${thumbSize(pictured.photos[0])} alt="${esc(pictured.title)}"><span>${esc(pictured.short)}</span></a>` : ''}
<div class="infobox-title">${esc(main.name)}</div>
<div class="infobox-sub">${esc(KIND[main.category])}</div>
<dl>
${row('Makes', groups.length ? groups.map((g) => `<a href="${g.href}">${esc(g.short)}</a>`).join(', ') : esc(uniq((p) => p.make).join(', ')))}
${row('Also read here', others.length ? others.map((p) => esc(p.name)).join(', ') : '', 'Other names this page covers')}
${row('Also known as', uniq((p) => p.aliases.join(', ')).filter(Boolean).map(esc).join(', '))}
${row('Modulation', uniq((p) => RF[p.rf]).map(esc).join('; '))}
${row('Presets', presets.map((p) => `<code>${esc(p)}</code>`).join(' '), 'The Flipper presets KAT sends it with')}
${row('Recorded at', bands.length ? `${bands.slice(0, 6).join(', ')}${bands.length > 6 ? ', …' : ''} MHz` : '', 'The frequencies of the recordings KAT reads it from')}
${row('Line coding', uniq((p) => CODING[p.coding] ?? p.coding).map(esc).join(', '))}
${row('Pulses', pulses.slice(0, 2).map(esc).join('; '), 'The short and the long pulse the decoder is built around')}
${row('Frame', bits.length ? `${bits.slice(0, 3).join(', ')} bits` : '', 'The shortest frame the decoder takes')}
${row('Code', uniq((p) => p.encryption).slice(0, 3).map(esc).join('; '))}
${row('In KAT', supportCell(main))}
${row('Recordings', caps.length ? `<a href="#recordings">${plural(caps.length, 'file')}</a>` : '<span class="dim">none in the library</span>')}
${row('FCC IDs', fcc.length ? `${fcc.slice(0, 6).map((i) => `<a href="remote-${anchor(i.id)}.html"><code>${esc(i.id)}</code></a>`).join(' ')}${fcc.length > 6 ? ` <a href="#fcc-ids">+${fcc.length - 6}</a>` : ''}` : '')}
${row('Source', code ? codeBreaks(code.html) : '')}
${ref.map((m) => row(esc(m.label), codeBreaks(m.html))).join('\n')}
</dl>
</aside>
`;
}

function protocolSections(page) {
  const out = [], toc = [];
  const caps = (capsOfPage.get(page) ?? []).slice().sort((a, b) => a.path.localeCompare(b.path));
  const own = new Set([page]);
  const frames = frameSection(page);
  if (frames) {
    out.push(frames);
    toc.push({ depth: 2, id: 'example-frames', text: 'Example frames' });
  }
  const fobs = remotesOfPage(page).filter((r) => r.photos.length).sort(byInterest);
  if (fobs.length) {
    out.push(`<h2 id="remotes">Remotes<a class="h-anchor" href="#remotes" aria-label="Link to this section">#</a></h2>
<p>Remotes tied to this protocol that the site has pictures of. Each opens its own page: the pictures, what is known of the hardware, and how sure the tie is.</p>
${remoteGrid(fobs.slice(0, 12))}`);
    toc.push({ depth: 2, id: 'remotes', text: 'Remotes' });
  }
  if (caps.length) {
    const frames = caps.reduce((n, c) => n + c.reads.filter((r) => r.entry?.page === page).reduce((m, r) => m + r.frames, 0), 0);
    const shown = caps.slice(0, 25), rest = caps.slice(25);
    out.push(`<h2 id="recordings">Recordings<a class="h-anchor" href="#recordings" aria-label="Link to this section">#</a></h2>
<p>KAT reads ${plural(frames, 'frame')} of this protocol from ${plural(caps.length, 'recording')} in the <a href="captures.html">library</a>. A file name opens the recording in the repository; the arrow downloads it.</p>
${captureTable(shown, { own, folder: true })}${rest.length ? `<details class="more"><summary>${plural(rest.length, 'more recording')}</summary>
${captureTable(rest, { own, folder: true })}</details>` : ''}`);
    toc.push({ depth: 2, id: 'recordings', text: 'Recordings' });
  }
  const fcc = fccOfPage(page);
  if (fcc.length) {
    out.push(`<h2 id="fcc-ids">FCC IDs<a class="h-anchor" href="#fcc-ids" aria-label="Link to this section">#</a></h2>
<p>Remotes whose FCC IDs are tied to this protocol, and how sure the tie is. An ID opens the remote's page. The <a href="fcc.html">FCC ID table</a> has them all, with the remotes of other protocols.</p>
${fccTable(fcc)}`);
    toc.push({ depth: 2, id: 'fcc-ids', text: 'FCC IDs' });
  }
  return { html: out.join('\n'), toc };
}

function protocolPage(page, r, parts) {
  const groups = groupsOfPage(page);
  const list = byPage.get(page) ?? [];
  const gate = list.length && list.every((p) => p.category === 'Gate');
  const extra = protocolSections(page);
  const g = groups[0];
  const siblings = g ? g.pages.map((p) => [`${p}.html`, titles.get(p) ?? p]) : [];
  const trail = [['./', 'KAT'], ['protocols.html', 'Protocols'], ...(g ? [[g.href, g.short]] : gate ? [['makes.html#gate', 'Gate and garage remotes']] : [])];
  const side = tocList([...r.toc, ...extra.toc]) + sideList(g ? g.short : '', siblings.slice(0, 40), `${page}.html`);
  return wikiPage({
    file: `${page}.html`, title: r.title, description: r.description, trail, side,
    lead: infobox(page, r),
    body: `${r.body}${extra.html}`,
    foot: docFoot(page, 'protocols.html', 'All protocols'),
  }, parts);
}

const docFoot = (page, back, backText) => `      <div class="doc-foot">
        <a href="${back}">← ${backText}</a>
        <span>This page is <a href="${REPO}/blob/main/docs/${page}.md">docs/${page}.md</a> in the KAT repository${byPage.has(page) ? '; its facts and tables come from the program and its recordings' : ''}.</span>
      </div>`;

// ---------------------------------------------------------------------------

const args = process.argv.slice(2);
const redirect = args[0] === '--redirect';
const target = redirect ? (args[1] ?? '').replace(/\/+$/, '') : null;
if (redirect && !/^https:\/\/[^/]+$/.test(target)) {
  console.error('usage: node build.mjs [--redirect https://host]');
  process.exit(2);
}

const files = readdirSync(HERE).filter((f) => !SKIP.has(f) && !f.startsWith('.'));
const pages = files.filter((f) => f.endsWith('.md')).map((f) => f.slice(0, -3)).sort();
// Pages this script writes itself.
const made = ['protocols', 'makes', ...TAX.groups.map((g) => `make-${g.slug}`), 'captures', 'fcc', 'coverage', 'guides',
  'remotes', 'chips', ...remotes.map((r) => r.slug), ...CHIPS.chips.map((c) => `chip-${c.key}`)];
const changelogSrc = join(HERE, '..', 'CHANGELOG.md');
if (existsSync(changelogSrc)) made.push('changelog');
for (const p of made) if (pages.includes(p)) throw new Error(`${p}.md would be overwritten by the page of that name this script writes`);

rmSync(OUT, { recursive: true, force: true });
mkdirSync(OUT, { recursive: true });

if (redirect) {
  writeFileSync(join(OUT, 'index.html'), redirectPage(`${target}/`));
  for (const p of [...pages, ...made]) writeFileSync(join(OUT, `${p}.html`), redirectPage(`${target}/${p}.html`));
  // Any other old address: GitHub Pages serves 404.html, which forwards the
  // same path, minus the /KAT project prefix, to the new host.
  writeFileSync(join(OUT, '404.html'), redirectPage(`${target}/`).replace(
    /<script>.*<\/script>/,
    `<script>location.replace(${JSON.stringify(target)} + location.pathname.replace(/^\\/KAT(?=\\/|$)/, '') + location.search + location.hash);</script>`));
  console.log(`dist/: ${pages.length + made.length + 2} pages forwarding to ${target}`);
  process.exit(0);
}

function redirectPage(to) {
  return `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>KAT has moved</title>
<meta name="robots" content="noindex">
<link rel="canonical" href="${esc(to)}">
<script>location.replace(${JSON.stringify(to)} + location.hash);</script>
<meta http-equiv="refresh" content="0; url=${esc(to)}">
</head>
<body>
<p>KAT's site has moved to <a href="${esc(to)}">${esc(to.replace(/^https:\/\//, ''))}</a>.</p>
</body>
</html>
`;
}

// Static files go across as they are.
for (const f of files) if (!f.endsWith('.md')) cpSync(join(HERE, f), join(OUT, f), { recursive: true });

// The landing page links the changelog page. Built without the repository beside this folder there is
// none, and the links go to the file in the repository.
const index = ((src) => (made.includes('changelog') ? src : src.replaceAll('href="changelog.html"', `href="${REPO}/blob/main/CHANGELOG.md"`)))(
  readFileSync(join(HERE, 'index.html'), 'utf8'));
const parts = {
  mesh: borrow(index, /<div class="bg-mesh"[\s\S]*?<\/div>\s*<\/div>/),
  nav: borrow(index, /<nav class="nav">[\s\S]*?<\/nav>/),
  footer: borrow(index, /<footer class="footer">[\s\S]*?<\/footer>/),
};

// Every Markdown page, read once: a page's list of its neighbours needs their titles.
const rendered = new Map(pages.map((p) => [p, render(p, readFileSync(join(HERE, `${p}.md`), 'utf8'))]));
const short = (t) => t.replace(/ Protocol(?= \(|$)/, '').replace(/ \((?:rtl_433|FoxFW)[^)]*\)$/, '');
const titles = new Map([...rendered].map(([p, r]) => [p, short(r.title)]));
const html = new Map(); // file name -> page
const search = [];      // what site.js searches

const guideSide = (here) => sideList('Guides', GUIDES.filter((p) => rendered.has(p)).map((p) => [`${p}.html`, titles.get(p)]), here)
  + sideList('Reference', [['glossary.html', 'Glossary'], ['about.html', 'About this site'], ...(made.includes('changelog') ? [['changelog.html', 'Changelog']] : [])], here);
const indexSide = (here) => sideList('Browse', [['protocols.html', 'Protocols'], ['makes.html', 'Makes'], ['remotes.html', 'Remotes'], ['chips.html', 'Chips'],
  ['captures.html', 'Recordings'], ['fcc.html', 'FCC IDs'], ['coverage.html', 'Coverage'], ['guides.html', 'Guides'], ['glossary.html', 'Glossary']], here);

for (const [p, r] of rendered) {
  if (byPage.has(p)) {
    html.set(`${p}.html`, protocolPage(p, r, parts));
    continue;
  }
  const guide = GUIDES.includes(p), ref = REFERENCE.includes(p);
  const trail = [['./', 'KAT'], guide ? ['guides.html', 'Guides'] : ref ? ['about.html', 'Reference'] : ['protocols.html', 'Protocols']];
  const facts = r.meta.length ? `<aside class="infobox" aria-label="Facts"><div class="infobox-title">${esc(short(r.title))}</div><dl>
${r.meta.map((m) => `<div><dt>${esc(/^rust module/i.test(m.label) ? 'Source' : m.label)}</dt><dd>${codeBreaks(m.html)}</dd></div>`).join('\n')}
</dl></aside>\n` : '';
  html.set(`${p}.html`, wikiPage({
    file: `${p}.html`, title: r.title, description: r.description, trail,
    side: tocList(r.toc) + (guide || ref ? guideSide(`${p}.html`) : indexSide('')),
    lead: facts, body: r.body,
    foot: docFoot(p, guide || ref ? 'guides.html' : 'protocols.html', guide || ref ? 'All guides' : 'All protocols'),
  }, parts));
  search.push({ t: short(r.title), u: `${p}.html`, k: guide ? 'Guide' : ref ? 'Reference' : 'Page', d: r.description });
}

// Glossary terms are found by their own names.
if (rendered.has('glossary')) for (const t of rendered.get('glossary').toc.filter((t) => t.depth === 3)) search.push({ t: t.text, u: `glossary.html#${t.id}`, k: 'Glossary' });

// --- protocols.html
{
  const list = entries.slice().sort((a, b) => a.name.localeCompare(b.name, 'en', { sensitivity: 'base' }));
  const body = `<p class="lead">KAT has ${counts.decoders} decoders: ${counts.car} for car key fobs and alarm remotes and ${counts.gate} for gate, garage and home remotes. This table lists them under every name they show, ${list.length} rows in all, and each name opens the protocol's page.</p>
<p>The columns are the program's own facts, written out by KAT itself when the site is built: the line coding and modulation it expects, the short and long pulse its decoder is built around, the shortest frame it takes, and whether it can also send. A click on a column head sorts the table.</p>
${filterBox('Filter: a name, a make, “manchester”, “fm”, “encode”, “keeloq” …', 'protocol')}
${protocolTable(list)}
<p class="note">A decoder can read several kinds of frame and shows each under its own name (Kia V0's page is also Suzuki V0's and Honda V0's). ${list.length - list.filter((p) => p.registered).length} rows are protocols that are ported and kept in the source but not in the decoder list, as in the references they come from; MARELLI replays saved frames and decodes nothing. “Rec.” counts the files of the <a href="captures.html">recording library</a> KAT reads the protocol from.</p>
`;
  html.set('protocols.html', wikiPage({
    file: 'protocols.html', title: 'Protocols', wide: true,
    description: `The ${counts.decoders} remote-control protocols KAT decodes in one table, with their line coding, modulation, pulse widths, frame length and code type.`,
    trail: [['./', 'KAT']], side: indexSide('protocols.html'), body,
  }, parts));
  for (const p of list) search.push({ t: p.name, u: `${p.page}.html`, k: 'Protocol', d: `${p.make} · ${CODING[p.coding] ?? p.coding}, ${RF_SHORT[p.rf]} · ${p.encryption}`, x: p.aliases.join(' ') });
}

// --- makes.html and a page per make
{
  const coverage = (g) => {
    const rows = COV.by_make.filter((m) => g.catalogue.includes(m.make));
    if (!rows.length) return null;
    const sum = (k) => rows.reduce((n, m) => n + m[k], 0);
    return { model_years: sum('model_years'), read: sum('some_remote_read'), likely: sum('likely_read'), blind: sum('recorded_no_data'), unread: sum('recorded_unread'), filed: sum('filing_only'), nothing: sum('nothing') };
  };
  const cards = TAX.groups.map((g) => `<a class="make-card" href="${g.href}">
<h3>${esc(g.title)}</h3>
<p>${esc(clip(g.blurb, 150))}</p>
<div class="make-stats"><span><b>${g.entries.length}</b> ${g.entries.length === 1 ? 'protocol' : 'protocols'}</span><span><b>${g.captures.length}</b> ${g.captures.length === 1 ? 'recording' : 'recordings'}</span><span><b>${g.fcc.length}</b> FCC IDs</span></div>
</a>`).join('\n');
  const gate = entries.filter((p) => p.category === 'Gate');
  const brands = new Map();
  for (const p of gate) (brands.get(p.make) ?? brands.set(p.make, []).get(p.make)).push(p);
  const gateRows = [...brands].sort((a, b) => a[0].localeCompare(b[0], 'en', { sensitivity: 'base' })).map(([make, list]) =>
    `<tr data-q="${esc(`${make} ${list.map((p) => p.name).join(' ')}`.toLowerCase())}"><td>${esc(make)}</td><td>${list.map((p) => pageLink(p)).join(', ')}</td><td>${esc([...new Set(list.map((p) => p.encryption))].join('; '))}</td></tr>`).join('\n');
  html.set('makes.html', wikiPage({
    file: 'makes.html', title: 'Makes', wide: true,
    description: 'Remote-control protocols, recordings and FCC IDs by car make, and the brands of gate, garage and home remotes.',
    trail: [['./', 'KAT']], side: indexSide('makes.html'),
    body: `<p class="lead">Car makers buy their remotes from suppliers, so one make sends several unrelated signals over the years and one signal turns up on several makes. Each page here collects what belongs to a make: the protocols, the recordings in the library, and the FCC IDs of its remotes.</p>
<div class="make-grid">
${cards}
</div>
<h2 id="gate">Gate, garage and home remotes<a class="h-anchor" href="#gate" aria-label="Link to this section">#</a></h2>
<p>${plural(gate.length, 'protocol')} of ${brands.size} brands: gate and garage door openers, barriers, shutters and awnings, doorbells, alarm sensors and key finders. The recordings are in the <a href="captures.html#gates">GATES folder</a> of the library.</p>
${filterBox('Filter brands and protocols', 'gate')}
<div class="table-wrap"><table class="data sortable"><thead><tr><th>Brand</th><th>Protocols</th><th>Code</th></tr></thead><tbody>
${gateRows}
</tbody></table></div>
`,
  }, parts));

  for (const g of TAX.groups) {
    const cov = coverage(g);
    const read = g.captures.filter((c) => c.reads.length).length;
    const pct = (n) => `${Math.round((100 * n) / cov.model_years)} %`;
    const toc = [];
    const h2 = (id, text) => { toc.push({ depth: 2, id, text }); return `<h2 id="${id}">${text}<a class="h-anchor" href="#${id}" aria-label="Link to this section">#</a></h2>`; };
    const facts = `<aside class="infobox" aria-label="Facts about ${esc(g.short)}">
<div class="infobox-title">${esc(g.short)}</div>
<dl>
<div><dt>Protocols</dt><dd>${g.entries.length}${g.entries.some((p) => p.encoder) ? ` <span class="dim">(${g.entries.filter((p) => p.encoder).length} with an encoder)</span>` : ''}</dd></div>
<div><dt>Recordings</dt><dd>${g.captures.length ? `${g.captures.length} <span class="dim">(${read} read)</span>` : '<span class="dim">none</span>'}</dd></div>
<div><dt>FCC IDs</dt><dd>${g.fcc.length || '<span class="dim">none tied yet</span>'}</dd></div>
${cov ? `<div><dt title="Model-years of this make in North American parts catalogues whose remote KAT reads">Catalogue</dt><dd>${pct(cov.read)} read${cov.likely ? `, ${pct(cov.likely)} likely` : ''} <span class="dim">of ${cov.model_years} model-years</span></dd></div>` : ''}
${g.folders.length ? `<div><dt>Folders</dt><dd>${g.folders.map((f) => `<a class="path" href="${REPO}/tree/${CAPS.branch}/${repoPath(`${CAPS.folder}/${f}`)}"><code>${esc(f)}</code></a>`).join(' ')}</dd></div>` : ''}
</dl>
</aside>
`;
    const body = `<p class="lead">${esc(g.blurb)}</p>
${g.entries.length ? `${h2('protocols', 'Protocols')}
${protocolTable(g.entries, { make: false })}` : ''}
${g.captures.length ? `${h2('recordings', 'Recordings')}
<p>${plural(g.captures.length, 'file')} in the library, ${read} of which KAT reads. A file name opens the recording in the repository; the arrow downloads it.</p>
${captureTable(g.captures.slice().sort((a, b) => a.path.localeCompare(b.path)), { folder: g.folders.length > 1 || g.captures.some((c) => c.more) })}` : ''}
${remotes.some((r) => r.groups.includes(g) && r.photos.length) ? `${h2('remotes', 'Remotes')}
<p>Remotes of this make the site has pictures of: the remote, its circuit board, what is known of its chips. <a href="remotes.html">All remotes</a>.</p>
${remoteGrid(remotes.filter((r) => r.groups.includes(g) && r.photos.length).sort(byInterest).slice(0, 18))}` : ''}
${g.fcc.length ? `${h2('fcc-ids', 'FCC IDs')}
<p>FCC IDs that parts catalogues list for this make's cars, the family of remotes each belongs to, and whether KAT reads it. How the ties were made is on the <a href="fcc.html">FCC ID page</a>.</p>
${fccTable(g.fcc.slice().sort((a, b) => b.model_years - a.model_years))}` : ''}
${cov ? `${h2('coverage', 'Coverage')}
<p>North American parts catalogues list ${plural(cov.model_years, 'model-year')} of ${esc(g.short)} cars with a remote that has an FCC ID. By the best-known remote of each:</p>
<div class="table-wrap"><table class="data"><thead><tr><th>Read</th><th>Likely read</th><th>Recorded, no data</th><th>Recorded, not read</th><th>Filing read only</th><th>Nothing yet</th></tr></thead>
<tbody><tr><td>${cov.read} <span class="dim">${pct(cov.read)}</span></td><td>${cov.likely} <span class="dim">${pct(cov.likely)}</span></td><td>${cov.blind} <span class="dim">${pct(cov.blind)}</span></td><td>${cov.unread} <span class="dim">${pct(cov.unread)}</span></td><td>${cov.filed} <span class="dim">${pct(cov.filed)}</span></td><td>${cov.nothing} <span class="dim">${pct(cov.nothing)}</span></td></tr></tbody></table></div>
<p class="note">What the columns mean, and the limits of these figures, are on the <a href="coverage.html">coverage page</a>.</p>` : ''}
`;
    html.set(g.href, wikiPage({
      file: g.href, title: g.title, wide: true,
      description: clip(g.blurb),
      trail: [['./', 'KAT'], ['makes.html', 'Makes']],
      side: tocList(toc) + sideList('Makes', TAX.groups.map((x) => [x.href, x.short]), g.href),
      lead: facts, body,
    }, parts));
    search.push({ t: g.title, u: g.href, k: 'Make', d: `${plural(g.entries.length, 'protocol')}, ${plural(g.captures.length, 'recording')}`, x: g.tokens.join(' ') });
  }
}

// --- captures.html
{
  const sections = [...TAX.groups.filter((g) => g.captures.length).map((g) => ({ slug: g.slug, title: g.short, href: g.href, list: g.captures })),
    ...TAX.captures_only.map((g) => ({ slug: g.slug, title: g.title, href: null, list: CAPS.captures.filter((c) => g.folders.includes(c.folder)) }))].filter((s) => s.list.length);
  const filed = new Set(sections.flatMap((s) => s.list));
  const left = CAPS.captures.filter((c) => !filed.has(c));
  if (left.length) throw new Error(`recordings in no group: ${[...new Set(left.map((c) => c.folder))].join(', ')} (add the folder to data/taxonomy.json)`);
  const all = CAPS.captures;
  const read = all.filter((c) => c.reads.length).length;
  const bytes = all.reduce((n, c) => n + c.bytes, 0);
  const toc = [];
  const body = `<p class="lead">${plural(all.length, 'recording')} of real remotes, ${size(bytes)} in all, kept in the <a href="${REPO}/tree/${CAPS.branch}/${CAPS.folder}"><code>IMPORTS</code> folder</a> of the repository. KAT reads ${read} of them; what it reads from each is listed, and pinned by tests, so a change to a decoder that loses a frame is noticed.</p>
<p>The files are Flipper Zero <code>.sub</code> files: <strong>RAW</strong> files hold the pulse widths the radio saw, <strong>key files</strong> one decoded frame. A file name opens the recording in the repository and the arrow downloads it. <code>kat --decode file.sub</code> prints what KAT reads from one; <a href="recording.html">Recording a remote</a> says how such files are made and why some hold no data.</p>
${filterBox('Filter: a file name, a protocol, “unread”, “key”, “raw” …', 'recording')}
${sections.map((s) => {
    toc.push({ depth: 2, id: s.slug, text: `${s.title} (${s.list.length})` });
    return `<h2 id="${s.slug}">${s.href ? `<a class="plain" href="${s.href}">${esc(s.title)}</a>` : esc(s.title)} <span class="count">${s.list.length}</span><a class="h-anchor" href="#${s.slug}" aria-label="Link to this section">#</a></h2>
${captureTable(s.list.slice().sort((a, b) => a.path.localeCompare(b.path)), { folder: true, ids: true })}`;
  }).join('\n')}
<p class="note">Recordings KAT does not read are kept on purpose: they are what the next decoder is worked out from. Some hold no data at all, because the preset they were made with could not hold the remote's signal; <a href="unidentified_captures.html">Unidentified captures</a> and <a href="recording.html">Recording a remote</a> say which and why.</p>
`;
  html.set('captures.html', wikiPage({
    file: 'captures.html', title: 'Recordings', wide: true,
    description: `${all.length} Flipper .sub recordings of real car key fobs and gate remotes, with what KAT reads from each and a link to every file.`,
    trail: [['./', 'KAT']], side: tocList(toc) + indexSide('captures.html'), body,
  }, parts));
  for (const c of all) search.push({ t: c.name, u: `captures.html#${c.id}`, k: 'Recording', d: c.reads.length ? c.reads.map((r) => r.protocol).join(', ') : 'nothing read', x: c.folder });
}

// --- fcc.html
{
  const tied = FCC.ids.filter((i) => i.protocols.length);
  const fams = FCC.families.map((f) => {
    const ids = FCC.ids.filter((i) => i.family === f.key);
    const protos = f.protocols.map((n) => byName.get(n)).filter(Boolean);
    return `<tr id="fam-${f.key}" data-q="${esc(`${f.title} ${f.signal} ${f.protocols.join(' ')} ${ids.map((i) => i.id).join(' ')}`.toLowerCase())}">
<td>${esc(f.title)}</td>
<td>${esc(f.signal)}</td>
<td>${protos.length ? protos.map((p) => pageLink(p)).join(', ') : '<span class="dim">–</span>'}</td>
<td>${status(f.status)} ${f.status === 'read' || f.status === 'likely' ? evidence(f.evidence) : ''}</td>
<td class="ids">${ids.slice(0, 8).map((i) => `<a href="#fcc-${anchor(i.id)}"><code>${esc(i.id)}</code></a>`).join(' ')}${ids.length > 8 ? ` <span class="dim">+${ids.length - 8}</span>` : ''}</td></tr>`;
  }).join('\n');
  const body = `<p class="lead">Every remote sold in the United States has an FCC ID, and behind the ID is a public filing with the remote's test report: its frequency, its modulation, often the length and timing of its transmission, and photos of its circuit board. This page ties those filings to KAT's protocols.</p>
<p>Of ${COV.catalogue.fcc_ids_known.toLocaleString('en-US')} car-remote FCC IDs collected from the remotes' makers, ${FCC.ids.length} are listed here: the ${FCC.ids.filter((i) => i.filing_read).length} whose filing was read, the other remotes of their families, and those a recording in the library or a protocol's source points to. ${tied.length} are tied to a protocol. How sure a tie is, is said beside it:</p>
<ul>
<li>${evidence('recording')} KAT reads a recording of a remote with this FCC ID.</li>
<li>${evidence('recording (the only remote listed for that car)')} KAT reads a recording of a car for which parts catalogues list this remote alone. Nobody has confirmed the remote's FCC ID.</li>
<li>${evidence('reference')} The source of the protocol (ProtoPirate, a Flipper firmware, rtl_433) names the FCC ID.</li>
<li>${evidence('signal')} The filing's figures (band, keying, bit time, burst length) match the protocol. No recording confirms it: read it as a pointer.</li>
</ul>
<h2 id="families">Remote families<a class="h-anchor" href="#families" aria-label="Link to this section">#</a></h2>
<p>FCC IDs that send the same signal, grouped. “What it sends” is taken from the filings and, where there are recordings, measured from them.</p>
${filterBox('Filter families: a maker, a protocol, an FCC ID …', 'family')}
<div class="table-wrap"><table class="data sortable"><thead><tr><th>Family</th><th>What it sends</th><th>Protocol</th><th>Status</th><th>FCC IDs</th></tr></thead><tbody>
${fams}
</tbody></table></div>
<h2 id="ids">FCC IDs<a class="h-anchor" href="#ids" aria-label="Link to this section">#</a></h2>
<p>An ID opens the remote's page, with pictures of its board and what its filing says. “Cars” are the makes and years North American parts catalogues list the remote for (hover for the models).</p>
${filterBox('Filter: an FCC ID, a maker, a car, a protocol …', 'fcc')}
${fccTable(FCC.ids)}
<p class="note">Limits. The car lists come from parts catalogues and carry their mistakes; years between the first and last a catalogue lists are filled in. A filing describes one sample on a test bench and seldom says how the bits are coded. Ties by signal match are a judgement made from numbers, not a decode. European, Asian and Australian remotes have no FCC ID and are not here.</p>
`;
  html.set('fcc.html', wikiPage({
    file: 'fcc.html', title: 'FCC IDs', wide: true,
    description: `${FCC.ids.length} FCC IDs of car remotes: what their filings say they send, the families of remotes they form, and the KAT protocol that reads each.`,
    trail: [['./', 'KAT']],
    side: tocList([{ depth: 2, id: 'families', text: 'Remote families' }, { depth: 2, id: 'ids', text: 'FCC IDs' }]) + indexSide('fcc.html'), body,
  }, parts));
  for (const i of FCC.ids) search.push({ t: i.id, u: `remote-${anchor(i.id)}.html`, k: 'Remote', d: `${titleCase(i.maker)}${i.makes.length ? ` · ${i.makes.join(', ')} ${i.years}` : ''}`, x: i.protocols.join(' ') });
}

// --- remotes.html, a page per remote, chips.html, a page per chip
{
  const shown = remotes.filter((r) => r.photos.length).sort(byInterest);
  const rest = remotes.filter((r) => !r.photos.length);
  const own = remotes.filter((r) => !r.fcc);
  html.set('remotes.html', wikiPage({
    file: 'remotes.html', title: 'Remotes', wide: true,
    description: `${remotes.length} car remotes: pictures of the remote and its circuit board, the chips on it, what its FCC filing says about the signal, and the KAT protocol that reads it.`,
    trail: [['./', 'KAT']], side: tocList([{ depth: 2, id: 'pictured', text: 'With pictures' }, { depth: 2, id: 'others', text: 'Without pictures' }]) + indexSide('remotes.html'),
    body: `<p class="lead">The hardware behind the protocols: ${remotes.length} car remotes, ${shown.length} of them with pictures of the remote and its circuit board. Each has a page with what is known of it: the chips on the board, the frequency and modulation, what its FCC filing says about the signal, the cars it is sold for, and the KAT protocol that reads it.</p>
<p>${plural(own.length, 'remote')} here ${own.length === 1 ? 'was' : 'were'} photographed and recorded by ${own.length === 1 ? 'its owner' : 'their owners'}; the pictures of the others are from the internal photos of their FCC filings, which are public records. Remotes sold outside the United States have no FCC ID and are here only when an owner sent one in. The <a href="chips.html">chips</a> have pages of their own, and the <a href="fcc.html">FCC ID table</a> lists every remote by family.</p>
<h2 id="pictured">With pictures <span class="count">${shown.length}</span><a class="h-anchor" href="#pictured" aria-label="Link to this section">#</a></h2>
${filterBox('Filter: an FCC ID, a maker, a car, a protocol, a chip …', 'remote')}
${remoteGrid(shown)}
<h2 id="others">Without pictures <span class="count">${rest.length}</span><a class="h-anchor" href="#others" aria-label="Link to this section">#</a></h2>
<p>Remotes whose filing has no public internal photos, or was not read yet.</p>
${fccTable(rest.map((r) => r.fcc))}
`,
  }, parts));
  for (const r of remotes) html.set(`${r.slug}.html`, remotePage(r, parts));
  for (const r of remotes.filter((x) => !x.fcc)) search.push({ t: r.title, u: `${r.slug}.html`, k: 'Remote', d: [r.maker, r.entries.map((e) => e.name).join(', ')].filter(Boolean).join(' · ') });

  const rows = CHIPS.chips.map((c) => {
    const n = remotesOfChip(c.key).length;
    return `<tr data-q="${esc(`${c.name} ${c.maker} ${c.kind}`.toLowerCase())}"><td><a href="chip-${c.key}.html">${inline(c.name)}</a></td><td>${orDash(c.maker)}</td><td>${esc(c.kind)}</td>
<td>${c.datasheet ? `<a href="${esc(c.datasheet.url)}" rel="noopener">PDF</a>` : '<span class="dim">–</span>'}</td><td class="num" data-v="${n}">${n || ''}</td></tr>`;
  }).join('\n');
  html.set('chips.html', wikiPage({
    file: 'chips.html', title: 'Chips', wide: true,
    description: `The chips in car remotes: ${CHIPS.chips.length} transmitters, key controllers, encoders and transponders seen on a remote's board or named in its FCC filing, with data sheets where they are public.`,
    trail: [['./', 'KAT'], ['remotes.html', 'Remotes']], side: indexSide('chips.html'),
    body: `<p class="lead">A remote is a few parts: a controller that makes the code, a transmitter (often in the same chip), a crystal or SAW resonator that sets the frequency, and in a key a transponder for the immobiliser. This table lists the chips seen on the boards of the <a href="remotes.html">remotes</a> or named in their FCC filings.</p>
<p>Makers of car access chips seldom publish data sheets, and filings keep schematics confidential, so the list is short and a chip is often known by the marking on its package alone. A data sheet is linked where the maker or a distributor offers one.</p>
${filterBox('Filter chips', 'chip')}
<div class="table-wrap"><table class="data sortable"><thead><tr><th>Chip</th><th>Maker</th><th>Kind</th><th>Data sheet</th><th class="num">Remotes</th></tr></thead><tbody>
${rows}
</tbody></table></div>
`,
  }, parts));
  for (const c of CHIPS.chips) {
    html.set(`chip-${c.key}.html`, chipPage(c, parts));
    search.push({ t: c.name.replace(/`/g, ''), u: `chip-${c.key}.html`, k: 'Chip', d: [c.maker, c.kind].filter(Boolean).join(' · ') });
  }
}

// --- coverage.html
{
  const t = COV.totals;
  const pct = (n, of = t.model_years) => `${Math.round((100 * n) / of)} %`;
  const head = [['KAT reads the remote', t.some_remote_read, 'A recording of the car is read, or the protocol’s source names the remote'],
    ['KAT probably reads it', t.likely_read, 'The filing’s figures match a protocol; no recording confirms it'],
    ['Recorded, but the recordings hold no data', t.recorded_no_data, 'The preset the recording was made with could not hold the signal'],
    ['Recorded, nothing read', t.recorded_unread, 'A recording exists and no decoder reads it'],
    ['FCC filing read, no usable recording', t.filing_only, 'The filing of the remote, or of another remote of its family, was read'],
    ['Nothing yet', t.nothing, '']];
  const makes = COV.by_make.filter((m) => m.model_years >= 8).map((m) => {
    const g = TAX.groups.find((x) => x.catalogue.includes(m.make));
    return `<tr><td>${g ? `<a href="${g.href}">${esc(m.make)}</a>` : esc(m.make)}</td><td class="num">${m.model_years}</td>
<td class="num" data-v="${m.some_remote_read / m.model_years}">${pct(m.some_remote_read, m.model_years)}</td><td class="num" data-v="${m.likely_read / m.model_years}">${pct(m.likely_read, m.model_years)}</td>
<td class="num" data-v="${m.recorded_no_data / m.model_years}">${pct(m.recorded_no_data, m.model_years)}</td><td class="num" data-v="${m.recorded_unread / m.model_years}">${pct(m.recorded_unread, m.model_years)}</td>
<td class="num" data-v="${m.filing_only / m.model_years}">${pct(m.filing_only, m.model_years)}</td><td class="num" data-v="${m.nothing / m.model_years}">${pct(m.nothing, m.model_years)}</td></tr>`;
  }).join('\n');
  const gaps = COV.gaps.slice(0, 40).map((g) => `<tr data-q="${esc(`${g.id} ${g.maker} ${g.makes.join(' ')} ${g.models}`.toLowerCase())}">${fccCell(g.id)}
<td>${orDash(titleCase(g.maker))}</td><td>${orDash(g.frequency)}</td>
<td title="${esc(g.models)}">${esc(g.makes.join(', '))} <span class="dim">${esc(g.years)}</span></td>
<td class="num">${g.model_years}</td><td>${status(g.status, g)}</td></tr>`).join('\n');
  const body = `<p class="lead">How much of what is on the road does KAT read? This page counts it for North America, where every remote has an FCC ID and parts catalogues say which cars each remote fits: ${COV.catalogue.fcc_ids_with_cars} remotes, ${t.model_years.toLocaleString('en-US')} model-years (a make, a model and a year).</p>
<h2 id="headline">By model-year<a class="h-anchor" href="#headline" aria-label="Link to this section">#</a></h2>
<p>Each model-year counts once, by the best-known of the remotes listed for it.</p>
<div class="table-wrap"><table class="data"><thead><tr><th>Status</th><th class="num">Model-years</th><th class="num">Share</th><th></th></tr></thead><tbody>
${head.map(([label, n, tip]) => `<tr><td${tip ? ` title="${esc(tip)}"` : ''}>${label}</td><td class="num">${n.toLocaleString('en-US')}</td><td class="num">${pct(n)}</td><td class="bar"><span style="width:${(100 * n / t.model_years).toFixed(1)}%"></span></td></tr>`).join('\n')}
</tbody></table></div>
<h2 id="makes">By make<a class="h-anchor" href="#makes" aria-label="Link to this section">#</a></h2>
<p>Makes with at least eight model-years in the catalogues.</p>
<div class="table-wrap"><table class="data sortable"><thead><tr><th>Make</th><th class="num">Model-years</th><th class="num">Read</th><th class="num">Likely</th><th class="num">No data</th><th class="num">Not read</th><th class="num">Filing only</th><th class="num">Nothing</th></tr></thead><tbody>
${makes}
</tbody></table></div>
<h2 id="wanted">Wanted: recordings of these remotes<a class="h-anchor" href="#wanted" aria-label="Link to this section">#</a></h2>
<p>The remotes KAT does not read yet, by the number of model-years each would add. A recording of any of them, made as <a href="recording.html">Recording a remote</a> describes, is the first step to a decoder; the <a href="${REPO}/issues">issue tracker</a> takes them.</p>
${filterBox('Filter: an FCC ID, a maker, a car …', 'wanted')}
<div class="table-wrap"><table class="data sortable"><thead><tr><th>FCC ID</th><th>Maker</th><th>Band</th><th>Cars</th><th class="num">Model-years</th><th>Status</th></tr></thead><tbody>
${gaps}
</tbody></table></div>
<h2 id="limits">How to read this<a class="h-anchor" href="#limits" aria-label="Link to this section">#</a></h2>
<ul>
<li>The set of cars is what North American parts catalogues list for each FCC ID. It is incomplete and has catalogue noise; years between the first and last a catalogue lists for a remote are filled in.</li>
<li>Hyundai Mobis keys have no car lists in that source, so Kia and Hyundai are under-counted.</li>
<li>European, Asian and Australian cars are not in this count at all, though many of KAT's protocols and recordings are of them.</li>
<li>“Read” means a recording of that car is read, or the protocol's source names the remote. “Likely” means the FCC filing's figures match a protocol and no recording confirms it.</li>
</ul>
`;
  html.set('coverage.html', wikiPage({
    file: 'coverage.html', title: 'Coverage', wide: true,
    description: `Of ${t.model_years.toLocaleString('en-US')} North American model-years with a remote that has an FCC ID, KAT reads ${pct(t.some_remote_read)}: the count by make, and the remotes still wanted.`,
    trail: [['./', 'KAT']],
    side: tocList([{ depth: 2, id: 'headline', text: 'By model-year' }, { depth: 2, id: 'makes', text: 'By make' }, { depth: 2, id: 'wanted', text: 'Wanted recordings' }, { depth: 2, id: 'limits', text: 'How to read this' }]) + indexSide('coverage.html'), body,
  }, parts));
}

// --- guides.html
{
  const item = (p) => (rendered.has(p) ? `<a class="guide-card" href="${p}.html"><h3>${esc(titles.get(p))}</h3><p>${esc(clip(rendered.get(p).description, 190))}</p></a>` : '');
  html.set('guides.html', wikiPage({
    file: 'guides.html', title: 'Guides', wide: true,
    description: 'How KAT receives, decodes and sends, how recordings are made, and the reference pages of the site.',
    trail: [['./', 'KAT']], side: indexSide('guides.html'),
    body: `<p class="lead">How KAT works and how to use it: the receive chain and the radios, sending, the tools for a signal KAT does not know, and how the recordings in the library are made.</p>
<div class="guide-grid">
${GUIDES.map(item).join('\n')}
</div>
<h2 id="reference">Reference<a class="h-anchor" href="#reference" aria-label="Link to this section">#</a></h2>
<div class="guide-grid">
${REFERENCE.map(item).join('\n')}
${made.includes('changelog') ? '<a class="guide-card" href="changelog.html"><h3>Changelog</h3><p>What changed in each release of KAT, newest first.</p></a>' : ''}
</div>
`,
  }, parts));
}

// --- changelog.html, when the repository is beside this folder
if (made.includes('changelog')) {
  const r = render('changelog', readFileSync(changelogSrc, 'utf8').replace(/\]\(docs\/([\w-]+)\.md/g, ']($1.md').replace(/\]\(README\.md/g, `](${REPO}/blob/main/README.md`));
  const toc = r.toc.filter((t) => t.depth === 2).slice(0, 30);
  html.set('changelog.html', wikiPage({
    file: 'changelog.html', title: 'Changelog',
    description: 'What changed in each release of KAT, the Keyfob Analysis Toolkit, newest first.',
    // A reference page like the glossary (the side list, the Guides page and the search file it there).
    trail: [['./', 'KAT'], ['about.html', 'Reference']], side: tocList(toc) + guideSide('changelog.html'), body: r.body,
    foot: `      <div class="doc-foot"><a href="guides.html">← All guides</a><span>This page is <a href="${REPO}/blob/main/CHANGELOG.md">CHANGELOG.md</a> in the KAT repository.</span></div>`,
  }, parts));
  search.push({ t: 'Changelog', u: 'changelog.html', k: 'Reference', d: 'What changed in each release' });
}

for (const [f, t, d] of [['protocols.html', 'Protocols', 'Every protocol in one table'], ['makes.html', 'Makes', 'By car make and gate brand'], ['captures.html', 'Recordings', 'The library of .sub recordings'],
  ['fcc.html', 'FCC IDs', 'Filings tied to protocols'], ['coverage.html', 'Coverage', 'What KAT reads, and what is wanted'], ['guides.html', 'Guides', 'How KAT works'],
  ['remotes.html', 'Remotes', 'Pictures, boards and filings of the remotes'], ['chips.html', 'Chips', 'The chips in the remotes']]) search.push({ t, u: f, k: 'Index', d });

// ---------------------------------------------------------------------------
// Write

writeFileSync(join(OUT, 'search.json'), JSON.stringify(search));
const hashes = new Map();
const hashFile = (rel, dir = HERE) => hashes.set(rel, createHash('sha256').update(readFileSync(join(dir, rel))).digest('hex').slice(0, 10));
for (const f of ['style.css', 'app.js', 'site.js', 'logo.svg', 'og.png']) if (existsSync(join(HERE, f))) hashFile(f);
hashFile('search.json', OUT);
const walk = (rel) => readdirSync(join(HERE, rel), { withFileTypes: true }).flatMap((e) => (e.isDirectory() ? walk(`${rel}/${e.name}`) : [`${rel}/${e.name}`]));
if (existsSync(join(HERE, 'img'))) for (const f of walk('img')) hashFile(f);

const landing = index.replace(/\{\{(\w+)\}\}/g, (m, k) => (k in counts ? counts[k].toLocaleString('en-US') : m));
if (/\{\{\w+\}\}/.test(landing)) throw new Error(`index.html: unknown placeholder ${/\{\{\w+\}\}/.exec(landing)[0]}`);
writeFileSync(join(OUT, 'index.html'), versioned(landing, hashes));
for (const [f, page] of html) writeFileSync(join(OUT, f), versioned(page, hashes));

// The server answers a missing address with this page (Caddy's handle_errors).
// <base> makes its links work at any depth, /a/b included.
const notFound = shell({
  file: '404.html', title: 'No page here', description: 'That address is not part of the KAT site.',
  main: `<main class="doc"><div class="container doc-inner"><article class="prose">
<h1>No page here</h1>
<p>That address isn't part of KAT's site. The <a href="protocols.html">protocol table</a> links every
protocol's page, the <a href="captures.html">recordings</a> and <a href="fcc.html">FCC IDs</a> have tables of their own, and
<a href="guides.html">the guides</a> cover the rest. The search box above finds a page by name.</p>
</article></div></main>`,
}, parts)
  .replace('<head>\n', '<head>\n<base href="/">\n<meta name="robots" content="noindex">\n')
  .replace(/<link rel="canonical"[^>]*>\n/, '');
writeFileSync(join(OUT, '404.html'), versioned(notFound, hashes));

// For search engines, and for checking which commit is live.
const urls = ['', ...[...html.keys()].sort()];
writeFileSync(join(OUT, 'sitemap.xml'), `<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
${urls.map((u) => `  <url><loc>${ORIGIN}/${u}</loc></url>`).join('\n')}
</urlset>
`);
writeFileSync(join(OUT, 'robots.txt'), `User-agent: *\nAllow: /\n\nSitemap: ${ORIGIN}/sitemap.xml\n`);
let commit = null;
try { commit = execFileSync('git', ['rev-parse', 'HEAD'], { cwd: HERE, encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore'] }).trim(); } catch { /* not a checkout */ }
writeFileSync(join(OUT, 'build.json'), JSON.stringify({ commit, built: new Date().toISOString(), pages: html.size, kat: PROTO.version }) + '\n');

console.log(`dist/: index.html and ${html.size} pages (${pages.length} from Markdown), ${search.length} search entries`);
