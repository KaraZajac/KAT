// build.mjs: builds kat.karazajac.io from this folder.
//
// index.html is written by hand. Every other page is one of the Markdown files
// here, rendered with marked into the same design and published at the address
// Jekyll gave it (radio.md becomes radio.html), so links into the docs keep
// working. Heading anchors follow the rule GitHub Pages used (kramdown's GFM
// ids). The docs pages take their header and footer from index.html, so those
// are edited in one place.
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
// Pages that explain how to use KAT rather than one protocol. Anything not
// listed here is filed under Protocols.
const GUIDES = new Set(['radio', 'flipper_radio', 'transmit', 'session', 'analyze', 'timing_analysis', 'compare',
  'add_manually', 'flipper_key_files', 'unidentified_captures']);

const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');

// ---------------------------------------------------------------------------
// Markdown to HTML

/** Jekyll front matter (`---` … `---`) at the top of a file, removed. */
const stripFrontMatter = (src) => src.replace(/^---\r?\n[\s\S]*?\r?\n---\r?\n/, '');

/** The plain text of inline tokens: what a reader sees, without markup. */
function plain(tokens = []) {
  return tokens.map((t) => (t.tokens ? plain(t.tokens) : t.type === 'br' ? ' ' : t.text ?? '')).join('');
}
const unentity = (s) => s.replace(/&(amp|lt|gt|quot|#39);/g, (_, e) => ({ amp: '&', lt: '<', gt: '>', quot: '"', '#39': "'" })[e]);

// kramdown's GFM header ids: lowercase the heading's source text, drop all but
// letters, digits, underscores, hyphens and spaces, make each space a hyphen,
// and number repeats -1, -2 …
function slugger() {
  const seen = new Map();
  return (raw) => {
    const base = raw.toLowerCase().replace(/[^\p{L}\p{M}\p{N}\p{Pc}\- \t]/gu, '').replace(/[ \t]/g, '-');
    const n = seen.has(base) ? seen.get(base) + 1 : 0;
    seen.set(base, n);
    return n ? `${base}-${n}` : base;
  };
}

/** A link to a sibling page's .md, pointed at its .html. Other links stay. */
function pageHref(href) {
  const m = /^([A-Za-z0-9_.-]+)\.md(#.*)?$/.exec(href ?? '');
  return m ? `${m[1]}.html${m[2] ?? ''}` : href;
}

/** Cut a description to about `max` characters, at a word. */
function clip(s, max = 200) {
  if (s.length <= max) return s;
  return s.slice(0, s.lastIndexOf(' ', max)).replace(/[\s,;:.(–—-]+$/, '') + '…';
}

function render(name, src) {
  const slug = slugger();
  let title = null, section = '', overview = null, first = null;
  const marked = new Marked({
    gfm: true,
    renderer: {
      heading({ tokens, depth, text }) {
        const id = slug(text);
        const words = unentity(plain(tokens)).trim();
        if (depth === 1 && title == null) title = words;
        section = words.toLowerCase();
        return `<h${depth} id="${esc(id)}">${this.parser.parseInline(tokens)}<a class="h-anchor" href="#${esc(id)}" aria-label="Link to this section">#</a></h${depth}>\n`;
      },
      paragraph({ tokens }) {
        // The page's description: the first paragraph under "Overview", or
        // else the first one that isn't a "Rust module:" style label line.
        const text = unentity(plain(tokens)).replace(/\s+/g, ' ').trim();
        const label = tokens[0]?.type === 'strong' && plain(tokens[0].tokens).trim().endsWith(':');
        if (overview == null && section === 'overview' && text) overview = text;
        if (first == null && !label && text) first = text;
        return false; // marked's own <p>
      },
      // The docs carry no HTML on purpose, so anything that looks like a tag is
      // text: radio.md's "Receiver stopped: <reason>" must not vanish.
      html({ text }) {
        return esc(text);
      },
      link({ href, title: t, tokens }) {
        return `<a href="${esc(pageHref(href))}"${t ? ` title="${esc(t)}"` : ''}>${this.parser.parseInline(tokens)}</a>`;
      },
    },
  });
  // Wide tables scroll sideways inside their own box on a phone.
  const body = marked.parse(stripFrontMatter(src))
    .replace(/<table>/g, '<div class="table-wrap"><table>')
    .replace(/<\/table>/g, '</table></div>');
  return { title: title ?? name, description: clip(overview ?? first ?? ''), body };
}

// ---------------------------------------------------------------------------
// Pages

/** ?v=<content hash> on every local asset a page names, so a changed file is
 *  never served from a stale cache under its old name. */
function versioned(html, hashes) {
  return html.replace(/(href|src|content)="((?:https:\/\/kat\.karazajac\.io\/)?)(style\.css|app\.js|logo\.svg|og\.png|img\/[^"?#]+)"/g,
    (m, attr, origin, path) => (hashes.has(path) ? `${attr}="${origin}${path}?v=${hashes.get(path)}"` : m));
}

/** index.html's own header or footer, with its #section links pointed back at
 *  the landing page. */
function borrow(index, re) {
  const m = re.exec(index);
  if (!m) throw new Error(`index.html: no match for ${re}`);
  return m[0].replace(/href="#"/g, 'href="./"').replace(/href="#([^"]+)"/g, 'href="./#$1"');
}

function docPage(p, { title, description, body }, parts) {
  const guide = GUIDES.has(p);
  const back = guide ? ['docs', 'Docs'] : ['protocols', 'Protocols'];
  const url = `${ORIGIN}/${p}.html`;
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
<meta property="og:type" content="article">
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
</head>
<body>

${parts.mesh}

${parts.nav}

<main class="doc">
  <div class="container doc-inner">
    <nav class="crumbs" aria-label="Breadcrumb"><a href="./">KAT</a><span aria-hidden="true">/</span><a href="./#${back[0]}">${back[1]}</a></nav>
    <article class="prose">
${body}    </article>
    <div class="doc-foot">
      <a href="./#${back[0]}">← All ${back[1].toLowerCase()}</a>
      <span>This page is <a href="${REPO}/blob/main/docs/${p}.md">docs/${p}.md</a> in the KAT repository.</span>
    </div>
  </div>
</main>

${parts.footer}

</body>
</html>
`;
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

rmSync(OUT, { recursive: true, force: true });
mkdirSync(OUT, { recursive: true });

if (redirect) {
  writeFileSync(join(OUT, 'index.html'), redirectPage(`${target}/`));
  for (const p of pages) writeFileSync(join(OUT, `${p}.html`), redirectPage(`${target}/${p}.html`));
  // Any other old address: GitHub Pages serves 404.html, which forwards the
  // same path, minus the /KAT project prefix, to the new host.
  writeFileSync(join(OUT, '404.html'), redirectPage(`${target}/`).replace(
    /<script>.*<\/script>/,
    `<script>location.replace(${JSON.stringify(target)} + location.pathname.replace(/^\\/KAT(?=\\/|$)/, '') + location.search + location.hash);</script>`));
  console.log(`dist/: ${pages.length + 2} pages forwarding to ${target}`);
  process.exit(0);
}

// Static files go across as they are.
for (const f of files) if (!f.endsWith('.md')) cpSync(join(HERE, f), join(OUT, f), { recursive: true });

const hashes = new Map();
const hashFile = (rel) => hashes.set(rel, createHash('sha256').update(readFileSync(join(HERE, rel))).digest('hex').slice(0, 10));
for (const f of ['style.css', 'app.js', 'logo.svg', 'og.png']) if (existsSync(join(HERE, f))) hashFile(f);
if (existsSync(join(HERE, 'img'))) for (const f of readdirSync(join(HERE, 'img'))) hashFile(`img/${f}`);

const index = readFileSync(join(HERE, 'index.html'), 'utf8');
const parts = {
  mesh: borrow(index, /<div class="bg-mesh"[\s\S]*?<\/div>\s*<\/div>/),
  nav: borrow(index, /<nav class="nav">[\s\S]*?<\/nav>/),
  footer: borrow(index, /<footer class="footer">[\s\S]*?<\/footer>/),
};
writeFileSync(join(OUT, 'index.html'), versioned(index, hashes));

for (const p of pages) {
  const html = docPage(p, render(p, readFileSync(join(HERE, `${p}.md`), 'utf8')), parts);
  writeFileSync(join(OUT, `${p}.html`), versioned(html, hashes));
}

// The server answers a missing address with this page (Caddy's handle_errors).
// <base> makes its links work at any depth, /a/b included.
const notFound = docPage('404', {
  title: 'No page here',
  description: 'That address is not part of the KAT site.',
  body: `<h1>No page here</h1>
<p>That address isn't part of KAT's site. The <a href="./#protocols">protocol list</a> links every
protocol's page, and <a href="./#docs">the docs</a> cover the rest.</p>
`,
}, parts)
  .replace('<head>\n', '<head>\n<base href="/">\n<meta name="robots" content="noindex">\n')
  .replace(/<link rel="canonical"[^>]*>\n/, '')
  .replace(/ *<nav class="crumbs"[\s\S]*?<\/nav>\n/, '')
  .replace(/\n    <div class="doc-foot">[\s\S]*?\n    <\/div>\n/, '\n');
writeFileSync(join(OUT, '404.html'), versioned(notFound, hashes));

// For search engines, and for checking which commit is live.
const urls = ['', ...pages.map((p) => `${p}.html`)];
writeFileSync(join(OUT, 'sitemap.xml'), `<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
${urls.map((u) => `  <url><loc>${ORIGIN}/${u}</loc></url>`).join('\n')}
</urlset>
`);
writeFileSync(join(OUT, 'robots.txt'), `User-agent: *\nAllow: /\n\nSitemap: ${ORIGIN}/sitemap.xml\n`);
let commit = null;
try { commit = execFileSync('git', ['rev-parse', 'HEAD'], { cwd: HERE, encoding: 'utf8' }).trim(); } catch { /* not a checkout */ }
writeFileSync(join(OUT, 'build.json'), JSON.stringify({ commit, built: new Date().toISOString(), pages: pages.length }) + '\n');

console.log(`dist/: index.html and ${pages.length} pages from Markdown`);
