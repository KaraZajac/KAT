/* KAT's reference pages: the search box, the menu on a phone, table filters
   and sorting, and the list of a page's sections. No network but search.json
   from this site, no dependencies. Every page reads fine without it: the
   tables are whole in the HTML and every result of a search is also a link
   in one of them. */
(function () {
  'use strict';
  const norm = (s) => String(s).toLowerCase().normalize('NFD').replace(/[̀-ͯ]/g, '').replace(/[^a-z0-9.+_]+/g, ' ').trim();
  const here = document.currentScript;

  /* ---- the menu on a narrow screen ---- */
  const toggle = document.querySelector('.nav-toggle');
  const links = document.getElementById('nav-links');
  if (toggle && links) {
    toggle.addEventListener('click', () => {
      const open = links.classList.toggle('open');
      toggle.setAttribute('aria-expanded', String(open));
    });
    links.addEventListener('click', (e) => { if (e.target.closest('a')) { links.classList.remove('open'); toggle.setAttribute('aria-expanded', 'false'); } });
  }

  /* ---- search ---- */
  const box = document.getElementById('site-search');
  const list = document.getElementById('search-results');
  if (box && list && here && here.dataset.search) {
    let index = null, loading = null, shown = [], sel = -1;
    const WEIGHT = { Protocol: 9, Make: 7, Index: 6, 'FCC ID': 4, Guide: 5, Reference: 5, Glossary: 4, Recording: 0, Page: 3 };
    const load = () => loading || (loading = fetch(here.dataset.search).then((r) => r.json()).then((rows) => {
      index = rows.map((r) => ({ r, t: norm(r.t), all: norm(`${r.t} ${r.x || ''} ${r.d || ''} ${r.k}`) }));
    }).catch(() => { index = []; }));
    const score = (e, words, q) => {
      let s = 0;
      for (const w of words) {
        if (!e.all.includes(w)) return -1;
        if (e.t === w) s += 60;
        else if (e.t.startsWith(w)) s += 40;
        else if ((' ' + e.t).includes(' ' + w)) s += 28;
        else if (e.t.includes(w)) s += 16;
        else s += 4;
      }
      if (e.t === q) s += 80; else if (e.t.startsWith(q)) s += 30;
      return s + (WEIGHT[e.r.k] || 0) - Math.min(8, e.t.length / 8);
    };
    const close = () => { list.hidden = true; box.setAttribute('aria-expanded', 'false'); sel = -1; };
    const mark = () => list.querySelectorAll('a').forEach((a, i) => a.classList.toggle('sel', i === sel));
    const show = () => {
      const q = norm(box.value);
      const words = q.split(' ').filter(Boolean);
      if (!words.length || !index) { close(); return; }
      shown = index.map((e) => [score(e, words, q), e]).filter((x) => x[0] >= 0).sort((a, b) => b[0] - a[0]).slice(0, 12).map((x) => x[1].r);
      list.textContent = '';
      if (!shown.length) {
        const p = document.createElement('p');
        p.className = 'none';
        p.textContent = 'Nothing by that name. The protocol table and the recordings can be filtered by any word.';
        list.appendChild(p);
      }
      for (const r of shown) {
        const a = document.createElement('a');
        a.href = r.u;
        const k = document.createElement('span'); k.className = 'kind'; k.textContent = r.k;
        const t = document.createElement('span'); t.className = 'title'; t.textContent = r.t;
        a.append(k, t);
        if (r.d) { const d = document.createElement('span'); d.className = 'desc'; d.textContent = r.d; a.appendChild(d); }
        list.appendChild(a);
      }
      sel = -1;
      list.hidden = false;
      box.setAttribute('aria-expanded', 'true');
    };
    box.addEventListener('focus', () => { load().then(show); });
    box.addEventListener('input', () => { load().then(show); });
    box.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') { close(); box.blur(); return; }
      if (list.hidden || !shown.length) return;
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        e.preventDefault();
        sel = (sel + (e.key === 'ArrowDown' ? 1 : -1) + shown.length) % shown.length;
        mark();
      } else if (e.key === 'Enter') {
        e.preventDefault();
        location.href = shown[Math.max(0, sel)].u;
      }
    });
    document.addEventListener('click', (e) => { if (!e.target.closest('.nav-search')) close(); });
    document.addEventListener('keydown', (e) => {
      if (e.key === '/' && !/^(INPUT|TEXTAREA|SELECT)$/.test(document.activeElement.tagName) && !e.ctrlKey && !e.metaKey && !e.altKey) {
        e.preventDefault();
        box.focus();
      }
    });
  }

  /* ---- a filter box hides the rows below it that lack one of its words ---- */
  document.querySelectorAll('.table-find').forEach((find) => {
    const input = find.querySelector('input');
    const out = find.querySelector('output');
    // Its rows: every table up to the next filter box.
    const blocks = [];
    for (let el = find.nextElementSibling; el && !el.classList.contains('table-find'); el = el.nextElementSibling) blocks.push(el);
    const rows = blocks.flatMap((el) => Array.from(el.querySelectorAll('tbody tr[data-q]')));
    if (!rows.length) { find.hidden = true; return; }
    const apply = () => {
      const words = norm(input.value).split(' ').filter(Boolean);
      let n = 0;
      rows.forEach((tr) => {
        const hit = words.every((w) => tr.dataset.q.includes(w));
        tr.hidden = !hit;
        if (hit) n++;
      });
      // A table left empty goes, with the heading above it; a folded part opens when it holds a hit.
      blocks.forEach((el) => {
        const own = el.querySelectorAll('tbody tr[data-q]');
        if (!own.length) return;
        const any = Array.from(own).some((tr) => !tr.hidden);
        if (el.tagName === 'DETAILS') { if (words.length && any) el.open = true; }
        else if (el.classList.contains('table-wrap')) {
          el.hidden = !any;
          const head = el.previousElementSibling;
          if (head && /^H[23]$/.test(head.tagName)) head.hidden = !any;
        }
      });
      out.textContent = words.length ? `${n} of ${rows.length}` : `${rows.length}`;
    };
    input.addEventListener('input', apply);
    apply();
  });

  /* ---- a click on a column head sorts the table ---- */
  document.querySelectorAll('table.sortable').forEach((table) => {
    const heads = table.querySelectorAll('thead th');
    heads.forEach((th, col) => {
      th.tabIndex = 0;
      th.setAttribute('role', 'button');
      const sort = () => {
        const dir = th.getAttribute('aria-sort') === 'ascending' ? -1 : 1;
        heads.forEach((h) => h.removeAttribute('aria-sort'));
        th.setAttribute('aria-sort', dir === 1 ? 'ascending' : 'descending');
        const body = table.tBodies[0];
        const key = (tr) => {
          const td = tr.cells[col];
          if (!td) return '';
          if (td.dataset.v !== undefined) return parseFloat(td.dataset.v) || 0;
          return td.textContent.trim().toLowerCase();
        };
        const rows = Array.from(body.rows).map((tr, i) => [key(tr), i, tr]);
        rows.sort((a, b) => {
          const x = a[0], y = b[0];
          const c = typeof x === 'number' && typeof y === 'number' ? x - y : String(x).localeCompare(String(y), 'en', { numeric: true });
          return c ? c * dir : a[1] - b[1];
        });
        rows.forEach((r) => body.appendChild(r[2]));
      };
      th.addEventListener('click', sort);
      th.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); sort(); } });
    });
  });

  /* ---- an address that points into a folded part opens it ---- */
  const reveal = () => {
    if (location.hash.length < 2) return;
    let el;
    try { el = document.getElementById(decodeURIComponent(location.hash.slice(1))); } catch (e) { return; }
    if (!el) return;
    const d = el.closest('details');
    if (d && !d.open) { d.open = true; el.scrollIntoView(); }
    if (el.tagName === 'TR') { el.classList.add('target'); }
  };
  window.addEventListener('hashchange', reveal);
  reveal();

  /* ---- the section being read is marked in the list beside the page ---- */
  const toc = document.querySelector('.side-toc');
  if (toc && 'IntersectionObserver' in window) {
    const map = new Map();
    toc.querySelectorAll('a[href^="#"]').forEach((a) => {
      const h = document.getElementById(decodeURIComponent(a.getAttribute('href').slice(1)));
      if (h) map.set(h, a);
    });
    const seen = new Set();
    const io = new IntersectionObserver((entries) => {
      entries.forEach((e) => { if (e.isIntersecting) seen.add(e.target); else seen.delete(e.target); });
      const first = Array.from(map.keys()).find((h) => seen.has(h));
      if (first) {
        map.forEach((a) => a.classList.remove('now'));
        map.get(first).classList.add('now');
      }
    }, { rootMargin: '-80px 0px -65% 0px' });
    map.forEach((a, h) => io.observe(h));
  }
})();
