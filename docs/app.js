/* KAT landing page: count-up stats, scroll reveal, copy buttons and the
   protocol filter. No network, no dependencies. The page reads fine without
   it: every section is in the HTML, and .reveal only hides things once this
   script has marked the page as running it. */
(function () {
  'use strict';
  const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  /* count-up numbers when the stats row scrolls into view */
  function countUp(el) {
    const target = parseInt(el.dataset.target, 10);
    if (isNaN(target)) return;
    if (!target || reduceMotion) { el.textContent = String(target); return; }
    const dur = 1100, start = performance.now();
    (function tick(now) {
      const p = Math.min(1, (now - start) / dur);
      el.textContent = String(Math.round(target * (1 - Math.pow(1 - p, 3))));
      if (p < 1) requestAnimationFrame(tick);
    })(performance.now());
  }

  const statsRow = document.getElementById('stats-row');
  if (statsRow) {
    const nums = statsRow.querySelectorAll('.num[data-target]');
    if ('IntersectionObserver' in window && !reduceMotion) {
      nums.forEach((el) => { el.textContent = '0'; });
      const io = new IntersectionObserver((entries, obs) => {
        entries.forEach((e) => {
          if (e.isIntersecting) { nums.forEach(countUp); obs.disconnect(); }
        });
      }, { threshold: 0.4 });
      io.observe(statsRow);
    }
  }

  /* scroll-triggered reveal */
  const reveals = document.querySelectorAll('.reveal');
  if ('IntersectionObserver' in window && !reduceMotion) {
    // What is already on screen stays put; only what is below it fades in.
    reveals.forEach((el) => { if (el.getBoundingClientRect().top < window.innerHeight) el.classList.add('in'); });
    document.documentElement.classList.add('js-reveal');
    const ro = new IntersectionObserver((entries, obs) => {
      entries.forEach((e) => {
        if (e.isIntersecting) { e.target.classList.add('in'); obs.unobserve(e.target); }
      });
    }, { threshold: 0.08, rootMargin: '0px 0px -40px 0px' });
    reveals.forEach((el) => ro.observe(el));
  } else {
    reveals.forEach((el) => el.classList.add('in'));
  }

  /* copy buttons on the command blocks; the comments are left behind */
  document.querySelectorAll('.code-wrap:not(.nocopy)').forEach((wrap) => {
    const pre = wrap.querySelector('pre');
    if (!pre || !navigator.clipboard) return;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'copy';
    btn.textContent = 'copy';
    btn.setAttribute('aria-label', 'Copy these commands');
    btn.addEventListener('click', () => {
      const text = pre.textContent.split('\n')
        .map((line) => line.replace(/\s+#\s.*$/, '').replace(/^#.*$/, '').trimEnd())
        .filter(Boolean).join('\n') + '\n';
      navigator.clipboard.writeText(text).then(() => {
        btn.textContent = 'copied';
        btn.classList.add('copied');
        setTimeout(() => { btn.textContent = 'copy'; btn.classList.remove('copied'); }, 1600);
      }, () => { btn.textContent = 'select'; });
    });
    wrap.appendChild(btn);
  });

  /* protocol filter: matches a protocol's name and its tooltip */
  const find = document.getElementById('proto-find');
  if (find) {
    const box = find.querySelector('input');
    const out = find.querySelector('output');
    const groups = document.querySelectorAll('.proto-group');
    const empty = document.getElementById('proto-empty');
    const norm = (s) => s.toLowerCase().normalize('NFD').replace(/[̀-ͯ]/g, '').replace(/[^a-z0-9.+]+/g, ' ').trim();
    groups.forEach((g) => g.querySelectorAll('.pill').forEach((p) => {
      p.dataset.q = norm(`${p.textContent} ${p.title || ''} ${g.querySelector('h3').textContent}`);
    }));
    find.hidden = false;
    box.addEventListener('input', () => {
      const words = norm(box.value).split(' ').filter(Boolean);
      let shown = 0;
      groups.forEach((g) => {
        let any = false;
        g.querySelectorAll('.pill').forEach((p) => {
          const hit = words.every((w) => p.dataset.q.includes(w));
          p.hidden = !hit;
          if (hit) { any = true; shown++; }
        });
        g.hidden = !any;
      });
      out.textContent = words.length ? `${shown} match${shown === 1 ? '' : 'es'}` : '';
      if (empty) empty.hidden = shown > 0;
    });
  }
})();
