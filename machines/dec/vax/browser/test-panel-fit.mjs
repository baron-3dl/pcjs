// test-panel-fit.mjs -- ovmx-cluster.html's console must be an 80x24 VT screen that exactly fills
// its window, with NO scroll bar of any kind, at desktop and phone sizes (Baron's review of the
// OpenVMX cluster demo, finding 1: "the text panels aren't the same size as the emulated screen,
// so there are scroll bars everywhere").  The demo embeds this page in a panel the shape of an
// 80x24 grid, so the page's window IS the panel.
//
// Drives the page's own test hook (__clusterNode.injectOut -- the call the real Worker's "out"
// message makes) with ~60 lines, so the scrollback overflows 24 rows, then at 1280x800, 1920x1080
// and 390x844 (each also at a 5:3 "panel" window) asserts:
//   - no element has a scroll bar gutter; neither document scrolls,
//   - the visible glyph grid is 80 columns x 24 rows,
//   - that grid fills >= 97% of the window in both axes (the window is 5:3),
//   - the screen is scrolled to the newest line.
// Before the fix (13px <pre> with overflow:auto, height calc(100vh - 28px)): FAILS (bars, ~103x21).
//
// Run: node machines/dec/vax/browser/test-panel-fit.mjs

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { chromium } from 'playwright';

const here = dirname(fileURLToPath(import.meta.url));
const FILES = { '/ovmx-cluster.html': 'text/html', '/vaxterm.js': 'text/javascript' };
const server = createServer(async (req, res) => {
  const p = req.url.split('?')[0];
  if (!FILES[p]) { res.writeHead(404); res.end('nf'); return; }
  res.writeHead(200, { 'content-type': FILES[p] }); res.end(await readFile(join(here, p.slice(1))));
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const url = `http://127.0.0.1:${server.address().port}/ovmx-cluster.html`;

const SAMPLE = Array.from({ length: 60 }, (_, i) => `%OVMX-I-LINE, console line ${String(i).padStart(2, '0')} ` + 'x'.repeat(i % 7 === 0 ? 90 : 20)).join('\r\n') + '\r\nUsername: ';

function measure() {
  const px = (v) => parseFloat(v) || 0;
  const bad = [];
  for (const el of document.querySelectorAll('*')) {
    const cs = getComputedStyle(el);
    if (cs.display === 'none' || cs.display === 'inline') continue;
    const gy = el.offsetWidth - el.clientWidth - px(cs.borderLeftWidth) - px(cs.borderRightWidth);
    const gx = el.offsetHeight - el.clientHeight - px(cs.borderTopWidth) - px(cs.borderBottomWidth);
    const id = el.tagName.toLowerCase() + (el.id ? '#' + el.id : '');
    if (el.clientWidth && gy > 1) bad.push(`${id}: vertical scroll bar ${gy}px`);
    if (el.clientHeight && gx > 1) bad.push(`${id}: horizontal scroll bar ${gx}px`);
  }
  const clipped = (o) => o === 'hidden' || o === 'clip';
  for (const el of [document.documentElement, document.body]) {
    const cs = getComputedStyle(el), dcs = getComputedStyle(document.documentElement);
    const hideX = clipped(cs.overflowX) || (el === document.body && clipped(dcs.overflowX));
    const hideY = clipped(cs.overflowY) || (el === document.body && clipped(dcs.overflowY));
    if (!hideX && el.scrollWidth > el.clientWidth + 1) bad.push(`${el.tagName.toLowerCase()} scrolls horizontally`);
    if (!hideY && el.scrollHeight > el.clientHeight + 1) bad.push(`${el.tagName.toLowerCase()} scrolls vertically`);
  }
  const s = document.getElementById('screen'), cs = getComputedStyle(s);
  const probe = document.createElement('span');
  probe.style.cssText = 'position:absolute;visibility:hidden;white-space:pre;font:' + cs.font;
  probe.textContent = 'M'.repeat(100); document.body.appendChild(probe);
  const cw = probe.getBoundingClientRect().width / 100; probe.remove();
  const lh = px(cs.lineHeight) || px(cs.fontSize) * 1.2;
  const w = s.clientWidth - px(cs.paddingLeft) - px(cs.paddingRight), h = s.clientHeight - px(cs.paddingTop) - px(cs.paddingBottom);
  return { bad, cols: w / cw, rows: h / lh, fw: w / innerWidth, fh: h / innerHeight,
    atEnd: s.scrollTop + s.clientHeight >= s.scrollHeight - 1, text: s.textContent.length };
}

const browser = await chromium.launch({ ignoreDefaultArgs: ['--hide-scrollbars'] });   // show real scroll bars
const fails = [];
try {
  for (const [W, H] of [[1280, 800], [1920, 1080], [390, 844]]) {
    for (const [pw, ph] of [[W, H], [Math.min(W, 872), Math.round(Math.min(W, 872) * 0.6)]]) {
      const page = await browser.newPage({ viewport: { width: pw, height: ph } });
      await page.goto(url, { waitUntil: 'load' });
      await page.waitForFunction(() => window.__clusterNode);
      await page.evaluate((b) => window.__clusterNode.injectOut(b), Array.from(Buffer.from(SAMPLE)));
      await page.waitForTimeout(500);
      const m = await page.evaluate(measure);
      const tag = `${pw}x${ph} (page ${W}x${H})`;
      console.log(`${tag}: ${m.cols.toFixed(2)}x${m.rows.toFixed(2)} fill ${(m.fw * 100).toFixed(0)}%x${(m.fh * 100).toFixed(0)}% bars=${m.bad.length}`);
      const exact5to3 = Math.abs(pw / ph - 5 / 3) < 0.02;
      for (const b of m.bad) fails.push(`${tag}: ${b}`);
      if (Math.abs(m.cols - 80) > 0.5 || Math.abs(m.rows - 24) > 0.5) fails.push(`${tag}: grid is ${m.cols.toFixed(2)}x${m.rows.toFixed(2)}, not 80x24`);
      if (exact5to3 === false && pw === W) { /* arbitrary window: only one axis can bind */ if (Math.max(m.fw, m.fh) < 0.97) fails.push(`${tag}: grid does not fill either axis`); }
      else if (m.fw < 0.97 || m.fh < 0.97) fails.push(`${tag}: grid fills only ${(m.fw * 100).toFixed(0)}%x${(m.fh * 100).toFixed(0)}% of the panel`);
      if (!m.atEnd) fails.push(`${tag}: not scrolled to the newest line`);
      await page.close();
    }
  }
} finally { await browser.close(); server.close(); }
if (fails.length) { console.error('FAIL test-panel-fit:\n  ' + fails.join('\n  ')); process.exit(1); }
console.log('PASS test-panel-fit: 80x24, fills the panel, no scroll bars');
