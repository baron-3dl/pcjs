// test-device-attributes.mjs -- rd vms-e097: the pcjs console must answer VT Device Attributes.
// LOGINOUT sends ESC[c (DA1) and ESC Z (DECID); a silent console left the guest's SHOW TERMINAL at
// "Device_Type: Unknown".  The widget is a VT100-class ANSI CRT (no AVO), so it answers
// ESC [ ? 1 ; 0 c ("VT100, no options") -- not a VT220 string -- through the keystroke path
// (onInput -> worker {cmd:"input"} -> guest console RX).
//
// Part 1 (pure): VaxTerminal unit behaviour.  Part 2 (browser): ovmx-cluster.html with a recording
// fake Worker; bytes fed through __clusterNode.injectOut; asserts the exact worker "input" message,
// for both node MACs (B and C), and that the screen text/consoleText are unchanged by the query.
//
// Run: node machines/dec/vax/browser/test-device-attributes.mjs
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { VaxTerminal, DA_RESPONSE } from './vaxterm.js';

const here = dirname(fileURLToPath(import.meta.url));
let fails = 0;
const check = (name, ok) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}`); if (!ok) fails++; };
const EXPECT = '\x1B[?1;0c';
check('DA_RESPONSE is VT100-no-options', DA_RESPONSE === EXPECT);

const enc = (s) => new TextEncoder().encode(s);
function unit(input) {
  const sent = [];
  const el = { tagName: 'PRE', textContent: '', scrollTop: 0, scrollHeight: 0, addEventListener() {} };
  const t = new VaxTerminal(el, (x) => sent.push(x));
  t.write(enc(input)); t.dirty = true; t.render();
  return { sent, t, text: el.textContent };
}
for (const [name, seq] of [['ESC[c', '\x1B[c'], ['ESC[0c', '\x1B[0c'], ['ESC Z', '\x1BZ']]) {
  const r = unit('ab' + seq + 'cd');
  check(`${name} -> exactly one DA reply`, r.sent.length === 1 && r.sent[0] === EXPECT);
  check(`${name} leaves no text artefacts`, r.t.lines[0] === 'abcd');
}
for (const [name, seq] of [['DA2 ESC[>c', '\x1B[>c'], ['ESC[?c', '\x1B[?c'], ['ESC[1c', '\x1B[1c'], ['ESC[5n', '\x1B[5n'], ['ESC[6n', '\x1B[6n'], ['ESC[K', '\x1B[K']])
  check(`${name} is not answered as DA1`, unit(seq).sent.length === 0);

// ---- browser part ----
let chromium;
try { ({ chromium } = await import('playwright')); }
catch { console.log('FAIL  playwright not importable'); process.exit(1); }
const FILES = { '/ovmx-cluster.html': 'text/html', '/vaxterm.js': 'text/javascript' };
const server = createServer(async (req, res) => {
  const p = req.url.split('?')[0];
  if (p === '/rom') { res.writeHead(200, { 'content-type': 'application/octet-stream' }); res.end(Buffer.from([1, 2, 3, 4])); return; }
  if (!FILES[p]) { res.writeHead(404); res.end('nf'); return; }
  res.writeHead(200, { 'content-type': FILES[p] }); res.end(await readFile(join(here, p.slice(1))));
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const base = `http://127.0.0.1:${server.address().port}/ovmx-cluster.html?rom=/rom`;
const browser = await chromium.launch();
for (const [label, mac] of [['Node B', '52:54:00:00:00:0B'], ['Node C', '52:54:00:00:00:0C']]) {
  const page = await browser.newPage();
  await page.addInitScript(() => {
    window.__posts = [];
    window.Worker = class { constructor() {} postMessage(m) { window.__posts.push(m); } };
  });
  await page.goto(`${base}&mac=${mac}`);
  await page.waitForFunction(() => window.__posts && window.__posts.some((m) => m.cmd === 'start'));
  await page.evaluate(() => {
    const e = (s) => Array.from(s, (c) => c.charCodeAt(0));
    window.__clusterNode.injectOut(e('Username: \x1B[c'));
    window.__clusterNode.injectOut(e('\x1BZ'));
  });
  const inputs = await page.evaluate(() => window.__posts.filter((m) => m.cmd === 'input').map((m) => m.text));
  check(`${label}: worker got exactly [DA1 reply, DECID reply]`,
    inputs.length === 2 && inputs[0] === EXPECT && inputs[1] === EXPECT);
  await page.waitForTimeout(200);
  const screen = await page.evaluate(() => document.getElementById('screen').textContent);
  const ct = await page.evaluate(() => window.__clusterNode.consoleText());
  check(`${label}: consoleText has the prompt, no escape residue`, ct === 'Username: ');
  check(`${label}: #screen shows the prompt`, screen.startsWith('Username: '));
  await page.close();
}
await browser.close(); server.close();
console.log(fails ? `\n${fails} FAILED` : '\nALL PASS');
process.exit(fails ? 1 : 0);
