// test-device-attributes.mjs -- rd vms-e097: the pcjs console must answer VT Device Attributes.
// LOGINOUT sends ESC[c (DA1) and ESC Z (DECID); a silent console left the guest's SHOW TERMINAL at
// "Device_Type: Unknown".  The terminal is a VT100-class ANSI CRT (no AVO), so it answers
// ESC [ ? 1 ; 0 c ("VT100, no options") -- not a VT220 string.
//
// rd vms-bb0: the ANSWER moved from the widget (main thread, host time) to browser/vaxmachine.js
// (the Worker, guest time), because a reply that crossed to the page and back reached the KA655
// ROM after it had stopped waiting and swallowed the auto-boot's "B".  So this file now grades:
// Part 1 (pure): VaxMachine answers DA1/DECID on the console line, exactly once each, and nothing
// else; the widget parses the same sequences, paints nothing for them, and does NOT answer them
// (two replies would be two terminals).  Part 2 (browser): ovmx-cluster.html with a recording fake
// Worker; bytes fed through __clusterNode.injectOut; asserts the page posts NO "input" for a query,
// for both node MACs (B and C), and that the screen text/consoleText are unchanged by the query.
// tests/consolequery.test.mjs grades the timing (the reply reaches the ROM inside its window).
//
// Run: node machines/dec/vax/browser/test-device-attributes.mjs
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { VaxTerminal, DA_RESPONSE } from './vaxterm.js';
import { VaxMachine } from './vaxmachine.js';

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
// The line's terminal, in the Worker: what VaxMachine queues for the guest after it writes `s`.
const rom = new Uint8Array(await readFile(join(here, 'ka655x.bin')));
function lineReply(s) {
  const log = console.log; console.log = () => {};             // rommachine.js's "Bus: Added ..." chatter
  const m = new VaxMachine({ romBytes: rom, disk: null, autoBoot: false });
  console.log = log;
  m.consoleDev.output.length = 0;                               // only what this check writes
  m.consoleDev.output.push(...enc(s));
  m.scanQueries(m.consoleDev);
  return m.answer;
}
for (const [name, seq] of [['ESC[c', '\x1B[c'], ['ESC[0c', '\x1B[0c'], ['ESC Z', '\x1BZ']]) {
  check(`${name} -> exactly one DA reply on the line`, lineReply('ab' + seq + 'cd') === EXPECT);
  const r = unit('ab' + seq + 'cd');
  check(`${name} -> the widget does not answer too`, r.sent.length === 0);
  check(`${name} leaves no text artefacts`, r.t.lines[0] === 'abcd');
}
for (const [name, seq] of [['DA2 ESC[>c', '\x1B[>c'], ['ESC[?c', '\x1B[?c'], ['ESC[1c', '\x1B[1c'], ['ESC[5n', '\x1B[5n'], ['ESC[6n', '\x1B[6n'], ['ESC[K', '\x1B[K']]) {
  check(`${name} is not answered as DA1`, lineReply(seq) === '');
  check(`${name} is not answered by the widget`, unit(seq).sent.length === 0);
}
check('a query split across two output scans is answered once', (() => {
  const log = console.log; console.log = () => {};
  const m = new VaxMachine({ romBytes: rom, disk: null, autoBoot: false });
  console.log = log;
  m.consoleDev.output.length = 0;
  m.consoleDev.output.push(0x1B); m.scanQueries(m.consoleDev);
  m.consoleDev.output.push(0x5B); m.scanQueries(m.consoleDev);
  m.consoleDev.output.push(0x63); m.scanQueries(m.consoleDev);
  return m.answer === EXPECT;
})());

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
  check(`${label}: the page sends no reply of its own (the Worker's machine answers)`, inputs.length === 0);
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
