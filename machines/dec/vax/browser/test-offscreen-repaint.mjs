// rd vms-0bc: ovmx-cluster.html's terminal only painted the DOM from a requestAnimationFrame
// tick (`(function frame(){term.render();requestAnimationFrame(frame);})()`). Chromium throttles
// rAF callbacks in an OFFSCREEN CROSS-ORIGIN iframe -- exactly how the live cluster demo
// (openvmx.3dl.dev/demo/cluster/) embeds this page, from vax.3dl.network. Measured live
// (tests/lab/captures/vms-1a1-visitor-20260928/probes/probe-c5.log in the vms repo): the real
// VAX's panel sat frozen at 211 characters of POST for 180 seconds while the guest was actually
// booting and clustering underneath (node A's own CNXMAN was already naming it); one
// iframe.scrollIntoView() (no click, no keystroke) and ten seconds later the panel held 6864
// characters -- the whole boot, cluster join, and Username: prompt.
//
// This test reproduces the trigger with a REAL headless Chromium (rAF-offscreen-iframe
// throttling is a browser scheduling behavior, not something a DOM-only/jsdom test can see):
//   1. serve ovmx-cluster.html on its own local origin (a different port = a different origin,
//      same as the real vax.3dl.network vs. openvmx.3dl.dev split) and embed it, unscrolled, far
//      below the fold of a parent page;
//   2. drive the terminal the same way a real boot does -- term.write() via the page's own
//      __clusterNode.injectOut() test hook (added alongside this fix), which is exactly the call
//      the real Worker's "out" message makes -- WITHOUT downloading a ROM/disk or running the
//      CPU emulator;
//   3. never scroll, never click, never focus the iframe;
//   4. assert the DOM text visible in the (still off-screen) panel has grown past the POST
//      countdown and into the "boot" content, including Username:.
//
// Before the fix (render() driven only by requestAnimationFrame): FAILS -- the panel's DOM text
// stays stuck at the first chunk (or less), because the rAF callback that would have painted the
// later writes never runs while the frame sits below the fold.
// After the fix (render() also driven by a plain setInterval, which Chromium does not throttle
// for an off-screen iframe in a foregrounded tab): PASSES.
//
// Run: node machines/dec/vax/browser/test-offscreen-repaint.mjs

import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, join, extname } from 'node:path';
import { chromium } from 'playwright';

const here = dirname(fileURLToPath(import.meta.url));

const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript' };

/** Minimal static file server rooted at `here`, for whichever set of paths it's handed. */
function serve(routes) {
  return new Promise((resolve) => {
    const server = createServer(async (req, res) => {
      const path = req.url.split('?')[0];
      const body = routes[path];
      if (body === undefined) { res.writeHead(404); res.end('not found'); return; }
      if (typeof body === 'string' && body.startsWith('FILE:')) {
        try {
          const data = await readFile(join(here, body.slice(5)));
          res.writeHead(200, { 'content-type': MIME[extname(body)] || 'application/octet-stream' });
          res.end(data);
        } catch (e) { res.writeHead(500); res.end(String(e)); }
        return;
      }
      res.writeHead(200, { 'content-type': 'text/html' });
      res.end(body);
    });
    server.listen(0, '127.0.0.1', () => resolve(server));
  });
}

// A tall parent page: the iframe sits 2000px down, well below any real viewport, and the page
// never scrolls it into view. This is the "vax.3dl.network node embedded in openvmx.3dl.dev"
// shape, just on two local ports instead of two real subdomains -- different ports on 127.0.0.1
// are still different origins, so the cross-origin condition the real bug depends on holds.
function parentHtml(nodeOrigin) {
  return `<!DOCTYPE html><html><head><meta charset="utf-8"></head>
<body style="margin:0">
  <div style="height:2000px;background:#111"></div>
  <iframe id="node" title="cluster node" src="${nodeOrigin}/ovmx-cluster.html"
          style="width:640px;height:400px;border:0"></iframe>
  <div style="height:2000px"></div>
</body></html>`;
}

// A representative slice of a real VAX/VMS boot transcript (shape verified against
// tests/lab/captures/vms-1a1-visitor-20260928/probes/probe-c5.log in the vms repo): the KA655
// POST countdown, then the OpenVMS boot + cluster-join lines, ending at the login gate.
const POST_CHUNK = [
  '', 'KA655-B V5.3, VMB 2.7', 'Performing normal system tests.',
  '40..39..38..37..36..35..34..33..32..31..30..29..28..27..26..25..',
  '24..23..22..21..20..19..18..17..16..15..14..13..12..11..10..09..',
  '08..07..06..05..04..03..', 'Tests completed.', '',
].join('\r\n');

const BOOT_CHUNKS = [
  '%SYSBOOT-I-SYSBOOT Mapping the SYSDUMP.DMP on the System Disk\r\n' +
  '   OpenVMS (TM) VAX Version V7.3     Major version id = 1 Minor version id = 0\r\n',
  '%SYSINIT, waiting to form or join a VMScluster system\r\n' +
  '%CNXMAN,  proposing formation of a VAXcluster\r\n' +
  '%CNXMAN,  now a VAXcluster member -- system VAXC\r\n' +
  '%CNXMAN,  completing VAXcluster state transition\r\n',
  '\r\n Welcome to OpenVMS (TM) VAX Operating System, Version V7.3    \r\n\r\nUsername: ',
];

async function main() {
  const nodeServer = await serve({ '/ovmx-cluster.html': 'FILE:ovmx-cluster.html', '/vaxterm.js': 'FILE:vaxterm.js' });
  const nodePort = nodeServer.address().port;
  const nodeOrigin = `http://127.0.0.1:${nodePort}`;

  const parentServer = await serve({ '/harness.html': parentHtml(nodeOrigin) });
  const parentPort = parentServer.address().port;

  const browser = await chromium.launch({ headless: true });
  try {
    const page = await browser.newPage({ viewport: { width: 800, height: 600 } });
    await page.goto(`http://127.0.0.1:${parentPort}/harness.html`, { waitUntil: 'load' });

    // Locate the cluster-node frame WITHOUT ever scrolling it into view. Playwright reads
    // frame/element state via CDP, which is not blocked by the frame's cross-origin-ness the way
    // page-context JS (iframe.contentDocument) would be -- this mirrors a real visitor's browser
    // devtools, not a same-origin cheat.
    const nodeFrame = page.frames().find((f) => f.url().startsWith(nodeOrigin));
    if (!nodeFrame) throw new Error('cluster-node iframe did not load');

    // Confirm the panel is genuinely off-screen and never becomes visible for the rest of the run.
    const rectBefore = await page.locator('#node').boundingBox();
    if (!rectBefore || rectBefore.y < 600) throw new Error(`iframe is not below the fold: ${JSON.stringify(rectBefore)}`);

    await nodeFrame.waitForFunction(() => !!(window.__clusterNode && window.__clusterNode.injectOut));

    // Drive the terminal exactly as a real boot would -- one write, a pause, more writes -- while
    // never touching scroll position, focus, or the iframe element.
    await nodeFrame.evaluate((s) => window.__clusterNode.injectOut(new TextEncoder().encode(s)), POST_CHUNK);
    for (const chunk of BOOT_CHUNKS) {
      await page.waitForTimeout(700);
      await nodeFrame.evaluate((s) => window.__clusterNode.injectOut(new TextEncoder().encode(s)), chunk);
    }

    // The decisive wait: nothing is clicked, typed, or scrolled from here on -- exactly the
    // "visitor never touches the VAX panel" shape of the field report.
    await page.waitForTimeout(6000);

    const rectAfter = await page.locator('#node').boundingBox();
    if (!rectAfter || rectAfter.y < 600) throw new Error('iframe became visible during the run -- test invalid');

    const screenText = await page.frameLocator('#node').locator('#screen').innerText();

    const checks = [
      ['DOM text grew past the POST countdown alone (> 300 chars)', screenText.length > 300],
      ['DOM text reached the OpenVMS boot banner', screenText.includes('OpenVMS (TM) VAX Operating System')],
      ['DOM text reached the cluster-join line', screenText.includes('now a VAXcluster member')],
      ['DOM text reached Username: -- i.e. the login gate, not just POST', screenText.includes('Username:')],
    ];

    console.log('=== rd vms-0bc: offscreen cross-origin panel repaint ===');
    console.log(`  iframe bounding box: ${JSON.stringify(rectAfter)} (never scrolled into a 600px viewport)`);
    console.log(`  DOM text length at end of run: ${screenText.length}`);
    let pass = true;
    for (const [name, ok] of checks) {
      console.log(`  ${ok ? 'PASS' : 'FAIL'}  ${name}`);
      if (!ok) pass = false;
    }
    if (!pass) {
      console.log(`\n  --- captured DOM text (${screenText.length} chars) ---\n${screenText}\n  --- end ---`);
    }
    console.log(pass ? '\nALL PASS' : '\nFAILURES');
    process.exitCode = pass ? 0 : 1;
  } finally {
    await browser.close();
    nodeServer.close();
    parentServer.close();
  }
}

main().catch((e) => { console.error('FATAL', e); process.exit(1); });
