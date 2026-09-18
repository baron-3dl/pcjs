/**
 * @fileoverview pcjsvax-636 -- drive browser/ovmx-cluster.html in a REAL browser, over CDP, and
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 *                 prove the pcjs VAX embeds as a cluster node whose DELQA hooks work end to end
 *
 * WHAT THIS GRADES  (frozen contract openvmx-site demo/cluster/PCJS-NODE-BRIDGE.md)
 * --------------------------------------------------------------------------------
 * tests/xqcluster.test.mjs proves the machine-side bridge (DELQA <-> postMessage) byte-exact under
 * Node.  THIS harness proves the BROWSER carrier: it serves the repo, then loads a tiny PARENT page
 * that embeds browser/ovmx-cluster.html as an iframe (exactly as the demo's node-pcjs.html embeds the
 * pcjs machine).  It asserts, in a real headless Chrome:
 *   1. the machine embeds and the DELQA HubPort comes up -> the iframe posts {t:'nic-ready'} to parent;
 *   2. a {t:'nic-rx'} the parent posts DOWN reaches the child's worker (the parent->iframe->worker
 *      leg of the contract), observed via the child's window.__clusterNode.nicRxCount.
 * The guest does not transmit here (no DECnet running with --no-autoboot), which is why TX is proven
 * deterministically in the Node test, not this one.  It is the pcjs analogue of tests/browserboot.mjs
 * and, like it, is NOT in the 34-check gate: it needs a Chrome binary and ~30s.
 *
 * USAGE
 *   node machines/dec/vax/tests/clusternodeboot.mjs [--rom PATH] [--diskgz PATH] [--chrome PATH]
 *   # ROM  defaults to disks/vaxdisks/ka655x.bin ; DISKGZ to disks/vaxdisks/vms55-rd54.dsk.gz
 */

import http from "http";
import fs from "fs";
import os from "os";
import path from "path";
import { spawn, execFileSync } from "child_process";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");   /* machines/dec/vax/tests -> repo root */
const PARENT_PATH = "/__clusternode_test_parent.html";

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }
function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

function findChrome()
{
    let explicit = getArg("--chrome", null);
    if (explicit) return explicit;
    let cache = path.join(os.homedir(), ".cache/ms-playwright");
    if (fs.existsSync(cache)) {
        for (let d of fs.readdirSync(cache)) {
            let p = path.join(cache, d, "chrome-linux64/chrome");
            if (fs.existsSync(p)) return p;
            p = path.join(cache, d, "chrome-linux/chrome");
            if (fs.existsSync(p)) return p;
        }
    }
    for (let n of ["chromium", "chromium-browser", "google-chrome", "google-chrome-stable"]) {
        try { return execFileSync("which", [n], { encoding: "utf8" }).trim(); } catch (e) {}
    }
    return null;
}

const MIME = { ".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript",
               ".css": "text/css", ".json": "application/json", ".bin": "application/octet-stream",
               ".gz": "application/gzip" };

/** The parent test page: embeds ovmx-cluster.html, gates on nic-ready, then delivers one nic-rx. */
function parentHtml(rom, diskgz)
{
    const q = `?rom=${encodeURIComponent(rom)}&diskgz=${encodeURIComponent(diskgz)}` +
              `&mac=52:54:00:00:00:0B&autoboot=0`;
    return `<!doctype html><meta charset="utf8"><title>cluster-node embed test</title><body>
<script>
window.__test = { nicReady:false, txCount:0, rxDelivered:false, childRx:0, childReady:false, err:null };
const iframe = document.createElement('iframe');
iframe.id = 'node';
iframe.src = '/machines/dec/vax/browser/ovmx-cluster.html${q}';
window.addEventListener('message', (e) => {
  if (e.source !== iframe.contentWindow) return;
  const m = e.data; if (!m || typeof m !== 'object') return;
  if (m.t === 'nic-ready') {
    window.__test.nicReady = true;
    // Act as the parent L2 switch: deliver ONE frame down into this node's DELQA RX.
    const f = new Uint8Array(60);
    f.set([0x52,0x54,0x00,0x00,0x00,0x0B], 0);   // dst = this node
    f.set([0x08,0x00,0x2B,0x0C,0x0C,0x0C], 6);   // src = a peer
    f[12] = 0x60; f[13] = 0x07;                  // SCS ethertype
    for (let i = 14; i < 60; i++) f[i] = i & 0xFF;
    iframe.contentWindow.postMessage({ t:'nic-rx', frame:f.buffer }, '*', [f.buffer]);
    window.__test.rxDelivered = true;
  } else if (m.t === 'nic-tx') {
    window.__test.txCount++;
  }
});
window.addEventListener('error', (e) => { window.__test.err = String(e.message||e); });
document.body.appendChild(iframe);
// Expose a poller the harness reads: mirror the child's introspection handle (same origin).
window.__poll = () => {
  try { const cn = iframe.contentWindow.__clusterNode || {};
        window.__test.childReady = !!cn.nicReady; window.__test.childRx = cn.nicRxCount|0; } catch (e) {}
  return window.__test;
};
</script></body>`;
}

function serve(port, rom, diskgz)
{
    return new Promise((resolve) => {
        let srv = http.createServer((req, res) => {
            let urlPath = req.url.split("?")[0];
            if (urlPath === PARENT_PATH) {
                res.writeHead(200, { "Content-Type": "text/html" });
                res.end(parentHtml(rom, diskgz));
                return;
            }
            let rel = decodeURIComponent(urlPath);
            let file = path.join(REPO_ROOT, rel);
            let real = fs.existsSync(file) ? fs.realpathSync(file) : file;   /* follow the vaxdisks symlinks */
            if (!fs.existsSync(real) || fs.statSync(real).isDirectory()) { res.writeHead(404); res.end("not found"); return; }
            res.writeHead(200, { "Content-Type": MIME[path.extname(urlPath)] || "application/octet-stream" });
            fs.createReadStream(real).pipe(res);
        });
        srv.listen(port, "127.0.0.1", () => resolve(srv));
    });
}

class CDP
{
    constructor(ws) { this.ws = ws; this.id = 0; this.waiting = new Map(); this.sessionId = null;
                      ws.onmessage = (e) => this.onMessage(JSON.parse(e.data)); }
    static async connect(url) {
        let ws = new WebSocket(url);
        await new Promise((res, rej) => { ws.onopen = res; ws.onerror = () => rej(new Error("CDP connect failed")); });
        return new CDP(ws);
    }
    onMessage(msg) {
        if (msg.id !== undefined && this.waiting.has(msg.id)) {
            let { resolve, reject } = this.waiting.get(msg.id); this.waiting.delete(msg.id);
            msg.error ? reject(new Error(msg.error.message)) : resolve(msg.result);
        }
    }
    send(method, params = {}, sessionId = this.sessionId) {
        let id = ++this.id, payload = { id, method, params };
        if (sessionId) payload.sessionId = sessionId;
        this.ws.send(JSON.stringify(payload));
        return new Promise((resolve, reject) => this.waiting.set(id, { resolve, reject }));
    }
    async evaluate(expression) {
        let r = await this.send("Runtime.evaluate", { expression, returnByValue: true, awaitPromise: true });
        if (r.exceptionDetails) throw new Error("page: " + (r.exceptionDetails.exception?.description || r.exceptionDetails.text));
        return r.result.value;
    }
}

async function main()
{
    const rom = getArg("--rom", "/disks/vaxdisks/ka655x.bin");
    const diskgz = getArg("--diskgz", "/disks/vaxdisks/vms55-rd54.dsk.gz");
    const port = parseInt(getArg("--port", "8135"), 10);
    const maxSeconds = parseInt(getArg("--max-seconds", "90"), 10);
    const chrome = findChrome();

    for (const [label, rel] of [["ROM", rom], ["diskgz", diskgz]]) {
        let f = path.join(REPO_ROOT, rel);
        if (!fs.existsSync(f)) { console.log(`clusternodeboot: ${label} ${rel} not found under ${REPO_ROOT}`); process.exit(2); }
    }
    if (!chrome) { console.log("clusternodeboot: no Chrome binary found; pass --chrome PATH"); process.exit(2); }

    console.log("clusternodeboot (pcjsvax-636) -- browser/ovmx-cluster.html embedded as a cluster node, over CDP");
    console.log(`  chrome  ${chrome}`);
    console.log(`  serving ${REPO_ROOT} on http://127.0.0.1:${port}`);
    console.log(`  rom     ${rom}`);
    console.log(`  diskgz  ${diskgz}`);

    let srv = await serve(port, rom, diskgz);
    let profile = fs.mkdtempSync(path.join(os.tmpdir(), "vaxcluster-"));
    let dbgPort = port + 1;
    let proc = spawn(chrome, [
        "--headless=new", "--no-sandbox", "--disable-gpu", "--disable-dev-shm-usage",
        "--no-first-run", "--no-default-browser-check", "--mute-audio",
        `--user-data-dir=${profile}`, `--remote-debugging-port=${dbgPort}`, "about:blank"
    ], { stdio: ["ignore", "ignore", "pipe"] });
    let chromeErr = ""; proc.stderr.on("data", (d) => chromeErr += d.toString());
    let cleanup = () => { try { proc.kill("SIGKILL"); } catch (e) {} srv.close();
                          try { fs.rmSync(profile, { recursive: true, force: true }); } catch (e) {} };
    process.on("exit", cleanup);

    let wsUrl = null;
    for (let i = 0; i < 100 && !wsUrl; i++) {
        await sleep(200);
        try {
            let j = await new Promise((res, rej) => {
                http.get(`http://127.0.0.1:${dbgPort}/json/version`, (r) => {
                    let b = ""; r.on("data", (c) => b += c); r.on("end", () => res(JSON.parse(b)));
                }).on("error", rej);
            });
            wsUrl = j.webSocketDebuggerUrl; console.log(`  browser ${j.Browser}`);
        } catch (e) {}
    }
    if (!wsUrl) { console.log(`clusternodeboot: Chrome never opened a DevTools endpoint.\n${chromeErr}`); cleanup(); process.exit(2); }

    let cdp = await CDP.connect(wsUrl);
    let { targetId } = await cdp.send("Target.createTarget", { url: `http://127.0.0.1:${port}${PARENT_PATH}` }, null);
    let { sessionId } = await cdp.send("Target.attachToTarget", { targetId, flatten: true }, null);
    cdp.sessionId = sessionId;
    await cdp.send("Runtime.enable");

    let t = null, deadline = Date.now() + maxSeconds * 1000;
    while (Date.now() < deadline) {
        await sleep(500);
        try { t = await cdp.evaluate("window.__poll ? window.__poll() : null"); } catch (e) {}
        if (t && t.err) { console.log(`  page error: ${t.err}`); break; }
        if (t && t.nicReady && t.childRx >= 1) break;
    }

    let passed = 0, failed = 0;
    const ok = (c, m) => { if (c) { passed++; console.log("  PASS " + m); } else { failed++; console.log("  FAIL " + m); } };
    console.log("");
    ok(t && t.nicReady, "the embedded pcjs machine posted {t:'nic-ready'} to its parent (DELQA HubPort up)");
    ok(t && t.childReady, "the child's __clusterNode reports nicReady (machine embedded + attached)");
    ok(t && t.rxDelivered, "the parent delivered a {t:'nic-rx'} frame down to the node");
    ok(t && t.childRx >= 1, "the {t:'nic-rx'} reached the child worker (parent->iframe->worker leg)");
    ok(!(t && t.err), "no page/console error");

    cleanup();
    if (failed) { console.log(`\nclusternodeboot: FAIL (${failed} of ${passed + failed})`); process.exit(1); }
    console.log(`\nclusternodeboot: ALL PASS (${passed}) -- the pcjs VAX embeds as a cluster node and its DELQA hooks work in a real browser`);
}

main().catch((e) => { console.log("clusternodeboot: ERROR " + (e && e.stack || e)); process.exit(1); });
