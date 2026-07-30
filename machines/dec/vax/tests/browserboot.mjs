/**
 * @fileoverview pcjsvax-de8 / pcjsvax-ae1 -- drive browser/vax.html in a REAL browser, over CDP,
 *               and grade the OpenVMS boot it produces
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY IT EXISTS
 * -------------
 * tests/vaxbrowserboot.js grades browser/vaxmachine.js and browser/imageprovider.js under NODE.
 * That is most of the deliverable, but it is not the claim: the claim is "OpenVMS boots in a
 * browser tab from a file the user picked".  Three things in that sentence exist only in a browser
 * and cannot be reached from Node at all --
 *
 *   1. `FileReaderSync` over `File.slice()`, which is the ONLY synchronous lazy read a browser has
 *      and therefore the whole reason the machine runs in a Worker (see browser/vaxworker.js);
 *   2. the module Worker itself, and the MessageChannel yield that keeps the tab alive;
 *   3. the file picker, i.e. a real `File` arriving through a real `<input type=file>`.
 *
 * -- so this runs the actual page in headless Chrome, feeds the two pickers with
 * `DOM.setFileInputFiles`, presses Start, and reads the TERMINAL's own line buffer.  Nothing is
 * simulated and no code path is substituted.
 *
 * *** IT IS NOT IN THE 34-CHECK GATE. ***  It needs a Chrome binary, a free TCP port and ~2 minutes,
 * none of which the gate has; the gate's job is the CPU and the devices.  It is run by hand, and
 * its result is a measurement to be pasted, not a green tick to be assumed.
 *
 * USAGE
 *   node machines/dec/vax/tests/browserboot.mjs --volume PATH [--rom PATH] [--chrome PATH]
 *                                              [--port N] [--max-seconds S] [--login USER]
 *
 * The ROM defaults to $PCJS_VAX_REPO/open-simh/VAX/ka655x.bin.  --chrome defaults to the Playwright
 * cache, then to whatever `chromium`/`google-chrome` is on PATH; if none is found it says so BY
 * NAME and exits non-zero rather than reporting a pass it did not observe.
 */

import fs from "fs";
import os from "os";
import path from "path";
import http from "http";
import { spawn, execFileSync } from "child_process";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");
const PAGE = "/machines/dec/vax/browser/vax.html";

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function findChrome()
{
    let explicit = getArg("--chrome", null);
    if (explicit) return explicit;
    let cache = path.join(os.homedir(), ".cache/ms-playwright");
    if (fs.existsSync(cache)) {
        for (let d of fs.readdirSync(cache).filter((n) => n.startsWith("chromium-")).sort().reverse()) {
            let p = path.join(cache, d, "chrome-linux64/chrome");
            if (fs.existsSync(p)) return p;
            p = path.join(cache, d, "chrome-linux/chrome");
            if (fs.existsSync(p)) return p;
        }
    }
    for (let n of ["chromium", "chromium-browser", "google-chrome", "google-chrome-stable"]) {
        try { return execFileSync("which", [n], {encoding: "utf8"}).trim(); } catch (e) {}
    }
    return null;
}

/* ---------------------------------------------------------------------------------------------- *
 * A static file server rooted at the repo, because the page loads ES modules and a module Worker   *
 * and both are blocked under file://.  Range is NOT implemented and does not need to be: the disk  *
 * arrives through the FILE PICKER, which is the path being graded.                                 *
 * ---------------------------------------------------------------------------------------------- */
const MIME = {".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript",
              ".css": "text/css", ".json": "application/json", ".bin": "application/octet-stream"};

function serve(port)
{
    return new Promise((resolve) => {
        let srv = http.createServer((req, res) => {
            let rel = decodeURIComponent(req.url.split("?")[0]);
            let file = path.join(REPO_ROOT, rel);
            if (!file.startsWith(REPO_ROOT) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
                res.writeHead(404); res.end("not found"); return;
            }
            res.writeHead(200, {"Content-Type": MIME[path.extname(file)] || "application/octet-stream"});
            fs.createReadStream(file).pipe(res);
        });
        srv.listen(port, "127.0.0.1", () => resolve(srv));
    });
}

/* ---------------------------------------------------------------------------------------------- *
 * The smallest CDP client that can do the job: Node 22+ has a global WebSocket, so there is no      *
 * dependency here at all.  Commands carry a sessionId so they reach the page target rather than     *
 * the browser target.                                                                               *
 * ---------------------------------------------------------------------------------------------- */
class CDP
{
    constructor(ws) { this.ws = ws; this.id = 0; this.waiting = new Map(); this.sessionId = null;
                      ws.onmessage = (e) => this.onMessage(JSON.parse(e.data)); }

    static async connect(url)
    {
        let ws = new WebSocket(url);
        await new Promise((res, rej) => { ws.onopen = res; ws.onerror = (e) => rej(new Error("CDP connect failed")); });
        return new CDP(ws);
    }

    onMessage(msg)
    {
        if (msg.id !== undefined && this.waiting.has(msg.id)) {
            let {resolve, reject} = this.waiting.get(msg.id);
            this.waiting.delete(msg.id);
            msg.error ? reject(new Error(`${msg.error.message} (${JSON.stringify(msg.error.data || "")})`)) : resolve(msg.result);
        }
    }

    send(method, params = {}, sessionId = this.sessionId)
    {
        let id = ++this.id;
        let payload = {id, method, params};
        if (sessionId) payload.sessionId = sessionId;
        this.ws.send(JSON.stringify(payload));
        return new Promise((resolve, reject) => this.waiting.set(id, {resolve, reject}));
    }

    async evaluate(expression)
    {
        let r = await this.send("Runtime.evaluate", {expression, returnByValue: true, awaitPromise: true});
        if (r.exceptionDetails) throw new Error("page: " + (r.exceptionDetails.exception?.description || r.exceptionDetails.text));
        return r.result.value;
    }
}

async function main()
{
    let volume = getArg("--volume", null);
    let romPath = getArg("--rom", path.join(process.env.PCJS_VAX_REPO || path.resolve(REPO_ROOT, "../pcjs-vax"),
                                            "open-simh/VAX/ka655x.bin"));
    let port = parseInt(getArg("--port", "8123"), 10);
    let maxSeconds = parseInt(getArg("--max-seconds", "600"), 10);
    let login = getArg("--login", "SYSTEM");
    let chrome = findChrome();

    if (!volume || !fs.existsSync(volume)) { console.log(`browserboot: --volume PATH is required and must exist (point it at a COPY)`); process.exit(2); }
    if (!fs.existsSync(romPath)) { console.log(`browserboot: the ROM ${romPath} does not exist; pass --rom PATH`); process.exit(2); }
    if (!chrome) { console.log(`browserboot: no Chrome binary found; pass --chrome PATH`); process.exit(2); }

    console.log(`browserboot (pcjsvax-de8/ae1) -- browser/vax.html in a REAL browser, over CDP`);
    console.log(`  chrome    ${chrome}`);
    console.log(`  serving   ${REPO_ROOT} on http://127.0.0.1:${port}`);
    console.log(`  page      ${PAGE}`);
    console.log(`  rom       ${romPath}`);
    console.log(`  volume    ${volume} (${fs.statSync(volume).size} bytes)`);

    let hashBefore = fs.statSync(volume).size + ":" + fs.statSync(volume).mtimeMs;
    let srv = await serve(port);
    let profile = fs.mkdtempSync(path.join(os.tmpdir(), "vaxchrome-"));
    let dbgPort = port + 1;
    let proc = spawn(chrome, [
        "--headless=new", "--no-sandbox", "--disable-gpu", "--disable-dev-shm-usage",
        "--no-first-run", "--no-default-browser-check", "--mute-audio",
        `--user-data-dir=${profile}`, `--remote-debugging-port=${dbgPort}`, "about:blank"
    ], {stdio: ["ignore", "ignore", "pipe"]});
    let chromeErr = "";
    proc.stderr.on("data", (d) => { chromeErr += d.toString(); });

    let cleanup = () => { try { proc.kill("SIGKILL"); } catch (e) {} srv.close();
                          try { fs.rmSync(profile, {recursive: true, force: true}); } catch (e) {} };
    process.on("exit", cleanup);

    /* Wait for the DevTools endpoint rather than sleeping a guessed amount. */
    let wsUrl = null;
    for (let i = 0; i < 100 && !wsUrl; i++) {
        await sleep(200);
        try {
            let j = await new Promise((res, rej) => {
                http.get(`http://127.0.0.1:${dbgPort}/json/version`, (r) => {
                    let b = ""; r.on("data", (c) => b += c); r.on("end", () => res(JSON.parse(b)));
                }).on("error", rej);
            });
            wsUrl = j.webSocketDebuggerUrl;
            console.log(`  browser   ${j.Browser}`);
        } catch (e) {}
    }
    if (!wsUrl) { console.log(`browserboot: Chrome never opened a DevTools endpoint.\n${chromeErr}`); cleanup(); process.exit(2); }

    let cdp = await CDP.connect(wsUrl);
    let {targetId} = await cdp.send("Target.createTarget", {url: `http://127.0.0.1:${port}${PAGE}`}, null);
    let {sessionId} = await cdp.send("Target.attachToTarget", {targetId, flatten: true}, null);
    cdp.sessionId = sessionId;
    await cdp.send("Runtime.enable");
    await cdp.send("Page.enable");
    await cdp.send("DOM.enable");

    /* Console/page errors are FATAL here.  A page that logs "FileReaderSync is unavailable" and
       then sits still would otherwise time out with a misleading reason. */
    let pageErrors = [];
    cdp.onMessage = ((orig) => function(msg) {
        if (msg.method === "Runtime.exceptionThrown") {
            pageErrors.push(msg.params.exceptionDetails.exception?.description || msg.params.exceptionDetails.text);
        }
        return orig.call(this, msg);
    })(cdp.onMessage);

    await sleep(1500);
    let title = await cdp.evaluate("document.title");
    console.log(`  loaded    "${title}"`);

    /* THE FILE PICKERS, fed with real local files.  `DOM.setFileInputFiles` is how a browser is
       handed a `File` without a human; the change event is dispatched explicitly afterwards so the
       page's handler runs regardless of the CDP version's event behaviour (the handlers are
       idempotent -- each just re-reads the input's own files). */
    let {root} = await cdp.send("DOM.getDocument", {depth: 1});
    for (let [sel, file] of [["#rom", path.resolve(romPath)], ["#disk", path.resolve(volume)]]) {
        let {nodeId} = await cdp.send("DOM.querySelector", {nodeId: root.nodeId, selector: sel});
        await cdp.send("DOM.setFileInputFiles", {files: [file], nodeId});
        await cdp.evaluate(`document.querySelector("${sel}").dispatchEvent(new Event("change", {bubbles: true}))`);
        await sleep(300);
    }
    console.log(`  pickers   ${await cdp.evaluate("document.getElementById('status').textContent")}`);

    let canRun = await cdp.evaluate("!document.getElementById('run').disabled");
    if (!canRun) { console.log(`browserboot: the Start button never enabled -- the pickers did not take`); cleanup(); process.exit(1); }

    await cdp.evaluate("document.getElementById('run').click()");
    console.log(`  started   ${new Date().toISOString()}`);

    /* THE TERMINAL'S OWN LINE BUFFER is what is read -- not a copy of the console stream.  What is
       graded is therefore what a human would SEE, escape sequences consumed and CR/BS applied. */
    const screen = () => cdp.evaluate("window.vaxTerm.lines.join('\\n')");
    let t0 = Date.now(), deadline = t0 + maxSeconds * 1000;
    let promptAt = 0, sawBarePrompt = false, text = "", lastLog = 0, stop = null;
    while (Date.now() < deadline) {
        await sleep(1000);
        if (pageErrors.length) { stop = "PAGE EXCEPTION"; break; }
        /* FAIL FAST.  A Worker that cannot even load its modules would otherwise sit here until the
           wall-clock cap and report the wrong reason -- 420 wasted seconds, measured once. */
        let werr = await cdp.evaluate("window.vaxError");
        if (werr) { stop = `WORKER/MACHINE ERROR: ${werr}`; break; }
        text = await screen();
        if (Date.now() - lastLog >= 15000) {
            lastLog = Date.now();
            console.log(`  .. ${((Date.now() - t0) / 1000).toFixed(0)}s  ${await cdp.evaluate("window.vaxStatus()")}`);
        }
        if (!sawBarePrompt && /(^|\n)Username: /.test(text) && !/Username: {2,}/.test(text.split("\n").filter((l) => /^Username: /.test(l)).pop() || "")) {
            sawBarePrompt = true; promptAt = Date.now();
            console.log(`  .. bare \`Username: \` on screen after ${((promptAt - t0) / 1000).toFixed(0)}s; typing "${login}"`);
            await cdp.evaluate(`window.vaxType(${JSON.stringify(login + "\r")})`);
        }
        if (promptAt && /Password:|Welcome to OpenVMS/.test(text)) { stop = "LOGGED IN"; break; }
        if (promptAt && Date.now() - promptAt > 60000) { stop = "THE PROMPT NEVER ANSWERED"; break; }
    }
    if (!stop) stop = "WALL-CLOCK CAP";

    let status = await cdp.evaluate("window.vaxStatus()");
    let attach = await cdp.evaluate("document.getElementById('attach').textContent");
    console.log(`\n  stopped   ${stop} after ${((Date.now() - t0) / 1000).toFixed(0)}s`);
    console.log(`  status    ${status}`);
    console.log(`  attach    ${attach.replace(/\n/g, "\n            ")}`);
    if (pageErrors.length) console.log(`  page errors:\n    ${pageErrors.join("\n    ")}`);

    let passed = 0, failed = 0;
    const check = (name, cond, detail) => {
        if (cond) { passed++; console.log(`  PASS  ${name}`); }
        else { failed++; console.log(`  FAIL  ${name}${detail ? " -- " + detail : ""}`); }
    };
    console.log("");
    check("no uncaught exception in the page", pageErrors.length === 0, pageErrors[0]);
    check("the KA655 ROM ran and reached the console prompt `>>>`", /^>>>/m.test(text) || />>>/.test(text));
    check("DUA0 attached read/write (copy-on-write overlay, not UNIT_RO)", /read\/write via copy-on-write overlay/.test(attach), attach);
    check("OpenVMS printed its banner", /VAX\/VMS V[0-9.]+ +node /.test(text));
    check("a BARE `Username: ` reached the SCREEN (not the padded OPCOM audit field)", sawBarePrompt);
    check(`the prompt ANSWERED "${login}"`, /Password:|Welcome to OpenVMS/.test(text), stop);
    check("the username was ECHOED, i.e. the keyboard path reached the terminal driver",
        new RegExp(`Username: ${login}`).test(text));
    check("the picked file was NOT modified", hashBefore === fs.statSync(volume).size + ":" + fs.statSync(volume).mtimeMs);

    console.log(`\n----- LAST 30 SCREEN LINES -----`);
    console.log(text.split("\n").slice(-30).join("\n"));
    console.log(`\nPASSED ${passed} / FAILED ${failed}`);
    cleanup();
    process.exit(failed ? 1 : 0);
}

main().catch((e) => { console.log("browserboot: " + (e.stack || e)); process.exit(2); });
