/**
 * @fileoverview pcjsvax-f23 -- drive the PCjs MACHINE XML page in a REAL browser, over CDP, and
 *               grade the OpenVMS boot it produces
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT IT GRADES, AND HOW IT DIFFERS FROM tests/browserboot.mjs
 * ------------------------------------------------------------
 * browserboot.mjs grades browser/vax.html: a hand-written page, fed by two FILE PICKERS driven with
 * DOM.setFileInputFiles.  This grades the PCjs machine definition instead --
 *
 *      http://127.0.0.1:<port>/machines/dec/vax/ka655/
 *
 * -- opened with NO QUERY PARAMETERS AT ALL and NOTHING PICKED.  Everything the machine needs comes
 * from the XML: the ROM from `<rom file="/vaxdisks/ka655x.json5"/>`, the system disk from the RQDX3
 * device XML's `autoMount='{DUA0:{path:"/vaxdisks/vms55-rd54.dsk"}}'`.  That is the whole point of
 * the item: a visitor clicks a link and watches a VAX boot.
 *
 * The page calls embedVAX(), which fetches machine.xml and machines/dec/vax/xsl/components.xsl and
 * transforms one with the other -- PCjs's normal embedding path.  See THE PAGE, below, for why the
 * page is synthesized here rather than read off disk, and why the XSLT polyfill is mandatory.
 *
 * THE GRADE IS THE SAME GRADE, deliberately: a BARE `Username: ` must reach the screen, a username
 * is TYPED, and an answer is required.  See HANDOFF.md section 0's two grading traps -- `Username:`
 * also appears as a PADDED FIELD inside an OPCOM audit record, and `LOGINOUT.EXE` appears only when
 * a login FAILS, so neither is sound on its own.
 *
 * USAGE
 *   node machines/dec/vax/tests/machineboot.mjs [--url PATH] [--chrome PATH] [--port N]
 *                                               [--max-seconds S] [--login USER] [--password PW]
 *
 * The media it loads is whatever /disks/vaxdisks holds -- the gitignored local mirror described by
 * _developer.yml.  Nothing is passed on the command line, because nothing is passed in the URL.
 */

import fs from "fs";
import os from "os";
import path from "path";
import http from "http";
import { spawn, execFileSync } from "child_process";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function findChrome()
{
    let explicit = getArg("--chrome", null);
    if (explicit) return explicit;
    let cache = path.join(os.homedir(), ".cache/ms-playwright");
    if (fs.existsSync(cache)) {
        for (let d of fs.readdirSync(cache).filter((n) => n.startsWith("chromium-")).sort().reverse()) {
            for (let sub of ["chrome-linux64/chrome", "chrome-linux/chrome"]) {
                let p = path.join(cache, d, sub);
                if (fs.existsSync(p)) return p;
            }
        }
    }
    for (let n of ["chromium", "chromium-browser", "google-chrome", "google-chrome-stable"]) {
        try { return execFileSync("which", [n], {encoding: "utf8"}).trim(); } catch (e) {}
    }
    return null;
}

/* ---------------------------------------------------------------------------------------------- *
 * A static file server rooted at the repo.  The CONTENT TYPES here are not decoration: Chrome will *
 * not apply an XSL stylesheet served as application/octet-stream, so .xsl and .xml must be typed   *
 * as XML -- which is exactly what `python3 -m http.server` does, and this mirrors it so the two    *
 * ways of serving the repo behave identically.                                                     *
 * ---------------------------------------------------------------------------------------------- */
const MIME = {".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript",
              ".css": "text/css", ".json": "application/json", ".json5": "application/json",
              ".xml": "application/xml", ".xsl": "application/xslt+xml",
              ".bin": "application/octet-stream", ".gz": "application/gzip"};

/* ---------------------------------------------------------------------------------------------- *
 * THE PAGE.                                                                                        *
 *                                                                                                  *
 * On the real site the page is machines/dec/vax/ka655/README.md and JEKYLL builds it, from the      *
 * `machines:` block in its front matter.  Jekyll is a Ruby toolchain and is not available in this   *
 * checkout, so this reproduces byte-for-byte the two things that block causes Jekyll to emit, and   *
 * NOTHING ELSE -- the header, footer and stylesheets of _layouts/default.html are not part of what  *
 * is being graded:                                                                                  *
 *                                                                                                  *
 *   1. _includes/machine.html, `machine_config contains ".xml"` branch: a placeholder <div> with    *
 *      the machine's id, which embedMachine() then replaces with the transformed machine.           *
 *   2. _includes/machines/scripts.html: the xslt-polyfill, then one `<script type="module">` per    *
 *      entry in machines.json's vax.modules (this machine sets `unbundled: true`, because VAXjs     *
 *      has no gulp bundle yet), then the `embedVAX(...)` call built from `machine_factory`,         *
 *      `machine_id`, `machine_config` and `machine_template`.                                       *
 *                                                                                                  *
 * THE POLYFILL IS NOT OPTIONAL.  Chrome 149 has REMOVED XSLT: `XSLTProcessor is not defined`, and   *
 * an `<?xml-stylesheet?>` processing instruction is ignored (the raw XML renders in the XML         *
 * viewer).  /assets/js/xslt-polyfill.min.js is upstream PCjs's own answer to that, already emitted  *
 * by scripts.html for every machine on the site.                                                    *
 * ---------------------------------------------------------------------------------------------- */
const MACHINE_ID = "vax3900";
const MACHINE_XML = "/machines/dec/vax/ka655/machine.xml";
const MACHINE_XSL = "/machines/dec/vax/xsl/components.xsl";

function buildPage()
{
    let machines = JSON.parse(fs.readFileSync(path.join(REPO_ROOT, "machines/machines.json"), "utf8"));
    let modules = machines["vax"]["modules"].filter((m) => m.indexOf("/embed.js") < 0);
    return `<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>DEC MicroVAX 3900 (KA655) with OpenVMS VAX</title>
<link rel="stylesheet" href="/machines/dec/vax/xsl/common.css">
<link rel="stylesheet" href="/machines/dec/vax/xsl/components.css">
</head><body>
<div id="${MACHINE_ID}" class="machine-placeholder">
  <p>[DEC MicroVAX 3900 (KA655) with 16Mb, RQDX3 and OpenVMS VAX "${MACHINE_ID}"]</p>
  <p class="machine-warning">Waiting for machine "${MACHINE_ID}" to load....</p>
</div>
<script src="/assets/js/xslt-polyfill.min.js"></script>
${modules.map((m) => `<script type="module" src="/${m.replace(/^\.\//, "")}"></script>`).join("\n")}
<script type="module">
  import { embedVAX, globals } from "/machines/modules/v2/embed.js";
  embedVAX('${MACHINE_ID}','${MACHINE_XML}','${MACHINE_XSL}','');
</script>
</body></html>
`;
}

function serve(port)
{
    return new Promise((resolve) => {
        let srv = http.createServer((req, res) => {
            let rel = decodeURIComponent(req.url.split("?")[0]);
            if (rel == "/machines/dec/vax/ka655/" || rel == "/machines/dec/vax/ka655/index.html") {
                let html = buildPage();
                res.writeHead(200, {"Content-Type": "text/html", "Content-Length": Buffer.byteLength(html)});
                res.end(html);
                return;
            }
            let file = path.join(REPO_ROOT, rel);
            if (!file.startsWith(REPO_ROOT) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
                res.writeHead(404); res.end("not found"); return;
            }
            res.writeHead(200, {"Content-Type": MIME[path.extname(file)] || "application/octet-stream",
                                "Content-Length": fs.statSync(file).size});
            fs.createReadStream(file).pipe(res);
        });
        srv.listen(port, "127.0.0.1", () => resolve(srv));
    });
}

class CDP
{
    constructor(ws) { this.ws = ws; this.id = 0; this.waiting = new Map(); this.sessionId = null;
                      ws.onmessage = (e) => this.onMessage(JSON.parse(e.data)); }

    static async connect(url)
    {
        let ws = new WebSocket(url);
        await new Promise((res, rej) => { ws.onopen = res; ws.onerror = () => rej(new Error("CDP connect failed")); });
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
    let page = getArg("--url", "/machines/dec/vax/ka655/");
    let port = parseInt(getArg("--port", "8133"), 10);
    let maxSeconds = parseInt(getArg("--max-seconds", "600"), 10);
    let login = getArg("--login", "SYSTEM");
    let password = getArg("--password", "QUOKKA1953");
    let chrome = findChrome();

    if (page.indexOf("?") >= 0) { console.log(`machineboot: the landing URL must take NO query parameters`); process.exit(2); }
    if (!chrome) { console.log(`machineboot: no Chrome binary found; pass --chrome PATH`); process.exit(2); }

    /* The media this run depends on, named so a missing mirror fails HERE rather than as a silent
       404 twenty seconds into a boot. */
    let media = ["disks/vaxdisks/ka655x.json5", "disks/vaxdisks/vms55-rd54.dsk"];
    for (let m of media) {
        if (!fs.existsSync(path.join(REPO_ROOT, m))) {
            console.log(`machineboot: ${m} is missing -- see machines/dec/vax/ka655/README.md`);
            process.exit(2);
        }
    }
    let volume = path.join(REPO_ROOT, "disks/vaxdisks/vms55-rd54.dsk");
    let volStat = fs.statSync(volume);
    let hashBefore = volStat.size + ":" + volStat.mtimeMs;

    console.log(`machineboot (pcjsvax-f23) -- a PCjs MACHINE XML in a REAL browser, over CDP`);
    console.log(`  chrome    ${chrome}`);
    console.log(`  serving   ${REPO_ROOT} on http://127.0.0.1:${port}`);
    console.log(`  page      ${page}   <-- no query parameters, nothing picked`);
    console.log(`  machine   ${MACHINE_XML} via ${MACHINE_XSL}`);
    console.log(`  media     ${media.join(", ")} (via /disks, gitignored)`);

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
    if (!wsUrl) { console.log(`machineboot: Chrome never opened a DevTools endpoint.\n${chromeErr}`); cleanup(); process.exit(2); }

    let cdp = await CDP.connect(wsUrl);
    let {targetId} = await cdp.send("Target.createTarget", {url: `http://127.0.0.1:${port}${page}`}, null);
    let {sessionId} = await cdp.send("Target.attachToTarget", {targetId, flatten: true}, null);
    cdp.sessionId = sessionId;
    await cdp.send("Runtime.enable");
    await cdp.send("Page.enable");
    await cdp.send("Log.enable");

    let pageErrors = [];
    let consoleLines = [];
    cdp.onMessage = ((orig) => function(msg) {
        if (msg.method === "Runtime.exceptionThrown") {
            pageErrors.push(msg.params.exceptionDetails.exception?.description || msg.params.exceptionDetails.text);
        }
        if (msg.method === "Log.entryAdded") {
            let e = msg.params.entry;
            let line = `${e.level}: ${e.text}${e.url ? " <" + e.url + ">" : ""}`;
            consoleLines.push(line);
            /* A headless Chrome asks for /favicon.ico on every navigation and the repo has none;
               that 404 is not a machine defect and must not be graded as one. */
            if (e.level === "error" && !/favicon\.ico/.test(e.url || "")) pageErrors.push(line);
        }
        return orig.call(this, msg);
    })(cdp.onMessage);

    /* The XSLT transform, the module load and the media fetches all have to happen before there is
       a machine at all; the 159 MB system disk is the long pole.  Poll for the machine rather than
       sleeping a guessed amount. */
    let t0 = Date.now(), booted = false;
    for (let i = 0; i < 300 && !booted; i++) {
        await sleep(1000);
        booted = await cdp.evaluate("!!(window.vaxMachine && window.vaxMachine.flags.powered)");
        if (pageErrors.length) break;
        if (i === 2) console.log(`  title     "${await cdp.evaluate("document.title")}"`);
        if (i % 5 === 4) {
            let desc = await cdp.evaluate("(document.querySelector('.pcjs-description')||{}).textContent||''");
            console.log(`  .. ${((Date.now() - t0) / 1000).toFixed(0)}s  ${desc}`);
        }
    }
    if (!booted) {
        console.log(`machineboot: the machine never powered up after ${((Date.now() - t0) / 1000).toFixed(0)}s`);
        if (pageErrors.length) console.log(`  page errors:\n    ${pageErrors.join("\n    ")}`);
        console.log(`  console:\n    ${consoleLines.slice(-20).join("\n    ")}`);
        cleanup(); process.exit(1);
    }
    console.log(`  powered   after ${((Date.now() - t0) / 1000).toFixed(0)}s: ${await cdp.evaluate("window.vaxStatus()")}`);

    const screen = () => cdp.evaluate("window.vaxTerm ? window.vaxTerm.lines.join('\\n') : ''");
    let tRun = Date.now(), deadline = tRun + maxSeconds * 1000;
    let promptAt = 0, passwordAt = 0, sawBarePrompt = false, sawDCL = false, text = "", lastLog = 0, stop = null;
    while (Date.now() < deadline) {
        await sleep(1000);
        if (pageErrors.length) { stop = "PAGE EXCEPTION"; break; }
        let werr = await cdp.evaluate("window.vaxError");
        if (werr) { stop = `WORKER/MACHINE ERROR: ${werr}`; break; }
        text = await screen();
        if (Date.now() - lastLog >= 15000) {
            lastLog = Date.now();
            console.log(`  .. ${((Date.now() - tRun) / 1000).toFixed(0)}s  ${await cdp.evaluate("window.vaxStatus()")}`);
        }
        /* GRADING TRAP 1 (HANDOFF section 0): `Username:` also appears as a PADDED FIELD inside an
           OPCOM audit record.  The real prompt is a BARE `Username: ` at end of line. */
        if (!sawBarePrompt && /(^|\n)Username: /.test(text) && !/Username: {2,}/.test(text.split("\n").filter((l) => /^Username: /.test(l)).pop() || "")) {
            sawBarePrompt = true; promptAt = Date.now();
            console.log(`  .. bare \`Username: \` on screen after ${((promptAt - tRun) / 1000).toFixed(0)}s; typing "${login}"`);
            await cdp.evaluate(`window.vaxType(${JSON.stringify(login + "\r")})`);
        }
        /* V5.5 answers a username with `Password: `.  Answering THAT and requiring a DCL prompt is
           the strongest available grade, and it is the one HANDOFF section 0 asks for: neither
           `Username:` (which also appears padded inside an OPCOM audit record) nor `LOGINOUT.EXE`
           (which appears only when a login FAILS) proves anything on its own. */
        if (promptAt && !passwordAt && /Password: ?$|Password: *\n/m.test(text)) {
            passwordAt = Date.now();
            console.log(`  .. \`Password: \` on screen after ${((passwordAt - promptAt) / 1000).toFixed(0)}s; answering`);
            await cdp.evaluate(`window.vaxType(${JSON.stringify(password + "\r")})`);
        }
        if (passwordAt && /\$ ?$|\n\$/m.test(text.split("Password:").pop() || "")) { stop = "LOGGED IN TO DCL"; sawDCL = true; break; }
        if (promptAt && Date.now() - promptAt > 90000) { stop = "THE PROMPT NEVER ANSWERED"; break; }
    }
    if (!stop) stop = "WALL-CLOCK CAP";

    let status = await cdp.evaluate("window.vaxStatus()");
    let attach = await cdp.evaluate("(window.vaxMachine.bindings['attach']||{}).textContent||''");
    console.log(`\n  stopped   ${stop} after ${((Date.now() - tRun) / 1000).toFixed(0)}s`);
    console.log(`  status    ${status}`);
    console.log(`  attach    ${attach.replace(/\n/g, "\n            ")}`);
    if (pageErrors.length) console.log(`  page errors:\n    ${pageErrors.join("\n    ")}`);

    let passed = 0, failed = 0;
    const check = (name, cond, detail) => {
        if (cond) { passed++; console.log(`  PASS  ${name}`); }
        else { failed++; console.log(`  FAIL  ${name}${detail ? " -- " + detail : ""}`); }
    };
    console.log("");
    /* What the XSLT is FOR is turning `<computer>`, `<cpu>`, `<rom>`, `<serial>` and `<device>`
       into PCjs component objects carrying their parms in `data-value`.  Counting those is the
       check; matching a word in the page text is not. */
    let objects = await cdp.evaluate("[...document.querySelectorAll('[class*=\"vax-\"][class*=\"-object\"]')].map(e=>e.className).join(',')");
    check("the machine XML rendered through its own XSL into PCjs component objects",
        /vax-computer-object/.test(objects) && /vax-cpu-object/.test(objects) &&
        /vax-rom-object/.test(objects) && /vax-serial-object/.test(objects) && /vax-device-object/.test(objects),
        objects);
    check("no uncaught exception in the page", pageErrors.length === 0, pageErrors[0]);
    check("the ROM loaded from `<rom file=...>` (no ?rom= query parameter)",
        /-byte ROM/.test(await cdp.evaluate("window.vaxMachine.describeConfig()")));
    check("DUA0 auto-mounted from the device XML (no picker, no ?diskgz=)",
        /DUA0 /.test(await cdp.evaluate("window.vaxMachine.describeConfig()")));
    check("the KA655 ROM ran and reached the console prompt `>>>`", />>>/.test(text));
    check("DUA0 attached read/write (copy-on-write overlay, not UNIT_RO)",
        /read\/write via copy-on-write overlay/.test(attach), attach);
    check("OpenVMS printed its banner", /VAX\/VMS (version )?V[0-9]+\.[0-9]+[-0-9A-Z]*/.test(text));
    check("a BARE `Username: ` reached the SCREEN (not the padded OPCOM audit field)", sawBarePrompt);
    check(`the prompt ANSWERED "${login}"`, /Password:|Welcome to (OpenVMS|VAX\/VMS version)/.test(text), stop);
    check(`the login COMPLETED to a DCL prompt`, sawDCL, stop);
    check("the username was ECHOED, i.e. the keyboard path reached the terminal driver",
        new RegExp(`Username: ${login}`).test(text));
    check("the backing image was NOT modified", hashBefore === fs.statSync(volume).size + ":" + fs.statSync(volume).mtimeMs);

    console.log(`\n----- LAST 30 SCREEN LINES -----`);
    console.log(text.split("\n").slice(-30).join("\n"));
    console.log(`\nPASSED ${passed} / FAILED ${failed}`);
    cleanup();
    process.exit(failed ? 1 : 0);
}

main().catch((e) => { console.log("machineboot: " + (e.stack || e)); process.exit(2); });
