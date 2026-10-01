// rd vms-553: boot a REAL OpenVMX/VAX disk in this page and assert its RAW console -- term.lines, the
// bytes the guest wrote, not the painted DOM -- carries no NetBSD substrate output from the KA655
// power-on to the point the VMS personality is up.
//
// Operator ruling (2026-10-01): the console must not show NetBSD/unix boot output, and that is done
// at the SOURCE (an OVMX_QUIET kernel + quiet secondary bootstrap, vms repo
// tools/cross-vax/netbsd-ovmx-quiet.patch), never by filtering text here. The page renders verbatim
// (test-console-verbatim.mjs), so a clean raw console is exactly what a visitor sees.
//
// Headless Chromium + the real emulator + a real disk (~3-6 min). Env:
//   PAGE      full page URL incl. ?rom=...&diskgz=...  (ovmx.html or ovmx-cluster.html)
//   UNTIL     regex the boot must reach before grading (default: Username:)
//   TIMEOUT_S overall cap (default 900)
// Run: PAGE='http://localhost:8301/machines/dec/vax/browser/ovmx.html?rom=ka655x.bin&diskgz=ovmx-vax-v0.7-5.img.gz' \
//        node machines/dec/vax/browser/test-console-raw-quiet.mjs

import { chromium } from "playwright";

const PAGE = process.env.PAGE;
if (!PAGE) { console.error("set PAGE=<ovmx.html or ovmx-cluster.html URL with ?rom=&diskgz=>"); process.exit(2); }
const UNTIL = new RegExp(process.env.UNTIL || "Username:");
const TIMEOUT_S = parseInt(process.env.TIMEOUT_S || "900", 10);

/* The NetBSD substrate's boot, by what it prints: the secondary bootstrap, the kernel copyright and
   version banner, memory sizing, autoconf, root/boot device -- and the kernel's message timestamp,
   which every kernel printf line carries (the executive's operator lines are written without one). */
export const SUBSTRATE_MARKERS = [
    /NetBSD/, /Copyright \(c\)/, /The NetBSD Foundation/, /\bmainbus0\b/, /total memory/,
    /avail memory/, /\broot on\b/, /boot device/, /^\s*\[\s*\d+\.\d+\]\s/,
];

const browser = await chromium.launch({ headless: true });
const page = await (await browser.newContext()).newPage();
page.on("pageerror", (e) => console.log("pageerror:", String(e)));
await page.goto(PAGE, { waitUntil: "load" });
await page.evaluate(() => document.querySelector("#bootbtn, button")?.click?.());

const read = () => page.evaluate(() =>
    window.__clusterNode?.consoleText?.() ?? (window.vaxTerm ? window.vaxTerm.lines.join("\n") : ""));
const t0 = Date.now();
let raw = "", pageErr = null;
while ((Date.now() - t0) / 1000 < TIMEOUT_S) {
    await page.waitForTimeout(3000);
    raw = await read();
    const err = await page.evaluate(() => window.vaxError || window.__clusterNode?.err || null);
    if (err) { pageErr = err; console.log("guest/page error:", err); break; }
    if (UNTIL.test(raw)) break;
}
await browser.close();

const reached = UNTIL.test(raw);
const head = reached ? raw.slice(0, raw.search(UNTIL)) : raw;
const hits = head.split("\n").filter((ln) => SUBSTRATE_MARKERS.some((re) => re.test(ln)));
console.log(`--- RAW console, power-on to ${UNTIL} (${head.length} chars) ---\n${head}\n---`);
let fails = 0;
if (!reached) { console.log(pageErr ? `FAIL  the page errored before ${UNTIL}: ${pageErr}` : `FAIL  the boot never reached ${UNTIL} within ${TIMEOUT_S}s`); fails++; }
if (/KA655/.test(head) === false) { console.log("FAIL  no KA655 power-on in the capture (wrong page?)"); fails++; }
if (hits.length) { console.log(`FAIL  ${hits.length} NetBSD substrate line(s) on the RAW console:`); hits.forEach((h) => console.log("   ! " + h)); fails++; }
else console.log("PASS  no NetBSD substrate output on the RAW console");
process.exit(fails ? 1 : 0);
