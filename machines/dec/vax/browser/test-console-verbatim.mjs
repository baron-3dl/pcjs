// rd vms-553: the OpenVMX/VAX console is shown VERBATIM -- no display-side filtering anywhere.
//
// Operator ruling (2026-10-01): the console must not show the NetBSD substrate's boot, and that is
// done at the SOURCE ("don't print it out from the kernel"), never by filtering text in the demo.
// The disk this page boots carries an OVMX_QUIET kernel and a quiet secondary bootstrap (vms repo,
// tools/cross-vax/netbsd-ovmx-quiet.patch); what reaches the console is what the visitor sees.
// The render-time substrate-hide that used to stand in for that (rd vms-1e2: VaxTerminal.hideRange
// + ovmx.html's updateHideRange) is gone. This test pins that it stays gone:
//
//   1. VaxTerminal paints EVERY line it is given -- including one that names NetBSD -- so if a
//      substrate line ever does reach the guest console, the page shows it (honest), instead of
//      quietly hiding it (the thing the ruling forbids). That is what makes the raw-console check
//      (test-console-raw-quiet.mjs) meaningful: the screen IS the raw console.
//   2. No page or the terminal carries a hide/filter mechanism (hideRange, substrate-hide markers).
//
// Pure node, no browser.  Run: node machines/dec/vax/browser/test-console-verbatim.mjs

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import { VaxTerminal } from "./vaxterm.js";

const here = dirname(fileURLToPath(import.meta.url));
let fails = 0;
const check = (name, ok) => { console.log(`${ok ? "PASS" : "FAIL"}  ${name}`); if (!ok) fails++; };

const el = { tagName: "PRE", textContent: "", scrollTop: 0, scrollHeight: 0, addEventListener() {} };
const term = new VaxTerminal(el, () => {});
const stream = [
    "KA655-B V5.3, VMB 2.7", ">>>B DUA0", "(BOOT/R5:0 DUA0)",
    ">> NetBSD/vax boot [1.12] <<",                 /* would have been hidden by the old filter */
    "[   1.0000000] mainbus0 (root)",
    "%OVMX-I-EXEC, VMS executive attached on /dev/vms",
    "    OpenVMX V0.7-5 - OpenVMS-compatible",
    "Username: ",
].join("\r\n");
term.write(new TextEncoder().encode(stream));
term.dirty = true; term.render();
const painted = el.textContent;
for (const ln of stream.split("\r\n"))
    check(`painted verbatim: ${JSON.stringify(ln.trim())}`, painted.includes(ln.trim()));
check("terminal has no hideRange", !("hideRange" in term));

const SOURCES = ["vaxterm.js", "ovmx.html", "ovmx-cluster.html"];
for (const f of SOURCES) {
    const src = readFileSync(join(here, f), "utf8");
    check(`${f}: no display-side hide/filter (hideRange / RE_HIDE_START / updateHideRange)`,
        !/hideRange|RE_HIDE_START|RE_REVEAL|updateHideRange/.test(src));
}

console.log(fails ? `\n${fails} FAILED` : "\nALL PASS");
process.exit(fails ? 1 : 0);
