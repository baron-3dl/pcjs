/**
 * @fileoverview The KA655 console ROM's terminal query is answered inside the ROM's own waiting
 *               window, on the guest's clock, however slow the host is (rd vms-bb0)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * WHAT THIS GRADES
 * ----------------
 * The KA655 ROM writes `ESC [ c` (DA1) at power-up and waits, for a bounded stretch of guest time,
 * for the terminal's reply.  The public cluster demo's CPU-throttled visitor gate (openvmx-site run
 * 41965964541, all-at-once-throttled2) showed Node B's console at ">>> DUA0 / ?22 ILL CMD": the reply
 * used to come from the page's terminal widget, a main-thread round trip in HOST time, and it
 * arrived after the ROM had given up waiting; the ROM read it as type-ahead during its self-test and
 * then swallowed the "B" of the auto-boot's "B DUA0".  browser/vaxmachine.js now answers on the line
 * itself, in guest time.
 *
 *   1. CONTROL (no answer from the machine; the reply typed 2,500,000 instructions after the query,
 *      as the host path delivered it): the ROM must lose the "B" -- ">>> DUA0", "?22 ILL CMD".  If
 *      it does not, this test no longer reproduces the defect and says so.
 *   2. THE FIX, with nothing typed by any host: for host pump granularity K from 1 instruction to a
 *      fast host's 20,000-instruction batch, idle elision off and on, the ROM reads all eight bytes
 *      of the reply BEFORE it prints its banner (i.e. while it is still waiting for them), and then
 *      accepts the auto-boot command: "(BOOT/R5:0 DUA0)", never "ILL CMD".
 *
 *      node machines/dec/vax/tests/consolequery.test.mjs
 */

import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";
import { VaxMachine } from "../browser/vaxmachine.js";
import { DA_RESPONSE } from "../browser/vaxterm.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const rom = new Uint8Array(fs.readFileSync(path.join(__dirname, "../browser/ka655x.bin")));
const MAX_STEPS = 20_000_000;
const LATE = 2_500_000;

/**
 * Run the ROM with the auto-boot rules on, pumping input every K instructions.  `late` (or null):
 * the machine does not answer, and the reply is typed `late` instructions after the query appears.
 */
function run(K, idle, late)
{
    const log = console.log;
    console.log = () => {};                         /* rommachine.js's "Bus: Added ..." chatter */
    const m = new VaxMachine({ romBytes: rom, disk: null, autoBoot: true, idle });
    console.log = log;
    if (late !== null) m.daResponse = null;
    const dev = m.consoleDev, rdRd = dev.rxdbRd.bind(dev);
    let reads = "", readBeforeBanner = "";
    dev.rxdbRd = () => {
        const v = rdRd();
        reads += String.fromCharCode(v);
        if (!Buffer.from(dev.output).toString("latin1").includes("KA655")) readBeforeBanner += String.fromCharCode(v);
        return v;
    };
    let queryAt = -1, typed = false, stop = null;
    for (let s = 0; s < MAX_STEPS; s++) {
        try { m.cpu.stepCPU(1); } catch (e) { stop = e.reason || String(e); break; }
        m.steps++;
        if ((s + 1) % K) continue;
        if (queryAt < 0 && Buffer.from(dev.output).toString("latin1").includes("\x1b[c")) queryAt = m.cpu.nTotalCycles;
        if (late !== null && queryAt >= 0 && !typed && m.cpu.nTotalCycles >= queryAt + late) { m.type(DA_RESPONSE); typed = true; }
        m.pumpInput();
        if (m.fired.has("console-prompt") && !m.pending.length && m.steps - m.marks[0].atStep > 3_000_000) break;
    }
    const out = Buffer.from(dev.output).toString("latin1").replace(/\r/g, "");
    const after = out.includes(">>>") ? out.slice(out.indexOf(">>>")) : "";
    return {queried: queryAt >= 0, readBeforeBanner, after, stop,
            booted: after.includes("(BOOT/R5:0 DUA0)") && !after.includes("ILL CMD")};
}

let fails = 0;
const report = (ok, what) => { console.log(`${ok ? "PASS" : "FAIL"}  ${what}`); if (!ok) fails++; };

{
    const r = run(60, true, LATE);
    report(r.queried && r.after.startsWith(">>> DUA0") && r.after.includes("?22 ILL CMD"),
           `control: reply typed ${LATE} instructions late loses the "B": ${JSON.stringify(r.after.slice(0, 30))}`);
}
for (const [K, idle] of [[1, false], [60, false], [20000, false], [1, true], [64, true], [20000, true]]) {
    const r = run(K, idle, null);
    const ok = r.queried && r.readBeforeBanner.endsWith(DA_RESPONSE) && r.booted;
    report(ok, `pump every ${K} instruction(s)${idle ? ", idle on" : ""}: ROM read ${JSON.stringify(r.readBeforeBanner)} ` +
               `before its banner; ${JSON.stringify(r.after.slice(0, 26))}${r.stop ? ` (stop: ${r.stop})` : ""}`);
}
process.exit(fails ? 1 : 0);
