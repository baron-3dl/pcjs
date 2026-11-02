/**
 * @fileoverview The auto-typed boot command reaches the KA655 console ROM intact however often the
 *               host pumps console input (rd vms-2a97)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * WHAT THIS GRADES
 * ----------------
 * browser/vaxmachine.js's pumpInput() runs once per slice, and a slice's length in instructions is
 * set by the HOST: ~20,000 instructions per batch on a fast machine, a few dozen on a slow or
 * CPU-throttled one (a slice also ends early whenever the guest idles).  The public cluster demo's
 * throttled visitor gate saw Node B's console print ">>> DUA0" and "?22 ILL CMD": characters of the
 * auto-typed "B DUA0<CR>" were lost, because the next queued character was injected as soon as the
 * ROM had read the last one, while it was still echoing it.  A real console line cannot do that --
 * SIMH's TTI delivers at most one character per 9600-baud character time of GUEST time.
 *
 * Here the KA655 ROM (no disk) is run to its >>> prompt with the auto-boot rules on, and the input
 * pump is called every K instructions for K from 1 (the slowest host imaginable) to a fast host's
 * batch, with guest-idle elision off and on.  For every K the ROM must accept the command: it echoes "(BOOT/R5:0 DUA0)", the console
 * ROM's own acknowledgement of a parsed BOOT command, and never "?22 ILL CMD".  Before the fix,
 * every K <= 60 failed (the RETURN was lost and the ROM sat at "B DUA0").
 *
 *      node machines/dec/vax/tests/consolepace.test.mjs
 */

import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";
import { VaxMachine } from "../browser/vaxmachine.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const rom = new Uint8Array(fs.readFileSync(path.join(__dirname, "../browser/ka655x.bin")));

/* [K, idle]: the tab runs with guest-idle elision ON (vaxmachine.js's default), the 34-check gate off. */
const CASES = [[1, false], [7, false], [60, false], [1000, false], [20000, false], [1, true], [64, true]];
const MAX_STEPS = 20_000_000;

let fails = 0;
const log = console.log;
for (const [K, idle] of CASES) {
    console.log = () => {};                         /* rommachine.js's "Bus: Added ..." chatter */
    const m = new VaxMachine({ romBytes: rom, disk: null, autoBoot: true, idle });
    console.log = log;
    let s = 0, stop = null;
    for (; s < MAX_STEPS; s++) {
        try { m.cpu.stepCPU(1); } catch (e) { stop = e.reason || String(e); break; }
        m.steps++;
        if ((s + 1) % K === 0) m.pumpInput();
        if (m.fired.has("console-prompt") && !m.pending.length && m.steps - m.marks[0].atStep > 3_000_000) break;
    }
    const out = Buffer.from(m.consoleDev.output).toString("latin1").replace(/\r/g, "");
    const after = out.slice(out.indexOf(">>>"));
    const ok = out.includes(">>>") && after.includes("(BOOT/R5:0 DUA0)") && !after.includes("ILL CMD");
    log(`${ok ? "PASS" : "FAIL"}  pump every ${K} instruction(s)${idle ? ", idle on" : ""}: ${JSON.stringify(after.slice(0, 40))}${stop ? ` (stop: ${stop})` : ""}`);
    if (!ok) fails++;
}
process.exit(fails ? 1 : 0);
