/**
 * @fileoverview Differential test: the CONSOLE BYTE STREAM the KA655 ROM emits on this machine vs.
 *               the byte stream a real Open SIMH microvax3900 emits over the same boot
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-bfb DONE CONDITION 1, and nothing else.  Conditions 2-6 -- the console REGISTER model,
 * both address paths, both directions, TTI/TTO at IPL 0x14 -- are tests/consoledif.js's, and were
 * met before this file existed.  What no check in this gate measured until now is the EMITTED BYTE
 * STREAM: consoledif.js grades register ACCESS, romdiff.js grades INSTRUCTIONS, and a machine can
 * pass both while writing the wrong bytes, or no bytes, to TXDB.  This file closes that hole and
 * only that hole.
 *
 * THE MEASUREMENT: run the ROM on this machine, collect every byte written to TXDB, run the ROM on
 * a real SIMH, collect every byte its console emits, and compare them BYTE FOR BYTE.
 *
 * WHY THIS EXISTS AS A SEPARATE INSTRUMENT FROM romdiff.js
 * ---------------------------------------------------------
 * romdiff.js is bounded at 249,743 instructions and cannot be raised: its comparison needs SIMH's
 * instruction-history ring, SIMH refuses a ring larger than HIST_MAX (250000, vax_defs.h:852), and a
 * ring that WRAPS writes one unfilled-result record per pass (vax_cpu.c:1676) that is
 * indistinguishable from a real divergence.
 *
 * MEASURED, on BOTH engines, at romdiff's EXACT ceiling: 249,743 instructions produce ZERO console
 * bytes.  So does 260,000, and so does 280,000.  The first two bytes appear by 300,000 and the
 * banner is not complete until past 1,200,000.  Done condition 1 therefore lies entirely beyond
 * romdiff's ceiling, and no value of --max-steps could ever have reached it -- which is why this
 * item's console output had never been measured despite romdiff running green on every gate.
 *
 * This file needs no history ring, because it compares OUTPUT rather than instructions -- so its
 * budget is bounded only by wall clock, and it walks an order of magnitude further than romdiff can.
 * That is the entire reason the two instruments are separate.  It also means this file makes NO
 * per-instruction claim: two engines whose byte streams agree may still have diverged internally,
 * and romdiff.js remains the only thing that grades that.  Neither file subsumes the other.
 *
 * HOW THE COMPARED RANGE IS DECIDED -- MEASURED, NOT DECLARED
 * ------------------------------------------------------------
 * A ROM self-test countdown is driven by timers whose rate differs between the two engines (this
 * machine's clk.js is instruction-count driven; SIMH's is a wall-clock-calibrated event queue), and
 * the ROM's own error dumps print live timer values.  So the oracle's byte stream is NOT
 * reproducible all the way to the end, and grading against the part that is not reproducible would
 * manufacture divergences exactly the way a wrapped history ring does.
 *
 * Rather than declare a cut-off, this file MEASURES one.  PHASE O captures the oracle from
 * INDEPENDENT SIMH processes and PHASE R derives:
 *
 *   REPRODUCIBLE ORACLE PREFIX = the longest common prefix of two same-budget captures.
 *
 * with a second, independent claim from a SHORTER capture: within that reproducible range, a
 * shorter run's stream must agree with a longer run's byte for byte.  If it does not, the oracle's
 * early bytes depend on how far it was allowed to run, and no offset into it means anything.  Both
 * are asserted, not assumed.
 *
 * The comparison in PHASE C is truncated to the reproducible prefix.  Nothing past it is graded, and
 * the report says so.
 *
 * THE ORACLE'S BUDGET IS SEARCHED FOR, NOT ASSUMED -- AND THIS IS WHY
 * --------------------------------------------------------------------
 * MEASURED, and the reason an earlier revision of this file was intermittent: at a FIXED step
 * budget the oracle's output LENGTH varies between runs on the same host.  Two 3,000,000-step
 * captures produced 465 and 178 bytes when this was first written; RE-MEASURED 2026-07-29 the
 * effect is unchanged -- two 4,000,000-step captures gave 481 and 194 bytes on one run and 481 and
 * 481 on the next, so the variance is in the RUN and not in any particular budget.  Contrast PHASE
 * S, which measures the same question on THIS machine and finds a fixed 1,407-byte length with six
 * bytes of one wall-clock field moving inside it: the two engines are nondeterministic in
 * different quantities, and that asymmetry is what bounds the graded range.
 * SIMH's interval timer is a wall-clock-CALIBRATED event
 * (sim_activate against a polled real-time rate), so how many timer interrupts land inside a fixed
 * instruction count depends on how fast the host executed them, and the ROM's self-tests are paced
 * by those interrupts.  Anything downstream of the first timer-paced test is therefore reproducible
 * in CONTENT but not in LENGTH.
 *
 * So the budget is not guessed.  ORACLE_SEED_STEPS is DELIBERATELY set below the point where the
 * ROM emits anything at all, and the capture budget DOUBLES until the oracle has produced
 * BANNER_MATCH_FLOOR_LINES complete lines -- the same "ask for what you need and raise the budget
 * until you have it" shape romdiff.js's captureSimhTrace() uses for history records.  The floor is
 * then a property of the OUTPUT, not of any step count, and a slower or faster host changes only
 * how many doublings it takes.
 *
 * THE JS BUDGET IS ALSO MEASURED, NOT DECLARED -- PHASE S (pcjsvax-e29)
 * ----------------------------------------------------------------------
 * The oracle's budget is searched for; for a long time THIS machine's was a constant justified by
 * a sentence, and the sentence went stale.  It claimed the JS stream was byte-identical at
 * 3,000,000 / 6,000,000 / 20,000,000 / 60,000,000 instructions.  cmctl.js and cqipc.js then landed
 * and carried the ROM further, and by 2026-07-29 the 3,000,000-step default truncated the stream at
 * byte 395 -- mid-countdown -- while this file went on exiting 0 and naming a first diverging byte.
 * Three self-test failures this machine emits past there (?51, ?46, ?80) were invisible to it.
 * That is HANDOFF.md standing rule 13 -- a gate that stops measuring is worse than one that fails --
 * arriving in romdiff.js's own replacement, and it is why the budget is now an assertion.
 *
 * PHASE S walks the machine THREE times: twice at the budget in force and once at
 * JS_SETTLE_MULTIPLE times it.  The same-budget pair DERIVES which byte offsets are wall-clock
 * -derived (two walks over identical instructions can differ only there); the cross-budget pair
 * must then match everywhere else, at the same length.  See JS_SETTLE_MULTIPLE's own comment for
 * why two walks are not enough and why the derived exemption is widened to whole hex fields.
 *
 * PHASE S bounds what this file can SEE.  It does not touch what this file GRADES -- that is still
 * the oracle's reproducible prefix and nothing beyond it.  PHASE U reports the rest and asserts
 * nothing about it, which is the honest split: the ROM's later output is real and worth printing,
 * and a single oracle capture cannot grade it.
 *
 * THE FLOOR IS STRUCTURAL, NOT A BYTE COUNT
 * -------------------------------------------
 * BANNER_MATCH_FLOOR_LINES is a number of COMPLETE CRLF-TERMINATED LINES of the ORACLE'S OWN
 * output that the two streams must agree over.  The byte count that corresponds to it is looked up
 * in the live oracle capture at run time, so this file never contains, and never needs to know, what
 * the ROM actually prints.  That is deliberate and is the item's own "HOW AN AGENT WOULD CHEAT
 * THIS": the banner must be a CONSEQUENCE of the ROM executing against this machine's registers,
 * never an input to this code.  There is no expected text anywhere in this file.
 *
 * The floor is an ABSOLUTE constant and does not scale with any budget or case count (HANDOFF.md
 * standing rule 4).  A smaller --js-steps or --oracle-steps does not lower it; it just fails.
 *
 * WHAT THIS FILE REPORTS THAT NOTHING ELSE CAN
 * ----------------------------------------------
 * A DEVICE CENSUS over the same walk (PHASE D).  HANDOFF.md §3 recorded "the devices are decoded,
 * not exercised", which was measured true over the first 392 instructions and had never been
 * re-measured since.  Every register-facing method of every device the machine builder constructs
 * is wrapped and counted -- the method list is derived from each device's own prototype and the
 * device list from tests/rommachine.js's own construction statements, so neither can be
 * hand-enumerated wrongly (HANDOFF.md standing rule 5).  The census also CROSS-CHECKS the byte
 * stream: the number of ConsoleVAX.txdbWr calls must equal the number of bytes captured, which is
 * two independent measurements of the same event and fails if either one is blind.
 *
 *      node machines/dec/vax/tests/conoutdiff.js [options]
 *        --simh PATH         patched microvax3900; else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch
 *                             build (the same search romdiff.js uses)
 *        --rom PATH          default $PCJS_VAX_REPO/open-simh/VAX/ka655x.bin
 *        --js-steps N        instructions to execute on THIS machine (default JS_STEPS_DEFAULT).
 *                             NOT a free knob: PHASE S re-walks at JS_SETTLE_MULTIPLE times
 *                             whatever is given here and FAILS if the stream is still growing, so
 *                             a budget too small to settle is rejected rather than obeyed
 *        --oracle-steps N    where the oracle's budget SEARCH starts, not a fixed budget -- the
 *                             search doubles from here (default ORACLE_SEED_STEPS; see the header)
 *        --selfcheck         prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import { VAX } from "../modules/v2/defines.js";
import { VAXStop } from "../modules/v2/cpustate.js";
import ConsoleVAX from "../modules/v2/console.js";
import { makeRomMachine } from "./rommachine.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/* ------------------------------------------------------------------------------------------- *
 * Bounds.  Every one of these FAILS the run when it is not met, and none of them scales with a    *
 * budget or a case count (HANDOFF.md standing rule 4).                                            *
 * ------------------------------------------------------------------------------------------- */

/** Instructions to execute on THIS machine.

    THE BUDGET IS ASSERTED BY THE RUN, NOT BY THIS COMMENT (pcjsvax-e29).  Whatever value is in
    force -- this default or a --js-steps override -- PHASE S below re-walks the machine at
    JS_SETTLE_MULTIPLE times it and FAILS unless the stream has demonstrably stopped growing.  Read
    that assertion as the authority and the numbers here as the record of one run; a budget
    justified only by a sentence is exactly what this file used to carry.  (The comparison is not
    bare byte-identity, and it cannot be -- see JS_SETTLE_MULTIPLE for the measurement that says
    why, and for the third walk that makes it deterministic.)

    WHAT THE OLD COMMENT CLAIMED, AND WHY IT IS GONE.  It asserted this machine's stream was
    "byte-identical at 3,000,000 / 6,000,000 / 20,000,000 / 60,000,000 instructions".  That was true
    when pcjsvax-bfb wrote it; cmctl.js and cqipc.js landed afterwards and carried the ROM further,
    and by 2026-07-29 it was false -- 3,000,000 steps produce 395 bytes and 6,000,000 produce 1,407.
    Everything past byte 395 was invisible while this file went on exiting 0 and naming a
    first-diverging byte, which is HANDOFF.md standing rule 13 happening to romdiff.js's own
    replacement.  Three self-test failures this machine emits and the oracle does not -- ?51, ?46
    and ?80 -- lived in that gap, and no assertion here could see them.

    RE-MEASURED 2026-07-29 on 818b7468f, one walk per budget from a fresh machine:

        200,000 / 249,743 / 260,000 / 280,000 ->     0 bytes  (romdiff's ceiling reaches nothing)
                                    300,000  ->     2 bytes
                                  1,200,000  ->     5 bytes
                                  3,000,000  ->   395 bytes  <- the ORIGINAL default, mid-countdown
                                  6,000,000  -> 1,407 bytes  <- settled AT THE TIME; see below

    RAISED AGAIN 2026-07-29 by pcjsvax-5c1, and PHASE S is what forced it rather than a guess.  That
    item decoded the CQBIC's Qbus memory window, which removed the ROM's `?80` self-test failure --
    so the ROM stopped bailing out early, ran its countdown to completion, and the stream moved
    again.  PHASE S failed the gate the same day with "6,000,000 produce 487 byte(s) but 12,000,000
    produce 546", which is precisely the job it was written to do: the budget is not allowed to go
    stale silently a second time.  Re-measured on the same tree:

                                  3,000,000  ->   395 bytes
                                  6,000,000  ->   487 bytes  <- NO LONGER SETTLED
                     12,000,000 / 20,000,000 ->   546 bytes  <- settled; runs 40..03 and reaches ">>>"

    (The stream is SHORTER than the old 1,407 bytes because three self-test failure dumps -- ?51,
    ?46 and ?80 -- are gone.  A smaller number here is progress, which is exactly why this file
    grades content and structure rather than a byte count; see HANDOFF.md section 3.)

    First console byte at instruction #281,966.  ~2.4s at this budget, ~10s including PHASE S. */
const JS_STEPS_DEFAULT = 12000000;

/** PHASE S walks this multiple of whatever budget is in force, on a machine built from scratch.
    Expressed as a MULTIPLE rather than as a second absolute constant on purpose: a --js-steps
    override is then held to the same standard instead of escaping it, so "3,000,000 is enough"
    cannot be re-asserted from the command line any more than it can from a comment.

    WHY IT TAKES THREE WALKS AND NOT TWO, which is the whole subtlety of this check.  The obvious
    form -- walk at N, walk at 2N, demand the streams be identical -- FAILS ON A CORRECT MACHINE,
    and measuring that is what this item's own first attempt did.  MEASURED 2026-07-29 over four
    walks: the stream is 1,407 bytes EVERY time, but exactly six byte offsets vary -- 295-297 and
    321-323, the low hex digits of r2 and r4 in the ?53 register dump (r4 == r2 + 10).  They vary
    between two walks at the SAME budget just as much as across budgets, because clk.js's todrRd()
    outside ROM is a deliberate, graded port of a SYNCHRONOUS host-clock read (see clk.js's header
    and tests/timerdiff.js).  A bare identity assertion would therefore have been a permanently red
    check, and "fix" it by widening a tolerance and it stops measuring anything.

    So the exempt set is DERIVED, never declared (HANDOFF.md standing rule 5 -- never hand-enumerate
    a scope list).  Two walks at the SAME budget execute identical instructions, so every offset
    that differs between them is wall-clock-derived BY CONSTRUCTION; that measured set is subtracted
    from the cross-budget comparison, and anything left over is real budget dependence.  The machine
    cannot know its own budget, so content that moves with the budget and not with the clock means
    the shorter walk was observing a different machine state -- which is precisely the defect this
    item exists to catch.  MAX_VOLATILE_OFFSETS then keeps the derived exemption from growing into a
    blanket.

    The asymmetry with the oracle is worth stating, because it is what earns the right to compare a
    single JS capture against a two-capture oracle prefix: the oracle varies in LENGTH at a fixed
    budget (PHASE O/R measured 481 and 194 bytes from two 4,000,000-step processes), while this
    machine varies only in the VALUE of one wall-clock field at a fixed length. */
const JS_SETTLE_MULTIPLE = 2;

/** Where the oracle budget SEARCH starts (--oracle-steps overrides it).  DELIBERATELY below the
    point where the ROM emits anything: MEASURED, 249,743 steps -- romdiff.js's exact ceiling --
    produce zero console bytes, and so do 260,000 and 280,000.  Starting under that floor means the
    search must observe "not enough output" at least once before it can succeed, so a search that
    silently never iterates would be visible in the report rather than invisible. */
const ORACLE_SEED_STEPS = 250000;

/** Hard ceiling on the doubling search.  A search that reaches this without producing the required
    lines FAILS and says so; it never quietly grades a short capture. */
const ORACLE_SEARCH_MAX_STEPS = 128000000;

/** The full captures (PHASE R's two reproducibility captures) run this multiple of the budget the
    search settled on.  Slack so the reproducible prefix is bounded by the oracle's own real-time
    dependence rather than by a capture that stopped mid-line. */
const ORACLE_FULL_MULTIPLE = 2;

/** Complete CRLF-terminated lines of the ORACLE'S OWN output that the two streams must agree over.
    THE FLOOR IS THE ITEM'S SCOPE BOUNDARY, NOT THE MEASUREMENT: pcjsvax-bfb owns the ROM's power-on
    output up to and including its "normal system tests" announcement, which is the 4th complete line
    the ROM writes, and explicitly does NOT own the self-test countdown that follows it (that belongs
    to the timer items).  The byte count is looked up in the live oracle capture, never written here
    (see the file header).

    MEASURED WHEN THIS FILE WAS WRITTEN (pcjsvax-bfb): the two engines agreed over 68 bytes, and the
    4th line ends at byte 64 -- 4 bytes of headroom, deliberate slack, because a floor set AT the
    measurement is a tripwire rather than a floor.

    RE-MEASURED after pcjsvax-b8a decoded the CQBIC doorbell: they now agree over the WHOLE
    reproducible oracle prefix -- through the `40..39..…31..` countdown and into the `?53` dump the
    oracle itself produces -- and the ROM no longer reports `?91`.  Three runs on the same host gave
    104, 115 and 115 matching bytes, differing only because the ORACLE's reproducible prefix differed
    (104, 295 and 167 bytes); this machine's own stream was byte-identical across all three OVER THE
    GRADED REGION.

    THAT LAST CLAUSE IS A CORRECTION, and it is this file's own instance of HANDOFF.md standing rule
    12.  The sentence used to end "this machine's own stream was byte-identical across all three",
    unqualified, and that is FALSE and was false when it was written: MEASURED 2026-07-29 by
    pcjsvax-e29, six bytes of every capture vary run to run -- the r2/r4 hex fields at 290-297 and
    316-323 in the ?53 dump, which are wall-clock-derived (PHASE S).  Those offsets were inside the
    395-byte stream the 3,000,000-step budget produced at the time, so the unqualified claim was
    already wrong; it survived because the GRADED region has never reached byte 290, which is
    exactly the shape rule 12 describes -- the measurement was right and the sentence about it
    claimed more than the measurement did.

    THE FLOOR IS DELIBERATELY NOT RAISED TO MATCH, and that is a measurement rather than caution: the
    5th complete line (the countdown) ends at byte 106 and the 6th at byte 108, so a floor of 5 or 6
    lines would have FAILED the 104-byte run.  Past the first timer-paced test the oracle is
    reproducible in CONTENT and not in LENGTH -- exactly what the file header says -- so a line floor
    out there is an intermittent test, which is worse than no floor at all.  What holds the
    doorbell's gain instead is tests/dbldiff.js, which grades the register itself rather than the
    ROM's reaction to it. */
const BANNER_MATCH_FLOOR_LINES = 4;

/** ABSOLUTE ceiling on how many byte offsets may differ between two walks at the SAME budget --
    PHASE S's wall-clock exemption.  See the check itself for why it must stay small. */
const MAX_VOLATILE_OFFSETS = 32;

/** ABSOLUTE peak-heap ceiling, in MB (HANDOFF.md standing rule 14).  RE-MEASURED 2026-07-29 with
    PHASE S's three walks in place: 21.1 / 21.2 / 21.2 MB over three consecutive runs.  It was
    17.5 MB with one walk at this budget, and the "~13 MB" this comment used to claim was measured
    at the old 3,000,000-step budget -- three walks at twice the budget cost about 8 MB more than
    one walk at the old one, which is well inside the ceiling and is why the ceiling does not move.
    Printed on every run so drift is visible long before it is fatal. */
const PEAK_HEAP_MB_MAX = 256;

/** Distinct bus-fault addresses whose detail is retained for the report.  The COUNTS below stay
    exact; only the retained detail is capped, and the report says when it was (standing rule 6). */
const MAX_REPORTED_FAULT_ADDRS = 32;

/** SCP `echo` sentinels that bracket the oracle's console stream in SIMH's stdout -- see
    OracleCapture.extract().  Deliberately unlikely to occur in any ROM output or SCP message. */
const SENTINEL_BEGIN = "<<<PCJS-CONOUT-BEGIN>>>";
const SENTINEL_END = "<<<PCJS-CONOUT-END>>>";

/** The trailing SCP message every bounded run ends with -- "Step expired, PC: X (dis)", "HALT
    instruction, PC: X (dis)", "Breakpoint, PC: X (dis)".  scp.c prints all of them with a LEADING
    newline of its own, which is why the newline is part of this pattern and is removed with it:
    it is SCP's, not the ROM's.  Anchored at the end of the bracketed region, so a run that stopped
    in some other shape does not silently contribute SCP text to the graded byte stream -- it fails
    and reports what SIMH actually said. */
const STOP_MSG_RE = /\n[A-Z][^\n]*, PC: [0-9A-Fa-f]+ \([^\n]*\)\n$/;

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

/** The largest `heapUsed` OBSERVED at this file's sample points -- after the JS walk, after each
    oracle capture, and at the end.  A sampled maximum, not a continuous one, and not claimed to be
    RSS or a true peak between samples. */
let g_peakHeapMB = 0;
function sampleHeap()
{
    let mb = process.memoryUsage().heapUsed / (1024 * 1024);
    if (mb > g_peakHeapMB) g_peakHeapMB = mb;
    return g_peakHeapMB;
}

/* ------------------------------------------------------------------------------------------- *
 * Plumbing -- the same shape romdiff.js and consoledif.js use, deliberately.  These four helpers   *
 * are duplicated rather than imported because romdiff.js calls main() at module scope, so importing *
 * anything from it would run the whole ROM gate as a side effect.  What is NOT duplicated is the    *
 * machine itself: that lives in rommachine.js and is shared (see its header).                       *
 * ------------------------------------------------------------------------------------------- */

function vaxRepo()
{
    if (process.env['PCJS_VAX_REPO']) return process.env['PCJS_VAX_REPO'];
    return path.resolve(__dirname, "../../../../../pcjs-vax");
}

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    for (let v of ['SIMH_CPU_BIN', 'SIMH_INT_BIN', 'SIMH_DECODE_BIN', 'SIMH_BIN']) {
        if (process.env[v]) candidates.push(process.env[v]);
    }
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    candidates.push(path.join(vaxRepo(), "open-simh/BIN/microvax3900"));
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error(
        "conoutdiff needs a REAL SIMH; it has no fixture fallback.  Build one with\n" +
        "machines/dec/vax/tests/simh/build.sh and pass --simh PATH or set $SIMH_CPU_BIN.  Tried:\n  " +
        candidates.join("\n  "));
}

function findRom(pathArg)
{
    let p = pathArg || path.join(vaxRepo(), "open-simh/VAX/ka655x.bin");
    if (!fs.existsSync(p)) throw new Error(`ka655x.bin not found at ${p}; pass --rom PATH`);
    return p;
}

/**
 * runSimh(bin, script, iniPath, timeoutMs)
 *
 * `latin1`, NOT `utf8`: this is the one place in this tree whose subject is RAW BYTES.  The ROM's
 * first console byte is ESC (0x1B) and its output is not UTF-8; decoding as UTF-8 would replace
 * every byte above 0x7F with U+FFFD and silently destroy the very thing being compared.  latin1 is
 * a byte-preserving round trip for all 256 values.
 *
 * @param {string} bin
 * @param {string} script
 * @param {string} iniPath
 * @param {number} [timeoutMs]
 * @returns {string} SIMH's stdout, one JS char per byte
 */
function runSimh(bin, script, iniPath, timeoutMs = 5 * 60 * 1000)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "latin1", maxBuffer: 1 << 29, timeout: timeoutMs});
}

/**
 * PHYSMEM_RANGES / nameAddress(addr) -- resolved from VAX.PHYSMEM itself, never hand-listed
 * (HANDOFF.md standing rule 5), used only to NAME a faulting address in the report.  Same
 * derivation romdiff.js uses, for the same reason and with the same output format, so the two
 * files' reports name the same address the same way.
 */
const PHYSMEM_RANGES = (function() {
    let out = [];
    for (let k in VAX.PHYSMEM) {
        if (!k.endsWith("_BASE")) continue;
        let stem = k.slice(0, -5);
        let len = VAX.PHYSMEM[stem + "_LENGTH"] || VAX.PHYSMEM[stem + "_SIZE"] || 0;
        out.push({name: stem, base: VAX.PHYSMEM[k] >>> 0, length: len >>> 0});
    }
    return out;
})();

function nameAddress(addr)
{
    addr = addr >>> 0;
    for (let r of PHYSMEM_RANGES) {
        if (addr >= r.base && addr < r.base + (r.length || 1)) {
            return `${r.name}+0x${(addr - r.base).toString(16).toUpperCase()} (${hex(addr)})`;
        }
    }
    return hex(addr);
}

/** Printable rendering of a byte range, for the report only.  Never parsed, never compared. */
function show(bytes, from, len)
{
    let out = "";
    for (let i = from; i < Math.min(bytes.length, from + len); i++) {
        let b = bytes[i];
        if (b === 0x0D) out += "\\r";
        else if (b === 0x0A) out += "\\n";
        else if (b === 0x1B) out += "\\e";
        else if (b >= 0x20 && b < 0x7F) out += String.fromCharCode(b);
        else out += "\\x" + hex(b, 2);
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 0 -- sys_model BY EXECUTION, never by assumption                                          *
 * ------------------------------------------------------------------------------------------- */

/**
 * querySysModel(simh, opts)
 *
 * vax_sysdev.c cpu_boot():1729 writes `sys_model ? 1 : 2` to ROMBASE+4.  Asked of the SAME oracle
 * binary this file grades against rather than assumed, exactly as romdiff.js's own querySysModel()
 * does -- this file cannot import that one (see the plumbing note above), and re-deriving it costs
 * one ~50ms SIMH invocation, which is cheaper than depending on another test having run.
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {{sysModel: number, magicByte: number}}
 */
function querySysModel(simh, opts)
{
    let out = runSimh(simh, "examine MODEL\nexit\n", path.join(opts.scratch, "conout-model.ini"));
    let m = /MODEL:\s*([0-9A-Fa-f]+)/.exec(out);
    if (!m) throw new Error("conoutdiff: could not read SIMH's MODEL register; SIMH said:\n" + out);
    let sysModel = parseInt(m[1], 16);
    return {sysModel, magicByte: sysModel ? 1 : 2};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE D -- the device census.  Installed on the SAME machine PHASE J walks, so it measures the   *
 * run being graded rather than a second one.                                                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * Census -- per (device class, method) call counts.
 *
 * `note()` is a plain object property so selfcheck() can PERTURB the shipped counting path
 * (HANDOFF.md standing rule 11) rather than substitute a copy of it.
 */
const Census = {
    note(tally, key) { tally.set(key, (tally.get(key) || 0) + 1); },

    /**
     * install(machine)
     *
     * Wraps EVERY function on each device's own prototype except its constructor.  The device list
     * comes from rommachine.js's `devices` (built by the same statements that construct the
     * machine) and the method list from each device's own prototype -- so neither is hand-enumerated
     * and neither can drift from the machine (HANDOFF.md standing rule 5).  The wrapper is installed
     * as an OWN property of the instance, shadowing the prototype method for this machine only; no
     * prototype is mutated, so a second machine in the same process is unaffected.
     *
     * @param {Object} machine as returned by makeRomMachine()
     * @returns {Map} tally, keyed "ClassName.methodName"
     */
    install(machine) {
        let tally = new Map();
        for (let {name, dev} of machine.devices) {
            let proto = Object.getPrototypeOf(dev);
            for (let m of Object.getOwnPropertyNames(proto)) {
                if (m === "constructor") continue;
                let d = Object.getOwnPropertyDescriptor(proto, m);
                if (!d || typeof d.value !== "function") continue;
                let orig = d.value, key = `${name}.${m}`;
                dev[m] = function(...args) { Census.note(tally, key); return orig.apply(this, args); };
            }
        }
        return tally;
    }
};

/* ------------------------------------------------------------------------------------------- *
 * PHASE J -- this machine's console byte stream                                                   *
 * ------------------------------------------------------------------------------------------- */

/**
 * Walk.budget(opts) -- the JS step budget, as its own object property so selfcheck() can perturb
 * the SHIPPED value.  The "js-budget-reverted-to-romdiff-ceiling" mutation exists because the whole
 * reason this file is a separate instrument is that romdiff's ceiling cannot reach the first
 * console byte; a mutation that silently reinstates that ceiling must be caught.
 */
const Walk = {
    budget(opts) { return opts.jsSteps; }
};

/**
 * runJS(romBytes, opts, magicByte)
 *
 * Boots the shared ROM machine exactly as romdiff.js does and executes Walk.budget() instructions,
 * collecting ConsoleVAX's `output` -- which console.js appends to inside txdb_wr()'s port of
 * vax_stddev.c:328-335, at the instant of the write, with nothing above it inspecting or gating on
 * the value.  This function reads that array AFTER the run and never before, so no byte can reach
 * the machine from this harness.
 *
 * Bus faults are counted per (address, access) for the report.  They are NOT graded here: grading a
 * fault against the oracle is romdiff.js's FaultGrader's job and duplicating it here is exactly the
 * drift HANDOFF.md standing rule 7 warns about.  What this function contributes is the ADDRESSES,
 * at step numbers romdiff's history-ring ceiling cannot reach.
 *
 * `budgetOverride` is how PHASE S walks a LONGER budget without going through Walk.budget().  That
 * matters for more than convenience: the "js-budget-truncated-mid-stream" mutation perturbs
 * Walk.budget() to a constant, and a settle walk that re-derived its own budget from the same
 * perturbed seam would return the SAME number, make both walks identical, and certify a truncation
 * it was written to catch.  PHASE S therefore multiplies the shipped seam's answer ONCE and passes
 * the product down here.  The seam is still the shipped one and is still what the mutation
 * perturbs (HANDOFF.md standing rule 11); what is not allowed is for the mutation to perturb both
 * sides of the comparison symmetrically, which is rule 16's "the verifier's own read was unsound"
 * in miniature.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 * @param {number} [budgetOverride]
 * @returns {Object}
 */
function runJS(romBytes, opts, magicByte, budgetOverride)
{
    let machine = makeRomMachine(romBytes);
    let {cpu, consoleDev} = machine;
    let tally = Census.install(machine);
    cpu.reset();
    cpu.boot(magicByte);

    let faults = new Map(), faultEvents = 0;
    /* `...rest` is load-bearing, not tidiness: onBusFault() takes a THIRD argument (pcjsvax-622's
       `fNoBto`, the handler-raised machine check that must not set the SSC bus-timeout bits -- see
       cpustate.js).  A wrapper that forwarded only the two it names would silently change the
       machine being measured, and would change it ONLY while instrumented, which is the worst
       possible place for a difference to live. */
    let realOnBusFault = cpu.onBusFault.bind(cpu);
    cpu.onBusFault = function(addr, access, ...rest) {
        let key = `${hex(addr >>> 0)}:${(access & VAX.ACCESS.WRITE) ? "W" : "R"}`;
        let e = faults.get(key);
        if (!e) faults.set(key, e = {addr: addr >>> 0, access, count: 0, firstStep: steps + 1});
        e.count++;
        faultEvents++;
        return realOnBusFault(addr, access, ...rest);
    };

    let steps = 0, stop = null, budget = budgetOverride || Walk.budget(opts);
    try {
        while (steps < budget) { cpu.stepCPU(1); steps++; }
    } catch (e) {
        if (!(e instanceof VAXStop)) throw e;
        stop = `${e.reason} detail=${e.detail}`;
    }
    return {
        bytes: Uint8Array.from(consoleDev.output), steps, budget, stop, tally, faults, faultEvents,
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0,
        /* The names of every device the machine ACTUALLY constructed, carried out of here because
           the census tally alone cannot answer "which devices were never touched": a method that is
           never called is never recorded, so an untouched device has NO tally entry to notice.  An
           earlier revision of this file computed "devices with ZERO calls" from the tally and could
           therefore only ever print "(none)" -- a line that claimed a measurement it structurally
           could not make (HANDOFF.md standing rule 12). */
        deviceNames: machine.devices.map((d) => d.name),
        txdbWrCalls: tally.get(`${consoleDev.constructor.name}.txdbWr`) || 0,
        consoleClass: consoleDev.constructor.name
    };
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE O -- the oracle's console byte stream                                                     *
 * ------------------------------------------------------------------------------------------- */

/**
 * OracleCapture -- a REAL `boot cpu`, bounded by a breakpoint at the ROM entry and then advanced by
 * an EXACT step count, with the console stream bracketed by SCP `echo` sentinels.
 *
 * Every member is a plain object property so selfcheck() can perturb the shipped extraction path.
 */
const OracleCapture = {

    /**
     * raw(simh, opts, steps, tag)
     *
     * Why a real `boot cpu` and not a hand-built deposit: scp.c's run_cmd() calls the real
     * cpu_boot() and then falls into the free-running loop with sim_step forced to 0, so BOOT
     * cannot be given a step count -- a breakpoint set before it bounds it at the first
     * instruction, and `step N` then runs an EXACT number.  romdiff.js's header records the same
     * reasoning at length; this file reproduces the mechanism, not the trace capture, because it
     * needs no history ring (see this file's header).
     *
     * NO `set cpu history=` HERE, deliberately.  That is what frees this capture from HIST_MAX.
     *
     * @param {string} simh
     * @param {Object} opts
     * @param {number} steps
     * @param {string} tag distinguishes the .ini files of concurrent captures in the scratch dir
     * @returns {string} SIMH's raw stdout, latin1, one char per byte
     */
    raw(simh, opts, steps, tag) {
        let script = [
            "set cpu 16m",
            `load -r ${opts.rom}`,
            `break ${hex(VAX.PHYSMEM.ROM_BASE >>> 0)}`,
            "boot cpu",
            `nobreak ${hex(VAX.PHYSMEM.ROM_BASE >>> 0)}`,
            `echo ${SENTINEL_BEGIN}`,
            `step ${steps}`,
            `echo ${SENTINEL_END}`,
            "exit", ""
        ].join("\n");
        let out = runSimh(simh, script, path.join(opts.scratch, `conout-${tag}.ini`), 10 * 60 * 1000);
        if (!/Breakpoint, PC:/.test(out)) {
            throw new Error(`conoutdiff: SIMH never stopped at the ROM entry ` +
                `${hex(VAX.PHYSMEM.ROM_BASE >>> 0)} after 'boot cpu' -- the capture has no defined ` +
                `starting point; SIMH said:\n` + out);
        }
        return out;
    },

    /**
     * extract(raw)
     *
     * The console byte stream, and ONLY it, out of SIMH's stdout.
     *
     * SIMH writes the simulated console (sim_putchar -> sim_os_putchar, sim_console.c:2943) and its
     * own SCP messages (sim_printf) to the SAME stream, and `SET CONSOLE LOG` does not separate them
     * (it mirrors sim_log, which carries both -- MEASURED, not assumed).  So the stream is bracketed
     * instead: SCP's own `echo` writes a sentinel line immediately before and immediately after the
     * `step`, and the only SCP text that can land between them is the ONE stop message the step
     * itself produces, which is removed by STOP_MSG_RE (see that constant for why its leading
     * newline is SCP's and goes with it).
     *
     * Both sentinels must occur EXACTLY ONCE and the stop message must be present and last.  Every
     * one of those is a hard failure rather than a fallback: a bracket that silently mis-locates
     * would feed SCP text into a byte-for-byte comparison, which is the one thing this file must
     * never do.
     *
     * @param {string} raw latin1 stdout
     * @returns {Uint8Array} the console bytes
     */
    extract(raw) {
        let occurrences = (s) => {
            let n = 0, i = -1;
            while ((i = raw.indexOf(s, i + 1)) >= 0) n++;
            return n;
        };
        for (let s of [SENTINEL_BEGIN, SENTINEL_END]) {
            let n = occurrences(s);
            if (n !== 1) {
                throw new Error(`conoutdiff: the sentinel ${s} occurs ${n} times in SIMH's output, ` +
                    `expected exactly 1 -- the console stream cannot be bracketed unambiguously.  ` +
                    `SIMH said:\n${raw.slice(0, 2000)}`);
            }
        }
        let iB = raw.indexOf(SENTINEL_BEGIN) + SENTINEL_BEGIN.length;
        if (raw[iB] !== "\n") {
            throw new Error(`conoutdiff: SCP's echo of ${SENTINEL_BEGIN} was not newline-terminated ` +
                `(next char is ${JSON.stringify(raw[iB])}); the bracket's start is not where this ` +
                `file believes it is.`);
        }
        let region = raw.slice(iB + 1, raw.indexOf(SENTINEL_END));
        let m = STOP_MSG_RE.exec(region);
        if (!m) {
            throw new Error(`conoutdiff: the bracketed region does not end in an SCP stop message, so ` +
                `SIMH did not stop the way this capture requires and SCP text would otherwise be graded ` +
                `as console output.  Region tail was: ${JSON.stringify(region.slice(-200))}`);
        }
        let text = region.slice(0, m.index);
        let bytes = new Uint8Array(text.length);
        for (let i = 0; i < text.length; i++) bytes[i] = text.charCodeAt(i) & 0xFF;
        return bytes;
    }
};

/**
 * ExtractClaim.verify(bytes)
 *
 * extract() does not get to be its own witness either (see PrefixClaim below for the same argument
 * applied to the comparison).  The one thing that must be true of every extracted stream, whatever
 * the run did, is that it carries NO SCP text: if a stop message survived the trim, SCP's words are
 * about to be compared byte-for-byte against a ROM's output.  Checked directly on the result rather
 * than inferred from the fact that extract() tried.
 *
 * @param {Uint8Array} bytes
 * @param {string} tag
 * @returns {?string} a problem, or null
 */
const ExtractClaim = {
    verify(bytes, tag) {
        let s = "";
        for (let i = 0; i < bytes.length; i++) s += String.fromCharCode(bytes[i]);
        if (STOP_MSG_RE.test(s)) {
            return `oracle capture "${tag}" still ends in an SCP stop message after extraction ` +
                `(${JSON.stringify(s.slice(-64))}) -- SCP's own words would be graded as ROM output`;
        }
        return null;
    }
};

/**
 * crlfEnds(bytes)
 *
 * The byte offset just PAST each complete CRLF-terminated line.  The single scanner both
 * countLines() and lineFloorBytes() are expressed in: the floor is stated in lines and enforced in
 * bytes, and two independent scans of the same thing are exactly the drift HANDOFF.md standing rule
 * 7 exists for -- one could count a line the other did not and the floor would then mean two
 * different things in the same run.
 *
 * @param {Uint8Array} bytes
 * @returns {number[]}
 */
function crlfEnds(bytes)
{
    let out = [];
    for (let i = 0; i + 1 < bytes.length; i++) {
        if (bytes[i] === 0x0D && bytes[i + 1] === 0x0A) { out.push(i + 2); i++; }
    }
    return out;
}

/** Complete CRLF-terminated lines in a byte stream -- the unit BANNER_MATCH_FLOOR_LINES is
    expressed in, and the unit the oracle budget search asks for. */
function countLines(bytes) { return crlfEnds(bytes).length; }

/* ------------------------------------------------------------------------------------------- *
 * PHASE R / PHASE C -- reproducibility, then the comparison                                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * Compare / PrefixClaim.
 *
 * `commonPrefix` is the one computation the entire verdict rests on, so it does NOT get to be its
 * own witness.  `PrefixClaim.verify()` re-checks the answer against the two arrays directly: the
 * claimed prefix must actually be equal on both sides, and the byte AT the claimed length must
 * actually differ when both arrays are longer than it.  A commonPrefix that overstates its answer
 * -- the failure mode that would turn this whole file green and blind -- violates one or the other.
 * Both are plain object properties so selfcheck() can perturb either independently.
 */
const Compare = {
    commonPrefix(a, b) {
        let n = Math.min(a.length, b.length), i = 0;
        while (i < n && a[i] === b[i]) i++;
        return i;
    }
};

/**
 * Diff.offsets(a, b) -- EVERY offset at which two equal-or-unequal-length streams differ, not just
 * the first.  PHASE S needs the whole set rather than a prefix length, because its question is not
 * "where do these diverge" but "WHICH bytes move, and are they the same ones that move when only
 * the wall clock has changed".  A first-difference answer cannot express that.
 *
 * Offsets past the shorter stream's end are reported too, so a length change is visible as offsets
 * rather than being silently outside the comparison.
 */
const Diff = {
    offsets(a, b) {
        let out = [], n = Math.max(a.length, b.length);
        for (let i = 0; i < n; i++) {
            if (i >= a.length || i >= b.length || a[i] !== b[i]) out.push(i);
        }
        return out;
    },

    /**
     * Diff.expandToFields(bytes, offsets)
     *
     * Widens each differing offset to the WHOLE hexadecimal field containing it.
     *
     * THIS IS THE DIFFERENCE BETWEEN A DETERMINISTIC CHECK AND A FLAKY ONE, and it was measured
     * rather than foreseen: PHASE S's first form derived its exempt set as "the offsets that
     * differed between two same-budget walks", and that set is a SAMPLE, not the field.  The
     * varying quantity is a host-clock value printed as 8 hex digits -- 7BAB348B vs 7BAB34FC
     * differs in two digits, 7BAB348B vs 7BAB3569 in three -- so which offsets move depends on how
     * much real time passed between the two walks being compared.  A cross-budget pair can
     * therefore move a digit the same-budget pair happened not to, and the check fails at random.
     * It did, on the very next run: "2 offset(s) that a same-budget repeat does NOT touch".
     *
     * HANDOFF.md standing rule 17: a mechanism by which the test can fail is a bug report about the
     * test, and the remedy is to close it by CONSTRUCTION rather than to sample more walks or widen
     * a tolerance -- both of those make it rarer and neither makes it impossible.  A field that
     * demonstrably moves with the wall clock is untrustworthy in EVERY digit, so the exemption is
     * the field.
     *
     * The exemption stays narrow in the way that matters: a field is exempt only where a SAME-BUDGET
     * repeat already proved it moves with the clock alone.  Budget dependence anywhere else is still
     * caught byte for byte.  Budget dependence INSIDE such a field is masked -- correctly, because
     * that field is a live clock reading and is ungradeable by construction, which is the same
     * reason PHASE C refuses to grade past the oracle's reproducible prefix.
     */
    expandToFields(bytes, offsets) {
        const isHex = (c) => (c >= 0x30 && c <= 0x39) || (c >= 0x41 && c <= 0x46) || (c >= 0x61 && c <= 0x66);
        let set = new Set();
        for (let i of offsets) {
            if (i >= bytes.length || !isHex(bytes[i])) { set.add(i); continue; }
            let a = i, b = i;
            while (a > 0 && isHex(bytes[a - 1])) a--;
            while (b + 1 < bytes.length && isHex(bytes[b + 1])) b++;
            for (let k = a; k <= b; k++) set.add(k);
        }
        return [...set].sort((x, y) => x - y);
    },

    /** "295-297, 321-323" -- contiguous runs, so the report shows FIELDS rather than a byte list. */
    ranges(offsets) {
        if (!offsets.length) return "(none)";
        let out = [], start = offsets[0], prev = offsets[0];
        for (let v of offsets.slice(1)) {
            if (v !== prev + 1) { out.push([start, prev]); start = v; }
            prev = v;
        }
        out.push([start, prev]);
        return out.map(([a, b]) => (a === b ? `${a}` : `${a}-${b}`)).join(", ");
    }
};

const PrefixClaim = {
    verify(a, b, n) {
        if (n < 0 || n > Math.min(a.length, b.length)) {
            return `the claimed common prefix ${n} is outside [0, ${Math.min(a.length, b.length)}]`;
        }
        for (let i = 0; i < n; i++) {
            if (a[i] !== b[i]) {
                return `the claimed common prefix is ${n} bytes, but the two streams already differ ` +
                    `at byte ${i} (0x${hex(a[i], 2)} vs 0x${hex(b[i], 2)}) -- the comparison is ` +
                    `overstating agreement, which is the one failure that would make this gate blind`;
            }
        }
        if (n < a.length && n < b.length && a[n] === b[n]) {
            return `the claimed common prefix is ${n} bytes, but byte ${n} is EQUAL on both sides ` +
                `(0x${hex(a[n], 2)}) -- the comparison stopped short of the real divergence`;
        }
        return null;
    }
};

/**
 * lineFloorBytes(oracleBytes, lines)
 *
 * The byte offset just past the Nth complete CRLF-terminated line of the ORACLE'S OWN output.  This
 * is how BANNER_MATCH_FLOOR_LINES becomes a byte count without this file ever containing the ROM's
 * text (see the file header).  Returns -1 when the oracle did not produce that many complete lines,
 * which is itself a failure -- the oracle, not this machine, then did not get far enough.
 *
 * @param {Uint8Array} oracleBytes
 * @param {number} lines
 * @returns {number}
 */
function lineFloorBytes(oracleBytes, lines)
{
    let ends = crlfEnds(oracleBytes);
    return lines >= 1 && ends.length >= lines ? ends[lines - 1] : -1;
}

/**
 * OUTCOME_KINDS -- every shape the comparison is allowed to end in.  `fatal` is whether the run
 * also FAILS.  Derived from this table at the report site, never hand-listed there, so a new
 * outcome cannot be added without deciding whether it passes (HANDOFF.md standing rule 13: a gate
 * that stops measuring is worse than one that fails, and "exit 0 having named nothing" is exactly
 * that).
 */
const OUTCOME_KINDS = {
    "identical":       {fatal: false, label: "IDENTICAL over the whole reproducible range"},
    "js-is-prefix":    {fatal: false, label: "THIS MACHINE'S STREAM IS A BYTE-EXACT PREFIX of the oracle's"},
    "diverged":        {fatal: false, label: "FIRST DIVERGING BYTE NAMED"},
    "short-of-floor":  {fatal: true,  label: "AGREEMENT SHORTER THAN THE FLOOR"}
};

/**
 * OutcomeReported.check(c) -- the successor to romdiff.js's StopReported, same invariant: a run must
 * always NAME what it found, and an unnamed or unknown outcome is not a pass.
 */
const OutcomeReported = {
    check(c) {
        if (!c.kind) return `the comparison produced NO named outcome at all`;
        if (!OUTCOME_KINDS[c.kind]) return `the comparison produced an unknown outcome "${c.kind}"`;
        if (!c.why) return `the comparison named outcome "${c.kind}" with no evidence attached`;
        return null;
    }
};

/**
 * compare(js, oracleReproducible, floorBytes)
 *
 * @param {Uint8Array} js
 * @param {Uint8Array} oracleReproducible
 * @param {number} floorBytes
 * @returns {Object}
 */
function compare(js, oracleReproducible, floorBytes)
{
    let n = Compare.commonPrefix(js, oracleReproducible);
    let problems = [];
    let bad = PrefixClaim.verify(js, oracleReproducible, n);
    if (bad) problems.push(bad);

    let kind, why;
    if (n < floorBytes) {
        kind = "short-of-floor";
        why = `the two streams agree over only ${n} byte(s), short of the ${floorBytes}-byte floor ` +
            `(${BANNER_MATCH_FLOOR_LINES} complete lines of the oracle's own output)` +
            (n < js.length && n < oracleReproducible.length
                ? `; first difference at byte ${n}: this machine wrote 0x${hex(js[n], 2)}, the oracle ` +
                  `wrote 0x${hex(oracleReproducible[n], 2)}`
                : `; this machine emitted ${js.length} byte(s) and the oracle ${oracleReproducible.length}`);
    } else if (n === js.length && n === oracleReproducible.length) {
        kind = "identical";
        why = `both engines emitted the same ${n} byte(s), with no divergence inside the oracle's ` +
            `reproducible range`;
    } else if (n === js.length) {
        kind = "js-is-prefix";
        why = `this machine emitted ${n} byte(s), every one identical to the oracle's, and then ` +
            `stopped; the oracle went on to emit ${oracleReproducible.length - n} more`;
    } else {
        kind = "diverged";
        why = `the two streams are identical over the first ${n} byte(s) and first differ at byte ` +
            `${n}: this machine wrote 0x${hex(js[n], 2)}, the oracle wrote ` +
            `0x${hex(oracleReproducible[n], 2)}`;
    }
    return {n, kind, why, problems, floorBytes};
}

/* ------------------------------------------------------------------------------------------- *
 * The run                                                                                         *
 * ------------------------------------------------------------------------------------------- */

/**
 * runAll(romBytes, opts, magicByte, simh, rawCache)
 *
 * Every phase, in order, returning the problems it found rather than exiting -- so selfcheck() can
 * run the identical path under a mutation and require it to fail.
 *
 * `rawCache` lets selfcheck() reuse the oracle's RAW stdout across mutations.  It caches the SIMH
 * PROCESS OUTPUT, not the extracted stream, so the extraction and comparison mutations still run
 * against real oracle bytes and are still caught.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 * @param {string} simh
 * @param {?Map} rawCache
 * @param {boolean} [fQuiet]
 * @returns {Object} {problems, ...}
 */
function runAll(romBytes, opts, magicByte, simh, rawCache, fQuiet = false)
{
    let problems = [];
    let say = (s) => { if (!fQuiet) console.log(s); };

    /* ---- PHASE J ---- */
    let js = runJS(romBytes, opts, magicByte);
    sampleHeap();

    /* ---- PHASE S: the budget is a MEASUREMENT, not a declaration (pcjsvax-e29) ---- */
    let jsRepeat = runJS(romBytes, opts, magicByte);
    sampleHeap();
    let settleSteps = Walk.budget(opts) * JS_SETTLE_MULTIPLE;
    let jsSettle = runJS(romBytes, opts, magicByte, settleSteps);
    sampleHeap();

    /* The SAME-budget pair derives this machine's own volatile offsets by measurement.  Two walks
       at one budget execute the identical instructions, so anything that differs between them is
       wall-clock-derived by construction and cannot be budget dependence. */
    let volatileOffsets = Diff.expandToFields(js.bytes, Diff.offsets(js.bytes, jsRepeat.bytes));
    let settleDiff = Diff.offsets(js.bytes, jsSettle.bytes);
    let budgetDependent = settleDiff.filter((i) => !volatileOffsets.includes(i));
    let settled = (js.bytes.length === jsSettle.bytes.length && !budgetDependent.length);

    if (js.bytes.length !== jsRepeat.bytes.length) {
        problems.push(`two walks at the SAME budget (${js.steps}) produced ${js.bytes.length} and ` +
            `${jsRepeat.bytes.length} bytes.  The stream's LENGTH is not reproducible on this machine, ` +
            `so no offset into it means anything and the volatile-offset set below cannot be derived`);
    } else if (js.bytes.length !== jsSettle.bytes.length) {
        problems.push(`the JS budget has NOT been shown to be large enough: ${js.steps} instruction(s) ` +
            `produce ${js.bytes.length} byte(s) but ${jsSettle.steps} produce ${jsSettle.bytes.length}.  ` +
            `The stream is STILL GROWING at the budget, so everything past it is invisible to this file ` +
            `while it goes on exiting 0 -- HANDOFF.md standing rule 13.  Raise --js-steps until this ` +
            `assertion holds; do NOT lower JS_SETTLE_MULTIPLE`);
    } else if (budgetDependent.length) {
        let i = budgetDependent[0];
        problems.push(`the ${js.steps}-instruction and ${jsSettle.steps}-instruction streams are the same ` +
            `LENGTH but differ at ${budgetDependent.length} offset(s) that a same-budget repeat does NOT ` +
            `touch, first at byte ${i} (0x${hex(js.bytes[i], 2)} vs 0x${hex(jsSettle.bytes[i], 2)}).  ` +
            `That is BUDGET DEPENDENCE, not the wall clock: the machine cannot know its own budget, so ` +
            `content that changes with it means the shorter walk is observing a different machine state`);
    }

    /* An ABSOLUTE bound (HANDOFF.md standing rule 4): it does not scale with the budget or with the
       stream length.  MEASURED 2026-07-29: the raw differing offsets are 295-297 and 321-323, which
       expand to the two 8-digit fields 290-297 and 316-323 -- r2 and r4 in the ?53 register dump,
       where r4 == r2 + 10.  Sixteen offsets, two fields, one wall-clock quantity.  If dozens of
       bytes start moving, something else in this machine became nondeterministic and must be
       DIAGNOSED rather than exempted -- an exemption derived from measurement still has to be small
       enough that it cannot quietly become a blanket. */
    if (volatileOffsets.length > MAX_VOLATILE_OFFSETS) {
        problems.push(`${volatileOffsets.length} byte offset(s) differ between two walks at the SAME ` +
            `budget, over the absolute ceiling of ${MAX_VOLATILE_OFFSETS}.  PHASE S exempts same-budget ` +
            `variation from the settle comparison because it is wall-clock-derived; an exemption this ` +
            `large would exempt real divergence with it`);
    }

    /* ---- PHASE O ---- */
    let capture = (steps, tag) => {
        let key = `${steps}:${tag}`;
        if (rawCache && rawCache.has(key)) return rawCache.get(key);
        let raw = OracleCapture.raw(simh, opts, steps, tag);
        if (rawCache) rawCache.set(key, raw);
        sampleHeap();
        return raw;
    };
    let extract = (steps, tag) => {
        let b = OracleCapture.extract(capture(steps, tag));
        let e = ExtractClaim.verify(b, `${tag}@${steps}`);
        if (e) problems.push(e);
        return b;
    };

    /* THE BUDGET SEARCH (see the file header): double until the oracle has produced the lines the
       floor is expressed in, rather than assuming any step count reaches them on this host. */
    let searchSteps = opts.oracleSeed, searched = [], oShort = null;
    for (;;) {
        oShort = extract(searchSteps, `s${searched.length}`);
        searched.push({steps: searchSteps, bytes: oShort.length, lines: countLines(oShort)});
        if (countLines(oShort) >= BANNER_MATCH_FLOOR_LINES) break;
        if (searchSteps >= ORACLE_SEARCH_MAX_STEPS) {
            problems.push(`the oracle produced only ${countLines(oShort)} complete line(s) at ` +
                `${searchSteps} steps, the search ceiling -- it never got far enough for the ` +
                `${BANNER_MATCH_FLOOR_LINES}-line floor to exist in it, so this machine was not graded`);
            break;
        }
        searchSteps = Math.min(searchSteps * 2, ORACLE_SEARCH_MAX_STEPS);
    }
    let fullSteps = searchSteps * ORACLE_FULL_MULTIPLE;
    let oA = extract(fullSteps, "a");
    let oB = extract(fullSteps, "b");

    /* ---- PHASE R: what part of the oracle is reproducible at all ---- */
    let repro = Compare.commonPrefix(oA, oB);
    let bad = PrefixClaim.verify(oA, oB, repro);
    if (bad) problems.push(`oracle reproducibility: ${bad}`);
    let oracleRepro = oA.subarray(0, repro);

    /* ---- the floor, looked up in the live oracle, never written here ---- */
    let floorBytes = lineFloorBytes(oracleRepro, BANNER_MATCH_FLOOR_LINES);
    if (floorBytes < 0) {
        problems.push(`two independent ${fullSteps}-step oracle captures agree over only ${repro} ` +
            `byte(s), which does not contain ${BANNER_MATCH_FLOOR_LINES} complete CRLF-terminated ` +
            `lines -- the oracle is not reproducible far enough for the floor to exist in it, so ` +
            `nothing was compared`);
        floorBytes = repro + 1;             // unreachable, so the comparison below reports short-of-floor
    }

    /* The independent monotonicity claim, stated only over the range anything is claimed over: a
       SHORTER oracle run's stream must agree with a LONGER one's, byte for byte, everywhere both
       the short run and the reproducible range reach.  It is deliberately NOT "the short run is a
       prefix of the long one" -- past the reproducible prefix the oracle disagrees with ITSELF (see
       the file header), so demanding agreement out there would be demanding the impossible and
       would make this check intermittent rather than true. */
    let monoNeed = Math.min(oShort.length, repro);
    let monoN = Compare.commonPrefix(oShort, oA);
    if (monoN < monoNeed) {
        problems.push(`the ${searchSteps}-step oracle capture (${oShort.length} bytes) disagrees with ` +
            `the ${fullSteps}-step one INSIDE the reproducible range: they first differ at byte ` +
            `${monoN} (0x${hex(oShort[monoN], 2)} vs 0x${hex(oA[monoN], 2)}), short of the ${monoNeed} ` +
            `bytes both reach.  The oracle's early output depends on its budget, so no offset into it ` +
            `can be trusted`);
    }

    /* ---- PHASE C ---- */
    let c = compare(js.bytes, oracleRepro, floorBytes);
    problems.push(...c.problems);
    let unnamed = OutcomeReported.check(c);
    if (unnamed) problems.push(unnamed);
    if (OUTCOME_KINDS[c.kind] && OUTCOME_KINDS[c.kind].fatal) problems.push(c.why);

    /* ---- PHASE D: the census, and its cross-check against the captured stream ---- */
    if (js.txdbWrCalls !== js.bytes.length) {
        problems.push(`the device census counted ${js.txdbWrCalls} ${js.consoleClass}.txdbWr call(s) but ` +
            `${js.bytes.length} byte(s) were captured -- two independent measurements of the same event ` +
            `disagree, so at least one of them is blind`);
    }
    if (js.bytes.length === 0) {
        problems.push(`this machine wrote NOTHING to TXDB in ${js.steps} instruction(s) -- the ROM's ` +
            `output routine never completed a character`);
    }

    sampleHeap();
    if (g_peakHeapMB > PEAK_HEAP_MB_MAX) {
        problems.push(`peak sampled JS heap ${g_peakHeapMB.toFixed(1)} MB exceeds the absolute ceiling ` +
            `${PEAK_HEAP_MB_MAX} MB (HANDOFF.md standing rule 14)`);
    }

    /* ---- the report ---- */
    say(`\nPHASE J -- this machine`);
    say(`  instructions executed : ${js.steps} of ${js.budget}${js.stop ? ` (stopped: ${js.stop})` : ""}`);
    say(`  final state           : PC=${hex(js.pc)} PSL=${hex(js.psl)}`);
    say(`  bytes written to TXDB : ${js.bytes.length}`);
    say(`  bus faults            : ${js.faultEvents} event(s) at ${js.faults.size} distinct address(es)`);
    {
        let listed = 0;
        for (let f of js.faults.values()) {
            if (listed++ >= MAX_REPORTED_FAULT_ADDRS) break;
            say(`     ${(f.access & VAX.ACCESS.WRITE) ? "write" : "read "} ${nameAddress(f.addr)} ` +
                `x${f.count}, first at instruction #${f.firstStep}`);
        }
        if (js.faults.size > listed) say(`     (${js.faults.size - listed} further address(es) not listed)`);
    }
    say(`  PHASE S same-budget   : ${jsRepeat.steps} instruction(s) -> ${jsRepeat.bytes.length} byte(s), ` +
        `${volatileOffsets.length} volatile offset(s)` +
        (volatileOffsets.length ? ` at ${Diff.ranges(volatileOffsets)} (wall-clock-derived)` : ""));
    say(`  PHASE S settle walk   : ${jsSettle.steps} instruction(s) -> ${jsSettle.bytes.length} byte(s), ` +
        `${settled ? "SETTLED -- same length, and nothing differs that the wall clock does not already move"
                   : "NOT SETTLED -- see FAILURES below"}`);

    say(`\nPHASE O/R -- the oracle`);
    for (let s of searched) {
        say(`  budget search         : ${String(s.steps).padStart(10)} steps -> ${String(s.bytes).padStart(4)} ` +
            `bytes, ${s.lines} complete line(s)${s.lines >= BANNER_MATCH_FLOOR_LINES ? "  <- settled here" : ""}`);
    }
    say(`  capture A (${fullSteps} steps) : ${oA.length} bytes`);
    say(`  capture B (${fullSteps} steps) : ${oB.length} bytes  [independent SIMH process]`);
    say(`  monotonicity          : the ${searchSteps}-step capture agrees with A over ${monoN} of the ` +
        `${monoNeed} bytes both reach`);
    say(`  REPRODUCIBLE PREFIX   : ${repro} bytes -- nothing past this is graded`);
    say(`  floor from the oracle : ${BANNER_MATCH_FLOOR_LINES} complete lines = ${floorBytes} bytes`);

    say(`\nPHASE C -- byte-for-byte`);
    say(`  ${OUTCOME_KINDS[c.kind] ? OUTCOME_KINDS[c.kind].label : "(UNNAMED)"}: ${c.why}`);
    say(`  matched  : "${show(js.bytes, 0, Math.min(c.n, 256))}"`);
    if (c.kind === "diverged") {
        say(`  this way : "${show(js.bytes, c.n, 48)}"`);
        say(`  oracle   : "${show(oracleRepro, c.n, 48)}"`);
    }

    /* ---- PHASE U: REPORT ONLY, and the distinction is the point (pcjsvax-e29) ---- */
    say(`\nPHASE U -- what this machine emits PAST the oracle's reproducible prefix.  NOT GRADED.`);
    {
        /* This section asserts NOTHING and must not.  The graded range is bounded by what two
           independent oracle captures agree on, and widening it to chase bytes the oracle cannot
           reproduce would manufacture divergences exactly the way a wrapped history ring does --
           that is pcjsvax-e29's own done-condition 4, and PHASE C stays untouched by this block.

           What was wrong before was not the graded range but the VISIBILITY: at the old 3,000,000
           budget this machine's stream stopped at byte 395, mid-countdown, and three self-test
           failures it emits past there were invisible while the run exited 0.  Printing the tail
           costs nothing and is the difference between "the instrument is silent about this" and
           "the instrument reports this and does not grade it".  Which of these the ORACLE emits is
           measured elsewhere, over many runs (pcjsvax-486 / 877 / aa5); this file cannot answer it
           from one capture and does not try. */
        let codes = [];
        let text = Buffer.from(js.bytes).toString("latin1");
        for (let m of text.matchAll(/\?(\d\d)\b/g)) codes.push({code: m[1], at: m.index});
        say(`  this machine's COMPLETE stream : ${js.bytes.length} byte(s), of which the first ${repro} ` +
            `were graded above`);
        say(`  self-test failure codes in it  : ${codes.length
            ? codes.map((k) => `?${k.code}@${k.at}`).join(" ")
            : "(none)"}`);
        for (let k of codes) {
            say(`     ?${k.code} at byte ${k.at}${k.at < repro ? " (inside the graded prefix)"
                                                              : " -- PAST the graded prefix, reported only"}`);
        }
        say(`  tail past the graded prefix    : "${show(js.bytes, repro, 160)}"`);
    }

    say(`\nPHASE D -- device census over the same ${js.steps}-instruction walk`);
    {
        /* THIS IS A REPORT, NOT A GATE.  A device the ROM never drives is a legitimate finding
           about the ROM, not a defect in the machine, so nothing here fails the run -- the one
           census assertion that DOES fail is the txdbWr/byte-count cross-check above.  The device
           roster comes from what the machine constructed (js.deviceNames), not from the tally, so a
           device with zero calls is genuinely reportable. */
        let byDev = new Map(js.deviceNames.map((d) => [d, []]));
        for (let [k, v] of js.tally) {
            let dot = k.indexOf(".");
            let dev = k.slice(0, dot);
            if (!byDev.has(dev)) byDev.set(dev, []);
            byDev.get(dev).push([k.slice(dot + 1), v]);
        }
        for (let [dev, ms] of byDev) {
            let live = ms.filter((m) => m[1] > 0).sort((a, b) => b[1] - a[1]);
            let total = live.reduce((a, m) => a + m[1], 0);
            let detail = live.length ? live.map((m) => `${m[0]}=${m[1]}`).join(" ") : "(NEVER TOUCHED)";
            say(`  ${dev.padEnd(11)} ${String(total).padStart(9)} call(s): ${detail}`);
        }
        let untouched = [...byDev.entries()].filter(([, ms]) => !ms.some((m) => m[1] > 0)).map((e) => e[0]);
        say(`  devices the ROM never touched: ${untouched.length ? untouched.join(", ") : "(none -- all " +
            `${byDev.size} constructed devices were driven by the ROM)`}`);
    }

    return {problems, js, oA, oB, oShort, searched, searchSteps, fullSteps, repro, oracleRepro,
            floorBytes, c};
}

/* ------------------------------------------------------------------------------------------- *
 * SELFCHECK                                                                                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * MUTATIONS -- each PERTURBS the shipped path and returns a restore function.  HANDOFF.md standing
 * rule 11: a mutation must compose over the original, never substitute its own copy, because a
 * substituted copy is idempotent when the original is already broken and certifies coverage it does
 * not have.  Every one below calls the original first and then perturbs its effect.
 */
const MUTATIONS = {

    /* The item's own named "single most likely failure mode": if TXCS never reports CSR_DONE the
       ROM's output routine spins forever and prints nothing.  Composed over the real
       maybeComplete(), so the deferred-completion logic still runs and is then undone. */
    "txcs-done-stuck-clear": function() {
        let orig = ConsoleVAX.prototype.maybeComplete;
        ConsoleVAX.prototype.maybeComplete = function() {
            orig.call(this);
            this.txcs = this.txcs & ~0x0080;
        };
        return () => { ConsoleVAX.prototype.maybeComplete = orig; };
    },

    /* RXDB/TXDB byte-lane defect, on the output side: the real write happens, then the byte that
       reached the stream is replaced by the wrong lane of the same value. */
    "txdb-wrong-byte-lane": function() {
        let orig = ConsoleVAX.prototype.txdbWr;
        ConsoleVAX.prototype.txdbWr = function(val) {
            orig.call(this, val);
            this.output[this.output.length - 1] = (val >>> 8) & 0xFF;
        };
        return () => { ConsoleVAX.prototype.txdbWr = orig; };
    },

    /* ONE byte lost MID-STREAM -- the stream keeps flowing, shifted by one, so what this proves is
       that the comparison finds a divergence at a NON-ZERO offset rather than only catching a stream
       that is wrong from its first byte or stops entirely.  `dropped` is a one-shot latch for
       exactly that reason: an earlier revision dropped whenever the length reached 4, which pinned
       the stream at 3 bytes and degenerated into the same "output stops" case the first two
       mutations already cover.

       THE LATCH IS PER-CONSOLE, NOT PER-MUTATION, and that distinction became load-bearing when
       pcjsvax-e29 made PHASE S walk the machine three times.  A latch held in this closure fires on
       the FIRST walk only, so the other two walks emit a full-length stream and the mutation is
       caught by PHASE S's same-budget LENGTH check instead of by the divergence comparison it was
       written to exercise -- it would still have been reported CAUGHT, while quietly no longer
       testing the thing its own comment claims (HANDOFF.md standing rule 12).  Latching on the
       console instance instead means every walk drops its 4th byte, all three streams are the same
       length, and the non-zero-offset divergence is once again what catches it. */
    "txdb-drops-one-byte-mid-stream": function() {
        let orig = ConsoleVAX.prototype.txdbWr;
        ConsoleVAX.prototype.txdbWr = function(val) {
            orig.call(this, val);
            if (!this.conoutDropped && this.output.length === 4) { this.output.pop(); this.conoutDropped = true; }
        };
        return () => { ConsoleVAX.prototype.txdbWr = orig; };
    },

    /* The instrument's own front bracket, off by one: the console stream is taken to start one byte
       earlier, so SCP's newline becomes the oracle's first byte. */
    "oracle-extract-off-by-one": function() {
        let orig = OracleCapture.extract;
        OracleCapture.extract = function(raw) {
            let b = orig.call(this, raw);
            let out = new Uint8Array(b.length + 1);
            out[0] = 0x0A;
            out.set(b, 1);
            return out;
        };
        return () => { OracleCapture.extract = orig; };
    },

    /* The instrument's own back bracket: SCP's stop message left ON the end of the oracle's stream.
       Caught by PHASE R's monotonicity claim -- the half-budget capture's stop message lands in the
       middle of the full-budget capture's real output -- which is the point: the front bracket and
       the back bracket are checked by DIFFERENT phases. */
    "oracle-trailer-not-trimmed": function() {
        let orig = OracleCapture.extract;
        OracleCapture.extract = function(raw) {
            let b = orig.call(this, raw);
            let tail = "\nStep expired, PC: 00000000 (NOP)\n";
            let out = new Uint8Array(b.length + tail.length);
            out.set(b, 0);
            for (let i = 0; i < tail.length; i++) out[b.length + i] = tail.charCodeAt(i);
            return out;
        };
        return () => { OracleCapture.extract = orig; };
    },

    /* THE MUTATION THAT WOULD MAKE THIS WHOLE FILE GREEN AND BLIND: a comparison that reports
       agreement it did not verify.  Composed over the real answer, then overstated to the full
       overlap.  PrefixClaim.verify() is the only thing standing between this and a false pass. */
    "common-prefix-overstated": function() {
        let orig = Compare.commonPrefix;
        Compare.commonPrefix = function(a, b) {
            orig.call(this, a, b);
            return Math.min(a.length, b.length);
        };
        return () => { Compare.commonPrefix = orig; };
    },

    /* The census made blind.  Caught by the cross-check against the captured byte count, which is
       the whole reason that cross-check exists rather than a bare "the console was touched" gate. */
    "census-records-nothing": function() {
        let orig = Census.note;
        Census.note = function(tally, key) { orig.call(this, tally, key); tally.set(key, 0); };
        return () => { Census.note = orig; };
    },

    /* The reason this file exists as a separate instrument: romdiff.js's ceiling (SIMH's HIST_MAX,
       249,743 usable) is BELOW the ROM's first console byte.  A change that silently reinstates that
       budget here would report "no banner" and look like a regression in the machine rather than in
       the instrument.  MEASURED: at this budget this machine emits zero bytes. */
    "js-budget-reverted-to-romdiff-ceiling": function() {
        let orig = Walk.budget;
        Walk.budget = function(opts) { orig.call(this, opts); return 200000; };
        return () => { Walk.budget = orig; };
    },

    /* THE DEFECT pcjsvax-e29 EXISTS TO REMOVE, and the one the mutation above structurally could
       not catch.  A budget silently reverted to 3,000,000 is FAR past romdiff's ceiling, so the
       stream is 395 bytes rather than zero: the banner is complete, four lines exist, the floor is
       met, PHASE C still names a first diverging byte and the run still exits 0.  Every assertion
       this file had before PHASE S is satisfied by it.  What it hides is the 1,012 bytes after
       byte 395, which is where ?51, ?46 and ?80 live.

       So this mutation is caught by PHASE S and by nothing else, which is what makes it worth
       having: it is a coverage hole with no other tripwire over it, and it shipped green for as
       long as it existed (HANDOFF.md standing rule 13).  Composed over the shipped Walk.budget --
       the original is called and its answer discarded -- rather than substituting a copy, so it
       still perturbs the path the run actually takes (standing rule 11). */
    "js-budget-truncated-mid-stream": function() {
        let orig = Walk.budget;
        Walk.budget = function(opts) { orig.call(this, opts); return 3000000; };
        return () => { Walk.budget = orig; };
    }
};

/**
 * selfcheck(romBytes, opts, magicByte, simh)
 *
 * Returns FALSE rather than calling process.exit(), so main()'s `finally` still removes the scratch
 * directory on the failing path.  An earlier revision of this function exited from here and leaked
 * one scratch directory per failed --selfcheck.  That is not hypothetical housekeeping: HANDOFF.md
 * §4 records a run that filled the root filesystem, and while this item was being worked the same
 * disk was carrying 83 abandoned differential scratch directories holding 10 GB, left behind by
 * checks that exit without cleaning up.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 * @param {string} simh
 * @returns {boolean} true if every mutation was caught
 */
function selfcheck(romBytes, opts, magicByte, simh)
{
    let names = Object.keys(MUTATIONS), survivors = [];
    let rawCache = new Map();

    /* The clean run first: a mutation suite whose baseline already fails proves nothing. */
    let base = runAll(romBytes, opts, magicByte, simh, rawCache, true);
    if (base.problems.length) {
        console.error(`FAILURES:\n  --selfcheck's own baseline run is not clean, so no mutation result ` +
            `below would mean anything:\n    ` + base.problems.join("\n    "));
        return false;
    }
    console.log(`baseline (unmutated): CLEAN -- ${base.c.n} byte(s) matched, floor ${base.floorBytes}\n`);

    for (let name of names) {
        let restore = MUTATIONS[name]();
        let caught = false, detail = "";
        try {
            let r = runAll(romBytes, opts, magicByte, simh, rawCache, true);
            caught = r.problems.length > 0;
            detail = r.problems.length ? r.problems[0] : `run was CLEAN with the mutation applied`;
        } catch (e) {
            caught = true;
            detail = `threw: ${e.message.split("\n")[0]}`;
        } finally {
            restore();
        }
        console.log(`  ${caught ? "CAUGHT " : "SURVIVED"}  ${name}`);
        console.log(`            ${detail.slice(0, 220)}`);
        if (!caught) survivors.push(name);
    }

    if (survivors.length) {
        console.error(`\nFAILURES:\n  ${survivors.length} mutation(s) SURVIVED -- each one is a coverage ` +
            `hole, not a tuning knob (HANDOFF.md standing rule 3):\n    ` + survivors.join("\n    "));
        return false;
    }
    console.log(`\nall ${names.length} mutations CAUGHT`);
    return true;
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                            *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, dflt)
{
    let i = process.argv.indexOf(name);
    return (i >= 0 && i + 1 < process.argv.length) ? process.argv[i + 1] : dflt;
}

function cleanupScratch(opts)
{
    try { fs.rmSync(opts.scratch, {recursive: true, force: true}); }
    catch (e) { console.log(`  (could not remove scratch ${opts.scratch}: ${e.message})`); }
}

function main()
{
    let opts = {
        jsSteps: +getArg("--js-steps", JS_STEPS_DEFAULT),
        oracleSeed: +getArg("--oracle-steps", ORACLE_SEED_STEPS),
        scratch: fs.mkdtempSync(path.join(os.tmpdir(), "conoutdiff-"))
    };
    opts.rom = findRom(getArg("--rom", null));
    let simh = findSimh(getArg("--simh", null));
    let fSelfcheck = process.argv.indexOf("--selfcheck") >= 0;

    console.log(`SIMH: ${simh}`);
    console.log(`ROM:  ${opts.rom}`);
    console.log(`scratch: ${opts.scratch}`);

    let romBytes = new Uint8Array(fs.readFileSync(opts.rom));
    if (romBytes.length !== VAX.PHYSMEM.ROM_SIZE) {
        console.error(`FAILURES:\n  ROM: ${opts.rom} is ${romBytes.length} bytes, expected exactly ` +
            `${VAX.PHYSMEM.ROM_SIZE} (VAX.PHYSMEM.ROM_SIZE)`);
        cleanupScratch(opts);
        process.exit(1);
    }
    if (!(opts.jsSteps > 0) || !(opts.oracleSeed > 0)) {
        console.error(`FAILURES:\n  --js-steps ${opts.jsSteps} / --oracle-steps ${opts.oracleSeed} must ` +
            `both be positive`);
        cleanupScratch(opts);
        process.exit(1);
    }

    let {sysModel, magicByte} = querySysModel(simh, opts);
    console.log(`\nsys_model (measured, examine MODEL on this exact oracle): ${sysModel} -> magic byte ` +
        `0x${hex(magicByte, 2)}`);

    try {
        if (fSelfcheck) {
            if (!selfcheck(romBytes, opts, magicByte, simh)) process.exitCode = 1;
            return;
        }
        let r = runAll(romBytes, opts, magicByte, simh, null);
        console.log(`\npeak JS heap: ${g_peakHeapMB.toFixed(1)} MB (absolute ceiling ${PEAK_HEAP_MB_MAX} MB)`);
        if (r.problems.length) {
            console.error(`\nFAILURES:\n  ` + r.problems.join("\n  "));
            process.exitCode = 1;
            return;
        }
        console.log(`\nOK`);
        console.log(`\nCONSOLE OUTPUT: this machine and the oracle emit the SAME ${r.c.n} byte(s), which ` +
            `is ${r.c.n >= r.floorBytes ? "at or past" : "SHORT OF"} the ${r.floorBytes}-byte floor ` +
            `(${BANNER_MATCH_FLOOR_LINES} complete lines of the oracle's own output).  ${r.c.why}.`);
    } finally {
        cleanupScratch(opts);
    }
}

main();

export { OracleCapture, Compare, PrefixClaim, Census, Walk, compare, lineFloorBytes, nameAddress };
