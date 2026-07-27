/**
 * @fileoverview Differential test: the KA655 console ROM, decoded and executed from the boot
 *               entry state, vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-223 (TRACE, MIRROR, SELFCHECK) and pcjsvax-320 (the SSC-BASE-RANDOM addition to TRACE
 * and SELFCHECK, and the boundary-accounting fix TRACE needed once its own advance exposed it).
 * Four claims, each with its own phase below:
 *
 *   TRACE     ka655x.bin is loaded at BusVAX.addRom(), CPUStateVAX.boot() sets PC/PSL/the model
 *             magic byte exactly as vax_sysdev.c's cpu_boot() does, and the machine executes from
 *             there.  A per-instruction trace comparison against a REAL SIMH -- booted for real,
 *             via the actual `BOOT CPU` command, not a hand reconstruction of its side effects --
 *             must show ZERO divergence up to the point where this machine's JS side first
 *             touches a physical range no item has decoded yet.  That point is reported BY NAME:
 *             this item's whole deliverable is telling the next items what the ROM needs and in
 *             what order.  Modeled on tests/cpudiff.js's EHKAA phase -- same trace format
 *             (tests/simhtrace.js), same oracle (tools/trace-differ/differ.py), same
 *             unavailable/CMPD/ZEROSPEC normalization, because it is the same kind of claim: a
 *             PROGRAM runs, not one instruction.  pcjsvax-320's own boundary-advance (past the SSC
 *             base register's store) exposed a real bug in how many trace records were comparable
 *             when the boundary lands on the INSTRUCTION AFTER one that now completes -- see
 *             BoundaryAccounting.compute()'s doc comment.
 *
 *   SSC-BASE- pcjsvax-320: the TRACE phase's boot run only ever stores ONE value into the SSC base
 *   RANDOM    register (0x20140000, which already satisfies its own mask), so verifySscBaseRandom()
 *             drives many MORE values -- boundary-chosen and random -- through a real instruction
 *             round trip on the live oracle, proving the SSCBASE_RW/SSCBASE_MBO mask itself, not
 *             just whether the address is backed.  Enforces its own coverage floor.
 *
 *   MIRROR    The upper half of the ROM (VAX.PHYSMEM.ROM_BASE + ROM_SIZE .. +ROM_LENGTH) must read
 *             back exactly what the lower half holds, at several offsets including the boundary,
 *             on BOTH sides -- and it must keep tracking a DISTINCT sentinel byte written directly
 *             into the primary after construction, proving true (live) aliasing rather than a
 *             load-time copy that could go stale.  NOT the boot-time magic byte: ka655x.bin
 *             already ships 0x02 at offset 4 and the measured magic byte is also 2, so re-writing
 *             it proves nothing -- a snapshot taken at load time would show the identical value.
 *             A prior version of this check used the magic byte and was vacuous for exactly that
 *             reason (caught by an adversarial veracity review); see verifyMirrorJS()'s header.
 *
 *   WALK     pcjsvax-fe7.  The TRACE phase no longer stops at the FIRST bus fault.  It stops at the
 *             first fault the two engines do not GRADE EQUAL.  See FaultGrader/WalkDecision below
 *             for the rule and PHASE F for the regression floor that keeps it honest.
 *
 *   FLOOR     pcjsvax-fe7.  Every ordinary run ALSO re-walks the ROM with one decoded device
 *             deliberately removed (--no-cdg) and requires the walk to stop at, and NAME, the
 *             resulting real hardware gap.  A walk rule that can skip a real gap is a blind meter.
 *
 *   SELFCHECK --selfcheck injects THIRTEEN deliberate defects into the SHIPPED code path (two
 *             distinct ROM-mirror failure modes -- reads garbage, and reads a stale load-time
 *             snapshot -- CPUStateVAX.boot()'s magic byte / PSL, the ROM's read-only enforcement,
 *             the SSC base register's decode and its mask, BoundaryAccounting's off-by-one,
 *             StopReported's enforcement, the fall-through revert, and pcjsvax-fe7's three: the
 *             graded-equality check weakened to "both faulted", the oracle probe cache answering
 *             for the wrong address, and the walk continuing past a fault SIMH services) and fails
 *             if any one of them is not caught.
 *
 * THE WALK RULE (pcjsvax-fe7) -- WHY "BOTH FAULTED" IS NOT ENOUGH
 * ---------------------------------------------------------------
 * The KA655 ROM autoconfigures the Qbus by PROBING addresses that are not populated and absorbing
 * the machine check that comes back.  Measured (pcjsvax-93e): from a cold boot the ROM's first bus
 * fault is now IOPAGE+0x1F00, SIMH machine-checks on it IDENTICALLY, and both engines dispatch into
 * the same handler (record 484 is `2004C004 041F0000| MOVB 4(SP),@#201407B3` on BOTH).  Stopping
 * there names no boundary, because there is no undecoded hardware to name.
 *
 * So the walk continues past a fault -- but ONLY when the JS fault and the SIMH fault are GRADED
 * EQUAL.  Two independent graders, each covering what the other cannot:
 *
 *   PER-ADDRESS (FaultGrader.verdict, cached): the SAME synthetic longword reference at the SAME
 *     address is executed on BOTH engines against an identical, fully-populated 64-entry SCB whose
 *     vector v lives at a DISTINCT handler address.  Compared: did it fault at all, WHICH SCB
 *     vector it dispatched to (read back from PC, not assumed), and the resulting PC and PSL.  Any
 *     disagreement stops the walk.  "Both faulted" alone never continues it.
 *   IN SITU (the trace comparison, which EXTENDS with the walk): every instruction the walk
 *     executes, including the ROM's own machine-check handler, is compared record-for-record
 *     against the oracle.
 *
 * Neither grader is sufficient alone and the code does not claim otherwise.  TWO named holes in the
 * per-address probe, both closed by the in-situ one:
 *   - A JS fault at a WRONG EFFECTIVE ADDRESS that happens to also be absent on the oracle: the
 *     probe grades the two engines equal AT THAT ADDRESS, and is right to -- but SIMH's real
 *     execution never went there, so the traces diverge.
 *   - GRANULARITY: the probe is a longword reference at `addr & ~3` (faultProbeBytes()), so an
 *     in-situ byte or word fault whose longword does not fault on EITHER engine grades "equal".
 *     Again correct about the address and wrong about the run -- and again the traces diverge,
 *     because SIMH completed the instruction the JS side faulted on.
 *
 * THE SIMH CAPTURE IS STEP-BOUNDED, NOT ONLY BREAKPOINT-BOUNDED (pcjsvax-fe7)
 * ---------------------------------------------------------------------------
 * MEASURED INSTRUMENT DEFECT (pcjsvax-93e, reproduced here): `SET CPU HISTORY=n:file` fills a
 * record's RESULT field at the TOP of the NEXT loop iteration (vax_cpu.c:654-680) but flushes the
 * whole ring on WRAP (vax_cpu.c:1676), so the last record of every ring pass reaches the file with
 * an unfilled result.  A walk longer than the ring therefore MANUFACTURES divergences.
 *
 * This file avoids the wrap entirely rather than working around it: the ring is set to SIMH's own
 * HIST_MAX (250000, vax_defs.h:852) and the capture is bounded by a STEP COUNT that can never
 * exceed it, so hst_p never wraps and the ONLY flush is the single positive-abort one at the end of
 * the step (vax_cpu.c:566).  That leaves exactly ONE artifact record -- the last one written -- and
 * SIMH_TRACE_MARGIN records of slack are demanded past the compared range so it lands outside it.
 * A breakpoint at ROM_BASE stops `boot cpu` at the first instruction (`boot` cannot be given a step
 * count -- see the section above), and `step K` then runs an EXACT, bounded number of iterations.
 *
 * VALIDATED AT A SMALL STEP COUNT BEFORE BEING TRUSTED AT A LARGE ONE, two ways, both at 3000 steps:
 *   (1) against the OLD breakpoint-bounded capture this replaces -- the two agree record-for-record
 *       over all 484 comparable records except index 32, an `MFPR #1B` (TODR) whose value is
 *       real-time-dependent and differs between ANY two SIMH runs by ANY method;
 *   (2) against DELIBERATELY WRAPPED captures of the same run -- rings of 1000 and 1001 differ from
 *       the non-wrapping 250000-entry capture at exactly one extra record each, at index 1999
 *       (index%1000 == 999) and index 2001 (index%1001 == 1000): the wrap boundaries, moving with
 *       the ring length exactly as vax_cpu.c predicts, and absent entirely when the ring does not
 *       wrap.  captureSimhTrace() asserts the non-wrapping condition rather than assuming it.
 *
 * A step count is NOT a record count: 3000 steps produced 2486 records (instruction attempts that
 * abort before vax_cpu.c:1632 consume a step and record nothing), so the capture asks for records
 * and RAISES the step count until it has enough, rather than assuming a ratio.
 *
 * WHY THE SIMH SIDE USES A REAL `BOOT CPU`, BOUNDED BY A BREAKPOINT, NOT A HAND-BUILT DEPOSIT
 * ---------------------------------------------------------------------------------------------
 * `BOOT CPU` (scp.c run_cmd(), RU_BOOT) calls the real `cpu_boot()` and then falls straight into
 * the free-running instruction loop -- `sim_step` is unconditionally reset to 0 for this command,
 * so there is no way to hand BOOT a step count, and on real ROM firmware that loop does not stop
 * on its own (it is the interactive console monitor, which waits on a terminal).  A breakpoint set
 * BEFORE `boot cpu` (`break <addr>`) bounds it without needing to guess anything about
 * `sysd_powerup()` or reconstruct `cpu_boot()` by hand -- SIMH does its own real boot, and control
 * returns to the script the instant the breakpoint hits, exactly like an interactive session.  The
 * breakpoint address is the JS run's OWN final PC (post-decode, pre-store) for the first
 * instruction it could not complete: for a non-branching store (which is what this item's boundary
 * turned out to be -- see below) that PC is exactly where the next instruction begins on BOTH
 * machines, and even if it were a few instructions early or late, `tools/trace-differ` truncates
 * SIMH's capture to the JS run's own length before comparing, so the breakpoint only needs to be
 * "at or after" the boundary, never exact.
 *
 *      node machines/dec/vax/tests/romdiff.js [options]
 *        --simh PATH        patched microvax3900; else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch
 *                            build (same search cpudiff.js uses)
 *        --rom PATH         default $PCJS_VAX_REPO/open-simh/VAX/ka655x.bin
 *        --max-steps N       the WALK BUDGET: how far the walk may run before "budget exhausted"
 *                            becomes the named stopping reason (default WALK_BUDGET_DEFAULT, hard
 *                            ceiling WALK_BUDGET_MAX -- see those constants for the SIMH ring bound)
 *        --ssc-base-cases N  randomized SSC base register mask cases (default 40, floor 8)
 *        --seed S            PRNG seed for --ssc-base-cases, printed on failure so it reproduces
 *        --no-cdg            omit the cache-diagnostic decode from the machine under test.  This is
 *                            the REGRESSION FLOOR's own knob (PHASE F runs it automatically on every
 *                            ordinary invocation); it exists so the floor can present a REAL,
 *                            currently-decoded hardware gap to the walk rule and require it to stop.
 *        --selfcheck         prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX, { VAXStop, ROM_MAGIC_BYTE } from "../modules/v2/cpustate.js";
import SimhTrace, { nonStoringResultOpcodes } from "./simhtrace.js";
import { OPCODES, DROM, DROM_STRIDE, DR } from "../modules/v2/drom.js";
import SSCVAX from "../modules/v2/ssc.js";
import { SCB } from "../modules/v2/exc.js";
import { makeRomMachine } from "./rommachine.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/* Enforced floor for --ssc-base-cases (standing rule: coverage assertions FAIL the run and do not
   scale down with case count).  4 fixed boundary values (0, all-ones, exactly SSCBASE_RW, exactly
   SSCBASE_MBO) plus at least 4 genuinely random ones -- see verifySscBaseRandom(). */
const SSC_BASE_CASES_FLOOR = 8;

/* ---- pcjsvax-fe7: the walk's own bounds.  Every one of these FAILS the run when exceeded; none
   scales with anything (HANDOFF.md standing rule 4). ---- */

/** SIMH's own vax_defs.h:852 HIST_MAX.  `set cpu history=` refuses anything larger, so this is the
    hard ceiling on how many instructions one capture can hold WITHOUT wrapping the ring -- and a
    wrap is what manufactures phantom divergences (see the file header). */
const SIMH_HIST_LNT = 250000;

/** Records of slack demanded PAST the compared range, so the single end-of-step artifact record
    (the one whose result field vax_cpu.c never got to fill) can never land inside it.  Only ONE is
    structurally possible; this is deliberate slack, not a measured requirement. */
const SIMH_TRACE_MARGIN = 256;

/** The largest capture this file will ever ask SIMH for.  The ring wraps when the hst_p index
    REACHES hst_lnt (vax_cpu.c:1673-1677), i.e. on the 250000th record -- so the last safe count is
    one less, and steps are capped there rather than at the ring length itself. */
const SIMH_STEPS_MAX = SIMH_HIST_LNT - 1;

/** Hard ceiling on --max-steps.  A JS run produces at most one comparable record per step, and the
    SIMH capture must fit BOTH that many records AND the margin inside a non-wrapping ring. */
const WALK_BUDGET_MAX = SIMH_STEPS_MAX - SIMH_TRACE_MARGIN;

/** The default walk budget.  "Budget exhausted" is a legitimate, NAMED stopping reason -- this is a
    bound on gate wall-clock, not a claim about the ROM.  MEASURED (pcjsvax-fe7): romdiff's total
    runtime at this budget keeps it inside the ~10-minute full-gate budget HANDOFF.md §4 documents.
    Raise it with --max-steps (ceiling WALK_BUDGET_MAX) to walk further. */
const WALK_BUDGET_DEFAULT = 200000;

/** Distinct faulting addresses the walk will pay a live oracle probe for before declaring itself
    lost.  The ROM's Qbus autoconfiguration probes a handful; hundreds means the walk is not
    absorbing device probes, it is executing garbage. */
const MAX_DISTINCT_FAULT_ADDRS = 64;

/** Fault EVENTS whose detail is retained for the report.  The ROM legitimately re-probes the same
    address repeatedly, so this bounds the report, not the walk: the event and distinct-address
    COUNTS stay exact and the report says so when detail was truncated. */
const MAX_REPORTED_FAULT_EVENTS = 32;

/** ABSOLUTE peak-heap ceiling, in MB (HANDOFF.md standing rule 14: assert an absolute bound that
    FAILS the run and does NOT scale with case count).  The measured peak for the shipped
    configuration is printed on every run, so drift is visible long before it is fatal. */
const PEAK_HEAP_MB_MAX = 1200;

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

/** The largest `heapUsed` OBSERVED at this file's sample points -- after the walk serialises its
    trace (runRomJS(), where a walk peaks), after the SIMH capture, after the trace comparison, after
    the floor walk, and at the end.  It is a sampled maximum, not a continuous one, and is not
    claimed to be either RSS or a true peak between samples. */
let g_peakHeapMB = 0;
function sampleHeap()
{
    let mb = process.memoryUsage().heapUsed / (1024 * 1024);
    if (mb > g_peakHeapMB) g_peakHeapMB = mb;
    return g_peakHeapMB;
}

/* ------------------------------------------------------------------------------------------- *
 * Plumbing -- same shape as cpudiff.js's, deliberately, so a reader of both trusts it once      *
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
        "romdiff needs a REAL SIMH (patch 0001, the REGS history line); it has no fixture\n" +
        "fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh PATH or\n" +
        "set $SIMH_CPU_BIN.  Tried:\n  " + candidates.join("\n  "));
}

function findDiffer()
{
    let p = path.join(vaxRepo(), "tools/trace-differ/differ.py");
    if (!fs.existsSync(p)) {
        throw new Error(`trace-differ not found at ${p}; set $PCJS_VAX_REPO to the pcjs-vax work repo`);
    }
    return p;
}

function findRom(pathArg)
{
    let p = pathArg || path.join(vaxRepo(), "open-simh/VAX/ka655x.bin");
    if (!fs.existsSync(p)) {
        throw new Error(`ka655x.bin not found at ${p}; pass --rom PATH`);
    }
    return p;
}

function runSimh(bin, script, iniPath, timeoutMs = 5 * 60 * 1000)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: timeoutMs});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test                                                                        *
 * ------------------------------------------------------------------------------------------- */

/**
 * makeMachine(romBytes, fOmitCdg)
 *
 * pcjsvax-bfb MOVED this function's body, unchanged, to tests/rommachine.js so that this file and
 * tests/conoutdiff.js boot the SAME machine instead of two hand-kept copies of it -- see that
 * file's header for why (HANDOFF.md standing rule 7).  Nothing about what is decoded, or about
 * `fOmitCdg`, changed in the move; this wrapper exists only so the ~5 call sites below keep their
 * `{bus, cpu}` shape and do not have to care that the builder now returns device handles too.
 *
 * @param {Uint8Array} romBytes
 * @param {boolean} [fOmitCdg]
 * @returns {Object} {bus, cpu}
 */
function makeMachine(romBytes, fOmitCdg = false)
{
    let {bus, cpu} = makeRomMachine(romBytes, fOmitCdg);
    return {bus, cpu};
}

/**
 * PHYSMEM_NAMES -- resolved from VAX.PHYSMEM itself (never hand-listed), used only to name a
 * stopping address in the report below.
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

/**
 * nameAddress(addr)
 *
 * @param {number} addr
 * @returns {string}
 */
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

/**
 * BoundaryAccounting.compute(records, finalizedPartial)
 *
 * pcjsvax-320: how many of `records` (hst.count after runRomJS()'s hst.finish(cpu) call) are
 * GENUINELY comparable against SIMH, and what is the boundary instruction's own 1-based ordinal.
 * A plain object property (not a bare top-level function) so selfcheck() below can patch it the
 * same way it patches every other shipped entry point -- see MUTATIONS' "boundary-off-by-one" for
 * why this needed its own mutation rather than trusting the live romdiff run alone.
 *
 * `finalizedPartial` is measured directly by runRomJS() (did hst.finish(cpu) change hst.count?),
 * not assumed: see runRomJS()'s doc comment for the two shapes a boundary can take and why
 * conflating them was a real bug this item's own boundary-advance exposed.
 *
 * @param {number} records
 * @param {boolean} finalizedPartial
 * @returns {{comparableRecords: number, instrNum: number}}
 */
const BoundaryAccounting = {
    compute(records, finalizedPartial) {
        let comparableRecords = finalizedPartial ? records - 1 : records;
        return {comparableRecords, instrNum: comparableRecords + 1};
    }
};

/**
 * STOP_KINDS -- every shape the walk is allowed to end in.  `named` is what report() must print;
 * `fatal` is whether the run also FAILS.  Derived from this table, never hand-listed at the call
 * sites, so a new stop kind cannot be added without deciding both questions.
 *
 * pcjsvax-fe7 replaced pcjsvax-320's BoundaryRequired with this.  The invariant BoundaryRequired
 * enforced -- "a run that stops must NAME where and why, never exit clean having named nothing"
 * (HANDOFF.md standing rule 13) -- is unchanged and is now StopReported.check()'s job.  What
 * changed is which stops are LEGITIMATE: a fault both engines grade equal is no longer a failure,
 * so "budget exhausted having absorbed N graded-equal faults" is a real, passing outcome -- but it
 * is still a NAMED one, and a run that produces no named stop at all is still not a pass.
 */
const STOP_KINDS = {
    "boundary":        {fatal: false, label: "UNDECODED-HARDWARE BOUNDARY"},
    "fault-diverged":  {fatal: true,  label: "FAULT-GRADING DIVERGENCE"},
    "cpu-stop":        {fatal: true,  label: "CPU STOP"},
    "budget":          {fatal: false, label: "WALK BUDGET EXHAUSTED"},
    "fault-addr-limit":{fatal: true,  label: "TOO MANY DISTINCT FAULTING ADDRESSES"}
};

/**
 * StopReported.check(js)
 *
 * The successor to pcjsvax-320's BoundaryRequired (see STOP_KINDS above for what changed and what
 * did not).  A run whose stop is missing, or is not one of the kinds this file knows how to NAME,
 * FAILS -- because HANDOFF.md standing rule 13 is that a gate which stops measuring is worse than
 * one that fails, and "exit 0 having named nothing" is exactly that failure.
 *
 * A plain object property (not a bare function) so selfcheck() can patch it the same way it patches
 * BoundaryAccounting.compute() above.
 *
 * @param {Object} js as returned by runRomJS()
 * @returns {?string} a problem string, or null if the run produced a nameable stop
 */
const StopReported = {
    check(js) {
        if (!js.stopKind) {
            return `the walk produced NO stopping reason at all -- it must always name where and why ` +
                `it stopped (boundary, divergence, or budget); a run that names nothing is not a pass`;
        }
        if (!STOP_KINDS[js.stopKind]) {
            return `the walk stopped with an unknown reason "${js.stopKind}" -- report() has no name ` +
                `for it, so this run would print no honest stopping point`;
        }
        if (!js.stopWhy) {
            return `the walk stopped with kind "${js.stopKind}" but no explanation attached -- ` +
                `naming the kind without naming the evidence is not a named stop`;
        }
        return null;
    }
};

/* ------------------------------------------------------------------------------------------- *
 * pcjsvax-fe7 -- GRADED-EQUALITY FAULT WALKING                                                   *
 * ------------------------------------------------------------------------------------------- */

/**
 * FAULT_PROBE -- the synthetic environment BOTH engines execute the per-address probe in.
 *
 * A FULLY POPULATED SCB (all VECTORS entries) whose vector v is handled at a DISTINCT address
 * (HANDLER + v*SLOT) is what makes "which SCB vector was taken" an OBSERVED fact -- read back out
 * of PC afterwards -- instead of an assumption that any fault must have been a machine check.  A
 * single-entry SCB could only ever answer "did PC leave the code", which is the weaker claim
 * pcjsvax-320's probeSimhBackedAt() made and which pcjsvax-fe7's rule explicitly is not allowed to
 * rest on.
 */
const FAULT_PROBE = {
    SCBB:    0x00100000,
    HANDLER: 0x00102000,
    CODE:    0x00104000,
    KSP:     0x00110000,
    IS:      0x00118000,
    VECTORS: 64,                    // SCB offsets 0x000..0x0FC -- covers every exception vector
    SLOT:    16                     // bytes of address space per vector, so PC identifies v exactly
};

/**
 * faultProbeBytes(addr, fWrite)
 *
 * The instruction the probe executes: a LONGWORD reference to the CONTAINING aligned longword,
 * matching the original access's DIRECTION.  Longword-granular and direction-only, exactly like
 * pcjsvax-320's probeSimhBackedAt() which this replaces -- the KA655 register decode is longword-
 * granular (vax_sysdev.c's `rg = (pa >> 2) & ...`), so a byte or word reference inside the same
 * longword classifies identically.  This comment claims nothing about SUB-longword behaviour,
 * because the probe measures none: tests/cqmerrdiff.js and tests/mchkdiff.js own that.
 *
 * @param {number} addr
 * @param {boolean} fWrite
 * @returns {number[]}
 */
function faultProbeBytes(addr, fWrite)
{
    let opcMOVL = OPCODES.indexOf("MOVL"), opcTSTL = OPCODES.indexOf("TSTL");
    if (opcMOVL < 0 || opcTSTL < 0) throw new Error("romdiff.js: MOVL/TSTL not found in drom.js OPCODES");
    let a = (addr >>> 0) & ~3;
    return fWrite
        ? [opcMOVL & 0xFF, 0x00, 0x9F, a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF]
        : [opcTSTL & 0xFF, 0x9F, a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF];
}

/**
 * faultProbeVector(pc)
 *
 * @param {number} pc as read back after the probe instruction
 * @returns {number} the SCB OFFSET dispatched to, or -1 if PC never entered the handler page
 */
function faultProbeVector(pc)
{
    pc = pc >>> 0;
    let d = (pc - FAULT_PROBE.HANDLER) >>> 0;
    if (pc < FAULT_PROBE.HANDLER || d >= FAULT_PROBE.VECTORS * FAULT_PROBE.SLOT) return -1;
    if (d % FAULT_PROBE.SLOT) return -1;
    return (d / FAULT_PROBE.SLOT) * 4;
}

/**
 * FaultGrader -- the walk's oracle.  Every member is a plain object property so selfcheck() can
 * PERTURB the shipped path (HANDOFF.md standing rule 11) rather than substitute a copy of it.
 */
const FaultGrader = {

    /**
     * cacheKey(addr, fWrite)
     *
     * The probe is longword-granular and direction-sensitive (see faultProbeBytes()), so the key is
     * too.  Getting this wrong is not a performance bug -- it makes the walk answer for the WRONG
     * ADDRESS, which is why selfcheck's "probe-cache-stale-address" mutation perturbs exactly this
     * function and nothing else.
     *
     * @param {number} addr
     * @param {boolean} fWrite
     * @returns {string}
     */
    cacheKey(addr, fWrite) { return `${hex((addr >>> 0) & ~3)}:${fWrite ? "W" : "R"}`; },

    /**
     * probeJS(ctx, addr, fWrite)
     *
     * @param {Object} ctx
     * @param {number} addr
     * @param {boolean} fWrite
     * @returns {{faulted: boolean, vector: number, pc: number, psl: number, threw: ?string}}
     */
    probeJS(ctx, addr, fWrite) {
        /* ONE probe machine, reused across every probe (HANDOFF.md standing rule 14: a fresh
           machine per case is what drove cdgdiff.js to 8.6 GB -- each construction registers the
           64 MB CDG span as thousands of blocks with a per-block access table). */
        if (!ctx.probeMachine) ctx.probeMachine = makeMachine(ctx.romBytes, ctx.omitCdg);
        let {bus, cpu} = ctx.probeMachine;
        let bytes = faultProbeBytes(addr, fWrite);
        cpu.reset();
        cpu.exc.scbb = FAULT_PROBE.SCBB;
        for (let v = 0; v < FAULT_PROBE.VECTORS; v++) {
            bus.setLong((FAULT_PROBE.SCBB + v * 4) >>> 0, (FAULT_PROBE.HANDLER + v * FAULT_PROBE.SLOT) | 0);
        }
        cpu.exc.stk[0] = FAULT_PROBE.KSP;                       // KERN
        cpu.exc.stk[4] = FAULT_PROBE.IS;                        // interrupt stack
        cpu.regs.fill(0);
        cpu.regs[14] = FAULT_PROBE.KSP;
        for (let i = 0; i < FAULT_PROBE.VECTORS * FAULT_PROBE.SLOT; i++) {
            bus.setByte((FAULT_PROBE.HANDLER + i) | 0, OPCODES.indexOf("NOP") & 0xFF);
        }
        for (let i = 0; i < bytes.length + 4; i++) bus.setByte((FAULT_PROBE.CODE + i) | 0, 0);
        for (let i = 0; i < bytes.length; i++) bus.setByte((FAULT_PROBE.CODE + i) | 0, bytes[i]);
        cpu.psl = 0;
        cpu.regs[15] = FAULT_PROBE.CODE;
        let threw = null;
        try { cpu.stepCPU(1); }
        catch (e) { threw = (e && e.reason) ? e.reason : String(e && e.message || e); }
        let pc = cpu.regs[15] >>> 0, psl = cpu.psl >>> 0;
        let vector = faultProbeVector(pc);
        return {faulted: threw !== null || vector >= 0, vector, pc, psl, threw};
    },

    /**
     * probeSimh(simh, opts, addr, fWrite)
     *
     * Answered by EXECUTION, not by a hand-maintained address-range list: console EXAMINE/DEPOSIT
     * bypasses ReadReg()/WriteReg()/ReadQb()/WriteQb() entirely (cpu_ex()/cpu_dep() check
     * ADDR_IS_MEM/CDG/ROM/NVR directly and never touch the register-space dispatch), so only an
     * actual instruction reproduces the real access path -- the same reasoning tests/mchkdiff.js's
     * calibrate() is built on, applied here to ONE address instead of a whole candidate pool.
     *
     * @param {string} simh
     * @param {Object} opts
     * @param {number} addr
     * @param {boolean} fWrite
     * @returns {{faulted: boolean, vector: number, pc: number, psl: number, threw: ?string}}
     */
    probeSimh(simh, opts, addr, fWrite) {
        const NOP_BYTE = OPCODES.indexOf("NOP") & 0xFF;
        let bytes = faultProbeBytes(addr, fWrite);
        let lines = ["set cpu 16m", "set cpu simhalt", "reset all",
                     `deposit SCBB ${hex(FAULT_PROBE.SCBB)}`,
                     `deposit KSP ${hex(FAULT_PROBE.KSP)}`, `deposit IS ${hex(FAULT_PROBE.IS)}`];
        for (let v = 0; v < FAULT_PROBE.VECTORS; v++) {
            lines.push(`deposit -l ${hex((FAULT_PROBE.SCBB + v * 4) >>> 0)} ${hex((FAULT_PROBE.HANDLER + v * FAULT_PROBE.SLOT) >>> 0)}`);
        }
        for (let i = 0; i < FAULT_PROBE.VECTORS * FAULT_PROBE.SLOT; i++) {
            lines.push(`deposit -b ${hex((FAULT_PROBE.HANDLER + i) >>> 0)} ${NOP_BYTE.toString(16)}`);
        }
        for (let i = 0; i < bytes.length + 4; i++) lines.push(`deposit -b ${hex((FAULT_PROBE.CODE + i) >>> 0)} 0`);
        for (let i = 0; i < bytes.length; i++) lines.push(`deposit -b ${hex((FAULT_PROBE.CODE + i) >>> 0)} ${bytes[i].toString(16)}`);
        lines.push(`deposit PSL 0`, `deposit PC ${hex(FAULT_PROBE.CODE)}`, "step 1",
                   "examine -h PC", "examine -h PSL", "exit", "");
        let out = runSimh(simh, lines.join("\n"), path.join(opts.scratch, "romdiff-fault-probe.ini"));
        let pcm = /^PC:\s*([0-9A-Fa-f]+)/m.exec(out);
        let pslm = /^PSL:\s*([0-9A-Fa-f]+)/m.exec(out);
        if (!pcm || !pslm) throw new Error("romdiff: fault probe produced no PC/PSL readback; SIMH said:\n" + out);
        let pc = parseInt(pcm[1], 16) >>> 0, psl = parseInt(pslm[1], 16) >>> 0;
        let vector = faultProbeVector(pc);
        return {faulted: vector >= 0, vector, pc, psl, threw: null};
    },

    /**
     * gradeEqual(jsP, simhP)
     *
     * THE RULE.  Continue ONLY when the two engines agree on ALL of: whether the reference faulted
     * at all, WHICH SCB vector it dispatched to, and the resulting PC and PSL.
     *
     * "Both faulted" is deliberately NOT sufficient and is the tempting wrong version of this
     * change -- selfcheck's "graded-equality-weakened-to-both-faulted" mutation exists precisely to
     * prove this function is what the walk consults, and that weakening it here is caught.
     *
     * @param {Object} jsP
     * @param {Object} simhP
     * @returns {{equal: boolean, why: string}}
     */
    gradeEqual(jsP, simhP) {
        if (jsP.faulted !== simhP.faulted) {
            return {equal: false, why: `JS ${jsP.faulted ? "faults" : "does NOT fault"} but SIMH ` +
                `${simhP.faulted ? "faults" : "does NOT fault"}`};
        }
        if (!jsP.faulted) {
            return {equal: true, why: `neither engine faults on this reference (PC ${hex(jsP.pc)} both sides)`};
        }
        if (jsP.vector !== simhP.vector) {
            return {equal: false, why: `both fault, but JS took SCB vector 0x${hex(jsP.vector >>> 0, 3)} ` +
                `and SIMH took 0x${hex(simhP.vector >>> 0, 3)} -- "both faulted" is not equality`};
        }
        if (jsP.pc !== simhP.pc) {
            return {equal: false, why: `both fault to SCB vector 0x${hex(jsP.vector >>> 0, 3)}, but the ` +
                `resulting PC differs: js=${hex(jsP.pc)} simh=${hex(simhP.pc)}`};
        }
        if (jsP.psl !== simhP.psl) {
            return {equal: false, why: `both fault to SCB vector 0x${hex(jsP.vector >>> 0, 3)} at PC ` +
                `${hex(jsP.pc)}, but the resulting PSL differs: js=${hex(jsP.psl)} simh=${hex(simhP.psl)}`};
        }
        return {equal: true, why: `both engines take SCB vector 0x${hex(jsP.vector >>> 0, 3)}, PC ` +
            `${hex(jsP.pc)} PSL ${hex(jsP.psl)} -- graded equal on address, vector, PC and PSL`};
    },

    /**
     * verdict(ctx, addr, fWrite)
     *
     * The cached per-address oracle probe.  Caching is not an optimization detail: without it the
     * walk respawns SIMH per FAULT EVENT rather than per distinct ADDRESS, and the ROM's Qbus
     * autoconfiguration re-probes the same addresses repeatedly.
     *
     * @param {Object} ctx
     * @param {number} addr
     * @param {boolean} fWrite
     * @returns {Object} {kind, why, jsP, simhP, addr, fWrite}
     */
    verdict(ctx, addr, fWrite) {
        let key = FaultGrader.cacheKey(addr, fWrite);
        if (ctx.faultCache.has(key)) { ctx.faultCacheHits++; return ctx.faultCache.get(key); }
        let jsP = FaultGrader.probeJS(ctx, addr, fWrite);
        let simhP = FaultGrader.probeSimh(ctx.simh, ctx.opts, addr, fWrite);
        let g = FaultGrader.gradeEqual(jsP, simhP);
        let kind;
        if (g.equal) kind = "equal";
        else if (jsP.faulted && !simhP.faulted) kind = "boundary";       // SIMH services it, we do not
        else if (!jsP.faulted && simhP.faulted) kind = "js-absorbs";     // we service what SIMH faults on
        else kind = "graded-unequal";                                    // both fault, differently
        let v = {kind, why: g.why, jsP, simhP, addr: addr >>> 0, fWrite};
        ctx.faultCache.set(key, v);
        return v;
    }
};

/**
 * WalkDecision.shouldContinue(verdict)
 *
 * The one place the walk decides to keep measuring.  Split out from runRomJS()'s loop so that
 * selfcheck's "walk-continues-past-serviced-fault" mutation can perturb the SHIPPED decision
 * (HANDOFF.md standing rule 11) instead of re-implementing the loop around it.
 *
 * @param {Object} verdict as returned by FaultGrader.verdict()
 * @returns {boolean}
 */
const WalkDecision = {
    shouldContinue(verdict) { return !!verdict && verdict.kind === "equal"; }
};

/* ------------------------------------------------------------------------------------------- *
 * PHASE 0 -- determine sys_model BY EXECUTION, never by assumption                               *
 * ------------------------------------------------------------------------------------------- */

/**
 * querySysModel(simh, opts)
 *
 * vax_sysdev.c cpu_boot():1729 writes `sys_model ? 1 : 2` to ROMBASE+4.  Rather than assume which
 * value a "microvax3900" binary carries (the naming is misleading -- see cpustate.js's
 * ROM_MAGIC_BYTE comment), ask the SAME oracle binary this test grades against.
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Object} {sysModel, magicByte}
 */
function querySysModel(simh, opts)
{
    let out = runSimh(simh, "examine MODEL\nexit\n", path.join(opts.scratch, "romdiff-model.ini"));
    let m = /MODEL:\s*([0-9A-Fa-f]+)/.exec(out);
    if (!m) throw new Error("romdiff: could not read SIMH's MODEL register; SIMH said:\n" + out);
    let sysModel = parseInt(m[1], 16);
    return {sysModel, magicByte: sysModel ? 1 : 2};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 1 -- the trace                                                                           *
 * ------------------------------------------------------------------------------------------- */

/**
 * runRomJS(romBytes, opts, magicByte, mutation)
 *
 * MEASURED CORRECTION (veracity re-dispatch, pcjsvax-446): pcjsvax-446 made onBusFault() dispatch
 * a REAL machine check instead of stopping the simulator -- correct, and the whole point of that
 * item.  But at this exact point in boot, SCBB and IS are both still 0 (cpu.reset()'s default,
 * matching SIMH's own cpu_reset() -- a real ROM's first instructions would set both up before
 * probing hardware, and this harness's boot() intentionally does not reconstruct that, per the
 * file header's rationale for using a real `BOOT CPU`).  So the machine check's OWN exception-
 * frame push targets SP = IS - 8 = -8, which wraps to physical ~0x3FFFFFF8 -- itself unbacked --
 * faulting AGAIN before the first fault's dispatch can complete.  stepInstruction()'s existing
 * depth-bound guard (exc.js, unrelated to this file) eventually throws VAXStop(INIE, ...), whose
 * `detail` is `-pending.code` (the LAST fault's negated SCB offset, e.g. 4 for SCB_MCHK) -- not
 * the address that started the cascade.  `bus.addrFault` is no help either: it is overwritten by
 * every fault in the cascade, so by the time it settles it holds the LAST address, not the first.
 *
 * So the FIRST bus-fault address (the actual undecoded-hardware boundary this item exists to
 * name) has to be captured directly, at the moment it happens, before any cascade obscures it.
 * `firstFaultAddr`/`firstFaultAccess`/`firstFaultPC` do exactly that -- reset at the top of every
 * instruction attempt, so if THIS attempt never faults they stay null, and if it does fault
 * (possibly repeatedly, cascading through the depth-bound guard) they hold the FIRST one.
 *
 * pcjsvax-fe7 REPLACES "stop at the first bus fault" WITH "stop at the first fault the two engines
 * do not grade equal".  The capture above stays, and is still sticky per instruction attempt for
 * exactly the reason recorded below (a cascade must not overwrite the FIRST address), but it is now
 * CONSUMED after every step rather than latched for the whole run: the fault is graded against the
 * live oracle (FaultGrader.verdict(), cached per address), and the walk continues only when
 * WalkDecision.shouldContinue() says the two engines agree on address, vector, PC and PSL.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 * @param {Object} ctx the walk context -- {simh, opts, romBytes, omitCdg, faultCache, ...}
 * @returns {Object}
 */
function runRomJS(romBytes, opts, magicByte, ctx)
{
    let {bus, cpu} = makeMachine(romBytes, ctx.omitCdg);
    cpu.reset();
    cpu.boot(magicByte);
    let hst = new SimhTrace();
    cpu.hst = hst;

    /*
     * pcjsvax-23c FIX: this capture is STICKY across the WHOLE run -- initialized once, here,
     * outside the loop, and never reset per-iteration.  MEASURED REGRESSION this replaces: the
     * previous version reset all three to null at the TOP of every loop iteration, so the capture
     * only survived if the run STOPPED (threw) on the very step that faulted.  That was always
     * true before pcjsvax-055 purely by circumstance -- SCBB/IS were still 0 that early in boot,
     * so any machine check cascaded into a double (KSNV-on-KSNV) fault and a genuine VAXStop --
     * never because a fault that the ROM's OWN handler ABSORBED (dispatched, and continued
     * executing) was actually impossible.  Once pcjsvax-055 unblocked progress far enough for the
     * ROM to have set up SCBB for real, the very next undecoded-hardware reference (a real MOVC5
     * into the CDG range, instruction #392) machine-checked into a handler that absorbed it and
     * kept going -- and the per-iteration reset erased the only record of it one loop iteration
     * later, so the run silently continued 82,000 further instructions into ROM error-recovery
     * code before eventually reaching an unrelated HALT.  `firstFaultAddr === null` inside
     * onBusFault() below already meant "first one wins, forever" -- the bug was that the OUTER
     * loop kept lying to it every iteration.
     */
    let firstFaultAddr = null, firstFaultAccess = null, firstFaultPC = null;
    /*
     * pcjsvax-fe7: `pending` is the FIRST fault of the CURRENT instruction attempt.  It is latched
     * (first one wins) for the duration of that attempt so a machine-check cascade cannot overwrite
     * the address that started it -- the pcjsvax-446 finding above -- and it is CONSUMED, not
     * reset, immediately after every step, so the pcjsvax-23c regression (an outer-loop reset
     * erasing an absorbed fault one iteration later) cannot come back: there is no reset, only a
     * take.  `firstFaultAddr`/`firstFaultPC` remain sticky across the whole run, as pcjsvax-23c
     * made them, purely for reporting the first fault the walk ever saw.
     */
    let pending = null;
    let realOnBusFault = cpu.onBusFault.bind(cpu);
    cpu.onBusFault = function(addr, access) {
        if (pending === null) {
            pending = {addr: addr >>> 0, access, pc: cpu.exc.faultPC >>> 0};
        }
        if (firstFaultAddr === null) {
            firstFaultAddr = addr >>> 0;
            firstFaultAccess = access;
            firstFaultPC = cpu.exc.faultPC >>> 0;      // the FAULTING instruction's own start PC
        }
        return realOnBusFault(addr, access);
    };

    let steps = 0, stop = null;
    /* `faults` is a REPORTING list and is bounded: the ROM can legitimately re-probe one address
       many times, and an unbounded list would grow with the walk (and print a line per event).  The
       COUNTS below are exact; only the retained detail is capped (HANDOFF.md standing rule 6: a case
       that does not reach the report is named, never silently dropped). */
    let faults = [], faultEvents = 0, absorbed = 0, stopFault = null, stopKind = null, stopWhy = null;

    /**
     * Grade whatever fault this instruction attempt produced, if any.  Returns true to keep
     * walking.  A fault that is graded EQUAL is counted and forgotten; anything else stops the
     * walk and becomes the named stopping reason.
     */
    let consumeFault = function() {
        if (pending === null) return true;
        let f = pending; pending = null;
        f.recordsBefore = hst.count;
        f.verdict = FaultGrader.verdict(ctx, f.addr, f.access === VAX.ACCESS.WRITE);
        faultEvents++;
        if (faults.length < MAX_REPORTED_FAULT_EVENTS) faults.push(f);
        if (WalkDecision.shouldContinue(f.verdict)) { absorbed++; return true; }
        stopFault = f;
        stopKind = (f.verdict.kind === "boundary") ? "boundary" : "fault-diverged";
        stopWhy = `${f.verdict.fWrite ? "write to" : "read of"} ${nameAddress(f.addr)} from PC ` +
            `${hex(f.pc)}: ${f.verdict.why}`;
        return false;
    };

    try {
        while (steps < opts.maxSteps) {
            cpu.stepCPU(1); steps++;
            if (!consumeFault()) break;
            if (ctx.faultCache.size > MAX_DISTINCT_FAULT_ADDRS) {
                stopKind = "fault-addr-limit";
                stopWhy = `the walk graded ${ctx.faultCache.size} DISTINCT faulting addresses ` +
                    `(ceiling ${MAX_DISTINCT_FAULT_ADDRS}); each one costs a live oracle probe, and a ` +
                    `walk that keeps finding new ones is not absorbing a device probe, it is lost`;
                break;
            }
        }
        if (stopKind === null && steps >= opts.maxSteps) {
            stopKind = "budget";
            stopWhy = `walked the full --max-steps ${opts.maxSteps} with ${faultEvents} fault(s) at ` +
                `${ctx.faultCache.size} distinct address(es), every one graded EQUAL against the live ` +
                `oracle; no boundary and no divergence was reached within the budget`;
        }
    } catch (e) {
        if (!(e instanceof VAXStop)) throw e;
        stop = e;
        /* A fault that cascaded into a genuine VAXStop never reached the loop's own consumeFault()
           call -- control jumped out of stepCPU() -- so grade it here, or the address that started
           the cascade is lost exactly as it was before pcjsvax-446. */
        consumeFault();
        /* PRECEDENCE, and it matters: a graded BOUNDARY or DIVERGENCE names the CAUSE, the VAXStop
           only names the wreckage downstream of it.  The cascade that follows an undecoded-hardware
           reference at boot (SCBB/IS still 0 -- see this function's pcjsvax-446 note) is exactly
           that wreckage, and reporting it instead of the boundary is what pcjsvax-320 had to fix
           once already.  So cpu-stop is claimed ONLY when the fault grader named nothing. */
        if (stopKind === null) {
            stopKind = "cpu-stop";
            stopWhy = `the JS machine stopped: ${stop.reason}` +
                (stop.detail !== undefined ? ` (touched ${nameAddress(stop.detail)})` : "") +
                (stopFault ? ` -- the last fault graded ${stopFault.verdict.kind}: ${stopFault.verdict.why}` : "") +
                `; SIMH's own execution does not stop here, so this is a divergence, not a boundary`;
        }
    }
    /*
     * MEASURED (pcjsvax-320): hst.finish(cpu) is a NO-OP when hst.pending is already null
     * (simhtrace.js:420, `if (!p) return;`) -- and pending is null whenever the instruction that
     * faulted never reached hst.record()'s call point (cpustate.js's fnExecute calls record()
     * AFTER specifier resolution, BEFORE the body).  pcjsvax-223's original boundary (a STORE
     * faulting inside its own body) always left a record() already pending, so
     * "force-finalize, then trim the last record" was always correct.  Once this item's decode
     * lets that store SUCCEED, the boundary can now land on operand resolution of the NEXT
     * instruction -- a fault BEFORE record() ever runs for it -- and hst.count is ALREADY the
     * exact count of genuinely comparable instructions; trimming one more would silently drop the
     * last one that actually completed (here: the MOVL that stores to SSC+0x0 itself) from the
     * SIMH comparison. `finalizedPartial` distinguishes the two shapes by directly observing
     * whether finish() changed hst.count, rather than assuming which shape occurred.
     *
     * MEASURED CORRECTION (pcjsvax-fe7): "finish() changed hst.count" was a sufficient
     * discriminator only while the walk could ONLY stop on a fault.  It is not once the walk can
     * also stop on a BUDGET, because then hst.pending holds a record for an instruction that
     * COMPLETED NORMALLY -- finish() finalizes it, hst.count rises, and the old test would have
     * trimmed a genuinely comparable instruction out of the oracle comparison.  simhtrace.js
     * already records the fact that tells the two apart (`pending.completed`, simhtrace.js:363,
     * set only when the body ran to completion), so it is read directly instead of inferred.
     */
    let pendingIncomplete = !!(hst.pending && !hst.pending.completed);
    let countBeforeFinish = hst.count;
    hst.finish(cpu);
    let finalizedPartial = (hst.count > countBeforeFinish) && pendingIncomplete;
    let tracePath = path.join(opts.scratch, "romdiff-js.trace");
    fs.writeFileSync(tracePath, hst.text());
    /* THE point a walk actually peaks: hst.text() materialises every record of the whole walk as one
       string while the records themselves are still live.  Sampling only in main() would miss it. */
    sampleHeap();
    return {
        bus, cpu, tracePath, records: hst.count, steps, stop, finalizedPartial,
        unavailable: hst.unavailable,
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0,
        firstFaultAddr, firstFaultAccess, firstFaultPC,
        faults, faultEvents, absorbed, stopFault, stopKind, stopWhy,
        distinctFaultAddrs: ctx.faultCache.size, probeCacheHits: ctx.faultCacheHits
    };
}

/**
 * makeWalkCtx(simh, opts, romBytes, fOmitCdg)
 *
 * One walk's oracle-probe cache and its (single, reused) probe machine.  A fresh ctx per walk means
 * the PHASE F floor walk cannot inherit a verdict measured against a DIFFERENT device set -- the
 * cache is keyed by address, and CDG+0x0's verdict is precisely what --no-cdg changes.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {Uint8Array} romBytes
 * @param {boolean} fOmitCdg
 * @returns {Object}
 */
function makeWalkCtx(simh, opts, romBytes, fOmitCdg)
{
    return {simh, opts, romBytes, omitCdg: !!fOmitCdg,
            faultCache: new Map(), faultCacheHits: 0, probeMachine: null};
}

/** Same mulberry32 PRNG every VAX differential in this tree uses (busdiff.js, mchkdiff.js, ...),
    duplicated rather than imported because none of them share a utility module -- see those files. */
function mulberry32(a)
{
    return function() {
        a |= 0; a = (a + 0x6D2B79F5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

/**
 * verifySscBaseRandom(simh, opts, seed, n)
 *
 * pcjsvax-320's REAL-WORKLOAD phase (the boot trace above) stores exactly ONE value into
 * SSCBASE -- 0x20140000 -- which already satisfies SSCBASE_RW/SSCBASE_MBO exactly (it IS a valid
 * base address), so the AND/OR mask this file transcribes from vax_sysdev.c's ssc_wr() `case 0x00`
 * is STRUCTURALLY UNTESTED by that run alone: a transcription bug in either mask (e.g. a swapped
 * hex digit in SSCBASE_RW) that only misbehaves on OTHER bit patterns would pass the real-workload
 * phase silently.  This is exactly the class of bug the project's standing rule about randomized
 * phases exists to catch (see docs/design/vax-on-pcjs.md's "uniform random address pools" lesson,
 * applied here to register VALUES rather than addresses).
 *
 * Driven through a REAL instruction round trip on the live oracle -- `MOVL S^#val,@#SSCBASE` then
 * `MOVL @#SSCBASE,R0`, examined via R0 -- the same "only an actual instruction reproduces the real
 * access path" reasoning FaultGrader.probeSimh() is built on (deposit/examine of a NAMED register like
 * `sysd base` bypasses ssc_wr()'s masking entirely and was confirmed, by direct probe, to read back
 * whatever was deposited UNMASKED -- exactly the false-negative this function exists to avoid).
 * Compared against a bare `new SSCVAX()` instance's own writeReg()/readReg() -- not the whole bus --
 * because the mask formula is what is under test, not the address decode (romdiff's PHASE 1 trace
 * comparison already proves the address decode).
 *
 * Batched into ONE SIMH invocation (mchkdiff.js's runBatch() convention): each case is independent
 * via `reset all`, so a fault or bad value in one case cannot contaminate another.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {number} seed
 * @param {number} n
 * @returns {Array.<string>} problems (empty if none)
 */
function verifySscBaseRandom(simh, opts, seed, n)
{
    let rnd = mulberry32(seed);
    let opcMOVL = OPCODES.indexOf("MOVL");
    if (opcMOVL < 0) throw new Error("romdiff.js: MOVL not found in drom.js OPCODES");
    const CODE = 0x00104000;
    const SSCBASE = VAX.PHYSMEM.SSC_BASE >>> 0;
    const MARK = "SSCBASERND";

    /* A few boundary-interesting values FIRST (all-zero, all-one, exactly SSCBASE_RW, exactly
       SSCBASE_MBO), THEN n-4 uniform random longwords -- covers both the deliberately-chosen edges
       and the general case, same shape as every other randomized phase in this tree. */
    let vals = [0x00000000, 0xFFFFFFFF | 0, 0x1FFFFC00, 0x20000000 | 0];
    for (let i = vals.length; i < n; i++) vals.push((rnd() * 0x100000000) >>> 0);

    let lines = ["set cpu 16m", "set cpu simhalt"];
    for (let i = 0; i < vals.length; i++) {
        let v = vals[i] >>> 0;
        lines.push(`echo ${MARK}${i}`, "reset all");
        let instr = [
            opcMOVL & 0xFF, 0x8F, v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF,
                            0x9F, SSCBASE & 0xFF, (SSCBASE >>> 8) & 0xFF, (SSCBASE >>> 16) & 0xFF, (SSCBASE >>> 24) & 0xFF,
            opcMOVL & 0xFF, 0x9F, SSCBASE & 0xFF, (SSCBASE >>> 8) & 0xFF, (SSCBASE >>> 16) & 0xFF, (SSCBASE >>> 24) & 0xFF,
                            0x50                                              // R0, direct (MODE.GRN | 0)
        ];
        for (let k = 0; k < instr.length; k++) lines.push(`deposit -b ${hex(CODE + k)} ${instr[k].toString(16)}`);
        lines.push(`deposit PSL 0`, `deposit PC ${hex(CODE)}`, "step 2", "examine -h R0");
    }
    lines.push("exit", "");
    let out = runSimh(simh, lines.join("\n"), path.join(opts.scratch, "romdiff-sscbase-random.ini"));

    let problems = [];
    let marks = [...out.matchAll(new RegExp(MARK + "(\\d+)", "g"))].map((m) => +m[1]);
    let r0s = [...out.matchAll(/^R0:\s*([0-9A-Fa-f]+)/gm)].map((m) => parseInt(m[1], 16) >>> 0);
    if (marks.length !== vals.length || r0s.length !== vals.length) {
        problems.push(`SSC-BASE-RANDOM: expected ${vals.length} cases, SIMH produced ${marks.length} ` +
            `markers and ${r0s.length} R0 readbacks -- some case did not reach comparison; SIMH said:\n` +
            out.slice(0, 2000));
        return problems;
    }
    let ssc = new SSCVAX();
    for (let i = 0; i < vals.length; i++) {
        ssc.reset();
        ssc.writeReg(0x00, vals[i] | 0);
        let want = ssc.readReg(0x00) >>> 0;
        let got = r0s[i] >>> 0;
        if (got !== want) {
            problems.push(`SSC-BASE-RANDOM: case ${i} val=0x${hex(vals[i])} -- SIMH readback=0x${hex(got)}, ` +
                `SSCVAX readback=0x${hex(want)}`);
        }
    }
    return problems;
}

/**
 * FALLTHROUGH_CASES -- the fall-through semantics veracity finding (post-merge re-dispatch).
 *
 * WHAT THIS PROVES: an earlier version of ssc.js/cqbic.js/ka655.js's readReg()/writeReg() returned
 * null/false for ANY register number their own switch did not case, which makeSscController()/
 * makeRegController() (correctly, at the time) turned into a bus fault.  That is backwards for a
 * reference INSIDE a decoded device's own span: real KA655 hardware's regtable dispatch (vax_
 * sysdev.c's ReadReg()/WriteReg(), and vax_io.c's cqbic_wr()) has no `default:` case anywhere in
 * this tree's scope EXCEPT CQBICVAX's MEAR/SEAR write (a genuine, deliberate bus error) -- every
 * other "no case matched" is a SILENT NO-OP: the reference completes normally, nothing is stored
 * (or nothing changes on read), and PC simply advances to the next instruction.  Three concrete,
 * MEASURED divergences this item's own model had:
 *
 *   - A write to KA655's BDR (read-only: ka_wr()'s whole body is `if (rg==0 && aligned) {...}`,
 *     no else) -- SIMH's PC advances normally, BDR's own readback is UNCHANGED.
 *   - An UNALIGNED write to KA655's CACR (the SAME `(pa&3)==0` gate rejects it) -- SIMH's PC
 *     advances normally, CACR's own readback is UNCHANGED by the attempt.
 *   - A read AND a write of an uncased offset inside the SSC's own decoded span (SSC+0x50, rg=0x14,
 *     nothing in ssc_rd()/ssc_wr()'s switches names it) -- SIMH reads 0 and accepts the write with
 *     no state change, no fault either direction.
 *
 * Each case below is a REAL instruction sequence, executed for real on the live oracle exactly the
 * way verifySscBaseRandom()/FaultGrader.probeSimh() already do ("only an actual instruction reproduces
 * the real access path"), with SCBB pointed at a NOP-filled handler far from the code so that if
 * EITHER machine actually machine-checks, PC lands there instead of completing the sequence --
 * making "did a fault happen" a directly observable, compared fact, not an assumption.
 *
 * @typedef {Object} FallthroughCase
 * @property {string} name
 * @property {number[]} bytes    the instruction stream (NOPs appended by the runner)
 * @property {number} steps      exact instruction count in `bytes` (a BYTE count is NOT an
 *                                instruction count -- see consoledif.js's own history for the bug
 *                                class this avoids)
 * @property {number[]} probes   absolute addresses to read back (via `examine -h`) after stepping
 */
const FALLTHROUGH_CASES = (function() {
    const KABASE = (VAX.PHYSMEM.REG_BASE + 0x4000) >>> 0;
    const KA_BDR = (KABASE + 4) >>> 0;
    const KA_CACR = KABASE >>> 0;
    const SSC_UNCASED = (VAX.PHYSMEM.SSC_BASE + 0x50) >>> 0;   // rg=0x14 -- ssc.js does not case it
    const R_PROBE = 0x00120000;                                 // scratch RAM, distinct from R_CODE
    const opcMOVL = OPCODES.indexOf("MOVL"), opcMOVB = OPCODES.indexOf("MOVB"), opcTSTL = OPCODES.indexOf("TSTL");
    if (opcMOVL < 0 || opcMOVB < 0 || opcTSTL < 0) throw new Error("romdiff.js: MOVL/MOVB/TSTL not found in drom.js OPCODES");

    function movlImmToAbs(bytes, immVal, absAddr) {
        bytes.push(opcMOVL & 0xFF, 0x8F, immVal & 0xFF, (immVal >>> 8) & 0xFF, (immVal >>> 16) & 0xFF, (immVal >>> 24) & 0xFF);
        bytes.push(0x9F, absAddr & 0xFF, (absAddr >>> 8) & 0xFF, (absAddr >>> 16) & 0xFF, (absAddr >>> 24) & 0xFF);
    }
    function movlAbsToAbs(bytes, srcAddr, absAddr) {
        bytes.push(opcMOVL & 0xFF, 0x9F, srcAddr & 0xFF, (srcAddr >>> 8) & 0xFF, (srcAddr >>> 16) & 0xFF, (srcAddr >>> 24) & 0xFF);
        bytes.push(0x9F, absAddr & 0xFF, (absAddr >>> 8) & 0xFF, (absAddr >>> 16) & 0xFF, (absAddr >>> 24) & 0xFF);
    }
    function movbImmToAbs(bytes, immVal, absAddr) {
        bytes.push(opcMOVB & 0xFF, 0x8F, immVal & 0xFF);
        bytes.push(0x9F, absAddr & 0xFF, (absAddr >>> 8) & 0xFF, (absAddr >>> 16) & 0xFF, (absAddr >>> 24) & 0xFF);
    }

    let cases = [];

    /* KA655 BDR: a longword write, then read BDR back -- must be UNCHANGED, no fault. */
    {
        let bytes = [];
        movlImmToAbs(bytes, 0x00000000, KA_BDR);
        movlAbsToAbs(bytes, KA_BDR, R_PROBE);
        cases.push({name: "ka-bdr-write-readonly", bytes, steps: 2, probes: [R_PROBE]});
    }
    /* KA655 CACR: an UNALIGNED (offset+2) byte write, then read CACR back (aligned) -- must be
       UNCHANGED, no fault -- vax_sysdev.c's `(pa & 3) == 0` gate rejects it before ever looking at
       the data, and there is no else branch to fault on the rejection. */
    {
        let bytes = [];
        movbImmToAbs(bytes, 0xFF, (KA_CACR + 2) >>> 0);
        movlAbsToAbs(bytes, KA_CACR, R_PROBE);
        cases.push({name: "ka-cacr-unaligned-write", bytes, steps: 2, probes: [R_PROBE]});
    }
    /* SSC uncased offset: TSTL (read, sets CC on the value -- probes CC via PSL, not a memory
       probe) then a write-then-readback round trip, both against the SAME uncased register. */
    {
        let bytes = [];
        bytes.push(opcTSTL & 0xFF, 0x9F, SSC_UNCASED & 0xFF, (SSC_UNCASED >>> 8) & 0xFF, (SSC_UNCASED >>> 16) & 0xFF, (SSC_UNCASED >>> 24) & 0xFF);
        cases.push({name: "ssc-uncased-read", bytes, steps: 1, probes: [], probePsl: true});
    }
    {
        let bytes = [];
        movlImmToAbs(bytes, 0xFFFFFFFF | 0, SSC_UNCASED);
        movlAbsToAbs(bytes, SSC_UNCASED, R_PROBE);
        cases.push({name: "ssc-uncased-write-then-read", bytes, steps: 2, probes: [R_PROBE]});
    }

    return cases;
})();

/**
 * verifyFallthroughSemantics(simh, opts)
 *
 * Runs every FALLTHROUGH_CASES entry for real on the live oracle AND on this machine's own JS
 * (built fresh per case via makeMachine(), PC/PSL overridden to the scratch code exactly like
 * romdiff.js's MUTATIONS/selfcheck already do), and requires BOTH: (1) neither machine faults
 * (PC does not land in the NOP-filled handler page), and (2) every probed register/PSL value
 * matches exactly.  A case that only checks "no fault" without also checking the resulting VALUE
 * would not catch a stub that avoids faulting by inventing a wrong stored value -- see this file's
 * own MUTATIONS section for the same discipline applied to the SSC base register.
 *
 * @param {Uint8Array} romBytes
 * @param {string} simh
 * @param {Object} opts
 * @param {boolean} [fOmitCdg] keep the machine under test identical to the walk's (--no-cdg)
 * @returns {Array.<string>} problems (empty if none)
 */
function verifyFallthroughSemantics(romBytes, simh, opts, fOmitCdg)
{
    const R_SCBB = 0x00100000, R_HANDLER = 0x00102000, R_CODE = 0x00104000, R_KSP = 0x00110000;
    const NOP_BYTE = OPCODES.indexOf("NOP") & 0xFF;
    if (NOP_BYTE < 0) throw new Error("romdiff.js: NOP not found in drom.js OPCODES");
    const MARK = "FALLTHROUGH_";
    let problems = [];

    /* ---- SIMH side: one script, all cases, `reset all` between them (matches every other batched
       phase in this file). ---- */
    let lines = ["set cpu 16m", "set cpu simhalt"];
    for (let c of FALLTHROUGH_CASES) {
        lines.push("reset all");
        lines.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit -l ${hex(R_SCBB + 4)} ${hex(R_HANDLER)}`);
        lines.push(`deposit KSP ${hex(R_KSP)}`);
        for (let k = 0; k < 16; k++) lines.push(`deposit -b ${hex(R_HANDLER + k)} ${NOP_BYTE.toString(16)}`);
        for (let k = 0; k < c.bytes.length + 4; k++) lines.push(`deposit -b ${hex(R_CODE + k)} 0`);
        for (let i = 0; i < c.bytes.length; i++) lines.push(`deposit -b ${hex(R_CODE + i)} ${c.bytes[i].toString(16)}`);
        for (let k = 0; k < 4; k++) lines.push(`deposit -b ${hex((R_CODE + c.bytes.length + k) | 0)} ${NOP_BYTE.toString(16)}`);
        lines.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`);
        lines.push(`echo ${MARK}${c.name}`);
        lines.push(`step ${c.steps}`, "examine -h PC");
        if (c.probePsl) lines.push("examine -h PSL");
        for (let p of c.probes) lines.push(`examine -h ${hex(p)}`);
    }
    lines.push("exit", "");
    let out = runSimh(simh, lines.join("\n") + "\n", path.join(opts.scratch, "romdiff-fallthrough.ini"));

    let simhResults = new Map();
    {
        let out_lines = out.split("\n");
        let i = 0;
        while (i < out_lines.length) {
            let m = out_lines[i].match(new RegExp(MARK + "(\\S+)"));
            if (!m) { i++; continue; }
            let name = m[1];
            let c = FALLTHROUGH_CASES.find((cc) => cc.name === name);
            let want = 1 + (c.probePsl ? 1 : 0) + c.probes.length;      // PC, [PSL], probes
            i++;
            let vals = [];
            while (i < out_lines.length && vals.length < want) {
                if (out_lines[i].indexOf(MARK) >= 0) break;
                let vm = out_lines[i].match(/^\S+:\s+([0-9A-Fa-f]+)/);
                if (vm) vals.push(parseInt(vm[1], 16) >>> 0);
                i++;
            }
            simhResults.set(name, vals);
        }
    }

    /* ---- JS side: fresh machine per case, same instruction bytes, same layout. ---- */
    for (let c of FALLTHROUGH_CASES) {
        let simhVals = simhResults.get(c.name);
        let want = 1 + (c.probePsl ? 1 : 0) + c.probes.length;
        if (!simhVals || simhVals.length < want) {
            problems.push(`FALLTHROUGH ${c.name}: SIMH produced ${simhVals ? simhVals.length : 0}/${want} readback(s) -- case did not reach comparison`);
            continue;
        }
        let simhPC = simhVals[0];
        let simhFaulted = simhPC >= R_HANDLER && simhPC < R_HANDLER + 16;
        if (simhFaulted) {
            problems.push(`FALLTHROUGH ${c.name}: the REAL ORACLE ITSELF machine-checked (PC=${hex(simhPC)}) -- this case's own premise (SIMH tolerates this reference) is wrong; fix the fixture, not the model`);
            continue;
        }

        let {bus, cpu} = makeMachine(romBytes, fOmitCdg);
        cpu.exc.scbb = R_SCBB;
        cpu.exc.stk[0] = R_KSP;                 // KERN
        cpu.regs.fill(0);
        cpu.regs[14] = R_KSP;
        for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
        for (let k = 0; k < c.bytes.length + 4; k++) bus.setByte((R_CODE + k) | 0, 0);
        for (let i2 = 0; i2 < c.bytes.length; i2++) bus.setByte((R_CODE + i2) | 0, c.bytes[i2]);
        for (let k = 0; k < 4; k++) bus.setByte((R_CODE + c.bytes.length + k) | 0, NOP_BYTE);
        cpu.psl = 0;
        cpu.regs[15] = R_CODE;

        let jsFaulted = false, stopReason = null;
        try {
            for (let s = 0; s < c.steps; s++) cpu.stepCPU(1);
        } catch (e) {
            jsFaulted = true;
            stopReason = e && e.reason ? e.reason : String(e);
        }
        let jsPC = cpu.regs[15] >>> 0;
        if (!jsFaulted) jsFaulted = jsPC >= R_HANDLER && jsPC < R_HANDLER + 16;

        if (jsFaulted) {
            problems.push(`FALLTHROUGH ${c.name}: JS machine-checked where the real oracle did not (PC=${hex(jsPC)}${stopReason ? `, ${stopReason}` : ""}) -- this is the exact class of divergence this case exists to catch`);
            continue;
        }
        if ((jsPC >>> 0) !== (simhPC >>> 0)) {
            problems.push(`FALLTHROUGH ${c.name}: PC js=${hex(jsPC)} simh=${hex(simhPC)}`);
        }
        let idx = 1;
        if (c.probePsl) {
            let jsPsl = cpu.psl >>> 0, simhPsl = simhVals[idx++];
            if (jsPsl !== simhPsl) problems.push(`FALLTHROUGH ${c.name}: PSL js=${hex(jsPsl)} simh=${hex(simhPsl)}`);
        }
        for (let p of c.probes) {
            let jsVal = bus.getLong(p) >>> 0, simhVal = simhVals[idx++];
            if (jsVal !== simhVal) problems.push(`FALLTHROUGH ${c.name}: probe@${hex(p)} js=${hex(jsVal)} simh=${hex(simhVal)}`);
        }
    }

    return problems;
}

/*
 * pcjsvax-23c FIX: a MUCH shorter timeout than runSimh()'s 5-minute default, specific to this call
 * site, bounding the worst case to seconds rather than the 5-minute/9.5GB-scratch-directory failure
 * that item measured and fixed.  pcjsvax-fe7 keeps it, but it is no longer the ONLY thing bounding
 * trace growth: the capture is now bounded by an explicit STEP COUNT that can never exceed the
 * history ring, so the file's size is bounded by construction and this timeout is a backstop.
 */
const CAPTURE_TIMEOUT_MS = 120 * 1000;

/** REGS lines are one-per-record in the trace-oracle patch's output (see cpu_show_hist_records). */
function countTraceRecords(tracePath)
{
    let n = 0, fd = fs.openSync(tracePath, "r");
    try {
        const CHUNK = 1 << 20;
        let buf = Buffer.allocUnsafe(CHUNK), carry = "";
        for (;;) {
            let got = fs.readSync(fd, buf, 0, CHUNK, null);
            if (!got) break;
            let text = carry + buf.toString("latin1", 0, got);
            let lines = text.split("\n");
            carry = lines.pop();
            for (let l of lines) if (l.startsWith("REGS")) n++;
        }
        if (carry.startsWith("REGS")) n++;
    } finally { fs.closeSync(fd); }
    return n;
}

/**
 * captureSimhTrace(simh, opts, neededRecords)
 *
 * A REAL `boot cpu` -- see the file header for why a hand-built deposit is not acceptable -- stopped
 * at the FIRST instruction by a breakpoint on the ROM entry, and then advanced by an EXACT step
 * count.  `boot` itself cannot be given a step count (scp.c run_cmd() resets sim_step to 0 for
 * RU_BOOT), which is why the breakpoint is still here; what it bounds is now zero instructions, not
 * the whole run.
 *
 * WHY NOT KEEP BOUNDING IT AT THE JS RUN'S STOPPING PC (pcjsvax-fe7): a breakpoint fires the FIRST
 * time SIMH fetches that address, and once the walk can run past absorbed faults the stopping PC is
 * routinely inside a loop the ROM has already executed -- the SOBGTR delay loop pcjsvax-93e
 * measured is one instruction branching to itself.  A PC is no longer a position in the run.  A step
 * count is.
 *
 * A STEP COUNT IS NOT A RECORD COUNT: an instruction attempt that aborts before vax_cpu.c:1632
 * consumes a step and records nothing (MEASURED: 3000 steps -> 2486 records from a cold boot).  So
 * this asks for RECORDS and raises the step count until it has enough, rather than assuming a ratio
 * -- and refuses to raise it past the ring length, because a wrap is what manufactures phantom
 * divergences (file header).
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {number} neededRecords records that must exist in the capture (compared range + margin)
 * @returns {{tracePath: string, records: number, steps: number}}
 */
function captureSimhTrace(simh, opts, neededRecords)
{
    let tracePath = path.join(opts.scratch, "romdiff-simh.trace");
    if (neededRecords > SIMH_HIST_LNT) {
        throw new Error(`romdiff: the comparison needs ${neededRecords} SIMH records, but SIMH's own ` +
            `HIST_MAX is ${SIMH_HIST_LNT} (vax_defs.h:852) -- a longer capture would WRAP the history ` +
            `ring, and a wrapped ring writes one unfilled result record per pass (vax_cpu.c:1676), ` +
            `which manufactures phantom divergences.  Lower --max-steps.`);
    }
    let steps = Math.min(SIMH_STEPS_MAX, neededRecords), records = 0, out = "";
    for (let attempt = 0; attempt < 4; attempt++) {
        if (fs.existsSync(tracePath)) fs.unlinkSync(tracePath);
        let script = [
            "set cpu 16m",
            `set cpu history=${SIMH_HIST_LNT}:${tracePath}`,
            `load -r ${opts.rom}`,
            `break ${hex(VAX.PHYSMEM.ROM_BASE >>> 0)}`,
            "boot cpu",
            `nobreak ${hex(VAX.PHYSMEM.ROM_BASE >>> 0)}`,
            `step ${steps}`,
            "examine PC",
            "exit", ""
        ].join("\n");
        try {
            out = runSimh(simh, script, path.join(opts.scratch, "romdiff-simh.ini"), CAPTURE_TIMEOUT_MS);
        } catch (e) {
            throw new Error(`romdiff: SIMH did not finish ${steps} steps within ${CAPTURE_TIMEOUT_MS}ms ` +
                `and was killed (${e.code || e.message}); it is not the previous 9.5GB/5-minute failure ` +
                `mode -- the capture is step-bounded now -- so this is a real slowdown, not a runaway.`);
        }
        if (!/Breakpoint/.test(out)) {
            throw new Error(`romdiff: SIMH never stopped at the ROM entry ${hex(VAX.PHYSMEM.ROM_BASE >>> 0)} ` +
                `after 'boot cpu' -- the capture has no defined starting point; SIMH said:\n` + out);
        }
        records = countTraceRecords(tracePath);
        /* THE INVARIANT THIS WHOLE FUNCTION EXISTS FOR, asserted rather than reasoned about: the
           ring must not have wrapped, because a wrapped ring writes one unfilled-result record per
           pass and those are indistinguishable from real divergences.  VALIDATED by direct
           measurement (pcjsvax-fe7): the same 3000-step capture taken with rings of 1000 and 1001
           differs from a non-wrapping 250000-entry capture at exactly one extra record each, at
           index 1999 (index%1000 == 999) and index 2001 (index%1001 == 1000) respectively -- the
           wrap boundaries, moving with the ring length exactly as vax_cpu.c predicts.  The only
           other difference in either pair is index 32, an `MFPR #1B` (TODR) whose value is
           real-time-dependent and differs between ANY two SIMH runs. */
        if (records >= SIMH_HIST_LNT || steps >= SIMH_HIST_LNT) {
            throw new Error(`romdiff: the capture reached ${records} records in ${steps} steps, at or past ` +
                `SIMH's ${SIMH_HIST_LNT}-entry history ring -- the ring WRAPPED, and a wrapped ring writes ` +
                `one unfilled-result record per pass (vax_cpu.c:1676) that is indistinguishable from a real ` +
                `divergence.  This capture is not trustworthy and is refused rather than compared.`);
        }
        if (records >= neededRecords) break;
        /* Raise the step count IN PROPORTION to the shortfall, not by a blind multiplier: the
           step-to-record ratio is close to 1 once the ROM leaves its early fault-heavy stretch
           (MEASURED: 0.83 over the first 3000 steps, 0.998 over 250000), so a 1.6x retry overshot
           straight into the ring ceiling and spent the entire safety margin on a 500-record miss. */
        let next = Math.min(SIMH_STEPS_MAX, Math.ceil(steps * neededRecords / Math.max(records, 1)) + 1024);
        if (next <= steps) break;
        steps = next;
    }
    if (records < neededRecords) {
        throw new Error(`romdiff: SIMH produced only ${records} trace records in ${steps} steps, but the ` +
            `comparison needs ${neededRecords} (the walk's ${neededRecords - SIMH_TRACE_MARGIN} compared ` +
            `records plus ${SIMH_TRACE_MARGIN} of margin past them).  Either SIMH stopped early (it said: ` +
            `${out.split("\n").filter((l) => /HALT|Breakpoint|Step|error/i.test(l)).join(" | ")}) or the ` +
            `step-to-record ratio collapsed; either way the walk cannot be graded this far.`);
    }
    return {tracePath, records, steps};
}

/**
 * The two normalized record classes, computed exactly as cpudiff.js does (imported nowhere because
 * it is four lines, not because it is not shared logic -- see that file for the full rationale).
 */
const ZEROSPEC = (function() {
    let s = new Set();
    for (let opc = 0; opc < 512; opc++) {
        if (!OPCODES[opc]) continue;
        let hdr = DROM[opc * DROM_STRIDE];
        if ((hdr & DR.NSPMASK) === 0 && ((hdr >> 4) & 0x7) > 0) s.add(OPCODES[opc]);
    }
    return s;
})();

const DIFF_DRIVER = `#!/usr/bin/env python3
import sys, os, json
differ_path, js_path, simh_path, unav_path, zerospec, max_js_records = sys.argv[1:7]
sys.path.insert(0, os.path.dirname(differ_path))
from differ import parse_trace, diff_traces

ZEROSPEC = set(x for x in zerospec.split(",") if x)
unavailable = set(json.load(open(unav_path)))

a = parse_trace(js_path)
# max_js_records < 0 means "no trim" (the ordinary case).  A boundary case passes len(a) - 1: the
# JS trace's LAST record is the boundary instruction itself, force-finalized by runRomJS()'s
# unconditional hst.finish(cpu) call after the exception even though its body never completed (it
# aborted mid-store) -- so it is not a real, comparable instruction and must not be counted against
# SIMH's capture, which (correctly bounded at the boundary instruction's OWN start PC) never
# executes it at all.
max_js_records = int(max_js_records)
if max_js_records >= 0:
    a = a[:max_js_records]
buf = []
n = 0
with open(simh_path, errors="replace") as f:
    for line in f:
        buf.append(line)
        if line.startswith("REGS"):
            n += 1
            if n >= len(a):
                break
tmp = simh_path + ".trunc"
open(tmp, "w").writelines(buf)
b = parse_trace(tmp)
os.unlink(tmp)

n_cmpd = n_zero = n_unav = 0
for recs in (a, b):
    for i, r in enumerate(recs):
        if r.mnemonic == "CMPD" and " ->" in r.detail:
            r.detail = r.detail.split(" ->")[0]; n_cmpd += 1
        elif r.mnemonic in ZEROSPEC:
            r.detail = r.mnemonic; n_zero += 1
        elif i in unavailable and " ->" in r.detail:
            r.detail = r.detail.split(" ->")[0]; n_unav += 1

d = diff_traces(a[:len(b)], b)
print(json.dumps({
    "js_records": len(a), "simh_records": len(b),
    "normalized_cmpd": n_cmpd, "normalized_zerospec": n_zero, "normalized_unavailable": n_unav,
    "match": d is None,
    "divergence": None if d is None else d.to_dict(),
}))
`;

/**
 * compareTraces(jsPath, simhPath, unavailable, opts, maxJsRecords)
 *
 * @param {number} [maxJsRecords] trim the JS trace to this many records before comparing (used
 *   for the boundary case -- see DIFF_DRIVER's comment); omit or pass a negative number for no trim
 * @returns {Object}
 */
function compareTraces(jsPath, simhPath, unavailable, opts, maxJsRecords = -1)
{
    let driver = path.join(opts.scratch, "romdiff-driver.py");
    fs.writeFileSync(driver, DIFF_DRIVER);
    let unavPath = path.join(opts.scratch, "romdiff-unavailable.json");
    fs.writeFileSync(unavPath, JSON.stringify(unavailable));
    let out = execFileSync("python3", [driver, findDiffer(), jsPath, simhPath, unavPath,
                                       [...ZEROSPEC].join(","), String(maxJsRecords)],
                           {encoding: "utf8", maxBuffer: 1 << 28});
    return JSON.parse(out);
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 2 -- the mirror                                                                         *
 * ------------------------------------------------------------------------------------------- */

/**
 * verifyMirrorJS(romBytes, opts, magicByte)
 *
 * Checks the mirror against the PRIMARY half on THIS machine, at several offsets including both
 * boundaries, before AND after a write.
 *
 * CORRECTED (post-dispatch veracity review): the ORIGINAL version of this function proved
 * liveness by calling `cpu.boot(magicByte)` and checking that the mirror picked up the magic
 * byte at +4.  That check is VACUOUS as shipped: ka655x.bin already ships 0x02 at offset 4, and
 * the measured magic byte is ALSO 2 (see cpustate.js's ROM_MAGIC_BYTE comment), so boot() writes
 * the SAME value that was already there.  A load-time snapshot-copy mirror (one array per half,
 * copied once from the file, never re-read) passes that check identically to a genuinely live
 * alias -- there is nothing to distinguish them, because nothing actually changed.  This was
 * caught by an adversarial re-run that built exactly that snapshot-copy mirror and confirmed it
 * passed.  The magic-byte-after-boot check below is kept as an ordinary integration assertion
 * (boot() really does write what it claims), but it is NOT what proves the mirror is live.
 *
 * The actual liveness proof is the DISTINCT SENTINEL below: a byte written directly via
 * setByteDirect(), chosen to differ from whatever is already at that offset (so the check cannot
 * degenerate the same way), with the mirror required to track it and the original content
 * restored afterward.  This is the same technique the "magic-byte-not-written" selfcheck mutation
 * already uses to get an observable divergence; it just was not applied here.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 * @param {boolean} [fOmitCdg] keep the machine under test identical to the walk's (--no-cdg)
 * @returns {Array.<string>} problems (empty if none)
 */
function verifyMirrorJS(romBytes, opts, magicByte, fOmitCdg)
{
    let problems = [];
    let {bus, cpu} = makeMachine(romBytes, fOmitCdg);
    let base = VAX.PHYSMEM.ROM_BASE >>> 0;
    let size = VAX.PHYSMEM.ROM_SIZE;
    let offsets = [0, 1, 4, 0x100, size >> 1, size - 2, size - 1];
    for (let off of offsets) {
        let primary = bus.getByte((base + off) >>> 0);
        let mirror = bus.getByte((base + size + off) >>> 0);
        if (primary !== mirror) {
            problems.push(`mirror BEFORE any write at +0x${off.toString(16)}: primary=0x${hex(primary, 2)} mirror=0x${hex(mirror, 2)}`);
        }
        if (primary !== romBytes[off]) {
            problems.push(`primary at +0x${off.toString(16)} is 0x${hex(primary, 2)}, expected the ROM file's own 0x${hex(romBytes[off], 2)}`);
        }
    }

    /*
     * THE LIVENESS PROOF.  A byte guaranteed to actually CHANGE, at an offset the boot sequence
     * never touches (so it cannot collide with the magic-byte check below), written through the
     * same Direct accessor boot() itself uses.  If the primary and its ORIGINAL content happen to
     * already equal 0x37, use 0x5A instead -- the whole point is that the write is observable.
     */
    let sentinelOff = 0x40;
    let sentinelAddr = (base + sentinelOff) >>> 0;
    let before = bus.getByte(sentinelAddr);
    let sentinel = (before === 0x37) ? 0x5A : 0x37;
    bus.setByteDirect(sentinelAddr, sentinel);
    let primarySentinel = bus.getByte(sentinelAddr);
    let mirrorSentinel = bus.getByte((base + size + sentinelOff) >>> 0);
    if (primarySentinel !== sentinel) {
        problems.push(`primary +0x${sentinelOff.toString(16)} after a direct sentinel write is 0x${hex(primarySentinel, 2)}, expected 0x${hex(sentinel, 2)}`);
    }
    if (mirrorSentinel !== sentinel) {
        problems.push(`mirror +0x${sentinelOff.toString(16)} after a direct sentinel write to the PRIMARY is 0x${hex(mirrorSentinel, 2)}, expected it to track 0x${hex(sentinel, 2)} -- ` +
            `the mirror is a load-time COPY, not a live alias (this is exactly the check a snapshot-copy mirror fails and the magic-byte check above cannot distinguish)`);
    }
    bus.setByteDirect(sentinelAddr, before);                  // restore -- this function must not leave the machine dirty

    /* Ordinary integration check: boot() really does write what it claims.  NOT a liveness proof
       on its own (see the file-header note above) -- ka655x.bin already ships 0x02 at +4 and the
       measured magic byte is also 2, so this can pass even against a snapshot-copy mirror. */
    cpu.reset();
    cpu.boot(magicByte);
    let primary4 = bus.getByte((base + 4) >>> 0);
    let mirror4 = bus.getByte((base + size + 4) >>> 0);
    if (primary4 !== magicByte) {
        problems.push(`primary +4 after boot() is 0x${hex(primary4, 2)}, expected the magic byte 0x${hex(magicByte, 2)}`);
    }
    if (mirror4 !== magicByte) {
        problems.push(`mirror +4 after boot() is 0x${hex(mirror4, 2)}, expected it to track the primary's magic byte 0x${hex(magicByte, 2)}`);
    }
    return problems;
}

/**
 * verifyMirrorSimh(simh, opts, magicByte)
 *
 * Same claim, against SIMH: examine the primary and mirror halves at the same offsets, both before
 * and after a real `boot cpu` (bounded the same way the trace capture is), and require every pair
 * to agree with itself AND with what verifyMirrorJS found -- this is what proves the two
 * implementations mean the same thing by "mirrored", not just that each is internally consistent.
 *
 * pcjsvax-fe7: the bounding breakpoint is now the ROM ENTRY, not the JS walk's stopping PC.  What
 * this function needs from `boot cpu` is only that vax_sysdev.c's cpu_boot() has run (that is what
 * writes the model magic byte at ROMBASE+4) -- which is complete the instant the first instruction
 * is fetched -- and a stopping PC is no longer a usable bound anyway now that the walk can run past
 * absorbed faults into code the ROM has already executed (see captureSimhTrace()).
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Object} {beforeBoot: [[primary,mirror],...], afterBoot: {primary4, mirror4}}
 */
function verifyMirrorSimh(simh, opts)
{
    let base = VAX.PHYSMEM.ROM_BASE >>> 0;
    let size = VAX.PHYSMEM.ROM_SIZE;
    let offsets = [0, 1, 4, 0x100, size >> 1, size - 2, size - 1];
    let lines = ["set cpu 16m", `load -r ${opts.rom}`];
    for (let off of offsets) {
        lines.push(`e -b ${hex((base + off) >>> 0)}`);
        lines.push(`e -b ${hex((base + size + off) >>> 0)}`);
    }
    lines.push(`break ${hex(base)}`, "boot cpu",
               `e -b ${hex((base + 4) >>> 0)}`, `e -b ${hex((base + size + 4) >>> 0)}`, "exit", "");
    /* pcjsvax-23c: same bounded timeout as captureSimhTrace() -- this call has the identical
       "breakpoint SIMH's own execution never reaches" risk profile. */
    let out = runSimh(simh, lines.join("\n"), path.join(opts.scratch, "romdiff-mirror.ini"), CAPTURE_TIMEOUT_MS);
    let vals = [...out.matchAll(/^[0-9A-F]+:\s+([0-9A-F]+)/gm)].map((m) => parseInt(m[1], 16));
    if (vals.length < offsets.length * 2 + 2) {
        throw new Error("romdiff: SIMH mirror probe produced too few results; SIMH said:\n" + out);
    }
    let beforeBoot = [];
    for (let i = 0; i < offsets.length; i++) beforeBoot.push({off: offsets[i], primary: vals[2 * i], mirror: vals[2 * i + 1]});
    let afterBoot = {primary4: vals[offsets.length * 2], mirror4: vals[offsets.length * 2 + 1]};
    return {beforeBoot, afterBoot};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 3 -- --selfcheck                                                                         *
 * ------------------------------------------------------------------------------------------- */

/**
 * MUTATIONS -- one function each, applied to the SHIPPED objects (BusVAX.makeRomAliasController,
 * CPUStateVAX.prototype.boot, MemoryVAX write-protection), returning an undo closure.  Every one
 * of these is the named-mutation list done condition 5 requires, at minimum.
 *
 * TWO mirror mutations, deliberately, not one, after a veracity review found the first one alone
 * was not distinguishing what it claimed to: "mirror-not-aliased" (a ZERO-FILLED buffer) is only
 * caught because 0x00 happens to differ from whatever byte is really there -- it proves "the
 * mirror reads garbage", not "the mirror is a stale copy".  "mirror-stale-copy" (a genuine
 * snapshot, `Int32Array.from(block.adw)`, taken at construction time) is what actually exercises
 * the failure mode this item's commit message claims to guard against: a mirror that read
 * correctly at load time and then silently stopped tracking the primary.  Both are checked with a
 * DISTINCT SENTINEL write (see below), never with the boot-time magic byte -- ka655x.bin already
 * ships 0x02 at offset 4 and the measured magic byte is ALSO 2, so a check keyed on that offset
 * cannot distinguish a live alias from a snapshot that merely happened to copy the right value
 * once.
 */
const MUTATIONS = {
    /* The mirror aliases nothing: each mirror block gets its OWN zero-filled buffer instead of the
       primary block's array. */
    "mirror-not-aliased": (cpu, bus) => {
        let orig = BusVAX.makeRomAliasController;
        BusVAX.makeRomAliasController = function() {
            let fake = new Int32Array(VAX.PHYSMEM.ROM_SIZE >> 2);
            return {
                getControllerBuffer() { return [fake, 0]; },
                getControllerAccess() {
                    return [
                        MemoryVAX.prototype.readByteMemory, undefined,
                        MemoryVAX.prototype.readWordMemory, undefined,
                        MemoryVAX.prototype.readLongMemory, undefined
                    ];
                }
            };
        };
        return () => { BusVAX.makeRomAliasController = orig; };
    },
    /* The mirror is a load-time SNAPSHOT of the primary -- correct at the moment addRom() builds
       it (the copy is taken after the ROM file's bytes are already loaded into the primary), but
       frozen from then on.  Every check that only reads INITIAL content, or that re-writes a value
       already present, passes this identically to a true alias; only a write of something NEW,
       read back through the mirror, can tell them apart. */
    "mirror-stale-copy": (cpu, bus) => {
        let orig = BusVAX.makeRomAliasController;
        BusVAX.makeRomAliasController = function(busArg) {
            return {
                getControllerBuffer(addr) {
                    let primaryAddr = ((addr >>> 0) - VAX.PHYSMEM.ROM_SIZE) >>> 0;
                    let block = busArg.aMemBlocks[primaryAddr >>> busArg.nBlockShift];
                    return [Int32Array.from(block.adw), 0];      // a COPY, not a reference
                },
                getControllerAccess() {
                    return [
                        MemoryVAX.prototype.readByteMemory, undefined,
                        MemoryVAX.prototype.readWordMemory, undefined,
                        MemoryVAX.prototype.readLongMemory, undefined
                    ];
                }
            };
        };
        return () => { BusVAX.makeRomAliasController = orig; };
    },
    /* boot() never writes the model magic byte at all. */
    "magic-byte-not-written": (cpu, bus) => {
        let orig = CPUStateVAX.prototype.boot;
        CPUStateVAX.prototype.boot = function() {
            this.setPC(VAX.PHYSMEM.ROM_BASE);
            this.psl = (1 << 26 | 0x1F << 16) >>> 0;
        };
        return () => { CPUStateVAX.prototype.boot = orig; };
    },
    /* boot() sets the wrong PSL -- IPL 0 instead of 1F, so the very first instruction's own PSL
       field already disagrees with SIMH's real cpu_boot(). */
    "wrong-boot-psl": (cpu, bus) => {
        let orig = CPUStateVAX.prototype.boot;
        CPUStateVAX.prototype.boot = function(magicByte) {
            this.setPC(VAX.PHYSMEM.ROM_BASE);
            this.psl = (1 << 26) >>> 0;                      // PSL_IS, IPL dropped to 0
            this.bus.setByteDirect((VAX.PHYSMEM.ROM_BASE + 4) >>> 0, (magicByte === undefined ? ROM_MAGIC_BYTE : magicByte) & 0xFF);
        };
        return () => { CPUStateVAX.prototype.boot = orig; };
    },
    /* The ROM accepts a NORMAL (non-Direct) write -- the thing that must never happen once the
       machine is running, however real hardware would react to it. */
    "rom-writable": (cpu, bus) => {
        let addr = (VAX.PHYSMEM.ROM_BASE + 0x100) >>> 0;
        let block = bus.getBlock(addr);
        let orig = block.writeByte;
        block.writeByte = block.writeByteDirect;
        return () => { block.writeByte = orig; };
    },
    /* pcjsvax-320: the SSC base register is no longer decoded at all -- SSCVAX.writeReg() stops
       recognizing REG_BASE, so a store to SSC+0x0 (this item's whole reason for existing) falls
       through to a bus fault exactly as it did before this item. */
    "ssc-base-not-decoded": (cpu, bus) => {
        let orig = SSCVAX.prototype.writeReg;
        SSCVAX.prototype.writeReg = function() { return false; };
        return () => { SSCVAX.prototype.writeReg = orig; };
    },
    /* pcjsvax-320: the SSC base register is decoded, but WITHOUT vax_sysdev.c's SSCBASE_RW/
       SSCBASE_MBO masking -- a store of a raw value would corrupt the must-be-one bits, which real
       hardware never allows software to clear. */
    "ssc-base-wrong-mask": (cpu, bus) => {
        let orig = SSCVAX.prototype.writeReg;
        SSCVAX.prototype.writeReg = function(rg, val) {
            if (rg === 0x00) { this.base = val | 0; return true; }
            return orig.call(this, rg, val);
        };
        return () => { SSCVAX.prototype.writeReg = orig; };
    },
    /* pcjsvax-320: BoundaryAccounting.compute() reverts to unconditionally trimming one record --
       the bug this item's own boundary-advance exposed (see runRomJS()'s and main()'s doc
       comments): correct only when the boundary instruction's fault happened inside its BODY
       (a force-finalized partial record), wrong when it happened during specifier resolution of
       the NEXT instruction (nothing was ever finalized, so js.records is already exact). */
    "boundary-off-by-one": (cpu, bus) => {
        let orig = BoundaryAccounting.compute;
        BoundaryAccounting.compute = function(records) {
            let comparableRecords = records - 1;
            return {comparableRecords, instrNum: comparableRecords + 1};
        };
        return () => { BoundaryAccounting.compute = orig; };
    },
    /* pcjsvax-320 VERACITY FINDING A, carried forward to pcjsvax-fe7's StopReported: the
       "every run NAMES where and why it stopped" invariant (HANDOFF.md standing rule 13) stops
       being enforced -- a run that names nothing, or names a kind report() cannot print, would
       exit clean again, which is exactly the regression that rule exists for. */
    "stop-reported-not-enforced": (cpu, bus) => {
        let orig = StopReported.check;
        StopReported.check = () => null;
        return () => { StopReported.check = orig; };
    },
    /*
     * pcjsvax-fe7, MUTATION 1 OF 3 -- THE TEMPTING WRONG VERSION OF THIS CHANGE.  The graded-
     * equality rule collapses to "both faulted", which is what a walk-past-faults change looks like
     * when it is written for the outcome instead of for the claim: the IOPAGE probe keeps working,
     * the gate goes green, and a JS fault that dispatches to a DIFFERENT SCB vector, or lands on a
     * different PC/PSL, than the oracle's is silently walked past forever.
     */
    "graded-equality-weakened-to-both-faulted": (cpu, bus) => {
        let orig = FaultGrader.gradeEqual;
        FaultGrader.gradeEqual = function(jsP, simhP) {
            return {equal: jsP.faulted === simhP.faulted, why: "both faulted (weakened)"};
        };
        return () => { FaultGrader.gradeEqual = orig; };
    },
    /*
     * pcjsvax-fe7, MUTATION 2 OF 3.  The per-address oracle probe cache stops distinguishing
     * addresses, so the FIRST verdict answers for every later one.  This is the failure mode that
     * makes the walk look fastest and be most wrong: one cheap probe, and every subsequent fault --
     * including a real undecoded-hardware gap -- inherits a verdict measured somewhere else.
     */
    "probe-cache-stale-address": (cpu, bus) => {
        let orig = FaultGrader.cacheKey;
        FaultGrader.cacheKey = function() { return "STALE"; };
        return () => { FaultGrader.cacheKey = orig; };
    },
    /*
     * pcjsvax-fe7, MUTATION 3 OF 3.  The walk keeps going past ANY fault, including one the oracle
     * SERVICES -- i.e. a real undecoded-hardware boundary.  This is the "green and blind" failure
     * HANDOFF.md standing rule 13 was written for; PHASE F (the regression floor) is the live
     * end-to-end proof that the unmutated code stops, and this mutation proves the decision that
     * floor depends on lives in WalkDecision and is what the walk actually consults.
     */
    "walk-continues-past-serviced-fault": (cpu, bus) => {
        let orig = WalkDecision.shouldContinue;
        WalkDecision.shouldContinue = function() { return true; };
        return () => { WalkDecision.shouldContinue = orig; };
    },
    /*
     * pcjsvax-bfb VERACITY FINDING (post-merge re-dispatch): reverts SSCVAX's fall-through fix to
     * its ORIGINAL, broken shape -- every uncased register faults again, INCLUDING the genuine
     * hardware gaps (SSC+0x50, KA655 BDR/misaligned-CACR) FALLTHROUGH_CASES exists to grade.  This
     * is the exact regression the live verifyFallthroughSemantics() phase (run on every ordinary
     * invocation, not merely --selfcheck) was built to catch; this mutation proves that phase would
     * actually catch it, fast, without spending another live SIMH round trip on every selfcheck run
     * -- the check below calls SSCVAX's OWN readReg()/writeReg() directly against a genuine gap
     * (rg=0x14, matching the SSC+0x50 fixture) and asserts they no longer return the silent
     * (0 / true) answer FALLTHROUGH_CASES's own live grading already proved correct.
     */
    "fallthrough-reverts-to-blanket-fault": (cpu, bus) => {
        let origRead = SSCVAX.prototype.readReg, origWrite = SSCVAX.prototype.writeReg;
        SSCVAX.prototype.readReg = function(rg) { return null; };
        SSCVAX.prototype.writeReg = function(rg, val) { return false; };
        return () => { SSCVAX.prototype.readReg = origRead; SSCVAX.prototype.writeReg = origWrite; };
    }
};

/**
 * selfcheck(romBytes, opts, magicByte)
 *
 * Every mutation is checked with a FAST, DETERMINISTIC, structural assertion rather than a second
 * full SIMH trace run: none of these is a subtle multi-instruction timing bug (that is what
 * cpudiff.js's own --selfcheck already proves this loop can catch); each is directly observable as a
 * single readback or a single pure-function call, and re-invoking SIMH once per mutation would cost
 * real wall-clock time for no additional detection power.
 *
 * ONE EXCEPTION, and it is deliberate: "probe-cache-stale-address" DOES spend two live oracle
 * probes, because a cache that answers for the wrong address is not observable to any check that
 * does not measure two genuinely different addresses for real.  Asserting it against hand-written
 * verdict objects would be asserting against this file's own idea of the answer -- which is the
 * "mutation substitutes its own copy" failure HANDOFF.md standing rule 11 forbids.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 * @param {string} simh needed by "probe-cache-stale-address", the ONE mutation whose defect is only
 *   observable against the live oracle -- a cache that answers for the wrong address is invisible to
 *   any check that does not measure two genuinely different addresses for real
 */
function selfcheck(romBytes, opts, magicByte, simh)
{
    console.log("\nSELFCHECK -- each mutation must be caught\n");
    /*
     * Both mirror mutations patch BusVAX.makeRomAliasController, which addRom() only CONSULTS at
     * construction time -- so unlike the other two (which patch CPUStateVAX.prototype.boot or an
     * already-built block, both of which take effect at the later CALL), they must be applied
     * BEFORE makeMachine() builds the bus, or the mutation silently never engages and "SURVIVED"
     * would be a false negative rather than a real coverage hole.
     */
    let PRE_MACHINE = new Set(["mirror-not-aliased", "mirror-stale-copy"]);
    let survived = [];
    for (let name of Object.keys(MUTATIONS)) {
        let caught = false, how = "";
        try {
            let bus, cpu, undo;
            if (PRE_MACHINE.has(name)) {
                undo = MUTATIONS[name](null, null);
                ({bus, cpu} = makeMachine(romBytes));
            } else {
                ({bus, cpu} = makeMachine(romBytes));
                undo = MUTATIONS[name](cpu, bus);
            }
            try {
                if (name === "mirror-not-aliased" || name === "mirror-stale-copy") {
                    /*
                     * A DISTINCT SENTINEL, never the boot-time magic byte: ka655x.bin already
                     * ships 0x02 at offset 4 and the measured magic byte is also 2, so a check
                     * keyed there cannot tell a live alias from a snapshot that merely copied the
                     * right value once (this is exactly the vacuity a veracity review found in an
                     * earlier version of this file).
                     */
                    let base = VAX.PHYSMEM.ROM_BASE >>> 0, size = VAX.PHYSMEM.ROM_SIZE;
                    let addr = (base + 0x40) >>> 0;
                    let before = bus.getByte(addr);
                    let sentinel = (before === 0x37) ? 0x5A : 0x37;
                    bus.setByteDirect(addr, sentinel);
                    let m = bus.getByte((base + size + 0x40) >>> 0);
                    if (m !== sentinel) {
                        caught = true;
                        how = `mirror +0x40 = 0x${hex(m, 2)} after a direct sentinel write of 0x${hex(sentinel, 2)} to the primary`;
                    }
                    bus.setByteDirect(addr, before);
                } else if (name === "magic-byte-not-written" || name === "wrong-boot-psl") {
                    cpu.reset();
                    cpu.boot(0x37);
                    let base = VAX.PHYSMEM.ROM_BASE >>> 0;
                    let byte4 = bus.getByte((base + 4) >>> 0);
                    if (byte4 !== 0x37) { caught = true; how = `ROMBASE+4 = 0x${hex(byte4, 2)}, expected 0x37`; }
                    let wantPsl = (1 << 26 | 0x1F << 16) >>> 0;
                    if (cpu.psl >>> 0 !== wantPsl) { caught = true; how += (how ? "; " : "") + `PSL = ${hex(cpu.psl)}, expected ${hex(wantPsl)}`; }
                } else if (name === "rom-writable") {
                    cpu.reset();
                    cpu.boot(magicByte);
                    let addr = (VAX.PHYSMEM.ROM_BASE + 0x100) >>> 0;
                    let before = bus.getByte(addr);
                    bus.setByte(addr, (before ^ 0xFF) & 0xFF);
                    let after = bus.getByte(addr);
                    if (after !== before) { caught = true; how = `ROMBASE+0x100 changed from 0x${hex(before, 2)} to 0x${hex(after, 2)} via a NORMAL write`; }
                } else if (name === "ssc-base-not-decoded") {
                    let addr = VAX.PHYSMEM.SSC_BASE >>> 0;
                    bus.setLong(addr, 0x12345678 | 0);
                    if (bus.checkFault()) {
                        caught = true;
                        how = `a store to SSC_BASE (0x${hex(addr)}) faulted -- the base register is no longer decoded`;
                    }
                } else if (name === "ssc-base-wrong-mask") {
                    let addr = VAX.PHYSMEM.SSC_BASE >>> 0;
                    bus.setLong(addr, 0);
                    let v = bus.getLong(addr) >>> 0;
                    let want = 0x20000000;                  // SSCBASE_MBO -- must-be-one bits
                    if (v !== want) {
                        caught = true;
                        how = `SSC_BASE readback after storing 0 is 0x${hex(v)}, expected 0x${hex(want)} (SSCBASE_MBO preserved)`;
                    }
                } else if (name === "boundary-off-by-one") {
                    /*
                     * A pure-function check, deliberately not touching bus/cpu at all: the exact
                     * (records=2, finalizedPartial=false) input this item's own live romdiff run
                     * produced (see runRomJS()'s doc comment) -- the correct answer is
                     * {comparableRecords: 2, instrNum: 3} (both records genuinely completed, the
                     * boundary is the THIRD instruction); the mutation above always subtracts one.
                     */
                    let got = BoundaryAccounting.compute(2, false);
                    if (got.comparableRecords !== 2 || got.instrNum !== 3) {
                        caught = true;
                        how = `BoundaryAccounting.compute(2, false) = ${JSON.stringify(got)}, expected ` +
                            `{comparableRecords: 2, instrNum: 3}`;
                    }
                } else if (name === "stop-reported-not-enforced") {
                    /*
                     * Pure-function check with synthetic `js` objects, one per shape the invariant
                     * has to cover.  r1: the walk named NOTHING at all (the instrumentation never
                     * armed) -- the original pcjsvax-320 regression.  r2: it named a kind report()
                     * has no label for, so the run would print no honest stopping point.  r3: it
                     * named a kind but attached no evidence -- naming the KIND without naming the
                     * WHY is not a named stop.  r4: a legitimate budget stop, which must pass.  The
                     * unmutated function must flag r1/r2/r3 and pass r4; the mutation returns null
                     * for everything, which only r1/r2/r3 can detect.
                     */
                    let r1 = StopReported.check({stopKind: null, stopWhy: null});
                    let r2 = StopReported.check({stopKind: "something-new", stopWhy: "x"});
                    let r3 = StopReported.check({stopKind: "budget", stopWhy: null});
                    let r4 = StopReported.check({stopKind: "budget", stopWhy: "walked the full budget"});
                    if (r1 === null || r2 === null || r3 === null) {
                        caught = true;
                        how = `StopReported.check() returned null (no problem) for a run that named no ` +
                            `honest stopping point -- r1(no stop kind)=${JSON.stringify(r1)}, ` +
                            `r2(unknown kind)=${JSON.stringify(r2)}, r3(kind but no evidence)=${JSON.stringify(r3)}`;
                    } else if (r4 !== null) {
                        /* not the mutation this branch grades, but a genuine regression in the same
                           function is still worth failing loudly on rather than silently passing */
                        caught = true;
                        how = `StopReported.check() flagged a problem for a legitimate, fully named budget ` +
                            `stop -- r4=${JSON.stringify(r4)}, expected null`;
                    }
                } else if (name === "graded-equality-weakened-to-both-faulted") {
                    /*
                     * Pure-function, deliberately: the shipped gradeEqual() is fed TWO probe results
                     * that BOTH faulted but disagree on everything the rule actually grades.  The
                     * unmutated function must reject each; the mutation accepts all of them because
                     * "both faulted" is all it looks at.  A live walk cannot make this check, because
                     * on the CURRENT tree the ROM's only fault IS graded equal -- there is nothing
                     * for a run-based assertion to observe.
                     */
                    let base = {faulted: true, vector: 0x04, pc: FAULT_PROBE.HANDLER + 4 * FAULT_PROBE.SLOT, psl: 0x041F0000, threw: null};
                    let diffVector = Object.assign({}, base, {vector: 0x0C, pc: FAULT_PROBE.HANDLER + 12 * FAULT_PROBE.SLOT});
                    let diffPC = Object.assign({}, base, {pc: (base.pc + 4) >>> 0});
                    let diffPSL = Object.assign({}, base, {psl: 0x001F0000});
                    let bad = [["SCB vector", diffVector], ["resulting PC", diffPC], ["resulting PSL", diffPSL]]
                        .filter(([, p]) => FaultGrader.gradeEqual(base, p).equal);
                    if (bad.length) {
                        caught = true;
                        how = `gradeEqual() called two faults EQUAL that differ in ` +
                            `${bad.map(([w]) => w).join(", ")} -- "both faulted" is not equality`;
                    } else if (!FaultGrader.gradeEqual(base, Object.assign({}, base)).equal) {
                        caught = true;
                        how = `gradeEqual() rejected two IDENTICAL fault results -- the rule has become ` +
                            `unsatisfiable, which stops the walk everywhere rather than only at real gaps`;
                    }
                } else if (name === "probe-cache-stale-address") {
                    /*
                     * The cache is exercised through the SHIPPED FaultGrader.verdict() against the
                     * LIVE oracle, with the probeJS/probeSimh calls counted -- not re-implemented --
                     * so what is graded is the real cached path.  Two addresses with genuinely
                     * DIFFERENT verdicts on this tree: SSC_BASE (decoded on both engines, no fault)
                     * and IOPAGE+0x1F00 (an unpopulated Qbus address both engines machine-check on --
                     * pcjsvax-93e's measured example).  Unmutated: two distinct keys, two probes, two
                     * different verdicts, and a repeat of the first is served from cache.  Mutated:
                     * one key, so the SECOND address inherits the FIRST address's verdict.
                     */
                    let ctx = makeWalkCtx(simh, opts, romBytes, false);
                    let a = FaultGrader.verdict(ctx, VAX.PHYSMEM.SSC_BASE >>> 0, false);
                    let b = FaultGrader.verdict(ctx, 0x20001F00, false);
                    let aAgain = FaultGrader.verdict(ctx, VAX.PHYSMEM.SSC_BASE >>> 0, false);
                    if (b.addr !== 0x20001F00) {
                        caught = true;
                        how = `verdict() for 0x20001F00 answered about ${nameAddress(b.addr)} instead -- ` +
                            `the cache served a verdict measured for a DIFFERENT address`;
                    } else if (a.jsP.faulted || !b.jsP.faulted || !b.simhP.faulted) {
                        caught = true;
                        how = `the two probe addresses no longer have distinguishable verdicts on this ` +
                            `tree (SSC_BASE faulted=${a.jsP.faulted}, IOPAGE+0x1F00 js=${b.jsP.faulted} ` +
                            `simh=${b.simhP.faulted}) -- this check cannot detect a stale cache without ` +
                            `them, so it fails loudly rather than certifying coverage it does not have`;
                    } else if (ctx.faultCacheHits !== 1 || ctx.faultCache.size !== 2) {
                        caught = true;
                        how = `after probing 2 distinct addresses and repeating one, the cache holds ` +
                            `${ctx.faultCache.size} entr(ies) with ${ctx.faultCacheHits} hit(s), expected 2 and 1`;
                    } else if (aAgain !== a) {
                        caught = true;
                        how = `the repeat probe of ${nameAddress(a.addr)} did not return the cached verdict object`;
                    }
                } else if (name === "walk-continues-past-serviced-fault") {
                    /*
                     * Pure-function on the SHIPPED decision.  The four verdict kinds FaultGrader
                     * produces, each asserted against what the rule must do with it.  "boundary" is
                     * the one this mutation is named for -- a fault the oracle SERVICES and this bus
                     * does not, i.e. a real hardware gap -- and continuing past it is precisely how a
                     * progress meter goes blind.  PHASE F proves the same thing end to end on a live
                     * walk; this proves the walk is asking WalkDecision at all.
                     */
                    let mustStop = ["boundary", "js-absorbs", "graded-unequal"]
                        .filter((k) => WalkDecision.shouldContinue({kind: k}));
                    if (mustStop.length) {
                        caught = true;
                        how = `the walk would CONTINUE past a fault graded ${mustStop.join(", ")} -- ` +
                            `"boundary" means the oracle services an address this bus does not decode, ` +
                            `and walking past it is a blind meter (HANDOFF.md standing rule 13)`;
                    } else if (!WalkDecision.shouldContinue({kind: "equal"})) {
                        caught = true;
                        how = `the walk would STOP on a fault graded EQUAL -- the rule has become ` +
                            `unsatisfiable, which is this change reverted rather than weakened`;
                    }
                } else if (name === "fallthrough-reverts-to-blanket-fault") {
                    /*
                     * SSC+0x50 (rg=0x14) is a GENUINE hardware gap -- verifyFallthroughSemantics()'s
                     * live "ssc-uncased-write-then-read" case already proved the real oracle accepts
                     * a reference there with no fault.  Through the REAL bus path (matching ssc-base-
                     * not-decoded's own established pattern), the unmutated code must not fault; the
                     * mutation reverts to the pre-fix blanket behavior, which does.
                     */
                    let addr = (VAX.PHYSMEM.SSC_BASE + 0x50) >>> 0;
                    bus.setLong(addr, 0xFFFFFFFF | 0);
                    let faultedOnWrite = bus.checkFault();
                    bus.getLong(addr);
                    let faultedOnRead = bus.checkFault();
                    if (faultedOnWrite || faultedOnRead) {
                        caught = true;
                        how = `SSC+0x50 (a genuine hardware gap, per verifyFallthroughSemantics()'s own live ` +
                            `oracle grading) faulted -- write=${faultedOnWrite} read=${faultedOnRead}`;
                    }
                }
            } finally {
                undo();
            }
        } catch (e) {
            caught = true;
            how = `threw ${e.name}: ${e.message}`;
        }
        console.log(`  ${caught ? "CAUGHT " : "SURVIVED"} ${name.padEnd(28)} ${how}`);
        if (!caught) survived.push(name);
    }
    if (survived.length) {
        console.error("\nFAILURES:");
        for (let n of survived) console.error(`  SELFCHECK: mutation "${n}" was NOT caught -- that is a coverage hole, not a pass`);
        console.error(`  (scratch KEPT for inspection: ${opts.scratch})`);
        process.exit(1);
    }
    cleanupScratch(opts);
    console.log("\nOK");
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                           *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, dflt)
{
    let i = process.argv.indexOf(name);
    return (i >= 0 && i + 1 < process.argv.length) ? process.argv[i + 1] : dflt;
}

function main()
{
    let fNoCdg = process.argv.indexOf("--no-cdg") >= 0;
    let opts = {
        maxSteps: +getArg("--max-steps", WALK_BUDGET_DEFAULT),
        scratch: fs.mkdtempSync(path.join(os.tmpdir(), "romdiff-"))
    };
    opts.rom = findRom(getArg("--rom", null));
    let simh = findSimh(getArg("--simh", null));
    let fSelfcheck = process.argv.indexOf("--selfcheck") >= 0;
    let problems = [];

    console.log(`SIMH: ${simh}`);
    console.log(`ROM:  ${opts.rom}`);
    console.log(`scratch: ${opts.scratch}`);

    let romBytes = new Uint8Array(fs.readFileSync(opts.rom));
    if (romBytes.length !== VAX.PHYSMEM.ROM_SIZE) {
        console.error(`FAILURES:\n  ROM: ${opts.rom} is ${romBytes.length} bytes, expected exactly ` +
            `${VAX.PHYSMEM.ROM_SIZE} (VAX.PHYSMEM.ROM_SIZE)`);
        process.exit(1);
    }

    let {sysModel, magicByte} = querySysModel(simh, opts);
    console.log(`\nsys_model (measured, examine MODEL on this exact oracle): ${sysModel} -> magic byte 0x${hex(magicByte, 2)}`);
    if (magicByte !== ROM_MAGIC_BYTE) {
        problems.push(`ROM_MAGIC_BYTE in cpustate.js is 0x${hex(ROM_MAGIC_BYTE, 2)}, but this oracle's ` +
            `sys_model=${sysModel} calls for 0x${hex(magicByte, 2)} -- the shipped default has drifted from the oracle it was measured against`);
    }

    if (fSelfcheck) {
        selfcheck(romBytes, opts, magicByte, simh);
        return;
    }

    /* The walk budget is bounded by SIMH's own history ring, not by taste -- see WALK_BUDGET_MAX. */
    if (!(opts.maxSteps > 0) || opts.maxSteps > WALK_BUDGET_MAX) {
        console.error(`FAILURES:\n  --max-steps ${opts.maxSteps} is outside [1, ${WALK_BUDGET_MAX}]; the ` +
            `ceiling is SIMH's HIST_MAX (${SIMH_HIST_LNT}) less the ${SIMH_TRACE_MARGIN}-record margin, ` +
            `because a longer capture WRAPS the history ring and a wrapped ring manufactures phantom ` +
            `divergences (vax_cpu.c:1676; see the file header).`);
        process.exit(1);
    }

    /* ---- PHASE 1: the walk, and the trace comparison that extends with it ---- */
    let ctx = makeWalkCtx(simh, opts, romBytes, fNoCdg);
    let js = runRomJS(romBytes, opts, magicByte, ctx);
    sampleHeap();
    console.log(`\nROM boot-entry walk${fNoCdg ? "  [--no-cdg: cache-diagnostic decode OMITTED]" : ""}`);
    console.log(`  instructions executed : ${js.records} comparable records in ${js.steps} steps (budget ${opts.maxSteps})`);
    console.log(`  faults walked past    : ${js.absorbed} graded EQUAL, of ${js.faultEvents} fault event(s) ` +
        `at ${js.distinctFaultAddrs} distinct address(es); ${js.probeCacheHits} oracle probe(s) served from cache` +
        (js.faultEvents > js.faults.length
            ? ` -- detail below shows the first ${js.faults.length} of ${js.faultEvents} events`
            : ""));
    for (let f of js.faults) {
        console.log(`     ${f.verdict.kind === "equal" ? "walked past" : "STOPPED AT  "} ` +
            `${(f.verdict.fWrite ? "write" : "read ")} ${nameAddress(f.addr)} from PC ${hex(f.pc)} ` +
            `after ${f.recordsBefore} records -- ${f.verdict.why}`);
    }
    console.log(`  stopped               : ${STOP_KINDS[js.stopKind] ? STOP_KINDS[js.stopKind].label : "(UNNAMED)"} ` +
        `at PC=${hex(js.pc)} PSL=${hex(js.psl)}`);
    console.log(`                          ${js.stopWhy || "(no reason recorded)"}`);

    /* HANDOFF.md standing rule 13: a run that stops without NAMING where and why is not a pass,
       whatever its exit code would otherwise have been. */
    let stopProblem = StopReported.check(js);
    if (stopProblem) problems.push(stopProblem);
    if (js.stopKind && STOP_KINDS[js.stopKind] && STOP_KINDS[js.stopKind].fatal) {
        problems.push(`${STOP_KINDS[js.stopKind].label}: ${js.stopWhy}`);
    }

    /* A boundary is the one stop kind that also names a DEVICE for the next item to build. */
    let boundary = (js.stopKind === "boundary")
        ? {addr: js.stopFault.addr, fWrite: js.stopFault.verdict.fWrite, instrPC: js.stopFault.pc,
           verdict: js.stopFault.verdict}
        : null;

    try {
        /*
         * MEASURED (veracity re-dispatch, pcjsvax-446): the stopping instruction's OWN record in
         * js.tracePath is a FORCE-FINALIZED partial one WHEN its fault happened inside the
         * instruction's BODY (runRomJS()'s unconditional `hst.finish(cpu)` call finalizes whatever
         * hst.record() already started, even though the body itself aborted) -- SIMH's capture keeps
         * going and does execute it, but the JS record is not a real, comparable instruction, so
         * that case compares `js.records - 1` records.
         *
         * MEASURED (pcjsvax-320): that is NOT the only shape a stop can take.  A fault reached
         * during SPECIFIER RESOLUTION happens BEFORE hst.record() ever runs for that instruction
         * (cpustate.js's fnExecute ordering), so hst.finish() is a genuine no-op and js.records is
         * ALREADY exact.  MEASURED (pcjsvax-fe7): and a BUDGET stop leaves a record pending for an
         * instruction that COMPLETED, which must not be trimmed at all.  runRomJS() distinguishes
         * all three by reading simhtrace.js's own `pending.completed` rather than inferring it --
         * see its comment at `pendingIncomplete`.
         */
        let {comparableRecords, instrNum: stopInstrNum} = BoundaryAccounting.compute(js.records, js.finalizedPartial);
        /*
         * THE COMPARISON EXTENDS WITH THE WALK (pcjsvax-fe7 done condition 1).  Every record the
         * walk produced is compared -- including the ROM's own machine-check handler, executed after
         * each absorbed fault.  This is the grader that catches a JS fault at a WRONG effective
         * address which the per-address probe would call equal: SIMH's real execution never faulted
         * there, so its trace diverges.
         */
        let capture = captureSimhTrace(simh, opts, comparableRecords + SIMH_TRACE_MARGIN);
        sampleHeap();
        let cmp = compareTraces(js.tracePath, capture.tracePath, js.unavailable, opts, comparableRecords);
        sampleHeap();
        console.log(`  SIMH capture           : ${capture.records} records in ${capture.steps} steps ` +
            `(needed ${comparableRecords} + ${SIMH_TRACE_MARGIN} margin; ring ${SIMH_HIST_LNT}, never wrapped)`);
        console.log(`  normalized records     : CMPD-tail=${cmp.normalized_cmpd} zerospec=${cmp.normalized_zerospec} store-faulted=${cmp.normalized_unavailable}`);
        console.log(`  trace comparison       : ${cmp.match ? "MATCH over all " + cmp.simh_records + " records" : "DIVERGE"}`);
        if (!cmp.match) {
            let d = cmp.divergence;
            problems.push(`TRACE: diverged at instruction ${d.index} (field=${d.field}, PC=${d.pc_a}): ${d.detail}`);
        }
        if (cmp.simh_records !== cmp.js_records) {
            problems.push(`TRACE: JS produced ${cmp.js_records} comparable records but only ${cmp.simh_records} ` +
                `were compared -- SIMH's capture was shorter than the walk, so some JS instructions never ` +
                `reached comparison`);
        }
        if (boundary) {
            boundary.instrNum = stopInstrNum;
            console.log(`\n  UNDECODED-HARDWARE BOUNDARY: instruction #${stopInstrNum} ` +
                `(${boundary.fWrite ? "write" : "read"}) touched ${nameAddress(boundary.addr)}, which SIMH ` +
                `SERVICES and this bus does not decode -- CONFIRMED on the live oracle by ` +
                `FaultGrader.verdict() (${boundary.verdict.why}), not assumed.  See the next device ` +
                `item, which will move this boundary forward.`);
        }
    } catch (e) {
        /* pcjsvax-23c FIX: this is the whole reason this block is wrapped -- a failure HERE used to
           kill main() with an uncaught exception before a single line of PHASE 1b/1c/2/F output, or
           even the already-pushed stop problem, was ever printed.  Record it and keep going. */
        problems.push(`TRACE comparison aborted: ${e.message || e}`);
    }

    /* ---- PHASE 1b: the SSC base register mask, RANDOMIZED against the real oracle ----
       See verifySscBaseRandom()'s doc comment for why the trace above cannot exercise this on its
       own: the ROM stores exactly ONE value (0x20140000), which already satisfies the mask, so a
       transcription bug in SSCBASE_RW/SSCBASE_MBO would pass the real-workload phase silently. */
    let sscCases = +getArg("--ssc-base-cases", 40);
    if (sscCases < SSC_BASE_CASES_FLOOR) {
        problems.push(`--ssc-base-cases ${sscCases} is below the enforced floor (${SSC_BASE_CASES_FLOOR}); ` +
            `this run would under-cover the SSC base register's mask formula and must not be allowed to pass.`);
    } else {
        let sscProblems = verifySscBaseRandom(simh, opts, +getArg("--seed", 0xB16B00B5), sscCases);
        console.log(`\nSSC BASE REGISTER MASK (randomized, ${sscCases} cases, real oracle): ` +
            `${sscProblems.length ? "DIVERGED" : "MATCH"}`);
        for (let p of sscProblems) problems.push(p);
    }

    /* ---- PHASE 1c: fall-through semantics (uncased-register / read-only-write / misaligned-write
       silent-no-op, NOT a fault), against the real oracle -- veracity finding, post-merge
       re-dispatch.  A FIXED, exhaustively enumerated case list (FALLTHROUGH_CASES) -- nothing here
       scales with a --cases flag, so there is no floor to enforce; standing rule 4 is satisfied by
       construction (see this file's own coverage-floor convention elsewhere for the flag-driven
       shape this is deliberately NOT using, matching hwintdiff.js's own fixed-matrix precedent). */
    {
        let fallthroughProblems = verifyFallthroughSemantics(romBytes, simh, opts, fNoCdg);
        console.log(`\nFALL-THROUGH SEMANTICS (${FALLTHROUGH_CASES.length} fixed cases, real oracle): ` +
            `${fallthroughProblems.length ? "DIVERGED" : "MATCH"}`);
        for (let p of fallthroughProblems) problems.push(p);
    }

    /* ---- PHASE 2: the mirror ---- */
    let mirrorProblems = verifyMirrorJS(romBytes, opts, magicByte, fNoCdg);
    for (let p of mirrorProblems) problems.push("MIRROR (JS): " + p);
    try {
        let simhMirror = verifyMirrorSimh(simh, opts);
        console.log("\nMirror (upper half == lower half), before and after boot():");
        for (let r of simhMirror.beforeBoot) {
            let match = r.primary === r.mirror;
            console.log(`  +0x${r.off.toString(16).padStart(5, "0")}: SIMH primary=0x${hex(r.primary, 2)} mirror=0x${hex(r.mirror, 2)} ${match ? "" : "<- SIMH ITSELF DISAGREES"}`);
            if (!match) problems.push(`MIRROR (SIMH): SIMH's own primary/mirror disagree at +0x${r.off.toString(16)}: 0x${hex(r.primary, 2)} vs 0x${hex(r.mirror, 2)}`);
        }
        console.log(`  +4 after boot(): SIMH primary=0x${hex(simhMirror.afterBoot.primary4, 2)} mirror=0x${hex(simhMirror.afterBoot.mirror4, 2)}`);
        if (simhMirror.afterBoot.primary4 !== magicByte) {
            problems.push(`MIRROR (SIMH): SIMH's own primary +4 after boot is 0x${hex(simhMirror.afterBoot.primary4, 2)}, expected the magic byte 0x${hex(magicByte, 2)}`);
        }
        if (simhMirror.afterBoot.mirror4 !== magicByte) {
            problems.push(`MIRROR (SIMH): SIMH's own mirror +4 after boot is 0x${hex(simhMirror.afterBoot.mirror4, 2)}, expected it to track the magic byte too`);
        }
    } catch (e) {
        problems.push(`MIRROR (SIMH) comparison aborted: ${e.message || e}`);
    }

    /* ---- PHASE F: THE REGRESSION FLOOR ---- */
    if (!fNoCdg) {
        for (let p of verifyRegressionFloor(romBytes, simh, opts, magicByte)) problems.push(p);
    } else {
        console.log(`\nREGRESSION FLOOR: skipped -- this run IS the floor configuration (--no-cdg)`);
    }

    /* ---- HANDOFF.md standing rule 14: an ABSOLUTE peak-heap bound that FAILS the run ---- */
    sampleHeap();
    console.log(`\npeak JS heap: ${g_peakHeapMB.toFixed(1)} MB (absolute ceiling ${PEAK_HEAP_MB_MAX} MB)`);
    if (g_peakHeapMB > PEAK_HEAP_MB_MAX) {
        problems.push(`MEMORY: peak JS heap reached ${g_peakHeapMB.toFixed(1)} MB, over the absolute ` +
            `${PEAK_HEAP_MB_MAX} MB ceiling.  This bound does NOT scale with --max-steps by design ` +
            `(HANDOFF.md standing rule 4/14): a differential that grows its own memory with its own ` +
            `workload is how cdgdiff.js reached 8.6 GB and OOM-killed a whole agent fleet.`);
    }

    report(problems, js, boundary, opts);
}

/**
 * cleanupScratch(opts)
 *
 * HANDOFF.md §4's disk warning is not hypothetical: a broken romdiff run once wrote 9.5 GB and
 * filled the root filesystem, and every invocation before pcjsvax-fe7 left its whole scratch tree
 * behind forever (20 accumulated trees were on this machine when this item started).  The traces
 * scale with --max-steps -- ~52 MB at the default budget -- so a gate that runs on every change
 * cannot keep them.  A FAILING run keeps its scratch, because that is exactly when someone needs the
 * two traces to look at; the path was printed at the top either way.
 *
 * @param {Object} opts
 */
function cleanupScratch(opts)
{
    try { fs.rmSync(opts.scratch, {recursive: true, force: true}); }
    catch (e) { console.log(`  (could not remove scratch ${opts.scratch}: ${e.message})`); }
}

/**
 * verifyRegressionFloor(romBytes, simh, opts, magicByte)
 *
 * THE FLOOR (pcjsvax-fe7 done condition 4).  Re-walks the ROM with ONE currently-decoded device
 * removed -- the cache-diagnostic space, via makeMachine()'s fOmitCdg -- and requires the walk rule
 * to STOP at the resulting real hardware gap and NAME it.
 *
 * WHY THIS RUNS ON EVERY ORDINARY INVOCATION rather than living in --selfcheck or in a separate
 * script: the walk rule this item introduces is the one change in this project that can make the
 * progress meter BLIND (HANDOFF.md standing rule 13, which exists because romdiff has twice stopped
 * measuring).  A relaxation of the stopping rule -- "keep going until something looks wrong" -- turns
 * romdiff green and blind in one move, and every other phase in this file would still pass.  This is
 * the phase that cannot.
 *
 * WHY IT ASSERTS A FIXED NUMBER: FLOOR_INSTR_NUM/FLOOR_ADDR are a MEASURED property of this ROM
 * against this tree with CDG removed, and HANDOFF.md §3 records the same 393/CDG+0x0 pair as the
 * project's progress meter before pcjsvax-0b7 decoded it.  A floor whose expectation moved with the
 * code would not be a floor.  If a later item legitimately changes it, changing it here is a
 * deliberate act, which is the point.
 *
 * WHAT IT DELIBERATELY DOES NOT DO: no trace comparison.  The claim under test is "the walk still
 * stops at, and names, a real gap", not "the CDG-less machine matches the oracle" -- that machine's
 * fidelity was graded by this same file before pcjsvax-0b7 and by tests/cdgdiff.js after it.  Adding
 * a second full oracle capture here would cost the gate real time for a claim already owned
 * elsewhere.
 *
 * @param {Uint8Array} romBytes
 * @param {string} simh
 * @param {Object} opts
 * @param {number} magicByte
 * @returns {Array.<string>} problems (empty if none)
 */
const FLOOR_INSTR_NUM = 393;
const FLOOR_ADDR = VAX.PHYSMEM.CDG_BASE >>> 0;

function verifyRegressionFloor(romBytes, simh, opts, magicByte)
{
    let problems = [];
    /* Its OWN budget: the floor's gap is at instruction 393 and a walk that has not stopped by an
       order of magnitude past it has already failed the claim.  Bounded independently of
       --max-steps so raising the walk budget can never make the floor slower or weaker. */
    let floorOpts = {maxSteps: 5000, scratch: opts.scratch, rom: opts.rom};
    let ctx = makeWalkCtx(simh, floorOpts, romBytes, true);
    let js = runRomJS(romBytes, floorOpts, magicByte, ctx);
    sampleHeap();
    let {instrNum} = BoundaryAccounting.compute(js.records, js.finalizedPartial);

    console.log(`\nREGRESSION FLOOR (the same walk rule, with the CDG decode removed):`);
    console.log(`  stopped               : ${STOP_KINDS[js.stopKind] ? STOP_KINDS[js.stopKind].label : "(UNNAMED)"} ` +
        `at instruction #${instrNum}, PC=${hex(js.pc)}`);
    console.log(`                          ${js.stopWhy || "(no reason recorded)"}`);

    let stopProblem = StopReported.check(js);
    if (stopProblem) problems.push("FLOOR: " + stopProblem);

    if (js.stopKind !== "boundary") {
        problems.push(`FLOOR: with the CDG decode REMOVED the walk must stop at a real undecoded-hardware ` +
            `boundary, but it stopped with "${js.stopKind}" instead (${js.stopWhy}).  A walk rule that can ` +
            `run past a gap the oracle SERVICES has made the progress meter blind, which HANDOFF.md ` +
            `standing rule 13 says is worse than a gate that fails.`);
        return problems;
    }
    let addr = js.stopFault.addr >>> 0;
    if (addr !== FLOOR_ADDR) {
        problems.push(`FLOOR: the boundary was named at ${nameAddress(addr)}, expected ${nameAddress(FLOOR_ADDR)} ` +
            `-- the floor's whole claim is that the meter still finds THIS gap.`);
    }
    if (instrNum !== FLOOR_INSTR_NUM) {
        problems.push(`FLOOR: the boundary was named at instruction #${instrNum}, expected #${FLOOR_INSTR_NUM} ` +
            `(HANDOFF.md §3's recorded progress-meter value for this ROM with the CDG decode absent).  ` +
            `A boundary that MOVED is either real progress that must be recorded here deliberately, or a ` +
            `walk that is now counting differently -- it is never something to accept silently.`);
    }
    if (!problems.length) {
        console.log(`  FLOOR HELD            : instruction #${instrNum} / ${nameAddress(addr)} still named`);
    }
    return problems;
}

function report(problems, js, boundary, opts)
{
    if (problems.length) {
        console.error("\nFAILURES:");
        for (let p of problems) console.error("  " + p);
        console.error(`  (scratch KEPT for inspection: ${opts.scratch})`);
        process.exit(1);
    }
    cleanupScratch(opts);
    console.log("\nOK");
    /*
     * HANDOFF.md standing rule 13.  EVERY outcome names a stopping point -- boundary, divergence, or
     * budget -- and StopReported.check() above has already failed the run if this could not.  There
     * is deliberately no `else` that prints nothing.
     */
    if (boundary) {
        /* boundary.instrNum is NOT always js.records: it is js.records only when the boundary
           instruction's own fault happened inside its BODY (a force-finalized partial record IS the
           last of js.records).  See main()'s comment at comparableRecords/stopInstrNum. */
        console.log(`\nSTOPPING POINT: instruction #${boundary.instrNum} (0-based index ${boundary.instrNum - 1}), ` +
            `PC=${hex(boundary.instrPC)}, attempted a ${boundary.fWrite ? "write to" : "read of"} ` +
            `${nameAddress(boundary.addr)} -- SIMH services this address; this bus does not decode it ` +
            `yet.`);
    } else {
        let k = STOP_KINDS[js.stopKind];
        console.log(`\nSTOPPING POINT: ${k ? k.label : "(UNNAMED)"} after ${js.records} comparable ` +
            `instructions, PC=${hex(js.pc)} PSL=${hex(js.psl)} -- ${js.stopWhy}`);
        console.log(`NO UNDECODED-HARDWARE BOUNDARY was reached: the ROM's ${js.faultEvents} bus fault(s) ` +
            `at ${js.distinctFaultAddrs} distinct address(es) were ALL graded equal against the live oracle, ` +
            `so there is no missing device for the next item to build from this run.  Raise --max-steps ` +
            `(ceiling ${WALK_BUDGET_MAX}) to walk further.`);
    }
}

main();

export { querySysModel, runRomJS, captureSimhTrace, compareTraces, verifyMirrorJS, nameAddress,
         FaultGrader, WalkDecision, StopReported, makeWalkCtx };
