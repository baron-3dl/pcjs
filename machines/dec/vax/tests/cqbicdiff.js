/**
 * @fileoverview Differential test: the ROM's Qbus-adapter probe -- every CQBIC local-register and
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this
 * work.
 *
 *               doorbell access the KA655 firmware makes, in order, vs. a real Open SIMH
 *               microvax3900; plus the read-only MEAR/SEAR rejection the ROM never exercises
 *
 * ============================================================================
 * WHAT THIS IS
 * ============================================================================
 * pcjsvax-69a.  The item's outcome is "the ROM's Qbus adapter probe succeeds", and by the time the
 * item was worked it already did -- the ROM runs its countdown unbroken and reaches `>>>`.  So the
 * work is not the fix; the work is that NOTHING GRADED IT.  tests/conoutdiff.js grades a console
 * BYTE STREAM and can say the self-tests passed while saying nothing about whether any register
 * answered correctly; tests/romdiff.js stops ~2.1M instructions before the probe's second half;
 * tests/dbldiff.js grades the doorbell as a register and never as something the ROM drives;
 * tests/cqmerrdiff.js and tests/cqmmapdiff.js grade the Qbus MAP, which is a different mechanism.
 * That is the exact shape of hole pcjsvax-aa5 found next door and HANDOFF.md standing rule 13 is
 * about: a symptom being absent is not the same as the behaviour being graded.
 *
 * PHASE C is therefore a LITERAL, ORDERED comparison of the ROM's own register traffic against
 * patch 0006's `SET SYSD DEBUG=DEVTRACE` records from a live oracle booting the same ROM -- every
 * CQBIC access, every doorbell access, in sequence, with the address, the access width, the value
 * and the PC.  It is the item's done-condition 2 word for word.
 *
 * ============================================================================
 * WHAT THE ROM ACTUALLY DOES -- MEASURED 2026-07-29, NOT ASSUMED
 * ============================================================================
 * The probe is 77 accesses long and ends at the doorbell WRITE (`QIOW 20001F40`, PC 2004D7AC).
 * In order:
 *
 *      #0..#4    SCR: read POK (0x00008000), write it back, read the POK BYTE at CQBICBASE+1,
 *                then set BHL -- the register reads 0x0000C000 from there on.
 *      #5..#10   DSER: read 0x80, write 0x80 to clear it; twice more from two different PCs.
 *      #11..#14  SCR longword, DSER, the SCR POK byte again, then the Qbus I/O-page doorbell READ.
 *      #15..#74  MBR: a 29-bit ADDRESS-LINE walk -- write 0x00008000, read it back, write its
 *                complement 0x1FFF0000, read it back, then 0x00010000/0x1FFE8000 ... up to
 *                0x10000000/0x0FFF8000, 30 writes and 28 read-backs, ending with MBR := 0.
 *      #75..#75  MEAR reads 0x0000000F, SEAR reads 0, DSER reads 0, the SCR POK byte again.
 *      #76       the doorbell WRITE.
 *
 * ALL FIVE CQBIC registers and BOTH doorbell directions are inside that window, which is why the
 * coverage floor in PHASE F can be a floor on the REQUIREMENT rather than on this machine's
 * contents (standing rule 4).
 *
 * WHAT THE ROM DOES *NOT* DO, stated because the floors below depend on it:
 *   - It NEVER writes MEAR or SEAR.  The read-only rejection path -- the one branch in the whole
 *     device that genuinely faults on write -- is UNREACHABLE from the firmware, so PHASE X
 *     constructs it, and PHASE X is the only coverage it will ever have.
 *   - It NEVER touches the CQIPC LOCAL register at CQIPCBASE (REGBASE+0x1F40).  It reaches the same
 *     register only through the Qbus I/O-page doorbell at 0x20001F40.  tests/dbldiff.js grades the
 *     local mount; this file records that the ROM does not use it, and the floor asks for the
 *     doorbell, not for both mounts.
 *   - It NEVER drives the CQBIC registers with a byte or word WRITE.  Only longword writes, plus
 *     byte READS of the SCR POK bit.  PHASE X covers the byte and word write lanes.
 *   - It NEVER exercises MBR's write mask.  Every one of the 30 values in the address-line walk is
 *     already inside CQMBR_MASK (0x1FFF8000), so `nval & CQMBR_MASK` is the identity for all of
 *     them.  MEASURED, and it cost this file a mutation: `mbr-mask-widened` was written for PHASE C
 *     and SURVIVED there, because removing a mask that never masks anything changes nothing --
 *     standing rule 11's idempotent mutation in its purest form.  It now lives in PHASE X, where
 *     a constructed `MOVL #FFFFFFFF` into MBR does exercise it.
 *
 * ============================================================================
 * THE DEFECT THIS FILE FOUND -- READ-ONLY IS NOT THE SAME AS UNDECODED
 * ============================================================================
 * vax_io.c:519-520's MEAR/SEAR case is `cq_merr (pa); MACH_CHECK (MCHK_WRITE);` -- the CQBIC
 * latches its OWN DSER<MNX>/MEAR and raises the check itself.  cqbic.js returned a plain `false`,
 * which is regblock.js's "I do not decode this", so cpustate.js's onBusFault() took
 * vax_sysdev.c's WriteReg() `default:` branch instead and set the SSC bus-timeout bits the C never
 * sets here.  MEASURED on `MOVL #FFFFFFFF,@#20080008`, one instruction:
 *
 *                  PC          BTO         DSER    MEAR
 *      oracle      00102000    00000000    80      0400
 *      before      00102000    C0000000    00      0000
 *
 * Three observable fields wrong, all in the same direction: the machine said "nothing answered"
 * where the real one says "the Qbus adapter refused the write".  BTO is readable by the firmware,
 * so this was not an internal detail.  Fixed in cqbic.js (writeReg returns regblock.js's REG_MCHK
 * after latching cq_merr); PHASE X grades all four fields, and the `readonly-write-bus-timeout`
 * mutation is the tripwire that fails if the fix is ever reverted.
 *
 * ============================================================================
 * WHAT THIS FILE DELIBERATELY DOES NOT GRADE
 * ============================================================================
 * The Qbus scatter/gather MAP.  pcjsvax-69a puts it out of scope and says to STOP and report if the
 * ROM demands more of it than a probe; it does, and pcjsvax-aa5 is that report.  MEASURED HERE, and
 * this is why the item's suggested "the map region answering as RAM" mutation is NOT in the suite
 * below: with the CQM window perturbed to answer as plain storage instead of translating, and
 * again with CQMAP/CQM left undecoded entirely (rommachine.js's `fOmitCqm`), THE 77-ACCESS WINDOW
 * ABOVE IS BYTE-IDENTICAL.  A mutation this instrument cannot see must not be carried by this
 * instrument (standing rule 11's spirit, and rule 16's: a mutation "caught" by the wrong check
 * certifies nothing).  That decode is graded by tests/conoutdiff.js's CqmDecodeFloor, which is
 * where pcjsvax-aa5 put it, and by tests/cqmmapdiff.js for the translation itself.
 *
 * ============================================================================
 * WHY TWO ORACLE CAPTURES, AND WHY THE WINDOW IS NOT AN LCP
 * ============================================================================
 * The obvious construction -- take the longest common prefix of two captures and grade that -- was
 * built first and REJECTED by measurement.  Piping SIMH's debug output through a filter appeared to
 * show the two captures diverging at access 77 about a third of the time; the divergence was
 * entirely an artifact of the OBSERVATION CHANNEL.  `SET DEBUG STDOUT` shares one stream with the
 * simulated console, whose bytes arrive without line terminators, so debug lines were being cut in
 * half and merged.  Written to a FILE instead, eight consecutive captures produced the identical
 * 77 accesses.  A derived window would have quietly shrunk to a handful of accesses on those runs
 * and still exited 0 -- standing rule 13's failure mode, arrived at by standing rule 16's route.
 * So the window is FIXED and anchored by CONTENT (the doorbell write), the floor on its length is
 * an absolute constant, and the two captures must AGREE over it or the run FAILS.  A disagreement
 * is reported, never absorbed.
 *
 * Usage:
 *      node cqbicdiff.js --simh PATH [--rom PATH] [--js-steps N] [--oracle-steps N] [--selfcheck]
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import { VAX } from "../modules/v2/defines.js";
import { OPCODES } from "../modules/v2/drom.js";
import { SCB } from "../modules/v2/exc.js";
import CQBICVAX, {
    CQBIC_BASE, CQBIC_SIZE, REG_SCR, REG_DSER, REG_MEAR, REG_SEAR, REG_MBR, CQSCR_POK
} from "../modules/v2/cqbic.js";
import CQIPCVAX, { CQIPC_BASE, CQIPC_SIZE, DBL_BASE, DBL_SIZE, DBLVAX } from "../modules/v2/cqipc.js";
import { makeRomMachine } from "./rommachine.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/* ------------------------------------------------------------------------------------------- *
 * ABSOLUTE constants.  None of these scales with a budget or a case count (standing rule 4).   *
 * ------------------------------------------------------------------------------------------- */

/** The ROM's probe ends here: a WRITE to the Qbus I/O-page doorbell.  The window is anchored by
    this CONTENT, not by an offset, so a change that adds or removes accesses ahead of it moves the
    window with it instead of silently grading a different 77 records. */
const ANCHOR = {kind: "QIOW", addr: DBL_BASE >>> 0};

/** How many accesses the anchored window must contain, AT LEAST.  MEASURED at exactly 77 over
    eight consecutive oracle captures and every JS walk since pcjsvax-5c1.  It is a FLOOR, not an
    expectation: the ROM gaining a probe step is progress and must not fail this run, while LOSING
    one means the firmware stopped exercising the adapter and the differential has gone blind. */
const WINDOW_FLOOR = 77;

/** The JS walk budget.  The anchor lands at instruction ~2,425,905 (MEASURED), so this carries
    ~24% margin.  It is asserted, not trusted: a walk that does not reach the anchor FAILS BY NAME
    (standing rule 6) rather than grading a short window. */
const JS_STEPS = 3000000;

/** The oracle's `step` budget.  The anchor lands at sim_time ~923,300; 1,200,000 covers it with
    margin and is what the eight-capture reproducibility measurement used. */
const ORACLE_STEPS = 1200000;

/** Free space the DEVTRACE log needs.  ONE capture of ORACLE_STEPS instructions is ~130 MB -- patch
    0006 traces every ReadReg, and ROM instruction fetch IS a ReadReg -- and the two captures run
    SEQUENTIALLY with the log deleted between them, so this is the peak, not the total.  HANDOFF.md
    §4: a broken run once filled the root filesystem.  Too little space FAILS the run; it does not
    skip it (pcjsvax-b6d: a gate check that cannot run now fails). */
const LOG_BYTES_REQUIRED = 1024 * 1024 * 1024;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure (rules 4 and 14).  The
    dominant terms are fixed: one 16 MB RAM allocation per machine, at most a handful of machines,
    and the DEVTRACE log is read in bounded chunks rather than slurped. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;
let PEAK_HEAP = 0;

/** How much of the DEVTRACE log to hold in memory at a time.  The log is ~130 MB; readFileSync on
    it would allocate that plus a same-sized string plus a line array (rule 14). */
const LOG_CHUNK_BYTES = 4 * 1024 * 1024;

/** ABSOLUTE ceiling on recorded accesses.  The JS walk also STOPS the instant the anchor is
    reached, which bounds the healthy case at the window itself.  This ceiling is for the unhealthy
    one, and it is not decoration: the first --selfcheck run of this file had a mutated machine
    probing the CQBIC forever and recording 2,999,869 accesses -- roughly 240 MB of strings for a
    window 77 long (rule 14).  Hitting the ceiling without the anchor FAILS BY NAME through
    anchoredWindow(); it is never a silent truncation. */
const EVENT_CAP = 100000;

const SENTINEL_BEGIN = "CQBICDIFF-BEGIN";
const SENTINEL_END   = "CQBICDIFF-END";

/* The KA655 firmware's own probe addresses, DERIVED from the modules that own them (rule 5).  Two
   register-space windows -- the CQBIC's five registers and the CQIPC local mount -- plus the Qbus
   I/O-page doorbell, which patch 0006 traces through a DIFFERENT pair of records (QIOR/QIOW, from
   vax_io.c's ReadIO/WriteIO) because it is dispatched by a different mechanism.  See cqipc.js's
   header: one register, two address paths, two dispatchers. */
const WINDOWS = [
    {name: "CQBIC", kinds: ["REGR", "REGW"], base: CQBIC_BASE >>> 0, size: CQBIC_SIZE},
    {name: "CQIPC", kinds: ["REGR", "REGW"], base: CQIPC_BASE >>> 0, size: CQIPC_SIZE},
    {name: "DBL",   kinds: ["QIOR", "QIOW"], base: DBL_BASE >>> 0,   size: DBL_SIZE}
];

/** The five CQBIC registers, by NAME, derived from cqbic.js's own register numbers. */
const CQBIC_REGS = [
    {rg: REG_SCR,  name: "SCR"},
    {rg: REG_DSER, name: "DSER"},
    {rg: REG_MEAR, name: "MEAR"},
    {rg: REG_SEAR, name: "SEAR"},
    {rg: REG_MBR,  name: "MBR"}
];

/* ------------------------------------------------------------------------------------------- *
 * Plumbing -- the same shape dbldiff.js / conoutdiff.js use, deliberately                      *
 * ------------------------------------------------------------------------------------------- */

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

function sampleHeap()
{
    let mu = process.memoryUsage();
    let used = mu.heapUsed + mu.external;
    if (used > PEAK_HEAP) PEAK_HEAP = used;
    return used;
}

function vaxRepo()
{
    if (process.env['PCJS_VAX_REPO']) return process.env['PCJS_VAX_REPO'];
    return path.resolve(__dirname, "../../../../../pcjs-vax");
}

function findSimhBin(pathArg)
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
    throw new Error("cqbicdiff needs a REAL SIMH microvax3900 built with patch 0006 (the DEVTRACE\n" +
        "records); it has no fixture fallback.  Build one with machines/dec/vax/tests/simh/build.sh\n" +
        "and pass --simh PATH.  Tried:\n  " + candidates.join("\n  "));
}

function findRom(pathArg)
{
    let p = pathArg || path.join(vaxRepo(), "open-simh/VAX/ka655x.bin");
    if (!fs.existsSync(p)) throw new Error(`ka655x.bin not found at ${p}; pass --rom PATH`);
    return p;
}

function runSimh(bin, script, iniPath, timeoutMs = 10 * 60 * 1000)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 28, timeout: timeoutMs});
}

function freeBytes(dir)
{
    /* fs.statfsSync landed in Node 18.15; if it is not there, say so and fail rather than proceed
       blind -- the whole point of the check is that this file writes a 130 MB log. */
    if (typeof fs.statfsSync !== "function") {
        throw new Error("cqbicdiff: fs.statfsSync is unavailable, so the free-space guard before a " +
            "~130 MB DEVTRACE log cannot run; upgrade Node (>= 18.15) rather than removing the guard");
    }
    let s = fs.statfsSync(dir);
    return s.bavail * s.bsize;
}

/* ------------------------------------------------------------------------------------------- *
 * Event normalization -- ONE shape, two engines                                                *
 * ------------------------------------------------------------------------------------------- */

/**
 * An access is `KIND ADDR LNT VAL PC`, exactly patch 0006's field order and widths, so the JS side
 * is built to the ORACLE's grammar rather than the oracle being reshaped to the JS's.
 *
 * VAL means what the C's record means, which is NOT the same on the two directions:
 *   REGR -- vax_sysdev.c's ReadReg logs `p->read (pa)`, the WHOLE longword cqbic_rd() returned;
 *           vax_mmu.h's ReadB()/ReadW() do the lane extraction ABOVE it.  So a byte read of the SCR
 *           POK bit records 00008000, not 00000080.
 *   REGW -- WriteReg logs the RAW value handed to it, before cqbic_wr()'s own shift and merge.
 *   QIOR -- ReadIO logs the ASSEMBLED datum after ReadQb(), positioned for the width.
 *   QIOW -- WriteIO logs the raw value, before dispatch.
 *
 * @param {string} kind
 * @param {number} addr
 * @param {number} lnt
 * @param {number} val
 * @param {number} pc
 * @returns {string}
 */
function event(kind, addr, lnt, val, pc)
{
    return `${kind} ${hex(addr)} ${hex(lnt)} ${hex(val)} ${hex(pc)}`;
}

/** @param {string} ev @returns {number} the address field */
function eventAddr(ev) { return parseInt(ev.split(" ")[1], 16) >>> 0; }

/** @param {string} ev @returns {string} the kind field */
function eventKind(ev) { return ev.split(" ")[0]; }

/**
 * inWindow(kind, addr) -- is this access one of the three windows this file grades?
 *
 * @param {string} kind
 * @param {number} addr
 * @returns {boolean}
 */
function inWindow(kind, addr)
{
    for (let w of WINDOWS) {
        if (w.kinds.indexOf(kind) < 0) continue;
        if (addr >= w.base && addr < (w.base + w.size)) return true;
    }
    return false;
}

/**
 * anchoredWindow(events, who)
 *
 * Truncate an access sequence at the ANCHOR, inclusive.  Reports BY NAME (standing rule 6) when the
 * anchor is absent -- that is a run that never reached comparison, not a short window to be graded.
 *
 * @param {Array.<string>} events
 * @param {string} who "oracle capture 1", "this machine", ...
 * @returns {{window: Array.<string>, problem: ?string}}
 */
function anchoredWindow(events, who)
{
    for (let i = 0; i < events.length; i++) {
        if (eventKind(events[i]) === ANCHOR.kind && eventAddr(events[i]) === ANCHOR.addr) {
            return {window: events.slice(0, i + 1), problem: null};
        }
    }
    return {window: events, problem: `${who} produced ${events.length} CQBIC/doorbell access(es) ` +
        `but NONE of them is the anchor (${ANCHOR.kind} at ${hex(ANCHOR.addr)}), so the ROM's Qbus ` +
        `adapter probe did not complete inside the budget -- nothing was graded`};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE O -- the oracle's access trace, from patch 0006's DEVTRACE records                     *
 * ------------------------------------------------------------------------------------------- */

/**
 * OracleRun -- every member is a plain object property so selfcheck() can PERTURB the shipped
 * capture path rather than substitute a copy of it (standing rule 11).
 */
const OracleRun = {

    /** @param {Object} opts @returns {number} the oracle's step budget */
    steps(opts) { return opts.oracleSteps; },

    /**
     * capture(simh, opts, tag)
     *
     * A real `boot cpu`, bounded by a breakpoint at the ROM entry so the starting point is defined,
     * then an EXACT step count with DEVTRACE armed.  The log goes to a FILE and not to stdout: see
     * this file's header for the measurement that forced that, and for what a shared stream did to
     * the records.
     *
     * SELF-VERIFYING (pcjsvax-e9e's lesson): SIMH treats argv[1] as a startup do file and EXITS 0
     * when it cannot open or finish one, so execFileSync never throws and a silently-dead oracle
     * looks like a well-formed empty answer.  Both sentinels must come back, and the run must stop
     * with `Step expired` -- anything else is a failure with a name, not a zero-length capture.
     *
     * @param {string} simh
     * @param {Object} opts
     * @param {string} tag
     * @returns {Array.<string>} normalized accesses, in order
     */
    capture(simh, opts, tag) {
        let logPath = path.join(opts.scratch, `cqbicdiff-${tag}.log`);
        if (fs.existsSync(logPath)) fs.unlinkSync(logPath);
        let free = freeBytes(opts.scratch);
        if (free < LOG_BYTES_REQUIRED) {
            throw new Error(`cqbicdiff: ${opts.scratch} has ${(free / (1 << 20)).toFixed(0)} MB free ` +
                `and this capture writes a DEVTRACE log of roughly 130 MB; refusing to start ` +
                `(HANDOFF.md §4 -- a broken run once filled the root filesystem)`);
        }
        let script = [
            "set console notelnet",
            "set cpu 16m",
            `load -r ${opts.rom}`,
            `break ${hex(VAX.PHYSMEM.ROM_BASE >>> 0)}`,
            "boot cpu",
            `nobreak ${hex(VAX.PHYSMEM.ROM_BASE >>> 0)}`,
            "set sysd debug=DEVTRACE",
            `set debug -n ${logPath}`,
            `echo ${SENTINEL_BEGIN}`,
            `step ${this.steps(opts)}`,
            `echo ${SENTINEL_END}`,
            "exit", ""
        ].join("\n");
        let out, events;
        try {
            out = runSimh(simh, script, path.join(opts.scratch, `cqbicdiff-${tag}.ini`));
            if (!/Breakpoint, PC:/.test(out)) {
                throw new Error(`cqbicdiff: SIMH never stopped at the ROM entry ` +
                    `${hex(VAX.PHYSMEM.ROM_BASE >>> 0)} after 'boot cpu'; the capture has no defined ` +
                    `starting point.  SIMH said:\n${out}`);
            }
            for (let s of [SENTINEL_BEGIN, SENTINEL_END]) {
                if (out.indexOf(s) < 0) {
                    throw new Error(`cqbicdiff: the oracle did not echo ${s}, so its do file did not ` +
                        `run to the end -- SIMH exits 0 in that case and the capture would look ` +
                        `merely empty (pcjsvax-e9e).  SIMH said:\n${out}`);
                }
            }
            if (!/Step expired/.test(out)) {
                throw new Error(`cqbicdiff: the oracle did not stop with 'Step expired' -- it halted ` +
                    `early, so the window it produced is not the one this file asked for.  SIMH ` +
                    `said:\n${out}`);
            }
            if (!fs.existsSync(logPath)) {
                throw new Error(`cqbicdiff: SIMH produced no DEVTRACE log at ${logPath}; the ` +
                    `simulator is almost certainly missing patch 0006`);
            }
            events = this.parse(logPath);
        } finally {
            if (fs.existsSync(logPath)) fs.unlinkSync(logPath);
        }
        return events;
    },

    /**
     * parse(logPath)
     *
     * Streams the log in bounded chunks (rule 14 -- it is ~130 MB) and keeps only accesses inside
     * the three windows.  The grammar is patch 0006's, documented in tests/simh/README.md.
     *
     * @param {string} logPath
     * @returns {Array.<string>}
     */
    parse(logPath) {
        const RE = /DEVTRACE:\s+(REGR|REGW|QIOR|QIOW)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)/;
        let events = [], fd = fs.openSync(logPath, "r"), buf = Buffer.allocUnsafe(LOG_CHUNK_BYTES);
        let carry = "", n;
        try {
            while ((n = fs.readSync(fd, buf, 0, LOG_CHUNK_BYTES, null)) > 0) {
                let text = carry + buf.toString("latin1", 0, n);
                let lines = text.split("\n");
                carry = lines.pop();
                for (let line of lines) {
                    let m = RE.exec(line);
                    if (!m) continue;
                    let kind = m[1], addr = parseInt(m[2], 16) >>> 0;
                    if (!inWindow(kind, addr)) continue;
                    if (events.length >= EVENT_CAP) continue;   // bounded exactly as the JS side is
                    events.push(event(kind, addr, parseInt(m[3], 16), parseInt(m[4], 16),
                                      parseInt(m[5], 16)));
                }
                sampleHeap();
            }
            let m = RE.exec(carry);
            if (m && inWindow(m[1], parseInt(m[2], 16) >>> 0)) {
                events.push(event(m[1], parseInt(m[2], 16), parseInt(m[3], 16), parseInt(m[4], 16),
                                  parseInt(m[5], 16)));
            }
        } finally {
            fs.closeSync(fd);
        }
        return events;
    }
};

/* ------------------------------------------------------------------------------------------- *
 * PHASE J -- this machine's access trace over the same ROM boot                                *
 * ------------------------------------------------------------------------------------------- */

/** The JS walk budget, as its own seam so selfcheck() can perturb the SHIPPED value. */
const Walk = {
    budget(opts) { return opts.jsSteps; }
};

/** The machine under test, as its own seam for the same reason.  rommachine.js is SHARED with
    romdiff.js and conoutdiff.js so that all three grade one machine (see that file's header). */
const MachineBuild = {
    build(romBytes) { return makeRomMachine(romBytes); }
};

/**
 * instrument(machine, evOut)
 *
 * Wraps the SHIPPED width-level accessors on the CQBIC, the CQIPC local mount and the doorbell,
 * COMPOSING over each original rather than replacing it (standing rule 11, which pcjsvax-aa5's own
 * probe defect proves applies to observers as much as to mutations: a probe that classified CQM
 * references by calling mapAddr() latched an error register on every failure path and reported 1
 * reference where the truth was 24,578).
 *
 * Nothing here calls a method that mutates.  The one value it needs that the accessor does not
 * return -- the WHOLE longword behind a byte or word READ, which is what the oracle's REGR record
 * carries -- comes from readReg(), which is a pure switch over already-latched state (see cqbic.js).
 *
 * @param {Object} machine as returned by makeRomMachine()
 * @param {Array.<string>} evOut appended in access order
 * @param {Object} state {anchored} -- set true the moment the anchor access is recorded, so the
 *   caller can stop the walk there.  That is the memory bound in the healthy case; EVENT_CAP is
 *   the one in the unhealthy case.
 */
function instrument(machine, evOut, state)
{
    let {cpu, cqbic, cqipc, dbl} = machine;
    const pc = () => cpu.regs[15] >>> 0;
    const push = (ev) => {
        if (evOut.length >= EVENT_CAP) return;
        evOut.push(ev);
        if (eventKind(ev) === ANCHOR.kind && eventAddr(ev) === ANCHOR.addr) state.anchored = true;
    };

    for (let [meth, lnt] of [["readLong", 4], ["readWord", 2], ["readByte", 1]]) {
        let orig = cqbic[meth].bind(cqbic);
        cqbic[meth] = function(addr) {
            let full = cqbic.readReg(((addr >>> 0) - (CQBIC_BASE >>> 0)) >>> 2);
            push(event("REGR", addr, lnt, full, pc()));
            return orig(addr);
        };
    }
    for (let [meth, lnt] of [["writeLong", 4], ["writeWord", 2], ["writeByte", 1]]) {
        let orig = cqbic[meth].bind(cqbic);
        cqbic[meth] = function(addr, val) {
            push(event("REGW", addr, lnt, val, pc()));
            return orig(addr, val);
        };
    }
    for (let [dev, kinds] of [[cqipc, ["REGR", "REGW"]], [dbl, ["QIOR", "QIOW"]]]) {
        for (let [meth, lnt] of [["readLong", 4], ["readWord", 2], ["readByte", 1]]) {
            if (typeof dev[meth] !== "function") continue;
            let orig = dev[meth].bind(dev);
            dev[meth] = function(addr) {
                let v = orig(addr);
                push(event(kinds[0], addr, lnt, v === null ? 0 : v, pc()));
                return v;
            };
        }
        for (let [meth, lnt] of [["writeLong", 4], ["writeWord", 2], ["writeByte", 1]]) {
            if (typeof dev[meth] !== "function") continue;
            let orig = dev[meth].bind(dev);
            dev[meth] = function(addr, val) {
                push(event(kinds[1], addr, lnt, val, pc()));
                return orig(addr, val);
            };
        }
    }
}

/**
 * runJS(romBytes, opts, mutate)
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {?function(Object)} [mutate] applied to the SHIPPED machine before the walk (selfcheck)
 * @returns {{events: Array.<string>, steps: number, deviceNames: Array.<string>}}
 */
function runJS(romBytes, opts, mutate = null)
{
    let machine = MachineBuild.build(romBytes);
    if (mutate) mutate(machine);
    let events = [], state = {anchored: false};
    instrument(machine, events, state);
    let {cpu} = machine;
    cpu.reset();
    cpu.boot(opts.magicByte);
    let steps = 0, budget = Walk.budget(opts);
    try {
        /* Stop AT the anchor, not at the budget: past it the ROM polls DSER indefinitely and every
           one of those accesses would be recorded and never graded.  The budget stays a CEILING,
           and it is still the seam the "js-budget-truncated" mutation perturbs -- a budget too
           small to reach the anchor fails, which is the property that mutation exists to prove. */
        while (steps < budget && !state.anchored) { cpu.stepCPU(1); steps++; }
    } catch (e) {
        /* A stop is not an error here: what matters is whether the anchor was reached, which
           anchoredWindow() answers by name either way. */
    }
    sampleHeap();
    return {events, steps, deviceNames: machine.devices.map((d) => d.name)};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE X -- the read-only rejection, CONSTRUCTED, because the ROM never does it                *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE = 0x01000000;
const MEM_MB  = MEMSIZE / (1024 * 1024);

const R_SCBB      = 0x00100000;
const R_MCHK_HDLR = 0x00102000;
const R_MERR_HDLR = 0x00103000;
const R_CODE      = 0x00104000;
const R_KSP       = 0x00110000;
const R_IS        = 0x00118000;
const HDLR_NOPS   = 16;

/* The widths SIMH's own QBA register table publishes (vax_io.c:158-166), which is all `EXAMINE QBA
   x` can show.  Both engines are masked to them so no difference is ever an artifact of what the
   oracle can print. */
const SCR_OBS_MASK  = 0xFFFF;
const DSER_OBS_MASK = 0xFF;
const MEAR_OBS_MASK = 0x1FFF;
const SEAR_OBS_MASK = 0xFFFFF;
const MBR_OBS_MASK  = 0x1FFFFFFF;

const OP_MOVL = OPCODES.indexOf("MOVL");
const OP_MOVW = OPCODES.indexOf("MOVW");
const OP_MOVB = OPCODES.indexOf("MOVB");
const OP_NOP  = OPCODES.indexOf("NOP");
for (let [name, opc] of [["MOVL", OP_MOVL], ["MOVW", OP_MOVW], ["MOVB", OP_MOVB], ["NOP", OP_NOP]]) {
    if (opc < 0 || opc > 0xFF) throw new Error(`cqbicdiff: ${name} opcode not found or not single-byte`);
}

/**
 * encodeOp(op) -- ONE op description, two engines (dbldiff.js's encodeOp shape).
 *
 * @param {Object} op {kind: "wl"|"ww"|"wb"|"rl", addr, val|reg}
 * @returns {Array.<number>} the bytes of exactly ONE instruction
 */
function encodeOp(op)
{
    let a = op.addr >>> 0;
    let abs = [0x9F, a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF];
    switch (op.kind) {
    case "wl": {
        let v = op.val >>> 0;
        return [OP_MOVL, 0x8F, v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF, ...abs];
    }
    case "ww": return [OP_MOVW, 0x8F, op.val & 0xFF, (op.val >>> 8) & 0xFF, ...abs];
    case "wb": return [OP_MOVB, 0x8F, op.val & 0xFF, ...abs];
    case "rl": return [OP_MOVL, ...abs, 0x50 | op.reg];
    }
    throw new Error("cqbicdiff: bad op kind " + op.kind);
}

/**
 * THE CONSTRUCTED CASES.
 *
 * `pre` is the register state deposited before the instruction runs, and it is what makes the
 * read-only cases prove something: MEAR is PRELOADED with a recognisable value, so a MEAR that
 * afterwards reads 0x0400 -- the PAGE of the faulting address, which is what cq_merr() latches --
 * proves the write did not land AND that the error latch fired, rather than merely proving the
 * register changed.  SEAR is preloaded too and must be UNTOUCHED: cq_merr() writes MEAR only.
 *
 * `fault: true` means the instruction must vector to SCB_MCHK, observed as PC sitting in the
 * handler.  Every case names what it is FOR, and the control cases are not decoration -- a device
 * that faulted on every write would satisfy the rejection cases alone.
 */
const CASES = [
    {name: "mear-write-long", fault: true,
     what: "MOVL to MEAR: vax_io.c:519 cq_merr(pa) + MACH_CHECK(MCHK_WRITE), NOT the WriteReg default",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "wl", addr: (CQBIC_BASE + (REG_MEAR << 2)) >>> 0, val: 0xFFFFFFFF}]},

    {name: "sear-write-long", fault: true,
     what: "MOVL to SEAR: the same branch; SEAR itself must be untouched, cq_merr latches MEAR only",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "wl", addr: (CQBIC_BASE + (REG_SEAR << 2)) >>> 0, val: 0xFFFFFFFF}]},

    {name: "mear-write-byte", fault: true,
     what: "MOVB into MEAR's second lane -- the byte path reaches the same read-only case",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "wb", addr: (CQBIC_BASE + (REG_MEAR << 2) + 1) >>> 0, val: 0xFF}]},

    {name: "sear-write-word", fault: true,
     what: "MOVW into SEAR's upper word -- the word path, and an odd-word lane",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "ww", addr: (CQBIC_BASE + (REG_SEAR << 2) + 2) >>> 0, val: 0xFFFF}]},

    {name: "mear-write-long-lost-error", fault: true,
     what: "cq_merr's OTHER branch: DSER already carries an unresolved error, so DSER<LST> is set too",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x80},
     ops: [{kind: "wl", addr: (CQBIC_BASE + (REG_MEAR << 2)) >>> 0, val: 0xFFFFFFFF}]},

    {name: "mear-read-long", fault: false,
     what: "CONTROL: MEAR is read-only, not inaccessible -- a READ must answer and must not latch",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "rl", addr: (CQBIC_BASE + (REG_MEAR << 2)) >>> 0, reg: 0}]},

    {name: "sear-read-long", fault: false,
     what: "CONTROL: the same for SEAR",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "rl", addr: (CQBIC_BASE + (REG_SEAR << 2)) >>> 0, reg: 0}]},

    {name: "scr-write-long", fault: false,
     what: "CONTROL: a WRITABLE register in the same block must still take a write -- and only its " +
           "RW bits (BHL|DBO), with POK reading back set regardless",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "wl", addr: (CQBIC_BASE + (REG_SCR << 2)) >>> 0, val: 0xFFFFFFFF}]},

    {name: "mbr-write-long", fault: false,
     what: "CONTROL: MBR takes a write, masked to CQMBR_MASK -- the register the ROM walks",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0x00},
     ops: [{kind: "wl", addr: (CQBIC_BASE + (REG_MBR << 2)) >>> 0, val: 0xFFFFFFFF}]},

    {name: "dser-write-one-to-clear", fault: false,
     what: "CONTROL: DSER is W1C -- writing 0x80 clears only that bit and leaves the rest standing",
     pre: {mear: 0x1234, sear: 0x5AA5, dser: 0xBD},
     ops: [{kind: "wl", addr: (CQBIC_BASE + (REG_DSER << 2)) >>> 0, val: 0x00000080}]}
];

/** The device under test in PHASE X, as its own seam so selfcheck() can perturb the SHIPPED
    construction and the SHIPPED methods (rule 11). */
const DeviceUnderTest = {
    /**
     * @param {Object} cpu
     * @param {Object} bus
     * @returns {Object} {cqbic, cqipc}
     */
    make(cpu, bus) {
        let cqbic = new CQBICVAX(cpu.exc, bus, MEMSIZE);
        let cqipc = new CQIPCVAX();
        cqbic.setIpc(cqipc);
        return {cqbic, cqipc};
    }
};

/** One machine, built once and reused for every case and every --selfcheck pass (rule 14). */
let X_MACHINE = null;

function xMachine(mutate)
{
    if (X_MACHINE && !mutate) return X_MACHINE;
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let {cqbic, cqipc} = DeviceUnderTest.make(cpu, bus);
    bus.addRegBlock([
        {base: CQBIC_BASE >>> 0, length: CQBIC_SIZE, dev: cqbic},
        {base: CQIPC_BASE >>> 0, length: CQIPC_SIZE, dev: cqipc}
    ]);
    let m = {bus, cpu, cqbic, cqipc};
    if (mutate) { mutate(m); return m; }
    X_MACHINE = m;
    sampleHeap();
    return m;
}

/**
 * runCaseJS(kase, m)
 *
 * Executes one case THROUGH THE CPU.  No case calls a device accessor: HANDOFF.md §7 premise 7 --
 * pcjsvax-855 shipped a stitch that a unit-level test would have "passed" over code nothing calls.
 *
 * @param {Object} kase
 * @param {Object} m
 * @returns {Object}
 */
function runCaseJS(kase, m)
{
    let {bus, cpu, cqbic, cqipc} = m;
    cqbic.reset();
    cqipc.reset();
    cpu.exc.cqDser = 0;
    cpu.exc.cqMear = 0;
    cpu.exc.sscBto = 0;
    cpu.exc.memErr = 0;
    cpu.exc.scbb = R_SCBB;
    cpu.regs[0] = 0;
    cpu.regs[14] = R_KSP;
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
    for (let k = 0; k < HDLR_NOPS; k++) {
        bus.setByte((R_MCHK_HDLR + k) >>> 0, OP_NOP);
        bus.setByte((R_MERR_HDLR + k) >>> 0, OP_NOP);
    }
    bus.setLong((R_SCBB + SCB.MCHK) >>> 0, R_MCHK_HDLR);
    bus.setLong((R_SCBB + SCB.MEMERR) >>> 0, R_MERR_HDLR);
    /* SIMH's `deposit QBA MEAR/SEAR/DSER` -- the same preload, on the same state.  cqDser/cqMear
       live on cpu.exc (one model, two latch sites; see cqbic.js's header), cq_sear on the device. */
    cpu.exc.cqMear = kase.pre.mear | 0;
    cpu.exc.cqDser = kase.pre.dser | 0;
    cqbic.sear = kase.pre.sear | 0;

    let bytes = [];
    for (let op of kase.ops) bytes.push(...encodeOp(op));
    for (let i = 0; i < bytes.length; i++) bus.setByte((R_CODE + i) >>> 0, bytes[i]);
    for (let k = 0; k < 16; k++) bus.setByte((R_CODE + bytes.length + k) >>> 0, OP_NOP);
    cpu.psl = 0;
    cpu.setPC(R_CODE);

    let stop = null;
    try {
        for (let s = 0; s < kase.ops.length; s++) cpu.stepCPU(1);
    } catch (e) {
        stop = e.reason || e.message || String(e);
    }
    sampleHeap();
    return {
        r0:   cpu.regs[0] >>> 0,
        pc:   cpu.regs[15] >>> 0,
        bto:  cpu.exc.sscBto >>> 0,
        scr:  cqbic.readReg(REG_SCR) & SCR_OBS_MASK,
        dser: cpu.exc.cqDser & DSER_OBS_MASK,
        mear: cpu.exc.cqMear & MEAR_OBS_MASK,
        sear: cqbic.sear & SEAR_OBS_MASK,
        mbr:  cqbic.mbr & MBR_OBS_MASK,
        stop
    };
}

const MARK = "CQXCASE";

/**
 * runCasesSimh(simh, opts) -- ONE invocation for the whole case list (mchkdiff.js's convention).
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Array.<?Object>} parallel to CASES; null for a case whose chunk never appeared, which
 *   grade() then reports BY NAME rather than silently dropping (standing rule 6)
 */
function runCasesSimh(simh, opts)
{
    let L = [`set cpu ${MEM_MB}m`, "set cpu simhalt"];
    for (let i = 0; i < CASES.length; i++) {
        let c = CASES[i];
        L.push(`echo ${MARK}${i}`, "reset -p all", "deposit MAPEN 0");
        L.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit KSP ${hex(R_KSP)}`,
               `deposit R14 ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`, "deposit R0 0",
               `deposit -l ${hex((R_SCBB + SCB.MCHK) >>> 0)} ${hex(R_MCHK_HDLR)}`,
               `deposit -l ${hex((R_SCBB + SCB.MEMERR) >>> 0)} ${hex(R_MERR_HDLR)}`);
        for (let k = 0; k < HDLR_NOPS; k++) {
            L.push(`deposit -b ${hex(R_MCHK_HDLR + k)} ${OP_NOP.toString(16)}`);
            L.push(`deposit -b ${hex(R_MERR_HDLR + k)} ${OP_NOP.toString(16)}`);
        }
        L.push(`deposit QBA MEAR ${hex(c.pre.mear & MEAR_OBS_MASK, 4)}`,
               `deposit QBA SEAR ${hex(c.pre.sear & SEAR_OBS_MASK, 5)}`,
               `deposit QBA DSER ${hex(c.pre.dser & DSER_OBS_MASK, 2)}`);
        let bytes = [];
        for (let op of c.ops) bytes.push(...encodeOp(op));
        for (let k = 0; k < bytes.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${bytes[k].toString(16)}`);
        for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_CODE + bytes.length + k)} ${OP_NOP.toString(16)}`);
        L.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`, `step ${c.ops.length}`);
        L.push("examine -h R0", "examine -h PC", "examine -h BTO");
        L.push("examine -h QBA SCR", "examine -h QBA DSER", "examine -h QBA MEAR",
               "examine -h QBA SEAR", "examine -h QBA MBR");
    }
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "cqbicdiff-cases.ini"));

    let results = new Array(CASES.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = +parts[i], chunk = parts[i + 1] || "";
        let g = (name) => new RegExp(`^${name}:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
        let f = {r0: g("R0"), pc: g("PC"), bto: g("BTO"), scr: g("SCR"), dser: g("DSER"),
                 mear: g("MEAR"), sear: g("SEAR"), mbr: g("MBR")};
        if (Object.values(f).some((x) => !x)) continue;
        results[idx] = {
            r0: parseInt(f.r0[1], 16) >>> 0,
            pc: parseInt(f.pc[1], 16) >>> 0,
            bto: parseInt(f.bto[1], 16) >>> 0,
            scr: parseInt(f.scr[1], 16) & SCR_OBS_MASK,
            dser: parseInt(f.dser[1], 16) & DSER_OBS_MASK,
            mear: parseInt(f.mear[1], 16) & MEAR_OBS_MASK,
            sear: parseInt(f.sear[1], 16) & SEAR_OBS_MASK,
            mbr: parseInt(f.mbr[1], 16) & MBR_OBS_MASK
        };
    }
    return results;
}

/** The fields compared on both engines, named once so a report can enumerate them. */
const X_FIELDS = ["r0", "pc", "bto", "scr", "dser", "mear", "sear", "mbr"];

/* ------------------------------------------------------------------------------------------- *
 * PHASE F -- coverage floors.  ABSOLUTE, printed on EVERY run, and they FAIL the run.          *
 * ------------------------------------------------------------------------------------------- */

/**
 * CoverageFloor -- the item's done-condition 4, in code.
 *
 * Every requirement here is a property of the SUBJECT (the ROM's probe, and the device's read-only
 * branch), never a count derived from what happened to run, so none of it scales down with a
 * budget or a case count (standing rule 4).  It is printed whether it holds or not, because a gate
 * that stops measuring is worse than one that fails (standing rule 13).
 */
const CoverageFloor = {

    /**
     * @param {Array.<string>} window the graded ROM access window
     * @param {Array.<Object>} xJs PHASE X's JS results, parallel to CASES
     * @returns {{problems: Array.<string>, report: Array.<string>}}
     */
    verify(window, xJs) {
        let problems = [], report = [];

        /* 1 -- all five CQBIC registers, by name, touched by the ROM inside the graded window. */
        let touched = new Map();
        for (let ev of window) {
            if (eventKind(ev) !== "REGR" && eventKind(ev) !== "REGW") continue;
            let a = eventAddr(ev);
            if (a < (CQBIC_BASE >>> 0) || a >= ((CQBIC_BASE >>> 0) + CQBIC_SIZE)) continue;
            let rg = (a - (CQBIC_BASE >>> 0)) >>> 2;
            touched.set(rg, (touched.get(rg) || 0) + 1);
        }
        report.push("CQBIC REGISTERS (graded ROM window): " +
            CQBIC_REGS.map((r) => `${r.name}=${touched.get(r.rg) || 0}`).join(" "));
        for (let r of CQBIC_REGS) {
            if (!touched.get(r.rg)) {
                problems.push(`COVERAGE FLOOR: the ROM never touched CQBIC register ${r.name} ` +
                    `(rg ${r.rg}, ${hex((CQBIC_BASE + (r.rg << 2)) >>> 0)}) inside the graded window -- ` +
                    `every walk that reaches the anchor drives all five, so zero means the window ` +
                    `moved or the register stopped being decoded`);
            }
        }

        /* 2 -- the SCR power-ok bit, OBSERVED SET in a value the ROM actually read back.  This is
           the item's done condition 2's "including the power-ok bit in SCR", and it is graded as a
           value the firmware saw, not as a field of this machine's state. */
        let pokReads = window.filter((ev) => {
            if (eventKind(ev) !== "REGR") return false;
            let a = eventAddr(ev);
            if (a < (CQBIC_BASE >>> 0) || a >= ((CQBIC_BASE >>> 0) + 4)) return false;
            return (parseInt(ev.split(" ")[3], 16) & CQSCR_POK) !== 0;
        }).length;
        report.push(`SCR POWER-OK (${hex(CQSCR_POK)}) set in ${pokReads} SCR read(s) the ROM made`);
        if (!pokReads) {
            problems.push(`COVERAGE FLOOR: not one SCR read in the graded window came back with the ` +
                `power-ok bit ${hex(CQSCR_POK)} set -- that bit is what the ROM's adapter probe ` +
                `looks for (vax_io.c qba_powerup), so the probe cannot have been graded`);
        }

        /* 3 -- the doorbell, in BOTH directions, driven by the ROM. */
        let dblR = window.filter((ev) => eventKind(ev) === "QIOR").length;
        let dblW = window.filter((ev) => eventKind(ev) === "QIOW").length;
        report.push(`DOORBELL ${hex(DBL_BASE >>> 0)}: ${dblR} read(s) ${dblW} write(s)`);
        if (!dblR || !dblW) {
            problems.push(`COVERAGE FLOOR: the ROM's graded window has ${dblR} doorbell read(s) and ` +
                `${dblW} write(s); both directions are required -- the write IS the anchor, so a ` +
                `zero here means the window is not what this file thinks it is`);
        }

        /* 4 -- the read-only rejection path, TAKEN, and taken as a rejection rather than as an
           undecoded address: DSER<MNX> latched, MEAR latched, and the SSC bus-timeout bits CLEAR. */
        let rejected = 0;
        for (let i = 0; i < CASES.length; i++) {
            let c = CASES[i], r = xJs[i];
            if (!c.fault || !r) continue;
            if (r.pc === R_MCHK_HDLR && (r.dser & 0x80) && r.bto === 0) rejected++;
        }
        report.push(`READ-ONLY REJECTION (MEAR/SEAR write): ${rejected} constructed case(s) took the ` +
            `branch with DSER<MNX> latched and BTO clear`);
        if (!rejected) {
            problems.push(`COVERAGE FLOOR: no constructed case reached the MEAR/SEAR read-only ` +
                `rejection with DSER<MNX> set and the SSC bus-timeout register clear.  The ROM ` +
                `NEVER writes those registers, so PHASE X is the only coverage this branch has; if ` +
                `it does not fire, the branch is ungraded`);
        }

        /* 5 -- the window itself did not shrink. */
        report.push(`WINDOW: ${window.length} access(es), floor ${WINDOW_FLOOR}`);
        if (window.length < WINDOW_FLOOR) {
            problems.push(`COVERAGE FLOOR: the graded window is ${window.length} access(es), below ` +
                `the measured floor of ${WINDOW_FLOOR} -- the ROM stopped exercising part of the ` +
                `adapter, or the anchor moved`);
        }
        return {problems, report};
    }
};

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * gradeWindow(jsWin, orWin)
 *
 * @param {Array.<string>} jsWin
 * @param {Array.<string>} orWin
 * @returns {Array.<string>} problems
 */
function gradeWindow(jsWin, orWin)
{
    let problems = [];
    if (jsWin.length !== orWin.length) {
        problems.push(`PHASE C: the anchored window is ${jsWin.length} access(es) on this machine ` +
            `and ${orWin.length} on the oracle -- the ROM's register traffic differs in COUNT, not ` +
            `merely in content`);
    }
    let n = Math.min(jsWin.length, orWin.length), shown = 0;
    for (let i = 0; i < n; i++) {
        if (jsWin[i] === orWin[i]) continue;
        if (++shown > 12) {
            problems.push(`PHASE C: ... and further differences past access ${i}`);
            break;
        }
        problems.push(`PHASE C: access ${i} differs\n    this machine: ${jsWin[i]}\n    oracle      : ${orWin[i]}`);
    }
    return problems;
}

/**
 * gradeCases(jsRes, orRes)
 *
 * @param {Array.<Object>} jsRes
 * @param {Array.<?Object>} orRes
 * @returns {Array.<string>} problems
 */
function gradeCases(jsRes, orRes)
{
    let problems = [];
    for (let i = 0; i < CASES.length; i++) {
        let c = CASES[i], a = jsRes[i], b = orRes[i];
        if (!b) {
            problems.push(`PHASE X: case "${c.name}" never reached comparison -- the oracle produced ` +
                `no parsable chunk for it (standing rule 6)`);
            continue;
        }
        for (let f of X_FIELDS) {
            if ((a[f] >>> 0) === (b[f] >>> 0)) continue;
            problems.push(`PHASE X: case "${c.name}" field ${f.toUpperCase()} -- this machine ` +
                `${hex(a[f])}, oracle ${hex(b[f])}   [${c.what}]`);
        }
        /* The fault/no-fault expectation is a property of the CASE, and is checked against the
           ORACLE's own answer as well: a case marked `fault` whose oracle PC is not in the handler
           means the case list is wrong, which is a defect in this file, not in the machine. */
        let oracleFaulted = (b.pc >>> 0) === (R_MCHK_HDLR >>> 0);
        if (oracleFaulted !== !!c.fault) {
            problems.push(`PHASE X: case "${c.name}" is declared fault=${!!c.fault} but the ORACLE ` +
                `${oracleFaulted ? "DID" : "did NOT"} vector to SCB_MCHK (oracle PC ${hex(b.pc)}) -- ` +
                `this file's own expectation is wrong`);
        }
    }
    return problems;
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck                                                                                   *
 * ------------------------------------------------------------------------------------------- */

/**
 * THE MUTATIONS.  Every one PERTURBS the shipped path -- it wraps a shipped method and alters what
 * that method's own body returned, or perturbs a shipped seam's value -- and never substitutes a
 * private reimplementation (standing rule 11).
 *
 * `phase` says which phase must catch it; a mutation caught only by a DIFFERENT phase than the one
 * it targets is reported as a miss, because that is rule 16's "caught by a check that is itself
 * broken" arriving quietly.
 */
const MUTATIONS = [
    {name: "scr-pok-clear", phase: "ROM",
     what: "the power-ok bit the ROM's adapter probe looks for stops being set in SCR reads",
     rom: (m) => {
         let orig = m.cqbic.readReg.bind(m.cqbic);
         m.cqbic.readReg = (rg) => { let v = orig(rg); return rg === REG_SCR ? (v & ~CQSCR_POK) : v; };
     }},

    {name: "dser-write-not-clear-on-write", phase: "ROM",
     what: "a write to DSER stops clearing the bits it names (the W1C semantics the ROM relies on)",
     rom: (m) => {
         let orig = m.cqbic.writeReg.bind(m.cqbic);
         m.cqbic.writeReg = (rg, val, sval, addr) => {
             let before = m.exc ? 0 : m.cqbic.exc.cqDser;
             let r = orig(rg, val, sval, addr);
             if (rg === REG_DSER) m.cqbic.exc.cqDser = before;
             return r;
         };
     }},

    {name: "mbr-mask-widened", phase: "X",
     what: "MBR stops masking to CQMBR_MASK.  Written for the ROM phase first and it SURVIVED " +
           "there -- every value the firmware's address-line walk writes is already inside the " +
           "mask, so removing the mask is a no-op on that data (see this file's header)",
     dev: (m) => {
         let orig = m.cqbic.writeReg.bind(m.cqbic);
         m.cqbic.writeReg = (rg, val, sval, addr) => {
             let r = orig(rg, val, sval, addr);
             if (rg === REG_MBR) m.cqbic.mbr = val | 0;
             return r;
         };
     }},

    {name: "cqbic-block-answers-as-ram", phase: "ROM",
     what: "the CQBIC's own 20 bytes answer as plain storage instead of as the device",
     rom: (m) => {
         let store = new Map();
         let origR = m.cqbic.readReg.bind(m.cqbic);
         m.cqbic.readReg = (rg) => { origR(rg); return store.has(rg) ? store.get(rg) : 0; };
         let origW = m.cqbic.writeReg.bind(m.cqbic);
         m.cqbic.writeReg = (rg, val, sval, addr) => { origW(rg, val, sval, addr); store.set(rg, val | 0); return true; };
     }},

    {name: "js-budget-truncated", phase: "ROM",
     what: "the walk stops before the ROM's probe completes, so the anchor is never reached",
     seam: () => { let orig = Walk.budget; Walk.budget = () => 1000; return () => { Walk.budget = orig; }; }},

    {name: "oracle-budget-truncated", phase: "ROM",
     what: "the oracle's own budget stops before the probe completes",
     seam: () => { let orig = OracleRun.steps; OracleRun.steps = () => 1000; return () => { OracleRun.steps = orig; }; }},

    {name: "readonly-write-bus-timeout", phase: "X",
     what: "MEAR/SEAR writes fall through to vax_sysdev.c's WriteReg default -- the SSC bus-timeout " +
           "bits SIMH never sets here.  THIS IS THE DEFECT pcjsvax-69a FOUND; it is the tripwire " +
           "that fails if that fix is reverted",
     dev: (m) => {
         let orig = m.cqbic.writeReg.bind(m.cqbic);
         m.cqbic.writeReg = (rg, val, sval, addr) => {
             let r = orig(rg, val, sval, addr);
             if (rg === REG_MEAR || rg === REG_SEAR) {
                 m.cqbic.exc.cqDser = 0;
                 m.cqbic.exc.cqMear = 0;
                 return false;
             }
             return r;
         };
     }},

    {name: "readonly-write-accepted", phase: "X",
     what: "MEAR/SEAR become writable -- the write lands and nothing faults",
     dev: (m) => {
         let orig = m.cqbic.writeReg.bind(m.cqbic);
         m.cqbic.writeReg = (rg, val, sval, addr) => {
             let r = orig(rg, val, sval, addr);
             if (rg === REG_MEAR) { m.cqbic.exc.cqMear = val | 0; return true; }
             if (rg === REG_SEAR) { m.cqbic.sear = val | 0; return true; }
             return r;
         };
     }},

    {name: "readonly-write-does-not-latch-merr", phase: "X",
     what: "the write is refused but the CQBIC forgets to record WHY -- DSER/MEAR never latch",
     dev: (m) => {
         let orig = m.cqbic.writeReg.bind(m.cqbic);
         m.cqbic.writeReg = (rg, val, sval, addr) => {
             let dser = m.cqbic.exc.cqDser, mear = m.cqbic.exc.cqMear;
             let r = orig(rg, val, sval, addr);
             if (rg === REG_MEAR || rg === REG_SEAR) {
                 m.cqbic.exc.cqDser = dser;
                 m.cqbic.exc.cqMear = mear;
             }
             return r;
         };
     }},

    {name: "merr-lost-error-not-latched", phase: "X",
     what: "cq_merr's SECOND branch goes missing: DSER<LST> stops being set when an unresolved " +
           "error was already there",
     dev: (m) => {
         let orig = m.cqbic.exc.cqMerr.bind(m.cqbic.exc);
         m.cqbic.exc.cqMerr = (addr) => { orig(addr); m.cqbic.exc.cqDser &= ~0x08; };
     }},

    {name: "dser-w1c-clears-whole-register", phase: "X",
     what: "a W1C write to DSER clears every bit rather than the ones it names",
     dev: (m) => {
         let orig = m.cqbic.writeReg.bind(m.cqbic);
         m.cqbic.writeReg = (rg, val, sval, addr) => {
             let r = orig(rg, val, sval, addr);
             if (rg === REG_DSER) m.cqbic.exc.cqDser = 0;
             return r;
         };
     }}
];

/**
 * selfcheck(simh, opts, romBytes, baseline)
 *
 * Each mutation is applied, the phase it targets is re-run, and it must be CAUGHT.  The ROM-phase
 * mutations reuse the oracle window the baseline already captured -- re-capturing it would cost
 * five minutes and could not differ, since a JS-side mutation cannot change what SIMH did.
 *
 * CAUGHT BY THE RIGHT PHASE, not merely caught.  Every mutation declares the phase GROUP that must
 * fail on it ("ROM" -- the anchored window and its comparison; "X" -- the constructed cases), and
 * a catch from the other group counts as a MISS.  That is standing rule 16 in its cheapest form: a
 * mutation reported CAUGHT by a check that was never written to see it certifies nothing, and this
 * file has already had one mutation (`mbr-mask-widened`) that looked catchable in the ROM phase and
 * was a literal no-op there.  Within a group the two failure shapes both count: a ROM mutation may
 * be caught either by an access differing or by the probe never reaching the anchor at all, and a
 * machine that stops completing the probe is exactly what a real regression looks like.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {Uint8Array} romBytes
 * @param {Object} baseline {orWin, orCases}
 * @returns {boolean} true when every mutation was caught
 */
function selfcheck(simh, opts, romBytes, baseline)
{
    console.log("\n=== --selfcheck: every mutation must be CAUGHT ===");
    let allCaught = true;
    for (let mu of MUTATIONS) {
        let romProblems = [], xProblems = [], restore = null;
        try {
            if (mu.seam) restore = mu.seam();
            if (mu.rom || mu.seam) {
                let js = runJS(romBytes, opts, mu.rom || null);
                let aw = anchoredWindow(js.events, "this machine");
                if (aw.problem) romProblems.push(`ROM: ${aw.problem}`);
                else romProblems.push(...gradeWindow(aw.window, baseline.orWin));
                if (mu.seam && mu.name === "oracle-budget-truncated") {
                    /* This one perturbs the ORACLE's seam, so it must be caught by re-capturing.
                       Bounded to one extra capture, and it is the only mutation that costs one. */
                    let ev = OracleRun.capture(simh, opts, "mut");
                    let ow = anchoredWindow(ev, "oracle capture (mutated)");
                    romProblems = ow.problem ? [`ROM: ${ow.problem}`] : gradeWindow(aw.window, ow.window);
                }
            }
            if (mu.dev) {
                let m = xMachine(mu.dev);
                let jsRes = CASES.map((c) => runCaseJS(c, m));
                xProblems.push(...gradeCases(jsRes, baseline.orCases));
            }
        } catch (e) {
            (mu.dev ? xProblems : romProblems).push(`threw: ${e.message}`);
        } finally {
            if (restore) restore();
        }
        let byGroup = {ROM: romProblems, X: xProblems};
        let caught = byGroup[mu.phase].length > 0;
        let elsewhere = (mu.phase === "ROM" ? xProblems : romProblems).length > 0;
        if (!caught) allCaught = false;
        console.log(`  ${caught ? "CAUGHT " : "SURVIVED"}  ${mu.name}  [phase ${mu.phase}]`);
        if (!caught) {
            console.log(`            ${mu.what}`);
            if (elsewhere) {
                console.log(`            NOTE: it DID fail the other phase group, which does not ` +
                    `count -- the phase written to see it did not (standing rule 16)`);
            }
        } else {
            console.log(`            first: ${byGroup[mu.phase][0].split("\n")[0]}`);
        }
    }
    return allCaught;
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                          *
 * ------------------------------------------------------------------------------------------- */

function parseArgs(argv)
{
    let opts = {simh: null, rom: null, jsSteps: JS_STEPS, oracleSteps: ORACLE_STEPS,
                selfcheck: false, scratch: null, magicByte: 2};
    for (let i = 2; i < argv.length; i++) {
        let a = argv[i];
        if (a === "--simh") opts.simh = argv[++i];
        else if (a === "--rom") opts.rom = argv[++i];
        else if (a === "--js-steps") opts.jsSteps = +argv[++i];
        else if (a === "--oracle-steps") opts.oracleSteps = +argv[++i];
        else if (a === "--scratch") opts.scratch = argv[++i];
        else if (a === "--selfcheck") opts.selfcheck = true;
        else if (a === "--ehkaa") i++;                 // accepted and ignored: the gate passes it to every check
        else throw new Error(`cqbicdiff: unknown argument ${a}`);
    }
    return opts;
}

function main()
{
    let opts = parseArgs(process.argv);
    opts.simh = findSimhBin(opts.simh);
    opts.rom = findRom(opts.rom);
    opts.scratch = opts.scratch || fs.mkdtempSync(path.join(os.tmpdir(), "cqbicdiff-"));
    fs.mkdirSync(opts.scratch, {recursive: true});
    console.log(`cqbicdiff: simh=${opts.simh}`);
    console.log(`cqbicdiff: rom=${opts.rom}  scratch=${opts.scratch}`);

    let romBytes = new Uint8Array(fs.readFileSync(opts.rom));
    let problems = [];

    /* ---- PHASE O: two INDEPENDENT oracle captures, which must agree over the window ---- */
    let or1 = OracleRun.capture(opts.simh, opts, "a");
    let ow1 = anchoredWindow(or1, "oracle capture 1");
    let or2 = OracleRun.capture(opts.simh, opts, "b");
    let ow2 = anchoredWindow(or2, "oracle capture 2");
    for (let w of [ow1, ow2]) if (w.problem) problems.push(`PHASE O: ${w.problem}`);
    if (!ow1.problem && !ow2.problem) {
        if (ow1.window.length !== ow2.window.length ||
            ow1.window.some((e, i) => e !== ow2.window[i])) {
            let i = ow1.window.findIndex((e, k) => e !== ow2.window[k]);
            problems.push(`PHASE O: two independent oracle captures DISAGREE over the anchored ` +
                `window (lengths ${ow1.window.length} and ${ow2.window.length}, first difference at ` +
                `access ${i < 0 ? "(length only)" : i}) -- the oracle is not reproducible here and ` +
                `this file will not silently grade the part they happen to share`);
        }
    }
    console.log(`\n=== PHASE O: oracle ===`);
    console.log(`  capture 1: ${or1.length} access(es), anchored window ${ow1.window.length}`);
    console.log(`  capture 2: ${or2.length} access(es), anchored window ${ow2.window.length}`);

    /* ---- PHASE J: this machine ---- */
    let js = runJS(romBytes, opts);
    let jw = anchoredWindow(js.events, "this machine");
    if (jw.problem) problems.push(`PHASE J: ${jw.problem}`);
    console.log(`\n=== PHASE J: this machine ===`);
    console.log(`  ${js.steps} instruction(s), ${js.events.length} access(es), anchored window ${jw.window.length}`);
    console.log(`  devices: ${js.deviceNames.join(", ")}`);

    /* ---- PHASE C ---- */
    console.log(`\n=== PHASE C: the ROM's register traffic, in order ===`);
    if (!jw.problem && !ow1.problem) {
        let p = gradeWindow(jw.window, ow1.window);
        problems.push(...p);
        console.log(p.length ? `  ${p.length} difference(s)` :
            `  MATCH over all ${jw.window.length} CQBIC/doorbell access(es) the ROM's probe makes`);
    } else {
        console.log(`  not run -- see PHASE O/J above`);
    }

    /* ---- PHASE X ---- */
    console.log(`\n=== PHASE X: the read-only rejection the ROM never exercises ===`);
    let m = xMachine(null);
    let jsCases = CASES.map((c) => runCaseJS(c, m));
    let orCases = runCasesSimh(opts.simh, opts);
    let xp = gradeCases(jsCases, orCases);
    problems.push(...xp);
    for (let i = 0; i < CASES.length; i++) {
        let r = orCases[i];
        console.log(`  ${CASES[i].fault ? "FAULT " : "ok    "} ${CASES[i].name.padEnd(30)} ` +
            (r ? `PC=${hex(r.pc)} BTO=${hex(r.bto)} DSER=${hex(r.dser, 2)} MEAR=${hex(r.mear, 4)} ` +
                 `SEAR=${hex(r.sear, 5)} SCR=${hex(r.scr, 4)} MBR=${hex(r.mbr)}`
               : "(oracle produced no chunk)"));
    }
    console.log(`  ${xp.length ? xp.length + " difference(s)" : CASES.length + " case(s), all fields identical on both engines"}`);

    /* ---- PHASE F ---- */
    console.log(`\n=== PHASE F: coverage floors (graded) ===`);
    let fl = CoverageFloor.verify(jw.window, jsCases);
    for (let line of fl.report) console.log(`  ${line}`);
    problems.push(...fl.problems);

    /* ---- memory bound (rules 4 and 14) ---- */
    sampleHeap();
    console.log(`\npeak heap+external: ${(PEAK_HEAP / (1 << 20)).toFixed(1)} MB (bound ${(MAX_HEAP_BYTES / (1 << 20)).toFixed(0)} MB)`);
    if (PEAK_HEAP > MAX_HEAP_BYTES) {
        problems.push(`MEMORY: peak heap+external ${(PEAK_HEAP / (1 << 20)).toFixed(1)} MB exceeds the ` +
            `absolute bound ${(MAX_HEAP_BYTES / (1 << 20)).toFixed(0)} MB (HANDOFF.md standing rule 14)`);
    }

    let ok = problems.length === 0;
    if (!ok) {
        console.log(`\n=== ${problems.length} PROBLEM(S) ===`);
        for (let p of problems) console.log(`  ${p}`);
    }

    if (opts.selfcheck) {
        if (!ok) {
            console.log(`\ncqbicdiff: --selfcheck NOT RUN -- the baseline is already failing, so a ` +
                `mutation "caught" by that failure would certify nothing (standing rule 16)`);
        } else if (!selfcheck(opts.simh, opts, romBytes, {orWin: ow1.window, orCases})) {
            console.log(`\ncqbicdiff: FAIL -- a mutation SURVIVED, which is a coverage hole, not a ` +
                `tuning knob (standing rule 3)`);
            ok = false;
        }
    }

    /* The bound is re-checked AFTER --selfcheck, because that is where the machines multiply and
       where a mutated machine can run away (rule 14). */
    sampleHeap();
    console.log(`\npeak heap+external (final): ${(PEAK_HEAP / (1 << 20)).toFixed(1)} MB ` +
        `(bound ${(MAX_HEAP_BYTES / (1 << 20)).toFixed(0)} MB)`);
    if (PEAK_HEAP > MAX_HEAP_BYTES) {
        console.log(`MEMORY: peak heap+external exceeds the absolute bound (standing rule 14)`);
        ok = false;
    }

    console.log(`\ncqbicdiff: ${ok ? "PASS" : "FAIL"}`);
    process.exit(ok ? 0 : 1);
}

main();
