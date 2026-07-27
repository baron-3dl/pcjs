/**
 * @fileoverview Differential test: the CQBIC doorbell (cq_ipc) at BOTH of its address paths -- the
 *               Qbus I/O-page IPCR and the CQIPCBASE local register -- vs. a real Open SIMH
 *               microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-b8a.  modules/v2/cqipc.js graded against the live oracle: VALUES and SIDE EFFECTS, not
 * merely "does not fault".  The ROM's `?91` self-test failure was the Qbus I/O-page IPCR read going
 * unanswered (see cqipc.js's header for the four-way measurement that corrected an earlier, wrong
 * attribution to CMCTL), and tests/conoutdiff.js is the instrument that shows the ROM getting past
 * it.  That instrument grades a BYTE STREAM; it can tell you the self-test passed and can tell you
 * nothing at all about whether the register is right.  This file is the other half, and the item's
 * own "how an agent would cheat this" is what it exists to make impossible: returning whatever
 * value silences the ROM satisfies conoutdiff and fails every phase below.
 *
 * EVERYTHING IS GRADED THROUGH REAL CPU EXECUTION -- no case calls a device accessor directly.
 * HANDOFF.md §7 premise 7: pcjsvax-855 shipped a stitch in ssc.js that a unit-level test of
 * ssc.readWord() would have "passed", and the branches turned out to be unreachable because mmu.js
 * handles unaligned accesses one layer above the device.  The same trap is live here in a sharper
 * form: mmu.js's writeL() ALREADY splits an aligned Qbus longword write into two word cycles, so
 * DBLVAX.writeLong() is not on any CPU path -- a direct-call test would have graded a method the
 * machine never uses while missing the split that actually implements WriteIO().
 *
 * THE TWO ADDRESS PATHS ARE THE SUBJECT, NOT A DETAIL
 * ----------------------------------------------------
 * vax_io.c's own comment above cqipc_rd() is "IPC can be read as local register or as Qbus I/O.
 * Because of the W1C."  One register, two mounts, and the mounts are NOT interchangeable: local
 * register space dispatches at LONGWORD granularity through vax_sysdev.c's ReadReg()/WriteReg(),
 * the Qbus I/O page at WORD granularity through vax_io.c's ReadIO()/WriteIO() with a 2-byte DIB.
 * MEASURED on the oracle, and each of these is a graded case below:
 *
 *      MOVL @#20081F40,R0          local:  answers, no exception
 *      MOVL @#20001F40,R0          Qbus:   MACHINE CHECK, BTO=00000000, DSER=80
 *      MOVL #FFFFFFFF,@#20081F40   local:  cq_ipc = 0161, no exception
 *      MOVL #FFFFFFFF,@#20001F40   Qbus:   cq_ipc = 0161 AND a DEFERRED MEMERR interrupt
 *
 * A test that graded only one mount would pass an implementation that got the other one backwards,
 * and a test that graded them as two independent registers would pass an implementation with two
 * separate copies of the state -- which the W1C bit makes observable, and which the
 * `two-mounts-have-separate-state` mutation below is there to catch.
 *
 * THE I/O-PAGE ADDRESS IS AUTOCONFIGURED, SO PHASE A ASKS THE ORACLE
 * -------------------------------------------------------------------
 * vax_io.c:155's `DIB qba_dib = { IOBA_AUTO, IOLN_DBL, ... }` leaves the doorbell's I/O-page address
 * to autoconfiguration.  cqipc.js writes it as a constant and says so; PHASE A re-derives it from
 * `SHOW QBA IOSPACE` on the LIVE oracle every run and fails if the QBA row disagrees in base or in
 * length.  The same output supplies the oracle's OTHER I/O-page device windows (DZ, RQ, TS, RL, XQ,
 * TQ, LPT -- real SIMH devices this project implements none of), which PHASE W needs in order to
 * exclude them BY NAME rather than mistake them for a decode this machine is missing.
 *
 * PHASE W IS THE BEACHHEAD ASSERTION
 * ------------------------------------
 * Decoding the Qbus I/O page at all is new (bus.js's addIoPage()), and the item's scope is "the
 * doorbell ONLY -- every other I/O page address must keep bus-faulting exactly as it does today, and
 * that must be asserted, not assumed."  PHASE W probes a programmatically derived address set --
 * the window's own bytes, its immediate neighbours, tests/mchkdiff.js's OWN candidatesFor() pool for
 * this range (imported, not re-derived), and a stride sweep of the whole page -- and requires that
 * the set of addresses THIS MACHINE decodes inside the I/O page is EXACTLY the doorbell window.
 * Against the oracle it requires identical classification at every probed address except those
 * inside a PHASE A device window, which are excluded by name and counted.
 *
 *      node machines/dec/vax/tests/dbldiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch build)
 *        --cases N         randomized cases (default RANDOM_CASES_DEFAULT; below the fixed floor
 *                           the run FAILS rather than clamping up)
 *        --seed S          PRNG seed, printed on every run so a failure is reproducible
 *        --selfcheck       prove the differential detects deliberate defects
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
import CQBICVAX, { CQBIC_BASE, REG_DSER, CQDSER_SME } from "../modules/v2/cqbic.js";
import CQIPCVAX, {
    DBLVAX, CQIPC_BASE, CQIPC_SIZE, DBL_BASE, DBL_SIZE,
    CQIPC_QME, CQIPC_INV, CQIPC_AHLT, CQIPC_DBIE, CQIPC_LME, CQIPC_DB,
    CQIPC_W1C, CQIPC_RW, CQIPC_MASK
} from "../modules/v2/cqipc.js";
/* RANGES already carries candidatesFor()'s output per reserved range, so importing the RESULT
   rather than the function is what keeps this file from re-deriving a pool mchkdiff.js already
   derived (rule 5) -- and what makes the load-time assertion below a statement about the addresses
   mchkdiff ACTUALLY probes, not about addresses this file recomputed and hoped were the same. */
import { RANGES } from "./mchkdiff.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/** 16MB, the SIMH microvax3900 default and every other differential's size. */
const MEMSIZE = 0x01000000;
const MEM_MB  = MEMSIZE / (1024 * 1024);

/* Fixed physical layout, MAPPING OFF -- the convention cmctldiff.js / tmrdiff.js use.  TWO handler
   pages, not one: a machine check and a deferred memory error must be told apart by PC alone, and
   this file grades a case (the Qbus longword write) whose whole point is that it takes the SECOND
   one and not the first. */
const R_SCBB       = 0x00100000;
const R_MCHK_HDLR  = 0x00102000;
const R_MERR_HDLR  = 0x00103000;
const R_CODE       = 0x00104000;
const R_KSP        = 0x00110000;
/* A machine check is a SEVERE exception: intexc() reloads SP from the INTERRUPT stack regardless of
   mode, so IS must be set or the frame push itself faults inside `in_ie` and SIMH stops hard.
   sscunaligneddiff.js's header records measuring exactly that. */
const R_IS         = 0x00118000;

/** NOPs deposited at each handler page, and therefore the width of the PC window classify() reads
    as "the machine dispatched here". */
const HDLR_NOPS = 16;

/** How many general registers a case may observe. */
const OBS_REGS = 4;

/** Randomized cases, and the FIXED floor beneath them.  Neither scales with anything (rule 4). */
const RANDOM_CASES_DEFAULT = 120;
const RANDOM_CASES_FLOOR   = 48;

/** PHASE W: the fixed minimum number of I/O-page addresses that must actually reach comparison.
    An ABSOLUTE floor -- it does not scale with the probe set, so shrinking the probe set fails the
    run instead of quietly lowering the bar (rule 4). */
const WINDOW_PROBE_FLOOR = 40;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure, not scaled by case
    count (rules 4 and 14).  The dominant term is fixed: one 16MB RAM allocation per machine
    VARIANT, and --selfcheck's wiring mutations add at most a handful of variants. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;
let PEAK_HEAP = 0;

function sampleHeap()
{
    let mu = process.memoryUsage();
    let used = mu.heapUsed + mu.external;
    if (used > PEAK_HEAP) PEAK_HEAP = used;
    return used;
}

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

/* The widths SIMH's own QBA register table publishes (vax_io.c:158-166), which is all `EXAMINE QBA
   x` can show.  Comparisons are masked to them on BOTH engines so the differential never reports a
   difference that is an artifact of what the oracle is able to print.  DSER/MEAR are graded
   exhaustively by tests/cqmerrdiff.js; here they are corroborating evidence that a Qbus error latch
   fired, not the subject. */
const IPC_OBS_MASK  = 0xFFFF;
const DSER_OBS_MASK = 0xFF;
const MEAR_OBS_MASK = 0x1FFF;

/* ------------------------------------------------------------------------------------------- *
 * Plumbing -- the same shape cmctldiff.js / romdiff.js use, deliberately                        *
 * ------------------------------------------------------------------------------------------- */

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
    throw new Error("dbldiff needs a REAL SIMH microvax3900; it has no fixture fallback.  Build one\n" +
        "with machines/dec/vax/tests/simh/build.sh and pass --simh PATH.  Tried:\n  " + candidates.join("\n  "));
}

function runSimh(bin, script, iniPath, timeoutMs = 5 * 60 * 1000)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: timeoutMs});
}

/** The same mulberry32 every VAX differential in this tree uses. */
function mulberry32(a)
{
    return function() {
        a |= 0; a = (a + 0x6D2B79F5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

/* ------------------------------------------------------------------------------------------- *
 * Instruction encoding -- ONE op description, two engines (cmctldiff.js's encodeOp shape)       *
 * ------------------------------------------------------------------------------------------- */

const OP_MOVL   = OPCODES.indexOf("MOVL");
const OP_MOVW   = OPCODES.indexOf("MOVW");
const OP_MOVB   = OPCODES.indexOf("MOVB");
const OP_MOVZWL = OPCODES.indexOf("MOVZWL");
const OP_MOVZBL = OPCODES.indexOf("MOVZBL");
const OP_NOP    = OPCODES.indexOf("NOP");
for (let [name, opc] of [["MOVL", OP_MOVL], ["MOVW", OP_MOVW], ["MOVB", OP_MOVB],
                         ["MOVZWL", OP_MOVZWL], ["MOVZBL", OP_MOVZBL], ["NOP", OP_NOP]]) {
    if (opc < 0 || opc > 0xFF) throw new Error(`dbldiff: ${name} opcode not found or not single-byte`);
}

/**
 * encodeOp(op)
 *
 * The reads are MOVZWL/MOVZBL rather than MOVW/MOVB for the reason cmctldiff.js records: a MOVW/MOVB
 * into a register leaves the upper bits alone, so the readback would mix this case's answer with the
 * previous case's leftovers and agree for the wrong reason.
 *
 * @param {Object} op {kind: "wl"|"ww"|"wb"|"rl"|"rw"|"rb", addr, val|reg}
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
    case "ww":
        return [OP_MOVW, 0x8F, op.val & 0xFF, (op.val >>> 8) & 0xFF, ...abs];
    case "wb":
        return [OP_MOVB, 0x8F, op.val & 0xFF, ...abs];
    case "rl":
        return [OP_MOVL, ...abs, 0x50 | op.reg];
    case "rw":
        return [OP_MOVZWL, ...abs, 0x50 | op.reg];
    case "rb":
        return [OP_MOVZBL, ...abs, 0x50 | op.reg];
    }
    throw new Error("dbldiff: bad op kind " + op.kind);
}

/* ------------------------------------------------------------------------------------------- *
 * The two mounts, named once and derived everywhere else (rule 5)                               *
 * ------------------------------------------------------------------------------------------- */

/** The two address paths to the SAME register.  `base`/`size` come from cqipc.js, never restated.
    `space` is what the mount's UNDECODED neighbours do -- the Qbus mechanism (cq_merr, no ssc_bto)
    or the register-space one (ssc_bto set, then the machine check). */
const MOUNTS = [
    {name: "IOP", base: DBL_BASE,   size: DBL_SIZE,   space: "qbus"},
    {name: "REG", base: CQIPC_BASE, size: CQIPC_SIZE, space: "reg"}
];

if (DBL_SIZE !== CQIPC_SIZE) {
    throw new Error(`dbldiff: cqipc.js gives the doorbell ${DBL_SIZE} byte(s) on the Qbus and ` +
        `${CQIPC_SIZE} in register space; vaxmod_defs.h's CQIPCSIZE and vax_io.c's IOLN_DBL are ` +
        `both 2 and one of the two constants is wrong.`);
}

/** mchkdiff.js's OWN candidate pool for this range, IMPORTED rather than re-derived (rule 5, and
    rule 7: two files' idea of one scope must not drift).  If any of those addresses ever landed
    inside the doorbell window, mchkdiff's committed EXPECTED_CALIBRATION for IOPAGE would silently
    change meaning -- so that is asserted at load time here, in the file that created the risk. */
const IOPAGE_RANGE = RANGES.find((r) => r.base === (VAX.PHYSMEM.IOPAGE_BASE >>> 0));
if (!IOPAGE_RANGE) {
    throw new Error("dbldiff: mchkdiff.js's RANGES no longer contains VAX.PHYSMEM.IOPAGE_BASE -- " +
        "the I/O page left BusVAX.RESERVED, which would also delete cqmerrdiff.js's own subject " +
        "from its pool (see bus.js's addIoPage()).");
}
for (let a of IOPAGE_RANGE.addrs) {
    for (let off = 0; off < 4; off++) {
        let x = (a + off) >>> 0;
        if (x >= DBL_BASE && x < DBL_BASE + DBL_SIZE) {
            throw new Error(`dbldiff: mchkdiff.js probes 0x${hex(x)}, which pcjsvax-b8a has now ` +
                `decoded as part of the doorbell window -- mchkdiff's EXPECTED_CALIBRATION for ` +
                `IOPAGE must be re-measured, and this assertion is what says so.`);
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE A -- the oracle's own I/O-page map                                                      *
 * ------------------------------------------------------------------------------------------- */

/**
 * ioSpace(simh, opts)
 *
 * `SHOW QBA IOSPACE`, parsed.  Rows look like
 *
 *      20001F40 - 20001F41              1 QBA
 *      20000040 - 2000005F* 0C0-0DC   4 4 DZ
 *
 * -- a base, an end (INCLUSIVE), an optional `*`, then vector/BR/count columns that vary in width,
 * and the device name LAST.  Only the two addresses and the trailing name are parsed, because those
 * are the only two things this file uses and the middle columns are the part most likely to change
 * shape between SIMH versions.
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Array.<{base: number, end: number, name: string}>}
 */
function ioSpace(simh, opts)
{
    let out = runSimh(simh, `set cpu ${MEM_MB}m\nshow qba iospace\nexit\n`,
                      path.join(opts.scratch, "dbldiff-iospace.ini"));
    let rows = [];
    let re = /^([0-9A-Fa-f]{8}) - ([0-9A-Fa-f]{8})\*?(.*)$/gm;
    let m;
    while ((m = re.exec(out)) !== null) {
        let tail = m[3].trim().split(/\s+/);
        rows.push({base: parseInt(m[1], 16) >>> 0, end: parseInt(m[2], 16) >>> 0,
                   name: tail.length ? tail[tail.length - 1] : "?"});
    }
    if (!rows.length) {
        throw new Error("dbldiff: could not parse any row out of `SHOW QBA IOSPACE`; SIMH said:\n" + out);
    }
    return rows;
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test -- ONE per wiring variant, reused across every case (rule 14)          *
 * ------------------------------------------------------------------------------------------- */

/**
 * makeMachine(opts)
 *
 * RAM at 0, the CQBIC and CQIPC local registers, and the Qbus I/O page with the doorbell on it.  No
 * console, no NVR, no timers: nothing graded here touches any of them.
 *
 * The three `opts` fields exist ONLY for --selfcheck's WIRING mutations, which are defects in how
 * the device is mounted rather than in what it computes and so cannot be expressed by perturbing
 * cqipc.js (HANDOFF.md standing rule 11 -- the shipped construction is perturbed, not replaced):
 *
 *   dblBaseDelta   move the Qbus window within the page
 *   dblSizeDelta   widen or narrow the Qbus window
 *   splitState     give the Qbus mount its OWN CQIPCVAX, i.e. two copies of one register
 *
 * @param {Object} [opts]
 * @returns {Object}
 */
function makeMachine(opts = {})
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let cqbic = new CQBICVAX(cpu.exc);
    let cqipc = new CQIPCVAX();
    cqbic.setIpc(cqipc);
    let dblIpc = opts.splitState ? new CQIPCVAX() : cqipc;
    let dbl = new DBLVAX(dblIpc);
    bus.addRegBlock([
        {base: CQBIC_BASE, length: 0x14, dev: cqbic},
        {base: CQIPC_BASE, length: CQIPC_SIZE, dev: cqipc}
    ]);
    bus.addIoPage([{
        base: (DBL_BASE + (opts.dblBaseDelta || 0)) >>> 0,
        length: (DBL_SIZE + (opts.dblSizeDelta || 0)) >>> 0,
        dev: dbl
    }]);
    sampleHeap();
    return {bus, cpu, cqbic, cqipc, dblIpc, dbl};
}

/** One machine per wiring variant, built once and reused for every case and every --selfcheck pass
    (HANDOFF.md standing rule 14: tests/cdgdiff.js reached 8.6GB RSS building one per case). */
const MACHINES = new Map();

function machine(opts = {})
{
    let key = `${opts.dblBaseDelta || 0}:${opts.dblSizeDelta || 0}:${opts.splitState ? 1 : 0}`;
    if (!MACHINES.has(key)) MACHINES.set(key, makeMachine(opts));
    return MACHINES.get(key);
}

/**
 * runCaseJS(kase, opts)
 *
 * Executes one case on the shared machine, THROUGH THE CPU.  Never calls a device accessor.
 *
 * @param {Object} kase {name, ipc0, psl, ops, steps}
 * @param {Object} [opts] wiring variant
 * @returns {Object}
 */
function runCaseJS(kase, opts = {})
{
    let m = machine(opts);
    let {bus, cpu, cqbic, cqipc, dblIpc} = m;

    /* SIMH's per-case `reset -p all`: qba_reset() zeroes cq_dser/cq_mear/cq_sear/cq_ipc, and
       sysd_powerup() zeroes ssc_bto.  cpu.exc carries DSER/MEAR/BTO/mem_err in this tree. */
    cqipc.reset();
    if (dblIpc !== cqipc) dblIpc.reset();
    cqbic.reset();
    cpu.exc.cqDser = 0;
    cpu.exc.cqMear = 0;
    cpu.exc.sscBto = 0;
    cpu.exc.memErr = 0;
    cpu.exc.scbb = R_SCBB;
    cpu.regs[14] = R_KSP;
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
    for (let k = 0; k < OBS_REGS; k++) cpu.regs[k] = 0;
    for (let k = 0; k < HDLR_NOPS; k++) {
        bus.setByte((R_MCHK_HDLR + k) >>> 0, OP_NOP);
        bus.setByte((R_MERR_HDLR + k) >>> 0, OP_NOP);
    }
    bus.setLong((R_SCBB + SCB.MCHK) >>> 0, R_MCHK_HDLR);
    bus.setLong((R_SCBB + SCB.MEMERR) >>> 0, R_MERR_HDLR);
    if (kase.ipc0 !== undefined && kase.ipc0 !== null) {
        cqipc.ipc = kase.ipc0 | 0;
        if (dblIpc !== cqipc) dblIpc.ipc = kase.ipc0 | 0;
    }

    let bytes = [];
    for (let op of kase.ops) bytes.push(...encodeOp(op));
    for (let i = 0; i < bytes.length; i++) bus.setByte((R_CODE + i) >>> 0, bytes[i]);
    for (let k = 0; k < 16; k++) bus.setByte((R_CODE + bytes.length + k) >>> 0, OP_NOP);
    cpu.psl = (kase.psl || 0) | 0;
    cpu.setPC(R_CODE);

    let stop = null;
    try {
        for (let s = 0; s < (kase.steps || kase.ops.length); s++) cpu.stepCPU(1);
    } catch (e) {
        stop = e.reason || e.message || String(e);
    }
    sampleHeap();
    return {
        regs: Array.from({length: OBS_REGS}, (_, i) => cpu.regs[i] >>> 0),
        pc: cpu.regs[15] >>> 0,
        bto: cpu.exc.sscBto >>> 0,
        ipc: (cqipc.ipc >>> 0) & IPC_OBS_MASK,
        dser: (cpu.exc.cqDser >>> 0) & DSER_OBS_MASK,
        mear: (cpu.exc.cqMear >>> 0) & MEAR_OBS_MASK,
        stop
    };
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side -- ONE invocation for the whole case list (mchkdiff.js's runBatch convention)   *
 * ------------------------------------------------------------------------------------------- */

const MARK = "DBCASE";

/**
 * runCasesSimh(simh, opts, cases)
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {Array.<Object>} cases
 * @returns {Array.<?Object>} parallel to `cases`; null for a case whose chunk never appeared, which
 *   grade() then reports BY NAME (HANDOFF.md standing rule 6) instead of silently dropping
 */
function runCasesSimh(simh, opts, cases)
{
    let L = [`set cpu ${MEM_MB}m`, "set cpu simhalt"];
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        L.push(`echo ${MARK}${i}`, "reset -p all", "deposit MAPEN 0");
        L.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit KSP ${hex(R_KSP)}`,
               `deposit R14 ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`,
               `deposit -l ${hex((R_SCBB + SCB.MCHK) >>> 0)} ${hex(R_MCHK_HDLR)}`,
               `deposit -l ${hex((R_SCBB + SCB.MEMERR) >>> 0)} ${hex(R_MERR_HDLR)}`);
        for (let k = 0; k < OBS_REGS; k++) L.push(`deposit R${k} 0`);
        for (let k = 0; k < HDLR_NOPS; k++) {
            L.push(`deposit -b ${hex(R_MCHK_HDLR + k)} ${OP_NOP.toString(16)}`);
            L.push(`deposit -b ${hex(R_MERR_HDLR + k)} ${OP_NOP.toString(16)}`);
        }
        if (c.ipc0 !== undefined && c.ipc0 !== null) L.push(`deposit QBA IPC ${hex(c.ipc0 & 0xFFFF, 4)}`);
        let bytes = [];
        for (let op of c.ops) bytes.push(...encodeOp(op));
        for (let k = 0; k < bytes.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${bytes[k].toString(16)}`);
        for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_CODE + bytes.length + k)} ${OP_NOP.toString(16)}`);
        L.push(`deposit PSL ${hex(c.psl || 0)}`, `deposit PC ${hex(R_CODE)}`,
               `step ${c.steps || c.ops.length}`);
        L.push(`examine -h ${Array.from({length: OBS_REGS}, (_, k) => "R" + k).join(",")}`);
        L.push("examine -h PC", "examine -h BTO");
        L.push("examine -h QBA IPC", "examine -h QBA DSER", "examine -h QBA MEAR");
    }
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "dbldiff-cases.ini"));

    let results = new Array(cases.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = +parts[i];
        let chunk = parts[i + 1] || "";
        let regs = [];
        for (let k = 0; k < OBS_REGS; k++) {
            let m = new RegExp(`^R${k}:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
            if (!m) { regs = null; break; }
            regs.push(parseInt(m[1], 16) >>> 0);
        }
        let g = (name) => new RegExp(`^${name}:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
        let pcm = g("PC"), btm = g("BTO"), ipm = g("IPC"), dsm = g("DSER"), mem = g("MEAR");
        if (!regs || !pcm || !btm || !ipm || !dsm || !mem) continue;
        results[idx] = {
            regs, pc: parseInt(pcm[1], 16) >>> 0, bto: parseInt(btm[1], 16) >>> 0,
            ipc: parseInt(ipm[1], 16) & IPC_OBS_MASK,
            dser: parseInt(dsm[1], 16) & DSER_OBS_MASK,
            mear: parseInt(mem[1], 16) & MEAR_OBS_MASK
        };
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction.  Every address and every value is COMPUTED from cqipc.js's own constants.  *
 * ------------------------------------------------------------------------------------------- */

/** The register's bits, taken from cqipc.js one at a time so the value sweep below is derived from
    the register definition rather than typed out (rule 5).  CQIPC_INV is in the list precisely
    because it is NOT in CQIPC_MASK: writing it and reading back zero is what proves the read mask. */
const IPC_BITS = [
    ["QME", CQIPC_QME], ["INV", CQIPC_INV], ["AHLT", CQIPC_AHLT],
    ["DBIE", CQIPC_DBIE], ["LME", CQIPC_LME], ["DB", CQIPC_DB]
];

/** Written values: zero, every single bit, the composite masks, and the two saturated words.  All
    derived; none transcribed. */
const WRITE_VALUES = [0, ...IPC_BITS.map((b) => b[1]), CQIPC_RW, CQIPC_W1C, CQIPC_MASK,
                      (~CQIPC_MASK) & 0xFFFF, 0xFF, 0xFFFF];

/** Preloaded cq_ipc values: zero, EVERY single bit, the full mask, and its complement.
    THE OUT-OF-MASK BITS ARE IN THIS LIST DELIBERATELY, and an earlier revision that filtered them
    out is what the `register-mask-widened` mutation SURVIVED against: no WRITE can ever put an
    out-of-mask bit into cq_ipc (the read/write step ANDs with CQIPC_RW), so the READ mask is
    unexercised unless the register is PRELOADED with a bit only the read mask can remove.  That is
    what `DEPOSIT QBA IPC` on the oracle -- and the matching store on this machine -- is for. */
const PRELOAD_VALUES = [0, ...IPC_BITS.map((b) => b[1]), CQIPC_MASK, (~CQIPC_MASK) & 0xFFFF];

/** Byte lanes inside the two-byte window, derived from its size. */
const LANES = Array.from({length: DBL_SIZE}, (_, i) => i);

/**
 * buildCases(nRandom, seed)
 *
 * @param {number} nRandom
 * @param {number} seed
 * @returns {{cases: Array, meta: Object}}
 */
function buildCases(nRandom, seed)
{
    let rnd = mulberry32(seed);
    let cases = [];
    let meta = {read: [], write: [], size: [], sme: [], memerr: [], reset: [], random: []};
    let add = (name, phase, ops, extra = {}) => {
        let idx = cases.length;
        cases.push({name, ops, ipc0: extra.ipc0, psl: extra.psl, steps: extra.steps});
        meta[phase].push(Object.assign({idx, name, ops}, extra));
        return idx;
    };

    /* ---------------- READ ----------------
       Every lane, every size, at both mounts, over every preload.  The reads that the oracle
       machine-checks (a Qbus longword) are cases here too -- their graded product is PC/BTO/DSER,
       not R0. */
    for (let mt of MOUNTS) {
        for (let v of PRELOAD_VALUES) {
            for (let lane of LANES) {
                add(`READ ${mt.name} rb lane ${lane} ipc=${hex(v, 4)}`, "read",
                    [{kind: "rb", addr: (mt.base + lane) >>> 0, reg: 0}], {ipc0: v, mount: mt.name, kind: "rb", lane});
            }
            add(`READ ${mt.name} rw ipc=${hex(v, 4)}`, "read",
                [{kind: "rw", addr: mt.base, reg: 0}], {ipc0: v, mount: mt.name, kind: "rw", lane: 0});
            add(`READ ${mt.name} rl ipc=${hex(v, 4)}`, "read",
                [{kind: "rl", addr: mt.base, reg: 0}], {ipc0: v, mount: mt.name, kind: "rl", lane: 0});
        }
    }

    /* ---------------- WRITE ----------------
       Every lane and every size at both mounts, over a preload matrix, each followed by a read of
       the OTHER mount's word.  The cross-mount readback is deliberate: it is the only observation
       that fails when the two mounts hold two copies of the register, and `EXAMINE QBA IPC` alone
       would not catch it because SIMH has only one copy to show. */
    for (let mt of MOUNTS) {
        let other = MOUNTS.find((x) => x !== mt);
        let readback = {kind: "rw", addr: other.base, reg: 1};
        for (let v of PRELOAD_VALUES) {
            for (let w of WRITE_VALUES) {
                for (let lane of LANES) {
                    add(`WRITE ${mt.name} wb lane ${lane} val=${hex(w & 0xFF, 2)} ipc=${hex(v, 4)}`, "write",
                        [{kind: "wb", addr: (mt.base + lane) >>> 0, val: w & 0xFF}, readback],
                        {ipc0: v, mount: mt.name, kind: "wb", lane, val: w & 0xFF});
                }
                add(`WRITE ${mt.name} ww val=${hex(w & 0xFFFF, 4)} ipc=${hex(v, 4)}`, "write",
                    [{kind: "ww", addr: mt.base, val: w & 0xFFFF}, readback],
                    {ipc0: v, mount: mt.name, kind: "ww", lane: 0, val: w & 0xFFFF});
                add(`WRITE ${mt.name} wl val=${hex(w, 8)} ipc=${hex(v, 4)}`, "write",
                    [{kind: "wl", addr: mt.base, val: w >>> 0}],
                    {ipc0: v, mount: mt.name, kind: "wl", lane: 0, val: w >>> 0, steps: 2});
            }
        }
    }

    /* ---------------- SIZE / NEIGHBOUR ----------------
       The addresses immediately around each window, at every size.  Their graded product is the
       CLASSIFICATION (answered / machine check / which machine check), which is what says the
       window is exactly DBL_SIZE bytes wide on both engines. */
    for (let mt of MOUNTS) {
        for (let off of [-4, -2, -1, DBL_SIZE, DBL_SIZE + 1, DBL_SIZE + 2, DBL_SIZE + 4]) {
            add(`SIZE ${mt.name} rb @${off >= 0 ? "+" : ""}${off}`, "size",
                [{kind: "rb", addr: (mt.base + off) >>> 0, reg: 0}], {ipc0: CQIPC_MASK, mount: mt.name, off});
        }
    }

    /* ---------------- SME ----------------
       cqbic_wr()'s DSER<SME> cross-clear of cq_ipc<QME>, both directions, at longword and at the
       byte lane the bit lives in.  DSER is CQBIC register REG_DSER; the address is computed. */
    let dser = (CQBIC_BASE + REG_DSER * 4) >>> 0;
    for (let [label, val] of [["SME", CQDSER_SME], ["noSME", 0], ["all-but-SME", (0xFF & ~CQDSER_SME)]]) {
        add(`SME wl DSER=${label}`, "sme",
            [{kind: "wl", addr: dser, val: val >>> 0}, {kind: "rw", addr: DBL_BASE, reg: 0}],
            {ipc0: CQIPC_MASK, sme: label, kind: "wl"});
        add(`SME wb DSER=${label}`, "sme",
            [{kind: "wb", addr: dser, val: val & 0xFF}, {kind: "rw", addr: DBL_BASE, reg: 0}],
            {ipc0: CQIPC_MASK, sme: label, kind: "wb"});
    }

    /* ---------------- MEMERR ----------------
       The Qbus longword write: its low word reaches the doorbell and its high word lands on nothing,
       which vax_io.c's WriteQb() reports as a DEFERRED mem_err rather than an exception.  Graded at
       IPL 0 (delivers on the next boundary, PC lands in the MEMERR handler) and at IPL 0x1F (masked;
       PC stays in the instruction stream), so the case proves delivery AND masking rather than just
       "something happened". */
    for (let [label, psl] of [["ipl00", 0], ["ipl1F", 0x001F0000]]) {
        add(`MEMERR IOP wl ${label}`, "memerr",
            [{kind: "wl", addr: DBL_BASE, val: 0xFFFFFFFF}],
            {ipc0: CQIPC_QME, psl, steps: 3, label});
    }

    /* ---------------- RESET ----------------
       The ONLY cases with no preload at all.  Both engines reset the device between cases -- SIMH's
       `reset -p all` runs qba_reset(), which zeroes cq_ipc -- so a read here must answer 0 on both.
       Every other phase deposits a value first, which makes reset() unobservable: this phase exists
       because the `reset-does-not-clear-the-register` mutation SURVIVED without it, carrying the
       previous case's value forward on this machine while the oracle started clean.  The write that
       seeds a non-zero value is a SEPARATE case on purpose -- the reset between the two is the
       subject, and putting both in one case would defeat it. */
    for (let mt of MOUNTS) {
        add(`RESET ${mt.name} seed a non-zero value`, "reset",
            [{kind: "ww", addr: mt.base, val: CQIPC_MASK}], {ipc0: CQIPC_MASK, mount: mt.name, seed: true});
        add(`RESET ${mt.name} rw with NO preload`, "reset",
            [{kind: "rw", addr: DBL_BASE, reg: 0}], {mount: mt.name, seed: false});
    }

    /* ---------------- RANDOM ----------------
       Uniform over mount, size, lane (including one byte past the window), value and preload -- a
       structurally different view from the enumerated matrix above, which is exhaustive at the
       boundaries and blind between them. */
    let kinds = ["rb", "rw", "rl", "wb", "ww", "wl"];
    for (let i = 0; i < nRandom; i++) {
        let mt = MOUNTS[Math.floor(rnd() * MOUNTS.length)];
        let kind = kinds[Math.floor(rnd() * kinds.length)];
        let lane = Math.floor(rnd() * (DBL_SIZE + 1));           // 0..DBL_SIZE, i.e. one past the end
        let addr = (mt.base + (kind[0] === "r" && kind !== "rb" ? 0 : lane)) >>> 0;
        let v = PRELOAD_VALUES[Math.floor(rnd() * PRELOAD_VALUES.length)];
        let w = (Math.floor(rnd() * 0x100000000) >>> 0);
        let ops = kind[0] === "r"
            ? [{kind, addr, reg: 0}]
            : [{kind, addr, val: kind === "wb" ? (w & 0xFF) : kind === "ww" ? (w & 0xFFFF) : w}];
        add(`RANDOM ${mt.name} ${kind} lane ${lane} ipc=${hex(v, 4)} val=${hex(w)}`, "random", ops,
            {ipc0: v, mount: mt.name, kind, lane, steps: 2});
    }

    return {cases, meta};
}

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

const OBSERVED = ["pc", "bto", "ipc", "dser", "mear"];

/**
 * grade(cases, simhResults, jsResults, failures)
 *
 * @returns {number} how many cases actually reached comparison
 */
function grade(cases, simhResults, jsResults, failures)
{
    let compared = 0;
    for (let i = 0; i < cases.length; i++) {
        let s = simhResults[i], j = jsResults[i];
        if (!s) {
            failures.push(`case ${i} "${cases[i].name}": the oracle produced no readable result -- ` +
                `not compared (reported by name rather than dropped)`);
            continue;
        }
        compared++;
        for (let k = 0; k < OBS_REGS; k++) {
            if (s.regs[k] !== j.regs[k]) {
                failures.push(`case ${i} "${cases[i].name}": R${k} = ${hex(j.regs[k])} here, ` +
                    `${hex(s.regs[k])} on the oracle`);
            }
        }
        for (let f of OBSERVED) {
            if (s[f] !== j[f]) {
                failures.push(`case ${i} "${cases[i].name}": ${f.toUpperCase()} = ${hex(j[f])} here, ` +
                    `${hex(s[f])} on the oracle`);
            }
        }
    }
    return compared;
}

/** How a probe came out, from the observables alone.  "mchk"/"memerr" are told apart by WHICH
    handler page the PC landed in, which is why this file installs two -- and by RANGE rather than
    by equality, because a case whose step budget outlives the dispatch goes on executing the
    handler's NOPs (HDLR_NOPS, declared with the physical layout at the top of this file).  PC
    itself is still compared EXACTLY between the engines by grade(); this coarser reading exists
    only for the coverage floors and PHASE W. */
function classify(r)
{
    if (!r) return null;
    let pc = r.pc >>> 0;
    if (pc >= R_MCHK_HDLR && pc < R_MCHK_HDLR + HDLR_NOPS) return r.bto ? "mchk+bto" : "mchk";
    if (pc >= R_MERR_HDLR && pc < R_MERR_HDLR + HDLR_NOPS) return "memerr";
    return "ok";
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE W -- the Qbus I/O-page beachhead                                                        *
 * ------------------------------------------------------------------------------------------- */

/**
 * windowProbes()
 *
 * The address set, derived three ways and never hand-listed (rule 5):
 *   - every byte of the doorbell window and of the two words on either side of it
 *   - tests/mchkdiff.js's OWN candidate pool for this range, imported
 *   - a stride sweep of the whole page, at a stride that is a divisor of neither the page size nor
 *     the window offset, so it cannot systematically miss or hit the window
 *
 * @returns {Array.<number>}
 */
function windowProbes()
{
    let s = new Set();
    for (let off = -4; off < DBL_SIZE + 4; off++) s.add((DBL_BASE + off) >>> 0);
    for (let a of IOPAGE_RANGE.addrs) s.add(a >>> 0);
    let base = VAX.PHYSMEM.IOPAGE_BASE >>> 0, len = VAX.PHYSMEM.IOPAGE_LENGTH;
    for (let off = 0; off < len; off += 322) s.add((base + off) >>> 0);
    return [...s].filter((a) => a >= base && a < base + len).sort((a, b) => a - b);
}

/**
 * buildWindowCases()
 *
 * One WORD read per probed address (word, because the Qbus is a word bus and a word read is the
 * access the ROM actually performs at the doorbell).  Byte reads at the window's own bytes are
 * already in PHASE READ; this phase is about the RANGE.
 */
function buildWindowCases()
{
    let addrs = windowProbes();
    return {
        addrs,
        cases: addrs.map((a) => ({
            name: `WINDOW rw @0x${hex(a)}`,
            ops: [{kind: (a & 1) ? "rb" : "rw", addr: a, reg: 0}],
            ipc0: CQIPC_MASK
        }))
    };
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors.  Every one FAILS the run; none scales with the case count (rule 4).          *
 * ------------------------------------------------------------------------------------------- */

/**
 * coverage(cases, meta, simhResults, jsResults, failures, acc)
 *
 * Counted from cases that ACTUALLY REACHED COMPARISON, never from the case list: a phase whose
 * results never arrived must not be able to certify its own coverage.
 */
function coverage(cases, meta, simhResults, jsResults, failures, acc)
{
    let reached = (idx) => !!simhResults[idx];

    /* Both mounts, every lane, every size, read AND written. */
    for (let mt of MOUNTS) {
        for (let kind of ["rb", "rw", "rl", "wb", "ww", "wl"]) {
            let any = [...meta.read, ...meta.write].some((e) => reached(e.idx) && e.mount === mt.name && e.kind === kind);
            if (!any) failures.push(`coverage: no graded case performed a "${kind}" at the ${mt.name} mount`);
        }
        for (let lane of LANES) {
            let r = meta.read.some((e) => reached(e.idx) && e.mount === mt.name && e.kind === "rb" && e.lane === lane);
            let w = meta.write.some((e) => reached(e.idx) && e.mount === mt.name && e.kind === "wb" && e.lane === lane);
            if (!r) failures.push(`coverage: the ${mt.name} mount's byte lane ${lane} was never READ by a graded case`);
            if (!w) failures.push(`coverage: the ${mt.name} mount's byte lane ${lane} was never WRITTEN by a graded case`);
        }
    }

    /* The register's SEMANTICS, read out of the ORACLE's own answers -- never predicted here.
       Each of these is a behaviour a wrong implementation would get wrong, and each must have been
       OBSERVED at least once or this file is grading a register whose interesting cases never ran. */
    for (let e of meta.write) {
        if (!reached(e.idx)) continue;
        let s = simhResults[e.idx];
        let before = (e.ipc0 >>> 0) & CQIPC_MASK, after = s.ipc >>> 0;
        if ((before & CQIPC_QME) && !(after & CQIPC_QME)) acc.qmeCleared = true;
        if ((before & CQIPC_QME) && (after & CQIPC_QME)) acc.qmeRetained = true;
        if (((e.val >>> 0) & CQIPC_RW & ~before) && ((after & CQIPC_RW) !== (before & CQIPC_RW))) acc.rwChanged = true;
        if (e.lane !== 0 && ((after & CQIPC_RW) === (before & CQIPC_RW)) && ((e.val >>> 0) & CQIPC_RW)) acc.gateBlocked = true;
        if (((e.val >>> 0) & ~CQIPC_MASK) && !(after & ~CQIPC_MASK)) acc.writeMaskDropped = true;
    }

    /* The READ mask, which is a DIFFERENT claim from the write mask and needs a different case: no
       write can put an out-of-mask bit into cq_ipc, so the only way to see CQIPC_MASK applied on the
       way OUT is to PRELOAD one and read it back as zero.  Read out of the oracle's own R0. */
    for (let e of meta.read) {
        if (!reached(e.idx) || e.kind === "rl") continue;
        let pre = (e.ipc0 >>> 0) & 0xFFFF;
        if (!(pre & ~CQIPC_MASK & 0xFFFF)) continue;
        if (!(simhResults[e.idx].regs[0] & ~CQIPC_MASK)) acc.readMaskDropped = true;
    }

    /* reset().  A graded read with NO preload must answer 0 on the oracle, and the case before it
       must have left a non-zero value behind. */
    let seeded = meta.reset.some((e) => reached(e.idx) && e.seed && (simhResults[e.idx].ipc >>> 0) !== 0);
    let cleared = meta.reset.some((e) => reached(e.idx) && e.seed === false &&
                                         (simhResults[e.idx].regs[0] >>> 0) === 0 &&
                                         (simhResults[e.idx].ipc >>> 0) === 0);
    if (!seeded) failures.push(`coverage: no RESET-phase case ever left cq_ipc non-zero on the oracle, ` +
        `so the reset that follows it is unobservable`);
    if (!cleared) failures.push(`coverage: no graded case read the register with NO preload and saw ` +
        `zero on the oracle -- reset() is untested`);
    if (!acc.qmeCleared) failures.push(`coverage: no graded write was ever observed CLEARING QME (the W1C bit)`);
    if (!acc.qmeRetained) failures.push(`coverage: no graded write was ever observed LEAVING QME set`);
    if (!acc.rwChanged) failures.push(`coverage: no graded write was ever observed CHANGING the read/write bits`);
    if (!acc.gateBlocked) failures.push(`coverage: no graded write above byte lane 0 was ever observed ` +
        `leaving the read/write bits ALONE -- the "(pa & 3) == 0" low-byte gate is unexercised`);
    if (!acc.writeMaskDropped) failures.push(`coverage: no graded write ever had an out-of-mask bit ` +
        `DROPPED on the way IN -- CQIPC_RW is unexercised`);
    if (!acc.readMaskDropped) failures.push(`coverage: no graded read ever had a PRELOADED ` +
        `out-of-mask bit dropped on the way OUT -- CQIPC_MASK (and CQIPC_INV's write-only-ness) is ` +
        `unexercised on the read side`);

    /* The two mounts must have been observed BEHAVING DIFFERENTLY on the oracle, or the whole
       "these are not the same device model" premise of this file is unobservable and it is grading
       one mount twice.  The longword read is the discriminator. */
    let iopRl = meta.read.find((e) => reached(e.idx) && e.mount === "IOP" && e.kind === "rl");
    let regRl = meta.read.find((e) => reached(e.idx) && e.mount === "REG" && e.kind === "rl");
    if (!iopRl || !regRl) {
        failures.push(`coverage: a longword read did not reach comparison at both mounts`);
    } else {
        let ci = classify(simhResults[iopRl.idx]), cr = classify(simhResults[regRl.idx]);
        if (ci !== "mchk") {
            failures.push(`coverage: the oracle's Qbus longword read classified as "${ci}", not a ` +
                `machine check without the bus-timeout bits -- the two mounts' difference is not ` +
                `observable and this file is grading nothing by it`);
        } else acc.iopLongMchk = true;
        if (cr !== "ok") {
            failures.push(`coverage: the oracle's local-register longword read classified as "${cr}", ` +
                `not a plain answer`);
        } else acc.regLongOk = true;
    }

    /* The deferred mem_err, both delivered and masked, read out of the oracle. */
    for (let e of meta.memerr) {
        if (!reached(e.idx)) continue;
        let c = classify(simhResults[e.idx]);
        if (e.label === "ipl00" && c === "memerr") acc.memErrDelivered = true;
        if (e.label === "ipl1F" && c === "ok") acc.memErrMasked = true;
    }
    if (!acc.memErrDelivered) failures.push(`coverage: the Qbus longword write's DEFERRED memory ` +
        `error was never observed being DELIVERED on the oracle`);
    if (!acc.memErrMasked) failures.push(`coverage: the Qbus longword write's deferred memory error ` +
        `was never observed being MASKED by IPL on the oracle`);

    /* The SME cross-clear, both directions, read out of the oracle. */
    for (let e of meta.sme) {
        if (!reached(e.idx)) continue;
        let after = simhResults[e.idx].ipc >>> 0;
        if (e.sme === "SME" && !(after & CQIPC_QME)) acc.smeCleared = true;
        if (e.sme !== "SME" && (after & CQIPC_QME)) acc.smeRetained = true;
    }
    if (!acc.smeCleared) failures.push(`coverage: writing DSER with SME set was never observed ` +
        `clearing QME on the oracle`);
    if (!acc.smeRetained) failures.push(`coverage: writing DSER WITHOUT SME was never observed ` +
        `leaving QME alone on the oracle`);

    /* The plumbing itself: at least one graded case must have used the last observable register,
       which here is the cross-mount readback's R1. */
    if (!meta.write.some((e) => reached(e.idx) && e.ops.length > 1)) {
        failures.push(`coverage: no graded case performed the cross-mount readback -- the ` +
            `"one register, two mounts" claim is untested`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11)         *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* The item's own named list, first. */
    "w1c-bits-treated-as-ordinary-read-write": () => {
        let w1c = CQIPCVAX.CQIPC_W1C, rw = CQIPCVAX.CQIPC_RW;
        CQIPCVAX.CQIPC_W1C = 0;
        CQIPCVAX.CQIPC_RW = (rw | CQIPC_QME) | 0;
        return () => { CQIPCVAX.CQIPC_W1C = w1c; CQIPCVAX.CQIPC_RW = rw; };
    },
    "low-byte-gate-dropped": () => {
        let prev = CQIPCVAX.LOW_BYTE_GATE;
        CQIPCVAX.LOW_BYTE_GATE = 0;             // `(pa & 3) & 0` is always 0, so the gate always fires
        return () => { CQIPCVAX.LOW_BYTE_GATE = prev; };
    },
    "read-write-mask-widened": () => {
        let prev = CQIPCVAX.CQIPC_RW;
        CQIPCVAX.CQIPC_RW = ~0;
        return () => { CQIPCVAX.CQIPC_RW = prev; };
    },
    "register-mask-widened": () => {
        let prev = CQIPCVAX.CQIPC_MASK;
        CQIPCVAX.CQIPC_MASK = ~0;               // CQIPC_INV stops being write-only
        return () => { CQIPCVAX.CQIPC_MASK = prev; };
    },
    "doorbell-decoded-at-the-wrong-offset": () => ({dblBaseDelta: DBL_SIZE}),

    /* Beyond the item's list, and each earned by something specific in cqipc.js's header. */
    "w1c-mask-widened": () => {
        let prev = CQIPCVAX.CQIPC_W1C;
        CQIPCVAX.CQIPC_W1C = ~0;
        return () => { CQIPCVAX.CQIPC_W1C = prev; };
    },
    "w1c-tests-the-unshifted-value": () => {
        /* Composed over the shipped wr(): run it, then redo the W1C step as if `nval` had never been
           shifted.  This is the defect that makes a BYTE write to the odd lane stop reaching QME --
           the only way a byte access can touch the high lane at all. */
        let orig = CQIPCVAX.prototype.wr;
        CQIPCVAX.prototype.wr = function(addr, val, lnt) {
            let before = this.ipc | 0;
            let r = orig.call(this, addr, val, lnt);
            /* Put back whatever the shipped W1C step cleared, then redo it against the
               RIGHT-JUSTIFIED value instead of the shifted one.  QME is not in CQIPC_RW, so the
               read/write step orig already ran neither set nor cleared it and this composes cleanly
               over the real code rather than replacing it. */
            let restored = (this.ipc | (before & CQIPCVAX.CQIPC_W1C)) | 0;
            this.ipc = (restored & ~(val & CQIPCVAX.CQIPC_W1C)) | 0;
            return r;
        };
        return () => { CQIPCVAX.prototype.wr = orig; };
    },
    "reset-does-not-clear-the-register": () => {
        let orig = CQIPCVAX.prototype.reset;
        CQIPCVAX.prototype.reset = function() { let keep = this.ipc | 0; orig.call(this); this.ipc = keep | 0; };
        return () => { CQIPCVAX.prototype.reset = orig; };
    },
    "qbus-longword-read-answers-instead-of-faulting": () => {
        let orig = DBLVAX.prototype.readLong;
        DBLVAX.prototype.readLong = function(addr) { orig.call(this, addr); return this.cqipc.rd(); };
        return () => { DBLVAX.prototype.readLong = orig; };
    },
    "qbus-byte-read-ignores-the-lane": () => {
        let orig = DBLVAX.prototype.readByte;
        DBLVAX.prototype.readByte = function(addr) { orig.call(this, addr); return this.cqipc.rd() & 0xFF; };
        return () => { DBLVAX.prototype.readByte = orig; };
    },
    "dser-sme-does-not-clear-qme": () => {
        let orig = CQBICVAX.prototype.writeReg;
        CQBICVAX.prototype.writeReg = function(rg, val, sval = val) {
            let keep = this.ipc ? (this.ipc.ipc | 0) : 0;
            let r = orig.call(this, rg, val, sval);
            if (this.ipc) this.ipc.ipc = keep | 0;
            return r;
        };
        return () => { CQBICVAX.prototype.writeReg = orig; };
    },
    "doorbell-window-widened-to-a-longword": () => ({dblSizeDelta: 2}),
    "the-two-mounts-have-separate-state": () => ({splitState: true})
};

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                        *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

/**
 * runPass(simh, opts, mutationOpts)
 *
 * One complete grading pass.  Returns the failure list; the caller decides whether an empty one is
 * the right answer (the normal run) or the wrong one (--selfcheck).
 */
function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [];
    let acc = {};

    /* ---- PHASE A ---- */
    let rows = opts.ioRows;
    let qba = rows.filter((r) => r.name === "QBA");
    if (qba.length !== 1) {
        failures.push(`PHASE A: \`SHOW QBA IOSPACE\` lists ${qba.length} QBA row(s); exactly one is ` +
            `the doorbell's autoconfigured window`);
    } else {
        let got = {base: qba[0].base, size: (qba[0].end - qba[0].base + 1) >>> 0};
        if (got.base !== (DBL_BASE >>> 0) || got.size !== DBL_SIZE) {
            failures.push(`PHASE A: the oracle autoconfigures the doorbell at 0x${hex(got.base)}` +
                `..+${got.size}, cqipc.js has 0x${hex(DBL_BASE)}..+${DBL_SIZE} -- the constant and ` +
                `the live autoconfiguration disagree`);
        }
        report.push(`  PHASE A  oracle autoconfigures QBA at 0x${hex(got.base)} for ${got.size} ` +
            `byte(s); cqipc.js agrees`);
    }
    let otherWindows = rows.filter((r) => r.name !== "QBA");
    report.push(`  PHASE A  ${otherWindows.length} other I/O-page device window(s) on the oracle, ` +
        `implemented by nothing in this tree: ${otherWindows.map((r) => r.name).join(", ") || "(none)"}`);

    /* ---- PHASES READ/WRITE/SIZE/SME/MEMERR/RANDOM ---- */
    let {cases, meta} = buildCases(opts.nRandom, opts.seed);
    let win = buildWindowCases();
    let all = cases.concat(win.cases);
    let simhResults = runCasesSimh(simh, opts, all);
    let jsResults = all.map((k) => runCaseJS(k, mutationOpts));

    let compared = grade(cases, simhResults.slice(0, cases.length), jsResults.slice(0, cases.length), failures);
    coverage(cases, meta, simhResults, jsResults, failures, acc);

    /* ---- PHASE W ---- */
    {
        let wS = simhResults.slice(cases.length), wJ = jsResults.slice(cases.length);
        let decodedHere = [], excluded = [], comparedW = 0;
        for (let i = 0; i < win.addrs.length; i++) {
            let a = win.addrs[i];
            let cj = classify(wJ[i]);
            if (cj === "ok") decodedHere.push(a);
            if (!wS[i]) {
                failures.push(`PHASE W: 0x${hex(a)} produced no readable oracle result -- not compared`);
                continue;
            }
            let owner = otherWindows.find((r) => a >= r.base && a <= r.end);
            if (owner) { excluded.push(`0x${hex(a)} (${owner.name})`); continue; }
            comparedW++;
            let cs = classify(wS[i]);
            if (cs !== cj) {
                failures.push(`PHASE W: 0x${hex(a)} classifies as "${cj}" here and "${cs}" on the oracle`);
            }
        }
        /* The beachhead assertion: the ONLY thing this machine decodes inside the I/O page is the
           doorbell window.  Stated over the probed set, and the probed set includes every byte of
           the window and of the words on either side of it. */
        let expected = [];
        for (let off = 0; off < DBL_SIZE; off++) {
            let a = (DBL_BASE + off) >>> 0;
            if (win.addrs.includes(a)) expected.push(a);
        }
        let ds = decodedHere.map((a) => hex(a)).sort().join(",");
        let es = expected.map((a) => hex(a)).sort().join(",");
        if (ds !== es) {
            failures.push(`PHASE W: this machine decodes {${ds || "(nothing)"}} inside the Qbus I/O ` +
                `page, expected exactly the doorbell window {${es}} -- every other I/O-page address ` +
                `must keep bus-faulting exactly as it did before pcjsvax-b8a`);
        }
        if (comparedW < WINDOW_PROBE_FLOOR) {
            failures.push(`PHASE W: only ${comparedW} I/O-page address(es) reached comparison, below ` +
                `the fixed floor of ${WINDOW_PROBE_FLOOR}`);
        }
        report.push(`  PHASE W  ${comparedW} of ${win.addrs.length} probed I/O-page address(es) ` +
            `classified identically on both engines; ${excluded.length} excluded BY NAME as backed ` +
            `by an oracle device this tree does not implement` +
            (excluded.length ? `: ${excluded.slice(0, 8).join(", ")}${excluded.length > 8 ? ", ..." : ""}` : ""));
        report.push(`  PHASE W  decoded inside the I/O page on this machine: ` +
            `${decodedHere.map((a) => "0x" + hex(a)).join(", ") || "(nothing)"}`);
    }

    /* The wiring the graded machine is actually holding, asserted rather than assumed. */
    if (!machine(mutationOpts).cqbic.hasIpc()) {
        failures.push(`the graded machine's CQBICVAX has no CQIPC wired, so cqbic_wr()'s DSER<SME> ` +
            `cross-clear cannot fire at all and the SME phase is grading a side effect that is not ` +
            `installed`);
    }

    report.push(`  GRADE    ${compared}/${cases.length} register case(s) compared`);
    return {failures, report};
}

/**
 * selfcheck(simh, opts)
 *
 * Every mutation must be CAUGHT.  The mutation is removed in a `finally` so one survivor cannot
 * poison the next.
 */
function selfcheck(simh, opts)
{
    let survived = [];
    for (let name of Object.keys(MUTATIONS)) {
        let apply = MUTATIONS[name]();
        let restore = typeof apply === "function" ? apply : () => {};
        let mutationOpts = typeof apply === "function" ? {} : apply;
        let failures;
        try {
            failures = runPass(simh, opts, mutationOpts).failures;
        } finally {
            restore();
        }
        if (!failures.length) survived.push(name);
        console.log(`  ${failures.length ? "CAUGHT " : "SURVIVED"}  ${name}` +
            (failures.length ? `  (${failures.length} failure(s), first: ${failures[0]})` : ""));
    }
    return survived;
}

function main()
{
    let simh = findSimhBin(getArg("--simh", null));
    let nRandom = +getArg("--cases", RANDOM_CASES_DEFAULT);
    let seed = +getArg("--seed", 20260727);
    let fSelfcheck = process.argv.includes("--selfcheck");

    if (nRandom < RANDOM_CASES_FLOOR) {
        console.error(`dbldiff: --cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "dbldiff-"));
    let code = 0;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized cases: ${nRandom}`);
        console.log(`doorbell: Qbus I/O page 0x${hex(DBL_BASE)}..+${DBL_SIZE}, local register ` +
            `0x${hex(CQIPC_BASE)}..+${CQIPC_SIZE}, mask 0x${hex(CQIPC_MASK, 4)}, ` +
            `W1C 0x${hex(CQIPC_W1C, 4)}, RW 0x${hex(CQIPC_RW, 4)}`);

        let opts = {scratch, nRandom, seed};
        opts.ioRows = ioSpace(simh, opts);

        let {failures, report} = runPass(simh, opts);
        console.log(`\nPHASES`);
        for (let line of report) console.log(line);

        let peakMB = PEAK_HEAP / (1024 * 1024);
        console.log(`\npeak JS heap+external: ${peakMB.toFixed(1)} MB (absolute ceiling ` +
            `${MAX_HEAP_BYTES / (1024 * 1024)} MB)`);
        if (PEAK_HEAP > MAX_HEAP_BYTES) {
            failures.push(`peak heap+external ${peakMB.toFixed(1)} MB exceeds the absolute ceiling ` +
                `${MAX_HEAP_BYTES / (1024 * 1024)} MB`);
        }

        if (failures.length) {
            console.error(`\nFAIL -- ${failures.length} difference(s) from the oracle (seed ${seed}):`);
            for (let f of failures.slice(0, 60)) console.error(`  ${f}`);
            if (failures.length > 60) console.error(`  ... and ${failures.length - 60} more`);
            code = 1;
        } else {
            console.log(`\nMATCH -- every graded doorbell access agrees with the oracle in value, in ` +
                `W1C and low-byte-gate side effects, in machine-check behaviour and SSC bus-timeout ` +
                `residue, in DSER/MEAR, and in deferred memory-error delivery -- at BOTH address paths.`);
        }

        if (fSelfcheck && !code) {
            console.log(`\nPHASE M -- mutations (${Object.keys(MUTATIONS).length})`);
            let survived = selfcheck(simh, opts);
            if (survived.length) {
                console.error(`\nFAIL -- ${survived.length} mutation(s) SURVIVED: ${survived.join(", ")}`);
                code = 1;
            } else {
                console.log(`\nall ${Object.keys(MUTATIONS).length} mutation(s) CAUGHT`);
            }
        }
        if (!code) console.log("\nOK");
    } finally {
        /* Every exit path, including a throw: HANDOFF.md's disk note (pcjsvax-bd1 exists because a
           differential leaked 10.4GB of scratch). */
        try { fs.rmSync(scratch, {recursive: true, force: true}); } catch (e) { /* best effort */ }
    }
    process.exit(code);
}

main();
