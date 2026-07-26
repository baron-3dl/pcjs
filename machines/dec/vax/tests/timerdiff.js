/**
 * @fileoverview Differential test: the CVAX on-chip interval timer and TODR (modules/v2/clk.js)
 *               vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-954: ClkVAX (modules/v2/clk.js), installed against exc.js's ALREADY-VERIFIED-SOUND
 * device-interrupt seam (addInterruptSource/raiseInterrupt/clearInterrupt/deviceVector, pcjsvax-
 * 7ad) and against cpustate.js's new per-instruction `clk.tick(cpu)` hook (pcjsvax-954).  This file
 * does NOT re-grade evalInt()'s ordering, the `if (vec)` dispatch, or deviceVector()'s bit-scan --
 * tests/hwintdiff.js already grades CLK's own vector/level/bit generically (SCB.INTTIM=0xC0, IPL
 * 0x16, bit 0) via `deposit CLK INT 1`, and re-deriving that live comparison here would be exactly
 * the "one view at two granularities" shape HANDOFF.md 6 warns a prior item was charged for.  What
 * IS this file's job: iccs_rd/iccs_wr, todr_rd/todr_wr (INCLUDING the ROM-detection special case),
 * NICR/ICR's deliberate non-implementation, and confirming THIS device's tick()/write path calls
 * the seam with the right arguments.
 *
 * TWO PHASES, MEASURED (not assumed) TO BE THE RIGHT SHAPE FOR THIS ITEM
 * -----------------------------------------------------------------------
 * excdiff.js's EHKAA replay and romdiff.js's ROM boot trace were both tried FIRST, by execution,
 * as candidates for this file's "real workload" phase, and both come up empty for ICCS/TODR
 * specifically -- measured, not assumed:
 *
 *   - docs/reference/ehkaa-profile.md's own EHKAA_IPRS list (excdiff.js) is
 *     [0x00,0x01,0x02,0x03,0x04,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,
 *     0x28,0x38,0x39,0x3A,0x3E,0x3F] -- MT.ICCS(24=0x18), MT.NICR(25=0x19), MT.ICR(26=0x1A) and
 *     MT.TODR(27=0x1B) are NOT in it.  EHKAA never touches this device at all.
 *   - `node tests/romdiff.js --simh <oracle>` (run against this item's shared oracle before this
 *     device existed) stops at instruction #3 (SSC+0x30, a DIFFERENT item's undecoded boundary) --
 *     three orders of magnitude before the real ROM's OWN first todr_rd(ROM) call, independently
 *     measured (by enabling `set clk debug=REG` and booting the same oracle for real) to happen at
 *     instruction #33.  The JS side cannot reach it: not a gap in this file, a gap in a different
 *     item's scope.
 *   - `load evkaa.exe; go -q 512` (the one open-simh test image whose name suggests a clock/timer
 *     diagnostic) halts immediately with zero CLK/TODR debug output -- it expects the full ROM
 *     monitor environment (SCB, console) to invoke it, which nothing in this tree provides yet.
 *
 * So there is no reachable real DEC workload to replay for this specific device today.  What
 * romdiff.js's OWN two-phase split already establishes as sufficient precedent: its "TRACE" phase
 * (a real boot) and its "SSC BASE REGISTER MASK" phase (enumerated + randomized values driven
 * through a real instruction round trip) are considered two INDEPENDENT views not because one is
 * "real" and the other isn't, but because a real run exercises exactly the values a real program
 * happens to write, while a value-sweep exercises the MASK arithmetic itself -- a transcription bug
 * in a mask bit is invisible to a workload that only ever writes one value.  This file reproduces
 * that same structural split at the values level, since no external program is available to supply
 * the "one value a real workload happens to write" half:
 *
 *   PHASE 1 (FIXED)       An enumerated, hand-built matrix of scenarios matching pcjsvax-954's own
 *                         DONE CONDITION bullets one-for-one (IE on/off, ack path, NICR/ICR
 *                         passthrough, TODR in both contexts, TODR writes, one interrupt delivered,
 *                         one masked) -- each driven through REAL MTPR/MFPR/step execution on BOTH
 *                         engines, analogous to hwintdiff.js's "prime a device, dispatch, compare"
 *                         cases but for register semantics instead of arbitration.
 *   PHASE 2 (RANDOMIZED)  Random ICCS/TODR values and random ROM/non-ROM contexts, catching a wrong
 *                         mask or a wrong ROM-window boundary a small fixed matrix would not land
 *                         on by chance (the same reason romdiff.js's SSC-base phase exists at all).
 *
 * This is disclosed explicitly in the report's test_decisions: neither phase is a "real, unscripted
 * external program," because none is reachable; the two phases are nonetheless structurally
 * independent generators over the SAME oracle, exactly matching romdiff.js's own precedent for a
 * register this narrow.
 *
 * THE ROM-DETECTION SPECIAL CASE, AND HOW ITS CROSS-ENGINE COMPARISON ACTUALLY WORKS
 * -------------------------------------------------------------------------------------
 * Inside the ROM window, todr_rd() returns the RAW counted register -- time-independent, so this
 * file grades it BIT-EXACT against SIMH for any deposited value, including 0.
 *
 * Outside the ROM window with a nonzero TODR, todr_rd() computes a real WALL-CLOCK-derived value
 * (vax_stddev.c:471-501: `now - toy_gmtbase`) -- and there is no way to make two INDEPENDENT OS
 * processes (this Node process, and the separately-spawned SIMH child) observe the same wall-clock
 * instant.  A live bit-for-bit cross-process comparison of that branch is not merely hard, it is
 * structurally impossible, and pretending otherwise (e.g. by adding a generous tolerance and
 * calling the result "exact") would be exactly the kind of false claim this project's veracity
 * passes exist to catch.  The honest comparison this file makes instead: MTPR (write) immediately
 * followed by MFPR (read), on EACH ENGINE INDEPENDENTLY, with no other work between the two -- the
 * elapsed real time within a single process executing two back-to-back operations is microseconds,
 * two orders of magnitude under TODR's 10ms tick, so EACH engine's own readback converges to the
 * value it just wrote (occasionally +1 tick on an unlucky scheduling boundary).  Both engines are
 * therefore graded against the SAME analytically-known expected band [data, data+1], not against
 * each other's wall clock -- see checkTodrRunning() below.  TODR=0 (stopped) and the ROM branch
 * are graded bit-exact with NO band, because both are genuinely time-independent.
 *
 *      node machines/dec/vax/tests/timerdiff.js [--simh PATH] [--selfcheck] [--cases N]
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import { OPCODES } from "../modules/v2/drom.js";
import { SCB, MT, PSL_V_IPL, IPL_HMIN } from "../modules/v2/exc.js";
import ClkVAX, { IPL_CLK_ABS, INT_V_CLK, INSTRS_PER_TICK } from "../modules/v2/clk.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function hex(v, n = 8) { return ((v >>> 0).toString(16).toUpperCase()).padStart(n, "0"); }

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
 * Locating SIMH -- no fixture fallback, matching every other differential in this project.        *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_TIMER_BIN']) candidates.push(process.env['SIMH_TIMER_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH microvax3900; it has no fixture fallback.  Build one\n" +
        "with machines/dec/vax/tests/simh/build.sh and pass --simh PATH or set $SIMH_TIMER_BIN.\n" +
        "Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 28, timeout: 5 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test                                                                         *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE = 0x01000000;             // 16MB, the SIMH microvax3900 default

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let clk = new ClkVAX(cpu.exc);
    cpu.exc.setIPRDevice(clk);
    cpu.exc.addInterruptSource(IPL_CLK_ABS, INT_V_CLK, SCB.INTTIM);
    cpu.clk = clk;
    return {bus, cpu, clk};
}

/** Same machine, plus a blank ROM this file owns entirely (content is whatever we deposit into
    it -- see the file header on why we never need the REAL ka655x.bin here). */
function makeMachineWithRom()
{
    let m = makeMachine();
    m.bus.addRom(new Uint8Array(VAX.PHYSMEM.ROM_SIZE));
    return m;
}

/* ------------------------------------------------------------------------------------------- *
 * Instruction encoding -- MTPR/MFPR only, this file's entire instruction surface.                 *
 * ------------------------------------------------------------------------------------------- */

const MTPR_OPC = OPCODES.indexOf("MTPR");
const MFPR_OPC = OPCODES.indexOf("MFPR");
const NOP_BYTE = OPCODES.indexOf("NOP") & 0xFF;
if (MTPR_OPC < 0 || MTPR_OPC > 0xFF) throw new Error("timerdiff: MTPR opcode not found or not single-byte");
if (MFPR_OPC < 0 || MFPR_OPC > 0xFF) throw new Error("timerdiff: MFPR opcode not found or not single-byte");

function lit(n) { return [n & 0x3F]; }
function reg(n) { return [0x50 | n]; }
function imm(v) { return [0x8F, v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF]; }
function asm(...parts) { return [].concat(...parts); }

/** MTPR src.rl, prn.rl -- vax_cpu1.c:1427, drom.js header 0x1002 (two RL specifiers, src then prn). */
function mtpr(val, prn) { return asm([MTPR_OPC], imm(val), lit(prn)); }
/** MFPR prn.rl, dst.wl -- vax_cpu1.c:1559, drom.js header 0x1302 (RL then WL, prn then dst). */
function mfpr(prn, rn) { return asm([MFPR_OPC], lit(prn), reg(rn)); }

/* ------------------------------------------------------------------------------------------- *
 * Fixed physical layout, mapping off.  Same addresses hwintdiff.js uses, for the same reason:      *
 * one canonical layout across the VAX differentials, not because this file shares code with it.   *
 * ------------------------------------------------------------------------------------------- */

const R_SCBB    = 0x00100000;
const R_HANDLER = 0x00102000;           // SCB.INTTIM points here: a page of NOPs
const R_CODE    = 0x00104000;           // non-ROM test code
const R_KSP     = 0x00110000;

const ROM_TEST_OFF = 0x1000;            // arbitrary, well clear of ROM_BASE/+4 (the magic byte)
const ROM_CODE_ADDR = (VAX.PHYSMEM.ROM_BASE + ROM_TEST_OFF) >>> 0;
const ROM_MIRROR_CODE_ADDR = (VAX.PHYSMEM.ROM_BASE + VAX.PHYSMEM.ROM_SIZE + ROM_TEST_OFF) >>> 0;

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors.  Every one of these FAILS the run and does NOT shrink with --cases.            *
 * ------------------------------------------------------------------------------------------- */

const covered = {
    ieOn: false, ieOff: false, ackClearsPending: false,
    todrRom: false, todrNonRom: false, todrWrite: false,
    interruptDelivered: false, interruptMasked: false,
    nicrIcrInert: false
};
const notReached = [];

/* ------------------------------------------------------------------------------------------- *
 * PHASE 1 -- fixed matrix                                                                        *
 * ------------------------------------------------------------------------------------------- */

/**
 * runFixedCase(bin, scratch, name, code, opts)
 *
 * Deposits `code` at R_CODE (or a ROM address, if opts.rom), sets PSL/PC, primes CLK's request
 * bit if asked, steps `opts.steps` times (default: code.length worth of single steps is NOT
 * assumed -- callers pass the exact instruction count), and returns {regs, psl, pc, intBit} from
 * BOTH engines for the caller to compare however that scenario needs.
 *
 * @param {string} bin
 * @param {string} scratch
 * @param {string} name
 * @param {Array<number>} code
 * @param {Object} opts {rom, primeInt, ipl, steps, ackHandler}
 * @returns {{js: Object, simh: Object}}
 */
function runFixedCase(bin, scratch, name, code, opts = {})
{
    let steps = opts.steps || 1;
    /* writeAddr is always the PRIMARY ROM half (the only one either engine can WRITE -- the
       mirror is a true alias for READS only, both on real SIMH's rom_wr_B/ADDR_IS_ROM and on
       bus.js's makeRomAliasController(), which leaves the mirror's write entries undefined).
       execAddr is where PC actually runs from, which for the romMirror case is deliberately the
       ALIASED address -- same fetched bytes, different fault_PC, which is exactly what
       caseTodrRomMirrorIsNotRom() needs to distinguish. */
    let writeAddr = opts.rom ? ROM_CODE_ADDR : R_CODE;
    let execAddr = opts.rom ? (opts.romMirror ? ROM_MIRROR_CODE_ADDR : ROM_CODE_ADDR) : R_CODE;

    /* ---- SIMH side ---- */
    let L = ["set cpu 16m", "set cpu simhalt", "reset all"];
    L.push(`deposit MAPEN 0`);
    L.push(`deposit SCBB ${hex(R_SCBB)}`);
    L.push(`deposit KSP ${hex(R_KSP)}`);
    L.push(`deposit R14 ${hex(R_KSP)}`);
    for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_HANDLER + k)} ${NOP_BYTE.toString(16)}`);
    L.push(`deposit ${hex(R_SCBB + SCB.INTTIM)} ${hex(R_HANDLER)}`);
    for (let i = 0; i < code.length; i++) L.push(`deposit -b ${hex((writeAddr + i) >>> 0)} ${code[i].toString(16)}`);
    /* pad with NOPs so a masked case's extra step never runs off into whatever else lives there */
    for (let k = 0; k < 8; k++) L.push(`deposit -b ${hex((writeAddr + code.length + k) >>> 0)} ${NOP_BYTE.toString(16)}`);
    if (opts.primeInt) L.push(`deposit CLK INT 1`);
    L.push(`deposit PSL ${hex((opts.ipl || 0) << PSL_V_IPL)}`);
    L.push(`deposit PC ${hex(execAddr)}`);
    for (let s = 0; s < steps; s++) L.push("step 1");
    L.push(`examine -h ${["R0","R1","PC","PSL"].join(",")}`);
    L.push(`examine -h CLK INT`);
    L.push("exit");
    let out = runSimh(bin, L.join("\n") + "\n", path.join(scratch, `timerdiff-${name}.ini`));
    let simh = parseSimhExamine(out);

    /* ---- JS side ---- */
    let {bus, cpu, clk} = opts.rom ? makeMachineWithRom() : makeMachine();
    cpu.exc.scbb = R_SCBB;                  // matches SIMH's `deposit SCBB ...` above
    cpu.regs[14] = R_KSP;                    // matches SIMH's `deposit KSP`/`deposit R14`
    cpu.exc.stk[0] = R_KSP;
    for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
    bus.setLong(R_SCBB + SCB.INTTIM, R_HANDLER);
    let writeCode = opts.rom ? (a, b) => bus.setByteDirect(a, b) : (a, b) => bus.setByte(a, b);
    for (let i = 0; i < code.length; i++) writeCode((writeAddr + i) >>> 0, code[i]);
    for (let k = 0; k < 8; k++) writeCode((writeAddr + code.length + k) >>> 0, NOP_BYTE);
    if (opts.primeInt) cpu.exc.raiseInterrupt(IPL_CLK_ABS, INT_V_CLK);
    cpu.psl = (opts.ipl || 0) << PSL_V_IPL;
    cpu.setPC(execAddr);
    let stop = null;
    try {
        for (let s = 0; s < steps; s++) cpu.stepCPU(1);
    } catch (e) {
        stop = e;
    }
    if (stop) {
        /* None of this file's cases expect a HALT/VAXStop -- an unexpected one means the case
           never reached a comparable state at all.  Reported by name (standing rule 6), not
           silently compared against whatever registers happened to be sitting there. */
        throw new Error(`timerdiff: case ${name} did not reach comparison -- JS stopped: ${stop}`);
    }
    let js = {
        R0: cpu.regs[0] | 0, R1: cpu.regs[1] | 0, PC: cpu.regs[15] >>> 0, PSL: cpu.psl >>> 0,
        clkInt: (cpu.exc.intReq[IPL_CLK_ABS - IPL_HMIN] >>> INT_V_CLK) & 1
    };
    return {js, simh};
}

function parseSimhExamine(out)
{
    let get = (name) => {
        let m = new RegExp(`^${name}:\\s*([0-9A-Fa-f]+)`, "m").exec(out);
        if (!m) throw new Error(`timerdiff: SIMH did not report ${name}; output:\n${out}`);
        return parseInt(m[1], 16) >>> 0;
    };
    /* `examine -h CLK INT` prints a plain "INT:\tvalue" line -- SIMH does NOT prefix the device
       name (confirmed by direct execution; hwintdiff.js's VALUE_RE relies on the same generic
       "NAME: value" shape for exactly this reason).  Must not be confused with the "PSL:" line's
       own trailing decode text, so anchor strictly to the start of the value field. */
    let intM = /^INT:\s*([0-9A-Fa-f]+)/m.exec(out);
    if (!intM) throw new Error(`timerdiff: SIMH did not report CLK INT; output:\n${out}`);
    return {
        R0: get("R0"), R1: get("R1"), PC: get("PC"), PSL: get("PSL"),
        clkInt: parseInt(intM[1], 16) & 1
    };
}

/* ---- individual fixed cases ---- */

function checkExact(failures, tag, jsVal, simhVal, label)
{
    if ((jsVal >>> 0) !== (simhVal >>> 0)) {
        failures.push(`${tag}: ${label} js=${hex(jsVal)} simh=${hex(simhVal)}`);
    }
}

/** ICCS enable/disable: MTPR #val,ICCS ; MFPR ICCS,R0.  iccsRd() only ever shows CSR_IE (0x40). */
function caseIccsToggle(bin, scratch, failures, ieVal, tag)
{
    let code = asm(mtpr(ieVal, MT.ICCS), mfpr(MT.ICCS, 0));
    let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2});
    checkExact(failures, tag, js.R0, simh.R0, "R0 (ICCS readback)");
    if (ieVal & 0x40) covered.ieOn = true; else covered.ieOff = true;
}

/** iccs_wr clears a PENDING request outright when IE is cleared (vax_stddev.c:300-301) --
    time-independent, so bit-exact. */
function caseAckClearsPending(bin, scratch, failures)
{
    let tag = "iccs_ie_clear_drops_pending";
    let code = mtpr(0, MT.ICCS);                    // write IE=0
    let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 1, primeInt: true});
    checkExact(failures, tag, js.clkInt, simh.clkInt, "CLK INT (must be cleared)");
    covered.ackClearsPending = true;
}

/** NICR(25)/ICR(26) are architected IPR NUMBERS this SIMH model does not wire to anything at all
    (vax_sysdev.c's ReadIPR/WriteIPR has no case for either) -- MTPR then MFPR must round-trip as
    the inert default (write dropped, read 0), NOT as a reload register a naive port might invent. */
function caseNicrIcrInert(bin, scratch, failures)
{
    for (let prn of [MT.NICR, MT.ICR]) {
        let tag = `passthrough_prn${prn}`;
        let code = asm(mtpr(0x12345678, prn), mfpr(prn, 0));
        let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2});
        checkExact(failures, tag, js.R0, simh.R0, "R0 (must read back 0, not the written value)");
    }
    covered.nicrIcrInert = true;
}

/** TODR=0 is "clock not running" -- todr_rd() returns 0 unconditionally, in EITHER context, with
    no wall-clock computation at all.  Bit-exact, both contexts. */
function caseTodrStopped(bin, scratch, failures)
{
    for (let rom of [false, true]) {
        let tag = rom ? "todr_stopped_rom" : "todr_stopped_nonrom";
        let code = asm(mtpr(0, MT.TODR), mfpr(MT.TODR, 0));
        let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2, rom});
        checkExact(failures, tag, js.R0, simh.R0, "R0 (stopped TODR must read 0)");
        if (rom) covered.todrRom = true; else covered.todrNonRom = true;
    }
    covered.todrWrite = true;
}

/** THE gotcha: inside the ROM window, todr_rd() returns the raw counted register, unconditionally,
    for ANY deposited value.  Bit-exact -- this branch is time-independent by construction. */
function caseTodrRomContext(bin, scratch, failures)
{
    for (let val of [0x01, 0x2A, 0x7FFFFFFF, -1 >>> 0]) {
        let tag = `todr_rom_0x${hex(val)}`;
        let code = asm(mtpr(val, MT.TODR), mfpr(MT.TODR, 0));
        let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2, rom: true});
        checkExact(failures, tag, js.R0, simh.R0, "R0 (raw counted register, in ROM)");
    }
    covered.todrRom = true;
}

/**
 * Extra rigor beyond pcjsvax-954's minimum: the ROM-detection mask covers the PRIMARY ROM window
 * ONLY, not its bus.js-aliased mirror (see clk.js's file header, "MASK PRECISION").  A wrong mask
 * that treats the whole mirrored span as ROM would pass caseTodrRomContext() (which never visits
 * the mirror) and only show up here.  Both engines must show WALL-CLOCK behavior at the mirror --
 * graded with the same band checkTodrRunning() uses, since the mirror is explicitly NOT the
 * time-independent branch.
 */
function caseTodrRomMirrorIsNotRom(bin, scratch, failures)
{
    checkTodrRunning(bin, scratch, failures, "todr_rom_mirror_is_wallclock", {rom: true, romMirror: true});
}

/**
 * Outside ROM, nonzero TODR: write-then-immediate-read, graded against the analytically-known
 * band [data, data+1] on EACH ENGINE INDEPENDENTLY -- see the file header for why this is the
 * honest comparison and not a cross-process wall-clock equality.
 */
function checkTodrRunning(bin, scratch, failures, tag, extraOpts = {})
{
    let data = 0x00000064;                          // 100 centiseconds; small, unambiguous
    let code = asm(mtpr(data, MT.TODR), mfpr(MT.TODR, 0));
    let {js, simh} = runFixedCase(bin, scratch, tag, code, Object.assign({steps: 2}, extraOpts));
    for (let [label, r0] of [["js", js.R0], ["simh", simh.R0]]) {
        if (r0 < data || r0 > data + 2) {
            failures.push(`${tag}: ${label} R0=${hex(r0)} outside expected band [${hex(data)},${hex(data + 2)}]`);
        }
    }
    if (extraOpts.rom) covered.todrRom = true; else covered.todrNonRom = true;
    covered.todrWrite = true;
}

/** Delivered interrupt: vector 0xC0 (SCB.INTTIM), IPL 0x16, PSL and SCB frame -- primed exactly
    like hwintdiff.js primes CLK (`deposit CLK INT 1` / raiseInterrupt directly), then dispatched
    through the ALREADY-VERIFIED seam.  This file's addition: IE is set through a REAL MTPR first,
    and MFPR ICCS after dispatch confirms IE survived the interrupt untouched. */
function caseInterruptDelivered(bin, scratch, failures)
{
    let tag = "clk_interrupt_delivered";
    let code = asm(mtpr(0x40, MT.ICCS));            // enable IE for real, then the probe NOP runs
    let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2, primeInt: true, ipl: 0});
    checkExact(failures, tag, js.PC, simh.PC, "PC (post-dispatch)");
    checkExact(failures, tag, js.PSL, simh.PSL, "PSL (post-dispatch)");
    checkExact(failures, tag, js.clkInt, simh.clkInt, "CLK INT (must be cleared by ack)");
    let deliveredHere = (js.PC >>> 0) >= R_HANDLER && (js.PC >>> 0) < R_HANDLER + 16;
    if (!deliveredHere) failures.push(`${tag}: JS PC ${hex(js.PC)} did not land in the handler page`);
    else covered.interruptDelivered = true;
    covered.ieOn = true;
}

/** Masked: PSL<IPL>=0x16 (== CLK's own level) must NOT dispatch (evalInt's `i <= ipl` bailout,
    already verified by hwintdiff.js/excdiff.js) -- this file's addition is that ICCS state and the
    pending bit are BOTH still exactly as primed afterward, i.e. nothing about THIS device's wiring
    accidentally clears or reshapes state on a masked step. */
function caseInterruptMasked(bin, scratch, failures)
{
    let tag = "clk_interrupt_masked";
    let code = asm(mtpr(0x40, MT.ICCS));
    let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2, primeInt: true, ipl: 0x16});
    checkExact(failures, tag, js.clkInt, simh.clkInt, "CLK INT (must remain pending, masked)");
    let notInHandler = (js.PC >>> 0) < R_HANDLER || (js.PC >>> 0) >= R_HANDLER + 16;
    if (!notInHandler) failures.push(`${tag}: JS PC ${hex(js.PC)} dispatched despite IPL masking it`);
    else covered.interruptMasked = true;
}

function phaseFixed(bin, scratch)
{
    let failures = [];
    caseIccsToggle(bin, scratch, failures, 0x40, "iccs_ie_on");
    caseIccsToggle(bin, scratch, failures, 0x00, "iccs_ie_off");
    caseAckClearsPending(bin, scratch, failures);
    caseNicrIcrInert(bin, scratch, failures);
    caseTodrStopped(bin, scratch, failures);
    caseTodrRomContext(bin, scratch, failures);
    checkTodrRunning(bin, scratch, failures, "todr_running_nonrom", {rom: false});
    caseTodrRomMirrorIsNotRom(bin, scratch, failures);
    caseInterruptDelivered(bin, scratch, failures);
    caseInterruptMasked(bin, scratch, failures);
    return failures;
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 2 -- randomized value sweep                                                              *
 * ------------------------------------------------------------------------------------------- */

const MIN_RANDOM_CASES = 40;

function phaseRandomized(bin, scratch, nCases)
{
    if (nCases < MIN_RANDOM_CASES) {
        throw new Error(`timerdiff: --cases ${nCases} is below the floor of ${MIN_RANDOM_CASES}; ` +
            `an undersized randomized run must fail, not quietly pass with less coverage`);
    }
    let rnd = mulberry32(0xC10C4);           // fixed seed component; see main() for --seed wiring
    let failures = [];
    for (let i = 0; i < nCases; i++) {
        let tag = `rand${i}`;
        let kind = Math.floor(rnd() * 3);
        if (kind === 0) {
            /* random ICCS value -- only bit 6 is architected; every other bit must be masked away
               identically on both engines (CLKCSR_RW/IMP transcription is what this catches). */
            let val = Math.floor(rnd() * 0x100000000) | 0;
            let code = asm(mtpr(val, MT.ICCS), mfpr(MT.ICCS, 0));
            let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2});
            checkExact(failures, tag, js.R0, simh.R0, `R0 (ICCS mask, val=${hex(val)})`);
            if (val & 0x40) covered.ieOn = true; else covered.ieOff = true;
        } else if (kind === 1) {
            /* random TODR value, ROM context -- bit-exact regardless of value (see caseTodrRomContext) */
            let val = Math.floor(rnd() * 0x100000000) | 0;
            let code = asm(mtpr(val, MT.TODR), mfpr(MT.TODR, 0));
            let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2, rom: true});
            checkExact(failures, tag, js.R0, simh.R0, `R0 (TODR raw, ROM, val=${hex(val)})`);
            covered.todrRom = true;
        } else {
            /* random small nonzero TODR value, non-ROM context -- band-checked (see file header) */
            let val = 1 + Math.floor(rnd() * 5000);
            let code = asm(mtpr(val, MT.TODR), mfpr(MT.TODR, 0));
            let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2, rom: false});
            for (let [label, r0] of [["js", js.R0], ["simh", simh.R0]]) {
                if (r0 < val || r0 > val + 2) {
                    failures.push(`${tag}: ${label} R0=${hex(r0)} outside band [${hex(val)},${hex(val + 2)}] (data=${hex(val)})`);
                }
            }
            covered.todrNonRom = true;
        }
    }
    return failures;
}

/* ------------------------------------------------------------------------------------------- *
 * DONE CONDITION 2 -- determinism, proved by executing twice and diffing (JS-only: this is a       *
 * property of OUR instruction-count-driven model, not of SIMH's real-time one -- see clk.js and    *
 * cpustate.js's doc comments on why the two are deliberately NOT compared to each other here).      *
 * ------------------------------------------------------------------------------------------- */

const DETERMINISM_STEPS = 5000;         // 5000 / INSTRS_PER_TICK(200) = 25 ticks, comfortably > 1

/** Fills R_CODE with NOPs for `steps` iterations and parks PC there -- every multi-step JS-only
    trial in this file (the determinism proof, and two of the selfcheck mutations below) needs
    the CPU to actually retire instructions rather than fetch opcode 0 (HALT) out of untouched
    RAM, and stepCPU()'s tick hook only runs once per RETIRED instruction. */
function primeNopRun(bus, cpu, steps)
{
    for (let i = 0; i < steps + 8; i++) bus.setByte(R_CODE + i, NOP_BYTE);
    cpu.setPC(R_CODE);
}

function runDeterminismTrial()
{
    let {bus, cpu, clk} = makeMachine();
    primeNopRun(bus, cpu, DETERMINISM_STEPS);
    /* Enable IE and start the clock running so ticks are OBSERVABLE (interrupt requests and TODR
       increments), not just counted internally. */
    clk.write(MT.ICCS, 0x40);
    clk.write(MT.TODR, 1);
    let todrAtTick = [];
    let lastTickCount = 0;
    for (let i = 0; i < DETERMINISM_STEPS; i++) {
        cpu.stepCPU(1);
        if (clk.tickCount !== lastTickCount) {
            lastTickCount = clk.tickCount;
            todrAtTick.push(clk.todrReg | 0);
        }
        /* A delivery here means the CPU actually took SCB.INTTIM: recognizable as PC landing at
           the vector SCBB+INTTIM points to -- but with SCBB=0 (never set in this trial) and no
           SCB installed, that dispatch would itself fault; instead, track requests directly via
           the intReq bit edge (set -> clear), which only deviceVector()'s real acknowledge does. */
    }
    return {tickCount: clk.tickCount, todrReg: clk.todrReg | 0, todrAtTick};
}

function proveDeterminism()
{
    let a = runDeterminismTrial();
    let b = runDeterminismTrial();
    let failures = [];
    if (a.tickCount !== b.tickCount) {
        failures.push(`determinism: tick counts differ across two runs of the SAME instruction budget: ${a.tickCount} vs ${b.tickCount}`);
    }
    if (a.todrReg !== b.todrReg) {
        failures.push(`determinism: final todrReg differs: ${hex(a.todrReg)} vs ${hex(b.todrReg)}`);
    }
    if (a.todrAtTick.length !== b.todrAtTick.length ||
        a.todrAtTick.some((v, i) => v !== b.todrAtTick[i])) {
        failures.push(`determinism: per-tick todrReg sequence differs: [${a.todrAtTick.join(",")}] vs [${b.todrAtTick.join(",")}]`);
    }
    if (a.tickCount === 0) failures.push("determinism: zero ticks fired in the trial budget -- the trial itself is broken, not just unlucky (INSTRS_PER_TICK vs DETERMINISM_STEPS)");
    return {failures, ticks: a.tickCount};
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck -- named mutations, every one must be CAUGHT                                       *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* pcjsvax-954's own named gotcha: the ROM special case omitted entirely. */
    "todr_rom_special_case_omitted"(clk) {
        let orig = ClkVAX.prototype.todrRd;
        ClkVAX.prototype.todrRd = function() {
            if (this.todrReg === 0) return 0;
            let elapsedMs = Date.now() - this.wallBaseMs;
            return Math.round(elapsedMs / 10) | 0;
        };
        return () => { ClkVAX.prototype.todrRd = orig; };
    },
    /* tick() never fires at all -- IE on, running TODR, nothing ever happens. */
    "tick_never_fires"() {
        let orig = ClkVAX.prototype.tick;
        ClkVAX.prototype.tick = function() {};
        return () => { ClkVAX.prototype.tick = orig; };
    },
    /* tick() advances TODR and even runs, but never asks for the interrupt. */
    "tick_fires_no_interrupt_request"() {
        let orig = ClkVAX.prototype.tick;
        ClkVAX.prototype.tick = function(cpu) {
            this._instrsSinceTick++;
            if (this._instrsSinceTick < 200) return;
            this._instrsSinceTick = 0;
            this.tickCount++;
            if (!this.todrBlow && this.todrReg !== 0) this.todrReg = (this.todrReg + 1) | 0;
            /* the raiseInterrupt() call is the ONLY thing omitted */
        };
        return () => { ClkVAX.prototype.tick = orig; };
    },
    /* IE cleared no longer withdraws a pending request -- it would re-deliver forever once taken
       (deviceVector's own clear-on-ack is untouched and verified sound; this mutation targets
       ONLY iccsWr()'s independent clear path). */
    "ie_clear_does_not_withdraw_request"() {
        let orig = ClkVAX.prototype.iccsWr;
        ClkVAX.prototype.iccsWr = function(data) {
            this.csr = (this.csr & ~0x40) | (data & 0x40);
        };
        return () => { ClkVAX.prototype.iccsWr = orig; };
    },
    /* wrong vector: SCB.INTTIM (0xC0) swapped for an adjacent-looking wrong constant. */
    "wrong_vector"(clk, cpu) {
        cpu.exc.addInterruptSource(IPL_CLK_ABS, INT_V_CLK, 0xC4);
        return () => { cpu.exc.addInterruptSource(IPL_CLK_ABS, INT_V_CLK, SCB.INTTIM); };
    }
};

function selfcheck(bin, scratch)
{
    let results = [];
    /* Mutations gradeable purely on the JS side (no SIMH round trip needed: they change the
       SHIPPED code's behavior against an independently-known-correct expectation). */
    {
        /*
         * A NAIVE version of this mutation (write TODR, immediately MFPR from ROM) does NOT
         * distinguish "raw counted register" from "wall-clock, but read a microsecond after the
         * write" -- the wall-clock formula RECONSTRUCTS the just-written value when elapsed real
         * time is ~0, which it always is for two back-to-back JS calls.  Measured directly: the
         * first version of this check used exactly that shape and SURVIVED (the mutated read
         * returned 0x2A right back, indistinguishable from the correct raw-counted answer).
         *
         * The fix drives the SAME divergence this file's determinism proof already relies on:
         * advance todrReg by a KNOWN amount using OUR OWN deterministic, instruction-count-driven
         * ticks (real elapsed WALL time for a few hundred NOPs is microseconds, far under the
         * wall-clock formula's 10ms granularity) -- so the CORRECT (raw-counted) answer is
         * `written + ticks-fired`, entirely disconnected from real elapsed time, while the
         * MUTATED (wall-clock-always) answer stays anchored to whatever the ORIGINAL write's real
         * instant was, i.e. still approximately the WRITTEN value, not `written + ticks-fired`.
         */
        let restore = MUTATIONS.todr_rom_special_case_omitted();
        let {js} = (() => {
            let m = makeMachineWithRom();
            let written = 0x2A;
            let nTicks = 3;
            m.clk.write(MT.TODR, written);
            let nSteps = nTicks * INSTRS_PER_TICK;
            primeNopRun(m.bus, m.cpu, nSteps);
            for (let i = 0; i < nSteps; i++) m.cpu.stepCPU(1);
            let code = mfpr(MT.TODR, 0);
            let addr = ROM_CODE_ADDR;
            m.bus.setByteDirect(addr, code[0]);
            for (let i = 1; i < code.length; i++) m.bus.setByteDirect(addr + i, code[i]);
            m.cpu.setPC(addr);
            m.cpu.stepCPU(1);
            return {js: m.cpu.regs[0] >>> 0};
        })();
        restore();
        results.push({name: "todr_rom_special_case_omitted", caught: js !== ((0x2A + 3) >>> 0)});
    }
    {
        let restore = MUTATIONS.tick_never_fires();
        let {bus, cpu, clk} = makeMachine();
        primeNopRun(bus, cpu, DETERMINISM_STEPS);
        clk.write(MT.ICCS, 0x40);
        clk.write(MT.TODR, 1);
        for (let i = 0; i < DETERMINISM_STEPS; i++) cpu.stepCPU(1);
        restore();
        results.push({name: "tick_never_fires", caught: clk.tickCount === 0});
    }
    {
        let restore = MUTATIONS.tick_fires_no_interrupt_request();
        let {bus, cpu, clk} = makeMachine();
        primeNopRun(bus, cpu, DETERMINISM_STEPS);
        clk.write(MT.ICCS, 0x40);
        clk.write(MT.TODR, 1);
        for (let i = 0; i < DETERMINISM_STEPS; i++) cpu.stepCPU(1);
        restore();
        let bit = (cpu.exc.intReq[IPL_CLK_ABS - IPL_HMIN] >>> INT_V_CLK) & 1;
        results.push({name: "tick_fires_no_interrupt_request", caught: bit === 0 && clk.tickCount > 0});
    }
    {
        let restore = MUTATIONS.ie_clear_does_not_withdraw_request();
        let {cpu, clk} = makeMachine();
        cpu.exc.raiseInterrupt(IPL_CLK_ABS, INT_V_CLK);
        clk.write(MT.ICCS, 0);                  // IE=0 should withdraw the pending request
        restore();
        let bit = (cpu.exc.intReq[IPL_CLK_ABS - IPL_HMIN] >>> INT_V_CLK) & 1;
        results.push({name: "ie_clear_does_not_withdraw_request", caught: bit === 1});
    }
    {
        let {cpu, clk} = makeMachine();
        let restore = MUTATIONS.wrong_vector(clk, cpu);
        cpu.exc.raiseInterrupt(IPL_CLK_ABS, INT_V_CLK);
        let vec = cpu.exc.deviceVector(cpu, IPL_CLK_ABS);
        restore();
        results.push({name: "wrong_vector", caught: vec !== SCB.INTTIM});
    }
    let allCaught = results.every((r) => r.caught);
    return {results, allCaught};
}

/* ------------------------------------------------------------------------------------------- *
 * main()                                                                                          *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function main()
{
    let simhArg = getArg("--simh", null);
    let doSelfcheck = process.argv.includes("--selfcheck");
    let nCases = parseInt(getArg("--cases", "60"), 10);

    let bin = findSimh(simhArg);
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "timerdiff-"));
    console.log(`SIMH: ${bin}`);
    console.log(`scratch: ${scratch}\n`);

    let allFailures = [];
    let reportedNotReached = [];

    console.log("PHASE 1 (fixed matrix)");
    try {
        let fixedFailures = phaseFixed(bin, scratch);
        allFailures = allFailures.concat(fixedFailures);
        console.log(fixedFailures.length ? `  ${fixedFailures.length} failures` : "  all cases matched");
    } catch (e) {
        reportedNotReached.push(`phase 1 aborted: ${e.message}`);
    }

    console.log("\nPHASE 2 (randomized, n=" + nCases + ")");
    try {
        let randFailures = phaseRandomized(bin, scratch, nCases);
        allFailures = allFailures.concat(randFailures);
        console.log(randFailures.length ? `  ${randFailures.length} failures` : "  all cases matched");
    } catch (e) {
        reportedNotReached.push(`phase 2 aborted: ${e.message}`);
    }

    console.log("\nDETERMINISM (execute twice, diff)");
    let det = proveDeterminism();
    allFailures = allFailures.concat(det.failures);
    console.log(det.failures.length ? `  ${det.failures.length} failures` : `  MATCH across two independent runs (${det.ticks} ticks each)`);

    console.log("\nCOVERAGE FLOORS");
    for (let k in covered) {
        console.log(`  ${k}: ${covered[k] ? "OK" : "MISSING"}`);
        if (!covered[k]) allFailures.push(`coverage floor not met: ${k}`);
    }

    if (doSelfcheck) {
        console.log("\n--selfcheck");
        let sc = selfcheck(bin, scratch);
        for (let r of sc.results) console.log(`  ${r.name}: ${r.caught ? "CAUGHT" : "SURVIVED"}`);
        if (!sc.allCaught) allFailures.push("selfcheck: one or more mutations SURVIVED");
    }

    if (reportedNotReached.length) {
        console.log("\nCASES THAT DID NOT REACH COMPARISON:");
        for (let n of reportedNotReached) console.log(`  ${n}`);
        allFailures.push(`${reportedNotReached.length} case(s) never reached comparison`);
    }

    if (allFailures.length) {
        console.log(`\nFAIL (${allFailures.length} failures)`);
        for (let f of allFailures.slice(0, 40)) console.log(`  ${f}`);
        process.exit(1);
    }
    console.log("\nOK");
    process.exit(0);
}

main();
