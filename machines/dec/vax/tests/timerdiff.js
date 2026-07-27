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
 *                         one masked, power-on state, wall-clock RATE) -- each driven through REAL
 *                         MTPR/MFPR/step execution on BOTH engines, analogous to hwintdiff.js's
 *                         "prime a device, dispatch, compare" cases but for register semantics.
 *   PHASE 2 (RANDOMIZED)  Random ICCS/TODR values and random ROM/non-ROM contexts, catching a wrong
 *                         mask or a wrong ROM-window boundary a small fixed matrix would not land
 *                         on by chance (the same reason romdiff.js's SSC-base phase exists at all).
 *
 * This exemption is disclosed, not concealed, in the report's test_decisions -- and it EXPIRES the
 * moment it stops being true: romdiff.js's own boundary sits at instruction #3 today (a DIFFERENT
 * item's undecoded SSC register), but the ROM's own first bare TODR read is only 30 instructions
 * further in (measured live: DBG(33), `MFPR #1B,34(R1)` at PC=0x200401E9).  The instant a later
 * item (pcjsvax-bfb/622/69a or similar) pushes that boundary past #33, romdiff.js starts replaying
 * a REAL todr_rd(ROM) call this file's grading must then agree with -- do not manufacture a
 * workload here in the meantime; wire the replay when the boundary actually reaches it.
 *
 * THE ROM-DETECTION SPECIAL CASE, AND HOW ITS CROSS-ENGINE COMPARISON ACTUALLY WORKS
 * -------------------------------------------------------------------------------------
 * MEASURED CORRECTION (veracity re-dispatch): an earlier version of this file graded the ROM
 * branch ONLY via "MTPR (write) immediately followed by MFPR (read)" and argued no live bit-exact
 * comparison was possible because two OS processes cannot share a wall-clock instant.  That
 * argument is correct about the WALL-CLOCK branch but was wrongly applied to the ROM branch too:
 * SIMH exposes todr_reg as a plain settable device register (`deposit CLK TODR n`, vax_stddev.c
 * REG table, DRDATAD TODR) that does NOT touch the wall-clock anchor (toy_gmtbase) -- so poking the
 * SAME raw value into both engines' counted register (`deposit CLK TODR n` / `clk.todrReg = n`,
 * bypassing MTPR/todrWr on BOTH sides) and reading back via a real MFPR from a ROM-window PC is a
 * BIT-EXACT, time-independent, live cross-engine comparison, strictly stronger than the
 * write-then-immediate-read band the earlier version relied on exclusively -- see
 * caseRomVsNonRomRawDiscriminator() below, which is now what actually distinguishes "returns the
 * raw counted register" from "always computes wall-clock" cross-engine (the earlier version's
 * write-then-immediate-read could not: reading back near-instantly after a WRITE reconstructs
 * approximately the SAME value via EITHER branch, which is why a "ROM special case omitted"
 * mutation survived that shape and had to be caught a different way -- see MUTATIONS below).
 *
 * Outside the ROM window with a nonzero TODR, todr_rd() computes a real WALL-CLOCK-derived value
 * (vax_stddev.c:471-501: `now - toy_gmtbase`) -- and there IS no way to make two INDEPENDENT OS
 * processes (this Node process, and the separately-spawned SIMH child) observe the same wall-clock
 * instant, so a live bit-for-bit comparison of THIS branch's absolute value remains structurally
 * impossible, and this file does not claim otherwise.  What it grades instead, on EACH ENGINE
 * INDEPENDENTLY:
 *
 *   - ROUND-TRIP: MTPR (write) immediately followed by MFPR (read), no other work between the two
 *     -- elapsed real time within a single process executing two back-to-back operations is
 *     microseconds, two orders of magnitude under TODR's 10ms tick, so each engine's own readback
 *     converges to the value it just wrote (occasionally +1/+2 ticks on an unlucky scheduling
 *     boundary).  Graded against the analytically-known band [data, data+2] -- see
 *     checkTodrRunning().  This proves the read is the INVERSE of the write; it does NOT, by
 *     itself, prove the RATE the formula advances at (a formula returning milliseconds instead of
 *     centiseconds -- a straight 10x scale bug -- is invisible when the elapsed real gap is itself
 *     under a millisecond either way).
 *   - RATE: MTPR (write), a REAL, SIMH-native `sleep <seconds>` (confirmed by direct execution to
 *     be a genuine blocking wait -- SIMH's event queue and todr_reg do NOT advance during it,
 *     since nothing is stepped) matched by an equal-duration host busy-wait on the JS side, then
 *     MFPR (read).  The expected delta is computed from the ACTUALLY MEASURED elapsed
 *     milliseconds on each engine independently (Date.now() before/after on the JS side; the
 *     `sleep` argument on the SIMH side), not assumed to be exact -- see caseTodrRate().  A 10x (or
 *     100x) scale bug is off by two-to-four orders of magnitude from ANY reasonable scheduling-
 *     jitter tolerance, so this is a low-precision, high-margin check, not a timing race.
 *
 * TODR=0 (stopped) and the ROM branch (via the raw-register discriminator above) are graded
 * BIT-EXACT with NO band, because both are genuinely time-independent.
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
import VAXExc, { SCB, MT, MT_MAX, PSL_V_IPL, IPL_HMIN, SSCBTO_BTO, SSCBTO_RWT } from "../modules/v2/exc.js";
import ClkVAX, { IPL_CLK_ABS, INT_V_CLK, INSTRS_PER_TICK, ROM_MASK, ROM_TAG, TOY_MAX_SECS, dayOfYear } from "../modules/v2/clk.js";

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
const ROM_BASE_CONST = VAX.PHYSMEM.ROM_BASE >>> 0;
const ROM_SIZE_CONST = VAX.PHYSMEM.ROM_SIZE >>> 0;
const ROM_CODE_ADDR = (ROM_BASE_CONST + ROM_TEST_OFF) >>> 0;
const ROM_MIRROR_CODE_ADDR = (ROM_BASE_CONST + ROM_SIZE_CONST + ROM_TEST_OFF) >>> 0;

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
    if (opts.rawTodr !== undefined) {
        /* A RAW poke of the counted register itself (vax_stddev.c's plain DRDATAD TODR entry),
           NOT a WriteIPR/todr_wr() -- this deliberately does NOT touch the wall-clock anchor
           (toy_gmtbase), which is exactly what makes a subsequent MFPR from ROM vs non-ROM PC a
           genuine, live, BIT-EXACT (in the ROM case) discriminator between the two branches --
           see caseRomVsNonRomRawDiscriminator() and the file header. */
        /* CLK's TODR register is DRDATAD (vax_stddev.c REG table) -- DECIMAL radix for deposit,
           unlike CSR's HRDATAD (hex).  Measured directly: `deposit CLK TODR 2A` -> "Invalid
           argument"; `deposit CLK TODR 42` (decimal) -> readback 0x2A via `examine -h`.  `-h` on
           examine forces hex DISPLAY regardless of the register's native deposit radix. */
        L.push(`deposit CLK TODR ${(opts.rawTodr >>> 0)}`);
    }
    for (let i = 0; i < code.length; i++) L.push(`deposit -b ${hex((writeAddr + i) >>> 0)} ${code[i].toString(16)}`);
    /* pad with NOPs so a masked case's extra step never runs off into whatever else lives there */
    for (let k = 0; k < 8; k++) L.push(`deposit -b ${hex((writeAddr + code.length + k) >>> 0)} ${NOP_BYTE.toString(16)}`);
    if (opts.primeInt) L.push(`deposit CLK INT 1`);
    L.push(`deposit PSL ${hex((opts.ipl || 0) << PSL_V_IPL)}`);
    L.push(`deposit PC ${hex(execAddr)}`);
    for (let s = 0; s < steps; s++) L.push("step 1");
    L.push(`examine -h ${["R0","R1","PC","PSL"].join(",")}`);
    L.push(`examine -h CLK INT`);
    /* SSC bus-timeout: pcjsvax-b4b.  A fresh SIMH PROCESS per case (runSimh spawns one every call,
       unlike mchkdiff.js's multi-case-per-process script) starts sysd_powerup()'s ssc_bto=0, so no
       `deposit sysd bto 0` is needed here the way mchkdiff.js needs one. */
    L.push(`examine -h sysd bto`);
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
    if (opts.rawTodr !== undefined) clk.todrReg = opts.rawTodr | 0;   // mirrors `deposit CLK TODR` above
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
        clkInt: (cpu.exc.intReq[IPL_CLK_ABS - IPL_HMIN] >>> INT_V_CLK) & 1,
        bto: cpu.exc.sscBto >>> 0
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
    /* `examine -h sysd bto` prints "BTO:\tvalue" the same generic way -- see mchkdiff.js's
       parseSimhBto() for the identical precedent. */
    let btoM = /^BTO:\s*([0-9A-Fa-f]+)/m.exec(out);
    if (!btoM) throw new Error(`timerdiff: SIMH did not report sysd BTO; output:\n${out}`);
    return {
        R0: get("R0"), R1: get("R1"), PC: get("PC"), PSL: get("PSL"),
        clkInt: parseInt(intM[1], 16) & 1,
        bto: parseInt(btoM[1], 16) >>> 0
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

/**
 * deriveUnownedIprs()
 *
 * Standing rule 5: derive the "no device owns this" IPR set programmatically, from the SHIPPED
 * dispatch's own observed behavior, rather than transcribe one.  Runs a real MFPR for every
 * architecturally-valid register number (0..MT_MAX) on a fresh JS machine and records which ones
 * leave sscBto set afterward.  On-chip write-only registers (SIRR/TBIA/TBIS/TBCHK) fault before
 * ever reaching readIPR() -- every SCB vector here points at a NOP page, so that fault dispatches
 * harmlessly instead of a HALT, and sscBto simply stays 0 for them, excluding them from the result
 * without any hand-picked exclusion list.  IPR_DEVICE members and SID/CONPC/CONPSL are excluded
 * the identical way: the dispatch itself never reaches the BTO-setting default for them.
 *
 * @returns {Array<number>}
 */
function deriveUnownedIprs()
{
    let unowned = [];
    for (let n = 0; n <= MT_MAX; n++) {
        let {bus, cpu} = makeMachine();
        cpu.exc.scbb = R_SCBB;
        cpu.regs[14] = R_KSP;
        cpu.exc.stk[0] = R_KSP;
        for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
        for (let v = 0; v < 0x200; v += 4) bus.setLong(R_SCBB + v, R_HANDLER);
        let code = mfpr(n, 0);
        for (let i = 0; i < code.length; i++) bus.setByte(R_CODE + i, code[i]);
        cpu.psl = 0;
        cpu.setPC(R_CODE);
        try { cpu.stepCPU(1); } catch (e) { continue; }
        if ((cpu.exc.sscBto >>> 0) === 0x80000000) unowned.push(n);
    }
    return unowned;
}

/**
 * caseNicrIcrInert(bin, scratch, failures)
 *
 * NICR(25)/ICR(26) are architected IPR NUMBERS this SIMH model does not wire to anything at all
 * (vax_sysdev.c's ReadIPR/WriteIPR has no case for either) -- MTPR then MFPR must round-trip as
 * the inert default (write dropped, read 0), NOT as a reload register a naive port might invent.
 *
 * MEASURED DIVERGENCE, RESOLVED (pcjsvax-b4b): vax_sysdev.c's ReadIPR/WriteIPR `default:` case --
 * the one NICR/ICR (and every other unowned off-chip number, see deriveUnownedIprs() above) falls
 * into -- ALSO sets SSCBTO_BTO (`examine sysd bto` measured 0x00000000 -> 0x80000000 across a real
 * MFPR NICR).  This used to be a PINNED divergence (simh=BTO set, js=BTO clear), asserted rather
 * than merely printed so a later exc.js change would force a red re-decision instead of silently
 * relabelling a new value "(match)".  exc.js's readIPR()/writeIPR() now set sscBto |= SSCBTO_BTO
 * (BTO only, never SSCBTO_RWT -- that stays busTimeout()'s exclusively, pcjsvax-446) on this same
 * default arm, so the pin is flipped here to an ordinary EQUALITY assertion, graded against the
 * full derived set -- not just NICR/ICR -- live against the oracle.
 */
function caseNicrIcrInert(bin, scratch, failures)
{
    let derived = deriveUnownedIprs();
    if (!derived.includes(MT.NICR) || !derived.includes(MT.ICR)) {
        failures.push(`nicrIcrInert: derived unowned-IPR set is missing NICR/ICR -- got [${derived.join(",")}]; ` +
            `exc.js's readIPR()/writeIPR() no longer sets BTO for them, which is this item's own regression to catch`);
    }
    for (let prn of derived) {
        let tag = `passthrough_prn${prn}`;
        let code = asm(mtpr(0x12345678, prn), mfpr(prn, 0));
        let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2});
        checkExact(failures, tag, js.R0, simh.R0, "R0 (must read back 0, not the written value)");
        checkExact(failures, tag, js.bto, simh.bto, "SSC BTO (pcjsvax-b4b: unowned IPR must set BTO on both engines)");
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
 * caseRomVsNonRomRawDiscriminator(bin, scratch, failures)
 *
 * THE authoritative, live, BIT-EXACT, time-independent grading of the ROM-detection special case
 * -- see the file header's "THE ROM-DETECTION SPECIAL CASE" section for why write-then-immediate-
 * read cannot do this (it converges to approximately the written value via EITHER branch) and why
 * a raw register poke can: `deposit CLK TODR n` (SIMH) / `clk.todrReg = n` (JS) sets the COUNTED
 * register directly, WITHOUT touching the wall-clock anchor, on both engines.
 *
 * THREE contexts, same poked value `n`, three fresh runFixedCase() invocations (one process each,
 * `reset all` between them -- no state leaks):
 *
 *   ROM PRIMARY   MFPR from ROM_CODE_ADDR must read back EXACTLY `n` -- raw register, unconditional
 *                 -- on BOTH engines, bit-exact.  A "todrRd always wall-clock" mutation (the item's
 *                 own named gotcha, inverse direction) fails HERE: the wall-clock formula ignores
 *                 the raw poke entirely (the anchor is still wherever the last real todr_wr/resync
 *                 left it), so a mutant's ROM-context read is NOT `n`.
 *   ROM MIRROR    MFPR from ROM_MIRROR_CODE_ADDR (same bytes, aliased) must NOT read back `n` on
 *                 EITHER engine -- the correct ROM_MASK does not match the mirror, so this is
 *                 STILL the wall-clock branch there.  A "ROM_MASK widened to the mirror" mutation
 *                 (MUTATIONS.rom_mask_includes_mirror) fails HERE: it WOULD read back `n`.
 *   NON-ROM       MFPR from R_CODE must NOT read back `n` on EITHER engine, for the same reason.
 *                 A "todrRd always raw" mutation (M6 -- the item's own gotcha in the FORWARD
 *                 direction: never taking the wall-clock branch at all) fails HERE: it WOULD read
 *                 back `n` outside ROM too.
 *
 * Cross-engine equality is asserted ONLY for the ROM-primary case (the one branch that is
 * genuinely time-independent); the mirror/non-ROM cases assert "not n" independently on each
 * engine, which is the honest, time-independent claim those two branches actually support.
 *
 * @param {string} bin
 * @param {string} scratch
 * @param {Array<string>} failures
 */
function caseRomVsNonRomRawDiscriminator(bin, scratch, failures)
{
    let n = 0x2A;
    let contexts = [
        {tag: "raw_rom_primary", opts: {rom: true, romMirror: false}, mustEqualN: true},
        {tag: "raw_rom_mirror", opts: {rom: true, romMirror: true}, mustEqualN: false},
        {tag: "raw_nonrom", opts: {rom: false}, mustEqualN: false}
    ];
    for (let c of contexts) {
        let code = mfpr(MT.TODR, 0);
        let {js, simh} = runFixedCase(bin, scratch, c.tag, code,
            Object.assign({steps: 1, rawTodr: n}, c.opts));
        if (c.mustEqualN) {
            if ((js.R0 >>> 0) !== n) failures.push(`${c.tag}: JS R0=${hex(js.R0)}, expected the raw-poked value ${hex(n)} (ROM branch)`);
            if ((simh.R0 >>> 0) !== n) failures.push(`${c.tag}: SIMH R0=${hex(simh.R0)}, expected the raw-poked value ${hex(n)} (ROM branch)`);
        } else {
            if (js.R0 === n) failures.push(`${c.tag}: JS R0 == raw-poked value 0x${hex(n)} -- wrongly took the ROM (raw) branch outside the primary ROM window`);
            if (simh.R0 === n) failures.push(`${c.tag}: SIMH R0 == raw-poked value 0x${hex(n)} unexpectedly (oracle itself took the raw branch here -- re-check the address)`);
        }
    }
    covered.todrRom = true;
    covered.todrNonRom = true;
}

/**
 * caseTodrStaysStoppedAcrossTicks(bin, scratch, failures)
 *
 * Kills a removed "!todrBlow && todrReg !== 0" tick() guard: with TODR raw-poked to 0 ("clock not
 * running"), run PAST INSTRS_PER_TICK real instructions (so the SHIPPED code's own deterministic
 * tick model genuinely fires at least once) and assert the RAW register is STILL exactly 0 on
 * BOTH engines.  This is a real, live, cross-engine comparison -- not merely a JS-only check --
 * and it needs no timing coordination between the two engines at all: real SIMH's OWN clk_svc
 * guard (`if (!todr_blow && todr_reg) todr_reg = todr_reg + 1`) ALSO refuses to start a stopped
 * clock, so the oracle's todr_reg stays 0 regardless of whether it services any real tick in the
 * (sub-millisecond) real time this file's small step budget takes -- the guard's zero-check, not
 * tick TIMING, is what both sides are being held to.
 *
 * @param {string} bin
 * @param {string} scratch
 * @param {Array<string>} failures
 */
function caseTodrStaysStoppedAcrossTicks(bin, scratch, failures)
{
    let tag = "todr_stopped_survives_many_ticks";
    let nSteps = INSTRS_PER_TICK * 3 + 5;

    let L = ["set cpu 16m", "reset all", `deposit CLK TODR 0`];
    for (let i = 0; i < nSteps + 8; i++) L.push(`deposit -b ${hex(R_CODE + i)} ${NOP_BYTE.toString(16)}`);
    L.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`);
    for (let s = 0; s < nSteps; s++) L.push("step 1");
    L.push("examine -h CLK TODR", "exit");
    let out = runSimh(bin, L.join("\n") + "\n", path.join(scratch, `timerdiff-${tag}.ini`));
    let m = /^TODR:\s*([0-9A-Fa-f]+)/m.exec(out);
    if (!m) throw new Error(`timerdiff: case ${tag} -- SIMH did not report CLK TODR; output:\n${out}`);
    let simhTodrValue = parseInt(m[1], 16) >>> 0;

    let {bus, cpu, clk} = makeMachine();
    clk.todrReg = 0;
    primeNopRun(bus, cpu, nSteps);
    for (let i = 0; i < nSteps; i++) cpu.stepCPU(1);

    if (simhTodrValue !== 0) failures.push(`${tag}: oracle's own raw TODR is ${hex(simhTodrValue)}, not 0 -- test premise broken`);
    if ((clk.todrReg >>> 0) !== 0) failures.push(`${tag}: JS todrReg=${hex(clk.todrReg)} after ${nSteps} instructions (>= ${INSTRS_PER_TICK * 3} ticks worth) -- stopped-clock guard did not hold`);
}

/**
 * caseTodrRate(bin, scratch, failures)
 *
 * Kills a mis-scaled wall-clock formula (ms instead of centiseconds, or any other wrong constant)
 * -- the round-trip band [data,data+2] used elsewhere proves the read is the INVERSE of the write
 * but cannot see absolute RATE (a scale bug is invisible when elapsed real time is itself under a
 * millisecond).  This case inserts REAL elapsed time -- SIMH's native `sleep <seconds>` (confirmed
 * by direct execution: `deposit CLK TODR 100; sleep 0.5; examine CLK TODR` leaves the RAW register
 * untouched, but a real MTPR/sleep/MFPR round trip through the wall-clock branch measured exactly
 * 100 + 50 = 150, i.e. the formula is correctly centiseconds-per-real-centisecond) -- matched by an
 * equal-duration host busy-wait on the JS side.  MEASURED CORRECTION (veracity re-dispatch round
 * 2): this doc comment previously claimed the expected delta is "computed from ACTUALLY MEASURED
 * elapsed time on each engine independently" -- true of the JS half (actualMs, from Date.now()
 * before/after the busy-wait) but FALSE of the SIMH half, where simhExpectedDelta is a FIXED
 * NOMINAL (`sleepSec * 100`), not a measurement -- `sleep` is not instrumented to report back how
 * long it actually blocked.  The tolerance (see TOL below) is generous enough to cover that
 * nominal-vs-actual gap for `sleep` too (measured directly: 130 real SIMH samples across idle,
 * loaded and heavily-loaded regimes, max observed drift 2 centiseconds against an 8-centisecond
 * tolerance -- and only ONE direction is reachable, since `sleep` can only overshoot its argument,
 * never undershoot it).  See caseTodrOverflowBoundary() below for a boundary probe that grades
 * this same RATE bit-exact with zero elapsed time and zero tolerance, if a fully deterministic
 * check is preferred to this one; both are kept, as this one is also sound.
 *
 * @param {string} bin
 * @param {string} scratch
 * @param {Array<string>} failures
 */
function caseTodrRate(bin, scratch, failures)
{
    let tag = "todr_wallclock_rate";
    let data = 100;
    let sleepSec = 0.4;

    /* ---- SIMH side ---- */
    let writeCode = mtpr(data, MT.TODR);
    let readCode = mfpr(MT.TODR, 0);
    let L = ["set cpu 16m", "set cpu simhalt", "reset all", "deposit MAPEN 0"];
    for (let i = 0; i < writeCode.length; i++) L.push(`deposit -b ${hex(R_CODE + i)} ${writeCode[i].toString(16)}`);
    for (let i = 0; i < readCode.length; i++) L.push(`deposit -b ${hex(R_CODE + 16 + i)} ${readCode[i].toString(16)}`);
    L.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`, "step 1", `sleep ${sleepSec}`,
        `deposit PC ${hex(R_CODE + 16)}`, "step 1", "examine -h R0", "exit");
    let out = runSimh(bin, L.join("\n") + "\n", path.join(scratch, `timerdiff-${tag}.ini`));
    let m = /^R0:\s*([0-9A-Fa-f]+)/m.exec(out);
    if (!m) throw new Error(`timerdiff: case ${tag} -- SIMH did not report R0; output:\n${out}`);
    let simhR0 = parseInt(m[1], 16) >>> 0;
    let simhExpectedDelta = Math.round(sleepSec * 100);   // NOMINAL, not measured -- see the file header

    /* ---- JS side: real host busy-wait of the SAME nominal duration, measured actual elapsed ---- */
    let {bus, cpu, clk} = makeMachine();
    for (let i = 0; i < writeCode.length; i++) bus.setByte(R_CODE + i, writeCode[i]);
    cpu.psl = 0;
    cpu.setPC(R_CODE);
    cpu.stepCPU(1);
    let t0 = Date.now();
    while (Date.now() - t0 < sleepSec * 1000) { /* busy-wait: no instructions retire, no ticks fire */ }
    let actualMs = Date.now() - t0;
    for (let i = 0; i < readCode.length; i++) bus.setByte(R_CODE + 16 + i, readCode[i]);
    cpu.setPC(R_CODE + 16);
    cpu.stepCPU(1);
    let jsR0 = cpu.regs[0] >>> 0;
    let jsExpectedDelta = Math.round(actualMs / 10);

    const TOL = 8;      // centiseconds; generous vs. OS jitter, minuscule vs. any 10x/100x scale bug
    if (simhR0 < data + simhExpectedDelta - TOL || simhR0 > data + simhExpectedDelta + TOL) {
        failures.push(`${tag}: SIMH R0=${hex(simhR0)} (=${simhR0 - data} centiseconds elapsed), ` +
            `expected ~${simhExpectedDelta} (sleep ${sleepSec}s) +/-${TOL}`);
    }
    if (jsR0 < data + jsExpectedDelta - TOL || jsR0 > data + jsExpectedDelta + TOL) {
        failures.push(`${tag}: JS R0=${hex(jsR0)} (=${jsR0 - data} centiseconds elapsed), ` +
            `expected ~${jsExpectedDelta} (measured ${actualMs}ms busy-wait) +/-${TOL}`);
    }
    covered.todrNonRom = true;
    covered.todrWrite = true;
}

/**
 * caseTodrOverflowBoundary(bin, scratch, failures)
 *
 * pcjsvax-954 veracity finding, round 2: todr_rd()'s TOY_MAX_SECS overflow branch (vax_stddev.c:
 * 490-494) was entirely UNPORTED and undisclosed in the first two rounds of this file -- neither
 * phase reaches it (phase 2 sends full-range randoms only through the ROM raw context, and caps
 * non-ROM randoms at 5000).  Measured live, cross-engine:
 *
 *     MTPR 0xFFFFFF9F,TODR ; MFPR TODR,R0   -> R0=0xFFFFFF9F  (below the threshold: unchanged)
 *     MTPR 0xFFFFFFA0,TODR ; MFPR TODR,R0   -> R0=0x00000000  (AT the threshold: register zeroed)
 *
 * 0xFFFFFFA0 = TOY_MAX_SECS * 100 -- a PURE FUNCTION of the centiseconds-per-second constant (see
 * clk.js's TOY_MAX_SECS doc comment), so this is not an arbitrary large value: it is the exact
 * boundary the overflow branch fires at, one centisecond above the value that must NOT overflow.
 *
 * MEASURED, NOT ASSUMED (round 2, second pass): 0xFFFFFFA0 EXACTLY is a real-time RACE on the live
 * oracle, not a stable boundary -- five back-to-back probes of the identical MTPR/sleep-free
 * write-then-immediate-read gave R0 = 0/0xFFFFFFA0/0/0xFFFFFFA0/0, roughly evenly split.  This is
 * NOT this file inventing tolerance where none is needed: `sim_timespec_diff`'s tv_sec/tv_nsec
 * borrow arithmetic makes the reconstructed elapsed-seconds value sensitive to sub-millisecond
 * scheduling noise EXACTLY when the true elapsed time sits within about a millisecond of a whole
 * second -- which is unavoidably true when the written value's OWN fractional part is designed to
 * land tv_sec precisely on TOY_MAX_SECS.  0xFFFFFFA1 (one centisecond further in) does not have
 * this problem -- measured stable at 0x00000000 across 3 probes on the live oracle AND 3 probes on
 * this port -- so it is used as the "at/past threshold" value instead of the mathematically exact
 * 0xFFFFFFA0.  It is still bit-exact, zero-sleep, effectively zero-tolerance (see below), and still
 * strictly stronger than caseTodrRate()'s banded check: a millisecond-scaled implementation could
 * never reach a value this large within a 32-bit register at all (it would need ~100x more range,
 * itself unrepresentable), so a 10x/100x rate bug that happened to dodge caseTodrRate()'s tolerance
 * would still show up here as "0xFFFFFFA1 read back unchanged instead of 0" or "0xFFFFFF9F read
 * back as 0".  The boundary decision only depends on which side of TOY_MAX_SECS the near-instant
 * round trip lands on, which write-then-immediate-read (see the file header) puts on the correct
 * side by construction for both chosen values -- comfortably more than the sub-millisecond noise
 * that makes the EXACT boundary value alone unsafe to assert on.
 *
 * @param {string} bin
 * @param {string} scratch
 * @param {Array<string>} failures
 */
function caseTodrOverflowBoundary(bin, scratch, failures)
{
    const BELOW = 0xFFFFFF9F, AT = 0xFFFFFFA1;     // NOT the mathematically exact 0xFFFFFFA0 -- see above
    for (let [label, val, expect] of [["below_threshold", BELOW, BELOW], ["at_threshold", AT, 0]]) {
        let tag = `todr_overflow_${label}`;
        let code = asm(mtpr(val, MT.TODR), mfpr(MT.TODR, 0));
        let {js, simh} = runFixedCase(bin, scratch, tag, code, {steps: 2, rom: false});
        if ((js.R0 >>> 0) !== (expect >>> 0)) failures.push(`${tag}: JS R0=${hex(js.R0)}, expected ${hex(expect)}`);
        if ((simh.R0 >>> 0) !== (expect >>> 0)) failures.push(`${tag}: SIMH R0=${hex(simh.R0)}, expected ${hex(expect)}`);
    }

    /*
     * MEASURED CORRECTION (veracity re-dispatch round 4): vax_stddev.c's overflow branch is
     * `return todr_reg = 0` -- an ASSIGNMENT, not just a return -- and the R0-only checks above
     * cannot see that: a mutation that keeps `return 0` but drops the `this.todrReg = 0` write
     * passes every assertion above (R0 is still 0 either way).  Measured live: after the
     * overflowing read, the real oracle's RAW register is ALSO 0 (`examine CLK TODR` reads 0, not
     * merely the MFPR result) -- compared here cross-engine, catching the assignment specifically.
     */
    {
        let tag = "todr_overflow_register_zeroed";
        let code = asm(mtpr(AT, MT.TODR), mfpr(MT.TODR, 0));
        let L = ["set cpu 16m", "set cpu simhalt", "reset all", "deposit MAPEN 0"];
        for (let i = 0; i < code.length; i++) L.push(`deposit -b ${hex(R_CODE + i)} ${code[i].toString(16)}`);
        L.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`, "step 2", "examine -h CLK TODR", "exit");
        let out = runSimh(bin, L.join("\n") + "\n", path.join(scratch, `timerdiff-${tag}.ini`));
        let m = /^TODR:\s*([0-9A-Fa-f]+)/m.exec(out);
        if (!m) throw new Error(`timerdiff: case ${tag} -- SIMH did not report CLK TODR; output:\n${out}`);
        let simhTodrAfter = parseInt(m[1], 16) >>> 0;

        let {bus, cpu, clk} = makeMachine();
        for (let i = 0; i < code.length; i++) bus.setByte(R_CODE + i, code[i]);
        cpu.psl = 0;
        cpu.setPC(R_CODE);
        cpu.stepCPU(2);
        let jsTodrAfter = clk.todrReg >>> 0;

        if (simhTodrAfter !== 0) failures.push(`${tag}: oracle's own raw TODR is ${hex(simhTodrAfter)} after the overflowing read, expected 0 -- test premise broken`);
        if (jsTodrAfter !== 0) failures.push(`${tag}: JS raw todrReg=${hex(jsTodrAfter)} after the overflowing read, expected 0 (the assignment side-effect of \`return todr_reg = 0\` did not happen)`);
    }

    covered.todrNonRom = true;
    covered.todrWrite = true;
}

/**
 * caseBareTodrAfterReset(bin, scratch, failures)
 *
 * pcjsvax-954 veracity finding: clk_reset() (vax_stddev.c:570-588) calls todr_resync() on a
 * fresh process's FIRST reset, so a BARE MFPR TODR -- no MTPR at all -- is non-zero and BLOW-clear
 * on real hardware.  clk.js's reset() now reproduces this (see its "POWER-ON RESYNC" doc).  The
 * EXACT computed value can never be bit-matched live (both engines resync from the real host
 * clock at slightly different instants in slightly different processes); what IS graded, on each
 * engine:
 *
 *   - BLOW == 0 (the clock is running the moment the process starts, never "battery low")
 *   - the ROM-context read is >= 0x10000000 and != 0 (resync's formula guarantees this shape)
 *   - the ROM-context read and the raw register examined directly agree WITHIN ITSELF (self-
 *     consistency: whatever the engine's own resync produced, ROM-context MFPR reports exactly
 *     that, unconditionally -- still the same time-independent claim caseRomVsNonRomRawDiscriminator
 *     makes, just against a LIVE resync value instead of a hand-picked one)
 *   - the non-ROM read is also != 0 (still running, not accidentally "stopped")
 *   - a BOUNDED cross-engine delta -- NOT bit-exact equality, but a delta bounded by the ACTUAL
 *     MEASURED real time the comparison took, not a fixed guess (see below)
 *
 * MEASURED CORRECTION (veracity re-dispatch round 3): round 2 added the bounded-delta idea with a
 * FIXED DELTA_TOL=500, justified as "generous against measured process-start skew (~80cs)" -- but
 * that 80cs was a measurement of a single BACK-TO-BACK MTPR/MFPR pair (checkTodrRunning()), not of
 * what this function actually spans.  The ORIGINAL version of this function measured simhRom,
 * THEN spawned SIMH AGAIN for simhNonRom, and only THEN built jsRom -- so the compared delta
 * covered TWO FULL SIMH PROCESS LIFETIMES, and scales with host load: measured 123-130cs light,
 * 241-455cs at load average 40, 330-867cs at load average 91 (15 of 20 trials over 500).  HANDOFF.md
 * 11 forbids the obvious fix (widen the tolerance) -- a wider fixed number is still a blind guess,
 * just a more generous one, and would eventually be wrong again under different load.
 *
 * THE FIX: bracket EACH engine's resync as tightly as the process model allows, and size the
 * tolerance from the ACTUAL MEASURED bracket width, the same "measured elapsed, not assumed"
 * technique caseTodrRate() already uses for its own real-time comparison.  `t0`/`t1` bracket the
 * SIMH ROM probe's ENTIRE process lifetime (spawn through exit); the JS machine is then built
 * IMMEDIATELY afterward (a few microseconds, synchronous), so its resync instant is essentially
 * `t1`.  SIMH's OWN resync instant is somewhere inside `[t0, t1]` -- unknown exactly, but bounded:
 * the true elapsed time between the two resyncs is therefore AT MOST `t1 - t0` (worst case: SIMH
 * resynced at t0) and AT LEAST ~0 (best case: SIMH resynced right at t1).  `NOISE` covers JS-side
 * scheduling/GC jitter on top of that bound.  This tolerance now WIDENS under load exactly as far
 * as the actual measured bracket widens, and TIGHTENS when the host is idle -- not a blind guess
 * in either direction -- while still catching the round-1 DST day-count bug (8,640,000cs) by five
 * or more orders of magnitude even at the worst measured load in this item's own history.
 *
 * ROM and non-ROM each get their OWN bracket and their OWN JS probe built immediately after it
 * (interleaved: simh-rom, js-rom, simh-nonrom, js-nonrom), rather than the original's "both SIMH
 * calls, then both JS calls" ordering, which is what created the two-lifetime gap in the first
 * place.
 *
 * DISCLOSED, NOT GUARDED: the resync formula encodes seconds-since-Jan-1 LOCAL time, so two
 * engines whose resync instants straddle a local New Year midnight would differ by ~3.15e9
 * centiseconds even though only milliseconds of real time separated them.  Not guarded against --
 * this file's own runs take well under a second end to end, so the only way to hit it is running
 * this exact differential within a few hundred milliseconds of local midnight on December 31st,
 * a coincidence not worth defending against -- but it is a real, disclosed limit of this bound.
 *
 * @param {string} bin
 * @param {string} scratch
 * @param {Array<string>} failures
 */
function caseBareTodrAfterReset(bin, scratch, failures)
{
    let tag = "todr_bare_after_reset";
    let readCode = mfpr(MT.TODR, 0);

    function probeSimh(execAddr, writeAddr, label) {
        let L = ["set cpu 16m", "set cpu simhalt", "reset all", "deposit MAPEN 0"];
        for (let i = 0; i < readCode.length; i++) L.push(`deposit -b ${hex(writeAddr + i)} ${readCode[i].toString(16)}`);
        L.push(`deposit PSL 0`, `deposit PC ${hex(execAddr)}`, "step 1",
            "examine -h R0", "examine -h CLK TODR", "examine -h CLK BLOW", "exit");
        let out = runSimh(bin, L.join("\n") + "\n", path.join(scratch, `timerdiff-${tag}-${label}.ini`));
        let r0 = /^R0:\s*([0-9A-Fa-f]+)/m.exec(out);
        let todr = /^TODR:\s*([0-9A-Fa-f]+)/m.exec(out);
        let blow = /^BLOW:\s*([0-9A-Fa-f]+)/m.exec(out);
        if (!r0 || !todr || !blow) throw new Error(`timerdiff: case ${tag} -- incomplete SIMH output:\n${out}`);
        return {r0: parseInt(r0[1], 16) >>> 0, todr: parseInt(todr[1], 16) >>> 0, blow: parseInt(blow[1], 16) & 1};
    }
    function probeJs(makeFn, addr, direct) {
        let m = makeFn();
        for (let i = 0; i < readCode.length; i++) (direct ? m.bus.setByteDirect(addr + i, readCode[i]) : m.bus.setByte(addr + i, readCode[i]));
        m.cpu.setPC(addr);
        m.cpu.stepCPU(1);
        return {r0: m.cpu.regs[0] >>> 0, todr: m.clk.todrReg >>> 0, blow: m.clk.todrBlow & 1};
    }

    const NOISE = 50;   // centiseconds; covers JS-side scheduling/GC jitter on top of the measured bracket

    let t0 = Date.now();
    let simhRom = probeSimh(ROM_CODE_ADDR, ROM_CODE_ADDR, "rom");
    let t1 = Date.now();
    let jsRom = probeJs(makeMachineWithRom, ROM_CODE_ADDR, true);
    let romTol = Math.round((t1 - t0) / 10) + NOISE;

    let t2 = Date.now();
    let simhNonRom = probeSimh(R_CODE, R_CODE, "nonrom");
    let t3 = Date.now();
    let jsNonRom = probeJs(makeMachine, R_CODE, false);
    let nonRomTol = Math.round((t3 - t2) / 10) + NOISE;

    for (let [label, r] of [["simh ROM", simhRom], ["simh non-ROM", simhNonRom], ["js ROM", jsRom], ["js non-ROM", jsNonRom]]) {
        if (r.blow !== 0) failures.push(`${tag}: ${label} BLOW=${r.blow}, expected 0 (resync must clear it)`);
        if (r.r0 === 0) failures.push(`${tag}: ${label} R0=0, expected non-zero after power-on resync`);
    }
    if (simhRom.r0 !== simhRom.todr) failures.push(`${tag}: simh ROM R0=${hex(simhRom.r0)} != its own raw TODR=${hex(simhRom.todr)} (self-consistency)`);
    if (jsRom.r0 !== jsRom.todr) failures.push(`${tag}: js ROM R0=${hex(jsRom.r0)} != its own raw todrReg=${hex(jsRom.todr)} (self-consistency)`);
    if ((jsRom.r0 >>> 0) < 0x10000000) failures.push(`${tag}: js ROM R0=${hex(jsRom.r0)} below 0x10000000 -- resync's formula shape not reproduced`);
    if ((simhRom.r0 >>> 0) < 0x10000000) failures.push(`${tag}: simh ROM R0=${hex(simhRom.r0)} below 0x10000000 -- test premise broken`);

    let romDelta = Math.abs((simhRom.r0 >>> 0) - (jsRom.r0 >>> 0));
    if (romDelta > romTol) {
        failures.push(`${tag}: ROM |simh R0 - js R0| = ${romDelta} centiseconds, exceeds measured-elapsed tolerance ${romTol} ` +
            `(bracket ${t1 - t0}ms, simh=${hex(simhRom.r0)} js=${hex(jsRom.r0)})`);
    }
    let nonRomDelta = Math.abs((simhNonRom.r0 >>> 0) - (jsNonRom.r0 >>> 0));
    if (nonRomDelta > nonRomTol) {
        failures.push(`${tag}: non-ROM |simh R0 - js R0| = ${nonRomDelta} centiseconds, exceeds measured-elapsed tolerance ${nonRomTol} ` +
            `(bracket ${t3 - t2}ms, simh=${hex(simhNonRom.r0)} js=${hex(jsNonRom.r0)})`);
    }

    covered.todrRom = true;
    covered.todrNonRom = true;
}

/**
 * Outside ROM, nonzero TODR: write-then-immediate-read, graded against the analytically-known
 * band [data, data+2] on EACH ENGINE INDEPENDENTLY -- see the file header for why this is the
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
    caseRomVsNonRomRawDiscriminator(bin, scratch, failures);
    caseTodrStaysStoppedAcrossTicks(bin, scratch, failures);
    caseTodrRate(bin, scratch, failures);
    caseTodrOverflowBoundary(bin, scratch, failures);
    caseBareTodrAfterReset(bin, scratch, failures);
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
    },
    /* ROM_MASK widened to cover the WHOLE mirrored span (ROM_BASE..ROM_BASE+2*ROM_SIZE), the
       naive "ADDR_IS_ROM-style range check" a port could plausibly reach for instead of the exact
       0xFFFE0000-equivalent window -- see clk.js's file header "MASK PRECISION" note. */
    "rom_mask_includes_mirror"() {
        let orig = ClkVAX.prototype.todrRd;
        ClkVAX.prototype.todrRd = function() {
            let pc = (this.exc && typeof this.exc.faultPC === "number") ? (this.exc.faultPC >>> 0) : 0;
            let widened = pc >= ROM_BASE_CONST && pc < (ROM_BASE_CONST + 2 * ROM_SIZE_CONST);
            if (widened) return this.todrReg | 0;
            if (this.todrReg === 0) return 0;
            let elapsedMs = Date.now() - this.wallBaseMs;
            return Math.round(elapsedMs / 10) | 0;
        };
        return () => { ClkVAX.prototype.todrRd = orig; };
    },
    /* vax_stddev.c's overflow branch is `return todr_reg = 0` -- an ASSIGNMENT.  This mutation
       keeps the RETURN VALUE (0, so any R0-only check still passes) but drops the register write,
       exactly the gap caseTodrOverflowBoundary()'s dedicated register-comparison closes. */
    "overflow_return_only_no_assignment"() {
        let orig = ClkVAX.prototype.todrRd;
        ClkVAX.prototype.todrRd = function() {
            let pc = (this.exc && typeof this.exc.faultPC === "number") ? (this.exc.faultPC >>> 0) : 0;
            if (((pc & ROM_MASK) >>> 0) === ROM_TAG) return this.todrReg | 0;
            if (this.todrReg === 0) return 0;
            let elapsedMs = Date.now() - this.wallBaseMs;
            if (elapsedMs / 1000 >= TOY_MAX_SECS) return 0;   // no `this.todrReg = 0` here
            return Math.round(elapsedMs / 10) | 0;
        };
        return () => { ClkVAX.prototype.todrRd = orig; };
    },
    /* pcjsvax-b4b: the pre-fix bug itself -- the off-chip default reads 0 / drops the write but
       never sets the bus-timeout bit at all.  Composes over the shipped readIPR/writeIPR (calls
       through to `orig`, then strips the bit back off) rather than substituting a hand-written
       replacement -- standing rule 11: this PERTURBS the shipped path, it does not replace it. */
    "bto_not_set_on_unowned_ipr"() {
        let origRead = VAXExc.prototype.readIPR;
        let origWrite = VAXExc.prototype.writeIPR;
        VAXExc.prototype.readIPR = function(prn) {
            let before = this.sscBto;
            let val = origRead.call(this, prn);
            this.sscBto = before;                   // undo whatever BTO the real path just set
            return val;
        };
        VAXExc.prototype.writeIPR = function(prn, val) {
            let before = this.sscBto;
            origWrite.call(this, prn, val);
            this.sscBto = before;
        };
        return () => { VAXExc.prototype.readIPR = origRead; VAXExc.prototype.writeIPR = origWrite; };
    },
    /* pcjsvax-b4b: the OTHER wrong shape -- collapsing the IPR-default path into busTimeout()'s
       BTO|RWT instead of keeping it BTO-only (vax_sysdev.c:913/982 never touches RWT; only the
       register-space/Qbus bus-fault path does, pcjsvax-446).  Composes over the shipped path the
       same way: let it run, then OR the extra bit in afterward. */
    "bto_set_with_rwt_also_set"() {
        let origRead = VAXExc.prototype.readIPR;
        let origWrite = VAXExc.prototype.writeIPR;
        VAXExc.prototype.readIPR = function(prn) {
            let val = origRead.call(this, prn);
            if ((this.sscBto & SSCBTO_BTO) !== 0) this.sscBto = (this.sscBto | SSCBTO_RWT) | 0;
            return val;
        };
        VAXExc.prototype.writeIPR = function(prn, val) {
            origWrite.call(this, prn, val);
            if ((this.sscBto & SSCBTO_BTO) !== 0) this.sscBto = (this.sscBto | SSCBTO_RWT) | 0;
        };
        return () => { VAXExc.prototype.readIPR = origRead; VAXExc.prototype.writeIPR = origWrite; };
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
    {
        /* JS-only check mirroring caseRomVsNonRomRawDiscriminator()'s live "mirror must NOT read
           back n" assertion, applied directly to the mutated prototype method. */
        let restore = MUTATIONS.rom_mask_includes_mirror();
        let m = makeMachineWithRom();
        let n = 0x2A;
        m.clk.todrReg = n;
        let code = mfpr(MT.TODR, 0);
        /* Writes go to the PRIMARY half (the mirror's write path is undefined -- true alias, read
           only, see runFixedCase()'s own writeAddr/execAddr split); PC runs from the MIRROR. */
        for (let i = 0; i < code.length; i++) m.bus.setByteDirect(ROM_CODE_ADDR + i, code[i]);
        m.cpu.setPC(ROM_MIRROR_CODE_ADDR);
        m.cpu.stepCPU(1);
        let r0 = m.cpu.regs[0] >>> 0;
        restore();
        /* CORRECT behavior: the mirror is NOT in ROM_MASK's window, so r0 must NOT equal n.
           The mutation's bug signature is r0 === n (it wrongly took the raw/ROM branch there) --
           "caught" means this check actually observed that wrong value. */
        results.push({name: "rom_mask_includes_mirror", caught: r0 === n});
    }
    {
        /* M10: the overflow branch's ASSIGNMENT side-effect (`return todr_reg = 0`) dropped,
           keeping only the return value.  R0-only checks cannot see this; the raw register can. */
        let restore = MUTATIONS.overflow_return_only_no_assignment();
        let {cpu, clk} = makeMachine();
        clk.write(MT.TODR, 0xFFFFFFA1 | 0);
        let addr = R_CODE;
        let code = mfpr(MT.TODR, 0);
        for (let i = 0; i < code.length; i++) cpu.bus.setByte(addr + i, code[i]);
        cpu.psl = 0;
        cpu.setPC(addr);
        cpu.stepCPU(1);
        let r0 = cpu.regs[0] >>> 0;
        let todrRegAfter = clk.todrReg >>> 0;
        restore();
        /* R0 is 0 either way (that is the whole point of this mutation); only the raw register
           distinguishes correct (todrRegAfter===0) from the mutation (todrRegAfter still 0xFFFFFFA1). */
        results.push({name: "overflow_return_only_no_assignment", caught: r0 === 0 && todrRegAfter !== 0});
    }
    {
        /*
         * M12/M13 (veracity re-dispatch round 4): the round-3 DST fix (dayOfYear(), a leap-year-
         * aware calendar walk replacing a DST-broken ms-division) had NO regression guard --
         * reverting to the ms-division, or dropping the leap-year adjustment, both SURVIVE the
         * rest of this file's checks, because they are unreachable on this host (UTC, no DST, and
         * today is not Dec 31) and the bounded-delta assertion cannot help (both engines agree
         * under UTC).  This asserts the SHIPPED dayOfYear() directly (not a monkey-patched mutant
         * -- dayOfYear is a plain function export, not a prototype method, so there is nothing to
         * monkey-patch from outside the module) against a table of known (TZ, local instant) ->
         * day-of-year pairs, computed independently via Date.UTC's ms-division of the SAME
         * calendar date (safe: UTC has no DST, so this cross-check does not share dayOfYear()'s
         * own failure mode).  Covers: a spring-forward-affected instant (catches M12: reverting to
         * ms-division), a fall-back-affected instant (same), a plain leap year, a century
         * non-leap year (1900) and a century leap year (2000) (catches M13: dropping the
         * `%400===0` leap-year term or the leap adjustment entirely).
         */
        let dayOfYearFailures = [];
        const YDAY_CASES = [
            {tz: "America/New_York", y: 2026, mo: 2, d: 9, h: 0, mi: 30, note: "spring-forward-affected"},
            {tz: "America/New_York", y: 2026, mo: 10, d: 2, h: 1, mi: 30, note: "fall-back-affected"},
            {tz: "UTC", y: 2024, mo: 11, d: 31, h: 12, mi: 0, note: "leap year"},
            {tz: "UTC", y: 1900, mo: 11, d: 31, h: 12, mi: 0, note: "century non-leap"},
            {tz: "UTC", y: 2000, mo: 11, d: 31, h: 12, mi: 0, note: "century leap"}
        ];
        let origTz = process.env.TZ;
        for (let c of YDAY_CASES) {
            process.env.TZ = c.tz;
            let d = new Date(c.y, c.mo, c.d, c.h, c.mi, 0, 0);
            let got = dayOfYear(d);
            let expected = Math.round((Date.UTC(c.y, c.mo, c.d) - Date.UTC(c.y, 0, 1)) / 86400000);
            if (got !== expected) {
                dayOfYearFailures.push(`dayOfYear(TZ=${c.tz} ${c.y}-${c.mo + 1}-${c.d} ${c.h}:${c.mi}, ${c.note}): got ${got}, expected ${expected}`);
            }
        }
        if (origTz === undefined) delete process.env.TZ; else process.env.TZ = origTz;
        for (let f of dayOfYearFailures) console.log(`  ${f}`);
        results.push({name: "dayOfYear_table (M12/M13 guard)", caught: dayOfYearFailures.length === 0});
    }
    {
        /* pcjsvax-b4b, done condition 5: "BTO not set on an unowned IPR" -- the pre-fix bug,
           reinstated by MUTATIONS.bto_not_set_on_unowned_ipr(), must leave sscBto at 0 after an
           MFPR of a register no device owns (MT.NICR), which is exactly the divergence
           caseNicrIcrInert()'s checkExact(js.bto, simh.bto) would now flag against the oracle. */
        let restore = MUTATIONS.bto_not_set_on_unowned_ipr();
        let {cpu} = makeMachine();
        cpu.exc.readIPR(MT.NICR);
        let bto = cpu.exc.sscBto >>> 0;
        restore();
        results.push({name: "bto_not_set_on_unowned_ipr", caught: bto === 0});
    }
    {
        /* pcjsvax-b4b, done condition 5: "BTO set but RWT also set" -- the wrong-shape fix that
           collapses the IPR-default path into busTimeout()'s BTO|RWT, reinstated by
           MUTATIONS.bto_set_with_rwt_also_set(), must leave sscBto with RWT riding along, which is
           NOT what a real MFPR NICR produces on either engine (BTO only, vax_sysdev.c:913/982). */
        let restore = MUTATIONS.bto_set_with_rwt_also_set();
        let {cpu} = makeMachine();
        cpu.exc.readIPR(MT.NICR);
        let bto = cpu.exc.sscBto >>> 0;
        restore();
        results.push({name: "bto_set_with_rwt_also_set", caught: bto === ((SSCBTO_BTO | SSCBTO_RWT) >>> 0)});
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

    /* Every exit path -- success, an assertion/coverage FAIL, or an uncaught exception from
       proveDeterminism()/selfcheck() -- runs through this try/finally, so scratch is always
       removed (HANDOFF.md pcjsvax-bd1: this file had no rmSync of scratch anywhere, on any path). */
    try {

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

    } finally {
        fs.rmSync(scratch, {recursive: true, force: true});
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
