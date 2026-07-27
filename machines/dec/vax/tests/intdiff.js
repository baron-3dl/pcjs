/**
 * @fileoverview Differential test: VAX integer/logical/bit-field execution vs. a real Open SIMH
 *               microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * A differential test of cpu.js's 107 integer/logical/bit-field instruction bodies against a
 * REAL, EXECUTED Open SIMH microvax3900 -- no fixtures, no golden files.  Two phases:
 *
 *   RANDOMIZED   For every one of the 107 opcodes cpu.js implements (see cpu.js's IMPLEMENTED
 *                export -- this file never hand-maintains a second copy of that list), build N
 *                random pre-states (registers, a hot per-case memory region, and condition
 *                codes), drive a LIVE SIMH console through `deposit`/`step 1`/`examine` for each,
 *                execute the identical bytes through cpu.js, and compare the FULL register file
 *                plus PSL.  Addressing is deliberately simple (register-direct, PC-relative
 *                immediate, or absolute-through-a-preset-address) because addressing-mode
 *                RESOLUTION is decode.js's already-proven domain (pcjsvax-8c0's decodediff.js);
 *                this phase exists to grade EXECUTION -- arithmetic, condition codes, and the
 *                store path -- not effective-address computation.
 *   EHKAA        Real code: the entire 335,444-instruction EHKAA diagnostic trace (the same
 *                capture mechanism decodediff.js uses) is scanned for instructions whose opcode
 *                is one of ours.  Patch 0001's REGS line gives the pre-resolution register file of
 *                every instruction; the NEXT record's REGS line is therefore the exact
 *                post-EXECUTION register file of THIS one, provided (checked per-instance) no trap
 *                or interrupt intervened, i.e. the next record's PC is exactly this one's PC plus
 *                its instruction length.  Every data read the DECODER performs during specifier
 *                resolution is replayed from patch 0002's MEMR log (see decodediff.js's
 *                ReplayMachine, whose read-matching discipline this file reuses); memory-mode
 *                variable-bit-field instructions need an EXECUTION-time read beyond what MEMR
 *                covers (SIMH arms the read log only for the specifier-resolution loop) and are
 *                therefore skipped in this phase -- COUNTED and reported, never silently dropped
 *                -- and are exclusively the randomized phase's job to cover, which it does.
 *
 * WHAT IS DELIBERATELY OUT OF SCOPE (see cpu.js's file header for the full reasoning)
 * -------------------------------------------------------------------------------
 *   - EXECUTION-time faults (ADAWI odd address, bit-field size>32 or register-pair-span, WRITE_Q
 *     register-pair overflow).  The randomized generator never produces a state that triggers one
 *     (SIMH's SCB is not initialized in this harness's memory layout, so a real fault there would
 *     double-fault); reserved-ADDRESSING-mode faults are decodediff.js's FAULTS phase, a different
 *     fault category entirely (decode-time, not execution-time).
 *   - Deferred arithmetic traps (integer overflow, integer divide-by-zero): SIMH dispatches these
 *     at the TOP of the NEXT instruction, never during the one that set them, so a single-
 *     instruction register-file-plus-PSL comparison cannot observe the dispatch either way.  The
 *     CC.V bit IS graded in every case that sets it.
 *
 * COVERAGE IS ASSERTED, NOT REPORTED
 * -----------------------------------
 * Uniform random value pools make most comparisons trivial (0 vs 0) -- the lesson from pcjsvax-cd6
 * and repeated in every VAX differential since.  Value pools here are edge-weighted (0, 1,
 * signed-min, signed-max, all-ones, plus uniform random) and, for divide instructions
 * specifically, biased toward divisor=0 and the signed-min/-1 overflow pair, because those are the
 * one-cc-bit-different case a uniform pool essentially never hits.  Both phases assert a per-
 * opcode floor and fail the run if it is not met, along with the fraction of memory-destination
 * (vs. register-destination) stores and the fraction of non-trivial (nonzero result / cc/=0)
 * comparisons.  An undersized run (--cases-per-opcode below its floor) fails outright.
 *
 * --selfcheck injects deliberate defects into cpu.js's own dispatch table (not a stub copy) and
 * asserts the differential catches every one, then exits.
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import VAXCpu, { IMPLEMENTED, DISPATCH, HANDLERS } from "../modules/v2/cpu.js";
import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import { DROM, DROM_STRIDE, DR, IG, SPEC, OPCODES } from "../modules/v2/drom.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

/* Plumbing -- same shape as cpudiff.js's / romdiff.js's, deliberately, so a reader of any of
 * them trusts it once: $PCJS_VAX_REPO first, "../pcjs-vax" (a sibling of the pcjs checkout)
 * only as the fallback, since that guess is wrong when this repo is checked out as a worktree. */
function vaxRepo()
{
    if (process.env['PCJS_VAX_REPO']) return process.env['PCJS_VAX_REPO'];
    return path.resolve(__dirname, "../../../../../pcjs-vax");
}

/* ------------------------------------------------------------------------------------------- *
 * PRNG (mulberry32, matching decodediff.js/mmudiff.js so a failing seed is reproducible)          *
 * ------------------------------------------------------------------------------------------- */

function mulberry32(a)
{
    return function() {
        a |= 0; a = (a + 0x6D2B79F5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

function hex(v, n = 8) { return ((v >>> 0).toString(16).toUpperCase()).padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Locating SIMH.  Reuses the SAME patched binary decodediff.js/mmudiff.js grade against: this     *
 * file needs the REGS line (patch 0001) and the MEMR read log (patch 0002); it does not need       *
 * patch 0003's MMU tracing.  No fallback to self-comparison -- a missing SIMH fails the run.        *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_DECODE_BIN']) candidates.push(process.env['SIMH_DECODE_BIN']);
    if (process.env['SIMH_INT_BIN']) candidates.push(process.env['SIMH_INT_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH built with the decode-replay patch; it has no\n" +
        "fixture fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh\n" +
        "PATH or set $SIMH_INT_BIN.  Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 30 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * Opcode metadata -- specifier-type strings per opcode, read directly from drom.js so this file   *
 * can never hand-maintain a second, driftable copy of the operand layout.                          *
 * ------------------------------------------------------------------------------------------- */

const SPECNAME = {};
for (const [k, v] of Object.entries(SPEC)) SPECNAME[v] = k;

/**
 * opcodeOf(mnemonic) -- the one true opc for a mnemonic, from drom.js's OPCODES table.
 */
const OPC_OF = {};
for (let opc = 0; opc < 512; opc++) if (OPCODES[opc]) OPC_OF[OPCODES[opc]] = opc;

/**
 * specTypes(opc) -- array of specifier-type name strings ("RB", "ML", "WQ", "VB", ...) in
 * decode-ROM order.
 */
function specTypes(opc)
{
    let hdr = DROM[opc * DROM_STRIDE];
    let nsp = hdr & DR.NSPMASK;
    let out = [];
    for (let i = 0; i < nsp; i++) out.push(SPECNAME[DROM[opc * DROM_STRIDE + 1 + i]]);
    return out;
}

const WIDTH_OF = {B: 1, W: 2, L: 4, Q: 8};

/* ------------------------------------------------------------------------------------------- *
 * Value pools -- edge-weighted, per README's "hot-region input pools" lesson.                     *
 * ------------------------------------------------------------------------------------------- */

function pick(rnd, arr) { return arr[Math.floor(rnd() * arr.length) % arr.length]; }

function randB(rnd) { return (rnd() < 0.4) ? pick(rnd, [0, 1, 0x7F, 0x80, 0xFF, 2, 0x81]) : Math.floor(rnd() * 256); }
function randW(rnd) { return (rnd() < 0.4) ? pick(rnd, [0, 1, 0x7FFF, 0x8000, 0xFFFF, 2, 0x8001]) : Math.floor(rnd() * 65536); }
function randL(rnd)
{
    if (rnd() < 0.4) return pick(rnd, [0, 1, 0x7FFFFFFF, 0x80000000, -1, 2, 0x80000001 | 0, -2]) | 0;
    return (Math.floor(rnd() * 0x100000000)) | 0;
}
function randQ(rnd) { return {lo: randL(rnd), hi: (rnd() < 0.3) ? pick(rnd, [0, -1, 0x80000000 | 0, 0x7FFFFFFF]) : randL(rnd)}; }

/** Divisor pools: heavily bias toward 0 and overflow-triggering combinations. */
function randDivisorB(rnd) { return (rnd() < 0.35) ? pick(rnd, [0, 0xFF, 1]) : randB(rnd); }
function randDivisorW(rnd) { return (rnd() < 0.35) ? pick(rnd, [0, 0xFFFF, 1]) : randW(rnd); }
function randDivisorL(rnd) { return (rnd() < 0.35) ? pick(rnd, [0, -1, 1, 0x80000000 | 0]) : randL(rnd); }

/* ------------------------------------------------------------------------------------------- *
 * Per-case layout                                                                                  *
 *
 *   CASE_STRIDE bytes per case: [0x000..0x03F) instruction bytes, [0x040..0x680) the hot data
 *   region (memory-mode operands, absolute-mode targets, memory-mode bit-field bases),
 *   [0x700..0x780) stack (PUSHL/PUSHAx land at SP-4).  64MB of memory / CASE_STRIDE gives
 *   generous headroom over any --cases-per-opcode this file's coverage floor will accept.
 * ------------------------------------------------------------------------------------------- */

const CASE_STRIDE = 0x800;
const CASE_BASE = 0x00100000;
const MEMSIZE = 0x04000000;             // 64MB
const SP_OFFSET = 0x780;
const HOT_START = 0x040;
const HOT_END = 0x680;

/**
 * @class Case
 *
 * One (opcode, pre-state) instance: instruction bytes, initial register file, initial PSL, and
 * any memory pre-sets (absolute-mode targets, immediate quadword halves).  Shared verbatim
 * between the JS side and the SIMH-script side, so both execute the SAME bytes from the SAME
 * pre-state.
 */
class Case {
    constructor(mnemonic, opc, index)
    {
        this.mnemonic = mnemonic;
        this.opc = opc;
        this.index = index;
        this.base = (CASE_BASE + index * CASE_STRIDE) | 0;
        this.pc = this.base;
        this.instr = [];                     // instruction bytes
        this.mem = [];                        // {addr, lnt, val} pre-sets in the hot region
        this.regs = new Int32Array(16);
        this.psl = 0;
        this.hotCursor = this.base + HOT_START;
        this.trivial = true;                  // set false once a non-edge-case write happens
        this.usedRegs = new Set();            // registers already claimed by an earlier specifier
    }

    /** Reserve `n` bytes of the hot region and return the address. */
    allocHot(n)
    {
        let a = this.hotCursor;
        this.hotCursor = (this.hotCursor + n + 4) | 0;      // pad, keep longword aligned
        this.hotCursor = (this.hotCursor + 3) & ~3;
        if (this.hotCursor > this.base + HOT_END) throw new Error(`case ${this.index}: hot region overflow`);
        return a;
    }

    setMemL(addr, val) { this.mem.push({addr: addr | 0, lnt: 4, val: val | 0}); }
    setMemW(addr, val) { this.mem.push({addr: addr | 0, lnt: 2, val: val & 0xFFFF}); }
    setMemB(addr, val) { this.mem.push({addr: addr | 0, lnt: 1, val: val & 0xFF}); }

    byte(b) { this.instr.push(b & 0xFF); }
    long(v) { v = v | 0; this.instr.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF); }
    word(v) { this.instr.push(v & 0xFF, (v >>> 8) & 0xFF); }

    get instrLen() { return this.instr.length; }
}

/* ------------------------------------------------------------------------------------------- *
 * Specifier emitters -- append bytes to c.instr (and optionally pre-set memory) implementing one   *
 * specifier of a given width.  Every one is a fully-legal, 8c0-proven addressing mode: register     *
 * direct (0x5n), PC-relative immediate (0x8F + literal bytes), or absolute (0x9F + 4-byte           *
 * address) through a preset location in the case's hot region.  A shared register pool (R0-R11)     *
 * is used for register-direct operands; R14/SP and R15/PC are never chosen as a data register.       *
 * ------------------------------------------------------------------------------------------- */

const REG_POOL = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

/**
 * pickFreeReg(c, rnd, maxIncl, needsPair)
 *
 * A register not already claimed by an earlier specifier of the SAME instruction, so that (e.g.)
 * a divisor specifier choosing R5 register-direct can never be silently clobbered by a later
 * specifier that also happens to land on R5 -- both sides of the differential would still agree
 * with each other in that case (they read the identical, if accidental, Case), which is exactly
 * why it is dangerous: it manufactures values (and occasionally faulting bit-field positions)
 * neither side intended, rather than failing loudly.  `needsPair` reserves rn+1 too, for a
 * register-mode quadword or a `.vb` base whose vfldrp1 partner is rn+1.
 *
 * @param {Case} c
 * @param {function(): number} rnd
 * @param {number} maxIncl
 * @param {boolean} [needsPair]
 * @returns {number}
 */
function pickFreeReg(c, rnd, maxIncl, needsPair)
{
    let candidates = REG_POOL.filter((r) => r <= maxIncl && !c.usedRegs.has(r) && (!needsPair || !c.usedRegs.has(r + 1)));
    if (!candidates.length) candidates = REG_POOL.filter((r) => r <= maxIncl);  // pool exhausted (rare): allow reuse rather than throw
    let rn = pick(rnd, candidates);
    c.usedRegs.add(rn);
    if (needsPair) c.usedRegs.add(rn + 1);
    return rn;
}

/**
 * emitRead(c, rnd, width, value) -- an R-type specifier whose value is `value` (an int, or
 * {lo,hi} for width 8).  Randomly register-direct or PC-relative immediate.
 */
function emitRead(c, rnd, width, value)
{
    let asReg = rnd() < 0.5;
    if (asReg && width <= 4) {
        let rn = pickFreeReg(c, rnd, 13, false);
        c.byte(0x50 | rn);
        c.regs[rn] = width === 1 ? (value & 0xFF) : width === 2 ? (value & 0xFFFF) : (value | 0);
        return;
    }
    if (asReg && width === 8) {
        let rn = pickFreeReg(c, rnd, 12, true);
        c.byte(0x50 | rn);
        c.regs[rn] = value.lo | 0;
        c.regs[rn + 1] = value.hi | 0;
        return;
    }
    /* PC-relative immediate: mode 0x8F, then the literal in the instruction stream. */
    c.byte(0x8F);
    if (width === 1) c.byte(value & 0xFF);
    else if (width === 2) c.word(value & 0xFFFF);
    else if (width === 4) c.long(value | 0);
    else { c.long(value.lo | 0); c.long(value.hi | 0); }
}

/**
 * emitModify(c, rnd, width, value) -- an M-type specifier: register-direct, or absolute through a
 * preset hot-region address (exercising the memory-destination store path).
 * @returns {boolean} true if a memory destination was chosen
 */
function emitModify(c, rnd, width, value)
{
    let asReg = rnd() < 0.42;
    if (asReg) {
        let rn = pickFreeReg(c, rnd, width === 8 ? 12 : 13, width === 8);
        c.byte(0x50 | rn);
        if (width === 8) { c.regs[rn] = value.lo | 0; c.regs[rn + 1] = value.hi | 0; }
        else c.regs[rn] = width === 1 ? (value & 0xFF) : width === 2 ? (value & 0xFFFF) : (value | 0);
        return false;
    }
    let a = c.allocHot(width === 8 ? 8 : 4);
    c.byte(0x9F);
    c.long(a);
    if (width === 8) { c.setMemL(a, value.lo); c.setMemL((a + 4) | 0, value.hi); }
    else if (width === 1) c.setMemB(a, value);
    else if (width === 2) c.setMemW(a, value);
    else c.setMemL(a, value);
    return true;
}

/**
 * emitWrite(c, rnd, width) -- a W/A-type specifier: register-direct destination, or absolute
 * through a fresh hot-region address (no preset value -- it is the destination).
 * @returns {{mem: boolean, rn: number, addr: number}}
 */
function emitWrite(c, rnd, width, forceMem)
{
    let asReg = !forceMem && rnd() < 0.42;
    if (asReg) {
        let rn = pickFreeReg(c, rnd, width === 8 ? 12 : 13, width === 8);
        c.byte(0x50 | rn);
        return {mem: false, rn: rn, addr: 0};
    }
    let a = c.allocHot(width === 8 ? 8 : 4);
    c.byte(0x9F);
    c.long(a);
    return {mem: true, rn: -1, addr: a};
}

/* ------------------------------------------------------------------------------------------- *
 * Per-opcode case generators.  Most opcodes fall through the GENERIC path, driven purely by       *
 * drom.js's specifier-type list; the handful with architectural legality constraints (bit-field     *
 * pos/size, shift counts, divisors, ADAWI's alignment) get a targeted generator.                    *
 * ------------------------------------------------------------------------------------------- */

const SPECIAL = {};

function genGeneric(mnemonic, opc, rnd, index)
{
    let c = new Case(mnemonic, opc, index);
    if (opc >= 0x100) c.byte(0xFD);
    c.byte(opc & 0xFF);
    let types = specTypes(opc);
    for (let t of types) {
        let kind = t[0];                        // R, M, A, W
        let width = WIDTH_OF[t.slice(1)] || 4;
        if (kind === "R") {
            let val = width === 1 ? randB(rnd) : width === 2 ? randW(rnd) : width === 4 ? randL(rnd) : randQ(rnd);
            emitRead(c, rnd, width, val);
        } else if (kind === "M") {
            let val = width === 1 ? randB(rnd) : width === 2 ? randW(rnd) : width === 4 ? randL(rnd) : randQ(rnd);
            let m = emitModify(c, rnd, width, val);
            if (m) c.trivial = false;
        } else if (kind === "W" || kind === "A") {
            /* An "A" (address) specifier has NO register form (MOVAB/MOVAL/PUSHAx's source) --
               forcing memory-only here is a legality requirement, not a coverage choice; using
               emitWrite's normal 50/50 register/memory split for it faulted RESAD on the SIMH
               side (uncaught, since it is architecturally illegal, not a scope-excluded fault). */
            let m = emitWrite(c, rnd, width, kind === "A");
            if (m.mem) c.trivial = false;
        } else {
            throw new Error(`genGeneric: ${mnemonic} has an unexpected specifier type ${t}`);
        }
    }
    return c;
}

/** ADAWI: RW,WW -- the WW destination must be aligned when it is memory, or SIMH RESOP-faults. */
SPECIAL.ADAWI = (opc, rnd, index) => {
    let c = new Case("ADAWI", opc, index);
    c.byte(opc & 0xFF);
    let val = randW(rnd);
    emitRead(c, rnd, 2, val);
    let asReg = rnd() < 0.5;
    if (asReg) {
        let rn = pickFreeReg(c, rnd, 13, false);
        c.byte(0x50 | rn);
        c.regs[rn] = randW(rnd);
    } else {
        let a = c.allocHot(4) & ~1;              // force alignment
        c.byte(0x9F);
        c.long(a);
        c.setMemW(a, randW(rnd));
        c.trivial = false;
    }
    return c;
};

/** ROTL: RB,RL,WL -- shift count is unconstrained (any byte value is legal). */
SPECIAL.ROTL = (opc, rnd, index) => {
    let c = new Case("ROTL", opc, index);
    c.byte(opc & 0xFF);
    emitRead(c, rnd, 1, (rnd() < 0.3) ? pick(rnd, [0, 1, 31, 32, 0xFF]) : randB(rnd));
    emitRead(c, rnd, 4, randL(rnd));
    let m = emitWrite(c, rnd, 4, false);
    if (m.mem) c.trivial = false;
    return c;
};

/** ASHL: RB,RL,WL -- shift count unconstrained (0x80|n selects a right shift of n). */
SPECIAL.ASHL = (opc, rnd, index) => {
    let c = new Case("ASHL", opc, index);
    c.byte(opc & 0xFF);
    let sc = pick(rnd, [0, 1, 4, 31, 0x80, 0x81, 0x9F, 0xE0, 0xFF, Math.floor(rnd() * 256)]);
    emitRead(c, rnd, 1, sc);
    emitRead(c, rnd, 4, randL(rnd));
    let m = emitWrite(c, rnd, 4, false);
    if (m.mem) c.trivial = false;
    return c;
};

/** ASHQ: RB,RQ,WQ. */
SPECIAL.ASHQ = (opc, rnd, index) => {
    let c = new Case("ASHQ", opc, index);
    c.byte(opc & 0xFF);
    let sc = pick(rnd, [0, 1, 4, 63, 0x80, 0x81, 0xBF, 0xC0, 0xFF, Math.floor(rnd() * 256)]);
    emitRead(c, rnd, 1, sc);
    emitRead(c, rnd, 8, randQ(rnd));
    let m = emitWrite(c, rnd, 8, false);
    if (m.mem) c.trivial = false;
    return c;
};

/** DIVx2/3, EDIV: specifier 0 is the divisor -- bias toward 0 and the overflow pair. */
function genDiv(width, isL3, randDiv)
{
    return (opc, rnd, index) => {
        let mnemonic = OPCODES[opc];
        let c = new Case(mnemonic, opc, index);
        c.byte(opc & 0xFF);
        let divisor = randDiv(rnd);
        emitRead(c, rnd, width, divisor);
        let types = specTypes(opc);
        if (types[1][0] === "M") {
            let val = width === 1 ? randB(rnd) : width === 2 ? randW(rnd) : randL(rnd);
            let m = emitModify(c, rnd, width, val);
            if (m) c.trivial = false;
        } else {
            let val = width === 1 ? randB(rnd) : width === 2 ? randW(rnd) : randL(rnd);
            emitRead(c, rnd, width, val);
            let m = emitWrite(c, rnd, width, false);
            if (m.mem) c.trivial = false;
        }
        return c;
    };
}
SPECIAL.DIVB2 = genDiv(1, false, randDivisorB);
SPECIAL.DIVB3 = genDiv(1, true, randDivisorB);
SPECIAL.DIVW2 = genDiv(2, false, randDivisorW);
SPECIAL.DIVW3 = genDiv(2, true, randDivisorW);
SPECIAL.DIVL2 = genDiv(4, false, randDivisorL);
SPECIAL.DIVL3 = genDiv(4, true, randDivisorL);

/** EDIV: RL(divisor),RQ(dividend),WL(quo),WL(rem). */
SPECIAL.EDIV = (opc, rnd, index) => {
    let c = new Case("EDIV", opc, index);
    c.byte(opc & 0xFF);
    let divisor = randDivisorL(rnd);
    emitRead(c, rnd, 4, divisor);
    /* Bias the dividend toward magnitudes comparable to the divisor, so the "quotient fits in
       32 bits" branch is not overwhelmingly the common case. */
    let dvd = (rnd() < 0.5) ? randQ(rnd) : {lo: randL(rnd), hi: pick(rnd, [0, -1])};
    emitRead(c, rnd, 8, dvd);
    let m1 = emitWrite(c, rnd, 4, false);
    if (m1.mem) c.trivial = false;
    let m2 = emitWrite(c, rnd, 4, false);
    if (m2.mem) c.trivial = false;
    return c;
};

/** EMUL: RL,RL,RL,WQ. */
SPECIAL.EMUL = (opc, rnd, index) => {
    let c = new Case("EMUL", opc, index);
    c.byte(opc & 0xFF);
    emitRead(c, rnd, 4, randL(rnd));
    emitRead(c, rnd, 4, randL(rnd));
    emitRead(c, rnd, 4, randL(rnd));
    let m = emitWrite(c, rnd, 8, false);
    if (m.mem) c.trivial = false;
    return c;
};

/** BISPSW/BICPSW: RW -- must have PSW<15:8> clear (PSW_MBZ) or SIMH RESOP-faults. */
function genPSW(setBits)
{
    return (opc, rnd, index) => {
        let mnemonic = OPCODES[opc];
        let c = new Case(mnemonic, opc, index);
        c.byte(opc & 0xFF);
        let val = Math.floor(rnd() * 0x100) & 0xFF;    // low byte only: CC + DV/FU/IV/T, never MBZ
        emitRead(c, rnd, 2, val);
        c.psl = c.psl | (Math.floor(rnd() * 16) << 4);  // randomize initial DV/FU/IV/T too
        return c;
    };
}
SPECIAL.BISPSW = genPSW(true);
SPECIAL.BICPSW = genPSW(false);

/** Bit-field family: pos.rl, size.rb, base.vb [, dst2 or src2]. Keep size in 0..32 always (or
    RESOP faults); when the base is register mode, keep pos in 0..31 and pos+size<=32 (or RESAD /
    RESOP fault, since the SCB is not set up in this harness). */
/**
 * genBitfield(trailer) -- `trailer` is what follows base.vb in the specifier list:
 *   "write"  EXTV/EXTZV/FFS/FFC: pos.rl, size.rb, base.vb, dst.wl  (base is NOT last)
 *   "read"   CMPV/CMPZV:         pos.rl, size.rb, base.vb, src2.rl (base is NOT last)
 *   "none"   INSV:               src.rl, pos.rl, size.rb, base.vb (base IS last -- op_insv
 *            writes through the SAME base.vb it read, there is no separate destination
 *            specifier; emitting one here was a real bug that fed SIMH one specifier too many
 *            and desynced every case after it in the instruction stream length check)
 */
function genBitfield(trailer)
{
    return (opc, rnd, index) => {
        let mnemonic = OPCODES[opc];
        let c = new Case(mnemonic, opc, index);
        c.byte(opc & 0xFF);
        if (mnemonic === "INSV") emitRead(c, rnd, 4, randL(rnd));       // INSV's src field, FIRST
        let asRegBase = rnd() < 0.5;
        let size = pick(rnd, [0, 1, 8, 16, 17, 31, 32, Math.floor(rnd() * 33)]);
        let pos;
        if (asRegBase) pos = Math.floor(rnd() * (33 - size));       // keeps pos+size<=32
        else pos = pick(rnd, [0, 1, 7, 8, 31, 32, Math.floor(rnd() * 900)]);
        emitRead(c, rnd, 4, pos);
        emitRead(c, rnd, 1, size);
        /* base.vb */
        if (asRegBase) {
            let rn = pickFreeReg(c, rnd, 12, true);
            c.byte(0x50 | rn);
            c.regs[rn] = randL(rnd);
            c.regs[rn + 1] = randL(rnd);
        } else {
            let a = c.allocHot(8);
            c.byte(0x9F);
            c.long(a);
            c.setMemL(a, randL(rnd));
            c.setMemL((a + 4) | 0, randL(rnd));
            c.trivial = false;
        }
        if (trailer === "read") emitRead(c, rnd, 4, randL(rnd));        // CMPV/CMPZV's src2
        else if (trailer === "write") {
            let m = emitWrite(c, rnd, 4, false);
            if (m.mem) c.trivial = false;
        }
        /* trailer === "none" (INSV): base.vb was the last specifier, nothing more to emit. */
        return c;
    };
}
SPECIAL.EXTV = genBitfield("write");
SPECIAL.EXTZV = genBitfield("write");
SPECIAL.FFS = genBitfield("write");
SPECIAL.FFC = genBitfield("write");
SPECIAL.CMPV = genBitfield("read");
SPECIAL.CMPZV = genBitfield("read");
SPECIAL.INSV = genBitfield("none");

/**
 * genCase(mnemonic, rnd, index)
 * @returns {Case}
 */
function genCase(mnemonic, rnd, index)
{
    let opc = OPC_OF[mnemonic];
    let c = SPECIAL[mnemonic] ? SPECIAL[mnemonic](opc, rnd, index) : genGeneric(mnemonic, opc, rnd, index);
    /* Common finishing touches every case needs regardless of family. */
    c.regs[15] = c.pc;
    c.regs[14] = (c.base + SP_OFFSET) | 0;
    /* Noise for registers NO specifier claimed -- c.usedRegs (set by pickFreeReg) is the only
       reliable signal that a register's value is meaningful; testing `=== 0` here would clobber
       a deliberately-zero constrained value (e.g. a bit-field size=0 or a divisor=0) that a
       SPECIAL generator set on purpose, corrupting the case into a state its own generator
       explicitly avoided (this WAS a real bug: it turned "size=0" into size=random-byte, which
       both sides still agreed on but which routinely violated the size<=32 legality this
       generator exists to guarantee, faulting SIMH's uninitialized SCB and corrupting every case
       after it in the same batch). */
    for (let r = 0; r < 14; r++) if (!c.usedRegs.has(r) && rnd() < 0.7) c.regs[r] = randL(rnd);
    let cc = Math.floor(rnd() * 16);
    c.psl = (c.psl & ~0xF) | cc;
    c.expectPCAfter = (c.pc + c.instr.length) | 0;
    return c;
}

/* ------------------------------------------------------------------------------------------- *
 * JS-side execution                                                                                *
 * ------------------------------------------------------------------------------------------- */

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new VAXCpu(bus);
    return {bus, cpu};
}

/**
 * runCaseJS(m, c) -- returns {regs: Int32Array(16), psl: number, pcOK: boolean, error: string|null}
 */
function runCaseJS(m, c)
{
    let {bus, cpu} = m;
    cpu.regs.fill(0);
    for (let i = 0; i < 16; i++) cpu.regs[i] = c.regs[i];
    cpu.psl = c.psl;
    for (let i = 0; i < c.instr.length; i++) bus.setByte((c.pc + i) | 0, c.instr[i]);
    for (let w of c.mem) {
        if (w.lnt === 4) bus.setLong(w.addr, w.val);
        else if (w.lnt === 2) bus.setWord(w.addr, w.val);
        else bus.setByte(w.addr, w.val);
    }
    try {
        cpu.step();
    } catch (e) {
        return {regs: null, psl: 0, pcOK: false, error: e.message || String(e)};
    }
    let pcOK = cpu.regs[15] === c.expectPCAfter;
    return {regs: Int32Array.from(cpu.regs), psl: cpu.psl, pcOK, error: null};
}

/* ------------------------------------------------------------------------------------------- *
 * SIMH-side batch driver.  One script drives MANY cases (deposit/step/examine per case,             *
 * delimited by `echo` markers) to amortize process-start overhead; each case is fully self-          *
 * contained (every register and every memory byte it touches is deposited fresh), so cases do       *
 * not need a `reset` between them and cannot interfere with each other -- CASE_STRIDE keeps their     *
 * address ranges disjoint.                                                                            *
 * ------------------------------------------------------------------------------------------- */

const CASE_MARK = "CASE_";

function buildScript(cases, memSizeM)
{
    /* SIMHALT (not the default CONHALT): a HALT -- or a fault whose SCB vector reads garbage
       physical 0 and lands on the HALT opcode 0x00, since this harness's memory has no real SCB
       -- stops the simulator instead of transferring to the console ROM.  CONHALT was the root
       cause of an entire batch's worth of cases silently corrupting: once one case's execution
       (deliberately or via a generator bug) reached the console ROM, every subsequent
       deposit-PC/step/examine in the SAME simh process kept re-running the console instead of the
       next case's instruction, and its wildly different register file was compared as if it were
       that case's real result. */
    let L = [`set cpu ${memSizeM}m`, "set cpu simhalt", "reset all"];
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        /* A pending deferred trap (TRPIRQ -- integer overflow, integer divide-by-zero) is a
           simulator-internal variable no `deposit R*`/`deposit PSL` reaches, so a case that left
           one pending (which the divisor pools deliberately provoke, on purpose, per the file
           header) would otherwise be silently dispatched at the TOP of the NEXT case's `step 1`,
           corrupting a case that has nothing to do with the one that set it (an 8/12-byte
           exception-frame push showing up as an unrelated SP/register mismatch).  Clearing it
           here every case is what makes each case truly independent despite sharing one simh
           process across a whole batch. */
        L.push("deposit TRPIRQ 0");
        L.push("deposit SISR 0");
        /* R15 is not a depositable/examinable register name on this console -- it is PC. */
        for (let r = 0; r < 15; r++) L.push(`deposit R${r} ${hex(c.regs[r])}`);
        L.push(`deposit PSL ${hex(c.psl)}`);
        for (let i = 0; i < c.instr.length; i++) L.push(`deposit -b ${hex(c.pc + i)} ${c.instr[i].toString(16)}`);
        for (let w of c.mem) {
            if (w.lnt === 4) L.push(`deposit ${hex(w.addr)} ${hex(w.val)}`);
            else if (w.lnt === 2) L.push(`deposit -w ${hex(w.addr)} ${(w.val & 0xFFFF).toString(16)}`);
            else L.push(`deposit -b ${hex(w.addr)} ${(w.val & 0xFF).toString(16)}`);
        }
        L.push(`deposit PC ${hex(c.pc)}`);
        L.push("step 1");
        let regList = [];
        for (let r = 0; r < 15; r++) regList.push("R" + r);
        regList.push("PC");
        regList.push("PSL");
        L.push(`examine -h ${regList.join(",")}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

const REG_LINE_RE = /^(R\d{1,2}|PSL|PC):\s+([0-9A-Fa-f]+)/;

/**
 * runBatch(simh, cases, scratch, memSizeM)
 *
 * @returns {Map<number, {regs: Int32Array, psl: number, reached: boolean}>} keyed by case.index
 */
function runBatch(simh, cases, scratch, memSizeM)
{
    let script = buildScript(cases, memSizeM);
    let out = runSimh(simh, script, path.join(scratch, "intdiff-batch.ini"));
    if (process.env.INTDIFF_DUMP) fs.writeFileSync(`${process.env.INTDIFF_DUMP}.${cases[0].index}`, out);
    let lines = out.split("\n");
    let results = new Map();
    let i = 0;
    let n = lines.length;
    while (i < n) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\d+)"));
        if (!m) { i++; continue; }
        let idx = +m[1];
        i++;
        let regs = new Int32Array(16), psl = 0, got = 0;
        /* Scan forward until we've collected 17 register/PSL lines or hit the next case marker. */
        while (i < n && got < 17) {
            if (lines[i].indexOf(CASE_MARK) >= 0) break;
            let rm = lines[i].match(REG_LINE_RE);
            if (rm) {
                let name = rm[1], val = parseInt(rm[2], 16) | 0;
                if (name === "PSL") { psl = val; got++; }
                else if (name === "PC") { regs[15] = val; got++; }
                else { let rn = parseInt(name.slice(1), 10); regs[rn] = val; got++; }
            }
            i++;
        }
        results.set(idx, {regs, psl, reached: got >= 17});
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * Comparison                                                                                        *
 * ------------------------------------------------------------------------------------------- */

function compare(mnemonic, c, js, simhResult)
{
    if (js.error) return `${mnemonic} case ${c.index}: cpu.js threw during execution: ${js.error}`;
    if (!simhResult || !simhResult.reached) return `${mnemonic} case ${c.index}: SIMH result not reached (case dropped from comparison -- see COVERAGE)`;
    if (!js.pcOK) return `${mnemonic} case ${c.index}: cpu.js PC after step = ${hex(js.regs[15])}, expected ${hex(c.expectPCAfter)} (execution-time fault in a case that should not have faulted?)`;
    for (let r = 0; r < 16; r++) {
        if (js.regs[r] !== simhResult.regs[r]) {
            return `${mnemonic} case ${c.index}: R${r} mismatch: cpu.js=${hex(js.regs[r])} simh=${hex(simhResult.regs[r])} (instr=${c.instr.map((b) => b.toString(16).padStart(2, "0")).join(" ")})`;
        }
    }
    if ((js.psl & 0xF) !== (simhResult.psl & 0xF)) {
        return `${mnemonic} case ${c.index}: CC mismatch: cpu.js=${(js.psl & 0xF).toString(2).padStart(4, "0")} simh=${(simhResult.psl & 0xF).toString(2).padStart(4, "0")} (instr=${c.instr.map((b) => b.toString(16).padStart(2, "0")).join(" ")})`;
    }
    if ((js.psl >>> 4) !== (simhResult.psl >>> 4)) {
        return `${mnemonic} case ${c.index}: PSL (non-CC) mismatch: cpu.js=${hex(js.psl)} simh=${hex(simhResult.psl)}`;
    }
    return null;
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors                                                                                    *
 * ------------------------------------------------------------------------------------------- */

const MIN_CASES_PER_OPCODE = 40;
const MIN_TOTAL_CASES = 4000;
const MIN_NONTRIVIAL_FRACTION = 0.5;
const MIN_MEM_DEST_FRACTION = 0.15;

/* ------------------------------------------------------------------------------------------- *
 * RANDOMIZED phase                                                                                  *
 * ------------------------------------------------------------------------------------------- */

function phaseRandomized(simh, scratch, opts)
{
    let rnd = mulberry32(opts.seed);
    let mnemonics = IMPLEMENTED.slice().sort();
    let allCases = [];
    let byMnemonic = new Map();
    let index = 0;
    for (let mn of mnemonics) {
        let list = [];
        for (let k = 0; k < opts.casesPerOpcode; k++) {
            let c = genCase(mn, rnd, index++);
            list.push(c);
            allCases.push(c);
        }
        byMnemonic.set(mn, list);
    }

    let memSizeM = Math.max(64, Math.ceil((allCases.length * CASE_STRIDE) / (1024 * 1024)) + 8);

    let failures = [];
    let notReached = [];
    let nontrivial = 0, memDest = 0, total = 0;
    const BATCH = 500;
    let m = makeMachine();
    for (let start = 0; start < allCases.length; start += BATCH) {
        let batch = allCases.slice(start, start + BATCH);
        let simhResults = runBatch(simh, batch, scratch, memSizeM);
        for (let c of batch) {
            total++;
            if (!c.trivial) nontrivial++;
            if (c.mem.length) memDest++;
            let js = runCaseJS(m, c);
            let sr = simhResults.get(c.index);
            if (!sr || !sr.reached) { notReached.push(`${c.mnemonic} case ${c.index}`); continue; }
            let bad = compare(c.mnemonic, c, js, sr);
            if (bad) failures.push(bad);
            if (bad && process.env.INTDIFF_DEBUG && c.mnemonic === process.env.INTDIFF_DEBUG) {
                console.error("DEBUG case", c.index, c.mnemonic, "regs=", Array.from(c.regs).map((v) => hex(v)).join(","), "psl=", hex(c.psl), "instr=", c.instr.map((b)=>b.toString(16).padStart(2,"0")).join(" "), "mem=", JSON.stringify(c.mem), "bad=", bad, "jsregs=", js.regs?Array.from(js.regs).map((v)=>hex(v)).join(","):null, "simhregs=", sr?Array.from(sr.regs).map((v)=>hex(v)).join(","):null);
            }
        }
    }

    let perOpcodeCounts = new Map();
    for (let c of allCases) perOpcodeCounts.set(c.mnemonic, (perOpcodeCounts.get(c.mnemonic) || 0) + 1);

    return {
        total, nontrivial, memDest, failures, notReached,
        mnemonics, perOpcodeCounts
    };
}

/* ------------------------------------------------------------------------------------------- *
 * EHKAA real-workload phase                                                                        *
 * ------------------------------------------------------------------------------------------- */

function parseTraceForOpcodes(pathTrace, wanted, onRecord)
{
    const fd = fs.openSync(pathTrace, "r");
    const CHUNK = 1 << 22;
    let buf = Buffer.allocUnsafe(CHUNK);
    let carry = "";
    let rec = null, nRecords = 0, lineno = 0;
    const INSTR_LINE_RE = /^([0-9A-Fa-f]{8}) ([0-9A-Fa-f]{8})\| (.*)$/;

    const fields = (line) => { let sp = line.indexOf(" "); return sp < 0 ? [] : line.slice(sp + 1).split(" "); };

    const handleLine = (line) => {
        lineno++;
        let m = INSTR_LINE_RE.exec(line);
        if (m) {
            if (rec) throw new Error(`${pathTrace}:${lineno}: new record before previous completed`);
            let mnemonic = m[3].split(" ", 1)[0];
            rec = {
                index: nRecords, pc: parseInt(m[1], 16) | 0, psl: parseInt(m[2], 16) | 0,
                mnemonic, want: wanted.has(mnemonic)
            };
            return;
        }
        if (!rec) return;
        if (line.startsWith("REGS ")) { rec.reg = fields(line).map((t) => parseInt(t, 16) | 0); return; }
        if (line.startsWith("PREG ")) { rec.preg = fields(line).map((t) => parseInt(t, 16) | 0); return; }
        if (line.startsWith("IBYT ")) {
            let f = fields(line);
            rec.ilen = +f[0];
            rec.ibyt = Uint8Array.from(f.slice(1).map((t) => parseInt(t, 16)));
            return;
        }
        if (line.startsWith("OPND ")) { return; }
        if (line.startsWith("RECQ ")) { return; }
        if (line.startsWith("MEMR ")) {
            let f = fields(line);
            rec.memr = [];
            for (let i = 1; i + 3 < f.length; i += 4) {
                rec.memr.push({va: parseInt(f[i], 16) | 0, lnt: +f[i + 1], val: parseInt(f[i + 2], 16) | 0, w: +f[i + 3]});
            }
            return;
        }
        if (line.startsWith("BRDP ")) {
            nRecords++;
            onRecord(rec);
            rec = null;
            return;
        }
    };

    for (;;) {
        let n = fs.readSync(fd, buf, 0, CHUNK, null);
        if (n <= 0) break;
        let text = carry + buf.toString("latin1", 0, n);
        let start = 0;
        for (;;) {
            let nl = text.indexOf("\n", start);
            if (nl < 0) break;
            let line = text.slice(start, nl);
            if (line.endsWith("\r")) line = line.slice(0, -1);
            handleLine(line);
            start = nl + 1;
        }
        carry = text.slice(start);
    }
    fs.closeSync(fd);
    if (carry.length) handleLine(carry);
    return nRecords;
}

/** A tiny VAXDecodeMachine that replays MEMR-logged reads, exactly like decodediff.js's
    ReplayMachine (the same discipline: an unmatched or mismatched read is a hard failure). */
class ReplayMachine {
    constructor() { this.regs = new Int32Array(16); }
    load(rec) { this.regs.set(rec.preg); this.iPC = rec.pc; this.ibyt = rec.ibyt; this.memr = rec.memr || []; this.memrIdx = 0; }
    getISTR(lnt)
    {
        let off = (this.regs[15] - this.iPC) | 0;
        if (off < 0 || off + lnt > this.ibyt.length) throw new Error("getISTR out of range");
        let v = 0;
        for (let i = lnt - 1; i >= 0; i--) v = (v << 8) | this.ibyt[off + i];
        this.regs[15] = (this.regs[15] + lnt) | 0;
        return lnt === 4 ? (v | 0) : v;
    }
    setPC(pc) { this.regs[15] = pc | 0; }
    readData(va, lnt, fWrite)
    {
        let e = this.memr[this.memrIdx];
        if (!e) throw new Error(`decoder issued an unlogged read (va=${hex(va)} lnt=${lnt})`);
        if (e.va !== (va | 0) || e.lnt !== lnt || e.w !== (fWrite ? 1 : 0)) throw new Error("read log mismatch");
        this.memrIdx++;
        return e.val;
    }
}

/** A cpu.js-shaped facade around ReplayMachine, so H[] handlers run unmodified. In decode-time
    the replay machine drives; once decode() returns, execution proceeds through the SAME
    register file and a REAL bus (only used by memory-mode bit-field/ADAWI reads/writes, which
    are excluded from this phase -- see notMemBacked below). */
function phaseEHKAA(opts)
{
    let ehkaaPath = opts.ehkaaExe;
    if (!fs.existsSync(ehkaaPath)) {
        return {skipped: true, reason: `EHKAA diagnostic not found at ${ehkaaPath}`};
    }
    let simh = opts.simh;
    let tracePath = path.join(opts.scratch, "intdiff-ehkaa.trace");
    let wanted = new Set(IMPLEMENTED);
    let script = [
        `set cpu 32m`,
        `set -d cpu history=100000:${tracePath}`,
        `load ${ehkaaPath}`,
        `go -q 200`
    ].join("\n") + "\n";
    runSimh(simh, script, path.join(opts.scratch, "intdiff-ehkaa.ini"));

    let records = [];
    let total = parseTraceForOpcodes(tracePath, wanted, (rec) => { if (rec.want) records.push(rec); });

    let failures = [];
    let skippedMemVB = 0, skippedTrap = 0, compared = 0;
    let perOpcode = new Map();

    /* Re-parse the trace a second time to build a pc -> nextRecord map cheaply: since records are
       already filtered to `want` above but we need the IMMEDIATELY FOLLOWING record (any opcode)
       for the after-state, collect ALL records' (pc, ilen, reg, psl) in a compact array first. */
    let allSeq = [];
    parseTraceForOpcodes(tracePath, new Set(), (rec) => {
        allSeq.push({pc: rec.pc, ilen: rec.ilen, psl: rec.psl, mnemonic: rec.mnemonic, memr: rec.memr, ibyt: rec.ibyt, preg: rec.preg});
    });

    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, 0x100000, MemoryVAX.TYPE.RAM);
    let cpu = new VAXCpu(bus);
    let replay = new ReplayMachine();
    cpu.decoder.cpu = replay;                                    // decode phase reads through the replay log
    let realGetISTR = replay.getISTR.bind(replay);

    for (let i = 0; i < allSeq.length - 1; i++) {
        let rec = allSeq[i];
        if (!wanted.has(rec.mnemonic)) continue;
        let next = allSeq[i + 1];
        if (next.pc !== (rec.pc + rec.ilen | 0)) { skippedTrap++; continue; }

        replay.load({preg: rec.preg, pc: rec.pc, ibyt: rec.ibyt, memr: rec.memr || []});
        cpu.decoder.cpu = replay;
        let d = cpu.decoder;
        try {
            d.decode(false);
        } catch (e) {
            failures.push(`EHKAA ${rec.mnemonic} @${hex(rec.pc)}: decode replay failed: ${e.message}`);
            continue;
        }
        /* Memory-mode variable-bit-field specifiers need an execution-time read the MEMR log
           does not contain (SIMH's read-log arms only during specifier resolution) -- skip,
           counted, not silently dropped. */
        let mn = rec.mnemonic;
        let isVB = (mn === "EXTV" || mn === "EXTZV" || mn === "CMPV" || mn === "CMPZV" ||
                    mn === "FFS" || mn === "FFC" || mn === "INSV");
        if (isVB) {
            let rnIdx = (mn === "INSV") ? 3 : 2;
            if (d.opnd[rnIdx] === -1) { skippedMemVB++; continue; }
        }
        if (mn === "ADAWI" && d.opnd[1] === -1) { skippedMemVB++; continue; }   // memory ADAWI needs an execute-time interlocked read too

        /* Splice the replay-derived register file into a REAL cpu (with real, writable memory)
           for execution, so register-destination stores land where the decoder's `va`/`rn`
           expect. Memory destinations are exercised by the randomized phase; EHKAA's memory
           image is not reproduced here (only reads were logged), so a memory-destination store
           in this phase writes to (initially zero) scratch memory -- harmless, since only the
           register file and PSL are compared. */
        cpu.regs.set(replay.regs);
        cpu.psl = rec.psl;
        try {
            cpu.execute();
        } catch (e) {
            failures.push(`EHKAA ${mn} @${hex(rec.pc)}: execution threw: ${e.message}`);
            continue;
        }
        compared++;
        perOpcode.set(mn, (perOpcode.get(mn) || 0) + 1);
        /* Ground truth is next.PREG (patch 0002), captured BEFORE instruction i+1's own specifier
           resolution -- i.e. exactly "the register file right after instruction i finished".
           next.REG (patch 0001) is captured AFTER i+1's resolution and so already carries i+1's
           OWN autoincrement/autodecrement side effects; comparing against it was a real bug here
           (a consistent off-by-{width} divergence on every instruction immediately followed by
           another autoincrement/autodecrement-addressed one, which a loop-heavy diagnostic like
           EHKAA hits constantly). */
        for (let r = 0; r < 16; r++) {
            if (r === 15) continue;                    // PC already verified equal via the ilen check
            if (cpu.regs[r] !== next.preg[r]) {
                failures.push(`EHKAA ${mn} @${hex(rec.pc)}: R${r} mismatch: cpu.js=${hex(cpu.regs[r])} simh=${hex(next.preg[r])}`);
                break;
            }
        }
        if ((cpu.psl & 0xF) !== (next.psl & 0xF)) {
            failures.push(`EHKAA ${mn} @${hex(rec.pc)}: CC mismatch: cpu.js=${(cpu.psl & 0xF).toString(2)} simh=${(next.psl & 0xF).toString(2)}`);
        }
    }

    return {skipped: false, total, compared, skippedMemVB, skippedTrap, failures, perOpcode};
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck: mutate cpu.js's own HANDLERS table in-process and confirm the differential          *
 * catches every mutation.  This is the shipped dispatch table, not a copy.                          *
 * ------------------------------------------------------------------------------------------- */

function selfcheck(simh, scratch)
{
    let rnd = mulberry32(0xC0FFEE);
    let mutations = [
        {mn: "ADDL2", break: (H) => { let orig = H.ADDL2; H.ADDL2 = (cpu, d) => { let save = d.opnd[0]; d.opnd[0] = (d.opnd[0] + 1) | 0; orig(cpu, d); d.opnd[0] = save; }; }},
        {mn: "BICL2", break: (H) => { H.BICL2 = H.BISL2; }},                          // wrong logical op entirely
        {mn: "CMPL", break: (H) => { H.CMPL = (cpu, d) => { cpu.setCC(0); }; }},        // condition codes always 0
        {mn: "INCB", break: (H) => { let orig = H.INCB; H.INCB = (cpu, d) => { orig(cpu, d); }; H.INCB = H.DECB; }},   // INCB behaves like DECB
        {mn: "MULL2", break: (H) => { H.MULL2 = (cpu, d) => { let r = (d.opnd[0] * d.opnd[1]) | 0; require_store(cpu, d, r); }; }},
        {mn: "EDIV", break: (H) => { H.EDIV = () => {}; }},                            // no-op: leaves quo/rem/cc untouched
        {mn: "ASHL", break: (H) => { let orig = H.ASHL; H.ASHL = (cpu, d) => { d.opnd[0] = (d.opnd[0] ^ 0x80) & 0xFF; orig(cpu, d); }; }},  // flip shift direction
        {mn: "EXTV", break: (H) => { H.EXTV = H.EXTZV; }},                             // drops sign extension
    ];

    function require_store(cpu, d, r) { if (d.isMemoryDestination()) cpu.mmu.writeData(d.va, r, 4, cpu.accW()); else cpu.regs[d.rn] = r; cpu.setCC(0); }

    let results = [];
    for (let mut of mutations) {
        let saved = HANDLERS[mut.mn];
        let savedDispatch = DISPATCH[OPC_OF[mut.mn]];
        mut.break(HANDLERS);
        DISPATCH[OPC_OF[mut.mn]] = HANDLERS[mut.mn];

        let m = makeMachine();
        let caught = false;
        let lastFail = null;
        for (let k = 0; k < 60; k++) {
            let c = genCase(mut.mn, rnd, 900000 + k);
            let js = runCaseJS(m, c);
            let simhResults = runBatch(simh, [c], scratch, 64);
            let sr = simhResults.get(c.index);
            let bad = compare(mut.mn, c, js, sr);
            if (bad) { caught = true; lastFail = bad; break; }
        }
        HANDLERS[mut.mn] = saved;
        DISPATCH[OPC_OF[mut.mn]] = savedDispatch;
        results.push({mn: mut.mn, caught});
        console.log(`  selfcheck ${mut.mn}: ${caught ? "CAUGHT" : "*** NOT CAUGHT ***"}`);
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                              *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function cleanupScratch(scratch)
{
    try { fs.rmSync(scratch, {recursive: true, force: true}); }
    catch (e) { console.log(`  (could not remove scratch ${scratch}: ${e.message})`); }
}

function main()
{
    let simh = findSimh(getArg("--simh", null));
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "intdiff-"));
    let seed = +getArg("--seed", String((Math.random() * 0xFFFFFFFF) | 0));
    console.log(`intdiff.js: simh=${simh} scratch=${scratch} seed=${seed}`);

    /* Every exit path -- success, an assertion/coverage FAIL, or a --selfcheck failure -- runs
       through this try/finally, so scratch is always removed (HANDOFF.md pcjsvax-bd1: this file
       had no rmSync of scratch anywhere, on any path -- 41 abandoned /tmp/intdiff-* dirs found). */
    try {
    if (process.argv.indexOf("--selfcheck") >= 0) {
        let results = selfcheck(simh, scratch);
        let bad = results.filter((r) => !r.caught);
        if (bad.length) {
            console.error(`SELFCHECK FAILED: ${bad.length} mutation(s) not caught: ${bad.map((b) => b.mn).join(", ")}`);
            process.exitCode = 1;
            return;
        }
        console.log(`selfcheck: all ${results.length} mutations caught.`);
        return;
    }

    let casesPerOpcode = +getArg("--cases-per-opcode", "150");
    if (casesPerOpcode < MIN_CASES_PER_OPCODE) {
        console.error(`FATAL: --cases-per-opcode ${casesPerOpcode} is below the coverage floor (${MIN_CASES_PER_OPCODE}); an undersized run must fail, not quietly pass.`);
        process.exitCode = 1;
        return;
    }

    let problems = [];

    console.log(`\n=== RANDOMIZED phase: ${casesPerOpcode} cases x ${IMPLEMENTED.length} opcodes ===`);
    let r = phaseRandomized(simh, scratch, {seed, casesPerOpcode});
    console.log(`  total=${r.total} nontrivial=${r.nontrivial} (${(100 * r.nontrivial / r.total).toFixed(1)}%) memDest=${r.memDest} (${(100 * r.memDest / r.total).toFixed(1)}%)`);
    console.log(`  failures=${r.failures.length} notReached=${r.notReached.length}`);
    if (r.total < MIN_TOTAL_CASES) problems.push(`COVERAGE: total cases ${r.total} < floor ${MIN_TOTAL_CASES}`);
    if (r.nontrivial / r.total < MIN_NONTRIVIAL_FRACTION) problems.push(`COVERAGE: non-trivial fraction ${(r.nontrivial / r.total).toFixed(3)} < floor ${MIN_NONTRIVIAL_FRACTION}`);
    if (r.memDest / r.total < MIN_MEM_DEST_FRACTION) problems.push(`COVERAGE: memory-destination fraction ${(r.memDest / r.total).toFixed(3)} < floor ${MIN_MEM_DEST_FRACTION}`);
    for (let mn of r.mnemonics) {
        let n = r.perOpcodeCounts.get(mn) || 0;
        if (n < MIN_CASES_PER_OPCODE) problems.push(`COVERAGE: opcode ${mn} got only ${n} cases (floor ${MIN_CASES_PER_OPCODE})`);
    }
    if (r.notReached.length) problems.push(`COVERAGE: ${r.notReached.length} case(s) never reached comparison: ${r.notReached.slice(0, 10).join("; ")}`);
    for (let f of r.failures.slice(0, 25)) problems.push("RANDOMIZED: " + f);
    if (r.failures.length > 25) problems.push(`RANDOMIZED: ...and ${r.failures.length - 25} more failures`);

    console.log(`\n=== EHKAA phase ===`);
    let e = phaseEHKAA({simh, scratch, ehkaaExe: getArg("--ehkaa", path.join(vaxRepo(), "open-simh/VAX/tests/ehkaa.exe"))});
    if (e.skipped) {
        console.log(`  SKIPPED: ${e.reason}`);
        problems.push(`EHKAA: skipped (${e.reason}) -- not a substitute for the randomized phase, but its absence means real-workload coverage was NOT exercised this run`);
    } else {
        console.log(`  trace records=${e.total} compared=${e.compared} skippedMemVB=${e.skippedMemVB} skippedTrap=${e.skippedTrap} failures=${e.failures.length}`);
        console.log(`  per-opcode instances: ${[...e.perOpcode.entries()].map(([k, v]) => `${k}=${v}`).sort().join(" ")}`);
        for (let f of e.failures.slice(0, 25)) problems.push("EHKAA: " + f);
        if (e.failures.length > 25) problems.push(`EHKAA: ...and ${e.failures.length - 25} more failures`);
        if (e.compared < 1000) problems.push(`COVERAGE: EHKAA compared only ${e.compared} instances (expected >=1000 across 107 opcodes in a 335,444-instruction trace)`);
    }

    if (problems.length) {
        console.error(`\nFAILED (${problems.length} problem(s)), seed=${seed}:`);
        for (let p of problems) console.error("  - " + p);
        process.exitCode = 1;
        return;
    }
    console.log(`\nPASSED. seed=${seed}`);
    } finally {
        cleanupScratch(scratch);
    }
}

main();
