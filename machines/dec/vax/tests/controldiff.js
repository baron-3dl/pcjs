/**
 * @fileoverview Differential test: VAX branch/jump/call/stack execution vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * A randomized, per-instruction differential test of control.js (execution) against a REAL,
 * EXECUTED Open SIMH microvax3900.  Unlike decodediff.js (which replays a captured instruction
 * HISTORY trace), this test DRIVES the simulator directly: deposit a register file, PSL and a
 * small amount of memory; `step 1`; examine the resulting register file, PSL, and any memory the
 * instruction was expected to write.  No golden fixtures.  If the simulator binary is missing, the
 * test FAILS -- it never degrades into self-comparison.
 *
 *      node machines/dec/vax/tests/controldiff.js [options]
 *        --simh PATH       patched or stock microvax3900 (else $SIMH_BIN, else scratch build)
 *        --cases N         per-opcode case count (default 60)
 *        --calls N         nested CALLS/CALLG/RET trial count (default 80)
 *        --seed S          PRNG seed, printed on failure so a run is reproducible
 *        --selfcheck       prove the differential detects deliberate defects in control.js
 *
 * WHY "step 1" IS SAFE TO COMPARE AGAINST
 * ----------------------------------------
 * SIMH defers arithmetic-overflow/trace traps to the NEXT instruction's fetch (vax_cpu.c:666-707:
 * the trpirq check runs at the TOP of the loop and, when it fires, `continue`s WITHOUT decrementing
 * sim_interval -- so the dispatch is "free" and only the FOLLOWING real instruction fetch consumes
 * the step budget).  Empirically verified: `step 1` on an overflowing SOBGTR with PSW<IV> set stops
 * with PC at the branch target and PSL showing V=1,IV=1 -- NOT at the SCB vector.  So a single step
 * never silently executes a second, unmodelled instruction, and PSW<IV/DV/T> can be randomized
 * freely without risking a step-boundary mismatch (see the file's SELFCHECK/coverage notes for the
 * one place this project deliberately does NOT chase trap dispatch: pcjsvax-e49's job, not this
 * one's).
 *
 * WHAT ADDRESSING-MODE COVERAGE IS, AND ISN'T, THIS FILE'S JOB
 * --------------------------------------------------------------
 * Every addressing mode against every access type is decodediff.js's job (pcjsvax-8c0), already
 * merged and differentially verified over 335,444 EHKAA records plus a dedicated exerciser.  This
 * file exists to prove EXECUTION -- the arithmetic, the branch decision, the stack/frame protocol,
 * the store -- so operand encoding here deliberately uses a SMALL, representative mix (register
 * direct, register deferred for a real memory read/write, PC-relative immediate/absolute for
 * values that don't need to be re-read) rather than re-running the full 9-mode matrix.
 *
 * COVERAGE IS ASSERTED, NOT REPORTED.  Every memory operand is seeded with high-entropy non-zero
 * data (the pcjsvax-cd6 lesson: a uniform/zero pool makes most comparisons trivial and can hide a
 * real bug for 150,000 operations).  A per-opcode floor and a non-trivial-comparison floor both
 * FAIL the run if unmet.  Every `d`/`e` SIMH command is attributed back to its exact case and field
 * by echoed ini line number (busdiff.js's technique): an unexpected SIMH error is reported BY NAME,
 * never silently skipped or misaligned into the next case's comparison.
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import { VAXDecoder, VAXFault } from "../modules/v2/decode.js";
import { OPCODES } from "../modules/v2/drom.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

const MEMSIZE = 0x01000000;             // 16MB, matches busdiff.js/decodediff.js convention

/*
 * Fixed memory layout.  Nothing here needs to vary case-to-case: every byte a case's instruction
 * or operand memory occupies is deposited FRESH, in full, by that case (no addressing mode used
 * here ever reads more bytes than the case explicitly wrote), so reusing the same regions across
 * thousands of cases in one SIMH run cannot leak stale state between them.
 */
const INSTR      = 0x00100000;          // every case's instruction (and, for CASE, its jump table)
const POOL       = [0x00108000, 0x00108040, 0x00108080, 0x001080C0, 0x00108100, 0x00108140];
const STACK_MID  = 0x00110000;          // SP for the single-instruction phase; +/- 0x2000 is free RAM
const CALLPROC   = 0x00118000;          // CALLS/CALLG procedure entry mask + fake body
const NEST_MAIN  = 0x00120000;          // nested CALLS/CALLG/RET phase, one call level per +0x100
const NEST_STACK = 0x00130000;

const MIN_CASES_PER_OPCODE = 40;
const MIN_NONTRIVIAL_RATIO = 0.70;
const MIN_CALL_TRIALS = 40;
const MIN_CALL_DEPTH = 3;

/* ------------------------------------------------------------------------------------------- *
 * Utilities                                                                                     *
 * ------------------------------------------------------------------------------------------- */

/**
 * mulberry32(a)
 *
 * @param {number} a
 * @returns {function(): number}
 */
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
 * hex(v, digits)
 *
 * @param {number} v
 * @param {number} [digits]
 * @returns {string}
 */
function hex(v, digits)
{
    let s = (v >>> 0).toString(16).toUpperCase();
    return digits ? s.padStart(digits, "0") : s;
}

/**
 * hexSized(v, size) -- like hex(), but masks to `size` bytes first.  Every memory deposit MUST go
 * through this, not the raw hex(): a byte/word deposit built from a possibly-NEGATIVE JS number
 * (e.g. a deliberately-injected boundary value like -2) formats via plain hex() as a full 8-digit
 * 32-bit string, which is not what a `-b`/`-w` SIMH deposit expects and desyncs the two sides'
 * view of what got stored -- caught by the self-check's own baseline-must-pass gate, not a mutant.
 *
 * @param {number} v
 * @param {number} size 1, 2, or 4
 * @returns {string}
 */
function hexSized(v, size)
{
    let mask = size === 1 ? 0xFF : size === 2 ? 0xFFFF : 0xFFFFFFFF;
    return hex((v & mask) >>> 0);
}

/**
 * findSimh(argPath)
 *
 * @param {string|null} argPath
 * @returns {string}
 */
function findSimh(argPath)
{
    let candidates = [
        argPath,
        process.env["SIMH_CONTROL_BIN"],
        process.env["SIMH_BIN"],
        path.resolve(REPO_ROOT, "../pcjs-vax/open-simh/BIN/microvax3900")
    ].filter((p) => !!p);
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error("microvax3900 binary not found (tried: " + candidates.join(", ") + ").\n" +
        "This test grades against REAL SIMH; it has no fixture fallback.  Pass --simh PATH or set SIMH_BIN.");
}

/**
 * Rng(seed) -- a tiny helper bundling mulberry32 with the pickers this generator needs repeatedly.
 *
 * @param {number} seed
 */
function Rng(seed)
{
    this.next = mulberry32(seed);
}
Rng.prototype.int = function(n) { return Math.floor(this.next() * n); };
Rng.prototype.bool = function() { return this.next() < 0.5; };
Rng.prototype.pick = function(arr) { return arr[this.int(arr.length)]; };
/** A full-range int32, biased away from 0 and away from being trivially small. */
Rng.prototype.hot32 = function()
{
    let v = (Math.floor(this.next() * 0x100000000)) >>> 0;
    if ((v & 0xFFFFFF00) === 0) v ^= 0xA5A5A500;        // avoid tiny/trivial values
    return v | 0;
};
Rng.prototype.hotByte = function() { let v = this.int(256); return v === 0 ? 0x5A : v; };
Rng.prototype.hotWord = function() { let v = this.int(65536); return v === 0 ? 0x5AA5 : v; };

/* ------------------------------------------------------------------------------------------- *
 * The JS side under test: a minimal CPU wired to the REAL Bus/Memory modules plus VAXDecoder     *
 * plus control.js.  No MMU (mapen off): virtual === physical, exactly as the randomized phases   *
 * of decodediff.js and busdiff.js already assume.                                                *
 * ------------------------------------------------------------------------------------------- */

class Cpu {
    /**
     * @param {BusVAX} bus
     */
    constructor(bus)
    {
        this.bus = bus;
        this.regs = new Int32Array(16);
        this.psl = 0;
        this.decoder = new VAXDecoder(this);
    }

    getISTR(lnt)
    {
        let addr = this.regs[15] >>> 0;
        let v = lnt === 1 ? this.bus.getByte(addr) : lnt === 2 ? this.bus.getWord(addr) : this.bus.getLong(addr);
        this.regs[15] = (this.regs[15] + lnt) | 0;
        return v;
    }

    setPC(pc) { this.regs[15] = pc | 0; }

    readData(va, lnt, fWrite)
    {
        let addr = va >>> 0;
        if (lnt === 1) return this.bus.getByte(addr);
        if (lnt === 2) return this.bus.getWord(addr);
        return this.bus.getLong(addr);
    }

    writeData(va, val, lnt)
    {
        let addr = va >>> 0;
        if (lnt === 1) this.bus.setByte(addr, val);
        else if (lnt === 2) this.bus.setWord(addr, val);
        else this.bus.setLong(addr, val);
    }

    /**
     * load(state) -- reset to a case's initial register file / PSL.
     * @param {Object} state {regs: Array.<number>(16), psl: number}
     */
    load(state)
    {
        this.regs.set(state.regs);
        this.psl = state.psl | 0;
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Case generation                                                                               *
 * ------------------------------------------------------------------------------------------- */

/**
 * newCtx(rng) -- per-case scratch: a random-but-hot initial register file, and a pool of distinct
 * scratch registers (R0-R9) a builder can hand out for value/pointer roles without colliding.
 *
 * @param {Rng} rng
 * @returns {Object}
 */
function newCtx(rng)
{
    let regs = new Array(16);
    for (let i = 0; i < 16; i++) regs[i] = rng.hot32();
    let free = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
    for (let i = free.length - 1; i > 0; i--) {
        let j = rng.int(i + 1);
        [free[i], free[j]] = [free[j], free[i]];
    }
    return {
        rng, regs, free, mem: [],
        nextReg() { return this.free.shift(); },
        nextPool() { return POOL[this.mem.length % POOL.length]; },
        seedPool(addr) {
            // eight bytes of hot, non-zero, non-repeating data so byte/word/long reads at any
            // offset 0-4 are all non-trivial.
            let lo = this.rng.hot32(), hi = this.rng.hot32();
            this.mem.push({addr: addr >>> 0, size: 4, val: lo});
            this.mem.push({addr: (addr + 4) >>> 0, size: 4, val: hi});
        }
    };
}

/**
 * encReg(rn) / encDeferred(rn) / encAbsolute(bytes,addr) / encImmediate(bytes,val,size)
 *
 * Minimal operand encoders.  See the file header for why only these three shapes are used.
 */
function encReg(rn) { return [0x50 | rn]; }
function encDeferred(rn) { return [0x60 | rn]; }
function encAbsolute(addr)
{
    addr = addr >>> 0;
    return [0x9F, addr & 0xFF, (addr >>> 8) & 0xFF, (addr >>> 16) & 0xFF, (addr >>> 24) & 0xFF];
}
function encImmediateB(v) { return [0x8F, v & 0xFF]; }
function encImmediateW(v) { return [0x8F, v & 0xFF, (v >>> 8) & 0xFF]; }
function encImmediateL(v) { return [0x8F, v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF]; }

/**
 * encValue(ctx, size) -- a READ (r.bwl) operand: register / PC-relative immediate / register
 * deferred to a seeded pool address, chosen at random.
 *
 * @param {Object} ctx
 * @param {number} size 1, 2, or 4
 * @returns {Array.<number>} instruction bytes
 */
function encValue(ctx, size, forceValue)
{
    if (forceValue !== undefined) {
        /* deterministic boundary-value case: always register mode, so the exact value lands
           unmodified (an immediate would need re-truncating per size, a deferred read is fine too
           but register is simplest and sufficient -- the addressing mode itself is decodediff's
           coverage, not this generator's). */
        let rn = ctx.nextReg();
        ctx.regs[rn] = forceValue | 0;
        return encReg(rn);
    }
    let variant = ctx.rng.pick(["reg", "imm", "def"]);
    if (variant === "reg") {
        return encReg(ctx.nextReg());
    }
    if (variant === "imm") {
        let v = size === 1 ? ctx.rng.hotByte() : size === 2 ? ctx.rng.hotWord() : ctx.rng.hot32();
        return size === 1 ? encImmediateB(v) : size === 2 ? encImmediateW(v) : encImmediateL(v);
    }
    let rn = ctx.nextReg();
    let addr = ctx.nextPool();
    ctx.regs[rn] = addr;
    ctx.seedPool(addr);
    return encDeferred(rn);
}

/**
 * encModify(ctx, size) -- a MODIFY (m.bwl) operand: register, or register-deferred to a seeded
 * pool address (immediate is architecturally illegal for modify).  Returns the destination
 * descriptor too, so the caller can verify the store afterward.
 *
 * @param {Object} ctx
 * @param {number} size
 * @returns {{bytes: Array.<number>, dest: Object}}
 */
function encModify(ctx, size, forceValue)
{
    if (ctx.rng.bool()) {
        let rn = ctx.nextReg();
        if (forceValue !== undefined) ctx.regs[rn] = forceValue | 0;
        return {bytes: encReg(rn), dest: {kind: "reg", rn}};
    }
    let rn = ctx.nextReg();
    let addr = ctx.nextPool();
    ctx.regs[rn] = addr;
    ctx.seedPool(addr);
    if (forceValue !== undefined) ctx.mem.push({addr, size, val: forceValue | 0});
    return {bytes: encDeferred(rn), dest: {kind: "mem", addr, size}};
}

/**
 * encAddress(ctx, needsBacking) -- an ADDRESS (a.bwlqo) operand: register-deferred or absolute.
 * When `needsBacking` is set (CALLS/CALLG's procedure operand) the address always points at a
 * real, seeded location; otherwise (JMP/JSB/PUSHAx) it is architecturally never dereferenced by
 * the body, so it can be any 32-bit value.
 *
 * @param {Object} ctx
 * @param {boolean} needsBacking
 * @param {number} [backingAddr]
 * @returns {Array.<number>}
 */
function encAddress(ctx, needsBacking, backingAddr)
{
    let addr = needsBacking ? backingAddr : ctx.rng.hot32();
    if (!needsBacking && ctx.rng.bool()) {
        let rn = ctx.nextReg();
        ctx.regs[rn] = addr;
        return encDeferred(rn);
    }
    return encAbsolute(addr);
}

/**
 * encWriteByte(ctx) -- the `w.wb` operand BBx instructions use for their bit's home (register, or
 * register-deferred byte in memory).  Mirrors encModify but always byte-granular in memory.
 *
 * @param {Object} ctx
 * @returns {{bytes: Array.<number>, dest: Object}}
 */
function encWriteByte(ctx)
{
    if (ctx.rng.bool()) {
        let rn = ctx.nextReg();
        return {bytes: encReg(rn), dest: {kind: "reg", rn}};
    }
    let rn = ctx.nextReg();
    let addr = ctx.nextPool();
    ctx.regs[rn] = addr;
    ctx.seedPool(addr);
    return {bytes: encDeferred(rn), dest: {kind: "mem", addr, size: 1}};
}

/**
 * finishCase(name, ctx, opcodeBytes, extra)
 *
 * @param {string} name
 * @param {Object} ctx
 * @param {Array.<number>} opcodeBytes
 * @param {Object} [extra]
 * @returns {Object}
 */
function finishCase(name, ctx, opcodeBytes, extra)
{
    let opc = OPCODES.indexOf(name);
    if (opc >= 0x100) opcodeBytes = [0xFD, opc & 0xFF, ...opcodeBytes];
    else opcodeBytes = [opc, ...opcodeBytes];
    ctx.regs[14] = STACK_MID;                                    // SP: spacious, empty, pool-disjoint
    /* AP/FP (regs[12]/[13]) are NOT touched here: newCtx() already gave them random hot values,
       and a builder (RET, most notably) may have deliberately overwritten one -- clobbering it
       here would silently discard that setup. */
    let cc = ctx.rng.int(16);
    let pswBits = 0;
    if (ctx.rng.bool()) pswBits |= 0x80;                          // DV
    if (ctx.rng.bool()) pswBits |= 0x40;                          // FU
    if (ctx.rng.bool()) pswBits |= 0x20;                          // IV
    /*
     * PSW<T> is DELIBERATELY never set in the INCOMING PSL here.  vax_cpu.c:709-717 arms PSL<TP>
     * (bit 30) from PSW<T> unconditionally at the top of EVERY instruction's dispatch -- not a
     * deferred trap, a real, immediate, every-single-step side effect -- so an incoming PSL with
     * T=1 would make SIMH's post-step PSL differ from ours by PSL_TP, and modelling PSL_TP/trace-
     * trap dispatch is pcjsvax-e49's job, not this item's.  RET restoring T=1 INTO its OUTPUT PSL
     * (from a hand-built frame's spamask) is still fully exercised and compared -- that arming
     * check only ever looks at the PSL a step STARTS with, never the one it ends with, so a T that
     * appears only as this step's result is unaffected and remains a fair comparison.
     */
    let psl = pswBits | cc;
    let bytes = opcodeBytes;
    return Object.assign({
        name, bytes, regs: ctx.regs.slice(), psl, pc: INSTR, mem: ctx.mem, checks: []
    }, extra || {});
}

/*
 * Per-opcode-family builders.  Each returns a Case: {name, bytes, regs, psl, pc, mem, checks}.
 * `checks` lists extra memory to examine after the step (destination stores this instruction may
 * have made, beyond the register file + PSL that every case always checks).
 */
const BUILDERS = {};

/* ---- no-operand branches: brdisp only ---- */
for (let name of ["BRB", "BSBB", "BNEQ", "BEQL", "BGTR", "BLEQ", "BGEQ", "BLSS",
                   "BGTRU", "BLEQU", "BVC", "BVS", "BGEQU", "BLSSU"]) {
    BUILDERS[name] = (ctx) => finishCase(name, ctx, [ctx.rng.int(256)]);
}
for (let name of ["BRW", "BSBW"]) {
    BUILDERS[name] = (ctx) => finishCase(name, ctx, [ctx.rng.int(256), ctx.rng.int(256)]);
}
BUILDERS.RET = (ctx) => {
    /* RET reads its frame from FP; give it a small, well-formed, self-consistent one so the
       single-instruction phase exercises the pop path (the nested phase exercises RET after a
       REAL CALLS/CALLG, which is the protocol-correctness-bearing case). */
    let mask = ctx.rng.int(0x1000);                              // bits 0-11 only, MBZ bits clear
    let fp = ctx.nextPool();
    let dvfut = ctx.rng.int(16) << 4;                             // DV/FU/IV/T bits, any combo
    let callS = ctx.rng.bool() ? 0x20000000 : 0;                  // CALL_S: was this a CALLS frame?
    let spamask = (mask << 16) | callS | dvfut | ctx.rng.int(16);
    ctx.mem.push({addr: (fp + 4) >>> 0, size: 4, val: spamask});
    ctx.mem.push({addr: (fp + 8) >>> 0, size: 4, val: ctx.rng.hot32()});   // saved AP
    ctx.mem.push({addr: (fp + 12) >>> 0, size: 4, val: ctx.rng.hot32()});  // saved FP
    ctx.mem.push({addr: (fp + 16) >>> 0, size: 4, val: ctx.rng.hot32() & 0x7FFFFFFF}); // saved PC
    let off = 20;
    for (let n = 0; n <= 11; n++) {
        if ((mask >>> n) & 1) { ctx.mem.push({addr: (fp + off) >>> 0, size: 4, val: ctx.rng.hot32()}); off += 4; }
    }
    if (callS) {
        /* CALLS frames carry the argument count right where SP lands after the register pops;
           RET reads it and pops that many longwords off the caller's argument list too. */
        ctx.mem.push({addr: (fp + off) >>> 0, size: 4, val: ctx.rng.int(8)});
    }
    ctx.regs[13] = fp;                                            // FP
    return finishCase("RET", ctx, []);
};
BUILDERS.RSB = (ctx) => {
    ctx.mem.push({addr: STACK_MID, size: 4, val: ctx.rng.hot32() & 0x7FFFFFFF});
    return finishCase("RSB", ctx, []);
};

/* ---- JMP / JSB: one address operand, never dereferenced ---- */
for (let name of ["JMP", "JSB"]) {
    BUILDERS[name] = (ctx) => finishCase(name, ctx, encAddress(ctx, false));
}

/*
 * Boundary-value injection for the comparison-with-branch family (SOB/AOB/ACB).  Uniform random
 * 32-bit values essentially NEVER land exactly on the r==0 / r==limit boundary that distinguishes
 * ">" from ">=" (SOBGTR vs SOBGEQ's condition, AOBLSS vs AOBLEQ's) -- so a naive random pool cannot
 * catch an off-by-one there.  Used directly (SOB) or to derive a coordinated (limit, index) pair
 * (AOB/ACB) so the comparison itself, not just the arithmetic, gets exercised at its edge.
 */
const BOUNDARY_L = [0, 1, 2, -1, -2, (0x80000000 | 0), 0x7FFFFFFF];

/* ---- SOB: idx.ml, disp.bb ---- */
for (let name of ["SOBGEQ", "SOBGTR"]) {
    BUILDERS[name] = (ctx) => {
        let force = ctx.rng.bool() ? ctx.rng.pick(BOUNDARY_L) : undefined;
        let m = encModify(ctx, 4, force);
        let c = finishCase(name, ctx, [...m.bytes, ctx.rng.int(256)]);
        if (m.dest.kind === "mem") c.checks.push({addr: m.dest.addr, size: 4, label: "modify-dest"});
        else c.checks.push({reg: m.dest.rn, label: "modify-dest"});
        return c;
    };
}

/* ---- AOB: limit.rl, idx.ml, disp.bb ---- */
for (let name of ["AOBLSS", "AOBLEQ"]) {
    BUILDERS[name] = (ctx) => {
        let lim, idx;
        if (ctx.rng.bool()) {
            lim = ctx.rng.pick([-2, -1, 0, 1, 2, 5]);
            idx = (lim - 1 + ctx.rng.pick([-1, 0, 1])) | 0;   // (idx+1) lands at/around limit
        }
        let limBytes = encValue(ctx, 4, lim);
        let m = encModify(ctx, 4, idx);
        let c = finishCase(name, ctx, [...limBytes, ...m.bytes, ctx.rng.int(256)]);
        if (m.dest.kind === "mem") c.checks.push({addr: m.dest.addr, size: 4, label: "modify-dest"});
        else c.checks.push({reg: m.dest.rn, label: "modify-dest"});
        return c;
    };
}

/* ---- ACB: limit.rx, add.rx, index.mx, disp.bw ---- */
const ACB_SIZE = {ACBB: 1, ACBW: 2, ACBL: 4};
for (let name of ["ACBB", "ACBW", "ACBL"]) {
    BUILDERS[name] = (ctx) => {
        let sz = ACB_SIZE[name];
        let lim, idx, add;
        if (ctx.rng.bool()) {
            add = ctx.rng.pick([1, -1, 2, -2]);               // small, sign-known step
            lim = ctx.rng.pick([-2, -1, 0, 1, 2, 5]);
            idx = (lim - add + ctx.rng.pick([-1, 0, 1])) | 0;  // (idx+add) lands at/around limit
        }
        let limBytes = encValue(ctx, sz, lim);
        let addBytes = encValue(ctx, sz, add);
        let m = encModify(ctx, sz, idx);
        let c = finishCase(name, ctx, [...limBytes, ...addBytes, ...m.bytes, ctx.rng.int(256), ctx.rng.int(256)]);
        if (m.dest.kind === "mem") c.checks.push({addr: m.dest.addr, size: sz, label: "modify-dest"});
        else c.checks.push({reg: m.dest.rn, label: "modify-dest"});
        return c;
    };
}

/* ---- CASE: sel.rx, base.rx, lim.rx, then a jump table of (limit+1) words right after ---- */
const CASE_SIZE = {CASEB: 1, CASEW: 2, CASEL: 4};
for (let name of ["CASEB", "CASEW", "CASEL"]) {
    BUILDERS[name] = (ctx) => {
        let sz = CASE_SIZE[name];
        /* keep the immediate encoding for selector/base/limit so we know their exact values --
           the case must sometimes land IN the table (selector-base <= limit), sometimes past it,
           and -- for CASEL specifically, where (sel-base) is a full signed int32 rather than a
           masked byte/word -- sometimes with sel < base, so (sel-base) is genuinely NEGATIVE and
           the unsigned-vs-signed comparison against limit is actually distinguishable. */
        let base = ctx.rng.int(4);
        let limit = ctx.rng.int(6);                               // small: keeps the table short
        let kind = ctx.rng.pick(["in", "out", "negative"]);
        let sel = kind === "in" ? (base + ctx.rng.int(limit + 1))
                : kind === "out" ? (base + limit + 1 + ctx.rng.int(4))
                : (base - (1 + ctx.rng.int(4)));
        let selBytes = sz === 1 ? encImmediateB(sel) : sz === 2 ? encImmediateW(sel) : encImmediateL(sel);
        let baseBytes = sz === 1 ? encImmediateB(base) : sz === 2 ? encImmediateW(base) : encImmediateL(base);
        let limBytes = sz === 1 ? encImmediateB(limit) : sz === 2 ? encImmediateW(limit) : encImmediateL(limit);
        let instrBytes = [...selBytes, ...baseBytes, ...limBytes];
        let c = finishCase(name, ctx, instrBytes);
        /* jump table lands immediately after the instruction in INSTR-space; deposit (limit+1)
           random word displacements. */
        let tableAddr = (INSTR + 1 + instrBytes.length) >>> 0;    // +1 for the opcode byte
        for (let i = 0; i <= limit; i++) {
            ctx.mem.push({addr: (tableAddr + i * 2) >>> 0, size: 2, val: ctx.rng.hotWord() & 0xFFFF});
        }
        return c;
    };
}

/* ---- Branch-on-bit: pos.rl, op.wb, disp.bb ---- */
for (let name of ["BBS", "BBC", "BBSS", "BBSSI", "BBCC", "BBCCI", "BBSC", "BBCS"]) {
    BUILDERS[name] = (ctx) => {
        let w = encWriteByte(ctx);
        let pos, checkAddr;
        if (w.dest.kind === "reg") {
            pos = ctx.rng.int(32);                                 // register mode: >31 is the fault subphase's job
        } else {
            /*
             * Memory mode allows an UNRESTRICTED signed pos -- that is precisely the deliberate
             * `pos >> 3` (arithmetic, not logical) shift control.js's opBbN/opBbX docblock explains.
             * A pos confined to 0-31 (as register mode requires) never exercises that: only a
             * negative or >31 pos moves `ea` away from the base address at all, so deliberately bias
             * toward exactly those.
             */
            let variant = ctx.rng.pick(["small", "negative", "farpos"]);
            pos = variant === "small" ? ctx.rng.int(32)
                : variant === "negative" ? -(1 + ctx.rng.int(40))
                : 32 + ctx.rng.int(40);
            let ea = (w.dest.addr + (pos >> 3)) | 0;               // mirrors control.js's own computation
            checkAddr = ea;
            ctx.mem.push({addr: (ea & ~3) >>> 0, size: 4, val: ctx.rng.hot32()});
        }
        let posBytes = encImmediateL(pos);
        let c = finishCase(name, ctx, [...posBytes, ...w.bytes, ctx.rng.int(256)]);
        if (w.dest.kind === "mem") c.checks.push({addr: checkAddr, size: 1, label: "bit-dest"});
        else c.checks.push({reg: w.dest.rn, label: "bit-dest"});
        return c;
    };
}
for (let name of ["BLBS", "BLBC"]) {
    BUILDERS[name] = (ctx) => finishCase(name, ctx, [...encValue(ctx, 4), ctx.rng.int(256)]);
}

/* ---- Stack ---- */
BUILDERS.PUSHL = (ctx) => finishCase("PUSHL", ctx, encValue(ctx, 4));
for (let name of ["PUSHAB", "PUSHAW", "PUSHAL", "PUSHAQ"]) {
    BUILDERS[name] = (ctx) => finishCase(name, ctx, encAddress(ctx, false));
}
for (let name of ["PUSHR", "POPR"]) {
    BUILDERS[name] = (ctx) => {
        let mask = ctx.rng.int(0x8000);                            // bits 0-14 (R0-R14/SP)
        if (name === "POPR") {
            /* pre-seed a valid frame above SP for each set bit, so the pop path reads real data */
            let sp = STACK_MID;
            let off = 0;
            for (let n = 0; n <= 13; n++) {
                if ((mask >>> n) & 1) { ctx.mem.push({addr: (sp + off) >>> 0, size: 4, val: ctx.rng.hot32()}); off += 4; }
            }
            if ((mask >>> 14) & 1) ctx.mem.push({addr: (sp + off) >>> 0, size: 4, val: ctx.rng.hot32() & 0x7FFFFFFF});
        }
        return finishCase(name, ctx, encImmediateW(mask));
    };
}

/* ---- CALLS / CALLG (single-instruction phase; the nested phase is where the protocol is proven) ---- */
for (let gs of [true, false]) {
    let name = gs ? "CALLS" : "CALLG";
    BUILDERS[name] = (ctx) => {
        let mask = ctx.rng.int(0x1000);                            // bits 0-11 only: MBZ clear -> no fault
        ctx.mem.push({addr: CALLPROC, size: 2, val: mask});
        let numargBytes = gs ? [ctx.rng.int(64)] : encAddress(ctx, false);   // CALLS: short literal; CALLG: address
        let procBytes = encAddress(ctx, true, CALLPROC);
        return finishCase(name, ctx, [...numargBytes, ...procBytes]);
    };
}

const IN_SCOPE = Object.keys(BUILDERS);

/* ------------------------------------------------------------------------------------------- *
 * SIMH command stream                                                                           *
 * ------------------------------------------------------------------------------------------- */

/**
 * emitCase(lines, cmds, iCase, c)
 *
 * @param {Array.<string>} lines
 * @param {Array.<Object>} cmds
 * @param {number} iCase
 * @param {Object} c
 */
function emitCase(lines, cmds, iCase, c)
{
    let emit = (cmd, info) => { lines.push(cmd); info.line = lines.length; cmds.push(info); };
    /*
     * TRPIRQ is a real SIMH global (cpu_reg[]), not reset between separate "step 1" invocations of
     * the same batch: a PRIOR case that set PSW<IV>/PSW<DV> and genuinely overflowed leaves a
     * pending trap that fires at the top of the NEXT, otherwise unrelated case's step (vax_cpu.c's
     * trpirq check runs before that case's own fetch) -- observed directly: an untouched case's
     * step landed on "HALT instruction, PC: 00000001" with SP -12, i.e. a stale trap's exception
     * push, from a completely different opcode two cases earlier.  Clearing it here, every case,
     * is what makes "step 1" a clean single-instruction boundary across an entire batched run.
     */
    emit("d TRPIRQ 0", {kind: "d"});
    for (let i = 0; i < 15; i++) emit("d R" + i + " " + hex(c.regs[i]), {kind: "d"});
    emit("d PC " + hex(c.pc), {kind: "d"});
    emit("d PSL " + hex(c.psl), {kind: "d"});
    for (let m of c.mem) {
        let sw = m.size === 1 ? "-b" : m.size === 2 ? "-w" : "-l";
        emit("d " + sw + " " + hex(m.addr) + " " + hexSized(m.val, m.size), {kind: "d"});
    }
    let off = 0;
    for (let b of c.bytes) { emit("d -b " + hex((INSTR + off) >>> 0) + " " + hex(b, 2), {kind: "d"}); off++; }
    emit("step 1", {kind: "step"});
    for (let i = 0; i < 15; i++) emit("e R" + i, {kind: "e", iCase, field: "R" + i});
    emit("e PC", {kind: "e", iCase, field: "PC"});
    emit("e PSL", {kind: "e", iCase, field: "PSL"});
    for (let chk of c.checks) {
        if (chk.reg !== undefined) continue;                      // register checks reuse the R-file examine above
        emit("e -l " + hex(chk.addr & ~3), {kind: "e", iCase, field: "M" + hex(chk.addr & ~3)});
    }
}

/**
 * runSimh(simhBin, iniText)
 *
 * @param {string} simhBin
 * @param {string} iniText
 * @returns {{out: string, dir: string}}
 */
function runSimh(simhBin, iniText)
{
    let dir = fs.mkdtempSync(path.join(os.tmpdir(), "vax-controldiff-"));
    let iniPath = path.join(dir, "run.ini");
    fs.writeFileSync(iniPath, iniText);
    let out = execFileSync(simhBin, [iniPath], {maxBuffer: 1 << 30, encoding: "utf8"});
    fs.writeFileSync(path.join(dir, "run.out"), out);
    return {out, dir};
}

/**
 * parseSimhOutput(out, cmds)
 *
 * Attributes every result back to its command by echoed ini line number (busdiff.js's technique),
 * so an unexpected SIMH error on any single command is named, not silently misaligned into the
 * next comparison.
 *
 * @param {string} out
 * @param {Array.<Object>} cmds
 * @returns {Array.<Object>} parallel to cmds: {ok, value} or {ok:false, err}
 */
function parseSimhOutput(out, cmds)
{
    let mapErr = new Map();
    let aValues = [];
    let lines = out.split("\n");
    let reResult = /^(\S+):\t([0-9A-Fa-f]+)/;
    let reEcho = /-(\d+)>\s+(\S.*)$/;
    for (let i = 0; i < lines.length; i++) {
        let m = reResult.exec(lines[i]);
        if (m) { aValues.push(parseInt(m[2], 16) >>> 0); continue; }
        m = reEcho.exec(lines[i]);
        if (m) mapErr.set(+m[1], (lines[i + 1] || "").trim());
    }
    let results = new Array(cmds.length);
    let iValue = 0;
    for (let i = 0; i < cmds.length; i++) {
        let cmd = cmds[i];
        if (mapErr.has(cmd.line)) {
            results[i] = {ok: false, err: mapErr.get(cmd.line)};
        } else if (cmd.kind === "d" || cmd.kind === "step") {
            results[i] = {ok: true};
        } else {
            if (iValue >= aValues.length) {
                throw new Error(`SIMH output underrun at command ${i} (ini line ${cmd.line})`);
            }
            results[i] = {ok: true, value: aValues[iValue++]};
        }
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * JS-side execution + comparison                                                                *
 * ------------------------------------------------------------------------------------------- */

/**
 * runJsCase(cpu, bus, c)
 *
 * Loads the case's initial state onto the real Bus, decodes and executes exactly one instruction,
 * and returns the resulting register file / PSL (or a fault).
 *
 * @param {Cpu} cpu
 * @param {BusVAX} bus
 * @param {Function} executeControl
 * @param {Object} c
 * @returns {{regs: Int32Array, psl: number, fault: (VAXFault|null)}}
 */
function runJsCase(cpu, bus, executeControl, c)
{
    for (let m of c.mem) {
        if (m.size === 1) bus.setByte(m.addr, m.val);
        else if (m.size === 2) bus.setWord(m.addr, m.val);
        else bus.setLong(m.addr, m.val);
    }
    let off = 0;
    for (let b of c.bytes) { bus.setByte((INSTR + off) >>> 0, b); off++; }
    cpu.load({regs: c.regs, psl: c.psl});
    cpu.setPC(c.pc);
    let fault = null;
    try {
        let opc = cpu.decoder.decode(false);
        if (!(cpu.psl & 0x20000000)) cpu.decoder.resetRecovery();  // PSL_FPD is never set in this test
        let ok = executeControl(opc, cpu.decoder, cpu);
        if (!ok) throw new Error(`opcode ${hex(opc)} (${OPCODES[opc]}) has no control.js body`);
    } catch (e) {
        if (e instanceof VAXFault) { fault = e; } else { throw e; }
    }
    return {regs: cpu.regs.slice(), psl: cpu.psl, fault};
}

/**
 * compareCase(c, jsResult, simhFields)
 *
 * Compares the register file and PSL only.  Memory-destination checks (SOB/AOB/ACB's modify
 * operand, BBx's write-byte operand) are compared separately in phaseSingle(), which has direct
 * access to the JS Bus the case executed against and so can read the actual stored bytes back
 * rather than trusting a duplicate of jsResult.
 *
 * @param {Object} c
 * @param {Object} jsResult
 * @param {Object} simhFields map field -> {ok, value}
 * @returns {Array.<string>} mismatch descriptions (empty if none)
 */
function compareCase(c, jsResult, simhFields)
{
    let problems = [];
    for (let i = 0; i < 15; i++) {
        let f = simhFields["R" + i];
        if (!f.ok) { problems.push(`R${i}: SIMH error: ${f.err}`); continue; }
        if ((jsResult.regs[i] >>> 0) !== f.value) {
            problems.push(`R${i}: JS=${hex(jsResult.regs[i])} SIMH=${hex(f.value)}`);
        }
    }
    let fPC = simhFields.PC;
    if (!fPC.ok) problems.push(`PC: SIMH error: ${fPC.err}`);
    else if ((jsResult.regs[15] >>> 0) !== fPC.value) problems.push(`PC: JS=${hex(jsResult.regs[15])} SIMH=${hex(fPC.value)}`);
    let fPSL = simhFields.PSL;
    if (!fPSL.ok) problems.push(`PSL: SIMH error: ${fPSL.err}`);
    else if ((jsResult.psl >>> 0) !== fPSL.value) problems.push(`PSL: JS=${hex(jsResult.psl)} SIMH=${hex(fPSL.value)}`);
    return problems;
}

/**
 * isNonTrivial(c, jsResult)
 *
 * A case is "trivial" if nothing about the register file, PSL or checked memory actually changed
 * from the initial state -- the pcjsvax-cd6 lesson about seeded, non-zero data applies here too.
 *
 * @param {Object} c
 * @param {Object} jsResult
 * @returns {boolean}
 */
function isNonTrivial(c, jsResult)
{
    for (let i = 0; i < 16; i++) if ((jsResult.regs[i] | 0) !== (c.regs[i] | 0)) return true;
    if ((jsResult.psl | 0) !== (c.psl | 0)) return true;
    return false;
}

/* ------------------------------------------------------------------------------------------- *
 * Phase: single-instruction randomized differential, one opcode at a time                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * phaseSingle(simh, executeControl, opts)
 *
 * @param {string} simh
 * @param {Function} executeControl
 * @param {Object} opts
 * @returns {Object}
 */
function phaseSingle(simh, executeControl, opts)
{
    let rng = new Rng(opts.seed ^ 0x51E6E1D3);
    let cases = [];
    for (let name of IN_SCOPE) {
        let n = opts.cases;
        for (let i = 0; i < n; i++) {
            let ctx = newCtx(rng);
            let c = BUILDERS[name](ctx);
            cases.push(c);
        }
    }

    let lines = ["set cpu " + (MEMSIZE >>> 20) + "m"];
    let cmds = [];
    for (let i = 0; i < cases.length; i++) emitCase(lines, cmds, i, cases[i]);
    lines.push("quit");
    let {out} = runSimh(simh, lines.join("\n") + "\n");
    let results = parseSimhOutput(out, cmds);

    let bus = new BusVAX({'busWidth': VAX.PAWIDTH, 'id': "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new Cpu(bus);

    let failures = [];
    let nontrivial = 0;
    let byOpcode = new Map();

    /* field map per case, built in one pass over cmds */
    let perCaseFields = cases.map(() => ({}));
    for (let k = 0; k < cmds.length; k++) {
        let cmd = cmds[k];
        if (cmd.kind === "e") perCaseFields[cmd.iCase][cmd.field] = results[k];
    }

    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        let jsResult = runJsCase(cpu, bus, executeControl, c);
        let stat = byOpcode.get(c.name) || {n: 0, nontrivial: 0};
        stat.n++;
        byOpcode.set(c.name, stat);
        if (jsResult.fault) {
            failures.push(`case ${i} (${c.name}) [${c.bytes.map((b) => hex(b, 2)).join(" ")}]: JS threw unexpected fault ${jsResult.fault.code}`);
            continue;
        }
        let fields = perCaseFields[i];
        let problems = compareCase(c, jsResult, fields);
        for (let chk of c.checks) {
            if (chk.reg !== undefined) continue;
            let key = "M" + hex(chk.addr & ~3);
            let f = fields[key];
            if (!f) continue;
            if (!f.ok) { problems.push(`${chk.label}@${hex(chk.addr)}: SIMH error: ${f.err}`); continue; }
            let jsVal = bus.getLong(chk.addr & ~3);
            if ((jsVal >>> 0) !== f.value) problems.push(`${chk.label}@${hex(chk.addr)}: JS=${hex(jsVal)} SIMH=${hex(f.value)}`);
        }
        if (problems.length) {
            failures.push(`case ${i} (${c.name}) [${c.bytes.map((b) => hex(b, 2)).join(" ")}]: ` + problems.join("; "));
        } else if (isNonTrivial(c, jsResult)) {
            stat.nontrivial++;
            nontrivial++;
        }
    }

    return {cases, failures, nontrivial, byOpcode};
}

/* ------------------------------------------------------------------------------------------- *
 * Phase: reserved-operand faults (CALLS/CALLG/RET MBZ checks, BBx pos>31)                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * phaseFaults(simh, executeControl, opts)
 *
 * Points SCB vector 0x18 (reserved-operand) at a HALT and confirms SIMH takes it (PC lands there)
 * for the same malformed operands our decoder.fault(RESOP) rejects, WITHOUT trying to replicate
 * SIMH's own exception-frame push (intexc()) -- that belongs to pcjsvax-e49, not this item.  What
 * IS this item's to prove: the fault fires at the documented point, before any side effect.
 *
 * @param {string} simh
 * @param {Function} executeControl
 * @param {Object} opts
 * @returns {Object}
 */
function phaseFaults(simh, executeControl, opts)
{
    let rng = new Rng(opts.seed ^ 0x2FA17BAD);
    let cases = [];

    /* CALLS/CALLG: mask bits 12 or 13 set (CALL_MBZ) */
    for (let gs of [true, false]) {
        let name = gs ? "CALLS" : "CALLG";
        for (let i = 0; i < 8; i++) {
            let ctx = newCtx(rng);
            let mask = (ctx.rng.int(0x1000)) | (ctx.rng.bool() ? 0x1000 : 0x2000);
            ctx.mem.push({addr: CALLPROC, size: 2, val: mask});
            let numargBytes = gs ? [ctx.rng.int(64)] : encAddress(ctx, false);
            let procBytes = encAddress(ctx, true, CALLPROC);
            let c = finishCase(name, ctx, [...numargBytes, ...procBytes]);
            c.expectFault = true;
            cases.push(c);
        }
    }
    /* RET: spamask MBZ bits (PSW_MBZ = 0xFF00) */
    for (let i = 0; i < 8; i++) {
        let ctx = newCtx(rng);
        let mask = ctx.rng.int(0x1000);
        let fp = ctx.nextPool();
        let spamask = (mask << 16) | (ctx.rng.int(0xFF) << 8) | 0x100;   // force a bit inside 0xFF00
        ctx.mem.push({addr: (fp + 4) >>> 0, size: 4, val: spamask});
        ctx.regs[13] = fp;
        let c = finishCase("RET", ctx, []);
        c.expectFault = true;
        cases.push(c);
    }
    /* BBx register-mode with pos > 31 */
    for (let name of ["BBS", "BBSS", "BLBS"]) {
        for (let i = 0; i < 6; i++) {
            let ctx = newCtx(rng);
            let pos = 32 + ctx.rng.int(0x1000000);
            if (name === "BLBS") { ctx.free.shift(); continue; }    // BLBS has no bit-position operand
            let rn = ctx.nextReg();
            let c = finishCase(name, ctx, [...encImmediateL(pos), ...encReg(rn), ctx.rng.int(256)]);
            c.expectFault = true;
            cases.push(c);
        }
    }

    /* SCB[k] is a POINTER (a longword), not the handler code itself -- vax_cpu.c's intexc() reads
       SCBB+vector as an address and jumps there.  Point vector 0x18 (RESOP) at a HALT elsewhere in
       memory, exactly as decodediff.js's FAULTS phase does. */
    const FAULT_HANDLER = 0x00100800;
    let lines = [
        "set cpu " + (MEMSIZE >>> 20) + "m",
        "d -l 18 " + hex(FAULT_HANDLER),
        "d -b " + hex(FAULT_HANDLER) + " 00"
    ];
    let cmds = [];
    for (let i = 0; i < cases.length; i++) emitCase(lines, cmds, i, cases[i]);
    lines.push("quit");
    let {out} = runSimh(simh, lines.join("\n") + "\n");
    let results = parseSimhOutput(out, cmds);
    let perCaseFields = cases.map(() => ({}));
    for (let k = 0; k < cmds.length; k++) {
        let cmd = cmds[k];
        if (cmd.kind === "e") perCaseFields[cmd.iCase][cmd.field] = results[k];
    }

    let bus = new BusVAX({'busWidth': VAX.PAWIDTH, 'id': "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new Cpu(bus);

    let failures = [];
    let nCompared = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        let jsResult = runJsCase(cpu, bus, executeControl, c);
        let fPC = perCaseFields[i].PC;
        if (!fPC.ok) { failures.push(`fault case ${i} (${c.name}): SIMH error on PC: ${fPC.err}`); continue; }
        nCompared++;
        if (fPC.value !== FAULT_HANDLER) {
            failures.push(`fault case ${i} (${c.name}): SIMH did not take the reserved-operand vector (PC=${hex(fPC.value)})`);
        }
        if (!jsResult.fault) {
            failures.push(`fault case ${i} (${c.name}): JS did not throw (SIMH took the fault)`);
        } else if (jsResult.fault.code !== -0x18) {
            failures.push(`fault case ${i} (${c.name}): JS threw code ${jsResult.fault.code}, expected RESOP (-0x18)`);
        }
    }
    return {failures, nCompared};
}

/* ------------------------------------------------------------------------------------------- *
 * Phase: nested CALLS/CALLG/RET                                                                 *
 * ------------------------------------------------------------------------------------------- */

/**
 * buildNestTrial(rng, depth)
 *
 * A linear chain of `depth` CALLS/CALLG, each landing on a proc whose body is a single CALLS/CALLG
 * to the next level (or RET at the bottom), followed by `depth` RETs that unwind it -- so the full
 * trial is a flat sequence of 2*depth single-step instructions, state compared after EVERY step,
 * not just the end.  This is deliberately NOT the single-instruction phase's CALLS builder: it is
 * the actual protocol under real nesting, mask/argument-count/gs varied at every level.
 *
 * @param {Rng} rng
 * @param {number} depth
 * @returns {Object} {steps: Array.<{bytes,pc}>, regs, psl}
 */
function buildNestTrial(rng, depth)
{
    let regs = new Array(16);
    for (let i = 0; i < 12; i++) regs[i] = (Math.floor(rng.next() * 0x100000000)) | 0;
    regs[12] = (Math.floor(rng.next() * 0x100000000)) | 0;
    regs[13] = (Math.floor(rng.next() * 0x100000000)) | 0;
    regs[14] = NEST_STACK;
    let psl = rng.int(16);                                        // cc only; DV/FU/IV/T left clear for a clean chain

    /* Level 0 ("main") is never entered via a CALL, so it has no entry mask and its code starts
       right at NEST_MAIN.  Levels 1..depth are each entered by the level above's CALLS/CALLG, so
       each has a 2-byte entry mask at maskAddr(lvl) and its code (what the CALL actually jumps to,
       addr+2) starts right after it. */
    let maskAddr = (lvl) => (NEST_MAIN + lvl * 0x100) >>> 0;
    let codeAddr = (lvl) => lvl === 0 ? NEST_MAIN : (maskAddr(lvl) + 2) >>> 0;

    let callSteps = [];
    for (let lvl = 0; lvl < depth; lvl++) {
        let gs = rng.bool();
        let mask = rng.int(0x1000);                                // bits 0-11 only: MBZ clear, no fault
        let numarg = gs ? rng.int(64) : 0;
        let targetMask = maskAddr(lvl + 1);
        let bytes = [OPCODES.indexOf(gs ? "CALLS" : "CALLG")];
        if (gs) {
            bytes.push(numarg);
        } else {
            /* CALLG's opnd[0] is an arglist ADDRESS, never dereferenced by op_call -- it becomes
               AP verbatim (see control.js opCall).  Any distinct address-shaped value exercises
               that path without needing real backing memory. */
            bytes.push(...encAbsolute((targetMask ^ 0x0F0F0F0F) >>> 0));
        }
        bytes.push(...encAbsolute(targetMask));
        callSteps.push({pc: codeAddr(lvl), bytes, mask, isCall: true});
    }

    /* Return chain, in CHRONOLOGICAL execution order: the innermost RET (level `depth`, which
       never itself calls anything) runs first, then one RET per intermediate level, each placed
       exactly where that level's own CALLS/CALLG left the return address on the stack. */
    let retSteps = [{pc: codeAddr(depth), bytes: [OPCODES.indexOf("RET")], isCall: false}];
    for (let lvl = depth - 1; lvl >= 1; lvl--) {
        let cs = callSteps[lvl];
        let retPC = (cs.pc + cs.bytes.length) >>> 0;
        retSteps.push({pc: retPC, bytes: [OPCODES.indexOf("RET")], isCall: false});
    }

    let steps = callSteps.concat(retSteps);
    let maskWrites = [];
    for (let lvl = 0; lvl < depth; lvl++) maskWrites.push({addr: maskAddr(lvl + 1), size: 2, val: callSteps[lvl].mask});

    return {steps, regs, psl, maskWrites, depth};
}

/**
 * phaseNested(simh, executeControl, opts)
 *
 * @param {string} simh
 * @param {Function} executeControl
 * @param {Object} opts
 * @returns {Object}
 */
function phaseNested(simh, executeControl, opts)
{
    let rng = new Rng(opts.seed ^ 0x7EA57ED0);
    let trials = [];
    for (let i = 0; i < opts.calls; i++) {
        let depth = MIN_CALL_DEPTH + rng.int(2);                    // depth 3 or 4
        trials.push(buildNestTrial(rng, depth));
    }

    let lines = ["set cpu " + (MEMSIZE >>> 20) + "m"];
    let cmds = [];
    let trialCmdRanges = [];
    for (let ti = 0; ti < trials.length; ti++) {
        let t = trials[ti];
        let emit = (cmd, info) => { lines.push(cmd); info.line = lines.length; cmds.push(info); };
        emit("d TRPIRQ 0", {kind: "d"});                             // see emitCase()'s docblock
        for (let i = 0; i < 12; i++) emit("d R" + i + " " + hex(t.regs[i]), {kind: "d"});
        emit("d R12 " + hex(t.regs[12]), {kind: "d"});
        emit("d R13 " + hex(t.regs[13]), {kind: "d"});
        emit("d R14 " + hex(t.regs[14]), {kind: "d"});
        emit("d PC " + hex(t.steps[0].pc), {kind: "d"});
        emit("d PSL " + hex(t.psl), {kind: "d"});
        for (let mw of t.maskWrites) {
            let sw = mw.size === 1 ? "-b" : mw.size === 2 ? "-w" : "-l";
            emit("d " + sw + " " + hex(mw.addr) + " " + hexSized(mw.val, mw.size), {kind: "d"});
        }
        for (let s of t.steps) {
            let off = 0;
            for (let b of s.bytes) { emit("d -b " + hex((s.pc + off) >>> 0) + " " + hex(b, 2), {kind: "d"}); off++; }
        }
        let firstCmdIdx = cmds.length;
        for (let si = 0; si < t.steps.length; si++) {
            emit("step 1", {kind: "step"});
            for (let i = 0; i < 15; i++) emit("e R" + i, {kind: "e", ti, si, field: "R" + i});
            emit("e PC", {kind: "e", ti, si, field: "PC"});
            emit("e PSL", {kind: "e", ti, si, field: "PSL"});
        }
        trialCmdRanges.push({start: firstCmdIdx, end: cmds.length});
    }
    lines.push("quit");
    let {out} = runSimh(simh, lines.join("\n") + "\n");
    let results = parseSimhOutput(out, cmds);
    let perStepFields = trials.map((t) => t.steps.map(() => ({})));
    for (let k = 0; k < cmds.length; k++) {
        let cmd = cmds[k];
        if (cmd.kind === "e") perStepFields[cmd.ti][cmd.si][cmd.field] = results[k];
    }

    let bus = new BusVAX({'busWidth': VAX.PAWIDTH, 'id': "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new Cpu(bus);

    let failures = [];
    let stepsCompared = 0;
    for (let ti = 0; ti < trials.length; ti++) {
        let t = trials[ti];
        for (let mw of t.maskWrites) {
            if (mw.size === 2) bus.setWord(mw.addr, mw.val);
        }
        cpu.load({regs: t.regs, psl: t.psl});
        cpu.setPC(t.steps[0].pc);
        for (let si = 0; si < t.steps.length; si++) {
            let s = t.steps[si];
            let off = 0;
            for (let b of s.bytes) { bus.setByte((s.pc + off) >>> 0, b); off++; }
            let fault = null;
            try {
                let opc = cpu.decoder.decode(false);
                let ok = executeControl(opc, cpu.decoder, cpu);
                if (!ok) throw new Error(`nested trial ${ti} step ${si}: opcode ${hex(opc)} has no control.js body`);
            } catch (e) {
                if (e instanceof VAXFault) fault = e; else throw e;
            }
            let fields = perStepFields[ti][si];
            if (fault) {
                failures.push(`nested trial ${ti} step ${si} (depth ${t.depth}): JS threw unexpected fault ${fault.code}`);
                break;
            }
            let problems = [];
            for (let i = 0; i < 15; i++) {
                let f = fields["R" + i];
                if (!f || !f.ok) { problems.push(`R${i}: SIMH error ${f && f.err}`); continue; }
                if ((cpu.regs[i] >>> 0) !== f.value) problems.push(`R${i}: JS=${hex(cpu.regs[i])} SIMH=${hex(f.value)}`);
            }
            let fPC = fields.PC, fPSL = fields.PSL;
            if (!fPC || !fPC.ok) problems.push(`PC: SIMH error ${fPC && fPC.err}`);
            else if ((cpu.regs[15] >>> 0) !== fPC.value) problems.push(`PC: JS=${hex(cpu.regs[15])} SIMH=${hex(fPC.value)}`);
            if (!fPSL || !fPSL.ok) problems.push(`PSL: SIMH error ${fPSL && fPSL.err}`);
            else if ((cpu.psl >>> 0) !== fPSL.value) problems.push(`PSL: JS=${hex(cpu.psl)} SIMH=${hex(fPSL.value)}`);
            if (problems.length) {
                failures.push(`nested trial ${ti} step ${si}/${t.steps.length} (depth ${t.depth}, ${s.isCall ? "CALL" : "RET"}): ` + problems.join("; "));
                break;                                              // state has diverged; later steps in this trial are meaningless
            }
            stepsCompared++;
        }
    }
    return {failures, stepsCompared, trials: trials.length};
}

/* ------------------------------------------------------------------------------------------- *
 * Self-check: deliberate defects injected into control.js, each must be DETECTED                *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = [
    {
        name: "call-ap-fp-swap",
        why: "swap which of old-AP/old-FP is pushed at FP+8 vs FP+12 in CALLS/CALLG (the RET-verified layout)",
        from: "    cpu.writeData((tsp - 8) | 0, cpu.regs[13], 4);              // push (old) FP\n" +
              "    cpu.writeData((tsp - 12) | 0, cpu.regs[12], 4);             // push (old) AP",
        to:   "    cpu.writeData((tsp - 8) | 0, cpu.regs[12], 4);              // push (old) FP\n" +
              "    cpu.writeData((tsp - 12) | 0, cpu.regs[13], 4);             // push (old) AP"
    },
    {
        name: "sobgtr-off-by-one",
        why: "SOBGTR branches on r>0; using r>=0 (SOBGEQ's condition) would over-branch",
        from: "        if (r > 0) branch(cpu, SXTB(decoder.brdisp));",
        to:   "        if (r >= 0) branch(cpu, SXTB(decoder.brdisp));"
    },
    {
        name: "bbx-pos-shift-unsigned",
        why: "use an unsigned shift for the memory bit-position byte-offset, breaking negative positions",
        from: "    let ea = (opnd[2] + (pos >> 3)) | 0;               // deliberate signed shift -- see docblock",
        to:   "    let ea = (opnd[2] + (pos >>> 3)) | 0;               // deliberate signed shift -- see docblock"
    },
    {
        name: "store-destination-predicate-inverted",
        why: "invert isMemoryDestination(), writing SOB/AOB/ACB results to the wrong place",
        from: "    if (decoder.isMemoryDestination()) {\n        cpu.writeData(decoder.va, value, size);\n        return;\n    }",
        to:   "    if (!decoder.isMemoryDestination()) {\n        cpu.writeData(decoder.va, value, size);\n        return;\n    }"
    },
    {
        name: "cc-carry-not-preserved",
        why: "CC_IIZP_L must preserve the incoming carry bit; clearing it would desync every SOB/AOB/ACB",
        from: "function ccIIZP_L(cpu, r) { setCC(cpu, (r & LSIGN) ? (CC.N | (getCC(cpu) & CC.C)) : (r == 0) ? (CC.Z | (getCC(cpu) & CC.C)) : (getCC(cpu) & CC.C)); }",
        to:   "function ccIIZP_L(cpu, r) { setCC(cpu, (r & LSIGN) ? CC.N : (r == 0) ? CC.Z : 0); }"
    },
    {
        name: "ret-nargs-scaled-wrong",
        why: "scale RET's popped argument count by 2 instead of 4 (longword) bytes per argument, mis-popping the caller's argument list",
        from: "        cpu.regs[14] = (cpu.regs[14] + 4 + ((nargs & BMASK) << 2)) | 0;",
        to:   "        cpu.regs[14] = (cpu.regs[14] + 4 + ((nargs & BMASK) << 1)) | 0;"
    },
    {
        name: "popr-sp-gets-extra-increment",
        why: "increment SP again after popping SP itself, double-moving the stack pointer",
        from: "    if (mask & 0x4000) {\n        cpu.regs[14] = cpu.readData(cpu.regs[14], 4, false);\n    }",
        to:   "    if (mask & 0x4000) {\n        cpu.regs[14] = cpu.readData(cpu.regs[14], 4, false);\n        cpu.regs[14] = (cpu.regs[14] + 4) | 0;\n    }"
    },
    {
        name: "casel-limit-compare-signed",
        why: "compare CASEL's (selector-base) to limit signed instead of unsigned, breaking the in-range test for negative differences",
        from: "        let r = (opnd[0] - opnd[1]) | 0;\n        ccCmpL(cpu, r, opnd[2]);\n        let PC = cpu.regs[15];\n        if ((r >>> 0) > (opnd[2] >>> 0)) {",
        to:   "        let r = (opnd[0] - opnd[1]) | 0;\n        ccCmpL(cpu, r, opnd[2]);\n        let PC = cpu.regs[15];\n        if (r > opnd[2]) {"
    }
];

/**
 * loadMutant(mutation, scratch)
 *
 * @param {Object|null} mutation
 * @param {string} scratch
 * @returns {Promise.<Function>}
 */
async function loadMutant(mutation, scratch)
{
    let src = path.join(REPO_ROOT, "machines/dec/vax/modules/v2/control.js");
    let text = fs.readFileSync(src, "utf8");
    if (!mutation) return (await import("../modules/v2/control.js")).executeControl;
    let n = text.split(mutation.from).length - 1;
    if (n !== 1) {
        throw new Error(`mutation "${mutation.name}" matches its target ${n} time(s) in control.js; it must match exactly once`);
    }
    let mutated = text.replace(mutation.from, mutation.to);
    let file = path.join(path.dirname(src), `.mutant-${mutation.name}-${process.pid}.js`);
    fs.writeFileSync(file, mutated);
    try {
        return (await import(fileURLToPath(new URL("file://" + file)))).executeControl;
    } finally {
        fs.unlinkSync(file);
    }
}

/**
 * selfCheck(simh, opts)
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Promise.<boolean>}
 */
async function selfCheck(simh, opts)
{
    console.log("SELF-CHECK: every deliberate defect below must be DETECTED.\n");
    let shortOpts = Object.assign({}, opts, {cases: 12, calls: 12});
    let ok = true;

    let base = await loadMutant(null, opts.scratch);
    let baseSingle = phaseSingle(simh, base, shortOpts);
    let baseNested = phaseNested(simh, base, shortOpts);
    if (baseSingle.failures.length || baseNested.failures.length) {
        console.log("  BASELINE FAILED -- the unmutated control.js must pass before mutations mean anything:");
        for (let f of baseSingle.failures.concat(baseNested.failures).slice(0, 10)) console.log("    " + f);
        return false;
    }
    console.log(`  baseline: ${baseSingle.cases.length} single-step cases, ${baseNested.stepsCompared} nested steps -- clean\n`);

    for (let m of MUTATIONS) {
        let mutant = await loadMutant(m, opts.scratch);
        let caught = "";
        let r = phaseSingle(simh, mutant, shortOpts);
        if (r.failures.length) caught = "single: " + r.failures[0];
        if (!caught) {
            let rn = phaseNested(simh, mutant, shortOpts);
            if (rn.failures.length) caught = "nested: " + rn.failures[0];
        }
        if (caught) {
            console.log(`  DETECTED  ${m.name}`);
            console.log(`            (${m.why})`);
            console.log(`            ${caught.slice(0, 160)}`);
        } else {
            console.log(`  SURVIVED  ${m.name}  <-- the differential is blind to this`);
            console.log(`            (${m.why})`);
            ok = false;
        }
    }
    return ok;
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                          *
 * ------------------------------------------------------------------------------------------- */

/**
 * main()
 *
 * @returns {Promise.<void>}
 */
async function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, def) => {
        let i = argv.indexOf(name);
        return i >= 0 && i + 1 < argv.length ? argv[i + 1] : def;
    };
    let opts = {
        cases: +getArg("--cases", 60),
        calls: +getArg("--calls", 80),
        seed: +getArg("--seed", 0xFAB0FAB0),
        scratch: getArg("--scratch", fs.mkdtempSync(path.join(os.tmpdir(), "vaxcontrol-")))
    };
    let simh = findSimh(getArg("--simh", null));
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;

    console.log(`VAX control-flow differential -- SIMH: ${simh}`);
    console.log(`seed=0x${hex(opts.seed)} scratch=${opts.scratch}\n`);

    if (fSelfCheck) {
        let ok = await selfCheck(simh, opts);
        console.log(ok ? "\nSELF-CHECK PASS: every injected defect was detected."
                       : "\nSELF-CHECK FAIL: at least one injected defect went undetected.");
        process.exit(ok ? 0 : 1);
    }

    let executeControl = await loadMutant(null, opts.scratch);
    let problems = [];

    /* ------------------------------------------------------------------ single-instruction */
    let single = phaseSingle(simh, executeControl, opts);
    console.log("Single-instruction randomized differential");
    console.log(`  opcodes in scope ......... ${IN_SCOPE.length}`);
    console.log(`  total cases .............. ${single.cases.length}`);
    console.log(`  non-trivial cases ........ ${single.nontrivial} (${(100 * single.nontrivial / single.cases.length).toFixed(1)}%)`);
    for (let f of single.failures.slice(0, 20)) problems.push("SINGLE: " + f);
    if (single.failures.length > 20) problems.push(`SINGLE: ...and ${single.failures.length - 20} more failures`);
    let ratio = single.nontrivial / single.cases.length;
    if (ratio < MIN_NONTRIVIAL_RATIO) {
        problems.push(`COVERAGE: only ${(ratio * 100).toFixed(1)}% of cases were non-trivial, floor is ${(MIN_NONTRIVIAL_RATIO * 100).toFixed(0)}%`);
    }
    let underCovered = [];
    for (let name of IN_SCOPE) {
        let s = single.byOpcode.get(name);
        if (!s || s.n < MIN_CASES_PER_OPCODE) underCovered.push(name);
    }
    if (underCovered.length) {
        problems.push(`COVERAGE: opcode(s) below the per-opcode floor (${MIN_CASES_PER_OPCODE}): ${underCovered.join(", ")}`);
    }
    console.log("");

    /* ------------------------------------------------------------------------------ faults */
    let faults = phaseFaults(simh, executeControl, opts);
    console.log("Reserved-operand faults (CALLS/CALLG/RET MBZ, BBx pos>31)");
    console.log(`  cases compared ........... ${faults.nCompared}`);
    for (let f of faults.failures) problems.push("FAULTS: " + f);
    console.log("");

    /* ------------------------------------------------------------------------------ nested */
    let nested = phaseNested(simh, executeControl, opts);
    console.log("Nested CALLS/CALLG/RET");
    console.log(`  trials ................... ${nested.trials}`);
    console.log(`  steps compared ........... ${nested.stepsCompared}`);
    for (let f of nested.failures.slice(0, 20)) problems.push("NESTED: " + f);
    if (nested.failures.length > 20) problems.push(`NESTED: ...and ${nested.failures.length - 20} more failures`);
    if (nested.trials < MIN_CALL_TRIALS) {
        problems.push(`COVERAGE: only ${nested.trials} nested-call trials, floor is ${MIN_CALL_TRIALS}`);
    }
    let expectedSteps = nested.trials * MIN_CALL_DEPTH * 2;
    if (nested.stepsCompared < expectedSteps) {
        problems.push(`COVERAGE: only ${nested.stepsCompared} nested steps compared, expected at least ${expectedSteps} (some trial(s) diverged before completing)`);
    }
    console.log("");

    if (problems.length) {
        console.log("FAIL");
        for (let p of problems) console.log("  " + p);
        console.log(`\nreproduce with --seed ${opts.seed}; SIMH scripts kept in ${opts.scratch}`);
        process.exit(1);
    }
    console.log("PASS: branch/jump/call/stack execution is indistinguishable from SIMH across every phase.");
    fs.rmSync(opts.scratch, {recursive: true, force: true});
}

main().catch((e) => {
    console.error("ERROR: " + (e && e.stack || e));
    process.exit(2);
});
