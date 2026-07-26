/**
 * @fileoverview Differential test: VAX string/queue/INDEX/PROBE/NOP execution vs. a real Open SIMH
 *               microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * A differential test of strq.js's 18 instruction bodies against a REAL, EXECUTED Open SIMH
 * microvax3900 -- no fixtures, no golden files.  Three phases:
 *
 *   RANDOMIZED    Every opcode except PROBER/PROBEW: deposit a register file, PSL, and a hot
 *                 buffer region; `step 1`; examine the full register file plus PSL.  Addressing
 *                 is absolute-mode through a preset address for every string/queue/INDEX operand
 *                 (decodediff.js already proved addressing-MODE resolution; this phase exists to
 *                 grade EXECUTION), edge-weighted per opcode family -- see each SPECIAL generator.
 *   PROBE         PROBER/PROBEW need real MMU protection state to exercise their CC_Z failure
 *                 path, which the plain randomized phase (mapen off, virtual==physical) cannot
 *                 reach.  A small, fixed, deliberately-chosen system-space page table (6 pages:
 *                 open RW, kernel-only RW, read-only-all, invalid/TNV, and a good/bad neighbor
 *                 pair for the two-probe cross-page case) is poked into both sides via patch
 *                 0003's `SHOW CPU MMUOP=4` (the same non-instruction IPR-poke mmudiff.js uses --
 *                 NOT a real MTPR execution, so this does not touch pcjsvax-e49's territory), then
 *                 driven the same deposit/step/examine way across every (page, explicit mode
 *                 operand, PSL current mode) combination.
 *   EHKAA         The entire EHKAA diagnostic trace (patch 0002's capture), scanned for NOP,
 *                 INDEX, MOVC3 and MOVC5 instances ONLY -- see "WHY ONLY FOUR OPCODES" below.
 *
 * WHY ONLY FOUR OPCODES IN THE EHKAA PHASE
 * -----------------------------------------
 * Patch 0002's MEMR log captures reads the DECODER issues during specifier resolution, exactly as
 * intdiff.js's own header documents.  Every opcode in this file that reads or writes memory
 * DURING EXECUTION (not specifier resolution) is instead splicing its reads onto a real, but
 * uninitialized (zero-filled), scratch bus -- fine when the outcome does not depend on the DATA
 * read (NOP: no operands; INDEX: pure arithmetic on register/opnd values, no memory access at
 * all; MOVC3/MOVC5: length/address bookkeeping only -- the copied byte VALUES never affect the
 * final register file or PSL).  It is NOT fine for CMPC3/CMPC5/LOCC/SKPC/SCANC/SPANC (the
 * comparison/match RESULT is exactly the data the log does not have) or for INSQUE/REMQUE/INSQHI/
 * INSQTI/REMQHI/REMQTI (the linked-list pointers read back ARE the data the log does not have) or
 * for PROBER/PROBEW (the outcome depends on real MMU/page-table state the trace does not capture
 * either).  Those fourteen opcodes are EHKAA-EXCLUDED here, on purpose, counted and reported, not
 * silently dropped -- exactly the same "memory-mode variable-bit-field instructions... exclusively
 * the randomized phase's job to cover, which it does" precedent intdiff.js's own header already
 * established for its own domain.
 *
 * A SECOND EHKAA guard: any trace record whose PRE-state already has PSL<FPD> set is a genuine
 * mid-string-op RESUME.  decode.js's `decode(fFPD)` has the resume branch built, but this file
 * (like intdiff.js's own EHKAA phase) always calls `decode(false)`, which cannot correctly replay
 * a resume -- it would re-resolve specifiers that were never re-fetched on real hardware.
 * Skipped, counted, reported.  The guard was ORIGINALLY there because strq.js had no `fault_PC` to
 * resume against at all; that half is fixed (pcjsvax-c05) and the resume path is now graded
 * end-to-end by tests/cpudiff.js.  See the guard's own comment for what is left.
 *
 *      node machines/dec/vax/tests/strqdiff.js [options]
 *        --simh PATH        patched microvax3900 (needs patches 0001+0002+0003; else $SIMH_INT_BIN,
 *                            $SIMH_MMU_BIN, $SIMH_BIN, else scratch build)
 *        --cases-per-opcode N   default 150, floor 40
 *        --ehkaa PATH        default ../pcjs-vax/open-simh/VAX/tests/ehkaa.exe
 *        --seed S
 *        --selfcheck         prove the differential detects deliberate defects in strq.js
 *
 * COVERAGE IS ASSERTED, NOT REPORTED -- see MIN_* floors below; an undersized run fails outright.
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import VAXCpu from "../modules/v2/cpu.js";
import { VAX } from "../modules/v2/defines.js";
import { OPCODES } from "../modules/v2/drom.js";
import MMUVAX from "../modules/v2/mmu.js";
import { executeStrq, STRQ_OPCODES, IMPLEMENTED, HANDLERS, PSL_FPD } from "../modules/v2/strq.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

/* ------------------------------------------------------------------------------------------- *
 * PRNG (mulberry32, matching every sibling differential so a failing seed is reproducible)        *
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
function pick(rnd, arr) { return arr[Math.floor(rnd() * arr.length) % arr.length]; }

/* ------------------------------------------------------------------------------------------- *
 * Locating SIMH.  Needs patch 0001 (REGS) + 0002 (decode-replay/MEMR/PREG) for the EHKAA phase,   *
 * and patch 0003 (SHOW CPU MMUOP=) for the PROBE phase's IPR poke.  No fallback to self-           *
 * comparison -- a missing SIMH fails the run.                                                       *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_INT_BIN']) candidates.push(process.env['SIMH_INT_BIN']);
    if (process.env['SIMH_MMU_BIN']) candidates.push(process.env['SIMH_MMU_BIN']);
    if (process.env['SIMH_BIN']) candidates.push(process.env['SIMH_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH built with patches 0001+0002+0003; it has no\n" +
        "fixture fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh\n" +
        "PATH or set $SIMH_INT_BIN.  Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 30 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * Per-case layout.  Buffers can get large (string "large length" cases), so this stride is        *
 * bigger than intdiff.js's/controldiff.js's -- see BUF_POOL below for the length distribution.     *
 * ------------------------------------------------------------------------------------------- */

const CASE_STRIDE = 0x4000;             // 16KB/case
const CASE_BASE = 0x00100000;
const SP_OFFSET = 0x3F00;
const HOT_START = 0x100;
const HOT_END = 0x3E00;                 // ~15.5KB of buffer room per case

/** A second, much bigger stride used ONLY by the small "true max length" mini-batch. */
const BIG_CASE_STRIDE = 0x24000;        // 144KB/case -- enough for a 65535-byte MOVC3/CMPC3/etc.
const BIG_HOT_START = 0x100;
const BIG_HOT_END = 0x23E00;

class Case {
    constructor(mnemonic, opc, index, stride, hotStart, hotEnd)
    {
        this.mnemonic = mnemonic;
        this.opc = opc;
        this.index = index;
        this.stride = stride || CASE_STRIDE;
        this.hotStartOff = hotStart != null ? hotStart : HOT_START;
        this.hotEndOff = hotEnd != null ? hotEnd : HOT_END;
        this.base = (CASE_BASE + index * this.stride) | 0;
        this.pc = this.base;
        this.instr = [];
        this.mem = [];
        this.regs = new Int32Array(16);
        this.psl = 0;
        this.hotCursor = this.base + this.hotStartOff;
        this.trivial = true;
        this.usedRegs = new Set();
    }

    /** Bump the cursor by exactly n bytes, no forced alignment -- lets a caller control whether a
        buffer starts aligned or not, which is exactly what exercises MOVC's head/align/tail split. */
    allocRaw(n)
    {
        let a = this.hotCursor;
        this.hotCursor = (this.hotCursor + n) | 0;
        if (this.hotCursor > this.base + this.hotEndOff) throw new Error(`case ${this.index} (${this.mnemonic}): hot region overflow (need ${n} more bytes)`);
        return a;
    }

    /** Bump the cursor to the next longword boundary, then reserve n bytes -- for operands whose
        specifier width matters (e.g. INSQHI/INSQTI/REMQHI/REMQTI quadword alignment). */
    allocAligned(n, align)
    {
        align = align || 4;
        this.hotCursor = (this.hotCursor + align - 1) & ~(align - 1);
        return this.allocRaw(n);
    }

    setMemL(addr, val) { this.mem.push({addr: addr | 0, lnt: 4, val: val | 0}); }
    setMemW(addr, val) { this.mem.push({addr: addr | 0, lnt: 2, val: val & 0xFFFF}); }
    setMemB(addr, val) { this.mem.push({addr: addr | 0, lnt: 1, val: val & 0xFF}); }

    byte(b) { this.instr.push(b & 0xFF); }
    long(v) { v = v | 0; this.instr.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF); }
    word(v) { this.instr.push(v & 0xFF, (v >>> 8) & 0xFF); }

    get instrLen() { return this.instr.length; }
}

const REG_POOL = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

function pickFreeReg(c, rnd, maxIncl)
{
    let candidates = REG_POOL.filter((r) => r <= maxIncl && !c.usedRegs.has(r));
    if (!candidates.length) candidates = REG_POOL.filter((r) => r <= maxIncl);
    let rn = pick(rnd, candidates);
    c.usedRegs.add(rn);
    return rn;
}

/** emitRead(c, rnd, width, value) -- R-type specifier: register-direct or PC-relative immediate. */
function emitRead(c, rnd, width, value)
{
    let asReg = rnd() < 0.5;
    if (asReg) {
        let rn = pickFreeReg(c, rnd, 13);
        c.byte(0x50 | rn);
        c.regs[rn] = width === 1 ? (value & 0xFF) : width === 2 ? (value & 0xFFFF) : (value | 0);
        return;
    }
    c.byte(0x8F);
    if (width === 1) c.byte(value & 0xFF);
    else if (width === 2) c.word(value & 0xFFFF);
    else c.long(value | 0);
}

/** emitAddr(c, addr) -- A-type specifier: absolute mode through a preset address (strings/queues
    have no register form for their address operands, matching genGeneric's precedent elsewhere
    in this project for "A" specifiers). */
function emitAddr(c, addr)
{
    c.byte(0x9F);
    c.long(addr);
}

/** emitWrite(c, rnd, forceMem) -- W.l specifier: register-direct or absolute-through-fresh-address.
    @returns {{mem: boolean, rn: number, addr: number}} */
function emitWrite(c, rnd, forceMem)
{
    let asReg = !forceMem && rnd() < 0.42;
    if (asReg) {
        let rn = pickFreeReg(c, rnd, 13);
        c.byte(0x50 | rn);
        return {mem: false, rn, addr: 0};
    }
    let a = c.allocAligned(4);
    c.byte(0x9F);
    c.long(a);
    return {mem: true, rn: -1, addr: a};
}

/* ------------------------------------------------------------------------------------------- *
 * Edge-weighted value pools                                                                        *
 * ------------------------------------------------------------------------------------------- */

function randL(rnd)
{
    if (rnd() < 0.4) return pick(rnd, [0, 1, 0x7FFFFFFF, 0x80000000, -1, 2, 0x80000001 | 0, -2]) | 0;
    return (Math.floor(rnd() * 0x100000000)) | 0;
}
function randSmallL(rnd) { return pick(rnd, [0, 1, -1, 10, -10, 100, -100, 1000, -1000]); }

/** String/table length pool: zero, tiny, small, medium, and multi-KB "large" -- see the file
    header's note on true-max-length being covered separately by the BIG_CASE mini-batch. */
function pickLen(rnd)
{
    let r = rnd();
    if (r < 0.15) return 0;
    if (r < 0.30) return pick(rnd, [1, 2, 3, 4]);
    if (r < 0.55) return Math.floor(rnd() * 61) + 5;             // 5-65
    if (r < 0.85) return Math.floor(rnd() * 400) + 66;           // 66-465
    return Math.floor(rnd() * 3000) + 466;                       // 466-3465
}

/** Skew the cursor by 0-3 bytes before allocating -- exercises MOVC3/MOVC5's head/align/tail
    alignment-loop split against every possible starting alignment. */
function allocSkewed(c, rnd, len)
{
    let skew = pick(rnd, [0, 1, 2, 3]);
    if (skew) c.allocRaw(skew);
    return c.allocRaw(Math.max(len, 1));
}

function allocQuadSafe(c, n) { return c.allocAligned(n, 8); }

/* ------------------------------------------------------------------------------------------- *
 * Per-opcode case generators                                                                       *
 * ------------------------------------------------------------------------------------------- */

const SPECIAL = {};

SPECIAL.NOP = (opc, rnd, index) => {
    let c = new Case("NOP", opc, index);
    c.byte(opc & 0xFF);
    return c;
};

/** INDEX: subscript.rl, low.rl, high.rl, size.rl, indexin.rl, result.wl.  Weighted so subscript
    lands ON each bound, INSIDE the range, and on both sides OUTSIDE it (the TRAP_SUBSCR case). */
SPECIAL.INDEX = (opc, rnd, index) => {
    let c = new Case("INDEX", opc, index);
    c.byte(opc & 0xFF);
    let low = randSmallL(rnd);
    let span = pick(rnd, [0, 1, 5, 100, Math.floor(rnd() * 100000)]);
    let high = (low + span) | 0;
    let r = rnd(), subscript;
    if (r < 0.30) subscript = low;
    else if (r < 0.50) subscript = high;
    else if (r < 0.70) subscript = (low + Math.floor(rnd() * Math.max(1, (high - low)))) | 0;
    else if (r < 0.85) subscript = (low - 1 - Math.floor(rnd() * 5)) | 0;
    else subscript = (high + 1 + Math.floor(rnd() * 5)) | 0;
    let size = pick(rnd, [0, 1, 2, 4, 8, Math.floor(rnd() * 1000) | 0]);
    let indexin = randL(rnd);
    emitRead(c, rnd, 4, subscript);
    emitRead(c, rnd, 4, low);
    emitRead(c, rnd, 4, high);
    emitRead(c, rnd, 4, size);
    emitRead(c, rnd, 4, indexin);
    emitWrite(c, rnd, false);
    /* Every field is edge-weighted (low/high/size/indexin are never uniformly zero), so the
       arithmetic result and/or the subscript-range trap decision are essentially always
       meaningfully varied -- unlike a generic opcode where a register destination and a boring
       zero operand can coincide, INDEX has no analogous "definitely boring" case here. */
    c.trivial = false;
    return c;
};

/** INSQUE: entry.ab, pred.ab.  All addresses drawn from THIS CASE's own safe allocations, so
    every read/write INSQUE performs lands inside mapped memory (mapen is off in this phase --
    execution-time faults are out of scope, matching every sibling differential's convention). */
SPECIAL.INSQUE = (opc, rnd, index) => {
    let c = new Case("INSQUE", opc, index);
    c.byte(opc & 0xFF);
    let e = c.allocAligned(8);
    let p = c.allocAligned(8);
    let sRegion = c.allocAligned(8);
    let sVal = pick(rnd, [e, p, sRegion, (p + 4) | 0, (e + 4) | 0]);
    c.setMemL(p, sVal);
    emitAddr(c, e);
    emitAddr(c, p);
    c.trivial = false;
    return c;
};

/** REMQUE: entry.ab, dest.wl.  (e) and (e+4) are the "s"/"p" pointers REMQUE reads AND (when the
    queue is non-empty) writes THROUGH -- both must be safe addresses, so both are drawn from this
    case's own allocations, covering empty (s===p===e), and non-empty in both CC directions. */
SPECIAL.REMQUE = (opc, rnd, index) => {
    let c = new Case("REMQUE", opc, index);
    c.byte(opc & 0xFF);
    let e = c.allocAligned(8);
    let sRegion = c.allocAligned(8);
    let pRegion = c.allocAligned(8);
    let mode = pick(rnd, ["empty", "nonEmpty"]);
    let sVal, pVal;
    if (mode === "empty") { sVal = e; pVal = e; }
    else {
        sVal = pick(rnd, [sRegion, pRegion, e]);
        pVal = pick(rnd, [sRegion, pRegion]);           // != e, so the queue is genuinely non-empty
    }
    c.setMemL(e, sVal);
    c.setMemL((e + 4) | 0, pVal);
    emitAddr(c, e);
    let w = emitWrite(c, rnd, false);
    c.trivial = mode !== "empty" || w.mem;
    return c;
};

/** INSQHI/INSQTI: entry.ab, header.aq.  Header content is EMPTY (0), BUSY (interlock bit set), or
    a quad-aligned relative displacement to a safe neighbor entry -- all three of op_insqhi's/
    op_insqti's branches, with no address ever leaving this case's own quad-aligned allocations
    (so a legality check this generator does not intend to exercise never fires). */
function genQueueIns(opc, rnd, index, mn)
{
    let c = new Case(mn, opc, index);
    c.byte(opc & 0xFF);
    let h = allocQuadSafe(c, 8);
    let d = allocQuadSafe(c, 8);
    let target = allocQuadSafe(c, 8);
    let mode = pick(rnd, ["empty", "nonEmpty", "busy"]);
    let aVal = mode === "empty" ? 0 : mode === "busy" ? 1 : (target - h) | 0;
    c.setMemL(h, aVal);
    emitAddr(c, d);
    emitAddr(c, h);
    c.trivial = mode !== "busy";
    return c;
}
SPECIAL.INSQHI = (opc, rnd, index) => genQueueIns(opc, rnd, index, "INSQHI");
SPECIAL.INSQTI = (opc, rnd, index) => genQueueIns(opc, rnd, index, "INSQTI");

/** REMQHI/REMQTI: header.aq, dest.wl.  Empty, busy, single-entry (tail wraps to head), and
    multi-entry (tail reaches a THIRD safe quad-aligned entry) -- covers both op_remqhi's and
    op_remqti's branches (REMQHI never reads (h+4); presetting it anyway is harmless). */
function genQueueRem(opc, rnd, index, mn)
{
    let c = new Case(mn, opc, index);
    c.byte(opc & 0xFF);
    let h = allocQuadSafe(c, 8);
    let entryA = allocQuadSafe(c, 8);
    let entryB = allocQuadSafe(c, 8);
    let mode = pick(rnd, ["empty", "busy", "single", "multi"]);
    let arVal = 0, tailVal = 0;
    if (mode === "empty") { arVal = 0; tailVal = 0; }
    else if (mode === "busy") { arVal = 1; tailVal = 0; }
    else if (mode === "single") {
        arVal = (entryA - h) | 0;
        tailVal = (entryA - h) | 0;
        c.setMemL(entryA, (h - entryA) | 0);
    } else {
        arVal = (entryA - h) | 0;
        tailVal = (entryB - h) | 0;
        c.setMemL(entryA, (entryB - entryA) | 0);
        c.setMemL(entryB, (h - entryB) | 0);
    }
    c.setMemL(h, arVal);
    c.setMemL((h + 4) | 0, tailVal);
    emitAddr(c, h);
    let w = emitWrite(c, rnd, false);
    c.trivial = mode === "busy" ? true : (w.mem || true);
    return c;
}
SPECIAL.REMQHI = (opc, rnd, index) => genQueueRem(opc, rnd, index, "REMQHI");
SPECIAL.REMQTI = (opc, rnd, index) => genQueueRem(opc, rnd, index, "REMQTI");

/** MOVC3/MOVC5: length.rw [srclen/dstlen for MOVC5], src.ab [fill.rb, dstlen.rw for MOVC5],
    dst.ab.  The overlap dimension is the architecturally interesting one -- SIMH picks forward vs.
    backward copy direction based on address comparison, and this is the ONLY way to prove that
    choice is correct: a non-overlapping case can't distinguish "copied forward" from "copied
    backward", but an overlapping one corrupts the result if the wrong direction is chosen. */
function genMovc(opc, rnd, index, five)
{
    let c = new Case(five ? "MOVC5" : "MOVC3", opc, index);
    c.byte(opc & 0xFF);
    let srclen = pickLen(rnd);
    let dstlen = five ? pickLen(rnd) : srclen;
    let big = Math.max(srclen, dstlen);
    let overlap = pick(rnd, ["separate", "forward", "backward", "same", "adjacent"]);
    let src, dst;
    if (overlap === "separate") {
        src = allocSkewed(c, rnd, srclen);
        dst = allocSkewed(c, rnd, dstlen);
    } else if (overlap === "same") {
        src = allocSkewed(c, rnd, big);
        dst = src;
    } else if (overlap === "adjacent") {
        src = allocSkewed(c, rnd, big);
        dst = (src + Math.max(big, 1)) | 0;
        c.allocRaw(Math.max(dstlen, 1));
    } else {
        let shift = Math.max(1, Math.floor(rnd() * Math.max(1, Math.min(big, 32))));
        let bufLen = big + shift + 4;
        let base = allocSkewed(c, rnd, bufLen);
        if (overlap === "forward") { src = base; dst = (base + shift) | 0; }
        else { dst = base; src = (base + shift) | 0; }
    }
    emitRead(c, rnd, 2, srclen);
    emitAddr(c, src);
    if (five) {
        let fill = pick(rnd, [0, 0x20, 0xFF, Math.floor(rnd() * 256)]);
        emitRead(c, rnd, 1, fill);
        emitRead(c, rnd, 2, dstlen);
        emitAddr(c, dst);
    } else {
        emitAddr(c, dst);
    }
    for (let i = 0; i < srclen; i++) c.setMemB((src + i) | 0, (i * 37 + 11) & 0xFF);
    c.trivial = big <= 0;
    return c;
}
SPECIAL.MOVC3 = (opc, rnd, index) => genMovc(opc, rnd, index, false);
SPECIAL.MOVC5 = (opc, rnd, index) => genMovc(opc, rnd, index, true);

/** CMPC3/CMPC5: content-driven -- equal, mismatch at the first byte, mismatch mid-string, and
    (CMPC5 only) mismatch confined to the fill-extended tail. */
function genCmpc(opc, rnd, index, five)
{
    let c = new Case(five ? "CMPC5" : "CMPC3", opc, index);
    c.byte(opc & 0xFF);
    let len1 = pickLen(rnd);
    let len2 = five ? pickLen(rnd) : len1;
    let fill = five ? pick(rnd, [0, 0x20, 0xFF, Math.floor(rnd() * 256)]) : 0;
    let src1 = allocSkewed(c, rnd, len1);
    let src2 = allocSkewed(c, rnd, len2);
    let mismatchMode = pick(rnd, ["equal", "mismatchStart", "mismatchMid"]);
    let maxLen = Math.max(len1, len2);
    let bytes1 = new Array(maxLen), bytes2 = new Array(maxLen);
    for (let i = 0; i < maxLen; i++) {
        let v = (i * 17 + 1) & 0xFF;
        bytes1[i] = i < len1 ? v : fill;
        bytes2[i] = i < len2 ? v : fill;
    }
    if (mismatchMode === "mismatchStart" && len1 > 0 && len2 > 0) {
        bytes1[0] = (bytes1[0] ^ 0xFF) & 0xFF;
    } else if (mismatchMode === "mismatchMid" && maxLen > 2) {
        let mid = Math.floor(maxLen / 2);
        bytes1[mid] = (bytes1[mid] ^ 0xFF) & 0xFF;
    }
    for (let i = 0; i < len1; i++) c.setMemB((src1 + i) | 0, bytes1[i]);
    for (let i = 0; i < len2; i++) c.setMemB((src2 + i) | 0, bytes2[i]);
    emitRead(c, rnd, 2, len1);
    emitAddr(c, src1);
    if (five) {
        emitRead(c, rnd, 1, fill);
        emitRead(c, rnd, 2, len2);
        emitAddr(c, src2);
    } else {
        emitAddr(c, src2);
    }
    c.trivial = maxLen <= 0;
    return c;
}
SPECIAL.CMPC3 = (opc, rnd, index) => genCmpc(opc, rnd, index, false);
SPECIAL.CMPC5 = (opc, rnd, index) => genCmpc(opc, rnd, index, true);

/** LOCC/SKPC: match.rb, len.rw, src.ab.  The match byte is placed at the first, middle, last, or
    no position -- LOCC/SKPC differ only in which outcome (found vs. not-found) sets CC.Z, so both
    opcodes need both outcomes covered to actually distinguish their `^ skpc` XOR from a stub. */
function genLocskp(opc, rnd, index, mn)
{
    let c = new Case(mn, opc, index);
    c.byte(opc & 0xFF);
    let len = pickLen(rnd);
    let match = Math.floor(rnd() * 256);
    let src = allocSkewed(c, rnd, len);
    for (let i = 0; i < len; i++) {
        let v = (match + 1 + i) & 0xFF;
        if (v === match) v = (v + 1) & 0xFF;
        c.setMemB((src + i) | 0, v);
    }
    let mode = pick(rnd, ["none", "first", "mid", "last"]);
    if (len > 0 && mode !== "none") {
        let pos = mode === "first" ? 0 : mode === "last" ? len - 1 : Math.floor(len / 2);
        c.setMemB((src + pos) | 0, match);
    }
    emitRead(c, rnd, 1, match);
    emitRead(c, rnd, 2, len);
    emitAddr(c, src);
    c.trivial = len <= 0;
    return c;
}
SPECIAL.LOCC = (opc, rnd, index) => genLocskp(opc, rnd, index, "LOCC");
SPECIAL.SKPC = (opc, rnd, index) => genLocskp(opc, rnd, index, "SKPC");

/** SCANC/SPANC: len.rw, src.ab, table.ab, mask.rb.  The 256-byte table is randomized per case (so
    every mask bit combination gets exercised across the run) and the source is filled with
    "no-hit" bytes (table[byte]&mask===0) with a deliberately placed "hit" byte at the first,
    middle, last, or no position -- the exact dual SCANC (find a hit) and SPANC (find a non-hit,
    i.e. this generator's DEFAULT fill) need. */
function genScnspn(opc, rnd, index, mn)
{
    let c = new Case(mn, opc, index);
    c.byte(opc & 0xFF);
    let len = pickLen(rnd);
    let mask = pick(rnd, [0x01, 0x80, 0xFF, Math.floor(rnd() * 256)]) || 1;
    let table = c.allocAligned(256);
    let tbl = new Uint8Array(256);
    for (let i = 0; i < 256; i++) {
        tbl[i] = (rnd() < 0.3) ? ((mask | Math.floor(rnd() * 256)) & 0xFF) : ((Math.floor(rnd() * 256)) & ~mask & 0xFF);
    }
    for (let i = 0; i < 256; i++) c.setMemB((table + i) | 0, tbl[i]);
    let src = allocSkewed(c, rnd, len);
    let safeBytes = [], hitBytes = [];
    for (let b = 0; b < 256; b++) { if ((tbl[b] & mask) === 0) safeBytes.push(b); else hitBytes.push(b); }
    if (!safeBytes.length) safeBytes = [0];
    if (!hitBytes.length) hitBytes = [255];
    for (let i = 0; i < len; i++) c.setMemB((src + i) | 0, pick(rnd, safeBytes));
    let mode = pick(rnd, ["none", "first", "mid", "last"]);
    if (len > 0 && mode !== "none") {
        let pos = mode === "first" ? 0 : mode === "last" ? len - 1 : Math.floor(len / 2);
        c.setMemB((src + pos) | 0, pick(rnd, hitBytes));
    }
    emitRead(c, rnd, 2, len);
    emitAddr(c, src);
    emitAddr(c, table);
    emitRead(c, rnd, 1, mask);
    c.trivial = len <= 0;
    return c;
}
SPECIAL.SCANC = (opc, rnd, index) => genScnspn(opc, rnd, index, "SCANC");
SPECIAL.SPANC = (opc, rnd, index) => genScnspn(opc, rnd, index, "SPANC");

/* PROBER/PROBEW are NOT in SPECIAL -- they are handled by the dedicated phaseProbe() below, which
   needs a real page table this generic per-opcode flow has no way to provide.  See RANDOMIZED_MN. */

/** Mnemonics the RANDOMIZED phase iterates -- IMPLEMENTED minus PROBER/PROBEW. */
const RANDOMIZED_MN = IMPLEMENTED.filter((mn) => mn !== "PROBER" && mn !== "PROBEW").sort();

const OPC_OF = {};
for (let opc = 0; opc < 512; opc++) if (OPCODES[opc]) OPC_OF[OPCODES[opc]] = opc;

function finishCase(c, rnd)
{
    c.regs[15] = c.pc;
    c.regs[14] = (c.base + SP_OFFSET) | 0;
    for (let r = 0; r < 14; r++) if (!c.usedRegs.has(r) && rnd() < 0.5) c.regs[r] = randL(rnd);
    let cc = Math.floor(rnd() * 16);
    c.psl = (c.psl & ~0xF) | cc;
    c.expectPCAfter = (c.pc + c.instr.length) | 0;
    return c;
}

function genCase(mnemonic, rnd, index)
{
    let opc = OPC_OF[mnemonic];
    let c = SPECIAL[mnemonic](opc, rnd, index);
    return finishCase(c, rnd);
}

/* ------------------------------------------------------------------------------------------- *
 * TRUE MAX-LENGTH mini-batch -- a handful of dedicated 65535-byte cases (the real architectural   *
 * ceiling for a .rw length field) using the much bigger BIG_CASE_STRIDE, so the bulk of the         *
 * randomized phase can stay at a cheap per-case footprint while the true max is still proven,       *
 * not just approximated by the "large" bucket in pickLen().                                          *
 * ------------------------------------------------------------------------------------------- */

const MAXLEN_MN = ["MOVC3", "MOVC5", "CMPC3", "CMPC5", "LOCC", "SKPC", "SCANC", "SPANC"];
const MAXLEN = 0xFFFF;

function genMaxLenCase(mnemonic, rnd, index)
{
    let opc = OPC_OF[mnemonic];
    let c = new Case(mnemonic, opc, index, BIG_CASE_STRIDE, BIG_HOT_START, BIG_HOT_END);
    c.byte(opc & 0xFF);
    if (mnemonic === "MOVC3" || mnemonic === "MOVC5") {
        let five = mnemonic === "MOVC5";
        let src = allocSkewed(c, rnd, MAXLEN);
        let dst = allocSkewed(c, rnd, MAXLEN);
        emitRead(c, rnd, 2, MAXLEN);
        emitAddr(c, src);
        if (five) {
            emitRead(c, rnd, 1, pick(rnd, [0, 0xFF]));
            emitRead(c, rnd, 2, MAXLEN);
            emitAddr(c, dst);
        } else {
            emitAddr(c, dst);
        }
        c.trivial = false;
    } else if (mnemonic === "CMPC3" || mnemonic === "CMPC5") {
        let five = mnemonic === "CMPC5";
        let src1 = allocSkewed(c, rnd, MAXLEN);
        let src2 = allocSkewed(c, rnd, MAXLEN);
        for (let i = 0; i < MAXLEN; i++) { c.setMemB((src1 + i) | 0, i & 0xFF); c.setMemB((src2 + i) | 0, i & 0xFF); }
        c.setMemB((src1 + MAXLEN - 1) | 0, 0xFF & ~(MAXLEN - 1));    // guarantee a tail mismatch
        emitRead(c, rnd, 2, MAXLEN);
        emitAddr(c, src1);
        if (five) {
            emitRead(c, rnd, 1, 0);
            emitRead(c, rnd, 2, MAXLEN);
            emitAddr(c, src2);
        } else {
            emitAddr(c, src2);
        }
        c.trivial = false;
    } else if (mnemonic === "LOCC" || mnemonic === "SKPC") {
        let src = allocSkewed(c, rnd, MAXLEN);
        for (let i = 0; i < MAXLEN; i++) c.setMemB((src + i) | 0, 0);
        c.setMemB((src + MAXLEN - 1) | 0, 0xAA);
        emitRead(c, rnd, 1, 0xAA);
        emitRead(c, rnd, 2, MAXLEN);
        emitAddr(c, src);
        c.trivial = false;
    } else {
        let src = allocSkewed(c, rnd, MAXLEN);
        let table = c.allocAligned(256);
        for (let i = 0; i < 256; i++) c.setMemB((table + i) | 0, 0);
        c.setMemB((table + 0xAA) | 0, 0x01);
        for (let i = 0; i < MAXLEN; i++) c.setMemB((src + i) | 0, 0);
        c.setMemB((src + MAXLEN - 1) | 0, 0xAA);
        emitRead(c, rnd, 2, MAXLEN);
        emitAddr(c, src);
        emitAddr(c, table);
        emitRead(c, rnd, 1, 0x01);
        c.trivial = false;
    }
    return finishCase(c, rnd);
}

/* ------------------------------------------------------------------------------------------- *
 * JS-side execution.  Not cpu.step()/cpu.execute() -- those only dispatch cpu.js's OWN 107          *
 * opcodes (see strq.js's file header and cpu.js's execute(): "not implemented by this item").       *
 * Every sibling execution module's differential (controldiff.js's runJsCase) calls decode() and     *
 * its own module's execute function directly instead, and this file does the same.                   *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE_DEFAULT = 0x04000000;      // 64MB floor, matching memSizeM's Math.max(64, ...) below

/**
 * makeMachine(memSizeBytes)
 *
 * The JS-side bus MUST cover at least as much address space as the SIMH-side `set cpu Nm` this
 * run computed -- a mismatch here would make out-of-range cases (an oversized --cases-per-opcode)
 * silently valid on one side and out-of-bounds on the other instead of failing loudly.
 *
 * @param {number} [memSizeBytes]
 */
function makeMachine(memSizeBytes)
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, memSizeBytes || MEMSIZE_DEFAULT, MemoryVAX.TYPE.RAM);
    let cpu = new VAXCpu(bus);
    return {bus, cpu};
}

function runCaseJS(m, c)
{
    let {bus, cpu} = m;
    cpu.regs.fill(0);
    for (let i = 0; i < 16; i++) cpu.regs[i] = c.regs[i];
    cpu.psl = c.psl;
    cpu.trpirq = 0;
    for (let i = 0; i < c.instr.length; i++) bus.setByte((c.pc + i) | 0, c.instr[i]);
    for (let w of c.mem) {
        if (w.lnt === 4) bus.setLong(w.addr, w.val);
        else if (w.lnt === 2) bus.setWord(w.addr, w.val);
        else bus.setByte(w.addr, w.val);
    }
    try {
        let opc = cpu.decoder.decode(false);
        if (!executeStrq(opc, cpu.decoder, cpu)) {
            return {regs: null, psl: 0, pcOK: false, error: `strq.js does not implement opcode ${hex(opc, 2)} (${OPCODES[opc] || "?"})`};
        }
    } catch (e) {
        return {regs: null, psl: 0, pcOK: false, error: e.message || String(e)};
    }
    let pcOK = cpu.regs[15] === c.expectPCAfter;
    return {regs: Int32Array.from(cpu.regs), psl: cpu.psl, pcOK, error: null};
}

/* ------------------------------------------------------------------------------------------- *
 * SIMH-side batch driver -- same discipline as intdiff.js/controldiff.js/busdiff.js: SIMHALT,       *
 * TRPIRQ/SISR cleared per case (INDEX's TRAP_SUBSCR request must not leak into the next case's       *
 * step), every register and every touched memory location deposited fresh.                            *
 * ------------------------------------------------------------------------------------------- */

const CASE_MARK = "CASE_";

function buildScript(cases, memSizeM)
{
    let L = [`set cpu ${memSizeM}m`, "set cpu simhalt", "reset all"];
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        L.push("deposit TRPIRQ 0");
        L.push("deposit SISR 0");
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
        regList.push("PC", "PSL");
        L.push(`examine -h ${regList.join(",")}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

const REG_LINE_RE = /^(R\d{1,2}|PSL|PC):\s+([0-9A-Fa-f]+)/;

function parseBatchOutput(out)
{
    let lines = out.split("\n");
    let results = new Map();
    let i = 0, n = lines.length;
    while (i < n) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\d+)"));
        if (!m) { i++; continue; }
        let idx = +m[1];
        i++;
        let regs = new Int32Array(16), psl = 0, got = 0;
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

function runBatch(simh, cases, scratch, memSizeM)
{
    let script = buildScript(cases, memSizeM);
    let out = runSimh(simh, script, path.join(scratch, "strqdiff-batch.ini"));
    return parseBatchOutput(out);
}

function compare(mnemonic, c, js, simhResult)
{
    if (js.error) return `${mnemonic} case ${c.index}: strq.js threw during execution: ${js.error}`;
    if (!simhResult || !simhResult.reached) return `${mnemonic} case ${c.index}: SIMH result not reached (case dropped from comparison -- see COVERAGE)`;
    if (!js.pcOK) return `${mnemonic} case ${c.index}: strq.js PC after step = ${hex(js.regs[15])}, expected ${hex(c.expectPCAfter)}`;
    for (let r = 0; r < 16; r++) {
        if (js.regs[r] !== simhResult.regs[r]) {
            return `${mnemonic} case ${c.index}: R${r} mismatch: strq.js=${hex(js.regs[r])} simh=${hex(simhResult.regs[r])} (instr=${c.instr.map((b) => b.toString(16).padStart(2, "0")).join(" ")})`;
        }
    }
    if ((js.psl & 0xF) !== (simhResult.psl & 0xF)) {
        return `${mnemonic} case ${c.index}: CC mismatch: strq.js=${(js.psl & 0xF).toString(2).padStart(4, "0")} simh=${(simhResult.psl & 0xF).toString(2).padStart(4, "0")}`;
    }
    if ((js.psl >>> 4) !== (simhResult.psl >>> 4)) {
        return `${mnemonic} case ${c.index}: PSL (non-CC) mismatch: strq.js=${hex(js.psl)} simh=${hex(simhResult.psl)}`;
    }
    return null;
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors -- FAIL the run, do not scale down with case count.                              *
 * ------------------------------------------------------------------------------------------- */

const MIN_CASES_PER_OPCODE = 40;
const MIN_TOTAL_CASES = 2000;
const MIN_NONTRIVIAL_FRACTION = 0.6;

/* ------------------------------------------------------------------------------------------- *
 * RANDOMIZED phase                                                                                  *
 * ------------------------------------------------------------------------------------------- */

function phaseRandomized(simh, scratch, opts)
{
    let rnd = mulberry32(opts.seed);
    let allCases = [];
    let index = 0;
    for (let mn of RANDOMIZED_MN) {
        for (let k = 0; k < opts.casesPerOpcode; k++) allCases.push(genCase(mn, rnd, index++));
    }

    let memSizeM = Math.max(64, Math.ceil((allCases.length * CASE_STRIDE) / (1024 * 1024)) + 8);

    let failures = [];
    let notReached = [];
    let nontrivial = 0, total = 0;
    const BATCH = 300;
    let m = makeMachine(memSizeM * 1024 * 1024);
    for (let start = 0; start < allCases.length; start += BATCH) {
        let batch = allCases.slice(start, start + BATCH);
        let simhResults = runBatch(simh, batch, scratch, memSizeM);
        for (let c of batch) {
            total++;
            if (!c.trivial) nontrivial++;
            let js = runCaseJS(m, c);
            let sr = simhResults.get(c.index);
            if (!sr || !sr.reached) { notReached.push(`${c.mnemonic} case ${c.index}`); continue; }
            let bad = compare(c.mnemonic, c, js, sr);
            if (bad) failures.push(bad);
        }
    }

    let perOpcodeCounts = new Map();
    for (let c of allCases) perOpcodeCounts.set(c.mnemonic, (perOpcodeCounts.get(c.mnemonic) || 0) + 1);

    return {total, nontrivial, failures, notReached, mnemonics: RANDOMIZED_MN, perOpcodeCounts};
}

/* ------------------------------------------------------------------------------------------- *
 * TRUE MAX-LENGTH mini-batch phase                                                                  *
 * ------------------------------------------------------------------------------------------- */

function phaseMaxLen(simh, scratch, opts)
{
    let rnd = mulberry32(opts.seed ^ 0x5EED);
    let cases = [];
    let index = 0;
    const PER_MN = 3;
    for (let mn of MAXLEN_MN) {
        for (let k = 0; k < PER_MN; k++) cases.push(genMaxLenCase(mn, rnd, index++));
    }
    let memSizeM = Math.max(64, Math.ceil((cases.length * BIG_CASE_STRIDE) / (1024 * 1024)) + 8);
    let m = makeMachine(memSizeM * 1024 * 1024);
    let failures = [], notReached = [], compared = 0;
    const BATCH = 4;                    // each case is ~144KB of deposits -- keep scripts small
    for (let start = 0; start < cases.length; start += BATCH) {
        let batch = cases.slice(start, start + BATCH);
        let simhResults = runBatch(simh, batch, scratch, memSizeM);
        for (let c of batch) {
            let js = runCaseJS(m, c);
            let sr = simhResults.get(c.index);
            if (!sr || !sr.reached) { notReached.push(`${c.mnemonic} case ${c.index} (maxlen)`); continue; }
            compared++;
            let bad = compare(c.mnemonic, c, js, sr);
            if (bad) failures.push("MAXLEN: " + bad);
        }
    }
    return {compared, failures, notReached, expected: MAXLEN_MN.length * PER_MN};
}

/* ------------------------------------------------------------------------------------------- *
 * PROBE phase -- PROBER/PROBEW need REAL MMU protection state to exercise the CC_Z failure path,   *
 * which mapen-off (virtual==physical, used by every other phase in this file, matching the           *
 * mapen-off convention intdiff.js/controldiff.js already established) cannot reach at all: with        *
 * mapping off, mmu.test() always succeeds.  A small, FIXED, hand-chosen system-space page table         *
 * -- not randomized, because the point is deliberate coverage of every protection class, not a          *
 * fuzz of the page-table format itself (that is mmudiff.js's job, already done) -- is poked into         *
 * both sides via patch 0003's `SHOW CPU MMUOP=4` (an IPR poke, NOT a real MTPR execution -- see           *
 * strq.js's file header: MTPR/MFPR stay pcjsvax-e49's).                                                    *
 *                                                                                                            *
 * Both the CODE (instruction bytes) and the PROBE TARGET live in the SAME system-space table --              *
 * running code from P0 would additionally need a working two-level P0 page table (mmudiff.js's own          *
 * SBR+P0BR+P1BR machinery) just so instruction FETCH succeeds once mapen is on; running code from            *
 * S0 instead needs only the ONE flat table this phase already builds, with the code pages given              *
 * fully-open (all-mode RW) protection so fetch/register access never itself faults -- only the                *
 * EXPLICIT PROBER/PROBEW target pages have the interesting (restrictive) protection.                          *
 * ------------------------------------------------------------------------------------------- */

const PAGE = 512;
const IPR = {SBR: 12, SLR: 13, MAPEN: 56};

const PROBE_SBR = 0x00010000;
const PROBE_TARGET_S0_BASE = 0x80000000 | 0;    // vpn 0 of the flat table
const PROBE_TARGET_PFN_BASE = 0x00020000 / PAGE;
const PROBE_CODE_PHYS_BASE = 0x00030000;
const PROBE_CODE_S0_VPN0 = 8;                   // code pages start right after the 6 target pages
const PROBE_MEMSIZE_M = 8;
const PROBE_CASE_STRIDE = PAGE;                 // one case per physical/virtual page, 1:1

/* vpn -> {prot, valid} for the six target pages, vax_mmu.c's cvtacc[] protection codes (see
   mmu.js's own CVTACC table, reproduced by VALUE here rather than imported, matching this
   project's per-file "own your constants" convention):
     4  = RW for all four modes (open)
     2  = RW kernel only (ACV from exec/supv/user)
     15 = R for all four modes, W for none (PROBEW ACV from every mode, PROBER always succeeds)
     (any, invalid) = PTE_V clear -> PR_TNV -> op_probe treats TNV as SUCCESS (architectural,
     not a bug: PROBE checks protection, not residency) */
const PROBE_PAGES = [
    {vpn: 0, prot: 4,  valid: true,  label: "open"},
    {vpn: 1, prot: 2,  valid: true,  label: "kernelOnly"},
    {vpn: 2, prot: 15, valid: true,  label: "readOnlyAll"},
    {vpn: 3, prot: 0,  valid: false, label: "invalid"},
    {vpn: 4, prot: 4,  valid: true,  label: "goodNeighbor"},
    {vpn: 5, prot: 2,  valid: true,  label: "badNeighbor"}
];

function buildProbeLayout(nCases)
{
    let ptes = [];
    for (let p of PROBE_PAGES) {
        let pfn = PROBE_TARGET_PFN_BASE + p.vpn;
        let val = ((p.valid ? MMUVAX.PTE_V : 0) | (p.prot << MMUVAX.PTE_V_ACC) | (pfn & 0x1FFFFF)) | 0;
        ptes.push({pa: (PROBE_SBR + 4 * p.vpn) | 0, val});
    }
    let codePfnBase = PROBE_CODE_PHYS_BASE / PAGE;
    for (let i = 0; i < nCases; i++) {
        let vpn = PROBE_CODE_S0_VPN0 + i;
        let val = (MMUVAX.PTE_V | (4 << MMUVAX.PTE_V_ACC) | ((codePfnBase + i) & 0x1FFFFF)) | 0;
        ptes.push({pa: (PROBE_SBR + 4 * vpn) | 0, val});
    }
    return {
        sbr: PROBE_SBR,
        slr: PROBE_CODE_S0_VPN0 + nCases,
        ptes,
        codeS0Base: (PROBE_TARGET_S0_BASE + PROBE_CODE_S0_VPN0 * PAGE) | 0,
        codePhysOf: (idx) => (PROBE_CODE_PHYS_BASE + idx * PAGE) | 0
    };
}

const PSL_V_PRV_LOCAL = 22;

function genProbeCase(index, rnd, opts)
{
    let {pageVpn, mode, prv, rw, crossInto} = opts;
    let mn = rw ? "PROBEW" : "PROBER";
    let c = new Case(mn, OPC_OF[mn], index, PROBE_CASE_STRIDE);
    c.base = (PROBE_TARGET_S0_BASE + (PROBE_CODE_S0_VPN0 + index) * PAGE) | 0;
    c.pc = c.base;
    c.byte(c.opc & 0xFF);

    let pageVA = (PROBE_TARGET_S0_BASE + pageVpn * PAGE) | 0;
    let length, base;
    if (crossInto) {
        length = PAGE - Math.floor(rnd() * 8);
        base = (pageVA + (PAGE - length)) | 0;
    } else {
        length = 1 + Math.floor(rnd() * (PAGE - 8));
        base = (pageVA + Math.floor(rnd() * Math.max(1, PAGE - length))) | 0;
    }
    emitRead(c, rnd, 1, mode);
    emitRead(c, rnd, 2, length);
    emitAddr(c, base);

    c.regs[15] = c.pc;
    c.regs[14] = c.base;
    c.psl = (prv << PSL_V_PRV_LOCAL) | 0;         // CUR=0 (kernel) -- code/stack pages are fully open regardless
    c.expectPCAfter = (c.pc + c.instr.length) | 0;
    c.trivial = false;
    return c;
}

function genProbeCases(rnd)
{
    let cases = [], index = 0;
    for (let p of PROBE_PAGES) {
        for (let rw = 0; rw <= 1; rw++) {
            for (let k = 0; k < 6; k++) {
                let mode = Math.floor(rnd() * 4), prv = Math.floor(rnd() * 4);
                cases.push(genProbeCase(index++, rnd, {pageVpn: p.vpn, mode, prv, rw, crossInto: false}));
            }
        }
    }
    for (let rw = 0; rw <= 1; rw++) {
        for (let prv of [0, 1, 2, 3]) {
            cases.push(genProbeCase(index++, rnd, {pageVpn: 4, mode: 3, prv, rw, crossInto: true}));
        }
    }
    return cases;
}

function buildProbeScript(cases, layout, memSizeM)
{
    let L = [`set cpu ${memSizeM}m`, "set cpu simhalt", "reset all"];
    for (let e of layout.ptes) L.push(`deposit ${hex(e.pa)} ${hex(e.val)}`);
    for (let c of cases) {
        let pa = layout.codePhysOf(c.index);
        for (let i = 0; i < c.instr.length; i++) L.push(`deposit -b ${hex((pa + i) | 0)} ${c.instr[i].toString(16)}`);
    }
    L.push(`show cpu mmuop=4:${hex(layout.sbr)}:1:0:${hex(IPR.SBR)}`);
    L.push(`show cpu mmuop=4:${hex(layout.slr)}:1:0:${hex(IPR.SLR)}`);
    L.push(`show cpu mmuop=4:1:1:0:${hex(IPR.MAPEN)}`);
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        L.push("deposit TRPIRQ 0");
        L.push("deposit SISR 0");
        for (let r = 0; r < 15; r++) L.push(`deposit R${r} ${hex(c.regs[r])}`);
        L.push(`deposit PSL ${hex(c.psl)}`);
        L.push(`deposit PC ${hex(c.pc)}`);
        L.push("step 1");
        let regList = [];
        for (let r = 0; r < 15; r++) regList.push("R" + r);
        regList.push("PC", "PSL");
        L.push(`examine -h ${regList.join(",")}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

function phaseProbe(simh, scratch, opts)
{
    let rnd = mulberry32(opts.seed ^ 0xB00B5);
    let cases = genProbeCases(rnd);
    let layout = buildProbeLayout(cases.length);

    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, PROBE_MEMSIZE_M * 1024 * 1024, MemoryVAX.TYPE.RAM);
    let cpu = new VAXCpu(bus);
    for (let e of layout.ptes) bus.setLong(e.pa, e.val);
    for (let c of cases) {
        let pa = layout.codePhysOf(c.index);
        for (let i = 0; i < c.instr.length; i++) bus.setByte((pa + i) | 0, c.instr[i]);
    }
    cpu.mmu.setSBR(layout.sbr);
    cpu.mmu.setSLR(layout.slr);
    cpu.mmu.setMAPEN(1);

    let script = buildProbeScript(cases, layout, PROBE_MEMSIZE_M);
    let out = runSimh(simh, script, path.join(scratch, "strqdiff-probe.ini"));
    let simhResults = parseBatchOutput(out);

    let failures = [], notReached = [], compared = 0;
    for (let c of cases) {
        cpu.regs.fill(0);
        for (let i = 0; i < 16; i++) cpu.regs[i] = c.regs[i];
        cpu.psl = c.psl;
        cpu.trpirq = 0;
        let js;
        try {
            let opc = cpu.decoder.decode(false);
            if (!executeStrq(opc, cpu.decoder, cpu)) throw new Error(`strq.js does not implement opcode ${hex(opc, 2)}`);
            js = {regs: Int32Array.from(cpu.regs), psl: cpu.psl, pcOK: cpu.regs[15] === c.expectPCAfter, error: null};
        } catch (e) {
            js = {regs: null, psl: 0, pcOK: false, error: e.message || String(e)};
        }
        let sr = simhResults.get(c.index);
        if (!sr || !sr.reached) { notReached.push(`${c.mnemonic} case ${c.index} (probe)`); continue; }
        compared++;
        let bad = compare(c.mnemonic, c, js, sr);
        if (bad) failures.push("PROBE: " + bad);
    }
    return {compared, failures, notReached, total: cases.length};
}

/* ------------------------------------------------------------------------------------------- *
 * EHKAA real-workload phase -- see the file header's "WHY ONLY FOUR OPCODES" note.  The trace       *
 * parser is intdiff.js's, unmodified in shape (same patch 0002 format).                              *
 * ------------------------------------------------------------------------------------------- */

const EHKAA_MN = new Set(["NOP", "INDEX", "MOVC3", "MOVC5"]);

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
            rec = {index: nRecords, pc: parseInt(m[1], 16) | 0, psl: parseInt(m[2], 16) | 0, mnemonic, want: wanted.has(mnemonic)};
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
        if (line.startsWith("OPND ")) return;
        if (line.startsWith("RECQ ")) return;
        if (line.startsWith("MEMR ")) {
            let f = fields(line);
            rec.memr = [];
            for (let i = 1; i + 3 < f.length; i += 4) {
                rec.memr.push({va: parseInt(f[i], 16) | 0, lnt: +f[i + 1], val: parseInt(f[i + 2], 16) | 0, w: +f[i + 3]});
            }
            return;
        }
        if (line.startsWith("BRDP ")) { nRecords++; onRecord(rec); rec = null; return; }
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

/** A tiny VAXDecodeMachine that replays MEMR-logged reads, exactly like intdiff.js's/decodediff.js's
    ReplayMachine -- used ONLY for the decode (specifier-resolution) half; execution runs through a
    REAL cpu/bus, exactly matching intdiff.js's own EHKAA-phase splice (see that file's comment on
    "harmless, since only the register file and PSL are compared" -- the same reasoning is why this
    phase is restricted to the four content-independent opcodes). */
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

function phaseEHKAA(opts)
{
    let ehkaaPath = opts.ehkaaExe;
    if (!fs.existsSync(ehkaaPath)) return {skipped: true, reason: `EHKAA diagnostic not found at ${ehkaaPath}`};

    let simh = opts.simh;
    let tracePath = path.join(opts.scratch, "strqdiff-ehkaa.trace");
    let script = [`set cpu 32m`, `set -d cpu history=100000:${tracePath}`, `load ${ehkaaPath}`, `go -q 200`].join("\n") + "\n";
    runSimh(simh, script, path.join(opts.scratch, "strqdiff-ehkaa.ini"));

    let allSeq = [];
    let total = parseTraceForOpcodes(tracePath, new Set(), (rec) => {
        allSeq.push({pc: rec.pc, ilen: rec.ilen, psl: rec.psl, mnemonic: rec.mnemonic, memr: rec.memr, ibyt: rec.ibyt, preg: rec.preg});
    });

    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, 0x100000, MemoryVAX.TYPE.RAM);
    let cpu = new VAXCpu(bus);
    let replay = new ReplayMachine();

    let failures = [];
    let compared = 0, skippedTrap = 0, skippedFPDResume = 0, skippedNotWanted = 0;
    let perOpcode = new Map();

    for (let i = 0; i < allSeq.length - 1; i++) {
        let rec = allSeq[i];
        if (!EHKAA_MN.has(rec.mnemonic)) { if (rec.mnemonic) skippedNotWanted++; continue; }
        let next = allSeq[i + 1];
        if (next.pc !== (rec.pc + rec.ilen | 0)) { skippedTrap++; continue; }
        /*
         * A genuine mid-string-op resume: PSL<FPD> was already set entering this instruction.
         *
         * THIS GUARD'S ORIGINAL REASON IS GONE.  It was written because nothing in the codebase
         * owned `fault_PC`, so strq.js's resume path could not compute `fault_PC + STR_GETDPC(R0)`
         * and any resume it replayed would have been wrong.  pcjsvax-c05 added the CPU loop, both
         * halves of the delta-PC are real, and tests/cpudiff.js drives the resume path against SIMH
         * directly -- an interrupted MOVC5 with a mid-copy page fault, a handler that fixes the PTE
         * and REIs, and the resumed instruction's register file compared.  Its `fpd-resume` case
         * kind and its `fpd-resume-no-delta` --selfcheck mutation are that coverage.
         *
         * WHAT REMAINS IS A PROPERTY OF THIS HARNESS, NOT OF strq.js: this phase replays one
         * instruction at a time through `decode(false)`, so it re-resolves specifiers that a real
         * resume never re-fetches.  That is unfixable here and uninteresting now that the loop is
         * graded elsewhere, so the guard stays -- but it is DEAD on the current capture
         * (skippedFPDResume is 0: EHKAA takes no interrupt inside the MOVC3/MOVC5 instances this
         * phase replays), and the count is printed so that stops being an assumption.
         */
        if (rec.psl & PSL_FPD) { skippedFPDResume++; continue; }

        replay.load({preg: rec.preg, pc: rec.pc, ibyt: rec.ibyt, memr: rec.memr || []});
        cpu.decoder.cpu = replay;
        let d = cpu.decoder;
        let opc;
        try {
            opc = d.decode(false);
        } catch (e) {
            failures.push(`EHKAA ${rec.mnemonic} @${hex(rec.pc)}: decode replay failed: ${e.message}`);
            continue;
        }

        cpu.regs.set(replay.regs);
        cpu.psl = rec.psl;
        cpu.trpirq = 0;
        try {
            if (!executeStrq(opc, d, cpu)) throw new Error(`opcode ${hex(opc, 2)} not implemented`);
        } catch (e) {
            failures.push(`EHKAA ${rec.mnemonic} @${hex(rec.pc)}: execution threw: ${e.message}`);
            continue;
        }
        compared++;
        perOpcode.set(rec.mnemonic, (perOpcode.get(rec.mnemonic) || 0) + 1);

        for (let r = 0; r < 16; r++) {
            if (r === 15) continue;             // PC already verified equal via the ilen check
            if (cpu.regs[r] !== next.preg[r]) {
                failures.push(`EHKAA ${rec.mnemonic} @${hex(rec.pc)}: R${r} mismatch: strq.js=${hex(cpu.regs[r])} simh=${hex(next.preg[r])}`);
                break;
            }
        }
        if ((cpu.psl & 0xF) !== (next.psl & 0xF)) {
            failures.push(`EHKAA ${rec.mnemonic} @${hex(rec.pc)}: CC mismatch: strq.js=${(cpu.psl & 0xF).toString(2)} simh=${(next.psl & 0xF).toString(2)}`);
        }
    }

    return {skipped: false, total, compared, skippedTrap, skippedFPDResume, skippedNotWanted, failures, perOpcode};
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck: mutate strq.js's own HANDLERS table in-process and confirm the differential           *
 * catches every mutation.  This is the shipped dispatch table, not a copy.                           *
 * ------------------------------------------------------------------------------------------- */

function selfcheck(simh, scratch)
{
    let rnd = mulberry32(0xC0FFEE);
    let H0 = {NOP: HANDLERS.NOP};
    const mutations = [
        {mn: "MOVC3", break: (H) => { H.MOVC3 = H0.NOP; }},                             // becomes a no-op
        {mn: "CMPC3", break: (H) => { let orig = H.CMPC3; H.CMPC3 = (cpu, opnd) => { orig(cpu, opnd); cpu.psl = cpu.psl & ~0xF; }; }},  // drops CC to 0 unconditionally
        {mn: "LOCC", break: (H) => { H.LOCC = H.SKPC; }},                                // wrong XOR sense entirely
        {mn: "SCANC", break: (H) => { H.SCANC = H.SPANC; }},
        {mn: "INDEX", break: (H) => { let orig = H.INDEX; H.INDEX = (cpu, opnd) => { let saved = opnd[3]; opnd[3] = (opnd[3] + 1) | 0; orig(cpu, opnd); opnd[3] = saved; }; }},  // size off-by-one
        {mn: "INSQUE", break: (H) => { H.INSQUE = H.REMQUE; }},
        {mn: "REMQHI", break: (H) => { H.REMQHI = H.REMQTI; }},
        {mn: "PROBER", break: (H) => { H.PROBER = () => {}; }},                          // leaves CC untouched
    ];

    let results = [];
    for (let mut of mutations) {
        let saved = HANDLERS[mut.mn];
        let savedDispatch = STRQ_OPCODES[OPC_OF[mut.mn]];
        mut.break(HANDLERS);
        STRQ_OPCODES[OPC_OF[mut.mn]] = HANDLERS[mut.mn];

        let caught = false;
        if (mut.mn === "PROBER") {
            /* PROBER needs the MMU/page-table environment -- run it through phaseProbe's own
               generator/comparator with a tiny case set instead of the plain per-opcode path. */
            let probeRnd = mulberry32(0xDEAD00 + 1);
            let cases = genProbeCases(probeRnd).filter((c) => c.mnemonic === "PROBER").slice(0, 20);
            let layout = buildProbeLayout(Math.max(...cases.map((c) => c.index)) + 1);
            let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
            bus.addMemory(0, PROBE_MEMSIZE_M * 1024 * 1024, MemoryVAX.TYPE.RAM);
            let cpu = new VAXCpu(bus);
            for (let e of layout.ptes) bus.setLong(e.pa, e.val);
            for (let c of cases) { let pa = layout.codePhysOf(c.index); for (let i = 0; i < c.instr.length; i++) bus.setByte((pa + i) | 0, c.instr[i]); }
            cpu.mmu.setSBR(layout.sbr); cpu.mmu.setSLR(layout.slr); cpu.mmu.setMAPEN(1);
            let script = buildProbeScript(cases, layout, PROBE_MEMSIZE_M);
            let out = runSimh(simh, script, path.join(scratch, "strqdiff-selfcheck-probe.ini"));
            let simhResults = parseBatchOutput(out);
            for (let c of cases) {
                cpu.regs.fill(0);
                for (let i = 0; i < 16; i++) cpu.regs[i] = c.regs[i];
                cpu.psl = c.psl;
                let js;
                try {
                    let opc = cpu.decoder.decode(false);
                    executeStrq(opc, cpu.decoder, cpu);
                    js = {regs: Int32Array.from(cpu.regs), psl: cpu.psl, pcOK: cpu.regs[15] === c.expectPCAfter, error: null};
                } catch (e) { js = {regs: null, psl: 0, pcOK: false, error: e.message}; }
                let sr = simhResults.get(c.index);
                if (compare(c.mnemonic, c, js, sr)) { caught = true; break; }
            }
        } else {
            let m = makeMachine();
            /* Small offset, NOT intdiff.js's 900000 -- this file's CASE_STRIDE is 8x intdiff.js's,
               so that offset would overflow int32 in Case's `(CASE_BASE + index*CASE_STRIDE)|0`
               and/or land outside the 64MB `runBatch(..., 64)` below configures on the SIMH side. */
            for (let k = 0; k < 60; k++) {
                let c = genCase(mut.mn, rnd, 3000 + k);
                let js = runCaseJS(m, c);
                let simhResults = runBatch(simh, [c], scratch, 64);
                let sr = simhResults.get(c.index);
                let bad = compare(mut.mn, c, js, sr);
                if (bad) { caught = true; break; }
            }
        }

        HANDLERS[mut.mn] = saved;
        STRQ_OPCODES[OPC_OF[mut.mn]] = savedDispatch;
        results.push({mn: mut.mn, caught});
        console.log(`  selfcheck ${mut.mn}: ${caught ? "CAUGHT" : "*** NOT CAUGHT ***"}`);
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                              *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function main()
{
    let simh = findSimh(getArg("--simh", null));
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "strqdiff-"));
    let seed = +getArg("--seed", String((Math.random() * 0xFFFFFFFF) | 0));
    console.log(`strqdiff.js: simh=${simh} scratch=${scratch} seed=${seed}`);

    if (process.argv.indexOf("--selfcheck") >= 0) {
        let results = selfcheck(simh, scratch);
        let bad = results.filter((r) => !r.caught);
        if (bad.length) {
            console.error(`SELFCHECK FAILED: ${bad.length} mutation(s) not caught: ${bad.map((b) => b.mn).join(", ")}`);
            process.exit(1);
        }
        console.log(`selfcheck: all ${results.length} mutations caught.`);
        process.exit(0);
    }

    let casesPerOpcode = +getArg("--cases-per-opcode", "150");
    if (casesPerOpcode < MIN_CASES_PER_OPCODE) {
        console.error(`FATAL: --cases-per-opcode ${casesPerOpcode} is below the coverage floor (${MIN_CASES_PER_OPCODE}); an undersized run must fail, not quietly pass.`);
        process.exit(1);
    }

    let problems = [];

    console.log(`\n=== RANDOMIZED phase: ${casesPerOpcode} cases x ${RANDOMIZED_MN.length} opcodes ===`);
    let r = phaseRandomized(simh, scratch, {seed, casesPerOpcode});
    console.log(`  total=${r.total} nontrivial=${r.nontrivial} (${(100 * r.nontrivial / r.total).toFixed(1)}%)`);
    console.log(`  failures=${r.failures.length} notReached=${r.notReached.length}`);
    if (r.total < MIN_TOTAL_CASES) problems.push(`COVERAGE: total cases ${r.total} < floor ${MIN_TOTAL_CASES}`);
    if (r.nontrivial / r.total < MIN_NONTRIVIAL_FRACTION) problems.push(`COVERAGE: non-trivial fraction ${(r.nontrivial / r.total).toFixed(3)} < floor ${MIN_NONTRIVIAL_FRACTION}`);
    for (let mn of r.mnemonics) {
        let n = r.perOpcodeCounts.get(mn) || 0;
        if (n < MIN_CASES_PER_OPCODE) problems.push(`COVERAGE: opcode ${mn} got only ${n} cases (floor ${MIN_CASES_PER_OPCODE})`);
    }
    if (r.notReached.length) problems.push(`COVERAGE: ${r.notReached.length} case(s) never reached comparison: ${r.notReached.slice(0, 10).join("; ")}`);
    for (let f of r.failures.slice(0, 25)) problems.push("RANDOMIZED: " + f);
    if (r.failures.length > 25) problems.push(`RANDOMIZED: ...and ${r.failures.length - 25} more failures`);

    console.log(`\n=== MAXLEN phase (true 65535-byte length) ===`);
    let ml = phaseMaxLen(simh, scratch, {seed});
    console.log(`  compared=${ml.compared} expected=${ml.expected} failures=${ml.failures.length} notReached=${ml.notReached.length}`);
    if (ml.compared < ml.expected) problems.push(`COVERAGE: MAXLEN compared only ${ml.compared}/${ml.expected} cases`);
    for (let f of ml.failures) problems.push(f);
    if (ml.notReached.length) problems.push(`COVERAGE: MAXLEN ${ml.notReached.length} case(s) never reached comparison: ${ml.notReached.join("; ")}`);

    console.log(`\n=== PROBE phase (PROBER/PROBEW real MMU protection) ===`);
    let pr = phaseProbe(simh, scratch, {seed});
    console.log(`  compared=${pr.compared} total=${pr.total} failures=${pr.failures.length} notReached=${pr.notReached.length}`);
    if (pr.compared < pr.total) problems.push(`COVERAGE: PROBE compared only ${pr.compared}/${pr.total} cases`);
    for (let f of pr.failures.slice(0, 25)) problems.push(f);
    if (pr.failures.length > 25) problems.push(`PROBE: ...and ${pr.failures.length - 25} more failures`);
    if (pr.notReached.length) problems.push(`COVERAGE: PROBE ${pr.notReached.length} case(s) never reached comparison: ${pr.notReached.join("; ")}`);

    console.log(`\n=== EHKAA phase ===`);
    /* $PCJS_VAX_REPO overrides the sibling-directory guess -- from a worktree, "../pcjs-vax"
       relative to REPO_ROOT resolves to a nonexistent sibling worktree, not the real pcjs-vax
       work repo (see the README's ENVIRONMENT note). */
    let defaultVaxRepo = process.env['PCJS_VAX_REPO'] || path.resolve(REPO_ROOT, "../pcjs-vax");
    let e = phaseEHKAA({simh, scratch, ehkaaExe: getArg("--ehkaa", path.resolve(defaultVaxRepo, "open-simh/VAX/tests/ehkaa.exe"))});
    if (e.skipped) {
        console.log(`  SKIPPED: ${e.reason}`);
        problems.push(`EHKAA: skipped (${e.reason}) -- not a substitute for the randomized phase, but its absence means real-workload coverage was NOT exercised this run`);
    } else {
        console.log(`  trace records=${e.total} compared=${e.compared} skippedTrap=${e.skippedTrap} skippedFPDResume=${e.skippedFPDResume} skippedNotWanted=${e.skippedNotWanted} failures=${e.failures.length}`);
        console.log(`  per-opcode instances: ${[...e.perOpcode.entries()].map(([k, v]) => `${k}=${v}`).sort().join(" ")}`);
        for (let f of e.failures.slice(0, 25)) problems.push("EHKAA: " + f);
        if (e.failures.length > 25) problems.push(`EHKAA: ...and ${e.failures.length - 25} more failures`);
        if (e.compared < 50) problems.push(`COVERAGE: EHKAA compared only ${e.compared} instances (expected >=50 across NOP/INDEX/MOVC3/MOVC5 in a 335,444-instruction trace)`);
        for (let mn of EHKAA_MN) {
            if (!(e.perOpcode.get(mn) > 0)) problems.push(`COVERAGE: EHKAA saw zero instances of ${mn} -- report by name, not silently absorbed`);
        }
    }

    if (problems.length) {
        console.error(`\nFAILED (${problems.length} problem(s)), seed=${seed}:`);
        for (let p of problems) console.error("  - " + p);
        process.exit(1);
    }
    console.log(`\nPASSED. seed=${seed}`);
}

main();
