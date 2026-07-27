/**
 * @fileoverview Differential test: VAX decode/operand resolution vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * A differential test of VAXDecoder against a REAL, EXECUTED Open SIMH microvax3900.  There are
 * no golden fixtures: every run launches the simulator, captures an instruction-history trace in
 * the extended format described in tests/simh/README.md, and replays every record through our
 * decoder.  If the patched simulator is missing, the test FAILS -- it never degrades into
 * self-comparison.
 *
 *      node machines/dec/vax/tests/decodediff.js [options]
 *        --simh PATH     patched microvax3900 (else $SIMH_DECODE_BIN, else scratch build)
 *        --trace FILE    reuse an existing EHKAA capture instead of running SIMH for it
 *        --cases N       randomized exerciser cases (default 4000)
 *        --faults N      fault/unwind cases (default 600)
 *        --seed S        PRNG seed, printed on failure so a run is reproducible
 *        --skip-ehkaa    run only the randomized phases
 *        --selfcheck     prove the differential detects deliberate defects, then exit
 *
 * WHY THE ORACLE IS SHAPED LIKE THIS
 * ----------------------------------
 * Decode is a pure function of three inputs -- the register file before the instruction, the
 * instruction bytes, and whatever memory the addressing modes read -- so the simulator is patched
 * to record all three alongside the outputs it already recorded.  The critical piece is the
 * READ LOG: every data read performed while resolving specifiers is captured as (address, length,
 * write-access, value).  On replay our decoder's reads are matched against that log IN ORDER, and
 * a read whose address, length or access type does not match SIMH's is a hard failure.  That is
 * what makes an effective-address bug detectable EXACTLY, rather than probabilistically via the
 * value that came back.
 *
 * WHAT IS ASSERTED, PER INSTRUCTION
 * ---------------------------------
 *   (1) the opcode we decoded is the one SIMH disassembled;
 *   (2) the operand queue matches in length and in every value;
 *   (3) the whole register file after resolution matches -- this is what grades autoincrement,
 *       autodecrement, and the register-conflict cases;
 *   (4) the recovery queue matches entry for entry;
 *   (5) the number of instruction-stream bytes consumed matches;
 *   (6) every read we issued matched SIMH's next logged read, and we issued exactly as many;
 *   (7) for branch opcodes, the branch displacement matches.
 *
 * THREE PHASES
 * ------------
 *   EHKAA      Replays the entire DEC EHKAA hardware-core diagnostic -- every instruction it
 *              executes, in order.  Real code, real addressing-mode mix.
 *   EXERCISER  Randomized instruction streams built to hit combinations real code does not:
 *              every addressing mode against every access type the CPU can reach, and
 *              specifically REGISTER-CONFLICT cases -- autoincrement or autodecrement on the same
 *              register being used as the index, or as a later specifier's base.  A naive
 *              generator never emits those, and they are where the bugs are.
 *   FAULTS     Instructions whose LAST specifier is a reserved addressing mode, placed after
 *              specifiers that already modified registers.  SIMH unwinds those modifications
 *              before taking the exception; so must we.  The exception handler is a HALT whose
 *              trace record shows the unwound register file, which is what we compare against.
 *
 * COVERAGE IS ASSERTED, NOT REPORTED
 * ----------------------------------
 * The lesson from pcjsvax-cd6 was that a uniform random pool made ~97% of comparisons trivial
 * (0 vs 0) and a deliberately broken boundary survived 150,000 operations undetected.  So the
 * memory this exerciser reads is pre-seeded with high-entropy non-zero values, every value read
 * during a deferred chain points back into seeded memory rather than into zeroed cold pages, and
 * the run FAILS if the mode/access matrix is not covered or if the fraction of non-trivial
 * comparisons falls below its floor.  An undersized run fails; it does not quietly pass.
 *
 * --selfcheck rebuilds the decoder from its own source with a deliberate defect textually
 * injected, runs a short pass against each, and fails if any defect is NOT caught.
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import { DROM, DROM_STRIDE, OPCODES, DR, IG, SPEC, MODE, RQ } from "../modules/v2/drom.js";
import { OP_MEM } from "../modules/v2/decode.js";

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

const MEMSIZE   = 0x01000000;       // 16MB, SIMH's microvax3900 default (vaxmod_defs.h INITMEMSIZE)

/*
 * Memory map for the randomized phases.  Nothing here is arbitrary:
 *
 *   TESTPC       where every generated instruction is deposited and started.  All of memory is
 *                zero at reset and the VAX opcode for HALT is 0x00, so the bytes after the
 *                instruction are already a halt field -- no fill needed, and a case that runs
 *                off the end of its own instruction stops instead of running away.
 *   SCB          SIMH's SCBB is 0 after reset, so the system control block is at physical 0.
 *                Every vector is pointed at a distinct HALT so an execution-time fault stops the
 *                simulator instead of wandering, and so the FAULTS phase can tell from the PC
 *                which vector was taken.
 *   HOT regions  the only memory the addressing modes are aimed at.  Seeded with values that are
 *                themselves addresses INSIDE a hot region, so that a deferred mode's pointer read
 *                lands on more seeded data instead of on a zeroed cold page.  That is the whole
 *                fix for the "97% of comparisons were 0 vs 0" failure mode.
 *   STACK        kept out of the hot regions so that the exception push in the FAULTS phase
 *                cannot perturb data an addressing mode is about to read.
 */
const TESTPC      = 0x00100000;
const HANDLER_BASE = 0x00100800;    // SCB vector k -> HANDLER_BASE + k*4, all HALT
const NUM_VECTORS  = 64;

/*
 * SEEDED MEMORY IS SPLIT IN TWO, AND THE SPLIT IS LOad-BEARING.
 *
 * The generated instructions do not merely decode -- SIMH executes them, and one whose
 * destination is a memory operand WRITES there.  If a written longword is one that a later
 * case's deferred addressing mode follows as a POINTER, that case reads a spliced address,
 * takes a machine check during resolution, and is dropped before its history record is written.
 * At 4,000 cases that compounds until a fifth of the run has silently evaporated.
 *
 * So the two roles are given disjoint memory:
 *
 *   PTR regions   are only ever READ, and only ever as pointers.  Every value in them is an
 *                 address inside a DATA region, deliberately unaligned so that the operand read
 *                 at the far end of the pointer is an unaligned access.  No effective address the
 *                 generator can produce lands here, so nothing can overwrite them.
 *   DATA regions  hold high-entropy longwords and are where every effective address ends up --
 *                 including the writes.  Corruption here is harmless: SIMH's own read log is the
 *                 oracle for values, so a changed byte changes both sides identically.
 *   LOW           where index-mode addresses land (a scaled small index plus a small base).
 *   STACK         where -(SP) and (SP)+ land, kept clear of everything else so the exception
 *                 push in the FAULTS phase cannot perturb data an addressing mode will read.
 */
const PTR_BASE    = 0x00200000;
const PTR_COUNT   = 4;
const PTR_STRIDE  = 0x00010000;
const PTR_SIZE    = 0x1000;
const DATA_BASE   = 0x00280000;
const DATA_COUNT  = 4;
const DATA_STRIDE = 0x00010000;
const DATA_SIZE   = 0x2000;         // 8KB: wide enough that a scaled index stays inside
const STACK       = 0x00300000;
const LOW_BASE    = 0x00000100;     // just above the SCB
const LOW_SIZE    = 0x1000;

/*
 * REGISTER ROLES, AND WHY THE EXERCISER NEEDS THEM.
 *
 * An index register is SCALED by the operand length before it is added to the base, so a register
 * holding a real address cannot also be an index: `(R5)[R5]` with R5 = 0x00280800 computes
 * 0x00280800*8 + 0x00280800, which is 20MB into a 16MB machine and takes a machine check during
 * resolution -- and a machine check aborts before the instruction-history record is written, so
 * the case would silently VANISH from the comparison instead of being tested.  That is the class
 * of silent hole this test exists to close, so the register file is split by role:
 *
 *   R0-R3    small index values, longword aligned.  Also usable as an index-mode BASE, which is
 *            how the register-conflict cases (same register indexed and auto-modified) stay in
 *            memory -- they land in the seeded low region.
 *   R4-R6    pointers into the PTR regions; the only registers a DEFERRED mode may use.
 *   R7-R14   addresses in the DATA regions; R14 (SP) points into the seeded stack region.
 *   R15      the PC, used for immediate, absolute and PC-relative forms.
 */
const IDX_REGS    = [0, 1, 2, 3];
const PTR_REGS    = [4, 5, 6];
const DATA_REGS   = [7, 8, 9, 10, 11, 12, 13, 14];

const VEC_RESAD   = 0x1C;           // SCB_RESAD, vax_defs.h:378

/* Coverage floors.  Below any of these the run FAILS; see the file header. */
const MIN_EHKAA_RECORDS   = 300000;
const MIN_EHKAA_NONTRIVIAL = 0.35;
const MIN_CASES           = 2000;
const MIN_FAULT_CASES     = 300;
const MIN_CONFLICT_CASES  = 200;
const MIN_MODE_COMBOS     = 100;
const MIN_EXERCISER_NONTRIVIAL = 0.80;

/* ------------------------------------------------------------------------------------------- *
 * Utilities                                                                                     *
 * ------------------------------------------------------------------------------------------- */

/**
 * mulberry32(a)
 *
 * Small deterministic PRNG so a failing run can be reproduced from its printed seed.
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
 * hex(v, n)
 *
 * @param {number} v
 * @param {number} [n]
 * @returns {string}
 */
function hex(v, n = 8)
{
    return (v >>> 0).toString(16).toUpperCase().padStart(n, "0");
}

/**
 * findSimh(pathArg)
 *
 * @param {string|null} pathArg
 * @returns {string}
 */
function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_DECODE_BIN']) candidates.push(process.env['SIMH_DECODE_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH built with the decode-replay patch; it has no\n" +
        "fixture fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh\n" +
        "PATH or set $SIMH_DECODE_BIN.  Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

/**
 * runSimh(bin, script, outPath)
 *
 * @param {string} bin
 * @param {string} script       full text of the .ini
 * @param {string} outPath      where to write the .ini (kept for post-mortem)
 * @returns {string} SIMH stdout
 */
function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 28, timeout: 30 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * Trace parsing                                                                                 *
 *                                                                                               *
 * The grammar is documented in machines/dec/vax/tests/simh/README.md.  A record is the stock     *
 * SIMH "PC PSL| disassembly" line plus its continuation lines, then REGS, then the six lines     *
 * the decode-replay patch adds.  BRDP is the anchor that terminates a record, because it is the  *
 * last line the patch emits.                                                                    *
 * ------------------------------------------------------------------------------------------- */

const INSTR_LINE_RE = /^([0-9A-Fa-f]{8}) ([0-9A-Fa-f]{8})\| (.*)$/;

/**
 * parseTrace(pathTrace, onRecord)
 *
 * Streams the trace, calling onRecord(rec) for each complete record.  Streaming rather than
 * slurping matters: a full EHKAA capture is a few hundred megabytes.
 *
 * @param {string} pathTrace
 * @param {function(Object): void} onRecord
 * @returns {number} number of records parsed
 */
function parseTrace(pathTrace, onRecord)
{
    const fd = fs.openSync(pathTrace, "r");
    const CHUNK = 1 << 22;
    let buf = Buffer.allocUnsafe(CHUNK);
    let carry = "";
    let rec = null, nRecords = 0, lineno = 0;

    const fields = (line) => {
        let sp = line.indexOf(" ");
        return sp < 0 ? [] : line.slice(sp + 1).split(" ");
    };

    const handleLine = (line) => {
        lineno++;
        let m = INSTR_LINE_RE.exec(line);
        if (m) {
            if (rec) throw new Error(`${pathTrace}:${lineno}: new record before previous one completed (PC=${hex(rec.pc)}) -- unpatched SIMH or truncated capture?`);
            rec = {
                index: nRecords,
                pc: parseInt(m[1], 16) | 0,
                psl: parseInt(m[2], 16) | 0,
                detail: m[3],
                mnemonic: m[3].split(" ", 1)[0],
                undef: / \(undefined\)$/.test(m[3]),
                fpd: / FPD set$/.test(m[3])
            };
            return;
        }
        if (!rec) return;                       // banner / prompt lines before the first record
        if (line.startsWith("REGS ")) { rec.reg = fields(line).map((t) => parseInt(t, 16) | 0); return; }
        if (line.startsWith("PREG ")) { rec.preg = fields(line).map((t) => parseInt(t, 16) | 0); return; }
        if (line.startsWith("IBYT ")) {
            let f = fields(line);
            rec.ilen = +f[0];
            rec.ibyt = Uint8Array.from(f.slice(1).map((t) => parseInt(t, 16)));
            return;
        }
        if (line.startsWith("OPND ")) {
            let f = fields(line);
            rec.nopnd = +f[0];
            rec.opnd = f.slice(1).map((t) => parseInt(t, 16) | 0);
            return;
        }
        if (line.startsWith("RECQ ")) {
            let f = fields(line);
            rec.recqptr = +f[0];
            rec.recq = f.slice(1).map((t) => parseInt(t, 16) | 0);
            return;
        }
        if (line.startsWith("MEMR ")) {
            let f = fields(line);
            rec.nmemr = +f[0];
            rec.memr = [];
            for (let i = 1; i + 3 < f.length; i += 4) {      // f[0] is the count
                rec.memr.push({
                    va: parseInt(f[i], 16) | 0,
                    lnt: +f[i + 1],
                    val: parseInt(f[i + 2], 16) | 0,
                    w: +f[i + 3]
                });
            }
            return;
        }
        if (line.startsWith("BRDP ")) {
            rec.brdisp = parseInt(fields(line)[0], 16) | 0;
            if (!rec.reg || !rec.preg || !rec.ibyt || !rec.opnd || !rec.recq || !rec.memr) {
                throw new Error(`${pathTrace}:${lineno}: record at PC=${hex(rec.pc)} is missing decode-replay lines -- was the capture made with "set -d cpu history=..."?`);
            }
            nRecords++;
            onRecord(rec);
            rec = null;
            return;
        }
        if (line.startsWith(" ")) { rec.detail += "\n" + line.trim(); return; }
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
    if (rec) throw new Error(`${pathTrace}: file ends mid-record (PC=${hex(rec.pc)}) -- truncated capture`);
    return nRecords;
}

/* ------------------------------------------------------------------------------------------- *
 * The replay machine                                                                            *
 * ------------------------------------------------------------------------------------------- */

/**
 * A comparison failure.  Distinct from a JS TypeError so a harness bug is never mistaken for a
 * decoder bug.
 *
 * @class Mismatch
 */
class Mismatch extends Error {}

/**
 * @class ReplayMachine
 *
 * Implements the decoder's VAXDecodeMachine interface out of one trace record.  Instruction-stream
 * fetches come from the recorded instruction bytes; data reads are matched against the recorded
 * read log, which is what turns an effective-address bug into an immediate, exact failure rather
 * than a wrong value further downstream.
 */
class ReplayMachine {
    constructor()
    {
        this.regs = new Int32Array(16);
        this.reads = 0;
    }

    /**
     * @param {Object} rec
     */
    load(rec)
    {
        this.regs.set(rec.preg);
        this.iPC = rec.pc;
        this.ibyt = rec.ibyt;
        this.memr = rec.memr;
        this.memrIdx = 0;
        this.reads = 0;
    }

    /**
     * @param {number} lnt
     * @returns {number}
     */
    getISTR(lnt)
    {
        let off = (this.regs[15] - this.iPC) | 0;
        if (off < 0 || off + lnt > this.ibyt.length) {
            throw new Mismatch(`instruction-stream fetch of ${lnt} byte(s) at offset ${off} is outside the ${this.ibyt.length} byte(s) SIMH recorded for this instruction -- our decoder consumed the wrong number of specifier bytes`);
        }
        let v = 0;
        for (let i = lnt - 1; i >= 0; i--) v = (v << 8) | this.ibyt[off + i];
        this.regs[15] = (this.regs[15] + lnt) | 0;
        return lnt == 4 ? (v | 0) : v;
    }

    /**
     * @param {number} pc
     */
    setPC(pc)
    {
        this.regs[15] = pc | 0;
    }

    /**
     * @param {number} va
     * @param {number} lnt
     * @param {boolean} fWrite
     * @returns {number}
     */
    readData(va, lnt, fWrite)
    {
        this.reads++;
        let e = this.memr[this.memrIdx];
        if (!e) {
            throw new Mismatch(`our decoder issued read #${this.memrIdx + 1} (va=${hex(va)} lnt=${lnt} ${fWrite ? "write-access" : "read-access"}) but SIMH performed only ${this.memr.length} read(s) resolving this instruction`);
        }
        if (e.va !== (va | 0) || e.lnt !== lnt || e.w !== (fWrite ? 1 : 0)) {
            throw new Mismatch(`read #${this.memrIdx + 1}: SIMH read va=${hex(e.va)} lnt=${e.lnt} ${e.w ? "write-access" : "read-access"}, we read va=${hex(va)} lnt=${lnt} ${fWrite ? "write-access" : "read-access"}`);
        }
        this.memrIdx++;
        return e.val;
    }

    /**
     * The decoder never writes during resolution; these exist only so that a mistake which caused
     * it to try is loud instead of a TypeError.
     *
     * @param {number} va
     */
    writeData(va)
    {
        throw new Mismatch(`operand resolution must not write memory (attempted at ${hex(va)})`);
    }

    /**
     * @param {number} va
     */
    testWrite(va)
    {
        throw new Mismatch(`operand resolution must not probe write access (attempted at ${hex(va)})`);
    }
}

/*
 * PSL<fpd>, vax_defs.h.  Transcribed rather than generated because it is a single bit and the
 * decoder's only dependency on the PSL.
 */
const PSL_FPD = 1 << 27;

/**
 * compareRecord(decoder, machine, rec, stats)
 *
 * Replays one record and returns null on success or a human-readable reason on failure.
 *
 * @param {Object} decoder
 * @param {ReplayMachine} machine
 * @param {Object} rec
 * @param {Object} stats
 * @returns {string|null}
 */
function compareRecord(decoder, machine, rec, stats)
{
    machine.load(rec);
    let opc;
    try {
        opc = decoder.decode(!!(rec.psl & PSL_FPD));
    } catch (e) {
        if (e instanceof Mismatch) return e.message;
        if (e && e.name == "VAXFault") {
            return `our decoder raised fault ${e.code} but SIMH resolved this instruction without faulting`;
        }
        throw e;
    }

    let mnemonic = OPCODES[opc];
    if (!rec.undef && mnemonic !== rec.mnemonic) {
        return `opcode: we decoded ${opc.toString(16)} (${mnemonic}) but SIMH disassembled "${rec.mnemonic}"`;
    }
    if (decoder.nOpnd !== rec.nopnd) {
        return `operand count: SIMH resolved ${rec.nopnd}, we resolved ${decoder.nOpnd}`;
    }
    for (let i = 0; i < rec.nopnd; i++) {
        if ((decoder.opnd[i] | 0) !== rec.opnd[i]) {
            return `opnd[${i}]: SIMH ${hex(rec.opnd[i])}, we ${hex(decoder.opnd[i])}`;
        }
    }
    for (let r = 0; r < 16; r++) {
        if (machine.regs[r] !== rec.reg[r]) {
            return `R${r} after resolution: SIMH ${hex(rec.reg[r])}, we ${hex(machine.regs[r])}`;
        }
    }
    if (decoder.recqptr !== rec.recqptr) {
        return `recovery queue length: SIMH ${rec.recqptr}, we ${decoder.recqptr}`;
    }
    for (let i = 0; i < rec.recqptr; i++) {
        if ((decoder.recq[i] | 0) !== rec.recq[i]) {
            return `recq[${i}]: SIMH ${hex(rec.recq[i])}, we ${hex(decoder.recq[i])}`;
        }
    }
    let ilen = (machine.regs[15] - rec.pc) | 0;
    if (ilen !== rec.ilen) {
        return `instruction length: SIMH consumed ${rec.ilen} byte(s), we consumed ${ilen}`;
    }
    if (machine.memrIdx !== rec.memr.length) {
        return `read count: SIMH performed ${rec.memr.length} read(s) resolving this instruction, we performed ${machine.memrIdx}`;
    }
    if (rec.memr.length !== rec.nmemr) {
        return `SIMH's read log overflowed (${rec.nmemr} reads, only ${rec.memr.length} recorded) -- raise DEC_RLOG_MAX and rebuild`;
    }
    let hdr = DROM[opc * DROM_STRIDE];
    let nsp = hdr & DR.NSPMASK;
    let lastAt = nsp ? DROM[opc * DROM_STRIDE + nsp] : 0;
    /*
     * The destination predicate.  An instruction body decides where to store its result from
     * `spec`, and that decision plus the address it uses are exactly what SIMH records in the
     * two operand-queue slots of a write specifier: OP_MEM and the address, or the register
     * number and its contents.  Grading it here means the three instruction groups that consume
     * this decoder inherit a checked predicate instead of three private copies of the rule.
     */
    if (nsp && lastAt < SPEC.BB && (lastAt & DR.ACMASK) == DR.W && rec.nopnd >= 2) {
        let flag = rec.opnd[rec.nopnd - 2], where = rec.opnd[rec.nopnd - 1];
        let fMem = decoder.isMemoryDestination();
        if (fMem !== (flag === OP_MEM)) {
            return `destination: SIMH says the ${flag === OP_MEM ? "memory" : "register"} form (opnd flag ${hex(flag)}), isMemoryDestination() says ${fMem ? "memory" : "register"} (spec=${hex(decoder.spec, 2)})`;
        }
        if (fMem) {
            if ((decoder.va | 0) !== where) return `destination address: SIMH ${hex(where)}, we ${hex(decoder.va)}`;
        } else {
            if (decoder.rn !== flag) return `destination register: SIMH R${flag}, we R${decoder.rn}`;
        }
        stats.destChecks++;
    }
    if (nsp && lastAt >= SPEC.BB) {
        if (decoder.brdisp !== rec.brdisp) {
            return `branch displacement: SIMH ${hex(rec.brdisp)}, we ${hex(decoder.brdisp)}`;
        }
        stats.branchDisp++;
    }

    /*
     * Non-triviality.  A record counts only if resolution actually did something that a broken
     * decoder could get wrong: read memory and got a non-zero value back, modified a register, or
     * produced a non-zero operand.  Counting records instead of counting instructions is the
     * whole point -- 335,444 records of "resolved nothing" would prove nothing.
     */
    let nontrivial = decoder.recqptr > 0;
    for (let e of rec.memr) if (e.val !== 0) { nontrivial = true; break; }
    if (!nontrivial) {
        for (let i = 0; i < rec.nopnd; i++) if (rec.opnd[i] !== 0) { nontrivial = true; break; }
    }
    if (nontrivial) stats.nontrivial++;
    stats.reads += rec.memr.length;
    stats.recq += decoder.recqptr;
    stats.opcodes.add(opc);
    return null;
}

/* ------------------------------------------------------------------------------------------- *
 * Decode-ROM driven knowledge of which (mode, access type) pairs are even reachable             *
 * ------------------------------------------------------------------------------------------- */

/** Every addressing-mode nibble, in architecture order.  SH0-SH3 are one mode. */
const MODE_NAMES = {
    [MODE.SH0]: "literal", [MODE.IDX]: "index", [MODE.GRN]: "register", [MODE.RGD]: "regdef",
    [MODE.ADC]: "autodec", [MODE.AIN]: "autoinc", [MODE.AID]: "autoincdef",
    [MODE.BDP]: "bytedisp", [MODE.BDD]: "bytedispdef", [MODE.WDP]: "worddisp",
    [MODE.WDD]: "worddispdef", [MODE.LDP]: "longdisp", [MODE.LDD]: "longdispdef"
};

/** Addressing modes whose resolution fetches a POINTER out of memory before the operand. */
const DEFERRED = new Set([MODE.AID, MODE.BDD, MODE.WDD, MODE.LDD]);

const SPEC_NAMES = Object.fromEntries(Object.entries(SPEC).map(([k, v]) => [v, k]));

/**
 * reachableOpcodes()
 *
 * The opcodes this CPU actually decodes specifiers for: base or floating subgroup, at least one
 * specifier, and not one of the ODC()-wrapped emulator-dispatched opcodes (which report zero
 * decodable specifiers).  Everything the exerciser generates comes from here, so the exerciser
 * cannot accidentally test an addressing mode against an access type the machine cannot produce.
 *
 * @returns {Array.<number>}
 */
function reachableOpcodes()
{
    let out = [];
    for (let opc = 0; opc < 512; opc++) {
        if (!OPCODES[opc]) continue;
        let hdr = DROM[opc * DROM_STRIDE];
        let grp = (hdr >> DR.V_IGMASK) & DR.M_IGMASK;
        if (grp != IG.BASE && grp != IG.BSGFL && grp != IG.BSDFL) continue;
        if (!(hdr & DR.NSPMASK)) continue;
        out.push(opc);
    }
    return out;
}

/*
 * Opcodes the exerciser must not generate, and why.  Everything else in the reachable set is
 * fair game, because the instruction-history record is written BEFORE the instruction body runs:
 * an execution-time trap or fault cannot corrupt the record we are grading, it only has to not
 * run away, which the HALT field and the HALT-pointed SCB guarantee.
 */
const EXCLUDE = new Set([
    /* control flow: would leave the halt field and execute whatever it landed on */
    "JMP", "JSB", "BSBB", "BSBW", "CALLG", "CALLS", "RET", "RSB",
    "CASEB", "CASEW", "CASEL",
    /* privileged or machine-state changing: would break the machine for later cases */
    "HALT", "MTPR", "MFPR", "LDPCTX", "SVPCTX", "REI", "CHMK", "CHME", "CHMS", "CHMU",
    "PROBER", "PROBEW", "XFC", "BPT", "BUGW", "BUGL",
    "INSQUE", "REMQUE", "INSQHI", "INSQTI", "REMQHI", "REMQTI",
    "BICPSW", "BISPSW", "MOVPSL", "PUSHR", "POPR"
]);

/**
 * specifierIsBranch(at)
 *
 * @param {number} at
 * @returns {boolean}
 */
function specifierIsBranch(at)
{
    return at >= SPEC.BB;
}

/* ------------------------------------------------------------------------------------------- *
 * The randomized exerciser                                                                      *
 * ------------------------------------------------------------------------------------------- */

/**
 * legalModes(at, fLast)
 *
 * Which addressing modes may be used for a specifier of decode ROM type `at` without provoking a
 * reserved addressing fault.  This encodes the architecture's rules, and it is the generator's
 * job to obey them -- a case that faults during decode produces NO history record, so it would
 * silently vanish from the comparison instead of being tested.  (The FAULTS phase deliberately
 * violates these rules, which is why it is a separate phase with its own oracle.)
 *
 * @param {number} at
 * @returns {Array.<number>}
 */
function legalModes(at)
{
    let acc = at & DR.ACMASK;
    let modes = [MODE.GRN, MODE.RGD, MODE.ADC, MODE.AIN, MODE.AID,
                 MODE.BDP, MODE.BDD, MODE.WDP, MODE.WDD, MODE.LDP, MODE.LDD, MODE.IDX];
    if (acc == DR.R || at == SPEC.RG) {
        /* short literal is read-only; RG is DR_M in the table but has a literal case */
        modes = [MODE.SH0].concat(modes);
    }
    if (acc == DR.A) {
        /* an address specifier has no register form */
        modes = modes.filter((m) => m != MODE.GRN);
    }
    return modes;
}

/**
 * registerConstraint(at, mode)
 *
 * The highest register number legal for this (access type, mode) pair, plus whether R15 is
 * allowed.  Register mode reads a register PAIR for quad and a QUAD for octa, so the base
 * register is bounded; the PC is illegal as a base for every register-based mode.
 *
 * @param {number} at
 * @param {number} mode
 * @returns {{max: number, allowPC: boolean}}
 */
function registerConstraint(at, mode)
{
    let lnt = at & DR.LNMASK;
    if (mode == MODE.GRN) {
        if (lnt == 4) return {max: 11, allowPC: false};         // octa: R[rn..rn+3], rn < nAP
        if (lnt == 3) return {max: 13, allowPC: false};         // quad: R[rn..rn+1], rn < nSP
        return {max: 14, allowPC: false};
    }
    if (mode == MODE.RGD || mode == MODE.ADC) return {max: 14, allowPC: false};
    if (mode == MODE.AIN || mode == MODE.AID) return {max: 15, allowPC: true};
    return {max: 15, allowPC: true};                            // displacement modes: PC-relative is legal
}

/**
 * @class Generator
 *
 * Builds instruction streams and the SIMH command script that runs them.  Also tallies coverage,
 * because the generator is the only thing that knows for certain which mode was used for which
 * specifier -- the decoder is deliberately not instrumented for the test's benefit.
 */
class Generator {
    /**
     * @param {function(): number} rnd
     */
    constructor(rnd)
    {
        this.rnd = rnd;
        this.combos = new Set();            // "mode/spec" pairs actually generated
        this.conflicts = 0;                 // cases containing a deliberate register conflict
        this.indexCases = 0;
        this.pcRelative = 0;
        this.immediates = 0;
    }

    /**
     * @param {number} n
     * @returns {number} 0..n-1
     */
    pick(n)
    {
        return Math.floor(this.rnd() * n) % n;
    }

    /**
     * @param {Array} a
     * @returns {*}
     */
    choose(a)
    {
        return a[this.pick(a.length)];
    }

    /**
     * dataAddr()
     *
     * An address in the middle of a seeded region, so that a displacement of up to +/-1KB and an
     * autoincrement of up to 16 bytes both stay on seeded data.
     *
     * @returns {number}
     */
    dataAddr()
    {
        let r = this.pick(DATA_COUNT);
        return (DATA_BASE + r * DATA_STRIDE + (DATA_SIZE >> 2) + this.pick(256) - 128) & ~3;
    }

    /**
     * ptrAddr()
     *
     * A longword-aligned address inside a PTR region.  Alignment is required, not cosmetic: a
     * pointer fetched from an unaligned offset is a splice of two different addresses, typically
     * far outside the machine.  See pointerRegs().
     *
     * @returns {number}
     */
    ptrAddr()
    {
        let r = this.pick(PTR_COUNT);
        return (PTR_BASE + r * PTR_STRIDE + (PTR_SIZE >> 1) + this.pick(256) - 128) & ~3;
    }

    /**
     * indexValue()
     *
     * A small index.  Bounded above so that `index << 3` (the largest scale this CPU can reach,
     * for quadword operands) plus a base of the same magnitude still lands inside the seeded low
     * region, and bounded below so it does not land in the SCB.
     *
     * @returns {number}
     */
    indexValue()
    {
        return (96 + this.pick(288)) & ~3;
    }

    /**
     * initialRegs()
     *
     * @returns {Int32Array}
     */
    initialRegs()
    {
        let regs = new Int32Array(16);
        for (let r of IDX_REGS) regs[r] = this.indexValue();
        for (let r of PTR_REGS) regs[r] = this.ptrAddr();
        for (let r of DATA_REGS) regs[r] = this.dataAddr();
        regs[14] = STACK + (LOW_SIZE >> 1);
        return regs;
    }

    /**
     * addrRegs(max, pool)
     *
     * Registers legal as the base of an address-forming mode: they must hold an address, and they
     * must satisfy the mode's own register-number ceiling.
     *
     * @param {number} max
     * @param {Array.<number>} [pool]
     * @returns {Array.<number>}
     */
    addrRegs(max, pool)
    {
        let a = (pool || DATA_REGS).filter((r) => r <= max);
        return a.length ? a : [7];
    }

    /**
     * pointerRegs(max, state, pool)
     *
     * Registers legal as the base of a DEFERRED mode, which reads a longword POINTER out of
     * memory rather than data.  The extra constraint is alignment, and it is not cosmetic: the
     * seeded regions hold unaligned addresses, so a pointer fetched from an unaligned location is
     * a splice of two different addresses -- typically far outside the 16MB machine, which takes
     * a machine check DURING resolution and therefore produces no instruction-history record at
     * all.  The case would vanish from the comparison instead of being tested, which is exactly
     * the silent hole this test is built to avoid.  So a deferred mode may only use a register
     * whose alignment no earlier specifier disturbed.
     *
     * The DATA read at the far end of the pointer is deliberately left unaligned, so unaligned
     * access is still exercised where it is harmless.
     *
     * @param {number} max
     * @param {Object} state
     * @param {Array.<number>} [pool]
     * @returns {Array.<number>}
     */
    pointerRegs(max, state, pool)
    {
        let a = (pool || PTR_REGS).filter((r) => r <= max && !state.unaligned.has(r));
        return a.length ? a : null;
    }

    /**
     * emitSpecifier(at, bytes, state)
     *
     * Appends one operand specifier -- and, for index mode, its base specifier -- to `bytes`.
     * `state.regsUsed` carries the registers earlier specifiers have already auto-modified, which
     * is what lets the generator DELIBERATELY create register conflicts instead of avoiding them.
     *
     * @param {number} at
     * @param {Array.<number>} bytes
     * @param {Object} state
     */
    emitSpecifier(at, bytes, state)
    {
        let modes = legalModes(at);
        let mode = this.choose(modes);
        if (DEFERRED.has(mode) && !this.pointerRegs(15, state)) {
            mode = MODE.BDP;                            // no aligned base left; stay non-deferred
        }
        if (mode == MODE.IDX) {
            /*
             * Index mode: an index register, then a complete base specifier.  This is where the
             * register-conflict cases are manufactured -- roughly half the time the base is an
             * autoincrement or autodecrement on the SAME register used as the index.  SIMH
             * captures the index contribution from the pre-modification value and then modifies
             * the register; a decoder that reads the register twice, or modifies it first, gets a
             * different effective address AND a different register file, and both are compared.
             */
            let rx = this.choose(IDX_REGS);             // index register, never the PC
            bytes.push(MODE.IDX | rx);
            let baseModes = [MODE.RGD, MODE.ADC, MODE.AIN, MODE.AID,
                             MODE.BDP, MODE.BDD, MODE.WDP, MODE.WDD, MODE.LDP, MODE.LDD];
            let bmode = this.choose(baseModes);
            if (DEFERRED.has(bmode) && state.unaligned.has(rx)) bmode = MODE.BDP;
            let conflict = this.rnd() < 0.5 && (bmode == MODE.ADC || bmode == MODE.AIN);
            let rb = conflict ? rx : (DEFERRED.has(bmode) ? this.choose(this.pointerRegs(14, state) || [4])
                                                          : this.choose(this.addrRegs(14)));
            if (conflict) this.conflicts++;
            this.emitBaseSpecifier(bmode, rb, bytes, state);
            this.combos.add("index+" + MODE_NAMES[bmode] + "/" + SPEC_NAMES[at]);
            this.indexCases++;
            this.noteSideEffect(bmode, rb, at, state);
            return;
        }
        if (mode == MODE.SH0) {
            bytes.push(this.pick(0x40));                // 0x00-0x3F is the short literal range
            this.combos.add("literal/" + SPEC_NAMES[at]);
            return;
        }
        let c = registerConstraint(at, mode);
        /*
         * Bias register choice towards registers an EARLIER specifier already auto-modified.
         * That is the second family of register conflicts: `MOVL (R3)+, (R3)+` must read the
         * second operand from the address the first specifier left behind, not from the original.
         */
        let rn;
        let pool = (mode == MODE.GRN) ? IDX_REGS.concat(PTR_REGS, DATA_REGS).filter((r) => r <= c.max)
                 : DEFERRED.has(mode) ? this.pointerRegs(c.max, state)
                 : this.addrRegs(c.max);
        if (!pool) { mode = MODE.BDP; pool = this.addrRegs(c.max); }
        let reuse = state.regsUsed.size && this.rnd() < 0.45;
        if (reuse) {
            let cand = [...state.regsUsed].filter((r) => pool.indexOf(r) >= 0);
            if (cand.length) { rn = this.choose(cand); this.conflicts++; }
            else rn = this.choose(pool);
        } else if (c.allowPC && this.rnd() < 0.12 && mode != MODE.BDD && mode != MODE.WDD) {
            /*
             * Immediate, absolute, or PC-relative.  The byte- and word-displacement DEFERRED
             * forms are excluded from the PC: their pointer would be fetched from the zeroed page
             * around the instruction, giving an effective address of 0 -- which is the system
             * control block, so a write specifier would overwrite the exception vectors and take
             * the machine with it.  `@L^d(PC)` covers the PC-relative deferred flow instead,
             * because a longword displacement can reach the seeded regions.
             */
            rn = 15;
        } else {
            rn = this.choose(pool);
        }
        bytes.push(mode | rn);
        this.emitTail(mode, rn, at, bytes, state);
        this.combos.add(MODE_NAMES[mode] + "/" + SPEC_NAMES[at]);
        this.noteSideEffect(mode, rn, at, state);
        if (rn == 15) {
            if (mode == MODE.AIN) this.immediates++;
            else if (mode == MODE.AID) this.immediates++;
            else this.pcRelative++;
        }
    }

    /**
     * emitBaseSpecifier(mode, rn, bytes, state)
     *
     * @param {number} mode
     * @param {number} rn
     * @param {Array.<number>} bytes
     * @param {Object} state
     */
    emitBaseSpecifier(mode, rn, bytes, state)
    {
        bytes.push(mode | rn);
        this.emitTail(mode, rn, SPEC.RL, bytes, state);
    }

    /**
     * noteSideEffect(mode, rn, at, state)
     *
     * Record that this specifier auto-modified a register: which register (so a later specifier
     * can deliberately collide with it), and whether the modification could have broken longword
     * alignment (so a later DEFERRED specifier does not fetch a spliced pointer -- see
     * pointerRegs()).  Autoincrement-deferred always steps by 4 and so is alignment-preserving.
     *
     * @param {number} mode
     * @param {number} rn
     * @param {number} at
     * @param {Object} state
     */
    noteSideEffect(mode, rn, at, state)
    {
        if (rn == 15) return;                           // PC side effects are not register side effects
        if (mode != MODE.ADC && mode != MODE.AIN && mode != MODE.AID) return;
        state.regsUsed.add(rn);
        let step = (mode == MODE.AID) ? 4 : (1 << (at & DR.LNMASK));
        if (step & 3) state.unaligned.add(rn);
    }

    /**
     * emitTail(mode, rn, at, bytes, state)
     *
     * The displacement or immediate bytes that follow a specifier byte.  Displacements are kept
     * small so the effective address stays inside a seeded region; PC-relative displacements are
     * aimed AT a seeded region from the instruction's own address instead.
     *
     * @param {number} mode
     * @param {number} rn
     * @param {number} at
     * @param {Array.<number>} bytes
     * @param {Object} state
     */
    emitTail(mode, rn, at, bytes, state)
    {
        let lnt = 1 << (at & DR.LNMASK);
        let pcRel = (rn == 15);
        let push = (v, n) => { for (let i = 0; i < n; i++) bytes.push((v >>> (i * 8)) & 0xFF); };

        if (mode == MODE.AIN && pcRel) {
            /* immediate: the operand's own bytes follow, and the operand length decides how many */
            for (let i = 0; i < lnt; i++) bytes.push(this.pick(256));
            return;
        }
        if (mode == MODE.AID && pcRel) {
            /* absolute: a full longword address follows */
            push(this.dataAddr(), 4);
            return;
        }
        if (mode == MODE.BDP || mode == MODE.BDD) {
            let d = this.pick(128) - 64;
            if (mode == MODE.BDD) {
                d &= ~3;                                // keep the pointer fetch aligned
                /*
                 * A PC-relative deferred mode fetches its POINTER from the instruction stream.
                 * Aimed forwards it would read the instruction's own bytes and treat them as an
                 * address -- usually one outside the 16MB machine, which machine-checks during
                 * resolution and loses the case.  Aim it backwards, into the zeroed page below
                 * the code, where the pointer is 0 and the effective address is still compared.
                 */
                if (pcRel) d = -64 + (this.pick(8) * 4);
            }
            bytes.push(d & 0xFF);
            return;
        }
        if (mode == MODE.WDP || mode == MODE.WDD) {
            if (pcRel) {
                /*
                 * PC-relative with a word displacement can only reach +/-32KB, which does not
                 * reach the seeded regions from TESTPC.  Aim it BACKWARDS into the zeroed page
                 * below the code instead: the value read is zero, but the ADDRESS is still
                 * compared exactly, which is the part that grades PC-relative arithmetic -- and
                 * aiming backwards keeps a deferred form from fetching its pointer out of the
                 * instruction's own bytes (see MODE.BDD above).
                 */
                push((-2048 + this.pick(1024) * 4) & 0xFFFF, 2);
            } else {
                let d = this.pick(2048) - 1024;
                if (mode == MODE.WDD) d &= ~3;          // keep the pointer fetch aligned
                push(d & 0xFFFF, 2);
            }
            return;
        }
        if (mode == MODE.LDP || mode == MODE.LDD) {
            if (pcRel) {
                /* a longword displacement CAN reach a seeded region from the instruction stream */
                push(((mode == MODE.LDD ? this.ptrAddr() : this.dataAddr()) - (TESTPC + bytes.length + 4)) | 0, 4);
            } else {
                let d = this.pick(2048) - 1024;
                if (mode == MODE.LDD) d &= ~3;          // keep the pointer fetch aligned
                push(d | 0, 4);
            }
            return;
        }
    }

    /**
     * buildCase(opcodes)
     *
     * @param {Array.<number>} opcodes
     * @returns {Object} {opc, bytes, regs}
     */
    buildCase(opcodes)
    {
        let opc = this.choose(opcodes);
        let hdr = DROM[opc * DROM_STRIDE];
        let nsp = hdr & DR.NSPMASK;
        let bytes = [];
        if (opc >= 0x100) bytes.push(0xFD);
        bytes.push(opc & 0xFF);
        let state = {regsUsed: new Set(), unaligned: new Set()};
        for (let i = 1; i <= nsp; i++) {
            let at = DROM[opc * DROM_STRIDE + i];
            if (specifierIsBranch(at)) {
                /*
                 * A branch displacement, not an addressing mode.  Keep it inside the halt field
                 * so that a taken branch stops instead of running away, and so that the case
                 * still grades the displacement fetch (zero-extended, byte or word).
                 */
                let n = (at & 1) ? 2 : 1;
                let d = this.pick(64) + 8;
                for (let k = 0; k < n; k++) bytes.push((d >>> (k * 8)) & 0xFF);
                this.combos.add("branchdisp/" + SPEC_NAMES[at]);
                break;
            }
            this.emitSpecifier(at, bytes, state);
        }
        return {opc, bytes, regs: this.initialRegs()};
    }
}

/**
 * ptrSeed(rnd)
 *
 * The value stored at a longword inside a PTR region: an address inside a DATA region, chosen
 * UNALIGNED so that the operand read at the far end of the pointer is an unaligned access.  The
 * offset is bounded so a quadword read at the far end cannot run past the region.
 *
 * @param {function(): number} rnd
 * @returns {number}
 */
function ptrSeed(rnd)
{
    let r = Math.floor(rnd() * DATA_COUNT) % DATA_COUNT;
    let off = Math.floor(rnd() * (DATA_SIZE - 64));
    return DATA_BASE + r * DATA_STRIDE + off;
}

/**
 * dataSeed(rnd)
 *
 * The value stored at a longword of readable DATA.  Full-width random, never zero: the point of
 * the pcjsvax-cd6 lesson is that a read returning zero from a never-written page compares equal
 * no matter how wrong the address was.  These values are never followed as pointers, so they do
 * not have to be valid addresses -- which is exactly why they can carry full entropy.
 *
 * @param {function(): number} rnd
 * @returns {number}
 */
function dataSeed(rnd)
{
    let v = (Math.floor(rnd() * 0x100000000) | 0);
    return v === 0 ? 0x5A5AA5A5 : v;
}

/**
 * buildPrologue(rnd, seedMem)
 *
 * SIMH commands that set up memory once: the SCB (every vector pointed at its own HALT, so an
 * execution-time fault stops the simulator instead of wandering, and so the FAULTS phase can tell
 * from the PC which vector was taken) and the seeded regions.  `seedMem` is filled with the same
 * values so the FAULTS phase can model memory in JS.
 *
 * @param {function(): number} rnd
 * @param {Map.<number, number>} seedMem
 * @returns {Array.<string>}
 */
function buildPrologue(rnd, seedMem)
{
    let L = [];
    L.push("set cpu " + (MEMSIZE >> 20) + "m");
    for (let k = 0; k < NUM_VECTORS; k++) {
        L.push(`d -l ${hex(k * 4)} ${hex(HANDLER_BASE + k * 4)}`);
    }
    let seed = (base, size, gen) => {
        for (let off = 0; off < size; off += 4) {
            let v = gen(rnd);
            seedMem.set(base + off, v);
            L.push(`d -l ${hex(base + off)} ${hex(v)}`);
        }
    };
    for (let r = 0; r < PTR_COUNT; r++) seed(PTR_BASE + r * PTR_STRIDE, PTR_SIZE, ptrSeed);
    for (let r = 0; r < DATA_COUNT; r++) seed(DATA_BASE + r * DATA_STRIDE, DATA_SIZE, dataSeed);
    seed(LOW_BASE, LOW_SIZE, dataSeed);
    seed(STACK, LOW_SIZE, dataSeed);
    return L;
}

/**
 * buildCaseScript(c, pc)
 *
 * @param {Object} c
 * @param {number} pc
 * @returns {Array.<string>}
 */
function buildCaseScript(c, pc)
{
    let L = [];
    const RNAME = ["R0","R1","R2","R3","R4","R5","R6","R7","R8","R9","R10","R11","AP","FP","SP"];
    for (let r = 0; r < 15; r++) L.push(`d ${RNAME[r]} ${hex(c.regs[r])}`);
    /*
     * Deposit ceil((len+1)/4) longwords so that the byte AFTER the instruction is always zero --
     * a HALT -- even when a previous, longer case left bytes behind.
     */
    let nlw = Math.ceil((c.bytes.length + 1) / 4);
    for (let w = 0; w < nlw; w++) {
        let v = 0;
        for (let b = 3; b >= 0; b--) v = ((v << 8) | (c.bytes[w * 4 + b] || 0)) >>> 0;
        L.push(`d -l ${hex(pc + w * 4)} ${hex(v)}`);
    }
    L.push(`go ${hex(pc)}`);
    return L;
}

/* ------------------------------------------------------------------------------------------- *
 * FAULTS phase: reserved addressing modes and recovery-queue unwind                             *
 * ------------------------------------------------------------------------------------------- */

/**
 * @class ModelMachine
 *
 * The decoder's machine interface backed by a model of the memory we deposited.  Used only by
 * the FAULTS phase, where the instruction faults during resolution and therefore produces no
 * history record -- and so no read log to replay against.  Everything it reads was put there by
 * buildPrologue(), and nothing in that phase ever executes, so the model cannot drift.
 */
class ModelMachine {
    /**
     * @param {Map.<number, number>} seedMem
     */
    constructor(seedMem)
    {
        this.regs = new Int32Array(16);
        this.mem = seedMem;
    }

    /**
     * @param {Int32Array} regs
     * @param {Uint8Array} bytes
     * @param {number} pc
     */
    load(regs, bytes, pc)
    {
        this.regs.set(regs);
        this.regs[15] = pc;
        this.iPC = pc;
        this.ibyt = bytes;
    }

    /**
     * @param {number} lnt
     * @returns {number}
     */
    getISTR(lnt)
    {
        let off = (this.regs[15] - this.iPC) | 0;
        let v = 0;
        for (let i = lnt - 1; i >= 0; i--) v = (v << 8) | (this.ibyt[off + i] || 0);
        this.regs[15] = (this.regs[15] + lnt) | 0;
        return lnt == 4 ? (v | 0) : v;
    }

    /**
     * @param {number} pc
     */
    setPC(pc)
    {
        this.regs[15] = pc | 0;
    }

    /**
     * @param {number} va
     * @param {number} lnt
     * @returns {number}
     */
    readData(va, lnt)
    {
        let v = 0;
        for (let i = lnt - 1; i >= 0; i--) {
            let a = (va + i) | 0;
            let lw = this.mem.get(a & ~3) || 0;
            v = ((v << 8) | ((lw >>> ((a & 3) * 8)) & 0xFF)) | 0;
        }
        return lnt == 4 ? (v | 0) : (v & ((1 << (lnt * 8)) - 1));
    }
}

/**
 * buildFaultCase(gen, opcodes)
 *
 * An instruction whose EARLIER specifiers modify registers and whose LAST specifier is a reserved
 * addressing mode.  SIMH resolves the earlier ones, faults on the last, and unwinds -- so the
 * register file the exception handler sees is the pre-instruction one.  That is the property
 * three Wave 2 items depend on and the reason fault recovery is in scope for this item.
 *
 * @param {Generator} gen
 * @param {Array.<number>} opcodes
 * @returns {Object|null}
 */
function buildFaultCase(gen, opcodes)
{
    /* need at least two specifiers: something to modify a register, then something illegal */
    let cands = opcodes.filter((opc) => {
        let nsp = DROM[opc * DROM_STRIDE] & DR.NSPMASK;
        if (nsp < 2) return false;
        for (let i = 1; i <= nsp; i++) if (specifierIsBranch(DROM[opc * DROM_STRIDE + i])) return false;
        return true;
    });
    if (!cands.length) return null;
    let opc = gen.choose(cands);
    let nsp = DROM[opc * DROM_STRIDE] & DR.NSPMASK;
    let bytes = [];
    if (opc >= 0x100) bytes.push(0xFD);
    bytes.push(opc & 0xFF);
    let state = {regsUsed: new Set(), unaligned: new Set()};
    let sideEffects = 0;
    for (let i = 1; i < nsp; i++) {
        let at = DROM[opc * DROM_STRIDE + i];
        /*
         * Force the leading specifiers to be autoincrement or autodecrement so there is always
         * something for the unwind to undo.  Address and write specifiers get the mode outright;
         * read and modify specifiers get it too, which additionally makes them read memory --
         * proving the unwind is not merely "nothing had happened yet".  Bases come from the
         * address-holding registers only, and never the stack pointer, because the exception
         * push writes below SP and the FAULTS phase models memory in JS.
         */
        let mode = gen.choose([MODE.ADC, MODE.AIN, MODE.AID]);
        let c = registerConstraint(at, mode);
        let max = Math.min(c.max, 13);
        let pool = (mode == MODE.AID) ? gen.pointerRegs(max, state) : gen.addrRegs(max);
        if (!pool) { mode = MODE.AIN; pool = gen.addrRegs(max); }
        let rn = gen.choose(pool);
        bytes.push(mode | rn);
        gen.emitTail(mode, rn, at, bytes, state);
        gen.noteSideEffect(mode, rn, at, state);
        sideEffects++;
    }
    if (!sideEffects) return null;
    /*
     * The illegal final specifier.  Which forms are reserved depends on the ACCESS TYPE, so the
     * choice is filtered rather than retried blindly: a short literal is only illegal for
     * modify/address/write, and register mode is illegal for the PC (all access types) and for
     * address access (any register).
     */
    let atLast = DROM[opc * DROM_STRIDE + nsp];
    let accLast = atLast & DR.ACMASK;
    let kinds = ["regdefpc", "autodecpc", "indexpc", "indeximm"];
    if (accLast != DR.A) kinds.push("regpc");
    if (accLast != DR.R && atLast != SPEC.RG) kinds.push("literal");
    let kind = gen.choose(kinds);
    switch (kind) {
    case "regpc":
        bytes.push(MODE.GRN | 15);
        break;
    case "regdefpc":
        bytes.push(MODE.RGD | 15);
        break;
    case "autodecpc":
        bytes.push(MODE.ADC | 15);
        break;
    case "literal":
        bytes.push(gen.pick(0x40));
        break;
    case "indexpc":
        bytes.push(MODE.IDX | 15);                          // the PC may not be an index register
        bytes.push(MODE.RGD | 4);
        break;
    case "indeximm":
        bytes.push(MODE.IDX | gen.choose(IDX_REGS));
        bytes.push(MODE.AIN | 15);                          // base[Rx] with an immediate base
        for (let k = 0; k < 4; k++) bytes.push(gen.pick(256));
        break;
    }
    return {opc, bytes, regs: gen.initialRegs(), kind};
}

/* ------------------------------------------------------------------------------------------- *
 * Phases                                                                                        *
 * ------------------------------------------------------------------------------------------- */

/**
 * newStats()
 *
 * @returns {Object}
 */
function newStats()
{
    return {records: 0, nontrivial: 0, reads: 0, recq: 0, branchDisp: 0, destChecks: 0, opcodes: new Set()};
}

/**
 * replayTrace(pathTrace, DecoderClass, opts)
 *
 * @param {string} pathTrace
 * @param {Function} DecoderClass
 * @param {Object} opts
 * @returns {Object} {stats, failures}
 */
function replayTrace(pathTrace, DecoderClass, opts = {})
{
    let machine = new ReplayMachine();
    let decoder = new DecoderClass(machine);
    let stats = opts.stats || newStats();
    let failures = [];
    /*
     * When the caller generated the cases, match records to them IN ORDER by instruction bytes.
     * A case whose record never appears faulted during resolution and was dropped by SIMH before
     * the history record was written -- the silent-skip failure mode.  Matching by bytes means
     * such a case is NAMED rather than merely counted.
     */
    let cases = opts.cases || null;
    let iCase = 0;
    let lost = [];
    const bytesOf = (a) => Array.from(a).map((b) => hex(b, 2)).join("");
    parseTrace(pathTrace, (rec) => {
        if (opts.filterPC !== undefined && rec.pc !== opts.filterPC) return;
        if (cases) {
            let want = bytesOf(rec.ibyt);
            while (iCase < cases.length && bytesOf(cases[iCase].bytes).slice(0, want.length) !== want) {
                lost.push(cases[iCase]);
                iCase++;
            }
            iCase++;
        }
        stats.records++;
        if (failures.length >= 5) return;
        let why = compareRecord(decoder, machine, rec, stats);
        if (why) {
            failures.push(`record ${rec.index} PC=${hex(rec.pc)} "${rec.detail.split("\n")[0]}": ${why}`);
        }
    });
    if (cases) while (iCase < cases.length) lost.push(cases[iCase++]);
    return {stats, failures, lost};
}

/**
 * phaseEHKAA(simh, DecoderClass, opts)
 *
 * @param {string} simh
 * @param {Function} DecoderClass
 * @param {Object} opts
 * @returns {Object}
 */
function phaseEHKAA(simh, DecoderClass, opts)
{
    let tracePath = opts.trace;
    if (!tracePath) {
        let ehkaa = opts.ehkaaExe;
        if (!fs.existsSync(ehkaa)) {
            throw new Error(`EHKAA diagnostic not found at ${ehkaa}; pass --ehkaa PATH or --skip-ehkaa`);
        }
        tracePath = path.join(opts.scratch, "ehkaa.trace");
        let script = [
            `set -d cpu history=100000:${tracePath}`,
            `load ${ehkaa}`,
            "go -q 200",
            "examine PC",
            "exit", ""
        ].join("\n");
        let out = runSimh(simh, script, path.join(opts.scratch, "ehkaa.ini"));
        if (!/PC:\s*80018AD1/.test(out)) {
            throw new Error("EHKAA did not halt at its documented PASS PC (0x80018AD1); SIMH said:\n" + out);
        }
    }
    return replayTrace(tracePath, DecoderClass);
}

/**
 * phaseExerciser(simh, DecoderClass, opts)
 *
 * @param {string} simh
 * @param {Function} DecoderClass
 * @param {Object} opts
 * @returns {Object}
 */
function phaseExerciser(simh, DecoderClass, opts)
{
    let rnd = mulberry32(opts.seed ^ 0x5EED0001);
    let gen = new Generator(rnd);
    let opcodes = reachableOpcodes().filter((opc) => !EXCLUDE.has(OPCODES[opc]) && !(DROM[opc * DROM_STRIDE] & DR.F));
    let cases = [];
    for (let i = 0; i < opts.cases; i++) cases.push(gen.buildCase(opcodes));

    /*
     * WHY THIS RUNS IN ROUNDS.  The generated instructions do not merely decode -- SIMH executes
     * them, and one whose destination lands in a seeded region overwrites a value a LATER case
     * was going to follow as a pointer.  That later case then reads a spliced address, takes a
     * machine check during resolution, and is dropped before its history record is written.
     *
     * Rather than pretend that cannot happen, or quietly tolerate the loss, every case dropped in
     * one round is re-run in the next against freshly deposited memory.  A case dropped because
     * of memory drift succeeds on retry; a case dropped because the GENERATOR emitted a reserved
     * addressing mode is dropped every time and is reported by name.  The run therefore either
     * compares every case it generated or says exactly which ones it could not.
     */
    let stats = newStats();
    let failures = [];
    let pending = cases;
    for (let round = 0; pending.length && round < 4; round++) {
        let seedMem = new Map();
        let L = buildPrologue(rnd, seedMem);
        let tracePath = path.join(opts.scratch, `exerciser${round}.trace`);
        if (fs.existsSync(tracePath)) fs.unlinkSync(tracePath);
        L.unshift(`set -d cpu history=100000:${tracePath}`);
        for (let c of pending) L.push(...buildCaseScript(c, TESTPC));
        L.push("exit", "");
        runSimh(simh, L.join("\n"), path.join(opts.scratch, `exerciser${round}.ini`));
        let r = replayTrace(tracePath, DecoderClass, {filterPC: TESTPC, cases: pending, stats});
        failures.push(...r.failures);
        if (round == 0) opts.exerciserTrace = tracePath;
        if (r.lost.length === pending.length) { pending = r.lost; break; }
        pending = r.lost;
    }
    return {stats, failures, lost: pending, gen};
}

/**
 * phaseFaults(simh, DecoderClass, opts)
 *
 * Runs the reserved-addressing cases and compares OUR post-unwind register file against the one
 * SIMH's exception handler saw.  The handler is a HALT at HANDLER_BASE + vector, so its trace
 * record's pre-decode register dump IS the unwound state -- after the exception has pushed PC and
 * PSL, which is why the stack pointer is compared against `unwound SP - 8` rather than directly.
 *
 * @param {string} simh
 * @param {Function} DecoderClass
 * @param {Object} opts
 * @returns {Object}
 */
function phaseFaults(simh, DecoderClass, opts)
{
    let rnd = mulberry32(opts.seed ^ 0x5EED0002);
    let gen = new Generator(rnd);
    let opcodes = reachableOpcodes().filter((opc) => !EXCLUDE.has(OPCODES[opc]) && !(DROM[opc * DROM_STRIDE] & DR.F));
    let seedMem = new Map();
    let L = buildPrologue(rnd, seedMem);
    let tracePath = path.join(opts.scratch, "faults.trace");
    if (fs.existsSync(tracePath)) fs.unlinkSync(tracePath);
    L.unshift(`set -d cpu history=100000:${tracePath}`);
    let cases = [];
    let guard = 0;
    while (cases.length < opts.faults && guard++ < opts.faults * 20) {
        let c = buildFaultCase(gen, opcodes);
        if (!c) continue;
        cases.push(c);
        L.push(...buildCaseScript(c, TESTPC));
    }
    L.push("exit", "");
    runSimh(simh, L.join("\n"), path.join(opts.scratch, "faults.ini"));

    /*
     * Collect, in order, the handler records (a fault happened, and which vector) and any record
     * at TESTPC (the instruction did NOT fault -- a generator bug, which must be reported rather
     * than silently skewing the pairing).
     */
    let handlerRecs = [], noFault = 0;
    parseTrace(tracePath, (rec) => {
        if (rec.pc === TESTPC) { noFault++; return; }
        if (rec.pc >= HANDLER_BASE && rec.pc < HANDLER_BASE + NUM_VECTORS * 4) handlerRecs.push(rec);
    });

    let failures = [];
    let machine = new ModelMachine(seedMem);
    let decoder = new DecoderClass(machine);
    let nCompared = 0, nUnwound = 0;
    if (noFault) {
        failures.push(`${noFault} of ${cases.length} fault case(s) did not fault in SIMH at all -- the generator emitted a legal specifier, so the pairing between cases and handler records is unreliable`);
    } else {
        for (let k = 0; k < cases.length && failures.length < 5; k++) {
            let c = cases[k], rec = handlerRecs[k];
            if (!rec) { failures.push(`fault case ${k} (${OPCODES[c.opc]}, ${c.kind}): SIMH produced no exception-handler record`); continue; }
            let vector = rec.pc - HANDLER_BASE;
            if (vector !== VEC_RESAD) {
                failures.push(`fault case ${k} (${OPCODES[c.opc]}, ${c.kind}): SIMH took SCB vector ${hex(vector, 2)}, expected reserved-addressing (${hex(VEC_RESAD, 2)})`);
                continue;
            }
            machine.load(c.regs, Uint8Array.from(c.bytes), TESTPC);
            let faulted = false;
            try {
                decoder.decode(false);
            } catch (e) {
                if (e && e.name !== "VAXFault") throw e;
                faulted = true;
            }
            if (!faulted) {
                failures.push(`fault case ${k} (${OPCODES[c.opc]}, ${c.kind}): SIMH raised a reserved-addressing fault, our decoder did not`);
                continue;
            }
            if (decoder.recqptr) nUnwound++;
            decoder.unwind();
            for (let r = 0; r < 14; r++) {
                if (machine.regs[r] !== rec.preg[r]) {
                    failures.push(`fault case ${k} (${OPCODES[c.opc]}, ${c.kind}): after unwind R${r} is ${hex(machine.regs[r])}, SIMH's handler saw ${hex(rec.preg[r])}`);
                    break;
                }
            }
            /* the exception pushed PC and PSL, so SIMH's SP is 8 below the unwound value */
            let expectSP = (machine.regs[14] - 8) | 0;
            if (expectSP !== rec.preg[14]) {
                failures.push(`fault case ${k} (${OPCODES[c.opc]}, ${c.kind}): after unwind SP is ${hex(machine.regs[14])}, so the handler should have seen ${hex(expectSP)} after its 8-byte exception push, but saw ${hex(rec.preg[14])}`);
            }
            decoder.resetRecovery();
            nCompared++;
        }
    }
    return {cases: cases.length, nCompared, nUnwound, failures, gen};
}

/* ------------------------------------------------------------------------------------------- *
 * Mutation self-check                                                                           *
 * ------------------------------------------------------------------------------------------- */

/*
 * Each mutation is a textual edit to decode.js that MUST be caught.  They are chosen to sit on
 * the exact semantics this item exists to get right, and each names the phase expected to catch
 * it -- a mutation caught by the wrong phase would mean a phase is not testing what it claims.
 */
const MUTATIONS = [
    {
        name: "autoinc-read-after-bump",
        why: "read the operand AFTER autoincrementing the register, i.e. off by the operand length",
        from: "                        opnd[j++] = cpu.readData(regs[rn], lnt, false);\n" +
              "                        regs[rn] = (regs[rn] + lnt) | 0;",
        to:   "                        regs[rn] = (regs[rn] + lnt) | 0;\n" +
              "                        opnd[j++] = cpu.readData(regs[rn], lnt, false);"
    },
    {
        name: "index-not-scaled",
        why: "fail to scale the index register by the operand length",
        from: "                index = regs[rn] << (at & DR.LNMASK);",
        to:   "                index = regs[rn] << 0;"
    },
    {
        name: "byte-disp-unsigned",
        why: "treat a byte displacement as unsigned instead of sign-extending it",
        from: "            temp = this.cpu.getISTR(1);\n" +
              "            return (temp & 0x80) ? (temp | ~0xFF) : temp;",
        to:   "            temp = this.cpu.getISTR(1);\n" +
              "            return temp;"
    },
    {
        name: "autoincdef-bumps-by-operand-length",
        why: "autoincrement-deferred must bump the pointer register by 4, not by the operand length",
        from: "            va = cpu.readData(regs[rn], 4, false);\n" +
              "            regs[rn] = (regs[rn] + 4) | 0;\n" +
              "            this.recordRecovery(MODE.AID | SPEC.RL, rn);",
        to:   "            va = cpu.readData(regs[rn], 4, false);\n" +
              "            regs[rn] = (regs[rn] + 8) | 0;\n" +
              "            this.recordRecovery(MODE.AID | SPEC.RL, rn);"
    },
    {
        name: "modify-uses-read-access",
        why: "resolve a modify specifier with read access, so a read-only page would not fault",
        from: "                    opnd[j++] = cpu.readData(va = regs[rn], lnt, true);\n" +
              "                    break;\n" +
              "                case SPEC.MQ:\n" +
              "                    if (rn == nPC) this.fault(VAXFAULT.RESAD);\n" +
              "                    opnd[j++] = cpu.readData(va = regs[rn], 4, true);",
        to:   "                    opnd[j++] = cpu.readData(va = regs[rn], lnt, false);\n" +
              "                    break;\n" +
              "                case SPEC.MQ:\n" +
              "                    if (rn == nPC) this.fault(VAXFAULT.RESAD);\n" +
              "                    opnd[j++] = cpu.readData(va = regs[rn], 4, true);"
    },
    {
        name: "unwind-wrong-direction",
        why: "undo an autoincrement by incrementing again instead of decrementing",
        from: "            if (e & RQ.DIR) {\n" +
              "                regs[rrn] = regs[rrn] - rlnt;\n" +
              "            } else {\n" +
              "                regs[rrn] = regs[rrn] + rlnt;\n" +
              "            }",
        to:   "            if (e & RQ.DIR) {\n" +
              "                regs[rrn] = regs[rrn] + rlnt;\n" +
              "            } else {\n" +
              "                regs[rrn] = regs[rrn] - rlnt;\n" +
              "            }",
        phase: "faults"
    },
    {
        name: "autodec-no-recovery-entry",
        why: "omit the recovery-queue entry for autodecrement, so a later fault leaves the register modified",
        from: "                    regs[rn] = (regs[rn] - lnt) | 0;\n" +
              "                    this.recordRecovery(disp, rn);\n" +
              "                    if (rn == nPC) this.fault(VAXFAULT.RESAD);\n" +
              "                    opnd[j++] = cpu.readData(va = regs[rn], lnt, false);",
        to:   "                    regs[rn] = (regs[rn] - lnt) | 0;\n" +
              "                    if (rn == nPC) this.fault(VAXFAULT.RESAD);\n" +
              "                    opnd[j++] = cpu.readData(va = regs[rn], lnt, false);"
    },
    {
        name: "destination-predicate-includes-register",
        why: "treat a register destination as a memory destination, so results would be written to the wrong place",
        from: "        return this.spec > (MODE.GRN | nPC);",
        to:   "        return this.spec >= MODE.GRN;"
    },
    {
        name: "branch-disp-sign-extended",
        why: "sign-extend the branch displacement in the decoder; the architecture leaves that to the branch body",
        from: "                this.brdisp = cpu.getISTR(1 << (at & 1));",
        to:   "                this.brdisp = cpu.getISTR(1 << (at & 1)) | 0x80000000;"
    }
];

/**
 * loadMutant(mutation, scratch)
 *
 * Writes a copy of decode.js with one deliberate defect and imports it.  The copy sits beside the
 * original so its relative import of drom.js still resolves; it is deleted afterwards.
 *
 * @param {Object|null} mutation
 * @param {string} scratch
 * @returns {Promise.<Function>}
 */
async function loadMutant(mutation, scratch)
{
    let src = path.join(REPO_ROOT, "machines/dec/vax/modules/v2/decode.js");
    let text = fs.readFileSync(src, "utf8");
    if (!mutation) return (await import("../modules/v2/decode.js")).VAXDecoder;
    let n = text.split(mutation.from).length - 1;
    if (n !== 1) {
        throw new Error(`mutation "${mutation.name}" matches its target ${n} time(s) in decode.js; it must match exactly once, or it is not testing what it claims`);
    }
    let mutated = text.replace(mutation.from, mutation.to);
    let file = path.join(path.dirname(src), `.mutant-${mutation.name}-${process.pid}.js`);
    fs.writeFileSync(file, mutated);
    try {
        return (await import(fileURLToPath(new URL("file://" + file)))).VAXDecoder;
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
    let shortOpts = Object.assign({}, opts, {cases: 600, faults: 300});
    let ok = true;

    /* one shared capture per phase, so the self-check costs one SIMH run, not one per mutation */
    let Base = await loadMutant(null, opts.scratch);
    let ex = phaseExerciser(simh, Base, shortOpts);
    let exTrace = shortOpts.exerciserTrace;
    let fa = phaseFaults(simh, Base, shortOpts);
    if (ex.failures.length || fa.failures.length) {
        console.log("  BASELINE FAILED -- the unmutated decoder must pass before mutations mean anything:");
        for (let f of ex.failures.concat(fa.failures)) console.log("    " + f);
        return false;
    }
    console.log(`  baseline: exerciser ${ex.stats.records} records, faults ${fa.nCompared} cases -- clean\n`);

    for (let m of MUTATIONS) {
        let Mutant = await loadMutant(m, opts.scratch);
        let caught = "";
        if (m.phase !== "faults") {
            let r = replayTrace(exTrace, Mutant, {filterPC: TESTPC});
            if (r.failures.length) caught = "exerciser: " + r.failures[0];
        }
        if (!caught) {
            let r = phaseFaults(simh, Mutant, Object.assign({}, shortOpts, {reuse: true}));
            if (r.failures.length) caught = "faults: " + r.failures[0];
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
    let scratchArg = getArg("--scratch", null);
    /* Only a directory THIS run created is ours to remove -- a caller-supplied --scratch is the
       caller's, never auto-deleted here.  `||`, not a default *argument*, so mkdtempSync() runs
       only when --scratch was NOT given: the previous `getArg("--scratch", fs.mkdtempSync(...))`
       evaluated its default eagerly on every call, so passing --scratch explicitly still created
       and then orphaned one throwaway vaxdecode-* directory per invocation (HANDOFF.md
       pcjsvax-bd1). */
    let autoScratch = scratchArg === null;
    let opts = {
        cases: +getArg("--cases", 4000),
        faults: +getArg("--faults", 600),
        seed: +getArg("--seed", 0xC0FFEE),
        trace: getArg("--trace", null),
        scratch: scratchArg || fs.mkdtempSync(path.join(os.tmpdir(), "vaxdecode-")),
        ehkaaExe: getArg("--ehkaa", path.join(vaxRepo(), "open-simh/VAX/tests/ehkaa.exe"))
    };
    let simh = findSimh(getArg("--simh", null));
    let fSkipEHKAA = argv.indexOf("--skip-ehkaa") >= 0;
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;
    let keepScratch = false;

    console.log(`VAX decode differential -- SIMH: ${simh}`);
    console.log(`seed=0x${hex(opts.seed)} scratch=${opts.scratch}\n`);

    /* Every exit path -- --selfcheck (pass or fail), the differential's own deliberate
       evidence-preserving FAIL (see the "kept in" message below, unchanged), and an unexpected
       thrown error -- runs through this try/finally.  HANDOFF.md pcjsvax-bd1: an earlier revision
       called process.exit() directly from the --selfcheck branch and let uncaught exceptions
       reach the bottom .catch(), and NEITHER path ever removed opts.scratch. */
    try {
        if (fSelfCheck) {
            let ok = await selfCheck(simh, opts);
            console.log(ok ? "\nSELF-CHECK PASS: every injected defect was detected."
                           : "\nSELF-CHECK FAIL: at least one injected defect went undetected.");
            if (!ok) process.exitCode = 1;
            return;
        }

        let VAXDecoder = await loadMutant(null, opts.scratch);
        let problems = [];

    /* ---------------------------------------------------------------- EHKAA */
    let ehkaa = null;
    if (!fSkipEHKAA) {
        ehkaa = phaseEHKAA(simh, VAXDecoder, opts);
        let s = ehkaa.stats;
        let ratio = s.records ? s.nontrivial / s.records : 0;
        console.log("EHKAA replay");
        console.log(`  records replayed ......... ${s.records}`);
        console.log(`  distinct opcodes ......... ${s.opcodes.size}`);
        console.log(`  decode-time reads ........ ${s.reads}`);
        console.log(`  register side effects .... ${s.recq}`);
        console.log(`  branch displacements ..... ${s.branchDisp}`);
        console.log(`  destination predicates ... ${s.destChecks}`);
        console.log(`  non-trivial records ...... ${s.nontrivial} (${(ratio * 100).toFixed(1)}%)`);
        for (let f of ehkaa.failures) problems.push("EHKAA: " + f);
        if (s.records < MIN_EHKAA_RECORDS) {
            problems.push(`COVERAGE: EHKAA replayed only ${s.records} records, floor is ${MIN_EHKAA_RECORDS} -- an undersized or truncated capture proves nothing`);
        }
        if (ratio < MIN_EHKAA_NONTRIVIAL) {
            problems.push(`COVERAGE: only ${(ratio * 100).toFixed(1)}% of EHKAA records were non-trivial, floor is ${(MIN_EHKAA_NONTRIVIAL * 100).toFixed(0)}%`);
        }
        console.log("");
    }

    /* ----------------------------------------------------------- EXERCISER */
    let ex = phaseExerciser(simh, VAXDecoder, opts);
    let s = ex.stats;
    let ratio = s.records ? s.nontrivial / s.records : 0;
    console.log("Randomized addressing-mode exerciser");
    console.log(`  cases generated .......... ${opts.cases}`);
    console.log(`  cases decoded by SIMH .... ${s.records}`);
    console.log(`  mode/access combinations . ${ex.gen.combos.size}`);
    console.log(`  index-mode specifiers .... ${ex.gen.indexCases}`);
    console.log(`  register-conflict cases .. ${ex.gen.conflicts}`);
    console.log(`  PC-relative specifiers ... ${ex.gen.pcRelative}`);
    console.log(`  immediate/absolute ....... ${ex.gen.immediates}`);
    console.log(`  decode-time reads ........ ${s.reads}`);
    console.log(`  register side effects .... ${s.recq}`);
    console.log(`  destination predicates ... ${s.destChecks}`);
    console.log(`  non-trivial records ...... ${s.nontrivial} (${(ratio * 100).toFixed(1)}%)`);
    for (let f of ex.failures) problems.push("EXERCISER: " + f);
    if (s.records < MIN_CASES) {
        problems.push(`COVERAGE: only ${s.records} exerciser cases reached the decoder, floor is ${MIN_CASES}`);
    }
    if (ex.lost && ex.lost.length) {
        let names = ex.lost.slice(0, 6).map((c) => `${OPCODES[c.opc]} [${Array.from(c.bytes).map((b) => hex(b, 2)).join(" ")}]`);
        problems.push(`COVERAGE: ${ex.lost.length} generated case(s) produced no history record -- they faulted DURING resolution, so SIMH dropped them before writing one and they were never compared.  That is the silent-skip failure mode this assertion exists to catch.  First few: ${names.join("; ")}`);
    }
    if (ex.gen.combos.size < MIN_MODE_COMBOS) {
        problems.push(`COVERAGE: only ${ex.gen.combos.size} distinct mode/access-type combinations generated, floor is ${MIN_MODE_COMBOS}`);
    }
    if (ex.gen.conflicts < MIN_CONFLICT_CASES) {
        problems.push(`COVERAGE: only ${ex.gen.conflicts} register-conflict specifiers generated, floor is ${MIN_CONFLICT_CASES} -- those are the cases a naive generator never emits`);
    }
    if (s.destChecks < MIN_CASES / 8) {
        problems.push(`COVERAGE: the destination predicate was exercised only ${s.destChecks} time(s); it must be graded on a substantial fraction of cases or it is not covered`);
    }
    if (ratio < MIN_EXERCISER_NONTRIVIAL) {
        problems.push(`COVERAGE: only ${(ratio * 100).toFixed(1)}% of exerciser records were non-trivial, floor is ${(MIN_EXERCISER_NONTRIVIAL * 100).toFixed(0)}% -- the seeded memory is not being hit`);
    }
    /* every addressing mode must appear at least once */
    let modesSeen = new Set([...ex.gen.combos].map((c) => c.split("/")[0].replace(/^index\+.*/, "index")));
    let modesWanted = ["literal", "register", "regdef", "autodec", "autoinc", "autoincdef",
                       "bytedisp", "bytedispdef", "worddisp", "worddispdef", "longdisp", "longdispdef", "index"];
    let missing = modesWanted.filter((m) => !modesSeen.has(m));
    if (missing.length) {
        problems.push(`COVERAGE: addressing mode(s) never generated: ${missing.join(", ")}`);
    }
    console.log("");

    /* -------------------------------------------------------------- FAULTS */
    let fa = phaseFaults(simh, VAXDecoder, opts);
    console.log("Reserved-addressing fault and recovery-queue unwind");
    console.log(`  fault cases .............. ${fa.cases}`);
    console.log(`  compared after unwind .... ${fa.nCompared}`);
    console.log(`  cases with side effects .. ${fa.nUnwound}`);
    for (let f of fa.failures) problems.push("FAULTS: " + f);
    if (fa.nCompared < MIN_FAULT_CASES) {
        problems.push(`COVERAGE: only ${fa.nCompared} fault cases compared, floor is ${MIN_FAULT_CASES}`);
    }
    if (fa.nUnwound < fa.nCompared * 0.9) {
        problems.push(`COVERAGE: only ${fa.nUnwound} of ${fa.nCompared} fault cases had register side effects to unwind -- the phase is not testing recovery`);
    }
    console.log("");

    if (problems.length) {
        console.log("FAIL");
        for (let p of problems) console.log("  " + p);
        console.log(`\nreproduce with --seed ${opts.seed}; SIMH scripts and traces kept in ${opts.scratch}`);
        process.exitCode = 1;
        keepScratch = true;                 // deliberate: this FAIL keeps scratch for reproduction
        return;
    }
    console.log("PASS: decode and operand resolution are indistinguishable from SIMH across every phase.");
    } catch (e) {
        console.error("ERROR: " + (e && e.stack || e));
        process.exitCode = 2;
    } finally {
        if (autoScratch && !keepScratch) fs.rmSync(opts.scratch, {recursive: true, force: true});
    }
}

main();
