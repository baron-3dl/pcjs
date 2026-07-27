/**
 * @fileoverview Differential test: the KA655 cache diagnostic space (CDG) vs. a real Open SIMH
 *               microvax3900, at register level
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-0b7.  romdiff.js's boundary walk reached instruction #393, a `MOVC5` into CDG+0x0, and
 * cdg.js decodes that range.  Making the MOVC5 merely *complete* would move romdiff's boundary
 * forward while implementing nothing, so this file grades the four things the range actually DOES,
 * each against a live oracle, each in its own phase:
 *
 *   ALIAS   The 64MB address range is backed by a 64KB store.  `CDG_GETROW(x) = ((x)&0xFFFF)>>2`
 *           discards every address bit above 15, so CDG+0x0, CDG+0x10000, CDG+0x3FF0000 ... are
 *           ONE longword.  Proved by writing through one member of a pair and reading back through
 *           the other, ON BOTH ENGINES -- and, so the check cannot be satisfied by a degenerate row
 *           function that maps everything to row 0, by NON-alias pairs that must NOT see each
 *           other.
 *
 *   CACR    Every CDG READ writes the KA655 CACR: vax_sysdev.c's cdg_rd() clears CACR_DRO and ORs
 *           in four diagnostic-parity bits computed from the longword just read, with an
 *           ALTERNATING seed (bytes 3 and 1 seed odd=1, bytes 2 and 0 seed odd=0).  All sixteen
 *           per-lane parity combinations are driven, so a wrong seed on any lane, or a swapped
 *           pair of bit positions, changes an observed value; random longwords follow.
 *
 *   MERGE   cdg_wr() does its OWN byte/word read-modify-merge (unlike the SSC, whose merges happen
 *           in the caller path).  Byte writes are graded at ALL FOUR byte lanes; word writes at the
 *           two lanes that are genuinely aligned words.  See "WHY WORD LANES 1 AND 3 ARE NOT HERE".
 *
 *   BACKED  Every address in [CDG_BASE, CDG_BASE + CDG_LENGTH) reads and writes without a machine
 *           check, on both engines -- the claim mchkdiff.js's calibration already encodes for the
 *           SIMH side and this bus previously contradicted.
 *
 *   SELFCHECK  --selfcheck perturbs the SHIPPED cdg.js/ka655.js code path (never a private copy of
 *           it -- standing rule 11) with six named defects and fails if any one survives.
 *
 * HOW THE TWO SIDES ARE DRIVEN
 * ----------------------------
 * SIMH runs REAL VAX INSTRUCTIONS, deposited into RAM and stepped, with the results read back out
 * of R0..R5 -- the same "only an actual instruction reproduces the real access path" reasoning
 * romdiff.js's verifySscBaseRandom()/FaultGrader.probeSimh() are built on.  It matters more here than
 * usual: SIMH's own `examine`/`deposit` of a CDG address goes through cpu_ex()/cpu_dep(), which
 * DOES reach cdg_rd() -- and would therefore perturb the very CACR this file is measuring between
 * the operation and its readback.  Every observable is captured by an instruction instead.
 *
 * Each case gets `reset -p all`, not `reset all`: `ka_cacr = 0` lives in sysd_powerup()
 * (vax_sysdev.c:1786), and sysd_reset() only calls it under `-p` (:1756).  Without the `-p` the
 * diagnostic-parity bits of every previous case leak into the next one, because -- see below --
 * they are never cleared.
 *
 * The JS side drives the BUS directly (BusVAX.setLong/getLong/...), not the CPU: instruction
 * execution is romdiff.js's and cpudiff.js's claim, already proved over 392 and 335,444
 * instructions respectively.  What is under test here is what the bus and the device do with an
 * access once it arrives.  Both sides are generated from ONE declarative op list per case
 * (`Case.ops`), so the two descriptions cannot drift apart.
 *
 * ONE MACHINE, REUSED -- AND WHY THAT IS ALSO MORE FAITHFUL, NOT JUST CHEAPER
 * --------------------------------------------------------------------------
 * An earlier version of this file built a FRESH machine per case.  BusVAX.addCdg() installs a
 * controller over 64MB, which is VAX.PHYSMEM.CDG_LENGTH / BusVAX.BLOCK_SIZE = 8192 MemoryVAX blocks
 * per machine.  MEASURED: 8.6GB RSS forty seconds in, still climbing, and the kernel OOM killer took
 * the whole box down -- three times.  This file now builds ONE machine (`machine()`), and
 * `runCaseJS()` resets only what a case actually dirties.  See bus.js's addCdg() and cdg.js's
 * makeCdgController() for the two halves of the underlying fix.
 *
 * What a case dirties is exactly the KA655 CACR, and that is reset per case -- because SIMH's
 * `reset -p all` zeroes `ka_cacr` (sysd_powerup, vax_sysdev.c:1786).  The CDG STORE is deliberately
 * NOT cleared per case, because SIMH does not clear it either: `cdg_dat` is a plain static array
 * (vax_sysdev.c:255) that no reset routine touches, so it carries across cases on the oracle.  The
 * per-case fresh machine was silently WRONG about that and got away with it only because every case
 * writes each location before reading it.  The store is zeroed once per grading PASS instead, which
 * is the state SIMH's process actually starts in.
 *
 * MEMORY IS PART OF THE DONE CONDITION.  Peak `heapUsed + external` is sampled per case and per
 * batch, and a run that exceeds MAX_HEAP_BYTES FAILS -- an absolute bound that does not scale with
 * the case count (standing rule 4), so the OOM defect cannot come back silently.  Every invocation
 * in this project's gate runs under `--max-old-space-size=2048`.
 *
 * THE CACR DIAGNOSTIC-PARITY BITS ARE NOT CLEARED BETWEEN READS -- MEASURED, NOT ASSUMED
 * -------------------------------------------------------------------------------------
 * `CACR_DRO` is 0x00FFFF00 (bits 8-23) and `CACR_V_DPAR` is 24, so cdg_rd()'s `ka_cacr &= ~CACR_DRO`
 * does NOT cover the four parity bits it then ORs in: they ACCUMULATE across reads.  This reads
 * like a SIMH bug and is not what a careful reader predicts, so this file does not take it on
 * faith -- the ACCUM cases below read two different longwords in sequence and compare the resulting
 * CACR against the live oracle, whichever way it goes.
 *
 * A consequence worth stating because it bounds what --selfcheck can honestly claim: NOTHING on
 * either engine ever SETS a CACR_DRO bit (ka_wr writes only CACR_FIXED|CACR_W1C|CACR_RW, bits 0-6;
 * cdg_rd writes only bits 24-27), so `& ~CACR_DRO` is provably inert and a "CACR_DRO not cleared"
 * mutation is UNDETECTABLE BY ANY DIFFERENTIAL -- the oracle itself cannot tell.  Rather than ship
 * a mutation that certifies coverage it does not have, this file (a) ships the DETECTABLE
 * neighbouring defect instead -- a DRO clear mask widened to also cover the parity bits, which
 * destroys the accumulation and IS caught -- and (b) asserts the inertness premise LIVE, on every
 * CACR case: if the oracle ever returns a CACR with a bit in 8-23 set, this run FAILS and the
 * paragraph you are reading is wrong.
 *
 * WHY WORD LANES 1 AND 3 ARE NOT IN THE MERGE PHASE
 * ------------------------------------------------
 * A word at physical offset 1 or 3 is not naturally aligned, so the VAX write path does not reach
 * WriteReg()/cdg_wr() with `lnt == L_WORD` at all -- it goes through WriteRegU()
 * (vax_sysdev.c:1084), which read-modify-writes via ReadReg()+WriteReg() and therefore drags
 * cdg_rd()'s CACR side effect along with a WRITE.  This bus reaches an unaligned register write
 * through MemoryVAX's own byte/word stitching, which does not reproduce that.  Modeling WriteRegU
 * is a bus-level change, not a CDG-level one, and is out of pcjsvax-0b7's scope; cdg.js's header
 * states the gap in the same words.  All four byte lanes ARE graded -- and byte writes are what the
 * ROM's MOVC5 at instruction #393 actually issues.
 *
 *      node machines/dec/vax/tests/cdgdiff.js [options]
 *        --simh PATH        patched microvax3900; else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch
 *                            build (the same search every other differential here uses)
 *        --cacr-cases N     randomized CACR longword cases (default 24, floor 8)
 *        --alias-cases N    randomized alias pairs (default 16, floor 8)
 *        --seed S           PRNG seed, printed on failure so it reproduces
 *        --selfcheck        prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import { OPCODES } from "../modules/v2/drom.js";
import KA655VAX from "../modules/v2/ka655.js";
import CQBICVAX from "../modules/v2/cqbic.js";
import CDGVAX, { CDASIZE, CDG_ROWS } from "../modules/v2/cdg.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const MEMSIZE   = 0x01000000;                           // 16MB, same default as every sibling
const CDG_BASE  = VAX.PHYSMEM.CDG_BASE >>> 0;
const CDG_LEN   = VAX.PHYSMEM.CDG_LENGTH >>> 0;
const CACR_ADDR = (VAX.PHYSMEM.REG_BASE + 0x4000) >>> 0;    // KABASE+0, the CACR
const CACR_DRO  = 0x00FFFF00;                           // vax_sysdev.c:131 -- see the file header

const CODE      = 0x00104000;                           // where each case's instructions are placed
const R_SCBB    = 0x00100000;
const R_HANDLER = 0x00102000;
const R_KSP     = 0x00110000;
const R_IS      = 0x00118000;

/* Enforced coverage floors.  FIXED -- they do not scale with the case count (standing rule 4), so
   shrinking a randomized phase cannot buy a green run. */
const CACR_CASES_FLOOR  = 8;
const ALIAS_CASES_FLOOR = 8;
const ALIAS_PAIRS_REQUIRED    = 8;      // deterministic same-row pairs
const ALIAS_NONPAIRS_REQUIRED = 6;      // deterministic DIFFERENT-row pairs
const CACR_LANE_COMBOS        = 16;     // all 2^4 per-lane parity combinations
const CACR_ACCUM_REQUIRED     = 4;      // two-read cases, for the accumulation question
const MERGE_BYTE_LANES        = 4;
const MERGE_WORD_LANES        = 2;
const BACKED_ADDRS_REQUIRED   = 26;
const BACKED_WINDOWS_REQUIRED = 10;     // distinct 64KB alias windows touched by BACKED

/*
 * ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure.  Not a floor that scales
 * with the case count -- the whole point is that adding cases must NOT cost memory, because the
 * machine is built once.  Measured steady state for the full run including --selfcheck is well
 * under 200MB; 384MB leaves room for GC scheduling without leaving room for the 8.6GB regression
 * this bound exists to catch.
 */
const MAX_HEAP_BYTES = 384 * 1024 * 1024;
let PEAK_HEAP = 0;

/**
 * sampleHeap()
 *
 * `external` is included because ArrayBuffer bytes (the 16MB RAM Int32Array, CDGVAX's 64KB store)
 * live outside heapUsed; a bound on heapUsed alone would not see a buffer leak.
 *
 * @returns {number} bytes
 */
function sampleHeap()
{
    let mu = process.memoryUsage();
    let used = mu.heapUsed + mu.external;
    if (used > PEAK_HEAP) PEAK_HEAP = used;
    return used;
}

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Plumbing -- the same shape as romdiff.js's, deliberately                                      *
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
    throw new Error("cdgdiff needs a REAL SIMH microvax3900; it has no fixture fallback.  Build one\n" +
        "with machines/dec/vax/tests/simh/build.sh and pass --simh PATH.  Tried:\n  " + candidates.join("\n  "));
}

function runSimh(bin, script, iniPath, timeoutMs = 5 * 60 * 1000)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: timeoutMs});
}

/** The same mulberry32 every VAX differential in this tree uses; duplicated because none of them
    share a utility module (see busdiff.js/mchkdiff.js/romdiff.js). */
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
 * The op list -- ONE description, two engines                                                   *
 * ------------------------------------------------------------------------------------------- */

/**
 * An op is {kind, addr, val, reg}:
 *   {kind: "wl"|"ww"|"wb", addr, val}   write a longword / word / byte
 *   {kind: "rl", addr, reg}             read a longword into Rn (and observe it)
 *
 * A CACR read is just an "rl" of CACR_ADDR: on both engines it is an ordinary register read with
 * no side effect of its own, which is exactly why it can be used to observe cdg_rd()'s.
 */

const OP_MOVL = OPCODES.indexOf("MOVL");
const OP_MOVW = OPCODES.indexOf("MOVW");
const OP_MOVB = OPCODES.indexOf("MOVB");
if (OP_MOVL < 0 || OP_MOVW < 0 || OP_MOVB < 0) {
    throw new Error("cdgdiff.js: MOVL/MOVW/MOVB not found in drom.js OPCODES");
}

/**
 * encodeOp(op)
 *
 * @param {Object} op
 * @returns {Array.<number>} instruction bytes (exactly ONE instruction)
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
    case "ww": {
        let v = op.val & 0xFFFF;
        return [OP_MOVW, 0x8F, v & 0xFF, (v >>> 8) & 0xFF, ...abs];
    }
    case "wb":
        return [OP_MOVB, 0x8F, op.val & 0xFF, ...abs];
    case "rl":
        return [OP_MOVL, ...abs, 0x50 | op.reg];
    }
    throw new Error("cdgdiff.js: bad op kind " + op.kind);
}

/**
 * makeMachine()
 *
 * RAM at 0, the KA655 register pair (so CACR is readable through the SAME bus the CDG side effect
 * writes it through), and CDG.  No CPU: this file drives the bus, see the file header.  CQBIC is
 * present only because BusVAX.addRegBlock()'s device list is how REG_BASE gets decoded at all and
 * romdiff.js's machine has it -- nothing here touches it.
 *
 * @returns {Object} {bus, ka, cdg}
 */
function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let ka = new KA655VAX();
    bus.addRegBlock([
        {base: VAX.PHYSMEM.REG_BASE >>> 0, length: 0x14, dev: new CQBICVAX()},
        {base: CACR_ADDR, length: 8, dev: ka}
    ]);
    let cdg = new CDGVAX(ka);
    bus.addCdg(cdg);
    return {bus, ka, cdg};
}

/** The ONE machine.  See the file header: building one per case OOM-killed the box. */
let MACHINE = null;

/**
 * machine()
 *
 * @returns {Object} {bus, ka, cdg}
 */
function machine()
{
    if (!MACHINE) MACHINE = makeMachine();
    return MACHINE;
}

/**
 * beginPass()
 *
 * Puts the shared machine back into the state SIMH's PROCESS starts in -- a zeroed cache
 * diagnostic store and a zeroed CACR -- so that repeated grading passes (--selfcheck runs one per
 * mutation) are identical to the first.  Per-CASE state is runCaseJS()'s business; see the file
 * header for why the store is pass-scoped and the CACR is case-scoped.
 */
function beginPass()
{
    let m = machine();
    m.cdg.dat.fill(0);
    m.ka.reset();
}

/**
 * runCaseJS(ops)
 *
 * Executes one case's op list against the shared machine and returns the observed values, in op
 * order, for every "rl".
 *
 * `ka.reset()` reproduces SIMH's per-case `reset -p all`, which zeroes `ka_cacr` via sysd_powerup().
 * The CDG store is deliberately NOT cleared here: `cdg_dat` is a plain static array that no SIMH
 * reset routine touches, so it carries across cases on the oracle and must carry across them here.
 *
 * @param {Array.<Object>} ops
 * @returns {Object} {values: Array.<number>, fault: ?string}
 */
function runCaseJS(ops)
{
    let m = machine();
    m.ka.reset();
    let values = [], fault = null;
    for (let op of ops) {
        m.bus.checkFault();
        switch (op.kind) {
        case "wl": m.bus.setLong(op.addr, op.val | 0); break;
        case "ww": m.bus.setWord(op.addr, op.val & 0xFFFF); break;
        case "wb": m.bus.setByte(op.addr, op.val & 0xFF); break;
        case "rl": values.push(m.bus.getLong(op.addr) >>> 0); break;
        }
        if (m.bus.checkFault() && !fault) {
            fault = `${op.kind}@0x${hex(op.addr)}`;
        }
    }
    sampleHeap();
    return {values, fault};
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side                                                                                 *
 * ------------------------------------------------------------------------------------------- */

const MARK = "CDGCASE";

/**
 * runCasesSimh(simh, opts, cases)
 *
 * Batched into ONE SIMH invocation (mchkdiff.js's runBatch() convention).  Every case is made
 * independent by `reset -p all` -- see the file header for why the `-p` is load-bearing.
 *
 * SCBB is pointed at a handler far from the code, and PC is read back after every case, so "did
 * this machine check" is a DIRECTLY OBSERVED fact rather than an assumption: a case whose final PC
 * is R_HANDLER faulted, and BACKED is the phase that grades that.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {Array.<Object>} cases each {name, ops}
 * @returns {Array.<Object>} parallel to `cases`: {values: Array.<number>, pc: number, reached}
 */
function runCasesSimh(simh, opts, cases)
{
    let lines = ["set cpu " + (MEMSIZE >> 20) + "m", "set cpu simhalt"];
    for (let i = 0; i < cases.length; i++) {
        lines.push(`echo ${MARK}${i}`, "reset -p all");
        lines.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit -l ${hex(R_SCBB + 4)} ${hex(R_HANDLER)}`,
                   `deposit KSP ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`);
        let bytes = [], nSteps = 0;
        for (let op of cases[i].ops) {
            bytes.push(...encodeOp(op));
            nSteps++;
        }
        for (let k = 0; k < bytes.length; k++) lines.push(`deposit -b ${hex(CODE + k)} ${bytes[k].toString(16)}`);
        lines.push(`deposit PSL 0`, `deposit PC ${hex(CODE)}`, `step ${nSteps}`);
        for (let op of cases[i].ops) {
            if (op.kind == "rl") lines.push(`examine -h R${op.reg}`);
        }
        lines.push("examine -h PC");
    }
    lines.push("exit", "");
    let out = runSimh(simh, lines.join("\n"), path.join(opts.scratch, "cdgdiff-cases.ini"));

    /* Split on the markers, then read each chunk's register lines IN ORDER.  A case whose chunk is
       missing, or short a readback, is reported BY NAME (standing rule 6) rather than skipped. */
    let results = new Array(cases.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = +parts[i];
        let chunk = parts[i + 1] || "";
        let regs = [...chunk.matchAll(/^R(\d+):\s*([0-9A-Fa-f]+)/gm)].map((m) => parseInt(m[2], 16) >>> 0);
        let pcm = /^PC:\s*([0-9A-Fa-f]+)/m.exec(chunk);
        results[idx] = {values: regs, pc: pcm ? (parseInt(pcm[1], 16) >>> 0) : -1, reached: !!pcm};
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

/**
 * buildCases(seed, nAlias, nCacr)
 *
 * Returns {cases, meta} where `meta` carries what each phase needs to grade and to count coverage.
 * Every address is derived from CDASIZE / CDG_LEN / CDG_ROWS, never hand-listed (standing rule 5).
 *
 * @param {number} seed
 * @param {number} nAlias
 * @param {number} nCacr
 * @returns {Object}
 */
function buildCases(seed, nAlias, nCacr)
{
    let rnd = mulberry32(seed);
    let cases = [];
    let meta = {alias: [], nonalias: [], cacr: [], merge: [], backed: []};
    let add = (name, ops, phase, extra) => {
        let idx = cases.length;
        cases.push({name, ops});
        meta[phase].push(Object.assign({idx, name, ops}, extra || {}));
    };

    /* ---------------- ALIAS ----------------
       A pair (a, b) is an ALIAS pair when rowOf(a) == rowOf(b), i.e. they agree on address bits
       15:2.  The tag (bits 25:16 of the offset) is what varies.  Write a distinctive longword
       through `a`, read it back through `b`.
       The FIRST read in each case is through `b`; a second read through `a` proves the store is one
       location rather than two that happen to hold the same value at this instant. */
    let tags = [1, 2, 3, 0x10, 0x3F, 0x100, 0x2AA, CDG_LEN / CDASIZE - 1];   // deterministic
    let aliasOffsets = [0x0000, 0x0004, 0x1234 & ~3, 0x7FFC, 0xFFFC, 0x0100, 0x5550, 0xAAA8];
    for (let i = 0; i < tags.length; i++) {
        let off = aliasOffsets[i % aliasOffsets.length];
        let a = (CDG_BASE + off) >>> 0;
        let b = (CDG_BASE + tags[i] * CDASIZE + off) >>> 0;
        let val = (0xA5A50000 + i * 0x1111) >>> 0;
        add(`ALIAS a=0x${hex(a)} b=0x${hex(b)} (tag ${tags[i]})`,
            [{kind: "wl", addr: a, val}, {kind: "rl", addr: b, reg: 0}, {kind: "rl", addr: a, reg: 1}],
            "alias", {val, aliased: true});
    }
    for (let i = 0; i < nAlias; i++) {
        let off = (Math.floor(rnd() * CDG_ROWS) << 2) >>> 0;
        let tag = 1 + Math.floor(rnd() * (CDG_LEN / CDASIZE - 1));
        let a = (CDG_BASE + off) >>> 0;
        let b = (CDG_BASE + tag * CDASIZE + off) >>> 0;
        let val = (rnd() * 0x100000000) >>> 0;
        add(`ALIAS-RANDOM a=0x${hex(a)} b=0x${hex(b)}`,
            [{kind: "wl", addr: a, val}, {kind: "rl", addr: b, reg: 0}, {kind: "rl", addr: a, reg: 1}],
            "alias", {val, aliased: true});
    }
    /* NON-alias pairs: same tag, DIFFERENT row -- and different tag AND different row.  Without
       these, a row function that collapsed everything to row 0 would satisfy the ALIAS pairs. */
    let nonPairs = [
        [0x0000, 0x0004], [0x0000, 0x1000], [0x0004, 0xFFFC], [0x8000, 0x8004],
        [0x0000, CDASIZE + 0x0004], [0x0FFC, 3 * CDASIZE + 0x1000]
    ];
    for (let i = 0; i < nonPairs.length; i++) {
        let a = (CDG_BASE + nonPairs[i][0]) >>> 0;
        let b = (CDG_BASE + nonPairs[i][1]) >>> 0;
        let vA = (0x5A5A0000 + i * 0x1111) >>> 0, vB = (0x0F0F0000 + i * 0x2222) >>> 0;
        add(`NONALIAS a=0x${hex(a)} b=0x${hex(b)}`,
            [{kind: "wl", addr: a, val: vA}, {kind: "wl", addr: b, val: vB},
             {kind: "rl", addr: a, reg: 0}, {kind: "rl", addr: b, reg: 1}],
            "nonalias", {vA, vB});
    }

    /* ---------------- CACR ----------------
       Sixteen deterministic longwords whose four bytes independently carry parity 0 or 1 (0x00 has
       even population, 0x01 odd), so each lane's SEED and each lane's BIT POSITION is separately
       observable; then random longwords; then the ACCUM cases, which read two DIFFERENT longwords
       in sequence and ask the oracle whether the parity bits accumulate. */
    for (let combo = 0; combo < CACR_LANE_COMBOS; combo++) {
        let val = 0;
        for (let lane = 0; lane < 4; lane++) if (combo & (1 << lane)) val |= 0x01 << (lane * 8);
        val = val >>> 0;
        add(`CACR-LANES val=0x${hex(val)}`,
            [{kind: "wl", addr: CDG_BASE, val}, {kind: "rl", addr: CDG_BASE, reg: 0},
             {kind: "rl", addr: CACR_ADDR, reg: 1}],
            "cacr", {val, kind: "lanes"});
    }
    for (let i = 0; i < nCacr; i++) {
        let val = (rnd() * 0x100000000) >>> 0;
        add(`CACR-RANDOM val=0x${hex(val)}`,
            [{kind: "wl", addr: CDG_BASE, val}, {kind: "rl", addr: CDG_BASE, reg: 0},
             {kind: "rl", addr: CACR_ADDR, reg: 1}],
            "cacr", {val, kind: "random"});
    }
    let accumPairs = [[0x00000000, 0x01010101], [0x01010101, 0x00000000],
                      [0xFFFFFFFF, 0x00000000], [0x00010000, 0x00000100]];
    for (let p of accumPairs) {
        let a = p[0] >>> 0, b = p[1] >>> 0;
        add(`CACR-ACCUM a=0x${hex(a)} b=0x${hex(b)}`,
            [{kind: "wl", addr: CDG_BASE, val: a}, {kind: "rl", addr: CDG_BASE, reg: 0},
             {kind: "wl", addr: CDG_BASE + 4, val: b}, {kind: "rl", addr: CDG_BASE + 4, reg: 1},
             {kind: "rl", addr: CACR_ADDR, reg: 2}],
            "cacr", {val: b, first: a, kind: "accum"});
    }

    /* ---------------- MERGE ----------------
       Preload a longword, write a sub-longword quantity at a lane, read the whole longword back.
       Byte writes at all four lanes; word writes at the two ALIGNED lanes (see the file header).
       Three preload/value combinations per lane so a merge that happens to be a no-op for one
       value is not mistaken for a correct merge. */
    let preloads = [0x00000000, 0xFFFFFFFF, 0x89ABCDEF];
    let byteVals = [0x00, 0xFF, 0x5A];
    for (let lane = 0; lane < 4; lane++) {
        for (let k = 0; k < preloads.length; k++) {
            let addr = (CDG_BASE + 0x40 + lane) >>> 0;
            add(`MERGE-BYTE lane=${lane} pre=0x${hex(preloads[k])} val=0x${hex(byteVals[k], 2)}`,
                [{kind: "wl", addr: CDG_BASE + 0x40, val: preloads[k]},
                 {kind: "wb", addr, val: byteVals[k]},
                 {kind: "rl", addr: CDG_BASE + 0x40, reg: 0}],
                "merge", {lane, width: 1});
        }
    }
    let wordVals = [0x0000, 0xFFFF, 0x1234];
    for (let lane of [0, 2]) {
        for (let k = 0; k < preloads.length; k++) {
            let addr = (CDG_BASE + 0x80 + lane) >>> 0;
            add(`MERGE-WORD lane=${lane} pre=0x${hex(preloads[k])} val=0x${hex(wordVals[k], 4)}`,
                [{kind: "wl", addr: CDG_BASE + 0x80, val: preloads[k]},
                 {kind: "ww", addr, val: wordVals[k]},
                 {kind: "rl", addr: CDG_BASE + 0x80, reg: 0}],
                "merge", {lane, width: 2});
        }
    }
    for (let k = 0; k < preloads.length; k++) {
        add(`MERGE-LONG pre=0x${hex(preloads[k])}`,
            [{kind: "wl", addr: CDG_BASE + 0xC0, val: preloads[k]},
             {kind: "wl", addr: CDG_BASE + 0xC0, val: 0x0BADF00D},
             {kind: "rl", addr: CDG_BASE + 0xC0, reg: 0}],
            "merge", {lane: 0, width: 4});
    }

    /* ---------------- BACKED ----------------
       End to end: the first longword, the last longword, the last BYTE, both ends of several 64KB
       alias windows, and the fractional points across the whole 64MB range.  Each case writes AND
       reads, so a fault in either direction lands on the handler and is observed through PC. */
    let backedOffsets = new Set([0, 4, 0x100, CDASIZE - 4, CDASIZE, CDASIZE + 4,
                                 CDG_LEN - 4, CDG_LEN - 8, CDG_LEN - CDASIZE]);
    for (let k = 1; k <= 10; k++) backedOffsets.add(Math.floor(CDG_LEN * k / 11) & ~3);
    for (let k = 1; k <= 10; k++) backedOffsets.add((k * 0x1234567) % (CDG_LEN - 4) & ~3);
    for (let off of [...backedOffsets].sort((x, y) => x - y)) {
        let addr = (CDG_BASE + off) >>> 0;
        add(`BACKED 0x${hex(addr)}`,
            [{kind: "wl", addr, val: (0xDEC00000 + (off & 0xFFFF)) >>> 0}, {kind: "rl", addr, reg: 0}],
            "backed", {addr, off});
    }

    return {cases, meta};
}

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * grade(cases, meta, simhResults)
 *
 * Compares the JS machine against the captured SIMH results, case by case, and enforces every
 * phase's own structural claim on top of plain equality.  `simhResults` is captured ONCE and
 * reused across --selfcheck mutations: the oracle does not change when this tree is broken.
 *
 * @param {Array.<Object>} cases
 * @param {Object} meta
 * @param {Array.<Object>} simhResults
 * @returns {Array.<string>} problems
 */
function grade(cases, meta, simhResults)
{
    let problems = [];
    beginPass();
    let js = cases.map((c) => runCaseJS(c.ops));

    /* Every case must have reached comparison, with the expected number of readbacks (rule 6). */
    for (let i = 0; i < cases.length; i++) {
        let nWant = cases[i].ops.filter((o) => o.kind == "rl").length;
        let sr = simhResults[i];
        if (!sr || !sr.reached) {
            problems.push(`NOT-REACHED: case ${i} "${cases[i].name}" produced no SIMH readback`);
            continue;
        }
        if (sr.values.length !== nWant) {
            problems.push(`NOT-REACHED: case ${i} "${cases[i].name}" -- SIMH returned ` +
                `${sr.values.length} register readbacks, expected ${nWant}`);
            continue;
        }
        if (js[i].values.length !== nWant) {
            problems.push(`NOT-REACHED: case ${i} "${cases[i].name}" -- JS produced ` +
                `${js[i].values.length} readbacks, expected ${nWant}`);
            continue;
        }
        for (let k = 0; k < nWant; k++) {
            if (sr.values[k] !== js[i].values[k]) {
                problems.push(`MISMATCH: ${cases[i].name} -- readback ${k}: SIMH=0x${hex(sr.values[k])} ` +
                    `JS=0x${hex(js[i].values[k])}`);
            }
        }
    }

    /* ALIAS: the oracle must actually SHOW the alias, or the phase is vacuous. */
    for (let c of meta.alias) {
        let sr = simhResults[c.idx];
        if (!sr || sr.values.length < 2) continue;                  // already reported above
        if (sr.values[0] !== (c.val >>> 0) || sr.values[1] !== (c.val >>> 0)) {
            problems.push(`ALIAS-PREMISE: ${c.name} -- the ORACLE did not alias: wrote 0x${hex(c.val)}, ` +
                `read back 0x${hex(sr.values[0])} / 0x${hex(sr.values[1])}.  The 64KB row mask ` +
                `premise this phase is built on is wrong; re-read vaxmod_defs.h's CDG_GETROW.`);
        }
    }
    /* NON-ALIAS: the oracle must show them as DISTINCT, or a collapsed row function would pass. */
    for (let c of meta.nonalias) {
        let sr = simhResults[c.idx];
        if (!sr || sr.values.length < 2) continue;
        if (sr.values[0] !== (c.vA >>> 0) || sr.values[1] !== (c.vB >>> 0)) {
            problems.push(`NONALIAS-PREMISE: ${c.name} -- the ORACLE aliased two addresses that ` +
                `differ in bits 15:2: wrote 0x${hex(c.vA)}/0x${hex(c.vB)}, read 0x${hex(sr.values[0])}/` +
                `0x${hex(sr.values[1])}`);
        }
    }

    /* CACR: the live check that keeps the file header's "CACR_DRO is inert" claim honest. */
    for (let c of meta.cacr) {
        let sr = simhResults[c.idx];
        if (!sr || !sr.values.length) continue;
        let cacr = sr.values[sr.values.length - 1] >>> 0;
        if (cacr & CACR_DRO) {
            problems.push(`CACR-DRO-PREMISE: ${c.name} -- the ORACLE returned CACR=0x${hex(cacr)}, ` +
                `which has a CACR_DRO bit (0x${hex(CACR_DRO)}) SET.  This file's header states that ` +
                `nothing on either engine ever sets those bits, and --selfcheck's mutation set is ` +
                `chosen on that basis; the statement is now false and both must be revisited.`);
        }
        if (!(cacr & ~CACR_DRO & 0xFF000000)) {
            /* Not an error by itself (a longword can legitimately produce all-zero parity), but a
               phase where NO case ever set a parity bit would be vacuous -- counted below. */
        }
    }

    /* BACKED: neither engine may machine-check anywhere in the range. */
    for (let c of meta.backed) {
        let sr = simhResults[c.idx];
        if (!sr || !sr.reached) continue;
        if (sr.pc === R_HANDLER) {
            problems.push(`BACKED: SIMH MACHINE-CHECKED at 0x${hex(c.addr)} -- the range is not ` +
                `backed end to end after all; mchkdiff.js's calibration says otherwise`);
        }
        if (js[c.idx].fault) {
            problems.push(`BACKED: the JS bus FAULTED at 0x${hex(c.addr)} (${js[c.idx].fault})`);
        }
    }
    /* And no case anywhere may have faulted on the JS side unexpectedly. */
    for (let i = 0; i < cases.length; i++) {
        if (js[i].fault && simhResults[i] && simhResults[i].reached && simhResults[i].pc !== R_HANDLER) {
            problems.push(`FAULT-DIVERGENCE: ${cases[i].name} -- JS faulted (${js[i].fault}) where ` +
                `SIMH completed (PC=0x${hex(simhResults[i].pc)})`);
        }
    }

    return problems;
}

/**
 * coverage(meta, simhResults)
 *
 * Structural floors.  These FAIL the run and do NOT scale with the case count -- the randomized
 * phases can be shrunk to their floors and every requirement below still has to be met by the
 * DETERMINISTIC cases alone (standing rule 4).
 *
 * @param {Object} meta
 * @param {Array.<Object>} simhResults
 * @returns {Array.<string>} problems
 */
function coverage(meta, simhResults)
{
    let bad = [];
    let require = (cond, msg) => { if (!cond) bad.push("COVERAGE: " + msg); };

    let deterministicAlias = meta.alias.filter((c) => c.name.startsWith("ALIAS ")).length;
    require(deterministicAlias >= ALIAS_PAIRS_REQUIRED,
        `only ${deterministicAlias} deterministic alias pairs, need ${ALIAS_PAIRS_REQUIRED}`);
    require(meta.nonalias.length >= ALIAS_NONPAIRS_REQUIRED,
        `only ${meta.nonalias.length} non-alias pairs, need ${ALIAS_NONPAIRS_REQUIRED}`);
    /* The alias pairs must span many DISTINCT tags, or "aliasing" is only proved for one window. */
    let tagsSeen = new Set(meta.alias.map((c) => Math.floor((((c.ops[1].addr >>> 0) - CDG_BASE)) / CDASIZE)));
    require(tagsSeen.size >= ALIAS_PAIRS_REQUIRED,
        `alias pairs cover only ${tagsSeen.size} distinct 64KB windows, need ${ALIAS_PAIRS_REQUIRED}`);

    let lanes = meta.cacr.filter((c) => c.kind == "lanes");
    require(lanes.length === CACR_LANE_COMBOS,
        `${lanes.length} per-lane CACR combinations, need exactly ${CACR_LANE_COMBOS}`);
    require(meta.cacr.filter((c) => c.kind == "accum").length >= CACR_ACCUM_REQUIRED,
        `too few CACR accumulation cases`);
    /* Each of the four diagnostic-parity bits must be observed BOTH set and clear on the ORACLE,
       or a lane could be permanently stuck and this phase would never notice. */
    for (let bit = 0; bit < 4; bit++) {
        let sawSet = false, sawClear = false;
        for (let c of meta.cacr) {
            let sr = simhResults[c.idx];
            if (!sr || !sr.values.length) continue;
            let cacr = sr.values[sr.values.length - 1] >>> 0;
            if (cacr & (1 << (24 + bit))) sawSet = true; else sawClear = true;
        }
        require(sawSet, `oracle never returned CACR bit ${24 + bit} SET across any case`);
        require(sawClear, `oracle never returned CACR bit ${24 + bit} CLEAR across any case`);
    }

    let byteLanes = new Set(meta.merge.filter((c) => c.width == 1).map((c) => c.lane));
    let wordLanes = new Set(meta.merge.filter((c) => c.width == 2).map((c) => c.lane));
    require(byteLanes.size === MERGE_BYTE_LANES, `byte-write merge covers ${byteLanes.size} lanes, need ${MERGE_BYTE_LANES}`);
    require(wordLanes.size === MERGE_WORD_LANES, `word-write merge covers ${wordLanes.size} lanes, need ${MERGE_WORD_LANES}`);
    require(meta.merge.filter((c) => c.width == 4).length >= 3, `too few longword-write merge cases`);

    require(meta.backed.length >= BACKED_ADDRS_REQUIRED,
        `only ${meta.backed.length} end-to-end backing addresses, need ${BACKED_ADDRS_REQUIRED}`);
    let windows = new Set(meta.backed.map((c) => Math.floor(c.off / CDASIZE)));
    require(windows.size >= BACKED_WINDOWS_REQUIRED,
        `backing addresses cover only ${windows.size} distinct 64KB windows, need ${BACKED_WINDOWS_REQUIRED}`);
    require(meta.backed.some((c) => c.off === 0), `no backing case at CDG_BASE itself`);
    require(meta.backed.some((c) => c.off === CDG_LEN - 4), `no backing case at the LAST longword of the range`);

    /*
     * RESOURCE.  An ABSOLUTE bound, not one that scales with the case count: adding cases must cost
     * no memory, because the machine is built once.  This is the assertion that keeps the OOM
     * regression (8.6GB RSS, kernel OOM killer, three times) from coming back silently.
     */
    require(PEAK_HEAP <= MAX_HEAP_BYTES,
        `peak heap ${(PEAK_HEAP / (1024 * 1024)).toFixed(1)}MB exceeds the ${MAX_HEAP_BYTES / (1024 * 1024)}MB ` +
        `bound -- something is allocating per case again (see the file header and bus.js's addCdg())`);

    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * Self-check -- deliberate defects injected into the SHIPPED code path                          *
 * ------------------------------------------------------------------------------------------- */

/**
 * MUTATIONS
 *
 * Standing rule 11: each entry PERTURBS the shipped cdg.js/ka655.js -- by composing over the real
 * method, or by changing the named data the real method reads -- and NEVER substitutes a private
 * copy of the code under test.  Each `apply` first ASSERTS the pre-mutation value it expects, and
 * throws if the shipped code is already in the mutated state: a mutation that is a no-op because
 * the defect is already live would otherwise print CAUGHT while proving nothing, which is the exact
 * failure mode rule 11 exists to stop.
 *
 * NOT PRESENT, deliberately: "CACR_DRO is not cleared on read".  CACR_DRO is bits 8-23; nothing on
 * either engine ever sets one (ka_wr writes bits 0-6, cdg_rd writes bits 24-27), so that mutation
 * is undetectable BY THE ORACLE ITSELF and would survive any correct differential.  The detectable
 * neighbour -- a DRO clear mask widened to swallow the parity bits too -- is DRO-MASK-WIDENED
 * below, and the premise is asserted live on every CACR case by grade().  See the file header.
 */
const MUTATIONS = {
    /**
     * The suggestion cdg.js's header exists to refute: treat the range as flat rather than aliased.
     * Composed over the shipped rowOf() so the base window still behaves, and only the TAG bits --
     * exactly what CDG_GETROW discards -- start to matter.
     */
    "ALIAS-DROPPED": function() {
        let orig = CDGVAX.prototype.rowOf;
        let probe = new CDGVAX();
        if (orig.call(probe, CDG_BASE) !== orig.call(probe, CDG_BASE + CDASIZE)) {
            throw new Error("ALIAS-DROPPED precondition failed: rowOf() does not alias already");
        }
        CDGVAX.prototype.rowOf = function(addr) {
            return (orig.call(this, addr) ^ (((addr >>> 16) & 0x3FF) << 2)) % CDG_ROWS;
        };
        return () => { CDGVAX.prototype.rowOf = orig; };
    },
    /**
     * The degenerate opposite: every address collapses to row 0.  This one PASSES an alias-only
     * test, which is why the NONALIAS pairs exist.
     */
    "ALIAS-COLLAPSED": function() {
        let orig = CDGVAX.prototype.rowOf;
        let probe = new CDGVAX();
        if (orig.call(probe, CDG_BASE) === orig.call(probe, CDG_BASE + 4)) {
            throw new Error("ALIAS-COLLAPSED precondition failed: rowOf() is already collapsed");
        }
        CDGVAX.prototype.rowOf = function(addr) { return 0; };
        return () => { CDGVAX.prototype.rowOf = orig; };
    },
    /** cdg_rd()'s ALTERNATING parity seed, inverted on ONE lane. */
    "CACR-SEED-INVERTED": function() {
        let orig = KA655VAX.CDG_DPAR_SEEDS.slice();
        if (orig.join() !== "0,1,0,1") {
            throw new Error(`CACR-SEED-INVERTED precondition failed: seeds are [${orig}], expected [0,1,0,1]`);
        }
        KA655VAX.CDG_DPAR_SEEDS = [0, 0, 0, 1];
        return () => { KA655VAX.CDG_DPAR_SEEDS = orig; };
    },
    /** Two of the four parity bits land in each other's CACR bit positions. */
    "CACR-BITS-SWAPPED": function() {
        let orig = KA655VAX.CDG_DPAR_SHIFTS.slice();
        if (orig.join() !== "0,1,2,3") {
            throw new Error(`CACR-BITS-SWAPPED precondition failed: shifts are [${orig}], expected [0,1,2,3]`);
        }
        KA655VAX.CDG_DPAR_SHIFTS = [0, 2, 1, 3];
        return () => { KA655VAX.CDG_DPAR_SHIFTS = orig; };
    },
    /**
     * The transcription error a careful reader is MOST likely to make here: assuming CACR_DRO
     * covers the diagnostic-parity bits it precedes, and therefore clearing them on every read.
     * It does not (see the file header) -- the bits accumulate -- so this is observable, and the
     * CACR-ACCUM cases are what observe it.
     */
    "CACR-DRO-MASK-WIDENED": function() {
        let orig = KA655VAX.CDG_DRO_CLEAR_MASK;
        if ((orig | 0) !== (~CACR_DRO | 0)) {
            throw new Error(`CACR-DRO-MASK-WIDENED precondition failed: mask is 0x${hex(orig)}, ` +
                `expected 0x${hex(~CACR_DRO)}`);
        }
        KA655VAX.CDG_DRO_CLEAR_MASK = ~(CACR_DRO | (0x0F << 24));
        return () => { KA655VAX.CDG_DRO_CLEAR_MASK = orig; };
    },
    /**
     * cdg_wr()'s `if (lnt < L_LONG)` gate never fires, so a byte write stores its byte as a whole
     * longword and clobbers the other three lanes.
     */
    "MERGE-DROPPED": function() {
        let orig = CDGVAX.L_LONG;
        if (orig !== 4) throw new Error(`MERGE-DROPPED precondition failed: L_LONG is ${orig}, expected 4`);
        CDGVAX.L_LONG = 0;
        return () => { CDGVAX.L_LONG = orig; };
    },
    /** A byte write merges with the WORD mask, so it also clobbers the neighbouring lane. */
    "MERGE-BYTE-MASK-WIDENED": function() {
        let orig = CDGVAX.MERGE_MASKS.slice();
        if (orig[1] !== 0xFF) {
            throw new Error(`MERGE-BYTE-MASK-WIDENED precondition failed: byte mask is 0x${hex(orig[1], 2)}`);
        }
        CDGVAX.MERGE_MASKS = orig.slice();
        CDGVAX.MERGE_MASKS[1] = 0xFFFF;
        return () => { CDGVAX.MERGE_MASKS = orig; };
    }
};

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                          *
 * ------------------------------------------------------------------------------------------- */

function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, dflt) => {
        let i = argv.indexOf(name);
        return (i >= 0 && i + 1 < argv.length) ? argv[i + 1] : dflt;
    };
    let simh = findSimh(getArg("--simh", ""));
    let seed = +getArg("--seed", 0x0B7CDC0F) >>> 0;
    let nCacr = +getArg("--cacr-cases", 24);
    let nAlias = +getArg("--alias-cases", 16);
    let fSelfCheck = argv.includes("--selfcheck");
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "cdgdiff-"));
    let opts = {scratch};

    /* Every exit path -- an undersized-args FAIL, an uncaught exception from buildCases/
       runCasesSimh/grade, a --selfcheck failure, or a clean PASS -- runs through this
       try/finally, so scratch is always removed (HANDOFF.md pcjsvax-bd1: an earlier revision
       left scratch behind on the args-floor exit and on any thrown exception, because
       process.exit(1) and an unguarded throw both skip past the rmSync() that previously ran
       only on the success path). */
    try {
        let errors = [];
        if (nCacr < CACR_CASES_FLOOR) {
            errors.push(`--cacr-cases ${nCacr} is below the enforced floor of ${CACR_CASES_FLOOR}`);
        }
        if (nAlias < ALIAS_CASES_FLOOR) {
            errors.push(`--alias-cases ${nAlias} is below the enforced floor of ${ALIAS_CASES_FLOOR}`);
        }
        if (errors.length) {
            for (let e of errors) console.log("FAILED: " + e);
            process.exitCode = 1;
            return;
        }

        console.log("cdgdiff: SIMH = %s", simh);
        console.log("cdgdiff: seed = 0x%s, cacr-cases = %d, alias-cases = %d", hex(seed), nCacr, nAlias);

        let {cases, meta} = buildCases(seed, nAlias, nCacr);
        console.log("cdgdiff: %d cases (alias %d, nonalias %d, cacr %d, merge %d, backed %d)",
            cases.length, meta.alias.length, meta.nonalias.length, meta.cacr.length,
            meta.merge.length, meta.backed.length);

        let simhResults = runCasesSimh(simh, opts, cases);
        sampleHeap();

        let problems = grade(cases, meta, simhResults);
        problems.push(...coverage(meta, simhResults));

        console.log("\nALIAS  (%d pairs + %d non-pairs): %s", meta.alias.length, meta.nonalias.length,
            problems.some((p) => /ALIAS|NONALIAS/.test(p)) ? "FAIL" : "MATCH");
        console.log("CACR   (%d cases, all 4 lanes both seeds): %s", meta.cacr.length,
            problems.some((p) => /CACR/.test(p)) ? "FAIL" : "MATCH");
        console.log("MERGE  (%d cases, byte lanes 0-3 + aligned word lanes): %s", meta.merge.length,
            problems.some((p) => /MERGE/.test(p)) ? "FAIL" : "MATCH");
        console.log("BACKED (%d addresses across the 64MB range): %s", meta.backed.length,
            problems.some((p) => /BACKED/.test(p)) ? "FAIL" : "MATCH");

        if (fSelfCheck) {
            console.log("\nSelf-check: every mutation of the SHIPPED path must be caught.");
            for (let name of Object.keys(MUTATIONS)) {
                let undo = null, caught = false, why = "";
                try {
                    undo = MUTATIONS[name]();
                    let p = grade(cases, meta, simhResults);
                    caught = p.length > 0;
                    why = caught ? p[0].split("\n")[0].slice(0, 120) : "NO PROBLEMS REPORTED";
                } catch (e) {
                    caught = true;
                    why = "threw: " + e.message.split("\n")[0];
                    if (/precondition failed/.test(e.message)) {
                        caught = false;                 // a no-op mutation proves NOTHING -- see MUTATIONS
                    }
                } finally {
                    if (undo) undo();
                }
                console.log("  %s %s (%s)", name.padEnd(26), caught ? "CAUGHT" : "SURVIVED", why);
                if (!caught) problems.push(`SELFCHECK: mutation '${name}' was not caught -- ${why}`);
            }
            /* The un-mutated tree must still be clean after all that restoring. */
            let after = grade(cases, meta, simhResults);
            if (after.length) {
                problems.push(`SELFCHECK: the tree did not restore cleanly -- ${after[0]}`);
            }
        }

        console.log("\npeak heap: %sMB (bound %dMB)", (PEAK_HEAP / (1024 * 1024)).toFixed(1),
            MAX_HEAP_BYTES / (1024 * 1024));
        if (problems.length) {
            console.log("\nFAILED (%d):", problems.length);
            for (let p of problems.slice(0, 60)) console.log("  " + p);
            if (problems.length > 60) console.log("  ... and %d more", problems.length - 60);
            process.exitCode = 1;
            return;
        }
        console.log("\nPASS: CDG is indistinguishable from SIMH over %d cases (aliasing, the CACR " +
            "diagnostic-parity side effect, the byte/word write merge, and end-to-end backing).", cases.length);
    } finally {
        fs.rmSync(scratch, {recursive: true, force: true});
    }
}

if (process.argv[1] && path.resolve(process.argv[1]) == path.resolve(fileURLToPath(import.meta.url))) {
    main();
}

export { buildCases, runCaseJS, makeMachine, beginPass, MUTATIONS };
