/**
 * @fileoverview Differential test: the CQM Qbus MEMORY window's MAPPED read/write path (through a
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
 *               VALID scatter-gather map entry) -- vs. a real Open SIMH microvax3900
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-55d.  pcjsvax-5c1 implemented cqm_rd()/cqm_wr() (cqbic.js's CQMVAX) and the ROM now
 * reaches `>>>` with self-test 80 passing, but nothing in the gate GRADES the success path: it is
 * proven only by the ROM booting, which is end-to-end evidence, not a differential.
 * tests/cqmerrdiff.js's own makeMachine() is bus+cpu ONLY -- no CQBIC, no CQMAP, no CQM -- and it
 * deliberately zeroes the whole map region so every entry reads VLD-clear; it therefore cannot
 * reach the mapped path at all. tests/qdmadiff.js grades the map and the DMA routines, never the
 * CPU-side window. This file is the missing piece: it programs the map with REAL CPU instructions
 * through cqbic.js's CQMAPVAX (never poking it directly), and then reads/writes CQMBASE+offset
 * with REAL CPU instructions too, grading the transferred value, the backing memory, DSER/MEAR/
 * SEAR, ssc_bto and mem_err against the live oracle -- pcjsvax-aa5's done-condition 4.
 *
 * DOES NOT TOUCH tests/cqmerrdiff.js. That file's master-NXM (map-zeroed) coverage is untouched;
 * this is a wholly separate file with its own machine, so nothing here can weaken it.
 *
 * WHAT IS GRADED, IN THE ITEM'S OWN PRIORITY ORDER
 * --------------------------------------------------
 *   1. MAPPED READ and MAPPED WRITE through a VALID entry pointing at real memory -- the transferred
 *      value (register for reads, backing memory for writes), the backing page, DSER/MEAR/SEAR,
 *      ssc_bto and mem_err.
 *   2. THE LONGWORD TWO-CYCLE SPLIT. cqbic.js's CQMVAX.readLong()/writeLong() call read()/write()
 *      TWICE (offset and offset+2), exactly ReadIO()/WriteIO()'s `(ReadQb(pa+2)<<16)|ReadQb(pa)`
 *      (vax_io.c:262) -- 5c1's first cut got this wrong (returned a machine check for the longword
 *      case) and only the ROM caught it, so it gets a case here, not just a comment.
 *   3. THE PAGE-STRADDLING CASE, the sharp edge of (2): a longword at offset 0x1FE of a 512-byte
 *      Qbus page takes its HIGH half from the NEXT map entry (idx+1). Three READ shapes (next entry
 *      valid-elsewhere / invalid / valid-but-out-of-memory) and the WRITE mirror of each -- writeLong()
 *      attempts BOTH halves unconditionally (write() never throws), so a low-half failure does NOT
 *      suppress the high-half attempt, while readLong() SHORT-CIRCUITS on the low half's failure and
 *      never even consults the second entry (see readLong()'s own `if (lo === REG_MCHK) return
 *      REG_MCHK` before the second read() call) -- both shapes are graded, not assumed.
 *   4. THE cq_serr PATH (a VALID entry pointing OUTSIDE memory -> DSER<SNX>+SEAR, distinct from
 *      cq_merr's DSER<MNX>+MEAR). MEASURED at ZERO occurrences over pcjsvax-aa5's 6M-instruction ROM
 *      walk (same conclusion pcjsvax-ee7 reached about cqmap_rd's out-of-memory READ branch), so the
 *      ROM will never reach it. CONSTRUCTED here, explicitly, by programming a VALID entry whose page
 *      lies past MEMSIZE -- byte/word/long, both directions (the cq-serr-* enumerated cases below).
 *   5. BYTE AND WORD ACCESS AT EVERY OFFSET ALIGNMENT within a page, including 0, near both page
 *      edges (0x1FC-0x1FF), and the odd offsets that may cross a page boundary at the WORD level
 *      too. cqm_rd() takes its half-select shift from `pa` (the untranslated address); cqm_wr() takes
 *      it from `ma` (the TRANSLATED one) -- cqbic.js's own file header preserves this because it is
 *      the C's own asymmetry, not a simplification. MEASURED HERE (see "A MEASURED, DISCLOSED
 *      NON-FINDING" below): for THIS window's geometry the two are PROVABLY equal on every
 *      successful translation (both derive their low 9 bits from the same pre-translation Qbus
 *      offset), so no case -- exhaustive or random -- can distinguish them. The asymmetry is
 *      preserved in the source and in this file's mutation list for documentary completeness, and
 *      the fact that it cannot be observed is disclosed rather than concealed with a rigged pass.
 *
 * TWO MEASURED, DISCLOSED NON-FINDINGS (standing rule 6 and rule 12 -- never claim a test does more
 * than it does)
 * ------------------------------------------------------------------------------------------------
 * The item names four mutations this file's --selfcheck must catch. Two of the four are implemented
 * below exactly as specified and are CAUGHT (map entry VLD bit ignored; CQMVAX returning null instead
 * of REG_MCHK). The other two are implemented exactly as specified too, but were MEASURED -- by
 * direct instrumentation of CQMVAX.readLong(), not by inspection -- to be UNREACHABLE through any
 * legitimate CPU-instruction access, for a reason neither this file's author nor (evidently) the
 * item anticipated: mmu.js's readData()/writeData() gate EVERY longword reference on physical
 * alignment (`(pa & 3) == 0`) BEFORE ever reaching a device. An ALIGNED reference calls
 * device.readLong(pa)/writeLong(pa) ONCE, with that exact `pa`. An UNALIGNED one (which is how the
 * "offset 0x1FE" straddle in item priority 3 is actually reached -- 0x1FE mod 4 == 2) is decomposed
 * by mmu.js's OWN generic readU()/writeU() stitch into TWO calls to readLong()/writeLong(), each at
 * a 4-byte-ALIGNED address (`pa & ~3` and `(pa + 4) & ~3`) -- confirmed by instrumenting
 * CQMVAX.readLong() while running this file's own "straddle-read-both-valid-discontiguous" case: it
 * is called with 0x...69FC and 0x...6A00, never with the raw 0x...69FE the instruction encodes.
 *
 * Consequence: CQMVAX.readLong()/writeLong()'s OWN internal two-`read()`/two-`write()` split (offset
 * and offset+2) is NEVER invoked with an `addr` whose own `addr+2` crosses a 512-byte page -- because
 * `addr` is always 4-aligned when this class is called, and the largest aligned offset inside a page
 * (0x1FC) plus 2 is 0x1FE, still inside the SAME page. So a mutation that skips the second
 * translation and reuses the first call's `mapMA` for both halves (mutation 1, "composed from ONE
 * backing longword") produces the IDENTICAL result to the correct code on every reachable input,
 * because both halves of any one call are always the same page's own translation anyway. This is the
 * SAME shape of finding as mutation 2's (`pa`/`ma` shift): the item's description is correct about
 * the ARCHITECTURAL requirement (a 16-bit Qbus genuinely needs two cycles, and a naive one-longword
 * read is wrong on real hardware), and the PAGE-STRADDLE EFFECT ITSELF is real and IS graded here
 * (see the straddle-* enumerated cases, priority 3) -- it is produced by mmu.js's OWN unaligned
 * stitch calling this class TWICE at different aligned addresses, not by this class's own single-call
 * internal split ever itself spanning two pages. Both mutations are kept, run, and reported by name
 * in main()'s selfcheck output (search for "NON-FINDING") rather than silently marked CAUGHT or
 * quietly dropped from the suite.
 *
 * NOT GRADED HERE, DELIBERATELY
 * -------------------------------
 *   - The machine-check FRAME shape (old PC/PSL, mcheck parameters). cqm_rd()'s failure path routes
 *     through the SAME onBusFault()/intexc() mechanism cqmerrdiff.js already grades byte-for-byte
 *     for the unbacked-reference case; what is NEW here is the TRANSLATION outcome (mapAddr/cqSerr/
 *     cqMerr), which DSER/MEAR/SEAR/PC/PSL/bto/mem_err are graded against. Re-deriving the frame here
 *     would duplicate cqmerrdiff.js's own scope rather than extend it.
 *   - MBR itself pointing out of memory. That interaction (a map-register WRITE or the DMA path
 *     failing its entry fetch because the MAP ITSELF is unreachable) is qdmadiff.js's territory
 *     (its `mbr-out-of-memory-*` cases); this file always uses one fixed, valid MBR.
 *   - Deferred mem_err DELIVERY (IPL masking, vector dispatch). cqmerrdiff.js's
 *     verifyDeferredDelivery() already grades that mechanism generically over the whole Qbus/CQBIC
 *     unbacked path; this file asserts mem_err gets SET, not how it is later delivered.
 *
 *      node machines/dec/vax/tests/cqmmapdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_BIN, else the scratch build)
 *        --cases N         randomized cases (default 240; below MIN_CASES_FLOOR the run FAILS --
 *                           it does not silently clamp up)
 *        --seed S          PRNG seed, printed on failure so a run is reproducible
 *        --selfcheck       prove the differential detects deliberate defects
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
import CPUStateVAX, { VAXStop } from "../modules/v2/cpustate.js";
import CQBICVAX, { CQMAPVAX, CQMVAX, CQMAP_BASE, CQMAPSIZE, CQMAP_VLD, CQMAP_PAG, VA_M_OFF,
    CQBIC_BASE, REG_MBR } from "../modules/v2/cqbic.js";
import { SCB } from "../modules/v2/exc.js";
/* REG_MCHK is used only by the file header's disclosed --selfcheck mutation 3 (a bug that returns
   `null` in its place); imported here at the top like everything else, not at point of use. */
import { REG_MCHK } from "../modules/v2/regblock.js";
import { findSimh } from "./mchkdiff.js";

/* ------------------------------------------------------------------------------------------- *
 * Small utilities -- same PRNG/hex/pick as every other VAX differential                          *
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
function rint(rnd, n) { return Math.floor(rnd() * n) % n; }

function runSimh(bin, script, iniPath)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 20 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * Physical layout                                                                                *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE    = 0x01000000;                 // 16MB, the SIMH microvax3900 default
const PAGE       = 512;                         // VA_PAGSIZE
const R_SCBB     = 0x00100000;
const R_MCHK_H   = 0x00102000;                  // SCBB+SCB.MCHK's target -- a faulted probe lands here
const R_CODE     = 0x00104000;
const R_CODE_LEN = 0x00000400;
const R_KSP      = 0x00110000;
const R_IS       = 0x00118000;
const DATA_BASE  = 0x00200000;                  // backing pages a mapped access can touch
const DATA_LEN   = 0x00020000;                  // 256 pages
const MBR        = 0x00300000;                  // the ONE fixed, valid MBR this file uses
const MAP_SPAN   = [MBR, (MBR + CQMAPSIZE) >>> 0];
const OOM_PAGE   = (MEMSIZE / PAGE) + 0x40;      // a map target page past the end of memory
const DATA_PAGE0 = DATA_BASE / PAGE;
const DATA_NPAGE = DATA_LEN / PAGE;
const CQM_BASE   = VAX.PHYSMEM.CQM_BASE >>> 0;
const CQM_LENGTH = VAX.PHYSMEM.CQM_LENGTH >>> 0;
const CQBIC_MBR_ADDR = (CQBIC_BASE + REG_MBR * 4) >>> 0;
const PSL_RUN    = 0x041F0000;                  // kernel, IS, IPL 31 -- masks every interrupt source
const SENTINEL   = 0xDEADBEEF | 0;              // R0's pre-load; a faulted read must leave it alone

function assertLayout()
{
    let regions = [
        ["CODE", R_CODE, R_CODE + R_CODE_LEN],
        ["DATA", DATA_BASE, DATA_BASE + DATA_LEN],
        ["MAP", MAP_SPAN[0], MAP_SPAN[1]]
    ];
    for (let i = 0; i < regions.length; i++) {
        let [n, lo, hi] = regions[i];
        if (hi > MEMSIZE) throw new Error(`cqmmapdiff.js: region ${n} ends past MEMSIZE`);
        for (let j = i + 1; j < regions.length; j++) {
            let [n2, lo2, hi2] = regions[j];
            if (lo < hi2 && lo2 < hi) throw new Error(`cqmmapdiff.js: regions ${n} and ${n2} overlap`);
        }
    }
    if (OOM_PAGE * PAGE < MEMSIZE) throw new Error("cqmmapdiff.js: OOM_PAGE is inside memory");
    /* CQMAPSIZE (vaxmod_defs.h) is the map's BYTE span (0x8000); CQMAPSIZE/4 is the entry COUNT
       (8192 longword entries). CQM_LENGTH must be exactly that many PAGE-sized (512-byte) Qbus
       pages, or this file's 1:1 Qbus-index-to-map-entry assumption (no index folding needed, unlike
       qdmadiff.js's 22-bit-bus fold) no longer holds. */
    if (CQM_LENGTH !== (CQMAPSIZE / 4) * PAGE) {
        throw new Error(`cqmmapdiff.js: CQM_LENGTH (0x${hex(CQM_LENGTH)}) does not cover exactly ` +
            `CQMAPSIZE/4 (${CQMAPSIZE / 4}) entries of PAGE (${PAGE}) bytes -- the 1:1 Qbus-index-to-map-entry ` +
            `assumption this file relies on (no index folding needed, unlike qdmadiff.js's 22-bit bus) no longer holds`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test -- built ONCE (standing rule 14)                                        *
 * ------------------------------------------------------------------------------------------- */

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let cqbic = new CQBICVAX(cpu.exc, bus, MEMSIZE);
    bus.addRegBlock([
        {base: VAX.PHYSMEM.REG_BASE >>> 0, length: 0x14, dev: cqbic},
        {base: CQMAP_BASE, length: CQMAPSIZE, dev: new CQMAPVAX(cqbic)}
    ]);
    bus.addCqm([{base: CQM_BASE, length: CQM_LENGTH, dev: new CQMVAX(cqbic)}]);
    return {bus, cpu, cqbic};
}

/** Opcode numbers, resolved by mnemonic -- never hand-transcribed. */
function opcodeOf(name)
{
    let opc = OPCODES.indexOf(name);
    if (opc < 0) throw new Error(`cqmmapdiff.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
    return opc;
}
const OPC = {
    MOVL: opcodeOf("MOVL"), MOVW: opcodeOf("MOVW"), MOVB: opcodeOf("MOVB"),
    MOVZWL: opcodeOf("MOVZWL"), MOVZBL: opcodeOf("MOVZBL")
};

/* ------------------------------------------------------------------------------------------- *
 * Instruction builders -- same shape as qdmadiff.js's (absolute mode 9F, immediate 8F)            *
 * ------------------------------------------------------------------------------------------- */

function lw(a) { a = a >>> 0; return [a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF]; }

/** MOVL/MOVW/MOVB I^#val, @#addr */
function movImmAbs(size, val, addr)
{
    if (size === 4) return [OPC.MOVL, 0x8F, ...lw(val), 0x9F, ...lw(addr)];
    if (size === 2) return [OPC.MOVW, 0x8F, val & 0xFF, (val >>> 8) & 0xFF, 0x9F, ...lw(addr)];
    return [OPC.MOVB, 0x8F, val & 0xFF, 0x9F, ...lw(addr)];
}

/** MOVL/MOVZWL/MOVZBL @#addr, Rn -- all three zero-extend into a full longword register */
function movAbsReg(size, addr, rn)
{
    let opc = (size === 4) ? OPC.MOVL : (size === 2) ? OPC.MOVZWL : OPC.MOVZBL;
    return [opc, 0x9F, ...lw(addr), 0x50 | (rn & 0xF)];
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                              *
 * ------------------------------------------------------------------------------------------- */

/** A valid entry pointing at DATA page k (0-based within the DATA region). */
function dpage(k) { return DATA_PAGE0 + (k % DATA_NPAGE); }

let autoPage = 0;
function nextDataPage() { return dpage(autoPage++); }

/**
 * makeCase(spec)
 *
 * @param {Object} spec {name, op:'read'|'write', size:1|2|4, qa (Qbus offset within the CQM
 *   window), entries:[{idx,page,valid,extra}] (REQUIRED: caller states exactly the indices this
 *   case's access needs -- idx=qa>>9, and idx+1 too if the access straddles a page), writeVal
 *   (for op='write'), dserPre/mearPre/searPre (pre-existing error state)}
 * @returns {Object} case
 */
function makeCase(spec)
{
    let c = Object.assign({
        entries: [], dserPre: 0, mearPre: 0, searPre: 0, writeVal: 0
    }, spec);
    if (c.qa + c.size > CQM_LENGTH) throw new Error(`cqmmapdiff.js: case "${c.name}" runs past the CQM window`);
    let idx = c.qa >>> 9;
    let needsHi = ((c.qa & VA_M_OFF) + c.size) > (VA_M_OFF + 1);
    let neededIdx = new Set([idx]);
    if (needsHi) neededIdx.add((idx + 1) >>> 0);
    for (let i of neededIdx) {
        if (!c.entries.some((e) => e.idx === i)) {
            throw new Error(`cqmmapdiff.js: case "${c.name}" needs map entry ${i} but its spec does not supply one`);
        }
    }
    c.needsHi = needsHi;

    /* The instruction stream: MBR, then every entry write, then the probe. Entry order is fixed
       (ascending idx) so the SAME script is produced regardless of object key order. */
    let code = [], nSetup = 0;
    let emit = (bytes) => { for (let b of bytes) code.push(b & 0xFF); };
    emit(movImmAbs(4, MBR, CQBIC_MBR_ADDR)); nSetup++;
    let sortedEntries = c.entries.slice().sort((a, b) => a.idx - b.idx);
    for (let e of sortedEntries) {
        let val = (((e.valid ? CQMAP_VLD : 0) | e.page | (e.extra || 0)) | 0) >>> 0;
        emit(movImmAbs(4, val, (CQMAP_BASE + e.idx * 4) >>> 0));
        nSetup++;
    }
    let addr = (CQM_BASE + c.qa) >>> 0;
    if (c.op === "read") {
        emit(movAbsReg(c.size, addr, 0));
    } else {
        emit(movImmAbs(c.size, c.writeVal, addr));
    }
    if (code.length > R_CODE_LEN) throw new Error(`cqmmapdiff.js: case "${c.name}" code overruns R_CODE_LEN`);
    c.code = code;
    c.nSetup = nSetup;

    /* Every VALID, in-memory entry's whole page is dumped and compared -- byte for byte, not just
       the bytes the case intends to touch, so a translation that lands on the WRONG page (or a
       write that corrupts a neighbour lane) is caught even though nothing asked to look there. */
    let pages = [];
    for (let e of sortedEntries) {
        if (e.valid && e.page * PAGE < MEMSIZE && pages.indexOf(e.page * PAGE) < 0) pages.push(e.page * PAGE);
    }
    c.ranges = pages.map((p) => [p, p + PAGE - 4]);
    c.nMemVals = c.ranges.reduce((n, r) => n + ((r[1] - r[0]) / 4 + 1), 0);

    /* Coverage classification, derived from the SPEC alone -- never from what the code under test
       actually did (a floor that asked the code under test what it covered would certify itself). */
    c.hasInvalid = sortedEntries.some((e) => neededIdx.has(e.idx) && !e.valid);
    c.hasOom = sortedEntries.some((e) => neededIdx.has(e.idx) && e.valid && e.page * PAGE >= MEMSIZE);
    c.allGood = !c.hasInvalid && !c.hasOom;
    c.unaligned = (c.qa % c.size) !== 0;
    return c;
}

/* ------------------------------------------------------------------------------------------- *
 * The enumerated phase -- one named case per thing the item's priority list names                *
 * ------------------------------------------------------------------------------------------- */

function enumeratedCases()
{
    let out = [];
    let add = (spec) => out.push(makeCase(spec));
    let BASE = 0x20;                             // an arbitrary but fixed base map index

    /* -- (1) MAPPED READ/WRITE through a valid entry, every size, safely mid-page ------------- */
    for (let op of ["read", "write"]) {
        for (let size of [1, 2, 4]) {
            let idx = BASE + size;
            add({name: `mapped-${op}-size${size}`, op, size, qa: idx * PAGE + 0x40,
                 entries: [{idx, page: nextDataPage(), valid: true}], writeVal: 0xA5A5A5A5 ^ (size << 24)});
        }
    }

    /* -- (2) the longword split, explicit, non-straddling (both halves same page) -------------- */
    for (let op of ["read", "write"]) {
        let idx = BASE + 10;
        add({name: `longword-split-${op}`, op, size: 4, qa: idx * PAGE + 0x80,
             entries: [{idx, page: nextDataPage(), valid: true}], writeVal: 0x12345678});
    }

    /* -- (3) PAGE-STRADDLING longword at offset 0x1FE -- READ, three next-entry shapes --------- */
    {
        let idx = BASE + 20;
        add({name: "straddle-read-both-valid-discontiguous", op: "read", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: true}, {idx: idx + 1, page: nextDataPage(), valid: true}]});
        idx = BASE + 21;
        add({name: "straddle-read-hi-invalid", op: "read", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: true}, {idx: idx + 1, page: nextDataPage(), valid: false}]});
        idx = BASE + 22;
        add({name: "straddle-read-hi-oom", op: "read", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: true}, {idx: idx + 1, page: OOM_PAGE, valid: true}]});
        /* the LOW half itself failing must short-circuit -- the high entry is never even consulted */
        idx = BASE + 23;
        add({name: "straddle-read-lo-invalid-hi-valid", op: "read", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: false}, {idx: idx + 1, page: nextDataPage(), valid: true}]});
    }

    /* -- (3) PAGE-STRADDLING longword at offset 0x1FE -- WRITE, both halves attempted always ---- */
    {
        let idx = BASE + 30;
        add({name: "straddle-write-both-valid-discontiguous", op: "write", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: true}, {idx: idx + 1, page: nextDataPage(), valid: true}],
             writeVal: 0xCAFEBABE});
        idx = BASE + 31;
        add({name: "straddle-write-hi-invalid", op: "write", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: true}, {idx: idx + 1, page: nextDataPage(), valid: false}],
             writeVal: 0xCAFEBABE});
        idx = BASE + 32;
        add({name: "straddle-write-hi-oom", op: "write", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: true}, {idx: idx + 1, page: OOM_PAGE, valid: true}],
             writeVal: 0xCAFEBABE});
        /* the LOW half failing does NOT suppress the high half's own, independent attempt */
        idx = BASE + 33;
        add({name: "straddle-write-lo-invalid-hi-valid", op: "write", size: 4, qa: idx * PAGE + 0x1FE,
             entries: [{idx, page: nextDataPage(), valid: false}, {idx: idx + 1, page: nextDataPage(), valid: true}],
             writeVal: 0xCAFEBABE});
    }

    /* -- (4) cq_serr, CONSTRUCTED: a VALID entry pointing OUTSIDE memory, every size, both dirs -- */
    for (let op of ["read", "write"]) {
        for (let size of [1, 2, 4]) {
            let idx = BASE + 40 + size;
            add({name: `cq-serr-${op}-size${size}`, op, size, qa: idx * PAGE + 0x10,
                 entries: [{idx, page: OOM_PAGE, valid: true}], writeVal: 0x5A5A5A5A});
        }
    }

    /* -- (5, and cq_merr the DOMINANT ROM path) an INVALID entry, every size, both directions --- */
    for (let op of ["read", "write"]) {
        for (let size of [1, 2, 4]) {
            let idx = BASE + 50 + size;
            add({name: `cq-merr-${op}-size${size}`, op, size, qa: idx * PAGE + 0x20,
                 entries: [{idx, page: nextDataPage(), valid: false}], writeVal: 0x3C3C3C3C});
        }
    }

    /* -- lost-error: a PRE-EXISTING unresolved error, both latch sites -------------------------- */
    {
        let idx = BASE + 60;
        add({name: "lost-error-master", op: "read", size: 2, qa: idx * PAGE, dserPre: 0x80,
             entries: [{idx, page: nextDataPage(), valid: false}]});
        idx = BASE + 61;
        add({name: "lost-error-slave", op: "write", size: 2, qa: idx * PAGE, dserPre: 0x01, writeVal: 0x1234,
             entries: [{idx, page: OOM_PAGE, valid: true}]});
    }

    /* -- (5) byte/word at every offset alignment within a page, including both page edges ------- */
    for (let off of [0, 1, 2, 3, 4, 8, 0x100, 0x1FC, 0x1FD, 0x1FE, 0x1FF]) {
        for (let size of [1, 2]) {
            for (let op of ["read", "write"]) {
                let idx = BASE + 70 + off;
                let needsHi = (off + size) > (VA_M_OFF + 1);
                let entries = [{idx, page: nextDataPage(), valid: true}];
                if (needsHi) entries.push({idx: idx + 1, page: nextDataPage(), valid: true});
                add({name: `alignment-${op}-size${size}-off${hex(off, 3)}`, op, size,
                     qa: idx * PAGE + off, entries, writeVal: 0x66 | (off << 8)});
            }
        }
    }

    /* -- map entries carrying garbage in the RESERVED bits between CQMAP_PAG and CQMAP_VLD ------ */
    {
        let idx = BASE + 90;
        let RESERVED_BITS = (~(CQMAP_VLD | CQMAP_PAG)) & 0x7FFFFFFF;
        add({name: "entry-reserved-bits-set", op: "read", size: 4, qa: idx * PAGE + 0x30,
             entries: [{idx, page: nextDataPage(), valid: true, extra: RESERVED_BITS}]});
    }

    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The randomized phase -- structurally independent draws (standing rule 1)                       *
 * ------------------------------------------------------------------------------------------- */

function randomCases(rnd, n, startIdxBase)
{
    let out = [];
    /* Every case gets its own 2-entry slice of the map (no cross-case aliasing within one run), and
       the slice must stay inside [0, CQMAPSIZE/4) with room for idx+1 -- CQMAPSIZE/4 is 8192 map
       entries, comfortably more than any --cases value this file's floor or gate ever uses. */
    const MAXIDX = (CQMAPSIZE / 4) - 4;
    for (let k = 0; k < n; k++) {
        let idxBase = (startIdxBase + k * 2) % MAXIDX;
        /* Bias toward the straddle boundary (the last 4 bytes of the page) about a third of the
           time; otherwise uniform across the whole page -- structurally different from the
           enumerated pool's fixed offset list. Must stay < PAGE: qa's own idx (idxBase) is only
           valid for offsets that keep `qa >> 9 === idxBase`, which makeCase() re-derives and
           checks against the entries this function supplies. */
        let off = (rnd() < 0.35) ? (PAGE - 4 + rint(rnd, 4)) : rint(rnd, PAGE);
        let size = pick(rnd, [1, 2, 4]);
        let op = pick(rnd, ["read", "write"]);
        let qa = idxBase * PAGE + off;
        let idx = idxBase;
        let needsHi = ((off & VA_M_OFF) + size) > (VA_M_OFF + 1);

        let mkEntry = (i) => {
            let r = rnd();
            if (r < 0.55) return {idx: i, page: nextDataPage(), valid: true};
            if (r < 0.80) return {idx: i, page: nextDataPage(), valid: false};
            return {idx: i, page: OOM_PAGE, valid: true};
        };
        let entries = [mkEntry(idx)];
        if (needsHi) entries.push(mkEntry((idx + 1) >>> 0));

        out.push(makeCase({
            name: `random#${k}`, op, size, qa, entries,
            dserPre: pick(rnd, [0, 0, 0, 0x80, 0x01]),
            writeVal: (Math.floor(rnd() * 0x100000000)) | 0
        }));
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The oracle                                                                                     *
 * ------------------------------------------------------------------------------------------- */

const CASE_MARK = "@@CM@@";
const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/;
const REG_NAMES = ["R0", "PC", "PSL"];
const NREG_VALS = REG_NAMES.length + 6;         // R0,PC,PSL + DSER,MEAR,SEAR,MBR,BTO,MEMERR

/** seedValue/seedTargets -- ONE deterministic pseudo-random pattern over the whole DATA region,
    applied identically to both engines before any case runs (same convention as qdmadiff.js). */
function seedValue(addr) { return (Math.imul(addr >>> 2, 2654435761) ^ 0x5DEECE66) | 0; }
function seedTargets()
{
    let t = [];
    for (let a = DATA_BASE; a < DATA_BASE + DATA_LEN; a += 4) t.push(a);
    return t;
}

function buildScript(cases)
{
    let L = ["set cpu " + (MEMSIZE >> 20) + "m", "set cpu simhalt", "reset all",
        `deposit -l ${hex(MAP_SPAN[0])}:${hex(MAP_SPAN[1] - 4)} 0`];
    for (let a of seedTargets()) L.push(`deposit -l ${hex(a)} ${hex(seedValue(a))}`);
    L.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit -l ${hex(R_SCBB + SCB.MCHK)} ${hex(R_MCHK_H)}`,
        `deposit KSP ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`);
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        L.push("deposit cpu memerr 0", "deposit sysd bto 0");
        L.push(`deposit qba dser ${hex(c.dserPre, 2)}`, `deposit qba mear ${hex(c.mearPre, 4)}`,
            `deposit qba sear ${hex(c.searPre, 5)}`, "deposit qba mbr 0");
        for (let i = 0; i < c.code.length; i += 4) {
            let v = (c.code[i] | (c.code[i + 1] << 8) | (c.code[i + 2] << 16) | (c.code[i + 3] << 24)) >>> 0;
            L.push(`deposit -l ${hex((R_CODE + i) >>> 0)} ${hex(v)}`);
        }
        L.push(`deposit R0 ${hex(SENTINEL)}`, `deposit PSL ${hex(PSL_RUN)}`, `deposit PC ${hex(R_CODE)}`);
        L.push(`step ${c.nSetup}`);
        L.push(`step 1`);
        L.push(`examine -h R0,PC,PSL`);
        L.push("examine -h qba dser", "examine -h qba mear", "examine -h qba sear", "examine -h qba mbr");
        L.push("examine -h sysd bto", "examine -h cpu memerr");
        for (let r of c.ranges) L.push(`examine -h ${hex(r[0])}:${hex(r[1])}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

function runOracle(simh, cases, scratch)
{
    let out = runSimh(simh, buildScript(cases), path.join(scratch, "cqmmapdiff-batch.ini"));
    let lines = out.split("\n");
    let results = new Map();
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\d+)"));
        if (!m) { i++; continue; }
        let idx = +m[1];
        i++;
        let vals = [];
        while (i < lines.length) {
            if (lines[i].indexOf(CASE_MARK) >= 0) break;
            let vm = lines[i].match(VALUE_RE);
            if (vm) vals.push(parseInt(vm[2], 16) | 0);
            i++;
        }
        results.set(idx, vals);
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * The JS side                                                                                    *
 * ------------------------------------------------------------------------------------------- */

function seedMachine(m)
{
    let {bus, cpu} = m;
    for (let a = MAP_SPAN[0]; a < MAP_SPAN[1]; a += 4) bus.setLong(a, 0);
    for (let a of seedTargets()) bus.setLong(a, seedValue(a));
    cpu.exc.scbb = R_SCBB;
    bus.setLong(R_SCBB + SCB.MCHK, R_MCHK_H);
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
}

function runCaseJS(m, c)
{
    let {bus, cpu, cqbic} = m;
    cpu.exc.memErr = 0;
    cpu.exc.sscBto = 0;
    cpu.exc.cqDser = c.dserPre | 0;
    cpu.exc.cqMear = c.mearPre | 0;
    cqbic.sear = c.searPre | 0;
    cqbic.mbr = 0;
    for (let i = 0; i < c.code.length; i++) bus.setByte((R_CODE + i) >>> 0, c.code[i]);
    cpu.regs[0] = SENTINEL;
    cpu.psl = PSL_RUN | 0;
    cpu.regs[15] = R_CODE;

    let stop = null;
    let run = () => { try { cpu.stepCPU(1); } catch (e) { if (e instanceof VAXStop) { stop = e; return true; } throw e; } return false; };
    for (let k = 0; k < c.nSetup && !stop; k++) {
        if (run()) throw new Error(`cqmmapdiff.js: case "${c.name}" halted during SETUP (instruction ${k}) -- ` +
            "a map/MBR write must never fault");
    }
    if (!stop) run();                                          // the probe -- may legitimately fault

    let vals = [cpu.regs[0] | 0, cpu.regs[15] | 0, cpu.psl | 0,
        cpu.exc.cqDser | 0, cpu.exc.cqMear | 0, cqbic.sear | 0, cqbic.mbr | 0,
        cpu.exc.sscBto | 0, cpu.exc.memErr | 0];
    for (let r of c.ranges) for (let a = r[0]; a <= r[1]; a += 4) vals.push(bus.getLong(a) | 0);
    return vals;
}

/* ------------------------------------------------------------------------------------------- *
 * Comparison                                                                                     *
 * ------------------------------------------------------------------------------------------- */

const VAL_NAMES = REG_NAMES.concat(["DSER", "MEAR", "SEAR", "MBR", "BTO", "MEMERR"]);

function compareCase(c, jsVals, srVals)
{
    let bad = [];
    let tag = `case#${c.index} "${c.name}" op=${c.op} size=${c.size} qa=0x${hex(c.qa)}`;
    if (!srVals) { bad.push(`${tag}: NOT REACHED (no oracle output for this case)`); return bad; }
    let want = NREG_VALS + c.nMemVals;
    if (srVals.length !== want) {
        bad.push(`${tag}: oracle returned ${srVals.length} values, expected ${want}`);
        return bad;
    }
    for (let i = 0; i < NREG_VALS; i++) {
        if ((jsVals[i] | 0) !== (srVals[i] | 0)) {
            bad.push(`${tag}: ${VAL_NAMES[i]} js=${hex(jsVals[i])} simh=${hex(srVals[i])}`);
        }
    }
    let i = NREG_VALS, nBad = 0;
    for (let r of c.ranges) {
        for (let a = r[0]; a <= r[1]; a += 4, i++) {
            if ((jsVals[i] | 0) !== (srVals[i] | 0)) {
                if (nBad++ < 6) bad.push(`${tag}: mem[${hex(a)}] js=${hex(jsVals[i])} simh=${hex(srVals[i])}`);
            }
        }
    }
    if (nBad > 6) bad.push(`${tag}: ... and ${nBad - 6} more diverging longwords`);
    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors -- ABSOLUTE, derived from the case's own spec, FAIL the run (standing rule 4)   *
 * ------------------------------------------------------------------------------------------- */

const MIN_CASES_FLOOR   = 150;
const MIN_TOTAL_OPS     = 200;
const MIN_PER_DIRECTION = 60;
const MIN_PER_SIZE      = {1: 20, 2: 20, 4: 40};
const MIN_STRADDLE      = 6;
const MIN_CQ_SERR       = 6;    // constructed (item priority 4) -- the ROM measured zero of these
const MIN_CQ_MERR       = 10;
const MIN_SUCCESS       = 60;   // the primary mapped success path (item priority 1)
const MIN_LOST_ERROR    = 2;
const MIN_UNALIGNED     = 10;

function runPhase(simh, scratch, opts, label)
{
    autoPage = 0;
    let rnd = mulberry32(opts.seed || 1);
    let m = makeMachine();
    seedMachine(m);

    let enumCases = enumeratedCases();
    let randCases = randomCases(rnd, opts.cases, 0x400);
    let cases = enumCases.concat(randCases);
    cases.forEach((c, i) => { c.index = i; });

    let sr = runOracle(simh, cases, scratch);
    let failures = [], notReached = [];
    let stats = {
        nOps: 0, byDir: {read: 0, write: 0}, bySize: {1: 0, 2: 0, 4: 0},
        nStraddle: 0, nCqSerr: 0, nCqMerr: 0, nSuccess: 0, nLostError: 0, nUnaligned: 0
    };
    for (let c of cases) {
        let srVals = sr.get(c.index);
        if (!srVals) { notReached.push(`${label} case ${c.index} "${c.name}"`); continue; }
        let jsVals = runCaseJS(m, c);
        let bad = compareCase(c, jsVals, srVals);
        if (bad.length) failures.push(...bad);
        stats.nOps++;
        stats.byDir[c.op]++;
        stats.bySize[c.size]++;
        if (c.needsHi) stats.nStraddle++;
        if (c.hasOom) stats.nCqSerr++;
        if (c.hasInvalid) stats.nCqMerr++;
        if (c.allGood) stats.nSuccess++;
        if (c.dserPre || c.mearPre || c.searPre) stats.nLostError++;
        if (c.unaligned) stats.nUnaligned++;
    }
    return {failures, notReached, stats};
}

/* ------------------------------------------------------------------------------------------- *
 * Self-check mutations                                                                          *
 * ------------------------------------------------------------------------------------------- */

const MUTATED_METHODS = [
    [CQMVAX.prototype, "readLong"], [CQMVAX.prototype, "write"], [CQMVAX.prototype, "read"],
    [CQBICVAX.prototype, "mapAddr"]
];
function snapshotProto() { return MUTATED_METHODS.map(([obj, name]) => [obj, name, obj[name]]); }
function restoreProto(save) { for (let [obj, name, fn] of save) obj[name] = fn; }

const MUTATIONS = [
    {name: "longword read composed from ONE backing longword instead of two mapped word cycles " +
        "(see this file's MEASURED, DISCLOSED NON-FINDING -- expected NOT CAUGHT)", apply() {
        CQMVAX.prototype.readLong = function(addr) {
            let c = this.cqbic;
            c.requireBus();
            let lo = this.read(addr, 2);                       // the ONE real translation this bug keeps
            if (lo === REG_MCHK) return REG_MCHK;
            let hi = (c.bus.getLong(c.mapMA & ~3) >>> 16) & 0xFFFF;   // BUG: reuses the SAME backing longword
            return ((hi << 16) | (lo & 0xFFFF)) | 0;
        };
    }},
    {name: "cqm_wr shift taken from pa instead of ma (see this file's MEASURED, DISCLOSED " +
        "NON-FINDING -- expected NOT CAUGHT)", apply() {
        CQMVAX.prototype.write = function(addr, val, lnt) {
            let c = this.cqbic;
            c.requireBus();
            let qa = ((addr >>> 0) & (CQM_LENGTH - 1)) >>> 0;
            if (!c.mapAddr(qa)) { c.exc.memErr = 1; return true; }
            let ma = c.mapMA >>> 0;
            let pa = addr >>> 0;                                // BUG: shift from pa, not ma
            let sc = (lnt === 1) ? ((pa & 3) << 3) : ((pa & 2) << 3);
            let mask = (lnt === 1) ? 0xFF : 0xFFFF;
            let t = c.bus.getLong(ma & ~3);
            c.bus.setLong(ma & ~3, ((t & ~(mask << sc)) | ((val & mask) << sc)) | 0);
            return true;
        };
    }},
    {name: "CQMVAX returning null instead of REG_MCHK on a failed translation (the double-latch bug)",
     apply() {
        CQMVAX.prototype.read = function(addr, lnt) {
            let c = this.cqbic;
            c.requireBus();
            let pa = addr >>> 0;
            let qa = (pa & (CQM_LENGTH - 1)) >>> 0;
            if (!c.mapAddr(qa)) return null;                    // BUG: was REG_MCHK
            let w = (c.bus.getLong(c.mapMA & ~3) >>> ((pa & 2) ? 16 : 0)) & 0xFFFF;
            return (lnt === 1) ? ((w >>> ((pa & 1) ? 8 : 0)) & 0xFF) : w;
        };
    }},
    {name: "map entry VLD bit ignored", apply() {
        CQBICVAX.prototype.mapAddr = function(qa) {
            qa = qa >>> 0;
            let qmma = this.mapEntryAddr(qa);
            if (this.isMem(qmma)) {
                let qmap = this.mapEntry(qmma);
                // BUG: no `if (qmap & CQMAP_VLD)` check -- every entry treated as valid
                let ma = ((((qmap & CQMAP_PAG) << 9) >>> 0) + (qa & VA_M_OFF)) >>> 0;
                if (this.isMem(ma)) { this.mapMA = ma; return true; }
                this.cqSerr(ma);
                return false;
            }
            this.cqSerr(0);
            return false;
        };
    }}
];

function selfcheck(simh, scratch, opts)
{
    let results = [];
    for (let mut of MUTATIONS) {
        let save = snapshotProto();
        let caught = false, why = "";
        try {
            mut.apply();
            let r = runPhase(simh, scratch, {seed: opts.seed ^ 0x5A5A, cases: 40}, "selfcheck");
            caught = (r.failures.length > 0) || (r.notReached.length > 0);
            why = caught ? (r.failures[0] || r.notReached[0]) : "NO FAILURES REPORTED";
        } catch (e) {
            caught = true;
            why = "threw: " + e.message.split("\n")[0];
        }
        restoreProto(save);
        results.push({name: mut.name, caught, why});
        console.log(`  selfcheck ${caught ? "CAUGHT   " : "*** NOT CAUGHT ***"} ${mut.name} (${why})`);
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                           *
 * ------------------------------------------------------------------------------------------- */

function parseCases(raw)
{
    if (raw === null || raw === undefined) return 240;
    let n = Number(raw);
    if (!Number.isFinite(n)) throw new Error(`cqmmapdiff.js: --cases "${raw}" is not a number`);
    return n;
}

function main()
{
    assertLayout();
    let argv = process.argv.slice(2);
    let getArg = (name, dflt) => { let i = argv.indexOf(name); return i >= 0 ? argv[i + 1] : dflt; };
    let simh = findSimh(getArg("--simh", null));
    let seed = +getArg("--seed", 0xC0FFEE);
    let nCases = parseCases(getArg("--cases", null));
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "vax-cqmmapdiff-"));

    console.log("VAX CQM mapped read/write differential test (pcjsvax-55d)");
    console.log("  SIMH binary: %s", simh);
    console.log("  seed=0x%s cases=%d", hex(seed), nCases);

    if (nCases < MIN_CASES_FLOOR) {
        console.log("\nFAILED: --cases %d is below the enforced floor (%d); this run would under-cover " +
            "and must not be allowed to pass.", nCases, MIN_CASES_FLOOR);
        fs.rmSync(scratch, {recursive: true, force: true});
        process.exit(1);
    }

    let errors = [];
    try {
        let t0 = Date.now();
        let {failures, notReached, stats} = runPhase(simh, scratch, {seed, cases: nCases}, "main");
        console.log("  elapsed: %ds", ((Date.now() - t0) / 1000).toFixed(1));

        console.log("\nComparisons: ops=%d read=%d write=%d byte=%d word=%d long=%d straddle=%d " +
            "cq_serr=%d cq_merr=%d success=%d lost-error=%d unaligned=%d",
            stats.nOps, stats.byDir.read, stats.byDir.write, stats.bySize[1], stats.bySize[2], stats.bySize[4],
            stats.nStraddle, stats.nCqSerr, stats.nCqMerr, stats.nSuccess, stats.nLostError, stats.nUnaligned);

        for (let f of failures.slice(0, 40)) errors.push(f);
        if (failures.length > 40) errors.push(`... and ${failures.length - 40} more failures`);
        for (let n of notReached) errors.push("NOT REACHED: " + n);

        let require = (cond, msg) => { if (!cond) errors.push("COVERAGE: " + msg); };
        require(stats.nOps >= MIN_TOTAL_OPS, `fewer than ${MIN_TOTAL_OPS} operations (${stats.nOps})`);
        require(stats.byDir.read >= MIN_PER_DIRECTION, `too few read probes (${stats.byDir.read})`);
        require(stats.byDir.write >= MIN_PER_DIRECTION, `too few write probes (${stats.byDir.write})`);
        for (let sz of [1, 2, 4]) {
            require(stats.bySize[sz] >= MIN_PER_SIZE[sz], `too few size=${sz} probes (${stats.bySize[sz]})`);
        }
        require(stats.nStraddle >= MIN_STRADDLE, `too few page-straddling longword probes (${stats.nStraddle})`);
        require(stats.nCqSerr >= MIN_CQ_SERR, `too few constructed cq_serr probes (${stats.nCqSerr}) -- ` +
            "the ROM measured ZERO of these; they exist only if this file constructs them");
        require(stats.nCqMerr >= MIN_CQ_MERR, `too few cq_merr probes (${stats.nCqMerr})`);
        require(stats.nSuccess >= MIN_SUCCESS, `too few fully-successful mapped probes (${stats.nSuccess})`);
        require(stats.nLostError >= MIN_LOST_ERROR, `too few pre-existing-error (lost-error) probes (${stats.nLostError})`);
        require(stats.nUnaligned >= MIN_UNALIGNED, `too few unaligned probes (${stats.nUnaligned})`);

        if (fSelfCheck) {
            console.log("\nSelf-check: the differential must FAIL when the mechanism is deliberately broken.");
            let results = selfcheck(simh, scratch, {seed});
            for (let r of results) {
                if (r.name.indexOf("NON-FINDING") >= 0) {
                    if (r.caught) {
                        console.log(`  NOTE: "${r.name}" WAS caught this run -- the file header's algebraic ` +
                            "argument for why it cannot be may be wrong, or this run got lucky; re-examine before trusting it as a non-finding.");
                    }
                    continue;           // NOT required to be caught -- see the file header's disclosure
                }
                if (!r.caught) errors.push(`SELFCHECK: mutation '${r.name}' was not detected`);
            }
        }
    } finally {
        if (!process.env["VAX_CQMMAPDIFF_KEEP"]) fs.rmSync(scratch, {recursive: true, force: true});
    }

    if (errors.length) {
        console.log("\nFAILED (%d):", errors.length);
        for (let e of errors) console.log("  " + e);
        process.exit(1);
    }
    console.log("\nPASS: the CQM window's mapped read/write path matches real SIMH -- valid-entry " +
        "reads/writes, the longword two-cycle split, page-straddling, constructed cq_serr, and " +
        "cq_merr all agree with the oracle.");
}

if (process.argv[1] && path.resolve(process.argv[1]) == path.resolve(fileURLToPath(import.meta.url))) {
    main();
}

export { makeMachine, runCaseJS, enumeratedCases, randomCases, makeCase, seedMachine };
