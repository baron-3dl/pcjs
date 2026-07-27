/**
 * @fileoverview Differential test: a Qbus device DMAs to and from VAX memory through the CQBIC
 *               scatter-gather map -- vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-e22.  HANDOFF.md section 5 records the gap: "VMS needs a disk controller that does not
 * exist in any form -- rq/MSCP lives behind the Qbus I/O page, and no item owns it yet."  This
 * file grades the half of that gap which is NOT the controller: the DMA data path every Qbus
 * mass-storage device uses to move data, which is the CQBIC's 8192-entry scatter-gather map
 * (CQMAPBASE, vaxmod_defs.h:196-199) plus the four routines devices call to walk it
 * (Map_ReadB/Map_ReadW/Map_WriteB/Map_WriteW, vaxmod_defs.h:459-462, vax_io.c:769-895).
 *
 * TWO HALVES, ONE MECHANISM, AND THAT IS THE POINT
 * -------------------------------------------------
 * The map's entries are not registers.  They are ORDINARY MAIN MEMORY at `(pa & CQMAPAMASK) +
 * cq_mbr`, and cqmap_rd()/cqmap_wr() (vax_io.c:574-602) are a window onto that memory.  So the
 * map has two callers that must not be allowed to drift apart:
 *
 *   THE CPU SIDE   A program programs the map with ordinary instructions -- `MOVL I^#entry,
 *                  @#CQMAPBASE+n*4` -- through cqbic.js's CQMAPVAX.  EVERY case below programs
 *                  its map, and its MBR, that way: real instructions through cpu.stepCPU(), not
 *                  a direct call to an accessor.  HANDOFF.md section 7 premise 7 is explicit
 *                  about why (pcjsvax-855 shipped an unaligned stitch into ssc.js whose branches
 *                  turned out to be unreachable, and it was caught ONLY because the differential
 *                  executed instructions).  Here the guard is structural rather than stylistic:
 *                  if CQMAPVAX's decode were dead code, every map entry would read back as
 *                  whatever was already in memory and every DMA below would transfer the wrong
 *                  bytes.  The CPU path cannot be dead while the DMA path passes.
 *   THE DEVICE SIDE  A DMA is by definition NOT a CPU instruction, so no instruction stream can
 *                  reach Map_ReadB.  The oracle for that half is patch 0007's SHOW QBA QDMA=,
 *                  which calls the real vax_io.c routines and reports the residual count and the
 *                  CQBIC error registers (tests/simh/README.md; the patch's own comment explains
 *                  why EXAMINE QBA, which uses the console-only qba_map_addr_c(), cannot serve).
 *
 * THE CHEAT THIS FILE EXISTS TO KILL
 * -----------------------------------
 * Identity-mapping the Qbus address onto a VAX physical address satisfies any single-page test
 * completely, and is wrong the moment VMS supplies a real scatter-gather list.  So:
 *   - the coverage floors REQUIRE a transfer spanning three or more map entries whose physical
 *     pages are DISCONTIGUOUS and out of order (assertion `threeDiscontig`, below), and
 *   - --selfcheck's first mutation is exactly that cheat ("map-lookup-bypassed"), and
 *   - --selfcheck additionally proves the cheat SURVIVES the single-page identity-mapped case
 *     (`identity-mapped-single-page`) and is killed only by the discontiguous one.  A suite that
 *     merely reports "CAUGHT" cannot tell you WHICH case did the catching; this one does, because
 *     the whole point of the floor is that the easy cases are blind to it.
 *
 * WHAT IS GRADED, PER CASE, BYTE FOR BYTE
 * ----------------------------------------
 *   1. The map registers' own read-back values -- R0..R6 after real MOVL/MOVZWL/MOVZBL probes.
 *   2. PC and PSL after the instruction stream (a faulting map access would show up here first).
 *   3. The CQBIC register file: SCR, DSER, MEAR, SEAR, MBR, plus ssc_bto and mem_err.
 *   4. The DMA's RESIDUAL COUNT -- the count NOT transferred; 0 on full success.
 *   5. The DMA's error side effects: DSER/MEAR/SEAR again, IPC, mem_err.
 *   6. Every byte of the device buffer AND every byte of every physical page the transfer could
 *      have touched -- whole 512-byte pages, not just the intended extent, so a transfer that
 *      writes outside its range is caught too.  Read cases dump their source pages for the same
 *      reason: a DMA read must not modify memory.
 *
 * ssc_bto IS GRADED AND MUST STAY 0.  A CQBIC map failure is cq_serr()/cq_merr(), never the SSC
 * bus-timeout mechanism mchkdiff.js owns -- the same distinction cqmerrdiff.js grades on the
 * CPU-reference side.
 *
 * TWO PHASES, STRUCTURALLY DIFFERENT VIEWS (standing rule 1)
 * -----------------------------------------------------------
 *   ENUMERATED   Deterministic, named cases, one per thing that can go wrong: within a page,
 *                across a page, across three and four DISCONTIGUOUS pages, unaligned start,
 *                unaligned count, odd count and odd address on the word routines, an INVALID map
 *                entry first / in the middle, a valid entry pointing PAST the end of memory
 *                (slave NXM -- a different error latch entirely), a pre-existing DSER error
 *                (lost-error bit), an MBR pointing past the end of memory, a zero-length
 *                transfer, a bus address above the CQBIC's 22-bit width, byte- and word-sized
 *                map-register writes and read-backs, and aliased entries.
 *   RANDOMIZED   Uniform draws over MBR, entry count, per-entry validity/target page, bus
 *                address, byte count, direction and routine width, plus a random pre-existing
 *                DSER.  Not a re-run of the enumerated pool at another granularity: the
 *                enumerated cases are built to hit named boundaries, the randomized ones draw
 *                page targets that are deliberately shuffled so most transfers are scattered.
 *
 * WHAT IS DELIBERATELY NOT GRADED, BY NAME (standing rule 6)
 * -----------------------------------------------------------
 *   - cqmap_rd()'s out-of-memory READ branch (MBR pointing past memory, then READING a map
 *     register with a CPU instruction).  SIMH does cq_serr() + MACH_CHECK(MCHK_READ) with
 *     ssc_bto UNTOUCHED; cqbic.js can latch the cq_serr() half but the ssc_bto suppression seam
 *     is cpustate.js's onBusFault(), outside pcjsvax-e22's change fence.  No case below reads a
 *     map register while MBR is out of memory, and `assertExclusions()` FAILS the run if one
 *     ever does.  The WRITE half of the same branch (cq_serr() + a deferred mem_err, no
 *     exception) is exact and IS graded -- see the `mbr-out-of-memory-*` cases.
 *   - cqm_rd()/cqm_wr(), the CPU-side Qbus-memory window at CQMBASE that walks the same map.
 *     Still undecoded; cqmerrdiff.js grades its present unbacked behaviour and stays green.
 *     Same fence reason (its map-failure path needs the same machine-check seam).
 *   - CQIPC.  Read and compared as a constant 0 (nothing here can change it), never programmed.
 *
 * THE EXCLUSION IS MEASURED, NOT MERELY ASSERTED.  Deleting the memory-bounds check from
 * cqbic.js's mapRegRead() -- `if (this.isMem(ma))` -> `if (true)` -- was injected into the shipped
 * source and SURVIVED a full run, which is exactly what the exclusion above predicts: for every
 * in-memory MBR the check is a no-op, and no case reads a map register with an out-of-memory MBR
 * (makeCase() sets `readsMapWithBadMbr`, and main() FAILS the run if any case ever does).  Two
 * other injected defects also survived, each for a stated reason rather than an unstated one:
 * dropping `ba & QBMAMASK` (provably redundant -- see the note above FLOORS), and, before the
 * staging guard tail existed, a wrong alignment test in the read routines (see stagedCount()).
 * Everything else injected -- the map-index fold, CQMAP_PAG's width, the per-page remap trigger,
 * the word routines\' two masks, both error latches, the lost-error rule, SEAR\'s page shift, the
 * sub-longword register lane, the deferred mem_err, the residual arithmetic -- turned the run red.
 *
 * MEMORY (standing rule 14, earned by cdgdiff.js reaching 8.6 GB RSS and OOM-killing its
 * siblings): ONE machine, ONE bus, ONE 4KB device buffer, all built once and reused by every
 * case; ONE SIMH process for the whole run, so both engines see a single continuous memory
 * timeline and nothing has to be re-seeded at a batch boundary.  `assertHeap()` fails the run on
 * an ABSOLUTE peak-heap bound that does not scale with case count.
 *
 *      node machines/dec/vax/tests/qdmadiff.js [options]
 *        --simh PATH       microvax3900 built WITH PATCH 0007 (else $SIMH_BIN, else the scratch
 *                           build).  A binary without 0007 is reported as such and FAILS; it is
 *                           never silently skipped.
 *        --cases N         randomized cases (default 240; below MIN_CASES_FLOOR the run FAILS --
 *                           it does not silently clamp up)
 *        --seed S          PRNG seed, printed on failure so a run is reproducible
 *        --selfcheck       prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import { OPCODES } from "../modules/v2/drom.js";
import CPUStateVAX, { VAXStop } from "../modules/v2/cpustate.js";
import CQBICVAX, { CQMAPVAX, CQMAP_BASE, CQMAPSIZE, CQMAP_VLD, CQMAP_PAG, QBMAMASK, VA_M_OFF } from "../modules/v2/cqbic.js";
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
 * Physical layout.  Every region below is disjoint, and `assertLayout()` proves it rather than    *
 * asserting it in a comment (standing rule 12).                                                  *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE     = 0x01000000;                 // 16MB, the SIMH microvax3900 default
const PAGE        = 512;                        // VA_PAGSIZE
const R_CODE      = 0x00104000;                 // the instruction stream
const R_CODE_LEN  = 0x00000400;
const DATA_BASE   = 0x00200000;                 // the pages a DMA transfers to/from
const DATA_LEN    = 0x00008000;                 // 64 pages
const MAP_BASES   = [0x00300000, 0x00308000, 0x00310000];   // legal MBR values (32KB aligned)
const MAP_SPAN    = [0x00300000, 0x00318000];   // the union of the three, for one-time zeroing
const STAGE       = 0x00400000;                 // the device buffer's staging area
const QDMA_MAXBC  = 0x1000;                     // patch 0007's QDMA_MAXBC
const QDMA_FILL   = 0xE5;                       // patch 0007's QDMA_FILL
const QDMA_GUARD  = 8;                          // patch 0007's QDMA_GUARD -- see stagedCount()
const IDX_MASK    = (CQMAPSIZE / 4) - 1;        // 8191: (qblk << 2) & CQMAPAMASK wraps here
const MAXBC       = 0x600;                      // 1536 -- three whole pages, the 3-entry floor
const MAX_PAGES   = 6;                          // most pages one case may dump
const OOM_MBR     = 0x01FF8000;                 // 32KB-aligned, past the end of memory
const OOM_PAGE    = (MEMSIZE / PAGE) + 0x40;    // a map target page past the end of memory
const DATA_PAGE0  = DATA_BASE / PAGE;           // first usable target page number
const DATA_NPAGE  = DATA_LEN / PAGE;

/* PSL: kernel mode, IS, IPL 31.  IPL 31 is load-bearing, not cosmetic: a map-register write with
   MBR out of memory sets the DEFERRED mem_err, and at IPL 31 it cannot dispatch, so the flag stays
   observable in `examine cpu memerr` instead of turning into an interrupt whose frame is
   cqmerrdiff.js's subject, not this file's. */
const PSL_RUN     = 0x041F0000;

const CQBIC_MBR   = (VAX.PHYSMEM.REG_BASE + 0x10) >>> 0;    // vax_io.c's cq_mbr, register 4

function assertLayout()
{
    let regions = [
        ["CODE",  R_CODE, R_CODE + R_CODE_LEN],
        ["DATA",  DATA_BASE, DATA_BASE + DATA_LEN],
        ["MAP",   MAP_SPAN[0], MAP_SPAN[1]],
        ["STAGE", STAGE, STAGE + QDMA_MAXBC]
    ];
    for (let i = 0; i < regions.length; i++) {
        let [n, lo, hi] = regions[i];
        if (hi > MEMSIZE) throw new Error(`qdmadiff.js: region ${n} ends past MEMSIZE`);
        for (let j = i + 1; j < regions.length; j++) {
            let [n2, lo2, hi2] = regions[j];
            if (lo < hi2 && lo2 < hi) throw new Error(`qdmadiff.js: regions ${n} and ${n2} overlap`);
        }
    }
    if (MAP_BASES.some((b) => b + CQMAPSIZE > MAP_SPAN[1] || b < MAP_SPAN[0])) {
        throw new Error("qdmadiff.js: a MAP_BASES entry falls outside MAP_SPAN");
    }
    if (MAXBC > QDMA_MAXBC - QDMA_GUARD - 4) throw new Error("qdmadiff.js: MAXBC exceeds patch 0007's own limit");
    if (OOM_MBR < MEMSIZE || OOM_PAGE * PAGE < MEMSIZE) {
        throw new Error("qdmadiff.js: the out-of-memory constants are inside memory");
    }
    /* The Qbus page index of every DATA page must be reachable through a 22-bit bus address, or a
       case could not address it at all.  Derived, not assumed. */
    if ((DATA_PAGE0 + DATA_NPAGE) * PAGE > (QBMAMASK + 1)) {
        throw new Error("qdmadiff.js: DATA pages are not reachable through a 22-bit Qbus address");
    }
    /* The observation vector's length and its NAMES are derived from the same NPROBE; assert they
       agree rather than trusting that two hand-kept lists stayed in step (standing rule 5). */
    if (VAL_NAMES.length !== NREG_VALS) {
        throw new Error(`qdmadiff.js: VAL_NAMES has ${VAL_NAMES.length} entries, NREG_VALS is ${NREG_VALS}`);
    }
    /* cqbic.js's bus-less guard, exercised rather than assumed (see its constructor doc): a CQBIC
       built without a bus must THROW from every map/DMA entry point, not quietly report a full
       residual, which is what it would do if the guard were removed. */
    let bare = new CQBICVAX({cqDser: 0, cqMear: 0, memErr: 0});
    for (let fn of ["mapReadB", "mapReadW", "mapWriteB", "mapWriteW"]) {
        let threw = false;
        try { bare[fn](0, 4, new Uint8Array(4)); } catch (e) { threw = true; }
        if (!threw) throw new Error(`qdmadiff.js: CQBICVAX.${fn}() did not throw without a bus`);
    }
    for (let fn of ["mapRegRead", "mapRegWrite"]) {
        let threw = false;
        try { bare[fn](CQMAP_BASE, 0, 4); } catch (e) { threw = true; }
        if (!threw) throw new Error(`qdmadiff.js: CQBICVAX.${fn}() did not throw without a bus`);
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
    return {bus, cpu, cqbic, buf: new Uint8Array(QDMA_MAXBC)};
}

/** Opcode numbers, resolved by mnemonic -- never hand-transcribed. */
function opcodeOf(name)
{
    let opc = OPCODES.indexOf(name);
    if (opc < 0) throw new Error(`qdmadiff.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
    return opc;
}
const OPC = {
    MOVL: opcodeOf("MOVL"), MOVW: opcodeOf("MOVW"), MOVB: opcodeOf("MOVB"),
    MOVZWL: opcodeOf("MOVZWL"), MOVZBL: opcodeOf("MOVZBL")
};

/* ------------------------------------------------------------------------------------------- *
 * Instruction builders.  Absolute mode is 9F (autoincrement deferred on PC); immediate is 8F      *
 * (autoincrement on PC), whose operand width follows the OPCODE's data size -- which is exactly    *
 * what makes MOVB/MOVW immediates one and two bytes here.                                          *
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

const NPROBE = 7;                               // R0..R6 are the read-back probes

/**
 * makeCase(spec)
 *
 * Turns a declarative spec into a fully resolved case: the instruction stream (with its length
 * and instruction count), the direct deposits that pre-stage memory, the map entry table, and --
 * derived from the SPEC, never from the code under test -- the exact list of physical longwords
 * that must be compared afterwards.  The harness deciding what to grade by asking cqbic.js where
 * the data went would grade a defect against itself.
 *
 * @param {Object} spec
 * @returns {Object} case
 */
function makeCase(spec)
{
    let c = Object.assign({
        mbr: MAP_BASES[0],
        entries: [],            // [{idx, page, valid, size}] programmed with CPU instructions
        probes: [],             // [{idx, size}] read back with CPU instructions into R0..
        deposits: [],           // [{addr, val}] direct memory pre-stage (both engines, in order)
        dserPre: 0, mearPre: 0, searPre: 0,
        op: 0, ba: 0, bc: 0
    }, spec);
    c.sa = STAGE;

    /* The Qbus indices this transfer can reach, computed the way the ROUTINE will mask, so a case
       that deliberately overflows the 22-bit bus address still zeroes the entries it will land on. */
    let ba0 = (c.ba & QBMAMASK) >>> 0;
    let bc0 = c.bc;
    if (c.op === 1 || c.op === 3) { ba0 = ba0 & ~1; bc0 = bc0 & ~1; }
    c.idxLo = bc0 > 0 ? (ba0 / PAGE) | 0 : -1;
    c.idxHi = bc0 > 0 ? ((ba0 + bc0 - 1) / PAGE) | 0 : -1;

    /* Every map entry index the case touches, programmed or merely read: zeroed first so a
       previous case's leftovers cannot make this one pass or fail by accident. */
    let idxs = new Set(c.entries.map((e) => entIdx(e.idx)));
    for (let i of c.probes.map((p) => entIdx(p.idx))) idxs.add(i);
    for (let i = c.idxLo; i >= 0 && i <= c.idxHi; i++) idxs.add(entIdx(i));
    c.zeroIdx = [...idxs].sort((a, b) => a - b);

    /* The entry VALUE table this case intends -- the harness's own model of the map, used only to
       decide which physical pages to dump. */
    let entVal = (e) => (((e.valid ? CQMAP_VLD : 0) | e.page | (e.extra || 0)) | 0);
    let want = new Map();
    for (let e of c.entries) want.set(entIdx(e.idx), entVal(e));

    let pages = [];
    for (let i = c.idxLo; i >= 0 && i <= c.idxHi; i++) {
        let v = want.get(entIdx(i));
        if (v === undefined) continue;
        if (!(v & CQMAP_VLD)) continue;
        let pa = (v & CQMAP_PAG) * PAGE;
        if (pa >= MEMSIZE) continue;
        if (pages.indexOf(pa) < 0) pages.push(pa);
    }
    if (pages.length > MAX_PAGES) throw new Error(`qdmadiff.js: case "${c.name}" touches ${pages.length} pages, over MAX_PAGES`);
    c.pages = pages;

    /* The longword ranges to compare: the staged device buffer, then every physical page the
       transfer could have reached, WHOLE -- so a transfer that writes outside its extent is caught. */
    c.sbc = stagedCount(c.op, c.bc);
    let ranges = [];
    if (c.sbc > 0) ranges.push([c.sa, (c.sa + ((c.sbc + 3) & ~3) - 4) >>> 0]);
    for (let p of pages) ranges.push([p, p + PAGE - 4]);
    c.ranges = ranges;
    c.nMemVals = ranges.reduce((n, r) => n + ((r[1] - r[0]) / 4 + 1), 0);

    /* The instruction stream: MBR first (the map window's address depends on it), then the entry
       writes, then the read-back probes. */
    let code = [], nInstr = 0;
    let emit = (bytes) => { for (let b of bytes) code.push(b & 0xFF); nInstr++; };
    emit(movImmAbs(4, c.mbr, CQBIC_MBR));
    for (let e of c.entries) {
        let val = entVal(e) >>> 0;
        let addr = (CQMAP_BASE + entIdx(e.idx) * 4) >>> 0;
        if (e.size === 4 || e.size === undefined) {
            emit(movImmAbs(4, val, addr));
        } else if (e.size === 2) {
            emit(movImmAbs(2, val & 0xFFFF, addr));
            emit(movImmAbs(2, (val >>> 16) & 0xFFFF, (addr + 2) >>> 0));
        } else {
            for (let k = 0; k < 4; k++) emit(movImmAbs(1, (val >>> (k * 8)) & 0xFF, (addr + k) >>> 0));
        }
    }
    if (c.probes.length > NPROBE) throw new Error(`qdmadiff.js: case "${c.name}" has more probes than registers`);
    c.probes.forEach((p, n) => {
        emit(movAbsReg(p.size, (CQMAP_BASE + entIdx(p.idx) * 4 + (p.off || 0)) >>> 0, n));
    });
    /* The last probe register always reads MBR back through the CQBIC's own register window, so
       every case proves the value the map window is using is the value the program wrote. */
    emit(movAbsReg(4, CQBIC_MBR, NPROBE - 1));
    if (code.length > R_CODE_LEN) throw new Error(`qdmadiff.js: case "${c.name}" code overruns R_CODE_LEN`);
    c.code = code;
    c.nInstr = nInstr;
    c.nProbes = c.probes.length;

    /* Exclusion guard (see the file header): reading a map register while MBR is out of memory is
       the ONE case cqbic.js cannot reproduce exactly.  No case may construct it. */
    c.readsMapWithBadMbr = (c.mbr + CQMAPSIZE > MEMSIZE) && c.probes.length > 0;
    return c;
}

/* ------------------------------------------------------------------------------------------- *
 * The enumerated phase -- one named case per thing that can go wrong                             *
 * ------------------------------------------------------------------------------------------- */

/**
 * stagedCount(op, bc)
 *
 * How many staged bytes patch 0007 copies back out after a transfer -- for a READ, `bc` rounded UP
 * to a longword plus a QDMA_GUARD tail; for a WRITE, just `bc` (the staging is input only).
 *
 * THE TAIL IS LOAD-BEARING, and it was added because a defect got past this file without it.
 * Map_ReadB/Map_ReadW pick their loop on `(ba | bc) & 3`, and the LONGWORD loop moves
 * ceil(bc/4)*4 bytes -- so a wrong alignment test over-reads into the caller's buffer past `bc`.
 * Every VAX page is a multiple of four, so that overrun can never cross a page boundary and
 * therefore never changes the residual or any error register.  Staging the tail is the ONLY way it
 * is observable.  Measured: injecting `(ba | bc) & 3` -> `ba & 3` into mapReadB SURVIVED a full run
 * before this existed, and is caught after.
 *
 * @param {number} op
 * @param {number} bc
 * @returns {number}
 */
function stagedCount(op, bc) { return (op < 2) ? (((bc + 3) & ~3) + QDMA_GUARD) : bc; }

/** The map entry index a Qbus block index actually lands on: `(qblk << 2) & CQMAPAMASK` folds
    everything at or above 8192 back to the bottom of the map.  Normalised in ONE place so the
    case builder, the coverage classifier and the instruction emitter cannot disagree. */
function entIdx(i) { return i & IDX_MASK; }

/** A valid entry pointing at DATA page k (0-based within the DATA region). */
function dpage(k) { return DATA_PAGE0 + (k % DATA_NPAGE); }

/** A Qbus address in map-index `idx` at page offset `off`. */
function qaddr(idx, off) { return (idx * PAGE + off) >>> 0; }

/** Everything between CQMAP_PAG's top bit and CQMAP_VLD -- derived, never typed out as a literal. */
const RESERVED_BITS = (~(CQMAP_VLD | CQMAP_PAG)) & 0x7FFFFFFF;

function enumeratedCases()
{
    let out = [];
    let BASE = 0x20;                            // an arbitrary but fixed base map index
    let add = (spec) => out.push(makeCase(spec));

    /* -- within a single page, both directions, both routine widths ------------------------- */
    for (let op of [0, 1, 2, 3]) {
        add({name: `within-page-op${op}`, entries: [{idx: BASE, page: dpage(3), valid: true}],
             probes: [{idx: BASE, size: 4}], op, ba: qaddr(BASE, 0), bc: 64});
    }

    /* -- crossing exactly one page boundary, discontiguous targets --------------------------- */
    for (let op of [0, 2]) {
        add({name: `page-crossing-op${op}`,
             entries: [{idx: BASE, page: dpage(9), valid: true}, {idx: BASE + 1, page: dpage(2), valid: true}],
             probes: [{idx: BASE, size: 4}, {idx: BASE + 1, size: 4}],
             op, ba: qaddr(BASE, PAGE - 16), bc: 64});
    }

    /* -- THREE map entries, discontiguous AND out of order: the case that kills the identity-map
          cheat.  Pages 40, 7, 23 are neither adjacent nor ascending. ------------------------- */
    for (let op of [0, 1, 2, 3]) {
        add({name: `three-entries-discontiguous-op${op}`,
             entries: [{idx: BASE, page: dpage(40), valid: true},
                       {idx: BASE + 1, page: dpage(7), valid: true},
                       {idx: BASE + 2, page: dpage(23), valid: true}],
             probes: [{idx: BASE, size: 4}, {idx: BASE + 1, size: 4}, {idx: BASE + 2, size: 4}],
             op, ba: qaddr(BASE, 0), bc: 3 * PAGE});
    }

    /* -- FOUR entries, discontiguous, starting mid-page so the first and last are partial ---- */
    for (let op of [0, 2]) {
        add({name: `four-entries-partial-ends-op${op}`,
             entries: [{idx: BASE, page: dpage(31), valid: true},
                       {idx: BASE + 1, page: dpage(5), valid: true},
                       {idx: BASE + 2, page: dpage(18), valid: true},
                       {idx: BASE + 3, page: dpage(11), valid: true}],
             probes: [{idx: BASE + 3, size: 4}],
             op, ba: qaddr(BASE, PAGE - 8), bc: 2 * PAGE + 24});
    }

    /* -- the single-page IDENTITY map: the shape that makes the cheat look correct.  Qbus page
          index == physical page number, so bypassing the map lookup entirely still "works". --- */
    add({name: "identity-mapped-single-page",
         entries: [{idx: DATA_PAGE0 + 1, page: DATA_PAGE0 + 1, valid: true}],
         probes: [{idx: DATA_PAGE0 + 1, size: 4}],
         op: 0, ba: qaddr(DATA_PAGE0 + 1, 0), bc: 128});

    /* -- unaligned start address and unaligned count: the BYTE loop, not the longword loop ---- */
    for (let off of [1, 2, 3]) {
        add({name: `unaligned-start-${off}-read`,
             entries: [{idx: BASE, page: dpage(13), valid: true}, {idx: BASE + 1, page: dpage(44), valid: true}],
             probes: [{idx: BASE, size: 4}],
             op: 0, ba: qaddr(BASE, PAGE - 8 + off), bc: 32});
        add({name: `unaligned-start-${off}-write`,
             entries: [{idx: BASE, page: dpage(14), valid: true}, {idx: BASE + 1, page: dpage(45), valid: true}],
             probes: [{idx: BASE, size: 4}],
             op: 2, ba: qaddr(BASE, PAGE - 8 + off), bc: 32});
    }
    for (let bc of [1, 3, 33, 0x1FF]) {
        add({name: `unaligned-count-${bc}-read`,
             entries: [{idx: BASE, page: dpage(21), valid: true}, {idx: BASE + 1, page: dpage(6), valid: true}],
             probes: [{idx: BASE, size: 4}], op: 0, ba: qaddr(BASE, 4), bc});
        add({name: `unaligned-count-${bc}-write`,
             entries: [{idx: BASE, page: dpage(22), valid: true}, {idx: BASE + 1, page: dpage(8), valid: true}],
             probes: [{idx: BASE, size: 4}], op: 2, ba: qaddr(BASE, 4), bc});
    }

    /* -- the word routines' own masking: an ODD address and an ODD count are both masked away -- */
    add({name: "odd-address-readW", entries: [{idx: BASE, page: dpage(17), valid: true}],
         probes: [{idx: BASE, size: 4}], op: 1, ba: qaddr(BASE, 9), bc: 32});
    add({name: "odd-count-writeW", entries: [{idx: BASE, page: dpage(19), valid: true}],
         probes: [{idx: BASE, size: 4}], op: 3, ba: qaddr(BASE, 8), bc: 33});
    add({name: "word-loop-unaligned-readW", entries: [{idx: BASE, page: dpage(25), valid: true},
             {idx: BASE + 1, page: dpage(4), valid: true}],
         probes: [{idx: BASE, size: 4}], op: 1, ba: qaddr(BASE, PAGE - 6), bc: 34});

    /* -- an INVALID map entry: first, and in the middle of a multi-page transfer ------------- */
    add({name: "invalid-entry-first-read",
         entries: [{idx: BASE, page: dpage(3), valid: false}],
         probes: [{idx: BASE, size: 4}], op: 0, ba: qaddr(BASE, 0), bc: 64});
    for (let op of [0, 1, 2, 3]) {
        add({name: `invalid-entry-middle-op${op}`,
             entries: [{idx: BASE, page: dpage(28), valid: true},
                       {idx: BASE + 1, page: dpage(9), valid: false},
                       {idx: BASE + 2, page: dpage(35), valid: true}],
             probes: [{idx: BASE + 1, size: 4}], op, ba: qaddr(BASE, 0), bc: 3 * PAGE});
    }
    add({name: "invalid-entry-middle-byteloop",
         entries: [{idx: BASE, page: dpage(29), valid: true}, {idx: BASE + 1, page: dpage(10), valid: false}],
         probes: [{idx: BASE, size: 4}], op: 0, ba: qaddr(BASE, PAGE - 7), bc: 23});

    /* -- a VALID entry pointing PAST the end of memory: SLAVE nxm, a different latch entirely -- */
    for (let op of [0, 3]) {
        add({name: `slave-nxm-op${op}`,
             entries: [{idx: BASE, page: dpage(12), valid: true}, {idx: BASE + 1, page: OOM_PAGE, valid: true}],
             probes: [{idx: BASE + 1, size: 4}], op, ba: qaddr(BASE, 0), bc: 2 * PAGE});
    }

    /* -- a PRE-EXISTING unresolved error: the lost-error bit, for both latch sites ----------- */
    add({name: "lost-error-master", dserPre: 0x80,
         entries: [{idx: BASE, page: dpage(3), valid: false}],
         probes: [{idx: BASE, size: 4}], op: 0, ba: qaddr(BASE, 0), bc: 32});
    add({name: "lost-error-slave", dserPre: 0x01,
         entries: [{idx: BASE, page: OOM_PAGE, valid: true}],
         probes: [{idx: BASE, size: 4}], op: 0, ba: qaddr(BASE, 0), bc: 32});

    /* -- MBR out of memory.  Map-register WRITES are cq_serr + a deferred mem_err with NO
          exception, which is exact and graded; the DMA then fails its entry fetch the same way.
          NO PROBES -- reading a map register here is the one excluded case (file header). ---- */
    add({name: "mbr-out-of-memory-write-and-dma", mbr: OOM_MBR,
         entries: [{idx: BASE, page: dpage(3), valid: true}],
         probes: [], op: 0, ba: qaddr(BASE, 0), bc: 64});
    add({name: "mbr-out-of-memory-dma-write", mbr: OOM_MBR,
         entries: [{idx: BASE, page: dpage(3), valid: true}],
         probes: [], op: 2, ba: qaddr(BASE, 0), bc: 64});

    /* -- MBR at each legal base: the map window really is MBR-relative ----------------------- */
    MAP_BASES.forEach((mbr, k) => {
        add({name: `mbr-base-${k}`, mbr,
             entries: [{idx: BASE + k, page: dpage(30 + k), valid: true}],
             probes: [{idx: BASE + k, size: 4}], op: 0, ba: qaddr(BASE + k, 0), bc: 96});
    });

    /* -- byte- and word-sized map-register WRITES, and byte/word read-backs ------------------ */
    for (let size of [1, 2]) {
        add({name: `map-reg-write-size${size}`,
             entries: [{idx: BASE, page: dpage(26), valid: true, size}],
             probes: [{idx: BASE, size: 4}], op: 0, ba: qaddr(BASE, 0), bc: 64});
    }
    add({name: "map-reg-readback-sizes",
         entries: [{idx: BASE, page: dpage(27), valid: true}],
         probes: [{idx: BASE, size: 4}, {idx: BASE, size: 2, off: 0}, {idx: BASE, size: 2, off: 2},
                  {idx: BASE, size: 1, off: 0}, {idx: BASE, size: 1, off: 1}, {idx: BASE, size: 1, off: 3}],
         op: 0, ba: qaddr(BASE, 0), bc: 64});

    /* -- two entries ALIASED to one physical page ------------------------------------------- */
    add({name: "aliased-entries",
         entries: [{idx: BASE, page: dpage(15), valid: true}, {idx: BASE + 1, page: dpage(15), valid: true}],
         probes: [{idx: BASE + 1, size: 4}], op: 2, ba: qaddr(BASE, PAGE - 16), bc: 64});

    /* -- a zero-length transfer: no map access at all, residual 0 ---------------------------- */
    add({name: "zero-length", entries: [{idx: BASE, page: dpage(3), valid: true}],
         probes: [{idx: BASE, size: 4}], op: 0, ba: qaddr(BASE, 0), bc: 0});

    /* -- a bus address ABOVE the CQBIC's 22-bit width: masked, not faulted.  QBMAMASK+1 is
          derived from the constant, so this case follows the width if it is ever corrected. --- */
    add({name: "bus-address-above-22-bits",
         entries: [{idx: BASE, page: dpage(37), valid: true}],
         probes: [{idx: BASE, size: 4}], op: 0, ba: ((QBMAMASK + 1) + qaddr(BASE, 0)) >>> 0, bc: 64});

    /* -- map entries carrying garbage in the RESERVED bits between CQMAP_PAG (20 bits) and
          CQMAP_VLD (bit 31).  qba_map_addr() masks the page with CQMAP_PAG, so those bits must be
          ignored entirely.  Measured: widening CQMAP_PAG past 20 bits SURVIVED a full run until
          this case existed, because every other case writes entries whose reserved bits are 0. -- */
    for (let op of [0, 2]) {
        add({name: `entry-reserved-bits-set-op${op}`,
             entries: [{idx: BASE, page: dpage(24), valid: true, extra: RESERVED_BITS},
                       {idx: BASE + 1, page: dpage(48), valid: true, extra: 0x00100000}],
             probes: [{idx: BASE, size: 4}],
             op, ba: qaddr(BASE, PAGE - 16), bc: 64});
    }

    /* -- the TOP of the Qbus address space: the map INDEX wraps (`(qblk << 2) & CQMAPAMASK`), so
          the page after the last one is governed by map entry 0, not by entry 8192. ------------- */
    add({name: "bus-address-wraps-map-index",
         entries: [{idx: IDX_MASK, page: dpage(33), valid: true},
                   {idx: IDX_MASK + 1, page: dpage(2), valid: true}],
         probes: [{idx: IDX_MASK, size: 4}],
         op: 0, ba: ((QBMAMASK + 1) - 16) >>> 0, bc: 64});

    /* -- a bus address near 2^32.  THIS is the case that makes `ba & QBMAMASK` observable at all:
          the map index, the page offset and MEAR all wrap at the same 4MB period, so masking a
          merely-too-large address changes nothing -- but an UNMASKED `ba + i` overflows 32 bits
          mid-transfer and lands on a completely different entry.  Measured: dropping the mask
          survived a full run until this case existed. ------------------------------------------- */
    add({name: "bus-address-near-32-bit-wrap",
         entries: [{idx: IDX_MASK, page: dpage(34), valid: true},
                   {idx: IDX_MASK + 1, page: dpage(6), valid: true}],
         probes: [{idx: IDX_MASK, size: 4}],
         op: 0, ba: 0xFFFFFFF0, bc: 64});
    add({name: "bus-address-near-32-bit-wrap-write",
         entries: [{idx: IDX_MASK, page: dpage(36), valid: true},
                   {idx: IDX_MASK + 1, page: dpage(20), valid: true}],
         probes: [{idx: IDX_MASK, size: 4}],
         op: 2, ba: 0xFFFFFFF0, bc: 64});

    /* -- the maximum transfer this file allows, page-aligned ---------------------------------- */
    add({name: "max-transfer",
         entries: [{idx: BASE, page: dpage(41), valid: true}, {idx: BASE + 1, page: dpage(1), valid: true},
                   {idx: BASE + 2, page: dpage(52), valid: true}],
         probes: [{idx: BASE, size: 4}], op: 2, ba: qaddr(BASE, 0), bc: MAXBC});

    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The randomized phase                                                                           *
 * ------------------------------------------------------------------------------------------- */

function randomCases(rnd, n)
{
    let out = [];
    for (let k = 0; k < n; k++) {
        let mbr = (rnd() < 0.04) ? OOM_MBR : pick(rnd, MAP_BASES);
        let badMbr = mbr === OOM_MBR;
        let base = 0x10 + rint(rnd, 0x1000);                  // a map index well inside the map
        let nEnt = 1 + rint(rnd, 4);
        let entries = [];
        /* Target pages are drawn and SHUFFLED, so a multi-entry transfer is scattered by
           construction rather than by luck -- the whole reason the map exists. */
        let picks = [];
        while (picks.length < nEnt) {
            let p = rint(rnd, DATA_NPAGE);
            if (picks.indexOf(p) < 0) picks.push(p);
        }
        for (let i = picks.length - 1; i > 0; i--) {
            let j = rint(rnd, i + 1);
            [picks[i], picks[j]] = [picks[j], picks[i]];
        }
        for (let i = 0; i < nEnt; i++) {
            let r = rnd();
            let valid = r > 0.12;
            let page = (r < 0.06) ? OOM_PAGE : dpage(picks[i]);
            entries.push({idx: base + i, page, valid, size: pick(rnd, [4, 4, 4, 4, 2, 1]),
                          extra: (rnd() < 0.15) ? (rint(rnd, 0x800) << 20) & RESERVED_BITS : 0});
        }
        let off = pick(rnd, [0, 1, 2, 3, 4, 8, PAGE - 8, PAGE - 3, PAGE - 1, rint(rnd, PAGE)]);
        let ba = qaddr(base, off);
        if (rnd() < 0.05) ba = (ba + (QBMAMASK + 1)) >>> 0;   // above the 22-bit width
        let maxSpan = (nEnt * PAGE) - off;
        let bc = rint(rnd, Math.max(1, Math.min(MAXBC, maxSpan + PAGE)) + 1);
        out.push(makeCase({
            name: `random#${k}`,
            mbr, entries,
            probes: badMbr ? [] : [{idx: base + rint(rnd, nEnt), size: pick(rnd, [4, 4, 2, 1])}],
            dserPre: pick(rnd, [0, 0, 0, 0x80, 0x01, 0x20]),
            op: rint(rnd, 4), ba, bc
        }));
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The oracle                                                                                     *
 * ------------------------------------------------------------------------------------------- */

const CASE_MARK = "@@QC";
const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/;
const QDMA_RE = /^QDMA\s+(\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)/;
const NREG_VALS = NPROBE + 2 + 7;      // R0..R6, PC, PSL, SCR, DSER, MEAR, SEAR, MBR, BTO, MEMERR

/**
 * seedScript() -- the ONE-TIME memory seeding, applied identically to both engines before any
 * case runs.  Every longword of the DMA-visible region and of the device buffer's staging area
 * gets a distinct pseudo-random value, so no transfer can appear correct by landing on a constant.
 * Deterministic (no PRNG state shared with the case generator) so the two engines cannot diverge.
 */
function seedValue(addr) { return (Math.imul(addr >>> 2, 2654435761) ^ 0x5DEECE66) | 0; }

function seedTargets()
{
    let t = [];
    for (let a = DATA_BASE; a < DATA_BASE + DATA_LEN; a += 4) t.push(a);
    for (let a = STAGE; a < STAGE + QDMA_MAXBC; a += 4) t.push(a);
    return t;
}

function buildScript(cases)
{
    let L = ["set cpu " + (MEMSIZE >> 20) + "m", "set cpu simhalt"];
    L.push(`deposit -l ${hex(MAP_SPAN[0])}:${hex(MAP_SPAN[1] - 4)} 0`);
    for (let a of seedTargets()) L.push(`deposit -l ${hex(a)} ${hex(seedValue(a))}`);
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        L.push("deposit cpu memerr 0");
        L.push("deposit sysd bto 0");
        L.push(`deposit qba dser ${hex(c.dserPre, 2)}`);
        L.push(`deposit qba mear ${hex(c.mearPre, 4)}`);
        L.push(`deposit qba sear ${hex(c.searPre, 5)}`);
        L.push("deposit qba mbr 0");
        if (c.mbr + CQMAPSIZE <= MEMSIZE) {
            for (let i of c.zeroIdx) L.push(`deposit -l ${hex((c.mbr + i * 4) >>> 0)} 0`);
        }
        for (let d of c.deposits) L.push(`deposit -l ${hex(d.addr)} ${hex(d.val)}`);
        for (let i = 0; i < c.code.length; i += 4) {
            let v = (c.code[i] | (c.code[i + 1] << 8) | (c.code[i + 2] << 16) | (c.code[i + 3] << 24)) >>> 0;
            L.push(`deposit -l ${hex((R_CODE + i) >>> 0)} ${hex(v)}`);
        }
        L.push(`deposit PSL ${hex(PSL_RUN)}`);
        L.push(`deposit PC ${hex(R_CODE)}`);
        L.push(`step ${c.nInstr}`);
        L.push("examine -h R0,R1,R2,R3,R4,R5,R6,PC,PSL");
        L.push("examine -h qba scr");
        L.push("examine -h qba dser");
        L.push("examine -h qba mear");
        L.push("examine -h qba sear");
        L.push("examine -h qba mbr");
        L.push("examine -h sysd bto");
        L.push("examine -h cpu memerr");
        L.push(`show qba qdma=${(c.op).toString(16)}:${hex(c.ba)}:${(c.bc).toString(16)}:${hex(c.sa)}`);
        for (let r of c.ranges) L.push(`examine -h ${hex(r[0])}:${hex(r[1])}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

function runOracle(simh, cases, scratch)
{
    let script = buildScript(cases);
    let out = runSimh(simh, script, path.join(scratch, "qdmadiff-batch.ini"));
    if (!out.split("\n").some((l) => QDMA_RE.test(l))) {
        throw new Error("this simulator produced no SHOW QBA QDMA= result line -- rebuild it WITH patch\n" +
            "0007-qbus-dma-differential-support.patch (rm -rf the build DEST first; build.sh REUSES it).\n" +
            "This test grades against REAL SIMH; it has no fixture fallback and does not skip.");
    }
    let lines = out.split("\n");
    let results = new Map();
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\d+)"));
        if (!m) { i++; continue; }
        let idx = +m[1];
        i++;
        let vals = [], qdma = null;
        while (i < lines.length) {
            if (lines[i].indexOf(CASE_MARK) >= 0) break;
            let q = lines[i].match(QDMA_RE);
            if (q) {
                qdma = {resid: parseInt(q[2], 16) | 0, dser: parseInt(q[3], 16) | 0,
                        mear: parseInt(q[4], 16) | 0, sear: parseInt(q[5], 16) | 0,
                        ipc: parseInt(q[6], 16) | 0, memerr: parseInt(q[7], 16) | 0};
            } else {
                let vm = lines[i].match(VALUE_RE);
                if (vm) vals.push(parseInt(vm[2], 16) | 0);
            }
            i++;
        }
        results.set(idx, {vals, qdma});
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * The JS side                                                                                    *
 * ------------------------------------------------------------------------------------------- */

function seedMachine(m)
{
    let {bus} = m;
    for (let a = MAP_SPAN[0]; a < MAP_SPAN[1]; a += 4) bus.setLong(a, 0);
    for (let a of seedTargets()) bus.setLong(a, seedValue(a));
}

function runCaseJS(m, c)
{
    let {bus, cpu, cqbic, buf} = m;
    cpu.exc.memErr = 0;
    cpu.exc.sscBto = 0;
    cpu.exc.cqDser = c.dserPre | 0;
    cpu.exc.cqMear = c.mearPre | 0;
    cqbic.sear = c.searPre | 0;
    cqbic.mbr = 0;
    if (c.mbr + CQMAPSIZE <= MEMSIZE) {
        for (let i of c.zeroIdx) bus.setLong((c.mbr + i * 4) >>> 0, 0);
    }
    for (let d of c.deposits) bus.setLong(d.addr, d.val);
    for (let i = 0; i < c.code.length; i++) bus.setByte((R_CODE + i) >>> 0, c.code[i]);
    cpu.psl = PSL_RUN | 0;
    cpu.regs[15] = R_CODE;

    let stop = null;
    for (let k = 0; k < c.nInstr; k++) {
        try { cpu.stepCPU(1); }
        catch (e) {
            if (e instanceof VAXStop) { stop = e; break; }
            throw new Error(`qdmadiff.js: case "${c.name}" threw at instruction ${k}: ${e && e.message}`);
        }
    }
    let vals = [];
    for (let r = 0; r < NPROBE; r++) vals.push(cpu.regs[r] | 0);
    vals.push(cpu.regs[15] | 0, cpu.psl | 0);
    vals.push(cqbic.readReg(0) | 0);                        // SCR, through its own read path
    vals.push(cpu.exc.cqDser | 0, cpu.exc.cqMear | 0, cqbic.sear | 0, cqbic.mbr | 0);
    vals.push(cpu.exc.sscBto | 0, cpu.exc.memErr | 0);

    /* The DMA.  The staging in and out mirrors patch 0007's own harness exactly -- it is the C
       command's buffer plumbing, not part of the code under test. */
    buf.fill(QDMA_FILL);
    if (c.op >= 2) for (let i = 0; i < c.bc; i++) buf[i] = bus.getByte((c.sa + i) >>> 0) & 0xFF;
    let resid;
    switch (c.op) {
    case 0: resid = cqbic.mapReadB(c.ba, c.bc, buf); break;
    case 1: resid = cqbic.mapReadW(c.ba, c.bc, buf); break;
    case 2: resid = cqbic.mapWriteB(c.ba, c.bc, buf); break;
    default: resid = cqbic.mapWriteW(c.ba, c.bc, buf); break;
    }
    if (c.op < 2) for (let i = 0; i < c.sbc; i++) bus.setByte((c.sa + i) >>> 0, buf[i]);

    let qdma = {resid: resid | 0, dser: cpu.exc.cqDser | 0, mear: cpu.exc.cqMear | 0,
                sear: cqbic.sear | 0, ipc: 0, memerr: cpu.exc.memErr | 0};
    let mem = [];
    for (let r of c.ranges) for (let a = r[0]; a <= r[1]; a += 4) mem.push(bus.getLong(a) | 0);
    return {vals: vals.concat(mem), qdma, stop};
}

/* ------------------------------------------------------------------------------------------- *
 * Comparison                                                                                     *
 * ------------------------------------------------------------------------------------------- */

const VAL_NAMES = (function() {
    let n = [];
    for (let r = 0; r < NPROBE; r++) n.push("R" + r);
    n.push("PC", "PSL", "SCR", "DSER", "MEAR", "SEAR", "MBR", "BTO", "MEMERR");
    return n;
})();

function compareCase(c, js, sr)
{
    let bad = [];
    let tag = `case#${c.index} "${c.name}" op=${c.op} ba=0x${hex(c.ba)} bc=0x${hex(c.bc, 4)}`;
    if (js.stop) bad.push(`${tag}: JS stopped early (${js.stop.message})`);
    if (!sr || !sr.qdma) { bad.push(`${tag}: the oracle produced no QDMA line`); return bad; }
    let want = NREG_VALS + c.nMemVals;
    if (sr.vals.length !== want) {
        bad.push(`${tag}: oracle returned ${sr.vals.length} values, expected ${want}`);
        return bad;
    }
    for (let i = 0; i < NREG_VALS; i++) {
        if ((js.vals[i] | 0) !== (sr.vals[i] | 0)) {
            bad.push(`${tag}: ${VAL_NAMES[i]} js=${hex(js.vals[i])} simh=${hex(sr.vals[i])}`);
        }
    }
    for (let k of ["resid", "dser", "mear", "sear", "ipc", "memerr"]) {
        if ((js.qdma[k] | 0) !== (sr.qdma[k] | 0)) {
            bad.push(`${tag}: QDMA ${k} js=${hex(js.qdma[k])} simh=${hex(sr.qdma[k])}`);
        }
    }
    /* Memory, byte for byte -- reported by ADDRESS, so a divergence names the page it is in. */
    let i = NREG_VALS, nBad = 0;
    for (let r of c.ranges) {
        for (let a = r[0]; a <= r[1]; a += 4, i++) {
            if ((js.vals[i] | 0) !== (sr.vals[i] | 0)) {
                if (nBad++ < 8) bad.push(`${tag}: mem[${hex(a)}] js=${hex(js.vals[i])} simh=${hex(sr.vals[i])}`);
            }
        }
    }
    if (nBad > 8) bad.push(`${tag}: ... and ${nBad - 8} more diverging longwords`);
    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors -- ABSOLUTE, derived from the case list, and they FAIL the run (rule 4)         *
 * ------------------------------------------------------------------------------------------- */

/**
 * classify(c) -- what a case actually exercises, computed from the case's OWN spec (its declared
 * map entries and transfer), never from cqbic.js.  A floor that asked the code under test what it
 * covered would certify itself.
 */
function classify(c)
{
    let f = new Set();
    let ba0 = (c.ba & QBMAMASK) >>> 0, bc0 = c.bc;
    if (c.op === 1 || c.op === 3) { ba0 = ba0 & ~1; bc0 = bc0 & ~1; }
    f.add(c.op < 2 ? "direction:read" : "direction:write");
    f.add(`routine:op${c.op}`);
    if (bc0 === 0) f.add("zero-length");
    if (((ba0 | bc0) & 3) !== 0) f.add("loop:byte-or-word"); else if (bc0 > 0) f.add("loop:longword");
    if ((c.ba >>> 0) > QBMAMASK) f.add("bus-address-above-22-bits");
    if ((c.ba >>> 0) > 0xFFFF0000) f.add("bus-address-near-32-bit-wrap");
    if (c.idxHi > IDX_MASK) f.add("map-index-wraps");
    if ((ba0 & 3) !== 0) f.add("unaligned-start");
    if ((bc0 & 3) !== 0) f.add("unaligned-count");
    if (c.mbr !== 0) f.add("mbr-nonzero");
    if (c.mbr + CQMAPSIZE > MEMSIZE) f.add("mbr-out-of-memory");
    if (c.dserPre !== 0) f.add("pre-existing-dser");
    for (let e of c.entries) {
        if (e.size === 1) f.add("map-reg-write:byte");
        else if (e.size === 2) f.add("map-reg-write:word");
        else f.add("map-reg-write:long");
    }
    for (let p of c.probes) f.add(`map-reg-read:${p.size === 4 ? "long" : p.size === 2 ? "word" : "byte"}`);
    if (c.entries.some((e) => (e.extra || 0) !== 0)) f.add("entry-reserved-bits-set");

    /* Which entries the transfer actually walks, and what they say. */
    let want = new Map();
    for (let e of c.entries) want.set(entIdx(e.idx), e);
    let walked = [], sawInvalid = false, sawOom = false, sawMissing = false;
    for (let i = c.idxLo; i >= 0 && i <= c.idxHi; i++) {
        let e = want.get(entIdx(i));
        if (!e) { sawMissing = true; break; }
        if (!e.valid) { sawInvalid = true; break; }
        if (e.page >= MEMSIZE / PAGE) { sawOom = true; break; }
        walked.push(e.page);
    }
    if (c.mbr + CQMAPSIZE <= MEMSIZE) {
        if (sawInvalid || sawMissing) f.add("invalid-entry");
        if (sawOom) f.add("slave-nxm");
        if (c.dserPre !== 0 && (sawInvalid || sawMissing || sawOom)) f.add("lost-error");
    } else if (bc0 > 0) {
        f.add("map-unreachable");
    }
    if (walked.length >= 2) f.add("page-crossing");
    if (walked.length === 1 && !sawInvalid && !sawOom && !sawMissing) f.add("within-page");
    if (walked.length >= 3) {
        f.add("three-entries");
        let contiguousAscending = walked.every((p, k) => k === 0 || p === walked[k - 1] + 1);
        if (!contiguousAscending) f.add("three-entries-discontiguous");
    }
    if (sawInvalid || sawOom || sawMissing) f.add("partial-transfer");
    else if (bc0 > 0) f.add("full-transfer");
    return f;
}

/* Every floor below must be met at least once.  These are ABSOLUTE counts: they do not scale with
   --cases, so shrinking the run to the minimum cannot quietly shrink the coverage requirement.

   ONE FLOOR HERE CANNOT CATCH ANYTHING, AND SAYING SO IS THE POINT (standing rule 12).
   `bus-address-above-22-bits` and `bus-address-near-32-bit-wrap` exercise a bus address outside the
   CQBIC's 22-bit width, and REMOVING `ba & QBMAMASK` from the shipped routines was injected and
   SURVIVED both.  That is not a coverage hole -- it is a proof that the mask is redundant: every
   consumer of the bus address reduces it modulo 4MB or modulo 512 (the map index is
   `((qa >> 9) << 2) & CQMAPAMASK`, the offset is `qa & VA_M_OFF`, and MEAR is `(qa >> 9) & 0x1FFF`,
   whose 13 bits span exactly the same 4MB), and 2^32 is itself a multiple of 4MB, so a uint32
   `ba + i` that wraps lands on the same entry and the same offset as the masked one.  The mask is
   kept because it is what vax_io.c does; the cases are kept because the map INDEX wrap they also
   exercise (`map-index-wraps`) IS observable and IS caught. */
const FLOORS = [
    "direction:read", "direction:write",
    "routine:op0", "routine:op1", "routine:op2", "routine:op3",
    "loop:byte-or-word", "loop:longword",
    "within-page", "page-crossing", "three-entries", "three-entries-discontiguous",
    "unaligned-start", "unaligned-count", "zero-length",
    "bus-address-above-22-bits", "bus-address-near-32-bit-wrap", "map-index-wraps",
    "entry-reserved-bits-set",
    "invalid-entry", "slave-nxm", "lost-error", "map-unreachable",
    "partial-transfer", "full-transfer", "pre-existing-dser",
    "mbr-nonzero", "mbr-out-of-memory",
    "map-reg-write:long", "map-reg-write:word", "map-reg-write:byte",
    "map-reg-read:long", "map-reg-read:word", "map-reg-read:byte"
];

function tallyCoverage(cases)
{
    let counts = new Map();
    for (let c of cases) for (let f of classify(c)) counts.set(f, (counts.get(f) || 0) + 1);
    return counts;
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck: mutations that PERTURB the shipped path (standing rule 11), each of which must      *
 * (a) demonstrably change JS behaviour and (b) then be caught by the comparison.  Requirement (a)   *
 * is the guard against the failure rule 11 names: a mutation that substitutes its own copy of       *
 * already-broken code changes nothing and still prints CAUGHT.  Here a mutation that changes         *
 * nothing is reported INERT and FAILS the run.                                                       *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = [
    {
        name: "map-lookup-bypassed (the bus address used as a physical address)",
        apply(P) {
            let orig = P.mapAddr;
            P.mapAddr = function(qa) {
                let ok = orig.call(this, qa);
                if (ok) this.mapMA = (qa >>> 0) & (MEMSIZE - 1);
                return ok;
            };
        }
    },
    {
        name: "page-offset-bits dropped from the translated address",
        apply(P) {
            let orig = P.mapAddr;
            P.mapAddr = function(qa) {
                let ok = orig.call(this, qa);
                if (ok) this.mapMA = (this.mapMA & ~VA_M_OFF) >>> 0;
                return ok;
            };
        }
    },
    {
        name: "map entry valid bit ignored",
        apply(P) {
            let orig = P.mapEntry;
            P.mapEntry = function(qmma) { return (orig.call(this, qmma) | CQMAP_VLD) | 0; };
        }
    },
    {
        name: "residual count always reported as 0",
        apply(P) {
            for (let k of ["mapReadB", "mapReadW", "mapWriteB", "mapWriteW"]) {
                let orig = P[k];
                P[k] = function(ba, bc, buf) { orig.call(this, ba, bc, buf); return 0; };
            }
        }
    },
    {
        name: "byte order reversed within each longword/word",
        apply(P) {
            let sl = P.splitLong, jl = P.joinLong, sw = P.splitWord, jw = P.joinWord;
            let bsw32 = (v) => ((v >>> 24) | ((v >>> 8) & 0xFF00) | ((v << 8) & 0xFF0000) | (v << 24)) | 0;
            let bsw16 = (v) => (((v >>> 8) & 0xFF) | ((v & 0xFF) << 8)) & 0xFFFF;
            P.splitLong = function(dat, buf, j) { return sl.call(this, bsw32(dat), buf, j); };
            P.joinLong = function(buf, j) { return bsw32(jl.call(this, buf, j)); };
            P.splitWord = function(dat, buf, j) { return sw.call(this, bsw16(dat), buf, j); };
            P.joinWord = function(buf, j) { return bsw16(jw.call(this, buf, j)); };
        }
    },
    {
        name: "map base register (MBR) ignored when locating an entry",
        apply(P) {
            let orig = P.mapEntryAddr;
            P.mapEntryAddr = function(qa) { return (orig.call(this, qa) - (this.mbr >>> 0)) >>> 0; };
        }
    },
    {
        name: "slave NXM latched as a master NXM (wrong DSER bit and wrong error register)",
        apply(P) {
            P.cqSerr = function(pa) { this.exc.cqMerr(pa); };
        }
    },
    {
        name: "map register write not reflected to memory",
        apply(P) {
            let orig = P.mapRegWrite;
            P.mapRegWrite = function(addr, val, lnt) { orig.call(this, addr, val, lnt); return true; };
            /* Compose, then undo the store: the ORIGINAL still runs (so its error latching is
               unchanged), and only the reflected value is reverted -- a perturbation, not a
               replacement. */
            let origW = P.mapRegWrite;
            P.mapRegWrite = function(addr, val, lnt) {
                let ma = (((addr >>> 0) & (CQMAPSIZE - 1)) + (this.mbr >>> 0)) >>> 0;
                let before = (ma < this.memSize) ? this.bus.getLong(ma) : 0;
                let r = origW.call(this, addr, val, lnt);
                if (ma < this.memSize) this.bus.setLong(ma, before);
                return r;
            };
        }
    }
];

function snapshotProto()
{
    let P = CQBICVAX.prototype, saved = {};
    for (let k of Object.getOwnPropertyNames(P)) saved[k] = P[k];
    return saved;
}

function restoreProto(saved)
{
    let P = CQBICVAX.prototype;
    for (let k of Object.keys(saved)) P[k] = saved[k];
}

/* ------------------------------------------------------------------------------------------- *
 * Heap bound (standing rule 14) -- ABSOLUTE, and it fails the run                                 *
 * ------------------------------------------------------------------------------------------- */

/* Measured, not guessed: a full 292-case --selfcheck run (nine passes over every case) peaks at
   ~24 MB, because the machine, the bus and the 4KB device buffer are built ONCE and every case
   reuses them.  The cap is ABSOLUTE and does not scale with --cases (standing rule 4): a run that
   starts allocating per case will blow it long before it can OOM-kill anything. */
const HEAP_CAP_MB = 128;
let peakHeapMB = 0;

function sampleHeap()
{
    let mb = process.memoryUsage().heapUsed / (1024 * 1024);
    if (mb > peakHeapMB) peakHeapMB = mb;
    return mb;
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                           *
 * ------------------------------------------------------------------------------------------- */

const MIN_CASES_FLOOR = 120;

function parseArgs(argv)
{
    let a = {simh: null, cases: 240, seed: 0x51DE, selfcheck: false};
    for (let i = 2; i < argv.length; i++) {
        if (argv[i] === "--simh") a.simh = argv[++i];
        else if (argv[i] === "--cases") a.cases = +argv[++i];
        else if (argv[i] === "--seed") a.seed = +argv[++i];
        else if (argv[i] === "--selfcheck") a.selfcheck = true;
        else if (argv[i] === "--ehkaa") i++;                    // accepted and ignored: gate uniformity
    }
    return a;
}

function main()
{
    assertLayout();
    let args = parseArgs(process.argv);
    console.log("VAX Qbus DMA / CQBIC scatter-gather map differential test (pcjsvax-e22)");

    if (args.cases < MIN_CASES_FLOOR) {
        console.error(`FAIL: --cases ${args.cases} is below MIN_CASES_FLOOR ${MIN_CASES_FLOOR}; ` +
            "a run this small cannot cover the randomized phase and must not be allowed to pass.");
        process.exit(1);
    }

    let simh = findSimh(args.simh);
    /* Removed on EVERY exit, not just the happy one.  Removing it only at the end of main() leaked
       a multi-megabyte .ini per FAILING run, which is precisely the hazard HANDOFF.md section 4
       records for romdiff (a broken run once wrote 9.5 GB and filled the root filesystem). */
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "qdmadiff-"));
    process.on("exit", () => fs.rmSync(scratch, {recursive: true, force: true}));
    let rnd = mulberry32(args.seed);

    let cases = enumeratedCases().concat(randomCases(rnd, args.cases));
    cases.forEach((c, i) => { c.index = i; });
    console.log(`simh:  ${simh}`);
    console.log(`cases: ${cases.length} (${enumeratedCases().length} enumerated + ${args.cases} randomized), seed 0x${hex(args.seed, 4)}`);

    /* Exclusion guard, by name (see the file header). */
    let excluded = cases.filter((c) => c.readsMapWithBadMbr);
    if (excluded.length) {
        console.error("FAIL: these cases READ a map register while MBR is out of memory, which is the " +
            "one branch cqbic.js cannot reproduce exactly (ssc_bto), and which this file therefore " +
            "excludes by construction:");
        for (let c of excluded) console.error(`        case#${c.index} "${c.name}"`);
        process.exit(1);
    }
    console.log("EXCLUDED BY NAME, not graded: cqmap_rd()'s out-of-memory READ branch (MBR past the end " +
        "of memory + a CPU read of a map register).\n" +
        "                                 See the file header; the WRITE half of the same branch IS graded.");

    /* Coverage, computed BEFORE the run so a failure is reported even if the comparison fails first. */
    let counts = tallyCoverage(cases);
    let missing = FLOORS.filter((f) => !(counts.get(f) >= 1));
    console.log("\ncoverage floors (absolute -- these do not scale with --cases):");
    for (let f of FLOORS) console.log(`  ${(counts.get(f) || 0) >= 1 ? "ok  " : "MISS"} ${f.padEnd(32)} ${counts.get(f) || 0}`);
    if (missing.length) {
        console.error(`\nFAIL: ${missing.length} coverage floor(s) not met: ${missing.join(", ")}`);
        process.exit(1);
    }

    let m = makeMachine();
    seedMachine(m);
    sampleHeap();

    console.log("\nrunning the oracle ...");
    let t0 = Date.now();
    let oracle = runOracle(simh, cases, scratch);
    console.log(`oracle done in ${((Date.now() - t0) / 1000).toFixed(1)}s`);

    let jsBase = new Map();
    let bad = [], nReached = 0;
    for (let c of cases) {
        let js = runCaseJS(m, c);
        jsBase.set(c.index, js);
        sampleHeap();
        let sr = oracle.get(c.index);
        if (!sr) { bad.push(`case#${c.index} "${c.name}": the oracle never reached this case`); continue; }
        nReached++;
        bad.push(...compareCase(c, js, sr));
    }
    if (nReached !== cases.length) {
        console.error(`FAIL: only ${nReached} of ${cases.length} cases reached comparison.`);
    }
    if (bad.length) {
        console.error(`\nFAIL: ${bad.length} divergence(s) (seed 0x${hex(args.seed, 4)}):`);
        for (let b of bad.slice(0, 60)) console.error("  " + b);
        if (bad.length > 60) console.error(`  ... and ${bad.length - 60} more`);
        process.exit(1);
    }

    let heapMB = sampleHeap();
    console.log(`\nMATCH over all ${cases.length} cases: transferred bytes, residual counts, map register ` +
        "read-backs, DSER/MEAR/SEAR, ssc_bto and mem_err all identical to real SIMH.");
    console.log(`peak heap: ${peakHeapMB.toFixed(1)} MB (absolute cap ${HEAP_CAP_MB} MB, current ${heapMB.toFixed(1)} MB)`);
    if (peakHeapMB > HEAP_CAP_MB) {
        console.error(`FAIL: peak heap ${peakHeapMB.toFixed(1)} MB exceeded the absolute cap of ${HEAP_CAP_MB} MB.`);
        process.exit(1);
    }

    if (args.selfcheck) {
        if (!selfcheck(m, cases, oracle, jsBase)) process.exit(1);
    }

    console.log("\nPASS: a Qbus device DMAs through the CQBIC scatter-gather map exactly as real SIMH does.");
}

/**
 * selfcheck()
 *
 * For each mutation: re-run the SAME cases in JS with the mutation applied, and require BOTH
 *   (a) at least one observable differs from the un-mutated JS run -- the mutation really did
 *       perturb the shipped path, so a "CAUGHT" cannot be an artifact of an inert edit, and
 *   (b) the comparison against the oracle now FAILS.
 * A mutation meeting only (a) SURVIVED (a coverage hole); one meeting neither is INERT (a broken
 * mutation, which standing rule 11 says is worse than no mutation at all).  Both fail the run.
 */
function selfcheck(m, cases, oracle, jsBase)
{
    console.log("\n--selfcheck: deliberate defects, each of which must PERTURB and then be CAUGHT");
    let saved = snapshotProto();
    let ok = true;
    let identityCase = cases.find((c) => c.name === "identity-mapped-single-page");
    let discontigCase = cases.find((c) => c.name === "three-entries-discontiguous-op0");
    if (!identityCase || !discontigCase) throw new Error("qdmadiff.js: the named cheat cases are missing");

    for (let mut of MUTATIONS) {
        restoreProto(saved);
        mut.apply(CQBICVAX.prototype);
        seedMachine(m);
        let perturbed = false, caught = 0;
        for (let c of cases) {
            let js = runCaseJS(m, c);
            let base = jsBase.get(c.index);
            if (!perturbed) {
                if (js.vals.length !== base.vals.length) perturbed = true;
                else {
                    for (let i = 0; i < js.vals.length && !perturbed; i++) if (js.vals[i] !== base.vals[i]) perturbed = true;
                    for (let k of ["resid", "dser", "mear", "sear", "memerr"]) if (js.qdma[k] !== base.qdma[k]) perturbed = true;
                }
            }
            if (compareCase(c, js, oracle.get(c.index)).length) caught++;
        }
        sampleHeap();
        let verdict = !perturbed ? "INERT" : (caught ? "CAUGHT" : "SURVIVED");
        console.log(`  ${verdict.padEnd(9)} ${mut.name}${caught ? ` (${caught} case(s) diverged)` : ""}`);
        if (verdict !== "CAUGHT") ok = false;
    }

    /* The item-specific assertion: the identity-map cheat is invisible to a single-page test and
       is killed by the discontiguous one.  A suite that only says "CAUGHT" cannot prove that. */
    /**
     * ATTRIBUTING THE CATCH TO ONE CASE, WITHOUT LYING ABOUT IT.  Memory carries across cases on
     * BOTH engines (one machine, one SIMH process), so neither obvious method works:
     *   - running the target case in isolation compares it against an oracle observation taken
     *     after every earlier case had already written memory, and
     *   - running the whole list mutated lets the EARLIER cases' wrong writes corrupt memory, so
     *     the target case then diverges for a reason that is not its own.
     * Both were tried; both reported the easy case "catching" a cheat it cannot see.  So: replay
     * every earlier case UNMUTATED (memory now matches the oracle's history exactly), mutate, run
     * ONLY the target case, and restore.  That measures what the target case alone can detect.
     */
    let cheatProbe = (target) => {
        restoreProto(saved);
        seedMachine(m);
        let n = -1;
        for (let c of cases) {
            if (c.index === target.index) {
                MUTATIONS[0].apply(CQBICVAX.prototype);
                let js = runCaseJS(m, c);
                n = compareCase(c, js, oracle.get(c.index)).length;
                restoreProto(saved);
            } else {
                runCaseJS(m, c);
            }
        }
        return n;
    };
    let identBad = cheatProbe(identityCase);
    let discBad = cheatProbe(discontigCase);
    restoreProto(saved);
    seedMachine(m);
    console.log(`  ${identBad === 0 ? "ok  " : "BAD "} the identity-map cheat SURVIVES "identity-mapped-single-page" (${identBad} divergence(s))`);
    console.log(`  ${discBad > 0 ? "ok  " : "BAD "} the identity-map cheat DIES on "three-entries-discontiguous-op0" (${discBad} divergence(s))`);
    if (identBad !== 0) {
        console.error("FAIL: the single-page identity case caught the cheat, so it no longer demonstrates " +
            "why the discontiguous floor exists -- the floor's justification has silently changed.");
        ok = false;
    }
    if (discBad === 0) {
        console.error("FAIL: the discontiguous case did NOT catch the identity-map cheat. That case is the " +
            "entire reason coverage floor 'three-entries-discontiguous' exists.");
        ok = false;
    }
    if (!ok) console.error("\nFAIL: --selfcheck did not clear.");
    return ok;
}

main();
