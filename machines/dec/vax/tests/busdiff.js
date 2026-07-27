/**
 * @fileoverview Differential test: VAX physical memory/bus vs. the real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * A randomized differential test of BusVAX/MemoryVAX against a REAL, EXECUTED Open SIMH
 * microvax3900 binary.  There are no fixtures and no recorded golden files: every run builds a
 * command stream, launches the simulator, and compares its actual output.  If the binary is
 * missing the test FAILS -- it never silently degrades into self-comparison.
 *
 *      node machines/dec/vax/tests/busdiff.js [--ops N] [--seed S] [--simh PATH]
 *
 * The simulator is located via --simh, then $SIMH_BIN, then ../pcjs-vax/open-simh/BIN/microvax3900
 * relative to this repository.
 *
 * HOW SIMH IS DRIVEN, AND WHAT THAT MEANS FOR THE COMPARISON
 * ----------------------------------------------------------
 * SIMH's console is driven with EXAMINE/DEPOSIT, one command per line of a generated .ini file.
 * Read scp.c and vax_sys.c before changing anything here, because the console has semantics of its
 * own that are NOT the memory system's semantics:
 *
 *   - vax_sys.c fprint_sym()/parse_sym() give us sizes: -b = 1 byte, -w = 2, -l = 4 (the CPU's
 *     default).  There is no quadword switch, so a quadword is two -l commands, which is exactly
 *     how BusVAX.getQuad()/setQuad() decompose it.
 *   - The memory system itself is BYTE granular here: cpu_ex()/cpu_dep() (vax_cpu.c:3399,3421) do
 *     ReadB/WriteB one byte at a time, and scp.c composes/decomposes the multi-byte value
 *     LITTLE-ENDIAN.  So SIMH is ground truth for byte content and byte order; it is not itself
 *     performing a native longword bus cycle.
 *   - cpu_ex()/cpu_dep() mask with PAMASK (30 bits, vax_defs.h:137).  THIS IS THE POINT OF THE
 *     WHOLE ITEM: `examine 80001000` reads physical 0x1000.  S0 addresses are generated
 *     deliberately and heavily below; see the coverage assertions at the end.
 *   - scp.c get_aval() (line 10110) pre-zeroes its value buffer and BREAKS at the first SCPE_NXM,
 *     returning an error only if the FIRST byte failed.  So an examine that starts in memory and
 *     runs off the end of it reports a zero-filled short read, not an error.  dep_addr() (10184)
 *     instead stops AND errors at the first failing byte, leaving a partial write behind.
 *     Both behaviors are reproduced on the JS side and both are compared.
 *
 * WHAT IS ASSERTED, PER READ OP
 * -----------------------------
 *   (1) A byte-at-a-time model of SCP's own rules, driven through BusVAX.getByteDirect(), must
 *       equal SIMH EXACTLY -- same value, same error/no-error -- for 100% of read ops.
 *   (2) The NATIVE multi-byte accessor (getWord/getLong/getQuad) must equal SIMH's value whenever
 *       no byte of the access hit non-existent memory, and must RAISE A FAULT whenever SIMH
 *       refused or short-read the access.  This is what grades the shift/mask stitching in
 *       memory.js and the block-straddle fallbacks in bus.js.
 *
 * Writes are performed on the JS side with the native accessor (setByte/setWord/setLong/setQuad)
 * and on the SIMH side with the corresponding deposit(s); every byte ever written is then read
 * back from BOTH at the end of the run, so no write can pass unverified.
 *
 * SELF-CHECK
 * ----------
 * --selfcheck re-runs a short pass with a deliberate defect injected -- a bus that indexes blocks
 * with a raw negative address, a block-straddle boundary that is off by one, and a longword
 * accessor with its byte order reversed -- and fails if the differential does NOT catch each one.
 * A test that cannot fail proves nothing.
 *
 * A mutation that was TRIED AND REJECTED, because it is worth knowing: computing the block index
 * as `(addr >> shift) & nBlockMask` from a raw negative int32, instead of `(addr & PAMASK) >>>
 * shift`, is not actually a bug.  The wrap mask re-selects exactly bits 13..29, discarding the
 * sign extension, so the two are provably identical over the whole 32-bit range.  On a masked
 * PHYSICAL bus, JavaScript is far more forgiving about signed addresses than folklore suggests --
 * which is precisely why defines.js states the convention in terms of the operations that are
 * genuinely unsafe (>>, relational compares, arithmetic, indexing) and singles out the 32-bit
 * VIRTUAL addresses of the MMU, where no mask can rescue you, as the place it will actually bite.
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CDGVAX from "../modules/v2/cdg.js";
import KA655VAX from "../modules/v2/ka655.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

/*
 * SIMH's microvax3900 default memory size (INITMEMSIZE, vaxmod_defs.h:125).  We set it explicitly
 * anyway so the test does not depend on a default we did not choose.
 */
const MEMSIZE = 0x01000000;             // 16MB
const SIZE_SWITCH = {1: "-b", 2: "-w", 4: "-l"};

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
 * hex(v, digits)
 *
 * @param {number} v
 * @param {number} [digits]
 * @returns {string}
 */
function hex(v, digits)
{
    let s = (v >>> 0).toString(16).toUpperCase();
    return digits? s.padStart(digits, "0") : s;
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
        process.env["SIMH_BIN"],
        path.resolve(REPO_ROOT, "../pcjs-vax/open-simh/BIN/microvax3900")
    ].filter((p) => !!p);
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error("microvax3900 binary not found (tried: " + candidates.join(", ") + ").\n" +
        "This test grades against REAL SIMH; it has no fixture fallback.  Pass --simh PATH or set SIMH_BIN.");
}

/* ------------------------------------------------------------------------------------------- *
 * Op generation
 * ------------------------------------------------------------------------------------------- */

/*
 * Address categories.  ALIAS_S0/ALIAS_S1 add 0x80000000 / 0xC0000000 to an in-memory address:
 * both mask to the same physical address under PAMASK, and both are NEGATIVE as int32.
 */
const CAT = {
    RAM:        "ram",              // uniform within RAM
    BLOCK:      "block",            // within +/- 8 bytes of an 8KB block boundary
    MEMTOP:     "memtop",           // within 16 bytes of the top of physical memory
    ALIAS_S0:   "alias-s0",         // RAM address + 0x80000000  (S0 system space)
    ALIAS_S1:   "alias-s1",         // RAM address + 0xC0000000
    ALIAS_TOP:  "alias-top",        // top-of-memory address + 0x80000000
    ALIAS_B30:  "alias-b30",        // RAM address + 0x40000000 (above PAMASK but still positive)
    NXM:        "nxm",              // wholly non-existent (above RAM, below the cache diag space)
    ALIAS_NXM:  "alias-nxm"         // non-existent address + 0x80000000 (masking BEFORE the
                                    // existence check, which is the ordering that can be wrong)
};

/*
 * HOT REGIONS -- why the address pool is not uniform over 16MB.
 *
 * Spreading 75k writes uniformly over 16MB means almost every read lands on memory nobody ever
 * wrote, so almost every "value matches" comparison is really only comparing zero to zero.  That
 * makes the test look enormous and prove very little; it is exactly how a byte-order or
 * shift/mask defect in the far end of an unaligned access can survive a 150k-op run (verified:
 * with a uniform pool, a deliberately broken block-straddle boundary went UNDETECTED).
 *
 * So in-memory addresses are drawn from a small set of 128-byte hot regions -- placed at address
 * zero, at the very top of physical memory, straddling 8KB block boundaries, and at random
 * longword-aligned offsets -- which guarantees that reads keep landing on bytes that earlier
 * writes actually set, with every alignment and every access size.
 */
const REGION_SIZE = 128;

/**
 * buildRegions(rng)
 *
 * @param {function(): number} rng
 * @returns {Object} {ram: Array.<number>, block: Array.<number>, top: number}
 */
function buildRegions(rng)
{
    let nBlocks = MEMSIZE / BusVAX.BLOCK_SIZE;
    let aRam = [0];
    let aBlock = [];
    for (let i = 0; i < 24; i++) {
        let k = 1 + Math.floor(rng() * (nBlocks - 1));
        aBlock.push(k * BusVAX.BLOCK_SIZE - (REGION_SIZE >> 1));    // centered on the boundary
    }
    for (let i = 0; i < 40; i++) {
        aRam.push(Math.floor(rng() * (MEMSIZE - REGION_SIZE)) & ~3);
    }
    return {ram: aRam, block: aBlock, top: MEMSIZE - REGION_SIZE};
}

/**
 * genAddr(rng, cat, regions)
 *
 * @param {function(): number} rng
 * @param {string} cat
 * @param {Object} regions
 * @returns {number} (may exceed 0x7FFFFFFF; that is the point)
 */
function genAddr(rng, cat, regions)
{
    let pick = (a) => a[Math.floor(rng() * a.length)] + Math.floor(rng() * REGION_SIZE);
    switch (cat) {
    case CAT.RAM:
        return pick(regions.ram);
    case CAT.BLOCK:
        return pick(regions.block);
    case CAT.MEMTOP:
        return MEMSIZE - 1 - Math.floor(rng() * 16);
    case CAT.ALIAS_S0:
        return 0x80000000 + pick(regions.ram);
    case CAT.ALIAS_S1:
        return 0xC0000000 + pick(regions.block);
    case CAT.ALIAS_TOP:
        return 0x80000000 + (MEMSIZE - 1 - Math.floor(rng() * 16));
    case CAT.ALIAS_B30:
        return 0x40000000 + pick(regions.ram);
    case CAT.ALIAS_NXM:
        return 0x80000000 + MEMSIZE + Math.floor(rng() * (0x0F000000 - 0x100));
    case CAT.NXM:
        /*
         * [0x01000000, 0x0FFFFF00) is above RAM and below CDGBASE (0x10000000), so it is
         * non-existent to SIMH for any access size, with no risk of straying into the cache
         * diagnostic space, the ROM or the NVR -- none of which this item decodes.
         */
        return MEMSIZE + Math.floor(rng() * (0x0F000000 - 0x100));
    }
    throw new Error("bad category " + cat);
}

const CAT_WEIGHTS = [
    [CAT.RAM,       35],
    [CAT.BLOCK,     18],
    [CAT.MEMTOP,     8],
    [CAT.ALIAS_S0,  14],
    [CAT.ALIAS_S1,   6],
    [CAT.ALIAS_TOP,  6],
    [CAT.ALIAS_B30,  4],
    [CAT.NXM,        5],
    [CAT.ALIAS_NXM,  4]
];
const CAT_TOTAL = CAT_WEIGHTS.reduce((n, e) => n + e[1], 0);

/**
 * pickCat(rng)
 *
 * @param {function(): number} rng
 * @returns {string}
 */
function pickCat(rng)
{
    let r = rng() * CAT_TOTAL;
    for (let e of CAT_WEIGHTS) {
        r -= e[1];
        if (r < 0) return e[0];
    }
    return CAT.RAM;
}

/**
 * genOps(nOps, seed)
 *
 * @param {number} nOps
 * @param {number} seed
 * @returns {Array.<Object>}
 */
function genOps(nOps, seed)
{
    let rng = mulberry32(seed);
    let regions = buildRegions(rng);
    let aOps = [];
    for (let i = 0; i < nOps; i++) {
        let cat = pickCat(rng);
        let addr = genAddr(rng, cat, regions);
        let r = rng();
        let size = r < 0.25? 1 : (r < 0.5? 2 : (r < 0.8? 4 : 8));
        let op = {
            cat,
            addr,
            /*
             * How the address is HANDED to the JS bus.  An address above 0x7FFFFFFF can reach the
             * bus either as a positive double (address arithmetic done in floating point) or as a
             * negative int32 (address arithmetic done with JS bitwise operators, which is what the
             * ported CPU will do, since GET_* macros and register values all end up int32).  Both
             * must work, so both are exercised.  SIMH always receives the unsigned hex form.
             */
            fSigned: rng() < 0.5,
            size,
            fWrite: rng() < 0.5,
            lo: (Math.floor(rng() * 0x100000000) >>> 0),
            hi: (Math.floor(rng() * 0x100000000) >>> 0)
        };
        aOps.push(op);
    }
    return aOps;
}

/* ------------------------------------------------------------------------------------------- *
 * SIMH command stream
 * ------------------------------------------------------------------------------------------- */

/**
 * buildIni(aOps, aVerify)
 *
 * Emits one SIMH command per line so that a failing command's echoed line number ("<file>-N> cmd")
 * identifies it unambiguously.  Returns the text plus the ini line number of each emitted command.
 *
 * @param {Array.<Object>} aOps
 * @param {Array.<number>} aVerify (aligned longword addresses to read back at the end)
 * @returns {Object} {text, aCmds} where aCmds[i] = {line, kind, iOp, part}
 */
function buildIni(aOps, aVerify)
{
    let aLines = [];
    let aCmds = [];
    let emit = (cmd, info) => {
        aLines.push(cmd);
        info.line = aLines.length;              // 1-based, matching SIMH's echoed line number
        aCmds.push(info);
    };

    aLines.push("set cpu " + (MEMSIZE >> 20) + "m");

    for (let i = 0; i < aOps.length; i++) {
        let op = aOps[i];
        if (op.size == 8) {
            let a1 = (op.addr + 4) >>> 0;
            if (op.fWrite) {
                emit("d -l " + hex(op.addr) + " " + hex(op.lo), {kind: "d", iOp: i, part: 0});
                emit("d -l " + hex(a1) + " " + hex(op.hi), {kind: "d", iOp: i, part: 1});
            } else {
                emit("e -l " + hex(op.addr), {kind: "e", iOp: i, part: 0, lnt: 4});
                emit("e -l " + hex(a1), {kind: "e", iOp: i, part: 1, lnt: 4});
            }
        } else {
            let sw = SIZE_SWITCH[op.size];
            let v = op.lo & (op.size == 1? 0xff : (op.size == 2? 0xffff : -1));
            if (op.fWrite) {
                emit("d " + sw + " " + hex(op.addr) + " " + hex(v), {kind: "d", iOp: i, part: 0});
            } else {
                emit("e " + sw + " " + hex(op.addr), {kind: "e", iOp: i, part: 0, lnt: op.size});
            }
        }
    }

    for (let i = 0; i < aVerify.length; i++) {
        emit("e -l " + hex(aVerify[i]), {kind: "v", iVerify: i, lnt: 4});
    }

    aLines.push("quit");
    return {text: aLines.join("\n") + "\n", aCmds};
}

/**
 * runSimh(simhBin, iniText, aCmds)
 *
 * Runs the simulator once and attributes every result / error back to its command.
 *
 * @param {string} simhBin
 * @param {string} iniText
 * @param {Array.<Object>} aCmds
 * @returns {Array.<Object>} results parallel to aCmds: {ok, value} or {ok: false, err}
 */
function runSimh(simhBin, iniText, aCmds)
{
    let dir = fs.mkdtempSync(path.join(os.tmpdir(), "vax-busdiff-"));
    let iniPath = path.join(dir, "run.ini");
    let outPath = path.join(dir, "run.out");
    try {
        fs.writeFileSync(iniPath, iniText);
        let out = execFileSync(simhBin, [iniPath], {maxBuffer: 1 << 30, encoding: "utf8"});
        fs.writeFileSync(outPath, out);

        /*
         * Successful EXAMINE prints "<ADDR>:\t<VALUE>".  A failing command is echoed with its ini
         * line number ("<path>/run.ini-<N>> <cmd>") followed by the error text; successful DEPOSITs
         * print nothing.  So: errors are attributed by line number, and the remaining EXAMINE
         * results are consumed in command order.
         */
        let mapErr = new Map();
        let aValues = [];
        let aLines = out.split("\n");
        let reResult = /^([0-9A-F]+):\t([0-9A-F]+)\s*$/;
        let reEcho = /-(\d+)>\s+(\S.*)$/;
        for (let i = 0; i < aLines.length; i++) {
            let line = aLines[i];
            let m = reResult.exec(line);
            if (m) {
                aValues.push(parseInt(m[2], 16) >>> 0);
                continue;
            }
            m = reEcho.exec(line);
            if (m) {
                mapErr.set(+m[1], (aLines[i + 1] || "").trim());
            }
        }

        let aResults = new Array(aCmds.length);
        let iValue = 0;
        for (let i = 0; i < aCmds.length; i++) {
            let cmd = aCmds[i];
            if (mapErr.has(cmd.line)) {
                aResults[i] = {ok: false, err: mapErr.get(cmd.line)};
            } else if (cmd.kind == "d") {
                aResults[i] = {ok: true};
            } else {
                if (iValue >= aValues.length) {
                    throw new Error("SIMH output underrun at command " + i + " (ini line " + cmd.line + "); " +
                        "results=" + aValues.length + ", errors=" + mapErr.size + "; see " + outPath);
                }
                aResults[i] = {ok: true, value: aValues[iValue++]};
            }
        }
        if (iValue != aValues.length) {
            throw new Error("SIMH produced " + (aValues.length - iValue) + " unattributed result lines; see " + outPath);
        }
        return aResults;
    } finally {
        if (!process.env["VAX_BUSDIFF_KEEP"]) fs.rmSync(dir, {recursive: true, force: true});
    }
}

/* ------------------------------------------------------------------------------------------- *
 * JS side
 * ------------------------------------------------------------------------------------------- */

/**
 * scpExamine(bus, addr, lnt)
 *
 * Reproduces scp.c get_aval() + vax_sys.c fprint_sym() over BusVAX's BYTE interface: read lnt
 * bytes, stop at the first non-existent one, leave the rest zero, and report an error only if the
 * very first byte failed.  Little-endian composition.
 *
 * @param {BusVAX} bus
 * @param {number} addr
 * @param {number} lnt
 * @returns {Object} {ok, value, nGood}
 */
function scpExamine(bus, addr, lnt)
{
    let v = 0, nGood = 0;
    for (let i = 0; i < lnt; i++) {
        bus.checkFault();
        let b = bus.getByteDirect(addr + i);
        if (bus.checkFault()) break;
        v |= (b << (i << 3));
        nGood++;
    }
    if (!nGood) return {ok: false, value: 0, nGood: 0};
    return {ok: true, value: v >>> 0, nGood};
}

/**
 * nativeRead(bus, addr, size)
 *
 * @param {BusVAX} bus
 * @param {number} addr
 * @param {number} size (1, 2, 4 or 8)
 * @returns {Object} {value|lo,hi, fFault}
 */
function nativeRead(bus, addr, size)
{
    bus.checkFault();
    if (size == 1) return {value: bus.getByte(addr) >>> 0, fFault: bus.checkFault()};
    if (size == 2) return {value: bus.getWord(addr) >>> 0, fFault: bus.checkFault()};
    if (size == 4) return {value: bus.getLong(addr) >>> 0, fFault: bus.checkFault()};
    let lo = bus.getLong(addr);
    let fLo = bus.checkFault();
    let hi = bus.getLong((addr >>> 0) + 4);
    let fHi = bus.checkFault();
    return {lo: lo >>> 0, hi: hi >>> 0, fLo, fHi};
}

/**
 * nativeWrite(bus, addr, size, lo, hi)
 *
 * @param {BusVAX} bus
 * @param {number} addr
 * @param {number} size
 * @param {number} lo
 * @param {number} hi
 */
function nativeWrite(bus, addr, size, lo, hi)
{
    switch (size) {
    case 1: bus.setByte(addr, lo & 0xff); break;
    case 2: bus.setWord(addr, lo & 0xffff); break;
    case 4: bus.setLong(addr, lo | 0); break;
    case 8: bus.setQuad(addr, lo | 0, hi | 0); break;
    }
}

/**
 * makeBus(mutation)
 *
 * @param {string} [mutation] one of "", "noNormalize", "endian"
 * @returns {BusVAX}
 */
function makeBus(mutation)
{
    let bus = new BusVAX({'busWidth': VAX.PAWIDTH, 'id': "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    if (mutation == "noNormalize") {
        /*
         * The crudest form of the defect this item exists to prevent: an entry point that indexes
         * the block array with a raw, possibly-negative int32 address.
         */
        bus.getByte = function(addr) {
            return this.aMemBlocks[addr >> this.nBlockShift].readByte(addr & this.nBlockLimit, addr);
        };
        bus.getByteDirect = bus.getByte;
    } else if (mutation == "straddle") {
        /*
         * The block-straddle boundary off by one: a longword starting 3 bytes from the end of a
         * block takes the in-block fast path, whose stitching then indexes one longword past the
         * end of the Int32Array.  JS returns undefined for that, `undefined << n` is 0, and the
         * read silently produces a value with a zeroed high byte instead of throwing.  This is the
         * failure mode a test that only reads ALIGNED longwords cannot see.
         */
        bus.getLong = function(addr) {
            addr = (addr >>> 0) & this.nBusMask;
            let off = addr & this.nBlockLimit;
            if (off <= this.nBlockLimit - 2) {
                return this.aMemBlocks[addr >>> this.nBlockShift].readLong(off, addr);
            }
            let n = this.nFaults, l = 0;
            for (let i = 0; i < 4; i++) {
                let b = this.getByte(addr + i);
                if (this.nFaults != n) break;
                l |= (b << (i << 3));
            }
            return l;
        };
    } else if (mutation == "endian") {
        /* Byte order reversed on longword reads: the differential must notice. */
        bus.getLong = function(addr) {
            let l = BusVAX.prototype.getLong.call(this, addr) | 0;
            return ((l >>> 24) | ((l >>> 8) & 0xff00) | ((l & 0xff00) << 8) | (l << 24)) | 0;
        };
    }
    return bus;
}

/* ------------------------------------------------------------------------------------------- *
 * The differential run
 * ------------------------------------------------------------------------------------------- */

/**
 * runDiff(simhBin, nOps, seed, mutation, fQuiet)
 *
 * @param {string} simhBin
 * @param {number} nOps
 * @param {number} seed
 * @param {string} [mutation]
 * @param {boolean} [fQuiet]
 * @returns {Object} {aFailures, stats}
 */
function runDiff(simhBin, nOps, seed, mutation, fQuiet)
{
    let aOps = genOps(nOps, seed);
    let bus = makeBus(mutation);

    let stats = {
        nOps, nReads: 0, nWrites: 0,
        bySize: {1: 0, 2: 0, 4: 0, 8: 0},
        byCat: {},
        nUnaligned: 0,
        nAboveS0Signed: 0,
        nBlockStraddle: 0,
        nMemTopStraddle: 0,
        nAboveS0: 0,
        nNativeValueChecks: 0,
        nNativeValueNonZero: 0,
        nScpChecks: 0,
        nSimhErrors: 0,
        nVerifyChecks: 0
    };
    for (let k in CAT) stats.byCat[CAT[k]] = 0;

    /*
     * Every byte we ever write is recorded so it can be read back from BOTH implementations at the
     * end of the run; a write whose effect is never observed is a write that was never tested.
     */
    let setVerify = new Set();
    let aFailures = [];
    let fail = (iOp, what, expected, actual) => {
        if (aFailures.length < 40) {
            let op = aOps[iOp];
            aFailures.push("op#" + iOp + " [" + (op? op.cat : "verify") + "] " +
                (op? (op.fWrite? "write" : "read") + " size=" + op.size + " addr=0x" + hex(op.addr, 8) : "") +
                " -- " + what + ": SIMH=" + expected + " JS=" + actual);
        }
        return false;
    };

    /*
     * Pass 1: JS side, recording what each op should have produced.  We do NOT touch SIMH yet;
     * the whole command stream is built first and executed in one process, which keeps 100k+ ops
     * to a couple of seconds.
     */
    let aExpect = [];               // parallel to read/verify commands, in command order
    for (let i = 0; i < aOps.length; i++) {
        let op = aOps[i];
        let addrPhys = (op.addr >>> 0) & VAX.PAMASK;
        /*
         * jsAddr is what the BUS is handed.  For op.fSigned this is the negative int32 form of the
         * same address; op.addr (always the positive double) is what SIMH is told.  If the bus ever
         * treated the two differently, every alias category would diverge.
         */
        let jsAddr = op.fSigned? (op.addr | 0) : op.addr;

        stats.bySize[op.size]++;
        stats.byCat[op.cat]++;
        if (op.addr >>> 0 >= 0x80000000) {
            stats.nAboveS0++;
            if (op.fSigned) stats.nAboveS0Signed++;
        }
        if (op.size > 1 && (addrPhys % op.size)) stats.nUnaligned++;
        if (op.size > 1) {
            let off = addrPhys & (BusVAX.BLOCK_SIZE - 1);
            if (off + op.size > BusVAX.BLOCK_SIZE) stats.nBlockStraddle++;
            if (addrPhys < MEMSIZE && addrPhys + op.size > MEMSIZE) stats.nMemTopStraddle++;
        }

        if (op.fWrite) {
            stats.nWrites++;
            nativeWrite(bus, jsAddr, op.size, op.lo, op.hi);
            for (let b = 0; b < op.size; b++) {
                let a = ((op.addr >>> 0) + b) & VAX.PAMASK;
                setVerify.add(a & ~3);
            }
            aExpect.push(null);
            if (op.size == 8) aExpect.push(null);
        } else {
            stats.nReads++;
            if (op.size == 8) {
                let n = nativeRead(bus, jsAddr, 8);
                aExpect.push({iOp: i, part: 0, lnt: 4, scp: scpExamine(bus, jsAddr, 4), nat: n.lo, fFault: n.fLo});
                aExpect.push({iOp: i, part: 1, lnt: 4, scp: scpExamine(bus, (op.addr >>> 0) + 4, 4), nat: n.hi, fFault: n.fHi});
            } else {
                let n = nativeRead(bus, jsAddr, op.size);
                aExpect.push({iOp: i, part: 0, lnt: op.size, scp: scpExamine(bus, jsAddr, op.size), nat: n.value, fFault: n.fFault});
            }
        }
    }

    let aVerify = Array.from(setVerify).sort((a, b) => a - b);
    let aVerifyExpect = aVerify.map((a) => scpExamine(bus, a, 4));

    /*
     * Pass 2: run the real simulator.
     */
    let {text, aCmds} = buildIni(aOps, aVerify);
    let aResults = runSimh(simhBin, text, aCmds);

    /*
     * Pass 3: compare, command by command.
     */
    let iExpect = 0;
    for (let i = 0; i < aCmds.length; i++) {
        let cmd = aCmds[i], res = aResults[i];

        if (cmd.kind == "v") {
            let exp = aVerifyExpect[cmd.iVerify];
            stats.nVerifyChecks++;
            if (exp.ok != res.ok) {
                fail(-1, "verify @0x" + hex(aVerify[cmd.iVerify], 8) + " existence", res.ok? "ok" : "NXM", exp.ok? "ok" : "NXM");
            } else if (exp.ok && exp.value !== res.value) {
                fail(-1, "verify @0x" + hex(aVerify[cmd.iVerify], 8), "0x" + hex(res.value, 8), "0x" + hex(exp.value, 8));
            }
            continue;
        }

        if (cmd.kind == "d") {
            /*
             * SIMH deposits byte-by-byte and stops at the first non-existent byte.  We do not
             * compare anything here directly (a deposit produces no value); its effect is caught by
             * the read-back sweep above, which covers every byte we asked either side to write.
             */
            if (!res.ok) stats.nSimhErrors++;
            continue;
        }

        let exp = aExpect[iExpect++];
        while (exp === null) exp = aExpect[iExpect++];      // skip the write placeholders
        if (exp.iOp != cmd.iOp || exp.part != cmd.part) {
            throw new Error("internal: expectation stream desynchronized at command " + i);
        }
        if (!res.ok) stats.nSimhErrors++;

        /* (1) byte-level SCP model must match SIMH exactly, always */
        stats.nScpChecks++;
        if (exp.scp.ok != res.ok) {
            fail(cmd.iOp, "scp-model existence(part " + cmd.part + ")", res.ok? "ok" : "NXM(" + res.err + ")", exp.scp.ok? "ok" : "NXM");
        } else if (res.ok && exp.scp.value !== res.value) {
            fail(cmd.iOp, "scp-model value(part " + cmd.part + ")", "0x" + hex(res.value, 8), "0x" + hex(exp.scp.value, 8));
        }

        /* (2) native multi-byte accessor */
        if (!exp.fFault) {
            if (!res.ok) {
                fail(cmd.iOp, "native accessor succeeded where SIMH reported NXM(part " + cmd.part + ")", "NXM", "0x" + hex(exp.nat, 8));
            } else {
                stats.nNativeValueChecks++;
                if (res.value) stats.nNativeValueNonZero++;    // a 0==0 comparison proves little
                if (exp.nat !== res.value) {
                    fail(cmd.iOp, "native value(part " + cmd.part + ")", "0x" + hex(res.value, 8), "0x" + hex(exp.nat, 8));
                }
            }
        } else {
            /*
             * We faulted.  SIMH must have either refused the access outright (first byte NXM) or
             * short-read it (some later byte NXM, hence a zero fill), i.e. it must NOT have been a
             * clean full-length read.
             */
            if (res.ok && exp.scp.nGood >= exp.lnt) {
                fail(cmd.iOp, "native accessor faulted on an access SIMH read cleanly(part " + cmd.part + ")", "0x" + hex(res.value, 8), "FAULT");
            }
        }
    }

    if (!fQuiet) {
        console.log("  ops=%d reads=%d writes=%d", stats.nOps, stats.nReads, stats.nWrites);
        console.log("  by size: b=%d w=%d l=%d q=%d", stats.bySize[1], stats.bySize[2], stats.bySize[4], stats.bySize[8]);
        console.log("  by category: %s", JSON.stringify(stats.byCat));
        console.log("  unaligned=%d block-straddle=%d memtop-straddle=%d addr>=0x80000000=%d (of which handed to the bus as a negative int32: %d)",
            stats.nUnaligned, stats.nBlockStraddle, stats.nMemTopStraddle, stats.nAboveS0, stats.nAboveS0Signed);
        console.log("  comparisons: scp-model=%d native-value=%d (non-zero data: %d) write-readback=%d simh-NXM=%d",
            stats.nScpChecks, stats.nNativeValueChecks, stats.nNativeValueNonZero, stats.nVerifyChecks, stats.nSimhErrors);
    }
    return {aFailures, stats};
}

/**
 * reportScopeGaps(simhBin)
 *
 * DISCLOSURE FOR THE UNDECODED ROWS, AND A GRADED COMPARISON FOR THE DECODED ONES.  This item
 * reserves the KA655 I/O / register / Qbus ranges but deliberately does NOT decode them (that is
 * the device items' work), so SIMH and this bus DIVERGE there by design: SIMH's console happily
 * reads them, we report non-existent memory.  Those addresses are excluded from the randomized
 * address pool, and this function probes them on both sides and prints the divergence explicitly,
 * so the gap is on the record rather than hidden behind a green result.
 *
 * Rows whose range a later item DECODED are not simply deleted from this table when they stop
 * diverging -- their EXPECTATION changes and becomes an ASSERTION, so the row keeps earning its
 * place.  Deleting it instead would quietly retire the only place this file ever looks at that
 * address.  Each row therefore carries what it must do, and a row that does anything else FAILS.
 *
 * WHICH ROWS CAN DIVERGE AT ALL IS DECIDED BY cpu_ex(), NOT BY THIS FILE'S SCOPE.  The SIMH side
 * of this probe is a console `e -b`, which reaches vax_cpu.c's cpu_ex() (:3399) -- and that
 * function services ONLY `ADDR_IS_MEM || ADDR_IS_CDG || ADDR_IS_ROM || ADDR_IS_NVR`, returning
 * SCPE_NXM for everything else.  So IOPAGE, REG and CQM read NXM on the SIMH side too, no matter
 * what SIMH's INSTRUCTION path would do with them (it decodes all three -- see mchkdiff.js, which
 * probes them by execution for exactly this reason).  Their agreement here is a property of the
 * console path, not evidence that this bus decodes them.  The three expectations:
 *
 *   ROM  "match"   -- decoded by pcjsvax-223 (BusVAX.addRom()), and cpu_ex services it.  The probe
 *                     bus loads a zero-filled ROM image first, matching SIMH's own un-loaded state
 *                     exactly (rom_reset() calloc's the ROM array to zero; this script never issues
 *                     `load -r`, so both sides read a fresh, unloaded ROM).
 *   CDG  "match"   -- decoded by pcjsvax-0b7 (BusVAX.addCdg(), cdg.js), and cpu_ex services it.
 *                     This row WAS the documented divergence "CDG @0x10000000: SIMH=0x00 JS=NXM";
 *                     it is now an asserted match.  The register-level grading of that range --
 *                     aliasing, the CACR diagnostic-parity side effect, the write merge, end-to-end
 *                     backing -- is tests/cdgdiff.js's; this row only holds the one address this
 *                     file has always probed.
 *   NVR  "diverge" -- cpu_ex SERVICES it but this bus does not decode it, so it is the one genuine
 *                     scope divergence left in this table.
 *   IOPAGE / REG / CQM  "match" -- both sides NXM, because cpu_ex does not service them either.
 *
 * @param {string} simhBin
 * @returns {Array.<string>} problems -- empty unless a row did something other than what it says.
 *   BOTH directions fail: a "match" row that diverges, and a "diverge" row that starts matching
 *   (which means the item that decoded it left a stale expectation here).
 */
function reportScopeGaps(simhBin)
{
    let aProbes = [
        ["IOPAGE", VAX.PHYSMEM.IOPAGE_BASE, "match"],       // cpu_ex does not service it either
        ["ROM",    VAX.PHYSMEM.ROM_BASE,    "match"],       // decoded, pcjsvax-223
        ["REG",    VAX.PHYSMEM.REG_BASE,    "match"],       // cpu_ex does not service it either
        ["NVR",    VAX.PHYSMEM.NVR_BASE,    "diverge"],     // cpu_ex services it; this bus does not
        ["CDG",    VAX.PHYSMEM.CDG_BASE,    "match"],       // decoded, pcjsvax-0b7
        ["CQM",    VAX.PHYSMEM.CQM_BASE,    "match"]        // cpu_ex does not service it either
    ];
    let aLines = ["set cpu " + (MEMSIZE >> 20) + "m"];
    let aCmds = [];
    for (let p of aProbes) {
        aLines.push("e -b " + hex(p[1]));
        aCmds.push({line: aLines.length, kind: "e"});
    }
    aLines.push("quit");
    let aResults = runSimh(simhBin, aLines.join("\n") + "\n", aCmds);

    let bus = makeBus("");
    bus.addRom(new Uint8Array(VAX.PHYSMEM.ROM_SIZE));
    bus.addCdg(new CDGVAX(new KA655VAX()));
    let problems = [];
    console.log("\nKA655 ranges probed through the SIMH console path (each row carries the " +
                "relationship it MUST hold -- see this function's header):");
    for (let i = 0; i < aProbes.length; i++) {
        let [name, addr, expect] = aProbes[i];
        let js = scpExamine(bus, addr, 1);
        let simhStr = aResults[i].ok? "0x" + hex(aResults[i].value, 2) : "NXM";
        let jsStr = js.ok? "0x" + hex(js.value, 2) : "NXM";
        let same = (aResults[i].ok == js.ok) && (!aResults[i].ok || aResults[i].value == js.value);
        let held = (expect == "match") == same;
        console.log("  %s @0x%s: SIMH=%s  JS=%s   <- expect %s, %s", name.padEnd(6), hex(addr, 8),
            simhStr, jsStr, expect.padEnd(7), held? "OK" : "VIOLATED");
        if (!held && expect == "match") {
            problems.push(`SCOPE: ${name} @0x${hex(addr, 8)} must MATCH but diverges ` +
                `(SIMH=${simhStr} JS=${jsStr})`);
        }
        if (!held && expect == "diverge") {
            problems.push(`SCOPE: ${name} @0x${hex(addr, 8)} is recorded as a scope DIVERGENCE but ` +
                `now agrees with SIMH (both ${simhStr}) -- the item that decoded it must change this ` +
                `row's expectation to "match" rather than leaving a stale one here`);
        }
    }
    return problems;
}

/* ------------------------------------------------------------------------------------------- *
 * main
 * ------------------------------------------------------------------------------------------- */

function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, dflt) => {
        let i = argv.indexOf(name);
        return i >= 0? argv[i + 1] : dflt;
    };
    let nOps = +getArg("--ops", 150000);
    let seed = +getArg("--seed", 0xC0FFEE);
    let simhBin = findSimh(getArg("--simh", null));
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;

    console.log("VAX bus/memory differential test");
    console.log("  SIMH binary: %s", simhBin);
    console.log("  ops=%d seed=0x%s memsize=0x%s", nOps, hex(seed), hex(MEMSIZE, 8));

    let t0 = Date.now();
    let {aFailures, stats} = runDiff(simhBin, nOps, seed, "", false);
    console.log("  elapsed: %ds", ((Date.now() - t0) / 1000).toFixed(1));

    let aErrors = aFailures.slice();

    /*
     * Coverage assertions.  The item requires >= 100k operations, block-boundary straddles, the
     * top of physical memory, and -- specifically -- S0 addresses above 0x80000000.  A run that
     * silently stopped covering one of those would otherwise still report "0 failures".
     */
    let require = (cond, msg) => { if (!cond) aErrors.push("COVERAGE: " + msg); };
    let scale = nOps / 150000;                  // secondary thresholds track the op count
    require(stats.nOps >= 100000, "fewer than 100000 operations (" + stats.nOps + ")");
    require(stats.nAboveS0 > 20000 * scale, "too few addresses >= 0x80000000 (" + stats.nAboveS0 + ")");
    require(stats.nAboveS0Signed > 8000 * scale, "too few S0 addresses delivered as a negative int32 (" + stats.nAboveS0Signed + ")");
    require(stats.nBlockStraddle > 3000 * scale, "too few block-boundary straddles (" + stats.nBlockStraddle + ")");
    require(stats.nMemTopStraddle > 1000 * scale, "too few top-of-physical-memory straddles (" + stats.nMemTopStraddle + ")");
    require(stats.nUnaligned > 30000 * scale, "too few unaligned accesses (" + stats.nUnaligned + ")");
    require(stats.bySize[1] && stats.bySize[2] && stats.bySize[4] && stats.bySize[8], "not all access sizes exercised");
    require(stats.nSimhErrors > 2000 * scale, "too few SIMH non-existent-memory reports (" + stats.nSimhErrors + ")");
    require(stats.nNativeValueChecks > 30000 * scale, "too few native-accessor value comparisons (" + stats.nNativeValueChecks + ")");
    require(stats.nNativeValueNonZero > 20000 * scale, "too few native-accessor comparisons against NON-ZERO data (" +
        stats.nNativeValueNonZero + "); a pool of untouched memory makes the run look bigger than it is");
    /*
     * The read-back sweep covers every DISTINCT longword ever written; with a hot address pool that
     * is a few thousand, not tens of thousands, and that is the point -- the same bytes get written
     * and re-read constantly during the run instead of once at the end.
     */
    require(stats.nVerifyChecks > 2000 * scale, "too few write read-back comparisons (" + stats.nVerifyChecks + ")");
    require(stats.byCat[CAT.ALIAS_NXM] > 2000 * scale, "too few non-existent S0 addresses (" + stats.byCat[CAT.ALIAS_NXM] + ")");

    aErrors.push(...reportScopeGaps(simhBin));

    if (fSelfCheck) {
        console.log("\nSelf-check: the differential must FAIL when the bus is deliberately broken.");
        for (let mut of ["noNormalize", "straddle", "endian"]) {
            let caught = false, why = "";
            try {
                let r = runDiff(simhBin, 4000, seed ^ 0x5555, mut, true);
                caught = r.aFailures.length > 0;
                why = caught? r.aFailures[0] : "NO FAILURES REPORTED";
            } catch (e) {
                caught = true;                  // a thrown TypeError is also a detection
                why = "threw: " + e.message.split("\n")[0];
            }
            console.log("  mutation '%s': %s (%s)", mut, caught? "detected" : "MISSED", why);
            if (!caught) aErrors.push("SELFCHECK: mutation '" + mut + "' was not detected");
        }
    }

    if (aErrors.length) {
        console.log("\nFAILED (%d):", aErrors.length);
        for (let f of aErrors) console.log("  " + f);
        process.exit(1);
    }
    console.log("\nPASS: BusVAX/MemoryVAX are indistinguishable from SIMH over %d operations.", stats.nOps);
}

if (process.argv[1] && path.resolve(process.argv[1]) == path.resolve(fileURLToPath(import.meta.url))) {
    main();
}

export { genOps, buildIni, runSimh, scpExamine, nativeRead, nativeWrite, makeBus, findSimh, hex, MEMSIZE, CAT };
