/**
 * @fileoverview Differential test: MMUVAX vs. the real Open SIMH microvax3900
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
 * WHAT THIS IS
 * ------------
 * A differential test of machines/dec/vax/modules/v2/mmu.js against a REAL, EXECUTED Open SIMH
 * microvax3900.  No fixtures, no golden files: every run builds a command stream or a trace,
 * launches the simulator, and compares its actual output.  If the binary is missing the test
 * FAILS -- it never degrades into self-comparison.
 *
 *      machines/dec/vax/tests/simh/build.sh                 # patched simulator, once
 *      node machines/dec/vax/tests/mmudiff.js --selfcheck
 *      node machines/dec/vax/tests/mmudiff.js
 *
 * TWO PHASES
 * ----------
 *   EXERCISER  A randomized walk over a page-table layout this file constructs, driven through
 *              `SHOW CPU MMUOP=` (patch 0003).  Both sides hold the SAME 16MB of real memory and
 *              the same page tables, so this phase compares TRANSLATED PHYSICAL ADDRESSES, DATA
 *              VALUES and FAULT CODES exactly -- not just "did it fault".  It covers P0, P1, S0
 *              and S1, all four access modes, read and write access, every alignment, page and
 *              longword boundary crossings, both TB invalidation instructions, MAPEN off and on,
 *              and page-table mutation underneath a live TB.
 *
 *   EHKAA      The DEC EHKAA hardware-core diagnostic run to its PASS halt with the MMU trace
 *              armed, then replayed: 343,000 real memory-management operations issued by real
 *              VAX code, including every MAPEN transition the diagnostic makes.  Memory is not
 *              modelled here; the PTE values SIMH read are fed from the trace and what is graded
 *              is the ADDRESSES we compute, the TB hit/miss decisions we make, and the faults we
 *              raise.  See "the fill oracle" below.
 *
 * WHY THE ORACLE IS SHAPED LIKE THIS
 * ----------------------------------
 * Translation is not observable from outside the machine.  SIMH's stock console can *almost* show
 * it -- `SHOW CPU VIRTUAL=n` is a thin wrapper over `Test()` -- but only for READ access in one of
 * the four modes, which leaves the entire write-access half of the protection matrix, the M bit,
 * and the unaligned/cross-page data path unobserved.  Patch 0003 adds the two things that close
 * that gap and nothing else:
 *
 *   SHOW CPU MMUOP=op:va:lnt:mode[:val]     one Read/Write/Test/MTPR, one machine-readable line
 *   SET CPU MMUTRACE=<file>                 a log of every MMU operation, PTE read and TB flush
 *
 * Both are additive; the simulator's own EHKAA self-test passes with them applied.
 *
 * THE FILL ORACLE (phase 2)
 * -------------------------
 * Replaying EHKAA's MMU activity would seem to require EHKAA's memory.  It does not.  The only
 * memory content that can affect a translation is the PTEs, and the trace records every PTE read
 * with its address and value.  So the replay installs those values in a sparse stub, runs the
 * REAL MMUVAX against it, and then checks three things:
 *
 *   (1) the physical address(es) we resolved match the `A`/`B` records;
 *   (2) the PTE addresses we READ match the `S`/`P`/`M` records, in order, as a prefix of our
 *       memory traffic -- so an effective-address bug in the page-table walk fails exactly,
 *       rather than probabilistically through whatever value it happened to fetch;
 *   (3) we faulted exactly when SIMH faulted, with the same code.
 *
 * (2) is what makes it a real test rather than a plausibility check: if our walk read a different
 * PTE address, the stub would hand back zero and we would report ACV -- which (1) and (3) would
 * catch anyway, but only by luck of the layout.
 *
 * COVERAGE IS ASSERTED, NOT REPORTED
 * ----------------------------------
 * The lesson from pcjsvax-cd6 (documented in docs/design/vax-on-pcjs.md) was that a uniform
 * random pool made 97% of comparisons trivial and let a deliberately broken boundary survive
 * 150,000 operations undetected.  So the virtual address pool here is built from HOT PAGES --
 * a small set of pages that are written and re-read constantly -- with offsets deliberately
 * concentrated at page and longword boundaries, and the run FAILS if any of the following drops
 * below its floor: total operations, operations per region, operations per access mode,
 * occurrences of each fault code, cross-page accesses, unaligned accesses, two-level (process)
 * page table walks, M-bit write-backs, and the fraction of read comparisons that returned
 * NON-ZERO data.
 *
 * NOTHING MAY BE SILENTLY DROPPED
 * -------------------------------
 * pcjsvax-8c0 lost 21% of its cases invisibly, because a case that machine-checked never reached
 * SIMH's history record.  That failure mode is MORE likely here, since translation faults are the
 * thing under test.  So every operation this file emits is expected to produce exactly one
 * `MMUOP` line, the count is checked, and any operation whose line is missing is reported BY
 * INDEX with its address, length and mode.  Likewise the EHKAA phase counts `O` records against
 * comparisons performed and fails on any difference.
 *
 * SELF-CHECK
 * ----------
 * --selfcheck re-runs a short pass with a deliberate defect injected into the SHIPPED code path,
 * and fails if the differential does not catch each one.  The defects are listed in
 * applyMutation(); the first of them, `signedPtidx`, is the `>>` vs `>>>` region hazard this item
 * exists to prevent.
 */

import fs from "fs";
import os from "os";
import path from "path";
import readline from "readline";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import MMUVAX from "../modules/v2/mmu.js";
import { VAXFault, VAXFAULT } from "../modules/v2/decode.js";
import { VAX } from "../modules/v2/defines.js";

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

const MEMSIZE = 0x01000000;             // 16MB, SIMH's microvax3900 default

/* ------------------------------------------------------------------------------------------- *
 * THE TEST MACHINE'S PAGE TABLES
 *
 * Every constant below is load-bearing and the relationships between them are checked by
 * assertLayout().  The single most important invariant is stated there: a page table walk must
 * never be able to read a longword that is not part of a page table.  If it could, a PTE would be
 * whatever random data happened to be there, its PFN could address the boot ROM or the Qbus I/O
 * page, and the two sides would diverge on device emulation this item does not implement -- an
 * out-of-scope divergence masquerading as an MMU bug.
 * ------------------------------------------------------------------------------------------- */

const PAGE          = 512;

/* System page table: 0x800 entries, PHYSICAL, mapping S0 va 0x80000000..0x800FFFFF. */
const SBR           = 0x00400000;
const SLR           = 0x00000800;
const SPT_PAGES     = 16;                       // 0x800 entries * 4 = 8KB = 16 pages
const SPT_PFN0      = SBR / PAGE;               // 0x2000

/* P0 page table: 0x200 entries, at S0 VIRTUAL 0x80020000 == S0 page 0x100. */
const P0PT_PA       = 0x00410000;
const P0BR          = 0x80020000;
const P0LR          = 0x00000200;
const P0PT_PAGES    = 4;                        // 0x200 entries * 4 = 2KB
const P0PT_SPAGE    = (P0BR - 0x80000000) / PAGE;   // 0x100
const P0PT_PFN0     = P0PT_PA / PAGE;           // 0x2080
/* P0 pages at or above this VPN have their PTE on the one P0-table page deliberately left
 * unmapped in system space, and therefore report PR_PTNV.  See buildLayout(). */
const P0_PTNV_VPN0  = (P0PT_PAGES - 1) * (PAGE / 4);    // 0x180

/*
 * P1 page table.  P1 grows DOWNWARD, so its valid region is at the TOP of P1 space and P1BR is
 * biased: the architecture indexes it with the full VPN (which for P1 starts at 0x200000), and
 * SIMH folds that bias out in set_map_reg().  The table is placed at the LAST S0 page that SLR
 * covers, deliberately, so that the upper half of the P1 range has its PTE beyond the system
 * length register and reports PR_PLNV -- a fault code no other part of this layout can produce.
 */
const P1PT_PA       = 0x00420000;
const P1_VPN0       = 0x003FFE00;               // first VALID P1 vpn -> va 0x7FFC0000
const P1PT_SVA      = 0x800FFE00;               // S0 va of the PTE for P1_VPN0 (S0 page 0x7FF)
const P1PT_SPAGE    = (P1PT_SVA - 0x80000000) / PAGE;   // 0x7FF
const P1BR          = (P1PT_SVA + 0x800000 - 4 * P1_VPN0) | 0;
const P1LR          = P1_VPN0 - 0x200000;       // 0x1FFE00
const P1PT_ENTRIES  = PAGE / 4;                 // 128 reachable entries; the rest are PLNV
const P1PT_PFN0     = P1PT_PA / PAGE;           // 0x2100

/* The SPT is ALSO visible in S0 space, at 0x80040000, so the test can rewrite PTEs through the
 * MMU (which is how real VMS does it, and what makes "modify a PTE under a live TB entry"
 * testable at all). */
const SPT_SVA       = 0x80040000;
const SPT_SPAGE     = (SPT_SVA - 0x80000000) / PAGE;    // 0x200

/* Data frames.  Deliberately FEW: 32 frames of 512 bytes is 16KB, so the hot virtual pages alias
 * heavily onto them and reads keep landing on bytes earlier writes actually set. */
const DATA_PA       = 0x00100000;
const DATA_FRAMES   = 32;
const DATA_PFN0     = DATA_PA / PAGE;           // 0x800

/* S0 pages whose mapping is STRUCTURAL and must never be rewritten by the op stream. */
const S0_STRUCTURAL = new Set();
for (let i = 0; i < P0PT_PAGES; i++) S0_STRUCTURAL.add(P0PT_SPAGE + i);
for (let i = 0; i < SPT_PAGES; i++)  S0_STRUCTURAL.add(SPT_SPAGE + i);
S0_STRUCTURAL.add(P1PT_SPAGE);

/*
 * PTE protection codes, weighted.  Codes 0 and 1 grant nothing (-> ACV in every mode); 4 grants
 * everything; the rest are the graded kernel/exec/supervisor/user ladder from cvtacc[].  The
 * weights exist to guarantee that EVERY mode sees both permitted and denied pages often enough to
 * clear the per-mode coverage floors.
 */
const PROT_WEIGHTS = [
    [0,  3], [1,  2], [2,  6], [3,  6], [4, 14], [5,  6], [6,  5], [7,  5],
    [8,  6], [9,  5], [10, 5], [11, 5], [12, 8], [13, 6], [14, 6], [15, 8]
];

/*
 * HOT pages get a deliberately more PERMISSIVE distribution, weighted toward the codes that grant
 * something to supervisor and user.  Without this the run is dominated by faults: a uniform draw
 * over the protection codes denies user-mode access on 10 of 16 of them, so most operations in
 * the two outer modes would abort and there would be almost nothing left to compare VALUES on.
 * The non-hot pages keep the uniform-ish distribution above, which is where the fault variety
 * comes from.  Both are needed; this is the same "concentrate on data that is actually there"
 * lesson as the hot address pool itself.
 */
const PROT_WEIGHTS_HOT = [
    [2,  3], [3,  3], [4, 26], [5,  4], [6,  3], [7,  3], [8,  8], [9,  6],
    [10, 5], [11, 5], [12, 14], [13, 8], [14, 6], [15, 12]
];

const OPS = {TESTR: 0, TESTW: 1, READ: 2, WRITE: 3, MTPR: 4};

const MODES = [0, 1, 2, 3];
const MODE_NAMES = ["K", "E", "S", "U"];

const PR_NAMES = ["ACV", "LNV", "PACV", "PLNV", "TNV", "?5", "PTNV", "OK"];

const IPR = {P0BR: 8, P0LR: 9, P1BR: 10, P1LR: 11, SBR: 12, SLR: 13, MAPEN: 56, TBIA: 57, TBIS: 58};

/*
 * Coverage floors.  MIN_OPS is ABSOLUTE and the rest are scaled by --ops / DEFAULT_OPS.  The
 * absolute floor is the important one: without it, `--ops 500` scales every other threshold down
 * with it and an undersized run passes quietly, which is precisely the failure mode this file's
 * coverage assertions exist to prevent.
 */
const DEFAULT_OPS = 150000;
const MIN_OPS     = 100000;

/* ------------------------------------------------------------------------------------------- *
 * Utilities
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
 * pickWeighted(rng, aWeights)
 *
 * @param {function(): number} rng
 * @param {Array.<Array.<number>>} aWeights [[value, weight], ...]
 * @returns {number}
 */
function pickWeighted(rng, aWeights)
{
    let total = aWeights.reduce((n, e) => n + e[1], 0);
    let r = rng() * total;
    for (let e of aWeights) {
        r -= e[1];
        if (r < 0) return e[0];
    }
    return aWeights[aWeights.length - 1][0];
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
        process.env["SIMH_MMU_BIN"],
        process.env["SIMH_BIN"],
        path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900")
    ].filter((p) => !!p);
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error("microvax3900 binary not found (tried: " + candidates.join(", ") + ").\n" +
        "This test grades against REAL SIMH and has no fixture fallback.  Build it with\n" +
        "    machines/dec/vax/tests/simh/build.sh\n" +
        "then pass --simh PATH or set SIMH_MMU_BIN.  The stock simulator is NOT sufficient:\n" +
        "patch 0003 adds SHOW CPU MMUOP= and SET CPU MMUTRACE=, which this test requires.");
}

/**
 * runSimh(bin, script, iniPath)
 *
 * @param {string} bin
 * @param {string} script
 * @param {string} iniPath
 * @returns {string} stdout
 */
function runSimh(bin, script, iniPath)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 30, stdio: ["ignore", "pipe", "pipe"]});
}

/**
 * requireMmuSupport(simhBin, scratch)
 *
 * Fails loudly and early if the simulator predates patch 0003, rather than letting every
 * comparison mysteriously report "no MMUOP line".
 *
 * @param {string} simhBin
 * @param {string} scratch
 */
function requireMmuSupport(simhBin, scratch)
{
    let out = runSimh(simhBin, "set cpu 16m\nshow cpu mmuop=0:00000004:1:0\nquit\n",
        path.join(scratch, "probe.ini"));
    if (!/^MMUOP /m.test(out)) {
        throw new Error("this simulator does not implement SHOW CPU MMUOP=; rebuild it with\n" +
            "    machines/dec/vax/tests/simh/build.sh\n" +
            "which applies patch 0003-mmu-differential-support.patch.  SIMH said:\n" + out);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * The layout: page tables, seeded data, and the invariant that keeps the walk in bounds
 * ------------------------------------------------------------------------------------------- */

/**
 * makePTE(pfn, prot, fValid, fMod)
 *
 * @param {number} pfn
 * @param {number} prot 0-15
 * @param {boolean} fValid
 * @param {boolean} [fMod]
 * @returns {number} signed int32
 */
function makePTE(pfn, prot, fValid, fMod)
{
    return ((fValid? MMUVAX.PTE_V : 0) | (prot << MMUVAX.PTE_V_ACC) | (fMod? MMUVAX.PTE_M : 0) |
        (pfn & 0x1FFFFF)) | 0;
}

/**
 * buildLayout(rng)
 *
 * Builds every longword that has to exist in physical memory before the run starts: the three
 * page tables and the initial contents of the data frames.
 *
 * @param {function(): number} rng
 * @returns {Object} {mem: Map, hotP0: Array, hotP1: Array, hotS0: Array}
 */
function buildLayout(rng)
{
    let mem = new Map();                        // physical longword address -> value
    let dataPFN = () => DATA_PFN0 + Math.floor(rng() * DATA_FRAMES);

    /*
     * Hot pages are chosen FIRST, because their page table entries get the permissive protection
     * distribution.  Each list starts with the first and last page of its region so that a
     * boundary bug at either end is reachable.
     *
     * P0_PTNV_VPN0 is the one place PR_PTNV comes from: the fourth page of the P0 page table is
     * left UNMAPPED in system space, so any P0 page whose PTE lives there faults before the PTE is
     * read.  That is safe for the walk-in-bounds invariant precisely because the fault precedes
     * the read.  Two hot P0 pages are placed inside that range on purpose.
     */
    let hotP0 = [0, 1, P0_PTNV_VPN0 - 2, P0_PTNV_VPN0 - 1, P0LR - 2, P0LR - 1];
    while (hotP0.length < 14) hotP0.push(Math.floor(rng() * P0LR));
    let hotP1 = [0, 1, P1PT_ENTRIES - 2, P1PT_ENTRIES - 1];
    while (hotP1.length < 12) hotP1.push(Math.floor(rng() * P1PT_ENTRIES));
    let hotS0 = [0, 1];
    while (hotS0.length < 16) {
        let p = Math.floor(rng() * SLR);
        if (!S0_STRUCTURAL.has(p)) hotS0.push(p);
    }
    let setHot = new Set(hotS0);
    let protS0 = (i) => pickWeighted(rng, setHot.has(i)? PROT_WEIGHTS_HOT : PROT_WEIGHTS);
    let setHotP0 = new Set(hotP0), setHotP1 = new Set(hotP1);

    /*
     * System page table.  Structural entries are written LAST so a random entry cannot clobber
     * one.
     */
    for (let i = 0; i < SLR; i++) {
        let fHot = setHot.has(i);
        mem.set(SBR + 4 * i, makePTE(dataPFN(), protS0(i), fHot || rng() > 0.10, false));
    }
    for (let i = 0; i < P0PT_PAGES; i++) {
        mem.set(SBR + 4 * (P0PT_SPAGE + i),
            makePTE(P0PT_PFN0 + i, 4, i != P0PT_PAGES - 1, false));      /* last page: PR_PTNV */
    }
    for (let i = 0; i < SPT_PAGES; i++) {
        mem.set(SBR + 4 * (SPT_SPAGE + i), makePTE(SPT_PFN0 + i, 4, true, false));
    }
    mem.set(SBR + 4 * P1PT_SPAGE, makePTE(P1PT_PFN0, 4, true, false));

    /*
     * P0 and P1 page tables.  Their PTEs are read only after the system page table has mapped the
     * page they live on, so this is where the two-level walk gets exercised.
     */
    for (let i = 0; i < P0LR; i++) {
        let fHot = setHotP0.has(i);
        mem.set(P0PT_PA + 4 * i, makePTE(dataPFN(),
            pickWeighted(rng, fHot? PROT_WEIGHTS_HOT : PROT_WEIGHTS), fHot || rng() > 0.10, false));
    }
    for (let i = 0; i < P1PT_ENTRIES; i++) {
        let fHot = setHotP1.has(i);
        mem.set(P1PT_PA + 4 * i, makePTE(dataPFN(),
            pickWeighted(rng, fHot? PROT_WEIGHTS_HOT : PROT_WEIGHTS), fHot || rng() > 0.10, false));
    }

    /*
     * Seed every byte of every data frame, so that a read is comparing real data from the first
     * operation of the run rather than zero against zero.
     */
    for (let f = 0; f < DATA_FRAMES; f++) {
        for (let off = 0; off < PAGE; off += 4) {
            mem.set(DATA_PA + f * PAGE + off, (Math.floor(rng() * 0x100000000)) | 0);
        }
    }
    return {mem, hotP0, hotP1, hotS0};
}

/**
 * assertLayout()
 *
 * THE INVARIANT.  A page table walk must never read a longword outside a page table, because such
 * a longword is arbitrary data whose PFN could address the ROM or the Qbus I/O page -- which SIMH
 * emulates and this item's Bus does not.  That would be an out-of-scope divergence reported as an
 * MMU bug.
 *
 * The walk reads exactly three kinds of longword, and each is bounded by a length register:
 *   S0:  SBR + 4*vpn, vpn < SLR                         -> inside the system page table
 *   P0:  P0BR + 4*vpn (virtual), vpn < P0LR             -> inside the P0 page table
 *   P1:  P1BR + 4*vpn (virtual), vpn >= P1LR + 0x200000 -> inside the P1 page table
 * plus the system PTE for each of those two virtual addresses, which lands in the system table by
 * the same argument.  So the invariant reduces to "each table fits inside the pages allotted to
 * it", plus "the op stream never writes a malformed PTE and never rewrites a structural mapping",
 * which genOps() enforces directly.
 */
function assertLayout()
{
    let fail = (s) => { throw new Error("layout invariant violated: " + s); };
    if (SBR + 4 * SLR > (SPT_PFN0 + SPT_PAGES) * PAGE) fail("system page table overruns its frames");
    if (P0PT_PA + 4 * P0LR > (P0PT_PFN0 + P0PT_PAGES) * PAGE) fail("P0 page table overruns its frames");
    if (P1PT_PA + 4 * P1PT_ENTRIES > (P1PT_PFN0 + 1) * PAGE) fail("P1 page table overruns its frame");
    if (((P1BR - 0x800000) + 4 * P1_VPN0) !== P1PT_SVA) fail("P1BR bias is wrong");
    if ((P1LR + 0x200000) !== P1_VPN0) fail("P1LR does not select the intended first valid page");
    if (P1PT_SPAGE + 1 !== SLR) fail("the P1 table must straddle the system length limit (for PLNV)");
    if ((DATA_PFN0 + DATA_FRAMES) * PAGE > SBR) fail("data frames collide with the page tables");
}

/* ------------------------------------------------------------------------------------------- *
 * Operation generation
 * ------------------------------------------------------------------------------------------- */

const REGION = {P0: "P0", P1: "P1", S0: "S0", S1: "S1"};

/*
 * Virtual address categories.  The weights are the whole point: a uniform draw over the 4GB
 * virtual space would spend the entire run reporting length violations, and a uniform draw within
 * the mapped regions would almost never cross a page boundary (a page is 512 bytes, so a random
 * longword crosses one about 0.6% of the time).  So offsets are drawn from a distribution that is
 * 40% "within 4 bytes of a page boundary".
 */
const VACAT = [
    ["p0-hot",      22],        // a hot P0 page
    ["p0-any",       5],        // any valid P0 page
    ["p0-lnv",       4],        // beyond P0LR
    ["p1-hot",      16],        // a hot P1 page
    ["p1-any",       4],
    ["p1-plnv",      4],        // P1 page whose PTE is beyond SLR
    ["p1-lnv",       4],        // below P1LR
    ["s0-hot",      22],
    ["s0-any",       5],
    ["s0-pt",        4],        // a page-table page, read-only traffic
    ["s0-lnv",       4],        // beyond SLR
    ["s1",           4],        // 0xC0000000+ : region 3, handled as system space, always LNV
    ["s0-alias",     2]         // the same physical page reached through S0 and through P0
];

/**
 * genVA(rng, layout, cat)
 *
 * @param {function(): number} rng
 * @param {Object} layout
 * @param {string} cat
 * @returns {Object} {va, region}
 */
function genVA(rng, layout, cat)
{
    /*
     * Offsets: 40% within 4 bytes of the END of a page (so lnt 2 and 4 cross), 15% within 4 bytes
     * of the START, 45% anywhere.  Never rounded to a longword -- unaligned is the common case
     * here on purpose.
     */
    let off;
    let r = rng();
    if (r < 0.40) off = PAGE - 4 + Math.floor(rng() * 4);
    else if (r < 0.55) off = Math.floor(rng() * 4);
    else off = Math.floor(rng() * PAGE);

    let pick = (a) => a[Math.floor(rng() * a.length)];
    switch (cat) {
    case "p0-hot":
        return {va: (pick(layout.hotP0) * PAGE + off) | 0, region: REGION.P0};
    case "p0-any":
        return {va: (Math.floor(rng() * P0LR) * PAGE + off) | 0, region: REGION.P0};
    case "p0-lnv":
        return {va: ((P0LR + Math.floor(rng() * P0LR)) * PAGE + off) | 0, region: REGION.P0};
    case "p1-hot":
        return {va: ((P1_VPN0 + pick(layout.hotP1)) * PAGE + off) | 0, region: REGION.P1};
    case "p1-any":
        return {va: ((P1_VPN0 + Math.floor(rng() * P1PT_ENTRIES)) * PAGE + off) | 0, region: REGION.P1};
    case "p1-plnv":
        return {va: ((P1_VPN0 + P1PT_ENTRIES + Math.floor(rng() * 0x100)) * PAGE + off) | 0, region: REGION.P1};
    case "p1-lnv":
        return {va: ((0x200000 + Math.floor(rng() * (P1_VPN0 - 0x200000))) * PAGE + off) | 0, region: REGION.P1};
    case "s0-hot":
        return {va: (0x80000000 + pick(layout.hotS0) * PAGE + off) | 0, region: REGION.S0};
    case "s0-any": {
        let pg;
        do { pg = Math.floor(rng() * SLR); } while (S0_STRUCTURAL.has(pg));
        return {va: (0x80000000 + pg * PAGE + off) | 0, region: REGION.S0};
    }
    case "s0-pt":
        return {va: (SPT_SVA + Math.floor(rng() * SLR) * 4 + (off & 3)) | 0, region: REGION.S0};
    case "s0-lnv":
        return {va: (0x80000000 + (SLR + Math.floor(rng() * SLR)) * PAGE + off) | 0, region: REGION.S0};
    case "s1":
        return {va: (0xC0000000 + Math.floor(rng() * 0x10000) * PAGE + off) | 0, region: REGION.S1};
    case "s0-alias":
        return {va: (0x80000000 + pick(layout.hotS0) * PAGE + off) | 0, region: REGION.S0};
    }
    throw new Error("bad va category " + cat);
}

/**
 * writeWouldHitAPageTable(va, lnt)
 *
 * THE GUARD FOR THE WALK-IN-BOUNDS INVARIANT.  A general data write must never land inside a page
 * table, because it would store arbitrary data where a PTE belongs, and an arbitrary PTE's page
 * frame number can address the boot ROM or the Qbus I/O page -- which SIMH emulates and this
 * item's Bus does not.  The result is an out-of-scope divergence that presents as an MMU bug.
 *
 * This was not hypothetical: the first 150,000-operation run tripped the layout assertion in
 * runExerciser() with `abort -4` (ABORT_MCHK) from SIMH, because the "any S0 page" category could
 * select the S0 window onto the system page table.  Page-table mutation is still exercised, but
 * only through the dedicated churn operation, which writes a WELL-FORMED PTE at a controlled
 * index.
 *
 * Both pages are tested, because an unaligned write near the end of a page stores into two.
 *
 * @param {number} va
 * @param {number} lnt
 * @returns {boolean}
 */
function writeWouldHitAPageTable(va, lnt)
{
    let u = va >>> 0;
    if (u < 0x80000000) return false;                   /* P0/P1 frames come from the data pool */
    let p0 = (u - 0x80000000) >>> 9;
    let p1 = (u + lnt - 1 - 0x80000000) >>> 9;
    return S0_STRUCTURAL.has(p0) || S0_STRUCTURAL.has(p1);
}

/**
 * genOps(rng, layout, nOps)
 *
 * Builds the operation stream.  It also SIMULATES the map-register state as it goes, because two
 * decisions depend on it:
 *
 *   - with MAPEN off, `pa = va & PAMASK`, so a write to a virtual address that happens to alias a
 *     page table would corrupt it.  Unmapped writes are therefore confined to addresses that
 *     alias the data frames -- which still exercises the S0-alias path (0x80100000 and 0x00100000
 *     are the same physical byte) that this whole convention exists for.
 *   - a page-table mutation must never rewrite a structural mapping, and must always write a
 *     WELL-FORMED PTE, or the walk-in-bounds invariant breaks.
 *
 * @param {function(): number} rng
 * @param {Object} layout
 * @param {number} nOps
 * @returns {Array.<Object>}
 */
function genOps(rng, layout, nOps)
{
    let ops = [];
    let mapen = 1;
    let nSinceMapen = 0;

    for (let i = 0; i < nOps; i++) {
        let r = rng();

        /* ---- 2%: an MTPR that perturbs the map registers or the TB ---- */
        if (r < 0.02) {
            let choice = Math.floor(rng() * 9);
            let op;
            switch (choice) {
            case 0: op = {op: OPS.MTPR, prn: IPR.TBIA, val: 0, name: "TBIA"}; break;
            case 1:
            case 2: {
                let t = genVA(rng, layout, pickWeighted(rng, VACAT.map((e) => [e[0], e[1]])));
                op = {op: OPS.MTPR, prn: IPR.TBIS, val: t.va, name: "TBIS"};
                break;
            }
            case 3: op = {op: OPS.MTPR, prn: IPR.P0LR, val: rng() < 0.5? P0LR : (P0LR >> 1), name: "P0LR"}; break;
            case 4: op = {op: OPS.MTPR, prn: IPR.SLR,  val: rng() < 0.5? SLR : (SLR >> 1), name: "SLR"}; break;
            case 5: op = {op: OPS.MTPR, prn: IPR.P1LR, val: rng() < 0.5? P1LR : (P1LR + 0x40), name: "P1LR"}; break;
            case 6: op = {op: OPS.MTPR, prn: IPR.P0BR, val: P0BR, name: "P0BR"}; break;
            case 7: op = {op: OPS.MTPR, prn: IPR.SBR,  val: SBR, name: "SBR"}; break;
            default:
                /*
                 * MAPEN.  Toggled rarely and always turned back on within a few operations: EHKAA
                 * toggles it three times in 335,444 instructions, so a run that spent half its
                 * time unmapped would be testing something the machine barely does.
                 */
                mapen = mapen? 0 : 1;
                op = {op: OPS.MTPR, prn: IPR.MAPEN, val: mapen, name: "MAPEN"};
                nSinceMapen = 0;
                break;
            }
            ops.push(op);
            continue;
        }

        if (!mapen && ++nSinceMapen > 40) {         /* back on; see above */
            mapen = 1;
            nSinceMapen = 0;
            ops.push({op: OPS.MTPR, prn: IPR.MAPEN, val: 1, name: "MAPEN"});
            continue;
        }

        /* ---- 3%: rewrite a PTE through the MMU, under a live TB ---- */
        if (r < 0.05 && mapen) {
            let which = Math.floor(rng() * 3);
            let va, pfn = DATA_PFN0 + Math.floor(rng() * DATA_FRAMES);
            let pte = makePTE(pfn, pickWeighted(rng, PROT_WEIGHTS), rng() > 0.15, false);
            if (which == 0) {
                let idx;
                do { idx = Math.floor(rng() * SLR); } while (S0_STRUCTURAL.has(idx));
                va = (SPT_SVA + 4 * idx) | 0;
            } else if (which == 1) {
                va = (P0BR + 4 * Math.floor(rng() * P0LR)) | 0;
            } else {
                va = (P1PT_SVA + 4 * Math.floor(rng() * P1PT_ENTRIES)) | 0;
            }
            ops.push({op: OPS.WRITE, va, lnt: 4, mode: 0, val: pte, region: REGION.S0, churn: true});
            continue;
        }

        /* ---- everything else: a memory-management operation ---- */
        let lnt = [1, 2, 4][Math.floor(rng() * 3)];
        let mode = MODES[Math.floor(rng() * 4)];
        let kind;
        let rk = rng();
        if (rk < 0.38) kind = OPS.READ;
        else if (rk < 0.70) kind = OPS.WRITE;
        else if (rk < 0.85) kind = OPS.TESTR;
        else kind = OPS.TESTW;

        let va, region;
        if (!mapen) {
            /*
             * Unmapped: address a data frame directly, half the time through its S0 alias, which
             * is the case defines.js rule 1 exists for (0x80100000 & PAMASK == 0x00100000).
             */
            let pa = DATA_PA + Math.floor(rng() * (DATA_FRAMES * PAGE - 8));
            va = (rng() < 0.5? pa : (pa + 0x80000000)) | 0;
            region = (va < 0)? REGION.S0 : REGION.P0;
        } else {
            let cat = pickWeighted(rng, VACAT.map((e) => [e[0], e[1]]));
            let g = genVA(rng, layout, cat);
            va = g.va;
            region = g.region;
            /* never clobber a page table with arbitrary data; see writeWouldHitAPageTable() */
            if (kind == OPS.WRITE && writeWouldHitAPageTable(va, lnt)) kind = OPS.READ;
        }
        /*
         * The value handed to a write is masked to its length HERE, once, for both sides.
         *
         * This is not cosmetic.  Open SIMH's aligned WriteB and WriteW (vax_mmu.h:341, 357) do
         * `M[id] = (M[id] & ~mask) | (val << sc)` and do NOT mask `val` first -- they rely on
         * every caller having already trimmed it, which every instruction body does.  Feed one an
         * untrimmed longword and its high bytes are OR'd into the NEIGHBOURING bytes of the same
         * longword.  BusVAX.setByte()/setWord() mask defensively, so the two disagree on an input
         * no real caller can produce.  Trimming here grades mmu.js on inputs the machine can
         * actually see, instead of on an undefined behaviour of the oracle.  (This cost an hour;
         * it presented as data landing one byte low, which looks exactly like an addressing bug.)
         */
        let vmask = lnt == 1? 0xFF : (lnt == 2? 0xFFFF : -1);
        ops.push({
            op: kind, va, lnt: (kind == OPS.TESTR || kind == OPS.TESTW)? 1 : lnt, mode,
            val: (Math.floor(rng() * 0x100000000) & vmask) | 0, region, mapen
        });
    }
    return ops;
}

/* ------------------------------------------------------------------------------------------- *
 * The JS side
 * ------------------------------------------------------------------------------------------- */

/**
 * makeMachine(layout, mutation)
 *
 * @param {Object} layout
 * @param {string} mutation
 * @returns {Object} {bus, mmu}
 */
function makeMachine(layout, mutation)
{
    let bus = new BusVAX({'busWidth': VAX.PAWIDTH, 'id': "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    for (let [pa, val] of layout.mem) bus.setLong(pa, val);

    let mmu = new MMUVAX(bus);
    mmu.setP0BR(P0BR);
    mmu.setP0LR(P0LR);
    mmu.setP1BR(P1BR);
    mmu.setP1LR(P1LR);
    mmu.setSBR(SBR);
    mmu.setSLR(SLR);
    mmu.setMAPEN(1);
    if (mutation) applyMutation(mmu, mutation);
    return {bus, mmu};
}

/**
 * jsMTPR(mmu, prn, val)
 *
 * The MTPR dispatch for the map registers, which belongs to the CPU's IPR item and is stubbed
 * here so this test can drive the same mutators SIMH's op_mtpr() drives.  Deliberately NOT part
 * of mmu.js: what an MTPR does to the rest of the machine (condition codes, IPL, the interrupt
 * request state) is not the MMU's business.
 *
 * @param {MMUVAX} mmu
 * @param {number} prn
 * @param {number} val
 */
function jsMTPR(mmu, prn, val)
{
    switch (prn) {
    case IPR.P0BR:  mmu.setP0BR(val); break;
    case IPR.P0LR:  mmu.setP0LR(val); break;
    case IPR.P1BR:  mmu.setP1BR(val); break;
    case IPR.P1LR:  mmu.setP1LR(val); break;
    case IPR.SBR:   mmu.setSBR(val); break;
    case IPR.SLR:   mmu.setSLR(val); break;
    case IPR.MAPEN: mmu.setMAPEN(val); break;
    case IPR.TBIA:  mmu.zapTB(1); break;
    case IPR.TBIS:  mmu.zapTBEnt(val); break;
    default: throw new Error("jsMTPR: unexpected IPR " + prn);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Phase 1: the randomized exerciser
 * ------------------------------------------------------------------------------------------- */

/**
 * newStats()
 *
 * @returns {Object}
 */
function newStats()
{
    return {
        nOps: 0, nCompared: 0,
        byRegion: {P0: 0, P1: 0, S0: 0, S1: 0},
        byMode: [0, 0, 0, 0],
        byOp: [0, 0, 0, 0, 0],
        byFault: [0, 0, 0, 0, 0, 0, 0, 0],
        nCrossPage: 0, nUnaligned: 0, nCrossLong: 0,
        nTwoLevel: 0, nMBit: 0, nFills: 0, nTbHits: 0,
        nValueChecks: 0, nValueNonZero: 0, nPaChecks: 0,
        nOK: 0, nFaulted: 0, nUnmapped: 0,
        nSweep: 0, nSweepNonZero: 0
    };
}

/**
 * buildExerciserIni(layout, ops)
 *
 * @param {Object} layout
 * @param {Array.<Object>} ops
 * @returns {string}
 */
function buildExerciserIni(layout, ops)
{
    let L = ["set cpu " + (MEMSIZE >> 20) + "m"];
    for (let [pa, val] of layout.mem) {
        L.push("dep -l " + hex(pa) + " " + hex(val, 8));
    }
    /*
     * The map registers are loaded with MTPR, not with a console deposit, because a deposit does
     * NOT run SIMH's zap_tb()/set_map_reg() and would leave the two sides' TBs out of step from
     * the very first operation.
     */
    L.push("show cpu mmuop=4:" + hex(P0BR) + ":1:0:" + hex(IPR.P0BR));
    L.push("show cpu mmuop=4:" + hex(P0LR) + ":1:0:" + hex(IPR.P0LR));
    L.push("show cpu mmuop=4:" + hex(P1BR) + ":1:0:" + hex(IPR.P1BR));
    L.push("show cpu mmuop=4:" + hex(P1LR) + ":1:0:" + hex(IPR.P1LR));
    L.push("show cpu mmuop=4:" + hex(SBR) + ":1:0:" + hex(IPR.SBR));
    L.push("show cpu mmuop=4:" + hex(SLR) + ":1:0:" + hex(IPR.SLR));
    L.push("show cpu mmuop=4:1:1:0:" + hex(IPR.MAPEN));
    let nPrologue = 7;

    for (let o of ops) {
        if (o.op == OPS.MTPR) {
            L.push("show cpu mmuop=4:" + hex(o.val) + ":1:0:" + hex(o.prn));
        } else {
            L.push("show cpu mmuop=" + o.op + ":" + hex(o.va) + ":" + o.lnt + ":" + o.mode +
                ":" + hex(o.val >>> 0));
        }
    }

    /*
     * The read-back sweep.  Everything the run could have written -- all three page tables and
     * every data frame -- is examined on both sides afterwards, so no write and no M-bit
     * write-back can pass unverified, and a write that landed at the wrong physical address is
     * caught even if nothing ever read it back through the MMU.
     */
    let sweep = [
        [SBR, SBR + 4 * SLR - 4],
        [P0PT_PA, P0PT_PA + 4 * P0LR - 4],
        [P1PT_PA, P1PT_PA + 4 * P1PT_ENTRIES - 4],
        [DATA_PA, DATA_PA + DATA_FRAMES * PAGE - 4]
    ];
    for (let s of sweep) L.push("e -l " + hex(s[0]) + "-" + hex(s[1]));
    L.push("quit", "");
    return {text: L.join("\n"), nPrologue, sweep};
}

/**
 * runExerciser(simhBin, nOps, seed, mutation, scratch, fQuiet)
 *
 * @param {string} simhBin
 * @param {number} nOps
 * @param {number} seed
 * @param {string} mutation
 * @param {string} scratch
 * @param {boolean} fQuiet
 * @returns {Object} {failures, stats}
 */
function runExerciser(simhBin, nOps, seed, mutation, scratch, fQuiet)
{
    assertLayout();
    let rng = mulberry32(seed);
    let layout = buildLayout(rng);
    let ops = genOps(rng, layout, nOps);
    let stats = newStats();
    let failures = [];
    let fail = (s) => { if (failures.length < 20) failures.push(s); };

    /* ---- run SIMH ---- */
    let {text, nPrologue, sweep} = buildExerciserIni(layout, ops);
    let out = runSimh(simhBin, text, path.join(scratch, "exerciser.ini"));
    let lines = out.split("\n");

    let aMmuop = [], aExam = [], aErrors = [];
    for (let line of lines) {
        if (line.startsWith("MMUOP ")) aMmuop.push(line.trim());
        else if (/^[0-9A-F]{1,8}:\t[0-9A-F]{8}\r?$/.test(line)) aExam.push(line.trim());
        else if (/%SIM-ERROR/.test(line)) aErrors.push(line.trim());
    }
    for (let e of aErrors) fail("SIMH reported an error: " + e);

    /*
     * NOTHING MAY BE SILENTLY DROPPED.  Every operation must have produced exactly one MMUOP
     * line.  If the counts differ, say which operation lost its line rather than quietly
     * comparing a shifted stream -- a shifted stream would still "pass" for a while.
     */
    if (aMmuop.length != nPrologue + ops.length) {
        fail("SIMH produced " + aMmuop.length + " MMUOP lines for " + (nPrologue + ops.length) +
            " operations; " + (nPrologue + ops.length - aMmuop.length) + " were dropped");
        /* fall through and compare what we can, so the first divergence is still localized */
    }

    /* ---- replay on the JS side, comparing as we go ---- */
    let {bus, mmu} = makeMachine(layout, mutation);
    let nFillBefore, stat = {code: 0};
    let origFill = mmu.fill.bind(mmu);
    mmu.fill = function(va, lnt, acc, st) { stats.nFills++; return origFill(va, lnt, acc, st); };

    for (let i = 0; i < ops.length; i++) {
        let o = ops[i];
        let line = aMmuop[nPrologue + i];
        let where = "op " + i + " " + describeOp(o);
        if (!line) { fail(where + ": SIMH produced no MMUOP line"); continue; }

        let m = line.match(/^MMUOP (\d+) ([0-9A-F]{8}) (\d+) ([0-9A-F]{8}) (ok|stat|abort) (.+)$/);
        if (!m) { fail(where + ": unparseable SIMH line '" + line + "'"); continue; }
        if (+m[1] != o.op || (parseInt(m[2], 16) | 0) != (((o.op == OPS.MTPR)? o.val : o.va) | 0)) {
            fail(where + ": MMUOP stream desynchronized (SIMH echoed op " + m[1] + " va " + m[2] + ")");
            continue;
        }

        stats.nOps++;
        stats.byOp[o.op]++;
        if (o.op == OPS.MTPR) {
            jsMTPR(mmu, o.prn, o.val);
            stats.nCompared++;
            continue;
        }
        stats.byRegion[o.region]++;
        stats.byMode[o.mode]++;
        if (!o.mapen) stats.nUnmapped++;
        if (o.lnt > 1) {
            if ((o.va & (o.lnt - 1)) != 0) stats.nUnaligned++;
            if (o.mapen && ((o.va & MMUVAX.VA_M_OFF) + o.lnt) > MMUVAX.VA_PAGSIZE) stats.nCrossPage++;
            else if (((o.va & 3) + o.lnt) > 4) stats.nCrossLong++;
        }

        nFillBefore = stats.nFills;
        let acc = (o.op == OPS.TESTW || o.op == OPS.WRITE)?
            MMUVAX.accWrite(o.mode) : MMUVAX.accRead(o.mode);

        let got;                                    // {kind: "ok"|"stat"|"abort", ...}
        try {
            if (o.op == OPS.TESTR || o.op == OPS.TESTW) {
                stat.code = MMUVAX.PR.OK;
                let pa = mmu.test(o.va, acc, stat);
                got = (pa < 0)? {kind: "stat", code: stat.code} : {kind: "ok", val: pa};
            } else if (o.op == OPS.READ) {
                got = {kind: "ok", val: mmu.readData(o.va, o.lnt, acc)};
            } else {
                mmu.writeData(o.va, o.val, o.lnt, acc);
                got = {kind: "ok", val: o.val};
            }
        } catch (e) {
            if (!(e instanceof VAXFault)) throw e;
            got = {kind: "abort", code: e.code, p1: e.p1, p2: e.p2};
        }
        if (stats.nFills > nFillBefore) {
            if (o.region == REGION.P0 || o.region == REGION.P1) stats.nTwoLevel++;
        } else {
            stats.nTbHits++;
        }
        if (bus.checkFault()) {
            fail(where + ": the Bus reported a non-existent-memory fault, which the layout" +
                " invariant is supposed to make impossible (physical address escaped RAM)");
        }

        /* ---- compare ---- */
        stats.nCompared++;
        let kind = m[5], rest = m[6].trim();
        if (kind != got.kind) {
            fail(where + ": SIMH=" + kind + " " + rest + "  JS=" + describeResult(got));
            continue;
        }
        if (kind == "ok") {
            let expect = parseInt(rest, 16) >>> 0;
            if (o.op == OPS.WRITE) {
                /* nothing to compare directly; the read-back sweep grades the store */
            } else {
                stats.nValueChecks++;
                if (expect) stats.nValueNonZero++;
                if (o.op == OPS.TESTR || o.op == OPS.TESTW) stats.nPaChecks++;
                if ((got.val >>> 0) !== expect) {
                    fail(where + ": value SIMH=0x" + hex(expect, 8) + " JS=0x" + hex(got.val, 8));
                }
            }
        } else if (kind == "stat") {
            let code = +rest;
            stats.byFault[code]++;
            if (code !== got.code) {
                fail(where + ": probe status SIMH=" + PR_NAMES[code] + " JS=" + PR_NAMES[got.code]);
            }
        } else {
            let a = rest.split(/\s+/);
            let code = +a[0], p1 = parseInt(a[1], 16) >>> 0, p2 = parseInt(a[2], 16) >>> 0;
            stats.nFaulted++;
            stats.byFault[(p1 & 3) | ((code == VAXFAULT.TNV)? 4 : 0)]++;
            if (code !== got.code || p1 !== (got.p1 >>> 0) || p2 !== (got.p2 >>> 0)) {
                fail(where + ": abort SIMH=(" + code + "," + hex(p1, 8) + "," + hex(p2, 8) +
                    ") JS=(" + got.code + "," + hex(got.p1, 8) + "," + hex(got.p2, 8) + ")");
            }
        }
        if (kind == "ok") stats.nOK++;
    }

    /* ---- read-back sweep ---- */
    let iExam = 0;
    for (let s of sweep) {
        for (let pa = s[0]; pa <= s[1]; pa += 4) {
            let line = aExam[iExam++];
            if (!line) { fail("read-back sweep: SIMH produced no line for 0x" + hex(pa, 8)); continue; }
            let m2 = line.match(/^([0-9A-F]+):\t([0-9A-F]{8})$/);
            if (!m2 || (parseInt(m2[1], 16) | 0) != pa) {
                fail("read-back sweep desynchronized at 0x" + hex(pa, 8) + " ('" + line + "')");
                continue;
            }
            let expect = parseInt(m2[2], 16) >>> 0;
            let actual = bus.getLong(pa) >>> 0;
            stats.nSweep++;
            if (expect) stats.nSweepNonZero++;
            if (expect !== actual) {
                fail("read-back @0x" + hex(pa, 8) + ": SIMH=0x" + hex(expect, 8) + " JS=0x" + hex(actual, 8));
            }
        }
    }
    if (iExam < aExam.length) fail("read-back sweep: " + (aExam.length - iExam) + " unmatched examine lines");

    /*
     * The M-bit count is derived, not observed: a PTE that started with M clear and ended with it
     * set is one write-back.  Counting it here rather than instrumenting fill() keeps the shipped
     * code free of test hooks.
     */
    for (let [pa, val] of layout.mem) {
        if (pa >= SBR && pa < SBR + 4 * SLR || pa >= P0PT_PA && pa < P0PT_PA + 4 * P0LR ||
            pa >= P1PT_PA && pa < P1PT_PA + 4 * P1PT_ENTRIES) {
            if (!(val & MMUVAX.PTE_M) && (bus.getLong(pa) & MMUVAX.PTE_M)) stats.nMBit++;
        }
    }

    if (!fQuiet) {
        console.log("  ops=%d compared=%d  regions=%s  modes=[K %d, E %d, S %d, U %d]",
            stats.nOps, stats.nCompared, JSON.stringify(stats.byRegion),
            stats.byMode[0], stats.byMode[1], stats.byMode[2], stats.byMode[3]);
        console.log("  by op: test-r=%d test-w=%d read=%d write=%d mtpr=%d   unmapped=%d",
            stats.byOp[0], stats.byOp[1], stats.byOp[2], stats.byOp[3], stats.byOp[4], stats.nUnmapped);
        console.log("  cross-page=%d cross-longword=%d unaligned=%d   TB fills=%d (two-level %d) TB hits=%d M-bit write-backs=%d",
            stats.nCrossPage, stats.nCrossLong, stats.nUnaligned, stats.nFills, stats.nTwoLevel,
            stats.nTbHits, stats.nMBit);
        console.log("  faults: %s   operations that succeeded: %d",
            [0, 1, 3, 4, 6].map((i) => PR_NAMES[i] + "=" + stats.byFault[i]).join(" "), stats.nOK);
        console.log("  comparisons: value=%d (non-zero %d) probe-pa=%d read-back=%d (non-zero %d)",
            stats.nValueChecks, stats.nValueNonZero, stats.nPaChecks, stats.nSweep, stats.nSweepNonZero);
    }
    return {failures, stats};
}

/**
 * describeOp(o)
 *
 * @param {Object} o
 * @returns {string}
 */
function describeOp(o)
{
    if (o.op == OPS.MTPR) return "MTPR " + o.name + "=0x" + hex(o.val, 8);
    return ["test-r", "test-w", "read", "write"][o.op] + " va=0x" + hex(o.va, 8) +
        " lnt=" + o.lnt + " mode=" + MODE_NAMES[o.mode] + " (" + o.region + ")";
}

/**
 * describeResult(r)
 *
 * @param {Object} r
 * @returns {string}
 */
function describeResult(r)
{
    if (r.kind == "ok") return "ok 0x" + hex(r.val, 8);
    if (r.kind == "stat") return "stat " + PR_NAMES[r.code];
    return "abort " + r.code + " " + hex(r.p1, 8) + " " + hex(r.p2, 8);
}

/* ------------------------------------------------------------------------------------------- *
 * Phase 2: replaying EHKAA's memory management
 * ------------------------------------------------------------------------------------------- */

/**
 * StubMemory
 *
 * A sparse stand-in for physical memory, used ONLY by the EHKAA phase.  It implements the four
 * Bus methods MMUVAX calls and records every address it is asked for, so the replay can assert
 * that our page table walk read exactly the longwords SIMH's did.  Reads of addresses the trace
 * never mentioned return 0, which is a deliberately hostile default: a walk that computes the
 * wrong PTE address gets an all-zero PTE, i.e. protection code 0, i.e. an immediate ACV -- a
 * loud, specific divergence rather than a plausible wrong answer.
 *
 * This is the one MOCK in this file, and it is a mock of MEMORY, never of the MMU: the code under
 * test is the shipped MMUVAX, unmodified.
 *
 * @class StubMemory
 */
class StubMemory {
    constructor()
    {
        this.mem = new Map();
        this.reads = [];
        this.writes = [];
    }
    reset() { this.reads.length = 0; this.writes.length = 0; }
    getLong(pa) { pa = (pa >>> 0) & VAX.PAMASK; this.reads.push(pa); return this.mem.get(pa) | 0; }
    setLong(pa, v) { pa = (pa >>> 0) & VAX.PAMASK; this.writes.push(pa); this.mem.set(pa, v | 0); }
    getByte(pa) { pa = (pa >>> 0) & VAX.PAMASK; this.reads.push(pa & ~3); return (this.getRaw(pa & ~3) >>> ((pa & 3) << 3)) & 0xFF; }
    getWord(pa) { pa = (pa >>> 0) & VAX.PAMASK; this.reads.push(pa & ~3); return (this.getRaw(pa & ~3) >>> ((pa & 2)? 16 : 0)) & 0xFFFF; }
    setByte(pa, v) { pa = (pa >>> 0) & VAX.PAMASK; this.writes.push(pa & ~3); }
    setWord(pa, v) { pa = (pa >>> 0) & VAX.PAMASK; this.writes.push(pa & ~3); }
    getRaw(pa) { return this.mem.get(pa) | 0; }
    checkFault() { return false; }
}

/**
 * parseMmuTrace(tracePath, onOp)
 *
 * Streams the MMU trace produced by SET CPU MMUTRACE=, grouping it into operations.  Streaming
 * rather than slurping matters: a full EHKAA capture is 14MB and 776,000 lines.
 *
 * Grammar (all fields hex unless noted; see machines/dec/vax/tests/simh/README.md):
 *
 *      G p0br p0lr p1br p1lr sbr slr mapen      set_map_reg() ran
 *      Z stb                                    zap_tb(stb)            (stb decimal)
 *      Y va                                     zap_tb_ent(va)
 *      O <R|W|T> va lnt acc mapen               a Read/Write/Test began   (lnt, mapen decimal)
 *        F va lnt acc                           fill() was entered
 *        S ptead pte                            a SYSTEM pte was read, for a process pte address
 *        P ptead pte                            the page's pte was read
 *        M ptead pte                            the M bit was written back
 *        E code                                 fill() reported/raised a fault  (code decimal)
 *        R tlbpte                               fill() succeeded
 *      A pa                                     the resolved physical address
 *      B pa1                                    the second physical address (unaligned only)
 *      X status                                 Test() failed  (decimal)
 *
 * @param {string} tracePath
 * @param {function(Object): void} onEvent
 * @returns {Promise}
 */
async function parseMmuTrace(tracePath, onEvent)
{
    const rl = readline.createInterface({
        input: fs.createReadStream(tracePath, {encoding: "utf8", highWaterMark: 1 << 20}),
        crlfDelay: Infinity
    });
    let op = null;
    let flush = () => { if (op) { onEvent(op); op = null; } };
    for await (const line of rl) {
        if (!line) continue;
        let a = line.split(" ");
        switch (a[0]) {
        case "O":
            flush();
            op = {kind: "op", rw: a[1], va: parseInt(a[2], 16) | 0, lnt: +a[3],
                  acc: parseInt(a[4], 16) | 0, mapen: +a[5],
                  fills: [], ptes: [], mwrites: [], fault: null, pa: null, pa1: null, xstat: null};
            break;
        case "F": if (op) op.fills.push({va: parseInt(a[1], 16) | 0, lnt: +a[2], acc: parseInt(a[3], 16) | 0}); break;
        case "S": if (op) op.ptes.push({addr: parseInt(a[1], 16) | 0, val: parseInt(a[2], 16) | 0, sys: true}); break;
        case "P": if (op) op.ptes.push({addr: parseInt(a[1], 16) | 0, val: parseInt(a[2], 16) | 0, sys: false}); break;
        case "M": if (op) op.mwrites.push({addr: parseInt(a[1], 16) | 0, val: parseInt(a[2], 16) | 0}); break;
        case "E": if (op) op.fault = +a[1]; break;
        case "R": break;
        case "A": if (op) op.pa = parseInt(a[1], 16) | 0; break;
        case "B": if (op) op.pa1 = parseInt(a[1], 16) | 0; break;
        case "X": if (op) op.xstat = +a[1]; break;
        case "G":
            flush();
            onEvent({kind: "regs", p0br: parseInt(a[1], 16) | 0, p0lr: parseInt(a[2], 16) | 0,
                     p1br: parseInt(a[3], 16) | 0, p1lr: parseInt(a[4], 16) | 0,
                     sbr: parseInt(a[5], 16) | 0, slr: parseInt(a[6], 16) | 0, mapen: +a[7]});
            break;
        case "Z": flush(); onEvent({kind: "zap", stb: +a[1]}); break;
        case "Y": flush(); onEvent({kind: "zapent", va: parseInt(a[1], 16) | 0}); break;
        default: break;
        }
    }
    flush();
}

/**
 * runEhkaa(simhBin, opts, mutation)
 *
 * @param {string} simhBin
 * @param {Object} opts
 * @param {string} mutation
 * @returns {Promise<Object>}
 */
async function runEhkaa(simhBin, opts, mutation)
{
    let tracePath = opts.trace;
    if (!tracePath) {
        if (!fs.existsSync(opts.ehkaaExe)) {
            throw new Error("EHKAA diagnostic not found at " + opts.ehkaaExe +
                "; pass --ehkaa PATH or --skip-ehkaa");
        }
        tracePath = path.join(opts.scratch, "ehkaa-mmu.trace");
        let script = [
            "set cpu mmutrace=" + tracePath,
            "load " + opts.ehkaaExe,
            "go -q 200",
            "examine PC",
            "set cpu nommutrace",
            "exit", ""
        ].join("\n");
        let out = runSimh(simhBin, script, path.join(opts.scratch, "ehkaa.ini"));
        if (!/PC:\s*80018AD1/.test(out)) {
            throw new Error("EHKAA did not halt at its documented PASS PC (0x80018AD1); SIMH said:\n" + out);
        }
    }

    let stub = new StubMemory();
    let mmu = new MMUVAX(stub);
    if (mutation) applyMutation(mmu, mutation);

    let failures = [];
    let fail = (s) => { if (failures.length < 20) failures.push(s); };
    let st = {
        nEvents: 0, nOps: 0, nCompared: 0, nMapped: 0, nUnmapped: 0,
        byRW: {R: 0, W: 0, T: 0}, nFills: 0, nTwoLevel: 0, nFaults: 0, nProbeFail: 0,
        nCrossPage: 0, nUnaligned: 0, nMBit: 0, nZap: 0, nZapEnt: 0, nMapenOn: 0, nMapenOff: 0,
        byFault: [0, 0, 0, 0, 0, 0, 0, 0], byRegion: {P0: 0, P1: 0, S0: 0, S1: 0}, nFar: 0,
        readAcc: [0, 0, 0, 0], writeAcc: [0, 0, 0, 0], nPaChecks: 0, nPa1Checks: 0,
        nPteAddrChecks: 0, nSkipped: 0
    };
    let lastMapen = -1;
    let iOp = 0;

    await parseMmuTrace(tracePath, (ev) => {
        st.nEvents++;
        if (ev.kind == "regs") {
            /*
             * set_map_reg() ran on SIMH's side.  Load the architectural registers straight in and
             * recompute the dynamic copies -- deliberately NOT through setP0BR() and friends,
             * because those also flush the TB and SIMH's set_map_reg() does not.  The flushes
             * arrive as their own Z records.
             */
            mmu.p0br = ev.p0br; mmu.p0lr = ev.p0lr;
            mmu.p1br = ev.p1br; mmu.p1lr = ev.p1lr;
            mmu.sbr = ev.sbr;   mmu.slr = ev.slr;
            if (ev.mapen != lastMapen) {
                if (ev.mapen) st.nMapenOn++; else st.nMapenOff++;
                lastMapen = ev.mapen;
            }
            mmu.mapen = ev.mapen;
            mmu.setMapReg();
            return;
        }
        if (ev.kind == "zap") { st.nZap++; mmu.zapTB(ev.stb); return; }
        if (ev.kind == "zapent") { st.nZapEnt++; mmu.zapTBEnt(ev.va); return; }

        /* ---- an operation ---- */
        let i = iOp++;
        st.nOps++;
        st.byRW[ev.rw]++;
        mmu.mapen = ev.mapen;
        if (ev.mapen) st.nMapped++; else st.nUnmapped++;
        st.byRegion[["P0", "P1", "S0", "S1"][ev.va >>> 30]]++;
        for (let m = 0; m < 4; m++) {
            if (ev.acc & (1 << m)) st.readAcc[m]++;
            if (ev.acc & (1 << (m + 4))) st.writeAcc[m]++;
        }
        st.nFills += ev.fills.length;
        if (ev.ptes.some((p) => p.sys)) st.nTwoLevel++;
        st.nMBit += ev.mwrites.length;
        if (ev.fault !== null) {
            st.nFaults++;
            st.byFault[ev.fault]++;
            /* a fault raised by the SECOND fill of a cross-page access */
            if (ev.fills.length > 1) st.nFar++;
        }
        if (ev.xstat !== null) st.nProbeFail++;
        if (ev.lnt > 1 && (ev.va & (ev.lnt - 1))) st.nUnaligned++;
        if (ev.mapen && ev.lnt > 1 && ((ev.va & MMUVAX.VA_M_OFF) + ev.lnt) > MMUVAX.VA_PAGSIZE) st.nCrossPage++;

        /*
         * Install the PTE values SIMH read.  This is the fill oracle: it makes the translation
         * reproducible without modelling EHKAA's 16MB of memory, and it is why the PTE ADDRESS
         * check below is the load-bearing assertion rather than the value check.
         */
        for (let p of ev.ptes) stub.mem.set(p.addr & VAX.PAMASK, p.val);
        stub.reset();

        let where = "ehkaa op " + i + " " + ev.rw + " va=0x" + hex(ev.va, 8) + " lnt=" + ev.lnt +
            " acc=0x" + hex(ev.acc, 8) + (ev.mapen? "" : " [mapen off]");

        let threw = null, probeStat = {code: MMUVAX.PR.OK}, pa = null;
        try {
            if (ev.rw == "T") {
                pa = mmu.test(ev.va, ev.acc, probeStat);
            } else if (ev.rw == "R") {
                mmu.readData(ev.va, ev.lnt, ev.acc);
            } else {
                mmu.writeData(ev.va, 0, ev.lnt, ev.acc);
            }
        } catch (e) {
            if (!(e instanceof VAXFault)) throw e;
            threw = e;
        }
        st.nCompared++;

        /* (3) faulted exactly when SIMH faulted */
        if (ev.fault !== null && ev.xstat === null) {
            if (!threw) {
                fail(where + ": SIMH raised " + PR_NAMES[ev.fault] + "; JS did not fault");
                return;
            }
            let expect = (ev.fault & MMUVAX.PR.TNV)? VAXFAULT.TNV : VAXFAULT.ACV;
            if (threw.code != expect) {
                fail(where + ": fault code SIMH=" + expect + " JS=" + threw.code);
            }
            /*
             * The fault parameters are those of the fill() that FAILED, which is not always the
             * one for the address the access started at: an unaligned access that crosses a page
             * boundary calls fill() a second time for `va + lnt`, and it is that address the
             * exception reports.  The last `F` record before the `E` is therefore the authority,
             * not the `O` record.  (Assuming the `O` address here produced six failures on real
             * EHKAA traffic and nothing at all on the randomized phase, which never happened to
             * fault on the far side of a page crossing -- exactly the kind of case a synthetic
             * generator misses and a real workload does not.)
             */
            let ff = ev.fills.length? ev.fills[ev.fills.length - 1] : {va: ev.va, acc: ev.acc};
            let p1 = ((ff.acc & MMUVAX.TLB_WACC)? 4 : 0) | (ev.fault & 3);
            if ((threw.p1 >>> 0) != p1 || (threw.p2 | 0) != ff.va) {
                fail(where + ": fault parameters SIMH=(" + hex(p1, 8) + "," + hex(ff.va, 8) +
                    ") JS=(" + hex(threw.p1, 8) + "," + hex(threw.p2, 8) + ")");
            }
            return;
        }
        if (ev.xstat !== null) {                    /* a probe that was refused */
            if (threw) { fail(where + ": SIMH's probe returned status " + PR_NAMES[ev.xstat] + "; JS faulted"); return; }
            if (pa !== -1) { fail(where + ": SIMH's probe was refused; JS returned pa 0x" + hex(pa, 8)); return; }
            if (probeStat.code != ev.xstat) {
                fail(where + ": probe status SIMH=" + PR_NAMES[ev.xstat] + " JS=" + PR_NAMES[probeStat.code]);
            }
            return;
        }
        if (threw) {
            fail(where + ": JS faulted (" + threw.code + ") where SIMH did not");
            return;
        }

        /* (1) the physical addresses match */
        if (ev.pa === null) { st.nSkipped++; fail(where + ": trace carries no A record"); return; }
        st.nPaChecks++;
        if ((mmu.pa | 0) !== ev.pa) {
            fail(where + ": physical address SIMH=0x" + hex(ev.pa, 8) + " JS=0x" + hex(mmu.pa, 8));
            return;
        }
        if (ev.pa1 !== null) {
            st.nPa1Checks++;
            if ((mmu.pa1 | 0) !== ev.pa1) {
                fail(where + ": second physical address SIMH=0x" + hex(ev.pa1, 8) +
                    " JS=0x" + hex(mmu.pa1, 8));
                return;
            }
        }

        /* (2) the page table walk read exactly the longwords SIMH's did, in order */
        for (let k = 0; k < ev.ptes.length; k++) {
            st.nPteAddrChecks++;
            let want = ev.ptes[k].addr & VAX.PAMASK;
            if (stub.reads[k] !== want) {
                fail(where + ": page-table read " + k + " SIMH=0x" + hex(want, 8) +
                    " JS=0x" + hex(stub.reads[k] === undefined? -1 : stub.reads[k], 8));
                return;
            }
        }
        for (let k = 0; k < ev.mwrites.length; k++) {
            let want = ev.mwrites[k].addr & VAX.PAMASK;
            if (stub.writes[k] !== want) {
                fail(where + ": M-bit write-back " + k + " SIMH=0x" + hex(want, 8) +
                    " JS=0x" + hex(stub.writes[k] === undefined? -1 : stub.writes[k], 8));
                return;
            }
        }
    });

    if (st.nOps != st.nCompared) {
        fail("EHKAA: " + (st.nOps - st.nCompared) + " of " + st.nOps + " operations never reached comparison");
    }
    return {failures, stats: st, tracePath};
}

/* ------------------------------------------------------------------------------------------- *
 * Mutations -- injected into the SHIPPED code path
 *
 * Each replacement below is a VERBATIM copy of the corresponding method in mmu.js with exactly
 * one expression changed, and the change is named in its comment.  Copying rather than wrapping
 * is deliberate: a wrapper proves the test can detect a wrapper.
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = [
    "signedPtidx",      // THE hazard: `>>` instead of `>>>` on the page table index
    "accessRead",       // the fill access check ignores the write half of the permission mask
    "noModifyBit",      // the M bit is set in the TB but never written back to the PTE
    "sharedTB",         // one TB instead of the architectural system/process split
    "crossPageNever",   // an unaligned access never translates its second page
    "pa1Unaligned",     // the second fragment's address is not truncated to a longword
    "faultParamRW",     // the fault parameter loses its write bit
    "zapProcessOnly"    // MTPR SBR/SLR/MAPEN flushes only the process TB
];

/*
 * A MUTATION THAT WAS TRIED AND REJECTED, because it is worth knowing.
 *
 * Changing the page-crossing test from `(off + lnt) > VA_PAGSIZE` to `>=` looks like a textbook
 * off-by-one and is provably NOT a defect.  The two differ only when `off + lnt == 512`, i.e.
 * off == 512 - lnt; for every length that reaches this code (1, 2, 4) that offset is a multiple
 * of the length, and a page frame is 512-byte aligned so `pa` is congruent to `off` modulo 4.
 * The access is therefore naturally aligned and has already returned from the aligned fast path
 * before the crossing test is reached -- the branch is unreachable, and the "bug" cannot be
 * observed by any oracle.  This is the same shape of result as busdiff.js's rejected
 * `noNormalize` mutation: an undetected mutation must be INVESTIGATED, not papered over with
 * more operations.  `crossPageNever` and `pa1Unaligned` replaced it.
 */

/**
 * applyMutation(mmu, name)
 *
 * @param {MMUVAX} mmu
 * @param {string} name
 */
function applyMutation(mmu, name)
{
    const VA_S0 = MMUVAX.VA_S0, VA_P1 = MMUVAX.VA_P1;
    const VA_M_OFF = MMUVAX.VA_M_OFF, VA_V_VPN = MMUVAX.VA_V_VPN, VA_M_VPN = MMUVAX.VA_M_VPN;
    const VA_M_TBI = MMUVAX.VA_M_TBI, VA_PAGSIZE = MMUVAX.VA_PAGSIZE;
    const PTE_V = MMUVAX.PTE_V, PTE_M = MMUVAX.PTE_M, PTE_V_ACC = MMUVAX.PTE_V_ACC;
    const TLB_M = MMUVAX.TLB_M, TLB_PFN = MMUVAX.TLB_PFN, TLB_WACC = MMUVAX.TLB_WACC;
    const CVTACC = MMUVAX.CVTACC, L_BYTE = 1, L_WORD = 2, L_LONG = 4;
    const INSERT = [0x00000000, 0x000000FF, 0x0000FFFF, 0x00FFFFFF];

    if (name == "faultParamRW") {
        mmu.mmErr = function(code, va, acc, stat) {
            if (stat) { stat.code = code; return 0; }
            this.p1 = (code & 3);                       /* CHANGED: dropped the write bit */
            this.p2 = va;
            throw new VAXFault((code & 4)? VAXFAULT.TNV : VAXFAULT.ACV, this.p1, this.p2);
        };
        return;
    }
    if (name == "zapProcessOnly") {
        mmu.setSBR = function(val) { this.sbr = val & MMUVAX.BR_MASK; this.zapTB(0); this.setMapReg(); };
        mmu.setSLR = function(val) { this.slr = val & MMUVAX.LR_MASK; this.zapTB(0); this.setMapReg(); };
        mmu.setMAPEN = function(val) { this.mapen = val & 1; this.zapTB(0); };   /* CHANGED: 0, not 1 */
        return;
    }

    if (name == "signedPtidx" || name == "accessRead" || name == "noModifyBit") {
        mmu.fill = function(va, lnt, acc, stat) {
            /* CHANGED (signedPtidx): `va >> 7` -- a signed shift, which silently selects the
             * wrong page table entry for every address with bit 31 or bit 30 set. */
            let ptidx = (name == "signedPtidx")? ((va >> 7) & ~0x03) : ((va >>> 7) & ~0x03);
            let ptead, pte, tbi, vpn, tlbpte;

            if (va & VA_S0) {
                if (ptidx >= this.d_slr) return this.mmErr(1, va, acc, stat);
                ptead = (this.d_sbr + ptidx) & VAX.PAMASK;
            } else {
                if (va & VA_P1) {
                    if (ptidx < this.d_p1lr) return this.mmErr(1, va, acc, stat);
                    ptead = (this.d_p1br + ptidx) | 0;
                } else {
                    if (ptidx >= this.d_p0lr) return this.mmErr(1, va, acc, stat);
                    ptead = (this.d_p0br + ptidx) | 0;
                }
                if ((ptead & VA_S0) == 0) throw new VAXFault(VAXFAULT.PPTE, ptead, va);
                vpn = (ptead >>> VA_V_VPN) & VA_M_VPN;
                tbi = vpn & VA_M_TBI;
                if (this.stlbTag[tbi] != vpn) {
                    ptidx = ptead >>> 7;
                    if (ptidx >= this.d_slr) return this.mmErr(3, va, acc, stat);
                    pte = this.readLP((this.d_sbr + ptidx) & VAX.PAMASK);
                    if ((pte & PTE_V) == 0) return this.mmErr(6, va, acc, stat);
                    this.stlbTag[tbi] = vpn;
                    this.stlbPte[tbi] = (CVTACC[(pte >>> PTE_V_ACC) & 0xF] | ((pte << 9) & TLB_PFN)) | 0;
                }
                ptead = ((this.stlbPte[tbi] & TLB_PFN) | (ptead & VA_M_OFF)) | 0;
            }
            pte = this.readL(ptead);
            tlbpte = (CVTACC[(pte >>> PTE_V_ACC) & 0xF] | ((pte << 9) & TLB_PFN)) | 0;
            /* CHANGED (accessRead): the write half of the permission mask is ignored, so a
             * write to a read-only page is permitted. */
            let accCheck = (name == "accessRead")? (acc & 0xF) | ((acc >>> 4) & 0xF) : acc;
            if ((tlbpte & accCheck) == 0) return this.mmErr(0, va, acc, stat);
            if ((pte & PTE_V) == 0) return this.mmErr(4, va, acc, stat);
            if (acc & TLB_WACC) {
                /* CHANGED (noModifyBit): the write-back is skipped. */
                if ((pte & PTE_M) == 0 && name != "noModifyBit") this.writeL(ptead, pte | PTE_M);
                tlbpte = tlbpte | TLB_M;
            }
            vpn = (va >>> VA_V_VPN) & VA_M_VPN;
            tbi = vpn & VA_M_TBI;
            if ((va & VA_S0) == 0) { this.ptlbTag[tbi] = vpn; this.ptlbPte[tbi] = tlbpte; return tlbpte; }
            this.stlbTag[tbi] = vpn; this.stlbPte[tbi] = tlbpte; return tlbpte;
        };
        return;
    }

    if (name == "sharedTB" || name == "crossPageNever" || name == "pa1Unaligned") {
        /* CHANGED (sharedTB): the TB is selected without the region bit, so a system and a
         * process page with the same VPN<11:0> evict each other. */
        let sysSel = (va) => (name == "sharedTB")? false : ((va & VA_S0) != 0);
        /* CHANGED (crossPageNever): an unaligned access that spans two pages never translates
         * the second one, so its far fragment lands in whatever physical page happens to follow
         * the first -- the classic VAX paging bug, and the reason pa1 exists at all. */
        let crosses = (off, lnt) => (name == "crossPageNever")? false : ((off + lnt) > VA_PAGSIZE);
        /* CHANGED (pa1Unaligned): the second fragment's physical address keeps its low two bits,
         * so readU()/writeU() shift it by the wrong amount. */
        let trunc = (a) => (name == "pa1Unaligned")? a : (a & ~0x03);

        mmu.readData = function(va, lnt, acc) {
            let vpn, off, tbi, pa, pa1, bo, sc, wl, wh, xpte;
            this.mchkVA = va;
            if (this.mapen) {
                vpn = (va >>> VA_V_VPN) & VA_M_VPN;
                off = va & VA_M_OFF;
                tbi = vpn & VA_M_TBI;
                xpte = sysSel(va)? this.stlbPte[tbi] : this.ptlbPte[tbi];
                if (((xpte & acc) == 0) || ((sysSel(va)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                    ((acc & TLB_WACC) && ((xpte & TLB_M) == 0))) {
                    xpte = this.fill(va, lnt, acc, null);
                    if (name == "sharedTB") { this.ptlbTag[tbi] = vpn; this.ptlbPte[tbi] = xpte; }
                }
                pa = (xpte & TLB_PFN) | off;
            } else { pa = va & VAX.PAMASK; off = 0; }
            this.pa = pa; this.pa1 = pa;
            if ((pa & (lnt - 1)) == 0) {
                if (lnt >= L_LONG) return this.readL(pa);
                if (lnt == L_WORD) return this.readW(pa);
                return this.readB(pa);
            }
            if (this.mapen && crosses(off, lnt)) {
                vpn = ((va + lnt) >>> VA_V_VPN) & VA_M_VPN;
                tbi = vpn & VA_M_TBI;
                xpte = sysSel(va)? this.stlbPte[tbi] : this.ptlbPte[tbi];
                if (((xpte & acc) == 0) || ((sysSel(va)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                    ((acc & TLB_WACC) && ((xpte & TLB_M) == 0))) {
                    xpte = this.fill((va + lnt) | 0, lnt, acc, null);
                    if (name == "sharedTB") { this.ptlbTag[tbi] = vpn; this.ptlbPte[tbi] = xpte; }
                }
                pa1 = trunc((xpte & TLB_PFN) | ((va + 4) & VA_M_OFF));
            } else {
                pa1 = trunc((pa + 4) & VAX.PAMASK);
            }
            this.pa1 = pa1;
            bo = pa & 3;
            if (lnt >= L_LONG) {
                sc = bo << 3;
                wl = this.readU(pa, L_LONG - bo);
                wh = this.readU(pa1, bo);
                return (wl | (wh << (32 - sc))) | 0;
            } else if (bo == 1) return this.readU(pa, L_WORD);
            wl = this.readU(pa, L_BYTE);
            wh = this.readU(pa1, L_BYTE);
            return wl | (wh << 8);
        };
        mmu.writeData = function(va, val, lnt, acc) {
            let vpn, off, tbi, pa, pa1, bo, sc, xpte;
            this.mchkVA = va;
            if (this.mapen) {
                vpn = (va >>> VA_V_VPN) & VA_M_VPN;
                off = va & VA_M_OFF;
                tbi = vpn & VA_M_TBI;
                xpte = sysSel(va)? this.stlbPte[tbi] : this.ptlbPte[tbi];
                if (((xpte & acc) == 0) || ((sysSel(va)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                    ((xpte & TLB_M) == 0)) {
                    xpte = this.fill(va, lnt, acc, null);
                    if (name == "sharedTB") { this.ptlbTag[tbi] = vpn; this.ptlbPte[tbi] = xpte; }
                }
                pa = (xpte & TLB_PFN) | off;
            } else { pa = va & VAX.PAMASK; off = 0; }
            this.pa = pa; this.pa1 = pa;
            if ((pa & (lnt - 1)) == 0) {
                if (lnt >= L_LONG) this.writeL(pa, val);
                else if (lnt == L_WORD) this.writeW(pa, val);
                else this.writeB(pa, val);
                return;
            }
            if (this.mapen && crosses(off, lnt)) {
                vpn = ((va + 4) >>> VA_V_VPN) & VA_M_VPN;
                tbi = vpn & VA_M_TBI;
                xpte = sysSel(va)? this.stlbPte[tbi] : this.ptlbPte[tbi];
                if (((xpte & acc) == 0) || ((sysSel(va)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                    ((xpte & TLB_M) == 0)) {
                    xpte = this.fill((va + lnt) | 0, lnt, acc, null);
                    if (name == "sharedTB") { this.ptlbTag[tbi] = vpn; this.ptlbPte[tbi] = xpte; }
                }
                pa1 = trunc((xpte & TLB_PFN) | ((va + 4) & VA_M_OFF));
            } else {
                pa1 = trunc((pa + 4) & VAX.PAMASK);
            }
            this.pa1 = pa1;
            bo = pa & 3;
            if (lnt >= L_LONG) {
                sc = bo << 3;
                this.writeU(pa, val & INSERT[L_LONG - bo], L_LONG - bo);
                this.writeU(pa1, (val >>> (32 - sc)) & INSERT[bo], bo);
            } else if (bo == 1) this.writeU(pa, val & 0xFFFF, L_WORD);
            else {
                this.writeU(pa, val & 0xFF, L_BYTE);
                this.writeU(pa1, (val >>> 8) & 0xFF, L_BYTE);
            }
        };
        if (name == "sharedTB") {
            mmu.test = function(va, acc, stat) {
                let vpn, off, tbi, xpte;
                if (stat) stat.code = MMUVAX.PR.OK;
                if (this.mapen) {
                    vpn = (va >>> VA_V_VPN) & VA_M_VPN;
                    off = va & VA_M_OFF;
                    tbi = vpn & VA_M_TBI;
                    xpte = this.ptlbPte[tbi];                   /* CHANGED: never the system TB */
                    if ((xpte & acc) && (this.ptlbTag[tbi] == vpn)) { this.pa = (xpte & TLB_PFN) | off; return this.pa; }
                    xpte = this.fill(va, L_BYTE, acc, stat);
                    if (!stat || stat.code == MMUVAX.PR.OK) {
                        this.ptlbTag[tbi] = vpn; this.ptlbPte[tbi] = xpte;
                        this.pa = (xpte & TLB_PFN) | off;
                        return this.pa;
                    }
                    return -1;
                }
                this.pa = va & VAX.PAMASK;
                return this.pa;
            };
        }
        return;
    }
    throw new Error("unknown mutation '" + name + "'");
}

/* ------------------------------------------------------------------------------------------- *
 * main
 * ------------------------------------------------------------------------------------------- */

async function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, dflt) => {
        let i = argv.indexOf(name);
        return i >= 0? argv[i + 1] : dflt;
    };
    let nOps = +getArg("--ops", DEFAULT_OPS);
    let seed = +getArg("--seed", 0x11FACE);
    let simhBin = findSimh(getArg("--simh", null));
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;
    let fSkipEhkaa = argv.indexOf("--skip-ehkaa") >= 0;
    let scratchArg = getArg("--scratch", null);
    /* Only a directory THIS run created is ours to remove -- a caller-supplied --scratch is the
       caller's, never auto-deleted here.  `||`, not a default *argument*, so mkdtempSync() runs
       only when --scratch was NOT given -- the previous `getArg("--scratch", fs.mkdtempSync(...))`
       evaluated its default eagerly on every call, so passing --scratch explicitly still created
       and then orphaned one throwaway vaxmmu-* directory per invocation (HANDOFF.md pcjsvax-bd1). */
    let autoScratch = scratchArg === null;
    let scratch = scratchArg || fs.mkdtempSync(path.join(os.tmpdir(), "vaxmmu-"));
    fs.mkdirSync(scratch, {recursive: true});

    console.log("VAX MMU differential test");
    console.log("  SIMH binary: %s", simhBin);
    console.log("  ops=%d seed=0x%s scratch=%s", nOps, hex(seed), scratch);

    /* Every exit path -- requireMmuSupport() throwing, a FAIL, a PASS, or any other thrown
       exception -- runs through this try/finally, so scratch is always removed.  HANDOFF.md
       pcjsvax-bd1: this file had NO rmSync of scratch anywhere, on any path -- 37 abandoned
       /tmp/vaxmmu-* directories were found on this disk. */
    try {
    requireMmuSupport(simhBin, scratch);

    let errors = [];
    let require_ = (cond, msg) => { if (!cond) errors.push("COVERAGE: " + msg); };

    /* ------------------------------------------------------------------ exerciser */
    console.log("\nPhase 1: randomized exerciser (P0/P1/S0/S1, all four modes, MMUOP-driven)");
    let t0 = Date.now();
    let ex = runExerciser(simhBin, nOps, seed, "", scratch, false);
    console.log("  elapsed: %ss", ((Date.now() - t0) / 1000).toFixed(1));
    errors.push(...ex.failures);

    let s = ex.stats;
    let scale = nOps / DEFAULT_OPS;
    require_(s.nOps >= MIN_OPS, "fewer than " + MIN_OPS + " operations (" + s.nOps +
        "); an undersized run is not a run -- raise --ops");
    require_(s.nOps >= 0.98 * nOps, "fewer operations compared (" + s.nOps + ") than generated (" + nOps + ")");
    require_(s.nCompared == s.nOps, (s.nOps - s.nCompared) + " operations never reached comparison");
    for (let r of ["P0", "P1", "S0"]) {
        require_(s.byRegion[r] > 2000 * scale, "too few " + r + " operations (" + s.byRegion[r] + ")");
    }
    require_(s.byRegion.S1 > 500 * scale, "too few S1 (region 3) operations (" + s.byRegion.S1 + ")");
    for (let m = 0; m < 4; m++) {
        require_(s.byMode[m] > 6000 * scale, "too few operations in " + MODE_NAMES[m] +
            " mode (" + s.byMode[m] + ")");
    }
    require_(s.byOp[OPS.TESTR] > 3000 * scale && s.byOp[OPS.TESTW] > 3000 * scale,
        "too few probe (Test) operations (" + s.byOp[OPS.TESTR] + "/" + s.byOp[OPS.TESTW] + ")");
    require_(s.byOp[OPS.READ] > 8000 * scale && s.byOp[OPS.WRITE] > 8000 * scale,
        "too few read/write operations");
    require_(s.nCrossPage > 2000 * scale, "too few page-boundary crossings (" + s.nCrossPage + ")");
    require_(s.nCrossLong > 2000 * scale, "too few longword-boundary crossings (" + s.nCrossLong + ")");
    require_(s.nUnaligned > 8000 * scale, "too few unaligned accesses (" + s.nUnaligned + ")");
    require_(s.nTwoLevel > 1000 * scale, "too few two-level (process) page table walks (" + s.nTwoLevel + ")");
    require_(s.nTbHits > 5000 * scale, "too few TB hits (" + s.nTbHits + "); the TB is not being exercised");
    require_(s.nMBit > 20, "too few M-bit write-backs (" + s.nMBit + ")");
    require_(s.nUnmapped > 200 * scale, "too few operations with MAPEN off (" + s.nUnmapped + ")");
    for (let code of [MMUVAX.PR.ACV, MMUVAX.PR.LNV, MMUVAX.PR.TNV, MMUVAX.PR.PLNV, MMUVAX.PR.PTNV]) {
        require_(s.byFault[code] > 50 * scale, "too few " + PR_NAMES[code] + " faults (" + s.byFault[code] + ")");
    }
    require_(s.nOK > 8000 * scale, "too few operations SUCCEEDED (" + s.nOK +
        "); a layout in which everything faults proves nothing");
    require_(s.nValueChecks > 8000 * scale, "too few value comparisons (" + s.nValueChecks + ")");
    require_(s.nValueNonZero > 0.80 * s.nValueChecks, "only " + s.nValueNonZero + " of " +
        s.nValueChecks + " value comparisons were against NON-ZERO data; a pool of untouched" +
        " memory makes the run look bigger than it is");
    require_(s.nSweepNonZero > 0.80 * s.nSweep, "only " + s.nSweepNonZero + " of " + s.nSweep +
        " read-back comparisons were against non-zero data");

    /* ------------------------------------------------------------------ EHKAA */
    let eh = null;
    if (!fSkipEhkaa) {
        console.log("\nPhase 2: EHKAA replay (every MMU operation the diagnostic issues)");
        t0 = Date.now();
        eh = await runEhkaa(simhBin, {
            scratch,
            trace: getArg("--trace", null),
            ehkaaExe: getArg("--ehkaa", path.join(vaxRepo(), "open-simh/VAX/tests/ehkaa.exe"))
        }, "");
        console.log("  elapsed: %ss", ((Date.now() - t0) / 1000).toFixed(1));
        errors.push(...eh.failures);
        let e = eh.stats;
        console.log("  ops=%d compared=%d  (read %d, write %d, probe %d)  mapped=%d unmapped=%d",
            e.nOps, e.nCompared, e.byRW.R, e.byRW.W, e.byRW.T, e.nMapped, e.nUnmapped);
        console.log("  regions=%s  read-acc by mode=[K %d, E %d, S %d, U %d]  write-acc by mode=[K %d, E %d, S %d, U %d]",
            JSON.stringify(e.byRegion), e.readAcc[0], e.readAcc[1], e.readAcc[2], e.readAcc[3],
            e.writeAcc[0], e.writeAcc[1], e.writeAcc[2], e.writeAcc[3]);
        console.log("  fills=%d (two-level %d) M-bit=%d  faults=%d probe-refusals=%d  TBIA/zap=%d TBIS=%d  MAPEN on=%d off=%d",
            e.nFills, e.nTwoLevel, e.nMBit, e.nFaults, e.nProbeFail, e.nZap, e.nZapEnt, e.nMapenOn, e.nMapenOff);
        console.log("  cross-page=%d unaligned=%d far-page faults=%d   comparisons: pa=%d pa1=%d page-table-address=%d",
            e.nCrossPage, e.nUnaligned, e.nFar, e.nPaChecks, e.nPa1Checks, e.nPteAddrChecks);

        require_(e.nOps > 300000, "fewer than 300,000 EHKAA MMU operations (" + e.nOps + ")");
        require_(e.nCompared == e.nOps, (e.nOps - e.nCompared) + " EHKAA operations never reached comparison");
        require_(e.nSkipped == 0, e.nSkipped + " EHKAA operations were skipped for want of a trace record");
        require_(e.nMapped > 50000, "too few EHKAA operations with MAPEN on (" + e.nMapped + ")");
        require_(e.nMapenOn >= 2 && e.nMapenOff >= 1,
            "EHKAA's MAPEN transitions were not observed (on=" + e.nMapenOn + " off=" + e.nMapenOff + ")");
        require_(e.nFills > 3000, "too few EHKAA TB fills (" + e.nFills + ")");
        require_(e.nTwoLevel > 10, "too few EHKAA two-level walks (" + e.nTwoLevel + ")");
        require_(e.nFaults > 300, "too few EHKAA translation faults (" + e.nFaults + ")");
        require_(e.nZapEnt > 200, "too few EHKAA TBIS (single-entry) invalidations (" + e.nZapEnt + ")");
        require_(e.nMBit > 50, "too few EHKAA M-bit write-backs (" + e.nMBit + ")");
        require_(e.nPa1Checks > 20000, "too few EHKAA second-address comparisons (" + e.nPa1Checks + ")");
        require_(e.nPteAddrChecks > 4000, "too few EHKAA page-table address comparisons (" + e.nPteAddrChecks + ")");
        require_(e.nFar > 0, "no fault was raised by the SECOND page of a cross-page access (" +
            e.nFar + "); that case grades the fault parameters of the far-page fill");
    } else {
        console.log("\nPhase 2: SKIPPED (--skip-ehkaa)");
        errors.push("COVERAGE: the EHKAA phase was skipped; this is not a complete run");
    }

    /* ------------------------------------------------------------------ self-check */
    if (fSelfCheck) {
        console.log("\nSelf-check: the differential must FAIL for each deliberate defect.");
        for (let mut of MUTATIONS) {
            let caught = false, why = "";
            try {
                let r = runExerciser(simhBin, 4000, seed ^ 0x5A5A, mut, scratch, true);
                caught = r.failures.length > 0;
                why = caught? r.failures[0] : "";
                if (!caught && eh) {
                    let r2 = await runEhkaa(simhBin, {scratch, trace: eh.tracePath}, mut);
                    caught = r2.failures.length > 0;
                    why = caught? "(EHKAA) " + r2.failures[0] : "NO FAILURES REPORTED";
                }
            } catch (e) {
                caught = true;                          /* a thrown TypeError is also a detection */
                why = "threw: " + e.message.split("\n")[0];
            }
            console.log("  %s: %s", mut.padEnd(18), caught? "detected -- " + why.slice(0, 130) : "MISSED");
            if (!caught) errors.push("SELFCHECK: mutation '" + mut + "' was not detected");
        }
    }

    if (errors.length) {
        console.log("\nFAILED (%d):", errors.length);
        for (let f of errors) console.log("  " + f);
        process.exitCode = 1;
        return;
    }
    console.log("\nPASS: MMUVAX is indistinguishable from SIMH over %d randomized operations%s.",
        ex.stats.nOps, eh? " and " + eh.stats.nOps + " EHKAA operations" : "");
    } finally {
        if (autoScratch) fs.rmSync(scratch, {recursive: true, force: true});
    }
}

if (process.argv[1] && path.resolve(process.argv[1]) == path.resolve(fileURLToPath(import.meta.url))) {
    main().catch((e) => { console.error(e); process.exitCode = 1; });
}

export {
    buildLayout, genOps, runExerciser, runEhkaa, parseMmuTrace, applyMutation, makeMachine,
    findSimh, hex, MUTATIONS, MEMSIZE, SBR, SLR, P0BR, P0LR, P1BR, P1LR
};
