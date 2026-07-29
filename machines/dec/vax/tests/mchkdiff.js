/**
 * @fileoverview Differential test: an absent physical register raises a VAX machine check,
 *               instead of stopping the simulator, vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-446: a physical reference to an address BusVAX.RESERVED reserves but does not decode
 * (the KA655 I/O, register, SSC, NVR and Qbus-memory ranges) must raise a VAX machine check
 * through the SCB -- exactly as a real KA655 does when the console ROM probes for hardware --
 * instead of stopping the simulator (cpustate.js's old onBusFault()).  The observable is TWO
 * things, not one: the machine check is delivered (new PC, new PSL, the exact 7-longword
 * exception+mcheck stack frame) AND the SSC bus-timeout register reads back with its timeout bit
 * set afterward -- EXCEPT for the one measured mechanism (below) where real SIMH itself never
 * sets that bit, which this file grades as precisely as it grades the bit being set. This item
 * does NOT decode any device register -- it is entirely about what happens when one is ABSENT.
 *
 * NO SIMH PATCH IS NEEDED.  Unlike busdiff.js's/excdiff.js's siblings, this differential does not
 * need instrumented state: `deposit`/`step 1`/`examine` (stock SCP) is enough, because everything
 * graded here -- the new PC, the new PSL, the pushed stack frame, and the SSC bus-timeout
 * register (SIMH: `examine sysd bto`) -- is architecturally visible after one step.
 *
 * VERACITY RE-DISPATCH (this file's second revision) -- READ BEFORE CHANGING THE POOL LOGIC
 * -------------------------------------------------------------------------------------------
 * The first revision calibrated an address into the graded pool only when SIMH BOTH dispatched a
 * machine check AND set ssc_bto.  That collapsed two independent questions -- "does SIMH dispatch
 * here" and "does my CURRENT implementation happen to agree" -- into one, and the excluded set
 * became an unexamined proxy for wherever the implementation was wrong: IOPAGE/CQM READS
 * genuinely DO dispatch a machine check on real SIMH (vax_io.c ReadQb() -> cq_merr() ->
 * MACH_CHECK()), just without ever touching ssc_bto -- and cpustate.js's onBusFault() had a real
 * bug that set ssc_bto there anyway.  Because those addresses were excluded rather than graded,
 * the bug was invisible to this file's own coverage floors.
 *
 * The fix: calibrate() now decides GRADED vs EXCLUDED on ONE fact only -- does SIMH dispatch a
 * machine check at all (PC reaches the handler)?  Whether ssc_bto ends up set is tracked
 * separately (`btoSeen`) and is graded like every other observable, by ordinary equality in
 * compareCase() -- it no longer gates pool membership.  See onBusFault()'s doc comment in
 * cpustate.js for the resulting read/write split on IOPAGE/CQM.
 *
 * WHY EVERY "RESERVED" ADDRESS IS NOT A "MUST MACHINE-CHECK" ADDRESS ON REAL SIMH
 * ---------------------------------------------------------------------------------
 * BusVAX.RESERVED marks FIVE ranges this bus reserves but does not decode (SIX until pcjsvax-320
 * removed SSC_BASE -- see below).  Real SIMH, however, DOES decode most of them (vax_sysdev.c's
 * `regtable[]`: CQBIC, CMCTL, KA655 regs, CQMAP, SSC, NVR) -- because SIMH implements the
 * peripherals this project has not built yet, or (ROM, and now the SSC base register) a sibling
 * item already has.  Measured directly (see calibrate() below, and its printed report, and
 * EXPECTED_CALIBRATION's committed counts):
 *
 *   - CDG_BASE was backed END TO END (`cdg_rd`/`cdg_wr` span exactly VAX.PHYSMEM.CDG_LENGTH) and
 *     was therefore a 100% expected divergence: reported, never graded.  pcjsvax-0b7 DECODED it
 *     (bus.js's addCdg(), cdg.js), so it left BusVAX.RESERVED and left this file's RANGES with it,
 *     exactly the SSC_BASE precedent below.  The divergence did not become invisible -- it became
 *     a MATCH, graded at register level (aliasing, the CACR diagnostic-parity side effect on read,
 *     the byte/word write merge, end-to-end backing) by tests/cdgdiff.js.
 *   - REG_BASE is a MIX: several real sub-windows (CQBIC at +0, CMCTL at +0x100, KA655 regs at
 *     +0x4000, CQIPC at +0x1F40, CQMAP at +0x8000) are backed; the rest of its 512KB genuinely
 *     machine-checks on both sides THROUGH ReadReg()/WriteReg(), the mechanism this item models.
 *   - NVR_BASE (added to BusVAX.RESERVED by the pcjsvax-446 re-dispatch -- see bus.js's
 *     isReserved(), standing rule 7) is ALSO backed end to end (`nvr_rd`/`nvr_wr`): NVR's OWN
 *     storage is a real device in SIMH, addressed through the same ReadReg/WriteReg table as
 *     everything else here.  100% expected divergence, as CDG's was -- but it must be IN the pool's
 *     candidate space (not a silent gap) for that fact to be checked and reported rather than
 *     assumed.
 *   - SSC_BASE is NO LONGER in BusVAX.RESERVED, and therefore no longer one of this file's RANGES
 *     at all, as of pcjsvax-320: it decodes the SSC base register (see bus.js's addSsc(), ssc.js)
 *     and installs a real controller over the whole SSC span, the same ROM_BASE precedent that
 *     already excluded ROM from this list.  This is exactly the address pcjsvax-223 measured as
 *     the ROM's FIRST hardware probe (SSC+0x0) and pcjsvax-320 measured as its SECOND (SSC+0x30,
 *     OTP) -- the sub-registers pcjsvax-320 does NOT decode still fault exactly as before (see
 *     ssc.js's file header), they are simply no longer reachable through THIS file's
 *     RESERVED-range-derived pool, since they are not a whole reserved RANGE anymore.
 *   - IOPAGE_BASE and CQM_BASE are ADDR_IS_IO()/ADDR_IS_CQM() territory: a KA655 routes Qbus
 *     I/O-page and Qbus-memory references through vax_io.c's `ReadQb()`/`WriteQb()`, not
 *     vax_sysdev.c's ReadReg()/WriteReg().  An unbacked reference there calls `cq_merr()` (sets
 *     the CQBIC's DSER/MEAR error registers, which this item does not model -- filed as
 *     pcjsvax-d22) and, for READS ONLY, the SAME `MACH_CHECK()` -- but NEVER touches ssc_bto.
 *     WRITES there don't even raise the exception synchronously: `WriteQb()`'s unbacked case sets
 *     `mem_err = 1` (a DEFERRED MEMERR interrupt) and returns normally.  So: IOPAGE/CQM READS are
 *     GRADED (btoSeen is expected false and IS graded, per the fix above); IOPAGE/CQM WRITES are
 *     EXCLUDED (SIMH never dispatches at all -- there is nothing to compare).  This matches the
 *     rd item's own measured-facts section, which cites ONLY ReadReg/WriteReg and ReadIPR/
 *     WriteIPR -- never ReadQb/WriteQb/cq_merr -- so modelling the Qbus mem_err/DSER path is
 *     pcjsvax-d22's work, not a gap in this one.
 *
 * So the address pool this differential grades against is not "every address in
 * BusVAX.RESERVED" -- it is CALIBRATED against the real oracle first: candidate addresses (walked
 * OFF BusVAX.RESERVED programmatically, never hand-enumerated) are probed once, and every one
 * where the machine check DISPATCHES AT ALL goes into the comparison pool, whatever ssc_bto ends
 * up being.  Only addresses where SIMH itself never dispatches are excluded.  Every exclusion is
 * reported by name -- the same convention busdiff.js's reportScopeGaps() established for the
 * console-EXAMINE path -- and EXPECTED_CALIBRATION asserts the excluded (and graded) counts
 * against committed numbers, so a rebase or an implementation change that silently widens what
 * gets excluded FAILS the run instead of quietly shrinking coverage.
 *
 * TWO PHASES, PER THE PROJECT'S STANDING RULE
 * --------------------------------------------
 *   ENUMERATED   Deterministic: every CONFIRMED address (boundary points of every reserved range,
 *                from calibrate()), both directions (read/write, MOVx vs TSTx), all three ALIGNED
 *                sizes (byte/word/long), PLUS every UNALIGNED word/long offset of every confirmed
 *                address (+1/+2/+3 for long, +1/+3 for word -- +2 is itself word-aligned).
 *                Guarantees full-range, full-size, full-alignment coverage that a random draw
 *                cannot promise -- this project has no real workload that probes devices yet (the
 *                console ROM is a LATER milestone), so this phase stands in for it.
 *   RANDOMIZED   Random draws from the same confirmed pool (aligned addresses only -- see
 *                UNALIGNED_OFFSETS' doc comment for why), with random surrounding machine state
 *                (PSL mode/IPL/condition codes, SISR, general registers) that the deterministic
 *                phase does not vary -- old PSL/CC values ride into the pushed exception frame
 *                unchanged, so this is the phase that catches a wrong pass-through of that state.
 *
 * BOTH PHASES DRAW FROM THE SAME CALIBRATED POOL, AND THAT IS A REAL LIMITATION, NOT AN OVERSIGHT:
 * they are one view at two granularities (exhaustive vs. varied-state), not two structurally
 * independent oracles.  Both were blind to the ssc_bto-on-IOPAGE/CQM-reads bug identically, for
 * the same reason: NEITHER phase can see a bug in the CRITERION that built the pool they both draw
 * from.  The genuine independent check on the criterion itself is EXPECTED_CALIBRATION's asserted
 * counts (committed once, against the real oracle, and re-verified every run) plus this file's own
 * --selfcheck, which mutates the SHIPPED code the pool-independent way: by breaking behavior, not
 * by narrowing what gets compared.
 *
 * WHAT IS DELIBERATELY NOT MODELLED (see exc.js's busTimeout()/takeFault() doc comments)
 * -----------------------------------------------------------------------------------------
 *   - SIMH's REF_P (mchk_ref=1) half of the machine-check "address" parameter.  Every case here
 *     reaches the fault through mmu.readData()/writeData() (an ordinary instruction's data
 *     reference), which is SIMH's REF_V (0) path -- the one a console-ROM probe actually uses.
 *   - CADR/MSER (state1's low 16 bits) are NO LONGER unmodelled -- pcjsvax-877 models them in
 *     exc.js (they are plain globals beside SIMH's own IPR switch, vax_sysdev.c:235-236, not
 *     SSC/CMCTL device state), and exc.js's SCB.MCHK case now builds the low half as
 *     `((cadr & 0xFF) << 8) | (mser & 0xFF)`, term for term with vax_sysdev.c:1654-1657.
 *     WHAT IS STILL NOT EXERCISED HERE: no case in this file writes either register, so both
 *     engines read 0 there and the low half is matched AT ZERO rather than driven.  That is
 *     agreement, not coverage, and saying so is the point of this list.  The behaviour itself is
 *     graded by tests/excdiff.js, whose IPR_POOL picked both registers up automatically when they
 *     left IPR_DEVICE (the pool is derived as "every prn not in IPR_DEVICE"), plus deterministic
 *     MTPR/MFPR edge cases for the three values the ROM's self-test 46 actually uses.
 *   - Any machine-check trigger other than a bus fault (parity/ECC, etc.) -- none exist yet.
 *   - The Qbus/CQBIC `cq_merr`/DSER/MEAR/deferred-`mem_err` mechanism itself (pcjsvax-d22) --
 *     IOPAGE/CQM writes are excluded from grading for exactly this reason (see above).
 *   - A real PROGRAM write to ssc_bto (the W1C -- write-one-to-clear -- semantics), and
 *     accumulation across MORE than two faults.  verifySecondFault() (below) DOES grade the
 *     ordinary case this item's own scope requires: ssc_bto stays set and the second fault's own
 *     frame is computed fresh across two INDEPENDENT faults back to back (the second reached via
 *     intexc()'s "already on the interrupt stack" branch, a different path than the first).
 *
 *      node machines/dec/vax/tests/mchkdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_BIN, else the scratch build)
 *        --cases N         randomized cases (default 300; below MIN_CASES_FLOOR the run FAILS --
 *                           it no longer silently clamps up)
 *        --seed S          PRNG seed, printed on failure so a run is reproducible
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
import CPUStateVAX, { VAXStop } from "../modules/v2/cpustate.js";
import { VAXExc, SCB } from "../modules/v2/exc.js";
import { VAXFault } from "../modules/v2/decode.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

/* ------------------------------------------------------------------------------------------- *
 * Small utilities (PRNG/hex match the other VAX differentials, so a failing seed reproduces)      *
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

function findSimh(pathArg)
{
    let candidates = [pathArg, process.env["SIMH_BIN"],
        path.resolve(REPO_ROOT, "../pcjs-vax/open-simh/BIN/microvax3900")].filter((p) => !!p);
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error("microvax3900 binary not found (tried: " + candidates.join(", ") + ").\n" +
        "This test grades against REAL SIMH; it has no fixture fallback.  Pass --simh PATH or set SIMH_BIN.");
}

function runSimh(bin, script, iniPath)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 10 * 60 * 1000});
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
    return {bus, cpu};
}

/* Fixed memory layout, deposited fresh (after `reset all`) by every case -- same convention
   excdiff.js/cpudiff.js use, so no case can leak into another even in a shared batch. */
const R_SCBB     = 0x00100000;
const R_HANDLER  = 0x00102000;          // SCBB+SCB.MCHK's target: a case reaching here MCHK'd
const R_HANDLER2 = 0x00102100;          // a DIFFERENT vector's target, for the "wrong vector" mutation
const R_CODE     = 0x00104000;
const R_KSP      = 0x00110000;
const R_IS       = 0x00118000;
const FRAME_LEN  = 28;                  // 8 (old PC/PSL) + 20 (mcheck's 5 longwords)

/** Opcode numbers, resolved by mnemonic -- never hand-transcribed. */
function opcodeOf(name)
{
    let opc = OPCODES.indexOf(name);
    if (opc < 0) throw new Error(`mchkdiff.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
    return opc;
}
const OPC = {
    MOVB: opcodeOf("MOVB"), MOVW: opcodeOf("MOVW"), MOVL: opcodeOf("MOVL"),
    TSTB: opcodeOf("TSTB"), TSTW: opcodeOf("TSTW"), TSTL: opcodeOf("TSTL")
};
const MOV_BY_SIZE = {1: "MOVB", 2: "MOVW", 4: "MOVL"};
const TST_BY_SIZE = {1: "TSTB", 2: "TSTW", 4: "TSTL"};

/**
 * buildInstr(fWrite, size, addr)
 *
 * A write probe is `MOVx #0, @#addr` (short-literal 0 -> absolute); a read probe is
 * `TSTx @#addr` (absolute).  Both are ordinary data references through mmu.readData()/
 * writeData() -- SIMH's REF_V path -- which is exactly how a console-ROM probe touches hardware.
 *
 * @param {boolean} fWrite
 * @param {number} size 1, 2 or 4
 * @param {number} addr
 * @returns {Array.<number>} instruction bytes
 */
function buildInstr(fWrite, size, addr)
{
    let b = [];
    if (fWrite) {
        b.push(OPC[MOV_BY_SIZE[size]] & 0xFF, 0x00);           // opcode, short-literal #0
    } else {
        b.push(OPC[TST_BY_SIZE[size]] & 0xFF);                 // opcode
    }
    b.push(0x9F);                                              // absolute mode, @#
    let a = addr >>> 0;
    b.push(a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF);
    return b;
}

/* ------------------------------------------------------------------------------------------- *
 * BusVAX.RESERVED -- programmatically derived candidate addresses, never hand-enumerated          *
 * ------------------------------------------------------------------------------------------- */

/**
 * candidatesFor(base, len)
 *
 * Boundary points plus a handful of interior strides, all longword-aligned so byte/word/long
 * probes at the same address are all in-range for the smallest ranges (IOPAGE is 8KB).
 *
 * @param {number} base
 * @param {number} len
 * @returns {Array.<number>}
 */
function candidatesFor(base, len)
{
    let a = new Set();
    let add = (off) => a.add((base + (off & ~3)) >>> 0);
    add(0);
    add(4);
    add(len >> 1);
    add(len - 8);
    add(len - 4);
    for (let k = 1; k <= 6; k++) add(Math.floor(len * k / 8));
    return [...a].filter((x) => x >= base && x < base + len).sort((x, y) => x - y);
}

/** {name, base, len, addrs} for every BusVAX.RESERVED entry -- names are diagnostic labels only.
    Order MUST track bus.js's BusVAX.RESERVED array (asserted at load time below).  "SSC" was
    removed by pcjsvax-320 and "CDG" by pcjsvax-0b7 (see the file header): neither is a whole
    reserved-but-undecoded range any more. */
const RANGE_NAMES = ["IOPAGE", "REG", "CQM", "NVR"];
if (RANGE_NAMES.length !== BusVAX.RESERVED.length) {
    throw new Error(`mchkdiff.js: RANGE_NAMES has ${RANGE_NAMES.length} entries, ` +
        `BusVAX.RESERVED has ${BusVAX.RESERVED.length} -- bus.js's reserved-range list changed ` +
        `shape; update RANGE_NAMES (and re-measure EXPECTED_CALIBRATION) to match.`);
}
const RANGES = BusVAX.RESERVED.map((r, i) => ({
    name: RANGE_NAMES[i] || `RESERVED[${i}]`,
    base: r[0], len: r[1],
    addrs: candidatesFor(r[0], r[1])
}));

/* ------------------------------------------------------------------------------------------- *
 * SIMH script construction, shared by calibration and the graded phases                          *
 * ------------------------------------------------------------------------------------------- */

const CASE_MARK = "@@MCHK@@";

/**
 * @class Case
 */
class Case {
    constructor(index, fWrite, size, addr)
    {
        this.index = index;
        this.fWrite = fWrite;
        this.size = size;
        this.addr = addr >>> 0;
        this.psl = 0;
        this.regs = new Int32Array(15);         // R0..R14 (GPRs + the live stack pointer)
        this.sisr = 0;
        /*
         * Set directly from the pool entry that generated this case (range/btoSeen/unaligned),
         * rather than re-derived later by matching `addr` back against the pool: an UNALIGNED
         * case's addr is base+offset, which does not equal any pool entry's addr, so an
         * address-based lookup would silently lose exactly the cases this re-dispatch added.
         */
        this.range = null;
        this.btoSeen = null;                    // expected ssc_bto state, or null if not from the pool
        this.unaligned = false;
    }
}

/* R0..R14, then PC, PSL, SISR as their own named registers -- matching excdiff.js's/cpudiff.js's
   convention (R14 is the LIVE stack pointer; PC is register 15 but SIMH exposes it as "PC"). */
const NAMES = (function() {
    let n = [];
    for (let r = 0; r < 15; r++) n.push("R" + r);
    n.push("PC", "PSL", "SISR");
    return n;
})();

function buildScript(cases)
{
    let L = ["set cpu " + (MEMSIZE >> 20) + "m", "set cpu simhalt"];
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        L.push("reset all");
        /*
         * MEASURED: `reset all` does NOT clear ssc_bto (verified directly -- a case that sets it
         * leaves it set for every later case in the same batch/process, i.e. the SAME process-wide
         * stickiness a real bus-timeout register has on hardware).  cpu.exc.reset() DOES clear
         * cpu.exc.sscBto (see exc.js); this line is what keeps the two sides starting from the
         * same state every case, not just the first one in a batch.
         */
        L.push("deposit sysd bto 0");
        /*
         * Zero the frame area (and the code area) BEFORE every case.  `reset all` clears
         * registers and devices, but NOT ordinary RAM -- and a case whose probe does NOT
         * machine-check (a RAM control, or an excluded/different-mechanism address) writes
         * NOTHING to the frame area, so without this it reads back whatever the PREVIOUS case
         * that DID fault left there.  That is invisible cross-case contamination on ONE side only
         * at a batch boundary: `runBatch()` spawns a FRESH SIMH process per batch (virgin RAM),
         * while the JS machine (`makeMachine()`) is reused for the WHOLE run -- so the first case
         * of every batch after the first compared a stale JS frame against SIMH's genuinely-empty
         * one.  Same convention as excdiff.js's ZERO_ADDRS, for the same reason.
         */
        let spZero = (R_IS - FRAME_LEN) >>> 0;
        for (let i = 0; i < FRAME_LEN; i += 4) L.push(`deposit -l ${hex(spZero + i)} 0`);
        L.push(`deposit SCBB ${hex(R_SCBB)}`);
        L.push(`deposit -l ${hex(R_SCBB + SCB.MCHK)} ${hex(R_HANDLER)}`);
        L.push(`deposit -l ${hex(R_SCBB + 0x08)} ${hex(R_HANDLER2)}`);       // KSNV vector; "wrong vector" mutation target
        L.push(`deposit SISR ${hex(c.sisr)}`);
        /*
         * R0..R14 BEFORE KSP/IS: "deposit R14" and "deposit KSP" alias the SAME storage while the
         * (post-reset, kernel-mode) current stack pointer is live, so whichever is deposited LAST
         * wins.  KSP/IS must win -- the machine check is a SEVERE exception and unconditionally
         * reloads SP from IS (vax_cpu1.c intexc()), so IS is what the frame addresses below
         * depend on, not whatever the noise loop put in R14.  runCaseJS() mirrors this ordering.
         */
        for (let r = 0; r < 15; r++) L.push(`deposit R${r} ${hex(c.regs[r])}`);
        L.push(`deposit KSP ${hex(R_KSP)}`);
        L.push(`deposit IS ${hex(R_IS)}`);
        let instr = buildInstr(c.fWrite, c.size, c.addr);
        for (let i = 0; i < instr.length; i++) L.push(`deposit -b ${hex(R_CODE + i)} ${instr[i].toString(16)}`);
        L.push(`deposit PSL ${hex(c.psl)}`);
        L.push(`deposit PC ${hex(R_CODE)}`);
        L.push("step 1");
        L.push(`examine -h ${NAMES.join(",")}`);
        L.push(`examine -h sysd bto`);
        let sp = (R_IS - FRAME_LEN) >>> 0;
        for (let i = 0; i < FRAME_LEN; i += 4) L.push(`examine -h ${hex(sp + i)}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

/* SIMH decorates some registers with a decoded bit list after the value (PSL prints
   "041F0000\tCM0 TP0 ..."), so this must NOT anchor at end of line. */
const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/;
const WANT_PER_CASE = NAMES.length + 1 + (FRAME_LEN / 4);       // regs/PSL/SISR + BTO + frame

function runBatch(simh, cases, scratch)
{
    let script = buildScript(cases);
    let out = runSimh(simh, script, path.join(scratch, "mchkdiff-batch.ini"));
    let lines = out.split("\n");
    let results = new Map();
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\d+)"));
        if (!m) { i++; continue; }
        let idx = +m[1];
        i++;
        let vals = [];
        while (i < lines.length && vals.length < WANT_PER_CASE) {
            if (lines[i].indexOf(CASE_MARK) >= 0) break;
            let vm = lines[i].match(VALUE_RE);
            if (vm) vals.push(parseInt(vm[2], 16) | 0);
            i++;
        }
        if (vals.length < WANT_PER_CASE) { results.set(idx, {reached: false, got: vals.length, want: WANT_PER_CASE}); continue; }
        let regs = new Int32Array(15);
        for (let r = 0; r < 15; r++) regs[r] = vals[r];
        let pc = vals[15], psl = vals[16], sisr = vals[17], bto = vals[18];
        let frame = vals.slice(19);
        results.set(idx, {reached: true, regs, pc, psl, sisr, bto, frame});
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * JS side                                                                                        *
 * ------------------------------------------------------------------------------------------- */

function runCaseJS(m, c)
{
    let {bus, cpu} = m;
    cpu.reset();
    let spZero = (R_IS - FRAME_LEN) >>> 0;
    for (let i = 0; i < FRAME_LEN; i += 4) bus.setLong(spZero + i, 0);   // see buildScript's matching comment
    cpu.exc.scbb = R_SCBB;
    bus.setLong(R_SCBB + SCB.MCHK, R_HANDLER);
    bus.setLong(R_SCBB + 0x08, R_HANDLER2);
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
    cpu.exc.sisr = c.sisr;
    cpu.regs.set(c.regs);
    let instr = buildInstr(c.fWrite, c.size, c.addr);
    for (let i = 0; i < instr.length; i++) bus.setByte(R_CODE + i, instr[i]);
    cpu.psl = c.psl | 0;
    /* R14 is whatever c.regs[14] says (deposited verbatim on the SIMH side too) -- KSP/IS are
       independent storage from the LIVE R14 on real hardware (measured directly: depositing KSP
       does not change a subsequent examine of R14), so forcing R14 here would desync the two
       sides' pre-fault state for no reason; the machine check unconditionally reloads SP from IS
       regardless of what R14 held before it. */
    cpu.regs[15] = R_CODE;

    let stop = null;
    try {
        cpu.stepCPU(1);
    } catch (e) {
        if (e instanceof VAXStop) stop = e.reason;
        else throw e;
    }
    let sp = (R_IS - FRAME_LEN) >>> 0;
    let frame = [];
    for (let i = 0; i < FRAME_LEN; i += 4) frame.push(bus.getLong(sp + i) | 0);
    return {
        regs: Int32Array.from(cpu.regs.slice(0, 15)), pc: cpu.regs[15] | 0, psl: cpu.psl,
        sisr: cpu.exc.sisr, bto: cpu.exc.sscBto | 0, frame, stop
    };
}

function compareCase(c, js, sr)
{
    let bad = [];
    let tag = `${c.fWrite ? "write" : "read"} size=${c.size} addr=0x${hex(c.addr)} case#${c.index}`;
    for (let r = 0; r < 15; r++) {
        if ((js.regs[r] | 0) !== (sr.regs[r] | 0)) bad.push(`${tag}: R${r} js=${hex(js.regs[r])} simh=${hex(sr.regs[r])}`);
    }
    if ((js.pc | 0) !== (sr.pc | 0)) bad.push(`${tag}: PC js=${hex(js.pc)} simh=${hex(sr.pc)}`);
    if ((js.psl | 0) !== (sr.psl | 0)) bad.push(`${tag}: PSL js=${hex(js.psl)} simh=${hex(sr.psl)}`);
    if ((js.bto | 0) !== (sr.bto | 0)) bad.push(`${tag}: BTO js=${hex(js.bto)} simh=${hex(sr.bto)}`);
    for (let i = 0; i < js.frame.length; i++) {
        if ((js.frame[i] | 0) !== (sr.frame[i] | 0)) {
            bad.push(`${tag}: frame[${i * 4}] js=${hex(js.frame[i])} simh=${hex(sr.frame[i])}`);
        }
    }
    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * Calibration -- confirm, against the real oracle, which candidate addresses this bus's           *
 * "reserved but not decoded" model actually matches SIMH on.                                      *
 * ------------------------------------------------------------------------------------------- */

function calibrate(simh, scratch)
{
    let cases = [];
    let index = 0;
    let key = [];                                       // parallel: {rangeName, addr, fWrite}
    for (let r of RANGES) {
        for (let addr of r.addrs) {
            for (let fWrite of [true, false]) {
                let c = new Case(index, fWrite, 4, addr);
                cases.push(c);
                key.push({range: r.name, addr, fWrite});
                index++;
            }
        }
    }
    let sr = runBatch(simh, cases, scratch);
    /*
     * TWO independent facts per (range, direction, address), tracked separately because they are
     * decided by DIFFERENT SIMH mechanisms (veracity re-dispatch, pcjsvax-446):
     *
     *   confirmed  -- does SIMH dispatch a machine check here AT ALL (PC reaches R_HANDLER)?  This
     *                 alone decides GRADED vs EXCLUDED.  An address is excluded only when SIMH
     *                 itself never dispatches -- a real device answered (REG's backed
     *                 sub-windows), or the reference is an IOPAGE/CQM WRITE, whose unbacked case
     *                 (vax_io.c WriteQb) sets a deferred `mem_err` and returns normally with no
     *                 synchronous exception at all (pcjsvax-d22).  Nothing else is excluded.
     *   btoSeen    -- given a dispatch happened, did ssc_bto actually get set?  IOPAGE/CQM READS
     *                 dispatch (ReadQb -> cq_merr -> the same MACH_CHECK) WITHOUT ever touching
     *                 ssc_bto (pcjsvax-d22) -- those are graded (confirmed=true) with btoSeen=false,
     *                 and cpustate.js's onBusFault() now reproduces exactly that split.  Everywhere
     *                 else that dispatches, ssc_bto is set (btoSeen=true).
     *
     * This is the fix for the veracity finding that calibrating "is it backed" and "does it match
     * my implementation" as ONE decision made the excluded set into an unexamined proxy for the
     * defect set: IOPAGE/CQM reads used to be excluded (as "different mechanism") instead of
     * graded, which is exactly where onBusFault() had a real bug (it set ssc_bto unconditionally).
     * Now the exclusion decision depends ONLY on whether SIMH dispatches -- never on whether the
     * CURRENT implementation happens to agree with it.
     */
    let confirmed = {write: new Map(), read: new Map()};     // rangeName -> Array<{addr, btoSeen}>
    let backed = {write: new Map(), read: new Map()};        // rangeName -> Array<addr>, SIMH never dispatched
    for (let r of RANGES) {
        confirmed.write.set(r.name, []); confirmed.read.set(r.name, []);
        backed.write.set(r.name, []); backed.read.set(r.name, []);
    }
    let notReached = [];
    for (let i = 0; i < cases.length; i++) {
        let res = sr.get(i);
        let k = key[i];
        let dir = k.fWrite ? "write" : "read";
        if (!res || !res.reached) { notReached.push(`calibrate ${dir} ${k.range}@0x${hex(k.addr)} (case ${i})`); continue; }
        let mchk = (res.pc >>> 0) === R_HANDLER;
        if (!mchk) {
            backed[dir].get(k.range).push(k.addr);
        } else {
            confirmed[dir].get(k.range).push({addr: k.addr, btoSeen: !!res.bto});
        }
    }
    return {confirmed, backed, notReached};
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors -- FAIL the run, do NOT scale down with case count                              *
 * ------------------------------------------------------------------------------------------- */

/*
 * EXPECTED_CALIBRATION -- committed against the real oracle (measured 2026-07-26, all six ranges,
 * candidatesFor()'s deterministic candidate set, ten addresses per range).  Asserted EXACTLY, not
 * as a floor: a rebase or an implementation change that silently widens the excluded set (or
 * shrinks the confirmed one) FAILS the run instead of the excluded set quietly absorbing whatever
 * calibrate() decides to hand it -- the exact failure mode the veracity re-dispatch found (the
 * excluded set was an unexamined proxy for wherever the implementation disagreed with the oracle).
 * `confirmed` counts BOTH btoSeen=true and btoSeen=false entries (IOPAGE/CQM reads are confirmed
 * with btoSeen=false -- see calibrate()'s doc comment); `backed` is SIMH never dispatching at all.
 */
const EXPECTED_CALIBRATION = {
    IOPAGE: {confirmed: {write: 0, read: 10}, backed: {write: 10, read: 0}},
    REG:    {confirmed: {write: 8, read: 8},  backed: {write: 2,  read: 2}},
    CQM:    {confirmed: {write: 0, read: 10}, backed: {write: 10, read: 0}},
    NVR:    {confirmed: {write: 0, read: 0},  backed: {write: 10, read: 10}}
    /* SSC removed by pcjsvax-320, CDG by pcjsvax-0b7 -- see RANGE_NAMES and the file header.
       CDG's row was `confirmed 0 / backed 10` in BOTH directions: it contributed nothing to the
       graded set and everything to the excluded one.  Now that bus.js decodes it (addCdg(),
       cdg.js) it is not a reserved range at all, and tests/cdgdiff.js grades it directly and at
       register level instead of this file recording it as a 100% expected divergence. */
};

/**
 * assertCalibration(cal)
 *
 * @param {Object} cal as returned by calibrate()
 * @returns {Array.<string>} mismatches (empty means calibration matches the committed numbers)
 */
function assertCalibration(cal)
{
    let bad = [];
    for (let r of RANGES) {
        let exp = EXPECTED_CALIBRATION[r.name];
        if (!exp) { bad.push(`CALIBRATION: range "${r.name}" has no EXPECTED_CALIBRATION entry`); continue; }
        for (let dir of ["write", "read"]) {
            let gotC = cal.confirmed[dir].get(r.name).length;
            let gotB = cal.backed[dir].get(r.name).length;
            if (gotC !== exp.confirmed[dir]) {
                bad.push(`CALIBRATION: ${r.name} confirmed.${dir} = ${gotC}, expected ${exp.confirmed[dir]}`);
            }
            if (gotB !== exp.backed[dir]) {
                bad.push(`CALIBRATION: ${r.name} backed.${dir} = ${gotB}, expected ${exp.backed[dir]}`);
            }
        }
    }
    return bad;
}

/*
 * MIN_RANGES_WITH_COVERAGE / MIN_RANGE_DIR_PAIRS / MIN_DISTINCT_ADDRESSES are DERIVED from
 * EXPECTED_CALIBRATION -- a committed, asserted constant (above) -- not from whatever a live run
 * happens to observe.  Computed once here so a change to EXPECTED_CALIBRATION (the only place
 * that legitimately changes, e.g. after a real device item lands) automatically updates the
 * floors that depend on it, instead of two numbers drifting apart silently.
 */
const RANGES_WITH_COVERAGE = Object.keys(EXPECTED_CALIBRATION)
    .filter((name) => EXPECTED_CALIBRATION[name].confirmed.write > 0 || EXPECTED_CALIBRATION[name].confirmed.read > 0);
const RANGE_DIR_PAIRS_WITH_COVERAGE = Object.keys(EXPECTED_CALIBRATION)
    .flatMap((name) => ["write", "read"].filter((dir) => EXPECTED_CALIBRATION[name].confirmed[dir] > 0)
        .map((dir) => `${name}:${dir}`));
const EXPECTED_DISTINCT_ADDRESSES = Object.values(EXPECTED_CALIBRATION)
    .reduce((n, e) => n + Math.max(e.confirmed.write, e.confirmed.read), 0);   // per-range union, upper bound

const MIN_RANGES_WITH_COVERAGE = RANGES_WITH_COVERAGE.length;
const MIN_RANGE_DIR_PAIRS = RANGE_DIR_PAIRS_WITH_COVERAGE.length;
/*
 * A FLOOR on distinct addresses used across the WHOLE pool actually driving case generation --
 * not merely on calibrate()'s own report.  This is what closes the adversary's specific attack:
 * truncating the pool AFTER calibration reports correctly (calibrate() itself would still pass
 * assertCalibration() above) but BEFORE case generation, so the run graded almost nothing while
 * every floor upstream of this one stayed green.
 */
const MIN_DISTINCT_ADDRESSES = Math.max(4, EXPECTED_DISTINCT_ADDRESSES - 8);   // measured 28; keep slack, not zero

/*
 * --cases FLOOR.  Below this, the run must FAIL, not clamp itself up to look fine (the specific
 * vacuous-floor attack the veracity re-dispatch found: `--cases 1` and `--cases 0` both used to
 * report PASS).  See main()'s argument parsing for the exit-non-zero enforcement.
 */
const MIN_CASES_FLOOR = 150;
const MIN_TOTAL_OPS = 240;
const MIN_PER_DIRECTION = 60;
const MIN_PER_SIZE = 70;
const MIN_BTO_SET = 150;

/* ------------------------------------------------------------------------------------------- *
 * Self-check mutations                                                                          *
 * ------------------------------------------------------------------------------------------- */

/*
 * Only the exact methods each mutation touches -- NOT a whole-prototype scan.  cpustate.js's
 * VAXExc/CPUStateVAX prototypes carry getters (e.g. `trpirq`) that dereference instance state
 * (`this.exc`), which throws when read off the bare prototype object; a generic
 * Object.getOwnPropertyNames() snapshot has no `this` to give them.
 */
const MUTATED_METHODS = [
    [VAXExc, "busTimeout"], [VAXExc, "intexc"], [CPUStateVAX, "onBusFault"]
];
function snapshotProto()
{
    return MUTATED_METHODS.map(([cls, name]) => [cls, name, cls.prototype[name]]);
}
function restoreProto(save) { for (let [cls, name, fn] of save) cls.prototype[name] = fn; }

const MUTATIONS = [
    {name: "machine check raised but ssc_bto not set", apply() {
        VAXExc.prototype.busTimeout = function(fWrite) { return fWrite ? 0x82 : 0x80; };   // BTO untouched
    }},
    {name: "ssc_bto set but no exception raised", apply() {
        let origBT = VAXExc.prototype.busTimeout;
        CPUStateVAX.prototype.onBusFault = function(addr, access) {
            this.exc.busTimeout(access === VAX.ACCESS.WRITE);
            /* fall through: no throw, access silently proceeds with the all-ones/discarded value */
        };
    }},
    {name: "wrong SCB vector (dispatches through KSNV's entry instead of MCHK's)", apply() {
        let orig = VAXExc.prototype.intexc;
        VAXExc.prototype.intexc = function(cpu, vec, ipl, ei) {
            if (vec === SCB.MCHK) vec = 0x08;      // KSNV's offset -- a different SCB entry
            return orig.call(this, cpu, vec, ipl, ei);
        };
    }},
    {name: "fault swallowed entirely (no exception, no ssc_bto -- the pre-fix behavior)", apply() {
        CPUStateVAX.prototype.onBusFault = function(addr, access) { /* nothing at all */ };
    }},
    {name: "ssc_bto set unconditionally, including on IOPAGE/CQM reads (bug 1, veracity re-dispatch)", apply() {
        /* Reproduces the exact regression this re-dispatch fixed: onBusFault() calling
           busTimeout() for EVERY dispatch, without the ADDR_IS_IO/ADDR_IS_CQM exception. */
        CPUStateVAX.prototype.onBusFault = function(addr, access) {
            let p1 = this.exc.busTimeout(access === VAX.ACCESS.WRITE);
            let delta = (this.regs[15] - this.exc.faultPC) | 0;
            throw new VAXFault(-SCB.MCHK, p1, delta);
        };
    }}
];

function selfcheck(simh, scratch, opts)
{
    let results = [];
    for (let mut of MUTATIONS) {
        let save = snapshotProto();
        mut.apply();
        let caught = false, why = "";
        try {
            let r = runPhase(simh, scratch, {seed: opts.seed ^ 0x5A5A, cases: 30}, "selfcheck");
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
 * Sticky ssc_bto across a SECOND, independent fault                                              *
 * ------------------------------------------------------------------------------------------- */

/**
 * verifySecondFault(simh, scratch, addr1, addr2)
 *
 * Closes the specific gap disclosed in this item's own test_decisions: every other case here
 * proves ssc_bto gets SET once, but nothing proved it stays correctly set (and the SECOND fault's
 * own frame is computed fresh, not accidentally reusing the first one's p1/p2/st1/st2) across a
 * SECOND, independent fault in the same run.  Two probes back to back:
 *
 *   R_CODE's instruction faults on addr1 -> dispatches to R_HANDLER (a SEVERE exception, so this
 *   ALWAYS lands on the interrupt stack, PSL<IS> set, SP = IS - FRAME_LEN).
 *   R_HANDLER's OWN first instruction (deliberately another probe, not a NOP) faults on addr2 ->
 *   dispatches AGAIN.  This time intexc() takes the "ALREADY on the interrupt stack" branch (SIMH:
 *   `if (oldpsl & PSL_IS) { newpsl = PSL_IS; }`, no SP reload from IS, no `stk[]` save) -- a
 *   DIFFERENT code path than the first dispatch exercised, so this is not a redundant re-check of
 *   the same thing under a different name.
 *
 * Frame 1 sits at [IS-FRAME_LEN, IS-1] (untouched by the second dispatch, which pushes BELOW it);
 * frame 2 sits at [IS-2*FRAME_LEN, IS-FRAME_LEN-1].  Both are graded, plus ssc_bto after both.
 *
 * @param {string} simh
 * @param {string} scratch
 * @param {number} addr1
 * @param {number} addr2
 * @returns {Array.<string>} failures (empty means the two sides agree)
 */
function verifySecondFault(simh, scratch, addr1, addr2)
{
    let instr1 = buildInstr(true, 4, addr1);      // MOVL #0,@#addr1 -- placed at R_CODE
    let instr2 = buildInstr(true, 4, addr2);      // MOVL #0,@#addr2 -- placed at R_HANDLER itself
    let sp1 = (R_IS - FRAME_LEN) >>> 0;
    let sp2 = (R_IS - 2 * FRAME_LEN) >>> 0;

    let lines = ["set cpu 16m", "set cpu simhalt", "reset all", "deposit sysd bto 0"];
    for (let i = 0; i < FRAME_LEN; i += 4) lines.push(`deposit -l ${hex(sp2 + i)} 0`);   // zero BOTH frames
    lines.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit -l ${hex(R_SCBB + SCB.MCHK)} ${hex(R_HANDLER)}`);
    lines.push(`deposit KSP ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`);
    for (let i = 0; i < instr1.length; i++) lines.push(`deposit -b ${hex(R_CODE + i)} ${instr1[i].toString(16)}`);
    for (let i = 0; i < instr2.length; i++) lines.push(`deposit -b ${hex(R_HANDLER + i)} ${instr2[i].toString(16)}`);
    lines.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`, "step 2");
    lines.push(`examine -h PC,PSL`, `examine -h sysd bto`);
    for (let i = 0; i < FRAME_LEN; i += 4) lines.push(`examine -h ${hex(sp1 + i)}`);
    for (let i = 0; i < FRAME_LEN; i += 4) lines.push(`examine -h ${hex(sp2 + i)}`);
    lines.push("exit", "");

    let out = runSimh(simh, lines.join("\n"), path.join(scratch, "mchkdiff-secondfault.ini"));
    let vals = [];
    for (let line of out.split("\n")) {
        let m = line.match(VALUE_RE);
        if (m) vals.push(parseInt(m[2], 16) | 0);
    }
    let want = 2 + 1 + (FRAME_LEN / 4) * 2;
    if (vals.length < want) {
        throw new Error(`mchkdiff: second-fault probe produced ${vals.length}/${want} values; SIMH said:\n${out}`);
    }
    let sr = {
        pc: vals[0], psl: vals[1], bto: vals[2],
        frame1: vals.slice(3, 3 + FRAME_LEN / 4),
        frame2: vals.slice(3 + FRAME_LEN / 4, 3 + FRAME_LEN / 2)
    };

    let m = makeMachine();
    let {bus, cpu} = m;
    cpu.reset();
    for (let i = 0; i < FRAME_LEN; i += 4) bus.setLong(sp2 + i, 0);
    cpu.exc.scbb = R_SCBB;
    bus.setLong(R_SCBB + SCB.MCHK, R_HANDLER);
    cpu.exc.stk[4] = R_IS;
    for (let i = 0; i < instr1.length; i++) bus.setByte(R_CODE + i, instr1[i]);
    for (let i = 0; i < instr2.length; i++) bus.setByte(R_HANDLER + i, instr2[i]);
    cpu.psl = 0;
    cpu.regs[15] = R_CODE;
    try { cpu.stepCPU(2); } catch (e) { if (!(e instanceof VAXStop)) throw e; }
    let js = {
        pc: cpu.regs[15] | 0, psl: cpu.psl | 0, bto: cpu.exc.sscBto | 0,
        frame1: [], frame2: []
    };
    for (let i = 0; i < FRAME_LEN; i += 4) js.frame1.push(bus.getLong(sp1 + i) | 0);
    for (let i = 0; i < FRAME_LEN; i += 4) js.frame2.push(bus.getLong(sp2 + i) | 0);

    let bad = [];
    let tag = `second-fault addr1=0x${hex(addr1)} addr2=0x${hex(addr2)}`;
    if ((js.pc | 0) !== (sr.pc | 0)) bad.push(`${tag}: PC js=${hex(js.pc)} simh=${hex(sr.pc)}`);
    if ((js.psl | 0) !== (sr.psl | 0)) bad.push(`${tag}: PSL js=${hex(js.psl)} simh=${hex(sr.psl)}`);
    if ((js.bto | 0) !== (sr.bto | 0)) bad.push(`${tag}: BTO js=${hex(js.bto)} simh=${hex(sr.bto)}`);
    for (let i = 0; i < FRAME_LEN / 4; i++) {
        if ((js.frame1[i] | 0) !== (sr.frame1[i] | 0)) bad.push(`${tag}: frame1[${i * 4}] js=${hex(js.frame1[i])} simh=${hex(sr.frame1[i])}`);
        if ((js.frame2[i] | 0) !== (sr.frame2[i] | 0)) bad.push(`${tag}: frame2[${i * 4}] js=${hex(js.frame2[i])} simh=${hex(sr.frame2[i])}`);
    }
    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * The graded run: deterministic enumeration + randomized                                          *
 * ------------------------------------------------------------------------------------------- */

function poolAddresses(cal)
{
    let pool = [];
    for (let r of RANGES) {
        for (let e of cal.confirmed.write.get(r.name)) pool.push({range: r.name, addr: e.addr, fWrite: true, btoSeen: e.btoSeen});
        for (let e of cal.confirmed.read.get(r.name)) pool.push({range: r.name, addr: e.addr, fWrite: false, btoSeen: e.btoSeen});
    }
    return pool;
}

/*
 * Unaligned offsets relative to a longword-aligned confirmed-absent base address.  vax_mmu.h's
 * WriteU()/ReadU() operate on the LONGWORD CONTAINING the target address, so "is this backed" for
 * an unaligned reference is answered by the SAME ReadReg/WriteReg lookup as its aligned
 * container -- no separate calibration needed; compareCase() grades it against the real oracle
 * exactly like every other case.  +2 is omitted for WORD: offset+2 relative to a longword base is
 * itself word-aligned, so a word access there takes the ALIGNED fast path (no read-modify-write)
 * and is not a genuinely unaligned case -- see cpustate.js's onBusFault() doc comment for why the
 * read-modify-write path specifically is where the misclassified-MCHK-type bug the veracity
 * re-dispatch found could have lived (measured: it does not reproduce on this item's remaining
 * ranges post-pcjsvax-223, since ROM -- the one range with a read-succeeds/write-fails split -- is
 * no longer part of this item's scope; see the file header).
 */
const UNALIGNED_OFFSETS = {4: [1, 2, 3], 2: [1, 3]};

function poolStats(pool)
{
    let addrs = new Set(pool.map((p) => p.addr));
    let pairs = new Set(pool.map((p) => `${p.range}:${p.fWrite ? "write" : "read"}`));
    return {distinctAddresses: addrs.size, distinctPairs: pairs.size};
}

function runPhase(simh, scratch, opts, label)
{
    let cal = calibrate(simh, scratch);
    let calMismatch = assertCalibration(cal);
    let pool = poolAddresses(cal);
    if (!pool.length) throw new Error(`mchkdiff: calibration found NO confirmed address anywhere; cannot grade anything`);

    let rnd = mulberry32(opts.seed || 1);
    let cases = [];
    let index = 0;

    /* ENUMERATED: every confirmed (range, direction) entry, all three ALIGNED sizes, plus every
       UNALIGNED word/long offset of that same address. */
    for (let p of pool) {
        for (let size of [1, 2, 4]) {
            let c = new Case(index++, p.fWrite, size, p.addr);
            c.psl = 0;
            c.range = p.range;
            c.btoSeen = p.btoSeen;
            cases.push(c);
        }
        for (let size of [2, 4]) {
            for (let off of UNALIGNED_OFFSETS[size]) {
                let c = new Case(index++, p.fWrite, size, (p.addr + off) >>> 0);
                c.psl = 0;
                c.range = p.range;
                c.btoSeen = p.btoSeen;
                c.unaligned = true;
                cases.push(c);
            }
        }
    }
    /* RAM controls: must NOT machine-check, both directions, all sizes. */
    let ramAddrs = [0x1000, 0x200000, MEMSIZE - 0x10];
    let ramCaseIdx = [];
    for (let a of ramAddrs) {
        for (let fWrite of [true, false]) {
            for (let size of [1, 2, 4]) {
                let c = new Case(index++, fWrite, size, a);
                ramCaseIdx.push(c.index);
                cases.push(c);
            }
        }
    }
    /*
     * RANDOMIZED: draws from the pool (ALIGNED addresses only -- the enumerated phase above is
     * what exhaustively covers unaligned offsets; this phase's job is varying the SURROUNDING
     * machine state the enumerated phase does not), with PSL mode/IPL/CC noise and register/SISR
     * noise.
     */
    let nRandom = opts.cases;
    for (let k = 0; k < nRandom; k++) {
        let p = pick(rnd, pool);
        let size = pick(rnd, [1, 2, 4]);
        let c = new Case(index++, p.fWrite, size, p.addr);
        c.range = p.range;
        c.btoSeen = p.btoSeen;
        /*
         * Legal PSLs only -- SIMH refuses to `step` at all ("Unreasonable PSL value") otherwise,
         * which showed up as SIMH not executing the probe (PC unchanged) while JS, having no such
         * guard, ran it anyway.  Rule, matching excdiff.js's baseCase(): kernel mode may carry any
         * IPL 0..0x1F; a non-kernel mode must have IPL 0 (and PSL<IS> clear, which this generator
         * never sets).  PRV (previous mode) must be >= CUR.
         */
        let cur = pick(rnd, [0, 0, 0, 1, 2, 3]);        // mostly kernel
        let ipl = (cur === 0) ? Math.floor(rnd() * 0x20) : 0;
        let prv = cur + Math.floor(rnd() * (4 - cur));
        let cc = Math.floor(rnd() * 16);
        c.psl = ((cur << 24) | (prv << 22) | (ipl << 16) | cc) | 0;
        for (let r = 0; r < 15; r++) c.regs[r] = (Math.floor(rnd() * 0x100000000)) | 0;
        c.sisr = (rnd() < 0.3) ? (1 << (1 + Math.floor(rnd() * 14))) : 0;
        cases.push(c);
    }

    let stats = {
        nOps: cases.length, byDir: {write: 0, read: 0}, bySize: {1: 0, 2: 0, 4: 0}, byRange: {},
        nBtoSet: 0, nBtoClearExpected: 0, nUnaligned: 0, nRamOk: 0
    };
    for (let r of RANGES) stats.byRange[r.name] = 0;

    let failures = calMismatch.slice(), notReached = [];
    let ramSet = new Set(ramCaseIdx);
    let m = makeMachine();
    const BATCH = 80;
    for (let start = 0; start < cases.length; start += BATCH) {
        let batch = cases.slice(start, start + BATCH);
        let sr = runBatch(simh, batch, scratch);
        for (let c of batch) {
            let res = sr.get(c.index);
            if (!res || !res.reached) { notReached.push(`${label} case ${c.index} (SIMH produced ${res ? res.got : 0}/${res ? res.want : "?"} values)`); continue; }
            let js = runCaseJS(m, c);
            let bad = compareCase(c, js, res);
            if (bad.length) failures.push(...bad);
            if (ramSet.has(c.index)) {
                /* The "must NOT machine-check" control (DONE CONDITION): independently assert
                   neither side actually dispatched, not merely that the two sides AGREE (both
                   sides could agreeing on a wrong fault and this floor would still be silent). */
                let simhFaulted = (res.pc >>> 0) === R_HANDLER;
                let jsFaulted = (js.pc >>> 0) === R_HANDLER;
                if (simhFaulted) failures.push(`${label} RAM control case#${c.index} addr=0x${hex(c.addr)}: SIMH unexpectedly machine-checked`);
                if (jsFaulted) failures.push(`${label} RAM control case#${c.index} addr=0x${hex(c.addr)}: JS unexpectedly machine-checked`);
                if (!simhFaulted && !jsFaulted) stats.nRamOk++;
            } else {
                stats.byDir[c.fWrite ? "write" : "read"]++;
                stats.bySize[c.size]++;
                if (c.unaligned) stats.nUnaligned++;
                if (c.range) stats.byRange[c.range]++;
                if (c.btoSeen === true && res.bto) stats.nBtoSet++;
                if (c.btoSeen === false && !res.bto) stats.nBtoClearExpected++;
            }
        }
    }

    return {failures, notReached, stats, cal, pool};
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                            *
 * ------------------------------------------------------------------------------------------- */

/**
 * parseCases(raw)
 *
 * MEASURED BUG, fixed here (veracity re-dispatch): `+getArg("--cases", 300)` treated `--cases 0`
 * as falsy and silently substituted 300, and a separate `Math.max(n, 150)` clamped anything below
 * the floor UP instead of failing -- so `--cases 1` and `--cases 0` both used to report PASS.
 * Below MIN_CASES_FLOOR the run must FAIL, not quietly cover more than it was asked to.
 *
 * @param {string|null} raw
 * @returns {number} the parsed case count (may be below the floor; caller checks that explicitly)
 */
function parseCases(raw)
{
    if (raw === null || raw === undefined) return 300;
    let n = Number(raw);
    if (!Number.isFinite(n)) throw new Error(`mchkdiff.js: --cases "${raw}" is not a number`);
    return n;
}

function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, dflt) => { let i = argv.indexOf(name); return i >= 0 ? argv[i + 1] : dflt; };
    let simh = findSimh(getArg("--simh", null));
    let seed = +getArg("--seed", 0xB16B00B5);
    let nCases = parseCases(getArg("--cases", null));
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "vax-mchkdiff-"));

    console.log("VAX machine-check-on-absent-register differential test (pcjsvax-446)");
    console.log("  SIMH binary: %s", simh);
    console.log("  seed=0x%s cases=%d", hex(seed), nCases);

    if (nCases < MIN_CASES_FLOOR) {
        console.log("\nFAILED: --cases %d is below the enforced floor (%d); this run would under-cover " +
            "and must not be allowed to pass.", nCases, MIN_CASES_FLOOR);
        /* Scratch was just created above and nothing has used it yet -- remove it here rather than
           process.exit()-ing past the try/finally below, which only guards the run that follows
           (HANDOFF.md pcjsvax-bd1). */
        fs.rmSync(scratch, {recursive: true, force: true});
        process.exit(1);
    }

    let errors = [];
    try {
        let t0 = Date.now();
        let {failures, notReached, stats, cal, pool} = runPhase(simh, scratch, {seed, cases: nCases}, "main");
        console.log("  elapsed: %ds", ((Date.now() - t0) / 1000).toFixed(1));

        console.log("\nCalibration (measured against the real oracle, asserted against EXPECTED_CALIBRATION):");
        for (let r of RANGES) {
            let cw = cal.confirmed.write.get(r.name).length, cr = cal.confirmed.read.get(r.name).length;
            let bw = cal.backed.write.get(r.name).length, br = cal.backed.read.get(r.name).length;
            console.log(`  ${r.name.padEnd(8)} confirmed(graded): write=${cw} read=${cr}   ` +
                `backed(excluded, SIMH never dispatches): write=${bw} read=${br}`);
        }
        if (cal.notReached.length) {
            console.log("  calibration cases that did not reach comparison:");
            for (let n of cal.notReached) console.log("    " + n);
        }
        let {distinctAddresses, distinctPairs} = poolStats(pool);
        console.log(`  pool: ${pool.length} (range,direction,address) entries, ` +
            `${distinctAddresses} distinct addresses, ${distinctPairs} distinct (range,direction) pairs`);

        console.log("\nComparisons: ops=%d write=%d read=%d byte=%d word=%d long=%d unaligned=%d " +
            "bto-set=%d bto-clear(expected)=%d ram-controls-ok=%d",
            stats.nOps, stats.byDir.write, stats.byDir.read, stats.bySize[1], stats.bySize[2], stats.bySize[4],
            stats.nUnaligned, stats.nBtoSet, stats.nBtoClearExpected, stats.nRamOk);
        console.log("  by range: %s", JSON.stringify(stats.byRange));

        for (let f of failures.slice(0, 40)) errors.push(f);
        if (failures.length > 40) errors.push(`... and ${failures.length - 40} more failures`);
        for (let n of notReached) errors.push("NOT REACHED: " + n);

        let require = (cond, msg) => { if (!cond) errors.push("COVERAGE: " + msg); };
        require(stats.nOps >= MIN_TOTAL_OPS, `fewer than ${MIN_TOTAL_OPS} operations (${stats.nOps})`);
        require(stats.byDir.write >= MIN_PER_DIRECTION, `too few write probes (${stats.byDir.write})`);
        require(stats.byDir.read >= MIN_PER_DIRECTION, `too few read probes (${stats.byDir.read})`);
        require(stats.bySize[1] >= MIN_PER_SIZE, `too few byte-size probes (${stats.bySize[1]})`);
        require(stats.bySize[2] >= MIN_PER_SIZE, `too few word-size probes (${stats.bySize[2]})`);
        require(stats.bySize[4] >= MIN_PER_SIZE, `too few long-size probes (${stats.bySize[4]})`);
        require(stats.nUnaligned >= MIN_PER_SIZE, `too few unaligned probes (${stats.nUnaligned})`);
        require(stats.nBtoSet >= MIN_BTO_SET, `too few cases observing ssc_bto SET (${stats.nBtoSet})`);
        require(stats.nBtoClearExpected >= 60, `too few cases observing ssc_bto correctly CLEAR ` +
            `(IOPAGE/CQM reads, ${stats.nBtoClearExpected}) -- this is exactly where the bug the ` +
            `veracity re-dispatch found lived; a floor of 0 here would let it back in silently`);
        require(stats.nRamOk >= ramCaseCountFloor(), `too few RAM control cases confirmed non-faulting (${stats.nRamOk})`);
        let rangesWithCoverage = RANGES.filter((r) => stats.byRange[r.name] > 0).length;
        require(rangesWithCoverage >= MIN_RANGES_WITH_COVERAGE,
            `too few reserved ranges contributing confirmed comparisons (${rangesWithCoverage} of ` +
            `${MIN_RANGES_WITH_COVERAGE} expected: ${RANGES_WITH_COVERAGE.join(", ")})`);
        /*
         * These two are what specifically closes the "truncate the pool after calibration reports
         * correctly" attack: a pool sliced down to (say) 2 entries cannot reach either floor, even
         * though calibrate() itself -- and therefore assertCalibration() above -- would still see
         * and report the full, correct set.
         */
        require(distinctAddresses >= MIN_DISTINCT_ADDRESSES,
            `too few distinct confirmed addresses actually used for case generation ` +
            `(${distinctAddresses} of a floor of ${MIN_DISTINCT_ADDRESSES}) -- the pool may have been ` +
            `truncated after calibration ran`);
        require(distinctPairs >= MIN_RANGE_DIR_PAIRS,
            `too few distinct (range,direction) pairs actually used for case generation ` +
            `(${distinctPairs} of a floor of ${MIN_RANGE_DIR_PAIRS}: ${RANGE_DIR_PAIRS_WITH_COVERAGE.join(", ")})`);

        /*
         * Sticky ssc_bto across a SECOND, independent fault -- see verifySecondFault()'s doc
         * comment.  Two DISTINCT confirmed-write addresses, drawn from the pool rather than
         * hand-picked, so this stays honest if candidatesFor()/calibrate() ever produce a
         * different set.
         */
        let writeAddrs = [...new Set(pool.filter((p) => p.fWrite).map((p) => p.addr))];
        if (writeAddrs.length < 2) {
            errors.push(`COVERAGE: fewer than 2 distinct confirmed write addresses available for the ` +
                `second-fault check (${writeAddrs.length})`);
        } else {
            let secondFaultBad = verifySecondFault(simh, scratch, writeAddrs[0], writeAddrs[1]);
            console.log(`\nSecond-fault check (ssc_bto stays set, frames independent): ${secondFaultBad.length ? "FAILED" : "PASS"}`);
            for (let f of secondFaultBad) errors.push(f);
        }

        if (fSelfCheck) {
            console.log("\nSelf-check: the differential must FAIL when the mechanism is deliberately broken.");
            let results = selfcheck(simh, scratch, {seed});
            for (let r of results) if (!r.caught) errors.push(`SELFCHECK: mutation '${r.name}' was not detected`);
        }
    } finally {
        if (!process.env["VAX_MCHKDIFF_KEEP"]) fs.rmSync(scratch, {recursive: true, force: true});
    }

    if (errors.length) {
        console.log("\nFAILED (%d):", errors.length);
        for (let e of errors) console.log("  " + e);
        process.exit(1);
    }
    console.log("\nPASS: an absent physical register machine-checks, with matching ssc_bto (set OR " +
        "correctly clear), exactly as real SIMH does.");
}

function ramCaseCountFloor() { return 3 * 2 * 3 - 2; }     // 3 addrs * 2 dirs * 3 sizes, minus slack

if (process.argv[1] && path.resolve(process.argv[1]) == path.resolve(fileURLToPath(import.meta.url))) {
    main();
}

export { findSimh, buildInstr, candidatesFor, RANGES, makeMachine, runCaseJS, calibrate, poolAddresses };
