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
 * (the KA655 I/O, ROM, register, SSC and Qbus-memory ranges) must raise a VAX machine check
 * through the SCB -- exactly as a real KA655 does when the console ROM probes for hardware --
 * instead of stopping the simulator (cpustate.js's old onBusFault()).  The observable is TWO
 * things, not one: the machine check is delivered (new PC, new PSL, the exact 7-longword
 * exception+mcheck stack frame) AND the SSC bus-timeout register reads back with its timeout bit
 * set afterward.  This item does NOT decode any device register -- it is entirely about what
 * happens when one is ABSENT.
 *
 * NO SIMH PATCH IS NEEDED.  Unlike busdiff.js's/excdiff.js's siblings, this differential does not
 * need instrumented state: `deposit`/`step 1`/`examine` (stock SCP) is enough, because everything
 * graded here -- the new PC, the new PSL, the pushed stack frame, and the SSC bus-timeout
 * register (SIMH: `examine sysd bto`) -- is architecturally visible after one step.
 *
 * WHY EVERY "RESERVED" ADDRESS IS NOT A "MUST MACHINE-CHECK" ADDRESS ON REAL SIMH
 * ---------------------------------------------------------------------------------
 * BusVAX.RESERVED marks five ranges this bus reserves but does not decode.  Real SIMH, however,
 * DOES decode most of them (vax_sysdev.c's `regtable[]`: CQBIC, CMCTL, KA655 regs, CQMAP, SSC,
 * NVR, and a real ROM image) -- because SIMH implements the peripherals this project has not
 * built yet.  Measured directly (see calibrate() below, and its printed report):
 *
 *   - CDG_BASE is backed END TO END (`cdg_rd`/`cdg_wr` span exactly VAX.PHYSMEM.CDG_LENGTH):
 *     every access SIMH does not machine-check.  100% expected divergence; reported, not graded.
 *   - ROM_BASE READS are backed end to end (a real ROM image); ROM_BASE WRITES are NOT (`rom_rd`
 *     but no `rom_wr` in SIMH's regtable, so a write falls through to the machine-check default
 *     exactly like an absent register) -- confirmed by direct probe before this file was written.
 *   - REG_BASE is a MIX: several real sub-windows (CQBIC at +0, CMCTL at +0x100, KA655 regs at
 *     +0x4000, CQIPC at +0x1F40, CQMAP at +0x8000) are backed; the rest of its 512KB genuinely
 *     machine-checks on both sides THROUGH ReadReg()/WriteReg(), the mechanism this item models.
 *   - IOPAGE_BASE and CQM_BASE are a SECOND, DIFFERENT divergence, discovered empirically while
 *     writing this file (an early run showed ssc_bto staying clear despite PC reaching the
 *     handler -- not a bug, a different SIMH code path).  `ADDR_IS_IO`/`ADDR_IS_CQM` route these
 *     two ranges to vax_io.c's `ReadQb()`/`ReadIO()`/`WriteQb()`, not vax_sysdev.c's ReadReg/
 *     WriteReg: an unbacked reference there calls `cq_merr()` (sets the CQBIC's DSER/MEAR error
 *     registers, which this item does not model) and, for READS ONLY, the SAME `MACH_CHECK()` --
 *     but NEVER touches ssc_bto.  WRITES there don't even raise the exception synchronously:
 *     `WriteQb()`'s unbacked case sets `mem_err = 1` (a DEFERRED MEMERR interrupt) and returns
 *     normally.  This matches the rd item's own measured-facts section, which cites ONLY
 *     ReadReg/WriteReg and ReadIPR/WriteIPR -- never ReadQb/WriteQb/cq_merr -- so modelling the
 *     Qbus path is a different (later, device-shaped) item's work, not a gap in this one.  Both
 *     ranges are therefore excluded from the graded pool and reported under "different mechanism"
 *     rather than silently absorbed into "confirmed absent".
 *
 * So the address pool this differential grades against is not "every address in
 * BusVAX.RESERVED" -- it is CALIBRATED against the real oracle first: candidate addresses (walked
 * OFF BusVAX.RESERVED programmatically, never hand-enumerated) are probed once, and only the ones
 * where BOTH the machine check fires AND ssc_bto is set go into the comparison pool -- exactly the
 * two observables the DONE CONDITION names.  Everything else is reported by name as an excluded,
 * out-of-scope divergence -- the same convention busdiff.js's reportScopeGaps() established for
 * the console-EXAMINE path.  This is why calibrate() is not optional plumbing: without it the
 * "randomized" pool would mostly compare a JS fault against a SIMH success (or a different SIMH
 * fault mechanism) and every failure would be noise, exactly the uniform-address-pool trap
 * docs/design/vax-on-pcjs.md's testing lesson describes for busdiff.js.
 *
 * TWO PHASES, PER THE PROJECT'S STANDING RULE
 * --------------------------------------------
 *   ENUMERATED   Deterministic: every CONFIRMED-ABSENT address (boundary points of every
 *                reserved range, from calibrate()), both directions (read/write, MOVx vs TSTx),
 *                all three sizes (byte/word/long).  Guarantees full-range, full-size coverage
 *                that a random draw cannot promise -- this project has no real workload that
 *                probes devices yet (the console ROM is a LATER milestone), so this phase stands
 *                in for it.
 *   RANDOMIZED   Random draws from the same confirmed pool, with random surrounding machine state
 *                (PSL mode/IPL/condition codes, SISR, general registers) that the deterministic
 *                phase does not vary -- old PSL/CC values ride into the pushed exception frame
 *                unchanged, so this is the phase that catches a wrong pass-through of that state.
 *
 * WHAT IS DELIBERATELY NOT MODELLED (see exc.js's busTimeout()/takeFault() doc comments)
 * -----------------------------------------------------------------------------------------
 *   - SIMH's REF_P (mchk_ref=1) half of the machine-check "address" parameter.  Every case here
 *     reaches the fault through mmu.readData()/writeData() (an ordinary instruction's data
 *     reference), which is SIMH's REF_V (0) path -- the one a console-ROM probe actually uses.
 *   - CADR/MSER (state1's low 16 bits): SSC/CMCTL device state exc.js defers (IPR_DEVICE). No
 *     case here writes them, so both sides read 0 there by construction, not by a modelled match.
 *   - Any machine-check trigger other than a bus fault (parity/ECC, etc.) -- none exist yet.
 *   - Unaligned / cross-boundary probe accesses. busdiff.js already grades the bus's byte/word/
 *     long stitching; this file grades the FAULT and its DISPATCH, not addressing arithmetic.
 *
 *      node machines/dec/vax/tests/mchkdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_BIN, else the scratch build)
 *        --cases N         randomized cases (default 300, floor 150)
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

/** {name, base, len, addrs} for every BusVAX.RESERVED entry -- names are diagnostic labels only. */
const RANGE_NAMES = ["CDG", "IOPAGE", "ROM", "REG", "CQM"];
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
    let confirmed = {write: new Map(), read: new Map()};     // rangeName -> Array<addr>
    let backed = {write: new Map(), read: new Map()};        // SIMH did not machine-check at all
    let otherMech = {write: new Map(), read: new Map()};     // SIMH machine-checked WITHOUT ssc_bto
    for (let r of RANGES) {
        confirmed.write.set(r.name, []); confirmed.read.set(r.name, []);
        backed.write.set(r.name, []); backed.read.set(r.name, []);
        otherMech.write.set(r.name, []); otherMech.read.set(r.name, []);
    }
    let notReached = [];
    for (let i = 0; i < cases.length; i++) {
        let res = sr.get(i);
        let k = key[i];
        let dir = k.fWrite ? "write" : "read";
        if (!res || !res.reached) { notReached.push(`calibrate ${dir} ${k.range}@0x${hex(k.addr)} (case ${i})`); continue; }
        let mchk = (res.pc >>> 0) === R_HANDLER;
        if (!mchk) {
            /* SIMH did not dispatch a machine check at all: either a real device answered (CDG,
               the backed windows of REG, ROM reads), or -- for IOPAGE/CQM writes -- SIMH's
               cq_merr()/WriteQb() only sets a DEFERRED mem_err (an async MEMERR interrupt), which
               is not this item's scope (see the file header's ReadQb/WriteQb note) and looks
               identical to "backed" from PC alone. Either way: excluded, not a bug. */
            backed[dir].get(k.range).push(k.addr);
        } else if (!res.bto) {
            /*
             * MEASURED, not assumed (see the file header): IOPAGE_BASE and CQM_BASE do not route
             * through vax_sysdev.c's ReadReg()/WriteReg() at all -- ADDR_IS_IO/ADDR_IS_CQM send
             * them to vax_io.c's ReadQb()/ReadIO(), whose unbacked case is `cq_merr()` (a CQBIC
             * DSER/MEAR error register this item does not model) followed by the SAME
             * MACH_CHECK(), but WITHOUT ever touching ssc_bto.  A real machine check that reaches
             * this handler with the SSC bus-timeout bit still clear is therefore evidence of that
             * DIFFERENT mechanism, not a bug in either machine -- excluded from the graded pool,
             * same as `backed`, and reported separately so it isn't confused with one.
             */
            otherMech[dir].get(k.range).push(k.addr);
        } else {
            confirmed[dir].get(k.range).push(k.addr);
        }
    }
    return {confirmed, backed, otherMech, notReached};
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors -- FAIL the run, do NOT scale down with case count                              *
 * ------------------------------------------------------------------------------------------- */

/*
 * Floors measured against the ENFORCED MINIMUM run (`--cases` clamped to 150 -- see nRandom in
 * runPhase()), not the default (300): a floor that only the default satisfies would silently pass
 * at the minimum while covering less than it claims.  Only TWO of the five BusVAX.RESERVED ranges
 * can ever contribute a confirmed-absent comparison in this item's scope (ROM writes and REG's
 * gaps) -- CDG is backed end to end, and IOPAGE/CQM are the different-mechanism exclusion the file
 * header documents -- so MIN_RANGES_WITH_COVERAGE is 2, not "most of 5".
 */
const MIN_TOTAL_OPS = 240;
const MIN_PER_DIRECTION = 60;
const MIN_PER_SIZE = 70;
const MIN_BTO_SET = 200;
const MIN_RANGES_WITH_COVERAGE = 2;           // of 5; see the comment above and the file header

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
 * The graded run: deterministic enumeration + randomized                                          *
 * ------------------------------------------------------------------------------------------- */

function poolAddresses(cal)
{
    let pool = [];
    for (let r of RANGES) {
        for (let addr of cal.confirmed.write.get(r.name)) pool.push({range: r.name, addr, fWrite: true});
        for (let addr of cal.confirmed.read.get(r.name)) pool.push({range: r.name, addr, fWrite: false});
    }
    return pool;
}

function runPhase(simh, scratch, opts, label)
{
    let cal = calibrate(simh, scratch);
    let pool = poolAddresses(cal);
    if (!pool.length) throw new Error(`mchkdiff: calibration found NO confirmed-absent address anywhere; cannot grade anything`);

    let rnd = mulberry32(opts.seed || 1);
    let cases = [];
    let index = 0;

    /* ENUMERATED: every confirmed (range, direction) entry, all three sizes. */
    for (let p of pool) {
        for (let size of [1, 2, 4]) {
            let c = new Case(index++, p.fWrite, size, p.addr);
            c.psl = 0;
            cases.push(c);
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
    /* RANDOMIZED: draws from the pool, with PSL mode/IPL/CC noise and register/SISR noise. */
    let nRandom = Math.max(opts.cases || 300, 150);
    for (let k = 0; k < nRandom; k++) {
        let p = pick(rnd, pool);
        let size = pick(rnd, [1, 2, 4]);
        let c = new Case(index++, p.fWrite, size, p.addr);
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

    let stats = {nOps: cases.length, byDir: {write: 0, read: 0}, bySize: {1: 0, 2: 0, 4: 0}, byRange: {}, nBtoSet: 0, nRamOk: 0};
    for (let r of RANGES) stats.byRange[r.name] = 0;

    let failures = [], notReached = [];
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
                if (res.bto) stats.nBtoSet++;
                let p = pool.find((q) => q.addr === c.addr && q.fWrite === c.fWrite);
                if (p) stats.byRange[p.range]++;
            }
        }
    }

    return {failures, notReached, stats, cal};
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                            *
 * ------------------------------------------------------------------------------------------- */

function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, dflt) => { let i = argv.indexOf(name); return i >= 0 ? argv[i + 1] : dflt; };
    let simh = findSimh(getArg("--simh", null));
    let seed = +getArg("--seed", 0xB16B00B5);
    let nCases = +getArg("--cases", 300);
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "vax-mchkdiff-"));

    console.log("VAX machine-check-on-absent-register differential test (pcjsvax-446)");
    console.log("  SIMH binary: %s", simh);
    console.log("  seed=0x%s cases=%d", hex(seed), nCases);

    let errors = [];
    try {
        let t0 = Date.now();
        let {failures, notReached, stats, cal} = runPhase(simh, scratch, {seed, cases: nCases}, "main");
        console.log("  elapsed: %ds", ((Date.now() - t0) / 1000).toFixed(1));

        console.log("\nCalibration (measured against the real oracle, not assumed):");
        for (let r of RANGES) {
            let cw = cal.confirmed.write.get(r.name).length, cr = cal.confirmed.read.get(r.name).length;
            let bw = cal.backed.write.get(r.name).length, br = cal.backed.read.get(r.name).length;
            let ow = cal.otherMech.write.get(r.name).length, or_ = cal.otherMech.read.get(r.name).length;
            console.log(`  ${r.name.padEnd(8)} confirmed-absent(graded): write=${cw} read=${cr}   ` +
                `backed-by-SIMH(excluded): write=${bw} read=${br}   ` +
                `different-mechanism(excluded, no ssc_bto): write=${ow} read=${or_}`);
        }
        if (cal.notReached.length) {
            console.log("  calibration cases that did not reach comparison:");
            for (let n of cal.notReached) console.log("    " + n);
        }

        console.log("\nComparisons: ops=%d write=%d read=%d byte=%d word=%d long=%d bto-set=%d ram-controls-ok=%d",
            stats.nOps, stats.byDir.write, stats.byDir.read, stats.bySize[1], stats.bySize[2], stats.bySize[4],
            stats.nBtoSet, stats.nRamOk);
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
        require(stats.nBtoSet >= MIN_BTO_SET, `too few cases observing ssc_bto set (${stats.nBtoSet})`);
        require(stats.nRamOk >= ramCaseCountFloor(), `too few RAM control cases confirmed non-faulting (${stats.nRamOk})`);
        let rangesWithCoverage = RANGES.filter((r) => stats.byRange[r.name] > 0).length;
        require(rangesWithCoverage >= MIN_RANGES_WITH_COVERAGE,
            `too few reserved ranges contributing confirmed-absent comparisons (${rangesWithCoverage}); ` +
            `note CDG_BASE is expected to contribute none (fully backed by SIMH's cache-diagnostic model)`);

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
    console.log("\nPASS: an absent physical register machine-checks, with matching ssc_bto, exactly as real SIMH does.");
}

function ramCaseCountFloor() { return 3 * 2 * 3 - 2; }     // 3 addrs * 2 dirs * 3 sizes, minus slack

if (process.argv[1] && path.resolve(process.argv[1]) == path.resolve(fileURLToPath(import.meta.url))) {
    main();
}

export { findSimh, buildInstr, candidatesFor, RANGES, makeMachine, runCaseJS, calibrate, poolAddresses };
