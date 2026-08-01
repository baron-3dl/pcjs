/**
 * @fileoverview Differential test: unbacked Qbus I/O-page / CQBIC-memory access uses cq_merr's
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
 *               DSER/MEAR latch and (for writes) a DEFERRED mem_err, not ssc_bto -- vs. a real
 *               Open SIMH microvax3900
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-d22, the item pcjsvax-446's veracity re-dispatch filed rather than silently absorbing
 * (its own file header points here for exactly this): an unbacked reference to the KA655's Qbus
 * I/O page or CQBIC Qbus-memory window (VAX.isQbusAddr() -- ADDR_IS_IO()/ADDR_IS_CQM(),
 * vaxmod_defs.h) is NOT the ReadReg()/WriteReg() "default case sets ssc_bto and machine-checks"
 * mechanism mchkdiff.js grades. It is vax_io.c's ReadQb()/WriteQb(), which:
 *
 *   - on READ: calls cq_merr() (latches the CQBIC's DSER/MEAR error registers) and THEN the SAME
 *     MACH_CHECK() the register-space path uses -- but NEVER touches ssc_bto.  mchkdiff.js already
 *     grades this half (PC/PSL/frame, 0 diverged) and explicitly excludes DSER/MEAR from its own
 *     scope, pointing here.
 *   - on WRITE: calls cq_merr() and sets a DEFERRED `mem_err` flag, THEN RETURNS NORMALLY -- no
 *     synchronous exception at all.  The write is discarded (there is nothing to store into) and
 *     the instruction completes; PC simply advances.  mem_err is delivered LATER, at the next
 *     instruction boundary where IPL permits, through the SAME generic hardware-interrupt seam
 *     every other device will use (evalInt()/getVector()'s IPL_MEMERR case, SCB.MEMERR) --
 *     confirmed sound and installed against, not rebuilt, here.
 *
 * This file grades all four things the item's DONE CONDITION names: (1) whether a synchronous
 * exception is taken at all (reads yes, writes no); (2) the machine-check frame when one is taken;
 * (3) DSER/MEAR contents; (4) the deferred mem_err's later delivery (vector, frame, and IPL
 * masking/unmasking).
 *
 * CALIBRATION IS NEEDED FOR IOPAGE, AND THIS WAS MEASURED, NOT ASSUMED.  An early draft of this
 * file assumed IOPAGE and CQM were both uniformly unbacked (bus.js's addRom()/addSsc() are both
 * entirely outside these two physical ranges, so nothing IN THIS PROJECT decodes any sub-window of
 * either).  Running the randomized phase against the real oracle immediately falsified that for
 * IOPAGE: `show qba iospace` on the stock microvax3900 lists DZ, RQ, TS, RL, XQ, TQ, the QBA's own
 * self-registration, and LPT autoconfigured at fixed IOPAGE sub-windows -- real SIMH devices this
 * project implements none of, which answer a probe instead of faulting.  mchkdiff.js's OWN IOPAGE
 * calibration (EXPECTED_CALIBRATION: confirmed.read=10, backed=0) never caught this because its
 * candidatesFor()-derived boundary/stride addresses happen, by small-integer coincidence, to dodge
 * every one of those windows -- not because IOPAGE is actually uniform.  calibrate() below settles
 * the question by EXECUTION, against the oracle ONLY (never against this file's own JS -- see
 * calibrate()'s doc comment for why that distinction is load-bearing), for every candidate this file
 * actually uses, every run: a leaf is CONFIRMED only when the oracle's observable state -- PC, DSER,
 * MEAR, and (for writes) whether the deferred interrupt actually delivers -- matches EXACTLY what an
 * architecturally unbacked reference must produce; anything else is BACKED (a real device answered,
 * or a device corrupted shared CQBIC state without being the generic fallback) and excluded,
 * reported by name, never silently dropped.  CQM is calibrated too, for the same rule-8 reason
 * ("verify agent claims by executing them") even though no run has found a backed CQM address there.
 *
 * That measured fact is *why* the RANDOMIZED phase below can draw a genuinely uniform address from
 * the WHOLE range instead of a small pre-vetted pool -- calibration decides CONFIRMED/BACKED per
 * draw, so a uniform draw stays valid without needing the range to be uniform in truth.  This is
 * what makes the randomized phase a structurally independent view from the ENUMERATED phase, not
 * "the same pool at a different granularity" (the shape the veracity re-dispatch on pcjsvax-446
 * charged: both of ITS phases drew from one oracle-calibrated pool and were blind to the same two
 * bugs identically). Here:
 *
 *   ENUMERATED   Deterministic: candidatesFor()'s boundary/stride addresses (imported from
 *                mchkdiff.js -- the SAME programmatic derivation off BusVAX.RESERVED-shaped
 *                ranges, never hand-enumerated), calibrated, every direction, every ALIGNED size,
 *                plus every UNALIGNED offset of every address -- exhaustive at the boundaries.
 *   RANDOMIZED   A byte offset drawn UNIFORMLY across the entire IOPAGE/CQM span (not from the
 *                boundary set above), calibrated, random size, random direction, and random
 *                surrounding PSL (mode/IPL/CC)/register state -- including IPL values AT and ABOVE
 *                IPL_MEMERR (0x1D), which is what exercises delivery masking: a case with IPL
 *                already >= 0x1D at write time must NOT see the interrupt on the very next
 *                instruction boundary, and the live-oracle comparison (not a hand-derived
 *                expectation) is what proves it either way.
 *
 * A THIRD, NAMED check -- verifyDeferredDelivery() -- is deterministic and not part of either
 * phase's pool: raise IPL to 0x1F, take an unbacked write, step across several NOPs proving NO
 * dispatch happens while masked, then lower IPL and prove the SAME pending mem_err delivers on the
 * very next boundary, vector SCB.MEMERR, frame and PSL<IPL> matching the real oracle exactly.
 *
 * A DISCOVERED, OUT-OF-SCOPE BUG (NOT FIXED HERE -- filed as pcjsvax-1be)
 * -------------------------------------------------------------------------
 * This file's cases originally randomized SISR the same way mchkdiff.js's do.  That exposed a REAL,
 * GENERAL defect no single-step differential (mchkdiff.js, excdiff.js, ...) could see: real SIMH
 * resets `in_ie = 0` unconditionally at the TOP of every `sim_instr()` call (vax_cpu.c:514) -- i.e.
 * at the start of EVERY SCP "step" -- so a step that hits STOP_INIE never poisons the NEXT one.
 * exc.js's `inIE` is only ever cleared inside intexc()'s own normal-completion path; if intexc()
 * itself throws (its parameter pushes can fault, e.g. onto a corrupted stack), `inIE` stays stuck
 * at 1, and cpustate.js's stepCPU() never resets it independently the way vax_cpu.c's entry does --
 * so EVERY subsequent step also throws STOP_INIE in JS, where real SIMH's second "step 1" recovers
 * and proceeds normally.  This file's OWN two-step-per-case shape is what surfaces it (a random
 * SISR bit eligible at the noise IPL preempts step 1's dispatch of the actual probe; if THAT
 * preemption's push then faults -- likelier once R14 is fully random -- step 2 diverges for a
 * reason that has nothing to do with cq_merr/DSER/MEAR/mem_err).  It is a general dispatch-retry
 * gap in exc.js/cpustate.js, not a Qbus/CQBIC defect, so it is not fixed by this item and this
 * file's SISR stays 0 by construction to avoid grading an unrelated, already-filed defect as if it
 * were this item's own (see randomPSL()'s doc comment).
 *
 * WHAT IS DELIBERATELY NOT GRADED
 * --------------------------------
 *   - ssc_bto: already mchkdiff.js's job; a Qbus fault never touches it (asserted here too, cheaply,
 *     as a sanity check alongside DSER/MEAR, but the exhaustive floors are mchkdiff.js's).
 *   - CQDSER_MPE/TMO/SNX (parity/Qbus-grant-timeout/slave-NXM) and DSER's W1C semantics -- no
 *     trigger for any of those exists in this machine; see exc.js's cqMerr() doc comment.
 *   - The REF_P (physical-context) half of mchk_ref, same disclosed gap as mchkdiff.js's.
 *   - The general inIE-retry gap above (pcjsvax-1be) -- discovered, disclosed, deliberately not
 *     exercised here (SISR stays 0), not fixed here.
 *
 *      node machines/dec/vax/tests/cqmerrdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_BIN, else the scratch build)
 *        --cases N         randomized cases (default 300; below MIN_CASES_FLOOR the run FAILS --
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
import { VAXExc, SCB } from "../modules/v2/exc.js";
import { VAXFault } from "../modules/v2/decode.js";
import MMUVAX from "../modules/v2/mmu.js";
import { buildInstr, candidatesFor, findSimh } from "./mchkdiff.js";

/* ------------------------------------------------------------------------------------------- *
 * Small utilities -- same PRNG/hex/pick as every other VAX differential                            *
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

function runSimh(bin, script, iniPath)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 10 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test                                                                         *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE = 0x01000000;

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    return {bus, cpu};
}

const R_SCBB     = 0x00100000;
const R_MCHK_H   = 0x00102000;          // SCBB+SCB.MCHK's target -- a read probe MCHK's here
const R_MEMERR_H = 0x00102100;          // SCBB+SCB.MEMERR's target -- a deferred write lands here
const R_CODE     = 0x00104000;
const R_KSP      = 0x00110000;
const R_IS       = 0x00118000;
const FRAME_LEN  = 28;                  // MCHK's 7-longword frame (8 old PC/PSL + 20 mcheck params)
const IFRAME_LEN = 8;                   // an ordinary interrupt's 2-longword frame (old PC/PSL)

/** Opcode numbers, resolved by mnemonic -- never hand-transcribed. */
function opcodeOf(name)
{
    let opc = OPCODES.indexOf(name);
    if (opc < 0) throw new Error(`cqmerrdiff.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
    return opc;
}
const OPC_NOP = opcodeOf("NOP"), OPC_HALT = opcodeOf("HALT");

/* ------------------------------------------------------------------------------------------- *
 * The Qbus ranges -- IOPAGE and CQM ONLY (REG/CDG/NVR are mchkdiff.js's, and go through            *
 * ReadReg/WriteReg, not ReadQb/WriteQb -- out of scope here).                                     *
 * ------------------------------------------------------------------------------------------- */

const RANGES = [
    {name: "IOPAGE", base: VAX.PHYSMEM.IOPAGE_BASE, len: VAX.PHYSMEM.IOPAGE_LENGTH},
    {name: "CQM",    base: VAX.PHYSMEM.CQM_BASE,    len: VAX.PHYSMEM.CQM_LENGTH}
];
/* candidatesFor() (mchkdiff.js) is the SAME programmatic boundary/stride derivation used there --
   reused rather than re-derived, per standing rule 5 (never hand-enumerate a scope list). */
const ENUM_ADDRS = RANGES.flatMap((r) => candidatesFor(r.base, r.len).map((addr) => ({range: r.name, addr})));

const UNALIGNED_OFFSETS = {4: [1, 2, 3], 2: [1, 3]};

/* ------------------------------------------------------------------------------------------- *
 * Case: one probe, ONE OR TWO instructions after it (a probe is always followed by a NOP so a     *
 * write's deferred delivery has somewhere to land the NEXT step; a read never reaches the NOP,     *
 * it MCHKs on the probe itself).                                                                    *
 * ------------------------------------------------------------------------------------------- */

const CASE_MARK = "@@CQM@@";

class Case {
    constructor(index, fWrite, size, addr, range)
    {
        this.index = index;
        this.fWrite = fWrite;
        this.size = size;
        this.addr = addr >>> 0;
        this.range = range;
        this.psl = 0;
        this.regs = new Int32Array(15);
        this.sisr = 0;
        this.unaligned = false;
    }
}

const NAMES = (function() {
    let n = [];
    for (let r = 0; r < 15; r++) n.push("R" + r);
    n.push("PC", "PSL", "SISR");
    return n;
})();

/*
 * CQBIC_MAP_LEN -- CQMAPSIZE (vaxmod_defs.h: CQMAPASIZE=15, CQMAPSIZE=1<<15=0x8000).  CQM's
 * cqm_rd()/cqm_wr() do NOT go straight to cq_merr(): they first walk qba_map_addr(), which reads a
 * map ENTRY out of real memory at `(qblk<<2 & CQMAPAMASK) + cq_mbr` (cq_mbr is 0 -- nothing in this
 * project, or in a driver-less boot, ever programs the QBA map register -- see pcjsvax-69a for
 * decoding CQBIC registers).  MEASURED DIRECTLY: on a freshly launched SIMH process this map region
 * is NOT reliably zero (ordinary process memory, not architecturally guaranteed clear), so whether
 * a CQM reference reads as an INVALID map entry (cq_merr, "master" NXM, DSER<MNX> -- what this file
 * models) or a VALID-but-garbage one (cq_serr, "slave" NXM, DSER<SNX> -- a different mechanism this
 * item does not model, since it depends on undefined map contents, not architecture) depends on
 * whatever bytes happen to be there.  Zeroing this region ONCE, before any case, makes every map
 * entry read VLD-clear deterministically on both sides -- exactly the master-NXM path this file
 * grades -- the same way mchkdiff.js's `deposit sysd bto 0` neutralizes a different piece of sticky
 * state for the same "make the oracle and the model start from the same footing" reason.  Zeroed
 * ONCE per script (not per case): nothing this file ever deposits touches an address below R_SCBB
 * (0x100000), so the map region cannot be dirtied mid-batch.
 */
const CQBIC_MAP_LEN = 0x8000;

/*
 * buildScript(cases) -- "step 2" always: the probe, then whichever instruction follows.  For a
 * READ, step 1 already lands on R_MCHK_H (a HALT there stops the SECOND step from doing anything
 * observable -- SCP just reports "HALT instruction" again and every examine after step 2 repeats
 * step 1's values, which compareCase() tolerates because it grades step 1 and step 2 the same way
 * a WRITE case does: nothing requires them to differ).  For a WRITE, step 1 completes normally
 * (PC advances past the probe) and step 2 is where a deferred mem_err delivers, IF THE STARTING
 * PSL's IPL permits it -- exactly the fact this file's randomized IPL noise is designed to probe.
 */
function buildScript(cases)
{
    let L = ["set cpu " + (MEMSIZE >> 20) + "m", "set cpu simhalt", `deposit 0:${(CQBIC_MAP_LEN - 1).toString(16).toUpperCase()} 0`];
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        L.push("reset all");
        L.push("deposit sysd bto 0");
        /* Zero BOTH frame areas fresh every case -- same convention as mchkdiff.js's, for the
           same cross-case/cross-batch contamination reason (a fresh SIMH process per batch vs. one
           JS machine reused for the whole run). */
        let spLo = (R_IS - 2 * FRAME_LEN) >>> 0;
        for (let i = 0; i < 2 * FRAME_LEN; i += 4) L.push(`deposit -l ${hex(spLo + i)} 0`);
        L.push(`deposit SCBB ${hex(R_SCBB)}`);
        L.push(`deposit -l ${hex(R_SCBB + SCB.MCHK)} ${hex(R_MCHK_H)}`);
        /* MEMERR's SCB entry has bit<0> SET (dispatch on the interrupt stack) -- the convention
           real OS SCB tables use for every hardware-interrupt vector, and what keeps the frame's
           location fixed at IS-8..IS-1 regardless of the case's noise-generated current mode,
           mirroring intexc()'s "already forced onto IS" path a SEVERE exception takes
           unconditionally (see mchkdiff.js's own R_HANDLER, which needs no such bit because MCHK
           is always severe). */
        L.push(`deposit -l ${hex(R_SCBB + SCB.MEMERR)} ${hex(R_MEMERR_H | 1)}`);
        for (let r = 0; r < 15; r++) L.push(`deposit R${r} ${hex(c.regs[r])}`);
        L.push(`deposit KSP ${hex(R_KSP)}`);
        L.push(`deposit IS ${hex(R_IS)}`);
        L.push(`deposit SISR ${hex(c.sisr)}`);
        let instr = buildInstr(c.fWrite, c.size, c.addr);
        let p = R_CODE;
        for (let i = 0; i < instr.length; i++) { L.push(`deposit -b ${hex(p)} ${instr[i].toString(16)}`); p++; }
        L.push(`deposit -b ${hex(p)} ${OPC_NOP.toString(16)}`);        // the instruction AFTER the probe
        L.push(`deposit -b ${hex(R_MCHK_H)} ${OPC_HALT.toString(16)}`);
        L.push(`deposit -b ${hex(R_MEMERR_H)} ${OPC_HALT.toString(16)}`);
        L.push(`deposit PSL ${hex(c.psl)}`);
        L.push(`deposit PC ${hex(R_CODE)}`);
        L.push("step 1");
        L.push(`examine -h ${NAMES.join(",")}`);
        L.push(`examine -h sysd bto`);
        L.push(`examine -h qba dser`);
        L.push(`examine -h qba mear`);
        let sp1 = (R_IS - FRAME_LEN) >>> 0;
        for (let i = 0; i < FRAME_LEN; i += 4) L.push(`examine -h ${hex(sp1 + i)}`);
        L.push("step 1");
        L.push(`examine -h PC,PSL`);
        let sp2 = (R_IS - IFRAME_LEN) >>> 0;
        for (let i = 0; i < IFRAME_LEN; i += 4) L.push(`examine -h ${hex(sp2 + i)}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/;
const WANT_STEP1 = NAMES.length + 1 + 1 + 1 + (FRAME_LEN / 4);   // regs/PC/PSL/SISR + BTO + DSER + MEAR + frame
const WANT_STEP2 = 2 + (IFRAME_LEN / 4);                          // PC,PSL + iframe
const WANT_PER_CASE = WANT_STEP1 + WANT_STEP2;

function runBatch(simh, cases, scratch)
{
    let script = buildScript(cases);
    let out = runSimh(simh, script, path.join(scratch, "cqmerrdiff-batch.ini"));
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
        let pc1 = vals[15], psl1 = vals[16], sisr1 = vals[17], bto = vals[18], dser = vals[19], mear = vals[20];
        let frame1 = vals.slice(21, 21 + FRAME_LEN / 4);
        let off = 21 + FRAME_LEN / 4;
        let pc2 = vals[off], psl2 = vals[off + 1];
        let iframe2 = vals.slice(off + 2, off + 2 + IFRAME_LEN / 4);
        results.set(idx, {reached: true, regs, pc1, psl1, sisr1, bto, dser, mear, frame1, pc2, psl2, iframe2});
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
    let spLo = (R_IS - 2 * FRAME_LEN) >>> 0;
    for (let i = 0; i < 2 * FRAME_LEN; i += 4) bus.setLong(spLo + i, 0);
    cpu.exc.scbb = R_SCBB;
    bus.setLong(R_SCBB + SCB.MCHK, R_MCHK_H);
    bus.setLong(R_SCBB + SCB.MEMERR, (R_MEMERR_H | 1) >>> 0);
    cpu.regs.set(c.regs);
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
    cpu.exc.sisr = c.sisr;
    let instr = buildInstr(c.fWrite, c.size, c.addr);
    let p = R_CODE;
    for (let i = 0; i < instr.length; i++) { bus.setByte(p, instr[i]); p++; }
    bus.setByte(p, OPC_NOP);
    bus.setByte(R_MCHK_H, OPC_HALT);
    bus.setByte(R_MEMERR_H, OPC_HALT);
    cpu.psl = c.psl | 0;
    cpu.regs[15] = R_CODE;

    let run = () => { try { cpu.stepCPU(1); } catch (e) { if (!(e instanceof VAXStop)) throw e; } };
    run();
    let sp1 = (R_IS - FRAME_LEN) >>> 0;
    let frame1 = [];
    for (let i = 0; i < FRAME_LEN; i += 4) frame1.push(bus.getLong(sp1 + i) | 0);
    let step1 = {
        regs: Int32Array.from(cpu.regs.slice(0, 15)), pc1: cpu.regs[15] | 0, psl1: cpu.psl,
        sisr1: cpu.exc.sisr, bto: cpu.exc.sscBto | 0, dser: cpu.exc.cqDser | 0, mear: cpu.exc.cqMear | 0, frame1
    };
    run();
    let sp2 = (R_IS - IFRAME_LEN) >>> 0;
    let iframe2 = [];
    for (let i = 0; i < IFRAME_LEN; i += 4) iframe2.push(bus.getLong(sp2 + i) | 0);
    return Object.assign(step1, {pc2: cpu.regs[15] | 0, psl2: cpu.psl, iframe2});
}

function compareCase(c, js, sr)
{
    let bad = [];
    let tag = `${c.fWrite ? "write" : "read"} size=${c.size} addr=0x${hex(c.addr)} (${c.range}) case#${c.index}`;
    if ((js.pc1 | 0) !== (sr.pc1 | 0)) bad.push(`${tag}: step1 PC js=${hex(js.pc1)} simh=${hex(sr.pc1)}`);
    if ((js.psl1 | 0) !== (sr.psl1 | 0)) bad.push(`${tag}: step1 PSL js=${hex(js.psl1)} simh=${hex(sr.psl1)}`);
    if ((js.bto | 0) !== (sr.bto | 0)) bad.push(`${tag}: BTO js=${hex(js.bto)} simh=${hex(sr.bto)} (a Qbus fault must NEVER touch this -- mchkdiff.js's territory)`);
    if ((js.dser | 0) !== (sr.dser | 0)) bad.push(`${tag}: DSER js=${hex(js.dser, 2)} simh=${hex(sr.dser, 2)}`);
    if ((js.mear | 0) !== (sr.mear | 0)) bad.push(`${tag}: MEAR js=${hex(js.mear, 4)} simh=${hex(sr.mear, 4)}`);
    for (let i = 0; i < js.frame1.length; i++) {
        if ((js.frame1[i] | 0) !== (sr.frame1[i] | 0)) {
            bad.push(`${tag}: frame1[${i * 4}] js=${hex(js.frame1[i])} simh=${hex(sr.frame1[i])}`);
        }
    }
    if ((js.pc2 | 0) !== (sr.pc2 | 0)) bad.push(`${tag}: step2 PC js=${hex(js.pc2)} simh=${hex(sr.pc2)}`);
    if ((js.psl2 | 0) !== (sr.psl2 | 0)) bad.push(`${tag}: step2 PSL js=${hex(js.psl2)} simh=${hex(sr.psl2)}`);
    for (let i = 0; i < js.iframe2.length; i++) {
        if ((js.iframe2[i] | 0) !== (sr.iframe2[i] | 0)) {
            bad.push(`${tag}: iframe2[${i * 4}] js=${hex(js.iframe2[i])} simh=${hex(sr.iframe2[i])}`);
        }
    }
    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * Legal-PSL generator -- same rule as mchkdiff.js's/excdiff.js's: kernel mode may carry any IPL    *
 * 0..0x1F; a non-kernel mode must have IPL 0.  PRV >= CUR.                                        *
 *                                                                                                   *
 * SISR IS DELIBERATELY NOT RANDOMIZED (unlike mchkdiff.js's noise, which this was first modelled    *
 * on) -- see "A DISCOVERED, OUT-OF-SCOPE BUG" in the file header: a pending software interrupt       *
 * eligible at the noise IPL preempts stepInstruction()'s FIRST dispatch, before this file's probe    *
 * ever executes, and if THAT dispatch's push then faults (an unrelated pre-existing risk once R14    *
 * is fully random), cpustate.js's inIE is left stuck set, corrupting this file's SECOND step in a    *
 * way single-step differentials (mchkdiff.js, excdiff.js) structurally cannot see.  That is a real,  *
 * filed, general dispatch-retry defect -- not this item's mechanism -- so it is avoided rather than  *
 * graded here, by construction: SISR stays 0.                                                        *
 * ------------------------------------------------------------------------------------------- */

function randomPSL(rnd)
{
    let cur = pick(rnd, [0, 0, 0, 1, 2, 3]);
    let ipl = (cur === 0) ? Math.floor(rnd() * 0x20) : 0;
    let prv = cur + Math.floor(rnd() * (4 - cur));
    let cc = Math.floor(rnd() * 16);
    return ((cur << 24) | (prv << 22) | (ipl << 16) | cc) | 0;
}

/* ------------------------------------------------------------------------------------------- *
 * Calibration -- IOPAGE is NOT uniformly unbacked (measured directly, see file header): the        *
 * stock microvax3900 autoconfigures DZ/RQ/TS/RL/XQ/TQ/the QBA's own self-registration/LPT at        *
 * fixed IOPAGE sub-windows this project implements none of.  THREE independent surprises, all       *
 * found by executing the randomized phase, not by inspection:                                       *
 *                                                                                                     *
 *   1. Device dispatch (vax_io.c's `idx = (pa & IOPAGEMASK) >> 1`) is WORD-SLOT granular, and an      *
 *      aligned LONG read is TWO Qbus word cycles -- an address whose first word-slot is backed and    *
 *      second is not can look "confirmed" for size=4 (the compound access still machine-checks, via   *
 *      the second word) while being genuinely BACKED for size=2 (touches only the first word-slot).   *
 *      Fix: calibrate with the EXACT size the graded case will use.                                    *
 *   2. Read and write dispatch tables are populated INDEPENDENTLY and are asymmetric: TS's window     *
 *      (no tape unit attached) answers a WORD READ at 0x20001550 without faulting, but a WORD WRITE    *
 *      to the SAME address sets DSER WITHOUT ever delivering a mem_err interrupt -- TS's OWN handler   *
 *      calls cq_merr() for reasons internal to its model, not the generic WriteQb-unbacked fallback.   *
 *      Fix: calibrate READ and WRITE independently, using delivery (not DSER alone) as the write        *
 *      signal.
 *   3. A GENUINE TRAP: an EARLIER revision of this calibration compared the ORACLE against THIS
 *      FILE'S OWN (JS) implementation via runBatch()/runCaseJS()/compareCase() -- reusing the exact
 *      grading machinery seemed appealing (one definition of "confirmed", no duplicated arithmetic to
 *      drift). It is EXACTLY THE MISTAKE pcjsvax-446's veracity re-dispatch already found and fixed
 *      once: calibrating "is this backed" against "does my CURRENT implementation happen to agree"
 *      turns the excluded set into an unexamined proxy for wherever the implementation is WRONG. Every
 *      one of this file's --selfcheck mutations passed uncaught the moment calibration compared
 *      against JS: a mutated cqMerr()/onBusFault()/writeL()/writeU() disagreed with the oracle on
 *      EVERY leaf, so calibration excluded everything as "backed", and the graded phase -- now
 *      testing nothing -- reported a clean PASS. Fix: calibration compares the ORACLE ONLY against a
 *      FORMULA this file derives from vax_io.c directly (expectedMear()), NEVER against JS. A mutation
 *      can break JS; it cannot change what the live SIMH process reports, so a leaf's confirmed/backed
 *      status cannot be perturbed by a bug in the code under test.
 *
 * CQM is calibrated too (rule 8: verify by executing rather than trust a one-time spot check) -- no
 * run has ever found a backed CQM address, but calibrating it costs nothing.
 * ------------------------------------------------------------------------------------------- */

const CAL_MARK = "@@CQMCAL@@";

/** expectedMear(addr) -- cq_merr()'s formula (vax_io.c:714), independent of any code under test. */
function expectedMear(addr) { return (addr >>> 9) & 0x1FFF; }

/**
 * expectedWriteOps(addr, size)
 *
 * How many 16-bit Qbus write cycles a KA655 issues for THIS reference, per vax_io.c's
 * WriteIO()/WriteIOU() (aligned vs. unaligned dispatch) -- an INDEPENDENT derivation from the
 * architecture, not a call into mmu.js's writeL()/writeU() (see calibrate()'s "GENUINE TRAP" note:
 * calibration must never depend on the code under test).  This is what decides whether DSER's LST
 * bit is expected on a purely-uniform-unbacked write (>=2 ops) or not (1 op) -- and, just as
 * importantly, a MISMATCH between this formula's prediction and the oracle's actual DSER is exactly
 * the signal that some sub-word of a COMPOUND unaligned/long write straddled into a real device
 * (DZ/RQ/TS/RL/XQ/TQ/LPT) that this calibration's single-leaf-address check would otherwise miss --
 * see the "0x20001942 straddles TQ" case this exact check was added for.
 *
 * @param {number} addr
 * @param {number} size 1, 2 or 4
 * @returns {number}
 */
function expectedWriteOps(addr, size)
{
    if (size === 1) return 1;                                   // WriteIO's L_BYTE: always 1 op
    if (size === 2) return (addr & 1) ? 2 : 1;                   // aligned: 1 op; odd: 2 byte ops (WriteIOU)
    let bo = addr & 3;
    if (bo === 0) return 2;                                       // WriteIO's L_LONG: always 2 word ops
    let opsFor = (a, lnt) => (lnt === 3) ? 2 : (lnt === 2) ? ((a & 1) ? 2 : 1) : 1;
    let addr2 = (addr + 4) & ~3;
    return opsFor(addr, 4 - bo) + opsFor(addr2, bo);
}

function leafKey(l) { return `${l.addr}:${l.size}`; }

/**
 * buildCalibrationScript(leaves)
 *
 * READ probe: one step, examine PC/DSER/MEAR.  Confirmed iff PC==R_MCHK_H AND DSER==0x80 (MNX only
 * -- a read never accumulates LST, it aborts after the first Qbus cycle) AND MEAR==expectedMear().
 *
 * WRITE probe: two steps (probe, then the delivery check), examine PC after each and DSER/MEAR
 * after the first.  Confirmed iff DSER's MNX bit (0x80) is set AND MEAR==expectedMear() AND step 2's
 * PC reaches R_MEMERR_H+1 (mem_err actually delivered -- see the file's own delivery-shape doc
 * comment in verifyDeferredDelivery()).  All three together are what rules out TS-shaped devices
 * that set DSER without being the genuine WriteQb-unbacked fallback.
 *
 * @param {Array<{addr:number, size:number}>} leaves
 * @returns {string}
 */
function buildCalibrationScript(leaves)
{
    let L = ["set cpu " + (MEMSIZE >> 20) + "m", "set cpu simhalt", `deposit 0:${(CQBIC_MAP_LEN - 1).toString(16).toUpperCase()} 0`];
    for (let i = 0; i < leaves.length; i++) {
        let leaf = leaves[i];
        for (let fWrite of [false, true]) {
            L.push(`echo ${CAL_MARK}${i}:${fWrite ? "w" : "r"}`);
            L.push("reset all");
            L.push(`deposit SCBB ${hex(R_SCBB)}`);
            L.push(`deposit -l ${hex(R_SCBB + SCB.MCHK)} ${hex(R_MCHK_H)}`);
            L.push(`deposit -l ${hex(R_SCBB + SCB.MEMERR)} ${hex(R_MEMERR_H | 1)}`);
            L.push(`deposit KSP ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`);
            let instr = buildInstr(fWrite, leaf.size, leaf.addr);
            let p = R_CODE;
            for (let b of instr) { L.push(`deposit -b ${hex(p)} ${b.toString(16)}`); p++; }
            L.push(`deposit -b ${hex(p)} ${OPC_NOP.toString(16)}`);      // step2's target for writes
            L.push(`deposit -b ${hex(R_MCHK_H)} ${OPC_HALT.toString(16)}`);
            L.push(`deposit -b ${hex(R_MEMERR_H)} ${OPC_HALT.toString(16)}`);
            L.push("deposit PSL 0", `deposit PC ${hex(R_CODE)}`);
            L.push("step 1");
            L.push("examine -h PC");
            L.push("examine -h qba dser");
            L.push("examine -h qba mear");
            L.push("step 1");
            L.push("examine -h PC");
        }
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

/**
 * calibrate(simh, scratch, leaves)
 *
 * @returns {Object} {confirmedRead, confirmedWrite, backed, notReached} -- confirmedRead/
 *          confirmedWrite are Sets of leafKey(); backed/notReached are arrays of
 *          {addr, size, fWrite, ...} for reporting by name.
 */
function calibrate(simh, scratch, leaves)
{
    let out = runSimh(simh, buildCalibrationScript(leaves), path.join(scratch, "cqmerrdiff-cal.ini"));
    let lines = out.split("\n");
    let results = new Map();     // "idx:r"/"idx:w" -> {pc1, dser, mear, pc2}
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(new RegExp(CAL_MARK + "(\\d+):([rw])"));
        if (!m) { i++; continue; }
        let key = `${m[1]}:${m[2]}`;
        i++;
        let vals = [];
        while (i < lines.length && vals.length < 4) {
            if (lines[i].indexOf(CAL_MARK) >= 0) break;
            let vm = lines[i].match(VALUE_RE);
            if (vm) vals.push(parseInt(vm[2], 16) | 0);
            i++;
        }
        if (vals.length === 4) results.set(key, {pc1: vals[0], dser: vals[1], mear: vals[2], pc2: vals[3]});
    }
    let deliveredPC = (R_MEMERR_H + 1) >>> 0;
    let confirmedRead = new Set(), confirmedWrite = new Set(), backed = [], notReached = [];
    for (let i = 0; i < leaves.length; i++) {
        let leaf = leaves[i];
        let mear = expectedMear(leaf.addr);

        let r = results.get(`${i}:r`);
        if (!r) {
            notReached.push(Object.assign({fWrite: false}, leaf));
        } else if ((r.pc1 >>> 0) === R_MCHK_H && (r.dser >>> 0) === 0x80 && (r.mear >>> 0) === mear) {
            confirmedRead.add(leafKey(leaf));
        } else {
            backed.push(Object.assign({fWrite: false, reason:
                `PC=0x${hex(r.pc1)} DSER=0x${hex(r.dser, 2)} MEAR=0x${hex(r.mear, 4)} (expected PC=0x${hex(R_MCHK_H)} DSER=0x80 MEAR=0x${hex(mear, 4)})`
            }, leaf));
        }

        let w = results.get(`${i}:w`);
        let expectedDser = 0x80 | (expectedWriteOps(leaf.addr, leaf.size) >= 2 ? 0x08 : 0);
        if (!w) {
            notReached.push(Object.assign({fWrite: true}, leaf));
        } else if ((w.dser >>> 0) === expectedDser && (w.mear >>> 0) === mear && (w.pc2 >>> 0) === deliveredPC) {
            confirmedWrite.add(leafKey(leaf));
        } else {
            backed.push(Object.assign({fWrite: true, reason:
                `DSER=0x${hex(w.dser, 2)} (expected 0x${hex(expectedDser, 2)}) MEAR=0x${hex(w.mear, 4)} delivered=${(w.pc2 >>> 0) === deliveredPC} ` +
                `(expected DSER<MNX> set, MEAR=0x${hex(mear, 4)}, delivered=true)`
            }, leaf));
        }
    }
    return {confirmedRead, confirmedWrite, backed, notReached};
}

/* ------------------------------------------------------------------------------------------- *
 * Leaf generation -- (range, addr, size, unaligned) combos, calibrated BEFORE a direction is       *
 * ever decided (see calibrate()'s doc comment for why size must be part of the calibration key).   *
 * The two independent phases (see file header) differ only in how addr/size are chosen.            *
 * ------------------------------------------------------------------------------------------- */

function enumeratedLeaves()
{
    let leaves = [];
    for (let e of ENUM_ADDRS) {
        for (let size of [1, 2, 4]) leaves.push({range: e.range, addr: e.addr, size, unaligned: false});
        for (let size of [2, 4]) {
            for (let off of UNALIGNED_OFFSETS[size]) {
                leaves.push({range: e.range, addr: (e.addr + off) >>> 0, size, unaligned: true});
            }
        }
    }
    return leaves;
}

function randomLeaves(rnd, n)
{
    let leaves = [];
    for (let k = 0; k < n; k++) {
        let r = pick(rnd, RANGES);
        /* Uniformly random BYTE offset across the WHOLE range -- not a boundary/stride point --
           is what makes this phase structurally independent of the enumerated one (see file
           header). */
        let off = Math.floor(rnd() * (r.len - 4));
        let addr = (r.base + off) >>> 0;
        let size = pick(rnd, [1, 2, 4]);
        leaves.push({range: r.name, addr, size, unaligned: (addr % size) !== 0});
    }
    return leaves;
}

/**
 * leavesToCases(startIndex, leaves, fWriteFor)
 *
 * @param {number} startIndex
 * @param {Array<Object>} leaves CONFIRMED leaves only (backed ones already excluded by the caller)
 * @param {function(Object): Array<boolean>} fWriteFor directions to generate per leaf
 * @returns {{cases: Array<Case>, nextIndex: number}}
 */
function leavesToCases(startIndex, leaves, fWriteFor)
{
    let cases = [];
    let index = startIndex;
    for (let leaf of leaves) {
        for (let fWrite of fWriteFor(leaf)) {
            let c = new Case(index++, fWrite, leaf.size, leaf.addr, leaf.range);
            c.unaligned = leaf.unaligned;
            cases.push(c);
        }
    }
    return {cases, nextIndex: index};
}

/* ------------------------------------------------------------------------------------------- *
 * verifyDeferredDelivery -- IPL masking, and eventual delivery once IPL drops.  Deterministic,     *
 * not part of either phase's pool (see file header).                                              *
 * ------------------------------------------------------------------------------------------- */

function buildDeliveryScript(addr)
{
    let instr = buildInstr(true, 4, addr);
    let L = ["set cpu 16m", "set cpu simhalt", "reset all",
        `deposit 0:${(CQBIC_MAP_LEN - 1).toString(16).toUpperCase()} 0`, "deposit sysd bto 0",
        `deposit SCBB ${hex(R_SCBB)}`, `deposit -l ${hex(R_SCBB + SCB.MEMERR)} ${hex(R_MEMERR_H | 1)}`,
        `deposit R14 ${hex(R_KSP)}`, `deposit KSP ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`];
    let p = R_CODE;
    for (let i = 0; i < instr.length; i++) { L.push(`deposit -b ${hex(p)} ${instr[i].toString(16)}`); p++; }
    for (let k = 0; k < 3; k++) { L.push(`deposit -b ${hex(p)} ${OPC_NOP.toString(16)}`); p++; }
    L.push(`deposit -b ${hex(R_MEMERR_H)} ${OPC_HALT.toString(16)}`);
    L.push(`deposit PSL ${hex(0x1F << 16)}`, `deposit PC ${hex(R_CODE)}`);
    /* step through the probe plus THREE NOPs while IPL == 0x1F (masked -- must NOT dispatch) */
    for (let k = 0; k < 4; k++) { L.push("step 1"); L.push("examine -h PC,PSL"); }
    L.push("deposit PSL 0");                 // lower IPL -- the pending mem_err must now deliver
    L.push("step 1");
    L.push("examine -h PC,PSL");
    L.push("quit");
    return L.join("\n") + "\n";
}

function runDeliveryJS(addr)
{
    let m = makeMachine();
    let {bus, cpu} = m;
    cpu.exc.scbb = R_SCBB;
    bus.setLong(R_SCBB + SCB.MEMERR, (R_MEMERR_H | 1) >>> 0);
    cpu.exc.stk[0] = R_KSP; cpu.exc.stk[4] = R_IS; cpu.regs[14] = R_KSP;
    let instr = buildInstr(true, 4, addr);
    let p = R_CODE;
    for (let i = 0; i < instr.length; i++) { bus.setByte(p, instr[i]); p++; }
    for (let k = 0; k < 3; k++) { bus.setByte(p, OPC_NOP); p++; }
    bus.setByte(R_MEMERR_H, OPC_HALT);
    cpu.psl = 0x1F << 16;
    cpu.regs[15] = R_CODE;
    let snapshots = [];
    let run = () => { try { cpu.stepCPU(1); } catch (e) { if (!(e instanceof VAXStop)) throw e; } };
    for (let k = 0; k < 4; k++) { run(); snapshots.push({pc: cpu.regs[15] | 0, psl: cpu.psl | 0}); }
    cpu.psl = 0;
    run();
    snapshots.push({pc: cpu.regs[15] | 0, psl: cpu.psl | 0});
    return snapshots;
}

function verifyDeferredDelivery(simh, scratch, addr)
{
    let out = runSimh(simh, buildDeliveryScript(addr), path.join(scratch, "cqmerrdiff-delivery.ini"));
    let vals = [];
    for (let line of out.split("\n")) { let vm = line.match(VALUE_RE); if (vm) vals.push(parseInt(vm[2], 16) | 0); }
    if (vals.length < 10) throw new Error(`cqmerrdiff: delivery probe produced ${vals.length}/10 values; SIMH said:\n${out}`);
    let srSnaps = [];
    for (let i = 0; i < 5; i++) srSnaps.push({pc: vals[i * 2], psl: vals[i * 2 + 1]});
    let jsSnaps = runDeliveryJS(addr);
    let bad = [];
    let tag = `deferred-delivery addr=0x${hex(addr)}`;
    for (let i = 0; i < 5; i++) {
        if ((jsSnaps[i].pc | 0) !== (srSnaps[i].pc | 0)) bad.push(`${tag}: step${i + 1} PC js=${hex(jsSnaps[i].pc)} simh=${hex(srSnaps[i].pc)}`);
        if ((jsSnaps[i].psl | 0) !== (srSnaps[i].psl | 0)) bad.push(`${tag}: step${i + 1} PSL js=${hex(jsSnaps[i].psl)} simh=${hex(srSnaps[i].psl)}`);
    }
    /* Assert the SHAPE this check exists to prove, not just bit-equality with the oracle (which a
       symmetrically-broken implementation could still satisfy by accident): masked for the first
       three steps (PC keeps advancing past R_CODE, never reaching R_MEMERR_H), delivered on the
       fifth.  A delivered INTERRUPT is caught at the TOP of stepInstruction()'s dispatch loop
       (unlike a mid-instruction MCHK, which consumes the whole step on the dispatch alone) -- the
       loop `continue`s and, per exc.js's own accounting doc comment, still fetches and executes
       ONE instruction before the step ends.  R_MEMERR_H holds a HALT, so the observable PC after
       the step that delivers is R_MEMERR_H + 1 (past the 1-byte HALT opcode fetch), exactly the
       same convention this file's own read-side R_MCHK_H checks do NOT need (a bus fault during
       instruction EXECUTION throws before any further fetch happens in that same step). */
    let delivered = (R_MEMERR_H + 1) >>> 0;
    for (let i = 0; i < 3; i++) {
        if ((srSnaps[i].pc >>> 0) === delivered) bad.push(`${tag}: step${i + 1}: ORACLE itself dispatched while IPL should have masked it -- test construction is wrong, not the implementation`);
        if ((jsSnaps[i].pc >>> 0) === delivered) bad.push(`${tag}: step${i + 1}: JS dispatched the deferred mem_err while PSL<IPL>=0x1F should have masked it`);
    }
    if ((srSnaps[4].pc >>> 0) !== delivered) bad.push(`${tag}: step5: ORACLE did not deliver after IPL was lowered -- test construction is wrong`);
    if ((jsSnaps[4].pc >>> 0) !== delivered) bad.push(`${tag}: step5: JS did not deliver the deferred mem_err once PSL<IPL> was lowered below 0x1D`);
    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * Calibration ASSERTIONS -- the excluded set is REPORTED by name above; that alone is not          *
 * enough, and a veracity pass measured why: nExcludedBacked lived only in the stats object and a    *
 * printf, nothing require()d it, so a targeted one-line error in expectedWriteOps() (the ORACLE-    *
 * side formula, not the code under test -- see calibrate()'s "GENUINE TRAP" note) grew the           *
 * excluded set 11x (14 -> 152 -> 307), silently dropped over a hundred write probes -- this item's   *
 * headline observable -- and the run still reported PASS with comfortable floor margin, because      *
 * the coverage floors below measure the GRADED pool, not the exclusions, and a shrunk pool can        *
 * still clear them.  Two committed checks close this, mirroring mchkdiff.js's EXPECTED_CALIBRATION:  *
 *                                                                                                     *
 *   ENUMERATED leaves are fully deterministic -- candidatesFor() never varies with --seed -- so       *
 *   their confirmed/backed counts, per range and per direction, are committed EXACTLY below and       *
 *   asserted in BOTH directions: too many exclusions (a formula regression) and too few (the           *
 *   formula becoming too permissive, silently grading a leaf it should have excluded) both fail.       *
 *   RANDOMIZED leaves vary with --seed and --cases by design (see the file header), so an exact        *
 *   count is not meaningful; measured across 8 seeds at --cases 300 the excluded RATE stayed in         *
 *   0.39%-1.17%.  A ceiling with 4x that slack (5%) comfortably passes every measured baseline and      *
 *   fails both injected-bug rates from the veracity pass (13.9% and 28%).                                *
 * ------------------------------------------------------------------------------------------- */

const EXPECTED_ENUM_CALIBRATION = {
    IOPAGE: {confirmed: {read: 80, write: 76}, backed: {read: 0, write: 4}},
    CQM:    {confirmed: {read: 80, write: 76}, backed: {read: 0, write: 4}}
};

/**
 * assertEnumCalibration(enumLeaves, cal)
 *
 * @param {Array<Object>} enumLeaves
 * @param {Object} cal as returned by calibrate()
 * @returns {Array.<string>} mismatches (empty means calibration matches the committed numbers)
 */
function assertEnumCalibration(enumLeaves, cal)
{
    let counts = {};
    for (let r of RANGES) counts[r.name] = {confirmed: {read: 0, write: 0}, backed: {read: 0, write: 0}};
    for (let l of enumLeaves) {
        let k = leafKey(l);
        let c = counts[l.range];
        if (cal.confirmedRead.has(k)) c.confirmed.read++; else c.backed.read++;
        if (cal.confirmedWrite.has(k)) c.confirmed.write++; else c.backed.write++;
    }
    let bad = [];
    for (let r of RANGES) {
        let exp = EXPECTED_ENUM_CALIBRATION[r.name];
        if (!exp) { bad.push(`CALIBRATION: range "${r.name}" has no EXPECTED_ENUM_CALIBRATION entry`); continue; }
        for (let dir of ["read", "write"]) {
            let got = counts[r.name];
            if (got.confirmed[dir] !== exp.confirmed[dir]) {
                bad.push(`CALIBRATION: ${r.name} enumerated confirmed.${dir} = ${got.confirmed[dir]}, expected ${exp.confirmed[dir]}`);
            }
            if (got.backed[dir] !== exp.backed[dir]) {
                bad.push(`CALIBRATION: ${r.name} enumerated backed.${dir} = ${got.backed[dir]}, expected ${exp.backed[dir]}`);
            }
        }
    }
    return bad;
}

/* Measured 0.39%-1.17% across seeds {0xC0FFEE,1,2,3,4,5,42,999999} at --cases 300; the veracity
   pass's two injected-bug scenarios measured 13.9% and 28% -- 5% sits with slack above every real
   baseline and well below both injected failures. */
const MAX_RANDOM_BACKED_RATE = 0.05;

/**
 * assertRandomCalibration(randLeavesAll, cal, enumKeySet)
 *
 * @param {Array<Object>} randLeavesAll
 * @param {Object} cal as returned by calibrate()
 * @param {Set<string>} enumKeySet leafKey()s already counted by assertEnumCalibration -- excludes
 *        them from the random-pool rate so the two assertions never double-count the same leaf
 * @returns {Array.<string>}
 */
function assertRandomCalibration(randLeavesAll, cal, enumKeySet)
{
    let randomBacked = cal.backed.filter((b) => !enumKeySet.has(leafKey(b)));
    let totalPairs = randLeavesAll.length * 2;
    if (!totalPairs) return [];
    let rate = randomBacked.length / totalPairs;
    if (rate > MAX_RANDOM_BACKED_RATE) {
        return [`CALIBRATION: randomized-pool excluded rate ${(rate * 100).toFixed(2)}% ` +
            `(${randomBacked.length}/${totalPairs}) exceeds the ${(MAX_RANDOM_BACKED_RATE * 100).toFixed(0)}% ` +
            `ceiling -- a formula regression (expectedMear/expectedWriteOps) or an implementation bug ` +
            `may be silently draining the graded pool`];
    }
    return [];
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors -- FAIL the run, do NOT scale down with case count                              *
 * ------------------------------------------------------------------------------------------- */

const MIN_CASES_FLOOR = 150;
const MIN_TOTAL_OPS = 300;
const MIN_PER_DIRECTION = 100;
const MIN_PER_SIZE = 60;
const MIN_PER_RANGE = 80;
const MIN_UNALIGNED = 20;
const MIN_DISTINCT_ADDRESSES = 100;      // enumerated pool alone already clears this; a floor, not a target
const MIN_WRITE_MASKED_BY_IPL = 3;        // randomized write cases whose starting IPL >= IPL_MEMERR (0x1D)
const IPL_MEMERR = 0x1D;

/* ------------------------------------------------------------------------------------------- *
 * Self-check mutations                                                                          *
 * ------------------------------------------------------------------------------------------- */

const MUTATED_METHODS = [
    [VAXExc, "cqMerr"], [CPUStateVAX, "onBusFault"], [MMUVAX, "writeL"], [MMUVAX, "writeU"]
];
function snapshotProto()
{
    return MUTATED_METHODS.map(([cls, name]) => [cls, name, cls.prototype[name]]);
}
function restoreProto(save) { for (let [cls, name, fn] of save) cls.prototype[name] = fn; }

const MUTATIONS = [
    {name: "write faults synchronously when it must not (reverts to the pre-pcjsvax-d22 behavior)", apply() {
        CPUStateVAX.prototype.onBusFault = function(addr, access) {
            let a = addr >>> 0;
            let fWrite = access === VAX.ACCESS.WRITE;
            let delta = (this.regs[15] - this.exc.faultPC) | 0;
            if (VAX.isQbusAddr(a)) {
                this.exc.cqMerr(a);
                throw new VAXFault(-SCB.MCHK, fWrite ? 0x82 : 0x80, delta);
            }
            let p1 = this.exc.busTimeout(fWrite);
            throw new VAXFault(-SCB.MCHK, p1, delta);
        };
    }},
    {name: "DSER/MEAR not updated (cqMerr is a no-op)", apply() {
        VAXExc.prototype.cqMerr = function() { /* nothing */ };
    }},
    {name: "LST bit never set (DSER always shows a fresh single-op error)", apply() {
        VAXExc.prototype.cqMerr = function(addr) {
            this.cqDser = 0x80;
            this.cqMear = ((addr >>> 9) & 0x1FFF) | 0;
        };
    }},
    {name: "deferred write never sets memErr (mem_err lost entirely)", apply() {
        let origOBF = CPUStateVAX.prototype.onBusFault;
        CPUStateVAX.prototype.onBusFault = function(addr, access) {
            let a = addr >>> 0;
            if (VAX.isQbusAddr(a) && access === VAX.ACCESS.WRITE) {
                this.exc.cqMerr(a);
                return;               // memErr never set -- the interrupt never arrives
            }
            return origOBF.call(this, addr, access);
        };
    }},
    {name: "aligned long Qbus write does only one Qbus cycle (LST under-counted)", apply() {
        MMUVAX.prototype.writeL = function(pa, val) {
            this.bus.setLong(pa & ~0x03, val);
        };
    }},
    {name: "unaligned Qbus write still read-modify-writes (spurious synchronous READ machine check)", apply() {
        MMUVAX.prototype.writeU = function(pa, val, lnt) {
            const INSERT = [0, 0xFF, 0xFFFF, 0xFFFFFF, 0xFFFFFFFF];
            let addr = pa & ~0x03;
            let sc = (pa & 3) << 3;
            let mask = INSERT[lnt] << sc;
            this.bus.setLong(addr, (this.bus.getLong(addr) & ~mask) | ((val & INSERT[lnt]) << sc));
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
 * The graded run                                                                                  *
 * ------------------------------------------------------------------------------------------- */

function runPhase(simh, scratch, opts, label)
{
    let rnd = mulberry32(opts.seed || 1);
    let m = makeMachine();

    /* Calibrate BEFORE building any case: ENUM_ADDRS (should all be confirmed; verified, not
       assumed -- rule 8) plus an oversampled random draw (some WILL land in a backed device
       window, at ~1% of IOPAGE's span -- oversampling absorbs the loss without silently
       shrinking the requested case count). */
    let enumLeaves = enumeratedLeaves();
    let randLeavesAll = randomLeaves(rnd, Math.ceil(opts.cases * 1.25) + 10);
    let cal = calibrate(simh, scratch, enumLeaves.concat(randLeavesAll));

    let dirsFor = (l) => {
        let k = leafKey(l);
        let dirs = [];
        if (cal.confirmedRead.has(k)) dirs.push(false);
        if (cal.confirmedWrite.has(k)) dirs.push(true);
        return dirs;
    };

    let calibrationReport = [];
    let enumKeySet = new Set(enumLeaves.map(leafKey));
    let backedFor = new Map();      // leafKey -> [{fWrite, reason}]
    for (let b of cal.backed) {
        let k = leafKey(b);
        if (!backedFor.has(k)) backedFor.set(k, []);
        backedFor.get(k).push(b);
    }
    for (let l of enumLeaves) {
        let dirs = dirsFor(l);
        if (dirs.length < 2) {
            let entries = backedFor.get(leafKey(l)) || [];
            for (let e of entries) {
                calibrationReport.push(`CALIBRATION: enumerated leaf ${l.range}@0x${hex(l.addr)} size=${l.size} ` +
                    `is BACKED for ${e.fWrite ? "write" : "read"} (${e.reason || "a real device answers, or a " +
                    "range-boundary straddle -- see calibrate()'s doc comment"}) -- excluded, not a ` +
                    `hand-enumeration gap (mchkdiff.js's own IOPAGE candidates dodge every device window by ` +
                    `coincidence -- see file header)`);
            }
        }
    }
    for (let b of cal.backed) {
        if (!enumKeySet.has(leafKey(b))) {
            calibrationReport.push(`CALIBRATION: random leaf 0x${hex(b.addr)} size=${b.size} is BACKED for ` +
                `${b.fWrite ? "write" : "read"} (${b.reason || "a real device answers"}) -- excluded`);
        }
    }
    for (let l of cal.notReached) {
        calibrationReport.push(`CALIBRATION: leaf 0x${hex(l.addr)} size=${l.size} (${l.fWrite ? "write" : "read"}) did not reach comparison`);
    }

    /* The excluded set above is DISCLOSED (reported by name); these two calls are what ASSERTS it,
       so a formula regression that inflates or hollows out the excluded set fails the run instead
       of the excluded set quietly absorbing the loss -- see the doc comment above these functions. */
    let calMismatch = assertEnumCalibration(enumLeaves, cal).concat(assertRandomCalibration(randLeavesAll, cal, enumKeySet));

    let {cases: enumCases, nextIndex} = leavesToCases(0, enumLeaves, dirsFor);
    let randCases = [];
    {
        let idx = nextIndex;
        let kept = 0;
        for (let leaf of randLeavesAll) {
            if (kept >= opts.cases) break;
            let dirs = dirsFor(leaf);
            if (!dirs.length) continue;                    // backed both ways -- excluded, already reported
            let fWrite = dirs.length === 2 ? (rnd() < 0.5) : dirs[0];
            let c = new Case(idx++, fWrite, leaf.size, leaf.addr, leaf.range);
            c.unaligned = leaf.unaligned;
            c.psl = randomPSL(rnd);
            for (let g = 0; g < 15; g++) c.regs[g] = (Math.floor(rnd() * 0x100000000)) | 0;
            randCases.push(c);
            kept++;
        }
    }
    let cases = enumCases.concat(randCases);

    let stats = {
        nOps: 0, byDir: {write: 0, read: 0}, bySize: {1: 0, 2: 0, 4: 0}, byRange: {IOPAGE: 0, CQM: 0},
        nUnaligned: 0, nWriteMaskedByIPL: 0, nExcludedBacked: cal.backed.length
    };
    let addrsSeen = new Set();
    let failures = calMismatch.slice();
    let notReached = [];
    const BATCH = 60;
    for (let start = 0; start < cases.length; start += BATCH) {
        let batch = cases.slice(start, start + BATCH);
        let sr = runBatch(simh, batch, scratch);
        for (let c of batch) {
            let res = sr.get(c.index);
            if (!res || !res.reached) { notReached.push(`${label} case ${c.index} (SIMH produced ${res ? res.got : 0}/${res ? res.want : "?"} values)`); continue; }
            let js = runCaseJS(m, c);
            let bad = compareCase(c, js, res);
            if (bad.length) failures.push(...bad);
            stats.nOps++;
            stats.byDir[c.fWrite ? "write" : "read"]++;
            stats.bySize[c.size]++;
            stats.byRange[c.range]++;
            if (c.unaligned) stats.nUnaligned++;
            addrsSeen.add(c.addr);
            if (c.fWrite && ((c.psl >>> 16) & 0x1F) >= IPL_MEMERR) stats.nWriteMaskedByIPL++;
        }
    }
    return {failures, notReached, stats, distinctAddresses: addrsSeen.size, calibrationReport};
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                            *
 * ------------------------------------------------------------------------------------------- */

function parseCases(raw)
{
    if (raw === null || raw === undefined) return 300;
    let n = Number(raw);
    if (!Number.isFinite(n)) throw new Error(`cqmerrdiff.js: --cases "${raw}" is not a number`);
    return n;
}

function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, dflt) => { let i = argv.indexOf(name); return i >= 0 ? argv[i + 1] : dflt; };
    let simh = findSimh(getArg("--simh", null));
    let seed = +getArg("--seed", 0xC0FFEE);
    let nCases = parseCases(getArg("--cases", null));
    let fSelfCheck = argv.indexOf("--selfcheck") >= 0;
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "vax-cqmerrdiff-"));

    console.log("VAX Qbus/CQBIC unbacked-access differential test (pcjsvax-d22)");
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
        let {failures, notReached, stats, distinctAddresses, calibrationReport} =
            runPhase(simh, scratch, {seed, cases: nCases}, "main");
        console.log("  elapsed: %ds", ((Date.now() - t0) / 1000).toFixed(1));

        console.log("\nCalibration (IOPAGE is NOT uniformly unbacked -- some addresses are real, " +
            "autoconfigured Qbus devices this machine does not implement; see file header):");
        if (calibrationReport.length) {
            for (let c of calibrationReport) console.log("  " + c);
        } else {
            console.log("  no exclusions this run");
        }

        console.log("\nComparisons: ops=%d write=%d read=%d byte=%d word=%d long=%d unaligned=%d " +
            "iopage=%d cqm=%d write-masked-by-ipl=%d excluded-backed=%d",
            stats.nOps, stats.byDir.write, stats.byDir.read, stats.bySize[1], stats.bySize[2], stats.bySize[4],
            stats.nUnaligned, stats.byRange.IOPAGE, stats.byRange.CQM, stats.nWriteMaskedByIPL, stats.nExcludedBacked);

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
        require(stats.byRange.IOPAGE >= MIN_PER_RANGE, `too few IOPAGE probes (${stats.byRange.IOPAGE})`);
        require(stats.byRange.CQM >= MIN_PER_RANGE, `too few CQM probes (${stats.byRange.CQM})`);
        require(stats.nUnaligned >= MIN_UNALIGNED, `too few unaligned probes (${stats.nUnaligned})`);
        require(distinctAddresses >= MIN_DISTINCT_ADDRESSES,
            `too few distinct addresses actually used for case generation (${distinctAddresses} of a floor of ${MIN_DISTINCT_ADDRESSES})`);
        require(stats.nWriteMaskedByIPL >= MIN_WRITE_MASKED_BY_IPL,
            `too few randomized write cases whose starting IPL >= 0x1D (${stats.nWriteMaskedByIPL}) -- ` +
            `the randomized phase's PSL noise did not exercise delivery masking`);

        console.log("\nDeferred-delivery check (masked while IPL>=0x1D, delivered once lowered):");
        let deliveryAddrs = [VAX.PHYSMEM.IOPAGE_BASE + 0x104, VAX.PHYSMEM.CQM_BASE + 0x2000];
        for (let addr of deliveryAddrs) {
            let bad = verifyDeferredDelivery(simh, scratch, addr);
            console.log(`  addr=0x${hex(addr)}: ${bad.length ? "FAILED" : "PASS"}`);
            for (let f of bad) errors.push(f);
        }

        if (fSelfCheck) {
            console.log("\nSelf-check: the differential must FAIL when the mechanism is deliberately broken.");
            let results = selfcheck(simh, scratch, {seed});
            for (let r of results) if (!r.caught) errors.push(`SELFCHECK: mutation '${r.name}' was not detected`);
        }
    } finally {
        if (!process.env["VAX_CQMERRDIFF_KEEP"]) fs.rmSync(scratch, {recursive: true, force: true});
    }

    if (errors.length) {
        console.log("\nFAILED (%d):", errors.length);
        for (let e of errors) console.log("  " + e);
        process.exit(1);
    }
    console.log("\nPASS: unbacked Qbus/CQBIC access matches real SIMH -- reads machine-check, writes " +
        "defer through mem_err, DSER/MEAR match, and delivery respects IPL masking.");
}

if (process.argv[1] && path.resolve(process.argv[1]) == path.resolve(fileURLToPath(import.meta.url))) {
    main();
}

export {
    RANGES, ENUM_ADDRS, makeMachine, runCaseJS, calibrate, enumeratedLeaves, randomLeaves, leafKey,
    expectedMear, expectedWriteOps
};
