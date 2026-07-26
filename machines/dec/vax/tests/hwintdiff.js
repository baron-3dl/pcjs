/**
 * @fileoverview Differential test: device-raised hardware interrupts (exc.js's deviceVector()
 *               seam) vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-7ad added the missing piece of exc.js's hardware-interrupt seam: an installer
 * (addInterruptSource), the mutators a device uses to assert/withdraw a request (raiseInterrupt/
 * clearInterrupt), and get_vector()'s device-acknowledge scan (deviceVector()).  evalInt() and the
 * `if (vec)` dispatch decision were ALREADY correct and are already graded by excdiff.js/intdiff.js;
 * this file does not re-grade them, and its selfcheck mutations are careful to touch only the code
 * this item added (or, for the two arbitration-shaped mutations, a --selfcheck-only monkeypatch of
 * evalInt exactly like excdiff.js's own selfcheck already does -- never the shipped evalInt itself).
 *
 * NO DEVICE IS IMPLEMENTED HERE.  A "prime" function plays the part of a device: it calls
 * addInterruptSource()/raiseInterrupt() directly, the way a real device's DIB/SET_INT would.  The
 * DISPATCH that follows is graded against REAL SIMH in every case where SIMH has a real device to
 * ground it against:
 *
 *   FIXED VECTOR    TTI (0xF8), TTO (0xFC), CSI (0xF0), CSO (0xF4) at hardware IPL 0x14, and CLK
 *                   (SCB_INTTIM, 0xC0) at IPL 0x16, are REAL SIMH devices (vax_stddev.c /
 *                   vax_sysdev.c).  Their per-device `INT` register (vax_stddev.c:144 etc.) is a
 *                   plain FLDATA bit -- unlike the QBA's aggregate IPL14..IPL17 view, which is
 *                   REG_RO -- so `deposit TTI INT 1` asserts int_req[] through the SAME storage a
 *                   real device write would, and the ensuing `step 1` runs SIMH's genuine
 *                   eval_int()/get_vector()/intexc().
 *   DYNAMIC VECTOR  The SSC timers T0/T1 (vax_sysdev.c) are the ONLY dynamic-vector hardware
 *                   interrupt source in this machine (TMR_VEC_MASK 0x3FC, tmr0_inta/tmr1_inta) --
 *                   the reason this seam has to support a vector supplied by a device callback at
 *                   all, not just a per-level constant.  T0/T1's request bit has no named REG (only
 *                   the QBA aggregate view sees it, and that is RO), so it cannot be deposited
 *                   directly; instead this file emits THREE REAL MOVL instructions the SIMH CPU
 *                   executes for real -- writing T0NI (next interval) = -1, T0VEC (TIVEC0) = a
 *                   chosen vector, then T0CSR = XFR|IE|SGL, which vax_sysdev.c's tmr_csr_wr()
 *                   synchronously single-steps the counter, overflows it, and calls SET_INT(TMR0)
 *                   with NO event-queue wait -- fully deterministic.  The JS side does not execute
 *                   those MOVLs (there is no SSC device model to react to the physical write, and
 *                   building one is explicitly out of scope -- see exc.js's file header); it starts
 *                   its PC at the address SIMH's PC will be at after executing them (computed, not
 *                   measured, since every setup instruction's length is fixed at emission) and
 *                   calls addInterruptSource()/raiseInterrupt() directly, exactly playing the part
 *                   real vax_sysdev.c code just played on the other side.  The DISPATCH that follows
 *                   is still graded against SIMH's real, event-queue-free interrupt delivery.
 *
 * MEASURED FACT: IPL 0x15 AND 0x17 HAVE NO DEVICE ON THIS MACHINE
 * ----------------------------------------------------------------
 * Every hardware-interrupt IPL macro vaxmod_defs.h defines (IPL_TTI, IPL_TTO, IPL_CSI, IPL_CSO,
 * IPL_RQ, IPL_RL, IPL_XQ, IPL_TMR0, IPL_TMR1, ... -- every one except IPL_CLK) resolves to
 * `(0x14 - IPL_HMIN)`; IPL_CLK is the only one at `(0x16 - IPL_HMIN)`.  Grepped exhaustively across
 * vax_stddev.c/vax_sysdev.c/vaxmod_defs.h: no code path in this SIMH build ever asserts a request at
 * IPL 0x15 or IPL 0x17 (Qbus BR5/BR7 are unused by every device this model attaches).  A live-SIMH
 * comparison at those two levels is therefore not a gap in this harness -- it is a structural
 * property of the emulated hardware.  The coverage floor below still requires all four levels
 * "requested and delivered at least once" (the item's own DONE CONDITION), so levels 0x15/0x17 are
 * exercised test-double-only, graded against an INDEPENDENTLY computed expected dispatch (the same
 * few lines of arithmetic intexc() performs -- old PSL/PC pushed, new PSL from mode+ipl, new PC from
 * the SCB slot -- written fresh here, not by calling into exc.js), rather than a second live process.
 * See phaseSynthetic()'s docblock.  This is disclosed, not hidden: see the report's test_decisions.
 *
 * COVERAGE IS ASSERTED, NOT REPORTED
 * -----------------------------------
 * Every one of: all 4 hardware IPLs (0x14-0x17) requested AND delivered, the evaporated-request
 * path, and the masked-then-unmasked path, is tracked in a Set/counter and the run FAILS if any is
 * missing -- the floors do not shrink with a smaller case list because the case list here is a fixed,
 * enumerated matrix, not a sample; nothing here scales with a --cases flag.
 *
 *      node machines/dec/vax/tests/hwintdiff.js [--simh PATH] [--selfcheck]
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import VAXCpu, { DISPATCH } from "../modules/v2/cpu.js";
import { OPCODES } from "../modules/v2/drom.js";
import VAXExc, {
    VAXStop, executeExc, SCB, IPL_HMIN, IPL_HMAX, QB_VEC_MASK,
    PSL_V_CUR, PSL_V_IPL, PSL_M_IPL, KERN
} from "../modules/v2/exc.js";

function hex(v, n = 8) { return ((v >>> 0).toString(16).toUpperCase()).padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Locating SIMH -- no fixture fallback, matching every other differential in this project.        *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_HWINT_BIN']) candidates.push(process.env['SIMH_HWINT_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH microvax3900; it has no fixture fallback.  Build one\n" +
        "with machines/dec/vax/tests/simh/build.sh and pass --simh PATH or set $SIMH_HWINT_BIN.\n" +
        "Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 28, timeout: 5 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test                                                                         *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE = 0x01000000;             // 16MB

class HwIntCpu extends VAXCpu {
    constructor(bus)
    {
        super(bus);
        this.exc = new VAXExc(this);
    }
    executeOne(opc, decoder, cpu)
    {
        if (executeExc(opc, decoder, cpu)) return;
        let fn = DISPATCH[opc];
        if (fn) { fn(cpu, decoder); return; }
        throw new Error(`hwintdiff: opcode ${hex(opc, 3)} (${OPCODES[opc] || "?"}) has no body wired into this harness`);
    }
    /** One `step 1` -- see exc.js's stepInstruction() for why one step can be several dispatches. */
    stepOne() { return this.exc.stepInstruction(this, (opc, d, c) => this.executeOne(opc, d, c)); }
}

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new HwIntCpu(bus);
    return {bus, cpu};
}

/* ------------------------------------------------------------------------------------------- *
 * Fixed physical layout, MAPPING OFF -- a virtual address is its own physical address.  Every     *
 * byte a case touches is deposited/set fresh, and `reset all` runs between SIMH cases so no          *
 * device's residual CSR/int_req state leaks from one case into the next.                            *
 * ------------------------------------------------------------------------------------------- */

const R_SCBB    = 0x00100000;
const R_HANDLER = 0x00102000;           // every SCB slot we use points here: a page of NOPs
const R_CODE    = 0x00104000;           // the case's instructions (setup MOVLs, if any, then NOPs)
const R_KSP     = 0x00110000;           // kernel stack top

const MOVL_OPC = OPCODES.indexOf("MOVL");
const NOP_BYTE = OPCODES.indexOf("NOP") & 0xFF;
if (MOVL_OPC < 0 || MOVL_OPC > 0xFF) throw new Error("hwintdiff: MOVL opcode not found or not single-byte");

const SSCBASE = 0x20140000;
/** SSC register-file offsets (vax_sysdev.c's `rg = (pa - SSCBASE) >> 2` switch), longwords. */
const SSC_RG = {T0CSR: 0x40, T0NI: 0x42, T0VEC: 0x43, T1CSR: 0x44, T1NI: 0x46, T1VEC: 0x47};
function sscAddr(rg) { return (SSCBASE + rg * 4) | 0; }

const TMR_CSR_IE = 0x40, TMR_CSR_SGL = 0x20, TMR_CSR_XFR = 0x10;
const TMR_SETUP_CSR = TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_SGL;      // 0x70 -- see file header

/* exc.js's own SCB table (imported above) does not carry the console/console-storage vectors --
   they are device SCB offsets, out of that table's scope (it stops at INTR).  Confirmed against
   vax_defs.h directly: SCB_CSI=0xF0, SCB_CSO=0xF4, SCB_TTI=0xF8, SCB_TTO=0xFC.  Kept as local
   constants rather than injected into the imported (shared, mutable) SCB object. */
const SCB_CSI = 0xF0, SCB_CSO = 0xF4, SCB_TTI = 0xF8, SCB_TTO = 0xFC;

/** The five REAL SIMH devices this file grounds fixed-vector delivery against. */
const REAL_DEVICES = [
    {name: "TTI", lvl: 0x14, bit: 8,  vec: SCB_TTI},
    {name: "TTO", lvl: 0x14, bit: 9,  vec: SCB_TTO},
    {name: "CSI", lvl: 0x14, bit: 13, vec: SCB_CSI},
    {name: "CSO", lvl: 0x14, bit: 14, vec: SCB_CSO},
    {name: "CLK", lvl: 0x16, bit: 0,  vec: SCB.INTTIM}
];

function emitMOVLimmToAbs(bytes, immVal, absAddr)
{
    bytes.push(MOVL_OPC & 0xFF);
    bytes.push(0x8F);
    pushLong(bytes, immVal);
    bytes.push(0x9F);
    pushLong(bytes, absAddr);
}
function pushLong(bytes, v) { v = v | 0; bytes.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF); }

/**
 * tmrSetupBytes(tmr, vecValue)
 *
 * The three real MOVL instructions that make SIMH's real SSC timer `tmr` (0 or 1) synchronously
 * raise a genuine hardware interrupt with a genuine dynamic vector -- see the file header.
 *
 * @param {number} tmr 0 or 1
 * @param {number} vecValue the raw value programmed into T#VEC (masked by TMR_VEC_MASK on write,
 *                           and AGAIN by QB_VEC_MASK when the interrupt is actually acknowledged)
 * @returns {number[]} instruction bytes
 */
function tmrSetupBytes(tmr, vecValue)
{
    let ni = tmr ? SSC_RG.T1NI : SSC_RG.T0NI;
    let vec = tmr ? SSC_RG.T1VEC : SSC_RG.T0VEC;
    let csr = tmr ? SSC_RG.T1CSR : SSC_RG.T0CSR;
    let bytes = [];
    emitMOVLimmToAbs(bytes, -1, sscAddr(ni));                 // T#NI = 0xFFFFFFFF
    emitMOVLimmToAbs(bytes, vecValue | 0, sscAddr(vec));      // T#VEC = vecValue (masked on write)
    emitMOVLimmToAbs(bytes, TMR_SETUP_CSR, sscAddr(csr));     // T#CSR = XFR|IE|SGL -> synchronous SET_INT
    return bytes;
}

/* ------------------------------------------------------------------------------------------- *
 * Case matrix.  Each case is EITHER "real" (graded against live SIMH) or "synthetic" (0x15/0x17, *
 * graded against an independently computed expectation -- see phaseSynthetic()).                  *
 * ------------------------------------------------------------------------------------------- */

/**
 * @typedef {Object} CaseSpec
 * @property {string} name
 * @property {number[]} setupBytes   real MOVL bytes to execute before the measured step(s) (may be
 *                                   empty)
 * @property {number} setupInstrs    how many `step`s the setup bytes are (0 if setupBytes is empty)
 * @property {Array<{lvl:number, bit:number, vec:number, simhName:?string}>} prime  sources to raise
 *           BEFORE the first measured step (simhName null for a dynamic/synthetic source with no
 *           depositable INT flag -- those are raised via setupBytes/real execution instead)
 * @property {Array<{lvl:number, bit:number, simhName:string}>} withdraw  sources to withdraw again
 *           before the first measured step (the evaporated-request case)
 * @property {Array<{ipl:number}>} steps  one measured `step 1` per entry, in order; `ipl` is
 *           deposited into PSL<IPL> immediately before that step
 */

const REAL_CASES = [
    /* One case per fixed-vector device: raise, one measured step, expect dispatch. */
    ...REAL_DEVICES.map((d) => ({
        name: `fixed_${d.name}`,
        setupBytes: [], setupInstrs: 0,
        prime: [{lvl: d.lvl, bit: d.bit, vec: d.vec, simhName: d.name}],
        withdraw: [],
        steps: [{ipl: 0}]
    })),

    /* Dynamic vector, timer 0, vector fully inside QB_VEC_MASK (no truncation). */
    {
        name: "dynamic_tmr0_lo",
        setupBytes: tmrSetupBytes(0, 0x100), setupInstrs: 3,
        prime: [{lvl: 0x14, bit: 15, vec: 0x100, simhName: null}],
        withdraw: [],
        steps: [{ipl: 0}]
    },
    /* Dynamic vector, timer 0, vector ABOVE QB_VEC_MASK -- SIMH truncates 0x2A8 -> 0x0A8 at
       acknowledge time; this is the case that actually exercises deviceVector()'s QB_VEC_MASK. */
    {
        name: "dynamic_tmr0_truncated",
        setupBytes: tmrSetupBytes(0, 0x2A8), setupInstrs: 3,
        prime: [{lvl: 0x14, bit: 15, vec: 0x2A8, simhName: null}],
        withdraw: [],
        steps: [{ipl: 0}]
    },
    /* Dynamic vector, timer 1 -- a second bit at the same level, proving the ack scan indexes by
       BIT, not just by level. */
    {
        name: "dynamic_tmr1",
        setupBytes: tmrSetupBytes(1, 0x180), setupInstrs: 3,
        prime: [{lvl: 0x14, bit: 16, vec: 0x180, simhName: null}],
        withdraw: [],
        steps: [{ipl: 0}]
    },

    /* Evaporated request: TTO raised, then withdrawn, before the CPU ever looks -- nothing should
       dispatch (mutation: "request not cleared on acknowledge" would still show 0 here, since this
       case never reaches an acknowledge at all; it is the OTHER required mutation -- see selfcheck). */
    {
        name: "evaporated",
        setupBytes: [], setupInstrs: 0,
        prime: [{lvl: 0x14, bit: 9, vec: SCB_TTO, simhName: "TTO"}],
        withdraw: [{lvl: 0x14, bit: 9, simhName: "TTO"}],
        steps: [{ipl: 0}]
    },

    /* Masked, then unmasked: TTO raised at PSL<IPL>=0x14 (masks its own level -- evalInt bails at
       i<=ipl before ever consulting intReq[0]), first step must NOT dispatch and must leave the
       request pending; second step at IPL 0 must dispatch it. */
    {
        name: "masked_then_unmasked",
        setupBytes: [], setupInstrs: 0,
        prime: [{lvl: 0x14, bit: 9, vec: SCB_TTO, simhName: "TTO"}],
        withdraw: [],
        steps: [{ipl: 0x14}, {ipl: 0}]
    },

    /* Multiple simultaneous requests at different levels: TTI (0x14) and CLK (0x16) raised
       together at IPL 0.  evalInt scans IPL_HMAX down to IPL_HMIN, so CLK must win the FIRST step;
       TTI is untouched by that (deviceVector only clears the bit it resolves) and must be
       delivered by the SECOND step. */
    {
        name: "multi_level",
        setupBytes: [], setupInstrs: 0,
        prime: [
            {lvl: 0x14, bit: 8, vec: SCB_TTI, simhName: "TTI"},
            {lvl: 0x16, bit: 0, vec: SCB.INTTIM, simhName: "CLK"}
        ],
        withdraw: [],
        steps: [{ipl: 0}, {ipl: 0}]
    }
];

/** The 2 of 4 hardware levels this SIMH build's device set structurally cannot ground live -- see
    the file header's MEASURED FACT.  Exercised test-double-only; see phaseSynthetic(). */
const SYNTHETIC_CASES = [
    {name: "synthetic_0x15", lvl: 0x15, bit: 3, vec: 0x140},
    {name: "synthetic_0x17", lvl: 0x17, bit: 5, vec: 0x160}
];

/* ------------------------------------------------------------------------------------------- *
 * SIMH-side script builder                                                                        *
 * ------------------------------------------------------------------------------------------- */

const CASE_MARK = "HCASE_";
/* Probed on every step: R0-R15, PSL, and the two longwords just below the PRE-step KSP -- where an
   exception frame (old PC at sp-8, old PSL at sp-4) would land if one was pushed.  A non-dispatching
   step must show these UNCHANGED (still the zero this file pre-deposits), which is exactly what
   makes the evaporated/masked-not-yet-unmasked steps a real assertion and not merely "didn't crash". */
function probeNames() { let n = []; for (let r = 0; r < 15; r++) n.push("R" + r); n.push("PC", "PSL"); return n; }

function buildScript(cases)
{
    let L = [`set cpu ${MEMSIZE / (1024 * 1024)}m`, "set cpu simhalt"];
    for (let c of cases) {
        L.push("reset all");
        /* Baseline: kernel mode, IPL 0, mapping off, SCB + a page of NOPs at every vector we use. */
        L.push(`deposit MAPEN 0`);
        L.push(`deposit SCBB ${hex(R_SCBB)}`);
        L.push(`deposit KSP ${hex(R_KSP)}`);
        L.push(`deposit R14 ${hex(R_KSP)}`);
        for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_HANDLER + k)} ${NOP_BYTE.toString(16)}`);
        let vectors = new Set([SCB_TTI, SCB_TTO, SCB_CSI, SCB_CSO, SCB.INTTIM]);
        for (let v of c.prime) vectors.add((v.vec & QB_VEC_MASK) & ~3);   // the DELIVERED vector, post get_vector()'s masking
        for (let v of vectors) L.push(`deposit ${hex(R_SCBB + v)} ${hex(R_HANDLER)}`);
        /* Zero the frame-probe words below the stack top and the code page, so a stray push or a
           leftover setup-instruction byte from an earlier case can never be mistaken for this one's. */
        for (let k = -8; k <= 0; k++) L.push(`deposit ${hex((R_KSP + k * 4) | 0)} 0`);
        for (let k = 0; k < 64; k++) L.push(`deposit -b ${hex(R_CODE + k)} 0`);
        for (let i = 0; i < c.setupBytes.length; i++) L.push(`deposit -b ${hex(R_CODE + i)} ${c.setupBytes[i].toString(16)}`);
        /* The probe NOP goes right after the setup bytes (0 bytes for a fixed-vector case). */
        /* 8 NOPs, not 1: a case with more than one measured step that does NOT dispatch (masking)
           advances PC one byte per step through here, and it must never run off the end into the
           zeroed bytes beyond -- opcode 0 is HALT (see exc.js's HALT body: "opcode 0x00 is also
           what an accidental branch into zeroed memory does"), which would stop the simulator
           instead of producing a comparable state. */
        let probePC = (R_CODE + c.setupBytes.length) | 0;
        for (let k = 0; k < 8; k++) L.push(`deposit -b ${hex(probePC + k)} ${NOP_BYTE.toString(16)}`);
        L.push(`deposit PSL 0`);
        L.push(`deposit PC ${hex(R_CODE)}`);
        if (c.setupInstrs > 0) L.push(`step ${c.setupInstrs}`);
        /* Prime the FIXED-vector devices (the dynamic ones were already primed for real by the
           setup instructions above -- see tmrSetupBytes()'s docblock). */
        for (let p of c.prime) if (p.simhName) L.push(`deposit ${p.simhName} INT 1`);
        for (let w of c.withdraw) L.push(`deposit ${w.simhName} INT 0`);
        L.push(`deposit PC ${hex(probePC)}`);
        for (let s of c.steps) {
            L.push(`deposit PSL ${hex(s.ipl << PSL_V_IPL)}`);
            L.push(`echo ${CASE_MARK}${c.name}`);
            L.push("step 1");
            L.push(`examine -h ${probeNames().join(",")}`);
            for (let k = -8; k <= 0; k++) L.push(`examine -h ${hex((R_KSP + k * 4) | 0)}`);
        }
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/;

/**
 * runAll(simh, cases, scratch)
 *
 * @returns {Map<string, Array<{regs:Int32Array, psl:number, frame:number[]}>>} keyed by case name,
 *          one array entry per measured step, in order
 */
function runAll(simh, cases, scratch)
{
    let script = buildScript(cases);
    let out = runSimh(simh, script, path.join(scratch, "hwintdiff.ini"));
    if (process.env.HWINTDIFF_DUMP) fs.writeFileSync(process.env.HWINTDIFF_DUMP, out);
    let lines = out.split("\n");
    let results = new Map();
    let want = 17 + 9;                                     // 15 regs + PC + PSL, then 9 frame words
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\S+)"));
        if (!m) { i++; continue; }
        let name = m[1];
        i++;
        let vals = [];
        while (i < lines.length && vals.length < want) {
            if (lines[i].indexOf(CASE_MARK) >= 0) break;
            let vm = lines[i].match(VALUE_RE);
            if (vm) vals.push(parseInt(vm[2], 16) | 0);
            i++;
        }
        if (vals.length < want) { if (!results.has(name)) results.set(name, []); continue; }
        let regs = new Int32Array(16);
        for (let r = 0; r < 15; r++) regs[r] = vals[r];
        regs[15] = vals[15];
        let psl = vals[16];
        let frame = vals.slice(17);
        if (!results.has(name)) results.set(name, []);
        results.get(name).push({regs, psl, frame, reached: true});
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * JS side                                                                                          *
 * ------------------------------------------------------------------------------------------- */

/** Prime the JS test double exactly as the SIMH script's `deposit <DEV> INT 1` / real MOVL setup
    did -- the installer + mutator this item added, called the way a real device would call them. */
function primeJS(cpu, c)
{
    for (let p of c.prime) {
        cpu.exc.addInterruptSource(p.lvl, p.bit, p.vec);
        cpu.exc.raiseInterrupt(p.lvl, p.bit);
    }
    for (let w of c.withdraw) cpu.exc.clearInterrupt(w.lvl, w.bit);
}

/**
 * runCaseJS(m, c)
 *
 * @returns {Array<{regs:Int32Array, psl:number, frame:number[]}>} one entry per measured step
 */
function runCaseJS(m, c)
{
    let {bus, cpu} = m;
    cpu.exc.reset();
    cpu.mmu.reset();
    cpu.mmu.setMAPEN(0);
    cpu.exc.scbb = R_SCBB;
    cpu.exc.stk[KERN] = R_KSP;
    cpu.regs.fill(0);
    cpu.regs[14] = R_KSP;
    for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
    let vectors = new Set([SCB_TTI, SCB_TTO, SCB_CSI, SCB_CSO, SCB.INTTIM]);
    for (let v of c.prime) vectors.add((v.vec & QB_VEC_MASK) & ~3);   // the DELIVERED vector, post get_vector()'s masking
    for (let v of vectors) bus.setLong(R_SCBB + v, R_HANDLER);
    for (let k = -8; k <= 0; k++) bus.setLong((R_KSP + k * 4) | 0, 0);
    for (let k = 0; k < 64; k++) bus.setByte(R_CODE + k, 0);
    /* The dynamic-vector setup MOVLs are NOT executed on this side -- see the file header: there is
       no SSC device model here to react to the physical write, by design (this item does not
       implement one).  PC starts at the same address SIMH's PC will be at after really executing
       them, and primeJS() below plays the device's part directly. */
    let probePC = (R_CODE + c.setupBytes.length) | 0;
    for (let k = 0; k < 8; k++) bus.setByte(probePC + k, NOP_BYTE);
    cpu.psl = 0;
    cpu.regs[15] = probePC;
    primeJS(cpu, c);

    let out = [];
    for (let s of c.steps) {
        cpu.psl = (s.ipl << PSL_V_IPL) | 0;
        try {
            cpu.stepOne();
        } catch (e) {
            if (e instanceof VAXStop) { out.push({stop: e.reason}); continue; }
            throw e;
        }
        let frame = [];
        for (let k = -8; k <= 0; k++) frame.push(bus.getLong((R_KSP + k * 4) | 0) | 0);
        out.push({regs: Int32Array.from(cpu.regs), psl: cpu.psl, frame});
    }
    return out;
}

function compareStep(name, stepIdx, js, sr)
{
    let bad = [];
    let tag = `${name} step${stepIdx}`;
    if (!sr || !sr.reached) { bad.push(`${tag}: SIMH result not reached`); return bad; }
    if (js.stop) { bad.push(`${tag}: cpu.js stopped: ${js.stop}`); return bad; }
    for (let r = 0; r < 16; r++) {
        if ((js.regs[r] | 0) !== (sr.regs[r] | 0)) bad.push(`${tag}: R${r} js=${hex(js.regs[r])} simh=${hex(sr.regs[r])}`);
    }
    if ((js.psl | 0) !== (sr.psl | 0)) bad.push(`${tag}: PSL js=${hex(js.psl)} simh=${hex(sr.psl)}`);
    for (let k = 0; k < js.frame.length; k++) {
        if ((js.frame[k] | 0) !== (sr.frame[k] | 0)) bad.push(`${tag}: frame[${k}] js=${hex(js.frame[k])} simh=${hex(sr.frame[k])}`);
    }
    return bad;
}

/* ------------------------------------------------------------------------------------------- *
 * phaseSynthetic -- IPL 0x15/0x17, no live device exists; grade against an INDEPENDENTLY            *
 * computed expected dispatch (old PC/PSL pushed, new PSL, new PC) rather than a second process.     *
 * The arithmetic here is deliberately NOT shared code with exc.js's intexc() -- it is written        *
 * fresh from the SRM's description of what an interrupt dispatch does, specifically so that a        *
 * bug shared between "the thing under test" and "the checker" cannot cancel out.                    *
 * ------------------------------------------------------------------------------------------- */

function phaseSynthetic()
{
    let failures = [];
    let levelsSeen = new Set();
    let {bus, cpu} = makeMachine();
    for (let s of SYNTHETIC_CASES) {
        cpu.exc.reset();
        cpu.mmu.reset();
        cpu.mmu.setMAPEN(0);
        cpu.exc.scbb = R_SCBB;
        cpu.exc.stk[KERN] = R_KSP;
        cpu.regs.fill(0);
        cpu.regs[14] = R_KSP;
        for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
        let vecAligned = s.vec & ~3;
        bus.setLong(R_SCBB + vecAligned, R_HANDLER);
        for (let k = -8; k <= 0; k++) bus.setLong((R_KSP + k * 4) | 0, 0);
        bus.setByte(R_CODE, NOP_BYTE);
        cpu.psl = 0;
        cpu.regs[15] = R_CODE;

        cpu.exc.addInterruptSource(s.lvl, s.bit, s.vec);
        cpu.exc.raiseInterrupt(s.lvl, s.bit);

        let oldpc = cpu.regs[15], oldpsl = cpu.psl, oldsp = cpu.regs[14];
        cpu.stepOne();

        /* Independently computed expectation, per intexc()'s documented algorithm (SCB.INTR-style
           interrupt: new mode kernel, new IPL = the level taken, cc cleared, frame = [oldPC, oldPSL]
           at [sp-8, sp-4], new PC from the SCB slot masked to a longword boundary) -- PLUS one byte,
           because stepOne() is one `step 1`, and stepInstruction() documents that a single step
           performs the dispatch AND THEN executes one real instruction in the same call (the NOP
           this file deposits at the handler entry), exactly as a real SIMH `step 1` would. */
        let expNewPC = (R_HANDLER & ~3) + 1;
        let expNewPSL = (KERN << PSL_V_CUR) | (s.lvl << PSL_V_IPL);
        let expSP = (oldsp - 8) | 0;
        let expFrameLo = oldpc;               // at expSP (old PC)
        let expFrameHi = oldpsl;              // at expSP+4 (old PSL)

        levelsSeen.add(s.lvl);
        let tag = s.name;
        if ((cpu.regs[15] | 0) !== (expNewPC | 0)) failures.push(`${tag}: PC js=${hex(cpu.regs[15])} expected=${hex(expNewPC)}`);
        if ((cpu.psl | 0) !== (expNewPSL | 0)) failures.push(`${tag}: PSL js=${hex(cpu.psl)} expected=${hex(expNewPSL)}`);
        if ((cpu.regs[14] | 0) !== (expSP | 0)) failures.push(`${tag}: SP js=${hex(cpu.regs[14])} expected=${hex(expSP)}`);
        if ((bus.getLong(expSP) | 0) !== (expFrameLo | 0)) failures.push(`${tag}: frame[sp] (old PC) js=${hex(bus.getLong(expSP))} expected=${hex(expFrameLo)}`);
        if ((bus.getLong((expSP + 4) | 0) | 0) !== (expFrameHi | 0)) failures.push(`${tag}: frame[sp+4] (old PSL) js=${hex(bus.getLong((expSP + 4) | 0))} expected=${hex(expFrameHi)}`);
    }
    return {failures, levelsSeen};
}

/* ------------------------------------------------------------------------------------------- *
 * selfcheck -- named mutations, each proven caught against the real case(s) that would expose it. *
 * ------------------------------------------------------------------------------------------- */

function selfcheck(simh, scratch)
{
    let results = [];

    function checkAgainst(mutName, caseNames, mutate, restore)
    {
        let m = makeMachine();
        mutate();
        let caught = false, detail = null;
        try {
            let cases = REAL_CASES.filter((c) => caseNames.includes(c.name));
            let sr = runAll(simh, cases, scratch);
            for (let c of cases) {
                let jsSteps = runCaseJS(m, c);
                let simhSteps = sr.get(c.name) || [];
                for (let i = 0; i < c.steps.length; i++) {
                    let bad = compareStep(c.name, i, jsSteps[i] || {stop: "missing"}, simhSteps[i]);
                    if (bad.length) { caught = true; detail = bad[0]; break; }
                }
                if (caught) break;
            }
        } finally {
            restore();
        }
        results.push({mn: mutName, caught, detail});
        console.log(`  selfcheck ${mutName}: ${caught ? "CAUGHT" : "*** NOT CAUGHT ***"}${detail ? " (" + detail + ")" : ""}`);
    }

    /* 1. Off-by-one in the intReq index (lvl - IPL_HMIN mapping).  Break addInterruptSource AND
          raiseInterrupt/clearInterrupt/deviceVector consistently at +1, so the device's own view of
          "its" slot is self-consistent but wrong relative to evalInt()'s (unmutated) indexing --
          exactly an off-by-one a port could introduce in one spot and not the other. */
    {
        let origAdd = VAXExc.prototype.addInterruptSource;
        let origRaise = VAXExc.prototype.raiseInterrupt;
        let origClear = VAXExc.prototype.clearInterrupt;
        checkAgainst("intReq off-by-one index", ["fixed_TTI", "fixed_CLK"],
            () => {
                VAXExc.prototype.addInterruptSource = function(lvl, bit, vec) { origAdd.call(this, lvl + 1, bit, vec); };
                VAXExc.prototype.raiseInterrupt = function(lvl, bit) { origRaise.call(this, lvl + 1, bit); };
                VAXExc.prototype.clearInterrupt = function(lvl, bit) { origClear.call(this, lvl + 1, bit); };
            },
            () => {
                VAXExc.prototype.addInterruptSource = origAdd;
                VAXExc.prototype.raiseInterrupt = origRaise;
                VAXExc.prototype.clearInterrupt = origClear;
            });
    }

    /* 2. Request not cleared on acknowledge -- re-delivers forever.  "multi_level"'s second step
          (still IPL 0) would then re-deliver CLK's vector again instead of TTI's -- a case
          purpose-built so two DIFFERENT devices are at stake and the defect cannot hide by
          coincidentally reading the same vector twice. */
    {
        let origDV = VAXExc.prototype.deviceVector;
        checkAgainst("acknowledge does not clear the request", ["multi_level"],
            () => {
                VAXExc.prototype.deviceVector = function(cpu, lvl) {
                    let l = lvl - IPL_HMIN;
                    let req = this.intReq[l];
                    let table = this.intVec[l];
                    for (let i = 0; i < 32 && req; i++) {
                        if ((req >>> i) & 1) {
                            /* deliberately NOT clearing this.intReq[l] here */
                            let v = table[i];
                            if (v === undefined) return 0;
                            let vec = (typeof v === "function") ? (v(cpu) | 0) : (v | 0);
                            return vec & QB_VEC_MASK;
                        }
                    }
                    return 0;
                };
            },
            () => { VAXExc.prototype.deviceVector = origDV; });
    }

    /* 3. Priority inverted so a lower level wins.  This is NOT a change to the shipped evalInt()
          (already correct, already graded by excdiff.js/intdiff.js) -- it is a --selfcheck-only
          monkeypatch scanning the hardware levels ascending instead of descending, exactly the
          established pattern excdiff.js's own selfcheck already uses for its software-level
          equivalent.  "multi_level" is exactly the case that can tell the difference: TTI (0x14)
          and CLK (0x16) are both pending, and only evalInt's SCAN ORDER decides which wins. */
    {
        let origEval = VAXExc.prototype.evalInt;
        checkAgainst("hardware-level priority inverted", ["multi_level"],
            () => {
                VAXExc.prototype.evalInt = function(cpu) {
                    let ipl = (cpu.psl >>> PSL_V_IPL) & PSL_M_IPL;
                    if (this.hltPin) return 0x1F;
                    for (let i = IPL_HMIN; i <= IPL_HMAX; i++) {          // ASCENDING: wrong
                        if (i <= ipl) continue;
                        if (this.intReq[i - IPL_HMIN]) return i;
                    }
                    return origEval.call(this, cpu);
                };
            },
            () => { VAXExc.prototype.evalInt = origEval; });
    }

    /* 4. Masking at the current IPL ignored.  Again a --selfcheck-only evalInt monkeypatch, not a
          shipped-code change.  "masked_then_unmasked"'s FIRST step is exactly the case this defect
          would flip from "no dispatch" to "dispatch". */
    {
        let origEval = VAXExc.prototype.evalInt;
        checkAgainst("masking at current IPL ignored", ["masked_then_unmasked"],
            () => {
                VAXExc.prototype.evalInt = function(cpu) {
                    let ipl = (cpu.psl >>> PSL_V_IPL) & PSL_M_IPL;
                    if (this.hltPin) return 0x1F;
                    for (let i = IPL_HMAX; i >= IPL_HMIN; i--) {
                        /* the `if (i <= ipl) return 0;` bail is deliberately OMITTED */
                        if (this.intReq[i - IPL_HMIN]) return i;
                    }
                    return origEval.call(this, cpu);
                };
            },
            () => { VAXExc.prototype.evalInt = origEval; });
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
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "hwintdiff-"));
    console.log(`hwintdiff.js: simh=${simh} scratch=${scratch}`);

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

    let problems = [];
    let m = makeMachine();

    console.log(`\n=== REAL phase: ${REAL_CASES.length} cases, graded against live SIMH ===`);
    let sr = runAll(simh, REAL_CASES, scratch);
    let levelsSeen = new Set();
    let fixedSeen = false, dynamicSeen = false, evaporatedSeen = false, maskedUnmaskedSeen = false, multiLevelSeen = false;
    let notReached = [];
    for (let c of REAL_CASES) {
        let jsSteps = runCaseJS(m, c);
        let simhSteps = sr.get(c.name) || [];
        if (simhSteps.length < c.steps.length) {
            notReached.push(`${c.name}: SIMH produced ${simhSteps.length}/${c.steps.length} step(s)`);
            continue;
        }
        for (let p of c.prime) levelsSeen.add(p.lvl);
        if (c.name.startsWith("fixed_")) fixedSeen = true;
        if (c.name.startsWith("dynamic_")) dynamicSeen = true;
        if (c.name === "evaporated") evaporatedSeen = true;
        if (c.name === "masked_then_unmasked") maskedUnmaskedSeen = true;
        if (c.name === "multi_level") multiLevelSeen = true;
        for (let i = 0; i < c.steps.length; i++) {
            for (let bad of compareStep(c.name, i, jsSteps[i] || {stop: "missing"}, simhSteps[i])) problems.push("REAL: " + bad);
        }
    }
    if (notReached.length) problems.push(`COVERAGE: case(s) did not reach comparison: ${notReached.join("; ")}`);

    console.log(`\n=== SYNTHETIC phase: IPL 0x15/0x17 (no live device exists -- see file header) ===`);
    let synth = phaseSynthetic();
    for (let f of synth.failures) problems.push("SYNTHETIC: " + f);
    for (let l of synth.levelsSeen) levelsSeen.add(l);

    console.log(`  levels requested+delivered: ${[...levelsSeen].sort((a, b) => a - b).map((v) => hex(v, 2)).join(",")}`);
    console.log(`  fixed=${fixedSeen} dynamic=${dynamicSeen} evaporated=${evaporatedSeen} maskedThenUnmasked=${maskedUnmaskedSeen} multiLevel=${multiLevelSeen}`);

    for (let want of [0x14, 0x15, 0x16, 0x17]) {
        if (!levelsSeen.has(want)) problems.push(`COVERAGE: IPL ${hex(want, 2)} was never requested AND delivered`);
    }
    if (!fixedSeen) problems.push("COVERAGE: no fixed-vector device case reached comparison");
    if (!dynamicSeen) problems.push("COVERAGE: no dynamic-vector device case reached comparison");
    if (!evaporatedSeen) problems.push("COVERAGE: the evaporated-request path was not taken");
    if (!maskedUnmaskedSeen) problems.push("COVERAGE: the masked-then-unmasked path was not taken");
    if (!multiLevelSeen) problems.push("COVERAGE: no multiple-simultaneous-level case reached comparison");

    if (problems.length) {
        console.error(`\nFAILED (${problems.length} problem(s)):`);
        for (let p of problems) console.error("  - " + p);
        process.exit(1);
    }
    console.log(`\nPASSED.`);
}

main();
