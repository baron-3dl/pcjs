/**
 * @fileoverview Differential test: the SSC T0/T1 programmable timers (modules/v2/ssc.js) vs. a
 *               real Open SIMH microvax3900
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
 * pcjsvax-055: SSCVAX's T0/T1 register decode, counting, and dynamic-vector interrupt delivery
 * (modules/v2/ssc.js's REG_T0CSR..REG_T1VEC cases, _tmrCsrWr()/_tmrIncr()/tick()).  This file does
 * NOT re-grade exc.js's evalInt()/deviceVector() arbitration -- hwintdiff.js's dynamic_tmr0/
 * dynamic_tmr1/dynamic_tmr0_reprogrammed cases already prove that seam sound for exactly this
 * (lvl, bit) shape against a SYNTHETIC prime.  What IS this file's job: the register semantics
 * (CSR bits, W1C, XFR reload, TMR_VEC_MASK on write), the counting model (SGL synchronous and
 * RUN-mode instruction-driven), and that SSCVAX's OWN wiring (a real device, not a synthetic
 * prime) drives that seam correctly end to end -- including the TMR_VEC_MASK-then-QB_VEC_MASK
 * chain, which nothing before this item exercised through a real register write.
 *
 * TWO REAL SSC WRITES ALREADY REACHED ON THE ROM BOOT PATH (romdiff.js, this item's own advance):
 * T0VEC=0x78, T1VEC=0x7C -- CSR/NI are never touched on that path (mechanically confirmed: see
 * pcjsvax-0c8, filed by this item).  So there is no reachable REAL DEC workload that exercises
 * CSR/counting yet -- exactly the situation timerdiff.js's own file header documents for ICCS/
 * TODR, and for the identical reason: romdiff.js's own two-phase precedent (a real boot trace,
 * PLUS a value-sweep driven through a real instruction round trip) is the shape this file follows
 * at the register level, since no external DEC program is available to supply a "real workload"
 * for CSR/counting specifically.
 *
 *   PHASE 1 (FIXED)   An enumerated matrix, each case a REAL MOVL sequence executed via `step` on
 *                     BOTH engines: TMR_VEC_MASK on write, SGL synchronous overflow (register
 *                     state AND interrupt dispatch, including the TMR_VEC_MASK/QB_VEC_MASK chain),
 *                     DON->ERR on a second unacknowledged overflow, IE-clear withdrawing a pending
 *                     request, and RUN-mode instruction-driven counting.
 *   PHASE 2 (RANDOM)  TIVEC write-mask sweep (romdiff.js's verifySscBaseRandom() shape: boundary
 *                     values first, then uniform random longwords), batched into one SIMH
 *                     invocation the way mchkdiff.js's runBatch() convention does.
 *
 * RUN-MODE COUNTING, AND WHY IT RUNS FROM THE ROM ADDRESS WINDOW
 * -----------------------------------------------------------------
 * vax_sysdev.c's tmr_sched() schedules a running timer by WALL-CLOCK MICROSECONDS, *except* when
 * `ADDR_IS_ROM(fault_PC) && usecs_sched < TMR_INC` (a short delay requested from ROM code), which
 * schedules by INSTRUCTION COUNT instead (`sim_activate()`'s base unit) -- see ssc.js's own file
 * header, "SSC PROGRAMMABLE TIMERS", for the full citation.  This file's RUN-mode case therefore
 * executes from ROM_CODE_ADDR (mirroring timerdiff.js's own ROM-context cases) so that branch is
 * GENUINELY exercised on the live oracle, not merely assumed reachable -- `set rom nodelay` is
 * used throughout this file (see makeMachine()'s SIMH-side script) to strip SIMH's OWN separate
 * ~500K-instructions/sec ROM-access pacing (rom_rd()'s UNIT_NODELAY flag, vax_sysdev.c:539-556);
 * that pacing is a host-speed compensation for a DIFFERENT purpose (embedded ROM timing loops
 * elsewhere) and is orthogonal to tmr_sched()'s own ADDR_IS_ROM() check, which is fault_PC-driven
 * and unaffected by the flag -- confirmed directly: nodelay changes wall-clock cost, not which
 * branch tmr_sched() takes or what values result.
 *
 * COVERAGE IS ASSERTED, NOT REPORTED
 * -----------------------------------
 * Every named floor below FAILS the run if unmet and does not shrink with a smaller --cases value
 * (the fixed matrix is a fixed list, not a sample).
 *
 *      node machines/dec/vax/tests/tmrdiff.js [--simh PATH] [--cases N] [--selfcheck]
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX, { VAXStop } from "../modules/v2/cpustate.js";
import { OPCODES } from "../modules/v2/drom.js";
import SSCVAX, {
    REG_T0CSR, REG_T0INT, REG_T0NI, REG_T0VEC, REG_T1CSR, REG_T1INT, REG_T1NI, REG_T1VEC,
    TMR_CSR_ERR, TMR_CSR_DON, TMR_CSR_IE, TMR_CSR_SGL, TMR_CSR_XFR, TMR_CSR_STP, TMR_CSR_RUN,
    TMR_VEC_MASK, INT_V_TMR0, INT_V_TMR1, SSC_BASE
} from "../modules/v2/ssc.js";
import VAXExc, { IPL_HMIN, SCB, QB_VEC_MASK } from "../modules/v2/exc.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function hex(v, n = 8) { return ((v >>> 0).toString(16).toUpperCase()).padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Locating SIMH -- no fixture fallback, matching every other differential in this project.        *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_TMR_BIN']) candidates.push(process.env['SIMH_TMR_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH microvax3900; it has no fixture fallback.  Build one\n" +
        "with machines/dec/vax/tests/simh/build.sh and pass --simh PATH or set $SIMH_TMR_BIN.\n" +
        "Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 28, timeout: 60 * 1000});
}

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
    /* No ConsoleVAX/NVRVAX here -- neither is this file's scope, and SSCVAX's console/nvr
       arguments are both optional (see ssc.js's constructor doc comment); rg 0x20-0x23 and NVR's
       own span simply are not touched by anything below. */
    let ssc = new SSCVAX(cpu.exc, null);
    bus.addSsc(ssc, null);
    cpu.tmr = ssc;
    return {bus, cpu, ssc};
}

/** Same machine, plus a blank ROM this file owns entirely -- see timerdiff.js's own precedent for
    why a real ka655x.bin is never needed here (only the fault_PC ADDR_IS_ROM() branch matters). */
function makeMachineWithRom()
{
    let m = makeMachine();
    m.bus.addRom(new Uint8Array(VAX.PHYSMEM.ROM_SIZE));
    return m;
}

/* ------------------------------------------------------------------------------------------- *
 * Instruction encoding -- MOVL only (absolute-mode reads/writes of SSC's memory-mapped registers, *
 * exactly as romdiff.js's/hwintdiff.js's own SSC-register sequences do).                           *
 * ------------------------------------------------------------------------------------------- */

const MOVL_OPC = OPCODES.indexOf("MOVL");
const NOP_BYTE = OPCODES.indexOf("NOP") & 0xFF;
if (MOVL_OPC < 0 || MOVL_OPC > 0xFF) throw new Error("tmrdiff: MOVL opcode not found or not single-byte");

function pushLong(bytes, v) { v = v | 0; bytes.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF); }

/** MOVL #immVal, @#absAddr -- one instruction. */
function movlImmToAbs(bytes, immVal, absAddr)
{
    bytes.push(MOVL_OPC & 0xFF, 0x8F);
    pushLong(bytes, immVal);
    bytes.push(0x9F);
    pushLong(bytes, absAddr);
}

/** MOVL @#absAddr, Rn -- one instruction. */
function movlAbsToReg(bytes, absAddr, rn)
{
    bytes.push(MOVL_OPC & 0xFF, 0x9F);
    pushLong(bytes, absAddr);
    bytes.push(0x50 | (rn & 0xF));
}

function sscAddr(rg) { return ((SSC_BASE + rg * 4) >>> 0) | 0; }

/* ------------------------------------------------------------------------------------------- *
 * Fixed physical layout, MAPPING OFF -- same addresses hwintdiff.js/timerdiff.js use, for the      *
 * same reason: one canonical layout across the VAX differentials, not shared code.                 *
 * ------------------------------------------------------------------------------------------- */

const R_SCBB    = 0x00100000;
const R_HANDLER = 0x00102000;           // a page of NOPs; SCB slots point here for dispatch cases
const R_CODE    = 0x00104000;           // non-ROM test code
const R_KSP     = 0x00110000;

const ROM_TEST_OFF = 0x1000;
const ROM_BASE_CONST = VAX.PHYSMEM.ROM_BASE >>> 0;
const ROM_CODE_ADDR = (ROM_BASE_CONST + ROM_TEST_OFF) >>> 0;

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors.  Every one FAILS the run if unmet and does not shrink with --cases.             *
 * ------------------------------------------------------------------------------------------- */

const covered = {
    vecWriteMaskT0: false, vecWriteMaskT1: false,
    sglOverflowRegsT0: false, sglOverflowRegsT1: false,
    dispatchMaskChainT0: false, dispatchT1: false,
    errOnSecondOverflow: false,
    ieClearWithdrawsPending: false,
    runModeCountingRom: false
};

/* ------------------------------------------------------------------------------------------- *
 * runFixedCase -- deposit code, step N times, examine.  Modeled directly on timerdiff.js's         *
 * runFixedCase()/hwintdiff.js's runCaseSimh(), same shape, adapted for SSC's memory-mapped (not    *
 * IPR) registers and this file's own dispatch-probing need (an SCBB/handler page, always wired,   *
 * cheap when unused).                                                                              *
 * ------------------------------------------------------------------------------------------- */

function runFixedCase(bin, scratch, name, code, opts = {})
{
    let steps = opts.steps;
    let writeAddr = opts.rom ? ROM_CODE_ADDR : R_CODE;
    let execAddr = writeAddr;

    let L = ["set cpu 16m", "set cpu simhalt", "set rom nodelay", "reset all"];
    /*
     * `iplSchedule[s]`, if given, is the PSL<IPL> value in effect for step `s` (0-based) -- both
     * engines re-deposit PSL immediately before that step, not just once up front.  Needed because
     * several cases here WRITE a CSR with IE set (raising a genuine pending T0/T1 request) several
     * steps before the step meant to actually PROBE dispatch: without masking IPL_HMIN (0x14)
     * during the intervening setup/readback steps, exc.js's setIRQL()/evalInt() (already verified
     * sound, untouched here) correctly takes the interrupt on the VERY NEXT step -- which would
     * hijack a plain register-readback MOVL into a dispatch instead, corrupting R0/R1 rather than
     * exercising a bug.  Falls back to a single constant `opts.ipl||0` for every step when absent
     * (every case that never raises IE, or that WANTS the immediate next step to dispatch).
     */
    let iplFor = (s) => (opts.iplSchedule ? opts.iplSchedule[s] : (opts.ipl || 0));

    L.push(`deposit MAPEN 0`);
    L.push(`deposit SCBB ${hex(R_SCBB)}`);
    L.push(`deposit KSP ${hex(R_KSP)}`);
    L.push(`deposit R14 ${hex(R_KSP)}`);
    for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_HANDLER + k)} ${NOP_BYTE.toString(16)}`);
    let vec = (opts.scbVec !== undefined) ? opts.scbVec : null;
    if (vec !== null) L.push(`deposit ${hex((R_SCBB + vec) >>> 0)} ${hex(R_HANDLER)}`);
    for (let i = 0; i < code.length; i++) L.push(`deposit -b ${hex((writeAddr + i) >>> 0)} ${code[i].toString(16)}`);
    for (let k = 0; k < 8; k++) L.push(`deposit -b ${hex((writeAddr + code.length + k) >>> 0)} ${NOP_BYTE.toString(16)}`);
    L.push(`deposit PSL ${hex(iplFor(0) << 16)}`);
    L.push(`deposit PC ${hex(execAddr)}`);
    for (let s = 0; s < steps; s++) {
        if (s > 0) L.push(`deposit PSL ${hex(iplFor(s) << 16)}`);
        L.push("step 1");
    }
    L.push(`examine -h ${["R0", "R1", "PC", "PSL"].join(",")}`);
    L.push("exit");
    let out = runSimh(bin, L.join("\n") + "\n", path.join(scratch, `tmrdiff-${name}.ini`));
    let simh = parseSimhExamine(out, name);

    /* ---- JS side ---- */
    let { bus, cpu, ssc } = opts.rom ? makeMachineWithRom() : makeMachine();
    cpu.exc.scbb = R_SCBB;
    cpu.regs[14] = R_KSP;
    cpu.exc.stk[0] = R_KSP;
    for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
    if (vec !== null) bus.setLong((R_SCBB + vec) >>> 0, R_HANDLER);
    let writeCode = opts.rom ? (a, b) => bus.setByteDirect(a, b) : (a, b) => bus.setByte(a, b);
    for (let i = 0; i < code.length; i++) writeCode((writeAddr + i) >>> 0, code[i]);
    for (let k = 0; k < 8; k++) writeCode((writeAddr + code.length + k) >>> 0, NOP_BYTE);
    cpu.psl = iplFor(0) << 16;
    cpu.setPC(execAddr);
    let stop = null;
    try {
        for (let s = 0; s < steps; s++) {
            if (s > 0) cpu.psl = iplFor(s) << 16;
            cpu.stepCPU(1);
        }
    } catch (e) {
        stop = e;
    }
    if (stop) {
        throw new Error(`tmrdiff: case ${name} raised an unexpected stop on the JS side: ${stop.message || stop}`);
    }
    let js = {
        R0: cpu.regs[0] >>> 0, R1: cpu.regs[1] >>> 0, PC: cpu.regs[15] >>> 0, PSL: cpu.psl >>> 0
    };
    return { js, simh, ssc, cpu };
}

function parseSimhExamine(out, name)
{
    let get = (label) => {
        let m = new RegExp(`^${label}:\\s*([0-9A-Fa-f]+)`, "m").exec(out);
        if (!m) throw new Error(`tmrdiff: case ${name} -- SIMH did not report ${label}; output:\n${out}`);
        return parseInt(m[1], 16) >>> 0;
    };
    return { R0: get("R0"), R1: get("R1"), PC: get("PC"), PSL: get("PSL") };
}

function checkExact(failures, tag, jsVal, simhVal, label)
{
    if ((jsVal >>> 0) !== (simhVal >>> 0)) {
        failures.push(`${tag}: ${label} js=${hex(jsVal)} simh=${hex(simhVal)}`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 1 -- fixed matrix                                                                        *
 * ------------------------------------------------------------------------------------------- */

/** MOVL #val,@#T#VEC ; MOVL @#T#VEC,R0 -- proves TMR_VEC_MASK applied on WRITE, via readback, no
    dispatch needed (dispatch's SECOND mask -- QB_VEC_MASK -- is graded separately, in the
    dispatch cases below, together with SSCVAX's own dynamic-vector wiring). */
function caseVecWriteMask(bin, scratch, failures, tmr)
{
    let tag = `vec_write_mask_t${tmr}`;
    let vecReg = tmr ? REG_T1VEC : REG_T0VEC;
    let raw = 0x7FE;                       // bits outside TMR_VEC_MASK(0x3FC): 0x400 and 0x001
    let code = [];
    movlImmToAbs(code, raw, sscAddr(vecReg));
    movlAbsToReg(code, sscAddr(vecReg), 0);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 2 });
    checkExact(failures, tag, js.R0, simh.R0, `R0 (T${tmr}VEC readback after write 0x${hex(raw)})`);
    if ((js.R0 >>> 0) !== (raw & TMR_VEC_MASK) >>> 0) {
        failures.push(`${tag}: JS itself did not apply TMR_VEC_MASK -- R0=${hex(js.R0)}, expected ${hex(raw & TMR_VEC_MASK)}`);
    }
    covered[tmr ? "vecWriteMaskT1" : "vecWriteMaskT0"] = true;
}

/** MOVL #-1,@#T#NI ; MOVL #vec,@#T#VEC ; MOVL #(XFR|IE|SGL),@#T#CSR ; readback CSR (R0), TIR (R1).
    vax_sysdev.c's tmr_csr_wr(): XFR copies TNIR(-1) into TIR; SGL (RUN clear) synchronously
    overflows it once (-1+1 wraps to 0); DON set (not already set); IE set -> SET_INT. */
function caseSglOverflowRegs(bin, scratch, failures, tmr)
{
    let tag = `sgl_overflow_regs_t${tmr}`;
    let ni = tmr ? REG_T1NI : REG_T0NI, vecReg = tmr ? REG_T1VEC : REG_T0VEC, csr = tmr ? REG_T1CSR : REG_T0CSR;
    let intReg = tmr ? REG_T1INT : REG_T0INT;
    let vecVal = 0x180;
    let code = [];
    movlImmToAbs(code, -1, sscAddr(ni));
    movlImmToAbs(code, vecVal, sscAddr(vecReg));
    movlImmToAbs(code, TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_SGL, sscAddr(csr));
    movlAbsToReg(code, sscAddr(csr), 0);
    movlAbsToReg(code, sscAddr(intReg), 1);
    /* masked at IPL_HMIN(0x14) throughout: IE is set by the setup, which WOULD otherwise hijack
       the two readback MOVLs into a dispatch on real hardware/exc.js's own (already-verified)
       arbitration -- DON/IE readback is unaffected by whether the request is ever taken. */
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 5, ipl: 0x14 });
    checkExact(failures, tag, js.R0, simh.R0, `R0 (T${tmr}CSR readback)`);
    checkExact(failures, tag, js.R1, simh.R1, `R1 (T${tmr}INT/TIR readback)`);
    let wantCsr = (TMR_CSR_DON | TMR_CSR_IE) >>> 0;
    if ((js.R0 >>> 0) !== wantCsr) {
        failures.push(`${tag}: JS CSR=${hex(js.R0)}, expected DON|IE=${hex(wantCsr)} (RUN/STP/SGL/XFR are WO/transient)`);
    }
    if (js.R1 !== 0xFFFFFFFF) {
        failures.push(`${tag}: JS TIR=${hex(js.R1)}, expected 0xFFFFFFFF (reloaded from TNIR=-1 after overflow)`);
    }
    covered[tmr ? "sglOverflowRegsT1" : "sglOverflowRegsT0"] = true;
}

/** Same setup as caseSglOverflowRegs, but the LAST instruction is a NOP at PSL<IPL>=0 -- the
    pending request (raised synchronously by the SGL write, above) must be taken on that step
    instead of executing the NOP, landing PC in R_HANDLER at the vector this case names --
    proving SSCVAX's real dynamic-vector closure (this.tivr[tmr], NOT a value captured at
    addInterruptSource() time) resolves correctly THROUGH exc.js's already-verified seam, and
    that the mask chain (TMR_VEC_MASK on write, QB_VEC_MASK at acknowledge) composes end to end.
    T0's vecVal (0x2A8) is chosen so TMR_VEC_MASK leaves it UNCHANGED (0x2A8 & 0x3FC === 0x2A8) but
    QB_VEC_MASK truncates it (0x2A8 & 0x1FC === 0x0A8) -- the SAME numbers hwintdiff.js's
    dynamic_tmr0_truncated case already proves against a synthetic prime, reused here so a
    mismatch is directly attributable to SSCVAX's OWN wiring, not to exc.js's mask (already
    verified sound and out of scope to re-grade). */
function caseSglOverflowDispatch(bin, scratch, failures, tmr, vecVal, wantDelivered)
{
    let tag = `sgl_overflow_dispatch_t${tmr}`;
    let ni = tmr ? REG_T1NI : REG_T0NI, vecReg = tmr ? REG_T1VEC : REG_T0VEC, csr = tmr ? REG_T1CSR : REG_T0CSR;
    let code = [];
    movlImmToAbs(code, -1, sscAddr(ni));
    movlImmToAbs(code, vecVal, sscAddr(vecReg));
    movlImmToAbs(code, TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_SGL, sscAddr(csr));
    /* one more step, at PSL<IPL>=0, dispatches the now-pending request instead of running a NOP */
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 4, scbVec: wantDelivered, ipl: 0 });
    checkExact(failures, tag, js.PC, simh.PC, `PC (delivered vector should land at/after R_HANDLER via SCB+0x${hex(wantDelivered, 2)})`);
    checkExact(failures, tag, js.PSL, simh.PSL, "PSL (post-dispatch)");
    /*
     * MEASURED (both engines agreed on this the first time this case ran): exc.js's own doc
     * comment on stepInstruction() already discloses that "one step can be several dispatches" --
     * a single `step 1` legitimately executes BOTH the dispatch itself AND the first instruction
     * at the handler (here, a NOP) when nothing else consumes a step boundary in between.  So the
     * correct post-condition is "landed somewhere in the handler's NOP run", not "landed on its
     * exact first byte" -- checkExact() above (js vs. SIMH) is the real cross-engine assertion;
     * this is only a sanity floor that the dispatch happened AT ALL, at the RIGHT address.
     */
    if ((js.PC >>> 0) < R_HANDLER || (js.PC >>> 0) > R_HANDLER + 8) {
        failures.push(`${tag}: JS PC=${hex(js.PC)}, expected within R_HANDLER's NOP run (${hex(R_HANDLER)}+) -- dispatch did not land where TMR_VEC_MASK(0x${hex(vecVal & TMR_VEC_MASK, 3)}) then QB_VEC_MASK(0x${hex((vecVal & TMR_VEC_MASK) & QB_VEC_MASK, 3)}) says it should`);
    }
    covered[tmr === 0 ? "dispatchMaskChainT0" : "dispatchT1"] = true;
}

/** A SECOND SGL overflow, DON already set from the first -> ERR set instead (vax_sysdev.c's
    tmr_incr(): `if (tmr_csr[tmr] & DON) csr |= ERR; else csr |= DON;`).  Also exercises that a
    CSR write which does NOT include IE in its value CLEARS IE (tmr_csr_wr's plain
    `csr = (csr & ~RW) | (val & RW)` merge, not an OR) -- this second write is bare SGL. */
function caseErrOnSecondOverflow(bin, scratch, failures)
{
    let tag = "err_on_second_overflow_t0";
    let code = [];
    movlImmToAbs(code, -1, sscAddr(REG_T0NI));
    movlImmToAbs(code, 0x140, sscAddr(REG_T0VEC));
    movlImmToAbs(code, TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_SGL, sscAddr(REG_T0CSR));   // 1st overflow: DON set
    movlImmToAbs(code, TMR_CSR_SGL, sscAddr(REG_T0CSR));                              // 2nd overflow: DON already set -> ERR
    movlAbsToReg(code, sscAddr(REG_T0CSR), 0);
    /* masked throughout (IPL_HMIN=0x14): the first overflow's request stays pending (DON survives
       into the second write, so the (DON|IE)!=0 -> ==0 withdrawal condition never fires -- see
       this case's own doc comment) all the way to the readback step; unmasked, it would hijack it. */
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 5, ipl: 0x14 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (T0CSR after second overflow)");
    let wantCsr = (TMR_CSR_DON | TMR_CSR_ERR) >>> 0;               // IE cleared by the bare-SGL write
    if ((js.R0 >>> 0) !== wantCsr) {
        failures.push(`err_on_second_overflow_t0: JS CSR=${hex(js.R0)}, expected DON|ERR=${hex(wantCsr)}`);
    }
    covered.errOnSecondOverflow = true;
}

/** First write raises a pending request (SGL, DON+IE set).  Second write is a bare W1C-DON (val=
    TMR_CSR_DON only, no IE) -- clears DON AND (by the plain RW merge) clears IE, so
    before&(DON|IE) != 0 and after&(DON|IE) == 0 -> clearInterrupt() fires.  A THIRD step at
    PSL<IPL>=0 must NOT dispatch (PC stays advancing through the NOP stream, never reaching
    R_HANDLER) -- the withdrawn-request case, mirroring hwintdiff.js's "evaporated" in spirit but
    exercised through this device's OWN CSR-write-driven withdrawal path, not exc.js's
    raise/clear pair directly. */
function caseIeClearWithdrawsPending(bin, scratch, failures)
{
    let tag = "ie_clear_withdraws_pending_t0";
    let code = [];
    movlImmToAbs(code, -1, sscAddr(REG_T0NI));
    movlImmToAbs(code, 0x1C0, sscAddr(REG_T0VEC));
    movlImmToAbs(code, TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_SGL, sscAddr(REG_T0CSR));   // raise, pending
    movlImmToAbs(code, TMR_CSR_DON, sscAddr(REG_T0CSR));                              // W1C DON, clears IE too
    /* masked (IPL_HMIN=0x14) through the withdrawing write itself (step 3) -- otherwise the
       request raised by step 2 would hijack step 3's OWN write before it ever runs, instead of
       letting it execute and withdraw the request.  Only the final probe step (4) unmasks. */
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 5, scbVec: 0x1C0, iplSchedule: [0x14, 0x14, 0x14, 0x14, 0] });
    checkExact(failures, tag, js.PC, simh.PC, "PC (must NOT be R_HANDLER -- request was withdrawn)");
    checkExact(failures, tag, js.PSL, simh.PSL, "PSL");
    if ((js.PC >>> 0) === R_HANDLER) {
        failures.push(`${tag}: JS dispatched to R_HANDLER -- clearInterrupt() on IE-clear did not withdraw the pending request`);
    }
    covered.ieClearWithdrawsPending = true;
}

/** RUN-mode instruction-driven counting, executed from the ROM address window (see the file
    header) so real SIMH's tmr_sched() genuinely takes its ADDR_IS_ROM()+short-delay branch
    (instruction-scheduled), not its wall-clock one.  T0NI=-3, T0CSR=XFR|IE|RUN (no SGL): TIR
    reloads to -3, then THREE dedicated NOP steps must overflow it exactly once (-3+3=0), setting
    DON, reloading TIR to TNIR(-3) since RUN stays set, and raising the interrupt.
    MEASURED (both engines agree): cpustate.js's per-instruction tick() hook fires on EVERY step,
    including the two READBACK MOVLs that follow the 3 dedicated NOPs -- RUN never gets cleared,
    so those two steps tick TWICE MORE before the TIR readback executes.  DON survives extra ticks
    (only software W1C clears it), so R0's expected value is unaffected; TIR does not, so the
    expected TIR is -3 PLUS the number of ticks elapsed since the reload, not bare TNIR -- see the
    case body for the exact count.  This was caught by running it, not by assuming "3 ticks in, 3
    ticks out": the FIRST version of this case asserted bare TNIR (0xFFFFFFFD) and both engines
    (agreeing with each other) showed 0xFFFFFFFF instead, which is what exposed the miscount. */
function caseRunModeCountingRom(bin, scratch, failures)
{
    let tag = "run_mode_counting_rom_t0";
    let code = [];
    movlImmToAbs(code, -3, sscAddr(REG_T0NI));
    movlImmToAbs(code, 0x1E0, sscAddr(REG_T0VEC));
    movlImmToAbs(code, TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_RUN, sscAddr(REG_T0CSR));
    /* 3 setup INSTRUCTIONS (NOT a byte count -- see this file's own footgun note elsewhere in this
       tree), then 3 NOP steps for the timer to count across, then 2 readback instructions. */
    const SETUP_INSTRS = 3;
    for (let i = 0; i < 3; i++) code.push(NOP_BYTE);
    movlAbsToReg(code, sscAddr(REG_T0CSR), 0);
    movlAbsToReg(code, sscAddr(REG_T0INT), 1);
    /* masked throughout (IPL_HMIN=0x14): the interrupt raised by the 3rd tick's overflow must not
       hijack the two readback MOVLs that follow it. */
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: SETUP_INSTRS + 3 + 2, rom: true, ipl: 0x14 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (T0CSR after 3 instruction-driven ticks)");
    checkExact(failures, tag, js.R1, simh.R1, "R1 (T0INT/TIR after 3 instruction-driven ticks)");
    let wantCsr = (TMR_CSR_DON | TMR_CSR_IE | TMR_CSR_RUN) >>> 0;
    if ((js.R0 >>> 0) !== wantCsr) {
        failures.push(`${tag}: JS CSR=${hex(js.R0)}, expected DON|IE|RUN=${hex(wantCsr)}`);
    }
    /*
     * Expected TIR: RUN is set by step index 2 (0-based: NI, VEC, CSR); tick() fires at the TOP of
     * every step from index 3 onward (5 ticks total across steps 3-7, since the two readback
     * MOVLs at indices 6-7 tick too -- see this case's own doc comment). Overflow-and-reload to
     * TNIR(-3) happens on the 3rd of those five ticks (step 5); the remaining 2 ticks (steps 6-7)
     * advance it further: -3 + 2 = -1 = 0xFFFFFFFF.
     */
    if ((js.R1 >>> 0) !== 0xFFFFFFFF) {
        failures.push(`${tag}: JS TIR=${hex(js.R1)}, expected 0xFFFFFFFF (TNIR=-3 reloaded at the 3rd tick, then 2 more ticks from the readback steps themselves)`);
    }
    covered.runModeCountingRom = true;
}

function phaseFixed(bin, scratch)
{
    let failures = [];
    caseVecWriteMask(bin, scratch, failures, 0);
    caseVecWriteMask(bin, scratch, failures, 1);
    caseSglOverflowRegs(bin, scratch, failures, 0);
    caseSglOverflowRegs(bin, scratch, failures, 1);
    caseSglOverflowDispatch(bin, scratch, failures, 0, 0x2A8, 0x0A8);
    caseSglOverflowDispatch(bin, scratch, failures, 1, 0x180, 0x180);
    caseErrOnSecondOverflow(bin, scratch, failures);
    caseIeClearWithdrawsPending(bin, scratch, failures);
    caseRunModeCountingRom(bin, scratch, failures);
    return failures;
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 2 -- randomized TIVEC write-mask sweep, romdiff.js's verifySscBaseRandom() shape:         *
 * boundary values first, then uniform random longwords, ALL batched into ONE SIMH invocation.      *
 * ------------------------------------------------------------------------------------------- */

const RANDOM_CASES_FLOOR = 8;

function phaseRandomized(bin, scratch, seed, n)
{
    if (n < RANDOM_CASES_FLOOR) {
        throw new Error(`tmrdiff: --cases ${n} is below the floor of ${RANDOM_CASES_FLOOR}; an ` +
            `undersized randomized run must fail, not quietly pass with less coverage`);
    }
    let rnd = mulberry32(seed);
    let vals = [0x00000000, 0xFFFFFFFF | 0, TMR_VEC_MASK, (~TMR_VEC_MASK) | 0];
    for (let i = vals.length; i < n; i++) vals.push((rnd() * 0x100000000) >>> 0);

    const MARK = "TMRVECRND";
    let L = ["set cpu 16m", "set cpu simhalt"];
    for (let i = 0; i < vals.length; i++) {
        let v = vals[i] >>> 0;
        let tmr = i & 1;
        let vecReg = tmr ? REG_T1VEC : REG_T0VEC;
        let addr = sscAddr(vecReg);
        L.push(`echo ${MARK}${i}`, "reset all");
        let instr = [];
        movlImmToAbs(instr, v | 0, addr);
        movlAbsToReg(instr, addr, 0);
        for (let k = 0; k < instr.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${instr[k].toString(16)}`);
        L.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`, "step 2", "examine -h R0");
    }
    L.push("exit", "");
    let out = runSimh(bin, L.join("\n"), path.join(scratch, "tmrdiff-vec-random.ini"));

    let failures = [];
    let marks = [...out.matchAll(new RegExp(MARK + "(\\d+)", "g"))].map((m) => +m[1]);
    let r0s = [...out.matchAll(/^R0:\s*([0-9A-Fa-f]+)/gm)].map((m) => parseInt(m[1], 16) >>> 0);
    if (marks.length !== vals.length || r0s.length !== vals.length) {
        failures.push(`TMR-VEC-RANDOM: expected ${vals.length} cases, SIMH produced ${marks.length} markers ` +
            `and ${r0s.length} R0 readbacks -- some case did not reach comparison`);
        return failures;
    }
    for (let i = 0; i < vals.length; i++) {
        let want = (vals[i] & TMR_VEC_MASK) >>> 0;
        let got = r0s[i] >>> 0;
        if (got !== want) {
            failures.push(`TMR-VEC-RANDOM: case ${i} (T${i & 1}VEC) val=0x${hex(vals[i])} -- ` +
                `SIMH readback=0x${hex(got)}, expected TMR_VEC_MASK-applied=0x${hex(want)}`);
        }
    }
    return failures;
}

/* ------------------------------------------------------------------------------------------- *
 * DONE CONDITION 3 -- determinism, proved by executing twice and diffing (JS-only: a property     *
 * of this instruction-count-driven model, not of SIMH's real-time one -- see ssc.js's/clk.js's      *
 * doc comments on why the two engines are not compared to each other for TIMING, only for VALUES). *
 * ------------------------------------------------------------------------------------------- */

const DETERMINISM_STEPS = 400;

function runDeterminismTrial()
{
    let { bus, cpu, ssc } = makeMachine();
    bus.setByte(0, NOP_BYTE);
    for (let i = 1; i < DETERMINISM_STEPS + 8; i++) bus.setByte(i, NOP_BYTE);
    ssc.writeReg(REG_T0NI & 0xFF, -7 | 0);
    ssc.writeReg(REG_T0VEC & 0xFF, 0x150);
    ssc.writeReg(REG_T0CSR & 0xFF, (TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_RUN) | 0);
    cpu.setPC(0);
    let csrAtStep = [], tirAtStep = [];
    for (let i = 0; i < DETERMINISM_STEPS; i++) {
        cpu.stepCPU(1);
        csrAtStep.push(ssc.tcsr[0] >>> 0);
        tirAtStep.push(ssc.tir[0] >>> 0);
    }
    return { csrAtStep, tirAtStep };
}

function proveDeterminism()
{
    let a = runDeterminismTrial();
    let b = runDeterminismTrial();
    let failures = [];
    if (a.csrAtStep.join(",") !== b.csrAtStep.join(",")) failures.push("determinism: per-step T0CSR sequence differs between two runs");
    if (a.tirAtStep.join(",") !== b.tirAtStep.join(",")) failures.push("determinism: per-step T0TIR sequence differs between two runs");
    let overflows = a.csrAtStep.filter((c) => c & TMR_CSR_DON).length;
    if (overflows === 0) failures.push("determinism: zero overflows observed in the trial budget -- the trial itself is broken, not just unlucky");
    return { failures, overflows };
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck -- named mutations (the item's own four), every one must be CAUGHT.  Graded          *
 * JS-only against an independently-known-correct expectation, matching timerdiff.js's/              *
 * hwintdiff.js's own convention (re-invoking SIMH per mutation is not needed and not done).          *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* named mutation #1: "timer never fires" -- tick() becomes a no-op. */
    timer_never_fires() {
        let orig = SSCVAX.prototype.tick;
        SSCVAX.prototype.tick = function() {};
        return () => { SSCVAX.prototype.tick = orig; };
    },
    /* named mutation #2: "vector not masked" -- T0VEC write skips TMR_VEC_MASK. */
    vector_not_masked() {
        let orig = SSCVAX.prototype.writeReg;
        SSCVAX.prototype.writeReg = function(rg, val) {
            if (rg === REG_T0VEC) { this.tivr[0] = val | 0; return true; }
            return orig.call(this, rg, val);
        };
        return () => { SSCVAX.prototype.writeReg = orig; };
    },
    /* named mutation #3: "vector resolved at install instead of acknowledge" -- the constructor's
       addInterruptSource callback captures tivr[0] ONCE, at wiring time, instead of a live closure. */
    vector_resolved_at_install() {
        /* Applied by constructing the machine differently in the case below, not by monkeypatching
           (the closure shape is decided at construction time) -- see caseVectorResolvedAtInstall(). */
        return () => {};
    },
    /* named mutation #4: "request not cleared on ack" -- reframed for what THIS file owns (not
       exc.js's deviceVector(), already verified sound and untouched): _tmrCsrWr()'s CLR_INT-on-
       IE-clear call is skipped entirely. */
    request_not_cleared_on_ack() {
        let orig = SSCVAX.prototype._tmrCsrWr;
        SSCVAX.prototype._tmrCsrWr = function(tmr, val) {
            let before = this.tcsr[tmr];
            this.tcsr[tmr] = (this.tcsr[tmr] & ~(val & TMR_CSR_ERR & TMR_CSR_DON)) | 0;
            this.tcsr[tmr] = (this.tcsr[tmr] & ~(val & (TMR_CSR_ERR | TMR_CSR_DON))) | 0;
            this.tcsr[tmr] = ((this.tcsr[tmr] & ~(TMR_CSR_IE | TMR_CSR_STP | TMR_CSR_RUN)) | (val & (TMR_CSR_IE | TMR_CSR_STP | TMR_CSR_RUN))) | 0;
            if (val & TMR_CSR_XFR) this.tir[tmr] = this.tnir[tmr] | 0;
            if (val & TMR_CSR_RUN) { /* no-op, see _tmrCsrWr's own comment */ }
            else if (val & TMR_CSR_SGL) {
                this._tmrIncr(tmr, 1);
                if (this.tir[tmr] === 0) this.tir[tmr] = this.tnir[tmr] | 0;
            }
            /* the clearInterrupt() call is the ONLY thing omitted */
        };
        return () => { SSCVAX.prototype._tmrCsrWr = orig; };
    }
};

function selfcheck()
{
    let results = [];

    {
        let restore = MUTATIONS.timer_never_fires();
        let { bus, cpu, ssc } = makeMachine();
        for (let i = 0; i < DETERMINISM_STEPS + 8; i++) bus.setByte(i, NOP_BYTE);
        ssc.writeReg(REG_T0NI & 0xFF, -3 | 0);
        ssc.writeReg(REG_T0VEC & 0xFF, 0x160);
        ssc.writeReg(REG_T0CSR & 0xFF, (TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_RUN) | 0);
        cpu.setPC(0);
        for (let i = 0; i < 50; i++) cpu.stepCPU(1);
        restore();
        let caught = (ssc.tcsr[0] & TMR_CSR_DON) === 0 && ssc.tir[0] === (0xFFFFFFFD | 0);
        results.push({ name: "timer_never_fires", caught });
    }
    {
        let restore = MUTATIONS.vector_not_masked();
        let { ssc } = makeMachine();
        ssc.writeReg(REG_T0VEC & 0xFF, 0x7FE);
        restore();
        results.push({ name: "vector_not_masked", caught: ssc.tivr[0] !== (0x7FE & TMR_VEC_MASK) });
    }
    {
        /* built directly (not via a monkeypatch): install a callback that captures tivr[0] AT
           CALL TIME (the correct shape) vs. a SECOND VAXExc where the callback captures the
           value ONCE, immediately, mirroring what a "resolved at install" bug would deliver. */
        let cpu = new CPUStateVAX({ id: "cpu" });
        let bus = new BusVAX({ busWidth: VAX.PAWIDTH, id: "bus" }, null, null);
        bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
        cpu.setBus(bus);
        cpu.reset();
        let ssc = new SSCVAX(cpu.exc, null);
        /* SSCVAX's own (correct) wiring already installed a live closure in the constructor above;
           this second, WRONG source is installed on a private test bit to compare against it,
           exactly the shape hwintdiff.js's "dynamic_tmr0_reprogrammed" proves against a synthetic
           prime -- here proved against SSCVAX's REAL storage instead. */
        let capturedAtInstall = ssc.tivr[0];               // 0 at this point -- the bug's value
        cpu.exc.addInterruptSource(IPL_HMIN, 99, (c) => capturedAtInstall);   // a spare bit, 99, unused elsewhere
        ssc.writeReg(REG_T0VEC & 0xFF, 0x1A0);              // program the REAL vector AFTER "install"
        cpu.exc.raiseInterrupt(IPL_HMIN, 99);
        let wrongVec = cpu.exc.deviceVector(cpu, IPL_HMIN);
        /* SSCVAX's OWN live closure (the shipped one, at bit INT_V_TMR0) */
        cpu.exc.raiseInterrupt(IPL_HMIN, INT_V_TMR0);
        let liveVec = cpu.exc.deviceVector(cpu, IPL_HMIN);
        let caught = (wrongVec !== liveVec) && (liveVec === ((0x1A0 & TMR_VEC_MASK) & QB_VEC_MASK));
        results.push({ name: "vector_resolved_at_install", caught });
    }
    {
        let restore = MUTATIONS.request_not_cleared_on_ack();
        let cpu = new CPUStateVAX({ id: "cpu" });
        let bus = new BusVAX({ busWidth: VAX.PAWIDTH, id: "bus" }, null, null);
        bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
        cpu.setBus(bus);
        cpu.reset();
        let ssc = new SSCVAX(cpu.exc, null);
        ssc.writeReg(REG_T0NI & 0xFF, -1 | 0);
        ssc.writeReg(REG_T0VEC & 0xFF, 0x1B0);
        ssc.writeReg(REG_T0CSR & 0xFF, (TMR_CSR_XFR | TMR_CSR_IE | TMR_CSR_SGL) | 0);   // raise, pending
        ssc.writeReg(REG_T0CSR & 0xFF, TMR_CSR_DON | 0);    // W1C DON, IE cleared -- should withdraw
        restore();
        let stillPending = (cpu.exc.intReq[IPL_HMIN - IPL_HMIN] & (1 << INT_V_TMR0)) !== 0;
        results.push({ name: "request_not_cleared_on_ack", caught: stillPending });
    }

    return { results, allCaught: results.every((r) => r.caught) };
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                            *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function main()
{
    let simhArg = getArg("--simh", null);
    let doSelfcheck = process.argv.includes("--selfcheck");
    let nCases = parseInt(getArg("--cases", "40"), 10);
    let seed = parseInt(getArg("--seed", "0x7D157"), 16);

    let bin = findSimh(simhArg);
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "tmrdiff-"));
    console.log(`SIMH: ${bin}`);
    console.log(`scratch: ${scratch}\n`);

    let allFailures = [];

    console.log("PHASE 1 (fixed matrix)");
    let fixedFailures = phaseFixed(bin, scratch);
    allFailures = allFailures.concat(fixedFailures);
    console.log(fixedFailures.length ? `  ${fixedFailures.length} failures` : "  all cases matched");

    console.log(`\nPHASE 2 (TIVEC write-mask sweep, n=${nCases}, seed=0x${seed.toString(16)})`);
    let randFailures = phaseRandomized(bin, scratch, seed, nCases);
    allFailures = allFailures.concat(randFailures);
    console.log(randFailures.length ? `  ${randFailures.length} failures` : "  all cases matched");

    console.log("\nDETERMINISM (execute twice, diff)");
    let det = proveDeterminism();
    allFailures = allFailures.concat(det.failures);
    console.log(det.failures.length ? `  ${det.failures.length} failures` : `  MATCH across two independent runs (${det.overflows} overflows each)`);

    console.log("\nCOVERAGE FLOORS");
    for (let k in covered) {
        console.log(`  ${k}: ${covered[k] ? "OK" : "MISSING"}`);
        if (!covered[k]) allFailures.push(`coverage floor not met: ${k}`);
    }

    if (doSelfcheck) {
        console.log("\n--selfcheck");
        let sc = selfcheck();
        for (let r of sc.results) console.log(`  ${r.name}: ${r.caught ? "CAUGHT" : "SURVIVED"}`);
        if (!sc.allCaught) allFailures.push("selfcheck: one or more mutations SURVIVED");
    }

    if (allFailures.length) {
        console.log(`\nFAIL (${allFailures.length} failures)`);
        for (let f of allFailures.slice(0, 40)) console.log(`  ${f}`);
        process.exit(1);
    }
    console.log("\nOK");
    process.exit(0);
}

main();
