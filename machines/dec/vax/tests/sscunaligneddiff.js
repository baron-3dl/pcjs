/**
 * @fileoverview Differential test: unaligned word(+3)/longword access spanning two adjacent
 *               SSC register-space longwords vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * ============================================================================
 * WHAT THIS IS -- AND WHY IT DOES NOT CHANGE ssc.js
 * ============================================================================
 * pcjsvax-855's own filing asked for a fix in modules/v2/ssc.js: an unaligned word(+3) or longword
 * access that reads/writes two adjacent register-space longwords, measured against the real oracle
 * (SSC+0x0C=0, SSC+0x10=0x5A5A5A5A -- MOVW @#2014000F -> 0x5A00, MOVL @#2014000D -> 0x5A000000,
 * both STITCHED across the two).  That measurement is correct.  The premise that ssc.js itself
 * needed a code change to reproduce it was NOT checked before being recorded, and is WRONG -- the
 * exact class of error HANDOFF.md's §7 exists to name.  modules/v2/mmu.js's readData()/writeData()
 * (an exact port of vax_mmu.h's Read()/Write()) are the ONE path every operand memory access in
 * this engine goes through (decode.js's own interface comment: "identical to the decoder's machine
 * interface"; grepping the whole module tree confirms nothing outside mmu.js/bus.js ever calls
 * bus.getWord()/getLong()/setWord()/setLong() directly).  For every unaligned fragment, readData()/
 * writeData() call readU()/writeU() (mmu.js), which mask `pa` to its containing ALIGNED longword
 * FIRST and do the sub-longword shift/mask/merge entirely within mmu.js -- so ssc.js's own
 * readWord()/readLong()/writeWord()/writeLong() are called by the bus ONLY at an already-aligned
 * address (word: bo 0/2; longword: bo 0).  An earlier revision of ssc.js added a two-register
 * stitch directly to those four methods, mirroring vax_mmu.h's shape literally; THIS file, run
 * through real `cpu.stepCPU()` execution (not a direct unit call to those methods), PASSED with
 * that stitch monkeypatched back to its pre-fix, single-register form -- proof, not assumption, that
 * the branches were unreachable dead code from the real CPU path.  They were removed from ssc.js
 * rather than shipped.  See ssc.js's own file header for the full account.
 *
 * So the fix this item actually needed is NOT a source change -- the two-register stitch was
 * already correct, end to end, via mmu.js -- but a REGRESSION: nothing before this item ever drove
 * a cross-longword unaligned SSC reference (per the item's own filing), so nothing would have
 * caught it if mmu.js's already-shipped stitch had ever regressed for THIS device's registers
 * specifically.  This file is that regression, graded directly against the live oracle exactly as
 * the item's own filing measured it, plus the boundary case its filing raised but left unmeasured.
 *
 * SHAPES GRADED, EACH DIRECTLY AGAINST SIMH, VIA REAL MOVB/MOVW/MOVL INSTRUCTIONS:
 *   - word bo=0/2 (aligned fast path)
 *   - word bo=1 ("read within lw", single register, bit-8 shift)
 *   - word bo=3 (crosses a longword boundary -- two registers, read AND write)
 *   - long bo=1/2/3 (crosses a longword boundary -- two registers, read AND write)
 *   - the item's own measured shape (an UNDECODED gap register adjacent to a named one, CNF/rg3)
 *   - the SSCBASE+SSCSIZE boundary: crossing OUT of the SSC device's own decoded span entirely is a
 *     REAL machine check on real hardware (mmu.js's readU()/writeU() still call bus.getLong()/
 *     setLong() at that address, which falls through makeSscController()'s own inRange()/
 *     nvrInRange() checks into the SAME readLongNone/writeLongNone fault path any undecoded address
 *     already used -- no new code needed here either) -- graded by installing an SCB.MCHK handler
 *     and confirming BOTH engines dispatch to it, not assumed from the C source.
 *   - byte accesses (regression: never cross)
 *
 * COVERAGE IS ASSERTED, NOT REPORTED -- every named floor below FAILS the run if unmet and does not
 * shrink with a smaller run (there is no --cases sweep in this file: every case is a fixed, named
 * shape, matching hwintdiff.js's/tmrdiff.js's own "PHASE 1" convention for a small, enumerated,
 * oracle-graded matrix).
 *
 * PEAK-HEAP BOUND (standing rule 14, added after a sibling differential OOM'd the host mid-swarm):
 * this file spawns at most a HANDFUL of short-lived SIMH child processes (one per case, freed
 * immediately after `execFileSync` returns) and holds no JS-side accumulator across cases, so its
 * own peak RSS should be flat regardless of which cases run.  `checkPeakHeap()` asserts an ABSOLUTE
 * bound (not a per-case scaled one) on `process.memoryUsage().heapUsed` at the end of main() --
 * FAILS the run if exceeded, exactly as every other coverage floor in this file does.  Run with
 * `NODE_OPTIONS=--max-old-space-size=2048` (or smaller) so a real leak here manifests as an OOM
 * kill, not a silently-tolerated large number.
 *
 * --selfcheck: named mutations, JS-only graded (no SIMH re-invocation per mutation -- matching
 * tmrdiff.js's own convention).  Since the logic actually under test lives in mmu.js's readU()/
 * writeU() (NOT ssc.js -- see above), the mutations monkeypatch MMUVAX.prototype at RUNTIME for the
 * duration of one check, then restore it; this is a test-time perturbation of a shipped method,
 * not a source edit, and is the same technique tmrdiff.js's own MUTATIONS use against SSCVAX.
 * EVERY mutation CALLS the real, shipped method first (`orig`) and then surgically forces only the
 * ONE effect it names -- standing rule 11 (a mutation that substitutes an independent
 * reimplementation of the code under test is idempotent, and therefore worthless, the moment the
 * shipped code is ALREADY broken the same way; tmrdiff.js's own history names three defects that
 * shipped and were "CAUGHT" by exactly that anti-pattern).
 *
 *     node machines/dec/vax/tests/sscunaligneddiff.js --simh PATH [--selfcheck]
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import MMUVAX from "../modules/v2/mmu.js";
import { OPCODES } from "../modules/v2/drom.js";
import SSCVAX, {
    SSC_BASE, SSC_LENGTH,
    REG_T0NI, REG_T0VEC
} from "../modules/v2/ssc.js";
import { SCB } from "../modules/v2/exc.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function hex(v, n = 8) { return ((v >>> 0).toString(16).toUpperCase()).padStart(n, "0"); }

/*
 * pcjsvax-855: SSC_REG_COUNT is the register-longword INDEX one past the last one vax_sysdev.c's
 * `regtable[]` entry for SSC (`{ SSCBASE, SSCBASE+SSCSIZE, &ssc_rd, &ssc_wr }`) claims -- computed
 * here, not in ssc.js, because ssc.js itself has no need of it (see this file's own header): the
 * boundary is enforced by makeSscController()'s existing inRange()/nvrInRange() fallback, reached
 * through mmu.js's readU()/writeU(), not by anything inside ssc.js's own switch.
 */
const SSC_REG_COUNT = SSC_LENGTH >>> 2;

/* ------------------------------------------------------------------------------------------- *
 * Locating SIMH -- no fixture fallback, matching every other differential in this project.        *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_SSCU_BIN']) candidates.push(process.env['SIMH_SSCU_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH microvax3900; it has no fixture fallback.  Build one\n" +
        "with machines/dec/vax/tests/simh/build.sh and pass --simh PATH or set $SIMH_SSCU_BIN.\n" +
        "Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 27, timeout: 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test -- no console/NVR: nothing graded here touches either (T0NI/T0VEC and     *
 * CNF/the rg3 gap register are entirely self-contained), mirroring tmrdiff.js's own minimal setup. *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE = 0x01000000;             // 16MB, the SIMH microvax3900 default

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let ssc = new SSCVAX(cpu.exc, null);
    bus.addSsc(ssc, null);
    cpu.tmr = ssc;
    return {bus, cpu, ssc};
}

/* ------------------------------------------------------------------------------------------- *
 * Instruction encoding -- MOVB/MOVW/MOVL, immediate-to-absolute and absolute-to-register, the      *
 * same specifier bytes tmrdiff.js's movlImmToAbs()/movlAbsToReg() use (0x8F immediate, 0x9F         *
 * absolute, 0x50|Rn register), generalized to any operand width.                                    *
 * ------------------------------------------------------------------------------------------- */

const MOVB_OPC = OPCODES.indexOf("MOVB");
const MOVW_OPC = OPCODES.indexOf("MOVW");
const MOVL_OPC = OPCODES.indexOf("MOVL");
const NOP_BYTE = OPCODES.indexOf("NOP") & 0xFF;
for (let [name, opc] of [["MOVB", MOVB_OPC], ["MOVW", MOVW_OPC], ["MOVL", MOVL_OPC]]) {
    if (opc < 0 || opc > 0xFF) throw new Error(`sscunaligneddiff: ${name} opcode not found or not single-byte`);
}

function pushByte(bytes, v) { bytes.push(v & 0xFF); }
function pushWord(bytes, v) { bytes.push(v & 0xFF, (v >>> 8) & 0xFF); }
function pushLong(bytes, v) { v = v | 0; bytes.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF); }

/** MOVx #immVal, @#absAddr -- one instruction.  `width` is 1 (MOVB), 2 (MOVW), or 4 (MOVL). */
function movImmToAbs(bytes, opc, width, immVal, absAddr)
{
    bytes.push(opc & 0xFF, 0x8F);
    if (width === 1) pushByte(bytes, immVal);
    else if (width === 2) pushWord(bytes, immVal);
    else pushLong(bytes, immVal);
    bytes.push(0x9F);
    pushLong(bytes, absAddr);
}

/** MOVx @#absAddr, Rn -- one instruction. */
function movAbsToReg(bytes, opc, absAddr, rn)
{
    bytes.push(opc & 0xFF, 0x9F);
    pushLong(bytes, absAddr);
    bytes.push(0x50 | (rn & 0xF));
}

function sscAddr(rg) { return ((SSC_BASE + rg * 4) >>> 0) | 0; }

/* ------------------------------------------------------------------------------------------- *
 * Fixed physical layout, MAPPING OFF -- same convention tmrdiff.js/hwintdiff.js use.                *
 * ------------------------------------------------------------------------------------------- */

const R_SCBB    = 0x00100000;
const R_HANDLER = 0x00102000;           // a page of NOPs; SCBB+SCB.MCHK points here for the boundary_fault case
const R_CODE    = 0x00104000;
const R_KSP     = 0x00110000;
/* pcjsvax-855: a machine check is a SEVERE exception -- vax_cpu1.c's intexc() unconditionally
   reloads SP from the INTERRUPT stack (IS), never KSP, regardless of which mode/stack was current
   -- mchkdiff.js's own file header names this exact requirement ("IS is what the frame addresses
   depend on, not whatever the noise loop put in R14").  Omitting it made a genuine machine check
   push its 5-longword frame through an unset/garbage IS, which itself faulted -- `in_ie` was
   already true when THAT happened, so real SIMH's machine_check() hit its own `if (in_ie)
   ABORT(STOP_INIE)` guard and fell all the way to a hard simulator stop ("Exception in interrupt
   or exception"), not a dispatch to R_HANDLER -- MEASURED directly while building this case: the
   very first version of caseBoundaryFault below produced exactly that stop on BOTH engines until
   IS was set. */
const R_IS      = 0x00118000;

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors.  Every one FAILS the run if unmet and does not shrink with anything.            *
 * ------------------------------------------------------------------------------------------- */

const covered = {
    wordAlignedBo0: false, wordAlignedBo2: false,
    wordOffset1: false,
    wordOffset3Cross: false, wordOffset3CrossWrite: false,
    longCrossBo1: false, longCrossBo2: false, longCrossBo3: false,
    longCrossWriteBo1: false, longCrossWriteBo2: false, longCrossWriteBo3: false,
    undecodedGapCross: false,
    boundaryFault: false,
    byteNoCrossRegression: false
};

/* ------------------------------------------------------------------------------------------- *
 * runFixedCase -- deposit code, step N times, examine.  Modeled directly on tmrdiff.js's           *
 * runFixedCase()/mchkdiff.js's own MCHK-vector convention.                                          *
 * ------------------------------------------------------------------------------------------- */

function runFixedCase(bin, scratch, name, code, opts = {})
{
    let steps = opts.steps;
    let L = ["set cpu 16m", "set cpu simhalt", "reset all"];

    L.push(`deposit MAPEN 0`);
    L.push(`deposit SCBB ${hex(R_SCBB)}`);
    L.push(`deposit KSP ${hex(R_KSP)}`);
    L.push(`deposit R14 ${hex(R_KSP)}`);
    L.push(`deposit IS ${hex(R_IS)}`);
    for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_HANDLER + k)} ${NOP_BYTE.toString(16)}`);
    if (opts.mchk) {
        L.push(`deposit -l ${hex((R_SCBB + SCB.MCHK) >>> 0)} ${hex(R_HANDLER)}`);
        /* zero the machine-check frame area before every case -- mchkdiff.js's own precedent
           (this file never reads the frame back, but a stale non-zero IS-relative stack from a
           PRIOR case in the same process is exactly the contamination that convention guards
           against; cheap to keep even though unused here). */
        let spZero = (R_IS - 20) >>> 0;
        for (let i = 0; i < 20; i += 4) L.push(`deposit -l ${hex((spZero + i) >>> 0)} 0`);
    }
    for (let i = 0; i < code.length; i++) L.push(`deposit -b ${hex((R_CODE + i) >>> 0)} ${code[i].toString(16)}`);
    for (let k = 0; k < 8; k++) L.push(`deposit -b ${hex((R_CODE + code.length + k) >>> 0)} ${NOP_BYTE.toString(16)}`);
    L.push(`deposit PSL 0`);
    L.push(`deposit PC ${hex(R_CODE)}`);
    for (let s = 0; s < steps; s++) L.push("step 1");
    L.push(`examine -h ${["R0", "R1", "R2", "PC", "PSL"].join(",")}`);
    L.push("exit");
    let out = runSimh(bin, L.join("\n") + "\n", path.join(scratch, `sscudiff-${name}.ini`));
    let simh = parseSimhExamine(out, name);

    let { bus, cpu, ssc } = makeMachine();
    cpu.exc.scbb = R_SCBB;
    cpu.regs[14] = R_KSP;
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
    for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
    if (opts.mchk) bus.setLong((R_SCBB + SCB.MCHK) >>> 0, R_HANDLER);
    for (let i = 0; i < code.length; i++) bus.setByte((R_CODE + i) >>> 0, code[i]);
    for (let k = 0; k < 8; k++) bus.setByte((R_CODE + code.length + k) >>> 0, NOP_BYTE);
    cpu.psl = 0;
    cpu.setPC(R_CODE);
    let stop = null;
    try {
        for (let s = 0; s < steps; s++) cpu.stepCPU(1);
    } catch (e) {
        stop = e;
    }
    if (stop && !opts.mchk) {
        throw new Error(`sscunaligneddiff: case ${name} raised an unexpected stop on the JS side: ${stop.message || stop}`);
    }
    let js = {
        R0: cpu.regs[0] >>> 0, R1: cpu.regs[1] >>> 0, R2: cpu.regs[2] >>> 0,
        PC: cpu.regs[15] >>> 0, PSL: cpu.psl >>> 0
    };
    return { js, simh, ssc, cpu };
}

function parseSimhExamine(out, name)
{
    let get = (label) => {
        let m = new RegExp(`^${label}:\\s*([0-9A-Fa-f]+)`, "m").exec(out);
        if (!m) throw new Error(`sscunaligneddiff: case ${name} -- SIMH did not report ${label}; output:\n${out}`);
        return parseInt(m[1], 16) >>> 0;
    };
    return { R0: get("R0"), R1: get("R1"), R2: get("R2"), PC: get("PC"), PSL: get("PSL") };
}

function checkExact(failures, tag, jsVal, simhVal, label)
{
    if ((jsVal >>> 0) !== (simhVal >>> 0)) {
        failures.push(`${tag}: ${label} js=${hex(jsVal)} simh=${hex(simhVal)}`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * FIXED MATRIX -- every case is a real instruction sequence executed on BOTH engines.               *
 * ------------------------------------------------------------------------------------------- */

/** word bo=0/2 -- ALIGNED fast path, unchanged by this item, regression only. */
function caseWordAligned(bin, scratch, failures)
{
    let tag = "word_aligned";
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0x11223344 | 0, sscAddr(REG_T0NI));
    movAbsToReg(code, MOVW_OPC, sscAddr(REG_T0NI), 0);                  // bo=0
    movAbsToReg(code, MOVW_OPC, (sscAddr(REG_T0NI) + 2) >>> 0, 1);      // bo=2
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 3 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (word bo=0)");
    checkExact(failures, tag, js.R1, simh.R1, "R1 (word bo=2)");
    covered.wordAlignedBo0 = true;
    covered.wordAlignedBo2 = true;
}

/** word bo=1 -- "read within lw", ONE register, bit-8 shift.  PREVIOUSLY WRONG (see ssc.js's file
    header): the pre-fix code reused the bo=0/2 formula (bit-0 shift) here, returning the LOW word
    instead of the middle two bytes. */
function caseWordOffset1(bin, scratch, failures)
{
    let tag = "word_offset1";
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0xA1B2C3D4 | 0, sscAddr(REG_T0NI));
    movAbsToReg(code, MOVW_OPC, (sscAddr(REG_T0NI) + 1) >>> 0, 0);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 2 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (word bo=1, distinct byte pattern 0xA1B2C3D4)");
    covered.wordOffset1 = true;
}

/** word bo=3 -- crosses into the NEXT register (REG_T0VEC, contiguous with REG_T0NI). */
function caseWordOffset3Cross(bin, scratch, failures)
{
    let tag = "word_offset3_cross";
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0x11223344 | 0, sscAddr(REG_T0NI));
    movImmToAbs(code, MOVL_OPC, 4, 0x2A8 | 0, sscAddr(REG_T0VEC));      // masked by TMR_VEC_MASK on write
    movAbsToReg(code, MOVW_OPC, (sscAddr(REG_T0NI) + 3) >>> 0, 0);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 3 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (word bo=3, crosses T0NI/T0VEC)");
    covered.wordOffset3Cross = true;
}

/** word bo=3 WRITE -- writes across T0NI's top byte and T0VEC's bottom byte, reads BOTH back
    aligned to prove the merge landed in the right register at the right byte position. */
function caseWordOffset3CrossWrite(bin, scratch, failures)
{
    let tag = "word_offset3_cross_write";
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0x11223344 | 0, sscAddr(REG_T0NI));
    movImmToAbs(code, MOVL_OPC, 4, 0x2A8 | 0, sscAddr(REG_T0VEC));
    movImmToAbs(code, MOVW_OPC, 2, 0xBEEF | 0, (sscAddr(REG_T0NI) + 3) >>> 0);
    movAbsToReg(code, MOVL_OPC, sscAddr(REG_T0NI), 0);
    movAbsToReg(code, MOVL_OPC, sscAddr(REG_T0VEC), 1);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 5 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (T0NI readback after crossing word write)");
    checkExact(failures, tag, js.R1, simh.R1, "R1 (T0VEC readback after crossing word write)");
    covered.wordOffset3CrossWrite = true;
}

/** longword bo=1/2/3 -- crosses into T0VEC, reading. */
function caseLongCross(bin, scratch, failures, bo)
{
    let tag = `long_cross_bo${bo}`;
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0x11223344 | 0, sscAddr(REG_T0NI));
    movImmToAbs(code, MOVL_OPC, 4, 0x2A8 | 0, sscAddr(REG_T0VEC));
    movAbsToReg(code, MOVL_OPC, (sscAddr(REG_T0NI) + bo) >>> 0, 0);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 3 });
    checkExact(failures, tag, js.R0, simh.R0, `R0 (long bo=${bo}, crosses T0NI/T0VEC)`);
    covered[`longCrossBo${bo}`] = true;
}

/** longword bo=1/2/3 WRITE -- writes a full pattern unaligned across T0NI/T0VEC, reads both back. */
function caseLongCrossWrite(bin, scratch, failures, bo)
{
    let tag = `long_cross_write_bo${bo}`;
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0x11223344 | 0, sscAddr(REG_T0NI));
    movImmToAbs(code, MOVL_OPC, 4, 0x2A8 | 0, sscAddr(REG_T0VEC));
    movImmToAbs(code, MOVL_OPC, 4, 0xCAFEBABE | 0, (sscAddr(REG_T0NI) + bo) >>> 0);
    movAbsToReg(code, MOVL_OPC, sscAddr(REG_T0NI), 0);
    movAbsToReg(code, MOVL_OPC, sscAddr(REG_T0VEC), 1);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 5 });
    checkExact(failures, tag, js.R0, simh.R0, `R0 (T0NI readback after long bo=${bo} write)`);
    checkExact(failures, tag, js.R1, simh.R1, `R1 (T0VEC readback after long bo=${bo} write)`);
    covered[`longCrossWriteBo${bo}`] = true;
}

/** Reproduces the item's OWN measured shape: an UNDECODED gap register (rg=3, byte offset 0x0C,
    reads 0) adjacent to a NAMED, decoded one (CNF, rg=4, byte offset 0x10). */
function caseUndecodedGapCross(bin, scratch, failures)
{
    let tag = "undecoded_gap_cross";
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0x5A5A5A5A | 0, sscAddr(4));         // REG_CNF, masked on write
    movAbsToReg(code, MOVB_OPC, (SSC_BASE + 0x0F) >>> 0, 0);            // stays within rg=3 (always 0)
    movAbsToReg(code, MOVW_OPC, (SSC_BASE + 0x0F) >>> 0, 1);            // crosses rg=3/rg=4
    movAbsToReg(code, MOVL_OPC, (SSC_BASE + 0x0D) >>> 0, 2);            // crosses rg=3/rg=4
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 4 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (byte @0x0F, stays in the undecoded rg=3)");
    checkExact(failures, tag, js.R1, simh.R1, "R1 (word @0x0F, crosses rg=3/CNF)");
    checkExact(failures, tag, js.R2, simh.R2, "R2 (long @0x0D, crosses rg=3/CNF)");
    if ((js.R0 >>> 0) !== 0) {
        failures.push(`${tag}: JS byte @0x0F=${hex(js.R0)}, expected 0 (undecoded gap register, never written)`);
    }
    covered.undecodedGapCross = true;
}

/**
 * caseBoundaryFault(bin, scratch, failures)
 *
 * The SSCBASE+SSCSIZE boundary -- see this file's own header.  The LAST valid register
 * (rg = SSC_REG_COUNT-1) crossed by an unaligned longword at bo=1 needs a SECOND register at
 * rg = SSC_REG_COUNT, which is genuinely PAST vax_sysdev.c's `regtable[]` entry for SSC
 * (`{ SSCBASE, SSCBASE+SSCSIZE, ... }`) -- real hardware machine-checks there (nothing else in
 * regtable[] claims that address; NVR's own entry starts 0x2B0 bytes further out).  Reached, on
 * this engine, through mmu.js's readU()'s `bus.getLong(pa1 & ~3)` falling through
 * makeSscController()'s inRange()/nvrInRange() checks into readLongNone -- already-shipped code,
 * nowhere touched by this item.  This is the live-oracle probe that proves the boundary is real,
 * rather than merely trusting the C source's range check.
 */
function caseBoundaryFault(bin, scratch, failures)
{
    let tag = "boundary_fault";
    let lastByteOff = (SSC_REG_COUNT - 1) * 4;
    let addr = (SSC_BASE + lastByteOff + 1) >>> 0;          // bo=1, crosses into rg=SSC_REG_COUNT
    let code = [];
    movAbsToReg(code, MOVL_OPC, addr, 0);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 1, mchk: true });
    checkExact(failures, tag, js.PC, simh.PC, "PC (must land in R_HANDLER -- a real machine check)");
    if ((js.PC >>> 0) < R_HANDLER || (js.PC >>> 0) > R_HANDLER + 8) {
        failures.push(`${tag}: JS PC=${hex(js.PC)}, expected within R_HANDLER's NOP run -- the SSC_REG_COUNT boundary did not fault`);
    }
    if ((simh.PC >>> 0) < R_HANDLER || (simh.PC >>> 0) > R_HANDLER + 8) {
        failures.push(`${tag}: SIMH PC=${hex(simh.PC)} did NOT dispatch to R_HANDLER either -- this case's own premise (a real machine check at this boundary) is WRONG; see ssc.js's file header, which must be corrected if this fires`);
    }
    covered.boundaryFault = true;
}

/** Byte accesses never cross a register -- regression only, unchanged by this item. */
function caseByteNoCrossRegression(bin, scratch, failures)
{
    let tag = "byte_no_cross_regression";
    let code = [];
    movImmToAbs(code, MOVL_OPC, 4, 0xA1B2C3D4 | 0, sscAddr(REG_T0NI));
    movAbsToReg(code, MOVB_OPC, sscAddr(REG_T0NI), 0);
    movAbsToReg(code, MOVB_OPC, (sscAddr(REG_T0NI) + 1) >>> 0, 1);
    movAbsToReg(code, MOVB_OPC, (sscAddr(REG_T0NI) + 2) >>> 0, 2);
    let { js, simh } = runFixedCase(bin, scratch, tag, code, { steps: 4 });
    checkExact(failures, tag, js.R0, simh.R0, "R0 (byte bo=0)");
    checkExact(failures, tag, js.R1, simh.R1, "R1 (byte bo=1)");
    checkExact(failures, tag, js.R2, simh.R2, "R2 (byte bo=2)");
    covered.byteNoCrossRegression = true;
}

function phaseFixed(bin, scratch)
{
    let failures = [];
    caseWordAligned(bin, scratch, failures);
    caseWordOffset1(bin, scratch, failures);
    caseWordOffset3Cross(bin, scratch, failures);
    caseWordOffset3CrossWrite(bin, scratch, failures);
    for (let bo of [1, 2, 3]) caseLongCross(bin, scratch, failures, bo);
    for (let bo of [1, 2, 3]) caseLongCrossWrite(bin, scratch, failures, bo);
    caseUndecodedGapCross(bin, scratch, failures);
    caseBoundaryFault(bin, scratch, failures);
    caseByteNoCrossRegression(bin, scratch, failures);
    return failures;
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck -- named mutations.  Graded JS-only against a known-correct expectation, matching    *
 * tmrdiff.js's/hwintdiff.js's own convention (re-invoking SIMH per mutation is not needed).          *
 * EVERY mutation calls the real, shipped method (`orig`) first -- standing rule 11.                 *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* #1: word bo=1 ("read within lw") reverts to the PRE-FIX symptom this project actually shipped
       once (bit-0 shift, i.e. the low word, instead of the bit-8 shift real hardware's ReadU(pa,
       L_WORD) uses).  The logic under test is MMUVAX.prototype.readU (mmu.js), NOT ssc.js -- see
       this file's own header.  Composes over `this.bus.getLong()` (an UNPERTURBED dependency) to
       recompute the specific wrong extraction, rather than reusing `real` (already correctly
       shifted, and not reversible into the wrong shift without re-reading the source). */
    word_offset1_reverted() {
        let orig = MMUVAX.prototype.readU;
        MMUVAX.prototype.readU = function(pa, lnt) {
            let real = orig.call(this, pa, lnt);
            if ((pa & 3) === 1 && lnt === 2) {
                let full = this.bus.getLong(pa & ~0x03);
                return full & 0xFFFF;                     // the pre-fix formula (shift 0, not 8)
            }
            return real;
        };
        return () => { MMUVAX.prototype.readU = orig; };
    },
    /* #2: word bo=3 stops stitching -- reverts to a single-register read (the containing aligned
       longword's top 16 bits only), ignoring the SECOND register readData()'s own wl/wh combine
       needs.  readData() (not readU()) is what actually stitches a word-crossing access (two
       separate readU(pa,L_BYTE)/readU(pa1,L_BYTE) calls combined at the caller), so THIS mutation
       composes over readData() -- calling `orig` first, perturbing only the lnt===2/bo===3 case,
       via `this.bus.getLong()` (unperturbed) rather than reusing `real`. Scoped to mapen===0,
       matching every case in this file (MAPEN 0 throughout, per runFixedCase()). */
    word_offset3_no_stitch() {
        let orig = MMUVAX.prototype.readData;
        MMUVAX.prototype.readData = function(va, lnt, acc) {
            let real = orig.call(this, va, lnt, acc);
            if (!this.mapen && lnt === 2) {
                let pa = va & VAX.PAMASK;
                if ((pa & 3) === 3) {
                    let full = this.bus.getLong(pa & ~0x03);
                    return (full >>> 16) & 0xFFFF;         // the pre-fix formula: single register
                }
            }
            return real;
        };
        return () => { MMUVAX.prototype.readData = orig; };
    },
    /* #3: any unaligned longword read stops stitching entirely -- reverts to a bare read of the
       FIRST containing aligned longword, discarding the second register's contribution.  Same
       composition shape as #2. */
    long_unaligned_no_stitch() {
        let orig = MMUVAX.prototype.readData;
        MMUVAX.prototype.readData = function(va, lnt, acc) {
            let real = orig.call(this, va, lnt, acc);
            if (!this.mapen && lnt === 4) {
                let pa = va & VAX.PAMASK;
                if ((pa & 3) !== 0) {
                    return this.bus.getLong(pa & ~0x03) | 0;   // the pre-fix formula: single register
                }
            }
            return real;
        };
        return () => { MMUVAX.prototype.readData = orig; };
    },
    /* #4: the SSCBASE+SSCSIZE boundary stops faulting -- the mechanism that actually enforces it
       (MemoryVAX.prototype.readLongNone, reached through makeSscController()'s own inRange()/
       nvrInRange() fallback, none of it touched by this item) is forced to succeed silently (return
       0, no bus.fault() call) instead of signaling the real machine check -- the exact false-
       negative shape a regression there would produce. */
    boundary_check_removed() {
        let orig = MemoryVAX.prototype.readLongNone;
        MemoryVAX.prototype.readLongNone = function(off, addr) {
            /* deliberately do NOT call orig -- calling it would raise the real fault this mutation
               exists to suppress; the perturbation IS the omission, not a wrapped call. */
            return 0;
        };
        return () => { MemoryVAX.prototype.readLongNone = orig; };
    }
};

function selfcheck()
{
    let results = [];

    {
        let restore = MUTATIONS.word_offset1_reverted();
        let { bus, cpu } = makeMachine();
        cpu.mmu.writeData(sscAddr(REG_T0NI), 0xA1B2C3D4 | 0, 4, cpu.accW());
        let v = cpu.mmu.readData((sscAddr(REG_T0NI) + 1) >>> 0, 2, cpu.accR());
        restore();
        /* correct answer: (0xA1B2C3D4 >>> 8) & 0xFFFF = 0xB2C3; the reverted formula instead
           yields (0xA1B2C3D4) & 0xFFFF = 0xC3D4. */
        results.push({ name: "word_offset1_reverted", caught: v !== 0xB2C3 && v === 0xC3D4 });
    }
    {
        let restore = MUTATIONS.word_offset3_no_stitch();
        let { bus, cpu } = makeMachine();
        cpu.mmu.writeData(sscAddr(REG_T0NI), 0x11223344 | 0, 4, cpu.accW());
        cpu.mmu.writeData(sscAddr(REG_T0VEC), 0xA8 | 0, 4, cpu.accW());
        let v = cpu.mmu.readData((sscAddr(REG_T0NI) + 3) >>> 0, 2, cpu.accR());
        restore();
        /* correct (stitched) answer: wl=0x11 (T0NI byte3), wh=0xA8 (T0VEC byte0) -> 0xA811.
           reverted (single-register) answer: (0x11223344 >>> 16) & 0xFFFF = 0x1122. */
        results.push({ name: "word_offset3_no_stitch", caught: v !== 0xA811 && v === 0x1122 });
    }
    {
        let restore = MUTATIONS.long_unaligned_no_stitch();
        let { bus, cpu } = makeMachine();
        cpu.mmu.writeData(sscAddr(REG_T0NI), 0x11223344 | 0, 4, cpu.accW());
        cpu.mmu.writeData(sscAddr(REG_T0VEC), 0xA8 | 0, 4, cpu.accW());
        let v = cpu.mmu.readData((sscAddr(REG_T0NI) + 1) >>> 0, 4, cpu.accR());
        restore();
        /* correct (stitched) answer for bo=1: 0xA8112233 (see this file's own header worked
           example).  reverted (no-stitch) answer: bare read of the first register, 0x11223344. */
        results.push({ name: "long_unaligned_no_stitch", caught: (v >>> 0) !== 0xA8112233 && (v >>> 0) === 0x11223344 });
    }
    {
        let restore = MUTATIONS.boundary_check_removed();
        let { bus, cpu } = makeMachine();
        let lastByteOff = (SSC_REG_COUNT - 1) * 4;
        let addr = (SSC_BASE + lastByteOff + 1) >>> 0;      // bo=1, crosses into rg=SSC_REG_COUNT
        let faulted = false;
        try { cpu.mmu.readData(addr, 4, cpu.accR()); } catch (e) { faulted = true; }
        restore();
        /* correct behavior: the read FAULTS (a bus fault propagates as a thrown VAXFault -- see
           excdiff.js's own convention for physical bus faults).  With readLongNone silenced, no
           fault is thrown and the read silently "succeeds". */
        results.push({ name: "boundary_check_removed", caught: !faulted });
    }

    return { results, allCaught: results.every((r) => r.caught) };
}

/* ------------------------------------------------------------------------------------------- *
 * Peak-heap bound (standing rule 14) -- an ABSOLUTE ceiling, not scaled by case count.              *
 * ------------------------------------------------------------------------------------------- */

const PEAK_HEAP_BOUND_BYTES = 400 * 1024 * 1024;    // 400MB: generous for a handful of short-lived
                                                      // execFileSync calls with no cross-case state
function checkPeakHeap(failures)
{
    let used = process.memoryUsage().heapUsed;
    console.log(`  heapUsed at end of run: ${(used / (1024 * 1024)).toFixed(1)} MB (bound: ${(PEAK_HEAP_BOUND_BYTES / (1024 * 1024)).toFixed(0)} MB)`);
    if (used > PEAK_HEAP_BOUND_BYTES) {
        failures.push(`peak heap ${(used / (1024 * 1024)).toFixed(1)} MB exceeds the ${(PEAK_HEAP_BOUND_BYTES / (1024 * 1024)).toFixed(0)} MB bound -- this file should hold no cross-case accumulator`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * main                                                                                            *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function main()
{
    let simhArg = getArg("--simh", null);
    let doSelfcheck = process.argv.includes("--selfcheck");

    let bin = findSimh(simhArg);
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "sscudiff-"));
    console.log(`SIMH: ${bin}`);
    console.log(`scratch: ${scratch}\n`);

    let allFailures = [];

    /* Wraps the run so an uncaught exception (e.g. from phaseFixed()/selfcheck()) still removes
       scratch -- the pre-existing cleanup below already ran on every NORMAL exit path (opt-in
       retained only via VAX_SSCUDIFF_KEEP), but an exception thrown before reaching it bypassed
       that line entirely (HANDOFF.md pcjsvax-bd1). */
    try {

    console.log("PHASE 1 (fixed matrix)");
    let fixedFailures = phaseFixed(bin, scratch);
    allFailures = allFailures.concat(fixedFailures);
    console.log(fixedFailures.length ? `  ${fixedFailures.length} failures` : "  all cases matched");

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

    console.log("\nPEAK HEAP");
    checkPeakHeap(allFailures);

    } finally {
        if (!process.env["VAX_SSCUDIFF_KEEP"]) fs.rmSync(scratch, {recursive: true, force: true});
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
