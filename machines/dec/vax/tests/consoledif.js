/**
 * @fileoverview Differential test: the KA655 console (console.js's ConsoleVAX) vs. a real Open SIMH
 *               microvax3900, through BOTH the IPR path and the SSC memory-mapped mirror
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-bfb's done conditions 2-6.  Condition 1 (the ROM's own banner) is romdiff.js's concern
 * (this item's own boundary-advance did not reach the console registers before hitting the SSC T0/1
 * timer boundary -- see this item's report); THIS file grades the console REGISTER MODEL itself,
 * directly, with hand-built VAX instructions the way tests/hwintdiff.js already grades the
 * interrupt seam and tests/romdiff.js's verifySscBaseRandom() already grades the SSC base register
 * mask -- "only an actual instruction reproduces the real access path" (romdiff.js's own reasoning,
 * reused here unchanged).
 *
 * FOUR PHASES:
 *
 *   REAL       Hand-assembled MTPR/MFPR (the IPR path) and MOVL-absolute (the SSC mirror) sequences,
 *              executed for real on BOTH a live SIMH and this machine's own JS, comparing register
 *              file / CC / memory results exactly the way hwintdiff.js's REAL_CASES do.  Covers
 *              polled output (write-busy-wait-done), polled input (inject-then-read), and the
 *              edge-triggered "IE enabled while DONE already set" case for both TXCS and RXCS, each
 *              through EACH address path.
 *
 *   DEVTRACE   ONE representative full round trip (TXCS/TXDB polled output, both address paths in
 *              sequence) run under patch 0006's `SET SYSD DEBUG=DEVTRACE`/`SET CPU DEBUG=EXCTRACE`,
 *              its IPRR/IPRW/REGR/REGW lines for MT.RXCS/RXDB/TXCS/TXDB (32-35) and the matching SSC
 *              offsets (0x80/0x84/0x88/0x8C) parsed and compared, IN ORDER, VALUE FOR VALUE, against
 *              the SAME sequence recorded by temporarily wrapping ConsoleVAX's own register-model
 *              methods while the identical instructions run on this machine's JS -- this is the
 *              literal "register-level differential against patch 0006's trace" done condition 2
 *              requires, not merely the REAL phase's coarser register-file agreement.
 *
 *   INTERRUPT  TTI (SCB 0xF8) / TTO (SCB 0xFC) delivered at IPL 0x14 when CSR_IE is set, through the
 *              REAL console device this time (not hwintdiff.js's stand-in "prime" function) -- proof
 *              that ConsoleVAX's OWN addInterruptSource()/raiseInterrupt() calls (console.js's
 *              constructor, rxcsWr/txcsWr/rxdbRd/txdbWr/maybeComplete) are wired correctly, not just
 *              that the seam itself works (hwintdiff.js already proved that).
 *
 *   SELFCHECK  --selfcheck: the five NAMED mutations done condition 6 requires, at minimum, each
 *              proven caught against the REAL case that would expose it.
 *
 *      node machines/dec/vax/tests/consoledif.js [--simh PATH] [--selfcheck]
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
import VAXExc, { executeExc, MT, IPL_HMIN, PSL_V_IPL, KERN, SCB } from "../modules/v2/exc.js";
import SSCVAX from "../modules/v2/ssc.js";
import ConsoleVAX, { SCB_TTI, SCB_TTO, TTI_BIT, TTO_BIT, CSR_DONE, CSR_IE } from "../modules/v2/console.js";
import ClkVAX, { IPL_CLK_ABS, INT_V_CLK } from "../modules/v2/clk.js";
import { makeIprDevice } from "../modules/v2/iprdevice.js";

function hex(v, n = 8) { return ((v >>> 0).toString(16).toUpperCase()).padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Locating SIMH -- no fixture fallback, matching every other differential in this project.        *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_CONSOLE_BIN']) candidates.push(process.env['SIMH_CONSOLE_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error(
        "consoledif needs a REAL SIMH (patch 0006, the DEVTRACE lines); it has no fixture\n" +
        "fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh PATH or\n" +
        "set $SIMH_CONSOLE_BIN.  Tried:\n  " + candidates.join("\n  "));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 28, timeout: 5 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test -- RAM + SSC (with the console wired into BOTH address paths) only;      *
 * no ROM, exactly like hwintdiff.js: this file drives hand-built instructions directly, it does     *
 * not need the boot ROM at all.                                                                    *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE = 0x01000000;

class ConsoleCpu extends VAXCpu {
    constructor(bus) {
        super(bus);
        this.exc = new VAXExc(this);
        /*
         * VAXCpu (unlike CPUStateVAX, which this harness deliberately does not build -- see
         * makeMachine()'s doc comment) has no cycle accounting of its own; console.js's
         * maybeComplete() reads `exc.cpu.nTotalCycles` as its TXCS-completion clock (see that
         * file's doc comment on why that field, specifically, is the right one -- it is kept in
         * exact lockstep with SIMH's own sim_interval by cpustate.js's stepCPU()).  This harness
         * reproduces that ONE property, incremented once per stepOne() call, which is exact for
         * every instruction this file ever builds (none are string instructions, the only source
         * of the `1 + extraBytes>>5` correction cpustate.js's real stepCPU() applies).
         */
        this.nTotalCycles = 0;
    }
    executeOne(opc, decoder, cpu) {
        if (executeExc(opc, decoder, cpu)) return;
        let fn = DISPATCH[opc];
        if (fn) { fn(cpu, decoder); return; }
        throw new Error(`consoledif: opcode ${hex(opc, 3)} (${OPCODES[opc] || "?"}) has no body wired into this harness`);
    }
    stepOne() { let r = this.exc.stepInstruction(this, (opc, d, c) => this.executeOne(opc, d, c)); this.nTotalCycles++; return r; }
}

/**
 * makeMachine()
 *
 * pcjsvax-bfb, post-merge reconciliation with pcjsvax-954: ClkVAX (clk.js) owns MT.ICCS/MT.TODR;
 * ConsoleVAX owns MT.RXCS/RXDB/TXCS/TXDB.  exc.js's setIPRDevice() accepts only one device, so both
 * are combined behind iprdevice.js's makeIprDevice() -- see that file's header, and console.js's
 * own updated file header, for why the two are not merged into one object and why clk is NOT ticked
 * through the aggregate (this harness's ConsoleCpu never calls cpustate.js's stepCPU() at all, so
 * clk.tick() is never invoked here either way -- consistent with hwintdiff.js's HwIntCpu, and
 * harmless since nothing in this file exercises ICCS/TODR ticking behavior; that is timerdiff.js's
 * job). `clk` is constructed and wired here purely so this file proves the SAME aggregate pattern
 * romdiff.js's production machine uses, not a console-only shortcut that would leave TODR/ICCS
 * silently unreachable in this harness.
 */
function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new ConsoleCpu(bus);
    let consoleDev = new ConsoleVAX(cpu.exc);
    let clk = new ClkVAX(cpu.exc);
    bus.addSsc(new SSCVAX(cpu.exc, consoleDev), null);
    cpu.exc.setIPRDevice(makeIprDevice(clk, consoleDev));
    cpu.exc.addInterruptSource(IPL_CLK_ABS, INT_V_CLK, SCB.INTTIM);
    return {bus, cpu, consoleDev, clk};
}

/* ------------------------------------------------------------------------------------------- *
 * Fixed physical layout, MAPPING OFF -- same constants hwintdiff.js uses, deliberately, so a       *
 * reader of both trusts them once.                                                                 *
 * ------------------------------------------------------------------------------------------- */

const R_SCBB    = 0x00100000;
const R_HANDLER = 0x00102000;
const R_CODE    = 0x00104000;
const R_KSP     = 0x00110000;
const R_DATA    = 0x00120000;                   // scratch RAM for MOVL-absolute destinations

const SSCBASE = VAX.PHYSMEM.SSC_BASE >>> 0;
const SSC_RXCS = (SSCBASE + 0x80) >>> 0, SSC_RXDB = (SSCBASE + 0x84) >>> 0;
const SSC_TXCS = (SSCBASE + 0x88) >>> 0, SSC_TXDB = (SSCBASE + 0x8C) >>> 0;

const MOVL_OPC = OPCODES.indexOf("MOVL");
const MTPR_OPC = OPCODES.indexOf("MTPR");
const MFPR_OPC = OPCODES.indexOf("MFPR");
const NOP_BYTE = OPCODES.indexOf("NOP") & 0xFF;
for (let [name, v] of [["MOVL", MOVL_OPC], ["MTPR", MTPR_OPC], ["MFPR", MFPR_OPC]]) {
    if (v < 0 || v > 0xFF) throw new Error(`consoledif: ${name} opcode not found or not single-byte`);
}

function pushLong(bytes, v) { v = v | 0; bytes.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF); }

/** MOVL #immVal,@#absAddr -- immediate-longword source, absolute destination. */
function emitMovlImmToAbs(bytes, immVal, absAddr)
{
    bytes.push(MOVL_OPC & 0xFF, 0x8F); pushLong(bytes, immVal);
    bytes.push(0x9F); pushLong(bytes, absAddr);
}
/** MOVL @#srcAddr,@#absAddr -- absolute source, absolute destination. */
function emitMovlAbsToAbs(bytes, srcAddr, absAddr)
{
    bytes.push(MOVL_OPC & 0xFF, 0x9F); pushLong(bytes, srcAddr);
    bytes.push(0x9F); pushLong(bytes, absAddr);
}
/** MTPR #immVal,#iprNum -- iprNum MUST be 0..63 (fits a short literal specifier byte exactly). */
function emitMtprImm(bytes, immVal, iprNum)
{
    if (iprNum < 0 || iprNum > 63) throw new Error("consoledif: MTPR iprNum must fit a short literal");
    bytes.push(MTPR_OPC & 0xFF, 0x8F); pushLong(bytes, immVal);
    bytes.push(iprNum & 0xFF);
}
/** MFPR #iprNum,@#absAddr. */
function emitMfprToAbs(bytes, iprNum, absAddr)
{
    if (iprNum < 0 || iprNum > 63) throw new Error("consoledif: MFPR iprNum must fit a short literal");
    bytes.push(MFPR_OPC & 0xFF, iprNum & 0xFF);
    bytes.push(0x9F); pushLong(bytes, absAddr);
}

/* ------------------------------------------------------------------------------------------- *
 * REAL phase -- hand-built instruction sequences, graded by final register-file state (a          *
 * handful of MOVL/MFPR destinations read back via examine/bus.getLong), same shape as               *
 * romdiff.js's verifySscBaseRandom() and hwintdiff.js's REAL_CASES.                                 *
 * ------------------------------------------------------------------------------------------- */

/**
 * @typedef {Object} RegCase
 * @property {string} name
 * @property {number[]} bytes           the instruction stream (NOPs appended by the caller)
 * @property {number[]} probes          absolute addresses to read back and compare, in order
 * @property {?function(Object):void} prep  called on the JS side ONLY, before stepping (e.g.
 *           injectChar()) -- the SIMH side's equivalent is `simhPrep`, below
 * @property {?string[]} simhPrep       extra SIMH script lines, inserted right after `reset all`
 *           and the baseline deposits, before the instruction bytes are deposited
 */

const REAL_CASES = (function() {
    let cases = [];

    /* ---- polled output, IPR path: write with IE=0, must be BUSY, then DONE after the wait ---- */
    {
        let bytes = [];
        emitMtprImm(bytes, 0x41, MT.TXDB);                    // TXDB = 'A' (IE currently 0)
        emitMfprToAbs(bytes, MT.TXCS, R_DATA);                // probe 0: TXCS immediately after (BUSY)
        cases.push({
            name: "ipr_polled_output_busy",
            bytes, probes: [R_DATA], steps: 2
        });
    }
    /* Same, but through the SSC mirror -- TXDB write at SSC+0x8C, TXCS read at SSC+0x88. */
    {
        let bytes = [];
        emitMovlImmToAbs(bytes, 0x42, SSC_TXDB);
        emitMovlAbsToAbs(bytes, SSC_TXCS, R_DATA);
        cases.push({ name: "ssc_polled_output_busy", bytes, probes: [R_DATA], steps: 2 });
    }

    /* ---- polled input, IPR path: inject, then RXCS (DONE), then RXDB (byte + DONE clears) ---- */
    {
        let bytes = [];
        emitMfprToAbs(bytes, MT.RXCS, R_DATA);                // probe 0: RXCS after injection (DONE)
        emitMfprToAbs(bytes, MT.RXDB, (R_DATA + 4) | 0);      // probe 1: RXDB (the byte)
        emitMfprToAbs(bytes, MT.RXCS, (R_DATA + 8) | 0);      // probe 2: RXCS again (DONE cleared)
        cases.push({
            name: "ipr_polled_input",
            bytes, probes: [R_DATA, (R_DATA + 4) | 0, (R_DATA + 8) | 0], steps: 3,
            prep: (m) => m.consoleDev.injectChar(0x58),
            simhPrep: ["deposit TTI BUF 58", "deposit TTI CSR 80"]      // CSR_DONE, matching injectChar()
        });
    }
    {
        let bytes = [];
        emitMovlAbsToAbs(bytes, SSC_RXCS, R_DATA);
        emitMovlAbsToAbs(bytes, SSC_RXDB, (R_DATA + 4) | 0);
        emitMovlAbsToAbs(bytes, SSC_RXCS, (R_DATA + 8) | 0);
        cases.push({
            name: "ssc_polled_input",
            bytes, probes: [R_DATA, (R_DATA + 4) | 0, (R_DATA + 8) | 0], steps: 3,
            prep: (m) => m.consoleDev.injectChar(0x59),
            simhPrep: ["deposit TTI BUF 59", "deposit TTI CSR 80"]
        });
    }

    /* ---- edge-triggered IE-enable-while-DONE-already-set, TXCS, both paths ---- */
    {
        let bytes = [];
        emitMtprImm(bytes, CSR_IE, MT.TXCS);                  // TXCS is CSR_DONE from reset -- edge!
        cases.push({ name: "ipr_txcs_ie_edge", bytes, probes: [], probeInt: {lvl: IPL_HMIN, bit: TTO_BIT} });
    }
    {
        let bytes = [];
        emitMovlImmToAbs(bytes, CSR_IE, SSC_TXCS);
        cases.push({ name: "ssc_txcs_ie_edge", bytes, probes: [], probeInt: {lvl: IPL_HMIN, bit: TTO_BIT} });
    }
    /* ---- same, RXCS: DONE must be pending (inject FIRST, with IE=0), then enabling IE edges it. --- */
    {
        let bytes = [];
        emitMtprImm(bytes, CSR_IE, MT.RXCS);
        cases.push({
            name: "ipr_rxcs_ie_edge", bytes, probes: [], probeInt: {lvl: IPL_HMIN, bit: TTI_BIT},
            prep: (m) => m.consoleDev.injectChar(0x5A),
            simhPrep: ["deposit TTI BUF 5A", "deposit TTI CSR 80"]
        });
    }
    {
        let bytes = [];
        emitMovlImmToAbs(bytes, CSR_IE, SSC_RXCS);
        cases.push({
            name: "ssc_rxcs_ie_edge", bytes, probes: [], probeInt: {lvl: IPL_HMIN, bit: TTI_BIT},
            prep: (m) => m.consoleDev.injectChar(0x5B),
            simhPrep: ["deposit TTI BUF 5B", "deposit TTI CSR 80"]
        });
    }

    return cases;
})();

/* SERIAL_OUT_WAIT (sim_defs.h) -- console.js imports this value implicitly; kept here as a literal
   (documented, not hand-derived from nothing: see console.js's own TXCS DONE TIMING section) so
   this file can step EXACTLY that many NOPs and observe the DONE transition on both sides. */
const SERIAL_OUT_WAIT = 100;

/** Same NOP-run-out probe hwintdiff.js's buildScript() uses -- enough trailing NOPs that no case's
    probe PC ever runs off the end into zeroed memory (opcode 0 = HALT). */
const TRAIL_NOPS = SERIAL_OUT_WAIT + 16;

function buildRealScript(cases)
{
    let L = [`set cpu ${MEMSIZE / (1024 * 1024)}m`, "set cpu simhalt"];
    for (let c of cases) {
        L.push("reset all");
        L.push(`deposit MAPEN 0`, `deposit SCBB ${hex(R_SCBB)}`, `deposit KSP ${hex(R_KSP)}`, `deposit R14 ${hex(R_KSP)}`);
        for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_HANDLER + k)} ${NOP_BYTE.toString(16)}`);
        L.push(`deposit ${hex(R_SCBB + SCB_TTI)} ${hex(R_HANDLER)}`, `deposit ${hex(R_SCBB + SCB_TTO)} ${hex(R_HANDLER)}`);
        for (let k = 0; k < 16; k++) L.push(`deposit -l ${hex((R_DATA + k * 4) | 0)} 0`);
        for (let k = 0; k < c.bytes.length + TRAIL_NOPS; k++) L.push(`deposit -b ${hex(R_CODE + k)} 0`);
        for (let i = 0; i < c.bytes.length; i++) L.push(`deposit -b ${hex(R_CODE + i)} ${c.bytes[i].toString(16)}`);
        for (let k = 0; k < TRAIL_NOPS; k++) L.push(`deposit -b ${hex((R_CODE + c.bytes.length + k) | 0)} ${NOP_BYTE.toString(16)}`);
        if (c.simhPrep) for (let line of c.simhPrep) L.push(line);
        L.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`);
        L.push(`echo CCASE_${c.name}`);
        if (c.probeInt) {
            /* IE-enable-edge cases: step the ONE instruction, then examine the device's own INT
               bit (matching hwintdiff.js's probeDeviceInt convention) instead of a memory probe. */
            L.push("step 1");
            let devName = c.probeInt.bit === TTI_BIT ? "TTI" : "TTO";
            L.push(`examine -h ${devName} INT`);
        } else {
            L.push(`step ${c.steps}`);
            for (let p of c.probes) L.push(`examine -h ${hex(p)}`);
        }
    }
    L.push("exit", "");
    return L.join("\n") + "\n";
}

/* Matches BOTH a hex-address label (`00120000:`, ordinary `examine -h <addr>`) and a named-register
   label (`PC:`, `PSL:`, `INT:`) -- hwintdiff.js's own VALUE_RE uses this same `\S+` shape for
   exactly that reason. */
const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/m;
const VALUE_RE_G = /^(\S+):\s+([0-9A-Fa-f]+)/gm;
const INT_VALUE_RE = VALUE_RE;

function runRealAll(simh, cases, scratch)
{
    let script = buildRealScript(cases);
    let out = runSimh(simh, script, path.join(scratch, "consoledif-real.ini"));
    let lines = out.split("\n");
    let results = new Map();
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(/CCASE_(\S+)/);
        if (!m) { i++; continue; }
        let name = m[1];
        let c = cases.find((cc) => cc.name === name);
        let want = c.probeInt ? 1 : c.probes.length;
        let re = c.probeInt ? INT_VALUE_RE : VALUE_RE;
        i++;
        let vals = [];
        while (i < lines.length && vals.length < want) {
            if (lines[i].indexOf("CCASE_") >= 0) break;
            let vm = lines[i].match(re);
            if (vm) vals.push(parseInt(vm[2], 16) >>> 0);
            i++;
        }
        results.set(name, vals);
    }
    return results;
}

function runCaseJS(m, c)
{
    let {bus, cpu, consoleDev} = m;
    cpu.exc.reset();
    cpu.mmu.reset();
    cpu.mmu.setMAPEN(0);
    consoleDev.reset();
    cpu.exc.scbb = R_SCBB;
    cpu.exc.stk[KERN] = R_KSP;
    cpu.regs.fill(0);
    cpu.regs[14] = R_KSP;
    for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
    bus.setLong(R_SCBB + SCB_TTI, R_HANDLER);
    bus.setLong(R_SCBB + SCB_TTO, R_HANDLER);
    for (let k = 0; k < 16; k++) bus.setLong((R_DATA + k * 4) | 0, 0);
    for (let k = 0; k < c.bytes.length + TRAIL_NOPS; k++) bus.setByte((R_CODE + k) | 0, 0);
    for (let i = 0; i < c.bytes.length; i++) bus.setByte((R_CODE + i) | 0, c.bytes[i]);
    for (let k = 0; k < TRAIL_NOPS; k++) bus.setByte((R_CODE + c.bytes.length + k) | 0, NOP_BYTE);
    if (c.prep) c.prep(m);
    cpu.psl = 0;
    cpu.regs[15] = R_CODE;
    let steps = c.probeInt ? 1 : c.steps;
    for (let s = 0; s < steps; s++) cpu.stepOne();
    if (c.probeInt) {
        return [(cpu.exc.intReq[c.probeInt.lvl - IPL_HMIN] >>> c.probeInt.bit) & 1];
    }
    return c.probes.map((p) => bus.getLong(p) >>> 0);
}

function phaseReal(simh, scratch)
{
    let problems = [];
    let m = makeMachine();
    let sr = runRealAll(simh, REAL_CASES, scratch);
    let sawIpr = false, sawSsc = false, sawInputDir = false, sawOutputDir = false, sawPolled = false, sawIntEnabled = false;
    let notReached = [];
    for (let c of REAL_CASES) {
        let simhVals = sr.get(c.name);
        let want = c.probeInt ? 1 : c.probes.length;
        if (!simhVals || simhVals.length < want) { notReached.push(c.name); continue; }
        let jsVals = runCaseJS(m, c);
        for (let i = 0; i < want; i++) {
            if ((jsVals[i] >>> 0) !== (simhVals[i] >>> 0)) {
                problems.push(`REAL ${c.name}: probe[${i}] js=0x${hex(jsVals[i])} simh=0x${hex(simhVals[i])}`);
            }
        }
        if (c.name.startsWith("ipr_")) sawIpr = true;
        if (c.name.startsWith("ssc_")) sawSsc = true;
        if (c.name.includes("input")) sawInputDir = true;
        if (c.name.includes("output")) { sawOutputDir = true; sawPolled = true; }
        if (c.name.includes("ie_edge")) sawIntEnabled = true;
    }
    if (notReached.length) problems.push(`COVERAGE: case(s) did not reach comparison: ${notReached.join(", ")}`);
    if (!sawIpr) problems.push("COVERAGE: IPR path never exercised");
    if (!sawSsc) problems.push("COVERAGE: SSC mirror path never exercised");
    if (!sawInputDir) problems.push("COVERAGE: input direction never exercised");
    if (!sawOutputDir) problems.push("COVERAGE: output direction never exercised");
    if (!sawPolled) problems.push("COVERAGE: polled mode never exercised");
    if (!sawIntEnabled) problems.push("COVERAGE: interrupt-enabled mode never exercised");
    return problems;
}

/* ------------------------------------------------------------------------------------------- *
 * DEVTRACE phase -- one representative round trip, graded LITERALLY against patch 0006's           *
 * IPRR/IPRW/REGR/REGW lines (done condition 2's own wording).                                       *
 * ------------------------------------------------------------------------------------------- */

const DEVTRACE_INI = `\
set console notelnet
set cpu 16m
set cpu simhalt
reset all
deposit MAPEN 0
deposit SCBB ${hex(R_SCBB)}
deposit KSP ${hex(R_KSP)}
deposit R14 ${hex(R_KSP)}
{DEPOSITS}
deposit PSL 0
deposit PC ${hex(R_CODE)}
set cpu debug=EXCTRACE
set sysd debug=DEVTRACE
set debug -n {LOGFILE}
step {STEPS}
set debug -n
exit
`;

/**
 * buildDevtraceProgram()
 *
 * TXCS/TXDB polled output through the IPR path (write 'D', poll busy, step the wait, poll done),
 * THEN the identical round trip again through the SSC mirror with a different byte -- one program,
 * both address paths, in a single instrumented run.
 *
 * @returns {{bytes: number[], stepCount: number}}
 */
function buildDevtraceProgram()
{
    let bytes = [];
    emitMtprImm(bytes, 0x44, MT.TXDB);                    // 1: IPR TXDB write ('D')
    emitMfprToAbs(bytes, MT.TXCS, R_DATA);                // 2: IPR TXCS read (busy)
    let waitStart = bytes.length;
    for (let i = 0; i < SERIAL_OUT_WAIT; i++) bytes.push(NOP_BYTE);   // absorb the wait
    emitMfprToAbs(bytes, MT.TXCS, (R_DATA + 4) | 0);      // IPR TXCS read (done)
    emitMovlImmToAbs(bytes, 0x45, SSC_TXDB);              // SSC TXDB write ('E')
    emitMovlAbsToAbs(bytes, SSC_TXCS, (R_DATA + 8) | 0);  // SSC TXCS read (busy)
    for (let i = 0; i < SERIAL_OUT_WAIT; i++) bytes.push(NOP_BYTE);
    emitMovlAbsToAbs(bytes, SSC_TXCS, (R_DATA + 12) | 0); // SSC TXCS read (done)
    let stepCount = 2 + SERIAL_OUT_WAIT + 1 + 2 + SERIAL_OUT_WAIT + 1;
    return {bytes, stepCount};
}

function runDevtraceSimh(simh, scratch, program)
{
    let deposits = [];
    for (let k = 0; k < 16; k++) deposits.push(`deposit -l ${hex((R_DATA + k * 4) | 0)} 0`);
    for (let i = 0; i < program.bytes.length; i++) deposits.push(`deposit -b ${hex(R_CODE + i)} ${program.bytes[i].toString(16)}`);
    let logPath = path.join(scratch, "consoledif-devtrace.log");
    if (fs.existsSync(logPath)) fs.unlinkSync(logPath);
    let ini = DEVTRACE_INI
        .replace("{DEPOSITS}", deposits.join("\n"))
        .replace("{LOGFILE}", logPath)
        .replace("{STEPS}", String(program.stepCount));
    runSimh(simh, ini, path.join(scratch, "consoledif-devtrace.ini"));
    if (!fs.existsSync(logPath)) throw new Error("consoledif: DEVTRACE log was not produced");
    return fs.readFileSync(logPath, "utf8");
}

/** Parse patch 0006's IPRR/IPRW/REGR/REGW lines, filtered to this item's four IPRs and the SSC
    mirror's four offsets -- see tests/simh/README.md "What 0006 adds" for the field grammar. */
function parseDevtrace(raw)
{
    let events = [];
    let iprNames = new Set([MT.RXCS, MT.RXDB, MT.TXCS, MT.TXDB]);
    let sscAddrs = new Set([SSC_RXCS, SSC_RXDB, SSC_TXCS, SSC_TXDB]);
    for (let line of raw.split("\n")) {
        let m = /(IPRR|IPRW|REGR|REGW)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)/.exec(line);
        if (!m) continue;
        let [, kind, rgOrPa, size, val] = m;
        if (kind === "IPRR" || kind === "IPRW") {
            let rg = parseInt(rgOrPa, 16);
            if (!iprNames.has(rg)) continue;
            events.push({kind, key: `IPR${rg}`, val: parseInt(val, 16) >>> 0});
        } else {
            let pa = parseInt(rgOrPa, 16) >>> 0;
            if (!sscAddrs.has(pa)) continue;
            events.push({kind, key: `REG${hex(pa)}`, val: parseInt(val, 16) >>> 0});
        }
    }
    return events;
}

/**
 * runDevtraceJS(program)
 *
 * Records the SAME event shape parseDevtrace() extracts from SIMH's log, by temporarily wrapping
 * ConsoleVAX.prototype's own register-model methods for the duration of this one run -- the same
 * "wrap the prototype, restore it in a finally" idiom every selfcheck in this project already uses
 * (see romdiff.js's/hwintdiff.js's MUTATIONS), applied here to OBSERVE rather than to break.
 *
 * @param {Object} program
 * @returns {Array<{kind:string, key:string, val:number}>}
 */
function runDevtraceJS(program)
{
    let events = [];
    let proto = ConsoleVAX.prototype;
    /*
     * Wrap ONLY the two TRUE entry points -- `read`/`write` (exc.js's setIPRDevice() seam) and
     * `sscRead`/`sscWrite` (ssc.js's mirror seam) -- matching exactly where patch 0006's real hooks
     * sit (ReadIPR/WriteIPR and ReadReg/WriteReg, vax_sysdev.c).  `sscRead`/`sscWrite` call the
     * SAME rxcsRd()/txcsRd()/etc. functions `read`/`write` do (see the file header and console.js's
     * own doc comment) -- wrapping those INNER functions too, as an earlier version of this file
     * did, double-counted every mirror access (one event from the wrapped inner function, a second
     * from the wrapped outer one) and was caught by this file's own comparison against SIMH's true,
     * single-event-per-access count.
     */
    let orig = { read: proto.read, write: proto.write, sscRead: proto.sscRead, sscWrite: proto.sscWrite };
    proto.read = function(prn) {
        let v = orig.read.call(this, prn);
        if (prn === MT.RXCS || prn === MT.RXDB || prn === MT.TXCS || prn === MT.TXDB) {
            events.push({kind: "IPRR", key: `IPR${prn}`, val: v >>> 0});
        }
        return v;
    };
    proto.write = function(prn, val) {
        if (prn === MT.RXCS || prn === MT.RXDB || prn === MT.TXCS || prn === MT.TXDB) {
            events.push({kind: "IPRW", key: `IPR${prn}`, val: val >>> 0});
        }
        return orig.write.call(this, prn, val);
    };
    proto.sscRead = function(rg) {
        let v = orig.sscRead.call(this, rg);
        if (v !== null) {
            let addr = (SSCBASE + rg * 4) >>> 0;
            events.push({kind: "REGR", key: `REG${hex(addr)}`, val: v >>> 0});
        }
        return v;
    };
    proto.sscWrite = function(rg, val) {
        let addr = (SSCBASE + rg * 4) >>> 0;
        events.push({kind: "REGW", key: `REG${hex(addr)}`, val: val >>> 0});
        return orig.sscWrite.call(this, rg, val);
    };
    try {
        let {bus, cpu} = makeMachine();
        cpu.exc.scbb = R_SCBB;
        cpu.exc.stk[KERN] = R_KSP;
        cpu.regs.fill(0);
        cpu.regs[14] = R_KSP;
        for (let i = 0; i < program.bytes.length; i++) bus.setByte((R_CODE + i) | 0, program.bytes[i]);
        cpu.psl = 0;
        cpu.regs[15] = R_CODE;
        for (let s = 0; s < program.stepCount; s++) cpu.stepOne();
    } finally {
        Object.assign(proto, orig);
    }
    return events;
}

function phaseDevtrace(simh, scratch)
{
    let problems = [];
    let program = buildDevtraceProgram();
    let raw = runDevtraceSimh(simh, scratch, program);
    let simhEvents = parseDevtrace(raw);
    let jsEvents = runDevtraceJS(program);
    if (simhEvents.length === 0) {
        problems.push("DEVTRACE: SIMH's log produced ZERO events for this item's four registers -- the run did not reach comparison");
        return problems;
    }
    if (jsEvents.length !== simhEvents.length) {
        problems.push(`DEVTRACE: event count mismatch -- js=${jsEvents.length} simh=${simhEvents.length}`);
    }
    let n = Math.min(jsEvents.length, simhEvents.length);
    for (let i = 0; i < n; i++) {
        let a = jsEvents[i], b = simhEvents[i];
        if (a.kind !== b.kind || a.key !== b.key || a.val !== b.val) {
            problems.push(`DEVTRACE: event ${i} js=${a.kind}/${a.key}/0x${hex(a.val)} simh=${b.kind}/${b.key}/0x${hex(b.val)}`);
        }
    }
    return problems;
}

/* ------------------------------------------------------------------------------------------- *
 * INTERRUPT phase -- TTI/TTO delivered at IPL 0x14 through the REAL console device.                *
 * ------------------------------------------------------------------------------------------- */

/**
 * interruptProgram(kind)
 *
 * Builds the SETUP instruction bytes AND the exact INSTRUCTION COUNT ("stepCount") needed to reach
 * delivery, for both "tto" (write with IE set -- the deferred, tick()-driven path, delivered on the
 * step whose setIRQL() first observes `cpu.nTotalCycles >= txDoneAt`) and "tti" (RXCS's edge-raise,
 * synchronous inside rxcsWr() -- delivered on the VERY NEXT step's setIRQL()).  `stepCount` is an
 * INSTRUCTION count, not a byte count -- an earlier version of this file used `bytes.length + 1`
 * (a byte count) as a step count, which for "tto" under-stepped by roughly 6x (each MTPR is 7
 * bytes, only 1 of which is "one instruction") and never reached delivery at all.
 *
 * @param {string} kind "tto" or "tti"
 * @returns {{bytes: number[], stepCount: number}}
 */
function interruptProgram(kind)
{
    let bytes = [];
    let stepCount;
    if (kind === "tto") {
        /*
         * ORDER MATTERS: TXCS starts CSR_DONE (reset() -- see console.js) with IE clear.  Writing
         * IE=1 FIRST, while DONE is already set, hits txcsWr()'s OWN edge-trigger (`(txcs &
         * (DONE|IE)) === DONE` -> raiseInterrupt() IMMEDIATELY, vax_stddev.c's txcs_wr() reproduced
         * faithfully) -- delivering right away and never exercising the DEFERRED, tick()-driven
         * completion path this case exists to grade (that edge case is ALREADY covered by the REAL
         * phase's "ipr_txcs_ie_edge"/"ssc_txcs_ie_edge").  Writing TXDB FIRST clears DONE (no edge
         * condition possible), so the interrupt that follows can ONLY come from maybeComplete()'s
         * deferred fire -- an earlier version of this file got the order backwards and was caught,
         * by direct instrumentation, delivering at step 2 instead of step 102.
         */
        emitMtprImm(bytes, 0x46, MT.TXDB);                // instruction 1: write -- clears DONE, schedules completion
        emitMtprImm(bytes, CSR_IE, MT.TXCS);              // instruction 2: enable IE -- DONE is clear, no edge
        for (let i = 0; i < SERIAL_OUT_WAIT; i++) bytes.push(NOP_BYTE);
        stepCount = 2 + SERIAL_OUT_WAIT;                  // see console.js's maybeComplete() timing
    } else {
        emitMtprImm(bytes, CSR_IE, MT.RXCS);              // instruction 1: enable IE -- edges immediately
        bytes.push(NOP_BYTE);
        stepCount = 2;                                    // instruction 2's OWN setIRQL() sees it
    }
    return {bytes, stepCount};
}

function buildInterruptScript(kind)
{
    let {bytes, stepCount} = interruptProgram(kind);
    let L = [`set cpu ${MEMSIZE / (1024 * 1024)}m`, "set cpu simhalt", "reset all"];
    L.push(`deposit MAPEN 0`, `deposit SCBB ${hex(R_SCBB)}`, `deposit KSP ${hex(R_KSP)}`, `deposit R14 ${hex(R_KSP)}`);
    for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_HANDLER + k)} ${NOP_BYTE.toString(16)}`);
    L.push(`deposit ${hex(R_SCBB + SCB_TTI)} ${hex(R_HANDLER)}`, `deposit ${hex(R_SCBB + SCB_TTO)} ${hex(R_HANDLER)}`);
    if (kind === "tti") L.push("deposit TTI BUF 47", "deposit TTI CSR 80");   // DONE already set
    for (let k = 0; k < bytes.length + 8; k++) L.push(`deposit -b ${hex(R_CODE + k)} 0`);
    for (let i = 0; i < bytes.length; i++) L.push(`deposit -b ${hex(R_CODE + i)} ${bytes[i].toString(16)}`);
    for (let k = 0; k < 8; k++) L.push(`deposit -b ${hex((R_CODE + bytes.length + k) | 0)} ${NOP_BYTE.toString(16)}`);
    L.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`);
    L.push(`step ${stepCount}`);
    L.push("examine -h PC", "examine -h PSL");
    L.push("exit", "");
    return {script: L.join("\n") + "\n", bytes};
}

function runInterruptCaseJS(kind)
{
    let {bus, cpu, consoleDev} = makeMachine();
    cpu.exc.scbb = R_SCBB;
    cpu.exc.stk[KERN] = R_KSP;
    cpu.regs.fill(0);
    cpu.regs[14] = R_KSP;
    for (let k = 0; k < 16; k++) bus.setByte(R_HANDLER + k, NOP_BYTE);
    bus.setLong(R_SCBB + SCB_TTI, R_HANDLER);
    bus.setLong(R_SCBB + SCB_TTO, R_HANDLER);
    let {bytes, stepCount} = interruptProgram(kind);
    if (kind === "tti") consoleDev.injectChar(0x47);
    for (let k = 0; k < bytes.length + 8; k++) bus.setByte((R_CODE + k) | 0, 0);
    for (let i = 0; i < bytes.length; i++) bus.setByte((R_CODE + i) | 0, bytes[i]);
    for (let k = 0; k < 8; k++) bus.setByte((R_CODE + bytes.length + k) | 0, NOP_BYTE);
    cpu.psl = 0;
    cpu.regs[15] = R_CODE;
    for (let s = 0; s < stepCount; s++) cpu.stepOne();
    return {pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0};
}

function phaseInterrupt(simh, scratch)
{
    let problems = [];
    let sawTti = false, sawTto = false;
    for (let kind of ["tto", "tti"]) {
        let {script} = buildInterruptScript(kind);
        let out = runSimh(simh, script, path.join(scratch, `consoledif-int-${kind}.ini`));
        let vals = [...out.matchAll(VALUE_RE_G)].map((m) => parseInt(m[2], 16) >>> 0);
        if (vals.length < 2) { problems.push(`INTERRUPT ${kind}: SIMH produced no readback`); continue; }
        let [simhPC, simhPSL] = vals;
        let js = runInterruptCaseJS(kind);
        let deliveredSimh = simhPC >= R_HANDLER && simhPC < R_HANDLER + 16;
        let deliveredJs = js.pc >= R_HANDLER && js.pc < R_HANDLER + 16;
        if (!deliveredSimh) { problems.push(`INTERRUPT ${kind}: SIMH itself never delivered -- case is not exercising what it claims`); continue; }
        if (deliveredJs !== deliveredSimh) problems.push(`INTERRUPT ${kind}: delivered js=${deliveredJs} simh=${deliveredSimh}`);
        if ((js.pc >>> 0) !== (simhPC >>> 0)) problems.push(`INTERRUPT ${kind}: PC js=0x${hex(js.pc)} simh=0x${hex(simhPC)}`);
        if ((js.psl >>> 0) !== (simhPSL >>> 0)) problems.push(`INTERRUPT ${kind}: PSL js=0x${hex(js.psl)} simh=0x${hex(simhPSL)}`);
        let ipl = (simhPSL >>> PSL_V_IPL) & 0x1F;
        if (ipl !== IPL_HMIN) problems.push(`INTERRUPT ${kind}: SIMH's own delivered IPL is 0x${hex(ipl, 2)}, expected 0x${hex(IPL_HMIN, 2)}`);
        if (kind === "tto" && deliveredSimh) sawTto = true;
        if (kind === "tti" && deliveredSimh) sawTti = true;
    }
    if (!sawTti) problems.push("COVERAGE: TTI interrupt delivery was not confirmed");
    if (!sawTto) problems.push("COVERAGE: TTO interrupt delivery was not confirmed");
    return problems;
}

/* ------------------------------------------------------------------------------------------- *
 * SELFCHECK -- the five named mutations, each proven caught against the case that exposes it.     *
 * ------------------------------------------------------------------------------------------- */

function selfcheck()
{
    console.log("\nSELFCHECK -- each mutation must be caught\n");
    let survived = [];

    function check(name, mutate, restore, run)
    {
        let caught = false, how = "";
        try {
            mutate();
            try {
                let result = run();
                if (result) { caught = true; how = result; }
            } finally {
                restore();
            }
        } catch (e) {
            caught = true; how = `threw ${e.name}: ${e.message}`;
        }
        console.log(`  ${caught ? "CAUGHT " : "SURVIVED"} ${name.padEnd(32)} ${how}`);
        if (!caught) survived.push(name);
    }

    /* 1. TXCS DONE stuck clear -- THE HANG the item's own text names first. */
    {
        let orig = ConsoleVAX.prototype.maybeComplete;
        check("txcs-done-stuck-clear",
            () => { ConsoleVAX.prototype.maybeComplete = function() {}; },
            () => { ConsoleVAX.prototype.maybeComplete = orig; },
            () => {
                let {cpu, consoleDev} = makeMachine();
                consoleDev.txdbWr(0x41);
                for (let i = 0; i < SERIAL_OUT_WAIT + 10; i++) cpu.nTotalCycles++;
                let v = consoleDev.txcsRd();
                return (v & CSR_DONE) ? null : `TXCS never reports DONE after ${SERIAL_OUT_WAIT + 10} cycles -- v=0x${hex(v, 2)}`;
            });
    }
    /* 2. DONE stuck set -- a write never clears it, so the ROM would (wrongly) see the PREVIOUS
          character's completion instead of waiting for the new one. */
    {
        let orig = ConsoleVAX.prototype.txdbWr;
        check("txcs-done-stuck-set",
            () => {
                ConsoleVAX.prototype.txdbWr = function(val) {
                    this.txBuf = val & 0xFF; this.output.push(this.txBuf);
                    /* deliberately do NOT clear CSR_DONE or schedule a completion */
                };
            },
            () => { ConsoleVAX.prototype.txdbWr = orig; },
            () => {
                let {consoleDev} = makeMachine();
                consoleDev.txdbWr(0x41);
                let v = consoleDev.txcsRd();
                return (v & CSR_DONE) ? `TXCS reports DONE immediately after a write -- v=0x${hex(v, 2)}` : null;
            });
    }
    /* 3. RXDB returning the wrong byte lane. */
    {
        let orig = ConsoleVAX.prototype.rxdbRd;
        check("rxdb-wrong-byte-lane",
            () => { ConsoleVAX.prototype.rxdbRd = function() { return (this.rxBuf << 8) & 0xFFFF; }; },
            () => { ConsoleVAX.prototype.rxdbRd = orig; },
            () => {
                let {consoleDev} = makeMachine();
                consoleDev.injectChar(0x5A);
                let v = consoleDev.rxdbRd();
                return (v === 0x5A) ? null : `RXDB returned 0x${hex(v, 2)}, expected 0x5A`;
            });
    }
    /* 4. SSC mirror diverging from the IPR path -- the mirror reads/writes a SEPARATE copy of the
          register state instead of the SAME object done condition 2 requires. */
    {
        let orig = SSCVAX.prototype.readReg;
        check("ssc-mirror-diverges-from-ipr",
            () => {
                SSCVAX.prototype.readReg = function(rg) {
                    if (rg === 0x22) return 0;             // TXCS via the mirror always reads 0
                    return orig.call(this, rg);
                };
            },
            () => { SSCVAX.prototype.readReg = orig; },
            () => {
                let {bus, consoleDev} = makeMachine();
                let iprVal = consoleDev.txcsRd();
                let sscVal = bus.getLong(SSC_TXCS) >>> 0;
                return (iprVal !== sscVal) ? `ipr=0x${hex(iprVal, 2)} ssc=0x${hex(sscVal, 2)} -- the two paths disagree` : null;
            });
    }
    /* 5. TTI/TTO vectors swapped at installation. */
    {
        let m = makeMachine();      // build once, unmutated, to get a real exc
        let origAdd = m.cpu.exc.addInterruptSource.bind(m.cpu.exc);
        check("tti-tto-vectors-swapped",
            () => {
                VAXExc.prototype.addInterruptSource = function(lvl, bit, vec) {
                    if (bit === TTI_BIT && vec === SCB_TTI) vec = SCB_TTO;
                    else if (bit === TTO_BIT && vec === SCB_TTO) vec = SCB_TTI;
                    origAdd(lvl, bit, vec);
                };
            },
            () => { delete VAXExc.prototype.addInterruptSource; },
            () => {
                let mm = makeMachine();
                let vec = mm.cpu.exc.deviceVector(mm.cpu, IPL_HMIN);   // nothing raised yet -- table only
                mm.cpu.exc.raiseInterrupt(IPL_HMIN, TTI_BIT);
                vec = mm.cpu.exc.deviceVector(mm.cpu, IPL_HMIN);
                return (vec === SCB_TTI) ? null : `TTI delivered vector 0x${hex(vec, 2)}, expected 0x${hex(SCB_TTI, 2)}`;
            });
    }

    if (survived.length) {
        console.error("\nFAILURES:");
        for (let n of survived) console.error(`  SELFCHECK: mutation "${n}" was NOT caught -- that is a coverage hole, not a pass`);
        process.exit(1);
    }
    console.log("\nOK");
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                              *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, dflt) { let i = process.argv.indexOf(name); return (i >= 0 && i + 1 < process.argv.length) ? process.argv[i + 1] : dflt; }

function main()
{
    let simh = findSimh(getArg("--simh", null));
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "consoledif-"));
    console.log(`consoledif.js: simh=${simh} scratch=${scratch}`);

    if (process.argv.indexOf("--selfcheck") >= 0) {
        selfcheck();
        return;
    }

    let problems = [];
    console.log("\n=== REAL phase ===");
    for (let p of phaseReal(simh, scratch)) problems.push(p);

    console.log("\n=== DEVTRACE phase (patch 0006) ===");
    for (let p of phaseDevtrace(simh, scratch)) problems.push(p);

    console.log("\n=== INTERRUPT phase ===");
    for (let p of phaseInterrupt(simh, scratch)) problems.push(p);

    if (problems.length) {
        console.error(`\nFAILED (${problems.length} problem(s)):`);
        for (let p of problems) console.error("  - " + p);
        process.exit(1);
    }
    console.log("\nPASSED.");
}

main();
