/**
 * @fileoverview Differential test: the VAX CPU loop (fetch/dispatch/trap) vs. a real Open SIMH
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
 *               microvax3900
 *
 * WHAT THIS IS, AND WHY IT IS NOT LIKE ITS EIGHT SIBLINGS
 * -------------------------------------------------------
 * Every other differential in this directory proves that AN INSTRUCTION MATCHES: it deposits a
 * pre-state, runs exactly one instruction on both sides, and compares.  This one proves that A
 * PROGRAM RUNS -- which is a different claim, and one no number of green opcodes implies, because
 * the things it grades (fault_PC, the FPD resume, deferred trap delivery, the abort/unwind
 * boundary, the SCB emulate trap, cycle accounting) are BY CONSTRUCTION invisible to a
 * single-instruction comparison.  Four phases:
 *
 *   EHKAA        The real workload.  The DEC EHKAA hardware-core diagnostic is loaded at physical
 *                0 and run from 0x200 -- `load ehkaa.exe` + `go 200`, byte for byte what SIMH
 *                does -- on BOTH machines, each emitting a SIMH-format instruction-history trace,
 *                and the two traces are compared by `tools/trace-differ/differ.py` in the pcjs-vax
 *                work repo.  That tool was built in Wave 0 for exactly this and had never been
 *                pointed at a running machine.  Every instruction's PC, PSL, disassembly, resolved
 *                operand queue, result and full register file is compared, in order, and the FIRST
 *                divergence is reported with an index and a PC.
 *
 *   RANDOMIZED   Live SIMH console.  Short PROGRAMS -- not single instructions -- built from eight
 *                case kinds chosen to hit the loop's own decisions, deposited on both sides, run
 *                with `step N`, and compared on the full register file, PSL, every privileged
 *                register and a window of every stack plus the case's own data.  `step N` is the
 *                cycle-accounting grade as well: SCP's STEP counts `sim_interval` units, which is
 *                `1 + (extra_bytes >> 5)` per instruction, so a machine that charges the wrong
 *                number of cycles for a string instruction stops at a different instruction and
 *                the register files disagree.
 *
 *   INIE-RESET   pcjsvax-1be.  EHKAA and RANDOMIZED both reset the WHOLE machine between every
 *                case (`cpu.reset()` / `reset all`), so neither can see a defect that exists only
 *                ACROSS two stepCPU() calls with nothing reset in between -- the boundary real
 *                SIMH draws at vax_cpu.c:514 (`in_ie = 0`, unconditional, every `sim_instr()` call,
 *                i.e. every SCP `step`).  This phase deposits ONE state, calls `stepCPU(1)` TWICE
 *                with no reset between (matching two `step 1` commands with no `reset all` between
 *                them), and grades PC/PSL/R14/SISR after EACH call.  See phaseInIEReset()'s own
 *                header for the deterministic sequence.
 *
 *   SELFCHECK    `--selfcheck` injects deliberate defects into the SHIPPED code path (cpustate.js's
 *                dispatch/loop, and the trap-request and FPD-resume lines this item added to
 *                cpu.js/control.js/strq.js) and fails if the differential does not catch each one.
 *                It also re-runs INIE-RESET once with pcjsvax-1be's own fix defeated (see
 *                applyInIEMutation()) and fails unless that defeats the comparison too.
 *
 * WHY BOTH A REAL WORKLOAD AND A RANDOMIZED PHASE
 * -----------------------------------------------
 * They fail on different things and each is blind to the other's failures.  EHKAA found the two
 * bugs that mattered most here -- the missing deferred integer-overflow trap (`ADDL3` at 80013524,
 * which no randomized generator in this project would have produced, because none of them arms
 * PSW<IV> and then overflows on purpose) and the emulate-trap dispatch -- but it exercises exactly
 * one path through each and cannot vary it.  The randomized phase walks the boundaries EHKAA
 * happens not to visit.
 *
 *      node machines/dec/vax/tests/cpudiff.js [options]
 *        --simh PATH        patched microvax3900 (patch 0001 for REGS; else $SIMH_CPU_BIN,
 *                            $SIMH_INT_BIN, $SIMH_BIN, else the scratch build)
 *        --ehkaa PATH       default $PCJS_VAX_REPO/open-simh/VAX/tests/ehkaa.exe
 *        --trace FILE       reuse an existing SIMH EHKAA capture instead of running SIMH for it
 *        --cases N          randomized cases, default 240, floor 80
 *        --seed S
 *        --skip-ehkaa       randomized phase only
 *        --selfcheck        prove the differential detects deliberate defects
 *
 * COVERAGE IS ASSERTED, NOT REPORTED.  The floors below FAIL the run and do not scale down with
 * `--cases`; `--cases` below its floor fails outright rather than shrinking them.
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import MMUVAX from "../modules/v2/mmu.js";
import { VAX } from "../modules/v2/defines.js";
import { OPCODES, DROM, DROM_STRIDE, DR, IG } from "../modules/v2/drom.js";
import CPUStateVAX, { DISPATCH, DISPATCH_OWNER, CIS_EMULATED } from "../modules/v2/cpustate.js";
import { VAXStop } from "../modules/v2/exc.js";
import SimhTrace, { nonStoringResultOpcodes } from "./simhtrace.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors.  Every one of these FAILS the run.                                            *
 * ------------------------------------------------------------------------------------------- */

const MIN_EHKAA_RECORDS   = 320000;

/**
 * The EXACT number of opcodes the dispatch table claims -- an equality, not a floor, and NOT the
 * same set as "the base-group opcodes EHKAA executes".
 *
 * THIS COMMENT USED TO READ "EHKAA executes every one of the 242 base-group opcodes
 * (docs/reference/ehkaa-profile.md §7)", which describes a different set than the assertion below
 * measures; the two merely happened to be the same size.  pcjsvax-486 separated them: ACBF is
 * claimed by control.js so it counts HERE, but it is IG_EMONL rather than base-group and EHKAA
 * never executes it -- which is exactly why it stayed unimplemented until the console ROM's
 * self-test 51 reached it.  243 = the previous 242 + ACBF.
 *
 * Standing rule 12 again: the number was right and the sentence about it claimed more than the
 * number did.  The base group's own completeness is asserted where it belongs, by
 * tests/base_group_residual.js --check-carveouts, which this value does not affect.
 */
const EXPECTED_DISPATCH_CLAIMS = 243;

/**
 * A FLOOR on how many DISTINCT opcodes EHKAA actually executes (measured: 279).  This is the
 * claim the old comment above was really making, and it is a genuinely different quantity from
 * EXPECTED_DISPATCH_CLAIMS -- which is why splitting them was the right fix rather than bumping
 * one number.  It deliberately does NOT move with ACBF: EHKAA never executes ACBF, so requiring
 * 243 here would demand coverage the workload cannot provide (docs/reference/ehkaa-profile.md §7).
 */
const MIN_DISTINCT_OPCODES = 242;
/** The emulate trap and the reserved-instruction fault for H_float both really happen in EHKAA. */
const MIN_EMULATE_TRAPS   = 40;

const MIN_CASES           = 80;
const DEFAULT_CASES       = 240;
/** Every randomized case kind must be exercised at least this many times. */
const MIN_PER_KIND        = 4;

const MEMSIZE = 0x01000000;             // 16MB, the SIMH microvax3900 default
const PAGE = 512;

const KERN = 0, EXEC = 1, SUPV = 2, USER = 3;
const PSL_V_CUR = 24, PSL_V_PRV = 22, PSL_V_IPL = 16;
const PSL_IS = 1 << 26, PSL_FPD = 1 << 27;
const PSW_T = 0x10, PSW_IV = 0x20;

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Plumbing                                                                                       *
 * ------------------------------------------------------------------------------------------- */

function vaxRepo()
{
    if (process.env['PCJS_VAX_REPO']) return process.env['PCJS_VAX_REPO'];
    return path.resolve(__dirname, "../../../../../pcjs-vax");
}

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    for (let v of ['SIMH_CPU_BIN', 'SIMH_INT_BIN', 'SIMH_DECODE_BIN', 'SIMH_BIN']) {
        if (process.env[v]) candidates.push(process.env[v]);
    }
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    candidates.push(path.join(vaxRepo(), "open-simh/BIN/microvax3900"));
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error(
        "This test grades against a REAL SIMH built with patch 0001 (the REGS line); it has no\n" +
        "fixture fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh\n" +
        "PATH or set $SIMH_CPU_BIN.  Tried:\n  " + candidates.join("\n  "));
}

function findDiffer()
{
    let p = path.join(vaxRepo(), "tools/trace-differ/differ.py");
    if (!fs.existsSync(p)) {
        throw new Error(`trace-differ not found at ${p}; set $PCJS_VAX_REPO to the pcjs-vax work repo`);
    }
    return p;
}

function runSimh(bin, script, iniPath)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 60 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test                                                                         *
 * ------------------------------------------------------------------------------------------- */

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    return {bus, cpu};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 1 -- EHKAA, the real workload                                                            *
 * ------------------------------------------------------------------------------------------- */

/**
 * The two record classes whose SIMH text is an artifact of the ORACLE rather than a statement
 * about the machine, normalized identically on BOTH traces before comparison, counted, and
 * reported.  Nothing else is normalized; in particular no register, PC, PSL or operand value is
 * ever touched, and the disassembly is dropped only for opcodes that decode ZERO specifiers.
 *
 *   CMPD          vax_sys.c gives it RB_Q but vax_cpu.c:2818 never assigns `r`/`rh`, so SIMH
 *                 prints two stale C locals.  `rh` in particular is last written by MULL2/MULL3
 *                 (through `op_emul`'s out-parameter) -- an opcode whose OWN result shape is RB_L
 *                 and which therefore leaves no trace of having written it.  Reproducing that
 *                 means shadowing sim_instr()'s locals across all 242 bodies; the value is not a
 *                 property of the VAX and grading it proves nothing.
 *
 *   ZEROSPEC      An opcode this CPU does not implement whose decode ROM declares ZERO decodable
 *                 specifiers (the ODC()-wrapped H_floating and octaword rows).  SIMH records only
 *                 `PC - fault_PC` = 2 instruction bytes for it and DISASSEMBLES ALL 52, so the
 *                 operand text is whatever the instruction that occupied that history slot 100,000
 *                 instructions ago left behind.  The mnemonic, PC, PSL and register file are
 *                 compared normally; only the operand text is dropped.
 *
 * A third class is dropped by INDEX, not by mnemonic: `unavailable`, the records whose store
 * FAULTED (see simhtrace.js's finish()).  Those indices come from the JS run itself.
 */
const ZEROSPEC = (function() {
    let s = new Set();
    for (let opc = 0; opc < 512; opc++) {
        if (!OPCODES[opc]) continue;
        let hdr = DROM[opc * DROM_STRIDE];
        if ((hdr & DR.NSPMASK) === 0 && ((hdr >> 4) & 0x7) > 0) s.add(OPCODES[opc]);
    }
    return s;
})();

/**
 * captureSimhTrace(simh, opts)
 *
 * `set cpu history=N:file` + `load` + `go 200`, i.e. the EHKAA gate's own invocation.  The history
 * buffer is circular and flushes to the logfile every time it wraps, so the capture is the WHOLE
 * run, not the last N instructions.
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {string} path to the trace
 */
function captureSimhTrace(simh, opts)
{
    if (opts.trace) return opts.trace;
    if (!fs.existsSync(opts.ehkaa)) {
        throw new Error(`EHKAA diagnostic not found at ${opts.ehkaa}; pass --ehkaa PATH or --skip-ehkaa`);
    }
    let tracePath = path.join(opts.scratch, "cpudiff-simh.trace");
    if (fs.existsSync(tracePath)) fs.unlinkSync(tracePath);
    let script = [
        `set cpu history=100000:${tracePath}`,
        `load ${opts.ehkaa}`,
        "go -q 200",
        "examine PC",
        "exit", ""
    ].join("\n");
    let out = runSimh(simh, script, path.join(opts.scratch, "cpudiff-simh.ini"));
    if (!/PC:\s*80018AD1/.test(out)) {
        throw new Error("SIMH did not reach EHKAA's documented PASS PC (0x80018AD1); SIMH said:\n" + out);
    }
    return tracePath;
}

/**
 * runEhkaaJS(opts)
 *
 * The same program, on this machine, emitting the same trace format.
 *
 * @param {Object} opts
 * @param {?string} mutation
 * @returns {Object}
 */
function runEhkaaJS(opts, mutation)
{
    let {cpu} = makeMachine();
    if (mutation) applyMutation(cpu, mutation);
    cpu.loadImage(new Uint8Array(fs.readFileSync(opts.ehkaa)));
    cpu.setPC(0x200);
    let hst = new SimhTrace();
    cpu.hst = hst;

    let opcodesSeen = new Set();
    let nEmulate = 0;
    let origExec = cpu.executeOne.bind(cpu);
    cpu.executeOne = function(opc, d, c) {
        opcodesSeen.add(opc);
        if (CIS_EMULATED.has(opc) && !DISPATCH[opc]) nEmulate++;
        return origExec(opc, d, c);
    };
    let stop = null, steps = 0;
    try {
        while (steps < opts.maxSteps) { cpu.stepCPU(1); steps++; }
    } catch (e) {
        if (!(e instanceof VAXStop)) throw e;
        stop = e;
    }
    hst.finish(cpu);                                    // flush the last pending record
    let tracePath = path.join(opts.scratch, "cpudiff-js.trace");
    fs.writeFileSync(tracePath, hst.text());
    return {
        tracePath, records: hst.count, steps, stop,
        unavailable: hst.unavailable, unreadable: hst.nUnreadable,
        opcodesSeen, nEmulate,
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0,
        cycles: cpu.getCycles()
    };
}

/**
 * compareTraces(jsPath, simhPath, unavailable, opts)
 *
 * Truncates the SIMH trace to the JS trace's length, applies the two documented normalizations to
 * BOTH sides plus the per-index `unavailable` drop, and hands the result to
 * `tools/trace-differ/differ.py` -- the actual comparison is that tool's, not this file's.
 *
 * @returns {Object}
 */
function compareTraces(jsPath, simhPath, unavailable, opts)
{
    let driver = path.join(opts.scratch, "cpudiff-driver.py");
    fs.writeFileSync(driver, DIFF_DRIVER);
    let unavPath = path.join(opts.scratch, "cpudiff-unavailable.json");
    fs.writeFileSync(unavPath, JSON.stringify(unavailable));
    let out = execFileSync("python3", [driver, findDiffer(), jsPath, simhPath, unavPath,
                                       [...ZEROSPEC].join(",")],
                           {encoding: "utf8", maxBuffer: 1 << 28});
    return JSON.parse(out);
}

/**
 * The Python driver.  It imports `differ.py` -- parse_trace() and diff_traces() are the oracle's,
 * verbatim -- normalizes, and reports.  Written out at run time rather than committed as a second
 * file so that the normalization and the assertion that counts it can never get separated.
 */
const DIFF_DRIVER = `#!/usr/bin/env python3
import sys, os, json
differ_path, js_path, simh_path, unav_path, zerospec = sys.argv[1:6]
sys.path.insert(0, os.path.dirname(differ_path))
from differ import parse_trace, diff_traces

ZEROSPEC = set(x for x in zerospec.split(",") if x)
unavailable = set(json.load(open(unav_path)))

a = parse_trace(js_path)
# Truncate the SIMH capture to the JS run's length WITHOUT parsing the whole file: EHKAA's full
# trace is ~70MB and the JS run may legitimately be shorter (it stops where the machine stops).
buf = []
n = 0
with open(simh_path, errors="replace") as f:
    for line in f:
        buf.append(line)
        if line.startswith("REGS"):
            n += 1
            if n >= len(a):
                break
tmp = simh_path + ".trunc"
open(tmp, "w").writelines(buf)
b = parse_trace(tmp)
os.unlink(tmp)

n_cmpd = n_zero = n_unav = 0
for recs in (a, b):
    for i, r in enumerate(recs):
        if r.mnemonic == "CMPD" and " ->" in r.detail:
            r.detail = r.detail.split(" ->")[0]; n_cmpd += 1
        elif r.mnemonic in ZEROSPEC:
            r.detail = r.mnemonic; n_zero += 1
        elif i in unavailable and " ->" in r.detail:
            r.detail = r.detail.split(" ->")[0]; n_unav += 1

d = diff_traces(a[:len(b)], b)
print(json.dumps({
    "js_records": len(a), "simh_records": len(b),
    "normalized_cmpd": n_cmpd, "normalized_zerospec": n_zero, "normalized_unavailable": n_unav,
    "match": d is None,
    "divergence": None if d is None else d.to_dict(),
}))
`;

/* ------------------------------------------------------------------------------------------- *
 * PHASE 2 -- randomized programs against a live SIMH console                                     *
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

/** Privileged registers, by the name BOTH sides use.  One list, so they cannot drift. */
const PRIV = [
    {name: "KSP",    get: (c) => c.exc.stk[0],   set: (c, v) => { c.exc.stk[0] = v; }},
    {name: "ESP",    get: (c) => c.exc.stk[1],   set: (c, v) => { c.exc.stk[1] = v; }},
    {name: "SSP",    get: (c) => c.exc.stk[2],   set: (c, v) => { c.exc.stk[2] = v; }},
    {name: "USP",    get: (c) => c.exc.stk[3],   set: (c, v) => { c.exc.stk[3] = v; }},
    {name: "IS",     get: (c) => c.exc.stk[4],   set: (c, v) => { c.exc.stk[4] = v; }},
    {name: "SCBB",   get: (c) => c.exc.scbb,     set: (c, v) => { c.exc.scbb = v; }},
    {name: "PCBB",   get: (c) => c.exc.pcbb,     set: (c, v) => { c.exc.pcbb = v; }},
    {name: "SBR",    get: (c) => c.mmu.sbr,      set: (c, v) => { c.mmu.setSBR(v); }},
    {name: "SLR",    get: (c) => c.mmu.slr,      set: (c, v) => { c.mmu.setSLR(v); },   mask: 0x003FFFFF},
    {name: "SISR",   get: (c) => c.exc.sisr,     set: (c, v) => { c.exc.sisr = v; },    mask: 0xFFFF},
    {name: "ASTLVL", get: (c) => c.exc.astlvl,   set: (c, v) => { c.exc.astlvl = v; },  mask: 0xF},
    {name: "MAPEN",  get: (c) => c.mmu.mapen,    set: (c, v) => { c.mmu.setMAPEN(v); }, mask: 0x1},
    {name: "TRPIRQ", get: (c) => c.exc.trpirq,   set: (c, v) => { c.exc.trpirq = v; },  mask: 0xFF}
];

/* Physical layout shared by every randomized case.  Flat, unmapped cases use these as virtual ==
   physical; the mapped kinds put the same pages in S0 through a system page table. */
const P = {
    SCBB:    0x00020000,
    CODE:    0x00021000,
    HANDLER: 0x00022000,
    KSTK:    0x00023000,        // stack grows DOWN from KSTK+0x400
    ISTK:    0x00024000,
    DATA:    0x00025000,
    DATA2:   0x00025200,        // the page immediately after DATA -- the FPD fault page
    SPT:     0x00028000         // system page table (mapped kinds)
};
const STK_TOP = P.KSTK + 0x400;
const IS_TOP = P.ISTK + 0x400;

/** S0 base the mapped kinds map P.* onto, page for page. */
const M_S0 = 0x80000000 | 0;
const M_PFN0 = (P.SCBB / PAGE) | 0;
function m_va(pa) { return (M_S0 + (pa - P.SCBB)) | 0; }

const PROT = (function() {
    let all = -1, none = -1;
    for (let c = 0; c < 16; c++) {
        let acc = MMUVAX.CVTACC[c];
        let needed = 0;
        for (let mode = 0; mode < 4; mode++) needed |= MMUVAX.accRead(mode) | MMUVAX.accWrite(mode);
        if (all < 0 && (acc & needed) === needed) all = c;
        if (none < 0 && acc === 0) none = c;
    }
    if (all < 0 || none < 0) throw new Error("cpudiff: cannot find all-access / no-access PTE protection codes");
    return {all, none};
})();

function makePTE(pfn, prot, valid)
{
    return ((valid ? MMUVAX.PTE_V : 0) | (prot << MMUVAX.PTE_V_ACC) | MMUVAX.PTE_M | (pfn & 0x1FFFFF)) | 0;
}

class Case {
    constructor(kind, index)
    {
        this.kind = kind;
        this.index = index;
        this.regs = new Int32Array(16);
        this.psl = 0;
        this.pc = 0;
        this.steps = 1;
        this.priv = {};
        for (let p of PRIV) this.priv[p.name] = 0;
        this.mem = new Map();       // physical longword address -> value
        this.probes = [];           // physical longword addresses compared afterwards
        this.note = "";
    }
    setLong(pa, val) { this.mem.set(pa >>> 0, val | 0); }
    setBytes(pa, bytes)
    {
        for (let i = 0; i < bytes.length; i++) {
            let a = (pa + i) >>> 0;
            let la = a & ~3, sh = (a & 3) * 8;
            let cur = this.mem.get(la) || 0;
            this.mem.set(la, ((cur & ~(0xFF << sh)) | ((bytes[i] & 0xFF) << sh)) | 0);
        }
    }
    probeRange(pa, n) { for (let i = 0; i < n; i++) this.probes.push((pa + i * 4) >>> 0); }
}

/** Opcode byte for a mnemonic, resolved through drom.js so no number is transcribed. */
function op(mn)
{
    let o = OPCODES.indexOf(mn);
    if (o < 0) throw new Error(`cpudiff: unknown mnemonic ${mn}`);
    if (o > 0xFF) return [0xFD, o & 0xFF];
    return [o];
}

/* Specifier byte helpers -- literal, register, and absolute (@#longword). */
function lit(n) { return [n & 0x3F]; }
function reg(n) { return [0x50 | n]; }
function regd(n) { return [0x60 | n]; }
function abs(a) { return [0x9F, a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF]; }
function ain(n) { return [0x80 | n]; }      // (Rn)+ -- autoincrement, so recq[] gets an entry
function imm(v) { return [0x8F, v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF]; }
function immW(v) { return [0x8F, v & 0xFF, (v >>> 8) & 0xFF]; }
function immB(v) { return [0x8F, v & 0xFF]; }
function asm(...parts) { return [].concat(...parts); }

/** NOP-filled handler that just returns: pop the frame parameters (if any) and REI. */
function handlerREI(nParams)
{
    let code = [];
    if (nParams) code = code.concat(op("ADDL2"), lit(nParams * 4), reg(14));
    return code.concat(op("REI"));
}

const KINDS = ["straight", "arith-trap", "divzro-trap", "trace-trap", "subscr-trap",
               "emulate", "softint", "fpd-resume", "str-cycles"];

/*
 * DETERMINISTIC BOUNDARY WALKS.  Five of the eight prior items in this project had a --selfcheck
 * mutation survive its first run, and in every case the fix was to reach a boundary ON PURPOSE
 * rather than to enlarge the run.  These three lists are cycled by case index, so a run at the
 * MINIMUM `--cases` still visits every entry:
 *
 *   EMULATE_VARIANTS   "fpd" is the only way to reach cpu_emulate_exception()'s TWO-longword frame
 *                      and its SCB_EMULFPD vector; without it a wrong vector or a wrong frame size
 *                      on that branch is untested, because nothing else in this project ever
 *                      presents a CIS opcode with PSL<FPD> already set.
 *   FPD_VARIANTS       "autoinc" gives the faulting MOVC5 a NON-EMPTY recovery queue, which is what
 *                      makes the `if (!(psl & PSL_FPD)) unwind()` guard observable: with an empty
 *                      queue, unwinding unconditionally is a no-op and the guard cannot be graded.
 *   STR_CYCLES_LENGTHS strides the `extra_bytes >> 5` cycle charge across its rounding boundary --
 *                      31 bytes costs nothing extra, 32 costs one, 200 costs six -- so a machine
 *                      that ignores extra_bytes stops `step N` at a different instruction.
 */
const EMULATE_VARIANTS = ["plain", "plain", "fpd"];
const FPD_VARIANTS = ["abs", "autoinc"];
const STR_CYCLES_LENGTHS = [31, 32, 33, 64, 200, 400];

/**
 * A flat (mapping off) case skeleton: SCB at P.SCBB with every vector pointing at P.HANDLER,
 * kernel stack at STK_TOP, code at P.CODE.
 *
 * @param {string} kind
 * @param {number} index
 * @returns {Case}
 */
function flatCase(kind, index)
{
    let c = new Case(kind, index);
    c.pc = P.CODE;
    c.priv.SCBB = P.SCBB;
    c.priv.KSP = STK_TOP;
    c.priv.ESP = c.priv.SSP = c.priv.USP = STK_TOP;
    c.priv.IS = IS_TOP;
    c.priv.MAPEN = 0;
    /*
     * ASTLVL = 4 (AST_MAX) means NO asynchronous system trap is pending.  Leaving it at 0 makes
     * every REI request a level-2 software interrupt on the way out (exc.js's rei() tail, SIMH's
     * op_rei AST check against the NEW mode), so a case whose handler REIs back into a resumed
     * instruction is immediately re-interrupted and never resumes anything.  Both machines do it
     * identically, which is exactly why it is invisible as a comparison failure and shows up only
     * as a case that quietly stops testing what it was built to test.
     */
    c.priv.ASTLVL = 4;
    c.regs[14] = STK_TOP;
    c.psl = (KERN << PSL_V_CUR);
    for (let v = 0; v < 0x100; v += 4) c.setLong(P.SCBB + v, P.HANDLER);
    for (let k = 0; k < 64; k++) c.setLong(P.CODE + k * 4, 0x01010101);        // NOPs
    for (let k = 0; k < 64; k++) c.setLong(P.HANDLER + k * 4, 0x01010101);
    for (let k = 0; k < 32; k++) c.setLong(P.DATA + k * 4, 0);
    c.probeRange(P.DATA, 16);
    for (let k = -14; k <= 1; k++) c.probes.push((STK_TOP + k * 4) >>> 0);
    for (let k = -14; k <= 1; k++) c.probes.push((IS_TOP + k * 4) >>> 0);
    return c;
}


/** The variable-length bit-field opcodes -- computed from the decode ROM's DR_SPFLAG, not listed. */
const VBF_OPCODES = (function() {
    let s = new Set();
    for (let opc = 0; opc < 512; opc++) {
        if (!OPCODES[opc]) continue;
        let hdr = DROM[opc * DROM_STRIDE];
        for (let i = 1; i <= (hdr & DR.NSPMASK); i++) {
            if (DROM[opc * DROM_STRIDE + i] & DR.SPFLAG) { s.add(OPCODES[opc]); break; }
        }
    }
    return s;
})();

/**
 * bitFieldInstruction(mn, opc, ns, rnd, index)
 *
 * EXTV/EXTZV/CMPV/CMPZV/FFS/FFC/INSV with a position inside the case's own data page and a size in
 * 0..32.  The `.vb` base is a REGISTER half the time, which is the case decode.js's contract calls
 * out (`vfldrp1`, the R[(rn+1)&15] capture) and which has no memory reference at all.
 */
function bitFieldInstruction(mn, opc, ns, rnd, index)
{
    let parts = [op(mn)];
    let useReg = rnd() < 0.5;
    let size = Math.floor(rnd() * 33);
    let pos = useReg ? Math.floor(rnd() * 32) : (Math.floor(rnd() * 60) - 30);
    for (let i = 1; i <= ns; i++) {
        let dr = DROM[opc * DROM_STRIDE + i];
        if (dr & DR.SPFLAG) { parts.push(useReg ? reg(Math.floor(rnd() * 8)) : abs(P.DATA + 16)); continue; }
        let ac = dr & DR.ACMASK;
        if (ac === DR.W || ac === DR.M) { parts.push(rnd() < 0.5 ? reg(Math.floor(rnd() * 10)) : abs(P.DATA + 32)); continue; }
        /* The two read specifiers ahead of the .vb base are pos then size, in that order for every
           one of these opcodes except INSV, whose first specifier is the source value. */
        if (mn === "INSV" && i === 1) { parts.push(imm((rnd() * 0x100000000) | 0)); continue; }
        parts.push((parts.length === (mn === "INSV" ? 2 : 1)) ? imm(pos) : lit(size));
    }
    return asm(...parts);
}

/**
 * randomInstruction(mn, rnd, index)
 *
 * Emit one syntactically LEGAL instance of `mn`: each specifier is chosen from the forms its
 * decode ROM ACCESS TYPE actually permits, because the illegal ones are not interesting here --
 * a PC-immediate write destination is a reserved-addressing-mode fault, decodediff.js's FAULTS
 * phase already grades that exhaustively, and letting one into this phase turns every downstream
 * instruction of the program into noise instead of coverage.
 *
 * @param {string} mn
 * @param {function():number} rnd
 * @param {number} index
 * @returns {Array.<number>}
 */
function randomInstruction(mn, rnd, index)
{
    let opc = OPCODES.indexOf(mn);
    let hdr = DROM[opc * DROM_STRIDE];
    let ns = hdr & DR.NSPMASK;
    /*
     * VARIABLE-LENGTH BIT FIELD: the POSITION operand is a signed BIT offset added to the base
     * address, so a random longword position addresses memory 256MB away from the base and the
     * reference is non-existent memory -- which on a real KA655 is a MACHINE CHECK, system-model
     * code exc.js deliberately does not model (its file header says so, and cpustate.js's
     * onBusFault stops the machine rather than inventing one).  SIMH does model it, so such a case
     * compares a machine that dispatched SCB_MCHK against a machine that stopped: a difference in
     * SCOPE, not in correctness, and it would drown out every real finding in this phase.  Bounded
     * here instead; cpu.js's own differential grades the field arithmetic exhaustively.
     */
    if (VBF_OPCODES.has(mn)) return bitFieldInstruction(mn, opc, ns, rnd, index);
    let parts = [op(mn)];
    for (let s = 0; s < ns; s++) {
        let dr = DROM[opc * DROM_STRIDE + s + 1];
        let ac = dr & DR.ACMASK;
        let lnt = 1 << (dr & DR.LNMASK);
        let slot = (index + s) % 6;
        if (ac === DR.W || ac === DR.M || ac === DR.A || (dr & DR.SPFLAG)) {
            /* write / modify / address / variable-bit-field-base: needs a real place */
            if (ac !== DR.A && rnd() < 0.5) parts.push(reg(Math.floor(rnd() * 10)));
            else parts.push(abs(P.DATA + 8 * slot));
        } else if (lnt > 4) {
            /*
             * A PC-IMMEDIATE MUST BE EXACTLY THE SPECIFIER'S LENGTH.  `#imm` for a `.rq` operand is
             * EIGHT bytes, not four; emitting four shifts every following byte of the program, and
             * the resulting garbage instruction addresses memory outside RAM -- a machine check,
             * which exc.js does not model, so the case compares a machine that dispatched against
             * a machine that stopped.  Quad and octa read operands therefore use a register, a
             * short literal (zero-extended to any length) or an absolute address, never `#imm`.
             */
            parts.push(rnd() < 0.4 ? reg(Math.floor(rnd() * 10)) :
                       rnd() < 0.5 ? lit(Math.floor(rnd() * 64)) : abs(P.DATA + 8 * slot));
        } else if (rnd() < 0.45) {
            parts.push(reg(Math.floor(rnd() * 11)));
        } else if (rnd() < 0.5) {
            parts.push(lit(Math.floor(rnd() * 64)));
        } else if (lnt === 1) {
            parts.push(immB((rnd() * 256) | 0));
        } else if (lnt === 2) {
            parts.push(immW((rnd() * 65536) | 0));
        } else {
            parts.push(imm((rnd() * 0x100000000) | 0));
        }
    }
    return asm(...parts);
}

/**
 * buildCase(kind, index, rnd)
 *
 * Each kind is a PROGRAM, not an instruction, and each is aimed at one decision the CPU loop owns.
 *
 * @param {string} kind
 * @param {number} index
 * @param {function():number} rnd
 * @returns {Case}
 */
function buildCase(kind, index, rnd)
{
    let c = flatCase(kind, index);
    let code = [];

    switch (kind) {

    /*
     * A straight run of ordinary instructions with no trap at all.  This is the dispatch table's
     * own grade -- it is the only kind that can catch two opcodes wired to each other's handler --
     * and it is also the control against which every other kind's `step N` accounting is read.
     */
    case "straight": {
        c.steps = 3 + Math.floor(rnd() * 6);
        let pool = ["ADDL2", "SUBL2", "BISL2", "BICL2", "XORL2", "MOVL", "MCOML", "MNEGL",
                    "INCL", "DECL", "ROTL", "ASHL", "MOVAB", "CMPL", "BITL", "TSTL",
                    "MOVW", "MOVB", "CVTBL", "CVTWL", "CVTLB", "MULL2", "PUSHL", "CLRL",
                    "MOVZBL", "MOVZWL", "ADDL3", "SUBL3", "BISL3", "EXTZV", "INSV", "FFS",
                    "MOVQ", "CLRQ", "ADAWI", "BICPSW", "BISPSW", "MOVPSL", "EMUL", "ASHQ"];
        for (let i = 0; i < c.steps + 3; i++) {
            let mn = pool[Math.floor(rnd() * pool.length)];
            code = code.concat(randomInstruction(mn, rnd, index));
        }
        for (let r = 0; r < 11; r++) c.regs[r] = (rnd() * 0x100000000) | 0;
        c.psl |= Math.floor(rnd() * 16);
        break;
    }

    /*
     * Integer overflow with PSW<IV> armed.  The whole point is the NEXT instruction: SIMH sets
     * CC<V>, stores the result, and requests TRAP_INTOV, which the loop delivers at the top of the
     * following instruction through SCB_ARITH with the trap code pushed as a parameter.  Stepping
     * past that boundary is the only way to see it, which is why intdiff.js's single-instruction
     * comparison could not and said so.
     */
    case "arith-trap": {
        let variants = ["ADDL2", "SUBL2", "MULL2", "INCL", "ASHL", "CVTLB", "CVTLW", "MULB2"];
        let mn = variants[index % variants.length];
        c.note = mn;
        c.psl |= PSW_IV;
        c.regs[0] = 0x7FFFFFFF;
        c.regs[1] = 1;
        switch (mn) {
        case "ADDL2": code = asm(op("ADDL2"), reg(1), reg(0)); break;
        case "SUBL2": c.regs[0] = 0x80000000 | 0; code = asm(op("SUBL2"), reg(1), reg(0)); break;
        case "MULL2": c.regs[1] = 0x40000000; code = asm(op("MULL2"), reg(1), reg(0)); break;
        case "INCL":  code = asm(op("INCL"), reg(0)); break;
        case "ASHL":  code = asm(op("ASHL"), lit(4), reg(0), reg(2)); break;
        case "CVTLB": code = asm(op("CVTLB"), reg(0), reg(2)); break;
        case "CVTLW": code = asm(op("CVTLW"), reg(0), reg(2)); break;
        case "MULB2": c.regs[0] = 0x7F; c.regs[1] = 0x7F; code = asm(op("MULB2"), reg(1), reg(0)); break;
        }
        /* Instructions after the trapping one, so a machine that FAILS to trap runs into them and
           the register file says so. */
        code = code.concat(asm(op("MOVL"), imm(0x11111111), reg(3)));
        code = code.concat(asm(op("MOVL"), imm(0x22222222), reg(4)));
        /* Handler: pop the one trap-code parameter, mark R5, REI. */
        let h = asm(op("MOVL"), regd(14), reg(5), handlerREI(1));
        c.setBytes(P.HANDLER, h);
        c.steps = 2 + Math.floor(rnd() * 4);
        break;
    }

    /* Divide by zero: the trap request is UNCONDITIONAL, unlike overflow.  A machine that gates it
       on PSW<IV> passes every IV-set case and fails exactly these. */
    case "divzro-trap": {
        c.psl |= (rnd() < 0.5) ? PSW_IV : 0;
        c.note = (rnd() < 0.5) ? "DIVL2" : "EDIV";
        c.regs[0] = 0;
        c.regs[1] = 0x1234;
        if (c.note === "DIVL2") code = asm(op("DIVL2"), reg(0), reg(1));
        else code = asm(op("EDIV"), reg(0), reg(1), reg(6), reg(7));
        code = code.concat(asm(op("MOVL"), imm(0x33333333), reg(3)));
        c.setBytes(P.HANDLER, asm(op("MOVL"), regd(14), reg(5), handlerREI(1)));
        c.steps = 2 + Math.floor(rnd() * 3);
        break;
    }

    /* PSW<T>: the loop arms PSL<TP> at the top of one instruction and takes SCB_TP at the top of
       the NEXT one.  Two instruction boundaries, so a single-instruction harness sees neither. */
    case "trace-trap": {
        c.psl |= PSW_T;
        code = asm(op("MOVL"), imm(0x44444444), reg(3));
        code = code.concat(asm(op("MOVL"), imm(0x55555555), reg(4)));
        code = code.concat(asm(op("MOVL"), imm(0x66666666), reg(6)));
        c.setBytes(P.HANDLER, asm(op("MOVL"), imm(0x77777777), reg(7), op("REI")));
        c.steps = 2 + Math.floor(rnd() * 4);
        break;
    }

    /* INDEX out of range requests TRAP_SUBSCR -- a third trap code through the same SCB_ARITH
       vector, with a different parameter. */
    case "subscr-trap": {
        c.regs[0] = (rnd() < 0.5) ? -5 : 500;               // subscript, outside [0,100]
        /* Every INDEX specifier is `.rl`: a PC-immediate for one MUST be four bytes, or the
           decoder reads the following specifier's bytes as the immediate's tail and the whole
           program shifts.  Short literals (mode 0) are length-independent and used where possible. */
        code = asm(op("INDEX"), reg(0), lit(0), imm(100), lit(1), lit(0), reg(2));
        code = code.concat(asm(op("MOVL"), imm(0x88888888), reg(3)));
        c.setBytes(P.HANDLER, asm(op("MOVL"), regd(14), reg(5), handlerREI(1)));
        c.steps = 2 + Math.floor(rnd() * 3);
        break;
    }

    /*
     * A packed-decimal opcode on a CPU without CIS: `cpu_emulate_exception()` builds a TWELVE
     * longword frame at fixed negative offsets from SP (with a two-longword HOLE between the
     * operands and the PC/PSL pair) and vectors through SCB_EMULATE, without ever calling
     * intexc().  The whole frame is in the stack probe window, so an off-by-one offset, a wrong
     * SP decrement or a missing CVTPL fixup all fail here.
     */
    case "emulate": {
        let variants = ["CMPP3", "MOVP", "CVTLP", "CVTPL", "ADDP4", "EDITPC", "MATCHC"];
        let mn = variants[index % variants.length];
        let ev = EMULATE_VARIANTS[(index / KINDS.length | 0) % EMULATE_VARIANTS.length];
        c.note = mn + " " + ev;
        let ns = DROM[OPCODES.indexOf(mn) * DROM_STRIDE] & DR.NSPMASK;
        let parts = [op(mn)];
        for (let s = 0; s < ns; s++) {
            let dr = DROM[OPCODES.indexOf(mn) * DROM_STRIDE + s + 1];
            let ac = dr & DR.ACMASK;
            if (ac === DR.W) parts.push(rnd() < 0.5 ? reg(6 + (s % 4)) : abs(P.DATA + 4 * s));
            else if (ac === DR.A) parts.push(abs(P.DATA + 4 * s));
            else parts.push(reg(s % 6));
        }
        code = asm(...parts);
        code = code.concat(asm(op("MOVL"), imm(0x99999999), reg(11)));
        for (let r = 0; r < 8; r++) c.regs[r] = (rnd() * 0x100000000) | 0;
        /* DIFFERENT handlers for the two vectors, so choosing the wrong one moves the PC. */
        c.setLong(P.SCBB + 0xC8, P.HANDLER);
        c.setLong(P.SCBB + 0xCC, P.HANDLER + 0x40);
        c.setBytes(P.HANDLER, asm(op("MOVL"), imm(0xAAAAAAAA), reg(10), op("HALT")));
        c.setBytes(P.HANDLER + 0x40, asm(op("MOVL"), imm(0xFDFDFDFD), reg(10), op("HALT")));
        if (ev === "fpd") {
            /* PSL<FPD> already set: decode() resolves NO specifiers (the opcode's DR_F bit allows
               it) and cpu_emulate_exception() takes its two-longword branch through SCB_EMULFPD. */
            c.psl |= PSL_FPD;
        }
        c.steps = 2;
        /*
         * `cpu_emulate_exception()` pushes SIX operand slots regardless of how many the opcode
         * declares, so slots `ns..5` are whatever the PREVIOUS decode left in `opnd[]` -- a stale
         * C array on SIMH's side and a stale Int32Array on ours, with no reason to agree after a
         * `reset all`.  Those frame words are dropped from the comparison BY ADDRESS, computed
         * from the opcode's own specifier count; every other word of the frame (the opcode, the
         * old PC, the declared operands, the new PC, the PSL) and the SP decrement are compared.
         */
        let frame = STK_TOP - 48;
        for (let k = ns; k < 6; k++) {
            let a = (frame + 8 + 4 * k) >>> 0;
            let i = c.probes.indexOf(a);
            if (i >= 0) c.probes.splice(i, 1);
        }
        break;
    }

    /*
     * MTPR to SIRR requests a software interrupt at a chosen level; the loop's IPL arbitration
     * decides whether to take it BEFORE the next instruction.  Half the cases raise the IPL above
     * the request so nothing is taken -- a machine that always dispatches fails those and only
     * those.
     */
    case "softint": {
        let lvl = 1 + Math.floor(rnd() * 15);
        let ipl = (rnd() < 0.5) ? 0 : lvl + Math.floor(rnd() * 3);
        c.note = `lvl=${lvl} ipl=${ipl}`;
        c.psl |= (Math.min(ipl, 0x1F) << PSL_V_IPL);
        code = asm(op("MTPR"), lit(lvl), lit(20));           // 20 = SIRR; both specifiers are .rl
        code = code.concat(asm(op("MOVL"), imm(0xBBBBBBBB), reg(3)));
        code = code.concat(asm(op("MOVL"), imm(0xCCCCCCCC), reg(4)));
        c.setBytes(P.HANDLER, asm(op("MOVL"), imm(0xDDDDDDDD), reg(7), op("REI")));
        c.steps = 3 + Math.floor(rnd() * 3);
        break;
    }

    /*
     * `extra_bytes` and the cycle charge.  A string instruction costs `1 + (extra_bytes >> 5)`
     * sim_interval units, charged on the NEXT instruction (vax_cpu.c:729-730), and SCP's `step N`
     * counts those units -- so with a long enough MOVC3 the two machines stop at DIFFERENT
     * instructions unless the accounting matches byte for byte.  The trailing markers make where
     * each stopped observable in the register file.
     */
    case "str-cycles": {
        let len = STR_CYCLES_LENGTHS[index % STR_CYCLES_LENGTHS.length];
        c.note = `len=${len}`;
        for (let i = 0; i < len; i++) {
            let a = P.DATA + i, la = a & ~3, sh = (a & 3) * 8;
            let cur = c.mem.get(la) || 0;
            c.mem.set(la, ((cur & ~(0xFF << sh)) | (((0x41 + (i % 26)) & 0xFF) << sh)) | 0);
        }
        code = asm(op("MOVC3"), immW(len), abs(P.DATA), abs(P.DATA + 0x400));
        for (let k = 0; k < 8; k++) code = code.concat(asm(op("MOVL"), imm(0x1000 + k), reg(6)));
        c.probes = [];
        c.probeRange(P.DATA, 8);
        c.probeRange(P.DATA + 0x400, Math.ceil(len / 4) + 2);
        for (let k = -14; k <= 1; k++) c.probes.push((STK_TOP + k * 4) >>> 0);
        c.steps = 2 + (index % 6);
        break;
    }

    /*
     * THE FPD RESUME.  A MOVC5 whose destination runs off the end of a valid page into an INVALID
     * one.  MOVC5 sets PSL<FPD> and packs `PC - fault_PC` into R0<31:24> before its copy loop, the
     * loop faults part way through, the abort handler restores PC to fault_PC WITHOUT unwinding
     * (FPD is set), the TNV handler makes the page valid and REIs, and the instruction re-decodes
     * with FPD set and resumes at `fault_PC + STR_GETDPC(R0)`.
     *
     * Every one of those five steps is this item's, and none of them can be exercised without a
     * loop: strq.js's own differential had to SKIP every real resume for exactly that reason.  If
     * the delta-PC is packed as zero (which is what strq.js did before this item), the resumed
     * instruction returns into the middle of its own operand specifiers and the register file
     * diverges within two instructions.
     */
    case "fpd-resume": {
        c.priv.MAPEN = 1;
        c.priv.SBR = P.SPT;
        /*
         * The page table must map ITSELF: the TNV handler's whole job is to write a valid PTE into
         * it and then TBIS, and it does that through an S0 virtual address.  A table sized to cover
         * only the code/data/stack pages leaves its own page beyond SLR, so the handler's very
         * first instruction takes a length violation, re-dispatches, and the case degenerates into
         * an infinite fault loop that LOOKS like a passing comparison because both machines loop
         * identically.  128 entries covers P.SCBB..P.SPT+table.
         */
        let nPages = 128;
        c.priv.SLR = nPages;
        c.pc = m_va(P.CODE);
        c.priv.SCBB = P.SCBB;                               // SCBB is PHYSICAL
        c.priv.KSP = m_va(STK_TOP);
        c.priv.ESP = c.priv.SSP = c.priv.USP = m_va(STK_TOP);
        c.priv.IS = m_va(IS_TOP);
        c.regs[14] = c.priv.KSP;

        let badVPN = ((P.DATA2 - P.SCBB) / PAGE) | 0;
        for (let i = 0; i < nPages; i++) {
            c.setLong(P.SPT + 4 * i, makePTE(M_PFN0 + i, PROT.all, i !== badVPN));
        }
        /* Every vector -> the handler, on the KERNEL stack (bit 0 clear). */
        for (let v = 0; v < 0x100; v += 4) c.setLong(P.SCBB + v, m_va(P.HANDLER) & ~1);

        /* The copy: source in DATA, destination straddling DATA2's boundary. */
        let variant = FPD_VARIANTS[index % FPD_VARIANTS.length];
        let srcLen = 24 + Math.floor(rnd() * 40);
        let dstOff = PAGE - 8 - Math.floor(rnd() * 8);
        let fill = 0x20 + Math.floor(rnd() * 0x40);
        c.note = `len=${srcLen} dstOff=${dstOff} fill=${hex(fill, 2)}`;
        let src = m_va(P.DATA);
        let dst = (m_va(P.DATA) + dstOff) | 0;
        for (let i = 0; i < srcLen + 8; i++) {
            let a = P.DATA + i;
            let la = a & ~3, sh = (a & 3) * 8;
            let cur = c.mem.get(la) || 0;
            c.mem.set(la, ((cur & ~(0xFF << sh)) | (((0x41 + (i % 26)) & 0xFF) << sh)) | 0);
        }
        if (variant === "autoinc") {
            /* `(R6)+` / `(R8)+` for the two `.ab` address specifiers: resolving them INCREMENTS R6
               and R8 before the copy loop faults, so recq[] is non-empty when the abort handler
               runs.  Those increments must SURVIVE -- PSL<FPD> is set, the instruction is resumed
               rather than restarted -- and a handler that unwinds anyway puts the addresses back
               and the resumed copy reads and writes the wrong place. */
            c.regs[6] = src;
            c.regs[8] = dst;
            code = asm(op("MOVC5"), immW(srcLen), ain(6), immB(fill), immW(srcLen + 6), ain(8));
        } else {
            code = asm(op("MOVC5"), immW(srcLen), abs(src), immB(fill), immW(srcLen + 6), abs(dst));
        }
        c.note += " " + variant;
        code = code.concat(asm(op("MOVL"), imm(0xEEEEEEEE), reg(11)));

        /*
         * The TNV handler: make the bad page valid, flush the TB entry for it, drop the two fault
         * parameters, and REI.  Written with absolute (@#) specifiers so nothing depends on a
         * register the faulting instruction owns -- MOVC5 uses R0-R5 as its resume state and
         * clobbering any of them would destroy the resume rather than test it.
         */
        let h = asm(op("MOVL"), imm(makePTE(M_PFN0 + badVPN, PROT.all, true)), abs(m_va(P.SPT + 4 * badVPN)));
        /* Both MTPR specifiers are `.rl`; 58 fits a short literal, the address needs four bytes. */
        h = h.concat(asm(op("MTPR"), imm(m_va(P.DATA2)), lit(58)));     // 58 = TBIS
        h = h.concat(asm(op("ADDL2"), lit(8), reg(14)));
        h = h.concat(op("REI"));
        c.setBytes(P.HANDLER, h);

        c.probes = [];
        c.probeRange(P.DATA, 48);
        c.probeRange(P.DATA2, 24);
        c.probeRange(P.SPT + 4 * badVPN, 1);
        for (let k = -12; k <= 1; k++) c.probes.push((STK_TOP + k * 4) >>> 0);
        c.steps = 14 + Math.floor(rnd() * 8);
        break;
    }
    }

    c.setBytes(P.CODE, code);
    /*
     * DEPOSIT ZERO TO EVERY PROBE ADDRESS THE CASE DOES NOT OTHERWISE SET.  `reset all` does not
     * clear SIMH's memory, so a stack window an earlier case pushed frames into still holds them
     * -- and the JS machine, whose bus this case writes only where c.mem says, does not.  That is
     * not a divergence, it is two different starting states; every byte a case compares must be
     * deposited by that case.
     */
    for (let a of c.probes) if (!c.mem.has(a >>> 0)) c.setLong(a, 0);
    c.regs[15] = c.pc;
    return c;
}

/* --- SIMH side ----------------------------------------------------------------------------- */

const CASE_MARK = "CPUCASE_";

function buildScript(cases)
{
    let L = ["set cpu 16m", "set cpu simhalt"];
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        L.push("reset all");
        for (let p of PRIV) L.push(`deposit ${p.name} ${hex(c.priv[p.name])}`);
        for (let r = 0; r < 15; r++) L.push(`deposit R${r} ${hex(c.regs[r])}`);
        for (let [a, v] of c.mem) L.push(`deposit ${hex(a)} ${hex(v)}`);
        L.push(`deposit PSL ${hex(c.psl)}`);
        L.push(`deposit PC ${hex(c.pc)}`);
        L.push(`step ${c.steps}`);
        let names = [];
        for (let r = 0; r < 15; r++) names.push("R" + r);
        names.push("PC", "PSL");
        for (let p of PRIV) names.push(p.name);
        L.push(`examine -h ${names.join(",")}`);
        for (let a of c.probes) L.push(`examine -h ${hex(a)}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/;

function runBatch(simh, cases, scratch)
{
    let out = runSimh(simh, buildScript(cases), path.join(scratch, "cpudiff-batch.ini"));
    if (process.env['CPUDIFF_DUMP']) fs.writeFileSync(process.env['CPUDIFF_DUMP'] + "." + cases[0].index, out);
    let lines = out.split("\n");
    let results = new Map();
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\d+)"));
        if (!m) { i++; continue; }
        let idx = +m[1];
        let c = cases.find((x) => x.index === idx);
        let want = 17 + PRIV.length + c.probes.length;
        i++;
        let vals = [];
        while (i < lines.length && vals.length < want) {
            if (lines[i].indexOf(CASE_MARK) >= 0) break;
            let vm = lines[i].match(VALUE_RE);
            if (vm) vals.push(parseInt(vm[2], 16) | 0);
            i++;
        }
        if (vals.length < want) { results.set(idx, {reached: false, got: vals.length, want}); continue; }
        let regs = new Int32Array(16);
        for (let r = 0; r < 15; r++) regs[r] = vals[r];
        regs[15] = vals[15];
        let priv = {};
        for (let k = 0; k < PRIV.length; k++) priv[PRIV[k].name] = vals[17 + k];
        results.set(idx, {reached: true, regs, psl: vals[16], priv, mem: vals.slice(17 + PRIV.length)});
    }
    return results;
}

/* --- JS side ------------------------------------------------------------------------------- */

function runJsCase(m, c)
{
    let {cpu, bus} = m;
    cpu.reset();
    cpu.mmu.reset();
    for (let [a, v] of c.mem) cpu.mmu.writeL(a >>> 0, v | 0);
    for (let p of PRIV) p.set(cpu, c.priv[p.name] | 0);
    for (let r = 0; r < 15; r++) cpu.regs[r] = c.regs[r];
    cpu.psl = c.psl | 0;
    cpu.setPC(c.pc);
    let stop = null;
    try {
        cpu.stepCPU(c.steps);
    } catch (e) {
        if (!(e instanceof VAXStop)) return {err: e};
        stop = e;
    }
    let priv = {};
    for (let p of PRIV) priv[p.name] = p.get(cpu) | 0;
    return {
        reached: true, stop,
        regs: Int32Array.from(cpu.regs), psl: cpu.psl | 0, priv,
        mem: c.probes.map((a) => cpu.mmu.readL(a >>> 0) | 0)
    };
}

function compareCase(c, js, simh)
{
    let problems = [];
    if (!simh.reached) return [`SIMH produced ${simh.got}/${simh.want} values -- case never reached comparison`];
    if (js.err) return [`JS threw ${js.err.name}: ${js.err.message}`];
    for (let r = 0; r < 16; r++) {
        if (js.regs[r] !== simh.regs[r]) problems.push(`R${r}: JS=${hex(js.regs[r])} SIMH=${hex(simh.regs[r])}`);
    }
    if ((js.psl >>> 0) !== (simh.psl >>> 0)) problems.push(`PSL: JS=${hex(js.psl)} SIMH=${hex(simh.psl)}`);
    for (let p of PRIV) {
        let mask = p.mask === undefined ? 0xFFFFFFFF : p.mask;
        let a = (js.priv[p.name] & mask) >>> 0, b = (simh.priv[p.name] & mask) >>> 0;
        if (a !== b) problems.push(`${p.name}: JS=${hex(a)} SIMH=${hex(b)}`);
    }
    for (let k = 0; k < c.probes.length; k++) {
        if ((js.mem[k] >>> 0) !== (simh.mem[k] >>> 0)) {
            problems.push(`M[${hex(c.probes[k])}]: JS=${hex(js.mem[k])} SIMH=${hex(simh.mem[k])}`);
        }
    }
    return problems;
}

/**
 * isNonTrivial(c, js)
 *
 * A case that changed nothing proves nothing.  Used for the non-trivial-fraction floor.
 */
function isNonTrivial(c, js)
{
    for (let r = 0; r < 15; r++) if (js.regs[r] !== c.regs[r]) return true;
    if ((js.psl >>> 0) !== (c.psl >>> 0)) return true;
    for (let v of js.mem) if (v !== 0) return true;
    return false;
}

function phaseRandomized(simh, opts, mutation)
{
    let rnd = mulberry32(opts.seed ^ 0x0C05C0DE);
    let cases = [];
    for (let i = 0; i < opts.cases; i++) cases.push(buildCase(KINDS[i % KINDS.length], i, rnd));

    let m = makeMachine();
    if (mutation) applyMutation(m.cpu, mutation);

    let failures = [];
    let perKind = new Map();
    let nonTrivial = 0, compared = 0, notReached = [];
    const BATCH = 40;
    for (let b = 0; b < cases.length; b += BATCH) {
        let batch = cases.slice(b, b + BATCH);
        let results = runBatch(simh, batch, opts.scratch);
        for (let c of batch) {
            let simhR = results.get(c.index);
            if (!simhR) { notReached.push(`${c.index} (${c.kind})`); continue; }
            let js = runJsCase(m, c);
            let stat = perKind.get(c.kind) || {n: 0, nontrivial: 0};
            stat.n++;
            perKind.set(c.kind, stat);
            if (!simhR.reached) { notReached.push(`${c.index} (${c.kind})`); continue; }
            compared++;
            let probs = compareCase(c, js, simhR);
            if (probs.length) {
                failures.push(`case ${c.index} [${c.kind}${c.note ? " " + c.note : ""}] steps=${c.steps}: ` +
                              probs.slice(0, 6).join("; "));
            } else if (isNonTrivial(c, js)) {
                stat.nontrivial++;
                nonTrivial++;
            }
        }
    }
    return {failures, perKind, nonTrivial, compared, notReached, total: cases.length};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 3 -- inIE reset at the stepCPU() call boundary (pcjsvax-1be)                             *
 *                                                                                                 *
 * DETERMINISTIC, not randomized -- this grades one specific boundary vax_cpu.c:514 draws, not a   *
 * distribution, and a defect here does not get more or less visible with more cases.  The         *
 * sequence:                                                                                       *
 *                                                                                                 *
 *   1. PSL=0 (kernel, IPL 0), R14=INIE_R14 (outside the 16MB this test maps -- always unbacked),   *
 *      SISR<INIE_SISR_LVL> set (a software interrupt immediately eligible, since the running IPL   *
 *      is 0), IS=INIE_IS (a VALID, well-backed interrupt-stack pointer), PC=INIE_PC, and an        *
 *      ordinary `MOVL #imm, @#INIE_R14` sitting at INIE_PC -- the SAME unbacked address, so the    *
 *      second step's fault is the plainest possible unbacked write, not a second special case.     *
 *                                                                                                 *
 *   2. FIRST stepCPU(1)/`step 1`: setIRQL() sees the pending softint and intexc() dispatches it;   *
 *      its own frame push (to R14, unbacked) faults WHILE intexc() has already set inIE=1 and      *
 *      before it reaches its own clear -- so takeFault() sees inIE already set and BOTH machines   *
 *      take STOP_INIE here, identically, whether or not pcjsvax-1be's fix exists.  This step        *
 *      establishes the poisoned precondition; it is not itself the thing being graded, which is    *
 *      why its own outcome is asserted (a setup self-check) rather than silently trusted.           *
 *                                                                                                 *
 *   3. SECOND stepCPU(1)/`step 1`, NO reset in between: PC is still INIE_PC (the failed intexc()    *
 *      never reached setPC()), so this executes the `MOVL` sitting there.  Its write faults again   *
 *      (same unbacked address) -- an entirely ordinary machine check, dispatched onto IS, which     *
 *      IS backed, so the dispatch itself succeeds on a machine that resets inIE at this call's      *
 *      entry.  Real SIMH does (vax_cpu.c:514); a stepCPU() that does not finds inIE still 1 from     *
 *      step 2 and throws STOP_INIE immediately, before intexc() is even attempted -- a step that     *
 *      should succeed instead stops the machine.  This is the actual grade.                         *
 *                                                                                                 *
 * Every step's PC/PSL/R14/SISR is compared, not merely the last one, so a divergence is             *
 * attributed to the step that produced it.                                                          *
 * ------------------------------------------------------------------------------------------- */

const INIE_R14 = 0x1FFFFFF0 | 0;         // outside the 16MB RAM both machines map -- always unbacked
const INIE_IS = 0x00002000;              // a valid interrupt-stack pointer, well inside RAM
const INIE_SISR_LVL = 4;                 // any softint level 1..15, below the IPL=0 both machines run at
const INIE_PC = 0x1000;
const INIE_FIELDS = ["PC", "PSL", "R14", "SISR"];

/** The one instruction both steps fault on: an ordinary write to the same unbacked address. */
function inIEResetCode() { return asm(op("MOVL"), imm(0x12345678), abs(INIE_R14)); }

/** The instruction bytes, packed into longwords at their (possibly unaligned) addresses. */
function inIEResetMem()
{
    let code = inIEResetCode();
    let mem = new Map();
    for (let i = 0; i < code.length; i++) {
        let a = (INIE_PC + i) >>> 0, la = a & ~3, sh = (a & 3) * 8;
        let cur = mem.get(la) || 0;
        mem.set(la, ((cur & ~(0xFF << sh)) | ((code[i] & 0xFF) << sh)) | 0);
    }
    return mem;
}

/**
 * runSimhInIEReset(simh, opts)
 *
 * ONE `reset all`, then TWO `step 1` commands with nothing reset between them -- the oracle side
 * of the sequence above.
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Object} {1: {PC,PSL,R14,SISR}, 2: {...}}
 */
function runSimhInIEReset(simh, opts)
{
    let L = ["set cpu 16m", "set cpu simhalt", "reset all",
             `deposit SISR ${hex(1 << INIE_SISR_LVL)}`, "deposit PSL 0",
             `deposit R14 ${hex(INIE_R14)}`, `deposit IS ${hex(INIE_IS)}`, `deposit PC ${hex(INIE_PC)}`];
    for (let [a, v] of inIEResetMem()) L.push(`deposit ${hex(a)} ${hex(v)}`);
    let MARK = "INIE_STEP";
    for (let step of [1, 2]) {
        L.push(`echo ${MARK}${step}`, "step 1", `examine -h ${INIE_FIELDS.join(",")}`);
    }
    L.push("quit");
    let out = runSimh(simh, L.join("\n") + "\n", path.join(opts.scratch, "cpudiff-inie.ini"));
    let steps = {1: {}, 2: {}};
    let cur = 0, field = 0;
    for (let line of out.split("\n")) {
        let mm = line.match(new RegExp(MARK + "(\\d)"));
        if (mm) { cur = +mm[1]; field = 0; continue; }
        if (!cur || field >= INIE_FIELDS.length) continue;
        let vm = line.match(VALUE_RE);
        if (vm) { steps[cur][INIE_FIELDS[field]] = parseInt(vm[2], 16) >>> 0; field++; }
    }
    for (let step of [1, 2]) {
        if (Object.keys(steps[step]).length !== INIE_FIELDS.length) {
            throw new Error(`cpudiff: INIE-RESET step ${step} produced ` +
                             `${Object.keys(steps[step]).length}/${INIE_FIELDS.length} SIMH values -- ` +
                             `parse failed, see raw output:\n${out}`);
        }
    }
    return steps;
}

/**
 * runJsInIEReset(mutation)
 *
 * The same deposit, then TWO `stepCPU(1)` calls with no `cpu.reset()` between them.
 *
 * @param {?function(Object):function()} mutation applied to the fresh cpu before stepping, undone after
 * @returns {Object} {1: {PC,PSL,R14,SISR,stop}, 2: {...}}
 */
function runJsInIEReset(mutation)
{
    let {cpu} = makeMachine();
    let undo = mutation ? mutation(cpu) : null;
    try {
        cpu.psl = 0;
        cpu.regs[14] = INIE_R14;
        cpu.exc.stk[4] = INIE_IS;
        cpu.exc.sisr = 1 << INIE_SISR_LVL;
        cpu.setPC(INIE_PC);
        let code = inIEResetCode();
        for (let i = 0; i < code.length; i++) cpu.mmu.writeB((INIE_PC + i) >>> 0, code[i]);

        let capture = (stop) => ({
            PC: cpu.regs[15] >>> 0, PSL: cpu.psl >>> 0, R14: cpu.regs[14] >>> 0,
            SISR: cpu.exc.sisr >>> 0, stop
        });
        let steps = {};
        for (let step of [1, 2]) {
            try { cpu.stepCPU(1); steps[step] = capture(null); }
            catch (e) { if (!(e instanceof VAXStop)) throw e; steps[step] = capture(e); }
        }
        return steps;
    } finally {
        if (undo) undo();
    }
}

/**
 * phaseInIEReset(simh, opts, mutation)
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {?function(Object):function()} mutation
 * @returns {Array.<string>} failures
 */
function phaseInIEReset(simh, opts, mutation)
{
    let simhSteps = runSimhInIEReset(simh, opts);
    let jsSteps = runJsInIEReset(mutation);
    let failures = [];

    /* The setup itself must land on the intended boundary -- step 1 is IDENTICAL with or without
       the fix (see the header), so if it is not STOP_INIE the SETUP is broken, not the fix. */
    if (!(jsSteps[1].stop instanceof VAXStop) || jsSteps[1].stop.reason !== VAXStop.REASON.INIE) {
        failures.push(`INIE-RESET: step 1 did not land on the intended STOP_INIE boundary ` +
                      `(JS stop=${jsSteps[1].stop ? jsSteps[1].stop.reason : "(none, ran to completion)"}) ` +
                      `-- the test's own setup is broken, not necessarily the fix`);
        return failures;
    }

    for (let step of [1, 2]) {
        for (let f of INIE_FIELDS) {
            if ((jsSteps[step][f] >>> 0) !== (simhSteps[step][f] >>> 0)) {
                failures.push(`INIE-RESET: step ${step} ${f}: JS=${hex(jsSteps[step][f])} ` +
                              `SIMH=${hex(simhSteps[step][f])}` +
                              (jsSteps[step].stop ? ` (JS stop=${jsSteps[step].stop.reason})` : ""));
            }
        }
    }
    return failures;
}

/**
 * applyInIEMutation(cpu)
 *
 * Defeats EXACTLY stepCPU()'s own `this.exc.inIE = 0` entry reset -- the one line pcjsvax-1be
 * added -- by swallowing the FIRST write made to `exc.inIE` inside EACH stepCPU() call (that
 * write is textually the fix's own reset, since it is the function's first statement) and passing
 * every OTHER write straight through unmodified: intexc()'s own success-path clear, exc.reset()'s
 * clear, and takeFault()'s assignments all still run for real.  This perturbs the real VAXExc
 * instance and the real CPUStateVAX.prototype.stepCPU through a property/method wrap; it never
 * substitutes a rewritten copy of stepCPU, intexc() or takeFault() (standing rule #11) -- every
 * line of the shipped dispatch still runs, this only un-does the one assignment under test.
 *
 * @param {Object} cpu
 * @returns {function()} undo
 */
function applyInIEMutation(cpu)
{
    let backing = cpu.exc.inIE | 0;
    let swallowNext = false;
    let origDesc = Object.getOwnPropertyDescriptor(cpu.exc, "inIE");
    Object.defineProperty(cpu.exc, "inIE", {
        configurable: true,
        get() { return backing; },
        set(v) { if (swallowNext) { swallowNext = false; return; } backing = v; }
    });
    let origStepCPU = CPUStateVAX.prototype.stepCPU;
    CPUStateVAX.prototype.stepCPU = function(n) {
        swallowNext = true;                  // arm: the NEXT write is stepCPU()'s own entry reset
        try {
            return origStepCPU.call(this, n);
        } finally {
            swallowNext = false;             // disarm even if stepCPU threw before reaching it
        }
    };
    return () => {
        CPUStateVAX.prototype.stepCPU = origStepCPU;
        if (origDesc) Object.defineProperty(cpu.exc, "inIE", origDesc); else delete cpu.exc.inIE;
        cpu.exc.inIE = backing;
    };
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 4 -- --selfcheck                                                                         *
 *                                                                                                *
 * Every mutation below is applied to the SHIPPED objects -- CPUStateVAX.prototype, the DISPATCH   *
 * table cpustate.js exports, and the trap/resume behavior this item added -- not to a copy.  Five *
 * of the eight prior items in this project had a mutation SURVIVE its first run, every one a real *
 * coverage hole, so the fixes below are DETERMINISTIC boundary walks (the arith-trap and          *
 * emulate kinds cycle their variants by case index; the fpd-resume kind is generated every 8th    *
 * case) rather than a bigger `--cases`, which would make the self-check pass while leaving a      *
 * floor-sized run blind.                                                                          *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* The dispatch table itself: two opcodes wired to each other's body. */
    "dispatch-swap": (cpu) => {
        let a = OPCODES.indexOf("ADDL2"), b = OPCODES.indexOf("SUBL2");
        let t = DISPATCH[a]; DISPATCH[a] = DISPATCH[b]; DISPATCH[b] = t;
        return () => { let t2 = DISPATCH[a]; DISPATCH[a] = DISPATCH[b]; DISPATCH[b] = t2; };
    },
    /* fault_PC = the PC AFTER decode instead of the instruction's own address. */
    "faultpc-after-decode": (cpu) => {
        let orig = cpu.exc.stepInstruction;
        cpu.exc.stepInstruction = function(c, execute) {
            return orig.call(this, c, (opc, d, cc) => {
                this.faultPC = cc.regs[15];
                return execute(opc, d, cc);
            });
        };
        return () => { cpu.exc.stepInstruction = orig; };
    },
    /* Unwind the recovery queue even when PSL<FPD> is set -- the guard decode.js's header calls
       "not optional". */
    "unwind-ignores-fpd": (cpu) => {
        let orig = cpu.exc.takeFault;
        cpu.exc.takeFault = function(c, f) {
            c.decoder.unwind();
            return orig.call(this, c, f);
        };
        return () => { cpu.exc.takeFault = orig; };
    },
    /* Integer overflow requests the trap unconditionally, ignoring PSW<IV>. */
    "intov-ignores-iv": (cpu) => {
        let orig = Object.getOwnPropertyDescriptor(CPUStateVAX.prototype, "trpirq");
        Object.defineProperty(cpu, "trpirq", {
            configurable: true,
            get() { return this.exc.trpirq; },
            set(v) { this.exc.trpirq = (this.exc.trpirq & 0x1F) | 0x20; }
        });
        return () => { delete cpu.trpirq; };
    },
    /* Divide-by-zero requests overflow's trap code instead of its own. */
    "divzro-wrong-code": (cpu) => {
        let orig = cpu.exc.setTrap;
        Object.defineProperty(cpu, "trpirq", {
            configurable: true,
            get() { return this.exc.trpirq; },
            set(v) { this.exc.trpirq = ((v & 0x1F) | ((v & 0xE0) ? 0x20 : 0)) | 0; }
        });
        return () => { delete cpu.trpirq; };
    },
    /* The emulate frame is 44 bytes deep instead of 48. */
    "emulate-frame-44": (cpu) => {
        let orig = CPUStateVAX.prototype.emulateException;
        CPUStateVAX.prototype.emulateException = function(opc, d) {
            let r = orig.call(this, opc, d);
            if (!(this.psl & PSL_FPD)) this.regs[14] = (this.regs[14] + 4) | 0;
            return r;
        };
        return () => { CPUStateVAX.prototype.emulateException = orig; };
    },
    /* The FPD emulate frame vectors through SCB_EMULATE instead of SCB_EMULFPD. */
    "emulate-fpd-wrong-vector": (cpu) => {
        let orig = CPUStateVAX.prototype.emulateException;
        CPUStateVAX.prototype.emulateException = function(opc, d) {
            let saved = this.psl;
            this.psl = saved & ~PSL_FPD;
            let r = orig.call(this, opc, d);
            return r;
        };
        return () => { CPUStateVAX.prototype.emulateException = orig; };
    },
    /* CVTPL's one's-complement destination fixup dropped. */
    "emulate-no-cvtpl-fixup": (cpu) => {
        let orig = CPUStateVAX.prototype.emulateException;
        let cvtpl = OPCODES.indexOf("CVTPL");
        CPUStateVAX.prototype.emulateException = function(o, d) {
            return orig.call(this, o === cvtpl ? -1 : o, d);
        };
        return () => { CPUStateVAX.prototype.emulateException = orig; };
    },
    /* The string FPD resume returns to fault_PC without the packed delta -- i.e. exactly the
       pre-pcjsvax-c05 behavior strq.js's header used to document as a known gap. */
    "fpd-resume-no-delta": (cpu) => {
        let orig = Object.getOwnPropertyDescriptor(CPUStateVAX.prototype, "faultPC");
        Object.defineProperty(cpu, "faultPC", {
            configurable: true,
            get() { return (this.psl & PSL_FPD) ? this.exc.faultPC - ((this.regs[0] >>> 24) & 0xFF) : this.exc.faultPC; }
        });
        return () => { delete cpu.faultPC; };
    },
    /* An unimplemented CIS opcode takes a reserved-instruction fault instead of the emulate trap
       -- which is what a dispatcher that keys on "no handler" alone would do. */
    "cis-no-emulate": (cpu) => {
        let orig = CPUStateVAX.prototype.executeOne;
        CPUStateVAX.prototype.executeOne = function(opc, d, c) {
            if (!DISPATCH[opc]) throw makeFault();
            return orig.call(this, opc, d, c);
        };
        return () => { CPUStateVAX.prototype.executeOne = orig; };
    },
    /* Cycle accounting ignores extra_bytes, so a string instruction costs 1 -- `step N` then stops
       at a different instruction. */
    "cycles-ignore-extrabytes": (cpu) => {
        let orig = Object.getOwnPropertyDescriptor(cpu, "extraBytes");
        Object.defineProperty(cpu, "extraBytes", {
            configurable: true, get() { return 0; }, set(v) {}
        });
        return () => { delete cpu.extraBytes; if (orig) Object.defineProperty(cpu, "extraBytes", orig); };
    },
    /* The trap/interrupt block runs AFTER the instruction instead of before it -- one instruction
       of latency, which only a multi-instruction comparison can see. */
    "trap-one-late": (cpu) => {
        let orig = cpu.exc.stepInstruction;
        let held = 0;
        cpu.exc.stepInstruction = function(c, execute) {
            let saved = this.trpirq;
            this.trpirq = held;
            let r = orig.call(this, c, execute);
            held = saved ? saved : this.trpirq;
            return r;
        };
        return () => { cpu.exc.stepInstruction = orig; };
    }
};

function makeFault()
{
    /* decode.js's VAXFault is not exported by name from cpustate.js; build one the same way the
       decoder does, through a real fault path, so `instanceof` still holds. */
    try { new CPUStateVAX({}).fault(-0x10); } catch (e) { return e; }
    throw new Error("cpudiff: could not construct a VAXFault");
}

function applyMutation(cpu, name)
{
    let fn = MUTATIONS[name];
    if (!fn) throw new Error(`cpudiff: unknown mutation ${name}`);
    return fn(cpu);
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                           *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, dflt)
{
    let i = process.argv.indexOf(name);
    return (i >= 0 && i + 1 < process.argv.length) ? process.argv[i + 1] : dflt;
}

function main()
{
    let opts = {
        seed: +getArg("--seed", 12345),
        cases: +getArg("--cases", DEFAULT_CASES),
        maxSteps: +getArg("--max-steps", 2000000),
        ehkaa: getArg("--ehkaa", path.join(vaxRepo(), "open-simh/VAX/tests/ehkaa.exe")),
        trace: getArg("--trace", null),
        scratch: fs.mkdtempSync(path.join(os.tmpdir(), "cpudiff-"))
    };
    /* Every exit path -- the args-floor FAIL, --dump-case, --selfcheck, a normal FAIL/PASS, or an
       uncaught exception -- runs through this try/finally, so scratch is always removed.
       HANDOFF.md pcjsvax-bd1: this file had NO rmSync of opts.scratch anywhere, on any path --
       every invocation leaked, which is why 29 abandoned /tmp/cpudiff-* directories were found on
       this disk. */
    try {
        return mainInner(opts);
    } finally {
        fs.rmSync(opts.scratch, {recursive: true, force: true});
    }
}

function mainInner(opts)
{
    let fSelfcheck = process.argv.indexOf("--selfcheck") >= 0;
    let fSkipEhkaa = process.argv.indexOf("--skip-ehkaa") >= 0;
    let simh = findSimh(getArg("--simh", null));
    let problems = [];

    /* Checked BEFORE anything runs: an undersized run must fail outright, not after spending five
       minutes on EHKAA first.  The floors below do not scale down with --cases. */
    if (opts.cases < MIN_CASES) {
        console.error(`--cases ${opts.cases} is below the floor of ${MIN_CASES}; the coverage floors do not scale down.`);
        process.exitCode = 1;
        return;
    }

    console.log(`SIMH: ${simh}`);
    console.log(`scratch: ${opts.scratch}`);

    /* ---- the dispatch table's own invariants, before anything runs ---- */
    let owners = {};
    for (let o = 0; o < 512; o++) if (DISPATCH[o]) owners[DISPATCH_OWNER[o]] = (owners[DISPATCH_OWNER[o]] || 0) + 1;
    let claimed = Object.values(owners).reduce((a, b) => a + b, 0);
    console.log(`\nDispatch table: ${claimed} opcodes -- ` +
                Object.entries(owners).map(([k, v]) => `${k}:${v}`).join(" "));
    if (claimed !== EXPECTED_DISPATCH_CLAIMS) {
        problems.push(`COVERAGE: dispatch table claims ${claimed} opcodes, expected ${EXPECTED_DISPATCH_CLAIMS}`);
    }
    /*
     * NONSTORING: simhtrace.js reconstructs SIMH's `r` by reading the destination back, and
     * hand-codes the two opcodes for which that is wrong because they store nothing.  The set is
     * COMPUTED from the decode ROM, so a drom regeneration that adds a third one must fail here
     * rather than silently mis-trace it.
     */
    let nonStoring = nonStoringResultOpcodes().sort().join(",");
    if (nonStoring !== "BITB,CMPD") {
        problems.push(`NONSTORING: opcodes with a result shape but no destination are now [${nonStoring}], ` +
                      `not [BITB,CMPD]; simhtrace.js's captureResult() hand-codes exactly those two`);
    }

    let dumpCase = getArg("--dump-case", null);
    if (dumpCase !== null) return dumpOneCase(simh, opts, +dumpCase);

    if (fSelfcheck) {
        return selfcheck(simh, opts);
    }

    /* ---- PHASE 1: EHKAA ---- */
    if (!fSkipEhkaa) {
        let simhTrace = captureSimhTrace(simh, opts);
        let js = runEhkaaJS(opts, null);
        let cmp = compareTraces(js.tracePath, simhTrace, js.unavailable, opts);
        console.log("\nEHKAA (real workload)");
        console.log(`  instructions executed : ${js.records}`);
        console.log(`  cycles charged        : ${js.cycles}`);
        console.log(`  distinct opcodes      : ${js.opcodesSeen.size}`);
        console.log(`  emulate traps taken   : ${js.nEmulate}`);
        console.log(`  stopped               : ${js.stop ? js.stop.reason : "(step limit)"} at PC=${hex(js.pc)} PSL=${hex(js.psl)}`);
        console.log(`  normalized records    : CMPD-tail=${cmp.normalized_cmpd} zerospec=${cmp.normalized_zerospec} store-faulted=${cmp.normalized_unavailable}`);
        console.log(`  trace comparison      : ${cmp.match ? "MATCH over all " + cmp.simh_records + " records" : "DIVERGE"}`);
        if (!cmp.match) {
            let d = cmp.divergence;
            problems.push(`EHKAA: diverged at instruction ${d.index} (field=${d.field}, PC=${d.pc_a}): ${d.detail}`);
        }
        if (js.records < MIN_EHKAA_RECORDS) {
            problems.push(`COVERAGE: EHKAA executed only ${js.records} instructions, floor is ${MIN_EHKAA_RECORDS}`);
        }
        if (js.opcodesSeen.size < MIN_DISTINCT_OPCODES) {
            let missing = [];
            for (let o = 0; o < 512; o++) if (DISPATCH[o] && !js.opcodesSeen.has(o)) missing.push(OPCODES[o]);
            problems.push(`COVERAGE: only ${js.opcodesSeen.size} distinct opcodes executed, floor is ` +
                          `${MIN_DISTINCT_OPCODES}; never dispatched: ${missing.join(", ")}`);
        }
        if (js.nEmulate < MIN_EMULATE_TRAPS) {
            problems.push(`COVERAGE: only ${js.nEmulate} emulate traps taken, floor is ${MIN_EMULATE_TRAPS}`);
        }
        if (js.unreadable) console.log(`  NOTE: ${js.unreadable} record(s) had unreadable instruction bytes`);
    }

    /* ---- PHASE 2: randomized ---- */
    let r = phaseRandomized(simh, opts, null);
    console.log("\nRANDOMIZED (short programs, step N)");
    console.log(`  cases compared        : ${r.compared}/${r.total}`);
    console.log(`  non-trivial           : ${r.nonTrivial}`);
    for (let [k, s] of [...r.perKind].sort()) console.log(`    ${k.padEnd(14)} ${s.n}`);
    for (let f of r.failures.slice(0, 25)) problems.push("RANDOMIZED: " + f);
    if (r.failures.length > 25) problems.push(`RANDOMIZED: ...and ${r.failures.length - 25} more`);
    if (r.notReached.length) {
        problems.push(`RANDOMIZED: ${r.notReached.length} case(s) never reached comparison: ${r.notReached.slice(0, 20).join(", ")}`);
    }
    for (let k of KINDS) {
        let s = r.perKind.get(k);
        if (!s || s.n < MIN_PER_KIND) {
            problems.push(`COVERAGE: case kind "${k}" ran ${s ? s.n : 0} times, floor is ${MIN_PER_KIND}`);
        }
    }
    if (r.nonTrivial < r.compared * 0.6) {
        problems.push(`COVERAGE: only ${r.nonTrivial}/${r.compared} cases changed any state, floor is 60%`);
    }

    /* ---- PHASE 3: inIE reset across the stepCPU() call boundary (pcjsvax-1be) ---- */
    let inie = phaseInIEReset(simh, opts, null);
    console.log("\nINIE-RESET (two stepCPU() calls, no reset between)");
    console.log(`  result                : ${inie.length ? "DIVERGE" : "MATCH"}`);
    for (let f of inie) problems.push(f);

    report(problems);
}

function selfcheck(simh, opts)
{
    console.log("\nSELFCHECK -- each mutation must be caught\n");
    let survived = [];
    for (let name of Object.keys(MUTATIONS)) {
        let caught = false, how = "";
        /* The randomized phase is the cheap detector; EHKAA is the backstop for the mutations that
           only a long real run can reach. */
        let undo = null;
        try {
            let rnd = mulberry32(opts.seed ^ 0x0C05C0DE);
            let cases = [];
            for (let i = 0; i < MIN_CASES; i++) cases.push(buildCase(KINDS[i % KINDS.length], i, rnd));
            let m = makeMachine();
            undo = applyMutation(m.cpu, name);
            let results = runBatch(simh, cases, opts.scratch);
            for (let c of cases) {
                let simhR = results.get(c.index);
                if (!simhR || !simhR.reached) continue;
                let js = runJsCase(m, c);
                let probs = compareCase(c, js, simhR);
                if (probs.length) { caught = true; how = `case ${c.index} [${c.kind}]: ${probs[0]}`; break; }
            }
        } catch (e) {
            caught = true;
            how = `threw ${e.name}: ${e.message}`;
        } finally {
            if (undo) { try { undo(); } catch (e) {} }
        }
        console.log(`  ${caught ? "CAUGHT " : "SURVIVED"} ${name.padEnd(28)} ${how}`);
        if (!caught) survived.push(name);
    }

    /*
     * INIE-RESET is graded separately rather than folded into MUTATIONS above: every mutation in
     * that table is checked by runJsCase(), which calls cpu.reset() before EVERY case -- and
     * cpu.reset() -> exc.reset() also clears inIE (its own doc comment says so), which would mask
     * this defect completely regardless of whether the fix exists.  This phase's own two-calls-
     * no-reset-between structure is the only thing that can see it, so it gets its own check here.
     */
    let baseline = phaseInIEReset(simh, opts, null);
    console.log(`  ${baseline.length ? "SURVIVED" : "CAUGHT "} ${"inie-reset-baseline".padEnd(28)} ` +
                (baseline.length ? baseline[0] : "(unmutated sequence matches SIMH, as it must)"));
    if (baseline.length) survived.push("inie-reset-baseline");

    let mutated = phaseInIEReset(simh, opts, applyInIEMutation);
    console.log(`  ${mutated.length ? "CAUGHT " : "SURVIVED"} ${"stepcpu-no-inie-reset".padEnd(28)} ` +
                (mutated.length ? mutated[0] : ""));
    if (!mutated.length) survived.push("stepcpu-no-inie-reset");

    report(survived.map((n) => `SELFCHECK: mutation "${n}" was NOT caught -- that is a coverage hole, not a pass`));
}

/**
 * dumpOneCase(simh, opts, index)
 *
 * `--dump-case N`: print the JS machine's own instruction trace for one randomized case, next to
 * SIMH's `step`-by-`step` view of the same case.  A divergence in a multi-instruction program is
 * not localizable from a post-state diff alone -- this is how you find WHICH instruction.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {number} index
 */
function dumpOneCase(simh, opts, index)
{
    let rnd = mulberry32(opts.seed ^ 0x0C05C0DE);
    let c = null;
    for (let i = 0; i <= index; i++) {
        let x = buildCase(KINDS[i % KINDS.length], i, rnd);
        if (i === index) c = x;
    }
    console.log(`\ncase ${index} [${c.kind}${c.note ? " " + c.note : ""}] steps=${c.steps} pc=${hex(c.pc)} psl=${hex(c.psl)}`);
    let m = makeMachine();
    let hst = new SimhTrace();
    m.cpu.hst = hst;
    /*
     * `CPUDIFF_FAULTS=1` prints every exception this case dispatches, with its code and both
     * parameters.  A multi-instruction program that ends in the wrong state usually ends there
     * because of an exception several instructions back, and the post-state diff cannot say which.
     */
    if (process.env['CPUDIFF_FAULTS']) {
        let ot = m.cpu.exc.takeFault.bind(m.cpu.exc);
        m.cpu.exc.takeFault = function(cc, f) {
            console.log(`  FAULT code=${f.code} p1=${f.p1} p2=${hex(f.p2)} faultPC=${hex(cc.exc.faultPC)} psl=${hex(cc.psl)}`);
            return ot(cc, f);
        };
    }
    let js = runJsCase(m, c);
    hst.finish(m.cpu);
    console.log("--- JS ---");
    console.log(hst.text());
    console.log("JS stop:", js.stop ? js.stop.message : "(none)", js.err ? js.err.message : "");
    let simhOut = runSimh(simh, buildScript([c]), path.join(opts.scratch, "cpudiff-one.ini"));
    console.log("--- SIMH ---");
    console.log(simhOut);
    let r = runBatch(simh, [c], opts.scratch).get(c.index);
    if (r && r.reached) {
        let probs = compareCase(c, js, r);
        console.log(probs.length ? "PROBLEMS:\n  " + probs.join("\n  ") : "MATCH");
    }
}

function report(problems)
{
    if (problems.length) {
        console.error("\nFAILURES:");
        for (let p of problems) console.error("  " + p);
        process.exitCode = 1;
        return;
    }
    console.log("\nOK");
}

main();
