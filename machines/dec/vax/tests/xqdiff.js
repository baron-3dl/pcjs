/**
 * @fileoverview Differential test: the DELQA (xq.js) register file, reset state and interrupt-bit
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 *               state machine -- vs. a real Open SIMH microvax3900
 *
 * WHAT THIS IS  (pcjsvax-1a45, the W1.0 device rung of epic pcjsvax-d1bf)
 * ------------
 * modules/v2/xq.js graded against the live oracle, register by register, the way tests/mscpinitdiff.js
 * grades rq.js: an IDENTICAL instruction stream runs on both engines and every observable is
 * compared -- the words each read leaves in R0..R14, and the controller's OWN examinable CSR and VAR
 * after the stream, read through SIMH's `examine xq csr/var`.  Nothing is ported by eye
 * (pcjsvax-e05): every value this file asserts is a value a REAL microvax3900 produced on this run.
 *
 * SCOPE.  The registers, reset state, and the interrupt-relevant CSR bits (RL/XL/IE/RE/SR, the
 * write-1-to-clear masking) -- NO packet movement.  Every case here stays INSIDE the two decoded
 * Qbus windows (it never bus-faults a neighbour and never writes RBDL/XBDL, which would start a
 * transfer this rung does not implement -- xq.js throws XQUnimplemented BY NAME if one is reached, so
 * an accidental transmit is a loud failure, not a green stub).  TX is pcjsvax-6b0, RX is pcjsvax-a7f.
 *
 * COEXISTENCE IS GRADED, NOT ASSUMED.  The machine mounts BOTH controllers -- the RQDX3 at RQ_BASE
 * on cpu.qbus and the DELQA at XQ_BASE on cpu.qbus2 -- and a case reads the RQDX3's SA register in
 * the same stream that reads the DELQA's CSR.  If the second addIoPage window had aliased the first,
 * or the DELQA's interrupt slot (0x14,6) had overwritten the RQDX3's (0x14,0), the RQ read would come
 * back as an XQ value; it does not.  The oracle's `show qba iospace` is parsed on every run and the
 * XQ row is required to be 20001920-2000192F, the base xq.js pins -- the same discipline RQ_BASE is
 * under in tests/mscpinitdiff.js.
 *
 *      node machines/dec/vax/tests/xqdiff.js [options]
 *        --simh PATH   microvax3900 (else $SIMH_BIN, else ../pcjs-vax/open-simh/BIN/microvax3900)
 *        --selfcheck   prove the differential detects deliberate defects injected into xq.js
 */

import fs from "fs";
import os from "os";
import path from "path";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import CQBICVAX from "../modules/v2/cqbic.js";
import RQVAX, { RQ_BASE, IOLN_RQ, RQDX3_CTYPE } from "../modules/v2/rq.js";
import XQVAX, {
    XQ_BASE, IOLN_XQ, XQ_CSR_RL, XQ_CSR_XL, XQ_CSR_IE, XQ_CSR_RE, XQ_CSR_SR, XQ_CSR_EL,
    XQ_VEC_MS, XQ_VEC_IV, XQ_VEC_ST, XQUnimplemented
} from "../modules/v2/xq.js";
import {
    Asm, OPC, R_CODE, hex, findSimhBin, runSimh, sampleHeap, peakHeap, MEM_MB
} from "./mscpharness.js";

/** RAM size -- the microvax3900 default every VAX differential in this tree uses. */
const MEMSIZE = 0x01000000;

/** An absolute step bound; every case here HALTs in well under 100 instructions. */
const MAX_STEPS = 4000;

/** The heap bound (rule 14): ONE machine, one 16MB RAM allocation, reused across cases. */
const MAX_HEAP_BYTES = 256 * 1024 * 1024;

/** The 15 general registers, compared in full so a case's leftovers in an unwritten register are a
    miss, not a shrug (both engines zero R0..R14 before each case). */
const OBS_REGS = 15;

/* Absolute addresses of the eight DELQA register words (window = XQ_BASE .. XQ_BASE+15). */
const XQ = {
    MAC0: (XQ_BASE + 0) >>> 0, MAC1: (XQ_BASE + 2) >>> 0, MAC2: (XQ_BASE + 4) >>> 0,
    MAC3: (XQ_BASE + 6) >>> 0, MAC4: (XQ_BASE + 8) >>> 0, MAC5: (XQ_BASE + 10) >>> 0,
    VAR:  (XQ_BASE + 12) >>> 0, CSR: (XQ_BASE + 14) >>> 0
};
/* The RQDX3's SA register -- read to prove the two windows do not alias. */
const RQ_SA = (RQ_BASE + 2) >>> 0;

/* ------------------------------------------------------------------------------------------- *
 * The machine under test -- ONE, built once, reused (rule 14).  BOTH controllers are mounted.   *
 * ------------------------------------------------------------------------------------------- */

function makeXqMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let cqbic = new CQBICVAX(cpu.exc, bus, MEMSIZE);
    let rq = new RQVAX(cqbic, {cnum: 0, ctype: RQDX3_CTYPE});
    let xq = new XQVAX(cqbic, {});
    /* ONE addIoPage call, TWO windows -- the coexistence this rung proves.  They never overlap. */
    bus.addIoPage([
        {base: RQ_BASE, length: IOLN_RQ, dev: rq},
        {base: XQ_BASE, length: IOLN_XQ, dev: xq}
    ]);
    cpu.qbus = rq;                                          /* the RQDX3's event queue */
    cpu.qbus2 = xq;                                         /* the DELQA's -- pcjsvax-1a45 */
    sampleHeap();
    return {bus, cpu, cqbic, rq, xq};
}

let MACHINE = null;
function machine() { if (!MACHINE) MACHINE = makeXqMachine(); return MACHINE; }

/* ------------------------------------------------------------------------------------------- *
 * The mutations --selfcheck injects into the REAL device (standing rule 11: perturb the shipped   *
 * instance, do not substitute a copy).  Each is re-applied after every per-case reset, and each    *
 * MUST make at least one case fail or the differential is not measuring what it claims.            *
 * ------------------------------------------------------------------------------------------- */
const MUTATIONS = {
    "csr-reset-adds-RE": (xq) => { xq.csr = (xq.csr | XQ_CSR_RE) & 0xFFFF; },
    "mac-rom-drops-high-byte": (xq) => {
        let orig = xq.rd.bind(xq);
        xq.rd = (pa) => { let v = orig(pa); let i = (pa >>> 1) & 7; return (i <= 5) ? (v & 0x00FF) : v; };
    },
    "checksum-corrupted": (xq) => { xq.mac_checksum = [(xq.mac_checksum[0] ^ 0x01) & 0xFF, xq.mac_checksum[1]]; },
    "var-reset-loses-MS": (xq) => { xq.var = xq.var & ~XQ_VEC_MS; },
    "el-mode-ignored": (xq) => {
        let orig = xq.rd.bind(xq);
        xq.rd = (pa) => { let i = (pa >>> 1) & 7; return (i <= 1) ? (0xFF00 | xq.mac[i]) : orig(pa); };
    },
    "csr-write1-clear-broken": (xq) => {
        let orig = xq.wrCsr.bind(xq);
        /* drop the write-1-to-clear masking: a plain set/clear of the RW bits only */
        xq.wrCsr = (data) => { data &= 0xFFFF; let set = data & 0x074B; xq.csrSetClr(set, (~set) & 0x074B); };
    }
};

function applyMutation(xq, mut) { if (mut && MUTATIONS[mut]) MUTATIONS[mut](xq); }

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

/** movAbsReg helpers spelled at the case level so the intent reads. */
function rdWord(a, addr, rn) { a.movAbsReg(2, addr, rn); }         /* MOVZWL @#addr, Rn */
function rdByte(a, addr, rn) { a.movAbsReg(1, addr, rn); }         /* MOVZBL @#addr, Rn */
function wrWord(a, val, addr) { a.movImmAbs(2, val, addr); }       /* MOVW I^#val, @#addr */
function wrByte(a, val, addr) { a.movImmAbs(1, val, addr); }       /* MOVB I^#val, @#addr */

function buildCases()
{
    let cases = [];
    let add = (name, build) => {
        let a = new Asm();
        build(a);
        a.halt();
        let code = a.b;
        if (code.length > 0x400) throw new Error(`xqdiff: case "${name}" code is ${code.length} bytes`);
        cases.push({idx: cases.length, name, code, haltPC: (R_CODE + code.length) >>> 0});
    };

    /* ---- RESET STATE + FULL READ DECODE: every window word into R0..R7.  MAC-ROM words carry the
       floating high byte (0xFF00 | mac[i]); VAR is 0x8000 (MS); CSR is 0x0030 (RL|XL).  This is the
       reset state AND the read decode in one stream. ---- */
    add("reset reads: all eight window words", (a) => {
        rdWord(a, XQ.MAC0, 0); rdWord(a, XQ.MAC1, 1); rdWord(a, XQ.MAC2, 2); rdWord(a, XQ.MAC3, 3);
        rdWord(a, XQ.MAC4, 4); rdWord(a, XQ.MAC5, 5); rdWord(a, XQ.VAR, 6); rdWord(a, XQ.CSR, 7);
    });

    /* ---- COEXISTENCE: the RQDX3's SA in the SAME stream as the DELQA's CSR + MAC.  If the windows
       aliased, R0 would not be 0x0B40.  Graded on both engines. ---- */
    add("coexistence: RQ SA, XQ CSR, XQ MAC0 all distinct", (a) => {
        rdWord(a, RQ_SA, 0); rdWord(a, XQ.CSR, 1); rdWord(a, XQ.MAC0, 2);
    });

    /* ---- EXTERNAL LOOPBACK exposes the MAC-ROM CHECKSUM: set EL, then read words 0/1 (now the
       bespoke xq_make_checksum bytes, not the MAC).  Read CSR back to confirm EL took. ---- */
    add("EL mode: MAC-ROM words 0/1 return the checksum", (a) => {
        wrWord(a, XQ_CSR_EL, XQ.CSR);
        rdWord(a, XQ.MAC0, 0); rdWord(a, XQ.MAC1, 1);
        rdWord(a, XQ.MAC2, 2);                              /* word 2 is still the MAC, not checksum */
        rdWord(a, XQ.CSR, 3);
    });

    /* ---- VAR WRITE: program MS + full vector field, read VAR back (data & XQ_VEC_RW), and read CSR
       (unchanged).  The vector echo (dib.vec = data & IV) lives in the VAR read-back's IV bits. ---- */
    add("VAR write: MS + vector field echoes in the read-back", (a) => {
        wrWord(a, (XQ_VEC_MS | XQ_VEC_IV) & 0xFFFF, XQ.VAR);
        rdWord(a, XQ.VAR, 0);
        rdWord(a, XQ.CSR, 1);
    });

    /* ---- VAR MS-CLEAR switches the reported mode to DEQNA and masks VAR<14:10> off; the read-back
       proves OS/RS/ST were cleared.  (mode is internal; its effect here is the masking.) ---- */
    add("VAR write: clearing MS masks VAR and drops to DEQNA mode", (a) => {
        wrWord(a, XQ_VEC_ST, XQ.VAR);                      /* try to set self-test status bits + clear MS */
        rdWord(a, XQ.VAR, 0);
        rdWord(a, XQ.CSR, 1);
    });

    /* ---- CSR IE with no XI/RI: IE goes high, but XIRI is low so NO interrupt is raised; the CSR
       read-back is RL|XL|IE.  Exercises the IE-transition arm of xq_csr_set_clr. ---- */
    add("CSR write: IE set with XI/RI low raises nothing", (a) => {
        wrWord(a, XQ_CSR_IE, XQ.CSR);
        rdWord(a, XQ.CSR, 0);
    });

    /* ---- CSR RE (receiver enable): RE goes high (RL|XL|RE).  With nothing attached the scheduled
       receiver-start service is a faithful no-op; the read-back proves the bit stuck. ---- */
    add("CSR write: RE set enables the receiver bit", (a) => {
        wrWord(a, XQ_CSR_RE, XQ.CSR);
        rdWord(a, XQ.CSR, 0);
    });

    /* ---- SOFTWARE RESET: set SR, then clear it -> xq_sw_reset -> CSR back to RL|XL.  The two-write
       sequence is the only way SR "transitions to cleared". ---- */
    add("CSR write: SR set then cleared soft-resets to RL|XL", (a) => {
        wrWord(a, XQ_CSR_SR, XQ.CSR);
        rdWord(a, XQ.CSR, 0);                              /* SR now set */
        wrWord(a, 0, XQ.CSR);                              /* SR -> clear: soft reset */
        rdWord(a, XQ.CSR, 1);                              /* back to RL|XL */
    });

    /* ---- BYTE LANES: a byte read of CSR's low lane (0x30) and high lane (0x00) proves (pa&1)
       selects the lane inside the word.  MOVZBL @#CSR and @#CSR+1. ---- */
    add("byte lanes: CSR low and high byte", (a) => {
        rdByte(a, XQ.CSR, 0);                              /* low lane = 0x30 */
        rdByte(a, (XQ.CSR + 1) >>> 0, 1);                  /* high lane = 0x00 */
        rdWord(a, XQ.CSR, 2);                              /* whole word = 0x0030 */
    });

    return cases;
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side -- ONE invocation for the whole case list                                       *
 * ------------------------------------------------------------------------------------------- */

const MARK = "XQCASE";

function simhCaseLines(c)
{
    let L = [];
    L.push(`echo ${MARK}${c.idx}`);
    /* independence: reset both controllers and zero the registers this case reads back into */
    L.push("reset xq", "reset rq");
    for (let k = 0; k < OBS_REGS; k++) L.push(`deposit R${k} 0`);
    for (let k = 0; k < c.code.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${c.code[k].toString(16)}`);
    L.push("deposit PSL 0", `deposit PC ${hex(R_CODE)}`);
    L.push(`step ${MAX_STEPS}`);
    L.push(`examine -h ${Array.from({length: OBS_REGS}, (_, k) => "R" + k).join(",")}`);
    L.push("examine -h PC");
    L.push("examine -h xq csr", "examine -h xq var");
    L.push("echo ENDCASE");
    return L;
}

function runCasesSimh(simh, scratch, cases)
{
    let L = ["set cpu " + MEM_MB + "m", "set cpu simhalt",
             "set rq rqdx3", "set xq enable", "set xq type=DELQA", "set xq mac=08:00:2B:00:00:00",
             "echo ===IOSPACE===", "show qba iospace", "echo ===ENDIOSPACE==="];
    for (let c of cases) L.push(...simhCaseLines(c));
    L.push("quit", "");
    let out = runSimh(simh, L.join("\n"), path.join(scratch, "xqdiff-cases.ini"));

    let iospace = /===IOSPACE===([\s\S]*?)===ENDIOSPACE===/.exec(out);
    let results = new Array(cases.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = cases.findIndex((c) => c.idx === +parts[i]);
        if (idx < 0) continue;
        results[idx] = parseChunk(parts[i + 1] || "", cases[idx]);
    }
    return {results, iospace: iospace ? iospace[1] : ""};
}

function parseChunk(chunk, c)
{
    let g = (name) => {
        let m = new RegExp(`^${name}:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
        return m ? parseInt(m[1], 16) >>> 0 : null;
    };
    let regs = [];
    for (let k = 0; k < OBS_REGS; k++) {
        let v = g("R" + k);
        if (v === null) return null;
        regs.push(v);
    }
    let pc = g("PC");
    let csr = g("CSR");
    let vr = g("VAR");
    if (pc === null || csr === null || vr === null) return null;
    let halted = /HALT instruction/.test(chunk);
    return {regs, pc, csr, var: vr, halted, atOwnHalt: halted && pc === c.haltPC};
}

/* ------------------------------------------------------------------------------------------- *
 * The JS side                                                                                   *
 * ------------------------------------------------------------------------------------------- */

function runCaseJS(c, mut)
{
    let m = machine();
    let {bus, cpu, rq, xq} = m;

    /* SIMH's `reset xq` / `reset rq`, term for term, then the injected defect (re-applied every
       reset so a monkeypatch is not wiped by powerUp). */
    xq.powerUp();
    rq.reset();
    applyMutation(xq, mut);

    for (let k = 0; k < OBS_REGS; k++) cpu.regs[k] = 0;
    for (let k = 0; k < c.code.length; k++) bus.setByte((R_CODE + k) >>> 0, c.code[k]);
    cpu.psl = 0;
    cpu.setPC(R_CODE);

    let halted = false, unimplemented = null;
    try {
        for (let s = 0; s < MAX_STEPS; s++) cpu.stepCPU(1);
    } catch (e) {
        if (e instanceof XQUnimplemented) unimplemented = e.message;
        else if (e.name === "VAXStop" && e.reason === "HALT instruction") halted = true;
        else unimplemented = `unexpected stop: ${e.reason || e.message || String(e)}`;
    }

    sampleHeap();
    return {
        regs: Array.from({length: OBS_REGS}, (_, i) => cpu.regs[i] >>> 0),
        pc: cpu.regs[15] >>> 0,
        csr: xq.csr >>> 0,
        var: xq.var >>> 0,
        halted, atOwnHalt: halted && (cpu.regs[15] >>> 0) === c.haltPC,
        unimplemented
    };
}

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

function grade(cases, sim, js, failures)
{
    let compared = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i], j = js[i];
        if (!s) { failures.push(`case ${c.idx} "${c.name}": the oracle produced no readable result`); continue; }
        if (j.unimplemented) {
            failures.push(`case ${c.idx} "${c.name}": xq.js reached an UNIMPLEMENTED path -- ${j.unimplemented}`);
            continue;
        }
        if (!s.halted || !j.halted) {
            failures.push(`case ${c.idx} "${c.name}": never stopped ` +
                `(oracle ${s.halted ? "halted" : "ran out of budget"}, here ${j.halted ? "halted" : "ran out of budget"})`);
            continue;
        }
        if (!s.atOwnHalt || !j.atOwnHalt) {
            failures.push(`case ${c.idx} "${c.name}": did not reach its own HALT at 0x${hex(c.haltPC)} ` +
                `(oracle PC=0x${hex(s.pc)}, here PC=0x${hex(j.pc)})`);
        }
        compared++;
        for (let k = 0; k < OBS_REGS; k++) {
            if (s.regs[k] !== j.regs[k]) {
                failures.push(`case ${c.idx} "${c.name}": R${k} = ${hex(j.regs[k])} here, ${hex(s.regs[k])} on the oracle`);
            }
        }
        for (let f of ["csr", "var"]) {
            if ((s[f] & 0xFFFF) !== (j[f] & 0xFFFF)) {
                failures.push(`case ${c.idx} "${c.name}": examine xq ${f.toUpperCase()} = ${hex(j[f] & 0xFFFF, 4)} here, ` +
                    `${hex(s[f] & 0xFFFF, 4)} on the oracle`);
            }
        }
    }
    return compared;
}

/**
 * assertIoSpace(iospace, failures)
 *
 * The RQ_BASE discipline (mscpinitdiff.js): re-read the oracle's own autoconfiguration and FAIL if
 * the XQ row's address window disagrees with the XQ_BASE/IOLN_XQ this port pins.  The RQ row is
 * checked too, because a shift in either would move the other.
 */
function assertIoSpace(iospace, failures)
{
    let want = [
        {name: "XQ", lo: XQ_BASE, hi: (XQ_BASE + IOLN_XQ - 1) >>> 0},
        {name: "RQ", lo: RQ_BASE, hi: (RQ_BASE + IOLN_RQ - 1) >>> 0}
    ];
    for (let w of want) {
        let re = new RegExp("^([0-9A-Fa-f]+)\\s*-\\s*([0-9A-Fa-f]+)\\*?\\s+\\S*\\s+\\d+\\s+\\d+\\s+" + w.name + "\\b", "m");
        let m = re.exec(iospace);
        if (!m) {
            /* looser fallback: the device name line with its two addresses */
            let re2 = new RegExp("^([0-9A-Fa-f]+)\\s*-\\s*([0-9A-Fa-f]+)[^\\n]*\\b" + w.name + "\\b", "m");
            m = re2.exec(iospace);
        }
        if (!m) {
            failures.push(`iospace: the oracle's SHOW QBA IOSPACE has no ${w.name} row -- cannot confirm the window`);
            continue;
        }
        let lo = parseInt(m[1], 16) >>> 0, hi = parseInt(m[2], 16) >>> 0;
        if (lo !== w.lo || hi !== w.hi) {
            failures.push(`iospace: the oracle autoconfigures ${w.name} at ${hex(lo)}-${hex(hi)} but ` +
                `${w.name === "XQ" ? "xq.js" : "rq.js"} pins ${hex(w.lo)}-${hex(w.hi)}`);
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                          *
 * ------------------------------------------------------------------------------------------- */

function parseArgs(argv)
{
    let o = {simh: null, selfcheck: false};
    for (let i = 0; i < argv.length; i++) {
        if (argv[i] === "--simh") o.simh = argv[++i];
        else if (argv[i] === "--selfcheck") o.selfcheck = true;
        else throw new Error("xqdiff: unknown argument " + argv[i]);
    }
    return o;
}

function main()
{
    let opts = parseArgs(process.argv.slice(2));
    let simh = findSimhBin(opts.simh);
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "pcjs-xqdiff-"));
    let cases = buildCases();

    /* The oracle is invoked ONCE for the clean run; --selfcheck reuses those SAME oracle results,
       because a mutation is a defect in xq.js and the oracle never changes. */
    let {results: sim, iospace} = runCasesSimh(simh, scratch, cases);

    let failures = [];
    assertIoSpace(iospace, failures);
    let js = cases.map((c) => runCaseJS(c, null));
    let compared = grade(cases, sim, js, failures);

    if (compared < cases.length) {
        failures.push(`only ${compared} of ${cases.length} cases reached comparison`);
    }

    if (opts.selfcheck) {
        for (let mut of Object.keys(MUTATIONS)) {
            let mjs = cases.map((c) => runCaseJS(c, mut));
            let mfail = [];
            grade(cases, sim, mjs, mfail);
            if (mfail.length === 0) {
                failures.push(`SELFCHECK: mutation "${mut}" produced NO differential failure -- ` +
                    `the grader is blind to it`);
            } else {
                console.log(`  selfcheck: "${mut}" caught (${mfail.length} diff${mfail.length > 1 ? "s" : ""})`);
            }
        }
    }

    let heap = peakHeap();
    if (heap > MAX_HEAP_BYTES) {
        failures.push(`peak heap ${(heap / (1 << 20)).toFixed(1)}MB exceeds the ${MAX_HEAP_BYTES >> 20}MB bound`);
    }

    try { fs.rmSync(scratch, {recursive: true, force: true}); } catch (e) {}

    if (failures.length) {
        console.error(`\nxqdiff: FAIL (${failures.length})`);
        for (let f of failures) console.error("  " + f);
        process.exit(1);
    }
    console.log(`\nxqdiff: PASS -- ${compared} cases bit-identical to the oracle; ` +
        `XQ + RQ coexist; iospace confirmed; peak heap ${(heap / (1 << 20)).toFixed(1)}MB` +
        (opts.selfcheck ? "; selfcheck all caught" : ""));
}

main();
