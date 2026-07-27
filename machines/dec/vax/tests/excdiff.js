/**
 * @fileoverview Differential test: VAX SCB exception/interrupt dispatch and privileged registers
 *               vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * A differential test of modules/v2/exc.js against a REAL, EXECUTED Open SIMH microvax3900 --
 * no fixtures, no golden files.  Three phases, each covering something the others structurally
 * cannot:
 *
 *   EHKAA        REAL WORKLOAD.  The MicroVAX 3900 Hardware Core Instruction diagnostic is run to
 *                its PASS halt with patch 0005's EXCTRACE armed, which dumps the COMPLETE
 *                privileged state at every intexc(), every REI, every CHMx and every MTPR/MFPR --
 *                entry state and result state as separate records.  Every one of those events is
 *                then replayed through exc.js from its recorded entry state and graded against
 *                its recorded result state.  This is the only phase that reaches the vectors a
 *                generator does not know how to provoke, and it reaches all 25 of them.
 *   RANDOMIZED   Live SIMH console: deposit a full machine state (registers, PSL, all 23
 *                privileged registers, an SCB, five stacks), `step 1`, examine everything back,
 *                and compare against exc.js executing the identical bytes from the identical
 *                state.  This is the only phase that grades the INSTRUCTION-BOUNDARY machinery --
 *                IPL arbitration, the trap-before-interrupt-before-trace-trap dispatch order, the
 *                fact that a dispatch does not consume the step -- and the only one that grades
 *                the values a real diagnostic never writes (reserved IPR numbers, illegal REI
 *                PSLs, MTPR from user mode).
 *   MAPPED       Memory management ON, over a purpose-built system page table with deliberately
 *                unreadable and invalid pages, so that ACV/TNV dispatch happens FOR REAL with its
 *                two parameters pushed and read back out of SIMH's memory -- and so that an
 *                exception whose own stack push faults becomes SCB_KSNV on the interrupt stack.
 *                The EHKAA phase observes those vectors but replays them with mapping off (it has
 *                no way to reconstruct EHKAA's page tables), so this phase is what grades the
 *                inIE/KSNV substitution end to end.
 *
 *      node machines/dec/vax/tests/excdiff.js [options]
 *        --simh PATH       microvax3900 built WITH patch 0005 (else $SIMH_EXC_BIN, else scratch)
 *        --ehkaa PATH      the EHKAA diagnostic image
 *        --cases N         randomized cases per kind (default 120, floor 40)
 *        --mapped N        mapped-fault cases (default 150, floor 60)
 *        --seed S          PRNG seed, printed on failure so a run is reproducible
 *        --selfcheck       prove the differential detects deliberate defects in exc.js
 *
 * TWO MEASURED FACTS THIS FILE DEPENDS ON, AND ONE THAT TURNED OUT TO BE WRONG
 * ----------------------------------------------------------------------------
 * docs/reference/ehkaa-profile.md §5 reports 1,675 dispatch events over 25 distinct vectors, and
 * §4.2 reports 24 distinct IPRs over 1,518 MTPRs and 391 MFPRs.  Re-measured here:
 *
 *   - The 25 vectors and the 24 IPRs are EXACTLY reproduced, and are asserted by SET, not by
 *     count: this file fails if any documented vector or IPR is missing, and equally if one turns
 *     up that the document does not list.
 *   - 1,518 MTPR and 391 MFPR reproduce exactly.
 *   - 1,675 DOES NOT.  The same binary, the same image and the same `go -q 200` produce 1,675
 *     dispatches with a light debug log and 2,356 with a heavy one -- and the stock, unpatched
 *     simulator does the same thing when given any comparably chatty debug category (verified:
 *     `set cpu debug=intexc;rei;rsvdfault;abort;context` on the UNPATCHED binary also yields
 *     2,356).  EHKAA's timer-driven tests iterate a number of times that depends on how much
 *     wall-clock time the host spends per simulated tick, so the event COUNT is a property of the
 *     measurement, not of the diagnostic.  The count is therefore asserted as a FLOOR (>= 1,675)
 *     and the vector SET as an equality.  Each run is internally deterministic (three consecutive
 *     runs: 2,356 / 2,356 / 2,356).
 *
 * COVERAGE IS ASSERTED, NOT REPORTED
 * -----------------------------------
 * Every phase fails the run when its floors are not met, and the floors do NOT scale down with
 * --cases: an undersized run fails outright rather than quietly passing.  Asserted: all 25 EHKAA
 * vectors replayed, all 24 EHKAA IPRs replayed, every SCB vector kind and every software IPL
 * exercised in the randomized phase, a register-computed (non-literal) MTPR and MFPR PR#, both
 * stack-switching directions, ACV and TNV and KSNV in the mapped phase, and a floor on the
 * fraction of comparisons that are non-trivial.  Any case that does not reach a comparison is
 * reported BY NAME and fails the run -- never silently skipped.
 *
 * WHAT --selfcheck TAUGHT THIS FILE
 * ---------------------------------
 * Three of the fifteen mutations survived their first run, and none of them was a wrong assertion:
 * all three were the same defect in the GENERATORS, which is that a uniform pool does not reach a
 * boundary you have to land on exactly.
 *
 *   - "MTPR to SIRR treats a request of 0 as level 0" needs MTPR, to register 0x14, with a value
 *     whose low nibble is zero, IN KERNEL MODE.  Roughly one uniform case in six hundred.
 *   - "a memory error is taken at its own IPL" needs PSL<IPL> to be EXACTLY 0x1D with MEMERR set.
 *     Roughly one in sixty.
 *   - "an ACV inside the exception flow is not turned into KSNV" needs a mapped case whose kernel
 *     stack page is broken; sampling 11 variants uniformly into a 25-case batch misses it often.
 *
 * The fix in each case was to walk the boundary DETERMINISTICALLY (MTPR_EDGE, MFPR_EDGE,
 * ERRINT_EDGE, MAPPED_VARIANTS, and forceKernel() for the edge tables) so that a run at the
 * MINIMUM case count still reaches it, rather than to enlarge the run and hope.  Each of those
 * tables carries the reason at its definition.  --selfcheck now passes on every seed tried
 * (5, 7, 11, 23, 42, 99).
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
import { VAXFault, VAXFAULT } from "../modules/v2/decode.js";
import VAXCpu, { DISPATCH } from "../modules/v2/cpu.js";
import { OPCODES } from "../modules/v2/drom.js";
import VAXExc, {
    VAXStop, executeExc, IMPLEMENTED as EXC_IMPLEMENTED, SCB, MT, MT_MAX, IPR_DEVICE, IE,
    PSL_TP, PSL_IS, PSL_CUR, PSL_IPL, PSL_V_CUR, PSL_V_PRV, PSL_V_IPL, PSL_M_IPL,
    PSW_T, CC_MASK, KERN, USER, IPL_MEMERR, IPL_CRDERR, IPL_SMAX, SISR_MASK, TIR_V_TRAP
} from "../modules/v2/exc.js";

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

/* ------------------------------------------------------------------------------------------- *
 * Small utilities (PRNG matches the other VAX differentials, so a failing seed reproduces)       *
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

/* ------------------------------------------------------------------------------------------- *
 * Locating SIMH.  This test needs patch 0005 (the EXCTRACE category); a simulator without it     *
 * produces an EMPTY trace, which is caught explicitly rather than passing as "nothing to check".  *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['SIMH_EXC_BIN']) candidates.push(process.env['SIMH_EXC_BIN']);
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh-exc/open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) {
        if (fs.existsSync(p)) return p;
    }
    throw new Error(
        "This test grades against a REAL SIMH built with the exception-trace patch (0005); it has\n" +
        "no fixture fallback.  Build one with:\n" +
        "    machines/dec/vax/tests/simh/build.sh $TMPDIR/pcjs-vax-simh-exc\n" +
        "and pass --simh PATH or set $SIMH_EXC_BIN.  Tried:\n  " + (candidates.join("\n  ") || "(nothing)"));
}

function runSimh(bin, script, outPath)
{
    fs.writeFileSync(outPath, script);
    return execFileSync(bin, [outPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 60 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test                                                                         *
 * ------------------------------------------------------------------------------------------- */

const MEMSIZE = 0x01000000;             // 16MB, the SIMH microvax3900 default

/**
 * @class ExcCpu
 *
 * VAXCpu (cpu.js) plus the privileged state and dispatch machinery this item adds.  The execute
 * hook chains exc.js's opcode table first and falls back to cpu.js's integer/logical table, which
 * is all the mapped phase's faulting operand instructions need.
 */
class ExcCpu extends VAXCpu {
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
        throw new Error(`excdiff: opcode ${hex(opc, 3)} (${OPCODES[opc] || "?"}) has no body wired into this harness`);
    }
    /** One `step 1`: see exc.js's stepInstruction() for why that is not the same as one instruction. */
    stepOne() { return this.exc.stepInstruction(this, (opc, d, c) => this.executeOne(opc, d, c)); }
}

function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new ExcCpu(bus);
    return {bus, cpu};
}

/* ------------------------------------------------------------------------------------------- *
 * The privileged register file, as one list, used by BOTH the SIMH script (deposit/examine names) *
 * and the JS side (load/store accessors).  One list, so the two can never drift apart.            *
 * ------------------------------------------------------------------------------------------- */

const PRIV = [
    {name: "KSP",    get: (c) => c.exc.stk[0],   set: (c, v) => { c.exc.stk[0] = v; }},
    {name: "ESP",    get: (c) => c.exc.stk[1],   set: (c, v) => { c.exc.stk[1] = v; }},
    {name: "SSP",    get: (c) => c.exc.stk[2],   set: (c, v) => { c.exc.stk[2] = v; }},
    {name: "USP",    get: (c) => c.exc.stk[3],   set: (c, v) => { c.exc.stk[3] = v; }},
    {name: "IS",     get: (c) => c.exc.stk[4],   set: (c, v) => { c.exc.stk[4] = v; }},
    {name: "SCBB",   get: (c) => c.exc.scbb,     set: (c, v) => { c.exc.scbb = v; }},
    {name: "PCBB",   get: (c) => c.exc.pcbb,     set: (c, v) => { c.exc.pcbb = v; }},
    {name: "P0BR",   get: (c) => c.mmu.p0br,     set: (c, v) => { c.mmu.setP0BR(v); }},
    {name: "P0LR",   get: (c) => c.mmu.p0lr,     set: (c, v) => { c.mmu.setP0LR(v); },  mask: 0x003FFFFF},
    {name: "P1BR",   get: (c) => c.mmu.p1br,     set: (c, v) => { c.mmu.setP1BR(v); }},
    {name: "P1LR",   get: (c) => c.mmu.p1lr,     set: (c, v) => { c.mmu.setP1LR(v); },  mask: 0x003FFFFF},
    {name: "SBR",    get: (c) => c.mmu.sbr,      set: (c, v) => { c.mmu.setSBR(v); }},
    {name: "SLR",    get: (c) => c.mmu.slr,      set: (c, v) => { c.mmu.setSLR(v); },   mask: 0x003FFFFF},
    {name: "SISR",   get: (c) => c.exc.sisr,     set: (c, v) => { c.exc.sisr = v; },    mask: 0xFFFF},
    {name: "ASTLVL", get: (c) => c.exc.astlvl,   set: (c, v) => { c.exc.astlvl = v; },  mask: 0xF},
    {name: "MAPEN",  get: (c) => c.mmu.mapen,    set: (c, v) => { c.mmu.setMAPEN(v); }, mask: 0x1},
    {name: "PME",    get: (c) => c.exc.pme,      set: (c, v) => { c.exc.pme = v; },     mask: 0x1},
    {name: "TRPIRQ", get: (c) => c.exc.trpirq,   set: (c, v) => { c.exc.trpirq = v; },  mask: 0xFF},
    {name: "CRDERR", get: (c) => c.exc.crdErr,   set: (c, v) => { c.exc.crdErr = v; },  mask: 0x1},
    {name: "MEMERR", get: (c) => c.exc.memErr,   set: (c, v) => { c.exc.memErr = v; },  mask: 0x1}
];

/* ------------------------------------------------------------------------------------------- *
 * PHASE 1 -- EHKAA replay                                                                        *
 * ------------------------------------------------------------------------------------------- */

/* patch 0005's 17-field state vector, in emission order.  Named here once. */
const ST = ["KSP", "ESP", "SSP", "USP", "IS", "SCBB", "PCBB", "ASTLVL", "SISR", "MAPEN",
            "P0BR", "P0LR", "P1BR", "P1LR", "SBR", "SLR", "PME"];

/** The 25 SCB vectors docs/reference/ehkaa-profile.md §5 documents EHKAA taking. */
const EHKAA_VECTORS = [0x08, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x34,
                       0x84, 0x88, 0x8C, 0x90, 0x94, 0x98, 0x9C, 0xA0, 0xA4, 0xA8,
                       0xAC, 0xB0, 0xB4, 0xB8, 0xBC];
/** The 24 IPRs docs/reference/ehkaa-profile.md §4.2 documents EHKAA accessing. */
const EHKAA_IPRS = [0x00, 0x01, 0x02, 0x03, 0x04, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0F,
                    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x28, 0x38, 0x39, 0x3A, 0x3E, 0x3F];
const EHKAA_MIN_EVENTS = 1675;          // the documented count; see the file header on why it is a floor
const EHKAA_MIN_MTPR = 1518;
const EHKAA_MIN_MFPR = 391;
const EHKAA_PASS_PC = 0x80018AD1;

/**
 * parseExcTrace(file)
 *
 * @param {string} file
 * @returns {Array.<Object>} records in emission order: {kind, f: number[]}
 */
function parseExcTrace(file)
{
    const RE = /\b(EXCA|EXCB|REIA|REIB|CHMA|CHMB|MTPA|MTPB|MFPA|MFPB) (.*)$/;
    let out = [];
    let text = fs.readFileSync(file, "latin1");
    /* SIMH writes its debug log with CRLF line endings, and JavaScript's `.` does not match \r --
       so an end-anchored pattern matches NOTHING against a raw split.  Strip it. */
    for (let raw of text.split("\n")) {
        let line = raw.endsWith("\r") ? raw.slice(0, -1) : raw;
        let m = RE.exec(line);
        if (!m) continue;
        let f = m[2].trim().split(/\s+/).map((t) => parseInt(t, 16) | 0);
        /* EXCA's second field is `ei`, printed with %d and legitimately negative (IE_SVE). */
        if (m[1] === "EXCA") f[1] = parseInt(m[2].trim().split(/\s+/)[1], 10);
        out.push({kind: m[1], f});
    }
    return out;
}

/** Read the 17-field state vector starting at `base` into a plain object. */
function stateOf(f, base)
{
    let o = {};
    for (let i = 0; i < ST.length; i++) o[ST[i]] = f[base + i];
    return o;
}

/** Load a parsed state vector plus PSL/SP/PC onto the machine, with mapping FORCED OFF. */
function loadState(cpu, st, psl, sp, pc, trpirq)
{
    cpu.mmu.setMAPEN(0);                    // see phaseEHKAA's docblock
    cpu.exc.reset();
    cpu.regs.fill(0);
    cpu.exc.stk[0] = st.KSP; cpu.exc.stk[1] = st.ESP; cpu.exc.stk[2] = st.SSP;
    cpu.exc.stk[3] = st.USP; cpu.exc.stk[4] = st.IS;
    cpu.exc.scbb = st.SCBB;
    cpu.exc.pcbb = st.PCBB;
    cpu.exc.astlvl = st.ASTLVL;
    cpu.exc.sisr = st.SISR;
    cpu.exc.pme = st.PME;
    cpu.exc.trpirq = trpirq | 0;
    cpu.mmu.setP0BR(st.P0BR); cpu.mmu.setP0LR(st.P0LR);
    cpu.mmu.setP1BR(st.P1BR); cpu.mmu.setP1LR(st.P1LR);
    cpu.mmu.setSBR(st.SBR);   cpu.mmu.setSLR(st.SLR);
    cpu.psl = psl | 0;
    cpu.regs[14] = sp | 0;
    cpu.regs[15] = pc | 0;
}

/** Compare a post-state against SIMH's, returning a list of field mismatches. */
function diffState(cpu, st, label)
{
    let got = {
        KSP: cpu.exc.stk[0], ESP: cpu.exc.stk[1], SSP: cpu.exc.stk[2], USP: cpu.exc.stk[3],
        IS: cpu.exc.stk[4], SCBB: cpu.exc.scbb, PCBB: cpu.exc.pcbb, ASTLVL: cpu.exc.astlvl,
        SISR: cpu.exc.sisr, MAPEN: cpu.mmu.mapen, P0BR: cpu.mmu.p0br, P0LR: cpu.mmu.p0lr,
        P1BR: cpu.mmu.p1br, P1LR: cpu.mmu.p1lr, SBR: cpu.mmu.sbr, SLR: cpu.mmu.slr,
        PME: cpu.exc.pme
    };
    let bad = [];
    for (let k of ST) {
        if (k === "MAPEN") continue;                    // forced off on our side, by construction
        if ((got[k] | 0) !== (st[k] | 0)) bad.push(`${label}: ${k} js=${hex(got[k])} simh=${hex(st[k])}`);
    }
    return bad;
}

/**
 * phaseEHKAA(opts)
 *
 * MAPPING IS FORCED OFF FOR THE REPLAY, DELIBERATELY.  EHKAA's page tables are its own private
 * memory image and are not reconstructible from the trace; with mapping on, every stack push this
 * phase replays would fault on a page table that is not there.  With mapping off, the pushes land
 * in scratch memory at the same numeric addresses -- which is exactly what is being graded here:
 * the ADDRESSES and VALUES pushed, the stack selected, the new PSL, and the new PC.  Address
 * TRANSLATION is mmudiff.js's proven domain and is not re-graded; the ACV/TNV/KSNV paths that need
 * real translation are the MAPPED phase's job below.  Every write the replay performs is captured
 * by a recording wrapper and compared, so a wrong push address cannot hide in memory that nobody
 * reads back.
 *
 * @param {Object} opts
 * @returns {Object}
 */
function phaseEHKAA(opts)
{
    let tracePath = path.join(opts.scratch, "excdiff-ehkaa.log");
    if (!opts.reuseTrace || !fs.existsSync(tracePath)) {
        if (!fs.existsSync(opts.ehkaaExe)) {
            return {skipped: true, reason: `EHKAA diagnostic not found at ${opts.ehkaaExe}`};
        }
        /* `set debug <file>` APPENDS.  Leaving a previous run's log in place silently doubles
           every count and replays every event twice -- caught the hard way. */
        try { fs.unlinkSync(tracePath); } catch (e) {}
        let script = [
            `set cpu debug=exctrace`,
            `set debug ${tracePath}`,
            `load ${opts.ehkaaExe}`,
            `go -q 200`,
            `examine PC`,
            `quit`
        ].join("\n") + "\n";
        let out = runSimh(opts.simh, script, path.join(opts.scratch, "excdiff-ehkaa.ini"));
        if (out.indexOf(hex(EHKAA_PASS_PC)) < 0) {
            return {skipped: false, fatal: `EHKAA did not halt at the documented PASS PC ${hex(EHKAA_PASS_PC)}`, out};
        }
    }
    let recs = parseExcTrace(tracePath);
    if (!recs.length) {
        return {skipped: false, fatal:
            "the EXCTRACE debug category produced no records: this simulator was built WITHOUT\n" +
            "  patch 0005 (machines/dec/vax/tests/simh/0005-exception-differential-support.patch).\n" +
            "  rm -rf the scratch tree and re-run build.sh -- an existing checkout is REUSED as-is."};
    }

    let {cpu} = makeMachine();
    let failures = [], notReached = [], chmProbeFaulted = [];
    let vecSeen = new Map(), iprSeen = new Map();
    let nExc = 0, nRei = 0, nReiFault = 0, nChm = 0, nChmGraded = 0, nMtpr = 0, nMfpr = 0;
    let nTbchkPartial = 0;
    let stackSwitch = {toIS: 0, toKSP: 0, stay: 0};

    /* Recording wrapper over the MMU's write path: exc.js's pushes go through mmu.writeData, and
       this captures (address, value, length, access) for each without disturbing the real store. */
    let writes = [];
    let realWrite = cpu.mmu.writeData.bind(cpu.mmu);
    cpu.mmu.writeData = function(va, val, lnt, acc) {
        writes.push({va: va | 0, val: val | 0, lnt, acc});
        return realWrite(va, val, lnt, acc);
    };

    for (let i = 0; i < recs.length; i++) {
        let r = recs[i];
        let next = recs[i + 1];

        if (r.kind === "EXCA") {
            /* EXCA vec ei ipl oldpsl PC oldsp trpirq <17> ; EXCB scbpa newpc PSL SP trpirq <17> */
            let [vec, ei, ipl, oldpsl, pc, oldsp, trpirq] = r.f;
            vecSeen.set(vec, (vecSeen.get(vec) || 0) + 1);
            nExc++;
            if (!next || next.kind !== "EXCB") {
                notReached.push(`EXCA vec=${hex(vec, 2)} @PC=${hex(pc)}: no matching EXCB record`);
                continue;
            }
            let stIn = stateOf(r.f, 7);
            let stOut = stateOf(next.f, 5);
            let [scbpa, rawNewPC, newpsl, newsp, ntrpirq] = next.f;
            loadState(cpu, stIn, oldpsl, oldsp, pc, trpirq);
            /* Seed the SCB longword at the PHYSICAL address SIMH read it from.  If exc.js computes
               a different address it reads zero and the resulting PC diverges -- and the address
               itself is compared exactly, below. */
            cpu.bus.setLong(scbpa & VAX.PAMASK, (ei === IE.SVE) ? (rawNewPC & ~1) : rawNewPC);
            writes.length = 0;
            let err = null;
            try {
                cpu.exc.intexc(cpu, vec, ipl, ei);
            } catch (e) {
                err = e;
            }
            if (err) { failures.push(`EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}: threw ${err.message}`); continue; }
            if ((cpu.exc.scbPA | 0) !== (scbpa | 0)) {
                failures.push(`EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}: SCB address js=${hex(cpu.exc.scbPA)} simh=${hex(scbpa)}`);
            }
            if ((cpu.psl | 0) !== (newpsl | 0)) {
                failures.push(`EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}: PSL js=${hex(cpu.psl)} simh=${hex(newpsl)}`);
            }
            /* SIMH logs SP BEFORE the two pushes; exc.js has already made them, hence the -8. */
            if ((cpu.regs[14] | 0) !== ((newsp - 8) | 0)) {
                failures.push(`EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}: SP js=${hex(cpu.regs[14])} simh=${hex(newsp - 8)} (post-push)`);
            }
            if ((cpu.regs[15] | 0) !== ((rawNewPC & ~3) | 0)) {
                failures.push(`EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}: PC js=${hex(cpu.regs[15])} simh=${hex(rawNewPC & ~3)}`);
            }
            if ((cpu.exc.trpirq | 0) !== (ntrpirq | 0)) {
                failures.push(`EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}: TRPIRQ js=${hex(cpu.exc.trpirq)} simh=${hex(ntrpirq)}`);
            }
            if (writes.length !== 2 ||
                writes[0].va !== ((newsp - 4) | 0) || writes[0].val !== (oldpsl | 0) ||
                writes[1].va !== ((newsp - 8) | 0) || writes[1].val !== (pc | 0) ||
                writes[0].acc !== MMUVAX.accWrite(KERN) || writes[1].acc !== MMUVAX.accWrite(KERN)) {
                failures.push(`EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}: frame pushes wrong: ` +
                    writes.map((w) => `${hex(w.va)}<-${hex(w.val)}/acc${w.acc}`).join(" ") +
                    ` expected ${hex(newsp - 4)}<-${hex(oldpsl)} ${hex(newsp - 8)}<-${hex(pc)}`);
            }
            for (let b of diffState(cpu, stOut, `EHKAA EXC vec=${hex(vec, 2)} @PC=${hex(pc)}`)) failures.push(b);
            if (oldpsl & PSL_IS) stackSwitch.stay++;
            else if (rawNewPC & 1) stackSwitch.toIS++;
            else stackSwitch.toKSP++;
            i++;                                        // consume the EXCB
            continue;
        }

        if (r.kind === "REIA") {
            /* REIA PSL SP newpc newpsl trpirq <17> ; REIB PSL SP newpc trpirq <17> (success only) */
            let [oldpsl, sp, newpc, newpsl, trpirq] = r.f;
            let stIn = stateOf(r.f, 5);
            nRei++;
            let ok = next && next.kind === "REIB";
            loadState(cpu, stIn, oldpsl, sp, 0, trpirq);
            /* Seed the frame the REI pops at the addresses SIMH popped it from. */
            cpu.bus.setLong(sp & VAX.PAMASK, newpc);
            cpu.bus.setLong((sp + 4) & VAX.PAMASK, newpsl);
            let cc = null, err = null;
            try {
                cc = cpu.exc.rei(cpu);
            } catch (e) {
                err = e;
            }
            if (!ok) {
                nReiFault++;
                if (!(err instanceof VAXFault) || err.code !== VAXFAULT.RESOP) {
                    failures.push(`EHKAA REI @SP=${hex(sp)} nPSL=${hex(newpsl)}: SIMH rejected this PSL (no REIB record) but exc.js ${err ? "threw " + err.message : "accepted it"}`);
                }
                continue;
            }
            if (err) { failures.push(`EHKAA REI @SP=${hex(sp)} nPSL=${hex(newpsl)}: threw ${err.message} but SIMH accepted it`); continue; }
            let stOut = stateOf(next.f, 4);
            let [fpsl, fsp, fpc, ftrpirq] = next.f;
            if ((cpu.psl & ~CC_MASK) !== (fpsl & ~CC_MASK) || (cc & CC_MASK) !== (newpsl & CC_MASK)) {
                failures.push(`EHKAA REI @SP=${hex(sp)}: PSL js=${hex(cpu.psl)} simh=${hex(fpsl | (newpsl & CC_MASK))}`);
            }
            if ((cpu.regs[14] | 0) !== (fsp | 0)) failures.push(`EHKAA REI @SP=${hex(sp)}: SP js=${hex(cpu.regs[14])} simh=${hex(fsp)}`);
            if ((cpu.regs[15] | 0) !== (fpc | 0)) failures.push(`EHKAA REI @SP=${hex(sp)}: PC js=${hex(cpu.regs[15])} simh=${hex(fpc)}`);
            if ((cpu.exc.trpirq | 0) !== (ftrpirq | 0)) failures.push(`EHKAA REI @SP=${hex(sp)}: TRPIRQ js=${hex(cpu.exc.trpirq)} simh=${hex(ftrpirq)}`);
            for (let b of diffState(cpu, stOut, `EHKAA REI @SP=${hex(sp)}`)) failures.push(b);
            i++;
            continue;
        }

        if (r.kind === "CHMA") {
            /* CHMA opc opnd0 PSL|cc PC SP <17> ; CHMB newpc PSL SP <17> (success only) */
            let [opc, op0, oldpsl, pc, sp] = r.f;
            let stIn = stateOf(r.f, 5);
            nChm++;
            if (!next || next.kind !== "CHMB") {
                /*
                 * SIMH's target-stack probe faulted, so there is no result to compare against.
                 * This replay runs with mapping OFF and cannot reproduce a translation fault, so
                 * the case is ACCOUNTED FOR by name rather than graded here -- and the CHMx probe
                 * fault it represents IS graded, against real page tables, by the MAPPED phase's
                 * chm-acv / chm-tnv variants.
                 */
                chmProbeFaulted.push(`CHMA opc=${hex(opc, 2)} @PC=${hex(pc)}`);
                continue;
            }
            let stOut = stateOf(next.f, 3);
            let [newpc, fpsl, fsp] = next.f;
            loadState(cpu, stIn, oldpsl, sp, pc, 0);
            /* The CHMx vector is read from SCBB + SCB_CHMK + (mode<<2), physically. */
            let mode = opc & 3;
            cpu.bus.setLong((stIn.SCBB + SCB.CHMK + (mode << 2)) & VAX.PAMASK & ~3, newpc);
            let err = null;
            try { cpu.exc.chm(cpu, opc, op0); } catch (e) { err = e; }
            if (err) { failures.push(`EHKAA CHM opc=${hex(opc, 2)} @PC=${hex(pc)}: threw ${err.message}`); continue; }
            if ((cpu.psl | 0) !== (fpsl | 0)) failures.push(`EHKAA CHM opc=${hex(opc, 2)} @PC=${hex(pc)}: PSL js=${hex(cpu.psl)} simh=${hex(fpsl)}`);
            if ((cpu.regs[14] | 0) !== (fsp | 0)) failures.push(`EHKAA CHM opc=${hex(opc, 2)} @PC=${hex(pc)}: SP js=${hex(cpu.regs[14])} simh=${hex(fsp)}`);
            if ((cpu.regs[15] | 0) !== ((newpc & ~3) | 0)) failures.push(`EHKAA CHM opc=${hex(opc, 2)} @PC=${hex(pc)}: PC js=${hex(cpu.regs[15])} simh=${hex(newpc & ~3)}`);
            for (let b of diffState(cpu, stOut, `EHKAA CHM opc=${hex(opc, 2)} @PC=${hex(pc)}`)) failures.push(b);
            nChmGraded++;
            i++;
            continue;
        }

        if (r.kind === "MTPA") {
            /* MTPA prn val PSL SP trpirq <17> ; MTPB cc PSL SP trpirq <17> (success only) */
            let [prn, val, psl, sp, trpirq] = r.f;
            let stIn = stateOf(r.f, 5);
            iprSeen.set(prn >>> 0, (iprSeen.get(prn >>> 0) || 0) + 1);
            nMtpr++;
            let ok = next && next.kind === "MTPB";
            loadState(cpu, stIn, psl, sp, 0, trpirq);
            let cc = null, err = null;
            try { cc = cpu.exc.mtpr(cpu, val, prn); } catch (e) { err = e; }
            if (!ok) {
                if (!(err instanceof VAXFault)) {
                    failures.push(`EHKAA MTPR prn=${hex(prn, 2)} val=${hex(val)}: SIMH faulted (no MTPB) but exc.js did not`);
                }
                continue;
            }
            if (err) { failures.push(`EHKAA MTPR prn=${hex(prn, 2)} val=${hex(val)}: threw ${err.message} but SIMH accepted it`); continue; }
            let stOut = stateOf(next.f, 4);
            let [fcc, fpsl, fsp, ftrpirq] = next.f;
            /*
             * TBCHK's CC<V> reports whether the translation buffer holds an entry for `val`.  This
             * replay starts from an EMPTY TB (EHKAA's TB contents are not in any log and cannot be
             * reconstructed), so that one bit is not gradeable here and is masked out -- COUNTED
             * and reported by name, and graded in full by the MAPPED phase's tbchk-hit /
             * tbchk-miss variants, which run with mapping ON and therefore have a TB to check.
             */
            let ccMask = CC_MASK;
            if ((prn >>> 0) === MT.TBCHK) { ccMask = CC_MASK & ~0x2; nTbchkPartial++; }
            if ((cc & ccMask) !== (fcc & ccMask)) {
                failures.push(`EHKAA MTPR prn=${hex(prn, 2)} val=${hex(val)}: cc js=${hex(cc, 1)} simh=${hex(fcc, 1)}`);
            }
            if ((cpu.psl & ~CC_MASK) !== (fpsl & ~CC_MASK)) {
                failures.push(`EHKAA MTPR prn=${hex(prn, 2)} val=${hex(val)}: PSL js=${hex(cpu.psl)} simh=${hex(fpsl)}`);
            }
            if ((cpu.exc.trpirq | 0) !== (ftrpirq | 0)) {
                failures.push(`EHKAA MTPR prn=${hex(prn, 2)}: TRPIRQ js=${hex(cpu.exc.trpirq)} simh=${hex(ftrpirq)}`);
            }
            /* MTPR of KSP (PSL<IS> clear) or of IS (PSL<IS> set) writes the LIVE stack pointer,
               R[14], and leaves stk[] alone -- the one place the "which register is the stack
               pointer" rule is observable from a single instruction.  Graded explicitly. */
            if ((cpu.regs[14] | 0) !== (fsp | 0)) {
                failures.push(`EHKAA MTPR prn=${hex(prn, 2)} val=${hex(val)}: SP js=${hex(cpu.regs[14])} simh=${hex(fsp)}`);
            }
            for (let b of diffState(cpu, stOut, `EHKAA MTPR prn=${hex(prn, 2)} val=${hex(val)}`)) failures.push(b);
            i++;
            continue;
        }

        if (r.kind === "MFPA") {
            /* MFPA prn PSL SP <17> ; MFPB val (success only).  SP matters: MFPR of KSP with
               PSL<IS> clear -- and of IS with it set -- returns the LIVE stack pointer, not the
               saved one, so replaying with a made-up SP would grade nothing for 188 of EHKAA's
               391 MFPRs. */
            let [prn, psl, sp] = r.f;
            let stIn = stateOf(r.f, 3);
            iprSeen.set(prn >>> 0, (iprSeen.get(prn >>> 0) || 0) + 1);
            nMfpr++;
            let ok = next && next.kind === "MFPB";
            loadState(cpu, stIn, psl, sp, 0, 0);
            /* loadState forces mapping OFF (see the docblock), but MFPR of MAPEN reports the bit
               itself.  mfpr() performs no memory access, so restoring the real bit here is safe
               and is the only way that one register is gradeable at all. */
            cpu.mmu.mapen = stIn.MAPEN & 1;
            let val = null, err = null;
            try { val = cpu.exc.mfpr(cpu, prn); } catch (e) { err = e; }
            if (!ok) {
                if (!(err instanceof VAXFault)) {
                    failures.push(`EHKAA MFPR prn=${hex(prn, 2)}: SIMH faulted (no MFPB) but exc.js did not`);
                }
                continue;
            }
            if (err) { failures.push(`EHKAA MFPR prn=${hex(prn, 2)}: threw ${err.message} but SIMH accepted it`); continue; }
            if ((val | 0) !== (next.f[0] | 0)) {
                failures.push(`EHKAA MFPR prn=${hex(prn, 2)}: value js=${hex(val)} simh=${hex(next.f[0])}`);
            }
            i++;
            continue;
        }
    }
    cpu.mmu.writeData = realWrite;

    return {
        skipped: false, failures, notReached, chmProbeFaulted,
        nExc, nRei, nReiFault, nChm, nChmGraded, nMtpr, nMfpr, nTbchkPartial, stackSwitch,
        vecSeen, iprSeen, records: recs.length
    };
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 2 -- randomized, live SIMH console                                                       *
 *
 * Fixed physical layout, mapping OFF (so a virtual address is its own physical address).  Every
 * byte a case touches is deposited fresh by that case, so cases cannot leak into one another even
 * though a whole batch shares one simulator process.
 * ------------------------------------------------------------------------------------------- */

const R_SCBB   = 0x00100000;            // SCB base (longword aligned; 0x200 bytes reserved)
const R_HANDLER = 0x00102000;           // every SCB vector points here; contains NOPs
const R_CODE   = 0x00104000;            // the case's instruction
const R_PCBB   = 0x00106000;            // process control block (LDPCTX/SVPCTX)
const R_DATA   = 0x00108000;            // scratch operands
const R_STACK  = [0x00110000, 0x00112000, 0x00114000, 0x00116000, 0x00118000];  // K,E,S,U,IS tops

/* The memory each case reads back from BOTH sides: 8 longwords below and 2 at/above every stack
   top (the exception frames land here), plus the PCB.  A fixed list, so the two sides always
   compare the same addresses in the same order. */
/*
 * Locations both sides zero before every case.  SIMH's memory persists across the cases in a batch
 * and the JS bus persists across cases in a run; unless BOTH are wiped identically, a case reads
 * back a previous case's exception frame and the comparison is against noise.  (It did, and it
 * was: 2,939 spurious failures in the first run.)
 */
const PROBE_ADDRS = (function() {
    let a = [];
    for (let top of R_STACK) {
        for (let k = -8; k <= 1; k++) a.push((top + k * 4) | 0);
    }
    for (let k = 0; k < 20; k++) a.push((R_PCBB + k * 4) | 0);
    return a;
})();

const ZERO_ADDRS = (function() {
    let a = PROBE_ADDRS.slice();
    for (let k = 0; k < 16; k++) a.push((R_CODE + k * 4) | 0);
    for (let k = 0; k < 16; k++) a.push((R_DATA + k * 4) | 0);
    return a;
})();

/**
 * @class Case
 */
class Case {
    constructor(kind, index)
    {
        this.kind = kind;
        this.index = index;
        this.instr = [];
        this.mem = [];                          // {addr, val} longwords
        this.regs = new Int32Array(16);
        this.psl = 0;
        this.priv = {};                          // name -> value, for every entry in PRIV
        this.pc = R_CODE;                        // virtual
        this.codePA = R_CODE;                    // where the instruction bytes are DEPOSITED
        this.zero = ZERO_ADDRS;                  // pre-zeroed on BOTH sides, so neither inherits
        this.nontrivial = false;
        this.note = "";
    }
    byte(b) { this.instr.push(b & 0xFF); }
    word(v) { this.instr.push(v & 0xFF, (v >>> 8) & 0xFF); }
    long(v) { v = v | 0; this.instr.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF); }
    setLong(addr, val) { this.mem.push({addr: addr | 0, val: val | 0}); }
}

/**
 * baseCase(kind, index, rnd)
 *
 * Every case starts from a fully-populated, ARCHITECTURALLY LEGAL machine: SIMH refuses to enter
 * sim_instr at all ("Unreasonable PSL value") when PSL<MBZ> is set, when a non-kernel mode has
 * PSL<IS> or a non-zero IPL, or when PSL<IS> is set with IPL 0 -- so a generator that sprays
 * random PSLs grades nothing.  The rules are enforced here, once.
 */
function baseCase(kind, index, rnd)
{
    let c = new Case(kind, index);
    let cur = pick(rnd, [KERN, KERN, KERN, 1, 2, USER]);
    let onIS = (cur === KERN) && rnd() < 0.25;
    let ipl = 0;
    if (cur === KERN) ipl = onIS ? (1 + Math.floor(rnd() * 0x1F)) : Math.floor(rnd() * 0x20);
    let prv = cur + Math.floor(rnd() * (4 - cur));      // rule 6 territory: prv >= cur
    c.psl = (cur << PSL_V_CUR) | (prv << PSL_V_PRV) | (ipl << PSL_V_IPL) |
            (onIS ? PSL_IS : 0) | (rnd() < 0.2 ? PSW_T : 0) | Math.floor(rnd() * 16);
    for (let r = 0; r < 14; r++) c.regs[r] = (Math.floor(rnd() * 0x100000000)) | 0;
    c.regs[14] = onIS ? R_STACK[4] : R_STACK[cur];
    c.regs[15] = c.pc;
    for (let p of PRIV) c.priv[p.name] = 0;
    c.priv.KSP = R_STACK[0]; c.priv.ESP = R_STACK[1]; c.priv.SSP = R_STACK[2];
    c.priv.USP = R_STACK[3]; c.priv.IS = R_STACK[4];
    c.priv.SCBB = R_SCBB;
    c.priv.PCBB = R_PCBB;
    c.priv.ASTLVL = Math.floor(rnd() * 5);
    c.priv.MAPEN = 0;
    /*
     * The SCB.  Every vector points at a NOP in the handler page; bit 0 (dispatch on the interrupt
     * stack) is randomized per vector, bit 1 is never set because it is STOP_ILLVEC on both sides
     * and would stop the simulator rather than produce a comparable state.
     */
    for (let v = 0; v < 0x200; v += 4) {
        let flag = (rnd() < 0.35) ? 1 : 0;
        c.setLong(R_SCBB + v, (R_HANDLER + (v & 0x3C)) | flag);
    }
    for (let k = 0; k < 16; k++) c.setLong(R_HANDLER + k * 4, 0x01010101);      // NOP NOP NOP NOP
    return c;
}

/* --- per-kind generators ------------------------------------------------------------------- */

/** Emit an MTPR/MFPR PR# operand: a short literal, or -- crucially -- a REGISTER, so the resolved
    PR# is computed at run time.  docs/reference/ehkaa-profile.md §4 records EHKAA doing exactly
    this, and an implementation that only ever sees a literal will not notice. */
function emitPRN(c, rnd, prn, forceReg)
{
    if (forceReg || rnd() < 0.5) {
        let rn = 1 + Math.floor(rnd() * 10);
        c.byte(0x50 | rn);
        c.regs[rn] = prn | 0;
        c.note = "register-computed PR#";
        return true;
    }
    if ((prn >>> 0) < 64) { c.byte(prn & 0x3F); return false; }      // short literal
    c.byte(0x8F); c.long(prn);                                       // PC-relative immediate
    return false;
}

const IPR_POOL = (function() {
    let dev = new Set(IPR_DEVICE);
    let a = [];
    for (let n = 0; n <= MT_MAX; n++) if (!dev.has(n)) a.push(n);
    return a;
})();

/*
 * DETERMINISTIC EDGE COVERAGE, ahead of the random pool.
 *
 * The first --selfcheck run left one mutation alive: "MTPR to SIRR treats a request of 0 as level
 * 0" (SISR |= 1 << (val & 0xF) with no zero test, setting SISR<0>).  It was not a wrong assertion
 * and not an unobservable difference -- SISR<0> is examined on both sides -- it was a COVERAGE
 * HOLE: reaching it needs MTPR, to register 0x14 specifically, with a value whose low nibble is
 * zero, and a uniformly-drawn (register, value) pool hits that combination roughly once in three
 * hundred cases.  EHKAA never does it either.  These tables are consumed IN ORDER by the first N
 * cases of their kind, so the combination is exercised by construction rather than by luck, and
 * the mutation is now caught.  Ordering matters: the entries most likely to distinguish a defect
 * come first, because an undersized run only gets the head of the list.
 */
const MTPR_EDGE = [
    [MT.SIRR, 0], [MT.SIRR, 16], [MT.SIRR, 1], [MT.SIRR, 15], [MT.SIRR, 7],
    [0x0F, 0x12345678], [0x28, 0x12345678], [0x3B, 1], [64, 0], [200, 0],
    [MT.ASTLVL, 0], [MT.ASTLVL, 4], [MT.ASTLVL, 5], [MT.ASTLVL, 7],
    [MT.IPL, 0], [MT.IPL, 0x1F], [MT.IPL, 0x25],
    [MT.SISR, 0xFFFF], [MT.SISR, 0], [MT.SISR, 0x0001],
    [MT.KSP, 0x00120000], [MT.IS, 0x00122000], [MT.ESP, 0x00124000], [MT.USP, 0x00126000],
    [MT.MAPEN, 0], [MT.MAPEN, 1], [MT.TBIA, 0], [MT.TBIS, R_CODE], [MT.TBCHK, R_CODE],
    [MT.SCBB, 0x00100003], [MT.PCBB, 0x00106002],
    [MT.P0BR, 0x80000003], [MT.P0LR, 0x00FFFFFF], [MT.SLR, 0x00FFFFFF], [MT.SBR, 0x00400002],
    [MT.PME, 1], [MT.PME, 0], [MT.SID, 0], [MT.CONPC, 0], [MT.CONPSL, 0]
];
const MFPR_EDGE = [
    MT.SIRR, MT.TBIA, MT.TBIS, MT.TBCHK,                    // write-only: reserved operand fault
    MT.KSP, MT.IS, MT.ESP, MT.SSP, MT.USP,
    MT.SID, MT.MAPEN, MT.PME, MT.SISR, MT.ASTLVL, MT.IPL, MT.SCBB, MT.PCBB,
    MT.P0BR, MT.P0LR, MT.P1BR, MT.P1LR, MT.SBR, MT.SLR,
    0x0F, 0x28, 0x3B, 64, 200
];

/** Per-kind cycle counters, reset at the start of every phaseRandomized so a run is reproducible
    from its seed alone and so --selfcheck's repeated small runs each get the head of the tables. */
const genCycle = {swint: 0, mtpr: 0, mfpr: 0, bptxfc: 0, ctx: 0, privfault: 0, errint: 0, mapped: 0};
function resetGenerators() { for (let k in genCycle) genCycle[k] = 0; }

/*
 * The six opcodes that live entirely inside this machinery and have no other owner:
 * pcjsvax-515's Base-Instruction-Group residual analysis found them unclaimed (cpu.js's header
 * assigned them to control.js, whose own scope note never claimed them).  They are implemented in
 * exc.js, and the generators below CYCLE rather than sample so that a run at the minimum case
 * count still executes every one of them -- and this list is asserted, so a future refactor that
 * drops one fails the run instead of quietly reducing coverage.
 */
const REQUIRED_MNEMONICS = ["REI", "BPT", "XFC", "HALT", "LDPCTX", "SVPCTX",
                            "CHMK", "CHME", "CHMS", "CHMU", "MTPR", "MFPR", "NOP"];

/**
 * forceKernel(c, onIS)
 *
 * Put a case in kernel mode, on the interrupt stack or not, preserving its condition codes and its
 * IPL.  PSL<IS> requires a non-zero IPL (SIMH rejects the combination at sim_instr entry).
 *
 * @param {Case} c
 * @param {boolean} onIS
 */
function forceKernel(c, onIS)
{
    let ipl = (c.psl >>> PSL_V_IPL) & PSL_M_IPL;
    if (onIS && ipl === 0) ipl = 1;
    c.psl = (KERN << PSL_V_CUR) | (KERN << PSL_V_PRV) | (ipl << PSL_V_IPL) |
            (onIS ? PSL_IS : 0) | (c.psl & (PSW_T | CC_MASK));
    c.regs[14] = onIS ? R_STACK[4] : R_STACK[0];
}

const GEN = {};

GEN.mtpr = (rnd, index) => {
    let c = baseCase("mtpr", index, rnd);
    let n = genCycle.mtpr++;
    let prn, val;
    if (n < MTPR_EDGE.length) {
        prn = MTPR_EDGE[n][0];
        val = MTPR_EDGE[n][1];
        /*
         * FORCE KERNEL MODE for the edge table.  baseCase() picks a random access mode, and MTPR
         * outside kernel is a reserved-INSTRUCTION fault that never reaches the register at all --
         * so half the deterministic edge cases were being thrown away before they tested anything,
         * and the SIRR-request-of-zero mutation survived --selfcheck on some seeds and not others.
         * A table that exists to make coverage deterministic must not be 50% luck.  PSL<IS> still
         * alternates, because it is what selects between the live and the saved stack pointer for
         * KSP and IS.  The privilege check itself is graded by the `privfault` kind.
         */
        forceKernel(c, (n & 1) !== 0);
    } else {
        prn = (rnd() < 0.08) ? (64 + Math.floor(rnd() * 200)) : pick(rnd, IPR_POOL);
        val = pick(rnd, [0, 1, 2, 3, 4, 5, 0xF, 0x1F, 0xFFFF, 0x7FFFFFFF, -1,
                         R_STACK[0], R_STACK[4], 0x00120000, (Math.floor(rnd() * 0x100000000)) | 0]);
        if (prn === MT.ASTLVL) val = Math.floor(rnd() * 7);          // straddle AST_MAX
        if (prn === MT.SIRR) val = Math.floor(rnd() * 20);           // straddle 0 and 0xF
    }
    c.mnemonic = "MTPR";
    c.byte(OPCODES.indexOf("MTPR") & 0xFF);
    /* value operand: register or immediate */
    if (rnd() < 0.5) { let rn = 1 + Math.floor(rnd() * 10); c.byte(0x50 | rn); c.regs[rn] = val | 0; }
    else { c.byte(0x8F); c.long(val); }
    emitPRN(c, rnd, prn);
    c.prn = prn;
    c.nontrivial = true;
    return c;
};

GEN.mfpr = (rnd, index) => {
    let c = baseCase("mfpr", index, rnd);
    let n = genCycle.mfpr++;
    let prn = (n < MFPR_EDGE.length) ? MFPR_EDGE[n]
            : ((rnd() < 0.08) ? (64 + Math.floor(rnd() * 200)) : pick(rnd, IPR_POOL));
    if (n < MFPR_EDGE.length) forceKernel(c, (n & 1) !== 0);   // see GEN.mtpr
    c.mnemonic = "MFPR";
    c.byte(OPCODES.indexOf("MFPR") & 0xFF);
    emitPRN(c, rnd, prn);
    let rn = 1 + Math.floor(rnd() * 10);
    c.byte(0x50 | rn);                                               // destination: a register
    c.prn = prn;
    /* Give the readable registers values worth reading. */
    c.priv.P0BR = 0x80000000 | 0; c.priv.P0LR = 0x123 + Math.floor(rnd() * 0x100);
    c.priv.P1BR = 0x40001000; c.priv.P1LR = 0x3FFF00 + Math.floor(rnd() * 0xFF);
    c.priv.SBR = 0x00400000; c.priv.SLR = 0x800;
    c.priv.SISR = Math.floor(rnd() * 0x10000) & SISR_MASK;
    c.priv.PME = rnd() < 0.5 ? 1 : 0;
    c.nontrivial = true;
    return c;
};

GEN.rei = (rnd, index) => {
    let c = baseCase("rei", index, rnd);
    c.mnemonic = "REI";
    c.byte(OPCODES.indexOf("REI") & 0xFF);
    let cur = (c.psl >>> PSL_V_CUR) & 3;
    let ipl = (c.psl >>> PSL_V_IPL) & PSL_M_IPL;
    /*
     * Half the frames are LEGAL returns (so the AST check, the stack write-back and the stack
     * selection are exercised), half are drawn from the rule-violation pool -- which is where a
     * rules port actually breaks, and which a legal-only generator never reaches.
     */
    let newpsl;
    if (rnd() < 0.5) {
        let ncur = cur + Math.floor(rnd() * (4 - cur));              // rule 1: ncur >= cur
        let nprv = ncur + Math.floor(rnd() * (4 - ncur));            // rule 6: nprv >= ncur
        let nipl = ncur ? 0 : Math.floor(rnd() * (ipl + 1));         // rules 5, 7
        let nis = (ncur === 0 && (c.psl & PSL_IS) && nipl > 0 && rnd() < 0.5) ? PSL_IS : 0;
        newpsl = (ncur << PSL_V_CUR) | (nprv << PSL_V_PRV) | (nipl << PSL_V_IPL) | nis |
                 (rnd() < 0.3 ? PSW_T : 0) | Math.floor(rnd() * 16);
    } else {
        newpsl = pick(rnd, [
            (c.psl & ~PSL_CUR) | (Math.max(0, cur - 1) << PSL_V_CUR),        // rule 1
            c.psl | 0x100,                                                    // rule 8 (PSW MBZ)
            c.psl | 0x00200000,                                               // rule 8 (PSL MBZ)
            (1 << PSL_V_CUR) | (1 << PSL_V_PRV) | PSL_IS,                     // rules 3/5
            (1 << PSL_V_CUR) | (1 << PSL_V_PRV) | (4 << PSL_V_IPL),           // rule 5
            (2 << PSL_V_CUR) | (1 << PSL_V_PRV),                              // rule 6
            PSL_IS | (0 << PSL_V_IPL),                                        // rule 4
            ((ipl + 3) & PSL_M_IPL) << PSL_V_IPL,                             // rule 7
            0x80000000 | 0                                                     // rule 9 (CM)
        ]);
    }
    let sp = c.regs[14];
    c.setLong(sp, R_HANDLER);                                        // new PC: a NOP
    c.setLong((sp + 4) | 0, newpsl);
    c.newpsl = newpsl;
    c.nontrivial = true;
    return c;
};

GEN.chm = (rnd, index) => {
    let c = baseCase("chm", index, rnd);
    /* CHMx cannot execute on the interrupt stack (STOP_CHMFI on both sides). */
    c.psl = c.psl & ~PSL_IS;
    let cur = (c.psl >>> PSL_V_CUR) & 3;
    if (((c.psl >>> PSL_V_IPL) & PSL_M_IPL) && cur) c.psl = c.psl & ~PSL_IPL;
    c.regs[14] = R_STACK[cur];
    let mn = pick(rnd, ["CHMK", "CHME", "CHMS", "CHMU"]);
    c.mnemonic = mn;
    c.byte(OPCODES.indexOf(mn) & 0xFF);
    let arg = pick(rnd, [0, 1, 0x7FFF, 0x8000, 0xFFFF, Math.floor(rnd() * 0x10000)]);
    c.byte(0x8F); c.word(arg);
    c.nontrivial = true;
    return c;
};

GEN.swint = (rnd, index) => {
    let c = baseCase("swint", index, rnd);
    /* A NOP, so the ONLY thing that happens in the step is whatever interrupt arbitration
       decides.  Half the cases seed SEVERAL SISR bits, so arbitration has to choose the highest
       eligible one; the other half seed exactly ONE bit at a level that CYCLES, because the
       highest-wins rule means a purely random pool essentially never lets a low level be taken and
       the run would report "IPL arbitration exercised" on 4 of the 15 levels. */
    c.mnemonic = "NOP";
    c.byte(OPCODES.indexOf("NOP") & 0xFF);
    let n = genCycle.swint++;
    let bits, ipl;
    if (n & 1) {
        /* TWO eligible bits, deliberately far apart, with the IPL below BOTH -- so "the highest
           eligible level wins" is a decision the case actually forces, not a coincidence. */
        let lo = 1 + ((n >> 1) % (IPL_SMAX - 1));
        let hi = 1 + ((lo + 6) % IPL_SMAX);
        if (hi === lo) hi = lo + 1;
        bits = (1 << lo) | (1 << hi);
        ipl = Math.floor(rnd() * Math.min(lo, hi));
    } else {
        /* ONE bit, at a level that cycles. */
        let lvl = 1 + ((n >> 1) % IPL_SMAX);
        bits = 1 << lvl;
        ipl = Math.floor(rnd() * lvl);                  // strictly below lvl: eligible
    }
    if (rnd() < 0.4) bits |= 1 << (1 + Math.floor(rnd() * IPL_SMAX));    // extra noise
    c.priv.SISR = bits & SISR_MASK;
    c.psl = (KERN << PSL_V_CUR) | (KERN << PSL_V_PRV) | (ipl << PSL_V_IPL) | Math.floor(rnd() * 16);
    c.regs[14] = R_STACK[0];
    c.nontrivial = true;
    return c;
};

/*
 * eval_int() compares the CURRENT IPL against the memory-error and CRD-error levels with `<`, not
 * `<=`: an error interrupt is masked AT its own level.  Distinguishing the two needs the IPL to be
 * exactly 0x1D or exactly 0x1A with the matching flag set, which a random pool over 32 levels and
 * two flags reaches about one case in sixty -- so the boundary is walked deterministically here.
 * (It was not, and the "memory error is taken at its own IPL" mutation survived --selfcheck on
 * three seeds out of four.)
 */
const ERRINT_EDGE = [
    {ipl: 0x1D, mem: 1, crd: 0}, {ipl: 0x1C, mem: 1, crd: 0},
    {ipl: 0x1A, mem: 0, crd: 1}, {ipl: 0x19, mem: 0, crd: 1},
    {ipl: 0x1D, mem: 1, crd: 1}, {ipl: 0x1A, mem: 1, crd: 1},
    {ipl: 0x1E, mem: 1, crd: 0}, {ipl: 0x1B, mem: 0, crd: 1},
    {ipl: 0x1F, mem: 1, crd: 1}, {ipl: 0x00, mem: 1, crd: 1},
    {ipl: 0x00, mem: 0, crd: 1}, {ipl: 0x00, mem: 1, crd: 0}
];

GEN.errint = (rnd, index) => {
    let c = baseCase("errint", index, rnd);
    c.mnemonic = "NOP";
    c.byte(OPCODES.indexOf("NOP") & 0xFF);
    let n = genCycle.errint++;
    let ipl;
    if (n < ERRINT_EDGE.length) {
        let e = ERRINT_EDGE[n];
        ipl = e.ipl; c.priv.MEMERR = e.mem; c.priv.CRDERR = e.crd;
    } else {
        c.priv.MEMERR = rnd() < 0.6 ? 1 : 0;
        c.priv.CRDERR = rnd() < 0.6 ? 1 : 0;
        ipl = pick(rnd, [0, 1, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, Math.floor(rnd() * 0x20)]);
    }
    c.priv.SISR = (rnd() < 0.5 ? (1 << (1 + Math.floor(rnd() * IPL_SMAX))) : 0) & SISR_MASK;
    c.psl = (KERN << PSL_V_CUR) | (ipl << PSL_V_IPL) | Math.floor(rnd() * 16);
    if (rnd() < 0.4 && ipl > 0) c.psl |= PSL_IS;
    c.regs[14] = (c.psl & PSL_IS) ? R_STACK[4] : R_STACK[0];
    c.nontrivial = true;
    return c;
};

GEN.trap = (rnd, index) => {
    let c = baseCase("trap", index, rnd);
    c.byte(OPCODES.indexOf("NOP") & 0xFF);
    /* A pending arithmetic trap outranks any interrupt -- seed both and check the ORDER. */
    c.priv.TRPIRQ = (1 + Math.floor(rnd() * 7)) << TIR_V_TRAP;
    /* An ELIGIBLE software interrupt is always pending too: the trap must still win, and a case
       with no competing interrupt cannot tell "traps first" from "interrupts first". */
    let ipl = Math.floor(rnd() * 4);
    c.priv.SISR = (1 << (ipl + 1 + Math.floor(rnd() * (IPL_SMAX - ipl - 1)))) & SISR_MASK;
    c.psl = (KERN << PSL_V_CUR) | (ipl << PSL_V_IPL) | Math.floor(rnd() * 16);
    c.regs[14] = R_STACK[0];
    c.nontrivial = true;
    return c;
};

GEN.trace = (rnd, index) => {
    let c = baseCase("trace", index, rnd);
    c.byte(OPCODES.indexOf("NOP") & 0xFF);
    c.psl = (c.psl & ~(PSW_T | PSL_TP)) | (rnd() < 0.6 ? PSW_T : 0) | (rnd() < 0.5 ? PSL_TP : 0);
    c.nontrivial = true;
    return c;
};

GEN.bptxfc = (rnd, index) => {
    let c = baseCase("bptxfc", index, rnd);
    let mn = ["BPT", "XFC"][genCycle.bptxfc++ % 2];
    c.mnemonic = mn;
    c.byte(OPCODES.indexOf(mn) & 0xFF);
    c.nontrivial = true;
    return c;
};

GEN.ctx = (rnd, index) => {
    let c = baseCase("ctx", index, rnd);
    let mn = ["LDPCTX", "SVPCTX"][genCycle.ctx++ % 2];
    c.mnemonic = mn;
    c.byte(OPCODES.indexOf(mn) & 0xFF);
    /* Non-kernel makes it a reserved-instruction fault, which is worth grading too. */
    if (rnd() < 0.25) c.psl = (c.psl & ~(PSL_CUR | PSL_IS | PSL_IPL)) | (USER << PSL_V_CUR) | (USER << PSL_V_PRV);
    else c.psl = c.psl & ~PSL_CUR;
    c.regs[14] = (c.psl & PSL_IS) ? R_STACK[4] : R_STACK[(c.psl >>> PSL_V_CUR) & 3];
    for (let k = 0; k < 20; k++) {
        c.setLong(R_PCBB + k * 4, k < 4 ? R_STACK[k] : (Math.floor(rnd() * 0x100000000)) | 0);
    }
    /* PCB+72/76 are the PC/PSL LDPCTX pushes; keep the PSL legal-ish and the PC a NOP. */
    c.setLong(R_PCBB + 72, R_HANDLER);
    c.setLong(R_PCBB + 76, (KERN << PSL_V_CUR) | Math.floor(rnd() * 16));
    c.setLong(R_PCBB + 80, 0x80000000 | 0);                          // P0BR
    c.setLong(R_PCBB + 84, (Math.floor(rnd() * 5) << 24) | 0x200);   // ASTLVL | P0LR
    c.setLong(R_PCBB + 88, 0x40001000);                              // P1BR
    c.setLong(R_PCBB + 92, ((rnd() < 0.5 ? 1 : 0) << 31) | 0x3FFE00);// PME | P1LR
    let sp = c.regs[14];
    c.setLong(sp, R_HANDLER);
    c.setLong((sp + 4) | 0, (KERN << PSL_V_CUR) | Math.floor(rnd() * 16));
    c.nontrivial = true;
    return c;
};

GEN.privfault = (rnd, index) => {
    let c = baseCase("privfault", index, rnd);
    /* Reserved/privileged instruction from a non-kernel mode -> SCB_RESIN dispatch, and reserved
       operand (a write-only IPR read, or PR# > 63) -> SCB_RESOP dispatch. */
    let mode = pick(rnd, [KERN, 1, 2, USER]);
    c.psl = (mode << PSL_V_CUR) | (mode << PSL_V_PRV) | Math.floor(rnd() * 16);
    c.regs[14] = R_STACK[mode];
    let what = ["mfpr-wo", "mtpr-big", "halt"][genCycle.privfault++ % 3];
    if (what === "mfpr-wo") {
        c.mnemonic = "MFPR";
        c.byte(OPCODES.indexOf("MFPR") & 0xFF);
        emitPRN(c, rnd, pick(rnd, [MT.SIRR, MT.TBIA, MT.TBIS, MT.TBCHK]));
        c.byte(0x51);
    } else if (what === "mtpr-big") {
        c.mnemonic = "MTPR";
        c.byte(OPCODES.indexOf("MTPR") & 0xFF);
        c.byte(0x8F); c.long(0x12345678);
        emitPRN(c, rnd, 64 + Math.floor(rnd() * 100), true);
    } else {
        c.mnemonic = "HALT";
        c.byte(OPCODES.indexOf("HALT") & 0xFF);
    }
    c.nontrivial = true;
    return c;
};

const KINDS = ["mtpr", "mfpr", "rei", "chm", "swint", "errint", "trap", "trace", "bptxfc", "ctx", "privfault"];

/* --- SIMH batch driver ---------------------------------------------------------------------- */

const CASE_MARK = "XCASE_";

function buildScript(cases, extra)
{
    let L = ["set cpu 16m", "set cpu simhalt"];
    for (let c of cases) {
        L.push(`echo ${CASE_MARK}${c.index}`);
        /*
         * `reset all` between cases, not just once per batch.  The translation buffer is NOT
         * memory and no `deposit` reaches it, so without this a case that rewrites a page table
         * underneath a TB entry an EARLIER case filled translates through the earlier case's
         * mapping -- SIMH read the operand fine while the JS side (whose MMU is reset per case)
         * took the ACV the page table actually calls for.  The tlb device's reset is what clears
         * it.  Every register the case cares about is deposited immediately afterwards.
         */
        L.push("reset all");
        for (let p of PRIV) L.push(`deposit ${p.name} ${hex(c.priv[p.name])}`);
        for (let r = 0; r < 15; r++) L.push(`deposit R${r} ${hex(c.regs[r])}`);
        for (let a of c.zero) L.push(`deposit ${hex(a)} 0`);
        for (let w of c.mem) L.push(`deposit ${hex(w.addr)} ${hex(w.val)}`);
        for (let i = 0; i < c.instr.length; i++) L.push(`deposit -b ${hex(c.codePA + i)} ${c.instr[i].toString(16)}`);
        L.push(`deposit PSL ${hex(c.psl)}`);
        L.push(`deposit PC ${hex(c.pc)}`);
        L.push("step 1");
        let names = [];
        for (let r = 0; r < 15; r++) names.push("R" + r);
        names.push("PC", "PSL");
        for (let p of PRIV) names.push(p.name);
        L.push(`examine -h ${names.join(",")}`);
        for (let a of (extra || PROBE_ADDRS)) L.push(`examine -h ${hex(a)}`);
    }
    L.push("quit");
    return L.join("\n") + "\n";
}

/* SIMH decorates some registers with a decoded bit list after the value (PSL prints
   "041F0000\tCM0 TP0 ..."), so this must NOT anchor at end of line -- anchoring it silently
   dropped exactly one value per case and made every case "not reached". */
const VALUE_RE = /^(\S+):\s+([0-9A-Fa-f]+)/;

/**
 * runBatch(simh, cases, scratch, extra)
 *
 * @returns {Map<number, {regs: Int32Array, psl: number, priv: Object, mem: number[], reached: boolean}>}
 */
function runBatch(simh, cases, scratch, extra)
{
    let script = buildScript(cases, extra);
    let out = runSimh(simh, script, path.join(scratch, "excdiff-batch.ini"));
    if (process.env.EXCDIFF_DUMP) fs.writeFileSync(`${process.env.EXCDIFF_DUMP}.${cases[0].index}`, out);
    let probes = extra || PROBE_ADDRS;
    let want = 17 + PRIV.length + probes.length;
    let lines = out.split("\n");
    let results = new Map();
    let i = 0;
    while (i < lines.length) {
        let m = lines[i].match(new RegExp(CASE_MARK + "(\\d+)"));
        if (!m) { i++; continue; }
        let idx = +m[1];
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
        let psl = vals[16];
        let priv = {};
        for (let k = 0; k < PRIV.length; k++) priv[PRIV[k].name] = vals[17 + k];
        let mem = vals.slice(17 + PRIV.length);
        results.set(idx, {reached: true, regs, psl, priv, mem});
    }
    return results;
}

/* --- JS side ------------------------------------------------------------------------------- */

/**
 * runCaseJS(m, c, probes)
 *
 * @returns {{regs: Int32Array, psl: number, priv: Object, mem: number[], stop: ?string}}
 */
function runCaseJS(m, c, probes)
{
    let {bus, cpu} = m;
    /* Exactly the wipe the SIMH script performs, in the same order -- see ZERO_ADDRS. */
    for (let a of c.zero) bus.setLong(a, 0);

    cpu.exc.reset();
    cpu.mmu.reset();
    cpu.mmu.setMAPEN(0);
    for (let p of PRIV) p.set(cpu, c.priv[p.name] | 0);
    cpu.regs.set(c.regs);
    cpu.psl = c.psl | 0;
    for (let w of c.mem) bus.setLong(w.addr, w.val);
    for (let i = 0; i < c.instr.length; i++) bus.setByte((c.codePA + i) | 0, c.instr[i]);
    cpu.regs[15] = c.pc;

    let stop = null;
    try {
        cpu.stepOne();
    } catch (e) {
        if (e instanceof VAXStop) stop = e.reason;
        else if (e instanceof VAXFault) stop = "unhandled VAXFault " + e.code;
        else throw e;
    }
    let priv = {};
    for (let p of PRIV) priv[p.name] = p.get(cpu) | 0;
    let mem = (probes || PROBE_ADDRS).map((a) => bus.getLong(a) | 0);
    return {regs: Int32Array.from(cpu.regs), psl: cpu.psl, priv, mem, stop};
}

/**
 * compareCase(c, js, sr, probes)
 *
 * @returns {Array.<string>}
 */
function compareCase(c, js, sr, probes)
{
    let bad = [];
    let tag = `${c.kind} case ${c.index}${c.note ? " (" + c.note + ")" : ""}`;
    for (let r = 0; r < 16; r++) {
        if ((js.regs[r] | 0) !== (sr.regs[r] | 0)) {
            bad.push(`${tag}: R${r} js=${hex(js.regs[r])} simh=${hex(sr.regs[r])}`);
        }
    }
    if ((js.psl | 0) !== (sr.psl | 0)) bad.push(`${tag}: PSL js=${hex(js.psl)} simh=${hex(sr.psl)}`);
    for (let p of PRIV) {
        let mask = p.mask === undefined ? -1 : p.mask;
        if (((js.priv[p.name] | 0) & mask) !== ((sr.priv[p.name] | 0) & mask)) {
            bad.push(`${tag}: ${p.name} js=${hex(js.priv[p.name])} simh=${hex(sr.priv[p.name])}`);
        }
    }
    let addrs = probes || PROBE_ADDRS;
    for (let k = 0; k < addrs.length; k++) {
        if ((js.mem[k] | 0) !== (sr.mem[k] | 0)) {
            bad.push(`${tag}: mem[${hex(addrs[k])}] js=${hex(js.mem[k])} simh=${hex(sr.mem[k])}`);
        }
    }
    if (bad.length) bad.push(`${tag}:   instr=${c.instr.map((b) => hex(b, 2)).join(" ")} psl=${hex(c.psl)}`);
    return bad;
}

/**
 * phaseRandomized(simh, scratch, opts)
 */
function phaseRandomized(simh, scratch, opts)
{
    let rnd = mulberry32(opts.seed ^ 0x5EC0);
    resetGenerators();
    let cases = [];
    let index = 0;
    for (let kind of KINDS) {
        for (let k = 0; k < opts.cases; k++) cases.push(GEN[kind](rnd, index++));
    }
    let m = makeMachine();
    let failures = [], notReached = [];
    let perKind = new Map(), mnemSeen = new Map();
    let regComputedPRN = 0, iprSeen = new Set(), swIPLSeen = new Set();
    let vecSeen = new Set(), nontrivial = 0;
    const BATCH = 60;
    for (let start = 0; start < cases.length; start += BATCH) {
        let batch = cases.slice(start, start + BATCH);
        let sr = runBatch(simh, batch, scratch);
        for (let c of batch) {
            perKind.set(c.kind, (perKind.get(c.kind) || 0) + 1);
            if (c.mnemonic) mnemSeen.set(c.mnemonic, (mnemSeen.get(c.mnemonic) || 0) + 1);
            if (c.note) regComputedPRN++;
            if (c.prn !== undefined) iprSeen.add(c.prn >>> 0);
            if (c.nontrivial) nontrivial++;
            let r = sr.get(c.index);
            if (!r || !r.reached) {
                notReached.push(`${c.kind} case ${c.index} (SIMH produced ${r ? r.got : 0}/${r ? r.want : "?"} values)`);
                continue;
            }
            let js = runCaseJS(m, c);
            /* Which SCB vector did the step actually reach?  Both sides agree by construction if
               the comparison passes; read it off the JS side for the coverage tally. */
            if ((js.regs[15] >>> 0) >= R_HANDLER && (js.regs[15] >>> 0) < R_HANDLER + 0x40) {
                vecSeen.add((js.regs[15] - R_HANDLER) & 0x3C);
            }
            if (c.kind === "swint") {
                let taken = (c.priv.SISR & ~js.priv.SISR) & 0xFFFF;
                for (let b = 1; b <= IPL_SMAX; b++) if ((taken >>> b) & 1) swIPLSeen.add(b);
            }
            for (let b of compareCase(c, js, r)) failures.push(b);
        }
    }
    return {total: cases.length, failures, notReached, perKind, mnemSeen, regComputedPRN, iprSeen, swIPLSeen, vecSeen, nontrivial};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 3 -- mapped faults (ACV / TNV / KSNV)                                                    *
 *
 * A purpose-built system page table: S0 virtual page i maps to physical frame M_PFN0 + i, with a
 * protection code this phase chooses per page.  Everything the case touches -- the instruction,
 * the SCB, all five stacks -- lives in S0, so any of them can be made unreadable or invalid.
 * ------------------------------------------------------------------------------------------- */

const PAGE = 512;
const M_SBR    = 0x00400000;            // system page table, physical
const M_SLR    = 0x400;                 // 1024 S0 pages mapped (512KB of S0)
const M_PFN0   = 0x00200000 / PAGE;     // S0 page i -> physical 0x200000 + i*512
const M_S0     = 0x80000000 | 0;
const M_VA = {
    CODE:   (M_S0 + 0x10 * PAGE) | 0,
    HANDLER:(M_S0 + 0x12 * PAGE) | 0,
    SCBB:   (M_S0 + 0x20 * PAGE) | 0,
    KSTK:   (M_S0 + 0x30 * PAGE) | 0,   // kernel stack TOP is at the END of this page
    ISTK:   (M_S0 + 0x40 * PAGE) | 0,
    DATA:   (M_S0 + 0x50 * PAGE) | 0
};
function m_pa(va) { return (M_PFN0 * PAGE + (((va >>> 0) - (M_S0 >>> 0)) | 0)) | 0; }
function m_vpn(va) { return (((va >>> 0) - (M_S0 >>> 0)) / PAGE) | 0; }

/** The PTE protection code granting read+write in every mode, and the one granting nothing --
    both read out of the shipped CVTACC table rather than hardcoded from a manual. */
const PROT = (function() {
    let all = -1, none = -1;
    for (let c = 0; c < 16; c++) {
        let acc = MMUVAX.CVTACC[c];
        let needed = 0;
        for (let mode = 0; mode < 4; mode++) needed |= MMUVAX.accRead(mode) | MMUVAX.accWrite(mode);
        if (all < 0 && (acc & needed) === needed) all = c;
        if (none < 0 && acc === 0) none = c;
    }
    if (all < 0 || none < 0) throw new Error("excdiff: cannot find all-access / no-access PTE protection codes");
    return {all, none};
})();

function makePTE(pfn, prot, valid)
{
    return ((valid ? MMUVAX.PTE_V : 0) | (prot << MMUVAX.PTE_V_ACC) | MMUVAX.PTE_M | (pfn & 0x1FFFFF)) | 0;
}

/** Every longword both sides read back in the mapped phase (physical addresses). */
const M_PROBES_DEF = (function() {
    let a = [];
    for (let base of [M_VA.KSTK, M_VA.ISTK]) {
        let top = (base + PAGE) | 0;
        for (let k = -8; k <= 1; k++) a.push(m_pa((top + k * 4) | 0));
    }
    return a;
})();
const M_PROBES = M_PROBES_DEF;
const M_ZERO = (function() {
    let a = M_PROBES.slice();
    for (let k = 0; k < 16; k++) a.push((m_pa(M_VA.CODE) + k * 4) | 0);
    for (let k = 0; k < 16; k++) a.push((m_pa(M_VA.DATA) + k * 4) | 0);
    return a;
})();

const MAPPED_VARIANTS = ["kstack-acv", "kstack-tnv", "operand-acv", "operand-tnv", "clean",
                         "chm-acv", "chm-tnv", "chm-clean", "tbchk-hit", "tbchk-miss", "tbis"];

function mappedCase(rnd, index)
{
    let c = new Case("mapped", index, rnd);
    c.index = index;
    c.pc = M_VA.CODE;
    c.codePA = m_pa(M_VA.CODE);                 // `deposit` addresses memory PHYSICALLY
    c.zero = M_ZERO;
    for (let p of PRIV) c.priv[p.name] = 0;
    c.priv.SCBB = m_pa(M_VA.SCBB);              // SCBB is PHYSICAL
    c.priv.PCBB = 0;
    c.priv.SBR = M_SBR;
    c.priv.SLR = M_SLR;
    c.priv.MAPEN = 1;
    c.priv.KSP = (M_VA.KSTK + PAGE) | 0;        // top of the kernel-stack page
    c.priv.ESP = c.priv.SSP = c.priv.USP = (M_VA.DATA + PAGE) | 0;
    c.priv.IS = (M_VA.ISTK + PAGE) | 0;
    c.psl = (KERN << PSL_V_CUR) | Math.floor(rnd() * 16);
    c.regs.fill(0);
    c.regs[14] = c.priv.KSP;
    c.regs[15] = c.pc;

    /* Which page is broken, and how. */
    /* Cycled, not sampled: a 25-case --selfcheck batch sampling 11 variants uniformly draws zero
       kstack-acv/kstack-tnv cases often enough to matter. */
    let variant = MAPPED_VARIANTS[genCycle.mapped++ % MAPPED_VARIANTS.length];
    c.variant = variant;
    let badVPN = -1, badValid = true;
    if (variant === "operand-acv" || variant === "chm-acv") { badVPN = m_vpn(M_VA.DATA); badValid = true; }
    else if (variant === "operand-tnv" || variant === "chm-tnv") { badVPN = m_vpn(M_VA.DATA); badValid = false; }
    else if (variant === "kstack-acv") { badVPN = m_vpn(M_VA.KSTK); badValid = true; }
    else if (variant === "kstack-tnv") { badVPN = m_vpn(M_VA.KSTK); badValid = false; }

    /* The system page table: PTE for S0 page i at physical M_SBR + 4*i. */
    for (let i = 0; i < M_SLR; i++) {
        let prot = PROT.all, valid = true;
        if (i === badVPN) { prot = badValid ? PROT.none : PROT.all; valid = badValid ? true : false; }
        c.setLong(M_SBR + 4 * i, makePTE(M_PFN0 + i, prot, valid));
    }
    /* SCB: every vector -> the handler page.  Bit 0 chooses the interrupt stack; the KSNV variants
       need the FIRST fault to dispatch onto the KERNEL stack (so its push faults), so bit 0 is
       cleared there for the fault vectors and set for KSNV's own vector (which SVE forces anyway). */
    for (let v = 0; v < 0x100; v += 4) {
        let onIS = (variant === "kstack-acv" || variant === "kstack-tnv") ? 0 : ((rnd() < 0.3) ? 1 : 0);
        c.setLong(m_pa(M_VA.SCBB) + v, (M_VA.HANDLER + (v & 0x3C)) | onIS);
    }
    for (let k = 0; k < 16; k++) c.setLong(m_pa(M_VA.HANDLER) + k * 4, 0x01010101);   // NOPs

    /* The instruction: MOVL <abs data address>, R1 -- a plain operand read through the page that
       may be broken.  For the kstack variants the operand page is fine and the FAULT comes from
       MTPR with an out-of-range PR#, so that the exception's own push is what fails. */
    if (variant.startsWith("tb")) {
        /*
         * MTPR to TBCHK / TBIS, with mapping ON.  This is the only place the translation buffer
         * is OBSERVABLE: the case's own instruction fetch fills the TB entry for the code page, so
         * TBCHK against a code-page address reports a hit (CC<V> set) and against an untouched
         * page a miss -- and TBIS then invalidates one entry, which the NEXT case's `reset all`
         * would hide but which the post-state comparison does not depend on.  With mapping OFF
         * (the randomized phase) no TB entry is ever filled and CC<V> is always clear, so without
         * these variants TBCHK's V bit would be graded by nothing anywhere.
         */
        let target = (variant === "tbchk-hit") ? M_VA.CODE : M_VA.DATA;
        c.byte(OPCODES.indexOf("MTPR") & 0xFF);
        c.byte(0x8F); c.long(target);
        c.byte(0x8F); c.long(variant === "tbis" ? MT.TBIS : MT.TBCHK);
    } else if (variant.startsWith("chm-")) {
        /*
         * CHMx from USER mode into EXECUTIVE: op_chm probes the TARGET stack (ESP, which lives in
         * the page this variant may have broken) BEFORE writing anything, so a broken page means
         * an ACV/TNV whose own dispatch then lands on the intact kernel stack.  This is the only
         * generated case in which a fault is raised by a stack PROBE rather than by an access.
         */
        c.psl = (USER << PSL_V_CUR) | (USER << PSL_V_PRV) | (c.psl & CC_MASK);
        c.regs[14] = c.priv.USP;
        c.byte(OPCODES.indexOf(pick(rnd, ["CHME", "CHMS"])) & 0xFF);
        c.byte(0x8F); c.word(0x1234);
    } else if (variant === "kstack-acv" || variant === "kstack-tnv") {
        c.byte(OPCODES.indexOf("MTPR") & 0xFF);
        c.byte(0x8F); c.long(0x11223344);
        c.byte(0x8F); c.long(200);                                  // PR# > MT_MAX -> RESOP
    } else {
        c.byte(OPCODES.indexOf("MOVL") & 0xFF);
        c.byte(0x9F); c.long(M_VA.DATA + 8);
        c.byte(0x51);
    }
    c.setLong(m_pa((M_VA.DATA + 8) | 0), 0xDEADBEEF | 0);
    c.nontrivial = true;
    return c;
}

function phaseMapped(simh, scratch, opts)
{
    let rnd = mulberry32(opts.seed ^ 0x0AC0);
    genCycle.mapped = 0;
    let cases = [];
    for (let k = 0; k < opts.mapped; k++) cases.push(mappedCase(rnd, 500000 + k));
    let m = makeMachine();
    let failures = [], notReached = [];
    let variantSeen = new Map();
    const BATCH = 25;
    for (let start = 0; start < cases.length; start += BATCH) {
        let batch = cases.slice(start, start + BATCH);
        let sr = runBatch(simh, batch, scratch, M_PROBES);
        for (let c of batch) {
            variantSeen.set(c.variant, (variantSeen.get(c.variant) || 0) + 1);
            let r = sr.get(c.index);
            if (!r || !r.reached) {
                notReached.push(`mapped ${c.variant} case ${c.index} (SIMH produced ${r ? r.got : 0}/${r ? r.want : "?"} values)`);
                continue;
            }
            let js = runMappedJS(m, c);
            for (let b of compareCase(c, js, r, M_PROBES)) failures.push(b);
        }
    }
    return {total: cases.length, failures, notReached, variantSeen};
}

function runMappedJS(m, c)
{
    let {bus, cpu} = m;
    for (let a of c.zero) bus.setLong(a, 0);
    cpu.exc.reset();
    cpu.mmu.reset();
    for (let w of c.mem) bus.setLong(w.addr, w.val);
    for (let p of PRIV) p.set(cpu, c.priv[p.name] | 0);
    cpu.regs.set(c.regs);
    cpu.psl = c.psl | 0;
    for (let i = 0; i < c.instr.length; i++) bus.setByte((c.codePA + i) | 0, c.instr[i]);
    cpu.regs[15] = c.pc;
    let stop = null;
    try {
        cpu.stepOne();
    } catch (e) {
        if (e instanceof VAXStop) stop = e.reason;
        else if (e instanceof VAXFault) stop = "unhandled VAXFault " + e.code;
        else throw e;
    }
    let priv = {};
    for (let p of PRIV) priv[p.name] = p.get(cpu) | 0;
    return {regs: Int32Array.from(cpu.regs), psl: cpu.psl, priv, mem: M_PROBES.map((a) => bus.getLong(a) | 0), stop};
}

/* ------------------------------------------------------------------------------------------- *
 * --selfcheck                                                                                    *
 *
 * Mutations are applied to the SHIPPED object -- VAXExc.prototype, which is what every opcode body
 * in exc.js and every phase of this file actually calls -- not to a copy.  Each mutation is then run past a SLICE of the EHKAA replay (the trace is
 * generated once and reused), a small randomized batch and a small mapped batch, and must be
 * caught by at least one.  A mutation that survives is either a coverage hole or a provable
 * non-bug; both outcomes must be recorded in this file, and both currently are (see MUTATIONS).
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = [
    {name: "intexc: push PC and PSL at swapped offsets", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.intexc;
        VAXExc.prototype.intexc = function(cpu, vec, ipl, ei) {
            let r = orig.call(this, cpu, vec, ipl, ei);
            let sp = cpu.regs[14];
            let a = cpu.mmu.readData(sp, 4, cpu.accR()), b = cpu.mmu.readData((sp + 4) | 0, 4, cpu.accR());
            cpu.mmu.writeData(sp, b, 4, cpu.accW());
            cpu.mmu.writeData((sp + 4) | 0, a, 4, cpu.accW());
            return r;
        };
    }},
    {name: "intexc: dispatch to the kernel stack even when the SCB asks for the interrupt stack",
     phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.intexc;
        VAXExc.prototype.intexc = function(cpu, vec, ipl, ei) {
            let save = this.stk[4];
            this.stk[4] = this.stk[0];
            try { return orig.call(this, cpu, vec, ipl, ei); } finally { this.stk[4] = save; }
        };
    }},
    {name: "intexc: omit PSL<PRV> on an exception", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.intexc;
        VAXExc.prototype.intexc = function(cpu, vec, ipl, ei) {
            let r = orig.call(this, cpu, vec, ipl, ei);
            cpu.psl = cpu.psl & ~(3 << PSL_V_PRV);
            return r;
        };
    }},
    {name: "intexc: do not clear pending traps", phases: ["rnd"], apply() {
        let orig = VAXExc.prototype.intexc;
        VAXExc.prototype.intexc = function(cpu, vec, ipl, ei) {
            let save = this.trpirq;
            let r = orig.call(this, cpu, vec, ipl, ei);
            this.trpirq = save;
            return r;
        };
    }},
    {name: "rei: drop rule 7 (new IPL may not exceed the current one)", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.rei;
        VAXExc.prototype.rei = function(cpu) {
            let savedPSL = cpu.psl;
            cpu.psl = cpu.psl | PSL_IPL;                // pretend the current IPL is maximal
            try { return orig.call(this, cpu); } finally {
                if (cpu.psl === (savedPSL | PSL_IPL)) cpu.psl = savedPSL;
            }
        };
    }},
    {name: "rei: reload SP from the OLD mode's saved stack", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.rei;
        VAXExc.prototype.rei = function(cpu) {
            let oldcur = (cpu.psl >>> PSL_V_CUR) & 3;
            let r = orig.call(this, cpu);
            if (!(cpu.psl & PSL_IS)) cpu.regs[14] = this.stk[oldcur];
            return r;
        };
    }},
    {name: "rei: never deliver the AST software interrupt", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.rei;
        VAXExc.prototype.rei = function(cpu) {
            let before = this.sisr;
            let r = orig.call(this, cpu);
            this.sisr = (this.sisr & ~0x4) | (before & 0x4);
            return r;
        };
    }},
    {name: "mtpr: KSP always writes the saved KSP, never the live SP", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.mtpr;
        VAXExc.prototype.mtpr = function(cpu, val, prn) {
            if ((prn >>> 0) === MT.KSP) {
                let cc = orig.call(this, cpu, val, MT.ESP);
                this.stk[1] = 0;
                this.stk[0] = val;
                return cc;
            }
            return orig.call(this, cpu, val, prn);
        };
    }},
    {name: "mtpr: SIRR treats a request of 0 as level 0", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.mtpr;
        VAXExc.prototype.mtpr = function(cpu, val, prn) {
            let r = orig.call(this, cpu, val, prn);
            if ((prn >>> 0) === MT.SIRR) this.sisr = this.sisr | (1 << (val & 0xF));
            return r;
        };
    }},
    {name: "mfpr: IS ignores PSL<IS>", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.mfpr;
        VAXExc.prototype.mfpr = function(cpu, prn) {
            if ((prn >>> 0) === MT.IS) return this.stk[4];
            return orig.call(this, cpu, prn);
        };
    }},
    {name: "evalInt: return the LOWEST eligible software level", phases: ["rnd"], apply() {
        let orig = VAXExc.prototype.evalInt;
        VAXExc.prototype.evalInt = function(cpu) {
            let ipl = (cpu.psl >>> PSL_V_IPL) & PSL_M_IPL;
            let r = orig.call(this, cpu);
            if (r > 0 && r <= IPL_SMAX) {
                for (let i = ipl + 1; i <= IPL_SMAX; i++) if ((this.sisr >>> i) & 1) return i;
            }
            return r;
        };
    }},
    {name: "evalInt: memory error is taken at its own IPL as well as below it", phases: ["rnd"], apply() {
        let orig = VAXExc.prototype.evalInt;
        VAXExc.prototype.evalInt = function(cpu) {
            let ipl = (cpu.psl >>> PSL_V_IPL) & PSL_M_IPL;
            if (ipl === IPL_MEMERR && this.memErr) return IPL_MEMERR;
            if (ipl === IPL_CRDERR && this.crdErr && !this.memErr) return IPL_CRDERR;
            return orig.call(this, cpu);
        };
    }},
    {name: "chm: do not clamp an outward change-mode to the current mode", phases: ["ehkaa", "rnd"], apply() {
        let orig = VAXExc.prototype.chm;
        VAXExc.prototype.chm = function(cpu, opc, op0) {
            let cur = (cpu.psl >>> PSL_V_CUR) & 3;
            let mode = opc & 3;
            if (cur < mode) {
                /* SIMH clamps to `cur`; do not. */
                this.stk[cur] = cpu.regs[14];
                let newpc = cpu.mmu.readLP((this.scbb + SCB.CHMK + (mode << 2)) & VAX.PAMASK);
                let tsp = this.stk[mode];
                let acc = MMUVAX.accWrite(mode);
                cpu.mmu.writeData((tsp - 12) | 0, op0, 4, acc);
                cpu.mmu.writeData((tsp - 8) | 0, cpu.regs[15], 4, acc);
                cpu.mmu.writeData((tsp - 4) | 0, cpu.psl, 4, acc);
                cpu.regs[14] = (tsp - 12) | 0;
                cpu.psl = (mode << PSL_V_CUR) | (cpu.psl & PSL_IPL) | (cur << PSL_V_PRV);
                cpu.setPC(newpc & ~3);
                return 0;
            }
            return orig.call(this, cpu, opc, op0);
        };
    }},
    {name: "takeFault: report ACV/TNV normally even inside the exception flow (no KSNV)",
     phases: ["mapped"], apply() {
        let orig = VAXExc.prototype.takeFault;
        VAXExc.prototype.takeFault = function(cpu, fault) {
            let save = this.inIE;
            this.inIE = 0;
            try { return orig.call(this, cpu, fault); } finally { if (this.inIE === 0) this.inIE = 0; }
        };
    }},
    {name: "stepInstruction: take pending interrupts BEFORE pending traps", phases: ["rnd"], apply() {
        let orig = VAXExc.prototype.stepInstruction;
        VAXExc.prototype.stepInstruction = function(cpu, execute) {
            this.setIRQL(cpu);
            if (this.trpirq && ((this.trpirq >>> TIR_V_TRAP) & 7) && (this.trpirq & PSL_M_IPL)) {
                let lvl = this.trpirq & PSL_M_IPL;
                if (lvl && lvl <= IPL_SMAX) {
                    this.sisr = this.sisr & ~(1 << lvl);
                    this.intexc(cpu, SCB.IPLSOFT + (lvl << 2), lvl, IE.INT);
                    this.setIRQL(cpu);
                }
            }
            return orig.call(this, cpu, execute);
        };
    }}
];

function snapshotProto()
{
    let save = {};
    for (let k of Object.getOwnPropertyNames(VAXExc.prototype)) {
        if (k === "constructor") continue;
        save[k] = VAXExc.prototype[k];
    }
    return save;
}
function restoreProto(save)
{
    for (let k in save) VAXExc.prototype[k] = save[k];
}

function selfcheck(simh, scratch, opts)
{
    /* Generate the EHKAA trace ONCE; every mutation replays the same slice of it. */
    let ehkaaOpts = Object.assign({}, opts, {scratch, simh, reuseTrace: false});
    let warm = phaseEHKAA(ehkaaOpts);
    if (warm.skipped || warm.fatal) {
        console.error(`selfcheck: cannot run -- ${warm.reason || warm.fatal}`);
        return null;
    }
    if (warm.failures.length) {
        console.error(`selfcheck: the UNMUTATED module already fails the EHKAA phase (${warm.failures.length} failures); fix that first`);
        for (let f of warm.failures.slice(0, 5)) console.error("  " + f);
        return null;
    }
    ehkaaOpts.reuseTrace = true;

    let smallRnd = {seed: opts.seed, cases: 12};
    let smallMapped = {seed: opts.seed, mapped: 25};
    let results = [];
    for (let mut of MUTATIONS) {
        let save = snapshotProto();
        mut.apply();
        let caught = false, by = null;
        try {
            if (!caught && mut.phases.includes("ehkaa")) {
                let r = phaseEHKAA(ehkaaOpts);
                if (r.failures.length) { caught = true; by = "EHKAA"; }
            }
            if (!caught && mut.phases.includes("rnd")) {
                let r = phaseRandomized(simh, scratch, smallRnd);
                if (r.failures.length) { caught = true; by = "RANDOMIZED"; }
            }
            if (!caught && mut.phases.includes("mapped")) {
                let r = phaseMapped(simh, scratch, smallMapped);
                if (r.failures.length) { caught = true; by = "MAPPED"; }
            }
        } catch (e) {
            /* A mutation that makes the module throw is still detected -- but only if the throw is
               not a harness bug, so it is reported with its message. */
            caught = true; by = "threw: " + e.message;
        }
        restoreProto(save);
        results.push({name: mut.name, caught, by});
        console.log(`  selfcheck ${caught ? "CAUGHT   " : "*** NOT CAUGHT ***"} [${by || "-"}] ${mut.name}`);
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors                                                                                *
 * ------------------------------------------------------------------------------------------- */

const MIN_CASES_PER_KIND = 40;
/*
 * The IPR numbers the randomized phase must reach.  Derived from the head of the two edge tables --
 * exactly the part a run at the MINIMUM case count consumes -- so this is a set the floor
 * guarantees rather than a count that happens to come out high on a lucky seed.  (It did not: at
 * --cases 40 the distinct count ranged 35..43 across seeds, which is precisely the kind of
 * coverage assertion that passes for the wrong reason.)
 */
const REQUIRED_IPRS = (function() {
    let set = new Set();
    for (let e of MTPR_EDGE.slice(0, MIN_CASES_PER_KIND)) set.add(e[0] >>> 0);
    for (let n of MFPR_EDGE.slice(0, MIN_CASES_PER_KIND)) set.add(n >>> 0);
    return [...set].sort((a, b) => a - b);
})();
const MIN_MAPPED_CASES = 60;
const MIN_REG_COMPUTED_PRN = 40;
const MIN_NONTRIVIAL_FRACTION = 0.9;

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                           *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function main()
{
    let simh = findSimh(getArg("--simh", null));
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "excdiff-"));
    let seed = +getArg("--seed", String((Math.random() * 0xFFFFFFFF) | 0));
    let ehkaaExe = getArg("--ehkaa", path.join(vaxRepo(), "open-simh/VAX/tests/ehkaa.exe"));
    console.log(`excdiff.js: simh=${simh} scratch=${scratch} seed=${seed}`);

    if (process.argv.indexOf("--selfcheck") >= 0) {
        let results = selfcheck(simh, scratch, {seed, ehkaaExe});
        if (!results) process.exit(1);
        let bad = results.filter((r) => !r.caught);
        if (bad.length) {
            console.error(`SELFCHECK FAILED: ${bad.length} mutation(s) not caught:`);
            for (let b of bad) console.error("  - " + b.name);
            process.exit(1);
        }
        console.log(`selfcheck: all ${results.length} mutations caught.`);
        process.exit(0);
    }

    let cases = +getArg("--cases", "120");
    let mapped = +getArg("--mapped", "150");
    if (cases < MIN_CASES_PER_KIND) {
        console.error(`FATAL: --cases ${cases} is below the coverage floor (${MIN_CASES_PER_KIND}); an undersized run must fail, not quietly pass.`);
        process.exit(1);
    }
    if (mapped < MIN_MAPPED_CASES) {
        console.error(`FATAL: --mapped ${mapped} is below the coverage floor (${MIN_MAPPED_CASES}); an undersized run must fail, not quietly pass.`);
        process.exit(1);
    }

    let problems = [];

    console.log(`\n=== EHKAA phase ===`);
    let e = phaseEHKAA({simh, scratch, ehkaaExe, seed});
    if (e.skipped) {
        problems.push(`EHKAA: skipped (${e.reason}) -- the real-workload phase did NOT run`);
    } else if (e.fatal) {
        problems.push(`EHKAA: ${e.fatal}`);
    } else {
        console.log(`  trace records=${e.records} dispatches=${e.nExc} REI=${e.nRei} (rejected ${e.nReiFault}) CHMx=${e.nChm} MTPR=${e.nMtpr} MFPR=${e.nMfpr}`);
        console.log(`  stack switches: stay-on-IS=${e.stackSwitch.stay} to-IS=${e.stackSwitch.toIS} to-KSP=${e.stackSwitch.toKSP}`);
        console.log(`  vectors=${[...e.vecSeen.keys()].sort((a, b) => a - b).map((v) => hex(v, 2)).join(" ")}`);
        console.log(`  IPRs=${[...e.iprSeen.keys()].sort((a, b) => a - b).map((v) => hex(v, 2)).join(" ")}`);
        if (e.nTbchkPartial) console.log(`  note: ${e.nTbchkPartial} MTPR/TBCHK record(s) compared with CC<V> masked (TB contents are not in the trace; graded in full by the MAPPED phase's tbchk-hit/tbchk-miss variants)`);
        for (let v of EHKAA_VECTORS) {
            if (!e.vecSeen.has(v)) problems.push(`COVERAGE: EHKAA vector ${hex(v, 2)} (documented in ehkaa-profile.md §5) was never replayed`);
        }
        for (let v of e.vecSeen.keys()) {
            if (EHKAA_VECTORS.indexOf(v) < 0) problems.push(`COVERAGE: EHKAA took UNDOCUMENTED vector ${hex(v, 2)} -- ehkaa-profile.md §5 needs regenerating`);
        }
        for (let r of EHKAA_IPRS) {
            if (!e.iprSeen.has(r)) problems.push(`COVERAGE: EHKAA IPR ${hex(r, 2)} (documented in ehkaa-profile.md §4.2) was never replayed`);
        }
        for (let r of e.iprSeen.keys()) {
            if (EHKAA_IPRS.indexOf(r) < 0) problems.push(`COVERAGE: EHKAA accessed UNDOCUMENTED IPR ${hex(r, 2)} -- ehkaa-profile.md §4.2 needs regenerating`);
        }
        if (e.nExc < EHKAA_MIN_EVENTS) problems.push(`COVERAGE: only ${e.nExc} dispatch events replayed, floor ${EHKAA_MIN_EVENTS}`);
        if (e.nMtpr < EHKAA_MIN_MTPR) problems.push(`COVERAGE: only ${e.nMtpr} MTPR replayed, floor ${EHKAA_MIN_MTPR}`);
        if (e.nMfpr < EHKAA_MIN_MFPR) problems.push(`COVERAGE: only ${e.nMfpr} MFPR replayed, floor ${EHKAA_MIN_MFPR}`);
        if (e.stackSwitch.toIS < 1 || e.stackSwitch.toKSP < 1 || e.stackSwitch.stay < 1) {
            problems.push(`COVERAGE: not every stack-selection outcome occurred (${JSON.stringify(e.stackSwitch)})`);
        }
        if (e.chmProbeFaulted.length) {
            console.log(`  CHMx whose target-stack probe faulted in SIMH (not replayable with mapping off; graded by the MAPPED phase's chm-acv/chm-tnv variants): ${e.chmProbeFaulted.length}`);
            for (let n of e.chmProbeFaulted) console.log(`    ${n}`);
        }
        /* Every CHMA record must be accounted for: graded, or named above.  Neither silently dropped. */
        if (e.nChm !== (e.nChmGraded + e.chmProbeFaulted.length)) {
            problems.push(`COVERAGE: ${e.nChm} CHMx records but ${e.nChmGraded} graded + ${e.chmProbeFaulted.length} named = ${e.nChmGraded + e.chmProbeFaulted.length}`);
        }
        if (e.notReached.length) problems.push(`COVERAGE: ${e.notReached.length} EHKAA event(s) never reached comparison: ${e.notReached.slice(0, 5).join("; ")}`);
        for (let f of e.failures.slice(0, 25)) problems.push("EHKAA: " + f);
        if (e.failures.length > 25) problems.push(`EHKAA: ...and ${e.failures.length - 25} more failures`);
    }

    console.log(`\n=== RANDOMIZED phase: ${cases} cases x ${KINDS.length} kinds ===`);
    let r = phaseRandomized(simh, scratch, {seed, cases});
    console.log(`  total=${r.total} failures=${r.failures.length} notReached=${r.notReached.length}`);
    console.log(`  register-computed PR# cases=${r.regComputedPRN} distinct IPRs=${r.iprSeen.size} software IPLs taken=${[...r.swIPLSeen].sort((a, b) => a - b).join(",")}`);
    console.log(`  SCB vectors reached=${[...r.vecSeen].sort((a, b) => a - b).map((v) => hex(v, 2)).join(" ")}`);
    console.log(`  opcodes executed: ${[...r.mnemSeen.entries()].sort().map(([k, v]) => `${k}=${v}`).join(" ")}`);
    for (let k of KINDS) {
        let n = r.perKind.get(k) || 0;
        if (n < MIN_CASES_PER_KIND) problems.push(`COVERAGE: kind ${k} got only ${n} cases (floor ${MIN_CASES_PER_KIND})`);
    }
    for (let mn of REQUIRED_MNEMONICS) {
        let n = r.mnemSeen.get(mn) || 0;
        if (n < 5) problems.push(`COVERAGE: opcode ${mn} was executed only ${n} time(s) (floor 5) -- every opcode exc.js implements must be graded`);
    }
    for (let mn of EXC_IMPLEMENTED) {
        if (REQUIRED_MNEMONICS.indexOf(mn) < 0) problems.push(`COVERAGE: exc.js implements ${mn} but this file's REQUIRED_MNEMONICS does not list it -- add a generator`);
    }
    if (r.regComputedPRN < MIN_REG_COMPUTED_PRN) problems.push(`COVERAGE: only ${r.regComputedPRN} register-computed PR# cases (floor ${MIN_REG_COMPUTED_PRN}); a literal-only MTPR/MFPR would pass`);
    {
        let missing = REQUIRED_IPRS.filter((n) => !r.iprSeen.has(n));
        if (missing.length) problems.push(`COVERAGE: IPR number(s) ${missing.map((n) => hex(n, 2)).join(" ")} were never exercised by MTPR or MFPR`);
    }
    if (r.swIPLSeen.size < 12) problems.push(`COVERAGE: only ${r.swIPLSeen.size} distinct software interrupt levels taken (floor 12 of 15) -- IPL arbitration was not exercised`);
    if (r.nontrivial / r.total < MIN_NONTRIVIAL_FRACTION) problems.push(`COVERAGE: non-trivial fraction ${(r.nontrivial / r.total).toFixed(3)} < floor ${MIN_NONTRIVIAL_FRACTION}`);
    if (r.notReached.length) problems.push(`COVERAGE: ${r.notReached.length} randomized case(s) never reached comparison: ${r.notReached.slice(0, 10).join("; ")}`);
    for (let f of r.failures.slice(0, 25)) problems.push("RANDOMIZED: " + f);
    if (r.failures.length > 25) problems.push(`RANDOMIZED: ...and ${r.failures.length - 25} more failures`);

    console.log(`\n=== MAPPED phase: ${mapped} cases ===`);
    let mp = phaseMapped(simh, scratch, {seed, mapped});
    console.log(`  total=${mp.total} failures=${mp.failures.length} notReached=${mp.notReached.length}`);
    console.log(`  variants: ${[...mp.variantSeen.entries()].map(([k, v]) => `${k}=${v}`).join(" ")}`);
    for (let v of MAPPED_VARIANTS) {
        if (!(mp.variantSeen.get(v) > 0)) problems.push(`COVERAGE: mapped variant ${v} never generated`);
    }
    if (mp.notReached.length) problems.push(`COVERAGE: ${mp.notReached.length} mapped case(s) never reached comparison: ${mp.notReached.slice(0, 10).join("; ")}`);
    for (let f of mp.failures.slice(0, 25)) problems.push("MAPPED: " + f);
    if (mp.failures.length > 25) problems.push(`MAPPED: ...and ${mp.failures.length - 25} more failures`);

    if (problems.length) {
        console.error(`\nFAILED (${problems.length} problem(s)), seed=${seed}:`);
        for (let p of problems) console.error("  - " + p);
        process.exit(1);
    }
    console.log(`\nPASSED. seed=${seed}`);
}

main();
