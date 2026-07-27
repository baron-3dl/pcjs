/**
 * @fileoverview Differentially grades modules/v2/cmctl.js -- the KA655 CMCTL memory-controller
 *               register file -- against a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * ============================================================================
 * WHAT THIS IS
 * ============================================================================
 * pcjsvax-622.  vax_sysdev.c's cmctl_rd()/cmctl_wr() (:1106-1163) are nineteen registers at
 * REGBASE+0x100 that the KA655 firmware's memory sizing and self-test read and write.  This file
 * grades every one of them -- values AND side effects, not merely "does not fault" -- against a
 * live `microvax3900`.
 *
 * ============================================================================
 * BOTH ENGINES RUN REAL INSTRUCTIONS.  THIS IS NOT A STYLE CHOICE.
 * ============================================================================
 * Every case here is a short sequence of MOVL/MOVW/MOVB/MOVZWL/MOVZBL instructions, deposited into
 * RAM and single-stepped, on SIMH AND on CPUStateVAX.stepCPU().  HANDOFF.md §7 premise 7 is why:
 * pcjsvax-855 shipped a fix to ssc.js whose branches nothing ever called, and found out only
 * because its differential executed real instructions instead of calling the device accessor.  Two
 * of this file's own subjects are unreachable any other way -- register 18's machine check exists
 * only as a control transfer to the SCB_MCHK handler, and the SSC bus-timeout residue that tells a
 * handler-raised machine check apart from an undecoded address is set on the fault path, not in the
 * device.
 *
 * ============================================================================
 * THE SPAN IS DERIVED THREE INDEPENDENT WAYS AND THEY MUST AGREE (standing rule 5)
 * ============================================================================
 * vaxmod_defs.h:206 writes `CMCTLSIZE (19 << 2)`.  Nothing here copies that 19.
 *
 *   1. cmctl.js COMPUTES it: MAXMEMSIZE / MEM_BANK config registers, plus the three named
 *      non-configuration registers.  16 + 3.
 *   2. PHASE SPAN MEASURES it on the LIVE ORACLE, by executing one longword read at each of
 *      CMCTLBASE-8, -4, +0 ... +(19<<2)+8 and classifying the outcome three ways -- answered
 *      normally / machine-checked with the SSC bus-timeout bits CLEAR / machine-checked with them
 *      SET.  Only an address vax_sysdev.c's regtable[] does not cover reaches ReadReg()'s
 *      fall-through, and only that fall-through sets the bus-timeout bits, so the third class is
 *      exactly "outside CMCTL" and the first two are exactly "inside".
 *   3. The SAME probe runs on THIS machine and must classify every address identically.
 *
 * If any two disagree the run fails and names the addresses.  The CIS opcode count in this project
 * went 7 -> 11 -> 17 -> 23 and every wrong value along the way was hand-derived.
 *
 * ============================================================================
 * THE EXTENSION REGISTER IS GRADED ON BOTH SIDES OF ITS BRANCH
 * ============================================================================
 * cmctl_rd()'s register 18 returns MEMSIZE when MEMSIZE > MAXMEMSIZE and machine-checks otherwise;
 * cmctl_wr()'s machine-checks at ANY size.  MAXMEMSIZE is 1<<26 = 64MB (vaxmod_defs.h:121-122) --
 * NOT the 128MB an earlier note in pcjsvax-622 recorded -- so the whole matrix is:
 *
 *              16MB (the SIMH default)            128MB (`set cpu 128m`)
 *      read    machine check, BTO unchanged       0x08000000, no fault
 *      write   machine check, BTO unchanged       machine check, BTO unchanged
 *
 * All FOUR cells are driven.  The 128MB half needs a genuinely 128MB machine on both engines --
 * `set cpu 128m` on the oracle, `bus.addMemory(0, 0x08000000)` here -- because MEMSIZE is also what
 * the signature request's ADDR_IS_MEM() tests, so a machine that merely TOLD CMCTL it had 128MB
 * would grade one branch and quietly falsify the other.  That second machine is built ONCE and
 * reused (standing rule 14); it is the single largest allocation this file makes and the peak-heap
 * bound below is sized for it and asserted.
 *
 * The 128MB configuration is also what makes the signature request's bank test observable in BOTH
 * directions: at 16MB banks 0-3 are populated and 4-15 are not, and at 128MB all sixteen are.
 *
 * ============================================================================
 * TWO OBSERVATION CHANNELS, AND WHY THE SECOND IS NOT REDUNDANT
 * ============================================================================
 * Every case reports (a) the registers the case's own read instructions loaded, the final PC, and
 * the SSC bus-timeout register -- what the PROGRAM can see -- and (b) all nineteen entries of the
 * controller's backing array, read on SIMH with `examine CMCSR[n]` and here from `cmctl.reg[n]`.
 * (b) sees what (a) cannot: cmctl_rd() masks its answer, so a defect that stores the wrong bits and
 * masks them away on the way out is invisible to the program until some LATER read of a different
 * register exposes it.  The CMERR register is the sharp case -- it is write-1-to-clear with no
 * software path that can ever SET it, so a preload through channel (b) is the only way to observe
 * its clear at all.  Preloads are applied identically on both engines (`deposit CMCSR[n]` /
 * `cmctl.reg[n] = `) and are harness state on both, not a modelled mechanism on either.
 *
 * ============================================================================
 * WHAT IS NOT GRADED, BY NAME
 * ============================================================================
 * UNALIGNED word and longword writes.  SIMH routes them through WriteRegU() (vax_sysdev.c:1084-1091),
 * which read-modify-writes via ReadReg()+WriteReg(); this bus reaches them through MemoryVAX's own
 * byte/word stitching, which does not reproduce that.  It is the identical, already-disclosed gap
 * cdg.js's and cdgdiff.js's headers name, it is a bus-level change rather than a CMCTL-level one,
 * and it is out of pcjsvax-622's scope.  Word writes are therefore graded only at the two genuinely
 * aligned lanes (0 and 2); BYTE writes, which are aligned by definition, are graded at all four.
 *
 *      node machines/dec/vax/tests/cmctldiff.js [options]
 *        --simh PATH        patched microvax3900; else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch
 *                            build (the same search every other differential here uses)
 *        --random-cases N   randomized cases per memory configuration (default 40, floor 16)
 *        --seed S           PRNG seed, printed on failure so it reproduces
 *        --selfcheck        prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import { VAX } from "../modules/v2/defines.js";
import { OPCODES } from "../modules/v2/drom.js";
import { SCB } from "../modules/v2/exc.js";
import { REG_MCHK } from "../modules/v2/regblock.js";
import CMCTLVAX, {
    CMCTL_BASE, CMCTL_LENGTH, CMCTL_REGS, CMCTL_CONFIG_REGS, CMCTL_SPECIAL,
    REG_CMERR, REG_CMCSR, REG_CMEXT,
    CMCNF_VLD, CMCNF_BA, CMCNF_SRQ, CMCNF_SIG, CMCNF_RW,
    CMERR_W1C, CMCSR_MASK, MEM_BANK, MEM_SIG
} from "../modules/v2/cmctl.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/** The two memory configurations, in MB.  SMALL is the SIMH microvax3900 default and every other
    differential's size; BIG is the smallest size SIMH offers ABOVE MAXMEMSIZE (its MEM_MODIFIERS
    table jumps 64M -> 128M -> 256M -> 512M, so there is nothing between 64 and 128). */
const MEM_SMALL_MB = 16;
const MEM_BIG_MB   = 128;

/* Fixed physical layout, MAPPING OFF -- the convention tmrdiff.js / sscunaligneddiff.js use. */
const R_SCBB    = 0x00100000;
const R_HANDLER = 0x00102000;           // a page of NOPs; SCBB+SCB.MCHK points here
const R_CODE    = 0x00104000;
const R_KSP     = 0x00110000;
/* A machine check is a SEVERE exception: vax_cpu1.c's intexc() reloads SP from the INTERRUPT stack
   regardless of the current mode, so IS must be set or the frame push itself faults inside `in_ie`
   and SIMH stops hard instead of dispatching.  sscunaligneddiff.js's header records measuring
   exactly that. */
const R_IS      = 0x00118000;

/** How many general registers a case may observe.  Cases are written against this, and the floor
    below asserts at least one case uses the last one, so the readback plumbing is never wider than
    what is proven to be parsed. */
const OBS_REGS = 4;

/** Randomized cases per memory configuration, and the FIXED floor beneath it.  The floor does not
    scale with anything (standing rule 4). */
const RANDOM_CASES_DEFAULT = 40;
const RANDOM_CASES_FLOOR   = 16;

/**
 * ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure and NOT scaled by case
 * count (standing rule 4/14).  The dominant term is fixed and known: the BIG machine's 128MB of RAM
 * plus the SMALL machine's 16MB, both allocated once.  512MB leaves room for GC scheduling and for
 * --selfcheck's repeated grading passes over the SAME two machines, without leaving room for the
 * 8.6GB machine-per-case regression this class of bound exists to catch.
 */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;
let PEAK_HEAP = 0;

function sampleHeap()
{
    let mu = process.memoryUsage();
    let used = mu.heapUsed + mu.external;
    if (used > PEAK_HEAP) PEAK_HEAP = used;
    return used;
}

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Plumbing -- the same shape romdiff.js / cdgdiff.js use, deliberately                          *
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
    throw new Error("cmctldiff needs a REAL SIMH microvax3900; it has no fixture fallback.  Build one\n" +
        "with machines/dec/vax/tests/simh/build.sh and pass --simh PATH.  Tried:\n  " + candidates.join("\n  "));
}

function runSimh(bin, script, iniPath, timeoutMs = 5 * 60 * 1000)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: timeoutMs});
}

/** The same mulberry32 every VAX differential in this tree uses; duplicated because none of them
    share a utility module (see busdiff.js/mchkdiff.js/romdiff.js/cdgdiff.js). */
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
 * Instruction encoding -- ONE op description, two engines                                       *
 * ------------------------------------------------------------------------------------------- */

const OP_MOVL   = OPCODES.indexOf("MOVL");
const OP_MOVW   = OPCODES.indexOf("MOVW");
const OP_MOVB   = OPCODES.indexOf("MOVB");
const OP_MOVZWL = OPCODES.indexOf("MOVZWL");
const OP_MOVZBL = OPCODES.indexOf("MOVZBL");
const OP_NOP    = OPCODES.indexOf("NOP");
for (let [name, opc] of [["MOVL", OP_MOVL], ["MOVW", OP_MOVW], ["MOVB", OP_MOVB],
                         ["MOVZWL", OP_MOVZWL], ["MOVZBL", OP_MOVZBL], ["NOP", OP_NOP]]) {
    if (opc < 0 || opc > 0xFF) throw new Error(`cmctldiff: ${name} opcode not found or not single-byte`);
}

/**
 * encodeOp(op)
 *
 * An op is one of
 *   {kind: "wl"|"ww"|"wb", addr, val}   write a longword / aligned word / byte  (MOVx #imm,@#addr)
 *   {kind: "rl", addr, reg}             read a longword into Rn                 (MOVL  @#addr,Rn)
 *   {kind: "rw", addr, reg}             read a word,  ZERO-EXTENDED into Rn     (MOVZWL @#addr,Rn)
 *   {kind: "rb", addr, reg}             read a byte,  ZERO-EXTENDED into Rn     (MOVZBL @#addr,Rn)
 *
 * The reads are MOVZWL/MOVZBL rather than MOVW/MOVB precisely BECAUSE a MOVW/MOVB to a register
 * leaves the register's upper bits alone: the readback would then be a mix of this case's answer
 * and whatever the previous case left behind, which is a differential that agrees for the wrong
 * reason.  The zero-extending forms make the observed longword the whole answer.
 *
 * @param {Object} op
 * @returns {Array.<number>} the bytes of exactly ONE instruction
 */
function encodeOp(op)
{
    let a = op.addr >>> 0;
    let abs = [0x9F, a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF];
    switch (op.kind) {
    case "wl": {
        let v = op.val >>> 0;
        return [OP_MOVL, 0x8F, v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF, ...abs];
    }
    case "ww":
        return [OP_MOVW, 0x8F, op.val & 0xFF, (op.val >>> 8) & 0xFF, ...abs];
    case "wb":
        return [OP_MOVB, 0x8F, op.val & 0xFF, ...abs];
    case "rl":
        return [OP_MOVL, ...abs, 0x50 | op.reg];
    case "rw":
        return [OP_MOVZWL, ...abs, 0x50 | op.reg];
    case "rb":
        return [OP_MOVZBL, ...abs, 0x50 | op.reg];
    }
    throw new Error("cmctldiff: bad op kind " + op.kind);
}

/** Address of CMCTL register `rg`, and of byte lane `lane` within it. */
function cm(rg, lane = 0) { return (CMCTL_BASE + rg * 4 + lane) >>> 0; }

/* ------------------------------------------------------------------------------------------- *
 * The machine under test -- ONE per memory configuration, reused across every case              *
 * ------------------------------------------------------------------------------------------- */

/**
 * makeMachine(memBytes, opts)
 *
 * RAM at 0 and the register block with CMCTL in it.  No console, no NVR, no timers: nothing graded
 * here touches any of them, the same minimal shape sscunaligneddiff.js and tmrdiff.js use.
 *
 * `opts.spanDelta` shortens or lengthens the DECLARED length of the CMCTL sub-range without
 * touching cmctl.js.  It exists for the `cmctl-span-one-register-short` mutation, which is a wiring
 * defect rather than a behavioural one and cannot be expressed by perturbing the device: the
 * shipped construction is perturbed, not replaced (standing rule 11).
 *
 * @param {number} memBytes
 * @param {Object} [opts]
 * @returns {Object} {bus, cpu, cmctl, memBytes}
 */
function makeMachine(memBytes, opts = {})
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, memBytes, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let cmctl = new CMCTLVAX(memBytes);
    bus.addRegBlock([
        {base: CMCTL_BASE, length: (CMCTL_LENGTH + (opts.spanDelta || 0)) >>> 0, dev: cmctl}
    ]);
    sampleHeap();
    return {bus, cpu, cmctl, memBytes};
}

/** The TWO machines.  See the file header: one per configuration, built once, reused for every case
    and every --selfcheck pass. */
const MACHINES = new Map();

function machine(memBytes, opts)
{
    let key = `${memBytes}:${opts && opts.spanDelta || 0}`;
    if (!MACHINES.has(key)) MACHINES.set(key, makeMachine(memBytes, opts));
    return MACHINES.get(key);
}

/**
 * runCaseJS(memBytes, kase, opts)
 *
 * Executes one case on the shared machine for its memory configuration, THROUGH THE CPU.
 *
 * @param {number} memBytes
 * @param {Object} kase {name, preload, ops, obs}
 * @param {Object} [opts]
 * @returns {Object} {regs, pc, bto, reg, stop}
 */
function runCaseJS(memBytes, kase, opts = {})
{
    let m = machine(memBytes, opts);
    let {bus, cpu, cmctl} = m;

    /* SIMH's per-case `reset -p all`: sysd_powerup() zeroes cmctl_reg and ssc_bto. */
    cmctl.reset();
    cpu.exc.sscBto = 0;
    cpu.exc.scbb = R_SCBB;
    cpu.regs[14] = R_KSP;
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
    for (let k = 0; k < OBS_REGS; k++) cpu.regs[k] = 0;
    for (let k = 0; k < 16; k++) bus.setByte((R_HANDLER + k) >>> 0, OP_NOP);
    bus.setLong((R_SCBB + SCB.MCHK) >>> 0, R_HANDLER);
    for (let p of kase.preload || []) cmctl.reg[p.rg] = p.val | 0;

    let bytes = [];
    for (let op of kase.ops) bytes.push(...encodeOp(op));
    for (let i = 0; i < bytes.length; i++) bus.setByte((R_CODE + i) >>> 0, bytes[i]);
    for (let k = 0; k < 8; k++) bus.setByte((R_CODE + bytes.length + k) >>> 0, OP_NOP);
    cpu.psl = 0;
    cpu.setPC(R_CODE);

    let stop = null;
    try {
        for (let s = 0; s < kase.ops.length; s++) cpu.stepCPU(1);
    } catch (e) {
        stop = e.reason || e.message || String(e);
    }
    sampleHeap();
    return {
        regs: Array.from({length: OBS_REGS}, (_, i) => cpu.regs[i] >>> 0),
        pc: cpu.regs[15] >>> 0,
        bto: cpu.exc.sscBto >>> 0,
        reg: Array.from({length: CMCTL_REGS}, (_, i) => cmctl.reg[i] >>> 0),
        stop
    };
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side -- ONE invocation per memory configuration (mchkdiff.js's runBatch convention)  *
 * ------------------------------------------------------------------------------------------- */

const MARK = "CMCASE";

/**
 * runCasesSimh(simh, opts, memMB, cases)
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {number} memMB
 * @param {Array.<Object>} cases
 * @returns {Array.<?Object>} parallel to `cases`; null for a case whose chunk never appeared
 */
function runCasesSimh(simh, opts, memMB, cases)
{
    let L = [`set cpu ${memMB}m`, "set cpu simhalt"];
    for (let i = 0; i < cases.length; i++) {
        L.push(`echo ${MARK}${i}`, "reset -p all", "deposit MAPEN 0");
        L.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit KSP ${hex(R_KSP)}`,
               `deposit R14 ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`,
               `deposit -l ${hex((R_SCBB + SCB.MCHK) >>> 0)} ${hex(R_HANDLER)}`);
        for (let k = 0; k < OBS_REGS; k++) L.push(`deposit R${k} 0`);
        for (let k = 0; k < 16; k++) L.push(`deposit -b ${hex(R_HANDLER + k)} ${OP_NOP.toString(16)}`);
        for (let p of cases[i].preload || []) L.push(`deposit CMCSR[${p.rg}] ${hex(p.val)}`);
        let bytes = [];
        for (let op of cases[i].ops) bytes.push(...encodeOp(op));
        for (let k = 0; k < bytes.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${bytes[k].toString(16)}`);
        for (let k = 0; k < 8; k++) L.push(`deposit -b ${hex(R_CODE + bytes.length + k)} ${OP_NOP.toString(16)}`);
        L.push("deposit PSL 0", `deposit PC ${hex(R_CODE)}`, `step ${cases[i].ops.length}`);
        L.push(`examine -h ${Array.from({length: OBS_REGS}, (_, k) => "R" + k).join(",")}`);
        L.push("examine -h PC", "examine -h BTO");
        for (let r = 0; r < CMCTL_REGS; r++) L.push(`examine -h CMCSR[${r}]`);
    }
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, `cmctldiff-${memMB}m.ini`));

    let results = new Array(cases.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = +parts[i];
        let chunk = parts[i + 1] || "";
        let regs = [];
        for (let k = 0; k < OBS_REGS; k++) {
            let m = new RegExp(`^R${k}:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
            if (!m) { regs = null; break; }
            regs.push(parseInt(m[1], 16) >>> 0);
        }
        let pcm = /^PC:\s*([0-9A-Fa-f]+)/m.exec(chunk);
        let btm = /^BTO:\s*([0-9A-Fa-f]+)/m.exec(chunk);
        let reg = [];
        for (let r = 0; r < CMCTL_REGS; r++) {
            let m = new RegExp(`^CMCSR\\[${r}\\]:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
            if (!m) { reg = null; break; }
            reg.push(parseInt(m[1], 16) >>> 0);
        }
        if (!regs || !pcm || !btm || !reg) continue;      // left null -> reported BY NAME by grade()
        results[idx] = {regs, pc: parseInt(pcm[1], 16) >>> 0, bto: parseInt(btm[1], 16) >>> 0, reg};
    }
    return results;
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

/** The address offsets PHASE SPAN probes, as REGISTER INDICES relative to CMCTLBASE.  Two below the
    base and three past the computed end, so the classification has to find BOTH edges rather than
    being handed one.  Derived from CMCTL_REGS, never written out. */
const SPAN_PROBE_LO = -2;
const SPAN_PROBE_HI = CMCTL_REGS + 2;

/**
 * buildCases(memBytes, nRandom, seed)
 *
 * Returns {cases, meta}.  `meta[phase]` carries what each phase needs to grade and to count.
 * Every address is computed from CMCTL_BASE / CMCTL_REGS / CMCTL_CONFIG_REGS (standing rule 5).
 *
 * @param {number} memBytes
 * @param {number} nRandom
 * @param {number} seed
 * @returns {Object}
 */
function buildCases(memBytes, nRandom, seed)
{
    let rnd = mulberry32(seed ^ memBytes);
    let cases = [];
    let meta = {span: [], config: [], srq: [], cmerr: [], cmcsr: [], ext: [], merge: [], subread: [], random: []};
    let add = (name, ops, phase, extra) => {
        let idx = cases.length;
        cases.push({name, ops, preload: (extra && extra.preload) || []});
        meta[phase].push(Object.assign({idx, name, ops}, extra || {}));
        return idx;
    };

    /* ---------------- SPAN ----------------
       One longword read per probed register index.  Classification is the GRADED product; nothing
       here asserts what the answer should be. */
    for (let k = SPAN_PROBE_LO; k < SPAN_PROBE_HI; k++) {
        add(`SPAN rg=${k} @0x${hex(cm(k))}`, [{kind: "rl", addr: cm(k), reg: 0}], "span", {rg: k});
    }

    /* ---------------- CONFIG ----------------
       Every configuration register, written and read back through the CPU.  The value matrix is
       chosen so that each of CMCNF_RW's two fields, the signature field, and the bits belonging to
       NEITHER are separately observable.  CMCNF_SRQ is deliberately absent here -- it has its own
       phase -- so a defect in the RW mask cannot hide behind a signature update. */
    let cfgVals = [
        CMCNF_VLD >>> 0,                    // the valid bit alone
        CMCNF_BA >>> 0,                     // the base-address field alone
        (CMCNF_VLD | CMCNF_BA) >>> 0,       // all of CMCNF_RW
        (~CMCNF_RW & ~CMCNF_SRQ) >>> 0,     // NONE of CMCNF_RW, and no signature request
        0x00000000,
        0x01234567,
        0x89ABCDEF & ~CMCNF_SRQ
    ];
    for (let rg = 0; rg < CMCTL_CONFIG_REGS; rg++) {
        let v = cfgVals[rg % cfgVals.length] >>> 0;
        add(`CONFIG rg=${rg} val=0x${hex(v)}`,
            [{kind: "wl", addr: cm(rg), val: v}, {kind: "rl", addr: cm(rg), reg: 0}],
            "config", {rg, val: v});
    }
    /* And every value at ONE register, so the matrix is a full cross rather than a diagonal. */
    for (let v of cfgVals) {
        add(`CONFIG-XV rg=0 val=0x${hex(v)}`,
            [{kind: "wl", addr: cm(0), val: v >>> 0}, {kind: "rl", addr: cm(0), reg: 0}],
            "config", {rg: 0, val: v >>> 0});
    }
    /* A PRELOADED register written with a value whose RW field is zero: proves the write REPLACES
       CMCNF_RW rather than OR-ing into it, and that it leaves the signature field alone. */
    for (let rg of [0, CMCTL_CONFIG_REGS - 1]) {
        add(`CONFIG-REPLACE rg=${rg}`,
            [{kind: "wl", addr: cm(rg), val: 0}, {kind: "rl", addr: cm(rg), reg: 0}],
            "config", {rg, val: 0, preload: [{rg, val: (CMCNF_VLD | CMCNF_BA | CMCNF_SIG) >>> 0}]});
    }

    /* ---------------- SRQ ----------------
       A signature request written to EVERY configuration register, reading back all four registers
       of the affected group plus one register OUTSIDE it.  The out-of-group read is what makes
       "signs a group of four" different from "signs everything". */
    for (let rg = 0; rg < CMCTL_CONFIG_REGS; rg++) {
        let g = rg & ~3;
        let outside = (g + 4) % CMCTL_CONFIG_REGS;
        let ops = [{kind: "wl", addr: cm(rg), val: CMCNF_SRQ >>> 0}];
        for (let i = 0; i < 4; i++) ops.push({kind: "rl", addr: cm(g + i), reg: i});
        add(`SRQ rg=${rg} group=${g}-${g + 3} (outside ${outside})`, ops, "srq", {rg, group: g, outside});
    }
    /* Preload the WHOLE group with signature bits, then request a signature: registers whose bank is
       not populated must come back with the signature CLEARED, not merely left alone. */
    for (let rg of [0, CMCTL_CONFIG_REGS - 4]) {
        let g = rg & ~3;
        let ops = [{kind: "wl", addr: cm(rg), val: CMCNF_SRQ >>> 0}];
        for (let i = 0; i < 4; i++) ops.push({kind: "rl", addr: cm(g + i), reg: i});
        add(`SRQ-CLEAR rg=${rg}`, ops, "srq",
            {rg, group: g, preload: Array.from({length: 4}, (_, i) => ({rg: g + i, val: CMCNF_SIG >>> 0}))});
    }

    /* ---------------- CMERR ----------------
       Write-1-to-clear, and the ONE register cmctl_rd() answers UNMASKED.  There is no software path
       that can set it, so every case preloads it -- see the file header. */
    let errPreload = 0xFFFFFFFF >>> 0;
    let errVals = [CMERR_W1C >>> 0, 0x00000000, 0x80000000 >>> 0, 0x00000180,
                   (~CMERR_W1C) >>> 0, 0xFFFFFFFF >>> 0];
    for (let v of errVals) {
        add(`CMERR val=0x${hex(v)}`,
            [{kind: "wl", addr: cm(REG_CMERR), val: v}, {kind: "rl", addr: cm(REG_CMERR), reg: 0}],
            "cmerr", {val: v, preload: [{rg: REG_CMERR, val: errPreload}]});
    }
    /* No write at all: the read must return the preload UNMASKED, bits outside CMERR_W1C included. */
    add(`CMERR-READ-UNMASKED`,
        [{kind: "rl", addr: cm(REG_CMERR), reg: 0}],
        "cmerr", {val: null, preload: [{rg: REG_CMERR, val: errPreload}]});

    /* ---------------- CMCSR ---------------- */
    let csrVals = [CMCSR_MASK >>> 0, (~CMCSR_MASK) >>> 0, 0xFFFFFFFF >>> 0, 0x00000000,
                   0x00002000, 0x0000007F, 0x12345678];
    for (let v of csrVals) {
        add(`CMCSR val=0x${hex(v)}`,
            [{kind: "wl", addr: cm(REG_CMCSR), val: v}, {kind: "rl", addr: cm(REG_CMCSR), reg: 0}],
            "cmcsr", {val: v, preload: [{rg: REG_CMCSR, val: 0xFFFFFFFF >>> 0}]});
    }

    /* ---------------- EXT (register 18) ----------------
       Read and write, and in BOTH cases the SSC bus-timeout register is part of the observation --
       that is what tells this machine check apart from the undecoded one at the very next longword,
       which the CONTROL case below performs in the same shape.  See the file header's matrix. */
    add(`EXT-READ`, [{kind: "rl", addr: cm(REG_CMEXT), reg: 0}], "ext", {op: "read"});
    add(`EXT-WRITE`, [{kind: "wl", addr: cm(REG_CMEXT), val: 0x11223344},
                      {kind: "rl", addr: cm(0), reg: 1}], "ext", {op: "write"});
    add(`EXT-CONTROL-UNDECODED`, [{kind: "rl", addr: cm(CMCTL_REGS), reg: 0}], "ext", {op: "control"});

    /* ---------------- MERGE ----------------
       A sub-longword write SHIFTS INTO POSITION AND CLOBBERS -- it does not merge.  Every case
       preloads the register with all of CMCNF_RW set, so a merge implementation and a clobber
       implementation give different answers at every lane.  Byte writes at all four lanes; word
       writes at the two genuinely aligned lanes only (see the file header). */
    let mergeTargets = [0, CMCTL_CONFIG_REGS - 1, REG_CMCSR];
    let bytePayloads = [0xFF, 0x00, 0x5A];
    for (let rg of mergeTargets) {
        let pre = (rg === REG_CMCSR) ? 0xFFFFFFFF >>> 0 : (CMCNF_VLD | CMCNF_BA | CMCNF_SIG) >>> 0;
        for (let lane = 0; lane < 4; lane++) {
            for (let v of bytePayloads) {
                add(`MERGE-BYTE rg=${rg} lane=${lane} val=0x${hex(v, 2)}`,
                    [{kind: "wb", addr: cm(rg, lane), val: v}, {kind: "rl", addr: cm(rg), reg: 0}],
                    "merge", {rg, lane, width: 1, preload: [{rg, val: pre}]});
            }
        }
        for (let lane of [0, 2]) {
            for (let v of [0xFFFF, 0x0000, 0x1234]) {
                add(`MERGE-WORD rg=${rg} lane=${lane} val=0x${hex(v, 4)}`,
                    [{kind: "ww", addr: cm(rg, lane), val: v}, {kind: "rl", addr: cm(rg), reg: 0}],
                    "merge", {rg, lane, width: 2, preload: [{rg, val: pre}]});
            }
        }
    }
    /* A BYTE write carrying the signature-request bit, at the lane that actually contains it.
       CMCNF_SRQ lives in byte lane 0, so this is the only lane at which a byte write can trigger a
       signature -- and at any other lane it must NOT. */
    for (let lane = 0; lane < 4; lane++) {
        let ops = [{kind: "wb", addr: cm(4, lane), val: CMCNF_SRQ}];
        for (let i = 0; i < 4; i++) ops.push({kind: "rl", addr: cm(4 + i), reg: i});
        add(`MERGE-SRQ-BYTE lane=${lane}`, ops, "merge", {rg: 4, lane, width: 1, srqLane: true});
    }

    /* ---------------- SUBREAD ----------------
       Byte and word READS extract from the whole longword cmctl_rd() returns, so every lane of a
       preloaded register is separately observable -- including the lanes the read masks touch. */
    for (let rg of [0, REG_CMERR, REG_CMCSR]) {
        let pre = (rg === REG_CMERR) ? 0xDEADBEEF >>> 0 : 0xFFFFFFFF >>> 0;
        for (let lane = 0; lane < 4; lane++) {
            add(`SUBREAD-BYTE rg=${rg} lane=${lane}`,
                [{kind: "rb", addr: cm(rg, lane), reg: 0}], "subread",
                {rg, lane, width: 1, preload: [{rg, val: pre}]});
        }
        for (let lane of [0, 2]) {
            add(`SUBREAD-WORD rg=${rg} lane=${lane}`,
                [{kind: "rw", addr: cm(rg, lane), reg: 0}], "subread",
                {rg, lane, width: 2, preload: [{rg, val: pre}]});
        }
    }

    /* ---------------- RANDOM ----------------
       Real workload and randomized phase both, per standing rule 1: the deterministic phases above
       are the shapes that were reasoned about, these are the ones that were not.  Register 18 is
       excluded because it machine-checks and would truncate the rest of a multi-op case; it has its
       own phase. */
    for (let i = 0; i < nRandom; i++) {
        let ops = [], nOps = 2 + Math.floor(rnd() * 4);
        for (let k = 0; k < nOps && ops.length < OBS_REGS + 3; k++) {
            let rg = Math.floor(rnd() * (CMCTL_REGS - 1));       // 0 .. 17
            let roll = rnd();
            if (roll < 0.45) {
                ops.push({kind: "wl", addr: cm(rg), val: (rnd() * 0x100000000) >>> 0});
            } else if (roll < 0.60) {
                ops.push({kind: "wb", addr: cm(rg, Math.floor(rnd() * 4)), val: Math.floor(rnd() * 256)});
            } else if (roll < 0.70) {
                ops.push({kind: "ww", addr: cm(rg, rnd() < 0.5 ? 0 : 2), val: Math.floor(rnd() * 65536)});
            } else {
                ops.push({kind: "rl", addr: cm(rg), reg: 0});
            }
        }
        /* Always finish by reading four registers, so the case's whole visible effect is observed
           rather than only its last op's. */
        let base = Math.floor(rnd() * (CMCTL_CONFIG_REGS - 3));
        for (let k = 0; k < OBS_REGS; k++) ops.push({kind: "rl", addr: cm(base + k), reg: k});
        add(`RANDOM #${i}`, ops, "random", {});
    }

    return {cases, meta};
}

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * grade(memBytes, cases, meta, simhResults, jsResults, failures)
 *
 * Every case is compared on ALL of its observation channels: the observed registers, the final PC
 * (a case whose PC is R_HANDLER machine-checked), the SSC bus-timeout register, and all nineteen
 * backing-array entries.  A case whose SIMH chunk never appeared is reported BY NAME and counted as
 * a failure -- never skipped (standing rule 6).
 */
function grade(memBytes, cases, meta, simhResults, jsResults, failures)
{
    let compared = 0;
    for (let i = 0; i < cases.length; i++) {
        let s = simhResults[i], j = jsResults[i], name = `[${memBytes >>> 20}MB] ${cases[i].name}`;
        if (!s) {
            failures.push(`${name}: SIMH produced no readable result for this case (chunk missing or short)`);
            continue;
        }
        if (j.stop) {
            failures.push(`${name}: this machine raised an unexpected stop: ${j.stop}`);
            continue;
        }
        compared++;
        for (let k = 0; k < OBS_REGS; k++) {
            if (j.regs[k] !== s.regs[k]) {
                failures.push(`${name}: R${k} js=${hex(j.regs[k])} simh=${hex(s.regs[k])}`);
            }
        }
        if (j.pc !== s.pc) failures.push(`${name}: PC js=${hex(j.pc)} simh=${hex(s.pc)}`);
        if (j.bto !== s.bto) failures.push(`${name}: BTO js=${hex(j.bto)} simh=${hex(s.bto)}`);
        for (let r = 0; r < CMCTL_REGS; r++) {
            if (j.reg[r] !== s.reg[r]) {
                failures.push(`${name}: cmctl_reg[${r}] js=${hex(j.reg[r])} simh=${hex(s.reg[r])}`);
            }
        }
    }
    return compared;
}

/**
 * classifySpan(results, cases, meta)
 *
 * Turns PHASE SPAN's raw results into the three-way classification the file header describes, for
 * ONE engine.  A probe that did not machine-check is `ok`; one that did is `mchk` or `mchk+bto`
 * according to whether the fall-through set the SSC bus-timeout bits.
 *
 * @returns {Map<number,string>} register index -> "ok" | "mchk" | "mchk+bto" | "missing"
 */
function classifySpan(results, meta)
{
    let out = new Map();
    for (let e of meta.span) {
        let r = results[e.idx];
        if (!r) { out.set(e.rg, "missing"); continue; }
        if (r.pc !== R_HANDLER) out.set(e.rg, "ok");
        else out.set(e.rg, r.bto ? "mchk+bto" : "mchk");
    }
    return out;
}

/**
 * spanFrom(cls)
 *
 * The DECODED extent implied by a classification: every index that is not "mchk+bto" is inside the
 * range vax_sysdev.c's regtable[] covers.  Returns null (with `why`) when the classification is not
 * a single contiguous run starting at 0, because a hole would make "the span" meaningless rather
 * than merely wrong.
 *
 * @param {Map<number,string>} cls
 * @returns {{span: ?number, why: ?string}}
 */
function spanFrom(cls)
{
    let inside = [...cls.entries()].filter(([, v]) => v === "ok" || v === "mchk").map(([k]) => k).sort((a, b) => a - b);
    if (!inside.length) return {span: null, why: "no probed address was decoded at all"};
    if (inside[0] !== 0) return {span: null, why: `the decoded run starts at register ${inside[0]}, not 0`};
    for (let i = 1; i < inside.length; i++) {
        if (inside[i] !== inside[i - 1] + 1) {
            return {span: null, why: `the decoded run has a hole: register ${inside[i - 1]} is decoded and ${inside[i - 1] + 1} is not`};
        }
    }
    return {span: inside.length, why: null};
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors.  Every one FAILS the run and none scales with the case count (rule 4).       *
 * ------------------------------------------------------------------------------------------- */

/**
 * coverage(memBytes, cases, meta, simhResults, failures)
 *
 * Counted from the cases that ACTUALLY REACHED COMPARISON, never from the case list: a phase whose
 * results never arrived must not be able to certify its own coverage.
 */
function coverage(memBytes, cases, meta, simhResults, failures, acc)
{
    let tag = `[${memBytes >>> 20}MB] coverage`;
    let reached = (idx) => !!simhResults[idx];

    /* Every register index READ and WRITTEN through a real instruction. */
    let read = new Set(), written = new Set();
    for (let i = 0; i < cases.length; i++) {
        if (!reached(i)) continue;
        for (let op of cases[i].ops) {
            let rg = (((op.addr >>> 0) - CMCTL_BASE) >>> 2);
            if (rg >= CMCTL_REGS) continue;
            if (op.kind[0] === "r") read.add(rg); else written.add(rg);
        }
    }
    for (let rg = 0; rg < CMCTL_REGS; rg++) {
        if (!read.has(rg)) failures.push(`${tag}: register ${rg} was never READ by a graded case`);
        if (!written.has(rg)) failures.push(`${tag}: register ${rg} was never WRITTEN by a graded case`);
    }

    /* All four byte-write lanes, both aligned word-write lanes, all four byte-read lanes, both
       aligned word-read lanes. */
    let lanes = {wb: new Set(), ww: new Set(), rb: new Set(), rw: new Set()};
    for (let i = 0; i < cases.length; i++) {
        if (!reached(i)) continue;
        for (let op of cases[i].ops) {
            if (lanes[op.kind]) lanes[op.kind].add((op.addr >>> 0) & 3);
        }
    }
    for (let [kind, want] of [["wb", [0, 1, 2, 3]], ["ww", [0, 2]], ["rb", [0, 1, 2, 3]], ["rw", [0, 2]]]) {
        for (let l of want) {
            if (!lanes[kind].has(l)) failures.push(`${tag}: no graded case performed a "${kind}" at byte lane ${l}`);
        }
    }

    /* Every signature-request group.  The two BANK OUTCOMES -- populated and not -- are recorded
       into the cross-configuration accumulator instead of being required here, because they are not
       both reachable in one configuration and a floor that cannot be met is a floor that gets
       lowered: at 128MB every one of the sixteen banks starts below MEMSIZE, so nothing can be left
       unsigned, and at 16MB only the first four can be signed.  The RUN as a whole must see both.
       The outcome is read from the ORACLE's own answer, never predicted here. */
    let groups = new Set();
    for (let e of meta.srq) {
        if (!reached(e.idx)) continue;
        groups.add(e.group);
        let s = simhResults[e.idx];
        for (let i = 0; i < 4; i++) {
            if ((s.reg[e.group + i] & CMCNF_SIG) === (MEM_SIG & CMCNF_SIG)) acc.bankSigned = true;
            if ((s.reg[e.group + i] & CMCNF_SIG) === 0) acc.bankCleared = true;
        }
    }
    for (let g = 0; g < CMCTL_CONFIG_REGS; g += 4) {
        if (!groups.has(g)) failures.push(`${tag}: signature-request group ${g}-${g + 3} was never exercised`);
    }

    /* CMERR: at least one graded case must observe a bit being CLEARED and a bit being RETAINED. */
    let errCleared = false, errRetained = false;
    for (let e of meta.cmerr) {
        if (!reached(e.idx) || e.val === null) continue;
        let before = e.preload[0].val >>> 0, after = simhResults[e.idx].reg[REG_CMERR] >>> 0;
        if ((before & ~after) !== 0) errCleared = true;
        if ((before & after) !== 0) errRetained = true;
    }
    if (!errCleared) failures.push(`${tag}: no graded CMERR case ever cleared a bit`);
    if (!errRetained) failures.push(`${tag}: no graded CMERR case ever retained a bit`);

    /* CMCSR: at least one graded case must observe an out-of-mask bit being DROPPED, and one must
       observe an in-mask bit being KEPT. */
    let csrDropped = false, csrKept = false;
    for (let e of meta.cmcsr) {
        if (!reached(e.idx)) continue;
        let wrote = e.val >>> 0, after = simhResults[e.idx].reg[REG_CMCSR] >>> 0;
        if ((wrote & ~after) !== 0) csrDropped = true;
        if ((wrote & after) !== 0) csrKept = true;
    }
    if (!csrDropped) failures.push(`${tag}: no graded CMCSR case ever dropped an out-of-mask bit`);
    if (!csrKept) failures.push(`${tag}: no graded CMCSR case ever kept an in-mask bit`);

    /* The extension register: its read and its write, plus the undecoded control, all graded. */
    for (let want of ["read", "write", "control"]) {
        let e = meta.ext.find((x) => x.op === want);
        if (!e || !reached(e.idx)) failures.push(`${tag}: the register-${REG_CMEXT} "${want}" case did not reach comparison`);
    }
    /* And the control case must actually have DISTINGUISHED the two machine checks on the oracle,
       or the EXT phase is grading a difference that is not there. */
    let ctl = meta.ext.find((x) => x.op === "control");
    let rd = meta.ext.find((x) => x.op === "read");
    if (ctl && rd && reached(ctl.idx) && reached(rd.idx)) {
        let c = simhResults[ctl.idx], r = simhResults[rd.idx];
        if (!(c.pc === R_HANDLER && c.bto !== 0)) {
            failures.push(`${tag}: the undecoded control address did not machine-check WITH the ` +
                `bus-timeout bits on the oracle (PC=${hex(c.pc)} BTO=${hex(c.bto)}) -- the EXT ` +
                `phase's whole distinction is unobservable and it is grading nothing`);
        }
        if (memBytes <= VAX.PHYSMEM.MAXMEMSIZE) {
            if (r.pc === R_HANDLER && r.bto === 0) acc.extReadMchk = true;
            else {
                failures.push(`${tag}: the register-${REG_CMEXT} read did not machine-check WITHOUT the ` +
                    `bus-timeout bits on the oracle (PC=${hex(r.pc)} BTO=${hex(r.bto)})`);
            }
        } else {
            if (r.pc !== R_HANDLER && (r.regs[0] >>> 0) === (memBytes >>> 0)) acc.extReadValue = true;
            else {
                failures.push(`${tag}: the register-${REG_CMEXT} read did not return MEMSIZE on the ` +
                    `oracle (PC=${hex(r.pc)} R0=${hex(r.regs[0])})`);
            }
        }
    }

    /* The observation plumbing itself: at least one graded case must use the LAST observable
       register, or OBS_REGS is wider than anything proven to be parsed. */
    let usedLast = meta.srq.some((e) => reached(e.idx)) || meta.random.some((e) => reached(e.idx));
    if (!usedLast) failures.push(`${tag}: no graded case observed R${OBS_REGS - 1}`);
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11)         *
 * ------------------------------------------------------------------------------------------- */

/**
 * Each entry returns a restore function.  Every one composes over, or edits data consulted by, the
 * SHIPPED cmctl.js -- so a mutation applied to code that is ALREADY broken still changes behaviour,
 * which is the failure mode standing rule 11 was earned by.
 *
 * NOT PRESENT.  pcjsvax-622's done-condition list also names three mutations that belong to OTHER
 * modules, and the status of each is stated rather than left to be inferred:
 *   - "CDG parity is ignored" is COVERED, in tests/cdgdiff.js, by CACR-SEED-INVERTED,
 *     CACR-BITS-SWAPPED and CACR-DRO-MASK-WIDENED.  That is cdg.js's and ka655.js's
 *     setCdgDiagParity(), pcjsvax-0b7's subject, and a second copy here would be a drifting
 *     duplicate of another file's scope (standing rule 7).
 *   - "CACR_CPE is not write-1-to-clear" and "the BDR halt-enable bit is inverted" are NOT covered
 *     by any mutation in this tree today -- not here, and not in cdgdiff.js, which mutates the
 *     diagnostic-parity lanes but not ka_wr()'s W1C or ka_bdr.  They are ka655.js's registers, and
 *     closing them belongs to ka655.js's own differential.  This sentence exists so that gap is
 *     NAMED rather than assumed shut by the presence of a file called cmctldiff.
 */
const MUTATIONS = {
    "ext-register-answers-unconditionally": () => {
        let orig = CMCTLVAX.prototype.readReg;
        CMCTLVAX.prototype.readReg = function(rg) {
            let v = orig.call(this, rg);
            return v === REG_MCHK ? (this.memSize | 0) : v;
        };
        return () => { CMCTLVAX.prototype.readReg = orig; };
    },
    "ext-register-faults-as-undecoded-and-sets-bto": () => {
        let origR = CMCTLVAX.prototype.readReg, origW = CMCTLVAX.prototype.writeReg;
        CMCTLVAX.prototype.readReg = function(rg) {
            let v = origR.call(this, rg);
            return v === REG_MCHK ? null : v;
        };
        CMCTLVAX.prototype.writeReg = function(...a) {
            let v = origW.apply(this, a);
            return v === REG_MCHK ? false : v;
        };
        return () => { CMCTLVAX.prototype.readReg = origR; CMCTLVAX.prototype.writeReg = origW; };
    },
    "ext-register-write-is-accepted": () => {
        let orig = CMCTLVAX.prototype.writeReg;
        CMCTLVAX.prototype.writeReg = function(...a) {
            let v = orig.apply(this, a);
            return v === REG_MCHK ? true : v;
        };
        return () => { CMCTLVAX.prototype.writeReg = orig; };
    },
    "cmerr-is-not-write-1-to-clear": () => {
        let prev = CMCTLVAX.CMERR_W1C;
        CMCTLVAX.CMERR_W1C = 0;                 // nothing is clearable
        return () => { CMCTLVAX.CMERR_W1C = prev; };
    },
    "cmerr-w1c-mask-widened": () => {
        let prev = CMCTLVAX.CMERR_W1C;
        CMCTLVAX.CMERR_W1C = ~0;                // everything is clearable
        return () => { CMCTLVAX.CMERR_W1C = prev; };
    },
    "cmcsr-write-mask-ignored": () => {
        let prev = CMCTLVAX.CMCSR_MASK;
        CMCTLVAX.CMCSR_MASK = ~0;
        return () => { CMCTLVAX.CMCSR_MASK = prev; };
    },
    "cmcsr-read-unmasked": () => {
        let orig = CMCTLVAX.prototype.readReg;
        CMCTLVAX.prototype.readReg = function(rg) {
            let v = orig.call(this, rg);
            return (rg === REG_CMCSR && typeof v === "number") ? (this.reg[rg] | 0) : v;
        };
        return () => { CMCTLVAX.prototype.readReg = orig; };
    },
    "config-read-unmasked": () => {
        let orig = CMCTLVAX.prototype.readReg;
        CMCTLVAX.prototype.readReg = function(rg) {
            let v = orig.call(this, rg);
            return (rg < CMCTL_CONFIG_REGS && typeof v === "number") ? (this.reg[rg] | 0) : v;
        };
        return () => { CMCTLVAX.prototype.readReg = orig; };
    },
    "signature-request-ignored": () => {
        let prev = CMCTLVAX.CMCNF_SRQ;
        CMCTLVAX.CMCNF_SRQ = 0;                 // the `val & CMCNF_SRQ` gate never fires
        return () => { CMCTLVAX.CMCNF_SRQ = prev; };
    },
    "signature-request-signs-only-the-written-register": () => {
        let prevMask = CMCTLVAX.CMCNF_SRQ_GROUP_MASK, prevN = CMCTLVAX.CMCNF_SRQ_GROUP;
        CMCTLVAX.CMCNF_SRQ_GROUP_MASK = ~0;
        CMCTLVAX.CMCNF_SRQ_GROUP = 1;
        return () => { CMCTLVAX.CMCNF_SRQ_GROUP_MASK = prevMask; CMCTLVAX.CMCNF_SRQ_GROUP = prevN; };
    },
    "signature-request-group-not-rounded-down": () => {
        let prev = CMCTLVAX.CMCNF_SRQ_GROUP_MASK;
        CMCTLVAX.CMCNF_SRQ_GROUP_MASK = ~0;     // the group starts AT the written register
        return () => { CMCTLVAX.CMCNF_SRQ_GROUP_MASK = prev; };
    },
    "every-bank-signs-as-populated": () => {
        let orig = CMCTLVAX.prototype.isMemBank;
        CMCTLVAX.prototype.isMemBank = function(i) { orig.call(this, i); return true; };
        return () => { CMCTLVAX.prototype.isMemBank = orig; };
    },
    "memory-signature-value-wrong": () => {
        let prev = CMCTLVAX.MEM_SIG;
        CMCTLVAX.MEM_SIG = CMCNF_SIG >>> 0;     // 0x1F instead of 0x17
        return () => { CMCTLVAX.MEM_SIG = prev; };
    },
    "sub-longword-write-merges-instead-of-clobbering": () => {
        let orig = CMCTLVAX.prototype.writeReg;
        CMCTLVAX.prototype.writeReg = function(rg, val, lnt, addr) {
            if (lnt < 4 && rg < CMCTL_REGS) {
                let before = this.reg[rg] | 0;
                let r = orig.call(this, rg, val, lnt, addr);
                let m = ((lnt === 2 ? 0xFFFF : 0xFF) << (((addr >>> 0) & 3) << 3)) | 0;
                this.reg[rg] = ((this.reg[rg] & m) | (before & ~m)) | 0;
                return r;
            }
            return orig.call(this, rg, val, lnt, addr);
        };
        return () => { CMCTLVAX.prototype.writeReg = orig; };
    },
    "sub-longword-write-not-shifted-into-position": () => {
        let prev = CMCTLVAX.L_LONG;
        CMCTLVAX.L_LONG = 0;                    // `lnt < L_LONG` never fires, so `val << sc` is skipped
        return () => { CMCTLVAX.L_LONG = prev; };
    },
    "cmctl-span-one-register-short": () => ({spanDelta: -4})
};

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                        *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

/**
 * runPass(simh, opts, mutationOpts)
 *
 * One complete grading pass over BOTH memory configurations.  Returns the failure list; the caller
 * decides whether an empty one is the right answer (the normal run) or the wrong one (--selfcheck).
 */
function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [];
    /* Floors that are only meetable ACROSS the two memory configurations -- see coverage(). */
    let acc = {bankSigned: false, bankCleared: false, extReadValue: false, extReadMchk: false};
    for (let memMB of [MEM_SMALL_MB, MEM_BIG_MB]) {
        let memBytes = memMB * 1024 * 1024;
        let {cases, meta} = buildCases(memBytes, opts.nRandom, opts.seed);
        let simhResults = runCasesSimh(simh, opts, memMB, cases);
        let jsResults = cases.map((k) => runCaseJS(memBytes, k, mutationOpts));

        let compared = grade(memBytes, cases, meta, simhResults, jsResults, failures);

        /* PHASE SPAN -- three derivations, compared. */
        let clsS = classifySpan(simhResults, meta), clsJ = classifySpan(jsResults, meta);
        let sS = spanFrom(clsS), sJ = spanFrom(clsJ);
        for (let [who, s] of [["the oracle", sS], ["this machine", sJ]]) {
            if (s.span === null) failures.push(`[${memMB}MB] SPAN: ${who}'s classification is not a contiguous range: ${s.why}`);
        }
        if (sS.span !== null && sS.span !== CMCTL_REGS) {
            failures.push(`[${memMB}MB] SPAN: the oracle decodes ${sS.span} CMCTL register(s); ` +
                `cmctl.js computes ${CMCTL_REGS} from MAXMEMSIZE/MEM_BANK + ${CMCTL_SPECIAL.length} ` +
                `(${CMCTL_SPECIAL.join("/")})`);
        }
        if (sJ.span !== null && sJ.span !== sS.span) {
            failures.push(`[${memMB}MB] SPAN: this machine decodes ${sJ.span} register(s), the oracle ${sS.span}`);
        }
        for (let rg of [...clsS.keys()].sort((a, b) => a - b)) {
            if (clsS.get(rg) !== clsJ.get(rg)) {
                failures.push(`[${memMB}MB] SPAN: register ${rg} (0x${hex(cm(rg))}) classifies as ` +
                    `"${clsJ.get(rg)}" here and "${clsS.get(rg)}" on the oracle`);
            }
        }

        coverage(memBytes, cases, meta, simhResults, failures, acc);

        report.push(`  ${String(memMB) + "MB"}: ${compared}/${cases.length} case(s) compared, ` +
            `span ${sS.span === null ? "?" : sS.span} register(s) measured on the oracle, ` +
            `${clsS.size} probed address(es) classified identically on both engines` +
            (sS.span === sJ.span && sS.span === CMCTL_REGS ? "" : "  <-- DISAGREEMENT"));
    }

    /* Cross-configuration floors.  Fixed, and they do not scale with anything (standing rule 4). */
    if (!acc.bankSigned) failures.push(`coverage: no graded signature request in EITHER memory ` +
        `configuration ever SIGNED a bank as populated`);
    if (!acc.bankCleared) failures.push(`coverage: no graded signature request in EITHER memory ` +
        `configuration ever left a bank's signature CLEAR`);
    if (!acc.extReadMchk) failures.push(`coverage: register ${REG_CMEXT}'s read was never graded on ` +
        `its MACHINE-CHECK branch (MEMSIZE <= MAXMEMSIZE)`);
    if (!acc.extReadValue) failures.push(`coverage: register ${REG_CMEXT}'s read was never graded on ` +
        `its ANSWER-WITH-MEMSIZE branch (MEMSIZE > MAXMEMSIZE) -- the ${MEM_BIG_MB}MB configuration ` +
        `did not run or did not reach it`);
    return {failures, report};
}

/**
 * selfcheck(simh, opts)
 *
 * Every mutation must be CAUGHT.  The run is over the SAME two machines the graded pass uses, and
 * the mutation is removed in a `finally` so one surviving mutation cannot poison the next.
 */
function selfcheck(simh, opts)
{
    let survived = [];
    for (let name of Object.keys(MUTATIONS)) {
        let apply = MUTATIONS[name]();
        let restore = typeof apply === "function" ? apply : () => {};
        let mutationOpts = typeof apply === "function" ? {} : apply;
        let failures;
        try {
            failures = runPass(simh, opts, mutationOpts).failures;
        } finally {
            restore();
        }
        if (!failures.length) survived.push(name);
        console.log(`  ${failures.length ? "CAUGHT " : "SURVIVED"}  ${name}` +
            (failures.length ? `  (${failures.length} failure(s), first: ${failures[0]})` : ""));
    }
    return survived;
}

function main()
{
    let simh = findSimh(getArg("--simh", null));
    let nRandom = +getArg("--random-cases", RANDOM_CASES_DEFAULT);
    let seed = +getArg("--seed", 20260727);
    let fSelfcheck = process.argv.includes("--selfcheck");

    if (nRandom < RANDOM_CASES_FLOOR) {
        console.error(`cmctldiff: --random-cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "cmctldiff-"));
    let opts = {scratch, nRandom, seed};
    let code = 0;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized cases per configuration: ${nRandom}`);
        console.log(`\nCMCTL span, COMPUTED by cmctl.js: ${CMCTL_CONFIG_REGS} configuration register(s) ` +
            `(MAXMEMSIZE 0x${hex(VAX.PHYSMEM.MAXMEMSIZE)} / MEM_BANK 0x${hex(MEM_BANK)}) + ` +
            `${CMCTL_SPECIAL.length} named (${CMCTL_SPECIAL.join("/")}) ` +
            `= ${CMCTL_REGS} register(s), 0x${hex(CMCTL_LENGTH, 4)} byte(s) at 0x${hex(CMCTL_BASE)}`);

        let {failures, report} = runPass(simh, opts);
        console.log(`\nPHASE GRADE / SPAN`);
        for (let line of report) console.log(line);

        let peakMB = PEAK_HEAP / (1024 * 1024);
        console.log(`\npeak JS heap+external: ${peakMB.toFixed(1)} MB (absolute ceiling ${MAX_HEAP_BYTES / (1024 * 1024)} MB)`);
        if (PEAK_HEAP > MAX_HEAP_BYTES) {
            failures.push(`peak heap+external ${peakMB.toFixed(1)} MB exceeds the absolute ceiling ` +
                `${MAX_HEAP_BYTES / (1024 * 1024)} MB`);
        }

        if (failures.length) {
            console.error(`\nFAIL -- ${failures.length} difference(s) from the oracle (seed ${seed}):`);
            for (let f of failures.slice(0, 60)) console.error(`  ${f}`);
            if (failures.length > 60) console.error(`  ... and ${failures.length - 60} more`);
            code = 1;
        } else {
            console.log(`\nMATCH -- every graded CMCTL access agrees with the oracle in value, in ` +
                `machine-check behaviour, in SSC bus-timeout residue, and in all ${CMCTL_REGS} ` +
                `backing-array entries.`);
        }

        if (fSelfcheck && !code) {
            console.log(`\nPHASE M -- mutations (${Object.keys(MUTATIONS).length})`);
            let survived = selfcheck(simh, opts);
            if (survived.length) {
                console.error(`\nFAIL -- ${survived.length} mutation(s) SURVIVED: ${survived.join(", ")}`);
                code = 1;
            } else {
                console.log(`\nall ${Object.keys(MUTATIONS).length} mutation(s) CAUGHT`);
            }
        }
        if (!code) console.log("\nOK");
    } finally {
        /* Every exit path, including a throw: HANDOFF.md's disk note (pcjsvax-bd1 exists because a
           differential leaked 10.4GB of scratch). */
        try { fs.rmSync(scratch, {recursive: true, force: true}); } catch (e) { /* best effort */ }
    }
    process.exit(code);
}

main();
