/**
 * @fileoverview Shared harness for the RQDX3/MSCP differentials (mscpinitdiff, mscpringdiff)
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * The parts of tests/mscpinitdiff.js (pcjsvax-c2c) that tests/mscpringdiff.js (pcjsvax-0b4) needs
 * VERBATIM: the SIMH plumbing, the fixed physical layout both differentials deposit into, the small
 * VAX assembler, the ONE machine both build and reuse, the observation vector transcribed from
 * SIMH's own rq_reg[], and the `SHOW RQ RINGS/FREEQ/RESPQ` renderers.
 *
 * It exists because the alternative was a second copy.  Two copies of an observation vector that is
 * a TRANSCRIPTION OF THE ORACLE'S OWN REGISTER TABLE is the shape of defect HANDOFF.md standing
 * rule 7 records -- two modules' headers disagreeing about who owned six opcodes, with all six
 * falling through the gap.  Nothing here decides anything: every function is either plumbing or a
 * rendering of state that is graded, as text, against the live simulator.
 *
 * WHAT IS DELIBERATELY *NOT* HERE
 * --------------------------------
 * Case construction, coverage floors, exclusion fences and mutations.  Those are each
 * differential's own argument about what it proves, and sharing them would let one test's coverage
 * certify another's.
 *
 * ONE THING MOVED HERE FROM mscpringdiff.js WHEN pcjsvax-f52 ARRIVED: geometry()/qbusPagesFor(),
 * the arithmetic that says where a comm region's two rings and packet buffers live.  It is on the
 * plumbing side of that line -- it decides no case and asserts nothing -- and it contains the one
 * fact in this whole device that is easiest to get backwards (*** THE RESPONSE RING IS AT `comm`
 * AND THE COMMAND RING ABOVE IT ***).  Two copies of that would be two chances to get it wrong, and
 * the second copy would be in the test least able to notice.
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
import CQBICVAX, { CQMAPVAX, CQBIC_BASE, CQMAP_BASE, CQMAPSIZE, CQMAP_VLD } from "../modules/v2/cqbic.js";
import RQVAX, {
    RQ_BASE, IOLN_RQ, RQDX3_CTYPE, CST_UP,
    SA_S1H_V_CQ, SA_S1H_M_CQ, SA_S1H_V_RQ, SA_S1H_M_RQ,
    SA_S2H_CLO, SA_S2H_PI, SA_S3H_PP, SA_S3H_CHI, SA_S4H_GO,
    SA_COMM_QQ, SA_COMM_CI, SA_COMM_MAX,
    RQ_NPKTS, RQ_PKT_SIZE_W, UQ_HCTC_V_CR, UQ_HCTC_M_CR,
    UQ_HCTC_V_TYP, UQ_HCTC_M_TYP, UQ_HCTC_V_CID, UQ_HCTC_M_CID
} from "../modules/v2/rq.js";
import { ods2VolumeBytesFrom } from "../modules/v2/ods2.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/* ------------------------------------------------------------------------------------------- *
 * The fixed physical layout, MAPPING OFF -- the convention dbldiff.js / cmctldiff.js /           *
 * qdmadiff.js use.  TWO handler pages, so a machine check and a deferred memory error are told   *
 * apart by PC alone.                                                                            *
 * ------------------------------------------------------------------------------------------- */

/** 16MB, the SIMH microvax3900 default and every other differential's size. */
export const MEMSIZE = 0x01000000;
export const MEM_MB  = MEMSIZE / (1024 * 1024);
export const PAGE    = 512;

export const R_SCBB      = 0x00100000;
export const R_MCHK_HDLR = 0x00102000;
export const R_MERR_HDLR = 0x00103000;
export const R_CODE      = 0x00104000;
/** A page of PHYSICAL scratch the host program stores its own observations into, dumped WHOLE and
    compared longword by longword.  It is what lets a case record more results than there are
    registers -- and it is why a host program can run a dozen commands and still be graded on every
    one of them. */
export const R_RESULT    = 0x00108000;
export const R_KSP       = 0x00110000;
/** A machine check is a SEVERE exception: intexc() reloads SP from the INTERRUPT stack regardless
    of mode, so IS must be set or the frame push faults inside `in_ie` and SIMH stops hard. */
export const R_IS        = 0x00118000;
/** The CQBIC scatter-gather map's backing store: 8192 longword entries.  Page-aligned. */
export const MAP_MBR     = 0x00200000;
export const MAP_HI      = (MAP_MBR + CQMAPSIZE) >>> 0;
/** The physical pages Qbus pages are scattered across.  Deliberately far from everything else. */
export const DATA_BASE   = 0x00300000;
export const DATA_NPAGE  = 64;
export const DATA_HI     = (DATA_BASE + DATA_NPAGE * PAGE) >>> 0;
/** Low memory that an UNPROGRAMMED map (MBR 0) would read entries out of -- zeroed so the fatal
    cases fail for the reason they are supposed to (no valid bit) rather than by accident. */
export const LOWMAP_HI   = 0x00001000;

export const HDLR_NOPS = 16;
/** R0..R14 -- R15 is PC, observed separately. */
export const OBS_REGS  = 15;

export const RQ_IP = RQ_BASE;
export const RQ_SA = (RQ_BASE + 2) >>> 0;

export { CQBIC_BASE, CQMAP_BASE, CQMAPSIZE, CQMAP_VLD, SCB, VAX };

/* ------------------------------------------------------------------------------------------- *
 * Plumbing                                                                                      *
 * ------------------------------------------------------------------------------------------- */

export function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

export function vaxRepo()
{
    if (process.env['PCJS_VAX_REPO']) return process.env['PCJS_VAX_REPO'];
    return path.resolve(__dirname, "../../../../../pcjs-vax");
}

export function findSimhBin(pathArg)
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
    throw new Error("this differential needs a REAL SIMH microvax3900; it has no fixture fallback.\n" +
        "Build one with machines/dec/vax/tests/simh/build.sh and pass --simh PATH.  Tried:\n  " +
        candidates.join("\n  "));
}

/**
 * runSimh(bin, script, iniPath)
 *
 * `run` IS FORBIDDEN IN EVERY DO-FILE EITHER DIFFERENTIAL WRITES, and the check is here rather than
 * in a comment.  SCP's `run` RESETS ALL DEVICES before starting, which silently destroyed three
 * stages of a handshake during pcjsvax-6a5's decomposition -- the controller went back to CST_S1 and
 * the SA read-backs looked like a plain implementation bug.  `step` and `go` do not reset.  This is
 * an environment gotcha of the same family as HANDOFF.md 4's exported-`E` trap, and asserting it
 * costs one regexp.
 */
export function runSimh(bin, script, iniPath, timeoutMs = 10 * 60 * 1000)
{
    if (/^\s*run\b/mi.test(script)) {
        throw new Error("mscp harness: a do-file line begins with `run`, which RESETS ALL DEVICES " +
            "and destroys the handshake under test.  Use `step` or `go`.");
    }
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: timeoutMs});
}

/** The same mulberry32 every VAX differential in this tree uses. */
export function mulberry32(a)
{
    return function() {
        a |= 0; a = (a + 0x6D2B79F5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

/* ------------------------------------------------------------------------------------------- *
 * Heap accounting.  HANDOFF.md standing rule 14: an ABSOLUTE bound, asserted as a failure, that   *
 * does NOT scale with case count.  Sampled here so every construction and every case is covered   *
 * whichever differential is running.                                                             *
 * ------------------------------------------------------------------------------------------- */

let PEAK_HEAP = 0;

export function sampleHeap()
{
    let mu = process.memoryUsage();
    let used = mu.heapUsed + mu.external;
    if (used > PEAK_HEAP) PEAK_HEAP = used;
    return used;
}

export function peakHeap() { return PEAK_HEAP; }

/* ------------------------------------------------------------------------------------------- *
 * A very small assembler.  Opcode NUMBERS come from drom.js's OPCODES table, never transcribed.  *
 * ------------------------------------------------------------------------------------------- */

export function opcodeOf(name)
{
    let opc = OPCODES.indexOf(name);
    if (opc < 0 || opc > 0xFF) throw new Error(`mscp harness: opcode "${name}" not found or not single-byte`);
    return opc;
}

export const OPC = {};
for (let n of ["MOVL", "MOVW", "MOVB", "MOVZWL", "MOVZBL", "CMPL", "BNEQ", "BRB", "INCL", "CLRL",
               "SOBGTR", "NOP", "HALT"]) OPC[n] = opcodeOf(n);

export function lw(a) { a = a >>> 0; return [a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF]; }

/**
 * @class Asm
 *
 * Emits bytes and counts INSTRUCTIONS.  The instruction count is not decoration: it is what the
 * poll loops' displacements are computed from, and a miscount would silently change the schedule
 * these differentials exist to grade.
 */
export class Asm {
    constructor() { this.b = []; }
    get len() { return this.b.length; }
    emit(...bytes) { this.b.push(...bytes); return this; }

    /** MOVL/MOVW/MOVB I^#val, @#addr */
    movImmAbs(size, val, addr) {
        if (size === 4) return this.emit(OPC.MOVL, 0x8F, ...lw(val), 0x9F, ...lw(addr));
        if (size === 2) return this.emit(OPC.MOVW, 0x8F, val & 0xFF, (val >>> 8) & 0xFF, 0x9F, ...lw(addr));
        return this.emit(OPC.MOVB, 0x8F, val & 0xFF, 0x9F, ...lw(addr));
    }
    /** MOVL/MOVZWL/MOVZBL @#addr, Rn -- all three leave a full, zero-extended longword in Rn, which
        is what keeps a case's read-back from mixing with the previous case's leftovers. */
    movAbsReg(size, addr, rn) {
        let opc = (size === 4) ? OPC.MOVL : (size === 2) ? OPC.MOVZWL : OPC.MOVZBL;
        return this.emit(opc, 0x9F, ...lw(addr), 0x50 | (rn & 0xF));
    }
    /** MOVL Rn, @#addr -- the store half, which is what makes the RESULT page usable. */
    movRegAbs(rn, addr) { return this.emit(OPC.MOVL, 0x50 | (rn & 0xF), 0x9F, ...lw(addr)); }
    clrl(rn) { return this.emit(OPC.CLRL, 0x50 | (rn & 0xF)); }
    halt() { return this.emit(OPC.HALT); }

    /**
     * poll(addr, curReg, prevReg, cntReg)
     *
     * The host's own busy-wait, and the instrument that makes SIMH's event schedule observable IN
     * BAND.  FIVE instructions per iteration:
     *
     *      loop: MOVZWL @#addr, Rcur ; CMPL Rcur, Rprev ; BNEQ out ; INCL Rcnt ; BRB loop
     *
     * `Rcnt` ends up holding the number of iterations that did NOT see the change, which for a
     * `sim_activate` of `delay` instructions is the smallest i with 1 + 5i >= delay.  Measured on
     * the oracle: 90 for ITIME 450, 2 for I4TIME 10.
     *
     * UNBOUNDED, deliberately: it polls a value the controller is scheduled to change, and a case
     * where it never does is a case whose step budget runs out and is reported BY NAME.  Where the
     * controller might legitimately never answer, use awaitL() instead.
     */
    poll(addr, curReg, prevReg, cntReg) {
        this.clrl(cntReg);
        let top = this.len;
        this.movAbsReg(2, addr, curReg);                            // 7 bytes
        this.emit(OPC.CMPL, 0x50 | curReg, 0x50 | prevReg);         // 3
        this.emit(OPC.BNEQ, 4);                                     // 2 -- skip INCL(2) + BRB(2)
        this.emit(OPC.INCL, 0x50 | cntReg);                         // 2
        let disp = top - (this.len + 2);
        this.emit(OPC.BRB, disp & 0xFF);                            // 2
        return this;
    }

    /**
     * delay(n, reg)
     *
     * `MOVL I^#n, Rreg` then a one-instruction `SOBGTR Rreg, .` loop: exactly n + 1 instructions,
     * used where there is nothing to poll ON.  The purge/poll path needs it -- writing 0 to SA in
     * CST_S3_PPA advances the state WITHOUT changing SA, so no value ever becomes visible and a
     * polling loop would spin forever.
     */
    delay(n, reg) {
        this.emit(OPC.MOVL, 0x8F, ...lw(n), 0x50 | reg);
        let top = this.len;
        this.emit(OPC.SOBGTR, 0x50 | reg, (top - (this.len + 3)) & 0xFF);
        return this;
    }

    /**
     * awaitUnbounded(addr, resultAddr, regs)
     *
     * *** A BOUNDED WAIT ON A RESPONSE THE CONTROLLER MUST SEND IS A FLAKE GENERATOR, AND THIS IS
     * THE REPLACEMENT. ***
     *
     *      MOVL @#addr, Rprev ; CLRL Rcnt
     * top: MOVL @#addr, Rcur ; CMPL Rcur, Rprev ; BNEQ out ; INCL Rcnt ; BRB top
     * out: MOVL Rcnt, @#result ; MOVL Rcur, @#result+4
     *
     * FIVE instructions per iteration and NO BUDGET: the host cannot proceed until the value it is
     * watching has actually changed.  That makes the OBSERVATION POINT deterministic -- every
     * register, packet word, ring slot and page compared afterwards is sampled with the command
     * provably finished on whichever engine is running -- which a bounded wait cannot promise when
     * the thing being waited for takes a non-reproducible time (rq_io_complete()'s re-schedule
     * shares its queue with a wall-clock timer; see tests/mscprwdiff.js's assertSchedule).
     *
     * Measured, on the sibling that had the bound: a budget large enough for the common case still
     * expired about one run in five, and an expired budget lets the host walk on and HALT with the
     * transfer still in flight -- at which point vax_cpu.c's drain stops at the first UNIT_IDLE
     * queue entry and the two engines are compared at DIFFERENT POINTS IN ONE COMMAND'S LIFE.  The
     * failure reads as a controller defect (PBSY, CPKT, the free queue, half the packet array) and
     * it is a measurement artefact.
     *
     * THE COST IS PAID SOMEWHERE HONEST.  A controller that never answers now burns the case's
     * whole step budget and is reported BY NAME as a machine that never reached its own HALT,
     * rather than quietly producing a comparison of two different moments.  Use this wherever the
     * controller is REQUIRED to answer; use awaitL() below where it may legitimately not.
     */
    awaitUnbounded(addr, resultAddr, regs) {
        let {prev, cur, cnt} = regs;
        this.movAbsReg(4, addr, prev);                              // 7
        this.clrl(cnt);                                             // 2
        let top = this.len;
        this.movAbsReg(4, addr, cur);                               // 7
        this.emit(OPC.CMPL, 0x50 | cur, 0x50 | prev);               // 3
        this.emit(OPC.BNEQ, 4);                                     // 2 -- skip INCL(2) + BRB(2)
        this.emit(OPC.INCL, 0x50 | cnt);                            // 2
        this.emit(OPC.BRB, (top - (this.len + 2)) & 0xFF);          // 2
        this.movRegAbs(cnt, resultAddr);
        this.movRegAbs(cur, (resultAddr + 4) >>> 0);
        return this;
    }

    /**
     * awaitL(addr, resultAddr, limit, regs)
     *
     * A BOUNDED longword busy-wait that writes its own answer to memory:
     *
     *      MOVL @#addr, Rprev ; CLRL Rcnt ; MOVL I^#limit, Rlim
     * top: MOVL @#addr, Rcur ; CMPL Rcur, Rprev ; BNEQ out ; INCL Rcnt ; SOBGTR Rlim, top
     * out: MOVL Rcnt, @#result ; MOVL Rcur, @#result+4 ; MOVL Rlim, @#result+8
     *
     * SIX instructions per iteration, and it stores THREE longwords: the iteration count (the
     * in-band measure of the controller's schedule -- the thing a synchronous implementation cannot
     * reproduce), the value finally seen, and the REMAINING budget.  The last one is what makes the
     * bound safe to have: a case whose wait timed out stores zero there, and the differential fails
     * it by name instead of comparing two engines that both gave up (HANDOFF.md standing rule 6).
     *
     * Bounded rather than unbounded because a case may legitimately be waiting on something the
     * controller has decided not to do -- a descriptor it does not own, a controller it has taken
     * fatal.  A spin there would burn the whole step budget and report "never halted" for a case
     * whose point is that nothing happened.
     */
    awaitL(addr, resultAddr, limit, regs) {
        let {prev, cur, cnt, lim} = regs;
        this.movAbsReg(4, addr, prev);                              // 7
        this.clrl(cnt);                                             // 2
        this.emit(OPC.MOVL, 0x8F, ...lw(limit), 0x50 | lim);        // 7
        let top = this.len;
        this.movAbsReg(4, addr, cur);                               // 7
        this.emit(OPC.CMPL, 0x50 | cur, 0x50 | prev);               // 3
        this.emit(OPC.BNEQ, 5);                                     // 2 -- skip INCL(2) + SOBGTR(3)
        this.emit(OPC.INCL, 0x50 | cnt);                            // 2
        this.emit(OPC.SOBGTR, 0x50 | lim, (top - (this.len + 3)) & 0xFF);   // 3
        this.movRegAbs(cnt, resultAddr);
        this.movRegAbs(cur, (resultAddr + 4) >>> 0);
        this.movRegAbs(lim, (resultAddr + 8) >>> 0);
        return this;
    }
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test -- ONE, built once, reused by every case (standing rule 14)             *
 * ------------------------------------------------------------------------------------------- */

/**
 * makeMachine(opts)
 *
 * RAM at 0, the CQBIC register file and its map window in REG_BASE space, and the Qbus I/O page
 * with ONE device on it: the RQ controller's four bytes.  No doorbell, no SSC, no console, no
 * timers -- nothing graded here touches any of them, and leaving them out means a defect in one of
 * them cannot make either differential pass or fail.
 *
 * `opts` exists ONLY for --selfcheck's WIRING mutations, which are defects in how the device is
 * MOUNTED rather than in what it computes and so cannot be expressed by perturbing rq.js (standing
 * rule 11 -- the shipped construction is perturbed, not replaced):
 *
 *   rqBaseDelta   move the RQ window within the I/O page
 *   rqSizeDelta   widen or narrow the RQ window
 *   noQbusHook    leave cpu.qbus unwired, i.e. no event queue at all
 */
export function makeMachine(opts = {})
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    let cqbic = new CQBICVAX(cpu.exc, bus, MEMSIZE);
    let rq = new RQVAX(cqbic, {cnum: 0, ctype: RQDX3_CTYPE});
    bus.addRegBlock([
        {base: CQBIC_BASE, length: 0x14, dev: cqbic},
        {base: CQMAP_BASE, length: CQMAPSIZE, dev: new CQMAPVAX(cqbic)}
    ]);
    bus.addIoPage([{
        base: (RQ_BASE + (opts.rqBaseDelta || 0)) >>> 0,
        length: (IOLN_RQ + (opts.rqSizeDelta || 0)) >>> 0,
        dev: rq
    }]);
    if (!opts.noQbusHook) cpu.qbus = rq;
    sampleHeap();
    return {bus, cpu, cqbic, rq};
}

const MACHINES = new Map();

export function machine(opts = {})
{
    let key = `${opts.rqBaseDelta || 0}:${opts.rqSizeDelta || 0}:${opts.noQbusHook ? 1 : 0}`;
    if (!MACHINES.has(key)) MACHINES.set(key, makeMachine(opts));
    return MACHINES.get(key);
}

/* ------------------------------------------------------------------------------------------- *
 * The observation vector.  Every RQ register is read through the SAME width/offset rq_reg[]      *
 * publishes, on BOTH engines, so the comparison never reports a difference that is an artifact   *
 * of what the oracle is able to print.                                                          *
 * ------------------------------------------------------------------------------------------- */

/**
 * rq_reg[] (pdp11_rq.c:1230-1262), transcribed as {name, width, offset, get}.  `offset` is
 * GRDATA's own right-shift -- CQLNT/CQIDX/RQLNT/RQIDX are published SHIFTED DOWN BY TWO, which is
 * why the oracle prints 01 for a four-byte ring.  Getting that wrong would make every ring length
 * differ by a factor of four and look like a decode bug in rq.js rather than a printing convention.
 */
export const RQ_OBS = [
    {name: "SA",     width: 16, offset: 0, get: (r) => r.sa},
    {name: "SAW",    width: 16, offset: 0, get: (r) => r.saw},
    {name: "S1DAT",  width: 16, offset: 0, get: (r) => r.s1dat},
    {name: "COMM",   width: 22, offset: 0, get: (r) => r.comm},
    {name: "CQIOFF", width: 32, offset: 0, get: (r) => r.cq.ioff},
    {name: "CQBA",   width: 22, offset: 0, get: (r) => r.cq.ba},
    {name: "CQLNT",  width:  8, offset: 2, get: (r) => r.cq.lnt},
    {name: "CQIDX",  width:  8, offset: 2, get: (r) => r.cq.idx},
    {name: "RQIOFF", width: 32, offset: 0, get: (r) => r.rq.ioff},
    {name: "RQBA",   width: 22, offset: 0, get: (r) => r.rq.ba},
    {name: "RQLNT",  width:  8, offset: 2, get: (r) => r.rq.lnt},
    {name: "RQIDX",  width:  8, offset: 2, get: (r) => r.rq.idx},
    {name: "FREE",   width:  5, offset: 0, get: (r) => r.freq},
    {name: "RESP",   width:  5, offset: 0, get: (r) => r.rspq},
    {name: "PBSY",   width:  5, offset: 0, get: (r) => r.pbsy},
    {name: "CFLGS",  width: 16, offset: 0, get: (r) => r.cflgs},
    {name: "CSTA",   width:  4, offset: 0, get: (r) => r.csta},
    {name: "PERR",   width:  9, offset: 0, get: (r) => r.perr},
    {name: "CRED",   width:  5, offset: 0, get: (r) => r.credits},
    {name: "HAT",    width: 17, offset: 0, get: (r) => r.hat},
    {name: "HTMO",   width: 17, offset: 0, get: (r) => r.htmo},
    {name: "PRGI",   width:  1, offset: 0, get: (r) => (r.prgi ? 1 : 0)},
    {name: "PIP",    width:  1, offset: 0, get: (r) => (r.pip ? 1 : 0)}
];

/**
 * rqFieldOf(rq, o)
 *
 * SIMH's GRDATA/DRDATA rendering: right-shift by the register's OFFSET, then take the low `width`
 * bits.  Done with >>> and a modulus rather than `(1 << width) - 1` because a 32-bit field
 * (CQIOFF, which holds SA_COMM_CI == -4) must come out as the unsigned word the oracle prints
 * rather than as a negative JS number -- and `1 << 32` is 1, not 0x100000000.
 */
export function rqFieldOf(rq, o)
{
    let shifted = ((o.get(rq) | 0) >>> (o.offset || 0)) >>> 0;
    if (o.width >= 32) return shifted >>> 0;
    return (shifted % Math.pow(2, o.width)) >>> 0;
}

/**
 * The packet array, as SIMH publishes it: `VBRDATAD (PKTS, rq_ctx.pak, DEV_RDX, 16,
 * sizeof(rq_ctx.pak)/2, ...)` is a FLAT 16-bit view over `struct rqpkt { uint16 link; uint16 d[32]; }
 * pak[32]`, so index i is packet `i / 33`, and within it the LINK when `i % 33 == 0` and `d[i%33 - 1]`
 * otherwise.
 */
export const PKT_WORDS = 1 + RQ_PKT_SIZE_W;

export function pktWord(rq, i)
{
    let pkt = (i / PKT_WORDS) | 0, fld = i % PKT_WORDS;
    return fld === 0 ? (rq.pakLink[pkt] & 0xFFFF) : (rq.pakData[pkt * RQ_PKT_SIZE_W + (fld - 1)] & 0xFFFF);
}

/* ------------------------------------------------------------------------------------------- *
 * SHOW RQ RINGS / FREEQ / RESPQ, reproduced from rq.js's state                                   *
 * ------------------------------------------------------------------------------------------- */

/**
 * showCtrl(rq, cqbic, flags)
 *
 * rq_show_ctrl() (pdp11_rq.c:3548-3597), rq_show_ring() and rq_show_pkt(), for the VAX arm's `%x`
 * formats.  This lives in the harness rather than in rq.js because it is a RENDERING of state, not
 * device behaviour -- but it is graded as text against the oracle's own output, which makes it a
 * real assertion about `cq`/`rq`/`freq`/`rspq`/`pip`/`hat`, about the DESCRIPTORS the rings point
 * at (rq_show_ring reads them back THROUGH THE MAP, so a comm region zeroed to the wrong physical
 * page prints non-zero descriptors here even when every register agrees), and about the CONTENTS of
 * any deferred response packet.
 *
 * @param {RQVAX} rq
 * @param {CQBICVAX} cqbic
 * @param {string} flags "RI" | "FR" | "RS"
 * @returns {string}
 */
export function showCtrl(rq, cqbic, flags)
{
    if (rq.csta !== CST_UP) return "Controller is not initialized\n";
    let out = "";
    if (flags === "RI") {
        out += rq.pip ? `Polling in progress, host timer = ${rq.hat}\n` : `Host timer = ${rq.hat}\n`;
        out += "Command " + showRing(rq.cq, cqbic);
        out += "Response " + showRing(rq.rq, cqbic);
    } else if (flags === "FR") {
        let q = rq.freeQueue();
        if (q.length) {
            for (let i = 0; i < q.length; i++) {
                if (i === 0) out += `Free queue = ${q[i]}`;
                else if ((i % 16) === 0) out += `,\n ${q[i]}`;
                else out += `, ${q[i]}`;
            }
            out += "\n";
        } else out += "Free queue is empty\n";
    } else {
        let q = rq.respQueue();
        if (q.length) for (let p of q) out += "Response " + showPkt(rq, p);
        else out += "Response queue is empty\n";
    }
    return out;
}

/** rq_show_ring() -- `%3d: %08x` per slot, read back THROUGH THE MAP. */
export function showRing(ring, cqbic)
{
    let out = `ring, base = ${(ring.ba >>> 0).toString(16)}, index = ${ring.idx >> 2}, ` +
              `length = ${ring.lnt >> 2}\n`;
    let buf = new Uint8Array(4);
    for (let i = 0; i < (ring.lnt >> 2); i++) {
        if (cqbic.mapReadW((ring.ba + (i << 2)) >>> 0, 4, buf)) {
            out += ` ${String(i).padStart(3)}: non-existent memory\n`;
            break;
        }
        let desc = (buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24)) >>> 0;
        out += ` ${String(i).padStart(3)}: ${desc.toString(16).padStart(8, "0")}\n`;
    }
    return out;
}

/** rq_show_pkt() -- RQ_SH_MAX (24) words, RQ_SH_PPL (8) per line, `%04x`. */
export const RQ_SH_MAX = 24;
export const RQ_SH_PPL = 8;

export function showPkt(rq, pkt)
{
    let cr  = rq.getp(pkt, 1, UQ_HCTC_V_CR, UQ_HCTC_M_CR);
    let typ = rq.getp(pkt, 1, UQ_HCTC_V_TYP, UQ_HCTC_M_TYP);
    let cid = rq.getp(pkt, 1, UQ_HCTC_V_CID, UQ_HCTC_M_CID);
    let out = `packet ${pkt}, credits = ${cr}, type = ${typ}, cid = ${cid}\n`;
    for (let i = 0; i < RQ_SH_MAX; i += RQ_SH_PPL) {
        out += ` ${String(i).padStart(2)}:`;
        for (let j = i; j < i + RQ_SH_PPL; j++) out += " " + rq.pd(pkt, j).toString(16).padStart(4, "0");
        out += "\n";
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * Case-independent scatter arithmetic                                                           *
 * ------------------------------------------------------------------------------------------- */

/** The physical page a Qbus page is mapped to, per case.  DESCENDING and STRIDED, so any region a
    case programs is scattered and out of order -- which an identity mapping cannot produce. */
export function physPageFor(qpage, spread)
{
    let i = ((qpage * 7) + spread * 13) % DATA_NPAGE;
    return ((DATA_BASE + (DATA_NPAGE - 1 - i) * PAGE) / PAGE) | 0;
}

/** A non-zero, page-distinct seed.  Zeroing is only observable against a non-zero background, and
    a page-distinct one also catches a transfer that landed on the wrong page entirely. */
export function seedFor(ppage) { return ((0xA5A50000 | ((ppage * 0x0101) & 0xFFFF)) >>> 0); }

/**
 * geometry(spec)
 *
 * Where everything lives in QBUS space, computed FROM THE SPEC -- never by asking rq.js, which
 * would grade a defect against itself.  *** THE RESPONSE RING COMES FIRST ***: rq_step4() does
 * `rq.ba = comm; cq.ba = comm + rq.lnt`, so the response ring is at the base and the command ring
 * sits above it.  Measured on the live oracle with a one-descriptor pair at comm = 0x2000:
 * `Command ring, base = 2004` and `Response ring, base = 2000`.
 *
 * Packet buffers start on the first 64-byte boundary above both rings.  A descriptor names the
 * address of the packet's BODY, four bytes above the buffer's start, because the two-word UQSSP
 * header lives at `descriptor address + UQ_HDR_OFF`.
 *
 * @param {Object} spec {comm, rqCode, cqCode, nCmdBuf, nRspBuf}
 * @returns {Object}
 */
export function geometry(spec)
{
    let rqLnt = 4 << spec.rqCode, cqLnt = 4 << spec.cqCode;
    let rqBa = spec.comm >>> 0, cqBa = (spec.comm + rqLnt) >>> 0;
    let nCmd = spec.nCmdBuf, nRsp = spec.nRspBuf;
    let pgUp = (a) => (a + PAGE - 1) & ~(PAGE - 1);
    /* THE THREE AREAS ARE ON SEPARATE QBUS PAGES, deliberately.  A page is the unit the CQBIC
       scatter-gather map validates, so "unmap the page the command packets live on" is only a
       statement about command packets if nothing else lives there -- and the PE_PRE / PE_PWE cases
       are exactly that statement.  A 64-byte packing would put the rings, the command buffers and
       the response buffers in one page and make both fatals indistinguishable from PE_QRE. */
    let cmdBase = pgUp(spec.comm + rqLnt + cqLnt);
    let rspBase = cmdBase + pgUp(nCmd * 64);
    return {
        rqLnt, cqLnt, rqBa, cqBa, cmdBase, rspBase,
        rqSlots: rqLnt >> 2, cqSlots: cqLnt >> 2,
        cmdBuf: (i) => (cmdBase + i * 64) >>> 0,
        cmdEnv: (i) => (cmdBase + i * 64 + 4) >>> 0,
        rspBuf: (j) => (rspBase + j * 64) >>> 0,
        rspEnv: (j) => (rspBase + j * 64 + 4) >>> 0,
        /* The interrupt flag words sit BELOW comm: SA_COMM_CI (-4) for the command ring, SA_COMM_RI
           (-2) for the response ring, and SA_COMM_QQ (-8) is the lowest word rq_step4() zeroes when
           the host asked for a purge interrupt.  So the region starts eight bytes below `comm`. */
        lo: (spec.comm - 8) >>> 0,
        hi: (rspBase + pgUp(nRsp * 64) - 1) >>> 0
    };
}

/** Every Qbus page a case can reference, derived from the geometry rather than listed. */
export function qbusPagesFor(g)
{
    let pages = [];
    for (let a = g.lo & ~(PAGE - 1); a <= g.hi; a += PAGE) pages.push((a / PAGE) | 0);
    return pages;
}

/**
 * fileImageProvider(path)
 *
 * THE NODE HALF OF rq.js's IMAGE PROVIDER CONTRACT, and the reason that contract exists: this is the
 * only place in the tree that knows a disk image is a FILE.  rq.js takes `{byteLength, read()}` and
 * cannot tell an `fs` descriptor from a browser `File` read into an ArrayBuffer -- which is what
 * makes HANDOFF.md 8's user-supplied-image decision implementable in a browser rather than only
 * under Node.
 *
 * The descriptor is opened once and left open; the caller closes it.  `read()` is deliberately
 * REAL rather than a stub even though nothing in pcjsvax-f52 calls it -- a stub would satisfy
 * rq.js's contract check while being useless to pcjsvax-346, which is the failure mode the check
 * exists to prevent.  pcjsvax-346 calls it on every transfer, so the check paid.
 *
 * *** THE OPEN MODE IS sim_disk_attach_ex2()'s, NOT A CONVENIENCE. ***  The C opens the container
 * "rb+" and, if that fails, reopens it "rb", sets UNIT_RO and prints "Unit is read only"
 * (sim_disk.c:2894-2905).  This does the same: "r+" first, "r" on failure, and a provider opened
 * read-only carries NO `write` member at all -- which is how rq.js's attach() learns to force the
 * unit read-only.  Passing `{readOnly: true}` asks for the second arm directly, which is what a
 * caller does for a file it must not modify.
 *
 * @param {string} p
 * @param {Object} [opts] {readOnly}
 * @returns {Object}
 */
export function fileImageProvider(p, opts = {})
{
    let fd = null, writable = false;
    if (!opts.readOnly) {
        try { fd = fs.openSync(p, "r+"); writable = true; } catch (e) { fd = null; }
    }
    if (fd === null) fd = fs.openSync(p, "r");
    let prov = {
        byteLength: fs.statSync(p).size,
        path: p,
        writable,
        /* What sim_disk's get_filesystem_size() reports for this container, or undefined when it
           recognises nothing.  See ods2VolumeBytes(). */
        filesystemBytes: ods2VolumeBytes(fd),
        read(offset, length, dst) {
            return fs.readSync(fd, dst, 0, length, offset);
        }
    };
    /* pwrite(2), which is exactly what sim_os_disk_wrsect() issues.  Present ONLY when the
       container opened writable, because its ABSENCE is the signal rq.js's attach() reads. */
    if (writable) {
        prov.write = function(offset, length, src) {
            return fs.writeSync(fd, src, 0, length, offset);
        };
        /**
         * restoreFrom(src)
         *
         * Rewrite this container's whole contents from `src` THROUGH THE ALREADY-OPEN DESCRIPTOR.
         *
         * *** THE POINT IS WHAT IT DOES NOT DO: CLOSE AND RE-OPEN. ***  A differential that runs
         * many passes over the same containers (one per --selfcheck mutation) has to put each
         * writable container back before every pass, and the obvious way -- close the provider,
         * copyFileSync, re-open -- churns descriptors and re-resolves paths dozens of times per
         * run.  That churn is what an intermittent ENOENT on a scratch container lives in.  One
         * open, one close, and the restore is pwrite + ftruncate in between.
         */
        prov.restoreFrom = function(src) {
            let sfd = fs.openSync(src, "r");
            let off = 0;
            try {
                let buf = Buffer.allocUnsafe(1 << 20), n;
                while ((n = fs.readSync(sfd, buf, 0, buf.length, off)) > 0) {
                    fs.writeSync(fd, buf, 0, n, off);
                    off += n;
                }
            } finally { fs.closeSync(sfd); }
            fs.ftruncateSync(fd, off);
            prov.byteLength = off;
            prov.filesystemBytes = ods2VolumeBytes(fd);
            return off;
        };
    }
    /* Closing twice would close a descriptor number Node has since handed to something else, so
       the second close is a no-op rather than a second close of "whatever fd 7 is now". */
    let closed = false;
    prov.close = function() { if (!closed) { closed = true; fs.closeSync(fd); } };
    return prov;
}

/**
 * ods2VolumeBytes(fd)
 *
 * THE NODE `fs` ADAPTER over modules/v2/ods2.js's ods2VolumeBytesFrom(), which is
 * get_ods2_filesystem_size() (sim_disk.c:1257-1340) reduced to the one number it returns:
 * `Scb.scb_l_volsize * 512`, the size the VOLUME declares for itself.
 *
 * *** THE WALK ITSELF MOVED (pcjsvax-ae1) AND THIS IS ALL THAT IS LEFT OF IT HERE. ***  A browser
 * image provider needs the same number -- rq.js's attach() takes it as `provider.filesystemBytes`
 * and a unit attached without it takes autosize's OTHER arm -- and none of `fs.fstatSync`,
 * `fs.readSync` or Buffer's LE readers exist there.  Two copies of an ODS-2 parser in one tree is
 * exactly the drift HANDOFF.md standing rule 7 exists for, so the parser is parameterised by a
 * block reader and this function supplies the `fs` one.  The behaviour, including the
 * `undefined`-means-unrecognised contract, is unchanged; tests/mscpunitdiff.js grades it.
 *
 * *** THIS IS A FILE SYSTEM PARSER AND IT DOES NOT LIVE IN rq.js ON PURPOSE. ***  A disk controller
 * does not parse volumes; sim_disk does, before the controller ever sees the unit, and hands the
 * result to autosize.  rq.js takes the result through the image provider's `filesystemBytes` and
 * does the arithmetic; this is the Node half that produces it, exactly as fileImageProvider() is
 * the Node half of `read`.
 *
 * @param {number} fd an open descriptor
 * @returns {number|undefined} the volume's size in BYTES, or undefined if this is not ODS-2
 */
export function ods2VolumeBytes(fd)
{
    let size;
    try { size = fs.fstatSync(fd).size; } catch (e) { return undefined; }
    return ods2VolumeBytesFrom(size, (lbn) => {
        let b = Buffer.alloc(512);
        try { if (fs.readSync(fd, b, 0, 512, lbn * 512) !== 512) return null; } catch (e) { return null; }
        return new Uint8Array(b.buffer, b.byteOffset, 512);
    });
}

/**
 * commExtent(spec)
 *
 * The Qbus byte range rq_step4() will zero, computed FROM THE SPEC -- never by asking rq.js where
 * the data went, which would grade a defect against itself.  This is rq_step4()'s arithmetic
 * written a second time on purpose (the same discipline qdmadiff.js applies to its own page list);
 * its ONLY use is deciding which map entries a case programs and which physical pages get dumped,
 * so a disagreement between the two shows up as a memory difference rather than as a silent pass.
 */
export function commExtent(spec)
{
    let rqLnt = (1 << ((spec.s1dat >>> SA_S1H_V_RQ) & SA_S1H_M_RQ)) << 2;
    let cqLnt = (1 << ((spec.s1dat >>> SA_S1H_V_CQ) & SA_S1H_M_CQ)) << 2;
    let base = spec.comm + (spec.prgi ? SA_COMM_QQ : SA_COMM_CI);
    let lnt = spec.comm + cqLnt + rqLnt - base;
    if (lnt > SA_COMM_MAX) lnt = SA_COMM_MAX;
    return {base: base >>> 0, lnt, rqLnt, cqLnt};
}

/**
 * walkScript(s1dat, comm, prgi, opts)
 *
 * The four-step handshake as a host writes it, with a poll after each write.  Shared because both
 * differentials need a controller in CST_UP and neither should have its own idea of how to get one.
 *
 * The step-2 word carries the comm region's LOW half masked with SA_S2H_CLO (0xFFFE) and its low
 * bit is the purge-interrupt flag; the step-3 word carries the HIGH half, which the controller
 * shifts left 16 and ORs in.  Both are computed from `comm` here, so a case with a comm region
 * above 64KB exercises the shift rather than agreeing with a zero.
 *
 * Registers: R0..R4 hold the SA read-backs, R5..R8 the polling ITERATION COUNTS.  A caller that
 * continues past CST_UP must leave those alone -- mscpringdiff.js uses R9..R13 for exactly that
 * reason, and stores everything else to the RESULT page.
 */
export function walkScript(s1dat, comm, prgi, opts = {})
{
    let s = [{a: "rsa", r: 0}];
    s.push({a: "wsa", v: s1dat, step: 1});
    s.push({a: "poll", r: 1, prev: 0, cnt: 5});
    s.push({a: "wsa", v: ((comm & SA_S2H_CLO) | (prgi ? SA_S2H_PI : 0)) & 0xFFFF, step: 2});
    s.push({a: "poll", r: 2, prev: 1, cnt: 6});
    if (opts.pp) {
        s.push({a: "wsa", v: (((comm >>> 16) & SA_S3H_CHI) | SA_S3H_PP) & 0xFFFF, step: 3});
        s.push({a: "poll", r: 3, prev: 2, cnt: 7});          // SA -> 0
        s.push({a: "wsa", v: opts.ppaBad === undefined ? 0 : opts.ppaBad, step: 3.5});
        s.push({a: "delay", n: opts.ppaDelay, r: 11});
        s.push({a: "rip", r: 9});                            // the IP READ that completes step 4
        s.push({a: "rsa", r: 3});
    } else {
        s.push({a: "wsa", v: ((comm >>> 16) & SA_S3H_CHI) & 0xFFFF, step: 3});
        s.push({a: "poll", r: 3, prev: 2, cnt: 7});
    }
    if (!opts.stopBeforeGo) {
        s.push({a: "wsa", v: SA_S4H_GO, step: 4});
        s.push({a: "poll", r: 4, prev: 3, cnt: 8});
    }
    return s;
}

/**
 * emitAction(a, act)
 *
 * The BASE action vocabulary a case's `script` is written in.  Every one of them is a REAL
 * instruction against a REAL address; there is no "call the device" action, by construction --
 * HANDOFF.md 7 premise 7 is the reason, and it bites twice here: SIMH's console CANNOT reach the
 * I/O page at all (`e -p 20001468` answers "Address space exceeded"), so a console-level probe of
 * this device is not merely weaker, it is IMPOSSIBLE -- but a real CPU instruction reaches it fine.
 *
 * Returns false for an action it does not know, so a caller can extend the vocabulary by wrapping
 * rather than by copying.
 */
export function emitAction(a, act)
{
    switch (act.a) {
    case "rsa":   a.movAbsReg(2, RQ_SA, act.r); return true;            // MOVZWL @#SA, Rr
    case "rip":   a.movAbsReg(2, RQ_IP, act.r); return true;            // MOVZWL @#IP, Rr
    case "ripl":  a.movAbsReg(4, RQ_IP, act.r); return true;            // MOVL   @#IP, Rr -- both slots
    case "rb":    a.movAbsReg(1, (RQ_BASE + act.off) >>> 0, act.r); return true;
    case "rd":    a.movAbsReg(2, act.addr >>> 0, act.r); return true;   // an arbitrary probe address
    case "rl":    a.movAbsReg(4, act.addr >>> 0, act.r); return true;
    case "wsa":   a.movImmAbs(2, act.v, RQ_SA); return true;
    case "wip":   a.movImmAbs(2, act.v, RQ_IP); return true;
    case "wb":    a.movImmAbs(1, act.v, (RQ_BASE + act.off) >>> 0); return true;
    case "went":  a.movImmAbs(4, act.v >>> 0, (CQMAP_BASE + act.q * 4) >>> 0); return true;
    case "poll":  a.poll(RQ_SA, act.r, act.prev, act.cnt); return true;
    case "delay": a.delay(act.n, act.r); return true;
    }
    return false;
}

/* ------------------------------------------------------------------------------------------- *
 * Per-case reset, written once for both engines so they cannot drift apart                      *
 * ------------------------------------------------------------------------------------------- */

/**
 * simhResetLines(c)
 *
 * SIMH's side of the per-case reset: everything both differentials do before depositing their own
 * seeds and code.  `c` supplies the four RQ timing registers.
 */
export function simhResetLines(c)
{
    let L = [];
    L.push("reset -p all");
    L.push("deposit MAPEN 0");
    L.push(`deposit rq itime ${c.itime}`, `deposit rq i4time ${c.i4time}`,
           `deposit rq qtime ${c.qtime}`, `deposit rq xtime ${c.xtime}`);
    L.push("deposit qba mbr 0", "deposit qba dser 0", "deposit qba mear 0", "deposit qba sear 0",
           "deposit sysd bto 0", "deposit cpu memerr 0");
    L.push(`deposit SCBB ${hex(R_SCBB)}`, `deposit KSP ${hex(R_KSP)}`,
           `deposit R14 ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`,
           `deposit -l ${hex((R_SCBB + SCB.MCHK) >>> 0)} ${hex(R_MCHK_HDLR)}`,
           `deposit -l ${hex((R_SCBB + SCB.MEMERR) >>> 0)} ${hex(R_MERR_HDLR)}`);
    for (let k = 0; k < HDLR_NOPS; k++) {
        L.push(`deposit -b ${hex(R_MCHK_HDLR + k)} ${OPC.NOP.toString(16)}`);
        L.push(`deposit -b ${hex(R_MERR_HDLR + k)} ${OPC.NOP.toString(16)}`);
    }
    for (let k = 0; k < OBS_REGS; k++) if (k !== 14) L.push(`deposit R${k} 0`);
    /* The map's backing store and the low-memory window an UNPROGRAMMED map would read entries out
       of, both zeroed, so a fatal case fails for the reason it is supposed to. */
    L.push(`deposit -l ${hex(MAP_MBR)}:${hex(MAP_HI - 4)} 0`);
    L.push(`deposit -l 0:${hex(LOWMAP_HI - 4)} 0`);
    return L;
}

/**
 * jsResetForCase(m, c)
 *
 * The JS side of the SAME reset, term for term -- and NOTHING MORE.  rq.reset() is rq_reset(),
 * which leaves `perr`, `saw`, `prgi` and the rings' `ioff` alone; they are static-struct fields on
 * the oracle and carry from case to case there, so clearing them here would diverge on the second
 * case that looked at one.  The fresh-SIMH-process state is rq.powerUp(), which a caller invokes
 * ONCE per pass -- that is the boundary the oracle actually has, since every pass writes a new
 * do-file and starts a new simulator.
 */
export function jsResetForCase(m, c)
{
    let {bus, cpu, cqbic, rq} = m;
    cpu.reset();
    rq.reset();
    rq.itime = c.itime; rq.itime4 = c.i4time; rq.qtime = c.qtime; rq.xtime = c.xtime;
    cqbic.reset();
    cpu.exc.cqDser = 0; cpu.exc.cqMear = 0; cpu.exc.sscBto = 0; cpu.exc.memErr = 0;
    cpu.exc.scbb = R_SCBB;
    cpu.regs[14] = R_KSP;
    cpu.exc.stk[0] = R_KSP;
    cpu.exc.stk[4] = R_IS;
    for (let k = 0; k < OBS_REGS; k++) if (k !== 14) cpu.regs[k] = 0;
    for (let k = 0; k < HDLR_NOPS; k++) {
        bus.setByte((R_MCHK_HDLR + k) >>> 0, OPC.NOP);
        bus.setByte((R_MERR_HDLR + k) >>> 0, OPC.NOP);
    }
    bus.setLong((R_SCBB + SCB.MCHK) >>> 0, R_MCHK_HDLR);
    bus.setLong((R_SCBB + SCB.MEMERR) >>> 0, R_MERR_HDLR);
    for (let a = MAP_MBR; a < MAP_HI; a += 4) bus.setLong(a >>> 0, 0);
    for (let a = 0; a < LOWMAP_HI; a += 4) bus.setLong(a >>> 0, 0);
}

export { RQVAX, RQ_BASE, IOLN_RQ, RQDX3_CTYPE, RQ_NPKTS, RQ_PKT_SIZE_W, CST_UP };
