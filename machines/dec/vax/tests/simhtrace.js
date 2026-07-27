/**
 * @fileoverview Emits a SIMH-format VAX instruction-history trace from a running CPUStateVAX
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
 * WHAT THIS IS FOR
 * ============================================================================
 * `tools/trace-differ/differ.py` (in the pcjs-vax work repo) compares two SIMH instruction-history
 * logfiles and reports the FIRST point of divergence.  It was built in Wave 0 as the day-one
 * grading mechanism for a RUNNING machine and, until pcjsvax-c05, there was no running machine to
 * point it at.  This module is the other half: it makes CPUStateVAX emit a trace in exactly the
 * format `differ.py` parses, so the oracle is a real SIMH run of the same program and the answer
 * is an instruction index, a PC and a field name.
 *
 * The format is `cpu_show_hist_records()` (vax_cpu.c:3600) with patch 0001's `REGS` line:
 *
 *     PC       PSL       IR
 *     <8-hex PC> <8-hex PSL>| <disassembly>
 *                        <operand hex...> [-> <result hex...>]
 *     REGS <16 8-hex values>
 *
 * Three parts of that are ported here, each from a named SIMH routine:
 *
 *   * `fprint_sym_m()` (vax_sys.c:816) -- the disassembler.  Reproduced to the character, including
 *     `%-X` (uppercase hex, unsigned, no padding, no `0x`) and the exact negative-displacement
 *     spelling `-%X(Rn)`.  This is not decoration: the disassembly is a second, independent
 *     statement of which bytes the specifier decoder consumed, so a decoder that resolved the
 *     right VALUE from the wrong BYTES fails here rather than silently.
 *   * `cpu_show_opnd()` (vax_cpu.c:3644) -- the operand line, driven by each specifier's decode ROM
 *     ACCESS TYPE, not its addressing mode, and using the same octaword continuation-line rules.
 *   * The result tail -- see RESULTS below, which is the one place this file does not simply port.
 *
 * ============================================================================
 * RESULTS -- THE ONE RECONSTRUCTION, AND HOW IT IS PROVED
 * ============================================================================
 * SIMH's result fields come from `r` and `rh`, two LOCALS of `sim_instr()` that every instruction
 * body assigns before storing.  A port whose instruction bodies live in five separate modules has
 * no such local, and threading one through 242 bodies to serve a trace would put trace bookkeeping
 * in the hot path of verified code.
 *
 * So the values are RECONSTRUCTED, per the decode ROM's own result shape:
 *
 *   RB_R0/R1/R3/R5   read R0..R5 -- which is literally what SIMH does, at the same moment
 *   RB_SP            read the longword at SP -- likewise
 *   RB_B/W/L/Q       read back the DESTINATION the decoder's last specifier names
 *
 * Only the last is a reconstruction rather than a port, and it is an assertion about SIMH that
 * this file does not get to make on its own credit: `r` is the value the body stored, so reading
 * the destination immediately afterwards must return it.  cpudiff.js proves or disproves that over
 * every instruction of a real EHKAA run -- a wrong reconstruction is a divergence with an opcode
 * name attached, not a silent weakening.  `--no-results` exists for the case where it is disproved
 * and the run still needs to grade PC/PSL/operands/registers.
 *
 * TIMING MATTERS AND IS EASY TO GET WRONG.  SIMH fills a record's result fields at the TOP OF THE
 * NEXT LOOP ITERATION (vax_cpu.c:613-649), not at the end of the instruction.  For an instruction
 * that FAULTED, that point is AFTER the exception has been dispatched -- so an RB_R0 record for a
 * faulting instruction shows the post-dispatch R0, and `r` for an RB_L record is whatever the
 * PREVIOUS successful instruction left in the local.  Both are reproduced: `captureResult()` runs
 * only when the body completed, and `finish()` runs at the top of the next iteration.
 */

import { OPCODES, DROM, DROM_STRIDE, DR, SPEC, MODE } from "../modules/v2/drom.js";
import { DISPATCH } from "../modules/v2/cpustate.js";
import MemoryVAX from "../modules/v2/memory.js";

const L_BYTE = 1, L_WORD = 2, L_LONG = 4, L_QUAD = 8, L_OCTA = 16;
const BSIGN = 0x80, WSIGN = 0x8000, LSIGN = 0x80000000 | 0;
const BMASK = 0xFF, WMASK = 0xFFFF;
const nPC = 15, nSP = 14;

const PSL_FPD = 1 << 27;

/* vax_defs.h RB_* >> DR_V_RESMASK, i.e. the values DR_GETRES() yields. */
const RB = {
    NONE: 0, B: 1, W: 2, L: 3, Q: 4, O: 5, OB: 6, OW: 7, OL: 8, OQ: 9,
    R0: 10, R1: 11, R3: 12, R5: 13, SP: 14
};

const REGNAME = ["R0", "R1", "R2", "R3", "R4", "R5", "R6", "R7",
                 "R8", "R9", "R10", "R11", "AP", "FP", "SP", "PC"];

const BITB_OPC = OPCODES.indexOf("BITB");
const CMPD_OPC = OPCODES.indexOf("CMPD");

/**
 * nonStoringResultOpcodes()
 *
 * The IMPLEMENTED opcodes that carry a B/W/L/Q result shape but whose LAST specifier is neither a
 * write nor a modify -- i.e. the ones for which "read back the destination" is not a reconstruction
 * of SIMH's `r` because there IS no destination.  Computed from the decode ROM, never listed by
 * hand, so a drom regeneration that adds a third one is caught rather than silently mis-traced;
 * cpudiff.js asserts the result is exactly {BITB, CMPD}.
 *
 * @returns {Array.<string>} mnemonics
 */
function nonStoringResultOpcodes()
{
    let out = [];
    for (let opc = 0; opc < 512; opc++) {
        if (!OPCODES[opc]) continue;
        /* Only opcodes this CPU actually EXECUTES: one with no body never produces a result at
           all, and captureResult() already returns early for those (see the DISPATCH check there).
           `CRC` is the reason this filter exists -- it satisfies the decode-ROM predicate but is
           IG_EMONL, so it takes the emulate trap and stores nothing for a different reason. */
        if (!DISPATCH[opc]) continue;
        let hdr = DROM[opc * DROM_STRIDE];
        let res = (hdr >> DR.V_RESMASK) & DR.M_RESMASK;
        if (res < RB.B || res > RB.Q) continue;
        let ns = hdr & DR.NSPMASK;
        let ac = DROM[opc * DROM_STRIDE + ns] & DR.ACMASK;
        if (ac != DR.W && ac != DR.M) out.push(OPCODES[opc]);
    }
    return out;
}

/** `%08X` */
function h8(v) { return (v >>> 0).toString(16).toUpperCase().padStart(8, "0"); }
/** `%-X` -- uppercase hex, unsigned, unpadded. */
function hx(v) { return (v >>> 0).toString(16).toUpperCase(); }

function SXTB(x) { return (x & BSIGN) ? (x | ~BMASK) : (x & BMASK); }
function SXTW(x) { return (x & WSIGN) ? (x | ~WMASK) : (x & WMASK); }

function drLnt(disp) { return 1 << (disp & DR.LNMASK); }

/**
 * disassemble(bytes, addr)
 *
 * vax_sys.c:816 `fprint_sym_m()`.  `bytes` is the instruction's own byte stream starting at
 * `addr`; returns the disassembly string, or null when the opcode has no mnemonic (which
 * cpu_show_hist_records() renders as "%03X (undefined)").
 *
 * `vp` is the index of the next byte to consume and is part of every PC-relative computation
 * (`addr + vp + disp`), so it must be advanced in exactly SIMH's order -- notably, the index-mode
 * byte is consumed BEFORE the specifier it indexes, and an immediate's bytes are consumed inside
 * the AIN case.
 *
 * @param {Uint8Array|Array} bytes
 * @param {number} addr
 * @returns {?string}
 */
function disassemble(bytes, addr)
{
    let vp = 0;
    let get = function(n) {
        let d = 0;
        for (let k = 0; k < n; k++) d = d | ((bytes[vp++] | 0) << (k * 8));
        return d;
    };
    let inst = bytes[0] | 0;
    vp = 1;
    if (inst == 0xFD) inst = 0x100 | (bytes[vp++] | 0);
    if (!OPCODES[inst]) return null;
    let hdr = DROM[inst * DROM_STRIDE];
    let numspec = hdr & DR.NSPMASK;
    if (numspec == 0) numspec = (hdr >> 4) & 0x7;       // DR_GETUSP
    let out = OPCODES[inst];

    for (let i = 0; i < numspec; i++) {
        out += i ? "," : " ";
        let disp = DROM[inst * DROM_STRIDE + i + 1];
        if (disp == SPEC.BB) {
            let num = get(1);
            out += hx(SXTB(num) + addr + vp);
            continue;
        }
        if (disp == SPEC.BW) {
            let num = get(2);
            out += hx(SXTW(num) + addr + vp);
            continue;
        }
        let spec = bytes[vp++] | 0;
        let index = 0;
        if ((spec & 0xF0) == MODE.IDX) {
            index = spec;
            spec = bytes[vp++] | 0;
        }
        let rn = spec & 0xF;
        let num;
        switch (spec & 0xF0) {
        case MODE.SH0: case MODE.SH1: case MODE.SH2: case MODE.SH3:
            out += "#" + hx(spec);
            break;
        case MODE.GRN:
            out += REGNAME[rn];
            break;
        case MODE.RGD:
            out += "(" + REGNAME[rn] + ")";
            break;
        case MODE.ADC:
            out += "-(" + REGNAME[rn] + ")";
            break;
        case MODE.AIN:
            if (rn != nPC) {
                out += "(" + REGNAME[rn] + ")+";
            } else if (drLnt(disp) == L_OCTA) {
                let r = qoimm(bytes, vp, 4); out += r.text; vp = r.vp;
            } else if (drLnt(disp) == L_QUAD) {
                let r = qoimm(bytes, vp, 2); out += r.text; vp = r.vp;
            } else {
                num = get(drLnt(disp));
                out += "#" + hx(num);
            }
            break;
        case MODE.AID:
            if (rn != nPC) out += "@(" + REGNAME[rn] + ")+";
            else { num = get(4); out += "@#" + hx(num); }
            break;
        case MODE.BDD:
            out += "@";
            /* fall through */
        case MODE.BDP:
            num = get(1);
            if (rn == nPC) out += hx(addr + vp + SXTB(num));
            else if (num & BSIGN) out += "-" + hx(-num & BMASK) + "(" + REGNAME[rn] + ")";
            else out += hx(num) + "(" + REGNAME[rn] + ")";
            break;
        case MODE.WDD:
            out += "@";
            /* fall through */
        case MODE.WDP:
            num = get(2);
            if (rn == nPC) out += hx(addr + vp + SXTW(num));
            else if (num & WSIGN) out += "-" + hx(-num & WMASK) + "(" + REGNAME[rn] + ")";
            else out += hx(num) + "(" + REGNAME[rn] + ")";
            break;
        case MODE.LDD:
            out += "@";
            /* fall through */
        case MODE.LDP:
            num = get(4);
            if (rn == nPC) out += hx(addr + vp + num);
            else if (num & LSIGN) out += "-" + hx(-num) + "(" + REGNAME[rn] + ")";
            else out += hx(num) + "(" + REGNAME[rn] + ")";
            break;
        }
        if (index) out += "[" + REGNAME[index & 0xF] + "]";
    }
    return out;
}

/**
 * qoimm(bytes, vp, lnt)
 *
 * vax_sys.c `fprint_sym_qoimm()`: a quad/octa PC-immediate is printed as `#<high>` followed by the
 * remaining longwords zero-padded, with leading all-zero longwords SUPPRESSED -- so `#0` and
 * `#1234567800000000` are both possible spellings and the padding rule changes after the first
 * printed longword.
 *
 * @param {Uint8Array|Array} bytes
 * @param {number} vp
 * @param {number} lnt number of longwords
 * @returns {{text: string, vp: number}}
 */
function qoimm(bytes, vp, lnt)
{
    let num = new Array(lnt).fill(0);
    for (let i = 0; i < lnt; i++) {
        let d = 0;
        for (let k = 0; k < 4; k++) d = d | ((bytes[vp++] | 0) << (k * 8));
        num[lnt - 1 - i] = d;
    }
    let text = "", startp = 0;
    for (let i = 0; i < lnt; i++) {
        if (startp) text += h8(num[i]);
        else if (num[i] || (i == lnt - 1)) { text += "#" + hx(num[i]); startp = 1; }
    }
    return {text, vp};
}

/**
 * @class SimhTrace
 *
 * Installed on a CPUStateVAX as `cpu.hst`.  Three callbacks, at the three points vax_cpu.c takes
 * the corresponding action -- see the file header for why the third one's timing is load-bearing.
 */
class SimhTrace {
    /**
     * @param {Object} [opts]
     * @param {boolean} [opts.results] emit the `-> ...` tail (default true)
     * @param {number} [opts.limit] stop recording after this many records (0 = unlimited)
     */
    constructor(opts = {})
    {
        this.fResults = opts.results !== false;
        this.limit = opts.limit || 0;
        this.lines = ["PC       PSL       IR", ""];
        this.count = 0;
        this.pending = null;
        this.rTrace = 0;
        this.rhTrace = 0;
        this.fResultValid = false;
        /* Counted, reported: a record whose instruction bytes could not be read back. */
        this.nUnreadable = 0;
        /* Record indices whose SIMH `r` is unreconstructable -- see finish(). */
        this.unavailable = [];
    }

    /**
     * record(cpu, opc)
     *
     * vax_cpu.c:1563-1590, taken AFTER specifier resolution and BEFORE the instruction body.
     *
     * @param {Object} cpu
     * @param {number} opc
     */
    record(cpu, opc)
    {
        if (this.limit && this.count >= this.limit) { this.pending = null; return; }
        let d = cpu.decoder;
        let iPC = cpu.exc.faultPC;
        let lim = (cpu.regs[nPC] - iPC) | 0;
        if (lim < 0 || lim > 52) lim = 52;               // INST_SIZE
        let inst = new Uint8Array(52);
        let ok = true;
        for (let i = 0; i < lim; i++) {
            let b = this.peekB(cpu, (iPC + i) | 0);
            if (b < 0) { inst[0] = inst[1] = 0xFF; ok = false; this.nUnreadable++; break; }
            inst[i] = b;
        }
        this.pending = {
            iPC: iPC >>> 0,
            psl: cpu.psl >>> 0,
            opc,
            inst,
            opnd: Int32Array.from(d.opnd),
            nOpnd: d.nOpnd,
            /*
             * patch 0001 copies R[0..15] HERE, at the history-record point -- i.e. AFTER specifier
             * resolution and BEFORE the body.  So `REGS` is the POST-RESOLUTION register file
             * (autoincrement side effects already applied, PC already past the last specifier),
             * not the post-execution one.  A `BRW` record proves it: SIMH prints R15 = the address
             * after the displacement, not the branch target.  Capturing it in finish() instead is
             * wrong for every instruction that moves the PC or writes a register.
             */
            regs: Int32Array.from(cpu.regs),
            res: [0, 0, 0, 0, 0, 0],
            completed: false,
            resultUnreliable: false,
            ok
        };
    }

    /**
     * captureResult(cpu)
     *
     * Not a SIMH routine: this is where `r`/`rh` are reconstructed (see the file header).  Called
     * only when the instruction body COMPLETED, which is what makes a faulting instruction inherit
     * the previous instruction's `r`, exactly as a stale C local does.
     *
     * @param {Object} cpu
     */
    captureResult(cpu)
    {
        if (!this.pending || !this.fResults) return;
        this.pending.completed = true;
        let opc = this.pending.opc;
        let res = (DROM[opc * DROM_STRIDE] >> DR.V_RESMASK) & DR.M_RESMASK;
        if (res != RB.B && res != RB.W && res != RB.L && res != RB.Q) return;
        let d = cpu.decoder;
        /*
         * TWO OPCODES IN THE WHOLE BASE GROUP CARRY A RESULT SHAPE AND DO NOT STORE ANYTHING, and
         * they are not guessable -- they are computed by asking the decode ROM which opcodes have a
         * B/W/L/Q result shape whose LAST specifier is neither `w` nor `m` (see cpudiff.js's
         * NONSTORING assertion, which recomputes the set and fails if it ever changes):
         *
         *   BITB  vax_sys.c gives it RB_B although BITW and BITL have none.  vax_cpu.c:1851 sets
         *         `r = op1 & op0` and never stores, so the trace shows the AND, not the operand.
         *   CMPD  vax_sys.c gives it RB_Q although CMPF and CMPG have none.  vax_cpu.c:2818 sets
         *         only `cc` -- it never touches `r` or `rh` at all -- so SIMH prints whatever the
         *         last instruction that DID produce a result left in those locals.  That is exactly
         *         what leaving rTrace/rhTrace alone here reproduces.
         */
        if (opc == BITB_OPC) {
            this.rTrace = (d.opnd[1] & d.opnd[0]) & BMASK;
            return;
        }
        if (opc == CMPD_OPC) return;                    // stale locals, deliberately
        /*
         * An opcode with no instruction body did not run one, so it produced no result either.  On
         * this CPU model that is every packed-decimal / emulated-only opcode: `op_cis()` hands them
         * to `cpu_emulate_exception()`, which builds a frame and vectors -- it never assigns `r`.
         * SIMH therefore prints the PREVIOUS result-producing instruction's `r`, which is exactly
         * what returning here leaves in rTrace.  EHKAA's first CVTPL is where this shows up.
         */
        if (!DISPATCH[opc]) return;
        let lnt = (res == RB.B) ? L_BYTE : (res == RB.W) ? L_WORD : L_LONG;
        try {
            if (d.isMemoryDestination()) {
                /*
                 * pcjsvax-bfb: the reconstruction's own load-bearing assertion (see the file header
                 * RESULTS section) is "r is the value the body stored, so reading the destination
                 * immediately afterwards must return it" -- true for RAM, false by DESIGN for a
                 * device register that transforms what it stores (a mask, a W1C bit, ...): SIMH's
                 * own `r` local still holds the RAW value the body computed, but reading the
                 * register back returns whatever the device's OWN write-side semantics produced,
                 * which can legitimately differ (first observed: SSC+0x30's OTP register masks to
                 * 4 bits, `MCOML #E,@#20140030` computes 0xFFFFFFF1 but a readback returns
                 * 0x00000001).  That is not a bug in the register model -- it is the ENTIRE REASON
                 * this item's done condition 2 grades device registers with a DIFFERENT, dedicated
                 * oracle (patch 0006's IPRR/IPRW/REGR/REGW trace, not this generic per-instruction
                 * reconstruction).  A destination backed by a MemoryVAX.TYPE.CONTROLLER block (every
                 * device model in this tree, not merely the console) is therefore marked
                 * resultUnreliable exactly like an unreadable-after-a-store-fault destination
                 * already is: dropped from BOTH traces by cpudiff.js/romdiff.js's DIFF_DRIVER (see
                 * `unavailable` below), never silently compared against a value it was never
                 * promised to reproduce.
                 *
                 * MEASURED CORRECTION (veracity re-dispatch, same item): `d.va` is a VIRTUAL
                 * address (mmu.readData() is what translates it) -- a first version of this check
                 * called `cpu.bus.getBlock(d.va)` directly, treating an UNTRANSLATED virtual address
                 * as if it were physical.  Under MAPEN that is not merely "the wrong block", it can
                 * be an OUT-OF-RANGE block index entirely (`aMemBlocks[hugeIndex]` -> `undefined`),
                 * and `undefined.type` throws inside this function's own try/catch -- silently
                 * discarded, leaving `rTrace` STALE from whatever the PREVIOUS result-producing
                 * instruction left there.  This is exactly the regression cpudiff.js's EHKAA phase
                 * caught: `MOVAL 800071C5,800206C0` diverged because the destination's result was
                 * never actually read back.  The fix reads the destination FIRST (mmu.readData()
                 * itself has already translated it, unconditionally, before this class ever existed)
                 * and THEN inspects `cpu.mmu.pa` -- the physical address that SAME translation just
                 * resolved (mmu.js's readData() sets `this.pa` on every call, `MMUVAX.readData()`'s
                 * own doc comment) -- for the controller check, never a second, independent
                 * translation of its own.
                 */
                this.rTrace = cpu.mmu.readData(d.va, lnt, cpu.accR()) | 0;
                if (res == RB.Q) this.rhTrace = cpu.mmu.readData((d.va + 4) | 0, L_LONG, cpu.accR()) | 0;
                let pa = (cpu.mmu.pa >>> 0) & cpu.bus.nBusMask;
                if (cpu.bus.getBlock(pa).type === MemoryVAX.TYPE.CONTROLLER) {
                    this.pending.resultUnreliable = true;
                }
            } else {
                this.rTrace = cpu.regs[d.rn] | 0;
                if (res == RB.Q) this.rhTrace = cpu.regs[(d.rn + 1) & 0xF] | 0;
            }
            if (res == RB.B) this.rTrace = this.rTrace & BMASK;
            else if (res == RB.W) this.rTrace = this.rTrace & WMASK;
        } catch (e) {
            /* A destination that cannot be read back leaves the previous value, which is also what
               a stale C local does.  Never allowed to perturb the machine: this is a trace. */
        }
    }

    /**
     * finish(cpu)
     *
     * vax_cpu.c:613-649 -- the TOP of the next loop iteration.  Fills the previous record's result
     * fields and emits it.
     *
     * @param {Object} cpu
     */
    finish(cpu)
    {
        let p = this.pending;
        if (!p) return;
        this.pending = null;
        let hdr = DROM[p.opc * DROM_STRIDE];
        let res = (hdr >> DR.V_RESMASK) & DR.M_RESMASK;
        /*
         * THE ONE CASE THE RECONSTRUCTION CANNOT REACH, recorded by index rather than tolerated.
         *
         * SIMH assigns `r` BEFORE the store (`r = <value>; WRITE_L (r);`), so an instruction whose
         * STORE faults -- EHKAA does this on purpose: the `INDEX ...,-(FP)` at 80015D74 writes into
         * an inaccessible page to prove the fault path -- still has a meaningful `r` in the trace.
         * Reading the destination back cannot produce it, because nothing was written.  Everything
         * else about such a record (PC, PSL, disassembly, operands, register file) is compared
         * normally; only the reconstructed result value is unavailable, and cpudiff.js drops that
         * one field from BOTH traces at exactly these indices, counts them, and reports the count.
         */
        if (this.fResults && (!p.completed || p.resultUnreliable) && res >= RB.B && res <= RB.Q) {
            this.unavailable.push(this.count);
        }
        if (this.fResults) {
            switch (res) {
            case RB.Q: p.res[1] = this.rhTrace; p.res[0] = this.rTrace; break;
            case RB.B: case RB.W: case RB.L: p.res[0] = this.rTrace; break;
            case RB.R5: p.res[5] = cpu.regs[5]; p.res[4] = cpu.regs[4];
                /* fall through */
            case RB.R3: p.res[3] = cpu.regs[3]; p.res[2] = cpu.regs[2];
                /* fall through */
            case RB.R1: p.res[1] = cpu.regs[1];
                /* fall through */
            case RB.R0: p.res[0] = cpu.regs[0]; break;
            case RB.SP:
                try { p.res[0] = cpu.mmu.readData(cpu.regs[nSP], L_LONG, cpu.accR()) | 0; }
                catch (e) { p.res[0] = 0; }
                break;
            default: break;
            }
        }
        this.emit(p, cpu);
        this.count++;
    }

    /**
     * peekB(cpu, va)
     *
     * `cpu_ex(..., SWMASK('V'))` -- read one byte through the CURRENT mapping without faulting and
     * without disturbing anything.  mmu.test() is exactly this seam (see the VAX README's "probe
     * seam"); with mapping off it is the identity, which mmu.test() already handles.
     *
     * @param {Object} cpu
     * @param {number} va
     * @returns {number} 0-255, or -1 if inaccessible
     */
    peekB(cpu, va)
    {
        try {
            let stat = {code: 0};
            let pa = cpu.mmu.test(va, cpu.accR(), stat);
            if (pa < 0) return -1;
            return cpu.mmu.readB(pa);
        } catch (e) {
            return -1;
        }
    }

    /**
     * emit(p, cpu)
     *
     * vax_cpu.c:3600 `cpu_show_hist_records()` for one record, plus patch 0001's REGS line.
     *
     * @param {Object} p
     * @param {Object} cpu
     */
    emit(p, cpu)
    {
        let hdr = DROM[p.opc * DROM_STRIDE];
        let numspec = hdr & DR.NSPMASK;
        let line = h8(p.iPC) + " " + h8(p.psl) + "| ";
        let mn = OPCODES[p.opc];
        if (!mn) {
            line += hx3(p.opc) + " (undefined)";
        } else if (p.psl & PSL_FPD) {
            line += mn + " FPD set";
        } else {
            let dis = p.ok ? disassemble(p.inst, p.iPC) : null;
            line += (dis === null) ? hx3(p.opc) + " (undefined)" : dis;
            if ((numspec > 1) || ((numspec == 1) && (DROM[p.opc * DROM_STRIDE + 1] < SPEC.BB))) {
                if (this.showOpnd(p, 0, (s) => { line += s; })) {
                    if (this.showOpnd(p, 1, (s) => { line += s; })) {
                        this.showOpnd(p, 2, (s) => { line += s; });
                        this.showOpnd(p, 3, (s) => { line += s; });
                    }
                }
            }
        }
        this.lines.push(line);
        let regs = "REGS";
        for (let i = 0; i < 16; i++) regs += " " + h8(p.regs[i]);
        this.lines.push(regs);
    }

    /**
     * showOpnd(p, line, put)
     *
     * vax_cpu.c:3644 `cpu_show_opnd()`.  The `RG -> RQ` fixup, the `disp >= BB` break, the
     * nine-space filler for a slot this continuation line does not print, and the write-specifier
     * rule (print opnd[j+1], the ADDRESS half) are all SIMH's and all load-bearing.
     *
     * @param {Object} p
     * @param {number} line
     * @param {function(string)} put
     * @returns {boolean} true if another continuation line is needed
     */
    showOpnd(p, line, put)
    {
        let numspec = DROM[p.opc * DROM_STRIDE] & DR.NSPMASK;
        put("\n                  ");
        let more = false;
        let j = 0;
        for (let i = 1; i <= numspec; i++) {
            let disp = DROM[p.opc * DROM_STRIDE + i];
            if (disp == SPEC.RG) disp = SPEC.RQ;
            if (disp >= SPEC.BB) break;
            switch (disp & (DR.LNMASK | DR.ACMASK)) {
            case SPEC.RB: case SPEC.RW: case SPEC.RL:
            case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
            case SPEC.MB: case SPEC.MW: case SPEC.ML:
                if (line == 0) put(" " + h8(p.opnd[j]));
                else put("         ");
                j += 1;
                break;
            case SPEC.RQ: case SPEC.MQ:
                if (line <= 1) put(" " + h8(p.opnd[j + line]));
                else put("         ");
                if (line == 0) more = true;
                j += 2;
                break;
            case SPEC.RO: case SPEC.MO:
                put(" " + h8(p.opnd[j + line]));
                more = true;
                j += 4;
                break;
            case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                if (line == 0) put(" " + h8(p.opnd[j + 1]));
                else put("         ");
                j += 2;
                break;
            }
        }
        let res = (DROM[p.opc * DROM_STRIDE] >> DR.V_RESMASK) & DR.M_RESMASK;
        if (line == 0 && res && this.fResults) {
            put(" ->");
            switch (res) {
            case RB.O:
                put(" " + h8(p.res[0]) + " " + h8(p.res[1]) + " " + h8(p.res[2]) + " " + h8(p.res[3]));
                break;
            case RB.Q:
                put(" " + h8(p.res[0]) + " " + h8(p.res[1]));
                break;
            case RB.B: case RB.W: case RB.L:
                put(" " + h8(p.res[0]));
                break;
            case RB.R0: case RB.R1: case RB.R3: case RB.R5: {
                let rcnts = {[RB.R0]: 1, [RB.R1]: 2, [RB.R3]: 4, [RB.R5]: 6};
                for (let i = 0; i < rcnts[res]; i++) put(" R" + i + ":" + h8(p.res[i]));
                break;
            }
            case RB.SP:
                put(" SP: " + h8(p.res[0]));
                break;
            default:
                break;
            }
        }
        return more;
    }

    /**
     * text()
     * @returns {string}
     */
    text() { return this.lines.join("\n") + "\n"; }
}

function hx3(v) { return (v >>> 0).toString(16).toUpperCase().padStart(3, "0"); }

export default SimhTrace;
export { SimhTrace, disassemble, nonStoringResultOpcodes, RB };
