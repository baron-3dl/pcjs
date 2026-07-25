/**
 * @fileoverview VAX instruction decode and operand-specifier resolution
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
 * WHAT THIS IS, AND THE CONTRACT IT PUBLISHES
 * ============================================================================
 * Everything between "fetch the opcode byte" and "the instruction body may now run": opcode
 * fetch (including the FD prefix), the instruction-group check that decides whether this CPU
 * implements the opcode at all, and the resolution of up to six operand specifiers through the
 * nine VAX addressing modes.  It does NOT execute instructions.
 *
 * This is a direct port of the specifier flow in Open SIMH `vax_cpu.c` `sim_instr()`, lines
 * 731-1560, and is deliberately structured so it can be read side by side with that source.
 * Because we port FROM SIMH, a divergence from SIMH is by construction a transcription bug with
 * a known location, not an argument about who misread the architecture manual.
 *
 * THE OUTPUT CONTRACT -- READ THIS BEFORE WRITING ANY INSTRUCTION BODY
 * -------------------------------------------------------------------
 * After a successful `decode()`, the following are valid until the next `decode()`:
 *
 *   opnd[0..nOpnd-1]  The resolved operand queue, in specifier order.  What a specifier
 *                     contributes depends on its DECODE ROM ACCESS TYPE, not on its addressing
 *                     mode -- this is the single most important thing to get right:
 *
 *                       r.bwl    opnd[j]        = the operand's VALUE
 *                       r.q      opnd[j:j+1]    = value, low longword first
 *                       r.o      opnd[j:j+3]    = value, low longword first
 *                       m.bwl    opnd[j]        = the operand's VALUE (read with write access)
 *                       m.q      opnd[j:j+1]    = value, low longword first
 *                       m.o      opnd[j:j+3]    = value, low longword first
 *                       a.bwlqo  opnd[j]        = the operand's ADDRESS
 *                       w.bwlqo  opnd[j]        = OP_MEM (-1) if the destination is memory,
 *                                                 otherwise the REGISTER NUMBER 0-15
 *                                opnd[j+1]      = the memory address, or R[rn] for a register
 *                                                 destination (in which case it is not an
 *                                                 address and must not be used as one)
 *
 *                     So a `w` specifier occupies TWO queue slots and an `a` specifier ONE.
 *                     Byte and word values are ZERO-EXTENDED (0..0xFF, 0..0xFFFF); longwords
 *                     are signed int32 and may be negative.  Sign-extend in the instruction
 *                     body, exactly as SIMH does.
 *
 *   spec, rn, va      State of the LAST specifier decoded, which is what an instruction body
 *                     stores its result through -- see isMemoryDestination() for the exact rule
 *                     and for the store itself, which is Wave 2's, not this module's.  `va` is a
 *                     signed int32 virtual address: coerce with `>>> 0` before comparing or
 *                     formatting it.
 *
 *   brdisp            For BB/BW opcodes, the branch displacement as fetched -- ZERO-extended,
 *                     not sign-extended.  SIMH sign-extends in the branch body (SXTB/SXTW) and
 *                     so must you.  Control-flow work (pcjsvax-fab) owns that.
 *
 *   vfldrp1           R[(rn+1) & 15] captured at the time a `.vb` (variable bit field) register
 *                     specifier was decoded, for the field instructions that read a 64-bit
 *                     register pair.
 *
 *   recq[0..recqptr-1]  The recovery queue.  See FAULTS below.
 *
 * FAULTS AND REGISTER-SIDE-EFFECT RECOVERY -- THE PART THAT IS EASY TO GET WRONG
 * -----------------------------------------------------------------------------
 * Autoincrement and autodecrement specifiers modify registers DURING resolution.  If a LATER
 * specifier of the same instruction faults, the architecture requires the instruction to be
 * restartable, so those register modifications must be UNDONE before the exception is taken.
 * SIMH does this with `recq[]` and an unwind loop in its abort handler (vax_cpu.c:533-541).
 *
 * We throw a `VAXFault` instead of longjmp'ing.  The CPU's catch handler MUST, for any fault
 * that is not a simulator stop:
 *
 *      if (!(psl & PSL_FPD)) decoder.unwind();      // undo autoinc/autodec side effects
 *      decoder.resetRecovery();                     // then clear the queue
 *      // ... restore PC to fault_PC and dispatch the exception
 *
 * The FPD (first-part-done) guard is not optional: an instruction that has already made
 * externally visible progress is resumed, not restarted, so its side effects must stand.
 *
 * The PC is NOT part of the recovery queue.  Autoincrement on R15 advances the PC via setPC()
 * and pushes nothing, because the CPU restores the PC to fault_PC wholesale when it takes the
 * exception.  Do not "fix" that.
 *
 * ADDRESS CONVENTION
 * ------------------
 * See defines.js.  Every effective address computed here is a full 32-bit VIRTUAL address kept
 * as a signed int32: there is no mask that makes an S0 address non-negative, so `>>` is never
 * used on one, and consumers must apply `>>> 0` before a relational compare, an index, or
 * formatting.  Arithmetic that produces an address is coerced back to int32 with `| 0` at the
 * point of computation, which is what makes the wraparound match SIMH's int32 C arithmetic.
 * ============================================================================
 */

import { DROM, DROM_STRIDE, DR, IG, SPEC, MODE, RQ } from "./drom.js";

/**
 * Fault codes, matching Open SIMH's ABORT_* values (the negated SCB vector offset, vax_defs.h:
 * 71-79) so that the CPU's exception dispatcher can use them directly as SCB offsets.
 */
const VAXFAULT = {
    RESIN:      -0x10,          // reserved or privileged instruction
    RESOP:      -0x18,          // reserved operand
    RESAD:      -0x1C           // reserved addressing mode
};

/**
 * Operand-queue marker meaning "this write destination is memory, and its address is in the next
 * queue slot".  SIMH's OP_MEM is 0xFFFFFFFF; as a signed int32, and therefore as stored in an
 * Int32Array, that is -1.  Compare against OP_MEM, never against 0xFFFFFFFF.
 */
const OP_MEM = -1;

const OPND_SIZE = 16;           // vax_defs.h:853 -- max operand queue entries (6 specifiers x octa)
const RECQ_SIZE = 6;            // vax_cpu.c:255 -- one entry per specifier, at most

const nAP = 12, nSP = 14, nPC = 15;

/**
 * A fault raised during decode or operand resolution.  Replaces SIMH's `ABORT()`/longjmp.
 * `code` is one of VAXFAULT.* (or, when thrown by the memory interface, the corresponding
 * negated SCB offset for ACV/TNV/machine check).
 *
 * @class VAXFault
 */
class VAXFault extends Error {
    /**
     * @param {number} code
     * @param {number} [p1]
     * @param {number} [p2]
     */
    constructor(code, p1 = 0, p2 = 0)
    {
        super("VAX fault " + code);
        this.name = "VAXFault";
        this.code = code;
        this.p1 = p1;
        this.p2 = p2;
    }
}

/**
 * The machine services the decoder needs.  Implemented by the CPU; stubbed by test harnesses.
 * Kept deliberately small, because every method here is a place the MMU (pcjsvax-061) will
 * eventually plug in.
 *
 * @typedef {Object} VAXDecodeMachine
 * @property {Int32Array} regs                  R0-R15; regs[15] is the PC
 * @property {function(number): number} getISTR  fetch 1/2/4 bytes from the instruction stream at
 *                                               the PC and advance it; byte and word results are
 *                                               zero-extended, a longword is signed int32
 * @property {function(number): void} setPC      set the PC and flush any instruction prefetch
 * @property {function(number, number, boolean): number} readData
 *                                               read `lnt` bytes at virtual address `va`; the
 *                                               third argument is TRUE when the access must be
 *                                               checked for WRITE permission (SIMH's WA) rather
 *                                               than read (SIMH's RA) -- which is what a `modify`
 *                                               specifier requires.  Byte/word zero-extended.
 */

/**
 * @class VAXDecoder
 * @property {VAXDecodeMachine} cpu
 * @property {number} opc            opcode of the instruction just decoded (0x000-0x1FF)
 * @property {number} numspec        number of specifiers the decode ROM declares for it
 * @property {Int32Array} opnd       resolved operand queue
 * @property {number} nOpnd          number of valid entries in opnd (SIMH's `j`)
 * @property {number} spec           last specifier byte decoded
 * @property {number} rn             register number of the last specifier decoded
 * @property {number} va             effective address of the last memory specifier (int32)
 * @property {number} brdisp         branch displacement, zero-extended
 * @property {number} vfldrp1        R[(rn+1)&15] captured for a .vb register specifier
 * @property {Int32Array} recq       recovery queue
 * @property {number} recqptr        number of valid entries in recq
 */
class VAXDecoder {
    /**
     * @param {VAXDecodeMachine} cpu
     */
    constructor(cpu)
    {
        this.cpu = cpu;
        this.opnd = new Int32Array(OPND_SIZE);
        this.recq = new Int32Array(RECQ_SIZE);
        this.opc = 0;
        this.numspec = 0;
        this.nOpnd = 0;
        this.spec = 0;
        this.rn = 0;
        this.va = 0;
        this.brdisp = 0;
        this.vfldrp1 = 0;
        this.recqptr = 0;
        /*
         * Instruction groups this CPU implements, as a bit per DR_GETIGRP value.  For a
         * microvax3900 build SIMH's CPU_INSTRUCTION_SET resolves to VAX_FULL_BASE
         * (vax_defs.h:1004-1009): the base group plus both floating subgroups.  Packed decimal,
         * extended accuracy and emulated-only are NOT here -- but note carefully that their
         * absence is not what makes them fault, and they are not rejected during decode at all.
         * See checkInstructionGroup().
         */
        this.instructionSet = (1 << IG.BASE) | (1 << IG.BSGFL) | (1 << IG.BSDFL);
    }

    /**
     * fault(code)
     *
     * SIMH's ABORT().  A throw, not a longjmp -- see the FAULTS section of the file header for
     * what the CPU's catch handler owes the decoder.
     *
     * @param {number} code
     * @throws {VAXFault}
     */
    fault(code)
    {
        throw new VAXFault(code);
    }

    /**
     * fetchOpcode()
     *
     * SIMH vax_cpu.c:731-734.  One byte, or two when the first is 0xFD, in which case the second
     * byte is flagged into bit 8 so that the FD-prefixed opcodes occupy DROM slots 0x100-0x1FF.
     *
     * @returns {number} 0x000-0x1FF
     */
    fetchOpcode()
    {
        let opc = this.cpu.getISTR(1);
        if (opc == 0xFD) {
            opc = this.cpu.getISTR(1) | 0x100;
        }
        return opc;
    }

    /**
     * checkInstructionGroup(opc)
     *
     * SIMH vax_cpu.c:736-742, the `#if !defined(FULL_VAX)` branch, which is the branch a
     * microvax3900 build compiles.  An opcode belonging to a group this CPU does not implement --
     * or explicitly reserved -- raises a reserved-instruction fault so the operating system can
     * emulate it.  This happens BEFORE any specifier is touched, which is why the decode ROM
     * reports zero decodable specifiers for those opcodes.
     *
     * WHAT IS *NOT* CHECKED HERE, AND WHY IT MATTERS.  SIMH tests exactly three conditions: the
     * D_float subgroup when D_float is absent, the G_float subgroup when G_float is absent, and
     * an explicitly reserved opcode.  It does NOT test for packed decimal, extended accuracy or
     * emulated-only.  Those opcodes decode normally and are dispatched to the operating system's
     * emulator from the instruction BODY -- and because vax_sys.c wraps their decode ROM header
     * in ODC(), they declare ZERO decodable specifiers, so this decoder correctly resolves
     * nothing for them and leaves the operand count in DR_GETUSP for the emulator interface.
     * Faulting them here instead would consume no specifier bytes and break the PC.
     *
     * @param {number} opc
     */
    checkInstructionGroup(opc)
    {
        let hdr = DROM[opc * DROM_STRIDE];
        let group = (hdr >> DR.V_IGMASK) & DR.M_IGMASK;
        if ((group == IG.BSDFL && !(this.instructionSet & (1 << IG.BSDFL))) ||
            (group == IG.BSGFL && !(this.instructionSet & (1 << IG.BSGFL))) ||
            (group == IG.RSVD)) {
            this.fault(VAXFAULT.RESIN);
        }
    }

    /**
     * decode(fFPD)
     *
     * Fetch the opcode and resolve its operand specifiers.  Equivalent to vax_cpu.c:731-820 plus
     * the specifier flow.  When first-part-done is set the instruction is being RESUMED, so no
     * specifier is re-resolved (and an opcode that is not restartable faults).
     *
     * @param {boolean} fFPD    state of PSL<fpd>
     * @returns {number} the opcode
     */
    decode(fFPD)
    {
        this.recqptr = 0;
        this.nOpnd = 0;
        this.brdisp = 0;
        let opc = this.opc = this.fetchOpcode();
        this.checkInstructionGroup(opc);
        let hdr = DROM[opc * DROM_STRIDE];
        if (fFPD) {
            if (!(hdr & DR.F)) this.fault(VAXFAULT.RESIN);
            this.numspec = 0;
            return opc;
        }
        this.numspec = hdr & DR.NSPMASK;
        this.decodeSpecifiers(opc);
        return opc;
    }

    /**
     * unwind()
     *
     * SIMH vax_cpu.c:533-541.  Undo every register modification the addressing modes of the
     * faulting instruction made, so the instruction can be restarted.  RQ_DIR distinguishes an
     * increment (undo by subtracting) from a decrement (undo by adding).
     *
     * Call this from the CPU's fault handler, and ONLY when PSL<fpd> is clear.
     */
    unwind()
    {
        let regs = this.cpu.regs;
        for (let i = 0; i < this.recqptr; i++) {
            let e = this.recq[i];
            let rrn = e & RQ.RN;
            let rlnt = 1 << ((e >> RQ.V_LNT) & RQ.M_LNT);
            if (e & RQ.DIR) {
                regs[rrn] = regs[rrn] - rlnt;
            } else {
                regs[rrn] = regs[rrn] + rlnt;
            }
        }
    }

    /**
     * resetRecovery()
     *
     * Clear the recovery queue.  SIMH does this both after unwinding and at the top of every
     * instruction (vax_cpu.c:546, 657).
     */
    resetRecovery()
    {
        this.recqptr = 0;
    }

    /**
     * recordRecovery(disp, rn)
     *
     * SIMH's `recq[recqptr++] = RQ_REC (disp, rn)`.  `disp` is the MERGED descriptor (addressing
     * mode nibble | decode ROM access/length), so bit 7 of the mode -- set for autoincrement,
     * autoincrement deferred and the displacement modes, clear for autodecrement -- lands on
     * RQ_DIR, and the length code lands in RQ_M_LNT.
     *
     * @param {number} disp
     * @param {number} rn
     */
    recordRecovery(disp, rn)
    {
        this.recq[this.recqptr++] = (disp << RQ.V_LNT) | rn;
    }

    /**
     * readOcta(va, fWrite)
     *
     * SIMH ReadOcta (vax_cpu.c:3237): four longwords, low first.
     *
     * @param {number} va
     * @param {boolean} fWrite
     */
    readOcta(va, fWrite)
    {
        let cpu = this.cpu, opnd = this.opnd, j = this.nOpnd;
        opnd[j++] = cpu.readData(va, 4, fWrite);
        opnd[j++] = cpu.readData((va + 4) | 0, 4, fWrite);
        opnd[j++] = cpu.readData((va + 8) | 0, 4, fWrite);
        opnd[j++] = cpu.readData((va + 12) | 0, 4, fWrite);
        this.nOpnd = j;
    }

    /**
     * decodeSpecifiers(opc)
     *
     * The specifier flow, vax_cpu.c:781-1560.
     *
     * STRUCTURAL NOTE.  SIMH writes this as ONE switch over `disp = (spec & ~RGMASK) | drom_disp`
     * with ~250 case labels and heavy deliberate fall-through.  We split it into an outer switch
     * on the addressing-mode nibble and an inner switch on the decode ROM descriptor.  That is
     * not a behavioural change and the equivalence is exact: `spec & ~RGMASK` contributes only
     * bits <7:4> (the specifier byte's mode nibble, since spec is a byte), and a decode ROM
     * descriptor contributes only bits <9:8> (access type), <3> (DR_SPFLAG) and <2:0> (length).
     * The two fields are disjoint, so `disp` is exactly `mode | at` and switching on the pair is
     * switching on `disp`.  SIMH itself relies on that same disjointness in its index case, where
     * it re-derives the access/length half as `disp & (DR_ACMASK|DR_SPFLAG|DR_LNMASK)`.
     *
     * Any (mode, descriptor) pair with no case falls to a reserved addressing fault, matching the
     * single `default:` at the end of SIMH's switch.
     *
     * @param {number} opc
     */
    decodeSpecifiers(opc)
    {
        let cpu = this.cpu, regs = cpu.regs, opnd = this.opnd;
        let base = opc * DROM_STRIDE;

        for (let i = 1; i <= this.numspec; i++) {
            let at = DROM[base + i];
            if (at >= SPEC.BB) {
                /*
                 * A branch displacement is not an addressing-mode specifier at all: it is a
                 * literal field in the instruction format.  `at & 1` is 0 for BB and 1 for BW, so
                 * DR_LNT() of it yields 1 or 2 bytes.  It is ZERO-extended here; the branch body
                 * sign-extends.  BB/BW is always the last specifier, hence the break.
                 */
                this.brdisp = cpu.getISTR(1 << (at & 1));
                break;
            }
            let spec = this.spec = cpu.getISTR(1);
            let rn = this.rn = spec & 0xF;
            let mode = spec & 0xF0;
            let disp = mode | at;               // SIMH's `disp`, exactly
            let lnt = 1 << (at & DR.LNMASK);    // SIMH's DR_LNT(disp)
            let j = this.nOpnd;
            let va, iad, temp, index;

            switch (mode) {

            /* -------------------------------------------------------------- Short literal */
            case MODE.SH0:
            case MODE.SH1:
            case MODE.SH2:
            case MODE.SH3:
                /*
                 * Only read access is permitted; every other access type falls through to the
                 * reserved addressing fault below.  The float cases synthesise a small positive
                 * F/D/G/H value directly from the 6-bit literal.
                 */
                switch (at) {
                case SPEC.RB: case SPEC.RW: case SPEC.RL:
                    opnd[j++] = spec;
                    break;
                case SPEC.RQ:
                    opnd[j++] = spec;
                    opnd[j++] = 0;
                    break;
                case SPEC.RO:
                    opnd[j++] = spec;
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    break;
                case SPEC.RF:
                    opnd[j++] = (spec << 4) | 0x4000;
                    break;
                case SPEC.RD:
                    opnd[j++] = (spec << 4) | 0x4000;
                    opnd[j++] = 0;
                    break;
                case SPEC.RG:
                    opnd[j++] = (spec << 1) | 0x4000;
                    opnd[j++] = 0;
                    break;
                case SPEC.RH:
                    opnd[j++] = ((spec & 0x7) << 29) | (0x4000 | ((spec >> 3) & 0x7));
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    opnd[j++] = 0;
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* ------------------------------------------------------------------- Register */
            case MODE.GRN:
                switch (at) {
                case SPEC.RB: case SPEC.MB:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = regs[rn] & 0xFF;
                    break;
                case SPEC.RW: case SPEC.MW:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = regs[rn] & 0xFFFF;
                    break;
                case SPEC.VB:
                    this.vfldrp1 = regs[(rn + 1) & 0xF];
                    /* falls through */
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    /*
                     * A register write destination: opnd[j] is the register NUMBER (not OP_MEM)
                     * and opnd[j+1] is its current contents.  Note that WQ and WO push only the
                     * one longword; the high half is R[rn+1] and the instruction body reaches it
                     * through rn.  That is SIMH's behaviour, deliberately preserved.
                     */
                    opnd[j++] = rn;
                    /* falls through */
                case SPEC.RL: case SPEC.RF: case SPEC.ML:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = regs[rn];
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG: case SPEC.MQ:
                    if (rn >= nSP) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = regs[rn];
                    opnd[j++] = regs[rn + 1];
                    break;
                case SPEC.RO: case SPEC.RH: case SPEC.MO:
                    if (rn >= nAP) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = regs[rn];
                    opnd[j++] = regs[rn + 1];
                    opnd[j++] = regs[rn + 2];
                    opnd[j++] = regs[rn + 3];
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* ------------------------------------------------ Register deferred -- (Rn) */
            case MODE.RGD:
                switch (at) {
                case SPEC.VB:
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    opnd[j++] = OP_MEM;
                    /* falls through */
                case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    va = opnd[j++] = regs[rn];
                    break;
                case SPEC.RB: case SPEC.RW: case SPEC.RL: case SPEC.RF:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], lnt, false);
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], 4, false);
                    opnd[j++] = cpu.readData((regs[rn] + 4) | 0, 4, false);
                    break;
                case SPEC.RO: case SPEC.RH:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    this.nOpnd = j;
                    this.readOcta(va = regs[rn], false);
                    j = this.nOpnd;
                    break;
                case SPEC.MB: case SPEC.MW: case SPEC.ML:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], lnt, true);
                    break;
                case SPEC.MQ:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], 4, true);
                    opnd[j++] = cpu.readData((regs[rn] + 4) | 0, 4, true);
                    break;
                case SPEC.MO:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    this.nOpnd = j;
                    this.readOcta(va = regs[rn], true);
                    j = this.nOpnd;
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* ------------------------------------------------- Autodecrement -- -(Rn) */
            case MODE.ADC:
                /*
                 * ORDER MATTERS AND IS NOT UNIFORM.  For the address/write forms SIMH checks for
                 * PC BEFORE decrementing; for the read/modify forms it decrements FIRST and
                 * checks after, which means `-(PC)` decrements R15 and only then faults -- and
                 * the recovery-queue entry pushed just before the fault is what puts it back.
                 * That asymmetry is transcribed exactly; do not "regularise" it.
                 */
                switch (at) {
                case SPEC.VB:
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    opnd[j++] = OP_MEM;
                    /* falls through */
                case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    regs[rn] = (regs[rn] - lnt) | 0;
                    va = opnd[j++] = regs[rn];
                    this.recordRecovery(disp, rn);
                    break;
                case SPEC.RB: case SPEC.RW: case SPEC.RL: case SPEC.RF:
                    regs[rn] = (regs[rn] - lnt) | 0;
                    this.recordRecovery(disp, rn);
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], lnt, false);
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG:
                    regs[rn] = (regs[rn] - 8) | 0;
                    this.recordRecovery(disp, rn);
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], 4, false);
                    opnd[j++] = cpu.readData((regs[rn] + 4) | 0, 4, false);
                    break;
                case SPEC.RO: case SPEC.RH:
                    regs[rn] = (regs[rn] - 16) | 0;
                    this.recordRecovery(disp, rn);
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    this.nOpnd = j;
                    this.readOcta(va = regs[rn], false);
                    j = this.nOpnd;
                    break;
                case SPEC.MB: case SPEC.MW: case SPEC.ML:
                    regs[rn] = (regs[rn] - lnt) | 0;
                    this.recordRecovery(disp, rn);
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], lnt, true);
                    break;
                case SPEC.MQ:
                    regs[rn] = (regs[rn] - 8) | 0;
                    this.recordRecovery(disp, rn);
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    opnd[j++] = cpu.readData(va = regs[rn], 4, true);
                    opnd[j++] = cpu.readData((regs[rn] + 4) | 0, 4, true);
                    break;
                case SPEC.MO:
                    regs[rn] = (regs[rn] - 16) | 0;
                    this.recordRecovery(disp, rn);
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    this.nOpnd = j;
                    this.readOcta(va = regs[rn], true);
                    j = this.nOpnd;
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* ------------------------------------ Autoincrement -- (Rn)+, and (PC)+ = immediate */
            case MODE.AIN:
                switch (at) {
                case SPEC.VB:
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    opnd[j++] = OP_MEM;
                    /* falls through */
                case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
                    va = opnd[j++] = regs[rn];
                    if (rn == nPC) {
                        cpu.setPC((regs[nPC] + lnt) | 0);
                    } else {
                        regs[rn] = (regs[rn] + lnt) | 0;
                        this.recordRecovery(disp, rn);
                    }
                    break;
                case SPEC.RB: case SPEC.RW: case SPEC.RL: case SPEC.RF:
                    va = regs[rn];
                    if (rn == nPC) {
                        opnd[j++] = cpu.getISTR(lnt);
                    } else {
                        opnd[j++] = cpu.readData(regs[rn], lnt, false);
                        regs[rn] = (regs[rn] + lnt) | 0;
                        this.recordRecovery(disp, rn);
                    }
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG:
                    va = regs[rn];
                    if (rn == nPC) {
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                    } else {
                        opnd[j++] = cpu.readData(va, 4, false);
                        opnd[j++] = cpu.readData((va + 4) | 0, 4, false);
                        regs[rn] = (regs[rn] + 8) | 0;
                        this.recordRecovery(disp, rn);
                    }
                    break;
                case SPEC.RO: case SPEC.RH:
                    va = regs[rn];
                    if (rn == nPC) {
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                    } else {
                        this.nOpnd = j;
                        this.readOcta(va, false);
                        j = this.nOpnd;
                        regs[rn] = (regs[rn] + 16) | 0;
                        this.recordRecovery(disp, rn);
                    }
                    break;
                case SPEC.MB: case SPEC.MW: case SPEC.ML:
                    va = regs[rn];
                    if (rn == nPC) {
                        opnd[j++] = cpu.getISTR(lnt);
                    } else {
                        opnd[j++] = cpu.readData(regs[rn], lnt, true);
                        regs[rn] = (regs[rn] + lnt) | 0;
                        this.recordRecovery(disp, rn);
                    }
                    break;
                case SPEC.MQ:
                    va = regs[rn];
                    if (rn == nPC) {
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                    } else {
                        opnd[j++] = cpu.readData(va, 4, true);
                        opnd[j++] = cpu.readData((va + 4) | 0, 4, true);
                        regs[rn] = (regs[rn] + 8) | 0;
                        this.recordRecovery(disp, rn);
                    }
                    break;
                case SPEC.MO:
                    va = regs[rn];
                    if (rn == nPC) {
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                        opnd[j++] = cpu.getISTR(4);
                    } else {
                        this.nOpnd = j;
                        this.readOcta(va, true);
                        j = this.nOpnd;
                        regs[rn] = (regs[rn] + 16) | 0;
                        this.recordRecovery(disp, rn);
                    }
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* ------------------------- Autoincrement deferred -- @(Rn)+, and @(PC)+ = absolute */
            case MODE.AID:
                /*
                 * The register is always incremented by FOUR here regardless of the operand
                 * length, because what is being autoincremented is a pointer -- hence the fixed
                 * `AID|RL` recovery descriptor rather than `disp`.
                 */
                switch (at) {
                case SPEC.VB:
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    opnd[j++] = OP_MEM;
                    /* falls through */
                case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
                    if (rn == nPC) {
                        va = opnd[j++] = cpu.getISTR(4);
                    } else {
                        va = opnd[j++] = cpu.readData(regs[rn], 4, false);
                        regs[rn] = (regs[rn] + 4) | 0;
                        this.recordRecovery(MODE.AID | SPEC.RL, rn);
                    }
                    break;
                case SPEC.RB: case SPEC.RW: case SPEC.RL: case SPEC.RF:
                    va = this.resolveDeferredPointer(rn);
                    opnd[j++] = cpu.readData(va, lnt, false);
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG:
                    va = this.resolveDeferredPointer(rn);
                    opnd[j++] = cpu.readData(va, 4, false);
                    opnd[j++] = cpu.readData((va + 4) | 0, 4, false);
                    break;
                case SPEC.RO: case SPEC.RH:
                    va = this.resolveDeferredPointer(rn);
                    this.nOpnd = j;
                    this.readOcta(va, false);
                    j = this.nOpnd;
                    break;
                case SPEC.MB: case SPEC.MW: case SPEC.ML:
                    va = this.resolveDeferredPointer(rn);
                    opnd[j++] = cpu.readData(va, lnt, true);
                    break;
                case SPEC.MQ:
                    va = this.resolveDeferredPointer(rn);
                    opnd[j++] = cpu.readData(va, 4, true);
                    opnd[j++] = cpu.readData((va + 4) | 0, 4, true);
                    break;
                case SPEC.MO:
                    va = this.resolveDeferredPointer(rn);
                    this.nOpnd = j;
                    this.readOcta(va, true);
                    j = this.nOpnd;
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* --------------------------------------------------- Displacement -- d(Rn) */
            case MODE.BDP:
            case MODE.WDP:
            case MODE.LDP:
                /*
                 * PC-relative addressing is legal, so unlike the register-based modes above there
                 * is deliberately NO check for R15 here.  Byte and word displacements are
                 * SIGN-extended; a longword displacement needs no extension.
                 */
                temp = this.fetchDisplacement(mode);
                switch (at) {
                case SPEC.VB:
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    opnd[j++] = OP_MEM;
                    /* falls through */
                case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
                    va = opnd[j++] = (regs[rn] + temp) | 0;
                    break;
                case SPEC.RB: case SPEC.RW: case SPEC.RL: case SPEC.RF:
                    va = (regs[rn] + temp) | 0;
                    opnd[j++] = cpu.readData(va, lnt, false);
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG:
                    va = (regs[rn] + temp) | 0;
                    opnd[j++] = cpu.readData(va, 4, false);
                    opnd[j++] = cpu.readData((va + 4) | 0, 4, false);
                    break;
                case SPEC.RO: case SPEC.RH:
                    va = (regs[rn] + temp) | 0;
                    this.nOpnd = j;
                    this.readOcta(va, false);
                    j = this.nOpnd;
                    break;
                case SPEC.MB: case SPEC.MW: case SPEC.ML:
                    va = (regs[rn] + temp) | 0;
                    opnd[j++] = cpu.readData(va, lnt, true);
                    break;
                case SPEC.MQ:
                    va = (regs[rn] + temp) | 0;
                    opnd[j++] = cpu.readData(va, 4, true);
                    opnd[j++] = cpu.readData((va + 4) | 0, 4, true);
                    break;
                case SPEC.MO:
                    va = (regs[rn] + temp) | 0;
                    this.nOpnd = j;
                    this.readOcta(va, true);
                    j = this.nOpnd;
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* ------------------------------------------ Displacement deferred -- @d(Rn) */
            case MODE.BDD:
            case MODE.WDD:
            case MODE.LDD:
                temp = this.fetchDisplacement(mode);
                iad = (regs[rn] + temp) | 0;
                switch (at) {
                case SPEC.VB:
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    opnd[j++] = OP_MEM;
                    /* falls through */
                case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
                    va = opnd[j++] = cpu.readData(iad, 4, false);
                    break;
                case SPEC.RB: case SPEC.RW: case SPEC.RL: case SPEC.RF:
                    va = cpu.readData(iad, 4, false);
                    opnd[j++] = cpu.readData(va, lnt, false);
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG:
                    va = cpu.readData(iad, 4, false);
                    opnd[j++] = cpu.readData(va, 4, false);
                    opnd[j++] = cpu.readData((va + 4) | 0, 4, false);
                    break;
                case SPEC.RO: case SPEC.RH:
                    va = cpu.readData(iad, 4, false);
                    this.nOpnd = j;
                    this.readOcta(va, false);
                    j = this.nOpnd;
                    break;
                case SPEC.MB: case SPEC.MW: case SPEC.ML:
                    va = cpu.readData(iad, 4, false);
                    opnd[j++] = cpu.readData(va, lnt, true);
                    break;
                case SPEC.MQ:
                    va = cpu.readData(iad, 4, false);
                    opnd[j++] = cpu.readData(va, 4, true);
                    opnd[j++] = cpu.readData((va + 4) | 0, 4, true);
                    break;
                case SPEC.MO:
                    va = cpu.readData(iad, 4, false);
                    this.nOpnd = j;
                    this.readOcta(va, true);
                    j = this.nOpnd;
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            /* ---------------------------------------------------- Index -- base[Rx] */
            case MODE.IDX:
                /*
                 * The index register is scaled by the OPERAND length, then a second, complete
                 * specifier byte is decoded to produce the base address.  Two things bite here
                 * and both are exercised deliberately by the test harness:
                 *
                 *   1. `rn` is REASSIGNED to the base specifier's register.  If the base mode is
                 *      autoincrement or autodecrement on the SAME register that was just used as
                 *      the index, the index contribution has already been captured from the
                 *      pre-modification value -- and it must stay that way.
                 *   2. The base's own autodecrement uses the OUTER operand length, not the base
                 *      specifier's, which is why the recovery descriptor is rebuilt from
                 *      `disp & DR_LNMASK` rather than reused.
                 *
                 * Index on the PC as index register, and `base[Rx]` with an immediate base, are
                 * both reserved addressing faults.
                 */
                if (rn == nPC) this.fault(VAXFAULT.RESAD);
                index = regs[rn] << (at & DR.LNMASK);
                spec = this.spec = cpu.getISTR(1);
                rn = this.rn = spec & 0xF;
                switch (spec & 0xF0) {
                case MODE.ADC:
                    regs[rn] = (regs[rn] - lnt) | 0;
                    this.recordRecovery(MODE.ADC | (at & DR.LNMASK), rn);
                    /* falls through */
                case MODE.RGD:
                    if (rn == nPC) this.fault(VAXFAULT.RESAD);
                    index = (index + regs[rn]) | 0;
                    break;
                case MODE.AIN:
                    index = (index + regs[rn]) | 0;
                    if (rn == nPC) {
                        this.fault(VAXFAULT.RESAD);     // SIMH IDX_IMM_TEST
                    } else {
                        regs[rn] = (regs[rn] + lnt) | 0;
                        this.recordRecovery(MODE.AIN | (at & DR.LNMASK), rn);
                    }
                    break;
                case MODE.AID:
                    if (rn == nPC) {
                        temp = cpu.getISTR(4);
                    } else {
                        temp = cpu.readData(regs[rn], 4, false);
                        regs[rn] = (regs[rn] + 4) | 0;
                        this.recordRecovery(MODE.AID | SPEC.RL, rn);
                    }
                    index = (temp + index) | 0;
                    break;
                case MODE.BDP:
                    temp = cpu.getISTR(1);
                    index = (index + regs[rn] + ((temp & 0x80) ? (temp | ~0xFF) : temp)) | 0;
                    break;
                case MODE.BDD:
                    temp = cpu.getISTR(1);
                    index = (index + cpu.readData((regs[rn] + ((temp & 0x80) ? (temp | ~0xFF) : temp)) | 0, 4, false)) | 0;
                    break;
                case MODE.WDP:
                    temp = cpu.getISTR(2);
                    index = (index + regs[rn] + ((temp & 0x8000) ? (temp | ~0xFFFF) : temp)) | 0;
                    break;
                case MODE.WDD:
                    temp = cpu.getISTR(2);
                    index = (index + cpu.readData((regs[rn] + ((temp & 0x8000) ? (temp | ~0xFFFF) : temp)) | 0, 4, false)) | 0;
                    break;
                case MODE.LDP:
                    temp = cpu.getISTR(4);
                    index = (index + regs[rn] + temp) | 0;
                    break;
                case MODE.LDD:
                    temp = cpu.getISTR(4);
                    index = (index + cpu.readData((regs[rn] + temp) | 0, 4, false)) | 0;
                    break;
                default:
                    /* short literal, register, or a second index prefix: all reserved */
                    this.fault(VAXFAULT.RESAD);
                }
                switch (at) {
                case SPEC.VB:
                case SPEC.WB: case SPEC.WW: case SPEC.WL: case SPEC.WQ: case SPEC.WO:
                    opnd[j++] = OP_MEM;
                    /* falls through */
                case SPEC.AB: case SPEC.AW: case SPEC.AL: case SPEC.AQ: case SPEC.AO:
                    va = opnd[j++] = index;
                    break;
                case SPEC.RB: case SPEC.RW: case SPEC.RL: case SPEC.RF:
                    opnd[j++] = cpu.readData(va = index, lnt, false);
                    break;
                case SPEC.RQ: case SPEC.RD: case SPEC.RG:
                    opnd[j++] = cpu.readData(va = index, 4, false);
                    opnd[j++] = cpu.readData((index + 4) | 0, 4, false);
                    break;
                case SPEC.RO: case SPEC.RH:
                    this.nOpnd = j;
                    this.readOcta(va = index, false);
                    j = this.nOpnd;
                    break;
                case SPEC.MB: case SPEC.MW: case SPEC.ML:
                    opnd[j++] = cpu.readData(va = index, lnt, true);
                    break;
                case SPEC.MQ:
                    opnd[j++] = cpu.readData(va = index, 4, true);
                    opnd[j++] = cpu.readData((index + 4) | 0, 4, true);
                    break;
                case SPEC.MO:
                    this.nOpnd = j;
                    this.readOcta(va = index, true);
                    j = this.nOpnd;
                    break;
                default:
                    this.fault(VAXFAULT.RESAD);
                }
                break;

            default:
                this.fault(VAXFAULT.RESAD);
            }

            this.nOpnd = j;
            if (va !== undefined) this.va = va;
        }
    }

    /**
     * resolveDeferredPointer(rn)
     *
     * The common head of every autoincrement-deferred read/modify case: fetch the pointer, and
     * unless it came from the instruction stream (absolute mode, `@#addr`), bump the register by
     * four and record the recovery entry.
     *
     * @param {number} rn
     * @returns {number} the effective address
     */
    resolveDeferredPointer(rn)
    {
        let cpu = this.cpu, regs = cpu.regs, va;
        if (rn == nPC) {
            va = cpu.getISTR(4);
        } else {
            va = cpu.readData(regs[rn], 4, false);
            regs[rn] = (regs[rn] + 4) | 0;
            this.recordRecovery(MODE.AID | SPEC.RL, rn);
        }
        return va;
    }

    /**
     * fetchDisplacement(mode)
     *
     * Byte and word displacements are sign-extended (SIMH SXTB/SXTW); a longword displacement is
     * already a signed int32.
     *
     * @param {number} mode     one of MODE.BDP/BDD/WDP/WDD/LDP/LDD
     * @returns {number} signed int32 displacement
     */
    fetchDisplacement(mode)
    {
        let temp;
        if (mode == MODE.BDP || mode == MODE.BDD) {
            temp = this.cpu.getISTR(1);
            return (temp & 0x80) ? (temp | ~0xFF) : temp;
        }
        if (mode == MODE.WDP || mode == MODE.WDD) {
            temp = this.cpu.getISTR(2);
            return (temp & 0x8000) ? (temp | ~0xFFFF) : temp;
        }
        return this.cpu.getISTR(4);
    }

    /**
     * isMemoryDestination()
     *
     * Whether the LAST specifier decoded named a memory location rather than a register -- the
     * test every instruction body needs before storing its result.  SIMH spells it inline in its
     * WRITE_B/WRITE_W/WRITE_L/WRITE_Q macros (vax_cpu.c:210-231) as `spec > (GRN | nPC)`, i.e. a
     * specifier byte above 0x5F.  It is exact: a modify specifier is always last, a short literal
     * is illegal as a modify or write specifier, and an index specifier leaves `spec` holding its
     * BASE specifier byte, which is always 0x60 or above.
     *
     * The store ITSELF is instruction execution and is deliberately not implemented here.  When
     * you write one, it is:
     *
     *      byte:   if (decoder.isMemoryDestination()) write(decoder.va, r, 1);
     *              else regs[decoder.rn] = (regs[decoder.rn] & ~0xFF) | (r & 0xFF);
     *      word:   ... & ~0xFFFF ... | (r & 0xFFFF)
     *      long:   ... else regs[decoder.rn] = r;
     *      quad:   two longword writes at `va` and `va + 4`, or R[rn] and R[rn+1] with rn < 14
     *              (a reserved operand fault otherwise).  SIMH additionally probes write access
     *              at `va + 7` BEFORE storing the low longword, so a quadword straddling into an
     *              inaccessible page faults with nothing yet written and the instruction stays
     *              restartable.  That probe needs the MMU (pcjsvax-061); do not skip it.
     *
     * @returns {boolean}
     */
    isMemoryDestination()
    {
        return this.spec > (MODE.GRN | nPC);
    }
}

export default VAXDecoder;
export { VAXDecoder, VAXFault, VAXFAULT, OP_MEM, OPND_SIZE, RECQ_SIZE };
