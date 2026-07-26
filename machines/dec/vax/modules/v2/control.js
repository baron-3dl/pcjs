/**
 * @fileoverview VAX branch, jump, call, and stack instruction execution
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
 * WHAT THIS IS
 * ============================================================================
 * Instruction EXECUTION (not decode) for the VAX control-flow and stack-manipulation opcodes:
 * simple branches, branch-on-bit, the loop-control family (SOB/AOB/ACB), CASE, JMP/JSB/RSB,
 * CALLS/CALLG/RET, and the stack instructions (PUSHR/POPR/PUSHL/PUSHAx).  It consumes exactly the
 * decoder's published contract -- `opnd[]`, `spec`/`rn`/`va` of the last specifier, `brdisp` -- and
 * nothing else; see decode.js's file header and machines/dec/vax/README.md for that contract.
 *
 * This is a direct port of the corresponding cases in Open SIMH `vax_cpu.c` (the "Control
 * instructions" section, `sim_instr()`) and `vax_cpu1.c` (`op_bb_n`, `op_bb_x`, `op_call`,
 * `op_ret`, `op_pushr`, `op_popr`).  Ported line-for-line where SIMH's own comments already
 * document a subtlety; deviations are called out explicitly below.
 *
 * Structured as a dispatch table of functions keyed by numeric opcode (`CONTROL_OPCODES`), built
 * from `OPCODES` (drom.js) at module load, rather than SIMH's single ~2500-line switch -- V8
 * prefers a monomorphic call site per opcode over one megamorphic switch.
 *
 * THE CPU INTERFACE THIS MODULE NEEDS
 * ------------------------------------
 * A `cpu` object with:
 *
 *   regs        Int32Array(16); R0-R11 general, regs[12]=AP, regs[13]=FP, regs[14]=SP, regs[15]=PC
 *   psl         current PSL as a signed int32; PSL<3:0> IS the condition-code field (CC.MASK)
 *   readData(va, lnt, fWrite)   identical to the decoder's machine interface (see decode.js);
 *                               byte/word zero-extended, longword signed int32
 *   writeData(va, val, lnt)     store `lnt` bytes of `val` at virtual address `va`
 *   setPC(pc)                   identical to the decoder's machine interface: set PC, flush prefetch
 *   fault(code)                 raise a VAXFault (decoder.fault has this exact signature; a CPU
 *                               typically just delegates to its decoder instance)
 *
 * WHAT IS DELIBERATELY NOT HERE, AND WHY
 * ---------------------------------------
 * - Trap DISPATCH (integer/decimal overflow, trace).  V_ADD_x/V_SUB_x set CC.V unconditionally,
 *   exactly as SIMH does, but SIMH's deferred trap (SET_TRAP, consumed at the top of the NEXT
 *   instruction's fetch, vax_cpu.c:675-707) is a different instruction boundary's problem --
 *   pcjsvax-e49 (SCB exception dispatch) owns it.  Empirically verified against SIMH (single
 *   `step 1` with PSW<IV> set on an overflowing SOBGTR) that a one-instruction step does NOT
 *   perform that dispatch, so CC.V is fully observable and testable here without it.
 * - WRITE_Q / quadword stores.  Every opcode in this file's scope is byte/word/longword-oriented;
 *   none needs a quadword destination.  Do not add one here -- see the README's note on
 *   WRITE_Q's cross-page probe needing the MMU's Test() (pcjsvax-061).
 * - ACBF/ACBD/ACBG/ACBH (floating-point ACB) and PUSHAO.  ACB-on-floating needs a floating ADD,
 *   which is pcjsvax-710's (floating point) scope, not this item's.  PUSHAO is EXTAC-group
 *   (octaword): SIMH's own decode ROM (vax_sys.c, via ODC()) reports ZERO specifiers for it --
 *   it is dispatched to OS software emulation, not executed by the CPU at all, exactly like packed
 *   decimal.  decode.js's checkInstructionGroup() comment documents this same exclusion.
 * - BICPSW/BISPSW/MOVPSL.  These manipulate the PSW directly; they are not branch, jump, call, or
 *   stack instructions, and belong with pcjsvax-e49's privileged/PSW-adjacent scope.
 * ============================================================================
 */

import { OPCODES } from "./drom.js";
import { OP_MEM, VAXFAULT } from "./decode.js";

/*
 * Condition codes.  PSL<3:0>, vax_defs.h:236-240.
 */
const CC = { N: 0x8, Z: 0x4, V: 0x2, C: 0x1, MASK: 0xF };

/*
 * PSW bits (the low byte of PSL above the condition codes), vax_defs.h:231-235.
 */
const PSW = { T: 0x10, IV: 0x20, FU: 0x40, DV: 0x80 };

const BSIGN = 0x80, WSIGN = 0x8000, LSIGN = 0x80000000 | 0;
const BMASK = 0xFF, WMASK = 0xFFFF;

/**
 * SXTB(x) / SXTW(x)
 *
 * vax_defs.h:672-673.
 *
 * @param {number} x
 * @returns {number}
 */
function SXTB(x) { return (x & BSIGN) ? (x | ~BMASK) : (x & BMASK); }
function SXTW(x) { return (x & WSIGN) ? (x | ~WMASK) : (x & WMASK); }

/**
 * getCC(cpu) / setCC(cpu, cc)
 *
 * @param {Object} cpu
 * @returns {number}
 */
function getCC(cpu) { return cpu.psl & CC.MASK; }
function setCC(cpu, cc) { cpu.psl = (cpu.psl & ~CC.MASK) | (cc & CC.MASK); }

/*
 * CC_IIZP_x(cpu, r) -- vax_defs.h:747-758.  Set N or Z per the result's sign/zero-ness, preserve
 * carry, clear V.  Used by every instruction in this file that stores an arithmetic-ish result
 * without itself defining what carry means (SOB/AOB/ACB/PUSHL/PUSHAx).
 */
function ccIIZP_B(cpu, r) { setCC(cpu, (r & BSIGN) ? (CC.N | (getCC(cpu) & CC.C)) : ((r & BMASK) == 0) ? (CC.Z | (getCC(cpu) & CC.C)) : (getCC(cpu) & CC.C)); }
function ccIIZP_W(cpu, r) { setCC(cpu, (r & WSIGN) ? (CC.N | (getCC(cpu) & CC.C)) : ((r & WMASK) == 0) ? (CC.Z | (getCC(cpu) & CC.C)) : (getCC(cpu) & CC.C)); }
function ccIIZP_L(cpu, r) { setCC(cpu, (r & LSIGN) ? (CC.N | (getCC(cpu) & CC.C)) : (r == 0) ? (CC.Z | (getCC(cpu) & CC.C)) : (getCC(cpu) & CC.C)); }

/*
 * V_ADD_x(cpu, r, s1, s2) / V_SUB_x(cpu, r, s1, s2) -- vax_defs.h:769-796.  Sets CC.V on overflow;
 * deliberately does NOT perform SIMH's deferred INTOV trap (see file header).
 */
function vAddL(cpu, r, s1, s2) { if (((~s1 ^ s2) & (s1 ^ r)) & LSIGN) setCC(cpu, getCC(cpu) | CC.V); }
function vSubL(cpu, r, s1, s2) { if (((s1 ^ s2) & (~s1 ^ r)) & LSIGN) setCC(cpu, getCC(cpu) | CC.V); }
function vAddB(cpu, r, s1, s2) { if (((~s1 ^ s2) & (s1 ^ r)) & BSIGN) setCC(cpu, getCC(cpu) | CC.V); }
function vAddW(cpu, r, s1, s2) { if (((~s1 ^ s2) & (s1 ^ r)) & WSIGN) setCC(cpu, getCC(cpu) | CC.V); }

/*
 * CC_CMP_x(cpu, s1, s2) -- vax_defs.h:813-827.  Used by CASEx to compare (selector-base) to limit.
 */
function ccCmpB(cpu, s1, s2) { let cc = (SXTB(s1) < SXTB(s2)) ? CC.N : (s1 == s2) ? CC.Z : 0; if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC.C; setCC(cpu, cc); }
function ccCmpW(cpu, s1, s2) { let cc = (SXTW(s1) < SXTW(s2)) ? CC.N : (s1 == s2) ? CC.Z : 0; if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC.C; setCC(cpu, cc); }
function ccCmpL(cpu, s1, s2) { let cc = (s1 < s2) ? CC.N : (s1 == s2) ? CC.Z : 0; if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC.C; setCC(cpu, cc); }

/**
 * branch(cpu, disp)
 *
 * SIMH's BRANCHB/BRANCHW/BRANCHB_ALWAYS/BRANCHW_ALWAYS, vax_defs.h:692-698, collapsed to one
 * function: the ALWAYS/non-ALWAYS split is purely SIMH's idle-loop-detection heuristic
 * (CHECK_FOR_IDLE_LOOP), which fast-forwards simulated wall-clock time and touches no
 * architectural state (no register, no PSL) -- so it cannot appear in a register-file+PSL diff
 * and there is nothing here to replicate.  Byte and word displacements collapse to the same
 * function too: `disp` must already be sign-extended by the caller (SXTB for a .bb displacement,
 * SXTW for .bw); `cpu.setPC` is the decoder-machine-interface primitive that also flushes
 * prefetch, matching FLUSH_ISTR.
 *
 * @param {Object} cpu
 * @param {number} disp already sign-extended
 */
function branch(cpu, disp) { cpu.setPC((cpu.regs[15] + disp) | 0); }

/**
 * jump(cpu, addr)
 *
 * SIMH's JUMP/JUMP_ALWAYS.
 *
 * @param {Object} cpu
 * @param {number} addr
 */
function jump(cpu, addr) { cpu.setPC(addr | 0); }

/**
 * store(decoder, cpu, size, value)
 *
 * The WRITE_B/WRITE_W/WRITE_L recipe from decode.js's isMemoryDestination() docblock, verbatim:
 * store through the LAST specifier decoded (decoder.spec/rn/va), not through opnd[].  Used by
 * every instruction here whose destination is a `modify` specifier (SOB/AOB/ACB).
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 * @param {number} size 1, 2, or 4
 * @param {number} value
 */
function store(decoder, cpu, size, value)
{
    if (decoder.isMemoryDestination()) {
        cpu.writeData(decoder.va, value, size);
        return;
    }
    let rn = decoder.rn;
    if (size == 1) {
        cpu.regs[rn] = (cpu.regs[rn] & ~0xFF) | (value & 0xFF);
    } else if (size == 2) {
        cpu.regs[rn] = (cpu.regs[rn] & ~0xFFFF) | (value & 0xFFFF);
    } else {
        cpu.regs[rn] = value | 0;
    }
}

/**
 * opBbN(decoder, cpu)
 *
 * SIMH vax_cpu1.c op_bb_n: return the tested bit, no modification.  opnd[0]=position, opnd[1]=
 * register-number-or-OP_MEM, opnd[2]=memory address (the `w.wb` two-slot queue contribution --
 * see decode.js's contract table).  The `pos >> 3` byte-offset conversion is a DELIBERATE signed
 * (arithmetic) shift, not the address-safety violation the project convention otherwise bans:
 * `pos` is an architected 32-bit bit-POSITION (not yet an address) that may legitimately be
 * negative, extending the effective address backward from the base operand -- this is what SIMH's
 * C `>>` does for a negative `int32 pos`, and matching it bit-for-bit requires the same signed
 * shift here.  The SUM that results is then treated as an address and passed through
 * `cpu.readData`, which normalizes at the Bus boundary exactly as rule 1 of the address convention
 * requires.
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 * @returns {number} 0 or 1
 */
function opBbN(decoder, cpu)
{
    let opnd = decoder.opnd;
    let pos = opnd[0];
    let rn = opnd[1];
    if (rn != OP_MEM) {
        if ((pos >>> 0) > 31) decoder.fault(VAXFAULT.RESOP);
        return (cpu.regs[rn] >>> pos) & 1;
    }
    let ea = (opnd[2] + (pos >> 3)) | 0;               // deliberate signed shift -- see docblock
    let bpos = pos & 7;
    let by = cpu.readData(ea, 1, false);
    return (by >>> bpos) & 1;
}

/**
 * opBbX(decoder, cpu, newb)
 *
 * SIMH vax_cpu1.c op_bb_x: return the tested bit, then set/clear it to `newb`.
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 * @param {number} newb 0 or 1
 * @returns {number} the bit's value BEFORE modification
 */
function opBbX(decoder, cpu, newb)
{
    let opnd = decoder.opnd;
    let pos = opnd[0];
    let rn = opnd[1];
    if (rn != OP_MEM) {
        if ((pos >>> 0) > 31) decoder.fault(VAXFAULT.RESOP);
        let bit = (cpu.regs[rn] >>> pos) & 1;
        cpu.regs[rn] = newb ? (cpu.regs[rn] | (1 << pos)) : (cpu.regs[rn] & ~(1 << pos));
        return bit;
    }
    let ea = (opnd[2] + (pos >> 3)) | 0;               // deliberate signed shift -- see opBbN
    let bpos = pos & 7;
    let by = cpu.readData(ea, 1, true);
    let bit = (by >>> bpos) & 1;
    by = newb ? (by | (1 << bpos)) : (by & ~(1 << bpos));
    cpu.writeData(ea, by, 1);
    return bit;
}

/*
 * rcnt[] -- vax_cpu1.c:79-88.  Precomputed "4 * popcount(index)" for a 7-bit register-mask nibble;
 * shared by CALL/RET (which only ever index it with a 6-bit value, R0-R5/R6-R11) and PUSHR/POPR
 * (which use the full 7 bits, R0-R6/R7-R13).
 */
const rcnt = [
 0, 4, 4, 8, 4, 8, 8,12, 4, 8, 8,12, 8,12,12,16,
 4, 8, 8,12, 8,12,12,16, 8,12,12,16,12,16,16,20,
 4, 8, 8,12, 8,12,12,16, 8,12,12,16,12,16,16,20,
 8,12,12,16,12,16,16,20,12,16,16,20,16,20,20,24,
 4, 8, 8,12, 8,12,12,16, 8,12,12,16,12,16,16,20,
 8,12,12,16,12,16,16,20,12,16,16,20,16,20,20,24,
 8,12,12,16,12,16,16,20,12,16,16,20,16,20,20,24,
12,16,16,20,16,20,20,24,16,20,20,24,20,24,24,28
];

const CALL_DV = 0x8000, CALL_IV = 0x4000, CALL_MBZ = 0x3000, CALL_MASK = 0x0FFF;
const CALL_V_SPA = 30, CALL_M_SPA = 0x3, CALL_V_S = 29, CALL_S = (1 << CALL_V_S);
const CALL_V_MASK = 16;
const PSW_MBZ = 0xFF00;

/**
 * opCall(decoder, cpu, gs)
 *
 * SIMH vax_cpu1.c op_call.  `gs` is true for CALLS, false for CALLG.  opnd[0] is the argument
 * COUNT (CALLS, a value) or the arglist ADDRESS (CALLG, an address -- op_call never dereferences
 * it, it becomes AP verbatim either way); opnd[1] is the procedure entry-point address.
 *
 * NOTE ON A PAIR OF SWAPPED SIMH COMMENTS.  The two `Write` lines that push the old FP and old AP
 * are commented "push AP" / "push FP" in vax_cpu1.c, backwards from what the code actually does.
 * Verified against op_ret's own reads (which agree with the CODE, not the comments): old AP lands
 * at FP+8, old FP at FP+12.  Ported the code; not the stale comments.
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 * @param {boolean} gs
 */
function opCall(decoder, cpu, gs)
{
    let opnd = decoder.opnd;
    let addr = opnd[1];
    let mask = cpu.readData(addr, 2, false);
    if (mask & CALL_MBZ) decoder.fault(VAXFAULT.RESOP);
    let stklen = rcnt[mask & 0x3F] + rcnt[(mask >>> 6) & 0x3F] + (gs ? 24 : 20);
    cpu.readData((cpu.regs[14] - stklen) | 0, 1, true);        // wchk: probe the far end of the frame
    if (gs) {
        cpu.writeData((cpu.regs[14] - 4) | 0, opnd[0], 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
    }
    let tsp = cpu.regs[14] & ~CALL_M_SPA;
    for (let n = 11; n >= 0; n--) {
        if ((mask >>> n) & 1) {
            tsp = (tsp - 4) | 0;
            cpu.writeData(tsp, cpu.regs[n], 4);
        }
    }
    cpu.writeData((tsp - 4) | 0, cpu.regs[15], 4);              // push PC
    cpu.writeData((tsp - 8) | 0, cpu.regs[13], 4);              // push (old) FP
    cpu.writeData((tsp - 12) | 0, cpu.regs[12], 4);             // push (old) AP
    let wd = (((cpu.regs[14] & CALL_M_SPA) << CALL_V_SPA) | ((gs ? 1 : 0) << CALL_V_S) |
              ((mask & CALL_MASK) << CALL_V_MASK) | (cpu.psl & 0xFFE0)) | 0;
    cpu.writeData((tsp - 16) | 0, wd, 4);
    cpu.writeData((tsp - 20) | 0, 0, 4);                        // condition handler, initially 0
    if (gs) {
        cpu.regs[12] = cpu.regs[14];                            // AP = SP (post arg-count push)
    } else {
        cpu.regs[12] = opnd[0];                                 // AP = arglist address, verbatim
    }
    cpu.regs[14] = cpu.regs[13] = (tsp - 20) | 0;                // SP = FP = new frame base
    cpu.psl = (cpu.psl & ~(PSW.DV | PSW.FU | PSW.IV)) |
              ((mask & CALL_DV) ? PSW.DV : 0) | ((mask & CALL_IV) ? PSW.IV : 0);
    jump(cpu, (addr + 2) | 0);
    setCC(cpu, 0);
}

/**
 * opRet(decoder, cpu)
 *
 * SIMH vax_cpu1.c op_ret.  Frame is based at FP; see opCall's docblock for the layout.
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 */
function opRet(decoder, cpu)
{
    let tsp = cpu.regs[13];
    let spamask = cpu.readData((tsp + 4) | 0, 4, false);
    if (spamask & PSW_MBZ) decoder.fault(VAXFAULT.RESOP);
    let stklen = rcnt[(spamask >>> CALL_V_MASK) & 0x3F] + rcnt[(spamask >>> (CALL_V_MASK + 6)) & 0x3F] +
                 ((spamask & CALL_S) ? 23 : 19);
    cpu.readData((tsp + stklen) | 0, 1, false);                 // rchk: probe the far end of the frame
    cpu.regs[12] = cpu.readData((tsp + 8) | 0, 4, false);        // AP
    cpu.regs[13] = cpu.readData((tsp + 12) | 0, 4, false);       // FP
    let newpc = cpu.readData((tsp + 16) | 0, 4, false);
    tsp = (tsp + 20) | 0;
    for (let n = 0; n <= 11; n++) {
        if ((spamask >>> (n + CALL_V_MASK)) & 1) {
            cpu.regs[n] = cpu.readData(tsp, 4, false);
            tsp = (tsp + 4) | 0;
        }
    }
    cpu.regs[14] = (tsp + ((spamask >>> CALL_V_SPA) & CALL_M_SPA)) | 0;
    if (spamask & CALL_S) {
        let nargs = cpu.readData(cpu.regs[14], 4, false);
        cpu.regs[14] = (cpu.regs[14] + 4 + ((nargs & BMASK) << 2)) | 0;
    }
    cpu.psl = (cpu.psl & ~(PSW.DV | PSW.FU | PSW.IV | PSW.T)) | (spamask & (PSW.DV | PSW.FU | PSW.IV | PSW.T));
    jump(cpu, newpc);
    setCC(cpu, spamask & CC.MASK);
}

/**
 * opPushr(decoder, cpu)
 *
 * SIMH vax_cpu1.c op_pushr.  opnd[0] is the 15-bit register mask (R0-R14, bit 14 = SP).
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 */
function opPushr(decoder, cpu)
{
    let mask = decoder.opnd[0] & 0x7FFF;
    if (mask == 0) return;
    let stklen = rcnt[(mask >>> 7) & 0x7F] + rcnt[mask & 0x7F] + ((mask & 0x4000) ? 4 : 0);
    cpu.readData((cpu.regs[14] - stklen) | 0, 1, true);         // wchk
    let tsp = cpu.regs[14];
    for (let n = 14; n >= 0; n--) {
        if ((mask >>> n) & 1) {
            tsp = (tsp - 4) | 0;
            cpu.writeData(tsp, cpu.regs[n], 4);                  // R[14] here is still the ORIGINAL SP
        }
    }
    cpu.regs[14] = tsp;
}

/**
 * opPopr(decoder, cpu)
 *
 * SIMH vax_cpu1.c op_popr.  Unlike PUSHR, pops mutate the LIVE SP incrementally (n=0..13); bit 14
 * (SP itself) is popped last and does NOT get the usual +4 afterward -- setting SP already moves
 * the pointer, so there is nothing left to increment.
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 */
function opPopr(decoder, cpu)
{
    let mask = decoder.opnd[0] & 0x7FFF;
    if (mask == 0) return;
    let stklen = rcnt[(mask >>> 7) & 0x7F] + rcnt[mask & 0x7F] + ((mask & 0x4000) ? 4 : 0);
    cpu.readData((cpu.regs[14] + stklen - 1) | 0, 1, false);    // rchk
    for (let n = 0; n <= 13; n++) {
        if ((mask >>> n) & 1) {
            cpu.regs[n] = cpu.readData(cpu.regs[14], 4, false);
            cpu.regs[14] = (cpu.regs[14] + 4) | 0;
        }
    }
    if (mask & 0x4000) {
        cpu.regs[14] = cpu.readData(cpu.regs[14], 4, false);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Opcode bodies, named by mnemonic (drom.js OPCODES).  Each takes (decoder, cpu).               *
 * ------------------------------------------------------------------------------------------- */

const BODIES = {

    /* -------------------------------------------------------- Simple branches and BSB */
    BRB(decoder, cpu) { branch(cpu, SXTB(decoder.brdisp)); },
    BRW(decoder, cpu) { branch(cpu, SXTW(decoder.brdisp)); },

    BSBB(decoder, cpu) {
        cpu.writeData((cpu.regs[14] - 4) | 0, cpu.regs[15], 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        branch(cpu, SXTB(decoder.brdisp));
    },
    BSBW(decoder, cpu) {
        cpu.writeData((cpu.regs[14] - 4) | 0, cpu.regs[15], 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        branch(cpu, SXTW(decoder.brdisp));
    },

    /* Bcc family.  BNEQ/BNEQU and BEQL/BEQLU are architecturally identical opcodes (equality does
       not care about signedness), so the decode ROM has one entry each; SIMH's idle-loop heuristics
       on BEQL/BVS touch no architectural state -- see branch()'s docblock. */
    BGEQ(decoder, cpu)  { if (!(getCC(cpu) & CC.N)) branch(cpu, SXTB(decoder.brdisp)); },
    BLSS(decoder, cpu)  { if (getCC(cpu) & CC.N) branch(cpu, SXTB(decoder.brdisp)); },
    BNEQ(decoder, cpu)  { if (!(getCC(cpu) & CC.Z)) branch(cpu, SXTB(decoder.brdisp)); },
    BEQL(decoder, cpu)  { if (getCC(cpu) & CC.Z) branch(cpu, SXTB(decoder.brdisp)); },
    BVC(decoder, cpu)   { if (!(getCC(cpu) & CC.V)) branch(cpu, SXTB(decoder.brdisp)); },
    BVS(decoder, cpu)   { if (getCC(cpu) & CC.V) branch(cpu, SXTB(decoder.brdisp)); },
    BGEQU(decoder, cpu) { if (!(getCC(cpu) & CC.C)) branch(cpu, SXTB(decoder.brdisp)); },
    BLSSU(decoder, cpu) { if (getCC(cpu) & CC.C) branch(cpu, SXTB(decoder.brdisp)); },
    BGTR(decoder, cpu)  { if (!(getCC(cpu) & (CC.N | CC.Z))) branch(cpu, SXTB(decoder.brdisp)); },
    BLEQ(decoder, cpu)  { if (getCC(cpu) & (CC.N | CC.Z)) branch(cpu, SXTB(decoder.brdisp)); },
    BGTRU(decoder, cpu) { if (!(getCC(cpu) & (CC.C | CC.Z))) branch(cpu, SXTB(decoder.brdisp)); },
    BLEQU(decoder, cpu) { if (getCC(cpu) & (CC.C | CC.Z)) branch(cpu, SXTB(decoder.brdisp)); },

    /* ------------------------------------------------------------ Jumps and subroutines */
    JSB(decoder, cpu) {
        cpu.writeData((cpu.regs[14] - 4) | 0, cpu.regs[15], 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        jump(cpu, decoder.opnd[0]);
    },
    JMP(decoder, cpu) { jump(cpu, decoder.opnd[0]); },
    RSB(decoder, cpu) {
        let temp = cpu.readData(cpu.regs[14], 4, false);
        cpu.regs[14] = (cpu.regs[14] + 4) | 0;
        jump(cpu, temp);
    },

    /* ------------------------------------------------------------------- SOB/AOB/ACB */
    SOBGEQ(decoder, cpu) {
        let op0 = decoder.opnd[0];
        let r = (op0 - 1) | 0;
        store(decoder, cpu, 4, r);
        ccIIZP_L(cpu, r);
        vSubL(cpu, r, 1, op0);
        if (r >= 0) branch(cpu, SXTB(decoder.brdisp));
    },
    SOBGTR(decoder, cpu) {
        let op0 = decoder.opnd[0];
        let r = (op0 - 1) | 0;
        store(decoder, cpu, 4, r);
        ccIIZP_L(cpu, r);
        vSubL(cpu, r, 1, op0);
        if (r > 0) branch(cpu, SXTB(decoder.brdisp));
    },
    AOBLSS(decoder, cpu) {
        let op0 = decoder.opnd[0], op1 = decoder.opnd[1];
        let r = (op1 + 1) | 0;
        store(decoder, cpu, 4, r);
        ccIIZP_L(cpu, r);
        vAddL(cpu, r, 1, op1);
        if (r < op0) branch(cpu, SXTB(decoder.brdisp));
    },
    AOBLEQ(decoder, cpu) {
        let op0 = decoder.opnd[0], op1 = decoder.opnd[1];
        let r = (op1 + 1) | 0;
        store(decoder, cpu, 4, r);
        ccIIZP_L(cpu, r);
        vAddL(cpu, r, 1, op1);
        if (r <= op0) branch(cpu, SXTB(decoder.brdisp));
    },
    ACBB(decoder, cpu) {
        let op0 = decoder.opnd[0], op1 = decoder.opnd[1], op2 = decoder.opnd[2];
        let r = (op2 + op1) & BMASK;
        store(decoder, cpu, 1, r);
        ccIIZP_B(cpu, r);
        vAddB(cpu, r, op1, op2);
        if ((op1 & BSIGN) ? (SXTB(r) >= SXTB(op0)) : (SXTB(r) <= SXTB(op0))) branch(cpu, SXTW(decoder.brdisp));
    },
    ACBW(decoder, cpu) {
        let op0 = decoder.opnd[0], op1 = decoder.opnd[1], op2 = decoder.opnd[2];
        let r = (op2 + op1) & WMASK;
        store(decoder, cpu, 2, r);
        ccIIZP_W(cpu, r);
        vAddW(cpu, r, op1, op2);
        if ((op1 & WSIGN) ? (SXTW(r) >= SXTW(op0)) : (SXTW(r) <= SXTW(op0))) branch(cpu, SXTW(decoder.brdisp));
    },
    ACBL(decoder, cpu) {
        let op0 = decoder.opnd[0], op1 = decoder.opnd[1], op2 = decoder.opnd[2];
        let r = (op2 + op1) | 0;
        store(decoder, cpu, 4, r);
        ccIIZP_L(cpu, r);
        vAddL(cpu, r, op1, op2);
        if ((op1 & LSIGN) ? (r >= op0) : (r <= op0)) branch(cpu, SXTW(decoder.brdisp));
    },

    /* ------------------------------------------------------------------------- CASE */
    CASEB(decoder, cpu) {
        let opnd = decoder.opnd;
        let r = (opnd[0] - opnd[1]) & BMASK;
        ccCmpB(cpu, r, opnd[2]);
        let PC = cpu.regs[15];
        if ((r >>> 0) > (opnd[2] >>> 0)) {
            jump(cpu, (PC + (opnd[2] + 1) * 2) | 0);
        } else {
            let temp = cpu.readData((PC + r * 2) | 0, 2, false);
            branch(cpu, SXTW(temp));
        }
    },
    CASEW(decoder, cpu) {
        let opnd = decoder.opnd;
        let r = (opnd[0] - opnd[1]) & WMASK;
        ccCmpW(cpu, r, opnd[2]);
        let PC = cpu.regs[15];
        if ((r >>> 0) > (opnd[2] >>> 0)) {
            jump(cpu, (PC + (opnd[2] + 1) * 2) | 0);
        } else {
            let temp = cpu.readData((PC + r * 2) | 0, 2, false);
            branch(cpu, SXTW(temp));
        }
    },
    CASEL(decoder, cpu) {
        let opnd = decoder.opnd;
        let r = (opnd[0] - opnd[1]) | 0;
        ccCmpL(cpu, r, opnd[2]);
        let PC = cpu.regs[15];
        if ((r >>> 0) > (opnd[2] >>> 0)) {
            jump(cpu, (PC + (opnd[2] + 1) * 2) | 0);
        } else {
            let temp = cpu.readData((PC + r * 2) | 0, 2, false);
            branch(cpu, SXTW(temp));
        }
    },

    /* ------------------------------------------------------------------ Branch on bit */
    BBS(decoder, cpu)   { if (opBbN(decoder, cpu)) branch(cpu, SXTB(decoder.brdisp)); },
    BBC(decoder, cpu)   { if (!opBbN(decoder, cpu)) branch(cpu, SXTB(decoder.brdisp)); },
    BBSS(decoder, cpu)  { if (opBbX(decoder, cpu, 1)) branch(cpu, SXTB(decoder.brdisp)); },
    BBSSI(decoder, cpu) { if (opBbX(decoder, cpu, 1)) branch(cpu, SXTB(decoder.brdisp)); },
    BBCC(decoder, cpu)  { if (!opBbX(decoder, cpu, 0)) branch(cpu, SXTB(decoder.brdisp)); },
    BBCCI(decoder, cpu) { if (!opBbX(decoder, cpu, 0)) branch(cpu, SXTB(decoder.brdisp)); },
    BBSC(decoder, cpu)  { if (opBbX(decoder, cpu, 0)) branch(cpu, SXTB(decoder.brdisp)); },
    BBCS(decoder, cpu)  { if (!opBbX(decoder, cpu, 1)) branch(cpu, SXTB(decoder.brdisp)); },
    BLBS(decoder, cpu)  { if (decoder.opnd[0] & 1) branch(cpu, SXTB(decoder.brdisp)); },
    BLBC(decoder, cpu)  { if (!(decoder.opnd[0] & 1)) branch(cpu, SXTB(decoder.brdisp)); },

    /* ------------------------------------------------------------------ Call/return */
    CALLS(decoder, cpu) { opCall(decoder, cpu, true); },
    CALLG(decoder, cpu) { opCall(decoder, cpu, false); },
    RET(decoder, cpu)   { opRet(decoder, cpu); },

    /* --------------------------------------------------------------------- Stack */
    PUSHR(decoder, cpu) { opPushr(decoder, cpu); },
    POPR(decoder, cpu)  { opPopr(decoder, cpu); },

    PUSHL(decoder, cpu) {
        let op0 = decoder.opnd[0];
        cpu.writeData((cpu.regs[14] - 4) | 0, op0, 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        ccIIZP_L(cpu, op0);
    },
    PUSHAB(decoder, cpu) {
        let op0 = decoder.opnd[0];
        cpu.writeData((cpu.regs[14] - 4) | 0, op0, 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        ccIIZP_L(cpu, op0);
    },
    PUSHAW(decoder, cpu) {
        let op0 = decoder.opnd[0];
        cpu.writeData((cpu.regs[14] - 4) | 0, op0, 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        ccIIZP_L(cpu, op0);
    },
    PUSHAL(decoder, cpu) {
        let op0 = decoder.opnd[0];
        cpu.writeData((cpu.regs[14] - 4) | 0, op0, 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        ccIIZP_L(cpu, op0);
    },
    PUSHAQ(decoder, cpu) {
        let op0 = decoder.opnd[0];
        cpu.writeData((cpu.regs[14] - 4) | 0, op0, 4);
        cpu.regs[14] = (cpu.regs[14] - 4) | 0;
        ccIIZP_L(cpu, op0);
    }
};

/*
 * CONTROL_OPCODES: the dispatch table, keyed by NUMERIC opcode (0x000-0x1FF), built once from
 * OPCODES (drom.js) so the numeric values never have to be transcribed by hand.
 */
const CONTROL_OPCODES = {};
for (let name in BODIES) {
    let opc = OPCODES.indexOf(name);
    if (opc < 0) throw new Error(`control.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
    CONTROL_OPCODES[opc] = BODIES[name];
}

/**
 * executeControl(opc, decoder, cpu)
 *
 * @param {number} opc
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 * @returns {boolean} true if this module handled the opcode
 */
function executeControl(opc, decoder, cpu)
{
    let fn = CONTROL_OPCODES[opc];
    if (!fn) return false;
    fn(decoder, cpu);
    return true;
}

export default executeControl;
export { executeControl, CONTROL_OPCODES, CC, PSW, SXTB, SXTW, getCC, setCC, store, opBbN, opBbX, opCall, opRet, opPushr, opPopr };
