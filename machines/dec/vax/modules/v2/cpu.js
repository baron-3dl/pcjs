/**
 * @fileoverview VAX CPU -- integer, logical, and variable-length bit-field instruction execution
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
 * SCOPE -- READ THIS BEFORE ADDING AN OPCODE
 * ============================================================================
 * This is the EXECUTION half of the Base Instruction Group's integer, logical, and variable-
 * length bit-field opcodes -- the arithmetic/logical/data-movement work that decode.js
 * (pcjsvax-8c0) and mmu.js (pcjsvax-061) exist to feed.  It is a direct port of the corresponding
 * `case` labels in Open SIMH `vax_cpu.c`'s `sim_instr()` switch, plus `op_extv`/`op_insv`/`op_ffs`
 * (vax_cpu1.c) and `op_emul`/`op_ediv`/`op_ashq` (vax_fpa.c), structured as a dispatch table of
 * functions keyed by mnemonic rather than SIMH's megamorphic switch.
 *
 * Three sibling items own the rest of the Base Instruction Group's 200 IG_BASE opcodes (plus the
 * 42 F_floating/D_floating/G_floating opcodes that live administratively inside IG_BASE/BSDFL/
 * BSGFL but are semantically floating point):
 *
 *   pcjsvax-fab   control flow: branches, jumps, SOB/AOB/ACB/CASE, calls/returns, branch-on-bit,
 *                 CHMx, exceptions (HALT/BPT/XFC/REI/LDPCTX/SVPCTX).
 *   pcjsvax-710   floating point: F_floating (technically IG_BASE) and all of D_/G_floating.
 *   pcjsvax-515   string/queue/remainder: MOVCn, CMPCn, LOCC, SKPC, SCANC, SPANC, MATCHC, EDITPC,
 *                 INSQxx, REMQxx, INDEX, PROBER/PROBEW, MTPR/MFPR, PUSHR/POPR, packed decimal/CIS.
 *
 * THE 107 OPCODES THIS FILE IMPLEMENTS (all of them; see the DISPATCH table at the bottom for the
 * authoritative list, generated positionally from drom.js's OPCODES so it can never drift from
 * what is actually wired up):
 *
 *   Move/clear/test/convert:  MOVB/W/L, MOVZBW/BL/WL, MOVAB/AW/AL/AQ, CLRB/W/L/Q, TSTB/W/L,
 *     MCOMB/W/L, MNEGB/W/L, CVTBW/BL/WB/WL/LB/LW, MOVQ, PUSHL, PUSHAB/AW/AL/AQ, MOVPSL
 *   Arithmetic:  ADDB/W/L 2/3, ADWC, SUBB/W/L 2/3, SBWC, MULB/W/L 2/3, DIVB/W/L 2/3, INCB/W/L,
 *     DECB/W/L, CMPB/W/L, ADAWI, EMUL, EDIV
 *   Logical:  BISB/W/L 2/3, BICB/W/L 2/3, XORB/W/L 2/3, BITB/W/L, BISPSW, BICPSW
 *   Shifts:  ROTL, ASHL, ASHQ
 *   Variable-length bit field:  EXTV, EXTZV, INSV, FFS, FFC, CMPV, CMPZV
 *
 * NOT HERE, ON PURPOSE:  reserved-ADDRESSING-mode faults during specifier resolution (decode.js's
 * job, already graded by decodediff.js's FAULTS phase) and the SCB dispatch of an EXECUTION-time
 * fault (ADAWI's odd-address check, a bit-field size>32 or register-pair-span check, WRITE_Q's
 * register-pair overflow check).  Those checks are ported faithfully -- each throws the same
 * VAXFault the decoder throws -- but no exception-frame push or PC redirection happens here; that
 * is full instruction-loop machinery outside a single-instruction execute() call, and this item's
 * done condition (README: "execute one instruction ... compare the full register file plus PSL")
 * does not require it.  See tests/intdiff.js's header for exactly what coverage that leaves open.
 * SET_TRAP-style deferred arithmetic traps (integer overflow, integer divide-by-zero) are ALSO not
 * dispatched here, for the same reason SIMH does not dispatch them inline: the trap fires at the
 * TOP of the NEXT instruction, never during the one that set it, so it is invisible to a single-
 * instruction register-file-plus-PSL comparison.  The CC.V bit IS set correctly in all cases; only
 * the deferred SCB dispatch is out of scope.
 *
 * ============================================================================
 * 64-BIT ARITHMETIC -- HI/LO PAIRS, NOT BigInt
 * ============================================================================
 * EMUL, EDIV and ASHQ need 64-bit intermediates.  BigInt is too slow for an inner loop (see
 * pcjs/machines/modules/v2/int36.js for the PDP-10 precedent of hand-rolled multi-word
 * arithmetic), so every 64-bit operation here is a pair of int32 halves (lo, hi) and a handful of
 * schoolbook helpers: `umul32_64` (unsigned 32x32->64 multiply via four 16x16 partial products,
 * the standard technique also used by e.g. Google Closure's Long.js), `udivmod64by32` (unsigned
 * 64/32 restoring division, 64 shift-and-subtract iterations -- run once per EDIV, not in any hot
 * loop), and `shl64`/`sar64` (word-boundary shifts, O(1), no bit-at-a-time loop).  Every
 * intermediate stays within JS's exact-integer range (2^53): the largest values these helpers ever
 * hold are individual 32-bit halves or small (<2^33) partial sums, never a full 64-bit magnitude
 * as one Number.
 *
 * ============================================================================
 * ADDRESS / DATA CONVENTION -- see defines.js and decode.js's file header
 * ============================================================================
 * `va` (from the decoder) is a signed int32 virtual address; it is never shifted with `>>` and is
 * coerced with `>>> 0` only where MMUVAX requires it (MMUVAX does that internally).  Longword data
 * is signed int32 throughout, matching decode.js's contract; byte and word operand values arrive
 * from the decoder already zero-extended and are sign-extended here with `sxtB`/`sxtW` exactly
 * where SIMH's SXTB/SXTW macros do it -- nowhere else.
 */

import { VAXFault, VAXFAULT, OP_MEM } from "./decode.js";
import VAXDecoder from "./decode.js";
import MMUVAX from "./mmu.js";
import { OPCODES } from "./drom.js";

const L_BYTE = 1, L_WORD = 2, L_LONG = 4;

const BMASK = 0xFF, WMASK = 0xFFFF, LMASK = 0xFFFFFFFF;
const BSIGN = 0x80, WSIGN = 0x8000, LSIGN = 0x80000000;

/* Condition codes, matching vax_defs.h CC_* exactly (mmu.js and decode.js do not need these). */
const CC_C = 0x1, CC_V = 0x2, CC_Z = 0x4, CC_N = 0x8, CC_MASK = 0xF;

/* PSW<15:8> reserved, must be zero -- BISPSW/BICPSW/CVT* reserved-operand check (vax_defs.h). */
const PSW_MBZ = 0xFF00;

/* PSW<T> (trace enable, vax_defs.h PSW_T=0x10) and PSL<TP> (trace pending, PSL_V_TP=30).  Not an
   instruction-specific behavior -- vax_cpu.c's main loop (line ~716) sets TP from T at the START
   of EVERY instruction dispatch, before the opcode is even fetched, so it belongs in step(), not
   in any one opcode's handler.  A state that already has TP set would instead redirect to the
   trace-trap SCB vector without executing the pending opcode at all; that dispatch (and the
   deferred trace TRAP once TP is taken) is exception machinery this item does not implement (see
   the file header), so tests/intdiff.js never constructs a starting state with TP set. */
const PSW_T = 0x10, PSL_TP = 1 << 30;

const nSP = 14, nPC = 15;

/* ------------------------------------------------------------------------------------------- *
 * Sign-extension, matching vax_defs.h SXTB/SXTW.  Byte/word operands arrive zero-extended        *
 * (decode.js's contract); these produce the signed int32 SIMH's macros produce.                  *
 * ------------------------------------------------------------------------------------------- */

function sxtB(v) { return (v & BSIGN) ? (v | ~BMASK) : (v & BMASK); }
function sxtW(v) { return (v & WSIGN) ? (v | ~WMASK) : (v & WMASK); }

/* byte_mask[]/byte_sign[] (vax_cpu.c:290-309), computed rather than tabulated: byte_mask[n] is the
   n-bit all-ones mask (0 for n=0, 0xFFFFFFFF for n>=32); byte_sign[n] is bit n-1 (0 for n=0). */
function byteMask(n) { return n >= 32 ? 0xFFFFFFFF : (((1 << n) - 1) >>> 0); }
function byteSign(n) { return n === 0 ? 0 : (((1 << (n - 1)) >>> 0)); }

/* ------------------------------------------------------------------------------------------- *
 * 64-bit hi/lo helpers -- see the file header.  Every function here takes/returns int32 halves   *
 * (via `| 0`) except where explicitly noted as an unsigned intermediate.                          *
 * ------------------------------------------------------------------------------------------- */

/**
 * umul32_64(aU, bU)
 *
 * Unsigned 32x32->64 multiply via four 16x16 partial products (schoolbook long multiplication).
 * Every intermediate here is bounded by roughly 2^32, far inside JS's exact-integer range.
 *
 * @param {number} aU unsigned 0..0xFFFFFFFF
 * @param {number} bU unsigned 0..0xFFFFFFFF
 * @returns {{lo: number, hi: number}} unsigned 32-bit halves (0..0xFFFFFFFF each)
 */
function umul32_64(aU, bU)
{
    let aLo = aU & 0xFFFF, aHi = aU >>> 16;
    let bLo = bU & 0xFFFF, bHi = bU >>> 16;
    let lo = aLo * bLo;
    let t = aHi * bLo + (lo >>> 16);
    let lo16 = t & 0xFFFF;
    let c2 = t >>> 16;
    t = aLo * bHi + lo16;
    let loFinal = (((t & 0xFFFF) << 16) | (lo & 0xFFFF)) >>> 0;
    let c3 = t >>> 16;
    let hiFinal = (aHi * bHi + c2 + c3) >>> 0;
    return {lo: loFinal, hi: hiFinal};
}

/**
 * mul32signed(a, b)
 *
 * Signed 32x32->64 multiply (op_emul's core), via the unsigned multiply above plus the standard
 * two's-complement correction: signed(a,b) === unsigned(au,bu) - 2^32*(a<0?bu:0) - 2^32*(b<0?au:0)
 * (mod 2^64), which only ever touches the high word.
 *
 * @param {number} a signed int32
 * @param {number} b signed int32
 * @returns {{lo: number, hi: number}} signed int32 halves
 */
function mul32signed(a, b)
{
    let lo = Math.imul(a, b);
    let au = a >>> 0, bu = b >>> 0;
    let {hi} = umul32_64(au, bu);
    if (a < 0) hi = (hi - bu) >>> 0;
    if (b < 0) hi = (hi - au) >>> 0;
    return {lo: lo, hi: hi | 0};
}

/**
 * neg64(loU, hiU)
 *
 * Two's-complement negate of an unsigned 64-bit hi/lo pair.
 *
 * @param {number} loU unsigned
 * @param {number} hiU unsigned
 * @returns {Array.<number>} [lo, hi] unsigned
 */
function neg64(loU, hiU)
{
    let lo2 = (~loU) >>> 0, hi2 = (~hiU) >>> 0;
    let lo3 = (lo2 + 1) >>> 0;
    let hi3 = (hi2 + (lo2 === 0xFFFFFFFF ? 1 : 0)) >>> 0;
    return [lo3, hi3];
}

/**
 * udivmod64by32(loU, hiU, divisorU)
 *
 * Unsigned 64-bit dividend / unsigned 32-bit (nonzero) divisor, via 64-iteration restoring
 * division.  `rem` stays under `divisorU` (<= 2^32-1) throughout, so it is safe as a plain JS
 * Number the whole way; only the per-iteration doubling ever approaches 2^33.
 *
 * @param {number} loU unsigned
 * @param {number} hiU unsigned
 * @param {number} divisorU unsigned, nonzero
 * @returns {{qlo: number, qhi: number, rem: number}} qlo/qhi unsigned int32 halves, rem unsigned
 */
function udivmod64by32(loU, hiU, divisorU)
{
    let rem = 0, qhi = 0, qlo = 0;
    for (let i = 63; i >= 0; i--) {
        let bit = i >= 32 ? ((hiU >>> (i - 32)) & 1) : ((loU >>> i) & 1);
        rem = rem * 2 + bit;
        if (rem >= divisorU) {
            rem -= divisorU;
            if (i >= 32) qhi = (qhi | (1 << (i - 32))) >>> 0;
            else qlo = (qlo | (1 << i)) >>> 0;
        }
    }
    return {qlo: qlo, qhi: qhi, rem: rem};
}

/**
 * shl64(lo, hi, n)
 *
 * 64-bit logical left shift by 0..63, word-boundary technique (no bit loop).
 *
 * @param {number} lo signed int32
 * @param {number} hi signed int32
 * @param {number} n 0..63
 * @returns {Array.<number>} [lo, hi] signed int32
 */
function shl64(lo, hi, n)
{
    if (n === 0) return [lo, hi];
    if (n < 32) return [(lo << n) | 0, ((hi << n) | (lo >>> (32 - n))) | 0];
    return [0, (lo << (n - 32)) | 0];
}

/**
 * sar64(lo, hi, n)
 *
 * 64-bit ARITHMETIC right shift by 0..63 (sign-extending from `hi`'s sign bit).
 *
 * @param {number} lo signed int32
 * @param {number} hi signed int32
 * @param {number} n 0..63
 * @returns {Array.<number>} [lo, hi] signed int32
 */
function sar64(lo, hi, n)
{
    if (n === 0) return [lo, hi];
    if (n < 32) return [((lo >>> n) | (hi << (32 - n))) | 0, hi >> n];
    return [(hi >> (n - 32)) | 0, hi >> 31];
}

/* ------------------------------------------------------------------------------------------- *
 * op_emul / op_ediv / op_ashq / op_extv / op_insv / op_ffs -- direct ports of vax_fpa.c /         *
 * vax_cpu1.c, using the hi/lo helpers above in place of t_int64.                                  *
 * ------------------------------------------------------------------------------------------- */

/**
 * opEmul(mpy, mpc, adder)
 *
 * EMUL's 32x32+32->64 (vax_cpu.c EMUL case): multiply, then add a 32-bit value into the 64-bit
 * product with carry into the high word.
 *
 * @param {number} mpy signed int32
 * @param {number} mpc signed int32
 * @param {number} adder signed int32
 * @returns {{lo: number, hi: number}}
 */
function opEmul(mpy, mpc, adder)
{
    let {lo, hi} = mul32signed(mpy, mpc);
    let r = (lo + adder) | 0;
    let rh = (hi + (((r >>> 0) < (adder >>> 0)) ? 1 : 0) - ((adder & LSIGN) ? 1 : 0)) | 0;
    return {lo: r, hi: rh};
}

/**
 * opEdiv(divisor, dvdLo, dvdHi)
 *
 * EDIV's 64/32->32r32 (vax_cpu1's op_ediv via vax_fpa.c).  `divisor` must be nonzero; the caller
 * (the EDIV handler) takes the divide-by-zero branch itself, matching SIMH's structure.
 *
 * @param {number} divisor signed int32, nonzero
 * @param {number} dvdLo signed int32 (low longword of the 64-bit dividend)
 * @param {number} dvdHi signed int32 (high longword)
 * @returns {{quo: number, rem: number, flg: number}} flg is CC_V (overflow) or 0
 */
function opEdiv(divisor, dvdLo, dvdHi)
{
    let divisorU = divisor >>> 0;
    let ldvrU = (divisor & LSIGN) ? ((~divisorU >>> 0) + 1) >>> 0 : divisorU;
    let loU = dvdLo >>> 0, hiU = dvdHi >>> 0;
    let negDividend = (dvdHi & LSIGN) !== 0;
    if (negDividend) { [loU, hiU] = neg64(loU, hiU); }
    if (hiU >= ldvrU) return {quo: dvdLo, rem: 0, flg: CC_V};
    let {qlo, rem} = udivmod64by32(loU, hiU, ldvrU);
    let quo = qlo | 0;
    if ((divisor ^ dvdHi) & LSIGN) {
        quo = (-quo) | 0;
        if (quo !== 0 && (quo & LSIGN) === 0) return {quo: dvdLo, rem: 0, flg: CC_V};
    } else if (quo & LSIGN) {
        return {quo: dvdLo, rem: 0, flg: CC_V};
    }
    let remSigned = rem | 0;
    if (negDividend) remSigned = (-remSigned) | 0;
    return {quo: quo, rem: remSigned, flg: 0};
}

/**
 * opAshq(sc, lo, hi)
 *
 * ASHQ's 64-bit arithmetic shift (op_ashq, vax_fpa.c).  `sc` is the zero-extended shift-count
 * byte as the decoder delivers it (0..255); bit 7 (BSIGN) selects a right shift, matching SIMH.
 *
 * @param {number} sc 0..255
 * @param {number} lo signed int32
 * @param {number} hi signed int32
 * @returns {{lo: number, hi: number, flg: number}} flg is 1 (overflow) or 0
 */
function opAshq(sc, lo, hi)
{
    if (sc & BSIGN) {
        let n = (0x100 - sc);
        if (n > 63) { let r = (hi & LSIGN) ? -1 : 0; return {lo: r, hi: r, flg: 0}; }
        let [rlo, rhi] = sar64(lo, hi, n);
        return {lo: rlo, hi: rhi, flg: 0};
    }
    if (sc > 63) return {lo: 0, hi: 0, flg: (lo !== 0 || hi !== 0) ? 1 : 0};
    let [rlo, rhi] = shl64(lo, hi, sc);
    let [blo, bhi] = sar64(rlo, rhi, sc);
    return {lo: rlo, hi: rhi, flg: (blo !== lo || bhi !== hi) ? 1 : 0};
}

/**
 * opFfs(wd, size)
 *
 * Find the first (least-significant) set bit, or `size` if none.
 *
 * @param {number} wd
 * @param {number} size
 * @returns {number}
 */
function opFfs(wd, size)
{
    wd = wd >>> 0;
    for (let i = 0; wd; i++, wd = wd >>> 1) {
        if (wd & 1) return i;
    }
    return size;
}

/**
 * opExtv(cpu, d, pos, size, rn, wd)
 *
 * op_extv (vax_cpu1.c): extract a 0..32 bit field.  Register mode reads the (rn, rn+1) pair via
 * `d.vfldrp1`; memory mode reads one or two longwords through the MMU with READ access.
 *
 * @param {VAXCpu} cpu
 * @param {VAXDecoder} d
 * @param {number} pos
 * @param {number} size
 * @param {number} rn OP_MEM or a register number
 * @param {number} wd R[rn] (register mode) or the base address (memory mode)
 * @returns {number} the field, right-justified, zero-extended (0..byteMask(size))
 */
function opExtv(cpu, d, pos, size, rn, wd)
{
    if (size === 0) return 0;
    if (size > 32) cpu.fault(VAXFAULT.RESOP);
    if (rn !== OP_MEM) {
        if ((pos >>> 0) > 31) cpu.fault(VAXFAULT.RESOP);
        if ((pos + size) > 32 && rn >= nSP) cpu.fault(VAXFAULT.RESAD);
        if (pos) wd = (wd >>> pos) | (d.vfldrp1 << (32 - pos));
        return (wd & byteMask(size)) >>> 0;
    }
    let ba = (wd + (pos >> 3)) | 0;
    let bitpos = (pos & 7) | ((ba & 3) << 3);
    ba = ba & ~3;
    let lo = cpu.mmu.readData(ba, L_LONG, cpu.accR());
    let hi = 0;
    if ((size + bitpos) > 32) hi = cpu.mmu.readData((ba + 4) | 0, L_LONG, cpu.accR());
    let combined = bitpos ? ((lo >>> bitpos) | (hi << (32 - bitpos))) : lo;
    return (combined & byteMask(size)) >>> 0;
}

/**
 * opInsv(cpu, d, ins, pos, size, rn, wd)
 *
 * op_insv (vax_cpu1.c): insert a 0..32 bit field.  Memory-mode reads use WRITE access, matching
 * SIMH's WA-flagged Read() -- the field's containing longword(s) are read-modify-written.
 *
 * @param {VAXCpu} cpu
 * @param {VAXDecoder} d
 * @param {number} ins the source field, unsigned
 * @param {number} pos
 * @param {number} size
 * @param {number} rn OP_MEM or a register number
 * @param {number} wd R[rn] (register mode) or the base address (memory mode)
 */
function opInsv(cpu, d, ins, pos, size, rn, wd)
{
    if (size === 0) return;
    if (size > 32) cpu.fault(VAXFAULT.RESOP);
    if (rn !== OP_MEM) {
        if ((pos >>> 0) > 31) cpu.fault(VAXFAULT.RESOP);
        if ((pos + size) > 32) {
            if (rn >= nSP) cpu.fault(VAXFAULT.RESAD);
            let mask = byteMask(pos + size - 32);
            let val = ins >>> (32 - pos);
            cpu.regs[rn + 1] = ((d.vfldrp1 & ~mask) | (val & mask)) | 0;
        }
        let mask2 = (byteMask(size) << pos) | 0;
        let val2 = (ins << pos) | 0;
        cpu.regs[rn] = ((cpu.regs[rn] & ~mask2) | (val2 & mask2)) | 0;
        return;
    }
    let ba = (wd + (pos >> 3)) | 0;
    let bitpos = (pos & 7) | ((ba & 3) << 3);
    ba = ba & ~3;
    let acc = cpu.accW();
    let loVal = cpu.mmu.readData(ba, L_LONG, acc);
    if ((bitpos + size) > 32) {
        let hiVal = cpu.mmu.readData((ba + 4) | 0, L_LONG, acc);
        let mask = byteMask(bitpos + size - 32);
        let val = ins >>> (32 - bitpos);
        cpu.mmu.writeData((ba + 4) | 0, ((hiVal & ~mask) | (val & mask)) | 0, L_LONG, acc);
    }
    let mask2 = (byteMask(size) << bitpos) | 0;
    let val2 = (ins << bitpos) | 0;
    cpu.mmu.writeData(ba, ((loVal & ~mask2) | (val2 & mask2)) | 0, L_LONG, acc);
}

/* ------------------------------------------------------------------------------------------- *
 * Store helpers -- decode.js's isMemoryDestination() docblock recipe, verbatim, plus the          *
 * quadword cross-page probe (WRITE_Q, vax_cpu.c:221-234).                                         *
 * ------------------------------------------------------------------------------------------- */

function storeB(cpu, d, r)
{
    if (d.isMemoryDestination()) cpu.mmu.writeData(d.va, r & BMASK, L_BYTE, cpu.accW());
    else cpu.regs[d.rn] = (cpu.regs[d.rn] & ~BMASK) | (r & BMASK);
}

function storeW(cpu, d, r)
{
    if (d.isMemoryDestination()) cpu.mmu.writeData(d.va, r & WMASK, L_WORD, cpu.accW());
    else cpu.regs[d.rn] = (cpu.regs[d.rn] & ~WMASK) | (r & WMASK);
}

function storeL(cpu, d, r)
{
    r = r | 0;
    if (d.isMemoryDestination()) cpu.mmu.writeData(d.va, r, L_LONG, cpu.accW());
    else cpu.regs[d.rn] = r;
}

function storeQ(cpu, d, lo, hi)
{
    lo = lo | 0; hi = hi | 0;
    if (d.isMemoryDestination()) {
        let acc = cpu.accW();
        let statHi = {code: 0}, statLo = {code: 0};
        let paHi = cpu.mmu.test((d.va + 7) | 0, acc, statHi);
        let paLo = cpu.mmu.test(d.va, acc, statLo);
        if (paHi >= 0 || paLo < 0) cpu.mmu.writeData(d.va, lo, L_LONG, acc);
        cpu.mmu.writeData((d.va + 4) | 0, hi, L_LONG, acc);
    } else {
        if (d.rn >= nSP) cpu.fault(VAXFAULT.RESAD);
        cpu.regs[d.rn] = lo;
        cpu.regs[d.rn + 1] = hi;
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Condition-code helpers -- transcribed from vax_defs.h's CC_* macros.  `cc` is threaded as a     *
 * plain local through each handler and folded into PSL<3:0> once, at the end of dispatch          *
 * (VAXCpu#execute), rather than SIMH's PSL-minus-cc-plus-cc-at-every-read trick: the two are       *
 * externally indistinguishable for a single-instruction register-file-plus-PSL comparison.         *
 * ------------------------------------------------------------------------------------------- */

function ccIIZZ_B(r) { r &= BMASK; return (r & BSIGN) ? CC_N : (r === 0 ? CC_Z : 0); }
function ccIIZZ_W(r) { r &= WMASK; return (r & WSIGN) ? CC_N : (r === 0 ? CC_Z : 0); }
function ccIIZZ_L(r) { return (r & LSIGN) ? CC_N : (r === 0 ? CC_Z : 0); }
function ccIIZZ_Q(rl, rh) { return (rh & LSIGN) ? CC_N : ((rl === 0 && rh === 0) ? CC_Z : 0); }

function ccIIZP_B(r, cc) { r &= BMASK; return (r & BSIGN) ? (CC_N | (cc & CC_C)) : (r === 0 ? (CC_Z | (cc & CC_C)) : (cc & CC_C)); }
function ccIIZP_W(r, cc) { r &= WMASK; return (r & WSIGN) ? (CC_N | (cc & CC_C)) : (r === 0 ? (CC_Z | (cc & CC_C)) : (cc & CC_C)); }
function ccIIZP_L(r, cc) { return (r & LSIGN) ? (CC_N | (cc & CC_C)) : (r === 0 ? (CC_Z | (cc & CC_C)) : (cc & CC_C)); }
function ccIIZP_Q(rl, rh, cc) { return (rh & LSIGN) ? (CC_N | (cc & CC_C)) : ((rl === 0 && rh === 0) ? (CC_Z | (cc & CC_C)) : (cc & CC_C)); }

function ccAdd_B(r, s1, s2)
{
    let cc = ccIIZZ_B(r);
    if (((~s1 ^ s2) & (s1 ^ r)) & BSIGN) cc |= CC_V;
    if ((r >>> 0) < (s2 >>> 0)) cc |= CC_C;
    return cc;
}
function ccAdd_W(r, s1, s2)
{
    let cc = ccIIZZ_W(r);
    if (((~s1 ^ s2) & (s1 ^ r)) & WSIGN) cc |= CC_V;
    if ((r >>> 0) < (s2 >>> 0)) cc |= CC_C;
    return cc;
}
function ccAdd_L(r, s1, s2)
{
    let cc = ccIIZZ_L(r);
    if (((~s1 ^ s2) & (s1 ^ r)) & LSIGN) cc |= CC_V;
    if ((r >>> 0) < (s2 >>> 0)) cc |= CC_C;
    return cc;
}

function ccSub_B(r, s1, s2)
{
    let cc = ccIIZZ_B(r);
    if (((s1 ^ s2) & (~s1 ^ r)) & BSIGN) cc |= CC_V;
    if ((s2 >>> 0) < (s1 >>> 0)) cc |= CC_C;
    return cc;
}
function ccSub_W(r, s1, s2)
{
    let cc = ccIIZZ_W(r);
    if (((s1 ^ s2) & (~s1 ^ r)) & WSIGN) cc |= CC_V;
    if ((s2 >>> 0) < (s1 >>> 0)) cc |= CC_C;
    return cc;
}
function ccSub_L(r, s1, s2)
{
    let cc = ccIIZZ_L(r);
    if (((s1 ^ s2) & (~s1 ^ r)) & LSIGN) cc |= CC_V;
    if ((s2 >>> 0) < (s1 >>> 0)) cc |= CC_C;
    return cc;
}

function ccCmp_B(s1, s2)
{
    let cc = (sxtB(s1) < sxtB(s2)) ? CC_N : (s1 === s2 ? CC_Z : 0);
    if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC_C;
    return cc;
}
function ccCmp_W(s1, s2)
{
    let cc = (sxtW(s1) < sxtW(s2)) ? CC_N : (s1 === s2 ? CC_Z : 0);
    if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC_C;
    return cc;
}
function ccCmp_L(s1, s2)
{
    let cc = (s1 < s2) ? CC_N : (s1 === s2 ? CC_Z : 0);
    if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC_C;
    return cc;
}

/* ------------------------------------------------------------------------------------------- *
 * VAXCpu -- register file, PSL, and the decoder/MMU plumbing.                                     *
 * ------------------------------------------------------------------------------------------- */

/**
 * @class VAXCpu
 * @property {Int32Array} regs R0-R15 (R12=AP, R13=FP, R14=SP, R15=PC)
 * @property {number} psl full PSL, condition codes included at all times (see the CC helper note)
 * @property {MMUVAX} mmu
 * @property {VAXDecoder} decoder
 */
class VAXCpu {
    /**
     * @param {BusVAX} bus
     */
    constructor(bus)
    {
        this.bus = bus;
        this.mmu = new MMUVAX(bus);
        this.regs = new Int32Array(16);
        this.psl = 0;
        this.decoder = new VAXDecoder(this);
    }

    get cc() { return this.psl & CC_MASK; }

    setCC(cc) { this.psl = (this.psl & ~CC_MASK) | (cc & CC_MASK); }

    /**
     * curMode() -- PSL<25:24>, SIMH's PSL_GETCUR.
     * @returns {number} 0=Kernel 1=Executive 2=Supervisor 3=User
     */
    curMode() { return (this.psl >>> 24) & 3; }

    accR() { return MMUVAX.accRead(this.curMode()); }
    accW() { return MMUVAX.accWrite(this.curMode()); }

    /**
     * fault(code)
     * @param {number} code one of VAXFAULT.*
     * @throws {VAXFault}
     */
    fault(code) { throw new VAXFault(code); }

    /* --- VAXDecodeMachine interface, consumed by decode.js --- */

    /**
     * @param {number} lnt 1, 2 or 4
     * @returns {number}
     */
    getISTR(lnt)
    {
        let v = this.mmu.readData(this.regs[nPC], lnt, this.accR());
        this.regs[nPC] = (this.regs[nPC] + lnt) | 0;
        return v;
    }

    /**
     * @param {number} pc
     */
    setPC(pc) { this.regs[nPC] = pc | 0; }

    /**
     * @param {number} va
     * @param {number} lnt
     * @param {boolean} fWrite
     * @returns {number}
     */
    readData(va, lnt, fWrite)
    {
        return this.mmu.readData(va, lnt, fWrite ? this.accW() : this.accR());
    }

    /**
     * step()
     *
     * Decode one instruction and execute it.  Does NOT unwind the recovery queue or dispatch an
     * exception on fault -- see the file header for why that is out of scope, and tests/intdiff.js
     * for how the test harness handles it (it simply does not generate fault-triggering states for
     * the correctness corpus, and reports separately any state that would).
     */
    step()
    {
        this.decoder.decode(false);
        this.execute();
    }

    execute()
    {
        if ((this.psl & PSW_T) && !(this.psl & PSL_TP)) this.psl |= PSL_TP;
        let d = this.decoder;
        let fn = DISPATCH[d.opc];
        if (!fn) throw new Error(`cpu.js: opcode ${d.opc.toString(16)} (${OPCODES[d.opc] || "?"}) is not implemented by this item -- see the file header for which item owns it`);
        fn(this, d);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Instruction bodies -- one function per drom.js specifier shape, shared across mnemonics that    *
 * use it identically (exactly mirroring how vax_cpu.c falls multiple `case` labels into one        *
 * body).  `op0..op6` name opnd[] slots the way vax_cpu.c's #defines do, for side-by-side reading.  *
 * ------------------------------------------------------------------------------------------- */

const H = {};   // mnemonic -> handler

/* --- Single-operand, write-only: CLRx dst.wx --- */

/* CC_ZZ1P (vax_defs.h): cc = CC_Z | (cc & CC_C) -- the incoming CARRY bit survives a clear. */
H.CLRB = (cpu, d) => { storeB(cpu, d, 0); cpu.setCC(CC_Z | (cpu.cc & CC_C)); };
H.CLRW = (cpu, d) => { storeW(cpu, d, 0); cpu.setCC(CC_Z | (cpu.cc & CC_C)); };
H.CLRL = (cpu, d) => { storeL(cpu, d, 0); cpu.setCC(CC_Z | (cpu.cc & CC_C)); };
H.CLRQ = (cpu, d) => { storeQ(cpu, d, 0, 0); cpu.setCC(CC_Z | (cpu.cc & CC_C)); };

/* --- Single-operand, read-only: TSTx src.rx --- */

H.TSTB = (cpu, d) => { cpu.setCC(ccIIZZ_B(d.opnd[0])); };
H.TSTW = (cpu, d) => { cpu.setCC(ccIIZZ_W(d.opnd[0])); };
H.TSTL = (cpu, d) => { cpu.setCC(ccIIZZ_L(d.opnd[0])); };

/* --- Single-operand, read/write: INCx/DECx src.mx --- */

H.INCB = (cpu, d) => { let op0 = d.opnd[0], r = (op0 + 1) & BMASK; storeB(cpu, d, r); cpu.setCC(ccAdd_B(r, 1, op0)); };
H.INCW = (cpu, d) => { let op0 = d.opnd[0], r = (op0 + 1) & WMASK; storeW(cpu, d, r); cpu.setCC(ccAdd_W(r, 1, op0)); };
H.INCL = (cpu, d) => { let op0 = d.opnd[0], r = (op0 + 1) | 0; storeL(cpu, d, r); cpu.setCC(ccAdd_L(r, 1, op0)); };
H.DECB = (cpu, d) => { let op0 = d.opnd[0], r = (op0 - 1) & BMASK; storeB(cpu, d, r); cpu.setCC(ccSub_B(r, 1, op0)); };
H.DECW = (cpu, d) => { let op0 = d.opnd[0], r = (op0 - 1) & WMASK; storeW(cpu, d, r); cpu.setCC(ccSub_W(r, 1, op0)); };
H.DECL = (cpu, d) => { let op0 = d.opnd[0], r = (op0 - 1) | 0; storeL(cpu, d, r); cpu.setCC(ccSub_L(r, 1, op0)); };

/* --- Push: PUSHL src.rl, PUSHAx src.ax --- */

H.PUSHL = (cpu, d) => {
    let op0 = d.opnd[0];
    let sp = (cpu.regs[nSP] - 4) | 0;
    cpu.mmu.writeData(sp, op0, L_LONG, cpu.accW());
    cpu.regs[nSP] = sp;
    cpu.setCC(ccIIZP_L(op0, cpu.cc));
};
H.PUSHAB = H.PUSHAW = H.PUSHAL = H.PUSHAQ = H.PUSHL;

/* --- Moves, converts, ADAWI: op src.rx, dst.wx --- */

H.MOVB = (cpu, d) => { let r = d.opnd[0]; storeB(cpu, d, r); cpu.setCC(ccIIZP_B(r, cpu.cc)); };
H.MOVW = (cpu, d) => { let r = d.opnd[0]; storeW(cpu, d, r); cpu.setCC(ccIIZP_W(r, cpu.cc)); };
H.MOVZBW = H.MOVW;
H.MOVL = (cpu, d) => { let r = d.opnd[0] | 0; storeL(cpu, d, r); cpu.setCC(ccIIZP_L(r, cpu.cc)); };
H.MOVZBL = H.MOVZWL = H.MOVAB = H.MOVAW = H.MOVAL = H.MOVAQ = H.MOVL;

H.MCOMB = (cpu, d) => { let r = d.opnd[0] ^ BMASK; storeB(cpu, d, r); cpu.setCC(ccIIZP_B(r, cpu.cc)); };
H.MCOMW = (cpu, d) => { let r = d.opnd[0] ^ WMASK; storeW(cpu, d, r); cpu.setCC(ccIIZP_W(r, cpu.cc)); };
H.MCOML = (cpu, d) => { let r = (d.opnd[0] ^ LMASK) | 0; storeL(cpu, d, r); cpu.setCC(ccIIZP_L(r, cpu.cc)); };

H.MNEGB = (cpu, d) => { let op0 = d.opnd[0], r = (-op0) & BMASK; storeB(cpu, d, r); cpu.setCC(ccSub_B(r, op0, 0)); };
H.MNEGW = (cpu, d) => { let op0 = d.opnd[0], r = (-op0) & WMASK; storeW(cpu, d, r); cpu.setCC(ccSub_W(r, op0, 0)); };
H.MNEGL = (cpu, d) => { let op0 = d.opnd[0], r = (-op0) | 0; storeL(cpu, d, r); cpu.setCC(ccSub_L(r, op0, 0)); };

H.CVTBW = (cpu, d) => { let r = sxtB(d.opnd[0]) & WMASK; storeW(cpu, d, r); cpu.setCC(ccIIZZ_W(r)); };
H.CVTBL = (cpu, d) => { let r = sxtB(d.opnd[0]) | 0; storeL(cpu, d, r); cpu.setCC(ccIIZZ_L(r)); };
H.CVTWL = (cpu, d) => { let r = sxtW(d.opnd[0]) | 0; storeL(cpu, d, r); cpu.setCC(ccIIZZ_L(r)); };

H.CVTLB = (cpu, d) => {
    let op0 = d.opnd[0] | 0, r = op0 & BMASK;
    storeB(cpu, d, r);
    let cc = ccIIZZ_B(r);
    if (op0 > 127 || op0 < -128) cc |= CC_V;
    cpu.setCC(cc);
};
H.CVTLW = (cpu, d) => {
    let op0 = d.opnd[0] | 0, r = op0 & WMASK;
    storeW(cpu, d, r);
    let cc = ccIIZZ_W(r);
    if (op0 > 32767 || op0 < -32768) cc |= CC_V;
    cpu.setCC(cc);
};
H.CVTWB = (cpu, d) => {
    let op0 = d.opnd[0], r = op0 & BMASK;
    storeB(cpu, d, r);
    let cc = ccIIZZ_B(r);
    let temp = sxtW(op0);
    if (temp > 127 || temp < -128) cc |= CC_V;
    cpu.setCC(cc);
};

H.ADAWI = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1], op2 = d.opnd[2];
    let temp;
    if (op1 >= 0) temp = cpu.regs[op1] & WMASK;
    else {
        if (op2 & 1) cpu.fault(VAXFAULT.RESOP);
        temp = cpu.mmu.readData(op2, L_WORD, cpu.accW());
    }
    let r = (op0 + temp) & WMASK;
    storeW(cpu, d, r);
    cpu.setCC(ccAdd_W(r, op0, temp));
};

/* --- Two-operand, read-only: CMPx src1.rx, src2.rx; BITx src1.rx, src2.rx --- */

H.CMPB = (cpu, d) => { cpu.setCC(ccCmp_B(d.opnd[0], d.opnd[1])); };
H.CMPW = (cpu, d) => { cpu.setCC(ccCmp_W(d.opnd[0], d.opnd[1])); };
H.CMPL = (cpu, d) => { cpu.setCC(ccCmp_L(d.opnd[0], d.opnd[1])); };

H.BITB = (cpu, d) => { let r = d.opnd[1] & d.opnd[0]; cpu.setCC(ccIIZP_B(r, cpu.cc)); };
H.BITW = (cpu, d) => { let r = d.opnd[1] & d.opnd[0]; cpu.setCC(ccIIZP_W(r, cpu.cc)); };
H.BITL = (cpu, d) => { let r = (d.opnd[1] & d.opnd[0]) | 0; cpu.setCC(ccIIZP_L(r, cpu.cc)); };

/* --- Two/three-operand read/write arithmetic and logical: op2 src.rx,dst.mx; op3 s1.rx,s2.rx,dst.wx --- */

function mkAdd(width)
{
    let mask = width === 1 ? BMASK : width === 2 ? WMASK : LMASK;
    let store = width === 1 ? storeB : width === 2 ? storeW : storeL;
    let ccAdd = width === 1 ? ccAdd_B : width === 2 ? ccAdd_W : ccAdd_L;
    return (cpu, d) => {
        let op0 = d.opnd[0], op1 = d.opnd[1];
        let r = width === 4 ? ((op1 + op0) | 0) : ((op1 + op0) & mask);
        store(cpu, d, r);
        cpu.setCC(ccAdd(r, op0, op1));
    };
}
function mkSub(width)
{
    let mask = width === 1 ? BMASK : width === 2 ? WMASK : LMASK;
    let store = width === 1 ? storeB : width === 2 ? storeW : storeL;
    let ccSub = width === 1 ? ccSub_B : width === 2 ? ccSub_W : ccSub_L;
    return (cpu, d) => {
        let op0 = d.opnd[0], op1 = d.opnd[1];
        let r = width === 4 ? ((op1 - op0) | 0) : ((op1 - op0) & mask);
        store(cpu, d, r);
        cpu.setCC(ccSub(r, op0, op1));
    };
}
function mkBis(width)
{
    let mask = width === 1 ? BMASK : width === 2 ? WMASK : LMASK;
    let store = width === 1 ? storeB : width === 2 ? storeW : storeL;
    let ccIIZP = width === 1 ? ccIIZP_B : width === 2 ? ccIIZP_W : ccIIZP_L;
    return (cpu, d) => {
        let r = (d.opnd[1] | d.opnd[0]) & mask;
        store(cpu, d, r);
        cpu.setCC(ccIIZP(r, cpu.cc));
    };
}
function mkBic(width)
{
    let mask = width === 1 ? BMASK : width === 2 ? WMASK : LMASK;
    let store = width === 1 ? storeB : width === 2 ? storeW : storeL;
    let ccIIZP = width === 1 ? ccIIZP_B : width === 2 ? ccIIZP_W : ccIIZP_L;
    return (cpu, d) => {
        let r = (d.opnd[1] & ~d.opnd[0]) & mask;
        store(cpu, d, r);
        cpu.setCC(ccIIZP(r, cpu.cc));
    };
}
function mkXor(width)
{
    let mask = width === 1 ? BMASK : width === 2 ? WMASK : LMASK;
    let store = width === 1 ? storeB : width === 2 ? storeW : storeL;
    let ccIIZP = width === 1 ? ccIIZP_B : width === 2 ? ccIIZP_W : ccIIZP_L;
    return (cpu, d) => {
        let r = (d.opnd[1] ^ d.opnd[0]) & mask;
        store(cpu, d, r);
        cpu.setCC(ccIIZP(r, cpu.cc));
    };
}
function mkMulShort(width)
{
    let sxt = width === 1 ? sxtB : sxtW;
    let mask = width === 1 ? BMASK : WMASK;
    let store = width === 1 ? storeB : storeW;
    let ccIIZZ = width === 1 ? ccIIZZ_B : ccIIZZ_W;
    let lo = width === 1 ? -128 : -32768, hi = width === 1 ? 127 : 32767;
    return (cpu, d) => {
        let temp = sxt(d.opnd[0]) * sxt(d.opnd[1]);
        let r = temp & mask;
        store(cpu, d, r);
        let cc = ccIIZZ(r);
        if (temp > hi || temp < lo) cc |= CC_V;
        cpu.setCC(cc);
    };
}
function mkDivShort(width)
{
    let sxt = width === 1 ? sxtB : sxtW;
    let mask = width === 1 ? BMASK : WMASK;
    let sign = width === 1 ? BSIGN : WSIGN;
    let store = width === 1 ? storeB : storeW;
    let ccIIZZ = width === 1 ? ccIIZZ_B : ccIIZZ_W;
    return (cpu, d) => {
        let op0 = d.opnd[0], op1 = d.opnd[1];
        let r, cc = 0;
        if (op0 === 0) { r = op1; cc = CC_V; }
        else if (op0 === mask && op1 === sign) { r = op1; cc = CC_V; }
        else r = Math.trunc(sxt(op1) / sxt(op0));
        r &= mask;
        store(cpu, d, r);
        cpu.setCC(ccIIZZ(r) | cc);
    };
}

H.ADDB2 = H.ADDB3 = mkAdd(1);
H.ADDW2 = H.ADDW3 = mkAdd(2);
H.ADDL2 = H.ADDL3 = mkAdd(4);
H.SUBB2 = H.SUBB3 = mkSub(1);
H.SUBW2 = H.SUBW3 = mkSub(2);
H.SUBL2 = H.SUBL3 = mkSub(4);
H.BISB2 = H.BISB3 = mkBis(1);
H.BISW2 = H.BISW3 = mkBis(2);
H.BISL2 = H.BISL3 = mkBis(4);
H.BICB2 = H.BICB3 = mkBic(1);
H.BICW2 = H.BICW3 = mkBic(2);
H.BICL2 = H.BICL3 = mkBic(4);
H.XORB2 = H.XORB3 = mkXor(1);
H.XORW2 = H.XORW3 = mkXor(2);
H.XORL2 = H.XORL3 = mkXor(4);
H.MULB2 = H.MULB3 = mkMulShort(1);
H.MULW2 = H.MULW3 = mkMulShort(2);
H.DIVB2 = H.DIVB3 = mkDivShort(1);
H.DIVW2 = H.DIVW3 = mkDivShort(2);

H.MULL2 = H.MULL3 = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1];
    let {lo: r, hi: rh} = mul32signed(op0, op1);
    storeL(cpu, d, r);
    let cc = ccIIZZ_L(r);
    if (rh !== ((r & LSIGN) ? -1 : 0)) cc |= CC_V;
    cpu.setCC(cc);
};

H.DIVL2 = H.DIVL3 = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1];
    let r, cc = 0;
    if (op0 === 0) { r = op1; cc = CC_V; }
    else if ((op0 >>> 0) === LMASK && (op1 >>> 0) === LSIGN) { r = op1; cc = CC_V; }
    else r = Math.trunc(op1 / op0);
    r = r | 0;
    storeL(cpu, d, r);
    cpu.setCC(ccIIZZ_L(r) | cc);
};

H.ADWC = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1];
    let r = (op1 + op0 + (cpu.cc & CC_C)) | 0;
    storeL(cpu, d, r);
    let cc = ccAdd_L(r, op0, op1);
    if (r === op1 && op0 !== 0) cc |= CC_C;
    cpu.setCC(cc);
};
H.SBWC = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1];
    let r = (op1 - op0 - (cpu.cc & CC_C)) | 0;
    storeL(cpu, d, r);
    let cc = ccSub_L(r, op0, op1);
    if (op0 === op1 && r !== 0) cc |= CC_C;
    cpu.setCC(cc);
};

/* --- MOVQ: movq src.rq, dst.wq --- */

H.MOVQ = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1];
    storeQ(cpu, d, op0, op1);
    cpu.setCC(ccIIZP_Q(op0, op1, cpu.cc));
};

/* --- Shifts --- */

H.ROTL = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1];
    let j = ((op0 % 32) + 32) % 32;
    let r = j ? (((op1 << j) | (op1 >>> (32 - j))) | 0) : op1;
    storeL(cpu, d, r);
    cpu.setCC(ccIIZP_L(r, cpu.cc));
};

H.ASHL = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1];
    let r, cc;
    if (op0 & BSIGN) {
        let n = (0x100 - op0);
        r = (n > 31) ? ((op1 & LSIGN) ? -1 : 0) : (op1 >> n);
        cc = ccIIZZ_L(r);
    } else {
        /* SIMH's `if (op1 != temp) V_INTOV;` is OUTSIDE the op0>31 vs op0<=31 split -- a shift
           that discards every bit of a nonzero op1 (op0>31, temp forced to 0) is exactly as much
           an overflow as one that discards some of them. */
        let temp;
        if (op0 > 31) { r = 0; temp = 0; }
        else { r = (op1 << op0) | 0; temp = r >> op0; }
        cc = ccIIZZ_L(r);
        if (op1 !== temp) cc |= CC_V;
    }
    storeL(cpu, d, r);
    cpu.setCC(cc);
};

H.ASHQ = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1], op2 = d.opnd[2];
    let {lo: r, hi: rh, flg} = opAshq(op0, op1, op2);
    storeQ(cpu, d, r, rh);
    let cc = ccIIZZ_Q(r, rh);
    if (flg) cc |= CC_V;
    cpu.setCC(cc);
};

H.EMUL = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1], op2 = d.opnd[2];
    let {lo: r, hi: rh} = opEmul(op0, op1, op2);
    storeQ(cpu, d, r, rh);
    cpu.setCC(ccIIZZ_Q(r, rh));
};

H.EDIV = (cpu, d) => {
    let op0 = d.opnd[0], op1 = d.opnd[1], op2 = d.opnd[2];
    let op3 = d.opnd[3], op4 = d.opnd[4], op5 = d.opnd[5], op6 = d.opnd[6];
    if (op5 < 0) cpu.mmu.readData(op6, L_LONG, cpu.accW());
    let r, rh, flg;
    if (op0 === 0) { flg = CC_V; r = op1; rh = 0; }
    else { let res = opEdiv(op0, op1, op2); r = res.quo; rh = res.rem; flg = res.flg; }
    if (op3 >= 0) cpu.regs[op3] = r; else cpu.mmu.writeData(op4, r, L_LONG, cpu.accW());
    if (op5 >= 0) cpu.regs[op5] = rh; else cpu.mmu.writeData(op6, rh, L_LONG, cpu.accW());
    cpu.setCC(ccIIZZ_L(r) | flg);
};

/* --- PSW instructions --- */

H.BISPSW = (cpu, d) => {
    let op0 = d.opnd[0];
    if (op0 & PSW_MBZ) cpu.fault(VAXFAULT.RESOP);
    cpu.psl = cpu.psl | (op0 & ~CC_MASK);
    cpu.setCC(cpu.cc | (op0 & CC_MASK));
};
H.BICPSW = (cpu, d) => {
    let op0 = d.opnd[0];
    if (op0 & PSW_MBZ) cpu.fault(VAXFAULT.RESOP);
    cpu.psl = cpu.psl & ~op0;
    cpu.setCC(cpu.cc & ~op0);
};
H.MOVPSL = (cpu, d) => { storeL(cpu, d, cpu.psl); };

/* --- Variable-length bit field instructions --- */

H.EXTV = (cpu, d) => {
    let pos = d.opnd[0], size = d.opnd[1], rn = d.opnd[2], wd = d.opnd[3];
    let r = opExtv(cpu, d, pos, size, rn, wd);
    if (r & byteSign(size)) r = r | ~byteMask(size);
    r = r | 0;
    storeL(cpu, d, r);
    cpu.setCC(ccIIZP_L(r, cpu.cc));
};
H.EXTZV = (cpu, d) => {
    let pos = d.opnd[0], size = d.opnd[1], rn = d.opnd[2], wd = d.opnd[3];
    let r = opExtv(cpu, d, pos, size, rn, wd) | 0;
    storeL(cpu, d, r);
    cpu.setCC(ccIIZP_L(r, cpu.cc));
};
H.CMPV = (cpu, d) => {
    let pos = d.opnd[0], size = d.opnd[1], rn = d.opnd[2], wd = d.opnd[3], op4 = d.opnd[4];
    let r = opExtv(cpu, d, pos, size, rn, wd);
    if (r & byteSign(size)) r = r | ~byteMask(size);
    cpu.setCC(ccCmp_L(r | 0, op4));
};
H.CMPZV = (cpu, d) => {
    let pos = d.opnd[0], size = d.opnd[1], rn = d.opnd[2], wd = d.opnd[3], op4 = d.opnd[4];
    let r = opExtv(cpu, d, pos, size, rn, wd);
    cpu.setCC(ccCmp_L(r | 0, op4));
};
H.FFS = (cpu, d) => {
    let pos = d.opnd[0], size = d.opnd[1], rn = d.opnd[2], wd = d.opnd[3];
    let r = opExtv(cpu, d, pos, size, rn, wd);
    let temp = opFfs(r, size);
    storeL(cpu, d, (pos + temp) | 0);
    cpu.setCC(r ? 0 : CC_Z);
};
H.FFC = (cpu, d) => {
    let pos = d.opnd[0], size = d.opnd[1], rn = d.opnd[2], wd = d.opnd[3];
    let r = opExtv(cpu, d, pos, size, rn, wd);
    r = r ^ byteMask(size);
    let temp = opFfs(r, size);
    storeL(cpu, d, (pos + temp) | 0);
    cpu.setCC(r ? 0 : CC_Z);
};
H.INSV = (cpu, d) => {
    let ins = d.opnd[0], pos = d.opnd[1], size = d.opnd[2], rn = d.opnd[3], wd = d.opnd[4];
    opInsv(cpu, d, ins, pos, size, rn, wd);
};

/* ------------------------------------------------------------------------------------------- *
 * DISPATCH -- built positionally from drom.js's OPCODES, so a handler is wired to EXACTLY the     *
 * opcode number the generated decode ROM assigns its mnemonic, and can never silently drift.      *
 * ------------------------------------------------------------------------------------------- */

const DISPATCH = new Array(512).fill(null);
for (let opc = 0; opc < 512; opc++) {
    let mn = OPCODES[opc];
    if (mn && H[mn]) DISPATCH[opc] = H[mn];
}

/** Mnemonics this file wires up, for a test harness to enumerate without re-deriving the list. */
const IMPLEMENTED = Object.keys(H);

export default VAXCpu;
export { VAXCpu, DISPATCH, IMPLEMENTED, H as HANDLERS, L_BYTE, L_WORD, L_LONG, CC_C, CC_V, CC_Z, CC_N, CC_MASK, PSW_MBZ };
