/**
 * @fileoverview VAX F_floating, D_floating and G_floating arithmetic
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
 * A direct port of Open SIMH `VAX/vax_fpa.c` -- specifically of its `#else` (32-bit) code path,
 * the one Supnik wrote for hosts without a 64-bit integer type.  That path is not a fallback
 * here, it is the RIGHT path: JavaScript's bitwise operators are exactly 32 bits, so the `UDP`
 * {hi, lo} pair and the `dp_*` routines that operate on it translate one-for-one, whereas the
 * `USE_INT64` path would need BigInt in the hot loop of every floating instruction.
 *
 * The two paths in vax_fpa.c are supposed to compute identical results.  Our differential
 * (tests/fpadiff.js) grades this port against a `microvax3900` binary built with `-DUSE_INT64`
 * (makefile:1560), i.e. against the OTHER path -- so a zero-divergence run is also an empirical
 * check of that claim, over deliberately generated denormal, rounding-boundary, overflow and
 * underflow inputs.  Two places where the paths are written differently but provably agree, in
 * case a future reader wonders:
 *
 *   vax_fdiv  the 64b loop exits early on `b->frac == 0` and shifts the quotient by
 *             `UF_V_NM - i + 1`; the 32b loop always runs `prec` times and shifts by
 *             `UF_V_NM - prec + 1`.  Both leave the quotient's most significant 1 at bit 63.
 *   vax_fadd  the 64b effective-subtract denormalizes with an ARITHMETIC right shift and then
 *             ORs ones into the top `ediff` bits; the 32b path uses a LOGICAL right shift and
 *             ORs the same ones in.  The OR makes the two identical.
 *
 * WHY THIS IS A PORT AND NOT A DERIVATION
 * ---------------------------------------
 * DEC floating point is not IEEE.  Different bias, different bit layout (the fraction is stored
 * word-swapped), no infinities, no NaNs, a "reserved operand" encoding where IEEE has negative
 * zero, and rounding that is add-a-half-then-truncate on a guard-extended fraction rather than
 * round-to-nearest-even.  Supnik already got the edges right.  Where a JavaScript idiom would be
 * cleaner but would move a rounding or normalization boundary, SIMH's form is kept and annotated.
 *
 * THE FORMATS (vax_defs.h:159-175)
 * --------------------------------
 * All three store the fraction WORD-SWAPPED: the most significant fraction bits live in the LOW
 * word of the low longword, next in its high word, and so on.  `WORDSWAP()` below is what puts
 * them back in order, and it is the reason a "fraction" here is never simply `val & mask`.
 *
 *      F  32b   sign<15>  exp<14:7>  frac<6:0>,<31:16>            bias 0x80,  hidden bit at 7
 *      D  64b   as F plus 32 more fraction bits in the high lw    bias 0x80,  55 bits of fraction
 *      G  64b   sign<15>  exp<14:4>  frac<3:0>,<31:16>,+32        bias 0x400, hidden bit at 4
 *
 * An exponent of 0 with sign 0 is a true zero.  An exponent of 0 with sign 1 is the RESERVED
 * OPERAND and faults -- it is not "negative zero", and treating it as one is the single most
 * common way to get DEC floating point wrong.  H_floating is deliberately NOT here: see SCOPE.
 *
 * SCOPE -- MEASURED, NOT ASSUMED
 * ------------------------------
 * F, D and G only.  Per docs/reference/ehkaa-profile.md §7, every one of the fifteen opcodes the
 * EHKAA hardware core diagnostic never executes is H_floating (ADDH2/3, CVTBH, CVTHB, CVTHL,
 * CVTHW, CVTLH, CVTRHL, CVTWH, DIVH2/3, MULH2/3, SUBH2/3); the fourteen Extended-Accuracy-Group
 * opcodes it DOES execute are G.  Also out of scope, and also for a measured reason: ACBD/F/G/H,
 * EMODD/F/G and POLYD/F/G are never executed either.  `vax_fmod()` and the `mhi`/`mlo` masking
 * arguments of `vaxFadd()`/`vaxFmul()` exist only to serve EMOD and POLY; the masks are ported
 * (they cost nothing and keep the routines identical to SIMH's) but no caller here passes a
 * non-zero one, and `op_emod*`/`op_poly*` are not implemented.
 *
 * ARITHMETIC CONVENTION -- WHERE THE SIGN BITES
 * ---------------------------------------------
 * See defines.js for the address convention; this module inherits it but is mostly about DATA.
 * The rule specific to floating point: a 64-bit fraction is carried as two UNSIGNED 32-bit halves
 * (`hi`, `lo`), each normalized with `>>> 0` after every operation that could set bit 31.  This is
 * not decoration.  `dp_cmp()`, the carry test in `dp_add()`, the borrow test in `dp_sub()` and the
 * `b->frac >= a->frac` divide step are all UNSIGNED comparisons in C, and every one of them is
 * silently wrong if the operand is a negative int32.  Longword DATA elsewhere in VAXjs is signed
 * int32; inside a UFP it is not, and the `>>> 0` at the end of each `dp_*` line is what enforces
 * that.  Values that leave this module (results, condition codes) are int32 again.
 * ============================================================================
 */

import { VAXFault, VAXFAULT } from "./decode.js";

/*
 * Floating point formats -- vax_defs.h:159-175, verbatim.
 */
const FPSIGN        = 0x00008000;               // sign bit, in the LOW word of the low longword

const FD_V_EXP      = 7;
const FD_M_EXP      = 0xFF;
const FD_BIAS       = 0x80;
const FD_EXP        = FD_M_EXP << FD_V_EXP;     // 0x7F80
const FD_HB         = 1 << FD_V_EXP;            // 0x80, the hidden bit
const FD_GUARD      = 15 - FD_V_EXP;            // 8 guard bits

const G_V_EXP       = 4;
const G_M_EXP       = 0x7FF;
const G_BIAS        = 0x400;
const G_EXP         = G_M_EXP << G_V_EXP;       // 0x7FF0
const G_HB          = 1 << G_V_EXP;             // 0x10
const G_GUARD       = 15 - G_V_EXP;             // 11 guard bits

/*
 * Unpacked fraction constants -- vax_fpa.c:633-640 (the 32b path).  UF_NM_H is the normalized
 * bit; the *_RND_* pairs are the value added to the guard-extended fraction to round it.
 */
const UF_NM_H       = 0x80000000;
const UF_FRND_H     = 0x00000080, UF_FRND_L = 0x00000000;
const UF_DRND_H     = 0x00000000, UF_DRND_L = 0x00000080;
const UF_GRND_H     = 0x00000000, UF_GRND_L = 0x00000400;
const UF_V_NM       = 63;

/*
 * PSL bits and trap/condition codes this module reads or writes -- vax_defs.h:233-239, 299-308,
 * 317-319.
 */
const PSW_FU        = 0x40;                     // floating underflow enable
const PSW_IV        = 0x20;                     // integer overflow enable
const PSL_M_IPL     = 0x1F;

const CC_N          = 0x08;
const CC_Z          = 0x04;
const CC_V          = 0x02;
const CC_C          = 0x01;

const TIR_V_TRAP    = 5;
const TRAP_INTOV    = 1 << TIR_V_TRAP;

const FLT_OVRFLO    = 0x8;
const FLT_DIVZRO    = 0x9;
const FLT_UNDFLO    = 0xA;

/*
 * ABORT_ARITH (vax_defs.h:77) is the negated SCB_ARITH offset, exactly like the codes decode.js
 * publishes in VAXFAULT.  It is defined here rather than added to VAXFAULT only because three
 * items are editing this tree concurrently and this file owes them no merge conflict; fold it in
 * when they land.  The `p1` carried with it is the FLT_* code above, which is what the exception
 * handler pushes as the trap parameter.
 */
const ABORT_ARITH   = -0x34;

const BMASK = 0xFF, WMASK = 0xFFFF, WSIGN = 0x8000, LMASK = 0xFFFFFFFF;
const GRN = 0x050;                              // vax_defs.h:504 -- register-mode specifier base
const nSP = 14, nPC = 15;

/*
 * Opcodes -- vax_defs.h:627-670.  Only the F/D/G subset this module implements.  `opc & 0x100`
 * selects G in the shared convert routines, `opc & 0x20` selects D over F, and `opc & 3` is the
 * destination length code for float-to-integer (0=B, 1=W, 2=L, 3=L with rounding).  Those three
 * encodings are properties of the opcode assignment, not conveniences -- vax_fpa.c relies on all
 * three, so do not renumber.
 */
const OPC = {
    ADDF2: 0x40, ADDF3: 0x41, SUBF2: 0x42, SUBF3: 0x43,
    MULF2: 0x44, MULF3: 0x45, DIVF2: 0x46, DIVF3: 0x47,
    CVTFB: 0x48, CVTFW: 0x49, CVTFL: 0x4A, CVTRFL: 0x4B,
    CVTBF: 0x4C, CVTWF: 0x4D, CVTLF: 0x4E,
    MOVF:  0x50, CMPF:  0x51, MNEGF: 0x52, TSTF:  0x53, CVTFD: 0x56,

    ADDD2: 0x60, ADDD3: 0x61, SUBD2: 0x62, SUBD3: 0x63,
    MULD2: 0x64, MULD3: 0x65, DIVD2: 0x66, DIVD3: 0x67,
    CVTDB: 0x68, CVTDW: 0x69, CVTDL: 0x6A, CVTRDL: 0x6B,
    CVTBD: 0x6C, CVTWD: 0x6D, CVTLD: 0x6E,
    MOVD:  0x70, CMPD:  0x71, MNEGD: 0x72, TSTD:  0x73, CVTDF: 0x76,

    CVTGF: 0x133, CVTFG: 0x199,

    ADDG2: 0x140, ADDG3: 0x141, SUBG2: 0x142, SUBG3: 0x143,
    MULG2: 0x144, MULG3: 0x145, DIVG2: 0x146, DIVG3: 0x147,
    CVTGB: 0x148, CVTGW: 0x149, CVTGL: 0x14A, CVTRGL: 0x14B,
    CVTBG: 0x14C, CVTWG: 0x14D, CVTLG: 0x14E,
    MOVG:  0x150, CMPG:  0x151, MNEGG: 0x152, TSTG:  0x153
};

/**
 * WORDSWAP(x) -- vax_fpa.c:620.
 *
 * Exchange the halves of a longword.  This is what converts between the VAX's stored,
 * word-swapped fraction and a plain most-significant-bit-first fraction.  Returned unsigned.
 *
 * @param {number} x
 * @returns {number} uint32
 */
function WORDSWAP(x)
{
    return (((x & WMASK) << 16) | ((x >>> 16) & WMASK)) >>> 0;
}

/**
 * SXTB/SXTW -- vax_defs.h:672-673.
 *
 * @param {number} x
 * @returns {number} int32
 */
function SXTB(x) { return (x & 0x80)? (x | ~BMASK) : (x & BMASK); }
function SXTW(x) { return (x & WSIGN)? (x | ~WMASK) : (x & WMASK); }

/**
 * An unpacked floating point value: SIMH's UFP with its UDP fraction flattened into two fields.
 *
 * `hi` and `lo` are UNSIGNED 32-bit values (see ARITHMETIC CONVENTION in the file header).  `exp`
 * is the BIASED exponent as an ordinary small integer, and may go negative or above the format
 * maximum during arithmetic -- that is exactly how underflow and overflow are detected in
 * rpackfd()/rpackg(), so it must not be clamped.  `sign` is 0 or FPSIGN, never a boolean.
 *
 * @class UFP
 * @property {number} sign
 * @property {number} exp
 * @property {number} hi
 * @property {number} lo
 */
class UFP {
    constructor()
    {
        this.sign = 0;
        this.exp = 0;
        this.hi = 0;
        this.lo = 0;
    }

    /**
     * copyFrom(b)
     *
     * SIMH's `*a = *b` on a UFP.
     *
     * @param {UFP} b
     */
    copyFrom(b)
    {
        this.sign = b.sign; this.exp = b.exp; this.hi = b.hi; this.lo = b.lo;
    }

    /**
     * swapWith(b)
     *
     * SIMH's three-line `t = *a; *a = *b; *b = t`.
     *
     * @param {UFP} b
     */
    swapWith(b)
    {
        let s = this.sign, e = this.exp, h = this.hi, l = this.lo;
        this.sign = b.sign; this.exp = b.exp; this.hi = b.hi; this.lo = b.lo;
        b.sign = s; b.exp = e; b.hi = h; b.lo = l;
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Double precision integer routines -- vax_fpa.c:1067-1185
 *
 * Each takes objects with `hi` and `lo` UNSIGNED 32-bit fields, so a UFP can be passed directly
 * (its sign and exp are untouched) exactly as SIMH passes `&a->frac`.  Every assignment ends in
 * `>>> 0` because every one of these is unsigned arithmetic in C, and the comparisons below are
 * unsigned comparisons that a negative int32 would invert.
 * ------------------------------------------------------------------------------------------- */

/**
 * dpCmp(a, b) -- vax_fpa.c:1067
 *
 * @param {{hi:number,lo:number}} a
 * @param {{hi:number,lo:number}} b
 * @returns {number} -1, 0 or +1
 */
function dpCmp(a, b)
{
    if (a.hi < b.hi) return -1;
    if (a.hi > b.hi) return +1;
    if (a.lo < b.lo) return -1;
    if (a.lo > b.lo) return +1;
    return 0;
}

/**
 * dpAdd(a, b) -- vax_fpa.c:1080.  a += b.
 *
 * @param {{hi:number,lo:number}} a
 * @param {{hi:number,lo:number}} b
 */
function dpAdd(a, b)
{
    a.lo = (a.lo + b.lo) >>> 0;
    if (a.lo < b.lo) a.hi = (a.hi + 1) >>> 0;   // carry
    a.hi = (a.hi + b.hi) >>> 0;
}

/**
 * dpInc(a) -- vax_fpa.c:1089.  a += 1.
 *
 * @param {{hi:number,lo:number}} a
 */
function dpInc(a)
{
    a.lo = (a.lo + 1) >>> 0;
    if (a.lo == 0) a.hi = (a.hi + 1) >>> 0;
}

/**
 * dpSub(a, b) -- vax_fpa.c:1097.  a -= b.  The borrow is detected BEFORE the low subtraction,
 * which is why the two statements are in this order and must stay in it.
 *
 * @param {{hi:number,lo:number}} a
 * @param {{hi:number,lo:number}} b
 */
function dpSub(a, b)
{
    if (a.lo < b.lo) a.hi = (a.hi - 1) >>> 0;   // borrow
    a.lo = (a.lo - b.lo) >>> 0;
    a.hi = (a.hi - b.hi) >>> 0;
}

/**
 * dpLsh(r, sc) -- vax_fpa.c:1106.
 *
 * The three-way split is not an optimization: C leaves a shift by >= the operand width
 * undefined, and JavaScript is worse than undefined -- it masks the shift count to 5 bits, so
 * `x << 32` is `x`, not 0.  Every branch below shifts by 1..31 only.
 *
 * @param {{hi:number,lo:number}} r
 * @param {number} sc   non-negative
 */
function dpLsh(r, sc)
{
    if (sc > 63) {
        r.hi = r.lo = 0;
    } else if (sc > 31) {
        r.hi = (r.lo << (sc - 32)) >>> 0;
        r.lo = 0;
    } else if (sc != 0) {
        r.hi = ((r.hi << sc) | (r.lo >>> (32 - sc))) >>> 0;
        r.lo = (r.lo << sc) >>> 0;
    }
}

/**
 * dpRsh(r, sc) -- vax_fpa.c:1121.  Unsigned.  See dpLsh() on the shift-count split.
 *
 * @param {{hi:number,lo:number}} r
 * @param {number} sc   non-negative
 */
function dpRsh(r, sc)
{
    if (sc > 63) {
        r.hi = r.lo = 0;
    } else if (sc > 31) {
        r.lo = (r.hi >>> (sc - 32)) >>> 0;
        r.hi = 0;
    } else if (sc != 0) {
        r.lo = ((r.lo >>> sc) | (r.hi << (32 - sc))) >>> 0;
        r.hi = (r.hi >>> sc) >>> 0;
    }
}

/**
 * dpRshS(r, sc, neg) -- vax_fpa.c:1136.
 *
 * A right shift that propagates a sign the caller asserts, rather than one carried in bit 63:
 * shift unsigned, then force the vacated top `sc` bits to ones.  `vax_fadd()` relies on the
 * forcing being unconditional, which is what makes it agree with the 64b path's arithmetic shift.
 *
 * @param {{hi:number,lo:number}} r
 * @param {number} sc
 * @param {number|boolean} neg
 */
function dpRshS(r, sc, neg)
{
    dpRsh(r, sc);
    if (neg && sc) {
        if (sc > 63) {
            r.hi = r.lo = LMASK >>> 0;
        } else {
            let ones = {hi: LMASK >>> 0, lo: LMASK >>> 0};
            dpLsh(ones, 64 - sc);
            r.hi = (r.hi | ones.hi) >>> 0;
            r.lo = (r.lo | ones.lo) >>> 0;
        }
    }
}

/**
 * dpImul(a, b, r) -- vax_fpa.c:1152.  32b * 32b -> 64b, via 16-bit chunks.
 *
 * The chunk products are at most (2^16-1)^2 < 2^32, so each is exact; the accumulations are
 * allowed to exceed 2^32 as JavaScript doubles and are wrapped by the trailing `>>> 0`, which is
 * ToUint32 and therefore a true modulo -- the same thing C's uint32 arithmetic does.
 *
 * @param {number} a    uint32
 * @param {number} b    uint32
 * @param {{hi:number,lo:number}} r
 */
function dpImul(a, b, r)
{
    if ((a == 0) || (b == 0)) {
        r.hi = r.lo = 0;
        return;
    }
    let ah = (a >>> 16) & WMASK, bh = (b >>> 16) & WMASK;
    let al = a & WMASK, bl = b & WMASK;
    let rhi = ah * bh;
    let rmid1 = ah * bl;
    let rmid2 = al * bh;
    let rlo = al * bl;
    rhi = rhi + ((rmid1 >>> 16) & WMASK) + ((rmid2 >>> 16) & WMASK);
    rmid1 = (rlo + ((rmid1 << 16) >>> 0)) >>> 0;    /* add mid1 to lo */
    if (rmid1 < rlo) rhi = rhi + 1;                 /* carry? incr hi */
    rmid2 = (rmid1 + ((rmid2 << 16) >>> 0)) >>> 0;  /* add mid2 to lo */
    if (rmid2 < rmid1) rhi = rhi + 1;               /* carry? incr hi */
    r.hi = rhi >>> 0;
    r.lo = rmid2;
}

/**
 * dpNeg(r) -- vax_fpa.c:1180.  Two's complement negate.
 *
 * @param {{hi:number,lo:number}} r
 */
function dpNeg(r)
{
    r.lo = (~r.lo + 1) >>> 0;                       // NEG(x), vax_defs.h:678
    r.hi = ((~r.hi >>> 0) + (r.lo == 0? 1 : 0)) >>> 0;
}

/**
 * The machine interface a VAXFloat needs.  Everything on it already exists: `decoder` is
 * decode.js's VAXDecoder (this module reads only its published output contract), `mmu` is
 * mmu.js's MMUVAX, and the rest are plain CPU state that the CPU keeps in sync exactly as SIMH's
 * sim_instr() keeps its locals.
 *
 * @typedef {Object} VAXFloatMachine
 * @property {Int32Array} regs   R0-R15
 * @property {number} psl        PSL with the condition codes SPLIT OUT, as inside sim_instr()
 * @property {number} cc         condition codes N/Z/V/C
 * @property {number} trpirq     pending trap/interrupt request, SIMH's `trpirq`
 * @property {number} acc        access mask for the current mode, in RA form (`ACC_MASK(cur)`)
 * @property {Object} decoder    supplies opnd[], spec, rn, va
 * @property {Object} mmu        supplies readData/writeData/test
 */

/**
 * @class VAXFloat
 * @property {VAXFloatMachine} cpu
 * @property {number} rh  the SECOND longword of the last quadword result -- SIMH's `int32 *rh`
 *                        out-parameter, which several routines write and whose PRESENCE (not
 *                        value) also selects D rounding over F rounding in rpackfd()
 * @property {number} flg SIMH's `int32 *flg` out-parameter: CC_V when a float-to-integer
 *                        conversion overflowed, else 0
 */
class VAXFloat {
    /**
     * @param {VAXFloatMachine} cpu
     */
    constructor(cpu)
    {
        this.cpu = cpu;
        this.rh = 0;
        this.flg = 0;
        /*
         * Scratch UFPs and UDPs, allocated once.  vax_fmul() in particular needs four temporary
         * 64-bit products per call; allocating them per call would put four objects into the
         * nursery for every MULD/MULG the machine executes.
         */
        this.ua = new UFP();
        this.ub = new UFP();
        this.mrhi = {hi: 0, lo: 0};
        this.mrlo = {hi: 0, lo: 0};
        this.mrm1 = {hi: 0, lo: 0};
        this.mrm2 = {hi: 0, lo: 0};
        this.dquo = {hi: 0, lo: 0};
        this.mstat = {code: 0};
    }

    /**
     * fault(code, p1)
     *
     * SIMH's ABORT().  A throw, not a longjmp -- see decode.js's header for what the CPU's catch
     * handler owes the decoder afterwards; it owes this module nothing, because nothing here
     * mutates architectural state before it faults.  That is a property of the port worth
     * stating: every routine below computes into scratch and stores only at the very end, so a
     * mid-instruction fault leaves the machine restartable without an unwind.
     *
     * @param {number} code
     * @param {number} [p1]
     * @throws {VAXFault}
     */
    fault(code, p1 = 0)
    {
        throw new VAXFault(code, p1);
    }

    /* --------------------------------------------------------------------------------------- *
     * Support routines -- vax_fpa.c:1187-1313
     * --------------------------------------------------------------------------------------- */

    /**
     * unpackf(hi, r) -- vax_fpa.c:1189
     *
     * @param {number} hi   the F_floating longword, signed int32 as it came from memory
     * @param {UFP} r
     */
    unpackf(hi, r)
    {
        r.sign = hi & FPSIGN;
        r.exp = (hi >>> FD_V_EXP) & FD_M_EXP;               // FD_GETEXP
        if (r.exp == 0) {                                   // exp = 0?
            if (r.sign) this.fault(VAXFAULT.RESOP);         // if -, rsvd op
            r.hi = r.lo = 0;                                // else 0
            return;
        }
        r.hi = WORDSWAP((hi & ~(FPSIGN | FD_EXP)) | FD_HB);
        r.lo = 0;
        dpLsh(r, FD_GUARD);
    }

    /**
     * unpackd(hi, lo, r) -- vax_fpa.c:1205
     *
     * @param {number} hi
     * @param {number} lo
     * @param {UFP} r
     */
    unpackd(hi, lo, r)
    {
        r.sign = hi & FPSIGN;
        r.exp = (hi >>> FD_V_EXP) & FD_M_EXP;
        if (r.exp == 0) {
            if (r.sign) this.fault(VAXFAULT.RESOP);
            r.hi = r.lo = 0;
            return;
        }
        r.hi = WORDSWAP((hi & ~(FPSIGN | FD_EXP)) | FD_HB);
        r.lo = WORDSWAP(lo);
        dpLsh(r, FD_GUARD);
    }

    /**
     * unpackg(hi, lo, r) -- vax_fpa.c:1221
     *
     * @param {number} hi
     * @param {number} lo
     * @param {UFP} r
     */
    unpackg(hi, lo, r)
    {
        r.sign = hi & FPSIGN;
        r.exp = (hi >>> G_V_EXP) & G_M_EXP;                 // G_GETEXP
        if (r.exp == 0) {
            if (r.sign) this.fault(VAXFAULT.RESOP);
            r.hi = r.lo = 0;
            return;
        }
        r.hi = WORDSWAP((hi & ~(FPSIGN | G_EXP)) | G_HB);
        r.lo = WORDSWAP(lo);
        dpLsh(r, G_GUARD);
    }

    /**
     * norm(r) -- vax_fpa.c:1237
     *
     * Left-shift until bit 63 is set, decrementing the exponent.  The mask/table pair is a binary
     * search for the first 1 bit; it is kept because it is what SIMH does, and because the
     * obvious `Math.clz32` rewrite would have to special-case `hi == 0` and would move nothing
     * else -- it is not worth the divergence.
     *
     * @param {UFP} r
     */
    norm(r)
    {
        const normmask = VAXFloat.NORMMASK;
        const normtab = VAXFloat.NORMTAB;
        if ((r.hi == 0) && (r.lo == 0)) {                   // if fraction = 0
            r.sign = r.exp = 0;                             // result is 0
            return;
        }
        while ((r.hi & UF_NM_H) == 0) {                     // normalized?
            let i;
            for (i = 0; i < 5; i++) {                       // find first 1
                if (r.hi & normmask[i]) break;
            }
            dpLsh(r, normtab[i]);                           // shift frac
            r.exp = r.exp - normtab[i];                     // decr exp
        }
    }

    /**
     * rpackfd(r, fQuad) -- vax_fpa.c:1260
     *
     * Round, renormalize, check for overflow and underflow, and pack.  `fQuad` is SIMH's `rh`
     * out-parameter reduced to the only two things its NULL-ness controls: whether a second
     * longword is produced, and -- the part that is easy to miss -- WHICH ROUNDING CONSTANT is
     * added.  F rounds at bit 39 of the guard-extended fraction (UF_FRND), D rounds at bit 7
     * (UF_DRND).  Passing the wrong one is a one-ulp error that a uniform random differential
     * would almost never expose, which is why tests/fpadiff.js's `fRoundAsD` mutation exists.
     *
     * The low longword is left in `this.rh`.
     *
     * @param {UFP} r
     * @param {boolean} fQuad
     * @returns {number} int32, the high (or only) longword
     */
    rpackfd(r, fQuad)
    {
        this.rh = 0;                                        // assume 0
        if ((r.hi == 0) && (r.lo == 0)) return 0;           // result 0?
        if (fQuad) dpAdd(r, VAXFloat.D_ROUND);              // round
        else dpAdd(r, VAXFloat.F_ROUND);
        if ((r.hi & UF_NM_H) == 0) {                        // carry out?
            dpRsh(r, 1);                                    // renormalize
            r.exp = r.exp + 1;
        }
        if (r.exp > FD_M_EXP) {                             // ovflo? fault
            this.fault(ABORT_ARITH, FLT_OVRFLO);
        }
        if (r.exp <= 0) {                                   // underflow?
            if (this.cpu.psl & PSW_FU) {                    // fault if fu
                this.fault(ABORT_ARITH, FLT_UNDFLO);
            }
            return 0;                                       // else 0
        }
        dpRsh(r, FD_GUARD);                                 // remove guard
        if (fQuad) this.rh = WORDSWAP(r.lo) | 0;            // get low
        return (r.sign | (r.exp << FD_V_EXP) |
            (WORDSWAP(r.hi) & ~(FD_HB | FPSIGN | FD_EXP))) | 0;
    }

    /**
     * rpackg(r) -- vax_fpa.c:1290.  Always produces a quadword; low longword in `this.rh`.
     *
     * @param {UFP} r
     * @returns {number} int32
     */
    rpackg(r)
    {
        this.rh = 0;
        if ((r.hi == 0) && (r.lo == 0)) return 0;
        dpAdd(r, VAXFloat.G_ROUND);                         // round
        if ((r.hi & UF_NM_H) == 0) {                        // carry out?
            dpRsh(r, 1);                                    // renormalize
            r.exp = r.exp + 1;
        }
        if (r.exp > G_M_EXP) {                              // ovflo? fault
            this.fault(ABORT_ARITH, FLT_OVRFLO);
        }
        if (r.exp <= 0) {                                   // underflow?
            if (this.cpu.psl & PSW_FU) {
                this.fault(ABORT_ARITH, FLT_UNDFLO);
            }
            return 0;
        }
        dpRsh(r, G_GUARD);                                  // remove guard
        this.rh = WORDSWAP(r.lo) | 0;                       // get low
        return (r.sign | (r.exp << G_V_EXP) |
            (WORDSWAP(r.hi) & ~(G_HB | FPSIGN | G_EXP))) | 0;
    }

    /* --------------------------------------------------------------------------------------- *
     * Unpacked floating point routines -- vax_fpa.c:905-1063
     * --------------------------------------------------------------------------------------- */

    /**
     * vaxFadd(a, b, mhi, mlo) -- vax_fpa.c:905.  a = a + b, signs included.
     *
     * Note the zero test is on the FRACTION, not on the exponent (vax_fpa.c's 2006 fix): POLYx
     * hands this routine deliberately denormalized values whose exponent is non-zero.  We do not
     * implement POLYx, but the test stays as SIMH writes it.
     *
     * @param {UFP} a
     * @param {UFP} b
     * @param {number} mhi  mask applied before normalizing (POLY/EMOD only; 0 for every caller here)
     * @param {number} mlo
     */
    vaxFadd(a, b, mhi, mlo)
    {
        if ((a.hi == 0) && (a.lo == 0)) {                   // s1 = 0?
            a.copyFrom(b);
            return;
        }
        if ((b.hi == 0) && (b.lo == 0)) return;             // s2 = 0?
        if ((a.exp < b.exp) ||                              // |s1| < |s2|? swap
            ((a.exp == b.exp) && (dpCmp(a, b) < 0))) {
            a.swapWith(b);
        }
        let ediff = a.exp - b.exp;                          // exp diff
        if (a.sign ^ b.sign) {                              // eff sub?
            if (ediff) {                                    // exp diff?
                dpNeg(b);                                   // negate fraction
                dpRshS(b, ediff, 1);                        // signed right
                dpAdd(a, b);                                // "add" frac
            } else {
                dpSub(a, b);                                // a >= b
            }
            a.hi = (a.hi & ~mhi) >>> 0;                     // mask before norm
            a.lo = (a.lo & ~mlo) >>> 0;
            this.norm(a);                                   // normalize
        } else {
            if (ediff) dpRsh(b, ediff);                     // add, denormalize
            dpAdd(a, b);                                    // add frac
            if (dpCmp(a, b) < 0) {                          // chk for carry
                dpRsh(a, 1);                                // renormalize
                a.hi = (a.hi | UF_NM_H) >>> 0;              // add norm bit
                a.exp = a.exp + 1;                          // skip norm
            }
            a.hi = (a.hi & ~mhi) >>> 0;                     // mask
            a.lo = (a.lo & ~mlo) >>> 0;
        }
    }

    /**
     * vaxFmul(a, b, qd, bias, mhi, mlo) -- vax_fpa.c:951.  a = a * b.
     *
     * `qd` selects the full 64x64 cross-product form (D and G) over the high-32-only form (F).
     *
     * @param {UFP} a
     * @param {UFP} b
     * @param {boolean} qd
     * @param {number} bias
     * @param {number} mhi
     * @param {number} mlo
     */
    vaxFmul(a, b, qd, bias, mhi, mlo)
    {
        let rhi = this.mrhi, rlo = this.mrlo, rmid1 = this.mrm1, rmid2 = this.mrm2;
        if ((a.exp == 0) || (b.exp == 0)) {                 // zero argument?
            a.hi = a.lo = 0;                                // result is zero
            a.sign = a.exp = 0;
            return;
        }
        a.sign = a.sign ^ b.sign;                           // sign of result
        a.exp = a.exp + b.exp - bias;                       // add exponents
        dpImul(a.hi, b.hi, rhi);                            // high result
        if (qd) {                                           // 64b needed?
            dpImul(a.hi, b.lo, rmid1);                      // cross products
            dpImul(a.lo, b.hi, rmid2);
            dpImul(a.lo, b.lo, rlo);                        // low result
            rhi.lo = (rhi.lo + rmid1.hi) >>> 0;             // add hi cross
            if (rhi.lo < rmid1.hi) rhi.hi = (rhi.hi + 1) >>> 0;
            rhi.lo = (rhi.lo + rmid2.hi) >>> 0;
            if (rhi.lo < rmid2.hi) rhi.hi = (rhi.hi + 1) >>> 0;
            rlo.hi = (rlo.hi + rmid1.lo) >>> 0;             // add mid1 to low res
            if (rlo.hi < rmid1.lo) dpInc(rhi);              // carry? incr high res
            rlo.hi = (rlo.hi + rmid2.lo) >>> 0;             // add mid2 to low res
            if (rlo.hi < rmid2.lo) dpInc(rhi);
        }
        a.hi = (rhi.hi & ~mhi) >>> 0;                       // mask fraction
        a.lo = (rhi.lo & ~mlo) >>> 0;
        this.norm(a);                                       // normalize
    }

    /**
     * vaxFdiv(a, b, prec, bias) -- vax_fpa.c:1038.  b = b / a.  The RESULT is in `b`.
     *
     * Restoring division, one quotient bit per iteration.  `prec` must be two more than the
     * format's fraction width because the first divide step can fail and at least one rounding
     * bit is needed: 26 for F, 58 for D, 55 for G.  Those three numbers are load-bearing.
     *
     * @param {UFP} a   divisor
     * @param {UFP} b   dividend, and the result
     * @param {number} prec
     * @param {number} bias
     */
    vaxFdiv(a, b, prec, bias)
    {
        let quo = this.dquo;
        quo.hi = quo.lo = 0;
        if (a.exp == 0) {                                   // divr = 0?
            this.fault(ABORT_ARITH, FLT_DIVZRO);
        }
        if (b.exp == 0) return;                             // divd = 0?
        b.sign = b.sign ^ a.sign;                           // result sign
        b.exp = b.exp - a.exp + bias + 1;                   // unbiased exp
        dpRsh(a, 1);                                        // allow 1 bit left
        dpRsh(b, 1);
        for (let i = 0; i < prec; i++) {                    // divide loop
            dpLsh(quo, 1);                                  // shift quo
            if (dpCmp(b, a) >= 0) {                         // div step ok?
                dpSub(b, a);                                // subtract
                quo.lo = (quo.lo + 1) >>> 0;                // quo bit = 1
            }
            dpLsh(b, 1);                                    // shift divd
        }
        dpLsh(quo, UF_V_NM - prec + 1);                     // put in position
        b.hi = quo.hi;
        b.lo = quo.lo;
        this.norm(b);                                       // normalize
    }

    /* --------------------------------------------------------------------------------------- *
     * Instruction-level routines -- vax_fpa.c:752-854, 1325-1489
     * --------------------------------------------------------------------------------------- */

    /**
     * opCmpfd(h1, l1, h2, l2) -- vax_fpa.c:754
     *
     * Note this compares UNPACKED values, so a reserved operand on either side faults before any
     * comparison, and a true zero compares equal to a true zero of either sign (there is only one
     * zero: sign 1 with exponent 0 is the reserved operand, which is why the sign test below is
     * reached only for non-zero values).  CMPF passes 0 for both low longwords.
     *
     * @param {number} h1
     * @param {number} l1
     * @param {number} h2
     * @param {number} l2
     * @returns {number} condition codes
     */
    opCmpfd(h1, l1, h2, l2)
    {
        let a = this.ua, b = this.ub;
        this.unpackd(h1, l1, a);
        this.unpackd(h2, l2, b);
        if (a.sign != b.sign) return (a.sign? CC_N : 0);
        let r = a.exp - b.exp;
        if (r == 0) r = dpCmp(a, b);
        if (r < 0) return (a.sign? 0 : CC_N);
        if (r > 0) return (a.sign? CC_N : 0);
        return CC_Z;
    }

    /**
     * opCmpg(h1, l1, h2, l2) -- vax_fpa.c:773
     *
     * @param {number} h1
     * @param {number} l1
     * @param {number} h2
     * @param {number} l2
     * @returns {number} condition codes
     */
    opCmpg(h1, l1, h2, l2)
    {
        let a = this.ua, b = this.ub;
        this.unpackg(h1, l1, a);
        this.unpackg(h2, l2, b);
        if (a.sign != b.sign) return (a.sign? CC_N : 0);
        let r = a.exp - b.exp;
        if (r == 0) r = dpCmp(a, b);
        if (r < 0) return (a.sign? 0 : CC_N);
        if (r > 0) return (a.sign? CC_N : 0);
        return CC_Z;
    }

    /**
     * opCvtifdg(val, fQuad, opc) -- vax_fpa.c:794.  Integer to floating.
     *
     * @param {number} val  sign-extended source, int32
     * @param {boolean} fQuad
     * @param {number} opc
     * @returns {number} int32; low longword in `this.rh`
     */
    opCvtifdg(val, fQuad, opc)
    {
        let a = this.ua;
        if (val == 0) {                                     // zero?
            if (fQuad) this.rh = 0;                         // return true 0
            return 0;
        }
        if (val < 0) {                                      // negative?
            a.sign = FPSIGN;                                // sign = -
            val = -val;
        } else {
            a.sign = 0;                                     // else sign = +
        }
        a.exp = 32 + ((opc & 0x100)? G_BIAS : FD_BIAS);     // initial exp
        /*
         * `val & LMASK` in C on an int32 that has just been negated.  0x80000000 negates to
         * itself, which is exactly the case that must survive: CVTLD of -2147483648.
         */
        a.hi = val >>> 0;                                   // fraction
        a.lo = 0;
        this.norm(a);                                       // normalize
        if (opc & 0x100) return this.rpackg(a);             // pack and return
        return this.rpackfd(a, fQuad);
    }

    /**
     * opCvtfdgi(opnd, opc) -- vax_fpa.c:818.  Floating to integer.
     *
     * Sets `this.flg` to CC_V on overflow.  The rounding form (CVTRxL, `opc & 3` == 3) adds one
     * at the round bit BEFORE the final justifying shift; the truncating form does not.  Note
     * that the overflow limit is asymmetric -- `maxv[lnt] + 1` for a negative value -- so
     * CVTFL of exactly -2147483648.0 does NOT overflow while +2147483648.0 does.
     *
     * @param {Int32Array|Array} opnd
     * @param {number} opc
     * @returns {number} int32
     */
    opCvtfdgi(opnd, opc)
    {
        let a = this.ua;
        let lnt = opc & 3;
        let ubexp;
        this.flg = 0;
        if (opc & 0x100) {                                  // G?
            this.unpackg(opnd[0], opnd[1], a);              // unpack
            ubexp = a.exp - G_BIAS;                         // unbiased exp
        } else {
            if (opc & 0x20) this.unpackd(opnd[0], opnd[1], a);  // F or D
            else this.unpackf(opnd[0], a);
            ubexp = a.exp - FD_BIAS;
        }
        if ((a.exp == 0) || (ubexp < 0)) return 0;          // true zero or frac?
        if (ubexp <= UF_V_NM) {                             // exp in range?
            dpRsh(a, UF_V_NM - ubexp);                      // leave rnd bit
            if (lnt == 3) dpInc(a);                         // if CVTR, round
            dpRsh(a, 1);                                    // now justified
            if ((a.hi != 0) ||
                (a.lo > (VAXFloat.MAXV[lnt] + (a.sign? 1 : 0)))) {
                this.flg = CC_V;
            }
        } else {
            this.flg = CC_V;                                // always ovflo
            if (ubexp > (UF_V_NM + 32)) return 0;           // in ext range?
            dpLsh(a, ubexp - UF_V_NM - 1);                  // no rnd bit
        }
        return (a.sign? ((~a.lo + 1) >>> 0) : a.lo) | 0;    // return lo frac
    }

    /**
     * opMovfd(val) -- vax_fpa.c:1325.  Only the high 32 bits are processed; a non-zero exponent
     * passes the value through unchanged, which is what makes MOVD/MOVG a pure copy of the low
     * longword too.
     *
     * @param {number} val
     * @returns {number} int32
     */
    opMovfd(val)
    {
        if (val & FD_EXP) return val;
        if (val & FPSIGN) this.fault(VAXFAULT.RESOP);
        return 0;
    }

    /**
     * opMnegfd(val) -- vax_fpa.c:1334
     *
     * @param {number} val
     * @returns {number} int32
     */
    opMnegfd(val)
    {
        if (val & FD_EXP) return (val ^ FPSIGN);
        if (val & FPSIGN) this.fault(VAXFAULT.RESOP);
        return 0;
    }

    /**
     * opMovg(val) -- vax_fpa.c:1343
     *
     * @param {number} val
     * @returns {number} int32
     */
    opMovg(val)
    {
        if (val & G_EXP) return val;
        if (val & FPSIGN) this.fault(VAXFAULT.RESOP);
        return 0;
    }

    /**
     * opMnegg(val) -- vax_fpa.c:1352
     *
     * @param {number} val
     * @returns {number} int32
     */
    opMnegg(val)
    {
        if (val & G_EXP) return (val ^ FPSIGN);
        if (val & FPSIGN) this.fault(VAXFAULT.RESOP);
        return 0;
    }

    /**
     * opCvtdf(opnd) -- vax_fpa.c:1363.  D to F.  There is no CVTFD routine: F to D is MOVFD,
     * because an F value is a D value whose low longword is zero.
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opCvtdf(opnd)
    {
        let a = this.ua;
        this.unpackd(opnd[0], opnd[1], a);
        return this.rpackfd(a, false);
    }

    /**
     * opCvtfg(opnd) -- vax_fpa.c:1371.  F to G; low longword in `this.rh`.
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opCvtfg(opnd)
    {
        let a = this.ua;
        this.unpackf(opnd[0], a);
        a.exp = a.exp - FD_BIAS + G_BIAS;
        return this.rpackg(a);
    }

    /**
     * opCvtgf(opnd) -- vax_fpa.c:1380.  G to F.
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opCvtgf(opnd)
    {
        let a = this.ua;
        this.unpackg(opnd[0], opnd[1], a);
        a.exp = a.exp - G_BIAS + FD_BIAS;
        return this.rpackfd(a, false);
    }

    /**
     * opAddf(opnd, sub) -- vax_fpa.c:1391.  SUBx negates the FIRST operand, not the second,
     * because the VAX computes `src2 - src1`.
     *
     * @param {Int32Array|Array} opnd
     * @param {boolean} sub
     * @returns {number} int32
     */
    opAddf(opnd, sub)
    {
        let a = this.ua, b = this.ub;
        this.unpackf(opnd[0], a);                           // F format
        this.unpackf(opnd[1], b);
        if (sub) a.sign = a.sign ^ FPSIGN;                  // sub? -s1
        this.vaxFadd(a, b, 0, 0);                           // add fractions
        return this.rpackfd(a, false);
    }

    /**
     * opAddd(opnd, sub) -- vax_fpa.c:1403
     *
     * @param {Int32Array|Array} opnd
     * @param {boolean} sub
     * @returns {number} int32
     */
    opAddd(opnd, sub)
    {
        let a = this.ua, b = this.ub;
        this.unpackd(opnd[0], opnd[1], a);
        this.unpackd(opnd[2], opnd[3], b);
        if (sub) a.sign = a.sign ^ FPSIGN;
        this.vaxFadd(a, b, 0, 0);
        return this.rpackfd(a, true);
    }

    /**
     * opAddg(opnd, sub) -- vax_fpa.c:1415
     *
     * @param {Int32Array|Array} opnd
     * @param {boolean} sub
     * @returns {number} int32
     */
    opAddg(opnd, sub)
    {
        let a = this.ua, b = this.ub;
        this.unpackg(opnd[0], opnd[1], a);
        this.unpackg(opnd[2], opnd[3], b);
        if (sub) a.sign = a.sign ^ FPSIGN;
        this.vaxFadd(a, b, 0, 0);
        return this.rpackg(a);                              // round and pack
    }

    /**
     * opMulf(opnd) -- vax_fpa.c:1429
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opMulf(opnd)
    {
        let a = this.ua, b = this.ub;
        this.unpackf(opnd[0], a);                           // F format
        this.unpackf(opnd[1], b);
        this.vaxFmul(a, b, false, FD_BIAS, 0, 0);           // do multiply
        return this.rpackfd(a, false);                      // round and pack
    }

    /**
     * opMuld(opnd) -- vax_fpa.c:1439
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opMuld(opnd)
    {
        let a = this.ua, b = this.ub;
        this.unpackd(opnd[0], opnd[1], a);                  // D format
        this.unpackd(opnd[2], opnd[3], b);
        this.vaxFmul(a, b, true, FD_BIAS, 0, 0);
        return this.rpackfd(a, true);
    }

    /**
     * opMulg(opnd) -- vax_fpa.c:1449
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opMulg(opnd)
    {
        let a = this.ua, b = this.ub;
        this.unpackg(opnd[0], opnd[1], a);                  // G format
        this.unpackg(opnd[2], opnd[3], b);
        this.vaxFmul(a, b, true, G_BIAS, 0, 0);
        return this.rpackg(a);
    }

    /**
     * opDivf(opnd) -- vax_fpa.c:1461.  Note vaxFdiv() leaves the quotient in `b`, so it is `b`
     * that is packed -- copying SIMH exactly, including which operand is which.
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opDivf(opnd)
    {
        let a = this.ua, b = this.ub;
        this.unpackf(opnd[0], a);                           // F format
        this.unpackf(opnd[1], b);
        this.vaxFdiv(a, b, 26, FD_BIAS);                    // do divide
        return this.rpackfd(b, false);                      // round and pack
    }

    /**
     * opDivd(opnd) -- vax_fpa.c:1471
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opDivd(opnd)
    {
        let a = this.ua, b = this.ub;
        this.unpackd(opnd[0], opnd[1], a);                  // D format
        this.unpackd(opnd[2], opnd[3], b);
        this.vaxFdiv(a, b, 58, FD_BIAS);
        return this.rpackfd(b, true);
    }

    /**
     * opDivg(opnd) -- vax_fpa.c:1481
     *
     * @param {Int32Array|Array} opnd
     * @returns {number} int32
     */
    opDivg(opnd)
    {
        let a = this.ua, b = this.ub;
        this.unpackg(opnd[0], opnd[1], a);                  // G format
        this.unpackg(opnd[2], opnd[3], b);
        this.vaxFdiv(a, b, 55, G_BIAS);
        return this.rpackg(b);
    }

    /* --------------------------------------------------------------------------------------- *
     * Destination stores -- vax_cpu.c:212-233, the WRITE_B/W/L/Q macros
     *
     * pcjsvax-8c0 shipped isMemoryDestination() (the `spec > (GRN | nPC)` test) but deliberately
     * not the stores, because storing is execution.  These are them, and they are here rather
     * than in decode.js for the same reason.
     * --------------------------------------------------------------------------------------- */

    /**
     * writeB(r) -- WRITE_B, vax_cpu.c:212
     *
     * @param {number} r
     */
    writeB(r)
    {
        let cpu = this.cpu, d = cpu.decoder;
        if (d.spec > (GRN | nPC)) cpu.mmu.writeData(d.va, r, 1, cpu.acc << 4);
        else cpu.regs[d.rn] = (cpu.regs[d.rn] & ~BMASK) | (r & BMASK);
    }

    /**
     * writeW(r) -- WRITE_W, vax_cpu.c:215
     *
     * @param {number} r
     */
    writeW(r)
    {
        let cpu = this.cpu, d = cpu.decoder;
        if (d.spec > (GRN | nPC)) cpu.mmu.writeData(d.va, r, 2, cpu.acc << 4);
        else cpu.regs[d.rn] = (cpu.regs[d.rn] & ~WMASK) | (r & WMASK);
    }

    /**
     * writeL(r) -- WRITE_L, vax_cpu.c:218
     *
     * @param {number} r
     */
    writeL(r)
    {
        let cpu = this.cpu, d = cpu.decoder;
        if (d.spec > (GRN | nPC)) cpu.mmu.writeData(d.va, r, 4, cpu.acc << 4);
        else cpu.regs[d.rn] = r;
    }

    /**
     * writeQ(rl, rh) -- WRITE_Q, vax_cpu.c:221
     *
     * READ THE PROBE CAREFULLY.  The C macro's indentation is misleading: the `if` guards ONLY
     * the low-longword Write, and the high-longword Write is unconditional.  That is not a bug,
     * it is the whole trick, and every case matters:
     *
     *   both pages OK        Test(va+7) >= 0 is true  -> write low, then write high.
     *   second page bad      both tests false         -> SKIP the low write, then the high write
     *                                                    faults.  Nothing is stored, so the
     *                                                    instruction is still restartable.
     *   first page bad       Test(va) < 0 is true     -> the low write runs and faults, so the
     *                                                    fault reported is the FIRST page's --
     *                                                    which is what the architecture requires
     *                                                    when both pages are inaccessible.
     *
     * Reproduce it literally.  A "cleaner" version that probes both and then writes both changes
     * which fault is reported when both pages are bad.
     *
     * Register destinations fault on R14/R15 (RSVD_ADDR_FAULT) because a quadword needs a pair.
     *
     * @param {number} rl
     * @param {number} rh
     */
    writeQ(rl, rh)
    {
        let cpu = this.cpu, d = cpu.decoder;
        if (d.spec > (GRN | nPC)) {
            let wa = cpu.acc << 4;
            let mstat = this.mstat;
            if ((cpu.mmu.test((d.va + 7) | 0, wa, mstat) >= 0) ||
                (cpu.mmu.test(d.va, wa, mstat) < 0)) {
                cpu.mmu.writeData(d.va, rl, 4, wa);
            }
            cpu.mmu.writeData((d.va + 4) | 0, rh, 4, wa);
        } else {
            if (d.rn >= nSP) this.fault(VAXFAULT.RESAD);
            cpu.regs[d.rn] = rl;
            cpu.regs[d.rn + 1] = rh;
        }
    }

    /* --------------------------------------------------------------------------------------- *
     * Instruction dispatch -- vax_cpu.c:2762-3000
     *
     * One case per in-scope opcode, transcribed from sim_instr()'s switch so the two can be read
     * side by side.  The condition-code macros are expanded inline (JavaScript has no macros)
     * but the CHOICE of macro per opcode is SIMH's and must not be "tidied": CC_IIZZ_FP clears
     * C, CC_IIZP_FP preserves it, and which instructions do which is architectural.
     * --------------------------------------------------------------------------------------- */

    /**
     * ccIIZZfp(r) -- CC_IIZZ_FP, i.e. CC_IIZZ_W (vax_defs.h:735, 745).
     *
     * The negative test is on FPSIGN (bit 15), not on bit 31 -- the VAX floating sign lives in
     * the low word.  The zero test is on the WHOLE longword, exactly as the macro writes it.
     *
     * @param {number} r
     */
    ccIIZZfp(r)
    {
        if (r & WSIGN) this.cpu.cc = CC_N;
        else if (r == 0) this.cpu.cc = CC_Z;
        else this.cpu.cc = 0;
    }

    /**
     * ccIIZPfp(r) -- CC_IIZP_FP, i.e. CC_IIZP_W (vax_defs.h:757, 767).  Carry preserved.
     *
     * @param {number} r
     */
    ccIIZPfp(r)
    {
        let cc = this.cpu.cc;
        if (r & WSIGN) this.cpu.cc = CC_N | (cc & CC_C);
        else if (r == 0) this.cpu.cc = CC_Z | (cc & CC_C);
        else this.cpu.cc = cc & CC_C;
    }

    /**
     * ccIIZZint(r, sign) -- CC_IIZZ_B / _W / _L (vax_defs.h:729-739), for the float-to-integer
     * conversions, whose result is an integer and whose sign bit is therefore the integer's.
     *
     * @param {number} r
     * @param {number} sign  BSIGN, WSIGN or LSIGN
     */
    ccIIZZint(r, sign)
    {
        if (r & sign) this.cpu.cc = CC_N;
        else if (r == 0) this.cpu.cc = CC_Z;
        else this.cpu.cc = 0;
    }

    /**
     * setIntOverflow() -- V_INTOV, vax_defs.h:676-677.
     *
     * Sets V unconditionally and REQUESTS a trap only when PSL<IV> is set.  It is a trap, not a
     * fault: the instruction completes and its result is stored.  Contrast the floating
     * overflow, underflow and divide-by-zero cases, which are faults raised from inside
     * rpackfd(), rpackg() and vaxFdiv() before anything is stored.
     */
    setIntOverflow()
    {
        let cpu = this.cpu;
        cpu.cc = cpu.cc | CC_V;
        if (cpu.psl & PSW_IV) cpu.trpirq = (cpu.trpirq & PSL_M_IPL) | TRAP_INTOV;
    }

    /**
     * execute(opc)
     *
     * Run one already-decoded floating instruction.  Reads the decoder's operand queue and
     * destination state, stores the result, and sets the condition codes.
     *
     * @param {number} opc
     * @returns {boolean} true if this module handled the opcode
     */
    execute(opc)
    {
        let cpu = this.cpu;
        let opnd = cpu.decoder.opnd;
        let r, rh;

        switch (opc) {

        case OPC.TSTF: case OPC.TSTD:
            r = this.opMovfd(opnd[0]);
            this.ccIIZZfp(r);
            break;

        case OPC.TSTG:
            r = this.opMovg(opnd[0]);
            this.ccIIZZfp(r);
            break;

        case OPC.MOVF:
            r = this.opMovfd(opnd[0]);
            this.writeL(r);
            this.ccIIZPfp(r);
            break;

        case OPC.MOVD:
            rh = opnd[1];
            if ((r = this.opMovfd(opnd[0])) == 0) rh = 0;
            this.writeQ(r, rh);
            this.ccIIZPfp(r);
            break;

        case OPC.MOVG:
            rh = opnd[1];
            if ((r = this.opMovg(opnd[0])) == 0) rh = 0;
            this.writeQ(r, rh);
            this.ccIIZPfp(r);
            break;

        case OPC.MNEGF:
            r = this.opMnegfd(opnd[0]);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.MNEGD:
            rh = opnd[1];
            if ((r = this.opMnegfd(opnd[0])) == 0) rh = 0;
            this.writeQ(r, rh);
            this.ccIIZZfp(r);
            break;

        case OPC.MNEGG:
            rh = opnd[1];
            if ((r = this.opMnegg(opnd[0])) == 0) rh = 0;
            this.writeQ(r, rh);
            this.ccIIZZfp(r);
            break;

        case OPC.CMPF:
            cpu.cc = this.opCmpfd(opnd[0], 0, opnd[1], 0);
            break;

        case OPC.CMPD:
            cpu.cc = this.opCmpfd(opnd[0], opnd[1], opnd[2], opnd[3]);
            break;

        case OPC.CMPG:
            cpu.cc = this.opCmpg(opnd[0], opnd[1], opnd[2], opnd[3]);
            break;

        case OPC.CVTBF:
            r = this.opCvtifdg(SXTB(opnd[0]), false, opc);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTWF:
            r = this.opCvtifdg(SXTW(opnd[0]), false, opc);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTLF:
            r = this.opCvtifdg(opnd[0], false, opc);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTBD: case OPC.CVTBG:
            r = this.opCvtifdg(SXTB(opnd[0]), true, opc);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTWD: case OPC.CVTWG:
            r = this.opCvtifdg(SXTW(opnd[0]), true, opc);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTLD: case OPC.CVTLG:
            r = this.opCvtifdg(opnd[0], true, opc);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTFB: case OPC.CVTDB: case OPC.CVTGB:
            r = this.opCvtfdgi(opnd, opc) & BMASK;
            this.writeB(r);
            this.ccIIZZint(r, 0x80);
            if (this.flg) this.setIntOverflow();
            break;

        case OPC.CVTFW: case OPC.CVTDW: case OPC.CVTGW:
            r = this.opCvtfdgi(opnd, opc) & WMASK;
            this.writeW(r);
            this.ccIIZZint(r, WSIGN);
            if (this.flg) this.setIntOverflow();
            break;

        case OPC.CVTFL: case OPC.CVTDL: case OPC.CVTGL:
        case OPC.CVTRFL: case OPC.CVTRDL: case OPC.CVTRGL:
            r = this.opCvtfdgi(opnd, opc) | 0;              // & LMASK
            this.writeL(r);
            this.ccIIZZint(r, UF_NM_H);                    // LSIGN
            if (this.flg) this.setIntOverflow();
            break;

        case OPC.CVTFD:
            r = this.opMovfd(opnd[0]);
            this.writeQ(r, 0);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTDF:
            r = this.opCvtdf(opnd);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTFG:
            r = this.opCvtfg(opnd);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.CVTGF:
            r = this.opCvtgf(opnd);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.ADDF2: case OPC.ADDF3:
            r = this.opAddf(opnd, false);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.ADDD2: case OPC.ADDD3:
            r = this.opAddd(opnd, false);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.ADDG2: case OPC.ADDG3:
            r = this.opAddg(opnd, false);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.SUBF2: case OPC.SUBF3:
            r = this.opAddf(opnd, true);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.SUBD2: case OPC.SUBD3:
            r = this.opAddd(opnd, true);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.SUBG2: case OPC.SUBG3:
            r = this.opAddg(opnd, true);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.MULF2: case OPC.MULF3:
            r = this.opMulf(opnd);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.MULD2: case OPC.MULD3:
            r = this.opMuld(opnd);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.MULG2: case OPC.MULG3:
            r = this.opMulg(opnd);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.DIVF2: case OPC.DIVF3:
            r = this.opDivf(opnd);
            this.writeL(r);
            this.ccIIZZfp(r);
            break;

        case OPC.DIVD2: case OPC.DIVD3:
            r = this.opDivd(opnd);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        case OPC.DIVG2: case OPC.DIVG3:
            r = this.opDivg(opnd);
            this.writeQ(r, this.rh);
            this.ccIIZZfp(r);
            break;

        default:
            return false;
        }
        return true;
    }
}

/*
 * Tables hung off the class so they are allocated once.  NORMMASK/NORMTAB are vax_fpa.c:1240-1243
 * and MAXV is vax_fpa.c:823.  The rounding constants are UDPs in SIMH ({lo, hi} order there --
 * written {hi, lo} here, so read the pairing carefully rather than by position).
 */
VAXFloat.NORMMASK = [0xc0000000, 0xf0000000, 0xff000000, 0xffff0000, 0xffffffff];
VAXFloat.NORMTAB  = [1, 2, 4, 8, 16, 32];
VAXFloat.MAXV     = [0x7F, 0x7FFF, 0x7FFFFFFF, 0x7FFFFFFF];
VAXFloat.F_ROUND  = {hi: UF_FRND_H, lo: UF_FRND_L};
VAXFloat.D_ROUND  = {hi: UF_DRND_H, lo: UF_DRND_L};
VAXFloat.G_ROUND  = {hi: UF_GRND_H, lo: UF_GRND_L};

export default VAXFloat;
export {
    VAXFloat, UFP, OPC, ABORT_ARITH,
    FPSIGN, FD_V_EXP, FD_M_EXP, FD_BIAS, FD_EXP, FD_HB, FD_GUARD,
    G_V_EXP, G_M_EXP, G_BIAS, G_EXP, G_HB, G_GUARD,
    PSW_FU, PSW_IV, CC_N, CC_Z, CC_V, CC_C,
    FLT_OVRFLO, FLT_DIVZRO, FLT_UNDFLO, TRAP_INTOV,
    WORDSWAP, SXTB, SXTW,
    dpCmp, dpAdd, dpInc, dpSub, dpLsh, dpRsh, dpRshS, dpImul, dpNeg
};
