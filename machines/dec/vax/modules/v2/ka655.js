/**
 * @fileoverview Implements the KA655 CPU-board register pair (CACR cache control, BDR boot/diag)
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
 * = BDR (boot/diagnostic register, READ-ONLY -- ka_wr()'s whole body is gated on `rg==0`, there is
 * no BDR write case at all, matching real hardware: BDR reflects boot-time switch/jumper state).
 *
 * VERACITY FINDING (pcjsvax-bfb re-dispatch) -- "no case" WAS RECORDED CORRECTLY AND CONCLUDED
 * WRONG: an earlier version of this paragraph observed exactly the fact above -- ka_wr()'s body is
 * `if ((rg == 0) && ((pa & 3) == 0)) { ka_cacr = ...; }` WITH NO ELSE -- and then had writeReg()
 * below return `false` for every case that gate excludes, which this project's convention (see
 * ssc.js's readNone/writeNone routing) turns into a BUS FAULT.  That is backwards.  "No case" in a
 * C switch/if with no else/default means the C FUNCTION RETURNS NORMALLY HAVING DONE NOTHING -- the
 * caller's PC advances, no exception is raised.  It does NOT mean "this reference is invalid" --
 * that would require SIMH's OWN machine-check machinery to be invoked explicitly, which ka_wr()
 * never does.  MEASURED against the real oracle: a longword write to BDR (@KABASE+4) leaves SIMH's
 * PC advancing normally with BDR's readback UNCHANGED (0x00000080, BDR_BRKENB); an UNALIGNED write
 * to CACR (@KABASE+2) likewise advances PC normally with CACR's own readback unaffected by the
 * write attempt.  Neither dispatches to the handler a genuine machine check would use.  This is the
 * SAME class of defect ssc.js's readReg()/writeReg() had (see that file's veracity-finding comment)
 * -- "the switch has no case for this" was conflated with "this access must fault" everywhere in
 * this tree's first pass, when only ONE register anywhere in these three files (CQBICVAX's MEAR/
 * SEAR, cqbic.js) is a genuine, deliberate bus error on write.  writeReg() below now returns `true`
 * (silent accept, matching vax_io.c's own no-else/no-default shape) for BOTH the wrong-register and
 * the misaligned-CACR case; graded by tests/romdiff.js's verifyFallthroughSemantics().
 *
 * CDG -- pcjsvax-0b7 IS the "later boundary" an earlier version of this paragraph said would have
 * to extend this file, so it did.  cdg.js now decodes VAX.PHYSMEM.CDG_BASE, and vax_sysdev.c's
 * cdg_rd() (:1226-1240) writes THIS register as a side effect of every CDG READ: it clears CACR_DRO
 * and ORs in four diagnostic-parity bits computed from the longword just read.  setCdgDiagParity()
 * below is exactly that, CDGVAX calls it on every read, and tests/cdgdiff.js's CACR phase grades all
 * four bit positions and both parity seeds against the live oracle.
 *
 * STILL NOT MODELED, because vax_sysdev.c does not model it either: CACR's CEN (cache enable) and
 * DIAG (diagnostic mode) bits GATE NOTHING.  Nothing in cdg_rd()/cdg_wr(), or anywhere else in
 * vax_sysdev.c, reads them back -- writeReg() below stores them and readReg() returns them, and
 * that is the whole of their behaviour on both engines.
 */

import { VAX } from "./defines.js";

const KA_BASE = (VAX.PHYSMEM.REG_BASE + 0x4000) >>> 0;   // vaxmod_defs.h:190, KABASE = REGBASE+0x4000

const REG_CACR = 0;
const REG_BDR  = 1;

/* vax_sysdev.c:127-140. */
const BDR_BRKENB = 0x00000080;
const CACR_FIXED = 0x00000040;
const CACR_CPE   = 0x00000020;
const CACR_W1C   = CACR_CPE;
const CACR_RW    = (0x00000010 | 0x00000004 | 0x00000002 | 0x00000001);   // CEN|DPE|WWP|DIAG
const CACR_DRO   = 0x00FFFF00;      // diag bits, READ-ONLY to software -- only cdg_rd() writes them
const CACR_V_DPAR = 24;             // bit position of diagnostic-parity lane 0

/**
 * parity(val, odd)
 *
 * vax_sysdev.c:1254-1261 -- XOR the seed with every set bit, i.e. `odd ^ (popcount(val) & 1)`.
 * Transcribed as the loop rather than the identity so a reader can check it against the C directly.
 *
 * @param {number} val
 * @param {number} odd the seed, 0 or 1
 * @returns {number} 0 or 1
 */
function parity(val, odd)
{
    for (val = val >>> 0; val != 0; val = val >>> 1) {
        if (val & 1) odd = odd ^ 1;
    }
    return odd;
}

export default class KA655VAX {
    /**
     * KA655VAX()
     */
    constructor()
    {
        this.reset();
    }

    /**
     * reset()
     *
     * vax_sysdev.c:242 (`int32 ka_bdr = BDR_BRKENB;`, a C static initializer -- ka_bdr is never
     * touched by sysd_reset() or sysd_powerup(), so this is its value from the FIRST instruction of
     * any run, not merely "after reset") and :1788 (sysd_powerup(): `ka_cacr = 0`).
     *
     * @this {KA655VAX}
     */
    reset()
    {
        this.cacr = 0;
        this.bdr = BDR_BRKENB | 0;
    }

    /**
     * @this {KA655VAX}
     * @param {number} rg
     * @returns {number} the register's value, or 0 if `rg` is not CACR/BDR (see the file header --
     *   this device's declared span is exactly {CACR, BDR}, so this default is unreachable through
     *   regblock.js's dispatch today, but is 0 rather than null for the same reason ssc.js's/
     *   cqbic.js's readReg() are: ka_rd() itself has no `default:` and falls through to `return 0`.
     */
    readReg(rg)
    {
        switch (rg) {
        case REG_CACR: return this.cacr;
        case REG_BDR:  return this.bdr;
        }
        return 0;
    }

    /**
     * writeReg(rg, val, fAligned)
     *
     * vax_sysdev.c:1211: `if ((rg == 0) && ((pa & 3) == 0)) { ka_cacr = ...; }` -- CACR accepts ONLY
     * an ALIGNED longword write (the byte/word merge every other register in this tree does via a
     * `t = ...rd()` read-modify step is deliberately NOT reproduced here: real hardware's own gate
     * excludes anything but an aligned longword before it ever looks at the data).  BDR is
     * read-only: no `rg == 1` case exists in ka_wr() at all.
     *
     * VERACITY FINDING: BOTH of those are SILENT NO-OPS on real hardware -- a write to BDR, and an
     * unaligned write to CACR, leave SIMH's PC advancing normally with the register's own readback
     * UNCHANGED.  See the file header's veracity-finding paragraph for the measurement and for why
     * an earlier version of this function returned `false` (fault) here instead.  This function
     * therefore ALWAYS returns `true`: there is no register in this device that is a genuine bus
     * error to write (unlike cqbic.js's MEAR/SEAR) -- only "applied" and "silently ignored".
     *
     * @this {KA655VAX}
     * @param {number} rg
     * @param {number} val
     * @param {boolean} fAligned true for an aligned longword write, matching `(pa & 3) == 0`
     * @returns {boolean} always true -- see above.
     */
    writeReg(rg, val, fAligned)
    {
        if (rg === REG_CACR && fAligned) {
            this.cacr = (this.cacr & ~(val & CACR_W1C)) | CACR_FIXED;
            this.cacr = (this.cacr & ~CACR_RW) | (val & CACR_RW);
        }
        return true;
    }

    /**
     * setCdgDiagParity(t)
     *
     * vax_sysdev.c:1231-1237, the tail of cdg_rd() -- a CDG READ writes THIS register:
     *
     *      ka_cacr = ka_cacr & ~CACR_DRO;
     *      ka_cacr = ka_cacr |
     *          (parity ((t >> 24) & 0xFF, 1) << (CACR_V_DPAR + 3)) |
     *          (parity ((t >> 16) & 0xFF, 0) << (CACR_V_DPAR + 2)) |
     *          (parity ((t >> 8)  & 0xFF, 1) << (CACR_V_DPAR + 1)) |
     *          (parity ( t        & 0xFF, 0) <<  CACR_V_DPAR);
     *
     * Note the ALTERNATING seed: bytes 3 and 1 seed odd=1, bytes 2 and 0 seed odd=0.
     *
     * THE PARITY BITS ACCUMULATE ACROSS READS, AND THAT IS NOT A TYPO HERE -- IT IS MEASURED.
     * `CACR_DRO` is 0x00FFFF00, i.e. bits 8-23, while `CACR_V_DPAR` is 24: the `& ~CACR_DRO` does
     * NOT cover the four bits the next line ORs in, so they are never cleared and the register ends
     * up holding the OR of every longword ever read.  A careful reader predicts the opposite (the
     * clear reads as if it were meant to reset the parity field), so this is not taken on faith:
     * tests/cdgdiff.js's CACR-ACCUM cases read two DIFFERENT longwords in sequence and compare the
     * resulting CACR against the live oracle, which returns 0x0F000000 where a non-accumulating
     * implementation returns 0x05000000.  Getting this "right" by clearing the parity bits too is
     * a real, catchable defect -- it is that file's CACR-DRO-MASK-WIDENED mutation.
     *
     * A consequence: `& ~CACR_DRO` is behaviourally INERT, because nothing on either engine ever
     * SETS a bit in 8-23 (ka_wr writes bits 0-6; this function writes 24-27).  It is reproduced
     * anyway, because the C does it, and cdgdiff.js asserts the inertness premise live on every
     * CACR case rather than leaving it as a claim in a comment.
     *
     * The seeds, the bit positions and the DRO clear mask are class data rather than four
     * hand-written lines so that tests/cdgdiff.js's --selfcheck can PERTURB the shipped computation
     * -- invert one lane's seed, swap two lanes' bit positions, widen the clear mask -- instead of
     * substituting its own copy of it (standing rule 11).
     *
     * @this {KA655VAX}
     * @param {number} t the longword cdg_rd() just returned
     */
    setCdgDiagParity(t)
    {
        let cacr = this.cacr & KA655VAX.CDG_DRO_CLEAR_MASK;
        for (let i = 0; i < 4; i++) {
            let b = (t >>> (i * 8)) & 0xFF;
            cacr |= parity(b, KA655VAX.CDG_DPAR_SEEDS[i]) << (CACR_V_DPAR + KA655VAX.CDG_DPAR_SHIFTS[i]);
        }
        this.cacr = cacr | 0;
    }

    /**
     * readLong(addr) / readWord(addr) / readByte(addr) / writeLong(addr, val) / writeWord(addr, val)
     * / writeByte(addr, val) -- `addr` is absolute physical, offset from KABASE by the caller
     * (regblock.js's makeRegController() passes the FULL address; `rg = addr >>> 2` here assumes
     * the caller has already normalized it relative to KABASE, matching cqbic.js's own convention
     * of taking `addr - BASE` -- see KA_BASE subtraction below).
     *
     * @this {KA655VAX}
     */
    readLong(addr) { return this.readReg(((addr >>> 0) - KA_BASE) >>> 2); }

    readWord(addr)
    {
        let v = this.readReg(((addr >>> 0) - KA_BASE) >>> 2);
        return (v >>> ((addr & 2) ? 16 : 0)) & 0xFFFF;
    }

    readByte(addr)
    {
        let v = this.readReg(((addr >>> 0) - KA_BASE) >>> 2);
        return (v >>> ((addr & 3) << 3)) & 0xFF;
    }

    /* Byte/word writes to CACR are NOT an aligned longword -- writeReg()'s own `fAligned` gate
       (reproducing vax_sysdev.c's `(pa & 3) == 0` check) already rejects them regardless of what
       value would result, so no read-modify-merge is needed here (unlike ssc.js/cqbic.js). */
    writeLong(addr, val) { return this.writeReg(((addr >>> 0) - KA_BASE) >>> 2, val | 0, (addr & 3) === 0); }
    writeWord(addr, val) { return this.writeReg(((addr >>> 0) - KA_BASE) >>> 2, val | 0, (addr & 3) === 0); }
    writeByte(addr, val) { return this.writeReg(((addr >>> 0) - KA_BASE) >>> 2, val | 0, (addr & 3) === 0); }
}

/*
 * cdg_rd()'s four diagnostic-parity lanes, as data.  Lane `i` takes byte `i` of the longword just
 * read, seeds parity() with CDG_DPAR_SEEDS[i] (vax_sysdev.c's ALTERNATING 0/1/0/1 -- bytes 3 and 1
 * seed odd=1, bytes 2 and 0 seed odd=0), and lands at bit CACR_V_DPAR + CDG_DPAR_SHIFTS[i].
 * CDG_DRO_CLEAR_MASK is cdg_rd()'s `ka_cacr & ~CACR_DRO`.  See setCdgDiagParity() for why these
 * are class data and not four inline lines.
 */
KA655VAX.CDG_DPAR_SEEDS  = [0, 1, 0, 1];
KA655VAX.CDG_DPAR_SHIFTS = [0, 1, 2, 3];
KA655VAX.CDG_DRO_CLEAR_MASK = ~CACR_DRO;

export { KA_BASE, CACR_DRO, CACR_V_DPAR };
