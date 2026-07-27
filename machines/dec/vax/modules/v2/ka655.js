/**
 * @fileoverview Implements the KA655 CPU-board register pair (CACR cache control, BDR boot/diag)
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * pcjsvax-bfb's own romdiff boundary-advance: REG+0x4004 (KABASE+4, physical 0x20084004) -- vax_
 * sysdev.c's ka_rd()/ka_wr() (:1193-1216).  Two registers, KABASE+0 = CACR (cache control), KABASE+4
 * = BDR (boot/diagnostic register, READ-ONLY -- ka_wr()'s whole body is gated on `rg==0`, there is
 * no BDR write case at all, matching real hardware: BDR reflects boot-time switch/jumper state).
 *
 * NOT MODELED: the cache-diagnostic (CDG) side effects CACR's CEN/DIAG bits would otherwise gate --
 * this item's boundary never reaches CDG (VAX.PHYSMEM.CDG_BASE, a SEPARATE, still-undecoded range;
 * see mchkdiff.js's calibration note "CDG_BASE is backed END TO END... 100% expected divergence").
 * If a later boundary shows the ROM touching CDG, that item must extend this file or add a sibling.
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
     * @returns {?number}
     */
    readReg(rg)
    {
        switch (rg) {
        case REG_CACR: return this.cacr;
        case REG_BDR:  return this.bdr;
        }
        return null;
    }

    /**
     * writeReg(rg, val, fAligned)
     *
     * vax_sysdev.c:1211: `if ((rg == 0) && ((pa & 3) == 0))` -- CACR accepts ONLY an ALIGNED
     * longword write (the byte/word merge every other register in this tree does via a `t = ...rd()`
     * read-modify step is deliberately NOT reproduced here: real hardware's own gate excludes
     * anything but an aligned longword before it ever looks at the data).  BDR is read-only: no
     * `rg == 1` case exists in ka_wr() at all.
     *
     * @this {KA655VAX}
     * @param {number} rg
     * @param {number} val
     * @param {boolean} fAligned true for an aligned longword write, matching `(pa & 3) == 0`
     * @returns {boolean}
     */
    writeReg(rg, val, fAligned)
    {
        if (rg === REG_CACR && fAligned) {
            this.cacr = (this.cacr & ~(val & CACR_W1C)) | CACR_FIXED;
            this.cacr = (this.cacr & ~CACR_RW) | (val & CACR_RW);
            return true;
        }
        return false;
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
        if (v === null) return null;
        return (v >>> ((addr & 2) ? 16 : 0)) & 0xFFFF;
    }

    readByte(addr)
    {
        let v = this.readReg(((addr >>> 0) - KA_BASE) >>> 2);
        if (v === null) return null;
        return (v >>> ((addr & 3) << 3)) & 0xFF;
    }

    /* Byte/word writes to CACR are NOT an aligned longword -- writeReg()'s own `fAligned` gate
       (reproducing vax_sysdev.c's `(pa & 3) == 0` check) already rejects them regardless of what
       value would result, so no read-modify-merge is needed here (unlike ssc.js/cqbic.js). */
    writeLong(addr, val) { return this.writeReg(((addr >>> 0) - KA_BASE) >>> 2, val | 0, (addr & 3) === 0); }
    writeWord(addr, val) { return this.writeReg(((addr >>> 0) - KA_BASE) >>> 2, val | 0, (addr & 3) === 0); }
    writeByte(addr, val) { return this.writeReg(((addr >>> 0) - KA_BASE) >>> 2, val | 0, (addr & 3) === 0); }
}
