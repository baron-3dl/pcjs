/**
 * @fileoverview Implements the KA655 NVR (non-volatile RAM) register block -- physical decode
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
 * pcjsvax-bfb's own romdiff boundary-advance: after the SSC output port register (SSC+0x30,
 * pcjsvax-bfb) is decoded, the ROM's SAME instruction (a two-operand access) reaches into NVR+0x100
 * (physical 0x20140500) -- vax_sysdev.c's nvr_rd()/nvr_wr() (:629-646), a SEPARATE regtable entry
 * from ssc_rd()/ssc_wr() (vax_sysdev.c:1004 vs :1006) but landing in the SAME physical 8KB bus
 * block as the SSC (SSC_BASE=0x20140000, NVR_BASE=0x20140400, PCjs block size 0x2000) -- see
 * ssc.js's file header and bus.js's addSsc()/addNvrSsc() for how one controller has to answer for
 * both ranges plus the block's unused tail.
 *
 * NVR is a flat array of longwords, 1KB (VAX.PHYSMEM.NVR_LENGTH / 4 = 256 entries), with NO
 * side effects on read or write -- vax_sysdev.c's nvr_rd()/nvr_wr() are a bare array index plus
 * (for sub-longword accesses) the same read-modify-merge every register in this tree uses.  On
 * real hardware this is battery-backed and survives a power cycle; SIMH models that as "only
 * calloc when the backing store is NULL" (nvr_reset(), vax_sysdev.c:678-684) -- SIMH's own
 * `reset all` does NOT re-zero it.  This class's reset() is called ONLY from the constructor (see
 * ssc.js's SSCVAX.reset() doc comment for the identical reasoning and the same caveat about a
 * FUTURE reset-handler wiring), which reproduces exactly that: "the value a brand-new instance
 * starts with" and "the value after a cold power-up" are the same question here.
 */

import { VAX } from "./defines.js";

const NVR_BASE = VAX.PHYSMEM.NVR_BASE >>> 0;
const NVR_LENGTH = VAX.PHYSMEM.NVR_LENGTH;
const NVR_LONGS = NVR_LENGTH >>> 2;

/**
 * @class NVRVAX
 */
export default class NVRVAX {
    /**
     * NVRVAX()
     */
    constructor()
    {
        this.reset();
    }

    /**
     * reset()
     *
     * vax_sysdev.c:678-684, nvr_reset() -- `if (nvr == NULL) nvr = calloc(...)`.  Called only from
     * the constructor (see the file header), which is why a flat zero-fill here is correct: nothing
     * calls this a second time, matching "only allocate once" for the one call site that exists.
     *
     * @this {NVRVAX}
     */
    reset()
    {
        this.data = new Int32Array(NVR_LONGS);
    }

    /**
     * readReg(rg) / writeReg(rg, val)
     *
     * vax_sysdev.c's nvr_rd()/nvr_wr() are a bare `nvr[rg]` -- no switch, no reserved sub-range (the
     * whole NVR_LENGTH span is backed).  `rg` is bounds-checked here because VAX.PHYSMEM.NVR_LENGTH
     * (1KB) is smaller than the containing bus block (8KB); makeNvrController() below only ever
     * calls in with an address already known to lie inside [NVR_BASE, NVR_BASE+NVR_LENGTH), but the
     * check is kept so a future caller cannot silently read/write past the backing array.
     *
     * @this {NVRVAX}
     * @param {number} rg
     * @returns {?number}
     */
    readReg(rg)
    {
        if (rg < 0 || rg >= NVR_LONGS) return null;
        return this.data[rg] | 0;
    }

    /**
     * @this {NVRVAX}
     * @param {number} rg
     * @param {number} val
     * @returns {boolean}
     */
    writeReg(rg, val)
    {
        if (rg < 0 || rg >= NVR_LONGS) return false;
        this.data[rg] = val | 0;
        return true;
    }

    /**
     * readLong(addr) / readWord(addr) / readByte(addr) -- same shape as SSCVAX's, see ssc.js.
     *
     * @this {NVRVAX}
     * @param {number} addr
     * @returns {?number}
     */
    readLong(addr)
    {
        let rg = ((addr >>> 0) - NVR_BASE) >>> 2;
        return this.readReg(rg);
    }

    readWord(addr)
    {
        let rg = ((addr >>> 0) - NVR_BASE) >>> 2;
        let v = this.readReg(rg);
        if (v === null) return null;
        return (v >>> ((addr & 2) ? 16 : 0)) & 0xFFFF;
    }

    readByte(addr)
    {
        let rg = ((addr >>> 0) - NVR_BASE) >>> 2;
        let v = this.readReg(rg);
        if (v === null) return null;
        return (v >>> ((addr & 3) << 3)) & 0xFF;
    }

    /**
     * writeLong(addr, val) / writeWord(addr, val) / writeByte(addr, val)
     *
     * vax_sysdev.c:636-648's read-modify-merge, reproduced exactly as SSCVAX's byte/word writers
     * are (see ssc.js) -- the FULL merged longword goes to writeReg().
     *
     * @this {NVRVAX}
     * @param {number} addr
     * @param {number} val
     * @returns {boolean}
     */
    writeLong(addr, val)
    {
        let rg = ((addr >>> 0) - NVR_BASE) >>> 2;
        return this.writeReg(rg, val | 0);
    }

    writeWord(addr, val)
    {
        let rg = ((addr >>> 0) - NVR_BASE) >>> 2;
        let cur = this.readReg(rg);
        if (cur === null) return false;
        let sc = (addr & 2) ? 16 : 0;
        let merged = ((val & 0xFFFF) << sc) | (cur & ~(0xFFFF << sc));
        return this.writeReg(rg, merged | 0);
    }

    writeByte(addr, val)
    {
        let rg = ((addr >>> 0) - NVR_BASE) >>> 2;
        let cur = this.readReg(rg);
        if (cur === null) return false;
        let sc = (addr & 3) << 3;
        let merged = ((val & 0xFF) << sc) | (cur & ~(0xFF << sc));
        return this.writeReg(rg, merged | 0);
    }
}

export { NVR_BASE, NVR_LENGTH };
