/**
 * @fileoverview Implements the KA655 SSC (System Support Chip) register block -- physical decode
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
 * pcjsvax-320.  vax_sysdev.c's regtable installs ONE {read, write} pair for the WHOLE span
 * [SSCBASE, SSCBASE+SSCSIZE) (ReadReg()/WriteReg(), vax_sysdev.c:1006) -- meaning a real KA655
 * NEVER machine-checks anywhere in that range: even an rg value ssc_rd()/ssc_wr()'s switch does
 * not list falls through to "return 0" / a silent no-op, not a fault (vax_sysdev.c:1264-1458).
 * pcjsvax-223 measured the ROM's SECOND instruction as a self-referential store to SSCBASE+0x0
 * (the base register): `MOVL #20140000,@#20140000`.  This file decodes exactly that register, and
 * whatever the item's own measurement (tests/romdiff.js, re-run after each addition) showed the ROM
 * touching immediately afterward -- see the register table below for the exact set and why each
 * one is or is not here.
 *
 * ============================================================================
 * SCOPE -- deliberately NOT the whole SSC (pcjsvax-320's own constraint)
 * ============================================================================
 * DECODED here (vax_sysdev.c ssc_rd()/ssc_wr(), rg = (pa - SSCBASE) >> 2):
 *
 *   rg    name   byte offset   what
 *   0x00  BASE   +0x00         the SSC base register itself (RW, SSCBASE_RW mask | SSCBASE_MBO)
 *
 * NOT HERE, BY DELIBERATE JUDGEMENT (both remain UNDECODED -- see "WHAT HAPPENS TO EVERYTHING
 * ELSE" below, not a silent gap):
 *
 *   - The console UART mirror (CSRS/CSRD/CSTS/CSTD at +0x70/+0x74/+0x78/+0x7C, RXCS/RXDB/TXCS/TXDB
 *     at +0x80/+0x84/+0x88/+0x8C).  vax_sysdev.c's ssc_rd()/ssc_wr() dispatch these to the SAME
 *     underlying register model MTPR/MFPR's IPR path uses for MT.CSRS..MT.TXDB (exc.js's
 *     IPR_DEVICE list, `setIPRDevice()`) -- one register model, two address paths.  Building that
 *     model here, before any item owns the console device itself, would mean re-deciding its shape
 *     twice.  Judged to belong with pcjsvax-bfb (the console item), which already owns the IPR half
 *     of the identical registers; this file leaves the memory-mapped half exactly as undecoded as
 *     it always was, so whichever item lands the model first wires BOTH address paths against it.
 *   - The SSC's two programmable timers T0/T1 (+0x100..+0x11C) and their address-strobe compare
 *     registers (+0x130/+0x134/+0x140/+0x144) -- explicitly the timer items' scope per this item's
 *     own text, not this one's, and no ROM boot-entry instruction this item measured needed them.
 *
 * ============================================================================
 * WHAT HAPPENS TO EVERYTHING ELSE IN THIS BLOCK (NOT a silent gap)
 * ============================================================================
 * bus.js installs this decode over exactly [SSC_BASE, SSC_BASE+SSC_LENGTH) (see BusVAX.addSsc()).
 * Physical memory blocks are managed in BusVAX.BLOCK_SIZE (8KB) chunks, and SSC_BASE and NVR_BASE
 * share ONE such block (SSC_BASE=0x20140000, NVR_BASE=0x20140400, block size 0x2000) -- so THIS
 * controller's functions are the ones invoked for every address in that whole 8KB span, not merely
 * SSC's own 0x150 bytes.  makeSscController() below checks the physical address against
 * [SSC_BASE, SSC_BASE+SSC_LENGTH) explicitly and, for anything outside it (NVR, and the unused
 * tail out to the next block boundary) or inside it but not one of the registers listed above,
 * falls through to the SAME readNone/writeNone/readWordNone/... the shared empty block would have
 * used -- i.e. a bus fault, dispatched by cpustate.js's onBusFault() into a real machine check
 * (pcjsvax-446), EXACTLY the status quo before this item for every one of those addresses.  That
 * fault is what tests/romdiff.js's probeSimhBackedAt() uses to find and NAME the next boundary --
 * the same mechanism that named THIS item's own starting point (SSC+0x0).
 *
 * ============================================================================
 * WHAT IS NOT MODELLED: cross-register unaligned access
 * ============================================================================
 * vax_mmu.h's ReadB/ReadW (and WriteB/WriteW) compute the FULL longword value of the containing
 * register (ssc_rd()'s `rg = (pa - SSCBASE) >> 2` truncates any misalignment WITHIN a longword to
 * the same rg, so this is exactly what real hardware does too) and then shift/mask a single byte
 * or word out of -- or into -- IT ALONE.  There is no code path, on real SIMH or here, that
 * stitches two ADJACENT registers together for an unaligned reference; unlike ordinary RAM, which
 * permits exactly that (memory.js's readWordMemory/readLongMemory).  Reproduced faithfully for the
 * single-register case; a reference straddling two SSC registers is not exercised by anything
 * tests/romdiff.js, tests/mchkdiff.js or tests/busdiff.js drives (none of them address SSCBASE at
 * all except romdiff.js, and only at the aligned base register), and is not modelled -- if a later
 * item's workload ever needs it, extend readByte()/readWord()/writeByte()/writeWord() below rather
 * than assuming the RAM-style stitch applies to a register file.
 */

import { VAX } from "./defines.js";

const SSC_BASE = VAX.PHYSMEM.SSC_BASE >>> 0;
const SSC_LENGTH = VAX.PHYSMEM.SSC_LENGTH;

/*
 * Register longword indices, vax_sysdev.c:1266 (`rg = (pa - SSCBASE) >> 2`) and :1354 (same, for
 * writes).  Only the registers this item decodes get a name; see the file header for the rest.
 */
const REG_BASE = 0x00;

/* SSC base register, vax_sysdev.c:144-145. */
const SSCBASE_MBO = 0x20000000 | 0;             // must-be-one bits
const SSCBASE_RW  = 0x1FFFFC00;                 // the only bits software can actually change

/**
 * @class SSCVAX
 */
export default class SSCVAX {
    /**
     * SSCVAX()
     */
    constructor()
    {
        this.reset();
    }

    /**
     * reset()
     *
     * vax_sysdev.c's global initializers (`int32 ssc_base = SSCBASE;`, vax_sysdev.c:244) and
     * sysd_powerup() (vax_sysdev.c:1787, `ssc_base = SSCBASE`) agree on the same value, so one
     * reset() serves both "cold" and "power-up" for the one register this file models.
     *
     * @this {SSCVAX}
     */
    reset()
    {
        this.base = SSC_BASE | 0;
    }

    /**
     * readReg(rg)
     *
     * vax_sysdev.c:1264, ssc_rd() -- the register switch only, no byte/word merge (see readLong()/
     * readWord()/readByte() below for that; SIMH's ssc_rd() itself is also merge-free, called with
     * the FULL aligned longword's value expected back, per vax_mmu.h's ReadB/ReadW/ReadL).
     *
     * @this {SSCVAX}
     * @param {number} rg
     * @returns {?number} the register's value, or null if this file does not decode it (the
     *   caller must treat that exactly like a probe of any other still-reserved address)
     */
    readReg(rg)
    {
        switch (rg) {
        case REG_BASE:
            return this.base;
        }
        return null;
    }

    /**
     * writeReg(rg, val)
     *
     * vax_sysdev.c:1352, ssc_wr() -- the register switch only.  `val` is the FULL merged longword
     * (see writeLong()/writeWord()/writeByte() below for the byte/word pre-merge SIMH's ssc_wr()
     * does inline via `t = ssc_rd(pa)` before this switch ever runs).
     *
     * @this {SSCVAX}
     * @param {number} rg
     * @param {number} val
     * @returns {boolean} true if this file decodes that register (and therefore wrote it), false
     *   if the caller must treat this exactly like a probe of any other still-reserved address
     */
    writeReg(rg, val)
    {
        switch (rg) {
        case REG_BASE:
            this.base = ((val & SSCBASE_RW) | SSCBASE_MBO) | 0;
            return true;
        }
        return false;
    }

    /**
     * readLong(addr) / readWord(addr) / readByte(addr)
     *
     * `addr` is the PHYSICAL address (not block-relative), already known by the caller to lie
     * within [SSC_BASE, SSC_BASE + SSC_LENGTH) -- makeSscController() below is what enforces that;
     * these three assume it.  `rg` truncates any misalignment to the containing register, exactly
     * as vax_sysdev.c's ssc_rd()/ssc_wr() do (see the file header's "WHAT IS NOT MODELLED" note).
     *
     * @this {SSCVAX}
     * @param {number} addr
     * @returns {?number}
     */
    readLong(addr)
    {
        let rg = ((addr >>> 0) - SSC_BASE) >>> 2;
        return this.readReg(rg);
    }

    readWord(addr)
    {
        let rg = ((addr >>> 0) - SSC_BASE) >>> 2;
        let v = this.readReg(rg);
        if (v === null) return null;
        return (v >>> ((addr & 2) ? 16 : 0)) & 0xFFFF;
    }

    readByte(addr)
    {
        let rg = ((addr >>> 0) - SSC_BASE) >>> 2;
        let v = this.readReg(rg);
        if (v === null) return null;
        return (v >>> ((addr & 3) << 3)) & 0xFF;
    }

    /**
     * writeLong(addr, val) / writeWord(addr, val) / writeByte(addr, val)
     *
     * The byte/word forms reproduce ssc_wr()'s inline read-modify-merge (vax_sysdev.c:1356-1361:
     * `sc = (pa & 3) << 3; t = ssc_rd(pa); val = ((val & mask) << sc) | (t & ~(mask << sc));`)
     * before handing the FULL merged longword to writeReg(), which is exactly what lets a byte
     * write to a W1C register (there are none decoded here yet, but the next one added inherits
     * this correctly) see only the ONE byte's worth of new bits.
     *
     * @this {SSCVAX}
     * @param {number} addr
     * @param {number} val
     * @returns {boolean}
     */
    writeLong(addr, val)
    {
        let rg = ((addr >>> 0) - SSC_BASE) >>> 2;
        return this.writeReg(rg, val | 0);
    }

    writeWord(addr, val)
    {
        let rg = ((addr >>> 0) - SSC_BASE) >>> 2;
        let cur = this.readReg(rg);
        if (cur === null) return false;
        let sc = (addr & 2) ? 16 : 0;
        let merged = ((val & 0xFFFF) << sc) | (cur & ~(0xFFFF << sc));
        return this.writeReg(rg, merged | 0);
    }

    writeByte(addr, val)
    {
        let rg = ((addr >>> 0) - SSC_BASE) >>> 2;
        let cur = this.readReg(rg);
        if (cur === null) return false;
        let sc = (addr & 3) << 3;
        let merged = ((val & 0xFF) << sc) | (cur & ~(0xFF << sc));
        return this.writeReg(rg, merged | 0);
    }
}

/**
 * makeSscController(ssc)
 *
 * A MemoryVAX controller (see bus.js's makeRomAliasController() for the same pattern applied to
 * the ROM mirror): `getControllerBuffer()` supplies no backing array (every access is computed,
 * never stored in an Int32Array), and `getControllerAccess()` returns six functions that are
 * installed as an ordinary MemoryVAX block's readByte/writeByte/readWord/writeWord/readLong/
 * writeLong -- called as `block.readByte(off, addr)`, so `this` inside them is the MemoryVAX
 * block, which is what makes `this.readNone`/`this.writeNone`/... (inherited from
 * MemoryVAX.prototype) the right fallback: the EXACT function the shared empty block would have
 * used for this address, before this controller existed.  See the file header's "WHAT HAPPENS TO
 * EVERYTHING ELSE" section for why that fallback fires for NVR, the block's unused tail, AND any
 * SSC sub-register this file does not decode -- three different reasons, one identical, correct,
 * bus-fault-and-let-romdiff-name-it outcome.
 *
 * @param {SSCVAX} ssc
 * @returns {Object}
 */
export function makeSscController(ssc)
{
    const LOW = SSC_BASE, HIGH = (SSC_BASE + SSC_LENGTH) >>> 0;
    function inRange(addr) { return addr >= LOW && addr < HIGH; }

    return {
        getControllerBuffer(addr) { return [null, 0]; },
        getControllerAccess() {
            return [
                function readByte(off, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr)) {
                        let v = ssc.readByte(addr);
                        if (v !== null) return v;
                    }
                    return this.readNone(off, addr);
                },
                function writeByte(off, b, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr) && ssc.writeByte(addr, b)) return;
                    this.writeNone(off, b, addr);
                },
                function readWord(off, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr)) {
                        let v = ssc.readWord(addr);
                        if (v !== null) return v;
                    }
                    return this.readWordNone(off, addr);
                },
                function writeWord(off, w, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr) && ssc.writeWord(addr, w)) return;
                    this.writeNone(off, w, addr);
                },
                function readLong(off, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr)) {
                        let v = ssc.readLong(addr);
                        if (v !== null) return v;
                    }
                    return this.readLongNone(off, addr);
                },
                function writeLong(off, l, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr) && ssc.writeLong(addr, l)) return;
                    this.writeNone(off, l, addr);
                }
            ];
        }
    };
}

export { SSC_BASE, SSC_LENGTH };
