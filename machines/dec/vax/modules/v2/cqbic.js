/**
 * @fileoverview Implements the KA655 CQBIC (Qbus adapter chip) local register block
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
 * pcjsvax-bfb's own romdiff boundary-advance, continued past ssc.js's OTP/NVR registers: the ROM's
 * next undecoded reference is REG+0x0 (physical 0x20080000, CQBICBASE) -- vax_io.c's cqbic_rd()/
 * cqbic_wr() (:470-524), the Qbus adapter's five local registers (vaxmod_defs.h:190-207 names the
 * layout; CQBICBASE == REGBASE).
 *
 * DSER (register 1) and MEAR (register 2) are NOT new state: pcjsvax-d22 already models them on
 * VAXExc (`cpu.exc.cqDser`/`cpu.exc.cqMear`) for the machine-check side of an unbacked Qbus/CQBIC
 * reference (`cqMerr()`).  vax_io.c's `cq_dser`/`cq_mear` are the SAME C globals cqbic_rd()/
 * cqbic_wr() and cq_merr() both touch -- one register model, two callers -- so this class takes an
 * `exc` reference and reads/writes THROUGH it rather than keeping a second, divergeable copy.  SCR,
 * SEAR and MBR are new state, owned here.
 *
 * SCOPE, deliberately narrow (see HANDOFF.md rule 5, "never speculatively implement"): CQIPC
 * (`cq_ipc`, the inter-processor doorbell/W1C register at CQIPCBASE, a DIFFERENT regtable entry --
 * vax_sysdev.c's regtable, not this file's range) and CQMAP (the Qbus-to-memory window) are NOT
 * decoded here.  DSER's write-side `if (val & CQDSER_SME) cq_ipc = cq_ipc & ~CQIPC_QME;` side effect
 * on cq_ipc is therefore not reproduced; nothing in this item's boot-entry trace writes DSER with
 * CQDSER_SME set (mirroring EHKAA's own already-established pattern of exercising only a fraction
 * of a register's full bit space -- see docs/reference/ehkaa-profile.md).  If a LATER boundary shows
 * the ROM setting that bit, or touching CQIPC/CQMAP directly, that is the item that must add them.
 */

import { VAX } from "./defines.js";

const CQBIC_BASE = VAX.PHYSMEM.REG_BASE >>> 0;    // vaxmod_defs.h:195, CQBICBASE == REGBASE

const REG_SCR  = 0;
const REG_DSER = 1;
const REG_MEAR = 2;
const REG_SEAR = 3;
const REG_MBR  = 4;

/* vax_io.c:58-88. */
const CQSCR_POK  = 0x00008000;
const CQSCR_BHL  = 0x00004000;
const CQSCR_AUX  = 0x00000400;
const CQSCR_DBO  = 0x0000000C;
const CQSCR_RW   = (CQSCR_BHL | CQSCR_DBO);
const CQSCR_MASK = (CQSCR_RW | CQSCR_POK | CQSCR_AUX);

const CQDSER_MASK  = 0x0000C0BD;
const CQMEAR_MASK  = 0x00001FFF;
const CQSEAR_MASK  = 0x000FFFFF;
const CQMBR_MASK   = (0x1FFF8000 | 0);

/**
 * @class CQBICVAX
 */
export default class CQBICVAX {
    /**
     * CQBICVAX(exc)
     *
     * @param {Object} exc the owning CPU's VAXExc (cqDser/cqMear live there -- see the file header)
     */
    constructor(exc)
    {
        this.exc = exc;
        this.reset();
    }

    /**
     * reset()
     *
     * vax_io.c:742/754, qba_powerup()/qba_reset(): `cq_scr = CQSCR_POK` on a full reset (powerup
     * ORs in nothing else; a plain `RESET` preserves CQSCR_BHL -- `(cq_scr & CQSCR_BHL) | CQSCR_POK`
     * -- but this class's reset() is called only from the constructor, the powerup case, per the
     * same reasoning ssc.js's SSCVAX.reset() documents).  `cq_sear`/`cq_mbr` are zeroed by SIMH's
     * device reset (not separately listed in qba_reset() -- module-scope C globals default to 0 and
     * nothing else touches them before a `boot cpu`, confirmed by this item's own oracle probe).
     * cqDser/cqMear are NOT reset here: they live on `exc`, and VAXExc.reset() (exc.js) already
     * owns clearing them -- reproducing that here too would be a second, divergeable copy.
     *
     * @this {CQBICVAX}
     */
    reset()
    {
        this.scr = CQSCR_POK | 0;
        this.sear = 0;
        this.mbr = 0;
    }

    /**
     * readReg(rg)
     *
     * VERACITY FINDING (pcjsvax-bfb re-dispatch, same class as ssc.js's readReg()): cqbic_rd()'s
     * switch (vax_io.c:470-491) has no `default:` label either -- `switch (rg) { ...5 cases...; }
     * return 0;` -- so an `rg` this file does not case is a SILENT 0, never a fault.  Currently
     * UNREACHABLE in practice (regblock.js's addRegBlock() registers this device over exactly
     * `length: 0x14` = CQBICSIZE = these 5 registers and nothing more, so `rg` is always in [0,5)
     * by construction of the caller), but returning 0 here rather than null keeps this file correct
     * if that registration is ever widened, and matches the same "no default means silent, not
     * fault" contract ssc.js's readReg()/writeReg() now document explicitly.
     *
     * @this {CQBICVAX}
     * @param {number} rg
     * @returns {number}
     */
    readReg(rg)
    {
        switch (rg) {
        case REG_SCR:  return (this.scr | CQSCR_POK) & CQSCR_MASK;
        case REG_DSER: return this.exc.cqDser & CQDSER_MASK;
        case REG_MEAR: return this.exc.cqMear & CQMEAR_MASK;
        case REG_SEAR: return this.sear & CQSEAR_MASK;
        case REG_MBR:  return this.mbr & CQMBR_MASK;
        }
        return 0;
    }

    /**
     * writeReg(rg, val)
     *
     * vax_io.c:495-527.  `case 2: case 3:` (MEAR/SEAR) are READ-ONLY latches on real hardware: a
     * program WRITE to either one is itself a bus error (`cq_merr()` + a synchronous machine check,
     * vax_io.c:518-520) -- reproduced here by returning false (the caller's bus-fault path).  THIS
     * IS THE ONLY CASE IN THIS ENTIRE DEVICE FILE THAT GENUINELY FAULTS ON WRITE -- do not confuse
     * it with "uncased register" (the trailing default below, and the KA655/SSC equivalents in
     * ka655.js/ssc.js), which is a SILENT NO-OP, not a fault.  Conflating the two -- "no case
     * matched" and "this specific register is a deliberate bus error" -- into a single `false`
     * return was exactly the veracity finding this re-dispatch fixes elsewhere in this codebase
     * (ka655.js's BDR, ssc.js's uncased offsets); it happens to have been RIGHT here by construction
     * only because MEAR/SEAR really do fault and every other rg reaching this switch is cased.
     *
     * @this {CQBICVAX}
     * @param {number} rg
     * @param {number} val
     * @returns {boolean} true for a handled write (cased and applied); false ONLY for MEAR/SEAR,
     *   which must genuinely fault -- never returned for "not cased" (see readReg()'s doc comment;
     *   the trailing default below returns true, a silent accept, matching cqbic_wr()'s own
     *   default-less switch).
     */
    writeReg(rg, val)
    {
        switch (rg) {
        case REG_SCR:
            this.scr = (((this.scr & ~CQSCR_RW) | (val & CQSCR_RW)) & CQSCR_MASK) | 0;
            return true;
        case REG_DSER:
            this.exc.cqDser = (this.exc.cqDser & ~val) & CQDSER_MASK;
            return true;
        case REG_MEAR:
        case REG_SEAR:
            return false;                        // vax_io.c: cq_merr() + MACH_CHECK -- a REAL bus error, not a fallthrough
        case REG_MBR:
            this.mbr = (val & CQMBR_MASK) | 0;
            return true;
        }
        return true;
    }

    /**
     * readLong(addr) / readWord(addr) / readByte(addr) -- same shape as SSCVAX's (see ssc.js);
     * `addr` is absolute physical, already known by the caller to lie inside CQBIC's range.
     *
     * @this {CQBICVAX}
     */
    readLong(addr) { return this.readReg(((addr >>> 0) - CQBIC_BASE) >>> 2); }

    readWord(addr)
    {
        let v = this.readReg(((addr >>> 0) - CQBIC_BASE) >>> 2);
        return (v >>> ((addr & 2) ? 16 : 0)) & 0xFFFF;
    }

    readByte(addr)
    {
        let v = this.readReg(((addr >>> 0) - CQBIC_BASE) >>> 2);
        return (v >>> ((addr & 3) << 3)) & 0xFF;
    }

    /**
     * writeLong(addr, val) / writeWord(addr, val) / writeByte(addr, val)
     *
     * vax_io.c:495-528's byte/word path computes TWO shifted values -- `nval` (read-modify-MERGED
     * with the current register, `t = cqbic_rd(pa)`, exactly ssc.js's writeWord()/writeByte()
     * pattern) for SCR/MBR, and plain shifted `val` (no merge) for DSER's W1C semantics, so that a
     * byte/word W1C write clears bits only in its own lane rather than being merged against a
     * register the FINAL step is going to AND-NOT anyway.  `writeReg()` above already expects the
     * MERGED-longword shape SCR/MBR/DSER's C bodies each individually produce (SCR/MBR: `nval`;
     * DSER: `val` itself, which -- once shifted into its own lane and combined with zero elsewhere
     * -- IS `cq_dser`'s post-clear intent one bit at a time, since `& ~val` on the untouched lanes'
     * zero bits is a no-op).  So a single read-modify-merge into a full longword, passed to
     * writeReg(), reproduces every case correctly without special-casing DSER: for SCR/MBR it is
     * exactly `nval`; for DSER, merging the shifted byte against the CURRENT `cqDser` value (rather
     * than zero) and handing the WHOLE THING to writeReg() -- which does `cqDser & ~val` -- clears
     * only the bits the write's own byte lane could have set, because the merge's OTHER lanes carry
     * `cqDser`'s OWN current bits, and `cqDser & ~cqDser-bit` for an unwritten lane is a no-op
     * exactly where it needs to be.  MEAR/SEAR (read-only, a bus error to write at all) never reach
     * writeReg() successfully either way, so the merge there is moot.
     *
     * @this {CQBICVAX}
     */
    writeLong(addr, val) { return this.writeReg(((addr >>> 0) - CQBIC_BASE) >>> 2, val | 0); }

    writeWord(addr, val)
    {
        let rg = ((addr >>> 0) - CQBIC_BASE) >>> 2;
        let cur = this.readReg(rg);
        if (cur === null) return false;
        let sc = (addr & 2) ? 16 : 0;
        let merged = ((val & 0xFFFF) << sc) | (cur & ~(0xFFFF << sc));
        return this.writeReg(rg, merged | 0);
    }

    writeByte(addr, val)
    {
        let rg = ((addr >>> 0) - CQBIC_BASE) >>> 2;
        let cur = this.readReg(rg);
        if (cur === null) return false;
        let sc = (addr & 3) << 3;
        let merged = ((val & 0xFF) << sc) | (cur & ~(0xFF << sc));
        return this.writeReg(rg, merged | 0);
    }
}

export { CQSCR_POK, CQSCR_MASK, CQDSER_MASK, CQMEAR_MASK, CQSEAR_MASK, CQMBR_MASK };
