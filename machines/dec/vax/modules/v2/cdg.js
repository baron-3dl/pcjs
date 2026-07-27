/**
 * @fileoverview Implements the KA655 cache diagnostic space (CDG) -- physical decode
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
 * pcjsvax-0b7's romdiff boundary-advance: the ROM's instruction #393 (PC=200420E9) is
 * `MOVC5 #0,#0,#0,#8000,@#10000000` -- a 0x8000-byte block fill into VAX.PHYSMEM.CDG_BASE, the
 * cache diagnostic space.  Open SIMH services the whole move through vax_sysdev.c's cdg_rd()/
 * cdg_wr() (:1226-1252), reached through the regtable[] entry `{ CDGBASE, CDGBASE+CDGSIZE,
 * &cdg_rd, &cdg_wr }` (vax_sysdev.c:1010).  Before this file existed the range was reserved and
 * undecoded, so the move raised a bus fault and machine-checked -- correct behaviour for an
 * undecoded range, wrong answer for this one.
 *
 * ============================================================================
 * THE GEOMETRY IS THE NON-OBVIOUS PART: A 64MB RANGE OVER A 64KB STORE
 * ============================================================================
 * vaxmod_defs.h:145-161.  CDGSIZE (the ADDRESS range) is CDASIZE * CTGSIZE = 0x10000 * 0x400 =
 * 0x04000000, sixty-four megabytes.  The BACKING STORE, `int32 cdg_dat[CDASIZE >> 2]`, is
 * CDASIZE bytes -- SIXTY-FOUR KILOBYTES, 16384 longwords -- because `CDG_GETROW(x)` is
 * `((x) & CDAMASK) >> 2` and CDAMASK is 0xFFFF.  The whole 64MB range therefore ALIASES onto one
 * 64KB store: physical 0x10000000, 0x10010000, 0x10020000 ... all address row 0.
 *
 * A flat 64MB array would be both a 64MB allocation and observably WRONG the moment anything reads
 * back through an alias, which tests/cdgdiff.js's ALIAS phase proves against the live oracle on
 * both engines.  This class models the 64KB store and the row mask, not the address range.
 *
 * `CDG_GETTAG(x)` and the CTG_V / CTG_WP bits exist in vaxmod_defs.h but cdg_rd()/cdg_wr() NEVER
 * REFERENCE THEM -- there is no tag store on this path, and none is invented here.
 *
 * ============================================================================
 * WHAT IS AND IS NOT MODELED
 * ============================================================================
 * MODELED: the row aliasing above; cdg_rd()'s side effect on the KA655 CACR (it clears CACR_DRO
 * and ORs in four computed diagnostic-parity bits -- see KA655VAX.setCdgDiagParity() in ka655.js,
 * which owns the CACR bit definitions, and which this class calls through the `ka655` constructor
 * argument); cdg_wr()'s OWN byte/word read-modify-merge (unlike the SSC, where the merges are done
 * in the caller path -- cdg_wr does its own, and has NO CACR side effect); and the fact that every
 * address in [CDG_BASE, CDG_BASE+CDG_LENGTH) reads and writes without faulting.
 *
 * NOT MODELED: WriteRegU() (vax_sysdev.c:1084-1091), the register-space UNALIGNED write path,
 * which read-modify-writes through ReadReg()/WriteReg() and therefore drags cdg_rd()'s CACR side
 * effect along with an unaligned write.  This bus reaches an unaligned register write through
 * MemoryVAX's own byte/word stitching instead, which does not reproduce that.  It is NOT a silent
 * gap: tests/cdgdiff.js's MERGE phase grades word writes only at the two byte lanes that are
 * genuinely aligned words (lanes 0 and 2, the only ones that reach cdg_wr with lnt == L_WORD on
 * real hardware), states in its own header that lanes 1 and 3 are the unaligned path, and byte
 * writes -- which are aligned by definition and are what the ROM's MOVC5 actually issues -- are
 * graded at ALL FOUR lanes.  Extending the bus to model WriteRegU is a separate item.
 */

import { VAX } from "./defines.js";

const CDG_BASE = VAX.PHYSMEM.CDG_BASE >>> 0;

/* vaxmod_defs.h:145-153.  Transcribed as the DEFINES, with CDGSIZE recomputed from them below and
   asserted against VAX.PHYSMEM.CDG_LENGTH, rather than the range length being restated by hand. */
const CDAAWIDTH = 16;                           // cache dat addr width
const CDASIZE   = 1 << CDAAWIDTH;               // cache dat length -- THE WHOLE BACKING STORE
const CDAMASK   = CDASIZE - 1;
const CTGAWIDTH = 10;                           // cache tag addr width
const CTGSIZE   = 1 << CTGAWIDTH;               // cache tag length
const CDGSIZE   = CDASIZE * CTGSIZE;            // diag ADDRESS length (not the store's size)
const CDG_ROWS  = CDASIZE >>> 2;                // `int32 cdg_dat[CDASIZE >> 2]`

if (CDGSIZE !== VAX.PHYSMEM.CDG_LENGTH) {
    throw new Error(`cdg.js: CDASIZE*CTGSIZE = 0x${CDGSIZE.toString(16)} but ` +
        `VAX.PHYSMEM.CDG_LENGTH = 0x${VAX.PHYSMEM.CDG_LENGTH.toString(16)} -- the cache geometry ` +
        `and the physical map disagree; one of them is wrong.`);
}

/**
 * @class CDGVAX
 */
export default class CDGVAX {
    /**
     * CDGVAX(ka655)
     *
     * @param {Object} [ka655] the KA655VAX instance whose CACR cdg_rd()'s side effect updates.
     *   Optional ONLY so a bare instance can be exercised without a whole machine; a machine that
     *   decodes both CDG and the KA655 register pair MUST pass it, or the diagnostic-parity bits
     *   silently never appear (tests/cdgdiff.js's CACR phase is what proves they do).
     */
    constructor(ka655)
    {
        this.ka = ka655 || null;
        this.reset();
    }

    /**
     * reset()
     *
     * vax_sysdev.c:1774 (sysd_reset(): `cdg_dat` is calloc'd once, in sysd_powerup()/rom_reset()
     * style -- a zero-filled store).  Called only from the constructor, matching nvr.js's identical
     * reasoning: "the value a brand-new instance starts with" and "the value after a cold power-up"
     * are the same question for a calloc'd array.
     *
     * @this {CDGVAX}
     */
    reset()
    {
        this.dat = new Int32Array(CDG_ROWS);
    }

    /**
     * rowOf(addr)
     *
     * vaxmod_defs.h:155, `CDG_GETROW(x) = ((x) & CDAMASK) >> 2`.  This ONE expression is the whole
     * aliasing contract: bits above 15 of the physical address are DISCARDED, so every 64KB window
     * of the 64MB range lands on the same 16384 rows.
     *
     * @this {CDGVAX}
     * @param {number} addr (physical)
     * @returns {number} row index into this.dat
     */
    rowOf(addr)
    {
        return ((addr >>> 0) & CDAMASK) >>> 2;
    }

    /**
     * readCdg(addr)
     *
     * vax_sysdev.c:1226-1240, cdg_rd().  Returns the WHOLE longword (the byte/word extraction is
     * the caller's, matching vax_mmu.h's ReadB()/ReadW(), which shift and mask what ReadReg()
     * hands back) and applies the CACR diagnostic-parity side effect -- for EVERY access size,
     * because cdg_rd() itself never sees the length.
     *
     * @this {CDGVAX}
     * @param {number} addr (physical)
     * @returns {number} int32
     */
    readCdg(addr)
    {
        let t = this.dat[this.rowOf(addr)] | 0;
        if (this.ka) this.ka.setCdgDiagParity(t);
        return t;
    }

    /**
     * writeCdg(addr, val, lnt)
     *
     * vax_sysdev.c:1242-1252, cdg_wr().  cdg_wr does its OWN byte/word read-modify-merge -- unlike
     * the SSC, whose merges happen in ssc_wr()'s caller path -- and, unlike cdg_rd(), touches
     * NOTHING on the CACR.
     *
     * `CDGVAX.L_LONG` is the C's `if (lnt < L_LONG)` gate, and `CDGVAX.MERGE_MASKS` is its
     * `(lnt == L_WORD)? 0xFFFF: 0xFF`, both kept as named data on the class so tests/cdgdiff.js's
     * --selfcheck can PERTURB the shipped merge (standing rule 11: a mutation composes over or
     * perturbs the real code, it never substitutes its own copy of it).
     *
     * @this {CDGVAX}
     * @param {number} addr (physical)
     * @param {number} val right-justified
     * @param {number} lnt 1, 2 or 4
     */
    writeCdg(addr, val, lnt)
    {
        let row = this.rowOf(addr);
        if (lnt < CDGVAX.L_LONG) {
            let sc = (addr & 3) << 3;
            let mask = CDGVAX.MERGE_MASKS[lnt];
            let t = this.dat[row] | 0;
            val = ((val & mask) << sc) | (t & ~(mask << sc));
        }
        this.dat[row] = val | 0;
    }

    /**
     * readLong(addr) / readWord(addr) / readByte(addr)
     *
     * vax_mmu.h:275-305's ReadB()/ReadW(): the register handler always returns a full longword and
     * the CALLER extracts, which is why all three go through readCdg() and therefore all three
     * trigger the CACR side effect.  Same shape as nvr.js's, minus its bounds check: the CDG range
     * is backed END TO END (rowOf() masks, so no address can escape the store).
     *
     * @this {CDGVAX}
     * @param {number} addr (physical)
     * @returns {number}
     */
    readLong(addr) { return this.readCdg(addr); }

    readWord(addr) { return (this.readCdg(addr) >>> ((addr & 2) ? 16 : 0)) & 0xFFFF; }

    readByte(addr) { return (this.readCdg(addr) >>> ((addr & 3) << 3)) & 0xFF; }

    /**
     * writeLong(addr, val) / writeWord(addr, val) / writeByte(addr, val)
     *
     * The merge lives in writeCdg() (cdg_wr's own), NOT here -- deliberately unlike nvr.js/ssc.js,
     * whose C counterparts merge in the caller.  Always true: nothing in this range faults.
     *
     * @this {CDGVAX}
     * @param {number} addr (physical)
     * @param {number} val
     * @returns {boolean} always true
     */
    writeLong(addr, val) { this.writeCdg(addr, val | 0, 4); return true; }

    writeWord(addr, val) { this.writeCdg(addr, val | 0, 2); return true; }

    writeByte(addr, val) { this.writeCdg(addr, val | 0, 1); return true; }
}

/* vax_defs.h's L_BYTE/L_WORD/L_LONG, and cdg_wr()'s `(lnt == L_WORD)? 0xFFFF: 0xFF`.  Indexed by
   lnt so the shipped gate reads exactly like the C.  See writeCdg() for why these are class data. */
CDGVAX.L_LONG = 4;
CDGVAX.MERGE_MASKS = [0, 0xFF, 0xFFFF, 0xFF, 0];

/**
 * makeCdgController(cdg)
 *
 * A MemoryVAX controller (memory.js's constructor contract: `getControllerBuffer(addr)` ->
 * `[adw, offset]`, `getControllerAccess()` -> the 6-entry afn table) covering the WHOLE
 * VAX.PHYSMEM.CDG_LENGTH span.  There is no buffer -- the store is CDGVAX's own 64KB Int32Array,
 * which is 1/1024th the size of the address range and therefore cannot be handed to MemoryVAX as
 * a per-block backing array the way BusVAX.makeRomAliasController() hands back the ROM's.
 *
 * Unlike regblock.js's dispatcher there is no fall-through-to-readNone path: cdg_rd()/cdg_wr()
 * answer for every address in the range, so a null return here would be a bug, not a gap.
 *
 * THE ACCESS TABLE IS BUILT ONCE PER CONTROLLER, NOT ONCE PER BLOCK.  MemoryVAX's constructor calls
 * getControllerBuffer() AND getControllerAccess() for EVERY block it creates (memory.js's
 * `if (controller)` path), and this controller is installed over 0x04000000 / BusVAX.BLOCK_SIZE =
 * 8192 blocks -- by a wide margin the largest span any controller in this tree covers (the SSC's is
 * one block, the register block's 64).  An earlier version returned a FRESH six-closure array from
 * getControllerAccess(), i.e. ~49,000 closures per machine constructed.  MEASURED, not theorised:
 * tests/cdgdiff.js, which then built one machine per case, reached 8.6GB RSS and was killed by the
 * kernel OOM killer.  Both halves of that defect were fixed -- the table below is hoisted so all
 * 8192 blocks share ONE array of six functions (the closures capture `cdg`, which is per-controller,
 * so sharing is safe), and cdgdiff.js now builds ONE machine and reuses it.
 *
 * @param {CDGVAX} cdg
 * @returns {Object}
 */
export function makeCdgController(cdg)
{
    const afn = [
        function readByte(off, addr) { return cdg.readByte(addr >>> 0); },
        function writeByte(off, b, addr) { cdg.writeByte(addr >>> 0, b); },
        function readWord(off, addr) { return cdg.readWord(addr >>> 0); },
        function writeWord(off, w, addr) { cdg.writeWord(addr >>> 0, w); },
        function readLong(off, addr) { return cdg.readLong(addr >>> 0); },
        function writeLong(off, l, addr) { cdg.writeLong(addr >>> 0, l); }
    ];
    const buf = [null, 0];
    return {
        getControllerBuffer(addr) { return buf; },
        getControllerAccess() { return afn; }
    };
}

export { CDG_BASE, CDGSIZE, CDASIZE, CDAMASK, CDG_ROWS };
