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
 * vax_sysdev.c's regtable, not this file's range) is NOT decoded here.  DSER's write-side
 * `if (val & CQDSER_SME) cq_ipc = cq_ipc & ~CQIPC_QME;` side effect
 * on cq_ipc is therefore not reproduced; nothing in this item's boot-entry trace writes DSER with
 * CQDSER_SME set (mirroring EHKAA's own already-established pattern of exercising only a fraction
 * of a register's full bit space -- see docs/reference/ehkaa-profile.md).  If a LATER boundary shows
 * the ROM setting that bit, or touching CQIPC directly, that is the item that must add them.
 *
 * ============================================================================
 * THE SCATTER-GATHER MAP AND THE DMA DATA PATH (pcjsvax-e22)
 * ============================================================================
 * CQMAP -- the 8192-entry Qbus-to-memory translation map at CQMAPBASE (REGBASE+0x8000,
 * vaxmod_defs.h:196-199) -- and the four DMA entry points every Qbus mass-storage device calls
 * (Map_ReadB/Map_ReadW/Map_WriteB/Map_WriteW, vaxmod_defs.h:459-462, defined in vax_io.c:769-895)
 * are added here by pcjsvax-e22, ahead of the MSCP/RQ controller that will be their first caller
 * (pcjsvax-6a5).  They are graded by tests/qdmadiff.js against a real Open SIMH microvax3900.
 *
 * ONE TRANSLATION CORE, TWO CALLERS, exactly as in the C: `mapAddr()` is qba_map_addr()
 * (vax_io.c:666-682), and it is what BOTH the DMA routines and (in SIMH) cqm_rd()/cqm_wr() walk.
 * The map ENTRIES are not registers at all -- they live in ordinary main memory at
 * `(pa & CQMAPAMASK) + cq_mbr`, and cqmap_rd()/cqmap_wr() (vax_io.c:574-602) are a WINDOW onto
 * that memory, which is why `mapRegRead()`/`mapRegWrite()` below take a bus reference rather than
 * owning storage.  A test that programs the map with real CPU instructions through CQMAPVAX and
 * then DMAs through `mapReadB()` is therefore grading one mechanism end to end, not two models
 * that could drift.
 *
 * NOT IMPLEMENTED HERE, and named rather than left silent:
 *   - cqm_rd()/cqm_wr() (vax_io.c:617-664), the CPU-side Qbus-memory WINDOW at CQMBASE that walks
 *     the same map.  It stays undecoded (tests/cqmerrdiff.js grades the current unbacked
 *     behaviour) because its map-failure path needs a machine check that does NOT set ssc_bto,
 *     and the seam that decides that -- cpustate.js's onBusFault() -- is outside pcjsvax-e22's
 *     change fence.  The DMA path needs no such seam: a failed qba_map_addr() there is reported
 *     as a residual count, never as an exception.
 *   - The READ half of cqmap_rd()'s out-of-memory branch, for the same reason: SIMH does
 *     cq_serr() + MACH_CHECK(MCHK_READ) with ssc_bto untouched, and this file can latch the
 *     cq_serr() half but cannot suppress busTimeout() from here.  `mapRegRead()` latches SEAR/DSER
 *     and returns null (a bus fault); qdmadiff.js excludes that one case BY NAME and does not
 *     grade it.  The WRITE half (cq_serr() + a deferred mem_err, no exception at all) IS exact,
 *     and IS graded.
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

/* vax_io.c:66-76.  CQDSER_ERR_MASK is CQDSER_ERR == MNX|MPE|TMO|SNX; exc.js carries the same
   three constants for cqMerr()'s half of this register (one register, two latch sites, as in the
   C -- see this file's header).  They are re-derived here rather than imported so that neither
   file's copy can be edited without the other's failing its own differential. */
const CQDSER_SNX      = 0x00000001;
const CQDSER_LST      = 0x00000008;
const CQDSER_ERR_MASK = 0x000000A5;

/* vaxmod_defs.h:196-199, vax_io.c:104-105, :111-113, vax_defs.h:257-268. */
const CQMAPASIZE   = 15;
const CQMAPSIZE    = (1 << CQMAPASIZE);           // 0x8000 -- 8192 longword map entries
const CQMAPAMASK   = (CQMAPSIZE - 1);
const CQMAP_BASE   = (VAX.PHYSMEM.REG_BASE + 0x8000) >>> 0;
const CQMAP_VLD    = (0x80000000 | 0);            // map entry valid
const CQMAP_PAG    = 0x000FFFFF;                  // map entry memory page
const QBMAWIDTH    = 22;
const QBMAMASK     = ((1 << QBMAWIDTH) - 1);      // 0x3FFFFF -- the CQBIC's bus address is 22 bits
const VA_V_VPN     = 9;
const VA_M_OFF     = 0x1FF;                       // 512-byte VAX page

/**
 * @class CQBICVAX
 */
export default class CQBICVAX {
    /**
     * CQBICVAX(exc)
     *
     * @param {Object} exc the owning CPU's VAXExc (cqDser/cqMear live there -- see the file header)
     * @param {Object} [bus] the physical bus.  REQUIRED for the map window and the DMA routines
     *   (the map's entries live in main memory, not in this object); omitted by a caller that
     *   wants only the five local registers, in which case requireBus() below makes every map and
     *   DMA entry point throw.  That guard is not decoration: without it a bus-less instance has
     *   memSize 0, isMem() is false for every address, and every translation would fail through
     *   cqSerr() and report a full residual -- a plausible-looking wrong answer instead of an
     *   error.  qdmadiff.js asserts the throw at startup, so the guard is exercised, not assumed.
     * @param {number} [memSize] MEMSIZE, for ADDR_IS_MEM() (vaxmod_defs.h:220).  Required with bus.
     */
    constructor(exc, bus = null, memSize = 0)
    {
        this.exc = exc;
        this.bus = bus;
        this.memSize = memSize >>> 0;
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

    /* ------------------------------------------------------------------------------------- *
     * The scatter-gather map and the Qbus DMA data path (pcjsvax-e22).  See the file header.  *
     * ------------------------------------------------------------------------------------- */

    /**
     * isMem(pa)
     *
     * ADDR_IS_MEM(x) -- `(x) < MEMSIZE` (vaxmod_defs.h:220).  MEMSIZE is a simulator global in the
     * C and a constructor argument here, because nothing else in this class has any reason to know
     * how much memory the machine has.
     *
     * @this {CQBICVAX}
     * @param {number} pa
     * @returns {boolean}
     */
    isMem(pa) { return (pa >>> 0) < this.memSize; }

    /**
     * requireBus()
     *
     * See the constructor.  Called once per DMA transfer and once per map-register access -- never
     * inside a per-page loop, so it costs nothing where it would matter.
     *
     * @this {CQBICVAX}
     */
    requireBus()
    {
        if (!this.bus) {
            throw new Error("CQBICVAX: the map window and the DMA routines need the bus this " +
                "instance was constructed without");
        }
    }

    /**
     * cqSerr(pa)
     *
     * cq_serr() (vax_io.c:720-726) -- the SLAVE (memory-side) error latch, the counterpart of
     * exc.js's cqMerr()/cq_merr().  It is reached when the map itself is unreachable (MBR points
     * outside memory) or when a VALID map entry points at a page that is not memory.  Distinct
     * from cqMerr() in every observable: DSER<SNX> rather than DSER<MNX>, and the 20-bit SEAR
     * (which lives here) rather than the 13-bit MEAR (which lives on exc).  The lost-error rule
     * is shared, and is written once per latch site exactly as the C writes it.
     *
     * @this {CQBICVAX}
     * @param {number} pa the MEMORY address that failed (not the Qbus address -- cq_merr() takes
     *   the Qbus address; the two error registers latch different things)
     */
    cqSerr(pa)
    {
        if (this.exc.cqDser & CQDSER_ERR_MASK) this.exc.cqDser = (this.exc.cqDser | CQDSER_LST) | 0;
        this.exc.cqDser = (this.exc.cqDser | CQDSER_SNX) | 0;
        this.sear = ((pa >>> VA_V_VPN) & CQSEAR_MASK) | 0;
    }

    /**
     * mapEntryAddr(qa) / mapEntry(qmma)
     *
     * The two halves of qba_map_addr()'s entry fetch, split out as a SEAM the way mmu.js splits
     * out `test()`: `mapEntry()` is the only place the map's backing store is read, so
     * qdmadiff.js's --selfcheck can compose a defect over it (a valid bit that is always set)
     * without substituting its own copy of the translation itself -- HANDOFF.md standing rule 11.
     * Both are used by the shipped path on every single translation; neither exists for the test.
     *
     * @this {CQBICVAX}
     * @param {number} qa Qbus address
     * @returns {number} physical address of the map entry governing qa
     */
    mapEntryAddr(qa) { return (((((qa >>> 0) >>> VA_V_VPN) << 2) & CQMAPAMASK) + (this.mbr >>> 0)) >>> 0; }

    /**
     * @this {CQBICVAX}
     * @param {number} qmma physical address of a map entry, already known to be memory
     * @returns {number} the map entry longword
     */
    mapEntry(qmma) { return this.bus.getLong(qmma & ~3) | 0; }

    /**
     * mapAddr(qa)
     *
     * qba_map_addr() (vax_io.c:666-682) EXACTLY, including which of the two error latches fires
     * for which failure, and including the ADDR_IS_MEM() check on the TRANSLATED address (a valid
     * map entry pointing past the end of memory is a SLAVE NXM, not a success).  The translated
     * address is returned through `this.mapMA` rather than an allocated object because this is
     * called once per 512-byte page of every DMA and allocating there is what HANDOFF.md standing
     * rule 14 is about.
     *
     * @this {CQBICVAX}
     * @param {number} qa Qbus address
     * @returns {boolean} true if `this.mapMA` now holds a usable physical address
     */
    mapAddr(qa)
    {
        qa = qa >>> 0;
        let qmma = this.mapEntryAddr(qa);
        if (this.isMem(qmma)) {                                  /* legit? */
            let qmap = this.mapEntry(qmma);                       /* get map */
            if (qmap & CQMAP_VLD) {                               /* valid? */
                let ma = ((((qmap & CQMAP_PAG) << VA_V_VPN) >>> 0) + (qa & VA_M_OFF)) >>> 0;
                if (this.isMem(ma)) {                             /* legit addr */
                    this.mapMA = ma;
                    return true;
                }
                this.cqSerr(ma);                                  /* slave nxm */
                return false;
            }
            this.exc.cqMerr(qa);                                  /* master nxm */
            return false;
        }
        this.cqSerr(0);                                           /* inv mem */
        return false;
    }

    /**
     * splitLong(dat, buf, j) / joinLong(buf, j) / splitWord(dat, buf, j) / joinWord(buf, j)
     *
     * The byte order of a longword or word as it appears in a DEVICE's buffer.  In the C these are
     * the four `*buf++ = dat & BMASK; ...` / `dat | (*buf++ << 8) | ...` sequences inside the DMA
     * loops, and the word variants are simply what a `uint16 *buf` dereference compiles to on the
     * little-endian hosts this project builds on -- which is also what a VAX device buffer is.
     * Named here so the shipped loops call them, and so --selfcheck can compose a byte-swap over
     * them (standing rule 11) instead of shipping a second, wrong copy of the loops.
     *
     * @this {CQBICVAX}
     */
    splitLong(dat, buf, j)
    {
        buf[j] = dat & 0xFF;
        buf[j + 1] = (dat >>> 8) & 0xFF;
        buf[j + 2] = (dat >>> 16) & 0xFF;
        buf[j + 3] = (dat >>> 24) & 0xFF;
    }

    joinLong(buf, j)
    {
        return (buf[j] | (buf[j + 1] << 8) | (buf[j + 2] << 16) | (buf[j + 3] << 24)) | 0;
    }

    splitWord(dat, buf, j)
    {
        buf[j] = dat & 0xFF;
        buf[j + 1] = (dat >>> 8) & 0xFF;
    }

    joinWord(buf, j)
    {
        return (buf[j] | (buf[j + 1] << 8)) & 0xFFFF;
    }

    /**
     * mapReadB(ba, bc, buf) / mapReadW / mapWriteB / mapWriteW
     *
     * Map_ReadB/Map_ReadW/Map_WriteB/Map_WriteW (vax_io.c:769-895).  "Read" and "Write" are from
     * the DEVICE's point of view, as in the C: a READ moves memory into the device's buffer.
     *
     * THE RETURN VALUE IS A RESIDUAL COUNT -- the number of bytes NOT transferred, 0 on full
     * success (confirmed against vax_io.c:779 `return (bc - i);` and the trailing `return 0;`,
     * and measured through the oracle's SHOW QBA QDMA=).  A device that treats a non-zero return
     * as success is the bug this convention exists to prevent.
     *
     * THE REMAP TRIGGER IS THE PHYSICAL OFFSET, NOT THE BUS OFFSET: the loops re-translate when
     * `(ma & VA_M_OFF) == 0`, with `ma` initialised to 0 so the first iteration always maps.  The
     * two agree only because qba_map_addr() carries the page offset through unchanged; the C is
     * written this way and so is this, rather than "recompute when the bus address crosses a page",
     * which would be a different program that happens to agree today.
     *
     * `buf` is a Uint8Array in every direction, INCLUDING the word variants whose C counterparts
     * take `uint16 *`: the differential compares device buffers byte for byte, which is the only
     * comparison that can see a byte-order defect at all.
     *
     * @this {CQBICVAX}
     * @param {number} ba Qbus bus address
     * @param {number} bc byte count
     * @param {Uint8Array} buf at least bc bytes
     * @returns {number} residual byte count (0 == everything transferred)
     */
    mapReadB(ba, bc, buf)
    {
        this.requireBus();
        ba = (ba & QBMAMASK) >>> 0;
        let ma = 0, j = 0;
        if ((ba | bc) & 3) {                                      /* check alignment */
            for (let i = 0; i < bc; i++, j++) {                    /* by bytes */
                if ((ma & VA_M_OFF) === 0) {                       /* need map? */
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                buf[j] = this.bus.getByte(ma) & 0xFF;
                ma = (ma + 1) >>> 0;
            }
        } else {
            for (let i = 0; i < bc; i += 4, j += 4) {              /* by longwords */
                if ((ma & VA_M_OFF) === 0) {
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                this.splitLong(this.bus.getLong(ma), buf, j);
                ma = (ma + 4) >>> 0;
            }
        }
        return 0;
    }

    /**
     * @this {CQBICVAX}
     * @param {number} ba
     * @param {number} bc
     * @param {Uint8Array} buf
     * @returns {number} residual byte count
     */
    mapReadW(ba, bc, buf)
    {
        this.requireBus();
        ba = (ba & QBMAMASK & ~1) >>> 0;
        bc = bc & ~1;
        let ma = 0, j = 0;
        if ((ba | bc) & 3) {
            for (let i = 0; i < bc; i += 2, j += 2) {              /* by words */
                if ((ma & VA_M_OFF) === 0) {
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                this.splitWord(this.bus.getWord(ma), buf, j);
                ma = (ma + 2) >>> 0;
            }
        } else {
            for (let i = 0; i < bc; i += 4, j += 4) {              /* by longwords */
                if ((ma & VA_M_OFF) === 0) {
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                let dat = this.bus.getLong(ma);
                this.splitWord(dat & 0xFFFF, buf, j);              /* low 16b */
                this.splitWord((dat >>> 16) & 0xFFFF, buf, j + 2); /* high 16b */
                ma = (ma + 4) >>> 0;
            }
        }
        return 0;
    }

    /**
     * @this {CQBICVAX}
     * @param {number} ba
     * @param {number} bc
     * @param {Uint8Array} buf
     * @returns {number} residual byte count
     */
    mapWriteB(ba, bc, buf)
    {
        this.requireBus();
        ba = (ba & QBMAMASK) >>> 0;
        let ma = 0, j = 0;
        if ((ba | bc) & 3) {
            for (let i = 0; i < bc; i++, j++) {
                if ((ma & VA_M_OFF) === 0) {
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                this.bus.setByte(ma, buf[j] & 0xFF);
                ma = (ma + 1) >>> 0;
            }
        } else {
            for (let i = 0; i < bc; i += 4, j += 4) {
                if ((ma & VA_M_OFF) === 0) {
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                this.bus.setLong(ma, this.joinLong(buf, j));
                ma = (ma + 4) >>> 0;
            }
        }
        return 0;
    }

    /**
     * @this {CQBICVAX}
     * @param {number} ba
     * @param {number} bc
     * @param {Uint8Array} buf
     * @returns {number} residual byte count
     */
    mapWriteW(ba, bc, buf)
    {
        this.requireBus();
        ba = (ba & QBMAMASK & ~1) >>> 0;
        bc = bc & ~1;
        let ma = 0, j = 0;
        if ((ba | bc) & 3) {
            for (let i = 0; i < bc; i += 2, j += 2) {
                if ((ma & VA_M_OFF) === 0) {
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                this.bus.setWord(ma, this.joinWord(buf, j));
                ma = (ma + 2) >>> 0;
            }
        } else {
            for (let i = 0; i < bc; i += 4, j += 4) {
                if ((ma & VA_M_OFF) === 0) {
                    if (!this.mapAddr((ba + i) >>> 0)) return bc - i;
                    ma = this.mapMA;
                }
                let dat = (this.joinWord(buf, j) | (this.joinWord(buf, j + 2) << 16)) | 0;
                this.bus.setLong(ma, dat);
                ma = (ma + 4) >>> 0;
            }
        }
        return 0;
    }

    /**
     * mapRegRead(addr)
     *
     * cqmap_rd() (vax_io.c:574-583): a map-register READ is a read of MAIN MEMORY at
     * `(pa & CQMAPAMASK) + cq_mbr`.  There is no register file here at all -- that is the whole
     * mechanism, and it is why programming the map is an ordinary MOVL to REGBASE+0x8000+n*4 and
     * why the DMA path and the CPU path cannot drift apart.
     *
     * `M[ma >> 2]` in the C is a LONGWORD-INDEXED array reference, so the low two bits of `ma` are
     * discarded -- which matters, because a byte or word write to a map register arrives here with
     * those bits SET, and the sub-longword merge below is what places the lane.  Passing the
     * unaligned `ma` straight to the bus (whose getLong()/setLong() do not mask -- mmu.js masks at
     * every one of its own call sites) put every byte-written entry's lanes in the wrong place;
     * measured against the oracle, not reasoned about.
     *
     * The out-of-memory branch is the ONE case this file cannot reproduce exactly: SIMH does
     * cq_serr() + MACH_CHECK(MCHK_READ) leaving ssc_bto untouched, while returning null here
     * reaches cpustate.js's generic register-space fault, which also sets ssc_bto.  The DSER/SEAR
     * latch below is exact; the ssc_bto residue is not, is disclosed in the file header, and
     * qdmadiff.js excludes this one case by name rather than grading it.
     *
     * @this {CQBICVAX}
     * @param {number} addr absolute physical address inside CQMAPBASE..+CQMAPSIZE
     * @returns {?number} the map entry longword, or null for a bus fault
     */
    mapRegRead(addr)
    {
        this.requireBus();
        let ma = (((addr >>> 0) & CQMAPAMASK) + (this.mbr >>> 0)) >>> 0;
        if (this.isMem(ma)) return this.bus.getLong(ma & ~3) | 0;
        this.cqSerr(ma);
        return null;
    }

    /**
     * mapRegWrite(addr, val, lnt)
     *
     * cqmap_wr() (vax_io.c:585-602).  The sub-longword merge is the C's own -- read the backing
     * MEMORY longword, splice the written lane in at `(pa & 3) << 3` -- not this class's
     * readReg()-based merge, which belongs to the five local registers and would fault differently.
     *
     * The out-of-memory branch here IS exact and IS graded: cq_serr() plus a DEFERRED mem_err,
     * with no exception at all, so the store is discarded and the instruction completes.
     *
     * @this {CQBICVAX}
     * @param {number} addr absolute physical address inside CQMAPBASE..+CQMAPSIZE
     * @param {number} val
     * @param {number} lnt 1, 2 or 4
     * @returns {boolean} always true -- a map-register write never faults
     */
    mapRegWrite(addr, val, lnt)
    {
        this.requireBus();
        let ma = (((addr >>> 0) & CQMAPAMASK) + (this.mbr >>> 0)) >>> 0;
        if (this.isMem(ma)) {
            if (lnt < 4) {
                let sc = (addr & 3) << 3;
                let mask = (lnt === 2) ? 0xFFFF : 0xFF;
                let t = this.bus.getLong(ma & ~3);
                val = (((val & mask) << sc) | (t & ~(mask << sc))) | 0;
            }
            this.bus.setLong(ma & ~3, val | 0);
        } else {
            this.cqSerr(ma);
            this.exc.memErr = 1;
        }
        return true;
    }
}

/**
 * @class CQMAPVAX
 *
 * The regblock.js sub-device for CQMAPBASE..+CQMAPSIZE (vax_sysdev.c:1002's regtable entry
 * `{ CQMAPBASE, CQMAPBASE+CQMAPSIZE, &cqmap_rd, &cqmap_wr }`).  It owns no state: every access is
 * forwarded to the CQBICVAX that owns MBR, because the map's entries are main memory and MBR is
 * what says where.  Same read/write shape as SSCVAX/NVRVAX/CQBICVAX (regblock.js's contract:
 * a read returns null for "not decoded", a write returns false for the same).
 */
export class CQMAPVAX {
    /**
     * @param {CQBICVAX} cqbic
     */
    constructor(cqbic)
    {
        this.cqbic = cqbic;
    }

    /**
     * ReadReg() hands the read routine the whole longword and lets vax_mmu.h's Read() do the
     * shifting, so readWord()/readByte() shift here for exactly the same reason cqbic.js's do.
     *
     * @this {CQMAPVAX}
     */
    readLong(addr) { return this.cqbic.mapRegRead(addr); }

    readWord(addr)
    {
        let v = this.cqbic.mapRegRead(addr);
        if (v === null) return null;
        return (v >>> ((addr & 2) ? 16 : 0)) & 0xFFFF;
    }

    readByte(addr)
    {
        let v = this.cqbic.mapRegRead(addr);
        if (v === null) return null;
        return (v >>> ((addr & 3) << 3)) & 0xFF;
    }

    /**
     * @this {CQMAPVAX}
     */
    writeLong(addr, val) { return this.cqbic.mapRegWrite(addr, val | 0, 4); }
    writeWord(addr, val) { return this.cqbic.mapRegWrite(addr, val & 0xFFFF, 2); }
    writeByte(addr, val) { return this.cqbic.mapRegWrite(addr, val & 0xFF, 1); }
}

export { CQSCR_POK, CQSCR_MASK, CQDSER_MASK, CQMEAR_MASK, CQSEAR_MASK, CQMBR_MASK,
         CQMAP_BASE, CQMAPSIZE, CQMAP_VLD, CQMAP_PAG, QBMAMASK, VA_M_OFF };
