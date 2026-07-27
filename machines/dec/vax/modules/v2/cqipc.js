/**
 * @fileoverview Implements the KA655 CQBIC interprocessor-communication register (the Qbus doorbell)
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
 * WHAT THIS IS -- AND WHY IT IS THE ROM'S `?91`
 * ============================================================================
 * pcjsvax-b8a.  `cq_ipc` (vax_io.c:122) and the FOUR C entry points that touch it:
 *
 *      cqipc_rd()/cqipc_wr()  (vax_io.c:533-552)  reached through vax_sysdev.c:1009's regtable
 *                                                 entry { CQIPCBASE, CQIPCBASE+CQIPCSIZE }, i.e.
 *                                                 REGBASE+0x1F40 -- LOCAL REGISTER space.
 *      dbl_rd()/dbl_wr()      (vax_io.c:556-566)  the QBA device's own DIB, autoconfigured into the
 *                                                 Qbus I/O PAGE -- a different address, the SAME
 *                                                 register.  dbl_wr() is a one-line forward to
 *                                                 cqipc_wr(), and this file is written the same way.
 *
 * SIMH's own comment above cqipc_rd() is the scope statement: "IPC can be read as local register or
 * as Qbus I/O.  Because of the W1C."  One state model, two address paths -- exactly the shape
 * console.js already carries for RXCS/RXDB/TXCS/TXDB (IPR path plus SSC mirror), and for the same
 * reason: two register models would drift, and the W1C bit is precisely what makes a drift
 * observable.  CQIPCVAX below owns the state and IS the local-register mount; DBLVAX is the I/O-page
 * mount and owns no state at all.
 *
 * THE MEASUREMENT THAT PUT THIS ITEM HERE, and the wrong answer it replaced: pcjsvax-bfb attributed
 * the ROM's `?91` self-test failure to the undecoded CMCTL registers, because that is where bus
 * faults were SEEN.  That was false.  Four boots of the same ROM, same 3,000,000-instruction budget,
 * reading the console byte stream (tests/conoutdiff.js):
 *
 *      neither decoded                     40.. then `?91 2 03 EF 04 0000`
 *      CMCTL decoded, doorbell not         40.. then `?91 ...`      BYTE-IDENTICAL -- no change
 *      doorbell decoded, CMCTL not         40..39..38.. then `?33`  (`?33` IS CMCTL)
 *      both decoded                        40..39..38..…31.. then `?53`
 *
 * The ROM reads the Qbus I/O-page IPCR FIFTY instructions BEFORE it reads CMCTL.  cmctl.js's header
 * carries the same table from the other side; neither file may be edited to claim sufficiency alone.
 *
 * ============================================================================
 * THE TWO ADDRESS PATHS ARE NOT THE SAME DEVICE MODEL -- MEASURED, NOT REASONED
 * ============================================================================
 * A local register is dispatched by vax_sysdev.c's ReadReg()/WriteReg() at LONGWORD granularity: the
 * handler is handed the whole longword and vax_mmu.h's ReadB()/ReadW() extract the lane.  The Qbus
 * I/O page is dispatched by vax_io.c's ReadIO()/WriteIO() at WORD granularity -- `iodispR[(pa &
 * IOPAGEMASK) >> 1]` -- and the QBA's DIB is `IOLN_DBL == 2`, ONE word slot.  So an aligned LONGWORD
 * reference is two Qbus cycles, the second of which lands on nothing.
 *
 * MEASURED on the live oracle (16MB, MAPEN 0, SCB_MCHK and SCB_MEMERR pointed at a page of NOPs):
 *
 *      MOVL  @#20081F40,R0     ->  R0 = cq_ipc & CQIPC_MASK, no exception
 *      MOVL  @#20001F40,R0     ->  MACHINE CHECK, BTO = 00000000, DSER = 80, MEAR = 000F
 *      MOVL  #FFFFFFFF,@#20081F40  ->  cq_ipc = 0161, no exception
 *      MOVL  #FFFFFFFF,@#20001F40  ->  cq_ipc = 0161, DSER = 80, MEAR = 000F, and a DEFERRED
 *                                      MEMERR interrupt delivered at the next instruction boundary
 *
 * -- i.e. the I/O-page longword WRITE still lands its low word on the doorbell and then reports the
 * high word as an unbacked Qbus reference, while the I/O-page longword READ machine-checks outright.
 * Both fall out of the existing bus for free and neither is special-cased here; see DBLVAX below for
 * exactly which mmu.js seam does which, and tests/dbldiff.js for both graded through real
 * instructions.
 *
 * ============================================================================
 * SCOPE -- THIS IS A QBUS I/O-PAGE BEACHHEAD AND THE WINDOW IS TWO BYTES
 * ============================================================================
 * Nothing in this tree decoded the Qbus I/O page before this file; the whole IOPAGE range sat in
 * BusVAX.RESERVED as reserved-but-undecoded.  bus.js's addIoPage() installs ONE controller over it,
 * and the sub-device this file owns is DBLVAX, over CQIPCSIZE == 2 bytes.  Every other I/O-page
 * address keeps falling through to MemoryVAX.readNone()/writeNone() -- the identical path it took
 * before this file existed, which is vax_io.c's ReadQb()/WriteQb() unbacked case (cq_merr() plus,
 * for reads only, a machine check that never touches ssc_bto; cpustate.js's onBusFault() routes it
 * by address).  tests/dbldiff.js PROBES that window on both engines rather than asserting it, and
 * tests/cqmerrdiff.js -- which grades exactly the unbacked Qbus mechanism -- must stay green.
 *
 * SINCE pcjsvax-c2c THERE IS A SECOND WINDOW on this page: modules/v2/rq.js's RQDX3 controller at
 * 0x20001468..0x2000146B, built on this same seam and mounted the same way.  It is a separate
 * device with separate state; what the two share is the ONE controller addIoPage() installs and the
 * fall-through that leaves the other 8186 bytes faulting.  tests/dbldiff.js's PHASE W is the
 * assertion that the decoded set is exactly those two windows and nothing else, and it derives that
 * set from a list rather than from a constant so a third window cannot slip in unnamed.
 *
 * WHAT THIS FILE DOES *NOT* USE, stated because it looks like it should: regblock.js's `REG_MCHK`
 * (pcjsvax-622).  That answer exists for a handler that IS reached and raises MACH_CHECK() itself
 * without the SSC bus-timeout bits -- cmctl_rd()'s register 18.  Neither cqipc_rd()/cqipc_wr() nor
 * dbl_rd()/dbl_wr() has such a branch; they always answer.  The machine checks around this device
 * are the ORDINARY unbacked ones, and on the Qbus side those already leave ssc_bto alone through an
 * older and different mechanism -- cpustate.js's onBusFault() routing by ADDRESS (VAX.isQbusAddr(),
 * pcjsvax-d22) -- so returning null is both sufficient and exact here.  pcjsvax-622's seam IS reused,
 * as the DISPATCHER: bus.js's addIoPage() installs regblock.js's makeRegController() unchanged.  It
 * is not rebuilt, and it is not needed as a fault channel.
 *
 * ============================================================================
 * THE I/O-PAGE ADDRESS IS AUTOCONFIGURED, SO IT IS VERIFIED AGAINST THE ORACLE, NOT ASSERTED
 * ============================================================================
 * `CQIPCBASE` is a compile-time constant in vaxmod_defs.h:201 and DBL_OFFSET below is derived from
 * it.  The I/O-page address is NOT: vax_io.c:155's `DIB qba_dib = { IOBA_AUTO, IOLN_DBL, ... }`
 * leaves it to pdp11_io_lib.c's autoconfiguration.  That it lands at the SAME 0x1F40 offset is a
 * property of the Qbus fixed-address map (the IPCR is Qbus 17777500(8), and 17777500 - 17760000 =
 * 017500 = 0x1F40), not of REGBASE -- so writing one in terms of the other would be a coincidence
 * dressed as a derivation.  tests/dbldiff.js re-reads `SHOW QBA IOSPACE` from the LIVE oracle on
 * every run and FAILS if the QBA row's base or length disagrees with the two constants below.
 */

import { VAX } from "./defines.js";

/** vaxmod_defs.h:200-201.  CQIPCBASE == REGBASE + 0x1F40, CQIPCSIZE == 2 bytes. */
const CQIPC_OFFSET = 0x1F40;
const CQIPC_SIZE   = 2;
const CQIPC_BASE   = (VAX.PHYSMEM.REG_BASE + CQIPC_OFFSET) >>> 0;

/** The QBA DIB's autoconfigured I/O-page address -- see the file header; VERIFIED against the live
    oracle by tests/dbldiff.js, never assumed to follow from CQIPC_BASE. */
const DBL_OFFSET = 0x1F40;
const DBL_BASE   = (VAX.PHYSMEM.IOPAGE_BASE + DBL_OFFSET) >>> 0;
const DBL_SIZE   = CQIPC_SIZE;                  // vax_io.c:153, IOLN_DBL == 002

/* vax_io.c:92-100.  Every bit SIMH marks "NI" (not implemented) is still modelled as STORAGE,
   because the firmware can read it back -- that is what "not implemented" means there: no device
   behaviour hangs off it, not that the bit does not exist. */
const CQIPC_QME  = 0x00008000;                  // Qbus read NXM, write-1-to-clear
const CQIPC_INV  = 0x00004000;                  // CAM invalidate, write-only, NOT in CQIPC_MASK
const CQIPC_AHLT = 0x00000100;                  // aux halt
const CQIPC_DBIE = 0x00000040;                  // doorbell interrupt enable
const CQIPC_LME  = 0x00000020;                  // local memory enable
const CQIPC_DB   = 0x00000001;                  // doorbell request
const CQIPC_W1C  = CQIPC_QME;
const CQIPC_RW   = (CQIPC_AHLT | CQIPC_DBIE | CQIPC_LME | CQIPC_DB);
const CQIPC_MASK = (CQIPC_RW | CQIPC_QME);

/**
 * @class CQIPCVAX
 *
 * The register state, AND the local-register mount at CQIPCBASE (regblock.js sub-device contract:
 * a read returns null for "not decoded", a write returns false for the same).
 */
export default class CQIPCVAX {
    /**
     * CQIPCVAX()
     *
     * No constructor arguments: unlike CQBICVAX this register shares no state with VAXExc.  `cq_ipc`
     * is a module-scope C global that only the four routines named in the file header touch -- plus
     * ONE cross-register side effect, cqbic_wr()'s DSER<SME> clear, which cqbic.js reaches through
     * clearQme() below rather than by keeping a second copy of the bit.
     */
    constructor()
    {
        this.reset();
    }

    /**
     * reset()
     *
     * vax_io.c:755, qba_reset(): `cq_dser = cq_mear = cq_sear = cq_ipc = 0;`.  `cq_ipc` is also a
     * zero-initialised C global, so this is its value from the first instruction of any run as well
     * as after a reset -- the same reasoning cmctl.js's and nvr.js's reset() comments give.
     *
     * @this {CQIPCVAX}
     */
    reset()
    {
        this.ipc = 0;
    }

    /**
     * rd()
     *
     * cqipc_rd() (vax_io.c:535-537) and dbl_rd() (vax_io.c:556-560) are the SAME expression --
     * `cq_ipc & CQIPC_MASK` -- which is why both mounts below call this one method.  Note that
     * CQIPC_INV is deliberately absent from CQIPC_MASK in the C: it is write-only on real hardware
     * and reads back as zero.
     *
     * @this {CQIPCVAX}
     * @returns {number}
     */
    rd()
    {
        return (this.ipc & CQIPCVAX.CQIPC_MASK) | 0;
    }

    /**
     * wr(addr, val, lnt)
     *
     * cqipc_wr() (vax_io.c:539-552), whole.  THREE things in it are easy to get subtly wrong, and
     * each has its own --selfcheck mutation in tests/dbldiff.js:
     *
     *   1. The W1C step tests the SHIFTED value (`nval`), so a BYTE write of 0x80 to the ODD byte of
     *      the register clears QME -- the only way a byte-wide access can reach a bit in the high
     *      lane at all.
     *   2. The read/write step tests `(pa & 3) == 0` -- SIMH's own comment calls it "low byte only".
     *      An access that does not start at the register's byte 0 performs the W1C and NOTHING ELSE.
     *   3. That step uses the RIGHT-JUSTIFIED `val`, not `nval`.  For lnt < L_LONG the two differ
     *      whenever the access is not at lane 0, and at lane 0 they are equal -- so the distinction
     *      is invisible in the only case the gate lets through, and copying `nval` there would be a
     *      defect no test that only exercised lane 0 could see.  It is written as the C writes it.
     *
     * MEASURED on the live oracle, one instruction each, cq_ipc preloaded through `DEPOSIT QBA IPC`:
     *
     *      cq_ipc=0000  MOVB #FF,@#20001F40   -> 0061      (RW bits of 0xFF; QME untouched)
     *      cq_ipc=0000  MOVB #FF,@#20001F41   -> 0000      (W1C only; no RW update)
     *      cq_ipc=8000  MOVB #80,@#20001F41   -> 0000      (W1C via the odd lane)
     *      cq_ipc=8000  MOVB #FF,@#20001F40   -> 8061      (RW update; QME NOT cleared by lane 0)
     *      cq_ipc=8000  MOVW #8161,@#20001F40 -> 0161      (W1C and RW in one word access)
     *
     * @this {CQIPCVAX}
     * @param {number} addr absolute physical address of the access (either mount)
     * @param {number} val RIGHT-JUSTIFIED, the convention every device in this tree uses
     * @param {number} lnt 1, 2 or 4
     * @returns {boolean} always true -- cqipc_wr() has no failing branch
     */
    wr(addr, val, lnt)
    {
        let nval = val | 0;
        if (lnt < CQIPCVAX.L_LONG) {
            let sc = ((addr >>> 0) & 3) << 3;
            nval = (val << sc) | 0;
        }
        this.ipc = (this.ipc & ~(nval & CQIPCVAX.CQIPC_W1C)) | 0;
        if ((((addr >>> 0) & 3) & CQIPCVAX.LOW_BYTE_GATE) === 0) {
            this.ipc = (((this.ipc & ~CQIPCVAX.CQIPC_RW) | (val & CQIPCVAX.CQIPC_RW)) & CQIPCVAX.CQIPC_MASK) | 0;
        }
        return true;
    }

    /**
     * clearQme()
     *
     * cqbic_wr()'s DSER case: `if (val & CQDSER_SME) cq_ipc = cq_ipc & ~CQIPC_QME;` (vax_io.c:515-
     * 516).  It lives HERE, as a method on the register that owns the bit, so cqbic.js can reach it
     * without carrying a second copy of CQIPC_QME -- the same "one register model, one owner" rule
     * cqbic.js already follows for DSER/MEAR (which it reads and writes through VAXExc).
     *
     * cqbic.js's header used to disclose this side effect as NOT reproduced, for want of a cq_ipc to
     * reproduce it on, and said "if a LATER boundary shows the ROM ... touching CQIPC directly, that
     * is the item that must add them."  This is that item.  tests/dbldiff.js grades it through real
     * instructions in both directions (SME set, and SME clear), so it is not untestable dead code.
     *
     * @this {CQIPCVAX}
     */
    clearQme()
    {
        this.ipc = (this.ipc & ~CQIPCVAX.CQIPC_QME) | 0;
    }

    /* ------------------------------------------------------------------------------------- *
     * The LOCAL-REGISTER mount (regblock.js sub-device at CQIPCBASE).  vax_sysdev.c's ReadReg()/  *
     * WriteReg() hand the handler the whole longword, so all three sizes go through rd()/wr()     *
     * and the lane extraction is the CALLER's -- cmctl.js and cqbic.js are written the same way.  *
     * ------------------------------------------------------------------------------------- */

    /**
     * @this {CQIPCVAX}
     * @param {number} addr
     * @returns {number}
     */
    readLong(addr) { return this.rd(); }

    readWord(addr) { return (this.rd() >>> ((addr & 2) ? 16 : 0)) & 0xFFFF; }

    readByte(addr) { return (this.rd() >>> ((addr & 3) << 3)) & 0xFF; }

    /**
     * @this {CQIPCVAX}
     * @param {number} addr
     * @param {number} val
     * @returns {boolean}
     */
    writeLong(addr, val) { return this.wr(addr, val | 0, 4); }

    writeWord(addr, val) { return this.wr(addr, val & 0xFFFF, 2); }

    writeByte(addr, val) { return this.wr(addr, val & 0xFF, 1); }
}

/* cqipc_wr()'s own constants, published as class data so tests/dbldiff.js's --selfcheck can PERTURB
   the shipped computation rather than substitute a copy of it (HANDOFF.md standing rule 11).
   LOW_BYTE_GATE is the `& 3` of SIMH's `(pa & 3) == 0`, kept separately so a mutation can drop the
   gate without also changing which lane `nval` is shifted into. */
CQIPCVAX.L_LONG        = 4;
CQIPCVAX.CQIPC_W1C     = CQIPC_W1C;
CQIPCVAX.CQIPC_RW      = CQIPC_RW;
CQIPCVAX.CQIPC_MASK    = CQIPC_MASK;
CQIPCVAX.CQIPC_QME     = CQIPC_QME;
CQIPCVAX.LOW_BYTE_GATE = 3;

/**
 * @class DBLVAX
 *
 * The QBUS I/O-PAGE mount -- vax_io.c's dbl_rd()/dbl_wr(), the QBA's own DIB.  It owns no state:
 * every access forwards to the CQIPCVAX that owns `cq_ipc`, exactly as dbl_wr() forwards to
 * cqipc_wr().  Same regblock.js sub-device contract as CQMAPVAX (cqbic.js): null / false mean "this
 * device does not answer for that reference", and the controller then takes the caller's own
 * unbacked path -- which for an I/O-page address is the Qbus one (cq_merr, no ssc_bto).
 *
 * WHY THE SIZES ARE NOT SYMMETRIC WITH CQIPCVAX'S, and where the split actually lives:
 *
 *   readByte/readWord   The Qbus is a WORD bus: ReadIO() positions the word with `<< ((pa & 2) ?
 *                       16 : 0)` and ReadB()/ReadW() then extract with the SAME shift, so the two
 *                       cancel and the lane within the answer is `pa & 1`, not `pa & 3`.  Writing
 *                       it as `pa & 1` is not a simplification of the aligned case -- it is the
 *                       word granularity, and it is what makes this mount textually different from
 *                       a register-space one.
 *   readLong            NULL.  ReadIO(pa, L_LONG) is `ReadQb(pa)` then `ReadQb(pa+2)`, and the
 *                       second word slot is not the doorbell -- IOLN_DBL is 2.  SIMH's MACH_CHECK
 *                       is a longjmp, so no value is ever combined and the ONLY observable is one
 *                       cq_merr() at an address in the same 512-byte page as this one (MEAR latches
 *                       the page).  Returning null reproduces that exactly, through the identical
 *                       fault path an entirely undecoded I/O-page longword read takes.  mmu.js's
 *                       readL() does NOT split Qbus longword reads -- correctly, because in the C
 *                       the split is INSIDE ReadIO() -- so this method IS reached by real
 *                       instructions and is graded by tests/dbldiff.js.
 *   writeLong           FALSE, and it performs no write.  mmu.js's writeL() ALREADY splits an
 *                       aligned Qbus longword write into two setWord() cycles (see its doc comment,
 *                       pcjsvax-d22), so the low word reaches writeWord() below and the high word
 *                       lands on nothing -- which is WriteIO(pa, val, L_LONG) exactly, deferred
 *                       mem_err included, and is what tests/dbldiff.js grades.  No CPU path reaches
 *                       this method; it returns false rather than silently accepting, so that a
 *                       caller that bypasses writeL() gets a fault it can see instead of a write
 *                       whose high half vanished.
 */
export class DBLVAX {
    /**
     * @param {CQIPCVAX} cqipc
     */
    constructor(cqipc)
    {
        this.cqipc = cqipc;
    }

    /**
     * @this {DBLVAX}
     * @param {number} addr
     * @returns {?number}
     */
    readLong(addr) { return null; }

    readWord(addr) { return this.cqipc.rd() & 0xFFFF; }

    readByte(addr) { return (this.cqipc.rd() >>> ((addr & 1) << 3)) & 0xFF; }

    /**
     * dbl_wr() (vax_io.c:562-566): `cqipc_wr (addr, data, (access == WRITEB) ? L_BYTE : L_WORD)`.
     *
     * @this {DBLVAX}
     * @param {number} addr
     * @param {number} val
     * @returns {boolean}
     */
    writeLong(addr, val) { return false; }

    writeWord(addr, val) { return this.cqipc.wr(addr, val & 0xFFFF, 2); }

    writeByte(addr, val) { return this.cqipc.wr(addr, val & 0xFF, 1); }
}

export {
    CQIPC_BASE, CQIPC_SIZE, CQIPC_OFFSET, DBL_BASE, DBL_SIZE, DBL_OFFSET,
    CQIPC_QME, CQIPC_INV, CQIPC_AHLT, CQIPC_DBIE, CQIPC_LME, CQIPC_DB,
    CQIPC_W1C, CQIPC_RW, CQIPC_MASK
};
