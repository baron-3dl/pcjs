/**
 * @fileoverview Implements the KA655 console: the CVAX on-chip TTI/TTO UART (IPR path) and its
 *               SSC memory-mapped mirror, plus the off-chip IPRs adjacent to it in ReadIPR/WriteIPR
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
 * pcjsvax-bfb.  THE MILESTONE'S OWN SPEC IS WRONG about this being a DZ/2681 UART -- see the item's
 * own text.  The real KA655 console is FOUR CVAX on-chip IPRs -- MT.RXCS=32, MT.RXDB=33, MT.TXCS=34,
 * MT.TXDB=35 (vax_defs.h:603-614) -- implemented in vax_stddev.c's rxcs_rd/wr, rxdb_rd, txcs_rd/wr,
 * txdb_wr (:275-335), reached through exc.js's ReadIPR/WriteIPR off-chip dispatch (setIPRDevice()).
 * vax_sysdev.c's ssc_rd()/ssc_wr() ALSO expose RXCS/RXDB/TXCS(read)/RXCS/TXCS/TXDB(write) at
 * SSC-relative registers 0x20/0x21/0x22/0x23 (physical SSCBASE+0x80/+0x84/+0x88/+0x8C) by calling
 * the IDENTICAL rxcs_rd()/rxdb_rd()/txcs_rd() functions -- so this class models the register state
 * ONCE (rxcs/rxBuf/txcs/txBuf below) and exposes it through TWO call surfaces: `read(prn)`/
 * `write(prn,val)` for exc.js's setIPRDevice() seam, and `sscRead(rg)`/`sscWrite(rg,val)` for
 * ssc.js's SSCVAX to call for the four SSC-mirror offsets -- see ssc.js's own doc comment on how it
 * wires this in.  Never implement the read/write LOGIC twice; only the two thin dispatchers above
 * are duplicated, and they are pure address-to-register-number translation, nothing else.
 *
 * ============================================================================
 * TXCS DONE TIMING -- THE MOST LIKELY WAY THIS HANGS, AND HOW IT IS MODELED
 * ============================================================================
 * vax_stddev.c's txdb_wr() clears CSR_DONE and calls `sim_activate (&tto_unit, tto_unit.wait)`
 * (`tto_unit.wait` = SERIAL_OUT_WAIT = 100, sim_defs.h:460) -- an event scheduled 100 "sim_interval"
 * units later, at which point tto_svc() sets CSR_DONE back (and raises TTO's interrupt if CSR_IE).
 * If TXCS never reports CSR_DONE again, the ROM's output routine (poll TXCS, branch back if DONE is
 * clear) spins forever and prints NOTHING -- a hang, not a crash; the item's own text names this as
 * the single most likely failure mode, and it is checked FIRST by this file's own --selfcheck
 * ("txcs-done-stuck-clear") in tests/consoledif.js.
 *
 * This project's own cycle accounting (cpustate.js's stepCPU(), `nCycles = 1 + (extraBytes >> 5)`)
 * is DELIBERATELY kept in exact lockstep with SIMH's own `sim_interval` charge -- see that file's
 * doc comment: "preserved exactly... because it is observable in any cycle comparison against
 * SIMH."  `cpu.nTotalCycles` is therefore the SAME "how many sim_interval units have elapsed" clock
 * SIMH's event queue measures a scheduled delay against, so a plain deadline compare against it
 * reproduces `sim_activate`'s relative-delay semantics without needing SIMH's own generic event
 * queue (which nothing else in this machine has built, and which this item's scope does not need
 * beyond this one relative delay): `txdbWr()` below records `txDoneAt = cpu.nTotalCycles +
 * SERIAL_OUT_WAIT`; `maybeComplete()` fires the deferred CSR_DONE/interrupt the first time it is
 * consulted at or after that cycle count.
 *
 * `maybeComplete()` is consulted from TWO places, matching real hardware's TWO ways a program can
 * observe or be told the completion happened:
 *   - `txcsRd()` -- a POLLING program's own read of TXCS naturally lands after tick() has already
 *     run for that instruction (see below), so by the time the read's OWN body executes, pending
 *     completions are already reflected -- txcsRd() does not need to re-check itself, but does so
 *     defensively (idempotent, costs nothing once already fired) in case it is ever called from
 *     somewhere that skipped tick(), e.g. this file's own --selfcheck harness.
 *   - `tick(cpu)` -- exc.js's setIRQL() (the ONE per-instruction hook this item adds there, see its
 *     own doc comment) calls `this.iprDevice.tick(cpu)` at the top of EVERY instruction, before
 *     `evalInt()` arbitrates -- this is what lets an INTERRUPT-ENABLED completion be delivered even
 *     if the running program is not polling TXCS at all (an ordinary VAX interrupt is asynchronous
 *     to the instruction stream it interrupts; a purely read-triggered completion would not be).
 *
 * ============================================================================
 * INTERRUPTS
 * ============================================================================
 * TTI (SCB 0xF8, bit 8) and TTO (SCB 0xFC, bit 9), both at absolute IPL 0x14 -- vax_defs.h:397-398,
 * IPL_TTI/IPL_TTO = 0x14 (vaxmod_defs.h:415-416), matching hwintdiff.js's OWN REAL_DEVICES table
 * (`{name:"TTI", lvl:0x14, bit:8, ...}`, `{name:"TTO", lvl:0x14, bit:9, ...}`) -- the bit numbers
 * here are not invented, they are the SAME ones that table already grades same-level acknowledge
 * ordering for (pcjsvax-f67).  Installed via exc.js's addInterruptSource() ONCE, at construction
 * (this class is always constructed with the owning CPU's `exc`); raiseInterrupt()/clearInterrupt()
 * are called on every edge vax_stddev.c's rxcs_wr()/txcs_wr()/rxdb_rd()/txdb_wr()/tto_svc()-
 * equivalent transition calls SET_INT/CLR_INT for -- see rxcsWr()/txcsWr()/rxdbRd()/txdbWr()/
 * maybeComplete() below, each with the vax_stddev.c line it reproduces.
 *
 * ============================================================================
 * MT.TODR -- see the doc comment on TODR_SENTINEL below (unchanged from this item's earlier,
 * pre-console commit; kept in this same file because setIPRDevice() only ever installs ONE device
 * for every off-chip IPR, TODR included).
 * ============================================================================
 */

import { MT, IPL_HMIN } from "./exc.js";

/* Generic UART status bits, vaxmod_defs.h (CSR_DONE/CSR_IE are architected at fixed offsets shared
   by every VAX CSR-shaped device; CSR_ERR is not decoded -- see the file header, RXDB never reports
   an error). TTICSR_IMP/TTOCSR_IMP/RW, vax_stddev.c:82-89. */
const CSR_DONE = 0x0080;
const CSR_IE   = 0x0040;
const TTICSR_IMP = (CSR_DONE | CSR_IE);
const TTOCSR_IMP = (CSR_DONE | CSR_IE);
const TTICSR_RW  = CSR_IE;
const TTOCSR_RW  = CSR_IE;

/* SCB vectors and hardware request bits -- vax_defs.h:397-398, matching hwintdiff.js's own table
   exactly (see the file header). */
const SCB_TTI = 0xF8, SCB_TTO = 0xFC;
const TTI_BIT = 8, TTO_BIT = 9;

/* sim_defs.h:460, SERIAL_OUT_WAIT -- see the file header's TXCS DONE TIMING section. */
const SERIAL_OUT_WAIT = 100;

/* ssc_rd()/ssc_wr()'s register-index constants for the SSC mirror, vax_sysdev.c:1264-1458 (see
   ssc.js's own doc comment for how these are reached). */
const SSC_RG_RXCS = 0x20, SSC_RG_RXDB = 0x21, SSC_RG_TXCS = 0x22, SSC_RG_TXDB = 0x23;

/*
 * MT.TODR's CC-preserving sentinel -- romdiff's boundary-advance hit `MFPR #1B,...` (MT.TODR = 27)
 * BEFORE ever reaching the console registers; exc.js's setIPRDevice() is the ONLY installer for
 * every off-chip IPR, not just this item's four, so getting the ROM past that instruction requires
 * SOMETHING here to answer TODR too.
 *
 * vax_stddev.c's todr_rd() has a ROM-diagnostics special case: `if ((fault_PC & 0xFFFE0000) ==
 * 0x20040000) return todr_reg;` -- while executing from the ROM (which is ALWAYS true for this
 * item), it returns the raw counter with NO live wall-clock re-sampling.  But `todr_reg` itself was
 * already seeded, once, at `reset`/`boot cpu` time by `todr_resync()`'s "not attached" branch --
 * `(seconds since local midnight)*100 + 0x10000000 + centiseconds` -- REAL WALL-CLOCK TIME captured
 * the instant SIMH's own process reset.  Because `captureSimhTrace()` runs SIMH as a SEPARATE
 * process invocation from this machine's own JS run, there is no wall-clock instant the two could
 * ever agree on -- a GENUINELY non-reproducible value, the same class HANDOFF.md's §7 already
 * documents for EHKAA's timer-dependent counts.
 *
 * What DOES matter, and is what this models: the CC bits an MFPR of TODR sets (an ordinary MOVL-
 * shaped result -- N from the sign bit, Z from being zero, V cleared, C preserved).  SIMH's
 * wall-clock formula is `secondsSinceMidnight*100 + 0x10000000 + centiseconds`; secondsSinceMidnight
 * maxes out at 86400, so the whole expression is bounded by roughly 0x10000000 + 8,640,000 + 99,
 * comfortably under 0x20000000 -- bit 31 is NEVER set and the value is NEVER zero, for any wall-clock
 * time whatsoever.  `read(MT.TODR)` below reproduces exactly that shape (a fixed, non-wall-clock
 * sentinel deliberately picked with N=0, Z=0, matching what SIMH's formula produces on EVERY
 * possible real invocation) so the CC bits this record sets AGREE with the oracle's on every run,
 * even though the raw 32-bit VALUE itself does not and is not required to: the ROM stores it into
 * NVR-range memory (a MemoryVAX.TYPE.CONTROLLER destination), which simhtrace.js's captureResult()
 * ALREADY marks resultUnreliable and excludes from the per-instruction VALUE comparison for exactly
 * this reason: a device register's stored value is graded by its OWN dedicated oracle, never by
 * this generic reconstruction.  TODR has no such dedicated oracle in this item (it is not one of
 * the four registers this item owns) and is not graded value-for-value anywhere.
 *
 * A REAL time-of-day clock is explicitly NOT modeled here and belongs to whichever future item owns
 * the SSC/CLK timer hardware; this file's TODR handling exists ONLY to keep the ROM's boot-entry
 * trace from diverging on an IPR this item does not own.
 */
const TODR_SENTINEL = 0x10000000;

export default class ConsoleVAX {
    /**
     * ConsoleVAX(exc)
     *
     * @param {Object} exc the owning CPU's VAXExc -- addInterruptSource()/raiseInterrupt()/
     *   clearInterrupt() are called through it (see the file header), and `exc.cpu.nTotalCycles`
     *   is this file's TXCS-completion clock (see maybeComplete()'s doc comment).  Required (unlike
     *   ssc.js's/cqbic.js's optional `exc`): a console with no CPU behind it cannot time anything.
     */
    constructor(exc)
    {
        this.exc = exc;
        this.exc.addInterruptSource(IPL_HMIN, TTI_BIT, SCB_TTI);
        this.exc.addInterruptSource(IPL_HMIN, TTO_BIT, SCB_TTO);
        /* Output captured for done condition 1 (the banner text) -- an array of bytes, never
           inspected or special-cased by ANY code path above txdbWr(): see the file header and the
           item's own "HOW AN AGENT WOULD CHEAT THIS" section.  A harness reads `.output` AFTER the
           run to check the banner; nothing here ever compares against expected text. */
        this.output = [];
        this.reset();
    }

    /**
     * reset()
     *
     * vax_stddev.c tti_reset()/tto_reset() (:373-378, :420-427): TTI starts with no input pending
     * and IE clear; TTO starts with CSR_DONE ALREADY SET (ready to accept the first character) and
     * any pending completion event cancelled (`sim_cancel(&tto_unit)`).
     *
     * @this {ConsoleVAX}
     */
    reset()
    {
        this.rxcs = 0;
        this.rxBuf = 0;
        this.txcs = CSR_DONE | 0;
        this.txBuf = 0;
        this.txDoneAt = null;                   // vax_stddev.c tto_reset(): sim_cancel(&tto_unit)
        this.exc.clearInterrupt(IPL_HMIN, TTI_BIT);
        this.exc.clearInterrupt(IPL_HMIN, TTO_BIT);
    }

    /* ----------------------------------------------------------------------------------------- *
     * The register model -- called from BOTH read(prn)/write(prn,val) (the IPR path) and             *
     * sscRead(rg)/sscWrite(rg,val) (the SSC mirror).  Every vax_stddev.c line reproduced is cited.    *
     * ----------------------------------------------------------------------------------------- */

    /** vax_stddev.c:275-278, rxcs_rd(). @this {ConsoleVAX} @returns {number} */
    rxcsRd() { return this.rxcs & TTICSR_IMP; }

    /**
     * vax_stddev.c:280-291, rxdb_rd().  Clears CSR_DONE and CLR_INT(TTI) on a genuine read of
     * pending data -- `sim_activate_after_abs` (schedule the next poll) is SIMH's own input-polling
     * machinery, not modeled: this item has no keyboard poller, only injectChar() (below), which is
     * the harness's equivalent of "a character arrived."
     *
     * @this {ConsoleVAX}
     * @returns {number}
     */
    rxdbRd()
    {
        let v = this.rxBuf & 0xFF;
        if (this.rxcs & CSR_DONE) {
            this.rxcs = (this.rxcs & ~CSR_DONE) | 0;
            this.exc.clearInterrupt(IPL_HMIN, TTI_BIT);
        }
        return v;
    }

    /** vax_stddev.c:293-296, txcs_rd().  See maybeComplete()'s doc comment for why this does not
        itself need `cpu` -- tick() has already run for this instruction by the time any read
        reaches here (exc.js's setIRQL() ordering) -- the call here is a defensive no-op once fired.
        @this {ConsoleVAX} @returns {number} */
    txcsRd() { this.maybeComplete(); return this.txcs & TTOCSR_IMP; }

    /**
     * vax_stddev.c:308-316, rxcs_wr().  Edge-triggered: disabling IE always clears the request;
     * ENABLING IE while data is ALREADY pending (DONE set, IE was clear) raises it immediately --
     * the same "was already true, now unmasked" edge txcs_wr() below reproduces for output.
     *
     * @this {ConsoleVAX}
     * @param {number} val
     */
    rxcsWr(val)
    {
        if ((val & CSR_IE) === 0) {
            this.exc.clearInterrupt(IPL_HMIN, TTI_BIT);
        } else if ((this.rxcs & (CSR_DONE | CSR_IE)) === CSR_DONE) {
            this.exc.raiseInterrupt(IPL_HMIN, TTI_BIT);
        }
        this.rxcs = ((this.rxcs & ~TTICSR_RW) | (val & TTICSR_RW)) | 0;
    }

    /** vax_stddev.c:318-326, txcs_wr() -- same edge-triggered shape as rxcsWr(), for TTO.
        @this {ConsoleVAX} @param {number} val */
    txcsWr(val)
    {
        if ((val & CSR_IE) === 0) {
            this.exc.clearInterrupt(IPL_HMIN, TTO_BIT);
        } else if ((this.txcs & (CSR_DONE | CSR_IE)) === CSR_DONE) {
            this.exc.raiseInterrupt(IPL_HMIN, TTO_BIT);
        }
        this.txcs = ((this.txcs & ~TTOCSR_RW) | (val & TTOCSR_RW)) | 0;
    }

    /**
     * vax_stddev.c:328-335, txdb_wr().  Clears CSR_DONE, clears TTO's request, and schedules the
     * deferred completion -- see the file header's TXCS DONE TIMING section for `txDoneAt`.  The
     * character is appended to `this.output` HERE, unconditionally, the instant the write happens
     * -- exactly when real hardware/SIMH would (`tto_unit.buf = data`) -- never gated on, inspected
     * by, or compared against any expected text: see the file header.
     *
     * @this {ConsoleVAX}
     * @param {number} val
     */
    txdbWr(val)
    {
        this.txBuf = val & 0xFF;
        this.output.push(this.txBuf);
        this.txcs = (this.txcs & ~CSR_DONE) | 0;
        this.exc.clearInterrupt(IPL_HMIN, TTO_BIT);
        this.txDoneAt = (this.exc.cpu.nTotalCycles >>> 0) + SERIAL_OUT_WAIT;
    }

    /**
     * maybeComplete()
     *
     * vax_stddev.c:404-419, tto_svc() -- the deferred half of a TXDB write: sets CSR_DONE and, if
     * CSR_IE, raises TTO's interrupt request.  See the file header for why `cpu.nTotalCycles` is
     * the right clock to compare `txDoneAt` against and why this needs to run both from a direct
     * poll (txcsRd()) and from tick() (exc.js's per-instruction hook).
     *
     * @this {ConsoleVAX}
     */
    maybeComplete()
    {
        if (this.txDoneAt === null) return;
        /*
         * `<=`, not `<` -- MEASURED against a real SIMH interrupt-delivery round trip
         * (tests/consoledif.js's INTERRUPT phase): a naive `nTotalCycles >= txDoneAt` fires one
         * whole instruction EARLIER than SIMH's own event queue does (delivered at PC=R_HANDLER+2
         * instead of the oracle's R_HANDLER+1, for the identical instruction-count budget on both
         * sides).  `sim_activate(uptr, wait)`'s countdown reaches zero only AFTER `wait` FURTHER
         * instructions have completed past the one that scheduled it, not merely `wait` instructions
         * later inclusive of the scheduling one -- this `<=` is the one-cycle correction that makes
         * `cpu.nTotalCycles`'s delta match that fencepost exactly, confirmed by the same test.
         */
        if ((this.exc.cpu.nTotalCycles >>> 0) <= (this.txDoneAt >>> 0)) return;
        this.txDoneAt = null;
        this.txcs = (this.txcs | CSR_DONE) | 0;
        if (this.txcs & CSR_IE) this.exc.raiseInterrupt(IPL_HMIN, TTO_BIT);
    }

    /**
     * tick(cpu)
     *
     * exc.js's setIRQL() per-instruction hook (see that file's doc comment).  `cpu` is accepted for
     * symmetry with that call site but not used -- `this.exc.cpu` is the same object (VAXExc's own
     * constructor sets it) and every OTHER method here already reads the clock through `this.exc`,
     * so using it here too keeps one source of truth rather than trusting two references to agree.
     *
     * @this {ConsoleVAX}
     * @param {Object} cpu
     */
    tick(cpu) { this.maybeComplete(); }

    /**
     * injectChar(byte)
     *
     * The harness's (or a future keyboard device's) equivalent of "a character arrived" -- vax_
     * stddev.c's tti_svc() finding real console input, minus the polling: sets RXDB/RXCS exactly as
     * that function does when it succeeds (:343-365), including raising TTI's interrupt if CSR_IE.
     * NOT part of the ROM's own boot-entry trace (nothing injects input during a boot-to-banner
     * run); this is done condition 3's own entry point, exercised by tests/consoledif.js directly.
     *
     * @this {ConsoleVAX}
     * @param {number} byte
     */
    injectChar(byte)
    {
        this.rxBuf = byte & 0xFF;
        this.rxcs = (this.rxcs | CSR_DONE) | 0;
        if (this.rxcs & CSR_IE) this.exc.raiseInterrupt(IPL_HMIN, TTI_BIT);
    }

    /* ----------------------------------------------------------------------------------------- *
     * exc.js's setIPRDevice() seam: read(prn)/write(prn,val).                                        *
     * ----------------------------------------------------------------------------------------- */

    /**
     * read(prn)
     *
     * vax_sysdev.c's ReadIPR() (:845-913): MT_RXCS/MT_RXDB/MT_TXCS call the SAME functions
     * sscRead() below calls; MT_TXDB reads as a PLAIN 0 (`case MT_TXDB: val = 0;` -- no `txdb_rd()`
     * exists, output is write-only, matching real hardware).  Every other off-chip register number
     * this device does not decode -- including every one already handled elsewhere in this machine
     * (SID is exc.js's own, not this device's) -- falls through to 0, exactly what exc.js's
     * readIPR() did BEFORE any iprDevice was installed, so installing this device changes nothing
     * for a register neither it nor exc.js decodes.
     *
     * @this {ConsoleVAX}
     * @param {number} prn
     * @returns {number}
     */
    read(prn)
    {
        switch (prn) {
        case MT.RXCS: return this.rxcsRd();
        case MT.RXDB: return this.rxdbRd();
        case MT.TXCS: return this.txcsRd();
        case MT.TXDB: return 0;                 // vax_sysdev.c ReadIPR() case MT_TXDB: val = 0
        case MT.TODR: return TODR_SENTINEL | 0;
        }
        return 0;
    }

    /**
     * write(prn, val)
     *
     * vax_sysdev.c's WriteIPR() (:916-975): MT_RXCS/MT_TXCS/MT_TXDB call the SAME functions
     * sscWrite() below calls; MT_RXDB is a NO-OP (`case MT_RXDB: break;` -- no `rxdb_wr()` exists,
     * input is read-only from software, matching real hardware).
     *
     * @this {ConsoleVAX}
     * @param {number} prn
     * @param {number} val
     */
    write(prn, val)
    {
        switch (prn) {
        case MT.RXCS: this.rxcsWr(val); return;
        case MT.RXDB: return;                   // vax_sysdev.c WriteIPR() case MT_RXDB: break
        case MT.TXCS: this.txcsWr(val); return;
        case MT.TXDB: this.txdbWr(val); return;
        }
    }

    /* ----------------------------------------------------------------------------------------- *
     * ssc.js's SSC-mirror seam: sscRead(rg)/sscWrite(rg,val) -- rg is ALREADY the SSC register       *
     * index (`(pa - SSCBASE) >> 2`), computed by ssc.js's own readReg()/writeReg() exactly as         *
     * SSCVAX's other cases are.                                                                      *
     * ----------------------------------------------------------------------------------------- */

    /**
     * sscRead(rg)
     *
     * vax_sysdev.c's ssc_rd() (:1264-1345): cases 0x20/0x21/0x22 call rxcs_rd()/rxdb_rd()/
     * txcs_rd() -- the IDENTICAL functions read() above calls.  rg 0x23 (TXDB) has NO CASE AT ALL
     * in ssc_rd()'s switch (only ssc_wr() lists it) -- vax_sysdev.c's implicit `return 0;` at the
     * end of the switch applies, so a mirror READ of TXDB is a silent 0, never routed to this
     * device and never a fault (see ssc.js's own file header on why the WHOLE SSC span never
     * machine-checks); ssc.js itself returns that 0 directly for rg 0x23 rather than calling here,
     * which is why this function has no case for it -- being called with rg=0x23 would be a bug in
     * the caller, not a case this function is expected to handle.
     *
     * @this {ConsoleVAX}
     * @param {number} rg
     * @returns {?number} null if `rg` is not one of the three this function decodes
     */
    sscRead(rg)
    {
        switch (rg) {
        case SSC_RG_RXCS: return this.rxcsRd();
        case SSC_RG_RXDB: return this.rxdbRd();
        case SSC_RG_TXCS: return this.txcsRd();
        }
        return null;
    }

    /**
     * sscWrite(rg, val)
     *
     * vax_sysdev.c's ssc_wr() (:1352-1458): cases 0x20/0x22/0x23 call rxcs_wr()/txcs_wr()/
     * txdb_wr() -- the IDENTICAL functions write() above calls.  rg 0x21 (RXDB) has NO CASE in
     * ssc_wr()'s switch either -- a mirror WRITE to RXDB is a silent no-op, matching WriteIPR()'s
     * own `case MT_RXDB: break;`; ssc.js handles that directly (return true, do nothing) for the
     * same reason sscRead() does not handle rg 0x23.
     *
     * @this {ConsoleVAX}
     * @param {number} rg
     * @param {number} val
     * @returns {boolean} false if `rg` is not one of the three this function decodes
     */
    sscWrite(rg, val)
    {
        switch (rg) {
        case SSC_RG_RXCS: this.rxcsWr(val); return true;
        case SSC_RG_TXCS: this.txcsWr(val); return true;
        case SSC_RG_TXDB: this.txdbWr(val); return true;
        }
        return false;
    }
}

export { SSC_RG_RXCS, SSC_RG_RXDB, SSC_RG_TXCS, SSC_RG_TXDB, SCB_TTI, SCB_TTO, TTI_BIT, TTO_BIT, CSR_DONE, CSR_IE };
