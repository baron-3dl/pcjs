/**
 * @fileoverview Implements the CVAX on-chip interval timer and time-of-day register (ICCS/TODR)
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
 * ============================================================================
 * WHAT THIS IS
 * ============================================================================
 * pcjsvax-954.  This is the ONLY IPR device MTPR/MFPR reaches on this chip: ICCS (MT_ICCS=24) and
 * TODR (MT_TODR=27), vax_stddev.c's iccs_rd/iccs_wr/todr_rd/todr_wr and clk_svc.  NICR (25) and ICR
 * (26) are architecturally-named IPR numbers this SIMH model does NOT wire to anything -- ReadIPR/
 * WriteIPR (vax_sysdev.c:845-988) has no `case MT_NICR`/`case MT_ICR` at all, so both fall to the
 * SAME `default:` every other unowned off-chip register falls to.  A port that invents reload
 * storage for them is inventing a feature the real KA655 does not have; see read()/write() below.
 *
 * It is a direct port of:
 *
 *   vax_stddev.c   iccs_rd/iccs_wr (270-273, 298-306), clk_svc (455-469), todr_rd/todr_wr
 *                  (471-533)
 *   vax_sysdev.c   ReadIPR/WriteIPR's MT_ICCS/MT_TODR cases (851-853, 887-890, 925-927, 929-931)
 *                  -- everything else in that switch is a DIFFERENT device (console UART, CADR/
 *                  MSER, IORESET) and stays exc.js's problem, not this file's.
 *
 * ============================================================================
 * THE ROM-DETECTION SPECIAL CASE (todr_rd, vax_stddev.c:476)
 * ============================================================================
 *     if ((fault_PC & 0xFFFE0000) == 0x20040000) return todr_reg;
 *
 * While the CPU is executing an instruction whose PC lies in the PRIMARY ROM window (NOT its
 * mirror -- see ROM_MASK below), TODR reads back the RAW COUNTED register, not a computed
 * wall-clock value.  This is deliberate: ROM diagnostics need a self-consistent, monotonically
 * counting TODR, and get one; VMS and everything else gets a real battery-clock reading.  Getting
 * this backwards (always wall-clock) is exactly the mutation --selfcheck injects (see
 * tests/timerdiff.js) and exactly the "gotcha" pcjsvax-954 names.
 *
 * MASK PRECISION, MEASURED NOT ASSUMED: 0xFFFE0000 masks to a 128KB (ROM_SIZE) window, and
 * ROM_BASE (0x20040000) & 0xFFFE0000 === 0x20040000, but ROM's own MIRROR (ROM_BASE+ROM_SIZE =
 * 0x20060000) & 0xFFFE0000 === 0x20060000 -- a DIFFERENT masked value.  So code executing from the
 * mirror half is NOT treated as "ROM" by this check on real SIMH, even though bus.js's
 * makeRomAliasController() makes the mirror read identical CONTENT.  ROM_MASK below is computed
 * from VAX.PHYSMEM.ROM_SIZE (never hand-transcribed as a literal 0xFFFE0000), and tests/
 * timerdiff.js's selfcheck ("rom_mask_includes_mirror") widens the check to the whole mirrored
 * span and confirms that mutation is CAUGHT, so this precision is actually graded, not incidental.
 *
 * ============================================================================
 * THE WALL-CLOCK PATH (todr_rd/todr_wr OUTSIDE rom), AND WHY IT IS NOT THE TIMING MODEL DECISION
 * ============================================================================
 * pcjsvax-954's own text directs a DETERMINISTIC, INSTRUCTION-COUNT-DRIVEN model for the thing
 * that recurs on a schedule: clk_svc, the 100Hz service that increments todr_reg and requests the
 * interrupt (see tick() below -- gated by INSTRS_PER_TICK, not real time, for exactly the reason
 * HANDOFF.md 7 records EHKAA's own dispatch counts as non-reproducible).
 *
 * todr_rd()/todr_wr() OUTSIDE rom are a DIFFERENT mechanism: a SYNCHRONOUS computation against the
 * REAL HOST CLOCK at the instant of the MFPR/MTPR, with no recurring service involved at all --
 * exactly like a battery-backed clock chip a real OS reads by asking "what time is it right now".
 * Porting that literally means using the real host clock (Date.now()) here too; refusing to
 * (e.g. always returning todrReg, i.e. treating every context as ROM) is a DIFFERENT, real bug --
 * see tests/timerdiff.js's file header for how the differential grades this without requiring two
 * independent OS processes to observe the same instant, which they structurally cannot.
 *
 * ============================================================================
 * POWER-ON RESYNC (todr_resync, vax_stddev.c:537-566) -- MEASURED CORRECTION, veracity re-dispatch
 * ============================================================================
 * clk_reset() (570-588) calls todr_resync() the FIRST time a process's clk_unit.filebuf is NULL --
 * i.e. on every fresh SIMH process's implicit startup reset, not just an explicit `reset all` typed
 * later.  The earlier version of this file asserted, as a measured fact, that "a fresh process's
 * clock is STOPPED until something writes it" (todrReg=0, todrBlow=1) -- that is true of the C
 * globals' STATIC INITIALIZERS and FALSE of the running simulator: measured directly, a bare
 * `reset all` with NO prior write leaves the real oracle's TODR non-zero and BLOW clear (e.g.
 * TODR=0x7A8EB365, BLOW=0), because resync() already ran.  resync()'s "not attached" branch (the
 * VMS-default form, the only one this tree ever exercises -- nothing here uses clk_unit ATTACH) is:
 *
 *     base = ((tm_yday*24 + tm_hour)*60 + tm_min)*60 + tm_sec       -- seconds since Jan 1, local time
 *     todr_wr((base*100) + 0x10000000 + round(tv_nsec/1e7))
 *
 * reset() below reproduces this using Date's local-time getters (tm_yday reconstructed the same way
 * a real localtime() would report it).  This value can NEVER be bit-matched against a live oracle
 * (both engines compute it from REAL wall-clock time at slightly different instants in slightly
 * different processes -- see tests/timerdiff.js's file header on why cross-process wall-clock
 * equality is structurally impossible, and its power-on cases for what IS graded: BLOW=0, the value
 * is >= 0x10000000 and non-zero, and the ROM/non-ROM readback shapes are self-consistent on each
 * engine independently).  Leaving this unmodeled was undisclosed, not merely deferred: with no
 * device this item's own boundary is a mere 30 instructions from the ROM's OWN first bare TODR
 * read (measured: DBG(33), `MFPR #1B,34(R1)` at PC=0x200401E9) -- the moment a later item's boundary
 * reaches it, an unmodeled power-on state diverges there for real, not hypothetically.
 *
 * ============================================================================
 * WHAT IS DELIBERATELY NOT HERE
 * ============================================================================
 * - SSC T0/T1 (the programmable timers at SSC_BASE+0x100+) -- a DIFFERENT facility, owned by the
 *   countdown item per pcjsvax-954's own constraint.  Not one register here overlaps that scope.
 *   THEY ARE NOT INDEPENDENT, THOUGH, and pcjsvax-a6f is what measured it: the KA655 ROM's
 *   self-test 53 cross-calibrates ssc.js's microsecond-counting TIR against THIS file's TODR, so
 *   the RATIO between the two is observable to software even though no register is.  Both are now
 *   expressed in terms of one time base -- USECS_PER_INSTR below -- and ssc.js asserts the
 *   calibration against INSTRS_PER_TICK at load time, so the two cannot drift apart silently again.
 * - The console UART (CSRS/CSRD/CSTS/CSTD/RXCS/RXDB/TXCS/TXDB) and CADR/MSER/IORESET -- other
 *   MT_* numbers ReadIPR/WriteIPR dispatch to; read()/write() below pass them straight through to
 *   the SAME VALUE (0 / dropped write) exc.js already produced for ALL of IPR_DEVICE before any
 *   device existed.  NOT the same in every respect, though -- see the next paragraph.
 * - ssc_bto (the SSC bus-timeout register) on a NICR/ICR access -- MEASURED, not assumed identical:
 *   vax_sysdev.c's ReadIPR/WriteIPR `default:` case (the one NICR/ICR fall into) ALSO sets
 *   SSCBTO_BTO (`examine sysd bto` goes 0x00000000 -> 0x80000000 across a real MFPR NICR).  exc.js's
 *   OWN default (no device installed) does not set this bit either -- a pre-existing, documented
 *   gap in exc.js itself (see its readIPR()/writeIPR() default-case comments), which this item's
 *   constraint (do not touch exc.js) leaves this file unable to close.  read()/write() below
 *   therefore reproduce exc.js's PRE-EXISTING value-level behavior for NICR/ICR (0 / dropped) but
 *   NOT its side-effect-level behavior (no bto bit) -- tests/timerdiff.js measures and reports this
 *   divergence explicitly rather than silently declaring the passthrough fully "inert".
 * - `sim_rtcn_tick_ack` on an ICCS ack write (data & CSR_DONE) -- MEASURED CORRECTION: the earlier
 *   version of this note called it "a host-timer-calibration hint with no architectural state,
 *   nothing reads it back", which is not accurate.  It calls sim_rtcn_tick_catchup_check(), and
 *   with sim_catchup_ticks defaulting TRUE that can schedule EXTRA clk_svc invocations -- extra
 *   SET_INT(CLK) and extra todr_reg increments the CPU genuinely observes on real hardware.  The
 *   omission is still correct under pcjsvax-954's directed deterministic, instruction-count-driven
 *   model (there is no "catch-up" concept once ticks are not wall-clock scheduled at all -- see the
 *   file header above), but the ORIGINAL justification for it was wrong; this is the true one.
 * - TODR's "battery low" / attach-a-TOY-file persistence mode (clk_unit attach/detach, the
 *   OS-Agnostic mode) -- no test in this tree runs offline across process restarts; `todrBlow`
 *   exists so todr_wr(0)'s "stop the clock" state has somewhere to live and reads back inert, and
 *   so resync()'s todr_wr() call (which always writes a non-zero value) clears it exactly once.
 */

import { VAX } from "./defines.js";

const MT_ICCS = 24, MT_TODR = 27;

/* vaxmod_defs.h:269-274 (CSR_V_IE=6): the ONLY bit ICCS actually implements on this model. */
const CSR_V_IE = 6;
const CSR_IE = 1 << CSR_V_IE;
const CLKCSR_IMP = CSR_IE;                       // vax_stddev.c:90, iccs_rd()'s readback mask
const CLKCSR_RW = CSR_IE;                        // vax_stddev.c:91, iccs_wr()'s writable mask

/* vaxmod_defs.h:406/437/340, SCB_INTTIM (vax_defs.h:392): CLK's hardware interrupt identity. */
const IPL_HMIN = 0x14;
const IPL_CLK_ABS = 0x16;                        // absolute IPL; exc.js's addInterruptSource/
                                                  // raiseInterrupt/clearInterrupt take this form
const INT_V_CLK = 0;
const SCB_INTTIM = 0xC0;

/* vax_stddev.c:476's ROM-detection mask, COMPUTED from VAX.PHYSMEM.ROM_SIZE rather than
   transcribed as the literal 0xFFFE0000 -- see the file header's "MASK PRECISION" note for why
   this must NOT also match the ROM mirror. */
const ROM_BASE = VAX.PHYSMEM.ROM_BASE >>> 0;
const ROM_MASK = (~(VAX.PHYSMEM.ROM_SIZE - 1)) >>> 0;
const ROM_TAG = (ROM_BASE & ROM_MASK) >>> 0;

/**
 * TODR_TICKS_PER_SEC -- clk_svc's rate, and the unit todr_reg counts in.
 *
 * vax_stddev.c's todr_resync() writes `(base*100) + ...` where `base` is a SECOND count (see the
 * POWER-ON RESYNC section above, and todrResync() below, which uses this same constant), and
 * todr_rd() reconstructs a centisecond count from elapsed real time.  So one clk_svc service --
 * one `todr_reg + 1` -- is one CENTISECOND, i.e. 10 ms, i.e. the 100 Hz this file's header names.
 */
const TODR_TICKS_PER_SEC = 100;

/**
 * USECS_PER_INSTR -- THE TIME BASE OF THIS WHOLE ENGINE, and it is a MEASURED quantity, not a
 * convenience.
 *
 * vax_sysdev.c:1476-1480 states it in its own words: "Various ROM activities, including testing
 * the Interval Timers, presume that ROM based code execute instructions at 1 instruction per
 * usec."  tmr_sched() (vax_sysdev.c:1608-1625) then ENCODES that equivalence literally -- for a
 * short delay requested from ROM it calls `sim_activate(&sysd_unit[tmr], usecs_sched)`, whose base
 * unit is INSTRUCTIONS, passing a MICROSECOND count unconverted.  One instruction, one microsecond.
 *
 * GRADED AGAINST THE LIVE ORACLE, so this is not a reading of a comment: tests/tmrdiff.js's
 * `run_mode_counting_rom_t0` case runs a T0 interval from the ROM address window on BOTH engines
 * and `checkExact`s the TIR readback -- real SIMH advances the SSC timer's microsecond-counting
 * TIR by exactly 1 per instruction retired there, and ssc.js does the same (see its
 * TIR_USECS_PER_INSTR, which is the same constant seen from the timer side, and the
 * cross-calibration assertion ssc.js makes against INSTRS_PER_TICK below).
 *
 * WHAT THIS REPLACED, AND WHY (pcjsvax-a6f).  INSTRS_PER_TICK was 200, chosen so that "a
 * differential's step budgets (hundreds to a few thousand instructions) exercise several ticks
 * without needing a huge run", and its comment asserted that "there is no meaningful 'instructions
 * per 10ms' conversion once the model is instruction-count driven".  THAT SENTENCE IS FALSE, and
 * the ROM is what falsifies it: KA655 self-test 53 cross-calibrates the SSC T1 interval register
 * against TODR, so the ratio between this file's tick and ssc.js's TIR count is a quantity the
 * ROM MEASURES.  At 200 instructions per tick the implied time base was 50 usec per instruction,
 * TODR ran 50x fast against a microsecond-counting TIR, and the ROM measured 2,002 microseconds
 * where 100,000 were required -- failing test 53 with subtest 09 ("too slow") on every run
 * (MEASURED, this machine, 2026-07-30: `?53 2 09 FF 00 0000  P1=00000002 P2=00000028
 * P3=80017ECA`, where P3 is the signed magnitude of measured-minus-100,000 usec).
 */
const USECS_PER_INSTR = 1;

/** One clk_svc service in microseconds: 1,000,000 / 100 = 10,000 usec = 10 ms. */
const TICK_USECS = 1000000 / TODR_TICKS_PER_SEC;

/**
 * INSTRS_PER_TICK -- the deterministic stand-in for clk_svc's real-time ~10ms/100Hz schedule
 * (vax_stddev.c UNIT clk_unit's CLK_DELAY, sim_rtcn_calb-recalibrated every service), now DERIVED
 * from the time base above rather than chosen (HANDOFF.md standing rule 5).  10,000 usec per tick
 * at 1 usec per instruction is 10,000 instructions per tick.
 *
 * What has NOT changed is the property DONE CONDITION 2 asks for and which is the reason this is a
 * fixed instruction count at all rather than a real-time schedule: it is a constant, so the same
 * run twice produces the same tick count at the same instruction counts, with no host-speed
 * dependency anywhere.  This engine can therefore be deterministic exactly where real SIMH -- whose
 * clk_svc is wall-clock calibrated -- cannot be, which is why the ROM's test 53 passes here on
 * every run and only sometimes on the oracle (HANDOFF.md 5).
 *
 * A differential that wants "several ticks" must now budget tens of thousands of instructions;
 * tests/timerdiff.js derives its own step counts from this constant rather than hard-coding them,
 * so raising it scales those runs instead of silently emptying them.
 */
const INSTRS_PER_TICK = TICK_USECS / USECS_PER_INSTR;

/**
 * TOY_MAX_SECS (vax_stddev.c:487, `#define TOY_MAX_SECS (0x40000000/25)`) -- computed, not
 * transcribed, so it stays tied to the source constant.  todr_rd()'s overflow branch (491-494,
 * see todrRd() below) zeroes the register once the reconstructed elapsed seconds reach this --
 * TOY_MAX_SECS*100 = 0xFFFFFFA0 centiseconds, a PURE FUNCTION of the centiseconds-per-second
 * constant and therefore a near-zero-tolerance cross-engine RATE probe (see tests/timerdiff.js's
 * caseTodrOverflowBoundary()) that a millisecond-scaled implementation could never even reach
 * within a 32-bit register, so a value 100x too large everywhere else would silently make it pass,
 * not fail -- documented explicitly on that case, not left implicit.  MEASURED: the mathematically
 * EXACT boundary value (0xFFFFFFA0) is itself a real-time race on live SIMH (five identical probes
 * split roughly evenly between 0 and unchanged) -- caseTodrOverflowBoundary() therefore grades
 * 0xFFFFFFA1 (one centisecond further in, empirically stable) as its "past threshold" value rather
 * than 0xFFFFFFA0 itself; see that case's own doc comment for the measurement.
 */
const TOY_MAX_SECS = Math.floor(0x40000000 / 25);

/**
 * dayOfYear(date)
 *
 * `date`'s tm_yday equivalent (days since Jan 1, 0-based) via LOCAL CALENDAR components -- NOT
 * `(date - jan1) / 86400000`, which is wrong across a DST transition (see resync()'s doc comment
 * for the measured divergence).  A plain days-per-month table, leap-year-aware; immune to DST by
 * construction because it never subtracts two absolute instants.
 *
 * @param {Date} date
 * @returns {number}
 */
function dayOfYear(date)
{
    const DAYS_IN_MONTH = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    let y = date.getFullYear();
    let leap = (y % 4 === 0 && (y % 100 !== 0 || y % 400 === 0));
    let doy = date.getDate() - 1;
    for (let m = 0; m < date.getMonth(); m++) {
        doy += DAYS_IN_MONTH[m] + ((m === 1 && leap) ? 1 : 0);
    }
    return doy;
}

/**
 * @class ClkVAX
 */
export default class ClkVAX {
    /**
     * ClkVAX(exc)
     *
     * @param {Object} [exc] a VAXExc instance (or anything exposing raiseInterrupt/clearInterrupt)
     *   -- OPTIONAL at construction because setIPRDevice(dev)'s write(prn,val) contract (exc.js)
     *   passes no `cpu`/`exc` argument, so this device must already know where to signal an
     *   interrupt before iccsWr()/tick() can be called.  bind(exc) sets it after the fact if it
     *   is not known yet at construction time (mirrors SSCVAX's constructor-only API, extended
     *   because this device, unlike SSCVAX, must reach OUTSIDE itself to request an interrupt).
     */
    constructor(exc)
    {
        this.exc = exc || null;
        this.reset();
    }

    /**
     * bind(exc)
     *
     * @this {ClkVAX}
     * @param {Object} exc
     */
    bind(exc) { this.exc = exc; }

    /**
     * reset()
     *
     * vax_stddev.c clk_reset() (570): clk_csr=0 and CLR_INT(CLK) unconditionally.  todr_reg/
     * todrBlow are DELIBERATELY NOT re-touched on a SECOND reset() call (matching ssc.js's
     * SSCVAX.reset() precedent for ssc_base: a real KA655's TODR is battery-backed and does not
     * zero on a plain CPU reset) -- resync() below runs ONLY on the first-ever call, mirroring
     * clk_reset()'s own `if (clk_unit.filebuf == NULL)` guard (570-588), which is true exactly
     * once per real SIMH process.  This class's reset() only runs from the constructor in every
     * test in this tree (one fresh instance per machine, exactly like SSCVAX), so "instance-
     * construction default" and "post-reset value" are the same question here -- see ssc.js's own
     * reset() doc comment for why that would stop being harmless the moment something wires this
     * to a reset handler that could fire MORE than once on a live instance.
     *
     * @this {ClkVAX}
     */
    reset()
    {
        this.csr = 0;
        if (this.exc) this.exc.clearInterrupt(IPL_CLK_ABS, INT_V_CLK);
        if (this.todrReg === undefined) {
            this.wallBaseMs = 0;
            this.todrReg = 0;
            this.todrBlow = 1;
            this.resync();                       // see the file header's "POWER-ON RESYNC" section
        }
        this._instrsSinceTick = 0;
        this.tickCount = 0;
    }

    /**
     * resync()
     *
     * vax_stddev.c todr_resync() (537-566), "not attached" branch ONLY -- the OS-Agnostic/attached
     * branch requires a TOY file this tree never creates, so it is not reachable and not ported
     * (see the file header's "WHAT IS DELIBERATELY NOT HERE").  Reproduces:
     *
     *     base = ((tm_yday*24 + tm_hour)*60 + tm_min)*60 + tm_sec
     *     todr_wr((base*100) + 0x10000000 + round(tv_nsec/1e7))
     *
     * using Date's LOCAL-time getters (matching localtime()'s locality) and reconstructing
     * tm_yday (days since Jan 1, 0-based) the same way, since JS's Date has no direct equivalent.
     * Routed through todrWr() itself -- not a hand-inlined assignment -- so the wall-clock anchor
     * (wallBaseMs) ends up in EXACTLY the state a real MTPR of this same value would have left it
     * in, which is what makes an immediate post-resync non-ROM read round-trip correctly (see
     * tests/timerdiff.js's power-on cases).
     *
     * MEASURED CORRECTION (veracity re-dispatch, round 2): an earlier version computed tm_yday as
     * `floor((now - midnightJan1) / 86400000)` -- a RAW MILLISECOND division, which is wrong the
     * instant a DST transition falls between Jan 1 and `now`: a spring-forward day is 23 real
     * hours, not 24, so dividing by a flat 86,400,000 undercounts by one for every local instant
     * from 00:00 to 00:59 on each such day.  Measured: at 2026-03-09 00:30 America/New_York (a
     * spring-forward-affected date), the ms-division form gave yday=66 where C's localtime()-based
     * tm_yday is 67 -- an 8,640,000-centisecond (24-hour) divergence invisible on this host (UTC
     * has no DST) but live the moment this runs anywhere DST applies.  dayOfYear() below instead
     * walks the LOCAL CALENDAR (year/month/date getters plus a leap-year-aware days-per-month
     * table), which is what tm_yday actually is -- a count of calendar days, not a duration.
     *
     * @this {ClkVAX}
     */
    resync()
    {
        let now = new Date();
        let ydays = dayOfYear(now);
        let base = ((ydays * 24 + now.getHours()) * 60 + now.getMinutes()) * 60 + now.getSeconds();
        let frac = Math.round(now.getMilliseconds() / 10);
        this.todrWr(((base * TODR_TICKS_PER_SEC) + 0x10000000 + frac) | 0);
    }

    /**
     * read(prn) / write(prn, val)
     *
     * The setIPRDevice(dev) contract exc.js's readIPR()/writeIPR() install against.  Anything
     * that is not ICCS or TODR (including NICR/ICR -- see the file header) passes straight through
     * to the SAME VALUE (0 / dropped write) those numbers already got with no device installed at
     * all -- but NOT the same SIDE EFFECT: real SIMH also sets ssc_bto here, which this file does
     * NOT (see the file header's "ssc_bto" paragraph for the measured divergence and why this
     * item's own constraint leaves it unclosed).
     *
     * @this {ClkVAX}
     * @param {number} prn
     * @returns {number}
     */
    read(prn)
    {
        if (prn === MT_ICCS) return this.iccsRd();
        if (prn === MT_TODR) return this.todrRd();
        return 0;
    }

    /**
     * @this {ClkVAX}
     * @param {number} prn
     * @param {number} val
     */
    write(prn, val)
    {
        if (prn === MT_ICCS) { this.iccsWr(val); return; }
        if (prn === MT_TODR) { this.todrWr(val); return; }
        /* SSC default for every other off-chip register (NICR, ICR, and everything this file does
           not own): drop the write.  Real SIMH ALSO sets ssc_bto here (measured: MFPR/MTPR NICR
           flips `examine sysd bto` 0 -> 0x80000000) -- exc.js does not model that for ANY off-chip
           register yet (a pre-existing, documented gap in exc.js itself, which this item's own
           constraint -- do not touch exc.js -- leaves unclosed here too).  tests/timerdiff.js
           measures and reports this divergence explicitly; see the file header. */
    }

    /**
     * iccsRd()
     *
     * vax_stddev.c:270-273: `return clk_csr & CLKCSR_IMP` -- CLKCSR_IMP is CSR_IE ALONE, so the
     * interrupt-PENDING state (int_req[CLK]) is deliberately invisible here; it is read back
     * through exc.js's own interrupt-request state (already verified sound), not through this
     * register at all.
     *
     * @this {ClkVAX}
     * @returns {number}
     */
    iccsRd() { return this.csr & CLKCSR_IMP; }

    /**
     * iccsWr(data)
     *
     * vax_stddev.c:298-306.  Clearing IE clears the pending request OUTRIGHT (300-301) -- this is
     * the one path, besides deviceVector()'s own acknowledge-time clear, that can withdraw a CLK
     * request before it is ever taken.  `data & CSR_DONE` (sim_rtcn_tick_ack) is a host-timer
     * recalibration hint with no architectural state; deliberately omitted (see file header).
     *
     * @this {ClkVAX}
     * @param {number} data
     */
    iccsWr(data)
    {
        if ((data & CSR_IE) === 0 && this.exc) this.exc.clearInterrupt(IPL_CLK_ABS, INT_V_CLK);
        this.csr = (this.csr & ~CLKCSR_RW) | (data & CLKCSR_RW);
    }

    /**
     * todrRd()
     *
     * vax_stddev.c:471-501.  FOUR branches, in SIMH's own order (MEASURED CORRECTION, veracity
     * re-dispatch round 2: the overflow branch below was missing entirely from the first version
     * of this port, undisclosed):
     *
     *   1. fault_PC inside the PRIMARY ROM window (NOT the mirror -- see ROM_MASK) -> the RAW
     *      counted register, unconditionally.  THE gotcha; see the file header.
     *   2. todrReg === 0 ("clock not running") -> 0, without ever touching the wall clock.
     *   3. Elapsed real time since the anchor >= TOY_MAX_SECS (vax_stddev.c:490-494) -> the
     *      register is ZEROED (`return todr_reg = 0` in the original is an assignment, not just a
     *      return -- reproduced here as an actual write to `this.todrReg`, not merely a return
     *      value) and 0 is returned.  Bit-exact, zero-tolerance cross-engine boundary: see
     *      TOY_MAX_SECS's own doc comment and tests/timerdiff.js's caseTodrOverflowBoundary().
     *   4. Otherwise: elapsed real time since the anchor todrWr() last recorded, in centiseconds,
     *      rounded to the nearest tick exactly as `(nsec + 5e6) / 1e7` rounds in the original.
     *
     * `this.exc.faultPC` is exc.js's own field (cpustate.js maintains it every instruction, and
     * exc.js already needed it for BPT/XFC) -- read directly, not duplicated.
     *
     * @this {ClkVAX}
     * @returns {number}
     */
    todrRd()
    {
        let pc = (this.exc && typeof this.exc.faultPC === "number") ? (this.exc.faultPC >>> 0) : 0;
        if (((pc & ROM_MASK) >>> 0) === ROM_TAG) return this.todrReg | 0;
        if (this.todrReg === 0) return 0;
        let elapsedMs = Date.now() - this.wallBaseMs;
        if (elapsedMs / 1000 >= TOY_MAX_SECS) { this.todrReg = 0; return 0; }
        return Math.round(elapsedMs / 10) | 0;
    }

    /**
     * todrWr(data)
     *
     * vax_stddev.c:503-533.  `data` is interpreted as a centisecond count (matching what
     * todrRd()'s branch 4 returns) and anchored against the REAL host clock right now:
     * `wallBaseMs = now - data*10`, so a read an instant later reconstructs `data` back out (see
     * tests/timerdiff.js's file header for why write-then-immediate-read is the only cross-engine
     * comparison this branch can honestly make).  data===0 is "stop the clock" (vax_stddev.c's
     * `else` branch: both toy_gmtbase fields go to 0) -- todrRd()'s branch 2 then short-circuits
     * before ever consulting wallBaseMs again.
     *
     * MEASURED BUG, caught by tests/timerdiff.js's overflow-boundary case (round 2): `data * 10`
     * MUST use the UNSIGNED magnitude.  The first version wrote `data = data | 0` (sign-extending
     * to a NEGATIVE int32 for any value >= 0x80000000) and then multiplied THAT by 10 -- exactly
     * the "arithmetic on a signed int32" hazard HANDOFF.md 7 names.  TODR is architecturally an
     * UNSIGNED 32-bit counter; `data >>> 0` is the magnitude the real centisecond count actually
     * is, and is what must be scaled to milliseconds.  `this.todrReg` itself is still stored via
     * plain `| 0` (an ordinary int32 BIT PATTERN, consistent with every other register field in
     * this tree -- see defines.js's masking-is-sign-agnostic rule) since nothing downstream ever
     * does arithmetic on it directly except tick()'s `+1`, which is sign-agnostic by construction.
     *
     * @this {ClkVAX}
     * @param {number} data
     */
    todrWr(data)
    {
        let magnitude = data >>> 0;
        if (magnitude !== 0) {
            this.todrBlow = 0;
            this.wallBaseMs = Date.now() - (magnitude * 10);
        } else {
            this.wallBaseMs = 0;
        }
        this.todrReg = data | 0;
    }

    /**
     * tick(cpu)
     *
     * The device-service hook cpustate.js's stepCPU() calls once per instruction retired (see its
     * own doc comment for the timing-model decision).  Throttled to INSTRS_PER_TICK so that the
     * OBSERVABLE effect -- one clk_svc equivalent -- still happens on a fixed, deterministic
     * schedule measured in instructions, matching vax_stddev.c's clk_svc body exactly at the
     * point it actually fires:
     *
     *   if (clk_csr & CSR_IE) SET_INT(CLK);                 -- every tick IE is set, unconditionally
     *   if (!todr_blow && todr_reg) todr_reg = todr_reg + 1; -- only while "running"
     *
     * raiseInterrupt() is idempotent (an OR of one bit -- exc.js's own doc comment), so calling it
     * on every tick while a previous request is still unacknowledged is exactly what real SIMH
     * does too (SET_INT called unconditionally, not gated on "is it already set"); the bit is only
     * ever cleared by deviceVector()'s acknowledge scan (already verified sound) or by iccsWr()
     * above clearing IE.
     *
     * @this {ClkVAX}
     * @param {Object} cpu
     */
    tick(cpu)
    {
        this._instrsSinceTick++;
        if (this._instrsSinceTick < INSTRS_PER_TICK) return;
        this._instrsSinceTick = 0;
        this.tickCount++;
        let exc = this.exc || (cpu && cpu.exc);
        if ((this.csr & CSR_IE) && exc) exc.raiseInterrupt(IPL_CLK_ABS, INT_V_CLK);
        if (!this.todrBlow && this.todrReg !== 0) this.todrReg = (this.todrReg + 1) | 0;
    }

    /**
     * instrsToEvent()
     *
     * pcjsvax-af8.  How many more tick() calls -- COUNTING THE ONE ABOUT TO HAPPEN AS 1 -- until
     * this device next does something the guest can observe.  tick() fires on the call that brings
     * `_instrsSinceTick` up to INSTRS_PER_TICK, so that call is the (INSTRS_PER_TICK -
     * _instrsSinceTick)-th from now.  Never zero and never negative: `_instrsSinceTick` is reset to
     * 0 by the firing call and only ever incremented by one per call in between.
     *
     * @this {ClkVAX}
     * @returns {number}
     */
    instrsToEvent() { return INSTRS_PER_TICK - this._instrsSinceTick; }

    /**
     * idleAble -- SIMH's UNIT_IDLE.  `UNIT clk_unit = { UDATA (&clk_svc, UNIT_IDLE+UNIT_FIX, ...)}`
     * (vax_stddev.c:218).  This is the ONE device in this machine whose next event is allowed to
     * put the host to sleep; see idle.js's idleSkip() for why that distinction is load-bearing.
     *
     * @this {ClkVAX}
     * @returns {boolean}
     */
    get idleAble() { return true; }

    /**
     * skipInstrs(n)
     *
     * pcjsvax-af8.  Advance this device's notion of retired instructions by `n` WITHOUT firing --
     * idle.js guarantees `n < instrsToEvent()`, so the tick boundary is never crossed here and the
     * next real tick() call fires on exactly the instruction it would have fired on anyway.  That
     * is what keeps TODR bit-identical across an idle skip; a skip that stopped this counter would
     * stop the guest clock, which is pcjsvax-c16 (`Event: Too Few Servers Detected`, a startup that
     * never completes).
     *
     * The bound is ASSERTED, not assumed: a caller that oversteps would silently swallow a clock
     * tick, and a swallowed tick is invisible in every direction except the guest's own idea of
     * what time it is.
     *
     * @this {ClkVAX}
     * @param {number} n
     */
    skipInstrs(n)
    {
        if (!(n >= 0) || n >= this.instrsToEvent()) {
            throw new Error(`clk.js: skipInstrs(${n}) would cross the tick boundary ` +
                `(${this._instrsSinceTick} of ${INSTRS_PER_TICK} elapsed, ` +
                `${this.instrsToEvent()} to go).  A skip past a device's own event silently ` +
                `swallows it; idle.js must pass min(instrsToEvent) - 1 over every device`);
        }
        this._instrsSinceTick += n;
    }
}

export {
    MT_ICCS, MT_TODR, CSR_IE, CLKCSR_IMP, CLKCSR_RW,
    IPL_CLK_ABS, INT_V_CLK, SCB_INTTIM, ROM_MASK, ROM_TAG, ROM_BASE, INSTRS_PER_TICK, TOY_MAX_SECS,
    TODR_TICKS_PER_SEC, USECS_PER_INSTR, TICK_USECS,
    dayOfYear
};
