/**
 * @fileoverview Implements the CVAX on-chip interval timer and time-of-day register (ICCS/TODR)
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
 * timerdiff.js's selfcheck includes a mutation that widens the check to the whole mirrored span,
 * to prove this precision is actually graded and not incidental.
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
 * WHAT IS DELIBERATELY NOT HERE
 * ============================================================================
 * - SSC T0/T1 (the programmable timers at SSC_BASE+0x100+) -- a DIFFERENT facility, owned by the
 *   countdown item per pcjsvax-954's own constraint.  Not one register here overlaps that scope.
 * - The console UART (CSRS/CSRD/CSTS/CSTD/RXCS/RXDB/TXCS/TXDB) and CADR/MSER/IORESET -- other
 *   MT_* numbers ReadIPR/WriteIPR dispatch to; read()/write() below pass them straight through to
 *   the SAME inert default (0 / dropped write) exc.js used for ALL of IPR_DEVICE before any device
 *   existed, so installing this class changes nothing about registers it does not own.
 * - `sim_rtcn_tick_ack` on an ICCS ack write (data & CSR_DONE) -- a host-timer-calibration hint
 *   with no architectural state (nothing reads it back); omitted, not forgotten.
 * - TODR's "battery low" / attach-a-TOY-file persistence mode (clk_unit attach/detach, the
 *   OS-Agnostic mode) -- no test in this tree runs offline across process restarts; `todrBlow`
 *   exists only so todr_wr(0)'s "stop the clock" state has somewhere to live and reads back inert.
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
 * INSTRS_PER_TICK -- the deterministic stand-in for clk_svc's real-time ~10ms/100Hz schedule
 * (vax_stddev.c UNIT clk_unit's CLK_DELAY, sim_rtcn_calb-recalibrated every service).  There is no
 * meaningful "instructions per 10ms" conversion once the model is instruction-count driven (that
 * would just reintroduce a host-speed dependency by another name) -- what DONE CONDITION 2
 * requires is that this number is FIXED, so the same run twice produces the same tick count at the
 * same instruction counts.  Deliberately small enough that a differential's step budgets (hundreds
 * to a few thousand instructions) exercise several ticks without needing a huge run.
 */
const INSTRS_PER_TICK = 200;

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
     * vax_stddev.c clk_reset() (570): clk_csr=0 and CLR_INT(CLK) -- todr_reg/todrBlow are
     * DELIBERATELY NOT touched here (matching ssc.js's SSCVAX.reset() precedent for ssc_base: a
     * real KA655's TODR is battery-backed and does not zero on a plain CPU reset).  This class's
     * reset() therefore only runs from the constructor in every test in this tree (one fresh
     * instance per machine, exactly like SSCVAX) -- see ssc.js's own reset() doc comment for why
     * that makes "instance-construction default" and "post-reset value" the same question here,
     * and why it would stop being harmless the moment something wires this to a reset handler.
     *
     * @this {ClkVAX}
     */
    reset()
    {
        this.csr = 0;
        if (this.exc) this.exc.clearInterrupt(IPL_CLK_ABS, INT_V_CLK);
        if (this.todrReg === undefined) {
            /* First-ever construction only (see doc comment above): vax_stddev.c's globals start
               `todr_reg = 0` and `todr_blow = 1` (vax_stddev.c:100-101) -- a fresh process's
               clock is STOPPED until something writes it, exactly like a never-set battery clock. */
            this.todrReg = 0;
            this.todrBlow = 1;
            this.wallBaseMs = 0;                 // toy_gmtbase's zero-initialized default
        }
        this._instrsSinceTick = 0;
        this.tickCount = 0;
    }

    /**
     * read(prn) / write(prn, val)
     *
     * The setIPRDevice(dev) contract exc.js's readIPR()/writeIPR() install against.  Anything
     * that is not ICCS or TODR (including NICR/ICR -- see the file header) passes straight through
     * to the SAME inert default (0 / dropped write) those numbers already got with no device
     * installed at all, so wiring this class in changes nothing about a register it does not own.
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
           not own): drop the write.  Real SIMH also sets ssc_bto here; exc.js does not model that
           for ANY off-chip register yet (a pre-existing, documented gap -- see exc.js's readIPR()/
           writeIPR() default comments) and this device does not newly introduce it either. */
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
     * vax_stddev.c:471-501.  Three branches, in SIMH's own order:
     *
     *   1. fault_PC inside the PRIMARY ROM window (NOT the mirror -- see ROM_MASK) -> the RAW
     *      counted register, unconditionally.  THE gotcha; see the file header.
     *   2. todrReg === 0 ("clock not running") -> 0, without ever touching the wall clock.
     *   3. Otherwise: elapsed real time since the anchor todrWr() last recorded, in centiseconds,
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
        return Math.round(elapsedMs / 10) | 0;
    }

    /**
     * todrWr(data)
     *
     * vax_stddev.c:503-533.  `data` is interpreted as a centisecond count (matching what
     * todrRd()'s branch 3 returns) and anchored against the REAL host clock right now:
     * `wallBaseMs = now - data*10`, so a read an instant later reconstructs `data` back out (see
     * tests/timerdiff.js's file header for why write-then-immediate-read is the only cross-engine
     * comparison this branch can honestly make).  data===0 is "stop the clock" (vax_stddev.c's
     * `else` branch: both toy_gmtbase fields go to 0) -- todrRd()'s branch 2 then short-circuits
     * before ever consulting wallBaseMs again.
     *
     * @this {ClkVAX}
     * @param {number} data
     */
    todrWr(data)
    {
        data = data | 0;
        if (data !== 0) {
            this.todrBlow = 0;
            this.wallBaseMs = Date.now() - (data * 10);
        } else {
            this.wallBaseMs = 0;
        }
        this.todrReg = data;
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
}

export {
    MT_ICCS, MT_TODR, CSR_IE, CLKCSR_IMP, CLKCSR_RW,
    IPL_CLK_ABS, INT_V_CLK, SCB_INTTIM, ROM_MASK, ROM_TAG, ROM_BASE, INSTRS_PER_TICK
};
