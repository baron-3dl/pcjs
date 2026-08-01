/**
 * @fileoverview Implements SIMH's guest-idle detection (cpu_idle) and the virtual-time skip it drives
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
 * WHAT THIS IS -- pcjsvax-af8
 * ============================================================================
 * A guest operating system with nothing to do does not stop executing: it spins in a loop looking
 * for work.  MEASURED on this engine 2026-07-30, VAX/VMS V5.5-2H4 sitting at a bare `Username: `
 * prompt with nobody typing retires **4.15M instructions/second forever**, of which 99.56% are two
 * instructions.  In a browser tab that is a pinned core and a laptop fan that never stops.
 *
 * SIMH solves this by RECOGNISING the idle loop -- not by throttling, not by sampling -- and this
 * file is a port of that recognition, site for site.  `cpu_idle()` is called from NINE places
 * across vax_cpu.c, vax_cpu1.c and vax_defs.h; every one of them is here.  Three of those nine
 * `if`s are a disjunction of unrelated predicates that happen to share a statement (TSTL covers
 * VMS *or* old-Ultrix *or* Quasijarus; BEQL covers a boot-ROM prompt *or* VAXELN), so IDLE_SITES
 * below names TWELVE arms over those nine calls -- one per independently reachable predicate,
 * because a coverage assertion over the calls would let an unreached arm pass (HANDOFF.md standing
 * rule 6).  The predicate for each is transcribed from the C, including the parts that look
 * redundant.
 *
 * ============================================================================
 * WHAT THE MEASUREMENT FOUND, AND WHY IT CHANGES WHICH SITE MATTERS
 * ============================================================================
 * pcjsvax-af8's own text names the TSTL site (vax_cpu.c:1640-1654) as *the* VMS mechanism.  That
 * is true of OpenVMS V7.1 and **FALSE of VAX/VMS V5.5-2H4, which is the demo target.**  Measured
 * directly, at a login prompt, over 2,000,000 sampled instructions on each volume:
 *
 *   V5.5-2H4 (~/vms55-rd54.dsk), 99.56% of instructions at IPL=3 on the interrupt stack:
 *       802868DF  BBC #3,0x26C(R3),+55        6 bytes, falls through          49.78%
 *       802868E5  BBS R1,@#80004EC0,-14       8 bytes, branches back          49.78%
 *     -- there is NO TSTL in this loop at all.  The only cpu_idle() site it reaches is the
 *        BBS one (vax_cpu.c:2470-2474).
 *
 *   OpenVMS V7.1 (~/vaxclone.dsk), 99.05% of instructions at IPL=3 on the interrupt stack:
 *       805B6B47  BBC #3,0x26C(R3),+0x4E      6 bytes, falls through          24.76%
 *       805B6B4D  TSTL @#800053DC             6 bytes, sets Z                 24.76%
 *       805B6B53  BEQL +6                     2 bytes, taken                  24.76%
 *       805B6B5B  BBS R1,@#80004EC0,-28       8 bytes, branches back          24.77%
 *     -- reaches BOTH the TSTL site and the BBS site.
 *
 * So a port that stopped at the TSTL case would have left the DEMO TARGET burning a core while
 * appearing to work on the other volume.  Both loops end on the same `BBS R1,@#80004EC0` --
 * SCH$GL_COMQS, the computable-queue summary longword -- which is why the BBS site is the one that
 * covers both releases.  This is also why vax_cpu.c's own os_tab (:3734) labels VAX_IDLE_VMS
 * "VMS 5.0 and 5.1": the mask is a family, not a release, and the several sites it gates are what
 * make it cover releases whose idle loops do not look alike.
 *
 * ============================================================================
 * WHAT "IDLE" DOES HERE, AND WHY IT IS NOT sim_idle()
 * ============================================================================
 * SIMH's `cpu_idle()` is `sim_idle (TMR_CLK, TRUE)`: consult the event queue, advance sim_interval
 * to the next event, and sleep the host until then.  Both halves matter and they are separable, so
 * this file separates them:
 *
 *   1. THE VIRTUAL-TIME SKIP (idleSkip(), below).  This engine's guest clock is derived from
 *      INSTRUCTIONS RETIRED -- clk.js's INSTRS_PER_TICK is 10,000 at USECS_PER_INSTR = 1
 *      (pcjsvax-a6f) -- so an idle that merely stops executing instructions STOPS THE GUEST CLOCK.
 *      pcjsvax-c16 is what that looks like from outside: DTSS `Event: Too Few Servers Detected`
 *      fifteen times, a startup that never completes, a boot that bugchecks.  So the skip does not
 *      skip time; it FAST-FORWARDS it.  It advances every per-instruction device counter by
 *      exactly the number of instructions elided, to exactly the next scheduled device event and
 *      no further, so TODR, the SSC's TIR pair and rq.js's event queue all see the same instruction
 *      count they would have seen had the idle loop actually run.  THE GUEST CLOCK IS BIT-IDENTICAL
 *      ACROSS AN IDLE SKIP; tests/idlediff.js asserts that as an EXACT equality, not a tolerance.
 *
 *   2. THE HOST SLEEP.  This file does NOT sleep -- it cannot, because "sleep" has no single
 *      meaning across the two hosts this engine runs on (a synchronous Atomics.wait under Node, a
 *      timer that must not re-arm eagerly in a browser Worker).  Instead it BANKS the guest
 *      microseconds elided in `cpu.idleUsecs`, and the driver decides.  browser/vaxmachine.js
 *      returns it from runSlice(); browser/vaxworker.js turns it into a timer instead of a
 *      MessageChannel re-arm; tests/vmsbootprobe.js turns it into an Atomics.wait.
 *
 * ============================================================================
 * IT IS OFF BY DEFAULT, AND THAT IS SIMH's SEMANTICS RATHER THAN A HEDGE
 * ============================================================================
 * `cpu.idleEnable` defaults FALSE.  On real SIMH `cpu_idle()` is called unconditionally but
 * `sim_idle()` returns immediately unless `sim_idle_enab` is set, and `sim_idle_enab` is set by
 * `SET CPU IDLE` -- which the 34-check gate never issues.  So the gate's oracle does not idle, and
 * neither does this engine under the gate: every differential in tests/ runs with idleEnable false
 * and is bit-for-bit unaffected by this file.  A driver that wants the behaviour asks for it, the
 * same way a SIMH user types `SET CPU IDLE=VMS`.
 *
 * *** THAT MAKES tests/idlediff.js LOAD-BEARING RATHER THAN OPTIONAL (HANDOFF.md standing rule
 * 13). ***  A feature no gate check exercises is a feature that silently rots; idlediff.js turns
 * idleEnable ON, drives the MEASURED instruction bytes of both volumes' real idle loops through
 * it, and fails the run if detection stops firing, if the clock stops matching, or if any of the
 * nine sites below stops being reached.
 *
 * ============================================================================
 * WHAT IS DELIBERATELY NOT PORTED
 * ============================================================================
 * - `CHECK_FOR_IDLE_LOOP`'s OTHER half (vax_defs.h:685-690): `if (PC == fault_PC && IPL == 0x1F)
 *   ABORT (STOP_LOOP)`.  A branch to itself with interrupts locked out is an infinite loop and
 *   SIMH stops the simulator.  The IDLE half of that macro is ported below (idleSelfLoop); the
 *   STOP half is NOT, because it would stop the machine in the 34-check gate at a point the gate
 *   has never stopped before, and no measurement in this item established that any graded run
 *   reaches it.  This is a real, disclosed gap -- not a claim that the case cannot happen -- and it
 *   is filed as its own item rather than left in a comment.
 * - `sim_idle`'s host-time calibration (sim_idle_rate_ms, sim_rtcn_calb catch-up).  SIMH sleeps
 *   only while it is running AHEAD of real time and re-calibrates its clock afterwards.  Here the
 *   guest clock is instruction-derived and therefore already exact across the skip, so there is
 *   nothing to re-calibrate; what the driver does with the banked microseconds is a throttling
 *   policy, and it lives with the driver.
 */

import { USECS_PER_INSTR } from "./clk.js";

/* vax_defs.h:832-840.  Transcribed, not renumbered. */
const VAX_IDLE_VMS        = 0x01;
const VAX_IDLE_ULT        = 0x02;                   /* Ultrix, more recent versions */
const VAX_IDLE_ULTOLD     = 0x04;                   /* Ultrix, older versions */
const VAX_IDLE_ULT1X      = 0x08;                   /* Ultrix 1.x */
const VAX_IDLE_QUAD       = 0x10;
const VAX_IDLE_BSDNEW     = 0x20;
const VAX_IDLE_ELN        = 0x40;                   /* VAXELN (== VAX_IDLE_SYSV in the C) */
const VAX_IDLE_INFOSERVER = 0x80;

/**
 * OS_TAB -- vax_cpu.c:3733-3753's `os_tab[]`, in its own order, so `SET CPU IDLE=<name>` means
 * here exactly what it means there.  Derived-not-hand-enumerated (HANDOFF.md standing rule 5) is
 * not available for this one: it IS the enumeration, and its authority is the C table it copies.
 */
const OS_TAB = [
    {name: "VMS",           mask: VAX_IDLE_VMS},
    {name: "INFOSERVER",    mask: VAX_IDLE_INFOSERVER},
    {name: "ULTRIX",        mask: VAX_IDLE_ULT},
    {name: "ULTRIXOLD",     mask: VAX_IDLE_ULTOLD},
    {name: "ULTRIX-1.X",    mask: VAX_IDLE_ULT1X},
    {name: "3BSD",          mask: VAX_IDLE_ULT1X},
    {name: "4.0BSD",        mask: VAX_IDLE_ULT1X},
    {name: "4.1BSD",        mask: VAX_IDLE_ULT1X},
    {name: "4.2BSD",        mask: VAX_IDLE_ULT1X},
    {name: "QUASIJARUS",    mask: VAX_IDLE_QUAD},
    {name: "4.3BSD",        mask: VAX_IDLE_QUAD},
    {name: "4.4BSD-Reno",   mask: VAX_IDLE_QUAD},
    {name: "NETBSD",        mask: VAX_IDLE_BSDNEW},
    {name: "NETBSDOLD",     mask: VAX_IDLE_ULTOLD},
    {name: "OPENBSD",       mask: VAX_IDLE_BSDNEW},
    {name: "OPENBSDOLD",    mask: VAX_IDLE_QUAD},
    {name: "32V",           mask: VAX_IDLE_VMS},
    {name: "ELN",           mask: VAX_IDLE_ELN},
    {name: "MDM",           mask: VAX_IDLE_ELN}
];

/**
 * IDLE_SITES -- every place vax_cpu.c / vax_cpu1.c calls cpu_idle(), by the name this file uses
 * for it, WITH the C line it came from.  tests/idlediff.js derives its coverage assertion from
 * THIS ARRAY rather than from a count typed into the test (HANDOFF.md standing rules 4, 5 and 6):
 * a site added here and left unexercised fails the run, and the assertion does not scale with how
 * many cases the test happens to contain.
 */
const IDLE_SITES = [
    {site: "tstl-vms",          c: "vax_cpu.c:1640-1654",  masks: VAX_IDLE_VMS},
    {site: "tstl-ultold",       c: "vax_cpu.c:1640-1654",  masks: VAX_IDLE_ULTOLD},
    {site: "tstl-quad",         c: "vax_cpu.c:1640-1654",  masks: VAX_IDLE_QUAD},
    {site: "bitl-ult",          c: "vax_cpu.c:1864-1871",  masks: VAX_IDLE_ULT},
    {site: "ffs-ult1x",         c: "vax_cpu.c:2578-2584",  masks: VAX_IDLE_ULT1X},
    {site: "beql-romprompt",    c: "vax_cpu.c:2246-2254",  masks: 0},
    {site: "beql-eln",          c: "vax_cpu.c:2250-2254",  masks: VAX_IDLE_ELN},
    {site: "bvs-infoserver",    c: "vax_cpu.c:2266-2270",  masks: VAX_IDLE_INFOSERVER},
    {site: "bbs-vms",           c: "vax_cpu.c:2470-2474",  masks: VAX_IDLE_VMS},
    {site: "blbc-romprompt",    c: "vax_cpu.c:2510-2511",  masks: 0},
    {site: "mtpr-ipl-bsdnew",   c: "vax_cpu1.c:1510-1513", masks: VAX_IDLE_BSDNEW},
    {site: "branch-self",       c: "vax_defs.h:685-690",   masks: 0}
];

/* PSL fields, same constants exc.js/cpustate.js use, restated locally so this file has no
   dependency on either (both of them depend, directly or transitively, on this one). */
const PSL_IS     = 1 << 26;
const PSL_V_IPL  = 16;
const PSL_M_IPL  = 0x1F;

/** vax_cpu.c's `PSL_GETIPL(PSL)`. */
function getIPL(psl) { return (psl >>> PSL_V_IPL) & PSL_M_IPL; }

/** `fault_PC & 0x80000000` -- "in system space?".  Signed-safe: bit 31 tested as a mask, not a
    comparison (HANDOFF.md 7.2 -- a relational compare against 0x80000000 is the real hazard). */
function inSystemSpace(pc) { return (pc & 0x80000000) !== 0; }

/** `(fault_PC & 0x7fffffff) < 0x<n>` -- "in LOW system space?". */
function inLowSystem(pc, limit) { return ((pc & 0x7FFFFFFF) >>> 0) < limit; }

/*
 * ---------------------------------------------------------------------------
 * THE NINE CALL SITES.  Each is transcribed from its C `if`, in the same order of tests, and each
 * ends in cpu.requestIdle(<site>).  Every one is invoked from its instruction body behind an
 * `if (cpu.idleEnable)` guard at the CALL SITE -- so with idling off the cost is one already-hot
 * boolean field test and V8 never enters the function.
 * ---------------------------------------------------------------------------
 */

/**
 * idleTstl(cpu)
 *
 * vax_cpu.c:1640-1654, the TSTL case.  The caller has already established `cc == CC_Z`, which for
 * CC_IIZZ_L means the operand was zero and nothing else -- so the caller's test is `op0 === 0`
 * and this function does not re-derive it.
 *
 * @param {Object} cpu
 */
function idleTstl(cpu)
{
    let psl = cpu.psl, faultPC = cpu.exc.faultPC, mask = cpu.idleMask;
    if (((cpu.regs[15] - faultPC) | 0) !== 6) return;       /* 6 byte instruction? */
    if (!inSystemSpace(faultPC)) return;                    /* in system space? */
    if ((mask & VAX_IDLE_VMS) && getIPL(psl) === 0x3 && (psl & PSL_IS)) {
        cpu.requestIdle("tstl-vms");
        return;
    }
    /* The Ultrix-old / Quasijarus arm shares the low-system-space test; the C nests it that way
       and the nesting is load-bearing (VMS does NOT require low system space, the other two do). */
    if (!inLowSystem(faultPC, 0x4000)) return;
    if ((mask & VAX_IDLE_ULTOLD) && getIPL(psl) === 0x1) { cpu.requestIdle("tstl-ultold"); return; }
    if ((mask & VAX_IDLE_QUAD) && getIPL(psl) === 0x0) { cpu.requestIdle("tstl-quad"); return; }
}

/**
 * idleBitl(cpu)
 *
 * vax_cpu.c:1864-1871, the BITL case.  Caller has established `cc == CC_Z`.
 *
 * @param {Object} cpu
 */
function idleBitl(cpu)
{
    let psl = cpu.psl, faultPC = cpu.exc.faultPC;
    if (!(cpu.idleMask & VAX_IDLE_ULT)) return;
    if (!(psl & PSL_IS)) return;                            /* on IS? */
    if (getIPL(psl) !== 0x18) return;                       /* at IPL 18 (hex)? */
    if (!inSystemSpace(faultPC)) return;
    if (((cpu.regs[15] - faultPC) | 0) !== 8) return;       /* 8 byte instruction? */
    if (!inLowSystem(faultPC, 0x6000)) return;
    cpu.requestIdle("bitl-ult");
}

/**
 * idleFfs(cpu)
 *
 * vax_cpu.c:2578-2584, the FFS case.  Caller has established `cc == CC_Z` (no set bits found).
 *
 * @param {Object} cpu
 */
function idleFfs(cpu)
{
    let psl = cpu.psl, faultPC = cpu.exc.faultPC;
    if (!(cpu.idleMask & VAX_IDLE_ULT1X)) return;
    if (getIPL(psl) !== 0x0) return;
    if (!inSystemSpace(faultPC)) return;
    if (!inLowSystem(faultPC, 0x3000)) return;
    cpu.requestIdle("ffs-ult1x");
}

/**
 * ROM_PROMPT_PC -- vax_cpu.c:2252-2253's two literal `fault_PC` values, the KA655 console ROM's
 * own character-prompt loop.  They are NOT gated by cpu_idle_mask in the C (no OS is running yet),
 * only by "on the interrupt stack, at IPL 31, with memory management off".
 */
const ROM_PROMPT_PC = [0x2004361B, 0x20046A36];

/** vax_cpu.c:2511's MicroVAX 2 boot-ROM character prompt, on BLBC.  Also ungated by any mask, and
    in the C it is gated by NOTHING ELSE AT ALL -- not IPL, not IS, not mapen.  Transcribed as it
    stands; see tests/idlediff.js's case of the same name, which grades exactly that. */
const MV2_PROMPT_PC = 0x20040C09;

/**
 * idleBeql(cpu, brdisp)
 *
 * vax_cpu.c:2246-2254, the BEQL case, AFTER the branch has been taken.  Two unrelated conditions
 * share one `if`: a boot-ROM character prompt (no OS, no mask) and VAXELN's idle loop.
 *
 * @param {Object} cpu
 * @param {number} brdisp the RAW (unsign-extended) branch displacement byte
 */
function idleBeql(cpu, brdisp)
{
    let psl = cpu.psl, faultPC = cpu.exc.faultPC;
    if (!(psl & PSL_IS)) return;                            /* both arms require the interrupt stack */
    if (getIPL(psl) === 0x1F && cpu.mmu.mapen === 0 &&
        (faultPC === ROM_PROMPT_PC[0] || faultPC === ROM_PROMPT_PC[1])) {
        cpu.requestIdle("beql-romprompt");
        return;
    }
    if ((cpu.idleMask & VAX_IDLE_ELN) && brdisp === 0xFA && getIPL(psl) === 0x4) {
        cpu.requestIdle("beql-eln");
    }
}

/**
 * idleBvs(cpu, brdisp)
 *
 * vax_cpu.c:2266-2270, the BVS case, AFTER the branch has been taken.
 *
 * @param {Object} cpu
 * @param {number} brdisp the RAW (unsign-extended) branch displacement byte
 */
function idleBvs(cpu, brdisp)
{
    let psl = cpu.psl;
    if (!(cpu.idleMask & VAX_IDLE_INFOSERVER)) return;
    if (!(psl & PSL_IS)) return;
    if (brdisp !== 0xF1) return;                            /* branch to the prior INCL */
    if (getIPL(psl) !== 0x3) return;
    cpu.requestIdle("bvs-infoserver");
}

/**
 * idleBbs(cpu)
 *
 * vax_cpu.c:2470-2474, the BBS case, AFTER the branch has been taken.  *** THIS IS THE SITE THAT
 * MATTERS FOR THE DEMO. ***  It is the ONLY cpu_idle() site VAX/VMS V5.5-2H4's idle loop reaches
 * (see the file header's measurement), and OpenVMS V7.1 reaches it too.
 *
 * Note what the C does NOT test: not the displacement, not the direction of the branch, not the
 * faulting PC.  On the interrupt stack, at IPL 3, with VMS idling selected, ANY taken BBS is an
 * idle.  Transcribed as it stands rather than narrowed -- narrowing it would be inventing a
 * predicate, and tests/idlediff.js's `bbs-vms-no-pc-test` case grades that this port did not.
 *
 * @param {Object} cpu
 */
function idleBbs(cpu)
{
    let psl = cpu.psl;
    if (!(psl & PSL_IS)) return;
    if (getIPL(psl) !== 0x3) return;
    if (!(cpu.idleMask & VAX_IDLE_VMS)) return;
    cpu.requestIdle("bbs-vms");
}

/**
 * idleBlbc(cpu)
 *
 * vax_cpu.c:2510-2511, the BLBC case, BEFORE the branch (the C tests fault_PC and calls cpu_idle()
 * ahead of BRANCHB, so the order is preserved here by the call site).
 *
 * @param {Object} cpu
 */
function idleBlbc(cpu)
{
    if (cpu.exc.faultPC === MV2_PROMPT_PC) cpu.requestIdle("blbc-romprompt");
}

/**
 * idleMtprIpl(cpu, val)
 *
 * vax_cpu1.c:1510-1513, the MT_IPL case, AFTER the PSL's IPL field has been written.  Note the C
 * tests **PC**, not fault_PC, here -- alone among the sites -- and the comment on the line says
 * why ("System Space (Not BOOT ROM)").  Transcribed.
 *
 * @param {Object} cpu
 * @param {number} val the value written to IPL
 */
function idleMtprIpl(cpu, val)
{
    if (!(cpu.idleMask & VAX_IDLE_BSDNEW)) return;
    if (!inSystemSpace(cpu.regs[15])) return;
    if (val !== 1) return;
    cpu.requestIdle("mtpr-ipl-bsdnew");
}

/**
 * idleSelfLoop(cpu)
 *
 * vax_defs.h:685-690's CHECK_FOR_IDLE_LOOP, the OS-INDEPENDENT half: a plain branch or jump whose
 * target is the instruction itself.  The C reaches this from BRANCHB / BRANCHW / JUMP / CMODE_JUMP
 * and *** DELIBERATELY NOT *** from BRANCHB_ALWAYS / BRANCHW_ALWAYS / JUMP_ALWAYS, because -- in
 * its own words -- "Instructions which have side effects (ACB, AOBLSS, BBSC, BBCS, etc.) can't be
 * an idle loop".  control.js previously collapsed all six macros onto one `branch()` on the stated
 * grounds that the split was "purely SIMH's idle-loop-detection heuristic ... there is nothing here
 * to replicate".  That was correct while nothing replicated it; it is not correct now, so the split
 * is restored -- see control.js's branch()/branchAlways()/jump()/jumpAlways().
 *
 * The STOP_LOOP half of the macro (IPL 31 -> stop the simulator) is NOT ported; see the file
 * header's "WHAT IS DELIBERATELY NOT PORTED".
 *
 * @param {Object} cpu
 */
function idleSelfLoop(cpu)
{
    if (cpu.regs[15] === cpu.exc.faultPC) cpu.requestIdle("branch-self");
}

/*
 * ---------------------------------------------------------------------------
 * THE SKIP
 * ---------------------------------------------------------------------------
 */

/**
 * IDLE_SKIP_MAX -- an absolute ceiling on one skip, in instructions.
 *
 * Every device below reports a finite distance to its own next event, so the ceiling is not what
 * bounds the skip in practice -- clk.js's tick is 10,000 instructions away at the very most.  It
 * exists because a device that reports a WRONG distance (a stopped timer mis-reported as running
 * with 4 billion counts to go, an event queue whose head is stale) would otherwise fast-forward
 * the guest clock by hours in one step, and a clock that jumps is a worse failure than a clock that
 * stops -- it is silent.  Set to two clk ticks so it can never be reached by correct behaviour and
 * is therefore never a tuning knob (HANDOFF.md standing rule 3).
 */
const IDLE_SKIP_MAX = 20000;

/**
 * idleSkip(cpu)
 *
 * `sim_idle()`'s first half: find the next scheduled device event and advance every per-instruction
 * device counter to just short of it, eliding the instructions in between.
 *
 * THE ARITHMETIC, AND WHY IT IS "MINUS ONE".  cpustate.js's loop calls each device's tick() once
 * per iteration, BEFORE the instruction executes.  Let P be the number of tick() calls from now
 * -- counting the one about to happen as 1 -- until a given device next fires.  Then P-1 calls can
 * be elided and the P-th must be left to happen for real, so the device fires on exactly the
 * instruction it would have.  The skip is min(P) - 1 over every device, so NO device is skipped
 * past its own event.
 *
 * Devices report P through `instrsToEvent()`; a device with nothing scheduled returns Infinity.
 * A device that also needs its internal counter moved implements `skipInstrs(n)`; rq.js does not,
 * because its queue is keyed on `cpu.nTotalCycles`, which this function advances directly.  A
 * device declares `idleAble` for SIMH's UNIT_IDLE -- see the gate at the top of the body.
 *
 * @param {Object} cpu
 * @returns {number} instructions elided (0 if there was nothing to elide)
 */
function idleSkip(cpu)
{
    /*
     * *** THE UNIT_IDLE GATE, AND IT IS THE DIFFERENCE BETWEEN A 22-SECOND BOOT AND A 65-SECOND
     * ONE. ***  sim_idle() (sim_timer.c:1567-1571) refuses to idle at all when the HEAD of the
     * event queue belongs to a unit that is not flagged UNIT_IDLE:
     *
     *     ((sim_clock_queue != QUEUE_LIST_END) &&
     *      ((sim_clock_queue->flags & UNIT_IDLE) == 0))    -> return FALSE
     *
     * `clk_unit` is `UNIT_IDLE+UNIT_FIX` (vax_stddev.c:218) and rq's RQ_QUEUE and drive units are
     * not (pdp11_rq.c:3298 flags only RQ_TIMER, which this tree does not model).  So a guest
     * spinning at IPL 3 while a DISK TRANSFER IS IN FLIGHT is not idle in SIMH's sense: it is
     * waiting for something that will arrive as fast as the emulator can produce it, and slowing
     * down to real time would be slowing down the very thing it is waiting for.  A guest spinning
     * with only the 100 Hz clock scheduled has nothing coming but the next tick.
     *
     * MEASURED, this machine, VAX/VMS V5.5-2H4 to a login prompt: WITHOUT this gate, boot took
     * 65.1 s against a 20.4 s baseline, because startup's disk waits were being slept through.
     * WITH it, boot is unchanged and the login prompt still idles -- because by then nothing is
     * queued but the clock.
     *
     * `idleAble` is that UNIT_IDLE flag, read off whichever device owns the NEAREST event.  A
     * device that does not declare one is treated as not idle-able, which is the safe direction:
     * the failure mode is "kept running", not "slept through an event".
     */
    let p = IDLE_SKIP_MAX + 1, able = false, any = false;
    /*
     * A TIE SUPPRESSES.  Two devices whose next events are the same distance away BOTH fire on that
     * instruction, so if either is not idle-able the machine is not idle -- there is no ordering
     * that makes one of them not happen.  This is not hypothetical: the clock's distance sweeps
     * through every value from INSTRS_PER_TICK down to 1 once per tick, so it collides with any
     * other pending event's distance EXACTLY ONCE PER TICK, and a "first strictly-nearest device
     * wins" rule therefore leaks one skip per tick straight through the gate.  MEASURED while
     * building tests/idlediff.js: one 49-instruction skip per tick past a device that had declared
     * itself not idle-able.
     */
    const consider = (n, idleAble) => {
        any = true;
        if (n < p) { p = n; able = idleAble === true; }
        else if (n === p && idleAble !== true) able = false;
    };
    if (cpu.clk)  consider(cpu.clk.instrsToEvent(), cpu.clk.idleAble);
    if (cpu.tmr)  consider(cpu.tmr.instrsToEvent(), cpu.tmr.idleAble);
    if (cpu.qbus) consider(cpu.qbus.instrsToEvent(cpu), cpu.qbus.idleAble);
    if (!any || !able) return 0;
    let skip = p - 1;
    if (!(skip > 0)) return 0;
    if (skip > IDLE_SKIP_MAX) skip = IDLE_SKIP_MAX;

    /*
     * ADVANCE, DO NOT SKIP.  Every counter below is part of the guest's own notion of time; a skip
     * that left any one of them behind is pcjsvax-c16 (see the file header).  `nTotalCycles` is
     * this engine's `sim_gtime`/`sim_interval` and is what rq.js's event queue is keyed on, so it
     * moves here rather than in a device.
     */
    cpu.nTotalCycles += skip;
    if (cpu.clk) cpu.clk.skipInstrs(skip);
    if (cpu.tmr) cpu.tmr.skipInstrs(skip);

    cpu.idleSkipped += skip;
    cpu.idleUsecs += skip * USECS_PER_INSTR;
    return skip;
}

/**
 * ============================================================================
 * IdleThrottle -- HOW LONG TO SLEEP, AND WHY IT IS NOT THE GUEST TIME
 * ============================================================================
 * The obvious answer -- "sleep for as long as the guest time you skipped" -- is wrong here, and
 * MEASURED wrong: it costs 14 seconds on a boot.  It is worth writing down why, because the
 * reasoning is the whole design.
 *
 * SIMH sleeps `w_ms = sim_interval / sim_idle_cyc_ms` (sim_timer.c:1615), where sim_idle_cyc_ms is
 * cycles per millisecond of REAL time as continuously re-calibrated by sim_rtcn_calb.  So SIMH
 * sleeps THE HOST TIME THOSE INSTRUCTIONS WOULD HAVE COST, not the guest time they represent.  On
 * SIMH the two are the same number, because its calibration pins the guest's 100 Hz clock to the
 * host's real second by moving instructions-per-tick.  On THIS engine they are not: pcjsvax-a6f
 * fixed instructions-per-tick at 10,000 on purpose, so the guest clock is deterministic and the
 * ROM's self-test 53 passes on every run rather than sometimes -- and the consequence is that this
 * engine runs the guest at whatever multiple of real time the host can manage (MEASURED: 3.85x).
 *
 * Sleeping the GUEST time therefore throttles the machine down to 1.0x real time whenever it is
 * idle, which is a 4x slowdown applied to every pause in a boot.  MEASURED, V5.5-2H4 to a login
 * prompt: 34.4 s against a 20.0 s baseline, 18.9 s of it asleep.
 *
 * Sleeping the HOST TIME SAVED is SIMH's arithmetic and it is exactly neutral -- it gives back the
 * time the emulator did not spend, so the machine advances at precisely the rate it always did,
 * and the only thing that changes is that a core is not being burned to do it.  MEASURED with the
 * same binary and volume: boot 21.6 s (baseline 20.0), host CPU at an idle login prompt 1.6%
 * (baseline 99.6%), and the guest clock ticking 78 guest-seconds per 20 wall-seconds -- the SAME
 * 3.9x the un-idled engine produced.
 *
 * THE RATE IS MEASURED, NEVER ASSUMED.  It is this host's instructions per millisecond, taken from
 * the driver's own execution slices, which is the closest thing this engine has to sim_rtcn_calb.
 * Until a slice has measured it, sleepMsFor() returns 0 -- the machine simply runs, exactly as it
 * did before this file existed.  That is the safe direction to be wrong in: an unmeasured rate
 * costs CPU, whereas a guessed-too-low rate would silently slow the machine down.
 */
class IdleThrottle
{
    /**
     * @param {Object} [opts]
     *   {number} [maxSleepMs]  bound on ONE sleep; throttling, not timekeeping, so a short sleep
     *                          costs accuracy in nothing -- the guest clock has already advanced.
     *   {number} [minSleepMs]  below this, a timer round trip costs more than it saves.
     */
    constructor(opts = {})
    {
        this.rate = 0;                                  /* retired instructions per millisecond */
        this.samples = 0;
        this.maxSleepMs = opts.maxSleepMs === undefined ? 50 : opts.maxSleepMs;
        this.minSleepMs = opts.minSleepMs === undefined ? 1 : opts.minSleepMs;
        this.sleptMs = 0;
        this.sleeps = 0;
    }

    /**
     * noteBusy(instrs, ms)
     *
     * Feed one EXECUTION measurement: instructions actually retired, and the wall-clock
     * milliseconds spent retiring them -- NOT including any sleep, which is the caller's to
     * exclude.  Slices too short to time (`ms` under 2) or too small to be representative are
     * dropped rather than averaged in, because a sub-millisecond sample is mostly clock
     * granularity; an idling machine produces nothing but such slices, so the rate it measured
     * while it had work is the rate that persists, which is the intent.
     *
     * @param {number} instrs
     * @param {number} ms
     */
    noteBusy(instrs, ms)
    {
        if (!(ms >= 2) || !(instrs >= 1000)) return;
        let r = instrs / ms;
        this.rate = this.samples === 0 ? r : this.rate + 0.25 * (r - this.rate);
        this.samples++;
    }

    /**
     * sleepMsFor(elidedInstrs)
     *
     * How many milliseconds of host time those elided instructions would have cost.  Zero -- "do
     * not sleep" -- when no rate has been measured yet, or when the answer rounds below
     * minSleepMs.
     *
     * @param {number} elidedInstrs
     * @returns {number}
     */
    sleepMsFor(elidedInstrs)
    {
        if (!(this.rate > 0) || !(elidedInstrs > 0)) return 0;
        let ms = Math.round(elidedInstrs / this.rate);
        if (ms < this.minSleepMs) return 0;
        if (ms > this.maxSleepMs) ms = this.maxSleepMs;
        this.sleptMs += ms; this.sleeps++;
        return ms;
    }
}

export {
    IdleThrottle,
    VAX_IDLE_VMS, VAX_IDLE_ULT, VAX_IDLE_ULTOLD, VAX_IDLE_ULT1X, VAX_IDLE_QUAD,
    VAX_IDLE_BSDNEW, VAX_IDLE_ELN, VAX_IDLE_INFOSERVER,
    OS_TAB, IDLE_SITES, IDLE_SKIP_MAX, ROM_PROMPT_PC, MV2_PROMPT_PC,
    idleTstl, idleBitl, idleFfs, idleBeql, idleBvs, idleBbs, idleBlbc, idleMtprIpl, idleSelfLoop,
    idleSkip, getIPL
};
