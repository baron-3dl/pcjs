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
 *   - UPDATE (pcjsvax-055): the SSC's two programmable timers T0/T1 (+0x100..+0x11C) ARE now
 *     decoded -- see the "SSC PROGRAMMABLE TIMERS T0/T1" section further down this file, and the
 *     readReg()/writeReg() cases for REG_T0CSR..REG_T1VEC.  This bullet is left in place, not
 *     deleted, only to record that they were ONCE deliberately out of scope here; do not read it
 *     as still describing the current decode.  Their address-strobe NEIGHBORS (ADS0M/ADS0K/ADS1M/
 *     ADS1K, +0x130/+0x134/+0x140/+0x144) were decoded earlier still, by pcjsvax-bfb -- see the
 *     SSCADS_MASK constant's own doc comment below for why they are a different, simpler animal.
 *
 * ============================================================================
 * WHAT HAPPENS TO EVERYTHING ELSE IN THIS BLOCK (two DIFFERENT answers, not one -- veracity finding)
 * ============================================================================
 * bus.js installs this decode over exactly [SSC_BASE, SSC_BASE+SSC_LENGTH) (see BusVAX.addSsc()).
 * Physical memory blocks are managed in BusVAX.BLOCK_SIZE (8KB) chunks, and SSC_BASE and NVR_BASE
 * share ONE such block (SSC_BASE=0x20140000, NVR_BASE=0x20140400, block size 0x2000) -- so THIS
 * controller's functions are the ones invoked for every address in that whole 8KB span, not merely
 * SSC's own 0x150 bytes.  makeSscController() below checks the physical address against
 * [SSC_BASE, SSC_BASE+SSC_LENGTH) and against NVR's own range, and these are the TWO GENUINELY
 * DIFFERENT outcomes, corrected by a post-merge veracity re-dispatch after an earlier version of
 * this file conflated them into one:
 *
 *   - An address INSIDE [SSC_BASE, SSC_BASE+SSC_LENGTH) but not one of the registers listed above
 *     (e.g. SSC+0x50, rg=0x14) is handled by THIS device -- readReg()/writeReg()'s own trailing
 *     default (0 / silent success) answers it, exactly matching vax_sysdev.c's ssc_rd()/ssc_wr(),
 *     whose switches have no `default:` label at all.  makeSscController() below never even sees a
 *     null/false from SSCVAX for such an address; there is nothing for it to fall through on.
 *     MEASURED live: SSC+0x50 reads 0 and accepts a write with NO machine check on the real oracle.
 *   - An address OUTSIDE both this device's range AND NVR's (the block's unused tail, or -- before
 *     nvr.js existed -- NVR itself) falls through to the SAME readNone/writeNone/readWordNone/...
 *     the shared empty block would have used -- i.e. a bus fault, dispatched by cpustate.js's
 *     onBusFault() into a real machine check (pcjsvax-446).  THAT fault is what tests/romdiff.js's
 *     FaultGrader.verdict() grades against the live oracle to find and NAME the next boundary -- the
 *     same mechanism that named THIS item's own starting point (SSC+0x0) and, later, NVR's.  Since
 *     pcjsvax-fe7 the fault alone is not the boundary: an address the ORACLE also faults on
 *     identically is walked past, and only one SIMH SERVICES is named.
 *
 * The bug this replaces: an earlier revision had readReg()/writeReg() return null/false for an
 * uncased-but-in-range register too, which makeSscController() then ALSO routed to readNone/
 * writeNone -- machine-checking on an address a real KA655 answers silently.  Reachability was
 * measured as latent on the boot-entry path this item's own boundary walk exercises (400
 * instructions of oracle execution past the current boundary touch only the SSC T0/T1 vector
 * registers, pcjsvax-055's scope, never a genuinely uncased offset) but is exactly the class of
 * divergence a LATER boundary advance would hit blind.
 *
 * ============================================================================
 * UNALIGNED ACCESS SPANNING TWO ADJACENT REGISTER-SPACE LONGWORDS -- pcjsvax-855, CORRECTED PREMISE
 * ============================================================================
 * CORRECTED TWICE.  First (pcjsvax-320 veracity re-dispatch): an earlier version of this section
 * claimed "no code path, on real SIMH or here, stitches two adjacent registers together for an
 * unaligned reference."  That is FALSE for every case except a byte access and a word access at
 * offset 1.  MEASURED directly against the real oracle (SSC+0x0C = 0, SSC+0x10 = 0x5A5A5A5A):
 *
 *     MOVB @#2014000F        -> one longword only (register 0x0C)      claim HOLDS for bytes
 *     MOVW @#2014000F        -> 0x5A00                                 STITCHED across 0x0C/0x10
 *     MOVL @#2014000D        -> 0x5A000000                             STITCHED across 0x0C/0x10
 *
 * Second, and this is the one that matters for THIS file (pcjsvax-855 itself): the first correction
 * assumed, without checking, that the stitch above therefore belonged HERE, in ssc.js's own
 * readWord()/readLong()/writeWord()/writeLong() -- a plausible-sounding but UNVERIFIED claim about
 * WHERE in this port the C source's Read()/Write() shape had to live, exactly the class of error
 * HANDOFF.md's §7 exists to name.  It does not belong here.  mmu.js's `readData()`/`writeData()`
 * (an exact, already-shipped port of vax_mmu.h's `Read()`/`Write()`, mmu.js ~line 630/705) are THE
 * canonical entry point every operand memory access in this engine goes through -- decode.js's own
 * interface comment calls `readData()` "identical to the decoder's machine interface", and grepping
 * this whole module tree confirms it: NOTHING outside mmu.js and bus.js itself ever calls
 * `bus.getWord()`/`getLong()`/`setWord()`/`setLong()` directly.  `readData()`/`writeData()` already
 * do EXACTLY the alignment branch vax_mmu.h's `Read()`/`Write()` do -- and for every unaligned
 * fragment (word bo=1, word bo=3, any unaligned longword) they route through `readU()`/`writeU()`
 * (mmu.js ~line 927/1016), which computes `let addr = pa & ~0x03` FIRST and only ever calls
 * `this.bus.getLong(addr)`/`setLong(addr, ...)` -- ALIGNED -- doing the shift/mask/merge themselves,
 * ENTIRELY WITHIN mmu.js, before this file's own `readReg()`/`writeReg()` switch ever runs.  So:
 *
 *   - ssc.js's readWord()/writeWord() are called by the bus ONLY at bo=0 or bo=2 (mmu.js's readW()/
 *     writeW() are reached ONLY from readData()'s/writeData()'s ALREADY-ALIGNED fast path).
 *   - ssc.js's readLong()/writeLong() are called by the bus ONLY at bo=0, for the SAME reason, PLUS
 *     as the "read/write the containing aligned longword" primitive `readU()`/`writeU()` use for
 *     EVERY unaligned fragment -- always at an aligned address, never at the original unaligned one.
 *
 * MEASURED, not merely reasoned from the two facts above: an earlier revision of this file added a
 * two-register stitch directly to readWord()/readLong()/writeWord()/writeLong() (bo=1/bo=3
 * branches), mirroring vax_mmu.h's shape literally.  tests/sscunaligneddiff.js exercises the exact
 * MOVW/MOVL sequences the item's own filing names, through REAL `cpu.stepCPU()` execution (not a
 * direct unit-level call to these methods) -- and it PASSED, matching the real oracle, with those
 * branches monkeypatched back to their PRE-fix, single-register form.  Mutating code that a real
 * instruction path never reaches cannot change that path's observable output; that null result is
 * the proof, not an assumption, that the branches were dead code.  They were removed rather than
 * shipped: unreachable code graded by nothing but a direct, non-oracle unit probe is exactly the
 * "mutation must perturb the shipped path" antipattern HANDOFF.md's rule 11 warns about, aimed at
 * this file's OWN structure instead of at a test.
 *
 * So the two-register stitch the item asked for IS modelled, correctly, end to end -- it always
 * was, via mmu.js, which this item does not need to (and does not) touch.  tests/sscunaligneddiff.js
 * is the regression this item actually needed: a real-CPU-instruction, oracle-graded case proving
 * that already-correct behavior for THIS device's registers specifically, since (per this item's own
 * filing) nothing before it ever drove a cross-longword unaligned SSC reference.  The SSCBASE+
 * SSCSIZE boundary the item also raised ("which side's fault wins if one longword is undecoded") is
 * likewise ALREADY correct at the layer that actually owns it: `readU(pa1)`'s `bus.getLong(pa1 & ~3)`
 * lands in makeSscController()'s own `inRange()`/`nvrInRange()` fallback (this file's existing
 * `readLongNone`/`writeLongNone` convention) exactly as any other undecoded address does -- no new
 * code, here or anywhere, was needed for that either.  tests/sscunaligneddiff.js's `boundary_fault`
 * case grades it directly against the oracle (a genuine machine check, confirmed live) as a
 * regression, not a claim.
 *
 * KNOWN_UNIMPLEMENTED_READ/_WRITE are UNCHANGED by this item -- no entry added or removed (see
 * those constants' own asymmetric-drift-risk warning).
 */

import { VAX } from "./defines.js";
import { NVR_BASE, NVR_LENGTH } from "./nvr.js";
import { SSCBTO_BTO, SSCBTO_RWT, IPL_HMIN } from "./exc.js";

const SSC_BASE = VAX.PHYSMEM.SSC_BASE >>> 0;
const SSC_LENGTH = VAX.PHYSMEM.SSC_LENGTH;

/*
 * Register longword indices, vax_sysdev.c:1266 (`rg = (pa - SSCBASE) >> 2`) and :1354 (same, for
 * writes).  Only the registers this item decodes get a name; see the file header for the rest.
 */
const REG_BASE = 0x00;
const REG_CNF  = 0x04;
const REG_BTO  = 0x08;
const REG_OTP  = 0x0C;
const REG_ADS0M = 0x4C;
const REG_ADS0K = 0x4D;
const REG_ADS1M = 0x50;
const REG_ADS1K = 0x51;
const REG_RXCS = 0x20;
const REG_RXDB = 0x21;
const REG_TXCS = 0x22;
const REG_TXDB = 0x23;
const REG_T0CSR = 0x40;
const REG_T0INT = 0x41;
const REG_T0NI  = 0x42;
const REG_T0VEC = 0x43;
const REG_T1CSR = 0x44;
const REG_T1INT = 0x45;
const REG_T1NI  = 0x46;
const REG_T1VEC = 0x47;

/* SSC bus-timeout register, vax_sysdev.c:179-183 and :1279-1280/1376-1378 (ssc_rd/ssc_wr case
   0x08) -- pcjsvax-bfb's own romdiff boundary-advance, instruction #28 (a READ of SSC+0x20).  NOT
   new state: pcjsvax-446 already models this exact register on VAXExc (`cpu.exc.sscBto`,
   exc.js's busTimeout()) for the machine-check side of an unbacked register-space reference --
   vax_sysdev.c's `ssc_bto` is the SAME C global both busTimeout()'s equivalent (ReadReg/WriteReg's
   `default:` case) and ssc_rd()/ssc_wr()'s case 0x08 touch.  So SSCVAX reads/writes THROUGH `exc`
   (a constructor argument, optional -- a caller that never wires it simply cannot reach this
   register, exactly as an undecoded one would) rather than keeping a second, divergeable copy. */
const SSCBTO_INTV = 0x00FFFFFF;                 // interval, NI (not implemented) -- plain RW storage
const SSCBTO_W1C = (SSCBTO_BTO | SSCBTO_RWT) | 0;
const SSCBTO_RW  = SSCBTO_INTV;

/* SSC base register, vax_sysdev.c:144-145. */
const SSCBASE_MBO = 0x20000000 | 0;             // must-be-one bits
const SSCBASE_RW  = 0x1FFFFC00;                 // the only bits software can actually change

/* SSC configuration register, vax_sysdev.c:149-157 and :1274-1277/1370-1373 (ssc_rd/ssc_wr case
   0x04) -- pcjsvax-bfb's own romdiff boundary-advance, instruction #27 of the ROM's boot-entry
   trace (a READ of SSC+0x10, physical 0x20140010).  SSCCNF_BLO ("battery low", W1C) starts SET:
   vax_sysdev.c's nvr_reset() ORs it in whenever NVR's backing store is freshly allocated
   (`if (nvr == NULL) { ...; ssc_cnf |= SSCCNF_BLO; }`, vax_sysdev.c:679-684) and nvr_attach()
   clears it once a real file is attached -- this project's harnesses never attach a persistent NVR
   file, so every run models the "fresh, no backing file" state, exactly as nvr.js's own NVRVAX
   models a freshly-zeroed array every construction. */
const SSCCNF_BLO = 0x80000000 | 0;
const SSCCNF_W1C = SSCCNF_BLO;
const SSCCNF_RW  = 0x0BF7F777;

/* SSC output port register, vax_sysdev.c:187 (SSCOTP_MASK) and :1282-1283/1381-1382 (ssc_rd/
   ssc_wr case 0x0C) -- pcjsvax-bfb's own romdiff boundary-advance, instruction #3 of the ROM's
   boot-entry trace (a READ of SSC+0x30, PC=2004002F).  No IE/W1C bits: a plain 4-bit RW field. */
const SSCOTP_MASK = 0x0000000F;

/* SSC address-strobe compare registers (ADS0M/ADS0K/ADS1M/ADS1K), vax_sysdev.c:226 (SSCADS_MASK)
   and :1336-1345/1442-1454 (ssc_rd/ssc_wr cases 0x4C/0x4D/0x50/0x51) -- pcjsvax-bfb's own romdiff
   boundary-advance, instruction #29 (a READ of SSC+0x130 = ADS0M).  Deliberately NOT the SAME
   animal as the SSC T0/T1 TIMER registers a few lines below in physical address space (+0x100..
   +0x11C, TMR_CSR/TMR_TNIR/TMR_TIVR, pcjsvax-055): those are genuine counting/interrupt-delivering
   state (see the "SSC PROGRAMMABLE TIMERS" section below), while ADS0M/ADS0K/ADS1M/ADS1K are
   plain masked RW storage with NO counting, NO interrupt, NO side effect of any kind on either the
   read or the write side -- vax_sysdev.c's own bodies are a bare `return ssc_adsm[i]` /
   `ssc_adsm[i] = val & SSCADS_MASK`. Decoding them here does not touch, anticipate, or duplicate
   anything the timer registers need, and this paragraph's decode predates pcjsvax-055 and is
   unchanged by it. */
const SSCADS_MASK = (0x3FFFFFFC | 0);

/*
 * ============================================================================
 * SSC PROGRAMMABLE TIMERS T0/T1 (pcjsvax-055) -- vax_sysdev.c:1508-1642
 * ============================================================================
 * Two independent 32-bit down-counters (stored here, as on real SIMH, as UP-counters that OVERFLOW
 * past 0xFFFFFFFF -- TIR holds the two's-complement of "how far to go", so "counting up by 1" and
 * "counting down by 1" are the same bit pattern) at rg 0x40-0x43 (T0) and 0x44-0x47 (T1):
 * T#CSR (control/status), T#INT (TIR, the live interval register -- READ-ONLY, vax_sysdev.c's
 * ssc_wr() has no `case` for 0x41/0x45 at all), T#NI (TNIR, the RELOAD value XFR copies into TIR),
 * T#VEC (TIVR, the dynamic interrupt vector, masked by TMR_VEC_MASK on WRITE only).
 *
 * TIMING MODEL, DECIDED HERE, mirroring clk.js's own precedent exactly (pcjsvax-954's file header,
 * "TIMING MODEL, DECIDED HERE"): real SIMH's tmr_sched()/tmr_svc() schedule tmr_incr() from the
 * SIMH event queue, using WALL-CLOCK MICROSECONDS -- *except* for one case it carves out itself,
 * vax_sysdev.c:1608-1625 (tmr_sched()): `if (ADDR_IS_ROM(fault_PC) && usecs_sched < TMR_INC)` (a
 * short delay requested from ROM code) schedules by INSTRUCTION COUNT instead
 * (`sim_activate(&sysd_unit[tmr], usecs_sched)`, whose base unit IS instructions, vs.
 * `sim_activate_after_d()`'s explicit real-time form used otherwise) -- SIMH's own comment at
 * vax_sysdev.c:540-543 names this: the ROM's calibration/delay loops need a deterministic,
 * host-speed-independent clock, so SIMH gives them one.  This engine has NO wall-clock concept at
 * all (HANDOFF.md 7: EHKAA's own dispatch counts already vary run to run for exactly that reason),
 * so tick() below always counts INSTRUCTIONS -- which is not a looser approximation of SIMH's
 * general behavior, it is a literal port of the ONE branch of tmr_sched() this project's whole
 * engine is structurally able to reach (real ROM code, run through this emulator, has no OTHER
 * clock to be scheduled against) -- confirmed by testing FROM the ROM address window (tests/
 * tmrdiff.js's run_mode_counting_rom case) so `ADDR_IS_ROM(fault_PC)` is genuinely true on the
 * live oracle too, not merely assumed.  The non-ROM / long-delay wall-clock branch is NOT modelled
 * (no test in this tree runs T0/T1-driven code outside the ROM window) -- exactly the same scope
 * boundary clk.js draws around TODR's wall-clock path vs. its own instruction-driven tick.
 *
 * INTERRUPT SEAM: T0/T1 are DYNAMIC-vector hardware interrupt sources, IPL 0x14 (vaxmod_defs.h's
 * IPL_TMR0/IPL_TMR1 both resolve to `(0x14 - IPL_HMIN)`, i.e. the SAME absolute level as TTI/TTO/
 * CSI/CSO -- IPL_HMIN itself), bits 15/16 (vaxmod_defs.h INT_V_TMR0=15/INT_V_TMR1=16) -- the exact
 * (lvl, bit) pair hwintdiff.js's dynamic_tmr0/dynamic_tmr1/dynamic_tmr0_reprogrammed cases already
 * exercise against a SYNTHETIC prime, proving the seam itself is sound for this shape.  Wired here
 * via addInterruptSource(IPL_HMIN, bit, (cpu) => this.tivr[i]) -- a LIVE closure over `this.tivr`,
 * not a value captured at wiring time, so a vector rewritten after the request is raised but before
 * it is acknowledged still resolves correctly (mirrors ConsoleVAX's own self-wiring-in-constructor
 * precedent, console.js:131-135, rather than clk.js's older external-wiring convention -- self-
 * wiring is followed here because, like ConsoleVAX, this device's vector storage IS `this`, so there
 * is no reason to make a caller thread it through separately).  Installed ONLY when `exc` is
 * provided (SSCVAX's existing optional-dependency contract -- omit it and T0/T1 behave exactly as
 * any other device-less register: readable/writable, but never able to raise a request).
 */
/* TMR_CSR bits, vax_sysdev.c:191-198. */
const TMR_CSR_ERR = 0x80000000 | 0;             // error, W1C
const TMR_CSR_DON = 0x00000080;                 // done, W1C
const TMR_CSR_IE  = 0x00000040;                 // interrupt enable
const TMR_CSR_SGL = 0x00000020;                 // single step, WO
const TMR_CSR_XFR = 0x00000010;                 // transfer TNIR->TIR, WO
const TMR_CSR_STP = 0x00000004;                 // stop-on-overflow
const TMR_CSR_RUN = 0x00000001;                 // run
const TMR_CSR_W1C = (TMR_CSR_ERR | TMR_CSR_DON) | 0;
const TMR_CSR_RW  = (TMR_CSR_IE | TMR_CSR_STP | TMR_CSR_RUN);

/* TMR_VEC_MASK, vax_sysdev.c:222 -- applied to TIVR on WRITE only; QB_VEC_MASK (exc.js) is applied
   AGAIN, generically, to every hardware vector at acknowledge time -- see the doc comment above. */
const TMR_VEC_MASK = 0x000003FC;

/* vaxmod_defs.h:362-363,422-423: T0/T1 share IPL 0x14 (= IPL_HMIN, the same absolute level as
   TTI/TTO/CSI/CSO) and are distinguished purely by request BIT. */
const INT_V_TMR0 = 15;
const INT_V_TMR1 = 16;

/*
 * ============================================================================
 * KNOWN_UNIMPLEMENTED_{READ,WRITE} -- MUST CONTINUE TO FAULT, NOT SILENTLY DEFAULT
 * ============================================================================
 * VERACITY FINDING, SECOND-ORDER CORRECTION (post-merge re-dispatch): the fix that made a truly
 * UNCASED `rg` return 0/silent-success (see readReg()'s/writeReg()'s own doc comments) is ONLY
 * correct for `rg` values vax_sysdev.c's REAL ssc_rd()/ssc_wr() switches ALSO do not case. It is
 * WRONG for an `rg` value that IS cased on real SIMH but this file has simply not ported yet --
 * TODR (0x1B) and the console-STORAGE registers CSRS/CSRD/CSTS/CSTD (0x1C-0x1F, a different device
 * from the console UART this item owns). Those registers have REAL, NON-TRIVIAL behavior on real
 * hardware (a counting timer, a live clock) that returning 0/silently-dropping does NOT reproduce
 * -- and unlike a genuine fault, a silent wrong answer has NO observable signal at all, so nothing
 * would ever catch the ROM proceeding on fabricated timer state.  MEASURED DIRECTLY: applying the
 * blanket "uncased-in-this-file therefore silent" rule to these registers let the JS boot run
 * sail 82,472 instructions past this item's own SSC+0x10C (T0VEC) boundary before diverging into a
 * spurious HALT at PC=0x20044368 -- a real, silent, false-negative regression this set exists to
 * prevent.  A register in either set below is DELIBERATELY excluded from the silent-default path
 * and instead falls through to a bus fault exactly as it did before the fall-through fix, so
 * tests/romdiff.js's boundary-advance keeps naming it as the next thing to implement, precisely as
 * it named SSC+0x10C itself.
 *
 * pcjsvax-055 REMOVED the SSC T0/T1 timer registers (0x40-0x47) from BOTH sets below -- the ONLY
 * item permitted to (see this file's own history: entries are removed ONLY in the same change that
 * implements them, never speculatively -- see the "DRIFT RISK IS ASYMMETRIC" warning that shipped
 * with this item's own rd record).  readReg()/writeReg() now case all eight; see the "SSC
 * PROGRAMMABLE TIMERS" section above.
 *
 * Two sets, not one, because vax_sysdev.c's ssc_rd()/ssc_wr() switches are NOT symmetric --
 * T0INT/T1INT (0x41/0x45) are READ-only status flags with no `case` in ssc_wr() at all (a TRUE
 * gap on the write side even though the SAME offset is real on the read side, exactly like CSRD/
 * CSTD (0x1D/0x1F) below, which are similarly one-directional the other way) -- so 0x41/0x45 were
 * NEVER in KNOWN_UNIMPLEMENTED_WRITE to begin with (a write to them was already a genuine,
 * unconditional no-op, matching real hardware, before this item touched anything) and this item
 * makes NO change to the WRITE set for those two numbers.  Each set is transcribed directly from
 * vax_sysdev.c's own case labels (:1264-1345 read, :1352-1458 write), not inferred from the other.
 */
const KNOWN_UNIMPLEMENTED_READ = new Set([
    0x1B,                               // TODR -- owned by clk.js's IPR path, not mirrored here
    0x1C, 0x1D, 0x1E                    // CSRS/CSRD/CSTS -- console STORAGE, a different device
]);
const KNOWN_UNIMPLEMENTED_WRITE = new Set([
    0x1B,                               // TODR
    0x1C, 0x1E, 0x1F                    // CSRS/CSTS/CSTD (CSRD has NO write case -- a true gap)
]);

/**
 * @class SSCVAX
 */
export default class SSCVAX {
    /**
     * SSCVAX(exc, console)
     *
     * @param {Object} [exc] the owning CPU's VAXExc -- REG_BTO reads/writes cpu.exc.sscBto through
     *   it (see the SSCBTO_* doc comment above); omit it and SSC+0x08 falls through to a bus fault
     *   exactly like any other undecoded register.
     * @param {Object} [console] pcjsvax-bfb's ConsoleVAX (console.js) -- rg 0x20/0x21/0x22/0x23
     *   (RXCS/RXDB/TXCS/TXDB) delegate to its sscRead()/sscWrite(), the SAME register state exc.js's
     *   setIPRDevice() seam exposes -- see console.js's file header.  Omit it and those four
     *   registers fall through to a bus fault exactly like any other undecoded register.
     */
    constructor(exc, console)
    {
        this.exc = exc || null;
        this.console = console || null;
        this.reset();
        /*
         * pcjsvax-055: self-wire T0/T1's dynamic-vector interrupt sources, mirroring ConsoleVAX's
         * own self-wiring-in-constructor precedent (console.js:131-135) rather than clk.js's older
         * external-wiring convention -- see the "SSC PROGRAMMABLE TIMERS" doc comment above for why.
         * `(cpu) => this.tivr[i]` is a LIVE closure, resolved at ACKNOWLEDGE time by exc.js's
         * deviceVector() (not memoized here) -- required for dynamic_tmr0_reprogrammed-shaped
         * correctness (hwintdiff.js already proves the seam itself honours this; this closure is
         * what makes SSCVAX's OWN storage honour it too).  Skipped entirely when `exc` is omitted,
         * matching every other exc-dependent register above (REG_BTO): T0/T1 stay readable/
         * writable but can never raise a request.
         */
        if (this.exc) {
            this.exc.addInterruptSource(IPL_HMIN, INT_V_TMR0, (cpu) => this.tivr[0]);
            this.exc.addInterruptSource(IPL_HMIN, INT_V_TMR1, (cpu) => this.tivr[1]);
        }
    }

    /**
     * reset()
     *
     * CORRECTED (pcjsvax-320 veracity re-dispatch): this models `sysd_powerup()`
     * (vax_sysdev.c:1787, `ssc_base = SSCBASE`), NOT `sysd_reset()` -- MEASURED directly, SIMH's
     * own `reset all` does NOT touch `ssc_base` at all (it is untouched by sysd_reset(), the same
     * way pcjsvax-446 already found `ssc_bto` untouched by it).  Harmless today because this
     * class's reset() is called ONLY from the constructor -- every test builds a fresh SSCVAX per
     * machine, so "the value a brand-new instance starts with" and "the value after a cold power-up"
     * are the same question here -- and nothing wires it to BusVAX.addResetHandler().  It would
     * NOT be harmless if a later item did that: calling this reset() in response to a plain
     * `reset all`-equivalent would incorrectly re-zero state SIMH's own reset leaves alone, exactly
     * the exc.js/sscBto divergence class pcjsvax-446 found and fixed.  Whoever wires a reset handler
     * here must first decide whether it should fire at all for a non-power-cycle reset.
     *
     * @this {SSCVAX}
     */
    reset()
    {
        this.base = SSC_BASE | 0;
        this.otp = 0;                          // vax_sysdev.c:1790, sysd_powerup(): ssc_otp = 0
        this.cnf = SSCCNF_BLO;                 // vax_sysdev.c:684, nvr_reset() -- see REG_CNF's doc
        this.adsm = [0, 0];                    // vax_sysdev.c:253, ssc_adsm[2] = { 0 }
        this.adsk = [0, 0];                    // vax_sysdev.c:254, ssc_adsk[2] = { 0 }
        /*
         * pcjsvax-055: T0/T1 timer state.  vax_sysdev.c's sysd_powerup() (:1776-1792) zeroes ONLY
         * tmr_tivr[] (the vector) -- tmr_csr[]/tmr_tir[]/tmr_tnir[] are NOT touched by powerup at
         * all; they are zeroed by sysd_reset() instead (:1748-1753, a plain `reset all`).  This
         * class's reset() is documented above as modelling powerup, not reset (see that doc comment)
         * -- but since reset() here runs ONLY from the constructor (a brand-new instance), and a
         * brand-new instance's csr/tir/tnir are already 0 with no other write path to make them
         * anything else, "the powerup value" and "the reset value" are the SAME zero for these
         * three arrays too, exactly as that doc comment already establishes for the rest of this
         * class's state.  tivr[] is zeroed here because powerup actually does zero it (not merely
         * by coincidence of construction).
         */
        this.tcsr = [0, 0];                    // tmr_csr[2]
        this.tir  = [0, 0];                    // tmr_tir[2]  (the live TIR; see readReg's T#INT)
        this.tnir = [0, 0];                    // tmr_tnir[2] (the XFR reload source)
        this.tivr = [0, 0];                    // tmr_tivr[2] -- vax_sysdev.c:1783, sysd_powerup()
    }

    /**
     * tick(cpu)
     *
     * The per-instruction device-service hook (cpustate.js's `cpu.tmr` slot, wired the same way
     * `cpu.clk` wires ClkVAX -- see cpustate.js's own doc comment on both).  See the "SSC
     * PROGRAMMABLE TIMERS" file-header section for why counting INSTRUCTIONS, unconditionally, is
     * the correct port of tmr_sched()'s ROM-address/short-delay branch rather than an approximation
     * of it.  `cpu` is accepted (matching ClkVAX.tick()'s own signature) but unused: T0/T1 need
     * nothing from the CPU beyond "one instruction retired," and raise their own request through
     * `this.exc`, already bound at construction.
     *
     * @this {SSCVAX}
     * @param {Object} cpu
     */
    tick(cpu)
    {
        if (this.tcsr[0] & TMR_CSR_RUN) this._tmrIncr(0, 1);
        if (this.tcsr[1] & TMR_CSR_RUN) this._tmrIncr(1, 1);
    }

    /**
     * _tmrIncr(tmr, inc)
     *
     * vax_sysdev.c:1575-1600, tmr_incr() -- ported literally, including the "ERR only after a
     * SECOND unacknowledged overflow" DON/ERR asymmetry and the RUN-gated reload-and-continue.
     * Unsigned overflow detection (`next < cur`) is done on VALUES ALREADY NORMALIZED via `>>> 0`
     * -- see defines.js's rule that a signed int32 relational compare is the hazard, not the mask;
     * both operands here are plain non-negative JS numbers by the time they are compared, so `<`
     * is safe.  `inc` is always 1 in this file (tick()'s per-instruction call and writeReg()'s SGL
     * synchronous call) -- vax_sysdev.c's own callers never pass anything else either (tmr_svc()'s
     * `~tmr_tir[tmr]+1` is a WALL-CLOCK usec delta this file's instruction-driven model has no
     * equivalent for; see the file header) -- so multi-wrap-in-one-call is not a case this needs to
     * handle, and does not.
     *
     * @this {SSCVAX}
     * @param {number} tmr 0 or 1
     * @param {number} inc
     */
    _tmrIncr(tmr, inc)
    {
        let cur = this.tir[tmr] >>> 0;
        let next = (cur + inc) >>> 0;
        if (next < cur) {                                  // overflow (wrapped past 0xFFFFFFFF)
            this.tir[tmr] = 0;
            if (this.tcsr[tmr] & TMR_CSR_DON) this.tcsr[tmr] = (this.tcsr[tmr] | TMR_CSR_ERR) | 0;
            else this.tcsr[tmr] = (this.tcsr[tmr] | TMR_CSR_DON) | 0;
            if (this.tcsr[tmr] & TMR_CSR_STP) this.tcsr[tmr] = (this.tcsr[tmr] & ~TMR_CSR_RUN) | 0;
            if (this.tcsr[tmr] & TMR_CSR_RUN) this.tir[tmr] = this.tnir[tmr] | 0;   // reload, continue
            if ((this.tcsr[tmr] & TMR_CSR_IE) && this.exc) {
                this.exc.raiseInterrupt(IPL_HMIN, tmr ? INT_V_TMR1 : INT_V_TMR0);
            }
        } else {
            this.tir[tmr] = next | 0;
        }
    }

    /**
     * _tmrCsrWr(tmr, val)
     *
     * vax_sysdev.c:1515-1554, tmr_csr_wr() -- ported in SIMH's own order (the order is observable:
     * XFR's TNIR->TIR copy happens BEFORE the RUN/SGL branch consults TIR, and the CLR_INT check at
     * the end reads the POST-write csr against the PRE-write `before`).  `sim_cancel`/`tmr_sched`'s
     * real-time scheduling calls have no equivalent here -- see _tmrIncr()'s doc comment -- RUN
     * alone gates tick(), so there is nothing to "cancel" or "(re)activate": clearing RUN silently
     * stops future tick()s from calling _tmrIncr(), and setting it silently resumes them.
     *
     * @this {SSCVAX}
     * @param {number} tmr 0 or 1
     * @param {number} val the FULL merged longword writeReg() received (see writeReg()'s own
     *   byte/word pre-merge doc comment)
     */
    _tmrCsrWr(tmr, val)
    {
        let before = this.tcsr[tmr];
        this.tcsr[tmr] = (this.tcsr[tmr] & ~(val & TMR_CSR_W1C)) | 0;         // W1C ERR/DON
        this.tcsr[tmr] = ((this.tcsr[tmr] & ~TMR_CSR_RW) | (val & TMR_CSR_RW)) | 0;   // IE/STP/RUN
        if (val & TMR_CSR_XFR) this.tir[tmr] = this.tnir[tmr] | 0;
        if (val & TMR_CSR_RUN) {
            /* real SIMH cancels/reschedules its event here; nothing to do -- see the doc comment */
        } else if (val & TMR_CSR_SGL) {
            this._tmrIncr(tmr, 1);
            if (this.tir[tmr] === 0) this.tir[tmr] = this.tnir[tmr] | 0;      // vax_sysdev.c:1549-1550
        }
        if ((before & (TMR_CSR_DON | TMR_CSR_IE)) !== 0 &&
            (this.tcsr[tmr] & (TMR_CSR_DON | TMR_CSR_IE)) === 0) {
            if (this.exc) this.exc.clearInterrupt(IPL_HMIN, tmr ? INT_V_TMR1 : INT_V_TMR0);
        }
    }

    /**
     * readReg(rg)
     *
     * vax_sysdev.c:1264, ssc_rd() -- the register switch only, no byte/word merge (see readLong()/
     * readWord()/readByte() below for that; SIMH's ssc_rd() itself is also merge-free, called with
     * the FULL aligned longword's value expected back, per vax_mmu.h's ReadB/ReadW/ReadL).
     *
     * VERACITY FINDING (pcjsvax-bfb re-dispatch): the switch's own `default` -- reached by any `rg`
     * in [0, SSC_LENGTH>>2) this file does not case -- returns 0, matching vax_sysdev.c's ssc_rd()
     * literally: `switch (rg) { ...cases... } return 0;` has NO `default:` label and there is
     * nothing after the switch but that one `return 0`.  Measured live: SSC+0x50 (rg=0x14, uncased
     * AND genuinely absent from vax_sysdev.c's own switch) reads 0 on the real oracle with NO
     * machine check.  An EARLIER version of this function returned `null` here unconditionally,
     * which makeSscController() below (correctly, at the time) mapped to a bus fault -- exactly
     * backwards for a TRUE gap.
     *
     * SECOND-ORDER CORRECTION: that fix, applied BLINDLY to every uncased `rg`, is wrong for the
     * subset that IS cased on real SIMH but this file has not ported yet -- see
     * KNOWN_UNIMPLEMENTED_READ's own doc comment for the measured regression (82,472 instructions
     * of silent divergence) that taught this.  Those specific `rg` values still return `null`
     * (fault) below, exactly as an undecoded register did before either fix; every OTHER uncased
     * `rg` (a genuine, permanent hardware gap) returns 0.
     *
     * @this {SSCVAX}
     * @param {number} rg
     * @returns {?number} the register's value; 0 for a genuine hardware gap; null (fault) for a
     *   register KNOWN_UNIMPLEMENTED_READ names -- see that constant's doc comment.
     */
    readReg(rg)
    {
        switch (rg) {
        case REG_BASE:
            return this.base;
        case REG_CNF:
            return this.cnf;
        case REG_BTO:
            return this.exc ? this.exc.sscBto : 0;
        case REG_OTP:
            return this.otp & SSCOTP_MASK;
        case REG_ADS0M: return this.adsm[0];
        case REG_ADS0K: return this.adsk[0];
        case REG_ADS1M: return this.adsm[1];
        case REG_ADS1K: return this.adsk[1];
        /*
         * pcjsvax-bfb: RXCS/RXDB/TXCS delegate to the console device -- see console.js's sscRead()
         * doc comment.  TXDB (0x23) has NO case at all in vax_sysdev.c's REAL ssc_rd() switch (only
         * ssc_wr() lists it) -- the implicit `return 0;` at the end of that switch applies
         * UNCONDITIONALLY, matching this function's own default (see above), so this returns 0
         * directly rather than asking the console device (which has no sscRead() case for it
         * either, by the same reasoning -- see that function's own doc comment).  If `this.console`
         * is not wired (a test harness that does not care about the UART), 0 is the same "not
         * specifically handled, but not a fault either" default every other uncased register in
         * this span gets -- not a special case.
         */
        case REG_RXCS: case REG_RXDB: case REG_TXCS:
            return this.console ? this.console.sscRead(rg) : 0;
        case REG_TXDB:
            return 0;
        /*
         * pcjsvax-055: T#INT (TIR) is the LIVE interval register.  vax_sysdev.c's tmr_tir_rd()
         * INTERPOLATES this from the SIMH event queue while running (`sim_activate_time(...)`,
         * vax_sysdev.c:1485-1499); this file has no lazy/scheduled state to interpolate FROM --
         * tick() advances `this.tir[i]` eagerly, every instruction, so the stored value already
         * IS the live one and no interpolation step is needed to reproduce the same read.
         */
        case REG_T0CSR: return this.tcsr[0];
        case REG_T0INT: return this.tir[0];
        case REG_T0NI:  return this.tnir[0];
        case REG_T0VEC: return this.tivr[0];
        case REG_T1CSR: return this.tcsr[1];
        case REG_T1INT: return this.tir[1];
        case REG_T1NI:  return this.tnir[1];
        case REG_T1VEC: return this.tivr[1];
        }
        return KNOWN_UNIMPLEMENTED_READ.has(rg) ? null : 0;
    }

    /**
     * writeReg(rg, val)
     *
     * vax_sysdev.c:1352, ssc_wr() -- the register switch only.  `val` is the FULL merged longword
     * (see writeLong()/writeWord()/writeByte() below for the byte/word pre-merge SIMH's ssc_wr()
     * does inline via `t = ssc_rd(pa)` before this switch ever runs).
     *
     * VERACITY FINDING (pcjsvax-bfb re-dispatch): same defect as readReg() above, mirrored on the
     * write side -- ssc_wr()'s switch also has no `default:` and nothing follows it, so an uncased
     * `rg` is a SILENT NO-OP on real hardware (PC advances, nothing stored), never a fault.
     *
     * SECOND-ORDER CORRECTION: as with readReg(), that fallback is `true` (silent) ONLY for a
     * genuine hardware gap -- KNOWN_UNIMPLEMENTED_WRITE names the `rg` values that ARE cased on
     * real SIMH but this file has not ported (see that constant's doc comment for the measured
     * regression this closes); those still return `false` (fault).  SSCVAX has NO register that is
     * BOTH implemented here AND genuinely machine-checks on write (unlike CQBICVAX's MEAR/SEAR,
     * cqbic.js) -- `false` from this function means EITHER "cqbic-style deliberate fault" (does not
     * occur in this class) OR "known-unimplemented, not yet safe to default", never "uncased and
     * harmless".
     *
     * @this {SSCVAX}
     * @param {number} rg
     * @param {number} val
     * @returns {boolean} true for a handled write (cased, or a genuine hardware gap); false for a
     *   KNOWN_UNIMPLEMENTED_WRITE register -- see that constant's doc comment.
     */
    writeReg(rg, val)
    {
        switch (rg) {
        case REG_BASE:
            this.base = ((val & SSCBASE_RW) | SSCBASE_MBO) | 0;
            return true;
        case REG_CNF:
            this.cnf = (this.cnf & ~(val & SSCCNF_W1C)) | 0;
            this.cnf = ((this.cnf & ~SSCCNF_RW) | (val & SSCCNF_RW)) | 0;
            return true;
        case REG_BTO:
            if (!this.exc) return true;
            this.exc.sscBto = (this.exc.sscBto & ~(val & SSCBTO_W1C)) | 0;
            this.exc.sscBto = ((this.exc.sscBto & ~SSCBTO_RW) | (val & SSCBTO_RW)) | 0;
            return true;
        case REG_OTP:
            this.otp = val & SSCOTP_MASK;
            return true;
        case REG_ADS0M: this.adsm[0] = (val & SSCADS_MASK) | 0; return true;
        case REG_ADS0K: this.adsk[0] = (val & SSCADS_MASK) | 0; return true;
        case REG_ADS1M: this.adsm[1] = (val & SSCADS_MASK) | 0; return true;
        case REG_ADS1K: this.adsk[1] = (val & SSCADS_MASK) | 0; return true;
        /*
         * pcjsvax-bfb: RXCS/TXCS/TXDB delegate to the console device.  RXDB (0x21) has NO case in
         * vax_sysdev.c's REAL ssc_wr() switch (matching WriteIPR()'s own `case MT_RXDB: break;` --
         * input is read-only from software on both address paths) -- a silent no-op, UNCONDITIONALLY
         * (see REG_TXDB's read-side comment above for the identical reasoning in the other
         * direction).  `this.console` missing gets the SAME silent-success default every other
         * uncased register in this span gets, not a fault.
         */
        case REG_RXCS: case REG_TXCS: case REG_TXDB:
            return this.console ? this.console.sscWrite(rg, val) : true;
        case REG_RXDB:
            return true;
        /*
         * pcjsvax-055.  T0INT/T1INT (0x41/0x45) are DELIBERATELY ABSENT from this switch -- see
         * KNOWN_UNIMPLEMENTED_WRITE's own doc comment: vax_sysdev.c's ssc_wr() has no `case` for
         * them either, so a write silently no-ops (the trailing `return true` below), matching real
         * hardware exactly.  This is also what self-resolves the byte/word write fault pcjsvax-bfb
         * disclosed for T0INT/T1INT: writeWord()/writeByte()'s read-modify-merge now gets a REAL
         * `cur` from readReg() (T0INT/T1INT are cased on read) instead of null, so the merge
         * succeeds and lands here -- a genuine no-op, not a fault.
         */
        case REG_T0CSR: this._tmrCsrWr(0, val); return true;
        case REG_T0NI:  this.tnir[0] = val | 0; return true;
        case REG_T0VEC: this.tivr[0] = (val & TMR_VEC_MASK) | 0; return true;
        case REG_T1CSR: this._tmrCsrWr(1, val); return true;
        case REG_T1NI:  this.tnir[1] = val | 0; return true;
        case REG_T1VEC: this.tivr[1] = (val & TMR_VEC_MASK) | 0; return true;
        }
        return KNOWN_UNIMPLEMENTED_WRITE.has(rg) ? false : true;
    }

    /**
     * readLong(addr) / readWord(addr) / readByte(addr)
     *
     * `addr` is the PHYSICAL address (not block-relative), already known by the caller to lie
     * within [SSC_BASE, SSC_BASE + SSC_LENGTH) -- makeSscController() below is what enforces that;
     * these three assume it.  `rg` truncates any misalignment to the containing register, exactly
     * as vax_sysdev.c's ssc_rd()/ssc_wr() do.
     *
     * pcjsvax-855: these three are called ONLY with an ALREADY-ALIGNED `addr` (word: bo 0 or 2;
     * longword: bo 0) -- mmu.js's readData()/readU() (an exact port of vax_mmu.h's Read()) resolve
     * EVERY unaligned CPU-level access (word bo=1/3, any unaligned longword) into a SEQUENCE of
     * aligned bus.getLong() calls, extracting/merging the sub-longword fragment entirely within
     * mmu.js, BEFORE this switch ever runs -- see the file header's "UNALIGNED ACCESS SPANNING TWO
     * ADJACENT REGISTER-SPACE LONGWORDS" section for the measurement that proved this (an earlier
     * revision added a two-register stitch here; a real-CPU-instruction differential showed it was
     * unreachable dead code and it was removed rather than shipped).
     *
     * @this {SSCVAX}
     * @param {number} addr
     * @returns {?number} null propagates a KNOWN_UNIMPLEMENTED_READ fault -- see readReg()'s doc
     *   comment; the caller (makeSscController() below) treats that exactly like an undecoded
     *   register always has.
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
     * pcjsvax-855: as with the read side above, these are called ONLY at an already-aligned `addr`
     * -- mmu.js's writeData()/writeU() do the unaligned-fragment merge themselves, one aligned
     * bus.setLong() at a time, before this switch ever runs.  See readLong()'s doc comment.
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

    /*
     * writeWord()/writeByte()'s read-modify-merge needs `cur` from readReg() FIRST (see the doc
     * comment above), which can itself return null for a KNOWN_UNIMPLEMENTED_READ register (e.g.
     * T0INT, 0x41) even on a register whose WRITE side is a genuine, harmless no-op (T0INT is
     * read-only on real hardware, so a longword write to it -- which skips this merge entirely --
     * already correctly no-ops via writeReg()'s own trailing default).  A byte/word write to such a
     * register therefore faults here rather than silently guessing what "current value" to merge
     * against -- a known, narrow, disclosed asymmetry (longword vs. byte/word write to the same
     * KNOWN_UNIMPLEMENTED-on-read-only register), not a gap in the fall-through fix itself: no
     * measured ROM access has ever needed a sub-longword write to a register in that state.
     */
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
 * makeSscController(ssc, nvr)
 *
 * A MemoryVAX controller (see bus.js's makeRomAliasController() for the same pattern applied to
 * the ROM mirror): `getControllerBuffer()` supplies no backing array (every access is computed,
 * never stored in an Int32Array), and `getControllerAccess()` returns six functions that are
 * installed as an ordinary MemoryVAX block's readByte/writeByte/readWord/writeWord/readLong/
 * writeLong -- called as `block.readByte(off, addr)`, so `this` inside them is the MemoryVAX
 * block, which is what makes `this.readNone`/`this.writeNone`/... (inherited from
 * MemoryVAX.prototype) the right fallback: the EXACT function the shared empty block would have
 * used for this address, before this controller existed.
 *
 * `nvr` (pcjsvax-bfb) is a SECOND device, NVRVAX (see nvr.js), checked whenever `ssc` does not
 * claim the address -- vax_sysdev.c installs nvr_rd()/nvr_wr() as a SEPARATE regtable entry from
 * ssc_rd()/ssc_wr() (vax_sysdev.c:1004 vs :1006), but both land in the SAME physical 8KB bus block
 * this controller answers for (SSC_BASE=0x20140000, NVR_BASE=0x20140400, PCjs block size 0x2000),
 * so one controller has to route to both.  `nvr` is optional (existing callers -- and any test that
 * only cares about the SSC base register -- pass none, and NVR then still falls through to
 * readNone/writeNone exactly as it always did before this item).  Anything neither `ssc` nor `nvr`
 * claims -- undecoded SSC sub-registers and the block's unused tail -- keeps faulting exactly as it
 * did before this method existed, address-by-address rather than by block granularity; that fault
 * is what tests/romdiff.js's FaultGrader.verdict() grades against the live oracle to find and NAME
 * the next boundary (pcjsvax-fe7: a fault the oracle produces identically is walked past instead).
 *
 * @param {SSCVAX} ssc
 * @param {NVRVAX} [nvr]
 * @returns {Object}
 */
export function makeSscController(ssc, nvr)
{
    const LOW = SSC_BASE, HIGH = (SSC_BASE + SSC_LENGTH) >>> 0;
    function inRange(addr) { return addr >= LOW && addr < HIGH; }
    const NVR_LOW = nvr ? NVR_BASE : 0, NVR_HIGH = nvr ? (NVR_BASE + NVR_LENGTH) >>> 0 : 0;
    function nvrInRange(addr) { return !!nvr && addr >= NVR_LOW && addr < NVR_HIGH; }

    return {
        getControllerBuffer(addr) { return [null, 0]; },
        getControllerAccess() {
            return [
                function readByte(off, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr)) {
                        let v = ssc.readByte(addr);
                        if (v !== null) return v;
                    } else if (nvrInRange(addr)) {
                        let v = nvr.readByte(addr);
                        if (v !== null) return v;
                    }
                    return this.readNone(off, addr);
                },
                function writeByte(off, b, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr) && ssc.writeByte(addr, b)) return;
                    if (nvrInRange(addr) && nvr.writeByte(addr, b)) return;
                    this.writeNone(off, b, addr);
                },
                function readWord(off, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr)) {
                        let v = ssc.readWord(addr);
                        if (v !== null) return v;
                    } else if (nvrInRange(addr)) {
                        let v = nvr.readWord(addr);
                        if (v !== null) return v;
                    }
                    return this.readWordNone(off, addr);
                },
                function writeWord(off, w, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr) && ssc.writeWord(addr, w)) return;
                    if (nvrInRange(addr) && nvr.writeWord(addr, w)) return;
                    this.writeNone(off, w, addr);
                },
                function readLong(off, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr)) {
                        let v = ssc.readLong(addr);
                        if (v !== null) return v;
                    } else if (nvrInRange(addr)) {
                        let v = nvr.readLong(addr);
                        if (v !== null) return v;
                    }
                    return this.readLongNone(off, addr);
                },
                function writeLong(off, l, addr) {
                    addr = addr >>> 0;
                    if (inRange(addr) && ssc.writeLong(addr, l)) return;
                    if (nvrInRange(addr) && nvr.writeLong(addr, l)) return;
                    this.writeNone(off, l, addr);
                }
            ];
        }
    };
}

export {
    SSC_BASE, SSC_LENGTH,
    REG_T0CSR, REG_T0INT, REG_T0NI, REG_T0VEC, REG_T1CSR, REG_T1INT, REG_T1NI, REG_T1VEC,
    TMR_CSR_ERR, TMR_CSR_DON, TMR_CSR_IE, TMR_CSR_SGL, TMR_CSR_XFR, TMR_CSR_STP, TMR_CSR_RUN,
    TMR_VEC_MASK, INT_V_TMR0, INT_V_TMR1
};
