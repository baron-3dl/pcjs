/**
 * @fileoverview Implements the RQDX3 (MSCP/UQSSP) controller: init handshake, rings, unit commands
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the Open SIMH PDP-11/VAX simulators, Copyright © 2002-2020 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * ============================================================================
 * WHAT THIS IS, AND WHAT IT IS NOT
 * ============================================================================
 * pcjsvax-c2c (the UQSSP initialisation state machine), pcjsvax-0b4 (the command/response RING
 * MACHINERY on top of it) and pcjsvax-f52 (UNITS: a user-supplied image attached to a drive, and the
 * seven unit-bearing MSCP commands that need no data transfer), the first three of pcjsvax-6a5's
 * children.  `open-simh/PDP11/pdp11_rq.c` is ~3,600 lines; THIS FILE IS:
 *   rq_rd(), rq_wr(), rq_reset(), rq_step4(), rq_fatal(), rq_init_int(), rq_ring_int();
 *   BOTH branches of rq_quesvc();
 *   rq_deqf/rq_deqh/rq_enqh/rq_enqt, rq_getpkt/rq_putpkt, rq_getdesc/rq_putdesc, rq_putr;
 *   rq_mscp()'s dispatch, rq_scc(), the OP_CCD/OP_DAP/OP_FLU no-ops and the illegal-opcode default;
 *   drv_tab[], rq_getucb(), rq_putr_unit(), rq_setf_unit(), RQ_WPH/RQ_RMV;
 *   rq_set_type()/rq_set_wlk()/rq_attach()/rq_detach()'s AUTOSIZE arithmetic (sim_disk.c);
 *   rq_abo(), rq_avl(), rq_fmt(), rq_gcs(), rq_gus(), rq_onl(), rq_suc().
 * DISK I/O (rq_rw/rq_svc and the five transfer opcodes), attention/unit-available messages and
 * interrupt DELIVERY are NOT here; each is named in the EXCLUSIONS section below with the fence that
 * keeps a graded case from reaching it.
 *
 * TWO 16-BIT REGISTERS, NOT FOUR.  `IOLN_RQ` is 004 (pdp11_rq.c:1212) and rq_rd()/rq_wr() decode
 * `(PA >> 1) & 01`: IP at +0, SA at +2.  Measured on the live oracle, `SHOW QBA IOSPACE` prints
 * `20001468 - 2000146B ... RQ`.  pcjsvax-6a5's parent sketch says "the controller's four Qbus
 * registers"; that is wrong and this file is the correction.
 *
 * READING IP ALWAYS RETURNS ZERO AND ALWAYS HAS SIDE EFFECTS.  `*data = 0;` is unconditional, and
 * then the state machine branches on `csta`: in CST_S3_PPB the read COMPLETES step 4 synchronously,
 * and in CST_UP it starts a host poll.  WRITING IP with any value at all calls rq_reset().  A model
 * that treats IP as a read-only zero register passes any test that only looks at values.
 *
 * ============================================================================
 * THE HANDSHAKE GOES THROUGH THE CQBIC SCATTER-GATHER MAP -- PROVEN BY MAKING IT FAIL
 * ============================================================================
 * rq_step4() zeroes the communications region with `Map_WriteW (base, lnt, zero)` -- a DMA, not a
 * memory store.  Measured on the live oracle with MBR and the map entries left unprogrammed, step 3
 * goes fatal: SA = 0x8007, CSTA = 8 (CST_DEAD), PERR = 007 (PE_QWE), COMM back to 0.  So
 * pcjsvax-e22's DMA path is load-bearing for the PROTOCOL and not merely for block data, and this
 * file CONSUMES cqbic.js's mapWriteW()/mapReadW() rather than carrying a second translator.  With
 * comm = 0x2000 and a one-entry response ring the zeroed extent is 0x1FFC..0x2007, which already
 * straddles a 512-byte Qbus page boundary -- the handshake alone exercises scatter-gather.
 *
 * ============================================================================
 * THE EVENT MODEL: WHY THERE IS A `tick()` AND WHAT ITS UNITS ARE
 * ============================================================================
 * rq_wr()'s SA case does not answer; it calls `sim_activate (units + RQ_QUEUE, rq_itime)` and the
 * answer appears rq_itime INSTRUCTIONS later.  SIMH's event queue is denominated in `sim_interval`,
 * which vax_cpu.c:729 decrements by `1 + (extra_bytes >> 5)` per instruction -- exactly what
 * cpustate.js's stepCPU() charges as `nCycles` -- and vax_cpu.c:659 tests it at the TOP of the
 * instruction loop, which is exactly where cpustate.js calls its device hooks.  So `tick(cpu)`
 * placed on such a hook and comparing `cpu.nTotalCycles` against a deadline is not an approximation
 * of the C; it is the same test at the same point in the same loop.
 *
 * MEASURED, and the reason the delay is graded IN BAND rather than by halting early: the HALT
 * INSTRUCTION ITSELF drains SIMH's event queue (vax_cpu.c:2643 -- see exc.js's HALT handler, which
 * carries the measurement), so a differential that halts before a scheduled answer observes the
 * state AFTER it and passes while being structurally blind.  What IS observable is a host polling
 * loop that counts its own iterations: five instructions per iteration returned **90** iterations
 * against ITIME = 450 and **2** against I4TIME = 10 on the live oracle -- the smallest i with
 * 1 + 5i >= delay in both cases, which is what this file's deadline arithmetic produces.
 * tests/mscpinitdiff.js grades that ITERATION COUNT, not just the value it eventually sees, and
 * pins ITIME/I4TIME/QTIME/XTIME on the oracle rather than inheriting them.
 *
 * `sim_activate()` on an ALREADY-ACTIVE unit is a no-op (`if (sim_is_active (uptr)) return
 * SCPE_OK;`, scp.c) -- so a second SA write before the first one's service does NOT re-arm the
 * timer.  activateQueue() below reproduces that, and it is graded: two SA writes in quick
 * succession leave the second one's answer arriving on the FIRST one's schedule.
 *
 * ============================================================================
 * THE RESPONSE RING COMES FIRST IN MEMORY
 * ============================================================================
 * rq_step4() does `cp->rq.ba = cp->comm; cp->cq.ba = cp->comm + cp->rq.lnt;` -- `rq` is the
 * RESPONSE ring and it is laid down at the base, with the COMMAND ring above it.  That is the
 * opposite of the order the names suggest and of the order the host writes them in, and it is the
 * single easiest thing to get backwards in this file.  Measured on the live oracle: with a
 * one-descriptor ring pair at comm = 0x2000, `SHOW RQ RINGS` prints `Command ring, base = 2004`
 * and `Response ring, base = 2000`.
 *
 * RING INDEX ARITHMETIC IS A MASK, NOT A MODULO.  `ring->idx = (ring->idx + 4) & (ring->lnt - 1)`
 * and the previous slot is `(ring->idx - 4) & (ring->lnt - 1)` -- correct only because `lnt` is
 * `4 << code`, a power of two.  Note `idx - 4` is computed in uint32 in the C and therefore
 * borrows to 0xFFFFFFFC at idx 0; JS's int32 `&` reproduces that exactly, which is why there is no
 * `>>> 0` in front of it.
 *
 * ============================================================================
 * UNITS, DRIVES AND THE IMAGE PROVIDER -- pcjsvax-f52
 * ============================================================================
 * *** THE DEVICE DECLARES `RQ_MAXDR + 2` UNITS AND THE LAST TWO ARE NOT DRIVES. ***  Index
 * RQ_MAXDR is RQ_TIMER (rq_tmrsvc's wall-clock unit) and RQ_MAXDR+1 is RQ_QUEUE (the queue thread);
 * both are UNIT_DIS and neither is a disk.  That is why rq_getucb() and rq_tmrsvc() both loop over
 * `dptr->numunits - 2`.  This file does not have the trap available to fall into: `units` below
 * holds ONLY drives, RQ_QUEUE is modelled as the `queDue` deadline and RQ_TIMER is not modelled at
 * all, so there is nothing for a unit walk to run off the end of.  The constructor asserts the
 * array's length against RQ_NUMDR so that a future controller with more drives cannot quietly
 * acquire two pseudo-drives instead.
 *
 * THE DRIVE TABLE IS DERIVED, NOT TRANSCRIBED BY HAND.  drv_tab[] has THIRTY-FOUR entries of
 * fourteen fields, built by a token-pasting macro out of ~480 #defines.  DRV_TAB below was
 * GENERATED from pdp11_rq.c by tests/mscpscope.js's extractor and tests/mscpscope.js re-derives and
 * re-compares it, field for field and IN ORDER, on every differential run (HANDOFF.md standing rule
 * 5).  Two traps it removes: `RQDF_DSSI` is `010`, which is EIGHT and not ten -- a leading zero is
 * octal in C and decimal in parseInt() -- and thirty-one `_TPG` values are `#define X_TPG X_SURF`
 * rather than literals.
 *
 * THE IMAGE ARRIVES THROUGH AN INJECTABLE PROVIDER, NEVER THROUGH `fs`.  attach() takes an object
 * with a `byteLength` and a `read(offset, length, dst)`; the Node differential wraps `fs` and a
 * browser wraps a `File`, and THIS FILE IMPORTS NEITHER.  That is what keeps the user-supplied-file
 * decision (HANDOFF.md 8 -- no OpenVMS media is shipped, the user brings the image) implementable
 * in a browser at all.  attach() VALIDATES the contract, because a provider that silently lacks
 * `read` would work perfectly here (nothing in this item reads a block) and fail in pcjsvax-346.
 *
 * AUTOSIZE IS sim_disk.c's, AND MEASUREMENT OVERTURNED THE OBVIOUS READING OF IT.  With autosize on
 * -- the RQ default, `show rq0` prints it -- and a RAW container with no recognisable file system,
 * sim_disk_attach_ex2() does NOT walk drv_types[] and does NOT change the drive type.  It takes the
 * "autosize by changing CAPACITY" arm: `if (container < current && !READ-ONLY) container = current`,
 * then `capac = container / 512`.  Measured on the live oracle: a 409,600-byte file, a 20,000,000-
 * byte file and a 159,334,400-byte file attached to RQ0 ALL leave it `RD54 ... 159MB`, and a
 * 200,000,300-byte file leaves it `RD54 ... 200MB` with capac 390,625 and still WRITE ENABLED.  The
 * drive-type walk lives in the `dontchangecapac` (i.e. NOAUTOSIZE) arm and additionally requires
 * get_filesystem_size() to have recognised a file system, which a pattern-filled scratch image does
 * not have.  So the drive TYPE comes from `set rqN <type>` and the drive SIZE from the file, and
 * both are graded.  (store_disk_footer() would have written SIMH metadata into the container and
 * made the type sticky -- it opens with an unconditional `return SCPE_OK;` in this vendor tree, so
 * it does not; the scratch files come back byte-identical and that is asserted, not assumed.)
 *
 * ============================================================================
 * SCOPE EXCLUSIONS -- each with the fence that makes it unreachable, not merely unvisited
 * ============================================================================
 *   THE FIVE DATA-TRANSFER MSCP COMMANDS.  rq_mscp() dispatches OP_ACC/CMP/ERS/RD/WR to rq_rw(),
 *     which starts a unit service and moves blocks.  They are pcjsvax-346's work; this file throws
 *     RQUnimplemented BY NAME rather than inventing an answer, and the list is not written down --
 *     it is every opcode whose C HANDLER is `rq_rw`, read out of MSCP_OP_HANDLER below, which
 *     tests/mscpscope.js re-derives from rq_mscp()'s own switch (opcode AND handler) every run.
 *     tests/mscpringdiff.js and tests/mscpunitdiff.js FAIL the run if any graded case sends one.
 *   AN IN-FLIGHT TRANSFER'S PACKET.  rq_abo() and rq_gcs() exist to inspect `uptr->cpkt` and
 *     `uptr->pktq`, and ONLY rq_rw() ever sets `cpkt`.  Their reachable behaviour with an idle unit
 *     IS implemented and graded (ST_SUC, ABO_LNT / GCS_LNT, the zeroed GCS status words); the arms
 *     that walk a unit's packets throw by name if a unit is ever found holding one.  Same for the
 *     `if (q && uptr->cpkt) rq_enqt (&uptr->pktq, pkt)` deferral in rq_avl/rq_onl/rq_suc/rq_fmt,
 *     which IS transcribed -- it is two lines and leaving it out would be a gap pcjsvax-346 has to
 *     rediscover -- but which no command in this file's scope can reach.
 *   THE UNIT QUEUES.  rq_quesvc()'s `for (i = 0; i < RQ_NUMDR; i++)` scan over `uptr->pktq` is
 *     transcribed, but only a deferral can put a packet there and only an in-flight transfer can
 *     cause a deferral, so the body is a throw, not a branch.
 *   MEDIA REMOVAL.  rq_avl()'s `if ((mdf & MD_SPD) && RQ_RMV (uptr)) sim_disk_unload (uptr)` spins
 *     a removable drive down.  sim_disk_unload() on this platform reaches
 *     sim_os_disk_unload_raw() -- the container is opened in RAW format, because sim_disk's AUTO
 *     detection tries a raw open before falling back to SIMH format and a plain file opens fine --
 *     and sets `media_removed`, which then makes sim_disk_isavailable() false and changes what
 *     ONLINE answers.  Ejecting the user's file is not this item's outcome; the arm throws by name
 *     and tests/mscpunitdiff.js FAILS the run if any graded case sets MD_SPD on a removable unit.
 *   NOAUTOSIZE.  `set rqN noautosize` selects sim_disk_attach_ex2()'s OTHER autosize arm, the one
 *     that walks drv_types[] and changes the drive TYPE -- and that arm additionally depends on
 *     get_filesystem_size() parsing an ODS-2/ODS-1/RT-11/Ultrix volume out of the container.  A
 *     file-system parser is not a disk controller; attach() throws by name if a unit has it set and
 *     no graded case sets it.
 *   INTERRUPT DELIVERY.  rq_init_int() raises a controller interrupt when the host's S1 data has
 *     BOTH SA_S1H_IE and a non-zero SA_S1H_VEC; rq_ring_int() raises one on SA_S1H_VEC ALONE --
 *     THE TWO CONDITIONS DIFFER, and pcjsvax-aef exists because of it.  This file records the
 *     request in `irq` (as the C's `cp->irq`) and wires it to nothing; `irq` is not in SIMH's
 *     rq_reg[] so it is not examinable either.  mscpinitdiff.js FAILS the run if any graded case
 *     supplies an s1dat with IE and VEC both set; mscpringdiff.js is stricter and FAILS if VEC is
 *     non-zero at all, because rq_ring_int() does not test IE.  Vector computation
 *     (`dibp->vec = (s1dat & SA_S1H_VEC) << 2`) IS reproduced, in `vec`, because it is pure state.
 *     What IS implemented and graded is rq_ring_int()'s OTHER half: the one-word flag it DMAs to
 *     `comm + ring->ioff`, and the fact that it IGNORES a failure of that write (the C casts the
 *     Map_WriteW result to void and says so in a comment).
 *   rq_tmrsvc(), THE HOST-ACCESS TIMER.  `sim_activate_after (units + RQ_TIMER, 1000000)` is a
 *     WALL-CLOCK schedule (one second), not an instruction count, and nothing in this tree maps
 *     wall-clock to instructions.  It is not implemented.  `hat`/`htmo` are still modelled as state
 *     because rq_reset() sets them and they are examinable; mscpinitdiff.js asserts on the ORACLE
 *     that HAT == HTMO at the end of every graded case, so a case long enough for the real timer to
 *     fire fails the run rather than diverging silently.
 *   THE OTHER THREE CONTROLLERS.  RQB/RQC/RQD are DEV_DIS on the oracle (pdp11_rq.c:1483/1556/1629),
 *     so they have NO I/O-page addresses at all -- `SHOW QBA IOSPACE` lists exactly one RQ row and
 *     autoconfiguration never assigns the other three.  There is therefore nothing to probe FOR
 *     them; what is asserted instead is the complement, and it is asserted twice: mscpinitdiff.js
 *     probes the words on either side of this window and requires them to machine-check identically
 *     on both engines, and tests/dbldiff.js's PHASE W requires the set of I/O-page addresses this
 *     tree decodes to be EXACTLY the doorbell's two bytes plus this window's four.  State here is
 *     per-INSTANCE and `cnum` is a constructor argument, so a second controller would be
 *     configuration -- one more `new RQVAX(...)` and one more entry in an addIoPage() list -- rather
 *     than more code in this file.
 *   THE ROM MACHINE.  Neither pcjsvax-c2c nor pcjsvax-0b4 wires this controller into
 *     tests/rommachine.js.  The ROM's self-test 53 currently fails identically on both engines with
 *     no disk attached, and tests/conoutdiff.js's agreement is the measurement that says so;
 *     changing what the ROM finds on the I/O page is a change to that measurement and belongs to
 *     whichever of pcjsvax-6a5's children owns it, with conoutdiff re-read rather than assumed.
 *     (conoutdiff's matched-byte COUNT varies run to run -- HANDOFF.md 3 -- so the thing to re-read
 *     is its FLOOR and its first diverging byte, never the headline number.)
 *
 * ============================================================================
 * THE ORDERED REQUEST TRACE
 * ============================================================================
 * `set rq debug=REQ` prints one line per controller event, and pcjsvax-0b4's parent criterion is
 * "every command issued, every response returned, IN ORDER, with matching fields".  `reqLog` below
 * is that stream, appended at EXACTLY the six points sim_debug(DBG_REQ, ...) fires in the C
 * (pdp11_rq.c:1670, 1700, 1839, 1865, 2848 and 3073) and formatted identically, so
 * tests/mscpringdiff.js can compare two SEQUENCES rather than two end states.  It is a diagnostic
 * stream, not device state: nothing in this file ever reads it, and reset() does not clear it
 * (SIMH's debug output is not reset either).  Its one non-obvious field is `poll started, PC=%X`,
 * whose PC is the C's OLDPC -- `#define OLDPC fault_PC` on the VAX arm (pdp11_rq.c:108) -- i.e. the
 * start PC of the instruction performing the IP read.
 *
 * ONLY THE ORDER AND THE TEXT ARE COMPARABLE, not the timestamps, and that is a property of the
 * ORACLE rather than a concession here.  scp.c:13836-13900 COLLAPSES consecutive identical debug
 * lines into the first one plus `same as above (N times)`, stamping the collapsed line with the
 * time of the LAST occurrence -- so the individual timestamps of a repeated event are not present
 * in SIMH's own output to compare against.  A `t` is recorded anyway because it costs nothing and
 * because a future item that patches the oracle's debug path would want it.
 */

import { VAX } from "./defines.js";

/* ------------------------------------------------------------------------------------------- *
 * pdp11_uqssp.h -- the UQSSP port registers.  Transcribed one line per #define, in the header's *
 * own order, so a reader can diff the two.                                                      *
 * ------------------------------------------------------------------------------------------- */

const SA_ER         = 0x8000;                   // error
const SA_S4         = 0x4000;                   // init step 4
const SA_S3         = 0x2000;                   // init step 3
const SA_S2         = 0x1000;                   // init step 2
const SA_S1         = 0x0800;                   // init step 1

const SA_S1C_NV     = 0x0400;                   // fixed vec NI
const SA_S1C_Q22    = 0x0200;                   // Q22 device
const SA_S1C_DI     = 0x0100;                   // ext diags
const SA_S1C_OD     = 0x0080;                   // odd addrs NI
const SA_S1C_MP     = 0x0040;                   // mapping
const SA_S1C_SM     = 0x0020;                   // spec fncs NI
const SA_S1C_CN     = 0x0010;                   // node name NI

const SA_S1H_VL     = 0x8000;                   // valid
const SA_S1H_WR     = 0x4000;                   // wrap mode
const SA_S1H_V_CQ   = 11;                       // cmd q len
const SA_S1H_M_CQ   = 0x7;
const SA_S1H_V_RQ   = 8;                        // resp q len
const SA_S1H_M_RQ   = 0x7;
const SA_S1H_IE     = 0x0080;                   // int enb
const SA_S1H_VEC    = 0x007F;                   // vector

const SA_S2C_PT     = 0x0000;                   // port type
const SA_S2C_V_EC   = 8;                        // info to echo
const SA_S2C_M_EC   = 0xFF;

const SA_S2H_CLO    = 0xFFFE;                   // comm addr lo
const SA_S2H_PI     = 0x0001;                   // adp prg int

const SA_S3C_V_EC   = 0;                        // info to echo
const SA_S3C_M_EC   = 0xFF;

const SA_S3H_PP     = 0x8000;                   // purge, poll test
const SA_S3H_CHI    = 0x7FFF;                   // comm addr hi

const SA_S4C_V_MOD  = 4;                        // adapter #
const SA_S4C_V_VER  = 0;                        // version #

const SA_S4H_CS     = 0x0400;                   // host scrpad NI
const SA_S4H_NN     = 0x0200;                   // snd node name NI
const SA_S4H_SF     = 0x0100;                   // spec fnc NI
const SA_S4H_LF     = 0x0002;                   // send last fail
const SA_S4H_GO     = 0x0001;                   // go

const PE_PRE        = 1;                        // packet read err
const PE_PWE        = 2;                        // packet write err
const PE_QRE        = 6;                        // queue read err
const PE_QWE        = 7;                        // queue write err
const PE_HAT        = 9;                        // host access tmo
const PE_ICI        = 14;                       // inv conn ident
const PE_PIE        = 20;                       // prot incompat
const PE_PPF        = 21;                       // prg/poll err
const PE_MRE        = 22;                       // map reg rd err
const PE_NSR        = 478;                      // no such rsrc -- the free packet list ran dry

const SA_COMM_QQ    = -8;                       // unused
const SA_COMM_PI    = -6;                       // purge int
const SA_COMM_CI    = -4;                       // cmd int
const SA_COMM_RI    = -2;                       // resp int
/* ((4 << SA_S1H_M_CQ) + (4 << SA_S1H_M_RQ) - SA_COMM_QQ), computed rather than folded to 1032, so
   that a change to either ring's width code carries into the paranoia clamp on its own. */
const SA_COMM_MAX   = (4 << SA_S1H_M_CQ) + (4 << SA_S1H_M_RQ) - SA_COMM_QQ;

const UQ_DESC_OWN   = 0x80000000;               // ownership
const UQ_DESC_F     = 0x40000000;               // flag
const UQ_ADDR       = 0x003FFFFE;               // addr, word aligned
const UQ_HDR_OFF    = -4;                       // offset

/* The UQSSP packet HEADER -- the two words that live at `descriptor address + UQ_HDR_OFF`, i.e.
   BELOW the address the descriptor names.  A packet fetched from the descriptor address itself
   instead is off by exactly these four bytes, which is a mutation mscpringdiff.js carries. */
const UQ_HLNT       = 0;                        // length
const UQ_HCTC       = 1;                        // credits, type, CID
const UQ_HCTC_V_CR  = 0;                        // credits
const UQ_HCTC_M_CR  = 0xF;
const UQ_HCTC_V_TYP = 4;                        // type
const UQ_HCTC_M_TYP = 0xF;
const UQ_TYP_SEQ    = 0;                        // sequential
const UQ_TYP_DAT    = 1;                        // datagram
const UQ_HCTC_V_CID = 8;                        // conn ID
const UQ_HCTC_M_CID = 0xFF;
const UQ_CID_MSCP   = 0;                        // MSCP
const UQ_CID_TMSCP  = 1;                        // TMSCP
const UQ_CID_DUP    = 2;                        // DUP
const UQ_CID_DIAG   = 0xFF;                     // diagnostic

/* ------------------------------------------------------------------------------------------- *
 * pdp11_mscp.h -- the MSCP protocol itself                                                      *
 * ------------------------------------------------------------------------------------------- */

/** The 21 MSCP OPCODES (pdp11_mscp.h:42-62), name -> value, in the header's order.  PUBLISHED as a
    table rather than written as 21 `const`s for two reasons: rq_mscp()'s dispatch below is built
    FROM it (so the scope lives in code, HANDOFF.md standing rule 7), and tests/mscpscope.js
    re-derives the same table from the header on every differential run and FAILS on any difference
    (standing rule 5 -- this project's CIS opcode count went 7 -> 11 -> 17 -> 23 and every
    hand-derived value was wrong). */
const OP = {
    ABO: 1, GCS: 2, GUS: 3, SCC: 4, AVL: 8, ONL: 9, SUC: 10, DAP: 11,
    ACC: 16, CCD: 17, ERS: 18, FLU: 19, ERG: 22, CMP: 32, RD: 33, WR: 34,
    WTM: 36, POS: 37, FMT: 47, AVA: 64, END: 0x80
};

/** The 23 MSCP STATUS codes (pdp11_mscp.h:144-166), same discipline.  ST.V_SUB and ST.V_INV are
    SHIFT POSITIONS rather than status values and are in the table because they are in the header's
    run of `#define ST_...` lines -- the extraction is mechanical, and a table that quietly dropped
    the two that "are not really status codes" would be a hand-curated list again. */
const ST = {
    SUC: 0, CMD: 1, ABO: 2, OFL: 3, AVL: 4, MFE: 5, WPR: 6, CMP: 7, DAT: 8, HST: 9,
    CNT: 10, DRV: 11, FMT: 12, BOT: 13, TMK: 14, RDT: 16, POL: 17, SXC: 18, LED: 19,
    BBR: 20, DIA: 31, V_SUB: 5, V_INV: 8
};

const I_OPCD        = 8 << ST.V_INV;            // inv opcode
const I_VRSN        = 12 << ST.V_INV;           // inv version
const I_FMTI        = 28 << ST.V_INV;           // inv format

/* Status SUBCODES (pdp11_mscp.h:167-181).  Each is `n << ST_V_SUB` -- written that way rather than
   folded to a number, because ST.V_SUB is one of the two entries mscpscope.js re-derives and a
   folded constant would not move with it. */
const SB_SUC_ON     = 8 << ST.V_SUB;            // already online
const SB_OFL_NV     = 1 << ST.V_SUB;            // no volume
const SB_AVL_INU    = 32 << ST.V_SUB;           // in use
const SB_WPR_HW     = 256 << ST.V_SUB;          // hwre write lock

/* Unit identifier class (pdp11_mscp.h:37) */
const UID_DISK      = 2;                        // disk class

/* Unit flags (pdp11_mscp.h:107-122) -- the word rq_putr_unit() builds at ONL_UFL/GUS_UFL */
const UF_RPL        = 0x8000;                   // d: ctrl bad blk repl
const UF_WPH        = 0x2000;                   // b: wr prot hwre
const UF_WPS        = 0x1000;                   // b: wr prot swre
const UF_RMV        = 0x0080;                   // d: removable
const UF_CMW        = 0x0002;                   // cmp writes NI
const UF_CMR        = 0x0001;                   // cmp reads NI
const UF_MSK        = UF_CMR | UF_CMW;          // settable flags

/* Command MODIFIERS (pdp11_mscp.h:66-88).  NOTE MD_NXU AND MD_SPD ARE THE SAME BIT (0x0001) with
   different meanings per command -- GET UNIT STATUS reads it as "next unit", AVAILABLE as
   "spin down".  Two names for one bit is the header's own doing and is reproduced. */
const MD_SWP        = 0x0004;                   // b suc: enb set wrp
const MD_NXU        = 0x0001;                   // b gus: next unit
const MD_SPD        = 0x0001;                   // d avl: spin-down

/* Abort packet (pdp11_mscp.h:240-243) */
const ABO_LNT       = 16;
const ABO_REFL      = 8;                        // ref #
const ABO_REFH      = 9;

/* Available packet -- minimum size (pdp11_mscp.h:247) */
const AVL_LNT       = 12;

/* Get command status packet (pdp11_mscp.h:270-274) */
const GCS_LNT       = 20;
const GCS_REFL      = 8;                        // ref #
const GCS_REFH      = 9;
const GCS_STSL      = 10;                       // status
const GCS_STSH      = 11;

/* Format packet (pdp11_mscp.h:278-279) */
const FMT_LNT       = 12;
const FMT_IH        = 17;                       // magic bit
/** `(cp->pak[pkt].d[FMT_IH] & 0100000) == 0` (pdp11_rq.c:2247).  OCTAL 0100000 is 0x8000 -- the
    header gives the WORD but not the BIT, so the constant lives in the .c and is octal there. */
const FMT_IH_MAGIC  = 0o100000;

/* Get unit status packet (pdp11_mscp.h:283-316).  The DISK length, not the tape one. */
const GUS_LNT_D     = 48;
const GUS_UVER      = 23;                       // unit version
const GUS_SHUN      = 18;                       // shadowing
const GUS_SHST      = 19;
const GUS_TRK       = 20;                       // track
const GUS_GRP       = 21;                       // group
const GUS_CYL       = 22;                       // cylinder
const GUS_RCTS      = 24;                       // RCT size
const GUS_RBSC      = 25;                       // RBNs, copies
const GUS_RB_V_RBNS = 0;                        // RBNs/track
const GUS_RB_V_RCTC = 8;                        // RCT copies

/* Unit online packet (pdp11_mscp.h:321-346).  rq_putr_unit() writes the ONL_ names for BOTH the
   long and the short form, and GET UNIT STATUS's GUS_MLUN/UFL/UIDA..D/MEDL are the SAME WORD
   INDICES (8, 9, 12..15, 16) -- which is why one function fills both. */
const ONL_LNT       = 44;
const ONL_MLUN      = 8;                        // mlun
const ONL_UFL       = 9;                        // flags
const ONL_RSVL      = 10;                       // reserved
const ONL_RSVH      = 11;
const ONL_UIDA      = 12;                       // unit ID
const ONL_UIDB      = 13;
const ONL_UIDC      = 14;
const ONL_UIDD      = 15;
const ONL_MEDL      = 16;                       // media ID
const ONL_MEDH      = 17;
const ONL_SHUN      = 18;                       // shadowing
const ONL_SHST      = 19;
const ONL_SIZL      = 20;                       // size
const ONL_SIZH      = 21;
const ONL_VSNL      = 22;                       // vol ser #
const ONL_VSNH      = 23;
const ONL_UIDD_V_MOD = 0;                       // unit model
const ONL_UIDD_V_CLS = 8;                       // unit class

/** `cp->pak[pkt].d[ONL_VSNL] = 01234 + lu;` (pdp11_rq.c:2963).  *** 01234 IS OCTAL: 668 DECIMAL. ***
    Written as a JS octal literal so the digits match the C's character for character; a reader who
    checks it against the source sees the same 1234 and a reader who computes it gets 668.  Reading
    it as decimal 1234 is the single most likely transcription error in rq_putr_unit(). */
const ONL_VSN_BASE  = 0o1234;

/* Set unit characteristics -- same layout as ONLINE (pdp11_mscp.h:379) */
const SUC_LNT       = 44;

/** rq_cmdname[] (pdp11_rq.c:1106-1135), DERIVED rather than transcribed: the C's table is non-empty
    at exactly the indices an OP_ code names, and it is indexed `cmd & 0x3f`, so OP_AVA (64) folds
    onto index 0 and its name is unreachable -- reproduced, not tidied. */
const CMD_NAMES = (function() {
    let t = new Array(64).fill("");
    for (let n of Object.keys(OP)) if (OP[n] < 64) t[OP[n]] = n;
    return t;
})();

/* Command packet header (pdp11_mscp.h:212-225) */
const CMD_REFL      = 2;                        // ref #
const CMD_REFH      = 3;
const CMD_UN        = 4;                        // unit #
const CMD_OPC       = 6;                        // opcode
const CMD_MOD       = 7;                        // modifier
const CMD_OPC_V_OPC = 0;
const CMD_OPC_M_OPC = 0xFF;

/* Response packet header (pdp11_mscp.h:227-238).  RSP_OPF is word 6 -- THE SAME WORD as CMD_OPC,
   which is why rq_putpkt() can read back the END flag rq_putr() just wrote with GETP(CMD_OPC,OPC). */
const RSP_LNT       = 12;
const RSP_REFL      = 2;                        // ref #
const RSP_REFH      = 3;
/** RSP_UN is word 4 -- THE SAME WORD as CMD_UN, which is why rq_gus()'s MD_NXU wrap can "return"
    the reset unit number by writing the word the host's own command arrived in. */
const RSP_UN        = 4;                        // unit #
const RSP_RSV       = 5;                        // reserved
const RSP_OPF       = 6;                        // opcd,flg
const RSP_STS       = 7;                        // status
const RSP_OPF_V_OPC = 0;
const RSP_OPF_V_FLG = 8;

/* SET CONTROLLER CHARACTERISTICS (pdp11_mscp.h:360-375) */
const SCC_LNT       = 32;
const SCC_MSV       = 8;                        // MSCP version
const SCC_CFL       = 9;                        // flags
const SCC_TMO       = 10;                       // timeout
const SCC_VER       = 11;                       // ctrl version
const SCC_CIDA      = 12;                       // ctrl ID
const SCC_CIDB      = 13;
const SCC_CIDC      = 14;
const SCC_CIDD      = 15;
const SCC_MBCL      = 16;                       // max byte count
const SCC_MBCH      = 17;
const SCC_VER_V_SVER = 0;
const SCC_VER_V_HVER = 8;
const SCC_CIDD_V_MOD = 0;
const SCC_CIDD_V_CLS = 8;

/* Read/write packet words -- referenced ONLY by the DBG_REQ trace line, which prints them for every
   command whatever the opcode (pdp11_rq.c:1865).  That is why a command packet's untouched tail
   shows up in the trace, and why the trace is evidence that the WHOLE 64 bytes round-tripped. */
const RW_BCL        = 8;
const RW_BCH        = 9;
const RW_BAL        = 10;
const RW_BAH        = 11;
const RW_LBNL       = 16;
const RW_LBNH       = 17;

/* pdp11_mscp.h:100 */
const CF_RPL        = 0x8000;                   // ctrl bad blk repl
const CF_ATN        = 0x0080;                   // enb attention

/* ------------------------------------------------------------------------------------------- *
 * pdp11_rq.c's own constants                                                                    *
 * ------------------------------------------------------------------------------------------- */

const CST_S1        = 0;                        // init stage 1
const CST_S1_WR     = 1;                        // stage 1 wrap
const CST_S2        = 2;                        // init stage 2
const CST_S3        = 3;                        // init stage 3
const CST_S3_PPA    = 4;                        // stage 3 sa wait
const CST_S3_PPB    = 5;                        // stage 3 ip wait
const CST_S4        = 6;                        // stage 4
const CST_UP        = 7;                        // online
const CST_DEAD      = 8;                        // fatal error

/** The nine controller states, name -> value, in the C's order.  Published so a test can walk ALL
    of them without transcribing a list of its own (HANDOFF.md standing rule 5). */
const CST_NAMES = ["CST_S1", "CST_S1_WR", "CST_S2", "CST_S3", "CST_S3_PPA", "CST_S3_PPB",
                   "CST_S4", "CST_UP", "CST_DEAD"];

const RQ_CLASS      = 1;                        // RQ class: mass storage controllers
const RQ_NUMBY      = 512;                      // bytes per block
const RQ_MAXDR      = 254;                      // max # drives
/** RQ_TIMER == RQ_MAXDR and RQ_QUEUE == RQ_MAXDR+1 (pdp11_rq.c:195-196): the DEVICE declares
    RQ_MAXDR+2 units and the last TWO ARE NOT DRIVES.  Published as a number so a reader can check
    the arithmetic, and used by nothing -- see the file header: this tree's `units` array holds only
    drives, so there is no `numunits - 2` to get wrong. */
const RQ_PSEUDO_UNITS = 2;
const RQ_HVER       = 1;                        // hardware version
const RQ_SVER       = 3;                        // software version
const RQ_DHTMO      = 60;                       // def host timeout
const RQ_DCTMO      = 120;                      // def ctrl timeout
const RQ_NUMDR      = 4;                        // def # drives
const RQ_NPKTS      = 32;                       // # packets (pwr of 2)
const RQ_M_NPKTS    = RQ_NPKTS - 1;
const RQ_PKT_SIZE_W = 32;                       // payload size (wds)
const RQ_PKT_SIZE   = RQ_PKT_SIZE_W * 2;

/** The VAX build's timing constants (pdp11_rq.c:105-107 -- the `defined (VM_VAX)` arm; the PDP-11
    arm's 200/500 are a DIFFERENT machine and are not what this file models).  All four are SIMH
    REGISTERS: settable and examinable, which is why mscpinitdiff.js pins them on the oracle instead
    of trusting the defaults to be these. */
const RQ_ITIME      = 450;                      // init time, except stage 4
const RQ_ITIME4     = 10;                       // stage 4
const RQ_QTIME      = 100;                      // response time for 'immediate' packets
const RQ_XTIME      = 200;                      // response time for data transfers

/** ctlr_tab[] (pdp11_rq.c:1041-1054), whole, indexed by ctype exactly as the C indexes it.  The
    RQDX3 row is the only one this item's oracle configuration uses, but the table is transcribed
    entire so that `ctype` remains a genuine parameter -- a one-row table would make the step-4 model
    number a constant wearing a lookup's clothes, and tests/mscpinitdiff.js's `wrong-port-model`
    mutation would then have nothing to perturb. */
const CTLR_TAB = [
    {uqpm: 0,  model: 0,  name: "DEFAULT"},
    {uqpm: 3,  model: 3,  name: "KLESI"},
    {uqpm: 10, model: 10, name: "RUX50"},
    {uqpm: 6,  model: 6,  name: "UDA50A"},
    {uqpm: 7,  model: 7,  name: "RQDX1"},
    {uqpm: 19, model: 19, name: "RQDX3"},
    {uqpm: 13, model: 13, name: "KDA50"},
    {uqpm: 16, model: 16, name: "KRQ50"},
    {uqpm: 26, model: 26, name: "KRU50"},
    {uqpm: 20, model: 20, name: "RQDX4"},
    {uqpm: 2,  model: 2,  name: "UDA50"}
];
const RQDX3_CTYPE   = CTLR_TAB.findIndex((c) => c.name === "RQDX3");

/* ------------------------------------------------------------------------------------------- *
 * drv_tab[] -- pdp11_rq.c:955-991.  GENERATED, NEVER TYPED.                                     *
 *                                                                                               *
 * Thirty-four drive types of fourteen fields, produced by the RQ_DRV token-pasting macro from     *
 * ~480 #defines.  This table was emitted by tests/mscpscope.js's own extractor and that same       *
 * extractor re-derives it from pdp11_rq.c and compares it ROW FOR ROW, FIELD FOR FIELD AND IN      *
 * ORDER on every differential run, failing the run on any difference (HANDOFF.md standing rule 5 -- *
 * this project's CIS opcode count went 7 -> 11 -> 17 -> 23 and every hand-derived value was wrong). *
 *                                                                                                 *
 * ORDER IS SEMANTIC, not presentation: the INDEX is the drive type, it is what `set rqN <type>`    *
 * stores in the unit, and it is what rq_putr_unit() and rq_gus() index.  A table with the right     *
 * contents in the wrong order gives every unit some other drive's media identifier.                *
 *                                                                                                 *
 * Two extraction traps the generator had to survive, recorded because they are invisible in the     *
 * result: RQDF_DSSI is `010`, which is EIGHT (a leading zero is octal in C), so every RF-series     *
 * `flgs` here is 8; and thirty-one `_TPG` values are `#define X_TPG X_SURF` rather than literals.   *
 *                                                                                                 *
 * `xbn`, `dbn`, `cyl`, `surf`, `lbn`, `rctc`, `rbn` are carried even though no command in this      *
 * file reads them, because the table is a TRANSCRIPTION OF drv_tab[] and a subset would be the      *
 * hand-curated list this discipline exists to prevent -- and because mscpscope.js compares all      *
 * fourteen, so a wrong one fails the run whether or not anything indexes it.  (`lbn` IS read: it    *
 * is the capacity `set rqN <type>` installs.)                                                      *
 * ------------------------------------------------------------------------------------------- */

const RQDF_RMV      = 0o001;                    // removable
const RQDF_RO       = 0o002;                    // read only
const RQDF_SDI      = 0o004;                    // SDI drive
const RQDF_DSSI     = 0o010;                    // DSSI drive -- OCTAL 010 IS EIGHT

const DRV_TAB = [
    {name: "RX50", sect: 10, surf: 1, cyl: 80, tpg: 5, gpc: 16, xbn: 0, dbn: 0, lbn: 800, rcts: 0, rctc: 0, rbn: 0, mod: 7, MediaId: 0x25658032, flgs: 1},
    {name: "RX33", sect: 15, surf: 2, cyl: 80, tpg: 2, gpc: 1, xbn: 0, dbn: 0, lbn: 2400, rcts: 0, rctc: 0, rbn: 0, mod: 10, MediaId: 0x25658021, flgs: 1},
    {name: "RD51", sect: 18, surf: 4, cyl: 306, tpg: 4, gpc: 1, xbn: 57, dbn: 87, lbn: 21600, rcts: 36, rctc: 4, rbn: 144, mod: 6, MediaId: 0x25644033, flgs: 0},
    {name: "RD31", sect: 17, surf: 4, cyl: 615, tpg: 4, gpc: 1, xbn: 54, dbn: 14, lbn: 41560, rcts: 3, rctc: 8, rbn: 100, mod: 12, MediaId: 0x2564401F, flgs: 0},
    {name: "RD52", sect: 17, surf: 8, cyl: 512, tpg: 8, gpc: 1, xbn: 54, dbn: 82, lbn: 60480, rcts: 4, rctc: 8, rbn: 168, mod: 8, MediaId: 0x25644034, flgs: 0},
    {name: "RD53", sect: 17, surf: 8, cyl: 1024, tpg: 8, gpc: 1, xbn: 54, dbn: 82, lbn: 138672, rcts: 5, rctc: 8, rbn: 280, mod: 9, MediaId: 0x25644035, flgs: 0},
    {name: "RD54", sect: 17, surf: 15, cyl: 1225, tpg: 15, gpc: 1, xbn: 54, dbn: 201, lbn: 311200, rcts: 7, rctc: 8, rbn: 609, mod: 13, MediaId: 0x25644036, flgs: 0},
    {name: "RA82", sect: 57, surf: 15, cyl: 1435, tpg: 15, gpc: 1, xbn: 3480, dbn: 3480, lbn: 1216665, rcts: 3420, rctc: 1, rbn: 21345, mod: 11, MediaId: 0x25641052, flgs: 4},
    {name: "RRD40", sect: 128, surf: 1, cyl: 10400, tpg: 1, gpc: 1, xbn: 0, dbn: 0, lbn: 1331200, rcts: 0, rctc: 0, rbn: 0, mod: 26, MediaId: 0x25652228, flgs: 3},
    {name: "RA72", sect: 51, surf: 20, cyl: 1921, tpg: 20, gpc: 1, xbn: 2080, dbn: 2080, lbn: 1953300, rcts: 2040, rctc: 1, rbn: 38300, mod: 37, MediaId: 0x25641048, flgs: 4},
    {name: "RA90", sect: 69, surf: 13, cyl: 2656, tpg: 13, gpc: 1, xbn: 1820, dbn: 1820, lbn: 2376153, rcts: 1794, rctc: 1, rbn: 34437, mod: 19, MediaId: 0x2564105A, flgs: 4},
    {name: "RA92", sect: 73, surf: 13, cyl: 3101, tpg: 13, gpc: 1, xbn: 174, dbn: 788, lbn: 2940951, rcts: 949, rctc: 1, rbn: 40287, mod: 29, MediaId: 0x2564105C, flgs: 4},
    {name: "RA8U", sect: 57, surf: 15, cyl: 1435, tpg: 15, gpc: 1, xbn: 0, dbn: 0, lbn: 1216665, rcts: 400, rctc: 8, rbn: 21345, mod: 11, MediaId: 0x25641052, flgs: 4},
    {name: "RA60", sect: 42, surf: 6, cyl: 1600, tpg: 6, gpc: 1, xbn: 1032, dbn: 1032, lbn: 400176, rcts: 1008, rctc: 1, rbn: 9528, mod: 4, MediaId: 0x22A4103C, flgs: 5},
    {name: "RA81", sect: 51, surf: 14, cyl: 1258, tpg: 14, gpc: 1, xbn: 2436, dbn: 2436, lbn: 891072, rcts: 2856, rctc: 1, rbn: 17472, mod: 5, MediaId: 0x25641051, flgs: 4},
    {name: "RA71", sect: 51, surf: 14, cyl: 1921, tpg: 14, gpc: 1, xbn: 1456, dbn: 1456, lbn: 1367310, rcts: 1428, rctc: 1, rbn: 26810, mod: 40, MediaId: 0x25641047, flgs: 4},
    {name: "RD32", sect: 17, surf: 6, cyl: 820, tpg: 6, gpc: 1, xbn: 54, dbn: 48, lbn: 83236, rcts: 4, rctc: 8, rbn: 200, mod: 15, MediaId: 0x25644020, flgs: 0},
    {name: "RC25", sect: 50, surf: 8, cyl: 1260, tpg: 8, gpc: 1, xbn: 0, dbn: 0, lbn: 50902, rcts: 0, rctc: 1, rbn: 0, mod: 2, MediaId: 0x20643019, flgs: 1},
    {name: "RCF25", sect: 50, surf: 8, cyl: 1260, tpg: 8, gpc: 1, xbn: 0, dbn: 0, lbn: 50902, rcts: 0, rctc: 1, rbn: 0, mod: 3, MediaId: 0x20643319, flgs: 0},
    {name: "RA80", sect: 31, surf: 14, cyl: 546, tpg: 14, gpc: 1, xbn: 0, dbn: 0, lbn: 237212, rcts: 0, rctc: 1, rbn: 0, mod: 1, MediaId: 0x25641050, flgs: 4},
    {name: "RA70", sect: 33, surf: 11, cyl: 1507, tpg: 11, gpc: 1, xbn: 0, dbn: 0, lbn: 547041, rcts: 198, rctc: 7, rbn: 16577, mod: 18, MediaId: 0x25641046, flgs: 4},
    {name: "RA73", sect: 70, surf: 21, cyl: 2667, tpg: 21, gpc: 1, xbn: 0, dbn: 0, lbn: 3920490, rcts: 198, rctc: 7, rbn: 56007, mod: 47, MediaId: 0x25641049, flgs: 4},
    {name: "RF30", sect: 37, surf: 6, cyl: 1320, tpg: 6, gpc: 1, xbn: 1456, dbn: 1456, lbn: 293040, rcts: 1428, rctc: 1, rbn: 26810, mod: 21, MediaId: 0x2264601E, flgs: 8},
    {name: "RF31", sect: 50, surf: 8, cyl: 1861, tpg: 8, gpc: 1, xbn: 1456, dbn: 1456, lbn: 744400, rcts: 1428, rctc: 1, rbn: 26810, mod: 27, MediaId: 0x2264601F, flgs: 8},
    {name: "RF35", sect: 57, surf: 14, cyl: 1861, tpg: 14, gpc: 1, xbn: 1456, dbn: 1456, lbn: 1664628, rcts: 1428, rctc: 1, rbn: 26810, mod: 27, MediaId: 0x2264601F, flgs: 8},
    {name: "RF71", sect: 37, surf: 16, cyl: 1320, tpg: 16, gpc: 1, xbn: 1456, dbn: 1456, lbn: 781440, rcts: 1428, rctc: 1, rbn: 26810, mod: 40, MediaId: 0x22646047, flgs: 8},
    {name: "RF72", sect: 50, surf: 21, cyl: 1861, tpg: 21, gpc: 1, xbn: 1456, dbn: 1456, lbn: 1954050, rcts: 1428, rctc: 1, rbn: 26810, mod: 28, MediaId: 0x22646048, flgs: 8},
    {name: "RF73", sect: 71, surf: 21, cyl: 2621, tpg: 21, gpc: 1, xbn: 1456, dbn: 1456, lbn: 3907911, rcts: 1428, rctc: 1, rbn: 26810, mod: 35, MediaId: 0x22646049, flgs: 8},
    {name: "ESE20", sect: 4, surf: 128, cyl: 480, tpg: 128, gpc: 1, xbn: 0, dbn: 0, lbn: 245760, rcts: 0, rctc: 1, rbn: 0, mod: 25, MediaId: 0x254B3294, flgs: 4},
    {name: "ESE52", sect: 4, surf: 128, cyl: 465, tpg: 128, gpc: 1, xbn: 0, dbn: 0, lbn: 238080, rcts: 0, rctc: 1, rbn: 0, mod: 31, MediaId: 0x254B32B4, flgs: 4},
    {name: "ESE56", sect: 4, surf: 128, cyl: 2337, tpg: 128, gpc: 1, xbn: 0, dbn: 0, lbn: 1196544, rcts: 0, rctc: 1, rbn: 0, mod: 48, MediaId: 0x254B32B8, flgs: 4},
    {name: "ESE58", sect: 4, surf: 128, cyl: 3741, tpg: 128, gpc: 1, xbn: 0, dbn: 0, lbn: 1915392, rcts: 0, rctc: 1, rbn: 0, mod: 49, MediaId: 0x254B32BA, flgs: 4},
    {name: "RD33", sect: 17, surf: 7, cyl: 1170, tpg: 7, gpc: 1, xbn: 54, dbn: 65, lbn: 138635, rcts: 5, rctc: 8, rbn: 317, mod: 24, MediaId: 0x25644021, flgs: 0},
    {name: "RX18", sect: 9, surf: 1, cyl: 40, tpg: 7, gpc: 1, xbn: 0, dbn: 0, lbn: 360, rcts: 0, rctc: 0, rbn: 0, mod: 17, MediaId: 0x25658012, flgs: 1}
];

/** Looked up by NAME rather than written as a number, so a table that grew an entry ahead of these
    two does not silently re-point them.  RX33 is the only drive rq_fmt() will format; RD54 and RX50
    are the microvax3900's own defaults (units 0..2 and unit 3 respectively -- measured: `show rq`
    on the stock oracle prints `RD54, autosize, 159MB` three times and `RX50, 409KB` once). */
const RX33_DTYPE    = DRV_TAB.findIndex((d) => d.name === "RX33");
const RD54_DTYPE    = DRV_TAB.findIndex((d) => d.name === "RD54");
const RX50_DTYPE    = DRV_TAB.findIndex((d) => d.name === "RX50");

/* ------------------------------------------------------------------------------------------- *
 * UNIT FLAGS.                                                                                   *
 *                                                                                               *
 * *** THESE BIT POSITIONS ARE THIS TREE'S, NOT SIMH'S. ***  In the C they are fields of           *
 * `uptr->flags`, packed above DKUF_V_UF alongside the SIX-BIT DRIVE TYPE, and rq_reg[] does not   *
 * publish `flags` at all -- so no host program and no `examine` can observe the encoding, only    *
 * the predicates built on it.  The drive type is kept in its own `dtype` field here rather than   *
 * shifted into a bit field, because GET_DTYPE() is then not a place a mask can be wrong.  What is  *
 * faithful is the SET of states and every predicate over them, and those are graded.              *
 * ------------------------------------------------------------------------------------------- */

const U_ATT         = 0x0001;                   // UNIT_ATT      -- a container is attached
const U_ONL         = 0x0002;                   // UNIT_ONL      -- MSCP ONLINE
const U_ATP         = 0x0004;                   // UNIT_ATP      -- attention pending
const U_WLK         = 0x0008;                   // UNIT_WLK      -- `set rqN locked`
const U_RO          = 0x0010;                   // UNIT_RO       -- attached with -R
const U_DIS         = 0x0020;                   // UNIT_DIS      -- `set rqN disable`
const U_NOAUTO      = 0x0040;                   // UNIT_NOAUTO   -- `set rqN noautosize`

/* ------------------------------------------------------------------------------------------- *
 * rq_mscp()'s DISPATCH, as three disjoint name sets over the OP table above (standing rule 7:    *
 * scope lives in code, not comments).  tests/mscpscope.js extracts the `case OP_x:` labels from   *
 * rq_mscp()'s own switch in pdp11_rq.c and FAILS the run unless UNIT_OPS + NOP_OPS is EXACTLY     *
 * that set -- so an opcode the C handles and this file does not cannot fall silently through to   *
 * the illegal-opcode default, which is the failure mode HANDOFF.md standing rule 7 records.       *
 * ------------------------------------------------------------------------------------------- */

/** The switch's own no-op arm -- `cmd |= OP_END; sts = ST_SUC; break;` -- reaches no function, so it
    is NAMED rather than left as an empty string, which would read as "not classified". */
const MSCP_NOP_ARM  = "(switch nop arm)";

/** *** THE DISPATCH, OPCODE AND C HANDLER TOGETHER. ***  This is rq_mscp()'s switch, and it is the
    single source for every scope question this file answers: which opcodes are dispatched at all,
    which reach a handler that is implemented here, and which throw by name.  tests/mscpscope.js
    extracts the same map -- `case OP_x:` label AND the `return rq_yyy (` it falls through to -- from
    pdp11_rq.c on every differential run and FAILS on any difference.

    Carrying the HANDLER and not merely the opcode is the point.  The unit-bearing arms are split
    three ways (answered here / needs an in-flight transfer / IS a transfer), and a classification
    nothing checks is the hand-curated list HANDOFF.md standing rule 5 forbids.  With the handler
    recorded, an opcode that moved between arms in a vendor update fails PHASE S instead of being
    silently answered by the wrong one. */
const MSCP_OP_HANDLER = {
    ABO: "rq_abo", AVL: "rq_avl", FMT: "rq_fmt", GCS: "rq_gcs", GUS: "rq_gus",
    ONL: "rq_onl", SCC: "rq_scc", SUC: "rq_suc",
    ACC: "rq_rw", CMP: "rq_rw", ERS: "rq_rw", RD: "rq_rw", WR: "rq_rw",
    CCD: MSCP_NOP_ARM, DAP: MSCP_NOP_ARM, FLU: MSCP_NOP_ARM
};

/** C handler -> the METHOD on RQVAX that is it.  A handler the C dispatches to and this map does
    not name is OUT OF SCOPE and throws by name; that is the whole fence, and it is one lookup
    rather than a list, so it cannot disagree with MSCP_OP_HANDLER above. */
const MSCP_METHOD = {
    rq_abo: "abo", rq_avl: "avl", rq_fmt: "fmt", rq_gcs: "gcs",
    rq_gus: "gus", rq_onl: "onl", rq_scc: "scc", rq_suc: "suc"
};

/** Why `rq_rw` is not in MSCP_METHOD, phrased for the exception a graded case would see. */
const MSCP_EXCLUDED = {
    rq_rw: "rq_rw() starts a unit service and moves blocks between the image and host memory; " +
           "data transfer is pcjsvax-346's work, not pcjsvax-f52's"
};

/** Dispatched by the C to a handler that needs a UNIT -- i.e. everything but SET CONTROLLER
    CHARACTERISTICS and the no-op arm.  DERIVED from the handler map rather than written down, so it
    cannot disagree with it.  tests/mscpringdiff.js (pcjsvax-0b4, which attaches no unit) uses it to
    fence its own case list; SEVEN of the twelve are answered here now and five still throw. */
const MSCP_UNIT_OPS = Object.keys(MSCP_OP_HANDLER).filter(
    (n) => MSCP_OP_HANDLER[n] !== MSCP_NOP_ARM && MSCP_OP_HANDLER[n] !== "rq_scc");

/** Dispatched by the C to the switch's own no-op arm.  Derived, same reason. */
const MSCP_NOP_OPS  = Object.keys(MSCP_OP_HANDLER).filter((n) => MSCP_OP_HANDLER[n] === MSCP_NOP_ARM);

/** SET CONTROLLER CHARACTERISTICS -- the only MSCP command that needs neither an attached unit nor
    a transfer, which is exactly why pcjsvax-0b4 uses it as the vehicle for grading the ring
    machinery and nothing else. */
const MSCP_SCC_OP   = "SCC";

/** The opcodes still out of scope, DERIVED: every one whose C handler has no method here. */
const MSCP_XFER_OPS = Object.keys(MSCP_OP_HANDLER).filter(
    (n) => MSCP_OP_HANDLER[n] !== MSCP_NOP_ARM && !(MSCP_OP_HANDLER[n] in MSCP_METHOD));

/** vax_io.c:155-style autoconfiguration, VERIFIED rather than assumed: `IOBA_AUTO` leaves the RQ
    DIB's base to pdp11_io_lib.c, and tests/mscpinitdiff.js re-reads `SHOW QBA IOSPACE` from the live
    oracle on every run and FAILS if the RQ row's base or length disagrees with these two constants.
    The same discipline cqipc.js's DBL_BASE is under, and for the same reason. */
const RQ_OFFSET     = 0x1468;
const RQ_BASE       = (VAX.PHYSMEM.IOPAGE_BASE + RQ_OFFSET) >>> 0;
const IOLN_RQ       = 4;                        // pdp11_rq.c:1212, IOLN_RQ == 004

/** Thrown by name rather than answered: see the SCOPE EXCLUSIONS section of the file header. */
class RQUnimplemented extends Error {}

/**
 * checkProvider(p)
 *
 * *** THE IMAGE PROVIDER CONTRACT. ***  An attachable image is any object with
 *
 *      byteLength                  the container's size in bytes, a non-negative integer
 *      read(offset, length, dst)   fill `dst` (a Uint8Array) with `length` bytes from `offset`,
 *                                  returning the number of bytes actually delivered
 *
 * and nothing else.  The Node differential wraps `fs`, a browser wraps a `File`/`Blob` read into an
 * ArrayBuffer, and THIS FILE IMPORTS NEITHER -- which is what makes HANDOFF.md 8's decision (ship no
 * OpenVMS media; the user supplies the image) implementable in a browser rather than only on a
 * server.  The controller must not be able to tell which it has, so `attach()` never inspects
 * anything but these two members.
 *
 * *** NOTHING IN THIS ITEM CALLS read(). ***  Block transfer is pcjsvax-346.  It is nevertheless
 * REQUIRED here and checked here, because a provider that silently lacks it would satisfy every
 * grading this file can do and fail only much later, in a different item, against a different
 * oracle -- and because a contract whose unused half is unenforced is not a contract.  The check is
 * itself graded: tests/mscpunitdiff.js attaches three malformed providers and requires each to be
 * refused BY NAME.
 *
 * @param {Object} p
 * @returns {Object} the same provider
 */
function checkProvider(p)
{
    if (!p || typeof p !== "object") {
        throw new Error("rq.js: attach() needs an IMAGE PROVIDER object {byteLength, read()}");
    }
    if (typeof p.byteLength !== "number" || !Number.isInteger(p.byteLength) || p.byteLength < 0) {
        throw new Error("rq.js: the image provider's byteLength must be a non-negative integer; " +
            "it is what sim_disk's autosize calls the CONTAINER SIZE and the unit's capacity is " +
            "computed from it");
    }
    if (typeof p.read !== "function") {
        throw new Error("rq.js: the image provider has no read(offset, length, dst) -- nothing in " +
            "pcjsvax-f52 calls it, which is exactly why it is checked here rather than discovered " +
            "missing by pcjsvax-346's first data transfer");
    }
    return p;
}

/**
 * bufferProvider(buf)
 *
 * The one provider that belongs in this file, because it depends on nothing outside the language: a
 * Uint8Array or ArrayBuffer, which is what a browser's FileReader hands back.  The Node
 * differential's `fs`-backed provider lives in tests/mscpharness.js and this file never sees it.
 *
 * @param {Uint8Array|ArrayBuffer} buf
 * @returns {Object}
 */
function bufferProvider(buf)
{
    let u8 = (buf instanceof Uint8Array) ? buf : new Uint8Array(buf);
    return {
        byteLength: u8.length,
        read(offset, length, dst) {
            let n = Math.max(0, Math.min(length, u8.length - offset));
            dst.set(u8.subarray(offset, offset + n));
            return n;
        }
    };
}

/** printf's `%04X`, which is what every field of the DBG_REQ trace is printed with. */
function h4(v) { return (v & 0xFFFF).toString(16).toUpperCase().padStart(4, "0"); }

/**
 * @class RQVAX
 *
 * ONE controller instance -- the C's `MSC` context struct plus the two register entry points.  All
 * state is per-instance and `cnum` is a constructor argument, so RQB/RQC/RQD would be three more
 * `new RQVAX(...)` calls and one more entry in the caller's addIoPage() list, not more code here.
 */
export default class RQVAX {
    /**
     * RQVAX(cqbic, opts)
     *
     * @param {Object} cqbic a CQBICVAX WITH a bus (cqbic.js) -- rq_step4()'s comm-region zeroing and
     *   rq_getdesc()'s descriptor fetch are DMAs through the scatter-gather map, and there is no
     *   other path to memory from here.  A CQBIC built without a bus throws from mapWriteW(), which
     *   is the guard qdmadiff.js already asserts at startup; this file does not add a second one.
     * @param {Object} [opts] {cnum, ctype, base} -- `base` is the Qbus I/O-page address this
     *   instance answers for, defaulting to the autoconfigured RQ_BASE.
     */
    constructor(cqbic, opts = {})
    {
        this.cqbic = cqbic;
        this.cnum = opts.cnum || 0;
        this.ctype = (opts.ctype === undefined) ? RQDX3_CTYPE : opts.ctype;
        this.base = ((opts.base === undefined) ? RQ_BASE : opts.base) >>> 0;

        /* rq_itime/rq_itime4/rq_qtime/rq_xtime are C GLOBALS shared by all four controllers and
           settable from SCP.  They are instance fields here because this tree has no SCP and because
           mscpinitdiff.js pins them from the oracle's own `EXAMINE RQ ITIME` rather than assuming
           the defaults -- see the file header's event-model section. */
        this.itime = RQ_ITIME;
        this.itime4 = RQ_ITIME4;
        this.qtime = RQ_QTIME;
        this.xtime = RQ_XTIME;

        /* The RQ_QUEUE unit's event-queue slot: a deadline in cpu.nTotalCycles units, or null for
           "not active".  `sim_is_active (uptr)` is exactly `queDue !== null`. */
        this.queDue = null;
        this.pakLink = new Uint16Array(RQ_NPKTS);
        this.pakData = new Uint16Array(RQ_NPKTS * RQ_PKT_SIZE_W);

        /* THE DRIVES, AND ONLY THE DRIVES.  rq_unit[] in the C is RQ_NUMDR entries of `UDATA (...,
           (RD54_DTYPE << UNIT_V_DTYPE), RQ_SIZE (RD54))` with the LAST one RX50 -- measured on the
           stock oracle, `show rq` prints RD54/RD54/RD54/RX50 and `examine rq plug[0..3]` prints
           0/1/2/3.  `unit_plug` is `d` on the VAX arm of rq_reset()'s plug initialisation, so plug
           == index by default; it is a separate field because it is a REGISTER the host addresses
           units by, and rq_getucb() matches on it rather than on the index.

           The RQ_TIMER and RQ_QUEUE pseudo-units are NOT here -- see the file header. */
        this.units = [RD54_DTYPE, RD54_DTYPE, RD54_DTYPE, RX50_DTYPE].map((dtype, i) => ({
            plug: i, dtype, flags: 0, uf: 0, capac: DRV_TAB[dtype].lbn, cpkt: 0, pktq: 0, image: null
        }));
        if (this.units.length !== RQ_NUMDR) {
            throw new Error("rq.js: the units array must hold exactly RQ_NUMDR drives and no " +
                "pseudo-units -- rq_getucb() and rq_tmrsvc() walk `numunits - 2` in the C for " +
                "exactly that reason, and this tree's walks have no such subtraction to omit");
        }

        /* The DMA staging buffers.  Allocated ONCE per controller rather than per operation:
           HANDOFF.md standing rule 14 is about exactly this kind of per-operation allocation, and a
           64-byte array built inside rq_getpkt() would be built once per MSCP command. */
        this.descBuf = new Uint8Array(4);
        this.pktBuf = new Uint8Array(RQ_PKT_SIZE);
        this.flagBuf = new Uint8Array(2);

        /** The `set rq debug=REQ` stream -- see the file header.  A diagnostic, not device state:
            nothing here reads it and neither reset() nor powerUp() clears it, because SIMH's debug
            output survives `reset -p all` too.  Its consumer truncates it. */
        this.reqLog = [];

        this.powerUp();
    }

    /**
     * powerUp()
     *
     * The state of the C's `MSC` struct at PROCESS START, which is not the same thing as the state
     * after rq_reset().  FOUR groups of fields survive `reset -p all` because rq_reset() never
     * assigns them, and every one was found by running tests/mscpinitdiff.js against the oracle
     * rather than by reading the C carefully enough:
     *
     *   perr        rq_fatal() sets it AFTER calling rq_reset(), and CST_S4 -> CST_UP is the only
     *               place that clears it.  So a controller re-initialised after a fatal error still
     *               reports the error that killed it, which is exactly what SA_S4H_LF is for.
     *   saw         the last word the host wrote to SA.  rq_reset() never assigns it.
     *   prgi        the purge-interrupt flag.  rq_reset() never assigns it either.
     *   cq/rq.ioff  rq_reset() clears each ring's ba, lnt and idx BY NAME and leaves ioff alone.
     *
     * In SIMH these are fields of a STATIC struct, so they carry from one `reset -p all` to the
     * next -- and across a differential's whole case list, which is why a harness that models a
     * fresh simulator process has to call this and a harness that models a per-case reset must not.
     *
     * @this {RQVAX}
     */
    powerUp()
    {
        this.perr = 0;
        this.saw = 0;
        this.prgi = 0;
        this.irq = 0;
        this.cq = {ioff: 0, ba: 0, lnt: 0, idx: 0};
        this.rq = {ioff: 0, ba: 0, lnt: 0, idx: 0};
        this.reset();
    }

    /**
     * reset()
     *
     * rq_reset() (pdp11_rq.c:3265-3366), minus the one-time `plugs_inited` unit-table construction
     * (this tree builds its units in the constructor instead) and minus `auto_config()` (nothing
     * here moves a device's address at run time; the base is fixed by the caller and checked against
     * the oracle's own autoconfiguration by mscpinitdiff.js).
     *
     * `sim_cancel (uptr)` for every unit is reproduced by clearing the queue deadline: an IP write
     * lands here, and a pending SA answer must NOT arrive afterwards.  That is observable -- write
     * SA, then write IP before the 450 instructions elapse, and the controller must stay in CST_S1
     * with SA = 0x0B40 rather than stepping to CST_S2 -- and it is graded.
     *
     * `perr`, `saw`, `prgi` and the rings' `ioff` are deliberately NOT cleared here -- see
     * powerUp() above for the full list and for what goes wrong if they are.
     *
     * @this {RQVAX}
     */
    reset()
    {
        this.queDue = null;                                 /* sim_cancel (all units) */
        this.csta = CST_S1;                                 /* init stage 1 */
        this.s1dat = 0;                                     /* no S1 data */
        this.vec = 0;                                       /* no vector */
        this.comm = 0;                                      /* no comm region */
        /* UNIBUS ? SA_S1|SA_S1C_DI|SA_S1C_MP : + SA_S1C_Q22.  A microvax3900 is a Qbus machine, so
           the Q22 arm is the one this tree takes -- and the measured 0x0B40 is that arm's value. */
        this.sa = (SA_S1 | SA_S1C_Q22 | SA_S1C_DI | SA_S1C_MP) | 0;
        this.cflgs = CF_RPL;                                /* ctrl flgs off */
        this.htmo = RQ_DHTMO;                               /* default timeout */
        this.hat = this.htmo;                               /* default timer */
        this.cq.ba = this.cq.lnt = this.cq.idx = 0;         /* clr cmd ring -- NOT ioff */
        this.rq.ba = this.rq.lnt = this.rq.idx = 0;         /* clr rsp ring -- NOT ioff */
        this.credits = (RQ_NPKTS / 2) - 1;                  /* init credits */
        this.freq = 1;                                      /* init free list */
        for (let i = 0; i < RQ_NPKTS; i++) {
            this.pakLink[i] = i ? ((i + 1) & RQ_M_NPKTS) : 0;
            for (let j = 0; j < RQ_PKT_SIZE_W; j++) this.pakData[i * RQ_PKT_SIZE_W + j] = 0;
        }
        this.rspq = 0;                                      /* no q'd rsp pkts */
        this.pbsy = 0;                                      /* all pkts free */
        this.pip = 0;                                       /* not polling */
        this.irq = 0;                                       /* rq_clrint */
        /* `uptr->flags = uptr->flags & ~(UNIT_ONL | UNIT_ATP); uptr->uf = 0; uptr->cpkt =
           uptr->pktq = 0;` -- pdp11_rq.c:3350-3357.  NOTE WHAT SURVIVES: the ATTACHMENT, the drive
           TYPE, the CAPACITY autosize computed and the write lock.  `reset -p all` does not detach a
           disk, and a model that dropped the image on reset would make every case after the first
           look unattached. */
        for (let u of this.units) {
            u.flags = u.flags & ~(U_ONL | U_ATP);
            u.uf = 0;
            u.cpkt = 0;
            u.pktq = 0;
        }
        /* `for (i = cp->max_plug = 0; i < numunits - 2; i++) if (!DIS && plug > max_plug) max_plug =
           plug;` -- recomputed on every reset, not maintained, and read by exactly one thing:
           rq_gus()'s MD_NXU wrap. */
        this.maxPlug = 0;
        for (let u of this.units) {
            if (!(u.flags & U_DIS) && u.plug > this.maxPlug) this.maxPlug = u.plug;
        }
    }

    /* --------------------------------------------------------------------------------------- *
     * Units: the SCP-level operations a user performs on a drive (pcjsvax-f52)                  *
     *                                                                                           *
     * These are not MSCP commands and no guest instruction reaches them -- they are `set rqN     *
     * <type>`, `set rqN locked`, `attach rqN <file>` and `detach rqN`, i.e. the acts by which a   *
     * USER supplies an image.  They are graded exactly as the MSCP commands are: the differential *
     * performs the same sequence on both engines and compares `examine rq capac[n]`, the unit     *
     * flags and every field of every response that depends on them.                              *
     * --------------------------------------------------------------------------------------- */

    /**
     * unit(i)
     *
     * The drive at INDEX i -- the SCP handle (`rqN`), not the MSCP logical unit.  Distinguished from
     * getucb() below on purpose: `set rq0 ...` addresses an index and an MSCP packet addresses a
     * PLUG, and conflating the two is invisible until a plug is changed.
     *
     * @this {RQVAX}
     * @param {number} i
     * @returns {Object}
     */
    unit(i)
    {
        if (!(i >= 0 && i < this.units.length)) {
            throw new Error(`rq.js: there is no unit ${i}; this controller has ${this.units.length}`);
        }
        return this.units[i];
    }

    /**
     * setType(i, dtype)
     *
     * rq_set_type() (pdp11_rq.c:3107-3127), minus the RA8U_DTYPE user-capacity argument (which
     * REWRITES drv_tab[RA8U].lbn in place -- a mutable drive table is not something this tree
     * reproduces, and no graded case passes a capacity).  Two things it does and both are graded:
     * it REFUSES while the unit is attached (SCPE_ALATT), and it resets `capac` to the type's own
     * LBN count -- which is what makes a later attach's autosize comparison start from the type
     * rather than from whatever the previous attach left behind.
     *
     * @this {RQVAX}
     * @param {number} i
     * @param {number} dtype
     */
    setType(i, dtype)
    {
        let u = this.unit(i);
        if (u.flags & U_ATT) {
            throw new Error("rq.js: setType() while the unit is ATTACHED -- rq_set_type() returns " +
                "SCPE_ALATT and changes nothing, so a caller that did this on one engine and not " +
                "the other would be comparing two different drives");
        }
        if (!(dtype >= 0 && dtype < DRV_TAB.length)) {
            throw new Error(`rq.js: drive type ${dtype} is not in drv_tab[]`);
        }
        u.dtype = dtype;
        u.capac = DRV_TAB[dtype].lbn;
    }

    /**
     * setWriteLock(i, locked)
     *
     * rq_set_wlk() (pdp11_rq.c:3083-3091) over scp.c's set_writelock() (scp.c:8085-8100).  Three
     * behaviours, and the middle one is a state dependence that measurement found rather than
     * reading:
     *
     *   1. A drive whose TABLE ENTRY carries RQDF_RO (RRD40, the CD-ROM) cannot be write-ENABLED;
     *      asking returns SCPE_NOFNC and changes nothing.
     *   2. *** LOCKING A DETACHED UNIT ALSO SETS UNIT_RO *** -- the C's own comment is "Next attach
     *      will be Read-Only" -- and that is not cosmetic: sim_disk_attach() tests UNIT_RO, opens
     *      the container read-only, and the read-only-ness then SUPPRESSES AUTOSIZE'S CLAMP, so a
     *      locked unit takes the FILE's size where an unlocked one would keep the drive's.
     *      Measured: `set rq0 locked` + `set rq0 rx33` + a 204,800-byte file gives capac 400, not
     *      RX33's 2400, and `show rq0` prints "read only, write locked".
     *   3. The guard at the top is `((flags & UNIT_WPRT) != 0) == val`, i.e. ALREADY AS DESIRED IS A
     *      NO-OP -- so locking a unit that is still UNIT_WLK from a previous attach does NOT re-set
     *      UNIT_RO, which detach cleared.  A caller that wants a deterministic starting point must
     *      write-ENABLE first; the differential's per-case setup does exactly that, and this is why.
     *   4. Write-enabling a unit ATTACHED READ-ONLY is refused (SCPE_ALATT).
     *
     * @this {RQVAX}
     * @param {number} i
     * @param {boolean} locked
     */
    setWriteLock(i, locked)
    {
        let u = this.unit(i);
        if ((DRV_TAB[u.dtype].flgs & RQDF_RO) && !locked) {
            throw new Error(`rq.js: ${DRV_TAB[u.dtype].name} carries RQDF_RO, so rq_set_wlk() ` +
                `refuses to enable writing on it (SCPE_NOFNC)`);
        }
        if (((u.flags & (U_WLK | U_RO)) !== 0) === !!locked) return;    /* already as desired */
        if (locked) {
            u.flags = u.flags | U_WLK;
            if (!(u.flags & U_ATT)) u.flags = u.flags | U_RO;   /* next attach will be read-only */
        } else if ((u.flags & U_ATT) && (u.flags & U_RO)) {
            throw new Error("rq.js: set_writelock() cannot enable writing on a unit that is " +
                "ATTACHED READ ONLY (SCPE_ALATT)");
        } else {
            u.flags = u.flags & ~(U_WLK | U_RO);
        }
    }

    /**
     * setPlug(i, plug)
     *
     * rq_set_plug() (pdp11_rq.c:3140-3159) -- `set rqN UNIT=<val>`.  THE PLUG IS THE NUMBER MSCP
     * PACKETS ADDRESS THE UNIT BY, and it is NOT the array index: rq_getucb() matches on it, and
     * `set rq0 unit=5` makes DUA5 the first drive.  Modelling it as the index is a defect no case
     * with default plugs can see, which is why the differential has cases that remap them.
     *
     * DUPLICATES ARE REFUSED, scanning `numunits - 2` units.  An unchanged plug is an early OK,
     * BEFORE the duplicate scan, so re-setting a unit to the plug it already has always works.
     *
     * *** `numunits - 2` IS 254 AND NOT 4, AND THAT IS OBSERVABLE. ***  The DEVICE declares
     * RQ_MAXDR + 2 units; rq_reset()'s one-time plug initialisation gives EVERY one of the 254
     * potential drives `unit_plug = d` and marks the ones past RQ_NUMDR as UNIT_DIS.  They are
     * invisible to rq_getucb() and to the `max_plug` scan, which both skip UNIT_DIS -- but
     * rq_set_plug()'s duplicate scan does NOT skip it, so PLUGS 4..253 ARE ALL TAKEN by drives that
     * do not exist.  Measured, after this cost the differential a debugging round: `set rq0 unit=7`
     * on the stock oracle answers "%SIM-ERROR: Unit Plug 7 Already In Use on RQ7".
     *
     * This tree models four drives, so it cannot refuse those plugs by finding the unit that holds
     * them; RESERVED_PLUGS below is that range, named.  It is a HARNESS GUARD and not a graded
     * behaviour: it turns "the differential asked for a plug the simulator would have refused, and
     * the two engines silently diverged" into an immediate, named error.  Nothing grades it, and
     * this comment does not claim otherwise.
     *
     * NOTE `max_plug` IS NOT RECOMPUTED HERE.  Only reset() does that, so a plug changed after a
     * reset leaves rq_gus()'s MD_NXU wrap using the OLD maximum until the next one -- which is why
     * the differential sets every plug BEFORE its `reset -p all` rather than after.
     *
     * @this {RQVAX}
     * @param {number} i
     * @param {number} plug
     */
    setPlug(i, plug)
    {
        let u = this.unit(i);
        if (!(plug >= 0 && plug <= 65534)) throw new Error(`rq.js: invalid unit plug ${plug}`);
        if (u.plug === plug) return;                        /* unchanged: OK before the scan */
        for (let o of this.units) {
            if (o.plug === plug) throw new Error(`rq.js: unit plug ${plug} is already in use`);
        }
        if (plug >= RQ_NUMDR && plug < RQ_MAXDR) {
            throw new Error(`rq.js: unit plug ${plug} is held by rq_unit[${plug}], one of the ` +
                `${RQ_MAXDR - RQ_NUMDR} DISABLED potential drives the DEVICE declares beyond the ` +
                `four this tree models.  rq_set_plug()'s duplicate scan walks all ${RQ_MAXDR} of ` +
                `them and does not skip UNIT_DIS, so the simulator answers "Unit Plug ${plug} ` +
                `Already In Use on RQ${plug}" and changes nothing.  Choose a plug below ` +
                `${RQ_NUMDR} (freeing it from its own unit first) or at or above ${RQ_MAXDR}.`);
        }
        u.plug = plug;
    }

    /**
     * setEnabled(i, on)
     *
     * `set rqN ENABLED` / `set rqN DISABLED`, i.e. UNIT_DIS.  A disabled unit is INVISIBLE TO MSCP:
     * rq_getucb() skips it, so its plug answers ST_OFL (bare) as though no such drive existed, and
     * reset()'s `max_plug` scan skips it too -- so disabling the highest-plug drive MOVES the point
     * at which rq_gus()'s MD_NXU walk wraps.  Both halves are graded.
     *
     * @this {RQVAX}
     * @param {number} i
     * @param {boolean} on
     */
    setEnabled(i, on)
    {
        let u = this.unit(i);
        if (!on && (u.flags & U_ATT)) {
            throw new Error("rq.js: refusing to disable an ATTACHED unit -- SCP refuses too, and " +
                "a unit that was disabled while holding a container is a state neither engine has");
        }
        u.flags = on ? (u.flags & ~U_DIS) : (u.flags | U_DIS);
    }

    /**
     * attach(i, provider, opts)
     *
     * rq_attach() (pdp11_rq.c:3229-3249) and the ONE arm of sim_disk_attach_ex2() a raw
     * user-supplied container reaches (sim_disk.c:3270-3286, "Autosize by changing capacity").
     * See the file header for the measurement that says it is that arm and not the drive-type walk.
     *
     *   0. `if ((sim_switches & SWMASK ('R')) || (uptr->flags & UNIT_RO))` -- the attachment is
     *      read-only if the CALLER asked (`attach -R`) *or* if the unit already carries UNIT_RO,
     *      which `set rqN locked` on a detached unit installs (see setWriteLock()).
     *   1. `if (drv_tab[dtyp].flgs & RQDF_RO) { sim_switches |= SWMASK ('R'); }` -- a read-only
     *      DRIVE TYPE forces a read-only ATTACHMENT whatever the caller asked for.
     *   2. `if (container < current && !UNIT_RO) container = current` -- a container SMALLER than
     *      the drive leaves the drive its own size, but ONLY if it is writable; a read-only
     *      attachment of a small file shrinks the unit to the file.  (Measured: an RX33 with
     *      `attach -R` and a 409,600-byte file reports capac 800, not RX33's 2400.)
     *   3. `capac = container / 512`, TRUNCATING -- a container that is not a whole number of
     *      blocks loses its tail, and that partial block is not reachable by any LBN.
     *   4. `if ((csta == CST_UP) && isavailable) flags |= UNIT_ATP` -- an attention is PENDING, and
     *      it is delivered by rq_tmrsvc(), the wall-clock timer this tree does not model.  The flag
     *      is still set because it is state, and the differential's HAT == HTMO fence is what keeps
     *      the timer from firing in a graded case.
     *
     * @this {RQVAX}
     * @param {number} i
     * @param {Object} provider {byteLength, read()} -- see checkProvider()
     * @param {Object} [opts] {readOnly}
     */
    attach(i, provider, opts = {})
    {
        let u = this.unit(i);
        if (u.flags & U_ATT) throw new Error(`rq.js: unit ${i} is already attached`);
        if (u.flags & U_NOAUTO) {
            throw new RQUnimplemented("rq.js: attach() with NOAUTOSIZE selects sim_disk's OTHER " +
                "autosize arm, which walks drv_types[] and needs get_filesystem_size() to have " +
                "parsed a real volume out of the container -- a file-system parser is not a disk " +
                "controller and is out of pcjsvax-f52's scope");
        }
        checkProvider(provider);

        let readOnly = !!opts.readOnly || !!(u.flags & U_RO);
        if (DRV_TAB[u.dtype].flgs & RQDF_RO) readOnly = true;

        let container = provider.byteLength;
        let current = u.capac * RQ_NUMBY;
        if (container < current && !readOnly) container = current;
        u.capac = Math.floor(container / RQ_NUMBY);

        u.image = provider;
        u.flags = u.flags | U_ATT | (readOnly ? U_RO : 0);
        if (this.csta === CST_UP && this.isAvailable(u)) u.flags = u.flags | U_ATP;
    }

    /**
     * detach(i)
     *
     * rq_detach() (pdp11_rq.c:3251-3262) over scp.c's detach_unit().  `uptr->flags &= ~(UNIT_ATT |
     * (UNIT_ROABLE ? UNIT_RO : 0))` and then `&= ~(UNIT_ONL | UNIT_ATP); uptr->uf = 0`.  RQ units
     * ARE UNIT_ROABLE, so a `-R` attachment's read-only-ness does NOT survive the detach -- which
     * matters because a differential that attached read-only once would otherwise see UF_WPH for
     * the rest of the run.  `capac` is NOT restored: whatever autosize computed stays until the next
     * `set rqN <type>`.
     *
     * @this {RQVAX}
     * @param {number} i
     */
    detach(i)
    {
        let u = this.unit(i);
        if (!(u.flags & U_ATT)) return;                     /* SCPE_UNATT: nothing happens */
        u.flags = u.flags & ~(U_ATT | U_RO | U_ONL | U_ATP);
        u.uf = 0;
        u.image = null;
    }

    /**
     * isAvailable(u)
     *
     * sim_disk_isavailable() (sim_disk.c).  MEASURED RATHER THAN ASSUMED, and the answer is simpler
     * than the function: a user-supplied file is opened in RAW format (sim_disk's AUTO detection
     * tries a raw open first and a plain file opens fine), so the RAW arm runs -- and that arm's
     * `sim_os_disk_isavailable_raw()` is `return TRUE;` on every platform without
     * IOCTL_STORAGE_EJECT_MEDIA, i.e. everywhere but Windows.  `media_removed` is the only thing
     * that could make it false and only sim_disk_unload() sets it, which is excluded by name (see
     * the file header) and fenced in the differential.
     *
     * It is a METHOD rather than an inlined `u.flags & U_ATT` so that --selfcheck can perturb the
     * predicate rq_onl() consults without substituting a copy of rq_onl() (HANDOFF.md rule 11).
     *
     * @this {RQVAX}
     * @param {Object} u
     * @returns {boolean}
     */
    isAvailable(u) { return (u.flags & U_ATT) !== 0; }

    /**
     * getucb(lu)
     *
     * rq_getucb() (pdp11_rq.c:2925-2935): `for (i = 0; i < dptr->numunits - 2; i++) if ((lu ==
     * units[i].unit_plug) && !(units[i].flags & UNIT_DIS)) return &units[i];`
     *
     * *** THE `- 2` IS NOT AN OFF-BY-ONE GUARD, IT IS THE TWO PSEUDO-UNITS. ***  See the file
     * header.  This walk has no such subtraction because `units` holds only drives; the constructor
     * asserts that, so the safety is structural rather than remembered.
     *
     * MATCHED ON THE PLUG, NOT THE INDEX -- and the plug is settable, so unit 0 need not be DUA0.
     *
     * @this {RQVAX}
     * @param {number} lu
     * @returns {?Object}
     */
    getucb(lu)
    {
        for (let u of this.units) if (u.plug === lu && !(u.flags & U_DIS)) return u;
        return null;
    }

    /**
     * wph(u) / rmv(u)
     *
     * RQ_WPH() and RQ_RMV() (pdp11_rq.c:177-180).  RQ_WPH is
     * `(drv_tab[dtyp].flgs & RQDF_RO) || (u->flags & UNIT_WPRT) || sim_disk_wrp (u)` -- and the last
     * two are THE SAME PREDICATE in this build: UNIT_WPRT is `UNIT_WLK|UNIT_RO` and sim_disk_wrp()
     * is `flags & DKUF_WRP`, which is `DKUF_WLK|UNIT_RO`.  Written as the C's three terms anyway,
     * because the C has three, and split out as methods so --selfcheck can perturb them.
     *
     * @this {RQVAX}
     * @param {Object} u
     * @returns {number} UF_WPH or 0
     */
    wph(u)
    {
        return ((DRV_TAB[u.dtype].flgs & RQDF_RO) || (u.flags & (U_WLK | U_RO))) ? UF_WPH : 0;
    }

    rmv(u) { return (DRV_TAB[u.dtype].flgs & RQDF_RMV) ? UF_RMV : 0; }

    /* --------------------------------------------------------------------------------------- *
     * The event queue.  See the file header for why the unit is cpu.nTotalCycles.               *
     * --------------------------------------------------------------------------------------- */

    /**
     * activateQueue(delay)
     *
     * `sim_activate (dptr->units + RQ_QUEUE, delay)`, INCLUDING scp.c's `if (sim_is_active (uptr))
     * return SCPE_OK;` -- an already-armed unit is NOT re-armed.  Dropping that guard is a defect
     * this file's differential has a mutation for, because it is invisible to any case that writes
     * SA once and waits.
     *
     * `this.cpu` is set by tick() on the first instruction boundary and is what supplies "now".  A
     * device that is written to before the CPU has ever ticked has no clock to schedule against;
     * that cannot happen through an instruction (the write IS an instruction) and asserting it here
     * turns a wiring mistake into a named failure instead of an event that never fires.
     *
     * @this {RQVAX}
     * @param {number} delay in instructions
     */
    activateQueue(delay)
    {
        if (this.queDue !== null) return;                   /* sim_is_active -> no-op */
        if (!this.cpu) throw new Error("rq.js: activateQueue() with no CPU clock -- tick() has never run");
        /* Deliberately NOT `| 0`: cpu.nTotalCycles is a running instruction total that passes 2^31
           within a few million instructions of a ROM boot, and an int32 coercion here would make the
           deadline go negative and the event fire immediately, forever. */
        this.queDue = this.cpu.nTotalCycles + delay;
    }

    /**
     * tick(cpu)
     *
     * vax_cpu.c:659's `if (sim_interval <= 0) sim_process_event ();`, at the same point in the same
     * loop.  cpustate.js calls this once per instruction retired, BEFORE that instruction's cycles
     * are charged, so `cpu.nTotalCycles` here is the total for every PREVIOUS instruction -- which
     * is what makes `nTotalCycles >= due` the same predicate as `sim_interval <= 0`.
     *
     * @this {RQVAX}
     * @param {Object} cpu
     */
    tick(cpu)
    {
        this.cpu = cpu;
        if (this.queDue !== null && cpu.nTotalCycles >= this.queDue) {
            this.queDue = null;
            this.quesvc();
        }
    }

    /**
     * drainOnHalt(cpu)
     *
     * The HALT INSTRUCTION's own event drain -- vax_cpu.c:2643-2652, called from exc.js's HALT
     * handler, which carries the measurement.  `sim_interval = 0; sim_process_event ()` in a loop
     * services the queued unit whatever remains of its delay, and repeats while the head of the
     * queue is not UNIT_IDLE.  RQ_QUEUE's flags are `UNIT_DIS` with no UNIT_IDLE, so it is drained;
     * RQ_TIMER's are `UNIT_IDLE|UNIT_DIS`, so the C's loop would stop AT it, which is one more
     * reason the wall-clock timer's absence here is not observable.
     *
     * THE LOOP BOUND IS LOAD-BEARING AND THE C HAS NONE.  rq_quesvc() re-arms its own unit whenever
     * `pkt` is non-zero, so the C's drain runs again -- correctly, while there is work.  But a
     * response queue whose ring the host never grants is work that never finishes: the thread
     * dequeues the packet, fails to place it, pushes it back and re-arms, forever.  *** THE REAL
     * SIMULATOR HANGS IN ITS HALT INSTRUCTION IF A HOST HALTS WITH A DEFERRED RESPONSE AND NO FREE
     * RESPONSE DESCRIPTOR *** -- measured, not deduced, and the reason tests/mscpringdiff.js's
     * response-ring-full case grants the descriptor BEFORE it halts.  Naming that by exception here
     * beats hanging the differential in sympathy with the oracle.
     *
     * The bound is `RQ_NPKTS * 2 + 4`: at most every packet in the pool can be delivered, each
     * costing its own service plus the re-arm, with slack for the poll that finds the ring empty.
     *
     * @this {RQVAX}
     * @param {Object} cpu
     */
    drainOnHalt(cpu)
    {
        this.cpu = cpu;
        for (let n = 0; this.queDue !== null; n++) {
            if (n > RQ_NPKTS * 2 + 4) {
                throw new Error("rq.js: drainOnHalt() ran " + n + " services without the queue " +
                    "going idle -- rq_quesvc() is re-arming itself without making progress.  The " +
                    "usual cause is a deferred response with no host-owned response descriptor, " +
                    "which hangs the real simulator's HALT the same way.");
            }
            this.queDue = null;
            this.quesvc();
        }
    }

    /* --------------------------------------------------------------------------------------- *
     * The two registers                                                                         *
     * --------------------------------------------------------------------------------------- */

    /**
     * rd(pa)
     *
     * rq_rd() (pdp11_rq.c:1650-1681).  `*data = 0` for IP is UNCONDITIONAL and precedes the state
     * dispatch, so the value a read of IP returns tells you nothing about what the read DID.
     *
     * @this {RQVAX}
     * @param {number} pa
     * @returns {number} the 16-bit word
     */
    rd(pa)
    {
        if (((pa >>> 1) & 1) === 0) {                       /* IP */
            if (this.csta === CST_S3_PPB) {                 /* waiting for poll? */
                this.step4();
            } else if (this.csta === CST_UP) {              /* if up */
                /* OLDPC is `fault_PC` on the VAX arm (pdp11_rq.c:108) -- the start PC of the
                   instruction doing the read, not the PC after it.  `this.cpu` is whatever tick()
                   last saw, and tick() runs at the top of every instruction, so it is current. */
                this.traceReq("poll started, PC=" +
                    ((this.cpu ? this.cpu.faultPC : 0) >>> 0).toString(16).toUpperCase());
                this.pip = 1;                               /* poll host */
                this.activateQueue(this.qtime);
            }
            return 0;                                       /* reads zero */
        }
        return this.sa & 0xFFFF;                            /* SA */
    }

    /**
     * wr(pa, data)
     *
     * rq_wr() (pdp11_rq.c:1683-1713).  Note what is NOT here: no masking of `data`, no test of its
     * value for the IP case ("write IP with ANY value" is the C, literally), and no `else` on the
     * CST_S4 test -- a SA write in CST_UP or CST_DEAD updates `saw` and schedules NOTHING.
     *
     * @this {RQVAX}
     * @param {number} pa
     * @param {number} data
     */
    wr(pa, data)
    {
        if (((pa >>> 1) & 1) === 0) {                       /* IP */
            this.reset();                                   /* init device */
            this.traceReq("initialization started");
            return;
        }
        this.saw = data & 0xFFFF;                           /* SA */
        if (this.csta < CST_S4) this.activateQueue(this.itime);
        else if (this.csta === CST_S4) this.activateQueue(this.itime4);
    }

    /* --------------------------------------------------------------------------------------- *
     * The Qbus I/O-page mount (regblock.js sub-device contract: null / false == "not decoded")   *
     *                                                                                           *
     * THE QBUS IS A WORD BUS, and this mount is written the way cqipc.js's DBLVAX is, for the     *
     * same measured reason: vax_io.c's ReadIO() positions the word with `<< ((pa & 2) ? 16 : 0)`  *
     * and vax_mmu.h's ReadB()/ReadW() extract with the SAME shift, so the two cancel and the lane *
     * WITHIN the answer is `pa & 1`.  Which of the TWO word slots answers is `(pa >> 1) & 1`, and *
     * that is rq_rd()/rq_wr()'s own decode -- so the slot select is not duplicated here, it is    *
     * passed straight through as the address.                                                    *
     * --------------------------------------------------------------------------------------- */

    /**
     * readLong(addr)
     *
     * ReadIO(pa, L_LONG) is `(ReadQb (pa + 2) << 16) | ReadQb (pa)` -- TWO Qbus cycles, and unlike
     * cqipc.js's two-byte doorbell BOTH of them land on this device, because IOLN_RQ is 4.  So an
     * aligned longword read at the base is a real value AND it performs the IP read's side effects.
     * mmu.js's readL() does not split Qbus longword reads (correctly -- in the C the split is inside
     * ReadIO()), so this method IS on a CPU path and is graded through real MOVL instructions.
     *
     * The C evaluates `ReadQb (pa)` first (it is the second operand of the `|`, but it is also the
     * assignment `iod = ReadQb (pa)` on the preceding line), so the IP side effect happens BEFORE
     * the SA fetch.  With `pa` at the base that ordering is observable: a longword read in
     * CST_S3_PPB completes step 4 through the IP half and then reads back the step-4 SA in the same
     * instruction.
     *
     * @this {RQVAX}
     * @param {number} addr
     * @returns {?number}
     */
    readLong(addr)
    {
        addr = addr >>> 0;
        if (addr + 4 > this.base + IOLN_RQ) return null;    /* second word slot is not ours */
        let lo = this.rd(addr) & 0xFFFF;
        let hi = this.rd((addr + 2) >>> 0) & 0xFFFF;
        return ((hi << 16) | lo) | 0;
    }

    /**
     * @this {RQVAX}
     * @param {number} addr
     * @returns {number}
     */
    readWord(addr) { return this.rd(addr >>> 0) & 0xFFFF; }

    /**
     * @this {RQVAX}
     * @param {number} addr
     * @returns {number}
     */
    readByte(addr) { return (this.rd(addr >>> 0) >>> ((addr & 1) << 3)) & 0xFF; }

    /**
     * writeLong(addr, val)
     *
     * FALSE, and it performs no write.  mmu.js's writeL() already splits an aligned Qbus longword
     * write into two setWord() cycles -- which is WriteIO(pa, val, L_LONG) exactly -- so both halves
     * reach writeWord() below and this method is not on any CPU path.  It returns false rather than
     * silently accepting, so a caller that bypasses writeL() gets a fault it can see instead of a
     * write whose halves were applied in the wrong order.  Same reasoning, same shape, as DBLVAX's.
     *
     * @this {RQVAX}
     * @param {number} addr
     * @param {number} val
     * @returns {boolean}
     */
    writeLong(addr, val) { return false; }

    /**
     * writeWord(addr, val)
     *
     * @this {RQVAX}
     * @param {number} addr
     * @param {number} val
     * @returns {boolean}
     */
    writeWord(addr, val) { this.wr(addr >>> 0, val & 0xFFFF); return true; }

    /**
     * writeByte(addr, val)
     *
     * WriteQb (pa, val, WRITEB) reaches rq_wr() with `data` = the BYTE, right-justified, and rq_wr()
     * uses `data` unshifted and unmerged -- so a byte write of 0x01 to base+2 sets `saw` to 0x0001,
     * and a byte write to base+3 sets `saw` to the byte value too, NOT to `byte << 8`.  That looks
     * like a defect and is the C; it is graded on both engines rather than tidied.
     *
     * @this {RQVAX}
     * @param {number} addr
     * @param {number} val
     * @returns {boolean}
     */
    writeByte(addr, val) { this.wr(addr >>> 0, val & 0xFF); return true; }

    /* --------------------------------------------------------------------------------------- *
     * The state machine                                                                         *
     * --------------------------------------------------------------------------------------- */

    /**
     * ringLenRQ(s1dat) / ringLenCQ(s1dat)
     *
     * SA_S1H_RQ(x) / SA_S1H_CQ(x), then rq_step4()'s own `<< 2`.  Split out and published as METHODS
     * so tests/mscpinitdiff.js's --selfcheck can perturb the decode (drop the shift, swap the two
     * field positions) without substituting a copy of rq_step4() -- HANDOFF.md standing rule 11.
     * They are on the shipped path: rq_step4() below has no other source for a ring length.
     *
     * @this {RQVAX}
     * @param {number} s1dat
     * @returns {number} ring length IN BYTES
     */
    ringLenRQ(s1dat) { return (1 << ((s1dat >>> SA_S1H_V_RQ) & SA_S1H_M_RQ)) << 2; }

    ringLenCQ(s1dat) { return (1 << ((s1dat >>> SA_S1H_V_CQ) & SA_S1H_M_CQ)) << 2; }

    /**
     * echoS2(s1dat) / echoS3(s1dat)
     *
     * SA_S2C_EC(x) / SA_S3C_EC(x) -- the bytes of the HOST's own S1 data that the controller echoes
     * back in its step-2 and step-3 answers.  Published for the same reason as the ring lengths: the
     * echo is the one part of the SA sequence that a differential could otherwise satisfy with a
     * constant, and the cheat this item exists to make impossible is exactly a replayed constant.
     *
     * @this {RQVAX}
     * @param {number} s1dat
     * @returns {number}
     */
    echoS2(s1dat) { return (s1dat >>> SA_S2C_V_EC) & SA_S2C_M_EC; }

    echoS3(s1dat) { return (s1dat >>> SA_S3C_V_EC) & SA_S3C_M_EC; }

    /**
     * step4()
     *
     * rq_step4() (pdp11_rq.c:1735-1765).  The comm region is zeroed with `Map_WriteW (base, lnt,
     * zero)` -- a DMA through the CQBIC scatter-gather map, NOT a memory store.  Writing it as a
     * store passes every case whose map happens to be an identity mapping and fails the moment the
     * host scatters the region, which is why the measured PE_QWE fatal (map unprogrammed) is a
     * graded case rather than a footnote.
     *
     * `zero` is allocated per call in the C (a 1032-byte stack array).  Here it is a lazily-created
     * instance buffer of the same maximum size, reused: HANDOFF.md standing rule 14 is about exactly
     * this kind of per-operation allocation, and the buffer's CONTENTS are re-zeroed each time so
     * reuse cannot leak a previous call's data into the DMA.
     *
     * @this {RQVAX}
     * @returns {boolean} true on success; false if the controller went fatal
     */
    step4()
    {
        this.rq.ioff = SA_COMM_RI;                          /* set intr offset */
        this.rq.ba = this.comm;                             /* set rsp q base */
        this.rq.lnt = this.ringLenRQ(this.s1dat);           /* get resp q len */
        this.cq.ioff = SA_COMM_CI;                          /* set intr offset */
        this.cq.ba = (this.comm + this.rq.lnt) >>> 0;       /* set cmd q base */
        this.cq.lnt = this.ringLenCQ(this.s1dat);           /* get cmd q len */
        this.cq.idx = this.rq.idx = 0;                      /* clear q idx's */
        let base = this.prgi ? (this.comm + SA_COMM_QQ) : (this.comm + SA_COMM_CI);
        let lnt = this.comm + this.cq.lnt + this.rq.lnt - base;     /* comm lnt */
        if (lnt > SA_COMM_MAX) lnt = SA_COMM_MAX;           /* paranoia */
        if (!this.zeroBuf || this.zeroBuf.length < SA_COMM_MAX) this.zeroBuf = new Uint8Array(SA_COMM_MAX);
        this.zeroBuf.fill(0, 0, lnt);
        if (this.cqbic.mapWriteW(base >>> 0, lnt, this.zeroBuf)) {  /* zero comm area */
            return this.fatal(PE_QWE);                      /* error? */
        }
        this.sa = (SA_S4 |                                  /* send step 4 */
            (RQVAX.CTLR_TAB[this.ctype].uqpm << SA_S4C_V_MOD) |
            (RQVAX.RQ_SVER << SA_S4C_V_VER)) & 0xFFFF;
        this.csta = CST_S4;                                 /* set step 4 */
        this.initInt();                                     /* poke host */
        return true;
    }

    /**
     * initInt()
     *
     * rq_init_int() (pdp11_rq.c:2991-2997).  Both conditions, and the request is recorded rather
     * than delivered -- see the file header's INTERRUPTS exclusion, and the fence in
     * mscpinitdiff.js that keeps any graded case from setting both bits.
     *
     * @this {RQVAX}
     */
    initInt()
    {
        if ((this.s1dat & SA_S1H_IE) && (this.s1dat & SA_S1H_VEC)) this.irq = 1;
    }

    /**
     * fatal(err)
     *
     * rq_fatal() (pdp11_rq.c:3067-3078).  ORDER IS THE WHOLE THING: reset() FIRST -- which wipes
     * comm, the rings, csta and sa, and cancels the pending event -- and only then are `sa`, `csta`
     * and `perr` set.  A version that set them first and reset afterwards would leave SA at 0x0B40
     * and csta at CST_S1, which is what the oracle would have shown if this were wrong.
     *
     * @this {RQVAX}
     * @param {number} err
     * @returns {boolean} false, the C's ERR
     */
    fatal(err)
    {
        this.traceReq("fatal err=" + (err >>> 0).toString(16).toUpperCase());
        this.reset();                                       /* reset device */
        this.sa = (SA_ER | err) & 0xFFFF;                   /* SA = dead code */
        this.csta = CST_DEAD;                               /* state = dead */
        this.perr = err | 0;                                /* save error */
        return false;
    }

    /**
     * quesvc()
     *
     * rq_quesvc() (pdp11_rq.c:1777-1895).  The `csta < CST_UP` half is complete; the CST_UP half is
     * implemented only as far as an EMPTY command ring reaches -- see the file header's exclusions.
     *
     * @this {RQVAX}
     */
    quesvc()
    {
        if (this.csta < CST_UP) {                           /* still init? */
            switch (this.csta) {
            case CST_S1:                                    /* need S1 reply */
                if (this.saw & SA_S1H_VL) {                 /* valid? */
                    if (this.saw & SA_S1H_WR) {             /* wrap? */
                        this.sa = this.saw & 0xFFFF;        /* echo data */
                        this.csta = CST_S1_WR;              /* endless loop */
                    } else {
                        this.s1dat = this.saw & 0xFFFF;     /* save data */
                        this.vec = (this.s1dat & SA_S1H_VEC) << 2;  /* get vector */
                        this.sa = (SA_S2 | SA_S2C_PT | this.echoS2(this.s1dat)) & 0xFFFF;
                        this.csta = CST_S2;                 /* now in step 2 */
                        this.initInt();                     /* intr if req */
                    }
                }                                           /* end if valid */
                break;

            case CST_S1_WR:                                 /* wrap mode */
                this.sa = this.saw & 0xFFFF;                /* echo data */
                break;

            case CST_S2:                                    /* need S2 reply */
                this.comm = this.saw & SA_S2H_CLO;          /* get low addr */
                this.prgi = this.saw & SA_S2H_PI;           /* get purge int */
                this.sa = (SA_S3 | this.echoS3(this.s1dat)) & 0xFFFF;
                this.csta = CST_S3;                         /* now in step 3 */
                this.initInt();                             /* intr if req */
                break;

            case CST_S3:                                    /* need S3 reply */
                this.comm = (((this.saw & SA_S3H_CHI) << 16) | this.comm) >>> 0;
                if (this.saw & SA_S3H_PP) {                 /* purge/poll test? */
                    this.sa = 0;                            /* put 0 */
                    this.csta = CST_S3_PPA;                 /* wait for 0 write */
                } else this.step4();                        /* send step 4 */
                break;

            case CST_S3_PPA:                                /* need purge test */
                if (this.saw) this.fatal(PE_PPF);           /* data not zero? */
                else this.csta = CST_S3_PPB;                /* wait for poll */
                break;

            case CST_S4:                                    /* need S4 reply */
                if (this.saw & SA_S4H_GO) {                 /* go set? */
                    this.traceReq("initialization complete");
                    this.csta = CST_UP;                     /* we're up */
                    this.sa = 0;                            /* clear SA */
                    /* sim_activate_after (units + RQ_TIMER, 1000000) -- the wall-clock host-access
                       timer, NOT modelled; see the file header's exclusion and the oracle-side
                       HAT == HTMO assertion that keeps it out of reach. */
                    if ((this.saw & SA_S4H_LF) && this.perr) this.plf();
                    this.perr = 0;
                }
                break;
            }
            return;
        }

        /* csta == CST_UP (or CST_DEAD, which reaches here only if a queue event outlived a fatal --
           it cannot, because fatal() calls reset(), which cancels it).
           `pkt` is the C's own local, and its value at the BOTTOM is what decides whether the queue
           thread re-arms.  Two things about it are easy to get wrong and both are graded:
             - rq_mscp() SUCCEEDING leaves `pkt` non-zero even though rq_putpkt() has already freed
               the packet, so a completed command still costs a SECOND queue service (the one that
               finds the ring empty and clears `pip`).
             - the response-queue arm REASSIGNS `pkt`, so draining a deferred response re-arms too. */
        let pkt = 0;
        for (let u of this.units) {                         /* chk unit q's */
            if (u.cpkt || u.pktq === 0) continue;
            throw new RQUnimplemented("rq.js: a unit queue is non-empty, so a command was deferred " +
                "behind an in-flight transfer -- only rq_rw() sets uptr->cpkt and only a set cpkt " +
                "causes a deferral, so this state belongs to pcjsvax-346");
        }
        if ((pkt === 0) && this.pip) {                      /* polling? */
            let got = this.getPkt();                        /* get host pkt */
            if (!got.ok) return;
            pkt = got.pkt;
            if (pkt) {                                      /* got one? */
                this.traceReq("cmd=" + h4(this.pd(pkt, CMD_OPC)) +
                    "(" + CMD_NAMES[this.pd(pkt, CMD_OPC) & 0x3F].padStart(3) + "), mod=" +
                    h4(this.pd(pkt, CMD_MOD)) + ", unit=" + this.pd(pkt, CMD_UN) +
                    ", bc=" + h4(this.pd(pkt, RW_BCH)) + h4(this.pd(pkt, RW_BCL)) +
                    ", ma=" + h4(this.pd(pkt, RW_BAH)) + h4(this.pd(pkt, RW_BAL)) +
                    ", lbn=" + h4(this.pd(pkt, RW_LBNH)) + h4(this.pd(pkt, RW_LBNL)));
                if (this.getp(pkt, UQ_HCTC, UQ_HCTC_V_TYP, UQ_HCTC_M_TYP) !== UQ_TYP_SEQ) {
                    this.fatal(PE_PIE);                     /* not seq -- term thread */
                    return;
                }
                let cnid = this.getp(pkt, UQ_HCTC, UQ_HCTC_V_CID, UQ_HCTC_M_CID);
                if (cnid === UQ_CID_MSCP) {                 /* MSCP packet? */
                    if (!this.mscp(pkt, true)) return;      /* proc, q non-seq */
                } else if (cnid === UQ_CID_DUP) {           /* DUP packet? */
                    this.putr(pkt, OP.END, 0, ST.CMD | I_OPCD, RSP_LNT, UQ_TYP_SEQ);
                    if (!this.putPkt(pkt, true)) return;    /* ill cmd */
                } else {
                    this.fatal(PE_ICI);                     /* no, term thread */
                    return;
                }
            } else this.pip = 0;                            /* discontinue poll */
        }
        if (this.rspq) {                                    /* resp q? */
            pkt = this.deqh(this, "rspq");                  /* get top of q */
            if (!this.putPkt(pkt, false)) return;           /* send to host */
        }
        if (pkt) this.activateQueue(this.qtime);            /* more to do? */
    }

    /* --------------------------------------------------------------------------------------- *
     * The packet pool: rq_deqf / rq_deqh / rq_enqh / rq_enqt (pdp11_rq.c:2765-2820).            *
     *                                                                                           *
     * The C passes `uint16 *lh` -- a POINTER to whichever list head is being manipulated         *
     * (cp->freq, cp->rspq, uptr->pktq).  JS has no such pointer, so each of these takes the      *
     * OWNING OBJECT and the FIELD NAME instead.  That is a mechanical substitution, not a model  *
     * change: `enqh(this, "freq", p)` and `enqh(unit, "pktq", p)` are the C's two call shapes.   *
     * --------------------------------------------------------------------------------------- */

    /**
     * deqf()
     *
     * rq_deqf() -- dequeue the head of the FREE list, fatal if there is none.  `pbsy` is
     * incremented HERE and decremented only in rq_putpkt()'s successful arm, so a packet that is
     * fetched and then lost to a fatal leaves `pbsy` non-zero until rq_reset() clears it.
     *
     * PACKET 0 IS NOT ON THE FREE LIST and must never be handed out: rq_reset() sets `pak[0].link`
     * to 0 and starts `freq` at 1, and 0 is the LIST TERMINATOR everywhere in this file.  A pool
     * that handed out 0 would make `enqh` a no-op and `if (pkt)` false, so the packet would vanish
     * silently -- which is why mscpringdiff.js carries `packet-0-handed-out-as-free` as a mutation.
     *
     * @this {RQVAX}
     * @returns {?number} the packet index, or null if the controller went fatal (PE_NSR)
     */
    deqf()
    {
        if (this.freq === 0) {                              /* no free pkts?? */
            this.fatal(PE_NSR);
            return null;
        }
        this.pbsy = this.pbsy + 1;                          /* cnt busy pkts */
        let pkt = this.freq;                                /* head of list */
        this.freq = this.pakLink[this.freq];                /* next */
        return pkt;
    }

    /**
     * deqh(obj, key) -- rq_deqh(), dequeue the head of any list.
     * @this {RQVAX}
     * @param {Object} obj
     * @param {string} key
     * @returns {number} the packet, or 0 if the list was empty
     */
    deqh(obj, key)
    {
        let ptr = obj[key];                                 /* head of list */
        if (ptr) obj[key] = this.pakLink[ptr];              /* next */
        return ptr;
    }

    /**
     * enqh(obj, key, pkt) -- rq_enqh(), push onto the head.
     * @this {RQVAX}
     * @param {Object} obj
     * @param {string} key
     * @param {number} pkt
     */
    enqh(obj, key, pkt)
    {
        if (pkt === 0) return;                              /* any pkt? */
        this.pakLink[pkt] = obj[key];                       /* link is old lh */
        obj[key] = pkt;                                     /* pkt is new lh */
    }

    /**
     * enqt(obj, key, pkt) -- rq_enqt(), append at the tail by chasing the links.
     * @this {RQVAX}
     * @param {Object} obj
     * @param {string} key
     * @param {number} pkt
     */
    enqt(obj, key, pkt)
    {
        if (pkt === 0) return;                              /* any pkt? */
        this.pakLink[pkt] = 0;                              /* it will be tail */
        if (obj[key] === 0) obj[key] = pkt;                 /* if empty, enqh */
        else {
            let ptr = obj[key];                             /* chase to end */
            while (this.pakLink[ptr]) ptr = this.pakLink[ptr];
            this.pakLink[ptr] = pkt;                        /* enq at tail */
        }
    }

    /* --------------------------------------------------------------------------------------- *
     * Packet field access.  `pak[p].d[w]` is a uint16 array in the C and a flat Uint16Array here. *
     * --------------------------------------------------------------------------------------- */

    /**
     * pd(pkt, w) / spd(pkt, w, v) -- read / write `cp->pak[pkt].d[w]`.
     * @this {RQVAX}
     * @param {number} pkt
     * @param {number} w
     * @returns {number}
     */
    pd(pkt, w) { return this.pakData[pkt * RQ_PKT_SIZE_W + w] & 0xFFFF; }

    spd(pkt, w, v) { this.pakData[pkt * RQ_PKT_SIZE_W + w] = v & 0xFFFF; }

    /**
     * getp(pkt, w, shift, mask) -- the C's GETP(p,w,f) macro, `(d[w] >> w##_V_##f) & w##_M_##f`.
     * The shift and mask are passed rather than pasted because JS has no token concatenation; every
     * call site below names the same two constants the C's macro would have expanded to.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {number} w
     * @param {number} shift
     * @param {number} mask
     * @returns {number}
     */
    getp(pkt, w, shift, mask) { return (this.pd(pkt, w) >>> shift) & mask; }

    /**
     * putp32(pkt, w, v)
     *
     * The C's PUTP32(p,w,v) macro: `d[w] = v & 0xFFFF; d[w+1] = (v >> 16) & 0xFFFF` -- LOW WORD
     * FIRST, which is the only ordering question a 32-bit MSCP field has and the one that
     * `media identifier byte order swapped` is a mutation for.  `>>> 16` rather than `>> 16` because
     * every 32-bit value this is called with (a MediaId, an LBN count) is unsigned and MediaIds have
     * bit 31 clear only by luck of the table.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {number} w
     * @param {number} v
     */
    putp32(pkt, w, v)
    {
        this.spd(pkt, w, v & 0xFFFF);
        this.spd(pkt, w + 1, (v >>> 16) & 0xFFFF);
    }

    /* --------------------------------------------------------------------------------------- *
     * MSCP command dispatch                                                                     *
     * --------------------------------------------------------------------------------------- */

    /**
     * mscp(pkt, q)
     *
     * rq_mscp() (pdp11_rq.c:1926-1981).  The switch is expressed as membership in the three name
     * sets derived from the OP table at the top of this file, so the dispatch and the scope are the
     * same object -- HANDOFF.md standing rule 7, earned by two modules whose headers disagreed
     * about who owned six opcodes.
     *
     * `q` is the C's `t_bool q`, "may this command be deferred onto a unit queue".  It is threaded
     * through to keep the signature honest even though every handler that USES it is excluded here.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean} the C's OK/ERR -- false ends the queue thread
     */
    mscp(pkt, q)
    {
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let name = RQVAX.OP_NAME_OF[cmd];
        let handler = (name === undefined) ? undefined : RQVAX.MSCP_OP_HANDLER[name];
        if (handler !== undefined && handler !== MSCP_NOP_ARM) {
            let method = RQVAX.MSCP_METHOD[handler];
            if (method === undefined) {
                throw new RQUnimplemented("rq.js: MSCP command OP_" + name + " (" + cmd + ") is " +
                    "dispatched by rq_mscp() to " + handler + "(), which is out of scope -- " +
                    (RQVAX.MSCP_EXCLUDED[handler] || "no reason recorded, which is itself a defect"));
            }
            return this[method](pkt, q);
        }
        let sts;
        if (handler === MSCP_NOP_ARM) {
            cmd = cmd | OP.END;                             /* set end flag */
            sts = ST.SUC;                                   /* success */
        } else {
            cmd = OP.END;                                   /* set end op */
            sts = ST.CMD | I_OPCD;                          /* ill op */
        }
        this.putr(pkt, cmd, 0, sts, RSP_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /* --------------------------------------------------------------------------------------- *
     * The unit-bearing commands that need no data transfer (pcjsvax-f52)                        *
     *                                                                                           *
     * All seven have the same skeleton and it is worth stating once: look the unit up BY PLUG,    *
     * answer ST_OFL (bare, no subcode) if there is no such unit at all, and otherwise answer      *
     * something derived from whether it is ATTACHED and whether it is ONLINE.  *** ST_OFL and     *
     * ST_OFL|SB_OFL_NV ARE DIFFERENT ANSWERS TO DIFFERENT QUESTIONS ***: the bare one means "no    *
     * such unit", the subcoded one means "the unit exists and has no volume".  A model that        *
     * conflated them would satisfy any test that only looked at the major status.                 *
     * --------------------------------------------------------------------------------------- */

    /**
     * putrUnit(pkt, u, lu, all)
     *
     * rq_putr_unit() (pdp11_rq.c:2950-2977).  Fills the unit-description words that ONLINE, SET UNIT
     * CHARACTERISTICS, GET UNIT STATUS and the unit-available attention packet all share -- the
     * ONL_ and GUS_ names are THE SAME WORD INDICES for the fields it writes, which is why one
     * function serves both.
     *
     * `all` is the C's `t_bool all`, the LONG FORM: it adds the unit's SIZE IN BLOCKS and its VOLUME
     * SERIAL NUMBER.  rq_gus() passes FALSE and rq_onl()/rq_suc() pass TRUE, so the short form's
     * words are whatever the HOST left in its command packet -- which is observable, and is why the
     * differential grades a GET UNIT STATUS whose ONL_SIZL/ONL_VSNL words carry the host's seed
     * straight back out.
     *
     * *** ONL_VSNL IS `01234 + lu` AND 01234 IS OCTAL: 668 DECIMAL. ***  See ONL_VSN_BASE.
     *
     * `maxlbn` is `(uint32) uptr->capac` -- the unit's capacity IN BLOCKS, which autosize computed
     * from the container.  Reporting it in BYTES is a mutation the differential carries, because
     * every geometry field around it would still be right.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {Object} u
     * @param {number} lu
     * @param {boolean} all
     */
    putrUnit(pkt, u, lu, all)
    {
        let d = RQVAX.DRV_TAB[u.dtype];
        let maxlbn = u.capac >>> 0;                         /* (uint32) uptr->capac */
        this.spd(pkt, ONL_MLUN, lu);                        /* unit */
        this.spd(pkt, ONL_UFL, u.uf | UF_RPL | this.wph(u) | this.rmv(u));
        this.spd(pkt, ONL_RSVL, 0);                         /* reserved */
        this.spd(pkt, ONL_RSVH, 0);
        this.spd(pkt, ONL_UIDA, lu);                        /* UID low */
        this.spd(pkt, ONL_UIDB, 0);
        this.spd(pkt, ONL_UIDC, 0);
        this.spd(pkt, ONL_UIDD, (UID_DISK << ONL_UIDD_V_CLS) |
                                (d.mod << ONL_UIDD_V_MOD)); /* UID hi */
        this.putp32(pkt, ONL_MEDL, d.MediaId >>> 0);        /* media type */
        if (all) {                                          /* if long form */
            this.putp32(pkt, ONL_SIZL, maxlbn);             /* user LBNs */
            this.spd(pkt, ONL_VSNL, ONL_VSN_BASE + lu);     /* vol serial # */
            this.spd(pkt, ONL_VSNH, 0);
        }
    }

    /**
     * setfUnit(pkt, u)
     *
     * rq_setf_unit() (pdp11_rq.c:2939-2947), the C's own "hack unit flags".  The host may set only
     * UF_CMR and UF_CMW (UF_MSK) -- everything else it asks for is DISCARDED -- and it may set
     * SOFTWARE WRITE PROTECT only if it ALSO set MD_SWP in the command's modifier.  Two conditions,
     * and dropping either makes a host able to write-protect a unit it should not, or unable to
     * write-protect one it should.
     *
     * It runs BEFORE putrUnit() at both call sites, so the flags it installs are the ones that go
     * back out in the SAME response.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {Object} u
     */
    setfUnit(pkt, u)
    {
        u.uf = this.pd(pkt, ONL_UFL) & UF_MSK;              /* settable flags */
        if ((this.pd(pkt, CMD_MOD) & MD_SWP) &&             /* swre wrp enb? */
            (this.pd(pkt, ONL_UFL) & UF_WPS)) {             /* swre wrp on? */
            u.uf = u.uf | UF_WPS;                           /* simon says... */
        }
    }

    /**
     * deferIfBusy(pkt, q, u)
     *
     * `if (q && uptr->cpkt) { rq_enqt (cp, &uptr->pktq, pkt); return OK; }` -- the head of rq_avl(),
     * rq_onl(), rq_suc() and rq_fmt().  ONLY rq_rw() ever sets `cpkt`, so nothing in this file's
     * scope can reach it; it is transcribed rather than omitted because it is two lines and because
     * a gap here is one pcjsvax-346 would have to rediscover from a diverging response rather than
     * from a named exclusion.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @param {Object} u
     * @returns {boolean} true if the packet was deferred and the caller must return OK
     */
    deferIfBusy(pkt, q, u)
    {
        if (q && u.cpkt) {                                  /* need to queue? */
            this.enqt(u, "pktq", pkt);                      /* do later */
            return true;
        }
        return false;
    }

    /**
     * rbpar(d)
     *
     * `if (drv_tab[dtyp].rcts) rbpar = 1; else rbpar = 0;` -- the flag rq_gus() reports twice, in
     * GUS_RBSC's RBNs-per-track and RCT-copies fields.  It is *** rcts, THE RCT SIZE, AND NOT rctc,
     * THE NUMBER OF RCT COPIES ***, and the two disagree for exactly seven drive types (RC25, RCF25,
     * RA80 and the four ESE solid-state disks all have rcts == 0 and rctc == 1), which is why the
     * differential's drive-table sweep has to cover the whole table rather than a sample.
     *
     * A METHOD rather than an inline expression for HANDOFF.md standing rule 11: --selfcheck must be
     * able to perturb this one decision without substituting a copy of rq_gus(), and a mutation
     * applied to rq_gus()'s RESULT would be too late -- the response has already been DMAd to the
     * host by the time rq_gus() returns.
     *
     * @this {RQVAX}
     * @param {Object} d a DRV_TAB row
     * @returns {number} 0 or 1
     */
    rbpar(d) { return d.rcts ? 1 : 0; }

    /**
     * gus(pkt, q)
     *
     * rq_gus() (pdp11_rq.c:2085-2124).  THE SHORT FORM of rq_putr_unit() plus the disk geometry.
     *
     * MD_NXU, "next unit", is the walk a host uses to enumerate what the controller has: it sends
     * GET UNIT STATUS with an increasing unit number, and when the number passes `max_plug` the
     * CONTROLLER RESETS IT TO ZERO AND SAYS SO by rewriting RSP_UN -- which is CMD_UN, the same
     * word.  Note it is `lu > max_plug`, so the unit EQUAL to max_plug does not wrap; and note the
     * reset happens BEFORE the lookup, so the answer is unit 0's, not "no such unit".  A host that
     * walks off the end therefore gets unit 0 back rather than ST_OFL, which is the behaviour the
     * differential's `walks off the end` case exists to pin.
     *
     * *** THE THREE GEOMETRY WORDS ARE NOT WHAT THEIR NAMES SAY. ***  `GUS_TRK = sect`,
     * `GUS_GRP = tpg`, `GUS_CYL = gpc` -- so the word called CYLINDER carries GROUPS PER CYLINDER
     * (1 for every drive but RX50, which has 16) and not the cylinder count.  That is the C,
     * verbatim; a "corrected" version would report drv_tab[].cyl and differ on every drive.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    gus(pkt, q)
    {
        let lu = this.pd(pkt, CMD_UN);                      /* unit # */
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let sts, u;
        if (this.pd(pkt, CMD_MOD) & MD_NXU) {               /* next unit? */
            if (lu > this.maxPlug) {                        /* beyond last unit plug? */
                lu = 0;                                     /* reset to 0 */
                this.spd(pkt, RSP_UN, lu);
            }
        }
        if ((u = this.getucb(lu))) {                        /* unit exist? */
            if (!(u.flags & U_ATT)) sts = ST.OFL | SB_OFL_NV;   /* not attached -> offl no vol */
            else if (u.flags & U_ONL) sts = ST.SUC;         /* online */
            else sts = ST.AVL;                              /* avail */
            this.putrUnit(pkt, u, lu, false);               /* fill unit fields */
            let d = RQVAX.DRV_TAB[u.dtype];                 /* get drive type */
            let rbpar = this.rbpar(d);                      /* ctrl bad blk? */
            this.spd(pkt, GUS_TRK, d.sect);
            this.spd(pkt, GUS_GRP, d.tpg);
            this.spd(pkt, GUS_CYL, d.gpc);
            this.spd(pkt, GUS_UVER, 0);
            this.spd(pkt, GUS_RCTS, d.rcts);
            this.spd(pkt, GUS_RBSC, (rbpar << GUS_RB_V_RBNS) | (rbpar << GUS_RB_V_RCTC));
        } else sts = ST.OFL;                                /* offline */
        this.spd(pkt, GUS_SHUN, lu);                        /* shadowing */
        this.spd(pkt, GUS_SHST, 0);
        this.putr(pkt, cmd | OP.END, 0, sts, GUS_LNT_D, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * onl(pkt, q)
     *
     * rq_onl() (pdp11_rq.c:2128-2163).  FOUR distinct answers, and the differential grades all four:
     *   no such unit                    ST_OFL           (bare)
     *   the unit exists, nothing attached  ST_OFL | SB_OFL_NV
     *   the unit is ALREADY online      ST_SUC | SB_SUC_ON   -- and it stays online, and NO flags
     *                                                            are hacked: rq_setf_unit() is NOT
     *                                                            called on this path, so a host
     *                                                            cannot software-write-protect a
     *                                                            unit by onlining it twice
     *   otherwise                       ST_SUC, and the unit goes online
     * The LONG form of rq_putr_unit() is used, so this is where a host learns the unit's SIZE -- and
     * the size is the one thing in the whole response that depends on the user's file rather than on
     * the drive type.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    onl(pkt, q)
    {
        let lu = this.pd(pkt, CMD_UN);                      /* unit # */
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let sts, u;
        if ((u = this.getucb(lu))) {                        /* unit exist? */
            if (this.deferIfBusy(pkt, q, u)) return true;   /* need to queue? */
            if (!(u.flags & U_ATT)) sts = ST.OFL | SB_OFL_NV;   /* not attached? offl no vol */
            else if (u.flags & U_ONL) sts = ST.SUC | SB_SUC_ON; /* already online? */
            else if (this.isAvailable(u)) {                 /* mark online */
                sts = ST.SUC;
                u.flags = u.flags | U_ONL;
                this.setfUnit(pkt, u);                      /* hack flags */
            } else sts = ST.OFL | SB_OFL_NV;                /* offl no vol */
            this.putrUnit(pkt, u, lu, true);                /* set fields */
        } else sts = ST.OFL;                                /* offline */
        this.spd(pkt, ONL_SHUN, lu);                        /* shadowing */
        this.spd(pkt, ONL_SHST, 0);
        this.putr(pkt, cmd | OP.END, 0, sts, ONL_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * suc(pkt, q)
     *
     * rq_suc() (pdp11_rq.c:2200-2227).  SET UNIT CHARACTERISTICS is ONLINE without the online: it
     * hacks the flags and returns the same long-form unit description, but it NEVER SETS UNIT_ONL
     * and it succeeds on an AVAILABLE unit as readily as on an online one -- so a host can
     * software-write-protect a unit it has not brought online, and the ONL_UFL word it gets back
     * says so.  Its ONE difference from ONLINE in the response is the length: SUC_LNT and ONL_LNT
     * are both 44, so the two responses are the same SIZE and differ only in the opcode word --
     * which is exactly why a differential must compare the opcode and not just the payload.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    suc(pkt, q)
    {
        let lu = this.pd(pkt, CMD_UN);                      /* unit # */
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let sts, u;
        if ((u = this.getucb(lu))) {                        /* unit exist? */
            if (this.deferIfBusy(pkt, q, u)) return true;   /* need to queue? */
            if (!(u.flags & U_ATT)) sts = ST.OFL | SB_OFL_NV;   /* not attached? offl no vol */
            else {                                          /* avail or onl */
                sts = ST.SUC;
                this.setfUnit(pkt, u);                      /* hack flags */
            }
            this.putrUnit(pkt, u, lu, true);                /* set fields */
        } else sts = ST.OFL;                                /* offline */
        this.spd(pkt, ONL_SHUN, lu);                        /* shadowing */
        this.spd(pkt, ONL_SHST, 0);
        this.putr(pkt, cmd | OP.END, 0, sts, SUC_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * avl(pkt, q)
     *
     * rq_avl() (pdp11_rq.c:2029-2053).  AVAILABLE is the inverse of ONLINE and it is DESTRUCTIVE in
     * a way its name hides: it clears UNIT_ONL *and* zeroes `uf`, so every software write protect
     * and compare flag the host installed with SET UNIT CHARACTERISTICS is gone.  It ALSO answers
     * ST_SUC for a unit with nothing attached -- there is no ATT test here, unlike every other
     * handler -- which is the one place `the unit exists` alone is enough.
     *
     * `if ((mdf & MD_SPD) && RQ_RMV (uptr)) sim_disk_unload (uptr)` EJECTS THE MEDIA on a removable
     * drive.  Excluded by name; see the file header.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    avl(pkt, q)
    {
        let lu = this.pd(pkt, CMD_UN);                      /* unit # */
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let mdf = this.pd(pkt, CMD_MOD);                    /* modifier */
        let sts, u;
        if ((u = this.getucb(lu))) {                        /* unit exist? */
            if (this.deferIfBusy(pkt, q, u)) return true;   /* need to queue? */
            u.flags = u.flags & ~U_ONL;                     /* not online */
            if ((mdf & MD_SPD) && this.rmv(u)) {            /* unload of removable device */
                throw new RQUnimplemented("rq.js: AVAILABLE with MD_SPD on a REMOVABLE unit calls " +
                    "sim_disk_unload(), which ejects the user's container and makes " +
                    "sim_disk_isavailable() false for every later command -- media removal is not " +
                    "pcjsvax-f52's outcome and is excluded by name");
            }
            u.uf = 0;                                       /* clr flags */
            sts = ST.SUC;                                   /* success */
        } else sts = ST.OFL;                                /* offline */
        this.putr(pkt, cmd | OP.END, 0, sts, AVL_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * fmt(pkt, q)
     *
     * rq_fmt() (pdp11_rq.c:2231-2264).  FORMAT is the RX33's alone, and its five refusals are a
     * strict ORDER that is itself the behaviour:
     *   not an RX33                     ST_CMD | I_OPCD   -- checked FIRST, before the magic bit,
     *                                                        so an RD54 never sees I_FMTI
     *   the magic bit (0100000 in FMT_IH) clear    ST_CMD | I_FMTI
     *   nothing attached                ST_OFL | SB_OFL_NV
     *   the unit is ONLINE              ST_AVL | SB_AVL_INU -- and it is taken OFFLINE and its
     *                                                        flags cleared as a SIDE EFFECT of the
     *                                                        refusal, which is the surprising part
     *   write protected                 ST_WPR | SB_WPR_HW
     *   otherwise                       ST_SUC  -- and NOTHING IS FORMATTED.  The C's own comment is
     *                                             `/*** for now ***\/`.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    fmt(pkt, q)
    {
        let lu = this.pd(pkt, CMD_UN);                      /* unit # */
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let sts, u;
        if ((u = this.getucb(lu))) {                        /* unit exist? */
            if (this.deferIfBusy(pkt, q, u)) return true;   /* need to queue? */
            if (u.dtype !== RQVAX.RX33_DTYPE) sts = ST.CMD | I_OPCD;    /* RX33? no, err */
            else if ((this.pd(pkt, FMT_IH) & FMT_IH_MAGIC) === 0) sts = ST.CMD | I_FMTI;
            else if (!(u.flags & U_ATT)) sts = ST.OFL | SB_OFL_NV;      /* offline? no vol */
            else if (u.flags & U_ONL) {                     /* online? */
                u.flags = u.flags & ~U_ONL;
                u.uf = 0;                                   /* clear flags */
                sts = ST.AVL | SB_AVL_INU;                  /* avail, in use */
            } else if (this.wph(u)) sts = ST.WPR | SB_WPR_HW;   /* write prot? can't fmt */
            else sts = ST.SUC;                              /*** for now ***/
        } else sts = ST.OFL;                                /* offline */
        this.putr(pkt, cmd | OP.END, 0, sts, FMT_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * abo(pkt, q)
     *
     * rq_abo() (pdp11_rq.c:1984-2025).  Everything ABORT does beyond answering ST_SUC is a search of
     * the unit's IN-FLIGHT packet (`cpkt`) and its DEFERRED queue (`pktq`) for a matching reference
     * number, and only rq_rw() ever puts a packet in either -- so with an idle unit the C's `tpkt`
     * stays 0, the `if (tpkt)` arm does not run, and the answer is ST_SUC with ABO_LNT whether or
     * not the unit even exists.  *** ABORT NEVER REPORTS ST_OFL. ***  That is the whole of its
     * reachable behaviour here and it is graded; the search arms throw by name if a unit is ever
     * found holding a packet, which is a state pcjsvax-346 creates and this item cannot.
     *
     * `ref = GETP32 (pkt, ABO_REFL)` is deliberately NOT computed: it is read only by the search
     * arms, and computing a value nothing uses would be a comment claiming coverage the code does
     * not have (HANDOFF.md standing rule 12).
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    abo(pkt, q)
    {
        let lu = this.pd(pkt, CMD_UN);                      /* unit # */
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let u = this.getucb(lu);                            /* get unit */
        if (u && (u.cpkt || u.pktq)) {
            throw new RQUnimplemented("rq.js: ABORT against a unit holding a packet searches " +
                "uptr->cpkt and uptr->pktq and may cancel an in-flight transfer -- only rq_rw() " +
                "puts a packet there, and that is pcjsvax-346's work");
        }
        this.putr(pkt, cmd | OP.END, 0, ST.SUC, ABO_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * gcs(pkt, q)
     *
     * rq_gcs() (pdp11_rq.c:2057-2081).  GET COMMAND STATUS reports the WORKING BYTE COUNT of the
     * unit's in-flight transfer, and its whole four-condition test -- unit exists AND has a current
     * packet AND that packet's reference matches AND its opcode is at least OP_ACC -- collapses to
     * FALSE for an idle unit.  So the reachable answer is the else arm: BOTH status words ZEROED,
     * ST_SUC, GCS_LNT, and again NO ST_OFL even for a unit that does not exist.  The zeroing is the
     * observable part: the host's own command words at GCS_STSL/GCS_STSH are overwritten, so a
     * controller that skipped the else arm would return the host's data and look plausible.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    gcs(pkt, q)
    {
        let lu = this.pd(pkt, CMD_UN);                      /* unit # */
        let cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
        let u = this.getucb(lu);                            /* valid lu? */
        if (u && u.cpkt) {
            throw new RQUnimplemented("rq.js: GET COMMAND STATUS against a unit with a current " +
                "packet reports that transfer's working byte count -- transfers are pcjsvax-346's");
        }
        this.spd(pkt, GCS_STSL, 0);                         /* return 0 */
        this.spd(pkt, GCS_STSH, 0);
        this.putr(pkt, cmd | OP.END, 0, ST.SUC, GCS_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * scc(pkt, q)
     *
     * rq_scc() (pdp11_rq.c:2166-2198).  Two things here are the C and look like defects:
     *   - the failure arm sets `cmd = 0`, so the response's opcode word is `0 | OP_END` == 0x0080
     *     rather than `OP_SCC | OP_END`.  A host cannot tell WHICH command failed from the opcode.
     *   - `cflgs` takes the host's whole 16-bit word ORed under CF_RPL, with NO CF_MSK filter, so
     *     bits the header says are unassigned survive into the controller and back out again.  The
     *     C's own comment calls it a "hack ctrl flgs".
     * `htmo` is rounded UP BY TWO when non-zero, and a zero timeout is stored as zero rather than
     * being rejected -- which is what makes `HAT` observable at 0 rather than at the default 60.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} q
     * @returns {boolean}
     */
    scc(pkt, q)
    {
        let sts, cmd;
        if (this.pd(pkt, SCC_MSV)) {                        /* MSCP ver = 0? */
            sts = ST.CMD | I_VRSN;                          /* no, lose */
            cmd = 0;
        } else {
            sts = ST.SUC;                                   /* success */
            cmd = this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC);
            this.cflgs = (this.cflgs & CF_RPL) | this.pd(pkt, SCC_CFL);
            this.htmo = this.pd(pkt, SCC_TMO);              /* set timeout */
            if (this.htmo) this.htmo = this.htmo + 2;       /* if nz, round up */
            this.spd(pkt, SCC_CFL, this.cflgs);             /* return flags */
            this.spd(pkt, SCC_TMO, RQ_DCTMO);               /* ctrl timeout */
            this.spd(pkt, SCC_VER, (RQVAX.RQ_HVER << SCC_VER_V_HVER) |
                                   (RQVAX.RQ_SVER << SCC_VER_V_SVER));
            this.spd(pkt, SCC_CIDA, 0);                     /* ctrl ID */
            this.spd(pkt, SCC_CIDB, 0);
            this.spd(pkt, SCC_CIDC, 0);
            this.spd(pkt, SCC_CIDD, (RQVAX.RQ_CLASS << SCC_CIDD_V_CLS) |
                                    (RQVAX.CTLR_TAB[this.ctype].model << SCC_CIDD_V_MOD));
            this.spd(pkt, SCC_MBCL, 0);                     /* max bc */
            this.spd(pkt, SCC_MBCH, 0);
        }
        this.putr(pkt, cmd | OP.END, 0, sts, SCC_LNT, UQ_TYP_SEQ);
        return this.putPkt(pkt, true);
    }

    /**
     * putr(pkt, cmd, flg, sts, lnt, typ)
     *
     * rq_putr() (pdp11_rq.c:2967-2977) -- overwrite the four header/response words IN PLACE, over
     * the command the host sent.  Everything it does NOT touch (the reference number at CMD_REFL/H,
     * the unit at CMD_UN, and every word past the ones a handler wrote) is the host's own data
     * travelling back out, which is why the response the host reads back is evidence that the whole
     * 64-byte packet round-tripped rather than just the fields a test looks at.
     *
     * Note UQ_HCTC is written with the credit field CLEAR; rq_putpkt() ORs the credits in later.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {number} cmd
     * @param {number} flg
     * @param {number} sts
     * @param {number} lnt
     * @param {number} typ
     */
    putr(pkt, cmd, flg, sts, lnt, typ)
    {
        this.spd(pkt, RSP_OPF, (cmd << RSP_OPF_V_OPC) | (flg << RSP_OPF_V_FLG));
        this.spd(pkt, RSP_STS, sts);
        this.spd(pkt, UQ_HLNT, lnt);                        /* length */
        this.spd(pkt, UQ_HCTC, (typ << UQ_HCTC_V_TYP) | (UQ_CID_MSCP << UQ_HCTC_V_CID));
    }

    /* --------------------------------------------------------------------------------------- *
     * Packet and descriptor handling                                                            *
     * --------------------------------------------------------------------------------------- */

    /**
     * getPkt()
     *
     * rq_getpkt() (pdp11_rq.c:2818-2836).  The C returns OK/ERR and writes the packet through an
     * out-parameter; this returns both, because a JS function cannot.  Note the ORDER, all of which
     * is observable: the descriptor is fetched FIRST (so an empty ring costs no packet), the free
     * packet is taken SECOND (so a full pool goes PE_NSR only when there was work to do), `hat` is
     * disabled THIRD, and the descriptor is released LAST -- after the read, so a packet-read
     * failure leaves the descriptor still owned by the controller.
     *
     * THE PACKET IS AT `desc & UQ_ADDR` PLUS UQ_HDR_OFF, i.e. FOUR BYTES BELOW the address the
     * descriptor names, and RQ_PKT_SIZE (64) bytes are read WHATEVER the command is -- the length
     * word in the packet is not consulted.  Both are mutations mscpringdiff.js carries, because
     * both are invisible to any test that only checks the fields a command defines.
     *
     * @this {RQVAX}
     * @returns {{ok: boolean, pkt: number}}
     */
    getPkt()
    {
        let desc = this.getDesc(this.cq);                   /* get cmd desc */
        if (desc === null) return {ok: false, pkt: 0};
        if ((desc & UQ_DESC_OWN) === 0) return {ok: true, pkt: 0};      /* none */
        let pkt = this.deqf();                              /* get cmd pkt */
        if (pkt === null) return {ok: false, pkt: 0};
        /* `cp->hat = 0` DISABLES THE HOST-ACCESS TIMER while a command is in flight, and it is
           STATE THIS ITEM CANNOT OBSERVE: rq_putpkt() sets `hat = htmo` again as soon as `pbsy`
           returns to zero, rq_reset() sets it on every fatal, and the only window in which it is 0
           is inside a single rq_quesvc() call -- which a host can only look at by halting, and a
           halt drains the event queue.  It is reproduced because it is the C; it is named here and
           in tests/mscpringdiff.js's exclusion list because nothing in pcjsvax-0b4's scope can
           falsify it.  It becomes observable when rq_tmrsvc()'s wall-clock timer is modelled. */
        this.hat = 0;                                       /* dsbl hst timer */
        let addr = this.descAddr(desc);                     /* get Q22 addr */
        if (this.cqbic.mapReadW(this.hdrAddr(addr), RQ_PKT_SIZE, this.pktBuf)) {
            this.fatal(PE_PRE);                             /* read pkt */
            return {ok: false, pkt};
        }
        let base = pkt * RQ_PKT_SIZE_W;
        for (let i = 0; i < RQ_PKT_SIZE_W; i++) {
            this.pakData[base + i] = this.pktBuf[i * 2] | (this.pktBuf[i * 2 + 1] << 8);
        }
        return {ok: this.putDesc(this.cq, desc), pkt};      /* release desc */
    }

    /**
     * hdrAddr(addr)
     *
     * `addr + UQ_HDR_OFF` -- the packet's header address, four bytes BELOW the descriptor's own.
     * Split out as a method so mscpringdiff.js's --selfcheck can perturb the offset without
     * substituting a copy of rq_getpkt()/rq_putpkt() (HANDOFF.md standing rule 11); both call it and
     * neither has another source for the address.  The C computes it in uint32, so an `addr` below
     * 4 borrows to the top of the address space rather than going negative; `>>> 0` reproduces that.
     *
     * @this {RQVAX}
     * @param {number} addr
     * @returns {number}
     */
    hdrAddr(addr) { return (addr + UQ_HDR_OFF) >>> 0; }

    /**
     * descAddr(desc)
     *
     * `desc & UQ_ADDR` -- the Q22 buffer address a descriptor names.  UQ_ADDR is 0x003FFFFE, so it
     * strips FOUR things and not two: the ownership bit, the flag bit, EVERY bit above 21 (the Qbus
     * is a 22-bit address space), and the ODD-BYTE BIT 0 (a UQSSP descriptor addresses words).  A
     * host that sets bit 0 or bit 22 is not making an error the controller reports -- the bits are
     * simply discarded -- which is why tests/mscpringdiff.js posts a descriptor with both set.
     * Published for the same reason as hdrAddr(), and used by BOTH rq_getpkt() and rq_putpkt() so
     * that a mutation of it perturbs the read and the write together.
     *
     * IT IS ALSO, MEASURABLY, REDUNDANT.  cqbic.js's mapReadW()/mapWriteW() open with
     * `ba & QBMAMASK & ~1`, which is vax_io.c:774/807 exactly, so every bit UQ_ADDR removes is
     * removed again before the address reaches memory -- and no host program can distinguish a
     * controller that applies this mask from one that does not.  tests/mscpringdiff.js records that
     * finding where it belongs: it keeps the case that posts a descriptor with those bits set (the
     * controller accepts it and RELEASES IT WITH THEM STILL SET, because rq_putdesc() rewrites the
     * whole word without re-masking) and it carries NO mutation for the mask, because two were
     * written and both were measured to survive.  The mask stays because it is the C.
     *
     * @this {RQVAX}
     * @param {number} desc
     * @returns {number}
     */
    descAddr(desc) { return (desc & UQ_ADDR) >>> 0; }

    /**
     * putPkt(pkt, qt)
     *
     * rq_putpkt() (pdp11_rq.c:2843-2877), including the C's own "clever hack about credits": the
     * controller hands the host ALL of its credits on the FIRST end packet (up to 14, plus the
     * implicit one that every packet carries) and exactly one per response thereafter.  So the
     * first response's UQ_HCTC credit field is 15 and CRED drops 15 -> 1; the second is 2 and CRED
     * drops to 0; every one after that is 1.  A model that never decrements CRED produces 15 every
     * time and is caught by the second response, never the first.
     *
     * `qt` decides which END of the response queue a packet goes on when the ring is FULL: TRUE
     * (from rq_mscp) appends, FALSE (from the queue thread re-trying a deferred response) pushes
     * back onto the head so ORDER is preserved across the retry.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @param {boolean} qt
     * @returns {boolean}
     */
    putPkt(pkt, qt)
    {
        if (pkt === 0) return true;                         /* any packet? */
        this.traceReq("rsp=" + h4(this.pd(pkt, RSP_OPF)) + ", sts=" + h4(this.pd(pkt, RSP_STS)));
        let desc = this.getDesc(this.rq);                   /* get rsp desc */
        if (desc === null) return false;
        if ((desc & UQ_DESC_OWN) === 0) {                   /* not valid? */
            if (qt) this.enqt(this, "rspq", pkt);           /* normal? q tail */
            else this.enqh(this, "rspq", pkt);              /* resp q call */
            this.activateQueue(this.qtime);                 /* activate q thrd */
            return true;
        }
        let addr = this.descAddr(desc);                     /* get Q22 addr */
        let lnt = this.responseLength(pkt);                 /* size, with hdr */
        if ((this.getp(pkt, UQ_HCTC, UQ_HCTC_V_TYP, UQ_HCTC_M_TYP) === UQ_TYP_SEQ) &&
            (this.getp(pkt, CMD_OPC, CMD_OPC_V_OPC, CMD_OPC_M_OPC) & OP.END)) {
            let cr = (this.credits >= 14) ? 14 : this.credits;       /* max 14 credits */
            this.credits = this.credits - cr;               /* decr credits */
            this.spd(pkt, UQ_HCTC, this.pd(pkt, UQ_HCTC) | ((cr + 1) << UQ_HCTC_V_CR));
        }
        let base = pkt * RQ_PKT_SIZE_W;
        for (let i = 0; i < RQ_PKT_SIZE_W; i++) {
            this.pktBuf[i * 2] = this.pakData[base + i] & 0xFF;
            this.pktBuf[i * 2 + 1] = (this.pakData[base + i] >>> 8) & 0xFF;
        }
        if (this.cqbic.mapWriteW(this.hdrAddr(addr), lnt, this.pktBuf)) {
            this.fatal(PE_PWE);                             /* write pkt */
            return false;
        }
        this.enqh(this, "freq", pkt);                       /* pkt is free */
        this.pbsy = this.pbsy - 1;                          /* decr busy cnt */
        if (this.pbsy === 0) this.hat = this.htmo;          /* idle? strt hst tmr */
        return this.putDesc(this.rq, desc);                 /* release desc */
    }

    /**
     * responseLength(pkt)
     *
     * `cp->pak[pkt].d[UQ_HLNT] - UQ_HDR_OFF` -- the MSCP message length the handler wrote, PLUS the
     * four header bytes, because the DMA starts four bytes below the descriptor's address.  Taking
     * it as `d[UQ_HLNT]` alone writes four bytes too few and leaves the last longword of the
     * response as whatever the host left there, which is a mutation mscpringdiff.js carries and
     * which no comparison of the fields a command defines can see.
     *
     * @this {RQVAX}
     * @param {number} pkt
     * @returns {number}
     */
    responseLength(pkt) { return (this.pd(pkt, UQ_HLNT) - UQ_HDR_OFF) >>> 0; }

    /**
     * getDesc(ring)
     *
     * rq_getdesc() (pdp11_rq.c:2879-2890) -- a 4-byte DMA READ through the same map, and the ONLY
     * memory reference the CST_UP poll makes when the ring is empty.  Its failure path is
     * rq_fatal(PE_QRE), which is a genuinely reachable state (poll with the map unprogrammed) and is
     * a graded case.
     *
     * @this {RQVAX}
     * @param {Object} ring
     * @returns {?number} the descriptor, or null if the controller went fatal
     */
    getDesc(ring)
    {
        let addr = (ring.ba + ring.idx) >>> 0;
        if (this.cqbic.mapReadW(addr, 4, this.descBuf)) {   /* fetch desc */
            this.fatal(PE_QRE);                             /* err? dead */
            return null;
        }
        return ((this.descBuf[0] | (this.descBuf[1] << 8) |
                 (this.descBuf[2] << 16) | (this.descBuf[3] << 24)) >>> 0);
    }

    /**
     * putDesc(ring, desc)
     *
     * rq_putdesc() (pdp11_rq.c:2897-2921) -- hand a descriptor back to the host and decide whether
     * to poke the ring's interrupt word.  Four separable things happen and each is graded:
     *
     *   1. The value written back is `(desc & ~UQ_DESC_OWN) | UQ_DESC_F`.  OWN is CLEARED (the host
     *      owns the slot again) and F is SET WHETHER OR NOT the host had set it -- so a host that
     *      posts a descriptor without F still gets one back with F, and still gets NO interrupt.
     *   2. The interrupt test reads the descriptor the HOST posted, not the one just written.
     *   3. `ring->lnt <= 4` -- a ONE-DESCRIPTOR ring -- interrupts unconditionally, because there is
     *      no "previous" slot to look at.  Any larger ring reads the previous slot and interrupts
     *      only if the HOST still owns it, i.e. only on an empty-to-non-empty / full-to-not-full
     *      transition.  Both arms are graded, both ways.
     *   4. The index advances LAST, and only on success -- a fatal leaves it where it was.
     *
     * `(ring->idx - 4) & (ring->lnt - 1)` is computed in uint32 in the C, so at idx 0 it borrows to
     * 0xFFFFFFFC and the mask picks the LAST slot.  JS's `&` runs its operands through ToInt32, so
     * `(0 - 4) & 15` is 12 here exactly as it is there -- and `idx - 4` WITHOUT the mask is a
     * different address at wrap, which is a mutation mscpringdiff.js carries.
     *
     * @this {RQVAX}
     * @param {Object} ring
     * @param {number} desc the descriptor as the HOST posted it
     * @returns {boolean}
     */
    putDesc(ring, desc)
    {
        let newd = this.releasedDesc(desc);
        let addr = (ring.ba + ring.idx) >>> 0;
        this.descBuf[0] = newd & 0xFF;                      /* 32b to 16b, LE */
        this.descBuf[1] = (newd >>> 8) & 0xFF;
        this.descBuf[2] = (newd >>> 16) & 0xFF;
        this.descBuf[3] = (newd >>> 24) & 0xFF;
        if (this.cqbic.mapWriteW(addr, 4, this.descBuf)) {  /* store desc */
            this.fatal(PE_QWE);                             /* err? dead */
            return false;
        }
        if (desc & UQ_DESC_F) {                             /* was F set? */
            if (ring.lnt <= 4) this.ringInt(ring);          /* lnt = 1? intr */
            else {                                          /* prv desc */
                let prva = (ring.ba + this.prevSlot(ring)) >>> 0;
                if (this.cqbic.mapReadW(prva, 4, this.descBuf)) {       /* read prv */
                    this.fatal(PE_QRE);
                    return false;
                }
                let prvd = ((this.descBuf[0] | (this.descBuf[1] << 8) |
                             (this.descBuf[2] << 16) | (this.descBuf[3] << 24)) >>> 0);
                if (prvd & UQ_DESC_OWN) this.ringInt(ring);
            }
        }
        ring.idx = this.nextSlot(ring);
        return true;
    }

    /**
     * releasedDesc(desc) / prevSlot(ring) / nextSlot(ring)
     *
     * The three arithmetic decisions inside rq_putdesc(), published as methods so --selfcheck can
     * PERTURB them without substituting a copy of rq_putdesc() itself (HANDOFF.md standing rule 11).
     * Each is on the shipped path and rq_putdesc() above has no other source for its value.
     *
     * @this {RQVAX}
     * @param {number} desc
     * @returns {number}
     */
    releasedDesc(desc) { return ((desc & ~UQ_DESC_OWN) | UQ_DESC_F) >>> 0; }

    prevSlot(ring) { return (ring.idx - 4) & (ring.lnt - 1); }

    nextSlot(ring) { return (ring.idx + 4) & (ring.lnt - 1); }

    /**
     * ringInt(ring)
     *
     * rq_ring_int() (pdp11_rq.c:3005-3014).  A one-WORD flag of 1 is DMAd to `comm + ring->ioff`
     * -- comm-4 for the command ring (SA_COMM_CI), comm-2 for the response ring (SA_COMM_RI), i.e.
     * BELOW the communications region proper, which is why rq_step4() zeroes from `comm + SA_COMM_CI`
     * rather than from `comm`.
     *
     * *** THE RESULT OF THAT WRITE IS DISCARDED. ***  The C casts it to `(void)` and its own comment
     * says "note that NXMs are ignored!".  So a ring interrupt whose flag word lands on an
     * unmapped page does NOT take the controller fatal, does NOT set the port error, and leaves the
     * ring index advancing normally -- which is the opposite of every other DMA in this file and is
     * a graded case rather than a footnote.
     *
     * The interrupt REQUEST itself (rq_setint) is recorded in `irq` and wired nowhere -- see the
     * file header's exclusion.  Note the condition is SA_S1H_VEC ALONE: unlike rq_init_int() it does
     * NOT test SA_S1H_IE, which is why mscpringdiff.js fences on VEC rather than on IE.
     *
     * @this {RQVAX}
     * @param {Object} ring
     */
    ringInt(ring)
    {
        let iadr = (this.comm + ring.ioff) >>> 0;           /* addr intr wd */
        this.flagBuf[0] = 1;
        this.flagBuf[1] = 0;
        this.cqbic.mapWriteW(iadr, 2, this.flagBuf);        /* write flag -- RESULT IGNORED */
        if (this.s1dat & SA_S1H_VEC) this.irq = 1;          /* if enb, intr */
    }

    /**
     * traceReq(line)
     *
     * One `sim_debug (DBG_REQ, ...)` line.  See the file header: this is a diagnostic stream, and
     * the five call sites are the five the C has.  Kept unbounded rather than ring-buffered because
     * its consumer truncates it per case; a run that never truncates it is a run whose consumer is
     * not the differential, and mscpringdiff.js asserts an absolute heap bound over the whole pass.
     *
     * @this {RQVAX}
     * @param {string} line
     */
    traceReq(line) { this.reqLog.push({t: this.cpu ? this.cpu.nTotalCycles : 0, line}); }

    /**
     * plf()
     *
     * rq_plf() builds and queues a "last failure" response PACKET, which is packet processing and
     * therefore out of this item's scope.  It is reachable ONLY by a host that sets SA_S4H_LF in its
     * step-4 GO word AFTER a fatal error, i.e. by a host that re-initialised a dead controller and
     * asked for the last-failure log.  Named and thrown rather than silently skipped: a case that
     * reaches it must fail the run, not quietly grade a controller that answered differently.
     *
     * @this {RQVAX}
     */
    plf()
    {
        throw new RQUnimplemented("rq.js: SA_S4H_LF with a pending port error requests a last-fail " +
            "PACKET -- MSCP packet processing is pcjsvax-6a5's later children, not pcjsvax-c2c");
    }

    /**
     * freeQueue()
     *
     * The free packet list as `SHOW RQ FREEQ` prints it, walked from `freq` through `pakLink` --
     * derived from the state, never a constructed list, so it reports what the controller actually
     * holds.  A malformed list would loop forever in the C's own walker; the RQ_NPKTS bound here
     * turns that into a reported anomaly instead.
     *
     * @this {RQVAX}
     * @returns {Array.<number>}
     */
    freeQueue()
    {
        let out = [];
        for (let p = this.freq; p && out.length <= RQ_NPKTS; p = this.pakLink[p]) out.push(p);
        return out;
    }

    /**
     * respQueue()
     *
     * The DEFERRED response list as `SHOW RQ RESPQ` walks it -- same discipline as freeQueue(), and
     * non-empty only while the host owns no response descriptor.  Bounded for the same reason.
     *
     * @this {RQVAX}
     * @returns {Array.<number>}
     */
    respQueue()
    {
        let out = [];
        for (let p = this.rspq; p && out.length <= RQ_NPKTS; p = this.pakLink[p]) out.push(p);
        return out;
    }
}

/* Published as class data so tests/mscpinitdiff.js's --selfcheck can PERTURB the shipped
   computation rather than substitute a copy of it (HANDOFF.md standing rule 11).  Every one of
   these is read by the methods above at call time, never captured in a closure. */
RQVAX.CTLR_TAB = CTLR_TAB;
RQVAX.RQ_SVER  = RQ_SVER;
RQVAX.RQ_HVER  = RQ_HVER;
RQVAX.RQ_CLASS = RQ_CLASS;
RQVAX.OP = OP;
RQVAX.ST = ST;
/** value -> name, inverted from OP so the dispatch never carries a second list of numbers. */
RQVAX.OP_NAME_OF = (function() {
    let m = {};
    for (let n of Object.keys(OP)) m[OP[n]] = n;
    return m;
})();
RQVAX.MSCP_UNIT_OPS = MSCP_UNIT_OPS;
RQVAX.MSCP_NOP_OPS = MSCP_NOP_OPS;
RQVAX.MSCP_SCC_OP = MSCP_SCC_OP;
RQVAX.MSCP_XFER_OPS = MSCP_XFER_OPS;
RQVAX.MSCP_OP_HANDLER = MSCP_OP_HANDLER;
RQVAX.MSCP_METHOD = MSCP_METHOD;
RQVAX.MSCP_EXCLUDED = MSCP_EXCLUDED;
RQVAX.MSCP_NOP_ARM = MSCP_NOP_ARM;
RQVAX.DRV_TAB = DRV_TAB;
RQVAX.RX33_DTYPE = RX33_DTYPE;
RQVAX.CMD_NAMES = CMD_NAMES;
RQVAX.RQUnimplemented = RQUnimplemented;

export {
    RQ_BASE, RQ_OFFSET, IOLN_RQ, RQDX3_CTYPE, CTLR_TAB, RQUnimplemented,
    CST_S1, CST_S1_WR, CST_S2, CST_S3, CST_S3_PPA, CST_S3_PPB, CST_S4, CST_UP, CST_DEAD, CST_NAMES,
    SA_ER, SA_S4, SA_S3, SA_S2, SA_S1,
    SA_S1C_NV, SA_S1C_Q22, SA_S1C_DI, SA_S1C_OD, SA_S1C_MP, SA_S1C_SM, SA_S1C_CN,
    SA_S1H_VL, SA_S1H_WR, SA_S1H_V_CQ, SA_S1H_M_CQ, SA_S1H_V_RQ, SA_S1H_M_RQ, SA_S1H_IE, SA_S1H_VEC,
    SA_S2C_PT, SA_S2C_V_EC, SA_S2C_M_EC, SA_S2H_CLO, SA_S2H_PI,
    SA_S3C_V_EC, SA_S3C_M_EC, SA_S3H_PP, SA_S3H_CHI,
    SA_S4C_V_MOD, SA_S4C_V_VER, SA_S4H_CS, SA_S4H_NN, SA_S4H_SF, SA_S4H_LF, SA_S4H_GO,
    PE_PRE, PE_PWE, PE_QRE, PE_QWE, PE_HAT, PE_ICI, PE_PIE, PE_PPF, PE_MRE, PE_NSR,
    SA_COMM_QQ, SA_COMM_PI, SA_COMM_CI, SA_COMM_RI, SA_COMM_MAX,
    UQ_DESC_OWN, UQ_DESC_F, UQ_ADDR, UQ_HDR_OFF,
    UQ_HLNT, UQ_HCTC, UQ_HCTC_V_CR, UQ_HCTC_M_CR, UQ_HCTC_V_TYP, UQ_HCTC_M_TYP,
    UQ_HCTC_V_CID, UQ_HCTC_M_CID, UQ_TYP_SEQ, UQ_TYP_DAT,
    UQ_CID_MSCP, UQ_CID_TMSCP, UQ_CID_DUP, UQ_CID_DIAG,
    OP, ST, I_OPCD, I_VRSN, I_FMTI, CMD_NAMES,
    MSCP_UNIT_OPS, MSCP_NOP_OPS, MSCP_SCC_OP, MSCP_XFER_OPS,
    MSCP_OP_HANDLER, MSCP_METHOD, MSCP_EXCLUDED, MSCP_NOP_ARM,
    SB_SUC_ON, SB_OFL_NV, SB_AVL_INU, SB_WPR_HW, UID_DISK,
    UF_RPL, UF_WPH, UF_WPS, UF_RMV, UF_CMW, UF_CMR, UF_MSK,
    MD_SWP, MD_NXU, MD_SPD,
    ABO_LNT, ABO_REFL, ABO_REFH, AVL_LNT,
    GCS_LNT, GCS_REFL, GCS_REFH, GCS_STSL, GCS_STSH,
    FMT_LNT, FMT_IH, FMT_IH_MAGIC,
    GUS_LNT_D, GUS_UVER, GUS_SHUN, GUS_SHST, GUS_TRK, GUS_GRP, GUS_CYL, GUS_RCTS, GUS_RBSC,
    GUS_RB_V_RBNS, GUS_RB_V_RCTC,
    ONL_LNT, ONL_MLUN, ONL_UFL, ONL_RSVL, ONL_RSVH,
    ONL_UIDA, ONL_UIDB, ONL_UIDC, ONL_UIDD, ONL_MEDL, ONL_MEDH,
    ONL_SHUN, ONL_SHST, ONL_SIZL, ONL_SIZH, ONL_VSNL, ONL_VSNH, ONL_VSN_BASE,
    ONL_UIDD_V_MOD, ONL_UIDD_V_CLS, SUC_LNT,
    RSP_REFL, RSP_REFH, RSP_UN, RSP_RSV,
    DRV_TAB, RQDF_RMV, RQDF_RO, RQDF_SDI, RQDF_DSSI,
    RX33_DTYPE, RD54_DTYPE, RX50_DTYPE,
    U_ATT, U_ONL, U_ATP, U_WLK, U_RO, U_DIS, U_NOAUTO,
    RQ_NUMBY, RQ_MAXDR, RQ_PSEUDO_UNITS, checkProvider, bufferProvider,
    CMD_REFL, CMD_REFH, CMD_UN, CMD_OPC, CMD_MOD, CMD_OPC_V_OPC, CMD_OPC_M_OPC,
    RSP_LNT, RSP_OPF, RSP_STS, RSP_OPF_V_OPC, RSP_OPF_V_FLG,
    SCC_LNT, SCC_MSV, SCC_CFL, SCC_TMO, SCC_VER, SCC_CIDA, SCC_CIDB, SCC_CIDC, SCC_CIDD,
    SCC_MBCL, SCC_MBCH, SCC_VER_V_SVER, SCC_VER_V_HVER, SCC_CIDD_V_MOD, SCC_CIDD_V_CLS,
    RW_BCL, RW_BCH, RW_BAL, RW_BAH, RW_LBNL, RW_LBNH,
    CF_RPL, CF_ATN, RQ_CLASS, RQ_HVER, RQ_SVER, RQ_DHTMO, RQ_DCTMO,
    RQ_NUMDR, RQ_NPKTS, RQ_M_NPKTS, RQ_PKT_SIZE_W, RQ_PKT_SIZE,
    RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME
};
