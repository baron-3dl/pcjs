/**
 * @fileoverview Implements the RQDX3 (MSCP/UQSSP) controller's four-step initialisation handshake
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
 * pcjsvax-c2c, the first of pcjsvax-6a5's children.  `open-simh/PDP11/pdp11_rq.c` is ~3,600 lines;
 * THIS FILE IS THE INITIALISATION STATE MACHINE ONLY -- rq_rd(), rq_wr(), rq_reset(), rq_step4(),
 * rq_fatal(), and the `csta < CST_UP` branch of rq_quesvc(), plus exactly as much of the CST_UP
 * branch as an IP-read poll of an EMPTY command ring reaches.  MSCP packet processing, disk I/O,
 * the response ring, attention/unit-available messages and interrupt delivery are NOT here; each
 * is named in the EXCLUSIONS section below with the fence that keeps a graded case from reaching it.
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
 * SCOPE EXCLUSIONS -- each with the fence that makes it unreachable, not merely unvisited
 * ============================================================================
 *   MSCP PACKET PROCESSING.  rq_quesvc()'s CST_UP branch runs the unit queues, then rq_getpkt(),
 *     then the response queue.  Implemented here: the unit-queue scan (every unit is idle in this
 *     item's configuration -- no disk is ever attached), rq_getpkt()'s descriptor fetch, and its
 *     `(desc & UQ_DESC_OWN) == 0` branch, which is the only one an empty ring can take.  If a
 *     descriptor ever arrives with OWN set, quesvc() throws RQUnimplemented by name rather than
 *     inventing an answer.  mscpinitdiff.js additionally asserts on the ORACLE that PBSY and
 *     RESP stayed 0 and FREE stayed 1 in every graded case, so a case that reached packet
 *     processing on the oracle fails the run instead of quietly grading a different program.
 *   INTERRUPTS.  rq_init_int() raises a controller interrupt only when the host's S1 data has BOTH
 *     SA_S1H_IE and a non-zero SA_S1H_VEC.  This file records the request in `irq` (as the C's
 *     `cp->irq`) and wires it to nothing; `irq` is not in SIMH's rq_reg[] so it is not examinable
 *     either.  mscpinitdiff.js FAILS the run if any graded case supplies an s1dat with IE and VEC
 *     both set, so the unwired path is unreachable rather than untested.  Vector computation
 *     (`dibp->vec = (s1dat & SA_S1H_VEC) << 2`) IS reproduced, in `vec`, because it is pure state.
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
 *   THE ROM MACHINE.  pcjsvax-c2c does NOT wire this controller into tests/rommachine.js.  The ROM's
 *     self-test 53 currently fails identically on both engines with no disk attached, and
 *     tests/conoutdiff.js's 115-byte agreement is the measurement that says so; changing what the
 *     ROM finds on the I/O page is a change to that measurement and belongs to whichever of
 *     pcjsvax-6a5's children owns it, with conoutdiff re-read rather than assumed.
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

const RQ_SVER       = 3;                        // software version
const RQ_DHTMO      = 60;                       // def host timeout
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
        this.units = Array.from({length: RQ_NUMDR}, () => ({cpkt: 0, pktq: 0}));

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
        for (let u of this.units) { u.cpkt = 0; u.pktq = 0; }
    }

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
     * The loop bound is not paranoia dressed as a constant: rq_quesvc() re-arms its own unit when it
     * has more to do (`if (pkt) sim_activate (uptr, rq_qtime)`), which in the C is a genuine
     * possibility and would make this loop run again.  It cannot happen in pcjsvax-c2c's scope --
     * that branch needs a packet -- so a bound that is exceeded means the state machine is looping,
     * and saying so by name beats hanging the differential.
     *
     * @this {RQVAX}
     * @param {Object} cpu
     */
    drainOnHalt(cpu)
    {
        this.cpu = cpu;
        for (let n = 0; this.queDue !== null; n++) {
            if (n > RQ_NPKTS) {
                throw new Error("rq.js: drainOnHalt() ran " + n + " services without the queue " +
                    "going idle -- rq_quesvc() is re-arming itself, which pcjsvax-c2c's scope cannot reach");
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
           The C carries a local `pkt` through this whole half and re-arms the unit at the end when
           it is non-zero.  Every statement that can make it non-zero -- rq_deqh() off a unit queue,
           rq_getpkt() off an owned descriptor, rq_deqh() off the response queue -- is a throw here,
           so a faithful transcription of the tail would be a branch nothing can take.  pcjsvax-855's
           lesson is the reason it is absent rather than transcribed: an unreachable branch that
           looks like coverage is worse than a stated gap.  Each of the three is named below. */
        for (let u of this.units) {
            if (u.cpkt || u.pktq === 0) continue;
            throw new RQUnimplemented("rq.js: a unit queue is non-empty -- MSCP packet processing " +
                "is pcjsvax-6a5's later children, not pcjsvax-c2c");
        }
        if (this.pip) {                                     /* polling? */
            let desc = this.getDesc(this.cq);
            if (desc === null) return;                      /* rq_getdesc failed -> fatal, thread ends */
            if (desc & UQ_DESC_OWN) {
                throw new RQUnimplemented("rq.js: the host owns a command descriptor -- MSCP packet " +
                    "processing is pcjsvax-6a5's later children, not pcjsvax-c2c");
            }
            this.pip = 0;                                   /* discontinue poll */
        }
        if (this.rspq) {
            throw new RQUnimplemented("rq.js: the response queue is non-empty -- MSCP packet " +
                "processing is pcjsvax-6a5's later children, not pcjsvax-c2c");
        }
    }

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
        if (!this.descBuf) this.descBuf = new Uint8Array(4);
        if (this.cqbic.mapReadW(addr, 4, this.descBuf)) {   /* fetch desc */
            this.fatal(PE_QRE);                             /* err? dead */
            return null;
        }
        return ((this.descBuf[0] | (this.descBuf[1] << 8) |
                 (this.descBuf[2] << 16) | (this.descBuf[3] << 24)) >>> 0);
    }

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
}

/* Published as class data so tests/mscpinitdiff.js's --selfcheck can PERTURB the shipped
   computation rather than substitute a copy of it (HANDOFF.md standing rule 11).  Every one of
   these is read by the methods above at call time, never captured in a closure. */
RQVAX.CTLR_TAB = CTLR_TAB;
RQVAX.RQ_SVER  = RQ_SVER;
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
    PE_PRE, PE_PWE, PE_QRE, PE_QWE, PE_HAT, PE_ICI, PE_PIE, PE_PPF, PE_MRE,
    SA_COMM_QQ, SA_COMM_PI, SA_COMM_CI, SA_COMM_RI, SA_COMM_MAX,
    UQ_DESC_OWN, UQ_DESC_F, UQ_ADDR, UQ_HDR_OFF,
    CF_RPL, CF_ATN, RQ_SVER, RQ_DHTMO, RQ_NUMDR, RQ_NPKTS, RQ_M_NPKTS, RQ_PKT_SIZE_W, RQ_PKT_SIZE,
    RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME
};
