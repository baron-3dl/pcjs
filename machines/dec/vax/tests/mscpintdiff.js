/**
 * @fileoverview Differential test: the RQDX3 interrupts the host at the vector the host programmed,
 *               and the SCB dispatch matches a real Open SIMH microvax3900's
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the Open SIMH VAX/PDP-11 simulators, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-aef, the sixth and last of pcjsvax-6a5's children.  The host programs a vector in
 * SA_S1H_VEC during initialisation step 1; the controller then interrupts at that vector, and this
 * grades WHERE the dispatch lands, WHEN, HOW MANY TIMES, IN WHAT ORDER and AT WHAT IPL -- against a
 * real simulator, on both engines, with the interrupts actually EXECUTED.
 *
 * *** THE INVOCATIONS ARE OBSERVED BY RUNNING THE HANDLER, NOT BY INSPECTING A FLAG. ***
 * HANDOFF.md 7 premise 7: pcjsvax-855 shipped code nothing could reach and a unit-level test would
 * have "passed" it.  So every SCB slot in the DEVICE half of the SCB (0x100..0x3FC, 192 of them)
 * gets its OWN 26-byte handler, and each one RECORDS ITS OWN INVOCATION into a physical log:
 *
 *      MOVL  (SP), (R13)          the interrupted PC   -- the frame intexc() just pushed
 *      MOVL  4(SP), 4(R13)        the interrupted PSL
 *      MOVPSL 8(R13)              the PSL the dispatch INSTALLED -- this is where IPL is read
 *      MOVL  I^#<scb offset>, 12(R13)   WHICH SCB SLOT dispatched
 *      ADDL2 S^#16, R13           advance
 *      REI
 *
 * R13 is the log pointer, set by the host's first instruction and touched by nothing else, so its
 * FINAL VALUE IS THE INVOCATION COUNT and the log's ORDER is the dispatch order.  Both engines are
 * compared on the whole log page, on R13, and on every register/PSL besides.  A handler per slot is
 * what makes "which vector did it dispatch to" a MEASUREMENT rather than an inference: nothing is
 * asked, the slot that ran says so itself.
 *
 * WHAT THE MEASUREMENT FOUND, AND WHY THE ITEM'S OWN PREMISE WAS INCOMPLETE
 * -------------------------------------------------------------------------
 * HANDOFF.md 7 premise 5 records that `TMR_VEC_MASK 0x3FC` is not the mask that reaches the SCB --
 * `QB_VEC_MASK 0x1FC` is.  TRUE FOR THE SSC TIMERS.  It is NOT the whole story for this device, and
 * the difference is not a mask at all:
 *
 *   get_vector() (vax_io.c:443-455) ends with
 *          vec |= int_vec_set[l][i];
 *          vec &= (int_vec_set[l][i] | QB_VEC_MASK);
 *   and pdp11_io_lib.c's build_vector_tab() sets int_vec_set[IPL_RQ][INT_V_RQ] = VEC_SET = 0x201
 *   for RQ, because RQ is DEV_QBUS and appears in auto_tab.  So an RQ vector is OR'd with 0x201 and
 *   masked with 0x3FD -- the mask is WIDENED BY WHAT WAS OR'D IN, and both new bits survive:
 *      0x200  moves the dispatch into the SECOND PAGE of the SCB, where Qbus vectors belong;
 *      0x001  is VEC_QBUS, which intexc() (vax_cpu1.c:1125) reads to force the new PSL<IPL> to
 *             0x17 -- so a controller that requests at BR4 runs its handler at IPL 0x17.
 *
 * MEASURED on a real unpatched microvax3900 before a line of this was written:
 *   - fresh oracle, `show rq` says "no vector"; the vector does not exist until step 1;
 *   - S1 vector field 0x7F -> `examine rq devvec` says 01FC, `show rq` says `vector=3FC*`, and
 *     THREE dispatches land on SCB offset 0x3FC with PSL 0x00170004 at each;
 *   - S1 vector field 0x01 -> devvec 0004, `vector=204`, dispatches on SCB 0x204;
 *   - S1 vector 0x7F with SA_S1H_IE CLEAR -> devvec 01FC and ZERO dispatches during the handshake;
 *   - SA_S1H_IE set with vector 0 -> devvec 0000 and zero dispatches.
 *
 * AND QB_VEC_MASK IS A NO-OP OVER THIS DEVICE'S WHOLE LEGAL RANGE, which is exactly the shape of
 * premise 5 and is therefore ASSERTED rather than argued: SA_S1H_VEC is 0x007F and rq_quesvc computes
 * `dibp->vec = (s1dat & SA_S1H_VEC) << 2`, so the largest programmable vector is 0x1FC; `deposit rq
 * devvec 2A8` is refused by SCP ("Read only argument"), so no back door reaches a larger one either.
 * assertMaskIsNoop() below ENUMERATES ALL 128 legal vector values and FAILS the run if any of them
 * would lose a bit to the mask -- so "we could not cover the folded case" is a live tripwire that
 * fires the day the range changes, not a note.  The mask IS still graded, on the device that CAN
 * exceed it: tests/hwintdiff.js's `dynamic_tmr0_truncated` (0x2A8 -> 0x0A8).
 *
 * THE ASYMMETRY A SINGLE TEST CANNOT SEE
 * --------------------------------------
 *      rq_init_int:  if ((s1dat & SA_S1H_IE) && (s1dat & SA_S1H_VEC)) rq_setint (cp);
 *      rq_ring_int:  if (s1dat & SA_S1H_VEC) rq_setint (cp);            <- NO IE TEST
 * A case that always sets both, or always clears both, cannot tell those apart.  So the case list
 * carries all four quadrants, and the decisive one is IE CLEAR / VEC SET: it must produce ZERO
 * interrupts across the entire four-step handshake and then a REAL interrupt the moment a ring
 * descriptor transitions.  coverage() FAILS the run if that case is missing or if it does not show
 * exactly that shape on the ORACLE.
 *
 * That premise is itself derived, not trusted: tests/mscpscope.js's intSeam() extracts both guards
 * from pdp11_rq.c and FAILS if they ever become the same condition -- because a coverage floor
 * cannot notice its own premise dissolving.  The three-and-two CALL SITE counts this file's floors
 * are sized from come from the same extraction.
 *
 * WHAT IS DELIBERATELY NOT GRADED, BY NAME (standing rule 6)
 * ----------------------------------------------------------
 *   - EVERYTHING THE OTHER FIVE MSCP DIFFERENTIALS GRADE.  Handshake register values, ring
 *     descriptor mechanics, unit commands, transfers and the refusal ladder are theirs.  This file
 *     drives exactly as much of each as it needs to make an interrupt happen, and grades the
 *     controller registers only as corroboration.  The ONE MSCP command it ever sends is SET
 *     CONTROLLER CHARACTERISTICS (OP_SCC), which needs no attached unit; assertExclusions() FAILS
 *     the run if a case sends anything else.
 *   - THE OTHER THREE CONTROLLERS.  rq_clrint()/rq_inta() walk all four contexts; RQB/RQC/RQD are
 *     DEV_DIS on the oracle so `peers` is `[this]`, the walk is written and reduces to one pass,
 *     and there is no oracle for a second controller's interference at all.
 *   - int_vec_set FOR ANY OTHER DEVICE.  RQ is the only autoconfigured Qbus device this tree
 *     constructs.  exc.js's seam takes the value per bit; nothing here asserts what RL or XQ would
 *     get, because neither exists.
 *
 *      node machines/dec/vax/tests/mscpintdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch build)
 *        --cases N         randomized cases (default RANDOM_CASES_DEFAULT; below the fixed floor
 *                           the run FAILS rather than clamping up)
 *        --seed S          PRNG seed, printed on every run so a failure is reproducible
 *        --selfcheck       prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";

import RQVAX, {
    RQUnimplemented, RQ_BASE, IOLN_RQ, CST_S1, CST_S2, CST_UP,
    SA_S1H_VL, SA_S1H_IE, SA_S1H_VEC, SA_S1H_V_CQ, SA_S1H_V_RQ,
    UQ_DESC_OWN, UQ_DESC_F, UQ_HLNT, UQ_HCTC, UQ_HCTC_V_TYP, UQ_HCTC_V_CID,
    UQ_TYP_SEQ, UQ_CID_MSCP, OP, CMD_OPC, CMD_MOD, CMD_UN, CMD_REFL, CMD_REFH,
    SCC_LNT, SCC_MSV, RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME
} from "../modules/v2/rq.js";
import { QB_VEC_MASK, VEC_SET } from "../modules/v2/exc.js";
import {
    PAGE, R_CODE, R_RESULT, R_SCBB, R_MCHK_HDLR, R_MERR_HDLR, MAP_MBR, OBS_REGS, MEM_MB,
    RQ_IP, RQ_SA, CQBIC_BASE, CQMAP_BASE, CQMAP_VLD, SCB,
    hex, findSimhBin, runSimh, mulberry32, sampleHeap, peakHeap,
    OPC, opcodeOf, Asm, machine, RQ_OBS, rqFieldOf, geometry, qbusPagesFor,
    physPageFor, seedFor, walkScript, emitAction, simhResetLines, jsResetForCase
} from "./mscpharness.js";
import { extract as scopeExtract } from "./mscpscope.js";

/** An absolute bound on the instructions any case may execute.  A case that does not HALT within it
    is reported BY NAME rather than compared at whatever PC it happened to reach -- which is also
    what a runaway re-delivery (an acknowledge that fails to clear the request) looks like. */
const MAX_STEPS = 60000;

const RANDOM_CASES_DEFAULT = 16;
const RANDOM_CASES_FLOOR   = 8;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure and NOT scaled by case
    count (standing rules 4 and 14).  ONE machine is built and reused. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;

/* ------------------------------------------------------------------------------------------- *
 * The SCB handler bank and the interrupt log                                                    *
 * ------------------------------------------------------------------------------------------- */

/** The DEVICE half of the SCB: every offset a Qbus vector can legally reach.  0x100 is SCB_INTR --
    everything below it is a CPU exception and belongs to other differentials. */
const SCB_LO = 0x100, SCB_HI = 0x3FC;
const SCB_SLOTS = (SCB_HI - SCB_LO) / 4 + 1;

/** One handler per slot, 32 bytes apart (26 used).  Placed FAR above everything else any MSCP
    differential touches, and with megabytes of empty RAM above the log, so that a defect which
    re-delivers forever writes into nothing and is reported as "never reached its own HALT" rather
    than by corrupting the instrument that is supposed to notice it. */
const R_HDLR   = 0x00700000;
const HDLR_STRIDE = 32;
const R_INTLOG = 0x00800000;
/** Records DUMPED and compared -- one 512-byte page.  A case that records more than this is FAILED
    by name (see gradeLog()); no graded case comes near it. */
const LOG_RECS = 32;
const LOG_REC_BYTES = 16;

function hdlrAddr(scb) { return (R_HDLR + ((scb - SCB_LO) >> 2) * HDLR_STRIDE) >>> 0; }

function lw(v) { v >>>= 0; return [v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF]; }

/**
 * handlerBytes(scb)
 *
 * The six instructions above, hand-encoded.  The opcode NUMBERS come from mscpharness's OPC table
 * (which reads drom.js), never transcribed; the ADDRESSING MODES are literal because the harness's
 * assembler has no emitter for register-deferred or displacement operands and this is the only
 * place in the tree that needs them.
 *
 *   0x6D  (R13)      register deferred      0x6E  (SP)
 *   0xAD  d(R13)     byte displacement      0xAE  d(SP)
 *   0x5D  R13        register               0x8F  I^#imm      0x10  S^#16
 */
const OPC_MOVPSL = opcodeOf("MOVPSL"), OPC_REI = opcodeOf("REI"), OPC_ADDL2 = opcodeOf("ADDL2");
const OPC_MTPR = opcodeOf("MTPR");
/** vax_defs.h's MT_IPL.  The ONE privileged register this file writes: `MTPR I^#n, S^#18` raises
    PSL<IPL> so a request can be RAISED AND LEFT PENDING while the host does something else to the
    controller -- which is the only way to observe that rq_reset() withdraws it. */
const MT_IPL = 18;

function handlerBytes(scb)
{
    return [
        OPC.MOVL, 0x6E, 0x6D,                       /* MOVL  (SP), (R13)        interrupted PC   */
        OPC.MOVL, 0xAE, 0x04, 0xAD, 0x04,           /* MOVL  4(SP), 4(R13)      interrupted PSL  */
        OPC_MOVPSL, 0xAD, 0x08,                     /* MOVPSL 8(R13)            the NEW PSL      */
        OPC.MOVL, 0x8F, ...lw(scb), 0xAD, 0x0C,     /* MOVL  I^#scb, 12(R13)    which SCB slot   */
        OPC_ADDL2, 0x10, 0x5D,                      /* ADDL2 S^#16, R13                          */
        OPC_REI                                     /* REI                                       */
    ];
}

if (handlerBytes(0).length > HDLR_STRIDE) throw new Error("mscpintdiff: handler does not fit its stride");

/** R13 is the log pointer and NOTHING else may touch it.  R0..R8 are walkScript's; R9..R12 are the
    ring phase's.  R14 is SP. */
const LOG_REG = 13;
const REGS = {prev: 9, cur: 10, cnt: 11, tmp: 12};

/* ------------------------------------------------------------------------------------------- *
 * The vector arithmetic, written a SECOND time here on purpose                                  *
 * ------------------------------------------------------------------------------------------- */

/**
 * deliveredVec(progVec) / deliveredSCB(progVec)
 *
 * get_vector()'s answer for THIS device, computed from the value the host programmed into
 * SA_S1H_VEC -- not by asking exc.js or rq.js, which would grade a defect against itself.  This is
 * the same discipline mscpharness's commExtent() is under.
 */
function deliveredVec(progVec)
{
    let vec = (progVec & SA_S1H_VEC) << 2;              /* rq_quesvc's CST_S1 arm */
    vec = vec | VEC_SET;                                /* int_vec_set[][]        */
    return vec & (VEC_SET | QB_VEC_MASK);
}
function deliveredSCB(progVec) { return deliveredVec(progVec) & ~3; }

/** pdp11_io_lib.c's AUTO_VECBASE (0300 octal).  Used only by the `*` in `show rq`'s vector line. */
const AUTO_VECBASE = 0o300;

/**
 * showVecLine(vec)
 *
 * show_vec() (pdp11_io_lib.c:225-277) for the ONE device this tree has, reduced to the text between
 * `address=...` and `, RQDX3`.  A RENDERING, graded as text against the oracle's own `show rq`, and
 * a SECOND independent view of the same `vec`: `examine rq devvec` shows the RAW DIB vector and this
 * shows the VEC_SET-folded one, so a defect in the fold shows up here even though devvec agrees.
 *
 * The `*` test reads the FOLDED value when the vector is non-zero and the raw 0 when it is not,
 * because the `#if (VEC_SET != 0)` block reassigns `vec` before it -- reproduced, not tidied.
 */
function showVecLine(vec)
{
    vec = vec >>> 0;
    let out, shown = vec;
    if (vec === 0) out = "no vector";
    else {
        shown = (vec | (VEC_SET & ~3)) & (VEC_SET | 0x1FF);
        out = "vector=" + shown.toString(16).toUpperCase();
    }
    if (shown >= ((VEC_SET | AUTO_VECBASE) & ~3)) out += "*";
    /* `br_lvl = dibp->vloc / 32` == 0, then `+ 4` on a VAX. */
    return out + ", BR4";
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

/**
 * sccWords(o)
 *
 * A SET CONTROLLER CHARACTERISTICS command packet -- the only MSCP command this file sends.  It
 * needs no attached unit and no transfer, so a ring transition can be produced without dragging in
 * anything tests/mscpunitdiff.js or tests/mscprwdiff.js own.
 */
function sccWords(o = {})
{
    let w = {};
    w[UQ_HLNT] = SCC_LNT;
    w[UQ_HCTC] = ((UQ_TYP_SEQ << UQ_HCTC_V_TYP) | (UQ_CID_MSCP << UQ_HCTC_V_CID)) & 0xFFFF;
    w[CMD_REFL] = (o.ref === undefined ? 0xBEEF : o.ref) & 0xFFFF;
    w[CMD_REFH] = 0x1234;
    w[CMD_UN] = 0;
    w[CMD_OPC] = OP.SCC;
    w[CMD_MOD] = 0;
    w[SCC_MSV] = 0;                                     /* MSCP version 0, or rq_scc() refuses */
    return w;
}

/**
 * buildCase(spec)
 *
 * spec:
 *   vec      the value programmed into SA_S1H_VEC (0..0x7F)
 *   ie       SA_S1H_IE set?
 *   cqCode/rqCode  ring length codes
 *   comm     the comm region's Qbus address
 *   stopAfter  how many of walkScript()'s nine actions to emit (undefined = all nine)
 *   pp       use the purge/poll variant of step 3
 *   cmds     [{cslot, rslot, pkt, rbuf, grantPrev}] -- SCC commands to push through the rings
 */
function buildCase(spec)
{
    let c = Object.assign({
        itime: RQ_ITIME, i4time: RQ_ITIME4, qtime: RQ_QTIME, xtime: RQ_XTIME,
        vec: 0, ie: 0, cqCode: 0, rqCode: 0, comm: 0x2000, prgi: 0, spread: 0,
        pp: false, cmds: [], grants: [], head: [], tail: [], noStopFloor: false
    }, spec);

    c.s1dat = (SA_S1H_VL | (c.ie ? SA_S1H_IE : 0) | (c.vec & SA_S1H_VEC) |
               (c.cqCode << SA_S1H_V_CQ) | (c.rqCode << SA_S1H_V_RQ)) & 0xFFFF;
    /* Enough packet buffers for every packet a case names -- commands AND the descriptors it
       pre-grants.  Sized from the case rather than from `cmds.length`: a grant naming buffer 1 with
       only one buffer allocated puts that buffer's envelope on a Qbus page the map never programs,
       and the controller takes a PE_PRE fatal instead of doing what the case asked. */
    c.nCmdBuf = Math.max(1, ...c.cmds.map((x) => x.pkt + 1),
                         ...c.grants.filter((g) => g.ring === "cq").map((g) => g.buf + 1));
    c.nRspBuf = Math.max(1, ...c.cmds.map((x) => (x.rbuf === undefined ? x.pkt : x.rbuf) + 1),
                         ...c.grants.filter((g) => g.ring !== "cq").map((g) => g.buf + 1));
    c.g = geometry(c);

    let qpages = qbusPagesFor(c.g);
    c.entries = qpages.map((q) => ({q, p: physPageFor(q, c.spread)}));
    c.qToP = new Map(c.entries.map((e) => [e.q, e.p]));
    c.zeroIdx = qpages.slice();
    c.dumpPages = [...new Set(c.entries.map((e) => e.p))].sort((a, b) => a - b);
    c.resultPage = (R_RESULT / PAGE) | 0;

    /* The Qbus -> physical translation done a SECOND time (mscpringdiff's discipline): the host
       addresses PHYSICAL memory, the controller addresses QBUS memory through the map, and a
       disagreement shows up as a memory difference rather than as a silent pass. */
    c.phys = (qaddr) => {
        let q = (qaddr / PAGE) | 0;
        if (!c.qToP.has(q)) throw new Error(`mscpintdiff: case "${c.name}" addresses unmapped Qbus page ${q}`);
        return (c.qToP.get(q) * PAGE + (qaddr % PAGE)) >>> 0;
    };

    /* Command packets, planted as physical longword deposits on both engines.  What this file is
       about is the INTERRUPT, so the packet gets there the cheap way; tests/mscpringdiff.js is what
       proves the CPU can build one. */
    c.presets = [];
    for (let cm of c.cmds) {
        let w = sccWords({ref: 0xBE00 | cm.pkt});
        for (let k of Object.keys(w)) {
            c.presets.push({addr: c.phys(c.g.cmdBuf(cm.pkt) + (+k) * 2), word: w[k] & 0xFFFF});
        }
    }

    /* RESULT-page slot assignment, done in the SAME walk the emitter uses. */
    c.slots = [];
    let off = 0;
    for (let cm of c.cmds) {
        cm.roff = off;
        c.slots.push({off, what: `cmd${cm.pkt} await iterations`});
        c.slots.push({off: off + 4, what: `cmd${cm.pkt} response descriptor`});
        off += 8;
    }
    if (off > PAGE) throw new Error(`mscpintdiff: case "${c.name}" needs ${off} RESULT bytes`);

    let a = new Asm();
    /* *** THE LOG POINTER IS THE FIRST INSTRUCTION. ***  An init interrupt can arrive as early as
       ITIME instructions after the step-1 write, so R13 has to be valid before anything else runs. */
    a.emit(OPC.MOVL, 0x8F, ...lw(R_INTLOG), 0x50 | LOG_REG);
    a.movImmAbs(4, MAP_MBR, (CQBIC_BASE + 4 * 4) >>> 0);            // REG_MBR == 4
    for (let e of c.entries) a.movImmAbs(4, (CQMAP_VLD | e.p) >>> 0, (CQMAP_BASE + e.q * 4) >>> 0);

    for (let act of c.head) emitStep(a, act, c);
    let walk = walkScript(c.s1dat, c.comm, c.prgi, c.pp ? {pp: true, ppaDelay: 1200} : {});
    c.walkLen = walk.length;
    let used = (c.stopAfter === undefined) ? walk : walk.slice(0, c.stopAfter);
    for (let act of used) {
        if (!emitAction(a, act)) throw new Error(`mscpintdiff: unknown handshake action "${act.a}"`);
    }

    for (let act of c.tail) emitStep(a, act, c);

    let descOf = (own, flag, addr) => (((own ? UQ_DESC_OWN : 0) | (flag ? UQ_DESC_F : 0) | addr) >>> 0);
    /* Descriptors the host grants BEFORE any command, used to make rq_putdesc()'s PREVIOUS-descriptor
       arm reachable: with a ring longer than one slot, a putdesc at index 0 reads slot lnt-1, and the
       interrupt fires only if that slot is still owned. */
    for (let gr of c.grants) {
        a.movImmAbs(4, descOf(true, gr.flag !== false, (gr.ring === "cq" ? c.g.cmdEnv(gr.buf) : c.g.rspEnv(gr.buf))),
                    c.phys((gr.ring === "cq" ? c.g.cqBa : c.g.rqBa) + gr.slot * 4));
    }
    for (let cm of c.cmds) {
        a.movImmAbs(4, descOf(true, true, c.g.rspEnv(cm.rbuf === undefined ? cm.pkt : cm.rbuf)),
                    c.phys(c.g.rqBa + cm.rslot * 4));               /* grant a response slot */
        a.movImmAbs(4, descOf(true, true, c.g.cmdEnv(cm.pkt)),
                    c.phys(c.g.cqBa + cm.cslot * 4));               /* post the command       */
        a.movAbsReg(2, RQ_IP, REGS.tmp);                            /* ring the doorbell      */
        /* UNBOUNDED (standing rule 17): an SCC the controller has accepted MUST be answered, so a
           budget here could only ever make the observation point probable instead of certain. */
        a.awaitUnbounded(c.phys(c.g.rqBa + cm.rslot * 4), (R_RESULT + cm.roff) >>> 0, REGS);
        a.delay(400, REGS.tmp);                                     /* let the controller settle */
    }
    a.halt();
    c.code = a.b;
    c.haltPC = (R_CODE + c.code.length) >>> 0;
    if (c.code.length > 0x1000) throw new Error(`mscpintdiff: case "${c.name}" code is ${c.code.length} bytes`);
    return c;
}

/**
 * emitStep(a, act, c)
 *
 * mscpharness's action vocabulary plus the ONE thing only this file needs: raising PSL<IPL> so a
 * request can be raised and left pending.  Every action is still a REAL instruction against a REAL
 * address; there is no "call the device" action here either.
 */
function emitStep(a, act, c)
{
    if (act.a === "ipl") {
        /* MTPR I^#n, S^#18 */
        return a.emit(OPC_MTPR, 0x8F, ...lw(act.v), MT_IPL & 0x3F);
    }
    if (emitAction(a, act)) return a;
    throw new Error(`mscpintdiff: case "${c.name}" uses unknown action "${act.a}"`);
}

/* ------------------------------------------------------------------------------------------- *
 * The enumerated case list -- a fixed matrix, not a sample; nothing here scales with --cases    *
 * ------------------------------------------------------------------------------------------- */

/** The largest legal SA_S1H_VEC, the smallest non-zero one, one either side of the `*` threshold,
    and zero.  0x7F is in the list because "the mask is a no-op over the legal range" and "the mask
    is not applied" are only distinguishable at the top of the range. */
const VEC_MAX = SA_S1H_VEC;                             /* 0x7F, derived from the field mask */

function enumeratedCases()
{
    let cases = [];
    let add = (spec) => { let c = buildCase(spec); c.idx = cases.length; cases.push(c); return c; };

    /* ---- FAMILY 1: the four IE x VEC quadrants over a FULL handshake.  These are what separate
       rq_init_int()'s two-bit condition from rq_ring_int()'s one-bit condition, and each carries a
       command through the rings so BOTH conditions are exercised in the same case. ---- */
    for (let q of [{ie: 1, vec: VEC_MAX, n: "ie+vecmax"}, {ie: 1, vec: 0x01, n: "ie+vecmin"},
                   {ie: 0, vec: VEC_MAX, n: "noie+vecmax"}, {ie: 1, vec: 0, n: "ie+vec0"},
                   {ie: 0, vec: 0, n: "noie+vec0"}]) {
        add({name: `quadrant ${q.n}`, ie: q.ie, vec: q.vec,
             cmds: [{cslot: 0, rslot: 0, pkt: 0}]});
    }

    /* ---- FAMILY 2: WHERE the dispatch lands, across the whole legal vector range.  One case per
       vector, handshake only.  0x30 is the smallest value whose delivered vector reaches
       `(VEC_SET | AUTO_VECBASE) & ~3` and therefore prints with SIMH's `*`; 0x2F is the largest
       that does not, so the pair straddles a boundary the ORACLE ITSELF renders differently. ---- */
    for (let v of [0x01, 0x02, 0x0F, 0x2F, 0x30, 0x40, 0x55, 0x7E, VEC_MAX]) {
        add({name: `vector ${hex(v, 2)} -> SCB ${hex(deliveredSCB(v), 3)}`, ie: 1, vec: v});
    }

    /* ---- FAMILY 3: WHEN.  One case per handshake step, so the invocation COUNT itself attributes
       each init interrupt to the transition that raised it (1, 2, 3) rather than leaving all three
       inside one case's log.  Sized from mscpscope's derived rq_init_int() call-site count. ---- */
    add({name: "init stop after step 1 (expect 1 interrupt)", ie: 1, vec: 0x21, stopAfter: 3});
    add({name: "init stop after step 2 (expect 2 interrupts)", ie: 1, vec: 0x21, stopAfter: 5});
    add({name: "init stop after step 3 (expect 3 interrupts)", ie: 1, vec: 0x21, stopAfter: 7});
    add({name: "init full walk (expect 3 interrupts)", ie: 1, vec: 0x21});
    /* rq_step4() reached through the PURGE/POLL arm instead of directly -- a different path to the
       SAME rq_init_int() call site. */
    add({name: "init via purge/poll", ie: 1, vec: 0x22, pp: true});

    /* ---- FAMILY 4: the two rq_putdesc() ring-interrupt arms.
       `ring->lnt <= 4` fires unconditionally on a ONE-SLOT ring; with a longer ring the interrupt
       comes only from the PREVIOUS-descriptor rule, so a two-slot ring whose other slot is still
       owned is the only way to reach that arm. ---- */
    add({name: "ring lnt=1, both arms' first (IE set)", ie: 1, vec: 0x11,
         cqCode: 0, rqCode: 0, cmds: [{cslot: 0, rslot: 0, pkt: 0}]});
    add({name: "ring lnt=1, IE CLEAR -- ring interrupt without init interrupts", ie: 0, vec: 0x12,
         cqCode: 0, rqCode: 0, cmds: [{cslot: 0, rslot: 0, pkt: 0}]});
    /* ONLY the response ring is pre-granted.  A pre-granted COMMAND descriptor would hand the
       controller a packet buffer full of this file's page seed, which is not an MSCP command and
       drags in scope tests/mscperrdiff.js owns; the response ring's previous-descriptor rule is
       reachable without it, because rq_putpkt() writes response slot 0 and reads slot 1. */
    add({name: "ring lnt=2, previous-descriptor rule (IE set)", ie: 1, vec: 0x13,
         cqCode: 1, rqCode: 1,
         grants: [{ring: "rq", slot: 1, buf: 1}],
         cmds: [{cslot: 0, rslot: 0, pkt: 0}]});
    add({name: "ring lnt=2, previous-descriptor rule, IE CLEAR", ie: 0, vec: 0x14,
         cqCode: 1, rqCode: 1,
         grants: [{ring: "rq", slot: 1, buf: 1}],
         cmds: [{cslot: 0, rslot: 0, pkt: 0}]});
    /* TWO commands, so the log carries more than one ring interrupt and its ORDER is graded. */
    add({name: "two commands, ring lnt=2, IE set", ie: 1, vec: 0x15,
         cqCode: 1, rqCode: 1,
         cmds: [{cslot: 0, rslot: 0, pkt: 0}, {cslot: 1, rslot: 1, pkt: 1}]});

    /* ---- FAMILY 5: the acknowledge CLEARS the request and it is NOT re-raised.  A long settling
       delay after the last interrupt with nothing left to raise one: any re-delivery shows up as
       extra log records on one engine and not the other. ---- */
    add({name: "acknowledge clears -- long settle, no re-delivery", ie: 1, vec: 0x7D,
         cmds: [{cslot: 0, rslot: 0, pkt: 0}], settleLong: true});

    /* ---- FAMILY 5b: WHAT A RESET WITHDRAWS.  Two facts the C states in two different lines of
       rq_reset() -- `dibp->vec = 0` and `rq_clrint (cp)` -- and neither is observable from any case
       above, because every one of them programs a vector and then leaves it programmed.
       The first is read straight off the oracle: `show rq` goes back to saying "no vector".
       The second needs the request to be PENDING AND UNDELIVERED when the reset happens, which is
       what the MTPR raising PSL<IPL> to 0x17 is for: the init interrupt is raised while masked, the
       IP write resets the controller, and lowering IPL afterwards must dispatch NOTHING.  Note what
       a defect here would look like: with the master request left set and `dibp->vec` zeroed,
       get_vector() returns `0 | VEC_SET` -- NOT zero -- so it dispatches to SCB 0x200, an interrupt
       from a controller that has none. ---- */
    add({name: "reset clears the programmed vector", ie: 0, vec: 0x66, stopAfter: 3,
         noStopFloor: true,
         tail: [{a: "wip", v: 0}, {a: "delay", n: 800, r: REGS.tmp}, {a: "rsa", r: 0}]});
    add({name: "reset withdraws an interrupt raised while masked", ie: 1, vec: 0x67, stopAfter: 2,
         noStopFloor: true,
         head: [{a: "ipl", v: 0x17}],
         tail: [{a: "delay", n: 1200, r: REGS.tmp},          /* the init interrupt is raised HERE */
                {a: "wip", v: 0},                            /* ... and the reset must withdraw it */
                {a: "delay", n: 200, r: REGS.tmp},
                {a: "ipl", v: 0},                            /* unmask: nothing may dispatch */
                {a: "delay", n: 400, r: REGS.tmp},
                {a: "rsa", r: 0}]});

    /* ---- FAMILY 6: a comm region above 64KB, so the step-3 shift is exercised while interrupts
       are being delivered (the two have no reason to interact and the case proves they do not). ---- */
    add({name: "comm above 64KB with interrupts enabled", ie: 1, vec: 0x39, comm: 0x12000,
         cmds: [{cslot: 0, rslot: 0, pkt: 0}]});

    return cases;
}

/**
 * randomCases(n, seed, startIdx)
 *
 * Standing rule 1's randomized phase.  Uniform draws over the vector field, the IE bit, the ring
 * codes and the comm region -- the enumerated list above is chosen and therefore blind between its
 * choices, and this is not.  Every draw is a LEGAL host program: the vector is masked to
 * SA_S1H_VEC, so this phase is also the exhaustive-in-expectation check that no legal vector
 * dispatches anywhere but where deliveredSCB() says.
 */
function randomCases(n, seed, startIdx)
{
    let rnd = mulberry32(seed), out = [];
    for (let i = 0; i < n; i++) {
        let vec = Math.floor(rnd() * 0x80) & SA_S1H_VEC;
        let ie = rnd() < 0.5 ? 1 : 0;
        let code = Math.floor(rnd() * 2);               /* 0 or 1 -- one or two ring slots */
        let comm = (0x2000 + Math.floor(rnd() * 8) * 0x1000) >>> 0;
        let withCmd = rnd() < 0.75;
        let c = buildCase({
            name: `random#${i} vec=${hex(vec, 2)} ie=${ie} code=${code} comm=${hex(comm, 6)}` +
                  (withCmd ? " +cmd" : ""),
            vec, ie, cqCode: code, rqCode: code, comm, spread: i,
            cmds: withCmd ? [{cslot: 0, rslot: 0, pkt: 0}] : []
        });
        c.idx = startIdx + i;
        out.push(c);
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side -- ONE invocation for the whole case list                                       *
 * ------------------------------------------------------------------------------------------- */

const MARK = "MSICASE";

/** The handler bank, deposited ONCE.  `reset -p all` does not clear memory (measured: a bank
    deposited before the first case still dispatches in the last one), so 4,992 deposit lines are
    paid once rather than per case. */
function handlerBankLines()
{
    let L = [];
    for (let s = SCB_LO; s <= SCB_HI; s += 4) {
        let a = hdlrAddr(s), b = handlerBytes(s);
        for (let i = 0; i < b.length; i++) L.push(`deposit -b ${hex(a + i)} ${b[i].toString(16)}`);
    }
    return L;
}

function simhCaseLines(c)
{
    let L = [];
    L.push(`echo ${MARK}${c.idx}`);
    L.push(...simhResetLines(c));
    /* The CPU half of the SCB, zeroed and then given back its two harness handlers.  A dispatch
       that lands below 0x100 therefore reads 0, jumps to PC 0 and HALTs there -- reported by name as
       a case that did not reach its own HALT, instead of running off into whatever was there. */
    L.push(`deposit -l ${hex(R_SCBB)}:${hex(R_SCBB + 0xFC)} 0`);
    L.push(`deposit -l ${hex((R_SCBB + SCB.MCHK) >>> 0)} ${hex(R_MCHK_HDLR)}`);
    L.push(`deposit -l ${hex((R_SCBB + SCB.MEMERR) >>> 0)} ${hex(R_MERR_HDLR)}`);
    for (let s = SCB_LO; s <= SCB_HI; s += 4) L.push(`deposit -l ${hex(R_SCBB + s)} ${hex(hdlrAddr(s))}`);
    L.push(`deposit -l ${hex(R_INTLOG)}:${hex(R_INTLOG + LOG_RECS * LOG_REC_BYTES - 4)} 0`);
    L.push(`deposit -l ${hex(R_RESULT)}:${hex(R_RESULT + PAGE - 4)} 0`);
    for (let q of c.zeroIdx) L.push(`deposit -l ${hex(CQMAP_BASE + q * 4)} 0`);
    for (let p of c.dumpPages) L.push(`deposit -l ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)} ${hex(seedFor(p))}`);
    for (let pr of c.presets) L.push(`deposit -w ${hex(pr.addr)} ${hex(pr.word, 4)}`);
    for (let k = 0; k < c.code.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${c.code[k].toString(16)}`);
    L.push("deposit PSL 0", `deposit PC ${hex(R_CODE)}`);
    L.push(`step ${MAX_STEPS}`);
    L.push(`examine -h ${Array.from({length: OBS_REGS}, (_, k) => "R" + k).join(",")}`);
    L.push("examine -h PC", "examine -h PSL");
    L.push("examine -h rq " + RQ_OBS.map((o) => o.name).join(","));
    L.push("echo DEVVEC", "examine rq devvec");
    L.push("echo SHOWRQ", "show rq");
    L.push("echo INTLOG", `examine -h ${hex(R_INTLOG)}:${hex(R_INTLOG + LOG_RECS * LOG_REC_BYTES - 4)}`);
    L.push("echo RESULT", `examine -h ${hex(R_RESULT)}:${hex(R_RESULT + PAGE - 4)}`);
    L.push("echo PAGES");
    for (let p of c.dumpPages) L.push(`examine -h ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)}`);
    L.push("echo ENDCASE");
    return L;
}

function runCasesSimh(simh, opts, cases)
{
    let L = [`set cpu ${MEM_MB}m`, "set cpu simhalt", "set rq rqdx3"];
    L.push(...handlerBankLines());
    for (let c of cases) L.push(...simhCaseLines(c));
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "mscpint-cases.ini"));
    if (process.env.MSCPINTDIFF_DUMP) fs.writeFileSync(process.env.MSCPINTDIFF_DUMP, out);

    let results = new Array(cases.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = cases.findIndex((c) => c.idx === +parts[i]);
        if (idx < 0) continue;
        results[idx] = parseChunk(parts[i + 1] || "", cases[idx]);
    }
    return results;
}

/** `examine -h A:B` prints `ADDR: VALUE`; a section is delimited by the echoes around it. */
function section(chunk, tag, next)
{
    let m = new RegExp(`^${tag}\\n([\\s\\S]*?)^(?:${next})$`, "m").exec(chunk);
    return m ? m[1] : null;
}

function memMap(text)
{
    let out = new Map(), re = /^([0-9A-F]{6,8}):\s*([0-9A-F]{8})\s*$/gm, m;
    while ((m = re.exec(text)) !== null) out.set(parseInt(m[1], 16) >>> 0, parseInt(m[2], 16) >>> 0);
    return out;
}

const SECTS = "DEVVEC|SHOWRQ|INTLOG|RESULT|PAGES|ENDCASE";

function parseChunk(chunk, c)
{
    let g = (name) => {
        let m = new RegExp(`^${name}:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
        return m ? parseInt(m[1], 16) >>> 0 : null;
    };
    let regs = [];
    for (let k = 0; k < OBS_REGS; k++) {
        let v = g("R" + k);
        if (v === null) return null;
        regs.push(v);
    }
    let r = {regs};
    for (let [k, n] of [["pc", "PC"], ["psl", "PSL"]]) {
        let v = g(n);
        if (v === null) return null;
        r[k] = v;
    }
    r.rq = {};
    for (let o of RQ_OBS) {
        let v = g(o.name);
        if (v === null) return null;
        r.rq[o.name] = v;
    }
    let dv = section(chunk, "DEVVEC", SECTS);
    let mdv = dv && /^DEVVEC:\s*([0-9A-Fa-f]+)/m.exec(dv);
    if (!mdv) return null;
    r.devvec = parseInt(mdv[1], 16) >>> 0;

    /* `RQ      address=20001468-2000146B, vector=3FC*, BR4, RQDX3, 4 units` */
    let sr = section(chunk, "SHOWRQ", SECTS);
    let msr = sr && /^RQ\s+address=[0-9A-Fa-f]+-[0-9A-Fa-f]+,\s*(.*?),\s*RQDX3,/m.exec(sr);
    if (!msr) return null;
    r.showVec = msr[1].trim();

    let li = section(chunk, "INTLOG", SECTS);
    if (li === null) return null;
    r.log = memMap(li);
    let re2 = section(chunk, "RESULT", SECTS);
    if (re2 === null) return null;
    r.result = memMap(re2);
    let pg = section(chunk, "PAGES", SECTS);
    if (pg === null) return null;
    r.mem = memMap(pg);

    r.halted = /HALT instruction/.test(chunk);
    r.atOwnHalt = r.halted && r.pc === c.haltPC;
    return r;
}

/* ------------------------------------------------------------------------------------------- *
 * The JS side                                                                                   *
 * ------------------------------------------------------------------------------------------- */

function runCaseJS(c, mutationOpts = {})
{
    let m = machine(mutationOpts);
    let {bus, cpu, cqbic, rq} = m;

    jsResetForCase(m, c);
    for (let s = SCB_LO; s <= SCB_HI; s += 4) {
        let a = hdlrAddr(s), b = handlerBytes(s);
        for (let i = 0; i < b.length; i++) bus.setByte((a + i) >>> 0, b[i]);
    }
    for (let a = R_SCBB; a <= R_SCBB + 0xFC; a += 4) bus.setLong(a >>> 0, 0);
    bus.setLong((R_SCBB + SCB.MCHK) >>> 0, R_MCHK_HDLR);
    bus.setLong((R_SCBB + SCB.MEMERR) >>> 0, R_MERR_HDLR);
    for (let s = SCB_LO; s <= SCB_HI; s += 4) bus.setLong((R_SCBB + s) >>> 0, hdlrAddr(s));
    for (let a = R_INTLOG; a < R_INTLOG + LOG_RECS * LOG_REC_BYTES; a += 4) bus.setLong(a >>> 0, 0);
    for (let a = R_RESULT; a < R_RESULT + PAGE; a += 4) bus.setLong(a >>> 0, 0);
    for (let q of c.zeroIdx) bus.setLong((CQMAP_BASE + q * 4) >>> 0, 0);
    for (let p of c.dumpPages) {
        let s = seedFor(p);
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) bus.setLong(a >>> 0, s);
    }
    for (let pr of c.presets) bus.setWord(pr.addr >>> 0, pr.word & 0xFFFF);
    for (let k = 0; k < c.code.length; k++) bus.setByte((R_CODE + k) >>> 0, c.code[k]);
    cpu.psl = 0;
    cpu.setPC(R_CODE);

    let halted = false, unimplemented = null;
    try {
        for (let s = 0; s < MAX_STEPS; s++) cpu.stepCPU(1);
    } catch (e) {
        if (e instanceof RQUnimplemented) unimplemented = e.message;
        else if (e.name === "VAXStop" && e.reason === "HALT instruction") halted = true;
        else unimplemented = `unexpected stop: ${e.reason || e.message || String(e)}`;
    }

    let r = {
        regs: Array.from({length: OBS_REGS}, (_, i) => cpu.regs[i] >>> 0),
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0,
        rq: {}, log: new Map(), result: new Map(), mem: new Map(), unimplemented,
        devvec: rq.vec >>> 0, showVec: showVecLine(rq.vec),
        halted, atOwnHalt: halted && (cpu.regs[15] >>> 0) === c.haltPC
    };
    for (let o of RQ_OBS) r.rq[o.name] = rqFieldOf(rq, o);
    for (let a = R_INTLOG; a < R_INTLOG + LOG_RECS * LOG_REC_BYTES; a += 4) r.log.set(a >>> 0, bus.getLong(a) >>> 0);
    for (let a = R_RESULT; a < R_RESULT + PAGE; a += 4) r.result.set(a >>> 0, bus.getLong(a) >>> 0);
    for (let p of c.dumpPages) {
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) r.mem.set(a >>> 0, bus.getLong(a) >>> 0);
    }
    /* The interrupt request bit the CPU is actually holding, and the controller's own flag -- the
       two halves of rq_setint()/rq_clrint().  Not comparable against SIMH (int_req[] has no named
       REG and `cp->irq` is not in rq_reg[]), so they are reported and fenced, never diffed. */
    r.irq = rq.irq ? 1 : 0;
    r.masterInt = (cpu.exc.intReq[0] >>> 0) & 1;
    sampleHeap();
    return r;
}

/* ------------------------------------------------------------------------------------------- *
 * Reading the log                                                                               *
 * ------------------------------------------------------------------------------------------- */

/**
 * logRecords(map, r)
 *
 * The recorded invocations, in order: {pc, psl, newpsl, scb}.  The COUNT comes from R13 rather than
 * from scanning for non-zero records, because "the handler wrote a record of all zeroes" and "the
 * handler never ran" must not be the same answer.
 */
function logRecords(map, r)
{
    let n = ((r.regs[LOG_REG] >>> 0) - R_INTLOG) / LOG_REC_BYTES;
    if (!Number.isInteger(n) || n < 0) return {n: -1, recs: []};
    let recs = [];
    for (let i = 0; i < Math.min(n, LOG_RECS); i++) {
        let b = R_INTLOG + i * LOG_REC_BYTES;
        recs.push({pc: map.get(b) >>> 0, psl: map.get(b + 4) >>> 0,
                   newpsl: map.get(b + 8) >>> 0, scb: map.get(b + 12) >>> 0});
    }
    return {n, recs};
}

const PSL_V_IPL = 16, PSL_M_IPL = 0x1F;
function iplOf(psl) { return (psl >>> PSL_V_IPL) & PSL_M_IPL; }

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

function grade(cases, sim, js, failures)
{
    let compared = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i], j = js[i];
        if (!s) {
            failures.push(`case ${c.idx} "${c.name}": the oracle produced no readable result -- not compared`);
            continue;
        }
        if (j.unimplemented) {
            failures.push(`case ${c.idx} "${c.name}": rq.js stopped: ${j.unimplemented}`);
            continue;
        }
        if (!s.atOwnHalt || !j.atOwnHalt) {
            failures.push(`case ${c.idx} "${c.name}": did not reach its own HALT ` +
                `(simh halted=${s.halted} PC=${hex(s.pc)}, js halted=${j.halted} PC=${hex(j.pc)}, ` +
                `expected ${hex(c.haltPC)}) -- an unbounded wait that never ended, or an interrupt ` +
                `that re-delivered forever`);
            continue;
        }
        compared++;
        for (let k = 0; k < OBS_REGS; k++) {
            if (s.regs[k] !== j.regs[k]) {
                failures.push(`case ${c.idx} "${c.name}": R${k} simh=${hex(s.regs[k])} js=${hex(j.regs[k])}` +
                    (k === LOG_REG ? "  (the interrupt-log pointer: the INVOCATION COUNT differs)" : ""));
            }
        }
        for (let [k, n] of [["pc", "PC"], ["psl", "PSL"], ["devvec", "rq devvec"]]) {
            if (s[k] !== j[k]) failures.push(`case ${c.idx} "${c.name}": ${n} simh=${hex(s[k])} js=${hex(j[k])}`);
        }
        if (s.showVec !== j.showVec) {
            failures.push(`case ${c.idx} "${c.name}": SHOW RQ vector line simh="${s.showVec}" js="${j.showVec}"`);
        }
        for (let o of RQ_OBS) {
            if (s.rq[o.name] !== j.rq[o.name]) {
                failures.push(`case ${c.idx} "${c.name}": rq ${o.name} simh=${hex(s.rq[o.name])} js=${hex(j.rq[o.name])}`);
            }
        }
        gradeLog(c, s, j, failures);
        for (let [what, sm, jm] of [["RESULT", s.result, j.result], ["page", s.mem, j.mem]]) {
            for (let [addr, v] of sm) {
                let w = jm.get(addr);
                if (w === undefined) { failures.push(`case ${c.idx} "${c.name}": ${what} 0x${hex(addr)} missing on the js side`); continue; }
                if (v !== w) failures.push(`case ${c.idx} "${c.name}": ${what} 0x${hex(addr)} simh=${hex(v)} js=${hex(w)}`);
            }
        }
        /* An expected-vector cross-check, from arithmetic written independently of both engines.
           DERIVED from the case's own instruction stream, not declared: a case whose tail writes IP
           has taken the controller through rq_reset(), whose `dibp->vec = 0` is one of the two facts
           those cases exist to observe. */
        let want = c.tail.some((t) => t.a === "wip") ? 0 : ((c.vec & SA_S1H_VEC) << 2);
        if (s.devvec !== want) {
            failures.push(`case ${c.idx} "${c.name}": the ORACLE's devvec is ${hex(s.devvec)} and ` +
                `rq_quesvc's own formula (s1dat & SA_S1H_VEC) << 2 gives ${hex(want)}`);
        }
        if (s.showVec !== showVecLine(want)) {
            failures.push(`case ${c.idx} "${c.name}": the ORACLE's SHOW RQ vector line is ` +
                `"${s.showVec}" and show_vec()'s own arithmetic gives "${showVecLine(want)}"`);
        }
    }
    return compared;
}

/**
 * gradeLog(c, s, j, failures)
 *
 * The recorded invocations: COUNT, ORDER, and PC/PSL/new-PSL/SCB slot at each -- compared record by
 * record, and then each record checked against arithmetic this file computed itself.  The
 * independent check matters: two engines agreeing that the dispatch went to SCB 0x1FC would compare
 * equal and both be wrong, which is precisely what shipping without int_vec_set would have done.
 */
function gradeLog(c, s, j, failures)
{
    let S = logRecords(s.log, s), J = logRecords(j.log, j);
    let tag = `case ${c.idx} "${c.name}"`;
    if (S.n < 0 || J.n < 0) {
        failures.push(`${tag}: the interrupt-log pointer is not a whole number of records ` +
            `(simh R13=${hex(s.regs[LOG_REG])} js R13=${hex(j.regs[LOG_REG])})`);
        return;
    }
    if (S.n > LOG_RECS) {
        failures.push(`${tag}: the ORACLE recorded ${S.n} invocations and only ${LOG_RECS} are dumped ` +
            `-- the log overflowed and the comparison would be blind past record ${LOG_RECS}`);
        return;
    }
    if (S.n !== J.n) {
        failures.push(`${tag}: ${S.n} interrupt(s) dispatched on the oracle, ${J.n} on rq.js`);
    }
    let wantSCB = deliveredSCB(c.vec);
    for (let i = 0; i < Math.min(S.recs.length, J.recs.length); i++) {
        let a = S.recs[i], b = J.recs[i];
        for (let f of ["pc", "psl", "newpsl", "scb"]) {
            if (a[f] !== b[f]) {
                failures.push(`${tag}: interrupt #${i} ${f} simh=${hex(a[f])} js=${hex(b[f])}`);
            }
        }
        /* Independent expectations, against the ORACLE's own record. */
        if (a.scb !== wantSCB) {
            failures.push(`${tag}: interrupt #${i} dispatched to SCB ${hex(a.scb, 3)} on the oracle; ` +
                `get_vector()'s own arithmetic for a programmed vector of ${hex(c.vec, 2)} gives ` +
                `${hex(wantSCB, 3)}`);
        }
        if (iplOf(a.newpsl) !== 0x17) {
            failures.push(`${tag}: interrupt #${i} ran at IPL ${hex(iplOf(a.newpsl), 2)} on the oracle; ` +
                `VEC_QBUS forces every Qbus dispatch to 0x17`);
        }
        if (iplOf(a.psl) !== 0) {
            failures.push(`${tag}: interrupt #${i} interrupted PSL ${hex(a.psl)}, IPL ` +
                `${hex(iplOf(a.psl), 2)} -- every case runs its host program at IPL 0`);
        }
        if (i > 0 && a.pc === S.recs[i - 1].pc && a.scb === S.recs[i - 1].scb) {
            /* Two dispatches from the same PC to the same slot: possible in principle, but in this
               case list it means a request was re-delivered rather than re-raised.  Reported, not
               failed -- coverage() is where the "not re-raised" floor lives. */
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * assertMaskIsNoop -- the tripwire under premise 5                                              *
 * ------------------------------------------------------------------------------------------- */

/**
 * assertMaskIsNoop(failures)
 *
 * The item this file closes asks for "at least one vector whose value IS folded by the CQBIC mask".
 * THERE IS NO SUCH VECTOR FOR THIS DEVICE, and that is a measurement rather than a concession: this
 * enumerates ALL 128 legal SA_S1H_VEC values and checks that get_vector()'s mask removes no bit
 * from any of them.  If the day ever comes that one is folded -- a wider SA_S1H_VEC, a different
 * int_vec_set, a machine whose QB_VEC_MASK is narrower -- this FAILS, and the case list has to grow
 * a case for it.  An unreachable coverage floor left as a comment is exactly the shape of gap
 * HANDOFF.md standing rule 6 is about; this is the same floor, armed.
 */
function assertMaskIsNoop(failures, report)
{
    let folded = [];
    for (let v = 0; v <= SA_S1H_VEC; v++) {
        let raw = ((v & SA_S1H_VEC) << 2) | VEC_SET;
        if ((raw & (VEC_SET | QB_VEC_MASK)) !== raw) folded.push(v);
    }
    if (folded.length) {
        failures.push(`assertMaskIsNoop: ${folded.length} legal SA_S1H_VEC value(s) ARE folded by ` +
            `get_vector()'s mask (first: 0x${hex(folded[0], 2)}) -- the case list has no case for ` +
            `that regime and must grow one`);
    }
    report.push(`  PHASE K  all ${SA_S1H_VEC + 1} legal SA_S1H_VEC values enumerated: ` +
        `get_vector()'s mask (VEC_SET 0x${hex(VEC_SET, 3)} | QB_VEC_MASK 0x${hex(QB_VEC_MASK, 3)} = ` +
        `0x${hex((VEC_SET | QB_VEC_MASK), 3)}) removes NO bit from any of them.  ` +
        `Largest legal vector 0x${hex(SA_S1H_VEC, 2)} -> devvec 0x${hex(SA_S1H_VEC << 2, 3)} -> ` +
        `SCB 0x${hex(deliveredSCB(SA_S1H_VEC), 3)}.  The mask IS graded where it bites: ` +
        `tests/hwintdiff.js's dynamic_tmr0_truncated`);
}

/* ------------------------------------------------------------------------------------------- *
 * Exclusion fences.  Every one FAILS the run; none scales with case count.                      *
 * ------------------------------------------------------------------------------------------- */

function assertExclusions(cases, sim, failures)
{
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i];
        for (let pr of c.presets) { void pr; }
        /* SET CONTROLLER CHARACTERISTICS and nothing else -- see the file header. */
        for (let cm of c.cmds) {
            let w = sccWords();
            if (w[CMD_OPC] !== OP.SCC) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" sends MSCP opcode ${w[CMD_OPC]}; ` +
                    `only OP_SCC is in scope here`);
            }
            void cm;
        }
        if ((c.s1dat & SA_S1H_VEC) !== (c.vec & SA_S1H_VEC)) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" has a spec vector of ` +
                `0x${hex(c.vec, 2)} and an S1 word carrying 0x${hex(c.s1dat & SA_S1H_VEC, 2)}`);
        }
        if (!s) continue;
        /* Every dispatch must land inside the handler bank; one outside it is unattributable and
           the log would be silently missing it. */
        let S = logRecords(s.log, s);
        for (let r of S.recs) {
            if (r.scb < SCB_LO || r.scb > SCB_HI) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" dispatched to SCB ` +
                    `0x${hex(r.scb, 3)}, outside the 0x${hex(SCB_LO, 3)}..0x${hex(SCB_HI, 3)} handler bank`);
            }
        }
        /* Same rule mscpinitdiff and mscpringdiff use: a case long enough for rq_tmrsvc()'s
           WALL-CLOCK host-access timer to fire is a case whose comparison is not reproducible. */
        if (s.rq.HAT !== 0 && s.rq.HAT !== s.rq.HTMO) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" left the ORACLE's HAT at ` +
                `${s.rq.HAT} (HTMO ${s.rq.HTMO}) -- rq_tmrsvc() fired and is not modelled`);
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors.  Every one FAILS the run; none scales with case count.                      *
 * ------------------------------------------------------------------------------------------- */

function coverage(cases, sim, js, failures, acc, seam)
{
    /* Everything below is computed from the ORACLE's results, so a defect on the JS side can never
       be what satisfies a floor. */
    let ok = (i) => sim[i] && sim[i].atOwnHalt;
    let nOf = (i) => logRecords(sim[i].log, sim[i]).n;
    let recsOf = (i) => logRecords(sim[i].log, sim[i]).recs;

    /* ---- 1. AN INIT INTERRUPT AT EACH OF STEPS 2, 3 AND 4.  Sized from mscpscope's derived count
       of rq_init_int() call sites, not from a number written here. ---- */
    let wantInit = seam.initCallers.length;
    let byStop = new Map();
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        if (!ok(i) || c.cmds.length || c.stopAfter === undefined || c.noStopFloor) continue;
        byStop.set(c.stopAfter, nOf(i));
    }
    acc.initSteps = [...byStop.entries()].sort((a, b) => a[0] - b[0]);
    for (let k = 1; k <= wantInit; k++) {
        let stopAt = 1 + 2 * k;                         /* walkScript action index after step k's poll */
        if (!byStop.has(stopAt)) {
            failures.push(`coverage: no case stops after handshake step ${k}, so the init interrupt ` +
                `that transition raises is not separately attributed`);
        } else if (byStop.get(stopAt) !== k) {
            failures.push(`coverage: the case stopping after handshake step ${k} recorded ` +
                `${byStop.get(stopAt)} interrupt(s) on the ORACLE, not ${k} -- the per-step ` +
                `attribution this floor rests on does not hold`);
        }
    }
    if (byStop.size < wantInit) {
        failures.push(`coverage: pdp11_rq.c has ${wantInit} rq_init_int() call site(s) and only ` +
            `${byStop.size} handshake stop point(s) are graded`);
    }

    /* ---- 2. BOTH rq_putdesc() RING-INTERRUPT ARMS.  With a ONE-SLOT ring the `ring->lnt <= 4` arm
       fires unconditionally; with a longer ring the ONLY way to get a ring interrupt is the
       previous-descriptor rule, so ring length is a sound attribution. ---- */
    let armShort = 0, armPrev = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        if (!ok(i) || !c.cmds.length) continue;
        /* Init interrupts are the ones raised before the rings exist; anything beyond the init
           count for this s1dat is a ring interrupt. */
        let initExpected = (c.s1dat & SA_S1H_IE) && (c.s1dat & SA_S1H_VEC) ? wantInit : 0;
        let ring = nOf(i) - initExpected;
        if (ring <= 0) continue;
        if (c.g.rqSlots === 1 && c.g.cqSlots === 1) armShort += ring;
        else armPrev += ring;
    }
    acc.armShort = armShort; acc.armPrev = armPrev;
    if (seam.ringCallers.length !== 2) {
        failures.push(`coverage: pdp11_rq.c now has ${seam.ringCallers.length} rq_ring_int() call ` +
            `site(s); this file's two-arm attribution assumes exactly 2`);
    }
    if (!armShort) failures.push(`coverage: no ring interrupt observed from the \`ring->lnt <= 4\` arm`);
    if (!armPrev) failures.push(`coverage: no ring interrupt observed from the previous-descriptor arm`);

    /* ---- 3. THE ASYMMETRY.  A case with SA_S1H_IE CLEAR and SA_S1H_VEC SET must show ZERO
       interrupts across the whole handshake and then at least one from a ring transition.  This is
       the floor no single-quadrant test can satisfy. ---- */
    let asym = 0, asymNames = [];
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        if (!ok(i) || (c.s1dat & SA_S1H_IE) || !(c.s1dat & SA_S1H_VEC) || !c.cmds.length) continue;
        let recs = recsOf(i);
        if (!recs.length) continue;
        asym++; asymNames.push(c.name);
    }
    acc.asym = asym;
    if (!asym) {
        failures.push(`coverage: no case with SA_S1H_IE CLEAR and SA_S1H_VEC SET produced an ` +
            `interrupt on the ORACLE.  That case is the ONLY thing that separates rq_ring_int()'s ` +
            `one-bit condition from rq_init_int()'s two-bit one`);
    }
    /* ... and the same S1 word must have produced NO interrupt during the handshake, which is what
       makes it an asymmetry rather than just "an interrupt happened". */
    let asymInitClean = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        if (!ok(i) || (c.s1dat & SA_S1H_IE) || !(c.s1dat & SA_S1H_VEC) || c.cmds.length) continue;
        if (nOf(i) === 0) asymInitClean++;
        else failures.push(`coverage: case ${c.idx} "${c.name}" has SA_S1H_IE clear and recorded ` +
            `${nOf(i)} interrupt(s) during a handshake-only run -- rq_init_int() tests IE`);
    }
    acc.asymInitClean = asymInitClean;
    if (!asymInitClean) {
        failures.push(`coverage: no handshake-only case with SA_S1H_IE clear and SA_S1H_VEC set is ` +
            `graded, so "the handshake raises nothing without IE" is untested`);
    }

    /* ---- 4. IE SET, VECTOR ZERO -> nothing, ever. ---- */
    let vecZero = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        if (!ok(i) || !(c.s1dat & SA_S1H_IE) || (c.s1dat & SA_S1H_VEC)) continue;
        if (nOf(i) !== 0) {
            failures.push(`coverage: case ${c.idx} "${c.name}" has SA_S1H_IE set and a ZERO vector ` +
                `and still dispatched ${nOf(i)} interrupt(s) on the ORACLE`);
        } else vecZero++;
    }
    acc.vecZero = vecZero;
    if (!vecZero) failures.push(`coverage: no case programs SA_S1H_IE with a ZERO vector`);

    /* ---- 5. THE LARGEST LEGAL VECTOR, DISPATCHED.  Premise 5's shape: only the top of the range
       can tell "the mask is a no-op here" from "the mask is not applied". ---- */
    let maxSeen = false, slots = new Set(), starred = 0, unstarred = 0;
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        let c = cases[i], recs = recsOf(i);
        if (!recs.length) continue;
        for (let r of recs) slots.add(r.scb);
        if ((c.vec & SA_S1H_VEC) === SA_S1H_VEC) maxSeen = true;
        if (/\*/.test(sim[i].showVec)) starred++; else unstarred++;
    }
    acc.slots = slots.size; acc.starred = starred; acc.unstarred = unstarred;
    if (!maxSeen) {
        failures.push(`coverage: the LARGEST legal SA_S1H_VEC (0x${hex(SA_S1H_VEC, 2)}) never ` +
            `dispatched -- see assertMaskIsNoop()'s docblock for why that value specifically`);
    }
    if (slots.size < 4) {
        failures.push(`coverage: only ${slots.size} distinct SCB slot(s) were dispatched to; a ` +
            `constant vector would satisfy anything less than 4`);
    }
    if (!starred || !unstarred) {
        failures.push(`coverage: every dispatching case is on the same side of SIMH's own ` +
            `\`show rq\` vector threshold (${starred} starred, ${unstarred} not) -- the VEC_SET ` +
            `fold is not exercised across the boundary the ORACLE renders differently`);
    }

    /* ---- 6. THE ACKNOWLEDGE CLEARS THE REQUEST AND IT IS NOT RE-RAISED.  Every graded case ends
       with the controller quiet: a request still pending at HALT with nothing to raise it would
       mean the acknowledge did not clear.  Checked on the JS side (the oracle publishes neither
       `cp->irq` nor int_req[]) and reported, plus the stronger structural check that no case
       recorded more invocations than its s1dat can explain. ---- */
    let quiet = 0;
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        if (js[i].irq === 0 && js[i].masterInt === 0) quiet++;
        else {
            failures.push(`coverage: case ${cases[i].idx} "${cases[i].name}" ended with the ` +
                `controller's request still asserted (irq=${js[i].irq} master=${js[i].masterInt}) ` +
                `-- the acknowledge did not clear it, or something re-raised it after the last one`);
        }
    }
    acc.quiet = quiet;

    /* ---- 6b. WHAT A RESET WITHDRAWS.  Both halves, from the ORACLE: the vector goes back to zero
       and a request raised while masked never dispatches. ---- */
    let resetVec = 0, resetInt = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        if (!ok(i) || !c.noStopFloor) continue;
        if (sim[i].devvec === 0 && sim[i].showVec.startsWith("no vector")) resetVec++;
        if (nOf(i) === 0) resetInt++;
        else failures.push(`coverage: case ${c.idx} "${c.name}" dispatched ${nOf(i)} interrupt(s) ` +
            `on the ORACLE across a controller reset -- rq_reset() calls rq_clrint()`);
    }
    acc.resetVec = resetVec; acc.resetInt = resetInt;
    if (!resetVec) failures.push(`coverage: no case observes rq_reset()'s \`dibp->vec = 0\``);
    if (resetInt < 2) failures.push(`coverage: fewer than two cases observe rq_reset()'s rq_clrint()`);

    /* ---- 7. ORDER.  At least one case must record more than one invocation, or "the ORDER of the
       recorded invocations" is a property nothing in the run has. ---- */
    let multi = 0, maxN = 0;
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        let n = nOf(i);
        if (n > maxN) maxN = n;
        if (n > 1) multi++;
    }
    acc.multi = multi; acc.maxN = maxN;
    if (multi < 2) {
        failures.push(`coverage: only ${multi} case(s) recorded more than one invocation, so the ` +
            `ORDER of the log is graded by almost nothing`);
    }

    /* ---- 8. EVERY DISPATCH RAN AT IPL 0x17 -- reported as a SET, so "some ran at 0x14" is visible
       rather than averaged away.  gradeLog() already fails each one individually. ---- */
    let ipls = new Set();
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let r of recsOf(i)) ipls.add(iplOf(r.newpsl));
    }
    acc.ipls = [...ipls];
    if (ipls.size !== 1 || !ipls.has(0x17)) {
        failures.push(`coverage: the ORACLE dispatched at IPL(s) {${[...ipls].map((v) => hex(v, 2)).join(", ")}}; ` +
            `VEC_QBUS forces every Qbus interrupt to 0x17`);
    }

    /* ---- 9. Every case must have reached comparison (standing rule 6). ---- */
    let unreached = cases.filter((c, i) => !ok(i)).map((c) => `${c.idx} "${c.name}"`);
    if (unreached.length) {
        failures.push(`coverage: ${unreached.length} case(s) did not reach comparison: ${unreached.join("; ")}`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11).        *
 *                                                                                              *
 * FOUR expectations per mutation, and a mutation that is CAUGHT but fails any of them is reported *
 * as a MISATTRIBUTION and fails the run exactly like a survivor:                                 *
 *                                                                                               *
 *   mustCatch    a regexp over CASE NAMES: some failure must come from a case matching it.       *
 *   mustNotName  a substring that must NOT appear in the FIRST failure.                          *
 *   require      a regexp that must appear in at least ONE failure -- the CHANNEL that must have *
 *                noticed (an `interrupt #N scb` difference, a `devvec` difference, ...).         *
 *   forbid       a regexp that must appear in NO failure -- the channel that must NOT move.      *
 *                                                                                               *
 * `require`/`forbid` are what make the pairs below sharp rather than merely loud.  Stripping     *
 * VEC_QBUS and stripping the 0x200 SCB-page bit are DUALS: the first must change only the        *
 * dispatched IPL and not the slot, the second only the slot and not the IPL, and widening        *
 * SA_S1H_VEC must change `examine rq devvec` while leaving the interrupt log BYTE-IDENTICAL.     *
 * Without those, all three would report a cheerful CAUGHT off whichever difference happened to   *
 * come first -- HANDOFF.md standing rule 16, which has now landed three times in this device,    *
 * most recently with ALL TWENTY of a sibling's mutations being "caught" by a leaked flag.        *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* 1. The vector taken as (s1dat & SA_S1H_VEC) WITHOUT the << 2.  Composed over the shipped
          quesvc(): let it run, then put back the wrong value if the CST_S1 arm just ran. */
    "vector taken without the <<2": {
        mustCatch: /^vector /, mustNotName: "R13",
        apply: () => {
            let orig = RQVAX.prototype.quesvc;
            RQVAX.prototype.quesvc = function() {
                let was = this.csta;
                orig.call(this);
                if (was === CST_S1 && this.csta === CST_S2) this.vec = this.s1dat & SA_S1H_VEC;
            };
            return () => { RQVAX.prototype.quesvc = orig; };
        }
    },
    /* 2. int_vec_set NOT applied -- i.e. exactly what deviceVector() did before pcjsvax-aef, a
          plain `vec & QB_VEC_MASK`.  Composed over the shipped call, not a re-implementation. */
    "int_vec_set (VEC_SET) not applied to the resolved vector": {
        mustCatch: /^(vector|quadrant) /, mustNotName: "HAT",
        /* Drops BOTH bits, so BOTH channels must move. */
        require: /interrupt #\d+ (scb|newpsl)/, forbid: /rq devvec/,
        apply: () => {
            let cls = Object.getPrototypeOf(machine().cpu.exc).constructor;
            let orig = cls.prototype.deviceVector;
            cls.prototype.deviceVector = function(cpu, lvl) { return orig.call(this, cpu, lvl) & QB_VEC_MASK; };
            return () => { cls.prototype.deviceVector = orig; };
        }
    },
    /* 3. VEC_QBUS (bit 0) stripped from the delivered vector.  The SCB read masks &~3, so the SLOT
          is untouched and ONLY the dispatched IPL changes -- which makes this the mutation that
          proves the recorded new-PSL is load-bearing rather than decoration. */
    "VEC_QBUS stripped -- right slot, wrong IPL": {
        mustCatch: /^(vector|quadrant|ring|two|init|comm|acknowledge|random)/, mustNotName: "devvec",
        require: /interrupt #\d+ newpsl/, forbid: /interrupt #\d+ scb|rq devvec/,
        apply: () => {
            let cls = Object.getPrototypeOf(machine().cpu.exc).constructor;
            let orig = cls.prototype.deviceVector;
            cls.prototype.deviceVector = function(cpu, lvl) { return orig.call(this, cpu, lvl) & ~1; };
            return () => { cls.prototype.deviceVector = orig; };
        }
    },
    /* 4. The SCB PAGE bit (0x200) stripped instead -- right IPL, wrong slot.  The complement of
          mutation 3, so neither can be "caught" by the other's signal. */
    "the 0x200 SCB-page bit stripped -- right IPL, wrong slot": {
        mustCatch: /^(vector|quadrant|ring|two|init|comm|acknowledge|random)/, mustNotName: "devvec",
        require: /interrupt #\d+ scb/, forbid: /interrupt #\d+ newpsl|rq devvec/,
        apply: () => {
            let cls = Object.getPrototypeOf(machine().cpu.exc).constructor;
            let orig = cls.prototype.deviceVector;
            cls.prototype.deviceVector = function(cpu, lvl) { return orig.call(this, cpu, lvl) & ~0x200; };
            return () => { cls.prototype.deviceVector = orig; };
        }
    },
    /* 5. rq_ring_int() raised on SA_S1H_IE instead of SA_S1H_VEC -- one half of the asymmetry. */
    "rq_ring_int raised on IE instead of VEC": {
        mustCatch: /IE CLEAR|quadrant noie/, mustNotName: "PHASE",
        /* rq_ring_int() cannot run in a handshake-only case, so none of those may be what catches it. */
        forbid: /"(init stop after|init full walk|init via purge|vector [0-9A-F]{2} ->|reset )/,
        apply: () => {
            let orig = RQVAX.prototype.ringInt;
            RQVAX.prototype.ringInt = function(ring) {
                let ie = this.s1dat & SA_S1H_IE, vec = this.s1dat & SA_S1H_VEC;
                /* Run the shipped code with an s1dat whose VEC field mirrors the IE bit, so the
                   FLAG-WORD DMA (the other half of ring_int) is still the shipped path. */
                this.s1dat = (this.s1dat & ~SA_S1H_VEC) | (ie ? 1 : 0);
                orig.call(this, ring);
                this.s1dat = (this.s1dat & ~SA_S1H_VEC) | vec;
            };
            return () => { RQVAX.prototype.ringInt = orig; };
        }
    },
    /* 6. rq_init_int() raised on SA_S1H_VEC ALONE -- the OTHER half.  Neither mutation 5 nor 6 can
          be caught by a case that sets both bits or clears both. */
    "rq_init_int raised on VEC alone (IE test dropped)": {
        mustCatch: /noie\+vecmax|IE CLEAR|random/, mustNotName: "PHASE",
        /* A no-op wherever IE is already set, so no IE-set case may be what catches it. */
        forbid: /"(quadrant ie\+|init stop|init full|init via|vector [0-9A-F]{2} ->)/,
        apply: () => {
            let orig = RQVAX.prototype.initInt;
            RQVAX.prototype.initInt = function() {
                let ie = this.s1dat & SA_S1H_IE;
                this.s1dat = this.s1dat | SA_S1H_IE;    /* pretend IE was always set */
                orig.call(this);
                if (!ie) this.s1dat = this.s1dat & ~SA_S1H_IE;
            };
            return () => { RQVAX.prototype.initInt = orig; };
        }
    },
    /* 7. The acknowledge does not clear the request -- it re-delivers forever, which shows up as a
          case that never reaches its own HALT. */
    "acknowledge does not clear the request": {
        mustCatch: /^(vector|quadrant|ring|two|init|comm|acknowledge|random)/, mustNotName: "PHASE",
        apply: () => {
            let orig = RQVAX.prototype.inta;
            RQVAX.prototype.inta = function() {
                let v = 0;
                for (let ncp of this.peers) if (ncp.irq) { v = ncp.vec | 0; break; }
                if (v) { if (this.exc) this.exc.raiseInterrupt(0x14, 0); return v; }
                return orig.call(this);
            };
            return () => { RQVAX.prototype.inta = orig; };
        }
    },
    /* 8. rq_setint() sets the controller's own flag but not the MASTER request bit -- the split
          rq_setint()'s two lines exist to prevent.  Nothing ever dispatches. */
    "rq_setint sets cp->irq but not the master request": {
        mustCatch: /^(vector|quadrant|ring|two|init|comm|acknowledge|random)/, mustNotName: "PHASE",
        apply: () => {
            let orig = RQVAX.prototype.setInt;
            RQVAX.prototype.setInt = function() { this.irq = 1; void orig; };
            return () => { RQVAX.prototype.setInt = orig; };
        }
    },
    /* 9. rq_reset() leaves the MASTER request asserted (`cp->irq = 0` without rq_clrint()'s
          re-scan).  Only observable while a request is PENDING AND UNDELIVERED, which is what
          "reset withdraws an interrupt raised while masked" exists for -- and the defect delivers an
          interrupt to SCB 0x200, because get_vector() ORs VEC_SET into a rq_inta() that answers 0. */
    "rq_reset leaves the master request asserted": {
        mustCatch: /reset withdraws/, mustNotName: "PHASE",
        forbid: /rq devvec/,
        apply: () => {
            let orig = RQVAX.prototype.clrInt;
            let inReset = false;
            let origReset = RQVAX.prototype.reset;
            RQVAX.prototype.reset = function() { inReset = true; try { origReset.call(this); } finally { inReset = false; } };
            RQVAX.prototype.clrInt = function() { if (inReset) { this.irq = 0; return; } return orig.call(this); };
            return () => { RQVAX.prototype.clrInt = orig; RQVAX.prototype.reset = origReset; };
        }
    },
    /* 10. rq_reset() leaves `dibp->vec` alone.  Caught by `examine rq devvec` and by the SHOW RQ
           vector line going back to "no vector" -- and by NOTHING ELSE, since no interrupt is
           delivered in that case at all. */
    "rq_reset does not zero the vector": {
        mustCatch: /reset clears the programmed vector/, mustNotName: "R13",
        require: /rq devvec/, forbid: /interrupt #/,
        apply: () => {
            let orig = RQVAX.prototype.reset;
            RQVAX.prototype.reset = function() { let v = this.vec; orig.call(this); this.vec = v; };
            return () => { RQVAX.prototype.reset = orig; };
        }
    },
    /* 11. SA_S1H_VEC widened by one bit, so the IE bit lands in the vector.
           *** THIS IS THE MUTATION THAT PROVES devvec IS GRADED SEPARATELY FROM THE DISPATCH. ***
           The wrong vector is `(0x80 | V) << 2` = `0x200 | (V << 2)`, and get_vector() ORs 0x200 in
           anyway -- so the DELIVERED SCB SLOT AND THE DISPATCHED IPL ARE BOTH UNCHANGED and the
           interrupt log is byte-identical.  It is caught only by `examine rq devvec` and by the
           SHOW RQ vector line, which is exactly why this file reads both. */
    "SA_S1H_VEC widened to include the IE bit": {
        mustCatch: /^(vector|quadrant ie|init|ring|two|comm|acknowledge|random)/, mustNotName: "interrupt #",
        /* The whole point: the interrupt log is BYTE-IDENTICAL and only devvec moves. */
        require: /rq devvec|SHOW RQ vector line/, forbid: /interrupt #|R13 /,
        apply: () => {
            let orig = RQVAX.prototype.quesvc;
            RQVAX.prototype.quesvc = function() {
                let was = this.csta;
                orig.call(this);
                if (was === CST_S1 && this.csta === CST_S2 && this.vec) this.vec = (this.s1dat & 0xFF) << 2;
            };
            return () => { RQVAX.prototype.quesvc = orig; };
        }
    }
};

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                        *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [], acc = {};

    assertMaskIsNoop(failures, report);

    let cases = enumeratedCases();
    let all = cases.concat(randomCases(opts.nRandom, opts.seed, cases.length));
    let sim = runCasesSimh(simh, opts, all);
    /* Each pass writes its own do-file and starts its OWN simulator, so the oracle's static MSC
       struct starts every pass at its C-global zero.  The JS machine is built once and reused
       (standing rule 14), so the pass boundary is where that has to be re-established -- and the
       CPU's interrupt request bit is part of it: a pass that ended mid-dispatch would otherwise
       leave int_req[] set and the NEXT pass's first case would dispatch an interrupt nothing
       raised.  This is HANDOFF.md standing rule 16's leaked-io_complete, one level up. */
    let m = machine(mutationOpts);
    m.rq.powerUp();
    m.cpu.exc.reset();
    let js = all.map((c) => runCaseJS(c, mutationOpts));

    assertExclusions(all, sim, failures);
    let compared = grade(all, sim, js, failures);
    coverage(all, sim, js, failures, acc, opts.seam);

    /* The wiring the graded machine is actually holding, asserted rather than assumed. */
    if (!mutationOpts.noQbusHook && m.cpu.qbus !== m.rq) {
        failures.push(`the graded machine's CPU has no Qbus event hook wired to the controller`);
    }
    if (m.rq.exc !== m.cpu.exc) {
        failures.push(`the graded machine's controller is not wired to the CPU's interrupt seam, so ` +
            `every "no interrupt" result here would be vacuous`);
    }
    if (m.cpu.exc.intVecSet[0][0] !== VEC_SET) {
        failures.push(`the graded machine's interrupt source at IPL 0x14 bit 0 carries int_vec_set ` +
            `0x${hex(m.cpu.exc.intVecSet[0][0] | 0, 3)}, not VEC_SET 0x${hex(VEC_SET, 3)}`);
    }

    report.push(`  CASES    ${compared}/${all.length} case(s) compared ` +
        `(${cases.length} enumerated + ${opts.nRandom} randomized)`);
    report.push(`  FLOORS   init steps ${JSON.stringify(acc.initSteps || [])}; ring interrupts ` +
        `${acc.armShort || 0} from the lnt<=4 arm and ${acc.armPrev || 0} from the ` +
        `previous-descriptor arm; ${acc.asym || 0} IE-clear/VEC-set ring case(s) + ` +
        `${acc.asymInitClean || 0} handshake-only; ${acc.vecZero || 0} IE-set/VEC-zero case(s); ` +
        `${acc.slots || 0} distinct SCB slot(s); show-rq star ${acc.starred || 0}/${acc.unstarred || 0}; ` +
        `${acc.multi || 0} multi-dispatch case(s), max ${acc.maxN || 0}; ` +
        `reset withdraws vector/request ${acc.resetVec || 0}/${acc.resetInt || 0}; ` +
        `dispatch IPL(s) {${(acc.ipls || []).map((v) => hex(v, 2)).join(",")}}`);
    return {failures, report, compared};
}

function selfcheck(simh, opts)
{
    let bad = [];
    for (let name of Object.keys(MUTATIONS)) {
        let mu = MUTATIONS[name];
        let restore = mu.apply();
        let failures;
        try {
            failures = runPass(simh, opts).failures;
        } finally {
            restore();
        }
        if (!failures.length) {
            bad.push(`${name}: SURVIVED`);
            console.log(`  SURVIVED  ${name}`);
            continue;
        }
        /* ATTRIBUTION, not a body count (HANDOFF.md standing rule 16). */
        let first = failures[0];
        let named = failures.find((f) => {
            let m = /^(?:case \d+ "|coverage: case \d+ ")(.*?)"/.exec(f);
            return m && mu.mustCatch.test(m[1]);
        });
        let attributed = named || failures.find((f) => mu.mustCatch.test(f));
        if (!attributed) {
            bad.push(`${name}: MISATTRIBUTED (no failure from a case matching ${mu.mustCatch})`);
            console.log(`  MISATTRIB ${name}  (first: ${first.split("\n")[0].slice(0, 130)})`);
            continue;
        }
        if (mu.mustNotName && first.includes(mu.mustNotName)) {
            bad.push(`${name}: caught by "${mu.mustNotName}", which it must not be`);
            console.log(`  MISATTRIB ${name}  (caught by ${mu.mustNotName})`);
            continue;
        }
        if (mu.require && !failures.some((f) => mu.require.test(f))) {
            bad.push(`${name}: no failure on the channel it must move (${mu.require})`);
            console.log(`  MISATTRIB ${name}  (nothing matched ${mu.require})`);
            continue;
        }
        if (mu.forbid) {
            let bogus = failures.find((f) => mu.forbid.test(f));
            if (bogus) {
                bad.push(`${name}: moved a channel it must not (${mu.forbid}): ${bogus.slice(0, 120)}`);
                console.log(`  MISATTRIB ${name}  (moved ${mu.forbid})`);
                continue;
            }
        }
        console.log(`  CAUGHT    ${name}  (${failures.length} failure(s), first: ` +
            `${first.split("\n")[0].slice(0, 110)})`);
    }
    return bad;
}

function main()
{
    let simh = findSimhBin(getArg("--simh", null));
    let nRandom = +getArg("--cases", RANDOM_CASES_DEFAULT);
    let seed = +getArg("--seed", 20260728);
    let fSelfcheck = process.argv.includes("--selfcheck");

    if (nRandom < RANDOM_CASES_FLOOR) {
        console.error(`mscpintdiff: --cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "mscpintdiff-"));
    let code = 0;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized cases: ${nRandom}`);
        console.log(`RQ: I/O page 0x${hex(RQ_BASE)}..+${IOLN_RQ}, IPL 0x14 bit 0 (BR4), ` +
            `int_vec_set VEC_SET 0x${hex(VEC_SET, 3)}, QB_VEC_MASK 0x${hex(QB_VEC_MASK, 3)}`);
        console.log(`SCB handler bank: ${SCB_SLOTS} slot(s) 0x${hex(SCB_LO, 3)}..0x${hex(SCB_HI, 3)} ` +
            `at 0x${hex(R_HDLR)}; interrupt log at 0x${hex(R_INTLOG)}, ${LOG_RECS} record(s) dumped`);

        let opts = {scratch, nRandom, seed};
        /* The derived premises this file's floors are sized from -- see mscpscope's intSeam(). */
        opts.seam = scopeExtract(simh).intSeam;
        console.log(`derived: rq_init_int {${opts.seam.initGuard.bits.join("+")}} from ` +
            `[${opts.seam.initCallers.join(", ")}]; rq_ring_int ` +
            `{${opts.seam.ringGuard.bits.join("+")}} from [${opts.seam.ringCallers.join(", ")}]`);

        let {failures, report} = runPass(simh, opts);
        console.log(`\nPHASES`);
        for (let line of report) console.log(line);

        let peak = peakHeap(), peakMB = peak / (1024 * 1024);
        console.log(`\npeak JS heap+external: ${peakMB.toFixed(1)} MB (absolute ceiling ` +
            `${MAX_HEAP_BYTES / (1024 * 1024)} MB)`);
        if (peak > MAX_HEAP_BYTES) {
            failures.push(`peak heap+external ${peakMB.toFixed(1)} MB exceeds the absolute ceiling ` +
                `${MAX_HEAP_BYTES / (1024 * 1024)} MB`);
        }

        if (failures.length) {
            console.error(`\nFAIL -- ${failures.length} difference(s) from the oracle (seed ${seed}):`);
            for (let f of failures.slice(0, 40)) console.error(`  ${f}`);
            if (failures.length > 40) console.error(`  ... and ${failures.length - 40} more`);
            code = 1;
        } else {
            console.log(`\nMATCH -- every graded interrupt agrees with the oracle: the COUNT, the ` +
                `ORDER, and the interrupted PC/PSL, the dispatched PSL (IPL 0x17) and the SCB slot ` +
                `at each; \`examine rq devvec\`; the SHOW RQ vector line including SIMH's own \`*\`; ` +
                `all ${RQ_OBS.length} examinable controller registers; and every byte of the ` +
                `interrupt log, the RESULT page and every physical page the comm region was ` +
                `scattered across.`);
        }

        if (fSelfcheck && !code) {
            console.log(`\nPHASE M -- mutations (${Object.keys(MUTATIONS).length}), each ATTRIBUTED`);
            let bad = selfcheck(simh, opts);
            if (bad.length) {
                console.error(`\nFAIL -- ${bad.length} mutation(s) not properly caught:`);
                for (let b of bad) console.error(`  ${b}`);
                code = 1;
            } else {
                console.log(`\nall ${Object.keys(MUTATIONS).length} mutation(s) CAUGHT by the case ` +
                    `written to kill them`);
            }
        }
    } finally {
        fs.rmSync(scratch, {recursive: true, force: true});
    }
    process.exit(code);
}

if (process.argv[1] && process.argv[1].endsWith("mscpintdiff.js")) main();
