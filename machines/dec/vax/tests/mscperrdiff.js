/**
 * @fileoverview Differential test: every REJECTED or FAILING MSCP transfer's status code, its
 *               residual, its error-log packets and the port errors that kill the controller,
 *               graded against a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-3c3, the fifth of pcjsvax-6a5's children.  Its four predecessors built a controller that
 * comes up (c2c), rings and a packet pool (0b4), units and drive types (f52) and five data transfers
 * that move real blocks (346).  This one grades what happens when a command DOES NOT WORK: the exact
 * MSCP status word the controller answers with, the exact residual it leaves in the host's own
 * packet, the error-log datagrams it does or does not emit, and the seven PORT ERRORS that take the
 * controller out of service entirely.
 *
 * *** THE ONE THING THIS FILE EXISTS FOR: rq_rw_valid()'s LADDER, AND ITS ORDER. ***
 * -----------------------------------------------------------------------------------
 * rq_rw_valid() (pdp11_rq.c:2312-2349) is an eleven-rung ladder that returns the FIRST matching
 * status.  So a READ that is BOTH odd-count AND off the end of the disk answers ST_HST|SB_HST_OC and
 * NOT ST_CMD|I_BCNT, and a WRITE into the replacement table on a hardware-locked drive answers
 * ST_CMD|I_LBN and NOT ST_WPR|SB_WPR_HW.  *** TESTING EACH REJECTION WITH A PACKET THAT TRIPS ONLY
 * THAT ONE CONDITION NEVER OBSERVES THE ORDER AT ALL *** -- every rung would pass with the ladder
 * shuffled into any order whatsoever.  This file therefore posts commands that trip TWO OR MORE
 * rungs at once, computes the tripped SET independently (trippedRungs() below, a second writing of
 * the C's predicates), and asserts that THE ORACLE ANSWERED THE FIRST OF THEM.  A coverage floor
 * requires at least THREE such commands and it FAILS the run rather than scaling.
 *
 * And the LENGTH of that ladder is not written here.  tests/mscpscope.js slices rq_rw_valid() out of
 * pdp11_rq.c on every run, strips its comments -- pdp11_rq.c:2330 carries a COMMENTED-OUT twelfth
 * rung, and reading it as live would demand a case for a branch that cannot be reached -- walks its
 * `return` statements in source order and evaluates each against the ST_/SB_/I_ tables it also
 * derives.  The coverage floor below is `scope.nLadder`, straight out of that.  HANDOFF.md standing
 * rule 5: this project's CIS opcode count went 7 -> 11 -> 17 -> 23 and every hand-derived value was
 * wrong.
 *
 * SEVEN PHASES
 * ------------
 *   PHASE S  the scope, re-derived by tests/mscpscope.js: 21 OP_, 23 ST_, 15 SB_, 7 I_, 13 PE_, the
 *            16 dispatch cases, the 34 drv_tab[] rows AND rq_rw_valid()'s twelve branches IN ORDER.
 *   PHASE L  the ladder: one graded command per derived branch, plus the multi-rung commands that
 *            are the only thing that observes the ORDER.
 *   PHASE E  the exits from rq_svc() that are not the ladder: a zero byte count (ST_SUC and no disk
 *            operation at all), a COMPARE mismatch (ST_CMP with `bc - i`), and a host-buffer NXM
 *            part way through the DMA on the READ path and on the WRITE path -- each grading
 *            RW_WBCL/RW_WBAL and each run BOTH with and without CF_THS, because rq_hbe()'s log
 *            packet is the ONLY difference the two produce.
 *   PHASE F  the fatal ladder: PE_PRE, PE_PWE, PE_QRE, PE_QWE, PE_ICI, PE_PIE and PE_PPF, each
 *            leaving csta = CST_DEAD, perr = the code and SA = SA_ER|code.
 *   PHASE P  rq_plf(): a host that re-initialises a DEAD controller and sets SA_S4H_LF in its GO
 *            word gets the LAST-FAILURE datagram -- and gets it with error logging DISABLED,
 *            because rq_plf() has no CF_THS test.  Graded against the two negatives as well.
 *   PHASE A  ABORT and GET COMMAND STATUS against a transfer IN FLIGHT -- the arms pcjsvax-346 made
 *            reachable, named as out of scope and left throwing.  They are implemented here.
 *   PHASE M  --selfcheck only: the mutations, including the one that PERMUTES the ladder.
 *
 * WHAT IS DELIBERATELY NOT GRADED, BY NAME (HANDOFF.md standing rule 6)
 * ---------------------------------------------------------------------
 *   - PE_HAT (9), the HOST ACCESS TIMEOUT.  rq_tmrsvc() is scheduled with
 *     `sim_activate_after (uptr, 1000000)` -- WALL-CLOCK microseconds, and the ONE nondeterministic
 *     scheduler in this device.  Every case here is asserted to complete far below that horizon
 *     (assertNoHostTimeout() reads HAT off BOTH engines and requires it still at HTMO, i.e. that no
 *     tick of that timer has been taken), and a case that did not would FAIL the run rather than
 *     compare two engines whose timers had drifted apart.
 *   - PE_MRE (22), the MAP REGISTER READ error.  That is pcjsvax-5c1's excluded branch --
 *     `cqmap_rd` with MBR pointing outside memory -- and tests/qdmadiff.js carries a LIVE TRIPWIRE
 *     on it.  Constructing it here would be constructing the case another differential exists to
 *     refuse.  *** The host-buffer NXM cases below are built the OTHER way: by programming a CQBIC
 *     map entry with its VALID BIT CLEAR in the middle of the target range, which is e22's
 *     already-graded mapAddr() failure path. ***
 *   - PE_T11 (475), PE_SND (476), PE_RCV (477).  "NI" in pdp11_uqssp.h; rq_fatal() is never called
 *     with any of them anywhere in pdp11_rq.c, which portErrorCensus() re-derives FROM THE C on
 *     every run rather than taking on trust.
 *   - PE_NSR (478), the free packet list running dry.  Graded by tests/mscpringdiff.js, which owns
 *     the packet pool; reported here as covered elsewhere rather than duplicated.
 *   - rq_dte(), the DISK TRANSFER ERROR log and its ST_DRV end packet.  It hangs off
 *     sim_disk_rdsect()/wrsect() returning non-SCPE_OK, and on the STD-format container a
 *     user-supplied file attaches as there is no such return: a read past the end of the file is
 *     zero-filled and successful (sim_disk.c's `_sim_disk_rdsect`), and a write to a container that
 *     cannot be written is refused two rungs EARLIER by rq_rw_valid()'s hardware write lock, because
 *     sim_disk_attach_ex2() sets UNIT_RO on a container it could only open read-only.  There is
 *     therefore NO do-file that makes the oracle take that arm, rq.js throws by name, and
 *     assertExclusions() FAILS the run if a graded case reaches it.  This was re-measured for this
 *     item and not inherited from pcjsvax-346's note.
 *   - CONTROLLER INTERRUPT DELIVERY (pcjsvax-aef); every case supplies SA_S1H_VEC == 0.
 *
 *      node machines/dec/vax/tests/mscperrdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch build)
 *        --cases N         randomized ladder probes (default RANDOM_CASES_DEFAULT; below the fixed
 *                           floor the run FAILS rather than clamping up)
 *        --seed S          PRNG seed, printed on every run so a failure is reproducible
 *        --selfcheck       prove the differential detects deliberate defects
 *        --dump N          both engines' DEBUG=REQ streams for case N, side by side
 */

import fs from "fs";
import os from "os";
import path from "path";

import RQVAX, {
    RQUnimplemented, CST_DEAD, CST_UP,
    SA_ER, SA_S1H_VL, SA_S1H_IE, SA_S1H_VEC, SA_S1H_V_CQ, SA_S1H_V_RQ,
    SA_S2H_CLO, SA_S2H_PI, SA_S3H_PP, SA_S3H_CHI, SA_S4H_GO, SA_S4H_LF,
    UQ_DESC_OWN, UQ_DESC_F,
    UQ_HLNT, UQ_HCTC, UQ_HCTC_V_CR, UQ_HCTC_V_TYP, UQ_HCTC_V_CID,
    UQ_TYP_SEQ, UQ_TYP_DAT, UQ_CID_MSCP, UQ_CID_DIAG,
    OP, ST, SB, I, PE, RW_VALID_LADDER,
    CMD_OPC, CMD_MOD, CMD_UN, CMD_REFL, CMD_REFH,
    RSP_LNT, RSP_OPF, RSP_STS, SCC_LNT, SCC_CFL, SCC_MSV, SCC_TMO,
    ABO_LNT, ABO_REFL, ABO_REFH, GCS_LNT, GCS_REFL, GCS_REFH, GCS_STSL, GCS_STSH,
    RW_BCL, RW_BCH, RW_BAL, RW_BAH, RW_MAPL, RW_MAPH, RW_LBNL, RW_LBNH,
    RW_WBCL, RW_WBAL, RW_LNT_D,
    PLF_LNT, PLF_ERR, ELP_REFL, FM_CNT, FM_BAD, LF_SNR, EF_LOG, CF_THS,
    HBE_LNT, HBE_BADL, HBE_BADH,
    UF_WPS, MD_SWP, ONL_UFL,
    DRV_TAB, RD54_DTYPE, U_ATT, U_ONL,
    RQ_NUMBY, RQ_NUMDR, RQ_MAXFR, RQ_MAPXFER,
    RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME
} from "../modules/v2/rq.js";
import CQBICVAX from "../modules/v2/cqbic.js";
import {
    PAGE, R_CODE, R_RESULT, MAP_MBR, OBS_REGS,
    RQ_IP, RQ_SA, CQBIC_BASE, CQMAP_BASE, CQMAP_VLD, MEM_MB,
    R_IS, hex, findSimhBin, runSimh, mulberry32, sampleHeap, peakHeap,
    Asm, machine, RQ_OBS, rqFieldOf, PKT_WORDS, pktWord,
    showCtrl, emitAction, simhResetLines, jsResetForCase, fileImageProvider
} from "./mscpharness.js";
import { checkScope } from "./mscpscope.js";

/** An absolute bound on the instructions any case may execute.  Generous: every in-band wait in
    this file is UNBOUNDED (see NO_WAIT_BUDGET), so a case that reaches this bound is a case whose
    controller never answered, and it is reported BY NAME as a machine that never reached its own
    HALT rather than compared as if it had. */
const MAX_STEPS = 3000000;

/** The host's scratch registers.  R0..R8 belong to the init handshake. */
const REGS = {prev: 9, cur: 10, cnt: 11, lim: 12, tmp: 13};

/** *** THERE IS NO WAIT BUDGET IN THIS FILE, AND ITS ABSENCE IS DELIBERATE (HANDOFF.md standing
    rule 17). ***  pcjsvax-346 shipped green with a bounded in-band wait that expired about one run
    in five; the expired budget let the host HALT mid-command and the two engines were then compared
    at different points in one command's life.  Raising the bound made it rarer and could not make
    it impossible.  Every wait here watches a value the controller is REQUIRED to change:
      - a response descriptor, for a command rq_mscp() must answer (and rq_rw() answers even the
        refusals -- that is this file's subject);
      - SA, for a command that takes the controller FATAL, because rq_fatal() writes SA_ER|code into
        a register that was 0.
    Both are Asm-level unbounded loops.  The one shape that legitimately never answers -- a WRITE
    whose buffer fetch fails ENTIRELY, which leaves the unit unscheduled forever -- is fenced out by
    assertExclusions() from the CASE LIST, so no wait here can be waiting on nothing. */
const NO_WAIT_BUDGET = true;

const RANDOM_CASES_DEFAULT = 24;
const RANDOM_CASES_FLOOR   = 12;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure and NOT scaled by case
    count (HANDOFF.md rules 4 and 14).  ONE machine is built and reused across every case and every
    mutation pass; the containers are reached through an `fs`-backed provider and never read into
    memory. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;

/** SIMH's flat 16-bit view of the packet array.  A refusal builds its response IN PLACE over the
    command, so the words the handler does not touch are the evidence the whole 64 bytes round
    tripped -- and for a REFUSED transfer that includes the EIGHT WORKING WORDS, which rq_rw() never
    wrote and which must therefore still carry the host's own seed.  Packets 0..6 plus 31 covers the
    command packets, the error-log packets rq_hbe()/rq_plf() take off the free list, and the far end
    of the pool. */
const PKT_PROBES = (function() {
    let out = [];
    for (let p of [0, 1, 2, 3, 4, 5, 6, 31]) for (let w = 0; w < PKT_WORDS; w++) out.push(p * PKT_WORDS + w);
    return out;
})();

/** rq_reg[]'s URDATA entries -- the per-unit registers, read with `examine -h`. */
const UNIT_OBS = [
    {name: "CAPAC", get: (u) => u.capac >>> 0},
    {name: "UFLG",  get: (u) => u.uf & 0xFFFF},
    {name: "PLUG",  get: (u) => u.plug >>> 0},
    {name: "CPKT",  get: (u) => u.cpkt & 0x1F},
    {name: "PKTQ",  get: (u) => u.pktq & 0x1F}
];

/* ------------------------------------------------------------------------------------------- *
 * PHYSICAL LAYOUT -- this file's own, above everything mscpharness uses                         *
 * ------------------------------------------------------------------------------------------- */

const ERR_DATA_BASE  = 0x00400000;
const ERR_DATA_NPAGE = 512;

/** The physical page a Qbus page is scattered to: DESCENDING and STRIDED with a large odd stride,
    so the entries a case programs are discontiguous AND out of order.  397 is odd (injective modulo
    512 requires only that; 397 and 512 are coprime) and large enough that consecutive Qbus pages
    jump across the window and back, which an identity or constant-offset map cannot reproduce.
    tests/mscprwdiff.js owns the argument that the map is consulted at all and re-derives the
    discontiguity from its own case's entries; this file inherits a WORKING map and is about what
    happens when one entry of it is INVALID. */
function physFor(qpage, spread)
{
    let i = ((qpage * 397) + spread * 13) % ERR_DATA_NPAGE;
    return ((ERR_DATA_BASE + (ERR_DATA_NPAGE - 1 - i) * PAGE) / PAGE) | 0;
}

/** A non-zero, PAGE-DISTINCT seed.  Two jobs: a transfer that landed on the wrong page shows up as
    the wrong seed rather than as plausible data, and -- which matters more here -- a REFUSED
    transfer must leave EVERY page still carrying its seed, so "nothing happened" is a positive
    observation rather than the absence of one. */
function seedFor(ppage) { return ((0x5A5A0000 | ((ppage * 0x0101) & 0xFFFF)) >>> 0); }

/** The 128 bytes BELOW the interrupt stack pointer -- where a machine check's exception frame lands.
    Zeroed and dumped on both engines in every case: without it a machine check inside a graded case
    shows up only as "the PC differs". */
const IS_WIN_LO = (R_IS - 128) >>> 0;

/* ------------------------------------------------------------------------------------------- *
 * The scratch containers                                                                        *
 * ------------------------------------------------------------------------------------------- */

/**
 * *** THE CAPACITIES ARE THE POINT, BECAUSE HALF THIS FILE'S LADDER IS ABOUT THE END OF THE DISK. ***
 *
 *   err   1,024 blocks, attached READ ONLY on an RD54.  `-R` skips autosize's `container < current`
 *         clamp, so the UNIT's capacity is the CONTAINER's 1,024 blocks exactly -- which puts the
 *         RCT window at LBN 1,024 and its far edge at 1,024 + RD54's rcts (7).  Attached WRITABLE
 *         the same file would give a 311,200-block unit and the RCT window would be unreachable.
 *   wr    360 blocks on an RX18, the SMALLEST drive in drv_tab[], attached WRITABLE.  360 is
 *         exactly RX18's own lbn count, so autosize's clamp is a no-op and the unit is its
 *         container; RX18's rcts is 0, which makes `maxlbn + rcts == maxlbn` and collapses the two
 *         RCT rungs onto one LBN -- a shape the read-only unit cannot produce.
 *   wr2   the same, a second copy, so the software-lock and hardware-lock rungs can be reached on
 *         two different units in one case.
 *
 * Each engine gets its OWN copy of the writable ones.  No graded case in this file performs a WRITE
 * that is ACCEPTED, so both copies must come back byte-identical to the pristine one -- which is
 * itself an assertion (a refused WRITE that touched the disk is a refusal that did not refuse).
 */
const IMAGES = [
    {tag: "err", blocks: 1024, writable: false},
    {tag: "wr",  blocks: 360,  writable: true},
    {tag: "wr2", blocks: 360,  writable: true},
    /* *** THE ONE CONTAINER A GRADED CASE IS ALLOWED TO CHANGE. ***  The WRITE whose host buffer
       takes a bus error part way through is ACCEPTED by rq_rw_valid() and its top end really does
       put the bytes it managed to fetch on the disk before the bottom end discovers the error -- so
       "wr3" is expected to differ from its pristine copy, and expected to be IDENTICAL between the
       two engines.  Keeping it separate is what lets every OTHER container be asserted untouched. */
    {tag: "wr3", blocks: 360,  writable: true, mayChange: true}
];

function blockBytes(tag, lbn, dst)
{
    let s = (lbn * 2654435761 + tag.charCodeAt(0) * 40503 + tag.length) >>> 0;
    for (let i = 0; i < RQ_NUMBY; i++) {
        s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
        dst[i] = (s >>> 24) & 0xFF;
    }
    return dst;
}

function makeImages(dir)
{
    let out = [], blk = new Uint8Array(RQ_NUMBY);
    for (let spec of IMAGES) {
        let p = path.join(dir, `mscperr-${spec.tag}.dsk`);
        let fd = fs.openSync(p, "w+");
        for (let lbn = 0; lbn < spec.blocks; lbn++) {
            fs.writeSync(fd, blockBytes(spec.tag, lbn, blk), 0, RQ_NUMBY, lbn * RQ_NUMBY);
        }
        fs.closeSync(fd);
        let simhPath = p, jsPath = p;
        if (spec.writable) {
            simhPath = path.join(dir, `mscperr-${spec.tag}.simh.dsk`);
            jsPath = path.join(dir, `mscperr-${spec.tag}.js.dsk`);
            fs.copyFileSync(p, simhPath);
            fs.copyFileSync(p, jsPath);
        }
        out.push({tag: spec.tag, blocks: spec.blocks, bytes: spec.blocks * RQ_NUMBY,
                  path: jsPath, simhPath, pristine: p, writable: spec.writable,
                  mayChange: !!spec.mayChange});
        sampleHeap();
    }
    return out;
}

/** *** NO CONTAINER IN THIS FILE MAY CHANGE, INCLUDING THE WRITABLE ONES. ***  Every WRITE a graded
    case posts is one rq_rw_valid() or rq_svc() must REFUSE, so a container that came back different
    is a refusal that reached the disk first -- which is a defect this file would otherwise report
    only as a status-word difference, and only if the status happened to differ too. */
function checkContainersUntouched(images, failures, acc)
{
    let firstDiff = (pa, pb) => {
        let a = fs.readFileSync(pa), b = fs.readFileSync(pb);
        if (a.length !== b.length) return {off: -1, len: [a.length, b.length]};
        let off = 0;
        while (off < a.length && a[off] === b[off]) off++;
        return off === a.length ? null : {off, len: [a.length, b.length]};
    };
    for (let im of images) {
        if (im.mayChange) {
            /* *** PHASE I. ***  This container IS written by a graded case, so the assertion is a
               different one and it is the only view that can grade a WRITE at all: the bytes leave
               host memory and never come back, so every memory comparison in this file is blind to
               them.  Two statements, both of which fail the run: the two engines' copies must be
               byte-identical, and at least one of them must DIFFER FROM THE PRISTINE COPY -- a
               comparison of two files neither engine wrote is not a pass. */
            let d = firstDiff(im.simhPath, im.path);
            if (d) {
                failures.push(`PHASE I: container "${im.tag}" differs between the engines -- first ` +
                    `difference at byte ${d.off} (block ${(d.off / RQ_NUMBY) | 0}), sizes ` +
                    `${d.len.join(" vs ")}.  The accepted WRITE whose host buffer faulted put ` +
                    `different bytes on the two disks.`);
            } else {
                acc.imagesMatched++;
            }
            if (!firstDiff(im.pristine, im.path)) {
                failures.push(`PHASE I: container "${im.tag}" came back IDENTICAL TO THE PRISTINE ` +
                    `COPY, so no graded WRITE reached the disk at all and this phase compared two ` +
                    `untouched files.  That is a coverage hole, not a pass.`);
            } else {
                acc.imagesChanged++;
            }
            continue;
        }
        for (let [what, p] of [["this engine's", im.path], ["the oracle's", im.simhPath]]) {
            let d = firstDiff(im.pristine, p);
            if (d) {
                failures.push(`container "${im.tag}" -- ${what} copy -- CHANGED during the run ` +
                    `(first difference at byte ${d.off}, block ${(d.off / RQ_NUMBY) | 0}).  Every ` +
                    `WRITE and ERASE a graded case posts against this container is one the ` +
                    `controller must REFUSE, so a disk that moved means a refusal wrote first.`);
            }
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Geometry                                                                                      *
 * ------------------------------------------------------------------------------------------- */

/**
 * errGeometry(spec)
 *
 * *** THE RESPONSE RING COMES FIRST ***: rq_step4() does `rq.ba = comm; cq.ba = comm + rq.lnt`.
 *
 * A SECOND writing of mscpharness's geometry() plus a data region, on purpose (the discipline
 * tests/qdmadiff.js applies to its own page list): the host program addresses PHYSICAL memory and
 * the controller addresses QBUS memory through the map, and a disagreement between the two shows up
 * as a memory difference rather than as a silent pass.
 *
 * THE FOUR AREAS ARE ON SEPARATE QBUS PAGES, and here that is load bearing rather than tidy: a page
 * is the unit the map validates, so "the command packets' page is not mapped" is only a statement
 * about command packets if nothing else lives there -- and PE_PRE, PE_PWE and PE_QRE are exactly
 * those three statements about three different pages.
 */
function errGeometry(spec)
{
    let rqLnt = 4 << spec.rqCode, cqLnt = 4 << spec.cqCode;
    let rqBa = spec.comm >>> 0, cqBa = (spec.comm + rqLnt) >>> 0;
    let pgUp = (a) => (a + PAGE - 1) & ~(PAGE - 1);
    let cmdBase = pgUp(spec.comm + rqLnt + cqLnt);
    let rspBase = cmdBase + pgUp(spec.nCmdBuf * 64);
    let dataBase = rspBase + pgUp(spec.nRspBuf * 64);
    let dataLnt = pgUp(spec.dataLnt || PAGE);
    return {
        rqLnt, cqLnt, rqBa, cqBa, cmdBase, rspBase, dataBase, dataLnt,
        rqSlots: rqLnt >> 2, cqSlots: cqLnt >> 2,
        cmdBuf: (i) => (cmdBase + i * 64) >>> 0,
        cmdEnv: (i) => (cmdBase + i * 64 + 4) >>> 0,
        rspBuf: (j) => (rspBase + j * 64) >>> 0,
        rspEnv: (j) => (rspBase + j * 64 + 4) >>> 0,
        cmdPage: (i) => (((cmdBase + i * 64) / PAGE) | 0),
        rspPage: (j) => (((rspBase + j * 64) / PAGE) | 0),
        ringPage: ((spec.comm / PAGE) | 0),
        /* The interrupt flag words sit BELOW comm: SA_COMM_CI (-4) and SA_COMM_RI (-2). */
        lo: (spec.comm - 8) >>> 0,
        hi: (dataBase + dataLnt - 1) >>> 0
    };
}

function qbusPagesFor(g)
{
    let pages = [];
    for (let a = g.lo & ~(PAGE - 1); a <= g.hi; a += PAGE) pages.push((a / PAGE) | 0);
    return pages;
}

/* ------------------------------------------------------------------------------------------- *
 * The command packet a host plants in memory                                                    *
 * ------------------------------------------------------------------------------------------- */

/**
 * cmdWords(o)
 *
 * The words of an MSCP command packet a case sets, as {index: value}.  EVERY WORD IT DOES NOT SET IS
 * LEFT AS THE PAGE SEED, and for this file that is the strongest single observation there is: a
 * REFUSED transfer must leave RW_WBAL..RW_WMPH exactly as the host wrote them, because rq_rw() only
 * copies the host's fields into the working words on the arm where `sts == 0`.  A response whose
 * working words are ZERO is a transfer that was accepted and then ended; one that still carries the
 * seed is a transfer that was refused before it started.  Nothing about the status word distinguishes
 * those two, and rq_rw_end() zeroes the working words on every accepted path.
 */
function cmdWords(o)
{
    let w = {};
    w[UQ_HLNT] = (o.hlnt === undefined) ? RSP_LNT : o.hlnt;
    w[UQ_HCTC] = (((o.cr || 0) << UQ_HCTC_V_CR) |
                  ((o.typ === undefined ? UQ_TYP_SEQ : o.typ) << UQ_HCTC_V_TYP) |
                  ((o.cid === undefined ? UQ_CID_MSCP : o.cid) << UQ_HCTC_V_CID)) & 0xFFFF;
    w[CMD_REFL] = (o.ref === undefined ? 0xBEEF : o.ref) & 0xFFFF;
    w[CMD_REFH] = (o.refh === undefined ? 0x1234 : o.refh) & 0xFFFF;
    w[CMD_UN] = (o.unit || 0) & 0xFFFF;
    w[CMD_OPC] = (o.opc === undefined ? OP.RD : o.opc) & 0xFFFF;
    w[CMD_MOD] = (o.mod || 0) & 0xFFFF;
    if (o.ufl !== undefined) w[ONL_UFL] = o.ufl & 0xFFFF;
    if (o.cfl !== undefined) { w[SCC_CFL] = o.cfl & 0xFFFF; w[SCC_MSV] = 0; w[SCC_TMO] = 0; }
    /* ABORT and GET COMMAND STATUS carry the reference of the command they are about in a DIFFERENT
       word from the one that identifies themselves: ABO_REFL/GCS_REFL are word 8, CMD_REFL is
       word 2.  Both are set from `tref`, and getting them confused is invisible until a unit
       actually holds a packet -- which is PHASE A's whole subject. */
    if (o.tref !== undefined) {
        w[ABO_REFL] = o.tref & 0xFFFF;
        w[ABO_REFH] = (o.trefh === undefined ? 0 : o.trefh) & 0xFFFF;
    }
    if (o.bc !== undefined) { w[RW_BCL] = o.bc & 0xFFFF; w[RW_BCH] = (o.bc >>> 16) & 0xFFFF; }
    if (o.ba !== undefined) { w[RW_BAL] = o.ba & 0xFFFF; w[RW_BAH] = (o.ba >>> 16) & 0xFFFF; }
    if (o.map !== undefined) { w[RW_MAPL] = o.map & 0xFFFF; w[RW_MAPH] = (o.map >>> 16) & 0xFFFF; }
    if (o.lbn !== undefined) { w[RW_LBNL] = o.lbn & 0xFFFF; w[RW_LBNH] = (o.lbn >>> 16) & 0xFFFF; }
    return w;
}

/* ------------------------------------------------------------------------------------------- *
 * rq_rw_valid()'s predicates, WRITTEN A SECOND TIME -- and that is the point                     *
 * ------------------------------------------------------------------------------------------- */

/**
 * trippedRungs(cmd, st, d)
 *
 * WHICH rungs of rq_rw_valid()'s ladder this command trips, as a sorted list of indices into
 * RW_VALID_LADDER -- NOT which one wins.
 *
 * *** THIS IS A SECOND, INDEPENDENT WRITING OF THE C's CONDITIONS, AND IT EXISTS SO THE ORDER CAN
 * BE ASSERTED RATHER THAN ASSUMED. ***  The differential does not ask rq.js which rung fired; it
 * computes the SET of rungs whose condition holds, and requires THE ORACLE to have answered
 * `RW_VALID_LADDER[min(set)].sts`.  A command that trips one rung says nothing about the ladder's
 * order.  A command that trips three says everything.
 *
 * `st` is the unit's state AT THE MOMENT THIS COMMAND RUNS -- {att, onl, wps, wph} -- declared by
 * the case, which knows because it just issued the ONLINE or the SET UNIT CHARACTERISTICS that
 * produced it.  A declaration that were WRONG would predict the wrong rung and FAIL the run, so it
 * is self-checking rather than trusted.  `d` is the drive-table row and `st.capac` the unit's
 * capacity, both of which decide where the replacement table begins.
 *
 * THE NESTING IS THE C's, and it is the part a re-derivation gets wrong: rungs 5 and 6 live INSIDE
 * `if (lbn >= maxlbn)` and rung 7 is that `if`'s ELSE, so no packet can ever trip 7 together with 5
 * or 6.  Rungs 8, 9 and 10 are inside `if ((cmd == OP_WR) || (cmd == OP_ERS))` and are unreachable
 * for a READ however locked the drive is.
 *
 * @param {Object} cmd  the command spec {opc, bc, ba, lbn}
 * @param {Object} st   {att, onl, wps, wph, capac}
 * @param {Object} d    DRV_TAB row
 * @returns {Array.<number>}
 */
function trippedRungs(cmd, st, d)
{
    let out = [];
    let bc = (cmd.bc === undefined ? 0 : cmd.bc) >>> 0;
    let ba = (cmd.ba === undefined ? 0 : cmd.ba) >>> 0;
    let lbn = (cmd.lbn === undefined ? 0 : cmd.lbn) >>> 0;
    let maxlbn = st.capac >>> 0;
    let opc = cmd.opc & 0xFF;

    if (!st.att) out.push(0);                                       /* not attached */
    if (!st.onl) out.push(1);                                       /* not online */
    if ((opc !== OP.ACC) && (opc !== OP.ERS) && (ba & 1)) out.push(2);      /* odd address */
    if (bc & 1) out.push(3);                                        /* odd byte count */
    if (bc & 0xF0000000) out.push(4);                               /* 'reasonable' bc */
    if (lbn >= maxlbn) {                                            /* accessing RCT? */
        if (lbn >= ((maxlbn + d.rcts) >>> 0)) out.push(5);          /* beyond copy 1 */
        if (bc !== RQ_NUMBY) out.push(6);                           /* bc must be 512 */
    } else if (((lbn + Math.floor((bc + (RQ_NUMBY - 1)) / RQ_NUMBY)) >>> 0) > maxlbn) {
        out.push(7);                                                /* spiral to RCT */
    }
    if ((opc === OP.WR) || (opc === OP.ERS)) {                      /* write op? */
        if (lbn >= maxlbn) out.push(8);                             /* accessing RCT */
        if (st.wps) out.push(9);                                    /* swre wlk */
        if (st.wph) out.push(10);                                   /* hwre wlk */
    }
    if (!out.length) out.push(RW_VALID_LADDER.length - 1);          /* the accept arm */
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

const RING_CODE_MAX = 7;
const PLUG_PARK = 900;

function unitSpec(o = {})
{
    return {
        dtype: (o.dtype === undefined) ? RD54_DTYPE : o.dtype,
        plug: o.plug,
        locked: !!o.locked,
        ro: !!o.ro,
        image: o.image === undefined ? null : o.image,
        disabled: !!o.disabled
    };
}

const IMG = {};                                             /* filled by main(), tag -> descriptor */

/** The capacity a container produces on a drive type, DERIVED from sim_disk's autosize arithmetic
    rather than written down -- and cross-checked against the oracle's own CAPAC register by grade().
    A read-only attach skips the clamp, which is what puts the RCT window in reach. */
function capacFor(image, dtype, ro)
{
    let container = image.bytes, current = DRV_TAB[dtype].lbn * RQ_NUMBY;
    if (container < current && !ro) container = current;
    return Math.floor(container / RQ_NUMBY);
}

function buildCase(spec)
{
    let c = Object.assign({
        itime: RQ_ITIME, i4time: RQ_ITIME4, qtime: RQ_QTIME, xtime: RQ_XTIME,
        cqCode: 1, rqCode: 1, comm: 0x2000, prgi: 0, spread: 0,
        nCmdBuf: 2, nRspBuf: 2, dataLnt: PAGE,
        noMap: false, unmappedQ: [], unmappedData: [], invalidData: [], invalidQ: [],
        packets: {}, steps: [], units: null, cmdSpecs: []
    }, spec);

    if (c.cqCode > RING_CODE_MAX || c.rqCode > RING_CODE_MAX) {
        throw new Error(`mscperrdiff: case "${c.name}" ring code out of range`);
    }
    c.units = (c.units || []).slice();
    while (c.units.length < RQ_NUMDR) c.units.push(unitSpec({}));
    for (let i = 0; i < RQ_NUMDR; i++) if (c.units[i].plug === undefined) c.units[i].plug = i;
    let seen = new Set();
    for (let u of c.units) {
        if (seen.has(u.plug)) throw new Error(`mscperrdiff: case "${c.name}" reuses plug ${u.plug}`);
        seen.add(u.plug);
        if (u.plug >= PLUG_PARK) throw new Error(`mscperrdiff: plug ${u.plug} collides with the park range`);
        if (u.plug >= RQ_NUMDR && u.plug < 254) {
            throw new Error(`mscperrdiff: case "${c.name}" wants plug ${u.plug}, which one of the ` +
                `DISABLED potential drives already holds -- the simulator would refuse the \`set\``);
        }
    }

    c.s1dat = (SA_S1H_VL | (c.cqCode << SA_S1H_V_CQ) | (c.rqCode << SA_S1H_V_RQ)) & 0xFFFF;
    c.g = errGeometry(c);

    let qpages = qbusPagesFor(c.g);
    let unmapped = new Set(c.unmappedQ);
    for (let k of c.unmappedData) unmapped.add(((c.g.dataBase / PAGE) | 0) + k);
    /* *** TWO KINDS OF "NOT MAPPED", AND THE DIFFERENCE IS THE MEASUREMENT. ***
         `unmappedQ`/`unmappedData` name pages whose map entry is NEVER WRITTEN -- the map's backing
           store is zeroed by the per-case reset, so the entry reads back as a longword of zeros.
         `invalidData` names pages whose entry IS WRITTEN, with a real physical page number and the
           VALID BIT DELIBERATELY CLEAR.
       A controller that tested "entry != 0" instead of "entry & CQMAP_VLD" behaves identically on
       the first and wrongly on the second, and the item this file belongs to asks specifically for
       the second: an INVALID entry programmed in the middle of the target range, through
       pcjsvax-e22's already-graded mapAddr() failure path.  Both are used. */
    let invalid = new Set(c.invalidQ || []);
    for (let k of c.invalidData) invalid.add(((c.g.dataBase / PAGE) | 0) + k);
    c.entries = [];
    if (!c.noMap) {
        for (let q of qpages) {
            if (unmapped.has(q)) continue;
            let p = physFor(q, c.spread);
            c.entries.push({q, p, valid: !invalid.has(q)});
        }
    }
    /* *** `phys()` RESOLVES FOR AN INVALID ENTRY TOO, AND THAT IS NOT AN OVERSIGHT. ***  The host
       program addresses PHYSICAL memory with ordinary MOVL instructions and never goes through the
       CQBIC map at all -- only the CONTROLLER's DMA does.  So a page whose map entry has its VALID
       BIT CLEAR is still a page the host can seed, deposit a command packet into and read a
       descriptor back out of; what it is not is a page the controller can reach.  That asymmetry is
       exactly what makes PE_PRE and PE_PWE constructible: the packet really is where the descriptor
       says it is, and the controller still cannot fetch it.  A page with NO entry at all is a
       different thing and still throws below. */
    c.qToP = new Map(c.entries.map((e) => [e.q, e.p]));
    c.validQ = new Set(c.entries.filter((e) => e.valid).map((e) => e.q));
    /* Every physical page the map names, VALID OR NOT -- an invalid entry's page is dumped too,
       because a controller that ignored the valid bit would write there and nothing else would see
       it. */
    c.dumpPages = [...new Set(c.entries.map((e) => e.p))].sort((a, b) => a - b);
    c.resultPage = (R_RESULT / PAGE) | 0;

    c.phys = (qaddr) => {
        let q = (qaddr / PAGE) | 0;
        if (!c.qToP.has(q)) {
            throw new Error(`mscperrdiff: case "${c.name}" addresses Qbus 0x${hex(qaddr, 6)}, whose ` +
                `page ${q} has NO map entry at all, so this file has no physical address for it`);
        }
        return (c.qToP.get(q) * PAGE + (qaddr % PAGE)) >>> 0;
    };

    c.presets = [];
    for (let i of Object.keys(c.packets)) {
        let w = c.packets[i];
        for (let k of Object.keys(w)) {
            c.presets.push({addr: c.phys(c.g.cmdBuf(+i) + (+k) * 2), word: w[k] & 0xFFFF});
        }
    }

    /* RESULT-page slot assignment, done in the SAME walk the emitter uses. */
    c.slots = [];
    let off = 0;
    let take = (n, what, band) => {
        let o = off;
        for (let k = 0; k < n; k++) c.slots.push({off: o + k * 4, what, band: !!(band && k === 0)});
        off += n * 4;
        return o;
    };
    for (let st of c.steps) {
        if (st.s === "await") {
            /* *** IS THIS AWAIT WAITING ON A DISK OPERATION? ***  A wait that covers one is NOT
               reproducible on the oracle (rq_io_complete()'s re-schedule shares its queue with a
               wall-clock-calibrated timer; tests/mscprwdiff.js measured 129 / 153 / 221 iterations
               for the same single-block READ on three consecutive runs).  A wait that covers only a
               REFUSAL is: rq_rw() answers inside the same queue service and no disk is touched.
               The flag is set from the COMMAND, conservatively -- any transfer opcode -- because
               whether a given transfer was refused is exactly what this file is measuring and a
               band that depended on the answer would be grading the answer with itself. */
            let k = (st.cmd === undefined) ? st.slot : st.cmd;
            let cs = c.cmdSpecs[k];
            let nm = cs ? RQVAX.OP_NAME_OF[cs.opc & 0xFF] : undefined;
            st.xfer = !!(nm && RQVAX.MSCP_XFER_OPS.indexOf(nm) >= 0);
            st.roff = take(2, `${st.what || "await"} (iterations / value)`, st.xfer);
        } else if (st.s === "awaitsa") {
            st.roff = take(2, `${st.what || "awaitsa"} (iterations / SA)`, false);
        } else if (st.s === "snap" || st.s === "snapsa") {
            st.roff = take(1, st.what || st.s);
        }
    }
    if (off > PAGE) throw new Error(`mscperrdiff: case "${c.name}" needs ${off} RESULT bytes`);

    let a = new Asm();
    if (!c.noMap) {
        a.movImmAbs(4, MAP_MBR, (CQBIC_BASE + 4 * 4) >>> 0);        // REG_MBR == 4
        for (let e of c.entries) {
            a.movImmAbs(4, ((e.valid ? CQMAP_VLD : 0) | e.p) >>> 0, (CQMAP_BASE + e.q * 4) >>> 0);
        }
    }
    for (let st of c.steps) { st.pc = (R_CODE + a.len) >>> 0; emitStep(a, st, c); }
    a.halt();
    c.code = a.b;
    c.haltPC = (R_CODE + c.code.length) >>> 0;
    if (c.code.length > 0x4000) throw new Error(`mscperrdiff: case "${c.name}" code is ${c.code.length} bytes`);
    return c;
}

/**
 * emitStep(a, st, c)
 *
 * Descriptors are written by the HOST, to PHYSICAL addresses, with real instructions -- the
 * controller reaches the same words through the CQBIC scatter-gather map.  Anything this switch does
 * not know falls through to mscpharness's emitAction(), which is the shared vocabulary the init
 * handshake is written in; so a case's `steps` list can interleave handshake writes, descriptor
 * writes and observations freely, which PHASE P needs (it performs TWO handshakes with a fatal error
 * between them).
 */
function emitStep(a, st, c)
{
    let g = c.g;
    let descOf = (own, flag, addr) => (((own ? UQ_DESC_OWN : 0) | (flag ? UQ_DESC_F : 0) | addr) >>> 0);
    switch (st.s) {
    case "cdesc":
        return a.movImmAbs(4, descOf(true, true, g.cmdEnv(st.pkt)), c.phys(g.cqBa + st.slot * 4));
    case "rdesc":
        return a.movImmAbs(4, descOf(true, true, g.rspEnv(st.buf)), c.phys(g.rqBa + st.slot * 4));
    /* ONE ALIGNED LONGWORD COVERS BOTH FLAG WORDS: SA_COMM_CI is comm-4 and SA_COMM_RI is comm-2,
       adjacent and little-endian.  Reading them as two words would mean an UNALIGNED longword at
       comm-2 whose upper half lies in the NEXT QBUS PAGE, which the map scatters somewhere else --
       HANDOFF.md standing rule 16, earned by a sibling of this file. */
    case "clrint":
        return a.movImmAbs(4, 0, c.phys((c.comm - 4) >>> 0));
    case "ip":
        return a.movAbsReg(2, RQ_IP, REGS.tmp);
    case "await":
        return a.awaitUnbounded(c.phys(g.rqBa + st.slot * 4), (R_RESULT + st.roff) >>> 0, REGS);
    /**
     * *** THE UNBOUNDED WAIT FOR A FATAL, AND WHY IT IS A DIFFERENT LOOP. ***
     *
     *      MOVZWL @#SA, Rprev ; <Asm.poll on SA> ; MOVL Rcnt,@#res ; MOVL Rcur,@#res+4
     *
     * A controller that goes fatal NEVER sends a response, so a case in PHASE F cannot wait on a
     * response descriptor -- but rq_fatal() writes `SA_ER | err` into SA, which at CST_UP was ZERO,
     * so SA is a value the controller is REQUIRED to change and the wait is sound and unbounded
     * (HANDOFF.md standing rule 17: the alternative, a `delay` sized to qtime, is a budget, and a
     * budget is a race made rarer rather than impossible).  It reads SA as a WORD: RQ's I/O window
     * is FOUR BYTES and a longword read at SA would run off the end of it.
     */
    case "awaitsa": {
        a.movAbsReg(2, RQ_SA, REGS.prev);
        a.poll(RQ_SA, REGS.cur, REGS.prev, REGS.cnt);
        a.movRegAbs(REGS.cnt, (R_RESULT + st.roff) >>> 0);
        return a.movRegAbs(REGS.cur, (R_RESULT + st.roff + 4) >>> 0);
    }
    case "snap":
        a.movAbsReg(4, c.phys(st.q >>> 0), REGS.prev);
        return a.movRegAbs(REGS.prev, (R_RESULT + st.roff) >>> 0);
    case "snapphys":
        a.movAbsReg(4, st.pa >>> 0, REGS.prev);
        return a.movRegAbs(REGS.prev, (R_RESULT + st.roff) >>> 0);
    case "snapsa":
        a.movAbsReg(2, RQ_SA, REGS.prev);
        return a.movRegAbs(REGS.prev, (R_RESULT + st.roff) >>> 0);
    }
    if (emitAction(a, Object.assign({a: st.s}, st))) return a;
    throw new Error(`mscperrdiff: unknown step "${st.s}"`);
}

/**
 * handshake(o)
 *
 * The four-step init as a host writes it.  mscpharness's walkScript() is not used here because this
 * file performs the handshake TWICE in one case (PHASE P) and needs the GO word to carry SA_S4H_LF
 * on the second pass -- and because a case may need to stop after step 3 to grant a response
 * descriptor before GO, which is the only moment at which one can be granted: rq_step4() ZEROES the
 * whole communications region, so a descriptor written before it is erased.
 *
 * Registers: R0..R4 hold the SA read-backs, R5..R8 the polling ITERATION COUNTS.  Running it twice
 * OVERWRITES them, deliberately -- there are not fifteen spare registers -- so anything the first
 * pass proved is snapshotted to the RESULT page instead.
 */
function handshake(o)
{
    let comm = o.comm, s = [{s: "rsa", r: 0}];
    s.push({s: "wsa", v: o.s1dat, step: 1});
    s.push({s: "poll", r: 1, prev: 0, cnt: 5});
    s.push({s: "wsa", v: ((comm & SA_S2H_CLO) | (o.prgi ? SA_S2H_PI : 0)) & 0xFFFF, step: 2});
    s.push({s: "poll", r: 2, prev: 1, cnt: 6});
    if (o.pp) {
        s.push({s: "wsa", v: (((comm >>> 16) & SA_S3H_CHI) | SA_S3H_PP) & 0xFFFF, step: 3});
        s.push({s: "poll", r: 3, prev: 2, cnt: 7});          // SA -> 0
        return s;
    }
    s.push({s: "wsa", v: ((comm >>> 16) & SA_S3H_CHI) & 0xFFFF, step: 3});
    s.push({s: "poll", r: 3, prev: 2, cnt: 7});
    if (o.stopBeforeGo) return s;
    s.push({s: "wsa", v: (o.go === undefined ? SA_S4H_GO : o.go), step: 4});
    s.push({s: "poll", r: 4, prev: 3, cnt: 8});
    return s;
}

/**
 * command(seq, o)
 *
 * ONE complete host transaction: clear the interrupt flag words, grant a response descriptor, post
 * the command descriptor, read IP, wait IN BAND for the response descriptor to change, then settle.
 */
function command(seq, o)
{
    let tag = o.tag || `cmd${o.pkt}`;
    seq.push({s: "clrint"});
    seq.push({s: "rdesc", slot: o.rslot, buf: o.rbuf === undefined ? o.pkt : o.rbuf});
    if (o.extraRdesc !== undefined) seq.push({s: "rdesc", slot: o.extraRdesc.slot, buf: o.extraRdesc.buf});
    seq.push({s: "cdesc", slot: o.cslot, pkt: o.pkt});
    seq.push({s: "ip"});
    seq.push({s: "await", slot: o.rslot, cmd: o.pkt, what: `${tag} response descriptor`});
    seq.push({s: "delay", n: o.settle === undefined ? 400 : o.settle, r: REGS.tmp});
    return seq;
}

/**
 * build(o)
 *
 * A case, built in the one order that works: the ring codes come from the COMMAND COUNT, the
 * geometry comes from the ring codes, and only then can a command name a buffer address.
 */
function build(o)
{
    let n = o.ncmds;
    let code = 0;
    while ((1 << code) < Math.max(1, n)) code++;
    if (code > RING_CODE_MAX) throw new Error(`mscperrdiff: ${n} commands needs ring code ${code}`);
    let spec = Object.assign({
        cqCode: code, rqCode: code, comm: 0x2000, nCmdBuf: Math.max(1, n), nRspBuf: Math.max(1, n),
        dataLnt: PAGE, spread: 0, unmappedQ: [], unmappedData: [], invalidData: [],
        invalidQ: [], noMap: false
    }, o.spec || {});
    let g = errGeometry(spec);
    let cmds = o.cmds ? o.cmds(g) : [];
    if (cmds.length !== n) {
        throw new Error(`mscperrdiff: case "${o.name}" declared ${n} commands and produced ${cmds.length}`);
    }
    let packets = {}, seq = [];
    for (let i = 0; i < n; i++) packets[i] = cmdWords(cmds[i]);
    if (o.steps) seq = o.steps(g, n);
    else {
        seq = handshake({s1dat: (SA_S1H_VL | (spec.cqCode << SA_S1H_V_CQ) |
                                 (spec.rqCode << SA_S1H_V_RQ)) & 0xFFFF,
                         comm: spec.comm, prgi: spec.prgi || 0});
        for (let i = 0; i < n; i++) {
            command(seq, {pkt: i, cslot: i, rslot: i, tag: (o.tags || [])[i] || `#${i}`});
        }
    }
    return buildCase(Object.assign({}, spec, {
        packets, steps: seq, name: o.name, kind: o.kind, units: o.units, cmdSpecs: cmds
    }));
}


/* The drive types the containers are attached to, looked up BY NAME so a table that grew an entry
   does not silently re-point them.  RX18 is the SMALLEST drive in drv_tab[] (360 blocks) and its
   `rcts` is ZERO -- which makes `maxlbn + rcts == maxlbn` and collapses rq_rw_valid()'s two RCT
   rungs onto a single LBN.  That is not a convenience: it is the only way to reach rung 6
   ("accessing the RCT with a byte count that is not 512") and rung 5 ("beyond copy 1") from LBNs a
   360-block container can hold, and it makes the pair reachable on ONE unit. */
const RX18_DTYPE = DRV_TAB.findIndex((d) => d.name === "RX18");

function enumeratedCases()
{
    let cases = [];
    let add = (c) => { c.idx = cases.length; cases.push(c); return c; };
    let U = unitSpec;

    /* THE READ CONFIGURATION.  Unit 0 carries the container READ ONLY (so autosize's clamp is
       skipped and the unit is exactly 1,024 blocks, putting the replacement table in reach); unit 1
       is an UNATTACHED RD54; unit 2 is attached and deliberately never brought ONLINE. */
    let RDU = () => [U({dtype: RD54_DTYPE, image: "err", ro: true}),
                     U({dtype: RD54_DTYPE}),
                     U({dtype: RD54_DTYPE, image: "err", ro: true})];
    let CAPR = () => capacFor(IMG.err, RD54_DTYPE, true);           /* 1024 */
    let RCTS = () => DRV_TAB[RD54_DTYPE].rcts;                      /* 7 */

    /* THE WRITE CONFIGURATION.  Unit 0 is HARDWARE locked (`set rq0 locked`), unit 1 is writable
       and picks up the SOFTWARE lock part way through the case, and unit 2 is the READ-ONLY RD54.
       *** UNIT 2 IS THERE BECAUSE RUNG 8 CANNOT WIN ON AN RX18. ***  RX18's `rcts` is ZERO, so
       `lbn >= maxlbn + rcts` is the same test as `lbn >= maxlbn` and rung 5 -- which is EARLIER --
       fires for every LBN that would reach rung 8.  Only a drive with a real replacement table has
       an LBN that is inside the RCT window (so rungs 5 and 6 pass) and still refused to a WRITE, and
       RD54's rcts is 7.  That is not a detail of the fixture: it is why the ladder has two
       `lbn >= maxlbn` tests at all, and a case list built only on RX18 leaves the second one
       unreachable while looking complete. */
    let WRU = () => [U({dtype: RX18_DTYPE, image: "wr", locked: true}),
                     U({dtype: RX18_DTYPE, image: "wr2"}),
                     U({dtype: RD54_DTYPE, image: "err", ro: true})];
    let CAPW = () => capacFor(IMG.wr, RX18_DTYPE, false);           /* 360 */

    let ONL = (u, ref) => ({opc: OP.ONL, unit: u, ref});
    /* An online, attached, unlocked unit -- the state every read rung below is measured against. */
    let LIVE = {att: 1, onl: 1, wps: 0, wph: 0};

    /* ---- 1. THE READ RUNGS, ONE COMMAND EACH.  Every one of these trips EXACTLY ONE rung, which
       is deliberately the WEAK form: it proves the rung's STATUS and proves nothing about the
       ladder's order.  Case 3 below is where the order is measured. ---- */
    add(build({
        name: "rq_rw_valid rungs 0-7: one command per rung, one rung per command",
        kind: "ladder-read", ncmds: 10, units: RDU(), spec: {spread: 3, dataLnt: 2 * PAGE},
        cmds: (g) => [
            ONL(0, 0x1000),
            /* rung 0 -- the unit exists (plug 1) and has no container */
            {opc: OP.RD, unit: 1, ref: 0x1001, bc: 512, ba: g.dataBase, lbn: 0,
             st: {att: 0, onl: 0, wps: 0, wph: 0}},
            /* rung 1 -- attached, never brought ONLINE.  A DIFFERENT answer (ST_AVL, bare) from
               the rung above it, and the two are three lines apart in the C. */
            {opc: OP.RD, unit: 2, ref: 0x1002, bc: 512, ba: g.dataBase, lbn: 0,
             st: {att: 1, onl: 0, wps: 0, wph: 0}},
            /* rung 2 -- odd BUFFER ADDRESS */
            {opc: OP.RD, unit: 0, ref: 0x1003, bc: 512, ba: g.dataBase + 1, lbn: 4, st: LIVE},
            /* rung 3 -- odd BYTE COUNT */
            {opc: OP.RD, unit: 0, ref: 0x1004, bc: 511, ba: g.dataBase, lbn: 4, st: LIVE},
            /* rung 4 -- bc & 0xF0000000.  Note it is EVEN, so rung 3 does not also fire. */
            {opc: OP.RD, unit: 0, ref: 0x1005, bc: 0xF0000000, ba: g.dataBase, lbn: 4, st: LIVE},
            /* rung 5 -- an LBN past the far edge of the replacement table */
            {opc: OP.RD, unit: 0, ref: 0x1006, bc: 512, ba: g.dataBase, lbn: CAPR() + RCTS(), st: LIVE},
            /* rung 6 -- INSIDE the replacement table, but not exactly one block */
            {opc: OP.RD, unit: 0, ref: 0x1007, bc: 1024, ba: g.dataBase, lbn: CAPR(), st: LIVE},
            /* rung 7 -- starts inside the disk and SPIRALS off the end.  This is the `else` of the
               `lbn >= maxlbn` test, so it can never fire together with 5 or 6. */
            {opc: OP.RD, unit: 0, ref: 0x1008, bc: 1024, ba: g.dataBase, lbn: CAPR() - 1, st: LIVE},
            /* rung 11 -- ACCEPTED.  One block, inside the disk, even everything.  Without it the
               ladder's accept arm is a branch nothing exercised. */
            {opc: OP.RD, unit: 0, ref: 0x1009, bc: 512, ba: g.dataBase + PAGE, lbn: 9, st: LIVE}
        ],
        tags: ["ONL", "rung 0 not attached", "rung 1 not online", "rung 2 odd address",
               "rung 3 odd count", "rung 4 unreasonable bc", "rung 5 beyond the RCT",
               "rung 6 RCT bc != 512", "rung 7 spiral", "rung 11 accepted"]
    }));

    /* ---- 2. THE WRITE RUNGS.  8, 9 and 10 live inside `if ((cmd == OP_WR) || (cmd == OP_ERS))`
       and are UNREACHABLE for a READ however locked the drive is -- which the case proves by
       reading the hardware-locked unit successfully in the middle of it.  Unit 1 acquires UF_WPS
       part way through via SET UNIT CHARACTERISTICS: *** BOTH the host's UF_WPS bit in the packet
       AND the MD_SWP modifier are required *** or rq_suc() ignores it. ---- */
    add(build({
        name: "rq_rw_valid rungs 8-10: the RCT is closed to writes, then the two locks",
        kind: "ladder-write", ncmds: 12, units: WRU(), spec: {spread: 5, dataLnt: 2 * PAGE},
        cmds: (g) => [
            ONL(0, 0x1100),
            ONL(1, 0x1101),
            ONL(2, 0x1109),
            /* *** RUNG 8, AND THE ONLY SHAPE IN WHICH IT CAN WIN. ***  Unit 2 is an RD54 whose rcts
               is 7, so an LBN of exactly `capac` with a byte count of exactly 512 passes rungs 5 and
               6 -- it IS a legal one-block access to the replacement table -- and is then refused by
               rung 8 because it is a WRITE.  A READ of the identical extent is LEGAL, and the
               command after it proves that on the same unit and the same LBN. */
            {opc: OP.WR, unit: 2, ref: 0x110A, bc: 512, ba: g.dataBase, lbn: CAPR(),
             st: {att: 1, onl: 1, wps: 0, wph: 1}},
            {opc: OP.RD, unit: 2, ref: 0x110B, bc: 512, ba: g.dataBase, lbn: CAPR(),
             st: {att: 1, onl: 1, wps: 0, wph: 1}},
            /* On an RX18 the SAME command is rung 5 instead, because rcts == 0 collapses the RCT
               window to nothing -- so these two are the measurement that the window exists. */
            {opc: OP.WR, unit: 1, ref: 0x1102, bc: 512, ba: g.dataBase, lbn: CAPW(),
             st: {att: 1, onl: 1, wps: 0, wph: 0}},
            {opc: OP.RD, unit: 1, ref: 0x1103, bc: 512, ba: g.dataBase, lbn: CAPW(),
             st: {att: 1, onl: 1, wps: 0, wph: 0}},
            /* An ERASE at the same LBN: OP_ERS takes the same arm as OP_WR and does NOT take the
               odd-address arm, so `ba` is left at zero here to prove that. */
            {opc: OP.ERS, unit: 1, ref: 0x1104, bc: 512, ba: 0, lbn: CAPW(),
             st: {att: 1, onl: 1, wps: 0, wph: 0}},
            {opc: OP.SUC, unit: 1, ref: 0x1105, ufl: UF_WPS, mod: MD_SWP},
            /* rung 9 -- software lock */
            {opc: OP.WR, unit: 1, ref: 0x1106, bc: 512, ba: g.dataBase, lbn: 4,
             st: {att: 1, onl: 1, wps: 1, wph: 0}},
            /* rung 10 -- hardware lock, on the unit that was `set locked` */
            {opc: OP.WR, unit: 0, ref: 0x1107, bc: 512, ba: g.dataBase, lbn: 4,
             st: {att: 1, onl: 1, wps: 0, wph: 1}},
            /* ...and a READ of that same hardware-locked unit is LEGAL, because rungs 9 and 10 are
               inside the write arm.  A model that hoisted the lock tests out of it fails here. */
            {opc: OP.RD, unit: 0, ref: 0x1108, bc: 512, ba: g.dataBase + PAGE, lbn: 4,
             st: {att: 1, onl: 1, wps: 0, wph: 1}}
        ],
        tags: ["ONL locked unit", "ONL writable unit", "ONL the RD54", "rung 8 WR into the RCT",
               "RD the same RCT block (legal)", "rung 5 WR past an rcts=0 disk", "rung 5 RD ditto",
               "rung 5 ERS ditto", "SUC UF_WPS + MD_SWP", "rung 9 software lock",
               "rung 10 hardware lock", "RD the locked unit (legal)"]
    }));

    /* ---- 3. *** THE CASE THE WHOLE FILE IS FOR: COMMANDS THAT TRIP TWO OR MORE RUNGS. ***
       Every command here is invalid in at least two ways at once, and the ONLY thing that decides
       what it answers is the ORDER of the ladder.  Shuffle rq_rw_valid()'s rungs into any other
       order and case 1 above still passes completely; this one does not.  grade() computes the
       tripped SET with trippedRungs() -- a second writing of the C's conditions that never consults
       rq.js -- and requires the ORACLE to have answered the FIRST of them. ---- */
    add(build({
        name: "MULTI-RUNG commands: which refusal wins when two or three apply at once",
        kind: "ladder-order", ncmds: 8, units: RDU(), spec: {spread: 7, dataLnt: 2 * PAGE},
        cmds: (g) => [
            ONL(0, 0x1200),
            /* rungs 2 AND 3: odd address AND odd count.  Answer must be SB_HST_OA. */
            {opc: OP.RD, unit: 0, ref: 0x1201, bc: 511, ba: g.dataBase + 1, lbn: 4, st: LIVE},
            /* rungs 3 AND 5: odd count AND an LBN past the replacement table.  Answer must be
               SB_HST_OC -- the ODD COUNT, not the bad block number, which is the example the item
               this file belongs to names explicitly. */
            {opc: OP.RD, unit: 0, ref: 0x1202, bc: 511, ba: g.dataBase, lbn: CAPR() + RCTS() + 40,
             st: LIVE},
            /* rungs 4 AND 7: an unreasonable byte count that also spirals off the end.  Answer must
               be the EARLIER one -- and both are ST_CMD|I_BCNT, so this pair is graded by the
               RESIDUAL and the packet rather than by the status word alone. */
            {opc: OP.RD, unit: 0, ref: 0x1203, bc: 0xF0000000, ba: g.dataBase, lbn: CAPR() - 1,
             st: LIVE},
            /* rungs 1 AND 2 AND 3: a unit that is attached and NOT ONLINE, addressed with an odd
               buffer address and an odd count.  Answer must be the bare ST_AVL. */
            {opc: OP.RD, unit: 2, ref: 0x1204, bc: 511, ba: g.dataBase + 1, lbn: 4,
             st: {att: 1, onl: 0, wps: 0, wph: 0}},
            /* rungs 0 AND 2 AND 3: an UNATTACHED unit, likewise.  ST_OFL|SB_OFL_NV wins over both. */
            {opc: OP.RD, unit: 1, ref: 0x1205, bc: 511, ba: g.dataBase + 1, lbn: 4,
             st: {att: 0, onl: 0, wps: 0, wph: 0}},
            /* rungs 5 AND 6 together: an LBN beyond copy 1 of the replacement table WITH a byte
               count that is not 512.  They are the two arms INSIDE `if (lbn >= maxlbn)` and the
               first is I_LBN while the second is I_BCNT, so the pair distinguishes them. */
            {opc: OP.RD, unit: 0, ref: 0x1206, bc: 1024, ba: g.dataBase, lbn: CAPR() + RCTS(),
             st: LIVE},
            /* *** THE ACCESS EXEMPTION MEETS THE ORDER. ***  OP_ACC skips rung 2 entirely, so the
               SAME odd address and odd count that answered SB_HST_OA above answer SB_HST_OC here.
               One command, and it grades the exemption and the order together. */
            {opc: OP.ACC, unit: 0, ref: 0x1207, bc: 511, ba: g.dataBase + 1, lbn: 4, st: LIVE}
        ],
        tags: ["ONL", "odd addr + odd count", "odd count + past the RCT", "huge bc + spiral",
               "not online + odd addr + odd count", "not attached + odd addr + odd count",
               "beyond the RCT + bad RCT count", "ACC: exempt from rung 2, so rung 3 wins"]
    }));

    /* ---- 4. THE MULTI-RUNG WRITE CASES.  The write arm's three rungs interleave with the earlier
       ones, and the two that matter are: a WRITE into the replacement table on a LOCKED drive
       (rung 8 beats rungs 9 and 10), and a drive that is locked BOTH WAYS (rung 9 beats rung 10 --
       and rq_svc() tests those same two IN THE OPPOSITE ORDER, which is why the pair is worth
       distinguishing at all). ---- */
    add(build({
        name: "MULTI-RUNG writes: the RCT beats both locks, and software beats hardware",
        kind: "ladder-order-write", ncmds: 10, spec: {spread: 11, dataLnt: 2 * PAGE},
        units: [U({dtype: RX18_DTYPE, image: "wr", locked: true}),
                U({dtype: RX18_DTYPE, image: "wr2", locked: true}),
                U({dtype: RD54_DTYPE, image: "err", ro: true})],
        cmds: (g) => [
            ONL(0, 0x1300),
            ONL(1, 0x1301),
            ONL(2, 0x1308),
            /* rungs 8 AND 10 on a drive that HAS a replacement table: a one-block WRITE at exactly
               `capac` passes rungs 5 and 6 and is then refused by rung 8, which is EARLIER than the
               hardware lock the read-only attach also puts on it.  ST_CMD|I_LBN wins. */
            {opc: OP.WR, unit: 2, ref: 0x1309, bc: 512, ba: g.dataBase, lbn: CAPR(),
             st: {att: 1, onl: 1, wps: 0, wph: 1}},
            /* rungs 5 AND 10 on a drive that has NONE: the same idea one rung earlier. */
            {opc: OP.WR, unit: 0, ref: 0x1302, bc: 512, ba: g.dataBase, lbn: CAPW(),
             st: {att: 1, onl: 1, wps: 0, wph: 1}},
            /* rungs 3 AND 10: an ODD COUNT on a hardware-locked drive.  The odd count is rung 3 and
               it beats the lock, which is eight lines further down. */
            {opc: OP.WR, unit: 0, ref: 0x1303, bc: 511, ba: g.dataBase, lbn: 4,
             st: {att: 1, onl: 1, wps: 0, wph: 1}},
            {opc: OP.SUC, unit: 1, ref: 0x1304, ufl: UF_WPS, mod: MD_SWP},
            /* rungs 9 AND 10: locked BOTH ways.  rq_rw_valid() answers SOFTWARE. */
            {opc: OP.WR, unit: 1, ref: 0x1305, bc: 512, ba: g.dataBase, lbn: 4,
             st: {att: 1, onl: 1, wps: 1, wph: 1}},
            /* rungs 5 AND 9 AND 10 at once: past the end of a drive locked both ways. */
            {opc: OP.ERS, unit: 1, ref: 0x1306, bc: 512, ba: 0, lbn: CAPW() + 3,
             st: {att: 1, onl: 1, wps: 1, wph: 1}},
            /* rungs 2 AND 9 AND 10: an odd ADDRESS on a doubly-locked drive.  OP_WR is NOT exempt
               from rung 2, so the odd address wins -- the mirror of the OP_ACC command above. */
            {opc: OP.WR, unit: 1, ref: 0x1307, bc: 512, ba: g.dataBase + 1, lbn: 4,
             st: {att: 1, onl: 1, wps: 1, wph: 1}}
        ],
        tags: ["ONL", "ONL", "ONL the RD54", "WR into a real RCT + hw lock", "WR past an rcts=0 disk",
               "WR odd count + hw lock", "SUC UF_WPS", "WR sw lock + hw lock",
               "ERS past the disk + both locks", "WR odd addr + both locks"]
    }));

    return cases;
}

/**
 * svcCases()
 *
 * PHASE E -- the exits from rq_svc() that rq_rw_valid() has already ACCEPTED.  These are the
 * failures that happen after the command is under way, and every one of them writes a RESIDUAL into
 * a field the host still owns.
 */
function svcCases(startIdx)
{
    let cases = [];
    let add = (c) => { c.idx = startIdx + cases.length; cases.push(c); return c; };
    let U = unitSpec;
    let RDU = () => [U({dtype: RD54_DTYPE, image: "err", ro: true}),
                     U({dtype: RD54_DTYPE}),
                     U({dtype: RD54_DTYPE, image: "err", ro: true})];
    let LIVE = {att: 1, onl: 1, wps: 0, wph: 0};
    let ONL = (u, ref) => ({opc: OP.ONL, unit: u, ref});

    /* ---- E1. A ZERO BYTE COUNT.  rq_rw_valid() ACCEPTS it -- 0 is even, is not 0xF0000000, and
       `lbn + ceil(0/512)` is `lbn`, which is inside the disk -- so the command starts, the unit is
       scheduled, and rq_svc()'s SECOND test answers ST_SUC and ends it.  *** NO DISK OPERATION
       HAPPENS AT ALL ***, which is the observable: the oracle's DEBUG=REQ stream carries no
       `sim_disk_rdsect` line for it, and coverage() requires that.  A model that treated bc == 0 as
       a refusal, or that issued a zero-length read, both answer ST_SUC. ---- */
    add(build({
        name: "a ZERO byte count -- accepted, ST_SUC, and no disk operation at all",
        kind: "zerobc", ncmds: 4, units: RDU(), spec: {spread: 13, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0, 0x2000),
            {opc: OP.RD, unit: 0, ref: 0x2001, bc: 0, ba: g.dataBase, lbn: 5, st: LIVE},
            /* The same zero count on an ACCESS and an ERASE: both reach the same second test, and
               ERASE would otherwise WRITE a block of zeros -- so a container that changed here is
               the strongest possible statement that this arm was not taken. */
            {opc: OP.ACC, unit: 0, ref: 0x2002, bc: 0, ba: g.dataBase, lbn: 5, st: LIVE},
            {opc: OP.RD, unit: 0, ref: 0x2003, bc: 512, ba: g.dataBase, lbn: 5, st: LIVE}],
        tags: ["ONL", "RD bc=0", "ACC bc=0", "RD 1 block (the control)"]
    }));

    /* ---- E2. A COMPARE THAT FINDS A DIFFERENCE.  rq_svc()'s COMPARE arm walks host memory a BYTE
       at a time and, on the first mismatch, writes `bc - i` into RW_WBCL and answers ST_CMP -- so
       the host's own RW_BCL comes back as `bc - (bc - i)` == `i`, THE OFFSET OF THE FIRST DIFFERING
       BYTE.  The case reads one block, compares it against ITSELF (ST_SUC) and then against a
       DIFFERENT block (ST_CMP), so the residual is a real measurement of where two blocks of
       generated data first diverge rather than a constant. ---- */
    add(build({
        name: "a COMPARE mismatch -- ST_CMP and a residual naming the first differing byte",
        kind: "cmpfail", ncmds: 5, units: RDU(), spec: {spread: 17, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0, 0x2100),
            {opc: OP.RD,  unit: 0, ref: 0x2101, bc: 1024, ba: g.dataBase, lbn: 40, st: LIVE},
            {opc: OP.CMP, unit: 0, ref: 0x2102, bc: 1024, ba: g.dataBase, lbn: 40, st: LIVE},
            {opc: OP.CMP, unit: 0, ref: 0x2103, bc: 1024, ba: g.dataBase, lbn: 41, st: LIVE},
            /* A COMPARE that diverges LATE -- the second block of a two-block extent -- so the
               residual is not merely non-zero but a specific number bigger than one block. */
            {opc: OP.CMP, unit: 0, ref: 0x2104, bc: 1024, ba: g.dataBase, lbn: 39, st: LIVE}],
        tags: ["ONL", "RD 2 blocks", "CMP the same -> ST_SUC", "CMP block 41 -> ST_CMP",
               "CMP block 39 -> ST_CMP"]
    }));

    /* ---- E3. A HOST-BUFFER NXM PART WAY THROUGH THE DMA, WITH ERROR LOGGING OFF.
       *** THE MAP ENTRY IS PROGRAMMED AND ITS VALID BIT IS CLEAR. ***  Not "left unwritten" -- the
       host writes a real physical page number into the CQBIC map entry for the buffer's SECOND page
       and simply does not set CQMAP_VLD, which is pcjsvax-e22's already-graded mapAddr() failure
       path.  A controller that tested `entry != 0` rather than `entry & CQMAP_VLD` completes the
       transfer and writes 512 bytes into a page nothing intended; the whole-page dump of that
       physical page is what catches it, and the page is dumped precisely BECAUSE its entry is
       invalid.
       *** THE FIRST PAGE IS DELIBERATELY VALID. ***  A transfer whose FETCH fails entirely is
       rq_svc()'s hang (no disk I/O, no callback, the unit never re-scheduled) and
       assertExclusions() refuses any case that arranges it.
       rq_svc() writes `bc - (tbc - t)` into RW_WBCL and `ba + (tbc - t)` into RW_WBAL, asks rq_hbe()
       for an error log -- which with CF_THS clear builds NOTHING and returns OK anyway -- and ends
       the command with EF_LOG set and ST_HST|SB_HST_NXM.  So the END PACKET IS IDENTICAL whether or
       not logging is on, and the only difference is a second packet on the ring. ---- */
    add(build({
        name: "an NXM part way through a READ -- residual, EF_LOG, ST_HST|SB_HST_NXM, logging OFF",
        kind: "nxm-off", ncmds: 3, units: RDU(),
        spec: {spread: 19, dataLnt: 4 * PAGE, invalidData: [1]},
        cmds: (g) => [ONL(0, 0x2200),
            {opc: OP.RD, unit: 0, ref: 0x2201, bc: 2048, ba: g.dataBase, lbn: 17, st: LIVE},
            /* A SECOND transfer whose buffer starts INSIDE the good page and runs into the invalid
               one at an offset -- so the residual is not a whole number of pages. */
            {opc: OP.RD, unit: 0, ref: 0x2202, bc: 1024, ba: g.dataBase + 256, lbn: 18, st: LIVE}],
        tags: ["ONL", "RD into a buffer whose second map entry is INVALID",
               "RD starting mid-page into the same invalid entry"]
    }));

    /* ---- E4. THE SAME NXM WITH CF_THS SET, plus the WRITE path and the COMPARE path.
       With logging on, rq_hbe() takes a packet off the FREE list and sends a DATAGRAM carrying
       FM_BAD, LF_SNR and -- the part that matters -- HBE_BADL/HBE_BADH, which is a COPY OF THE
       WORKING BUS ADDRESS the caller just adjusted.  That word is the ONLY way a host ever sees
       RW_WBAL, because rq_rw_end() zeroes the working words before the end packet goes out.  So:
         - the READ arm's `ba + (tbc - t)` is graded here and nowhere else;
         - the COMPARE arm's `bc - i` -- A BYTE COUNT WRITTEN INTO THE BUS ADDRESS FIELD, which is
           almost certainly a vendor typo and is reproduced verbatim -- is graded here and nowhere
           else;
         - the WRITE arm's `bc - abc` / `ba + abc` is graded here, and its transfer is the ONLY
           accepted write in this whole file (it goes to its own container, "wr3", which PHASE I
           compares between the two engines as a FILE).
       *** EVERY COMMAND NEEDS TWO RESPONSE DESCRIPTORS, AND A HOST THAT GRANTS ONE HANGS. ***
       rq_hbe() builds a SECOND packet for the same command; with no free descriptor it goes on the
       response queue and the queue thread re-arms itself forever, which vax_cpu.c's HALT drain runs
       into.  So each transaction grants two and the second is SNAPSHOTTED rather than waited on --
       both are already released by the time the first wait returns, and a wait that reads its
       "previous" value AFTER the change can never see one. ---- */
    add(build({
        name: "the same NXM with CF_THS set -- an HBE datagram, and the READ / WRITE / COMPARE residuals",
        kind: "nxm-on", ncmds: 6,
        units: [U({dtype: RD54_DTYPE, image: "err", ro: true}),
                U({dtype: RX18_DTYPE, image: "wr3"}),
                U({dtype: RD54_DTYPE})],
        /* *** THE RESPONSE RING NEEDS SIXTEEN SLOTS, NOT EIGHT. ***  Each of the three logged
           transfers grants TWO descriptors (rq_hbe()'s datagram and the end packet) on top of the
           three set-up commands, so the highest slot this case writes is 8 -- and a `rqCode` of 3
           gives a ring of 8 slots, whose slot 8 is the first longword of the COMMAND ring.  The
           first version of this case did exactly that: the host wrote a descriptor over the command
           ring, the controller never answered, and every wait in this file being unbounded, the
           case was reported as a machine that never reached its own HALT. */
        spec: {spread: 23, dataLnt: 4 * PAGE, invalidData: [1], cqCode: 3, rqCode: 4, nRspBuf: 12},
        cmds: (g) => [
            {opc: OP.SCC, unit: 0, ref: 0x2300, cfl: CF_THS},
            ONL(0, 0x2301),
            ONL(1, 0x2302),
            {opc: OP.RD,  unit: 0, ref: 0x2303, bc: 2048, ba: g.dataBase, lbn: 17, st: LIVE},
            {opc: OP.CMP, unit: 0, ref: 0x2304, bc: 2048, ba: g.dataBase, lbn: 17, st: LIVE},
            {opc: OP.WR,  unit: 1, ref: 0x2305, bc: 1024, ba: g.dataBase, lbn: 10, st: LIVE}],
        steps: (g) => {
            let seq = handshake({s1dat: (SA_S1H_VL | (3 << SA_S1H_V_CQ) | (4 << SA_S1H_V_RQ)) & 0xFFFF,
                                 comm: 0x2000, prgi: 0});
            command(seq, {pkt: 0, cslot: 0, rslot: 0, tag: "SCC CF_THS"});
            command(seq, {pkt: 1, cslot: 1, rslot: 1, tag: "ONL unit 0"});
            command(seq, {pkt: 2, cslot: 2, rslot: 2, tag: "ONL unit 1"});
            let logged = (pkt, cslot, rslot, tag) => {
                seq.push({s: "clrint"});
                seq.push({s: "rdesc", slot: rslot, buf: rslot});
                seq.push({s: "rdesc", slot: rslot + 1, buf: rslot + 1});    /* rq_hbe()'s log packet */
                seq.push({s: "cdesc", slot: cslot, pkt});
                seq.push({s: "ip"});
                /* rq_hbe() runs BEFORE rq_rw_end(), so the LOG datagram takes the first descriptor
                   and the END packet the second -- the wait is therefore on the SECOND slot, which
                   is the last one to change, and the first is snapshotted afterwards. */
                seq.push({s: "await", slot: rslot + 1, cmd: pkt, what: `${tag} END packet`});
                seq.push({s: "delay", n: 400, r: REGS.tmp});
                seq.push({s: "snap", q: (g.rqBa + rslot * 4) >>> 0, what: `${tag} HBE datagram descriptor`});
            };
            logged(3, 3, 3, "RD nxm");
            logged(4, 4, 5, "CMP nxm");
            logged(5, 5, 7, "WR nxm");
            return seq;
        },
        tags: ["SCC CF_THS", "ONL unit 0", "ONL unit 1", "RD nxm (logged)", "CMP nxm (logged)",
               "WR nxm (logged)"]
    }));

    return cases;
}

/**
 * fatalCases()
 *
 * PHASE F -- the seven PORT ERRORS a host can provoke, each of which calls rq_fatal() and takes the
 * controller out of service: `rq_reset()` FIRST, and only then `sa = SA_ER | err`, `csta = CST_DEAD`
 * and `perr = err`.  The ORDER matters and is graded: a version that set the three fields and reset
 * afterwards leaves SA at 0x0B40 and csta at CST_S1.
 *
 * *** A DEAD CONTROLLER NEVER SENDS A RESPONSE, SO NONE OF THESE CAN WAIT ON THE RESPONSE RING. ***
 * They wait on SA instead, which rq_fatal() is REQUIRED to change (at CST_UP it is zero and becomes
 * 0x800x), with the same unbounded loop and for the same reason -- a `delay` sized to `qtime` would
 * be a budget, and HANDOFF.md standing rule 17 is that a budget is a race made rarer rather than
 * impossible.
 */
function fatalCases(startIdx)
{
    let cases = [];
    let add = (c) => { c.idx = startIdx + cases.length; cases.push(c); return c; };
    let COMM = 0x2000;
    let S1 = (code) => (SA_S1H_VL | (code << SA_S1H_V_CQ) | (code << SA_S1H_V_RQ)) & 0xFFFF;

    /** A fatal provoked by a COMMAND: full handshake, post one packet, wait for SA to move. */
    let byCommand = (o) => add(build({
        name: o.name, kind: o.kind, ncmds: 1, units: o.units || [],
        spec: Object.assign({comm: COMM, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
                             spread: o.spread}, o.spec || {}),
        cmds: () => [o.cmd],
        steps: (g) => {
            let seq = handshake({s1dat: S1(0), comm: COMM, prgi: 0});
            seq.push({s: "clrint"});
            seq.push({s: "rdesc", slot: 0, buf: 0});
            seq.push({s: "cdesc", slot: 0, pkt: 0});
            if (o.invalidateAfter !== undefined) {
                /* PROGRAMMED INVALID, not erased: a real physical page number with CQMAP_VLD clear.
                   Written HERE rather than at build time because the same page had to be VALID for
                   the handshake -- rq_step4() zeroes the communications region through the map, and
                   an entry that was already invalid would answer PE_QWE at step 4 instead of the
                   error this case is about. */
                seq.push({s: "went", q: o.invalidateAfter, v: physFor(o.invalidateAfter, o.spread)});
            }
            seq.push({s: "ip"});
            seq.push({s: "awaitsa", what: "SA after the fatal"});
            seq.push({s: "delay", n: 400, r: REGS.tmp});
            seq.push({s: "snapsa", what: "SA re-read after the fatal"});
            /* The ring interrupt flag words, which rq_reset() does NOT touch -- so whatever the
               controller had written there before it died is still there. */
            seq.push({s: "snap", q: (COMM - 4) >>> 0, what: "the interrupt flag words after the fatal"});
            return seq;
        },
        tags: [o.tag]
    }));

    /* ---- PE_PIE (20).  The packet's UQ_HCTC TYPE field is not UQ_TYP_SEQ.  rq_quesvc() tests it
       BEFORE it looks at the connection ID, so a packet that is both non-sequential and unknown
       answers PE_PIE and not PE_ICI -- an ordering inside the fatal path, tested the same way the
       ladder's is. ---- */
    byCommand({name: "a packet whose UQ_HCTC type is not UQ_TYP_SEQ -> rq_fatal(PE_PIE)",
               kind: "fatal-pie", spread: 29, tag: "GUS with type = UQ_TYP_DAT",
               cmd: {opc: OP.GUS, unit: 0, ref: 0x3000, typ: UQ_TYP_DAT, cid: 0x33}});

    /* ---- PE_ICI (14).  A SEQUENTIAL packet whose connection ID is neither UQ_CID_MSCP nor
       UQ_CID_DUP.  (UQ_CID_DUP is answered with ST_CMD|I_OPCD and is NOT fatal -- that arm is
       tests/mscpringdiff.js's.) ---- */
    byCommand({name: "a packet whose connection ID is neither MSCP nor DUP -> rq_fatal(PE_ICI)",
               kind: "fatal-ici", spread: 31, tag: "GUS with CID = 0x33",
               cmd: {opc: OP.GUS, unit: 0, ref: 0x3100, cid: 0x33}});

    /* ---- PE_PRE (1).  The COMMAND PACKET's own Qbus page carries a map entry with the VALID BIT
       CLEAR.  rq_getpkt() gets the descriptor (the ring page is fine), takes a free packet, disables
       the host timer and THEN fails the 64-byte read.  *** THE PACKET IS NOT RETURNED TO THE FREE
       LIST *** -- PBSY would be 1 -- except that rq_fatal() calls rq_reset(), which rebuilds the
       pool; so the observable is that the pool is INTACT and the controller is dead. ---- */
    byCommand({name: "the command packet's map entry INVALID -> rq_fatal(PE_PRE)",
               kind: "fatal-pre", spread: 37, tag: "GUS whose packet cannot be fetched",
               cmd: {opc: OP.GUS, unit: 0, ref: 0x3200},
               spec: {invalidQ: [((0x2000 / PAGE) | 0) + 1]}});

    /* ---- PE_PWE (2).  The RESPONSE PACKET's page instead.  The command is fetched, DISPATCHED and
       ANSWERED -- so the DEBUG=REQ trace carries both a `cmd=` line and an `rsp=` line -- and the
       controller dies writing the answer out.  That ordering is the whole difference from PE_PRE
       and it is graded as text. ---- */
    byCommand({name: "the response packet's map entry INVALID -> rq_fatal(PE_PWE)",
               kind: "fatal-pwe", spread: 41, tag: "GUS whose response cannot be stored",
               cmd: {opc: OP.GUS, unit: 0, ref: 0x3300},
               spec: {invalidQ: [((0x2000 / PAGE) | 0) + 2]}});

    /* ---- PE_QRE (6).  The RING page, invalidated AFTER the handshake so that step 4 succeeds and
       the failure lands in rq_getdesc()'s read rather than in rq_step4()'s zeroing.  Getting that
       wrong reports PE_QWE, which is a different code down a different call path and is the case
       below. ---- */
    byCommand({name: "the ring page's map entry invalidated after step 4 -> rq_fatal(PE_QRE)",
               kind: "fatal-qre", spread: 43, tag: "GUS polled through a ring nobody can read",
               cmd: {opc: OP.GUS, unit: 0, ref: 0x3400},
               invalidateAfter: ((0x2000 / PAGE) | 0)});

    /* ---- PE_QWE (7).  NO MAP AT ALL.  rq_step4()'s zeroing of the communications region is the
       FIRST DMA the controller ever performs, so an unprogrammed map kills it at the step-3 reply,
       before the host has written GO.  Measured on the live oracle: SA = 0x8007, CSTA = 8,
       PERR = 007, COMM reset to 0.  The host must NOT write GO afterwards -- SA never moves again
       and a poll for a change would spin forever. ---- */
    add(build({
        name: "an unprogrammed CQBIC map -- step 4 cannot zero the comm region -> rq_fatal(PE_QWE)",
        kind: "fatal-qwe", ncmds: 0, units: [],
        spec: {comm: COMM, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1, spread: 47, noMap: true},
        cmds: () => [],
        steps: () => {
            let seq = handshake({s1dat: S1(0), comm: COMM, prgi: 0, stopBeforeGo: true});
            seq.push({s: "delay", n: 400, r: REGS.tmp});
            seq.push({s: "snapsa", what: "SA after the step-4 fatal"});
            return seq;
        },
        tags: []
    }));

    /* ---- PE_PPF (21).  The PURGE/POLL test: the host asks for it with SA_S3H_PP in its step-3
       word, the controller answers SA = 0 and waits for a ZERO write, and ANY NON-ZERO value is
       fatal.  No DMA is involved at all, which is why this is the fatal PHASE P below uses to arm
       the last-failure log. ---- */
    add(build({
        name: "a NON-ZERO write in CST_S3_PPA -> rq_fatal(PE_PPF)",
        kind: "fatal-ppf", ncmds: 0, units: [],
        spec: {comm: COMM, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1, spread: 53},
        cmds: () => [],
        steps: () => {
            let seq = handshake({s1dat: S1(0), comm: COMM, prgi: 0, pp: true});
            seq.push({s: "wsa", v: 0x0004, step: 3.5});
            seq.push({s: "awaitsa", what: "SA after the purge/poll fatal"});
            seq.push({s: "delay", n: 400, r: REGS.tmp});
            seq.push({s: "snapsa", what: "SA re-read after the purge/poll fatal"});
            return seq;
        },
        tags: []
    }));

    return cases;
}

/**
 * plfCases()
 *
 * PHASE P -- rq_plf(), the PORT LAST FAILURE datagram.
 *
 * The only path to it: a controller that has gone FATAL, re-initialised by a write to IP, taken
 * back through the four-step handshake, and given a GO word with SA_S4H_LF alongside SA_S4H_GO.
 * rq_quesvc()'s CST_S4 arm then does `if ((cp->saw & SA_S4H_LF) && cp->perr) rq_plf (cp, cp->perr);
 * cp->perr = 0;`.
 *
 * *** rq_plf() HAS NO CF_THS TEST. ***  rq_hbe() and rq_dte() both open with one; this does not.  So
 * the last-failure log arrives on a controller that has never had error logging enabled -- and it
 * cannot have, because the fatal's own rq_reset() put `cflgs` back to CF_RPL.  That asymmetry was
 * misdocumented in rq.js's own header until this item measured it, which is HANDOFF.md standing
 * rule 12 (never let a comment claim more than the code) in its usual shape.
 *
 * THREE CASES, and the two negatives are what make the positive mean anything: GO with LF and a
 * pending error (the datagram), GO WITHOUT LF and a pending error (nothing), and GO with LF and NO
 * pending error (nothing).  All three run the SAME script apart from the GO word and whether a fatal
 * preceded it, and all three end by posting a GET UNIT STATUS and waiting for ITS response -- so the
 * response ring's first slot is snapshotted at a moment when a datagram, had one been produced,
 * would already have taken it.  No delay, no budget.
 */
function plfCases(startIdx)
{
    let cases = [];
    let add = (c) => { c.idx = startIdx + cases.length; cases.push(c); return c; };
    let COMM = 0x2000;
    let S1 = (SA_S1H_VL | (1 << SA_S1H_V_CQ) | (1 << SA_S1H_V_RQ)) & 0xFFFF;

    let plf = (o) => add(build({
        name: o.name, kind: o.kind, ncmds: 1,
        units: [unitSpec({dtype: RD54_DTYPE, image: "err", ro: true})],
        spec: {comm: COMM, cqCode: 1, rqCode: 1, nCmdBuf: 1, nRspBuf: 2, spread: o.spread},
        cmds: () => [{opc: OP.GUS, unit: 0, ref: 0x4000}],
        steps: (g) => {
            let seq = [];
            if (o.fatalFirst) {
                /* Arm `perr` with a purge/poll failure -- the one fatal that needs no DMA, so this
                   case's map is irrelevant to it and the error code (21) is unambiguous. */
                seq.push(...handshake({s1dat: S1, comm: COMM, prgi: 0, pp: true}));
                seq.push({s: "wsa", v: 0x0004, step: 3.5});
                seq.push({s: "awaitsa", what: "SA after the purge/poll fatal"});
                seq.push({s: "snapsa", what: "SA while DEAD"});
                /* *** WRITE IP TO RESTART INITIALISATION. ***  rq_wr()'s IP arm is `rq_reset()` with
                   no test of the value and no test of the state -- and rq_reset() DOES NOT CLEAR
                   `perr`, which is the entire reason the last-failure log has anything to report. */
                seq.push({s: "wip", v: 0});
            }
            seq.push(...handshake({s1dat: S1, comm: COMM, prgi: 0, stopBeforeGo: true}));
            /* *** THE RESPONSE DESCRIPTOR IS GRANTED HERE AND NOWHERE EARLIER. ***  rq_step4() has
               just ZEROED the whole communications region, so a descriptor written before the
               step-3 reply is erased; and it must exist before GO, because rq_plf() runs inside the
               GO write's own queue service. */
            seq.push({s: "clrint"});
            seq.push({s: "rdesc", slot: 0, buf: 0});
            seq.push({s: "wsa", v: o.go, step: 4});
            seq.push({s: "poll", r: 4, prev: 3, cnt: 8});
            seq.push({s: "delay", n: 400, r: REGS.tmp});
            /* A REAL COMMAND, waited on UNBOUNDED.  Its response cannot arrive before a datagram
               the GO word already asked for, so the snapshot below is taken at a moment when the
               presence or absence of that datagram is settled. */
            seq.push({s: "clrint"});
            seq.push({s: "rdesc", slot: 1, buf: 1});
            seq.push({s: "cdesc", slot: 0, pkt: 0});
            seq.push({s: "ip"});
            /* *** WHICH SLOT THE GET UNIT STATUS RESPONSE LANDS IN IS THE CASE'S OWN CLAIM. ***  The
               response ring index advances one slot per packet the controller sends, so a case that
               EXPECTS the last-failure datagram expects it in slot 0 and the GUS answer in slot 1,
               and a case that expects NO datagram expects the GUS answer in slot 0.  The wait is
               placed accordingly -- and because it is UNBOUNDED, a case whose expectation is wrong
               does not silently pass with a weaker assertion: it burns the step budget and is
               reported BY NAME as a machine that never reached its own HALT.  The claim is the
               assertion (HANDOFF.md standing rule 17: no budget, and no tolerance either).  The
               OTHER slot is snapshotted, so "nothing arrived there" is checked as well. */
            seq.push({s: "await", slot: o.expectPlf ? 1 : 0, cmd: 0,
                      what: "the GET UNIT STATUS response"});
            seq.push({s: "delay", n: 400, r: REGS.tmp});
            seq.push({s: "snap", q: (g.rqBa + (o.expectPlf ? 0 : 1) * 4) >>> 0,
                      what: o.expectPlf ? "the last-failure datagram's descriptor"
                                        : "response ring slot 1, which must be UNTOUCHED"});
            seq.push({s: "snapsa", what: "SA at the end"});
            return seq;
        },
        tags: ["GUS"]
    }));

    plf({name: "SA_S4H_LF at GO after a fatal -- the LAST-FAILURE datagram, with logging DISABLED",
         kind: "plf", spread: 59, fatalFirst: true, go: SA_S4H_GO | SA_S4H_LF, expectPlf: true});
    plf({name: "GO WITHOUT SA_S4H_LF after the same fatal -- no last-failure packet",
         kind: "plf-nolf", spread: 61, fatalFirst: true, go: SA_S4H_GO, expectPlf: false});
    plf({name: "SA_S4H_LF at GO with NO pending port error -- no last-failure packet",
         kind: "plf-noerr", spread: 67, fatalFirst: false, go: SA_S4H_GO | SA_S4H_LF, expectPlf: false});

    return cases;
}

/**
 * inflightCases(startIdx, tail)
 *
 * PHASE A -- ABORT and GET COMMAND STATUS against a unit that is HOLDING A PACKET.
 *
 * pcjsvax-346 made that state reachable (only rq_rw() ever sets `cpkt` or fills `pktq`), named the
 * two search arms as out of scope and left them throwing by name.  They are implemented by this item
 * and graded here.
 *
 * *** THE CASES DEPEND ON A SCHEDULE, AND IT IS ARRANGED RATHER THAN HOPED FOR. ***  Both descriptors
 * go into the command ring BEFORE the single IP read, so the queue thread services them one `qtime`
 * apart; and `xtime` is set two orders of magnitude larger than `qtime` so the drive's BOTTOM END --
 * which would clear `cpkt` and make every search arm miss -- lands long after the second command has
 * been processed.  That is a property of the deposited timing registers, identical on both engines,
 * not a race: rq_io_complete() re-schedules at `iostarttime + xtime` and the queue thread at
 * `qtime`, and 100 < 20000 with no wall clock anywhere in between.
 *
 * *** AND EVERY WAIT IS ON THE **LAST** RESPONSE, WITH THE EARLIER ONES SNAPSHOTTED. ***  A second
 * unbounded wait on a descriptor that has ALREADY changed reads the new value as its "previous" and
 * never exits -- the trap tests/mscprwdiff.js hit and documented.  Which slot is last is decided by
 * the schedule above, not by which command the host posted first.
 */
function inflightCases(startIdx, mode)
{
    let cases = [];
    let want = (kind) => (mode === "tail") === (kind === "abo-cpkt");
    let add = (c) => { c.idx = startIdx + cases.length; cases.push(c); return c; };
    let U = unitSpec;
    let UNITS = () => [U({dtype: RD54_DTYPE, image: "err", ro: true}),
                       U({dtype: RD54_DTYPE}),
                       U({dtype: RD54_DTYPE})];
    /* qtime 100 (the default) against xtime 20000: the second command is processed 100 instructions
       after the IP read and the drive's bottom end 20,000 after the first was accepted. */
    let TIMING = {qtime: RQ_QTIME, xtime: 20000};
    let LIVE = {att: 1, onl: 1, wps: 0, wph: 0};
    let RDREF = 0x5A01;

    /**
     * `probes` are the commands posted TOGETHER with the transfer, in order.  `nResp` is how many
     * responses the whole burst produces; the host waits on the LAST and snapshots the rest.
     */
    let inflight = (o) => want(o.kind) && add(build({
        name: o.name, kind: o.kind, ncmds: 2 + o.probes.length,
        units: UNITS(),
        spec: Object.assign({cqCode: 3, rqCode: 3, nCmdBuf: 8, nRspBuf: 8, dataLnt: 4 * PAGE,
                             spread: o.spread}, TIMING),
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x5A00},
            {opc: OP.RD, unit: 0, ref: RDREF, bc: o.bc || 1024, ba: g.dataBase, lbn: 20, st: LIVE}
        ].concat(o.probes.map((p, k) => p(g, k))),
        steps: (g, n) => {
            let seq = handshake({s1dat: (SA_S1H_VL | (3 << SA_S1H_V_CQ) | (3 << SA_S1H_V_RQ)) & 0xFFFF,
                                 comm: 0x2000, prgi: 0});
            command(seq, {pkt: 0, cslot: 0, rslot: 0, tag: "ONL"});
            seq.push({s: "clrint"});
            /* One response descriptor per response the burst will produce, granted before any of it
               starts -- rq_putpkt() would otherwise queue a packet the host cannot receive and the
               queue thread would re-arm itself forever, which the HALT drain runs into. */
            for (let j = 1; j <= o.nResp; j++) seq.push({s: "rdesc", slot: j, buf: j});
            /* ALL the command descriptors, THEN ONE IP READ.  A second IP read would start a second
               poll and the deferral this case depends on would never happen. */
            for (let i = 1; i < n; i++) seq.push({s: "cdesc", slot: i, pkt: i});
            seq.push({s: "ip"});
            seq.push({s: "await", slot: o.nResp, cmd: 1, what: `the LAST of ${o.nResp} responses`});
            for (let j = 1; j < o.nResp; j++) {
                seq.push({s: "snap", q: (g.rqBa + j * 4) >>> 0, what: `response descriptor ${j}`});
            }
            seq.push({s: "delay", n: 600, r: REGS.tmp});
            return seq;
        },
        tags: ["ONL", "the transfer"].concat(o.tags)
    }));

    /* ---- A1. GET COMMAND STATUS, REFERENCE MATCHING.  All four terms of rq_gcs()'s `&&` hold, so
       the host gets GCS_STSL/GCS_STSH from the transfer's WORKING BYTE COUNT.  The transfer's top
       end has issued its disk read and its bottom end has not run, so RW_WBCL is still the count
       rq_rw() copied out of the host's own packet -- which is a fact about the SCHEDULE, and is
       therefore graded against what the ORACLE reported rather than against a number predicted here.
       Response order: the GCS is answered one `qtime` after the IP read and the transfer 20,000
       later, so the GCS takes descriptor 1 and the transfer descriptor 2. ---- */
    inflight({name: "GET COMMAND STATUS against a transfer IN FLIGHT, reference MATCHING",
              kind: "gcs-match", spread: 71, nResp: 2,
              probes: [() => ({opc: OP.GCS, unit: 0, ref: 0x5A02, tref: RDREF, trefh: 0x1234})],
              tags: ["GCS matching the in-flight reference"]});

    /* ---- A2. THE SAME, REFERENCE NOT MATCHING.  The third term of the `&&` fails and the ELSE arm
       ZEROES both status words -- over the host's own command words, which is the observable: a
       controller that skipped the else arm returns the host's data and looks plausible. ---- */
    inflight({name: "GET COMMAND STATUS against a transfer in flight, reference NOT matching",
              kind: "gcs-nomatch", spread: 73, nResp: 2,
              probes: [() => ({opc: OP.GCS, unit: 0, ref: 0x5A03, tref: 0xDEAD, trefh: 0xBEEF})],
              tags: ["GCS with a reference that matches nothing"]});

    /* ---- A3. ABORT, REFERENCE NOT MATCHING.  None of the three search arms fires, `tpkt` stays 0,
       and the answer is a bare ST_SUC with ABO_LNT -- *** ABORT NEVER REPORTS ST_OFL ***.  The
       transfer is untouched and completes normally, which is what distinguishes this from A6. ---- */
    inflight({name: "ABORT against a transfer in flight, reference NOT matching -- nothing cancelled",
              kind: "abo-nomatch", spread: 79, nResp: 2,
              probes: [() => ({opc: OP.ABO, unit: 0, ref: 0x5A04, tref: 0xDEAD, trefh: 0xBEEF})],
              tags: ["ABORT with a reference that matches nothing"]});

    /* ---- A4. ABORT MATCHING THE **HEAD** OF THE UNIT QUEUE.  A second transfer posted to a busy
       drive is DEFERRED onto `uptr->pktq`; the ABORT's SECOND arm unlinks it by hand
       (`uptr->pktq = pak[tpkt].link`, not through rq_deqh()) and answers it with ST_ABO and
       RSP_LNT -- a TWELVE-byte response for a command whose own end packet would have been 32.  The
       in-flight transfer is NOT touched.
       Response order: ST_ABO (deferred transfer), ST_SUC (the ABORT), then ST_SUC (the transfer). ---- */
    inflight({name: "ABORT the HEAD of the unit queue -- ST_ABO for the deferred transfer",
              kind: "abo-qhead", spread: 83, nResp: 3,
              probes: [(g) => ({opc: OP.RD, unit: 0, ref: 0x5A11, bc: 512, ba: g.dataBase + 2 * PAGE,
                                lbn: 30, st: LIVE}),
                       () => ({opc: OP.ABO, unit: 0, ref: 0x5A05, tref: 0x5A11, trefh: 0x1234})],
              tags: ["a second transfer, deferred", "ABORT matching the deferred one"]});

    /* ---- A5. ABORT MATCHING A PACKET **DEEPER IN** THE QUEUE -- rq_abo()'s THIRD arm, the walk.
       It starts at `pak[prv].link`, i.e. at the SECOND entry, because the head is the second arm's
       job; and it reads RSP_REFL where the two arms above read CMD_REFL -- *** THE SAME WORD UNDER
       TWO NAMES ***, which is invisible until a queue is three deep.  The first deferred transfer
       survives and runs afterwards, so this case produces FIVE responses. ---- */
    inflight({name: "ABORT a packet DEEPER in the unit queue -- rq_abo()'s third arm walks the list",
              /* FOUR responses, not five: ST_ABO for the packet that was unlinked, ST_SUC for the
                 ABORT, and then ST_SUC for the in-flight transfer and for the deferred one that
                 SURVIVED.  The aborted packet does not run, so it produces no second answer. */
              kind: "abo-qwalk", spread: 89, nResp: 4,
              probes: [(g) => ({opc: OP.RD, unit: 0, ref: 0x5A21, bc: 512, ba: g.dataBase + 2 * PAGE,
                                lbn: 31, st: LIVE}),
                       (g) => ({opc: OP.RD, unit: 0, ref: 0x5A22, bc: 512, ba: g.dataBase + 3 * PAGE,
                                lbn: 32, st: LIVE}),
                       () => ({opc: OP.ABO, unit: 0, ref: 0x5A06, tref: 0x5A22, trefh: 0x1234})],
              tags: ["deferred #1", "deferred #2", "ABORT matching deferred #2"]});

    /* ---- A6. *** ABORT MATCHING THE IN-FLIGHT PACKET ITSELF, AND IT IS THE LAST CASE IN THE RUN
       FOR A MEASURED REASON. ***  rq_abo()'s FIRST arm clears `uptr->cpkt`, calls `sim_cancel
       (uptr)` and arms the queue thread -- and it does NOT clear `io_complete`, which the drive's
       own top end set when its disk read completed inline.  rq_reset() does not clear it either
       (its per-unit loop is sim_cancel / sim_disk_reset / cnum / flags / uf / cpkt / pktq and
       nothing else), so THE FLAG SURVIVES INTO THE NEXT CASE on both engines and the next transfer
       to that drive would enter rq_svc()'s BOTTOM end holding no packet.
       That is the shipped C on both sides and it would compare equal -- but it makes every later
       case order-dependent, so this case is placed LAST and assertExclusions() FAILS the run if any
       case after it addresses that unit with a transfer.  Named rather than tidied (HANDOFF.md
       standing rule 6). ---- */
    {
        inflight({name: "ABORT the transfer IN FLIGHT -- ST_ABO, the drive cancelled, io_complete left set",
                  kind: "abo-cpkt", spread: 97, nResp: 2,
                  probes: [() => ({opc: OP.ABO, unit: 0, ref: 0x5A07, tref: RDREF, trefh: 0x1234})],
                  tags: ["ABORT matching the in-flight reference"]});
    }

    return cases;
}

/* ------------------------------------------------------------------------------------------- *
 * The randomized phase (HANDOFF.md standing rule 1)                                             *
 * ------------------------------------------------------------------------------------------- */

/**
 * randomCases(n, seed, startIdx)
 *
 * The enumerated cases above are chosen to hit named rungs and named PAIRS of rungs.  These are
 * chosen to hit COMBINATIONS NOBODY THOUGHT OF: an odd address against an LBN one block inside the
 * replacement table against an ERASE against a drive that is read-only.  Every one of them is graded
 * the same way the enumerated ones are -- trippedRungs() computes the set independently and the
 * ORACLE must have answered the first of it -- so a rung ordering that happens to be right for every
 * pair somebody enumerated still fails here.
 *
 * Unit 0 carries the container attached READ ONLY, so RQ_WPH() is TRUE for it and every WRITE and
 * ERASE this phase generates is refused: no randomized case can reach the disk with a write, which
 * is what lets checkContainersUntouched() assert that no container moved at all.
 */
function randomCases(n, seed, startIdx)
{
    let rnd = mulberry32(seed);
    let pick = (a) => a[(rnd() * a.length) | 0];
    let U = unitSpec;
    let cap = capacFor(IMG.err, RD54_DTYPE, true);
    let rcts = DRV_TAB[RD54_DTYPE].rcts;
    let cases = [];
    for (let k = 0; k < n; k++) {
        let unit = pick([0, 0, 0, 0, 1, 2, 9]);
        let opc = pick([OP.RD, OP.RD, OP.ACC, OP.CMP, OP.WR, OP.ERS]);
        let bc = pick([0, 512, 511, 1024, 1023, 2, 0xF0000000, 0xF0000200, 4096]);
        let baOff = pick([0, 1, 2, 256, 257, PAGE, PAGE + 1]);
        let lbn = pick([0, 3, cap - 2, cap - 1, cap, cap + 1, cap + rcts - 1, cap + rcts,
                        cap + rcts + 100, 0x80000000, 0xF0000001]);
        /* The unit's state at the moment the probe runs, DECLARED and then checked by construction:
           if it were wrong the predicted rung would differ from the one the oracle answered and the
           run would FAIL.  Unit 0 is attached read-only and brought ONLINE by the case's first
           command; unit 1 has no container; unit 2 is attached and never onlined. */
        let st = unit === 0 ? {att: 1, onl: 1, wps: 0, wph: 1}
               : unit === 1 ? {att: 0, onl: 0, wps: 0, wph: 0}
               : unit === 2 ? {att: 1, onl: 0, wps: 0, wph: 0}
               : null;
        let c = build({
            name: `random #${k}: ${RQVAX.CMD_NAMES[opc & 0x3F].trim()} unit=${unit} bc=${bc} ` +
                  `lbn=${lbn} ba+${baOff}`,
            kind: "random", ncmds: 2,
            units: [U({dtype: RD54_DTYPE, image: "err", ro: true}),
                    U({dtype: RD54_DTYPE}),
                    U({dtype: RD54_DTYPE, image: "err", ro: true})],
            spec: {spread: 101 + k, dataLnt: 4 * PAGE},
            cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x6000 + k},
                {opc, unit, ref: 0x6100 + k, bc, ba: g.dataBase + baOff, lbn, st}],
            tags: ["ONL", "the random probe"]
        });
        c.idx = startIdx + k;
        cases.push(c);
    }
    return cases;
}

/* ------------------------------------------------------------------------------------------- *
 * Unit setup: the SCP sequence a USER performs, written once for both engines                   *
 * ------------------------------------------------------------------------------------------- */

function simhSetupLines(c)
{
    let L = [];
    for (let i = 0; i < RQ_NUMDR; i++) {
        L.push(`detach rq${i}`);
        L.push(`set rq${i} ${DRV_TAB[RD54_DTYPE].name}`);
        L.push(`set rq${i} writeenabled`);
        L.push(`set rq${i} enabled`);
        L.push(`set rq${i} unit=${PLUG_PARK + i}`);
    }
    for (let i = 0; i < RQ_NUMDR; i++) L.push(`set rq${i} unit=${c.units[i].plug}`);
    for (let i = 0; i < RQ_NUMDR; i++) {
        let u = c.units[i];
        L.push(`set rq${i} ${DRV_TAB[u.dtype].name}`);
        if (u.locked) L.push(`set rq${i} locked`);
        if (u.disabled) L.push(`set rq${i} disabled`);
    }
    return L;
}

function simhAttachLines(c)
{
    let L = [];
    for (let i = 0; i < RQ_NUMDR; i++) {
        let u = c.units[i];
        if (!u.image) continue;
        L.push(`attach ${u.ro ? "-R " : ""}rq${i} ${IMG[u.image].simhPath}`);
    }
    return L;
}

function jsSetupUnits(rq, c)
{
    for (let i = 0; i < RQ_NUMDR; i++) {
        rq.detach(i);
        rq.setType(i, RD54_DTYPE);
        rq.setWriteLock(i, false);
        rq.setEnabled(i, true);
        rq.setPlug(i, PLUG_PARK + i);
    }
    for (let i = 0; i < RQ_NUMDR; i++) rq.setPlug(i, c.units[i].plug);
    for (let i = 0; i < RQ_NUMDR; i++) {
        let u = c.units[i];
        rq.setType(i, u.dtype);
        if (u.locked) rq.setWriteLock(i, true);
        if (u.disabled) rq.setEnabled(i, false);
    }
}

function jsAttachUnits(rq, c, providers)
{
    for (let i = 0; i < RQ_NUMDR; i++) {
        let u = c.units[i];
        if (!u.image) continue;
        rq.attach(i, providers[u.image], {readOnly: u.ro});
    }
}

/* ------------------------------------------------------------------------------------------- *
 * The oracle                                                                                    *
 * ------------------------------------------------------------------------------------------- */

const MARK = "MERRCASE";

function simhCaseLines(c)
{
    let L = [];
    L.push(`echo ${MARK}${c.idx}`);
    L.push(...simhSetupLines(c));
    L.push(...simhResetLines(c));
    L.push(...simhAttachLines(c));
    for (let p of c.dumpPages) L.push(`deposit -l ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)} ${hex(seedFor(p))}`);
    L.push(`deposit -l ${hex(c.resultPage * PAGE)}:${hex(c.resultPage * PAGE + PAGE - 4)} 0`);
    L.push(`deposit -l ${hex(IS_WIN_LO)}:${hex(R_IS - 4)} 0`);
    for (let pr of c.presets) L.push(`deposit -w ${hex(pr.addr)} ${hex(pr.word, 4)}`);
    for (let k = 0; k < c.code.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${c.code[k].toString(16)}`);
    L.push("deposit PSL 0", `deposit PC ${hex(R_CODE)}`);
    L.push(`step ${MAX_STEPS}`);
    L.push(`examine -h ${Array.from({length: OBS_REGS}, (_, k) => "R" + k).join(",")}`);
    L.push("examine -h PC", "examine -h PSL");
    L.push("examine -h qba dser", "examine -h qba mear", "examine -h qba sear",
           "examine -h sysd bto", "examine -h cpu memerr");
    L.push("examine -h rq " + RQ_OBS.map((o) => o.name).join(","));
    for (let o of UNIT_OBS) {
        for (let n = 0; n < RQ_NUMDR; n++) L.push(`examine -h rq ${o.name}[${n}]`);
    }
    for (let i of PKT_PROBES) L.push(`examine -h rq pkts[${i}]`);
    for (let p of c.dumpPages) L.push(`examine -h ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)}`);
    L.push(`examine -h ${hex(c.resultPage * PAGE)}:${hex(c.resultPage * PAGE + PAGE - 4)}`);
    L.push(`examine -h ${hex(IS_WIN_LO)}:${hex(R_IS - 4)}`);
    L.push("echo RINGS", "show rq rings", "echo FREEQ", "show rq freeq", "echo RESPQ", "show rq respq",
           "echo ENDCASE");
    return L;
}

function runCasesSimh(simh, opts, cases)
{
    let L = [`set cpu ${MEM_MB}m`, "set cpu simhalt", "set rq rqdx3",
             "set debug stdout", "set rq debug=REQ"];
    for (let c of cases) L.push(...simhCaseLines(c));
    L.push(...Array.from({length: RQ_NUMDR}, (_, i) => `detach rq${i}`));
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "mscperr-cases.ini"));
    let results = new Array(cases.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = cases.findIndex((c) => c.idx === +parts[i]);
        if (idx < 0) continue;
        results[idx] = parseChunk(parts[i + 1] || "", cases[idx]);
    }
    return results;
}

/**
 * parseChunk(chunk, c)
 *
 * NOT anchored at end of line: `EXAMINE -H PSL` prints the value AND a decoded flag string, and an
 * end-anchored pattern silently matches nothing -- which looks exactly like a do-file that aborted.
 */
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
    for (let [k, n] of [["pc", "PC"], ["psl", "PSL"], ["dser", "DSER"], ["mear", "MEAR"],
                        ["sear", "SEAR"], ["bto", "BTO"], ["memerr", "MEMERR"]]) {
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
    r.units = [];
    for (let n = 0; n < RQ_NUMDR; n++) {
        let u = {};
        for (let o of UNIT_OBS) {
            let m = new RegExp(`^${o.name}\\[${n}\\]:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
            if (!m) return null;
            u[o.name] = parseInt(m[1], 16) >>> 0;
        }
        r.units.push(u);
    }
    r.pkts = [];
    for (let i of PKT_PROBES) {
        let m = new RegExp(`^PKTS\\[${i}\\]:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
        if (!m) return null;
        r.pkts.push(parseInt(m[1], 16) >>> 0);
    }
    /* *** NOT `{6,8}`. ***  SIMH does not zero-pad an EXAMINE address to a fixed width, so a dump of
       low memory prints `1E00: ...` and a six-digit pattern silently drops the whole page -- which
       reads as "the oracle never reported memory there", i.e. as a broken do-file rather than as a
       broken parser.  A register name cannot collide: every RQ_OBS name contains a non-hex letter. */
    r.mem = new Map();
    let re = /^([0-9A-F]{1,8}):\s*([0-9A-F]{8})\s*$/gm, m;
    while ((m = re.exec(chunk)) !== null) r.mem.set(parseInt(m[1], 16) >>> 0, parseInt(m[2], 16) >>> 0);
    /* *** SIMH COLLAPSES CONSECUTIVE IDENTICAL DEBUG LINES *** (scp.c:13836-13900): a run of N+1
       identical lines prints as the first followed by `same as above (N times)`, and the collapse
       line carries NO DEVICE TAG -- so a parser anchored on `RQ REQ:` reads N repeats as one.  It
       matters here: a case that posts three identical refusals produces three identical `cmd=` lines. */
    r.trace = [];
    let tre = /^DBG\(\s*([0-9.]+)\)>\s*(.*?)\s*$/gm, tm;
    while ((tm = tre.exec(chunk)) !== null) {
        let run = /^same as above \((\d+) times?\)$/.exec(tm[2]);
        if (!run) {
            let body = /^RQ\s+REQ:\s*(.*)$/.exec(tm[2]);
            if (!body) return null;
            r.trace.push({t: parseFloat(tm[1]), line: body[1]});
            continue;
        }
        if (!r.trace.length) return null;
        let prev = r.trace[r.trace.length - 1].line;
        for (let k = 0; k < +run[1]; k++) r.trace.push({t: parseFloat(tm[1]), line: prev});
    }
    for (let [k, tag] of [["rings", "RINGS"], ["freeq", "FREEQ"], ["respq", "RESPQ"]]) {
        let mm = new RegExp(`^${tag}\\n([\\s\\S]*?)^(?:RINGS|FREEQ|RESPQ|ENDCASE)$`, "m").exec(chunk);
        r[k] = mm ? mm[1] : null;
        if (r[k] === null) return null;
    }
    r.halted = /HALT instruction/.test(chunk);
    r.atOwnHalt = r.halted && r.pc === c.haltPC;
    return r;
}

/* ------------------------------------------------------------------------------------------- *
 * This engine                                                                                   *
 * ------------------------------------------------------------------------------------------- */

function runCaseJS(c, providers, mutationOpts = {})
{
    let m = machine(mutationOpts);
    let {bus, cpu, cqbic, rq} = m;

    jsSetupUnits(rq, c);
    jsResetForCase(m, c);
    jsAttachUnits(rq, c, providers);

    for (let p of c.dumpPages) {
        let s = seedFor(p);
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) bus.setLong(a >>> 0, s);
    }
    for (let a = c.resultPage * PAGE; a < c.resultPage * PAGE + PAGE; a += 4) bus.setLong(a >>> 0, 0);
    for (let a = IS_WIN_LO; a < R_IS; a += 4) bus.setLong(a >>> 0, 0);
    for (let pr of c.presets) bus.setWord(pr.addr, pr.word);
    for (let k = 0; k < c.code.length; k++) bus.setByte((R_CODE + k) >>> 0, c.code[k]);
    rq.reqLog.length = 0;                                   /* the trace is per-case, like SIMH's chunk */
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
        pc: cpu.regs[15] >>> 0,
        psl: cpu.psl >>> 0,
        dser: cpu.exc.cqDser >>> 0,
        mear: cpu.exc.cqMear >>> 0,
        sear: cqbic.sear >>> 0,
        bto: cpu.exc.sscBto >>> 0,
        memerr: cpu.exc.memErr >>> 0,
        rq: {}, units: [], mem: new Map(), unimplemented,
        halted, atOwnHalt: halted && (cpu.regs[15] >>> 0) === c.haltPC,
        ioComplete: rq.units.map((u) => u.ioComplete ? 1 : 0),
        unitDue: rq.units.map((u) => u.due === null ? 0 : 1)
    };
    for (let o of RQ_OBS) r.rq[o.name] = rqFieldOf(rq, o);
    for (let n = 0; n < RQ_NUMDR; n++) {
        let u = {};
        for (let o of UNIT_OBS) u[o.name] = o.get(rq.units[n]) >>> 0;
        r.units.push(u);
    }
    r.pkts = PKT_PROBES.map((i) => pktWord(rq, i));
    for (let p of c.dumpPages) {
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) r.mem.set(a >>> 0, bus.getLong(a) >>> 0);
    }
    for (let a = c.resultPage * PAGE; a < c.resultPage * PAGE + PAGE; a += 4) {
        r.mem.set(a >>> 0, bus.getLong(a) >>> 0);
    }
    for (let a = IS_WIN_LO; a < R_IS; a += 4) r.mem.set(a >>> 0, bus.getLong(a) >>> 0);
    r.trace = rq.reqLog.slice();
    r.rings = showCtrl(rq, cqbic, "RI");
    r.freeq = showCtrl(rq, cqbic, "FR");
    r.respq = showCtrl(rq, cqbic, "RS");
    sampleHeap();
    return r;
}

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

const DSER_MASK = 0xFF, MEAR_MASK = 0x1FFF, SEAR_MASK = 0x1FFFFF;

function norm(name, v)
{
    if (name === "dser") return v & DSER_MASK;
    if (name === "mear") return v & MEAR_MASK;
    if (name === "sear") return v & SEAR_MASK;
    return v >>> 0;
}

function slotOf(c, addr)
{
    let off = (addr >>> 0) - R_RESULT;
    if (off < 0 || off >= PAGE) return null;
    for (let s of c.slots) if (s.off === off) return s;
    return null;
}

function whereIs(c, pa)
{
    let page = (pa / PAGE) | 0;
    for (let [q, p] of c.qToP) {
        if (p !== page) continue;
        let qa = q * PAGE + (pa % PAGE), g = c.g;
        let rel = (base, lnt, what) => (qa >= base && qa < base + lnt)
            ? `${what}+0x${(qa - base).toString(16)}` : null;
        return "Qbus 0x" + hex(qa, 6) + (c.validQ.has(q) ? "" : " [MAP ENTRY INVALID]") + " [" + (
            rel(g.dataBase, g.dataLnt, "the transfer buffer") ||
            rel(g.rspBase, c.nRspBuf * 64, "a response packet") ||
            rel(g.cmdBase, c.nCmdBuf * 64, "a command packet") ||
            rel(g.rqBa, g.rqLnt, "the response ring") ||
            rel(g.cqBa, g.cqLnt, "the command ring") || "the comm region") + "]";
    }
    return "physical 0x" + hex(pa);
}

/**
 * traceCommands(trace) / traceResponses(trace)
 *
 * The oracle's own DEBUG=REQ stream, parsed back into the two things this file grades against:
 * every command the controller FETCHED (with the fields it fetched), and every packet it SENT.
 * Reading the answers out of the ORACLE's text rather than out of this engine's state is what makes
 * the ladder assertion an assertion about SIMH and not about rq.js.
 */
function traceCommands(trace)
{
    let out = [];
    for (let k = 0; k < trace.length; k++) {
        let m = /^cmd=([0-9A-F]{4})\(\s*\S+\), mod=([0-9A-F]{4}), unit=(\d+), bc=([0-9A-F]{4})([0-9A-F]{4}), ma=([0-9A-F]{4})([0-9A-F]{4}), lbn=([0-9A-F]{4})([0-9A-F]{4})$/
                .exec(trace[k].line);
        if (!m) continue;
        out.push({at: k, opc: parseInt(m[1], 16), mod: parseInt(m[2], 16), unit: +m[3],
                  bc: ((parseInt(m[4], 16) << 16) | parseInt(m[5], 16)) >>> 0,
                  ba: ((parseInt(m[6], 16) << 16) | parseInt(m[7], 16)) >>> 0,
                  lbn: ((parseInt(m[8], 16) << 16) | parseInt(m[9], 16)) >>> 0});
    }
    return out;
}

function traceResponses(trace)
{
    let out = [];
    for (let k = 0; k < trace.length; k++) {
        let m = /^rsp=([0-9A-F]{4}), sts=([0-9A-F]{4})$/.exec(trace[k].line);
        if (m) out.push({at: k, opf: parseInt(m[1], 16), sts: parseInt(m[2], 16)});
    }
    return out;
}

/**
 * gradeLadder(c, s, failures, acc)
 *
 * *** THE ASSERTION THIS WHOLE FILE IS BUILT AROUND. ***
 *
 * For every command a case declared a unit STATE for, compute the SET of rq_rw_valid() rungs whose
 * condition holds -- with trippedRungs(), which never consults rq.js -- and require that THE ORACLE
 * ANSWERED THE FIRST OF THEM.  A command that trips one rung says nothing about the ladder's order;
 * a command that trips three says everything, and a permuted ladder cannot survive one.
 *
 * The pairing is done through the oracle's OWN trace: the k'th `cmd=` line is the k'th command the
 * controller fetched, and its EVERY FIELD is checked against what the case posted before its answer
 * is used -- so a mis-pairing is reported as a mis-pairing rather than silently grading the wrong
 * command's status.  The answer is the first following `rsp=` line whose opcode is
 * `command | OP_END`, which no error-log datagram can be (their "opcode" word is a FORMAT: FM_CNT
 * is 0 and FM_BAD is 1).
 */
function gradeLadder(c, s, failures, acc)
{
    let cmds = traceCommands(s.trace), rsps = traceResponses(s.trace);
    for (let k = 0; k < c.cmdSpecs.length; k++) {
        let cs = c.cmdSpecs[k];
        if (!cs.st) continue;                               /* not a ladder command */
        if (k >= cmds.length) {
            failures.push(`case ${c.idx} "${c.name}": the ORACLE fetched only ${cmds.length} ` +
                `command(s) and this case posted ${c.cmdSpecs.length}; command ${k} never reached ` +
                `the controller, so the ladder rung it was written for is UNGRADED`);
            return;
        }
        let t = cmds[k];
        let want = {opc: cs.opc & 0xFF, unit: cs.unit || 0, bc: (cs.bc || 0) >>> 0,
                    lbn: (cs.lbn || 0) >>> 0};
        if ((t.opc & 0xFF) !== want.opc || t.unit !== want.unit || t.bc !== want.bc ||
            t.lbn !== want.lbn) {
            failures.push(`case ${c.idx} "${c.name}": the ORACLE's ${k}'th fetched command is ` +
                `opc=${hex(t.opc, 4)} unit=${t.unit} bc=${hex(t.bc)} lbn=${hex(t.lbn)} and this ` +
                `case's ${k}'th is opc=${hex(want.opc, 4)} unit=${want.unit} bc=${hex(want.bc)} ` +
                `lbn=${hex(want.lbn)} -- the trace and the case list are out of step and no ladder ` +
                `answer read out of it means anything`);
            return;
        }
        let ans = rsps.find((r) => r.at > t.at && (r.opf & 0xFF) === ((want.opc | OP.END) & 0xFF));
        if (!ans) {
            failures.push(`case ${c.idx} "${c.name}": the ORACLE never answered command ${k} ` +
                `(opc=${hex(want.opc, 4)}) -- there is no rsp= line carrying ` +
                `${hex((want.opc | OP.END) & 0xFF, 2)} after its cmd= line`);
            continue;
        }
        let u = c.units.find((x) => x.plug === want.unit);
        let d = DRV_TAB[u.dtype];
        let capac = u.image ? capacFor(IMG[u.image], u.dtype, u.ro) : d.lbn;
        let tripped = trippedRungs(cs, Object.assign({capac}, cs.st), d);
        let first = tripped[0];
        acc.rungsSeen.add(first);
        if (tripped.length > 1) acc.multiRung++;
        acc.laddered++;

        if (first < RW_VALID_LADDER.length - 1) {
            let expect = RW_VALID_LADDER[first].sts >>> 0;
            if (ans.sts !== expect) {
                failures.push(`case ${c.idx} "${c.name}": command ${k} (${cs.tag || ""} opc=` +
                    `${hex(want.opc, 2)} unit=${want.unit} bc=${hex(want.bc)} lbn=${hex(want.lbn)}) ` +
                    `trips rq_rw_valid() rung(s) [${tripped.join(",")}] -- ` +
                    `${tripped.map((i) => RW_VALID_LADDER[i].name).join(", ")} -- so the FIRST of ` +
                    `them, "${RW_VALID_LADDER[first].name}", must win and the answer must be ` +
                    `0x${hex(expect, 4)}.  THE ORACLE ANSWERED 0x${hex(ans.sts, 4)}` +
                    (tripped.length > 1
                        ? `.  This command trips ${tripped.length} rungs at once, so this is a ` +
                          `statement about the LADDER'S ORDER, not about one rung's status.`
                        : `.`));
            } else if (tripped.length > 1) {
                acc.orderProven.add(`${first}<${tripped.slice(1).join(",")}`);
            }
        } else {
            /* THE ACCEPT ARM.  What follows is rq_svc()'s, not rq_rw_valid()'s, so the status is
               whichever of ITS exits the transfer took -- and the set of those is small, closed and
               derived from the C rather than from what this engine happened to produce. */
            /* rq_svc()'s own exits, PLUS ST_ABO -- which does not come from rq_svc() at all.  An
               ACCEPTED transfer can still be answered by rq_abo(), which takes the packet off the
               drive and ends it with ST_ABO and RSP_LNT before rq_svc() ever reaches it.  Leaving
               it out of this set made PHASE A's own cases fail their ladder check, which is the
               right kind of failure to have had: the set is closed and derived, so a status outside
               it is a real question rather than a tolerance to widen. */
            let ok = [ST.SUC, ST.CMP, ST.ABO, (ST.HST | SB.HST_NXM) >>> 0, (ST.WPR | SB.WPR_SW) >>> 0,
                      (ST.WPR | SB.WPR_HW) >>> 0, (ST.OFL | SB.OFL_NV) >>> 0];
            if (ok.indexOf(ans.sts) < 0) {
                failures.push(`case ${c.idx} "${c.name}": command ${k} trips NO rung of ` +
                    `rq_rw_valid(), so it is ACCEPTED and its answer must be one of rq_svc()'s ` +
                    `exits or ST_ABO (${ok.map((v) => "0x" + hex(v, 4)).join(", ")}).  THE ORACLE ` +
                    `ANSWERED 0x${hex(ans.sts, 4)} -- either the command was refused by a rung this ` +
                    `file does not model, or it was ended by a path this file does not know about`);
            }
            acc.accepted++;
        }
    }
}

function grade(cases, sim, js, failures, acc)
{
    let compared = 0;
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i], j = js[i];
        if (!s) {
            failures.push(`case ${c.idx} "${c.name}": the oracle produced no readable result -- not compared`);
            continue;
        }
        if (j.unimplemented) {
            failures.push(`case ${c.idx} "${c.name}": rq.js reached an UNIMPLEMENTED path -- ${j.unimplemented}`);
            continue;
        }
        if (!s.halted || !j.halted) {
            failures.push(`case ${c.idx} "${c.name}": the machine never stopped ` +
                `(oracle ${s.halted ? "halted" : "ran out of step budget at PC=0x" + hex(s.pc)}, ` +
                `here ${j.halted ? "halted" : "ran out of step budget at PC=0x" + hex(j.pc)}) -- ` +
                `every wait in this file is UNBOUNDED, so this is a controller that never answered`);
            continue;
        }
        if (!s.atOwnHalt || !j.atOwnHalt) {
            failures.push(`case ${c.idx} "${c.name}": did not reach its own HALT at 0x${hex(c.haltPC)} ` +
                `(oracle PC=0x${hex(s.pc)}, here PC=0x${hex(j.pc)})`);
        }
        compared++;
        /* R9..R12 are the wait loops' scratch and hold the LAST wait's count and value; a wait that
           covered a disk operation is not reproducible on the oracle (see assertSchedule()). */
        let banded = c.steps.some((st) => st.s === "await" && st.xfer);
        for (let k = 0; k < OBS_REGS; k++) {
            if (k >= 9 && k <= 12 && banded) continue;
            if (s.regs[k] !== j.regs[k]) {
                failures.push(`case ${c.idx} "${c.name}": R${k} = ${hex(j.regs[k])} here, ${hex(s.regs[k])} on the oracle`);
            }
        }
        for (let f of ["pc", "psl", "dser", "mear", "sear", "bto", "memerr"]) {
            if (norm(f, s[f]) !== norm(f, j[f])) {
                failures.push(`case ${c.idx} "${c.name}": ${f.toUpperCase()} = ${hex(norm(f, j[f]))} here, ` +
                    `${hex(norm(f, s[f]))} on the oracle`);
            }
        }
        for (let o of RQ_OBS) {
            if (s.rq[o.name] !== j.rq[o.name]) {
                failures.push(`case ${c.idx} "${c.name}": RQ ${o.name} = ${hex(j.rq[o.name])} here, ` +
                    `${hex(s.rq[o.name])} on the oracle`);
            }
        }
        for (let n = 0; n < RQ_NUMDR; n++) {
            for (let o of UNIT_OBS) {
                if (s.units[n][o.name] !== j.units[n][o.name]) {
                    failures.push(`case ${c.idx} "${c.name}": RQ ${o.name}[${n}] = ` +
                        `${j.units[n][o.name]} here, ${s.units[n][o.name]} on the oracle ` +
                        `(unit ${n} is ${DRV_TAB[c.units[n].dtype].name}, plug ${c.units[n].plug}, ` +
                        `${c.units[n].image ? "image " + c.units[n].image : "unattached"})`);
                }
            }
        }
        let pktDiffs = 0;
        for (let k = 0; k < PKT_PROBES.length; k++) {
            if (s.pkts[k] !== j.pkts[k] && pktDiffs++ < 4) {
                let n = PKT_PROBES[k], pkt = (n / PKT_WORDS) | 0, fld = n % PKT_WORDS;
                failures.push(`case ${c.idx} "${c.name}": PKTS[${n}] (packet ${pkt} ` +
                    `${fld === 0 ? "link" : "d[" + (fld - 1) + "]"}) = ${hex(j.pkts[k], 4)} here, ` +
                    `${hex(s.pkts[k], 4)} on the oracle`);
            }
        }
        if (pktDiffs > 4) failures.push(`case ${c.idx} "${c.name}": ... and ${pktDiffs - 4} more packet-array differences`);
        /* *** EVERY BYTE OF EVERY PHYSICAL PAGE THE MAP NAMES, WHOLE -- INCLUDING THE PAGES BEHIND
           INVALID ENTRIES. ***  For a REFUSED transfer the assertion is that those pages still carry
           their seeds, which is how "nothing happened" becomes a positive observation; for an NXM
           it is that the page behind the invalid entry was never written. */
        let memDiffs = 0;
        for (let [a, v] of j.mem) {
            let sv = s.mem.get(a);
            if (sv === undefined) {
                failures.push(`case ${c.idx} "${c.name}": the oracle never reported memory at 0x${hex(a)}`);
                break;
            }
            let sl0 = slotOf(c, a);
            if (sl0 && sl0.band) continue;                  /* see assertSchedule() */
            if (sv !== v && memDiffs++ < 5) {
                let sl = slotOf(c, a);
                failures.push(`case ${c.idx} "${c.name}": memory 0x${hex(a)}` +
                    (sl ? ` [${sl.what}]` : ` (${whereIs(c, a)})`) +
                    ` = ${hex(v)} here, ${hex(sv)} on the oracle`);
            }
        }
        if (memDiffs > 5) failures.push(`case ${c.idx} "${c.name}": ... and ${memDiffs - 5} more memory differences`);
        gradeTrace(c, s, j, failures);
        for (let [k, tag] of [["rings", "SHOW RQ RINGS"], ["freeq", "SHOW RQ FREEQ"], ["respq", "SHOW RQ RESPQ"]]) {
            let sv = (s[k] || "").replace(/\r/g, "").trim(), jv = (j[k] || "").trim();
            if (sv !== jv) {
                failures.push(`case ${c.idx} "${c.name}": ${tag} differs.\n    here:   ` +
                    `${JSON.stringify(jv).slice(0, 400)}\n    oracle: ${JSON.stringify(sv).slice(0, 400)}`);
            }
        }
        gradeLadder(c, s, failures, acc);
    }
    return compared;
}

function gradeTrace(c, s, j, failures)
{
    let sl = s.trace.map((e) => e.line), jl = j.trace.map((e) => e.line);
    if (sl.length !== jl.length) {
        failures.push(`case ${c.idx} "${c.name}": the DEBUG=REQ trace has ${jl.length} line(s) here and ` +
            `${sl.length} on the oracle.\n    here:   ${JSON.stringify(jl).slice(0, 600)}\n` +
            `    oracle: ${JSON.stringify(sl).slice(0, 600)}`);
        return;
    }
    for (let k = 0; k < sl.length; k++) {
        if (sl[k] !== jl[k]) {
            failures.push(`case ${c.idx} "${c.name}": DEBUG=REQ trace line ${k} is\n    here:   ` +
                `${JSON.stringify(jl[k])}\n    oracle: ${JSON.stringify(sl[k])}`);
            return;
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Exclusion fences and coverage floors.  Every one FAILS the run; none scales with case count.   *
 * ------------------------------------------------------------------------------------------- */

function assertExclusions(cases, sim, js, failures, acc)
{
    /* Which unit indices are left with a stale `io_complete` by an earlier case, and from which
       case index onward.  See the abo-cpkt case's own comment. */
    let staleFrom = new Map();
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i], j = js[i];
        if (c.s1dat & SA_S1H_VEC) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" supplies an S1 word with a non-zero ` +
                `SA_S1H_VEC; interrupt DELIVERY is pcjsvax-aef's work`);
        }
        if (c.s1dat & SA_S1H_IE) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" supplies an S1 word with SA_S1H_IE set`);
        }
        for (let cs of c.cmdSpecs) {
            /* *** A WRITE WHOSE BUFFER FETCH FAILS ENTIRELY HANGS THE REAL SIMULATOR. ***  rq_svc()'s
               top end issues no disk I/O, takes no callback and never re-schedules the unit, so a
               case that made a write's FIRST page unreachable would wait forever on BOTH engines --
               and every wait in this file is unbounded, so it would burn the step budget rather than
               silently compare two hangs.  Refused from the CASE LIST so it cannot be arranged. */
            if ((cs.opc === OP.WR) && cs.ba !== undefined) {
                let first = ((cs.ba & ~RQ_MAPXFER) / PAGE) | 0;
                if (!c.qToP.has(first) || !c.validQ.has(first)) {
                    failures.push(`exclusion: case ${c.idx} "${c.name}" posts a WRITE whose buffer ` +
                        `STARTS on a Qbus page with no VALID map entry.  rq_svc() would issue no ` +
                        `disk write, take no callback and leave the unit unscheduled forever`);
                }
            }
        }
        /* *** rq_dte()'s FENCE, BY NAME. ***  The disk-transfer error log and its ST_DRV end packet
           hang off sim_disk_rdsect()/wrsect() returning non-SCPE_OK, and no do-file can make the
           oracle do that on the STD-format container a user-supplied file attaches as: a read past
           the end of the file is zero-filled and successful, and a write to a container that could
           only be opened read-only is refused two rungs earlier by rq_rw_valid()'s hardware write
           lock, because sim_disk_attach_ex2() sets UNIT_RO on it.  So there is no oracle for that
           arm, rq.js throws by name, and a case that reached it must FAIL the run rather than be
           graded against an answer this tree invented.  Checked here as its OWN statement and not
           merely as "some unimplemented path", so the reason travels with the failure. */
        if (j && j.unimplemented && /rq_dte|disk transfer returned an I\/O error/.test(j.unimplemented)) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" reached rq_dte() -- the DISK ` +
                `TRANSFER ERROR log.  That arm has no oracle (see this file's header) and is ` +
                `excluded BY NAME; a case that provokes it cannot be graded, only invented.`);
        }
        if (!j || j.unimplemented) continue;
        for (let n = 0; n < RQ_NUMDR; n++) {
            if (j.ioComplete[n] && !staleFrom.has(n)) staleFrom.set(n, i);
            if (j.unitDue[n]) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" ends with unit ${n} still on the ` +
                    `event queue -- a transfer that neither completed nor was cancelled`);
            }
        }
        if (!s) continue;
        /* *** THE ONE NONDETERMINISTIC SCHEDULER IN THIS DEVICE, FENCED BY MEASUREMENT. ***
           rq_tmrsvc() is armed at CST_UP with `sim_activate_after (uptr, 1000000)` -- WALL-CLOCK
           microseconds -- and each tick decrements `hat`, reaching rq_fatal(PE_HAT) at zero.  Every
           case here must complete far below that horizon, and the way to know is that HAT is still
           at HTMO on BOTH engines: rq_getpkt() zeroes it while a command is in flight and
           rq_putpkt() restores it to HTMO the moment `pbsy` returns to zero, so a case sampled at
           rest shows HTMO unless a TICK has been taken.  A case that failed this would be a case
           whose two engines' timers had drifted apart, and nothing else it reports would mean
           anything -- so it fails the run rather than being compared. */
        for (let [who, r] of [["the ORACLE", s.rq], ["THIS ENGINE", j.rq]]) {
            if (r.HAT !== r.HTMO) {
                failures.push(`PE_HAT FENCE: case ${c.idx} "${c.name}" ended with ${who}'s HAT = ` +
                    `${r.HAT} and HTMO = ${r.HTMO}.  rq_tmrsvc()'s WALL-CLOCK timer has ticked ` +
                    `during this case, which is the one scheduler in this device that is not ` +
                    `reproducible; PE_HAT is excluded by name and no case may approach it`);
            }
        }
        acc.hatFenced++;
    }
    /* No case may address a unit with a transfer AFTER another case left that unit's `io_complete`
       set -- rq_reset() does not clear it and the next rq_svc() would enter at the BOTTOM end. */
    for (let [n, from] of staleFrom) {
        acc.staleIoComplete.add(n);
        for (let i = from + 1; i < cases.length; i++) {
            let c = cases[i];
            for (let cs of c.cmdSpecs) {
                let nm = RQVAX.OP_NAME_OF[cs.opc & 0xFF];
                if (!nm || RQVAX.MSCP_XFER_OPS.indexOf(nm) < 0) continue;
                let u = c.units.find((x) => x.plug === (cs.unit || 0));
                if (u && c.units.indexOf(u) === n) {
                    failures.push(`exclusion: case ${cases[from].idx} "${cases[from].name}" leaves ` +
                        `unit ${n}'s io_complete SET (rq_abo()'s sim_cancel does not clear it and ` +
                        `rq_reset() does not either), and case ${c.idx} "${c.name}" then posts a ` +
                        `transfer to that unit -- which would enter rq_svc()'s BOTTOM end holding ` +
                        `no packet.  The cancelling case must be LAST`);
                    break;
                }
            }
        }
    }
}

/**
 * assertSchedule(cases, sim, js, failures, acc)
 *
 * What is left to say about the in-band iteration counts once the ones covering a disk operation
 * are excluded from exact comparison.  Three statements, all of which FAIL the run:
 *   - a wait must be NON-ZERO on both engines (a controller that answered inside the IP read gives
 *     zero, which is the synchronous cheat);
 *   - the ORACLE's wait must be at least this engine's, because this engine models the FLOOR;
 *   - the counts that do NOT cover a disk operation -- every refusal in this file, and every fatal
 *     -- are compared EXACTLY, because nothing wall-clock-driven is between the IP read and the
 *     answer.  That is the majority of this file and it is where its schedule grading lives.
 */
function assertSchedule(cases, sim, js, failures, acc)
{
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i], j = js[i];
        if (!s || !s.mem || !j || !j.mem) continue;
        for (let st of c.steps) {
            if (st.s !== "await" && st.s !== "awaitsa") continue;
            let count = s.mem.get((R_RESULT + st.roff) >>> 0);
            let jcount = j.mem.get((R_RESULT + st.roff) >>> 0);
            if (count === undefined || jcount === undefined) continue;
            acc.waitCounts.add(count);
            if (count > 0) acc.nonZeroWaits++; else acc.zeroWaits++;
            if (!(count > 0)) {
                failures.push(`schedule: case ${c.idx} "${c.name}": the ORACLE answered ${st.what} ` +
                    `on the FIRST poll iteration -- there is no delay to grade, which is what a ` +
                    `controller that answered synchronously inside the register access produces`);
            }
            if (!(jcount > 0)) {
                failures.push(`schedule: case ${c.idx} "${c.name}": THIS ENGINE answered ${st.what} ` +
                    `on the first poll iteration, i.e. synchronously inside the register access`);
            }
            if (st.xfer) {
                acc.bandedWaits++;
                if (count < jcount) {
                    failures.push(`schedule: case ${c.idx} "${c.name}": the ORACLE's wait for ` +
                        `${st.what} was ${count} iteration(s) and this engine's was ${jcount}.  ` +
                        `This engine models the FLOOR (qtime + xtime with no per-chunk re-wait), so ` +
                        `it can never legally exceed the oracle`);
                }
            } else {
                acc.exactWaits++;                           /* compared exactly by grade()'s memory pass */
            }
        }
    }
}

/**
 * portErrorCensus(scope)
 *
 * WHICH PE_ codes pdp11_rq.c can actually produce, DERIVED from the C's own `rq_fatal (cp, PE_x)`
 * call sites rather than from a list here.  Three of the thirteen in pdp11_uqssp.h are "NI" and are
 * never passed to rq_fatal() anywhere; a vendor tree that started using one would change this
 * census and the coverage floor below moves with it instead of silently staying satisfied.
 */
function portErrorCensus(scope)
{
    let src = fs.readFileSync(path.join(scope.dir, "pdp11_rq.c"), "utf8").replace(/\r/g, "");
    let out = new Set(), re = /rq_fatal\s*\(\s*cp\s*,\s*(PE_[A-Z0-9]+)\s*\)/g, m;
    while ((m = re.exec(src)) !== null) out.add(m[1].slice(3));
    return out;
}

/** The port errors this file CANNOT construct, each with the reason, checked against the census
    above so a code that became constructible would show up as an unexplained gap rather than as a
    silently excluded one. */
const PE_EXCLUDED = {
    HAT: "rq_tmrsvc()'s host-access timeout is scheduled with sim_activate_after(1000000) -- " +
         "WALL-CLOCK microseconds, the one nondeterministic scheduler in this device.  Every case " +
         "is fenced below that horizon by assertExclusions()'s HAT == HTMO check.",
    MRE: "the map-register READ error is pcjsvax-5c1's excluded branch (cqmap_rd with MBR outside " +
         "memory) and tests/qdmadiff.js carries a LIVE TRIPWIRE on it -- constructing it here " +
         "would be constructing the case another differential exists to refuse.",
    NSR: "the free packet list running dry is graded by tests/mscpringdiff.js, which owns the " +
         "packet pool; it needs 32 commands with no response slots and is not duplicated here."
};

function coverage(cases, sim, js, failures, acc, opts)
{
    let ok = (i) => sim[i] && js[i] && !js[i].unimplemented && sim[i].halted && js[i].halted &&
                    sim[i].atOwnHalt && js[i].atOwnHalt;
    let byKind = (k) => cases.findIndex((c, i) => ok(i) && c.kind === k);

    /* ---- *** THE LADDER FLOOR, AND ITS COUNT COMES FROM THE C. ***  tests/mscpscope.js sliced
       rq_rw_valid() out of pdp11_rq.c and counted its `return` statements; every one of them must
       have been the FIRST TRIPPED RUNG of at least one graded command. ---- */
    for (let r = 0; r < opts.scope.nLadder; r++) {
        if (!acc.rungsSeen.has(r)) {
            let e = RW_VALID_LADDER[r];
            failures.push(`coverage: rq_rw_valid() branch ${r} ("${e ? e.name : "?"}", status ` +
                `0x${hex(e ? e.sts : 0, 4)}) was never the first tripped rung of any graded ` +
                `command, so that branch of the ladder is UNEXERCISED.  The branch COUNT ` +
                `(${opts.scope.nLadder}) is derived from pdp11_rq.c by tests/mscpscope.js, not ` +
                `written here, so this floor moves with the vendor source`);
        }
    }
    /* ---- *** AND THE FLOOR THAT MAKES THE ONE ABOVE MEAN SOMETHING. ***  A command that trips one
       rung proves the rung's STATUS.  Only a command that trips two or more can observe the ORDER,
       and without at least three of them the ladder could be permuted arbitrarily and every case
       above would still pass. ---- */
    if (acc.multiRung < 3) {
        failures.push(`coverage: only ${acc.multiRung} graded command(s) tripped TWO OR MORE rungs ` +
            `of rq_rw_valid() at once; the floor is 3.  A command that trips exactly one rung says ` +
            `nothing whatever about the ladder's ORDER -- which is the property this differential ` +
            `exists to grade -- and a suite made entirely of them passes with the rungs shuffled`);
    }
    if (acc.orderProven.size < 3) {
        failures.push(`coverage: only ${acc.orderProven.size} distinct rung ORDERINGS were proven ` +
            `(each is "winner<losers"); the floor is 3`);
    }

    /* ---- THE STATUS CENSUS, READ OUT OF THE ORACLE'S OWN TRACE.  Every status word any graded
       case saw, and -- HANDOFF.md standing rule 6 -- every code in the derived tables that NEVER
       reached comparison, REPORTED BY NAME below rather than silently absent. ---- */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let r of traceResponses(sim[i].trace)) {
            acc.stsSeen.add(r.sts);
            acc.opfSeen.add(r.opf);
        }
        for (let e of sim[i].trace) if (/^cmd=/.test(e.line)) acc.cmdLines++;
        if (sim[i].rq.CSTA === CST_DEAD) acc.peSeen.add(sim[i].rq.PERR);
    }
    for (let [v, what] of [
        [ST.SUC, "ST_SUC -- a command that simply worked"],
        [ST.CMP, "ST_CMP -- a COMPARE that found a difference"],
        [ST.ABO, "ST_ABO -- a transfer cancelled by ABORT"],
        [ST.AVL, "ST_AVL, BARE -- an attached unit that is not ONLINE"],
        [ST.OFL | SB.OFL_NV, "ST_OFL|SB_OFL_NV -- the unit exists and has no volume"],
        [ST.HST | SB.HST_OA, "ST_HST|SB_HST_OA -- an odd buffer address"],
        [ST.HST | SB.HST_OC, "ST_HST|SB_HST_OC -- an odd byte count"],
        [ST.HST | SB.HST_NXM, "ST_HST|SB_HST_NXM -- a host bus error during the DMA"],
        [ST.CMD | I.BCNT, "ST_CMD|I_BCNT -- an unreasonable, spiralling or non-512 byte count"],
        [ST.CMD | I.LBN, "ST_CMD|I_LBN -- an LBN past the replacement table, or a write into it"],
        [ST.WPR | SB.WPR_HW, "ST_WPR|SB_WPR_HW -- a hardware-locked unit"],
        [ST.WPR | SB.WPR_SW, "ST_WPR|SB_WPR_SW -- a software-locked unit"],
        [ST.CNT, "ST_CNT -- the controller-error status rq_plf() sends its last-failure log with"]
    ]) {
        if (!acc.stsSeen.has(v >>> 0)) {
            failures.push(`coverage: the oracle never answered ${what} (status 0x${hex(v, 4)}) in ` +
                `any graded case`);
        }
    }

    /* ---- THE PORT-ERROR FLOOR, DERIVED FROM THE C's OWN rq_fatal() CALL SITES.  Every code
       pdp11_rq.c can pass to rq_fatal() must have been produced by a graded case, EXCEPT the three
       named in PE_EXCLUDED -- and a code that is neither produced nor excluded fails the run. ---- */
    let census = portErrorCensus(opts.scope);
    acc.peConstructible = census;
    for (let name of census) {
        if (name in PE_EXCLUDED) continue;
        let v = PE[name];
        if (v === undefined) {
            failures.push(`coverage: pdp11_rq.c calls rq_fatal(cp, PE_${name}) and rq.js's PE table ` +
                `has no such code`);
        } else if (!acc.peSeen.has(v)) {
            failures.push(`coverage: pdp11_rq.c can call rq_fatal(cp, PE_${name}) (=${v}) and no ` +
                `graded case left the ORACLE dead with PERR = ${v}.  That port error is ` +
                `constructible and unexercised, and it is not one of the three excluded by name`);
        }
    }
    /* Every code the ORACLE reported must also be one the C can produce -- the census the other way,
       which catches a case that provoked something nobody understands. */
    for (let v of acc.peSeen) {
        let name = RQVAX.PE_NAME_OF[v];
        if (!name || !census.has(name)) {
            failures.push(`coverage: a graded case left the oracle DEAD with PERR = ${v} ` +
                `(${name ? "PE_" + name : "no PE_ name at all"}), which is not a code any ` +
                `rq_fatal() call site in pdp11_rq.c passes`);
        }
    }

    /* ---- THE ERROR-LOG FLOOR, BOTH WAYS.  rq_hbe()'s "logging disabled" arm returns OK, so the END
       packet is identical with and without CF_THS and the ONLY difference is a DATAGRAM on the
       response ring.  Both must have been seen or the CF_THS test is indistinguishable from a
       constant.  The datagram's "opcode" word is a FORMAT and a FLAG, not an opcode:
       `(FM_BAD << RSP_OPF_V_OPC) | (LF_SNR << RSP_OPF_V_FLG)` == 0x0101 -- COMPUTED, because the
       first version of a sibling's check looked for a literal and reported a packet as missing while
       both engines were producing it. ---- */
    let hbeOpf = ((FM_BAD << 0) | (LF_SNR << 8)) & 0xFFFF;
    let plfOpf = ((FM_CNT << 0) | (LF_SNR << 8)) & 0xFFFF;
    let dgCount = (kind, opf) => {
        let i = byKind(kind);
        if (i < 0) return -1;
        return traceResponses(sim[i].trace).filter((r) => r.opf === opf).length;
    };
    acc.hbeOn = dgCount("nxm-on", hbeOpf);
    acc.hbeOff = dgCount("nxm-off", hbeOpf);
    if (acc.hbeOn <= 0) {
        failures.push(`coverage: the CF_THS case produced ${acc.hbeOn} FM_BAD error-log datagram(s) ` +
            `on the oracle (rsp=${hex(hbeOpf, 4)}); rq_hbe()'s packet-building half is unexercised`);
    }
    if (acc.hbeOff !== 0) {
        failures.push(`coverage: the case WITHOUT CF_THS produced ${acc.hbeOff} error-log ` +
            `datagram(s) on the oracle, so the two NXM cases are not measuring the difference they ` +
            `claim to -- rq_hbe()'s CF_THS test is doing nothing`);
    }

    /* ---- THE LAST-FAILURE FLOOR, all three ways.  rq_plf() has NO CF_THS test, so the datagram
       must appear on a controller whose error logging has never been enabled; and it must NOT appear
       without SA_S4H_LF, nor with SA_S4H_LF and no pending error. ---- */
    acc.plfOn = dgCount("plf", plfOpf);
    acc.plfNoLf = dgCount("plf-nolf", plfOpf);
    acc.plfNoErr = dgCount("plf-noerr", plfOpf);
    if (acc.plfOn !== 1) {
        failures.push(`coverage: the SA_S4H_LF case produced ${acc.plfOn} last-failure datagram(s) ` +
            `on the oracle (rsp=${hex(plfOpf, 4)}) and exactly ONE is required.  rq_plf() has no ` +
            `CF_THS test, so "error logging was off" is not an explanation for a missing one`);
    }
    if (acc.plfNoLf !== 0 || acc.plfNoErr !== 0) {
        failures.push(`coverage: a last-failure datagram appeared without SA_S4H_LF ` +
            `(${acc.plfNoLf}) or with no pending port error (${acc.plfNoErr}); rq_plf()'s two ` +
            `guards are not being measured`);
    }

    /* ---- THE ZERO-BYTE-COUNT FLOOR: rq_svc() answers ST_SUC and issues NO disk operation.  Read
       out of the ORACLE's own trace -- a `sim_disk_` line inside that case means the transfer was
       accepted AND performed, which is a different behaviour with the same status word. ---- */
    let zi = byKind("zerobc");
    if (zi < 0) {
        failures.push(`coverage: the zero-byte-count case never reached comparison`);
    } else {
        let ops = sim[zi].trace.filter((e) => /sim_disk_/.test(e.line)).length;
        acc.zeroBcDiskOps = ops;
        /* The case's LAST command is a real one-block READ, so exactly ONE disk operation is
           expected: the control.  More than one means a zero-count transfer touched the disk. */
        if (ops !== 1) {
            failures.push(`coverage: the zero-byte-count case produced ${ops} disk operation(s) on ` +
                `the oracle and exactly ONE (its control READ) is expected -- a zero-count transfer ` +
                `must be answered ST_SUC without touching the disk at all`);
        }
    }

    /* ---- THE RESIDUAL, COUNTED OUT OF THE ORACLE'S OWN DELIVERED RESPONSE PACKETS.  A response
       packet is DMA'd to `rspEnv(j) + UQ_HDR_OFF`, i.e. to rspBuf(j), so its word `d[w]` lands at
       `rspBuf(j) + 2w` and RW_BCL/RW_BCH are the aligned longword at `rspBuf(j) + 16`.  That is
       BYTES PROCESSED -- `bc - wbc` -- written over the host's own request, and it is read here out
       of the bytes THE ORACLE PUT IN MEMORY rather than out of either engine's packet array (which
       holds only the last packet, packets being returned to the head of the free list and reused). ---- */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        let c = cases[i];
        if (["cmpfail", "nxm-off", "nxm-on"].indexOf(c.kind) < 0) continue;
        let posted = new Set(c.cmdSpecs.map((cs) => (cs.bc || 0) >>> 0));
        for (let j = 0; j < c.nRspBuf; j++) {
            let pa = c.phys(c.g.rspBuf(j) + 16);
            let v = sim[i].mem.get(pa >>> 0);
            if (v === undefined) continue;
            /* A buffer no response reached still carries the page seed; a residual is only a
               residual if it is smaller than something this case actually asked for. */
            let biggest = Math.max(...posted);
            if (v > 0 && v < biggest && !posted.has(v)) acc.partialResiduals++;
        }
    }

    /* ---- THE RESIDUAL FLOOR.  A refusal that reported the WHOLE byte count and a refusal that
       reported none are indistinguishable unless at least one graded case comes back with a
       PARTIAL one.  Read out of the oracle's packet array: RW_BCL of the end packet. ---- */
    if (acc.partialResiduals < 2) {
        failures.push(`coverage: only ${acc.partialResiduals} graded response carried a residual ` +
            `that was neither zero nor the whole requested byte count; the floor is 2.  Without ` +
            `one, "bytes processed" is indistinguishable from a constant`);
    }

    /* ---- Each named behaviour must have been OBSERVED on the oracle at least once. ---- */
    for (let [k, what] of [
        ["ladder-read",       "rq_rw_valid()'s read rungs, one command per rung"],
        ["ladder-write",      "rq_rw_valid()'s write rungs"],
        ["ladder-order",      "READ commands that trip two or more rungs at once"],
        ["ladder-order-write", "WRITE commands that trip two or more rungs at once"],
        ["zerobc",            "a zero byte count"],
        ["cmpfail",           "a COMPARE that found a difference"],
        ["nxm-off",           "a host-buffer NXM with error logging OFF"],
        ["nxm-on",            "the same with CF_THS set"],
        ["fatal-pie",         "a non-sequential packet type (PE_PIE)"],
        ["fatal-ici",         "an unknown connection ID (PE_ICI)"],
        ["fatal-pre",         "a packet READ failure (PE_PRE)"],
        ["fatal-pwe",         "a packet WRITE failure (PE_PWE)"],
        ["fatal-qre",         "a descriptor READ failure (PE_QRE)"],
        ["fatal-qwe",         "the unprogrammed-map fatal (PE_QWE)"],
        ["fatal-ppf",         "a non-zero write in CST_S3_PPA (PE_PPF)"],
        ["plf",               "the last-failure datagram"],
        ["plf-nolf",          "GO without SA_S4H_LF"],
        ["plf-noerr",         "SA_S4H_LF with no pending port error"],
        ["gcs-match",         "GET COMMAND STATUS against a transfer in flight"],
        ["gcs-nomatch",       "the same with a reference that matches nothing"],
        ["abo-nomatch",       "ABORT with a reference that matches nothing"],
        ["abo-qhead",         "ABORT of the head of a unit queue"],
        ["abo-qwalk",         "ABORT of a packet deeper in a unit queue"],
        ["abo-cpkt",          "ABORT of the transfer in flight"],
        ["random",            "the randomized phase"]
    ]) {
        if (byKind(k) < 0) failures.push(`coverage: no graded case exercised ${what}`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11)          *
 *                                                                                                *
 * *** HANDOFF.md STANDING RULE 16: A MUTATION CAN BE CAUGHT BY A CHECK THAT IS ITSELF BROKEN. *** *
 * Every read this file makes was chosen against that.  The destination pages are dumped WHOLE and *
 * compared longword by longword at PHYSICAL addresses, so no comparison splices two Qbus pages;   *
 * the response PACKETS are compared as the bytes the oracle delivered into those same pages; the  *
 * packet array comes from the simulator's own rq_reg[]; the STATUS WORDS the ladder assertion      *
 * grades come out of the ORACLE's DEBUG=REQ text, which no defect in rq.js can influence.  The    *
 * `expect` clauses below name WHICH CASE must catch a mutation, and selfcheck() FAILS the run if  *
 * a different one does -- because a ladder-order mutation caught by a single-rung case would mean *
 * the multi-rung cases are not the arrangement they claim to be.                                  *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* --- *** THE ONE THIS FILE EXISTS FOR. ***  Two rungs of rq_rw_valid() swapped: the ODD BYTE
       COUNT test moved AHEAD of the ODD ADDRESS test.  Both rungs still answer exactly the status
       they always did, every single-rung command in PHASE L still passes, and the ONLY thing that
       changes is what a command that is odd BOTH ways answers.  It must be caught by a multi-rung
       case and by nothing else. --- */
    "rq_rw_valid-ladder-ORDER-permuted-odd-count-before-odd-address": {
        expect: {mustCatch: "ladder-order", mustNotName: "ladder-read"},
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                /* Compose over the original: if it answered the ODD ADDRESS but the count is ALSO
                   odd, report the odd count instead -- which is exactly what the C would do with
                   those two rungs exchanged. */
                if (sts === ((ST.HST | SB.HST_OA) >>> 0) && (this.getp32(pkt, RW_BCL) & 1)) {
                    return (ST.HST | SB.HST_OC) >>> 0;
                }
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },

    /* --- THE WRITE RUNGS HOISTED ABOVE THE RCT TEST.  Same shape, the other end of the ladder: a
       WRITE into the replacement table on a locked drive answers the LOCK instead of I_LBN. --- */
    "rq_rw_valid-ladder-ORDER-permuted-write-locks-before-the-RCT": {
        expect: {mustCatch: "ladder-order-write"},
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                if (sts === ((ST.CMD | I.LBN) >>> 0) && (cmd === OP.WR || cmd === OP.ERS)) {
                    if (u.uf & UF_WPS) return (ST.WPR | SB.WPR_SW) >>> 0;
                    if (this.wph(u)) return (ST.WPR | SB.WPR_HW) >>> 0;
                }
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },

    /* --- ST_V_SUB WRONG.  Every status that carries a SUB-CODE re-encoded with a shift of 4 instead
       of 5.  Perturbed at rq_putr(), which is the ONE place every status word in this device passes
       through, so it reaches the ladder's refusals, rq_svc()'s exits AND the error-log packets --
       rather than at rwValid(), which would leave rq_hbe()'s ST_HST|SB_HST_NXM untouched. --- */
    "ST_V_SUB-is-4-so-every-sub-code-lands-one-bit-low": {
        apply: () => {
            let orig = RQVAX.prototype.putr;
            RQVAX.prototype.putr = function(pkt, cmd, flg, sts, lnt, typ) {
                let base = sts & ((1 << ST.V_SUB) - 1), sub = sts >>> ST.V_SUB;
                return orig.call(this, pkt, cmd, flg, (base | (sub << (ST.V_SUB - 1))) & 0xFFFF, lnt, typ);
            };
            return () => { RQVAX.prototype.putr = orig; };
        }
    },

    /* --- ST_V_INV WRONG.  The invalid-command sub-code field only.  ST_CMD is 1, so the affected
       statuses are exactly rq_rw_valid()'s I_BCNT and I_LBN rungs and rq_mscp()'s I_OPCD. --- */
    "ST_V_INV-is-7-so-every-invalid-command-code-lands-one-bit-low": {
        apply: () => {
            let orig = RQVAX.prototype.putr;
            RQVAX.prototype.putr = function(pkt, cmd, flg, sts, lnt, typ) {
                if ((sts & ((1 << ST.V_SUB) - 1)) === ST.CMD) {
                    let inv = sts >>> ST.V_INV;
                    sts = (ST.CMD | (inv << (ST.V_INV - 1))) & 0xFFFF;
                }
                return orig.call(this, pkt, cmd, flg, sts, lnt, typ);
            };
            return () => { RQVAX.prototype.putr = orig; };
        }
    },

    /* --- THE RCT ALLOWANCE DROPPED.  `lbn >= maxlbn` treated as an error outright, instead of as an
       access to the replacement table that is legal for exactly one block.  Rungs 5 and 6 collapse
       into one answer and the legal RCT read becomes illegal. --- */
    "the-RCT-allowance-is-dropped-so-any-lbn-past-the-disk-is-I_LBN": {
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                if (this.getp32(pkt, RW_LBNL) >= (u.capac >>> 0)) return (ST.CMD | I.LBN) >>> 0;
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },

    /* --- THE WRITE-LOCK CHECKS APPLIED TO READS.  Rungs 9 and 10 live inside `if ((cmd == OP_WR) ||
       (cmd == OP_ERS))`; hoisting them out makes every READ of a locked drive fail.  PHASE L reads
       a hardware-locked drive on purpose, which is what kills this. --- */
    "the-write-lock-rungs-are-applied-to-READS-as-well": {
        expect: {mustCatch: "ladder-write"},
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                if (sts === 0) {
                    if (u.uf & UF_WPS) return (ST.WPR | SB.WPR_SW) >>> 0;
                    if (this.wph(u)) return (ST.WPR | SB.WPR_HW) >>> 0;
                }
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },

    /* --- THE OP_ACC / OP_ERS EXEMPTION FROM THE ODD-ADDRESS RUNG, REMOVED.  Neither touches host
       memory, so neither takes rung 2 -- and the ACCESS command in the multi-rung case is odd BOTH
       ways, so with the exemption gone it answers SB_HST_OA instead of SB_HST_OC. --- */
    "the-odd-address-rung-is-applied-to-OP_ACC-and-OP_ERS-too": {
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                if ((cmd === OP.ACC || cmd === OP.ERS) && (this.pd(pkt, RW_BAL) & 1) &&
                    (u.flags & U_ATT) && (u.flags & U_ONL)) {
                    return (ST.HST | SB.HST_OA) >>> 0;
                }
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },

    /* --- THE RESIDUAL COMPUTED WITH THE WRONG SUBTRAHEND.  `bc - i` instead of `bc - (tbc - t)` is
       the misreading a careful person arrives at, because `t` reads like "transferred" and is in
       fact WHAT WAS NOT TRANSFERRED.  Both nxm arms take their value from the same seam, so this
       perturbs the READ and the WRITE together. --- */
    "the-nxm-residual-uses-t-instead-of-tbc-minus-t": {
        expect: {mustCatch: "nxm-off"},
        apply: () => {
            let orig = RQVAX.prototype.nxmResidual;
            RQVAX.prototype.nxmResidual = function(bc, tbc, t) { return (bc - t) >>> 0; };
            return () => { RQVAX.prototype.nxmResidual = orig; };
        }
    },
    "the-nxm-bus-address-is-not-advanced-by-what-the-DMA-moved": {
        expect: {mustCatch: "nxm-on"},
        apply: () => {
            let orig = RQVAX.prototype.nxmAddr;
            RQVAX.prototype.nxmAddr = function(ba, tbc, t) { return ba >>> 0; };
            return () => { RQVAX.prototype.nxmAddr = orig; };
        }
    },

    /* --- SA_ER NOT SET ON A FATAL.  The controller still dies, `csta` still becomes CST_DEAD and
       `perr` still carries the code -- only the ERROR BIT in SA is missing, which is the one thing
       a host polling the register can see. --- */
    "SA_ER-is-not-set-on-a-fatal": {
        apply: () => {
            let orig = RQVAX.prototype.fatal;
            RQVAX.prototype.fatal = function(err) {
                let r = orig.call(this, err);
                this.sa = this.sa & ~SA_ER & 0xFFFF;
                return r;
            };
            return () => { RQVAX.prototype.fatal = orig; };
        }
    },

    /* --- rq_plf() GATED ON CF_THS LIKE rq_hbe().  The exact misreading this item found written into
       rq.js's own header: rq_hbe() and rq_dte() open with `if ((cp->cflgs & CF_THS) == 0) return
       OK;` and rq_plf() does not.  With the gate added the last-failure log NEVER arrives, because
       the fatal's own rq_reset() has just cleared CF_THS. --- */
    "rq_plf-is-gated-on-CF_THS-the-way-rq_hbe-is": {
        expect: {mustCatch: "plf"},
        apply: () => {
            let orig = RQVAX.prototype.plf;
            RQVAX.prototype.plf = function(err) {
                if ((this.cflgs & CF_THS) === 0) return true;
                return orig.call(this, err);
            };
            return () => { RQVAX.prototype.plf = orig; };
        }
    },

    /* --- THE LAST-FAILURE DATAGRAM'S CONNECTION ID.  rq_plf() ORs UQ_CID_DIAG into UQ_HCTC AFTER
       rq_putr() has already written UQ_CID_MSCP there, so the OR is not a no-op: it turns CID 0 into
       CID 0xFF.  Doing it BEFORE rq_putr() leaves it at 0 -- and the packet is otherwise identical. --- */
    "the-last-fail-datagram-keeps-CID-MSCP-instead-of-CID-DIAG": {
        expect: {mustCatch: "plf"},
        apply: () => {
            let orig = RQVAX.prototype.plf;
            RQVAX.prototype.plf = function(err) {
                /* Intercept the ONE putPkt() this call makes and strip the CID the line above it
                   just ORed in.  Composed over the original -- every other field of the datagram is
                   the shipped one, so the difference is exactly the connection ID. */
                let origPut = this.putPkt;
                this.putPkt = function(p, qt) {
                    this.spd(p, UQ_HCTC, this.pd(p, UQ_HCTC) & ~(UQ_CID_DIAG << UQ_HCTC_V_CID));
                    return origPut.call(this, p, qt);
                };
                try { return orig.call(this, err); } finally { delete this.putPkt; }
            };
            return () => { RQVAX.prototype.plf = orig; };
        }
    },

    /* --- `perr` CLEARED BY THE FATAL RATHER THAN BY THE NEXT GO.  rq_reset() does NOT clear it, and
       that is the ONLY reason a re-initialised controller has anything to report.  Clearing it makes
       the last-failure log silently empty while every register still agrees. --- */
    "the-fatal-clears-perr-so-the-next-GO-has-nothing-to-report": {
        expect: {mustCatch: "plf"},
        apply: () => {
            let orig = RQVAX.prototype.reset;
            RQVAX.prototype.reset = function() { let r = orig.call(this); this.perr = 0; return r; };
            return () => { RQVAX.prototype.reset = orig; };
        }
    },

    /* --- ABORT ANSWERS ITS OWN RESPONSE BEFORE THE PACKET IT CANCELLED.  Both packets still go out,
       with the same statuses and the same lengths; only the ORDER on the response ring changes --
       which is invisible to anything that compares the two responses as a set. --- */
    "ABORT-sends-its-own-response-before-the-packet-it-cancelled": {
        expect: {mustCatch: "abo-qhead"},
        apply: () => {
            let orig = RQVAX.prototype.abo;
            RQVAX.prototype.abo = function(pkt, q) {
                /* Buffer whatever putPkt() calls the SHIPPED rq_abo() makes and replay them in the
                   opposite order.  The original still decides which packets, with which statuses
                   and which lengths; only the sequence on the response ring changes. */
                let sent = [];
                let origPut = this.putPkt;
                this.putPkt = function(p, qt) { sent.push([p, qt]); return true; };
                try { orig.call(this, pkt, q); } finally { delete this.putPkt; }
                let out = true;
                for (let e of sent.reverse()) out = this.putPkt(e[0], e[1]) && out;
                return out;
            };
            return () => { RQVAX.prototype.abo = orig; };
        }
    },

    /* --- ABORT DOES NOT CANCEL THE DRIVE.  `sim_cancel (uptr)` dropped: the packet is still
       answered ST_ABO and `cpkt` is still cleared, but the drive's pending event survives and
       rq_svc() then runs with no current packet -- which is STOP_RQ in the C. --- */
    "ABORT-does-not-cancel-the-drive-s-pending-event": {
        expect: {mustCatch: "abo-cpkt"},
        apply: () => {
            let orig = RQVAX.prototype.abo;
            RQVAX.prototype.abo = function(pkt, q) {
                let due = this.units.map((u) => u.due);
                let r = orig.call(this, pkt, q);
                this.units.forEach((u, k) => { if (u.due === null) u.due = due[k]; });
                return r;
            };
            return () => { RQVAX.prototype.abo = orig; };
        }
    },

    /* --- GET COMMAND STATUS IGNORES THE REFERENCE NUMBER.  The third term of rq_gcs()'s four-term
       `&&` dropped, so a probe naming a DIFFERENT command still reports the in-flight transfer's
       working byte count.  Every other field of the response is identical. --- */
    "GET-COMMAND-STATUS-ignores-the-reference-number": {
        expect: {mustCatch: "gcs-nomatch"},
        apply: () => {
            let orig = RQVAX.prototype.gcs;
            RQVAX.prototype.gcs = function(pkt, q) {
                let u = this.getucb(this.pd(pkt, CMD_UN));
                if (u && u.cpkt) {
                    /* Force the reference to match by copying the in-flight packet's own. */
                    this.putp32(pkt, GCS_REFL, this.getp32(u.cpkt, CMD_REFL));
                }
                return orig.call(this, pkt, q);
            };
            return () => { RQVAX.prototype.gcs = orig; };
        }
    },

    /* --- rq_hbe() BUILDS ITS LOG PACKET EVEN WITH CF_THS CLEAR.  The END packet is unchanged and
       the ONLY difference is an extra datagram on the response ring -- which is precisely the
       difference the two NXM cases exist to measure, so it must die in the one WITHOUT CF_THS. --- */
    "rq_hbe-builds-its-log-packet-even-with-error-logging-OFF": {
        expect: {mustCatch: "nxm-off", mustNotName: "cmpfail"},
        apply: () => {
            let orig = RQVAX.prototype.hbe;
            RQVAX.prototype.hbe = function(u) {
                let saved = this.cflgs;
                this.cflgs = saved | CF_THS;
                try { return orig.call(this, u); } finally { this.cflgs = saved; }
            };
            return () => { RQVAX.prototype.hbe = orig; };
        }
    },

    /* --- A ZERO BYTE COUNT REFUSED INSTEAD OF ACCEPTED.  rq_rw_valid() accepts it and rq_svc()'s
       second test answers ST_SUC; refusing it in the ladder gives a plausible-looking I_BCNT for a
       command the real controller completes. --- */
    "a-zero-byte-count-is-refused-by-the-ladder": {
        expect: {mustCatch: "zerobc"},
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                if (sts === 0 && this.getp32(pkt, RW_BCL) === 0) return (ST.CMD | I.BCNT) >>> 0;
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },

    /* --- THE COMPARE MISMATCH REPORTS THE WHOLE BYTE COUNT INSTEAD OF `bc - i`.  ST_CMP either way;
       only the residual -- which is what tells the host WHERE the two differed -- changes. --- */
    "a-COMPARE-mismatch-reports-no-residual": {
        expect: {mustCatch: "cmpfail"},
        apply: () => {
            let orig = RQVAX.prototype.rwEnd;
            RQVAX.prototype.rwEnd = function(u, flg, sts) {
                if (sts === ST.CMP) this.putp32(u.cpkt, RW_WBCL, 0);
                return orig.call(this, u, flg, sts);
            };
            return () => { RQVAX.prototype.rwEnd = orig; };
        }
    },

    /* --- THE REFUSAL ARM DOES NOT ZERO THE HOST'S BYTE COUNT.  rq_rw()'s failure arm sets
       `d[RW_BCL] = d[RW_BCH] = 0` before building the end packet, so a REFUSED transfer reports ZERO
       bytes processed.  Leaving the host's own count there is invisible in the status word and
       visible in every refused response packet. --- */
    "a-refused-transfer-reports-the-REQUESTED-byte-count-as-processed": {
        expect: {mustCatch: "ladder-read"},
        apply: () => {
            let orig = RQVAX.prototype.rw;
            RQVAX.prototype.rw = function(pkt, q) {
                /* *** PERTURBED **WHILE THE HANDLER RUNS**, NOT AFTER IT (HANDOFF.md standing rule
                   11). ***  The first version of this mutation restored the byte count AFTER
                   rq_rw() returned -- which is after rq_putpkt() has already DMA'd the response to
                   the host, so the bytes the host received were the CORRECT ones and the only trace
                   of the mutation was in the controller's own packet array.  Packets are returned to
                   the HEAD of the free list and immediately reused, so that array holds only the
                   LAST command of a case: the mutation was caught by whichever case happened to END
                   with a refusal, and survived in the case written to kill it.  Undoing the zeroing
                   inside the failure arm's own putPkt() call is the perturbation that reaches the
                   host. */
                let bc = this.getp32(pkt, RW_BCL);
                let origPut = this.putPkt;
                this.putPkt = function(p, qt) {
                    if (p === pkt) this.putp32(p, RW_BCL, bc);
                    return origPut.call(this, p, qt);
                };
                try { return orig.call(this, pkt, q); } finally { delete this.putPkt; }
            };
            return () => { RQVAX.prototype.rw = orig; };
        }
    }
};

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                        *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

/** The oracle's answers do not depend on rq.js, so they are computed ONCE and reused by every
    mutation pass -- keyed on the case parameters, so a mutation that changed the case list would
    re-run the simulator and the key is what says so rather than a comment. */
const ORACLE_CACHE = new Map();

function cached(key, fn) {
    if (!ORACLE_CACHE.has(key)) ORACLE_CACHE.set(key, fn());
    return ORACLE_CACHE.get(key);
}

function openProviders(opts, providers)
{
    for (let im of opts.images) {
        if (im.writable) fs.copyFileSync(im.pristine, im.path);
        /* *** EVERY SCRATCH CONTAINER IS OPENED READ/WRITE, INCLUDING THE ONES NO CASE WRITES. ***
           The provider's writability is not a policy here -- it is a MEASUREMENT the controller
           makes: attach() forces a unit read-only when the container cannot be written, exactly as
           sim_disk_attach_ex2() does, and a read-only unit skips autosize's clamp and comes out a
           DIFFERENT SIZE.  Opening a container read-only "for safety" would therefore silently
           change the unit the guest sees -- and the RCT window this file's whole ladder is measured
           against moves with it.  What keeps the containers untouched is that no graded case
           performs an accepted write to them, and checkContainersUntouched() proves it. */
        providers[im.tag] = fileImageProvider(im.path);
    }
    return providers;
}

function restoreWritable(opts, providers)
{
    for (let im of opts.images) {
        if (!im.writable) continue;
        providers[im.tag].restoreFrom(im.pristine);
    }
}

function assertContainers(opts, failures)
{
    let dirOk = fs.existsSync(opts.scratch);
    let listing = dirOk ? fs.readdirSync(opts.scratch).join(", ") : "(the scratch directory is GONE)";
    for (let im of opts.images) {
        for (let [what, p] of [["pristine", im.pristine], ["this engine's", im.path],
                               ["the oracle's", im.simhPath]]) {
            if (!fs.existsSync(p)) {
                failures.push(`container "${im.tag}" -- ${what} copy -- is MISSING at ${p}.  ` +
                    `Scratch is ${opts.scratch}; it holds: ${listing}.`);
                return false;
            }
            let sz = fs.statSync(p).size;
            if (sz !== im.bytes) {
                failures.push(`container "${im.tag}" -- ${what} copy at ${p} -- is ${sz} bytes and ` +
                    `was created with ${im.bytes}.  A container that changed size changes the ` +
                    `unit's capacity, and this file's entire ladder is measured against that number.`);
                return false;
            }
        }
    }
    return true;
}

function allCases(opts)
{
    let enumerated = enumeratedCases();
    let svc = svcCases(enumerated.length);
    let fatal = fatalCases(enumerated.length + svc.length);
    let plf = plfCases(enumerated.length + svc.length + fatal.length);
    let inflight = inflightCases(enumerated.length + svc.length + fatal.length + plf.length, "main");
    let base = enumerated.concat(svc, fatal, plf, inflight);
    let rnd = randomCases(opts.nRandom, opts.seed, base.length);
    /* *** THE ABORT-IN-FLIGHT CASE IS LAST, AND assertExclusions() CHECKS THAT IT STAYS LAST. ***
       It is the only case that leaves a drive's `io_complete` set, and rq_reset() does not clear
       it -- see that case's own comment. */
    let tail = inflightCases(base.length + rnd.length, "tail");
    return base.concat(rnd, tail);
}

function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [];
    let acc = {waitCounts: new Set(), nonZeroWaits: 0, zeroWaits: 0, bandedWaits: 0, exactWaits: 0,
               rungsSeen: new Set(), multiRung: 0, laddered: 0, accepted: 0,
               orderProven: new Set(), stsSeen: new Set(), opfSeen: new Set(), peSeen: new Set(),
               peConstructible: new Set(), cmdLines: 0, hatFenced: 0,
               staleIoComplete: new Set(), partialResiduals: 0,
               hbeOn: 0, hbeOff: 0, plfOn: 0, plfNoLf: 0, plfNoErr: 0, zeroBcDiskOps: 0,
               imagesMatched: 0, imagesChanged: 0};

    /* ---- PHASE S ---- */
    for (let f of opts.scope.failures) failures.push(f);
    if (RW_VALID_LADDER.length !== opts.scope.nLadder) {
        failures.push(`PHASE S: rq.js's RW_VALID_LADDER has ${RW_VALID_LADDER.length} entries and ` +
            `pdp11_rq.c's rq_rw_valid() has ${opts.scope.nLadder} return branches`);
    }
    report.push(`  PHASE S  ${opts.scope.nOp} OP_, ${opts.scope.nSt} ST_, ${opts.scope.nSb} SB_, ` +
        `${opts.scope.nI} I_, ${opts.scope.nPe} PE_ codes, ${opts.scope.nSwitch} dispatch case(s), ` +
        `${opts.scope.nDrv} drive type(s) and rq_rw_valid()'s ${opts.scope.nLadder} branches\n` +
        `           (${opts.scope.nLadder - 1} refusals + accept) re-derived IN ORDER from ` +
        `${opts.scope.dir}; the "reasonable lbn" rung is ` +
        `${opts.scope.deadRungCommentedOut ? "still COMMENTED OUT, as this port assumes" : "LIVE"}`);

    /* ---- PHASES L / E / F / P / A ---- */
    let all = allCases(opts);
    let sim = cached("cases:" + opts.nRandom + ":" + opts.seed, () => runCasesSimh(simh, opts, all));
    /* Each SIMH invocation starts a NEW simulator, so its static MSC struct starts at the C's global
       zero.  The JS machine is built once and reused (standing rule 14), so the pass boundary is
       where that has to be re-established. */
    machine(mutationOpts).rq.powerUp();
    if (!assertContainers(opts, failures)) return {failures, report, compared: 0, acc, cases: all, sim, js: []};
    restoreWritable(opts, opts.providers);
    let js = all.map((c) => runCaseJS(c, opts.providers, mutationOpts));

    assertExclusions(all, sim, js, failures, acc);
    let compared = grade(all, sim, js, failures, acc);
    assertSchedule(all, sim, js, failures, acc);
    coverage(all, sim, js, failures, acc, opts);
    checkContainersUntouched(opts.images, failures, acc);

    /* The wiring the graded machine is actually holding, asserted rather than assumed. */
    let m = machine(mutationOpts);
    if (m.cpu.qbus !== m.rq) failures.push(`the graded machine's CPU has no Qbus event hook wired to the controller`);
    if (!m.rq.cqbic || !m.rq.cqbic.bus) failures.push(`the graded machine's controller has no CQBIC with a bus`);

    report.push(`  PHASE L  ${acc.laddered} command(s) graded against rq_rw_valid()'s ladder; ` +
        `${acc.rungsSeen.size}/${opts.scope.nLadder} branches were the FIRST tripped rung of one; ` +
        `\n           ${acc.multiRung} command(s) tripped TWO OR MORE rungs at once and proved ` +
        `${acc.orderProven.size} distinct ordering(s); ${acc.accepted} command(s) were accepted`);
    report.push(`  PHASE E  ${acc.partialResiduals} partial residual(s) in delivered response ` +
        `packets, ${acc.zeroBcDiskOps} disk operation(s) in the zero-byte-count case, ` +
        `\n           ${acc.hbeOn} HBE datagram(s) with CF_THS set and ${acc.hbeOff} without`);
    report.push(`  PHASE F  ${[...acc.peSeen].sort((a, b) => a - b).map((v) => "PE_" + (RQVAX.PE_NAME_OF[v] || "?") + "(" + v + ")").join(", ")} ` +
        `left the oracle DEAD;\n           ${[...acc.peConstructible].sort().map((n) => "PE_" + n).join(", ")} ` +
        `are the codes pdp11_rq.c passes to rq_fatal()`);
    report.push(`  PHASE P  ${acc.plfOn} last-failure datagram(s) with SA_S4H_LF and a pending ` +
        `error, ${acc.plfNoLf} without the flag, ${acc.plfNoErr} without an error`);
    report.push(`  PHASE I  the one written container came back ` +
        `${acc.imagesMatched ? "byte-identical between the two engines" : "DIFFERENT"} and ` +
        `${acc.imagesChanged ? "changed from its pristine copy" : "UNCHANGED"}`);
    report.push(`  PHASE A  ABORT/GET COMMAND STATUS against an in-flight packet; ` +
        `${acc.staleIoComplete.size} unit(s) left with io_complete set, by the LAST case only`);
    report.push(`  FENCES   ${acc.hatFenced} case(s) checked to have completed without a tick of ` +
        `rq_tmrsvc()'s WALL-CLOCK timer (HAT == HTMO on both engines)`);
    report.push(`  WAITS    ${acc.waitCounts.size} distinct in-band iteration count(s) ` +
        `(${acc.nonZeroWaits} non-zero, ${acc.zeroWaits} immediate); ${acc.exactWaits} compared ` +
        `EXACTLY and ${acc.bandedWaits} as a band (those covering a disk operation)`);
    report.push(`  TRACE    ${acc.cmdLines} command line(s) and ${acc.stsSeen.size} distinct status ` +
        `word(s) read out of the ORACLE's own DEBUG=REQ stream`);
    report.push(`  PHASE C  ${compared}/${all.length} case(s) compared`);
    return {failures, report, compared, acc, cases: all, sim, js};
}

/**
 * codeCensus(acc, scope)
 *
 * HANDOFF.md standing rule 6: any code that did NOT reach comparison is REPORTED BY NAME rather
 * than silently absent.  One item silently lost 21% of its cases; another found a broken boundary
 * surviving 150,000 operations undetected.  This is the whole derived scope -- 23 ST_, 15 SB_, 7 I_
 * and 13 PE_ codes -- partitioned into what a run actually constructed and what it did not, with
 * the reason where there is one.
 */
function codeCensus(acc, scope)
{
    let lines = [];
    let seenBase = new Set([...acc.stsSeen].map((v) => v & ((1 << ST.V_SUB) - 1)));
    let stMissing = [], stSeen = [];
    for (let n of Object.keys(scope.st)) {
        if (n === "V_SUB" || n === "V_INV") continue;
        (seenBase.has(scope.st[n]) ? stSeen : stMissing).push("ST_" + n);
    }
    lines.push(`  ST_ codes CONSTRUCTED (${stSeen.length}/${stSeen.length + stMissing.length}): ${stSeen.join(" ")}`);
    lines.push(`  ST_ codes NOT REACHED (${stMissing.length}): ${stMissing.join(" ")}`);
    lines.push(`    -- the tape ones (BOT/TMK/RDT/POL/LED) cannot occur on a DISK controller at all;` +
               ` the rest are\n       arms of pdp11_rq.c this device's five commands do not have.`);

    let subSeen = new Set([...acc.stsSeen].map((v) => v >>> ST.V_SUB));
    let sbSeen = [], sbMissing = [];
    for (let n of Object.keys(scope.sb)) {
        (subSeen.has(scope.sb[n] >>> ST.V_SUB) ? sbSeen : sbMissing).push("SB_" + n);
    }
    lines.push(`  SB_ sub-codes CONSTRUCTED (${sbSeen.length}/${sbSeen.length + sbMissing.length}): ${sbSeen.join(" ")}`);
    lines.push(`  SB_ sub-codes NOT REACHED (${sbMissing.length}): ${sbMissing.join(" ")}`);
    lines.push(`    -- SB_HST_PAR and SB_HST_PTE are host-error sub-codes pdp11_rq.c never writes;` +
               ` the rest are tape.`);

    let invSeen = new Set([...acc.stsSeen].filter((v) => (v & ((1 << ST.V_SUB) - 1)) === ST.CMD)
                                          .map((v) => v >>> ST.V_INV));
    let iSeen = [], iMissing = [];
    for (let n of Object.keys(scope.inv)) {
        (invSeen.has(scope.inv[n] >>> ST.V_INV) ? iSeen : iMissing).push("I_" + n);
    }
    lines.push(`  I_ codes CONSTRUCTED (${iSeen.length}/${iSeen.length + iMissing.length}): ${iSeen.join(" ")}`);
    lines.push(`  I_ codes NOT REACHED (${iMissing.length}): ${iMissing.join(" ")}`);
    lines.push(`    -- I_VRSN and I_FMTI share values with I_BCNT and I_LBN, so a run that produced` +
               ` either\n       CANNOT tell them apart; I_FLAG and I_MODF have no call site in pdp11_rq.c.`);

    let peSeen = [], peMissing = [];
    for (let n of Object.keys(scope.pe)) {
        (acc.peSeen.has(scope.pe[n]) ? peSeen : peMissing).push("PE_" + n);
    }
    lines.push(`  PE_ port errors CONSTRUCTED (${peSeen.length}/${peSeen.length + peMissing.length}): ${peSeen.join(" ")}`);
    lines.push(`  PE_ port errors NOT REACHED (${peMissing.length}): ${peMissing.join(" ")}`);
    for (let n of peMissing.map((x) => x.slice(3))) {
        if (n in PE_EXCLUDED) lines.push(`    PE_${n}: ${PE_EXCLUDED[n]}`);
        else if (!acc.peConstructible.has(n)) {
            lines.push(`    PE_${n}: pdp11_rq.c has NO rq_fatal() call site for it (derived from the ` +
                `C on this run), so no host can provoke it.`);
        }
    }
    return lines;
}

function selfcheck(simh, opts)
{
    let survived = [], misplaced = [];
    let names = Object.keys(MUTATIONS);
    for (let name of names) {
        let mut = MUTATIONS[name];
        let restore = mut.apply();
        let failures, cases = [];
        try {
            let r = runPass(simh, opts, {});
            failures = r.failures;
            cases = r.cases;
        } catch (e) {
            failures = [`threw: ${e.message}`];
        } finally {
            restore();
        }
        let kinds = new Set(), idxs = new Set();
        for (let f of failures) {
            let m = /^case (\d+) "/.exec(f);
            if (!m) continue;
            idxs.add(+m[1]);
            let c = cases.find((x) => x.idx === +m[1]);
            if (c) kinds.add(c.kind);
        }
        let where = kinds.size ? [...kinds].sort().join(",") : (failures.length ? "not case-scoped" : "-");
        if (!failures.length) survived.push(name);
        if (mut.expect) {
            if (mut.expect.mustCatch && !kinds.has(mut.expect.mustCatch)) {
                misplaced.push(`${name}: was NOT caught by the "${mut.expect.mustCatch}" case, which ` +
                    `is the case that is supposed to kill it (caught by: ${where})`);
            }
            if (mut.expect.mustNotName && kinds.has(mut.expect.mustNotName)) {
                misplaced.push(`${name}: WAS caught by the "${mut.expect.mustNotName}" case, which ` +
                    `exists to be survived -- so that case is not the arrangement it claims to be`);
            }
        }
        console.log(`  ${failures.length ? "CAUGHT " : "SURVIVED"}  ${name}` +
            (failures.length ? `\n              by: ${where}  (${failures.length} failure(s), first: ` +
                `${failures[0].split("\n")[0].slice(0, 160)})` : ""));
    }
    return {survived, misplaced, total: names.length};
}

function dumpCase(cases, sim, js, n)
{
    let i = cases.findIndex((c) => c.idx === n);
    if (i < 0) { console.log(`--dump ${n}: no such case`); return; }
    let c = cases[i];
    console.log(`\n--- case ${c.idx} "${c.name}" (kind=${c.kind}) ---`);
    console.log(`    comm=0x${c.comm.toString(16)} data=0x${c.g.dataBase.toString(16)}+` +
        `0x${c.g.dataLnt.toString(16)} entries=${c.entries.length} ` +
        `invalid=[${c.entries.filter((e) => !e.valid).map((e) => e.q).join(",")}]`);
    for (let st of c.steps) {
        console.log(`      0x${hex(st.pc)}  ${st.s}${st.what ? " " + st.what : ""}` +
            `${st.slot !== undefined ? " slot=" + st.slot : ""}${st.pkt !== undefined ? " pkt=" + st.pkt : ""}`);
    }
    let sl = sim[i] ? sim[i].trace : [], jl = js[i] ? js[i].trace : [];
    for (let k = 0; k < Math.max(sl.length, jl.length); k++) {
        console.log(`  ${String(k).padStart(3)}  oracle ${sl[k] ? sl[k].line : "(none)"}`);
        console.log(`       here   ${jl[k] ? jl[k].line : "(none)"}`);
    }
    if (sim[i]) {
        console.log(`  oracle RQ: ` + RQ_OBS.map((o) => `${o.name}=${hex(sim[i].rq[o.name], 4)}`).join(" "));
        console.log(`  here   RQ: ` + RQ_OBS.map((o) => `${o.name}=${hex(js[i].rq[o.name], 4)}`).join(" "));
    }
}

function main()
{
    let simh = findSimhBin(getArg("--simh", null));
    let dumpN = getArg("--dump", null);
    let nRandom = +getArg("--cases", RANDOM_CASES_DEFAULT);
    let seed = +getArg("--seed", 20260728);
    let fSelfcheck = process.argv.includes("--selfcheck");

    if (nRandom < RANDOM_CASES_FLOOR) {
        console.error(`mscperrdiff: --cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "mscperrdiff-"));
    let images = null, providers = {}, code = 0;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized ladder probes: ${nRandom}`);

        images = makeImages(scratch);
        for (let im of images) IMG[im.tag] = im;
        console.log(`containers: ${images.map((i) => `${i.tag}=${i.blocks}blk` +
            `${i.writable ? " (a copy per engine)" : ""}`).join(", ")}`);
        console.log(`  GENERATED here and deleted on every exit path -- HANDOFF.md 8: no image is shipped`);

        let opts = {scratch, nRandom, seed, images, providers};
        opts.scope = checkScope(simh);
        openProviders(opts, providers);

        let pass = runPass(simh, opts);
        let {failures, report} = pass;
        if (dumpN !== null) for (let n of String(dumpN).split(",")) dumpCase(pass.cases, pass.sim, pass.js, +n);

        console.log(`\nPHASES`);
        for (let line of report) console.log(line);
        console.log(`\nCODE CENSUS -- what this run CONSTRUCTED and what it did not (standing rule 6)`);
        for (let line of codeCensus(pass.acc, opts.scope)) console.log(line);

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
            console.log(`\nMATCH -- every rejected and failing MSCP transfer answered the oracle's ` +
                `exact status word and left its exact residual: rq_rw_valid()'s ladder graded ONE ` +
                `CASE PER BRANCH with the branch count derived from pdp11_rq.c, the ORDER of the ` +
                `ladder proven by commands that trip two and three rungs at once, rq_svc()'s ` +
                `zero-count / compare-mismatch / host-NXM exits with their residuals, rq_hbe()'s ` +
                `log packet present with CF_THS and absent without, the seven port errors each ` +
                `leaving CST_DEAD and SA_ER|code, rq_plf()'s last-failure datagram, and ABORT and ` +
                `GET COMMAND STATUS against a transfer in flight.`);
        }

        if (fSelfcheck && !code) {
            console.log(`\nPHASE M -- mutations (${Object.keys(MUTATIONS).length})`);
            let {survived, misplaced, total} = selfcheck(simh, opts);
            if (survived.length) {
                console.error(`\nFAIL -- ${survived.length} mutation(s) SURVIVED: ${survived.join(", ")}`);
                code = 1;
            }
            if (misplaced.length) {
                console.error(`\nFAIL -- ${misplaced.length} mutation(s) were caught by the WRONG case:`);
                for (let m of misplaced) console.error(`  ${m}`);
                code = 1;
            }
            if (!code) console.log(`\nall ${total} mutation(s) CAUGHT, and both ladder-ORDER ` +
                `permutations were killed by the multi-rung cases rather than by the single-rung ones`);
        }
        if (!code) console.log("\nOK");
    } finally {
        for (let k of Object.keys(providers)) {
            try { providers[k].close(); } catch (e) { /* best effort */ }
        }
        if (process.argv.includes("--keep-scratch")) {
            console.error(`--keep-scratch: leaving ${scratch} in place for diagnosis`);
        } else {
            try { fs.rmSync(scratch, {recursive: true, force: true}); } catch (e) { /* best effort */ }
        }
    }
    process.exit(code);
}

main();
