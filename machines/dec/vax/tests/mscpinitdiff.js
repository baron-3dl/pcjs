/**
 * @fileoverview Differential test: a host program walks the RQDX3 four-step UQSSP initialisation to
 *               CST_UP -- vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-c2c, the first of pcjsvax-6a5's children.  modules/v2/rq.js graded against the live
 * oracle: an IDENTICAL instruction stream runs on both engines and every observable is compared --
 * the SA values the host reads back, the POLLING ITERATION COUNT at which each answer becomes
 * visible, PC/PSL/R0..R14 at HALT, every byte of the physical pages the comm-region zeroing could
 * have touched, twenty-three of the controller's examinable registers, nine words of its packet
 * array, and the text of `SHOW RQ RINGS` / `FREEQ` / `RESPQ`.
 *
 * The plumbing, the physical layout, the assembler, the machine, the observation vector and the
 * SHOW renderers live in tests/mscpharness.js, shared with tests/mscpringdiff.js.  What stays here
 * is this differential's own argument: its cases, its coverage floors, its exclusion fences and its
 * mutations -- sharing THOSE would let one test's coverage certify another's.
 *
 * EVERYTHING IS GRADED THROUGH REAL CPU EXECUTION.  No case calls a device accessor directly.
 * HANDOFF.md 7 premise 7 is the reason and it bites twice here: SIMH's console CANNOT reach the
 * I/O page at all (`e -p 20001468` answers "Address space exceeded"), so a console-level probe of
 * this device is not merely weaker, it is IMPOSSIBLE -- but a real CPU instruction reaches it fine.
 * That is also why this file needs NO eighth SIMH patch, which was established by execution during
 * pcjsvax-6a5's decomposition rather than by argument.
 *
 * THE CHEAT THIS FILE EXISTS TO KILL
 * -----------------------------------
 * Hard-coding the measured SA sequence 0x0B40 -> 0x1080 -> 0x2000 -> 0x4133 -> 0x0000 as constants
 * and replaying it.  Every one of those values except the first is a function of the HOST's own S1
 * data -- SA_S2C_EC / SA_S3C_EC echo bytes of it, and the ring lengths that decide the zeroed extent
 * are `1 << ((s1dat >> 11) & 7)` and `1 << ((s1dat >> 8) & 7)`, scaled by 4.  So the enumerated
 * cases below use FOUR different s1dat values with four different ring-length code pairs and four
 * different echo-byte pairs, the randomized phase draws more, and the coverage floors FAIL the run
 * if fewer than three distinct (cqCode, rqCode, echoS2, echoS3) tuples ever reached comparison.
 *
 * The second cheat is zeroing the comm region by writing straight to physical memory instead of
 * through the CQBIC scatter-gather map.  The measured PE_QWE fatal proves the real controller cannot
 * do that: with MBR and the map entries left unprogrammed, step 3 goes DEAD with SA = 0x8007,
 * CSTA = 8, PERR = 007.  That is a graded case (`fatal-qwe-*`), the map is deliberately SCATTERED
 * and OUT OF ORDER in every normal case (the coverage floor `scatteredPages` requires at least one
 * graded case whose comm region lands on discontiguous, descending physical pages), and
 * --selfcheck's `comm-zero-bypasses-the-map` mutation is exactly the cheat.
 *
 * THE TIMING IS GRADED IN BAND, BECAUSE OUT OF BAND IS STRUCTURALLY BLIND
 * -----------------------------------------------------------------------
 * A HALT DRAINS SIMH'S EVENT QUEUE.  Measured during decomposition: with a ZERO-instruction delay
 * after the SA write, `show time` still reports 453 and CSTA is ALREADY 2.  So a differential that
 * halts early to observe a pre-event state sees the post-event state and passes while grading
 * nothing about the schedule.  Every case here therefore observes the delay the way a real host
 * does: a five-instruction polling loop that counts its own iterations into a register.  The oracle
 * returns 90 iterations against ITIME 450 and 2 against I4TIME 10 (1 + 5i >= delay), and those
 * COUNTS are compared, not just the values they eventually see.
 *
 * ITIME / I4TIME / QTIME / XTIME are SIMH REGISTERS, examinable and settable.  PHASE T reads the
 * oracle's defaults and fails if they disagree with rq.js's constants, then PINS all four with
 * explicit deposits so nothing later inherits them -- and the `timing-*` cases run with NON-DEFAULT
 * values on both engines, which is what proves the delay is driven by the register rather than by a
 * constant that happens to equal it.
 *
 * WHAT IS DELIBERATELY NOT GRADED, BY NAME (standing rule 6)
 * -----------------------------------------------------------
 *   - MSCP packet processing and the response ring.  rq.js DOES implement them (pcjsvax-0b4) and
 *     tests/mscpringdiff.js grades them; THIS file grades the handshake, and a case here that
 *     reached a command would be grading two things at once.  `assertExclusions()` below FAILS the
 *     run if the ORACLE ever reports PBSY != 0, RESP != 0 or FREE != 1 -- so a case that reached
 *     packet processing on the oracle is a failure, not a silently different program.
 *   - Disk I/O and the twelve unit-bearing MSCP commands.  pcjsvax-f52.  rq.js throws
 *     RQUnimplemented by name rather than inventing an answer.
 *   - Controller INTERRUPT delivery.  rq_init_int() fires only when the host's s1dat has BOTH
 *     SA_S1H_IE and a non-zero SA_S1H_VEC.  Delivery LANDED in pcjsvax-aef and is graded by
 *     tests/mscpintdiff.js, so this is a SCOPE boundary rather than a gap: `assertExclusions()`
 *     still FAILS the run if a graded case supplies BOTH bits, because what this file grades is the
 *     POLLING ITERATION COUNT at which each handshake answer becomes visible and an SCB dispatch
 *     inside those loops would change it (and no SCB handler is installed for the RQ vector here).
 *     The VECTOR COMPUTATION is still exercised HERE and not only there: the randomized cases
 *     program a non-zero SA_S1H_VEC with SA_S1H_IE clear, which sets `dibp->vec` without raising
 *     anything.  It is graded indirectly (s1dat is compared, and vec is a pure function of it);
 *     `examine rq devvec` is what tests/mscpintdiff.js reads directly.
 *   - rq_tmrsvc(), the once-per-second host-access timer.  It is a WALL-CLOCK schedule, not an
 *     instruction count.  `assertExclusions()` FAILS the run if the oracle ever reports HAT != HTMO
 *     at the end of a case, which is what firing would look like.
 *   - The doorbell (cqipc.js) and the CMCTL/KA655/SSC register files.  Not in this machine at all;
 *     tests/dbldiff.js owns the doorbell and now also owns the assertion that the I/O page decodes
 *     EXACTLY these two windows and nothing else.
 *
 *      node machines/dec/vax/tests/mscpinitdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch build)
 *        --cases N         randomized cases (default RANDOM_CASES_DEFAULT; below the fixed floor
 *                           the run FAILS rather than clamping up)
 *        --seed S          PRNG seed, printed on every run so a failure is reproducible
 *        --selfcheck       prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";

import CQBICVAX from "../modules/v2/cqbic.js";
import RQVAX, {
    RQ_BASE, IOLN_RQ, RQDX3_CTYPE, CTLR_TAB, RQUnimplemented,
    CST_S1, CST_S1_WR, CST_S2, CST_S3, CST_S3_PPA, CST_S3_PPB, CST_S4, CST_UP, CST_DEAD, CST_NAMES,
    SA_S1H_VL, SA_S1H_WR, SA_S1H_IE, SA_S1H_VEC, SA_S1H_V_CQ, SA_S1H_M_CQ, SA_S1H_V_RQ, SA_S1H_M_RQ,
    SA_S2H_CLO, SA_S2H_PI, SA_S3H_PP, SA_S3H_CHI, SA_S4H_GO, SA_S4H_LF,
    SA_COMM_QQ, SA_COMM_CI, SA_COMM_MAX,
    PE_QWE, PE_QRE, PE_PPF, RQ_SVER, RQ_NPKTS, RQ_PKT_SIZE_W,
    RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME
} from "../modules/v2/rq.js";
import {
    MEMSIZE, MEM_MB, PAGE, R_SCBB, R_MCHK_HDLR, R_MERR_HDLR, R_CODE, R_KSP, R_IS,
    MAP_MBR, MAP_HI, DATA_BASE, DATA_NPAGE, DATA_HI, LOWMAP_HI, HDLR_NOPS, OBS_REGS,
    RQ_IP, RQ_SA, CQBIC_BASE, CQMAP_BASE, CQMAPSIZE, CQMAP_VLD,
    hex, findSimhBin, runSimh, mulberry32, sampleHeap, peakHeap,
    OPC, Asm, makeMachine, machine, RQ_OBS, rqFieldOf, PKT_WORDS, pktWord,
    showCtrl, physPageFor, seedFor, commExtent, walkScript, emitAction,
    simhResetLines, jsResetForCase
} from "./mscpharness.js";

/** An absolute bound on the instructions any case may execute.  A case that does not HALT within it
    is reported BY NAME rather than compared at whatever PC it happened to reach. */
const MAX_STEPS = 40000;

const RANDOM_CASES_DEFAULT = 24;
const RANDOM_CASES_FLOOR   = 12;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure and NOT scaled by case
    count (rules 4 and 14).  ONE machine is built and reused; the dominant term is its single 16MB
    RAM allocation, plus one Uint8Array per dumped page. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;

/** Nine words of SIMH's flat 16-bit view over the packet array, PROBED rather than dumped whole:
    1056 words per case would dominate the do-file, and the interesting words are the free list's own
    links and the first and last data words.  Grading these is what keeps rq_reset()'s packet
    initialisation from being state nothing reads. */
const PKT_PROBES = [0, 1, PKT_WORDS, PKT_WORDS + 1, 2 * PKT_WORDS, 2 * PKT_WORDS + 1,
                    31 * PKT_WORDS, 31 * PKT_WORDS + 1, 32 * PKT_WORDS - 1];

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

/**
 * qbusPagesFor(spec)
 *
 * Every Qbus page the case can reference: the comm-zeroing extent, plus the command ring's first
 * descriptor (which the CST_UP poll and `SHOW RQ RINGS` read back) and every descriptor slot
 * rq_show_ring walks.  Derived, never listed.
 */
function qbusPagesFor(spec)
{
    let e = commExtent(spec);
    let lo = e.base, hi = e.base + Math.max(e.lnt, 1) - 1;
    let ringHi = spec.comm + e.rqLnt + e.cqLnt - 1;
    if (ringHi > hi) hi = ringHi;
    let pages = new Set();
    for (let a = lo & ~(PAGE - 1); a <= hi; a += PAGE) pages.add((a / PAGE) | 0);
    pages.add(((hi / PAGE) | 0));
    return [...pages].sort((a, b) => a - b);
}

/**
 * buildCase(spec)
 *
 * Turns a declarative spec into a fully resolved case: the map entries, the instruction stream, the
 * physical pages to dump, and the fences the exclusions depend on.
 */
function buildCase(spec)
{
    let c = Object.assign({
        itime: RQ_ITIME, i4time: RQ_ITIME4, qtime: RQ_QTIME, xtime: RQ_XTIME,
        mbr: MAP_MBR, mapEntries: true, spread: 0, s1dat: 0x8000, comm: 0x2000, prgi: 0,
        endsInHandler: false
    }, spec);

    let qpages = qbusPagesFor(c);
    c.entries = c.mapEntries ? qpages.map((q) => ({q, p: physPageFor(q, c.spread)})) : [];
    c.dumpPages = [...new Set(c.entries.map((e) => e.p))].sort((a, b) => a - b);
    /* The map entry indices this case touches, zeroed first on both engines so a previous case's
       leftovers cannot make this one pass or fail by accident. */
    c.zeroIdx = qpages.slice();

    let a = new Asm();
    if (c.mbr !== null) a.movImmAbs(4, c.mbr, (CQBIC_BASE + 4 * 4) >>> 0);       // REG_MBR == 4
    for (let e of c.entries) {
        a.movImmAbs(4, (CQMAP_VLD | e.p) >>> 0, (CQMAP_BASE + e.q * 4) >>> 0);
    }
    for (let act of c.script) emitCase(a, act);
    a.halt();
    c.code = a.b;
    c.haltPC = (R_CODE + c.code.length) >>> 0;
    if (c.code.length > 0x800) throw new Error(`mscpinitdiff: case "${c.name}" code is ${c.code.length} bytes`);
    return c;
}

/**
 * emitCase(a, act)
 *
 * The base vocabulary lives in mscpharness.js; this differential adds none of its own, so the whole
 * job here is turning "the harness does not know this action" into a failure with the action's name
 * in it rather than a silently missing instruction.
 */
function emitCase(a, act)
{
    if (!emitAction(a, act)) throw new Error(`mscpinitdiff: unknown script action "${act.a}"`);
}

/* ------------------------------------------------------------------------------------------- *
 * The enumerated case list                                                                      *
 * ------------------------------------------------------------------------------------------- */

/** FOUR host S1 words, chosen so no two share a ring-length code pair OR an echo-byte pair.  The
    IE bit (0x0080) is clear in every one of them -- see the INTERRUPTS exclusion; assertExclusions()
    re-derives that from the case list rather than trusting this comment. */
const S1DATS = [
    0x8000,     // cq code 0, rq code 0, echoS2 0x80, echoS3 0x00   -- the ROM's own shape
    0x9A55,     // cq code 3, rq code 2, echoS2 0x9A, echoS3 0x55
    0xBB01,     // cq code 7, rq code 3, echoS2 0xBB, echoS3 0x01
    0x8D2A      // cq code 1, rq code 5, echoS2 0x8D, echoS3 0x2A
];

/**
 * reachScript(state)
 *
 * The shortest instruction prefix that leaves the controller in a named state with NO event
 * pending -- which matters, because the HALT instruction itself drains the event queue
 * (vax_cpu.c:2643, see exc.js's HALT handler), so a prefix that halted with a service still armed
 * would observe the state AFTER it rather than the state it named.  Every prefix here therefore
 * ends on a poll that has already seen its answer, or on a delay longer than ITIME.
 */
function reachScript(st)
{
    switch (st) {
    case "CST_S1":     return [{a: "rsa", r: 0}];
    case "CST_S1_WR":  return [{a: "rsa", r: 0},
                               {a: "wsa", v: (SA_S1H_VL | SA_S1H_WR | 0x21) & 0xFFFF, step: 1},
                               {a: "poll", r: 1, prev: 0, cnt: 5}];
    case "CST_S2":     return walkScript(0x8000, 0x2000, 0, {}).slice(0, 3);
    case "CST_S3":     return walkScript(0x8000, 0x2000, 0, {}).slice(0, 5);
    case "CST_S3_PPA": return walkScript(0x8000, 0x2000, 0, {pp: true, ppaDelay: 1200}).slice(0, 7);
    case "CST_S3_PPB": return walkScript(0x8000, 0x2000, 0, {pp: true, ppaDelay: 1200}).slice(0, 9);
    case "CST_S4":     return walkScript(0x8000, 0x2000, 0, {}).slice(0, 7);
    case "CST_UP":     return walkScript(0x8000, 0x2000, 0, {});
    case "CST_DEAD":   return walkScript(0x8000, 0x2000, 0, {stopBeforeGo: true});
    }
    throw new Error(`mscpinitdiff: no prefix reaches ${st}`);
}

function enumeratedCases()
{
    let cases = [], meta = [];
    let add = (spec) => { let c = buildCase(spec); c.idx = cases.length; cases.push(c); meta.push(c); return c; };

    /* ---- NORMAL: four s1dat values, two comm regions (one above 64KB so the step-3 shift is
       exercised), both purge-interrupt settings.  Each one lands its comm region on scattered,
       descending physical pages. ---- */
    for (let i = 0; i < S1DATS.length; i++) {
        for (let comm of [0x2000, 0x0A3000]) {
            for (let prgi of [0, 1]) {
                add({
                    name: `normal s1dat=${hex(S1DATS[i], 4)} comm=${hex(comm, 6)} prgi=${prgi}`,
                    s1dat: S1DATS[i], comm, prgi, spread: i,
                    kind: "normal",
                    script: walkScript(S1DATS[i], comm, prgi, {})
                });
            }
        }
    }

    /* ---- WRAP MODE: SA_S1H_WR with SA_S1H_VL.  The controller enters CST_S1_WR and echoes every
       subsequent SA write forever; three different echoes prove it is an echo and not a constant. ---- */
    add({
        name: "wrap mode -- three echoes",
        s1dat: 0x8000, kind: "wrap",
        script: [
            {a: "rsa", r: 0},
            {a: "wsa", v: (SA_S1H_VL | SA_S1H_WR | 0x0123) & 0xFFFF, step: 1},
            {a: "poll", r: 1, prev: 0, cnt: 5},
            {a: "wsa", v: (SA_S1H_VL | SA_S1H_WR | 0x0456) & 0xFFFF, step: 1},
            {a: "poll", r: 2, prev: 1, cnt: 6},
            {a: "wsa", v: 0x0789, step: 1},                          // NOT valid, NOT wrap -- still echoed
            {a: "poll", r: 3, prev: 2, cnt: 7}
        ]
    });

    /* ---- CST_S1 IGNORES AN INVALID WORD: without SA_S1H_VL the service runs and does nothing, so
       SA never changes.  Graded as "the delay elapsed and SA is still 0x0B40", which a model that
       accepted any word would fail. ---- */
    add({
        name: "step 1 ignores a word without SA_S1H_VL",
        kind: "invalid",
        script: [
            {a: "rsa", r: 0},
            {a: "wsa", v: 0x7FFF, step: 1},
            {a: "delay", n: 1200, r: 11},
            {a: "rsa", r: 1},
            {a: "wsa", v: S1DATS[1], step: 1},
            {a: "poll", r: 2, prev: 1, cnt: 5}
        ]
    });

    /* ---- PURGE / POLL: SA_S3H_PP -> CST_S3_PPA -> a ZERO SA write -> CST_S3_PPB -> an IP **READ**
       completes step 4.  The IP read is the subject: a model that treats IP as a read-only zero
       register produces the right VALUE from it and then never leaves CST_S3_PPB. ---- */
    for (let i = 0; i < 2; i++) {
        add({
            name: `purge/poll s1dat=${hex(S1DATS[i], 4)}`,
            s1dat: S1DATS[i], comm: 0x2000, prgi: i, spread: i + 2, kind: "pp",
            script: walkScript(S1DATS[i], 0x2000, i, {pp: true, ppaDelay: 1200})
                .concat([{a: "wsa", v: SA_S4H_GO, step: 4}, {a: "poll", r: 4, prev: 3, cnt: 8}])
        });
    }

    /* ---- PURGE/POLL FAILURE: a NON-ZERO write in CST_S3_PPA is rq_fatal(PE_PPF), error 21. ---- */
    add({
        name: "purge/poll -- non-zero write in CST_S3_PPA is PE_PPF",
        s1dat: 0x8000, comm: 0x2000, kind: "fatal-ppf",
        script: walkScript(0x8000, 0x2000, 0, {pp: true, ppaBad: 0x0004, ppaDelay: 1200, stopBeforeGo: true})
            .concat([{a: "rsa", r: 4}])
    });

    /* ---- THE MAP IS LOAD-BEARING FOR THE PROTOCOL: with MBR and the entries unprogrammed, step 3's
       transition to step 4 cannot zero the comm region and the controller goes DEAD with PE_QWE.
       Measured on the live oracle: SA = 0x8007, CSTA = 8, PERR = 007, COMM back to 0. ---- */
    for (let i = 0; i < 2; i++) {
        add({
            name: `fatal-qwe -- unprogrammed map, s1dat=${hex(S1DATS[i], 4)}`,
            s1dat: S1DATS[i], comm: 0x2000, mbr: null, mapEntries: false, kind: "fatal-qwe",
            script: walkScript(S1DATS[i], 0x2000, 0, {stopBeforeGo: true}).concat([{a: "rsa", r: 4}])
        });
    }

    /* ---- THE SAME FATAL THROUGH THE POLL PATH: reach CST_UP with a good map, then INVALIDATE the
       command ring's map entry and read IP.  rq_getdesc() fails and the controller goes DEAD with
       PE_QRE (6) -- a different error code down a different call path. ---- */
    {
        let spec = {s1dat: 0x8000, comm: 0x2000, prgi: 0};
        let cqPage = ((0x2000 + ((1 << ((0x8000 >>> SA_S1H_V_RQ) & SA_S1H_M_RQ)) << 2)) / PAGE) | 0;
        add({
            name: "fatal-qre -- CST_UP poll with the command ring's map entry invalidated",
            s1dat: 0x8000, comm: 0x2000, spread: 3, kind: "fatal-qre",
            script: walkScript(0x8000, 0x2000, 0, {}).concat([
                {a: "went", q: cqPage, v: 0},
                {a: "rip", r: 9},
                {a: "delay", n: 400, r: 11},
                {a: "rsa", r: 10}
            ])
        });
    }

    /* ---- THE CST_UP HOST POLL: an IP READ sets `pip` and schedules the queue service at QTIME.
       Two cases, one that waits LESS than QTIME (pip must still be 1, and `SHOW RQ RINGS` says
       "Polling in progress") and one that waits longer (the service ran, found the ring empty, and
       cleared pip).  A model that ignores the IP read passes neither. ---- */
    for (let [label, n] of [["short", 40], ["long", 400]]) {
        add({
            name: `CST_UP host poll -- ${label} wait (QTIME ${RQ_QTIME})`,
            s1dat: S1DATS[1], comm: 0x2000, spread: 1, kind: "poll-" + label,
            script: walkScript(S1DATS[1], 0x2000, 0, {}).concat([
                {a: "rip", r: 9},
                {a: "delay", n, r: 11},
                {a: "rsa", r: 10}
            ])
        });
    }

    /* ---- EVERY ONE OF THE NINE CONTROLLER STATES IS REACHED, AND THEN RE-INITIALISED BY AN IP
       WRITE.  Two cases per state, from the SAME prefix: one that HALTS in the state (so the
       ORACLE'S OWN CSTA is the proof it was reached, checked against the state the case names) and
       one that writes IP there and reads SA back (which must be 0x0B40 in CST_S1 whatever state it
       came from).  The state list is imported from rq.js, not written here (standing rule 5), so a
       state that stops being reachable fails the run instead of being quietly dropped.

       CST_DEAD is reached the only way it can be: by leaving the map unprogrammed, so the step-3
       transition's comm-region DMA fails.  That is why its two cases carry `mbr: null`. ---- */
    for (let st of CST_NAMES) {
        let pre = reachScript(st);
        let dead = st === "CST_DEAD";
        add({
            name: `reach ${st} and halt there`,
            s1dat: 0x8000, comm: 0x2000, spread: 2, kind: "reach", reachState: st,
            mbr: dead ? null : MAP_MBR, mapEntries: !dead,
            script: pre.concat([{a: "rsa", r: 10}])
        });
        add({
            name: `IP write re-initialises from ${st}`,
            s1dat: 0x8000, comm: 0x2000, spread: 2, kind: "ipwrite", reachState: st,
            mbr: dead ? null : MAP_MBR, mapEntries: !dead,
            script: pre.concat([
                {a: "rsa", r: 10},
                {a: "wip", v: 0},
                {a: "rsa", r: 11},
                {a: "rip", r: 12}
            ])
        });
    }

    /* ---- SIM_ACTIVATE IS A NO-OP ON AN ALREADY-ACTIVE UNIT (scp.c).  Two SA writes back to back:
       the second updates `saw` but does NOT re-arm the timer, so the answer arrives on the FIRST
       write's schedule and reflects the SECOND write's data.  Both halves are observable -- the
       iteration count is ~2 instructions short of a full ITIME, and S1DAT is the second word. ---- */
    /* THE GAP BETWEEN THE TWO WRITES IS THE WHOLE CASE, and it is 100 instructions rather than 0 for
       a measured reason: with the writes ADJACENT, re-arming moves the deadline by ONE instruction,
       and a five-instruction polling loop resolves the answer to the same iteration either way --
       `sim_activate-re-arms-an-already-active-unit` SURVIVED the first version of this case for
       exactly that reason.  Closed by walking the boundary deterministically (HANDOFF.md standing
       rule 3), not by adding cases: 100 instructions is 20 loop iterations of separation, and it is
       still well inside ITIME so the unit really is active when the second write lands. */
    add({
        name: "sim_activate does not re-arm an already-active unit",
        s1dat: S1DATS[2], comm: 0x2000, spread: 1, kind: "rearm",
        script: [
            {a: "rsa", r: 0},
            {a: "wsa", v: S1DATS[1], step: 1},
            {a: "delay", n: 100, r: 11},
            {a: "wsa", v: S1DATS[2], step: 1},
            {a: "poll", r: 1, prev: 0, cnt: 5}
        ]
    });

    /* ---- NON-DEFAULT TIMING.  Same walk, ITIME/I4TIME deposited to values that are not the
       defaults on BOTH engines: the iteration counts must move with them, which is what proves the
       schedule is driven by the register rather than by a constant that happens to match. ---- */
    for (let [it, i4] of [[100, 60], [900, 5]]) {
        add({
            name: `timing ITIME=${it} I4TIME=${i4}`,
            s1dat: S1DATS[3], comm: 0x2000, spread: 3, itime: it, i4time: i4, kind: "timing",
            script: walkScript(S1DATS[3], 0x2000, 0, {})
        });
    }

    /* ---- THE WINDOW IS FOUR BYTES WIDE AND ITS NEIGHBOURS STILL BUS-FAULT.  0x20001466 and
       0x2000146C are the two the item names; the addresses in between are the window's own bytes,
       read one lane at a time to prove `pa & 1` selects the lane inside the word and `(pa >> 1) & 1`
       selects the register.  Each probe is its OWN case, because a machine check ends the stream. ---- */
    for (let off of [-4, -2, IOLN_RQ, IOLN_RQ + 2]) {
        add({
            name: `neighbour probe @RQ${off >= 0 ? "+" : ""}${off} (0x${hex((RQ_BASE + off) >>> 0)})`,
            kind: "neighbour", s1dat: 0x8000, comm: 0x2000, endsInHandler: true,
            script: [{a: "rd", addr: (RQ_BASE + off) >>> 0, r: 0}, {a: "rsa", r: 1}]
        });
    }
    add({
        name: "byte lanes across the whole four-byte window",
        kind: "lanes", s1dat: S1DATS[1], comm: 0x2000, spread: 2,
        script: walkScript(S1DATS[1], 0x2000, 0, {}).concat([
            {a: "rb", off: 0, r: 9}, {a: "rb", off: 1, r: 10},
            {a: "rb", off: 2, r: 11}, {a: "rb", off: 3, r: 12}
        ])
    });

    /* ---- AN ALIGNED LONGWORD READ AT THE BASE IS TWO QBUS CYCLES: ReadQb(pa) then ReadQb(pa+2).
       In CST_S3_PPB the FIRST of them completes step 4, so the SECOND reads back the step-4 SA in
       the SAME instruction -- an ordering that a model reading SA first would get backwards. ---- */
    add({
        name: "longword read at the base completes step 4 through its IP half",
        s1dat: 0x8000, comm: 0x2000, spread: 0, kind: "longread",
        script: walkScript(0x8000, 0x2000, 0, {pp: true, ppaDelay: 1200, stopBeforeGo: true})
            .slice(0, 9)                                    // stop just after the zero write + delay
            .concat([{a: "ripl", r: 9}, {a: "rsa", r: 10},
                     {a: "wsa", v: SA_S4H_GO, step: 4}, {a: "poll", r: 4, prev: 3, cnt: 8}])
    });

    /* ---- A BYTE WRITE REACHES rq_wr() WITH THE BYTE, UNSHIFTED.  A byte write of 0x01 to base+3
       therefore sets `saw` to 0x0001, not 0x0100 -- and a byte write to base+1 hits IP and resets
       the controller.  Both look like defects and are the C. ---- */
    add({
        name: "byte writes: base+3 sets SAW to the byte, base+1 resets through IP",
        s1dat: 0x8000, comm: 0x2000, spread: 1, kind: "bytewrite",
        script: [
            {a: "rsa", r: 0},
            {a: "wb", off: 3, v: 0x01},
            {a: "delay", n: 1200, r: 11},
            {a: "rsa", r: 1},
            {a: "wsa", v: S1DATS[0], step: 1},
            {a: "poll", r: 2, prev: 1, cnt: 5},
            {a: "wb", off: 1, v: 0xFF},
            {a: "rsa", r: 3}
        ]
    });

    /* ---- AN IP WRITE CANCELS A PENDING SERVICE (rq_reset -> sim_cancel).  Write SA, then write IP
       well before ITIME elapses, then wait past it: the controller must still be in CST_S1 with
       SA = 0x0B40, not stepped to CST_S2. ---- */
    add({
        name: "an IP write cancels the pending queue service",
        s1dat: 0x8000, comm: 0x2000, kind: "cancel",
        script: [
            {a: "rsa", r: 0},
            {a: "wsa", v: S1DATS[0], step: 1},
            {a: "delay", n: 20, r: 11},
            {a: "wip", v: 0},
            {a: "delay", n: 1500, r: 10},
            {a: "rsa", r: 1}
        ]
    });

    return {cases, meta};
}

/**
 * randomCases(n, seed)
 *
 * A structurally different view from the enumerated matrix, which is exhaustive at named boundaries
 * and blind between them: uniform draws over the s1dat word (with SA_S1H_VL forced and SA_S1H_IE
 * forced CLEAR -- see the INTERRUPTS exclusion), the comm region, the purge-interrupt bit, the
 * purge/poll path, the map scatter, and ITIME/I4TIME.
 */
function randomCases(n, seed, startIdx)
{
    let rnd = mulberry32(seed);
    let out = [];
    for (let i = 0; i < n; i++) {
        let s1dat = (SA_S1H_VL |
                     ((Math.floor(rnd() * 8) & SA_S1H_M_CQ) << SA_S1H_V_CQ) |
                     ((Math.floor(rnd() * 8) & SA_S1H_M_RQ) << SA_S1H_V_RQ) |
                     (Math.floor(rnd() * 0x80) & SA_S1H_VEC)) & 0xFFFF;
        /* Word-aligned, and drawn across the WHOLE 22-bit Qbus address space rather than the low
           64KB: the step-3 word carries `comm >> 16`, so a pool that never went above 0xFFFF would
           leave the shift exercised only by the two enumerated `comm=0A3000` cases and blind in the
           randomized phase.  The Qbus address is not a physical one -- physPageFor() decides where
           each page LANDS -- so a high comm cannot collide with the code or the map. */
        let comm = (((Math.floor(rnd() * 0x1000) * PAGE) + (Math.floor(rnd() * 4) * 2)) & 0x3FFFFE) >>> 0;
        let prgi = rnd() < 0.5 ? 1 : 0;
        let pp = rnd() < 0.4;
        let itime = [RQ_ITIME, 60, 250, 700][Math.floor(rnd() * 4)];
        let i4time = [RQ_ITIME4, 7, 35][Math.floor(rnd() * 3)];
        let spread = Math.floor(rnd() * DATA_NPAGE);
        let spec = {
            name: `random#${i} s1dat=${hex(s1dat, 4)} comm=${hex(comm, 6)} prgi=${prgi} pp=${pp ? 1 : 0} ` +
                  `ITIME=${itime} I4TIME=${i4time}`,
            s1dat, comm, prgi, spread, itime, i4time, kind: "random",
            script: walkScript(s1dat, comm, prgi, pp ? {pp: true, ppaDelay: itime + 200} : {})
        };
        let c = buildCase(spec);
        c.idx = startIdx + out.length;
        out.push(c);
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side -- ONE invocation for the whole case list                                       *
 * ------------------------------------------------------------------------------------------- */

const MARK = "MSCASE";

function simhCaseLines(c)
{
    let L = [];
    L.push(`echo ${MARK}${c.idx}`);
    L.push(...simhResetLines(c));
    for (let p of c.dumpPages) L.push(`deposit -l ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)} ${hex(seedFor(p))}`);
    for (let k = 0; k < c.code.length; k++) L.push(`deposit -b ${hex(R_CODE + k)} ${c.code[k].toString(16)}`);
    L.push("deposit PSL 0", `deposit PC ${hex(R_CODE)}`);
    L.push(`step ${MAX_STEPS}`);
    L.push(`examine -h ${Array.from({length: OBS_REGS}, (_, k) => "R" + k).join(",")}`);
    L.push("examine -h PC", "examine -h PSL");
    L.push("examine -h qba dser", "examine -h qba mear", "examine -h qba sear",
           "examine -h sysd bto", "examine -h cpu memerr");
    L.push("examine -h rq " + RQ_OBS.map((o) => o.name).join(","));
    for (let i of PKT_PROBES) L.push(`examine -h rq pkts[${i}]`);
    for (let p of c.dumpPages) L.push(`examine -h ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)}`);
    L.push("echo RINGS", "show rq rings", "echo FREEQ", "show rq freeq", "echo RESPQ", "show rq respq",
           "echo ENDCASE");
    return L;
}

function runCasesSimh(simh, opts, cases)
{
    let L = [`set cpu ${MEM_MB}m`, "set cpu simhalt", "set rq rqdx3"];
    for (let c of cases) L.push(...simhCaseLines(c));
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "mscpinit-cases.ini"));

    let results = new Array(cases.length).fill(null);
    let parts = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
    for (let i = 1; i < parts.length; i += 2) {
        let idx = cases.findIndex((c) => c.idx === +parts[i]);
        if (idx < 0) continue;
        results[idx] = parseChunk(parts[i + 1] || "", cases[idx]);
    }
    return results;
}

function parseChunk(chunk, c)
{
    /* NOT anchored at end of line: `EXAMINE -H PSL` prints the value AND a decoded flag string
       ("00000009\tCM0 TP0 FPD0 IS0 CURMOD=K ... C1"), and an end-anchored pattern silently matched
       nothing for every case -- which looked exactly like a do-file that had aborted. */
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
    r.pkts = [];
    for (let i of PKT_PROBES) {
        let m = new RegExp(`^PKTS\\[${i}\\]:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
        if (!m) return null;
        r.pkts.push(parseInt(m[1], 16) >>> 0);
    }
    /* The memory dump: every `ADDR: VALUE` line whose address falls in a dumped page, keyed by
       address so a missing or reordered line is a miss rather than a shift. */
    r.mem = new Map();
    let re = /^([0-9A-F]{6,8}):\s*([0-9A-F]{8})\s*$/gm, m;
    while ((m = re.exec(chunk)) !== null) r.mem.set(parseInt(m[1], 16) >>> 0, parseInt(m[2], 16) >>> 0);
    for (let [k, tag] of [["rings", "RINGS"], ["freeq", "FREEQ"], ["respq", "RESPQ"]]) {
        let mm = new RegExp(`^${tag}\\n([\\s\\S]*?)^(?:RINGS|FREEQ|RESPQ|ENDCASE)$`, "m").exec(chunk);
        r[k] = mm ? mm[1] : null;
        if (r[k] === null) return null;
    }
    /* SCP prints the stop reason before the register dump; a case that did not HALT is reported by
       name rather than compared at whatever PC the step budget left it at. */
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

    /* SIMH's `reset -p all`, term for term -- and NOTHING MORE.  rq.reset() is rq_reset(), which
       leaves `perr`, `saw`, `prgi` and the rings' `ioff` alone; they are static-struct fields on the
       oracle and carry from case to case there, so clearing them here would diverge on the second
       case that looked at one.  The fresh-SIMH-process state is rq.powerUp(), called ONCE per pass
       by runPass() -- which is the boundary the oracle actually has, since every pass writes a new
       do-file and starts a new simulator. */
    jsResetForCase(m, c);
    for (let p of c.dumpPages) {
        let s = seedFor(p);
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) bus.setLong(a >>> 0, s);
    }
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
        pc: cpu.regs[15] >>> 0,
        psl: cpu.psl >>> 0,
        dser: cpu.exc.cqDser >>> 0,
        mear: cpu.exc.cqMear >>> 0,
        sear: cqbic.sear >>> 0,
        bto: cpu.exc.sscBto >>> 0,
        memerr: cpu.exc.memErr >>> 0,
        rq: {}, mem: new Map(), unimplemented,
        halted, atOwnHalt: halted && (cpu.regs[15] >>> 0) === c.haltPC
    };
    for (let o of RQ_OBS) r.rq[o.name] = rqFieldOf(rq, o);
    r.pkts = PKT_PROBES.map((i) => pktWord(rq, i));
    for (let p of c.dumpPages) {
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) r.mem.set(a >>> 0, bus.getLong(a) >>> 0);
    }
    r.rings = showCtrl(rq, cqbic, "RI");
    r.freeq = showCtrl(rq, cqbic, "FR");
    r.respq = showCtrl(rq, cqbic, "RS");
    sampleHeap();
    return r;
}

/* ------------------------------------------------------------------------------------------- *
 * Grading                                                                                       *
 * ------------------------------------------------------------------------------------------- */

/** SCP's own masks for the CQBIC latches, so a difference is never an artifact of what the oracle
    prints.  DSER/MEAR/SEAR are graded exhaustively by cqmerrdiff.js/qdmadiff.js; here they are
    corroborating evidence that a map failure fired, not the subject. */
const DSER_MASK = 0xFF, MEAR_MASK = 0x1FFF, SEAR_MASK = 0x1FFFFF;

function norm(name, v)
{
    if (name === "dser") return v & DSER_MASK;
    if (name === "mear") return v & MEAR_MASK;
    if (name === "sear") return v & SEAR_MASK;
    return v >>> 0;
}

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
            failures.push(`case ${c.idx} "${c.name}": rq.js reached an UNIMPLEMENTED path -- ${j.unimplemented}`);
            continue;
        }
        if (!s.halted || !j.halted) {
            failures.push(`case ${c.idx} "${c.name}": the machine never stopped ` +
                `(oracle ${s.halted ? "halted" : "ran out of step budget at PC=0x" + hex(s.pc)}, ` +
                `here ${j.halted ? "halted" : "ran out of step budget at PC=0x" + hex(j.pc)}) -- not compared`);
            continue;
        }
        /* A case whose stream ENDS in a machine-check handler is supposed to stop there: the
           handler page is 16 NOPs followed by zeroed memory, and opcode 0x00 IS HALT, so both
           engines stop at the same address inside it -- which grade() then compares as PC like any
           other observable.  `endsInHandler` says which cases those are, so a case that was meant
           to run to its own HALT and did not is still a failure rather than a shrug. */
        if (!c.endsInHandler && (!s.atOwnHalt || !j.atOwnHalt)) {
            failures.push(`case ${c.idx} "${c.name}": did not reach its own HALT at 0x${hex(c.haltPC)} ` +
                `(oracle PC=0x${hex(s.pc)}, here PC=0x${hex(j.pc)}) -- the instruction stream did not ` +
                `run to completion`);
        }
        compared++;
        for (let k = 0; k < OBS_REGS; k++) {
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
        for (let k = 0; k < PKT_PROBES.length; k++) {
            if (s.pkts[k] !== j.pkts[k]) {
                let i = PKT_PROBES[k], pkt = (i / PKT_WORDS) | 0, fld = i % PKT_WORDS;
                failures.push(`case ${c.idx} "${c.name}": PKTS[${i}] (packet ${pkt} ` +
                    `${fld === 0 ? "link" : "d[" + (fld - 1) + "]"}) = ${hex(j.pkts[k], 4)} here, ` +
                    `${hex(s.pkts[k], 4)} on the oracle`);
            }
        }
        let memDiffs = 0;
        for (let [a, v] of j.mem) {
            let sv = s.mem.get(a);
            if (sv === undefined) {
                failures.push(`case ${c.idx} "${c.name}": the oracle never reported memory at 0x${hex(a)}`);
                break;
            }
            if (sv !== v && memDiffs++ < 4) {
                failures.push(`case ${c.idx} "${c.name}": memory 0x${hex(a)} = ${hex(v)} here, ${hex(sv)} on the oracle`);
            }
        }
        if (memDiffs > 4) failures.push(`case ${c.idx} "${c.name}": ... and ${memDiffs - 4} more memory differences`);
        for (let [k, tag] of [["rings", "SHOW RQ RINGS"], ["freeq", "SHOW RQ FREEQ"], ["respq", "SHOW RQ RESPQ"]]) {
            let sv = (s[k] || "").replace(/\r/g, "").trim(), jv = (j[k] || "").trim();
            if (sv !== jv) {
                failures.push(`case ${c.idx} "${c.name}": ${tag} differs.\n    here:   ` +
                    `${JSON.stringify(jv).slice(0, 300)}\n    oracle: ${JSON.stringify(sv).slice(0, 300)}`);
            }
        }
    }
    return compared;
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors and exclusion fences.  Every one FAILS the run; none scales with case count.   *
 * ------------------------------------------------------------------------------------------- */

/**
 * assertExclusions(cases, sim, failures)
 *
 * The three named exclusions in the file header, enforced as PROPERTIES OF THE CASE LIST AND OF THE
 * ORACLE'S ANSWERS rather than as promises.  Standing rule 6: an exclusion that is merely unvisited
 * is a gap; one that fails the run when it is reached is a fence.
 */
function assertExclusions(cases, sim, failures)
{
    /* `step` is stamped by the case builders on the writes they INTEND as step-1 / step-4 words --
       the only writes at which the controller interprets those bits at all.  Testing every SA write
       instead would flag, say, a wrap-mode echo word that merely happens to have bits 0 and 1 set,
       which is what a first version of this fence did: it reported eleven "violations" that were
       words the controller was never going to read as a step-4 GO. */
    for (let c of cases) {
        for (let act of c.script) {
            if (act.a !== "wsa") continue;
            let v = act.v & 0xFFFF;
            if (act.step === 1 && (v & SA_S1H_VL) && !(v & SA_S1H_WR) &&
                (v & SA_S1H_IE) && (v & SA_S1H_VEC)) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" writes an S1 word 0x${hex(v, 4)} with ` +
                    `BOTH SA_S1H_IE and a non-zero SA_S1H_VEC, which requests a controller INTERRUPT. ` +
                    `Delivery LANDED in pcjsvax-aef and is graded by tests/mscpintdiff.js; this fence ` +
                    `is now a SCOPE boundary, not a gap.  What this file grades is the POLLING ` +
                    `ITERATION COUNT at which each handshake answer becomes visible, and an SCB ` +
                    `dispatch executing inside those poll loops would fold interrupt delivery into ` +
                    `that measurement -- and no SCB handler is installed for the RQ vector here, so ` +
                    `one would dispatch to a zero SCB slot and HALT at PC 1.  NOTE the fence tests ` +
                    `IE *and* VEC: this file's random cases DO program a non-zero VEC with IE clear, ` +
                    `which is what keeps \`dibp->vec\` graded here without any interrupt being raised.`);
            }
            if (act.step === 4 && (v & SA_S4H_LF)) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" writes a step-4 word 0x${hex(v, 4)} with ` +
                    `SA_S4H_LF, which requests a last-failure PACKET -- MSCP packet processing is ` +
                    `pcjsvax-6a5's later work.`);
            }
        }
    }
    /* Every SA write a case makes must be accounted for: an untagged one would slip past both fences
       above.  Tagging is the builders' job, and this is the check that they did it. */
    for (let c of cases) {
        for (let act of c.script) {
            if (act.a === "wsa" && act.step === undefined && act.v !== 0) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" makes an SA write of 0x${hex(act.v & 0xFFFF, 4)} ` +
                    `that no builder tagged with the handshake step it belongs to, so neither exclusion ` +
                    `fence above can see it`);
            }
        }
    }
    for (let i = 0; i < cases.length; i++) {
        let s = sim[i];
        if (!s) continue;
        if (s.rq.PBSY !== 0 || s.rq.RESP !== 0 || s.rq.FREE !== 1) {
            failures.push(`exclusion: case ${cases[i].idx} "${cases[i].name}" left the ORACLE with ` +
                `PBSY=${s.rq.PBSY} RESP=${s.rq.RESP} FREE=${s.rq.FREE} -- a packet was allocated, which ` +
                `means the case reached MSCP packet processing.  That is pcjsvax-6a5's later work and ` +
                `rq.js does not implement it.`);
        }
        if (s.rq.HAT !== s.rq.HTMO) {
            failures.push(`exclusion: case ${cases[i].idx} "${cases[i].name}" left the ORACLE with ` +
                `HAT=${s.rq.HAT} != HTMO=${s.rq.HTMO} -- rq_tmrsvc(), the once-per-second WALL-CLOCK ` +
                `host-access timer, fired.  It is not modelled (see rq.js's exclusions) and this case ` +
                `is long enough to reach it.`);
        }
    }
}

/**
 * coverage(cases, sim, js, failures, acc)
 *
 * Counted from cases that ACTUALLY REACHED COMPARISON, never from the case list: a phase whose
 * results never arrived must not be able to certify its own coverage.
 */
function coverage(cases, sim, js, failures, acc)
{
    let ok = (i) => sim[i] && js[i] && !js[i].unimplemented && sim[i].halted && js[i].halted;
    let atEnd = (i) => ok(i) && sim[i].atOwnHalt && js[i].atOwnHalt;

    /* Every state the controller can be in must have been OBSERVED on the ORACLE, and the whole set
       comes from rq.js's CST_NAMES, not from a list written here. */
    let statesSeen = new Set();
    let cstaValues = {CST_S1, CST_S1_WR, CST_S2, CST_S3, CST_S3_PPA, CST_S3_PPB, CST_S4, CST_UP, CST_DEAD};
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let n of CST_NAMES) if (sim[i].rq.CSTA === cstaValues[n]) statesSeen.add(n);
        /* A `reach` case HALTS in the state it names, so the oracle's own CSTA is the proof that the
           state was reached -- and it is checked against the state the case INTENDED, which is what
           turns "some case ended in CST_S3" into "the case that drives to CST_S3 gets there". */
        if (cases[i].kind === "reach" && sim[i].rq.CSTA !== cstaValues[cases[i].reachState]) {
            failures.push(`coverage: the case that drives the controller to ${cases[i].reachState} left ` +
                `the ORACLE in CSTA=${sim[i].rq.CSTA}, not ${cstaValues[cases[i].reachState]}`);
        }
        if (cases[i].kind === "ipwrite" && cases[i].reachState) acc.ipWriteFrom = acc.ipWriteFrom || new Set();
        if (cases[i].kind === "ipwrite" && cases[i].reachState) {
            /* R10 is the SA read taken IN the state, before the IP write; R11 is the SA after it.
               The state is proven REACHED by the oracle's own pre-write SA, not by our intent. */
            acc.ipWriteFrom.add(cases[i].reachState);
            if (sim[i].rq.SA !== 0x0B40 || sim[i].rq.CSTA !== CST_S1) {
                failures.push(`coverage: the IP write from ${cases[i].reachState} did not leave the ORACLE ` +
                    `in CST_S1 with SA=0B40 (got SA=${hex(sim[i].rq.SA, 4)} CSTA=${sim[i].rq.CSTA})`);
            }
        }
    }
    for (let n of CST_NAMES) {
        if (!statesSeen.has(n)) {
            failures.push(`coverage: the oracle was never observed in ${n} at the end of a graded case`);
        }
    }
    if (!acc.ipWriteFrom || acc.ipWriteFrom.size !== CST_NAMES.length) {
        failures.push(`coverage: an IP write was graded from ${acc.ipWriteFrom ? acc.ipWriteFrom.size : 0} of ` +
            `the ${CST_NAMES.length} controller states, not all of them`);
    }

    /* THE ANTI-REPLAY FLOOR.  At least three DISTINCT (cqCode, rqCode, echoS2, echoS3) tuples must
       have reached comparison and produced their OWN step-2/step-3 answers on the oracle -- which is
       what a hard-coded SA sequence cannot do. */
    let tuples = new Set(), echoOk = 0;
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i) || sim[i].rq.S1DAT === 0) continue;
        let d = sim[i].rq.S1DAT;
        tuples.add(`${(d >>> SA_S1H_V_CQ) & SA_S1H_M_CQ}:${(d >>> SA_S1H_V_RQ) & SA_S1H_M_RQ}:` +
                   `${(d >>> 8) & 0xFF}:${d & 0xFF}`);
        echoOk++;
    }
    if (tuples.size < 3) {
        failures.push(`coverage: only ${tuples.size} distinct (cq code, rq code, echo hi, echo lo) tuple(s) ` +
            `reached comparison; the floor is 3, because fewer cannot tell a COMPUTED step-2/step-3 answer ` +
            `from a replayed constant`);
    }
    acc.s1Tuples = tuples.size;

    /* THE SCATTER FLOOR: at least one graded case's comm region must have landed on discontiguous,
       DESCENDING physical pages -- the arrangement an identity map cannot produce. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i) || cases[i].entries.length < 2) continue;
        let ps = cases[i].entries.map((e) => e.p);
        let descending = ps.every((p, k) => k === 0 || p < ps[k - 1]);
        let gap = ps.some((p, k) => k > 0 && Math.abs(p - ps[k - 1]) > 1);
        if (descending && gap) acc.scatteredPages = true;
    }
    if (!acc.scatteredPages) {
        failures.push(`coverage: no graded case zeroed its comm region across discontiguous, DESCENDING ` +
            `physical pages -- an identity mapping would satisfy every case that ran`);
    }

    /* THE SCHEDULE FLOOR: at least three DISTINCT iteration counts must have been observed and
       matched, and at least one of them under a NON-DEFAULT ITIME.  A model that answers
       instantly produces zero for all of them and a model with a hard-coded delay produces one
       value for all of them. */
    let counts = new Set(), nonDefault = false;
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let act of cases[i].script) {
            if (act.a !== "poll") continue;
            counts.add(sim[i].regs[act.cnt]);
            if (cases[i].itime !== RQ_ITIME || cases[i].i4time !== RQ_ITIME4) nonDefault = true;
        }
    }
    acc.pollCounts = counts.size;
    if (counts.size < 3) {
        failures.push(`coverage: only ${counts.size} distinct polling ITERATION COUNT(s) reached ` +
            `comparison; the floor is 3, because fewer cannot tell a scheduled answer from an ` +
            `instantaneous one or from a hard-coded delay`);
    }
    if ([...counts].every((n) => n === 0)) {
        failures.push(`coverage: every graded polling loop saw the answer on its FIRST iteration -- ` +
            `the controller is answering instantaneously and the whole event model is unexercised`);
    }
    if (!nonDefault) {
        failures.push(`coverage: no graded case ran with a NON-DEFAULT ITIME/I4TIME, so the schedule ` +
            `could be a constant that happens to equal the default`);
    }

    /* Each named behaviour must have been OBSERVED on the oracle at least once. */
    let sawKind = (k) => cases.some((c, i) => ok(i) && c.kind === k);
    for (let [k, what] of [
        ["normal",     "the plain S1 -> S2 -> S3 -> S4 -> CST_UP walk"],
        ["wrap",       "wrap mode (SA_S1H_WR), the endless echo"],
        ["pp",         "the purge/poll path completed by an IP READ"],
        ["fatal-ppf",  "a non-zero write in CST_S3_PPA (PE_PPF)"],
        ["fatal-qwe",  "the unprogrammed-map fatal (PE_QWE)"],
        ["fatal-qre",  "the CST_UP poll's descriptor-read fatal (PE_QRE)"],
        ["poll-short", "an IP read in CST_UP with pip still set"],
        ["poll-long",  "an IP read in CST_UP whose service ran and cleared pip"],
        ["neighbour",  "an access one word outside the four-byte window"],
        ["cancel",     "an IP write cancelling a pending queue service"],
        ["rearm",      "a second SA write NOT re-arming an already-active unit"],
        ["timing",     "a walk under non-default ITIME/I4TIME"],
        ["longread",   "an aligned longword read at the base"],
        ["bytewrite",  "a byte write reaching rq_wr() unshifted"]
    ]) {
        if (!sawKind(k)) failures.push(`coverage: no graded case exercised ${what}`);
    }

    /* The two fatal codes must have been READ OUT OF THE ORACLE, not predicted here. */
    let sawErr = (code) => cases.some((c, i) => ok(i) && sim[i].rq.PERR === code && sim[i].rq.CSTA === CST_DEAD);
    for (let [code, name] of [[PE_QWE, "PE_QWE"], [PE_PPF, "PE_PPF"], [PE_QRE, "PE_QRE"]]) {
        if (!sawErr(code)) {
            failures.push(`coverage: the oracle was never observed DEAD with PERR = ${code} (${name})`);
        }
    }

    /* The purge-interrupt flag changes WHERE the zeroing starts (comm-8 vs comm-4).  Both must have
       been observed on the oracle, or the SA_COMM_QQ branch of rq_step4() is unexercised. */
    let prgiSeen = new Set();
    for (let i = 0; i < cases.length; i++) if (ok(i) && sim[i].rq.CSTA === CST_UP) prgiSeen.add(sim[i].rq.PRGI);
    if (prgiSeen.size < 2) {
        failures.push(`coverage: the oracle reached CST_UP with only PRGI=${[...prgiSeen].join("/")}; ` +
            `both settings are needed or rq_step4()'s SA_COMM_QQ branch never runs`);
    }

    /* The comm region must have been observed being ZEROED -- i.e. at least one graded case must
       have left a dumped page DIFFERENT from its seed on the oracle.  Without this, a controller
       that performed no DMA at all would satisfy every byte comparison above. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let [a, v] of sim[i].mem) {
            if (v !== seedFor((a / PAGE) | 0)) { acc.commZeroed = true; break; }
        }
        if (acc.commZeroed) break;
    }
    if (!acc.commZeroed) {
        failures.push(`coverage: no graded case's dumped pages ever differed from their seed on the ` +
            `oracle -- nothing was ever written to memory, so every byte comparison above is vacuous`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11)         *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    "sa-echo-bits-dropped": () => {
        let s2 = RQVAX.prototype.echoS2, s3 = RQVAX.prototype.echoS3;
        RQVAX.prototype.echoS2 = function(d) { return s2.call(this, d) & 0; };
        RQVAX.prototype.echoS3 = function(d) { return s3.call(this, d) & 0; };
        return () => { RQVAX.prototype.echoS2 = s2; RQVAX.prototype.echoS3 = s3; };
    },
    "sa-echo-halves-swapped": () => {
        let s2 = RQVAX.prototype.echoS2, s3 = RQVAX.prototype.echoS3;
        RQVAX.prototype.echoS2 = function(d) { return s3.call(this, d); };
        RQVAX.prototype.echoS3 = function(d) { return s2.call(this, d); };
        return () => { RQVAX.prototype.echoS2 = s2; RQVAX.prototype.echoS3 = s3; };
    },
    "step-2-comm-low-not-masked-with-SA_S2H_CLO": () => {
        let orig = RQVAX.prototype.quesvc;
        RQVAX.prototype.quesvc = function() {
            let was = this.csta, saw = this.saw;
            orig.call(this);
            /* Composed over the shipped code: if the step-2 branch just ran, put back the bit
               SA_S2H_CLO removed.  It changes nothing when the low bit was already clear, which is
               exactly why a case with an ODD comm word is in the enumerated list. */
            if (was === CST_S2) this.comm = (this.comm | (saw & 1)) >>> 0;
        };
        return () => { RQVAX.prototype.quesvc = orig; };
    },
    "step-3-comm-high-not-shifted": () => {
        let orig = RQVAX.prototype.quesvc;
        RQVAX.prototype.quesvc = function() {
            let was = this.csta, saw = this.saw, before = this.comm;
            orig.call(this);
            if (was === CST_S3) {
                /* Re-do the merge without the << 16, then let the rest of the original's work stand
                   by re-running step 4 only if the original reached it. */
                let wrong = ((saw & SA_S3H_CHI) | before) >>> 0;
                if (this.csta === CST_S3_PPA) this.comm = wrong;
                else { this.comm = wrong; }
            }
        };
        return () => { RQVAX.prototype.quesvc = orig; };
    },
    "ring-lengths-not-decoded-as-a-power-of-two": () => {
        let r = RQVAX.prototype.ringLenRQ, c = RQVAX.prototype.ringLenCQ;
        RQVAX.prototype.ringLenRQ = function(d) { return (((d >>> SA_S1H_V_RQ) & SA_S1H_M_RQ) + 1) << 2; };
        RQVAX.prototype.ringLenCQ = function(d) { return (((d >>> SA_S1H_V_CQ) & SA_S1H_M_CQ) + 1) << 2; };
        return () => { RQVAX.prototype.ringLenRQ = r; RQVAX.prototype.ringLenCQ = c; };
    },
    "ring-lengths-not-scaled-by-four": () => {
        let r = RQVAX.prototype.ringLenRQ, c = RQVAX.prototype.ringLenCQ;
        RQVAX.prototype.ringLenRQ = function(d) { return r.call(this, d) >>> 2; };
        RQVAX.prototype.ringLenCQ = function(d) { return c.call(this, d) >>> 2; };
        return () => { RQVAX.prototype.ringLenRQ = r; RQVAX.prototype.ringLenCQ = c; };
    },
    "ring-length-fields-swapped": () => {
        let r = RQVAX.prototype.ringLenRQ, c = RQVAX.prototype.ringLenCQ;
        RQVAX.prototype.ringLenRQ = function(d) { return c.call(this, d); };
        RQVAX.prototype.ringLenCQ = function(d) { return r.call(this, d); };
        return () => { RQVAX.prototype.ringLenRQ = r; RQVAX.prototype.ringLenCQ = c; };
    },
    "comm-zero-bypasses-the-map": () => {
        /* THE CHEAT, exactly: write the zeros straight to physical memory instead of through the
           CQBIC scatter-gather map.  Composed over the shipped step4() by intercepting the ONE call
           it makes, so everything else about step 4 is still the shipped code. */
        let orig = CQBICVAX.prototype.mapWriteW;
        CQBICVAX.prototype.mapWriteW = function(ba, bc, buf) {
            for (let i = 0; i < bc; i++) this.bus.setByte((ba + i) >>> 0, buf[i]);
            return 0;
        };
        return () => { CQBICVAX.prototype.mapWriteW = orig; };
    },
    "ip-read-returns-sa-instead-of-zero": () => {
        let orig = RQVAX.prototype.rd;
        RQVAX.prototype.rd = function(pa) {
            let v = orig.call(this, pa);
            return (((pa >>> 1) & 1) === 0) ? (this.sa & 0xFFFF) : v;
        };
        return () => { RQVAX.prototype.rd = orig; };
    },
    "ip-read-has-no-side-effects": () => {
        let orig = RQVAX.prototype.rd;
        RQVAX.prototype.rd = function(pa) {
            if (((pa >>> 1) & 1) === 0) return 0;            /* the value, without the state machine */
            return orig.call(this, pa);
        };
        return () => { RQVAX.prototype.rd = orig; };
    },
    "ip-write-does-not-reset": () => {
        let orig = RQVAX.prototype.wr;
        RQVAX.prototype.wr = function(pa, data) {
            if (((pa >>> 1) & 1) === 0) return;
            return orig.call(this, pa, data);
        };
        return () => { RQVAX.prototype.wr = orig; };
    },
    "wrong-port-model-in-the-step-4-answer": () => {
        let prev = RQVAX.CTLR_TAB;
        RQVAX.CTLR_TAB = prev.map((c) => ({uqpm: c.uqpm ^ 1, model: c.model, name: c.name}));
        return () => { RQVAX.CTLR_TAB = prev; };
    },
    "wrong-software-version-in-the-step-4-answer": () => {
        let prev = RQVAX.RQ_SVER;
        RQVAX.RQ_SVER = prev + 1;
        return () => { RQVAX.RQ_SVER = prev; };
    },
    "answers-instantly-instead-of-after-ITIME": () => {
        let orig = RQVAX.prototype.activateQueue;
        RQVAX.prototype.activateQueue = function(delay) { return orig.call(this, 0); };
        return () => { RQVAX.prototype.activateQueue = orig; };
    },
    "schedule-uses-a-constant-instead-of-the-register": () => {
        let orig = RQVAX.prototype.activateQueue;
        RQVAX.prototype.activateQueue = function(delay) { return orig.call(this, RQ_ITIME); };
        return () => { RQVAX.prototype.activateQueue = orig; };
    },
    "sim_activate-re-arms-an-already-active-unit": () => {
        let orig = RQVAX.prototype.activateQueue;
        RQVAX.prototype.activateQueue = function(delay) { this.queDue = null; return orig.call(this, delay); };
        return () => { RQVAX.prototype.activateQueue = orig; };
    },
    /* The field is `itime4`, matching the C's `rq_itime4`.  A first version of this mutation assigned
       `this.i4time` -- a name that exists only in this file's case specs -- so it created a new
       property, perturbed nothing, and SURVIVED.  A mutation that does not change the shipped path
       reports the suite's coverage rather than measuring it, which is the failure mode HANDOFF.md
       standing rule 11 is about; the guard below asserts the field is real before touching it. */
    "stage-4-uses-ITIME-instead-of-I4TIME": () => {
        let orig = RQVAX.prototype.wr;
        RQVAX.prototype.wr = function(pa, data) {
            if (this.itime4 === undefined) throw new Error("mscpinitdiff: RQVAX has no `itime4`");
            if (((pa >>> 1) & 1) === 1 && this.csta === CST_S4) { this.itime4 = this.itime; }
            return orig.call(this, pa, data);
        };
        return () => { RQVAX.prototype.wr = orig; };
    },
    "fatal-does-not-reset-the-controller-first": () => {
        let orig = RQVAX.prototype.fatal;
        RQVAX.prototype.fatal = function(err) {
            let comm = this.comm, cq = Object.assign({}, this.cq), rq = Object.assign({}, this.rq);
            let r = orig.call(this, err);
            this.comm = comm; this.cq = cq; this.rq = rq;   /* undo reset()'s wipe, keep the rest */
            return r;
        };
        return () => { RQVAX.prototype.fatal = orig; };
    },
    "reset-clears-the-port-error": () => {
        let orig = RQVAX.prototype.reset;
        RQVAX.prototype.reset = function() { orig.call(this); this.perr = 0; };
        return () => { RQVAX.prototype.reset = orig; };
    },
    "reset-does-not-cancel-the-pending-service": () => {
        let orig = RQVAX.prototype.reset;
        RQVAX.prototype.reset = function() { let due = this.queDue; orig.call(this); this.queDue = due; };
        return () => { RQVAX.prototype.reset = orig; };
    },
    "purge-poll-accepts-a-non-zero-write": () => {
        let orig = RQVAX.prototype.quesvc;
        RQVAX.prototype.quesvc = function() {
            if (this.csta === CST_S3_PPA) { this.csta = CST_S3_PPB; return; }
            return orig.call(this);
        };
        return () => { RQVAX.prototype.quesvc = orig; };
    },
    "rq-window-decoded-at-the-wrong-offset": () => ({rqBaseDelta: IOLN_RQ}),
    "rq-window-narrowed-to-one-register": () => ({rqSizeDelta: -2}),
    "no-event-queue-at-all": () => ({noQbusHook: true})
};

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                       *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

/**
 * ioSpace(simh, opts)
 *
 * PHASE A.  `SHOW QBA IOSPACE`, parsed the same way dbldiff.js parses it -- base, inclusive end,
 * trailing device name -- and used to VERIFY rq.js's RQ_BASE/IOLN_RQ against the oracle's live
 * autoconfiguration rather than to trust two constants.
 */
function ioSpace(simh, opts)
{
    let out = runSimh(simh, `set cpu ${MEM_MB}m\nset rq rqdx3\nshow qba iospace\nexit\n`,
                      path.join(opts.scratch, "mscpinit-iospace.ini"));
    let rows = [], re = /^([0-9A-Fa-f]{8}) - ([0-9A-Fa-f]{8})\*?(.*)$/gm, m;
    while ((m = re.exec(out)) !== null) {
        let tail = m[3].trim().split(/\s+/);
        rows.push({base: parseInt(m[1], 16) >>> 0, end: parseInt(m[2], 16) >>> 0,
                   name: tail.length ? tail[tail.length - 1] : "?"});
    }
    if (!rows.length) throw new Error("mscpinitdiff: could not parse `SHOW QBA IOSPACE`; SIMH said:\n" + out);
    return rows;
}

/**
 * timingDefaults(simh, opts)
 *
 * PHASE T.  The oracle's OWN ITIME/I4TIME/QTIME/XTIME defaults, read before anything is deposited,
 * so rq.js's four constants are checked against the simulator rather than against a memory of it.
 */
function timingDefaults(simh, opts)
{
    let out = runSimh(simh, `set cpu ${MEM_MB}m\nset rq rqdx3\nexamine -h rq itime,i4time,qtime,xtime\nexit\n`,
                      path.join(opts.scratch, "mscpinit-timing.ini"));
    let g = (n) => { let m = new RegExp(`^${n}:\\s*([0-9A-Fa-f]+)`, "m").exec(out); return m ? parseInt(m[1], 16) : null; };
    return {itime: g("ITIME"), i4time: g("I4TIME"), qtime: g("QTIME"), xtime: g("XTIME")};
}

function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [], acc = {};

    /* ---- PHASE A ---- */
    let rows = opts.ioRows;
    let rqRows = rows.filter((r) => r.name === "RQ");
    if (rqRows.length !== 1) {
        failures.push(`PHASE A: \`SHOW QBA IOSPACE\` lists ${rqRows.length} RQ row(s); the oracle has ` +
            `exactly ONE enabled MSCP controller (RQB/RQC/RQD are DEV_DIS)`);
    } else {
        let got = {base: rqRows[0].base, size: (rqRows[0].end - rqRows[0].base + 1) >>> 0};
        if (got.base !== (RQ_BASE >>> 0) || got.size !== IOLN_RQ) {
            failures.push(`PHASE A: the oracle autoconfigures RQ at 0x${hex(got.base)}..+${got.size}, ` +
                `rq.js has 0x${hex(RQ_BASE)}..+${IOLN_RQ} -- the constant and the live ` +
                `autoconfiguration disagree`);
        }
        report.push(`  PHASE A  oracle autoconfigures RQ at 0x${hex(got.base)} for ${got.size} byte(s); ` +
            `rq.js agrees.  ${rows.length - 1} other I/O-page window(s): ` +
            rows.filter((r) => r.name !== "RQ").map((r) => r.name).join(", "));
    }

    /* ---- PHASE T ---- */
    let td = opts.timing;
    for (let [n, got, want] of [["ITIME", td.itime, RQ_ITIME], ["I4TIME", td.i4time, RQ_ITIME4],
                                ["QTIME", td.qtime, RQ_QTIME], ["XTIME", td.xtime, RQ_XTIME]]) {
        if (got !== want) {
            failures.push(`PHASE T: the oracle's default ${n} is ${got}, rq.js has ${want}`);
        }
    }
    report.push(`  PHASE T  oracle defaults ITIME=${td.itime} I4TIME=${td.i4time} QTIME=${td.qtime} ` +
        `XTIME=${td.xtime}; rq.js agrees, and every case DEPOSITS all four rather than inheriting them`);

    /* ---- THE CASES ---- */
    let {cases} = enumeratedCases();
    let all = cases.concat(randomCases(opts.nRandom, opts.seed, cases.length));
    let sim = runCasesSimh(simh, opts, all);
    /* Each pass writes its own do-file and starts its OWN simulator, so the oracle's static `MSC`
       struct starts every pass at its C-global zero.  The JS machine is built once and reused
       (standing rule 14), so the pass boundary is where that has to be re-established -- and
       nowhere else, because the fields powerUp() clears deliberately survive a per-case reset. */
    machine(mutationOpts).rq.powerUp();
    let js = all.map((c) => runCaseJS(c, mutationOpts));

    assertExclusions(all, sim, failures);
    let compared = grade(all, sim, js, failures);
    coverage(all, sim, js, failures, acc);

    /* The wiring the graded machine is actually holding, asserted rather than assumed. */
    let m = machine(mutationOpts);
    if (!mutationOpts.noQbusHook && m.cpu.qbus !== m.rq) {
        failures.push(`the graded machine's CPU has no Qbus event hook wired to the controller, so ` +
            `rq_quesvc() can never run and every "answer" would be whatever the last write left`);
    }
    if (!m.rq.cqbic || !m.rq.cqbic.bus) {
        failures.push(`the graded machine's controller has no CQBIC with a bus, so rq_step4()'s ` +
            `comm-region DMA cannot reach memory at all`);
    }

    report.push(`  CASES    ${compared}/${all.length} case(s) compared ` +
        `(${cases.length} enumerated + ${opts.nRandom} randomized)`);
    report.push(`  FLOORS   ${acc.s1Tuples || 0} distinct s1dat decode tuple(s), ` +
        `${acc.pollCounts || 0} distinct polling iteration count(s), ` +
        `scattered comm pages: ${acc.scatteredPages ? "yes" : "NO"}, ` +
        `comm region observed zeroed: ${acc.commZeroed ? "yes" : "NO"}`);
    return {failures, report, compared};
}

function selfcheck(simh, opts)
{
    let survived = [];
    for (let name of Object.keys(MUTATIONS)) {
        let apply = MUTATIONS[name]();
        let restore = typeof apply === "function" ? apply : () => {};
        let mutationOpts = typeof apply === "function" ? {} : apply;
        let failures;
        try {
            failures = runPass(simh, opts, mutationOpts).failures;
        } finally {
            restore();
        }
        if (!failures.length) survived.push(name);
        console.log(`  ${failures.length ? "CAUGHT " : "SURVIVED"}  ${name}` +
            (failures.length ? `  (${failures.length} failure(s), first: ${failures[0].split("\n")[0]})` : ""));
    }
    return survived;
}

function main()
{
    let simh = findSimhBin(getArg("--simh", null));
    let nRandom = +getArg("--cases", RANDOM_CASES_DEFAULT);
    let seed = +getArg("--seed", 20260727);
    let fSelfcheck = process.argv.includes("--selfcheck");

    if (nRandom < RANDOM_CASES_FLOOR) {
        console.error(`mscpinitdiff: --cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "mscpinitdiff-"));
    let code = 0;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized cases: ${nRandom}`);
        console.log(`RQ: Qbus I/O page 0x${hex(RQ_BASE)}..+${IOLN_RQ}, ctype ${RQDX3_CTYPE} ` +
            `(${CTLR_TAB[RQDX3_CTYPE].name}, UQPM ${CTLR_TAB[RQDX3_CTYPE].uqpm}), SVER ${RQ_SVER}, ` +
            `${RQ_NPKTS} packets`);

        let opts = {scratch, nRandom, seed};
        opts.ioRows = ioSpace(simh, opts);
        opts.timing = timingDefaults(simh, opts);

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
            console.log(`\nMATCH -- every graded step of the UQSSP initialisation agrees with the oracle: ` +
                `the SA values the host reads back, the polling ITERATION COUNT at which each answer ` +
                `becomes visible, PC/PSL/R0..R14, every byte of every physical page the comm region was ` +
                `scattered across, all ${RQ_OBS.length} examinable controller registers, ` +
                `${PKT_PROBES.length} words of the packet array, and the text of SHOW RQ RINGS/FREEQ/RESPQ.`);
        }

        if (fSelfcheck && !code) {
            console.log(`\nPHASE M -- mutations (${Object.keys(MUTATIONS).length})`);
            let survived = selfcheck(simh, opts);
            if (survived.length) {
                console.error(`\nFAIL -- ${survived.length} mutation(s) SURVIVED: ${survived.join(", ")}`);
                code = 1;
            } else {
                console.log(`\nall ${Object.keys(MUTATIONS).length} mutation(s) CAUGHT`);
            }
        }
        if (!code) console.log("\nOK");
    } finally {
        /* Every exit path, including a throw. */
        try { fs.rmSync(scratch, {recursive: true, force: true}); } catch (e) { /* best effort */ }
    }
    process.exit(code);
}

main();
