/**
 * @fileoverview Differential test: a host posts MSCP commands in the command ring and the RQDX3
 *               returns responses in the response ring -- vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-0b4, the second of pcjsvax-6a5's children.  With the controller UP (pcjsvax-c2c, graded by
 * tests/mscpinitdiff.js), a host program builds MSCP packets in mapped Qbus memory, posts them by
 * setting the command ring descriptors' ownership bits, rings the doorbell by READING IP, and gets
 * response packets written back into the response ring.  Every descriptor bit, ring index, interrupt
 * flag word, credit count, free-queue transition and packet byte is compared with the oracle.
 *
 * THE VEHICLE COMMAND IS SET CONTROLLER CHARACTERISTICS (OP_SCC, 4), because it is the only MSCP
 * command that needs neither an attached unit nor a transfer.  It exercises the ring machinery and
 * nothing else, which is the point: the twelve unit-bearing commands are NOT this item's, and
 * `assertExclusions()` below FAILS the run if any graded case sends one.  (Seven of the twelve are
 * now ANSWERED by rq.js -- pcjsvax-f52 landed them, with no unit attached they mostly report
 * ST_OFL, and tests/mscpunitdiff.js is what grades them.  The fence stays exactly as it was:
 * pcjsvax-0b4's argument is about the RING MACHINERY, and letting a unit command into this case
 * list would make one item's coverage certify another's.)
 *
 * THREE INDEPENDENT UNPATCHED VIEWS, WHICH MUST AGREE.  No eighth SIMH patch:
 *   1. the RESPONSE PACKETS IN HOST PHYSICAL MEMORY, dumped WHOLE PAGES at a time with `examine`;
 *   2. SIMH'S OWN COPIES -- `examine rq pkts[N]` over the packet array, plus the 23 examinable
 *      controller registers and `show rq rings` / `freeq` / `respq`;
 *   3. the ORDERED `set rq debug=REQ` TRACE, one line per command and per response, parsed as a
 *      SEQUENCE and compared line for line against rq.js's own `reqLog`.
 * A model that satisfies (1) can still fail (2) -- a response written to memory by a controller
 * whose internal packet pool is wrong -- and a model that satisfies both can still fail (3), which
 * is the only one of the three that can tell "the right responses" from "the right responses in the
 * right ORDER".
 *
 * THE CHEAT THIS FILE EXISTS TO KILL
 * -----------------------------------
 * Servicing the command SYNCHRONOUSLY inside the IP read.  The real controller sets `pip` and
 * SCHEDULES rq_quesvc; the response appears rq_qtime (100) instructions later, and a host polling
 * immediately MUST see it not yet there.  A synchronous implementation passes any test that only
 * checks the final memory image.  So every command in every case is followed by a BOUNDED, IN-BAND
 * busy-wait (Asm.awaitL) that counts its own iterations into the RESULT page, and those COUNTS are
 * compared -- a synchronous controller answers on iteration 0 and every count goes to zero.  The
 * `answers-synchronously-inside-the-IP-read` mutation is exactly that cheat.
 *
 * The second cheat is keeping the packet in a JS object and writing back only the fields the test
 * looks at.  Every command packet is planted in memory with only a few words set, over a
 * page-distinct 0xA5A5.... seed, so the tail of every packet is data the controller must carry
 * through untouched; the whole 64 bytes are read and `d[UQ_HLNT] - UQ_HDR_OFF` bytes written back
 * through mapReadW/mapWriteW; and the comparison is of WHOLE PHYSICAL PAGES, not of fields.  The
 * DBG_REQ trace corroborates it from the other side: its `bc=`/`ma=`/`lbn=` fields print packet
 * words no SCC command ever assigns, so they show the seed the controller carried in.
 *
 * THE TIMING IS GRADED IN BAND, BECAUSE OUT OF BAND IS STRUCTURALLY BLIND
 * -----------------------------------------------------------------------
 * A HALT DRAINS SIMH'S EVENT QUEUE (vax_cpu.c:2643).  A differential that halts early to observe a
 * pre-event state sees the post-event state and passes while grading nothing about the schedule.
 * Hence the in-band counts above.  It also has a sharper consequence here than it did for
 * pcjsvax-c2c: *** A HOST THAT HALTS WITH A DEFERRED RESPONSE AND NO HOST-OWNED RESPONSE DESCRIPTOR
 * HANGS THE REAL SIMULATOR INSIDE ITS HALT INSTRUCTION ***, because rq_quesvc re-arms itself
 * forever trying to place a packet the ring has no room for.  The response-ring-full case therefore
 * grants the descriptor BEFORE it halts, and rq.js's drainOnHalt() carries a bound that names the
 * condition rather than hanging in sympathy.
 *
 * WHAT IS DELIBERATELY NOT GRADED, BY NAME (standing rule 6)
 * -----------------------------------------------------------
 *   - The twelve UNIT-BEARING MSCP commands and all disk I/O.  Seven are answered by rq.js and
 *     graded by tests/mscpunitdiff.js (pcjsvax-f52); the five transfer opcodes still throw
 *     RQUnimplemented by name (pcjsvax-346).  Either way NO case here sends one:
 *     `assertExclusions()` FAILS the run if a case does, and PHASE S FAILS the run if the C's
 *     dispatch grows a case rq.js does not classify or moves one to a different handler.
 *   - CONTROLLER INTERRUPT DELIVERY.  rq_ring_int() raises an interrupt on SA_S1H_VEC ALONE -- it
 *     does NOT test SA_S1H_IE, unlike rq_init_int().  That asymmetry LANDED in pcjsvax-aef and is
 *     graded by tests/mscpintdiff.js, so this is a SCOPE boundary rather than a gap: every graded
 *     case here still supplies VEC == 0 and `assertExclusions()` still FAILS the run if one does
 *     not, because this file grades awaitL()'s ITERATION COUNTS and an SCB dispatch inside those
 *     loops would change them (and no SCB handler is installed for the RQ vector here).  What IS
 *     graded here is rq_ring_int()'s FLAG WORD: the one-word 1 it DMAs to comm-4 / comm-2,
 *     including the fact that a failure of that write is IGNORED.
 *   - rq_plf(), the last-failure packet.  Reachable only by a host that sets SA_S4H_LF in its
 *     step-4 GO word after a fatal; no case does, and the fence is the same one mscpinitdiff uses.
 * *   - rq_getpkt()'s `cp->hat = 0`.  It disables the host-access timer while a command is in
 *     flight, and NOTHING IN THIS ITEM'S SCOPE CAN FALSIFY IT: rq_putpkt() restores `hat = htmo`
 *     the moment `pbsy` returns to zero, rq_reset() sets it on every fatal, and the only window in
 *     which it is 0 is inside a single rq_quesvc() call -- which a host can only look at by
 *     halting, and a halt drains the event queue.  A mutation for it was written, measured to
 *     SURVIVE for that reason, and removed rather than left to be reported as coverage; it becomes
 *     observable when rq_tmrsvc() is modelled.  (The alternative -- halting with a packet still
 *     busy -- is the hang described above, not a test.)
 *   - rq_tmrsvc(), the once-per-second WALL-CLOCK host-access timer.  Not an instruction count and
 *     not modelled.  The fence is sharper here than in mscpinitdiff, because a completed command
 *     legitimately moves HAT: `assertExclusions()` requires the ORACLE's HAT to be either 0 (a
 *     command is in flight) or exactly HTMO (idle), which is what firing would break.
 *
 *      node machines/dec/vax/tests/mscpringdiff.js [options]
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
    RQUnimplemented, CST_UP, CST_DEAD,
    SA_S1H_VL, SA_S1H_IE, SA_S1H_VEC, SA_S1H_V_CQ, SA_S1H_V_RQ,
    UQ_DESC_OWN, UQ_DESC_F, UQ_ADDR, UQ_HDR_OFF,
    UQ_HLNT, UQ_HCTC, UQ_HCTC_V_CR, UQ_HCTC_V_TYP, UQ_HCTC_V_CID,
    UQ_TYP_SEQ, UQ_TYP_DAT, UQ_CID_MSCP, UQ_CID_DUP, UQ_CID_DIAG,
    OP, ST, I_OPCD, CMD_OPC, CMD_MOD, CMD_UN, CMD_REFL, CMD_REFH,
    RSP_LNT, SCC_LNT, SCC_MSV, SCC_CFL, SCC_TMO,
    PE_PRE, PE_PWE, PE_QRE, PE_QWE, PE_ICI, PE_PIE, PE_NSR,
    RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME
} from "../modules/v2/rq.js";
import {
    PAGE, R_CODE, R_RESULT, MAP_MBR, DATA_NPAGE, OBS_REGS,
    RQ_IP, RQ_SA, CQBIC_BASE, CQMAP_BASE, CQMAP_VLD, MEM_MB,
    hex, findSimhBin, runSimh, mulberry32, sampleHeap, peakHeap,
    Asm, machine, RQ_OBS, rqFieldOf, PKT_WORDS, pktWord,
    showCtrl, physPageFor, seedFor, walkScript, emitAction,
    simhResetLines, jsResetForCase, geometry, qbusPagesFor
} from "./mscpharness.js";
import { checkScope } from "./mscpscope.js";

/** An absolute bound on the instructions any case may execute.  A case that does not HALT within it
    is reported BY NAME rather than compared at whatever PC it happened to reach. */
const MAX_STEPS = 150000;

/** The host's ring-phase scratch registers.  R0..R8 belong to the handshake (mscpharness's
    walkScript) and must survive to the HALT, where they are compared; R14 is SP.  Everything a
    ring step needs to remember goes to the RESULT page instead, which is what lets a case run
    thirty-two commands and still be graded on every one of them. */
const REGS = {prev: 9, cur: 10, cnt: 11, lim: 12, tmp: 13};

/** Iterations an in-band wait may burn before giving up.  SIX instructions per iteration, so this
    is ~24,000 instructions -- two orders of magnitude past QTIME.  A wait that exhausts it stores a
    zero remaining-budget longword and `assertWaits()` FAILS the case BY NAME rather than comparing
    two engines that both gave up (standing rule 6). */
const AWAIT_LIMIT = 4000;

const RANDOM_CASES_DEFAULT = 16;
const RANDOM_CASES_FLOOR   = 8;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure and NOT scaled by case
    count (rules 4 and 14).  ONE machine is built and reused across every case and every mutation
    pass; the dominant term is its single 16MB RAM allocation. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;

/** SIMH's flat 16-bit view of the packet array: packet p's LINK is index p*PKT_WORDS and its data
    word w is p*PKT_WORDS + 1 + w.  Probed over the FOUR packets any case can allocate plus the last
    one, all 33 words each, because a response is built IN PLACE over the command and the words a
    handler does not touch are the evidence that it round-tripped. */
const PKT_PROBES = (function() {
    let out = [];
    for (let p of [0, 1, 2, 3, 31]) for (let w = 0; w < PKT_WORDS; w++) out.push(p * PKT_WORDS + w);
    return out;
})();

/* ------------------------------------------------------------------------------------------- *
 * The command packet a host plants in memory                                                    *
 * ------------------------------------------------------------------------------------------- */

/**
 * cmdWords(o)
 *
 * The words of an MSCP command packet a case sets, as {index: value}.  EVERY WORD IT DOES NOT SET
 * IS LEFT AS THE PAGE SEED, deliberately: the controller reads all 64 bytes whatever the command
 * is, carries them through rq_putr(), and writes back `d[UQ_HLNT] - UQ_HDR_OFF` of them, so the
 * seed in the tail is what proves the round trip happened rather than a field-by-field
 * reconstruction.  UQ_HLNT itself is set to a value the controller OVERWRITES, which is how a case
 * shows that the response length is the handler's and not the host's.
 */
function cmdWords(o)
{
    let w = {};
    w[UQ_HLNT] = (o.hlnt === undefined) ? SCC_LNT : o.hlnt;
    w[UQ_HCTC] = (((o.cr || 0) << UQ_HCTC_V_CR) |
                  ((o.typ === undefined ? UQ_TYP_SEQ : o.typ) << UQ_HCTC_V_TYP) |
                  ((o.cid === undefined ? UQ_CID_MSCP : o.cid) << UQ_HCTC_V_CID)) & 0xFFFF;
    w[CMD_REFL] = (o.ref === undefined ? 0xBEEF : o.ref) & 0xFFFF;
    w[CMD_REFH] = (o.refh === undefined ? 0x1234 : o.refh) & 0xFFFF;
    w[CMD_UN] = o.unit || 0;
    w[CMD_OPC] = (o.opc === undefined ? OP.SCC : o.opc) & 0xFFFF;
    w[CMD_MOD] = o.mod || 0;
    if (o.msv !== undefined) w[SCC_MSV] = o.msv;
    else w[SCC_MSV] = 0;                                    /* MSCP version 0, or rq_scc fails */
    if (o.cfl !== undefined) w[SCC_CFL] = o.cfl;
    if (o.tmo !== undefined) w[SCC_TMO] = o.tmo;
    return w;
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

const RING_CODE_MAX = 7;

/**
 * buildCase(spec)
 *
 * Turns a declarative spec into a fully resolved case: the S1 word, the geometry, the map entries,
 * the physical presets that plant the command packets, the instruction stream, the RESULT-page slot
 * assignment, and the physical pages to dump.
 *
 * `spec.steps` is the RING PHASE, appended to the four-step handshake mscpharness supplies.  Its
 * vocabulary is emitRingStep() below; every one of those is a real VAX instruction against a real
 * physical address, and there is no "call the device" action by construction.
 */
function buildCase(spec)
{
    let c = Object.assign({
        itime: RQ_ITIME, i4time: RQ_ITIME4, qtime: RQ_QTIME, xtime: RQ_XTIME,
        cqCode: 0, rqCode: 0, comm: 0x2000, prgi: 0, spread: 0,
        nCmdBuf: 1, nRspBuf: 1, unmappedQ: [], packets: {}, steps: []
    }, spec);

    if (c.cqCode > RING_CODE_MAX || c.rqCode > RING_CODE_MAX) {
        throw new Error(`mscpringdiff: case "${c.name}" ring code out of range`);
    }
    /* SA_S1H_VL, the ring length codes, and NOTHING ELSE.  SA_S1H_VEC is zero in every case here --
       see the INTERRUPT DELIVERY exclusion; assertExclusions() re-derives that from the case list
       rather than trusting this comment. */
    c.s1dat = (SA_S1H_VL | (c.cqCode << SA_S1H_V_CQ) | (c.rqCode << SA_S1H_V_RQ)) & 0xFFFF;
    c.g = geometry(c);

    let qpages = qbusPagesFor(c.g);
    let unmapped = new Set(c.unmappedQ);
    c.entries = qpages.filter((q) => !unmapped.has(q)).map((q) => ({q, p: physPageFor(q, c.spread)}));
    c.qToP = new Map(c.entries.map((e) => [e.q, e.p]));
    c.zeroIdx = qpages.slice();
    c.dumpPages = [...new Set(c.entries.map((e) => e.p))].sort((a, b) => a - b);
    /* The RESULT page is PHYSICAL scratch, not Qbus: it is where the host program stores its own
       observations, and it is dumped and compared whole like every other page. */
    c.resultPage = (R_RESULT / PAGE) | 0;

    /**
     * The Qbus -> physical translation, done a SECOND TIME here on purpose (the discipline
     * qdmadiff.js applies to its own page list): the host program addresses PHYSICAL memory, the
     * controller addresses QBUS memory through the CQBIC map, and if these two arithmetics ever
     * disagree the case shows up as a memory difference rather than as a silent pass.
     */
    c.phys = (qaddr) => {
        let q = (qaddr / PAGE) | 0;
        if (!c.qToP.has(q)) {
            throw new Error(`mscpringdiff: case "${c.name}" addresses Qbus 0x${hex(qaddr, 6)}, whose ` +
                `page ${q} is deliberately UNMAPPED -- the host cannot reach it either`);
        }
        return (c.qToP.get(q) * PAGE + (qaddr % PAGE)) >>> 0;
    };

    /* The command packets, planted as PHYSICAL longword deposits on both engines.  Memory content
       is memory content; what the differential is about is what the CONTROLLER does with it.  The
       `host-written` case family below plants its packet with real MOVW instructions instead, and a
       coverage floor requires at least one of those to have reached comparison. */
    c.presets = [];
    for (let i of Object.keys(c.packets)) {
        let w = c.packets[i];
        for (let k of Object.keys(w)) {
            c.presets.push({addr: c.phys(c.g.cmdBuf(+i) + (+k) * 2), word: w[k] & 0xFFFF});
        }
    }

    /* RESULT-page slot assignment.  Done in the SAME walk the emitter uses, so the reporter and the
       instruction stream cannot disagree about which longword means what. */
    c.slots = [];
    let off = 0;
    let take = (n, what) => { let o = off; for (let k = 0; k < n; k++) c.slots.push({off: o + k * 4, what}); off += n * 4; return o; };
    for (let st of c.steps) {
        if (st.s === "await") st.roff = take(3, `${st.what || "await"} (iterations / value / budget)`);
        else if (st.s === "snap" || st.s === "snapsa") st.roff = take(1, st.what || st.s);
    }
    if (off > PAGE) throw new Error(`mscpringdiff: case "${c.name}" needs ${off} RESULT bytes`);
    c.resultUsed = off;

    let a = new Asm();
    a.movImmAbs(4, MAP_MBR, (CQBIC_BASE + 4 * 4) >>> 0);            // REG_MBR == 4
    for (let e of c.entries) a.movImmAbs(4, (CQMAP_VLD | e.p) >>> 0, (CQMAP_BASE + e.q * 4) >>> 0);
    for (let act of walkScript(c.s1dat, c.comm, c.prgi, {})) {
        if (!emitAction(a, act)) throw new Error(`mscpringdiff: unknown handshake action "${act.a}"`);
    }
    for (let st of c.steps) emitRingStep(a, st, c);
    a.halt();
    c.code = a.b;
    c.haltPC = (R_CODE + c.code.length) >>> 0;
    if (c.code.length > 0x1000) throw new Error(`mscpringdiff: case "${c.name}" code is ${c.code.length} bytes`);
    return c;
}

/**
 * emitRingStep(a, st, c)
 *
 * The ring-phase action vocabulary.  Descriptors are written by the HOST, to PHYSICAL addresses,
 * with real instructions -- the controller reaches the same words through the CQBIC scatter-gather
 * map, and the two paths meeting is half of what this differential proves.
 */
function emitRingStep(a, st, c)
{
    let g = c.g;
    let descOf = (own, flag, addr) => (((own ? UQ_DESC_OWN : 0) | (flag ? UQ_DESC_F : 0) | addr) >>> 0);
    switch (st.s) {
    case "cdesc":                                       /* post a command */
        return a.movImmAbs(4, descOf(st.own !== false, st.flag !== false, g.cmdEnv(st.pkt)),
                           c.phys(g.cqBa + st.slot * 4));
    case "rdesc":                                       /* grant a response slot */
        return a.movImmAbs(4, descOf(st.own !== false, st.flag !== false, g.rspEnv(st.buf)),
                           c.phys(g.rqBa + st.slot * 4));
    case "rawdesc":                                     /* an arbitrary descriptor value */
        return a.movImmAbs(4, st.v >>> 0,
                           c.phys((st.ring === "cq" ? g.cqBa : g.rqBa) + st.slot * 4));
    /* ONE ALIGNED LONGWORD COVERS BOTH FLAG WORDS: SA_COMM_CI is comm-4 and SA_COMM_RI is comm-2,
       adjacent and little-endian, so the low half is the COMMAND ring's flag and the high half is
       the RESPONSE ring's.  Reading them as two separate words would mean an UNALIGNED longword at
       comm-2, whose upper half lies in the NEXT QBUS PAGE -- which the map scatters somewhere else
       entirely, so the host's own read would splice two unrelated physical pages together and the
       value would not be a flag word at all.  (It compares equal on both engines and grades
       nothing, which is the worst kind of check to have.) */
    case "clrint":                                      /* zero BOTH ring interrupt flag words */
        return a.movImmAbs(4, 0, c.phys((c.comm - 4) >>> 0));
    case "ip":                                          /* ring the doorbell */
        return a.movAbsReg(2, RQ_IP, REGS.tmp);
    case "await":
        return a.awaitL(c.phys((st.ring === "cq" ? g.cqBa : g.rqBa) + st.slot * 4),
                        (R_RESULT + st.roff) >>> 0, AWAIT_LIMIT, REGS);
    case "snap":
        a.movAbsReg(4, c.phys(st.q >>> 0), REGS.prev);
        return a.movRegAbs(REGS.prev, (R_RESULT + st.roff) >>> 0);
    case "snapsa":
        a.movAbsReg(2, RQ_SA, REGS.prev);
        return a.movRegAbs(REGS.prev, (R_RESULT + st.roff) >>> 0);
    case "went":                                        /* host INVALIDATES a map entry */
        return a.movImmAbs(4, st.v >>> 0, (CQMAP_BASE + st.q * 4) >>> 0);
    case "delay":
        return a.delay(st.n, REGS.tmp);
    case "wpkt": {                                      /* the HOST builds the packet, word by word */
        let w = st.words;
        for (let k of Object.keys(w)) a.movImmAbs(2, w[k] & 0xFFFF, c.phys(g.cmdBuf(st.pkt) + (+k) * 2));
        return a;
    }
    }
    throw new Error(`mscpringdiff: unknown ring step "${st.s}"`);
}

/**
 * command(seq, o)
 *
 * ONE complete host transaction, appended to `seq`: clear the two interrupt flag words, post the
 * command descriptor, grant a response descriptor, read IP, wait IN BAND for the response
 * descriptor to change, then snapshot both flag words and the response descriptor's final value.
 *
 * The `delay` at the end is not padding.  Without it, the controller's SECOND queue service (the
 * one that finds the ring empty and clears `pip`) can land in the middle of the next command's
 * descriptor writes, which is legal, deterministic and identical on both engines but makes every
 * subsequent iteration count depend on instruction alignment rather than on the schedule.  With it,
 * each transaction starts from a settled controller and the counts mean what they say.
 */
function command(seq, o)
{
    let tag = o.tag || `cmd${o.pkt}`;
    if (o.clrint !== false) seq.push({s: "clrint"});
    if (o.hostWrites) seq.push({s: "wpkt", pkt: o.pkt, words: o.words});
    if (o.grant !== false) seq.push({s: "rdesc", slot: o.rslot, buf: o.rbuf === undefined ? o.pkt : o.rbuf});
    seq.push({s: "cdesc", slot: o.cslot, pkt: o.pkt, flag: o.flag, own: o.own});
    seq.push({s: "ip"});
    if (o.wait !== false) seq.push({s: "await", ring: "rq", slot: o.rslot, what: `${tag} response descriptor`});
    else seq.push({s: "delay", n: o.waitN === undefined ? 600 : o.waitN});
    seq.push({s: "snap", q: (o.comm - 4) >>> 0, what: `${tag} ring interrupt flag words`});
    if (o.settle !== false) seq.push({s: "delay", n: o.settle === undefined ? 400 : o.settle});
    return seq;
}

/* ------------------------------------------------------------------------------------------- *
 * The enumerated case list                                                                      *
 * ------------------------------------------------------------------------------------------- */

function enumeratedCases()
{
    let cases = [];
    let add = (spec) => { let c = buildCase(spec); c.idx = cases.length; cases.push(c); return c; };

    /* ---- ONE DESCRIPTOR EACH WAY, THE HOST BUILDING THE PACKET WITH REAL INSTRUCTIONS.  The
       simplest possible transaction, and the only family where the packet reaches memory through
       the CPU rather than through a deposit.  `ring->lnt <= 4` in rq_putdesc(), so BOTH rings
       interrupt unconditionally and both flag words must come back 1. ---- */
    for (let [cfl, tmo] of [[0x00F0, 30], [0x0000, 0]]) {
        let comm = 0x2000;
        add({
            name: `1x1 host-built SCC cfl=${hex(cfl, 4)} tmo=${tmo}`,
            kind: "hostbuilt", comm, spread: 0, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            steps: command([], {pkt: 0, cslot: 0, rslot: 0, comm, hostWrites: true,
                                words: cmdWords({cfl, tmo, ref: 0xBEEF})})
        });
    }

    /* ---- THE SAME TRANSACTION WITH A SCATTERED, HIGH COMM REGION.  comm above 64KB exercises the
       step-3 shift, and the purge-interrupt flag moves where rq_step4() starts zeroing. ---- */
    for (let prgi of [0, 1]) {
        let comm = 0x0A3000;
        add({
            name: `1x1 SCC comm=${hex(comm, 6)} prgi=${prgi}`,
            kind: "normal", comm, prgi, spread: 5, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            packets: {0: cmdWords({cfl: 0x0030, tmo: 7, ref: 0x0F0F})},
            steps: command([], {pkt: 0, cslot: 0, rslot: 0, comm})
        });
    }

    /* ---- FOUR DESCRIPTORS EACH WAY, NINE COMMANDS: the index wraps TWICE (idx runs 0,4,8,C,0,...
       and `(idx + 4) & (lnt - 1)` is a MASK, not a modulo).  Nine commands also walk the CREDIT
       HACK all the way down: the first end packet carries 15 credits and CRED drops 15 -> 1, the
       second carries 2 and CRED drops to 0, and every one after that carries 1. ---- */
    {
        let comm = 0x2000, nc = 9, pkts = {};
        for (let i = 0; i < 4; i++) pkts[i] = cmdWords({cfl: 0x0010 * (i + 1), tmo: 5 + i, ref: 0xC000 + i});
        let seq = [];
        for (let i = 0; i < nc; i++) {
            command(seq, {pkt: i & 3, cslot: i & 3, rslot: i & 3, comm, tag: `#${i}`});
        }
        add({
            name: "4x4 rings, nine commands -- the index wraps twice and the credits run out",
            kind: "wrap", comm, spread: 1, cqCode: 2, rqCode: 2, nCmdBuf: 4, nRspBuf: 4,
            packets: pkts, steps: seq
        });
    }

    /* ---- THE INTERRUPT RULE, BOTH WAYS, IN ONE CASE.  Four command descriptors and four response
       descriptors posted AT ONCE, then a single IP read.  rq_putdesc() interrupts only if the
       descriptor the host posted had UQ_DESC_F set AND the PREVIOUS slot is still host-owned; with
       every slot posted up front the previous slot IS owned for the first three releases and is NOT
       for the last, so the flag words come back differently within the same case.  The flag words
       are zeroed before each wait, so each snapshot is that release's own answer. ---- */
    {
        let comm = 0x2000, pkts = {};
        for (let i = 0; i < 4; i++) pkts[i] = cmdWords({cfl: 0x0080 + i, tmo: 11 + i, ref: 0xD000 + i});
        let seq = [{s: "clrint"}];
        for (let i = 0; i < 4; i++) seq.push({s: "rdesc", slot: i, buf: i});
        for (let i = 0; i < 4; i++) seq.push({s: "cdesc", slot: i, pkt: i});
        seq.push({s: "ip"});
        for (let i = 0; i < 4; i++) {
            seq.push({s: "await", ring: "rq", slot: i, what: `#${i} response descriptor`});
            seq.push({s: "snap", q: (comm - 4) >>> 0, what: `#${i} ring interrupt flag words`});
            seq.push({s: "clrint"});
        }
        seq.push({s: "delay", n: 600});
        add({
            name: "4x4 rings, four commands posted at once -- rq_putdesc's previous-slot rule both ways",
            kind: "intrule", comm, spread: 2, cqCode: 2, rqCode: 2, nCmdBuf: 4, nRspBuf: 4,
            packets: pkts, steps: seq
        });
    }

    /* ---- A DESCRIPTOR POSTED WITHOUT UQ_DESC_F.  rq_putdesc() still writes F back (the released
       value is `(desc & ~OWN) | F` unconditionally) but raises NO interrupt at all, on a
       one-descriptor ring where it otherwise always would.  Two observables that a model conflating
       "F in the value written" with "F in the value read" gets backwards. ---- */
    {
        let comm = 0x2000;
        add({
            name: "1x1 SCC with UQ_DESC_F clear -- released WITH F, but no ring interrupt",
            kind: "noflag", comm, spread: 3, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            packets: {0: cmdWords({cfl: 0x0044, tmo: 3})},
            steps: command([], {pkt: 0, cslot: 0, rslot: 0, comm, flag: false})
        });
    }

    /* ---- THE COMMAND DESCRIPTOR IS NOT OWNED.  The controller must find nothing, CLEAR `pip` and
       stop -- not spin, and not advance the ring index.  Posted after a completed command so the
       index it must NOT advance is a non-zero one. ---- */
    {
        let comm = 0x2000;
        let seq = command([], {pkt: 0, cslot: 0, rslot: 0, comm, tag: "first"});
        seq.push({s: "cdesc", slot: 1, pkt: 1, own: false});
        seq.push({s: "ip"});
        seq.push({s: "delay", n: 600});
        seq.push({s: "snap", q: (comm - 4) >>> 0, what: "ring interrupt flag words after the un-owned poll"});
        add({
            name: "4x4 rings, a command descriptor with OWN clear -- the poll stops and clears pip",
            kind: "noown", comm, spread: 4, cqCode: 2, rqCode: 2, nCmdBuf: 2, nRspBuf: 2,
            packets: {0: cmdWords({cfl: 0x0020, tmo: 9}), 1: cmdWords({cfl: 0x0021, tmo: 9})},
            steps: seq
        });
    }

    /* ---- THE RESPONSE RING IS FULL.  The host posts a command and grants NO response descriptor:
       rq_putpkt() cannot place the answer, so it ENQUEUES the packet on `rspq` (rq_enqt) and
       re-arms the queue thread, which retries and re-queues indefinitely.  The host waits, observes
       that the response buffer is STILL its seed and the descriptor STILL its own value, THEN grants
       the descriptor and waits again -- and the answer arrives, which is the proof the packet was
       retained rather than dropped.
       *** THE GRANT MUST HAPPEN BEFORE THE HALT. *** A HALT drains SIMH's event queue while the head
       is not UNIT_IDLE, and a queue thread re-arming forever never lets it finish: the real
       simulator hangs.  That is measured, and it is why this case is shaped the way it is. ---- */
    {
        let comm = 0x2000;
        let spec = {name: "the response ring is full -- the packet is queued and delivered after the grant",
                    kind: "respfull", comm, spread: 6, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
                    packets: {0: cmdWords({cfl: 0x0008, tmo: 21})}};
        let g = geometry(spec);
        spec.steps = [
            {s: "clrint"},
            {s: "cdesc", slot: 0, pkt: 0},
            {s: "ip"},
            {s: "delay", n: 900},
            {s: "snap", q: (comm - 4) >>> 0, what: "ring interrupt flag words while deferred"},
            /* The response buffer's own first longword, which must STILL BE THE PAGE SEED while the
               packet sits on rspq -- the observation that separates "deferred" from "delivered". */
            {s: "snap", q: g.rspBuf(0), what: "response buffer while deferred"},
            {s: "snap", q: (comm + 0) >>> 0, what: "response descriptor while deferred"},
            {s: "rdesc", slot: 0, buf: 0},
            {s: "await", ring: "rq", slot: 0, what: "deferred response, after the grant"},
            {s: "snap", q: (comm - 4) >>> 0, what: "ring interrupt flag words after delivery"},
            {s: "delay", n: 400}
        ];
        add(spec);
    }

    /* ---- A DESCRIPTOR CARRYING BITS UQ_ADDR DISCARDS.  UQ_ADDR is 0x003FFFFE: besides the
       ownership and flag bits it strips the ODD-BYTE BIT 0 and every bit above 21.  A host that
       sets them is not making an error the controller reports -- they are simply discarded -- and
       the released descriptor carries them BACK, because rq_putdesc() writes `(desc & ~OWN) | F`
       over the whole 32-bit word and never re-masks the address.  Both halves of that are graded,
       and without this case `desc & UQ_ADDR` is indistinguishable from no mask at all. ---- */
    {
        let comm = 0x2000;
        let spec = {name: "descriptors carrying the odd-byte bit and a bit above 21 -- UQ_ADDR discards both",
                    kind: "addrbits", comm, spread: 18, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
                    packets: {0: cmdWords({cfl: 0x0011, tmo: 25, ref: 0xADD5})}};
        let g = geometry(spec);
        let junk = (0x00400001) >>> 0;
        spec.steps = [
            {s: "clrint"},
            {s: "rawdesc", ring: "rq", slot: 0, v: (UQ_DESC_OWN | UQ_DESC_F | junk | g.rspEnv(0)) >>> 0},
            {s: "rawdesc", ring: "cq", slot: 0, v: (UQ_DESC_OWN | UQ_DESC_F | junk | g.cmdEnv(0)) >>> 0},
            {s: "ip"},
            {s: "await", ring: "rq", slot: 0, what: "response descriptor with junk address bits"},
            {s: "snap", q: (comm - 4) >>> 0, what: "ring interrupt flag words"},
            {s: "delay", n: 400}
        ];
        add(spec);
    }

    /* ---- rq_ring_int()'s FLAG WRITE IGNORES NXM, and this is the case that says so.  The two flag
       words live at comm-4 and comm-2, i.e. on the QBUS PAGE BELOW a page-aligned comm region --
       everything else the controller touches (both rings, both packet areas) is above it.  So the
       host completes the handshake with that page mapped (rq_step4 zeroes from comm-4 and would go
       PE_QWE otherwise), then INVALIDATES its map entry and runs a command.  The command must
       succeed with PERR still 0: the C casts rq_ring_int()'s Map_WriteW to void and its comment
       says "note that NXMs are ignored!".  The host reads the flag words back PHYSICALLY, which
       the map cannot stop, and they must still be zero.
       Any model that propagated that failure reports PE_QWE and dies here. ---- */
    {
        let comm = 0x2000;
        let spec = {name: "the ring interrupt flag words on an unmapped page -- the write is IGNORED",
                    kind: "ringint-nxm", comm, spread: 19, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
                    packets: {0: cmdWords({cfl: 0x0012, tmo: 27, ref: 0x1717})}};
        spec.steps = [
            {s: "clrint"},
            {s: "rdesc", slot: 0, buf: 0},
            {s: "cdesc", slot: 0, pkt: 0},
            {s: "went", q: ((comm - 8) / PAGE) | 0, v: 0},
            {s: "ip"},
            {s: "await", ring: "rq", slot: 0, what: "response with the flag page unmapped"},
            {s: "snap", q: (comm - 4) >>> 0, what: "ring interrupt flag words (must stay zero)"},
            {s: "delay", n: 400}
        ];
        add(spec);
    }

    /* ---- TWO DEFERRED RESPONSES, DELIVERED IN ORDER.  rq_putpkt() appends to the response queue
       with rq_enqt when it is called from rq_mscp and pushes back onto the HEAD with rq_enqh when
       the queue thread is re-trying one -- which together preserve arrival order across any number
       of retries.  With ONE response descriptor and TWO commands outstanding, the order is
       observable and nothing else is: both packets carry the same fields except their reference
       numbers, so the only difference a head/tail confusion makes is WHICH buffer gets which ref.
       The host grants the single descriptor twice, pointing it at a different buffer each time.
       `delay` rather than `await` after each grant, deliberately: an in-band wait would have to
       read its baseline AFTER the grant instruction, and the controller may legitimately deliver
       between those two instructions, so the wait would time out on both engines and grade
       nothing.  Order is what this case is for; the schedule is graded everywhere else. ---- */
    {
        let comm = 0x2000;
        let spec = {name: "two deferred responses are delivered in ARRIVAL order",
                    kind: "respq-order", comm, spread: 20, cqCode: 1, rqCode: 0, nCmdBuf: 2, nRspBuf: 2,
                    packets: {0: cmdWords({cfl: 0x0013, tmo: 29, ref: 0xB001, refh: 0x0001}),
                              1: cmdWords({cfl: 0x0014, tmo: 31, ref: 0xB002, refh: 0x0002})}};
        let g = geometry(spec);
        spec.steps = [
            {s: "clrint"},
            {s: "cdesc", slot: 0, pkt: 0},
            {s: "cdesc", slot: 1, pkt: 1},
            {s: "ip"},
            {s: "delay", n: 900},                           /* both fetched, both queued on rspq */
            {s: "rdesc", slot: 0, buf: 0},
            {s: "delay", n: 500},
            {s: "snap", q: (g.rspBuf(0) + CMD_REFL * 2) >>> 0, what: "reference number of the FIRST delivered response"},
            {s: "rdesc", slot: 0, buf: 1},
            {s: "delay", n: 500},
            {s: "snap", q: (g.rspBuf(1) + CMD_REFL * 2) >>> 0, what: "reference number of the SECOND delivered response"},
            {s: "snap", q: (comm - 4) >>> 0, what: "ring interrupt flag words"},
            {s: "delay", n: 400}
        ];
        add(spec);
    }

    /* ---- THE PROTOCOL ERRORS.  Each takes the controller FATAL, which resets it, so the case ends
       with a delay and an SA read-back rather than with a wait for a response that will never come.
       PERR survives rq_reset() (it is set after it), which is what makes each distinguishable. ---- */
    let fatalCase = (name, kind, words, extra = {}) => {
        let comm = 0x2000;
        let seq = [{s: "clrint"}];
        seq.push({s: "rdesc", slot: 0, buf: 0});
        seq.push({s: "cdesc", slot: 0, pkt: 0});
        seq.push({s: "ip"});
        seq.push({s: "delay", n: 600});
        seq.push({s: "snapsa", what: "SA after the fatal"});
        seq.push({s: "snap", q: (comm - 4) >>> 0, what: "ring interrupt flag words after the fatal"});
        add(Object.assign({
            name, kind, comm, spread: 7, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            packets: {0: words}, steps: seq
        }, extra));
    };
    fatalCase("UQ_HCTC type is not UQ_TYP_SEQ -> rq_fatal(PE_PIE)", "fatal-pie",
              cmdWords({typ: UQ_TYP_DAT, cfl: 0x0001, tmo: 1}));
    fatalCase("UQ_HCTC CID is neither MSCP nor DUP -> rq_fatal(PE_ICI)", "fatal-ici",
              cmdWords({cid: UQ_CID_DIAG, cfl: 0x0002, tmo: 2}));

    /* ---- A UQ_CID_DUP PACKET IS ANSWERED, NOT REJECTED: rq_quesvc builds an OP_END response with
       ST_CMD|I_OPCD directly, WITHOUT going through rq_mscp() at all -- so the opcode word in the
       response is 0x0080 whatever the command was. ---- */
    {
        let comm = 0x2000;
        add({
            name: "a UQ_CID_DUP packet gets an immediate ST_CMD|I_OPCD end packet",
            kind: "dup", comm, spread: 8, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            packets: {0: cmdWords({cid: UQ_CID_DUP, opc: OP.SCC, cfl: 0x0003, tmo: 4})},
            steps: command([], {pkt: 0, cslot: 0, rslot: 0, comm})
        });
    }

    /* ---- EVERY OPCODE rq_mscp() DOES NOT DISPATCH TO A UNIT.  The three no-op arms answer
       `cmd | OP_END` with ST_SUC; everything else answers OP_END alone with ST_CMD|I_OPCD, INCLUDING
       the named tape opcodes (OP_ERG, OP_WTM, OP_POS) and OP_AVA, which look like real commands and
       are not in this controller's switch.  The list is DERIVED from rq.js's own tables, so an
       opcode that changes classification changes this case list too (standing rule 5). ---- */
    {
        let inScope = [OP.SCC, ...RQVAX.MSCP_NOP_OPS.map((n) => OP[n])];
        let unhandled = Object.keys(OP).filter((n) => RQVAX.MSCP_UNIT_OPS.indexOf(n) < 0 &&
                                                      RQVAX.MSCP_NOP_OPS.indexOf(n) < 0 && n !== "SCC")
                                      .map((n) => OP[n]);
        let opcodes = [...inScope, ...unhandled, 0x00, 0x7F, 0xFE];
        let comm = 0x2000;
        for (let opc of opcodes) {
            add({
                name: `opcode ${hex(opc, 4)} (${RQVAX.OP_NAME_OF[opc] ? "OP_" + RQVAX.OP_NAME_OF[opc] : "unassigned"})`,
                kind: "opcode", comm, spread: (opc & 0x1F) + 9, cqCode: 0, rqCode: 0,
                nCmdBuf: 1, nRspBuf: 1,
                packets: {0: cmdWords({opc, cfl: 0x0055, tmo: 13, ref: 0x1000 + opc})},
                steps: command([], {pkt: 0, cslot: 0, rslot: 0, comm})
            });
        }
    }

    /* ---- rq_scc()'s FAILURE ARM.  A non-zero SCC_MSV is ST_CMD|I_VRSN, and `cmd` is set to ZERO
       rather than left as OP_SCC, so the response opcode word is 0x0080 -- and NONE of the
       controller-characteristics words is written, so the whole tail of the response is the host's
       own command data coming straight back.  CFLGS and HTMO must be unchanged on the oracle. ---- */
    {
        let comm = 0x2000;
        add({
            name: "SET CONTROLLER CHARACTERISTICS with a non-zero MSCP version -> ST_CMD|I_VRSN",
            kind: "sccver", comm, spread: 11, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            packets: {0: cmdWords({msv: 1, cfl: 0x00FF, tmo: 99})},
            steps: command([], {pkt: 0, cslot: 0, rslot: 0, comm})
        });
    }

    /* ---- A COMMAND DESCRIPTOR POINTING AT AN UNMAPPED QBUS PAGE.  rq_getpkt() gets the descriptor
       and the free packet, then the 64-byte packet read fails -> rq_fatal(PE_PRE).  The packet is
       NOT returned to the free list, so PBSY would be 1 -- except that rq_fatal() calls rq_reset()
       first, which is exactly the ordering pcjsvax-c2c graded and this case re-grades from a
       different call path. ---- */
    {
        let comm = 0x2000;
        let seq = [{s: "clrint"}];
        seq.push({s: "rdesc", slot: 0, buf: 0});
        /* A descriptor whose address is in the case's own extent but on a page left out of the map.
           `rawdesc` rather than `cdesc`, because there is no buffer index for an address the host
           cannot itself reach. */
        seq.push({s: "rawdesc", ring: "cq", slot: 0, v: 0});         // patched below
        seq.push({s: "ip"});
        seq.push({s: "delay", n: 600});
        seq.push({s: "snapsa", what: "SA after the packet-read fatal"});
        let spec = {
            name: "a command descriptor on an UNMAPPED page -> rq_fatal(PE_PRE)",
            kind: "fatal-pre", comm, spread: 12, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            steps: seq
        };
        let g = geometry(spec);
        seq[2].v = (UQ_DESC_OWN | UQ_DESC_F | g.cmdEnv(0)) >>> 0;
        add(Object.assign({}, spec, {unmappedQ: [(g.cmdBuf(0) / PAGE) | 0]}));
    }

    /* ---- A RESPONSE DESCRIPTOR POINTING AT AN UNMAPPED QBUS PAGE.  Same shape, other direction:
       the command is fetched and processed, the response is BUILT, and only the write-back fails ->
       rq_fatal(PE_PWE).  The DBG_REQ trace therefore shows both a `cmd=` line and an `rsp=` line
       before the controller dies, which is the ordering evidence a memory comparison cannot give. */
    {
        let comm = 0x2000;
        let seq = [{s: "clrint"}];
        seq.push({s: "rawdesc", ring: "rq", slot: 0, v: 0});         // patched below
        seq.push({s: "cdesc", slot: 0, pkt: 0});
        seq.push({s: "ip"});
        seq.push({s: "delay", n: 600});
        seq.push({s: "snapsa", what: "SA after the packet-write fatal"});
        let spec = {
            name: "a response descriptor on an UNMAPPED page -> rq_fatal(PE_PWE)",
            kind: "fatal-pwe", comm, spread: 13, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            packets: {0: cmdWords({cfl: 0x0006, tmo: 6})}, steps: seq
        };
        let g = geometry(spec);
        seq[1].v = (UQ_DESC_OWN | UQ_DESC_F | g.rspEnv(0)) >>> 0;
        add(Object.assign({}, spec, {unmappedQ: [(g.rspBuf(0) / PAGE) | 0]}));
    }

    /* ---- THE RING INTERRUPT'S FLAG WRITE IGNORES NXM.  The comm region's own page is mapped (it
       has to be -- rq_step4 zeroed it), so the flag word cannot be made unreachable by unmapping;
       what CAN be done is to INVALIDATE its map entry after the handshake, from the host, and then
       run a command.  rq_getdesc()'s read of the command ring fails first -> PE_QRE, which is the
       same page.  So this case grades the pair: a controller that treated rq_ring_int()'s write as
       fatal would report PE_QWE somewhere in the sequence and the oracle never does. ---- */
    {
        let comm = 0x2000;
        let seq = command([], {pkt: 0, cslot: 0, rslot: 0, comm, tag: "before"});
        let spec = {
            name: "the comm page's map entry invalidated after a command -> PE_QRE, never PE_QWE",
            kind: "fatal-qre", comm, spread: 14, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            packets: {0: cmdWords({cfl: 0x000C, tmo: 15})}, steps: seq
        };
        seq.push({s: "cdesc", slot: 0, pkt: 0});
        seq.push({s: "went", q: (comm / PAGE) | 0, v: 0});
        seq.push({s: "ip"});
        seq.push({s: "delay", n: 600});
        seq.push({s: "snapsa", what: "SA after the descriptor-read fatal"});
        add(spec);
    }

    /* ---- EXHAUST THE FREE PACKET LIST.  Thirty-two command descriptors posted at once on a
       32-slot ring, no response descriptors granted at all: every command allocates a packet that
       is then queued on `rspq` and never freed.  There are 31 packets (PACKET 0 IS THE LIST
       TERMINATOR and is never handed out), so the 32nd rq_deqf() finds `freq == 0` and takes the
       controller fatal with PE_NSR -- error 478, which is why `perr` is a NINE-bit register.
       The host then grants a response descriptor before halting, so the drain terminates. ---- */
    {
        let comm = 0x2000, n = 32, pkts = {};
        for (let i = 0; i < n; i++) pkts[i] = cmdWords({cfl: 0x0100 + i, tmo: 0, ref: 0xE000 + i});
        let seq = [{s: "clrint"}];
        for (let i = 0; i < n; i++) seq.push({s: "cdesc", slot: i, pkt: i});
        seq.push({s: "ip"});
        seq.push({s: "delay", n: 9000});
        seq.push({s: "snapsa", what: "SA after the free list ran dry"});
        add({
            name: "thirty-two commands with no response slots -- the 31 free packets run out (PE_NSR)",
            kind: "fatal-nsr", comm, spread: 15, cqCode: 5, rqCode: 0, nCmdBuf: n, nRspBuf: 1,
            packets: pkts, steps: seq
        });
    }

    /* ---- NON-DEFAULT QTIME.  The in-band iteration counts must MOVE with the register, which is
       what proves the response's schedule is driven by it rather than by a constant that happens to
       equal 100. ---- */
    for (let qt of [40, 400]) {
        let comm = 0x2000;
        add({
            name: `QTIME=${qt} -- the response wait must move with the register`,
            kind: "timing", comm, spread: 16, cqCode: 0, rqCode: 0, nCmdBuf: 1, nRspBuf: 1,
            qtime: qt,
            packets: {0: cmdWords({cfl: 0x0060, tmo: 17})},
            steps: command([], {pkt: 0, cslot: 0, rslot: 0, comm})
        });
    }

    /* ---- ASYMMETRIC RINGS.  A 1-descriptor command ring and an 8-descriptor response ring: the
       two rings' `lnt <= 4` decisions differ within a single transaction, so the command ring
       interrupts unconditionally and the response ring consults its previous slot. ---- */
    {
        let comm = 0x2000, seq = [], pkts = {};
        for (let i = 0; i < 3; i++) pkts[i] = cmdWords({cfl: 0x0090 + i, tmo: 23 + i, ref: 0xA100 + i});
        for (let i = 0; i < 3; i++) command(seq, {pkt: i, cslot: 0, rslot: i, comm, tag: `#${i}`});
        add({
            name: "asymmetric rings: one command descriptor, eight response descriptors",
            kind: "asym", comm, spread: 17, cqCode: 0, rqCode: 3, nCmdBuf: 3, nRspBuf: 3,
            packets: pkts, steps: seq
        });
    }

    return cases;
}

/**
 * randomCases(n, seed, startIdx)
 *
 * A structurally different view from the enumerated matrix, which is exhaustive at named boundaries
 * and blind between them: uniform draws over the ring length codes, the comm region, the purge
 * flag, the map scatter, QTIME, the number of commands, and each command's opcode, connection ID,
 * flag bit and host credit grant.
 *
 * FENCED, not filtered: the opcode pool EXCLUDES the twelve unit-bearing commands by DERIVING the
 * complement from rq.js's own table, and SA_S1H_VEC is structurally zero because buildCase() never
 * sets it.  A draw that could reach an excluded path would be a defect in the pool, and
 * assertExclusions() re-checks the resulting case list rather than trusting the draw.
 */
function randomCases(n, seed, startIdx)
{
    let rnd = mulberry32(seed);
    let pool = [];
    for (let v = 0; v <= 0xFF; v++) {
        let nm = RQVAX.OP_NAME_OF[v];
        if (nm && RQVAX.MSCP_UNIT_OPS.indexOf(nm) >= 0) continue;
        pool.push(v);
    }
    let out = [];
    for (let i = 0; i < n; i++) {
        let cqCode = Math.floor(rnd() * 4);                 /* 1..8 descriptors */
        let rqCode = Math.floor(rnd() * 4);
        let cqSlots = 1 << cqCode, rqSlots = 1 << rqCode;
        /* Word-aligned and PAGE-aligned across the whole 22-bit Qbus space, high enough that
           `comm - 8` never borrows below zero and low enough that the buffers stay inside it. */
        let comm = ((1 + Math.floor(rnd() * 0x600)) * PAGE) & 0x3FFE00;
        let prgi = rnd() < 0.5 ? 1 : 0;
        let qtime = [RQ_QTIME, 60, 250][Math.floor(rnd() * 3)];
        let spread = Math.floor(rnd() * DATA_NPAGE);
        let nc = 1 + Math.floor(rnd() * 5);
        let nbuf = Math.max(cqSlots, rqSlots, 1);
        let pkts = {}, seq = [];
        for (let k = 0; k < nbuf; k++) {
            pkts[k] = cmdWords({
                opc: pool[Math.floor(rnd() * pool.length)],
                cid: rnd() < 0.15 ? UQ_CID_DUP : UQ_CID_MSCP,
                cr: Math.floor(rnd() * 16),
                cfl: Math.floor(rnd() * 0x10000),
                tmo: Math.floor(rnd() * 200),
                msv: rnd() < 0.15 ? 1 : 0,
                mod: Math.floor(rnd() * 0x10000),
                ref: Math.floor(rnd() * 0x10000),
                hlnt: [SCC_LNT, RSP_LNT, 64][Math.floor(rnd() * 3)]
            });
        }
        for (let k = 0; k < nc; k++) {
            command(seq, {pkt: k % nbuf, cslot: k % cqSlots, rslot: k % rqSlots, comm,
                          flag: rnd() < 0.8, tag: `r#${k}`});
        }
        let c = buildCase({
            name: `random#${i} cq=${cqSlots} rq=${rqSlots} comm=${hex(comm, 6)} prgi=${prgi} ` +
                  `n=${nc} QTIME=${qtime}`,
            kind: "random", comm, prgi, spread, cqCode, rqCode, qtime,
            nCmdBuf: nbuf, nRspBuf: nbuf, packets: pkts, steps: seq
        });
        c.idx = startIdx + out.length;
        out.push(c);
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side -- ONE invocation for the whole case list                                       *
 * ------------------------------------------------------------------------------------------- */

const MARK = "MSRCASE";

function simhCaseLines(c)
{
    let L = [];
    L.push(`echo ${MARK}${c.idx}`);
    L.push(...simhResetLines(c));
    for (let p of c.dumpPages) L.push(`deposit -l ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)} ${hex(seedFor(p))}`);
    L.push(`deposit -l ${hex(c.resultPage * PAGE)}:${hex(c.resultPage * PAGE + PAGE - 4)} 0`);
    for (let pr of c.presets) L.push(`deposit -w ${hex(pr.addr)} ${hex(pr.word, 4)}`);
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
    L.push(`examine -h ${hex(c.resultPage * PAGE)}:${hex(c.resultPage * PAGE + PAGE - 4)}`);
    L.push("echo RINGS", "show rq rings", "echo FREEQ", "show rq freeq", "echo RESPQ", "show rq respq",
           "echo ENDCASE");
    return L;
}

function runCasesSimh(simh, opts, cases)
{
    /* `set debug stdout` + `set rq debug=REQ` is what makes view (3) exist.  It goes to the SAME
       stream as `echo`, so the trace lines fall inside their own case's chunk and their ORDER
       relative to everything else is preserved. */
    let L = [`set cpu ${MEM_MB}m`, "set cpu simhalt", "set rq rqdx3",
             "set debug stdout", "set rq debug=REQ"];
    for (let c of cases) L.push(...simhCaseLines(c));
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "mscpring-cases.ini"));

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
    r.pkts = [];
    for (let i of PKT_PROBES) {
        let m = new RegExp(`^PKTS\\[${i}\\]:\\s*([0-9A-Fa-f]+)`, "m").exec(chunk);
        if (!m) return null;
        r.pkts.push(parseInt(m[1], 16) >>> 0);
    }
    r.mem = new Map();
    let re = /^([0-9A-F]{6,8}):\s*([0-9A-F]{8})\s*$/gm, m;
    while ((m = re.exec(chunk)) !== null) r.mem.set(parseInt(m[1], 16) >>> 0, parseInt(m[2], 16) >>> 0);
    /* The DBG_REQ trace.  `DBG(<sim_time>)> RQ REQ: <text>` -- the timestamp is captured but only
       its DELTAS within a case are meaningful, because sim_time accumulates across the whole
       do-file and every case starts wherever the previous one stopped. */
    /* *** SIMH COLLAPSES CONSECUTIVE IDENTICAL DEBUG LINES *** (scp.c:13836-13900): a run of N+1
       identical lines is printed as the first one followed by `same as above (N times)`, stamped
       with the time of the LAST occurrence.  Parsing that literally makes eleven deferred-response
       retries look like one -- exactly the kind of silence a differential must not have -- so the
       run is EXPANDED back to N+1 lines here.  It also means the individual TIMESTAMPS of a
       repeated event do not exist in the oracle's output, which is why only the ORDER and the TEXT
       of the trace are graded; rq.js's own header carries the same note. */
    r.trace = [];
    /* Note the two SHAPES: a real line is `DBG(t)> RQ REQ: text`, but the collapse line carries only
       the prefix up to `)> ` (scp.c copies `debug_line_last_prefix` and appends the count), so it is
       `DBG(t)> same as above (N times)` with NO device tag.  Anchoring the pattern on `RQ REQ:`
       therefore misses every collapse line -- which reads as "the oracle only did it once". */
    let tre = /^DBG\(\s*([0-9.]+)\)>\s*(.*?)\s*$/gm, tm;
    while ((tm = tre.exec(chunk)) !== null) {
        let run = /^same as above \((\d+) times?\)$/.exec(tm[2]);
        if (!run) {
            let body = /^RQ\s+REQ:\s*(.*)$/.exec(tm[2]);
            /* Only RQ's REQ flag is enabled, so any other debug line means the do-file turned on
               something this parser cannot place in the sequence.  Refusing the case beats guessing
               where it belongs (standing rule 6). */
            if (!body) return null;
            r.trace.push({t: parseFloat(tm[1]), line: body[1]});
            continue;
        }
        {
            /* A run whose first line landed in ANOTHER case's chunk cannot be expanded, and
               guessing would silently attribute one case's events to another.  Report the whole
               case unreadable instead (standing rule 6). */
            if (!r.trace.length) return null;
            let prev = r.trace[r.trace.length - 1].line;
            for (let k = 0; k < +run[1]; k++) r.trace.push({t: parseFloat(tm[1]), line: prev});
        }
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
 * The JS side                                                                                   *
 * ------------------------------------------------------------------------------------------- */

function runCaseJS(c, mutationOpts = {})
{
    let m = machine(mutationOpts);
    let {bus, cpu, cqbic, rq} = m;

    jsResetForCase(m, c);
    for (let p of c.dumpPages) {
        let s = seedFor(p);
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) bus.setLong(a >>> 0, s);
    }
    for (let a = c.resultPage * PAGE; a < c.resultPage * PAGE + PAGE; a += 4) bus.setLong(a >>> 0, 0);
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
        rq: {}, mem: new Map(), unimplemented,
        halted, atOwnHalt: halted && (cpu.regs[15] >>> 0) === c.haltPC
    };
    for (let o of RQ_OBS) r.rq[o.name] = rqFieldOf(rq, o);
    r.pkts = PKT_PROBES.map((i) => pktWord(rq, i));
    for (let p of c.dumpPages) {
        for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) r.mem.set(a >>> 0, bus.getLong(a) >>> 0);
    }
    for (let a = c.resultPage * PAGE; a < c.resultPage * PAGE + PAGE; a += 4) {
        r.mem.set(a >>> 0, bus.getLong(a) >>> 0);
    }
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

/** The RESULT-page longword a case's slot describes, so a difference is reported as "#3's response
    descriptor came back X" rather than as "0x108024 differs". */
function slotOf(c, addr)
{
    let off = (addr >>> 0) - R_RESULT;
    if (off < 0 || off >= PAGE) return null;
    for (let s of c.slots) if (s.off === off) return s;
    return null;
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
        if (!s.atOwnHalt || !j.atOwnHalt) {
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
        let memDiffs = 0;
        for (let [a, v] of j.mem) {
            let sv = s.mem.get(a);
            if (sv === undefined) {
                failures.push(`case ${c.idx} "${c.name}": the oracle never reported memory at 0x${hex(a)}`);
                break;
            }
            if (sv !== v && memDiffs++ < 5) {
                let sl = slotOf(c, a);
                failures.push(`case ${c.idx} "${c.name}": memory 0x${hex(a)}` +
                    (sl ? ` [${sl.what}]` : "") + ` = ${hex(v)} here, ${hex(sv)} on the oracle`);
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
    }
    return compared;
}

/**
 * gradeTrace(c, s, j, failures)
 *
 * View (3): the ordered `set rq debug=REQ` stream, compared as a SEQUENCE.  This is the parent
 * item's criterion -- "every command issued, every response returned, IN ORDER, with matching
 * fields" -- and it is the only one of the three views that can distinguish "the right answers"
 * from "the right answers in the right order".
 *
 * The LENGTH is compared before the lines, so a run that produced half the events is reported as
 * that rather than as a first-line difference.  Every line is compared, not a prefix.
 */
function gradeTrace(c, s, j, failures)
{
    let sl = s.trace.map((e) => e.line), jl = j.trace.map((e) => e.line);
    if (sl.length !== jl.length) {
        failures.push(`case ${c.idx} "${c.name}": the DEBUG=REQ trace has ${jl.length} line(s) here and ` +
            `${sl.length} on the oracle.\n    here:   ${JSON.stringify(jl).slice(0, 400)}\n` +
            `    oracle: ${JSON.stringify(sl).slice(0, 400)}`);
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
 * Coverage floors and exclusion fences.  Every one FAILS the run; none scales with case count.   *
 * ------------------------------------------------------------------------------------------- */

/**
 * assertExclusions(cases, sim, failures)
 *
 * The named exclusions, enforced as PROPERTIES OF THE CASE LIST AND OF THE ORACLE'S ANSWERS rather
 * than as promises.  Standing rule 6: an exclusion that is merely unvisited is a gap; one that
 * fails the run when it is reached is a fence.
 */
function assertExclusions(cases, sim, failures)
{
    for (let c of cases) {
        if (c.s1dat & SA_S1H_VEC) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" supplies an S1 word 0x${hex(c.s1dat, 4)} ` +
                `with a non-zero SA_S1H_VEC.  rq_ring_int() raises an interrupt on VEC ALONE -- it ` +
                `does NOT test SA_S1H_IE -- and that asymmetry is graded by tests/mscpintdiff.js ` +
                `(pcjsvax-aef), which LANDED delivery.  This fence is now a SCOPE boundary, not a ` +
                `gap: this file grades awaitL()'s ITERATION COUNTS, and an SCB dispatch executing ` +
                `inside those loops would fold interrupt delivery into a measurement whose subject ` +
                `is the controller's event schedule -- and no SCB handler is installed for the RQ ` +
                `vector here, so one would dispatch to a zero SCB slot and HALT at PC 1.`);
        }
        if (c.s1dat & SA_S1H_IE) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" supplies an S1 word with SA_S1H_IE set`);
        }
        /* EVERY packet a case plants, however it plants it: the enumerated `packets` map (physical
           deposits) AND the `wpkt` steps (real host instructions).  A fence that only looked at one
           of the two would be blind to exactly the family that proves the CPU path works. */
        let planted = Object.keys(c.packets || {}).map((i) => c.packets[i])
                        .concat(c.steps.filter((st) => st.s === "wpkt").map((st) => st.words));
        for (let w of planted) {
            let opc = w[CMD_OPC] & 0xFF;
            let nm = RQVAX.OP_NAME_OF[opc];
            if (nm && RQVAX.MSCP_UNIT_OPS.indexOf(nm) >= 0) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" plants a command packet with opcode ` +
                    `OP_${nm} (${opc}), which rq_mscp() dispatches to a UNIT-bearing handler.  Those ` +
                    `twelve commands are graded by tests/mscpunitdiff.js (pcjsvax-f52) and ` +
                    `tests/mscprwdiff.js (pcjsvax-346), not here -- this item's argument is about ` +
                    `the ring machinery and must not borrow another's coverage.`);
            }
        }
    }
    for (let i = 0; i < cases.length; i++) {
        let s = sim[i];
        if (!s) continue;
        /* rq_tmrsvc() decrements HAT once per WALL-CLOCK second.  A completed command legitimately
           sets HAT to 0 (in flight) or to HTMO (idle); anything between is the timer having fired. */
        if (s.rq.HAT !== 0 && s.rq.HAT !== s.rq.HTMO) {
            failures.push(`exclusion: case ${cases[i].idx} "${cases[i].name}" left the ORACLE with ` +
                `HAT=${s.rq.HAT}, which is neither 0 nor HTMO=${s.rq.HTMO} -- rq_tmrsvc(), the ` +
                `once-per-second WALL-CLOCK host-access timer, fired.  It is not modelled.`);
        }
    }
}

/**
 * assertWaits(cases, sim, js, failures)
 *
 * Every in-band wait must have SEEN ITS ANSWER on both engines, not run out of budget.  The third
 * longword of each `await` slot is the SOBGTR counter's remainder; zero means the loop gave up, and
 * two engines that both gave up compare equal while grading nothing (standing rule 6).
 */
function assertWaits(cases, sim, js, failures, acc)
{
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i];
        if (!s || !s.mem) continue;
        for (let st of c.steps) {
            if (st.s !== "await") continue;
            let budget = s.mem.get((R_RESULT + st.roff + 8) >>> 0);
            let count = s.mem.get((R_RESULT + st.roff) >>> 0);
            if (budget === undefined) continue;
            if (budget === 0) {
                failures.push(`case ${c.idx} "${c.name}": the in-band wait for ${st.what} EXHAUSTED its ` +
                    `${AWAIT_LIMIT}-iteration budget on the ORACLE -- the response never arrived, so ` +
                    `this case grades nothing about when it did`);
            } else {
                acc.waitCounts.add(count);
                if (count > 0) acc.nonZeroWaits++;
                else acc.zeroWaits++;
            }
        }
    }
}

function coverage(cases, sim, js, failures, acc)
{
    let ok = (i) => sim[i] && js[i] && !js[i].unimplemented && sim[i].halted && js[i].halted &&
                    sim[i].atOwnHalt && js[i].atOwnHalt;

    /* THE SYNCHRONY FLOOR.  At least three DISTINCT in-band iteration counts must have been
       observed and matched, and NOT ALL of them may be zero.  A controller that answers inside the
       IP read produces zero for every wait; one with a hard-coded delay produces one value for all
       of them.  This is the floor the "cheat to avoid" in pcjsvax-0b4 names by name. */
    if (acc.waitCounts.size < 3) {
        failures.push(`coverage: only ${acc.waitCounts.size} distinct in-band response-wait ITERATION ` +
            `COUNT(s) reached comparison; the floor is 3, because fewer cannot tell a SCHEDULED ` +
            `response from a synchronous one or from a hard-coded delay`);
    }
    if (acc.nonZeroWaits === 0) {
        failures.push(`coverage: every graded response wait saw its answer on the FIRST iteration -- ` +
            `the controller is answering synchronously inside the IP read and the whole event model ` +
            `is unexercised`);
    }

    /* THE WRAP FLOOR.  Some graded case must have driven a ring index past its wrap TWICE.  Counted
       from the ORACLE's own CQIDX trajectory?  No -- the index is only visible at the end.  It is
       counted instead from the number of releases a case performs on a ring of known length, which
       is a property of the case, AND corroborated by requiring the oracle to have ended such a case
       with the index the wrap arithmetic predicts. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        let c = cases[i];
        let releases = c.steps.filter((st) => st.s === "cdesc" && st.own !== false).length;
        if (c.g.cqSlots > 1 && releases >= 2 * c.g.cqSlots + 1) acc.wrappedTwice = true;
        if (c.g.cqSlots === 1) acc.oneSlotRing = true;
        if (c.g.cqSlots >= 4) acc.fourSlotRing = true;
    }
    if (!acc.oneSlotRing) failures.push(`coverage: no graded case used a ONE-descriptor command ring`);
    if (!acc.fourSlotRing) failures.push(`coverage: no graded case used a command ring of FOUR or more descriptors`);
    if (!acc.wrappedTwice) {
        failures.push(`coverage: no graded case drove a ring index past its wrap TWICE, so ` +
            `\`(idx + 4) & (lnt - 1)\` is indistinguishable from an index that only ever grows`);
    }

    /* THE INTERRUPT-RULE FLOOR.  Both answers to rq_putdesc()'s question must have been observed on
       the ORACLE, in the interrupt flag words the host read back: at least one release that set a
       flag word and at least one that did not. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let st of cases[i].steps) {
            if (st.s !== "snap" || !/ring interrupt flag words/.test(st.what || "")) continue;
            let v = sim[i].mem.get((R_RESULT + st.roff) >>> 0);
            if (v === undefined) continue;
            /* Low half = the COMMAND ring's flag (comm-4), high half = the RESPONSE ring's
               (comm-2).  Each is counted on its own, so a case where one ring interrupted and the
               other did not registers BOTH answers. */
            for (let half of [v & 0xFFFF, (v >>> 16) & 0xFFFF]) {
                if (half) acc.intFired = true; else acc.intSilent = true;
            }
        }
    }
    if (!acc.intFired) {
        failures.push(`coverage: the oracle never wrote a ring INTERRUPT FLAG WORD in any graded case, ` +
            `so rq_ring_int() is unexercised`);
    }
    if (!acc.intSilent) {
        failures.push(`coverage: every graded release wrote a ring interrupt flag word, so ` +
            `rq_putdesc()'s previous-descriptor rule is indistinguishable from "always interrupt"`);
    }

    /* THE CREDIT FLOOR.  The oracle must have been observed with at least THREE distinct CRED
       values across graded cases, including the initial 15 and the exhausted 0 -- fewer cannot tell
       the credit hack from a constant. */
    for (let i = 0; i < cases.length; i++) if (ok(i)) acc.credValues.add(sim[i].rq.CRED);
    if (acc.credValues.size < 3 || !acc.credValues.has(0)) {
        failures.push(`coverage: the oracle ended graded cases with CRED in {${[...acc.credValues].join(",")}}; ` +
            `the floor is at least three distinct values INCLUDING 0, because the credit hack sends ` +
            `min(credits,14)+1 on the first end packet and 1 thereafter`);
    }

    /* THE FREE-LIST FLOOR.  A packet must have been ALLOCATED and RETURNED on the oracle (PBSY back
       to 0 with FREE back to 1 after a completed command) and, separately, the pool must have been
       observed EXHAUSTED. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        if (sim[i].rq.PBSY === 0 && sim[i].rq.FREE === 1 && sim[i].trace.some((e) => /^rsp=/.test(e.line))) {
            acc.freeRoundTrip = true;
        }
    }
    if (!acc.freeRoundTrip) {
        failures.push(`coverage: no graded case both delivered a response AND returned its packet to ` +
            `the head of the free list`);
    }

    /* Each named behaviour must have been OBSERVED on the oracle at least once. */
    let sawKind = (k) => cases.some((c, i) => ok(i) && c.kind === k);
    for (let [k, what] of [
        ["hostbuilt", "a command packet built in Qbus memory by REAL CPU INSTRUCTIONS"],
        ["normal",    "a scattered, high comm region"],
        ["wrap",      "a multi-descriptor ring driven past wrap-around"],
        ["intrule",   "rq_putdesc's previous-descriptor interrupt rule"],
        ["noflag",    "a descriptor posted WITHOUT UQ_DESC_F"],
        ["noown",     "a command descriptor with OWN clear"],
        ["respfull",  "a full response ring, with the packet queued and delivered later"],
        ["respq-order", "two deferred responses delivered in arrival order"],
        ["addrbits",  "a descriptor carrying bits UQ_ADDR discards"],
        ["ringint-nxm", "a ring interrupt flag word whose DMA fails and is ignored"],
        ["fatal-pie", "a non-sequential packet type (PE_PIE)"],
        ["fatal-ici", "an unknown connection ID (PE_ICI)"],
        ["fatal-pre", "a packet READ failure (PE_PRE)"],
        ["fatal-pwe", "a packet WRITE failure (PE_PWE)"],
        ["fatal-qre", "a descriptor READ failure (PE_QRE)"],
        ["fatal-nsr", "the free packet list running dry (PE_NSR)"],
        ["dup",       "a UQ_CID_DUP packet"],
        ["opcode",    "the opcode sweep over everything rq_mscp() does not send to a unit"],
        ["sccver",    "SET CONTROLLER CHARACTERISTICS with a bad MSCP version"],
        ["timing",    "a response under a non-default QTIME"],
        ["asym",      "rings of DIFFERENT lengths"],
        ["random",    "the randomized phase"]
    ]) {
        if (!sawKind(k)) failures.push(`coverage: no graded case exercised ${what}`);
    }

    /* The port errors must have been READ OUT OF THE ORACLE, not predicted here. */
    let sawErr = (code) => cases.some((c, i) => ok(i) && sim[i].rq.PERR === code && sim[i].rq.CSTA === CST_DEAD);
    for (let [code, name] of [[PE_PIE, "PE_PIE"], [PE_ICI, "PE_ICI"], [PE_PRE, "PE_PRE"],
                              [PE_PWE, "PE_PWE"], [PE_QRE, "PE_QRE"], [PE_NSR, "PE_NSR"]]) {
        if (!sawErr(code)) {
            failures.push(`coverage: the oracle was never observed DEAD with PERR = ${code} (${name})`);
        }
    }

    /* THE ANSWER FLOOR.  At least three DISTINCT response STATUS words and three distinct response
       OPCODE words must have been seen in the oracle's own trace -- a controller that answered every
       command identically would satisfy every byte comparison in a single-status case list. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let e of sim[i].trace) {
            let m = /^rsp=([0-9A-F]{4}), sts=([0-9A-F]{4})$/.exec(e.line);
            if (m) { acc.rspOpc.add(m[1]); acc.rspSts.add(m[2]); }
            if (/^cmd=/.test(e.line)) acc.cmdLines++;
        }
    }
    if (acc.rspSts.size < 3) {
        failures.push(`coverage: only ${acc.rspSts.size} distinct response STATUS word(s) in the oracle's ` +
            `trace; the floor is 3 (ST_SUC, ST_CMD|I_OPCD, ST_CMD|I_VRSN all occur in the case list)`);
    }
    if (acc.rspOpc.size < 3) {
        failures.push(`coverage: only ${acc.rspOpc.size} distinct response OPCODE word(s) in the oracle's trace`);
    }
    if (acc.cmdLines < 40) {
        failures.push(`coverage: the oracle's trace carried only ${acc.cmdLines} command line(s); a case ` +
            `list this size must issue far more, so most cases never reached a command at all`);
    }

    /* THE SCATTER FLOOR: at least one graded case's Qbus region must have landed on discontiguous,
       DESCENDING physical pages -- the arrangement an identity map cannot produce. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i) || cases[i].entries.length < 3) continue;
        let ps = cases[i].entries.map((e) => e.p);
        let descending = ps.every((p, k) => k === 0 || p < ps[k - 1]);
        let gap = ps.some((p, k) => k > 0 && Math.abs(p - ps[k - 1]) > 1);
        if (descending && gap) acc.scatteredPages = true;
    }
    if (!acc.scatteredPages) {
        failures.push(`coverage: no graded case placed its rings and packets on discontiguous, ` +
            `DESCENDING physical pages -- an identity mapping would satisfy every case that ran`);
    }

    /* THE ROUND-TRIP FLOOR.  Some graded case's RESPONSE buffer must contain, on the oracle, a word
       the controller never assigns -- i.e. the page seed the HOST put in the COMMAND packet's tail,
       carried through.  Without this a controller that reconstructed only the fields a command
       defines would satisfy every comparison. */
    /* The COMMAND's reference number (CMD_REFL/CMD_REFH) is a longword rq_putr() never touches and
       no handler assigns.  Finding the host's own value in the RESPONSE buffer, on the ORACLE, is
       the direct proof that the whole packet travelled in and back out rather than being
       reconstructed from the fields a command defines. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        let c = cases[i];
        for (let k of Object.keys(c.packets || {})) {
            let w = c.packets[k];
            if (w[CMD_REFL] === undefined) continue;
            let rq = ((w[CMD_REFH] & 0xFFFF) << 16 | (w[CMD_REFL] & 0xFFFF)) >>> 0;
            for (let j = 0; j < c.nRspBuf; j++) {
                let qa = (c.g.rspBuf(j) + CMD_REFL * 2) >>> 0;
                if (!c.qToP.has((qa / PAGE) | 0)) continue;
                if (sim[i].mem.get(c.phys(qa)) === rq && rq !== 0) acc.roundTrip = true;
            }
        }
    }
    if (!acc.roundTrip) {
        failures.push(`coverage: no graded case's response buffer ever carried the COMMAND's own ` +
            `reference number back to the host, so the 64-byte round trip through mapReadW/mapWriteW ` +
            `is unproven -- a controller that reconstructed only the fields a handler assigns would ` +
            `satisfy every comparison above`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11)         *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* NOTE ON A MUTATION THAT IS NOT HERE: `descriptor-address-not-masked-with-UQ_ADDR`.
       *** `desc & UQ_ADDR` IS REDUNDANT, MEASURED, IN BOTH ENGINES. ***  UQ_ADDR (0x003FFFFE)
       strips the two flag bits, every bit above 21 and the odd-byte bit -- and Map_ReadW /
       Map_WriteW open with `ba = ba & QBMAMASK & ~01` (vax_io.c:774/807, and cqbic.js's
       mapReadW/mapWriteW at the same point), which strips exactly the same bits again.  Every
       address the mask could change is re-masked identically before it reaches memory, so NO host
       program can tell a controller that applies UQ_ADDR from one that does not.  Two mutations
       were written for it and both were measured to SURVIVE: `desc & (UQ_ADDR|1)`, which is a
       no-op for any even address, and `desc & ~(UQ_DESC_OWN|UQ_DESC_F)`, which the CQBIC undoes.
       The `addrbits` case above stays, because what it DOES prove is real: a descriptor carrying
       those bits is accepted, used, and RELEASED WITH THEM STILL SET, since rq_putdesc() rewrites
       the whole 32-bit word without re-masking the address.  Leaving a mutation here that can
       never fail would report coverage instead of measuring it -- HANDOFF.md standing rule 11.

    --- the seven falsifiable ones of the eight the item names --- */
    "packet-fetched-from-the-descriptor-address-not-the-header": () => {
        let orig = RQVAX.prototype.hdrAddr;
        RQVAX.prototype.hdrAddr = function(addr) { return addr >>> 0; };
        return () => { RQVAX.prototype.hdrAddr = orig; };
    },
    "UQ_DESC_F-never-set-on-release": () => {
        let orig = RQVAX.prototype.releasedDesc;
        RQVAX.prototype.releasedDesc = function(d) { return (orig.call(this, d) & ~UQ_DESC_F) >>> 0; };
        return () => { RQVAX.prototype.releasedDesc = orig; };
    },
    "ring-index-advanced-by-one-instead-of-four": () => {
        let orig = RQVAX.prototype.nextSlot;
        RQVAX.prototype.nextSlot = function(ring) { return (ring.idx + 1) & (ring.lnt - 1); };
        return () => { RQVAX.prototype.nextSlot = orig; };
    },
    "previous-slot-probe-not-masked-at-wrap": () => {
        /* `idx - 4` WITHOUT the `& (lnt - 1)`: identical everywhere except at idx 0, where the C
           borrows to 0xFFFFFFFC and the mask selects the LAST slot.  Invisible to any case that
           never wraps, which is why the wrap floor is a floor. */
        let orig = RQVAX.prototype.prevSlot;
        RQVAX.prototype.prevSlot = function(ring) { return (ring.idx - 4) >>> 0; };
        return () => { RQVAX.prototype.prevSlot = orig; };
    },
    "credits-never-decremented": () => {
        let orig = RQVAX.prototype.putPkt;
        RQVAX.prototype.putPkt = function(pkt, qt) {
            let before = this.credits;
            let r = orig.call(this, pkt, qt);
            this.credits = before;
            return r;
        };
        return () => { RQVAX.prototype.putPkt = orig; };
    },
    "response-length-omits-the-four-header-bytes": () => {
        let orig = RQVAX.prototype.responseLength;
        RQVAX.prototype.responseLength = function(pkt) { return (orig.call(this, pkt) + UQ_HDR_OFF) >>> 0; };
        return () => { RQVAX.prototype.responseLength = orig; };
    },
    "packet-0-handed-out-as-free": () => {
        /* rq_reset() links packet 0 to 0 and starts `freq` at 1.  Making 0 the head hands out the
           LIST TERMINATOR: `enqh` becomes a no-op and `if (pkt)` false, so the packet vanishes. */
        let orig = RQVAX.prototype.reset;
        RQVAX.prototype.reset = function() { orig.call(this); this.pakLink[0] = 1; this.freq = 0; };
        return () => { RQVAX.prototype.reset = orig; };
    },

    /* --- the two named cheats --- */
    "answers-synchronously-inside-the-IP-read": () => {
        /* THE CHEAT.  The IP read still sets `pip` and still schedules the queue -- and then runs
           the service immediately, so the response is already in memory when the host's first poll
           iteration looks.  Every byte of every page still matches; only the IN-BAND ITERATION
           COUNTS do not. */
        let orig = RQVAX.prototype.rd;
        RQVAX.prototype.rd = function(pa) {
            let v = orig.call(this, pa);
            if (((pa >>> 1) & 1) === 0 && this.pip) { this.queDue = null; this.quesvc(); }
            return v;
        };
        return () => { RQVAX.prototype.rd = orig; };
    },
    "only-the-fields-the-handler-assigns-are-written-back": () => {
        /* THE SECOND CHEAT: write back the response header and the eighteen words rq_scc() and
           rq_putr() actually assign, and leave the rest of the host's buffer alone. */
        let orig = RQVAX.prototype.responseLength;
        RQVAX.prototype.responseLength = function(pkt) { return Math.min(orig.call(this, pkt), 24); };
        return () => { RQVAX.prototype.responseLength = orig; };
    },

    /* --- the rest of the ring machinery --- */
    "OWN-not-cleared-on-release": () => {
        let orig = RQVAX.prototype.releasedDesc;
        RQVAX.prototype.releasedDesc = function(d) { return (orig.call(this, d) | (d & UQ_DESC_OWN)) >>> 0; };
        return () => { RQVAX.prototype.releasedDesc = orig; };
    },
    "the-interrupt-rule-is-inverted": () => {
        let orig = RQVAX.prototype.putDesc;
        RQVAX.prototype.putDesc = function(ring, desc) {
            /* Flip the ONE bit the rule reads, then let the shipped code decide: a release that
               should have interrupted does not, and one that should not, does. */
            return orig.call(this, ring, (desc ^ UQ_DESC_F) >>> 0);
        };
        return () => { RQVAX.prototype.putDesc = orig; };
    },
    "one-descriptor-rings-consult-a-previous-slot": () => {
        /* `ring->lnt <= 4` is the arm that makes a single-descriptor ring interrupt every time.
           Removing it sends a 4-byte ring down the previous-slot path, where `(idx-4) & 3` is the
           SAME slot -- which the controller has just released, so it never interrupts. */
        let orig = RQVAX.prototype.putDesc;
        RQVAX.prototype.putDesc = function(ring, desc) {
            let saved = ring.lnt;
            if (ring.lnt <= 4) ring.lnt = 8;
            let r = orig.call(this, ring, desc);
            ring.lnt = saved;
            return r;
        };
        return () => { RQVAX.prototype.putDesc = orig; };
    },
    "the-ring-interrupt-flag-write-is-treated-as-fatal": () => {
        /* rq_ring_int() DISCARDS its Map_WriteW result -- the C says "note that NXMs are ignored!".
           Honouring it turns a benign flag write into a port error. */
        let orig = RQVAX.prototype.ringInt;
        RQVAX.prototype.ringInt = function(ring) {
            /* Composed over the shipped ringInt(): intercept the ONE DMA it makes, keep its result,
               and let the shipped code run unchanged.  The interception is removed exactly as it was
               installed -- the CQBIC's method is inherited, so restoring it by assignment would
               leave an own property shadowing the prototype for every later pass. */
            let had = Object.prototype.hasOwnProperty.call(this.cqbic, "mapWriteW");
            let prev = this.cqbic.mapWriteW, hit = 0;
            this.cqbic.mapWriteW = function(...a) { hit = prev.apply(this, a); return hit; };
            try { orig.call(this, ring); }
            finally { if (had) this.cqbic.mapWriteW = prev; else delete this.cqbic.mapWriteW; }
            if (hit) this.fatal(PE_QWE);
        };
        return () => { RQVAX.prototype.ringInt = orig; };
    },
    "the-ring-interrupt-flag-goes-to-the-wrong-ring": () => {
        let orig = RQVAX.prototype.ringInt;
        RQVAX.prototype.ringInt = function(ring) {
            return orig.call(this, ring === this.cq ? this.rq : this.cq);
        };
        return () => { RQVAX.prototype.ringInt = orig; };
    },
    "the-two-rings-are-swapped-in-memory": () => {
        /* rq_step4 lays the RESPONSE ring at `comm` and the COMMAND ring above it.  The opposite
           order is the single most likely transcription error in this file. */
        let orig = RQVAX.prototype.step4;
        RQVAX.prototype.step4 = function() {
            let r = orig.call(this);
            let a = this.rq.ba; this.rq.ba = this.cq.ba; this.cq.ba = a;
            return r;
        };
        return () => { RQVAX.prototype.step4 = orig; };
    },
    "the-free-packet-is-returned-to-the-TAIL": () => {
        let orig = RQVAX.prototype.enqh;
        RQVAX.prototype.enqh = function(obj, key, pkt) {
            if (key === "freq") return this.enqt(obj, key, pkt);
            return orig.call(this, obj, key, pkt);
        };
        return () => { RQVAX.prototype.enqh = orig; };
    },
    "a-deferred-response-is-queued-at-the-HEAD": () => {
        let orig = RQVAX.prototype.enqt;
        RQVAX.prototype.enqt = function(obj, key, pkt) {
            if (key === "rspq") return this.enqh(obj, key, pkt);
            return orig.call(this, obj, key, pkt);
        };
        return () => { RQVAX.prototype.enqt = orig; };
    },
    "pbsy-is-not-counted": () => {
        let orig = RQVAX.prototype.deqf;
        RQVAX.prototype.deqf = function() {
            let r = orig.call(this);
            if (r !== null) this.pbsy = this.pbsy - 1;
            return r;
        };
        return () => { RQVAX.prototype.deqf = orig; };
    },
    "an-unknown-opcode-answers-ST_SUC": () => {
        let orig = RQVAX.prototype.putr;
        RQVAX.prototype.putr = function(pkt, cmd, flg, sts, lnt, typ) {
            return orig.call(this, pkt, cmd, flg, sts === (ST.CMD | I_OPCD) ? ST.SUC : sts, lnt, typ);
        };
        return () => { RQVAX.prototype.putr = orig; };
    },
    "the-response-opcode-keeps-the-command-opcode-instead-of-OP_END": () => {
        let orig = RQVAX.prototype.putr;
        RQVAX.prototype.putr = function(pkt, cmd, flg, sts, lnt, typ) {
            return orig.call(this, pkt, cmd & ~OP.END, flg, sts, lnt, typ);
        };
        return () => { RQVAX.prototype.putr = orig; };
    },
    "the-connection-id-is-not-reset-in-the-response-header": () => {
        let orig = RQVAX.prototype.putr;
        RQVAX.prototype.putr = function(pkt, cmd, flg, sts, lnt, typ) {
            let hctc = this.pd(pkt, UQ_HCTC);
            orig.call(this, pkt, cmd, flg, sts, lnt, typ);
            this.spd(pkt, UQ_HCTC, hctc);
        };
        return () => { RQVAX.prototype.putr = orig; };
    },
    "SET-CONTROLLER-CHARACTERISTICS-does-not-round-the-timeout-up": () => {
        let orig = RQVAX.prototype.scc;
        RQVAX.prototype.scc = function(pkt, q) {
            let r = orig.call(this, pkt, q);
            if (this.htmo >= 2) this.htmo = this.htmo - 2;
            return r;
        };
        return () => { RQVAX.prototype.scc = orig; };
    },
    "SET-CONTROLLER-CHARACTERISTICS-filters-the-host-flags": () => {
        let orig = RQVAX.prototype.scc;
        RQVAX.prototype.scc = function(pkt, q) {
            let cfl = this.pd(pkt, SCC_CFL);
            this.spd(pkt, SCC_CFL, cfl & 0x00F0);           /* CF_MSK, which the C does NOT apply */
            return orig.call(this, pkt, q);
        };
        return () => { RQVAX.prototype.scc = orig; };
    },
    "a-full-response-ring-drops-the-packet": () => {
        let orig = RQVAX.prototype.enqt;
        RQVAX.prototype.enqt = function(obj, key, pkt) {
            if (key === "rspq") return;                     /* silently lose it */
            return orig.call(this, obj, key, pkt);
        };
        return () => { RQVAX.prototype.enqt = orig; };
    },
    "the-queue-thread-does-not-re-arm-after-a-command": () => {
        /* rq_quesvc re-arms whenever its local `pkt` is non-zero, INCLUDING after a command whose
           packet rq_putpkt already freed.  Dropping that costs the second service -- the one that
           finds the ring empty and clears `pip` -- which is visible in PIP, in `show rq rings` and
           in the next command's iteration count. */
        let orig = RQVAX.prototype.quesvc;
        RQVAX.prototype.quesvc = function() {
            let up = this.csta === CST_UP;
            orig.call(this);
            if (up && this.rspq === 0) this.queDue = null;
        };
        return () => { RQVAX.prototype.quesvc = orig; };
    },
    "rq-window-decoded-at-the-wrong-offset": () => ({rqBaseDelta: 4}),
    "no-event-queue-at-all": () => ({noQbusHook: true})
};

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                       *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [];
    let acc = {waitCounts: new Set(), nonZeroWaits: 0, zeroWaits: 0, credValues: new Set(),
               rspOpc: new Set(), rspSts: new Set(), cmdLines: 0};

    /* ---- PHASE S: the scope, re-derived from the C on every run ---- */
    let scope = opts.scope;
    for (let f of scope.failures) failures.push(f);
    report.push(`  PHASE S  ${scope.nOp} OP_ codes, ${scope.nSt} ST_ codes and ${scope.nSwitch} ` +
        `rq_mscp() dispatch case(s) re-derived from ${scope.dir}; rq.js agrees`);

    let cases = enumeratedCases();
    let all = cases.concat(randomCases(opts.nRandom, opts.seed, cases.length));
    let sim = runCasesSimh(simh, opts, all);
    /* Each pass writes its own do-file and starts its OWN simulator, so the oracle's static `MSC`
       struct starts every pass at its C-global zero.  The JS machine is built once and reused
       (standing rule 14), so the pass boundary is where that has to be re-established. */
    machine(mutationOpts).rq.powerUp();
    let js = all.map((c) => runCaseJS(c, mutationOpts));

    assertExclusions(all, sim, failures);
    let compared = grade(all, sim, js, failures);
    assertWaits(all, sim, js, failures, acc);
    coverage(all, sim, js, failures, acc);

    /* The wiring the graded machine is actually holding, asserted rather than assumed. */
    let m = machine(mutationOpts);
    if (!mutationOpts.noQbusHook && m.cpu.qbus !== m.rq) {
        failures.push(`the graded machine's CPU has no Qbus event hook wired to the controller, so ` +
            `rq_quesvc() can never run and every "answer" would be whatever the last write left`);
    }
    if (!m.rq.cqbic || !m.rq.cqbic.bus) {
        failures.push(`the graded machine's controller has no CQBIC with a bus, so no descriptor, ` +
            `packet or interrupt flag can reach memory at all`);
    }

    report.push(`  CASES    ${compared}/${all.length} case(s) compared ` +
        `(${cases.length} enumerated + ${opts.nRandom} randomized)`);
    report.push(`  TRACE    ${acc.cmdLines} command line(s) in the oracle's DEBUG=REQ stream, ` +
        `${acc.rspOpc.size} distinct response opcode(s), ${acc.rspSts.size} distinct status word(s)`);
    report.push(`  FLOORS   ${acc.waitCounts.size} distinct in-band response-wait count(s) ` +
        `(${acc.nonZeroWaits} non-zero, ${acc.zeroWaits} immediate), ` +
        `CRED values {${[...acc.credValues].sort((a, b) => a - b).join(",")}}, ` +
        `wrapped twice: ${acc.wrappedTwice ? "yes" : "NO"}, ` +
        `ring interrupt seen both ways: ${acc.intFired && acc.intSilent ? "yes" : "NO"}, ` +
        `scattered pages: ${acc.scatteredPages ? "yes" : "NO"}`);
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
        } catch (e) {
            /* A mutation that makes rq.js THROW is caught too -- loudly, and by name.  What is not
               acceptable is a mutation that changes nothing. */
            failures = [`threw: ${e.message}`];
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
        console.error(`mscpringdiff: --cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "mscpringdiff-"));
    let code = 0;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized cases: ${nRandom}`);

        let opts = {scratch, nRandom, seed};
        opts.scope = checkScope(simh);

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
            console.log(`\nMATCH -- every graded command posted in the command ring came back as a ` +
                `response in the response ring exactly as the oracle returned it: every descriptor ` +
                `bit and ring index, both interrupt flag words, the credit and free-queue ` +
                `bookkeeping, PC/PSL/R0..R14, every byte of every physical page the rings and ` +
                `packets were scattered across, all ${RQ_OBS.length} examinable controller ` +
                `registers, ${PKT_PROBES.length} words of the packet array, the text of SHOW RQ ` +
                `RINGS/FREEQ/RESPQ, and the ORDERED DEBUG=REQ trace line for line.`);
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
