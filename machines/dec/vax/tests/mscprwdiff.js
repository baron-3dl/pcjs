/**
 * @fileoverview Differential test: MSCP data transfer -- blocks moved between a user-supplied image
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
 *               and host memory through the CQBIC scatter-gather map, graded against a real Open
 *               SIMH microvax3900
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-346, the fourth of pcjsvax-6a5's children and THE ONE THAT ACTUALLY MOVES DATA.  With the
 * controller up (pcjsvax-c2c), the rings working (pcjsvax-0b4) and a user's image attached to a
 * drive (pcjsvax-f52), the guest posts a READ -- and 512 or 131,584 bytes of that file appear in
 * host memory, scattered across whatever physical pages the CQBIC map points at.
 *
 * *** THE PROJECT-WIDE CHEAT THIS FILE EXISTS TO KILL. ***  "The blocks match" is satisfied
 * completely by reading the image file directly whenever the guest asks for a block, implementing
 * NO CONTROLLER AT ALL.  The blocks here ARRIVE AS A CONSEQUENCE OF THE GUEST DRIVING THE
 * REGISTERS: a descriptor posted in the command ring, a poll started by a read of IP, a packet
 * fetched by DMA, rq_svc()'s chunking, a DMA through the scatter-gather map, and an end packet
 * placed in the response ring with a residual computed against fields the host still owns.  Every
 * one of those is graded here and in the sibling differentials, and nothing in this file compares
 * the image to memory directly -- it compares THIS ENGINE'S memory to THE ORACLE'S memory.
 *
 * *** THE INHERITED CHEAT: IDENTITY-MAPPING THE QBUS ADDRESS. ***  pcjsvax-e22 proved by
 * construction that treating a Qbus address as a physical address survives ANY single-page case.
 * It is RE-ASSERTED here rather than inherited: the `map-lookup-bypassed` mutation below is
 * required to SURVIVE a deliberately identity-mapped case and to DIE on a transfer spanning three
 * or more DISCONTIGUOUS, OUT-OF-ORDER map entries -- and --selfcheck reports WHICH case caught it,
 * the way tests/qdmadiff.js does.
 *
 * SIX PHASES
 * ----------
 *   PHASE S  the MSCP opcode / status / dispatch scope, re-derived from pdp11_rq.c by
 *            tests/mscpscope.js on every run, plus the five transfer opcodes re-derived from the
 *            C's own dispatch rather than listed here.
 *   PHASE V  the PREMISES this item was handed, each re-measured rather than assumed: that the
 *            oracle is built NOASYNCH (so the disk callback runs inline and the schedule is
 *            deterministic), that RQ_MAXFR / RQ_MAPXFER / RQ_M_PFN are what rq.js says, and that
 *            rq.js's VA_M_OFF is the same constant cqbic.js translates with.
 *   PHASE C  the transfers themselves, driven through the rings by real VAX instructions, with
 *            every byte of every physical page the transfer COULD have touched compared WHOLE.
 *   PHASE R  the SAME grading against a REAL OpenVMS ODS-2 volume, attached read-only.
 *   PHASE I  the two engines' WRITE and ERASE output compared as FILES: each gets its own copy of
 *            the same container, runs the same commands, and the copies must come back identical.
 *   PHASE M  --selfcheck only: the mutations.
 *
 * WHAT IS DELIBERATELY NOT GRADED, BY NAME (standing rule 6)
 * ----------------------------------------------------------
 *   - CANCELLING AN IN-FLIGHT TRANSFER.  rq_abo()'s and rq_gcs()'s search of `uptr->cpkt` /
 *     `uptr->pktq`.  pcjsvax-346 made the state reachable and did not take it into scope; rq.js
 *     throws by name and assertExclusions() FAILS the run if a case sends ABORT or GET COMMAND
 *     STATUS to a unit holding a packet.
 *   - rq_dte(), the disk-error log.  It hangs off a failing pread(2)/pwrite(2), which no do-file
 *     can arrange on a RAW container -- reading past the end of the file returns ZEROS and SCPE_OK.
 *     There is no oracle for it; rq.js throws by name and assertNoIoErrors() fences it.
 *   - A WRITE WHOSE BUFFER FETCH FAILS COMPLETELY.  rq_svc()'s top end issues no disk write, takes
 *     no callback and leaves the unit unscheduled FOREVER -- the real simulator hangs the command.
 *     assertExclusions() FAILS the run if a case unmaps the FIRST page of a write's buffer.
 *   - CONTROLLER INTERRUPT DELIVERY.  LANDED in pcjsvax-aef and graded by tests/mscpintdiff.js --
 *     a SCOPE boundary here, not a gap; every case still supplies SA_S1H_VEC == 0.  See
 *     assertExclusions() for why letting one in would corrupt this file's own measurement.
 *   - MEDIA REMOVAL and NOAUTOSIZE, as in tests/mscpunitdiff.js.
 *
 *      node machines/dec/vax/tests/mscprwdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch build)
 *        --volume PATH     a REAL OpenVMS ODS-2 container for PHASE R (else $PCJS_VAX_VOLUME, else
 *                           the default below).  Attached READ ONLY on both engines.
 *        --no-volume       run without PHASE R.  The skip is reported BY NAME and the run still
 *                           says so in its final line; without this flag a missing volume FAILS.
 *        --cases N         randomized cases (default RANDOM_CASES_DEFAULT; below the fixed floor
 *                           the run FAILS rather than clamping up)
 *        --seed S          PRNG seed, printed on every run so a failure is reproducible
 *        --selfcheck       prove the differential detects deliberate defects
 */

import crypto from "crypto";
import fs from "fs";
import os from "os";
import path from "path";

import RQVAX, {
    RQUnimplemented,
    SA_S1H_VL, SA_S1H_IE, SA_S1H_VEC, SA_S1H_V_CQ, SA_S1H_V_RQ,
    UQ_DESC_OWN, UQ_DESC_F,
    UQ_HLNT, UQ_HCTC, UQ_HCTC_V_CR, UQ_HCTC_V_TYP, UQ_HCTC_V_CID,
    UQ_TYP_SEQ, UQ_TYP_DAT, UQ_CID_MSCP,
    OP, ST, CMD_OPC, CMD_MOD, CMD_UN, CMD_REFL, CMD_REFH,
    RSP_LNT, SCC_LNT, SCC_CFL, SCC_MSV, SCC_TMO,
    RW_BCL, RW_BCH, RW_BAL, RW_BAH, RW_MAPL, RW_MAPH, RW_LBNL, RW_LBNH,
    RW_WBCL, RW_WBAL, RW_WBLL, RW_WMPL, RW_LNT_D,
    SB_OFL_NV, SB_WPR_SW, SB_WPR_HW, SB_HST_OA, SB_HST_OC, SB_HST_NXM,
    I_BCNT, I_LBN, EF_LOG, CF_THS, CF_RPL,
    HBE_LNT, HBE_BADL, ELP_REFL, FM_BAD, LF_SNR,
    UF_WPS, MD_SWP, ONL_UFL,
    DRV_TAB, RD54_DTYPE, RQDF_RO,
    U_ATT, U_ONL,
    RQ_NUMBY, RQ_NUMDR, RQ_MAXFR, RQ_MAPXFER, RQ_M_PFN, VA_M_OFF, VA_N_OFF, VA_M_VPN, VA_N_VPN, PTE_V,
    RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME, RQ_DEV_NAMES, bufferProvider
} from "../modules/v2/rq.js";
import CQBICVAX, { VA_M_OFF as CQ_VA_M_OFF, QBMAMASK } from "../modules/v2/cqbic.js";
import {
    PAGE, R_CODE, R_RESULT, MAP_MBR, OBS_REGS, MEMSIZE,
    RQ_IP, RQ_SA, CQBIC_BASE, CQMAP_BASE, CQMAP_VLD, MEM_MB,
    R_IS, hex, findSimhBin, runSimh, mulberry32, sampleHeap, peakHeap,
    Asm, machine, RQ_OBS, rqFieldOf, PKT_WORDS, pktWord,
    showCtrl, walkScript, emitAction,
    simhResetLines, jsResetForCase, fileImageProvider
} from "./mscpharness.js";
import { checkScope } from "./mscpscope.js";

/** An absolute bound on the instructions any case may execute.  Generous, and deliberately so: the
    oracle's completion delay for a transfer is WALL-CLOCK influenced (see assertSchedule()), so a
    budget sized to the delay this engine models is a budget that expires intermittently -- and a
    case whose in-band wait expires HALTS MID-TRANSFER, at which point vax_cpu.c's HALT drain stops
    at the first UNIT_IDLE queue entry and leaves the drive holding its packet.  That failure looks
    exactly like a controller defect and it is a timing budget.  A bound that is never reached costs
    nothing: the waits exit as soon as the descriptor changes. */
const MAX_STEPS = 3000000;

/** The host's transfer-phase scratch registers.  R0..R8 belong to the handshake. */
const REGS = {prev: 9, cur: 10, cnt: 11, lim: 12, tmp: 13};

/** *** THERE IS NO WAIT BUDGET IN THIS FILE ANY MORE, AND ITS ABSENCE IS THE POINT. ***  Every
    command a graded case posts is one rq_mscp() MUST answer -- rq_rw() answers even the refusals,
    and the one shape that never answers (a WRITE whose buffer fetch fails entirely) is fenced out
    by assertExclusions().  So the host waits with Asm.awaitUnbounded() and cannot walk on while a
    command is still in flight.  A bounded wait here was measured failing about one run in five:
    the oracle's completion delay is not reproducible, an expired budget let the host HALT
    mid-command, vax_cpu.c's drain stopped at the first UNIT_IDLE queue entry, and the two engines
    were then compared at different points in one command's life -- PBSY, CPKT, the free queue and
    half the packet array all legitimately disagreeing.  Raising the bound made it rarer and could
    not make it impossible; removing it does. */
const NO_WAIT_BUDGET = true;

const RANDOM_CASES_DEFAULT = 10;
const RANDOM_CASES_FLOOR   = 5;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure and NOT scaled by case
    count (HANDOFF.md rules 4 and 14).  ONE machine is built and reused across every case and every
    mutation pass; the dominant terms are its single 16MB RAM allocation and the controller's four
    64KB `rqxb` sector buffers.  The containers are NEVER read into memory -- they are reached
    through an `fs`-backed provider that opens a descriptor and nothing more, which is what makes a
    2.5GB sparse container and a 1.5GB real volume cost no heap at all. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;

/** SIMH's flat 16-bit view of the packet array.  A transfer builds its response IN PLACE over the
    command, so the words a handler does not touch are the evidence the whole 64 bytes round-tripped
    -- and for a transfer that includes the EIGHT WORKING WORDS, which rq_rw_end() must have zeroed
    and which carry the chunk state while it is in flight. */
const PKT_PROBES = (function() {
    let out = [];
    for (let p of [0, 1, 2, 3, 4, 5, 31]) for (let w = 0; w < PKT_WORDS; w++) out.push(p * PKT_WORDS + w);
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
 * PHYSICAL LAYOUT -- THIS FILE'S OWN, deliberately not mscpharness's                            *
 *                                                                                               *
 * mscpharness exports DATA_BASE / DATA_NPAGE / physPageFor(), a SIXTY-FOUR page scatter window   *
 * that tests/mscpringdiff.js and tests/mscpunitdiff.js both compute their cases from.  A         *
 * transfer larger than RQ_MAXFR needs 257 Qbus pages of destination on its own, so this file     *
 * needs a bigger window -- and WIDENING THE SHARED ONE WOULD CHANGE WHERE EVERY CASE IN BOTH     *
 * SIBLINGS LANDS.  Its own window, above everything the harness uses, is the version of that     *
 * change that cannot make another differential's cases move.                                    *
 * ------------------------------------------------------------------------------------------- */

/** Above mscpharness's MAP_HI (0x208000) and DATA_HI (0x308000), and 1024 pages (512KB) wide --
    enough for the three-chunk case's 257 destination pages plus its rings and packets, with room
    for the scatter to be a permutation rather than a near-identity. */
const RW_DATA_BASE  = 0x00400000;
const RW_DATA_NPAGE = 1024;
const RW_DATA_HI    = (RW_DATA_BASE + RW_DATA_NPAGE * PAGE) >>> 0;

/** The MSCP-LEVEL map table -- an ordinary table in host memory, NOT the CQBIC map.  Placed well
    away from RW_DATA so a case that walks off either one lands in unrelated memory rather than in
    the other.  8192 longwords, page aligned. */
const MSCPMAP_BASE  = 0x00500000;
const MSCPMAP_LNT   = 8192 * 4;
const MSCPMAP_HI    = (MSCPMAP_BASE + MSCPMAP_LNT) >>> 0;

/** `rq_map_ba()` indexes `ma + (VA_GETVPN(ba) << 2)` with the VPN taken from the WHOLE buffer
    address, RQ_MAXFER's bit 31 INCLUDED -- so a mapped transfer's first entry sits 0x1000000 bytes
    above `ma`.  A host that wants its table at MSCPMAP_BASE therefore supplies this, and uint32
    wraparound does the rest.  COMPUTED from the constants rather than written down, so it moves
    with them. */
/** The `ma` a command packet carries for a mapped transfer.  *** IT IS THE TABLE'S OWN ADDRESS,
    WITH NO BIAS, AND THAT WAS MEASURED RATHER THAN DERIVED. ***  rq_map_ba() indexes
    `ma + (VA_GETVPN(ba) << 2)` and VA_M_VPN is TWENTY-TWO bits, so RQ_MAPXFER's bit 31 is masked
    OUT of the index and the entry number is simply the Qbus page.  The first version of this file
    assumed a 23-bit VPN and put `ma` 0x1000000 BELOW the table to compensate; the oracle answered
    with a MACHINE CHECK from inside the DMA (MCHK code 0x80 = ReadReg, CBTCR = C0000000, the unit
    left holding its packet, no response ever sent).  PHASE V now derives the mask from vax_defs.h
    so the two readings cannot be confused again. */
const MSCPMAP_MA    = MSCPMAP_BASE;

/** The 128 bytes BELOW the interrupt stack pointer -- where a machine check's exception frame
    lands, and the only place its MCHK CODE and FAULTING VIRTUAL ADDRESS are visible.  Zeroed and
    dumped on both engines in every case: without it a machine check inside a graded case shows up
    only as "the PC differs", which is how one was nearly diagnosed as a transfer defect. */
const IS_WIN_LO = (R_IS - 128) >>> 0;

/**
 * physFor(qpage, spread)
 *
 * The physical page a Qbus page is scattered to.  DESCENDING and STRIDED, so the map entries a case
 * programs are discontiguous AND out of order -- which is the whole point: an identity map, or any
 * monotone one, produces the right bytes for a single page and the wrong ones for three.
 *
 * `qpage * 7` is injective modulo 1024 (7 is odd), so no two Qbus pages of a case can collide.
 */
function physFor(qpage, spread)
{
    /* *** 397, NOT 7. ***  A small odd stride is injective but MONOTONE: consecutive Qbus pages
       land on consecutive-by-7 physical pages, in order, and an identity map or ANY constant-offset
       map produces the same bytes for a multi-page transfer.  coverage() re-derives that property
       from the case's own entries and fails the run if it holds, which is how the first version of
       this function was caught.  397 is odd (so still injective modulo 1024) and large enough that
       consecutive pages jump across the window and back. */
    let i = ((qpage * 397) + spread * 13) % RW_DATA_NPAGE;
    return ((RW_DATA_BASE + (RW_DATA_NPAGE - 1 - i) * PAGE) / PAGE) | 0;
}

/** A non-zero, PAGE-DISTINCT seed.  Two jobs: a transfer that landed on the wrong page shows up as
    the wrong seed rather than as plausible data, and a READ that overran its extent leaves the
    seed's neighbours overwritten in a page the case did not intend. */
function seedFor(ppage) { return ((0x5A5A0000 | ((ppage * 0x0101) & 0xFFFF)) >>> 0); }

/* ------------------------------------------------------------------------------------------- *
 * The scratch containers                                                                        *
 * ------------------------------------------------------------------------------------------- */

/**
 * The containers this differential generates, attaches to both engines and deletes.
 *
 * SIZES CHOSEN FOR WHAT THE TRANSFER PATH DOES TO THEM:
 *   small   1,024 blocks -- fits inside every drive type, so `set rq0 <type>` decides the unit's
 *           capacity and the RCT window past it is reachable
 *   short   100 blocks on an RD54 (311,200) -- autosize's clamp leaves the UNIT far larger than the
 *           CONTAINER, so a legal READ inside the unit runs off the end of the file and must come
 *           back ZERO-FILLED.  This is the same shape the real OpenVMS volume has and it is here so
 *           that shape is graded even when PHASE R is skipped.
 *   huge    *** SPARSE, 2.5 GiB apparent, a few kilobytes on disk. ***  5,242,880 blocks, so the
 *           last block's byte offset is 2,684,354,048 -- PAST 2^31.  `(lbn * 512) | 0` is the
 *           natural JS idiom for that multiplication and it goes NEGATIVE here; the container
 *           exists so the `image-read-offset-computed-in-32-bits` mutation dies without needing the
 *           user's 1.5GB volume to be present.  Created with ftruncate and three pwrites, so it
 *           costs no disk space -- which matters: this project has already filled the root
 *           filesystem once (HANDOFF.md 4).
 *   wr      256 blocks, and the ONLY one any graded case writes to.  Each engine gets its OWN copy
 *           and PHASE I compares the two copies byte for byte.
 */
const IMAGES = [
    {tag: "small", blocks: 1024,    sparse: false, writable: false},
    {tag: "short", blocks: 100,     sparse: false, writable: false},
    {tag: "huge",  blocks: 5242880, sparse: true,  writable: false},
    {tag: "wr",    blocks: 360,     sparse: false, writable: true},
    {tag: "wr2",   blocks: 360,     sparse: false, writable: true}
];

/** The blocks a SPARSE container has real bytes in: the first, the last, and one in between whose
    offset is also past 2^31.  Everything else reads as a hole, i.e. zeros -- on both engines,
    because pread(2) over a hole and pread(2) past EOF both deliver zeros. */
function sparseBlocks(spec) { return [0, 4000000, spec.blocks - 1]; }

/**
 * blockBytes(tag, lbn, dst)
 *
 * The 512 bytes of one block, as a pure function of the container's tag and the block number.
 * DETERMINISTIC and position-dependent: an LCG seeded from both, so a transfer that read the wrong
 * block, or read the right block into the wrong place, or read the right bytes in the wrong ORDER,
 * all show up as different bytes rather than as plausible noise.
 */
function blockBytes(tag, lbn, dst)
{
    let s = (lbn * 2654435761 + tag.charCodeAt(0) * 40503 + tag.length) >>> 0;
    for (let i = 0; i < RQ_NUMBY; i++) {
        s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
        dst[i] = (s >>> 24) & 0xFF;
    }
    return dst;
}

/**
 * makeImages(dir)
 *
 * Writes the containers.  Deliberately NOT zeros and NOT a recognisable structure: sim_disk's
 * get_filesystem_size() probes for ODS-2, RT-11, Ultrix and several other volume headers, and a
 * container it RECOGNISES takes a different autosize path (see tests/mscpunitdiff.js's header).
 * Noise is what a user-supplied file that is not a VMS disk looks like.  PHASE R is where a
 * container the simulator DOES recognise gets graded.
 */
function makeImages(dir)
{
    let out = [];
    let blk = new Uint8Array(RQ_NUMBY);
    for (let spec of IMAGES) {
        let p = path.join(dir, `mscprw-${spec.tag}.dsk`);
        let fd = fs.openSync(p, "w+");
        if (spec.sparse) {
            fs.ftruncateSync(fd, spec.blocks * RQ_NUMBY);
            for (let lbn of sparseBlocks(spec)) {
                fs.writeSync(fd, blockBytes(spec.tag, lbn, blk), 0, RQ_NUMBY, lbn * RQ_NUMBY);
            }
        } else {
            for (let lbn = 0; lbn < spec.blocks; lbn++) {
                fs.writeSync(fd, blockBytes(spec.tag, lbn, blk), 0, RQ_NUMBY, lbn * RQ_NUMBY);
            }
        }
        fs.closeSync(fd);
        /* *** A WRITABLE CONTAINER IS COPIED, SO EACH ENGINE GETS ITS OWN. ***  Attaching the SAME
           file to both would make them overwrite each other's results and every comparison would
           pass by construction.  Two copies, the same commands, and PHASE I requires the two files
           to come back byte-identical -- which is a grading of the WRITE path that no memory
           comparison can do, because the bytes never come back into memory. */
        let simhPath = p, jsPath = p;
        if (spec.writable) {
            simhPath = path.join(dir, `mscprw-${spec.tag}.simh.dsk`);
            jsPath = path.join(dir, `mscprw-${spec.tag}.js.dsk`);
            fs.copyFileSync(p, simhPath);
            fs.copyFileSync(p, jsPath);
        }
        out.push({tag: spec.tag, blocks: spec.blocks, bytes: spec.blocks * RQ_NUMBY,
                  path: jsPath, simhPath, pristine: p,
                  sparse: spec.sparse, writable: spec.writable, sha: spec.writable ? null : shaOf(p)});
        sampleHeap();
    }
    return out;
}

/** sha256 of a whole container, streamed a megabyte at a time so a 2.5GB sparse file costs no
    heap (standing rule 14 -- readFileSync() of that file is a 2.5GB Buffer). */
function shaOf(p)
{
    let h = crypto.createHash("sha256");
    let fd = fs.openSync(p, "r");
    try {
        let buf = Buffer.allocUnsafe(1 << 20), off = 0, n;
        while ((n = fs.readSync(fd, buf, 0, buf.length, off)) > 0) { h.update(buf.subarray(0, n)); off += n; }
    } finally { fs.closeSync(fd); }
    return h.digest("hex");
}

/** The READ-ONLY containers must come back BYTE-IDENTICAL.  sim_disk.c's store_disk_footer() exists
    to append SIMH metadata to a container it autosized -- which would change the file's SIZE and so
    the unit's capacity on the next attach -- and only an unconditional early `return SCPE_OK;` in
    this vendor tree stops it.  That is a property of the vendor tree, so it is CHECKED. */
function checkImagesUntouched(images, failures)
{
    for (let im of images) {
        if (im.sha === null) continue;                      /* the write target: PHASE I grades it */
        let sz = fs.statSync(im.path).size;
        let sha = shaOf(im.path);
        if (sz !== im.bytes || sha !== im.sha) {
            failures.push(`the scratch container "${im.tag}" was MODIFIED by the run (${im.bytes} ` +
                `bytes / ${im.sha.slice(0, 16)} before, ${sz} / ${sha.slice(0, 16)} after).  No ` +
                `graded case writes to it, so either a transfer went to the wrong unit or the ` +
                `oracle appended metadata -- and nothing here is comparable until that is understood.`);
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Geometry -- where everything lives in QBUS space, computed FROM THE SPEC                      *
 * ------------------------------------------------------------------------------------------- */

/**
 * rwGeometry(spec)
 *
 * mscpharness's geometry() plus the two regions a transfer needs, and it is a SECOND writing of the
 * same arithmetic on purpose (the discipline tests/qdmadiff.js applies to its own page list): the
 * host program addresses PHYSICAL memory and the controller addresses QBUS memory through the map,
 * and a disagreement between the two shows up as a memory difference rather than as a silent pass.
 *
 * *** THE RESPONSE RING COMES FIRST ***: rq_step4() does `rq.ba = comm; cq.ba = comm + rq.lnt`.
 *
 * `data` is the buffer the command packet names.  `mirror` exists only for an RQ_MAPXFER case: the
 * MSCP-level map translates the DATA pages to the MIRROR pages, and the CQBIC map then translates
 * the mirror pages to physical.  Two levels, deliberately pointing in opposite directions, so a
 * model that applied one of them twice or the other not at all lands somewhere graded.
 */
function rwGeometry(spec)
{
    let rqLnt = 4 << spec.rqCode, cqLnt = 4 << spec.cqCode;
    let rqBa = spec.comm >>> 0, cqBa = (spec.comm + rqLnt) >>> 0;
    let nCmd = spec.nCmdBuf, nRsp = spec.nRspBuf;
    let pgUp = (a) => (a + PAGE - 1) & ~(PAGE - 1);
    /* THE AREAS ARE ON SEPARATE QBUS PAGES, deliberately: a page is the unit the map validates, so
       "unmap the page the transfer's buffer lives on" is only a statement about the buffer if
       nothing else lives there. */
    let cmdBase = pgUp(spec.comm + rqLnt + cqLnt);
    let rspBase = cmdBase + pgUp(nCmd * 64);
    let dataBase = rspBase + pgUp(nRsp * 64);
    let dataLnt = pgUp(spec.dataLnt || PAGE);
    let mirrorBase = spec.mapped ? (dataBase + dataLnt) : 0;
    let mirrorLnt = spec.mapped ? dataLnt : 0;
    return {
        rqLnt, cqLnt, rqBa, cqBa, cmdBase, rspBase, dataBase, dataLnt, mirrorBase, mirrorLnt,
        rqSlots: rqLnt >> 2, cqSlots: cqLnt >> 2,
        cmdBuf: (i) => (cmdBase + i * 64) >>> 0,
        cmdEnv: (i) => (cmdBase + i * 64 + 4) >>> 0,
        rspBuf: (j) => (rspBase + j * 64) >>> 0,
        rspEnv: (j) => (rspBase + j * 64 + 4) >>> 0,
        /* The interrupt flag words sit BELOW comm: SA_COMM_CI (-4) and SA_COMM_RI (-2). */
        lo: (spec.comm - 8) >>> 0,
        hi: (dataBase + dataLnt + mirrorLnt - 1) >>> 0
    };
}

/** Every Qbus page a case can reference, derived from the geometry rather than listed. */
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
 * The words of an MSCP command packet a case sets, as {index: value}.  EVERY WORD IT DOES NOT SET
 * IS LEFT AS THE PAGE SEED: the controller reads all 64 bytes whatever the command is and writes
 * back `d[UQ_HLNT] - UQ_HDR_OFF` of them, so the seed in the tail is what proves the round trip.
 *
 * *** THE EIGHT WORKING WORDS ARE LEFT AS THE SEED TOO, AND THAT IS THE POINT. ***  rq_rw() writes
 * them from the host's own fields and rq_rw_end() ZEROES them, so a response whose RW_WBAL..RW_WMPH
 * still carry the host's seed is a transfer that never started, and one that carries a live chunk
 * address is a transfer that never ended.
 */
function cmdWords(o)
{
    let w = {};
    w[UQ_HLNT] = (o.hlnt === undefined) ? RSP_LNT : o.hlnt;
    w[UQ_HCTC] = (((o.cr || 0) << UQ_HCTC_V_CR) |
                  (UQ_TYP_SEQ << UQ_HCTC_V_TYP) |
                  (UQ_CID_MSCP << UQ_HCTC_V_CID)) & 0xFFFF;
    w[CMD_REFL] = (o.ref === undefined ? 0xBEEF : o.ref) & 0xFFFF;
    w[CMD_REFH] = (o.refh === undefined ? 0x1234 : o.refh) & 0xFFFF;
    w[CMD_UN] = (o.unit || 0) & 0xFFFF;
    w[CMD_OPC] = (o.opc === undefined ? OP.RD : o.opc) & 0xFFFF;
    w[CMD_MOD] = (o.mod || 0) & 0xFFFF;
    if (o.ufl !== undefined) w[ONL_UFL] = o.ufl & 0xFFFF;
    if (o.cfl !== undefined) { w[SCC_CFL] = o.cfl & 0xFFFF; w[SCC_MSV] = 0; w[SCC_TMO] = 0; }
    /* The transfer parameters.  `ba` is a QBUS address (plus RQ_MAPXFER for a mapped transfer) and
       `map` is a HOST PHYSICAL address -- two different address spaces in adjacent words of one
       packet, which is exactly the confusion rq_map_ba() invites. */
    if (o.bc !== undefined) { w[RW_BCL] = o.bc & 0xFFFF; w[RW_BCH] = (o.bc >>> 16) & 0xFFFF; }
    if (o.ba !== undefined) { w[RW_BAL] = o.ba & 0xFFFF; w[RW_BAH] = (o.ba >>> 16) & 0xFFFF; }
    if (o.map !== undefined) { w[RW_MAPL] = o.map & 0xFFFF; w[RW_MAPH] = (o.map >>> 16) & 0xFFFF; }
    if (o.lbn !== undefined) { w[RW_LBNL] = o.lbn & 0xFFFF; w[RW_LBNH] = (o.lbn >>> 16) & 0xFFFF; }
    return w;
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

const RING_CODE_MAX = 7;
const PLUG_PARK = 900;

/** One drive's configuration.  `image` is an IMAGES tag or null. */
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

function buildCase(spec)
{
    let c = Object.assign({
        itime: RQ_ITIME, i4time: RQ_ITIME4, qtime: RQ_QTIME, xtime: RQ_XTIME,
        cqCode: 1, rqCode: 1, comm: 0x2000, prgi: 0, spread: 0,
        nCmdBuf: 2, nRspBuf: 2, dataLnt: PAGE, mapped: false, identity: false,
        unmappedQ: [], invalidMscpQ: [], packets: {}, steps: [], units: null
    }, spec);

    if (c.cqCode > RING_CODE_MAX || c.rqCode > RING_CODE_MAX) {
        throw new Error(`mscprwdiff: case "${c.name}" ring code out of range`);
    }
    c.units = (c.units || []).slice();
    while (c.units.length < RQ_NUMDR) c.units.push(unitSpec({}));
    for (let i = 0; i < RQ_NUMDR; i++) if (c.units[i].plug === undefined) c.units[i].plug = i;
    let seen = new Set();
    for (let u of c.units) {
        if (seen.has(u.plug)) throw new Error(`mscprwdiff: case "${c.name}" reuses plug ${u.plug}`);
        seen.add(u.plug);
        if (u.plug >= PLUG_PARK) throw new Error(`mscprwdiff: plug ${u.plug} collides with the park range`);
        if (u.plug >= RQ_NUMDR && u.plug < 254) {
            throw new Error(`mscprwdiff: case "${c.name}" wants plug ${u.plug}, which one of the ` +
                `DISABLED potential drives already holds -- the simulator would refuse the \`set\``);
        }
    }

    c.s1dat = (SA_S1H_VL | (c.cqCode << SA_S1H_V_CQ) | (c.rqCode << SA_S1H_V_RQ)) & 0xFFFF;
    c.g = rwGeometry(c);

    let qpages = qbusPagesFor(c.g);
    let unmapped = new Set(c.unmappedQ);
    /* `unmappedData` names pages RELATIVE TO THE BUFFER, because a case cannot know the absolute
       Qbus page until the geometry exists and the geometry needs the ring codes -- see build(). */
    for (let k of (c.unmappedData || [])) unmapped.add(((c.g.dataBase / PAGE) | 0) + k);
    /* THE SCATTER.  `identity` maps every Qbus page to the physical page of the same number -- the
       one arrangement in which the map is indistinguishable from no map at all, and therefore the
       one a case must have if the `map-lookup-bypassed` mutation is to be shown SURVIVING it. */
    c.entries = qpages.filter((q) => !unmapped.has(q))
                      .map((q) => ({q, p: c.identity ? q : physFor(q, c.spread)}));
    c.qToP = new Map(c.entries.map((e) => [e.q, e.p]));
    c.dumpPages = [...new Set(c.entries.map((e) => e.p))].sort((a, b) => a - b);
    c.resultPage = (R_RESULT / PAGE) | 0;

    c.phys = (qaddr) => {
        let q = (qaddr / PAGE) | 0;
        if (!c.qToP.has(q)) {
            throw new Error(`mscprwdiff: case "${c.name}" addresses Qbus 0x${hex(qaddr, 6)}, whose ` +
                `page ${q} is deliberately UNMAPPED`);
        }
        return (c.qToP.get(q) * PAGE + (qaddr % PAGE)) >>> 0;
    };

    /* THE MSCP-LEVEL MAP, for an RQ_MAPXFER case only.  Entry `k` of the DATA region translates to
       the mirror region's page `n-1-k`, so the second level is itself out of order and a model
       that skipped it delivers the bytes in the wrong order rather than not at all. */
    c.mscpEntries = [];
    if (c.mapped) {
        let n = c.g.dataLnt / PAGE;
        for (let k = 0; k < n; k++) {
            let dq = ((c.g.dataBase / PAGE) | 0) + k;
            let mq = ((c.g.mirrorBase / PAGE) | 0) + (n - 1 - k);
            if (c.invalidMscpQ.indexOf(k) >= 0) c.mscpEntries.push({idx: dq, val: 0});
            else c.mscpEntries.push({idx: dq, val: (PTE_V | (mq & RQ_M_PFN)) >>> 0});
        }
    }

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
    /* `band` marks the ITERATION COUNT (slot 0) as not-exactly-comparable; the VALUE SEEN (slot 1)
       is the descriptor the controller wrote and is always compared exactly. */
    let take = (n, what, band) => {
        let o = off;
        for (let k = 0; k < n; k++) c.slots.push({off: o + k * 4, what, band: !!(band && k === 0)});
        off += n * 4;
        return o;
    };
    for (let st of c.steps) {
        if (st.s === "await") {
            /* *** IS THIS AWAIT WAITING ON A UNIT SERVICE? ***  Every case in this file posts
               command `k` into response slot `k`, so the slot names the command; if that command is
               one of the five rq_rw() opcodes the wait covers a DISK operation and its ITERATION
               COUNT is not reproducible on the oracle -- see assertSchedule(). */
            let k = (st.cmd === undefined) ? st.slot : st.cmd;
            let cs = (c.cmdSpecs || [])[k];
            let nm = cs ? RQVAX.OP_NAME_OF[cs.opc & 0xFF] : undefined;
            st.xfer = !!(nm && RQVAX.MSCP_XFER_OPS.indexOf(nm) >= 0);
            st.roff = take(2, `${st.what || "await"} (iterations / value)`, st.xfer);
        } else if (st.s === "snap") st.roff = take(1, st.what || st.s);
    }
    if (off > PAGE) throw new Error(`mscprwdiff: case "${c.name}" needs ${off} RESULT bytes`);

    let a = new Asm();
    a.movImmAbs(4, MAP_MBR, (CQBIC_BASE + 4 * 4) >>> 0);            // REG_MBR == 4
    for (let e of c.entries) a.movImmAbs(4, (CQMAP_VLD | e.p) >>> 0, (CQMAP_BASE + e.q * 4) >>> 0);
    /* The MSCP map is ORDINARY MEMORY, so the host writes it with a plain MOVL to a PHYSICAL
       address -- not through CQMAP_BASE, which is the CQBIC's register window.  Getting these two
       the same way round is the whole distinction rq_map_ba() rests on. */
    for (let e of c.mscpEntries) a.movImmAbs(4, e.val, (MSCPMAP_BASE + e.idx * 4) >>> 0);
    for (let act of walkScript(c.s1dat, c.comm, c.prgi, {})) {
        if (!emitAction(a, act)) throw new Error(`mscprwdiff: unknown handshake action "${act.a}"`);
    }
    /* The code OFFSET each step begins at.  A diagnostic, and it earned its place: a machine check
       inside a graded case reports a PC, and without this the only way back from that PC to the
       host action that caused it is counting instruction bytes by hand. */
    for (let st of c.steps) { st.pc = (R_CODE + a.len) >>> 0; emitStep(a, st, c); }
    a.halt();
    c.code = a.b;
    c.haltPC = (R_CODE + c.code.length) >>> 0;
    if (c.code.length > 0x4000) throw new Error(`mscprwdiff: case "${c.name}" code is ${c.code.length} bytes`);
    return c;
}

/**
 * emitStep(a, st, c)
 *
 * Descriptors are written by the HOST, to PHYSICAL addresses, with real instructions -- the
 * controller reaches the same words through the CQBIC scatter-gather map.
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
    case "snap":
        a.movAbsReg(4, c.phys(st.q >>> 0), REGS.prev);
        return a.movRegAbs(REGS.prev, (R_RESULT + st.roff) >>> 0);
    case "delay":
        return a.delay(st.n, REGS.tmp);
    }
    throw new Error(`mscprwdiff: unknown step "${st.s}"`);
}

/**
 * command(seq, o)
 *
 * ONE complete host transaction: clear the interrupt flag words, grant a response descriptor, post
 * the command descriptor, read IP, wait IN BAND for the response descriptor to change, then settle.
 *
 * *** THE IN-BAND WAIT IS THE INSTRUMENT THAT KILLS THE SYNCHRONOUS-TRANSFER CHEAT. ***  A real
 * transfer costs rq_qtime for the poll plus rq_xtime for the disk, and the host's polling loop
 * counts the difference; a controller that moved the blocks inside the IP read would send every
 * iteration count to zero.  It ALSO tells a one-chunk transfer from a three-chunk one, because
 * only the FIRST chunk waits (rq_io_complete() re-schedules at `iostarttime + xtime`, which is
 * already past from chunk two onward).
 */
function command(seq, o)
{
    let tag = o.tag || `cmd${o.pkt}`;
    seq.push({s: "clrint"});
    seq.push({s: "rdesc", slot: o.rslot, buf: o.rbuf === undefined ? o.pkt : o.rbuf});
    seq.push({s: "cdesc", slot: o.cslot, pkt: o.pkt});
    seq.push({s: "ip"});
    seq.push({s: "await", slot: o.rslot, what: `${tag} response descriptor`});
    seq.push({s: "delay", n: o.settle === undefined ? 400 : o.settle});
    return seq;
}

/**
 * transaction(o)
 *
 * A whole case in one call: N commands, each into its own command buffer and its own response
 * buffer, over a ring pair big enough for them.
 */
function transaction(o)
{
    let n = o.cmds.length;
    let code = 0;
    while ((1 << code) < n) code++;
    if (code > 4) throw new Error(`mscprwdiff: ${n} commands needs a ring code of ${code}`);
    let packets = {}, seq = [];
    for (let i = 0; i < n; i++) packets[i] = cmdWords(o.cmds[i]);
    for (let i = 0; i < n; i++) {
        command(seq, {pkt: i, cslot: i, rslot: i, tag: o.tags ? o.tags[i] : `#${i}`});
    }
    return Object.assign({
        cqCode: code, rqCode: code, nCmdBuf: n, nRspBuf: n, packets, steps: seq
    }, o.spec || {});
}

/**
 * build(o)
 *
 * A case, built in the one order that works: the ring codes come from the COMMAND COUNT, the
 * geometry comes from the ring codes, and only then can a command name a buffer address -- because
 * `ba` is a Qbus address INSIDE that geometry.  A helper rather than four lines per case because
 * getting the order wrong produces a case whose buffer overlaps its own response packets, which
 * looks exactly like a controller that wrote the wrong place.
 */
function build(o)
{
    let n = o.ncmds;
    let code = 0;
    while ((1 << code) < n) code++;
    if (code > RING_CODE_MAX) throw new Error(`mscprwdiff: ${n} commands needs ring code ${code}`);
    let spec = Object.assign({
        cqCode: code, rqCode: code, comm: 0x2000, nCmdBuf: n, nRspBuf: n,
        dataLnt: PAGE, mapped: false, identity: false, spread: 0,
        unmappedQ: [], invalidMscpQ: []
    }, o.spec || {});
    let g = rwGeometry(spec);
    let cmds = o.cmds(g);
    if (cmds.length !== n) {
        throw new Error(`mscprwdiff: case "${o.name}" declared ${n} commands and produced ${cmds.length}`);
    }
    let packets = {}, seq = [];
    for (let i = 0; i < n; i++) packets[i] = cmdWords(cmds[i]);
    if (o.steps) seq = o.steps(g, n);
    else for (let i = 0; i < n; i++) {
        command(seq, {pkt: i, cslot: i, rslot: i, tag: (o.tags || [])[i] || `#${i}`});
    }
    return buildCase(Object.assign({}, spec, {
        packets, steps: seq, name: o.name, kind: o.kind, units: o.units, cmdSpecs: cmds
    }));
}

/* The drive types the containers are attached to, looked up BY NAME so a table that grew an entry
   does not silently re-point them.  RX18 is the SMALLEST drive in drv_tab[] (360 blocks) and it is
   the write target's type for a measured reason: autosize's clamp raises a WRITABLE unit's capacity
   to the DRIVE's size whenever the container is smaller, so the only way to have a writable unit
   that is exactly its container is to make the container exactly the smallest drive. */
const RX18_DTYPE = DRV_TAB.findIndex((d) => d.name === "RX18");
const RA92_DTYPE = DRV_TAB.findIndex((d) => d.name === "RA92");

/** The capacities the containers produce, DERIVED from sim_disk's autosize arithmetic rather than
    written down -- and cross-checked against the oracle's own CAPAC register by grade(). */
function capacFor(image, dtype, ro)
{
    let container = image.bytes, current = DRV_TAB[dtype].lbn * RQ_NUMBY;
    if (container < current && !ro) container = current;
    return Math.floor(container / RQ_NUMBY);
}

const IMG = {};                                             /* filled by main(), tag -> descriptor */

function enumeratedCases()
{
    let cases = [];
    let add = (c) => { c.idx = cases.length; cases.push(c); return c; };
    let U = unitSpec;
    /* Unit 0 carries the container under test; unit 1 is an UNATTACHED RD54 and unit 2 is attached
       but never brought ONLINE, so the two "which unit" refusals are reachable in any case. */
    let smallRO = () => [U({dtype: RD54_DTYPE, image: "small", ro: true}),
                         U({dtype: RD54_DTYPE}),
                         U({dtype: RD54_DTYPE, image: "small", ro: true})];
    let CAP = () => capacFor(IMG.small, RD54_DTYPE, true);  /* 1024 */
    let ONL = (ref) => ({opc: OP.ONL, unit: 0, ref});

    /* ---- 1. THE SINGLE-PAGE IDENTITY CASE.  This one exists to be SURVIVED: with every Qbus page
       mapped to the physical page of the same number, a controller that ignored the map entirely
       produces byte-identical results.  --selfcheck requires the `map-lookup-bypassed` mutation to
       pass HERE and to die on case "three discontiguous out-of-order map entries" below. ---- */
    add(build({
        name: "one block, one Qbus page, IDENTITY-MAPPED (the case the map cheat survives)",
        kind: "identity", ncmds: 2, units: smallRO(),
        spec: {identity: true, dataLnt: PAGE},
        cmds: (g) => [ONL(0x1000), {opc: OP.RD, unit: 0, ref: 0x1001, bc: 512, ba: g.dataBase, lbn: 7}],
        tags: ["ONL", "RD 1 block, identity map"]
    }));

    /* ---- 2. THE SAME TRANSFER, SCATTERED.  Identical command, identical bytes, a map that is not
       the identity.  The pair is the measurement: if both pass, the map is being consulted. ---- */
    add(build({
        name: "one block, one Qbus page, scattered",
        kind: "single", ncmds: 2, units: smallRO(), spec: {spread: 3},
        cmds: (g) => [ONL(0x1100), {opc: OP.RD, unit: 0, ref: 0x1101, bc: 512, ba: g.dataBase, lbn: 7}],
        tags: ["ONL", "RD 1 block, scattered"]
    }));

    /* ---- 3. A READ CROSSING ONE MAP-ENTRY BOUNDARY.  512 bytes starting 256 into a page: half in
       one physical page and half in another, wholly unrelated one. ---- */
    add(build({
        name: "a read crossing a map-entry boundary",
        kind: "cross", ncmds: 2, units: smallRO(), spec: {spread: 5, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0x1200),
                      {opc: OP.RD, unit: 0, ref: 0x1201, bc: 512, ba: g.dataBase + 256, lbn: 11}],
        tags: ["ONL", "RD across a page boundary"]
    }));

    /* ---- 4. *** THE FLOOR THAT KILLS THE IDENTITY-MAP CHEAT. ***  2,048 bytes starting 256 into a
       page spans FIVE Qbus pages, and physFor() sends them to five physical pages that are neither
       contiguous nor in order.  coverage() re-derives that from the case's own entries and FAILS
       the run if the scatter it produced is monotone or adjacent -- so this floor cannot be
       satisfied by a map that happens to be tidy. ---- */
    add(build({
        name: "three discontiguous out-of-order map entries",
        kind: "discontig", ncmds: 2, units: smallRO(), spec: {spread: 7, dataLnt: 5 * PAGE},
        cmds: (g) => [ONL(0x1300),
                      {opc: OP.RD, unit: 0, ref: 0x1301, bc: 2048, ba: g.dataBase + 256, lbn: 13}],
        tags: ["ONL", "RD across five scattered pages"]
    }));

    /* ---- 5. LARGER THAN RQ_MAXFR: THREE CHUNKS.  131,584 bytes is 65,536 + 65,536 + 512, so
       rq_svc() runs six times, issues three disk reads at LBN 0 / 128 / 256, rewrites RW_WBAL,
       RW_WBCL and RW_WBLL twice, and sends exactly ONE end packet.  The DEBUG=REQ trace carries all
       three `sim_disk_rdsect lbn: ... len: ...` lines in order and they are compared as text. ---- */
    add(build({
        name: "a transfer larger than RQ_MAXFR -- THREE chunks",
        kind: "chunk", ncmds: 2, units: smallRO(),
        spec: {spread: 11, dataLnt: (2 * RQ_MAXFR + 512)},
        cmds: (g) => [ONL(0x1400),
                      {opc: OP.RD, unit: 0, ref: 0x1401, bc: 2 * RQ_MAXFR + 512, ba: g.dataBase, lbn: 0}],
        tags: ["ONL", "RD 131584 bytes = 3 chunks"]
    }));

    /* ---- 6. A BYTE COUNT THAT IS NOT A MULTIPLE OF 512.  The controller reads ceil(700/512) == 2
       BLOCKS and stores exactly 700 BYTES.  *** THE 324-BYTE TAIL OF THE SECOND BLOCK MUST NOT
       REACH MEMORY *** -- it is real disk data sitting in rqxb, and the whole-page dump is what
       proves the destination page still carries its seed there.  The trace's `len:` field says 700
       while the read that produced it covered 1024. ---- */
    add(build({
        name: "a byte count that is not a multiple of 512 -- the last block's tail must not arrive",
        kind: "partial", ncmds: 3, units: smallRO(), spec: {spread: 13, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0x1500),
                      {opc: OP.RD, unit: 0, ref: 0x1501, bc: 700, ba: g.dataBase, lbn: 3},
                      /* and one that is not a multiple of 512 AND starts mid-page, so the short
                         tail and the page split interact */
                      {opc: OP.RD, unit: 0, ref: 0x1502, bc: 514, ba: g.dataBase + PAGE + 300, lbn: 9}],
        tags: ["ONL", "RD 700 bytes", "RD 514 bytes off-page"]
    }));

    /* ---- 7. THE ENDS OF THE DISK.  LBN 0, LBN maxlbn-1, LBN maxlbn (the RCT window -- LEGAL for
       exactly one block and answered from a ZEROED buffer without touching the file), a two-block
       read in the RCT window (ST_CMD|I_BCNT) and an LBN past the RCT (ST_CMD|I_LBN). ---- */
    add(build({
        name: "LBN 0, maxlbn-1, the RCT window and past it",
        kind: "ends", ncmds: 6, units: smallRO(), spec: {spread: 17, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0x1600),
            {opc: OP.RD, unit: 0, ref: 0x1601, bc: 512, ba: g.dataBase, lbn: 0},
            {opc: OP.RD, unit: 0, ref: 0x1602, bc: 512, ba: g.dataBase + 512, lbn: CAP() - 1},
            {opc: OP.RD, unit: 0, ref: 0x1603, bc: 512, ba: g.dataBase + 1024, lbn: CAP()},
            {opc: OP.RD, unit: 0, ref: 0x1604, bc: 1024, ba: g.dataBase + 1536, lbn: CAP()},
            {opc: OP.RD, unit: 0, ref: 0x1605, bc: 512, ba: g.dataBase + 512,
             lbn: CAP() + DRV_TAB[RD54_DTYPE].rcts}],
        tags: ["ONL", "RD LBN 0", "RD LBN maxlbn-1", "RD the RCT window", "RD 2 blocks of RCT",
               "RD past the RCT"]
    }));

    /* ---- 8. rq_rw_valid()'s LADDER, IN ORDER.  Each rung is a different refusal and the order is
       the answer: a command that is both odd and off the end reports the ODD one.  *** OP_ACC IS
       EXEMPT FROM THE ODD-ADDRESS TEST *** because it never touches host memory, so the same odd
       address that is ST_HST|SB_HST_OA for a READ is a legal ACCESS. ---- */
    add(build({
        name: "rq_rw_valid's ladder: odd address, odd count, huge count, spiral, wrong unit",
        kind: "ladder", ncmds: 8, units: smallRO(), spec: {spread: 19, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0x1700),
            {opc: OP.RD,  unit: 0, ref: 0x1701, bc: 512, ba: g.dataBase + 1, lbn: 4},
            {opc: OP.ACC, unit: 0, ref: 0x1702, bc: 512, ba: g.dataBase + 1, lbn: 4},
            {opc: OP.RD,  unit: 0, ref: 0x1703, bc: 511, ba: g.dataBase, lbn: 4},
            {opc: OP.RD,  unit: 0, ref: 0x1704, bc: 0xF0000000, ba: g.dataBase, lbn: 4},
            {opc: OP.RD,  unit: 0, ref: 0x1705, bc: 1024, ba: g.dataBase, lbn: CAP() - 1},
            {opc: OP.RD,  unit: 1, ref: 0x1706, bc: 512, ba: g.dataBase, lbn: 0},
            {opc: OP.RD,  unit: 9, ref: 0x1707, bc: 512, ba: g.dataBase, lbn: 0}],
        tags: ["ONL", "RD odd address", "ACC odd address (legal)", "RD odd byte count",
               "RD bc & 0xF0000000", "RD spiralling off the end", "RD an unattached unit",
               "RD no such plug"]
    }));

    /* ---- 9. THE UNIT THAT IS THERE AND NOT ONLINE.  rq_rw_valid()'s second rung answers ST_AVL,
       and it is a different answer from ST_OFL|SB_OFL_NV three lines above it.  Unit 2 is attached
       and deliberately never brought online. ---- */
    add(build({
        name: "a transfer to an attached unit that is not ONLINE -- ST_AVL",
        kind: "notonline", ncmds: 3, units: smallRO(), spec: {spread: 23},
        cmds: (g) => [ONL(0x1800),
            {opc: OP.RD, unit: 2, ref: 0x1801, bc: 512, ba: g.dataBase, lbn: 0},
            {opc: OP.RD, unit: 0, ref: 0x1802, bc: 512, ba: g.dataBase, lbn: 0}],
        tags: ["ONL unit 0", "RD unit 2 (attached, offline)", "RD unit 0"]
    }));

    /* ---- 10. THE OTHER FOUR OPCODES ON THE SAME rq_svc() PATH.  ACCESS reads the blocks and
       discards them.  COMPARE reads them and walks host memory a BYTE AT A TIME through rq_readb()
       -- so a COMPARE that MATCHES is independent evidence that the READ before it delivered
       exactly the right bytes, checked by the ORACLE rather than by this file.  A COMPARE against a
       DIFFERENT block mismatches and answers ST_CMP with a residual of `bc - i`, which names the
       first differing byte. ---- */
    add(build({
        name: "ACCESS, COMPARE that matches, COMPARE that mismatches",
        kind: "cmp", ncmds: 5, units: smallRO(), spec: {spread: 29, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0x1900),
            {opc: OP.ACC, unit: 0, ref: 0x1901, bc: 1024, ba: g.dataBase, lbn: 40},
            {opc: OP.RD,  unit: 0, ref: 0x1902, bc: 1024, ba: g.dataBase, lbn: 40},
            {opc: OP.CMP, unit: 0, ref: 0x1903, bc: 1024, ba: g.dataBase, lbn: 40},
            {opc: OP.CMP, unit: 0, ref: 0x1904, bc: 1024, ba: g.dataBase, lbn: 41}],
        tags: ["ONL", "ACC (a read whose data is discarded)", "RD 2 blocks",
               "CMP the same 2 blocks -> ST_SUC", "CMP a different block -> ST_CMP"]
    }));

    /* ---- 11. A HOST BUS ERROR IN THE MIDDLE OF THE DMA.  The buffer's SECOND page is left out of
       the CQBIC map, so Map_WriteW stops there and returns a residual.  rq_svc() writes
       `bc - (tbc - t)` into RW_WBCL and `ba + (tbc - t)` into RW_WBAL, asks rq_hbe() to post an
       error log, and ends the command with EF_LOG and ST_HST|SB_HST_NXM.  The FIRST page must have
       been written and the pages after it must still carry their seed, which the whole-page dump is
       what proves.  The controller's DSER/MEAR also move, and they are compared.
       *** THE FIRST PAGE IS DELIBERATELY MAPPED. ***  A WRITE whose fetch fails ENTIRELY issues no
       disk I/O, takes no callback and leaves the unit unscheduled forever -- see the file header --
       so assertExclusions() refuses any case that unmaps a write's first page. ---- */
    add(build({
        name: "an NXM part way through the DMA -- residual, EF_LOG, ST_HST|SB_HST_NXM",
        kind: "nxm", ncmds: 3, units: smallRO(),
        spec: {spread: 31, dataLnt: 4 * PAGE, unmappedData: [1]},
        cmds: (g) => [ONL(0x1A00),
            {opc: OP.RD, unit: 0, ref: 0x1A01, bc: 2048, ba: g.dataBase, lbn: 17},
            /* *** A COMPARE THAT HITS THE SAME UNMAPPED PAGE, because its nxm path is a DIFFERENT
               ONE. ***  rq_svc()'s COMPARE arm walks host memory a BYTE at a time through
               rq_readb() and, when one fails, writes `bc - i` into BOTH RW_WBCL and RW_WBAL -- a
               byte count into the working BUS ADDRESS, where every other nxm path in the function
               puts an address.  It is almost certainly a vendor typo and it is reproduced; without
               this command nothing in the case list ever reaches it. */
            {opc: OP.CMP, unit: 0, ref: 0x1A02, bc: 2048, ba: g.dataBase, lbn: 17}],
        tags: ["ONL", "RD into a buffer whose second page is unmapped",
               "CMP into the same buffer -- the other nxm path"]
    }));

    /* ---- 12. THE SAME NXM WITH ERROR LOGGING TURNED ON.  rq_hbe()'s "logging disabled" arm
       returns OK, so the END packet is IDENTICAL either way and the only difference a host can see
       is a SECOND, DATAGRAM packet on the response ring carrying FM_BAD, LF_SNR and the faulting
       address in HBE_BADL/HBE_BADH.  SET CONTROLLER CHARACTERISTICS turns it on -- and rq_scc()
       stores the host's whole word under CF_RPL with no CF_MSK filter, which is why CF_THS alone is
       enough.  THREE response buffers for TWO commands, because the log packet needs one. ---- */
    add(build({
        name: "the same NXM with CF_THS set -- an HBE error-log datagram as well as the end packet",
        kind: "hbe", ncmds: 5, units: smallRO(),
        spec: {spread: 37, dataLnt: 4 * PAGE, unmappedData: [1], cqCode: 3, rqCode: 3, nRspBuf: 8},
        cmds: (g) => [
            {opc: OP.SCC, unit: 0, ref: 0x1B00, cfl: CF_THS},
            ONL(0x1B01),
            {opc: OP.RD, unit: 0, ref: 0x1B02, bc: 2048, ba: g.dataBase, lbn: 17},
            /* *** THE COMPARE'S nxm PATH, AND IT IS HERE RATHER THAN IN THE CASE ABOVE BECAUSE
               ONLY CF_THS MAKES ITS ANSWER VISIBLE. ***  rq_svc() writes `bc - i` into the working
               BUS ADDRESS there -- a byte count in an address field -- and the only way a host
               ever sees that word is rq_hbe()'s log packet, which copies it to HBE_BADL/HBE_BADH.
               With logging off the word is zeroed by rq_rw_end() before anyone can read it, which
               is why a mutation for the typo SURVIVED until this command existed. */
            {opc: OP.CMP, unit: 0, ref: 0x1B03, bc: 2048, ba: g.dataBase, lbn: 17},
            {opc: OP.GUS, unit: 0, ref: 0x1B04}],
        /* *** THE ERROR LOG NEEDS A RESPONSE DESCRIPTOR OF ITS OWN, AND A HOST THAT DOES NOT GRANT
           ONE HANGS THE SIMULATOR IN ITS HALT INSTRUCTION. ***  rq_hbe() builds a SECOND packet for
           the same command; with no free descriptor it goes on the response queue and the queue
           thread re-arms itself forever, which vax_cpu.c's HALT drain runs into (pcjsvax-0b4
           measured that, and rq.js's drainOnHalt() names it instead of hanging in sympathy).  So
           the transfer's step grants TWO descriptors before posting its command. */
        steps: (g) => {
            let seq = [];
            command(seq, {pkt: 0, cslot: 0, rslot: 0, tag: "SCC CF_THS"});
            command(seq, {pkt: 1, cslot: 1, rslot: 1, tag: "ONL"});
            seq.push({s: "clrint"});
            seq.push({s: "rdesc", slot: 2, buf: 2});
            seq.push({s: "rdesc", slot: 3, buf: 4});     /* for rq_hbe()'s log packet */
            seq.push({s: "cdesc", slot: 2, pkt: 2});
            seq.push({s: "ip"});
            seq.push({s: "await", slot: 2, cmd: 2, what: "RD with an unmapped second page"});
            /* *** A SECOND awaitL() HERE WOULD ALWAYS TIME OUT, AND IT DID. ***  rq_hbe() runs
               BEFORE rq_rw_end(), so the log datagram takes response slot 2 and the END packet
               takes slot 3 -- both are already released by the time the first wait returns, and a
               wait that reads its "previous" value AFTER the change can never see one.  The second
               descriptor is SNAPSHOTTED instead, and the snapshot is compared exactly. */
            seq.push({s: "delay", n: 400});
            seq.push({s: "snap", q: (g.rqBa + 3 * 4) >>> 0, what: "the RD's HBE second descriptor"});
            seq.push({s: "clrint"});
            seq.push({s: "rdesc", slot: 4, buf: 4});
            seq.push({s: "rdesc", slot: 5, buf: 5});     /* the COMPARE's log packet */
            seq.push({s: "cdesc", slot: 3, pkt: 3});
            seq.push({s: "ip"});
            seq.push({s: "await", slot: 4, cmd: 3, what: "CMP into the same unmapped buffer"});
            seq.push({s: "delay", n: 400});
            seq.push({s: "snap", q: (g.rqBa + 5 * 4) >>> 0, what: "the CMP's HBE second descriptor"});
            command(seq, {pkt: 4, cslot: 4, rslot: 6, tag: "GUS (drains the ring)"});
            return seq;
        },
        tags: ["SCC CF_THS", "ONL", "RD with an unmapped second page",
               "CMP with an unmapped second page", "GUS (drains the ring)"]
    }));

    /* ---- 13. THE RQ_MAPXFER PATH.  Bit 31 of the buffer address selects the MSCP-level map, whose
       entries this case programs at MSCPMAP_BASE with plain MOVLs -- ORDINARY MEMORY, not the
       CQBIC's register window -- and whose `ma` is MSCPMAP_BASE minus 0x1000000 because rq_map_ba()
       takes the VPN from the whole address, bit 31 included.  The MSCP map sends the four data
       pages to the four MIRROR pages IN REVERSE, and the CQBIC map then scatters those.  Whether
       VMS ever sets this bit on a MicroVAX 3900 is unknown (pcjsvax-3f6 will answer it); the branch
       is live in the shipped C and is graded from synthetic packets. ---- */
    add(build({
        name: "RQ_MAPXFER -- the MSCP-level map on top of the CQBIC map",
        kind: "mapxfer", ncmds: 2, units: smallRO(),
        /* *** THE BUFFER STARTS 0x180 INTO A PAGE, AND THAT IS THE ONLY REASON rq_readw()'s
           `0x200 - (ba & VA_M_OFF)` SPLIT IS GRADED AT ALL. ***  With a page-aligned buffer every
           sub-transfer is exactly 0x200 bytes and 0xFF, 0x1FF or 0x3FF as the mask all give the
           same answer; starting mid-page makes the FIRST sub-transfer short and every later one a
           whole page, which only the right mask produces. */
        spec: {spread: 41, dataLnt: 5 * PAGE, mapped: true},
        cmds: (g) => [ONL(0x1C00),
            {opc: OP.RD, unit: 0, ref: 0x1C01, bc: 4 * PAGE,
             ba: (RQ_MAPXFER | (g.dataBase + 0x180)) >>> 0, map: MSCPMAP_MA, lbn: 21}],
        tags: ["ONL", "RD through the MSCP map, starting mid-page"]
    }));

    /* ---- 14. AN INVALID MSCP MAP ENTRY.  rq_map_ba() returns ZERO for an entry without PTE_V, and
       rq_writew() reads that as failure and returns the residual -- so a mapped transfer whose
       third page has no entry stops there, exactly as an unmapped CQBIC page would, but through a
       COMPLETELY DIFFERENT mechanism.  Grading both is what stops the two maps being conflated. ---- */
    add(build({
        name: "RQ_MAPXFER with an INVALID MSCP map entry -- a residual from the other map",
        kind: "mapinv", ncmds: 2, units: smallRO(),
        spec: {spread: 43, dataLnt: 4 * PAGE, mapped: true, invalidMscpQ: [2]},
        cmds: (g) => [ONL(0x1D00),
            {opc: OP.RD, unit: 0, ref: 0x1D01, bc: 4 * PAGE, ba: (RQ_MAPXFER | g.dataBase) >>> 0,
             map: MSCPMAP_MA, lbn: 25}],
        tags: ["ONL", "RD through an MSCP map with one entry invalid"]
    }));

    /* ---- 15. THE UNIT QUEUE.  Both command descriptors are posted BEFORE the single IP read, so
       the queue thread's first service starts transfer #1 and its second finds the unit BUSY and
       defers command #2 onto `uptr->pktq`.  rq_rw_end() then re-arms the queue thread, whose
       unit-queue scan drains it.  *** ONLY rq_rw() EVER SETS cpkt, SO THIS WHOLE PATH WAS
       UNREACHABLE BEFORE THIS ITEM *** -- rq_quesvc()'s unit-queue arm and rq_rw()'s own
       `if (q && uptr->cpkt)` deferral are both live for the first time here. ---- */
    add(build({
        name: "two transfers to one drive -- the second is DEFERRED onto the unit queue",
        kind: "unitq", ncmds: 3, units: smallRO(), spec: {spread: 47, dataLnt: 2 * PAGE},
        cmds: (g) => [ONL(0x1E00),
            {opc: OP.RD, unit: 0, ref: 0x1E01, bc: 512, ba: g.dataBase, lbn: 30},
            {opc: OP.RD, unit: 0, ref: 0x1E02, bc: 512, ba: g.dataBase + PAGE, lbn: 31}],
        steps: (g, n) => {
            let seq = [];
            command(seq, {pkt: 0, cslot: 0, rslot: 0, tag: "ONL"});
            /* BOTH descriptors, THEN one IP read.  A second IP read would start a second poll and
               the deferral would never happen. */
            seq.push({s: "clrint"});
            seq.push({s: "rdesc", slot: 1, buf: 1});
            seq.push({s: "rdesc", slot: 2, buf: 2});
            seq.push({s: "cdesc", slot: 1, pkt: 1});
            seq.push({s: "cdesc", slot: 2, pkt: 2});
            seq.push({s: "ip"});
            seq.push({s: "await", slot: 1, what: "first transfer's response descriptor"});
            seq.push({s: "await", slot: 2, what: "DEFERRED transfer's response descriptor"});
            seq.push({s: "delay", n: 600});
            return seq;
        }
    }));

    /* ---- 16. THE WRITE-PROTECT LADDER.  Unit 0 is `set locked`, so RQ_WPH is true and a WRITE is
       ST_WPR|SB_WPR_HW while a READ of the same blocks is fine.  Unit 1 is writable and gets
       UF_WPS through SET UNIT CHARACTERISTICS with MD_SWP -- *** BOTH the host's UF_WPS bit AND
       MD_SWP are required *** -- and its WRITE is ST_WPR|SB_WPR_SW.  rq_rw_valid() tests the
       SOFTWARE lock first and rq_svc() tests the HARDWARE one first, so the two rungs are reachable
       independently.  An ERASE at the RCT window is ST_CMD|I_LBN, the second `lbn >= maxlbn` test
       that only write operations reach. ---- */
    add(build({
        name: "the write-protect ladder: hardware lock, software lock, and ERASE into the RCT",
        kind: "wprot", ncmds: 7, spec: {spread: 53, dataLnt: 2 * PAGE},
        units: [unitSpec({dtype: RX18_DTYPE, image: "wr", locked: true}),
                unitSpec({dtype: RX18_DTYPE, image: "wr2"}),
                unitSpec({dtype: RD54_DTYPE})],
        cmds: (g) => [
            {opc: OP.ONL, unit: 0, ref: 0x1F00},
            {opc: OP.ONL, unit: 1, ref: 0x1F01},
            {opc: OP.WR,  unit: 0, ref: 0x1F02, bc: 512, ba: g.dataBase, lbn: 4},
            {opc: OP.RD,  unit: 0, ref: 0x1F03, bc: 512, ba: g.dataBase, lbn: 4},
            {opc: OP.SUC, unit: 1, ref: 0x1F04, ufl: UF_WPS, mod: MD_SWP},
            {opc: OP.WR,  unit: 1, ref: 0x1F05, bc: 512, ba: g.dataBase, lbn: 4},
            {opc: OP.ERS, unit: 1, ref: 0x1F06, bc: 512, ba: g.dataBase,
             lbn: capacFor(IMG.wr, RX18_DTYPE, false)}],
        tags: ["ONL locked unit", "ONL writable unit", "WR a locked unit -> SB_WPR_HW",
               "RD the locked unit (legal)", "SUC UF_WPS + MD_SWP",
               "WR a software-locked unit -> SB_WPR_SW", "ERS into the RCT -> I_LBN"]
    }));

    /* ---- 17. THE CONTAINER THAT IS SHORTER THAN THE UNIT.  A WRITABLE attach takes autosize's
       clamp, so a 100-block file on an RD54 gives a 311,200-block UNIT -- and a perfectly legal
       READ that starts inside the file and runs off the end comes back HALF REAL DATA AND HALF
       ZEROS, because a short pread(2) is not an error on a RAW container.  This is the same shape
       the real OpenVMS volume has (the file is 2,936,985 blocks and the volume declares 2,940,951
       sectors), and it is here so that shape is graded even when PHASE R is skipped. ---- */
    add(build({
        name: "a read straddling the END OF THE CONTAINER inside a larger unit",
        kind: "shortfile", ncmds: 4,
        units: [unitSpec({dtype: RD54_DTYPE, image: "short"})],
        spec: {spread: 59, dataLnt: 4 * PAGE},
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x2000},
            {opc: OP.RD, unit: 0, ref: 0x2001, bc: 2048, ba: g.dataBase, lbn: 98},
            {opc: OP.RD, unit: 0, ref: 0x2002, bc: 512, ba: g.dataBase + 2048, lbn: 5000},
            {opc: OP.RD, unit: 0, ref: 0x2003, bc: 512, ba: g.dataBase + 2560, lbn: 99}],
        tags: ["ONL", "RD straddling the end of the file", "RD far past the end of the file",
               "RD the last real block"]
    }));

    /* ---- 18. *** THE 2.5 GiB SPARSE CONTAINER. ***  Block 5,242,879's byte offset is
       2,684,354,048 -- past 2^31 -- and `(lbn * 512) | 0`, the natural JS idiom for that
       multiplication, goes NEGATIVE there.  This case is why the `image-read-offset-computed-in-
       32-bits` mutation dies without the user's real volume being present.  Three blocks have real
       bytes and everything between them is a hole, which reads as zeros on both engines. ---- */
    add(build({
        name: "a 2.5 GiB container -- block offsets past 2^31",
        kind: "bigoffset", ncmds: 4,
        units: [unitSpec({dtype: RA92_DTYPE, image: "huge", ro: true})],
        spec: {spread: 61, dataLnt: 2 * PAGE},
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x2100},
            {opc: OP.RD, unit: 0, ref: 0x2101, bc: 512, ba: g.dataBase, lbn: IMG.huge.blocks - 1},
            {opc: OP.RD, unit: 0, ref: 0x2102, bc: 512, ba: g.dataBase + 512, lbn: 4000000},
            {opc: OP.RD, unit: 0, ref: 0x2103, bc: 1024, ba: g.dataBase + 1024, lbn: 4000000}],
        tags: ["ONL", "RD the LAST block of a 2.5GiB container", "RD block 4,000,000",
               "RD 2 blocks at 4,000,000 (one real, one hole)"]
    }));

    /* ---- 19. THE WRITES.  Everything that changes a container happens here and ONLY here, on the
       one image each engine has its own copy of -- PHASE I compares the two copies byte for byte.
       A WRITE of 700 bytes is the mirror of case 6: rq_svc() rounds UP to a whole block, ZEROES the
       tail of rqxb from the host's last byte to the block boundary, and writes 1,024 bytes -- so
       the tail of the block on DISK must be zeros and not the host's next bytes.  ERASE writes
       zeros without reading host memory at all. ---- */
    add(build({
        name: "WRITE, a partial-block WRITE and ERASE -- then read them all back",
        kind: "write", ncmds: 7,
        units: [unitSpec({dtype: RX18_DTYPE, image: "wr"})],
        spec: {spread: 67, dataLnt: 4 * PAGE},
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x2200},
            {opc: OP.WR,  unit: 0, ref: 0x2201, bc: 1024, ba: g.dataBase, lbn: 10},
            {opc: OP.WR,  unit: 0, ref: 0x2202, bc: 700,  ba: g.dataBase + 1024, lbn: 30},
            {opc: OP.ERS, unit: 0, ref: 0x2203, bc: 1024, ba: 0, lbn: 20},
            {opc: OP.RD,  unit: 0, ref: 0x2204, bc: 1024, ba: g.dataBase + 2048, lbn: 10},
            {opc: OP.RD,  unit: 0, ref: 0x2205, bc: 1024, ba: g.dataBase + 3072, lbn: 20},
            {opc: OP.CMP, unit: 0, ref: 0x2206, bc: 1024, ba: g.dataBase, lbn: 10}],
        tags: ["ONL", "WR 2 blocks", "WR 700 bytes (the block's tail must be zeroed on disk)",
               "ERS 2 blocks", "RD back the WR", "RD back the ERS", "CMP the WR -> ST_SUC"]
    }));

    return cases;
}

/* ------------------------------------------------------------------------------------------- *
 * Unit setup: the SCP sequence a USER performs, written once for both engines                   *
 *                                                                                               *
 * The order is tests/mscpunitdiff.js's and every step undoes a state a PREVIOUS case could have  *
 * left behind -- `reset -p all` detaches nothing, `set <type>` is refused while attached, and    *
 * UNIT_WLK survives a detach so `set writeenabled` has to run before `set locked` can mean       *
 * anything.  See that file's header for the full argument.                                      *
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

const MARK = "MRWCASE";

function simhCaseLines(c)
{
    let L = [];
    L.push(`echo ${MARK}${c.idx}`);
    L.push(...simhSetupLines(c));
    L.push(...simhResetLines(c));
    L.push(...simhAttachLines(c));
    /* The MSCP-level map's backing store, zeroed, so a case that does NOT program it fails through
       "no valid entry" rather than through whatever a previous case left there. */
    L.push(`deposit -l ${hex(MSCPMAP_BASE)}:${hex(MSCPMAP_HI - 4)} 0`);
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
    let script = L.join("\n");
    let out = runSimh(simh, script, path.join(opts.scratch, "mscprw-cases.ini"));
    if (opts.rawdump !== null && opts.rawdump !== undefined) {
        let parts0 = out.split(new RegExp("^" + MARK + "(\\d+)\\s*$", "m"));
        for (let i = 1; i < parts0.length; i += 2) {
            if (String(parts0[i]) !== String(opts.rawdump)) continue;
            console.log(`\n=== RAW ORACLE OUTPUT FOR CASE ${parts0[i]} ===`);
            let txt = parts0[i + 1] || "";
            console.log(txt.split("\n").filter((l) => !/^[0-9A-F]{1,8}:\s*[0-9A-F]{8}\s*$/.test(l) || /^11[0-9A-F]{4}:/.test(l)).join("\n").slice(0, 8000));
        }
    }

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
    r.mem = new Map();
    /* *** NOT `{6,8}`. ***  SIMH does not zero-pad an EXAMINE address to a fixed width, so a dump
       of low memory prints `1E00: ...` and a six-digit pattern silently drops the whole page --
       which reads as "the oracle never reported memory there", i.e. as a broken do-file rather
       than as a broken parser.  The identity-mapped case is the only one in this file whose
       physical pages ARE its Qbus pages, so it is the only one that reaches low memory at all, and
       it is exactly the case whose survival the map-cheat floor rests on.  A register name cannot
       collide with this pattern: every RQ_OBS name contains at least one letter that is not a hex
       digit. */
    let re = /^([0-9A-F]{1,8}):\s*([0-9A-F]{8})\s*$/gm, m;
    while ((m = re.exec(chunk)) !== null) r.mem.set(parseInt(m[1], 16) >>> 0, parseInt(m[2], 16) >>> 0);
    /* *** SIMH COLLAPSES CONSECUTIVE IDENTICAL DEBUG LINES *** (scp.c:13836-13900): a run of N+1
       identical lines is printed as the first one followed by `same as above (N times)`, stamped
       with the time of the LAST occurrence, and the collapse line carries NO DEVICE TAG -- so a
       parser anchored on `RQ REQ:` reads N repeats as one.  It matters more here than anywhere: a
       three-chunk transfer's three disk-trace lines differ only in `lbn:`, but a case that reads
       the SAME extent twice produces two identical ones and they WILL be collapsed. */
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

    for (let a = MSCPMAP_BASE; a < MSCPMAP_HI; a += 4) bus.setLong(a >>> 0, 0);
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
        /* The two states this item made reachable and did not take into scope, sampled so
           assertExclusions() can fence them on evidence rather than on the case list alone. */
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
 * PHASE R -- a REAL OpenVMS ODS-2 volume                                                        *
 * ------------------------------------------------------------------------------------------- */

/** The ODS-2 home block's format identifier.  It lives in LBN 1 of a DECFILE11B volume, and it is
    checked in the bytes THE ORACLE delivered rather than in the file: that is the difference
    between "the transfer worked" and "the file contains this string". */
const ODS2_SIG = "DECFILE11B";

/**
 * realVolumeCases(vol)
 *
 * The real volume, attached READ ONLY on both engines.  Read-only because a read-only run is
 * reproducible and cannot corrupt state a later run depends on -- and because with `-R` autosize
 * skips the `container < current` clamp, so the unit's capacity is the CONTAINER's block count
 * exactly and the RCT window begins at the real end of the file.
 *
 * *** THE DETAIL WORTH GRADING. ***  The file is 2,936,985 blocks; the volume itself declares
 * 2,940,951 sectors (`Volume Name: VAX1  Format: DECFILE11B  Sectors In Volume: 2940951`).  The
 * container is SHORTER than the volume says it is -- real-world data, not corruption -- so a
 * transfer near the end has to behave exactly as SIMH does.  Read-only, that edge is `capac`
 * itself: the last block of the file, the first block of the RCT window (answered from a ZEROED
 * buffer without touching the container), and an LBN past the RCT (ST_CMD|I_LBN).
 */
function realVolumeCases(vol)
{
    let cases = [];
    let add = (c) => { c.idx = 1000 + cases.length; cases.push(c); return c; };
    let cap = Math.floor(vol.bytes / RQ_NUMBY);
    let units = () => [unitSpec({dtype: RA92_DTYPE, image: "volume", ro: true})];

    /* The ODS-2 HOME BLOCK, on its own page, so its bytes can be searched for the volume format
       identifier in the ORACLE's own dump. */
    add(build({
        name: "REAL VOLUME: the ODS-2 home block",
        kind: "vol-home", ncmds: 2, units: units(), spec: {spread: 71, dataLnt: PAGE},
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x3000},
            {opc: OP.RD, unit: 0, ref: 0x3001, bc: 512, ba: g.dataBase, lbn: 1}],
        tags: ["ONL", "RD the home block"]
    }));

    /* Eight blocks across five scattered, out-of-order pages -- the discontiguity floor again, this
       time over data nobody generated. */
    add(build({
        name: "REAL VOLUME: 4KB across scattered pages",
        kind: "vol-scatter", ncmds: 2, units: units(), spec: {spread: 73, dataLnt: 10 * PAGE},
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x3100},
            {opc: OP.RD, unit: 0, ref: 0x3101, bc: 4096, ba: g.dataBase + 256, lbn: 0}],
        tags: ["ONL", "RD 8 blocks starting mid-page"]
    }));

    /* THE END OF THE CONTAINER.  The last real block, the first block of the RCT window, and an
       LBN past the RCT.  The last block's byte offset is 1,503,735,808 -- past 2^31. */
    add(build({
        name: "REAL VOLUME: the last block, the RCT window and past it",
        kind: "vol-end", ncmds: 4, units: units(), spec: {spread: 79, dataLnt: 2 * PAGE},
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x3200},
            {opc: OP.RD, unit: 0, ref: 0x3201, bc: 512, ba: g.dataBase, lbn: cap - 1},
            {opc: OP.RD, unit: 0, ref: 0x3202, bc: 512, ba: g.dataBase + 512, lbn: cap},
            {opc: OP.RD, unit: 0, ref: 0x3203, bc: 512, ba: g.dataBase + 1024,
             lbn: cap + DRV_TAB[RA92_DTYPE].rcts}],
        tags: ["ONL", "RD the last block of the container", "RD the RCT window", "RD past the RCT"]
    }));

    /* A WRITE to a READ-ONLY attachment, which is how the volume is protected from this run by the
       CONTROLLER rather than by the test remembering not to. */
    add(build({
        name: "REAL VOLUME: a WRITE is refused -- the container is attached read only",
        kind: "vol-wprot", ncmds: 3, units: units(), spec: {spread: 83},
        cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x3300},
            {opc: OP.WR, unit: 0, ref: 0x3301, bc: 512, ba: g.dataBase, lbn: 100},
            {opc: OP.ERS, unit: 0, ref: 0x3302, bc: 512, ba: 0, lbn: 100}],
        tags: ["ONL", "WR -> ST_WPR|SB_WPR_HW", "ERS -> ST_WPR|SB_WPR_HW"]
    }));

    return cases;
}

/* ------------------------------------------------------------------------------------------- *
 * The randomized phase                                                                          *
 * ------------------------------------------------------------------------------------------- */

/**
 * randomCases(n, seed, startIdx)
 *
 * HANDOFF.md standing rule 1: a real-workload phase AND a randomized one, because each has caught a
 * bug the other structurally could not see.  The enumerated cases above are chosen to hit named
 * branches; these are chosen to hit combinations nobody thought of -- an odd buffer offset against
 * an odd ring code against a comm region above 64KB against a byte count that straddles three
 * pages.  Every parameter is drawn so the command is VALID, because an invalid one is answered
 * before any of the machinery under test runs.
 */
function randomCases(n, seed, startIdx)
{
    let rnd = mulberry32(seed);
    let pick = (a) => a[(rnd() * a.length) | 0];
    let cases = [];
    for (let k = 0; k < n; k++) {
        let writeCase = rnd() < 0.3;
        let img = writeCase ? "wr" : pick(["small", "small", "short"]);
        let dtype = writeCase ? RX18_DTYPE : RD54_DTYPE;
        let ro = !writeCase && img === "small";
        let cap = capacFor(IMG[img], dtype, ro);
        let nblk = 1 + ((rnd() * 6) | 0);                   /* 1..6 blocks of buffer */
        let bc = 2 * (1 + ((rnd() * (nblk * RQ_NUMBY / 2)) | 0));
        let baOff = 2 * ((rnd() * ((RQ_NUMBY * 2) / 2)) | 0);
        /* Keep the transfer inside the unit: rq_rw_valid() rejects a spiral, and a case that is
           refused grades the ladder rather than the transfer -- which the enumerated cases already
           do, deliberately and one rung at a time. */
        let need = Math.ceil(bc / RQ_NUMBY);
        let lbn = (rnd() * Math.max(1, cap - need)) | 0;
        let opc = writeCase ? pick([OP.WR, OP.ERS, OP.WR]) : pick([OP.RD, OP.RD, OP.RD, OP.ACC, OP.CMP]);
        if (writeCase) lbn = Math.min(lbn, IMG[img].blocks - need);  /* never extend the container */
        let comm = pick([0x2000, 0x14000, 0x22000]);
        let code = 1 + ((rnd() * 3) | 0);
        let dataLnt = (Math.ceil((baOff + bc) / PAGE) + 1) * PAGE;
        cases.push(build({
            name: `random #${k}: ${RQVAX.CMD_NAMES[opc & 0x3F].trim()} bc=${bc} lbn=${lbn} ` +
                  `ba+${baOff} comm=0x${comm.toString(16)} rings=${1 << code}`,
            kind: "random", ncmds: 2,
            units: [unitSpec({dtype, image: img, ro})],
            spec: {spread: 101 + k, comm, cqCode: code, rqCode: code, dataLnt},
            cmds: (g) => [{opc: OP.ONL, unit: 0, ref: 0x5000 + k},
                {opc, unit: 0, ref: 0x5100 + k, bc, ba: g.dataBase + baOff, lbn}],
            tags: ["ONL", "the random transfer"]
        }));
        cases[cases.length - 1].idx = startIdx + k;
    }
    return cases;
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

/** Which Qbus address a physical one came from, for a failure message that says "the third page of
    the transfer's buffer" rather than "0x0047FE00". */
function whereIs(c, pa)
{
    let page = (pa / PAGE) | 0;
    for (let [q, p] of c.qToP) {
        if (p !== page) continue;
        let qa = q * PAGE + (pa % PAGE);
        let g = c.g;
        let rel = (base, lnt, what) => (qa >= base && qa < base + lnt)
            ? `${what}+0x${(qa - base).toString(16)}` : null;
        return "Qbus 0x" + hex(qa, 6) + " [" + (
            rel(g.dataBase, g.dataLnt, "the transfer buffer") ||
            (g.mirrorBase ? rel(g.mirrorBase, g.mirrorLnt, "the MSCP map's mirror region") : null) ||
            rel(g.rspBase, c.nRspBuf * 64, "a response packet") ||
            rel(g.cmdBase, c.nCmdBuf * 64, "a command packet") ||
            rel(g.rqBa, g.rqLnt, "the response ring") ||
            rel(g.cqBa, g.cqLnt, "the command ring") || "the comm region") + "]";
    }
    return "physical 0x" + hex(pa);
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
                `(oracle PC=0x${hex(s.pc)}, here PC=0x${hex(j.pc)})`);
        }
        compared++;
        /* *** R9..R12 ARE THE AWAIT LOOP'S SCRATCH AND THEY ARE NOT COMPARED EXACTLY. ***  They
           hold the LAST wait's iteration count and remaining budget, and a wait that covered a disk
           operation is not reproducible on the oracle -- assertSchedule() below states exactly what
           IS required of them.  R0..R8 are the handshake's read-backs and poll counts and remain
           exact; R13 is the delay/IP scratch and is exact. */
        for (let k = 0; k < OBS_REGS; k++) {
            if (k >= 9 && k <= 12 && c.steps.some((st) => st.s === "await" && st.xfer)) continue;
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
        /* *** EVERY BYTE OF EVERY PHYSICAL PAGE THE TRANSFER COULD HAVE TOUCHED, WHOLE. ***  Not the
           intended extent: the pages are dumped and compared entire, so a transfer that overran by
           one byte, or wrote the right bytes to the page next door, fails here rather than passing a
           comparison of the range it was supposed to fill. */
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
    }
    return compared;
}

/**
 * gradeTrace(c, s, j, failures)
 *
 * The ordered `set rq debug=REQ` stream, compared as a SEQUENCE.  For this item it is the strongest
 * single view there is: besides `cmd=` and `rsp=`, it carries sim_disk_data_trace()'s ONE LINE PER
 * DISK OPERATION -- `RQ0 sim_disk_rdsect lbn: 00000000 len: 00010000` -- so the CHUNKING of a
 * transfer larger than RQ_MAXFR, the `ceil(bc/512)` blocks a partial-block read issues, and the
 * order of a WRITE's trace-then-write against a READ's read-then-trace are all compared as TEXT
 * against the simulator rather than inferred from the bytes that arrived.
 */
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

/**
 * gradeImages(images, failures, acc)
 *
 * PHASE I.  Each engine wrote to its OWN copy of the same container; the two must come back
 * byte-identical.  *** THIS IS THE ONLY VIEW THAT CAN GRADE A WRITE AT ALL. ***  A WRITE's bytes
 * leave host memory and never come back, so every memory comparison in this file is blind to them;
 * what proves rq_svc()'s write path -- the fetch through the map, the zero-fill of the last
 * block's tail, the block number, the chunking -- is the file it produced.
 */
function gradeImages(images, failures, acc)
{
    for (let im of images) {
        if (!im.writable) continue;
        let a = fs.statSync(im.simhPath).size, b = fs.statSync(im.path).size;
        if (a !== b) {
            failures.push(`PHASE I: container "${im.tag}" is ${b} bytes here and ${a} on the oracle ` +
                `(it started at ${im.bytes}).  A pwrite(2) past the end of a container EXTENDS it, ` +
                `so the two engines disagree about how far a transfer reached.`);
            continue;
        }
        let sa = shaOf(im.simhPath), sb = shaOf(im.path);
        acc.imagesCompared++;
        if (fs.statSync(im.pristine).size === a && shaOf(im.pristine) === sa && im.tag === "wr") {
            failures.push(`PHASE I: container "${im.tag}" came back IDENTICAL TO THE PRISTINE COPY ` +
                `on the oracle, so no graded WRITE or ERASE actually reached the disk and this ` +
                `phase compared two untouched files.  That is a coverage hole, not a pass.`);
        }
        if (sa !== sb) {
            /* Name the first differing BLOCK, because "the files differ" is not actionable and the
               block number is what maps back to a command. */
            let bad = firstDifferingBlock(im.simhPath, im.path);
            failures.push(`PHASE I: container "${im.tag}" differs after the run -- first difference ` +
                `in BLOCK ${bad.lbn} at byte ${bad.off} (here 0x${bad.js.toString(16)}, oracle ` +
                `0x${bad.simh.toString(16)}).  The two engines wrote different bytes to the disk.`);
        } else {
            acc.imagesMatched++;
        }
    }
}

function firstDifferingBlock(pa, pb)
{
    let fa = fs.openSync(pa, "r"), fb = fs.openSync(pb, "r");
    try {
        let ba = Buffer.allocUnsafe(1 << 16), bb = Buffer.allocUnsafe(1 << 16), off = 0;
        for (;;) {
            let na = fs.readSync(fa, ba, 0, ba.length, off);
            let nb = fs.readSync(fb, bb, 0, bb.length, off);
            let n = Math.min(na, nb);
            if (n <= 0) return {lbn: -1, off: -1, js: -1, simh: -1};
            for (let i = 0; i < n; i++) {
                if (ba[i] !== bb[i]) {
                    let abs = off + i;
                    return {lbn: (abs / RQ_NUMBY) | 0, off: abs % RQ_NUMBY, js: bb[i], simh: ba[i]};
                }
            }
            off += n;
        }
    } finally { fs.closeSync(fa); fs.closeSync(fb); }
}

/* ------------------------------------------------------------------------------------------- *
 * Exclusion fences and coverage floors.  Every one FAILS the run; none scales with case count.   *
 * ------------------------------------------------------------------------------------------- */

function assertExclusions(cases, sim, js, failures)
{
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i];
        if (c.s1dat & SA_S1H_VEC) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" supplies an S1 word with a non-zero ` +
                `SA_S1H_VEC.  Interrupt delivery LANDED in pcjsvax-aef and is graded by ` +
                `tests/mscpintdiff.js; this fence is now a SCOPE boundary, not a gap -- every wait ` +
                `in this file is an IN-BAND loop whose ITERATION COUNT is graded, an SCB dispatch ` +
                `inside one would fold interrupt delivery into a measurement of the controller's ` +
                `event schedule, and no SCB handler is installed for the RQ vector here at all.`);
        }
        if (c.s1dat & SA_S1H_IE) {
            failures.push(`exclusion: case ${c.idx} "${c.name}" supplies an S1 word with SA_S1H_IE set`);
        }
        for (let cs of c.cmdSpecs) {
            /* ABORT and GET COMMAND STATUS against a unit HOLDING A PACKET reach rq.js's throws.
               No case sends either at all, and the fence says so from the case list rather than
               from a comment -- a case that added one would have to think about it. */
            if (cs.opc === OP.ABO || cs.opc === OP.GCS) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" sends OP_${cs.opc === OP.ABO ? "ABO" : "GCS"}, ` +
                    `whose behaviour against a unit with a transfer in flight is out of ` +
                    `pcjsvax-346's scope and throws by name in rq.js`);
            }
            /* *** A WRITE WHOSE BUFFER FETCH FAILS ENTIRELY HANGS THE REAL SIMULATOR. ***  rq_svc()'s
               top end issues no disk I/O, takes no callback, and never re-schedules the unit.  A
               case that unmapped the FIRST page of a write's buffer would wait out its in-band
               budget on BOTH engines and compare two hangs as equal. */
            if ((cs.opc === OP.WR) && cs.ba !== undefined) {
                let first = ((cs.ba & ~RQ_MAPXFER) / PAGE) | 0;
                if (!c.qToP.has(first) && !c.mapped) {
                    failures.push(`exclusion: case ${c.idx} "${c.name}" posts a WRITE whose buffer ` +
                        `STARTS on an unmapped Qbus page.  rq_svc() would issue no disk write, take ` +
                        `no callback and leave the unit unscheduled forever -- the real simulator ` +
                        `hangs the command, and two engines that both hang compare equal`);
                }
            }
        }
        let j = js[i];
        if (!j || j.unimplemented) continue;
        /* THE STATE THIS ITEM MADE REACHABLE AND DID NOT TAKE INTO SCOPE, checked on EVIDENCE: every
           graded case must end with no transfer in flight and no drive event pending, which is
           exactly the state that makes rq_abo()'s and rq_gcs()'s search arms unreachable and stops
           `io_complete` -- which rq_reset() does NOT clear -- from carrying into the next case. */
        for (let n = 0; n < RQ_NUMDR; n++) {
            if (j.ioComplete[n]) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" ends with unit ${n}'s ` +
                    `io_complete SET.  rq_reset() does not clear it, so the next case would resume ` +
                    `at the BOTTOM end of a read that no longer has a packet`);
            }
            if (j.unitDue[n]) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" ends with unit ${n} still on ` +
                    `the event queue -- a transfer that neither completed nor was cancelled`);
            }
        }
        let s = sim[i];
        if (!s) continue;
        /* *** THE OBSERVATION POINT, ASSERTED ON BOTH ENGINES RATHER THAN ASSUMED. ***  Every
           graded case is sampled only after its last in-band wait has seen its response, and the
           waits are unbounded -- so the controller must be IDLE here: no packet busy, no queued
           response, no poll in progress.  If it is not, the two engines are being compared at
           different points in some command's life and every register below is noise.  This is the
           check that would have named the flake the bounded wait produced, instead of letting it
           surface as twenty scattered state differences on a randomized WRITE case. */
        for (let [name, want] of [["PBSY", 0], ["RESP", 0], ["PIP", 0]]) {
            if (s.rq[name] !== want) {
                failures.push(`OBSERVATION POINT: case ${c.idx} "${c.name}" was sampled with the ` +
                    `ORACLE's ${name} = ${hex(s.rq[name], 4)}, not ${want}.  The controller is not ` +
                    `idle at the point this case compares state, so it is being observed part way ` +
                    `through a command and nothing else it reports is comparable.`);
            }
            if (j.rq[name] !== want) {
                failures.push(`OBSERVATION POINT: case ${c.idx} "${c.name}" was sampled with THIS ` +
                    `ENGINE's ${name} = ${hex(j.rq[name], 4)}, not ${want} -- same statement, this side`);
            }
        }
        for (let n = 0; n < RQ_NUMDR; n++) {
            if (s.units[n].CPKT !== 0 || s.units[n].PKTQ !== 0) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" leaves unit ${n} holding ` +
                    `CPKT=${s.units[n].CPKT} PKTQ=${s.units[n].PKTQ} ON THE ORACLE -- a transfer in ` +
                    `flight past the end of a case, which is the state rq_abo()/rq_gcs()'s excluded ` +
                    `arms need and which would make the next case order-dependent`);
            }
        }
    }
}

function assertSchedule(cases, sim, js, failures, acc)
{
    for (let i = 0; i < cases.length; i++) {
        let c = cases[i], s = sim[i];
        if (!s || !s.mem) continue;
        for (let st of c.steps) {
            if (st.s !== "await") continue;
            let count = s.mem.get((R_RESULT + st.roff) >>> 0);
            if (count === undefined) continue;
            acc.waitCounts.add(count);
            if (count > 0) acc.nonZeroWaits++; else acc.zeroWaits++;
            if (!st.xfer) continue;
            /* *** THE MEASUREMENT THAT REPLACED AN EXACT COMPARISON, AND WHY. ***  This item was
               handed "NOASYNCH=1, so sim_disk_rdsect_a completes inline and rq_io_complete's
               re-schedule is DETERMINISTIC -- confirm that rather than assuming it".  Confirmed
               FALSE in its second half: the completion is inline (PHASE V derives that from the
               source), but the delay it produces IS NOT REPRODUCIBLE.  Measured on the live oracle,
               same binary, same do-file, three consecutive runs: the in-band wait for the SAME
               single-block READ came back 129, 153 and 221 iterations.  A three-chunk transfer
               moved between 623 and 730.  The queue rq_io_complete()'s `sim_activate_notbefore()`
               inserts into is shared with RQ_TIMER, whose `sim_activate_after (…, 1000000)` is
               WALL-CLOCK calibrated -- the same root cause HANDOFF.md section 3 records for
               conoutdiff's byte count.
               So the exact count is NOT graded for a wait that covers a disk operation.  What IS,
               and all three fail the run:
                 - the wait must be NON-ZERO on both engines (a controller that moved the blocks
                   inside the IP read gives zero, which is the synchronous-transfer cheat);
                 - the ORACLE's wait must be at least this engine's, because this engine models the
                   floor -- qtime for the poll plus xtime for the disk -- and the oracle only ever
                   adds to it;
                 - this engine's wait must be at least the floor DERIVED from the timing registers
                   the oracle itself was pinned with, so a model that skipped xtime is caught here
                   rather than passing a comparison that no longer looks at the number. */
            let jcount = js[i] && js[i].mem.get((R_RESULT + st.roff) >>> 0);
            /* Both waits are UNBOUNDED, so reaching this point at all means BOTH engines saw the
               descriptor change -- i.e. both were sampled with the command finished.  What is left
               to say about the numbers is a band; see the comment on NO_WAIT_BUDGET. */
            /* THE FLOOR IS XTIME ALONE, NOT QTIME + XTIME, and the difference is not cosmetic: a
               command DEFERRED onto a unit queue is picked up by a queue thread that is ALREADY
               ARMED, so it pays no fresh qtime and its wait is legitimately shorter than a freshly
               polled command's.  Measured: the oracle answered a deferred transfer in 63 poll
               iterations and this engine in 38, both of which include the disk delay and neither of
               which includes a second qtime.  What every transfer that reaches rq_svc must pay is
               XTIME, and that is what is required here. */
            let floor = Math.floor(c.xtime / 6) - 4;
            if (!(count > 0)) {
                failures.push(`schedule: case ${c.idx} "${c.name}": the ORACLE answered ${st.what} ` +
                    `on the FIRST poll iteration -- there is no delay to grade`);
            }
            if (!(jcount > 0)) {
                failures.push(`schedule: case ${c.idx} "${c.name}": THIS ENGINE answered ${st.what} ` +
                    `on the first poll iteration, i.e. synchronously inside the IP read`);
            }
            /* *** THE FLOOR APPLIES ONLY WHEN THE ORACLE ITSELF WAITED THAT LONG. ***  A transfer
               rq_rw_valid() REFUSES never reaches rq_svc and never touches the disk, so it answers
               after qtime alone -- about 20 iterations -- on both engines, and demanding the
               qtime+xtime floor of it would fail eight perfectly correct refusals.  The oracle is
               what says which kind of wait this was: if IT waited out the disk delay, so must
               this engine. */
            if (count >= floor && jcount < floor) {
                failures.push(`schedule: case ${c.idx} "${c.name}": the ORACLE waited ${count} ` +
                    `iteration(s) for ${st.what} and this engine waited ${jcount}, below the floor ` +
                    `of ${floor} derived from the oracle's own QTIME (${c.qtime}) and XTIME ` +
                    `(${c.xtime}) -- the transfer did not wait out the disk delay`);
            }
            if (count >= floor) acc.diskWaits++;
            if (count < jcount) {
                failures.push(`schedule: case ${c.idx} "${c.name}": the ORACLE's wait for ${st.what} ` +
                    `was ${count} iteration(s) and this engine's was ${jcount}.  This engine models ` +
                    `the FLOOR (qtime + xtime with no per-chunk re-wait), so it can never legally ` +
                    `exceed the oracle`);
            }
            acc.xferWaits++;
        }
    }
}

/** The trace lines a disk operation produces, parsed back out of the ORACLE's own stream. */
function diskOps(trace)
{
    let out = [];
    for (let e of trace) {
        let m = /^(RQ\d+)\s+(sim_disk_\S+)\s+lbn:\s*([0-9A-F]{8})\s+len:\s*([0-9A-F]{8})$/.exec(e.line);
        if (m) out.push({unit: m[1], op: m[2], lbn: parseInt(m[3], 16) >>> 0, len: parseInt(m[4], 16) >>> 0});
    }
    return out;
}

function coverage(cases, sim, js, failures, acc, opts)
{
    let ok = (i) => sim[i] && js[i] && !js[i].unimplemented && sim[i].halted && js[i].halted &&
                    sim[i].atOwnHalt && js[i].atOwnHalt;
    let byKind = (k) => cases.findIndex((c, i) => ok(i) && c.kind === k);

    /* ---- THE SYNCHRONY FLOOR, re-asserted rather than inherited: at least three DISTINCT in-band
       iteration counts matched, and not all of them zero.  A controller that moved the blocks
       inside the IP read produces zero for every wait. ---- */
    if (acc.waitCounts.size < 3) {
        failures.push(`coverage: only ${acc.waitCounts.size} distinct in-band response-wait ITERATION ` +
            `COUNT(s) reached comparison; the floor is 3`);
    }
    if (acc.nonZeroWaits === 0) {
        failures.push(`coverage: every graded response wait saw its answer on the FIRST iteration -- ` +
            `the controller is answering synchronously inside the IP read`);
    }

    /* ---- *** THE DISCONTIGUITY FLOOR. ***  Re-derived from the case's own map entries, not
       assumed from the way physFor() is written: the transfer's Qbus pages must reach THREE OR MORE
       physical pages that are neither adjacent nor in order.  A scatter function that quietly
       became monotone would fail HERE rather than leaving the identity-map cheat alive. ---- */
    let di = byKind("discontig");
    if (di < 0) {
        failures.push(`coverage: the discontiguous-map case never reached comparison, so the ` +
            `identity-map cheat pcjsvax-e22 named is UNTESTED in this run`);
    } else {
        let c = cases[di], g = c.g;
        let qs = [];
        for (let a = g.dataBase; a < g.dataBase + g.dataLnt; a += PAGE) qs.push((a / PAGE) | 0);
        let ps = qs.map((q) => c.qToP.get(q));
        acc.discontigPages = ps.length;
        let monotone = ps.every((p, k) => k === 0 || p > ps[k - 1]) ||
                       ps.every((p, k) => k === 0 || p < ps[k - 1]);
        let adjacent = ps.every((p, k) => k === 0 || Math.abs(p - ps[k - 1]) === 1);
        if (ps.length < 3) {
            failures.push(`coverage: the discontiguous case spans only ${ps.length} map entr(ies); ` +
                `the floor is 3 -- fewer cannot tell a map from an identity`);
        }
        if (monotone || adjacent) {
            failures.push(`coverage: the discontiguous case's ${ps.length} physical pages are ` +
                `${adjacent ? "ADJACENT" : "MONOTONE"} (${ps.join(",")}), so an identity or a ` +
                `constant-offset map would produce the same bytes and the floor grades nothing`);
        }
    }

    /* ---- THE CHUNKING FLOOR, read out of the ORACLE's own disk-trace lines: three or more disk
       operations for ONE command, at three DIFFERENT block numbers, and exactly ONE end packet. ---- */
    let ci = byKind("chunk");
    if (ci < 0) {
        failures.push(`coverage: the >RQ_MAXFR case never reached comparison, so rq_svc()'s chunking ` +
            `and its rewrite of RW_WBAL/RW_WBCL/RW_WBLL are UNGRADED`);
    } else {
        let ops = diskOps(sim[ci].trace);
        acc.chunkOps = ops.length;
        let lbns = new Set(ops.map((o) => o.lbn));
        if (ops.length < 3 || lbns.size < 3) {
            failures.push(`coverage: the >RQ_MAXFR case produced ${ops.length} disk operation(s) at ` +
                `${lbns.size} distinct LBN(s) ON THE ORACLE; the floor is 3 and 3.  A single ` +
                `operation means the transfer was not chunked at all`);
        }
        if (!ops.some((o) => o.len === RQ_MAXFR)) {
            failures.push(`coverage: no disk operation in the >RQ_MAXFR case moved exactly RQ_MAXFR ` +
                `(${RQ_MAXFR}) bytes on the oracle, so the chunk boundary itself is unexercised`);
        }
        let ends = sim[ci].trace.filter((e) => /^rsp=/.test(e.line)).length;
        if (ends !== 2) {                                   /* the ONLINE and the transfer */
            failures.push(`coverage: the >RQ_MAXFR case produced ${ends} response packet(s) on the ` +
                `oracle and a chunked transfer must produce exactly ONE for the whole command`);
        }
    }

    /* ---- THE PARTIAL-BLOCK FLOOR: a disk operation whose `len:` is NOT a multiple of 512 must have
       been seen on the oracle, and the read that produced it must have covered MORE blocks than the
       byte count needs.  That is the tail that must not reach memory. ---- */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let o of diskOps(sim[i].trace)) {
            if (o.op === "sim_disk_rdsect" && (o.len % RQ_NUMBY) !== 0) acc.partialReads++;
            if (/wrsect/.test(o.op) && (o.len % RQ_NUMBY) === 0) acc.wholeBlockWrites++;
        }
    }
    if (acc.partialReads === 0) {
        failures.push(`coverage: no graded READ on the oracle had a byte count that was not a whole ` +
            `number of ${RQ_NUMBY}-byte blocks, so "the tail of the last block must not reach ` +
            `memory" is unexercised`);
    }

    /* ---- THE OPCODE FLOOR, DERIVED: every opcode whose C handler is rq_rw() must have been sent
       AND answered on the oracle.  The list comes from rq.js's MSCP_XFER_OPS, which comes from
       rq_mscp()'s own dispatch, which PHASE S re-derives from pdp11_rq.c on every run -- so an
       opcode the vendor moved into or out of rq_rw() changes this floor rather than slipping past
       a list written here. ---- */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let cs of cases[i].cmdSpecs) {
            let nm = RQVAX.OP_NAME_OF[cs.opc & 0xFF];
            if (nm && RQVAX.MSCP_XFER_OPS.indexOf(nm) >= 0) acc.xferOps.add(nm);
        }
        for (let e of sim[i].trace) {
            let m = /^rsp=([0-9A-F]{4}), sts=([0-9A-F]{4})$/.exec(e.line);
            if (m) {
                acc.rspSts.add(parseInt(m[2], 16));
                acc.rspOpc.add(parseInt(m[1], 16) & 0xFF);
            }
            if (/^cmd=/.test(e.line)) acc.cmdLines++;
        }
    }
    for (let nm of RQVAX.MSCP_XFER_OPS) {
        if (!acc.xferOps.has(nm)) {
            failures.push(`coverage: OP_${nm} -- which rq_mscp() dispatches to rq_rw() -- was never ` +
                `sent by a graded case, so one of the five commands that share rq_svc() is ungraded`);
        }
    }

    /* ---- THE STATUS FLOOR.  Each of these is a different rung of rq_rw_valid() or a different exit
       from rq_svc(), and every one is READ OUT OF THE ORACLE's trace rather than predicted here. ---- */
    for (let [v, what] of [
        [ST.SUC, "ST_SUC -- a transfer that simply worked"],
        [ST.CMP, "ST_CMP -- a COMPARE that found a difference"],
        [ST.AVL, "ST_AVL -- a transfer to an attached unit that is not ONLINE"],
        [ST.OFL, "ST_OFL, BARE -- no such unit"],
        [ST.OFL | SB_OFL_NV, "ST_OFL|SB_OFL_NV -- the unit exists and has no volume"],
        [ST.HST | SB_HST_OA, "ST_HST|SB_HST_OA -- an odd buffer address"],
        [ST.HST | SB_HST_OC, "ST_HST|SB_HST_OC -- an odd byte count"],
        [ST.HST | SB_HST_NXM, "ST_HST|SB_HST_NXM -- a host bus error during the DMA"],
        [ST.CMD | I_BCNT, "ST_CMD|I_BCNT -- an unreasonable or spiralling byte count"],
        [ST.CMD | I_LBN, "ST_CMD|I_LBN -- an LBN past the replacement table"],
        [ST.WPR | SB_WPR_HW, "ST_WPR|SB_WPR_HW -- a hardware-locked unit"],
        [ST.WPR | SB_WPR_SW, "ST_WPR|SB_WPR_SW -- a software-locked unit"]
    ]) {
        if (!acc.rspSts.has(v)) {
            failures.push(`coverage: the oracle never answered ${what} (status 0x${hex(v, 4)}) in any ` +
                `graded case, so that arm of rq_rw_valid()/rq_svc() is unexercised`);
        }
    }
    if (!acc.rspOpc.has((OP.RD | OP.END) & 0xFF)) {
        failures.push(`coverage: no graded response carried OP_RD|OP_END in its opcode word`);
    }

    /* ---- THE EF_LOG FLOOR, BOTH WAYS.  rq_hbe()'s "logging disabled" arm returns OK, so the END
       packet is identical with and without CF_THS and the only difference is a DATAGRAM on the
       response ring.  Both must have been seen, or the CF_THS test is indistinguishable from a
       constant. ---- */
    let hi = byKind("hbe"), ni = byKind("nxm");
    if (hi < 0 || ni < 0) {
        failures.push(`coverage: the NXM case with error logging ON and the one with it OFF did not ` +
            `both reach comparison, so rq_hbe()'s CF_THS test is unexercised`);
    } else {
        /* The error-log datagram's "opcode" word is a FORMAT and a FLAG, not an opcode:
           `(FM_BAD << RSP_OPF_V_OPC) | (LF_SNR << RSP_OPF_V_FLG)` == 0x0101.  Computed rather than
           written down -- the first version of this check looked for `rsp=0001` and reported the
           datagram as missing while both engines were producing it, which is a floor that fails on
           correct code and would have been "fixed" by weakening the case. */
        let dgOpc = "rsp=" + hex((FM_BAD << 0) | (LF_SNR << 8), 4) + ", sts=";
        let dg = sim[hi].trace.filter((e) => e.line.startsWith(dgOpc)).length;
        acc.hbePackets = dg;
        if (dg === 0) {
            failures.push(`coverage: the CF_THS case produced no FM_BAD (rsp=0001) error-log ` +
                `datagram on the oracle, so rq_hbe()'s packet-building half is unexercised`);
        }
        if (sim[ni].trace.some((e) => e.line.startsWith(dgOpc))) {
            failures.push(`coverage: the case WITHOUT CF_THS produced an error-log datagram on the ` +
                `oracle, so the two NXM cases are not measuring the difference they claim to`);
        }
    }

    /* ---- REAL FILE DATA MUST HAVE REACHED MEMORY.  A page whose every longword is still the seed
       is a page nothing was written to, and a controller that filled the destination with zeros
       would satisfy every "the two engines agree" comparison in this file if BOTH did it.  The
       oracle is asked directly: at least one dumped page of at least one graded READ must carry a
       longword that is neither its seed nor zero. ---- */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let p of cases[i].dumpPages) {
            let seed = seedFor(p);
            for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) {
                let v = sim[i].mem.get(a >>> 0);
                if (v !== undefined && v !== seed && v !== 0) { acc.realDataPages.add(p); break; }
            }
        }
    }
    if (acc.realDataPages.size < 3) {
        failures.push(`coverage: only ${acc.realDataPages.size} physical page(s) came back from the ` +
            `oracle carrying something that is neither the page seed nor zero; the floor is 3.  A ` +
            `controller that delivered nothing, or delivered zeros, would satisfy every ` +
            `engine-to-engine comparison in this file`);
    }

    /* ---- THE >2^31 OFFSET FLOOR: a graded read whose block offset does not fit in a signed 32-bit
       integer.  Without one, `(lbn * 512) | 0` is indistinguishable from the correct arithmetic. ---- */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let o of diskOps(sim[i].trace)) {
            if (o.lbn * RQ_NUMBY >= 0x80000000) acc.bigOffsets++;
        }
    }
    if (acc.bigOffsets === 0) {
        failures.push(`coverage: no graded disk operation reached a byte offset at or past 2^31 on ` +
            `the oracle, so a 32-bit block-offset computation is indistinguishable from a correct one`);
    }

    /* ---- THE RQ_MAPXFER FLOOR: the MSCP-level map must have been used, and an INVALID entry in it
       must have produced a residual -- the two halves of rq_map_ba(). ---- */
    if (byKind("mapxfer") < 0 || byKind("mapinv") < 0) {
        failures.push(`coverage: the RQ_MAPXFER path did not reach comparison in both its valid and ` +
            `its invalid-entry form, so rq_map_ba() is ungraded`);
    }

    /* ---- PHASE I: the write path.  At least two containers compared, at least one of them CHANGED
       from its pristine copy -- a comparison of two files neither engine wrote is not a pass. ---- */
    if (acc.imagesCompared < 2) {
        failures.push(`coverage: PHASE I compared ${acc.imagesCompared} container(s); the floor is 2`);
    }
    if (acc.imagesMatched !== acc.imagesCompared) {
        /* gradeImages() already reported the difference; this is the coverage statement. */
    }

    /* ---- Each named behaviour must have been OBSERVED on the oracle at least once. ---- */
    for (let [k, what] of [
        ["identity",  "a single-page IDENTITY-MAPPED transfer (the case the map cheat survives)"],
        ["single",    "the same transfer through a scattered map"],
        ["cross",     "a read crossing one map-entry boundary"],
        ["discontig", "a read spanning three or more discontiguous out-of-order map entries"],
        ["chunk",     "a transfer larger than RQ_MAXFR"],
        ["partial",   "a byte count that is not a multiple of 512"],
        ["ends",      "LBN 0, maxlbn-1, the RCT window and past it"],
        ["ladder",    "rq_rw_valid()'s refusals"],
        ["notonline", "a transfer to a unit that is attached and not online"],
        ["cmp",       "ACCESS, a COMPARE that matches and a COMPARE that does not"],
        ["nxm",       "a host bus error part way through the DMA"],
        ["hbe",       "the same error with CF_THS set"],
        ["mapxfer",   "the RQ_MAPXFER second-level map"],
        ["mapinv",    "an INVALID MSCP map entry"],
        ["unitq",     "a second transfer DEFERRED onto a unit queue"],
        ["wprot",     "the write-protect ladder"],
        ["shortfile", "a read straddling the end of a container inside a larger unit"],
        ["bigoffset", "a container whose block offsets pass 2^31"],
        ["write",     "WRITE, a partial-block WRITE and ERASE"],
        ["random",    "the randomized phase"]
    ]) {
        if (byKind(k) < 0) failures.push(`coverage: no graded case exercised ${what}`);
    }

    /* ---- PHASE R, and its absence is REPORTED rather than silent. ---- */
    if (opts.volume) {
        let vi = byKind("vol-home");
        if (vi < 0) {
            failures.push(`coverage: the real volume's home-block case did not reach comparison`);
        } else {
            let c = cases[vi];
            let bytes = [];
            for (let p of c.dumpPages) {
                for (let a = p * PAGE; a < p * PAGE + PAGE; a += 4) {
                    let v = sim[vi].mem.get(a >>> 0);
                    if (v === undefined) continue;
                    bytes.push(v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF);
                }
            }
            acc.ods2 = String.fromCharCode(...bytes).indexOf(ODS2_SIG) >= 0;
            if (!acc.ods2) {
                failures.push(`coverage: the bytes the ORACLE delivered for LBN 1 of ${opts.volume} ` +
                    `do not contain "${ODS2_SIG}".  Either the transfer did not deliver the home ` +
                    `block or that container is not the ODS-2 volume this phase is written for -- ` +
                    `and in both cases PHASE R proves nothing about a real volume`);
            }
        }
        if (byKind("vol-end") < 0) {
            failures.push(`coverage: the real volume's end-of-container case did not reach comparison, ` +
                `so the FILE-SHORTER-THAN-THE-VOLUME edge is ungraded on real data`);
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE V -- the PREMISES, re-measured                                                          *
 *                                                                                               *
 * This item was handed four measured facts.  Three held.  ONE DID NOT, and it is the kind that   *
 * HANDOFF.md section 7 exists for: the conclusion was right and the reason was wrong.            *
 * ------------------------------------------------------------------------------------------- */

/** `#define NAME value` out of a C source, for the handful of forms these constants use.  Refuses
    anything it cannot evaluate rather than guessing -- interpreting a define wrongly is worse than
    declining to (tests/mscpscope.js takes the same line and for the same reason). */
function cdefine(text, name)
{
    let m = new RegExp(`^#define\\s+${name}\\s+(.+?)\\s*(?://|/\\*|$)`, "m").exec(text);
    if (!m) return null;
    let v = m[1].trim();
    let sh = /^\(\s*1u?\s*<<\s*(\d+)\s*\)$/.exec(v);
    if (sh) return (Math.pow(2, +sh[1])) >>> 0;
    if (/^0x[0-9A-Fa-f]+$/.test(v)) return parseInt(v, 16) >>> 0;
    if (/^\d+$/.test(v)) return parseInt(v, 10) >>> 0;
    return {unparsed: v};
}

function phaseV(simh, scope, failures, report)
{
    let src = fs.readFileSync(path.join(scope.dir, "pdp11_rq.c"), "utf8").replace(/\r/g, "");

    /* ---- The constants, DERIVED rather than transcribed (standing rule 5). ---- */
    for (let [name, shipped] of [["RQ_MAXFR", RQ_MAXFR], ["RQ_MAPXFER", RQ_MAPXFER],
                                 ["RQ_M_PFN", RQ_M_PFN], ["RQ_NUMBY", RQ_NUMBY]]) {
        let derived = cdefine(src, name);
        if (derived === null || typeof derived === "object") {
            failures.push(`PHASE V: pdp11_rq.c's ${name} is ${derived === null ? "absent" :
                JSON.stringify(derived.unparsed)}, which this extraction cannot evaluate -- so the ` +
                `constant rq.js chunks and maps with is UNCHECKED against the C`);
        } else if ((derived >>> 0) !== (shipped >>> 0)) {
            failures.push(`PHASE V: ${name} is 0x${hex(derived)} in pdp11_rq.c and 0x${hex(shipped)} ` +
                `in rq.js`);
        }
    }

    /* ---- ONE PAGE MASK, TWO MODULES.  rq_readw()'s 512-byte split and qba_map_addr()'s page
       offset are the SAME VA_M_OFF in the C (both come from vax_defs.h), and this tree has it in
       two files because the C has it in two translation units.  Asserted rather than shared, so
       they cannot drift without failing a run. ---- */
    if (VA_M_OFF !== CQ_VA_M_OFF) {
        failures.push(`PHASE V: rq.js's VA_M_OFF is 0x${hex(VA_M_OFF)} and cqbic.js's is ` +
            `0x${hex(CQ_VA_M_OFF)}.  They are the SAME vax_defs.h constant and rq_readw()'s page ` +
            `split has to agree with the map's page offset or a mapped transfer splits where the ` +
            `map does not.`);
    }
    /* *** THE MASK THAT COST A MACHINE CHECK. ***  vax_defs.h says `VA_N_VPN (31 - VA_N_OFF)`, so
       the VPN is 22 bits and RQ_MAPXFER's own bit 31 is masked OUT of rq_map_ba()'s index.  The
       reading a careful person arrives at -- 32 address bits minus a 9-bit offset, 23 bits -- puts
       every mapped transfer's map read 0x1000000 bytes too high, which on a 16MB machine is
       register space: ReadReg(), CBTCR = C0000000, MCHK code 0x80, taken from INSIDE the DMA, with
       the unit left holding its packet and no response ever sent.  Derived from the header here so
       the two readings can never be confused again. */
    let defs = fs.readFileSync(path.join(scope.dir, "..", "VAX", "vax_defs.h"), "utf8").replace(/\r/g, "");
    let mv = /^#define\s+VA_N_VPN\s+\(31\s*-\s*VA_N_OFF\)/m.test(defs);
    if (!mv) {
        failures.push(`PHASE V: vax_defs.h no longer defines VA_N_VPN as (31 - VA_N_OFF).  rq.js's ` +
            `VA_M_VPN is 0x${hex(VA_M_VPN)} (${VA_N_VPN} bits) and rq_map_ba()'s whole index -- and ` +
            `therefore where a mapped transfer reads its map table -- depends on it`);
    }
    if (VA_N_VPN !== 31 - VA_N_OFF || VA_M_VPN !== (Math.pow(2, 31 - VA_N_OFF) - 1)) {
        failures.push(`PHASE V: rq.js's VA_M_VPN is 0x${hex(VA_M_VPN)} and vax_defs.h's arithmetic ` +
            `gives 0x${hex(Math.pow(2, 31 - VA_N_OFF) - 1)}`);
    }
    if ((RQ_MAPXFER >>> 0) !== 0x80000000 || PTE_V !== RQ_MAPXFER) {
        failures.push(`PHASE V: RQ_MAPXFER (0x${hex(RQ_MAPXFER)}) and PTE_V (0x${hex(PTE_V)}) are ` +
            `both bit 31 in the C and rq.js disagrees`);
    }

    /* ---- *** THE PREMISE THAT DID NOT HOLD. ***  This item was told "the build is NOASYNCH=1, so
       sim_disk_rdsect_a() completes INLINE", and instructed to confirm it rather than assume it.
       CONFIRMED FALSE, IN THE STATED FORM: `NOASYNCH` IS NOT A VARIABLE THE OPEN SIMH MAKEFILE HAS
       -- `make NOASYNCH=1 ... vax` sets an unused variable -- and the binary this gate runs against
       reports "Asynchronous I/O support" in `SHOW VERSION`.  sim_disk.c's AIO_CALL is therefore the
       THREADED macro, not the one-line inline one.
       THE CONCLUSION SURVIVES, BY A DIFFERENT MECHANISM, AND IT IS DERIVED HERE RATHER THAN
       ASSUMED: that macro's body is `if (ctx->asynch_io) { ...post to the I/O thread... } else if
       (_callback) (_callback) (uptr, r);`, and `ctx->asynch_io` is assigned in EXACTLY ONE PLACE --
       sim_disk_set_async() -- which pdp11_rq.c never calls.  The disk context is calloc'd, so it
       stays 0 and the callback runs inline.  Both halves are checked below, on the source and on
       the binary, so a vendor update that made RQ asynchronous fails this phase instead of turning
       every in-band wait count in this file into noise. ---- */
    let setAsync = (src.match(/sim_disk_set_async/g) || []).length;
    if (setAsync !== 0) {
        failures.push(`PHASE V: pdp11_rq.c now calls sim_disk_set_async() (${setAsync} time(s)), so ` +
            `ctx->asynch_io can become non-zero and sim_disk_rdsect_a() would post the transfer to ` +
            `an I/O THREAD instead of running the callback inline.  Every in-band wait count this ` +
            `file grades assumes the inline path; none of them mean anything until that is resolved.`);
    }
    let disk = fs.readFileSync(path.join(scope.dir, "..", "sim_disk.c"), "utf8").replace(/\r/g, "");
    let assigns = (disk.match(/ctx->asynch_io\s*=\s*/g) || []).length;
    if (assigns !== 2) {
        failures.push(`PHASE V: sim_disk.c assigns ctx->asynch_io in ${assigns} place(s); this ` +
            `phase's argument that it stays zero for an RQ unit rests on there being exactly two ` +
            `(sim_disk_set_async and sim_disk_clr_async), neither reachable from pdp11_rq.c`);
    }

    let ver = "";
    try {
        ver = runSimh(simh, "show version\nexit\n",
                      path.join(os.tmpdir(), "mscprw-ver.ini"), 60000);
    } catch (e) { ver = ""; }
    let asyncBuilt = /Asynchronous I\/O support/.test(ver);
    report.push(`  PHASE V  RQ_MAXFR/RQ_MAPXFER/RQ_M_PFN/RQ_NUMBY re-derived from pdp11_rq.c; ` +
        `VA_M_OFF agrees across rq.js and cqbic.js;\n` +
        `           the oracle IS built with asynchronous I/O (${asyncBuilt ? "SHOW VERSION says so" :
        "SHOW VERSION does NOT say so"}) -- the item's "NOASYNCH=1" premise is WRONG, `+
        `\n           but pdp11_rq.c never calls sim_disk_set_async(), so ctx->asynch_io stays 0 ` +
        `and the callback runs INLINE anyway`);
    return {asyncBuilt};
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11)          *
 *                                                                                                *
 * *** HANDOFF.md STANDING RULE 16: A MUTATION CAN BE CAUGHT BY A CHECK THAT IS ITSELF BROKEN. *** *
 * Every read this file makes was chosen against that.  The destination pages are dumped WHOLE and *
 * compared longword by longword at PHYSICAL addresses, so no comparison here splices two Qbus     *
 * pages; the packet array comes from the simulator's own rq_reg[]; the disk operations come from  *
 * the oracle's DEBUG=REQ text; the written containers are compared as FILES.  A mutation below    *
 * that is caught ONLY by one of those and by nothing else is called out where it happens.         *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* --- THE INHERITED CHEAT, RE-ASSERTED.  Composed over the original so every error latch the
       real translation sets still fires and ONLY the translated address is perturbed -- rule 11.
       `expect` is checked by selfcheck(): it must SURVIVE the identity case and DIE on the
       discontiguous one, and the run reports WHICH case caught it. --- */
    "map-lookup-bypassed-bus-address-used-as-physical": {
        expect: {mustCatch: "discontig", mustNotName: "identity"},
        apply: () => {
            let orig = CQBICVAX.prototype.mapAddr;
            CQBICVAX.prototype.mapAddr = function(qa) {
                let ok = orig.call(this, qa);
                if (ok) this.mapMA = qa >>> 0;
                return ok;
            };
            return () => { CQBICVAX.prototype.mapAddr = orig; };
        }
    },

    /* --- THE FOUR OTHERS THE ITEM NAMES --- */
    "RQ_MAXFR-chunking-removed": {
        apply: () => {
            let orig = RQVAX.prototype.trimChunk;
            RQVAX.prototype.trimChunk = function(bc) { return bc; };
            return () => { RQVAX.prototype.trimChunk = orig; };
        }
    },
    "the-residual-returned-by-mapWriteW-is-ignored": {
        apply: () => {
            let orig = RQVAX.prototype.writeW;
            RQVAX.prototype.writeW = function(ba, bc, ma, buf) { orig.call(this, ba, bc, ma, buf); return 0; };
            return () => { RQVAX.prototype.writeW = orig; };
        }
    },
    "the-block-number-is-not-advanced-between-chunks": {
        /* blocksFor() has TWO callers and they must move together: the number of blocks a read
           issues and the amount the block number advances.  Perturbing it in the chunk-advance role
           alone is not expressible without substituting rq_svc(), so this perturbs the seam and the
           chunking case is where it lands. */
        apply: () => {
            let orig = RQVAX.prototype.blocksFor;
            let depth = 0;
            RQVAX.prototype.blocksFor = function(bc) {
                let n = orig.call(this, bc);
                return (depth++ % 2 === 1) ? n - 1 : n;
            };
            return () => { RQVAX.prototype.blocksFor = orig; };
        }
    },
    "the-512-byte-page-split-uses-the-WRONG-MASK": {
        apply: () => {
            let orig = RQVAX.prototype.pageSplit;
            RQVAX.prototype.pageSplit = function(ba) { return 0x200 - (ba & 0xFF); };
            return () => { RQVAX.prototype.pageSplit = orig; };
        }
    },
    "the-image-read-offset-is-computed-in-32-bits": {
        expect: {mustCatch: "bigoffset"},
        apply: () => {
            let orig = RQVAX.prototype.imageOffset;
            RQVAX.prototype.imageOffset = function(lbn) { return orig.call(this, lbn) | 0; };
            return () => { RQVAX.prototype.imageOffset = orig; };
        }
    },

    /* --- THE TAIL OF THE LAST BLOCK.  A READ issues ceil(bc/512) blocks and stores exactly bc
       bytes; storing the rounded-up count instead delivers up to 510 bytes of REAL DISK DATA the
       host never asked for.  Every byte of it is right, which is why only a WHOLE-PAGE dump sees
       it -- the intended extent compares equal. --- */
    "the-tail-of-the-last-block-reaches-memory": {
        apply: () => {
            let orig = RQVAX.prototype.writeW;
            RQVAX.prototype.writeW = function(ba, bc, ma, buf) {
                return orig.call(this, ba, (bc + RQ_NUMBY - 1) & ~(RQ_NUMBY - 1), ma, buf);
            };
            return () => { RQVAX.prototype.writeW = orig; };
        }
    },

    /* --- THE WORKING FIELDS.  rq_rw() copies the host's into them and rq_rw_end() reports
       `original - remaining` and zeroes them.  Each half is separately observable in the packet
       array, and NEITHER is observable in the bytes that arrived. --- */
    "rq_rw_end-does-not-clear-the-eight-working-words": {
        apply: () => {
            let orig = RQVAX.prototype.rwEnd;
            RQVAX.prototype.rwEnd = function(u, flg, sts) {
                let pkt = u.cpkt;
                let saved = [RW_WBAL, RW_WBAL + 1, RW_WBCL, RW_WBCL + 1,
                             RW_WBLL, RW_WBLL + 1, RW_WMPL, RW_WMPL + 1].map((w) => this.pd(pkt, w));
                let r = orig.call(this, u, flg, sts);
                [RW_WBAL, RW_WBAL + 1, RW_WBCL, RW_WBCL + 1, RW_WBLL, RW_WBLL + 1,
                 RW_WMPL, RW_WMPL + 1].forEach((w, k) => this.spd(pkt, w, saved[k]));
                return r;
            };
            return () => { RQVAX.prototype.rwEnd = orig; };
        }
    },
    "the-end-packet-reports-the-REQUESTED-byte-count-not-the-bytes-processed": {
        apply: () => {
            let orig = RQVAX.prototype.rwEnd;
            RQVAX.prototype.rwEnd = function(u, flg, sts) {
                let pkt = u.cpkt;
                let bc = this.getp32(pkt, RW_BCL);
                let r = orig.call(this, u, flg, sts);
                this.putp32(pkt, RW_BCL, bc);
                return r;
            };
            return () => { RQVAX.prototype.rwEnd = orig; };
        }
    },

    /* --- THE SCHEDULE.  Completing the disk I/O immediately instead of at `iostarttime + xtime`
       is the synchronous-transfer cheat in its purest form: every byte still arrives, in the right
       place, and every in-band wait count goes to its floor. --- */
    "the-disk-completion-does-not-wait-out-xtime": {
        apply: () => {
            let orig = RQVAX.prototype.ioDone;
            RQVAX.prototype.ioDone = function(u, status) {
                let saved = this.xtime;
                this.xtime = 0;
                try { return orig.call(this, u, status); } finally { this.xtime = saved; }
            };
            return () => { RQVAX.prototype.ioDone = orig; };
        }
    },
    /* *** NOT A MUTATION, AND SAID HERE RATHER THAN LEFT OUT SILENTLY (standing rule 6). ***
       `iostarttime` is set ONCE, in rq_rw(), so from the second chunk onward `iostarttime + xtime`
       is already past and sim_activate_notbefore() runs the completion immediately.  Refreshing it
       per chunk is the obvious misreading and it IS WRONG -- but it is observable ONLY in the
       in-band iteration count of a multi-chunk transfer, and that count is NOT REPRODUCIBLE on the
       oracle (see assertSchedule()).  A mutation for it would have to be graded against this
       tree's own model, which grades nothing.  It is left ungraded and named. */

    /* --- rq_rw_valid()'s LADDER.  Each of these is one rung, and every one of them still produces
       a plausible answer. --- */
    "the-odd-address-test-is-applied-to-OP_ACC-as-well": {
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                if (sts === 0 && cmd === OP.ACC && (this.pd(pkt, RW_BAL) & 1)) {
                    return ST.HST | SB_HST_OA;
                }
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },
    "an-LBN-in-the-RCT-window-is-refused-outright": {
        apply: () => {
            let orig = RQVAX.prototype.rwValid;
            RQVAX.prototype.rwValid = function(pkt, u, cmd) {
                let sts = orig.call(this, pkt, u, cmd);
                if (sts === 0 && this.getp32(pkt, RW_LBNL) >= (u.capac >>> 0)) return ST.CMD | I_LBN;
                return sts;
            };
            return () => { RQVAX.prototype.rwValid = orig; };
        }
    },

    /* --- THE SECOND MAP.  Two ways to get rq_map_ba() wrong that both produce a working transfer
       for SOME address: ignoring the valid bit, and taking the VPN without bit 31. --- */
    "rq_map_ba-ignores-PTE_V": {
        apply: () => {
            let orig = RQVAX.prototype.mapBa;
            RQVAX.prototype.mapBa = function(ba, ma) {
                let r = orig.call(this, ba, ma);
                if (r) return r;
                let rg = this.readLongPhys((ma + (((ba >>> 9) & 0x7FFFFF) << 2)) >>> 0);
                return ((((rg & RQ_M_PFN) << 9) >>> 0) | (ba & VA_M_OFF)) >>> 0;
            };
            return () => { RQVAX.prototype.mapBa = orig; };
        }
    },
    /* *** THE DEFECT THIS FILE ACTUALLY SHIPPED, TURNED INTO A MUTATION. ***  VA_M_VPN is
       `(1u << (31 - VA_N_OFF)) - 1` -- TWENTY-TWO bits.  Reading it as 23 (a 32-bit address minus a
       9-bit offset, which is what a careful person derives) leaves RQ_MAPXFER's own bit 31 in the
       index and puts every map read 0x1000000 bytes higher.  The first version of this item did
       exactly that and the oracle answered with a machine check from inside the DMA.
       A mutation that MASKED bit 31 out instead was written first and SURVIVED -- correctly, and
       usefully: with a 22-bit mask the bit is already gone, so removing it is a literal no-op and
       the suite would have printed CAUGHT for nothing (HANDOFF.md standing rule 11).  This is the
       perturbation that is not a no-op. */
    "rq_map_ba-uses-a-23-bit-VPN-so-bit-31-survives-into-the-index": {
        expect: {mustCatch: "mapxfer"},
        apply: () => {
            let orig = RQVAX.prototype.mapBa;
            RQVAX.prototype.mapBa = function(ba, ma) {
                let idx23 = (((ba >>> 9) & 0x7FFFFF) << 2);
                let idx22 = (((ba >>> 9) & 0x3FFFFF) << 2);
                return orig.call(this, ba, (ma + (idx23 - idx22)) >>> 0);
            };
            return () => { RQVAX.prototype.mapBa = orig; };
        }
    },

    /* --- THE ERROR LOG.  EF_LOG is set on the END packet whether or not a log packet was built,
       because rq_hbe()'s "logging disabled" arm returns OK.  Suppressing it when logging is off is
       the reading a careful person arrives at and it is wrong. --- */
    "EF_LOG-is-suppressed-when-error-logging-is-off": {
        apply: () => {
            let orig = RQVAX.prototype.hbe;
            RQVAX.prototype.hbe = function(u) {
                let r = orig.call(this, u);
                return (this.cflgs & CF_THS) ? r : false;
            };
            return () => { RQVAX.prototype.hbe = orig; };
        }
    },

    /* --- THE UNIT QUEUE, which this item made reachable.  Dropping the deferral answers the second
       command out of the first one's transfer, which produces two plausible responses in the wrong
       order and a ring index that is right. --- */
    "a-command-to-a-BUSY-unit-is-not-deferred": {
        apply: () => {
            let orig = RQVAX.prototype.rw;
            RQVAX.prototype.rw = function(pkt, q) { return orig.call(this, pkt, false); };
            return () => { RQVAX.prototype.rw = orig; };
        }
    },

    /* --- THE COMPARE nxm PATH's VENDOR TYPO.  `PUTP32 (pkt, RW_WBAL, bc - i)` puts a BYTE COUNT
       in the working BUS ADDRESS.  "Fixing" it to `ba + i` is the mutation, because the fix is what
       a careful reader would do and it makes this tree disagree with the simulator. --- */
    "the-COMPARE-nxm-path-is-corrected-to-report-ba+i": {
        expect: {mustCatch: "hbe"},
        apply: () => {
            let orig = RQVAX.prototype.cmpNxmAddr;
            RQVAX.prototype.cmpNxmAddr = function(bc, i, ba) { return (ba + i) >>> 0; };
            return () => { RQVAX.prototype.cmpNxmAddr = orig; };
        }
    },

    /* --- THE WRITE PATH, which NO memory comparison in this file can see.  Both of these are
       caught only by PHASE I -- the two engines' containers compared as files -- and they are here
       to prove that phase is load bearing. --- */
    "a-partial-block-WRITE-does-not-zero-the-tail-of-the-block": {
        apply: () => {
            let orig = RQVAX.prototype.diskWrite;
            RQVAX.prototype.diskWrite = function(u, lbn, nsect) {
                let save = u.rqxb.slice(0, nsect * RQ_NUMBY);
                for (let i = 0; i < nsect * RQ_NUMBY; i++) if (u.rqxb[i] === 0) u.rqxb[i] = 0xEE;
                try { return orig.call(this, u, lbn, nsect); } finally { u.rqxb.set(save, 0); }
            };
            return () => { RQVAX.prototype.diskWrite = orig; };
        }
    },
    "a-WRITE-goes-to-the-block-BEFORE-the-one-the-host-named": {
        apply: () => {
            let orig = RQVAX.prototype.diskWrite;
            RQVAX.prototype.diskWrite = function(u, lbn, nsect) {
                return orig.call(this, u, lbn > 0 ? lbn - 1 : lbn, nsect);
            };
            return () => { RQVAX.prototype.diskWrite = orig; };
        }
    },

    /* --- THE DISK TRACE.  It is a DEBUG stream, so it changes no bytes anywhere -- and it is the
       only view that grades the chunking as a sequence.  Dropping the LBN from it proves the trace
       comparison is doing work rather than agreeing with itself. --- */
    "the-disk-trace-reports-the-transfer-LBN-instead-of-the-chunk-LBN": {
        apply: () => {
            let orig = RQVAX.prototype.diskTrace;
            RQVAX.prototype.diskTrace = function(u, txt, lbn, len) {
                return orig.call(this, u, txt, this.getp32(u.cpkt, RW_LBNL), len);
            };
            return () => { RQVAX.prototype.diskTrace = orig; };
        }
    }

    /* *** NO MUTATION FOR sim_disk_rdsect()'s SINGLE-SECTOR SHORTCUT, AND THE REASON IS THAT IT IS
       UNOBSERVABLE HERE (standing rule 6 -- name it, do not omit it). ***  A ONE-SECTOR read at or
       past the unit's capacity is answered from a ZEROED buffer without touching the container; a
       multi-sector read at the same LBN goes to pread(2), which returns ZEROS past the end of the
       file.  The two therefore differ only for a unit whose CAPACITY is smaller than its
       container's block count -- and autosize cannot produce one: with an unrecognised file system
       `capac` is the container's own size or larger, and with a recognised one it is the file
       system's, which is larger still.  A mutation for it was written, SURVIVED, and was removed
       rather than have the case list bent to reach a state the arithmetic forbids. */
};

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                        *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

/** HANDOFF.md section 8: no OpenVMS media is shipped, downloaded or committed.  The volume is a
    PATH the user supplies; this is only where it happens to live on the machine this item was
    worked on, and its absence is reported BY NAME rather than passing quietly. */
const DEFAULT_VOLUME = "/home/baron/vax1/data/d0.dsk";

/** The oracle's answers do not depend on rq.js, so they are computed ONCE and reused by every
    mutation pass -- keyed on the do-file's CONTENT, so a mutation that changed the case list would
    re-run the simulator and the key is what says so rather than a comment. */
const ORACLE_CACHE = new Map();

function cached(key, fn) {
    if (!ORACLE_CACHE.has(key)) ORACLE_CACHE.set(key, fn());
    return ORACLE_CACHE.get(key);
}

/**
 * openProviders(opts, providers)
 *
 * Opens every container ONCE, for the whole run.  Called from main() and nowhere else.
 */
function openProviders(opts, providers)
{
    for (let im of opts.images) {
        if (im.writable) fs.copyFileSync(im.pristine, im.path);
        /* *** EVERY SCRATCH CONTAINER IS OPENED READ/WRITE, INCLUDING THE ONES NO CASE WRITES. ***
           The provider's writability is not a policy here -- it is a MEASUREMENT the controller
           makes: attach() forces a unit read-only when the container cannot be written, exactly as
           sim_disk_attach_ex2() does, and a read-only unit skips autosize's clamp and comes out a
           DIFFERENT SIZE.  Opening a container read-only "for safety" therefore silently changes
           the unit the guest sees.  What keeps the read-only containers untouched is that no case
           writes to them and checkImagesUntouched() checksums them, not the open mode.  The REAL
           VOLUME is the one exception and it is opened read-only on purpose. */
        providers[im.tag] = fileImageProvider(im.path);
    }
    if (opts.volume) providers["volume"] = fileImageProvider(opts.volume, {readOnly: true});
    return providers;
}

/**
 * restoreWritable(opts)
 *
 * *** THE WRITABLE CONTAINERS ARE PUT BACK BEFORE EVERY JS PASS. ***  The oracle runs ONCE and its
 * copy is written once; this engine runs again for every --selfcheck mutation, and a pass that
 * started from the previous pass's output would compare two files neither of which is what this
 * pass wrote.
 *
 * *** IT NO LONGER CLOSES AND RE-OPENS ANYTHING, AND THAT IS THE FIX FOR A REAL FLAKE. ***  The
 * first version closed every provider, copyFileSync'd, and re-opened -- twenty-one times in a
 * --selfcheck run, across five containers, one of them 2.5 GiB.  That produced an intermittent
 * `ENOENT ... mscprw-small.dsk` from inside a later pass, at roughly one run in five, measured on
 * a quiet machine.  Descriptors are now opened once by openProviders() and closed once by main()'s
 * finally; the restore is pwrite + ftruncate through the descriptor that is already open, so there
 * is no window in which a container's path has to resolve again.
 */
function restoreWritable(opts, providers)
{
    for (let im of opts.images) {
        if (!im.writable) continue;
        providers[im.tag].restoreFrom(im.pristine);
    }
}

/**
 * assertContainers(opts, failures)
 *
 * Every container this run needs, checked to EXIST WITH ITS EXPECTED SIZE at the start of every
 * pass -- and, when one does not, reported BY NAME together with what the scratch directory
 * actually holds (HANDOFF.md standing rule 6).  An opaque ENOENT stack trace out of a provider is
 * the same event with the diagnosis removed, and it cost a merge.
 */
function assertContainers(opts, failures)
{
    let dirOk = fs.existsSync(opts.scratch);
    let listing = dirOk ? fs.readdirSync(opts.scratch).join(", ") : "(the scratch directory itself is GONE)";
    for (let im of opts.images) {
        for (let [what, p] of [["pristine", im.pristine], ["this engine's", im.path],
                               ["the oracle's", im.simhPath]]) {
            if (!fs.existsSync(p)) {
                failures.push(`container "${im.tag}" -- ${what} copy -- is MISSING at ${p}.  ` +
                    `Scratch is ${opts.scratch}; it holds: ${listing}.  Nothing in this ` +
                    `differential removes a container before main()'s finally, so either a cleanup ` +
                    `path fired early or something outside this process took it.`);
                return false;
            }
            let sz = fs.statSync(p).size;
            if (!im.writable && sz !== im.bytes) {
                failures.push(`container "${im.tag}" -- ${what} copy at ${p} -- is ${sz} bytes and ` +
                    `was created with ${im.bytes}.  A read-only container changed size, which ` +
                    `changes the unit's capacity and makes every later case order-dependent.`);
                return false;
            }
        }
    }
    if (opts.volume && !fs.existsSync(opts.volume)) {
        failures.push(`the real volume ${opts.volume} disappeared during the run`);
        return false;
    }
    return true;
}

function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [];
    let acc = {waitCounts: new Set(), nonZeroWaits: 0, zeroWaits: 0,
               rspOpc: new Set(), rspSts: new Set(), cmdLines: 0, xferOps: new Set(),
               partialReads: 0, wholeBlockWrites: 0, bigOffsets: 0, chunkOps: 0,
               discontigPages: 0, hbePackets: 0, realDataPages: new Set(), xferWaits: 0, diskWaits: 0,
               imagesCompared: 0, imagesMatched: 0, ods2: false};

    /* ---- PHASE S ---- */
    for (let f of opts.scope.failures) failures.push(f);
    let xfer = Object.keys(RQVAX.MSCP_OP_HANDLER)
                     .filter((n) => opts.scope.dispatch[n] === "rq_rw").sort();
    let shipped = RQVAX.MSCP_XFER_OPS.slice().sort();
    if (xfer.join(",") !== shipped.join(",")) {
        failures.push(`PHASE S: rq_mscp() dispatches {${xfer.join(",")}} to rq_rw() and rq.js's ` +
            `MSCP_XFER_OPS is {${shipped.join(",")}} -- the set of commands this differential is ` +
            `required to exercise comes from the C's own switch and the two disagree`);
    }
    report.push(`  PHASE S  ${opts.scope.nOp} OP_ codes, ${opts.scope.nSt} ST_ codes, ` +
        `${opts.scope.nSwitch} rq_mscp() dispatch case(s) re-derived from ${opts.scope.dir}; ` +
        `${xfer.length} of them reach rq_rw() and every one is required below`);

    /* ---- PHASE V ---- */
    cached("phaseV", () => phaseV(simh, opts.scope, opts.vFailures = [], opts.vReport = []));
    for (let f of opts.vFailures) failures.push(f);
    for (let l of opts.vReport) report.push(l);

    /* ---- PHASE C + PHASE R ---- */
    let enumerated = enumeratedCases();
    let vol = opts.volume ? realVolumeCases(opts.volumeInfo) : [];
    let rnd = randomCases(opts.nRandom, opts.seed, enumerated.length + vol.length);
    for (let k = 0; k < vol.length; k++) vol[k].idx = enumerated.length + k;
    let all = enumerated.concat(vol, rnd);

    let sim = cached("cases:" + opts.nRandom + ":" + opts.seed + ":" + (opts.volume || "none"),
                     () => runCasesSimh(simh, opts, all));
    /* Each SIMH invocation starts a NEW simulator, so its static MSC struct starts at the C's global
       zero.  The JS machine is built once and reused (standing rule 14), so the pass boundary is
       where that has to be re-established. */
    machine(mutationOpts).rq.powerUp();
    if (!assertContainers(opts, failures)) return {failures, report, compared: 0, acc, cases: all, sim, js: []};
    restoreWritable(opts, opts.providers);
    let js = all.map((c) => runCaseJS(c, opts.providers, mutationOpts));

    assertExclusions(all, sim, js, failures);
    let compared = grade(all, sim, js, failures);
    assertSchedule(all, sim, js, failures, acc);
    gradeImages(opts.images, failures, acc);
    coverage(all, sim, js, failures, acc, opts);

    /* The wiring the graded machine is actually holding, asserted rather than assumed. */
    let m = machine(mutationOpts);
    if (m.cpu.qbus !== m.rq) failures.push(`the graded machine's CPU has no Qbus event hook wired to the controller`);
    if (!m.rq.cqbic || !m.rq.cqbic.bus) failures.push(`the graded machine's controller has no CQBIC with a bus`);

    report.push(`  PHASE C  ${compared}/${all.length} case(s) compared ` +
        `(${enumerated.length} enumerated + ${vol.length} real-volume + ${opts.nRandom} randomized)`);
    if (opts.volume) {
        report.push(`  PHASE R  ${opts.volume} (${opts.volumeInfo.bytes} bytes = ` +
            `${Math.floor(opts.volumeInfo.bytes / RQ_NUMBY)} blocks), attached READ ONLY; ` +
            `ODS-2 signature in the oracle's delivered home block: ${acc.ods2 ? "yes" : "NO"}`);
    } else {
        report.push(`  PHASE R  *** SKIPPED BY NAME: no real ODS-2 volume (--no-volume) ***`);
    }
    report.push(`  PHASE I  ${acc.imagesMatched}/${acc.imagesCompared} written container(s) ` +
        `byte-identical between the two engines`);
    report.push(`  TRACE    ${acc.cmdLines} command line(s), ${acc.rspSts.size} distinct status ` +
        `word(s), ${acc.chunkOps} disk operation(s) in the chunking case, ` +
        `${acc.partialReads} partial-block read(s), ${acc.bigOffsets} operation(s) past 2^31`);
    report.push(`  FLOORS   ${acc.discontigPages} discontiguous map entries in the floor case, ` +
        `${acc.waitCounts.size} distinct in-band wait count(s) (${acc.nonZeroWaits} non-zero, ` +
        `${acc.zeroWaits} immediate), ${acc.xferWaits} transfer wait(s) graded as a BAND ` +
        `(the oracle's is not reproducible; ${acc.diskWaits} of them covered a real disk delay), ` +
        `${acc.realDataPages.size} page(s) with real file data`);
    return {failures, report, compared, acc, cases: all, sim, js};
}

/**
 * selfcheck(simh, opts)
 *
 * *** THE ONE THING THIS REPORTS THAT A PLAIN CAUGHT/SURVIVED TABLE DOES NOT: WHICH CASE CAUGHT
 * EACH MUTATION. ***  tests/qdmadiff.js does the same, and for this item it is the whole argument
 * about the inherited cheat: `map-lookup-bypassed` must SURVIVE the identity-mapped case and DIE on
 * the discontiguous one, and a run that merely said CAUGHT would not distinguish that from a
 * mutation caught by the single-page case -- which would mean the identity case is not identity at
 * all and the floor is measuring nothing.
 */
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
        /* WHICH CASES the mutation was caught by, read out of the failure text rather than
           predicted: every per-case failure this file emits opens with `case <idx> "<name>"`. */
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
                misplaced.push(`${name}: WAS caught by the "${mut.expect.mustNotName}" case.  That ` +
                    `case exists to be SURVIVED -- if the mutation dies there, the case is not the ` +
                    `arrangement it claims to be and the floor below it is measuring nothing`);
            }
        }
        console.log(`  ${failures.length ? "CAUGHT " : "SURVIVED"}  ${name}` +
            (failures.length ? `\n              by: ${where}  (${failures.length} failure(s), first: ` +
                `${failures[0].split("\n")[0].slice(0, 150)})` : ""));
    }
    return {survived, misplaced, total: names.length};
}

/**
 * dumpCase(cases, sim, js, n)
 *
 * --dump N: the two engines' DEBUG=REQ streams for one case, side by side and WITH TIMESTAMPS.
 * A diagnostic, not a grading: the timestamps are not comparable (SIMH collapses repeated lines and
 * stamps the collapsed one with the LAST occurrence), but they are how a schedule disagreement gets
 * localised to an event instead of being read off a single in-band iteration count.
 */
function dumpCase(cases, sim, js, n)
{
    let i = cases.findIndex((c) => c.idx === n);
    if (i < 0) { console.log(`--dump ${n}: no such case`); return; }
    let c = cases[i];
    console.log(`\n--- case ${c.idx} "${c.name}" (kind=${c.kind}) ---`);
    console.log(`    comm=0x${c.comm.toString(16)} data=0x${c.g.dataBase.toString(16)}+` +
        `0x${c.g.dataLnt.toString(16)} mirror=0x${(c.g.mirrorBase || 0).toString(16)} ` +
        `pages=${c.entries.length} unmapped=[${[...(c.unmappedData || [])].join(",")}]`);
    console.log(`    code 0x${hex(R_CODE)}..0x${hex(c.haltPC)}; steps:`);
    for (let st of c.steps) {
        console.log(`      0x${hex(st.pc)}  ${st.s}${st.what ? " " + st.what : ""}` +
            `${st.slot !== undefined ? " slot=" + st.slot : ""}${st.pkt !== undefined ? " pkt=" + st.pkt : ""}`);
    }
    let sl = sim[i] ? sim[i].trace : [], jl = js[i] ? js[i].trace : [];
    for (let k = 0; k < Math.max(sl.length, jl.length); k++) {
        console.log(`  ${String(k).padStart(3)}  oracle t=${sl[k] ? sl[k].t : "-"} ${sl[k] ? sl[k].line : "(none)"}`);
        console.log(`       here   t=${jl[k] ? jl[k].t : "-"} ${jl[k] ? jl[k].line : "(none)"}`);
    }
    if (sim[i]) {
        console.log(`  oracle units: ` + sim[i].units.map((u, k) => `${k}:CPKT=${u.CPKT} PKTQ=${u.PKTQ} CAPAC=${u.CAPAC}`).join("  "));
        console.log(`  oracle RQ: ` + RQ_OBS.map((o) => `${o.name}=${hex(sim[i].rq[o.name], 4)}`).join(" "));
        console.log(`  here   RQ: ` + RQ_OBS.map((o) => `${o.name}=${hex(js[i].rq[o.name], 4)}`).join(" "));
    }
}

function main()
{
    let simh = findSimhBin(getArg("--simh", null));
    let dumpN = getArg("--dump", null);
    let rawN = getArg("--rawdump", null);
    let nRandom = +getArg("--cases", RANDOM_CASES_DEFAULT);
    let seed = +getArg("--seed", 20260727);
    let fSelfcheck = process.argv.includes("--selfcheck");
    let noVolume = process.argv.includes("--no-volume");
    let volume = getArg("--volume", process.env['PCJS_VAX_VOLUME'] || DEFAULT_VOLUME);

    if (nRandom < RANDOM_CASES_FLOOR) {
        console.error(`mscprwdiff: --cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let volumeInfo = null, volumeSkip = null;
    if (noVolume) {
        volumeSkip = `--no-volume was given explicitly`;
        volume = null;
    } else if (!fs.existsSync(volume)) {
        /* SKIP-OR-FAIL LOUDLY BY NAME.  HANDOFF.md 8: the volume is the USER's local file and is
           never shipped or committed, so a machine without one must be able to run this gate -- but
           it must SAY SO, and it must be told to.  A missing volume with no --no-volume is a
           FAILURE, because "no disk was found" and "the disk works" must never look alike. */
        console.error(`\nmscprwdiff: PHASE R needs a real OpenVMS ODS-2 container and\n` +
            `    ${volume}\n` +
            `  is not present.  Pass --volume PATH, set PCJS_VAX_VOLUME, or pass --no-volume to\n` +
            `  run the other five phases without it.  No image is shipped with this project ` +
            `(HANDOFF.md 8),\n  so this is a missing LOCAL FILE and not a defect -- but it is not a pass either.\n`);
        process.exit(1);
    }
    if (volume) {
        let st = fs.statSync(volume);
        volumeInfo = {path: volume, bytes: st.size};
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "mscprwdiff-"));
    let images = null, providers = {};
    let code = 0;
    let volBefore = null;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized cases: ${nRandom}`);

        let df = null;
        try { df = fs.statfsSync ? fs.statfsSync("/") : null; } catch (e) { df = null; }
        if (df) {
            let freeMB = (df.bavail * df.bsize) / (1024 * 1024);
            console.log(`free on /: ${freeMB.toFixed(0)} MB (the sparse container is 2.5 GiB ` +
                `APPARENT and a few KB real -- ftruncate, not zeros)`);
            if (freeMB < 512) {
                console.error(`mscprwdiff: only ${freeMB.toFixed(0)} MB free on / -- this project ` +
                    `has filled the root filesystem before (HANDOFF.md 4) and this run declines to try`);
                process.exit(1);
            }
        }

        images = makeImages(scratch);
        for (let im of images) IMG[im.tag] = im;
        console.log(`containers: ${images.map((i) => `${i.tag}=${i.blocks}blk${i.sparse ? " SPARSE" : ""}` +
            `${i.writable ? " (a copy per engine)" : ""}`).join(", ")}`);
        console.log(`  GENERATED here and deleted on every exit path -- HANDOFF.md 8: no image is shipped`);
        if (volume) {
            IMG["volume"] = {tag: "volume", path: volume, simhPath: volume, pristine: volume,
                             bytes: volumeInfo.bytes, blocks: Math.floor(volumeInfo.bytes / RQ_NUMBY),
                             writable: false, sparse: false, sha: null};
            /* The user's own file: size and the two megabytes at its ends, before and after.  A
               whole-file sha256 of 1.5GB on every run is 3GB of reads for a container this run
               attaches READ ONLY on both engines; the ends are where sim_disk would append a
               footer and where every graded transfer touches. */
            volBefore = volumeFingerprint(volume);
            console.log(`real volume: ${volume} (${volumeInfo.bytes} bytes = ` +
                `${Math.floor(volumeInfo.bytes / RQ_NUMBY)} blocks), attached READ ONLY`);
        }

        let opts = {scratch, nRandom, seed, images, providers, volume, volumeInfo, rawdump: rawN};
        opts.scope = checkScope(simh);
        openProviders(opts, providers);

        let pass = runPass(simh, opts);
        let {failures, report} = pass;
        if (dumpN !== null) {
            for (let n of String(dumpN).split(",")) {
                dumpCase(pass.cases, pass.sim, pass.js, +n);
            }
        }
        checkImagesUntouched(images, failures);
        if (volume) {
            let after = volumeFingerprint(volume);
            if (after !== volBefore) {
                failures.push(`THE USER'S VOLUME ${volume} CHANGED during the run (${volBefore} -> ` +
                    `${after}).  It is attached READ ONLY on both engines and nothing here may write ` +
                    `to it; something did.`);
            }
        }
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
            console.log(`\nMATCH -- every graded MSCP transfer moved the same bytes into the same ` +
                `physical pages as the oracle: every byte of every page the transfer could have ` +
                `touched, the END packet's opcode, flags, status and residual, the eight working ` +
                `words while in flight and zeroed after, ${PKT_PROBES.length} words of the packet ` +
                `array, all ${RQ_OBS.length} controller registers, the per-unit registers, ` +
                `SHOW RQ RINGS/FREEQ/RESPQ, the ORDERED DEBUG=REQ trace including one line per DISK ` +
                `OPERATION, and -- for WRITE and ERASE, which no memory comparison can see -- the ` +
                `two engines' containers compared as FILES.` +
                (volumeSkip ? `\n*** PHASE R WAS SKIPPED: ${volumeSkip}.  No real ODS-2 volume was ` +
                    `graded in this run. ***` : ``));
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
            if (!code) console.log(`\nall ${total} mutation(s) CAUGHT, and the map-lookup cheat was ` +
                `caught by the DISCONTIGUOUS case while surviving the IDENTITY-MAPPED one`);
        }
        if (!code) console.log("\nOK");
    } finally {
        /* EVERY EXIT PATH, INCLUDING A THROW.  The sparse container is 2.5 GiB apparent; a run that
           left its scratch behind would look like a filesystem that filled up. */
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

/** Size plus the first and last megabyte of the user's volume.  Enough to catch a footer append or
    a stray write near either end -- which is where every graded transfer goes -- without 3 GB of
    reads on every invocation. */
function volumeFingerprint(p)
{
    let st = fs.statSync(p);
    let fd = fs.openSync(p, "r");
    try {
        let h = crypto.createHash("sha256");
        h.update(String(st.size));
        let buf = Buffer.allocUnsafe(1 << 20);
        for (let off of [0, Math.max(0, st.size - (1 << 20))]) {
            let n = fs.readSync(fd, buf, 0, buf.length, off);
            h.update(buf.subarray(0, n));
        }
        return h.digest("hex");
    } finally { fs.closeSync(fd); }
}

main();
