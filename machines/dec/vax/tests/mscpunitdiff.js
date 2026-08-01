/**
 * @fileoverview Differential test: a user-supplied image attached to an RQDX3 unit -- geometry,
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
 *               media identifier and status graded against a real Open SIMH microvax3900
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-f52, the third of pcjsvax-6a5's children and the first that touches a real disk image.
 * With the controller UP (pcjsvax-c2c) and the rings working (pcjsvax-0b4), a user attaches a local
 * file to a drive and the guest asks the controller about it: GET UNIT STATUS, ONLINE, SET UNIT
 * CHARACTERISTICS, AVAILABLE, FORMAT, ABORT and GET COMMAND STATUS.  Every field of every response,
 * every examinable per-unit register, and the whole autosize arithmetic that turns a file size into
 * a unit capacity are compared with the oracle.
 *
 * *** NO IMAGE IS SHIPPED, DOWNLOADED OR COMMITTED. ***  HANDOFF.md 8: OpenVMS media is a licensing
 * question this project is designed around, and the disk layer takes a USER-SUPPLIED file.  This
 * differential therefore GENERATES its own scratch containers -- deterministic pseudo-random bytes,
 * four sizes, written into a mkdtemp directory -- attaches the SAME files to both engines, and
 * deletes them on every exit path.  It also CHECKSUMS them before and after: the oracle must hand
 * the user's file back unmodified, which is not free (sim_disk.c's store_disk_footer() would append
 * SIMH metadata to a container it autosized, and only an unconditional early `return SCPE_OK;` in
 * this vendor tree stops it).
 *
 * FOUR PHASES, EACH ANSWERING A DIFFERENT QUESTION
 * ------------------------------------------------
 *   PHASE S  the MSCP opcode / status / dispatch scope AND drv_tab[] itself, re-derived from
 *            pdp11_rq.c by tests/mscpscope.js on every run.  THIRTY-FOUR drive types of fourteen
 *            fields are not hand-transcribable (HANDOFF.md standing rule 5).
 *   PHASE T  `set rq0 <type>` for EVERY one of those 34 types on the live oracle, cross-checking
 *            the derived `lbn` against `examine rq capac[0]` and the derived NAME against what
 *            `show rq0` prints.  This is what makes PHASE S's extraction a measurement rather than
 *            a second transcription: an extractor that read the table's columns in the wrong order
 *            would agree with rq.js and disagree with the simulator.
 *   PHASE A  the AUTOSIZE arithmetic, swept over every drive type x every scratch image x three
 *            write-lock modes, with no CPU involved at all -- one `attach` and one
 *            `examine rq capac[0]` per point.  Hundreds of data points for the one computation that
 *            turns "the user's file" into "the unit's size".
 *   PHASE C  the MSCP commands themselves, driven through the rings by real VAX instructions
 *            exactly as pcjsvax-0b4 does, with responses read out of host memory, out of
 *            `examine rq pkts[N]`, out of the per-unit registers and out of the ordered
 *            `set rq debug=REQ` trace.
 *
 * A MEASUREMENT THAT OVERTURNED THIS ITEM'S OWN PREMISE -- READ THIS BEFORE "FIXING" A FLOOR
 * -------------------------------------------------------------------------------------------
 * The item was written expecting "at least THREE different drive geometries CHOSEN BY AUTOSIZE from
 * THREE different file sizes".  *** THAT IS NOT WHAT AUTOSIZE DOES TO A RAW CONTAINER. ***  Measured
 * on the live oracle: 409,600-byte, 20,000,000-byte and 159,334,400-byte files attached to RQ0 all
 * leave it `RD54 ... 159MB`, and a 200,000,300-byte file leaves it `RD54 ... 200MB`.  The drive-type
 * walk over drv_types[] lives in sim_disk_attach_ex2()'s `dontchangecapac` arm -- reached only with
 * NOAUTOSIZE set -- and it additionally requires get_filesystem_size() to have parsed an ODS-2 (or
 * RT-11, or Ultrix) volume out of the container, which a pattern-filled scratch image does not have.
 * With autosize ON, the arm taken is "autosize by changing CAPACITY": the drive TYPE is untouched
 * and only `capac` moves.
 *
 * So the floor is met, but by the mechanism the simulator actually has, and BOTH halves are floors:
 *   - at least THREE distinct drv_tab[] ENTRIES must reach comparison through real MSCP responses
 *     (they are selected by `set rqN <type>`, which is how a user picks a drive), and PHASE T
 *     cross-checks all thirty-four; and
 *   - at least THREE distinct unit CAPACITIES must be produced BY THREE DIFFERENT FILE SIZES on the
 *     ORACLE, including one from a file that is not a whole number of blocks and matches no table
 *     entry's size.
 * Neither scales with case count and both fail the run.
 *
 * THE CHEAT THIS FILE EXISTS TO KILL
 * -----------------------------------
 * Hard-coding RD54 because the scratch image happens to be RD54-shaped.  Every geometry field a
 * response carries -- the media identifier, the unit model, sectors/track, tracks/group,
 * groups/cylinder, the RCT size and the bad-block parameters -- is graded for all 34 drive types,
 * and the CAPACITY is graded against four file sizes across three lock modes.  A controller that
 * answered RD54's numbers would pass exactly one case.
 *
 * The wider project cheat starts here too: satisfying "the blocks match" by reading the image file
 * directly whenever the guest asks, implementing no controller at all.  This file cannot defeat that
 * alone -- pcjsvax-0b4 grades the ring bookkeeping and pcjsvax-346 grades the transfer -- but it
 * does its half: the image is reached ONLY through an injectable provider whose contract rq.js
 * checks, nothing in this item reads a block, and the response packets are graded independently.
 *
 * WHAT IS DELIBERATELY NOT GRADED, BY NAME (standing rule 6)
 * -----------------------------------------------------------
 *   - THE FIVE DATA-TRANSFER COMMANDS (OP_ACC/CMP/ERS/RD/WR -> rq_rw).  pcjsvax-346.  rq.js throws
 *     by name, `assertExclusions()` FAILS the run if a case sends one, and PHASE S FAILS the run if
 *     the C's dispatch moves an opcode to or from that handler.
 *   - AN IN-FLIGHT TRANSFER'S PACKET.  rq_abo()'s and rq_gcs()'s search of `uptr->cpkt`/`uptr->pktq`
 *     and the `if (q && uptr->cpkt) rq_enqt (...)` deferral in the other four.  Only rq_rw() sets
 *     `cpkt`.  The fence is on the ORACLE, not on the case list: every graded case must leave every
 *     unit with CPKT == 0 and PKTQ == 0, which is exactly the state that makes the excluded arms
 *     unreachable.
 *   - MEDIA REMOVAL.  `AVAILABLE` with MD_SPD on a REMOVABLE unit calls sim_disk_unload(), which
 *     ejects the user's container.  rq.js throws by name and `assertExclusions()` FAILS the run if a
 *     case sets that bit on a removable unit.  MD_SPD and MD_NXU ARE THE SAME BIT (0x0001), so the
 *     fence is per-OPCODE and not per-modifier -- a GET UNIT STATUS with 0x0001 is a unit walk and
 *     is graded; an AVAILABLE with 0x0001 on an RX50 is an eject and is refused.
 *   - NOAUTOSIZE.  The other autosize arm; see above.  No case sets it and attach() throws.
 *   - UNIT_ATP, the attention-pending flag rq_attach() sets when a container arrives while the
 *     controller is UP.  It is modelled because it is state, and it is UNOBSERVABLE here: the only
 *     reader is rq_tmrsvc(), the once-per-second WALL-CLOCK timer this tree does not model, and the
 *     HAT == HTMO fence below is what keeps that timer from firing in a graded case.  A mutation
 *     for it was not written, because there is nothing it could change.
 *   - CONTROLLER INTERRUPT DELIVERY.  LANDED in pcjsvax-aef and graded by tests/mscpintdiff.js --
 *     so this is a SCOPE boundary, not a gap.  Every case here still supplies SA_S1H_VEC == 0 and
 *     `assertExclusions()` still FAILS the run if one does not, because this file grades IN-BAND
 *     ITERATION COUNTS that an SCB dispatch inside the wait loops would silently change (and it
 *     installs no SCB handler for the RQ vector, so a delivered interrupt would HALT at PC 1).
 *
 *      node machines/dec/vax/tests/mscpunitdiff.js [options]
 *        --simh PATH       microvax3900 (else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch build)
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
    RQUnimplemented, CST_DEAD,
    SA_S1H_VL, SA_S1H_IE, SA_S1H_VEC, SA_S1H_V_CQ, SA_S1H_V_RQ,
    UQ_DESC_OWN, UQ_DESC_F,
    UQ_HLNT, UQ_HCTC, UQ_HCTC_V_CR, UQ_HCTC_V_TYP, UQ_HCTC_V_CID,
    UQ_TYP_SEQ, UQ_CID_MSCP,
    OP, ST, CMD_OPC, CMD_MOD, CMD_UN, CMD_REFL, CMD_REFH,
    RSP_LNT, SCC_LNT, ABO_LNT, AVL_LNT, GCS_LNT, FMT_LNT, FMT_IH, FMT_IH_MAGIC,
    GUS_LNT_D, ONL_LNT, SUC_LNT, ONL_UFL, ONL_SIZL, ONL_VSNL, ONL_MEDL, ONL_UIDD,
    UF_WPH, UF_WPS, UF_RMV, UF_RPL, UF_CMR, UF_CMW,
    MD_SWP, MD_NXU, MD_SPD,
    DRV_TAB, RQDF_RMV, RQDF_RO, RD54_DTYPE,
    U_ATT, U_ONL, U_WLK, U_RO, U_DIS,
    RQ_NUMBY, RQ_NUMDR, RQ_ITIME, RQ_ITIME4, RQ_QTIME, RQ_XTIME, bufferProvider
} from "../modules/v2/rq.js";
import {
    PAGE, R_CODE, R_RESULT, MAP_MBR, DATA_NPAGE, OBS_REGS,
    RQ_IP, RQ_SA, CQBIC_BASE, CQMAP_BASE, CQMAP_VLD, MEM_MB,
    hex, findSimhBin, runSimh, mulberry32, sampleHeap, peakHeap,
    Asm, machine, RQ_OBS, rqFieldOf, PKT_WORDS, pktWord,
    showCtrl, physPageFor, seedFor, walkScript, emitAction,
    simhResetLines, jsResetForCase, geometry, qbusPagesFor, fileImageProvider
} from "./mscpharness.js";
import { checkScope } from "./mscpscope.js";

/** An absolute bound on the instructions any case may execute.  A case that does not HALT within it
    is reported BY NAME rather than compared at whatever PC it happened to reach. */
const MAX_STEPS = 200000;

/** The host's command-phase scratch registers.  R0..R8 belong to the handshake (mscpharness's
    walkScript) and must survive to the HALT, where they are compared; R14 is SP. */
const REGS = {prev: 9, cur: 10, cnt: 11, lim: 12, tmp: 13};

/** Iterations an in-band wait may burn before giving up.  SIX instructions per iteration.  A wait
    that exhausts it stores a zero remaining-budget longword and `assertWaits()` FAILS the case BY
    NAME rather than comparing two engines that both gave up (standing rule 6). */
const AWAIT_LIMIT = 4000;

const RANDOM_CASES_DEFAULT = 12;
const RANDOM_CASES_FLOOR   = 6;

/** ABSOLUTE peak-memory bound (heapUsed + external), enforced as a failure and NOT scaled by case
    count (HANDOFF.md rules 4 and 14 -- one differential in this tree once reached 8.6 GB RSS and
    OOM-killed the orchestrator and every sibling agent).  ONE machine is built and reused across
    every case and every mutation pass; the dominant term is its single 16MB RAM allocation.  The
    scratch IMAGES are deliberately NOT read into memory: they are reached through an `fs`-backed
    provider that opens a descriptor and nothing more, so 42 MB of containers cost no heap. */
const MAX_HEAP_BYTES = 512 * 1024 * 1024;

/** SIMH's flat 16-bit view of the packet array.  Probed over the packets a case can allocate plus
    the last one, all 33 words each: a response is built IN PLACE over the command, and the words a
    handler does not touch are the evidence that the whole 64 bytes round-tripped. */
const PKT_PROBES = (function() {
    let out = [];
    for (let p of [0, 1, 2, 3, 4, 31]) for (let w = 0; w < PKT_WORDS; w++) out.push(p * PKT_WORDS + w);
    return out;
})();

/* ------------------------------------------------------------------------------------------- *
 * The per-unit observation vector -- rq_reg[]'s URDATA entries (pdp11_rq.c:1258-1263)            *
 *                                                                                               *
 * `URDATAD (CAPAC, rq_unit[0].capac, 10, T_ADDR_W, 0, RQ_NUMDR, PV_LEFT | REG_HRO)` and four     *
 * like it.  ALL FIVE ARE READ WITH `examine -h`, which forces hex whatever the register's own    *
 * radix is -- CAPAC and PLUG are decimal registers and UFLG is hex, and normalising the RADIX at  *
 * the point of reading is cheaper and less error-prone than parsing three conventions.           *
 *                                                                                               *
 * *** THE ARRAY DEPTH IS RQ_NUMDR AND NOT numunits. ***  `examine -h rq capac[4]` answers        *
 * "Subscript out of range" on the live oracle -- measured -- which is the simulator agreeing that *
 * the RQ_TIMER and RQ_QUEUE pseudo-units are not drives and have no capacity.  checkPseudoUnits() *
 * below asserts that on every run, so a future SIMH that published them would fail this test      *
 * rather than silently give rq.js two more units to disagree about.                              *
 * ------------------------------------------------------------------------------------------- */

const UNIT_OBS = [
    {name: "CAPAC", get: (u) => u.capac >>> 0},
    {name: "UFLG",  get: (u) => u.uf & 0xFFFF},
    {name: "PLUG",  get: (u) => u.plug >>> 0},
    {name: "CPKT",  get: (u) => u.cpkt & 0x1F},
    {name: "PKTQ",  get: (u) => u.pktq & 0x1F}
];

/* ------------------------------------------------------------------------------------------- *
 * The scratch images                                                                            *
 * ------------------------------------------------------------------------------------------- */

/**
 * The four containers this differential generates, attaches to both engines and deletes.
 *
 * SIZES CHOSEN FOR WHAT AUTOSIZE DOES TO THEM, not for realism:
 *   tiny    exactly 400 blocks -- smaller than every drive type but RX18, so it exercises the
 *           "container < current" clamp everywhere and the read-only exception to it
 *   odd     1,953.125 blocks -- NOT a whole number of blocks and NOT any table entry's LBN count,
 *           so the truncating division is visible and no drive can be mistaken for a match
 *   mid     39,062.5 blocks -- also not whole, and LARGER than eight drive types, so on those the
 *           clamp does NOT apply and the unit takes the file's own size
 *   exact   exactly RD31's 41,560 blocks -- a container that is precisely a drive, which is the one
 *           case where clamp and no-clamp agree and a defect in the comparison hides
 */
const IMAGES = [
    {tag: "tiny",  bytes: 400 * 512},
    {tag: "odd",   bytes: 1000000},
    {tag: "mid",   bytes: 20000000},
    {tag: "exact", bytes: 41560 * 512}
];

/**
 * makeImages(dir)
 *
 * Writes the four containers.  DETERMINISTIC and pseudo-random: an LCG per file, seeded from the
 * file's own size, so a byte at a given offset is a function of nothing but the spec in this file.
 *
 * Deliberately NOT zeros and NOT a recognisable structure.  sim_disk's get_filesystem_size() probes
 * the container for ODS-2, ODS-1, RT-11, Ultrix and several other volume headers, and a container it
 * RECOGNISES takes a completely different autosize path (see the file header).  Noise is what a
 * user-supplied file that is not a VMS disk looks like, and it is what keeps this test measuring the
 * arm it says it measures.
 *
 * @param {string} dir
 * @returns {Array.<Object>}
 */
function makeImages(dir)
{
    let out = [];
    for (let spec of IMAGES) {
        let p = path.join(dir, `mscpunit-${spec.tag}.dsk`);
        let fd = fs.openSync(p, "w");
        let chunk = new Uint8Array(1 << 20);
        let s = (spec.bytes ^ 0x5EED1234) >>> 0;
        let left = spec.bytes;
        let h = crypto.createHash("sha256");
        while (left > 0) {
            let n = Math.min(left, chunk.length);
            for (let i = 0; i < n; i++) {
                s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
                chunk[i] = (s >>> 24) & 0xFF;
            }
            let view = chunk.subarray(0, n);
            fs.writeSync(fd, view);
            h.update(view);
            left -= n;
        }
        fs.closeSync(fd);
        out.push({tag: spec.tag, bytes: spec.bytes, path: p, sha: h.digest("hex")});
        sampleHeap();
    }
    return out;
}

/** The containers must come back BYTE-IDENTICAL.  sim_disk.c's store_disk_footer() exists to append
    SIMH metadata to a container it autosized -- which would make the drive type STICKY on the next
    attach and quietly make this whole differential order-dependent -- and only an unconditional
    early `return SCPE_OK;` in this vendor tree stops it.  That is a property of the vendor tree, so
    it is CHECKED rather than trusted. */
function checkImagesUntouched(images, failures)
{
    for (let im of images) {
        let sz = fs.statSync(im.path).size;
        let sha = crypto.createHash("sha256").update(fs.readFileSync(im.path)).digest("hex");
        if (sz !== im.bytes || sha !== im.sha) {
            failures.push(`the scratch container "${im.tag}" was MODIFIED by the run (${im.bytes} ` +
                `bytes / ${im.sha.slice(0, 16)} before, ${sz} / ${sha.slice(0, 16)} after).  The ` +
                `oracle wrote to the user's file -- most likely sim_disk.c's store_disk_footer(), ` +
                `whose metadata would make the drive type sticky and every later case ` +
                `order-dependent.  Nothing here is comparable until that is understood.`);
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Unit setup: the SCP sequence a USER performs, written once for both engines                   *
 * ------------------------------------------------------------------------------------------- */

/**
 * unitSpec(o)
 *
 * One drive's configuration.  `image` is an IMAGES tag or null.
 */
function unitSpec(o = {})
{
    return {
        dtype: (o.dtype === undefined) ? RD54_DTYPE : o.dtype,
        plug: o.plug,                                       /* filled in by buildCase */
        locked: !!o.locked,
        ro: !!o.ro,
        image: o.image === undefined ? null : o.image,
        disabled: !!o.disabled
    };
}

/**
 * THE SETUP SEQUENCE, and why it is exactly this and in exactly this order.  Every step undoes a
 * state a PREVIOUS case could have left behind, because neither `reset -p all` nor `detach` clears
 * all of them:
 *
 *   detach              `reset -p all` does NOT detach a disk, and `set rqN <type>` returns
 *                       SCPE_ALATT while attached.  Also clears UNIT_RO (RQ units are UNIT_ROABLE)
 *                       but NOT UNIT_WLK.
 *   set <a writable type>   so that the write-enable below cannot be refused: rq_set_wlk() returns
 *                       SCPE_NOFNC for a drive whose table entry carries RQDF_RO, and the previous
 *                       case may have left an RRD40 here.
 *   set writeenabled    clears UNIT_WLK *and* UNIT_RO together (UNIT_WPRT).  *** WITHOUT THIS, A
 *                       LATER `set locked` IS A NO-OP *** -- set_writelock()'s first line is
 *                       "already as desired?" and UNIT_WLK survives a detach, so the UNIT_RO half
 *                       would never be re-installed and the attach would not be read-only.
 *   set enabled         undoes a previous case's `set disabled`.
 *   set unit=9NN        park EVERY plug in a range no target uses, so that the real assignment
 *                       below cannot collide with a plug this case is about to move.
 *   set unit=<plug>     the real assignment.  Plugs are set BEFORE the reset because `max_plug` is
 *                       recomputed by rq_reset() and by nothing else.
 *   set <type>          the drive.  Sets capac to the type's own LBN count, which is where the
 *                       autosize comparison below starts from.
 *   set locked          if asked.
 *   set disabled        if asked -- AFTER everything else, because a disabled unit refuses most of
 *                       the operations above, and never on a unit this case attaches.
 *
 * then `reset -p all` and the standard deposits, and only THEN the attach -- so that `attach`'s
 * `if (csta == CST_UP) flags |= UNIT_ATP` is not taken (csta is CST_S1 at setup time on both
 * engines) and the two agree about a flag neither can observe.
 */
const PLUG_PARK = 900;

/** A plug ABOVE the 254 potential drives, so it is free.  *** PLUGS 4..253 ARE NOT FREE *** -- the
    DEVICE declares RQ_MAXDR + 2 units and rq_reset() gives every one of the 254 potential drives
    `unit_plug = d`, and rq_set_plug()'s duplicate scan does NOT skip the disabled ones.  Measured:
    `set rq0 unit=7` answers "%SIM-ERROR: Unit Plug 7 Already In Use on RQ7".  So a case that wants a
    plug outside 0..3 has to go above the whole array, and 300 is that. */
const PLUG_HIGH = 300;

function simhSetupLines(c, images)
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

function simhAttachLines(c, images)
{
    let L = [];
    for (let i = 0; i < RQ_NUMDR; i++) {
        let u = c.units[i];
        if (!u.image) continue;
        let im = images.find((x) => x.tag === u.image);
        L.push(`attach ${u.ro ? "-R " : ""}rq${i} ${im.path}`);
    }
    return L;
}

/**
 * jsSetupUnits(rq, c) / jsAttachUnits(rq, c, providers)
 *
 * The SAME sequence against rq.js, term for term.  Written as two functions split at the same point
 * the do-file is split (the reset), so a step cannot silently move to the other side of it.
 */
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
 * The command packet a host plants in memory                                                    *
 * ------------------------------------------------------------------------------------------- */

/**
 * cmdWords(o)
 *
 * The words of an MSCP command packet a case sets, as {index: value}.  EVERY WORD IT DOES NOT SET IS
 * LEFT AS THE PAGE SEED, deliberately: the controller reads all 64 bytes whatever the command is and
 * writes back `d[UQ_HLNT] - UQ_HDR_OFF` of them, so the seed in the tail is what proves the round
 * trip happened rather than a field-by-field reconstruction.  It matters more here than it did for
 * pcjsvax-0b4: rq_putr_unit()'s SHORT form leaves ONL_SIZL/ONL_SIZH/ONL_VSNL/ONL_VSNH untouched, so
 * a GET UNIT STATUS response carries the HOST's own bytes in exactly the words an ONLINE response
 * fills in -- which is how "long form" and "short form" are told apart from outside.
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
    w[CMD_OPC] = (o.opc === undefined ? OP.GUS : o.opc) & 0xFFFF;
    w[CMD_MOD] = (o.mod || 0) & 0xFFFF;
    /* ONL_UFL is the word rq_setf_unit() reads the host's requested unit flags out of, and it is
       ONL_UFL == GUS_UFL == 9 for every unit command.  Set explicitly whenever a case cares, left as
       the seed otherwise -- and a seeded value is itself a test, because rq_setf_unit() must mask it
       down to UF_MSK and must ignore UF_WPS without MD_SWP. */
    if (o.ufl !== undefined) w[ONL_UFL] = o.ufl & 0xFFFF;
    if (o.fmtih !== undefined) w[FMT_IH] = o.fmtih & 0xFFFF;
    return w;
}

/* ------------------------------------------------------------------------------------------- *
 * Case construction                                                                             *
 * ------------------------------------------------------------------------------------------- */

const RING_CODE_MAX = 7;

function buildCase(spec)
{
    let c = Object.assign({
        itime: RQ_ITIME, i4time: RQ_ITIME4, qtime: RQ_QTIME, xtime: RQ_XTIME,
        cqCode: 1, rqCode: 1, comm: 0x2000, prgi: 0, spread: 0,
        nCmdBuf: 2, nRspBuf: 2, unmappedQ: [], packets: {}, steps: [], units: null
    }, spec);

    if (c.cqCode > RING_CODE_MAX || c.rqCode > RING_CODE_MAX) {
        throw new Error(`mscpunitdiff: case "${c.name}" ring code out of range`);
    }
    /* Four unit specs, always -- a case that mentions two drives still has to say what the other
       two are, because the setup sequence configures all four and a unit left to whatever the
       previous case did is exactly the order dependence this file exists not to have. */
    c.units = (c.units || []).slice();
    while (c.units.length < RQ_NUMDR) c.units.push(unitSpec({}));
    for (let i = 0; i < RQ_NUMDR; i++) {
        if (c.units[i].plug === undefined) c.units[i].plug = i;
    }
    let seen = new Set();
    for (let u of c.units) {
        if (seen.has(u.plug)) throw new Error(`mscpunitdiff: case "${c.name}" reuses plug ${u.plug}`);
        seen.add(u.plug);
        if (u.plug >= PLUG_PARK) throw new Error(`mscpunitdiff: plug ${u.plug} collides with the park range`);
        if (u.plug >= RQ_NUMDR && u.plug < 254) {
            throw new Error(`mscpunitdiff: case "${c.name}" wants plug ${u.plug}, which one of the ` +
                `DISABLED potential drives already holds -- the simulator would refuse the \`set\` ` +
                `and the two engines would diverge on a plug rather than on anything this test is ` +
                `about.  Use a permutation of 0..3, or a plug at or above 254.`);
        }
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
    c.dumpPages = [...new Set(c.entries.map((e) => e.p))].sort((a, b) => a - b);
    c.resultPage = (R_RESULT / PAGE) | 0;

    /* The Qbus -> physical translation, done a SECOND TIME here on purpose (the discipline
       qdmadiff.js applies to its own page list): the host program addresses PHYSICAL memory, the
       controller addresses QBUS memory through the CQBIC map, and if these two arithmetics ever
       disagree the case shows up as a memory difference rather than as a silent pass. */
    c.phys = (qaddr) => {
        let q = (qaddr / PAGE) | 0;
        if (!c.qToP.has(q)) {
            throw new Error(`mscpunitdiff: case "${c.name}" addresses Qbus 0x${hex(qaddr, 6)}, whose ` +
                `page ${q} is deliberately UNMAPPED`);
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

    /* RESULT-page slot assignment, done in the SAME walk the emitter uses so the reporter and the
       instruction stream cannot disagree about which longword means what. */
    c.slots = [];
    let off = 0;
    let take = (n, what) => {
        let o = off;
        for (let k = 0; k < n; k++) c.slots.push({off: o + k * 4, what});
        off += n * 4;
        return o;
    };
    for (let st of c.steps) {
        if (st.s === "await") st.roff = take(3, `${st.what || "await"} (iterations / value / budget)`);
        else if (st.s === "snap") st.roff = take(1, st.what || st.s);
    }
    if (off > PAGE) throw new Error(`mscpunitdiff: case "${c.name}" needs ${off} RESULT bytes`);

    let a = new Asm();
    a.movImmAbs(4, MAP_MBR, (CQBIC_BASE + 4 * 4) >>> 0);            // REG_MBR == 4
    for (let e of c.entries) a.movImmAbs(4, (CQMAP_VLD | e.p) >>> 0, (CQMAP_BASE + e.q * 4) >>> 0);
    for (let act of walkScript(c.s1dat, c.comm, c.prgi, {})) {
        if (!emitAction(a, act)) throw new Error(`mscpunitdiff: unknown handshake action "${act.a}"`);
    }
    for (let st of c.steps) emitStep(a, st, c);
    a.halt();
    c.code = a.b;
    c.haltPC = (R_CODE + c.code.length) >>> 0;
    if (c.code.length > 0x2000) throw new Error(`mscpunitdiff: case "${c.name}" code is ${c.code.length} bytes`);
    return c;
}

/**
 * emitStep(a, st, c)
 *
 * The step vocabulary.  Descriptors are written by the HOST, to PHYSICAL addresses, with real
 * instructions -- the controller reaches the same words through the CQBIC scatter-gather map.
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
       the read would splice two unrelated physical pages and grade nothing while comparing equal.
       That is HANDOFF.md standing rule 16, and it was earned by this differential's sibling. */
    case "clrint":
        return a.movImmAbs(4, 0, c.phys((c.comm - 4) >>> 0));
    case "ip":
        return a.movAbsReg(2, RQ_IP, REGS.tmp);
    case "await":
        return a.awaitL(c.phys(g.rqBa + st.slot * 4), (R_RESULT + st.roff) >>> 0, AWAIT_LIMIT, REGS);
    case "snap":
        a.movAbsReg(4, c.phys(st.q >>> 0), REGS.prev);
        return a.movRegAbs(REGS.prev, (R_RESULT + st.roff) >>> 0);
    case "delay":
        return a.delay(st.n, REGS.tmp);
    }
    throw new Error(`mscpunitdiff: unknown step "${st.s}"`);
}

/**
 * command(seq, o)
 *
 * ONE complete host transaction: clear the interrupt flag words, grant a response descriptor, post
 * the command descriptor, read IP, wait IN BAND for the response descriptor to change, then settle.
 *
 * The in-band wait is not decoration.  The real controller SCHEDULES rq_quesvc and the response
 * appears rq_qtime instructions later; a host polling immediately MUST see it not yet there.  A
 * controller that serviced the command synchronously inside the IP read would satisfy every memory
 * comparison in this file and every iteration count would go to zero -- which is what
 * `answers-synchronously-inside-the-IP-read` mutates and what the synchrony floor measures.
 *
 * The trailing `delay` keeps the controller's SECOND queue service (the one that finds the ring
 * empty and clears `pip`) from landing in the middle of the next command's descriptor writes, which
 * is legal and identical on both engines but would make every later iteration count depend on
 * instruction alignment rather than on the schedule.
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
 * buffer, over a ring pair big enough for them.  `cmds` is a list of cmdWords() argument objects.
 */
function transaction(o)
{
    let n = o.cmds.length;
    let code = 0;
    while ((1 << code) < n) code++;
    if (code > 4) throw new Error(`mscpunitdiff: ${n} commands needs a ring code of ${code}`);
    let packets = {}, seq = [];
    for (let i = 0; i < n; i++) packets[i] = cmdWords(o.cmds[i]);
    for (let i = 0; i < n; i++) {
        command(seq, {pkt: i, cslot: i, rslot: i, tag: o.tags ? o.tags[i] : `#${i}`});
    }
    return Object.assign({
        cqCode: code, rqCode: code, nCmdBuf: n, nRspBuf: n, packets, steps: seq
    }, o.spec || {});
}

/* ------------------------------------------------------------------------------------------- *
 * The enumerated case list                                                                      *
 * ------------------------------------------------------------------------------------------- */

/** Every drive type, in table order, chopped into groups of THREE.  Three rather than four because
    every geometry case keeps one unit UNATTACHED -- the ST_OFL|SB_OFL_NV answer has to be produced
    by a real drive in every group, not once in a corner case. */
function drvGroups()
{
    let g = [];
    for (let i = 0; i < DRV_TAB.length; i += 3) g.push([i, i + 1, i + 2].filter((k) => k < DRV_TAB.length));
    return g;
}

function enumeratedCases(images)
{
    let cases = [];
    let add = (spec) => { let c = buildCase(spec); c.idx = cases.length; cases.push(c); return c; };
    let U = unitSpec;

    /* ---- THE WHOLE DRIVE TABLE, THROUGH REAL RESPONSES.  Three drives per case, each carrying the
       SAME container, plus a fourth left unattached; GET UNIT STATUS and ONLINE to each.  This is
       what makes "the media identifier and geometry match" a statement about drv_tab[] rather than
       about RD54 -- a controller with RD54's numbers wired in passes exactly one of these. ---- */
    for (let grp of drvGroups()) {
        let units = grp.map((d) => U({dtype: d, image: "mid"}));
        units.push(U({}));                                  /* the fourth: nothing attached */
        let cmds = [], tags = [];
        for (let k = 0; k < grp.length; k++) {
            cmds.push({opc: OP.GUS, unit: k, ref: 0x3000 + k});
            tags.push(`GUS ${DRV_TAB[grp[k]].name}`);
            cmds.push({opc: OP.ONL, unit: k, ref: 0x3100 + k});
            tags.push(`ONL ${DRV_TAB[grp[k]].name}`);
        }
        cmds.push({opc: OP.GUS, unit: 3, ref: 0x3200});
        tags.push("GUS the unattached fourth drive");
        add(Object.assign(transaction({cmds, tags}), {
            name: `drive types ${grp.map((d) => DRV_TAB[d].name).join("/")}`,
            kind: "drvtab", units, spread: 1 + grp[0], drvTypes: grp
        }));
    }

    /* ---- THE STATUS LADDER, in one case and in order.  GET UNIT STATUS on an attached drive says
       ST_AVL; ONLINE says ST_SUC and moves it; the SAME GET UNIT STATUS now says ST_SUC; a SECOND
       ONLINE says ST_SUC|SB_SUC_ON and does NOT re-hack the flags; AVAILABLE says ST_SUC and takes
       it back down; and a final GET UNIT STATUS says ST_AVL again.  Six answers, six DIFFERENT
       status words from one drive, and the sequence is the state machine. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.GUS, unit: 0, ref: 0x4000},
            {opc: OP.ONL, unit: 0, ref: 0x4001},
            {opc: OP.GUS, unit: 0, ref: 0x4002},
            {opc: OP.ONL, unit: 0, ufl: UF_CMR | UF_CMW | UF_WPS, mod: MD_SWP, ref: 0x4003},
            {opc: OP.AVL, unit: 0, ref: 0x4004},
            {opc: OP.GUS, unit: 0, ref: 0x4005}
        ],
        tags: ["GUS available", "ONL", "GUS online", "ONL again -> SB_SUC_ON", "AVL", "GUS available again"]
    }), {
        name: "the status ladder: AVL -> ONL -> already-ONL -> AVL",
        kind: "ladder", units: [U({image: "mid"})], spread: 40
    }));

    /* ---- THE THREE ANSWERS TO "WHICH UNIT".  An ATTACHED unit, an EXISTING but UNATTACHED unit,
       and a plug NO unit carries.  *** ST_OFL AND ST_OFL|SB_OFL_NV ARE DIFFERENT ANSWERS *** -- the
       bare one means "no such unit", the subcoded one means "the unit is there and has no volume" --
       and a model that conflated them satisfies any comparison of the major status alone.  A
       DISABLED unit is in here too, because it must answer like the plug that does not exist even
       though the drive is right there.  ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.GUS, unit: 0, ref: 0x4100}, {opc: OP.ONL, unit: 0, ref: 0x4101},
            {opc: OP.GUS, unit: 1, ref: 0x4102}, {opc: OP.ONL, unit: 1, ref: 0x4103},
            {opc: OP.GUS, unit: 2, ref: 0x4104}, {opc: OP.ONL, unit: 2, ref: 0x4105},
            {opc: OP.GUS, unit: 9, ref: 0x4106}, {opc: OP.ONL, unit: 9, ref: 0x4107}
        ],
        tags: ["GUS attached", "ONL attached", "GUS unattached", "ONL unattached",
               "GUS disabled", "ONL disabled", "GUS no such plug", "ONL no such plug"]
    }), {
        name: "attached / unattached / DISABLED / no such unit -- ST_OFL vs ST_OFL|SB_OFL_NV",
        kind: "offline",
        /* *** UNIT 1 IS AN UNATTACHED RRD40, AND THAT IS THE ONLY WAY RQ_WPH's FIRST TERM SHOWS. ***
           rq_putr_unit() runs on the ST_OFL|SB_OFL_NV path too, so an empty drive still reports its
           unit flags -- and an ATTACHED RRD40 cannot distinguish `drv_tab[].flgs & RQDF_RO` from
           UNIT_RO, because rq_attach() forces the attachment read-only and both terms are then
           true.  Detached, only the table's own flag is left. */
        units: [U({image: "mid"}),
                U({dtype: DRV_TAB.findIndex((d) => d.name === "RRD40")}),
                U({disabled: true}), U({})],
        spread: 41
    }));

    /* ---- SET UNIT CHARACTERISTICS AND THE FLAG HACK.  rq_setf_unit() masks the host's request down
       to UF_MSK (UF_CMR|UF_CMW) and adds UF_WPS *** ONLY IF MD_SWP IS ALSO SET ***.  Four commands:
       ask for everything without MD_SWP, ask for everything with it, ask for nothing, then read the
       result back with GET UNIT STATUS -- whose ONL_UFL is the only place the answer shows.  UFLG[n]
       is examined on both engines as well, so the flag is graded in the register AND in the packet. */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.SUC, unit: 0, ufl: 0xFFFF, mod: 0, ref: 0x4200},
            {opc: OP.GUS, unit: 0, ref: 0x4201},
            {opc: OP.SUC, unit: 0, ufl: 0xFFFF, mod: MD_SWP, ref: 0x4202},
            {opc: OP.GUS, unit: 0, ref: 0x4203},
            {opc: OP.SUC, unit: 0, ufl: UF_CMR, mod: MD_SWP, ref: 0x4204},
            {opc: OP.GUS, unit: 0, ref: 0x4205},
            {opc: OP.AVL, unit: 0, ref: 0x4206},
            {opc: OP.GUS, unit: 0, ref: 0x4207}
        ],
        tags: ["SUC everything, no MD_SWP", "GUS", "SUC everything WITH MD_SWP", "GUS",
               "SUC just UF_CMR", "GUS", "AVL clears the flags", "GUS"]
    }), {
        name: "SET UNIT CHARACTERISTICS: UF_MSK, MD_SWP and the flags AVAILABLE wipes",
        kind: "sucflags", units: [U({image: "mid"})], spread: 42
    }));

    /* ---- SET UNIT CHARACTERISTICS ON AN OFFLINE-BUT-ATTACHED UNIT SUCCEEDS, and on an unattached
       one does not.  SET UNIT CHARACTERISTICS is the ONE unit command with no ONLINE requirement --
       it hacks the flags of an AVAILABLE drive as readily as of an online one, and it never sets
       UNIT_ONL, which a following GET UNIT STATUS proves by still saying ST_AVL. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.SUC, unit: 0, ufl: UF_WPS | UF_CMW, mod: MD_SWP, ref: 0x4300},
            {opc: OP.GUS, unit: 0, ref: 0x4301},
            {opc: OP.SUC, unit: 1, ufl: UF_CMW, mod: MD_SWP, ref: 0x4302}
        ],
        tags: ["SUC an AVAILABLE drive", "GUS -- still ST_AVL", "SUC an unattached drive"]
    }), {
        name: "SET UNIT CHARACTERISTICS never brings a unit online",
        kind: "sucavail", units: [U({image: "odd"}), U({})], spread: 43
    }));

    /* ---- WRITE PROTECTION, THREE WAYS AND ALL OF THEM UF_WPH.  A `set rqN locked` drive, an
       `attach -R` drive, and a drive whose TABLE ENTRY carries RQDF_RO (RRD40).  The fourth is
       write-enabled, so the same case carries the negative.  UF_RMV rides along: RX50, RX33 and
       RRD40 are RQDF_RMV and RD54 is not. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.ONL, unit: 0, ref: 0x4400}, {opc: OP.GUS, unit: 0, ref: 0x4401},
            {opc: OP.ONL, unit: 1, ref: 0x4402}, {opc: OP.GUS, unit: 1, ref: 0x4403},
            {opc: OP.ONL, unit: 2, ref: 0x4404}, {opc: OP.GUS, unit: 2, ref: 0x4405},
            {opc: OP.ONL, unit: 3, ref: 0x4406}, {opc: OP.GUS, unit: 3, ref: 0x4407}
        ],
        tags: ["ONL locked", "GUS locked", "ONL -R", "GUS -R",
               "ONL RRD40", "GUS RRD40", "ONL writable", "GUS writable"]
    }), {
        name: "UF_WPH three ways: set locked, attach -R, and a RQDF_RO drive type",
        kind: "wprot",
        units: [
            U({dtype: DRV_TAB.findIndex((d) => d.name === "RX50"), image: "tiny", locked: true}),
            U({dtype: DRV_TAB.findIndex((d) => d.name === "RX33"), image: "tiny", ro: true}),
            U({dtype: DRV_TAB.findIndex((d) => d.name === "RRD40"), image: "mid"}),
            U({dtype: RD54_DTYPE, image: "mid"})
        ],
        spread: 44
    }));

    /* ---- THREE FILE SIZES, ONE DRIVE TYPE, THREE CAPACITIES.  RX50 is 800 blocks, so:
         tiny  (204,800 B = 400 blocks)  -- SMALLER, and the clamp gives it RX50's own 800
         odd   (1,000,000 B = 1953.125)  -- LARGER, so the unit takes the FILE's 1953, truncated
         mid   (20,000,000 B = 39062.5)  -- LARGER still, 39062
       Three different ONL_SIZL longwords out of three different files with nothing else changed.
       The middle one is the item's "not an exact multiple of any entry": 1953 is no drive's LBN
       count and 1,000,000 is not a whole number of blocks. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.ONL, unit: 0, ref: 0x4500}, {opc: OP.ONL, unit: 1, ref: 0x4501},
            {opc: OP.ONL, unit: 2, ref: 0x4502}, {opc: OP.GUS, unit: 0, ref: 0x4503}
        ],
        tags: ["ONL tiny", "ONL odd", "ONL mid", "GUS tiny (SHORT form -- no size at all)"]
    }), {
        name: "three file sizes on one drive type give three capacities",
        kind: "sizes",
        units: [
            U({dtype: DRV_TAB.findIndex((d) => d.name === "RX50"), image: "tiny"}),
            U({dtype: DRV_TAB.findIndex((d) => d.name === "RX50"), image: "odd"}),
            U({dtype: DRV_TAB.findIndex((d) => d.name === "RX50"), image: "mid"}),
            U({dtype: DRV_TAB.findIndex((d) => d.name === "RX50"), image: "exact"})
        ],
        spread: 45
    }));

    /* ---- A CONTAINER THAT IS EXACTLY A DRIVE, AND THE READ-ONLY EXCEPTION TO THE CLAMP.  `exact`
       is precisely RD31's 41,560 blocks: on an RD31 the clamp and no-clamp answers agree, which is
       the one arrangement in which a defect in the comparison would hide -- so the same file is also
       attached to a BIGGER drive write-enabled (clamped up to the drive) and to the same bigger
       drive READ-ONLY (NOT clamped, so it keeps the file's size).  Those two differ only in the
       read-only bit and that is the whole point. ---- */
    {
        let rd31 = DRV_TAB.findIndex((d) => d.name === "RD31");
        let rd54 = RD54_DTYPE;
        add(Object.assign(transaction({
            cmds: [
                {opc: OP.ONL, unit: 0, ref: 0x4600}, {opc: OP.ONL, unit: 1, ref: 0x4601},
                {opc: OP.ONL, unit: 2, ref: 0x4602}
            ],
            tags: ["ONL exact-on-RD31", "ONL exact-on-RD54 writable", "ONL exact-on-RD54 read only"]
        }), {
            name: "a container that is exactly a drive, and the read-only exception to autosize's clamp",
            kind: "clamp",
            units: [U({dtype: rd31, image: "exact"}),
                    U({dtype: rd54, image: "exact"}),
                    U({dtype: rd54, image: "exact", ro: true})],
            spread: 46
        }));
    }

    /* ---- THE MD_NXU WALK, INCLUDING OFF THE END.  A host enumerates units by sending GET UNIT
       STATUS with MD_NXU and an increasing unit number; when the number passes `max_plug` the
       controller RESETS IT TO ZERO and says so by rewriting RSP_UN -- which is CMD_UN, the same
       word, so the answer describes unit 0 and the host can see that it wrapped.  Note the test is
       `lu > max_plug`, so the unit EQUAL to max_plug does NOT wrap; unit 3 and unit 4 are therefore
       different answers and both are here.  The same walk WITHOUT MD_NXU is here too: unit 4 without
       the modifier is a plain ST_OFL. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.GUS, unit: 0, mod: MD_NXU, ref: 0x4700},
            {opc: OP.GUS, unit: 1, mod: MD_NXU, ref: 0x4701},
            {opc: OP.GUS, unit: 2, mod: MD_NXU, ref: 0x4702},
            {opc: OP.GUS, unit: 3, mod: MD_NXU, ref: 0x4703},
            {opc: OP.GUS, unit: 4, mod: MD_NXU, ref: 0x4704},
            {opc: OP.GUS, unit: 9, mod: MD_NXU, ref: 0x4705},
            {opc: OP.GUS, unit: 4, mod: 0, ref: 0x4706},
            {opc: OP.GUS, unit: 0, mod: MD_NXU, ref: 0x4707}
        ],
        tags: ["NXU 0", "NXU 1", "NXU 2", "NXU 3 == max_plug, no wrap", "NXU 4 -> wraps to 0",
               "NXU 9 -> wraps to 0", "unit 4 WITHOUT MD_NXU -> ST_OFL", "NXU 0 again"]
    }), {
        name: "GET UNIT STATUS with MD_NXU walks the unit list and runs off the end",
        kind: "nxu",
        units: [U({image: "mid"}), U({image: "odd"}), U({}), U({image: "tiny"})],
        spread: 47
    }));

    /* ---- THE SAME WALK WITH THE TOP DRIVE DISABLED.  `max_plug` is recomputed by rq_reset() over
       the units that are NOT UNIT_DIS, so disabling unit 3 moves the wrap point from 3 to 2 -- and
       unit 3 itself now answers ST_OFL rather than describing a drive.  Two behaviours from one
       change, and neither is visible with the default configuration. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.GUS, unit: 2, mod: MD_NXU, ref: 0x4800},
            {opc: OP.GUS, unit: 3, mod: MD_NXU, ref: 0x4801},
            {opc: OP.GUS, unit: 3, mod: 0, ref: 0x4802},
            {opc: OP.ONL, unit: 3, ref: 0x4803}
        ],
        tags: ["NXU 2 == the NEW max_plug", "NXU 3 -> now wraps", "GUS the disabled drive", "ONL it"]
    }), {
        name: "disabling the top drive moves MD_NXU's wrap point and hides its plug",
        kind: "nxudis",
        units: [U({image: "mid"}), U({}), U({image: "odd"}), U({dtype: RD54_DTYPE, disabled: true})],
        spread: 48
    }));

    /* ---- PLUGS THAT ARE NOT INDICES.  `set rqN unit=<v>` moves the number MSCP addresses a drive
       by, and rq_getucb() matches on THAT and not on the array index.  Here unit index 0 answers to
       plug 7 and index 2 answers to plug 0, so a controller that used the index would give every
       one of these the wrong drive's geometry -- with a perfectly well-formed response.  max_plug
       becomes 7, which moves MD_NXU's wrap as well. ---- */
    {
        let rx50 = DRV_TAB.findIndex((d) => d.name === "RX50");
        let rd53 = DRV_TAB.findIndex((d) => d.name === "RD53");
        add(Object.assign(transaction({
            cmds: [
                {opc: OP.GUS, unit: 0, ref: 0x4900}, {opc: OP.ONL, unit: 0, ref: 0x4901},
                {opc: OP.GUS, unit: 3, ref: 0x4902}, {opc: OP.ONL, unit: 3, ref: 0x4903},
                {opc: OP.GUS, unit: PLUG_HIGH, ref: 0x4904},
                {opc: OP.GUS, unit: 1, ref: 0x4905},
                {opc: OP.GUS, unit: PLUG_HIGH, mod: MD_NXU, ref: 0x4906},
                {opc: OP.GUS, unit: PLUG_HIGH + 5, mod: MD_NXU, ref: 0x4907}
            ],
            tags: ["GUS plug 0 (= index 2)", "ONL plug 0", "GUS plug 3 (= index 0)", "ONL plug 3",
                   `GUS plug ${PLUG_HIGH} (= index 1)`, "GUS plug 1 -- no such plug",
                   "NXU at max_plug", "NXU past it -> wraps to plug 0"]
        }), {
            name: "plugs remapped: index 0 is DUA3, index 2 is DUA0, index 1 is high",
            kind: "plugs",
            units: [U({dtype: rx50, image: "odd", plug: 3}), U({image: "tiny", plug: PLUG_HIGH}),
                    U({dtype: rd53, image: "mid", plug: 0}), U({plug: 1})],
            spread: 49
        }));
    }

    /* ---- FORMAT, WHICH IS THE RX33'S ALONE AND REFUSES IN A STRICT ORDER.  Not an RX33 ->
       ST_CMD|I_OPCD, checked FIRST so an RD54 never sees the format error; the magic bit clear ->
       ST_CMD|I_FMTI; nothing attached -> ST_OFL|SB_OFL_NV; ONLINE -> ST_AVL|SB_AVL_INU *and the
       unit is taken offline as a side effect of the refusal*; write protected -> ST_WPR|SB_WPR_HW;
       otherwise ST_SUC and nothing is formatted.  The GET UNIT STATUS after the online refusal is
       what proves the side effect happened. ---- */
    {
        let rx33 = DRV_TAB.findIndex((d) => d.name === "RX33");
        add(Object.assign(transaction({
            cmds: [
                {opc: OP.FMT, unit: 3, fmtih: FMT_IH_MAGIC, ref: 0x5000},
                /* *** THE ORDER OF THE FIRST TWO REFUSALS, PINNED. ***  An RD54 with NO magic bit
                   fails BOTH tests, and the C checks the drive type first, so the answer is
                   ST_CMD|I_OPCD and never ST_CMD|I_FMTI.  Without this command the ordering is
                   only ever exercised by a random draw, which is not a floor. */
                {opc: OP.FMT, unit: 3, fmtih: 0, ref: 0x5001},
                {opc: OP.FMT, unit: 0, fmtih: 0, ref: 0x5002},
                {opc: OP.FMT, unit: 2, fmtih: FMT_IH_MAGIC, ref: 0x5003},
                {opc: OP.FMT, unit: 0, fmtih: FMT_IH_MAGIC, ref: 0x5004},
                {opc: OP.ONL, unit: 0, ref: 0x5005},
                {opc: OP.FMT, unit: 0, fmtih: FMT_IH_MAGIC, ref: 0x5006},
                {opc: OP.GUS, unit: 0, ref: 0x5007},
                {opc: OP.FMT, unit: 1, fmtih: FMT_IH_MAGIC, ref: 0x5008}
            ],
            tags: ["FMT an RD54 -> I_OPCD", "FMT an RD54 with NO magic bit -> STILL I_OPCD",
                   "FMT an RX33 with no magic bit -> I_FMTI", "FMT unattached RX33",
                   "FMT an available RX33 -> ST_SUC", "ONL it", "FMT it online -> ST_AVL|SB_AVL_INU",
                   "GUS -- the refusal took it OFFLINE", "FMT a LOCKED RX33 -> ST_WPR|SB_WPR_HW"]
        }), {
            name: "FORMAT: five refusals in order, and the one that changes the unit",
            kind: "fmt",
            units: [U({dtype: rx33, image: "tiny"}),
                    U({dtype: rx33, image: "tiny", locked: true}),
                    U({dtype: rx33}),
                    U({dtype: RD54_DTYPE, image: "mid"})],
            spread: 50
        }));
    }

    /* ---- ABORT AND GET COMMAND STATUS AGAINST IDLE UNITS.  Both exist to inspect a transfer that
       is in flight, and with none in flight both answer ST_SUC -- *** INCLUDING FOR A UNIT THAT DOES
       NOT EXIST ***, which is the one place in this whole command set where "no such unit" is not
       ST_OFL.  GET COMMAND STATUS also ZEROES two words of the host's own packet, which is the only
       visible thing it does, so a controller that skipped the else arm would return the host's data
       and look plausible. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.ABO, unit: 0, ref: 0x5100}, {opc: OP.ABO, unit: 9, ref: 0x5101},
            {opc: OP.GCS, unit: 0, ref: 0x5102}, {opc: OP.GCS, unit: 9, ref: 0x5103},
            {opc: OP.ONL, unit: 0, ref: 0x5104}, {opc: OP.ABO, unit: 0, ref: 0x5105},
            {opc: OP.GUS, unit: 0, ref: 0x5106}
        ],
        tags: ["ABO an idle drive", "ABO no such unit -- still ST_SUC", "GCS an idle drive",
               "GCS no such unit", "ONL", "ABO an online drive", "GUS -- ABORT changed nothing"]
    }), {
        name: "ABORT and GET COMMAND STATUS with nothing in flight",
        kind: "aborep", units: [U({image: "mid"})], spread: 51
    }));

    /* ---- AVAILABLE ON AN UNATTACHED UNIT SUCCEEDS.  Every other handler tests UNIT_ATT; rq_avl()
       does not, so a drive with no volume answers ST_SUC to AVAILABLE and ST_OFL|SB_OFL_NV to
       everything else.  A no-such-unit AVAILABLE is still ST_OFL, so the two are told apart. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.AVL, unit: 1, ref: 0x5200}, {opc: OP.GUS, unit: 1, ref: 0x5201},
            {opc: OP.AVL, unit: 9, ref: 0x5202},
            {opc: OP.AVL, unit: 0, mod: MD_SPD, ref: 0x5203}
        ],
        tags: ["AVL an unattached drive -> ST_SUC", "GUS it -> ST_OFL|SB_OFL_NV",
               "AVL no such unit -> ST_OFL", "AVL with MD_SPD on a NON-removable drive"]
    }), {
        name: "AVAILABLE succeeds with no volume, and MD_SPD is inert on a fixed disk",
        kind: "avlbare",
        units: [U({dtype: RD54_DTYPE, image: "mid"}), U({})], spread: 52
    }));

    /* ---- THE COMMANDS THAT NEED NO UNIT, ALONGSIDE ONES THAT DO.  SET CONTROLLER CHARACTERISTICS
       and the three no-op opcodes are pcjsvax-0b4's, and they are here for ONE reason: to prove that
       adding the unit handlers did not disturb the dispatch around them.  An illegal opcode is here
       for the same reason -- it must still be ST_CMD|I_OPCD and not some unit answer. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.SCC, unit: 0, hlnt: SCC_LNT, ref: 0x5300},
            {opc: OP.GUS, unit: 0, ref: 0x5301},
            {opc: OP.CCD, unit: 0, ref: 0x5302},
            {opc: OP.FLU, unit: 9, ref: 0x5303},
            {opc: OP.ERG, unit: 0, ref: 0x5304},
            {opc: 0x7E, unit: 0, ref: 0x5305}
        ],
        tags: ["SCC", "GUS", "CCD nop", "FLU nop on no such unit", "ERG -- not dispatched",
               "an unassigned opcode"]
    }), {
        name: "the unit-free commands still answer exactly as they did",
        kind: "unitfree", units: [U({image: "odd"})], spread: 53
    }));

    /* ---- A SCATTERED, HIGH COMM REGION WITH A PURGE INTERRUPT.  Everything above uses comm =
       0x2000; this one puts the whole apparatus above 64KB so the step-3 address shift is exercised
       under a unit command, and turns on the purge-interrupt flag so rq_step4() starts zeroing eight
       bytes lower.  The unit answers must not care, and that is what is being said. ---- */
    add(Object.assign(transaction({
        cmds: [
            {opc: OP.GUS, unit: 0, ref: 0x5400}, {opc: OP.ONL, unit: 0, ref: 0x5401},
            {opc: OP.SUC, unit: 0, ufl: UF_CMW, mod: MD_SWP, ref: 0x5402}
        ],
        tags: ["GUS", "ONL", "SUC"]
    }), {
        name: "a scattered comm region above 64KB with the purge-interrupt flag",
        kind: "highcomm",
        units: [U({dtype: DRV_TAB.findIndex((d) => d.name === "RA92"), image: "mid"})],
        comm: 0x0A3000, prgi: 1, spread: 54
    }));

    return cases;
}

/**
 * randomCases(n, seed, startIdx)
 *
 * A structurally different view from the enumerated matrix, which is exhaustive at named boundaries
 * and blind between them: uniform draws over the drive types, the containers, the lock modes, the
 * plugs, the ring lengths, the comm region, the map scatter and each command's opcode, unit number
 * and modifier.
 *
 * FENCED, NOT FILTERED, in three places:
 *   - the opcode pool EXCLUDES the five transfer commands by DERIVING the complement from rq.js's
 *     own handler map, so an opcode that changed class would change the pool;
 *   - MD_SPD is never drawn for OP_AVL, because MD_SPD and MD_NXU ARE THE SAME BIT and an
 *     AVAILABLE carrying it on a removable drive would eject the container;
 *   - SA_S1H_VEC is structurally zero because buildCase() never sets it.
 * assertExclusions() re-checks the resulting case list rather than trusting the draws.
 */
function randomCases(n, seed, startIdx)
{
    let rnd = mulberry32(seed);
    let pool = [];
    for (let v = 0; v <= 0xFF; v++) {
        let nm = RQVAX.OP_NAME_OF[v];
        if (nm && RQVAX.MSCP_XFER_OPS.indexOf(nm) >= 0) continue;
        pool.push(v);
    }
    let out = [];
    for (let i = 0; i < n; i++) {
        let units = [];
        /* One of four LEGAL plug assignments, so plug != index is common rather than exceptional
           and max_plug moves from case to case.  LEGAL is not a formality: plugs 4..253 are held by
           the 250 disabled potential drives the DEVICE declares (see rq.js's setPlug()), so the only
           values a case may use are a permutation of 0..3 and anything at or above RQ_MAXDR. */
        let plugs = [[0, 1, 2, 3], [3, 2, 1, 0], [1, 0, 3, 2],
                     [0, PLUG_HIGH, 2, PLUG_HIGH + 1]][Math.floor(rnd() * 4)].slice();
        for (let k = 0; k < RQ_NUMDR; k++) {
            let attached = rnd() < 0.7;
            let dtype = Math.floor(rnd() * DRV_TAB.length);
            units.push(unitSpec({
                dtype, plug: plugs[k],
                image: attached ? IMAGES[Math.floor(rnd() * IMAGES.length)].tag : null,
                locked: attached && rnd() < 0.2,
                ro: attached && rnd() < 0.2,
                disabled: !attached && rnd() < 0.15
            }));
        }
        let nc = 1 + Math.floor(rnd() * 6);
        let cmds = [], tags = [];
        for (let k = 0; k < nc; k++) {
            let opc = pool[Math.floor(rnd() * pool.length)];
            let mod = Math.floor(rnd() * 0x10000);
            if (opc === OP.AVL) mod = mod & ~MD_SPD;        /* never eject the container */
            cmds.push({
                opc, mod, unit: Math.floor(rnd() * 10),
                ufl: Math.floor(rnd() * 0x10000),
                fmtih: Math.floor(rnd() * 0x10000),
                ref: Math.floor(rnd() * 0x10000),
                hlnt: [RSP_LNT, GUS_LNT_D, ONL_LNT, 64][Math.floor(rnd() * 4)]
            });
            tags.push(`r#${k} op=${hex(opc, 2)}`);
        }
        let c = buildCase(Object.assign(transaction({cmds, tags}), {
            name: `random#${i} n=${nc} plugs=${plugs.join(",")}`,
            kind: "random", units,
            comm: ((1 + Math.floor(rnd() * 0x400)) * PAGE) & 0x3FFE00,
            prgi: rnd() < 0.5 ? 1 : 0,
            spread: Math.floor(rnd() * DATA_NPAGE),
            qtime: [RQ_QTIME, 60, 250][Math.floor(rnd() * 3)]
        }));
        c.idx = startIdx + out.length;
        out.push(c);
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * The SIMH side                                                                                 *
 * ------------------------------------------------------------------------------------------- */

const MARK = "MSUCASE";

/** PHASE T -- every drive type, cross-checked against the simulator's own `set`/`show`. */
function runPhaseT(simh, opts)
{
    let L = [`set cpu ${MEM_MB}m`, "set rq rqdx3"];
    for (let i = 0; i < DRV_TAB.length; i++) {
        L.push(`echo DRVTYPE${i}`);
        L.push("detach rq0", `set rq0 ${DRV_TAB[RD54_DTYPE].name}`, "set rq0 writeenabled");
        L.push(`set rq0 ${DRV_TAB[i].name}`);
        L.push("examine -h rq capac[0]");
        L.push("show rq0");
    }
    /* The pseudo-unit fence, asked of the simulator rather than assumed: rq_reg[]'s per-unit arrays
       are RQ_NUMDR deep, so the index one past the last drive must be refused.  If a future SIMH
       published RQ_TIMER and RQ_QUEUE as units this would start answering and the run must fail. */
    L.push("echo PSEUDOUNITS", `examine -h rq capac[${RQ_NUMDR}]`, "echo ENDPSEUDO");
    L.push("exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "mscpunit-types.ini"));
    let rows = [];
    let parts = out.split(/^DRVTYPE(\d+)\s*$/m);
    for (let i = 1; i < parts.length; i += 2) {
        let chunk = parts[i + 1] || "";
        let cap = /^CAPAC\[0\]:\s*([0-9A-Fa-f]+)/m.exec(chunk);
        /* rq_show_type() prints the drive name first on the unit's SECOND line, followed by the
           plug and the autosize state: `        RD54, UNIT=0, autosize`. */
        let nm = /^\s+([A-Z0-9]+), UNIT=0, (autosize|noautosize)\s*$/m.exec(chunk);
        rows[+parts[i]] = {capac: cap ? parseInt(cap[1], 16) >>> 0 : null,
                           name: nm ? nm[1] : null, auto: nm ? nm[2] : null};
    }
    let pseudo = /^PSEUDOUNITS\n([\s\S]*?)^ENDPSEUDO$/m.exec(out);
    return {rows, pseudo: pseudo ? pseudo[1] : null};
}

/** PHASE A -- the autosize arithmetic, swept with no CPU at all. */
function attachSweep()
{
    let pts = [];
    for (let d = 0; d < DRV_TAB.length; d++) {
        for (let im of IMAGES) {
            for (let mode of ["rw", "locked", "switchR", "reattach"]) {
                pts.push({dtype: d, image: im.tag, mode});
            }
        }
    }
    return pts;
}

function runPhaseA(simh, opts, images, pts)
{
    let L = [`set cpu ${MEM_MB}m`, "set rq rqdx3"];
    for (let k = 0; k < pts.length; k++) {
        let p = pts[k], im = images.find((x) => x.tag === p.image);
        L.push(`echo ATT${k}`);
        L.push("detach rq0", `set rq0 ${DRV_TAB[RD54_DTYPE].name}`, "set rq0 writeenabled");
        L.push(`set rq0 ${DRV_TAB[p.dtype].name}`);
        if (p.mode === "locked") L.push("set rq0 locked");
        /* THE REATTACH POINT: a read-only attach, a detach, and a PLAIN attach with NOTHING in
           between.  detach_unit() clears UNIT_RO for a UNIT_ROABLE device, so the second attach is
           writable and autosize's clamp applies again -- and a model whose detach kept UNIT_RO
           would take the file's own size instead.  There is deliberately no `set writeenabled`
           between the two: that would clear UNIT_RO by another route and hide the behaviour, which
           is exactly what every other point in this sweep does and why none of them could see it. */
        if (p.mode === "reattach") {
            L.push(`attach -R rq0 ${im.path}`);
            L.push("detach rq0");
            /* *** THE `set` IS LOAD-BEARING AND ITS ABSENCE MADE THIS POINT MEASURE NOTHING. ***
               detach does not restore `capac`, so without this the second attach's "current unit
               size" is the FIRST attach's answer and the clamp can never fire -- the point returned
               the same number whether or not UNIT_RO survived the detach.  Re-setting the type puts
               `capac` back to the drive's own LBN count, which is what makes the clamp a decision
               again.  There is still deliberately no `set writeenabled` here: that would clear
               UNIT_RO by another route and hide the very thing this mode exists to see. */
            L.push(`set rq0 ${DRV_TAB[p.dtype].name}`);
        }
        L.push(`attach ${p.mode === "switchR" ? "-R " : ""}rq0 ${im.path}`);
        L.push("examine -h rq capac[0]");
    }
    L.push("detach rq0", "exit", "");
    let out = runSimh(simh, L.join("\n"), path.join(opts.scratch, "mscpunit-attach.ini"));
    let caps = [];
    let parts = out.split(/^ATT(\d+)\s*$/m);
    for (let i = 1; i < parts.length; i += 2) {
        let m = /^CAPAC\[0\]:\s*([0-9A-Fa-f]+)/m.exec(parts[i + 1] || "");
        caps[+parts[i]] = m ? parseInt(m[1], 16) >>> 0 : null;
    }
    return caps;
}

function simhCaseLines(c, images)
{
    let L = [];
    L.push(`echo ${MARK}${c.idx}`);
    L.push(...simhSetupLines(c, images));
    L.push(...simhResetLines(c));
    L.push(...simhAttachLines(c, images));
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
    for (let o of UNIT_OBS) {
        for (let n = 0; n < RQ_NUMDR; n++) L.push(`examine -h rq ${o.name}[${n}]`);
    }
    for (let i of PKT_PROBES) L.push(`examine -h rq pkts[${i}]`);
    for (let p of c.dumpPages) L.push(`examine -h ${hex(p * PAGE)}:${hex(p * PAGE + PAGE - 4)}`);
    L.push(`examine -h ${hex(c.resultPage * PAGE)}:${hex(c.resultPage * PAGE + PAGE - 4)}`);
    L.push("echo RINGS", "show rq rings", "echo FREEQ", "show rq freeq", "echo RESPQ", "show rq respq",
           "echo ENDCASE");
    return L;
}

function runCasesSimh(simh, opts, images, cases)
{
    let L = [`set cpu ${MEM_MB}m`, "set cpu simhalt", "set rq rqdx3",
             "set debug stdout", "set rq debug=REQ"];
    for (let c of cases) L.push(...simhCaseLines(c, images));
    L.push(...Array.from({length: RQ_NUMDR}, (_, i) => `detach rq${i}`));
    L.push("exit", "");
    let script = L.join("\n");
    let out = runSimh(simh, script, path.join(opts.scratch, "mscpunit-cases.ini"));

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
    let re = /^([0-9A-F]{6,8}):\s*([0-9A-F]{8})\s*$/gm, m;
    while ((m = re.exec(chunk)) !== null) r.mem.set(parseInt(m[1], 16) >>> 0, parseInt(m[2], 16) >>> 0);
    /* *** SIMH COLLAPSES CONSECUTIVE IDENTICAL DEBUG LINES *** (scp.c:13836-13900): a run of N+1
       identical lines is printed as the first one followed by `same as above (N times)`, stamped
       with the time of the LAST occurrence, and the collapse line carries NO DEVICE TAG -- so a
       parser anchored on `RQ REQ:` reads N retries as one.  Expanded here, exactly as
       mscpringdiff.js does; only the ORDER and TEXT of the trace are graded, because the individual
       timestamps of a repeated event do not exist in the oracle's output at all. */
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
 * The JS side                                                                                   *
 * ------------------------------------------------------------------------------------------- */

function runCaseJS(c, providers, mutationOpts = {})
{
    let m = machine(mutationOpts);
    let {bus, cpu, cqbic, rq} = m;

    /* The SAME split the do-file has: setup, then the reset, then the attach.  See simhSetupLines(). */
    jsSetupUnits(rq, c);
    jsResetForCase(m, c);
    jsAttachUnits(rq, c, providers);

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
        rq: {}, units: [], mem: new Map(), unimplemented,
        halted, atOwnHalt: halted && (cpu.regs[15] >>> 0) === c.haltPC
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
    r.trace = rq.reqLog.slice();
    r.rings = showCtrl(rq, cqbic, "RI");
    r.freeq = showCtrl(rq, cqbic, "FR");
    r.respq = showCtrl(rq, cqbic, "RS");
    sampleHeap();
    return r;
}

/** PHASE A's JS half: the same `set`/`attach` sequence against a FRESH-STATE controller, using the
    graded machine's own instance (standing rule 14 -- one machine, reused). */
function runAttachSweepJS(pts, providers, mutationOpts = {})
{
    let rq = machine(mutationOpts).rq;
    let out = [];
    for (let p of pts) {
        rq.detach(0);
        rq.setType(0, RD54_DTYPE);
        rq.setWriteLock(0, false);
        rq.setType(0, p.dtype);
        if (p.mode === "locked") rq.setWriteLock(0, true);
        if (p.mode === "reattach") {
            rq.attach(0, providers[p.image], {readOnly: true});
            rq.detach(0);
            rq.setType(0, p.dtype);                         /* see simhAttachLines(): load-bearing */
        }
        rq.attach(0, providers[p.image], {readOnly: p.mode === "switchR"});
        out.push(rq.units[0].capac >>> 0);
    }
    rq.detach(0);
    rq.setType(0, RD54_DTYPE);
    rq.setWriteLock(0, false);
    sampleHeap();
    return out;
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
 * The ordered `set rq debug=REQ` stream, compared as a SEQUENCE.  It is the only one of this file's
 * views that can distinguish "the right answers" from "the right answers in the right ORDER", and it
 * is where a unit command's `rsp=`/`sts=` line appears -- so a status word that is right in memory
 * and wrong in the controller's own copy fails here.
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
 * PHASE T / PHASE A grading                                                                     *
 * ------------------------------------------------------------------------------------------- */

function gradeTypes(t, failures, acc)
{
    for (let i = 0; i < DRV_TAB.length; i++) {
        let row = t.rows[i], d = DRV_TAB[i];
        if (!row || row.capac === null || row.name === null) {
            failures.push(`PHASE T: the oracle gave no readable answer for \`set rq0 ${d.name}\` -- ` +
                `drive type ${i} is UNMEASURED, which is not the same as agreeing`);
            continue;
        }
        if (row.name !== d.name) {
            failures.push(`PHASE T: \`set rq0 ${d.name}\` leaves the oracle showing ${row.name}, so ` +
                `drv_tab[${i}]'s NAME as derived from pdp11_rq.c is not the name the simulator uses`);
        }
        if (row.capac !== (d.lbn >>> 0)) {
            failures.push(`PHASE T: \`set rq0 ${d.name}\` leaves CAPAC[0] = ${row.capac} and ` +
                `drv_tab[${i}].lbn is ${d.lbn} -- the extracted table and the running simulator ` +
                `disagree about how big this drive is`);
        }
        if (row.auto !== "autosize") {
            failures.push(`PHASE T: \`set rq0 ${d.name}\` leaves the unit ${row.auto}, not autosize`);
        }
        acc.typesMeasured++;
    }
    if (t.pseudo === null || !/Subscript out of range/.test(t.pseudo)) {
        failures.push(`PHASE T: \`examine rq capac[${RQ_NUMDR}]\` did NOT answer "Subscript out of ` +
            `range" -- rq_reg[]'s per-unit arrays are supposed to be exactly RQ_NUMDR deep, and if ` +
            `the simulator now publishes the RQ_TIMER / RQ_QUEUE pseudo-units as drives then the ` +
            `unit list this differential grades is the wrong length.  Got: ` +
            `${JSON.stringify((t.pseudo || "").trim()).slice(0, 200)}`);
    }
}

function gradeAttach(pts, simCaps, jsCaps, failures, acc)
{
    for (let k = 0; k < pts.length; k++) {
        let p = pts[k];
        let what = `${DRV_TAB[p.dtype].name} + ${p.image} (${p.mode})`;
        if (simCaps[k] === null || simCaps[k] === undefined) {
            failures.push(`PHASE A: the oracle gave no CAPAC for ${what} -- that point is UNMEASURED`);
            continue;
        }
        acc.attachMeasured++;
        acc.attachCaps.add(simCaps[k]);
        if (simCaps[k] !== jsCaps[k]) {
            failures.push(`PHASE A: attaching ${what} gives capac ${jsCaps[k]} here and ` +
                `${simCaps[k]} on the oracle (the container is ` +
                `${IMAGES.find((x) => x.tag === p.image).bytes} bytes = ` +
                `${Math.floor(IMAGES.find((x) => x.tag === p.image).bytes / RQ_NUMBY)} blocks; the ` +
                `drive is ${DRV_TAB[p.dtype].lbn} blocks)`);
        }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * Coverage floors and exclusion fences.  Every one FAILS the run; none scales with case count.   *
 * ------------------------------------------------------------------------------------------- */

function assertExclusions(cases, sim, failures)
{
    for (let c of cases) {
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
        for (let u of c.units) {
            if (u.disabled && u.image) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" disables a unit it also attaches ` +
                    `-- SCP refuses that and neither engine has the state`);
            }
        }
        for (let k of Object.keys(c.packets)) {
            let w = c.packets[k];
            let opc = w[CMD_OPC] & 0xFF;
            let nm = RQVAX.OP_NAME_OF[opc];
            if (nm && RQVAX.MSCP_XFER_OPS.indexOf(nm) >= 0) {
                failures.push(`exclusion: case ${c.idx} "${c.name}" plants a command packet with ` +
                    `opcode OP_${nm} (${opc}), which rq_mscp() dispatches to rq_rw() -- data ` +
                    `transfer is pcjsvax-346's work and rq.js throws by name`);
            }
            /* MD_SPD AND MD_NXU ARE THE SAME BIT.  An AVAILABLE carrying it against a REMOVABLE unit
               calls sim_disk_unload() and ejects the user's container; a GET UNIT STATUS carrying it
               is a unit walk and is graded.  So the fence is per-opcode, and it has to resolve the
               unit the same way rq_getucb() does -- by PLUG, not by index. */
            if (opc === OP.AVL && (w[CMD_MOD] & MD_SPD)) {
                let u = c.units.find((x) => x.plug === (w[CMD_UN] & 0xFFFF) && !x.disabled);
                if (u && (DRV_TAB[u.dtype].flgs & RQDF_RMV)) {
                    failures.push(`exclusion: case ${c.idx} "${c.name}" sends AVAILABLE with MD_SPD ` +
                        `to plug ${w[CMD_UN]}, a ${DRV_TAB[u.dtype].name} -- RQDF_RMV, so the C ` +
                        `calls sim_disk_unload() and EJECTS the user's container`);
                }
            }
        }
    }
    for (let i = 0; i < cases.length; i++) {
        let s = sim[i];
        if (!s) continue;
        /* rq_tmrsvc() decrements HAT once per WALL-CLOCK second.  A completed command legitimately
           sets HAT to 0 (in flight) or to HTMO (idle); anything between is the timer having fired,
           and the timer is also the only reader of UNIT_ATP. */
        if (s.rq.HAT !== 0 && s.rq.HAT !== s.rq.HTMO) {
            failures.push(`exclusion: case ${cases[i].idx} "${cases[i].name}" left the ORACLE with ` +
                `HAT=${s.rq.HAT}, which is neither 0 nor HTMO=${s.rq.HTMO} -- rq_tmrsvc(), the ` +
                `once-per-second WALL-CLOCK host-access timer, fired.  It is not modelled, and it ` +
                `is also what would have delivered the UNIT_ATP attention this item does not grade.`);
        }
        /* THE FENCE FOR THE EXCLUDED ARMS, ASSERTED ON THE ORACLE.  rq_abo()'s and rq_gcs()'s packet
           searches and the four deferrals are unreachable exactly while every unit is idle, and
           "every unit is idle" is a thing the simulator can be asked. */
        for (let n = 0; n < RQ_NUMDR; n++) {
            if (s.units[n].CPKT !== 0 || s.units[n].PKTQ !== 0) {
                failures.push(`exclusion: case ${cases[i].idx} "${cases[i].name}" left the ORACLE's ` +
                    `unit ${n} holding CPKT=${s.units[n].CPKT} PKTQ=${s.units[n].PKTQ} -- a packet ` +
                    `on a unit means a transfer, which is pcjsvax-346's, and it also makes rq_abo()` +
                    `/rq_gcs()'s search arms reachable when this file says they are not`);
            }
        }
    }
}

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
                failures.push(`case ${c.idx} "${c.name}": the in-band wait for ${st.what} EXHAUSTED ` +
                    `its ${AWAIT_LIMIT}-iteration budget on the ORACLE -- the response never ` +
                    `arrived, so this case grades nothing about when it did`);
            } else {
                acc.waitCounts.add(count);
                if (count > 0) acc.nonZeroWaits++; else acc.zeroWaits++;
            }
        }
    }
}

function coverage(cases, sim, js, failures, acc)
{
    let ok = (i) => sim[i] && js[i] && !js[i].unimplemented && sim[i].halted && js[i].halted &&
                    sim[i].atOwnHalt && js[i].atOwnHalt;

    /* THE SYNCHRONY FLOOR (inherited from pcjsvax-0b4 and re-asserted, not borrowed): at least three
       DISTINCT in-band iteration counts must have been observed and matched, and not all of them may
       be zero.  A controller answering inside the IP read produces zero for every wait. */
    if (acc.waitCounts.size < 3) {
        failures.push(`coverage: only ${acc.waitCounts.size} distinct in-band response-wait ITERATION ` +
            `COUNT(s) reached comparison; the floor is 3`);
    }
    if (acc.nonZeroWaits === 0) {
        failures.push(`coverage: every graded response wait saw its answer on the FIRST iteration -- ` +
            `the controller is answering synchronously inside the IP read`);
    }

    /* THE DRIVE-TABLE FLOOR.  Every entry must have been cross-checked against the simulator in
       PHASE T, and at least three DISTINCT entries must have reached comparison through real MSCP
       RESPONSES in PHASE C -- three, because two cannot tell a table lookup from an if/else. */
    if (acc.typesMeasured !== DRV_TAB.length) {
        failures.push(`coverage: PHASE T measured ${acc.typesMeasured} of ${DRV_TAB.length} drive ` +
            `types on the oracle`);
    }
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let u of cases[i].units) if (u.image) acc.dtypesGraded.add(u.dtype);
    }
    if (acc.dtypesGraded.size < 3) {
        failures.push(`coverage: only ${acc.dtypesGraded.size} distinct drv_tab[] entry/entries ` +
            `reached comparison through an attached unit in a graded case; the floor is 3, because ` +
            `fewer cannot tell a TABLE from a special case for the drive the scratch image happens ` +
            `to suit -- which is this item's named cheat`);
    }

    /* THE CAPACITY FLOOR.  At least three DISTINCT unit capacities produced BY THREE DIFFERENT FILE
       SIZES, read off the ORACLE, and at least one of them from a container that is neither a whole
       number of blocks nor any drive's own size.  This is the item's "three geometries from three
       file sizes" floor, expressed in the mechanism the simulator actually has (see the header). */
    let byImage = new Map();
    for (let k = 0; k < acc.attachPts.length; k++) {
        let p = acc.attachPts[k], cap = acc.attachSim[k];
        if (cap === null || cap === undefined) continue;
        if (!byImage.has(p.image)) byImage.set(p.image, new Set());
        byImage.get(p.image).add(cap);
    }
    let fromFile = new Set();
    for (let im of IMAGES) {
        let blocks = Math.floor(im.bytes / RQ_NUMBY);
        for (let [tag, caps] of byImage) {
            if (tag === im.tag && caps.has(blocks)) fromFile.add(blocks);
        }
    }
    if (fromFile.size < 3) {
        failures.push(`coverage: only ${fromFile.size} distinct unit capacit(ies) were produced BY ` +
            `THE CONTAINER'S OWN SIZE on the oracle ({${[...fromFile].join(",")}}); the floor is 3 ` +
            `from three different files, because fewer cannot tell autosize from a constant`);
    }
    let notWhole = IMAGES.filter((im) => im.bytes % RQ_NUMBY !== 0)
                         .filter((im) => fromFile.has(Math.floor(im.bytes / RQ_NUMBY)));
    if (!notWhole.length) {
        failures.push(`coverage: no graded container that is NOT a whole number of ${RQ_NUMBY}-byte ` +
            `blocks ever determined a unit's capacity, so the truncating division is unexercised`);
    }
    let tableSizes = new Set(DRV_TAB.map((d) => d.lbn >>> 0));
    if (![...fromFile].some((b) => !tableSizes.has(b))) {
        failures.push(`coverage: every capacity a container determined is also some drv_tab[] ` +
            `entry's own size, so a controller that answered from the TABLE rather than from the ` +
            `FILE would satisfy every one of them`);
    }

    /* THE STATUS FLOOR.  The named answers must have been READ OUT OF THE ORACLE's trace, not
       predicted here.  Each is a different arm of a different handler and all of them are reachable
       only with the right unit state, so this is a statement about the case list too. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        for (let e of sim[i].trace) {
            let m = /^rsp=([0-9A-F]{4}), sts=([0-9A-F]{4})$/.exec(e.line);
            if (m) { acc.rspOpc.add(m[1]); acc.rspSts.add(parseInt(m[2], 16)); }
            if (/^cmd=/.test(e.line)) acc.cmdLines++;
        }
    }
    let SB_OFL_NV = 1 << ST.V_SUB, SB_SUC_ON = 8 << ST.V_SUB;
    let SB_AVL_INU = 32 << ST.V_SUB, SB_WPR_HW = 256 << ST.V_SUB;
    for (let [v, what] of [
        [ST.SUC, "ST_SUC (a command that simply worked)"],
        [ST.AVL, "ST_AVL (an attached unit that is not online)"],
        [ST.OFL, "ST_OFL, BARE -- no such unit"],
        [ST.OFL | SB_OFL_NV, "ST_OFL|SB_OFL_NV -- the unit exists and has no volume"],
        [ST.SUC | SB_SUC_ON, "ST_SUC|SB_SUC_ON -- ONLINE on an already-online unit"],
        [ST.AVL | SB_AVL_INU, "ST_AVL|SB_AVL_INU -- FORMAT refusing an online unit"],
        [ST.WPR | SB_WPR_HW, "ST_WPR|SB_WPR_HW -- FORMAT refusing a write-protected unit"]
    ]) {
        if (!acc.rspSts.has(v)) {
            failures.push(`coverage: the oracle never answered ${what} (status 0x${hex(v, 4)}) in any ` +
                `graded case, so that arm is unexercised`);
        }
    }
    if (acc.cmdLines < 80) {
        failures.push(`coverage: the oracle's trace carried only ${acc.cmdLines} command line(s); a ` +
            `case list this size must issue far more, so most cases never reached a command`);
    }

    /* THE LONG/SHORT FORM FLOOR -- REWRITTEN, AND THE REWRITE IS THE POINT.
       *** GUS_TRK/GUS_GRP/GUS_CYL/GUS_UVER ARE WORDS 20..23, THE SAME WORDS AS
       ONL_SIZL/ONL_SIZH/ONL_VSNL/ONL_VSNH. ***  So the two forms are not "with or without the size":
       an ONLINE response carries the unit's CAPACITY and volume serial in those words and a GET UNIT
       STATUS response carries the drive's GEOMETRY in the very same ones.  The floor therefore reads
       the oracle's own answer out of the response buffer and requires BOTH shapes to have been seen:
       a longword that IS the capacity (read from the oracle's CAPAC register, not predicted here)
       and a longword that IS (tpg << 16) | sect (read from the derived and PHASE-T-cross-checked
       drive table).
       The first version of this floor compared word 20 against the host's SEED and "passed" -- but
       the only responses that leave the seed there are the ones that describe NO UNIT AT ALL, so it
       was measuring the ST_OFL path and reporting it as the short form.  HANDOFF.md standing rule
       16: when a check is satisfied, satisfy yourself that the observation channel is sound. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        let c = cases[i];
        for (let k of Object.keys(c.packets)) {
            let opc = c.packets[k][CMD_OPC] & 0xFF;
            if (opc !== OP.GUS && opc !== OP.ONL && opc !== OP.SUC) continue;
            /* Resolve the unit the way rq_getucb() does -- BY PLUG, skipping disabled drives -- and
               only count responses that described a real, ATTACHED drive. */
            let lu = c.packets[k][CMD_UN] & 0xFFFF;
            let n = c.units.findIndex((x) => x.plug === lu && !x.disabled);
            if (n < 0 || !c.units[n].image) continue;
            let qa = (c.g.rspBuf(+k) + ONL_SIZL * 2) >>> 0;
            if (!c.qToP.has((qa / PAGE) | 0)) continue;
            /* Word 20 is byte 40 of a packet that starts on a 64-byte boundary, so this is an
               ALIGNED longword wholly inside one Qbus page -- not a splice of two scattered pages. */
            let v = sim[i].mem.get(c.phys(qa));
            if (v === undefined) continue;
            let d = DRV_TAB[c.units[n].dtype];
            if (opc === OP.GUS) {
                if (v === ((((d.tpg & 0xFFFF) << 16) | (d.sect & 0xFFFF)) >>> 0)) acc.shortForm = true;
            } else if (v === (sim[i].units[n].CAPAC >>> 0)) {
                acc.longForm = true;
            }
        }
    }
    if (!acc.shortForm) {
        failures.push(`coverage: no graded GET UNIT STATUS response carried the drive's ` +
            `(tpg << 16) | sect in words 20/21 on the oracle, so rq_gus()'s geometry -- which ` +
            `OVERWRITES rq_putr_unit()'s output in exactly those words -- is unexercised`);
    }
    if (!acc.longForm) {
        failures.push(`coverage: no graded ONLINE or SET UNIT CHARACTERISTICS response carried the ` +
            `unit's own CAPACITY in words 20/21 on the oracle, so rq_putr_unit()'s LONG form -- the ` +
            `only place a host learns how big the user's image is -- is unexercised`);
    }

    /* THE UNIT-FLAG FLOOR.  UF_WPH must have been seen SET and CLEAR on the oracle -- in the ONL_UFL
       word of a real response, not in a register -- and likewise UF_RMV.  Each of the four is a
       different predicate over the drive table and the unit's own state. */
    for (let i = 0; i < cases.length; i++) {
        if (!ok(i)) continue;
        let c = cases[i];
        for (let k of Object.keys(c.packets)) {
            let opc = c.packets[k][CMD_OPC] & 0xFF;
            if (opc !== OP.GUS && opc !== OP.ONL && opc !== OP.SUC) continue;
            let qa = (c.g.rspBuf(+k) + ONL_UFL * 2) >>> 0;
            if (!c.qToP.has((qa / PAGE) | 0)) continue;
            let lw = sim[i].mem.get(c.phys((qa - 2) >>> 0));     /* ONL_MLUN(8)/ONL_UFL(9) longword */
            if (lw === undefined) continue;
            let ufl = (lw >>> 16) & 0xFFFF;
            /* Only count responses that actually described a unit: rq_putr_unit() is skipped
               entirely on the ST_OFL path, and the seed's own bits would otherwise be read as flags.
               UF_RPL is OR-ed into every real unit description and into nothing else, so it is the
               marker that says "this word came from rq_putr_unit()". */
            if (!(ufl & UF_RPL)) continue;
            if (ufl & UF_WPH) acc.wphSet = true; else acc.wphClear = true;
            if (ufl & UF_RMV) acc.rmvSet = true; else acc.rmvClear = true;
            if (ufl & UF_WPS) acc.wpsSet = true;
        }
    }
    for (let [got, what] of [
        [acc.wphSet, "UF_WPH SET (a write-protected unit)"],
        [acc.wphClear, "UF_WPH CLEAR (a writable unit)"],
        [acc.rmvSet, "UF_RMV SET (a removable drive type)"],
        [acc.rmvClear, "UF_RMV CLEAR (a fixed drive type)"],
        [acc.wpsSet, "UF_WPS SET (software write protect, which needs BOTH the host's bit and MD_SWP)"]
    ]) {
        if (!got) {
            failures.push(`coverage: the oracle never reported ${what} in a graded unit description, ` +
                `so that predicate is indistinguishable from a constant`);
        }
    }

    /* Each named behaviour must have been OBSERVED on the oracle at least once. */
    let sawKind = (k) => cases.some((c, i) => ok(i) && c.kind === k);
    for (let [k, what] of [
        ["drvtab",   "the whole drive table through real GET UNIT STATUS / ONLINE responses"],
        ["ladder",   "the AVAILABLE -> ONLINE -> already-ONLINE -> AVAILABLE ladder"],
        ["offline",  "attached / unattached / disabled / no-such-unit told apart"],
        ["sucflags", "SET UNIT CHARACTERISTICS's UF_MSK mask and MD_SWP condition"],
        ["sucavail", "SET UNIT CHARACTERISTICS on a unit that is not online"],
        ["wprot",    "UF_WPH from a lock, from -R and from a read-only drive type"],
        ["sizes",    "three container sizes on one drive type"],
        ["clamp",    "autosize's clamp and the read-only exception to it"],
        ["nxu",      "GET UNIT STATUS with MD_NXU, including off the end of the unit list"],
        ["nxudis",   "a disabled drive moving MD_NXU's wrap point"],
        ["plugs",    "unit plugs that are not array indices"],
        ["fmt",      "FORMAT's five refusals"],
        ["aborep",   "ABORT and GET COMMAND STATUS with nothing in flight"],
        ["avlbare",  "AVAILABLE on a unit with no volume"],
        ["unitfree", "the unit-free commands still answering as they did"],
        ["highcomm", "a scattered comm region above 64KB"],
        ["random",   "the randomized phase"]
    ]) {
        if (!sawKind(k)) failures.push(`coverage: no graded case exercised ${what}`);
    }
}

/* ------------------------------------------------------------------------------------------- *
 * MUTATIONS -- each PERTURBS the shipped path, never substitutes a copy of it (rule 11)         *
 *                                                                                               *
 * *** HANDOFF.md STANDING RULE 16: A MUTATION CAN BE CAUGHT BY A CHECK THAT IS ITSELF BROKEN. ***  *
 * The sibling differential printed CAUGHT for four literal no-ops because its observation channel  *
 * -- an UNALIGNED longword spanning two Qbus pages -- returned run-order residue.  Every read this  *
 * file makes was chosen against that: the per-unit registers come from the simulator's own          *
 * rq_reg[]; the response fields are read as WHOLE PHYSICAL PAGES, longword-aligned, never across a  *
 * page boundary (the ONL_MLUN/ONL_UFL pair above is one aligned longword by construction, word 8    *
 * and word 9 of a 64-byte packet that starts on a 64-byte boundary); the trace is text; and PHASE  *
 * A involves no memory at all.  A mutation below that is caught ONLY by a memory comparison and by  *
 * nothing else deserves a second look.                                                             *
 * ------------------------------------------------------------------------------------------- */

const MUTATIONS = {
    /* --- the five the item names --- */
    "media-identifier-byte-order-swapped": () => {
        let orig = RQVAX.prototype.putp32;
        RQVAX.prototype.putp32 = function(pkt, w, v) {
            return orig.call(this, pkt, w, (((v & 0xFFFF) << 16) | ((v >>> 16) & 0xFFFF)) >>> 0);
        };
        return () => { RQVAX.prototype.putp32 = orig; };
    },
    "the-LBN-count-is-reported-in-BYTES-not-blocks": () => {
        /* The unit's capacity is in BLOCKS and ONL_SIZL says so.  Multiplying by RQ_NUMBY produces a
           number that is right about the disk and wrong about the field -- and every geometry word
           around it stays correct, which is why only a comparison of THIS longword catches it. */
        let orig = RQVAX.prototype.putrUnit;
        RQVAX.prototype.putrUnit = function(pkt, u, lu, all) {
            let saved = u.capac;
            u.capac = (u.capac * RQ_NUMBY) >>> 0;
            try { return orig.call(this, pkt, u, lu, all); } finally { u.capac = saved; }
        };
        return () => { RQVAX.prototype.putrUnit = orig; };
    },
    "UF_RPL-not-OR-ed-into-the-unit-flags": () => {
        let orig = RQVAX.prototype.putrUnit;
        RQVAX.prototype.putrUnit = function(pkt, u, lu, all) {
            let r = orig.call(this, pkt, u, lu, all);
            this.spd(pkt, ONL_UFL, this.pd(pkt, ONL_UFL) & ~UF_RPL);
            return r;
        };
        return () => { RQVAX.prototype.putrUnit = orig; };
    },
    "the-unit-identifier-class-and-model-field-zeroed": () => {
        let orig = RQVAX.prototype.putrUnit;
        RQVAX.prototype.putrUnit = function(pkt, u, lu, all) {
            let r = orig.call(this, pkt, u, lu, all);
            this.spd(pkt, ONL_UIDD, 0);
            return r;
        };
        return () => { RQVAX.prototype.putrUnit = orig; };
    },
    "the-volume-serial-base-read-as-DECIMAL-1234": () => {
        /* 01234 IS OCTAL: 668.  Reading the C's digits as decimal is the classic transcription trap
           this item's description names, and it moves ONL_VSNL by 566 -- a difference no geometry
           field and no status word shows. */
        let orig = RQVAX.prototype.putrUnit;
        RQVAX.prototype.putrUnit = function(pkt, u, lu, all) {
            let r = orig.call(this, pkt, u, lu, all);
            if (all) this.spd(pkt, ONL_VSNL, 1234 + lu);
            return r;
        };
        return () => { RQVAX.prototype.putrUnit = orig; };
    },

    /* --- AUTOSIZE.  The item names "autosize picking the first table entry instead of the SMALLEST
       THAT FITS"; measurement (see the file header) shows the raw-container arm does not walk the
       table at all, so these four mutate the arithmetic it DOES perform.  Each is a decision the
       C makes and each changes a capacity that PHASE A reads straight out of the simulator. --- */
    "autosize-takes-the-container-size-unconditionally": () => {
        let orig = RQVAX.prototype.attach;
        RQVAX.prototype.attach = function(i, provider, opts = {}) {
            let u = this.unit(i);
            let saved = u.capac;
            u.capac = 0;                                    /* current == 0, so the clamp never fires */
            try { return orig.call(this, i, provider, opts); }
            catch (e) { u.capac = saved; throw e; }
        };
        return () => { RQVAX.prototype.attach = orig; };
    },
    "autosize-clamps-even-a-read-only-attachment": () => {
        let orig = RQVAX.prototype.attach;
        RQVAX.prototype.attach = function(i, provider, opts = {}) {
            let u = this.unit(i);
            let r = orig.call(this, i, provider, opts);
            if (u.capac * RQ_NUMBY < DRV_TAB[u.dtype].lbn * RQ_NUMBY) u.capac = DRV_TAB[u.dtype].lbn;
            return r;
        };
        return () => { RQVAX.prototype.attach = orig; };
    },
    "autosize-rounds-the-container-UP-to-a-whole-block": () => {
        let orig = RQVAX.prototype.attach;
        RQVAX.prototype.attach = function(i, provider, opts = {}) {
            let u = this.unit(i);
            let r = orig.call(this, i, provider, opts);
            let b = provider.byteLength;
            if (b % RQ_NUMBY && u.capac === Math.floor(b / RQ_NUMBY)) u.capac = u.capac + 1;
            return r;
        };
        return () => { RQVAX.prototype.attach = orig; };
    },
    "set-type-does-not-reset-the-capacity": () => {
        /* rq_set_type() installs the type's own LBN count, which is what makes the next attach's
           comparison start from the DRIVE rather than from whatever the last container left. */
        let orig = RQVAX.prototype.setType;
        RQVAX.prototype.setType = function(i, dtype) {
            let before = this.unit(i).capac;
            let r = orig.call(this, i, dtype);
            this.unit(i).capac = before;
            return r;
        };
        return () => { RQVAX.prototype.setType = orig; };
    },
    "a-write-locked-unit-still-attaches-writable": () => {
        /* set_writelock()'s "Next attach will be Read-Only" -- dropping it leaves the container
           write-enabled, which does not change UF_WPH (UNIT_WLK alone still sets it) but DOES change
           autosize, because the clamp applies again. */
        let orig = RQVAX.prototype.setWriteLock;
        RQVAX.prototype.setWriteLock = function(i, locked) {
            let r = orig.call(this, i, locked);
            if (locked && !(this.unit(i).flags & U_ATT)) this.unit(i).flags &= ~U_RO;
            return r;
        };
        return () => { RQVAX.prototype.setWriteLock = orig; };
    },
    "detach-does-not-clear-the-read-only-attachment": () => {
        let orig = RQVAX.prototype.detach;
        RQVAX.prototype.detach = function(i) {
            let was = this.unit(i).flags & U_RO;
            let r = orig.call(this, i);
            this.unit(i).flags |= was;
            return r;
        };
        return () => { RQVAX.prototype.detach = orig; };
    },

    /* --- THE UNIT LOOKUP --- */
    "getucb-matches-the-ARRAY-INDEX-instead-of-the-PLUG": () => {
        let orig = RQVAX.prototype.getucb;
        RQVAX.prototype.getucb = function(lu) {
            let u = this.units[lu];
            return (u && !(u.flags & U_DIS)) ? u : null;
        };
        return () => { RQVAX.prototype.getucb = orig; };
    },
    "getucb-ignores-the-DISABLED-flag": () => {
        let orig = RQVAX.prototype.getucb;
        RQVAX.prototype.getucb = function(lu) {
            for (let u of this.units) if (u.plug === lu) return u;
            return null;
        };
        return () => { RQVAX.prototype.getucb = orig; };
    },
    "max_plug-counts-disabled-drives": () => {
        let orig = RQVAX.prototype.reset;
        RQVAX.prototype.reset = function() {
            orig.call(this);
            for (let u of this.units) if (u.plug > this.maxPlug) this.maxPlug = u.plug;
        };
        return () => { RQVAX.prototype.reset = orig; };
    },

    /* --- THE STATUS LADDER.  Each of these produces a WELL-FORMED response with the wrong word. --- */
    "no-such-unit-answers-ST_OFL|SB_OFL_NV-like-an-empty-drive": () => {
        let orig = RQVAX.prototype.getucb;
        let SB = 1 << ST.V_SUB;
        RQVAX.prototype.getucb = function(lu) {
            let u = orig.call(this, lu);
            if (u) return u;
            /* Hand back a synthetic UNATTACHED unit, which is exactly the conflation: the C answers
               a BARE ST_OFL for a unit that is not there and ST_OFL|SB_OFL_NV for one that is. */
            return {plug: lu, dtype: RD54_DTYPE, flags: 0, uf: 0,
                    capac: DRV_TAB[RD54_DTYPE].lbn, cpkt: 0, pktq: 0, image: null};
        };
        return () => { RQVAX.prototype.getucb = orig; void SB; };
    },
    /* *** THESE FOUR PERTURB THE HANDLER WHILE IT RUNS, NOT ITS RESULT AFTERWARDS. ***  HANDOFF.md
       standing rule 11, learned here the hard way: four mutations written as "call the shipped
       handler, then fix up the packet" all SURVIVED, and the reason is that a handler's last act is
       rq_putpkt(), which has ALREADY DMAd the response into host memory by the time it returns.
       Editing `pak[]` afterwards changes a copy nothing looks at again -- the free list recycles the
       same packet for the next command, so even `examine rq pkts[N]` shows the LAST command's data.
       A mutation that runs after the write is not a perturbation of the shipped path; it is a no-op
       that prints CAUGHT if anything else happens to differ.  The technique below installs an
       OWN-PROPERTY override for the duration of the call and removes it in a `finally`, which is the
       same shape the sibling differential uses for the CQBIC's inherited mapWriteW(). */

    "ONLINE-on-an-already-online-unit-drops-SB_SUC_ON": () => {
        let origOnl = RQVAX.prototype.onl, origPutr = RQVAX.prototype.putr;
        let SB_SUC_ON = 8 << ST.V_SUB;
        RQVAX.prototype.onl = function(pkt, q) {
            this.putr = function(p, cmd, flg, sts, lnt, typ) {
                return origPutr.call(this, p, cmd, flg,
                    sts === (ST.SUC | SB_SUC_ON) ? ST.SUC : sts, lnt, typ);
            };
            try { return origOnl.call(this, pkt, q); } finally { delete this.putr; }
        };
        return () => { RQVAX.prototype.onl = origOnl; };
    },
    "ONLINE-hacks-the-flags-even-when-the-unit-was-already-online": () => {
        /* The C calls rq_setf_unit() ONLY on the arm that actually brings a unit online, so a host
           cannot software-write-protect a drive by onlining it twice.  Running it on the
           already-online arm too -- BEFORE the handler builds its response, while the host's own
           request words are still in the packet -- is the missing `else`. */
        let orig = RQVAX.prototype.onl;
        RQVAX.prototype.onl = function(pkt, q) {
            let u = this.getucb(this.pd(pkt, CMD_UN));
            if (u && (u.flags & U_ONL) && (u.flags & U_ATT)) this.setfUnit(pkt, u);
            return orig.call(this, pkt, q);
        };
        return () => { RQVAX.prototype.onl = orig; };
    },
    /* *** A MUTATION THAT IS NOT HERE: `GET-UNIT-STATUS-uses-the-LONG-form`. ***
       It was written, it was measured to SURVIVE, and the reason is worth more than the mutation:
       *** GUS_TRK, GUS_GRP, GUS_CYL AND GUS_UVER ARE WORDS 20, 21, 22 AND 23 -- THE SAME WORDS AS
       ONL_SIZL, ONL_SIZH, ONL_VSNL AND ONL_VSNH. ***  rq_gus() calls rq_putr_unit() FIRST and then
       writes the geometry OVER those four words, so forcing `all` true adds four writes that are
       overwritten before the packet leaves.  The `all` argument is therefore unobservable in the GUS
       direction, and no host program can tell a controller that passes FALSE from one that passes
       TRUE.  Leaving a mutation here that can never fail would report coverage instead of measuring
       it (HANDOFF.md standing rule 11), so it is gone and the OTHER direction -- forcing the SHORT
       form, where rq_onl() writes nothing over words 20..23 -- is the one that carries the load.
       The long/short coverage floor was rewritten for the same reason: its first version compared a
       GUS response's word 20 against the host's SEED, which only matched when the response described
       NO UNIT AT ALL, so it was measuring the ST_OFL path and calling it the short form. */
    "ONLINE-uses-the-SHORT-form": () => {
        let orig = RQVAX.prototype.putrUnit;
        RQVAX.prototype.putrUnit = function(pkt, u, lu, all) { return orig.call(this, pkt, u, lu, false); };
        return () => { RQVAX.prototype.putrUnit = orig; };
    },
    "AVAILABLE-does-not-clear-the-unit-flags": () => {
        let orig = RQVAX.prototype.avl;
        RQVAX.prototype.avl = function(pkt, q) {
            let u = this.getucb(this.pd(pkt, CMD_UN));
            let uf = u ? u.uf : 0;
            let r = orig.call(this, pkt, q);
            if (u) u.uf = uf;
            return r;
        };
        return () => { RQVAX.prototype.avl = orig; };
    },
    "AVAILABLE-does-not-take-the-unit-offline": () => {
        let orig = RQVAX.prototype.avl;
        RQVAX.prototype.avl = function(pkt, q) {
            let u = this.getucb(this.pd(pkt, CMD_UN));
            let onl = u ? (u.flags & U_ONL) : 0;
            let r = orig.call(this, pkt, q);
            if (u) u.flags |= onl;
            return r;
        };
        return () => { RQVAX.prototype.avl = orig; };
    },

    /* --- THE FLAG HACK --- */
    "rq_setf_unit-ignores-MD_SWP-and-honours-UF_WPS-anyway": () => {
        let orig = RQVAX.prototype.setfUnit;
        RQVAX.prototype.setfUnit = function(pkt, u) {
            orig.call(this, pkt, u);
            if (this.pd(pkt, ONL_UFL) & UF_WPS) u.uf |= UF_WPS;
        };
        return () => { RQVAX.prototype.setfUnit = orig; };
    },
    "rq_setf_unit-does-not-mask-the-host's-request-with-UF_MSK": () => {
        let orig = RQVAX.prototype.setfUnit;
        RQVAX.prototype.setfUnit = function(pkt, u) {
            orig.call(this, pkt, u);
            u.uf |= this.pd(pkt, ONL_UFL) & (UF_CMR | UF_CMW | UF_RMV);
        };
        return () => { RQVAX.prototype.setfUnit = orig; };
    },

    /* --- THE DRIVE TABLE'S PREDICATES --- */
    "UF_WPH-ignores-the-drive-type's-RQDF_RO": () => {
        let orig = RQVAX.prototype.wph;
        RQVAX.prototype.wph = function(u) { return (u.flags & (U_WLK | U_RO)) ? UF_WPH : 0; };
        return () => { RQVAX.prototype.wph = orig; };
    },
    "UF_RMV-reads-RQDF_RO-instead-of-RQDF_RMV": () => {
        let orig = RQVAX.prototype.rmv;
        RQVAX.prototype.rmv = function(u) { return (DRV_TAB[u.dtype].flgs & RQDF_RO) ? UF_RMV : 0; };
        return () => { RQVAX.prototype.rmv = orig; };
    },
    "the-geometry-words-are-sect/surf/cyl-rather-than-sect/tpg/gpc": () => {
        /* The C's GUS_CYL carries GROUPS PER CYLINDER and GUS_GRP carries TRACKS PER GROUP.  Filling
           them with the fields whose NAMES match is the reading a careful transcriber would produce
           and it is wrong for every drive. */
        let origGus = RQVAX.prototype.gus, origSpd = RQVAX.prototype.spd;
        RQVAX.prototype.gus = function(pkt, q) {
            let u = this.getucb(this.pd(pkt, CMD_UN));
            let d = u ? DRV_TAB[u.dtype] : null;
            this.spd = function(p, w, v) {
                if (d && p === pkt && w === 21) v = d.surf;      /* GUS_GRP: tpg -> surf */
                if (d && p === pkt && w === 22) v = d.cyl;       /* GUS_CYL: gpc -> cyl */
                return origSpd.call(this, p, w, v);
            };
            try { return origGus.call(this, pkt, q); } finally { delete this.spd; }
        };
        return () => { RQVAX.prototype.gus = origGus; };
    },
    "the-bad-block-parameters-are-computed-from-rctc-not-rcts": () => {
        /* rcts is the RCT SIZE and rctc the NUMBER OF COPIES; they disagree for seven drive types
           (RC25, RCF25, RA80 and the four ESE disks all have rcts == 0 and rctc == 1), which is why
           the drive-table sweep must cover the WHOLE table.  Perturbing rq.js's published rbpar()
           rather than rq_gus()'s result, for the reason above. */
        let orig = RQVAX.prototype.rbpar;
        RQVAX.prototype.rbpar = function(d) { return d.rctc ? 1 : 0; };
        return () => { RQVAX.prototype.rbpar = orig; };
    },

    /* --- MD_NXU --- */
    "MD_NXU-wraps-at->=-max_plug-instead-of->": () => {
        let orig = RQVAX.prototype.gus;
        RQVAX.prototype.gus = function(pkt, q) {
            if ((this.pd(pkt, CMD_MOD) & MD_NXU) && this.pd(pkt, CMD_UN) === this.maxPlug) {
                this.spd(pkt, CMD_UN, 0);
            }
            return orig.call(this, pkt, q);
        };
        return () => { RQVAX.prototype.gus = orig; };
    },
    "MD_NXU-wraps-without-rewriting-the-unit-word": () => {
        /* The C sets `d[RSP_UN] = lu` so the HOST can see that its walk wrapped; the answer itself
           is unit 0's either way, so only the returned unit number differs. */
        let origGus = RQVAX.prototype.gus, origSpd = RQVAX.prototype.spd;
        RQVAX.prototype.gus = function(pkt, q) {
            let first = true;
            this.spd = function(p, w, v) {
                /* rq_gus()'s FIRST write is `d[RSP_UN] = lu` on the wrap path and nothing else
                   writes word 4, so suppressing it leaves the host's own unit number in the
                   response while the ANSWER is still unit 0's. */
                if (p === pkt && w === 4 && first) { first = false; return; }
                return origSpd.call(this, p, w, v);
            };
            try { return origGus.call(this, pkt, q); } finally { delete this.spd; }
        };
        return () => { RQVAX.prototype.gus = origGus; };
    },

    /* --- FORMAT, ABORT, GET COMMAND STATUS --- */
    "FORMAT-checks-the-magic-bit-before-the-drive-type": () => {
        let orig = RQVAX.prototype.fmt;
        RQVAX.prototype.fmt = function(pkt, q) {
            let u = this.getucb(this.pd(pkt, CMD_UN));
            if (u && u.dtype !== RQVAX.RX33_DTYPE && (this.pd(pkt, FMT_IH) & FMT_IH_MAGIC) === 0) {
                /* Swap only the ORDER of the first two refusals, which is invisible unless a case
                   sends a non-RX33 without the magic bit. */
                let saved = u.dtype;
                u.dtype = RQVAX.RX33_DTYPE;
                try { return orig.call(this, pkt, q); } finally { u.dtype = saved; }
            }
            return orig.call(this, pkt, q);
        };
        return () => { RQVAX.prototype.fmt = orig; };
    },
    "FORMAT-refusing-an-online-unit-leaves-it-online": () => {
        let orig = RQVAX.prototype.fmt;
        RQVAX.prototype.fmt = function(pkt, q) {
            let u = this.getucb(this.pd(pkt, CMD_UN));
            let onl = u ? (u.flags & U_ONL) : 0;
            let r = orig.call(this, pkt, q);
            if (u) u.flags |= onl;
            return r;
        };
        return () => { RQVAX.prototype.fmt = orig; };
    },
    "ABORT-answers-ST_OFL-for-a-unit-that-does-not-exist": () => {
        /* *** ABORT NEVER REPORTS ST_OFL. ***  Every other unit command answers a bare ST_OFL for a
           plug no drive carries; rq_abo() has no such arm at all and says ST_SUC.  Making it behave
           like its neighbours is the reading a transcriber generalising from the other six would
           produce. */
        let origAbo = RQVAX.prototype.abo, origPutr = RQVAX.prototype.putr;
        RQVAX.prototype.abo = function(pkt, q) {
            let missing = !this.getucb(this.pd(pkt, CMD_UN));
            this.putr = function(p, cmd, flg, sts, lnt, typ) {
                return origPutr.call(this, p, cmd, flg, missing ? ST.OFL : sts, lnt, typ);
            };
            try { return origAbo.call(this, pkt, q); } finally { delete this.putr; }
        };
        return () => { RQVAX.prototype.abo = origAbo; };
    },
    "GET-COMMAND-STATUS-returns-the-host's-own-status-words": () => {
        /* rq_gcs()'s else arm ZEROES GCS_STSL/GCS_STSH, which is the only visible thing it does.
           Suppressing just those two writes -- while the shipped handler runs unchanged -- leaves
           the host's own command data in the response, which looks entirely plausible. */
        let origGcs = RQVAX.prototype.gcs, origSpd = RQVAX.prototype.spd;
        RQVAX.prototype.gcs = function(pkt, q) {
            this.spd = function(p, w, v) {
                if (p === pkt && (w === 10 || w === 11)) return;     /* GCS_STSL / GCS_STSH */
                return origSpd.call(this, p, w, v);
            };
            try { return origGcs.call(this, pkt, q); } finally { delete this.spd; }
        };
        return () => { RQVAX.prototype.gcs = origGcs; };
    },
    "the-response-lengths-are-all-RSP_LNT": () => {
        let orig = RQVAX.prototype.putr;
        RQVAX.prototype.putr = function(pkt, cmd, flg, sts, lnt, typ) {
            return orig.call(this, pkt, cmd, flg, sts, RSP_LNT, typ);
        };
        return () => { RQVAX.prototype.putr = orig; };
    },

    /* --- THE PROVIDER, AND THE CHEAT IT EXISTS TO PREVENT --- */
    "sim_disk_isavailable-always-says-NO": () => {
        /* rq_onl() asks sim_disk_isavailable() before it brings a unit online, and on this platform
           that function is `return TRUE;` for any attached RAW container -- measured, not assumed
           (the only arm that can say otherwise is Windows' media-eject ioctl, and the only thing
           that sets `media_removed` is sim_disk_unload(), which is excluded by name).  A controller
           that answered NO would report ST_OFL|SB_OFL_NV for a perfectly good image, which is a
           plausible-looking answer to the one question this whole item is about. */
        let orig = RQVAX.prototype.isAvailable;
        RQVAX.prototype.isAvailable = function(u) { return false; };
        return () => { RQVAX.prototype.isAvailable = orig; };
    },
    "the-image-provider's-contract-is-not-checked": () => {
        /* Removing the check is not visible in any response -- nothing here reads a block.  It is
           caught by the three malformed providers PHASE P offers, which is why that phase exists:
           a contract whose unused half is unenforced is not a contract, and pcjsvax-346 would be the
           one to find out. */
        let orig = RQVAX.prototype.attach;
        RQVAX.prototype.attach = function(i, provider, opts = {}) {
            let p = provider;
            if (p && typeof p === "object" && typeof p.read !== "function") {
                p = {byteLength: p.byteLength, read() { return 0; }};
            }
            return orig.call(this, i, p, opts);
        };
        return () => { RQVAX.prototype.attach = orig; };
    },
    "the-unit's-capacity-comes-from-the-DRIVE-TABLE-and-not-the-file": () => {
        /* THE ITEM'S NAMED CHEAT, stated as a mutation: answer RD54's numbers (or any drive's own)
           and ignore the user's container entirely.  It survives every case whose file happens to be
           smaller than its drive -- which is most of them -- and dies on PHASE A. */
        let orig = RQVAX.prototype.attach;
        RQVAX.prototype.attach = function(i, provider, opts = {}) {
            let u = this.unit(i);
            let r = orig.call(this, i, provider, opts);
            u.capac = DRV_TAB[u.dtype].lbn;
            return r;
        };
        return () => { RQVAX.prototype.attach = orig; };
    },

    /* --- THE CHEAT pcjsvax-0b4 NAMED, RE-ASSERTED HERE because this file's cases go through the
       same rings and would otherwise inherit its coverage rather than have any --- */
    "answers-synchronously-inside-the-IP-read": () => {
        let orig = RQVAX.prototype.rd;
        RQVAX.prototype.rd = function(pa) {
            let v = orig.call(this, pa);
            if (((pa >>> 1) & 1) === 0 && this.pip) { this.queDue = null; this.quesvc(); }
            return v;
        };
        return () => { RQVAX.prototype.rd = orig; };
    },
    "rq-window-decoded-at-the-wrong-offset": () => ({rqBaseDelta: 4}),
    "no-event-queue-at-all": () => ({noQbusHook: true})
};

/* ------------------------------------------------------------------------------------------- *
 * PHASE P -- the provider contract, checked by offering three broken ones                       *
 * ------------------------------------------------------------------------------------------- */

/**
 * checkProviderContract(mutationOpts)
 *
 * rq.js's attach() must REFUSE a provider that is not one.  There is no oracle for this -- SIMH has
 * no notion of an injectable image source, it has `fs` -- so it is graded against the contract
 * itself, which is the only honest way to grade an interface whose whole point is that the
 * controller cannot tell what is behind it.
 *
 * The three refusals are the three ways the browser half could be got wrong: a Blob whose size has
 * not been read yet (no byteLength), an ArrayBuffer passed raw (byteLength but no read), and a
 * length that is a float because it came out of a division.
 */
function checkProviderContract(failures, mutationOpts, providers)
{
    let rq = machine(mutationOpts).rq;
    let providersFor = (im) => providers[im.tag];
    let bad = [
        {what: "no byteLength", p: {read() { return 0; }}},
        {what: "no read()", p: {byteLength: 1024}},
        {what: "a fractional byteLength", p: {byteLength: 1024.5, read() { return 0; }}},
        {what: "not an object at all", p: null}
    ];
    for (let b of bad) {
        let threw = null;
        try {
            rq.detach(0);
            rq.attach(0, b.p, {});
        } catch (e) {
            threw = e;
        }
        if (!threw) {
            failures.push(`PHASE P: attach() ACCEPTED an image provider with ${b.what}.  Nothing in ` +
                `this item reads a block, so an unchecked contract fails silently here and loudly ` +
                `in pcjsvax-346 -- against a different oracle, in a different session.`);
            rq.detach(0);
        }
    }
    rq.detach(0);

    /* *** AND THE POSITIVE HALF: THE CONTROLLER MUST NOT BE ABLE TO TELL ONE PROVIDER FROM
       ANOTHER. ***  rq.js ships bufferProvider() -- the browser-side wrapper, over a Uint8Array,
       which is what a FileReader hands back -- and tests/mscpharness.js ships the `fs`-backed one.
       Attaching the SAME NUMBER OF BYTES through each must give the same unit, or the interface that
       makes HANDOFF.md 8's user-supplied-image decision implementable in a browser is a fiction.
       Graded against each other rather than against the oracle, because SIMH has no notion of an
       injectable image source at all -- it has `fs` -- so there is nothing else to compare to. */
    let rx33 = DRV_TAB.findIndex((d) => d.name === "RX33");
    let im = IMAGES.find((x) => x.tag === "odd");
    let cap = [];
    for (let prov of [providersFor(im), bufferProvider(new Uint8Array(im.bytes))]) {
        rq.detach(0);
        rq.setType(0, RD54_DTYPE);
        rq.setWriteLock(0, false);
        rq.setType(0, rx33);
        rq.attach(0, prov, {});
        cap.push(rq.units[0].capac >>> 0);
    }
    if (cap[0] !== cap[1]) {
        failures.push(`PHASE P: the same ${im.bytes}-byte image gives capac ${cap[0]} through the ` +
            `Node fs provider and ${cap[1]} through rq.js's own bufferProvider().  The controller ` +
            `is distinguishing WHERE the image came from, which is exactly what the injectable ` +
            `provider interface exists to make impossible -- and it is the browser half of it that ` +
            `would be wrong.`);
    }

    rq.detach(0);
    rq.setType(0, RD54_DTYPE);
    rq.setWriteLock(0, false);
}

/* ------------------------------------------------------------------------------------------- *
 * Driver                                                                                       *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

/**
 * The oracle's answers do not depend on rq.js, so they are computed ONCE and reused by every
 * mutation pass.  Keyed on the do-file TEXT rather than on "we already ran it": a mutation that
 * changed the case list would change the script and re-run the simulator, and the key is what says
 * so rather than a comment.  (The sibling differential re-runs SIMH once per mutation; with 38
 * mutations and three phases that is most of an hour of simulator startup for output that cannot
 * have changed.)
 */
const ORACLE_CACHE = new Map();

function cached(key, fn) {
    if (!ORACLE_CACHE.has(key)) ORACLE_CACHE.set(key, fn());
    return ORACLE_CACHE.get(key);
}

function runPass(simh, opts, mutationOpts = {})
{
    let failures = [], report = [];
    let acc = {waitCounts: new Set(), nonZeroWaits: 0, zeroWaits: 0,
               rspOpc: new Set(), rspSts: new Set(), cmdLines: 0,
               dtypesGraded: new Set(), typesMeasured: 0, attachMeasured: 0, attachCaps: new Set(),
               shortForm: false, longForm: false,
               wphSet: false, wphClear: false, rmvSet: false, rmvClear: false, wpsSet: false};

    /* ---- PHASE S: the scope, re-derived from the C on every run ---- */
    let scope = opts.scope;
    for (let f of scope.failures) failures.push(f);
    report.push(`  PHASE S  ${scope.nOp} OP_ codes, ${scope.nSt} ST_ codes, ${scope.nSwitch} ` +
        `rq_mscp() dispatch case(s) and ${scope.nDrv} drv_tab[] entries re-derived from ` +
        `${scope.dir}; rq.js agrees`);

    /* ---- PHASE T: every drive type against the live simulator ---- */
    let types = cached("types", () => runPhaseT(simh, opts));
    gradeTypes(types, failures, acc);
    report.push(`  PHASE T  ${acc.typesMeasured}/${DRV_TAB.length} drive type(s) cross-checked ` +
        `against \`set rq0 <type>\` + \`show rq0\` + \`examine rq capac[0]\` on the oracle`);

    /* ---- PHASE A: the autosize arithmetic ---- */
    let pts = attachSweep();
    let simCaps = cached("attach", () => runPhaseA(simh, opts, opts.images, pts));
    let jsCaps = runAttachSweepJS(pts, opts.providers, mutationOpts);
    acc.attachPts = pts;
    acc.attachSim = simCaps;
    gradeAttach(pts, simCaps, jsCaps, failures, acc);
    report.push(`  PHASE A  ${acc.attachMeasured}/${pts.length} attach point(s) compared ` +
        `(${DRV_TAB.length} drive types x ${IMAGES.length} containers x 4 attach modes), ` +
        `${acc.attachCaps.size} distinct capacit(ies) on the oracle`);

    /* ---- PHASE P: the provider contract ---- */
    checkProviderContract(failures, mutationOpts, opts.providers);
    report.push(`  PHASE P  attach() refuses 4 malformed image providers, and the fs-backed and buffer-backed\n           providers give the same unit for the same bytes`);

    /* ---- PHASE C: the MSCP commands ---- */
    let cases = enumeratedCases(opts.images);
    let all = cases.concat(randomCases(opts.nRandom, opts.seed, cases.length));
    let sim = cached("cases:" + opts.nRandom + ":" + opts.seed,
                     () => runCasesSimh(simh, opts, opts.images, all));
    /* Each SIMH invocation starts a NEW simulator, so the oracle's static `MSC` struct starts at its
       C-global zero.  The JS machine is built once and reused (standing rule 14), so the pass
       boundary is where that has to be re-established. */
    machine(mutationOpts).rq.powerUp();
    let js = all.map((c) => runCaseJS(c, opts.providers, mutationOpts));

    assertExclusions(all, sim, failures);
    let compared = grade(all, sim, js, failures);
    assertWaits(all, sim, js, failures, acc);
    coverage(all, sim, js, failures, acc);

    /* The wiring the graded machine is actually holding, asserted rather than assumed. */
    let m = machine(mutationOpts);
    if (!mutationOpts.noQbusHook && m.cpu.qbus !== m.rq) {
        failures.push(`the graded machine's CPU has no Qbus event hook wired to the controller`);
    }
    if (!m.rq.cqbic || !m.rq.cqbic.bus) {
        failures.push(`the graded machine's controller has no CQBIC with a bus`);
    }
    if (m.rq.units.length !== RQ_NUMDR) {
        failures.push(`the graded machine's controller has ${m.rq.units.length} units and not ` +
            `RQ_NUMDR (${RQ_NUMDR}) -- the RQ_TIMER and RQ_QUEUE pseudo-units must not be drives`);
    }

    report.push(`  PHASE C  ${compared}/${all.length} case(s) compared ` +
        `(${cases.length} enumerated + ${opts.nRandom} randomized)`);
    report.push(`  TRACE    ${acc.cmdLines} command line(s) in the oracle's DEBUG=REQ stream, ` +
        `${acc.rspOpc.size} distinct response opcode(s), ${acc.rspSts.size} distinct status word(s)`);
    report.push(`  FLOORS   ${acc.dtypesGraded.size} drive type(s) graded through responses, ` +
        `${acc.waitCounts.size} distinct in-band wait count(s), ` +
        `long/short form seen: ${acc.longForm && acc.shortForm ? "both" : "NO"}, ` +
        `UF_WPH both ways: ${acc.wphSet && acc.wphClear ? "yes" : "NO"}, ` +
        `UF_RMV both ways: ${acc.rmvSet && acc.rmvClear ? "yes" : "NO"}`);
    return {failures, report, compared, acc};
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
        console.error(`mscpunitdiff: --cases ${nRandom} is below the fixed floor of ${RANDOM_CASES_FLOOR}`);
        process.exit(1);
    }

    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "mscpunitdiff-"));
    let images = null, providers = {};
    let code = 0;
    try {
        console.log(`SIMH: ${simh}`);
        console.log(`scratch: ${scratch}`);
        console.log(`seed: ${seed}   randomized cases: ${nRandom}`);

        images = makeImages(scratch);
        console.log(`containers: ${images.map((i) => `${i.tag}=${i.bytes}B`).join(", ")} ` +
            `(GENERATED here and deleted on exit -- HANDOFF.md 8: no image is shipped)`);
        for (let im of images) providers[im.tag] = fileImageProvider(im.path);

        let opts = {scratch, nRandom, seed, images, providers};
        opts.scope = checkScope(simh);

        let {failures, report} = runPass(simh, opts);
        checkImagesUntouched(images, failures);
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
            console.log(`\nMATCH -- a user-supplied image attached to a unit appears to the guest ` +
                `exactly as it does on the oracle: all ${DRV_TAB.length} drive types cross-checked ` +
                `against the simulator's own \`set\`/\`show\`, the autosize arithmetic over ` +
                `${DRV_TAB.length * IMAGES.length * 4} attach points, and every field of every ` +
                `GET UNIT STATUS / ONLINE / SET UNIT CHARACTERISTICS / AVAILABLE / FORMAT / ABORT / ` +
                `GET COMMAND STATUS response -- media identifier, unit identifier, unit flags, ` +
                `geometry, capacity and volume serial -- plus the per-unit registers, ` +
                `${PKT_PROBES.length} words of the packet array, every byte of every physical page ` +
                `the rings and packets were scattered across, and the ORDERED DEBUG=REQ trace.`);
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
        /* EVERY EXIT PATH, INCLUDING A THROW.  The containers are up to 42 MB and the scratch
           directory is under /tmp; a run that left them behind would fill the root filesystem after
           a few dozen invocations, which has happened in this tree before (HANDOFF.md 4). */
        for (let k of Object.keys(providers)) {
            try { providers[k].close(); } catch (e) { /* best effort */ }
        }
        try { fs.rmSync(scratch, {recursive: true, force: true}); } catch (e) { /* best effort */ }
    }
    process.exit(code);
}

main();
