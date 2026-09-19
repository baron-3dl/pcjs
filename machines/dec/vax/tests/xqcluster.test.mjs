/**
 * @fileoverview pcjsvax-636 -- the pcjs VAX DELQA, re-pointed from the in-process L2 hub to the
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 *                 parent page via postMessage -- the cluster-node hook round-trip proof
 *
 * WHAT THIS IS  (frozen contract openvmx-site demo/cluster/PCJS-NODE-BRIDGE.md)
 * ----------------------------------------------------------------------------
 * tests/xqhub.test.mjs already proves the whole DELQA datapath (guest memory -> CQBIC map -> device
 * TX -> hub -> device RX -> map -> guest memory) against the REAL in-process L2Hub.  This test proves
 * that the SAME DELQA, with its HubPort re-pointed at the postMessage bridge
 * (browser/nic-postmessage-hub.js), honours the frozen browser cluster-node contract EXACTLY:
 *
 *   guest DELQA TX  ->  emit  ->  {t:'nic-tx', frame:<ArrayBuffer>}  posted UP to the page, VERBATIM
 *   {t:'nic-rx', frame:<ArrayBuffer>} from the page  ->  onMessage  ->  guest DELQA RX, VERBATIM
 *   HubPort attaches ->  {t:'nic-ready'}  posted UP (the page gates its wiring on this)
 *
 * It is the pcjs analogue of the qemu-wasm Node A's {onNicTx, deliverToGuest}, and it is graded under
 * Node with NO browser -- the bridge's `post` is a capture fn and inbound frames are fed through
 * `onMessage`, so the transport-agnostic seam is exercised end to end against the real XQVAX device.
 *
 *      node machines/dec/vax/tests/xqcluster.test.mjs
 */

import { createPostMessageHub } from "../browser/nic-postmessage-hub.js";
import HubPortEthernetLink from "../modules/v2/ethlink-hubport.js";
import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import CQBICVAX from "../modules/v2/cqbic.js";
import XQVAX, { XQ_BASE, XQ_CSR_RE, XQ_CSR_IL, XQ_DSC_V, XQ_DSC_E } from "../modules/v2/xq.js";

const MEMSIZE = 0x01000000;
const PAGE = 512;

/* CQBIC map constants (mirrors cqbic.js / xqhub.test.mjs) -- a valid entry is VLD | (physPage). */
const MAP_MBR = 0x00200000;
const CQMAP_VLD = 0x80000000;
const DATA_BASE = 0x00300000;

/* Register-word addresses inside the DELQA window. */
const R_RBDL_LO = (XQ_BASE + 4) >>> 0, R_RBDL_HI = (XQ_BASE + 6) >>> 0;
const R_XBDL_LO = (XQ_BASE + 8) >>> 0, R_XBDL_HI = (XQ_BASE + 10) >>> 0;
const R_CSR = (XQ_BASE + 14) >>> 0;

/* Qbus addresses (each on its own 512-byte page) and the scattered physical pages they map to. */
const QB_DESC_TX = 0x2000, QB_BUF_TX = 0x2200, QB_DESC_RX = 0x2400, QB_BUF_RX = 0x2600;
const PH_DESC_TX = (DATA_BASE + 5 * PAGE) >>> 0, PH_BUF_TX = (DATA_BASE + 2 * PAGE) >>> 0;
const PH_DESC_RX = (DATA_BASE + 9 * PAGE) >>> 0, PH_BUF_RX = (DATA_BASE + 0 * PAGE) >>> 0;
const DMA_MAP = [
    { qb: QB_DESC_TX, ph: PH_DESC_TX }, { qb: QB_BUF_TX, ph: PH_BUF_TX },
    { qb: QB_DESC_RX, ph: PH_DESC_RX }, { qb: QB_BUF_RX, ph: PH_BUF_RX }
];
const RX_BG = 0xEE;

const MAC_SELF = [0x08, 0x00, 0x2B, 0x0B, 0x0B, 0x0B];  /* this node */
const MAC_PEER = [0x08, 0x00, 0x2B, 0x0C, 0x0C, 0x0C];  /* the peer that sends us a frame */

function mapEntry(ph) { return (CQMAP_VLD | ((ph >>> 9) & 0x000FFFFF)) >>> 0; }
function mapEntryPhys(qb) { return (MAP_MBR + (((qb >>> 9) << 2) & 0x7FFF)) >>> 0; }
function twosWords(nbytes) { return ((~(nbytes >> 1) + 1) & 0xFFFF) >>> 0; }
function u16le(w) { return [w & 0xFF, (w >>> 8) & 0xFF]; }

let passed = 0, failed = 0;
function ok(cond, msg) { if (cond) { passed++; console.log("  PASS " + msg); } else { failed++; console.log("  FAIL " + msg); } }

/** Build one guest node: bus + 16MB RAM + CPU + CQBIC + DELQA, its map programmed, mac set. */
function makeNode(mac)
{
    let bus = new BusVAX({ busWidth: VAX.PAWIDTH, id: "bus" }, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({ id: "cpu" });
    cpu.setBus(bus);
    cpu.reset();
    let cqbic = new CQBICVAX(cpu.exc, bus, MEMSIZE);
    let xq = new XQVAX(cqbic, {});
    xq.setMac(mac);
    cqbic.mbr = MAP_MBR >>> 0;
    for (let e of DMA_MAP) bus.setLong(mapEntryPhys(e.qb), mapEntry(e.ph));
    return { bus, cpu, cqbic, xq };
}

function seedTx(node, frame)
{
    let d = [0, XQ_DSC_V | XQ_DSC_E | ((QB_BUF_TX >>> 16) & 0x3F), QB_BUF_TX & 0xFFFF, twosWords(frame.length), 0, 0];
    let db = []; for (let w of d) db.push(...u16le(w));
    for (let k = 0; k < db.length; k++) node.bus.setByte((PH_DESC_TX + k) >>> 0, db[k]);
    for (let k = 0; k < frame.length; k++) node.bus.setByte((PH_BUF_TX + k) >>> 0, frame[k]);
}

function postRx(node, capBytes)
{
    let d = [0, XQ_DSC_V | ((QB_BUF_RX >>> 16) & 0x3F), QB_BUF_RX & 0xFFFF, twosWords(capBytes), 0, 0];
    let db = []; for (let w of d) db.push(...u16le(w));
    for (let k = 0; k < db.length; k++) node.bus.setByte((PH_DESC_RX + k) >>> 0, db[k]);
    for (let k = 0; k < capBytes; k++) node.bus.setByte((PH_BUF_RX + k) >>> 0, RX_BG);
    node.xq.wr(R_RBDL_LO, QB_DESC_RX & 0xFFFF);
    node.xq.wr(R_RBDL_HI, (QB_DESC_RX >>> 16) & 0x3F);
    node.xq.wr(R_CSR, XQ_CSR_RE);
}

function readRx(node, n)
{
    let b = new Uint8Array(n);
    for (let k = 0; k < n; k++) b[k] = node.bus.getByte((PH_BUF_RX + k) >>> 0) & 0xFF;
    return b;
}

function transmit(node)
{
    node.xq.wr(R_CSR, XQ_CSR_IL);                       /* IL: normal transmit (not loopback) */
    node.xq.wr(R_XBDL_LO, QB_DESC_TX & 0xFFFF);
    node.xq.wr(R_XBDL_HI, (QB_DESC_TX >>> 16) & 0x3F);
}

function frame(dst, src, type, payloadLen, fill)
{
    let f = new Uint8Array(14 + payloadLen);
    f.set(dst, 0); f.set(src, 6); f[12] = (type >>> 8) & 0xFF; f[13] = type & 0xFF;
    for (let i = 0; i < payloadLen; i++) f[14 + i] = (fill + i) & 0xFF;
    return f;
}

/* ------------------------------------------------------------------------------------------- *
 * The bridge: one DELQA node whose HubPort is re-pointed at a captured postMessage "page".        *
 * ------------------------------------------------------------------------------------------- */

console.log("pcjsvax-636: the pcjs DELQA over the postMessage cluster-node bridge\n");

/* The "page" side: every message the machine posts UP is captured here (the node page would forward
   these to connectNicPipe / the parent switch).  A real transfer would neuter frame.buffer; the Node
   capture keeps it, which is exactly what lets us assert the bytes. */
let up = [];
const { hub, onMessage, stats } = createPostMessageHub({ post: (m) => up.push(m) });

const node = makeNode(MAC_SELF);

console.log("attach: the HubPort comes up and signals readiness:");
{
    node.xq.attach(new HubPortEthernetLink({ name: "node-B" }).connectHub(hub));
    ok(up.length === 1 && up[0].t === "nic-ready", "{t:'nic-ready'} posted UP exactly once on attach");
    up = [];
}

/* ---- (1) guest TX surfaces as ONE verbatim {t:'nic-tx'} ---- */
console.log("\nguest DELQA TX -> exactly one {t:'nic-tx'}, bytes verbatim, private copy:");
{
    postRx(node, 256);                                  /* node also listens, to prove no self-hear */
    const tx = frame(MAC_PEER, MAC_SELF, 0x6007, 50, 0xA0);   /* dst peer, src us, SCS ethertype */
    seedTx(node, tx);
    transmit(node);

    let txMsgs = up.filter((m) => m.t === "nic-tx");
    ok(txMsgs.length === 1, "exactly one {t:'nic-tx'} surfaced for one transmitted frame");
    let got = txMsgs.length ? new Uint8Array(txMsgs[0].frame) : new Uint8Array(0);
    ok(got.length === tx.length && got.every((b, i) => b === tx[i]),
       "the nic-tx frame is byte-exact with what the guest transmitted (dst/src/type/payload)");

    /* independence: the frame handed UP is a private copy, not an alias of guest memory */
    let before = got.slice();
    node.bus.setByte(PH_BUF_TX >>> 0, (tx[0] ^ 0xFF) & 0xFF);   /* stomp the guest TX buffer */
    ok(got.every((b, i) => b === before[i]), "the nic-tx frame did not alias guest memory (verbatim copy)");

    /* no self-hear: emit() only posts UP; the node's own RX buffer is untouched */
    let rxSelf = readRx(node, tx.length);
    ok(rxSelf.every((b) => b === RX_BG), "the node did not hear its own transmit (no self-hear)");
    up = [];
}

/* ---- (2) an inbound {t:'nic-rx'} is delivered into the guest RX, verbatim ---- */
console.log("\n{t:'nic-rx'} from the page -> guest DELQA RX, byte-exact through the CQBIC map:");
{
    postRx(node, 256);
    const rx = frame(MAC_SELF, MAC_PEER, 0x6007, 60, 0x50);    /* dst us, src peer */
    let buf = rx.slice().buffer;
    let delivered = onMessage({ t: "nic-rx", frame: buf });
    ok(delivered, "onMessage delivered the nic-rx to the DELQA");

    let rxBuf = readRx(node, rx.length);
    ok(rxBuf.length === rx.length && rxBuf.every((b, i) => b === rx[i]),
       "the guest received the exact bytes the page delivered");
    ok(node.xq.csr & 0x8000, "the DELQA raised RI (receive-complete) in CSR");
}

/* ---- (3) never-crash-a-peer: malformed / oversize / undecodable nic-rx is DROPPED, guest untouched ---- */
console.log("\nnever-crash-a-peer: malformed nic-rx frames are dropped, not delivered, no throw:");
{
    postRx(node, 256);                                  /* fresh RX buffer, background-filled */
    let dropBefore = stats.rxDrop;
    let threw = false;
    try {
        onMessage({ t: "nic-rx", frame: new Uint8Array(4).buffer });      /* < 14: runt */
        onMessage({ t: "nic-rx", frame: new Uint8Array(2000).buffer });   /* > 1600: giant */
        onMessage({ t: "nic-rx", frame: null });                          /* undecodable */
        onMessage({ t: "nic-rx" });                                       /* no frame at all */
    } catch (e) { threw = true; }
    ok(!threw, "no malformed frame threw (the guest is never crashed)");
    ok(stats.rxDrop - dropBefore === 4, "all four malformed frames were counted as drops");
    let rxBuf = readRx(node, 60);
    ok(rxBuf.every((b) => b === RX_BG), "no malformed frame reached the guest RX buffer");
}

/* ---- (4) a frame NOT addressed to us is filtered (reuses EthernetLink._classify) ---- */
console.log("\nRX filter: a unicast to a different station is dropped by the device filter:");
{
    postRx(node, 256);
    const other = frame([0x08, 0x00, 0x2B, 0x99, 0x99, 0x99], MAC_PEER, 0x6007, 46, 0x77);
    onMessage({ t: "nic-rx", frame: other.slice().buffer });
    let rxBuf = readRx(node, other.length);
    ok(rxBuf.every((b) => b === RX_BG), "a frame addressed to another station did not reach our RX");
}

console.log("");
console.log(`stats: tx=${stats.tx} txDrop=${stats.txDrop} rx=${stats.rx} rxDrop=${stats.rxDrop}`);
if (failed) { console.log(`xqcluster.test: FAIL (${failed} of ${passed + failed})`); process.exit(1); }
console.log(`xqcluster.test: ALL PASS (${passed}) -- the pcjs DELQA honours the postMessage cluster-node contract`);
