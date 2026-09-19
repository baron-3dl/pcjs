/**
 * @fileoverview MILESTONE pcjsvax-c96 -- two DELQA guests exchange REAL Ethernet frames through the
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 *                 in-process L2 hub -- the datapath proof (epic pcjsvax-d1bf, Wave 2)
 *
 * WHAT THIS IS
 * ------------
 * Two full XQVAX (DELQA) instances, each with its OWN bus + CQBIC scatter-gather map, share ONE real
 * L2Hub (the frozen contract-v1 hub.mjs, vendored -- NOT a mock).  A frame is GATHERED out of guest A's
 * VAX memory through A's CQBIC map by the real xq_process_xbdl TX path, flooded by the hub, and
 * SCATTERED into guest B's VAX memory through B's CQBIC map by the real xq_process_rbdl RX path.  The
 * bytes B receives are compared to the bytes A wrote, BYTE FOR BYTE -- the never-crash-a-peer invariant
 * (a single wrong byte to a real VMS peer is the crash class this NIC exists to prevent).
 *
 * This is the whole datapath end to end: guest memory -> map -> device TX -> hub -> device RX -> map ->
 * guest memory, with zero parallel transport.  It closes the "networking actually works" gate before any
 * browser transport (Wave 3).
 *
 *      node machines/dec/vax/tests/xqhub.test.mjs
 */

import { L2Hub } from "./vendor/l2/hub.mjs";
import HubPortEthernetLink from "../modules/v2/ethlink-hubport.js";
import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import CQBICVAX from "../modules/v2/cqbic.js";
import XQVAX, {
    XQ_BASE, XQ_CSR_RE, XQ_CSR_IL, XQ_DSC_V, XQ_DSC_E
} from "../modules/v2/xq.js";

const MEMSIZE = 0x01000000;
const PAGE = 512;

/* CQBIC map constants (mirrors cqbic.js / mscpharness) -- a valid entry is VLD | (physPage). */
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

const MAC_A = [0x08, 0x00, 0x2B, 0x11, 0x11, 0x11];
const MAC_B = [0x08, 0x00, 0x2B, 0x22, 0x22, 0x22];
const BCAST = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];

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
    /* program the CQBIC map so the Qbus buffer pages translate to scattered physical pages */
    cqbic.mbr = MAP_MBR >>> 0;
    for (let e of DMA_MAP) bus.setLong(mapEntryPhys(e.qb), mapEntry(e.ph));
    return { bus, cpu, cqbic, xq };
}

/** Seed a TX descriptor + payload into a node's physical memory (through the map's physical pages). */
function seedTx(node, frame)
{
    let d = [0, XQ_DSC_V | XQ_DSC_E | ((QB_BUF_TX >>> 16) & 0x3F), QB_BUF_TX & 0xFFFF, twosWords(frame.length), 0, 0];
    let db = []; for (let w of d) db.push(...u16le(w));
    for (let k = 0; k < db.length; k++) node.bus.setByte((PH_DESC_TX + k) >>> 0, db[k]);
    for (let k = 0; k < frame.length; k++) node.bus.setByte((PH_BUF_TX + k) >>> 0, frame[k]);
}

/** Post an empty RX descriptor + background-filled buffer into a node's physical memory. */
function postRx(node, capBytes)
{
    let d = [0, XQ_DSC_V | ((QB_BUF_RX >>> 16) & 0x3F), QB_BUF_RX & 0xFFFF, twosWords(capBytes), 0, 0];
    let db = []; for (let w of d) db.push(...u16le(w));
    for (let k = 0; k < db.length; k++) node.bus.setByte((PH_DESC_RX + k) >>> 0, db[k]);
    for (let k = 0; k < capBytes; k++) node.bus.setByte((PH_BUF_RX + k) >>> 0, RX_BG);
    /* drive the RBDL registers: dispatch_rbdl (clears RL, fetches the descriptor) */
    node.xq.wr(R_RBDL_LO, QB_DESC_RX & 0xFFFF);
    node.xq.wr(R_RBDL_HI, (QB_DESC_RX >>> 16) & 0x3F);
    /* enable the receiver (RE) */
    node.xq.wr(R_CSR, XQ_CSR_RE);
}

/** Read `n` bytes out of a node's RX buffer (physical). */
function readRx(node, n)
{
    let b = new Uint8Array(n);
    for (let k = 0; k < n; k++) b[k] = node.bus.getByte((PH_BUF_RX + k) >>> 0) & 0xFF;
    return b;
}

/** Trigger a normal (on-the-wire) transmit: set IL to force the non-loopback arm, drive XBDL. */
function transmit(node)
{
    node.xq.wr(R_CSR, XQ_CSR_IL);                       /* IL: normal transmit (not loopback) */
    node.xq.wr(R_XBDL_LO, QB_DESC_TX & 0xFFFF);
    node.xq.wr(R_XBDL_HI, (QB_DESC_TX >>> 16) & 0x3F); /* clears XL -> dispatch_xbdl -> gather + send */
}

function frame(dst, src, type, payloadLen, fill)
{
    let f = new Uint8Array(14 + payloadLen);
    f.set(dst, 0); f.set(src, 6); f[12] = (type >>> 8) & 0xFF; f[13] = type & 0xFF;
    for (let i = 0; i < payloadLen; i++) f[14 + i] = (fill + i) & 0xFF;
    return f;
}

/* ------------------------------------------------------------------------------------------- *
 * The segment: real L2Hub + node A + node B, each on its own HubPort.                            *
 * ------------------------------------------------------------------------------------------- */

console.log("MILESTONE pcjsvax-c96: two DELQA guests over the real hub.mjs\n");

let drops = [];
const hub = new L2Hub({ onDrop: (reason, port) => drops.push({ reason, port }) });

const A = makeNode(MAC_A);
const B = makeNode(MAC_B);

A.xq.attach(new HubPortEthernetLink({ name: "node-A" }).connectHub(hub));
B.xq.attach(new HubPortEthernetLink({ name: "node-B" }).connectHub(hub));
console.log("segment wired: node-A + node-B on the real L2Hub\n");

/* ---- (1) UNICAST A->B: a 0x6007 (SCS) frame gathered from A's memory arrives byte-exact in B's ---- */
console.log("unicast A->B, byte-exact through both CQBIC maps:");
{
    postRx(B, 256);                                     /* B posts a receive buffer + enables RX */
    postRx(A, 256);                                     /* A also listens, to prove it does NOT self-hear */
    const tx = frame(MAC_B, MAC_A, 0x6007, 50, 0xA0);   /* dst B, src A, SCS ethertype */
    seedTx(A, tx);
    transmit(A);

    const rxB = readRx(B, tx.length);
    let exact = rxB.length === tx.length && rxB.every((b, i) => b === tx[i]);
    ok(exact, "B received the exact " + tx.length + " bytes A transmitted (dst/src/type/payload)");

    /* A must NOT have received its own frame (hub no-loopback + from_me filter) */
    const rxA = readRx(A, tx.length);
    let aUntouched = rxA.every((b) => b === RX_BG);
    ok(aUntouched, "A did not hear its own frame (no self-delivery)");
    ok(B.xq.csr & 0x8000, "B raised RI (receive-complete) in CSR");
    ok(drops.length === 0, "the hub dropped nothing");
}

/* ---- (2) BROADCAST A->*: B hears it (its filter includes the broadcast address, as a VMS setup
   packet programs), A does not.  Faithful to sim_ether: broadcast is accepted ONLY when the filter
   table carries FF:FF:FF:FF:FF:FF (or all-multicast is set) -- not implicitly. ---- */
console.log("\nbroadcast A->*, delivered to B and not looped to A:");
{
    B.xq.etherface.setFilter([MAC_B, BCAST], false, false);  /* VMS programs broadcast into the filter */
    postRx(B, 256);
    postRx(A, 256);
    const tx = frame(BCAST, MAC_A, 0x6007, 46, 0x30);
    seedTx(A, tx);
    transmit(A);

    const rxB = readRx(B, tx.length);
    ok(rxB.every((b, i) => b === tx[i]), "B received the broadcast frame byte-exact");
    const rxA = readRx(A, tx.length);
    ok(rxA.every((b) => b === RX_BG), "A did not receive its own broadcast");
}

/* ---- (3) UNICAST B->A the other direction: proves the segment is bidirectional ---- */
console.log("\nunicast B->A (reverse direction):");
{
    postRx(A, 256);
    postRx(B, 256);
    const tx = frame(MAC_A, MAC_B, 0x6007, 60, 0x50);
    seedTx(B, tx);
    transmit(B);

    const rxA = readRx(A, tx.length);
    ok(rxA.every((b, i) => b === tx[i]), "A received the exact frame B transmitted");
    const rxB = readRx(B, tx.length);
    ok(rxB.every((b) => b === RX_BG), "B did not hear its own frame");
}

console.log("");
if (failed) { console.log(`xqhub.test: FAIL (${failed} of ${passed + failed})`); process.exit(1); }
console.log(`xqhub.test: ALL PASS (${passed}) -- two DELQA guests exchange real frames through the real L2 hub`);
