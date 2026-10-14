/**
 * @fileoverview DELQA transmit gather across a buffer that starts on an ODD byte (the H bit)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * WHAT THIS GRADES
 * ----------------
 * A transmit buffer descriptor whose buffer starts on an odd byte carries the H ("high byte start")
 * bit.  The Qbus moves WORDS, so the controller fetches from the word that contains the first byte
 * and starts at its high byte: the start byte is (address & ~1) + 1.  Address bit 0 is not a
 * second "+1".  A driver may write the buffer's own odd byte address into the descriptor (NetBSD's
 * if_qe.c does exactly that: `qe_addr_lo = LOWORD(buffer)` with QE_ODDBEGIN set when `buffer & 1`),
 * and the port inherited SIMH's `address += 1`, which on such a descriptor reads ONE BYTE PAST the
 * buffer start: the frame on the wire loses the segment's first byte, every later byte shifts left
 * by one, and one byte from beyond the buffer is appended so the length still looks right.
 *
 * That is the corruption the 3-node browser cluster demo captured from its real VAX/VMS node: a
 * 484-byte clusterwide logical-name database push left the VAX with one byte deleted at database
 * offset 61 and everything after it shifted (openvmx-site run 38577678113).
 *
 * The frames below are built as multi-buffer chains -- header, then segments that start on odd and
 * even bytes, with and without L -- and every one must arrive on the hub byte-for-byte.
 *
 *      node machines/dec/vax/tests/xqoddbuf.test.mjs
 */

import { L2Hub } from "./vendor/l2/hub.mjs";
import HubPortEthernetLink from "../modules/v2/ethlink-hubport.js";
import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import CQBICVAX from "../modules/v2/cqbic.js";
import XQVAX, { XQ_BASE, XQ_CSR_IL, XQ_DSC_V, XQ_DSC_E, XQ_DSC_H, XQ_DSC_L } from "../modules/v2/xq.js";

const MEMSIZE = 0x01000000;
const PAGE = 512;
const MAP_MBR = 0x00200000;
const CQMAP_VLD = 0x80000000;
const DATA_BASE = 0x00300000;
const R_XBDL_LO = (XQ_BASE + 8) >>> 0, R_XBDL_HI = (XQ_BASE + 10) >>> 0;
const R_CSR = (XQ_BASE + 14) >>> 0;

/* Qbus pages 0x10..0x1F map to SCATTERED physical pages, so a segment that crosses a page also
   crosses a map register (the descriptor page is Qbus page 0x10). */
const QB_DESC = 0x2000, QB_BUF = 0x2200, NPAGES = 8;
const physOf = (qbPage) => (DATA_BASE + ((qbPage * 7) % 31) * PAGE) >>> 0;

const MAC_A = [0x52, 0x54, 0x00, 0x00, 0x00, 0x0C];
const MAC_B = [0x52, 0x54, 0x00, 0x00, 0x00, 0x0A];

let passed = 0, failed = 0;
function ok(cond, msg) { if (cond) { passed++; console.log("  PASS " + msg); } else { failed++; console.log("  FAIL " + msg); } }
const u16le = (w) => [w & 0xFF, (w >>> 8) & 0xFF];

function makeNode()
{
    let bus = new BusVAX({ busWidth: VAX.PAWIDTH, id: "bus" }, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({ id: "cpu" });
    cpu.setBus(bus);
    cpu.reset();
    let cqbic = new CQBICVAX(cpu.exc, bus, MEMSIZE);
    let xq = new XQVAX(cqbic, {});
    xq.setMac(MAC_A);
    cqbic.mbr = MAP_MBR >>> 0;
    for (let p = 0; p <= NPAGES + 1; p++) {
        let qb = QB_DESC + p * PAGE;
        bus.setLong((MAP_MBR + (((qb >>> 9) << 2) & 0x7FFF)) >>> 0, (CQMAP_VLD | ((physOf(qb >>> 9) >>> 9) & 0xFFFFF)) >>> 0);
    }
    return { bus, xq };
}

/** Write `bytes` into guest memory at Qbus address `qa` (through the map, page by page). */
function poke(node, qa, bytes)
{
    for (let k = 0; k < bytes.length; k++) {
        let a = qa + k;
        node.bus.setByte((physOf(a >>> 9) + (a & (PAGE - 1))) >>> 0, bytes[k]);
    }
}

/**
 * The descriptor for a segment of `n` bytes starting at byte address `start`, the way a driver
 * that writes the buffer's own byte address computes it (NetBSD if_qe.c qestart()): odd start ->
 * H and one more word, odd end -> L.
 */
function descFor(start, n, eom)
{
    let len = n, flags = XQ_DSC_V | (eom ? XQ_DSC_E : 0);
    if ((start & 1) || (len & 1)) len += 2;
    if (start & 1) flags |= XQ_DSC_H;
    if ((start + len) & 1) flags |= XQ_DSC_L;
    return [0, flags | ((start >>> 16) & 0x3F), start & 0xFFFF, (-(len >> 1)) & 0xFFFF, 0, 0];
}

/**
 * Transmit `frame` split at `cuts` (byte offsets into the frame), each segment placed at its own
 * Qbus address with the given start parity.  Returns the frame the hub saw.
 */
function sendChain(frame, cuts, oddStarts)
{
    const node = makeNode();
    const hub = new L2Hub();
    const seen = [];
    hub.addPort({ name: "tap", send: (f) => seen.push(Uint8Array.from(f)) });
    node.xq.attach(new HubPortEthernetLink({ name: "vax" }).connectHub(hub));

    const bounds = [0, ...cuts, frame.length];
    let qa = QB_BUF, descs = [];
    for (let s = 0; s + 1 < bounds.length; s++) {
        const seg = frame.subarray(bounds[s], bounds[s + 1]);
        qa = (qa + 64) & ~1;                              /* gap between segments */
        if (oddStarts[s]) qa |= 1;
        poke(node, qa, Array.from({ length: seg.length + 4 }, () => 0xEE));   /* guard bytes after */
        poke(node, qa, seg);
        descs.push(descFor(qa, seg.length, s + 2 === bounds.length));
        qa += seg.length;
    }
    let db = []; for (let d of descs) for (let w of d) db.push(...u16le(w));
    db.push(...u16le(0), ...u16le(0));                    /* terminator: V clear */
    poke(node, QB_DESC, db);

    node.xq.wr(R_CSR, XQ_CSR_IL);
    node.xq.wr(R_XBDL_LO, QB_DESC & 0xFFFF);
    node.xq.wr(R_XBDL_HI, (QB_DESC >>> 16) & 0x3F);
    return seen[0] || null;
}

function makeFrame(len)
{
    let f = new Uint8Array(len);
    f.set(MAC_B, 0); f.set(MAC_A, 6); f[12] = 0x60; f[13] = 0x07;
    for (let i = 14; i < len; i++) f[i] = (i * 37 + 11) & 0xFF;
    return f;
}

function firstDiff(a, b)
{
    if (!a || !b) return 0;
    for (let i = 0; i < Math.max(a.length, b.length); i++) if (a[i] !== b[i]) return i;
    return -1;
}

console.log("DELQA TX gather: segments starting on odd bytes (H) arrive byte-exact\n");

const CASES = [
    /* the demo's shape: a 568-byte SCS frame whose last segment starts on an odd byte at frame
       offset 145 (database offset 61) */
    { name: "568-byte frame, header + data, last segment starts odd at offset 145",
      len: 568, cuts: [84, 145], odd: [false, false, true] },
    { name: "odd-start middle segment, even-start tail", len: 300, cuts: [16, 101], odd: [false, true, false] },
    { name: "odd start AND odd length (H + L)", len: 128, cuts: [14, 47], odd: [false, true, true] },
    { name: "single odd-start buffer", len: 64, cuts: [], odd: [true] },
    { name: "odd-start segment crossing a Qbus page (two map registers)", len: 900, cuts: [14], odd: [false, true] },
    { name: "all even (control: the path that always worked)", len: 200, cuts: [14, 100], odd: [false, false, false] },
];

for (const c of CASES) {
    const f = makeFrame(c.len);
    const got = sendChain(f, c.cuts, c.odd);
    const d = firstDiff(f, got);
    ok(got && got.length === f.length && d === -1,
       c.name + (d === -1 ? "" : ` -- first differing byte at offset ${d}` +
                 (got ? ` (want 0x${f[d].toString(16)}, got 0x${(got[d] ?? 0).toString(16)})` : " (no frame)")));
}

console.log("");
if (failed) { console.log(`xqoddbuf.test: FAIL (${failed} of ${passed + failed})`); process.exit(1); }
console.log(`xqoddbuf.test: ALL PASS (${passed})`);
