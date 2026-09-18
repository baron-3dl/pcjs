/**
 * @fileoverview Transport-seam proof: DELQA HubPort EthernetLink <-> the REAL frozen L2 hub (ee9)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * Item pcjsvax-ee9 DONE condition (port spec §5.5): wire two HubPort-backed EthernetLinks as two
 * ports of the REAL hub.mjs (NOT a mock); prove a 0x6007 frame emitted on link A arrives at link B
 * byte-identical and is NOT looped back to A; a frame injected on the hub is filtered + delivered;
 * malformed frames are dropped; every broadcast recipient gets an INDEPENDENT buffer (never alias --
 * never-crash-a-peer, Node C is real VMS).
 *
 * The hub is the exact frozen source vendored from the vms repo origin/main (see vendor/l2/
 * PROVENANCE.txt); this is the real switch core, imported directly and injected -- no reimplementation.
 *
 * Plain node script: `node hubport-seam.test.mjs`.  Prints PASS lines, exits 1 on any failure.
 */
import { L2Hub } from "./vendor/l2/hub.mjs";
import HubPortEthernetLink from "../modules/v2/ethlink-hubport.js";

let failures = 0;
function ok(cond, msg) { if (cond) { console.log("  PASS", msg); } else { console.error("  FAIL", msg); failures++; } }
function eqBytes(a, b) { if (a.length !== b.length) return false; for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false; return true; }

const MAC_A = [0x08, 0x00, 0x2b, 0x00, 0x00, 0x0a];
const MAC_B = [0x08, 0x00, 0x2b, 0x00, 0x00, 0x0b];
const MAC_C = [0x08, 0x00, 0x2b, 0x00, 0x00, 0x0c];
const BCAST = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff];

function frame(dst, src, type = 0x6007, payloadLen = 46, fill = 0x11) {
    const f = new Uint8Array(14 + payloadLen);
    f.set(dst, 0); f.set(src, 6);
    f[12] = (type >> 8) & 0xff; f[13] = type & 0xff;
    for (let i = 0; i < payloadLen; i++) f[14 + i] = fill;
    return f;
}

/* Build the segment: real L2Hub + two DELQA HubPort links (nodes A and B) + a capture of drops. */
const drops = [];
const hub = new L2Hub({ onDrop: (reason, port) => drops.push({ reason, port }) });

const linkA = new HubPortEthernetLink({ name: "OVMXA" }).connectHub(hub);
const linkB = new HubPortEthernetLink({ name: "OVMXB" }).connectHub(hub);
linkA.setFilter([MAC_A, BCAST], false, false);
linkB.setFilter([MAC_B, BCAST], false, false);
let rxA = [], rxB = [];
linkA.setReceiveHandler((f) => rxA.push(f));
linkB.setReceiveHandler((f) => rxB.push(f));
linkA.attach();
linkB.attach();

console.log("segment wired against the real hub.mjs:");
ok(hub.size === 2, `hub has 2 ports (${hub.portNames().join(", ")})`);

console.log("A transmits 0x6007 frame to B (the DELQA <-> hub round-trip):");
{
    rxA = []; rxB = [];
    const tx = frame(MAC_B, MAC_A, 0x6007, 46, 0x6b);
    const forwarded = linkA.send(tx);
    ok(forwarded === true, "hub forwarded the frame to at least one other port");
    ok(rxB.length === 1, "B received exactly one frame");
    ok(rxB.length === 1 && eqBytes(rxB[0], tx), "B's frame is byte-identical to what A transmitted (verbatim, no mutation)");
    ok(rxB.length === 1 && ((rxB[0][12] << 8) | rxB[0][13]) === 0x6007, "ethertype preserved as 0x6007");
    ok(rxA.length === 0, "NOT looped back to A (no-loopback INV)");
}

console.log("A broadcasts -> both B and C receive INDEPENDENT buffers:");
{
    /* add a third node C */
    const linkC = new HubPortEthernetLink({ name: "OVMXC" }).connectHub(hub);
    linkC.setFilter([MAC_C, BCAST], false, false);
    let rxC = [];
    linkC.setReceiveHandler((f) => rxC.push(f));
    linkC.attach();
    rxA = []; rxB = []; rxC = [];

    const tx = frame(BCAST, MAC_A, 0x6007, 46, 0x99);
    linkA.send(tx);
    ok(rxB.length === 1 && rxC.length === 1, "broadcast delivered to both B and C");
    ok(rxA.length === 0, "broadcast not looped back to sender A");
    if (rxB.length === 1 && rxC.length === 1) {
        ok(eqBytes(rxB[0], tx) && eqBytes(rxC[0], tx), "both copies byte-identical to A's frame");
        ok(rxB[0].buffer !== rxC[0].buffer, "B and C got DIFFERENT ArrayBuffers (per-recipient copy)");
        rxB[0][20] ^= 0xff;                             /* corrupt B's copy... */
        ok(rxC[0][20] === 0x99, "...C's copy is unaffected (buffers are not aliased -- never-crash-a-peer)");
    }
    linkC.detach();
    ok(hub.size === 2, "C left the segment cleanly");
}

console.log("a frame injected on the hub is filtered + delivered:");
{
    rxA = []; rxB = [];
    /* an independent injector port (models another NIC backend, e.g. the qemu-wasm node) */
    let rxInj = [];
    const inj = hub.addPort({ name: "INJECTOR", send: (f) => rxInj.push(f) });
    const tx = frame(MAC_B, MAC_C, 0x6007, 46, 0x5a);   /* to B, from C */
    inj.emit(tx);
    ok(rxB.length === 1 && eqBytes(rxB[0], tx), "B (a real DELQA link) received the hub-injected frame verbatim");
    ok(rxA.length === 0, "A did not (unicast to B, A's filter rejects it)");
    inj.remove();
}

console.log("malformed / oversize frames are dropped (never forwarded broken):");
{
    rxA = []; rxB = [];
    const runt = new Uint8Array(10);                    /* < MIN_FRAME 14 */
    const okRunt = linkA.send(runt);
    ok(okRunt === false, "hub refused to forward a runt (<14) frame");
    ok(rxB.length === 0, "B received nothing from the runt");
    const oversize = new Uint8Array(2000);              /* > MAX_FRAME 1600 */
    oversize.set(MAC_B, 0); oversize.set(MAC_A, 6);
    ok(linkA.send(oversize) === false && rxB.length === 0, "hub refused to forward an oversize (>1600) frame");
    ok(drops.length >= 2, `hub reported the drops (${drops.map(d => d.reason).join(", ")})`);
}

console.log("never-crash-a-peer: one peer's RX throwing does not stop delivery to the others:");
{
    rxA = []; rxB = [];
    let errors = [];
    const hub2 = new L2Hub({ onError: (err, port) => errors.push(port) });
    const good = new HubPortEthernetLink({ name: "GOOD" }).connectHub(hub2);
    good.setFilter([MAC_B], false, false);
    let rxGood = [];
    good.setReceiveHandler((f) => rxGood.push(f));
    good.attach();
    /* a hostile/broken peer whose send throws */
    hub2.addPort({ name: "BROKEN", send: () => { throw new Error("peer NIC blew up"); } });
    const src = new HubPortEthernetLink({ name: "SRC" }).connectHub(hub2);
    src.setFilter([MAC_A], false, false);
    src.attach();
    src.send(frame(MAC_B, MAC_A, 0x6007));
    ok(rxGood.length === 1, "the good DELQA peer still received its frame despite a broken peer");
    ok(errors.includes("BROKEN"), "the hub isolated + reported the broken peer's error");
}

linkA.detach();
linkB.detach();
ok(hub.size === 0, "both DELQA links left the segment; hub empty");

console.log(failures ? `\nhubport-seam.test: ${failures} FAILURE(S)` : "\nhubport-seam.test: ALL PASS");
process.exit(failures ? 1 : 0);
