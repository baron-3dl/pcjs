/**
 * @fileoverview Unit test for the EthernetLink seam (item pcjsvax-549)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * Proves (port spec docs/design/delqa-port.md §5):
 *   - eth_crc32 matches known vectors (standard Ethernet/zlib CRC-32);
 *   - the _eth_callback accept/reject filter (to_me && !from_me): accepts unicast-to-us, broadcast,
 *     joined-multicast, all-multicast, promiscuous; rejects others and our own reflected frames;
 *   - MOP Ethernet-loopback (type 0x9000) forward frames round-trip to a correct reply.
 *
 * Plain node script: `node ethlink.test.mjs`.  Prints PASS lines, exits 1 on any failure.
 */
import {
    EthernetLink, LoopbackEthernetLink, eth_crc32, eth_hash_key,
    ETHERTYPE_LOOPBACK
} from "../modules/v2/ethlink.js";

let failures = 0;
function ok(cond, msg) { if (cond) { console.log("  PASS", msg); } else { console.error("  FAIL", msg); failures++; } }
function eqBytes(a, b) { if (a.length !== b.length) return false; for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false; return true; }

/* A link that captures what the transport would send (for loopback-reply assertions). */
class CaptureLink extends EthernetLink {
    constructor() { super(); this.sent = []; }
    send(f) { this.sent.push(Uint8Array.from(f)); return true; }
}

const MAC_A = [0x08, 0x00, 0x2b, 0x00, 0x00, 0x0a];
const MAC_B = [0x08, 0x00, 0x2b, 0x00, 0x00, 0x0b];
const MAC_X = [0x08, 0x00, 0x2b, 0x00, 0x00, 0x0c];
const BCAST = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff];
const MCAST = [0xab, 0x00, 0x04, 0x01, 0x00, 0x00];   /* LAVC/cluster-style multicast (group bit set) */

/** Build dst|src|type|payload (no CRC). */
function frame(dst, src, type = 0x6007, payloadLen = 46) {
    const f = new Uint8Array(14 + payloadLen);
    f.set(dst, 0); f.set(src, 6);
    f[12] = (type >> 8) & 0xff; f[13] = type & 0xff;
    for (let i = 0; i < payloadLen; i++) f[14 + i] = i & 0xff;
    return f;
}

console.log("eth_crc32 known vectors:");
{
    const enc = new TextEncoder();
    ok(eth_crc32(0, enc.encode("123456789")) === 0xCBF43926, "CRC32('123456789') == 0xCBF43926");
    ok(eth_crc32(0, new Uint8Array(0)) === 0x00000000, "CRC32('') == 0");
    ok(eth_crc32(0, enc.encode("The quick brown fox jumps over the lazy dog")) === 0x414FA339,
       "CRC32('The quick brown fox...') == 0x414FA339");
    /* hash key is deterministic and in range */
    const k = eth_hash_key(Uint8Array.from(MCAST), 0);
    ok(k >= 0 && k <= 63, `eth_hash_key in [0,63] (got ${k})`);
}

console.log("filter accept/reject (to_me && !from_me):");
{
    const link = new CaptureLink();
    link.setFilter([MAC_A, BCAST], false, false);           /* station A; broadcast in filter (as real setup does) */

    ok(link._classify(frame(MAC_A, MAC_B)).accept === true,  "unicast to us (A), from B -> ACCEPT");
    ok(link._classify(frame(MAC_B, MAC_X)).accept === false, "unicast to B (not us) -> REJECT");
    ok(link._classify(frame(MAC_A, MAC_A)).accept === false, "our own reflected frame (src==A) -> REJECT (from_me)");
    ok(link._classify(frame(BCAST, MAC_B)).accept === true,  "broadcast (in filter), from B -> ACCEPT");
    ok(link._classify(frame(MCAST, MAC_B)).accept === false, "unjoined multicast, no all-mcast -> REJECT");
}
{
    const link = new CaptureLink();
    link.setFilter([MAC_A, MCAST], false, false);            /* joined a specific multicast group */
    ok(link._classify(frame(MCAST, MAC_B)).accept === true,  "joined multicast (exact) -> ACCEPT");
}
{
    const link = new CaptureLink();
    link.setFilter([MAC_A], true, false);                    /* all-multicast on */
    ok(link._classify(frame(MCAST, MAC_B)).accept === true,  "all-multicast: any group frame -> ACCEPT");
    ok(link._classify(frame(MAC_B, MAC_X)).accept === false, "all-multicast: unicast to other -> still REJECT");
}
{
    const link = new CaptureLink();
    link.setFilter([MAC_A], false, true);                    /* promiscuous */
    ok(link._classify(frame(MAC_B, MAC_X)).accept === true,  "promiscuous: unicast to other -> ACCEPT");
    ok(link._classify(frame(MAC_A, MAC_A)).accept === false, "promiscuous: still drops our own (from_me)");
}

console.log("receive-side delivery + malformed drop:");
{
    const link = new CaptureLink();
    link.setFilter([MAC_A], false, false);
    let got = null;
    link.setReceiveHandler((f) => { got = f; });
    const f = frame(MAC_A, MAC_B, 0x6007);
    ok(link._deliverReceive(f) === true && got && eqBytes(got, f), "accepted frame delivered to device RX, byte-identical");
    got = null;
    ok(link._deliverReceive(frame(MAC_B, MAC_X)) === false && got === null, "filtered frame NOT delivered");
    ok(link._deliverReceive(new Uint8Array(13)) === false, "runt (<14) dropped");
    ok(link._deliverReceive(new Uint8Array(1601)) === false, "oversize (>1600) dropped");
    ok(link.stats.rxDropMalformed === 2, "two malformed frames counted as drops");
}

console.log("MOP Ethernet-loopback (0x9000) forward -> reply:");
{
    const link = new CaptureLink();
    link.setFilter([MAC_A], false, false);                   /* station A */
    /* forward-function loopback frame addressed to A, forward-address = B, skipcount 0 */
    const f = new Uint8Array(60);
    f.set(MAC_A, 0); f.set(MAC_X, 6);                         /* dst A, src X (so !from_me) */
    f[12] = 0x90; f[13] = 0x00;                               /* ethertype 0x9000 */
    f[14] = 0x00; f[15] = 0x00;                               /* skipcount 0 -> offset 16 */
    f[16] = 0x02; f[17] = 0x00;                               /* function 2 = forward */
    f.set(MAC_B, 18);                                         /* forward address = B */
    const delivered = link._deliverReceive(f);
    ok(delivered === false, "loopback forward consumed (not delivered up)");
    ok(link.sent.length === 1, "one loopback reply generated");
    if (link.sent.length === 1) {
        const r = link.sent[0];
        ok(eqBytes(r.slice(0, 6), Uint8Array.from(MAC_B)), "reply dst = forward address (B)");
        ok(eqBytes(r.slice(6, 12), Uint8Array.from(MAC_A)), "reply src = our station (A)");
        ok(r[12] === 0x90 && r[13] === 0x00, "reply keeps ethertype 0x9000");
        ok(r[14] === 8 && r[15] === 0, "reply skip/offset rewritten to 8");
        ok(r[16] === 0x02 && r[17] === 0x00, "reply still function=forward");
    }
    /* a non-loopback frame to us is delivered up, not consumed */
    let got = null; link.setReceiveHandler((x) => { got = x; });
    ok(link._deliverReceive(frame(MAC_A, MAC_X, 0x6007)) === true && got, "non-loopback frame to us delivered up");
}

console.log("LoopbackEthernetLink null backend:");
{
    const link = new LoopbackEthernetLink();
    ok(link.send(frame(MAC_B, MAC_A)) === true && link.stats.tx === 1, "null backend: send accepted (dropped on wire)");
    const refl = new LoopbackEthernetLink({ reflect: true });
    refl.setFilter([MAC_A], false, false);
    let got = null; refl.setReceiveHandler((x) => { got = x; });
    refl.send(frame(MAC_A, MAC_X));                          /* external-loopback: comes back, to us, not from us */
    ok(got !== null, "reflect backend: transmitted frame loops back into RX");
}

console.log(failures ? `\nethlink.test: ${failures} FAILURE(S)` : "\nethlink.test: ALL PASS");
process.exit(failures ? 1 : 0);
