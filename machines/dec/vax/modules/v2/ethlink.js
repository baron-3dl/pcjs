/**
 * @fileoverview EthernetLink transport seam for the DELQA (the sim_ether roles, in JS)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * ============================================================================
 * WHAT THIS IS  (item pcjsvax-549; port spec docs/design/delqa-port.md §5)
 * ============================================================================
 * The DELQA device (xq.js) never touches a concrete network.  It talks to an injected EthernetLink
 * that mirrors the ROLES of Open SIMH's sim_ether (not its C API):
 *
 *   send(frameU8)                              -- device TX: a complete Ethernet frame
 *                                                 dst[6] src[6] type[2] payload, NO CRC
 *                                                 (matches ETH_PACK.len, which excludes the 4-byte CRC)
 *   setReceiveHandler(cb)                      -- backend delivers inbound frames via cb(frameU8)
 *   setFilter(macList, allMulticast, promisc)  -- device programs its MAC filter (xq_process_setup)
 *   setHashFilter(hash8)                       -- optional LANCE 64-bit multicast hash (AUTODIN II)
 *   attach() / detach()
 *
 * The base class owns the receive-side accept/reject filter (sim_ether.c:3682-3764, the TAP/UDP path
 * this transport emulates), the eth_crc32 helper (sim_ether.c:555-575, ported verbatim), and MOP
 * Ethernet-loopback (type 0x9000) reply generation (_eth_process_loopback, sim_ether.c:3623-3680).
 * Concrete transports subclass it and implement send() + how inbound frames arrive (they call
 * _deliverReceive).  LoopbackEthernetLink here is the default null backend (device faithful with no
 * network); HubPortEthernetLink (ethlink-hubport.js, item pcjsvax-ee9) binds to the frozen L2 hub.
 *
 * NEVER-CRASH-A-PEER: this seam carries REAL SCS frames to a REAL OpenVMS peer.  It forwards frame
 * bytes verbatim -- never mutate, truncate, reorder, inject, or hand out an aliased buffer.  A
 * malformed/oversize frame is dropped, never forwarded broken.
 */

/** Ethernet frame geometry (bytes).  Matches the frozen L2 contract's MIN_FRAME/MAX_FRAME and
 *  sim_ether.h ETH_MIN/MAX_PACKET (payload sizes; the contract bound 1600 is generous for jumbo). */
export const ETH_ADDR_LEN = 6;
export const ETH_HLEN     = 14;     /* dst[6] + src[6] + type[2] */
export const ETH_MIN_FRAME = 14;    /* header only (contract MIN_FRAME) */
export const ETH_MAX_FRAME = 1600;  /* contract MAX_FRAME */

export const ETHERTYPE_LOOPBACK = 0x9000;  /* MOP Ethernet loopback (DECnet exercises it at startup) */

/* ---------------------------------------------------------------------------
 * eth_crc32 -- sim_ether.c:509-575, ported verbatim.
 * The crcTable there is the standard reflected CRC-32 table (poly 0xEDB88320;
 * crcTable[1]==0x77073096 confirms it), so we generate it at load rather than
 * hardcode 256 constants.  eth_crc32(0, buf, len) is the standard Ethernet/zlib
 * CRC-32 of buf: init/final XOR with 0xFFFFFFFF, exactly as the C does.
 * ------------------------------------------------------------------------- */
const CRC_TABLE = (() => {
    const t = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
        let c = n;
        for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
        t[n] = c >>> 0;
    }
    return t;
})();

/**
 * eth_crc32(crc, u8, off, len) -- CRC-32 over u8[off .. off+len).
 * @param {number} crc seed (0 for a fresh CRC)
 * @param {Uint8Array} u8
 * @param {number} [off]
 * @param {number} [len]
 * @returns {number} unsigned 32-bit CRC
 */
export function eth_crc32(crc, u8, off = 0, len = u8.length - off)
{
    crc = (crc ^ 0xFFFFFFFF) >>> 0;                     /* crc ^= mask */
    for (let i = 0; i < len; i++)
        crc = ((crc >>> 8) ^ CRC_TABLE[(crc ^ u8[off + i]) & 0xFF]) >>> 0;
    return (crc ^ 0xFFFFFFFF) >>> 0;                    /* return crc ^ mask */
}

/**
 * LANCE 6-bit multicast hash key (sim_ether.c:3184-3191): key = 0x3f & (crc32(dst,6) >> 26), then
 * key ^= 0x3f; the multicast address matches iff hash[key>>3] has bit (key&7) set.
 * @param {Uint8Array} u8 frame (uses the first 6 bytes = destination MAC)
 * @param {number} [off]
 * @returns {number} 0..63
 */
export function eth_hash_key(u8, off = 0)
{
    return (0x3f & (eth_crc32(0, u8, off, 6) >>> 26)) ^ 0x3f;
}

/* MAC helpers */
function macEq(u8, aOff, mac)
{
    for (let i = 0; i < ETH_ADDR_LEN; i++) if (u8[aOff + i] !== mac[i]) return false;
    return true;
}
/** true iff the group/multicast bit (LSB of the first octet) is set -- includes broadcast. */
function isGroup(u8, off = 0) { return (u8[off] & 0x01) !== 0; }

/**
 * EthernetLink -- abstract transport base.  The DELQA holds one of these.
 */
export class EthernetLink {
    constructor()
    {
        this._rx = null;                                /* receive handler cb(frameU8) */
        this._attached = false;
        /* MAC filter (programmed by the guest's setup packet via setFilter) */
        this._addrs = [];                               /* array of 6-byte Uint8Array/Array MACs */
        this._allMulticast = false;
        this._promiscuous = false;
        this._hash = null;                              /* optional Uint8Array(8) LANCE hash, or null */
        /* counters -- observable, help the differential and prove drops are drops */
        this.stats = { rx: 0, rxDropFilter: 0, rxDropMalformed: 0, tx: 0, loopReplies: 0 };
    }

    /** @param {function(Uint8Array):void} cb */
    setReceiveHandler(cb) { this._rx = cb; }

    /**
     * Program the MAC filter (from xq_process_setup / eth_filter).  Copies the inputs.
     * @param {Array<Array<number>|Uint8Array>} macList
     * @param {boolean} allMulticast
     * @param {boolean} promiscuous
     */
    setFilter(macList, allMulticast, promiscuous)
    {
        this._addrs = (macList || []).map((m) => Uint8Array.from(m).slice(0, ETH_ADDR_LEN));
        this._allMulticast = !!allMulticast;
        this._promiscuous = !!promiscuous;
    }

    /** Optional LANCE 64-bit multicast hash (AUTODIN II).  @param {Uint8Array|null} hash8 */
    setHashFilter(hash8) { this._hash = hash8 ? Uint8Array.from(hash8).slice(0, 8) : null; }

    /** @returns {boolean} */
    isAttached() { return this._attached; }

    /* Subclasses implement the transport. */
    attach() { this._attached = true; return this; }
    detach() { this._attached = false; return this; }
    /**
     * Device TX.  frameU8 is dst|src|type|payload (NO CRC).
     * @param {Uint8Array} frameU8
     * @returns {boolean} true if handed to the transport
     */
    send(frameU8) { throw new Error("EthernetLink.send() not implemented by " + this.constructor.name); }

    /* ------------------------------------------------------------------
     * Receive side (sim_ether.c:3682-3764).  A concrete transport calls
     * _deliverReceive() for each frame it pulls off the wire.
     * ------------------------------------------------------------------ */

    /**
     * The _eth_callback accept/reject test for the non-BPF (TAP/UDP/NAT) API path -- the model this
     * transport emulates (sim_ether.c:3714-3736,3764).  ACCEPT iff (to_me && !from_me).
     * @param {Uint8Array} u8
     * @returns {{toMe:boolean, fromMe:boolean, accept:boolean}}
     */
    _classify(u8)
    {
        let toMe = false, fromMe = false;
        for (const a of this._addrs) {
            if (macEq(u8, 0, a)) toMe = true;           /* dst == one of our addresses */
            if (macEq(u8, 6, a)) fromMe = true;         /* src == one of our addresses */
        }
        if (this._allMulticast && isGroup(u8, 0)) toMe = true;
        if (this._promiscuous) toMe = true;
        if (this._hash && !toMe && isGroup(u8, 0)) {
            const key = eth_hash_key(u8, 0);
            if (this._hash[key >> 3] & (1 << (key & 7))) toMe = true;
        }
        return { toMe, fromMe, accept: toMe && !fromMe };
    }

    /**
     * MOP Ethernet-loopback forward-function handler (_eth_process_loopback, sim_ether.c:3623-3680).
     * If `u8` is a forward-function loopback frame addressed to us, build + send the reply and return
     * true (packet consumed).  Otherwise return false (deliver it up normally).
     * @param {Uint8Array} u8
     * @returns {boolean} true iff consumed as a loopback forward
     */
    _processLoopback(u8)
    {
        const len = u8.length;
        const protocol = u8[12] | (u8[13] << 8);        /* NB: byte-swapped compare, exactly as the C */
        if (protocol !== 0x0090) return false;          /* not ethernet loopback */
        let offset = 16 + (u8[14] | (u8[15] << 8));
        if (offset >= len) return false;
        const func = u8[offset] | (u8[offset + 1] << 8);
        if (func !== 2) return false;                   /* only 'forward' */

        /* only respond to frames directed at us (or bcast/mcast we're listening to); the C checks
         * filter_address[0] for the unicast case (sim_ether.c:3649-3651). */
        const station = this._addrs[0];
        if (!isGroup(u8, 0) && (!station || !macEq(u8, 0, station))) return false;
        /* forwarding TO a multicast/broadcast forward-address is consumed and ignored (3653-3656) */
        if (u8[offset + 2] & 0x01) return true;

        /* build forward response (3662-3673): dst = forward address (u8[offset+2..]),
         * src = our station MAC, then rewrite the skip-count at bytes 14/15. */
        const resp = u8.slice();                        /* private copy; never alias */
        for (let i = 0; i < ETH_ADDR_LEN; i++) {
            resp[i] = u8[offset + 2 + i];               /* new Ethernet destination */
            resp[6 + i] = station ? station[i] : u8[6 + i];  /* new Ethernet source */
        }
        offset += 8 - 16;                               /* account for header + offset (3668) */
        resp[14] = offset & 0xFF;
        resp[15] = (offset >> 8) & 0xFF;
        this.stats.loopReplies++;
        this.send(resp);
        return true;
    }

    /**
     * Entry point a transport calls for each inbound frame.  Applies the filter, handles MOP
     * loopback, else delivers to the device receive handler.  NEVER mutates the caller's buffer.
     * @param {Uint8Array} frameU8
     * @returns {boolean} true iff delivered up to the device
     */
    _deliverReceive(frameU8)
    {
        if (!frameU8 || frameU8.length < ETH_MIN_FRAME || frameU8.length > ETH_MAX_FRAME) {
            this.stats.rxDropMalformed++;               /* drop, don't crash (contract INV 3) */
            return false;
        }
        if (!this._classify(frameU8).accept) { this.stats.rxDropFilter++; return false; }
        if (this._processLoopback(frameU8)) { this.stats.rx++; return false; } /* consumed as loopback */
        this.stats.rx++;
        if (this._rx) this._rx(frameU8);
        return true;
    }
}

/**
 * LoopbackEthernetLink -- the default null backend.  Models an oracle with `set xq enable` and no
 * tap: the device is faithful with no network attached.  send() drops (nothing on the wire), so the
 * only frames the guest ever "receives" are the internal/external loopback packets that the DELQA's
 * own xq_process_xbdl path enqueues directly (port spec §4) -- this backend is not on that path.
 *
 * With {reflect:true} it feeds each transmitted frame back into its own receive path (a physical
 * external-loopback connector), which lets MOP loopback and diagnostics round-trip with no hub.
 */
export class LoopbackEthernetLink extends EthernetLink {
    constructor({ reflect = false } = {}) { super(); this._reflect = reflect; }

    send(frameU8)
    {
        if (!frameU8 || frameU8.length < ETH_MIN_FRAME || frameU8.length > ETH_MAX_FRAME) return false;
        this.stats.tx++;
        if (this._reflect) {
            /* external loopback connector: our own frame comes back on the wire, verbatim copy */
            this._deliverReceive(frameU8.slice());
        }
        return true;                                    /* accepted (and dropped, unless reflecting) */
    }
}

export default EthernetLink;
