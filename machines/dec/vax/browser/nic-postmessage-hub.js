/**
 * @fileoverview pcjsvax-636 -- the postMessage-backed L2 hub shim that re-points the DELQA's
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * ============================================================================
 * WHAT THIS IS  (item pcjsvax-636; frozen contract openvmx-site demo/cluster/PCJS-NODE-BRIDGE.md)
 * ============================================================================
 * The DELQA already speaks the FROZEN contract-v1 L2 hub-port API through HubPortEthernetLink
 * (modules/v2/ethlink-hubport.js, pcjsvax-ee9), which is TRANSPORT-AGNOSTIC by construction: it
 * binds to anything implementing `hub.addPort({name, send})` and calls `port.emit(frameU8)` on TX.
 * modules/v2/tests/xqhub.test.mjs proves the whole datapath against the REAL in-process L2Hub
 * (hub.mjs).  This module is the SAME hub-port API, but the two directions are re-pointed from the
 * in-process hub to the parent page via postMessage -- so the pcjs VAX machine becomes a browser-
 * embeddable CLUSTER NODE, wired identically to the qemu-wasm Node A (boot/nic-classic.js's
 * installQemuNicWebSocket({onNicTx, deliverToGuest})).  Nothing in the DELQA, EthernetLink or
 * HubPort changes; this is the drop-in replacement for L2Hub on the node side.
 *
 * THE WIRE (contract PCJS-NODE-BRIDGE.md, verbatim), machine worker <-> node page:
 *   machine -> page:  {t:'nic-tx',   frame:<ArrayBuffer>}   guest DELQA transmitted a frame (guest TX)
 *   page -> machine:  {t:'nic-rx',   frame:<ArrayBuffer>}   deliver one frame into the DELQA RX
 *   machine -> page:  {t:'nic-ready'}                       the HubPort is up (page gates wiring on this)
 * FRAME = a contract-v1 raw Ethernet frame (dst[6] src[6] ethertype[2] payload, NO FCS), Uint8Array.
 *
 * NEVER-CRASH-A-PEER (Node C is REAL OpenVMS -- a wrong byte is the crash class):
 *  - TX/RX bytes are forwarded VERBATIM: no mutate/truncate/reorder/inject.
 *  - the frame handed UP is a PRIVATE copy of the device's buffer (the device's frameU8 aliases guest
 *    memory and is reused), so the transfer never neuters guest RAM.
 *  - a malformed/oversize frame (outside [14,1600]) is DROPPED on both directions, never forwarded
 *    broken and never crashing the guest.  _deliverReceive re-checks the same bound RX-side, so the
 *    guard here is the contract boundary + a countable drop, not the only line of defence.
 *  - NO self-hear: emit() posts UP only -- it never loops a TX frame back into the local RX.  (The
 *    parent switch also never floods a frame to its origin port, and EthernetLink._classify rejects
 *    fromMe frames -- three independent guards.)
 *
 * TRANSPORT-AGNOSTIC / TESTABLE: `post` is injected (the worker passes self.postMessage; a Node test
 * passes a capture fn) and inbound frames are fed through `onMessage(msg)`, so the whole bridge is
 * unit-graded under Node against the real XQVAX device with no browser -- see
 * tests/xqcluster.test.mjs (the postMessage analogue of xqhub.test.mjs).
 */

import { ETH_MIN_FRAME, ETH_MAX_FRAME } from "../modules/v2/ethlink.js";

/** true iff `len` is a legal contract-v1 frame length (matches ethlink.js / CONTRACT.md MIN/MAX). */
function inBounds(len) { return len >= ETH_MIN_FRAME && len <= ETH_MAX_FRAME; }

/**
 * createPostMessageHub({post})
 *
 * @param {Object} opts
 *   {function(Object, Array=):void} post   deliver a message toward the page.  In the worker this is
 *                                          `(m, t) => self.postMessage(m, t || [])`; in a Node test
 *                                          it is a capture function.  The second arg is the transfer
 *                                          list (an ArrayBuffer for a nic-tx frame; absent otherwise).
 * @returns {{hub:Object, onMessage:function(Object):boolean, stats:Object, portCount:function():number}}
 *   hub        -- pass to HubPortEthernetLink.connectHub(); implements addPort({name, send}).
 *   onMessage  -- feed each inbound page message here (returns true iff it was a delivered nic-rx).
 *   stats      -- observable counters (tx/txDrop/rx/rxDrop), so drops are provably drops.
 */
export function createPostMessageHub({ post })
{
    if (typeof post !== "function") throw new Error("createPostMessageHub: `post` must be a function");

    const ports = [];                                   /* {name, send} -- one per attached DELQA */
    const stats = { tx: 0, txDrop: 0, rx: 0, rxDrop: 0 };

    const hub = {
        /**
         * addPort({name, send}) -- the frozen contract-v1 hub-port API.  `send(frameU8)` is "deliver
         * an inbound frame INTO this node's RX" (HubPort wires it to EthernetLink._deliverReceive).
         * Returns the handle HubPort holds: { name, emit, remove }.
         */
        addPort({ name = "DELQA", send })
        {
            if (typeof send !== "function") throw new Error("postMessage hub addPort: `send` required");
            const port = { name, send };
            ports.push(port);
            /* The HubPort has attached its egress/ingress -- the DELQA's two hooks are now live.
               Tell the page so it wires the pipe (the node page gates on nic-ready). */
            post({ t: "nic-ready" });
            return {
                name,
                /**
                 * emit(frameU8) -- guest DELQA TX -> parent switch, VERBATIM.  Drops an out-of-bounds
                 * frame (never forwards broken).  Hands UP a PRIVATE copy (never the guest buffer).
                 * @returns {boolean} true iff forwarded up
                 */
                emit(frameU8)
                {
                    if (!frameU8 || !inBounds(frameU8.length)) { stats.txDrop++; return false; }
                    const copy = frameU8.slice();       /* verbatim private copy; transferable */
                    stats.tx++;
                    post({ t: "nic-tx", frame: copy.buffer }, [copy.buffer]);
                    return true;
                },
                remove()
                {
                    const i = ports.indexOf(port);
                    if (i >= 0) ports.splice(i, 1);
                }
            };
        }
    };

    /**
     * onMessage(msg) -- feed one inbound page message.  Delivers a {t:'nic-rx'} frame into the DELQA
     * RX (one private copy per port), VERBATIM; drops a malformed/oversize/undecodable frame without
     * crashing the guest.  Ignores any message that is not a nic-rx.
     * @returns {boolean} true iff a valid nic-rx was delivered to at least one port
     */
    function onMessage(msg)
    {
        if (!msg || typeof msg !== "object" || msg.t !== "nic-rx") return false;
        let frame;
        try { frame = new Uint8Array(msg.frame); } catch (e) { stats.rxDrop++; return false; }
        if (!inBounds(frame.length)) { stats.rxDrop++; return false; }   /* drop malformed, never crash */
        let delivered = false;
        for (const p of ports) {
            /* each recipient gets its OWN copy, exactly as hub.mjs does -- so no port can ever alias
               another's buffer (belt-and-braces; the device copies bytes into guest RAM and mutates
               nothing, and there is normally exactly one port). */
            p.send(frame.slice());
            delivered = true;
        }
        stats.rx++;
        return delivered;
    }

    return { hub, onMessage, stats, portCount: () => ports.length };
}

export default createPostMessageHub;
