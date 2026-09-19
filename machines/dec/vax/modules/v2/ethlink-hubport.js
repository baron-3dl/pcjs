/**
 * @fileoverview HubPort EthernetLink backend -- binds the DELQA to the frozen L2 hub port API
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * ============================================================================
 * WHAT THIS IS  (item pcjsvax-ee9; port spec docs/design/delqa-port.md §5.5)
 * ============================================================================
 * A concrete EthernetLink transport that connects the DELQA to the in-browser L2 switch defined by
 * the FROZEN seam contract v1:
 *   vms repo tools/cluster-web-demo/CONTRACT.md  +  l2/hub.mjs  (L2Hub)  +  l2/wire.mjs
 *
 * The hub is TRANSPORT-AGNOSTIC and INJECTED (never imported here) so this module has no dependency
 * on the vms repo and is testable in-process against the real hub.mjs (import it in the test and pass
 * it to connectHub) before any iframe/postMessage wiring exists.
 *
 * The contract seam (CONTRACT.md "The port API"):
 *   const port = hub.addPort({ name, send: (frameU8) => <inject into THIS node's RX> });
 *   port.emit(frameU8);   // call when this node TRANSMITS -> hub floods to every OTHER port
 *   port.remove();        // leave the segment
 *
 * Binding:
 *   device TX   -> EthernetLink.send(frameU8) -> port.emit(frameU8)      (hub floods to others)
 *   hub inbound -> port's send callback -> EthernetLink._deliverReceive  (filter, then device RX)
 *
 * Contract/never-crash-a-peer invariants honored (all also enforced hub-side; we never violate them):
 *  - forward the frame VERBATIM: no mutate/truncate/reorder/inject;
 *  - the hub hands each recipient its OWN copy (hub.mjs emit does u8.slice()), so the frame this link
 *    receives is already private -- we never alias a buffer across ports;
 *  - out-of-bounds frames ([14,1600]) are dropped by the hub on emit and by _deliverReceive on RX.
 */

import EthernetLink from "./ethlink.js";

/**
 * HubPortEthernetLink -- one DELQA's port on an L2Hub.
 */
export default class HubPortEthernetLink extends EthernetLink {
    /**
     * @param {object} [opts]
     * @param {string} [opts.name] port name for diagnostics/UI
     */
    constructor({ name = "DELQA" } = {})
    {
        super();
        this._name = name;
        this._hub = null;
        this._port = null;                              /* the hub handle: { emit, remove, ... } */
    }

    /**
     * Inject the L2 switch (an L2Hub instance, or anything implementing addPort).  Transport-agnostic:
     * the same code works against the real in-process hub.mjs and against a postMessage-backed shim.
     * @param {{addPort:function(object):object}} hub
     * @returns {HubPortEthernetLink} this
     */
    connectHub(hub) { this._hub = hub; return this; }

    /** @override */
    attach()
    {
        if (this._port) return this;                    /* idempotent */
        if (!this._hub) throw new Error("HubPortEthernetLink.attach(): no hub -- call connectHub() first");
        /* the hub's `send` = "deliver an inbound frame INTO this node's RX"; wire it to our receive
         * path.  The hub already gave us a private copy, so _deliverReceive may hold it safely. */
        this._port = this._hub.addPort({
            name: this._name,
            send: (frameU8) => this._deliverReceive(frameU8),
        });
        this._attached = true;
        return this;
    }

    /** @override */
    detach()
    {
        if (this._port) { this._port.remove(); this._port = null; }
        this._attached = false;
        return this;
    }

    /**
     * Device TX -> hub flood.  Verbatim; the hub validates length and copies per recipient.
     * @override
     * @param {Uint8Array} frameU8
     * @returns {boolean} true iff the hub forwarded it to at least one other port
     */
    send(frameU8)
    {
        if (!this._port) return false;
        this.stats.tx++;
        return this._port.emit(frameU8);
    }

    /** @returns {string} */
    get portName() { return this._port ? this._port.name : this._name; }
}
