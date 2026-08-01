/**
 * @fileoverview Implements the VAX console serial port component (pcjsvax-f23)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2012-2026 Jeff Parsons, © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 * Portions adapted from PCjs, Copyright © 2012-2026 Jeff Parsons, used under the MIT
 * license.
 *
 * `<serial id="console" adapter="0" binding="print"/>`, in the shape of
 * machines/dec/pdp11/modules/v2/serial.js's `<serial id="dl11" adapter="0" binding="print"/>`.
 *
 * WHAT IT BINDS TO.  A control whose binding is "print" -- upstream PDPjs resolves that against the
 * Panel component's print textarea via Component.bindExternalControl(); here the control is a child
 * of the `<serial>` element itself, which is the same mechanism (`<control type="textarea"
 * binding="print"/>`) without requiring a Panel this machine does not have.
 *
 * WHAT DRIVES IT.  The KA655 console UART (modules/v2/console.js) lives in the Worker, so this
 * component does not touch RXDB/TXDB.  computer.js posts the bytes the Worker drains out of
 * console.js's txdbWr() to receiveOutput(), and hands whatever the user types back to the Worker.
 * The screen model is browser/vaxterm.js's VaxTerminal -- a GLASS TTY, not a VT100; EDT and other
 * screen programs will not paint until pcjsvax-582 lands, which is stated rather than discovered.
 */

import Component from "../../../../modules/v2/component.js";
import WebLib from "../../../../modules/v2/weblib.js";
import MESSAGE from "./message.js";
import { APPCLASS } from "./defines.js";
import { VaxTerminal } from "../../browser/vaxterm.js";

/**
 * @class SerialPortVAX
 * @unrestricted
 */
export default class SerialPortVAX extends Component {
    /**
     * SerialPortVAX(parmsSerial)
     *
     * The SerialPortVAX component expects the following (parmsSerial) properties:
     *
     *      adapter: 0 for the console line (the only line a KA655 has on the CPU module)
     *      binding: the name of the control this port prints to (eg, "print")
     *
     * @param {Object} parmsSerial
     */
    constructor(parmsSerial)
    {
        super("SerialPort", parmsSerial, MESSAGE.DEVICE);

        this.iAdapter = +parmsSerial['adapter'] || 0;
        this.sBinding = parmsSerial['binding'];
        this.term = null;
        /** @type {function(string)|null} set by computer.js; where keystrokes go */
        this.sendInput = null;
        /** Keystrokes typed before the machine is powered are DROPPED rather than queued: a
            character the guest never saw is less confusing than one that arrives minutes later. */
        this.setReady();
    }

    /**
     * setBinding(sHTMLType, sBinding, control, sValue)
     *
     * @this {SerialPortVAX}
     * @param {string} sHTMLType
     * @param {string} sBinding ("print")
     * @param {HTMLElement} control
     * @param {string} [sValue]
     * @returns {boolean}
     */
    setBinding(sHTMLType, sBinding, control, sValue)
    {
        if (sBinding != this.sBinding && sBinding != "print") return false;

        this.bindings[sBinding] = control;
        /*
         * The XSL emits `readonly` on a textarea, which is right for a component that only ever
         * prints.  This one also READS, so the flag comes off; every key is preventDefault()ed by
         * VaxTerminal, so the control's own value is never edited by the browser.
         */
        if (control.readOnly !== undefined) control.readOnly = false;

        let serial = this;
        this.term = new VaxTerminal(control, function onInput(sText) {
            if (serial.sendInput) serial.sendInput(sText);
        });
        /*
         * One repaint per frame rather than one per message: a boot posts console bytes ~60 times a
         * second, and re-rendering the scrollback on each one is the only thing on this page that
         * could drop frames.
         */
        (function frame() {
            serial.term.render();
            (globals_requestAnimationFrame())(frame);
        })();
        return true;
    }

    /**
     * receiveOutput(bytes)
     *
     * @this {SerialPortVAX}
     * @param {Uint8Array} bytes
     */
    receiveOutput(bytes)
    {
        if (this.term) this.term.write(bytes);
    }

    /**
     * clear()
     *
     * @this {SerialPortVAX}
     */
    clear()
    {
        if (this.term) this.term.clear();
    }

    /**
     * focus()
     *
     * @this {SerialPortVAX}
     */
    focus()
    {
        let control = this.bindings[this.sBinding] || this.bindings["print"];
        if (control && control.focus) control.focus();
    }

    /**
     * powerUp(data, fRepower)
     *
     * @this {SerialPortVAX}
     * @param {Object|null} data
     * @param {boolean} [fRepower]
     * @returns {boolean}
     */
    powerUp(data, fRepower)
    {
        return true;
    }

    /**
     * powerDown(fSave, fShutdown)
     *
     * @this {SerialPortVAX}
     * @param {boolean} [fSave]
     * @param {boolean} [fShutdown]
     * @returns {Object|boolean}
     */
    powerDown(fSave, fShutdown)
    {
        return true;
    }

    /**
     * SerialPortVAX.init()
     */
    static init()
    {
        let aeSerial = Component.getElementsByClass(APPCLASS, "serial");
        for (let iSerial = 0; iSerial < aeSerial.length; iSerial++) {
            let eSerial = aeSerial[iSerial];
            let parmsSerial = Component.getComponentParms(eSerial);
            let serial = new SerialPortVAX(parmsSerial);
            Component.bindComponentControls(serial, eSerial, APPCLASS);
        }
    }
}

/**
 * globals_requestAnimationFrame()
 *
 * A tiny indirection so this module can be imported under Node (where the Component layer is not
 * exercised, but the import graph still has to evaluate) without referencing a browser global at
 * module scope.
 *
 * @returns {function(function())}
 */
function globals_requestAnimationFrame()
{
    if (typeof requestAnimationFrame == "function") return requestAnimationFrame;
    return function(fn) { setTimeout(fn, 16); };
}

WebLib.onInit(SerialPortVAX.init);
