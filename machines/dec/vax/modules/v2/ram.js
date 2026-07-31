/**
 * @fileoverview Implements the VAX RAM component (pcjsvax-f23)
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * `<ram addr size/>`, in the shape of machines/dec/pdp11/modules/v2/ram.js.  Like rom.js here, it
 * does NOT allocate anything: the memory blocks live in the Worker (browser/vaxworker.js), so this
 * component exists to carry the machine XML's declared size into computer.js's power-up message and
 * to give the page a place to change it.
 *
 * The KA655's memory controller (modules/v2/cmctl.js) reports its size through the CMCTL registers,
 * and the ROM's memory sizing test reads them, so the size is not decorative -- a machine that
 * declares 16 MB boots with `>>> SHOW MEM` reporting 16 MB.
 */

import Component from "../../../../modules/v2/component.js";
import WebLib from "../../../../modules/v2/weblib.js";
import MESSAGE from "./message.js";
import { APPCLASS } from "./defines.js";

/**
 * @class RAMVAX
 * @unrestricted
 */
export default class RAMVAX extends Component {
    /**
     * RAMVAX(parmsRAM)
     *
     * The RAMVAX component expects the following (parmsRAM) properties:
     *
     *      addr: starting physical address (0 on a KA655; declared for symmetry with PDPjs)
     *      size: amount of RAM, in bytes
     *
     * @param {Object} parmsRAM
     */
    constructor(parmsRAM)
    {
        super("RAM", parmsRAM, MESSAGE.MEMORY);

        this.addrRAM = +parmsRAM['addr'] || 0;
        this.sizeRAM = +parmsRAM['size'] || (16 * 1024 * 1024);
        this.setReady();
    }

    /**
     * getSize()
     *
     * @this {RAMVAX}
     * @returns {number} size in bytes
     */
    getSize()
    {
        return this.sizeRAM;
    }

    /**
     * setBinding(sHTMLType, sBinding, control, sValue)
     *
     * @this {RAMVAX}
     * @param {string} sHTMLType
     * @param {string} sBinding ("memSize")
     * @param {HTMLElement} control
     * @param {string} [sValue]
     * @returns {boolean}
     */
    setBinding(sHTMLType, sBinding, control, sValue)
    {
        let ram = this;
        switch (sBinding) {
        case "memSize":
            this.bindings[sBinding] = control;
            control.value = String(this.sizeRAM / (1024 * 1024));
            control.onchange = function onMemSizeChange() {
                ram.sizeRAM = (parseInt(control.value, 10) || 16) * 1024 * 1024;
            };
            return true;
        }
        return false;
    }

    /**
     * powerUp(data, fRepower)
     *
     * @this {RAMVAX}
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
     * @this {RAMVAX}
     * @param {boolean} [fSave]
     * @param {boolean} [fShutdown]
     * @returns {Object|boolean}
     */
    powerDown(fSave, fShutdown)
    {
        return true;
    }

    /**
     * RAMVAX.init()
     */
    static init()
    {
        let aeRAM = Component.getElementsByClass(APPCLASS, "ram");
        for (let iRAM = 0; iRAM < aeRAM.length; iRAM++) {
            let eRAM = aeRAM[iRAM];
            let parmsRAM = Component.getComponentParms(eRAM);
            let ram = new RAMVAX(parmsRAM);
            Component.bindComponentControls(ram, eRAM, APPCLASS);
        }
    }
}

WebLib.onInit(RAMVAX.init);
