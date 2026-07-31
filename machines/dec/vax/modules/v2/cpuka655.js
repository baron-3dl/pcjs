/**
 * @fileoverview Implements the VAX `<cpu>` machine-XML component (pcjsvax-f23)
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY THIS FILE IS NOT CALLED cpu.js.  In PDPjs, machines/dec/pdp11/modules/v2/cpu.js IS the
 * Component that a `<cpu>` element instantiates.  Here `modules/v2/cpu.js` was already taken, years
 * of this port earlier, by the SIMH-derived Base Instruction Group EXECUTOR -- 107 opcodes graded
 * by cpudiff.js.  Renaming it to make room for a framework class would touch a file the 35-check
 * gate grades, for no behavioural gain, so the framework class lives here instead.
 *
 * WHAT IT DOES.  It carries `model`, `cycles` and `autoStart` from the machine XML, and it owns the
 * "Run"/"Pause" bindings.  It does NOT execute anything: the instruction stream is in the Worker
 * (browser/vaxworker.js), and computer.js is what talks to it.  This component is the machine XML's
 * handle on that -- so `<cpu autoStart="true"/>` means the same thing it means on every other PCjs
 * machine, "start executing as soon as the machine is powered".
 */

import Component from "../../../../modules/v2/component.js";
import WebLib from "../../../../modules/v2/weblib.js";
import MESSAGE from "./message.js";
import { APPCLASS } from "./defines.js";

/**
 * @class CPUVAX
 * @unrestricted
 */
export default class CPUVAX extends Component {
    /**
     * CPUVAX(parmsCPU)
     *
     * The CPUVAX component expects the following (parmsCPU) properties:
     *
     *      model: "KA655" (the only model this port implements)
     *      cycles: cycles per second (informational; the Worker measures its own rate)
     *      autoStart: true to begin executing as soon as the machine is powered
     *      autoBoot: true to run the console auto-boot script (see browser/vaxmachine.js AUTO_RULES)
     *
     * @param {Object} parmsCPU
     */
    constructor(parmsCPU)
    {
        super("CPU", parmsCPU, MESSAGE.CPU);

        this.model = parmsCPU['model'] || "KA655";
        this.nCyclesPerSecond = +parmsCPU['cycles'] || 0;
        this.fAutoStart = (parmsCPU['autoStart'] !== false && parmsCPU['autoStart'] !== "false");
        /*
         * `autoBoot` is NOT a PCjs-wide attribute; it is this machine's, and it is separate from
         * `autoStart` on purpose.  `autoStart` decides whether the CPU runs; `autoBoot` decides
         * whether the console auto-boot script types `B DUA0` and answers VMS's date prompt.  A
         * visitor who wants the `>>>` prompt and nothing else turns the second one off and still
         * gets a running machine.
         */
        this.fAutoBoot = (parmsCPU['autoBoot'] !== false && parmsCPU['autoBoot'] !== "false");

        /** @type {ComputerVAX|null} set by computer.js */
        this.cmp = null;
        this.flags.running = false;
        this.setReady();
    }

    /**
     * setBinding(sHTMLType, sBinding, control, sValue)
     *
     * @this {CPUVAX}
     * @param {string} sHTMLType
     * @param {string} sBinding ("run", "pause", "autoBoot")
     * @param {HTMLElement} control
     * @param {string} [sValue]
     * @returns {boolean}
     */
    setBinding(sHTMLType, sBinding, control, sValue)
    {
        let cpu = this;
        switch (sBinding) {
        case "run":
            this.bindings[sBinding] = control;
            control.onclick = function onClickRun() {
                if (cpu.cmp) cpu.cmp.startCPU();
            };
            return true;
        case "pause":
            this.bindings[sBinding] = control;
            control.onclick = function onClickPause() {
                if (cpu.cmp) cpu.cmp.pauseCPU();
            };
            return true;
        }
        return false;
    }

    /**
     * updateStatus(fRunning)
     *
     * @this {CPUVAX}
     * @param {boolean} fRunning
     */
    updateStatus(fRunning)
    {
        this.flags.running = fRunning;
        let controlRun = this.bindings["run"], controlPause = this.bindings["pause"];
        if (controlRun) controlRun.disabled = fRunning;
        if (controlPause) {
            controlPause.disabled = false;
            controlPause.textContent = fRunning ? "Pause" : "Resume";
        }
    }

    /**
     * powerUp(data, fRepower)
     *
     * @this {CPUVAX}
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
     * @this {CPUVAX}
     * @param {boolean} [fSave]
     * @param {boolean} [fShutdown]
     * @returns {Object|boolean}
     */
    powerDown(fSave, fShutdown)
    {
        return true;
    }

    /**
     * CPUVAX.init()
     */
    static init()
    {
        let aeCPUs = Component.getElementsByClass(APPCLASS, "cpu");
        for (let iCPU = 0; iCPU < aeCPUs.length; iCPU++) {
            let eCPU = aeCPUs[iCPU];
            let parmsCPU = Component.getComponentParms(eCPU);
            let cpu = new CPUVAX(parmsCPU);
            Component.bindComponentControls(cpu, eCPU, APPCLASS);
        }
    }
}

WebLib.onInit(CPUVAX.init);
