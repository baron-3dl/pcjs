/**
 * @fileoverview Implements the VAX Computer component (pcjsvax-f23)
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this
 * work.
 *
 * ============================================================================================
 * WHAT THIS IS -- AND THE ONE PLACE THIS MACHINE DIFFERS FROM EVERY OTHER PCjs MACHINE
 * ============================================================================================
 * This is the `<computer>` element of a VAX machine XML, and it does the job ComputerPDP11 does:
 * find the other components the XSL instantiated, WAIT until every one of them reports ready
 * (which is what makes `<rom file="..."/>` and the RQDX3's `autoMount` work -- the machine does not
 * power until the resources have arrived), then power them up in order.
 *
 * WHERE IT DIFFERS.  ComputerPDP11 constructs a Bus and calls initBus() on everything, because the
 * CPU, the memory blocks and the devices are all main-thread Components.  Here they are NOT: the
 * KA655, its bus, its memory and its Qbus devices run in a WEB WORKER, and this component's power-up
 * is a `postMessage`.
 *
 * THAT IS FORCED, NOT PREFERRED, and it is worth being exact about why, because it is the single
 * reason this machine is not a straight Component port.  modules/v2/rq.js's diskRead() is
 *
 *      let n = u.image.read(offset, want, u.rqxb);
 *
 * -- a SYNCHRONOUS read issued from inside the instruction stream while an MSCP command is being
 * serviced.  There is no await to put anywhere; the CPU is mid-instruction.  On the main thread a
 * browser has NO synchronous way to read a slice of a File: FileReader is callback-based,
 * Blob.arrayBuffer() is a promise, and synchronous XHR is deprecated there.  So a main-thread port
 * would have to hold the entire container in memory, and the container is 159,334,400 bytes for
 * VMS 5.5 (1 GB for V7.1).  In a Worker, FileReaderSync over File.slice() exists, and a boot
 * touches a MEASURED 55.3 MiB of 1000 with a 16 MiB cache ceiling.
 *
 * So: the machine XML, the XSLT, the Component lifecycle, the bindings and the media paths are all
 * PCjs's.  What crosses the Worker boundary is exactly four things -- the ROM bytes, the RAM size,
 * the DUA0 File, and the auto-boot flag -- and what comes back is console output and statistics.
 *
 * WHAT IS NOT WIRED, stated rather than left to be discovered:
 *   - There is no save/restore.  PCjs's State/localStorage resume path is not implemented, because
 *     the machine state lives in the Worker and pushing it across is a separate piece of work.
 *     pcjsvax-ad1 (cpustate.js's restore() has no length validation) therefore stays UNREACHABLE.
 *   - There is no Debugger and no Panel component.
 *   - There is no `<ram file=...>` preload and no `<rom notify=...>`.
 */

import Component from "../../../../modules/v2/component.js";
import WebLib from "../../../../modules/v2/weblib.js";
import MESSAGE from "./message.js";
import { APPCLASS, APPNAME, APPVERSION, COPYRIGHT, DEBUG, LICENSE, globals } from "./defines.js";

/**
 * @class ComputerVAX
 * @unrestricted
 */
export default class ComputerVAX extends Component {
    /**
     * ComputerVAX(parmsComputer, parmsMachine, fSuspended)
     *
     * The ComputerVAX component expects the following (parmsComputer) properties:
     *
     *      autoPower: true to power the machine as soon as every component is ready (default)
     *      busWidth: number of physical address bits (30 on a KA655; informational here)
     *
     * @param {Object} parmsComputer
     * @param {Object} [parmsMachine]
     * @param {boolean} [fSuspended]
     */
    constructor(parmsComputer, parmsMachine, fSuspended)
    {
        super("Computer", parmsComputer, MESSAGE.COMPUTER);

        this.flags.powered = false;
        this.parmsMachine = parmsMachine || {};
        this.nBusWidth = +parmsComputer['busWidth'] || 30;
        this.fAutoPower = (parmsComputer['autoPower'] !== false && parmsComputer['autoPower'] !== "false");

        this.worker = null;
        this.fRunning = false;
        this.sStatus = "";
        this.nSteps = 0;

        this.cpu    = /** @type {CPUVAX} */      (Component.getComponentByType("CPU", this.id));
        this.ram    = /** @type {RAMVAX} */      (Component.getComponentByType("RAM", this.id));
        this.rom    = /** @type {ROMVAX} */      (Component.getComponentByType("ROM", this.id));
        this.serial = /** @type {SerialPortVAX} */ (Component.getComponentByType("SerialPort", this.id));
        this.rqdx3  = /** @type {RQDX3} */       (Component.getComponentByType("RQDX3", this.id));

        if (!this.cpu) {
            Component.error("Unable to find CPU component");
            return;
        }
        this.cpu.cmp = this;

        if (this.serial) {
            let computer = this;
            this.serial.sendInput = function(sText) { computer.sendInput(sText); };
        }

        this.printf(MESSAGE.NONE, "%s v%s\n%s\n%s\n", APPNAME, APPVERSION, COPYRIGHT, LICENSE);
        this.printf(MESSAGE.NONE, "Portions adapted from the Open SIMH VAX simulator, © 1998-2019 Robert M Supnik\n");

        /*
         * The test hooks tests/browserboot.mjs and tests/machineboot.mjs read.  They are the SAME
         * names browser/vax.html exposes, deliberately: the grading vocabulary should not change
         * just because the page that hosts the machine did.
         */
        let computerSelf = this;
        globals.window['vaxTerm'] = this.serial && this.serial.term;
        globals.window['vaxType'] = function(s) { computerSelf.sendInput(s); };
        globals.window['vaxStatus'] = function() { return computerSelf.sStatus; };
        globals.window['vaxError'] = null;
        globals.window['vaxMachine'] = this;

        this.setReady();
        this.wait(this.donePowerOn);
    }

    /**
     * wait(fn, parms)
     *
     * Waits for every component in this machine to report ready, then calls fn.  Verbatim in
     * mechanism from ComputerPDP11.wait() (computer.js:414) -- this is the hook that makes a
     * `<rom file="...">` load and an RQDX3 `autoMount` gate the power-up.
     *
     * @this {ComputerVAX}
     * @param {function(Object|undefined)} fn
     * @param {Object} [parms]
     */
    wait(fn, parms)
    {
        let computer = this;
        let aComponents = Component.getComponents(this.id);
        for (let iComponent = 0; iComponent <= aComponents.length; iComponent++) {
            let component = (iComponent < aComponents.length ? aComponents[iComponent] : this);
            if (!component.isReady()) {
                component.isReady(function onComponentReady() {
                    computer.wait(fn, parms);
                });
                return;
            }
        }
        if (DEBUG) this.printf("ComputerVAX.wait(ready)\n");
        fn.call(this, parms);
    }

    /**
     * donePowerOn()
     *
     * Every component is ready.  Power them up in order -- the CPU last, exactly as PCjs does, so
     * that it is assured everything else is "powered" -- and then start executing if the machine's
     * `<cpu autoStart="true"/>` says to.
     *
     * @this {ComputerVAX}
     */
    donePowerOn()
    {
        if (this.flags.powered) return;

        let aComponents = Component.getComponents(this.id);
        for (let i = 0; i < aComponents.length; i++) {
            let component = aComponents[i];
            if (component === this || component === this.cpu) continue;
            if (component.powerUp) component.powerUp(null, false);
            component.flags.powered = true;
        }
        if (this.cpu.powerUp) this.cpu.powerUp(null, false);
        this.cpu.flags.powered = true;
        this.flags.powered = true;

        if (this.serial) globals.window['vaxTerm'] = this.serial.term;

        this.status(this.describeConfig());

        if (!this.fAutoPower) return;
        if (this.cpu.fAutoStart) this.startCPU();
    }

    /**
     * describeConfig()
     *
     * @this {ComputerVAX}
     * @returns {string}
     */
    describeConfig()
    {
        let drive = this.rqdx3 && this.rqdx3.getDrive("DUA0");
        return `${this.cpu.model}, ${(this.ram ? this.ram.getSize() : 0) / (1024 * 1024)} MB` +
               (drive ? `, DUA0 ${drive.sDiskName} (${drive.file.size.toLocaleString()} bytes)` : ", no DUA0") +
               (this.rom && this.rom.getData() ? `, ${this.rom.getData().length}-byte ROM` : ", NO ROM");
    }

    /**
     * startCPU()
     *
     * @this {ComputerVAX}
     */
    startCPU()
    {
        if (!this.flags.powered) return;
        if (this.worker) {                              // already built: this is a resume
            if (!this.fRunning) {
                this.worker.postMessage({cmd: "resume"});
                this.fRunning = true;
                this.cpu.updateStatus(true);
            }
            return;
        }
        let abROM = this.rom && this.rom.getData();
        if (!abROM) {
            this.status("No ROM loaded -- the machine cannot be started");
            return;
        }
        let drive = this.rqdx3 && this.rqdx3.getDrive("DUA0");

        let computer = this;
        /*
         * The Worker is the SAME one browser/vax.html drives (browser/vaxworker.js): one machine
         * transport, graded under Node by tests/vaxbrowserboot.js.  What changed with pcjsvax-f23
         * is who supplies its inputs -- a machine XML and its components, rather than a hand-written
         * page and its query parameters.
         */
        this.worker = new Worker("/machines/dec/vax/browser/vaxworker.js", {type: "module"});
        this.worker.onerror = function(e) {
            globals.window['vaxError'] = `${e.message} (${e.filename}:${e.lineno})`;
            computer.status(`Worker error: ${globals.window['vaxError']}`);
        };
        this.worker.onmessage = function(e) { computer.onWorkerMessage(e.data); };

        /*
         * A COPY of the ROM bytes is transferred, not the component's own array: the ROM component
         * keeps its data so the machine can be restarted without re-fetching it.
         */
        let ab = abROM.slice().buffer;
        this.worker.postMessage({
            cmd: "start",
            rom: ab,
            disk: drive ? {file: drive.file} : null,
            memMB: (this.ram ? this.ram.getSize() : 16 * 1024 * 1024) / (1024 * 1024),
            autoBoot: this.cpu.fAutoBoot
        }, [ab]);

        this.fRunning = true;
        this.cpu.updateStatus(true);
        if (this.serial) this.serial.focus();
    }

    /**
     * pauseCPU()
     *
     * @this {ComputerVAX}
     */
    pauseCPU()
    {
        if (!this.worker) return;
        if (this.fRunning) {
            this.worker.postMessage({cmd: "pause"});
            this.fRunning = false;
        } else {
            this.worker.postMessage({cmd: "resume"});
            this.fRunning = true;
        }
        this.cpu.updateStatus(this.fRunning);
    }

    /**
     * sendInput(sText)
     *
     * @this {ComputerVAX}
     * @param {string} sText
     */
    sendInput(sText)
    {
        if (this.worker && this.fRunning) this.worker.postMessage({cmd: "input", text: sText});
    }

    /**
     * onWorkerMessage(m)
     *
     * @this {ComputerVAX}
     * @param {Object} m
     */
    onWorkerMessage(m)
    {
        switch (m.type) {

        case "out":
            if (this.serial) this.serial.receiveOutput(new Uint8Array(m.bytes));
            break;

        case "info": {
            let a = m.attach;
            this.attach(
                `DUA0  ${m.diskName || "(no disk)"}\n` +
                (a ? `      container ${a.containerBytes.toLocaleString()} bytes; volume declares ` +
                     `${a.filesystemBytes === undefined ? "(not ODS-2)" : a.filesystemBytes.toLocaleString()}; ` +
                     `unit ${a.capacBefore.toLocaleString()} -> ${a.capacAfter.toLocaleString()} blocks, ` +
                     `${a.readOnly ? "READ-ONLY (OpenVMS will stop at MOUNTVER)" : "read/write via copy-on-write overlay"}\n` : "") +
                `RAM   ${(m.memBytes / (1 << 20)).toFixed(0)} MB\n` +
                `Bus   ${m.devices.join(", ")}`);
            break;
        }

        case "stat": {
            let d = m.disk;
            this.nSteps = m.steps;
            this.status(`${(m.steps / 1e6).toFixed(1)}M instructions in ${m.elapsed.toFixed(0)}s ` +
                `(${m.mips.toFixed(2)}M instr/s)` +
                (m.idlePct !== undefined ? ` | idle ${m.idlePct.toFixed(0)}%` : "") +
                (d ? ` | disk ${d.reads} reads ${d.writes} writes, ${ComputerVAX.mib(d.rawBytes)} of the image touched` +
                     ` | overlay ${ComputerVAX.mib(d.overlayBytes)} / ${ComputerVAX.mib(d.overlayCeilingBytes)}` +
                     ` | cache ${ComputerVAX.mib(d.cacheResidentBytes)} / ${ComputerVAX.mib(d.cacheCeilingBytes)}` : "") +
                (m.typed && m.typed.length ? ` | auto-typed: ${m.typed.join(", ")}` : ""));
            break;
        }

        case "stopped":
            globals.window['vaxError'] = `HALTED: ${m.reason}`;
            this.status(`HALTED: ${m.reason}`);
            this.fRunning = false;
            this.cpu.updateStatus(false);
            break;

        case "error":
            globals.window['vaxError'] = m.message;
            this.status(`ERROR: ${m.message}`);
            this.fRunning = false;
            this.cpu.updateStatus(false);
            break;
        }
    }

    /**
     * ComputerVAX.mib(n)
     *
     * @param {number} n
     * @returns {string}
     */
    static mib(n) { return (n / (1 << 20)).toFixed(1) + " MiB"; }

    /**
     * status(sMessage)
     *
     * @this {ComputerVAX}
     * @param {string} sMessage
     */
    status(sMessage)
    {
        this.sStatus = sMessage;
        let control = this.bindings["status"];
        if (control) control.textContent = sMessage;
    }

    /**
     * attach(sMessage)
     *
     * @this {ComputerVAX}
     * @param {string} sMessage
     */
    attach(sMessage)
    {
        let control = this.bindings["attach"];
        if (control) control.textContent = sMessage;
    }

    /**
     * setBinding(sHTMLType, sBinding, control, sValue)
     *
     * @this {ComputerVAX}
     * @param {string} sHTMLType
     * @param {string} sBinding ("power", "reset", "status", "attach")
     * @param {HTMLElement} control
     * @param {string} [sValue]
     * @returns {boolean}
     */
    setBinding(sHTMLType, sBinding, control, sValue)
    {
        let computer = this;
        switch (sBinding) {
        case "power":
            this.bindings[sBinding] = control;
            control.onclick = function onClickPower() { computer.startCPU(); };
            return true;
        case "reset":
            this.bindings[sBinding] = control;
            control.onclick = function onClickReset() {
                if (computer.worker) {
                    computer.worker.terminate();
                    computer.worker = null;
                    computer.fRunning = false;
                }
                if (computer.serial) computer.serial.clear();
                globals.window['vaxError'] = null;
                computer.startCPU();
            };
            return true;
        case "status":
        case "attach":
            this.bindings[sBinding] = control;
            return true;
        }
        return false;
    }

    /**
     * ComputerVAX.init()
     *
     * Same shape as ComputerPDP11.init() (computer.js:1500): walk every machine on the page, find
     * its computer element, and construct.  Because computer.js is imported LAST by
     * modules/v2/machine.js, this runs after every other component's WebLib.onInit() handler, which
     * is what lets the constructor find them with getComponentByType().
     */
    static init()
    {
        let aeMachines = Component.getElementsByClass(APPCLASS, "machine");
        for (let iMachine = 0; iMachine < aeMachines.length; iMachine++) {
            let eMachine = aeMachines[iMachine];
            let parmsMachine = Component.getComponentParms(eMachine);
            let aeComputers = Component.getElementsByClass(APPCLASS, "computer", eMachine);
            for (let iComputer = 0; iComputer < aeComputers.length; iComputer++) {
                let eComputer = aeComputers[iComputer];
                let parmsComputer = Component.getComponentParms(eComputer);
                let computer = new ComputerVAX(parmsComputer, parmsMachine, false);
                Component.bindComponentControls(computer, eComputer, APPCLASS);
            }
        }
    }
}

WebLib.onInit(ComputerVAX.init);
