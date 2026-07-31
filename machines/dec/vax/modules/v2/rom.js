/**
 * @fileoverview Implements the VAX ROM component (pcjsvax-f23)
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS.  The `<rom addr size file/>` element of a VAX machine XML, in the same shape as
 * machines/dec/pdp11/modules/v2/rom.js: a Component that loads a ROM resource named by `file=`,
 * becomes ready when the bytes have arrived, and hands them to whoever asks.
 *
 * WHAT IT IS NOT.  It does NOT call bus.addMemory().  On this machine the bus, the memory blocks
 * and the CPU live in a Worker (see browser/vaxworker.js's header for WHY -- rq.js reads disk
 * blocks SYNCHRONOUSLY from inside the instruction stream, and the main thread has no synchronous
 * Blob read), so this component's job ends at "the bytes are here"; computer.js posts them across.
 *
 * ROM FILE FORMATS
 * ----------------
 * `.json5` / `.json` / `.hex`   the PCjs convention, parsed by WebLib.parseMemoryResource().
 *                              THIS IS WHAT THE SHIPPED MACHINE XML USES, because it is the only
 *                              format upstream's rom.js can load without a server: any other
 *                              extension is rewritten into a DumpAPI request
 *                              (`/api/v1/dump?file=...`), a server-side converter that does not
 *                              exist on a static file server or on GitHub Pages.
 *                              tools/rom2json5.mjs produces ours from the raw image.
 * `.bin` (or anything else)    ALSO ACCEPTED HERE, as an arraybuffer fetch.  This is a deliberate
 *                              DEPARTURE from upstream rom.js, which would send it to DumpAPI.
 *                              It exists because the raw KA655 image is what SIMH and every one of
 *                              this port's 35 gate checks consume, so being able to point a machine
 *                              XML at the same bytes without a conversion step is worth one branch.
 *                              It is not the shipped path; see machines/dec/vax/ka655/machine.xml.
 */

import Component from "../../../../modules/v2/component.js";
import StrLib from "../../../../modules/v2/strlib.js";
import WebLib from "../../../../modules/v2/weblib.js";
import MESSAGE from "./message.js";
import { APPCLASS, DEBUG } from "./defines.js";

/**
 * @class ROMVAX
 * @unrestricted
 */
export default class ROMVAX extends Component {
    /**
     * ROMVAX(parmsROM)
     *
     * The ROMVAX component expects the following (parmsROM) properties:
     *
     *      addr: physical address of the ROM
     *      size: expected size of the ROM, in bytes (0 to accept whatever arrives)
     *      file: path of the ROM resource
     *
     * @param {Object} parmsROM
     */
    constructor(parmsROM)
    {
        super("ROM", parmsROM, MESSAGE.ROM);

        this.abROM = null;
        this.addrROM = +parmsROM['addr'] || 0;
        this.sizeROM = +parmsROM['size'] || 0;
        this.sFilePath = parmsROM['file'];
        this.sFileName = StrLib.getBaseName(this.sFilePath || "");

        if (!this.sFilePath) {
            /*
             * A ROM with no file is not an error at construction time: browser/vax.html's file
             * picker path exists precisely so a user can supply their own image, and the machine
             * XML keeps a `<control type="file" binding="loadRom"/>` for the same reason.  We
             * simply stay un-ready until something calls setData().
             */
            return;
        }

        let sFileURL = this.sFilePath;
        let sFileExt = StrLib.getExtension(this.sFileName);      // NOTE: this folds "json5" -> "json"
        let rom = this;

        if (sFileExt == "json" || sFileExt == "hex") {
            if (DEBUG) this.printf(MESSAGE.LOG, "load(\"%s\")\n", sFileURL);
            WebLib.getResource(sFileURL, null, true, function doneLoad(sURL, sResponse, nErrorCode) {
                rom.finishLoad(sURL, sResponse, nErrorCode);
            });
        } else {
            /*
             * The raw-image branch documented in this file's header.  Upstream would hand this to
             * DumpAPI; we read the bytes.
             */
            WebLib.getResource(sFileURL, "arraybuffer", true, function doneLoadRaw(sURL, resource, nErrorCode) {
                if (nErrorCode || !resource) {
                    rom.printf(MESSAGE.NOTICE, "Unable to load ROM resource (error %d: %s)\n", nErrorCode, sURL);
                    rom.setError("unable to load " + sURL);
                    rom.setReady();
                    return;
                }
                rom.setData(new Uint8Array(/** @type {ArrayBuffer} */ (resource)), sURL);
            });
        }
    }

    /**
     * finishLoad(sURL, sData, nErrorCode)
     *
     * @this {ROMVAX}
     * @param {string} sURL
     * @param {string} sData
     * @param {number} nErrorCode
     */
    finishLoad(sURL, sData, nErrorCode)
    {
        if (nErrorCode) {
            this.printf(MESSAGE.NOTICE, "Unable to load ROM resource (error %d: %s)\n", nErrorCode, sURL);
            this.setError("unable to load " + sURL);
            this.setReady();
            return;
        }
        Component.addMachineResource(this.idMachine, sURL, sData);
        let resource = WebLib.parseMemoryResource(sURL, sData);
        if (!resource || !resource.aBytes) {
            this.setError("unrecognized ROM resource: " + sURL);
            this.setReady();
            return;
        }
        this.setData(Uint8Array.from(resource.aBytes), sURL);
    }

    /**
     * setData(abROM, sURL)
     *
     * The single point at which this component becomes ready, whether the bytes came from `file=`
     * or from a user-picked file.
     *
     * @this {ROMVAX}
     * @param {Uint8Array} abROM
     * @param {string} [sURL]
     */
    setData(abROM, sURL)
    {
        if (this.sizeROM && abROM.length != this.sizeROM) {
            /*
             * Same reasoning as upstream rom.js: the declared size exists to confirm that the ROM
             * you received is the ROM you expected, and setError() keeps setReady() from marking
             * the component ready, which keeps the Computer from powering a machine that would
             * then fail in some much less legible way.
             */
            this.setError("ROM size (" + abROM.length + ") does not match specified size (" + this.sizeROM + ")");
            this.setReady();
            return;
        }
        this.abROM = abROM;
        this.printf(MESSAGE.STATUS, "Loaded %d-byte ROM at %#010x%s\n", abROM.length, this.addrROM, sURL ? " from " + sURL : "");
        this.setReady();
    }

    /**
     * getData()
     *
     * @this {ROMVAX}
     * @returns {Uint8Array|null}
     */
    getData()
    {
        return this.abROM;
    }

    /**
     * setBinding(sHTMLType, sBinding, control, sValue)
     *
     * @this {ROMVAX}
     * @param {string} sHTMLType
     * @param {string} sBinding ("loadRom")
     * @param {HTMLElement} control
     * @param {string} [sValue]
     * @returns {boolean}
     */
    setBinding(sHTMLType, sBinding, control, sValue)
    {
        let rom = this;
        switch (sBinding) {
        case "loadRom":
            this.bindings[sBinding] = control;
            /*
             * The control the XSL emits for `<control type="file"/>` is a <form> wrapping an
             * <input type="file"> and a submit button; upstream drive.js reads it exactly this way.
             */
            let controlInput = control.querySelector ? control.querySelector('input[type="file"]') : null;
            let controlSubmit = control.querySelector ? control.querySelector('input[type="submit"]') : null;
            if (!controlInput) return false;
            if (controlSubmit) controlSubmit.disabled = false;
            controlInput.addEventListener('change', function onROMChange() {
                let file = controlInput.files[0];
                if (!file) return;
                file.arrayBuffer().then(function(ab) {
                    rom.clearError();
                    rom.setData(new Uint8Array(ab), file.name);
                });
            });
            if (control.tagName == "FORM") {
                control.onsubmit = function(event) { event.preventDefault(); return false; };
            }
            return true;
        }
        return false;
    }

    /**
     * powerUp(data, fRepower)
     *
     * @this {ROMVAX}
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
     * @this {ROMVAX}
     * @param {boolean} [fSave]
     * @param {boolean} [fShutdown]
     * @returns {Object|boolean}
     */
    powerDown(fSave, fShutdown)
    {
        return true;
    }

    /**
     * ROMVAX.init()
     *
     * Operates on every HTML element of class "vax-rom", exactly as ROMPDP11.init() does for
     * "pdp11-rom".
     */
    static init()
    {
        let aeROM = Component.getElementsByClass(APPCLASS, "rom");
        for (let iROM = 0; iROM < aeROM.length; iROM++) {
            let eROM = aeROM[iROM];
            let parmsROM = Component.getComponentParms(eROM);
            let rom = new ROMVAX(parmsROM);
            Component.bindComponentControls(rom, eROM, APPCLASS);
        }
    }
}

WebLib.onInit(ROMVAX.init);
