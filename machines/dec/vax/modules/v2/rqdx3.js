/**
 * @fileoverview Implements the RQDX3 MSCP Disk Controller device component (pcjsvax-f23)
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * THE CONVENTION THIS IMPLEMENTS, quoted from machines/dec/pdp11/rk11/README.md:
 *
 *     "Device XML files not only configure a device, but also list all the resources the device
 *      will use, and define UI elements used to control the device, such as choosing which disks
 *      should be auto-mounted by the RK11 device."
 *
 * So machines/dec/vax/rqdx3/default.xml carries a `<control type="list" binding="listDisks">` of
 * `<disk id name path/>` entries, a `loadDisk` button, a `mountDisk` file picker and a `descDisk`
 * description, and the machine XML supplies `autoMount='{DUA0:{path:"..."}}'` on its `<device
 * ref="/machines/dec/vax/rqdx3/default.xml"/>` element.  That is the same shape as
 * machines/dec/pdp11/rk11/default.xml and machines/dec/pdp11/pc11/default.xml, and the parsing
 * below is machines/dec/pdp11/modules/v2/drive.js's parseConfig()/autoMount() reduced to what an
 * MSCP controller with no removable media needs.
 *
 * WHERE THIS DIVERGES, and it is ONE thing rather than several.
 * ------------------------------------------------------------
 * PDPjs's DriveController hands `sDiskPath` to disk.js, which calls WebLib.getResource() and then
 * buildDisk() -- and disk.js ALREADY accepts a raw binary image on that path (pcx86 disk.js:115
 * documents `*.img`; :610 getResource(url, sFormat, ...); :744 "If we received binary data instead
 * of JSON, we can use the same buildDisk() function").  So loading a raw image by `path` is
 * conventional and JSON encoding is NOT required.
 *
 * What is different here is WHERE the bytes end up.  PCjs's Disk holds the whole image as
 * aDiskData in the tab.  Ours is 159,334,400 bytes and the CPU that reads it lives in a Worker,
 * because rq.js's diskRead() is a SYNCHRONOUS `u.image.read()` issued from inside the instruction
 * stream and the main thread has no synchronous Blob read (browser/vaxworker.js's header).  So this
 * component's `load` produces a `File`, posts it to the Worker, and the Worker reads 64 KB at a
 * time out of it with FileReaderSync -- MEASURED over a boot: 55.3 MiB touched of 1000, 16 MiB
 * cache ceiling regardless of container size.  A `File` is used rather than a `Blob` only so the
 * status line has a name; vaxworker.js's fileBacking() calls nothing but `.size` and `.slice()`.
 *
 * GZIP.  A `path` ending in `.gz` is decompressed transparently with the standard
 * `DecompressionStream` before the File is made.  This is a hosting constraint, not a preference:
 * GitHub hard-rejects any file over 100 MB, and the raw image is 159 MB (19.1 MB gzipped).  The
 * SHIPPED path in rqdx3/default.xml is the RAW image, served by the media host pcjsvax-f07 is
 * provisioning; the `.gz` entry is listed alongside it for a repo-hosted deployment.  Either way
 * the landing URL takes NO query parameters -- browser/vax.html's `?diskgz=` survives only as an
 * override on the old adapter page.
 */

import Component from "../../../../modules/v2/component.js";
import StrLib from "../../../../modules/v2/strlib.js";
import WebLib from "../../../../modules/v2/weblib.js";
import MESSAGE from "./message.js";
import { DEBUG } from "./defines.js";

/**
 * @class RQDX3
 * @unrestricted
 */
export default class RQDX3 extends Component {
    /**
     * RQDX3(parms)
     *
     * The RQDX3 component has the following component-specific (parms) properties:
     *
     *      autoMount: a JSON-encoded object of drive names, each with a 'path' and optional 'name'
     *
     * @param {Object} parms
     */
    constructor(parms)
    {
        super("RQDX3", parms, MESSAGE.DEVICE);

        this.configMount = RQDX3.parseConfig(parms['autoMount']);

        /** @type {Object} drive name -> {sDiskName, sDiskPath, file} */
        this.aDrives = {};
        this.sDiskSource = "";
        this.cLoading = 0;

        /*
         * Same contract as PDPjs's DriveController: if there is anything to auto-mount, we stay
         * un-ready until it has arrived, so computer.js does not power a machine whose system disk
         * is still in flight.  With nothing to mount we are ready immediately and the machine comes
         * up at `>>>` with no DUA0, which is a legitimate way to use a console ROM.
         */
        if (!this.autoMount()) this.setReady();
    }

    /**
     * RQDX3.parseConfig(config)
     *
     * @param {string|Object|undefined} config
     * @returns {Object}
     */
    static parseConfig(config)
    {
        if (config && typeof config == "string") {
            try {
                /*
                 * The eval() is upstream drive.js's, and for the same reason: these strings are
                 * authored in the machine XML with unquoted keys (`{DUA0:{path:"..."}}`), which
                 * JSON.parse() rejects.  The input is a repo-controlled configuration file.
                 */
                config = eval("(" + config + ")");
            } catch (e) {
                Component.error("RQDX3 auto-mount error: " + e.message + " (" + config + ")");
                config = undefined;
            }
        }
        return /** @type {Object} */ (config) || {};
    }

    /**
     * setBinding(sHTMLType, sBinding, control, sValue)
     *
     * @this {RQDX3}
     * @param {string} sHTMLType
     * @param {string} sBinding ("listDisks", "loadDisk", "descDisk", "mountDisk")
     * @param {HTMLElement} control
     * @param {string} [sValue]
     * @returns {boolean}
     */
    setBinding(sHTMLType, sBinding, control, sValue)
    {
        let dc = this;
        switch (sBinding) {

        case "listDisks":
            this.bindings[sBinding] = control;
            control.onchange = function onDiskChange() {
                let controlDesc = dc.bindings["descDisk"];
                let controlOption = control.options && control.options[control.selectedIndex];
                if (controlDesc && controlOption) {
                    let sDesc = controlOption.getAttribute("desc") || "";
                    let sHref = controlOption.getAttribute("href") || "";
                    if (sHref) sDesc = '<a href="' + sHref + '" target="_blank">' + sDesc + '</a>';
                    controlDesc.innerHTML = sDesc;
                }
            };
            return true;

        case "loadDisk":
            this.bindings[sBinding] = control;
            control.onclick = function onClickLoadDisk() {
                let controlDisks = dc.bindings["listDisks"];
                if (!controlDisks || !controlDisks.options.length) return;
                let sDiskPath = controlDisks.value;
                let sDiskName = controlDisks.options[controlDisks.selectedIndex].textContent;
                dc.load("DUA0", sDiskName, sDiskPath);
            };
            return true;

        case "descDisk":
            this.bindings[sBinding] = control;
            return true;

        case "mountDisk": {
            this.bindings[sBinding] = control;
            let controlInput = control.querySelector ? control.querySelector('input[type="file"]') : null;
            let controlSubmit = control.querySelector ? control.querySelector('input[type="submit"]') : null;
            if (!controlInput) return false;
            if (controlSubmit) controlSubmit.disabled = false;
            controlInput.addEventListener('change', function onDiskSelect() {
                let file = controlInput.files[0];
                if (!file) return;
                dc.setDrive("DUA0", StrLib.getBaseName(file.name, true), file.name, file);
                dc.setReady();
            });
            if (control.tagName == "FORM") {
                control.onsubmit = function(event) { event.preventDefault(); return false; };
            }
            return true;
        }
        }
        return false;
    }

    /**
     * autoMount()
     *
     * @this {RQDX3}
     * @returns {boolean} true if any disk is being loaded, false if none
     */
    autoMount()
    {
        let fLoading = false;
        for (let sDrive in this.configMount) {
            let configDisk = this.configMount[sDrive];
            let sDiskPath = configDisk['path'] || "";
            if (!sDiskPath) continue;
            let sDiskName = configDisk['name'] || StrLib.getBaseName(sDiskPath, true);
            this.load(sDrive, sDiskName, sDiskPath);
            fLoading = true;
        }
        return fLoading;
    }

    /**
     * load(sDrive, sDiskName, sDiskPath)
     *
     * @this {RQDX3}
     * @param {string} sDrive (eg, "DUA0")
     * @param {string} sDiskName
     * @param {string} sDiskPath
     */
    load(sDrive, sDiskName, sDiskPath)
    {
        let dc = this;
        let sURL = WebLib.getResourceURL(sDiskPath);
        let fGzip = /\.gz$/i.test(sDiskPath);

        this.cLoading++;
        this.status(`Loading ${sDiskName} (${sDiskPath})...`);
        if (DEBUG) this.printf(MESSAGE.LOG, "load(\"%s\",\"%s\")\n", sDrive, sURL);

        fetch(sURL).then(function(resp) {
            if (!resp.ok) throw new Error(`HTTP ${resp.status} fetching ${sURL}`);
            /*
             * The progress line matters here in a way it does not for a 2.4 MB RK03: a VAX system
             * disk is ~8x the largest media upstream ships (/harddisks/pcx86/20mb/*.json), and a
             * demo that shows nothing for the length of that transfer reads as broken.
             */
            let total = +(resp.headers.get("Content-Length") || 0);
            let body = resp.body;
            if (body && total) body = RQDX3.progressStream(body, total, function(loaded) {
                dc.status(`Loading ${sDiskName}: ${(100 * loaded / total).toFixed(0)}%`);
            });
            let stream = fGzip && body ? body.pipeThrough(new DecompressionStream("gzip")) : body;
            return stream ? new Response(stream).blob() : resp.blob();
        }).then(function(blob) {
            let sFileName = StrLib.getBaseName(sDiskPath).replace(/\.gz$/i, "");
            let file = new File([blob], sFileName, {type: "application/octet-stream"});
            dc.setDrive(sDrive, sDiskName, sDiskPath, file);
            dc.status(`${sDrive}: ${sDiskName} (${file.size.toLocaleString()} bytes, read lazily; this file is not modified)`);
            if (!--dc.cLoading) dc.setReady();
        }).catch(function(err) {
            dc.status(`Unable to load ${sDiskPath}: ${err.message}`);
            dc.printf(MESSAGE.NOTICE, "Unable to load %s: %s\n", sDiskPath, err.message);
            dc.setError("unable to load " + sDiskPath);
            if (!--dc.cLoading) dc.setReady();
        });
    }

    /**
     * RQDX3.progressStream(body, total, progress)
     *
     * @param {ReadableStream} body
     * @param {number} total
     * @param {function(number)} progress
     * @returns {ReadableStream}
     */
    static progressStream(body, total, progress)
    {
        let loaded = 0, next = 0;
        return body.pipeThrough(new TransformStream({
            transform(chunk, controller) {
                loaded += chunk.byteLength;
                if (loaded >= next) { progress(loaded); next = loaded + (total / 50); }
                controller.enqueue(chunk);
            }
        }));
    }

    /**
     * setDrive(sDrive, sDiskName, sDiskPath, file)
     *
     * @this {RQDX3}
     * @param {string} sDrive
     * @param {string} sDiskName
     * @param {string} sDiskPath
     * @param {File} file
     */
    setDrive(sDrive, sDiskName, sDiskPath, file)
    {
        this.aDrives[sDrive] = {sDiskName, sDiskPath, file};
    }

    /**
     * getDrive(sDrive)
     *
     * @this {RQDX3}
     * @param {string} sDrive
     * @returns {Object|null}
     */
    getDrive(sDrive)
    {
        return this.aDrives[sDrive] || null;
    }

    /**
     * status(sMessage)
     *
     * @this {RQDX3}
     * @param {string} sMessage
     */
    status(sMessage)
    {
        let control = this.bindings["descDisk"];
        if (control) control.textContent = sMessage;
        this.printf(MESSAGE.STATUS, "%s\n", sMessage);
    }

    /**
     * powerUp(data, fRepower)
     *
     * @this {RQDX3}
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
     * @this {RQDX3}
     * @param {boolean} [fSave]
     * @param {boolean} [fShutdown]
     * @returns {Object|boolean}
     */
    powerDown(fSave, fShutdown)
    {
        return true;
    }
}
