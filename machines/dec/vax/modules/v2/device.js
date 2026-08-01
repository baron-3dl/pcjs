/**
 * @fileoverview Dispatches VAX `<device type="..."/>` elements to their components (pcjsvax-f23)
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
 * The same job DevicePDP11.init() does (machines/dec/pdp11/modules/v2/device.js:1163): walk every
 * element of class "vax-device", read its `type` out of the XSL-generated `data-value`, and
 * construct the matching component.  PDPjs dispatches 'pc11'/'rk11'/'rl11'/'rx11'; we have one
 * disk controller today, so this dispatches 'rqdx3'.
 */

import Component from "../../../../modules/v2/component.js";
import WebLib from "../../../../modules/v2/weblib.js";
import RQDX3 from "./rqdx3.js";
import { APPCLASS } from "./defines.js";

/**
 * @class DeviceVAX
 */
export default class DeviceVAX {
    /**
     * DeviceVAX.init()
     */
    static init()
    {
        let aeDevice = Component.getElementsByClass(APPCLASS, "device");
        for (let iDevice = 0; iDevice < aeDevice.length; iDevice++) {
            let device;
            let eDevice = aeDevice[iDevice];
            let parmsDevice = Component.getComponentParms(eDevice);
            switch (parmsDevice['type']) {
            case 'rqdx3':
                device = new RQDX3(parmsDevice);
                Component.bindComponentControls(device, eDevice, APPCLASS);
                break;
            }
        }
    }
}

WebLib.onInit(DeviceVAX.init);
