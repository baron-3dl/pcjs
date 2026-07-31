/**
 * @fileoverview VAXjs machine entry point -- loads every Component a VAX machine XML can name
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * This is what machines/dec/vax/xsl/components.xsl's componentScripts template loads.  PDPjs loads
 * a BUNDLE built by gulp (/machines/dec/pdp11/releases/2.21/pdp11.js) whose module list lives in
 * machines/machines.json; VAXjs has no bundle yet, so this module stands in for it -- the same
 * shape PCjs's own `site.pcjs.unbundled` mode uses, with the import graph resolving the list.
 *
 * ORDER MATTERS, exactly as it does in machines.json's "modules" array: each component registers a
 * WebLib.onInit() handler at import time, and they fire in registration order.  computer.js MUST be
 * last, because ComputerVAX's constructor looks the other components up with
 * Component.getComponentByType() and they have to exist by then.  This is why machines.json lists
 * "./machines/dec/pdp11/modules/v2/computer.js" second-to-last, ahead of only state.js and embed.js.
 */

import { globals } from "./defines.js";
import "./rom.js";
import "./ram.js";
import "./cpuka655.js";
import "./serial.js";
import "./device.js";
import "./computer.js";       /* MUST be last -- see the header */

/*
 * The local-media rewrite.  weblib.js's getResourceURL() maps a media class path such as
 * "/vaxdisks/vms55-rd54.dsk" onto "/disks/vaxdisks/..." when LOCALDISKS is set AND the page is
 * being served from a local host -- and onto "https://vaxdisks.pcjs.org/..." otherwise.  Setting it
 * here is the same thing _includes/machines/scripts.html emits for `site.pcjs.localdisks`, and it
 * is INERT off a local host because weblib gates it on the hostname as well.  It is what lets ONE
 * machine XML serve both a developer running `python3 -m http.server` over the gitignored
 * /disks/vaxdisks mirror and the hosted demo (pcjsvax-f07), with no query parameters in either.
 *
 * This runs after every import above has been evaluated (ES module imports are hoisted), but LONG
 * before any of it matters: nothing fetches media until the components' WebLib.onInit() handlers
 * run, and those fire on the window 'load' event.
 */
globals.window['LOCALDISKS'] = true;
