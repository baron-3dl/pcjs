/**
 * @fileoverview PCjs Global Definitions
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 */

/**
 * @define {string}
 */
const APPVERSION = "2.xx";              // this @define is overridden with the version in machines.json

/**
 * COMPILED is false by default; overridden with true in the Closure Compiler release.
 *
 * @define {boolean}
 */
const COMPILED = false;                 // this @define is overridden by the Closure Compiler

/**
 * @define {string}
 */
const COPYRIGHT = "Copyright © 2012-2026 Jeff Parsons <Jeff@pcjs.org>";

/**
 * @define {string}
 */
const CSSCLASS = "pcjs";

/**
 * DEBUG is true by default, enabling assertions and other runtime checks; overridden with false
 * in the Closure Compiler release, which generally results in the removal of any DEBUG code.  Our
 * gulpfile, however, takes the extra precaution of physically removing all "assert" method calls
 * from the concatenated file that is generated for the Closure Compiler.
 *
 * @define {boolean}
 */
const DEBUG = true;

/**
 * WARNING: DEBUGGER needs to accurately reflect whether or not the Debugger component is (or will be) loaded.
 * In the compiled case, we rely on the Closure Compiler to override DEBUGGER as appropriate.  When it's *false*,
 * nearly all of debugger.js will be conditionally removed by the compiler, reducing it to little more than a
 * "type skeleton", which also solves some type-related warnings we would otherwise have if we tried to remove
 * debugger.js from the compilation process altogether.
 *
 * However, when we're in "development mode" and running uncompiled code in debugger-less configurations,
 * I would like to skip loading debugger.js altogether.  When doing that, we must ALSO arrange for an additional file
 * (nodebugger.js) to be loaded immediately after this file, which *explicitly* overrides DEBUGGER with *false*.
 *
 * @define {boolean}
 */
var DEBUGGER = true;

/**
 * @define {string}
 */
const LICENSE = "License: MIT <https://www.pcjs.org/LICENSE.txt>";

/**
 * MAXDEBUG can be set to true to enable MAXDEBUG-only code (ie, hyper-aggressive DEBUG checks that should only be
 * enabled in special cases); it always overridden with false in the Closure Compiler release.
 *
 * @define {boolean}
 */
const MAXDEBUG = false;

/**
 * PRIVATE can be set to true to enable PRIVATE-only code (ie, code that should never be enabled in a public release);
 * it always overridden with false in the Closure Compiler release.
 *
 * @define {boolean}
 */
const PRIVATE = false;

/*
 * RS-232 DB-25 Pin Definitions, mapped to bits 1-25 in a 32-bit status value.
 *
 * SerialPorts in PCjs machines are considered DTE (Data Terminal Equipment), which means they should be "virtually"
 * connected to each other via a null-modem cable, which assumes the following cross-wiring:
 *
 *     G       1  <->  1        G       (Ground)
 *     TD      2  <->  3        RD      (Received Data)
 *     RD      3  <->  2        TD      (Transmitted Data)
 *     RTS     4  <->  5        CTS     (Clear To Send)
 *     CTS     5  <->  4        RTS     (Request To Send)
 *     DSR   6+8  <->  20       DTR     (Data Terminal Ready)
 *     SG      7  <->  7        SG      (Signal Ground)
 *     DTR    20  <->  6+8      DSR     (Data Set Ready + Carrier Detect)
 *     RI     22  <->  22       RI      (Ring Indicator)
 *
 * TODO: Move these definitions to a more appropriate shared file at some point.
 */
const RS232 = {
    RTS: {
        PIN:  4,
        MASK: 0x00000010
    },
    CTS: {
        PIN:  5,
        MASK: 0x00000020
    },
    DSR: {
        PIN:  6,
        MASK: 0x00000040
    },
    CD: {
        PIN:  8,
        MASK: 0x00000100
    },
    DTR: {
        PIN:  20,
        MASK: 0x00100000
    },
    RI: {
        PIN:  22,
        MASK: 0x00400000
    }
};

/**
 * SITEURL is the URL of the web server; it is replaced by the Closure Compiler (eg, "https://www.pcjs.org")
 *
 * @define {string}
 */
const SITEURL = "http://localhost:8088";

/**
 * LOCALDISKS is intended to reflect the webserver's operating mode.  Normally, we assume that all web
 * resources should be accessed remotely, but if the webserver is running in "developer" mode, then the
 * webserver should indicate that fact by setting the global variable 'LOCALDISKS' to true on any pages
 * with embedded machines.
 *
 * @define {boolean}
 */
var LOCALDISKS = false;

/*
 * Platform-agnostic way to isolate global variables (both mine and the system's).
 *
 * pcjsvax-de8: this used to read `(typeof window != "undefined")? {} : global`, i.e. it treated
 * "there is no window" as "this is Node".  There is a THIRD platform where that is false -- a Web
 * WORKER, which has neither `window` nor `global` -- and the reference threw
 * `ReferenceError: global is not defined` at module load, taking down every module that imports
 * this one.  That is not hypothetical: machines/dec/vax/browser/vaxworker.js runs the VAX in a
 * Worker because rq.js reads disk blocks SYNCHRONOUSLY and `FileReaderSync` is the only synchronous
 * lazy read a browser has, so it is the only place a 1 GB disk image can be served from at all.
 *
 * `globalThis` is the platform-agnostic spelling and exists in Node, in a window and in a Worker,
 * so `window` needs nothing else.  `node` is now decided by asking about NODE rather than by
 * asking about `window`: under Node it is still the global object (globalThis === global there),
 * and in a browser -- window or worker -- it is still `{}`, which is what weblib.js's
 * `globals.node.readFileSync` guard expects.
 */
const IS_NODE = (typeof process != "undefined") && !!(process.versions && process.versions.node);
let globals = {
    browser: (typeof window != "undefined"),
    node: IS_NODE? globalThis : {},
    process: (typeof process != "undefined")? process : {},
    window: (typeof window != "undefined")? window : globalThis,
    document: (typeof document != "undefined")? document : {},
    pcjs: { 'machines': {}, 'components': [], 'commands': {}, 'files': null }
};

if (globals.window['PCjs']) {
    globals.pcjs = globals.window['PCjs'];
} else {
    globals.window['PCjs'] = globals.pcjs;
}

globals.window['LOCALDISKS'] = LOCALDISKS;

export { APPVERSION, COMPILED, COPYRIGHT, CSSCLASS, DEBUG, DEBUGGER, LICENSE, LOCALDISKS, MAXDEBUG, PRIVATE, RS232, SITEURL, globals };
