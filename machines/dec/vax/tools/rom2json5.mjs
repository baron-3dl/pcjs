/**
 * @fileoverview pcjsvax-f23 -- convert a raw KA655 console ROM image into the PCjs JSON5 ROM
 *               resource that a <rom file="..."/> element loads
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY THIS EXISTS -- i.e. WHY THE ROM IS NOT SERVED AS A RAW .bin
 * ---------------------------------------------------------------
 * PCjs's ROM component (machines/dec/pdp11/modules/v2/rom.js:191-201) accepts a `file=` whose
 * extension is `.json`/`.json5` or `.hex` DIRECTLY; ANY OTHER extension -- including `.bin` -- is
 * rewritten into a request against `DumpAPI.ENDPOINT` (`/api/v1/dump?file=...&format=bytes`), a
 * SERVER-SIDE converter that does not exist on a static file server and does not exist on GitHub
 * Pages.  So on the deployment this demo actually targets, a raw `.bin` in a `file=` attribute is
 * not "less conventional", it simply 404s.  That is the whole reason every ROM upstream ships is a
 * `.json5` (machines/dec/pdp11/rom/M9312/23-616F1.json5).
 *
 * StrLib.getExtension() folds "json5" to "json" (strlib.js:536), which is what makes the `.json5`
 * suffix work on the direct path.
 *
 * FORMAT.  WebLib.parseMemoryResource() (weblib.js:354) accepts `bytes`, `words` or `longs`; we
 * emit `bytes`.  It takes the JSON.parse() fast path only when the text contains NO "0x" and no
 * "0o" (weblib.js:397), otherwise it falls back to eval() on the whole string -- so every number
 * here is DECIMAL and every key is quoted, which keeps a 128 KB ROM on JSON.parse().
 *
 * Usage:
 *   node machines/dec/vax/tools/rom2json5.mjs <in.bin> <out.json5> [loadAddr]
 */

import fs from "fs";

let [,, sIn, sOut, sAddr] = process.argv;
if (!sIn || !sOut) {
    console.error("usage: rom2json5.mjs <in.bin> <out.json5> [loadAddr]");
    process.exit(1);
}

/** KA655 boot/diagnostic ROM base (modules/v2/defines.js: VAX.PHYSMEM.ROM_BASE). */
const ROM_BASE = 0x20040000;

let addr = sAddr ? (sAddr.startsWith("0x") ? parseInt(sAddr, 16) : parseInt(sAddr, 10)) : ROM_BASE;
let ab = new Uint8Array(fs.readFileSync(sIn));

/*
 * 16 bytes per line purely so the file diffs readably; the parser does not care.
 */
let lines = [];
for (let i = 0; i < ab.length; i += 16) {
    lines.push(Array.from(ab.subarray(i, i + 16)).join(","));
}

let out = `{"load":${addr >>> 0},"exec":${addr >>> 0},\n"bytes":[\n${lines.join(",\n")}\n]}\n`;
fs.writeFileSync(sOut, out);
console.log(`${sIn}: ${ab.length} bytes -> ${sOut}: ${out.length} bytes, load=${(addr >>> 0).toString(16)}`);
