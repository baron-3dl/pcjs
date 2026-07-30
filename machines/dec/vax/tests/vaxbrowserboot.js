/**
 * @fileoverview pcjsvax-de8 / pcjsvax-ae1 -- boot OpenVMS through the BROWSER driver and the
 *               COPY-ON-WRITE image provider, under Node, and grade the result
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY IT EXISTS
 * -------------
 * browser/vaxmachine.js and browser/imageprovider.js are the two files the browser tab runs, and
 * both of them would otherwise be graded by nothing: a driver and a provider that only exist inside
 * a Worker cannot be reached by any differential in this tree.  This runs the SAME two files under
 * Node -- the only substitution is `readRaw`, which here is `fs.readSync` and in the tab is
 * `FileReaderSync` over `File.slice()` -- and grades the boot they produce.
 *
 * It is not a differential: there is no oracle in the process and nothing is compared byte for
 * byte.  It grades exactly three things, and they are the three the browser deliverable is judged
 * on:
 *
 *   1. the CONSOLE reaches a *** BARE *** `Username: ` at end of line, AND ANSWERS A USERNAME WITH
 *      `Password:`.  HANDOFF.md's §0 records that this has already produced one false positive:
 *      `Username:` ALSO appears as a PADDED FIELD inside an OPCOM audit record
 *      (`Username:                 SYSTEM`) in both engines' startup, so a naive `grep Username:`
 *      passes on a machine that never reached a login prompt.
 *
 *      HANDOFF.md §0 says to confirm the real prompt with `LOGINOUT.EXE` in the stream.  *** THAT
 *      IS NOT A PROPERTY OF THE CONSOLE STREAM AND IT IS NOT GRADED HERE. ***  MEASURED 2026-07-30
 *      over a full boot to the prompt: the console contains `VAX/VMS V7.1  node VAX1` followed by
 *      a bare `Username: ` and the string `LOGINOUT.EXE` appears NOWHERE in it -- LOGINOUT is the
 *      IMAGE running on `_OPA0:`, which is something a `SHOW SYSTEM` reports, not something the
 *      login prompt prints.  Grading a string the guest never emits would have made a green run
 *      impossible, and adding it as a soft check would have taught the next reader the same wrong
 *      thing.  So the padded-field trap is closed a STRONGER way instead: this file TYPES a
 *      username at the prompt and requires `Password:` back.  An OPCOM audit record cannot answer.
 *   2. the unit is NOT write-locked.  A read-only DUA0 stops OpenVMS dead at
 *      `%SYSTEM-I-MOUNTVER, VAX1$DUA0: has been write-locked` -- that is what the copy-on-write
 *      overlay exists to prevent, so its absence is graded rather than assumed.
 *   3. the user's container is BYTE-FOR-BYTE UNCHANGED afterwards.  The overlay's whole claim is
 *      that a browser `File` can back a writable unit without being written to; a run that quietly
 *      modified the image would satisfy 1 and 2 and still be the wrong thing.
 *
 * It also REPORTS the two memory ceilings the browser deliverable is bounded by -- the overlay's
 * dirty-block count and the chunk cache's resident set -- because "what does a boot actually cost"
 * is a number that has to be measured, not asserted.
 *
 * USAGE
 *   node machines/dec/vax/tests/vaxbrowserboot.js --volume PATH [--rom PATH] [--mem MB]
 *                                                 [--max-seconds S] [--console-file P]
 *                                                 [--login USER]   (default SYSTEM)
 *
 * *** POINT --volume AT A COPY. ***  The overlay means this run will not write to it, and check 3
 * grades exactly that -- but a bug in the thing under test is not a reason to risk the original.
 */

import fs from "fs";
import path from "path";
import crypto from "crypto";

import { vaxRepo } from "./mscpharness.js";
import { overlayImageProvider } from "../browser/imageprovider.js";
import { VaxMachine } from "../browser/vaxmachine.js";

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

/** tests/vmsbootprobe.js's `readable()`, same conventions, so two transcripts line up literally. */
function readable(bytes)
{
    let s = "";
    for (let b of bytes) {
        if (b === 0x0D || b === 0x0A) s += "\n";
        else if (b === 0x1B) s += "";
        else if (b >= 0x20 && b < 0x7F) s += String.fromCharCode(b);
    }
    return s;
}

function sha256(p) { return crypto.createHash("sha256").update(fs.readFileSync(p)).digest("hex"); }

function main()
{
    let volume = getArg("--volume", null);
    let romPath = getArg("--rom", path.join(vaxRepo(), "open-simh/VAX/ka655x.bin"));
    let memMB = parseInt(getArg("--mem", "16"), 10);
    let maxSeconds = parseInt(getArg("--max-seconds", "600"), 10);
    let consoleFile = getArg("--console-file", null);
    let login = getArg("--login", "SYSTEM");
    if (!volume) { console.log("vaxbrowserboot: --volume PATH is required (point it at a COPY)"); process.exit(2); }
    if (!fs.existsSync(volume)) { console.log(`vaxbrowserboot: the volume ${volume} does not exist`); process.exit(2); }
    if (!fs.existsSync(romPath)) { console.log(`vaxbrowserboot: the ROM ${romPath} does not exist`); process.exit(2); }

    let romBytes = new Uint8Array(fs.readFileSync(romPath));
    let fd = fs.openSync(volume, "r");                      /* READ-ONLY on purpose: see check 3 */
    let disk = overlayImageProvider({
        byteLength: fs.statSync(volume).size,
        name: volume,
        readRaw: (offset, length) => {
            let b = Buffer.alloc(length);
            let n = fs.readSync(fd, b, 0, length, offset);
            return new Uint8Array(b.buffer, b.byteOffset, n);
        }
    });

    console.log(`vaxbrowserboot (pcjsvax-de8/ae1) -- browser/vaxmachine.js + browser/imageprovider.js under Node`);
    console.log(`  volume    ${volume} (${disk.byteLength} bytes; volume declares ${disk.filesystemBytes})`);
    console.log(`  rom       ${romPath}`);
    console.log(`  memory    ${memMB} MB`);

    let hashBefore = sha256(volume);
    let m = new VaxMachine({romBytes, memBytes: (memMB * 1024 * 1024) >>> 0, disk, autoBoot: true});
    let a = m.attachReport;
    console.log(`  DUA0      capac ${a.capacBefore} -> ${a.capacAfter} blocks, provider ` +
        `${a.providerWritable ? "WRITABLE" : "read-only"}, unit ${a.readOnly ? "READ-ONLY" : "read/write"}`);
    console.log(`  devices   ${m.deviceNames.join(", ")}`);

    /* THE SAME SHAPE THE TAB RUNS: bounded slices, with the caller re-arming.  Under Node there is
       nothing to yield TO, so the slice budget is large; the point is that this exercises
       runSlice()/pumpInput() rather than a bespoke `while` loop. */
    let out = [], t0 = Date.now(), deadline = t0 + maxSeconds * 1000, lastBeat = 0;
    let stop = null, promptAt = 0, sawBarePrompt = false;
    while (Date.now() < deadline) {
        let r = m.runSlice(200);
        let chunk = m.drainOutput();
        if (chunk) for (let b of chunk) out.push(b);
        if (r.stop) { stop = r.stop; break; }
        if (m.steps - lastBeat >= 50e6) {
            lastBeat = m.steps;
            process.stderr.write(`  .. ${(m.steps / 1e6).toFixed(0)}M steps, ${((Date.now() - t0) / 1000).toFixed(0)} s, ` +
                `console ${out.length} bytes, overlay ${disk.stats().overlayBlocks} blocks\n`);
            if (consoleFile) { try { fs.writeFileSync(consoleFile, Buffer.from(out)); } catch (e) {} }
        }
        /* THE PROMPT IS ANSWERED, NOT JUST OBSERVED.  The moment a bare `Username: ` is the tail of
           the stream, type a username -- through the SAME machine.type() path a human at the
           browser terminal uses -- and keep running until LOGINOUT answers `Password:`.  That is
           what tells a real login prompt apart from the padded OPCOM audit field, which cannot
           answer anything; see the file header. */
        let sofar = readable(out);
        if (!promptAt && /(^|\n)Username: $/.test(sofar)) {
            promptAt = m.steps;
            sawBarePrompt = true;
            console.log(`  .. bare \`Username: \` reached at step ${promptAt}; typing "${login}"`);
            m.type(login + "\r");
        }
        /* EITHER answer proves LOGINOUT is the image on the other end: a password prompt, or --
           for an account with no password, which is what this volume's SYSTEM is -- the welcome
           banner.  MEASURED 2026-07-30: this volume answers with the banner and then a DCL `$`. */
        if (promptAt && /Password:|Welcome to OpenVMS/.test(sofar)) { stop = "LOGGED IN"; break; }
        /* A bounded window after the prompt, so a guest that never answers fails rather than
           running to the wall-clock cap and reporting the wrong reason. */
        if (promptAt && m.steps - promptAt > 60e6) { stop = "THE PROMPT NEVER ANSWERED"; break; }
    }
    if (!stop) stop = "WALL-CLOCK CAP";

    let elapsed = (Date.now() - t0) / 1000;
    let s = disk.stats();
    let text = readable(out);
    if (consoleFile) fs.writeFileSync(consoleFile, Buffer.from(out));

    console.log(`\n  ran       ${m.steps} instructions in ${elapsed.toFixed(1)} s ` +
        `(${(m.steps / elapsed / 1e6).toFixed(2)} M instr/s)`);
    console.log(`  stopped   ${stop}`);
    console.log(`  typed     ${m.marks.map((k) => k.rule).join(", ") || "(nothing)"}`);
    console.log(`  console   ${out.length} bytes`);
    console.log(`  disk      ${s.reads} reads / ${s.writes} writes; backing store ${s.rawReads} raw reads, ` +
        `${(s.rawBytes / (1 << 20)).toFixed(1)} MiB of ${(disk.byteLength / (1 << 20)).toFixed(0)} MiB touched`);
    console.log(`  ceilings  overlay ${s.overlayBlocks} blocks = ${(s.overlayBytes / (1 << 20)).toFixed(1)} MiB ` +
        `of ${(s.overlayCeilingBytes / (1 << 20)).toFixed(0)} MiB; cache resident ` +
        `${(s.cacheResidentBytes / (1 << 20)).toFixed(1)} MiB of ${(s.cacheCeilingBytes / (1 << 20)).toFixed(0)} MiB`);
    console.log(`  heap      ${(process.memoryUsage().heapUsed / (1 << 20)).toFixed(0)} MB used, ` +
        `${(process.memoryUsage().external / (1 << 20)).toFixed(0)} MB external`);

    let passed = 0, failed = 0;
    const check = (name, cond, detail) => {
        if (cond) { passed++; console.log(`  PASS  ${name}`); }
        else { failed++; console.log(`  FAIL  ${name}${detail ? " -- " + detail : ""}`); }
    };
    console.log("");
    /* Graded on the flag set WHEN IT HAPPENED, not on the final transcript: answering the prompt
       appends the echoed username to that very line, so a check re-run at the end would look for a
       string the run has deliberately consumed. */
    check("a BARE `Username: ` line at end of stream (NOT the padded OPCOM audit field)",
        sawBarePrompt, `padded-field-also-present=${/Username: {2,}/.test(text)}`);
    check("the VMS banner precedes it", /VAX\/VMS V[0-9.]+ +node /.test(text));
    check(`the prompt ANSWERED "${login}" (an OPCOM audit field cannot)`,
        /Password:|Welcome to OpenVMS/.test(text), stop);
    check(`the username was ECHOED back, i.e. the input path reached the terminal driver`,
        new RegExp(`Username: ${login}`).test(text));
    /* Reported, never graded -- see the file header.  MEASURED absent, and a check on it would be
       a check on a string the guest does not emit. */
    console.log(`  NOTE  \`LOGINOUT.EXE\` in the console stream: ${/LOGINOUT\.EXE/.test(text) ? "present" : "ABSENT"} ` +
        `(HANDOFF.md \u00a70 suggests it as a confirmation; it is the image on _OPA0:, not console output)`);
    check("DUA0 was NOT write-locked", !/has been write-locked/.test(text));
    check("the unit attached read/write", a.readOnly === false);
    check("the user's container is byte-for-byte unchanged", sha256(volume) === hashBefore);
    check("the whole image was NEVER loaded (backing reads under 25% of the container)",
        s.rawBytes < disk.byteLength / 4, `${s.rawBytes} of ${disk.byteLength}`);

    fs.closeSync(fd);
    console.log(`\nPASSED ${passed} / FAILED ${failed}`);
    if (failed) {
        console.log(`\n----- LAST 3000 CHARACTERS OF THE CONSOLE -----`);
        console.log(text.slice(-3000));
    }
    process.exit(failed ? 1 : 0);
}

main();
