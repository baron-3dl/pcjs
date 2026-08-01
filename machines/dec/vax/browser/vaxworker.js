/**
 * @fileoverview pcjsvax-de8 / pcjsvax-ae1 -- the Worker the VAX actually runs in, and the two
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 *               SYNCHRONOUS browser backings for rq.js's image provider
 *
 * *** WHY A WORKER, AND NOT THE MAIN THREAD.  THIS IS THE LOAD-BEARING DECISION IN THE WHOLE
 * BROWSER DELIVERABLE, AND IT IS FORCED BY rq.js's PROVIDER CONTRACT. ***
 *
 * rq.js's diskRead() is `let n = u.image.read(offset, want, u.rqxb)` -- a SYNCHRONOUS call that
 * returns the byte count, made from inside the instruction stream while an MSCP command is being
 * serviced.  There is no await to put anywhere: the CPU is mid-instruction.  On the MAIN THREAD a
 * browser has no synchronous way to read a slice of a `File` at all -- `FileReader` is callback
 * based, `Blob.arrayBuffer()` is a promise, and synchronous XHR is deprecated there -- so a
 * main-thread port would have to load the entire container up front, and the container is 1 GB.
 *
 * In a WORKER both synchronous readers exist:
 *   - `FileReaderSync.readAsArrayBuffer(file.slice(off, off + len))` for a picked local file, and
 *   - a synchronous `XMLHttpRequest` with a `Range` header for a hosted one.
 *
 * So the Worker is not a performance nicety, it is what makes constraint 1 -- never load the whole
 * image -- implementable at all.  It also happens to buy the obvious second thing: the UI thread
 * never runs a single guest instruction, so the page stays responsive by construction rather than
 * by the run loop being polite.  The run loop still yields anyway (see `pump()`), because the
 * Worker has to process the keystrokes the main thread posts to it.
 *
 * WHAT IS NOT HERE
 * ----------------
 * No DOM, no terminal, no PCjs framework.  The machine is browser/vaxmachine.js's, the provider is
 * browser/imageprovider.js's, and both are graded under Node by tests/vaxbrowserboot.js and
 * tests/overlayprovider.js.  This file is the transport between them and the tab, and it is the
 * only file in the deliverable that cannot be run outside a browser.
 */

import { overlayImageProvider } from "./imageprovider.js";
import { VaxMachine } from "./vaxmachine.js";

/**
 * *** THE RUN-LOOP YIELD. ***
 *
 * A slice this long, then hand the event loop back.  16 ms is one frame: long enough that the
 * per-slice overhead (a Date.now() pair, an output drain, a rule scan) is noise against ~4M
 * instructions/second, short enough that a keystroke posted from the main thread waits at most one
 * frame before `pumpInput()` sees it.
 */
const SLICE_MS = 16;

/**
 * *** THE IDLE WAIT (pcjsvax-af8). ***
 *
 * The yield above is deliberately a MessageChannel round trip and NOT `setTimeout(0)`, because a
 * busy machine must re-arm with no clamp.  An IDLE machine wants the opposite: it wants to stop
 * re-arming and give the core back.  So when `runSlice()` reports that it fast-forwarded through
 * guest time instead of executing (see browser/vaxmachine.js's runSlice), the Worker waits on a
 * TIMER for as long as that guest time was worth, and the tab drops to whatever a sleeping timer
 * costs.  MEASURED at a VAX/VMS V5.5-2H4 login prompt: this is the difference between 100% of a
 * core and ~1%.
 *
 * setTimeout's 4 ms clamp after five nested calls, which disqualifies it for the busy path, is
 * irrelevant here -- these waits are milliseconds long by construction, and a wait rounded UP is a
 * wait, not a stall.
 *
 * HOW LONG the wait is is NOT this file's decision and NOT the guest time skipped: it is
 * `r.idleSleepMs`, the host time those elided instructions would have cost at this host's measured
 * rate.  modules/v2/idle.js's IdleThrottle carries the measurement and the reason -- in one line,
 * sleeping the GUEST time throttles a 3.9x-real-time emulator down to 1.0x and adds 14 seconds to
 * a boot, while sleeping the host time saved is exactly neutral.
 *
 * IDLE_WAIT_MAX_MS bounds ONE wait.  It is not about accuracy (the guest clock has already
 * advanced -- the wait is pure throttling, so a wait cut short only makes the emulator run a shade
 * faster).  It is about the tab staying answerable: a device or a `pause` message should never be
 * more than this far away, and `pcjsvax-6c9` is the standing reminder of what a console that goes
 * quiet looks like from outside.  A keystroke does not wait even this long -- see the `input`
 * message below, which cancels the timer outright.
 */
const IDLE_WAIT_MAX_MS = 50;

/** How often the Worker reports progress upstream.  Every slice would post 60 messages a second
    for numbers a human reads once a second. */
const STAT_MS = 500;

/**
 * fileBacking(file)
 *
 * `readRaw` over a picked local `File`.  *** THE FILE IS NEVER READ WHOLE. ***  `File.slice()`
 * returns a `Blob` that is a VIEW, not a copy, and FileReaderSync reads only that view -- so a
 * 1,048,624,640-byte container costs exactly the 64 KiB chunks the guest actually touches.
 */
function fileBacking(file)
{
    if (typeof FileReaderSync === "undefined") {
        throw new Error("vaxworker: FileReaderSync is unavailable in this Worker.  rq.js reads " +
            "disk blocks SYNCHRONOUSLY from inside the instruction stream, so there is no " +
            "asynchronous fallback that does not mean loading the whole image");
    }
    let fr = new FileReaderSync();
    return {
        byteLength: file.size,
        name: file.name,
        readRaw(offset, length) {
            let end = Math.min(offset + length, file.size);
            if (end <= offset) return new Uint8Array(0);
            return new Uint8Array(fr.readAsArrayBuffer(file.slice(offset, end)));
        }
    };
}

/**
 * urlBacking(url)
 *
 * `readRaw` over HTTP byte ranges, with a synchronous XHR -- legal in a Worker, and the only
 * synchronous HTTP read a browser has.
 *
 * *** THE SERVER MUST HONOUR `Range`. ***  Python's `http.server` DOES NOT: it ignores the header
 * and returns 200 with the whole body, which for a 1 GB image is exactly the failure this design
 * exists to avoid.  So a 200 where a 206 was asked for is refused BY NAME rather than silently
 * downloading a gigabyte per read.  Use the file picker, or serve with something that supports
 * ranges.  Filed as pcjsvax-ba6.
 */
function urlBacking(url)
{
    let head = new XMLHttpRequest();
    head.open("HEAD", url, false);
    head.send(null);
    if (head.status !== 200) throw new Error(`vaxworker: HEAD ${url} returned ${head.status}`);
    let byteLength = parseInt(head.getResponseHeader("Content-Length") || "0", 10);
    if (!(byteLength > 0)) throw new Error(`vaxworker: ${url} reported no Content-Length`);
    return {
        byteLength,
        name: url,
        readRaw(offset, length) {
            let end = Math.min(offset + length, byteLength) - 1;
            if (end < offset) return new Uint8Array(0);
            let xhr = new XMLHttpRequest();
            xhr.open("GET", url, false);
            xhr.responseType = "arraybuffer";
            xhr.setRequestHeader("Range", `bytes=${offset}-${end}`);
            xhr.send(null);
            if (xhr.status !== 206) {
                throw new Error(`vaxworker: ${url} answered a Range request with ${xhr.status}, ` +
                    `not 206.  This server ignores Range and would send the whole ` +
                    `${byteLength}-byte image on EVERY disk block.  Use the file picker instead`);
            }
            return new Uint8Array(xhr.response);
        }
    };
}

let machine = null, disk = null, running = false, t0 = 0, lastStat = 0, lastSteps = 0;
let idleWaitMs = 0, idleWaits = 0;

/* The yield primitive.  A MessageChannel round trip is a macrotask with NO 4 ms clamp -- which
   `setTimeout(0)` acquires after five nested calls, and five nested calls is a quarter of a second
   into a two-minute boot.  Measured cost is under 0.1 ms per yield against a 16 ms slice. */
const yieldChannel = new MessageChannel();
let armed = false;
let idleTimer = null;
yieldChannel.port1.onmessage = () => { armed = false; pump(); };
function schedule() {
    if (idleTimer !== null) { clearTimeout(idleTimer); idleTimer = null; }
    if (!armed) { armed = true; yieldChannel.port2.postMessage(0); }
}

/**
 * scheduleIdle(ms)
 *
 * pcjsvax-af8.  Re-arm LATER instead of immediately.  `wake()` below collapses this back to an
 * immediate re-arm, which is what makes a keystroke prompt rather than "within one wait".
 */
function scheduleIdle(ms)
{
    if (armed || idleTimer !== null) return;
    idleTimer = setTimeout(() => { idleTimer = null; pump(); }, ms);
}

/** Cancel an outstanding idle wait and run NOW.  Called on any message that gives the guest
    something to do -- today that is a keystroke, and a keystroke that arrives late reads as a hung
    console (pcjsvax-6c9), so this is not an optimisation. */
function wake()
{
    if (idleTimer !== null) { clearTimeout(idleTimer); idleTimer = null; schedule(); }
}

function post(msg, transfer) { self.postMessage(msg, transfer || []); }

function flushOutput()
{
    let bytes = machine.drainOutput();
    if (bytes) post({type: "out", bytes: bytes.buffer}, [bytes.buffer]);
}

function pump()
{
    if (!running || !machine) return;
    let r;
    try {
        r = machine.runSlice(SLICE_MS);
    } catch (e) {
        running = false;
        flushOutput();
        post({type: "error", message: String((e && e.stack) || e)});
        return;
    }
    flushOutput();
    let now = Date.now();
    if (now - lastStat >= STAT_MS) {
        let s = disk ? disk.stats() : null;
        post({type: "stat",
              steps: machine.steps,
              elapsed: (now - t0) / 1000,
              mips: (machine.steps - lastSteps) / (now - lastStat) / 1000,
              /* pcjsvax-af8: what fraction of the last reporting window the machine spent NOT
                 executing because the guest had nothing to do.  Reported rather than inferred, so
                 "is it actually idling" is answerable from the page instead of from a profiler. */
              idlePct: Math.min(100, 100 * idleWaitMs / Math.max(1, now - lastStat)),
              idleWaits,
              typed: machine.marks.map((m) => m.rule),
              disk: s && {reads: s.reads, writes: s.writes, rawBytes: s.rawBytes,
                          overlayBlocks: s.overlayBlocks, overlayBytes: s.overlayBytes,
                          overlayCeilingBytes: s.overlayCeilingBytes,
                          cacheResidentBytes: s.cacheResidentBytes,
                          cacheCeilingBytes: s.cacheCeilingBytes}});
        lastStat = now; lastSteps = machine.steps; idleWaitMs = 0; idleWaits = 0;
    }
    if (r.stop) { running = false; post({type: "stopped", reason: r.stop}); return; }
    /* pcjsvax-af8.  The guest fast-forwarded through `r.idleUsecs` of its own time rather than
       executing it, so give the host that time back instead of re-arming into another slice. */
    if (r.idleSleepMs > 0) {
        let ms = Math.min(IDLE_WAIT_MAX_MS, r.idleSleepMs);
        idleWaitMs += ms; idleWaits++; scheduleIdle(ms); return;
    }
    schedule();
}

self.onmessage = function(e)
{
    let msg = e.data;
    try {
        switch (msg.cmd) {

        case "start": {
            let backing = null;
            if (msg.disk) {
                backing = msg.disk.file ? fileBacking(msg.disk.file) : urlBacking(msg.disk.url);
                disk = overlayImageProvider({
                    byteLength: backing.byteLength,
                    readRaw: backing.readRaw,
                    name: backing.name,
                    readOnly: !!msg.diskReadOnly,
                    maxOverlayBlocks: msg.maxOverlayBlocks || undefined
                });
            }
            machine = new VaxMachine({
                romBytes: new Uint8Array(msg.rom),
                memBytes: ((msg.memMB || 16) * 1024 * 1024) >>> 0,
                disk,
                autoBoot: !!msg.autoBoot
            });
            post({type: "info",
                  devices: machine.deviceNames,
                  memBytes: machine.machine.memSize,
                  attach: machine.attachReport,
                  diskName: backing && backing.name,
                  filesystemBytes: disk && disk.filesystemBytes});
            t0 = lastStat = Date.now(); lastSteps = 0;
            running = true;
            schedule();
            break;
        }

        case "input":
            /* A keystroke is the one thing that must not wait out an idle timer (pcjsvax-6c9: a
               console that answers late is indistinguishable from a console that has hung).
               `wake()` collapses the pending wait into an immediate MessageChannel re-arm, so the
               character is injected on the next macrotask rather than up to IDLE_WAIT_MAX_MS
               later.  MEASURED under Node against the same driver: 0.0 ms. */
            if (machine) { machine.type(msg.text); wake(); }
            break;

        case "pause":
            running = false;
            if (idleTimer !== null) { clearTimeout(idleTimer); idleTimer = null; }
            post({type: "paused"});
            break;

        case "resume":
            if (machine && !running) { running = true; lastStat = Date.now(); schedule(); }
            break;

        default:
            post({type: "error", message: `vaxworker: unknown command ${msg.cmd}`});
        }
    } catch (err) {
        running = false;
        post({type: "error", message: String((err && err.stack) || err)});
    }
};

post({type: "ready"});
