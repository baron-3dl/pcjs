/**
 * @fileoverview pcjsvax-319 -- drive THIS PORT's KA655 machine through an OpenVMS boot attempt from
 *               a real ODS-2 volume and report where its console stream leaves the oracle's
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS, AND WHAT IT IS NOT
 * --------------------------------
 * This is a PROBE, not a differential.  It asserts nothing, it grades nothing, it has no
 * --selfcheck and it is not in the gate.  Its whole output is a console byte stream and a small
 * amount of accounting about how that stream was produced, so that a human (or the next item) can
 * put it beside the oracle transcript captured by pcjsvax-459 and name the first byte at which the
 * two stop agreeing.  Everything a differential would do -- an oracle in the same process, a
 * comparison, a coverage floor, mutations -- is deliberately absent, because pcjsvax-319's
 * deliverable is a DIVERGENCE POINT and a differential that stopped at the first mismatch would
 * report less than a full transcript does.
 *
 * WHY IT IS A SEPARATE FILE AND NOT A MODE OF conoutdiff.js
 * ---------------------------------------------------------
 * conoutdiff.js grades a SETTLED, input-free boot to `>>>` against a step-exact SIMH capture.  This
 * one types at the console, runs two orders of magnitude longer, and has a disk attached -- three
 * things that change what "settled" means.  Folding it in would have moved that differential's
 * budget seam, which HANDOFF.md's §0 records going stale twice already.
 *
 * WHAT IT MOUNTS
 * --------------
 * tests/rommachine.js's machine, with pcjsvax-319's one additive `opts` parameter:
 *   - `memSize`, so the oracle's `set cpu 128m` can be reproduced without a second hardcoded
 *     constant (pcjsvax-59f fixed 16MB as the SELF-TEST target; that ruling is about the self-test
 *     harness, not about VMS).  Default here is 16MB, i.e. the shipped machine.
 *   - `qbus`, which returns the RQDX3 window.  The RQVAX construction, the image-provider contract
 *     and the read-only open are tests/mscpharness.js's, IMPORTED rather than re-typed: a second
 *     copy of an attach sequence whose read-only arm silently changes the unit's capacity (see
 *     rq.js's attach() and mscpharness.js's fileImageProvider()) is exactly the drift standing
 *     rule 7 describes.
 *
 * THE VOLUME'S ATTACH MODE IS A PARAMETER, AND WHICH ONE YOU PICK IS OBSERVABLE IN THE TRANSCRIPT.
 * The default is READ-ONLY, because a probe run dozens of times while a divergence is chased must
 * not change what it is measuring between runs; HANDOFF.md §8 records Baron's confirmation that
 * writes to this particular file are permitted because it is a throwaway copy.  rq.js's attach()
 * reads the ABSENCE of the provider's `write` member as sim_disk_attach_ex2()'s "rb+" -> "rb"
 * fallback and forces UNIT_RO, and that is NOT cosmetic: MEASURED 2026-07-30, a read-only DUA0
 * takes OpenVMS to `%SYSTEM-I-MOUNTVER, VAX1$DUA0: has been write-locked.  Mount verification in
 * progress.` and no further, which is a difference from the oracle produced entirely by this
 * switch.  `--writable` selects the other arm; point `--volume` at a COPY when you use it.  Both
 * the provider's writability and the unit's capacity before and after autosize are printed, because
 * a read-only unit also skips autosize's `container < current` clamp.
 *
 * HOW THE CONSOLE IS TYPED AT
 * ---------------------------
 * The oracle is driven by SIMH `expect`/`send`.  There is no SCP here, so the same job is done by
 * console.js's injectChar() against a small rule list with the SAME one-shot semantics pcjsvax-459
 * records as necessary: a rule that re-fires injects `B DUA0` into VMS's date prompt and halts the
 * system.  A rule fires when its pattern appears in the console stream AFTER the byte offset at
 * which the previous rule fired, and each fires at most once.
 *
 * Characters are handed over ONE AT A TIME and only when the previous one has been consumed --
 * rxdbRd() clearing CSR_DONE -- because RXDB is a single byte with no silo: shovelling a whole line
 * in would overwrite characters the ROM had not read yet, and the resulting truncated command would
 * look exactly like a console defect.
 *
 * USAGE
 *   node --max-old-space-size=2048 machines/dec/vax/tests/vmsbootprobe.js [options]
 *
 *     --volume PATH     ODS-2 container to attach to DUA0 (default: pcjsvax-459's)
 *     --writable        attach DUA0 read/write (point --volume at a COPY)
 *     --rom PATH        default $PCJS_VAX_REPO/open-simh/VAX/ka655x.bin
 *     --mem MB          RAM size in MB (default 16; the oracle reference ran at 128)
 *     --max-steps N     instruction cap (default 400000000)
 *     --max-seconds S   wall-clock cap (default 900)
 *     --no-disk         build the SAME machine with NO RQ window at all, the control run
 *     --tick-scale N    divide the interval timer's rate by N (see makeTickScale)
 *     --quiet           omit the escaped byte-by-byte stream
 */

import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

import { VAX } from "../modules/v2/defines.js";
import { VAXStop, ROM_MAGIC_BYTE } from "../modules/v2/cpustate.js";
import ClkVAX, { INSTRS_PER_TICK } from "../modules/v2/clk.js";
import { makeRomMachine } from "./rommachine.js";
import {
    RQVAX, RQ_BASE, IOLN_RQ, RQDX3_CTYPE, fileImageProvider, vaxRepo, hex
} from "./mscpharness.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/** pcjsvax-459's volume.  HANDOFF.md §8: no OpenVMS media is shipped or committed; this is a PATH,
    and its absence is reported by name (standing rule 6) rather than passing quietly. */
const DEFAULT_VOLUME = "/home/baron/vax1/data/d0.dsk";

/** console.js's CSR_DONE.  Not imported because console.js does not export it; it is the architected
    bit at the architected offset every VAX CSR-shaped device shares, and it is read ONLY to answer
    "has the ROM taken the last character yet". */
const CSR_DONE = 0x0080;

/**
 * THE INPUT SCRIPT, and it is pcjsvax-459's `expect`/`send` list transcribed one-for-one.
 *
 * `once` is not optional and not a nicety: 459 records that a `>>>` rule without a match count
 * re-fires mid-boot and injects `B DUA0` into VMS's date prompt.  Everything here fires at most once
 * and only on text that appeared after the previous rule fired.
 */
const INPUT_RULES = [
    {name: "console-prompt", match: ">>>", send: "B DUA0\r"},
    {name: "vms-date-prompt", match: "PLEASE ENTER DATE AND TIME", send: "30-JUL-2026 12:00\r"}
];

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }
function hasArg(name) { return process.argv.indexOf(name) >= 0; }

function findRom(pathArg)
{
    let p = pathArg || path.join(vaxRepo(), "open-simh/VAX/ka655x.bin");
    if (!fs.existsSync(p)) throw new Error(`ka655x.bin not found at ${p}; pass --rom PATH`);
    return new Uint8Array(fs.readFileSync(p));
}

/** conoutdiff.js's escaper, same conventions, so two transcripts can be diffed literally. */
function esc(bytes)
{
    let out = "";
    for (let b of bytes) {
        if (b === 0x0D) out += "\\r";
        else if (b === 0x0A) out += "\\n";
        else if (b === 0x1B) out += "\\e";
        else if (b >= 0x20 && b < 0x7F) out += String.fromCharCode(b);
        else out += "\\x" + hex(b, 2);
    }
    return out;
}

/** The same stream with CR/LF made real, which is the form that lines up with the oracle capture
    after `tr '\r' '\n'` (pcjsvax-459's grep gotcha). */
function readable(bytes)
{
    let s = "";
    for (let b of bytes) {
        if (b === 0x0D) s += "\n";
        else if (b === 0x0A) s += "\n";
        else if (b === 0x1B) s += "";
        else if (b >= 0x20 && b < 0x7F) s += String.fromCharCode(b);
    }
    return s.replace(/\n{3,}/g, "\n\n");
}

/**
 * makeQbus(volume, failures)
 *
 * The `opts.qbus` callback rommachine.js calls.  It builds the controller mscpharness.js's
 * makeMachine() builds, with the SAME constructor arguments, and attaches the volume to unit 0.
 */
function makeQbus(volume, writable, report)
{
    return function({cqbic}) {
        let rq = new RQVAX(cqbic, {cnum: 0, ctype: RQDX3_CTYPE});
        let capacBefore = rq.units[0].capac;
        let prov = fileImageProvider(volume, {readOnly: !writable});
        rq.attach(0, prov, {});
        report.rq = rq;
        report.prov = prov;
        report.attach = {
            path: volume,
            containerBytes: prov.byteLength,
            filesystemBytes: prov.filesystemBytes,
            providerWritable: prov.writable,
            capacBefore, capacAfter: rq.units[0].capac,
            flags: rq.units[0].flags
        };
        return {windows: [{base: RQ_BASE, length: IOLN_RQ, dev: rq}], tickDev: rq};
    };
}

/**
 * makeTickScale(n)
 *
 * THE ONE EXPERIMENT THIS PROBE PERFORMS ON THE SHIPPED CODE, and it COMPOSES over ClkVAX's own
 * tick() rather than replacing it (HANDOFF.md standing rule 11): the original is called only every
 * n'th time, so the EFFECTIVE rate becomes clk.js's `INSTRS_PER_TICK * n` and every other thing
 * tick() does -- the TODR increment, the interrupt raise, the `todrBlow` gate -- is still the
 * shipped code doing it.
 *
 * WHY IT EXISTS.  clk.js's `INSTRS_PER_TICK` is 200, a deliberate deterministic stand-in for
 * vax_stddev.c's `clk_svc` 100 Hz REAL-TIME schedule, so one guest second costs 20,000 retired
 * instructions here where the oracle's calibrated timer makes it cost however many the host can run
 * in a real second.  MEASURED 2026-07-30 over the SAME startup on both engines (`%STDRV-I-STARTUP`
 * to `INTERnet Started`): oracle 31.67 guest seconds, this port 26,920.6 -- a factor of 850.  This
 * switch is how the next item tests whether a divergence is a consequence of that rate without
 * editing a shipped module: run once at 1 and once at n, and compare.
 *
 * It is a PROBE OPTION and nothing is graded on it.
 *
 * @param {number} n
 * @returns {Function} an uninstaller
 */
function makeTickScale(n)
{
    let orig = ClkVAX.prototype.tick, i = 0;
    ClkVAX.prototype.tick = function(cpu) { if ((++i % n) === 0) return orig.call(this, cpu); };
    return () => { ClkVAX.prototype.tick = orig; };
}

/**
 * run(opts)
 *
 * One boot attempt.  Every bound here is ABSOLUTE and fails the run rather than scaling with
 * anything (HANDOFF.md standing rules 4 and 14): an instruction cap, a wall-clock cap, and a peak
 * heap sample taken on the same schedule as the console poll.
 */
function run(opts)
{
    let romBytes = findRom(opts.rom);
    let unscale = opts.tickScale > 1 ? makeTickScale(opts.tickScale) : null;
    let qreport = {};
    let machine = makeRomMachine(romBytes, false, false, {
        memSize: opts.memBytes,
        qbus: opts.noDisk ? undefined : makeQbus(opts.volume, opts.writable, qreport)
    });
    let {cpu, consoleDev} = machine;
    cpu.reset();
    cpu.boot(opts.magicByte);

    /* Bus faults per (address, direction), for the report only.  romdiff.js's FaultGrader owns
       grading them; duplicating that here is the drift standing rule 7 warns about. */
    let faults = new Map(), faultEvents = 0;
    let realOnBusFault = cpu.onBusFault.bind(cpu);
    cpu.onBusFault = function(addr, access, ...rest) {
        let key = `${hex(addr >>> 0)}:${(access & VAX.ACCESS.WRITE) ? "W" : "R"}`;
        let e = faults.get(key);
        if (!e) faults.set(key, e = {addr: addr >>> 0, access, count: 0, firstStep: steps + 1});
        e.count++; faultEvents++;
        return realOnBusFault(addr, access, ...rest);
    };

    let steps = 0, stop = null, stopStep = null;
    let ruleIdx = 0, scanFrom = 0, pending = null, pendingIdx = 0, sent = [];
    let peakHeap = 0, t0 = Date.now(), deadline = t0 + opts.maxSeconds * 1000, lastBeat = 0;
    /* Console output length at the moment each rule fired, so the transcript can be split at the
       exact byte the probe typed rather than at a string search done afterwards. */
    let marks = [];
    /* The stream is polled every CHUNK instructions rather than every instruction: injectChar() and
       a substring search per instruction would dominate the run, and the ROM's own polling loop is
       thousands of instructions wide. */
    const CHUNK = 2000;

    try {
        while (steps < opts.maxSteps) {
            for (let i = 0; i < CHUNK && steps < opts.maxSteps; i++) { cpu.stepCPU(1); steps++; }

            /* Feed one character at a time, and only into an empty RXDB. */
            if (pending !== null) {
                while (pendingIdx < pending.length && !(consoleDev.rxcs & CSR_DONE)) {
                    consoleDev.injectChar(pending.charCodeAt(pendingIdx++) & 0xFF);
                }
                if (pendingIdx >= pending.length) { pending = null; pendingIdx = 0; }
            } else if (ruleIdx < INPUT_RULES.length) {
                let text = "";
                for (let k = scanFrom; k < consoleDev.output.length; k++) {
                    text += String.fromCharCode(consoleDev.output[k]);
                }
                let r = INPUT_RULES[ruleIdx];
                let at = text.indexOf(r.match);
                if (at >= 0) {
                    scanFrom = scanFrom + at + r.match.length;
                    marks.push({rule: r.name, atByte: consoleDev.output.length, atStep: steps});
                    sent.push(r);
                    pending = r.send; pendingIdx = 0;
                    ruleIdx++;
                }
            }

            let mu = process.memoryUsage();
            let used = mu.heapUsed + mu.external;
            if (used > peakHeap) peakHeap = used;
            /* A heartbeat on STDERR, so a run bounded in the hundreds of seconds is observable
               while it happens rather than only when it ends.  It never touches the machine. */
            if (opts.heartbeat && steps - lastBeat >= opts.heartbeat) {
                lastBeat = steps;
                process.stderr.write(`  .. ${(steps / 1e6).toFixed(0)}M steps, ` +
                    `${((Date.now() - t0) / 1000).toFixed(0)} s, console ${consoleDev.output.length} bytes, ` +
                    `PC=${hex(cpu.regs[15] >>> 0)}\n`);
            }
            if (Date.now() > deadline) { stop = "WALL-CLOCK CAP"; stopStep = steps; break; }
        }
        if (stop === null && steps >= opts.maxSteps) { stop = "INSTRUCTION CAP"; stopStep = steps; }
    } catch (e) {
        if (!(e instanceof VAXStop)) throw e;
        stop = `VAXStop ${e.reason} detail=${e.detail}`;
        stopStep = steps;
    }

    if (unscale) unscale();
    return {
        bytes: Uint8Array.from(consoleDev.output), steps, stop, stopStep, faults, faultEvents,
        marks, sentNames: sent.map((r) => r.name), rulesLeft: INPUT_RULES.slice(ruleIdx).map((r) => r.name),
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0, elapsedMs: Date.now() - t0, peakHeap,
        deviceNames: machine.devices.map((d) => d.name),
        memSize: machine.memSize, qreport
    };
}

function main()
{
    let opts = {
        volume: getArg("--volume", DEFAULT_VOLUME),
        rom: getArg("--rom", null),
        memMB: parseInt(getArg("--mem", "16"), 10),
        maxSteps: parseInt(getArg("--max-steps", "400000000"), 10),
        maxSeconds: parseInt(getArg("--max-seconds", "900"), 10),
        noDisk: hasArg("--no-disk"),
        writable: hasArg("--writable"),
        quiet: hasArg("--quiet"),
        heartbeat: parseInt(getArg("--heartbeat", "50000000"), 10),
        tickScale: parseInt(getArg("--tick-scale", "1"), 10),
        magicByte: parseInt(getArg("--magic", String(ROM_MAGIC_BYTE)), 10)
    };
    opts.memBytes = (opts.memMB * 1024 * 1024) >>> 0;
    if (!opts.noDisk && !fs.existsSync(opts.volume)) {
        console.log(`vmsbootprobe: the volume ${opts.volume} does not exist.  Pass --volume PATH.`);
        process.exit(2);
    }

    console.log(`vmsbootprobe (pcjsvax-319) -- a PROBE, not a gate: it asserts nothing`);
    console.log(`  tree      ${__dirname}`);
    console.log(`  memory    ${opts.memMB} MB (oracle reference pcjsvax-459 ran at 128 MB)`);
    console.log(`  volume    ${opts.noDisk ? "(none -- --no-disk control run)" : opts.volume} ` +
                `${opts.noDisk ? "" : (opts.writable ? "READ/WRITE" : "READ-ONLY")}`);
    console.log(`  caps      ${opts.maxSteps} instructions / ${opts.maxSeconds} s wall clock`);
    console.log(`  clk       INSTRS_PER_TICK ${INSTRS_PER_TICK} x tick-scale ${opts.tickScale} = ` +
                `${INSTRS_PER_TICK * opts.tickScale} instructions per 100 Hz tick, i.e. ` +
                `${INSTRS_PER_TICK * opts.tickScale * 100} instructions per GUEST second`);

    let r = run(opts);

    console.log(`\n  devices   ${r.deviceNames.join(", ")}`);
    if (r.qreport.attach) {
        let a = r.qreport.attach;
        console.log(`  DUA0      container ${a.containerBytes} bytes, filesystem ` +
            `${a.filesystemBytes === undefined ? "(not recognised)" : a.filesystemBytes} bytes, ` +
            `provider ${a.providerWritable ? "WRITABLE" : "read-only"}; unit capac ` +
            `${a.capacBefore} -> ${a.capacAfter} blocks, flags 0x${hex(a.flags, 4)}`);
    }
    console.log(`  ran       ${r.steps} instructions in ${(r.elapsedMs / 1000).toFixed(1)} s ` +
        `(${(r.steps / Math.max(1, r.elapsedMs) / 1000).toFixed(2)} M/s), peak heap ` +
        `${(r.peakHeap / (1 << 20)).toFixed(0)} MB`);
    console.log(`  stopped   ${r.stop} at step ${r.stopStep}, PC=${hex(r.pc)} PSL=${hex(r.psl)}`);
    console.log(`  typed     ${r.sentNames.length ? r.sentNames.join(", ") : "(nothing)"}` +
        `${r.rulesLeft.length ? `; NEVER FIRED: ${r.rulesLeft.join(", ")}` : ""}`);
    for (let m of r.marks) console.log(`            "${m.rule}" matched at console byte ${m.atByte}, step ${m.atStep}`);
    console.log(`  console   ${r.bytes.length} bytes`);
    console.log(`  faults    ${r.faultEvents} event(s) over ${r.faults.size} distinct (address, direction)`);
    let sorted = [...r.faults.values()].sort((a, b) => a.firstStep - b.firstStep);
    for (let f of sorted.slice(0, 40)) {
        console.log(`            ${hex(f.addr)} ${(f.access & VAX.ACCESS.WRITE) ? "W" : "R"} ` +
            `x${f.count}, first at step ${f.firstStep}`);
    }
    if (sorted.length > 40) console.log(`            ... and ${sorted.length - 40} more`);

    if (!opts.quiet) {
        console.log(`\n----- CONSOLE STREAM, ESCAPED -----`);
        console.log(esc(r.bytes));
    }
    console.log(`\n----- CONSOLE STREAM, READABLE -----`);
    console.log(readable(r.bytes));
    if (r.qreport.prov) r.qreport.prov.close();
}

main();
