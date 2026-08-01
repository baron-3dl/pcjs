/**
 * @fileoverview pcjsvax-319 -- drive THIS PORT's KA655 machine through an OpenVMS boot attempt from
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
 *               a real ODS-2 volume and report where its console stream leaves the oracle's
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
 *     --console-file P  rewrite the console stream to P on every heartbeat (watch a run live)
 *     --login USER      type USER at a BARE `Username: ` prompt (see loginRules); off by default
 *     --password PW     and PW at the `Password: ` that follows.  A run with --login and no
 *                       --password still proves the prompt answered: the guest prints `Password: `
 *
 *   pcjsvax-6c9's instrument -- all OFF by default, all observation only (see makeInputTrace):
 *     --trace-input     wrap the console model, the interrupt seam and the dispatch chokepoint
 *     --trace-arm NAME  arm the log when this INPUT_RULES rule fires (default post-startup-wakeup)
 *     --trace-limit N / --trace-pre N       bounds on the log after / ring before arming
 *     --trace-exc-after N   after the TTI vector dispatches, log EVERY exception/interrupt for N
 *     --trace-pc N      capture N executed PCs from the injection, for diffing against SIMH's
 *                       own `SET CPU HISTORY` -- this is how pcjsvax-6c9 found its divergence
 *     --trace-stop-after N  end the run N instructions after arming
 *     --snap-pc HEX --snap-reg N --snap-len N   one register+memory snapshot the first time the
 *                       guest is about to execute HEX, to diff against SIMH stopped on a BREAK
 *     --dump-va HEX --dump-len N   guest virtual memory, read in kernel mode after the run
 */

import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

import { VAX } from "../modules/v2/defines.js";
import { VAXStop, ROM_MAGIC_BYTE } from "../modules/v2/cpustate.js";
import ClkVAX, { INSTRS_PER_TICK, USECS_PER_INSTR } from "../modules/v2/clk.js";
import { IdleThrottle } from "../modules/v2/idle.js";        /* pcjsvax-af8 */
import { makeRomMachine } from "./rommachine.js";

/** pcjsvax-af8.  Node's only SYNCHRONOUS sleep, and this probe's run loop is synchronous.  The
    buffer is never written, so the wait always runs to its timeout; `Atomics.wait` on the main
    thread requires a SharedArrayBuffer, which is why it is not a plain Int32Array. */
const IDLE_SLEEPER = new Int32Array(new SharedArrayBuffer(4));

/** Bound on ONE idle wait, milliseconds.  See browser/vaxworker.js's IDLE_WAIT_MAX_MS for the
    reasoning; it is throttling, not timekeeping, so a short wait costs accuracy in nothing. */
const IDLE_WAIT_MAX_MS = 50;


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

/** console.js's SCB_TTI, the console-receive SCB offset (vax_defs.h:397).  Read ONLY by the
    pcjsvax-6c9 instrument, to recognise the dispatch it is watching for. */
const SCB_TTI_VEC = 0x00F8;

/**
 * THE INPUT SCRIPT, and it is pcjsvax-459's `expect`/`send` list transcribed one-for-one.
 *
 * `once` is not optional and not a nicety: 459 records that a `>>>` rule without a match count
 * re-fires mid-boot and injects `B DUA0` into VMS's date prompt.  Everything here fires at most once
 * and only on text that appeared after the previous rule fired.
 */
const INPUT_RULES = [
    {name: "console-prompt", match: ">>>", send: "B DUA0\r"},
    {name: "vms-date-prompt", match: "PLEASE ENTER DATE AND TIME", send: "30-JUL-2026 12:00\r"},
    /* The V7.3 INSTALLATION procedure asks the same question in different words and different case
       ("* Please enter the date and time (DD-MMM-YYYY HH:MM)"), so the rule above never fires on it
       and the boot stalls at the prompt.  Matched on the mixed-case wording rather than folding case
       on the rule above, because folding case would let one rule swallow both prompts and make the
       installer and the booted system indistinguishable in the transcript. */
    {name: "vms-install-date-prompt", match: "Please enter the date and time", send: "30-JUL-2026 12:00\r"},
    /* MEASURED 2026-07-30: when startup finishes, the console goes SILENT and stays silent -- the
       last thing printed is the startup job's accounting block, ending "Elapsed time:".  That is not
       a hang and not a divergence: LOGINOUT on OPA0 is waiting for a keystroke before it paints the
       login prompt, exactly as a real console terminal does.  Nothing in this probe types unless a
       rule tells it to, so without this the run parks forever one RETURN short of the prompt.
       Matched on "Elapsed time:" because it is the LAST line of that block; matching the earlier
       "job terminated" fires mid-block and the RETURN is swallowed before LOGINOUT is listening. */
    /* `quiet` -- MEASURED 2026-07-30, pcjsvax-6c9, and it is the whole difference between this
       rule waking VMS and being swallowed.  "Elapsed time:" is the last LINE of the accounting
       block but not its last BYTE: the value, the CRLF and the trailing blank line are still
       queued in the terminal driver's output path when the match fires, so a character injected
       at the very next chunk boundary arrives at a console the driver considers BUSY WRITING.
       VMS's console ISR (SYSLOA655 at 80ACA689) reaches `TSTW 70(R1)` on the OPA0 UCB and takes
       the not-zero arm, which is NOT the unsolicited-input arm that notifies the job controller.
       The oracle never hits this because SIMH's own `tti_svc` polls the keyboard on the CALIBRATED
       100 Hz clock -- on a modern host that is ~1e6 instructions after `expect` fired, by which
       time the line has long drained.  `quiet` reproduces that by construction rather than by
       luck: hold the keystroke until the console has emitted NOTHING for this many instructions,
       which is also what a human at a terminal does. */
    {name: "post-startup-wakeup", match: "Elapsed time:", send: "\r", quiet: 200000}
];

/**
 * THE LOGIN RULES, and they are OPT-IN (`--login USER [--password PW]`) rather than part of the
 * list above, so the default transcript is byte-for-byte the one every earlier item captured.
 *
 * WHY TYPING A USERNAME IS THE MEASUREMENT AND `Username:` ON ITS OWN IS NOT.  pcjsvax-cff records
 * two false positives already scored here: `Username:` also appears as a PADDED FIELD inside the
 * OPCOM audit record both engines print during startup (`Username:                 SYSTEM`), and
 * `LOGINOUT.EXE` -- the other thing that was grepped for -- appears NOWHERE in a successful boot,
 * because it is the image name printed only in the audit record a FAILED login generates.  The only
 * sound test is to type a name at the prompt and require the guest to answer it.
 *
 * `atEnd` is what separates the real prompt from the padded field WITHOUT relying on the padded
 * field being absent.  A rule with `atEnd` matches only when its text is the LAST thing on the
 * console, which a bare `Username: ` is (LOGINOUT then waits) and an audit record's field is not
 * (its own value follows on the same line).  Combined with `quiet` -- hold the keystroke until the
 * console has been silent, the condition pcjsvax-6c9 measured for the wake-up RETURN -- the pair
 * says "the guest printed this and then stopped talking", which is what a prompt IS.
 *
 * MEASURED 2026-07-30 on the VMS 5.5-2H4 volume built by pcjsvax-cff: its startup prints NO padded
 * `Username:` audit field at all, so on 5.5 the discrimination is not even exercised; it is written
 * this way because V7.1's startup does print one.
 *
 * THERE IS A DELIBERATE HARD DEADLINE ON THE ANSWER, and it is the guest's, not ours: LOGINOUT gives
 * up on an unanswered prompt and prints `Error reading command input` / `Timeout period expired`.
 * MEASURED on this port, that is what a run with no login rule ends with -- which makes "the prompt
 * was answered" a fact the transcript carries rather than a claim this file makes.
 */
function loginRules(user, password)
{
    let rules = [];
    if (user) rules.push({name: "login-username", match: "Username: ", send: user + "\r", atEnd: true, quiet: 20000});
    if (user && password) rules.push({name: "login-password", match: "Password: ", send: password + "\r", atEnd: true, quiet: 20000});
    return rules;
}

/**
 * REPEATABLE rules, consulted only when no one-shot rule above matches.
 *
 * The OpenVMS VAX V7.3 installation procedure is a long sequence of questions, and the volume Baron
 * supplied is the FIRST-BOOT half of a DEC installation: the media is a BACKUP saveset restored onto
 * the disk, and booting it runs a procedure that self-assembles the system.  Most of its questions
 * want their own bracketed default, so `[...]:` answered with a bare RETURN carries the procedure a
 * long way without this file having to have transcribed every prompt in advance.
 *
 * `]: ` is deliberately narrow.  It is the tail of DEC's own defaulted-prompt convention
 * ("* Enter the volume label for this system disk [OVMSVAXSYS]: "), so it appears when input is
 * WANTED rather than in ordinary output.  A question with no bracketed default -- a bare Y/N -- will
 * NOT match, and the run will park at it; that is intended, because guessing an unprompted yes/no
 * during an OS installation is how you silently answer something that mattered.  When a run parks,
 * read the console file, add a one-shot rule for that specific question, and re-run.
 */
const REPEAT_RULES = [
    {name: "accept-bracketed-default", match: "]: ", send: "\r"}
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
 * makeInputTrace(cpu, consoleDev, getStep, opts)
 *
 * pcjsvax-6c9's INSTRUMENT.  Observation only, and it COMPOSES over the shipped instance methods
 * rather than replacing them (HANDOFF.md standing rule 11): every wrapper calls the original and
 * reports what the original did.  Nothing here changes a register, an interrupt request or a clock,
 * so a traced run and an untraced run execute the same instructions.
 *
 * It answers, by running, the three questions pcjsvax-6c9 asks:
 *   1. what RXCS (and therefore CSR_IE) is at the instant injectChar() lands;
 *   2. whether the TTI request that injectChar() raises is ever ARBITRATED (getVector at IPL 0x14)
 *      and DISPATCHED (intexc with that vector);
 *   3. what the guest does with RXCS/RXDB afterwards.
 *
 * Tracing is ARMED by name -- the first time the named input rule fires -- so the pre-boot traffic
 * (thousands of ROM polls) does not swamp the window that matters.  A bounded ring keeps the last
 * `pre` events before arming, and the log stops at `limit` events after it; counters keep running
 * either way, so a truncated log still reports totals.
 *
 * @param {Object} cpu
 * @param {Object} consoleDev
 * @param {function():number} getStep
 * @param {{limit:number, pre:number}} opts
 * @returns {Object} the trace state (`arm(name)`, `counts`, `pre`, `events`)
 */
function makeInputTrace(cpu, consoleDev, getStep, opts)
{
    let exc = cpu.exc;
    let st = {armed: false, armedAt: null, events: [], pre: [], counts: {}, limit: opts.limit, preMax: opts.pre};
    st.arm = function(name) {
        if (st.armed) return;
        st.armed = true; st.armedAt = getStep();
    };
    const bump = (k) => { st.counts[k] = (st.counts[k] || 0) + 1; };
    const where = () => `PC=${hex(cpu.regs[15] >>> 0)} PSL=${hex(cpu.psl >>> 0)} IPL=${hex((cpu.psl >>> 16) & 0x1F, 2)}`;
    /* A polling guest reads RXCS thousands of times with an identical result; logging each one
       buries the transitions that matter.  Consecutive identical event TEXTS (the step number is
       excluded from the comparison) collapse into one line with a repeat count -- no event is
       dropped, and the first and last step of each run are both reported. */
    const log = (s) => {
        let sink = st.armed ? st.events : st.pre;
        let last = sink[sink.length - 1];
        if (last && last.text === s) { last.n++; last.lastStep = getStep(); return; }
        if (!st.armed) { sink.push({text: s, n: 1, step: getStep(), lastStep: getStep()});
                         if (sink.length > st.preMax) sink.shift(); return; }
        if (sink.length < st.limit) sink.push({text: s, n: 1, step: getStep(), lastStep: getStep()});
        else st.dropped++;
    };
    st.dropped = 0;
    st.fmt = (e) => `[${e.step}${e.n > 1 ? `..${e.lastStep}, x${e.n}` : ""}] ${e.text}`;

    for (let name of ["rxcsRd", "rxdbRd", "txcsRd"]) {
        let orig = consoleDev[name];
        consoleDev[name] = function(...a) {
            let v = orig.apply(this, a);
            bump(name);
            if (name !== "txcsRd") log(`${name} -> ${hex(v >>> 0, 4)} rxcs=${hex(this.rxcs >>> 0, 4)} ${where()}`);
            return v;
        };
    }
    for (let name of ["rxcsWr", "txcsWr"]) {
        let orig = consoleDev[name];
        consoleDev[name] = function(val) {
            bump(name);
            if (name === "rxcsWr") log(`rxcsWr ${hex(val >>> 0, 4)} (was ${hex(this.rxcs >>> 0, 4)}) ${where()}`);
            return orig.call(this, val);
        };
    }
    {
        let orig = consoleDev.txdbWr;
        consoleDev.txdbWr = function(val) { bump("txdbWr"); return orig.call(this, val); };
    }
    {
        let orig = consoleDev.injectChar;
        consoleDev.injectChar = function(byte) {
            bump("injectChar");
            let before = this.rxcs >>> 0;
            let r = orig.call(this, byte);
            log(`injectChar ${hex(byte & 0xFF, 2)} rxcs ${hex(before, 4)} -> ${hex(this.rxcs >>> 0, 4)} ` +
                `IE=${(before & 0x40) ? 1 : 0} ` +
                `intReq[0]=${hex(exc.intReq[0] >>> 0, 8)} ${where()}`);
            /* The PC capture starts HERE, not at the TTI dispatch, because injectChar() is called
               between two instruction chunks while the dispatch happens inside one -- starting it
               on the dispatch would begin at the next chunk boundary and miss the driver's ISR
               entirely, which is the one place the two engines most need to be compared. */
            if (st.armed && st.pcTrace === null && st.pcWanted > 0) st.pcTrace = [];
            return r;
        };
    }
    for (let name of ["raiseInterrupt", "clearInterrupt"]) {
        let orig = exc[name];
        exc[name] = function(lvl, bit) {
            if (lvl === 0x14 && (bit === 8 || bit === 9)) {
                bump(`${name}-${bit === 8 ? "TTI" : "TTO"}`);
                if (bit === 8) log(`${name}(TTI) ${where()}`);
            }
            return orig.call(this, lvl, bit);
        };
    }
    {
        let orig = exc.getVector;
        exc.getVector = function(c, lvl) {
            let v = orig.call(this, c, lvl);
            if (lvl === 0x14) { bump("getVector-0x14"); log(`getVector(lvl=14) -> ${hex(v >>> 0, 4)} ${where()}`); }
            return v;
        };
    }
    {
        let orig = exc.intexc;
        exc.intexc = function(c, vec, lvl, ie, ...rest) {
            bump(`intexc-vec${hex(vec >>> 0, 4)}`);
            /* Every dispatch is COUNTED; only IPL>=0x14 hardware interrupts are LOGGED by default,
               because the 100 Hz clock and the software-interrupt levels would bury the window.
               Once the TTI vector has been dispatched, --trace-exc-after opens the log to EVERY
               dispatch for that many events, which is what makes "what did the guest DO with the
               character" an observation rather than an inference. */
            if (lvl >= 0x14 || st.excOpen-- > 0) {
                log(`intexc vec=${hex(vec >>> 0, 4)} lvl=${hex(lvl, 2)} ${where()}`);
            }
            if ((vec >>> 0) === SCB_TTI_VEC) {
                if (st.pcTrace === null && st.pcWanted > 0) st.pcTrace = [];
                st.excOpen = st.excWanted;
            }
            return orig.call(this, c, vec, lvl, ie, ...rest);
        };
    }
    st.pcTrace = null;
    st.pcWanted = opts.pc | 0;
    st.excOpen = 0;
    st.excWanted = opts.exc | 0;
    /* --snap-pc: one snapshot of the register file plus a block of guest memory based at one of
       those registers, taken the first time the guest is ABOUT to execute a named instruction.
       It exists to be diffed against the oracle stopped at a SIMH breakpoint on the same PC.
       NOTE the perturbation this costs, since HANDOFF standing rule 11 applies to probes: the
       memory read goes through mmu.readData() and so can FILL A TLB ENTRY the guest had not
       filled yet.  It is taken one instruction before the guest reads the same page itself, so
       the entry would have been filled either way -- but it is not free, and this option must
       stay off in any run whose instruction stream is being compared. */
    st.snapPc = opts.snapPc;
    st.snapReg = opts.snapReg | 0;
    st.snapLen = opts.snapLen | 0;
    st.snap = null;
    st.takeSnap = function() {
        let base = cpu.regs[st.snapReg] | 0, bytes = [], error = null, savedPsl = cpu.psl;
        try {
            cpu.psl = 0;
            for (let i = 0; i < st.snapLen; i++) bytes.push(cpu.mmu.readData((base + i) | 0, 1, cpu.accR()) & 0xFF);
        } catch (e) { error = String((e && e.message) || e); }
        cpu.psl = savedPsl;
        st.snap = {step: getStep(), pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0,
                   regs: Array.from(cpu.regs, (r) => r >>> 0), base: base >>> 0, bytes, error};
    };
    return st;
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
    /* pcjsvax-af8: guest-idle detection, ON by default here.  This probe's whole job is to sit at
       a login prompt for minutes at a time; without it that is minutes of a pinned core.  `--no-idle`
       turns it off, which is how the before/after measurement in the item was taken. */
    cpu.idleEnable = !opts.noIdle;
    if (!cpu.setIdleMode(opts.idleMode)) {
        console.log(`vmsbootprobe: --idle-mode ${opts.idleMode} is not an OS idle.js knows`);
        process.exit(2);
    }
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
    let fired = new Set(), scanFrom = 0, pending = null, pendingIdx = 0, sent = [];
    /* `quiet` support (see INPUT_RULES): the console length and the step at which it last changed,
       sampled once per chunk, plus the number of quiet instructions a held keystroke still owes. */
    let lastOutLen = 0, lastOutStep = 0, holdQuiet = 0;
    let peakHeap = 0, t0 = Date.now(), deadline = t0 + opts.maxSeconds * 1000, lastBeat = 0;
    let idleSleptMs = 0, idleWaits = 0;                 /* pcjsvax-af8, reported at the end */
    /* HOW LONG to sleep is a measured host rate, not the guest time skipped -- see
       modules/v2/idle.js's IdleThrottle for the measurement that decided that. */
    let throttle = new IdleThrottle({maxSleepMs: IDLE_WAIT_MAX_MS});
    /* Console output length at the moment each rule fired, so the transcript can be split at the
       exact byte the probe typed rather than at a string search done afterwards. */
    let marks = [];
    /* The stream is polled every CHUNK instructions rather than every instruction: injectChar() and
       a substring search per instruction would dominate the run, and the ROM's own polling loop is
       thousands of instructions wide. */
    const CHUNK = 2000;

    /* pcjsvax-6c9's instrument -- observation only, see makeInputTrace().  Off unless asked for. */
    let trace = opts.traceInput
        ? makeInputTrace(cpu, consoleDev, () => steps,
              {limit: opts.traceLimit, pre: opts.tracePre, pc: opts.tracePc, exc: opts.traceExcAfter,
               snapPc: opts.snapPc === null ? null : (parseInt(opts.snapPc, 16) >>> 0),
               snapReg: opts.snapReg, snapLen: opts.snapLen})
        : null;
    let armStep = null;

    try {
        while (steps < opts.maxSteps) {
            let chunkT0 = Date.now();               /* pcjsvax-af8: pure execution time, no sleep in it */
            /* Two copies of the same loop rather than a per-instruction `if`: the PC capture is a
               diagnostic window measured in thousands of instructions inside a run measured in
               hundreds of millions, and a branch on every one of those would be paid by every run
               whether or not it asked for a trace. */
            let wantPc = trace && trace.pcTrace !== null && trace.pcTrace.length < trace.pcWanted;
            let wantSnap = trace && trace.armed && trace.snapPc !== null && trace.snap === null;
            if (wantPc || wantSnap) {
                for (let i = 0; i < CHUNK && steps < opts.maxSteps; i++) {
                    if (wantPc && trace.pcTrace.length < trace.pcWanted) trace.pcTrace.push(cpu.regs[15] >>> 0);
                    if (wantSnap && trace.snap === null && (cpu.regs[15] >>> 0) === trace.snapPc) trace.takeSnap();
                    cpu.stepCPU(1); steps++;
                }
            } else {
                for (let i = 0; i < CHUNK && steps < opts.maxSteps; i++) { cpu.stepCPU(1); steps++; }
            }

            /*
             * pcjsvax-af8.  `idleUsecs` is guest time the CPU fast-forwarded through rather than
             * executing.  The GUEST CLOCK HAS ALREADY ADVANCED BY IT -- this sleep is pure host
             * throttling, and skipping the sleep would keep correct time and burn a core, which is
             * exactly what this item exists to stop.  Atomics.wait on a zeroed SharedArrayBuffer is
             * the only SYNCHRONOUS sleep Node has, and synchronous is required: this is a plain
             * `while` loop with no event loop to return to.
             */
            let idleUs = cpu.takeIdleUsecs();
            if (idleUs === 0) throttle.noteBusy(CHUNK, Date.now() - chunkT0);
            let ms = throttle.sleepMsFor(idleUs / USECS_PER_INSTR);
            if (ms > 0) { idleSleptMs += ms; idleWaits++; Atomics.wait(IDLE_SLEEPER, 0, 0, ms); }

            /* Console quiescence, sampled once per chunk -- the clock a `quiet` rule waits on.
               GUEST TIME, not instructions retired: see browser/vaxmachine.js's pumpInput() for the
               measurement (pcjsvax-af8).  `cpu.nTotalCycles` counts elided instructions too, so a
               `quiet` threshold means the same number of guest microseconds whether the machine
               idled through them or executed them. */
            let guestNow = cpu.nTotalCycles;
            if (consoleDev.output.length !== lastOutLen) { lastOutLen = consoleDev.output.length; lastOutStep = guestNow; }

            /* Feed one character at a time, and only into an empty RXDB.  A keystroke with a
               `quiet` requirement is HELD until the console has been silent that long (see
               INPUT_RULES); the hold is released once and does not re-arm mid-string, because a
               reply the guest is echoing would otherwise stall its own remaining characters. */
            if (pending !== null && holdQuiet > 0) {
                if (guestNow - lastOutStep >= holdQuiet) holdQuiet = 0;
            } else if (pending !== null) {
                while (pendingIdx < pending.length && !(consoleDev.rxcs & CSR_DONE)) {
                    consoleDev.injectChar(pending.charCodeAt(pendingIdx++) & 0xFF);
                }
                /* Retire the hold with the string it belonged to -- see browser/vaxmachine.js's
                   pumpInput() for what leaving it set costs a human typing at the prompt. */
                if (pendingIdx >= pending.length) { pending = null; pendingIdx = 0; holdQuiet = 0; }
            } else if (fired.size < INPUT_RULES.length) {
                let text = "";
                for (let k = scanFrom; k < consoleDev.output.length; k++) {
                    text += String.fromCharCode(consoleDev.output[k]);
                }
                /* ANY not-yet-fired rule may match, not just the next one in the list.  The original
                   version advanced a single `ruleIdx` and so required the transcript to hit the rules
                   in the order written -- which was an accident of there being exactly two rules for
                   one volume, not a design requirement, and it silently wedges on any other volume:
                   MEASURED 2026-07-30, the OpenVMS V7.3 INSTALLATION disk never prints the booted
                   system's "PLEASE ENTER DATE AND TIME", so rule 2 never fired, rule 3 was never
                   evaluated, and the run sat at the installer's own date prompt until the cap.
                   One-shot semantics -- the property pcjsvax-459 records as necessary, because a
                   re-firing `>>>` rule injects `B DUA0` into VMS's date prompt and halts the system
                   -- are preserved by `fired`, which retires each rule permanently.  The EARLIEST
                   match in the stream wins, so a rule cannot jump the queue on text that appeared
                   before an earlier prompt. */
                let best = null, bestAt = -1;
                for (let r of INPUT_RULES) {
                    if (fired.has(r.name)) continue;
                    /* `atEnd` (see loginRules): match ONLY when the pattern is the tail of what the
                       guest has printed, i.e. it printed this and then stopped.  That is what tells
                       a bare `Username: ` prompt apart from the same ten characters appearing as a
                       padded field inside an OPCOM audit record, whose value follows on the line. */
                    let at = r.atEnd
                        ? (text.endsWith(r.match) ? text.length - r.match.length : -1)
                        : text.indexOf(r.match);
                    if (at >= 0 && (bestAt < 0 || at < bestAt)) { best = r; bestAt = at; }
                }
                /* REPEATABLE rules are consulted ONLY when no one-shot rule matched, so a specific
                   answer always beats the generic one no matter where each appears in the stream.
                   They exist for the OpenVMS installation procedure, which asks a long sequence of
                   questions that mostly want their own default -- one-shot rules cannot answer a
                   prompt shape that recurs.  A repeatable rule never retires, so its pattern must be
                   something that only appears when input is actually wanted; `scanFrom` still
                   advances past each match, so it cannot re-answer text it has already consumed. */
                if (!best) {
                    for (let r of REPEAT_RULES) {
                        let at = text.indexOf(r.match);
                        if (at >= 0 && (bestAt < 0 || at < bestAt)) { best = r; bestAt = at; }
                    }
                }
                if (best) {
                    scanFrom = scanFrom + bestAt + best.match.length;
                    marks.push({rule: best.name, atByte: consoleDev.output.length, atStep: steps});
                    sent.push(best);
                    fired.add(best.name);
                    pending = best.send; pendingIdx = 0;
                    holdQuiet = best.quiet | 0;
                    if (trace && opts.traceArm && best.name === opts.traceArm) {
                        trace.arm(best.name);
                        armStep = steps;
                    }
                }
            }
            if (armStep !== null && opts.traceStopAfter && steps - armStep >= opts.traceStopAfter) {
                stop = "TRACE WINDOW COMPLETE"; stopStep = steps; break;
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
                /* --console-file writes the stream so far on every heartbeat.  Without it the console
                   is only readable once the run ENDS, which makes discovering an interactive prompt a
                   one-guess-per-run loop -- and the OpenVMS installer is a sequence of prompts nobody
                   has transcribed yet.  Written whole rather than appended so the file is always a
                   valid prefix of the stream even if the run is killed mid-write.  Observation only:
                   it never touches the machine, and the run's own result still carries the bytes. */
                if (opts.consoleFile) {
                    try {
                        fs.writeFileSync(opts.consoleFile, Buffer.from(consoleDev.output));
                    } catch (e) { /* a probe must not die because its observation channel failed */ }
                }
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

    /* --dump-va: read guest VIRTUAL memory through the same mmu.readData() the CPU uses, in KERNEL
       mode (PSL is saved and restored around it) so an S0 address is readable whatever mode the run
       happened to stop in.  Taken AFTER the run, which is sound only for S0 space -- SBR/SLR do not
       move once VMS is up -- and is how a driver's own instruction bytes get disassembled. */
    let dump = null;
    if (opts.dumpVa) {
        let va = parseInt(opts.dumpVa, 16) | 0, savedPsl = cpu.psl;
        dump = {va: va >>> 0, bytes: [], error: null};
        try {
            cpu.psl = 0;
            for (let i = 0; i < opts.dumpLen; i++) dump.bytes.push(cpu.mmu.readData((va + i) | 0, 1, cpu.accR()) & 0xFF);
        } catch (e) { dump.error = String((e && e.message) || e); }
        cpu.psl = savedPsl;
    }

    return {
        dump,
        bytes: Uint8Array.from(consoleDev.output), steps, stop, stopStep, faults, faultEvents,
        marks, sentNames: sent.map((r) => r.name), trace,
        rulesLeft: INPUT_RULES.filter((r) => !fired.has(r.name)).map((r) => r.name),
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0, elapsedMs: Date.now() - t0, peakHeap,
        deviceNames: machine.devices.map((d) => d.name),
        memSize: machine.memSize, qreport,
        /* pcjsvax-af8 */
        idleCount: cpu.idleCount, idleSkipped: cpu.idleSkipped,
        idleSites: [...cpu.idleSites.entries()], idleSleptMs, idleWaits
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
        /* pcjsvax-af8.  Idling is ON here; `--no-idle` is what the before/after measurement used. */
        noIdle: hasArg("--no-idle"),
        idleMode: getArg("--idle-mode", "VMS"),
        consoleFile: getArg("--console-file", null),
        magicByte: parseInt(getArg("--magic", String(ROM_MAGIC_BYTE)), 10),
        /* pcjsvax-6c9's instrument.  See makeInputTrace(). */
        traceInput: hasArg("--trace-input"),
        traceArm: getArg("--trace-arm", "post-startup-wakeup"),
        traceLimit: parseInt(getArg("--trace-limit", "4000"), 10),
        tracePre: parseInt(getArg("--trace-pre", "40"), 10),
        traceStopAfter: parseInt(getArg("--trace-stop-after", "0"), 10),
        tracePc: parseInt(getArg("--trace-pc", "0"), 10),
        traceExcAfter: parseInt(getArg("--trace-exc-after", "0"), 10),
        /* pcjsvax-cff's login rules -- opt-in, see loginRules(). */
        login: getArg("--login", null),
        password: getArg("--password", null),
        dumpVa: getArg("--dump-va", null),
        dumpLen: parseInt(getArg("--dump-len", "256"), 10),
        snapPc: getArg("--snap-pc", null),
        snapReg: parseInt(getArg("--snap-reg", "1"), 10),
        snapLen: parseInt(getArg("--snap-len", "256"), 10)
    };
    opts.memBytes = (opts.memMB * 1024 * 1024) >>> 0;
    INPUT_RULES.push(...loginRules(opts.login, opts.password));
    /* `--wake-quiet` exists so the wake-up condition can be RE-DERIVED per volume rather than
       inherited.  pcjsvax-6c9 measured V7.1's; pcjsvax-cff had to establish 5.5's own, and the way
       to establish it is to run the same volume at two values and read the transcript, not to
       assume the number carries over.  Nothing is graded on it; it changes one rule's hold. */
    if (getArg("--wake-quiet", null) !== null) {
        let n = parseInt(getArg("--wake-quiet", "0"), 10);
        for (let r of INPUT_RULES) if (r.name === "post-startup-wakeup") r.quiet = n;
    }
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
    console.log(`  idle      ${opts.noIdle ? "OFF (--no-idle)" : "ON, mode " + opts.idleMode.toUpperCase()}` +
                ` -- pcjsvax-af8, modules/v2/idle.js`);

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
    /* pcjsvax-af8.  Both halves are reported, because either alone is misleading: instructions
       ELIDED is what the guest clock was advanced by, and milliseconds SLEPT is what the host got
       back.  A run with a large elision and no sleep is a run that saved nothing. */
    console.log(`  idle      ${r.idleCount} skips over ${r.idleSkipped} elided instructions ` +
        `(${(100 * r.idleSkipped / Math.max(1, r.steps + r.idleSkipped)).toFixed(1)}% of guest time), ` +
        `${r.idleWaits} waits totalling ${(r.idleSleptMs / 1000).toFixed(1)} s ` +
        `(${(100 * r.idleSleptMs / Math.max(1, r.elapsedMs)).toFixed(1)}% of wall clock)` +
        (r.idleSites.length ? `; sites ${r.idleSites.map(([k, v]) => `${k}=${v}`).join(" ")}` : ""));
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

    if (r.trace) {
        let t = r.trace;
        console.log(`\n----- pcjsvax-6c9 INPUT TRACE -----`);
        console.log(`  armed     ${t.armed ? `yes, at step ${t.armedAt}` : `NO -- rule "${opts.traceArm}" never fired`}`);
        console.log(`  counts    ${Object.entries(t.counts).map(([k, v]) => `${k}=${v}`).join(" ") || "(none)"}`);
        console.log(`  last ${t.pre.length} distinct console event(s) BEFORE arming:`);
        for (let e of t.pre) console.log(`    ${t.fmt(e)}`);
        console.log(`  ${t.events.length} distinct event(s) AFTER arming` +
            `${t.dropped ? ` (${t.dropped} DROPPED past --trace-limit ${t.limit})` : ""}:`);
        for (let e of t.events) console.log(`    ${t.fmt(e)}`);
        if (t.pcTrace) {
            console.log(`  PC trace, ${t.pcTrace.length} instruction(s) from the TTI dispatch:`);
            let line = [], prev = null, run = 0;
            for (let p of t.pcTrace) {
                if (p === prev) { run++; continue; }
                if (prev !== null) line.push(hex(prev) + (run > 1 ? `x${run}` : ""));
                prev = p; run = 1;
                if (line.length === 8) { console.log(`    ${line.join(" ")}`); line = []; }
            }
            if (prev !== null) line.push(hex(prev) + (run > 1 ? `x${run}` : ""));
            if (line.length) console.log(`    ${line.join(" ")}`);
        }
    }
    if (r.trace && r.trace.snapPc !== null) {
        let s = r.trace.snap;
        console.log(`\n----- SNAPSHOT at PC=${hex(r.trace.snapPc)} -----`);
        if (!s) console.log(`  NEVER REACHED -- the guest did not execute PC=${hex(r.trace.snapPc)} after arming`);
        else {
            console.log(`  step ${s.step}  PC=${hex(s.pc)} PSL=${hex(s.psl)}`);
            console.log(`  regs ${s.regs.map((v, i) => `R${i}=${hex(v)}`).join(" ")}`);
            if (s.error) console.log(`  memory read FAILED: ${s.error}`);
            for (let i = 0; i < s.bytes.length; i += 16) {
                console.log(`  ${hex((s.base + i) >>> 0)}  ` + s.bytes.slice(i, i + 16).map((b) => hex(b, 2)).join(" "));
            }
        }
    }
    if (r.dump) {
        console.log(`\n----- MEMORY DUMP at ${hex(r.dump.va)} -----`);
        if (r.dump.error) console.log(`  FAILED: ${r.dump.error}`);
        for (let i = 0; i < r.dump.bytes.length; i += 16) {
            console.log(`  ${hex((r.dump.va + i) >>> 0)}  ` +
                r.dump.bytes.slice(i, i + 16).map((b) => hex(b, 2)).join(" "));
        }
    }

    if (!opts.quiet) {
        console.log(`\n----- CONSOLE STREAM, ESCAPED -----`);
        console.log(esc(r.bytes));
    }
    console.log(`\n----- CONSOLE STREAM, READABLE -----`);
    console.log(readable(r.bytes));
    if (r.qreport.prov) r.qreport.prov.close();
}

main();
