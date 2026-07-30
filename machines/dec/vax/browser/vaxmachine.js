/**
 * @fileoverview pcjsvax-de8 -- the KA655 machine as a YIELDING, EVENT-DRIVEN driver, so it can run
 *               in a browser tab without hanging it
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS, AND WHAT IT DELIBERATELY IS NOT
 * ---------------------------------------------
 * Every driver of this machine so far -- tests/romdiff.js, tests/conoutdiff.js,
 * tests/vmsbootprobe.js -- owns a `while` loop that runs to completion and then prints.  That shape
 * cannot go in a tab: MEASURED under Node, this engine retires ~4M instructions/second and a boot
 * to the OpenVMS login prompt is 5e8-1e9 of them, so a synchronous loop freezes the page for two to
 * four minutes, paints nothing, and accepts no keystroke.
 *
 * So the loop is INVERTED.  `runSlice(ms)` executes for a bounded wall-clock budget and returns;
 * the caller decides when to call it again.  Nothing in here schedules itself, touches the DOM,
 * or knows what a Worker is -- which is what lets tests/vaxbrowserboot.js run the SAME driver under
 * Node and grade the boot it produces (HANDOFF.md standing rule 5: the browser path must be graded
 * by something, and a driver that only exists inside a Worker cannot be).
 *
 * WHY IT IS NOT A PCjs Component SUBCLASS
 * ---------------------------------------
 * pcjsvax-de8's scope says "PCjs Component wiring".  It is NOT done here, and that is a decision
 * rather than an omission.  The v2 VAX modules are plain ES modules with their own BusVAX, their own
 * CPUStateVAX and their own reset/boot sequence; they are not `Component.subclass()` devices and
 * they do not participate in computer.js's powerUp/powerDown lifecycle, `bindings`, `parmsDevice`
 * parsing or the `COMPILED`/`DEBUG` globals.  Subclassing Component means adapting all of them,
 * which is a rewrite of the very files the 34-check gate grades -- for zero change to what a user
 * sees.  A thin adapter (this file plus browser/vaxworker.js plus browser/vax.html's bindings) gets
 * the machine into a tab today and leaves the framework integration as a separate, gradeable step;
 * it is filed as `pcjsvax-f23`.  The cost of the choice is real and stated: `machines/dec/vax/` still
 * has no `.xml` machine definition, so the VAX cannot yet be embedded with `embedVAX()` the way
 * `machines/dec/pdp11/rk11/default.xml` is, and cpustate.js's restore() (pcjsvax-ad1) stays
 * unreachable because nothing calls the PCjs save/restore path.
 *
 * INPUT
 * -----
 * Two sources, and they share one queue so they cannot race:
 *   - `type(text)`, what the human at the terminal typed;
 *   - the AUTO-BOOT RULES, which are tests/vmsbootprobe.js's INPUT_RULES/REPEAT_RULES mechanism --
 *     one-shot rules matched against the console stream after the previous rule fired, earliest
 *     match wins, plus repeatable rules consulted only when no one-shot matched.
 *
 * Characters go in ONE AT A TIME and only into an empty RXDB (`rxcs & CSR_DONE` clear), because
 * RXDB is a single byte with no silo -- shovelling a line in overwrites characters the guest has
 * not read yet and the truncated command looks exactly like a console defect.
 *
 * `quiet` is carried over from tests/vmsbootprobe.js and it is NOT cosmetic: pcjsvax-6c9 measured
 * that a keystroke injected while the console driver is still writing is silently dropped (VMS's
 * console ISR takes the not-unsolicited arm), which is why the auto-boot rule that wakes LOGINOUT
 * holds its RETURN until the console has been silent for a while.  A HUMAN typing never hits this;
 * an automated demo hits it every time.
 */

import { VAXStop, ROM_MAGIC_BYTE } from "../modules/v2/cpustate.js";
import { IdleThrottle } from "../modules/v2/idle.js";
import { makeRomMachine } from "../tests/rommachine.js";
import RQVAX, { RQ_BASE, IOLN_RQ, RQDX3_CTYPE, U_RO } from "../modules/v2/rq.js";
import { USECS_PER_INSTR } from "../modules/v2/clk.js";

/** console.js's CSR_DONE.  console.js does not export it; it is the architected bit at the
    architected offset every VAX CSR-shaped device shares, and it is read ONLY to answer "has the
    guest taken the last character yet".  Same constant, same reason, as tests/vmsbootprobe.js. */
const CSR_DONE = 0x0080;

/** Instructions between wall-clock samples inside a slice.  Small enough that a slice overruns its
    budget by well under a frame, large enough that Date.now() is not in the hot path. */
const BATCH = 20000;

/** How often, inside a BATCH, the slice looks at `cpu.idleUsecs` (pcjsvax-af8).  A slice must
    RETURN once the guest has gone idle, or the caller never gets the chance to sleep instead of
    re-arming -- but a test on every instruction would be paid by the whole boot, which is not
    idle.  64 is small enough that the overshoot is a rounding error against a 10 ms clk tick and
    coarse enough to disappear into the call to stepCPU() next to it. */
const IDLE_POLL = 64;

/** The guest microseconds a slice must have elided before it hands control back to sleep on them.
    Four clk ticks: at this engine's measured ~3,900 instructions per millisecond that converts to
    about 10 ms of real sleep (see modules/v2/idle.js's IdleThrottle for why the conversion is a
    measured rate and not the guest time), which is one frame -- often enough that a keystroke or a
    device is never far away, rare enough that the wakeups are ~100/s rather than ~400/s. */
const IDLE_YIELD_USECS = 40000;

/**
 * THE AUTO-BOOT SCRIPT.  tests/vmsbootprobe.js's INPUT_RULES, unchanged in mechanism and in intent:
 * boot DUA0, answer the date prompt, and -- once startup has finished and gone quiet -- press
 * RETURN so LOGINOUT paints the login prompt.
 *
 * It is OPTIONAL and off unless the page asks for it.  A human who unticks "auto-boot" gets a bare
 * `>>>` and types `B DUA0` themselves, which is the whole point of having a terminal.
 */
const MONTHS = ["JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"];

/** VMS's `DD-MMM-YYYY HH:MM`, from the host clock.  A LITERAL date would make every browser boot
    claim to be the day this file was written, and OpenVMS records it in the audit log. */
export function vmsDateTime(d = new Date())
{
    const p2 = (n) => String(n).padStart(2, "0");
    return `${p2(d.getDate())}-${MONTHS[d.getMonth()]}-${d.getFullYear()} ${p2(d.getHours())}:${p2(d.getMinutes())}`;
}

export const AUTO_RULES = [
    {name: "console-prompt", match: ">>>", send: "B DUA0\r"},
    {name: "vms-date-prompt", match: "PLEASE ENTER DATE AND TIME", send: vmsDateTime() + "\r"},
    {name: "vms-install-date-prompt", match: "Please enter the date and time", send: vmsDateTime() + "\r"},
    /* MEASURED 2026-07-30 (pcjsvax-6c9): when startup finishes the console goes silent and stays
       silent -- LOGINOUT on OPA0 is waiting for a keystroke, exactly as a real console terminal
       does.  `quiet` holds the RETURN until the console has emitted nothing for this many
       instructions, because a character that arrives while the driver is still writing is dropped. */
    {name: "post-startup-wakeup", match: "Elapsed time:", send: "\r", quiet: 200000}
];

/** Consulted only when no one-shot rule matches.  `]: ` is the tail of DEC's own defaulted-prompt
    convention, so it appears when input is WANTED rather than in ordinary output. */
export const AUTO_REPEAT_RULES = [
    {name: "accept-bracketed-default", match: "]: ", send: "\r"}
];

/**
 * makeQbus(provider, report)
 *
 * The `opts.qbus` callback rommachine.js calls.  The RQVAX construction is rq.js's own, with the
 * same constructor arguments tests/mscpharness.js uses; the image provider is whatever the caller
 * built (browser/imageprovider.js in a tab, an `fs` one under Node).
 */
function makeQbus(provider, report)
{
    return function({cqbic}) {
        let rq = new RQVAX(cqbic, {cnum: 0, ctype: RQDX3_CTYPE});
        let capacBefore = rq.units[0].capac;
        rq.attach(0, provider, {});
        report.rq = rq;
        report.attach = {
            containerBytes: provider.byteLength,
            filesystemBytes: provider.filesystemBytes,
            providerWritable: !!provider.writable,
            capacBefore, capacAfter: rq.units[0].capac,
            flags: rq.units[0].flags,
            /* U_RO is rq.js's UNIT_RO.  Surfaced because a read-only DUA0 takes OpenVMS to
               `%SYSTEM-I-MOUNTVER, VAX1$DUA0: has been write-locked` and no further, and a user
               staring at that message needs to be told WHY, not left to guess. */
            readOnly: !!(rq.units[0].flags & U_RO) || typeof provider.write !== "function"
        };
        return {windows: [{base: RQ_BASE, length: IOLN_RQ, dev: rq}], tickDev: rq};
    };
}

export class VaxMachine
{
    /**
     * @param {Object} opts
     *   {Uint8Array} romBytes      the KA655 console ROM (ka655x.bin)
     *   {number}     [memBytes]    RAM size; defaults to rommachine.js's 16 MB
     *   {Object}     [disk]        an rq.js image provider for DUA0, or null for no disk at all
     *   {boolean}    [autoBoot]    run AUTO_RULES
     *   {boolean}    [idle]        guest-idle detection; DEFAULTS TRUE here (pcjsvax-af8)
     *   {string}     [idleMode]    which OS's idle loops to recognise; defaults to VMS
     */
    constructor(opts)
    {
        this.opts = opts;
        this.qreport = {};
        this.machine = makeRomMachine(opts.romBytes, false, false, {
            memSize: opts.memBytes,
            qbus: opts.disk ? makeQbus(opts.disk, this.qreport) : undefined
        });
        this.cpu = this.machine.cpu;
        this.consoleDev = this.machine.consoleDev;
        this.cpu.reset();
        this.cpu.boot(ROM_MAGIC_BYTE);

        this.steps = 0;
        this.stop = null;
        this.outIndex = 0;                                  /* how much of consoleDev.output the caller has seen */
        this.text = "";                                     /* unconsumed console text, for rule matching */
        this.scanned = 0;                                   /* how much of consoleDev.output has entered `text` */
        this.fired = new Set();
        this.pending = "";                                  /* characters still to be injected */
        this.holdQuiet = 0;
        this.lastOutLen = 0;
        this.lastOutStep = 0;
        this.marks = [];
        /* pcjsvax-af8: idling ON unless the caller says otherwise.  This is the tab's machine, and
           a tab that pins a core at a login prompt is the defect the item exists to close.  The
           34-check gate does NOT come through here -- it builds its machines from
           tests/rommachine.js directly, where cpustate.js's own default (off) stands. */
        this.cpu.idleEnable = opts.idle === undefined ? true : !!opts.idle;
        this.throttle = new IdleThrottle();
        if (opts.idleMode && !this.cpu.setIdleMode(opts.idleMode)) {
            throw new Error(`VaxMachine: unknown idle mode "${opts.idleMode}"`);
        }

        this.autoBoot = !!opts.autoBoot;
        this.rules = opts.rules || AUTO_RULES;
        this.repeatRules = opts.repeatRules || AUTO_REPEAT_RULES;
    }

    /** The attach report, for the UI: capacity, writability, whether autosize grew the unit. */
    get attachReport() { return this.qreport.attach || null; }

    /** The device roster rommachine.js derives from its own constructions. */
    get deviceNames() { return this.machine.devices.map((d) => d.name); }

    /** Console bytes the caller has not seen yet.  Consuming is what keeps the UI incremental. */
    drainOutput()
    {
        let out = this.consoleDev.output;
        if (this.outIndex >= out.length) return null;
        let slice = Uint8Array.from(out.slice(this.outIndex));
        this.outIndex = out.length;
        return slice;
    }

    /** Queue what a human typed.  It shares the injection queue with the auto-boot rules, so the
        two cannot interleave inside one string. */
    type(text) { this.pending += text; }

    /**
     * runSlice(msBudget)
     *
     * Execute for at most `msBudget` milliseconds of wall clock, then return.  THIS is the whole
     * yielding contract: the caller re-arms it from a timer/MessageChannel, so the event loop --
     * and therefore the UI, and therefore the keyboard -- runs between slices.
     *
     * *** IT ALSO RETURNS EARLY WHEN THE GUEST GOES IDLE (pcjsvax-af8). ***  `idleUsecs` is guest
     * time the CPU fast-forwarded through instead of executing; the moment a slice has banked a
     * clk tick's worth of it, the slice ENDS and reports it, so browser/vaxworker.js can wait on a
     * timer instead of re-arming its MessageChannel immediately.  Without the early return a slice
     * would happily spend its whole 16 ms budget skipping through 60+ ticks of guest time and the
     * tab would still pin a core -- the skip alone saves instructions, only the caller can save
     * CPU.  MEASURED: at a V5.5 login prompt this returns after ~1,200 retired instructions where
     * the un-idled slice retired ~65,000.
     *
     * @param {number} msBudget
     * @returns {{steps:number, stop:(string|null), idleUsecs:number, idleSleepMs:number}}
     */
    runSlice(msBudget)
    {
        if (this.stop) return {steps: 0, stop: this.stop, idleUsecs: 0, idleSleepMs: 0};
        let t0 = Date.now(), n0 = this.steps, cpu = this.cpu;
        try {
            do {
                for (let i = 0; i < BATCH; i += IDLE_POLL) {
                    for (let j = 0; j < IDLE_POLL; j++) cpu.stepCPU(1);
                    this.steps += IDLE_POLL;
                    if (cpu.idleUsecs >= IDLE_YIELD_USECS) break;
                }
            } while (cpu.idleUsecs < IDLE_YIELD_USECS && Date.now() - t0 < msBudget);
        } catch (e) {
            if (!(e instanceof VAXStop)) { this.stop = `ERROR ${(e && e.message) || e}`; throw e; }
            this.stop = `VAXStop ${e.reason} detail=${e.detail}`;
        }
        this.pumpInput();
        let steps = this.steps - n0, idleUsecs = cpu.takeIdleUsecs();
        /* The rate estimate is fed ONLY from slices that actually executed -- `Date.now() - t0`
           here is pure execution time, because the caller has not slept yet.  An idling slice
           retires a few hundred instructions in well under a millisecond and is dropped by
           noteBusy()'s own filter, so it cannot drag the estimate down and make the machine sleep
           longer than it should. */
        if (idleUsecs === 0) this.throttle.noteBusy(steps, Date.now() - t0);
        return {steps, stop: this.stop, idleUsecs,
                idleSleepMs: this.throttle.sleepMsFor(idleUsecs / USECS_PER_INSTR)};
    }

    /**
     * setIdle(on, mode)
     *
     * pcjsvax-af8.  `SET CPU IDLE=<os>` / `SET CPU NOIDLE`.  Returns false if `mode` is not a name
     * modules/v2/idle.js's OS_TAB carries, so a typo cannot silently mean "no idling".
     *
     * @param {boolean} on
     * @param {string} [mode]
     * @returns {boolean}
     */
    setIdle(on, mode)
    {
        if (mode && !this.cpu.setIdleMode(mode)) return false;
        this.cpu.idleEnable = !!on;
        return true;
    }

    /**
     * pumpInput()
     *
     * Called once per slice, never per instruction: injectChar() plus a substring search on every
     * instruction would dominate the run, and the guest's own polling loop is thousands of
     * instructions wide.
     */
    pumpInput()
    {
        let dev = this.consoleDev;

        /*
         * Console quiescence, sampled once per slice -- the clock a `quiet` rule waits on.
         *
         * *** THE CLOCK IS GUEST TIME, NOT INSTRUCTIONS RETIRED (pcjsvax-af8). ***  It used to be
         * `this.steps`, and that was equivalent right up until the CPU learned to fast-forward
         * through an idle loop instead of executing it.  A quiet threshold of 200,000 instructions
         * is 200,000 microseconds of guest time (clk.js's USECS_PER_INSTR = 1) and that is what the
         * rule means; measured against RETIRED instructions on an idling machine it becomes
         * 200,000 / 17,000 per second = ELEVEN SECONDS, and the RETURN that wakes LOGINOUT arrives
         * that much late -- which is pcjsvax-6c9's symptom exactly, reintroduced by the fix for a
         * different item.  `cpu.nTotalCycles` counts retired AND elided instructions, so it is the
         * same number it always was whether or not the machine idled through it.
         */
        let now = this.cpu.nTotalCycles;
        if (dev.output.length !== this.lastOutLen) { this.lastOutLen = dev.output.length; this.lastOutStep = now; }

        if (this.pending.length && this.holdQuiet > 0) {
            if (now - this.lastOutStep >= this.holdQuiet) this.holdQuiet = 0;
            else return;
        }
        while (this.pending.length && !(dev.rxcs & CSR_DONE)) {
            dev.injectChar(this.pending.charCodeAt(0) & 0xFF);
            this.pending = this.pending.slice(1);
        }
        /* *** RETIRE THE HOLD WITH THE STRING IT BELONGED TO. ***  `holdQuiet` is a property of the
           RULE that queued these characters, not a mode of the machine, and nothing used to clear
           it: once `post-startup-wakeup` (quiet: 200000) had fired, every LATER keystroke -- a
           human's, at the login prompt -- inherited its hold and waited out a quiescence it was
           never meant to wait for.  Invisible while the queue only ever carried auto-boot rules;
           a keystroke that answers seconds late the moment a human types one. */
        if (!this.pending.length) this.holdQuiet = 0;
        if (this.pending.length || !this.autoBoot) return;
        /* Once every one-shot rule has fired the script is DONE and nothing types again -- the same
           guard tests/vmsbootprobe.js uses, and here it also means the machine stops typing the
           moment it has reached the login prompt, so a human's session is never interfered with. */
        if (this.fired.size >= this.rules.length) return;

        /* Grow the matching text from the console stream.  Text a rule has already consumed is
           DROPPED rather than skipped over, so a repeatable rule cannot re-answer a prompt it
           already answered and the buffer does not grow with the transcript. */
        for (let k = this.scanned; k < dev.output.length; k++) this.text += String.fromCharCode(dev.output[k]);
        this.scanned = dev.output.length;
        let text = this.text;

        /* ANY not-yet-fired rule may match, not just the next one in the list, and the EARLIEST
           match in the stream wins -- so a rule cannot jump the queue on text that appeared before
           an earlier prompt.  One-shot semantics are preserved by `fired`, which retires each rule
           permanently: pcjsvax-459 records that a re-firing `>>>` rule injects `B DUA0` into VMS's
           date prompt and halts the system. */
        let best = null, bestAt = -1;
        for (let r of this.rules) {
            if (this.fired.has(r.name)) continue;
            let at = text.indexOf(r.match);
            if (at >= 0 && (bestAt < 0 || at < bestAt)) { best = r; bestAt = at; }
        }
        if (!best) {
            for (let r of this.repeatRules) {
                let at = text.indexOf(r.match);
                if (at >= 0 && (bestAt < 0 || at < bestAt)) { best = r; bestAt = at; }
            }
        }
        if (!best) return;
        this.text = this.text.slice(bestAt + best.match.length);
        this.fired.add(best.name);
        this.marks.push({rule: best.name, atByte: dev.output.length, atStep: this.steps});
        this.pending += best.send;
        this.holdQuiet = best.quiet | 0;
    }
}

export default VaxMachine;
