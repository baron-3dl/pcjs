#!/usr/bin/env node
/**
 * @fileoverview Target Adapter: runs the JS VAX to implement pcjs-vax's EHKAA gate protocol
 * @author Chris Baron
 * @copyright © 2026
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-796: the Target Adapter that lets `tools/ehkaa-gate/gate.py` (in the pcjs-vax work repo)
 * grade THIS machine -- machines/dec/vax/modules/v2/cpustate.js -- instead of only real SIMH.  It
 * implements docs/reference/target-adapter-protocol.md exactly, modelled on that repo's reference
 * adapter, tools/ehkaa-gate/adapters/simh_adapter.py, which does the equivalent thing for SIMH.
 *
 * It does NOT reimplement the CPU loop.  It builds the same bare machine (bus + CPUStateVAX) that
 * tests/cpudiff.js's makeMachine() builds -- the one already proven, by a per-instruction trace
 * diff against real SIMH, to run EHKAA to its documented PASS halt with zero divergence over all
 * 335,444 instructions -- loads the image, sets PC, and calls CPUStateVAX.stepCPU() in a loop
 * until it stops.
 *
 * INSTRUCTION COUNTING
 * ---------------------
 * `stepCPU()` does not RETURN when the HALT instruction runs: HALT's body (exc.js) throws VAXStop
 * from INSIDE executeOne(), and stepCPU() does not catch VAXStop (see its own doc comment -- HALT
 * is "return to SCP", not a fault).  A loop shaped like `stepCPU(1); count++;` therefore
 * undercounts the halting instruction by exactly one, because `count++` never runs on the call
 * that throws.  This adapter instead wraps `cpu.executeOne` -- the callback stepInstruction()
 * calls exactly once per instruction fetch (exc.js's stepInstruction(), the `execute(opc, ...)`
 * call after the trap/interrupt/trace-trap block settles) -- so every instruction the loop
 * actually dispatches is counted, including the one that halts it.  That is the same event SIMH's
 * `SET CPU HISTORY` counts (tools/ehkaa-gate's ground truth) and what tests/cpudiff.js's own
 * record count (`hst.count`, via the same fnExecute chokepoint) already proved zero-divergent for
 * this exact program: 335,444.
 *
 * NOT FAKEABLE
 * ------------
 * `--selfcheck` (see below) proves, by REAL execution of `tools/ehkaa-gate/gate.py` against this
 * adapter, that every one of the 17 fields the gate grades (pc, instruction_count, R0..R14) is
 * actually load-bearing: an `EHKAA_ADAPTER_MUTATE` back door (read from the environment, never set
 * by the gate's own invocation) flips one bit of one reported field after a REAL run, and the
 * selfcheck asserts gate.py exits 1 (graded FAIL) for every one of the 17, and exits 0 (PASS) for
 * the unmutated baseline.  This is the shipped code path, not a separate throwaway script: the
 * mutation hook lives in `runTarget()` below, the same function every real gate invocation calls.
 *
 * CLI use
 * -------
 *     ehkaa_adapter.js --image PATH --load-addr 200 [--timeout SECONDS] [--max-instructions N]
 *     ehkaa_adapter.js --selfcheck [--image PATH] [--load-addr 200]
 *
 * Emits exactly one JSON line on stdout per the protocol; exit 0 if halted, 1 otherwise.
 */
import fs from "fs";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import { VAX } from "../modules/v2/defines.js";
import { VAXStop } from "../modules/v2/exc.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const MEMSIZE = 0x01000000;                 // 16MB, the SIMH microvax3900 default (tests/cpudiff.js makeMachine())
const DEFAULT_MAX_INSTRUCTIONS = 2000000;   // >> EHKAA's 335,444; a runaway-loop safety valve, not a step limit
                                             // the protocol defines -- the gate's own subprocess timeout is the
                                             // authoritative bound, this just fails cleanly instead of hanging
                                             // when the adapter is run standalone.
const DEFAULT_TIMEOUT_S = 90;

function hex(v) { return (v >>> 0).toString(16).toUpperCase().padStart(8, "0"); }

/**
 * vaxRepo() -- resolve the pcjs-vax work repo, same convention tests/cpudiff.js uses
 * (`$PCJS_VAX_REPO`, else `../../../../../pcjs-vax` relative to this file). Needed only by
 * --selfcheck, to find tools/ehkaa-gate/gate.py and the default EHKAA image; the protocol run
 * itself never touches the work repo.
 */
function vaxRepo()
{
    if (process.env["PCJS_VAX_REPO"]) return process.env["PCJS_VAX_REPO"];
    return path.resolve(__dirname, "../../../../../pcjs-vax");
}

/**
 * makeMachine()
 *
 * The bare machine: a 16MB physical bus and CPUStateVAX, nothing else -- exactly what
 * tests/cpudiff.js's makeMachine() builds, because that is the construction already proven (by a
 * 335,444-instruction, zero-divergence trace diff against real SIMH) to run EHKAA correctly. Not
 * re-derived here: same bus width, same memory size, same reset() call.
 *
 * @returns {CPUStateVAX}
 */
function makeMachine()
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    cpu.reset();
    return cpu;
}

/**
 * applyMutation(result, mutate)
 *
 * Flips one bit of one reported field. Used ONLY by --selfcheck, via the EHKAA_ADAPTER_MUTATE
 * environment variable -- never set by the gate's own invocation of this adapter, so a real grading
 * run is never affected. `mutate` is one of "pc", "count", or "r<N>" for 0 <= N < registers.length.
 *
 * @param {Object} result
 * @param {string} mutate
 */
function applyMutation(result, mutate)
{
    if (mutate === "pc") {
        result.pc = hex((parseInt(result.pc, 16) ^ 1) >>> 0);
        return;
    }
    if (mutate === "count") {
        result.instruction_count = result.instruction_count - 1;
        return;
    }
    let m = /^r(\d+)$/.exec(mutate);
    if (m) {
        let i = +m[1];
        if (i < 0 || i >= result.registers.length) {
            throw new Error(`EHKAA_ADAPTER_MUTATE: register index out of range: ${mutate}`);
        }
        result.registers[i] = hex((parseInt(result.registers[i], 16) ^ 1) >>> 0);
        return;
    }
    throw new Error(`EHKAA_ADAPTER_MUTATE: unrecognized target: ${mutate}`);
}

/**
 * runTarget(imagePath, loadAddr, opts)
 *
 * Loads `imagePath` at physical 0 -- SIMH's `load FILE` with no `-o` switch, byte for byte what
 * simh_adapter.py and tests/cpudiff.js both do -- sets PC to `loadAddr`, and drives the real CPU
 * loop (`cpu.stepCPU()`, never reimplemented) until it halts, errors, or a safety valve trips.
 *
 * @param {string} imagePath
 * @param {number} loadAddr
 * @param {Object} opts {maxInstructions, timeoutS}
 * @returns {Object} the protocol's success or failure report
 */
function runTarget(imagePath, loadAddr, opts)
{
    if (!fs.existsSync(imagePath)) {
        return {halted: false, error: `image not found: ${imagePath}`};
    }

    let cpu;
    try {
        cpu = makeMachine();
        cpu.loadImage(new Uint8Array(fs.readFileSync(imagePath)));
        cpu.setPC(loadAddr);
    } catch (e) {
        return {halted: false, error: `failed to load image: ${e.message}`};
    }

    /*
     * Count every instruction the loop actually dispatches -- see the file header for why this is
     * NOT "count how many stepCPU() calls returned".
     */
    let count = 0;
    let origExec = cpu.executeOne.bind(cpu);
    cpu.executeOne = function(opc, decoder, c) {
        count++;
        return origExec(opc, decoder, c);
    };

    let start = Date.now();
    let timeoutMs = opts.timeoutS * 1000;
    try {
        for (;;) {
            if (count >= opts.maxInstructions) {
                return {
                    halted: false,
                    error: `exceeded --max-instructions (${opts.maxInstructions}) without halting; ` +
                           `pc=${hex(cpu.regs[15])}`,
                };
            }
            if ((Date.now() - start) > timeoutMs) {
                return {
                    halted: false,
                    error: `internal timeout after ${opts.timeoutS}s without halting; pc=${hex(cpu.regs[15])}`,
                };
            }
            cpu.stepCPU(1);
        }
    } catch (e) {
        if (!(e instanceof VAXStop)) {
            return {halted: false, error: `unexpected ${e.name || "Error"}: ${e.message}`};
        }
        if (e.reason !== VAXStop.REASON.HALT) {
            return {halted: false, error: `simulator stop was not HALT: ${e.reason} (detail=${e.detail})`};
        }
    }

    let registers = [];
    for (let i = 0; i < 15; i++) registers.push(hex(cpu.regs[i]));

    let result = {
        halted: true,
        pc: hex(cpu.regs[15]),
        instruction_count: count,
        registers,
    };

    let mutate = process.env["EHKAA_ADAPTER_MUTATE"];
    if (mutate) applyMutation(result, mutate);

    return result;
}

/* --------------------------------------------------------------------------------------------- *
 * --selfcheck                                                                                      *
 * --------------------------------------------------------------------------------------------- */

/**
 * mutationScope(imagePath, loadAddr, opts)
 *
 * The full list of gate-graded fields, derived from what a REAL run actually reports -- not hand-
 * enumerated. `["pc", "count", "r0", ..., "r<registers.length-1>"]`. If the protocol's register
 * count ever changes, this list changes with it instead of silently under-covering.
 *
 * @param {string} imagePath
 * @param {number} loadAddr
 * @param {Object} opts
 * @returns {{baseline: Object, targets: Array.<string>}}
 */
function mutationScope(imagePath, loadAddr, opts)
{
    let baseline = runTarget(imagePath, loadAddr, opts);
    if (!baseline.halted) {
        throw new Error(`--selfcheck: baseline run did not halt: ${baseline.error}`);
    }
    let targets = ["pc", "count"];
    for (let i = 0; i < baseline.registers.length; i++) targets.push(`r${i}`);
    return {baseline, targets};
}

/**
 * runGate(gatePy, adapterPath, imagePath, loadAddr, env)
 *
 * Invokes the REAL gate.py as a subprocess against this adapter (never against a mock), exactly
 * as the milestone's done condition does. Returns {passed, exitCode, stdout}.
 *
 * @param {string} gatePy
 * @param {string} adapterPath
 * @param {string} imagePath
 * @param {string} loadAddr
 * @param {Object} env
 * @returns {Object}
 */
function runGate(gatePy, adapterPath, imagePath, loadAddr, env)
{
    let args = [gatePy, "--adapter", adapterPath, "--image", imagePath, "--load-addr", loadAddr, "--json"];
    try {
        let out = execFileSync("python3", args, {encoding: "utf8", env, timeout: 180000});
        let report = JSON.parse(out.trim().split("\n").pop());
        return {exitCode: 0, passed: report.passed === true, stdout: out};
    } catch (e) {
        let out = (e.stdout || "").toString();
        let report = null;
        try { report = JSON.parse(out.trim().split("\n").pop()); } catch (_e) { /* adapter/usage error */ }
        return {exitCode: e.status, passed: report ? report.passed === true : false, stdout: out};
    }
}

/**
 * selfCheck(imagePath, loadAddr, opts)
 *
 * Proves the adapter is not fakeable by REAL execution: runs `gate.py` against this adapter once
 * unmutated (must PASS, exit 0) and once per graded field with that field corrupted via
 * EHKAA_ADAPTER_MUTATE (each must FAIL -- graded wrong, exit 1, not an adapter/usage error). Every
 * case that does not land where expected is reported by name; nothing is silently skipped.
 *
 * @param {string} imagePath
 * @param {string} loadAddrHex
 * @param {Object} opts
 * @returns {number} process exit code: 0 if every case landed as expected, 1 otherwise
 */
function selfCheck(imagePath, loadAddrHex, opts)
{
    let repo = vaxRepo();
    let gatePy = path.join(repo, "tools/ehkaa-gate/gate.py");
    if (!fs.existsSync(gatePy)) {
        console.error(`--selfcheck: gate.py not found at ${gatePy}; set $PCJS_VAX_REPO`);
        return 2;
    }
    let adapterPath = fileURLToPath(import.meta.url);

    let scope;
    try {
        scope = mutationScope(imagePath, parseInt(loadAddrHex, 16), opts);
    } catch (e) {
        console.error(`--selfcheck: ${e.message}`);
        return 2;
    }

    console.error(`--selfcheck: baseline halted at pc=${scope.baseline.pc}, ` +
                   `${scope.baseline.instruction_count} instructions`);
    console.error(`--selfcheck: mutation scope (${scope.targets.length} fields): ${scope.targets.join(", ")}`);

    let failures = [];

    let base = runGate(gatePy, adapterPath, imagePath, loadAddrHex, process.env);
    if (!base.passed) {
        failures.push(`baseline (unmutated): expected gate PASS, got exitCode=${base.exitCode} ` +
                       `passed=${base.passed}\n${base.stdout}`);
    }
    console.error(`--selfcheck: baseline ${base.passed ? "PASS (as expected)" : "FAIL (WRONG -- see below)"}`);

    for (let target of scope.targets) {
        let env = Object.assign({}, process.env, {EHKAA_ADAPTER_MUTATE: target});
        let r = runGate(gatePy, adapterPath, imagePath, loadAddrHex, env);
        let caught = !r.passed && r.exitCode === 1;
        if (!caught) {
            failures.push(`mutation ${target}: expected gate FAIL (exit 1), got exitCode=${r.exitCode} ` +
                           `passed=${r.passed}\n${r.stdout}`);
        }
        console.error(`--selfcheck: mutate ${target.padEnd(6)} -> ${caught ? "CAUGHT (gate FAILED, as required)" : "NOT CAUGHT -- SURVIVED"}`);
    }

    if (failures.length) {
        console.error(`\n--selfcheck: ${failures.length}/${scope.targets.length + 1} case(s) did not land as expected:`);
        for (let f of failures) console.error(`  - ${f}`);
        return 1;
    }
    console.error(`\n--selfcheck: PASS -- baseline gate-PASSes, and all ${scope.targets.length} ` +
                   `single-field mutations gate-FAIL. The gate is not fakeable by this adapter reporting ` +
                   `a wrong pc, instruction_count, or any of R0..R14.`);
    return 0;
}

/* --------------------------------------------------------------------------------------------- *
 * CLI                                                                                              *
 * --------------------------------------------------------------------------------------------- */

function parseArgs(argv)
{
    let args = {
        image: null,
        loadAddr: null,
        timeoutS: DEFAULT_TIMEOUT_S,
        maxInstructions: DEFAULT_MAX_INSTRUCTIONS,
        selfcheck: false,
    };
    for (let i = 0; i < argv.length; i++) {
        switch (argv[i]) {
        case "--image": args.image = argv[++i]; break;
        case "--load-addr": args.loadAddr = argv[++i]; break;
        case "--timeout": args.timeoutS = +argv[++i]; break;
        case "--max-instructions": args.maxInstructions = +argv[++i]; break;
        case "--selfcheck": args.selfcheck = true; break;
        default:
            throw new Error(`unrecognized argument: ${argv[i]}`);
        }
    }
    return args;
}

function main(argv)
{
    let args;
    try {
        args = parseArgs(argv);
    } catch (e) {
        console.error(`usage error: ${e.message}`);
        return 2;
    }

    let repo = vaxRepo();
    let image = args.image || path.join(repo, "open-simh/VAX/tests/ehkaa.exe");
    let loadAddrHex = args.loadAddr || "200";

    if (args.selfcheck) {
        return selfCheck(image, loadAddrHex, {maxInstructions: args.maxInstructions, timeoutS: args.timeoutS});
    }

    if (!args.image || !args.loadAddr) {
        console.error("usage: ehkaa_adapter.js --image PATH --load-addr HEX [--timeout S] [--max-instructions N]");
        return 2;
    }

    let result = runTarget(image, parseInt(args.loadAddr, 16), {
        maxInstructions: args.maxInstructions,
        timeoutS: args.timeoutS,
    });
    console.log(JSON.stringify(result));
    return result.halted ? 0 : 1;
}

process.exitCode = main(process.argv.slice(2));
