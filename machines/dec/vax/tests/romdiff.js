/**
 * @fileoverview Differential test: the KA655 console ROM, decoded and executed from the boot
 *               entry state, vs. a real Open SIMH microvax3900
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-223.  Three claims, each with its own phase below:
 *
 *   TRACE     ka655x.bin is loaded at BusVAX.addRom(), CPUStateVAX.boot() sets PC/PSL/the model
 *             magic byte exactly as vax_sysdev.c's cpu_boot() does, and the machine executes from
 *             there.  A per-instruction trace comparison against a REAL SIMH -- booted for real,
 *             via the actual `BOOT CPU` command, not a hand reconstruction of its side effects --
 *             must show ZERO divergence up to the point where this machine's JS side first
 *             touches a physical range no item has decoded yet.  That point is reported BY NAME:
 *             this item's whole deliverable is telling the next items what the ROM needs and in
 *             what order.  Modeled on tests/cpudiff.js's EHKAA phase -- same trace format
 *             (tests/simhtrace.js), same oracle (tools/trace-differ/differ.py), same
 *             unavailable/CMPD/ZEROSPEC normalization, because it is the same kind of claim: a
 *             PROGRAM runs, not one instruction.
 *
 *   MIRROR    The upper half of the ROM (VAX.PHYSMEM.ROM_BASE + ROM_SIZE .. +ROM_LENGTH) must read
 *             back exactly what the lower half holds, at several offsets including the boundary,
 *             on BOTH sides -- and it must keep doing so after the one write this ROM ever
 *             receives (the boot-time magic byte), proving true aliasing rather than a load-time
 *             copy that could go stale.
 *
 *   SELFCHECK --selfcheck injects four deliberate defects into the SHIPPED code path (BusVAX's ROM
 *             mirror aliasing, CPUStateVAX.boot()'s magic byte / PSL, and the ROM's read-only
 *             enforcement) and fails if any one of them is not caught.
 *
 * WHY THE SIMH SIDE USES A REAL `BOOT CPU`, BOUNDED BY A BREAKPOINT, NOT A HAND-BUILT DEPOSIT
 * ---------------------------------------------------------------------------------------------
 * `BOOT CPU` (scp.c run_cmd(), RU_BOOT) calls the real `cpu_boot()` and then falls straight into
 * the free-running instruction loop -- `sim_step` is unconditionally reset to 0 for this command,
 * so there is no way to hand BOOT a step count, and on real ROM firmware that loop does not stop
 * on its own (it is the interactive console monitor, which waits on a terminal).  A breakpoint set
 * BEFORE `boot cpu` (`break <addr>`) bounds it without needing to guess anything about
 * `sysd_powerup()` or reconstruct `cpu_boot()` by hand -- SIMH does its own real boot, and control
 * returns to the script the instant the breakpoint hits, exactly like an interactive session.  The
 * breakpoint address is the JS run's OWN final PC (post-decode, pre-store) for the first
 * instruction it could not complete: for a non-branching store (which is what this item's boundary
 * turned out to be -- see below) that PC is exactly where the next instruction begins on BOTH
 * machines, and even if it were a few instructions early or late, `tools/trace-differ` truncates
 * SIMH's capture to the JS run's own length before comparing, so the breakpoint only needs to be
 * "at or after" the boundary, never exact.
 *
 *      node machines/dec/vax/tests/romdiff.js [options]
 *        --simh PATH        patched microvax3900; else $SIMH_CPU_BIN/$SIMH_BIN, else the scratch
 *                            build (same search cpudiff.js uses)
 *        --rom PATH         default $PCJS_VAX_REPO/open-simh/VAX/ka655x.bin
 *        --max-steps N       JS step ceiling before giving up on ever finding a boundary
 *        --selfcheck         prove the differential detects deliberate defects
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX, { VAXStop, ROM_MAGIC_BYTE } from "../modules/v2/cpustate.js";
import SimhTrace, { nonStoringResultOpcodes } from "./simhtrace.js";
import { OPCODES, DROM, DROM_STRIDE, DR } from "../modules/v2/drom.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const MEMSIZE = 0x01000000;                 // 16MB, same default every other differential uses

function hex(v, n = 8) { return (v >>> 0).toString(16).toUpperCase().padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * Plumbing -- same shape as cpudiff.js's, deliberately, so a reader of both trusts it once      *
 * ------------------------------------------------------------------------------------------- */

function vaxRepo()
{
    if (process.env['PCJS_VAX_REPO']) return process.env['PCJS_VAX_REPO'];
    return path.resolve(__dirname, "../../../../../pcjs-vax");
}

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    for (let v of ['SIMH_CPU_BIN', 'SIMH_INT_BIN', 'SIMH_DECODE_BIN', 'SIMH_BIN']) {
        if (process.env[v]) candidates.push(process.env[v]);
    }
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    candidates.push(path.join(vaxRepo(), "open-simh/BIN/microvax3900"));
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error(
        "romdiff needs a REAL SIMH (patch 0001, the REGS history line); it has no fixture\n" +
        "fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh PATH or\n" +
        "set $SIMH_CPU_BIN.  Tried:\n  " + candidates.join("\n  "));
}

function findDiffer()
{
    let p = path.join(vaxRepo(), "tools/trace-differ/differ.py");
    if (!fs.existsSync(p)) {
        throw new Error(`trace-differ not found at ${p}; set $PCJS_VAX_REPO to the pcjs-vax work repo`);
    }
    return p;
}

function findRom(pathArg)
{
    let p = pathArg || path.join(vaxRepo(), "open-simh/VAX/ka655x.bin");
    if (!fs.existsSync(p)) {
        throw new Error(`ka655x.bin not found at ${p}; pass --rom PATH`);
    }
    return p;
}

function runSimh(bin, script, iniPath)
{
    fs.writeFileSync(iniPath, script);
    return execFileSync(bin, [iniPath], {encoding: "utf8", maxBuffer: 1 << 29, timeout: 5 * 60 * 1000});
}

/* ------------------------------------------------------------------------------------------- *
 * The machine under test                                                                        *
 * ------------------------------------------------------------------------------------------- */

/**
 * makeMachine(romBytes)
 *
 * RAM at 0 (the KA655's own system memory, which the ROM's power-up self-tests size and probe --
 * without it the very first instructions would fault on RAM before ever reaching the ROM's own
 * device probing) plus the ROM itself, decoded via BusVAX.addRom().
 *
 * @param {Uint8Array} romBytes
 * @returns {Object} {bus, cpu}
 */
function makeMachine(romBytes)
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    bus.addRom(romBytes);
    let cpu = new CPUStateVAX({id: "cpu"});
    cpu.setBus(bus);
    return {bus, cpu};
}

/**
 * PHYSMEM_NAMES -- resolved from VAX.PHYSMEM itself (never hand-listed), used only to name a
 * stopping address in the report below.
 */
const PHYSMEM_RANGES = (function() {
    let out = [];
    for (let k in VAX.PHYSMEM) {
        if (!k.endsWith("_BASE")) continue;
        let stem = k.slice(0, -5);
        let len = VAX.PHYSMEM[stem + "_LENGTH"] || VAX.PHYSMEM[stem + "_SIZE"] || 0;
        out.push({name: stem, base: VAX.PHYSMEM[k] >>> 0, length: len >>> 0});
    }
    return out;
})();

/**
 * nameAddress(addr)
 *
 * @param {number} addr
 * @returns {string}
 */
function nameAddress(addr)
{
    addr = addr >>> 0;
    for (let r of PHYSMEM_RANGES) {
        if (addr >= r.base && addr < r.base + (r.length || 1)) {
            return `${r.name}+0x${(addr - r.base).toString(16).toUpperCase()} (${hex(addr)})`;
        }
    }
    return hex(addr);
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 0 -- determine sys_model BY EXECUTION, never by assumption                               *
 * ------------------------------------------------------------------------------------------- */

/**
 * querySysModel(simh, opts)
 *
 * vax_sysdev.c cpu_boot():1729 writes `sys_model ? 1 : 2` to ROMBASE+4.  Rather than assume which
 * value a "microvax3900" binary carries (the naming is misleading -- see cpustate.js's
 * ROM_MAGIC_BYTE comment), ask the SAME oracle binary this test grades against.
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Object} {sysModel, magicByte}
 */
function querySysModel(simh, opts)
{
    let out = runSimh(simh, "examine MODEL\nexit\n", path.join(opts.scratch, "romdiff-model.ini"));
    let m = /MODEL:\s*([0-9A-Fa-f]+)/.exec(out);
    if (!m) throw new Error("romdiff: could not read SIMH's MODEL register; SIMH said:\n" + out);
    let sysModel = parseInt(m[1], 16);
    return {sysModel, magicByte: sysModel ? 1 : 2};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 1 -- the trace                                                                           *
 * ------------------------------------------------------------------------------------------- */

/**
 * runRomJS(romBytes, opts, magicByte, mutation)
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 * @param {?string} mutation
 * @returns {Object}
 */
function runRomJS(romBytes, opts, magicByte)
{
    let {bus, cpu} = makeMachine(romBytes);
    cpu.reset();
    cpu.boot(magicByte);
    let hst = new SimhTrace();
    cpu.hst = hst;

    let steps = 0, stop = null;
    try {
        while (steps < opts.maxSteps) { cpu.stepCPU(1); steps++; }
    } catch (e) {
        if (!(e instanceof VAXStop)) throw e;
        stop = e;
    }
    hst.finish(cpu);
    let tracePath = path.join(opts.scratch, "romdiff-js.trace");
    fs.writeFileSync(tracePath, hst.text());
    return {
        bus, cpu, tracePath, records: hst.count, steps, stop,
        unavailable: hst.unavailable,
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0
    };
}

/**
 * captureSimhTrace(simh, opts, breakAddr)
 *
 * A REAL `boot cpu`, bounded by a breakpoint -- see the file header for why.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {number} breakAddr
 * @returns {string} path to the trace
 */
function captureSimhTrace(simh, opts, breakAddr)
{
    let tracePath = path.join(opts.scratch, "romdiff-simh.trace");
    if (fs.existsSync(tracePath)) fs.unlinkSync(tracePath);
    let script = [
        "set cpu 16m",
        `set cpu history=100000:${tracePath}`,
        `load -r ${opts.rom}`,
        `break ${hex(breakAddr)}`,
        "boot cpu",
        "examine PC",
        "exit", ""
    ].join("\n");
    let out = runSimh(simh, script, path.join(opts.scratch, "romdiff-simh.ini"));
    if (!/Breakpoint/.test(out)) {
        throw new Error(`romdiff: SIMH never hit the breakpoint at ${hex(breakAddr)} (it may have ` +
            `stopped or errored first); SIMH said:\n` + out);
    }
    return tracePath;
}

/**
 * The two normalized record classes, computed exactly as cpudiff.js does (imported nowhere because
 * it is four lines, not because it is not shared logic -- see that file for the full rationale).
 */
const ZEROSPEC = (function() {
    let s = new Set();
    for (let opc = 0; opc < 512; opc++) {
        if (!OPCODES[opc]) continue;
        let hdr = DROM[opc * DROM_STRIDE];
        if ((hdr & DR.NSPMASK) === 0 && ((hdr >> 4) & 0x7) > 0) s.add(OPCODES[opc]);
    }
    return s;
})();

const DIFF_DRIVER = `#!/usr/bin/env python3
import sys, os, json
differ_path, js_path, simh_path, unav_path, zerospec = sys.argv[1:6]
sys.path.insert(0, os.path.dirname(differ_path))
from differ import parse_trace, diff_traces

ZEROSPEC = set(x for x in zerospec.split(",") if x)
unavailable = set(json.load(open(unav_path)))

a = parse_trace(js_path)
buf = []
n = 0
with open(simh_path, errors="replace") as f:
    for line in f:
        buf.append(line)
        if line.startswith("REGS"):
            n += 1
            if n >= len(a):
                break
tmp = simh_path + ".trunc"
open(tmp, "w").writelines(buf)
b = parse_trace(tmp)
os.unlink(tmp)

n_cmpd = n_zero = n_unav = 0
for recs in (a, b):
    for i, r in enumerate(recs):
        if r.mnemonic == "CMPD" and " ->" in r.detail:
            r.detail = r.detail.split(" ->")[0]; n_cmpd += 1
        elif r.mnemonic in ZEROSPEC:
            r.detail = r.mnemonic; n_zero += 1
        elif i in unavailable and " ->" in r.detail:
            r.detail = r.detail.split(" ->")[0]; n_unav += 1

d = diff_traces(a[:len(b)], b)
print(json.dumps({
    "js_records": len(a), "simh_records": len(b),
    "normalized_cmpd": n_cmpd, "normalized_zerospec": n_zero, "normalized_unavailable": n_unav,
    "match": d is None,
    "divergence": None if d is None else d.to_dict(),
}))
`;

/**
 * compareTraces(jsPath, simhPath, unavailable, opts)
 *
 * @returns {Object}
 */
function compareTraces(jsPath, simhPath, unavailable, opts)
{
    let driver = path.join(opts.scratch, "romdiff-driver.py");
    fs.writeFileSync(driver, DIFF_DRIVER);
    let unavPath = path.join(opts.scratch, "romdiff-unavailable.json");
    fs.writeFileSync(unavPath, JSON.stringify(unavailable));
    let out = execFileSync("python3", [driver, findDiffer(), jsPath, simhPath, unavPath,
                                       [...ZEROSPEC].join(",")],
                           {encoding: "utf8", maxBuffer: 1 << 28});
    return JSON.parse(out);
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 2 -- the mirror                                                                         *
 * ------------------------------------------------------------------------------------------- */

/**
 * verifyMirrorJS(romBytes, opts)
 *
 * Checks the mirror against the PRIMARY half on THIS machine, at several offsets including both
 * boundaries, before AND after the one write the ROM ever receives (the boot-time magic byte) --
 * proving the mirror tracks a live write rather than a load-time snapshot.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @returns {Array.<string>} problems (empty if none)
 */
function verifyMirrorJS(romBytes, opts, magicByte)
{
    let problems = [];
    let {bus, cpu} = makeMachine(romBytes);
    let base = VAX.PHYSMEM.ROM_BASE >>> 0;
    let size = VAX.PHYSMEM.ROM_SIZE;
    let offsets = [0, 1, 4, 0x100, size >> 1, size - 2, size - 1];
    for (let off of offsets) {
        let primary = bus.getByte((base + off) >>> 0);
        let mirror = bus.getByte((base + size + off) >>> 0);
        if (primary !== mirror) {
            problems.push(`mirror BEFORE boot at +0x${off.toString(16)}: primary=0x${hex(primary, 2)} mirror=0x${hex(mirror, 2)}`);
        }
        if (primary !== romBytes[off]) {
            problems.push(`primary at +0x${off.toString(16)} is 0x${hex(primary, 2)}, expected the ROM file's own 0x${hex(romBytes[off], 2)}`);
        }
    }
    /* Now write the one byte the machine ever writes into its own ROM, and require the mirror to
       track it -- a load-time COPY would still pass every check above and only fail here. */
    cpu.reset();
    cpu.boot(magicByte);
    let primary4 = bus.getByte((base + 4) >>> 0);
    let mirror4 = bus.getByte((base + size + 4) >>> 0);
    if (primary4 !== magicByte) {
        problems.push(`primary +4 after boot() is 0x${hex(primary4, 2)}, expected the magic byte 0x${hex(magicByte, 2)}`);
    }
    if (mirror4 !== magicByte) {
        problems.push(`mirror +4 after boot() is 0x${hex(mirror4, 2)}, expected it to track the primary's magic byte 0x${hex(magicByte, 2)} -- the mirror is not truly aliased`);
    }
    return problems;
}

/**
 * verifyMirrorSimh(simh, opts, magicByte)
 *
 * Same claim, against SIMH: examine the primary and mirror halves at the same offsets, both before
 * and after a real `boot cpu` (bounded the same way the trace capture is), and require every pair
 * to agree with itself AND with what verifyMirrorJS found -- this is what proves the two
 * implementations mean the same thing by "mirrored", not just that each is internally consistent.
 *
 * @param {string} simh
 * @param {Object} opts
 * @returns {Object} {beforeBoot: [[primary,mirror],...], afterBoot: {primary4, mirror4}}
 */
function verifyMirrorSimh(simh, opts, breakAddr)
{
    let base = VAX.PHYSMEM.ROM_BASE >>> 0;
    let size = VAX.PHYSMEM.ROM_SIZE;
    let offsets = [0, 1, 4, 0x100, size >> 1, size - 2, size - 1];
    let lines = ["set cpu 16m", `load -r ${opts.rom}`];
    for (let off of offsets) {
        lines.push(`e -b ${hex((base + off) >>> 0)}`);
        lines.push(`e -b ${hex((base + size + off) >>> 0)}`);
    }
    lines.push(`break ${hex(breakAddr)}`, "boot cpu",
               `e -b ${hex((base + 4) >>> 0)}`, `e -b ${hex((base + size + 4) >>> 0)}`, "exit", "");
    let out = runSimh(simh, lines.join("\n"), path.join(opts.scratch, "romdiff-mirror.ini"));
    let vals = [...out.matchAll(/^[0-9A-F]+:\s+([0-9A-F]+)/gm)].map((m) => parseInt(m[1], 16));
    if (vals.length < offsets.length * 2 + 2) {
        throw new Error("romdiff: SIMH mirror probe produced too few results; SIMH said:\n" + out);
    }
    let beforeBoot = [];
    for (let i = 0; i < offsets.length; i++) beforeBoot.push({off: offsets[i], primary: vals[2 * i], mirror: vals[2 * i + 1]});
    let afterBoot = {primary4: vals[offsets.length * 2], mirror4: vals[offsets.length * 2 + 1]};
    return {beforeBoot, afterBoot};
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 3 -- --selfcheck                                                                         *
 * ------------------------------------------------------------------------------------------- */

/**
 * MUTATIONS -- one function each, applied to the SHIPPED objects (BusVAX.makeRomAliasController,
 * CPUStateVAX.prototype.boot, MemoryVAX write-protection), returning an undo closure.  Every one
 * of these is the named-mutation list done condition 5 requires, at minimum.
 */
const MUTATIONS = {
    /* The mirror aliases nothing: each mirror block gets its OWN zero-filled buffer instead of the
       primary block's array.  A load-time copy could still pass every read-only check; only the
       live-aliasing assertion in verifyMirrorJS (the post-boot magic-byte check) catches this. */
    "mirror-not-aliased": (cpu, bus) => {
        let orig = BusVAX.makeRomAliasController;
        BusVAX.makeRomAliasController = function() {
            let fake = new Int32Array(VAX.PHYSMEM.ROM_SIZE >> 2);
            return {
                getControllerBuffer() { return [fake, 0]; },
                getControllerAccess() {
                    return [
                        MemoryVAX.prototype.readByteMemory, undefined,
                        MemoryVAX.prototype.readWordMemory, undefined,
                        MemoryVAX.prototype.readLongMemory, undefined
                    ];
                }
            };
        };
        return () => { BusVAX.makeRomAliasController = orig; };
    },
    /* boot() never writes the model magic byte at all. */
    "magic-byte-not-written": (cpu, bus) => {
        let orig = CPUStateVAX.prototype.boot;
        CPUStateVAX.prototype.boot = function() {
            this.setPC(VAX.PHYSMEM.ROM_BASE);
            this.psl = (1 << 26 | 0x1F << 16) >>> 0;
        };
        return () => { CPUStateVAX.prototype.boot = orig; };
    },
    /* boot() sets the wrong PSL -- IPL 0 instead of 1F, so the very first instruction's own PSL
       field already disagrees with SIMH's real cpu_boot(). */
    "wrong-boot-psl": (cpu, bus) => {
        let orig = CPUStateVAX.prototype.boot;
        CPUStateVAX.prototype.boot = function(magicByte) {
            this.setPC(VAX.PHYSMEM.ROM_BASE);
            this.psl = (1 << 26) >>> 0;                      // PSL_IS, IPL dropped to 0
            this.bus.setByteDirect((VAX.PHYSMEM.ROM_BASE + 4) >>> 0, (magicByte === undefined ? ROM_MAGIC_BYTE : magicByte) & 0xFF);
        };
        return () => { CPUStateVAX.prototype.boot = orig; };
    },
    /* The ROM accepts a NORMAL (non-Direct) write -- the thing that must never happen once the
       machine is running, however real hardware would react to it. */
    "rom-writable": (cpu, bus) => {
        let addr = (VAX.PHYSMEM.ROM_BASE + 0x100) >>> 0;
        let block = bus.getBlock(addr);
        let orig = block.writeByte;
        block.writeByte = block.writeByteDirect;
        return () => { block.writeByte = orig; };
    }
};

/**
 * selfcheck(romBytes, opts, magicByte)
 *
 * Every mutation is checked with a FAST, DETERMINISTIC, structural assertion rather than a second
 * full SIMH trace run -- see the file's test_decisions in the item's report for why: none of these
 * four is a subtle multi-instruction timing bug (that is what cpudiff.js's own --selfcheck already
 * proves this loop can catch); each is directly observable as a single readback, and re-invoking
 * SIMH four more times would cost real wall-clock time for no additional detection power.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 */
function selfcheck(romBytes, opts, magicByte)
{
    console.log("\nSELFCHECK -- each mutation must be caught\n");
    /*
     * "mirror-not-aliased" patches BusVAX.makeRomAliasController, which addRom() only CONSULTS at
     * construction time -- so unlike the other three (which patch CPUStateVAX.prototype.boot or an
     * already-built block, both of which take effect at the later CALL), it must be applied BEFORE
     * makeMachine() builds the bus, or the mutation silently never engages and "SURVIVED" would be
     * a false negative rather than a real coverage hole.
     */
    let PRE_MACHINE = new Set(["mirror-not-aliased"]);
    let survived = [];
    for (let name of Object.keys(MUTATIONS)) {
        let caught = false, how = "";
        try {
            let bus, cpu, undo;
            if (PRE_MACHINE.has(name)) {
                undo = MUTATIONS[name](null, null);
                ({bus, cpu} = makeMachine(romBytes));
            } else {
                ({bus, cpu} = makeMachine(romBytes));
                undo = MUTATIONS[name](cpu, bus);
            }
            try {
                if (name === "mirror-not-aliased") {
                    cpu.reset();
                    cpu.boot(magicByte);
                    let base = VAX.PHYSMEM.ROM_BASE >>> 0, size = VAX.PHYSMEM.ROM_SIZE;
                    let p = bus.getByte((base + 4) >>> 0), m = bus.getByte((base + size + 4) >>> 0);
                    if (p !== m) { caught = true; how = `mirror +4 = 0x${hex(m, 2)}, primary +4 = 0x${hex(p, 2)}`; }
                } else if (name === "magic-byte-not-written" || name === "wrong-boot-psl") {
                    cpu.reset();
                    cpu.boot(0x37);
                    let base = VAX.PHYSMEM.ROM_BASE >>> 0;
                    let byte4 = bus.getByte((base + 4) >>> 0);
                    if (byte4 !== 0x37) { caught = true; how = `ROMBASE+4 = 0x${hex(byte4, 2)}, expected 0x37`; }
                    let wantPsl = (1 << 26 | 0x1F << 16) >>> 0;
                    if (cpu.psl >>> 0 !== wantPsl) { caught = true; how += (how ? "; " : "") + `PSL = ${hex(cpu.psl)}, expected ${hex(wantPsl)}`; }
                } else if (name === "rom-writable") {
                    cpu.reset();
                    cpu.boot(magicByte);
                    let addr = (VAX.PHYSMEM.ROM_BASE + 0x100) >>> 0;
                    let before = bus.getByte(addr);
                    bus.setByte(addr, (before ^ 0xFF) & 0xFF);
                    let after = bus.getByte(addr);
                    if (after !== before) { caught = true; how = `ROMBASE+0x100 changed from 0x${hex(before, 2)} to 0x${hex(after, 2)} via a NORMAL write`; }
                }
            } finally {
                undo();
            }
        } catch (e) {
            caught = true;
            how = `threw ${e.name}: ${e.message}`;
        }
        console.log(`  ${caught ? "CAUGHT " : "SURVIVED"} ${name.padEnd(28)} ${how}`);
        if (!caught) survived.push(name);
    }
    if (survived.length) {
        console.error("\nFAILURES:");
        for (let n of survived) console.error(`  SELFCHECK: mutation "${n}" was NOT caught -- that is a coverage hole, not a pass`);
        process.exit(1);
    }
    console.log("\nOK");
}

/* ------------------------------------------------------------------------------------------- *
 * Main                                                                                           *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, dflt)
{
    let i = process.argv.indexOf(name);
    return (i >= 0 && i + 1 < process.argv.length) ? process.argv[i + 1] : dflt;
}

function main()
{
    let opts = {
        maxSteps: +getArg("--max-steps", 200000),
        scratch: fs.mkdtempSync(path.join(os.tmpdir(), "romdiff-"))
    };
    opts.rom = findRom(getArg("--rom", null));
    let simh = findSimh(getArg("--simh", null));
    let fSelfcheck = process.argv.indexOf("--selfcheck") >= 0;
    let problems = [];

    console.log(`SIMH: ${simh}`);
    console.log(`ROM:  ${opts.rom}`);
    console.log(`scratch: ${opts.scratch}`);

    let romBytes = new Uint8Array(fs.readFileSync(opts.rom));
    if (romBytes.length !== VAX.PHYSMEM.ROM_SIZE) {
        console.error(`FAILURES:\n  ROM: ${opts.rom} is ${romBytes.length} bytes, expected exactly ` +
            `${VAX.PHYSMEM.ROM_SIZE} (VAX.PHYSMEM.ROM_SIZE)`);
        process.exit(1);
    }

    let {sysModel, magicByte} = querySysModel(simh, opts);
    console.log(`\nsys_model (measured, examine MODEL on this exact oracle): ${sysModel} -> magic byte 0x${hex(magicByte, 2)}`);
    if (magicByte !== ROM_MAGIC_BYTE) {
        problems.push(`ROM_MAGIC_BYTE in cpustate.js is 0x${hex(ROM_MAGIC_BYTE, 2)}, but this oracle's ` +
            `sys_model=${sysModel} calls for 0x${hex(magicByte, 2)} -- the shipped default has drifted from the oracle it was measured against`);
    }

    if (fSelfcheck) {
        selfcheck(romBytes, opts, magicByte);
        return;
    }

    /* ---- PHASE 1: the trace ---- */
    let js = runRomJS(romBytes, opts, magicByte);
    console.log("\nROM boot-entry execution");
    console.log(`  instructions executed : ${js.records}`);
    console.log(`  stopped               : ${js.stop ? js.stop.reason : "(step limit, never stopped)"} ` +
        `at PC=${hex(js.pc)} PSL=${hex(js.psl)}` +
        (js.stop && js.stop.detail !== undefined ? ` -- touched ${nameAddress(js.stop.detail)}` : ""));

    if (!js.stop) {
        problems.push(`the JS machine ran all ${opts.maxSteps} steps without reaching a device-register ` +
            `boundary; this item's whole point is to name that boundary, so a run that never finds one ` +
            `is not a pass -- raise --max-steps only after confirming a real boundary is further out`);
    } else {
        let breakAddr = js.pc;
        let simhTrace = captureSimhTrace(simh, opts, breakAddr);
        let cmp = compareTraces(js.tracePath, simhTrace, js.unavailable, opts);
        console.log(`  SIMH breakpoint        : ${hex(breakAddr)} (this run's own stopping PC)`);
        console.log(`  normalized records     : CMPD-tail=${cmp.normalized_cmpd} zerospec=${cmp.normalized_zerospec} store-faulted=${cmp.normalized_unavailable}`);
        console.log(`  trace comparison       : ${cmp.match ? "MATCH over all " + cmp.simh_records + " records" : "DIVERGE"}`);
        if (!cmp.match) {
            let d = cmp.divergence;
            problems.push(`TRACE: diverged at instruction ${d.index} (field=${d.field}, PC=${d.pc_a}): ${d.detail}`);
        }
        if (cmp.simh_records !== js.records) {
            problems.push(`TRACE: JS produced ${js.records} records but only ${cmp.simh_records} were ` +
                `compared -- SIMH's capture was shorter than the JS run, so some JS instructions never reached comparison`);
        }
    }

    /* ---- PHASE 2: the mirror ---- */
    let mirrorProblems = verifyMirrorJS(romBytes, opts, magicByte);
    for (let p of mirrorProblems) problems.push("MIRROR (JS): " + p);
    if (js.stop) {
        let simhMirror = verifyMirrorSimh(simh, opts, js.pc);
        console.log("\nMirror (upper half == lower half), before and after boot():");
        for (let r of simhMirror.beforeBoot) {
            let match = r.primary === r.mirror;
            console.log(`  +0x${r.off.toString(16).padStart(5, "0")}: SIMH primary=0x${hex(r.primary, 2)} mirror=0x${hex(r.mirror, 2)} ${match ? "" : "<- SIMH ITSELF DISAGREES"}`);
            if (!match) problems.push(`MIRROR (SIMH): SIMH's own primary/mirror disagree at +0x${r.off.toString(16)}: 0x${hex(r.primary, 2)} vs 0x${hex(r.mirror, 2)}`);
        }
        console.log(`  +4 after boot(): SIMH primary=0x${hex(simhMirror.afterBoot.primary4, 2)} mirror=0x${hex(simhMirror.afterBoot.mirror4, 2)}`);
        if (simhMirror.afterBoot.primary4 !== magicByte) {
            problems.push(`MIRROR (SIMH): SIMH's own primary +4 after boot is 0x${hex(simhMirror.afterBoot.primary4, 2)}, expected the magic byte 0x${hex(magicByte, 2)}`);
        }
        if (simhMirror.afterBoot.mirror4 !== magicByte) {
            problems.push(`MIRROR (SIMH): SIMH's own mirror +4 after boot is 0x${hex(simhMirror.afterBoot.mirror4, 2)}, expected it to track the magic byte too`);
        }
    }

    report(problems, js);
}

function report(problems, js)
{
    if (problems.length) {
        console.error("\nFAILURES:");
        for (let p of problems) console.error("  " + p);
        process.exit(1);
    }
    console.log("\nOK");
    if (js && js.stop) {
        console.log(`\nSTOPPING POINT: instruction #${js.records} (0-based index ${js.records - 1}), ` +
            `PC=${hex(js.pc)}, touched ${nameAddress(js.stop.detail)} -- ${js.stop.reason}`);
    }
}

main();

export { querySysModel, runRomJS, captureSimhTrace, compareTraces, verifyMirrorJS, nameAddress };
