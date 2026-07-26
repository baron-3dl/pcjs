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
 * pcjsvax-223 (TRACE, MIRROR, SELFCHECK) and pcjsvax-320 (the SSC-BASE-RANDOM addition to TRACE
 * and SELFCHECK, and the boundary-accounting fix TRACE needed once its own advance exposed it).
 * Four claims, each with its own phase below:
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
 *             PROGRAM runs, not one instruction.  pcjsvax-320's own boundary-advance (past the SSC
 *             base register's store) exposed a real bug in how many trace records were comparable
 *             when the boundary lands on the INSTRUCTION AFTER one that now completes -- see
 *             BoundaryAccounting.compute()'s doc comment.
 *
 *   SSC-BASE- pcjsvax-320: the TRACE phase's boot run only ever stores ONE value into the SSC base
 *   RANDOM    register (0x20140000, which already satisfies its own mask), so verifySscBaseRandom()
 *             drives many MORE values -- boundary-chosen and random -- through a real instruction
 *             round trip on the live oracle, proving the SSCBASE_RW/SSCBASE_MBO mask itself, not
 *             just whether the address is backed.  Enforces its own coverage floor.
 *
 *   MIRROR    The upper half of the ROM (VAX.PHYSMEM.ROM_BASE + ROM_SIZE .. +ROM_LENGTH) must read
 *             back exactly what the lower half holds, at several offsets including the boundary,
 *             on BOTH sides -- and it must keep tracking a DISTINCT sentinel byte written directly
 *             into the primary after construction, proving true (live) aliasing rather than a
 *             load-time copy that could go stale.  NOT the boot-time magic byte: ka655x.bin
 *             already ships 0x02 at offset 4 and the measured magic byte is also 2, so re-writing
 *             it proves nothing -- a snapshot taken at load time would show the identical value.
 *             A prior version of this check used the magic byte and was vacuous for exactly that
 *             reason (caught by an adversarial veracity review); see verifyMirrorJS()'s header.
 *
 *   SELFCHECK --selfcheck injects eight deliberate defects into the SHIPPED code path (two distinct
 *             ROM-mirror failure modes -- reads garbage, and reads a stale load-time snapshot --
 *             CPUStateVAX.boot()'s magic byte / PSL, the ROM's read-only enforcement, the SSC base
 *             register's decode and its mask, and BoundaryAccounting's off-by-one) and fails if any
 *             one of them is not caught.
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
 *        --ssc-base-cases N  randomized SSC base register mask cases (default 40, floor 8)
 *        --seed S            PRNG seed for --ssc-base-cases, printed on failure so it reproduces
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
import SSCVAX from "../modules/v2/ssc.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const MEMSIZE = 0x01000000;                 // 16MB, same default every other differential uses

/* Enforced floor for --ssc-base-cases (standing rule: coverage assertions FAIL the run and do not
   scale down with case count).  4 fixed boundary values (0, all-ones, exactly SSCBASE_RW, exactly
   SSCBASE_MBO) plus at least 4 genuinely random ones -- see verifySscBaseRandom(). */
const SSC_BASE_CASES_FLOOR = 8;

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
 * device probing) plus the ROM itself, decoded via BusVAX.addRom(), plus the SSC base register,
 * decoded via BusVAX.addSsc() -- pcjsvax-320, the item this file's own boundary report pointed at.
 *
 * @param {Uint8Array} romBytes
 * @returns {Object} {bus, cpu}
 */
function makeMachine(romBytes)
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
    bus.addRom(romBytes);
    bus.addSsc(new SSCVAX());
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

/**
 * BoundaryAccounting.compute(records, finalizedPartial)
 *
 * pcjsvax-320: how many of `records` (hst.count after runRomJS()'s hst.finish(cpu) call) are
 * GENUINELY comparable against SIMH, and what is the boundary instruction's own 1-based ordinal.
 * A plain object property (not a bare top-level function) so selfcheck() below can patch it the
 * same way it patches every other shipped entry point -- see MUTATIONS' "boundary-off-by-one" for
 * why this needed its own mutation rather than trusting the live romdiff run alone.
 *
 * `finalizedPartial` is measured directly by runRomJS() (did hst.finish(cpu) change hst.count?),
 * not assumed: see runRomJS()'s doc comment for the two shapes a boundary can take and why
 * conflating them was a real bug this item's own boundary-advance exposed.
 *
 * @param {number} records
 * @param {boolean} finalizedPartial
 * @returns {{comparableRecords: number, instrNum: number}}
 */
const BoundaryAccounting = {
    compute(records, finalizedPartial) {
        let comparableRecords = finalizedPartial ? records - 1 : records;
        return {comparableRecords, instrNum: comparableRecords + 1};
    }
};

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
 * MEASURED CORRECTION (veracity re-dispatch, pcjsvax-446): pcjsvax-446 made onBusFault() dispatch
 * a REAL machine check instead of stopping the simulator -- correct, and the whole point of that
 * item.  But at this exact point in boot, SCBB and IS are both still 0 (cpu.reset()'s default,
 * matching SIMH's own cpu_reset() -- a real ROM's first instructions would set both up before
 * probing hardware, and this harness's boot() intentionally does not reconstruct that, per the
 * file header's rationale for using a real `BOOT CPU`).  So the machine check's OWN exception-
 * frame push targets SP = IS - 8 = -8, which wraps to physical ~0x3FFFFFF8 -- itself unbacked --
 * faulting AGAIN before the first fault's dispatch can complete.  stepInstruction()'s existing
 * depth-bound guard (exc.js, unrelated to this file) eventually throws VAXStop(INIE, ...), whose
 * `detail` is `-pending.code` (the LAST fault's negated SCB offset, e.g. 4 for SCB_MCHK) -- not
 * the address that started the cascade.  `bus.addrFault` is no help either: it is overwritten by
 * every fault in the cascade, so by the time it settles it holds the LAST address, not the first.
 *
 * So the FIRST bus-fault address (the actual undecoded-hardware boundary this item exists to
 * name) has to be captured directly, at the moment it happens, before any cascade obscures it.
 * `firstFaultAddr`/`firstFaultAccess`/`firstFaultPC` do exactly that -- reset at the top of every
 * instruction attempt, so if THIS attempt never faults they stay null, and if it does fault
 * (possibly repeatedly, cascading through the depth-bound guard) they hold the FIRST one.
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

    let firstFaultAddr = null, firstFaultAccess = null, firstFaultPC = null;
    let realOnBusFault = cpu.onBusFault.bind(cpu);
    cpu.onBusFault = function(addr, access) {
        if (firstFaultAddr === null) {
            firstFaultAddr = addr >>> 0;
            firstFaultAccess = access;
            firstFaultPC = cpu.exc.faultPC >>> 0;      // the FAULTING instruction's own start PC
        }
        return realOnBusFault(addr, access);
    };

    let steps = 0, stop = null;
    try {
        while (steps < opts.maxSteps) {
            firstFaultAddr = null; firstFaultAccess = null; firstFaultPC = null;
            cpu.stepCPU(1); steps++;
        }
    } catch (e) {
        if (!(e instanceof VAXStop)) throw e;
        stop = e;
    }
    /*
     * MEASURED (pcjsvax-320): hst.finish(cpu) is a NO-OP when hst.pending is already null
     * (simhtrace.js:420, `if (!p) return;`) -- and pending is null whenever the instruction that
     * faulted never reached hst.record()'s call point (cpustate.js's fnExecute calls record()
     * AFTER specifier resolution, BEFORE the body).  pcjsvax-223's original boundary (a STORE
     * faulting inside its own body) always left a record() already pending, so
     * "force-finalize, then trim the last record" was always correct.  Once this item's decode
     * lets that store SUCCEED, the boundary can now land on operand resolution of the NEXT
     * instruction -- a fault BEFORE record() ever runs for it -- and hst.count is ALREADY the
     * exact count of genuinely comparable instructions; trimming one more would silently drop the
     * last one that actually completed (here: the MOVL that stores to SSC+0x0 itself) from the
     * SIMH comparison. `finalizedPartial` distinguishes the two shapes by directly observing
     * whether finish() changed hst.count, rather than assuming which shape occurred.
     */
    let countBeforeFinish = hst.count;
    hst.finish(cpu);
    let finalizedPartial = hst.count > countBeforeFinish;
    let tracePath = path.join(opts.scratch, "romdiff-js.trace");
    fs.writeFileSync(tracePath, hst.text());
    return {
        bus, cpu, tracePath, records: hst.count, steps, stop, finalizedPartial,
        unavailable: hst.unavailable,
        pc: cpu.regs[15] >>> 0, psl: cpu.psl >>> 0,
        firstFaultAddr, firstFaultAccess, firstFaultPC
    };
}

/**
 * probeSimhBackedAt(simh, opts, addr, fWrite)
 *
 * Does SIMH itself service this exact physical reference, or does it machine-check too?
 * Answered by EXECUTION, not by a hand-maintained address-range list: console EXAMINE/DEPOSIT
 * bypasses ReadReg()/WriteReg()/ReadQb()/WriteQb() entirely (cpu_ex()/cpu_dep() check
 * ADDR_IS_MEM/CDG/ROM/NVR directly and never touch the register-space dispatch), so only an
 * actual instruction reproduces the real access path -- the same reasoning tests/mchkdiff.js's
 * calibrate() is built on, applied here to ONE address instead of a whole candidate pool.
 *
 * A longword probe of the CONTAINING aligned longword (`addr & ~3`), matching the ORIGINAL
 * access's direction: `MOVL #0, @#addr` for a write, `TSTL @#addr` for a read.  Fresh SCBB/KSP/IS
 * are deposited (unlike the boundary instruction itself, this probe is not trying to reproduce the
 * boot state, only to classify the ONE address), so a dispatch here -- if SIMH takes one at all --
 * completes cleanly rather than cascading; only whether PC reaches the handler matters.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {number} addr
 * @param {boolean} fWrite
 * @returns {boolean} true if SIMH does NOT machine-check here (i.e. it is backed)
 */
function probeSimhBackedAt(simh, opts, addr, fWrite)
{
    const R_SCBB = 0x00100000, R_HANDLER = 0x00102000, R_CODE = 0x00104000;
    const R_KSP = 0x00110000, R_IS = 0x00118000;
    let opcMOVL = OPCODES.indexOf("MOVL"), opcTSTL = OPCODES.indexOf("TSTL");
    if (opcMOVL < 0 || opcTSTL < 0) throw new Error("romdiff.js: MOVL/TSTL not found in drom.js OPCODES");
    let a = (addr >>> 0) & ~3;
    let instr = fWrite
        ? [opcMOVL & 0xFF, 0x00, 0x9F, a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF]
        : [opcTSTL & 0xFF, 0x9F, a & 0xFF, (a >>> 8) & 0xFF, (a >>> 16) & 0xFF, (a >>> 24) & 0xFF];
    let lines = [
        "set cpu 16m", "set cpu simhalt", "reset all",
        `deposit SCBB ${hex(R_SCBB)}`, `deposit -l ${hex(R_SCBB + 4)} ${hex(R_HANDLER)}`,
        `deposit KSP ${hex(R_KSP)}`, `deposit IS ${hex(R_IS)}`
    ];
    for (let i = 0; i < instr.length; i++) lines.push(`deposit -b ${hex(R_CODE + i)} ${instr[i].toString(16)}`);
    lines.push(`deposit PSL 0`, `deposit PC ${hex(R_CODE)}`, "step 1", "examine PC", "exit", "");
    let out = runSimh(simh, lines.join("\n"), path.join(opts.scratch, "romdiff-boundary-probe.ini"));
    let m = /^PC:\s*([0-9A-Fa-f]+)/m.exec(out);
    if (!m) throw new Error("romdiff: boundary probe produced no PC readback; SIMH said:\n" + out);
    return (parseInt(m[1], 16) >>> 0) !== R_HANDLER;
}

/** Same mulberry32 PRNG every VAX differential in this tree uses (busdiff.js, mchkdiff.js, ...),
    duplicated rather than imported because none of them share a utility module -- see those files. */
function mulberry32(a)
{
    return function() {
        a |= 0; a = (a + 0x6D2B79F5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

/**
 * verifySscBaseRandom(simh, opts, seed, n)
 *
 * pcjsvax-320's REAL-WORKLOAD phase (the boot trace above) stores exactly ONE value into
 * SSCBASE -- 0x20140000 -- which already satisfies SSCBASE_RW/SSCBASE_MBO exactly (it IS a valid
 * base address), so the AND/OR mask this file transcribes from vax_sysdev.c's ssc_wr() `case 0x00`
 * is STRUCTURALLY UNTESTED by that run alone: a transcription bug in either mask (e.g. a swapped
 * hex digit in SSCBASE_RW) that only misbehaves on OTHER bit patterns would pass the real-workload
 * phase silently.  This is exactly the class of bug the project's standing rule about randomized
 * phases exists to catch (see docs/design/vax-on-pcjs.md's "uniform random address pools" lesson,
 * applied here to register VALUES rather than addresses).
 *
 * Driven through a REAL instruction round trip on the live oracle -- `MOVL S^#val,@#SSCBASE` then
 * `MOVL @#SSCBASE,R0`, examined via R0 -- the same "only an actual instruction reproduces the real
 * access path" reasoning probeSimhBackedAt() is built on (deposit/examine of a NAMED register like
 * `sysd base` bypasses ssc_wr()'s masking entirely and was confirmed, by direct probe, to read back
 * whatever was deposited UNMASKED -- exactly the false-negative this function exists to avoid).
 * Compared against a bare `new SSCVAX()` instance's own writeReg()/readReg() -- not the whole bus --
 * because the mask formula is what is under test, not the address decode (romdiff's PHASE 1 trace
 * comparison already proves the address decode).
 *
 * Batched into ONE SIMH invocation (mchkdiff.js's runBatch() convention): each case is independent
 * via `reset all`, so a fault or bad value in one case cannot contaminate another.
 *
 * @param {string} simh
 * @param {Object} opts
 * @param {number} seed
 * @param {number} n
 * @returns {Array.<string>} problems (empty if none)
 */
function verifySscBaseRandom(simh, opts, seed, n)
{
    let rnd = mulberry32(seed);
    let opcMOVL = OPCODES.indexOf("MOVL");
    if (opcMOVL < 0) throw new Error("romdiff.js: MOVL not found in drom.js OPCODES");
    const CODE = 0x00104000;
    const SSCBASE = VAX.PHYSMEM.SSC_BASE >>> 0;
    const MARK = "SSCBASERND";

    /* A few boundary-interesting values FIRST (all-zero, all-one, exactly SSCBASE_RW, exactly
       SSCBASE_MBO), THEN n-4 uniform random longwords -- covers both the deliberately-chosen edges
       and the general case, same shape as every other randomized phase in this tree. */
    let vals = [0x00000000, 0xFFFFFFFF | 0, 0x1FFFFC00, 0x20000000 | 0];
    for (let i = vals.length; i < n; i++) vals.push((rnd() * 0x100000000) >>> 0);

    let lines = ["set cpu 16m", "set cpu simhalt"];
    for (let i = 0; i < vals.length; i++) {
        let v = vals[i] >>> 0;
        lines.push(`echo ${MARK}${i}`, "reset all");
        let instr = [
            opcMOVL & 0xFF, 0x8F, v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF,
                            0x9F, SSCBASE & 0xFF, (SSCBASE >>> 8) & 0xFF, (SSCBASE >>> 16) & 0xFF, (SSCBASE >>> 24) & 0xFF,
            opcMOVL & 0xFF, 0x9F, SSCBASE & 0xFF, (SSCBASE >>> 8) & 0xFF, (SSCBASE >>> 16) & 0xFF, (SSCBASE >>> 24) & 0xFF,
                            0x50                                              // R0, direct (MODE.GRN | 0)
        ];
        for (let k = 0; k < instr.length; k++) lines.push(`deposit -b ${hex(CODE + k)} ${instr[k].toString(16)}`);
        lines.push(`deposit PSL 0`, `deposit PC ${hex(CODE)}`, "step 2", "examine -h R0");
    }
    lines.push("exit", "");
    let out = runSimh(simh, lines.join("\n"), path.join(opts.scratch, "romdiff-sscbase-random.ini"));

    let problems = [];
    let marks = [...out.matchAll(new RegExp(MARK + "(\\d+)", "g"))].map((m) => +m[1]);
    let r0s = [...out.matchAll(/^R0:\s*([0-9A-Fa-f]+)/gm)].map((m) => parseInt(m[1], 16) >>> 0);
    if (marks.length !== vals.length || r0s.length !== vals.length) {
        problems.push(`SSC-BASE-RANDOM: expected ${vals.length} cases, SIMH produced ${marks.length} ` +
            `markers and ${r0s.length} R0 readbacks -- some case did not reach comparison; SIMH said:\n` +
            out.slice(0, 2000));
        return problems;
    }
    let ssc = new SSCVAX();
    for (let i = 0; i < vals.length; i++) {
        ssc.reset();
        ssc.writeReg(0x00, vals[i] | 0);
        let want = ssc.readReg(0x00) >>> 0;
        let got = r0s[i] >>> 0;
        if (got !== want) {
            problems.push(`SSC-BASE-RANDOM: case ${i} val=0x${hex(vals[i])} -- SIMH readback=0x${hex(got)}, ` +
                `SSCVAX readback=0x${hex(want)}`);
        }
    }
    return problems;
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
differ_path, js_path, simh_path, unav_path, zerospec, max_js_records = sys.argv[1:7]
sys.path.insert(0, os.path.dirname(differ_path))
from differ import parse_trace, diff_traces

ZEROSPEC = set(x for x in zerospec.split(",") if x)
unavailable = set(json.load(open(unav_path)))

a = parse_trace(js_path)
# max_js_records < 0 means "no trim" (the ordinary case).  A boundary case passes len(a) - 1: the
# JS trace's LAST record is the boundary instruction itself, force-finalized by runRomJS()'s
# unconditional hst.finish(cpu) call after the exception even though its body never completed (it
# aborted mid-store) -- so it is not a real, comparable instruction and must not be counted against
# SIMH's capture, which (correctly bounded at the boundary instruction's OWN start PC) never
# executes it at all.
max_js_records = int(max_js_records)
if max_js_records >= 0:
    a = a[:max_js_records]
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
 * compareTraces(jsPath, simhPath, unavailable, opts, maxJsRecords)
 *
 * @param {number} [maxJsRecords] trim the JS trace to this many records before comparing (used
 *   for the boundary case -- see DIFF_DRIVER's comment); omit or pass a negative number for no trim
 * @returns {Object}
 */
function compareTraces(jsPath, simhPath, unavailable, opts, maxJsRecords = -1)
{
    let driver = path.join(opts.scratch, "romdiff-driver.py");
    fs.writeFileSync(driver, DIFF_DRIVER);
    let unavPath = path.join(opts.scratch, "romdiff-unavailable.json");
    fs.writeFileSync(unavPath, JSON.stringify(unavailable));
    let out = execFileSync("python3", [driver, findDiffer(), jsPath, simhPath, unavPath,
                                       [...ZEROSPEC].join(","), String(maxJsRecords)],
                           {encoding: "utf8", maxBuffer: 1 << 28});
    return JSON.parse(out);
}

/* ------------------------------------------------------------------------------------------- *
 * PHASE 2 -- the mirror                                                                         *
 * ------------------------------------------------------------------------------------------- */

/**
 * verifyMirrorJS(romBytes, opts, magicByte)
 *
 * Checks the mirror against the PRIMARY half on THIS machine, at several offsets including both
 * boundaries, before AND after a write.
 *
 * CORRECTED (post-dispatch veracity review): the ORIGINAL version of this function proved
 * liveness by calling `cpu.boot(magicByte)` and checking that the mirror picked up the magic
 * byte at +4.  That check is VACUOUS as shipped: ka655x.bin already ships 0x02 at offset 4, and
 * the measured magic byte is ALSO 2 (see cpustate.js's ROM_MAGIC_BYTE comment), so boot() writes
 * the SAME value that was already there.  A load-time snapshot-copy mirror (one array per half,
 * copied once from the file, never re-read) passes that check identically to a genuinely live
 * alias -- there is nothing to distinguish them, because nothing actually changed.  This was
 * caught by an adversarial re-run that built exactly that snapshot-copy mirror and confirmed it
 * passed.  The magic-byte-after-boot check below is kept as an ordinary integration assertion
 * (boot() really does write what it claims), but it is NOT what proves the mirror is live.
 *
 * The actual liveness proof is the DISTINCT SENTINEL below: a byte written directly via
 * setByteDirect(), chosen to differ from whatever is already at that offset (so the check cannot
 * degenerate the same way), with the mirror required to track it and the original content
 * restored afterward.  This is the same technique the "magic-byte-not-written" selfcheck mutation
 * already uses to get an observable divergence; it just was not applied here.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
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
            problems.push(`mirror BEFORE any write at +0x${off.toString(16)}: primary=0x${hex(primary, 2)} mirror=0x${hex(mirror, 2)}`);
        }
        if (primary !== romBytes[off]) {
            problems.push(`primary at +0x${off.toString(16)} is 0x${hex(primary, 2)}, expected the ROM file's own 0x${hex(romBytes[off], 2)}`);
        }
    }

    /*
     * THE LIVENESS PROOF.  A byte guaranteed to actually CHANGE, at an offset the boot sequence
     * never touches (so it cannot collide with the magic-byte check below), written through the
     * same Direct accessor boot() itself uses.  If the primary and its ORIGINAL content happen to
     * already equal 0x37, use 0x5A instead -- the whole point is that the write is observable.
     */
    let sentinelOff = 0x40;
    let sentinelAddr = (base + sentinelOff) >>> 0;
    let before = bus.getByte(sentinelAddr);
    let sentinel = (before === 0x37) ? 0x5A : 0x37;
    bus.setByteDirect(sentinelAddr, sentinel);
    let primarySentinel = bus.getByte(sentinelAddr);
    let mirrorSentinel = bus.getByte((base + size + sentinelOff) >>> 0);
    if (primarySentinel !== sentinel) {
        problems.push(`primary +0x${sentinelOff.toString(16)} after a direct sentinel write is 0x${hex(primarySentinel, 2)}, expected 0x${hex(sentinel, 2)}`);
    }
    if (mirrorSentinel !== sentinel) {
        problems.push(`mirror +0x${sentinelOff.toString(16)} after a direct sentinel write to the PRIMARY is 0x${hex(mirrorSentinel, 2)}, expected it to track 0x${hex(sentinel, 2)} -- ` +
            `the mirror is a load-time COPY, not a live alias (this is exactly the check a snapshot-copy mirror fails and the magic-byte check above cannot distinguish)`);
    }
    bus.setByteDirect(sentinelAddr, before);                  // restore -- this function must not leave the machine dirty

    /* Ordinary integration check: boot() really does write what it claims.  NOT a liveness proof
       on its own (see the file-header note above) -- ka655x.bin already ships 0x02 at +4 and the
       measured magic byte is also 2, so this can pass even against a snapshot-copy mirror. */
    cpu.reset();
    cpu.boot(magicByte);
    let primary4 = bus.getByte((base + 4) >>> 0);
    let mirror4 = bus.getByte((base + size + 4) >>> 0);
    if (primary4 !== magicByte) {
        problems.push(`primary +4 after boot() is 0x${hex(primary4, 2)}, expected the magic byte 0x${hex(magicByte, 2)}`);
    }
    if (mirror4 !== magicByte) {
        problems.push(`mirror +4 after boot() is 0x${hex(mirror4, 2)}, expected it to track the primary's magic byte 0x${hex(magicByte, 2)}`);
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
 *
 * TWO mirror mutations, deliberately, not one, after a veracity review found the first one alone
 * was not distinguishing what it claimed to: "mirror-not-aliased" (a ZERO-FILLED buffer) is only
 * caught because 0x00 happens to differ from whatever byte is really there -- it proves "the
 * mirror reads garbage", not "the mirror is a stale copy".  "mirror-stale-copy" (a genuine
 * snapshot, `Int32Array.from(block.adw)`, taken at construction time) is what actually exercises
 * the failure mode this item's commit message claims to guard against: a mirror that read
 * correctly at load time and then silently stopped tracking the primary.  Both are checked with a
 * DISTINCT SENTINEL write (see below), never with the boot-time magic byte -- ka655x.bin already
 * ships 0x02 at offset 4 and the measured magic byte is ALSO 2, so a check keyed on that offset
 * cannot distinguish a live alias from a snapshot that merely happened to copy the right value
 * once.
 */
const MUTATIONS = {
    /* The mirror aliases nothing: each mirror block gets its OWN zero-filled buffer instead of the
       primary block's array. */
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
    /* The mirror is a load-time SNAPSHOT of the primary -- correct at the moment addRom() builds
       it (the copy is taken after the ROM file's bytes are already loaded into the primary), but
       frozen from then on.  Every check that only reads INITIAL content, or that re-writes a value
       already present, passes this identically to a true alias; only a write of something NEW,
       read back through the mirror, can tell them apart. */
    "mirror-stale-copy": (cpu, bus) => {
        let orig = BusVAX.makeRomAliasController;
        BusVAX.makeRomAliasController = function(busArg) {
            return {
                getControllerBuffer(addr) {
                    let primaryAddr = ((addr >>> 0) - VAX.PHYSMEM.ROM_SIZE) >>> 0;
                    let block = busArg.aMemBlocks[primaryAddr >>> busArg.nBlockShift];
                    return [Int32Array.from(block.adw), 0];      // a COPY, not a reference
                },
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
    },
    /* pcjsvax-320: the SSC base register is no longer decoded at all -- SSCVAX.writeReg() stops
       recognizing REG_BASE, so a store to SSC+0x0 (this item's whole reason for existing) falls
       through to a bus fault exactly as it did before this item. */
    "ssc-base-not-decoded": (cpu, bus) => {
        let orig = SSCVAX.prototype.writeReg;
        SSCVAX.prototype.writeReg = function() { return false; };
        return () => { SSCVAX.prototype.writeReg = orig; };
    },
    /* pcjsvax-320: the SSC base register is decoded, but WITHOUT vax_sysdev.c's SSCBASE_RW/
       SSCBASE_MBO masking -- a store of a raw value would corrupt the must-be-one bits, which real
       hardware never allows software to clear. */
    "ssc-base-wrong-mask": (cpu, bus) => {
        let orig = SSCVAX.prototype.writeReg;
        SSCVAX.prototype.writeReg = function(rg, val) {
            if (rg === 0x00) { this.base = val | 0; return true; }
            return orig.call(this, rg, val);
        };
        return () => { SSCVAX.prototype.writeReg = orig; };
    },
    /* pcjsvax-320: BoundaryAccounting.compute() reverts to unconditionally trimming one record --
       the bug this item's own boundary-advance exposed (see runRomJS()'s and main()'s doc
       comments): correct only when the boundary instruction's fault happened inside its BODY
       (a force-finalized partial record), wrong when it happened during specifier resolution of
       the NEXT instruction (nothing was ever finalized, so js.records is already exact). */
    "boundary-off-by-one": (cpu, bus) => {
        let orig = BoundaryAccounting.compute;
        BoundaryAccounting.compute = function(records) {
            let comparableRecords = records - 1;
            return {comparableRecords, instrNum: comparableRecords + 1};
        };
        return () => { BoundaryAccounting.compute = orig; };
    }
};

/**
 * selfcheck(romBytes, opts, magicByte)
 *
 * Every mutation is checked with a FAST, DETERMINISTIC, structural assertion rather than a second
 * full SIMH trace run -- see the file's test_decisions in the item's report for why: none of these
 * is a subtle multi-instruction timing bug (that is what cpudiff.js's own --selfcheck already
 * proves this loop can catch); each is directly observable as a single readback or a single pure-
 * function call, and re-invoking SIMH eight more times would cost real wall-clock time for no
 * additional detection power.
 *
 * @param {Uint8Array} romBytes
 * @param {Object} opts
 * @param {number} magicByte
 */
function selfcheck(romBytes, opts, magicByte)
{
    console.log("\nSELFCHECK -- each mutation must be caught\n");
    /*
     * Both mirror mutations patch BusVAX.makeRomAliasController, which addRom() only CONSULTS at
     * construction time -- so unlike the other two (which patch CPUStateVAX.prototype.boot or an
     * already-built block, both of which take effect at the later CALL), they must be applied
     * BEFORE makeMachine() builds the bus, or the mutation silently never engages and "SURVIVED"
     * would be a false negative rather than a real coverage hole.
     */
    let PRE_MACHINE = new Set(["mirror-not-aliased", "mirror-stale-copy"]);
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
                if (name === "mirror-not-aliased" || name === "mirror-stale-copy") {
                    /*
                     * A DISTINCT SENTINEL, never the boot-time magic byte: ka655x.bin already
                     * ships 0x02 at offset 4 and the measured magic byte is also 2, so a check
                     * keyed there cannot tell a live alias from a snapshot that merely copied the
                     * right value once (this is exactly the vacuity a veracity review found in an
                     * earlier version of this file).
                     */
                    let base = VAX.PHYSMEM.ROM_BASE >>> 0, size = VAX.PHYSMEM.ROM_SIZE;
                    let addr = (base + 0x40) >>> 0;
                    let before = bus.getByte(addr);
                    let sentinel = (before === 0x37) ? 0x5A : 0x37;
                    bus.setByteDirect(addr, sentinel);
                    let m = bus.getByte((base + size + 0x40) >>> 0);
                    if (m !== sentinel) {
                        caught = true;
                        how = `mirror +0x40 = 0x${hex(m, 2)} after a direct sentinel write of 0x${hex(sentinel, 2)} to the primary`;
                    }
                    bus.setByteDirect(addr, before);
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
                } else if (name === "ssc-base-not-decoded") {
                    let addr = VAX.PHYSMEM.SSC_BASE >>> 0;
                    bus.setLong(addr, 0x12345678 | 0);
                    if (bus.checkFault()) {
                        caught = true;
                        how = `a store to SSC_BASE (0x${hex(addr)}) faulted -- the base register is no longer decoded`;
                    }
                } else if (name === "ssc-base-wrong-mask") {
                    let addr = VAX.PHYSMEM.SSC_BASE >>> 0;
                    bus.setLong(addr, 0);
                    let v = bus.getLong(addr) >>> 0;
                    let want = 0x20000000;                  // SSCBASE_MBO -- must-be-one bits
                    if (v !== want) {
                        caught = true;
                        how = `SSC_BASE readback after storing 0 is 0x${hex(v)}, expected 0x${hex(want)} (SSCBASE_MBO preserved)`;
                    }
                } else if (name === "boundary-off-by-one") {
                    /*
                     * A pure-function check, deliberately not touching bus/cpu at all: the exact
                     * (records=2, finalizedPartial=false) input this item's own live romdiff run
                     * produced (see runRomJS()'s doc comment) -- the correct answer is
                     * {comparableRecords: 2, instrNum: 3} (both records genuinely completed, the
                     * boundary is the THIRD instruction); the mutation above always subtracts one.
                     */
                    let got = BoundaryAccounting.compute(2, false);
                    if (got.comparableRecords !== 2 || got.instrNum !== 3) {
                        caught = true;
                        how = `BoundaryAccounting.compute(2, false) = ${JSON.stringify(got)}, expected ` +
                            `{comparableRecords: 2, instrNum: 3}`;
                    }
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

    let boundary = null;
    if (!js.stop) {
        problems.push(`the JS machine ran all ${opts.maxSteps} steps without reaching a device-register ` +
            `boundary; this item's whole point is to name that boundary, so a run that never finds one ` +
            `is not a pass -- raise --max-steps only after confirming a real boundary is further out`);
    } else {
        /*
         * MEASURED (veracity re-dispatch, pcjsvax-446): the FIRST bus fault's address, if there was
         * one, is checked against the real oracle BEFORE anything else -- because if SIMH itself
         * services that address (SSC/NVR/CDG: backed end-to-end, same category this item's own
         * calibration already established for tests/mchkdiff.js), then js.pc (the PC AFTER the
         * whole exception-cascade this item's fix now genuinely dispatches) is NOT a point SIMH's
         * real, non-faulting execution will ever reach -- it is deep inside JS-only exception
         * machinery.  Bounding the SIMH capture there either never hits the breakpoint in any
         * useful sense, or hits it by coincidence far later having recorded a different number of
         * instructions -- which is exactly the "SIMH's capture was shorter than the JS run"
         * confusion this fix replaces with a named, honest boundary.
         *
         * The fix: when the first-fault address is confirmed BACKED, bound the SIMH capture at the
         * BOUNDARY INSTRUCTION'S OWN PC instead (`firstFaultPC` -- both machines reach that address
         * identically, since everything before it is unaffected by what pcjsvax-320 has not yet
         * decoded).  That makes the two traces directly comparable over the LEADING portion (every
         * instruction before the boundary must still match exactly -- a real divergence there is
         * still a failure, never swallowed), and the boundary itself is reported by name, not as a
         * trace-length mismatch.
         */
        if (js.firstFaultAddr !== null) {
            let fWrite = js.firstFaultAccess === VAX.ACCESS.WRITE;
            if (probeSimhBackedAt(simh, opts, js.firstFaultAddr, fWrite)) {
                boundary = {addr: js.firstFaultAddr, fWrite, instrPC: js.firstFaultPC};
            }
        }
        let breakAddr = boundary ? boundary.instrPC : js.pc;
        let simhTrace = captureSimhTrace(simh, opts, breakAddr);
        /*
         * MEASURED (veracity re-dispatch, pcjsvax-446): the boundary instruction's OWN record in
         * js.tracePath is a FORCE-FINALIZED partial one WHEN its fault happened inside the
         * instruction's BODY (runRomJS()'s unconditional `hst.finish(cpu)` call finalizes whatever
         * hst.record() already started, even though the body itself aborted) -- SIMH's capture,
         * correctly bounded at that same instruction's start PC, never executes it at all (the
         * breakpoint fires on FETCH), so that case compares `js.records - 1` records: everything
         * genuinely completed before the boundary, not `js.records`.
         *
         * MEASURED (pcjsvax-320): that is NOT the only shape a boundary can take, and assuming it
         * unconditionally is what pcjsvax-320 exposed the first time a boundary landed on
         * SPECIFIER RESOLUTION of the instruction AFTER one that used to be the boundary and now
         * completes successfully -- a fault reached BEFORE hst.record() ever ran for it, per
         * cpustate.js's fnExecute ordering (record() fires after specifier resolution, before the
         * body).  hst.finish(cpu) is then a genuine no-op (simhtrace.js: `if (!p) return;`), so
         * js.records is ALREADY the exact count of comparable instructions and trimming one more
         * would silently drop the LAST one that actually completed -- here, the very MOVL storing
         * to SSC+0x0 this item exists to decode -- out of the SIMH comparison entirely, the one
         * case this item most needs graded.  `js.finalizedPartial` (measured directly, by whether
         * hst.finish() changed hst.count -- not assumed from which kind of boundary this is)
         * distinguishes the two shapes; `comparableRecords` and `boundaryInstrNum` below are the
         * single computation both the trim and every "instruction #N" label use, so they cannot
         * drift apart the way the pre-existing code's inlined `js.records`/`js.records - 1` did.
         */
        let {comparableRecords, instrNum: boundaryInstrNum} = BoundaryAccounting.compute(js.records, js.finalizedPartial);
        let maxJsRecords = boundary ? comparableRecords : -1;
        let cmp = compareTraces(js.tracePath, simhTrace, js.unavailable, opts, maxJsRecords);
        console.log(`  SIMH breakpoint        : ${hex(breakAddr)} ` +
            (boundary ? "(the boundary instruction's own PC -- SIMH reaches this identically)"
                      : "(this run's own stopping PC)"));
        console.log(`  normalized records     : CMPD-tail=${cmp.normalized_cmpd} zerospec=${cmp.normalized_zerospec} store-faulted=${cmp.normalized_unavailable}`);
        console.log(`  trace comparison       : ${cmp.match ? "MATCH over all " + cmp.simh_records + " records" : "DIVERGE"}`);
        if (!cmp.match) {
            let d = cmp.divergence;
            problems.push(`TRACE: diverged at instruction ${d.index} (field=${d.field}, PC=${d.pc_a}): ${d.detail}`);
        }
        if (cmp.simh_records !== cmp.js_records) {
            problems.push(`TRACE: JS produced ${cmp.js_records} comparable records but only ${cmp.simh_records} ` +
                `were compared -- SIMH's capture was shorter than the JS run, so some JS instructions never ` +
                `reached comparison`);
        }
        if (boundary) {
            boundary.instrNum = boundaryInstrNum;
            console.log(`\n  UNDECODED-HARDWARE BOUNDARY: instruction #${boundaryInstrNum} ` +
                `(${boundary.fWrite ? "write" : "read"}) touched ${nameAddress(boundary.addr)}, which SIMH ` +
                `services (ssc_rd/ssc_wr -- backed end-to-end, same category as CDG) but this bus does ` +
                `not decode yet.  See the next device item, which will move this boundary forward.`);
        }
    }

    /* ---- PHASE 1b: the SSC base register mask, RANDOMIZED against the real oracle ----
       See verifySscBaseRandom()'s doc comment for why the trace above cannot exercise this on its
       own: the ROM stores exactly ONE value (0x20140000), which already satisfies the mask, so a
       transcription bug in SSCBASE_RW/SSCBASE_MBO would pass the real-workload phase silently. */
    let sscCases = +getArg("--ssc-base-cases", 40);
    if (sscCases < SSC_BASE_CASES_FLOOR) {
        problems.push(`--ssc-base-cases ${sscCases} is below the enforced floor (${SSC_BASE_CASES_FLOOR}); ` +
            `this run would under-cover the SSC base register's mask formula and must not be allowed to pass.`);
    } else {
        let sscProblems = verifySscBaseRandom(simh, opts, +getArg("--seed", 0xB16B00B5), sscCases);
        console.log(`\nSSC BASE REGISTER MASK (randomized, ${sscCases} cases, real oracle): ` +
            `${sscProblems.length ? "DIVERGED" : "MATCH"}`);
        for (let p of sscProblems) problems.push(p);
    }

    /* ---- PHASE 2: the mirror ---- */
    let mirrorProblems = verifyMirrorJS(romBytes, opts, magicByte);
    for (let p of mirrorProblems) problems.push("MIRROR (JS): " + p);
    if (js.stop) {
        let simhMirror = verifyMirrorSimh(simh, opts, boundary ? boundary.instrPC : js.pc);
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

    report(problems, js, boundary);
}

function report(problems, js, boundary)
{
    if (problems.length) {
        console.error("\nFAILURES:");
        for (let p of problems) console.error("  " + p);
        process.exit(1);
    }
    console.log("\nOK");
    if (boundary) {
        /* The NAMED boundary this item exists to report -- see main()'s doc comment above for why
           js.stop.detail (the cascading depth-bound guard's LAST fault code, not the first fault's
           address) is not what gets named here.  boundary.instrNum (main()'s boundaryInstrNum) is
           NOT always js.records: it is js.records only when the boundary instruction's own fault
           happened inside its BODY (a force-finalized partial record IS the last of js.records) --
           when it instead happened during operand resolution of the instruction AFTER one that now
           completes (this item's own new case), nothing was ever finalized for it and its true
           ordinal is js.records + 1.  See main()'s comment at comparableRecords/boundaryInstrNum
           for the measurement that tells the two apart. */
        console.log(`\nSTOPPING POINT: instruction #${boundary.instrNum} (0-based index ${boundary.instrNum - 1}), ` +
            `PC=${hex(boundary.instrPC)}, attempted a ${boundary.fWrite ? "write to" : "read of"} ` +
            `${nameAddress(boundary.addr)} -- SIMH services this address; this bus does not decode it ` +
            `yet.`);
    } else if (js && js.stop) {
        console.log(`\nSTOPPING POINT: instruction #${js.records} (0-based index ${js.records - 1}), ` +
            `PC=${hex(js.pc)}, touched ${nameAddress(js.stop.detail)} -- ${js.stop.reason}`);
    }
}

main();

export { querySysModel, runRomJS, captureSimhTrace, compareTraces, verifyMirrorJS, nameAddress, probeSimhBackedAt };
