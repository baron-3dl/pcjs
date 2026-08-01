/**
 * @fileoverview Computes -- and grades against a REAL, EXECUTED Open SIMH microvax3900 -- what the
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
 *               EHKAA diagnostic actually requires of the VAX CIS (packed-decimal / character-
 *               string) instruction groups.
 *
 * WHY THIS EXISTS
 * ---------------
 * `tests/base_group_residual.js` is this project's precedent: a scope number that anybody can
 * re-derive beats a scope number somebody once counted.  Every hand-derived CIS scope number in
 * this project has been wrong -- the profile's §6 said 7 opcodes, a correction said 17, and both
 * were answering a question ("which CIS mnemonics appear in the trace?") that is NOT the question
 * the milestone actually turns on ("what must this CPU implement to pass EHKAA?").
 *
 * This script answers the second question, from four independent, re-runnable measurements, none
 * of which is a reading of `vax_cis.c`:
 *
 *   GROUPS    drom.js's own IG_PACKD/IG_EMONL classification -- the same table decode.js uses.
 *   SHOWV     `SHOW CPU -V` on the real binary: SIMH's own report of which groups this model
 *             IMPLEMENTS and which it merely EMULATES.  Ground truth from the executable.
 *   TRACE     The EHKAA instruction history, parsed on the MNEMONIC FIELD ONLY (a `grep` for the
 *             mnemonics over the raw trace both over-matches operand/disassembly text and misses
 *             opcodes -- that is how "7" and "17" both happened), giving the touched set and the
 *             per-opcode counts, cross-referenced against SIMH's OWN `EMULFAULT` and `INTEXC`
 *             debug categories -- captured in the SAME run -- for the part no prior count looked
 *             at: the DISPOSITION of every single instance.  Did the CPU EXECUTE it, or did it
 *             take the emulated-instruction exception, or some other fault?
 *   GATE      EHKAA run to completion three times: once as the model ships, once with SIMH's
 *             `SET CPU INSTRUCTION=PACKED;EMULATED` forcing NATIVE CIS execution, once with
 *             `=EXTENDED`.  Whether the diagnostic still reaches its documented PASS halt is the
 *             only measurement that actually settles what the gate requires.
 *
 * WHAT IT MEASURES (and what that overturned)
 * -------------------------------------------
 * Run it.  The disposition column is the finding: EHKAA issues 23 CIS opcodes and executes ZERO
 * of them.  Every instance takes a fault or an exception:
 *
 *   - most take the emulated-instruction exception -- `vax_cpu.c`'s `cpu_emulate_exception()`, a
 *     48-byte operand frame through `SCB_EMULATE` (0xC8), or an 8-byte frame through
 *     `SCB_EMULFPD` (0xCC) when PSL<FPD> is already set -- which EHKAA's own handler services;
 *   - `EMODH`/`POLYH` are routed by `vax_cpu.c` to `op_octa()`, NOT to `op_cis()`, and `op_octa()`
 *     opens by faulting when `VAX_EXTAC` is absent: they take a RESERVED-INSTRUCTION fault
 *     (`SCB` 0x10).  They need no H_float here at all -- see the note below;
 *   - at least one instance takes an ACV (`SCB` 0x20) instead, because the emulate frame's own
 *     stack write-probe faults on a deliberately-inaccessible executive-mode stack.  EHKAA is
 *     testing that too.
 *
 * And the GATE phase shows the consequence: implementing CIS NATIVELY does not merely fail to
 * help, it BREAKS the gate.  The diagnostic stops reaching its PASS halt.  EHKAA is testing the
 * emulation trap, not the arithmetic.
 *
 * The reason no earlier measurement caught this is worth recording: `cpu_emulate_exception()` is
 * NOT a call to `intexc()`, so it emits nothing in the `INTEXC` debug category -- the exact log
 * `docs/reference/ehkaa-profile.md` §5 built its 25-vector list from.  0xC8 and 0xCC are missing
 * from that list because the instrument could not see them, not because they were not taken.
 *
 * Usage:
 *     node machines/dec/vax/tests/cis_group_scope.js
 *     node machines/dec/vax/tests/cis_group_scope.js --json
 *     node machines/dec/vax/tests/cis_group_scope.js --check     # exit non-zero on any drift
 *     node machines/dec/vax/tests/cis_group_scope.js --selfcheck # prove it would notice if wrong
 *     node machines/dec/vax/tests/cis_group_scope.js --simh PATH --ehkaa PATH --keep DIR
 *
 * `--keep DIR` writes the instruction history and the debug log there and reuses them on a later
 * run (the capture is ~145MB and takes a few seconds); without it both are temporary.  The
 * capture needs patches 0001+0002 (`PREG`/`IBYT`), same as intdiff.js.
 */

import fs from "fs";
import os from "os";
import path from "path";
import { execFileSync } from "child_process";
import { fileURLToPath } from "url";

import { DROM, DROM_STRIDE, DR, IG, OPCODES } from "../modules/v2/drom.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

/* The documented EHKAA PASS halt, docs/reference/ehkaa-profile.md §1.  Reaching any OTHER halt is
   a diagnostic failure, which is precisely what the GATE phase is looking for.  `| 0` because a
   VAX S0 address is NEGATIVE as a JavaScript int32 -- see defines.js, and README.md's "Read this
   before writing any VAX code".  Comparing the literal against a parsed `| 0` PC without it is
   the exact defect that convention exists to prevent, and it bit this file once. */
const EHKAA_PASS_PC = 0x80018AD1 | 0;

/* The two CIS instruction groups.  IG_EXTAC (Extended-Accuracy, H_float) is a THIRD group SIMH
   also merely emulates, but no opcode of it belongs to CIS; it is reported separately below
   because §7 of the profile treats it as settled and this script re-derives that too. */
const CIS_GROUPS = new Map([[IG.PACKD, "Packed-Decimal-String-Group"], [IG.EMONL, "Emulated-Only-Group"]]);

function hex(v, n = 8) { return ((v >>> 0).toString(16).toUpperCase()).padStart(n, "0"); }

/* ------------------------------------------------------------------------------------------- *
 * GROUPS -- membership straight out of the decode ROM decode.js itself consults.                  *
 * ------------------------------------------------------------------------------------------- */

function groupOpcodes(groups)
{
    let byGroup = new Map();
    for (let g of groups.keys()) byGroup.set(g, []);
    for (let opc = 0; opc < 512; opc++) {
        let hdr = DROM[opc * DROM_STRIDE];
        if (hdr === 0) continue;                                     // undefined slot
        let group = (hdr >> DR.V_IGMASK) & DR.M_IGMASK;
        if (!byGroup.has(group)) continue;
        let mn = OPCODES[opc];
        if (!mn) throw new Error(`opcode 0x${opc.toString(16)} is in group ${group} but has no mnemonic`);
        byGroup.get(group).push(mn);
    }
    for (let list of byGroup.values()) list.sort();
    return byGroup;
}

/* ------------------------------------------------------------------------------------------- *
 * SIMH plumbing -- same discovery order and same no-fallback rule as every sibling differential.   *
 * ------------------------------------------------------------------------------------------- */

function findSimh(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    for (let v of ["SIMH_CIS_BIN", "SIMH_INT_BIN", "SIMH_DECODE_BIN", "SIMH_BIN"]) {
        if (process.env[v]) candidates.push(process.env[v]);
    }
    let scratch = process.env['PCJS_VAX_SCRATCH'];
    if (scratch) candidates.push(path.join(scratch, "open-simh/BIN/microvax3900"));
    candidates.push(path.join(os.tmpdir(), "pcjs-vax-simh/open-simh/BIN/microvax3900"));
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error(
        "This computation grades against a REAL SIMH microvax3900 (patches 0001+0002); it has no\n" +
        "fixture fallback.  Build one with machines/dec/vax/tests/simh/build.sh and pass --simh\n" +
        "PATH or set $SIMH_CIS_BIN.  Tried:\n  " + candidates.join("\n  "));
}

function findEhkaa(pathArg)
{
    let candidates = [];
    if (pathArg) candidates.push(pathArg);
    if (process.env['EHKAA_EXE']) candidates.push(process.env['EHKAA_EXE']);
    let repo = process.env['PCJS_VAX_REPO'];
    if (repo) candidates.push(path.join(repo, "open-simh/VAX/tests/ehkaa.exe"));
    candidates.push(path.resolve(REPO_ROOT, "../pcjs-vax/open-simh/VAX/tests/ehkaa.exe"));
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error("EHKAA diagnostic not found.  Pass --ehkaa PATH or set $PCJS_VAX_REPO.  Tried:\n  " +
                    candidates.join("\n  "));
}

function runSimh(bin, script, scratch, tag)
{
    let ini = path.join(scratch, `cis_group_scope-${tag}.ini`);
    fs.writeFileSync(ini, script);
    try {
        return execFileSync(bin, [ini], {encoding: "utf8", maxBuffer: 1 << 28, timeout: 20 * 60 * 1000});
    } finally {
        try { fs.unlinkSync(ini); } catch (e) { /* best effort */ }
    }
}

/* ------------------------------------------------------------------------------------------- *
 * SHOWV -- SIMH's own account of what this model implements versus merely emulates.                *
 * ------------------------------------------------------------------------------------------- */

function phaseShowV(simh, scratch)
{
    let out = runSimh(simh, "show cpu -v\nquit\n", scratch, "showv");
    /* Output shape: a line "Implementing: <group>" or "Emulating: <group>", then tab-indented
       mnemonic rows, then the next group line.  Parse the section headers and attribute every
       group name that follows to that disposition. */
    let disposition = new Map();
    let mode = null;
    for (let line of out.split("\n")) {
        let m = /^\s*(Implementing|Emulating):\s*(.*?)\s*$/.exec(line);
        if (m) {
            mode = m[1];
            if (m[2]) disposition.set(m[2], mode);
            continue;
        }
        if (line.startsWith("\t") || !line.trim()) continue;          // mnemonic rows / blanks
        /* A bare group-name continuation line under the current heading. */
        let name = line.trim();
        if (mode && /-Group$|^G-Float$|^D-Float$|^Base Instruction Group$/.test(name)) {
            disposition.set(name, mode);
        } else if (!/^CPU\b|^\s*\d+MB|^MicroVAX|^Goodbye/.test(line)) {
            mode = null;
        }
    }
    return {raw: out, disposition};
}

/* ------------------------------------------------------------------------------------------- *
 * TRACE -- mnemonic-field-only parse, plus the per-instance disposition nobody measured before.    *
 * ------------------------------------------------------------------------------------------- */

/**
 * Streams the patch-0002 instruction history.  Only the fields this computation needs are kept:
 * the header line's PC/PSL/mnemonic and `IBYT`'s length, so "did control fall through to PC+ilen"
 * is answerable.
 *
 * The mnemonic is `line.split(" ", 1)[0]` of the text AFTER the "PC PSL| " prefix -- the mnemonic
 * FIELD, never a substring search over the whole line.  That distinction is the entire reason the
 * two previous hand counts disagreed.
 */
function parseTrace(tracePath, onRecord)
{
    const fd = fs.openSync(tracePath, "r");
    const CHUNK = 1 << 22;
    let buf = Buffer.allocUnsafe(CHUNK);
    let carry = "";
    let rec = null, n = 0, lineno = 0;
    const HDR = /^([0-9A-Fa-f]{8}) ([0-9A-Fa-f]{8})\| (.*)$/;
    const fields = (line) => { let sp = line.indexOf(" "); return sp < 0 ? [] : line.slice(sp + 1).split(" "); };

    const handleLine = (line) => {
        lineno++;
        let m = HDR.exec(line);
        if (m) {
            if (rec) throw new Error(`${tracePath}:${lineno}: new record before previous completed`);
            rec = {index: n, pc: parseInt(m[1], 16) | 0, psl: parseInt(m[2], 16) | 0,
                   mnemonic: m[3].split(" ", 1)[0], ilen: 0};
            return;
        }
        if (!rec) return;
        if (line.startsWith("IBYT ")) { rec.ilen = +fields(line)[0]; return; }
        if (line.startsWith("BRDP ")) { n++; onRecord(rec); rec = null; return; }
    };

    for (;;) {
        let got = fs.readSync(fd, buf, 0, CHUNK, null);
        if (got <= 0) break;
        let text = carry + buf.toString("latin1", 0, got);
        let start = 0;
        for (;;) {
            let nl = text.indexOf("\n", start);
            if (nl < 0) break;
            let line = text.slice(start, nl);
            if (line.endsWith("\r")) line = line.slice(0, -1);
            handleLine(line);
            start = nl + 1;
        }
        carry = text.slice(start);
    }
    fs.closeSync(fd);
    if (carry.length) handleLine(carry);
    return n;
}

/**
 * Parses the debug log SIMH itself writes for the `EMULFAULT` and `INTEXC` categories, in order.
 * These are the DISPOSITION oracle: `cpu_emulate_exception()` announces itself in EMULFAULT (and
 * NOWHERE else -- it never calls `intexc()`, which is why nothing in this project saw it before),
 * and every other exception dispatch announces itself in INTEXC with the SCB vector it took.
 *
 *   DBG(n)> CPU EMULFAULT: OP=CMPP3, fault_PC=800123b6, PC=..., PSL=..., SP=..., nPC=...
 *   DBG(n)> CPU EMULFAULT: FPD OP=ADDP4, fault_PC=80012c51, ...
 *   DBG(n)> CPU INTEXC: PC=80018155, PSL=..., SP=..., VEC=00000020, nPSL=..., nSP=...
 *   DBG(n)> same as above (156 times)
 *
 * THE LAST LINE IS THE TRAP.  `sim_debug` COLLAPSES a run of byte-identical messages into one
 * "same as above (k times)" line.  EHKAA's reserved-instruction test is a loop that pokes a
 * different opcode into the SAME address and re-executes it, so its whole sweep -- every
 * Extended-Accuracy opcode plus EMODH/POLYH -- collapses into a single logged line.  Reading the
 * log without expanding those runs undercounts a 157-event burst as one event, which presents as
 * "instances SIMH never dispatched" -- i.e. exactly like a native execution.  Expand them.
 */
function parseEvents(logPath, expandRepeats = true)
{
    let events = [];
    const EMUL = /CPU EMULFAULT:\s*(FPD )?OP=(\S+?), fault_PC=([0-9a-fA-F]+)/;
    const INTX = /CPU INTEXC: PC=([0-9a-fA-F]+),.*?VEC=([0-9a-fA-F]+)/;
    const RPT  = /same as above(?:\s*\((\d+) times\))?/;
    for (let line of fs.readFileSync(logPath, "latin1").split("\n")) {
        let m = EMUL.exec(line);
        if (m) {
            events.push({kind: m[1] ? "emulate-exception(FPD)" : "emulate-exception",
                         pc: parseInt(m[3], 16) | 0, mnemonic: m[2]});
            continue;
        }
        m = INTX.exec(line);
        if (m) {
            events.push({kind: `exception(SCB ${hex(parseInt(m[2], 16), 2)})`, pc: parseInt(m[1], 16) | 0});
            continue;
        }
        m = expandRepeats && RPT.exec(line);
        if (m) {
            let last = events[events.length - 1];
            if (!last) throw new Error(`${logPath}: "same as above" with no preceding event`);
            let n = m[1] ? +m[1] : 1;
            for (let i = 0; i < n; i++) events.push(last);
        }
    }
    return events;
}

function phaseTrace(simh, ehkaa, keepDir, scratch, cisMnemonics, opts = {})
{
    let tag = opts.tag || "";
    let dir = keepDir || scratch;
    let tracePath = path.join(dir, `cis_group_scope${tag}.trace`);
    let logPath = path.join(dir, `cis_group_scope${tag}.debug`);
    if (!fs.existsSync(tracePath) || !fs.existsSync(logPath)) {
        /* ONE run produces both, so the history and the disposition log describe the SAME
           execution.  Two runs would be two executions, and the profile's own §5 correction warns
           that EHKAA has timer-driven tests whose iteration count is not stable between runs. */
        try { fs.unlinkSync(tracePath); } catch (e) { /* may not exist */ }
        try { fs.unlinkSync(logPath); } catch (e) { /* may not exist */ }
        let script = ["set cpu 32m"];
        if (opts.instructionSet) script.push(`set cpu instruction=${opts.instructionSet}`);
        script.push(
            `set debug ${logPath}`,
            "set cpu debug=INTEXC;EMULFAULT",
            `set -d cpu history=100000:${tracePath}`,
            `load ${ehkaa}`,
            "go -q 200",
            "quit");
        runSimh(simh, script.join("\n") + "\n", scratch, `trace${tag}`);
    }
    if (!fs.existsSync(tracePath)) throw new Error(`SIMH produced no instruction history at ${tracePath}`);
    if (!fs.existsSync(logPath)) throw new Error(`SIMH produced no EMULFAULT/INTEXC debug log at ${logPath}`);

    let events = parseEvents(logPath, opts.expandRepeats !== false);
    let cis = new Set(cisMnemonics);
    let counts = new Map(), dispositions = new Map(), unresolved = [];
    let instances = [];
    let total = 0;

    /* Pass 1: every CIS record, in order, with whether control fell through to PC+ilen.  The
       record AFTER a CIS record is the one that answers that, so carry the pending record. */
    let pending = null;
    total = parseTrace(tracePath, (rec) => {
        if (pending) {
            instances.push({mnemonic: pending.mnemonic, pc: pending.pc, nextPC: rec.pc,
                            fell: (rec.pc | 0) === ((pending.pc + pending.ilen) | 0)});
            pending = null;
        }
        if (cis.has(rec.mnemonic)) pending = rec;
    });
    /* The FINAL record has no successor.  If it is a CIS instance, say so by name rather than
       letting it vanish -- the same "counted, never silently dropped" rule the sibling
       differentials apply to any case that cannot reach a comparison. */
    if (pending) {
        instances.push({mnemonic: pending.mnemonic, pc: pending.pc, nextPC: null, fell: null});
    }

    /* Pass 2: attribute each instance to SIMH's own event stream.  Events and instances are both
       in execution order, so a forward scan from the last match is correct even though the event
       stream also carries every UNRELATED exception the diagnostic takes. */
    let ei = 0;
    for (let inst of instances) {
        counts.set(inst.mnemonic, (counts.get(inst.mnemonic) || 0) + 1);
        let how = null;
        if (inst.fell === true) {
            how = "executed";                                        // the instruction really ran
        } else {
            for (let j = ei; j < events.length; j++) {
                if (events[j].pc === inst.pc) { how = events[j].kind; ei = j + 1; break; }
            }
            if (!how) {
                how = "unresolved";
                unresolved.push(`${inst.mnemonic} @${hex(inst.pc)}` +
                                (inst.nextPC === null ? " is the last record in the trace; no successor"
                                                      : ` transferred to ${hex(inst.nextPC)} with no matching EMULFAULT/INTEXC event`));
            }
        }
        let d = dispositions.get(inst.mnemonic) || {};
        d[how] = (d[how] || 0) + 1;
        dispositions.set(inst.mnemonic, d);
    }

    if (!keepDir) {
        try { fs.unlinkSync(tracePath); } catch (e) { /* best effort */ }
        try { fs.unlinkSync(logPath); } catch (e) { /* best effort */ }
    }
    return {total, counts, dispositions, unresolved, instances: instances.length, events: events.length};
}

/* ------------------------------------------------------------------------------------------- *
 * GATE -- the only measurement that settles what the milestone requires.                          *
 * ------------------------------------------------------------------------------------------- */

function runToHalt(simh, ehkaa, scratch, instructionSet, tag)
{
    let script = ["set cpu 32m"];
    if (instructionSet) script.push(`set cpu instruction=${instructionSet}`);
    script.push(`load ${ehkaa}`, "go 200", "quit");
    let out = runSimh(simh, script.join("\n") + "\n", scratch, tag);
    let m = /HALT instruction, PC:\s*([0-9A-Fa-f]+)/.exec(out);
    if (!m) throw new Error(`EHKAA (instruction=${instructionSet || "as shipped"}) never halted:\n${out}`);
    return parseInt(m[1], 16) | 0;
}

function phaseGate(simh, ehkaa, scratch)
{
    return {
        asShipped: runToHalt(simh, ehkaa, scratch, null, "gate-shipped"),
        nativeCIS: runToHalt(simh, ehkaa, scratch, "PACKED;EMULATED", "gate-native"),
        nativeEXTAC: runToHalt(simh, ehkaa, scratch, "EXTENDED", "gate-extac")
    };
}

/* ------------------------------------------------------------------------------------------- *
 * MAIN                                                                                            *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, dflt)
{
    let i = process.argv.indexOf(name);
    return (i >= 0 && i + 1 < process.argv.length) ? process.argv[i + 1] : dflt;
}

/* ------------------------------------------------------------------------------------------- *
 * SELFCHECK -- prove this computation would NOTICE if its finding stopped being true.             *
 *                                                                                                 *
 * A measurement whose conclusion is "nothing happened" is worthless unless it can be shown to     *
 * react when something does.  Both mutations below act on the REAL measurement path, not on a     *
 * mock: the first changes what the simulator does, the second removes a line of this file's own   *
 * parser.  Each must produce a failure; a mutation that survives is a hole in the computation.    *
 * ------------------------------------------------------------------------------------------- */

function selfcheck(simh, ehkaa, scratch, cisMnemonics)
{
    let mutations = [
        {
            name: "SIMH configured to IMPLEMENT the CIS groups natively",
            why: "the whole finding is 'zero instances execute'; if the computation cannot tell " +
                 "an executed instance from a faulted one it is measuring nothing",
            run: () => phaseTrace(simh, ehkaa, null, scratch, cisMnemonics,
                                  {tag: "-native", instructionSet: "PACKED;EMULATED"}),
            expect: (t) => [...t.dispositions.values()].some(d => (d.executed || 0) > 0),
            expectation: "at least one CIS instance reported as `executed`"
        },
        {
            name: "sim_debug run-collapse expansion disabled in parseEvents()",
            why: "SIMH folds a burst of identical events into one `same as above (k times)` line; " +
                 "a parser that misses that undercounts EHKAA's reserved-instruction sweep and " +
                 "reports real faults as instances SIMH never dispatched",
            run: () => phaseTrace(simh, ehkaa, null, scratch, cisMnemonics,
                                  {tag: "-norpt", expandRepeats: false}),
            expect: (t) => t.unresolved.length > 0,
            expectation: "at least one CIS instance reported as `unresolved`"
        }
    ];

    let survivors = [];
    for (let m of mutations) {
        let caught = false, detail = "";
        try {
            let t = m.run();
            caught = m.expect(t);
            detail = caught ? "" : `  expected ${m.expectation}, got dispositions ` +
                                   JSON.stringify(Object.fromEntries([...t.dispositions].sort()));
        } catch (e) {
            caught = true;                                  // a throw is a detection too
            detail = `  (detected by throwing: ${e.message})`;
        }
        console.log(`    ${caught ? "CAUGHT " : "SURVIVED"}  ${m.name}`);
        if (detail) console.log(detail);
        if (!caught) survivors.push(m.name);
    }
    return survivors;
}

function main()
{
    let asJson = process.argv.includes("--json");
    let check = process.argv.includes("--check");
    let simh = findSimh(getArg("--simh", null));
    let ehkaa = findEhkaa(getArg("--ehkaa", null));
    let scratch = fs.mkdtempSync(path.join(os.tmpdir(), "cis-scope-"));

    let byGroup, showv, trace, gate;
    try {
        byGroup = groupOpcodes(CIS_GROUPS);
        let cisMnemonics = [].concat(...byGroup.values()).sort();

        if (process.argv.includes("--selfcheck")) {
            console.log("SELFCHECK -- deliberate defects injected into the measurement path:");
            let survivors = selfcheck(simh, ehkaa, scratch, cisMnemonics);
            if (survivors.length) {
                console.log(`\n${survivors.length} mutation(s) SURVIVED -- this computation does not ` +
                            "prove what it claims.");
                process.exitCode = 1;
            } else {
                console.log("\nAll mutations caught.");
            }
            return {selfcheck: survivors};
        }
        showv = phaseShowV(simh, scratch);
        trace = phaseTrace(simh, ehkaa, getArg("--keep", null), scratch, cisMnemonics);
        gate = phaseGate(simh, ehkaa, scratch);

        let touched = [...trace.counts.keys()].sort();
        let untouched = cisMnemonics.filter(mn => !trace.counts.has(mn));
        let executed = touched.filter(mn => (trace.dispositions.get(mn).executed || 0) > 0);

        let failures = [];

        /* SHOWV: every CIS group must be reported by the binary as EMULATED, not implemented. */
        for (let name of CIS_GROUPS.values()) {
            let d = showv.disposition.get(name);
            if (d !== "Emulating") {
                failures.push(`SHOW CPU -V reports "${name}" as ${d || "(absent)"}, expected "Emulating"`);
            }
        }

        /* TRACE: every instance must be attributable, and none may have EXECUTED. */
        for (let u of trace.unresolved) failures.push(`unresolved CIS instance: ${u}`);
        for (let mn of executed) {
            failures.push(`${mn} has ${trace.dispositions.get(mn).executed} instance(s) that EXECUTED ` +
                          `natively -- this machine is not the KA655 the rest of the port models`);
        }

        /* GATE: as shipped EHKAA must PASS; with native CIS it must NOT. */
        if (gate.asShipped !== EHKAA_PASS_PC) {
            failures.push(`EHKAA as shipped halted at ${hex(gate.asShipped)}, not the documented PASS halt ${hex(EHKAA_PASS_PC)}`);
        }
        if (gate.nativeCIS === EHKAA_PASS_PC) {
            failures.push("EHKAA still PASSES with native CIS enabled -- the central finding of this " +
                          "computation no longer reproduces; re-derive the scope before trusting it");
        }

        let result = {
            groups: Object.fromEntries([...byGroup].map(([g, l]) => [CIS_GROUPS.get(g), l])),
            cisOpcodeTotal: cisMnemonics.length,
            simhDisposition: Object.fromEntries(showv.disposition),
            traceInstructions: trace.total,
            cisInstances: trace.instances,
            simhEvents: trace.events,
            touched, untouched,
            counts: Object.fromEntries([...trace.counts].sort()),
            dispositions: Object.fromEntries([...trace.dispositions].sort()),
            executedNatively: executed,
            unresolved: trace.unresolved,
            gate: {
                asShipped: hex(gate.asShipped),
                nativeCIS: hex(gate.nativeCIS),
                nativeEXTAC: hex(gate.nativeEXTAC),
                passPC: hex(EHKAA_PASS_PC),
                shippedPasses: gate.asShipped === EHKAA_PASS_PC,
                nativeCISPasses: gate.nativeCIS === EHKAA_PASS_PC,
                nativeEXTACPasses: gate.nativeEXTAC === EHKAA_PASS_PC
            },
            failures
        };

        if (asJson) {
            console.log(JSON.stringify(result, null, 2));
        } else {
            console.log(`SIMH:  ${simh}`);
            console.log(`EHKAA: ${ehkaa}`);
            console.log("");
            console.log("GROUPS (drom.js IG_PACKD | IG_EMONL)");
            for (let [g, list] of byGroup) {
                console.log(`    ${CIS_GROUPS.get(g)} (${list.length}): ${list.join(" ")}`);
            }
            console.log("");
            console.log("SHOW CPU -V (the binary's own report)");
            for (let [name, d] of showv.disposition) console.log(`    ${d.padEnd(12)} ${name}`);
            console.log("");
            console.log(`TRACE (${trace.total} instructions; mnemonic FIELD parse, not a substring grep)`);
            console.log(`    CIS opcodes issued:   ${touched.length} of ${cisMnemonics.length}`);
            console.log(`    CIS instances:        ${trace.instances}, against ${trace.events} SIMH EMULFAULT/INTEXC events`);
            console.log(`    never issued (${untouched.length}): ${untouched.join(" ") || "(none)"}`);
            console.log("");
            console.log("    Mnemonic   Count  Disposition (from SIMH's own EMULFAULT/INTEXC log)");
            for (let mn of touched) {
                let d = trace.dispositions.get(mn);
                let how = Object.entries(d).map(([k, v]) => `${k}=${v}`).join(" ");
                console.log(`    ${mn.padEnd(10)} ${String(trace.counts.get(mn)).padStart(5)}  ${how}`);
            }
            console.log("");
            console.log(`    EXECUTED natively: ${executed.length ? executed.join(" ") : "NONE -- every instance faulted or took the emulated-instruction exception"}`);
            console.log("");
            console.log("GATE (EHKAA run to halt)");
            console.log(`    as shipped                       -> ${hex(gate.asShipped)} ${gate.asShipped === EHKAA_PASS_PC ? "(PASS)" : "(NOT the PASS halt)"}`);
            console.log(`    SET CPU INSTRUCTION=PACKED;EMULATED -> ${hex(gate.nativeCIS)} ${gate.nativeCIS === EHKAA_PASS_PC ? "(PASS)" : "(NOT the PASS halt)"}`);
            console.log(`    SET CPU INSTRUCTION=EXTENDED        -> ${hex(gate.nativeEXTAC)} ${gate.nativeEXTAC === EHKAA_PASS_PC ? "(PASS)" : "(NOT the PASS halt)"}`);
            console.log("");
            if (failures.length) {
                console.log(`DRIFT (${failures.length}):`);
                for (let f of failures) console.log(`    ${f}`);
            } else {
                console.log(`No drift: SIMH emulates (does not implement) every CIS group, EHKAA executes`);
                console.log(`none of the ${touched.length} CIS opcodes it issues, and forcing native CIS breaks the gate.`);
            }
        }

        if (check && failures.length) process.exitCode = 1;
        return result;
    } finally {
        try { fs.rmSync(scratch, {recursive: true, force: true}); } catch (e) { /* best effort */ }
    }
}

const result = main();
export { groupOpcodes, parseTrace, parseEvents, CIS_GROUPS, EHKAA_PASS_PC, result };
