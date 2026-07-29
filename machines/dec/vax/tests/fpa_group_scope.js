/**
 * @fileoverview Re-derives the floating-point opcode scope from Open SIMH's own source and grades
 *               this port's dispatch table against it, the way tests/base_group_residual.js does
 *               for the Base Instruction Group and tests/cis_group_scope.js does for CIS.
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY THIS EXISTS -- pcjsvax-040, filed by pcjsvax-486
 * -----------------------------------------------------
 * ACBF (opcode 0x4F) was implemented by NO module for the whole life of this project: control.js
 * excluded it for needing a floating add, fpa.js excluded it as "never executed" (measured against
 * EHKAA alone), and cpustate.js's buildDispatch() residue-claim loop -- the mechanism that hands
 * fpa.js every unclaimed BASE/BSGFL/BSDFL opcode -- skips IG_EMONL entirely, on purpose (see
 * cpustate.js's own file header: widening that condition would let fpa.js claim EMONL opcodes it
 * has no switch case for, which fails only at RUNTIME, not at load). So DISPATCH[0x4F] stayed
 * undefined, nothing in the 31-check gate noticed, and it surfaced only because the KA655 console
 * ROM's self-test 51 executes it ~3.2M instructions in and printed `?51`.
 *
 * tests/base_group_residual.js does not cover this gap BY CONSTRUCTION -- ACBF is IG_EMONL, outside
 * the Base Instruction Group, so it is out of that file's scope. tests/cis_group_scope.js covers
 * PACKD/EMONL from the OTHER side (whether the real SIMH BINARY implements or emulates them); it
 * says nothing about what THIS PORT dispatches. Nothing asserted the float groups' OWN scope.
 *
 * DERIVATION, NOT A HAND LIST (HANDOFF.md standing rule 5)
 * ---------------------------------------------------------
 * The candidate float scope is:
 *
 *   IG_BSGFL (21 opcodes) + IG_BSDFL (21 opcodes)   -- drom.js's own group classification, the SAME
 *                                                       table decode.js and cpustate.js's
 *                                                       buildDispatch() consult.
 *   + the IG_EMONL opcodes that are REAL F/D/G ARITHMETIC in vax_cpu.c, as opposed to CIS/string
 *     emulation (op_cis) or H_floating (op_octa).
 *
 * That last set is not guessable from the mnemonics -- ACBH LOOKS like it belongs with ACBD/ACBF/
 * ACBG, and EMODH/POLYH look like they belong with EMODD/F/G and POLYD/F/G, but vax_cpu.c:3163
 * dispatches ACBH/EMODH/POLYH to `op_octa()` (the SAME reserved-instruction path as every other
 * H_floating opcode) while ACBD/F/G, EMODD/F/G and POLYD/F/G reach real `op_add{f,d,g}`/
 * `op_emod{f,d,g}`/`op_poly{f,d,g}` bodies (vax_cpu.c:2986-3095). parseSwitchDispatch() below reads
 * that straight out of vax_cpu.c's own `switch (opc)` rather than trusting either grouping, and its
 * own coverage check (every one of drom.js's 304 defined opcodes must be classified) is what caught
 * the multi-label-per-line case ("case MOVP: case CMPP3: ...") the first draft of this parser missed.
 *
 * WHAT WAS MEASURED, TWICE, THIS SESSION (2026-07-29) -- carveouts below cite this, not a bare
 * assertion (done condition 2)
 * -------------------------------------------------------------------------------------------------
 * Two independent live runs of THIS PORT'S OWN JS engine (no SIMH rebuild needed for either -- the
 * ROM walk only needs rommachine.js and the EHKAA walk only needs cpustate.js + ehkaa.exe), each with
 * CPUStateVAX.prototype.executeOne wrapped to tally opcodes as they are actually fetched:
 *
 *   ROM WALK    rommachine.js's KA655 console ROM, 6,000,000 instructions (the budget pcjsvax-486's
 *               own probe used): ACBF executed 3 times; ACBD, ACBG, EMODD, EMODF, EMODG, POLYD,
 *               POLYF, POLYG executed ZERO times.
 *   EHKAA       the full diagnostic to its documented PASS halt (337,275 instructions, HALT at
 *               PC=80018AD1): ACBD, ACBF, ACBG, EMODD, EMODF, EMODG, POLYD, POLYF, POLYG all
 *               executed ZERO times. (EMODH/POLYH executed 3 times each and ACBH zero -- consistent
 *               with those three being H_floating/op_octa, not part of this file's candidate set.)
 *
 * This reconfirms pcjsvax-486's own recorded measurement ("over a 6M-instruction ROM walk, ACBF is
 * the ONLY unimplemented opcode executed... ACBD/ACBG/ACBH, EMODD/F/G and POLYD/F/G are never
 * executed by this ROM") and fpa.js's file header ("per docs/reference/ehkaa-profile.md §7... ACBD/
 * G/H, EMODD/F/G and POLYD/F/G are never executed either") against BOTH named workloads, independent
 * of either prior note.
 *
 * WHAT THIS FILE CHECKS, TWO DIRECTIONS (done condition 1)
 * -----------------------------------------------------------
 * "Implemented" and "dispatched" are different facts about different tables, and conflating them is
 * exactly how ACBF fell through: fpa.js's OPC table said what its own `execute()` switch handles;
 * cpustate.js's DISPATCH_OWNER says what the CPU loop can actually reach. This file cross-checks them
 * BOTH ways, over the WHOLE float surface (not just IG_EMONL):
 *
 *   assertDispatchedButUnimplemented()  every opcode cpustate.js's residue loop claims FOR fpa.js
 *                                       must have a real entry in fpa.js's own OPC table -- otherwise
 *                                       `cpu.fpu.execute()` returns false and throws the first time
 *                                       that opcode is actually fetched, which buildDispatch()'s own
 *                                       load-time coverage check CANNOT see (it only checks that
 *                                       *some* function is in the table slot, not that fpa.js's
 *                                       switch has a real case behind it).
 *   assertImplementedButUndispatched()  every mnemonic fpa.js's OPC table claims to implement must
 *                                       be owned by "fpa.js" in cpustate.js's DISPATCH_OWNER --
 *                                       otherwise the arithmetic exists and nothing can reach it,
 *                                       which is ACBF's ORIGINAL bug shape reproduced exactly.
 *   assertEmonlResidual()                the scope question base_group_residual.js/cis_group_scope.js
 *                                       already answer for their own groups: which of the candidate
 *                                       float opcodes are neither dispatched nor a documented,
 *                                       measured carveout.
 *
 * Usage:
 *     node machines/dec/vax/tests/fpa_group_scope.js
 *     node machines/dec/vax/tests/fpa_group_scope.js --json
 *     node machines/dec/vax/tests/fpa_group_scope.js --check       # exit non-zero on any drift
 *     node machines/dec/vax/tests/fpa_group_scope.js --selfcheck   # prove it would notice if wrong
 */

import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

import { DROM, DROM_STRIDE, DR, IG, OPCODES } from "../modules/v2/drom.js";
import { OPC as FPA_OPC } from "../modules/v2/fpa.js";
import { DISPATCH_OWNER } from "../modules/v2/cpustate.js";
import { vaxRepo } from "./mscpharness.js";
import { stripComments } from "./mscpscope.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/* ------------------------------------------------------------------------------------------- *
 * GROUP MEMBERSHIP -- straight out of the decode ROM, the SAME table cpustate.js consults.        *
 * ------------------------------------------------------------------------------------------- */

/** Every opcode drom.js classifies as belonging to `group`, as mnemonics. */
export function groupNames(group)
{
    let out = [];
    for (let opc = 0; opc < 512; opc++) {
        let hdr = DROM[opc * DROM_STRIDE];
        if (hdr === 0) continue;
        let g = (hdr >> DR.V_IGMASK) & DR.M_IGMASK;
        if (g !== group) continue;
        let mn = OPCODES[opc];
        if (!mn) throw new Error(`fpa_group_scope: opcode 0x${opc.toString(16)} is in group ${group} but drom.js has no mnemonic for it`);
        out.push(mn);
    }
    return out.sort();
}

/** Reverse of drom.js's OPCODES: mnemonic -> numeric opcode. Throws on a collision -- two opcodes
    sharing a mnemonic would make every table keyed by name ambiguous. */
export function opcodeOfTable()
{
    let out = {};
    for (let opc = 0; opc < 512; opc++) {
        let mn = OPCODES[opc];
        if (!mn) continue;
        if (mn in out) {
            throw new Error(`fpa_group_scope: mnemonic ${mn} names BOTH opcode 0x${out[mn].toString(16)} and 0x${opc.toString(16)} -- drom.js's OPCODES table is not one-to-one`);
        }
        out[mn] = opc;
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * vax_cpu.c's own switch (opc) -- WHICH FUNCTION does each opcode really reach?                    *
 * ------------------------------------------------------------------------------------------- */

function vaxCpuCPath(simhSrcDir)
{
    let candidates = [];
    if (simhSrcDir) candidates.push(path.join(simhSrcDir, "vax_cpu.c"));
    candidates.push(path.join(vaxRepo(), "open-simh/VAX/vax_cpu.c"));
    for (let p of candidates) if (fs.existsSync(p)) return p;
    throw new Error("fpa_group_scope: cannot find vax_cpu.c.  It is needed to RE-DERIVE which " +
        "opcodes reach real F/D/G arithmetic versus op_cis/op_octa (HANDOFF.md standing rule 5) " +
        "rather than trust a hand-typed split of ACB/EMOD/POLY by suffix letter.  Tried:\n  " +
        candidates.join("\n  "));
}

/**
 * parseSwitchDispatch(text)
 *
 * Every `case NAME:` inside `sim_instr()`'s big `switch (opc)`, mapped to the FIRST `op_xxx(...)` or
 * `cpu_emulate_exception(...)` call reached before that case group's `break;`. Comments are blanked
 * (not removed -- stripComments() preserves newlines so brace-depth counting below still lines up)
 * before ANY of this runs, so a case label mentioned only in prose cannot be misread as code.
 *
 * TWO SHAPES THIS FILE'S FIRST DRAFT GOT WRONG, both caught by the coverage check at the end:
 *   - MULTIPLE `case X:` labels on ONE LINE (`case MOVP: case CMPP3: case CMPP4: case CVTLP:`) --
 *     a parser that captures only the first per line silently drops the rest.
 *   - The call is not on the SAME line as the label; several blank/comment lines and multi-line
 *     case groups separate them, so the scan must carry PENDING labels forward until it finds a
 *     call, exactly like mscpscope.js's mscpDispatch() does for rq_mscp()'s C switch.
 *
 * @param {string} text  vax_cpu.c, as read from disk
 * @returns {Object} {NAME: "op_xxx"} for every case label in the switch
 */
export function parseSwitchDispatch(text)
{
    let clean = stripComments(text.replace(/\r/g, ""));
    let m0 = /switch \(opc\) \{/.exec(clean);
    if (!m0) throw new Error("fpa_group_scope: could not find sim_instr()'s `switch (opc) {` in vax_cpu.c");
    let openIdx = m0.index + m0[0].length - 1;              // index of the switch's own '{'
    let depth = 0, i = openIdx;
    for (; i < clean.length; i++) {
        if (clean[i] === "{") depth++;
        else if (clean[i] === "}") { depth--; if (depth === 0) break; }
    }
    if (depth !== 0) throw new Error("fpa_group_scope: switch (opc) { ... } is never closed in vax_cpu.c");
    let body = clean.slice(openIdx + 1, i);

    const CASE_RE = /\bcase ([A-Za-z0-9_]+):/g;
    const ALL_CASE_LINE = /^(case [A-Za-z0-9_]+:\s*)+$/;
    const CALL_RE = /\b(op_[a-z0-9_]+|cpu_emulate_exception)\s*\(/;

    let dispatch = {}, pending = [], nLabels = 0;
    for (let raw of body.split("\n")) {
        let t = raw.trim();
        if (!t) continue;
        if (ALL_CASE_LINE.test(t)) {
            for (let c of t.matchAll(CASE_RE)) { pending.push(c[1]); nLabels++; }
            continue;
        }
        if (!pending.length) {
            if (/^default:/.test(t)) pending = [];
            continue;
        }
        let call = CALL_RE.exec(t);
        if (call) {
            for (let n of pending) dispatch[n] = call[1];
            pending = [];
        }
    }
    if (pending.length) {
        throw new Error(`fpa_group_scope: case(s) [${pending.join(", ")}] reach neither an op_xxx(...) ` +
            `call nor cpu_emulate_exception(...) before the switch ends -- the extraction lost the ` +
            `end of the group, and an opcode with no handler recorded is one this check cannot police`);
    }

    /* SELF-VERIFYING: every opcode drom.js defines must have been classified.  This is what caught
       the multi-label-per-line bug during development -- nLabels was short of drom.js's 304 defined
       opcodes until ALL_CASE_LINE captured every label on a line, not just the first. */
    let definedCount = 0;
    for (let opc = 0; opc < 512; opc++) if (OPCODES[opc]) definedCount++;
    if (nLabels !== definedCount || Object.keys(dispatch).length !== definedCount) {
        throw new Error(`fpa_group_scope: vax_cpu.c's switch classifies ${Object.keys(dispatch).length} ` +
            `distinct opcode(s) (${nLabels} case labels) but drom.js defines ${definedCount} -- this ` +
            `parser is not reading the whole switch, and every conclusion drawn from it is unverified`);
    }
    return dispatch;
}

/**
 * The C function FAMILIES that mean "real F/D/G floating arithmetic", as opposed to:
 *   op_cis     packed-decimal / CIS string emulation (cpu_emulate_exception via op_cis)
 *   op_octa    H_floating / octaword -- reserved-instruction fault when VAX_EXTAC is absent, which
 *              is every VAX this project models.  ACBH/EMODH/POLYH are IG_EMONL by drom.js's group
 *              classification but reach op_octa exactly like every other H_floating opcode -- see
 *              this file's header for why that is not guessable from the mnemonic.
 *
 * These are C ENTRY-POINT NAMES, not opcode mnemonics -- classifying by the function actually
 * reached is the same technique tests/mscpscope.js's mscpDispatch() uses (HANDLER NAMES, not a
 * hand list of opcodes) and is far more stable than pattern-matching on mnemonic suffixes.
 */
export const FLOAT_ARITH_FUNCS = new Set([
    "op_addf", "op_addd", "op_addg",                 // ACBF / ACBD / ACBG (add, then compare/branch)
    "op_emodf", "op_emodd", "op_emodg",               // EMODF / EMODD / EMODG
    "op_polyf", "op_polyd", "op_polyg"                // POLYF / POLYD / POLYG
]);

/* ------------------------------------------------------------------------------------------- *
 * The documented, MEASURED carveouts (see file header for the two 2026-07-29 measurements).      *
 * ------------------------------------------------------------------------------------------- */

/**
 * ACBF is deliberately NOT here: pcjsvax-486 implemented it in control.js (the loop/branch half)
 * calling fpa.js's opAddf()/opCmpfd() (the arithmetic half), so it is fully dispatched and this
 * residual must NOT include it. The remaining eight are carved out on the MEASURED evidence in this
 * file's header, not on "EHKAA never executes it" alone (HANDOFF.md standing rule 12 -- that exact
 * sentence, measured against EHKAA ALONE, is what let ACBF fall through in the first place).
 */
export const KNOWN_CARVEOUTS = new Set(["ACBD", "ACBG", "EMODD", "EMODF", "EMODG", "POLYD", "POLYF", "POLYG"]);

/* ------------------------------------------------------------------------------------------- *
 * The two cross-checks between "implemented" (fpa.js's OPC table) and "dispatched"                *
 * (cpustate.js's DISPATCH_OWNER).                                                                  *
 * ------------------------------------------------------------------------------------------- */

/**
 * Every opcode cpustate.js's buildDispatch() residue loop assigned to "fpa.js" must have a REAL
 * entry in fpa.js's own OPC table -- otherwise `cpu.fpu.execute()` returns false for it and THROWS
 * at runtime the first time it is fetched, a failure buildDispatch()'s own load-time coverage check
 * cannot see (it only asserts the table SLOT is non-null, which the wrapping closure always is).
 *
 * @param {Array} dispatchOwner  cpustate.js's DISPATCH_OWNER (or a perturbed copy, for --selfcheck)
 * @param {Object} fpaOpc        fpa.js's OPC (or a perturbed copy)
 * @returns {Array.<string>} failures
 */
export function assertDispatchedButUnimplemented(dispatchOwner, fpaOpc)
{
    let failures = [];
    for (let opc = 0; opc < dispatchOwner.length; opc++) {
        if (dispatchOwner[opc] !== "fpa.js") continue;
        let mn = OPCODES[opc];
        if (fpaOpc[mn] !== opc) {
            failures.push(`opcode 0x${opc.toString(16).toUpperCase()} (${mn || "?"}) is DISPATCHED to ` +
                `fpa.js by cpustate.js's residue claim, but fpa.js's OPC table ` +
                (mn in fpaOpc ? `maps ${mn} to a DIFFERENT opcode (0x${fpaOpc[mn].toString(16).toUpperCase()})`
                               : `has no entry for ${mn} at all`) +
                ` -- fpu.execute() will return false and THROW the first time this opcode is fetched`);
        }
    }
    return failures;
}

/**
 * Every mnemonic fpa.js's OPC table claims to implement must be owned by "fpa.js" in cpustate.js's
 * DISPATCH_OWNER -- otherwise the arithmetic exists and the CPU loop can never reach it. This is
 * ACBF's ORIGINAL bug reproduced exactly: fpa.js excluding a floating add for "never executed" while
 * nothing claimed the dispatch slot.
 *
 * @param {Array} dispatchOwner
 * @param {Object} fpaOpc
 * @returns {Array.<string>} failures
 */
export function assertImplementedButUndispatched(dispatchOwner, fpaOpc)
{
    let failures = [];
    for (let mn in fpaOpc) {
        let opc = fpaOpc[mn];
        let owner = dispatchOwner[opc];
        if (owner !== "fpa.js") {
            failures.push(`fpa.js's OPC table maps ${mn} to opcode 0x${opc.toString(16).toUpperCase()}, ` +
                `but cpustate.js's DISPATCH_OWNER for that opcode is ` +
                (owner ? `"${owner}"` : "unclaimed") +
                ` -- this is ACBF's original bug shape: arithmetic exists in fpa.js but the CPU loop ` +
                `can never reach it`);
        }
    }
    return failures;
}

/**
 * The scope question itself: of the candidate float opcodes, which are neither dispatched (by ANY
 * module) nor a documented carveout -- in EITHER direction, exactly like
 * base_group_residual.js --check-carveouts.
 *
 * @param {Array.<string>} candidateNames
 * @param {Array} dispatchOwner
 * @param {Object} opcodeOf        mnemonic -> numeric opcode
 * @param {Set.<string>} carveouts
 * @returns {Object} {residual, unexpected, missing}
 */
export function residualAgainstCarveouts(candidateNames, dispatchOwner, opcodeOf, carveouts)
{
    let residual = candidateNames.filter((mn) => {
        let opc = opcodeOf[mn];
        if (opc === undefined) throw new Error(`fpa_group_scope: candidate ${mn} has no numeric opcode`);
        return !dispatchOwner[opc];
    }).sort();
    let residualSet = new Set(residual);
    let unexpected = residual.filter((mn) => !carveouts.has(mn));
    let missing = [...carveouts].filter((mn) => !residualSet.has(mn));
    return {residual, unexpected, missing};
}

/* ------------------------------------------------------------------------------------------- *
 * The candidate scope, assembled.                                                                  *
 * ------------------------------------------------------------------------------------------- */

export function deriveCandidate(simhSrcDir)
{
    let cPath = vaxCpuCPath(simhSrcDir);
    let cDispatch = parseSwitchDispatch(fs.readFileSync(cPath, "utf8"));

    let bsgfl = groupNames(IG.BSGFL);
    let bsdfl = groupNames(IG.BSDFL);
    let emonl = groupNames(IG.EMONL);

    let emonlHandler = {};
    for (let mn of emonl) {
        if (!(mn in cDispatch)) throw new Error(`fpa_group_scope: ${mn} is IG_EMONL but vax_cpu.c's switch never classified it`);
        emonlHandler[mn] = cDispatch[mn];
    }
    for (let fn of FLOAT_ARITH_FUNCS) {
        if (!Object.values(emonlHandler).includes(fn)) {
            throw new Error(`fpa_group_scope: no IG_EMONL opcode maps to ${fn}() -- parseSwitchDispatch() ` +
                `is not reading vax_cpu.c correctly, or vax_cpu.c has been restructured and this file's ` +
                `FLOAT_ARITH_FUNCS needs review`);
        }
    }
    let emonlFloat = emonl.filter((mn) => FLOAT_ARITH_FUNCS.has(emonlHandler[mn])).sort();

    return {cPath, bsgfl, bsdfl, emonl, emonlHandler, emonlFloat,
            candidate: [...new Set([...bsgfl, ...bsdfl, ...emonlFloat])].sort()};
}

/* ------------------------------------------------------------------------------------------- *
 * SELFCHECK -- prove this computation would NOTICE if its finding stopped being true.              *
 *                                                                                                    *
 * Every mutation PERTURBS a COPY of the REAL, imported data (DISPATCH_OWNER / FPA_OPC) and re-runs *
 * the SAME shipped comparison functions above -- composing over the original (HANDOFF.md standing   *
 * rule 11), never substituting a private re-implementation of the check.                            *
 * ------------------------------------------------------------------------------------------- */

function selfcheck(candidate, opcodeOf)
{
    let mutations = [
        {
            name: "ACBF removed from the dispatch table (DISPATCH_OWNER[0x4F] deleted)",
            why: "the ORIGINAL bug: an opcode owned by no module. If deleting one entry from the " +
                 "real dispatch-ownership table does not turn up as an unexpected residual, this " +
                 "check would not have caught ACBF either",
            run: () => {
                let owner = DISPATCH_OWNER.slice();
                owner[0x4F] = null;
                return residualAgainstCarveouts(candidate, owner, opcodeOf, KNOWN_CARVEOUTS);
            },
            expect: (r) => r.unexpected.includes("ACBF"),
            expectation: "ACBF reported as an UNEXPECTED residual opcode"
        },
        {
            name: "fpa.js OPC table loses ADDF2 (fpa.js claims the opcode but declines to implement it)",
            why: "cpustate.js's residue loop wraps fpa.js's claim in a closure that always fills the " +
                 "table slot, so a missing switch case inside fpa.js is invisible to buildDispatch()'s " +
                 "own load-time check and only throws the first time ADDF2 is fetched",
            run: () => {
                let opc = FPA_OPC.ADDF2;
                let opcCopy = {...FPA_OPC};
                delete opcCopy.ADDF2;
                return assertDispatchedButUnimplemented(DISPATCH_OWNER, opcCopy).map((f) => ({f, opc}));
            },
            expect: (r) => r.some((x) => x.f.includes("0x40") || x.f.includes("ADDF2")),
            expectation: "opcode 0x40 (ADDF2) reported as dispatched-but-unimplemented"
        },
        {
            name: "fpa.js OPC table GAINS ACBD (implemented in fpa.js, never wired into the dispatch)",
            why: "this is ACBF's exact original shape, reproduced: arithmetic exists in fpa.js's " +
                 "switch but cpustate.js's residue loop never claims IG_EMONL opcodes, so the CPU " +
                 "loop can never reach it",
            run: () => {
                let opcCopy = {...FPA_OPC, ACBD: opcodeOf.ACBD};
                return assertImplementedButUndispatched(DISPATCH_OWNER, opcCopy);
            },
            expect: (r) => r.some((f) => f.includes("ACBD")),
            expectation: "ACBD reported as implemented-but-undispatched"
        }
    ];

    let survivors = [];
    for (let m of mutations) {
        let caught = false, detail = "";
        try {
            let r = m.run();
            caught = m.expect(r);
            detail = caught ? "" : `  expected ${m.expectation}, got ${JSON.stringify(r)}`;
        } catch (e) {
            caught = true;
            detail = `  (detected by throwing: ${e.message})`;
        }
        console.log(`    ${caught ? "CAUGHT " : "SURVIVED"}  ${m.name}`);
        if (detail) console.log(detail);
        if (!caught) survivors.push(m.name);
    }
    return survivors;
}

/* ------------------------------------------------------------------------------------------- *
 * MAIN                                                                                              *
 * ------------------------------------------------------------------------------------------- */

function getArg(name, dflt)
{
    let i = process.argv.indexOf(name);
    return (i >= 0 && i + 1 < process.argv.length) ? process.argv[i + 1] : dflt;
}

function main()
{
    let asJson = process.argv.includes("--json");
    let check = process.argv.includes("--check");
    let opcodeOf = opcodeOfTable();
    let d = deriveCandidate(getArg("--simh-src", null));

    if (process.argv.includes("--selfcheck")) {
        console.log("SELFCHECK -- deliberate defects injected into copies of the real dispatch/OPC data:");
        let survivors = selfcheck(d.candidate, opcodeOf);
        if (survivors.length) {
            console.log(`\n${survivors.length} mutation(s) SURVIVED -- this computation does not prove what it claims.`);
            process.exitCode = 1;
        } else {
            console.log("\nAll mutations caught.");
        }
        return {selfcheck: survivors};
    }

    let dispatchedUnimplemented = assertDispatchedButUnimplemented(DISPATCH_OWNER, FPA_OPC);
    let implementedUndispatched = assertImplementedButUndispatched(DISPATCH_OWNER, FPA_OPC);
    let {residual, unexpected, missing} = residualAgainstCarveouts(d.candidate, DISPATCH_OWNER, opcodeOf, KNOWN_CARVEOUTS);

    let failures = [...dispatchedUnimplemented, ...implementedUndispatched];
    for (let mn of unexpected) failures.push(`UNEXPECTED residual: ${mn} is a candidate float opcode, dispatched by no module, and not in KNOWN_CARVEOUTS`);
    for (let mn of missing) failures.push(`MISSING carveout: ${mn} is in KNOWN_CARVEOUTS but is actually dispatched -- the carveout list is stale`);

    let result = {
        source: d.cPath,
        bsgfl: d.bsgfl, bsdfl: d.bsdfl, emonl: d.emonl, emonlHandler: d.emonlHandler,
        emonlFloat: d.emonlFloat, candidate: d.candidate,
        candidateTotal: d.candidate.length,
        dispatchedTotal: d.candidate.length - residual.length,
        residual, knownCarveouts: [...KNOWN_CARVEOUTS].sort(),
        failures
    };

    if (asJson) {
        console.log(JSON.stringify(result, null, 2));
    } else {
        console.log(`fpa_group_scope: derived from ${d.cPath}`);
        console.log(`  IG_BSGFL: ${d.bsgfl.length}  IG_BSDFL: ${d.bsdfl.length}  IG_EMONL: ${d.emonl.length} total, ${d.emonlFloat.length} F/D/G arithmetic`);
        console.log(`  IG_EMONL dispatch, from vax_cpu.c's own switch:`);
        for (let mn of d.emonl) {
            console.log(`    ${mn.padEnd(8)} -> ${d.emonlHandler[mn].padEnd(14)}` +
                (FLOAT_ARITH_FUNCS.has(d.emonlHandler[mn]) ? "  (float arithmetic -- candidate)" : ""));
        }
        console.log(`  candidate float scope: ${d.candidate.length} opcodes`);
        console.log(`  dispatched (by any module): ${d.candidate.length - residual.length}`);
        console.log(`  residual (${residual.length}): ${residual.join(" ") || "(none)"}`);
        if (!unexpected.length && !missing.length) {
            console.log(`  residual matches the ${KNOWN_CARVEOUTS.size} documented, measured carveouts exactly`);
        }
        if (failures.length) {
            console.log(`\nDRIFT (${failures.length}):`);
            for (let f of failures) console.log(`    ${f}`);
        } else {
            console.log(`\nNo drift: every candidate float opcode is either dispatched or a documented, measured carveout,`);
            console.log(`and every fpa.js OPC entry is claimed by exactly the opcode cpustate.js dispatches it to.`);
        }
    }

    if (check && failures.length) process.exitCode = 1;
    return result;
}

if (process.argv[1] && process.argv[1].endsWith("fpa_group_scope.js")) main();

export { main };
