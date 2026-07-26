/**
 * @fileoverview Computes the residual of the VAX Base Instruction Group: every opcode drom.js
 *               marks as belonging to the group this CPU implements (IG_BASE | IG_BSGFL |
 *               IG_BSDFL -- see decode.js's `instructionSet`, vax_defs.h's VAX_FULL_BASE) that is
 *               NOT wired up by cpu.js, control.js, fpa.js or strq.js's own exported opcode lists.
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY THIS EXISTS
 * ----------------
 * The Base Instruction Group's membership is not hand-countable from a header comment -- prior
 * items (BISPSW/BICPSW/MOVPSL) show a stale "not implemented" note can drift out of sync with
 * what a sibling module actually wired up.  This script is the single, re-runnable source of
 * truth for "what is left": it reads the SAME exported lists each execution module's dispatch
 * itself uses (cpu.js's IMPLEMENTED, control.js's CONTROL_OPCODES keys resolved through drom.js's
 * OPCODES, fpa.js's OPC keys, strq.js's IMPLEMENTED) and the SAME group classification decode.js
 * uses to decide whether this CPU implements an opcode at all, and prints the set difference.  No
 * hand-maintained mnemonic list lives here or anywhere else in this file.
 *
 * As of pcjsvax-515, the residual is exactly the 12 opcodes documented in strq.js's file header
 * as coordination carve-outs -- MTPR/MFPR (pcjsvax-e49) and CHMK/CHME/CHMS/CHMU/BPT/XFC/REI/
 * LDPCTX/SVPCTX/HALT (all need SCB exception dispatch and/or the privileged IPR register file,
 * also pcjsvax-e49's).  `--check-carveouts` asserts the residual is EXACTLY that set and exits
 * non-zero on any drift (an opcode silently added or removed from the carve-out list without
 * updating both this script and strq.js's header is exactly the kind of drift this script exists
 * to catch).
 *
 * Usage:
 *     node machines/dec/vax/tests/base_group_residual.js
 *     node machines/dec/vax/tests/base_group_residual.js --json
 *     node machines/dec/vax/tests/base_group_residual.js --check-carveouts
 */

import { DROM, DR, IG, OPCODES } from "../modules/v2/drom.js";
import { IMPLEMENTED as CPU_IMPLEMENTED } from "../modules/v2/cpu.js";
import { CONTROL_OPCODES } from "../modules/v2/control.js";
import { OPC as FPA_OPC } from "../modules/v2/fpa.js";
import { IMPLEMENTED as STRQ_IMPLEMENTED } from "../modules/v2/strq.js";

/*
 * The known, documented coordination carve-outs as of pcjsvax-515 -- see strq.js's file header
 * for the full reasoning behind each.  Not a hand-maintained "done" list; used only to assert the
 * computed residual hasn't silently grown or shrunk.
 */
const KNOWN_CARVEOUTS = new Set([
    "MTPR", "MFPR",                                    // pcjsvax-e49: IPR register file
    "CHMK", "CHME", "CHMS", "CHMU",                     // pcjsvax-e49: SCB exception dispatch
    "BPT", "XFC", "REI", "LDPCTX", "SVPCTX", "HALT"     // pcjsvax-e49: SCB dispatch and/or IPRs
]);

const DROM_STRIDE = 7;

/*
 * The full base group this CPU implements, per decode.js's own `instructionSet` gate --
 * IG_BASE, IG_BSGFL (G-float subgroup) and IG_BSDFL (D-float subgroup).  Any other group
 * (PACKD, EXTAC, EMONL, VECTR, RSVD) is out of scope for this CPU entirely, not merely
 * unimplemented -- decode.js itself faults those as reserved/emulated instructions.
 */
const BASE_GROUPS = new Set([IG.BASE, IG.BSGFL, IG.BSDFL]);

function baseGroupOpcodes()
{
    let set = new Set();
    for (let opc = 0; opc < 512; opc++) {
        let hdr = DROM[opc * DROM_STRIDE];
        if (hdr === 0) continue;                       // undefined slot
        let group = (hdr >> DR.V_IGMASK) & DR.M_IGMASK;
        if (BASE_GROUPS.has(group)) {
            let mn = OPCODES[opc];
            if (!mn) throw new Error(`opcode 0x${opc.toString(16)} is base-group but has no mnemonic`);
            set.add(mn);
        }
    }
    return set;
}

function controlImplemented()
{
    let set = new Set();
    for (let opc in CONTROL_OPCODES) {
        let mn = OPCODES[opc];
        if (!mn) throw new Error(`control.js opcode ${opc} has no mnemonic in drom.js`);
        set.add(mn);
    }
    return set;
}

function main()
{
    let base = baseGroupOpcodes();
    let implemented = new Set([
        ...CPU_IMPLEMENTED,
        ...controlImplemented(),
        ...Object.keys(FPA_OPC),
        ...STRQ_IMPLEMENTED
    ]);

    let residual = [...base].filter(mn => !implemented.has(mn)).sort();
    let asJson = process.argv.includes("--json");
    let checkCarveouts = process.argv.includes("--check-carveouts");

    if (asJson) {
        console.log(JSON.stringify({
            baseGroupTotal: base.size,
            implementedTotal: [...base].filter(mn => implemented.has(mn)).length,
            residual
        }, null, 2));
    } else {
        console.log(`Base Instruction Group (IG_BASE|IG_BSGFL|IG_BSDFL): ${base.size} opcodes`);
        console.log(`Implemented (cpu.js + control.js + fpa.js + strq.js): ${base.size - residual.length} opcodes`);
        console.log(`Residual (${residual.length}):`);
        for (let mn of residual) console.log(`    ${mn}`);
    }

    if (checkCarveouts) {
        let residualSet = new Set(residual);
        let unexpected = residual.filter(mn => !KNOWN_CARVEOUTS.has(mn));
        let missing = [...KNOWN_CARVEOUTS].filter(mn => !residualSet.has(mn));
        if (unexpected.length || missing.length) {
            if (unexpected.length) console.error(`UNEXPECTED residual opcodes (not a known carve-out): ${unexpected.join(", ")}`);
            if (missing.length) console.error(`MISSING carve-outs (documented but not actually residual): ${missing.join(", ")}`);
            process.exitCode = 1;
        } else {
            console.log(`--check-carveouts: residual matches the ${KNOWN_CARVEOUTS.size} documented carve-outs exactly.`);
        }
    }

    return residual;
}

const residual = main();
export { baseGroupOpcodes, controlImplemented, residual, KNOWN_CARVEOUTS };
