/**
 * @fileoverview Computes the residual of the VAX Base Instruction Group: every opcode drom.js
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
 *               marks as belonging to the group this CPU implements (IG_BASE | IG_BSGFL |
 *               IG_BSDFL -- see decode.js's `instructionSet`, vax_defs.h's VAX_FULL_BASE) that is
 *               NOT wired up by cpu.js, control.js, fpa.js or strq.js's own exported opcode lists.
 *
 * WHY THIS EXISTS
 * ----------------
 * The Base Instruction Group's membership is not hand-countable from a header comment -- prior
 * items (BISPSW/BICPSW/MOVPSL) show a stale "not implemented" note can drift out of sync with
 * what a sibling module actually wired up.  This script is the single, re-runnable source of
 * truth for "what is left": it reads the SAME exported lists each execution module's dispatch
 * itself uses (cpu.js's IMPLEMENTED, control.js's CONTROL_OPCODES keys resolved through drom.js's
 * OPCODES, fpa.js's OPC keys, strq.js's IMPLEMENTED, exc.js's IMPLEMENTED) and the SAME group classification decode.js
 * uses to decide whether this CPU implements an opcode at all, and prints the set difference.  No
 * hand-maintained mnemonic list lives here or anywhere else in this file.
 *
 * As of pcjsvax-515 the residual was exactly the 12 opcodes documented in strq.js's file header as
 * coordination carve-outs -- MTPR/MFPR and CHMK/CHME/CHMS/CHMU/BPT/XFC/REI/LDPCTX/SVPCTX/HALT, all
 * of which needed SCB exception dispatch and/or the privileged IPR register file.  pcjsvax-e49
 * delivered both, exc.js wired up all twelve, and **the residual is now empty**: the Base
 * Instruction Group is 242/242.
 *
 * `--check-carveouts` asserts the residual is EXACTLY `KNOWN_CARVEOUTS` and exits non-zero on any
 * drift in either direction -- an opcode that stops being implemented, or one that starts being
 * implemented without this list being updated.  Both directions have now fired for real: the list
 * went stale the moment exc.js closed the twelve, and this script failed until it was emptied.
 * That is the script working, not the script being wrong; leave the check armed.
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
import { IMPLEMENTED as EXC_IMPLEMENTED } from "../modules/v2/exc.js";

/*
 * The known, documented coordination carve-outs.  EMPTY as of pcjsvax-e49, which delivered the SCB
 * exception dispatch and the privileged IPR register file the last twelve were waiting on (MTPR/
 * MFPR, CHMK/CHME/CHMS/CHMU, BPT/XFC/REI/LDPCTX/SVPCTX/HALT -- see exc.js).  Not a hand-maintained
 * "done" list; used only to assert the computed residual hasn't silently grown or shrunk.
 *
 * Note what is NOT here and must never be added: packed-decimal/CIS.  Those opcodes are IG_PACKD/
 * IG_EMONL, not Base Group, so they never enter this residual at all -- and per
 * tests/cis_group_scope.js this machine must not implement them.
 */
const KNOWN_CARVEOUTS = new Set([]);

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
        ...STRQ_IMPLEMENTED,
        ...EXC_IMPLEMENTED
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
        console.log(`Implemented (cpu.js + control.js + fpa.js + strq.js + exc.js): ${base.size - residual.length} opcodes`);
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
        } else if (KNOWN_CARVEOUTS.size) {
            console.log(`--check-carveouts: residual matches the ${KNOWN_CARVEOUTS.size} documented carve-outs exactly.`);
        } else {
            console.log("--check-carveouts: residual is empty, as documented -- the Base Instruction Group is complete.");
        }
    }

    return residual;
}

const residual = main();
export { baseGroupOpcodes, controlImplemented, residual, KNOWN_CARVEOUTS };
