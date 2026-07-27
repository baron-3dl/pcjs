/**
 * @fileoverview Re-derives the MSCP opcode / status / dispatch scope from Open SIMH's own sources
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY THIS FILE EXISTS -- HANDOFF.md STANDING RULE 5
 * ---------------------------------------------------
 * "Never hand-enumerate a scope list.  Derive it programmatically and commit the computation."
 * That rule was earned here: this project's CIS opcode count went 7 -> 11 -> 17 -> 23, and EVERY
 * hand-derived value along the way was wrong.  modules/v2/rq.js carries three lists that would
 * otherwise be hand-enumerated:
 *
 *   OP           the 21 MSCP opcodes            (pdp11_mscp.h:42-62)
 *   ST           the 23 MSCP status codes       (pdp11_mscp.h:144-166)
 *   MSCP_UNIT_OPS + MSCP_NOP_OPS + MSCP_SCC_OP  the opcodes rq_mscp()'s switch has a case for
 *                                               (pdp11_rq.c, `t_bool rq_mscp`)
 *
 * This file extracts all three FROM THE C and compares them to what rq.js publishes.  It is not a
 * one-off: tests/mscpringdiff.js runs it as PHASE S on every invocation and FAILS the run on any
 * difference, so a vendor tree that grows an opcode makes the differential go red rather than
 * making the dispatch quietly fall through to the illegal-opcode default.
 *
 * The third list is the one that matters most, and it is the shape of HANDOFF.md standing rule 7:
 * an opcode the C dispatches to a real handler and rq.js does not know about would be answered with
 * ST_CMD|I_OPCD -- a plausible-looking response, in the right packet, at the right time, for the
 * wrong command.
 *
 *      node machines/dec/vax/tests/mscpscope.js [--print] [--simh PATH]
 *
 * It ALWAYS compares against rq.js and ALWAYS exits non-zero on any difference -- there is no
 * "extract without checking" mode, because a scope extraction whose result nothing asserts on is
 * the hand-curated list this file exists to replace.
 *        --print     also print the extracted tables
 *        --simh PATH a microvax3900 binary; its BIN/../PDP11 is where the sources are looked for
 */

import fs from "fs";
import path from "path";

import RQVAX from "../modules/v2/rq.js";
import { findSimhBin, vaxRepo } from "./mscpharness.js";

/**
 * sourceDir(simhPath)
 *
 * Where pdp11_mscp.h and pdp11_rq.c live.  TWO candidates, tried in order, because the two trees
 * this project uses are in different places and only one of them is guaranteed present:
 *   1. the tree the ORACLE WAS BUILT FROM -- `BIN/microvax3900` is `<tree>/BIN/microvax3900`, so the
 *      sources are `<tree>/PDP11`.  This is the right answer whenever there is an oracle at all,
 *      because it is by construction the same source the binary's behaviour came from.
 *   2. $PCJS_VAX_REPO/open-simh/PDP11 -- the vendor reference checkout.  HANDOFF.md 2 records that
 *      it is GITIGNORED, so it may legitimately be absent; that is why it is the fallback and not
 *      the primary.
 *
 * @param {?string} simhPath
 * @returns {string}
 */
export function sourceDir(simhPath)
{
    let cands = [];
    if (simhPath) cands.push(path.resolve(path.dirname(simhPath), "../PDP11"));
    cands.push(path.join(vaxRepo(), "open-simh/PDP11"));
    for (let d of cands) if (fs.existsSync(path.join(d, "pdp11_mscp.h"))) return d;
    throw new Error("mscpscope: cannot find pdp11_mscp.h.  It is needed to RE-DERIVE the MSCP\n" +
        "opcode and status tables rather than trust the ones transcribed into rq.js\n" +
        "(HANDOFF.md standing rule 5).  Tried:\n  " + cands.join("\n  "));
}

/**
 * defines(text, prefix)
 *
 * Every `#define <prefix>_<NAME> <value>` in the header, in source order, as {NAME: value}.  Values
 * are accepted in C's decimal and 0x forms; a shift-expression form (there is none in these two
 * runs) would be rejected loudly rather than silently skipped, which is the whole point.
 *
 * @param {string} text
 * @param {string} prefix
 * @returns {Object}
 */
export function defines(text, prefix)
{
    let out = {}, re = new RegExp(`^#define[ \\t]+${prefix}_([A-Z0-9_]+)[ \\t]+(\\S+)`, "gm"), m;
    while ((m = re.exec(text)) !== null) {
        let raw = m[2];
        if (!/^(0x[0-9A-Fa-f]+|\d+)$/.test(raw)) {
            throw new Error(`mscpscope: ${prefix}_${m[1]} is defined as "${raw}", which is not a ` +
                `plain integer -- the extraction would have to interpret C to get it right, and a ` +
                `silently skipped entry is exactly the hand-curated list this file exists to prevent`);
        }
        out[m[1]] = raw.startsWith("0x") ? parseInt(raw, 16) : parseInt(raw, 10);
    }
    return out;
}

/**
 * mscpSwitchOps(text)
 *
 * The OP_ names that appear as `case OP_x:` inside rq_mscp()'s own switch -- i.e. exactly the
 * opcodes the C dispatches somewhere other than the illegal-opcode default.  Sliced to the function
 * body first, so a `case OP_RD:` in rq_svc() or rq_abo() cannot contaminate the answer.
 *
 * @param {string} text
 * @returns {Array.<string>}
 */
export function mscpSwitchOps(text)
{
    /* The DEFINITION, not the forward declaration -- the two differ only by a trailing semicolon,
       and matching the wrong one yields an empty case list that would look like "the C dispatches
       nothing" rather than like a broken extraction. */
    let m0 = /^t_bool rq_mscp \(MSC \*cp, uint16 pkt, t_bool q\)$\n\{/m.exec(text);
    if (!m0) throw new Error("mscpscope: could not find rq_mscp()'s definition in pdp11_rq.c");
    let i = m0.index;
    let j = text.indexOf("\n}\n", i);
    if (j < 0) throw new Error("mscpscope: could not find the end of rq_mscp()");
    let body = text.slice(i, j);
    let out = [], re = /case OP_([A-Z0-9]+):/g, m;
    while ((m = re.exec(body)) !== null) out.push(m[1]);
    return out;
}

/**
 * extract(simhPath)
 * @param {?string} simhPath
 * @returns {{dir: string, op: Object, st: Object, switchOps: Array.<string>}}
 */
export function extract(simhPath)
{
    let dir = sourceDir(simhPath);
    /* CRLF STRIPPED ON READ.  The vendor checkout this project builds its oracle from has DOS line
       endings; an extraction anchored on "\n{" silently found nothing there and reported a scope of
       zero dispatch cases, which is indistinguishable from "the C dispatches nothing". */
    let rd = (f) => fs.readFileSync(path.join(dir, f), "utf8").replace(/\r/g, "");
    let h = rd("pdp11_mscp.h");
    let c = rd("pdp11_rq.c");
    return {dir, op: defines(h, "OP"), st: defines(h, "ST"), switchOps: mscpSwitchOps(c)};
}

/**
 * checkScope(simhPath)
 *
 * @param {?string} simhPath
 * @returns {{dir: string, nOp: number, nSt: number, nSwitch: number, failures: Array.<string>}}
 */
export function checkScope(simhPath)
{
    let e = extract(simhPath), failures = [];
    let cmp = (what, derived, shipped) => {
        for (let k of Object.keys(derived)) {
            if (!(k in shipped)) {
                failures.push(`PHASE S: the C defines ${what}_${k} = ${derived[k]} and rq.js has no ` +
                    `such ${what} code -- the shipped table is a hand-enumerated subset`);
            } else if (shipped[k] !== derived[k]) {
                failures.push(`PHASE S: ${what}_${k} is ${derived[k]} in the C and ${shipped[k]} in rq.js`);
            }
        }
        for (let k of Object.keys(shipped)) {
            if (!(k in derived)) {
                failures.push(`PHASE S: rq.js publishes ${what}_${k} = ${shipped[k]}, which the C does ` +
                    `not define -- an invented code answers a command the controller never sees`);
            }
        }
    };
    cmp("OP", e.op, RQVAX.OP);
    cmp("ST", e.st, RQVAX.ST);

    /* The dispatch.  Every `case OP_x:` in rq_mscp() must be classified by rq.js as either
       unit-bearing (excluded, throws) or a no-op arm, plus OP_SCC which is handled; and rq.js must
       not claim to classify an opcode the C's switch does not name. */
    let derived = new Set(e.switchOps);
    let shipped = new Set([...RQVAX.MSCP_UNIT_OPS, ...RQVAX.MSCP_NOP_OPS, RQVAX.MSCP_SCC_OP]);
    for (let n of derived) {
        if (!shipped.has(n)) {
            failures.push(`PHASE S: rq_mscp() has a \`case OP_${n}:\` and rq.js classifies OP_${n} ` +
                `nowhere, so it falls through to the ILLEGAL-OPCODE default and answers ` +
                `ST_CMD|I_OPCD for a command the controller really handles`);
        }
    }
    for (let n of shipped) {
        if (!derived.has(n)) {
            failures.push(`PHASE S: rq.js classifies OP_${n} as dispatched, but rq_mscp()'s switch ` +
                `has no case for it -- the C answers ST_CMD|I_OPCD and rq.js would not`);
        }
    }
    if (derived.size !== e.switchOps.length) {
        failures.push(`PHASE S: rq_mscp()'s switch names ${e.switchOps.length} cases but only ` +
            `${derived.size} distinct opcodes -- the extraction is reading something it should not`);
    }
    return {dir: e.dir, nOp: Object.keys(e.op).length, nSt: Object.keys(e.st).length,
            nSwitch: derived.size, failures, op: e.op, st: e.st, switchOps: e.switchOps};
}

function main()
{
    let i = process.argv.indexOf("--simh");
    let simh = null;
    try { simh = i >= 0 ? process.argv[i + 1] : findSimhBin(null); } catch (e) { simh = null; }
    let r = checkScope(simh);
    console.log(`mscpscope: derived from ${r.dir}`);
    console.log(`  ${r.nOp} OP_ codes, ${r.nSt} ST_ codes, ${r.nSwitch} rq_mscp() dispatch cases`);
    if (process.argv.includes("--print")) {
        console.log(`  OP: ${Object.keys(r.op).map((k) => `${k}=${r.op[k]}`).join(" ")}`);
        console.log(`  ST: ${Object.keys(r.st).map((k) => `${k}=${r.st[k]}`).join(" ")}`);
        console.log(`  dispatch: ${r.switchOps.join(" ")}`);
    }
    if (r.failures.length) {
        console.error(`\nFAIL -- ${r.failures.length} scope difference(s):`);
        for (let f of r.failures) console.error(`  ${f}`);
        process.exit(1);
    }
    console.log("\nOK -- rq.js's opcode, status and dispatch scope match the C exactly");
}

if (process.argv[1] && process.argv[1].endsWith("mscpscope.js")) main();
