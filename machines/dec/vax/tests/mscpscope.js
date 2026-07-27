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
 *   MSCP_OP_HANDLER                             the opcodes rq_mscp()'s switch has a case for AND
 *                                               the C function each one is dispatched to
 *                                               (pdp11_rq.c, `t_bool rq_mscp`)
 *   DRV_TAB      the 34 drive types             (pdp11_rq.c's drv_tab[] and its RQ_DRV macro)
 *
 * This file extracts all four FROM THE C and compares them to what rq.js publishes.  It is not a
 * one-off: tests/mscpringdiff.js and tests/mscpunitdiff.js run it as PHASE S on every invocation and
 * FAIL the run on any difference, so a vendor tree that grows an opcode or a drive type makes the
 * differential go red rather than making the dispatch quietly fall through to the illegal-opcode
 * default or a geometry silently disagree.
 *
 * The third list is the one that matters most, and it is the shape of HANDOFF.md standing rule 7:
 * an opcode the C dispatches to a real handler and rq.js does not know about would be answered with
 * ST_CMD|I_OPCD -- a plausible-looking response, in the right packet, at the right time, for the
 * wrong command.  It carries the HANDLER NAME and not merely the opcode, because pcjsvax-f52 splits
 * rq_mscp()'s unit-bearing arms into three classes -- implemented here, needs an in-flight transfer,
 * IS a transfer -- and a class assignment nothing checks is a hand-curated list again.
 *
 * The fourth is pcjsvax-f52's: drv_tab[] has THIRTY-FOUR entries of FOURTEEN fields each, built by
 * a token-pasting macro out of ~480 #defines.  Hand-transcribing that is precisely what standing
 * rule 5 forbids, and the fields are not decoration: `mod` and `MediaId` go out in every ONLINE and
 * GET UNIT STATUS response, `sect`/`tpg`/`gpc`/`rcts` are the geometry GET UNIT STATUS reports, and
 * `flgs` decides UF_RMV and UF_WPH.
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
 * cint(raw)
 *
 * ONE C integer literal.  *** A LEADING ZERO IS OCTAL IN C AND DECIMAL IN parseInt(). ***  That is
 * not hypothetical here: RQDF_DSSI is `010`, which is 8 and not 10, and reading it decimally makes
 * every DSSI drive's flag word wrong in a way that changes nothing visible until an RF-series unit
 * is asked for its removability.  Returns null for anything that is not a plain literal, so callers
 * can decide whether to resolve it further or reject it.
 *
 * @param {string} raw
 * @returns {?number}
 */
export function cint(raw)
{
    if (/^0[xX][0-9A-Fa-f]+$/.test(raw)) return parseInt(raw.slice(2), 16);
    if (/^0[0-7]+$/.test(raw)) return parseInt(raw.slice(1), 8);
    if (/^\d+$/.test(raw)) return parseInt(raw, 10);
    return null;
}

/**
 * defines(text, prefix)
 *
 * Every `#define <prefix>_<NAME> <value>` in the header, in source order, as {NAME: value}.  Values
 * are accepted in C's decimal, octal and 0x forms; a shift-expression form (there is none in these
 * two runs) is rejected loudly rather than silently skipped, which is the whole point.
 *
 * @param {string} text
 * @param {string} prefix
 * @returns {Object}
 */
export function defines(text, prefix)
{
    let out = {}, re = new RegExp(`^#define[ \\t]+${prefix}_([A-Z0-9_]+)[ \\t]+(\\S+)`, "gm"), m;
    while ((m = re.exec(text)) !== null) {
        let v = cint(m[2]);
        if (v === null) {
            throw new Error(`mscpscope: ${prefix}_${m[1]} is defined as "${m[2]}", which is not a ` +
                `plain integer -- the extraction would have to interpret C to get it right, and a ` +
                `silently skipped entry is exactly the hand-curated list this file exists to prevent`);
        }
        out[m[1]] = v;
    }
    return out;
}

/* ------------------------------------------------------------------------------------------- *
 * drv_tab[] -- pcjsvax-f52                                                                      *
 * ------------------------------------------------------------------------------------------- */

/** The fourteen struct drvtyp members, IN DECLARATION ORDER, and the `#define <TYPE>_<SUFFIX>` each
    one is pasted from by the RQ_DRV macro.  The order matters: RQ_DRV is a positional initialiser,
    so a member read out of order would silently swap two fields of every one of the 34 rows.  The
    member NAMES are the C's own (`MediaId` really is spelled that way in a struct of lower-case
    names), because rq.js indexes the table by them. */
export const DRV_FIELDS = [
    ["sect", "SECT"], ["surf", "SURF"], ["cyl", "CYL"], ["tpg", "TPG"], ["gpc", "GPC"],
    ["xbn", "XBN"], ["dbn", "DBN"], ["lbn", "LBN"], ["rcts", "RCTS"], ["rctc", "RCTC"],
    ["rbn", "RBN"], ["mod", "MOD"], ["MediaId", "MED"], ["flgs", "FLGS"]
];

/**
 * drvNames(text)
 *
 * The drive type NAMES, in drv_tab[]'s own order, sliced out of the initialiser itself rather than
 * out of the parallel `drv_types[]` string table -- the two are separate lists in the C and a
 * differential that read the second would not notice the first growing an entry.  The `{ 0 }`
 * terminator is dropped; a row that is not `RQ_DRV (NAME)` is an error rather than a skip.
 *
 * @param {string} text
 * @returns {Array.<string>}
 */
export function drvNames(text)
{
    let m = /static struct drvtyp drv_tab\[\] = \{([\s\S]*?)\n *\};/.exec(text);
    if (!m) throw new Error("mscpscope: could not find drv_tab[]'s initialiser in pdp11_rq.c");
    let out = [];
    for (let line of m[1].split("\n")) {
        let t = line.trim().replace(/,$/, "");
        if (!t || t === "{ 0 }") continue;
        let r = /^RQ_DRV \(([A-Z0-9]+)\)$/.exec(t);
        if (!r) {
            throw new Error(`mscpscope: drv_tab[] row "${t}" is not an RQ_DRV(NAME) row -- the ` +
                `extraction cannot tell which #defines it pastes, and guessing would hand back a ` +
                `table that is short by one drive type with no sign that it is`);
        }
        out.push(r[1]);
    }
    return out;
}

/**
 * drvTable(text)
 *
 * drv_tab[] itself: for each name, the fourteen `#define <NAME>_<SUFFIX>` values RQ_DRV pastes.
 * THREE value shapes occur and all three are resolved rather than accepted or skipped:
 *   - a plain C literal (cint above, INCLUDING OCTAL);
 *   - another drive-type symbol -- `#define RA82_TPG RA82_SURF` and 30 more like it;
 *   - a parenthesised OR of RQDF_ flags -- `#define RRD40_FLGS (RQDF_RMV | RQDF_RO)`.
 * Anything else throws.  Resolution is bounded (a symbol may not chain more than a few deep) so a
 * circular #define is reported rather than hung on.
 *
 * @param {string} text
 * @returns {Array.<Object>}
 */
export function drvTable(text)
{
    let names = drvNames(text);
    let flags = defines(text, "RQDF");
    let raw = {}, re = /^#define[ \t]+([A-Z0-9_]+)[ \t]+(.+?)[ \t]*(?:\/\*.*)?$/gm, m;
    while ((m = re.exec(text)) !== null) if (!(m[1] in raw)) raw[m[1]] = m[2].trim();

    let resolve = (sym, depth) => {
        if (depth > 8) throw new Error(`mscpscope: ${sym} does not resolve to a number in 8 steps`);
        if (!(sym in raw)) throw new Error(`mscpscope: pdp11_rq.c has no #define for ${sym}`);
        let v = raw[sym];
        let lit = cint(v);
        if (lit !== null) return lit;
        if (/^[A-Z0-9_]+$/.test(v)) return resolve(v, depth + 1);
        let fl = /^\(([A-Z0-9_ |]+)\)$/.exec(v);
        if (fl) {
            let acc = 0;
            for (let t of fl[1].split("|")) {
                let name = t.trim();
                if (!name.startsWith("RQDF_") || !(name.slice(5) in flags)) {
                    throw new Error(`mscpscope: ${sym} is "${v}" and "${name}" is not an RQDF_ flag`);
                }
                acc |= flags[name.slice(5)];
            }
            return acc;
        }
        throw new Error(`mscpscope: ${sym} is defined as "${v}", which this extraction cannot ` +
            `evaluate -- interpreting it wrongly is worse than refusing it`);
    };

    return names.map((n) => {
        let row = {name: n};
        for (let [f, suffix] of DRV_FIELDS) row[f] = resolve(`${n}_${suffix}`, 0);
        return row;
    });
}

/** The switch's own no-op arm -- `cmd |= OP_END; sts = ST_SUC; break;` -- has no handler function,
    so it is named rather than left as an empty string, which would read as "not classified".  The
    name is rq.js's, not a second copy of it: two spellings of the same sentinel would make every
    no-op opcode compare unequal and the failure would read as a dispatch difference. */
export const NOP_ARM = RQVAX.MSCP_NOP_ARM;

/**
 * mscpDispatch(text)
 *
 * Every `case OP_x:` inside rq_mscp()'s own switch, MAPPED TO THE C FUNCTION IT REACHES.  Sliced to
 * the function body first, so a `case OP_RD:` in rq_svc() or rq_abo() cannot contaminate the answer.
 *
 * Cases FALL THROUGH IN GROUPS in this switch -- OP_ACC/CMP/ERS/RD/WR all reach rq_rw(), and
 * OP_CCD/DAP/FLU all reach the no-op arm -- so the walk accumulates pending labels and assigns the
 * handler to the whole group when it reaches the `return rq_xxx (` or the `break;` that ends it.
 * Reading only the first label of a group is the mistake that would report five opcodes as
 * unclassified; reading only the last would report four.
 *
 * @param {string} text
 * @returns {Object} {OPNAME: handler}
 */
export function mscpDispatch(text)
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
    let out = {}, pending = [], nLabels = 0;
    for (let line of body.split("\n")) {
        let t = line.trim();
        let c = /^case OP_([A-Z0-9]+):/.exec(t);
        if (c) { pending.push(c[1]); nLabels++; continue; }
        if (!pending.length) continue;
        let h = /^return (rq_[a-z_0-9]+) \(/.exec(t);
        if (h) { for (let n of pending) out[n] = h[1]; pending = []; continue; }
        if (/^break;$/.test(t)) { for (let n of pending) out[n] = NOP_ARM; pending = []; continue; }
    }
    if (pending.length) {
        throw new Error(`mscpscope: rq_mscp()'s \`case OP_${pending.join("/OP_")}:\` reaches neither ` +
            `a \`return rq_xxx (\` nor a \`break;\` -- the extraction has lost the end of the arm, ` +
            `and an opcode with no handler recorded is an opcode this check cannot police`);
    }
    if (nLabels !== Object.keys(out).length) {
        throw new Error(`mscpscope: rq_mscp()'s switch names ${nLabels} case labels but only ` +
            `${Object.keys(out).length} distinct opcodes -- a duplicate label means the extraction ` +
            `is reading something it should not`);
    }
    return out;
}

/**
 * extract(simhPath)
 * @param {?string} simhPath
 * @returns {{dir: string, op: Object, st: Object, dispatch: Object, drv: Array.<Object>}}
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
    return {dir, op: defines(h, "OP"), st: defines(h, "ST"),
            dispatch: mscpDispatch(c), drv: drvTable(c)};
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

    /* THE DISPATCH, OPCODE AND HANDLER TOGETHER.  Every `case OP_x:` in rq_mscp() must be
       classified by rq.js, and classified as reaching THE SAME C FUNCTION.  Checking only the
       opcode set would let rq.js move an opcode between its three unit-bearing classes -- answered
       here, needs an in-flight transfer, is a transfer -- without anything noticing, and those
       classes are exactly what decides whether a command is answered or throws by name. */
    let shipped = RQVAX.MSCP_OP_HANDLER;
    for (let n of Object.keys(e.dispatch)) {
        if (!(n in shipped)) {
            failures.push(`PHASE S: rq_mscp() has a \`case OP_${n}:\` and rq.js classifies OP_${n} ` +
                `nowhere, so it falls through to the ILLEGAL-OPCODE default and answers ` +
                `ST_CMD|I_OPCD for a command the controller really handles`);
        } else if (shipped[n] !== e.dispatch[n]) {
            failures.push(`PHASE S: rq_mscp() dispatches OP_${n} to ${e.dispatch[n]}() and rq.js ` +
                `records it as ${shipped[n]}() -- the two disagree about which handler, and ` +
                `therefore about which of rq.js's scope classes OP_${n} belongs to`);
        }
    }
    for (let n of Object.keys(shipped)) {
        if (!(n in e.dispatch)) {
            failures.push(`PHASE S: rq.js classifies OP_${n} as dispatched, but rq_mscp()'s switch ` +
                `has no case for it -- the C answers ST_CMD|I_OPCD and rq.js would not`);
        }
    }

    /* THE DRIVE TABLE.  Compared row for row and FIELD FOR FIELD, in table order, because the index
       into it is what `set rqN <type>` stores in the unit's flags and what every ONLINE and GET UNIT
       STATUS response is built from -- a table that agreed on contents but not on ORDER would give
       every unit a different drive's media identifier. */
    let dshipped = RQVAX.DRV_TAB;
    if (dshipped.length !== e.drv.length) {
        failures.push(`PHASE S: drv_tab[] has ${e.drv.length} entries in the C and ${dshipped.length} ` +
            `in rq.js -- a drive type that exists in only one of the two is a geometry that cannot ` +
            `be graded`);
    }
    for (let i = 0; i < Math.min(dshipped.length, e.drv.length); i++) {
        let d = e.drv[i], s = dshipped[i];
        if (d.name !== s.name) {
            failures.push(`PHASE S: drv_tab[${i}] is ${d.name} in the C and ${s.name} in rq.js -- ` +
                `the tables are in different ORDER, so every drive type index means a different drive`);
            continue;
        }
        for (let [f] of DRV_FIELDS) {
            if ((d[f] >>> 0) !== (s[f] >>> 0)) {
                failures.push(`PHASE S: drv_tab[${i}] (${d.name}) .${f} is ${d[f]} in the C and ` +
                    `${s[f]} in rq.js`);
            }
        }
    }

    return {dir: e.dir, nOp: Object.keys(e.op).length, nSt: Object.keys(e.st).length,
            nSwitch: Object.keys(e.dispatch).length, nDrv: e.drv.length,
            failures, op: e.op, st: e.st, dispatch: e.dispatch, drv: e.drv};
}

function main()
{
    let i = process.argv.indexOf("--simh");
    let simh = null;
    try { simh = i >= 0 ? process.argv[i + 1] : findSimhBin(null); } catch (e) { simh = null; }
    let r = checkScope(simh);
    console.log(`mscpscope: derived from ${r.dir}`);
    console.log(`  ${r.nOp} OP_ codes, ${r.nSt} ST_ codes, ${r.nSwitch} rq_mscp() dispatch cases, ` +
        `${r.nDrv} drv_tab[] entries`);
    if (process.argv.includes("--print")) {
        console.log(`  OP: ${Object.keys(r.op).map((k) => `${k}=${r.op[k]}`).join(" ")}`);
        console.log(`  ST: ${Object.keys(r.st).map((k) => `${k}=${r.st[k]}`).join(" ")}`);
        console.log(`  dispatch: ${Object.keys(r.dispatch).map((k) => `${k}->${r.dispatch[k]}`).join(" ")}`);
        for (let d of r.drv) {
            console.log(`  drv ${d.name.padEnd(6)} ${DRV_FIELDS.map(([f]) => `${f}=${d[f]}`).join(" ")}`);
        }
    }
    if (r.failures.length) {
        console.error(`\nFAIL -- ${r.failures.length} scope difference(s):`);
        for (let f of r.failures) console.error(`  ${f}`);
        process.exit(1);
    }
    console.log("\nOK -- rq.js's opcode, status, dispatch and drive-table scope match the C exactly");
}

if (process.argv[1] && process.argv[1].endsWith("mscpscope.js")) main();
