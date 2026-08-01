/**
 * @fileoverview Generates machines/dec/vax/modules/v2/drom.js from vendored Open SIMH source
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
 * WHY THIS EXISTS
 * ---------------
 * The VAX decode ROM is a 512 x 7 table: for every opcode, a header word (specifier count, FPD
 * flag, result shape, instruction group) plus up to six operand-specifier descriptors.  Hand
 * transcribing it from the architecture manual is 3,584 numbers of pure opportunity for error,
 * and there is no way to review it.  So it is EXTRACTED, positionally, from the same C source
 * we are porting -- `open-simh/VAX/vax_sys.c` -- and the symbolic names in that table are
 * resolved against the #defines in `open-simh/VAX/vax_defs.h`.  Nothing here is typed in by hand
 * except the self-check assertions at the bottom, which exist precisely so that a silent parse
 * failure cannot produce a plausible-looking wrong table.
 *
 * `../../../../../pcjs-vax/tools/ehkaa-profile/parse_drom.py` already parses the same two arrays
 * for the profiling tool; this is the JS-emitting sibling of it, and deliberately reproduces its
 * VAX_610 preprocessor handling (the microvax3900 target does not define VAX_610, so the #else
 * branch is kept).
 *
 *      node machines/dec/vax/tests/gen_drom.js [--simh-src DIR] [--out FILE] [--check]
 *
 * --check regenerates in memory and fails if the committed drom.js differs, which is how CI (or
 * a reviewer) confirms the committed table still matches the vendored source.
 */

import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "../../../..");

const NUM_INST = 512;
const MAX_SPEC = 6;

/**
 * findSimhSrc(dirArg)
 *
 * @param {string|null} dirArg
 * @returns {string} directory containing vax_sys.c and vax_defs.h
 */
function findSimhSrc(dirArg)
{
    let candidates = [];
    if (dirArg) candidates.push(dirArg);
    if (process.env['SIMH_SRC']) candidates.push(process.env['SIMH_SRC']);
    candidates.push(path.resolve(REPO_ROOT, "../pcjs-vax/open-simh/VAX"));
    for (let dir of candidates) {
        if (fs.existsSync(path.join(dir, "vax_sys.c")) && fs.existsSync(path.join(dir, "vax_defs.h"))) {
            return dir;
        }
    }
    throw new Error("Open SIMH VAX sources not found; pass --simh-src DIR or set SIMH_SRC.  Tried:\n  " + candidates.join("\n  "));
}

/**
 * stripVAX610(text)
 *
 * Drops the `#if defined (VAX_610)` branch and keeps the `#else` branch, because the
 * microvax3900 build target does not define VAX_610.  Same rule as parse_drom.py; see its
 * docstring for why the choice is documented rather than silent (both observed instances have
 * byte-identical specifier lists, only the IG_* group flag differs).
 *
 * @param {string} text
 * @returns {string}
 */
function stripVAX610(text)
{
    let out = [], skipping = false, depth = 0;
    for (let line of text.split("\n")) {
        let s = line.trim();
        if (s.startsWith("#if defined (VAX_610)") || s.startsWith("#if defined(VAX_610)")) {
            skipping = true; depth = 1; continue;
        }
        if (skipping && s.startsWith("#else") && depth == 1) { skipping = false; continue; }
        if (s.startsWith("#endif") && depth == 1 && !skipping) { depth = 0; continue; }
        if (skipping) continue;
        out.push(line);
    }
    return out.join("\n");
}

/**
 * extractBracedBlock(text, marker)
 *
 * @param {string} text
 * @param {string} marker
 * @returns {string} text between the first "{" after marker and its matching "}"
 */
function extractBracedBlock(text, marker)
{
    let idx = text.indexOf(marker);
    if (idx < 0) throw new Error("marker not found: " + marker);
    let start = text.indexOf("{", idx);
    let depth = 0;
    for (let i = start; i < text.length; i++) {
        if (text[i] == "{") depth++;
        else if (text[i] == "}") {
            if (--depth == 0) return text.slice(start + 1, i);
        }
    }
    throw new Error("unbalanced braces scanning from " + marker);
}

/**
 * identifiers(expr)
 *
 * Names referenced by a C constant expression.  Numeric literals are blanked FIRST, because
 * "0x000" would otherwise yield the bogus identifier "x000".
 *
 * @param {string} expr
 * @returns {Array.<string>}
 */
function identifiers(expr)
{
    let stripped = expr.replace(/\b0[xX][0-9A-Fa-f]+\b/g, " ").replace(/\b\d+\b/g, " ");
    return stripped.match(/[A-Za-z_][A-Za-z0-9_]*/g) || [];
}

/**
 * parseDefines(defsSrc)
 *
 * Builds name -> integer for every OBJECT-like #define in vax_defs.h whose right-hand side is a
 * C integer expression over numeric literals and previously defined names.  Function-like macros
 * (`#define DR_LNT(x) ...`) are skipped: they are parameterised and are transcribed into JS by
 * hand in drom.js/decode.js, where they can be reviewed.  Anything that fails to evaluate is
 * skipped rather than guessed at; the assertions in main() are what prove the names we actually
 * consume came out right.
 *
 * @param {string} defsSrc
 * @returns {Object} map of name to number
 */
function parseDefines(defsSrc)
{
    let vals = {};
    /*
     * Preprocessor conditionals in vax_defs.h select CPU-model-specific values.  We are the
     * microvax3900 (VAX_650 / KA655), so keep the plain (non-model-conditional) definitions and
     * let later definitions win, then verify by assertion.  None of the names this generator
     * consumes -- the decode-ROM field and access-type constants -- live inside a model #if.
     */
    let re = /^\s*#\s*define\s+([A-Za-z_][A-Za-z0-9_]*)\s+(.+?)\s*$/;
    for (let raw of defsSrc.split("\n")) {
        let line = raw.replace(/\/\*.*?\*\//g, "").replace(/\/\/.*$/, "");
        let m = re.exec(line);
        if (!m) continue;
        let name = m[1], expr = m[2].trim();
        if (!expr) continue;
        /* Reject C-isms we do not evaluate: casts, suffixes, strings, char literals. */
        if (/["']/.test(expr)) continue;
        let jsExpr = expr.replace(/([0-9])[uU]\b/g, "$1");
        if (!/^[0-9A-Za-z_|&~<>()+\-*\s]+$/.test(jsExpr)) continue;
        if (/[A-Za-z_][A-Za-z0-9_]*\s*\(/.test(jsExpr)) continue;   // function-like macro invocation
        let names = identifiers(jsExpr);
        let ok = true;
        for (let n of names) {
            if (!(n in vals)) { ok = false; break; }
        }
        if (!ok) continue;
        let scope = Object.keys(vals).map((k) => `const ${k}=${vals[k]};`).join("");
        let v;
        try {
            v = Function(`"use strict";${scope}return (${jsExpr})|0;`)();
        } catch (e) {
            continue;
        }
        if (typeof v == "number" && Number.isFinite(v)) vals[name] = v;
    }
    return vals;
}

/**
 * evalTokenExpr(expr, vals)
 *
 * Evaluates one drom[] table field, e.g. "3+DR_F+RB_R3+IG_PACKD" or "RB" or "0".
 *
 * @param {string} expr
 * @param {Object} vals
 * @returns {number}
 */
function evalTokenExpr(expr, vals)
{
    let jsExpr = expr.trim();
    /*
     * ODC() is the one function-like macro appearing in drom[]: vax_sys.c:52-56 defines it as
     * `(x)` for FULL_VAX and `((x) << DR_V_USPMASK)` otherwise.  The microvax3900 target builds
     * from vax650_defs.h, which neither defines nor undefines FULL_VAX -- and FULL_VAX is not
     * defined anywhere else for it -- so the subset (shifted) form is the one that applies.  That
     * is the same conclusion the running simulator reports as "Emulating: Packed-Decimal-String-
     * Group Extended-Accuracy-Group Emulated-Only-Group".  ODC only carries the operand count for
     * instructions this CPU traps to an emulator, so it never reaches the specifier decoder; it is
     * expanded correctly anyway so the header word is faithful.
     */
    jsExpr = jsExpr.replace(/\bODC\s*\(\s*([0-9]+)\s*\)/g, (m0, n) => `((${n}) << DR_V_USPMASK)`);
    if (!/^[0-9A-Za-z_|&~<>()+\-*\s]+$/.test(jsExpr)) throw new Error("unparseable drom field: " + expr);
    let names = identifiers(jsExpr);
    for (let n of names) {
        if (!(n in vals)) throw new Error(`drom field ${expr!==n?expr+" -> ":""}${n}: undefined constant`);
    }
    let scope = names.map((k) => `const ${k}=${vals[k]};`).join("");
    return Function(`"use strict";${scope}return (${jsExpr})|0;`)();
}

/**
 * parseOpcodeTable(sysSrc)
 *
 * @param {string} sysSrc
 * @returns {Array.<string|null>} 512 mnemonics
 */
function parseOpcodeTable(sysSrc)
{
    let body = extractBracedBlock(sysSrc, "char const * const opcode[] = {");
    body = body.replace(/\/\*[\s\S]*?\*\//g, "");
    let tokens = body.split(",").map((t) => t.trim()).filter((t) => t !== "");
    let names = tokens.map((t) => {
        if (t == "NULL") return null;
        let m = /^"([^"]*)"$/.exec(t);
        if (!m) throw new Error("unrecognized opcode[] token: " + t);
        return m[1];
    });
    if (names.length != NUM_INST) throw new Error(`expected ${NUM_INST} opcode[] entries, parsed ${names.length}`);
    return names;
}

/**
 * parseDromTable(sysSrc, vals)
 *
 * @param {string} sysSrc
 * @param {Object} vals
 * @returns {Array.<Array.<number>>} 512 rows of 7 numbers (header + 6 specifiers)
 */
function parseDromTable(sysSrc, vals)
{
    let src = stripVAX610(sysSrc);
    let body = extractBracedBlock(src, "const uint16 drom[NUM_INST][MAX_SPEC + 1] = {");
    body = body.replace(/\/\*[\s\S]*?\*\//g, "");
    let rows = [], depth = 0, cur = [];
    for (let ch of body) {
        if (ch == "{") { if (++depth == 1) { cur = []; continue; } }
        if (ch == "}") { if (--depth == 0) { rows.push(cur.join("")); continue; } }
        if (depth >= 1) cur.push(ch);
    }
    if (rows.length != NUM_INST) throw new Error(`expected ${NUM_INST} drom[] rows, parsed ${rows.length}`);
    return rows.map((text) => {
        let fields = text.split(",").map((f) => f.trim()).filter((f) => f !== "");
        if (fields.length != MAX_SPEC + 1) {
            throw new Error(`drom row has ${fields.length} fields, expected ${MAX_SPEC + 1}: ${text}`);
        }
        return {
            values: fields.map((f) => evalTokenExpr(f, vals)),
            tokens: fields
        };
    });
}

/**
 * emit(mnemonics, rows, vals)
 *
 * @param {Array.<string|null>} mnemonics
 * @param {Array.<Array.<number>>} rows
 * @param {Object} vals
 * @returns {string}
 */
function emit(mnemonics, rows, vals)
{
    let L = [];
    L.push("/**");
    L.push(" * @fileoverview VAX decode ROM -- GENERATED, DO NOT EDIT BY HAND");
    //
    // This header is EMITTED into the generated drom.js, so it carries the same corrected
    // attribution as every hand-written file (rd pcjsvax-422).  If it drifts back to claiming
    // Jeff Parsons as author, the next regeneration silently reintroduces the defect into a file
    // nobody re-reads, because the first line tells them not to edit it by hand.
    //
    L.push(" * @author Chris Baron <baron@3dl.dev>");
    L.push(" * @copyright © 2026 Chris Baron");
    L.push(" * @license MIT <https://www.pcjs.org/LICENSE.txt>");
    L.push(" *");
    L.push(" * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.");
    L.push(" * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT");
    L.push(" * license.");
    L.push(" *");
    L.push(" * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,");
    L.push(" * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.");
    L.push(" *");
    L.push(" * Generated by machines/dec/vax/tests/gen_drom.js from open-simh/VAX/vax_sys.c (the `opcode[]`");
    L.push(" * and `drom[NUM_INST][MAX_SPEC+1]` arrays) and open-simh/VAX/vax_defs.h (the symbolic field");
    L.push(" * values).  Regenerate with:");
    L.push(" *");
    L.push(" *      node machines/dec/vax/tests/gen_drom.js");
    L.push(" *");
    L.push(" * and verify the committed copy still matches its source with:");
    L.push(" *");
    L.push(" *      node machines/dec/vax/tests/gen_drom.js --check");
    L.push(" *");
    L.push(" * LAYOUT.  DROM is a flat Uint16Array of 512 rows x 7 columns, row-major, indexed");
    L.push(" * DROM[opc * DROM_STRIDE + i].  Column 0 is the opcode header word (specifier count in");
    L.push(" * <2:0>, DR_F in <7>, result shape in <11:8>, instruction group in <14:12>); columns 1..6");
    L.push(" * are the operand-specifier descriptors, left to right, 0 for unused slots.  Opcodes");
    L.push(" * 0x000-0x0FF are one-byte; 0x100-0x1FF are the FD-prefixed two-byte opcodes.");
    L.push(" */");
    L.push("");
    L.push("const DROM_STRIDE = " + (MAX_SPEC + 1) + ";");
    L.push("const NUM_INST = " + NUM_INST + ";");
    L.push("");
    L.push("/*");
    L.push(" * Decode ROM field values, extracted from vax_defs.h so that decode.js never has to");
    L.push(" * repeat a magic number that lives in the vendored source.");
    L.push(" */");
    L.push("const DR = {");
    let fields = [
        ["F", "DR_F"], ["NSPMASK", "DR_NSPMASK"], ["ACMASK", "DR_ACMASK"], ["SPFLAG", "DR_SPFLAG"],
        ["LNMASK", "DR_LNMASK"], ["V_RESMASK", "DR_V_RESMASK"], ["M_RESMASK", "DR_M_RESMASK"],
        ["V_IGMASK", "DR_V_IGMASK"], ["M_IGMASK", "DR_M_IGMASK"],
        ["R", "DR_R"], ["M", "DR_M"], ["A", "DR_A"], ["W", "DR_W"]
    ];
    for (let [k, n] of fields) L.push(`    ${k}: 0x${(vals[n] >>> 0).toString(16).toUpperCase()},`);
    L.push(L.pop().replace(/,$/, ""));
    L.push("};");
    L.push("");
    L.push("/*");
    L.push(" * Instruction groups (vax_defs.h IG_*), pre-shifted down by DR_V_IGMASK, i.e. these are");
    L.push(" * the values DR_GETIGRP() yields.  IG_RSVD means the opcode is explicitly reserved.");
    L.push(" */");
    L.push("const IG = {");
    for (let n of ["IG_RSVD", "IG_BASE", "IG_BSGFL", "IG_BSDFL", "IG_PACKD", "IG_EXTAC", "IG_EMONL", "IG_VECTR"]) {
        L.push(`    ${n.slice(3)}: ${(vals[n] >>> vals["DR_V_IGMASK"]) & vals["DR_M_IGMASK"]},`);
    }
    L.push(L.pop().replace(/,$/, ""));
    L.push("};");
    L.push("");
    L.push("/*");
    L.push(" * Operand-specifier descriptor values (vax_defs.h): access type in <9:8>, DR_SPFLAG in");
    L.push(" * <3>, data length code in <2:0>.  BB/BW are not addressing-mode specifiers at all --");
    L.push(" * they are branch displacement fields baked into the instruction format.");
    L.push(" */");
    L.push("const SPEC = {");
    for (let n of ["RB","RW","RL","RQ","RO","MB","MW","ML","MQ","MO","AB","AW","AL","AQ","AO",
                   "WB","WW","WL","WQ","WO","VB","RF","RD","RG","RH","BB","BW"]) {
        L.push(`    ${n}: 0x${(vals[n] >>> 0).toString(16).toUpperCase().padStart(3, "0")},`);
    }
    L.push(L.pop().replace(/,$/, ""));
    L.push("};");
    L.push("");
    L.push("/*");
    L.push(" * Addressing-mode nibble values (vax_defs.h): the top nibble of a specifier byte.");
    L.push(" * SH0..SH3 are all short literal (the literal's high bits live in that nibble).");
    L.push(" */");
    L.push("const MODE = {");
    for (let n of ["SH0","SH1","SH2","SH3","IDX","GRN","RGD","ADC","AIN","AID","BDP","BDD","WDP","WDD","LDP","LDD"]) {
        L.push(`    ${n}: 0x${(vals[n] >>> 0).toString(16).toUpperCase().padStart(3, "0")},`);
    }
    L.push(L.pop().replace(/,$/, ""));
    L.push("};");
    L.push("");
    L.push("/*");
    L.push(" * Recovery-queue field encoding (vax_defs.h): an entry is (descriptor << RQ_V_LNT) | regnum.");
    L.push(" * RQ_DIR distinguishes an increment (undo by subtracting) from a decrement (undo by adding);");
    L.push(" * it is bit <11>, which is exactly bit <7> of the addressing-mode nibble, so autoincrement");
    L.push(" * (0x80), autoincrement deferred (0x90) and the displacement modes set it while");
    L.push(" * autodecrement (0x70) does not.");
    L.push(" */");
    L.push("const RQ = {");
    for (let n of ["RQ_RN", "RQ_V_LNT", "RQ_M_LNT", "RQ_DIR"]) {
        L.push(`    ${n.slice(3)}: 0x${(vals[n] >>> 0).toString(16).toUpperCase()},`);
    }
    L.push(L.pop().replace(/,$/, ""));
    L.push("};");
    L.push("");
    L.push("const DROM = new Uint16Array([");
    for (let opc = 0; opc < NUM_INST; opc++) {
        let row = rows[opc].map((v) => "0x" + (v >>> 0).toString(16).toUpperCase().padStart(4, "0"));
        let mn = mnemonics[opc] === null ? "-" : mnemonics[opc];
        L.push(`    ${row.join(",")},  /* ${(opc >>> 0).toString(16).toUpperCase().padStart(3, "0")} ${mn} */`);
    }
    L.push("]);");
    L.push("");
    L.push("/*");
    L.push(" * Mnemonics, positionally indexed like DROM.  null = undefined opcode slot.  Used by the");
    L.push(" * debugger and by test harnesses; the decoder itself never looks at a mnemonic.");
    L.push(" */");
    L.push("const OPCODES = [");
    for (let i = 0; i < NUM_INST; i += 8) {
        L.push("    " + mnemonics.slice(i, i + 8).map((m) => m === null ? "null" : `"${m}"`).join(", ") + ",");
    }
    L.push(L.pop().replace(/,$/, ""));
    L.push("];");
    L.push("");
    L.push("export { DROM, DROM_STRIDE, NUM_INST, OPCODES, DR, IG, SPEC, MODE, RQ };");
    L.push("");
    return L.join("\n");
}

function main()
{
    let argv = process.argv.slice(2);
    let getArg = (name, def) => {
        let i = argv.indexOf(name);
        return i >= 0 && i + 1 < argv.length ? argv[i + 1] : def;
    };
    let srcDir = findSimhSrc(getArg("--simh-src", null));
    let outFile = getArg("--out", path.join(REPO_ROOT, "machines/dec/vax/modules/v2/drom.js"));
    let fCheck = argv.indexOf("--check") >= 0;

    let defsSrc = fs.readFileSync(path.join(srcDir, "vax_defs.h"), "utf8");
    let sysSrc = fs.readFileSync(path.join(srcDir, "vax_sys.c"), "utf8");
    let vals = parseDefines(defsSrc);

    /*
     * SELF-CHECK.  These are the only numbers in this file typed in by a human, and they exist so
     * that a silently mis-parsed vax_defs.h cannot produce a plausible-looking table.  Each is
     * derived in the comment from the #define it must equal.
     */
    let expected = {
        DR_R: 0x000, DR_M: 0x100, DR_A: 0x200, DR_W: 0x300,     /* access type, <9:8> */
        DR_SPFLAG: 0x008, DR_LNMASK: 0x007, DR_ACMASK: 0x300,
        RB: 0x000, RW: 0x001, RL: 0x002, RQ: 0x003, RO: 0x004,  /* DR_R | length code */
        MB: 0x100, MW: 0x101, ML: 0x102, MQ: 0x103, MO: 0x104,  /* DR_M | length code */
        AB: 0x200, AW: 0x201, AL: 0x202, AQ: 0x203, AO: 0x204,  /* DR_A | length code */
        WB: 0x300, WW: 0x301, WL: 0x302, WQ: 0x303, WO: 0x304,  /* DR_W | length code */
        VB: 0x308,                                              /* DR_SPFLAG | WB */
        RF: 0x00A, RD: 0x00B, RG: 0x10B, RH: 0x00C,             /* DR_SPFLAG | RL/RQ/MQ/RO */
        BB: 0x30E, BW: 0x30F,                                   /* DR_SPFLAG | WB | 6,7 */
        SH0: 0x000, SH1: 0x010, SH2: 0x020, SH3: 0x030, IDX: 0x040, GRN: 0x050, RGD: 0x060,
        ADC: 0x070, AIN: 0x080, AID: 0x090, BDP: 0x0A0, BDD: 0x0B0, WDP: 0x0C0, WDD: 0x0D0,
        LDP: 0x0E0, LDD: 0x0F0,
        RQ_RN: 0xF, RQ_V_LNT: 4, RQ_M_LNT: 0x7, RQ_DIR: 0x800,
        DR_F: 0x80, DR_NSPMASK: 0x07, DR_V_IGMASK: 12, DR_M_IGMASK: 0x0007,
        nPC: 15, nSP: 14, nAP: 12, OP_MEM: -1                   /* OP_MEM is 0xFFFFFFFF as int32 */
    };
    let bad = [];
    for (let k of Object.keys(expected)) {
        if (!(k in vals)) { bad.push(`${k}: not parsed from vax_defs.h`); continue; }
        if ((vals[k] | 0) != (expected[k] | 0)) {
            bad.push(`${k}: parsed 0x${(vals[k] >>> 0).toString(16)} expected 0x${(expected[k] >>> 0).toString(16)}`);
        }
    }
    if (bad.length) {
        console.error("FATAL: vax_defs.h parse disagrees with the expected VAX decode-ROM encoding:");
        for (let b of bad) console.error("  " + b);
        process.exit(2);
    }

    let mnemonics = parseOpcodeTable(sysSrc);
    let rows = parseDromTable(sysSrc, vals);

    /*
     * Cross-check the parsed table against facts independently knowable from the architecture:
     * every defined opcode's header specifier count must equal the number of non-zero specifier
     * columns, and every undefined slot must be entirely zero.
     */
    let nDefined = 0, nODC = 0;
    for (let opc = 0; opc < NUM_INST; opc++) {
        let row = rows[opc];
        let nsp = row.values[0] & vals["DR_NSPMASK"];
        /*
         * Count SOURCE TOKENS, not values: RB is legitimately 0x000 (DR_R | DR_BYTE), so a
         * value-based "is this slot used" test would mis-count PROBER (`RB,RB,AB`) as having two
         * specifiers.  An unused slot is written literally as `0` in the C source.
         */
        let nUsed = 0;
        for (let i = 1; i <= MAX_SPEC; i++) if (row.tokens[i] !== "0") nUsed++;
        if (mnemonics[opc] === null) {
            /*
             * An undefined slot has no specifiers.  Its header word is NOT required to be zero:
             * vax_sys.c gives reserved slots an explicit instruction group (IG_RSVD == 0 shifted,
             * so the header may still carry other bits), and vax_cpu.c faults on the group before
             * ever consulting the specifier columns.
             */
            if (row.tokens.slice(1).some((t) => t !== "0")) {
                console.error(`FATAL: opcode ${opc.toString(16)} has no mnemonic but non-zero specifier slots`);
                process.exit(2);
            }
            if (nsp != 0) {
                console.error(`FATAL: opcode ${opc.toString(16)} has no mnemonic but header claims ${nsp} specifiers`);
                process.exit(2);
            }
            continue;
        }
        nDefined++;
        /*
         * ODC() rows are the instructions this CPU does not implement (extended-accuracy and
         * emulated-only groups).  vax_sys.c deliberately reports ZERO specifiers in <2:0> for
         * them -- vax_cpu.c must fault to the emulator BEFORE resolving anything -- and stores
         * the operand count in <6:4> (DR_GETUSP) instead, for cpu_emulate_exception() to push.
         * So they are checked against DR_GETUSP, not DR_GETNSP.
         */
        let usp = (row.values[0] >> vals["DR_V_USPMASK"]) & vals["DR_M_USPMASK"];
        let isODC = row.tokens[0].includes("ODC");
        let nExpect = isODC ? usp : nsp;
        if (isODC && nsp != 0) {
            console.error(`FATAL: opcode ${opc.toString(16)} (${mnemonics[opc]}) is ODC() but header claims ${nsp} decodable specifiers`);
            process.exit(2);
        }
        if (nExpect != nUsed) {
            console.error(`FATAL: opcode ${opc.toString(16)} (${mnemonics[opc]}) header says ${nExpect} specifiers but ${nUsed} slots are used`);
            process.exit(2);
        }
        if (isODC) nODC++;
        /* Every used slot must be a descriptor this decoder knows how to dispatch on. */
        for (let i = 1; i <= nUsed; i++) {
            let d = row.values[i];
            let at = d & (vals["DR_ACMASK"] | vals["DR_SPFLAG"] | vals["DR_LNMASK"]);
            if (at !== d) {
                console.error(`FATAL: opcode ${opc.toString(16)} specifier ${i} = 0x${d.toString(16)} has bits outside DR_ACMASK|DR_SPFLAG|DR_LNMASK`);
                process.exit(2);
            }
        }
    }

    let text = emit(mnemonics, rows.map((r) => r.values), vals);
    if (fCheck) {
        let cur = fs.existsSync(outFile) ? fs.readFileSync(outFile, "utf8") : "";
        if (cur !== text) {
            console.error(`FAIL: ${outFile} does not match a fresh generation from ${srcDir}`);
            process.exit(1);
        }
        console.log(`OK: drom.js matches ${srcDir} (${nDefined} defined opcodes)`);
        return;
    }
    fs.mkdirSync(path.dirname(outFile), {recursive: true});
    fs.writeFileSync(outFile, text);
    console.log(`wrote ${outFile}: ${nDefined} defined opcodes of ${NUM_INST} slots (${nODC} not implemented by this CPU, emulator-dispatched)`);
}

main();
