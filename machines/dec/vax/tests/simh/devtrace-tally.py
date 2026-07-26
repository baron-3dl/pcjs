#!/usr/bin/env python3
"""
devtrace-tally.py -- pcjsvax-e9a's own gate on its own instrumentation.

Boots ka655x.bin to the console ">>>" prompt under a patch-0006-instrumented SIMH binary with
`SET CPU DEBUG=EXCTRACE` and `SET SYSD DEBUG=DEVTRACE`, then asserts that every KA655 device
register family and the delivered-interrupt trace the device-emulation items after EHKAA depend on
actually produced events. Exits non-zero and names, by family, any that reached zero.

Why this exists (veracity re-dispatch, 2026-07-26): the patch alone -- applying cleanly, building,
passing EHKAA, and passing all eleven differentials -- does not prove the *new* instrumentation
works. An adversary built a variant with the WriteReg trace call guarded by `if (0 && sim_deb)`:
it applies clean, builds clean, EHKAA passes, all eleven differentials pass, and it silently
produces zero REGW events -- killing CMCTL writes, KA CACR writes, CQBIC writes and SSC T0/T1
writes, four of the families this item's own done condition requires, with nothing catching it.
This script is the thing that has to catch that: it is invoked from verify-patches.sh's run_build
(every normal run) and from --selfcheck's mutation 4/4 (which constructs exactly that mutation and
asserts THIS SCRIPT reports it, by name, as a failure).

Two independent things are derived, not remembered, and for two different reasons:

  - The physical-address / IPR-number VALUES that identify each family are parsed out of the
    actual patched source tree (VAX/vax_defs.h, VAX/vaxmod_defs.h, VAX/vax_sysdev.c) on every
    run, never hand-typed. HANDOFF.md Sec6 rule 5: a hand-derived scope list is how the CIS
    opcode count went 7 -> 11 -> 17 -> 23, every value wrong, until it was computed instead of
    remembered.

  - Which direction(s) (read and/or write) each family is REQUIRED to show nonzero is the
    empirically measured behaviour of an actual boot-to->>> run (see this directory's README.md,
    "What 0006 adds"), not a theoretical claim about what the hardware supports. ReadIPR's switch
    has a `case MT_ICCS:` -- the read path structurally exists -- but this ROM's boot-to-console
    path never issues an MFPR ICCS, so requiring a nonzero ICCS read here would be a FALSE floor
    (flaky on firmware/timing variation), not a stronger one. The required directions below are
    exactly what a real run produces, no more.

Usage:
    devtrace-tally.py --simh PATH/TO/microvax3900 --src PATH/TO/open-simh [--runlimit '200M instructions']
                       [--timeout 120] [--keep-log FILE]

Exit 0: every required family/direction reached at least one event.
Exit 1: the boot did not reach >>>, or at least one required family/direction reached zero events
        (named explicitly on stderr) -- or the source tree's shape changed enough that a family's
        address/IPR-number/sub-range could not be derived at all (fail loud, not silently skip).
"""
import argparse
import ast
import operator
import os
import re
import subprocess
import sys
import tempfile
import textwrap
from collections import Counter

# ---------------------------------------------------------------------------------------------
# Restricted arithmetic evaluator for header #define bodies (e.g. "(REGBASE + 0x4000)",
# "(1u << KAAWIDTH)"). Deliberately NOT a bare eval() -- this only ever sees text pulled from a
# vendor C header, but the point of a differential project is to never trust a string when a
# parser is this cheap to write.
# ---------------------------------------------------------------------------------------------
_BINOPS = {
    ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
    ast.LShift: operator.lshift, ast.RShift: operator.rshift,
    ast.BitOr: operator.or_, ast.BitAnd: operator.and_, ast.BitXor: operator.xor,
}
_UNARYOPS = {ast.USub: operator.neg, ast.UAdd: operator.pos}


def _safe_eval_int(expr):
    tree = ast.parse(expr, mode="eval")

    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.BinOp) and type(node.op) in _BINOPS:
            return _BINOPS[type(node.op)](ev(node.left), ev(node.right))
        if isinstance(node, ast.UnaryOp) and type(node.op) in _UNARYOPS:
            return _UNARYOPS[type(node.op)](ev(node.operand))
        if isinstance(node, ast.Constant) and isinstance(node.value, int):
            return node.value
        raise ValueError("unsupported node in device-constant arithmetic: %s" % ast.dump(node))

    return ev(tree)


_DEFINE_RE = re.compile(r'^\s*#define\s+(\w+)\s+(.+)$')
_INT_SUFFIX_RE = re.compile(r'\b(0[xX][0-9A-Fa-f]+|\d+)[uUlL]+\b')
_IDENT_RE = re.compile(r'\b[A-Za-z_][A-Za-z0-9_]*\b')


def load_defines(path):
    """Parse every `#define NAME BODY` in a header into {NAME: raw-body-text}, comments and
    trailing whitespace stripped. Returns raw text -- resolve() does the arithmetic."""
    defs = {}
    with open(path) as f:
        for line in f:
            line = line.split("/*", 1)[0].rstrip()
            m = _DEFINE_RE.match(line)
            if m:
                defs[m.group(1)] = m.group(2).strip()
    return defs


def resolve(name, defs, cache):
    """Resolve a #define to an int, substituting any other macro names its body references
    (recursively) before evaluating. Raises KeyError/ValueError loudly if the shape of the
    header no longer matches what this script expects -- never silently falls back."""
    if name in cache:
        return cache[name]
    body = defs[name]

    def repl(m):
        ident = m.group(0)
        if ident in defs and ident != name:
            return "(%d)" % resolve(ident, defs, cache)
        return ident

    resolved = _IDENT_RE.sub(repl, body)
    resolved = _INT_SUFFIX_RE.sub(r'\1', resolved)
    val = _safe_eval_int(resolved)
    cache[name] = val
    return val


def ssc_timer_offset_range(vax_sysdev_c):
    """Derive the SSC T0/T1 timer register sub-range (case labels 0x40..0x47 in ssc_rd/ssc_wr,
    each commented /* T0CSR */ .. /* T1VEC */) by parsing the source, rather than hardcoding the
    offsets this script's author already knows. Returns (min_offset, max_offset), inclusive,
    in units of the SSC register index (physical byte offset from SSCBASE is offset * 4)."""
    pat = re.compile(r'case\s+(0x[0-9A-Fa-f]+):\s*/\*\s*(T[01](?:CSR|INT|NI|VEC))\s*\*/')
    offsets = set()
    with open(vax_sysdev_c) as f:
        for line in f:
            m = pat.search(line)
            if m:
                offsets.add(int(m.group(1), 16))
    if not offsets:
        raise SystemExit(
            "FATAL: found no T0/T1 timer case labels (e.g. 'case 0x40: /* T0CSR */') in %s -- "
            "ssc_rd/ssc_wr's structure changed; re-derive the SSC timer sub-range by hand and "
            "fix this parser, do not guess a range." % vax_sysdev_c)
    return min(offsets), max(offsets)


# ---------------------------------------------------------------------------------------------
# Boot ka655x.bin to >>> and capture the DEVTRACE/EXCTRACE debug log.
# ---------------------------------------------------------------------------------------------
_INI_TEMPLATE = """\
set console notelnet
set debug -n {logfile}
set cpu debug=EXCTRACE
set sysd debug=DEVTRACE
set runlimit {runlimit}
set on
on error ignore
expect ">>>" echof "\\r\\n*** PCJS-DEVTRACE: REACHED CONSOLE PROMPT ***\\n"; exit 0
on runtime echof "\\r\\n*** PCJS-DEVTRACE: RUNTIME LIMIT HIT, NO >>> SEEN ***\\n"; exit 1
boot cpu
"""


def boot_and_capture(simh_bin, runlimit, timeout):
    logfd, logpath = tempfile.mkstemp(prefix="pcjs-devtrace-", suffix=".log")
    os.close(logfd)
    inifd, inipath = tempfile.mkstemp(prefix="pcjs-devtrace-", suffix=".ini")
    with os.fdopen(inifd, "w") as f:
        f.write(_INI_TEMPLATE.format(logfile=logpath, runlimit=runlimit))
    try:
        try:
            proc = subprocess.run([simh_bin, inipath], capture_output=True, text=True,
                                   timeout=timeout)
        except subprocess.TimeoutExpired:
            raise SystemExit(
                "FATAL: %s did not return within %ds wall clock -- hung, not merely a runlimit "
                "miss. devtrace tally cannot proceed." % (simh_bin, timeout))
    finally:
        os.unlink(inipath)
    if "REACHED CONSOLE PROMPT" not in proc.stdout:
        raise SystemExit(
            "FATAL: %s did not reach the console >>> prompt within runlimit=%s. Last output:\n%s"
            % (simh_bin, runlimit, proc.stdout[-2000:]))
    return logpath


# ---------------------------------------------------------------------------------------------
# Tally the log.
# ---------------------------------------------------------------------------------------------
_RECORD_TYPES = ("IPRR", "IPRW", "REGR", "REGW")


def tally_log(logpath):
    counts = {t: Counter() for t in _RECORD_TYPES}
    intd_count = 0
    with open(logpath, "r", errors="replace") as f:
        for line in f:
            for t in _RECORD_TYPES:
                idx = line.find(t)
                if idx < 0:
                    continue
                rest = line[idx + len(t):].split()
                if rest:
                    try:
                        counts[t][int(rest[0], 16)] += 1
                    except ValueError:
                        pass
                break
            else:
                if "INTD" in line:
                    intd_count += 1
    return counts, intd_count


def sum_in_range(counter, lo, hi):
    return sum(v for k, v in counter.items() if lo <= k < hi)


def count_exact(counter, key):
    return counter.get(key, 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--simh", required=True, help="path to the patched microvax3900 binary")
    ap.add_argument("--src", required=True,
                     help="path to that binary's open-simh SOURCE tree (patched VAX/*.c/.h) -- "
                          "used to derive every family's address/IPR-number, never hand-typed")
    ap.add_argument("--runlimit", default="200M instructions")
    ap.add_argument("--timeout", type=int, default=120)
    ap.add_argument("--keep-log", help="copy the raw debug log here instead of deleting it")
    args = ap.parse_args()

    vax_defs_h = os.path.join(args.src, "VAX", "vax_defs.h")
    vaxmod_defs_h = os.path.join(args.src, "VAX", "vaxmod_defs.h")
    vax_sysdev_c = os.path.join(args.src, "VAX", "vax_sysdev.c")
    for p in (vax_defs_h, vaxmod_defs_h, vax_sysdev_c):
        if not os.path.isfile(p):
            raise SystemExit("FATAL: expected source file not found: %s" % p)

    defs_h = load_defines(vax_defs_h)
    mod_h = load_defines(vaxmod_defs_h)
    cache = {}

    def need(name, table):
        if name not in table:
            raise SystemExit(
                "FATAL: #define %s not found where expected -- source shape changed, "
                "re-derive rather than assume the old value still applies." % name)
        return name

    MT_ICCS = int(defs_h[need("MT_ICCS", defs_h)], 0)
    MT_TODR = int(defs_h[need("MT_TODR", defs_h)], 0)
    MT_RXCS = int(defs_h[need("MT_RXCS", defs_h)], 0)
    MT_RXDB = int(defs_h[need("MT_RXDB", defs_h)], 0)
    MT_TXCS = int(defs_h[need("MT_TXCS", defs_h)], 0)
    MT_TXDB = int(defs_h[need("MT_TXDB", defs_h)], 0)

    KABASE = resolve(need("KABASE", mod_h), mod_h, cache)
    KASIZE = resolve(need("KASIZE", mod_h), mod_h, cache)
    CQBICBASE = resolve(need("CQBICBASE", mod_h), mod_h, cache)
    CQBICSIZE = resolve(need("CQBICSIZE", mod_h), mod_h, cache)
    CMCTLBASE = resolve(need("CMCTLBASE", mod_h), mod_h, cache)
    CMCTLSIZE = resolve(need("CMCTLSIZE", mod_h), mod_h, cache)
    SSCBASE = resolve(need("SSCBASE", mod_h), mod_h, cache)

    t0_min, t1_max = ssc_timer_offset_range(vax_sysdev_c)
    SSC_TMR_LO = SSCBASE + t0_min * 4
    SSC_TMR_HI = SSCBASE + (t1_max + 1) * 4

    logpath = boot_and_capture(args.simh, args.runlimit, args.timeout)
    try:
        counts, intd_count = tally_log(logpath)
        if args.keep_log:
            import shutil
            shutil.copy(logpath, args.keep_log)
    finally:
        os.unlink(logpath)

    iprr, iprw, regr, regw = counts["IPRR"], counts["IPRW"], counts["REGR"], counts["REGW"]

    # (name, read-count, write-count, require-read, require-write) -- required directions are
    # the measured behaviour of a real boot-to->>> run (see README.md "What 0006 adds"), not a
    # claim about what the hardware could theoretically do in either direction.
    families = [
        ("ICCS",      count_exact(iprr, MT_ICCS), count_exact(iprw, MT_ICCS), False, True),
        ("TODR",      count_exact(iprr, MT_TODR), count_exact(iprw, MT_TODR), True, False),
        ("RXCS",      count_exact(iprr, MT_RXCS), count_exact(iprw, MT_RXCS), True, True),
        ("RXDB",      count_exact(iprr, MT_RXDB), count_exact(iprw, MT_RXDB), True, False),
        ("TXCS",      count_exact(iprr, MT_TXCS), count_exact(iprw, MT_TXCS), True, True),
        ("TXDB",      count_exact(iprr, MT_TXDB), count_exact(iprw, MT_TXDB), True, True),
        ("SSC-T0/T1", sum_in_range(regr, SSC_TMR_LO, SSC_TMR_HI),
                       sum_in_range(regw, SSC_TMR_LO, SSC_TMR_HI), True, True),
        ("CMCTL",     sum_in_range(regr, CMCTLBASE, CMCTLBASE + CMCTLSIZE),
                       sum_in_range(regw, CMCTLBASE, CMCTLBASE + CMCTLSIZE), True, True),
        ("KA",        sum_in_range(regr, KABASE, KABASE + KASIZE),
                       sum_in_range(regw, KABASE, KABASE + KASIZE), True, True),
        ("CQBIC",     sum_in_range(regr, CQBICBASE, CQBICBASE + CQBICSIZE),
                       sum_in_range(regw, CQBICBASE, CQBICBASE + CQBICSIZE), True, True),
    ]

    violations = []
    print("DEVTRACE TALLY (derived from %s):" % args.src)
    for name, r, w, req_r, req_w in families:
        print("  %-10s read=%-10d (required=%-5s)  write=%-10d (required=%-5s)"
              % (name, r, req_r, w, req_w))
        if req_r and r == 0:
            violations.append("%s READ" % name)
        if req_w and w == 0:
            violations.append("%s WRITE" % name)
    print("  %-10s count=%-9d (required=%-5s)" % ("INTD", intd_count, True))
    if intd_count == 0:
        violations.append("INTD")

    if violations:
        sys.stderr.write(
            "DEVTRACE TALLY FAILED -- the following required family/direction(s) reached ZERO "
            "events (a family with zero events is a FAILURE of this item, not a property of the "
            "ROM):\n")
        for v in violations:
            sys.stderr.write("  - %s\n" % v)
        sys.exit(1)

    print("DEVTRACE TALLY PASSED -- every required family/direction reachable.")
    sys.exit(0)


if __name__ == "__main__":
    main()
