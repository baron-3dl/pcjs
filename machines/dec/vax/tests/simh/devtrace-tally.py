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

pcjsvax-62a (2026-07-26) extended patch 0006 with two more families, QIOR/QIOW, at ReadIO/WriteIO/
ReadIOU/WriteIOU in vax_io.c -- the choke point vax_mmu.h's ReadB/W/L/WriteB/W/L branch to for the
THIRD way a physical access can go (ADDR_IS_IO, neither RAM nor the ReadReg/WriteReg register-space
"else" branch 0006 already covered): Qbus I/O-page space (ADDR_IS_IOP, 0x20000000, dispatched
through iodispR[]/iodispW[] to rl/rq/rqb/rqc/rqd/ts/tq/xq/xqb/dz/dpv -- see vax_syslist.c) and Qbus
memory-window space (ADDR_IS_CQM, 0x30000000, dispatched to cqm_rd/cqm_wr). Both are reached via
ReadQb/WriteQb from the same four functions, so one hook per direction covers both branches, same
as 0006's original ReadReg/WriteReg design. The two new families below are named QBIOP and QBCQM,
one per branch -- NOT one per named Qbus device (rl/rq/ts/tq/xq/dz/...): a no-disk, no-media boot
to >>> genuinely never touches those controllers' own registers at all (measured: RL 20001900-1909,
RQ 20001468-146B, TS 20001550-1553, TQ 20001940-1943, XQ 20001920-192F and DZ 20000040-005F all show
ZERO reads and ZERO writes in a real boot with nothing attached), so a per-device floor would be a
FALSE floor -- exactly the "a direction the ROM never exercises" trap this file's own docstring
already warns against, one level up (a whole family the ROM never exercises, not just one
direction). What the no-disk boot DOES exercise, every time, is the QBA's own doorbell register
(0x20001F40, inside the IOP range, structurally identical dispatch path to rl/rq/etc. -- same
iodispR[]/iodispW[] table, same ReadIO/WriteIO hook, no per-device special-casing exists anywhere
in this patch) and the CQM Qbus-memory window (heavy traffic from the console's own memory-mapping
setup). Both are measured nonzero in BOTH directions on a real boot (see README.md "What 0006
adds"), which is what QBIOP/QBCQM's floors below require.

SECOND veracity re-dispatch (2026-07-26, same day): the QBIOP/QBCQM boot-derived floors above cover
only two of the FOUR hooks pcjsvax-62a's patch actually added -- ReadIO and WriteIO (the ALIGNED
Qbus path). ReadIOU and WriteIOU (the UNALIGNED path -- reached whenever a Qbus access straddles a
word/longword boundary, e.g. a longword read starting at an odd address) had NO coverage assertion
anywhere. Proven by construction: a mutation silencing only ReadIOU's guard, and a separate one
silencing only WriteIOU's guard, each apply clean, build clean, pass EHKAA, and PASSED this tally at
exit 0 -- because the no-disk boot-to->>> never enters the unaligned path at all (QBIOP/QBCQM's
byte-for-byte counts are IDENTICAL between the good binary and both mutants). A boot-derived floor
cannot close this: requiring ">0 unaligned events" from the boot would be exactly the false-floor /
rule-11 trap this file already avoids for the per-device families, one level up again.

The fix (`unaligned_probe()` below) is a short, DETERMINISTIC, non-boot probe run on the same
binary after the boot-derived tally: derive an I/O-page device's CSR address programmatically (by
running `SHOW RL` against the binary under test and parsing its own reported address range -- never
hand-typed, standing rule 5), hand-assemble two `MOVL` instructions (`MOVL @#<csr+1>,R0` and
`MOVL R0,@#<csr+1>` -- deliberately misaligned by one byte) into scratch RAM, deposit PC/PSL, and
step exactly two instructions. A longword access starting one byte off a word boundary is exactly
the case ReadIOU's/WriteIOU's own header comments describe (their "tribyte" case): it produces one
QIOR/QIOW record with `size=00000003`, a value ReadIO/WriteIO can NEVER emit (their `lnt` parameter
is restricted to L_BYTE/L_WORD/L_LONG/L_QUAD = 1/2/4/8 by every call site in vax_mmu.h -- see
ReadIO's/WriteIO's own doc comments in vax_io.c). A `size=00000003` record at the probed address is
therefore unambiguous, unfakeable proof that ReadIOU/WriteIOU specifically fired, not merely that
SOME Qbus access happened. Missing either one exits non-zero and names exactly which path (read or
write) produced nothing -- see `unaligned_probe()`'s own docstring for the precalibration hazard
this probe has to route around (SIMH deposits and runs a one-time clock-precalibration self-test at
addresses 0x100-0x10D on the FIRST GO/STEP/RUN of any session, which would otherwise clobber this
probe's own PC/PSL setup if not absorbed first).

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
# pcjsvax-62a veracity re-dispatch: deterministic unaligned-Qbus-access probe.
#
# The boot-derived QBIOP/QBCQM floors above never exercise ReadIOU/WriteIOU (the boot's own I/O
# traffic is entirely aligned), so they cannot prove those two hooks are alive. This probe drives
# one genuinely unaligned access directly, deterministically, with no dependency on ROM behavior,
# wall-clock timing, or run length.
# ---------------------------------------------------------------------------------------------
_SHOW_DEV_TEMPLATE = "set console notelnet\nshow {dev}\nexit\n"
_DEV_ADDR_RE = re.compile(r'address=([0-9A-Fa-f]+)-[0-9A-Fa-f]+')


def get_device_csr_base(simh_bin, dev, timeout):
    """Derive dev's Qbus CSR base address by parsing the BINARY'S OWN `SHOW <dev>` output --
    never hand-typed (standing rule 5). The autoconfigured address is assigned at runtime by
    build_dib_tab()/auto_config() and could shift if the device list or configuration changes."""
    inifd, inipath = tempfile.mkstemp(prefix="pcjs-devtrace-show-", suffix=".ini")
    with os.fdopen(inifd, "w") as f:
        f.write(_SHOW_DEV_TEMPLATE.format(dev=dev))
    try:
        proc = subprocess.run([simh_bin, inipath], capture_output=True, text=True, timeout=timeout)
    finally:
        os.unlink(inipath)
    m = _DEV_ADDR_RE.search(proc.stdout)
    if not m:
        raise SystemExit(
            "FATAL: could not parse a Qbus address out of `SHOW %s` -- device disabled, renamed, "
            "or SIMH's SHOW format changed. Output was:\n%s" % (dev, proc.stdout))
    return int(m.group(1), 16)


def _movl_absolute_bytes(csr_addr):
    """Hand-assemble two VAX MOVL instructions targeting csr_addr+1 (one byte off any word/
    longword boundary, so every access against it is unaligned by construction):

        MOVL @#<csr_addr+1>,R0    D0 9F <addr LE32> 50
        MOVL R0,@#<csr_addr+1>    D0 50 9F <addr LE32>

    Opcode 0xD0 (MOVL) and the addressing-mode nibbles (0x9_ autoincrement-deferred/absolute,
    0x5_ register-direct) are fixed VAX architecture constants frozen since DEC's original VAX
    Architecture Reference Manual -- not project-specific data subject to rebase drift the way a
    device's CSR address is. Verified empirically against this exact patched binary: SIMH's own
    `EXAMINE -M` disassembles these bytes back as `MOVL @#<addr>,R0` / `MOVL R0,@#<addr>`, and
    stepping them produces exactly the QIOR/QIOW records this probe asserts on.

    A longword access one byte off alignment is precisely the "tribyte" case ReadIOU's and
    WriteIOU's own header comments describe (see vax_io.c) -- it cannot be satisfied by a single
    aligned Qbus word access, so the CPU's own unaligned-access path (ReadU/WriteU in vax_mmu.h)
    is the only way either instruction can complete, regardless of what this patch does or does
    not instrument. That is what makes the resulting `size=00000003` record unfakeable proof of
    the unaligned path specifically, not merely of some Qbus access having happened.
    """
    addr = csr_addr + 1
    addr_le = list(addr.to_bytes(4, "little"))
    read_instr = [0xD0, 0x9F] + addr_le + [0x50]           # MOVL @#addr,R0
    write_instr = [0xD0, 0x50, 0x9F] + addr_le             # MOVL R0,@#addr
    return read_instr + write_instr, addr


_PROBE_BASE = 0x1000  # scratch RAM, well clear of the 0x100-0x10D precalibration scratch code

_UNALIGNED_PROBE_TEMPLATE = """\
set console notelnet
step 1
set cpu debug=EXCTRACE
set sysd debug=DEVTRACE
set debug -n {logfile}
{deposits}
deposit PC {base:X}
deposit PSL 0
step 2
examine PC
exit
"""


def unaligned_probe(simh_bin, csr_addr, timeout):
    """Deposit two hand-assembled unaligned MOVL instructions at _PROBE_BASE, step exactly two
    instructions, and report whether a QIOR and a QIOW record with size=00000003 (tribyte -- see
    _movl_absolute_bytes' docstring) appeared at csr_addr+1.

    The leading `step 1` (before any of our own deposits) is required and load-bearing: SIMH's
    generic clock-precalibration self-test (sim_timer_precalibrate_execution_rate(), wired in via
    cpu_reset()'s `sim_clock_precalibrate_commands`) deposits ITS OWN throwaway code at physical
    addresses 0x100-0x10D and runs it on the FIRST GO/STEP/RUN of any process, regardless of what
    the caller asked for. Skipping this absorption step does not corrupt our deposits (they land
    at _PROBE_BASE = 0x1000, nowhere near 0x100-0x10D) -- it corrupts the *step count*: without
    it, the requested `step 2` is consumed by (part of) the precalibration loop instead of our
    instructions, and PC never reaches 0x1000 at all. This was found and fixed by direct execution
    trace instrumentation of get_istr()/ReadLP() during this item's own development -- see the
    module docstring's veracity-re-dispatch note.

    Returns (read_ok: bool, write_ok: bool, raw_log: str).
    """
    instr_bytes, unaligned_addr = _movl_absolute_bytes(csr_addr)
    deposit_lines = "\n".join(
        "deposit -b %X %X" % (_PROBE_BASE + i, b) for i, b in enumerate(instr_bytes))

    logfd, logpath = tempfile.mkstemp(prefix="pcjs-unaligned-probe-", suffix=".log")
    os.close(logfd)
    inifd, inipath = tempfile.mkstemp(prefix="pcjs-unaligned-probe-", suffix=".ini")
    with os.fdopen(inifd, "w") as f:
        f.write(_UNALIGNED_PROBE_TEMPLATE.format(
            logfile=logpath, deposits=deposit_lines, base=_PROBE_BASE))
    try:
        try:
            proc = subprocess.run([simh_bin, inipath], capture_output=True, text=True,
                                   timeout=timeout)
        except subprocess.TimeoutExpired:
            raise SystemExit(
                "FATAL: unaligned-probe run of %s did not return within %ds -- hung. Cannot "
                "assert ReadIOU/WriteIOU coverage." % (simh_bin, timeout))
        expected_end_pc = "%08X" % (_PROBE_BASE + len(instr_bytes))
        if expected_end_pc not in proc.stdout:
            raise SystemExit(
                "FATAL: unaligned probe did not complete both instructions -- expected PC=%s "
                "after `step 2`, not found in output:\n%s" % (expected_end_pc, proc.stdout))
        with open(logpath, "r", errors="replace") as f:
            raw_log = f.read()
    finally:
        os.unlink(inipath)
        if os.path.exists(logpath):
            os.unlink(logpath)

    read_needle = "QIOR %08X 00000003" % unaligned_addr
    write_needle = "QIOW %08X 00000003" % unaligned_addr
    return (read_needle in raw_log, write_needle in raw_log, raw_log)


# ---------------------------------------------------------------------------------------------
# Tally the log.
# ---------------------------------------------------------------------------------------------
_RECORD_TYPES = ("IPRR", "IPRW", "REGR", "REGW", "QIOR", "QIOW")


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

    # pcjsvax-62a: the two Qbus branches ADDR_IS_IO() covers -- IOPAGEBASE/IOPAGESIZE (ADDR_IS_IOP,
    # the I/O-page dispatched through iodispR[]/iodispW[] to rl/rq/ts/tq/xq/dz/... and the QBA's own
    # doorbell register) and CQMBASE/CQMSIZE (ADDR_IS_CQM, the Qbus memory window dispatched to
    # cqm_rd/cqm_wr). Both #define'd in vaxmod_defs.h, resolved the same way as every other base/size
    # pair above -- never hand-typed.
    IOPAGEBASE = resolve(need("IOPAGEBASE", mod_h), mod_h, cache)
    IOPAGESIZE = resolve(need("IOPAGESIZE", mod_h), mod_h, cache)
    CQMBASE = resolve(need("CQMBASE", mod_h), mod_h, cache)
    CQMSIZE = resolve(need("CQMSIZE", mod_h), mod_h, cache)

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
    qior, qiow = counts["QIOR"], counts["QIOW"]

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
        # pcjsvax-62a: the two Qbus branches, not one entry per named disk/tape/net/mux device --
        # see this file's module docstring for why a per-device floor here would be a false floor
        # for any no-disk, no-media boot (rl/rq/ts/tq/xq/dz all measure ZERO in both directions).
        # QBIOP is proven nonzero by the QBA's own doorbell register at a fixed IOP address, which
        # goes through the exact same iodispR[]/iodispW[] dispatch every other IOP device does.
        ("QBIOP",     sum_in_range(qior, IOPAGEBASE, IOPAGEBASE + IOPAGESIZE),
                       sum_in_range(qiow, IOPAGEBASE, IOPAGEBASE + IOPAGESIZE), True, True),
        ("QBCQM",     sum_in_range(qior, CQMBASE, CQMBASE + CQMSIZE),
                       sum_in_range(qiow, CQMBASE, CQMBASE + CQMSIZE), True, True),
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

    # pcjsvax-62a veracity re-dispatch: QBIOP/QBCQM above only prove ReadIO/WriteIO (the ALIGNED
    # Qbus path) are alive -- the boot never drives an unaligned access, so it cannot prove
    # ReadIOU/WriteIOU. A short, deterministic, non-boot probe closes that: see unaligned_probe()'s
    # docstring and this file's module docstring for why a boot-derived floor cannot do this job.
    csr_addr = get_device_csr_base(args.simh, "RL", args.timeout)
    unaligned_read_ok, unaligned_write_ok, unaligned_raw_log = unaligned_probe(
        args.simh, csr_addr, args.timeout)
    print("  %-10s read=%-10s (required=%-5s) write=%-10s (required=%-5s)  [probed @ RL CSR+1 = "
          "%08X]" % ("UNALIGNED", unaligned_read_ok, True, unaligned_write_ok, True, csr_addr + 1))
    if not unaligned_read_ok:
        violations.append("UNALIGNED READ")
    if not unaligned_write_ok:
        violations.append("UNALIGNED WRITE")
    if args.keep_log:
        with open(args.keep_log + ".unaligned-probe", "w") as f:
            f.write(unaligned_raw_log)

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
