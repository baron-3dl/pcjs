#!/usr/bin/env bash
#
# Builds the Open SIMH microvax3900 that machines/dec/vax/tests/decodediff.js,
# machines/dec/vax/tests/mmudiff.js and machines/dec/vax/tests/fpadiff.js grade against.
#
# The stock simulator does not record enough to replay operand resolution: its instruction
# history has the resolved operands and the register file AFTER resolution, but not the register
# file BEFORE it, not the instruction bytes in a machine-readable form, and -- most importantly --
# not the memory reads the addressing modes performed.  Nor can its console observe a translation
# for anything but a READ in one access mode, nor can it perform one floating point operation on
# operands you choose.  Four patches add what is missing:
#
#   0001  (lives in the pcjs-vax work repo) adds the R0-R15 dump: the "REGS" line.
#   0002  (here) adds the decode-replay capture: "PREG", "IBYT", "OPND", "RECQ", "MEMR", "BRDP".
#   0003  (here) adds SHOW CPU MMUOP= (one Read/Write/Test/MTPR, machine-readable) and
#         SET CPU MMUTRACE= (a log of every MMU operation, PTE read and TB flush), plus a -W
#         switch so SHOW CPU VIRTUAL= can probe WRITE access.
#   0004  (here) adds SHOW CPU FPOP=, which runs one F/D/G floating instruction -- the real
#         op_* routines, the real condition-code macros and the real WRITE_B/W/L/Q store -- on
#         an operand queue you supply, and reports the result, the condition codes, the trap
#         request and any abort.
#   0005  (here) adds the EXCTRACE debug category: a machine-readable dump of the complete
#         privileged-register state at every intexc(), op_rei(), op_chm() and op_mtpr()/op_mfpr(),
#         which is what excdiff.js replays.  The state a dispatch depends on (STK[], IS, SCBB)
#         is simulator-internal and appears in NO existing log, so without this the EHKAA
#         exception sequence cannot be graded at all.
#   0006  (here) adds the DEVTRACE debug category (IPRR/IPRW at ReadIPR/WriteIPR, REGR/REGW at
#         ReadReg/WriteReg, and -- as extended by pcjsvax-62a -- QIOR/QIOW at ReadIO/WriteIO/
#         ReadIOU/WriteIOU) and extends 0005's EXCTRACE with an INTD record at the IE_INT call
#         site in sim_instr()'s dispatch loop -- one machine-readable line per KA655 device
#         register access (console UART, TODR, SSC timers, CMCTL, KA, CQBIC/CQMAP/CQIPC/CDG,
#         Qbus I/O-page and Qbus-memory-window traffic) and per delivered interrupt (vector + IPL),
#         which is the oracle the device-emulation items after the EHKAA milestone grade against.
#         NOTE: INTD only observes DELIVERED interrupts -- SET_INT/CLR_INT are macros with no call
#         site to hook, so a masked interrupt REQUEST produces no trace record at all until (if
#         ever) it is later delivered. See tests/simh/README.md's "CAVEAT" section.
#
# None changes any instruction semantics; the simulator's own EHKAA self-test still passes,
# which the build runs automatically.
#
# Usage:
#     machines/dec/vax/tests/simh/build.sh [DEST_DIR]
#
# DEST_DIR defaults to $TMPDIR/pcjs-vax-simh.  The binary lands at DEST_DIR/open-simh/BIN/
# microvax3900; export it as $SIMH_DECODE_BIN / $SIMH_MMU_BIN / $SIMH_FP_BIN, or pass it to any
# of the tests with --simh.
#
# IF YOU ADD A PATCH, DELETE THE DEST FIRST.  An existing DEST/open-simh is REUSED as-is (see
# below), so a new patch is silently absent from the binary and every test that needs it fails
# with "this simulator does not implement ...".  `rm -rf "$DEST"` and re-run.
#
# The vendored open-simh checkout in the pcjs-vax work repo is COPIED, never built in place:
# other tooling measures against the unpatched binary there, and the patches README in that repo
# asks for exactly this.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCJS_ROOT="$(cd "$HERE/../../../../.." && pwd)"
VAX_WORK="${PCJS_VAX_REPO:-$(cd "$PCJS_ROOT/.." && pwd)/pcjs-vax}"
DEST="${1:-${TMPDIR:-/tmp}/pcjs-vax-simh}"

SRC="$VAX_WORK/open-simh"
PATCH1="$VAX_WORK/patches/0001-inst-history-add-register-file.patch"
PATCH2="$HERE/0002-inst-history-decode-replay.patch"
PATCH3="$HERE/0003-mmu-differential-support.patch"
PATCH4="$HERE/0004-fp-differential-support.patch"
PATCH5="$HERE/0005-exception-differential-support.patch"
PATCH6="$HERE/0006-device-register-trace.patch"

for f in "$SRC" "$PATCH1" "$PATCH2" "$PATCH3" "$PATCH4" "$PATCH5" "$PATCH6"; do
    if [[ ! -e "$f" ]]; then
        echo "FATAL: required input not found: $f" >&2
        echo "       Set \$PCJS_VAX_REPO to the pcjs-vax work repo if it is not beside this one." >&2
        exit 1
    fi
done

mkdir -p "$DEST"
if [[ -L "$DEST/open-simh" ]]; then
    echo "FATAL: $DEST/open-simh is a symlink (possibly left over from a build that hit the" >&2
    echo "       cp -a/symlink hazard this script now guards against) -- refusing to build" >&2
    echo "       through it. rm -rf \"$DEST\" and re-run." >&2
    exit 1
elif [[ -d "$DEST/open-simh" ]]; then
    echo "reusing existing checkout at $DEST/open-simh (delete it to start clean)"
else
    echo "copying $SRC -> $DEST/open-simh"
    # $SRC may itself be a symlink (a worktree's own open-simh is set up that way, pointing at
    # the pristine vendor checkout it must never write through). "cp -a SRC DEST" on a symlink
    # SOURCE copies the symlink itself, not the directory it points to -- "-a" implies "-d"
    # (--no-dereference), so a naive "cp -a $SRC $DEST/open-simh" makes $DEST/open-simh a symlink
    # BACK to the pristine checkout, and every subsequent "git apply"/"make" in this script then
    # silently patches and builds the pristine tree instead of a disposable copy. The trailing
    # "/." forces cp to resolve that top-level symlink and copy its CONTENTS; internal symlinks
    # inside the tree (if any) are still preserved as symlinks, since "-a" is otherwise unchanged.
    mkdir -p "$DEST/open-simh"
    cp -a "$SRC/." "$DEST/open-simh"
    if [[ -L "$DEST/open-simh" ]]; then
        echo "FATAL: $DEST/open-simh is a symlink after copy -- refusing to build/apply through it." >&2
        exit 1
    fi
    cd "$DEST/open-simh"
    git checkout -- VAX/vax_cpu.c VAX/vax_defs.h VAX/vax_mmu.c VAX/vax_mmu.h VAX/vax_sysdev.c 2>/dev/null || true
    git apply "$PATCH1"
    git apply "$PATCH2"
    git apply "$PATCH3"
    git apply "$PATCH4"
    git apply "$PATCH5"
    git apply "$PATCH6"
    echo "applied 0001, 0002, 0003, 0004, 0005 and 0006"
fi

cd "$DEST/open-simh"
make NOASYNCH=1 NONETWORK=1 NOVIDEO=1 -j"$(nproc 2>/dev/null || echo 4)" vax

echo
echo "built: $DEST/open-simh/BIN/microvax3900"
echo "run the differentials with:"
echo "    node machines/dec/vax/tests/decodediff.js --simh $DEST/open-simh/BIN/microvax3900"
echo "    node machines/dec/vax/tests/mmudiff.js    --simh $DEST/open-simh/BIN/microvax3900"
echo "    node machines/dec/vax/tests/fpadiff.js    --simh $DEST/open-simh/BIN/microvax3900"
echo "    node machines/dec/vax/tests/excdiff.js    --simh $DEST/open-simh/BIN/microvax3900"
