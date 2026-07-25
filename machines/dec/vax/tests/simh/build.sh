#!/usr/bin/env bash
#
# Builds the Open SIMH microvax3900 that machines/dec/vax/tests/decodediff.js grades against.
#
# The stock simulator does not record enough to replay operand resolution: its instruction
# history has the resolved operands and the register file AFTER resolution, but not the register
# file BEFORE it, not the instruction bytes in a machine-readable form, and -- most importantly --
# not the memory reads the addressing modes performed.  Two patches add those:
#
#   0001  (lives in the pcjs-vax work repo) adds the R0-R15 dump: the "REGS" line.
#   0002  (here) adds the decode-replay capture: "PREG", "IBYT", "OPND", "RECQ", "MEMR", "BRDP".
#
# Neither changes any instruction semantics; the simulator's own EHKAA self-test still passes,
# which the build runs automatically.
#
# Usage:
#     machines/dec/vax/tests/simh/build.sh [DEST_DIR]
#
# DEST_DIR defaults to $TMPDIR/pcjs-vax-simh.  The binary lands at DEST_DIR/open-simh/BIN/
# microvax3900; export it as $SIMH_DECODE_BIN, or pass it to decodediff.js with --simh.
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

for f in "$SRC" "$PATCH1" "$PATCH2"; do
    if [[ ! -e "$f" ]]; then
        echo "FATAL: required input not found: $f" >&2
        echo "       Set \$PCJS_VAX_REPO to the pcjs-vax work repo if it is not beside this one." >&2
        exit 1
    fi
done

mkdir -p "$DEST"
if [[ -d "$DEST/open-simh" ]]; then
    echo "reusing existing checkout at $DEST/open-simh (delete it to start clean)"
else
    echo "copying $SRC -> $DEST/open-simh"
    cp -a "$SRC" "$DEST/open-simh"
    cd "$DEST/open-simh"
    git checkout -- VAX/vax_cpu.c VAX/vax_defs.h 2>/dev/null || true
    git apply "$PATCH1"
    git apply "$PATCH2"
    echo "applied 0001 and 0002"
fi

cd "$DEST/open-simh"
make NOASYNCH=1 NONETWORK=1 NOVIDEO=1 -j"$(nproc 2>/dev/null || echo 4)" vax

echo
echo "built: $DEST/open-simh/BIN/microvax3900"
echo "run the differential with:"
echo "    node machines/dec/vax/tests/decodediff.js --simh $DEST/open-simh/BIN/microvax3900"
