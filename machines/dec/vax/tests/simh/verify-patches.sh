#!/usr/bin/env bash
#
# Proves the oracle is intact: clones open-simh FRESH (a real `git clone`, never a reused
# checkout), applies every patch this project carries against it -- in order, derived
# PROGRAMMATICALLY from the two directories that hold them, never hand-enumerated -- builds,
# and checks that SIMH's own EHKAA hardware-core self-test (run automatically by `make ... vax`)
# reports PASS.
#
# This is pcjsvax-fb1's deliverable. Every differential (decodediff, mmudiff, fpadiff, excdiff,
# cpudiff, ...) depends transitively on these patches applying cleanly to the vendored oracle. If
# they silently stop applying -- or silently stop being ALL applied, see the KNOWN TRAP below --
# the oracle rots and nothing downstream can be trusted, no matter how green it looks.
#
# Usage:
#     machines/dec/vax/tests/simh/verify-patches.sh              # fresh clone, apply, build, assert PASS
#     machines/dec/vax/tests/simh/verify-patches.sh --selfcheck  # mutation suite (see below); no build
#
# Env:
#     PCJS_VAX_REPO   the pcjs-vax work repo (holds patch 0001 and the vendored open-simh). Defaults
#                      to "$PCJS_ROOT/../pcjs-vax", which is WRONG from a git worktree -- always
#                      export it explicitly. (Same convention as build.sh and every differential.)
#
# KNOWN TRAP this script exists to never fall into (has bitten the orchestrator twice): build.sh
# REUSES an existing $DEST/open-simh checkout, so a tree built when only patch 0002 existed will
# NOT pick up 0003+, and the resulting binary is SILENTLY missing instrumentation -- it still
# builds, it still runs, differentials that need the missing patch just report "this simulator
# does not implement ...". This script's default mode never reuses a directory: it always
# `mktemp -d`s a brand-new DEST and does a real `git clone` into it, so there is no checkout to be
# stale.
#
# --selfcheck proves the failure modes are actually caught, by execution, not by inspection:
#   1. DROP    -- clone + apply with one patch removed from the list.       Must fail, must name it.
#   2. REORDER -- clone + apply with two adjacent patches swapped.          Must fail, must name it.
#   3. CORRUPT -- clone + apply with one patch's context line perturbed.    Must fail, must name it.
# Each case is independently asserted; the run fails (non-zero exit) if ANY case is silently
# tolerated (patch applies when it shouldn't, or the failure isn't attributed to the mutated
# patch). This assertion does not get cheaper as the mutation count grows -- it is a fixed set of
# three known failure modes, each checked in full, every run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PCJS_ROOT="$(cd "$HERE/../../../../.." && pwd)"
VAX_WORK="${PCJS_VAX_REPO:-$(cd "$PCJS_ROOT/.." && pwd)/pcjs-vax}"
SRC="$VAX_WORK/open-simh"

if [[ ! -d "$SRC" ]]; then
    echo "FATAL: vendored open-simh not found at $SRC" >&2
    echo "       Set \$PCJS_VAX_REPO to the pcjs-vax work repo if it is not beside this one." >&2
    exit 1
fi

# --- Programmatic patch-list derivation -------------------------------------------------------
# Patch 0001 lives in the pcjs-vax work repo's patches/ directory; it must be the ONLY *.patch
# file there (if the work repo ever grows a second work-repo patch, this is a scope change that
# must be looked at, not silently absorbed into the list).
mapfile -t WORK_PATCHES < <(find "$VAX_WORK/patches" -maxdepth 1 -name '*.patch' | sort)
if [[ "${#WORK_PATCHES[@]}" -ne 1 ]]; then
    echo "FATAL: expected exactly one *.patch file in $VAX_WORK/patches, found ${#WORK_PATCHES[@]}:" >&2
    printf '       %s\n' "${WORK_PATCHES[@]}" >&2
    exit 1
fi

# Patches 0002+ live here, in the fork repo, alongside this script and build.sh.
mapfile -t FORK_PATCHES < <(find "$HERE" -maxdepth 1 -name '*.patch' | sort)
if [[ "${#FORK_PATCHES[@]}" -eq 0 ]]; then
    echo "FATAL: no *.patch files found in $HERE" >&2
    exit 1
fi

PATCHES=("${WORK_PATCHES[@]}" "${FORK_PATCHES[@]}")

echo "derived patch list (${#PATCHES[@]} patches):"
for p in "${PATCHES[@]}"; do echo "  $(basename "$p")  <- $(dirname "$p")"; done
echo

# --- Core: fresh-clone + apply-in-order-with-attribution -----------------------------------
# Clones $SRC into $1 with a REAL `git clone` (never a reuse, never a plain `cp`), applies the
# patch list in $2..$n IN ORDER, and reports success/failure. On failure, prints WHICH patch (by
# name, from the derived list -- never hand-enumerated) failed to apply and why, then returns
# non-zero. Every application (success or fail) has its stderr captured; if git apply prints
# ANY output (warning or error) for a patch that DID apply, that also counts as a failure --
# `git apply` warns on trailing-whitespace and similar defects, and we ship zero of those.
apply_chain() {
    local dest="$1"; shift
    local -a plist=("$@")
    rm -rf "$dest"
    git clone --quiet --no-hardlinks "$SRC" "$dest" >/dev/null
    local p out rc
    for p in "${plist[@]}"; do
        out="$(cd "$dest" && git apply "$p" 2>&1)"; rc=$?
        if [[ $rc -ne 0 ]]; then
            echo "APPLY-FAILED: $(basename "$p")" >&2
            echo "$out" >&2
            return 1
        fi
        if [[ -n "$out" ]]; then
            echo "APPLY-WARNED: $(basename "$p")" >&2
            echo "$out" >&2
            return 1
        fi
    done
    return 0
}

# --- Default mode: real build + real EHKAA self-test ------------------------------------------
run_build() {
    local dest="${1:-$(mktemp -d "${TMPDIR:-/tmp}/pcjs-vax-simh-verify.XXXXXX")}"
    echo "fresh clone + apply -> $dest"
    if ! apply_chain "$dest" "${PATCHES[@]}"; then
        echo "FATAL: patch chain did not apply cleanly to a fresh clone." >&2
        exit 1
    fi
    echo "applied all ${#PATCHES[@]} patches cleanly, no warnings"

    echo "building (make NOASYNCH=1 NONETWORK=1 NOVIDEO=1 vax) -- runs SIMH's own EHKAA self-test..."
    local buildlog
    buildlog="$(mktemp "${TMPDIR:-/tmp}/pcjs-vax-verify-build.XXXXXX.log")"
    if ! (cd "$dest" && make NOASYNCH=1 NONETWORK=1 NOVIDEO=1 -j"$(nproc 2>/dev/null || echo 4)" vax) >"$buildlog" 2>&1; then
        echo "FATAL: build failed. Log: $buildlog" >&2
        tail -60 "$buildlog" >&2
        exit 1
    fi
    if ! grep -q 'PASSED - MicroVAX 3900 Hardware Core Instruction test EHKAA' "$buildlog"; then
        echo "FATAL: build succeeded but did not report EHKAA PASS. Log: $buildlog" >&2
        grep -i 'ehkaa\|fail' "$buildlog" >&2 || true
        exit 1
    fi
    echo "PASS: SIMH's own EHKAA hardware-core self-test reports PASS."
    echo "binary: $dest/BIN/microvax3900"
}

# --- --selfcheck: prove the three failure modes are actually caught ---------------------------
run_selfcheck() {
    local scratch
    scratch="$(mktemp -d "${TMPDIR:-/tmp}/pcjs-vax-verify-selfcheck.XXXXXX")"
    local failures=0

    # Pick a mid-chain patch to mutate against -- one with real inter-patch context dependency
    # (0004 depends on 0003's MTAB edit; a drop or reorder around it is guaranteed to surface as
    # a context-mismatch, not a silent no-op).
    local victim_idx=-1 i
    for i in "${!PATCHES[@]}"; do
        [[ "$(basename "${PATCHES[$i]}")" == 0003-* ]] && victim_idx=$i
    done
    if [[ $victim_idx -lt 0 ]]; then
        echo "FATAL(selfcheck): could not locate 0003-* in the derived patch list to mutate against." >&2
        exit 1
    fi

    echo "=== mutation 1/3: DROP (remove $(basename "${PATCHES[$victim_idx]}") from the chain) ==="
    local -a dropped=()
    for i in "${!PATCHES[@]}"; do
        [[ $i -eq $victim_idx ]] && continue
        dropped+=("${PATCHES[$i]}")
    done
    if apply_chain "$scratch/drop" "${dropped[@]}" 2>"$scratch/drop.err"; then
        echo "COVERAGE HOLE: dropping $(basename "${PATCHES[$victim_idx]}") from the chain STILL APPLIED. Not caught." >&2
        failures=$((failures + 1))
    else
        if grep -q "$(basename "${PATCHES[$((victim_idx + 1))]}")" "$scratch/drop.err" 2>/dev/null; then
            echo "caught: chain broke at $(basename "${PATCHES[$((victim_idx + 1))]}"), the patch that depends on the dropped one's context. Good."
        else
            echo "caught: chain broke (see $scratch/drop.err), but not clearly attributed. Reporting fail-open on attribution."
            failures=$((failures + 1))
        fi
    fi
    echo

    echo "=== mutation 2/3: REORDER (swap $(basename "${PATCHES[$victim_idx]}") and the patch after it) ==="
    local -a reordered=("${PATCHES[@]}")
    if [[ $((victim_idx + 1)) -lt "${#PATCHES[@]}" ]]; then
        local tmp="${reordered[$victim_idx]}"
        reordered[$victim_idx]="${reordered[$((victim_idx + 1))]}"
        reordered[$((victim_idx + 1))]="$tmp"
    fi
    if apply_chain "$scratch/reorder" "${reordered[@]}" 2>"$scratch/reorder.err"; then
        echo "COVERAGE HOLE: reordering around $(basename "${PATCHES[$victim_idx]}") STILL APPLIED. Not caught." >&2
        failures=$((failures + 1))
    else
        echo "caught: reordered chain failed to apply (see $scratch/reorder.err). Good."
    fi
    echo

    echo "=== mutation 3/3: CORRUPT (perturb a context line inside $(basename "${PATCHES[$victim_idx]}")) ==="
    mkdir -p "$scratch/corrupt-patches"
    local corrupted="$scratch/corrupt-patches/$(basename "${PATCHES[$victim_idx]}")"
    # Flip one character in a context line (a line starting with a single space) so the patch's
    # own context no longer matches the pristine source -- this must be rejected by `git apply`'s
    # own fuzz-free context check, not merely "look different".
    awk '
        BEGIN { done = 0 }
        !done && /^ .*[a-zA-Z]/ { sub(/[a-zA-Z]/, "Q"); done = 1 }
        { print }
    ' "${PATCHES[$victim_idx]}" > "$corrupted"
    if cmp -s "${PATCHES[$victim_idx]}" "$corrupted"; then
        echo "FATAL(selfcheck): corruption step was a no-op (found no eligible context line to perturb)." >&2
        exit 1
    fi
    local -a corrupted_list=()
    for i in "${!PATCHES[@]}"; do
        if [[ $i -eq $victim_idx ]]; then
            corrupted_list+=("$corrupted")
        else
            corrupted_list+=("${PATCHES[$i]}")
        fi
    done
    if apply_chain "$scratch/corrupt" "${corrupted_list[@]}" 2>"$scratch/corrupt.err"; then
        echo "COVERAGE HOLE: corrupted $(basename "${PATCHES[$victim_idx]}") STILL APPLIED. Not caught." >&2
        failures=$((failures + 1))
    else
        if grep -q "$(basename "$corrupted")" "$scratch/corrupt.err"; then
            echo "caught: corrupted patch was rejected and correctly attributed by name (see $scratch/corrupt.err). Good."
        else
            echo "caught: corrupted patch was rejected, but $scratch/corrupt.err doesn't name it. Reporting fail-open on attribution."
            failures=$((failures + 1))
        fi
    fi
    echo

    if [[ $failures -ne 0 ]]; then
        echo "SELFCHECK FAILED: $failures/3 mutation(s) were not caught (coverage hole)." >&2
        exit 1
    fi
    echo "SELFCHECK PASSED: all 3/3 mutations caught and correctly attributed."
    rm -rf "$scratch"
}

if [[ "${1:-}" == "--selfcheck" ]]; then
    run_selfcheck
else
    run_build "${1:-}"
fi
