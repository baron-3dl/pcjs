# Decode-replay trace capture

`decodediff.js` grades our operand-specifier decoder against a real Open SIMH
`microvax3900`. Doing that requires the simulator to record enough for the
decode to be *replayed*, not merely inspected. This directory holds the patch
that makes it, and the script that builds it.

```
machines/dec/vax/tests/simh/build.sh          # -> $TMPDIR/pcjs-vax-simh/open-simh/BIN/microvax3900
export SIMH_DECODE_BIN=$TMPDIR/pcjs-vax-simh/open-simh/BIN/microvax3900
node machines/dec/vax/tests/decodediff.js
```

## Why a patch at all

Decode and operand resolution are a pure function of exactly three inputs:

1. the register file **before** the instruction,
2. the instruction bytes,
3. whatever memory the addressing modes read.

Stock SIMH's instruction history records none of the three. It records the
*outputs* — the resolved operand queue, the PSL, and (with patch 0001, which
lives in the `pcjs-vax` work repo) the register file **after** resolution.
Without the inputs there is nothing to replay against, and without the read log
an effective-address bug is only detectable through the value it happened to
return, which is a probabilistic test rather than an exact one.

## What 0002 adds

Six lines per instruction record, emitted only when the history was armed with
the `-D` switch (`set -d cpu history=<n>:<logfile>`), so the stock format is
unchanged for anything else parsing it:

| Line   | Contents |
|--------|----------|
| `PREG` | `R0`–`R15` immediately **before** the opcode fetch. `R15` equals the record's `iPC`. |
| `IBYT` | length, then that many instruction bytes — opcode, specifiers, immediates, branch displacement. |
| `OPND` | count, then the resolved operand queue `opnd[0..j-1]`. |
| `RECQ` | count, then the recovery queue `recq[0..recqptr-1]`. |
| `MEMR` | count, then one `address length value write-access` quadruple per data read performed while resolving specifiers. |
| `BRDP` | the branch displacement as fetched. **Not reset between instructions**; it is only meaningful for an opcode whose decode ROM row ends in `BB`/`BW`. |

`MEMR` is the one that matters most. It is captured by wrapping `Read()` — a
static inline in `vax_mmu.h` — in `vax_cpu.c` and `#define`-ing the name, so the
log covers `ReadOcta()` and all fifty-odd specifier cases without touching the
MMU or any call site. The log is armed only at the top of the specifier loop,
and the history record is taken before the instruction body runs, so reads made
by the body cannot contaminate it. Instruction-stream fetches go through
`get_istr()`/`ReadLP()` and deliberately do **not** appear in it — they are
replayed from `IBYT` instead.

`DEC_RLOG_MAX` (40) bounds how many reads are stored, but `rlogn` counts them
all, so an overflow is detected by the replay rather than silently truncating.

## What is deliberately *not* in the trace

An instruction that faults **during** resolution never reaches the history
record — SIMH's `ABORT()` longjmps past it. So the recovery-queue unwind cannot
be observed directly. `decodediff.js` observes it indirectly: it points every
SCB vector at its own `HALT`, and the trace record for that `HALT` carries a
`PREG` line showing the register file the exception handler saw, which is the
unwound one (less the eight bytes the exception pushed).

## Provenance and rebasing

Both patches are against Open SIMH at commit `a1f57fa3`. Keep them small and
additive so they keep applying as upstream moves; net diff for 0002 is two
files, +79 lines, 0 removed, with both copyright headers untouched. Neither
patch changes instruction semantics — the simulator's own EHKAA self-test,
which `make ... vax` runs, passes unmodified with both applied.

Open SIMH is MIT, © 1998–2019 Robert M Supnik.
