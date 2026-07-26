# Instrumented Open SIMH for the VAX differentials

`decodediff.js` and `mmudiff.js` grade our decoder and our MMU against a real
Open SIMH `microvax3900`. Doing that requires the simulator to expose things it
does not expose on its own: enough state for a decode to be *replayed* rather
than merely inspected, and a way to observe a translation at all. This directory
holds the patches that add them, and the script that builds them.

```
machines/dec/vax/tests/simh/build.sh          # -> $TMPDIR/pcjs-vax-simh/open-simh/BIN/microvax3900
export SIMH_DECODE_BIN=$TMPDIR/pcjs-vax-simh/open-simh/BIN/microvax3900
export SIMH_MMU_BIN=$SIMH_DECODE_BIN
node machines/dec/vax/tests/decodediff.js
node machines/dec/vax/tests/mmudiff.js
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

## What 0003 adds

Memory management is the one part of the machine that is essentially
*unobservable* from outside it. The stock console comes close — `SHOW CPU
VIRTUAL=n` is a thin wrapper over `Test()` — but only for a **read** in one of
the four access modes, which leaves the entire write half of the protection
matrix, the M bit, the unaligned/cross-page data path, and every TB-invalidation
rule unobserved. Three additions close that, and nothing else:

| Addition | Purpose |
|---|---|
| `SHOW CPU MMUOP=op:va:lnt:mode[:val]` | Perform exactly one MMU operation and print one machine-readable line. `op` 0 = `Test` read, 1 = `Test` write, 2 = `Read`, 3 = `Write`, 4 = `MTPR` (`va` is the value, the trailing field the IPR number). Colon separated, because `SHOW` splits its argument on commas. |
| `SET CPU MMUTRACE=<file>` / `NOMMUTRACE` | Log every `Read`/`Write`/`Test`, every `fill()` with the PTE addresses and values it read, every M-bit write-back, every `zap_tb`/`zap_tb_ent`, and every `set_map_reg`. |
| `-W` switch on `cpu_get_vsw()` | `SHOW -W CPU VIRTUAL=n` and `EXAMINE -V -W` probe **write** access instead of read. |

Two details are load-bearing:

* **`op 4` calls SIMH's real `op_mtpr()`,** not a private copy of the register
  assignments. A console `DEPOSIT P0BR` does *not* run `zap_tb()` or
  `set_map_reg()`, so a test driven with deposits would have the two machines'
  translation buffers out of step from its first operation and would never grade
  the invalidation rules at all.
* **`SHOW CPU MMUOP=` owns a `setjmp` of its own.** `Read` and `Write` abort
  through `ABORT()`, which is a `longjmp` to `save_env`; called from the console
  there is no `sim_instr` frame to land in. The routine therefore re-arms
  `save_env` and reports `abort <code> <p1> <p2>` instead of jumping into a stale
  stack.

The trace hooks live in `vax_mmu.h`'s **inline** `Read`/`Write`/`Test`, so they
cover every caller in `vax_cpu.c`, `vax_cpu1.c`, `vax_cis.c`, `vax_octa.c` and
the rest without touching a single call site. Each is one statement that expands
to a `NULL` test when the log is closed.

Trace grammar (hex unless noted):

| Line | Contents |
|---|---|
| `G p0br p0lr p1br p1lr sbr slr mapen` | `set_map_reg()` ran |
| `Z stb` | `zap_tb(stb)` — process TB, or both when `stb` |
| `Y va` | `zap_tb_ent(va)` — MTPR TBIS |
| `O <R\|W\|T> va lnt acc mapen` | a `Read`/`Write`/`Test` began |
| `F va lnt acc` | `fill()` was entered (twice for a cross-page access) |
| `S ptead pte` | a **system** PTE was read, to translate a **process** PTE's address |
| `P ptead pte` | the page's own PTE was read |
| `M ptead pte` | the M bit was written back |
| `E code` | `fill()` reported or raised a fault |
| `R tlbpte` | `fill()` succeeded |
| `A pa` | the resolved physical address |
| `B pa1` | the **second** physical address (unaligned accesses only) |
| `X status` | `Test()` refused the access |

A full EHKAA run produces 776,000 lines / 14MB covering 343,557 memory-management
operations, so `mmudiff.js` streams it rather than slurping it.

### An upstream quirk this made visible

`get_istr()` (`vax_cpu.c:3205`) probes the instruction stream with
`Test ((PC + ibcnt) & ~03, RD, &t)`. `RD` is not an access code — it is the
decode-ROM `.rd` specifier constant `(DR_SPFLAG|RQ)` = `0x0B`, where `RA` was
presumably meant. The trace shows `acc=0000000B` on every instruction-stream
probe, i.e. the fetch is checked against "readable in kernel, executive **or**
user" rather than against the current mode. We reproduce it exactly, because
`acc` is opaque to the MMU and we port *from* SIMH; it is recorded here so
nobody spends a day rediscovering it.

## Provenance and rebasing

All three patches are against Open SIMH at commit `a1f57fa3`. Keep them small and
additive so they keep applying as upstream moves; net diff is two files +79 lines
for 0002 and three files +166/-4 for 0003, with every copyright header untouched.
No patch changes instruction semantics — the simulator's own EHKAA self-test,
which `make ... vax` runs, passes unmodified with all three applied.

Open SIMH is MIT, © 1998–2019 Robert M Supnik.
