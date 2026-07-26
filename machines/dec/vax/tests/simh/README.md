# Instrumented Open SIMH for the VAX differentials

`decodediff.js`, `mmudiff.js`, `fpadiff.js` and `excdiff.js` grade our decoder,
our MMU, our floating point and our exception dispatch against a real Open SIMH
`microvax3900`. Doing that requires the simulator to expose things it does not
expose on its own: enough state for a decode to be *replayed* rather than merely
inspected, a way to observe a translation at all, a way to hand one floating
instruction the operands you choose rather than the operands a program happened
to produce, and the privileged state an exception dispatch depends on — which
lives entirely in simulator variables and appears in no existing log. This
directory holds the patches that add them, and the script that builds them.

```
machines/dec/vax/tests/simh/build.sh          # -> $TMPDIR/pcjs-vax-simh/open-simh/BIN/microvax3900
export SIMH_DECODE_BIN=$TMPDIR/pcjs-vax-simh/open-simh/BIN/microvax3900
export SIMH_MMU_BIN=$SIMH_DECODE_BIN
export SIMH_FP_BIN=$SIMH_DECODE_BIN
node machines/dec/vax/tests/decodediff.js
node machines/dec/vax/tests/mmudiff.js
node machines/dec/vax/tests/fpadiff.js
export SIMH_EXC_BIN=$SIMH_DECODE_BIN
node machines/dec/vax/tests/excdiff.js
```

**`build.sh` REUSES an existing destination directory.** If you add a patch, or
pull one, delete the destination first — otherwise the new patch is silently
absent from the binary and the test that needs it fails with "this simulator
does not implement ...", which reads like a missing feature rather than a stale
build.

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

## What 0004 adds

Floating point is unobservable from outside the machine for a different reason
than memory management: it is perfectly observable, but only for the operands a
running program produces — and a running program never produces a reserved
operand, an exact rounding tie, or an exponent one step past overflow. Those are
precisely the cases a port gets wrong. One addition closes it:

| Addition | Purpose |
|---|---|
| `SHOW CPU FPOP=opc:psl:cc:spec:rn:va:d0:d1:n:o0:o1:...` | Execute exactly one F/D/G floating instruction on an operand queue you supply, and print one machine-readable line. Colon separated, because `SHOW` splits its argument on commas. |

Output is `FPOP <opc> ok <cc> <trpirq> <R[rn]> <R[rn+1]>`, or
`FPOP <opc> abort <abortval> <p1> <p2>`.

Three details are load-bearing:

* **The case bodies are copied verbatim out of `sim_instr()`'s dispatch switch**,
  between two marker comments, rather than reimplemented. So the arithmetic is
  the real `op_*` routines in `vax_fpa.c`, the condition codes are the real
  `CC_IIZZ_FP` / `CC_IIZP_FP` macros in `vax_defs.h`, and the store is the real
  `WRITE_B`/`WRITE_W`/`WRITE_L`/`WRITE_Q` macros at the top of `vax_cpu.c` —
  including `WRITE_Q`'s `Test (va + 7)` probe, which is what makes a quadword
  store that straddles into an inaccessible page leave nothing behind. The only
  thing transcribed is which case body belongs to which opcode.
* **`trpirq` is reported, not just aborts.** Floating overflow, underflow and
  divide by zero are *faults*: they abort. The integer overflow from `CVTF/D/G`
  to `B`/`W`/`L` is a *trap*: the instruction completes, stores its result, and
  only requests the trap — and only when `PSL<IV>` is set. A test that looked at
  aborts alone could not tell the two apart.
* **Only `PSW_FU` and `PSW_IV` are taken from the caller's `psl`.** The rest of
  the PSL, including the current access mode that decides `acc`, is left alone,
  so a caller cannot construct an unreasonable PSL. `PSL` and `trpirq` are saved
  and restored around the call.

Like `SHOW CPU MMUOP=`, it owns a `setjmp` of its own: the floating routines
abort through `ABORT()`, and called from the console there is no `sim_instr`
frame to land in.

A *memory* destination is deliberately **not** initialized by the command — the
caller pre-loads it with `DEPOSIT`, which is physical — so that a destination
made deliberately inaccessible can still be given a known prior value, which is
how `fpadiff.js`'s QUADSTORE phase proves that nothing was partially written.

## What 0005 adds

An exception dispatch is a pure function of the SCB base, the four per-mode stack
pointers, the interrupt stack pointer and the PSL — and **not one of those is
observable while the machine runs**. `SET CPU DEBUG=INTEXC` reports the vector,
the old PSL and the new one, which is enough to *count* dispatches and not nearly
enough to *reproduce* one: the new stack pointer comes from `STK[]` or `IS`, the
vector address from `SCBB`, and neither appears in any log. The same is true of
`REI` (which restores a stack pointer from `STK[]` and can raise an AST software
interrupt from `ASTLVL`), of `CHMx`, and of `MTPR`/`MFPR` (whose entire job is
those registers).

0005 adds one debug category, `EXCTRACE`, that emits a machine-readable
entry-state and result-state record around each of them:

| Record | Emitted | Fields (after the 17-field state vector, which every record carries) |
|---|---|---|
| `EXCA` / `EXCB` | `intexc()`, before / after computing the new PSL and stack | `vec ei ipl oldpsl PC oldsp trpirq` / `scbpa newpc PSL SP trpirq` |
| `REIA` / `REIB` | `op_rei()`, after popping the frame / after committing it | `PSL SP newpc newpsl trpirq` / `PSL SP newpc trpirq` |
| `CHMA` / `CHMB` | `op_chm()`, entry / after building the new PSL | `opc opnd0 PSL&#124;cc PC SP` / `newpc PSL SP` |
| `MTPA` / `MTPB` | `op_mtpr()`, entry / exit | `prn val PSL SP trpirq` / `cc PSL SP trpirq` |
| `MFPA` / `MFPB` | `op_mfpr()`, entry / exit | `prn PSL SP` / `val` |

The 17-field state vector is
`KSP ESP SSP USP IS SCBB PCBB ASTLVL SISR MAPEN P0BR P0LR P1BR P1LR SBR SLR PME`.

Three details are load-bearing:

* **The `A` record is emitted before the operation can fault, the `B` record only
  on success.** An `A` with no `B` is therefore not a gap in the log, it is the
  statement "SIMH rejected this" — which is how `excdiff.js` grades REI's nine
  PSL validity rules and MTPR/MFPR's privilege and range checks against real
  workload data, rather than only against generated ones.
* **`SP` is in the `MTPA`/`MTPB`/`MFPA` records** even though it is not a
  privileged register, because `MTPR`/`MFPR` of `KSP` (with `PSL<IS>` clear) and
  of `IS` (with it set) read and write the LIVE stack pointer, `R[14]`, not the
  saved one. Without it, 188 of EHKAA's 391 `MFPR`s would grade nothing.
* **`intexc()`'s SCB read address is hoisted into a local and logged.** It is
  computed identically to before (`(SCBB + vec) & (PAMASK & ~3)`, still evaluated
  from the same `SCBB`), so the read is unchanged; logging it is what lets the
  differential grade the address rather than only the value it returned.

### The debug log changes what EHKAA does — and that is not this patch's doing

With `EXCTRACE` on, EHKAA takes **2,356** dispatches instead of the **1,675** that
`docs/reference/ehkaa-profile.md` §5 records. The vectors are the same 25 and the
IPRs the same 24; only the counts move. This is **not** an artifact of the patch:
the **unpatched** simulator, given any comparably chatty debug category
(`set cpu debug=intexc;rei;rsvdfault;abort;context`), also produces 2,356. EHKAA
has timer-driven tests whose iteration count depends on how much wall-clock time
the host spends per simulated tick, and a large debug log moves that. Each
configuration is internally deterministic (three consecutive runs: 2,356 / 2,356
/ 2,356). `excdiff.js` therefore asserts the vector and IPR **sets** as
equalities and the event **count** as a floor.

## Provenance and rebasing

All five patches are against Open SIMH at commit `a1f57fa3`. Keep them small and
additive so they keep applying as upstream moves; net diff is two files +79 lines
for 0002, three files +166/-4 for 0003, one file +356/-0 for 0004 and three files
+71/-1 for 0005, with every copyright header untouched. No patch changes
instruction semantics — the simulator's own EHKAA self-test, which `make ... vax`
runs, passes unmodified with all five applied.

Open SIMH is MIT, © 1998–2019 Robert M Supnik.
