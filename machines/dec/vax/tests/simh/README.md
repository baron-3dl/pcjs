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

The same problem recurs one milestone up: the device items after EHKAA (console
ROM, timer, UART, Qbus adapter) need an oracle for every KA655 device register
access and every delivered interrupt, and stock SIMH exposes none of it in
machine-readable form either. Patch 0006 closes that gap; see "What 0006 adds"
below.

And once more, one milestone further up: the Qbus DMA data path (`pcjsvax-e22`) cannot be reached
from the console at all — a DMA is not a CPU instruction, and the only in-tree callers of
`Map_ReadB`/`Map_WriteB` are device models not yet ported. Patch 0007 adds a console entry point
for them; see "What 0007 adds" below.

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

## Proving the patches still apply: verify-patches.sh

`verify-patches.sh` is the oracle-health check (`pcjsvax-fb1`). Unlike `build.sh`, which
deliberately reuses an existing `$DEST` for iteration speed, `verify-patches.sh` never reuses
anything: it derives the patch list *programmatically* from the two directories that hold
patches (one file in `pcjs-vax/patches/`, everything matching `*.patch` here, sorted), does a
real `git clone` of the vendored `open-simh` into a brand-new temp directory, applies every patch
in order, builds, and asserts SIMH's own EHKAA self-test reported PASS.

```
machines/dec/vax/tests/simh/verify-patches.sh              # fresh clone, apply, build, assert PASS
machines/dec/vax/tests/simh/verify-patches.sh --selfcheck   # mutation suite: drop/reorder/corrupt a patch, prove each is caught
```

`--selfcheck` doesn't inspect anything — it removes a patch from the chain, reorders two adjacent
patches, and corrupts a context line inside one, and asserts by execution that all three are
rejected by `git apply` and that the rejection names the responsible patch. A future rebase that
silently breaks a patch's applicability is exactly the failure this project cannot afford to
discover only when a differential goes red for an unrelated reason — `git status` on the pristine
vendor checkout should stay the only thing anyone needs to trust, everything else is re-derived.

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

## What 0006 adds

The device items after the EHKAA milestone (console ROM, timer, UART, Qbus adapter) need a
per-access oracle for every KA655 device register read/write and every delivered interrupt. Three
choke points cover all of it without touching a single device's own read/write body:

| Addition | Purpose |
|---|---|
| `IPRR`/`IPRW` at `ReadIPR`/`WriteIPR` (`vax_sysdev.c`) | Every IPR-space access — console UART (`ICCS`, `RXCS`, `RXDB`, `TXCS`, `TXDB`) and `TODR` — regardless of which internal case handles it. |
| `REGR`/`REGW` at `ReadReg`/`WriteReg` (`vax_sysdev.c`) | Every memory-mapped register-space access — `CQMAP`, `ROM`, `NVR`, `CMCTL`, `SSC` (including timers T0/T1), `KA` (`CACR`/`BDR`), `CQBIC`, `CQIPC`, `CDG` — dispatched through the single `regtable[]` lookup, so no per-device instrumentation is needed. |
| `INTD` at the `IE_INT` call site in `sim_instr()`'s dispatch loop (`vax_cpu.c`) | The vector and IPL of every interrupt actually delivered, logged at the point of delivery rather than reconstructed from `EXCA` records (which also fire for traps). Extends 0005's `EXCTRACE` mechanism (same debug category, same `sim_debug(LOG_CPU_X, &cpu_dev, ...)` call) rather than inventing a parallel one. |

New debug category `DEVTRACE` (`DBG_DEVT`, `sysd_dev`) gates `IPRR`/`IPRW`/`REGR`/`REGW`; `INTD`
rides the existing `EXCTRACE` (`LOG_CPU_X`, `cpu_dev`) category 0005 added. Record format, all four
fields hex except `INTD`'s IPL (decimal):

| Line | Fields |
|---|---|
| `IPRR` / `IPRW` | `rg size val PC` — `rg` is the IPR number (`MT_ICCS`=24, `MT_TODR`=27, `MT_RXCS`=32, `MT_RXDB`=33, `MT_TXCS`=34, `MT_TXDB`=35, …), `size` is always `L_LONG` (IPRs have no other width) |
| `REGR` / `REGW` | `pa size val PC` — `pa` is the physical address, `size` is the access length in bytes (B/W/L/Q) |
| `INTD` | `vec ipl PC` — the SCB vector and IPL of the interrupt about to be dispatched via `intexc()` |

Booting `ka655x.bin` to `>>>` under the patched simulator with `SET CPU DEBUG=EXCTRACE` and `SET
SYSD DEBUG=DEVTRACE` produces (one representative run): `IPRR`=58,750, `IPRW`=336, `REGR`=7,085,341,
`REGW`=1,275,226, `INTD`=16,385 — every required family nonzero (`ICCS` write=5, `TODR` read=35,387,
`RXCS` read=22,500/write=4, `RXDB` read=2, `TXCS` read=847/write=4, `TXDB` read=1/write=243, SSC
T0/T1 read=273,555/write=241, `CMCTL` read=1,794/write=5,500, `KA` `CACR` read=12/write=6, `KA`
`BDR` read=18, `CQBIC` read=41,005/write=32,805). `RXDB`/`TODR`/`ICCS` are one-directional in this
run because the ROM firmware genuinely only exercises them that way during a no-disk boot to the
console prompt (`RXDB`/`TODR` read-only, `ICCS` write-only) — not a coverage hole in the patch, a
property of what this particular boot path does; each family is still proven *reachable*.

Two details are load-bearing:

* **`ReadReg`/`WriteReg` is the single choke point for eight register-space families at once**
  (`CQMAP`, `ROM`, `NVR`, `CMCTL`, `SSC`, `KA`, `CQBIC`, `CQIPC`, `CDG`) because every one of them is
  reached through the same `regtable[]` dispatch loop. Hooking the eight individual `_rd`/`_wr`
  functions instead (`cmctl_rd`, `ka_rd`, `cqbic_rd`, …) would have meant eight edits instead of
  two, split across two files (`vax_sysdev.c` and `vax_io.c`), for the same coverage.
* **`tti_dev`/`tto_dev` have zero `DEV_DEBUG` instrumentation, and none was added.** The console
  UART's `TXCS`/`TXDB`/`RXCS`/`RXDB` registers are reached exclusively through `MTPR`/`MFPR` →
  `op_mtpr`/`op_mfpr`'s `default:` case → `WriteIPR`/`ReadIPR`, which `IPRR`/`IPRW` already covers
  completely. Adding a second, device-local trace would duplicate the same events under a different
  name.
* **0006's `WriteReg` hunk is not strictly additive-only — it drops two trailing spaces from an
  existing context line.** The pristine line `p->write (pa, val, lnt);  ` (note the trailing
  whitespace, inherited the same way 0003's did — see the CRLF hazard entry below) becomes
  `p->write (pa, val, lnt);` with no trailing whitespace once the new trace line is inserted above
  it. Keeping the trailing spaces would trip `git apply`'s whitespace check on a context line that
  is unrelated to this patch's own additions, which done condition 5 (zero whitespace warnings)
  does not allow; dropping them is the smaller deviation and is judged the correct tradeoff. A
  rebase that regenerates this hunk from a fresh diff should expect this one line to show as
  changed context, not pure addition, and should not "fix" it back to preserve the trailing
  whitespace.

### CAVEAT: masked interrupt requests are invisible to `INTD` (and to everything else)

`INTD` (above) fires exactly once per interrupt **delivered** — the instant `sim_instr()`'s
dispatch loop calls `intexc()` for it. It does **not** fire for an interrupt **requested**: `SET_INT`
and `CLR_INT` (`vaxmod_defs.h:453-454`) are plain macros that OR/AND-clear a bit directly into
`int_req[]`, with no function call and therefore no site this patch (or any patch) can hook. A
request raised while `PSL<IPL>` or `SISR` masks it produces **no trace record of any kind** — not
`INTD`, not anything else — until (if ever) it is later unmasked and actually taken. Verified by
execution: with `PSL<IPL>=31` and `SISR` bit 4 set, four `STEP`s produce zero `INTD` records;
dropping `IPL` to 0 delivers the same pending request immediately (`INTD 00000090 4 00001004`).

This was reviewed (`pcjsvax-e17`) and ruled **acceptable** on the condition that it be documented —
a device harness reading this trace must know that "no `INTD` for vector V" means "V has not been
**delivered** yet," not "V has not been **requested**." A device model that raises an interrupt and
then polls this trace for confirmation it was *seen* by the CPU needs a different signal (e.g. the
CPU's own `PSL<IPL>` dropping below the device's level) — this trace was never going to give it one,
by construction, and that construction is the same for every future device, not something 0006's or
62a's design failed to reach.

## What 0006 adds, extended by pcjsvax-62a: Qbus I/O-page and CQM-window traffic

0006 (above) covers two of the three ways `vax_mmu.h`'s `ReadB`/`ReadW`/`ReadL`/`WriteB`/`WriteW`/
`WriteL` can resolve a non-RAM physical access: `ADDR_IS_IO` false → the IPR/register-space branch
(`ReadIPR`/`WriteIPR`, `ReadReg`/`WriteReg`). It never touched the **third** branch — `ADDR_IS_IO`
true → `ReadIO`/`WriteIO`/`ReadIOU`/`WriteIOU` (`vax_io.c`) → `ReadQb`/`WriteQb` → either
`cqm_rd`/`cqm_wr` (Qbus memory window, `ADDR_IS_CQM`, base `0x30000000`) or `iodispR[]`/`iodispW[]`
(Qbus I/O page, `ADDR_IS_IOP`, base `0x20000000`, built by `build_dib_tab()` from every enabled
device's `DIB` — `rl`, `rq`, `rqb`, `rqc`, `rqd`, `ts`, `tq`, `xq`, `xqb`, `dz`, `dpv`; see
`vax_syslist.c:68-101`). `pcjsvax-e17`'s review measured this directly: `SHOW RL` reports the RL
controller at `20001900-20001909`, a hand-built `MOVL @#20001900,R0` executed and returned
`R0=00000080` — a real RL status value, the read genuinely worked — and the `DEVTRACE` log was
**completely empty** for it. `pcjsvax-62a` closes that gap.

| Addition | Purpose |
|---|---|
| `QIOR` at `ReadIO`/`ReadIOU` (`vax_io.c`), after the value is resolved, before return | Every Qbus I/O-page **and** Qbus-memory-window read, whichever branch `ReadQb` took internally. |
| `QIOW` at `WriteIO`/`WriteIOU` (`vax_io.c`), at entry, before dispatch | The write-side equivalent — fires even if the dispatched write itself would be a no-op (unmapped I/O-page slot → `cq_merr`/`mem_err`), same rationale as `WriteIPR`'s pre-switch placement. |

Same `DBG_DEVT` category and `sysd_dev` handle 0006 already uses — `SET SYSD DEBUG=DEVTRACE` remains
the one knob that arms every family, IPR space through Qbus I/O page alike. `vax_io.c` cannot
`#include` `DBG_DEVT`'s definition from `vax_sysdev.c` (it is a private `#define`, not a header
symbol), so it is declared a second time there, with a comment tying the two together; a rebase that
changes one must change the other. Record format matches `REGR`/`REGW` exactly:

| Line | Fields |
|---|---|
| `QIOR` / `QIOW` | `pa size val PC` — `pa` is the physical address (`0x20000000`+ for I/O page, `0x30000000`+ for the Qbus memory window), `size` is the access length in bytes (B/W/L/Q), matching `lnt` as the caller of `ReadIO`/`WriteIO` supplied it |

Booting `ka655x.bin` to `>>>` under the patched simulator (same `SET CPU DEBUG=EXCTRACE`, `SET SYSD
DEBUG=DEVTRACE` as above) produces, in the I/O-page range, `QIOR`=16,385 / `QIOW`=1, and in the
CQM range, `QIOR`=8,240 / `QIOW`=24,624 — every direction of both branches nonzero. **None of it is
`rl`/`rq`/`ts`/`tq`/`xq`/`dz` traffic.** A no-disk, no-media boot to the console prompt never issues
a single read or write to any of those controllers' own registers — measured directly (`SHOW RL`
gives `20001900-20001909`, `SHOW RQ` gives `20001468-2000146B`, `SHOW TS` gives `20001550-20001553`,
`SHOW TQ` gives `20001940-20001943`, `SHOW XQ` gives `20001920-2000192F`, `SHOW DZ` gives
`20000040-2000005F`; every one of those ranges shows zero `QIOR`/`QIOW` events in a real boot). What
the boot *does* exercise, every time, is the QBA's own doorbell register at `0x20001F40` (I/O page)
and the Qbus memory window itself (CQM) — both reached through the **identical** `ReadIO`/`WriteIO`
→ `ReadQb`/`WriteQb` code path as `rl`/`rq`/etc. would use if attached and probed, with no per-device
special-casing anywhere in this patch. Requiring `rl`/`rq`/`ts`/`tq`/`xq`/`dz` themselves to show
traffic in this oracle would be the same false-floor trap `ICCS`/`RXDB`/`TODR`'s one-directionality
already warns against (see above), one level up: a whole family the ROM structurally never touches
in this configuration, not one direction of a family it does. `devtrace-tally.py` therefore asserts
two families, `QBIOP` (I/O page) and `QBCQM` (Qbus memory window) — one per branch named in the gap,
not one per named device — both directions required for both, which is exactly what a real boot
produces; see that script's module docstring for the full reasoning.

Two details are load-bearing here too:

* **`ReadIO`/`WriteIO`/`ReadIOU`/`WriteIOU` are the single choke point for BOTH remaining branches**
  (I/O page and Qbus memory window) for the same reason `ReadReg`/`WriteReg` was the single choke
  point for eight register-space families: `ReadQb`/`WriteQb` themselves branch on `ADDR_IS_CQM`
  internally, so hooking one level up, before that branch, catches both without touching `cqm_rd`,
  `cqm_wr`, or any of the eleven devices' own `_rd`/`_wr` functions.
* **This extends patch 0006 rather than landing as a new 0007.** 0006's own stated purpose was "a
  per-access oracle for every KA655 device register read/write" — this item closes an acknowledged
  gap in that same claim (0006 covered two of the three `vax_mmu.h` branches, not all three), reuses
  0006's `DBG_DEVT` category and `sysd_dev` handle rather than inventing a parallel mechanism, and
  touches no file 0006 didn't already touch conceptually (it adds a third file, `vax_io.c`, to
  0006's existing `vax_cpu.c`/`vax_sysdev.c`, but the *feature* — one debug category gating every
  KA655 device-register access — is one thing, not two). A separate 0007 would fragment one feature
  across two patch files for no benefit; `verify-patches.sh` derives the patch list from disk, not
  from a hand-maintained count, so extending 0006 in place costs nothing there either.

### A second veracity pass found a narrower gap: the boot-derived floors only prove the ALIGNED half

The patch above hooks all four functions — `ReadIO`/`WriteIO` (aligned) and `ReadIOU`/`WriteIOU`
(unaligned) — from the start. `devtrace-tally.py`'s QBIOP/QBCQM floors, however, are derived from a
real `ka655x.bin` boot to `>>>`, and that boot's own Qbus traffic is entirely aligned — it never
straddles a word or longword boundary. A mutation that silences only `ReadIOU`'s guard (or only
`WriteIOU`'s), leaving `ReadIO`/`WriteIO` untouched, applies clean, builds clean, passes EHKAA, and
produces QBIOP/QBCQM counts **byte-for-byte identical** to the unmutated binary — the boot-derived
floors cannot see it, by construction, no matter how long the boot runs.

`devtrace-tally.py` closes this with `unaligned_probe()`: after the boot-derived tally, it derives
an I/O-page device's CSR address from the binary's own `SHOW RL` output (never hand-typed), deposits
two hand-assembled `MOVL` instructions targeting one byte off that address (`MOVL @#<csr+1>,R0` and
`MOVL R0,@#<csr+1>`), and steps them directly — no boot, no wall-clock dependency. A longword access
one byte off alignment is exactly the "tribyte" case `ReadIOU`/`WriteIOU`'s own header comments
describe; it produces a `QIOR`/`QIOW` record with `size=00000003`, a value `ReadIO`/`WriteIO` can
never emit (their `lnt` is restricted to `L_BYTE`/`L_WORD`/`L_LONG`/`L_QUAD` = 1/2/4/8 by every call
site). That makes a `size=00000003` record at the probed address unfakeable proof the unaligned path
specifically fired. `verify-patches.sh --selfcheck` gained two more mutations (6 and 7) that silence
`ReadIOU`'s and `WriteIOU`'s guards respectively and assert this probe — and only this probe — catches
each one, by name (`UNALIGNED READ` / `UNALIGNED WRITE`).

One development hazard worth recording here: driving the CPU directly via `DEPOSIT`/`STEP` (rather
than `BOOT CPU`) on a freshly-started simulator process collides with SIMH's own one-time clock
precalibration self-test (`sim_timer_precalibrate_execution_rate()`, wired in by `cpu_reset()`),
which deposits **its own** throwaway code at physical `0x100`-`0x10D` and runs it on the *first*
`GO`/`STEP`/`RUN` of any session — consuming the requested step count instead of the caller's own
setup if not absorbed first. `unaligned_probe()` issues one throwaway `step 1` before touching
anything else for exactly this reason; see its docstring.

## What 0007 adds

`pcjsvax-e22` needs an oracle for the **Qbus DMA data path** — the CQBIC scatter-gather map and the
four routines every Qbus mass-storage device (RQ/MSCP, TQ, RL) calls to move data through it:

```c
int32 Map_ReadB  (uint32 ba, int32 bc, uint8 *buf);        /* vaxmod_defs.h:459-462 */
int32 Map_ReadW  (uint32 ba, int32 bc, uint16 *buf);
int32 Map_WriteB (uint32 ba, int32 bc, const uint8 *buf);
int32 Map_WriteW (uint32 ba, int32 bc, const uint16 *buf);
```

**Nothing in a stock `microvax3900` can be made to call them from the console.** A DMA is by
definition not a CPU instruction, so no instruction stream reaches them, and the only in-tree
callers are device models this project has not ported yet. `EXAMINE QBA` / `DEPOSIT QBA`
(`qba_ex`/`qba_dep`) come closest but go through `qba_map_addr_c()`, the *console* variant of the
translation, which is word-only and **deliberately latches no error state at all** — so it cannot
grade the residual count, the `DSER`/`MEAR`/`SEAR` side effects, or the per-page remap loop, which
is the entire subject.

0007 adds one `SHOW` on the QBA device — not on the CPU, so the whole patch is confined to
`vax_io.c`, the file that already owns these routines:

```
SHOW QBA QDMA=op:ba:bc:sa       (every field hex, colon separated -- SHOW splits on commas)

  op   0 = Map_ReadB, 1 = Map_ReadW, 2 = Map_WriteB, 3 = Map_WriteW
       ("read"/"write" are from the DEVICE's point of view, as in the routine names:
        a read moves memory into the device's buffer)
  ba   Qbus bus address (the routines apply QBMAMASK themselves)
  bc   byte count, 0 <= bc <= QDMA_MAXBC (4096)
  sa   the device buffer's STAGING address in physical memory -- not a Qbus address,
       not touched by the map

  -> QDMA <op> <resid> <dser> <mear> <sear> <ipc> <memerr>
  -> QDMA <op> abort <abortval>
```

`resid` is the routine's own return value: **the count NOT transferred, 0 on full success.** A
device that treats a non-zero return as success is the defect this convention exists to prevent.

The buffer travels through **memory**, not the command line, for a measured reason: a
scatter-gather transfer has to span three or more 512-byte map pages to be worth grading at all
(a single-page transfer is satisfied by an identity map, which is exactly the wrong implementation),
and several KB of hex would not survive SCP's command buffer. For a read the buffer is pre-filled
with `QDMA_FILL` (`0xE5`) and copied out for the **full** `bc`, so the untransferred tail of a
partial transfer is directly observable rather than inferred from `resid`. The `Map_ReadW`/
`Map_WriteW` buffer is a `uint16 *` in the C and is staged here as its underlying **bytes**, so a
byte-order defect in a port shows up in the staged buffer instead of being hidden by a word-level
comparison.

The CQBIC error registers are read *after* the transfer and are **not** cleared by the command:
they are sticky (that is what `CQDSER_LST` means), so clearing them would destroy the accumulation
a test wants to grade. The caller decides what state each case starts from, with
`DEPOSIT QBA DSER/MEAR/SEAR`.

The command owns a `setjmp` of its own, like `SHOW CPU MMUOP=` and `SHOW CPU FPOP=`, because the
`Map_*` routines can `ABORT()` and there is no `sim_instr` frame to land in when they are called
from the console. In principle it is unreachable — `qba_map_addr()` returns `FALSE` for every
address `ReadB`/`ReadL` could fault on — so the abort line is *reported*, not assumed away.

`tests/qdmadiff.js` is the consumer. A binary built without 0007 makes it **fail with a message
naming the patch**; it is never silently skipped.

## Upstream-drift hazards, per patch

What each patch hangs off of — check these first on a rebase; if any of them moved or changed
signature, that patch's hunks likely still apply as *text* (small, additive hunks tend to) but may
no longer mean what the prose above says.

| Patch | Files | Hooks (exact functions/macros/structs) |
|---|---|---|
| 0001 | `vax_cpu.c`, `vax_defs.h` | `InstHistory` struct (adds `reg[16]`); the history-record capture inside `sim_instr()`'s main loop (`for (i = 0; i < j; i++) h->opnd[i] = ...` site); `cpu_show_hist_records()`'s per-record print loop. |
| 0002 | `vax_cpu.c`, `vax_defs.h` | `Read()` — a `static SIM_INLINE` in `vax_mmu.h` — wrapped and `#define`d to `Read_dlog` at file scope in `vax_cpu.c`; the specifier-loop entry (`numspec = numspec & DR_NSPMASK`) where `dec_rlogn` is armed; the same history-record capture site 0001 touches (extends it with `preg`/`rlog`/`recq`/`nopnd`/`brdisp`/`ilen`); `InstHistory` struct again. |
| 0003 | `vax_cpu.c`, `vax_mmu.c`, `vax_mmu.h` | `cpu_mod[]` MTAB table (adds `MMUOP`/`MMUTRACE`/`NOMMUTRACE` entries — order-sensitive only in that 0004 inserts its own entry immediately after MMUOP's); `cpu_get_vsw()` (adds the `-W` switch); new `cpu_show_mmuop()`/`cpu_set_mmutrace()` functions (call `Test()`/`Read()`/`Write()`/`op_mtpr()` and `save_env`/`setjmp` directly — these are the real fault/abort paths, a signature change there is high-risk); `vax_mmu.h`'s inline `Read`/`Write`/`Test` (each gets one `PCJS_MMU_T(...)` call inserted — these three functions are the highest-traffic hook point in the whole patch set, called from `vax_cpu.c`, `vax_cpu1.c`, `vax_cis.c`, `vax_octa.c`); `fill()`, `set_map_reg()`, `zap_tb()`, `zap_tb_ent()` in `vax_mmu.c` (one trace call each). |
| 0004 | `vax_cpu.c` | `cpu_mod[]` MTAB table (inserts the `FPOP` entry right after 0003's `MMUOP` entry — **this is why 0004 fails to apply if 0003 is missing or reordered after it**); new `cpu_show_fpop()`, whose body is *copied verbatim* out of `sim_instr()`'s F/D/G-opcode dispatch `switch` between two marker comments — on a rebase, diff `sim_instr()`'s float-opcode cases against this function body directly, since that's the actual source of truth being duplicated; `op_*` routines in `vax_fpa.c` (called, not modified); `CC_IIZZ_FP`/`CC_IIZP_FP` macros and `WRITE_B`/`WRITE_W`/`WRITE_L`/`WRITE_Q` macros in `vax_defs.h`/top of `vax_cpu.c` (referenced, not modified — a macro signature change is the real hazard here). |
| 0005 | `vax_cpu.c`, `vax_cpu1.c`, `vax_defs.h` | `cpu_deb[]` DEBTAB table (adds `EXCTRACE`); new `LOG_CPU_X` bit in `vax_defs.h` (must stay clear of the existing `LOG_CPU_FAULT_*` bits — currently `0x200`, one past `LOG_CPU_FAULT_EMUL`'s `0x100`); new `exc_trace_state()`; and five existing functions get entry/exit trace calls spliced in: `intexc()` (also hoists its SCB-read address into a local, `pcjs_scbpa`, logged and then used in place of the inline expression — the read itself is unchanged), `op_rei()`, `op_chm()`, `op_mtpr()`, `op_mfpr()`. A rebase that changes any of these five functions' control flow (early returns, additional fault paths) needs the corresponding `PCJS_TRACING` block re-sited, not just re-hunked. |
| 0006 | `vax_cpu.c`, `vax_sysdev.c`, `vax_io.c` | `sysd_debug[]` DEBTAB table (adds `DEVTRACE`, bit `0x0040`, one past `DBG_CNF`'s `0x0020` — must stay clear of `DBG_REGR`/`DBG_REGW`/`DBG_INT`/`DBG_SCHD`/`DBG_TODR`/`DBG_CNF`); `ReadIPR`'s single common `return val` (traces `IPRR`, changes nothing about which case sets `val`) and `WriteIPR`'s entry (traces `IPRW` before the switch, so it fires even for the `RSVD_OPND_FAULT` cases); `ReadReg`/`WriteReg`'s `regtable[]` dispatch loop in `vax_sysdev.c` (each gets one `val`/trace line inserted at its existing early `return`/`p->write(...)` call — this is the highest-leverage hook in the patch, since it's the *only* place all eight register-space families converge); the `IE_INT` call site inside `sim_instr()`'s dispatch loop in `vax_cpu.c` (reuses 0005's `LOG_CPU_X`/`cpu_dev`, adds no new bit — its inline comment also carries the masked-interrupt-invisibility caveat, see above); **(pcjsvax-62a)** `ReadIO`/`ReadIOU`/`WriteIO`/`WriteIOU` in `vax_io.c` (each gets one `val`/trace line — `ReadIO`/`ReadIOU` after the value is resolved and before `return`, `WriteIO`/`WriteIOU` at entry before dispatch — reusing `DBG_DEVT`/`sysd_dev` via a second `extern DEVICE sysd_dev;` + `#define DBG_DEVT 0x0040` local to `vax_io.c`, since the original is a private `#define` in `vax_sysdev.c`, not a header symbol). A rebase that changes `ReadReg`/`WriteReg`'s single-loop-with-early-return shape, or splits `regtable[]` into per-family dispatch, needs the trace call re-sited to wherever the new common exit is; the same applies to `ReadIO`/`WriteIO`/`ReadIOU`/`WriteIOU` if a rebase changes their signatures (e.g. folds the aligned/unaligned pairs together) or if `vax_sysdev.c`'s `DBG_DEVT` value ever moves (the `vax_io.c` copy must move with it — nothing enforces the two definitions staying equal except this note). |

| 0007 | `vax_io.c` | `qba_mod[]` MTAB table (appends the `QDMA` entry after 0006's untouched entries — order-insensitive, it is last); new `qba_show_qdma()`, which calls `Map_ReadB`/`Map_ReadW`/`Map_WriteB`/`Map_WriteW` and `ReadB`/`WriteB` directly and takes `save_env`/`setjmp` — a signature change to any `Map_*` routine (as in `is1000_sysdev.c`, whose versions take an extra `t_bool map` argument) breaks it loudly at compile time, which is the desired failure; `cq_dser`/`cq_mear`/`cq_sear`/`cq_ipc` module globals and `mem_err` (`vax_defs.h:891`) are read, not modified; `ADDR_IS_MEM` bounds the staging address. |

Provenance: patches 0001-0005 are against Open SIMH at commit `a1f57fa3`; 0006 is generated against
that same base with 0001-0005 already applied (its context lines reflect the post-0005 source, as
`git apply`'s sequential model requires); 0006 was extended in place by `pcjsvax-62a` against the
same base plus 0001-0005 (not a fresh diff against later SIMH state), for the same sequential-model
reason. Keep them small and additive so they keep applying as
upstream moves; net diff is two files +79 lines for 0002, three files +166/-4 for 0003, one file
+356/-0 for 0004, three files +71/-1 for 0005, (as extended by pcjsvax-62a) three files +67/-4
for 0006, and one file +157/-0 for 0007 (generated against the same base with 0001-0006 applied,
for the same sequential-model reason), with every copyright
header untouched. No patch changes instruction semantics — the simulator's own EHKAA self-test,
which `make ... vax` runs, passes unmodified with all seven applied.

### Two hazards fixed by `pcjsvax-fb1` — read before assuming either is still true

* **0003, not 0004, was the trailing-whitespace source.** `git apply`'s default whitespace check
  flagged one line in 0003 (`vax_mmu.h`, the `Test()` case in the read-vs-fill inline) on every
  application: the added line converts a single-statement `if` into a brace block, and it
  inherited a trailing space that was *already present* on the original single-statement line in
  upstream `vax_mmu.h` — i.e. the trailing space predates this patch set and isn't something 0003
  introduced, it just carried it onto a `+` line where `git apply` now sees it. Fixed by dropping
  the one inherited trailing space on 0003's added line; the hunk's surrounding context is
  otherwise byte-identical, and a fresh clone + apply of all five (verified by
  `verify-patches.sh`) is silent. If a rebase reintroduces a whitespace warning, check the new
  hunk against the *current* upstream line first — it may be carrying forward a different
  upstream artifact, not repeating this one.
* **`build.sh`'s `cp -a "$SRC" "$DEST/open-simh"` corrupts the pristine vendor checkout if `$SRC`
  (i.e. `$PCJS_VAX_REPO/open-simh`) is itself a symlink** — which it legitimately is in every
  worktree set up per this project's own convention (a worktree's `open-simh` is a symlink to the
  one pristine checkout, so it isn't cloned N times). `cp -a` implies `-d`/`--no-dereference`, so
  copying a symlink SOURCE copies the symlink itself, not the directory it points to — `$DEST/open-simh`
  silently becomes a second symlink back to the *same* pristine tree, and every `git apply`/`make`
  that follows patches and builds the pristine checkout in place, with nothing about the run's
  output indicating it. This bit an agent working `pcjsvax-fb1` directly (mid-task, mid-July 2026)
  and was caught only because the pristine tree's `git status` was checked externally. Fixed: `build.sh`
  now copies `"$SRC/."` (the trailing `/.` forces resolution through a top-level symlink while still
  preserving any symlinks *inside* the tree) and asserts `$DEST/open-simh` is a real directory, not
  a symlink, immediately after — in both the fresh-copy branch and the reuse branch, failing loudly
  rather than silently building through the pristine tree. **Never `git apply` or build with a
  working directory that resolves — via any symlink hop — into `pcjs-vax/open-simh`.** If you
  aren't sure a path is safe, `realpath` it and compare against the pristine checkout's realpath
  before doing anything destructive.

Open SIMH is MIT, © 1998–2019 Robert M Supnik.
