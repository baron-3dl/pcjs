---
layout: page
title: DEC VAX
permalink: /machines/dec/vax/
---

DEC VAX
-------

A MicroVAX 3900 (KA655) machine for PCjs, ported from [Open SIMH](https://github.com/open-simh/simh)
(MIT, © 1998-2019 Robert M Supnik).  Work in progress: at present this directory contains the
physical memory and bus layer, instruction decode and operand resolution, memory management, and
the integer/logical/variable-length-bit-field and control-flow slices of instruction
execution, and F/D/G floating point — still missing are SCB exception dispatch and
privileged registers, packed-decimal/CIS, the string/queue instructions, devices, and
the CPU loop that ties the instruction bodies together.

### Read this before writing any VAX code

The unsigned address convention is stated in full at the top of
[modules/v2/defines.js](modules/v2/defines.js), and every module here inherits it.  The short form:

* VAX S0 system space starts at `0x80000000`, which is **negative** as a JavaScript int32.
* Physical addresses are normalized on entry to every public Bus interface with
  `addr = (addr >>> 0) & VAX.PAMASK` and are provably in `0..0x3FFFFFFF` thereafter.
* `>>` applied to an address is a defect.  So is a relational compare, an arithmetic operation, or
  an array index on an address that has not been normalized.
* **Virtual** addresses are 32 bits and cannot be masked down — the region select lives in bits
  `<31:30>`.  That is where this bug will actually cost you a day, not here.
* Longword *data* is deliberately the exception: it is a signed int32, matching `Int32Array` and
  JS bitwise semantics.  Apply `>>> 0` at the point of use.

The physical bus is capped at **30 bits** (`VAX.PAWIDTH`), matching the CVAX CDAL bus and SIMH's
`PAMASK`.  Do not configure a 32-bit bus: `bus.js` eagerly allocates one block slot per
`nBlockSize` of address space at construction.

Memory management
-----------------

`modules/v2/mmu.js` is a direct port of Open SIMH's `vax_mmu.h` (the inline
`Read`/`Write`/`Test` fast path) and `vax_mmu.c` (`fill()`, `zap_tb()`, `zap_tb_ent()`,
`set_map_reg()`), including a split direct-mapped translation buffer — `stlb[]` for system space
and `ptlb[]` for process space, 4096 entries each — that mirrors SIMH's exactly.

**There is no block-per-page paging, and there must not be.**  PCx86 maps the 386 page table onto
the Bus block array (`cpux86.js` `enablePageBlocks`), which works only because an x86 page and a
PCjs block are both 4KB.  A VAX page is **512 bytes**; a block-per-page array over the 4GB virtual
space would be 8M entries.  This module translates explicitly on every access, as PDPjs does.

**`>>` on a virtual address is where this module bites.**  The physical bus is 30 bits, so
`PAMASK` rescues nearly everything in `bus.js`.  A virtual address is a full 32 bits with the
region select in `<31:30>`, and the page-table index in `fill()` —

```js
let ptidx = (va >>> 7) & ~0x03;         // CORRECT
let ptidx = (va >>  7) & ~0x03;         // silently translates through the WRONG page table
```

— is the one line where a signed shift produces no exception, no obviously wrong value, and no
fault.  `& ~0x03` does not rescue it: it clears two bits at the bottom while the shift corrupts
the top.  Several nearby expressions that *look* like the same hazard are provably safe and are
written the way SIMH writes them; the file header says which and why.

### The probe seam

`test(va, acc, stat)` is SIMH's `Test()`: it returns the physical address for `va`, filling the TB
exactly as a real access would, but reports failure through `stat` **instead of faulting**.

```js
let stat = {code: 0};
let pa = mmu.test(va, MMUVAX.accWrite(mode), stat);
if (pa < 0) { /* stat.code is one of MMUVAX.PR.* */ }
```

Three consumers need precisely this and should use it rather than inventing their own:

* a **quadword or octaword write** that must not perform a partial store when its second page is
  inaccessible — SIMH's `WRITE_Q` (`vax_cpu.c:222`) probes `va + 7` first and `va` second, so that
  when both pages are bad the fault reported is the one for the *first* page;
* **PROBER / PROBEW**, which the architecture defines in terms of it;
* the debugger's "show virtual", which must not perturb the machine.

Passing `stat` as null makes it fault instead, which is what an instruction body wants.

### What the MMU deliberately does not do

* It does not dispatch exceptions.  A translation fault throws a `VAXFault` carrying the ACV
  (`-0x20`) or TNV (`-0x24`) SCB offset plus SIMH's `p1`/`p2` fault parameters.
* It does not decode MTPR/MFPR.  It publishes the register mutators the IPR item needs —
  `setP0BR()`, `setSLR()`, `setMAPEN()`, `zapTB()`, `zapTBEnt()`, `chkTBEnt()` — each flushing
  exactly what SIMH's MTPR handler flushes.
* It does not turn a non-existent-memory fault into a machine check.
* It does not handle quadword or octaword references, exactly as SIMH's `Read`/`Write` do not.

Floating point
--------------

`modules/v2/fpa.js` is a direct port of Open SIMH's `vax_fpa.c` — specifically of its **32-bit**
code path, the one Supnik wrote for hosts without a 64-bit integer type.  That is not a fallback
here, it is the right path: JavaScript's bitwise operators are exactly 32 bits, so SIMH's `UDP`
{hi, lo} pair and its `dp_*` routines translate one for one, whereas the `USE_INT64` path would
need BigInt in the hot loop of every floating instruction.  The differential grades us against a
simulator built **with** `USE_INT64`, so a zero-divergence run is also an empirical check that
SIMH's two paths agree — over denormal-adjacent, rounding-boundary, overflow and underflow inputs
that no program produces by accident.

**DEC floating point is not IEEE.**  Different bias, different bit layout (the fraction is stored
word-swapped, most significant bits in the LOW word of the LOW longword), no infinities, no NaNs,
and — the one that catches every port — **no negative zero**: an exponent of 0 with the sign set
is the RESERVED OPERAND, and it faults.  The bit pattern `0x00008000`, which is what an IEEE-shaped
port produces for `-0.0`, is a reserved operand fault on a VAX.  There are no denormals either;
the analogous edge is the exponent-0 cliff, where a result whose exponent falls to 0 or below
becomes a true zero — or faults, if `PSL<FU>` is set.

Scope is **F, D and G**, measured rather than assumed (`docs/reference/ehkaa-profile.md` §7): every
one of the fifteen Extended-Accuracy-Group opcodes the EHKAA diagnostic never executes is
H_floating, and the fourteen it does execute are G.  `ACBD/F/G`, `EMODD/F/G` and `POLYD/F/G` are
likewise never executed and are not implemented; the masking arguments that `vaxFadd()` and
`vaxFmul()` carry exist only to serve them, and are ported unused so those two routines stay
identical to SIMH's.

The module also owns the **destination stores** — `writeB/W/L/Q`, SIMH's `WRITE_B`/`WRITE_W`/
`WRITE_L`/`WRITE_Q` macros (`vax_cpu.c:212-233`), which `pcjsvax-8c0` deliberately left out of the
decoder because storing is execution.  `writeQ()`'s comment is worth reading before touching it:
the C macro's indentation lies, the `if` guards only the low-longword write, and that is what
makes a quadword straddling into an inaccessible page store *nothing* while still reporting the
*first* page's fault when both pages are bad.

Decoder contract
----------------

**Read this before writing an instruction body.**  `modules/v2/decode.js` resolves operand
specifiers; three separate bodies of instruction work consume its output in parallel, so the
contract is stated once, here and in that file's header, rather than rediscovered three times.

`VAXDecoder.decode(fFPD)` fetches the opcode (including the `FD` prefix), applies the
instruction-group check that decides whether this CPU implements the opcode at all, and resolves
its operand specifiers.  Afterwards these are valid until the next `decode()`:

| Field | Meaning |
|---|---|
| `opnd[0..nOpnd-1]` | resolved operand queue, in specifier order — see below |
| `spec`, `rn`, `va` | state of the **last memory specifier**, which is what `writeB/W/L/Q` store through |
| `brdisp` | branch displacement, **zero-extended**; the branch body sign-extends |
| `vfldrp1` | `R[(rn+1)&15]`, captured for a `.vb` register specifier |
| `recq[0..recqptr-1]` | recovery queue — see *Faults* below |

What a specifier contributes to the queue depends on its decode ROM **access type**, not on its
addressing mode:

| Access | Queue contribution |
|---|---|
| `r.bwl`, `m.bwl` | `opnd[j]` = the operand's **value** |
| `r.q`, `m.q` | `opnd[j:j+1]` = value, low longword first |
| `r.o`, `m.o` | `opnd[j:j+3]` = value, low longword first |
| `a.bwlqo` | `opnd[j]` = the operand's **address** |
| `w.bwlqo` | `opnd[j]` = `OP_MEM` (-1) for a memory destination, else the **register number**; `opnd[j+1]` = the memory address (or `R[rn]`, which is *not* an address) |

So a write specifier occupies **two** queue slots and an address specifier one.  Byte and word
values are zero-extended; longwords are signed int32 and may be negative — sign-extend in the
body, as SIMH does.  `va` is a signed int32 virtual address: apply `>>> 0` before comparing,
indexing or formatting it.

**Faults and register-side-effect recovery.**  Autoincrement and autodecrement modify registers
*during* resolution.  If a later specifier faults, the architecture requires the instruction to be
restartable, so those modifications must be undone before the exception is taken.  The decoder
throws a `VAXFault` (in place of SIMH's `longjmp`) and records what it changed in `recq[]`; the
CPU's catch handler owes it:

```js
if (!(psl & PSL_FPD)) decoder.unwind();      // undo autoinc/autodec side effects
decoder.resetRecovery();
// ...then restore PC to fault_PC and dispatch the exception
```

The first-part-done guard is not optional — an instruction that has already made externally
visible progress is resumed, not restarted.  The PC is deliberately *not* in the recovery queue;
the CPU restores it wholesale.

### Testing

Both layers are graded by randomized differential tests against a **real, executed**
`microvax3900` binary — no fixtures, no golden files.

The memory/bus layer:

    node machines/dec/vax/tests/busdiff.js --selfcheck

150,000 random operations across every access size (byte/word/longword/quadword) and alignment,
including accesses that straddle 8KB block boundaries and the top of physical memory, and a heavy
mix of S0 addresses above `0x80000000` delivered both as positive doubles and as negative int32s.

Decode and operand resolution:

    machines/dec/vax/tests/simh/build.sh                 # patched simulator, once
    node machines/dec/vax/tests/decodediff.js --selfcheck
    node machines/dec/vax/tests/decodediff.js

Three phases: the entire 335,444-instruction EHKAA diagnostic replayed instruction by
instruction; a randomized exerciser covering every addressing mode against every access type the
CPU can reach, including register-conflict cases; and reserved-addressing faults that grade the
recovery-queue unwind.  Every read the decoder issues is matched against the address, length and
access type SIMH used, so an effective-address bug fails exactly rather than probabilistically.

Memory management:

    node machines/dec/vax/tests/mmudiff.js --selfcheck
    node machines/dec/vax/tests/mmudiff.js

Two phases.  150,000 randomized operations against a page-table layout the test builds, with the
same 16MB of real memory on both sides, so translated **physical addresses**, **data values** and
**fault codes** are all compared exactly — across P0, P1, S0 and S1, all four access modes, read
and write access, every alignment, page-boundary and longword-boundary crossings, both TB
invalidation instructions, MAPEN off and on, and page-table mutation underneath a live TB.  Then
the EHKAA diagnostic is run to its PASS halt with the MMU trace armed and its **343,557** real
memory-management operations are replayed, grading the addresses we compute, the TB hit/miss
decisions we make, and the faults we raise.

The randomized phase's virtual-address pool is built from *hot pages* with offsets concentrated at
page boundaries, for the reason stated below; the run fails if operations per region, operations
per access mode, occurrences of each fault code, cross-page accesses, unaligned accesses,
two-level page-table walks, M-bit write-backs, or the fraction of comparisons against non-zero
data drops below its floor.  `--ops` below 100,000 fails outright rather than scaling the floors
down with it.

Instruction execution (`modules/v2/cpu.js`) currently covers the Base Instruction Group's 107
integer, logical, and variable-length-bit-field opcodes (control flow, floating point, and
string/queue/CIS belong to sibling items):

    node machines/dec/vax/tests/intdiff.js --selfcheck
    node machines/dec/vax/tests/intdiff.js --simh PATH --ehkaa PATH

Two phases.  A randomized differential drives a **live** SIMH console (`deposit`/`step 1`/
`examine`) through N pre-states per opcode (`--cases-per-opcode`, default 150, floor 40) — edge-
weighted register and memory values (0, ±1, signed min/max, all-ones), condition codes, and both
register and memory destinations — and compares the full 16-register file plus PSL after one
instruction.  Then the entire EHKAA diagnostic trace (the same capture `decodediff.js` uses) is
scanned for every instance of one of these 107 opcodes; the next trace record's **pre-resolution**
register file (patch 0002's `PREG`, not patch 0001's post-resolution `REGS` — the latter already
carries the *next* instruction's own autoincrement side effects, which was a real bug caught only
by running this against 335,444 real instructions) is the exact post-execution ground truth,
provided (checked per instance) no trap or interrupt intervened between the two.  Memory-mode
variable-bit-field instructions need an execution-time read the decode-phase `MEMR` log does not
contain and are skipped in this phase (counted, never silently dropped) — exclusively the
randomized phase's job, which it does. Both execution-time faults (a state no addressing-mode
legality check the decoder itself enforces would catch) and deferred arithmetic traps (integer
overflow/divide-by-zero, which SIMH dispatches at the top of the *next* instruction and which are
therefore invisible to a single-instruction comparison either way) are out of scope; `cpu.js`'s
file header states exactly why. `--selfcheck` mutates the shipped `HANDLERS` dispatch table itself.

All four tests assert their own coverage and **fail** when it is not met — an undersized run does
not quietly pass — and all four ship `--selfcheck`, which injects deliberate defects into the
shipped code path and fails if the differential does not catch each one.  `mmudiff.js`'s first
mutation is the `>>` vs `>>>` page-table-index hazard described above.

The simulator is located via `--simh PATH`, then `$SIMH_BIN` / `$SIMH_DECODE_BIN` /
`$SIMH_MMU_BIN` / `$SIMH_INT_BIN`, then `../pcjs-vax/open-simh/BIN/microvax3900` (`busdiff.js`) or
`$TMPDIR/pcjs-vax-simh/...` (`mmudiff.js` and `intdiff.js`, which need patch 0003 and patches
0001+0002 respectively). If it cannot be found, the tests fail rather than falling back to
self-comparison.

Floating point:

    machines/dec/vax/tests/simh/build.sh $TMPDIR/pcjs-vax-simh-fp   # needs patch 0004
    node machines/dec/vax/tests/fpadiff.js --selfcheck
    node machines/dec/vax/tests/fpadiff.js

Three phases.  A randomized exerciser of **60,000** cases over every one of the 61 in-scope
opcodes, both destination kinds, `PSL<FU>` and `PSL<IV>` both ways — with operands generated by
*class* rather than uniformly, because two random longwords are two ordinary numbers of wildly
different magnitude and adding them returns the larger one unchanged.  The classes are true zero,
reserved operand (including the exact `-0.0` bit pattern), bottom-of-range and top-of-range
exponents, **exact rounding ties** (the second operand scaled so its leading bit lands precisely
on the round bit: an exponent difference of 24 for F, 56 for D, 53 for G) and their two
neighbours, near-equal opposite-sign pairs that cancel and force a deep renormalization, exponent
differences straddling every shift-count branch in `dp_lsh`/`dp_rsh` (0, 1-31, 32-63, ≥ 64 — where
a JavaScript port dies, because `x << 32` is `x`, not 0), overflow, underflow, divide by zero, and
float-to-integer conversions landing **exactly** on the asymmetric integer limit.  Then the EHKAA
diagnostic's own **884** F/D/G instructions, lifted opcode-by-opcode out of the instruction
history and graded both against the result the diagnostic itself printed and against a replay.
Then `WRITE_Q`'s cross-page probe with memory management on, storing quadwords across every
boundary between writable, read-only, no-access and invalid pages, with the whole data area read
back on both sides.

The run fails if any per-opcode or per-class floor is missed, if fewer than 60% of stored results
are non-trivial, or if any case fails to reach a comparison.  `--cases` below 40,000 fails
outright rather than scaling the floors down with it.  `--selfcheck` injects twelve deliberate
defects into the shipped code path — two of them rounding constants and one a normalization loop —
and fails if the differential does not catch each.

The decode ROM in `modules/v2/drom.js` is **generated**, not transcribed, from Open SIMH's
`vax_sys.c` and `vax_defs.h`:

    node machines/dec/vax/tests/gen_drom.js            # regenerate
    node machines/dec/vax/tests/gen_drom.js --check    # verify the committed copy matches
