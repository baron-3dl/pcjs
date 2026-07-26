---
layout: page
title: DEC VAX
permalink: /machines/dec/vax/
---

DEC VAX
-------

A MicroVAX 3900 (KA655) machine for PCjs, ported from [Open SIMH](https://github.com/open-simh/simh)
(MIT, © 1998-2019 Robert M Supnik).  Work in progress: at present this directory contains the
physical memory and bus layer, instruction decode and operand resolution, and memory management —
no instruction execution, no devices.

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

All three tests assert their own coverage and **fail** when it is not met — an undersized run does
not quietly pass — and all three ship `--selfcheck`, which injects deliberate defects into the
shipped code path and fails if the differential does not catch each one.  `mmudiff.js`'s first
mutation is the `>>` vs `>>>` page-table-index hazard described above.

The simulator is located via `--simh PATH`, then `$SIMH_BIN` / `$SIMH_DECODE_BIN` /
`$SIMH_MMU_BIN`, then `../pcjs-vax/open-simh/BIN/microvax3900` (`busdiff.js`) or
`$TMPDIR/pcjs-vax-simh/...` (`mmudiff.js`, which needs patch 0003).  If it cannot be found, the
tests fail rather than falling back to self-comparison.

The decode ROM in `modules/v2/drom.js` is **generated**, not transcribed, from Open SIMH's
`vax_sys.c` and `vax_defs.h`:

    node machines/dec/vax/tests/gen_drom.js            # regenerate
    node machines/dec/vax/tests/gen_drom.js --check    # verify the committed copy matches
