---
layout: page
title: DEC VAX
permalink: /machines/dec/vax/
---

DEC VAX
-------

A MicroVAX 3900 (KA655) machine for PCjs, ported from [Open SIMH](https://github.com/open-simh/simh)
(MIT, © 1998-2019 Robert M Supnik).  Work in progress: at present this directory contains the
physical memory and bus layer, plus instruction decode and operand resolution — no instruction
execution, no MMU, no devices.

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

Both tests assert their own coverage and **fail** when it is not met — an undersized run does not
quietly pass — and both ship `--selfcheck`, which injects deliberate defects and fails if the
differential does not catch each one.

The simulator is located via `--simh PATH`, then `$SIMH_BIN` / `$SIMH_DECODE_BIN`, then
`../pcjs-vax/open-simh/BIN/microvax3900`.  If it cannot be found, the tests fail rather than
falling back to self-comparison.

The decode ROM in `modules/v2/drom.js` is **generated**, not transcribed, from Open SIMH's
`vax_sys.c` and `vax_defs.h`:

    node machines/dec/vax/tests/gen_drom.js            # regenerate
    node machines/dec/vax/tests/gen_drom.js --check    # verify the committed copy matches
