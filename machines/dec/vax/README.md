---
layout: page
title: DEC VAX
permalink: /machines/dec/vax/
---

DEC VAX
-------

A MicroVAX 3900 (KA655) machine for PCjs, ported from [Open SIMH](https://github.com/open-simh/simh)
(MIT, © 1998-2019 Robert M Supnik).  Work in progress: at present this directory contains the
physical memory and bus layer only — no CPU, no MMU, no devices.

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

### Testing

The memory/bus layer is graded by a randomized differential test against a **real, executed**
`microvax3900` binary — no fixtures, no golden files:

    node machines/dec/vax/tests/busdiff.js --selfcheck

150,000 random operations across every access size (byte/word/longword/quadword) and alignment,
including accesses that straddle 8KB block boundaries and the top of physical memory, and a heavy
mix of S0 addresses above `0x80000000` delivered both as positive doubles and as negative int32s.
`--selfcheck` additionally injects deliberate defects and fails if the differential does not catch
them.

The simulator is located via `--simh PATH`, then `$SIMH_BIN`, then
`../pcjs-vax/open-simh/BIN/microvax3900`.  If it cannot be found, the test fails rather than
falling back to self-comparison.
