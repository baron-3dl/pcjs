/**
 * @fileoverview VAX compile-time definitions
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2012-2026 Jeff Parsons, © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 * Portions adapted from PCjs, Copyright © 2012-2026 Jeff Parsons, used under the MIT
 * license.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this
 * work.
 */

import { APPVERSION, COMPILED, COPYRIGHT, CSSCLASS, DEBUG, DEBUGGER, LICENSE, MAXDEBUG, PRIVATE, RS232, SITEURL, globals } from "../../../../modules/v2/defines.js";

/**
 * @define {string}
 */
const APPCLASS = "vax";         // this @define is the default application class (eg, "pcx86", "pdp11")

/**
 * APPNAME is used more for display purposes than anything else now.  APPCLASS is what matters in terms
 * of folder and file names, CSS styles, etc.
 *
 * @define {string}
 */
const APPNAME = "VAXjs";        // this @define is the default application name (eg, "PCx86", "PDPjs")

/**
 * TYPEDARRAYS enables use of typed arrays for Memory blocks.  Unlike PDPjs (which retains a
 * non-typed-array fallback for very old browsers), VAXjs requires typed arrays: every VAX Memory
 * block is an Int32Array of little-endian longwords.  See memory.js.
 *
 * @define {boolean}
 */
const TYPEDARRAYS = true;

/**
 * ============================================================================
 * THE UNSIGNED ADDRESS CONVENTION -- READ THIS BEFORE WRITING ANY VAX CODE
 * ============================================================================
 *
 * VAX S0 (system) space begins at 0x80000000.  As a JavaScript int32 that value is NEGATIVE
 * (-2147483648).  Every naive address comparison, block index, mask or increment written the way
 * one would write it for a 16- or 22-bit PDP-11 bus -- or even for PCx86's 32-bit 386 bus, whose
 * machines never actually go above 2GB -- is silently WRONG the first time a VAX touches system
 * space.  PCx86 contains latent instances of exactly this bug; do NOT assume that copying an x86
 * pattern is safe here.
 *
 * Be precise about WHERE it bites, because JavaScript is not uniformly dangerous here.  The
 * operations that are SAFE on a possibly-negative int32 address are the bitwise ones: `&`, `|`,
 * `^`, `<<` and `>>>` all run their operands through ToInt32/ToUint32 first, so
 * `0x80000000 & 0x3FFFFFFF` is 0 whether or not you wrote `>>> 0`, and the result of masking with
 * a mask that clears bit 31 is always a non-negative small integer.  The operations that are NOT
 * safe, and that are where every real instance of this bug will live, are:
 *
 *      >>                      signed shift: 0x80001000 >> 13 is -262139, not 262148
 *      <  >  <=  >=            relational compare: 0x80000000 < 0x1000 is TRUE as an int32
 *      +  -  *  /  %           arithmetic, including Math.floor(addr / n)
 *      ===  !==  switch        against any literal above 0x7FFFFFFF
 *      arr[addr]  map.get()    a negative index silently yields undefined
 *      .toString(16)  printf   "-80000000" is not an address
 *
 * The convention, which every later VAXjs module inherits:
 *
 *   1. NORMALIZE AT THE BOUNDARY, THEN STOP WORRYING.
 *      Every address entering a public Bus interface is normalized on the FIRST line:
 *
 *          addr = (addr >>> 0) & VAX.PAMASK;
 *
 *      After that the value is provably in 0..0x3FFFFFFF, so every operation in the list above is
 *      safe on it and no further coercion is needed (or wanted -- it is noise).  The `>>> 0` is
 *      redundant when it is immediately followed by that mask; it is written anyway because it is
 *      the idiom you must carry UNCHANGED into the places where the mask is absent or too wide.
 *      Which brings us to the part that actually matters:
 *
 *   2. VIRTUAL ADDRESSES ARE 32 BITS AND CANNOT BE MASKED DOWN.  THIS IS WHERE IT BITES.
 *      The physical bus is 30 bits, so PAMASK rescues it.  The MMU is not: a VAX virtual address
 *      uses all 32 bits, and the region select bits <31:30> are the WHOLE POINT (00/01 = P0/P1,
 *      10 = S0, 11 = reserved).  There is no mask that makes an S0 address non-negative.  Any code
 *      that classifies, compares, indexes, or does arithmetic on a virtual address MUST coerce
 *      with `>>> 0` explicitly, and `va >>> 30` (never `va >> 30`) is how you get the region.
 *
 *   3. SHIFT ADDRESSES WITH `>>>`, NEVER `>>`.  There is no case in this codebase where `>>` on an
 *      address is correct, so a reviewer can treat any occurrence as a defect without thinking.
 *
 *   4. ADDRESS ARITHMETIC RE-ENTERS RULE 1.
 *      addr + 1 at the top of the physical space must wrap by going back through a normalizing
 *      entry point, exactly as SIMH re-masks every byte address it touches in cpu_ex()/cpu_dep().
 *      Never carry an un-normalized sum across a call boundary.
 *
 *   5. DATA LONGWORDS ARE SIGNED int32; BYTES AND WORDS ARE ZERO-EXTENDED.
 *      This is the deliberate exception, and it is about data, not addresses.  Memory blocks are
 *      Int32Array, JS bitwise operators produce int32, and SIMH stores memory as uint32 longwords
 *      that round-trip identically -- so getLong() returns a signed int32, possibly negative.
 *      Keeping longwords in int32/Smi range avoids boxing every value the CPU touches.  Apply
 *      `>>> 0` at the point of use when you need the unsigned value: when formatting, when
 *      comparing magnitudes, and ALWAYS when a longword is about to be used as an address -- at
 *      which point rules 1-4 take over, and rule 2 is the one that will get you.
 *      getByte()/getWord() return 0..0xff / 0..0xffff and are never negative.
 *
 * Summary for reviewers: `>>` on an address is a bug.  A relational compare or an array index on
 * an address that has not been through rule 1 (physical) or an explicit `>>> 0` (virtual) is a bug.
 * ============================================================================
 */

/**
 * @class VAX
 * @unrestricted
 */
class VAX {}

/*
 * Physical address space (KA655 / CVAX).  From Open SIMH vax_defs.h and vaxmod_defs.h.
 *
 * PAWIDTH is 30, NOT 32.  bus.js eagerly allocates nBlockTotal = addrTotal / nBlockSize slots at
 * construction, so a 32-bit bus would allocate millions of slots up front for no benefit: the CVAX
 * CDAL bus simply does not have address lines 30 and 31.
 */
VAX.PAWIDTH         = 30;                       // physical address width (vax_defs.h:137)
VAX.PASIZE          = 0x40000000;               // 1 << PAWIDTH
VAX.PAMASK          = 0x3FFFFFFF;               // PASIZE - 1

/*
 * KA655 physical memory map (vaxmod_defs.h).  This item RESERVES these ranges; it does not decode
 * them.  Anything not backed by RAM reads/writes as non-existent memory (see MemoryVAX.readNone).
 */
VAX.PHYSMEM = {
    MAXMEMSIZE:     0x04000000,                 // 64MB, max memory on a standard KA655
    MAXMEMSIZE_X:   0x20000000,                 // 512MB, max memory on a KA655X
    INITMEMSIZE:    0x01000000,                 // 16MB, the SIMH microvax3900 default
    CDG_BASE:       0x10000000,                 // cache diagnostic space
    CDG_LENGTH:     0x04000000,                 // CDASIZE * CTGSIZE (64KB * 1024)
    IOPAGE_BASE:    0x20000000,                 // Qbus I/O page
    IOPAGE_LENGTH:  0x00002000,                 // 8KB
    ROM_BASE:       0x20040000,                 // KA655 boot/diagnostic ROM (appears twice)
    ROM_LENGTH:     0x00040000,                 // ROMSIZE * 2 (128KB * 2) -- the full mirrored span
    ROM_SIZE:       0x00020000,                 // ROMSIZE (vaxmod_defs.h:174) -- one copy, unmirrored
    REG_BASE:       0x20080000,                 // local register space (CQBIC, CMCTL, KA655 regs)
    REG_LENGTH:     0x00080000,
    SSC_BASE:       0x20140000,                 // system support chip
    SSC_LENGTH:     0x00000150,
    NVR_BASE:       0x20140400,                 // non-volatile RAM
    NVR_LENGTH:     0x00000400,                 // 1KB
    CQM_BASE:       0x30000000,                 // CQBIC Qbus memory space, as seen from the CVAX
    CQM_LENGTH:     0x00400000                  // 4MB
};

/*
 * Access types, reported to the fault handler for diagnostic purposes only.  Mapping a
 * non-existent-memory fault onto a VAX machine check / SCB vector is deliberately NOT done here;
 * that belongs with the CPU and the MMU, and is left to those items.
 */
VAX.ACCESS = {
    READ:           0x1,
    WRITE:          0x2,
    BYTE:           0x4,
    WORD:           0x8,
    LONG:           0x10,
    QUAD:           0x20
};
VAX.ACCESS.READ_BYTE  = VAX.ACCESS.READ  | VAX.ACCESS.BYTE;
VAX.ACCESS.READ_WORD  = VAX.ACCESS.READ  | VAX.ACCESS.WORD;
VAX.ACCESS.READ_LONG  = VAX.ACCESS.READ  | VAX.ACCESS.LONG;
VAX.ACCESS.READ_QUAD  = VAX.ACCESS.READ  | VAX.ACCESS.QUAD;
VAX.ACCESS.WRITE_BYTE = VAX.ACCESS.WRITE | VAX.ACCESS.BYTE;
VAX.ACCESS.WRITE_WORD = VAX.ACCESS.WRITE | VAX.ACCESS.WORD;
VAX.ACCESS.WRITE_LONG = VAX.ACCESS.WRITE | VAX.ACCESS.LONG;
VAX.ACCESS.WRITE_QUAD = VAX.ACCESS.WRITE | VAX.ACCESS.QUAD;

/**
 * isQbusAddr(addr)
 *
 * ADDR_IS_IO(x) || ADDR_IS_CQM(x) (vaxmod_defs.h:234, :229) -- true when a physical reference
 * lands in the Qbus I/O page or the CQBIC's Qbus-memory window, the two ranges a KA655 routes
 * through vax_io.c's ReadQb()/WriteQb() rather than vax_sysdev.c's ReadReg()/WriteReg().  Shared
 * by cpustate.js's onBusFault() (pcjsvax-446/pcjsvax-d22) and mmu.js's writeL()/writeU()
 * (pcjsvax-d22), so the range check cannot drift between the two call sites.
 *
 * @param {number} addr
 * @returns {boolean}
 */
VAX.isQbusAddr = function(addr) {
    let a = addr >>> 0;
    return (a >= VAX.PHYSMEM.IOPAGE_BASE && a < VAX.PHYSMEM.IOPAGE_BASE + VAX.PHYSMEM.IOPAGE_LENGTH) ||
           (a >= VAX.PHYSMEM.CQM_BASE && a < VAX.PHYSMEM.CQM_BASE + VAX.PHYSMEM.CQM_LENGTH);
};

globals.window['VAXjs'] = globals.window['VAXjs'] || {};

export { APPCLASS, APPNAME, APPVERSION, COMPILED, COPYRIGHT, CSSCLASS, DEBUG, DEBUGGER, LICENSE, MAXDEBUG, PRIVATE, RS232, SITEURL, TYPEDARRAYS, VAX, globals };
