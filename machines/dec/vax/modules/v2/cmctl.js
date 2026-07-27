/**
 * @fileoverview Implements the KA655 CMCTL memory-controller register file
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * ============================================================================
 * WHAT THIS IS
 * ============================================================================
 * pcjsvax-622.  vax_sysdev.c's cmctl_rd()/cmctl_wr() (:1106-1163), reached through the regtable[]
 * entry `{ CMCTLBASE, CMCTLBASE+CMCTLSIZE, &cmctl_rd, &cmctl_wr }` (:1005).  CMCTLBASE is
 * REGBASE+0x100, so this is a sub-device of the register block (regblock.js).
 *
 * WHY IT MATTERS, MEASURED -- AND WHAT IT DOES *NOT* FIX ON ITS OWN
 * ============================================================================
 * With this range undecoded, the ROM bus-faults reading REG+0x140 (register 16) at instruction
 * #2,424,767.  That is 2.1 MILLION instructions past anything tests/romdiff.js can walk to, which
 * is why an output-based instrument (tests/conoutdiff.js) found it and a trace-based one
 * structurally could not.
 *
 * pcjsvax-622's own progress note recorded that fault as the cause of the ROM's `?91` self-test
 * failure.  IT IS NOT, and the correction is written here rather than left in a session log,
 * because the next reader will otherwise re-derive it.  MEASURED, four boots of the same ROM, same
 * 3,000,000-instruction budget, reading the console byte stream:
 *
 *      neither decoded                     40.. then `?91 2 03 EF 04 0000`
 *      CMCTL decoded, doorbell not         40.. then `?91 2 03 EF 04 0000`   (unchanged)
 *      doorbell decoded, CMCTL not         40..39..38.. then `?33 2 01 EF 04 0000`
 *      both decoded                        40..39..38..37..36..35..34..33..32..31.. then `?53`
 *
 * `?91` is the CQBIC doorbell: the ROM reads the Qbus I/O page's interprocessor-communication
 * register at IOPAGE+0x1F40 at instruction #2,424,717 -- FIFTY INSTRUCTIONS BEFORE the CMCTL read
 * -- and vax_io.c's `dbl_rd()` answers it on the oracle while this bus does not.  Decoding CMCTL
 * moves the ROM's countdown from 38 to 31 and is necessary for the oracle's own sequence; it is not
 * sufficient, and no comment here should imply that it is.  The doorbell is a Qbus I/O-page device
 * and is explicitly out of pcjsvax-622's scope.
 *
 * SIMH's own comment (:1095-1097) is the scope statement, and it is honoured here: the sixteen
 * configuration registers "are here merely to entertain the firmware; the actual configuration of
 * memory is unaffected by the settings here."  Nothing in this file moves, sizes or remaps memory.
 * What it DOES model exactly is what the firmware can observe: the read masks, the write masks, the
 * signature-request side effect, and the THREE registers that are not configuration at all.
 *
 * ============================================================================
 * THE REGISTER COUNT IS COMPUTED, NOT TRANSCRIBED (HANDOFF.md standing rule 5)
 * ============================================================================
 * vaxmod_defs.h:206 writes `CMCTLSIZE (19 << 2)` with the 19 spelled out.  That 19 is NOT copied
 * here.  It is derived below from the two quantities that actually determine it --
 *
 *      config registers = MAXMEMSIZE / MEM_BANK      (one per 4MB bank of the largest standard
 *                                                     KA655 memory, 0x04000000 / 0x00400000 = 16)
 *      + CMCTL_SPECIAL_REGS                          (err status, CSR, KA655X extension)
 *
 * -- and tests/cmctldiff.js's SPAN phase then re-derives the same span a THIRD way, from the LIVE
 * ORACLE, by walking longwords upward from CMCTLBASE and classifying each as decoded / decoded-but-
 * machine-checking / undecoded (the last is distinguishable because only the undecoded case sets
 * the SSC bus-timeout bits -- see the EXTENSION REGISTER section below).  If any two of the three
 * disagree the run fails and says which.  The project's CIS opcode count went 7 -> 11 -> 17 -> 23
 * and every wrong value along the way was hand-derived; this one is arithmetic that a reader can
 * check and a test that a machine can.
 *
 * ============================================================================
 * REGISTER 18 IS A MACHINE CHECK, NOT A BUS FAULT -- AND THE DIFFERENCE IS OBSERVABLE
 * ============================================================================
 * `cmctl_rd()`'s case 18 is:
 *
 *      if (MEMSIZE > MAXMEMSIZE) return ((int32) MEMSIZE);
 *      MACH_CHECK (MCHK_READ);
 *
 * and `cmctl_wr()`'s case 18 is `MACH_CHECK (MCHK_WRITE)` unconditionally, at ANY memory size.
 * That is NOT the same thing as an undecoded address.  ReadReg()/WriteReg()'s fall-through
 * (:1031, :1070) does `ssc_bto |= SSCBTO_BTO | SSCBTO_RWT` BEFORE its MACH_CHECK; cmctl_rd()'s
 * does not touch ssc_bto at all.  MEASURED on the live oracle, 16MB, one instruction each:
 *
 *      MOVL @#20080148,R0   -> PC = the SCB_MCHK handler,  BTO = 00000000
 *      MOVL #x,@#20080148   -> PC = the SCB_MCHK handler,  BTO = 00000000
 *      MOVL @#2008014C,R0   -> PC = the SCB_MCHK handler,  BTO = C0000000   (one longword past
 *                                                                           the end of CMCTL)
 *
 * so "did the machine check set the bus-timeout bits" is exactly what tells a decoded-but-checking
 * register apart from an undecoded one, and the ROM can read BTO.  Before this item nothing in this
 * tree could express that: every device signalled a fault by returning null, which regblock.js
 * routes to MemoryVAX.readNone() -> BusVAX.fault() -> CPUStateVAX.onBusFault(), and that path
 * always set the bus-timeout bits.  So this item adds ONE seam: a device may return
 * `REG_MCHK` (regblock.js) instead of null, and regblock.js then raises the same machine check with
 * BusVAX.fault()'s new `fNoBto` argument.  cqbic.js's file header names TWO other call sites that
 * want exactly this seam and had to disclose a gap for want of it (cqmap_rd()'s read branch, and
 * cqm_rd()/cqm_wr()); wiring THOSE is pcjsvax-5c1's, not this item's -- the seam is built here
 * because register 18 cannot be modelled without it, and this file is its first caller.
 *
 * At MEMSIZE > MAXMEMSIZE (the KA655X configuration -- `set cpu 128m` on the oracle) the READ
 * instead returns the memory size and does not fault, while the WRITE still machine-checks.  Both
 * sides are driven and graded: tests/cmctldiff.js's EXT phase builds a second machine with 128MB
 * and runs the same instructions on both engines.  A "just always answer with the size"
 * implementation passes the 128MB half and fails the 16MB half, which is the normal configuration.
 *
 * ============================================================================
 * A SUB-LONGWORD WRITE CLOBBERS, IT DOES NOT MERGE
 * ============================================================================
 * Every other register device in this tree (ssc.js, cqbic.js, nvr.js) read-modify-merges a byte or
 * word write into the register's current contents, because their C counterparts do.  cmctl_wr()
 * does NOT:
 *
 *      if (lnt < L_LONG) { int32 sc = (pa & 3) << 3; val = val << sc; }
 *
 * -- it shifts the written lane into position and then applies the SAME full-longword masks the
 * longword path uses, so the untouched lanes of CMCNF_RW are cleared rather than preserved.
 * MEASURED, because it reads like a bug and is not: preload register 0 with 0x9FF00000, then
 * `MOVB #FF,@#20080103`, and the oracle reads back 0x9F000000 -- not 0x9FF00000.  A merge
 * implementation returns 0x9FF00000 and is caught by cmctldiff.js's MERGE phase and by its
 * `sub-longword-write-merges-instead-of-clobbering` mutation.
 *
 * UNALIGNED longword and word writes are NOT modelled here and are not silently skipped: SIMH
 * routes them through WriteRegU() (vax_sysdev.c:1084-1091), which read-modify-writes via
 * ReadReg()+WriteReg(), while this bus reaches them through MemoryVAX's own byte/word stitching.
 * That is the identical, already-disclosed gap cdg.js's header names; cmctldiff.js grades word
 * writes only at the two genuinely-aligned lanes (0 and 2) and says so.
 *
 * ============================================================================
 * WHAT THE SIGNATURE REQUEST ACTUALLY DOES
 * ============================================================================
 * Writing a configuration register with CMCNF_SRQ set re-signs a GROUP OF FOUR registers -- `rg &
 * ~3` through `rg | 3`, not the written register alone -- setting each one's five signature bits to
 * MEM_SIG when that bank's base address is inside memory and clearing them when it is not.
 * MEASURED at 16MB (four 4MB banks): writing SRQ to register 1 leaves registers 0-3 reading 0x17,
 * and writing SRQ to register 5 leaves registers 4-7 reading 0x00, because banks 4-7 start at 16MB
 * and up.  Both the group rounding and the bank test are separately mutated in cmctldiff.js
 * (`signature-request-signs-only-the-written-register` and `every-bank-signs-as-populated`).
 */

import { VAX } from "./defines.js";
import { REG_MCHK } from "./regblock.js";

const CMCTL_BASE = (VAX.PHYSMEM.REG_BASE + 0x100) >>> 0;   // vaxmod_defs.h:207, CMCTLBASE

/* vax_sysdev.c:91-98 and :103-122.  Transcribed as the C's own DEFINES; every derived quantity
   below is computed from them rather than restated. */
const CMCNF_VLD  = 0x80000000|0;                // addr valid
const CMCNF_BA   = 0x1FF00000;                  // base addr
const CMCNF_SRQ  = 0x00000020;                  // signature request, write-only
const CMCNF_SIG  = 0x0000001F;                  // signature
const CMCNF_RW   = (CMCNF_VLD | CMCNF_BA) | 0;
const CMCNF_MASK = (CMCNF_RW | CMCNF_SIG) | 0;
const MEM_BANK   = 1 << 22;                     // bank size, 4MB
const MEM_SIG    = 0x17;                        // ECC, 4 x 4MB

const CMERR_RDS  = 0x80000000|0;
const CMERR_FRQ  = 0x40000000;
const CMERR_CRD  = 0x20000000;
const CMERR_DMA  = 0x00000100;
const CMERR_BUS  = 0x00000080;
const CMERR_W1C  = (CMERR_RDS | CMERR_FRQ | CMERR_CRD | CMERR_DMA | CMERR_BUS) | 0;

const CMCSR_PMI  = 0x00002000;
const CMCSR_CRD  = 0x00001000;
const CMCSR_DET  = 0x00000400;
const CMCSR_FDT  = 0x00000200;
const CMCSR_DCM  = 0x00000080;
const CMCSR_SYN  = 0x0000007F;
const CMCSR_MASK = (CMCSR_PMI | CMCSR_CRD | CMCSR_DET | CMCSR_FDT | CMCSR_DCM | CMCSR_SYN) | 0;

/**
 * The three registers that are not per-bank configuration, in the order cmctl_rd()'s switch names
 * them.  They are the ONLY hand-written part of the register count, and they are named rather than
 * counted so that the arithmetic below reads as what it is.
 */
const CMCTL_SPECIAL = ["CMERR", "CMCSR", "CMEXT"];

/** One configuration register per 4MB bank of the largest STANDARD KA655 memory -- see the file
    header.  0x04000000 / 0x00400000 = 16. */
const CMCTL_CONFIG_REGS = (VAX.PHYSMEM.MAXMEMSIZE / MEM_BANK) | 0;

/** vaxmod_defs.h:206's `CMCTLSIZE (19 << 2)`, DERIVED.  See the file header. */
const CMCTL_REGS   = CMCTL_CONFIG_REGS + CMCTL_SPECIAL.length;
const CMCTL_LENGTH = CMCTL_REGS << 2;

const REG_CMERR = CMCTL_CONFIG_REGS;            // 16
const REG_CMCSR = CMCTL_CONFIG_REGS + 1;        // 17
const REG_CMEXT = CMCTL_CONFIG_REGS + 2;        // 18

if (CMCTL_CONFIG_REGS * MEM_BANK !== VAX.PHYSMEM.MAXMEMSIZE) {
    throw new Error(`cmctl.js: ${CMCTL_CONFIG_REGS} banks of 0x${MEM_BANK.toString(16)} bytes is ` +
        `not VAX.PHYSMEM.MAXMEMSIZE (0x${VAX.PHYSMEM.MAXMEMSIZE.toString(16)}) -- the bank ` +
        `geometry and the memory map disagree; one of them is wrong.`);
}

/**
 * @class CMCTLVAX
 */
export default class CMCTLVAX {
    /**
     * CMCTLVAX(memSize)
     *
     * @param {number} memSize the machine's configured memory size -- SIMH's `MEMSIZE`, which is
     *   `cpu_unit.capac`, the size `SET CPU nnnM` sets and the size the caller passed to
     *   BusVAX.addMemory().  Two branches read it: register 18's `MEMSIZE > MAXMEMSIZE` test, and
     *   the signature request's `ADDR_IS_MEM (i * MEM_BANK)`.  It is a constructor argument rather
     *   than a bus query because SIMH's is: `ADDR_IS_MEM` compares against the CPU unit's capacity,
     *   not against whatever happens to answer on the bus.
     */
    constructor(memSize)
    {
        this.memSize = (memSize >>> 0) || 0;
        this.reset();
    }

    /**
     * reset()
     *
     * vax_sysdev.c:1780-1782, sysd_powerup(): `for (i = 0; i < (CMCTLSIZE >> 2); i++) cmctl_reg[i]
     * = 0;`.  The array is also a zero-initialized C static, so this is its value from the first
     * instruction of any run as well as after a power-up -- the same reasoning cdg.js's and
     * nvr.js's reset() comments give.
     *
     * @this {CMCTLVAX}
     */
    reset()
    {
        this.reg = new Int32Array(CMCTL_REGS);
    }

    /**
     * readReg(rg)
     *
     * vax_sysdev.c:1106-1128, cmctl_rd().  Note that the ERROR register is the one register read
     * back UNMASKED -- its `case 16` returns `cmctl_reg[rg]` with no `&` at all, unlike the
     * configuration registers (CMCNF_MASK) and the CSR (CMCSR_MASK).
     *
     * @this {CMCTLVAX}
     * @param {number} rg
     * @returns {number|symbol} the register's value as an int32, or REG_MCHK when the reference is
     *   a machine check that must NOT set the SSC bus-timeout bits (register 18 below the KA655X
     *   memory threshold -- see the file header).
     */
    readReg(rg)
    {
        switch (rg) {
        case REG_CMERR:
            return this.reg[REG_CMERR] | 0;
        case REG_CMCSR:
            return (this.reg[REG_CMCSR] & CMCSR_MASK) | 0;
        case REG_CMEXT:
            if (this.memSize > VAX.PHYSMEM.MAXMEMSIZE) return this.memSize | 0;
            return REG_MCHK;
        }
        return (this.reg[rg] & CMCNF_MASK) | 0;
    }

    /**
     * writeReg(rg, val, lnt, addr)
     *
     * vax_sysdev.c:1130-1163, cmctl_wr().  `val` arrives RIGHT-JUSTIFIED (the convention every
     * caller in this tree uses); the `lnt < L_LONG` shift below is the C's own, and it is a SHIFT
     * INTO POSITION rather than a merge -- see the file header, and cmctldiff.js's MERGE phase.
     *
     * The masks and the signature constants are class data rather than inline literals so that
     * cmctldiff.js's --selfcheck can PERTURB the shipped computation instead of substituting its
     * own copy of it (HANDOFF.md standing rule 11).
     *
     * @this {CMCTLVAX}
     * @param {number} rg
     * @param {number} val right-justified
     * @param {number} lnt 1, 2 or 4
     * @param {number} addr the absolute physical address, for the C's `(pa & 3) << 3`
     * @returns {boolean|symbol} true when the write was accepted, or REG_MCHK for register 18.
     */
    writeReg(rg, val, lnt, addr)
    {
        if (lnt < CMCTLVAX.L_LONG) {
            let sc = ((addr >>> 0) & 3) << 3;
            val = (val << sc) | 0;
        }
        switch (rg) {
        case REG_CMERR:
            this.reg[REG_CMERR] = (this.reg[REG_CMERR] & ~(val & CMCTLVAX.CMERR_W1C)) | 0;
            return true;
        case REG_CMCSR:
            this.reg[REG_CMCSR] = (val & CMCTLVAX.CMCSR_MASK) | 0;
            return true;
        case REG_CMEXT:
            return REG_MCHK;
        }
        if (val & CMCTLVAX.CMCNF_SRQ) {
            let g = rg & CMCTLVAX.CMCNF_SRQ_GROUP_MASK;
            for (let i = g; i < g + CMCTLVAX.CMCNF_SRQ_GROUP; i++) {
                this.reg[i] = (this.reg[i] & ~CMCNF_SIG) | 0;
                if (this.isMemBank(i)) this.reg[i] = (this.reg[i] | CMCTLVAX.MEM_SIG) | 0;
            }
        }
        this.reg[rg] = ((this.reg[rg] & ~CMCNF_RW) | (val & CMCNF_RW)) | 0;
        return true;
    }

    /**
     * isMemBank(i)
     *
     * cmctl_wr()'s `ADDR_IS_MEM (i * MEM_BANK)`, i.e. `(i * MEM_BANK) < MEMSIZE` (vaxmod_defs.h:127).
     * Its own method so cmctldiff.js's `every bank signs as populated` mutation can perturb exactly
     * this predicate and nothing else.
     *
     * @this {CMCTLVAX}
     * @param {number} i bank / configuration-register index
     * @returns {boolean}
     */
    isMemBank(i)
    {
        return (i * MEM_BANK) < this.memSize;
    }

    /**
     * readLong(addr) / readWord(addr) / readByte(addr)
     *
     * `addr` is the ABSOLUTE physical address; regblock.js's dispatcher passes it through and this
     * class subtracts its own base, the same convention cqbic.js and ka655.js use.  vax_mmu.h's
     * Read() hands the register routine the whole longword and does the extraction in the CALLER,
     * which is why all three sizes go through readReg().
     *
     * @this {CMCTLVAX}
     * @param {number} addr
     * @returns {number|symbol}
     */
    readLong(addr) { return this.readReg(this.regOf(addr)); }

    readWord(addr)
    {
        let v = this.readReg(this.regOf(addr));
        if (typeof v !== "number") return v;
        return (v >>> ((addr & 2) ? 16 : 0)) & 0xFFFF;
    }

    readByte(addr)
    {
        let v = this.readReg(this.regOf(addr));
        if (typeof v !== "number") return v;
        return (v >>> ((addr & 3) << 3)) & 0xFF;
    }

    /**
     * writeLong(addr, val) / writeWord(addr, val) / writeByte(addr, val)
     *
     * @this {CMCTLVAX}
     * @param {number} addr
     * @param {number} val right-justified
     * @returns {boolean|symbol}
     */
    writeLong(addr, val) { return this.writeReg(this.regOf(addr), val | 0, 4, addr); }

    writeWord(addr, val) { return this.writeReg(this.regOf(addr), val & 0xFFFF, 2, addr); }

    writeByte(addr, val) { return this.writeReg(this.regOf(addr), val & 0xFF, 1, addr); }

    /**
     * regOf(addr)
     *
     * cmctl_rd()/cmctl_wr()'s `(pa - CMCTLBASE) >> 2`.
     *
     * @this {CMCTLVAX}
     * @param {number} addr absolute physical
     * @returns {number}
     */
    regOf(addr) { return (((addr >>> 0) - CMCTL_BASE) >>> 2); }
}

/* cmctl_wr()'s own constants, published as class data so tests/cmctldiff.js's --selfcheck can
   perturb the SHIPPED computation rather than substitute a copy of it (standing rule 11).
   CMCNF_SRQ_GROUP is the C's "group of 4" and CMCNF_SRQ_GROUP_MASK is its `rg & ~3`; they are two
   names for one quantity and are kept separately only so a mutation can break the ROUNDING without
   also changing how many registers the loop visits. */
CMCTLVAX.L_LONG = 4;
CMCTLVAX.CMERR_W1C = CMERR_W1C;
CMCTLVAX.CMCSR_MASK = CMCSR_MASK;
CMCTLVAX.CMCNF_SRQ = CMCNF_SRQ;
CMCTLVAX.CMCNF_SRQ_GROUP = 4;
CMCTLVAX.CMCNF_SRQ_GROUP_MASK = ~3;
CMCTLVAX.MEM_SIG = MEM_SIG;

export {
    CMCTL_BASE, CMCTL_LENGTH, CMCTL_REGS, CMCTL_CONFIG_REGS, CMCTL_SPECIAL,
    REG_CMERR, REG_CMCSR, REG_CMEXT,
    CMCNF_VLD, CMCNF_BA, CMCNF_SRQ, CMCNF_SIG, CMCNF_RW, CMCNF_MASK,
    CMERR_W1C, CMCSR_MASK, MEM_BANK, MEM_SIG
};
