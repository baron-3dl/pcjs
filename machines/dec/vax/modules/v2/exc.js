/**
 * @fileoverview VAX SCB exception/interrupt dispatch and privileged (processor) registers
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
 * The half of the VAX that is not an instruction: the System Control Block dispatch of every
 * exception and interrupt, the privileged register file the operating system talks to through
 * MTPR/MFPR, the IPL arbitration that decides WHICH pending interrupt (if any) is taken, and the
 * instructions that live entirely inside that machinery -- REI, CHMK/CHME/CHMS/CHMU, LDPCTX,
 * SVPCTX, BPT, XFC, HALT.
 *
 * It is a direct port of:
 *
 *   vax_cpu1.c   intexc(), op_chm(), op_rei(), op_ldpctx(), op_svpctx(), op_mtpr(), op_mfpr()
 *   vax_cpu.c    the abort handler (`abortval < 0` switch, vax_cpu.c:530-604) and the
 *                "non-instruction dispatches, in SRM order" block at the top of sim_instr()'s
 *                main loop (vax_cpu.c:666-720) -- traps, then interrupts, then PSL<TP>
 *   vax_io.c     eval_int(), get_vector()
 *   vax_sysdev.c ReadIPR()/WriteIPR(), for the two behaviors that are NOT device state (SID, and
 *                the write-only/read-only reserved-operand faults) -- see "IPRs that are not on
 *                the CPU chip" below
 *
 * ============================================================================
 * THE THREE THINGS THAT ARE EASY TO GET WRONG
 * ============================================================================
 * 1. WHERE THE CONDITION CODES LIVE.  SIMH splits the PSL while executing: `PSL` holds
 *    everything EXCEPT PSL<3:0>, and the condition codes ride in a separate local `cc` that is
 *    re-merged (`PSL | cc`) whenever the whole longword is needed.  This module keeps the
 *    condition codes IN `cpu.psl` at all times, exactly as decode.js/cpu.js/control.js do, which
 *    changes three lines and no semantics:
 *
 *      SIMH                                    here
 *      oldpsl = PSL | cc                       oldpsl = cpu.psl
 *      PSL = newpsl | ...          (cc lost)   cpu.psl = newpsl | ...    (cc becomes 0, same thing)
 *      PSL = (PSL & PSL_TP) | (newpsl & ~CC_MASK); return newpsl & CC_MASK
 *                                              cpu.psl = (cpu.psl & PSL_TP) | newpsl
 *
 *    The last one is the REI case and is the one worth re-reading: SIMH's caller ORs the returned
 *    cc back in, so the two forms are identical longwords.
 *
 * 2. THE STACK POINTER IS NOT ONE REGISTER.  R[14] is whichever of KSP/ESP/SSP/USP/IS the current
 *    PSL selects.  `stk[0..3]` here are the four per-mode saved stack pointers (SIMH's STK[KERN],
 *    STK[EXEC], STK[SUPV], STK[USER]) and `stk[4]` is the interrupt stack (SIMH's IS == STK[4]).
 *    The LIVE pointer is always R[14]; the entry in `stk[]` for the current mode is STALE until
 *    something (intexc, REI, CHMx, SVPCTX, MTPR) writes it back.  MFPR of KSP must therefore
 *    return R[14] when PSL<IS> is clear and `stk[0]` when it is set -- and MTPR of KSP the mirror
 *    image.  Getting that backwards produces a machine that works perfectly until the first
 *    interrupt nests.
 *
 * 3. A FAULT INSIDE THE EXCEPTION FLOW IS A DIFFERENT EXCEPTION.  `inIE` is SIMH's `in_ie`: set
 *    across intexc() and across the parameter pushes that follow it.  An ACV/TNV raised while it
 *    is set is NOT reported as ACV/TNV -- it becomes SCB_KSNV (kernel stack not valid, a SEVERE
 *    exception forced onto the interrupt stack), and a second one on the interrupt stack halts
 *    the machine.  That is why takeFault() below loops rather than being called once: SIMH gets
 *    the same re-entry for free from longjmp, because its setjmp target is the top of sim_instr.
 *
 * ============================================================================
 * IPRs THAT ARE NOT ON THE CPU CHIP
 * ============================================================================
 * SIMH's op_mtpr/op_mfpr handle the architecturally-defined IPRs inline and pass everything else
 * to WriteIPR()/ReadIPR() in the SYSTEM module (vax_sysdev.c), where they become SSC and CMCTL
 * *device* registers -- an interval timer, a time-of-day clock, a console UART.  Those devices are
 * not this item's scope and are not modelled here.  What IS here is every behavior of that
 * boundary that is not device state:
 *
 *   - MT.SID reads the hardwired CVAX system ID (CVAX_SID | CVAX_UREV), a constant.
 *   - Writing MT.SID, MT.CONPC or MT.CONPSL is a reserved-operand fault.
 *   - Reading MT.SIRR, MT.TBIA, MT.TBIS or MT.TBCHK is a reserved-operand fault (write-only).
 *   - Every other off-chip register number -- including the ones the architecture leaves
 *     RESERVED, which the KA655 does NOT fault on -- reads 0 and ignores writes, which is exactly
 *     what vax_sysdev.c's `default:` case does apart from setting the SSC bus-timeout bit
 *     (ssc_bto), a bit nothing in the CPU ever reads back.  EHKAA exercises two such reserved
 *     numbers (0x0F and 0x28, docs/reference/ehkaa-profile.md §4.2) and this is why they behave.
 *
 * `setIPRDevice(dev)` installs the real device model when the device item lands; until then the
 * numbers in IPR_DEVICE are the ONLY IPRs whose values this module cannot reproduce, and
 * tests/excdiff.js excludes exactly that set from its randomized pool (and asserts that EHKAA
 * touches none of them).
 *
 * ============================================================================
 * WHAT IS DELIBERATELY NOT HERE
 * ============================================================================
 * - MACHINE CHECK (SCB_MCHK).  machine_check() is per-system-model code in vax_sysdev.c that
 *   builds a CVAX-specific parameter block from the bus error state.  Nothing in the current
 *   machine raises it: bus.js reports non-existent memory as a read of all-ones / a discarded
 *   write and defines.js says mapping that onto a machine check is deliberately left to the CPU
 *   and MMU items, neither of which does it.  takeFault() throws VAXStop rather than pretending.
 * - COMPATIBILITY MODE (PSL<CM>).  A KA655 does not have it: vaxmod_defs.h does not define
 *   CMPM_VAX, so open-simh compiles the "Subset VAX" half of vax_cmode.c in which BadCmPSL()
 *   returns TRUE unconditionally and op_cmode() is a reserved-instruction fault.  badCmPSL() here
 *   is that same constant, which is why the REI PSL<CM> path always faults.
 * - DEVICE INTERRUPT VECTORS.  eval_int()'s hardware-interrupt scan and get_vector()'s device
 *   acknowledge are ported, but int_req[]/int_vec[] are owned by the Qbus/device item; `intReq`
 *   is present, always zero, and `deviceVector()` is the seam.  The memory-error and CRD-error
 *   interrupt levels (IPL 0x1D / 0x1A), which are CPU state rather than device state, ARE
 *   implemented and are graded.
 * - The CIS/octaword emulation traps (SCB_EMULATE / SCB_EMULFPD).
 */

import { VAXFault, VAXFAULT } from "./decode.js";
import MMUVAX from "./mmu.js";
import { OPCODES } from "./drom.js";
import { VAX } from "./defines.js";

const L_LONG = 4;
const nSP = 14, nPC = 15;

/* PSL fields, vax_defs.h:211-243.  PSL_CM is bit 31 and is therefore NEGATIVE as a JS int32;
   every use below is a bitwise test, which is safe (see defines.js rule on int32 bitwise ops). */
const PSL_V_CM = 31, PSL_CM = (1 << PSL_V_CM);
const PSL_V_TP = 30, PSL_TP = (1 << PSL_V_TP);
const PSL_V_FPD = 27, PSL_FPD = (1 << PSL_V_FPD);
const PSL_V_IS = 26, PSL_IS = (1 << PSL_V_IS);
const PSL_V_CUR = 24, PSL_V_PRV = 22, PSL_M_MODE = 0x3;
const PSL_CUR = PSL_M_MODE << PSL_V_CUR;
const PSL_PRV = PSL_M_MODE << PSL_V_PRV;
const PSL_V_IPL = 16, PSL_M_IPL = 0x1F;
const PSL_IPL = PSL_M_IPL << PSL_V_IPL;
const PSL_IPL1 = 0x01 << PSL_V_IPL;
const PSL_IPL17 = 0x17 << PSL_V_IPL;
const PSL_IPL1F = 0x1F << PSL_V_IPL;
const PSW_MBZ = 0xFF00;
const PSL_MBZ = (0x30200000 | PSW_MBZ);
const PSW_DV = 0x80, PSW_FU = 0x40, PSW_IV = 0x20, PSW_T = 0x10;
const CC_N = 0x8, CC_Z = 0x4, CC_V = 0x2, CC_C = 0x1, CC_MASK = 0xF;
const LSIGN = 0x80000000 | 0;

const KERN = 0, EXEC = 1, SUPV = 2, USER = 3;

/* Software interrupt summary register / AST level, vax_defs.h:245-254. */
const SISR_MASK = 0xFFFE;
const SISR_2 = 1 << 2;
const AST_MASK = 7, AST_MAX = 4;

/* Trap and interrupt request word (SIMH's `trpirq`), vax_defs.h:300-313. */
const TIR_V_IRQL = 0, TIR_V_TRAP = 5, TIR_M_TRAP = 0x7;
const TIR_TRAP = TIR_M_TRAP << TIR_V_TRAP;
const TRAP_INTOV = 1 << TIR_V_TRAP;
const TRAP_DIVZRO = 2 << TIR_V_TRAP;

/* SCB vector offsets, vax_defs.h:370-399. */
const SCB = {
    MCHK:       0x04,
    KSNV:       0x08,
    PWRFL:      0x0C,
    RESIN:      0x10,
    XFC:        0x14,
    RESOP:      0x18,
    RESAD:      0x1C,
    ACV:        0x20,
    TNV:        0x24,
    TP:         0x28,
    BPT:        0x2C,
    CMODE:      0x30,
    ARITH:      0x34,
    CHMK:       0x40,
    CHME:       0x44,
    CHMS:       0x48,
    CHMU:       0x4C,
    CRDERR:     0x54,
    MEMERR:     0x60,
    IPLSOFT:    0x80,
    INTTIM:     0xC0,
    EMULATE:    0xC8,
    EMULFPD:    0xCC,
    INTR:       0x100
};

/* Interrupt levels.  IPL_HMIN/IPL_HMAX are the Qbus hardware levels (vaxmod_defs.h:436-439);
   IPL_HLTPIN/IPL_MEMERR/IPL_CRDERR are CPU-internal (vax_defs.h:401-403). */
const IPL_HLTPIN = 0x1F, IPL_MEMERR = 0x1D, IPL_CRDERR = 0x1A;
const IPL_HMAX = 0x17, IPL_HMIN = 0x14, IPL_HLVL = IPL_HMAX - IPL_HMIN + 1;
const IPL_SMAX = 0xF;
const VEC_QBUS = 1;

/* intexc()'s `ei` argument, vax_defs.h:407-409. */
const IE = {SVE: -1, EXC: 0, INT: 1};

/* Memory-management fault parameters, vax_defs.h:577-582. */
const MM_WRITE = 4, MM_EMASK = 3;

/* Processor register numbers, vax_defs.h:586-620 plus vaxmod_defs.h:101-106. */
const MT = {
    KSP:     0,  ESP:     1,  SSP:     2,  USP:     3,  IS:      4,
    P0BR:    8,  P0LR:    9,  P1BR:   10,  P1LR:   11,  SBR:    12,  SLR:    13,
    PCBB:   16,  SCBB:   17,  IPL:    18,  ASTLVL: 19,  SIRR:   20,  SISR:   21,
    ICCS:   24,  NICR:   25,  ICR:    26,  TODR:   27,
    CSRS:   28,  CSRD:   29,  CSTS:   30,  CSTD:   31,
    RXCS:   32,  RXDB:   33,  TXCS:   34,  TXDB:   35,
    CADR:   37,  MSER:   39,  CONPC:  42,  CONPSL: 43,  IORESET: 55,
    MAPEN:  56,  TBIA:   57,  TBIS:   58,  PME:    61,  SID:    62,  TBCHK:  63
};
const MT_MAX = 63;                              // vaxmod_defs.h:106, last valid IPR on a KA655

/* The off-chip registers that belong to the SSC/CMCTL device model, not to this item.  See the
   file header; tests/excdiff.js excludes exactly this set from its randomized MTPR/MFPR pool. */
const IPR_DEVICE = [MT.ICCS, MT.NICR, MT.ICR, MT.TODR, MT.CSRS, MT.CSRD, MT.CSTS, MT.CSTD,
                    MT.RXCS, MT.RXDB, MT.TXCS, MT.TXDB, MT.CADR, MT.MSER, MT.IORESET];

/* Hardwired CVAX system identification, vaxmod_defs.h:84-85. */
const CVAX_SID = (10 << 24), CVAX_UREV = 6;

const BR_MASK = 0xFFFFFFFC | 0;

/**
 * @class VAXStop
 *
 * A SIMULATOR stop, not a VAX exception: the machine has reached a state a real VAX cannot
 * express (a bad SCB vector, an exception inside the exception flow already on the interrupt
 * stack, HALT), and SIMH's response is to return to SCP rather than to dispatch anything.  Kept
 * distinct from VAXFault so a caller can never mistake one for a vector to dispatch.
 */
class VAXStop extends Error {
    /**
     * @param {string} reason one of VAXStop.REASON.*
     * @param {number} [detail]
     */
    constructor(reason, detail = 0)
    {
        super("VAX simulator stop: " + reason);
        this.name = "VAXStop";
        this.reason = reason;
        this.detail = detail;
    }
}
VAXStop.REASON = {
    HALT:    "HALT instruction",                // STOP_HALT
    ILLVEC:  "illegal SCB vector flags",        // STOP_ILLVEC
    INIE:    "exception inside exception flow", // STOP_INIE
    CHMFI:   "CHMx on the interrupt stack",     // STOP_CHMFI
    UIPL:    "undefined interrupt level",       // STOP_UIPL
    CMODE:   "compatibility mode",              // no CMPM_VAX on a KA655
    MCHK:    "machine check (not modelled)",
    UNKABO:  "unknown abort code"        // STOP_UNKABO
};

/**
 * ccIIZZ_L(r) -- vax_defs.h:737.  N/Z from the result, V and C cleared.
 *
 * @param {number} r
 * @returns {number}
 */
function ccIIZZ_L(r) { return (r & LSIGN) ? CC_N : ((r | 0) === 0 ? CC_Z : 0); }

/**
 * badCmPSL(newpsl)
 *
 * vax_cmode.c:1304, the "Subset VAX" definition -- the one a KA655 compiles, because
 * vaxmod_defs.h does not define CMPM_VAX.  Compatibility mode is never legal on this processor,
 * so this is a constant.  It is written as a function anyway so the one caller reads exactly like
 * SIMH's, and so a full-VAX model can replace it without moving the call site.
 *
 * @param {number} newpsl
 * @returns {boolean}
 */
function badCmPSL(newpsl) { return true; }

/**
 * storeL(cpu, r)
 *
 * WRITE_L (vax_cpu.c:218) -- store a longword through the LAST specifier the decoder resolved,
 * which is the only destination form any instruction in this file has (MFPR's).  Same recipe as
 * cpu.js's storeL and control.js's store(); see decode.js's isMemoryDestination() docblock.
 *
 * @param {Object} cpu
 * @param {number} r
 */
function storeL(cpu, r)
{
    let d = cpu.decoder;
    r = r | 0;
    if (d.isMemoryDestination()) cpu.mmu.writeData(d.va, r, L_LONG, cpu.accW());
    else cpu.regs[d.rn] = r;
}

/**
 * @class VAXExc
 *
 * The privileged machine state, and the dispatch machinery over it.  One instance per CPU; the
 * CPU reaches it as `cpu.exc` and this module reaches the CPU back through the same interface
 * cpu.js/control.js already define:
 *
 *   regs        Int32Array(16)
 *   psl         full PSL, condition codes included (see the file header)
 *   mmu         MMUVAX -- readData/writeData/test for virtual, readLP/writeLP for physical, and
 *               the map-register mutators MTPR needs (setP0BR/setSLR/... /zapTB/zapTBEnt/chkTBEnt)
 *   decoder     VAXDecoder -- opnd[]/spec/rn/va, and unwind()/resetRecovery() for the fault path
 *   curMode()   PSL<25:24>
 *   accR()/accW()  MMU access masks for the current mode
 *   setPC(pc)
 *
 * @property {Int32Array} stk stk[0..3] = KSP/ESP/SSP/USP, stk[4] = IS (SIMH's STK[])
 */
class VAXExc {
    /**
     * @param {Object} [cpu]
     */
    constructor(cpu)
    {
        this.cpu = cpu || null;
        this.stk = new Int32Array(5);
        this.scbb = 0;
        this.pcbb = 0;
        this.astlvl = 0;
        this.sisr = 0;
        this.pme = 0;
        this.trpirq = 0;
        this.inIE = 0;
        this.memErr = 0;
        this.crdErr = 0;
        this.hltPin = 0;
        this.intReq = new Int32Array(IPL_HLVL);
        this.iprDevice = null;
        /*
         * fault_PC (vax_cpu.c:258): the PC of the instruction currently being executed, captured
         * before its opcode is fetched.  The abort handler restores PC to it so the exception
         * frame records the FAULTING instruction, not the middle of it -- which is what makes the
         * instruction restartable.  BPT and XFC read it for the same reason.
         */
        this.faultPC = 0;
    }

    /**
     * reset()
     *
     * @this {VAXExc}
     */
    reset()
    {
        this.stk.fill(0);
        this.scbb = this.pcbb = this.astlvl = this.sisr = this.pme = 0;
        this.trpirq = 0;
        this.inIE = 0;
        this.memErr = this.crdErr = this.hltPin = 0;
        this.intReq.fill(0);
        this.faultPC = 0;
    }

    /**
     * setIPRDevice(dev)
     *
     * Install the off-chip (SSC/CMCTL) IPR model.  `dev` must expose read(prn) and write(prn,val).
     *
     * @this {VAXExc}
     * @param {Object} dev
     */
    setIPRDevice(dev) { this.iprDevice = dev; }

    /* --------------------------------------------------------------------------------------- *
     * IPL arbitration                                                                           *
     * --------------------------------------------------------------------------------------- */

    /**
     * evalInt(cpu)
     *
     * vax_io.c:390, eval_int() -- "find highest priority outstanding interrupt".  Returns the IPL
     * of the interrupt to take, or 0 for none.  The order is the whole point and is NOT a simple
     * numeric maximum: the halt pin outranks everything unconditionally (it is not maskable by
     * IPL at all), memory and CRD errors are compared against the CURRENT IPL, the four Qbus
     * hardware levels are scanned downward and BAIL OUT the moment they reach the current IPL
     * (`if (i <= ipl) return 0` inside the loop, not a loop bound), and only then are the fifteen
     * software levels considered through a per-IPL eligibility mask.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @returns {number} 0, or an IPL in 1..0x1F
     */
    evalInt(cpu)
    {
        let ipl = (cpu.psl >>> PSL_V_IPL) & PSL_M_IPL;
        if (this.hltPin) return IPL_HLTPIN;
        if ((ipl < IPL_MEMERR) && this.memErr) return IPL_MEMERR;
        if ((ipl < IPL_CRDERR) && this.crdErr) return IPL_CRDERR;
        for (let i = IPL_HMAX; i >= IPL_HMIN; i--) {
            if (i <= ipl) return 0;
            if (this.intReq[i - IPL_HMIN]) return i;
        }
        if (ipl >= IPL_SMAX) return 0;
        let t = this.sisr & SW_INT_MASK[ipl];
        if (t === 0) return 0;
        for (let i = IPL_SMAX; i > ipl; i--) {
            if ((t >>> i) & 1) return i;
        }
        return 0;
    }

    /**
     * setIRQL(cpu)
     *
     * SET_IRQL (vax_defs.h:311): re-evaluate the interrupt request, PRESERVING any pending trap.
     * SIMH runs this on entry to sim_instr (so every `step`/`go` re-arbitrates from the current
     * PSL and SISR), after servicing the clock queue, after every trap/interrupt dispatch, and
     * after CHMx, REI and MTPR -- the three instructions that can change PSL<IPL> or SISR.  It is
     * deliberately NOT run after every instruction.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     */
    setIRQL(cpu) { this.trpirq = (this.trpirq & TIR_TRAP) | this.evalInt(cpu); }

    /**
     * setTrap(cpu, trap)
     *
     * SET_TRAP (vax_defs.h:310): request a deferred arithmetic trap.  The trap is taken at the TOP
     * of the NEXT instruction, never during the one that requested it -- which is why cpu.js and
     * fpa.js can set CC<V> and move on without knowing this module exists.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {number} trap one of TRAP_*
     */
    setTrap(cpu, trap) { this.trpirq = (this.trpirq & PSL_M_IPL) | trap; }

    /**
     * deviceVector(cpu, lvl)
     *
     * get_vector()'s device-acknowledge scan (vax_io.c:443-455).  int_req[]/int_vec[]/int_ack[]
     * belong to the Qbus and device item; until it lands there are no hardware interrupt sources,
     * `intReq` is always zero, and this returns 0 -- which the caller treats exactly as SIMH does
     * (`if (vec)`: a zero vector means the request evaporated and nothing is dispatched).
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {number} lvl
     * @returns {number}
     */
    deviceVector(cpu, lvl) { return 0; }

    /**
     * getVector(cpu, lvl)
     *
     * vax_io.c:428, get_vector().  The memory-error and CRD-error levels are CPU state, and
     * reading their vector CLEARS the request -- an edge-triggered acknowledge, not a level.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {number} lvl
     * @returns {number} SCB offset, or 0 if the request evaporated
     */
    getVector(cpu, lvl)
    {
        if (lvl === IPL_MEMERR) { this.memErr = 0; return SCB.MEMERR; }
        if (lvl === IPL_CRDERR) { this.crdErr = 0; return SCB.CRDERR; }
        if (lvl > IPL_HMAX) throw new VAXStop(VAXStop.REASON.UIPL, lvl);
        return this.deviceVector(cpu, lvl);
    }

    /* --------------------------------------------------------------------------------------- *
     * Exception and interrupt dispatch                                                          *
     * --------------------------------------------------------------------------------------- */

    /**
     * intexc(cpu, vec, ipl, ei)
     *
     * vax_cpu1.c:1094, intexc() -- the single chokepoint every exception and every interrupt goes
     * through.  Four things happen, in this order, and the order is observable:
     *
     *   1. The SCB longword at PHYSICAL address (SCBB + vec) & (PAMASK & ~3) is read.  Its low two
     *      bits are FLAGS, not address: <0> means "dispatch on the interrupt stack", <1> means
     *      "pass to a second-level handler", which SIMH does not implement and stops on.  A
     *      SEVERE exception (ei = IE.SVE) forces <0> on regardless of what the SCB says.
     *   2. The stack is selected.  Already on the interrupt stack -> stay, and do NOT write back
     *      stk[] (there is nothing to save: R[14] IS the interrupt stack).  Otherwise the current
     *      mode's stack pointer is saved to stk[cur] and R[14] is loaded from IS or KSP.  Note
     *      the K->K case: stk[0] is written and then immediately read back, so SP is unchanged --
     *      which is why 273 of EHKAA's 1,675 dispatches show no stack switch at all.
     *   3. The new PSL is built.  For an INTERRUPT it is (IS?) plus the new IPL and NOTHING else:
     *      current mode becomes kernel (0), previous mode becomes 0, the condition codes are
     *      cleared.  For an EXCEPTION the IPL is INHERITED (or forced to 1F when dispatching to
     *      the interrupt stack) and the old current mode is recorded as PSL<PRV>.
     *   4. The old PSL and the old PC are pushed, in that order, at SP-4 and SP-8, with KERNEL
     *      access -- the new mode's access, not the old one's.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {number} vec SCB offset
     * @param {number} ipl new IPL, meaningful only when ei is IE.INT
     * @param {number} ei IE.SVE, IE.EXC or IE.INT
     * @returns {number} the new condition codes (always 0)
     */
    intexc(cpu, vec, ipl, ei)
    {
        let oldpsl = cpu.psl;
        let oldcur = (oldpsl >>> PSL_V_CUR) & PSL_M_MODE;
        let oldsp = cpu.regs[nSP];
        let newpsl;

        this.inIE = 1;
        this.trpirq = this.trpirq & ~TIR_TRAP;                  // CLR_TRAPS
        let scbpa = (this.scbb + vec) & (VAX.PAMASK & ~3);
        this.scbPA = scbpa;                                     // diagnostic (tests/excdiff.js)
        let newpc = cpu.mmu.readLP(scbpa);
        if (ei === IE.SVE) newpc = newpc | 1;                   // severe? force onto the int stack
        if (newpc & 2) throw new VAXStop(VAXStop.REASON.ILLVEC, newpc);
        if (oldpsl & PSL_IS) {
            newpsl = PSL_IS;
        } else {
            this.stk[oldcur] = oldsp;                           // save the stack we are leaving
            if (newpc & 1) {
                newpsl = PSL_IS;
                cpu.regs[nSP] = this.stk[4];                    // IS
            } else {
                newpsl = 0;
                cpu.regs[nSP] = this.stk[KERN];                 // KSP
            }
        }
        /*
         * THE CONDITION CODES SURVIVE UNTIL THE DISPATCH COMMITS.  SIMH's `cc` is a LOCAL in
         * sim_instr, and `cc = intexc (...)` only assigns when intexc RETURNS -- so if one of the
         * two pushes below faults (a kernel stack whose page is not writable: the KSNV path), the
         * longjmp leaves the caller's `cc` holding the PRE-dispatch condition codes while PSL
         * already holds the new value.  The nested KSNV dispatch then pushes `PSL | cc`, i.e. the
         * new PSL carrying the OLD condition codes.
         *
         * With the condition codes living inside psl, reproducing that means NOT clearing them
         * here: they are cleared on the way out, after the pushes, which is the moment SIMH's
         * assignment happens.  Writing `cpu.psl = newpsl | ...` directly instead was wrong in
         * exactly one place, and only one: every KSNV frame in the MAPPED differential pushed
         * PSL with cc = 0 where SIMH pushed the faulting instruction's condition codes.
         */
        let keepCC = cpu.psl & CC_MASK;
        if (ei > 0) {                                           /* interrupt: brand new IPL */
            let newipl = (VEC_QBUS & vec) ? PSL_IPL17 : (ipl << PSL_V_IPL);
            cpu.psl = newpsl | newipl | keepCC;
        } else {                                                /* exception: inherit IPL */
            cpu.psl = newpsl |
                ((newpc & 1) ? PSL_IPL1F : (oldpsl & PSL_IPL)) | (oldcur << PSL_V_PRV) | keepCC;
        }
        let acc = MMUVAX.accWrite(KERN);                        // new mode is kernel
        let sp = cpu.regs[nSP];
        cpu.mmu.writeData((sp - 4) | 0, oldpsl, L_LONG, acc);   // push old PSL
        cpu.mmu.writeData((sp - 8) | 0, cpu.regs[nPC], L_LONG, acc);    // push old PC
        cpu.regs[nSP] = (sp - 8) | 0;
        cpu.setPC(newpc & ~3);
        cpu.psl = cpu.psl & ~CC_MASK;                           // `cc = intexc (...)` commits here
        this.inIE = 0;
        return 0;
    }

    /**
     * takeFault(cpu, fault)
     *
     * vax_cpu.c:530-604, the `abortval < 0` half of the setjmp handler.  Three responsibilities
     * that must happen in this order:
     *
     *   1. UNDO the addressing modes' register side effects, unless PSL<FPD> says the instruction
     *      has already made externally visible progress and will be RESUMED rather than restarted.
     *      decode.js records them; see its header and machines/dec/vax/README.md.
     *   2. Clear PSL<TP> and restore PC to fault_PC, so the frame records the faulting
     *      instruction's own address.
     *   3. Dispatch, with the per-category parameter pushes.  ARITH and CMODE push one parameter
     *      after the frame; ACV and TNV push two (p1 = the MM_PARAM access descriptor, p2 = the
     *      faulting virtual address) -- and if the fault happened while ALREADY inside the
     *      exception flow, they do not push anything, they become KSNV instead.
     *
     * The parameter pushes are done with inIE set precisely so that a fault ON THE PUSH is caught
     * by the KSNV rule rather than recursing forever; the caller (stepInstruction) re-enters here
     * when that happens, which is what SIMH's longjmp-to-the-top-of-sim_instr does implicitly.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {VAXFault} fault
     */
    takeFault(cpu, fault)
    {
        let vec = -fault.code;
        if (!(cpu.psl & PSL_FPD)) cpu.decoder.unwind();
        cpu.psl = cpu.psl & ~PSL_TP;
        cpu.decoder.resetRecovery();
        cpu.setPC(this.faultPC);
        switch (vec) {

        case SCB.RESIN:
        case SCB.RESAD:
        case SCB.RESOP:
            if (this.inIE) throw new VAXStop(VAXStop.REASON.INIE, vec);
            this.intexc(cpu, vec, 0, IE.EXC);
            break;

        case SCB.CMODE:
        case SCB.ARITH:
            if (this.inIE) throw new VAXStop(VAXStop.REASON.INIE, vec);
            this.intexc(cpu, vec, 0, IE.EXC);
            this.inIE = 1;
            this.pushParams(cpu, [fault.p1]);
            this.inIE = 0;
            break;

        case SCB.ACV:
        case SCB.TNV:
            if (this.inIE) {
                if (cpu.psl & PSL_IS) throw new VAXStop(VAXStop.REASON.INIE, vec);
                this.intexc(cpu, SCB.KSNV, 0, IE.SVE);
            } else {
                this.intexc(cpu, vec, 0, IE.EXC);
                this.inIE = 1;
                this.pushParams(cpu, [fault.p1, fault.p2]);
                this.inIE = 0;
            }
            break;

        case SCB.MCHK:
            throw new VAXStop(VAXStop.REASON.MCHK, fault.p1);

        default:
            /*
             * STOP_UNKABO.  Reached for any abort code that is not an SCB offset at all -- most
             * usefully VAXFAULT.PPTE, mmu.js's deliberately POSITIVE code for a process page table
             * outside system space, which is an impossible machine state rather than a fault.
             */
            throw new VAXStop(VAXStop.REASON.UNKABO, fault.code);
        }
    }

    /**
     * pushParams(cpu, params)
     *
     * The exception-parameter push that follows intexc() for the fault categories that have one.
     * SIMH writes them at fixed offsets below the CURRENT SP and then subtracts once, so the
     * FIRST parameter ends up DEEPEST (p1 at SP-8, p2 at SP-4 for a two-parameter fault) -- i.e.
     * exactly the order the SRM's exception frame diagram shows, and the reverse of the order a
     * naive "push each in turn" loop would produce.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {Array.<number>} params
     */
    pushParams(cpu, params)
    {
        let acc = cpu.accW();                       // PSL<cur> is kernel now: GET_CUR after intexc
        let n = params.length;
        let sp = cpu.regs[nSP];
        for (let i = 0; i < n; i++) {
            cpu.mmu.writeData((sp - (n - i) * 4) | 0, params[i], L_LONG, acc);
        }
        cpu.regs[nSP] = (sp - n * 4) | 0;
    }

    /**
     * stepInstruction(cpu, execute)
     *
     * One `step 1` worth of machine: SIMH's main loop from the top through exactly one instruction
     * fetch, including every non-instruction dispatch that precedes it.  Structured to match
     * vax_cpu.c:666-720 literally, because the STEP ACCOUNTING is the subtle part -- a trap, an
     * interrupt or a trace trap `continue`s WITHOUT decrementing sim_interval, so one `step 1` can
     * take several dispatches and then still execute an instruction.  A comparison harness that
     * assumed one step == one dispatch would silently drift.
     *
     * Order (SRM order, and SIMH's):
     *   1. Pending TRAP (arithmetic) -- outranks any interrupt.
     *   2. Pending INTERRUPT, at the level SET_IRQL arbitrated.
     *   3. PSL<TP> -- the trace trap armed by the PREVIOUS instruction.
     *   4. Otherwise arm PSL<TP> from PSL<T> and execute one instruction.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {function(number, Object, Object)} execute called as (opc, decoder, cpu)
     * @returns {number} the number of non-instruction dispatches taken before the instruction
     */
    stepInstruction(cpu, execute)
    {
        this.setIRQL(cpu);                          // sim_instr entry re-arbitrates
        let dispatches = 0;
        for (;;) {
            if (dispatches > 64) throw new VAXStop(VAXStop.REASON.INIE, dispatches);
            if (this.trpirq) {
                let trap = (this.trpirq >>> TIR_V_TRAP) & TIR_M_TRAP;
                if (trap) {
                    this.intexc(cpu, SCB.ARITH, 0, IE.EXC);
                    this.inIE = 1;
                    this.pushParams(cpu, [trap]);
                    this.inIE = 0;
                } else {
                    let lvl = (this.trpirq >>> TIR_V_IRQL) & PSL_M_IPL;
                    if (lvl) {
                        let vec;
                        if (lvl === IPL_HLTPIN) {
                            /* con_halt() enters the console firmware -- ROM code, and the boot ROM
                               is not part of this item.  SIMH's own `set cpu simhalt` turns the
                               equivalent HALT case into a simulator stop; do the same here. */
                            this.hltPin = 0;
                            this.trpirq = 0;
                            throw new VAXStop(VAXStop.REASON.HALT, IPL_HLTPIN);
                        } else if (lvl >= IPL_HMIN) {
                            vec = this.getVector(cpu, lvl);
                        } else if (lvl > IPL_SMAX) {
                            throw new VAXStop(VAXStop.REASON.UIPL, lvl);
                        } else {
                            vec = SCB.IPLSOFT + (lvl << 2);
                            this.sisr = this.sisr & ~(1 << lvl);
                        }
                        if (vec) this.intexc(cpu, vec, lvl, IE.INT);
                    } else {
                        this.trpirq = 0;
                    }
                }
                this.setIRQL(cpu);
                dispatches++;
                continue;
            }
            if (cpu.psl & (PSL_CM | PSL_TP | PSW_T)) {
                if (cpu.psl & PSL_TP) {
                    cpu.psl = cpu.psl & ~PSL_TP;
                    this.intexc(cpu, SCB.TP, 0, IE.EXC);
                    dispatches++;
                    continue;
                }
                if (cpu.psl & PSW_T) cpu.psl = cpu.psl | PSL_TP;
                if (cpu.psl & PSL_CM) throw new VAXStop(VAXStop.REASON.CMODE);
            }
            break;
        }

        this.faultPC = cpu.regs[nPC];
        let pending = null;
        try {
            let opc = cpu.decoder.decode((cpu.psl & PSL_FPD) != 0);
            execute(opc, cpu.decoder, cpu);
        } catch (e) {
            if (!(e instanceof VAXFault)) throw e;
            pending = e;
        }
        /*
         * A fault raised BY the fault dispatch is a fresh dispatch, not a lost one: SIMH's longjmp
         * lands back at the same setjmp with in_ie still set, which is what turns the second one
         * into KSNV and the third into a halt.  Three is therefore the real bound (fault, KSNV,
         * stop); the guard is generous and throws rather than looping.
         */
        for (let depth = 0; pending; depth++) {
            if (depth > 3) throw new VAXStop(VAXStop.REASON.INIE, -pending.code);
            let f = pending;
            pending = null;
            try {
                this.takeFault(cpu, f);
            } catch (e) {
                if (!(e instanceof VAXFault)) throw e;
                pending = e;
            }
        }
        return dispatches;
    }

    /* --------------------------------------------------------------------------------------- *
     * Instruction bodies that live entirely in this machinery                                   *
     * --------------------------------------------------------------------------------------- */

    /**
     * rei(cpu)
     *
     * vax_cpu1.c:1210, op_rei().  Nine validity rules on the popped PSL, and they are NOT a
     * uniform list: rules 2, 4 and 7 are skipped when returning to a non-kernel mode, and rules 3,
     * 5 and 6 are skipped when returning to kernel.  SIMH's comment block above op_rei states all
     * nine in SRM form; the grouping below is its code, which is the authority.
     *
     * The tail is where a port goes wrong.  After the rules pass, the stack pointer is popped and
     * saved to the stack it is LEAVING (IS if PSL<IS>, else stk[old current mode]), the new PSL is
     * installed, and only THEN is R[14] loaded from the stack the new PSL selects.  Two different
     * stk[] entries are touched, in that order, and the AST check that can raise a software
     * interrupt reads the NEW mode.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @returns {number} the new condition codes
     */
    rei(cpu)
    {
        let acc = cpu.accR();
        let sp = cpu.regs[nSP];
        let newpc = cpu.mmu.readData(sp, L_LONG, acc);
        let newpsl = cpu.mmu.readData((sp + 4) | 0, L_LONG, acc);
        let newcur = (newpsl >>> PSL_V_CUR) & PSL_M_MODE;
        let oldcur = (cpu.psl >>> PSL_V_CUR) & PSL_M_MODE;

        if ((newpsl & PSL_MBZ) ||                               /* rule 8 */
            (newcur < oldcur)) {                                /* rule 1 */
            throw new VAXFault(VAXFAULT.RESOP);
        }
        if (newcur) {                                           /* to E/S/U: skip 2, 4, 7 */
            if ((newpsl & (PSL_IS | PSL_IPL)) ||                /* rules 3, 5 */
                (newcur > ((newpsl >>> PSL_V_PRV) & PSL_M_MODE))) {     /* rule 6 */
                throw new VAXFault(VAXFAULT.RESOP);
            }
        } else {                                                /* to K: skip 3, 5, 6 */
            let newipl = (newpsl >>> PSL_V_IPL) & PSL_M_IPL;
            if ((newpsl & PSL_IS) &&                            /* setting IS? */
                (((cpu.psl & PSL_IS) === 0) || (newipl === 0))) {        /* rules 2, 4 */
                throw new VAXFault(VAXFAULT.RESOP);
            }
            if (newipl > ((cpu.psl >>> PSL_V_IPL) & PSL_M_IPL)) {       /* rule 7 */
                throw new VAXFault(VAXFAULT.RESOP);
            }
        }
        if (newpsl & PSL_CM) {
            /*
             * Rule 9, compatibility mode.  badCmPSL() is a CONSTANT TRUE on a KA655 (see the file
             * header), so this always faults and the R0-R6/PC word-masking SIMH does next is
             * unreachable on this processor -- it is deliberately not transcribed, because dead
             * code that looks live is worse than an explained absence.
             */
            if (badCmPSL(newpsl)) throw new VAXFault(VAXFAULT.RESOP);
        }
        cpu.regs[nSP] = (sp + 8) | 0;                           /* pop the frame */
        if (cpu.psl & PSL_IS) this.stk[4] = cpu.regs[nSP];      /* save the stack being left */
        else this.stk[oldcur] = cpu.regs[nSP];
        /*
         * SIMH: PSL = (PSL & PSL_TP) | (newpsl & ~CC_MASK), and the caller then ORs in the
         * returned newpsl & CC_MASK.  With the condition codes living in psl the two collapse.
         */
        cpu.psl = (cpu.psl & PSL_TP) | newpsl;
        if (cpu.psl & PSL_IS) {
            cpu.regs[nSP] = this.stk[4];
        } else {
            cpu.regs[nSP] = this.stk[newcur];
            if (newcur >= this.astlvl) this.sisr = this.sisr | SISR_2;  /* AST delivered */
        }
        cpu.setPC(newpc);
        return newpsl & CC_MASK;
    }

    /**
     * chm(cpu, opc, op0)
     *
     * vax_cpu1.c:1151, op_chm() -- CHMK/CHME/CHMS/CHMU.  A change-mode is NOT an exception: it
     * does not go through intexc(), it does not touch the IPL, and it builds a THREE-longword
     * frame (argument, PC, PSL) rather than the two-longword one an exception pushes.  That is
     * why docs/reference/ehkaa-profile.md §5 lists no vector 0x40 even though EHKAA executes 73
     * CHMKs, and it is part of why 2,967 REIs return from 1,675 dispatches.
     *
     * `mode` is the LOW TWO BITS OF THE OPCODE (CHMK=0xBC -> kernel, ... CHMU=0xBF -> user), and
     * a change-mode can only ever go INWARD: a request for a less privileged mode than the
     * current one is silently clamped to the current one.
     *
     * Both stack probes use the NEW mode's write access and are done BEFORE any store, so a CHMx
     * onto an inaccessible target stack faults without having half-built a frame.  The far end
     * (tsp-12) is probed second, matching SIMH, so when both ends are bad the reported fault is
     * the one for tsp-1.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {number} opc
     * @param {number} op0 the change-mode argument (opnd[0])
     * @returns {number} the new condition codes (always 0)
     */
    chm(cpu, opc, op0)
    {
        let mode = opc & PSL_M_MODE;
        let cur = (cpu.psl >>> PSL_V_CUR) & PSL_M_MODE;

        if (cpu.psl & PSL_IS) throw new VAXStop(VAXStop.REASON.CHMFI);
        let newpc = cpu.mmu.readLP((this.scbb + SCB.CHMK + (mode << 2)) & VAX.PAMASK);
        if (cur < mode) mode = cur;                             /* only inward */
        this.stk[cur] = cpu.regs[nSP];                          /* save current stack */
        let tsp = this.stk[mode];                               /* the stack we are moving to */
        let acc = MMUVAX.accWrite(mode);
        let sta = {code: 0};
        if (cpu.mmu.test((tsp - 1) | 0, acc, sta) < 0) {
            throw new VAXFault((sta.code & 4) ? VAXFAULT.TNV : VAXFAULT.ACV,
                               MM_WRITE | (sta.code & MM_EMASK), (tsp - 1) | 0);
        }
        sta.code = 0;
        if (cpu.mmu.test((tsp - 12) | 0, acc, sta) < 0) {
            throw new VAXFault((sta.code & 4) ? VAXFAULT.TNV : VAXFAULT.ACV,
                               MM_WRITE | (sta.code & MM_EMASK), (tsp - 12) | 0);
        }
        /* SXTW: the argument is a word operand, sign-extended into the pushed longword. */
        let arg = (op0 & 0x8000) ? (op0 | ~0xFFFF) : (op0 & 0xFFFF);
        cpu.mmu.writeData((tsp - 12) | 0, arg, L_LONG, acc);
        cpu.mmu.writeData((tsp - 8) | 0, cpu.regs[nPC], L_LONG, acc);
        cpu.mmu.writeData((tsp - 4) | 0, cpu.psl, L_LONG, acc);
        cpu.regs[nSP] = (tsp - 12) | 0;
        cpu.psl = (mode << PSL_V_CUR) | (cpu.psl & PSL_IPL) | (cur << PSL_V_PRV);
        cpu.setPC(newpc & ~3);
        return 0;
    }

    /**
     * ldpctx(cpu)
     *
     * vax_cpu1.c:1265, op_ldpctx().  Reloads the entire process context from the PCB at PHYSICAL
     * address PCBB: four stack pointers, R0-R13, the P0/P1 map registers, ASTLVL and PME -- and
     * then pushes the PCB's saved PC/PSL onto the (new) kernel stack so that an REI resumes the
     * process.  It does NOT set PC or PSL directly.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     */
    ldpctx(cpu)
    {
        if (cpu.psl & PSL_CUR) throw new VAXFault(VAXFAULT.RESIN);      // must be kernel
        let pcbpa = this.pcbb & VAX.PAMASK;
        let mmu = cpu.mmu;
        this.stk[KERN] = mmu.readLP(pcbpa);
        this.stk[EXEC] = mmu.readLP((pcbpa + 4) & VAX.PAMASK);
        this.stk[SUPV] = mmu.readLP((pcbpa + 8) & VAX.PAMASK);
        this.stk[USER] = mmu.readLP((pcbpa + 12) & VAX.PAMASK);
        for (let i = 0; i <= 13; i++) {
            cpu.regs[i] = mmu.readLP((pcbpa + 16 + i * 4) & VAX.PAMASK);
        }
        let newpc = mmu.readLP((pcbpa + 72) & VAX.PAMASK);
        let newpsl = mmu.readLP((pcbpa + 76) & VAX.PAMASK);

        let t = mmu.readLP((pcbpa + 80) & VAX.PAMASK);
        mmu.setP0BR(t);                                         // ML_PXBR_TEST is a NOP on a KA655
        t = mmu.readLP((pcbpa + 84) & VAX.PAMASK);
        mmu.setP0LR(t & MMUVAX.LR_MASK);
        let ast = (t >>> 24) & AST_MASK;                        // LP_AST_TEST is a NOP on a KA655
        this.astlvl = ast;
        t = mmu.readLP((pcbpa + 88) & VAX.PAMASK);
        mmu.setP1BR(t);
        t = mmu.readLP((pcbpa + 92) & VAX.PAMASK);
        mmu.setP1LR(t & MMUVAX.LR_MASK);
        this.pme = (t >>> 31) & 1;

        /*
         * SIMH calls zap_tb(0) + set_map_reg() once here; the mmu.setPxBR/setPxLR mutators above
         * each already do exactly that (see mmu.js), so the process TB is flushed and the derived
         * map registers rebuilt by the time we get here.  Doing it again would be a no-op.
         */
        if (cpu.psl & PSL_IS) this.stk[4] = cpu.regs[nSP];      // if on istk, save it
        cpu.psl = cpu.psl & ~PSL_IS;                            // switch to the kernel stack
        let sp = (this.stk[KERN] - 8) | 0;
        cpu.regs[nSP] = sp;
        let acc = cpu.accW();
        mmu.writeData(sp, newpc, L_LONG, acc);
        mmu.writeData((sp + 4) | 0, newpsl, L_LONG, acc);
    }

    /**
     * svpctx(cpu)
     *
     * vax_cpu1.c:1327, op_svpctx().  The mirror image: pop the PC/PSL an exception left on the
     * kernel stack, switch to the interrupt stack (forcing IPL to at least 1 if it was 0, because
     * PSL<IS> with IPL 0 is an illegal combination REI would reject), and write the whole context
     * back to the PCB.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     */
    svpctx(cpu)
    {
        if (cpu.psl & PSL_CUR) throw new VAXFault(VAXFAULT.RESIN);      // must be kernel
        let mmu = cpu.mmu;
        let acc = cpu.accR();
        let sp = cpu.regs[nSP];
        let savpc = mmu.readData(sp, L_LONG, acc);
        let savpsl = mmu.readData((sp + 4) | 0, L_LONG, acc);
        if (cpu.psl & PSL_IS) {
            cpu.regs[nSP] = (sp + 8) | 0;
        } else {
            this.stk[KERN] = (sp + 8) | 0;                      // pop the kernel stack
            cpu.regs[nSP] = this.stk[4];                        // switch to the interrupt stack
            if ((cpu.psl & PSL_IPL) === 0) cpu.psl = cpu.psl | PSL_IPL1;
            cpu.psl = cpu.psl | PSL_IS;
        }
        let pcbpa = this.pcbb & VAX.PAMASK;
        mmu.writeLP(pcbpa, this.stk[KERN]);
        mmu.writeLP((pcbpa + 4) & VAX.PAMASK, this.stk[EXEC]);
        mmu.writeLP((pcbpa + 8) & VAX.PAMASK, this.stk[SUPV]);
        mmu.writeLP((pcbpa + 12) & VAX.PAMASK, this.stk[USER]);
        for (let i = 0; i <= 13; i++) {
            mmu.writeLP((pcbpa + 16 + i * 4) & VAX.PAMASK, cpu.regs[i]);
        }
        mmu.writeLP((pcbpa + 72) & VAX.PAMASK, savpc);
        mmu.writeLP((pcbpa + 76) & VAX.PAMASK, savpsl);
    }

    /* --------------------------------------------------------------------------------------- *
     * MTPR / MFPR                                                                               *
     * --------------------------------------------------------------------------------------- */

    /**
     * mtpr(cpu, val, prn)
     *
     * vax_cpu1.c:1427, op_mtpr().  Returns the condition codes: N/Z from the VALUE WRITTEN (not
     * from anything the register did with it), plus CC<V> from TBCHK.  The caller re-adds the
     * incoming CC<C>.
     *
     * `prn` is the RESOLVED register number, an ordinary longword operand -- MTPR's second
     * specifier is `RL`, so it can be, and in EHKAA sometimes is, computed in a register at
     * runtime rather than written as a literal.  Nothing here may assume a constant.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {number} val
     * @param {number} prn
     * @returns {number} condition codes
     */
    mtpr(cpu, val, prn)
    {
        let mmu = cpu.mmu;
        if (cpu.psl & PSL_CUR) throw new VAXFault(VAXFAULT.RESIN);      // must be kernel
        if ((prn >>> 0) > MT_MAX) throw new VAXFault(VAXFAULT.RESOP);
        prn = prn >>> 0;
        let cc = ccIIZZ_L(val | 0);
        switch (prn) {

        case MT.KSP:
            if (cpu.psl & PSL_IS) this.stk[KERN] = val;         // on the int stack: store KSP
            else cpu.regs[nSP] = val;                           // else store the live SP
            break;

        case MT.ESP: case MT.SSP: case MT.USP:
            this.stk[prn] = val;
            break;

        case MT.IS:
            if (cpu.psl & PSL_IS) cpu.regs[nSP] = val;          // on the int stack: it IS the SP
            else this.stk[4] = val;
            break;

        case MT.P0BR: mmu.setP0BR(val); break;                  // each mutator flushes exactly
        case MT.P0LR: mmu.setP0LR(val & MMUVAX.LR_MASK); break; // what SIMH's MTPR flushes --
        case MT.P1BR: mmu.setP1BR(val); break;                  // process TB for P0/P1, the whole
        case MT.P1LR: mmu.setP1LR(val & MMUVAX.LR_MASK); break; // TB for the system registers
        case MT.SBR:  mmu.setSBR(val); break;
        case MT.SLR:  mmu.setSLR(val & MMUVAX.LR_MASK); break;

        case MT.SCBB: this.scbb = val & BR_MASK; break;         // ML_PA_TEST is a NOP on a KA655
        case MT.PCBB: this.pcbb = val & BR_MASK; break;

        case MT.IPL:
            cpu.psl = (cpu.psl & ~PSL_IPL) | ((val & PSL_M_IPL) << PSL_V_IPL);
            break;

        case MT.ASTLVL:
            if ((val >>> 0) > AST_MAX) throw new VAXFault(VAXFAULT.RESOP);      // MT_AST_TEST
            this.astlvl = val;
            break;

        case MT.SIRR:
            /*
             * SIRR is the software-interrupt REQUEST register: writing level n sets bit n of
             * SISR, and writing 0 requests nothing at all (there is no software level 0).  Only
             * the low four bits are considered -- MTPR #17,SIRR requests level 1, not a fault.
             */
            val = val & 0xF;
            if (val !== 0) this.sisr = this.sisr | (1 << val);
            break;

        case MT.SISR:
            this.sisr = val & SISR_MASK;
            break;

        case MT.MAPEN:
            /*
             * SIMH FALLS THROUGH from MAPEN into TBIA, and it must: turning mapping on or off
             * invalidates every translation, not just the process ones.  mmu.setMAPEN() does both
             * halves (mapen = val & 1, then zap_tb(1)), which is why there is no fallthrough here.
             */
            mmu.setMAPEN(val);
            break;

        case MT.TBIA:
            mmu.zapTB(1);
            break;

        case MT.TBIS:
            mmu.zapTBEnt(val);
            break;

        case MT.TBCHK:
            if (mmu.chkTBEnt(val)) cc = cc | CC_V;
            break;

        case MT.PME:
            this.pme = val & 1;
            break;

        default:
            this.writeIPR(prn, val);
            break;
        }
        return cc;
    }

    /**
     * mfpr(cpu, prn)
     *
     * vax_cpu1.c:1559, op_mfpr().  The four write-only registers (SIRR, TBIA, TBIS, TBCHK) are a
     * reserved-operand FAULT to read, not a zero.
     *
     * @this {VAXExc}
     * @param {Object} cpu
     * @param {number} prn
     * @returns {number} the register's value
     */
    mfpr(cpu, prn)
    {
        let mmu = cpu.mmu;
        if (cpu.psl & PSL_CUR) throw new VAXFault(VAXFAULT.RESIN);      // must be kernel
        if ((prn >>> 0) > MT_MAX) throw new VAXFault(VAXFAULT.RESOP);
        prn = prn >>> 0;
        switch (prn) {

        case MT.KSP:  return (cpu.psl & PSL_IS) ? this.stk[KERN] : cpu.regs[nSP];
        case MT.ESP: case MT.SSP: case MT.USP:  return this.stk[prn];
        case MT.IS:   return (cpu.psl & PSL_IS) ? cpu.regs[nSP] : this.stk[4];

        case MT.P0BR: return mmu.p0br;
        case MT.P0LR: return mmu.p0lr;
        case MT.P1BR: return mmu.p1br;
        case MT.P1LR: return mmu.p1lr;
        case MT.SBR:  return mmu.sbr;
        case MT.SLR:  return mmu.slr;

        case MT.SCBB: return this.scbb;
        case MT.PCBB: return this.pcbb;

        case MT.IPL:  return (cpu.psl >>> PSL_V_IPL) & PSL_M_IPL;
        case MT.ASTLVL: return this.astlvl;
        case MT.SISR: return this.sisr & SISR_MASK;
        case MT.MAPEN: return mmu.mapen & 1;
        case MT.PME:  return this.pme & 1;

        case MT.SIRR:
        case MT.TBIA:
        case MT.TBIS:
        case MT.TBCHK:
            throw new VAXFault(VAXFAULT.RESOP);                 // write only

        default:
            return this.readIPR(prn);
        }
    }

    /**
     * readIPR(prn)
     *
     * vax_sysdev.c:845, ReadIPR() -- the off-chip half.  See the file header for what is and is
     * not modelled.
     *
     * @this {VAXExc}
     * @param {number} prn
     * @returns {number}
     */
    readIPR(prn)
    {
        if (prn === MT.SID) return (CVAX_SID | CVAX_UREV) | 0;
        if (this.iprDevice) return this.iprDevice.read(prn) | 0;
        return 0;                                               // SSC default: BTO, reads 0
    }

    /**
     * writeIPR(prn, val)
     *
     * vax_sysdev.c:921, WriteIPR() -- the off-chip half.
     *
     * @this {VAXExc}
     * @param {number} prn
     * @param {number} val
     */
    writeIPR(prn, val)
    {
        if (prn === MT.SID || prn === MT.CONPC || prn === MT.CONPSL) {
            throw new VAXFault(VAXFAULT.RESOP);                 // read-only / halt registers
        }
        if (this.iprDevice) { this.iprDevice.write(prn, val); return; }
        /* SSC default: sets the bus-timeout bit and drops the write. */
    }
}

/*
 * sw_int_mask[] (vax_io.c:394-400): for a current IPL of `n`, which SISR bits are ELIGIBLE.
 * Computed rather than transcribed -- entry n is "all bits above n", 0xFFFF << (n+1) truncated to
 * 16 bits -- and then checked against SIMH's literal table, because a table this regular is
 * exactly the kind a transcription typo survives in.
 */
const SW_INT_MASK = (function() {
    let a = new Int32Array(IPL_SMAX);
    for (let n = 0; n < IPL_SMAX; n++) a[n] = (0xFFFF << (n + 1)) & 0xFFFF;
    const SIMH = [0xFFFE, 0xFFFC, 0xFFF8, 0xFFF0, 0xFFE0, 0xFFC0, 0xFF80, 0xFF00,
                  0xFE00, 0xFC00, 0xF800, 0xF000, 0xE000, 0xC000, 0x8000];
    for (let n = 0; n < IPL_SMAX; n++) {
        if (a[n] !== SIMH[n]) throw new Error(`exc.js: sw_int_mask[${n}] is ${a[n]}, SIMH says ${SIMH[n]}`);
    }
    return a;
})();

/* ------------------------------------------------------------------------------------------- *
 * Opcode bodies.  Same shape as control.js's: keyed by mnemonic, resolved to numeric opcodes     *
 * through drom.js's OPCODES so the numbers are never transcribed by hand.                        *
 * ------------------------------------------------------------------------------------------- */

const BODIES = {

    /*
     * MTPR/MFPR condition codes (vax_cpu.c:3122-3138).  Both are documented as "N and Z from the
     * value, V cleared, C unchanged", and SIMH implements that by stashing the incoming C, letting
     * op_mtpr/CC_IIZZ_L build a fresh cc, and ORing C back.  MTPR's TBCHK is the one case that
     * puts V back on afterwards.
     */
    MTPR(decoder, cpu) {
        let keep = cpu.psl & CC_C;
        let cc = cpu.exc.mtpr(cpu, decoder.opnd[0], decoder.opnd[1]);
        cpu.psl = (cpu.psl & ~CC_MASK) | ((cc | keep) & CC_MASK);
        cpu.exc.setIRQL(cpu);
    },

    MFPR(decoder, cpu) {
        let keep = cpu.psl & CC_C;
        let r = cpu.exc.mfpr(cpu, decoder.opnd[0]) | 0;
        storeL(cpu, r);
        cpu.psl = (cpu.psl & ~CC_MASK) | ((ccIIZZ_L(r) | keep) & CC_MASK);
    },

    REI(decoder, cpu) {
        let cc = cpu.exc.rei(cpu);
        cpu.psl = (cpu.psl & ~CC_MASK) | (cc & CC_MASK);
        cpu.exc.setIRQL(cpu);
    },

    CHMK(decoder, cpu) { chmBody(decoder, cpu); },
    CHME(decoder, cpu) { chmBody(decoder, cpu); },
    CHMS(decoder, cpu) { chmBody(decoder, cpu); },
    CHMU(decoder, cpu) { chmBody(decoder, cpu); },

    LDPCTX(decoder, cpu) { cpu.exc.ldpctx(cpu); },
    SVPCTX(decoder, cpu) { cpu.exc.svpctx(cpu); },

    /*
     * BPT and XFC both rewind PC to the faulting instruction before dispatching, because the SRM
     * defines them as TRAPS reported at the instruction's own address -- the pushed PC must point
     * AT the BPT, not past it, or a debugger cannot resume.  Clearing PSL<TP> matters for the same
     * reason the abort handler clears it: the trace trap this instruction armed must not also fire
     * on top of the frame it is about to push.
     */
    BPT(decoder, cpu) { trapBody(cpu, SCB.BPT); },

    XFC(decoder, cpu) { trapBody(cpu, SCB.XFC); },

    /*
     * HALT is a reserved-INSTRUCTION fault outside kernel mode (it is opcode 0x00, so this is also
     * what an accidental branch into zeroed memory does).  In kernel it stops the machine; SIMH
     * can instead enter the console ROM (`set cpu conhalt`), which is boot-ROM territory and not
     * modelled -- see stepInstruction()'s halt-pin note.
     */
    HALT(decoder, cpu) {
        if (cpu.psl & PSL_CUR) throw new VAXFault(VAXFAULT.RESIN);
        throw new VAXStop(VAXStop.REASON.HALT);
    },

    /* NOP has no owner elsewhere and no body worth a file of its own. */
    NOP(decoder, cpu) {}
};

/**
 * trapBody(cpu, vec)
 *
 * WRITE THIS AS TWO STATEMENTS.  `cpu.psl = (cpu.psl & ~CC_MASK) | cpu.exc.intexc(...)` reads
 * cpu.psl for the left operand BEFORE the call runs, so it restores the PRE-dispatch PSL over the
 * one intexc() just installed -- a BPT taken in user mode came back with PSL<CUR> still 3.  The
 * differential caught it; the shape is preserved here so it cannot come back.
 *
 * @param {Object} cpu
 * @param {number} vec
 */
function trapBody(cpu, vec)
{
    cpu.setPC(cpu.exc.faultPC);
    cpu.psl = cpu.psl & ~PSL_TP;
    let cc = cpu.exc.intexc(cpu, vec, 0, IE.EXC);
    cpu.psl = (cpu.psl & ~CC_MASK) | (cc & CC_MASK);
}

/**
 * chmBody(decoder, cpu)
 *
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 */
function chmBody(decoder, cpu)
{
    let cc = cpu.exc.chm(cpu, decoder.opc, decoder.opnd[0]);
    cpu.psl = (cpu.psl & ~CC_MASK) | (cc & CC_MASK);
    cpu.exc.setIRQL(cpu);
}

/*
 * EXC_OPCODES: dispatch table keyed by NUMERIC opcode, built once from drom.js's OPCODES.
 */
const EXC_OPCODES = {};
for (let name in BODIES) {
    let opc = OPCODES.indexOf(name);
    if (opc < 0) throw new Error(`exc.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
    EXC_OPCODES[opc] = BODIES[name];
}

/** Mnemonics this file wires up, for a harness to enumerate without re-deriving the list. */
const IMPLEMENTED = Object.keys(BODIES);

/**
 * executeExc(opc, decoder, cpu)
 *
 * @param {number} opc
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 * @returns {boolean} true if this module handled the opcode
 */
function executeExc(opc, decoder, cpu)
{
    let fn = EXC_OPCODES[opc];
    if (!fn) return false;
    fn(decoder, cpu);
    return true;
}

export default VAXExc;
export {
    VAXExc, VAXStop, executeExc, EXC_OPCODES, BODIES as EXC_HANDLERS, IMPLEMENTED,
    SCB, MT, MT_MAX, IPR_DEVICE, IE, SW_INT_MASK,
    PSL_CM, PSL_TP, PSL_FPD, PSL_IS, PSL_CUR, PSL_PRV, PSL_IPL, PSL_MBZ,
    PSL_V_CUR, PSL_V_PRV, PSL_V_IPL, PSL_M_IPL, PSL_M_MODE, PSL_IPL1, PSL_IPL1F, PSL_IPL17,
    PSW_T, PSW_IV, PSW_FU, PSW_DV, PSW_MBZ,
    CC_N, CC_Z, CC_V, CC_C, CC_MASK,
    KERN, EXEC, SUPV, USER,
    SISR_MASK, SISR_2, AST_MAX,
    IPL_HLTPIN, IPL_MEMERR, IPL_CRDERR, IPL_HMAX, IPL_HMIN, IPL_SMAX,
    TIR_TRAP, TIR_V_TRAP, TIR_M_TRAP, TRAP_INTOV, TRAP_DIVZRO,
    CVAX_SID, CVAX_UREV, BR_MASK, ccIIZZ_L
};
