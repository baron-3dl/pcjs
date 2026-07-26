/**
 * @fileoverview VAX CPU state: the fetch/dispatch/trap loop
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
 * `sim_instr()` (open-simh VAX/vax_cpu.c:495-3175) minus the parts that already live somewhere
 * else.  Every prior VAX item in this directory proved that AN INSTRUCTION MATCHES; this file is
 * what makes A PROGRAM RUN.  Concretely it owns exactly four things:
 *
 *   1. ONE dispatch table across all five execution modules -- cpu.js (integer/logical/field),
 *      control.js (branch/jump/call/stack), fpa.js (F/D/G float), strq.js (string/queue/INDEX/
 *      PROBE) and exc.js (MTPR/MFPR/REI/CHMx/LDPCTX/SVPCTX/HALT/BPT/XFC/NOP).  Built positionally
 *      from drom.js's OPCODES, with a duplicate-claim check and a coverage check that FAIL AT
 *      MODULE LOAD, so the table can never silently drift from what the modules implement.
 *
 *   2. `fault_PC` and the FPD (first-part-done) resume path.  `fault_PC` is vax_cpu.c:261 -- the
 *      PC of the instruction currently executing, captured before its opcode is fetched.  It
 *      physically lives on VAXExc (exc.js already needed it for BPT/XFC), and this loop is what
 *      maintains it.  Its absence is what forced strq.js's restartable string instructions to
 *      leave PC where it was rather than compute `fault_PC + STR_GETDPC(R[0])`; with this loop in
 *      place they do the real thing.
 *
 *   3. The abort/unwind boundary.  SIMH uses setjmp/longjmp; PCjs's PDP-11 uses throw/catch and
 *      expects stepCPU()'s CALLER to catch.  decode.js, mmu.js, fpa.js and strq.js all throw
 *      `VAXFault`; exc.js's takeFault() is SIMH's abort handler.  This file is the `setjmp` point:
 *      it is the one place that catches a VAXFault and hands it to takeFault(), and the one place
 *      that catches the re-throw takeFault() itself can produce.
 *
 *   4. `stepCPU(nMinCycles)` and the cycle accounting, over the PCjs Component lifecycle
 *      (initBus/powerUp/powerDown/reset/save/restore), modelled on
 *      machines/dec/pdp11/modules/v2/cpustate.js.
 *
 * ============================================================================
 * WHAT IS DELIBERATELY NOT HERE
 * ============================================================================
 * * The instruction-boundary trap/interrupt/trace-trap block, `intexc()`, `takeFault()` and the
 *   IPL arbitration.  All of that is exc.js's `stepInstruction()`, which was written to be exactly
 *   this loop's inner body and is CALLED, not reimplemented.
 * * Devices.  No Qbus, no console, no interval timer.  `VAXExc.deviceVector()` returns 0, so no
 *   hardware interrupt can be requested and none is dispatched.  A physical access outside RAM/ROM
 *   is a bus fault; `onBusFault()` (pcjsvax-446) reproduces vax_sysdev.c's ReadReg()/WriteReg()
 *   default case -- SSC bus-timeout bit, then SCB_MCHK through exc.js's existing SCB dispatch --
 *   for exactly that one trigger.  It is not a full machine-check model: see onBusFault()'s and
 *   exc.js's doc comments for what is and is not covered, and pcjsvax-d22 for the Qbus/CQBIC
 *   (`ReadQb`/`cq_merr`/deferred `mem_err`) mechanism this deliberately does not reproduce.
 * * Packed decimal / CIS and H_floating ARITHMETIC.  But see EMULATED INSTRUCTIONS below: the
 *   *interface* by which a VAX without those options hands them to software lives in vax_cpu.c,
 *   i.e. here, and is implemented.
 * * The run loop / speed governor / UI bindings that CPUPDP11 (machines/dec/pdp11/modules/v2/
 *   cpu.js) provides for PDPjs.  This class extends Component directly and implements only the
 *   lifecycle and stepCPU(); a VAX Computer/Panel component can layer the rest on later without
 *   changing anything here.
 *
 * ============================================================================
 * EMULATED INSTRUCTIONS -- WHY A CIS OPCODE IS NOT A HOLE
 * ============================================================================
 * On a KA655 the packed-decimal and emulated-only groups are NOT implemented in hardware.  SIMH
 * models that exactly: vax_cis.c's `op_cis()` tests the instruction set and, finding no
 * VAX_PACKED/VAX_EMONL, returns `cpu_emulate_exception()` (vax_cpu.c:3259) before its own switch
 * runs.  That routine pushes a 12-longword frame (or a 2-longword frame when PSL<FPD> is set) and
 * vectors through SCB_EMULATE / SCB_EMULFPD, handing the instruction to software.
 *
 * That is a CPU-loop mechanism, not a CIS implementation, so it is ported here.  It matters more
 * than it sounds: the EHKAA diagnostic supplies its own SCB_EMULATE handler, so a machine with
 * this routine and no CIS arithmetic at all executes EHKAA's packed-decimal tests correctly.
 *
 * H_floating is NOT the same case.  vax_octa.c's `op_octa()` raises a RESERVED INSTRUCTION FAULT
 * when VAX_EXTAC is absent -- it does not emulate -- and that includes `EMODH`/`POLYH`, which are
 * IG_EMONL by group but reach op_octa by opcode.  So the routing below keys on SIMH's explicit
 * `case` list (`CIS_EMULATED`), never on the decode ROM's instruction group; getting that wrong
 * sends EMODH down the emulate path and silently changes the machine.
 *
 * ============================================================================
 * ADDRESS / DATA CONVENTION -- see defines.js and decode.js's file header
 * ============================================================================
 * Same as every sibling module: virtual addresses are full 32-bit signed int32s (`>>` on one is a
 * defect), physical addresses are normalized at the Bus boundary, longword data is signed int32.
 */

import Component from "../../../../modules/v2/component.js";
import MESSAGE from "./message.js";
import MMUVAX from "./mmu.js";
import { VAX } from "./defines.js";
import { OPCODES, DROM, DROM_STRIDE, DR, IG } from "./drom.js";
import { VAXDecoder, VAXFault, VAXFAULT } from "./decode.js";
import { DISPATCH as INT_DISPATCH, CC_MASK } from "./cpu.js";
import { CONTROL_OPCODES } from "./control.js";
import { STRQ_OPCODES } from "./strq.js";
import VAXFloat from "./fpa.js";
import VAXExc, {
    VAXStop, EXC_OPCODES, SCB, PSL_FPD, PSL_TP, PSW_T, PSW_IV, PSW_FU, PSW_DV, MCHK_READ
} from "./exc.js";

const L_BYTE = 1, L_WORD = 2, L_LONG = 4;
const nSP = 14, nPC = 15;

const PSL_IS = 1 << 26;
const PSL_IPL1F = 0x1F << 16;
const AST_MAX = 4;                          // vax_cpu.c cpu_reset(): ASTLVL = 4

/**
 * The console ROM's model-detection byte, written to ROMBASE+4 before execution (vax_sysdev.c
 * cpu_boot():1729 -- `rom_wr_B (ROMBASE+4, sys_model ? 1 : 2)`).  MEASURED, not assumed
 * (pcjsvax-223): `examine MODEL` against a stock `microvax3900` SIMH binary -- the exact oracle
 * this machine is graded against -- returns 0, even though the binary's name and this project's
 * stated target are both "MicroVAX 3900".  vax_sysdev.c:1859 prints sys_model=0 as "VAXserver
 * 3900 (KA655)", not "MicroVAX" -- the naming is misleading, the bit value is what the ROM reads.
 * So sys_model=0 on the actual oracle, and the byte this ROM needs is 2, not 1.
 * tests/romdiff.js RE-DERIVES this from a live SIMH `examine MODEL` rather than trusting this
 * comment, and fails loudly if the oracle's default ever changes.
 */
const ROM_MAGIC_BYTE = 2;

/**
 * The opcodes vax_cpu.c routes to `op_cis()` (vax_cpu.c:3138-3147).  On this CPU model every one
 * of them lands in `cpu_emulate_exception()`.  Written as MNEMONICS and resolved through
 * drom.js's OPCODES for the same reason every other table here is: an opcode NUMBER transcribed by
 * hand is a bug with no test.
 *
 * NOTE the two names that are NOT here and look like they should be: `EMODH` and `POLYH` are
 * IG_EMONL, but vax_cpu.c lists them with the octaword cases, so they reach `op_octa()` and take a
 * reserved-instruction fault instead.  See the file header.
 */
const CIS_EMULATED_NAMES = [
    "CVTPL",
    "MOVP", "CMPP3", "CMPP4", "CVTLP",
    "CVTPS", "CVTSP", "CVTTP", "CVTPT",
    "ADDP4", "ADDP6", "SUBP4", "SUBP6",
    "MULP", "DIVP", "ASHP", "CRC",
    "MOVTC", "MOVTUC", "MATCHC", "EDITPC"
];

/**
 * SIX OPCODES ARE IMPLEMENTED TWICE, AND NOTHING BEFORE THIS FILE NOTICED.
 *
 * tests/base_group_residual.js -- the re-runnable proof that the Base Instruction Group is fully
 * implemented -- unions the five modules' exported lists into a Set, so an opcode claimed by two
 * modules is indistinguishable from an opcode claimed by one.  Wiring a single dispatch table is
 * the first thing that has to CHOOSE, and it found these:
 *
 *   PUSHL, PUSHAB, PUSHAW, PUSHAL, PUSHAQ   cpu.js and control.js
 *   NOP                                     strq.js and exc.js
 *
 * Both copies of each were read side by side against vax_cpu.c and both are correct and
 * behaviorally identical (the PUSHx pair differ only in whether SP is decremented into a local
 * before the Write or after it -- SIMH writes at SP-4 and then assigns SP, which is what both
 * do), and both are separately graded by their own module's differential.  So this is a
 * duplication, not a conflict, and picking a side changes nothing observable.
 *
 * It is still resolved EXPLICITLY, by name, rather than by table-build order, so that the choice
 * is reviewable and so that a SEVENTH duplicate -- which would not be so harmless -- still fails
 * the load.  The assignment follows each module's own stated scope: control.js's header claims
 * "the stack instructions (PUSHR/POPR/PUSHL/PUSHAx)", and strq.js's header claims NOP with the
 * reasoning for why it lives there.
 */
const OVERLAP_OWNER = {
    PUSHL: "control.js", PUSHAB: "control.js", PUSHAW: "control.js",
    PUSHAL: "control.js", PUSHAQ: "control.js",
    NOP: "strq.js"
};

/**
 * buildDispatch()
 *
 * ONE table, 512 entries, `(cpu, decoder) => void`.  Built by asking each execution module which
 * numeric opcodes it claims -- never by re-listing mnemonics here, which is how a dispatcher
 * drifts from the modules it dispatches to.
 *
 * Two invariants are checked HERE, at module load, and throw rather than warn:
 *
 *   (a) NO OPCODE IS CLAIMED TWICE.  Two modules implementing the same opcode is not a merge
 *       conflict a table build should resolve by precedence; it means one of them is wrong.
 *   (b) EVERY opcode in the instruction set this CPU implements (IG_BASE + IG_BSGFL + IG_BSDFL,
 *       which is SIMH's VAX_FULL_BASE for a microvax3900) HAS a handler.  This is the shipped-code
 *       form of tests/base_group_residual.js's computation: if a future edit removes a body, the
 *       machine refuses to load instead of taking a reserved-instruction fault at run time.
 *
 * @returns {Array} 512 entries, null where unclaimed
 */
function buildDispatch()
{
    let table = new Array(512).fill(null);
    let owner = new Array(512).fill(null);

    let claimCount = new Array(512).fill(0);
    let claim = function(opc, name, fn) {
        opc |= 0;
        let mn = OPCODES[opc];
        claimCount[opc]++;
        let arbiter = OVERLAP_OWNER[mn];
        if (arbiter) {
            if (arbiter != name) return;                // the other module's copy: deliberately unused
            if (table[opc]) {
                throw new Error(`cpustate.js: ${mn} claimed twice by its own arbitrated owner ${name}`);
            }
            table[opc] = fn;
            owner[opc] = name;
            return;
        }
        if (table[opc]) {
            throw new Error(`cpustate.js: opcode ${opc.toString(16)} (${mn || "?"}) is claimed by both ${owner[opc]} and ${name}`);
        }
        table[opc] = fn;
        owner[opc] = name;
    };

    /*
     * exc.js and strq.js publish {opcode: fn} maps; cpu.js publishes a positional array; control.js
     * publishes a {opcode: fn} map.  fpa.js is the odd one out -- it dispatches internally through
     * one `execute(opc)` switch (a direct port of vax_cpu.c's own float cases), so its claim is
     * per-opcode but its body is the same closure.  OPC (fpa.js) is not exported as a table, so the
     * claim set is computed by ASKING it: an opcode belongs to fpa.js exactly when its decode ROM
     * group is IG_BSGFL/IG_BSDFL, or when it is one of the F-format base-group opcodes fpa.js owns.
     * That last set is not guessable, so fpa.js is claimed LAST over whatever no one else took --
     * and (b) below then proves nothing was left over.
     */
    for (let opc in EXC_OPCODES) {
        let fn = EXC_OPCODES[opc];
        claim(+opc, "exc.js", (cpu, d) => fn(d, cpu));
    }
    for (let opc = 0; opc < 512; opc++) {
        if (INT_DISPATCH[opc]) claim(opc, "cpu.js", INT_DISPATCH[opc]);
    }
    for (let opc in CONTROL_OPCODES) {
        let fn = CONTROL_OPCODES[opc];
        claim(+opc, "control.js", (cpu, d) => fn(d, cpu));
    }
    for (let opc in STRQ_OPCODES) {
        let fn = STRQ_OPCODES[opc];
        claim(+opc, "strq.js", (cpu, d) => fn(cpu, d.opnd));
    }
    /*
     * fpa.js last, over the residue of the implemented instruction set.  `fpu.execute()` returns
     * FALSE for an opcode it does not handle, which turns into the same load-time failure (b)
     * below produces, rather than a silent no-op.
     */
    let fpaClaimed = [];
    for (let opc = 0; opc < 512; opc++) {
        if (table[opc] || !OPCODES[opc]) continue;
        let grp = (DROM[opc * DROM_STRIDE] >> DR.V_IGMASK) & DR.M_IGMASK;
        if (grp != IG.BASE && grp != IG.BSGFL && grp != IG.BSDFL) continue;
        fpaClaimed.push(opc);
        claim(opc, "fpa.js", (cpu, d) => {
            if (!cpu.fpu.execute(d.opc)) {
                throw new Error(`cpustate.js: fpa.js declined opcode ${d.opc.toString(16)} (${OPCODES[d.opc] || "?"})`);
            }
        });
    }

    /*
     * (a') the arbitration list is neither stale nor speculative: the opcodes claimed more than
     * once must be EXACTLY the OVERLAP_OWNER keys.  An entry that stopped being a duplicate is as
     * much a drift as a duplicate that stopped being arbitrated -- the first would silently
     * suppress the surviving module's body.
     */
    let dupes = [];
    for (let opc = 0; opc < 512; opc++) if (claimCount[opc] > 1) dupes.push(OPCODES[opc]);
    let arbitrated = Object.keys(OVERLAP_OWNER);
    let stale = arbitrated.filter((mn) => dupes.indexOf(mn) < 0);
    let unarbitrated = dupes.filter((mn) => !OVERLAP_OWNER[mn]);
    if (stale.length || unarbitrated.length) {
        throw new Error(`cpustate.js: OVERLAP_OWNER drift -- no longer duplicated: [${stale.join(", ")}]; duplicated but unarbitrated: [${unarbitrated.join(", ")}]`);
    }

    /* (b) coverage over the implemented instruction set */
    let missing = [];
    for (let opc = 0; opc < 512; opc++) {
        if (!OPCODES[opc]) continue;
        let grp = (DROM[opc * DROM_STRIDE] >> DR.V_IGMASK) & DR.M_IGMASK;
        if (grp != IG.BASE && grp != IG.BSGFL && grp != IG.BSDFL) continue;
        if (!table[opc]) missing.push(OPCODES[opc]);
    }
    if (missing.length) {
        throw new Error(`cpustate.js: no instruction body for ${missing.length} implemented opcode(s): ${missing.join(", ")}`);
    }
    return {table, owner, fpaClaimed};
}

const BUILT = buildDispatch();
const DISPATCH = BUILT.table;
const DISPATCH_OWNER = BUILT.owner;

/** Numeric opcodes routed to cpu_emulate_exception(). */
const CIS_EMULATED = (function() {
    let s = new Set();
    for (let name of CIS_EMULATED_NAMES) {
        let opc = OPCODES.indexOf(name);
        if (opc < 0) throw new Error(`cpustate.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
        s.add(opc);
    }
    return s;
})();

/**
 * @class CPUStateVAX
 * @unrestricted
 *
 * @property {BusVAX} bus
 * @property {MMUVAX} mmu
 * @property {VAXDecoder} decoder
 * @property {VAXExc} exc
 * @property {VAXFloat} fpu
 * @property {Int32Array} regs   R0-R15 (R12=AP, R13=FP, R14=SP, R15=PC)
 * @property {number} psl        full PSL, condition codes included at all times
 */
export default class CPUStateVAX extends Component {
    /**
     * CPUStateVAX(parmsCPU)
     *
     * @param {Object} [parmsCPU]
     */
    constructor(parmsCPU)
    {
        super("CPU", parmsCPU || {}, MESSAGE.CPU);

        this.bus = null;
        this.mmu = null;
        this.dbg = null;

        this.regs = new Int32Array(16);
        this.psl = 0;

        /*
         * SIMH's `extra_bytes` (vax_cpu.c:278): bytes referenced by the current string instruction.
         * The string bodies in strq.js increment it; stepCPU() charges `1 + (extra_bytes >> 5)`
         * cycles per instruction and then clears it, exactly as vax_cpu.c:729-730 does.  Note the
         * ORDER SIMH uses and this reproduces: the charge is levied BEFORE the opcode fetch, so an
         * instruction pays for the string bytes the PREVIOUS instruction referenced.
         */
        this.extraBytes = 0;

        /*
         * SIMH's `r`/`rh` locals, reconstructed for the instruction-history trace only.  See
         * traceResult().  Nothing in the machine's behavior reads them.
         */
        this.rTrace = 0;
        this.rhTrace = 0;

        this.nBurstCycles = 0;
        this.nStepCycles = 0;
        this.nTotalCycles = 0;
        this.fStopped = false;
        this.stopReason = null;

        /*
         * Instruction-history hook, or null.  Two calls, at exactly the two points vax_cpu.c takes
         * its own history: `record(cpu, opc)` after specifier resolution and before the body
         * (vax_cpu.c:1563-1590), and `finish(cpu)` at the TOP OF THE NEXT ITERATION
         * (vax_cpu.c:613-649).  The second point is not cosmetic -- SIMH fills a record's RESULT
         * fields there, so an instruction that FAULTED gets its results filled after the exception
         * has already been dispatched, and a trace that fills them immediately after the body
         * diverges on exactly the records that matter most.
         */
        this.hst = null;

        this.decoder = new VAXDecoder(this);
        this.exc = new VAXExc(this);
        this.fpu = new VAXFloat(this);

        /* allocated once: stepInstruction() takes the instruction body as a callback */
        this.fnExecute = (opc, d, c) => {
            if (this.hst) this.hst.record(this, opc);
            this.executeOne(opc, d, c);
            if (this.hst) this.hst.captureResult(this);
        };

        this.setReady();
    }

    /**
     * initBus(cmp, bus, cpu, dbg)
     *
     * @this {CPUStateVAX}
     * @param {Object} cmp
     * @param {BusVAX} bus
     * @param {CPUStateVAX} cpu
     * @param {Object} dbg
     */
    initBus(cmp, bus, cpu, dbg)
    {
        this.cmp = cmp;
        this.dbg = dbg || null;
        this.setBus(bus);
    }

    /**
     * setBus(bus)
     *
     * Usable standalone (a harness constructs a Bus and a CPU with no Computer), which is how
     * every differential in tests/ builds its machine.
     *
     * @this {CPUStateVAX}
     * @param {BusVAX} bus
     */
    setBus(bus)
    {
        this.bus = bus;
        this.mmu = new MMUVAX(bus);
        bus.setFaultHandler((addr, access) => this.onBusFault(addr, access));
    }

    /**
     * onBusFault(addr, access)
     *
     * A physical reference to an address BusVAX.RESERVED reserves but does not decode (the
     * KA655 I/O, register, NVR and Qbus-memory ranges -- see bus.js) -- pcjsvax-446 -- OR to an SSC
     * sub-register pcjsvax-320's decode does not cover (the console UART mirror, the T0/T1 timers;
     * see ssc.js) -- lands here identically, via the same bus.fault() -> onBusFault() path, whether
     * the whole range is reserved or just one register within a partially-decoded one is.  This is
     * how the console ROM discovers hardware: probe an address, and if nothing answers, take a
     * machine check instead of ending the run.
     *
     * TWO DIFFERENT SIMH MECHANISMS LAND HERE, and they must be told apart (veracity re-dispatch,
     * pcjsvax-446; the Qbus branch completed by pcjsvax-d22):
     *
     *   - vax_sysdev.c's ReadReg()/WriteReg() `default:` case (:1031-1032, :1071-1072) -- reached
     *     for CDG/REG-gap/SSC/NVR addresses -- sets the SSC bus-timeout bit AND raises SCB_MCHK.
     *   - vax_io.c's ReadQb()/WriteQb() (reached instead for IOPAGE/CQM addresses, via
     *     VAX.isQbusAddr() == ADDR_IS_IO()/ADDR_IS_CQM(): a KA655 routes Qbus I/O-page and
     *     Qbus-memory references through the CQBIC, not ReadReg/WriteReg) calls `cq_merr()`
     *     (exc.js's cqMerr() -- the CQBIC's DSER/MEAR error registers, pcjsvax-d22) on BOTH
     *     directions, and NEVER touches ssc_bto -- but the two directions then diverge completely:
     *     READS take the SAME MACH_CHECK() the register-space branch does; WRITES do not reach
     *     MACH_CHECK() at all.  WriteQb's unbacked case sets a DEFERRED `mem_err` flag (exc.js's
     *     `this.exc.memErr`) and returns NORMALLY -- there is nothing to store into, so the write
     *     is silently discarded and the instruction completes and PC simply advances, exactly as
     *     if it had succeeded.  The deferred mem_err is delivered LATER, at the next instruction
     *     boundary where IPL permits, through the ALREADY-existing generic interrupt seam
     *     (evalInt()/getVector()'s `lvl === IPL_MEMERR` case, addInterruptSource()'s neighbor --
     *     see exc.js's file header) -- installed against, not rebuilt, by pcjsvax-d22.
     *
     * Suppressing busTimeout() for a Qbus address is therefore correct, not incidental: setting
     * ssc_bto there would be a genuine divergence from real SIMH (measured directly;
     * tests/mchkdiff.js grades exactly this).  cqMerr()'s LST (lost-error) bit for a Qbus WRITE
     * depends on how many 16-bit Qbus cycles the reference took, which is why mmu.js's writeL()/
     * writeU() -- not this method -- decide how many times onBusFault() (and therefore cqMerr())
     * fires for a single write; see their doc comments.
     *
     * `delta` (PC - fault_PC, "how far decode had consumed the faulting instruction") is captured
     * HERE, before takeFault()'s common preamble resets PC back to fault_PC, and is smuggled to
     * exc.js's SCB.MCHK case through the VAXFault's p2 slot -- see its doc comment for why it
     * cannot be recomputed there.  A deferred write never reaches takeFault() at all, so `delta`
     * is irrelevant there -- nothing is thrown.
     *
     * @this {CPUStateVAX}
     * @param {number} addr
     * @param {number} access one of VAX.ACCESS.* -- VAX.ACCESS.WRITE for a write, otherwise a read
     */
    onBusFault(addr, access)
    {
        let a = addr >>> 0;
        let fWrite = access === VAX.ACCESS.WRITE;
        if (VAX.isQbusAddr(a)) {
            this.exc.cqMerr(a);
            if (fWrite) {
                this.exc.memErr = 1;      // deferred -- no synchronous exception; PC just advances
                return;
            }
            let delta = (this.regs[nPC] - this.exc.faultPC) | 0;
            throw new VAXFault(-SCB.MCHK, MCHK_READ, delta);
        }
        let p1 = this.exc.busTimeout(fWrite);
        let delta = (this.regs[nPC] - this.exc.faultPC) | 0;
        throw new VAXFault(-SCB.MCHK, p1, delta);
    }

    /* --------------------------------------------------------------------------------------- *
     * PSL accessors.  `cc` needs a SETTER here: cpu.js's VAXCpu publishes a getter only, and     *
     * fpa.js assigns `cpu.cc` directly (it is a separate local inside sim_instr, which is what    *
     * fpa.js was ported against).  `acc` is fpa.js's ACC_MASK(cur) in RA form.  `trpirq` is       *
     * SIMH's own variable, which fpa.js and strq.js write for a deferred arithmetic trap and      *
     * exc.js reads at the top of every instruction -- one variable, published in one place, so    *
     * the two conventions cannot drift.                                                          *
     * --------------------------------------------------------------------------------------- */

    get cc() { return this.psl & CC_MASK; }
    set cc(v) { this.psl = (this.psl & ~CC_MASK) | (v & CC_MASK); }

    setCC(cc) { this.psl = (this.psl & ~CC_MASK) | (cc & CC_MASK); }

    get acc() { return MMUVAX.accRead(this.curMode()); }

    get trpirq() { return this.exc.trpirq; }
    set trpirq(v) { this.exc.trpirq = v | 0; }

    /** SIMH's fault_PC (vax_cpu.c:261), maintained by stepCPU()/stepInstruction(). */
    get faultPC() { return this.exc.faultPC; }

    /**
     * curMode() -- PSL<25:24>, SIMH's PSL_GETCUR.
     * @returns {number} 0=Kernel 1=Executive 2=Supervisor 3=User
     */
    curMode() { return (this.psl >>> 24) & 3; }

    accR() { return MMUVAX.accRead(this.curMode()); }
    accW() { return MMUVAX.accWrite(this.curMode()); }

    /**
     * fault(code)
     * @param {number} code one of VAXFAULT.*
     * @throws {VAXFault}
     */
    fault(code) { throw new VAXFault(code); }

    /* --- VAXDecodeMachine interface, consumed by decode.js --- */

    /**
     * getISTR(lnt)
     *
     * vax_cpu.c's GET_ISTR.  SIMH keeps an 8-byte prefetch buffer (`get_istr`, vax_cpu.c:3196);
     * this reads through the MMU per reference instead, for the same reason mmu.js has no
     * block-per-page paging: the buffer is a host-side optimization, and reproducing its
     * invalidation rules is a source of bugs with no architectural payoff.  The observable
     * difference is the number of MMU references, which nothing in the trace records.
     *
     * @this {CPUStateVAX}
     * @param {number} lnt 1, 2 or 4
     * @returns {number}
     */
    getISTR(lnt)
    {
        let v = this.mmu.readData(this.regs[nPC], lnt, this.accR());
        this.regs[nPC] = (this.regs[nPC] + lnt) | 0;
        return v;
    }

    /**
     * @this {CPUStateVAX}
     * @param {number} pc
     */
    setPC(pc) { this.regs[nPC] = pc | 0; }

    /**
     * @this {CPUStateVAX}
     * @param {number} va
     * @param {number} lnt
     * @param {boolean} fWrite   TRUE for SIMH's WA (a `modify` specifier), FALSE for RA
     * @returns {number}
     */
    readData(va, lnt, fWrite)
    {
        return this.mmu.readData(va, lnt, fWrite ? this.accW() : this.accR());
    }

    /**
     * writeData(va, val, lnt)
     *
     * control.js's store path (it was ported against a harness CPU that had this); every other
     * module goes through `cpu.mmu.writeData(va, val, lnt, acc)` directly.
     *
     * @this {CPUStateVAX}
     * @param {number} va
     * @param {number} val
     * @param {number} lnt
     */
    writeData(va, val, lnt)
    {
        this.mmu.writeData(va, val, lnt, this.accW());
    }

    /* --------------------------------------------------------------------------------------- *
     * Lifecycle                                                                                 *
     * --------------------------------------------------------------------------------------- */

    /**
     * reset()
     *
     * vax_cpu.c `cpu_reset()` (3323).  PSL = PSL_IS|PSL_IPL1F, SISR = 0, ASTLVL = 4, mapen = 0,
     * halt pin and both error flags clear, register file zero.
     *
     * ASTLVL is set HERE rather than in VAXExc.reset(): exc.js's reset() is the "clear my state"
     * primitive its own differential drives, and 4 is a CPU-reset value, not a neutral one.
     *
     * @this {CPUStateVAX}
     */
    reset()
    {
        this.regs.fill(0);
        this.exc.reset();
        this.exc.astlvl = AST_MAX;
        this.psl = (PSL_IS | PSL_IPL1F) | 0;
        if (this.mmu) {
            this.mmu.reset();
            this.mmu.setMAPEN(0);
        }
        this.decoder.resetRecovery();
        this.extraBytes = 0;
        this.rTrace = this.rhTrace = 0;
        this.nTotalCycles = 0;
        this.fStopped = false;
        this.stopReason = null;
    }

    /**
     * powerUp(data, fRepower)
     *
     * @this {CPUStateVAX}
     * @param {Object|null} [data]
     * @param {boolean} [fRepower]
     * @returns {boolean}
     */
    powerUp(data, fRepower)
    {
        if (!fRepower) {
            if (!data) {
                this.reset();
            } else if (!this.restore(data)) {
                return false;
            }
        }
        return true;
    }

    /**
     * powerDown(fSave, fShutdown)
     *
     * @this {CPUStateVAX}
     * @param {boolean} [fSave]
     * @param {boolean} [fShutdown]
     * @returns {Object|boolean}
     */
    powerDown(fSave, fShutdown)
    {
        return fSave ? this.save() : true;
    }

    /**
     * save()
     *
     * The complete architectural state: register file, PSL, the privileged state exc.js owns, and
     * the map registers mmu.js owns.  The translation buffer is deliberately NOT saved -- it is a
     * cache, and restore() refills it on demand.
     *
     * @this {CPUStateVAX}
     * @returns {Object}
     */
    save()
    {
        let e = this.exc, m = this.mmu;
        return {
            regs: Array.from(this.regs),
            psl: this.psl,
            cycles: this.nTotalCycles,
            extraBytes: this.extraBytes,
            exc: {
                stk: Array.from(e.stk), scbb: e.scbb, pcbb: e.pcbb, astlvl: e.astlvl,
                sisr: e.sisr, pme: e.pme, trpirq: e.trpirq, inIE: e.inIE,
                memErr: e.memErr, crdErr: e.crdErr, hltPin: e.hltPin, faultPC: e.faultPC
            },
            mmu: m ? {
                mapen: m.mapen, p0br: m.p0br, p0lr: m.p0lr, p1br: m.p1br, p1lr: m.p1lr,
                sbr: m.sbr, slr: m.slr
            } : null
        };
    }

    /**
     * restore(data)
     *
     * @this {CPUStateVAX}
     * @param {Object} data
     * @returns {boolean}
     */
    restore(data)
    {
        if (!data || !data.regs) return false;
        this.regs.set(data.regs);
        this.psl = data.psl | 0;
        this.nTotalCycles = data.cycles | 0;
        this.extraBytes = data.extraBytes | 0;
        let e = this.exc, s = data.exc;
        if (s) {
            e.stk.set(s.stk);
            e.scbb = s.scbb; e.pcbb = s.pcbb; e.astlvl = s.astlvl; e.sisr = s.sisr;
            e.pme = s.pme; e.trpirq = s.trpirq; e.inIE = s.inIE;
            e.memErr = s.memErr; e.crdErr = s.crdErr; e.hltPin = s.hltPin;
            e.faultPC = s.faultPC;
        }
        if (this.mmu && data.mmu) {
            let m = data.mmu;
            this.mmu.setP0BR(m.p0br); this.mmu.setP0LR(m.p0lr);
            this.mmu.setP1BR(m.p1br); this.mmu.setP1LR(m.p1lr);
            this.mmu.setSBR(m.sbr);   this.mmu.setSLR(m.slr);
            this.mmu.setMAPEN(m.mapen);
        }
        this.fStopped = false;
        this.stopReason = null;
        return true;
    }

    /**
     * loadImage(bytes, origin)
     *
     * SIMH's `LOAD` for this system (vax_syslist.c:114 `sim_load`): with no `-R`/`-N`/`-O` switch
     * it is a raw byte stream deposited at physical 0.  Nothing more -- there is no header, no
     * relocation, no symbol table; `load ehkaa.exe` really is `memcpy`.
     *
     * @this {CPUStateVAX}
     * @param {Uint8Array} bytes
     * @param {number} [origin]
     */
    loadImage(bytes, origin = 0)
    {
        for (let i = 0; i < bytes.length; i++) {
            this.mmu.writeB((origin + i) >>> 0, bytes[i]);
        }
    }

    /**
     * boot(magicByte = ROM_MAGIC_BYTE)
     *
     * vax_sysdev.c `cpu_boot()` (1714-1732), MINUS `sysd_powerup()`'s device-register clears
     * (CMCTL, SSC timer/address-match registers, `ka_cacr`): no such devices are modeled yet
     * (pcjsvax-223's constraints exclude them; that is the boundary the differential is built to
     * find), and on a machine that has only ever seen reset() -- never actually run -- those
     * registers don't exist to need clearing, so the omission is a no-op here, not a divergence.
     * See docs item pcjsvax-223 for the full accounting.
     *
     * NOT `cpu_reset()`: PC is NOT set to ROMBASE by reset (vax_cpu.c:3323-3349 confirms it), only
     * by this.  Call reset() first for a clean register file; boot() itself touches only PC, PSL
     * and the ROM's own magic byte, exactly as cpu_boot() does.
     *
     * The magic byte is written through the bus's DIRECT accessor.  The ROM is genuinely read-only
     * to the EXECUTING machine (BusVAX.addRom() makes normal writes fault), but this one byte is
     * written BEFORE execution starts, which is what real hardware's cpu_boot() does too -- a
     * boot-time console action, not a bus write the running program could ever issue.  Modeling it
     * any other way (e.g. leaving the ROM permanently writable) would be dishonest about the
     * ordering.
     *
     * @this {CPUStateVAX}
     * @param {number} [magicByte]
     */
    boot(magicByte = ROM_MAGIC_BYTE)
    {
        this.setPC(VAX.PHYSMEM.ROM_BASE);
        this.psl = (PSL_IS | PSL_IPL1F) >>> 0;
        this.bus.setByteDirect((VAX.PHYSMEM.ROM_BASE + 4) >>> 0, magicByte & 0xFF);
    }

    /* --------------------------------------------------------------------------------------- *
     * Dispatch                                                                                  *
     * --------------------------------------------------------------------------------------- */

    /**
     * executeOne(opc, decoder, cpu)
     *
     * vax_cpu.c's `switch (opc)` (1595-3174), collapsed to a table lookup with SIMH's two
     * non-table outcomes preserved:
     *
     *   * an opcode on the `op_cis()` list becomes `cpu_emulate_exception()` -- see the file
     *     header, and note that it is keyed on the opcode LIST, not the instruction group;
     *   * anything else with no body is `default: RSVD_INST_FAULT`, which is also what
     *     `op_octa()` produces for the H_floating group on a machine without VAX_EXTAC.
     *
     * `buildDispatch()` already proved at load time that no IMPLEMENTED opcode reaches those two
     * outcomes, so a reserved-instruction fault here is always a genuine architectural one.
     *
     * @this {CPUStateVAX}
     * @param {number} opc
     * @param {VAXDecoder} decoder
     * @param {CPUStateVAX} cpu
     */
    executeOne(opc, decoder, cpu)
    {
        let fn = DISPATCH[opc];
        if (fn) {
            fn(cpu, decoder);
            return;
        }
        if (CIS_EMULATED.has(opc)) {
            this.emulateException(opc, decoder);
            return;
        }
        throw new VAXFault(VAXFAULT.RESIN);
    }

    /**
     * emulateException(opc, decoder)
     *
     * vax_cpu.c:3259, `cpu_emulate_exception()`.  Hands an instruction this CPU does not implement
     * to software through SCB_EMULATE / SCB_EMULFPD.
     *
     * THREE THINGS A PORT GETS WRONG HERE, all of them in SIMH's source and none of them obvious:
     *
     *   1. The FPD frame is TWO longwords (old PC, PSL) and the non-FPD frame is TWELVE (opcode,
     *      old PC, six operands, current PC, PSL) -- but the non-FPD frame is written at FIXED
     *      NEGATIVE OFFSETS from the CURRENT SP and SP is decremented ONCE, by 48, at the end.
     *      Note the hole: the six operands land at SP-40..SP-20, then SP-16 and SP-12 are skipped
     *      and the two PC/PSL longwords are at SP-8 and SP-4.  A "push each in turn" loop produces
     *      a different frame.
     *   2. `Read (SP - 1, L_BYTE, WA)` first, purely to probe write access to the stack -- so a
     *      broken kernel stack faults BEFORE any of the frame is written.
     *   3. CVTPL's third operand is a `.wl` write specifier, so opnd[2] holds OP_MEM-or-register-
     *      number and opnd[3] holds the address.  SIMH folds those two slots into one longword the
     *      emulator can use: `opnd[2] = (opnd[2] >= 0)? ~opnd[2]: opnd[3]`, i.e. a register
     *      destination becomes the ONE'S COMPLEMENT of its register number and a memory
     *      destination becomes its address.  Only CVTPL.
     *
     * @this {CPUStateVAX}
     * @param {number} opc
     * @param {VAXDecoder} decoder
     */
    emulateException(opc, decoder)
    {
        let acc = this.accW();
        let mmu = this.mmu;
        let sp = this.regs[nSP];
        let vec;
        if (this.psl & PSL_FPD) {
            mmu.readData((sp - 1) | 0, L_BYTE, acc);            /* wchk stack */
            mmu.writeData((sp - 8) | 0, this.exc.faultPC, L_LONG, acc);
            mmu.writeData((sp - 4) | 0, this.psl, L_LONG, acc);
            this.regs[nSP] = (sp - 8) | 0;
            vec = mmu.readLP((this.exc.scbb + SCB.EMULFPD) & VAX.PAMASK);
        } else {
            let opnd = decoder.opnd;
            let op2 = opnd[2];
            if (opc === OPCODES.indexOf("CVTPL")) {
                op2 = (op2 >= 0) ? ~op2 : opnd[3];
            }
            mmu.readData((sp - 1) | 0, L_BYTE, acc);            /* wchk stack */
            mmu.writeData((sp - 48) | 0, opc, L_LONG, acc);
            mmu.writeData((sp - 44) | 0, this.exc.faultPC, L_LONG, acc);
            mmu.writeData((sp - 40) | 0, opnd[0], L_LONG, acc);
            mmu.writeData((sp - 36) | 0, opnd[1], L_LONG, acc);
            mmu.writeData((sp - 32) | 0, op2, L_LONG, acc);
            mmu.writeData((sp - 28) | 0, opnd[3], L_LONG, acc);
            mmu.writeData((sp - 24) | 0, opnd[4], L_LONG, acc);
            mmu.writeData((sp - 20) | 0, opnd[5], L_LONG, acc);
            mmu.writeData((sp - 8) | 0, this.regs[nPC], L_LONG, acc);
            mmu.writeData((sp - 4) | 0, this.psl, L_LONG, acc);
            this.regs[nSP] = (sp - 48) | 0;
            vec = mmu.readLP((this.exc.scbb + SCB.EMULATE) & VAX.PAMASK);
        }
        this.psl = this.psl & ~(PSL_TP | PSL_FPD | PSW_DV | PSW_FU | PSW_IV | PSW_T);
        this.setPC(vec & ~0x03);
        this.setCC(0);
    }

    /* --------------------------------------------------------------------------------------- *
     * The loop                                                                                  *
     * --------------------------------------------------------------------------------------- */

    /**
     * stepCPU(nMinCycles)
     *
     * `sim_instr()`'s `for(;;)` (vax_cpu.c:609-3175) plus the `setjmp` above it (520-604).
     *
     * WHAT THIS FUNCTION OWNS AND exc.js DOES NOT.  `VAXExc.stepInstruction()` is the body of one
     * iteration: the trap/interrupt/trace-trap block, one decode, one execute, and the abort
     * handler for a fault raised BY the instruction.  It does NOT own (a) the loop, (b) the cycle
     * accounting, (c) the case where the top-of-loop DISPATCH itself faults -- `intexc()` writing a
     * frame onto a broken kernel stack -- which in SIMH longjmps to the same abort handler and here
     * escapes stepInstruction() as a VAXFault.  This function is the setjmp point, so it catches
     * that too, and routes it through the same `takeFault()`.
     *
     * A VAXStop is NOT caught: HALT, an illegal SCB vector, an exception inside the exception flow
     * and a non-existent memory reference are all "return to SCP", which for PCjs means stop the
     * machine.  It is recorded in `stopReason` and re-thrown, because stepCPU()'s CALLER catches --
     * that is the PDPjs idiom (machines/dec/pdp11/modules/v2/cpustate.js stepCPU()), and it is what
     * lets a debugger or a differential harness see the reason rather than a return code.
     *
     * @this {CPUStateVAX}
     * @param {number} nMinCycles
     * @returns {number} cycles executed
     */
    stepCPU(nMinCycles)
    {
        this.nBurstCycles = this.nStepCycles = nMinCycles;
        this.fStopped = false;

        while (this.nStepCycles > 0) {
            /*
             * vax_cpu.c:613-649 -- fill in the PREVIOUS instruction's result fields.  This is the
             * top of the loop, so it runs AFTER any exception the previous instruction's fault
             * dispatched.  See the `hst` property comment.
             */
            if (this.hst) this.hst.finish(this);

            /*
             * vax_cpu.c:729-730.  `sim_interval` is charged BEFORE the fetch, and `extra_bytes`
             * is the count the PREVIOUS instruction accumulated.  Preserved exactly, including
             * that off-by-one-instruction attribution, because it is observable in any cycle
             * comparison against SIMH.
             */
            let nCycles = 1 + (this.extraBytes >> 5);
            this.extraBytes = 0;
            this.nStepCycles -= nCycles;
            this.nTotalCycles += nCycles;

            try {
                this.exc.stepInstruction(this, this.fnExecute);
            } catch (e) {
                if (e instanceof VAXFault) {
                    /*
                     * A fault out of the top-of-loop dispatch block (see the docblock).  SIMH's
                     * longjmp lands in the abort handler with in_ie still set, which is what turns
                     * a second one into KSNV and a third into a stop; takeFault() implements that,
                     * and the bounded retry here is the same shape stepInstruction() uses.
                     */
                    let pending = e;
                    for (let depth = 0; pending; depth++) {
                        if (depth > 3) {
                            this.fStopped = true;
                            this.stopReason = new VAXStop(VAXStop.REASON.INIE, -pending.code);
                            throw this.stopReason;
                        }
                        let f = pending;
                        pending = null;
                        try {
                            this.exc.takeFault(this, f);
                        } catch (e2) {
                            if (!(e2 instanceof VAXFault)) { this.noteStop(e2); throw e2; }
                            pending = e2;
                        }
                    }
                    continue;
                }
                this.noteStop(e);
                throw e;
            }
        }
        return this.nBurstCycles - this.nStepCycles;
    }

    /**
     * noteStop(e)
     *
     * @this {CPUStateVAX}
     * @param {Error} e
     */
    noteStop(e)
    {
        this.fStopped = true;
        this.stopReason = (e instanceof VAXStop) ? e : null;
    }

    /**
     * stopCPU()
     *
     * @this {CPUStateVAX}
     */
    stopCPU()
    {
        this.nStepCycles = 0;
    }

    /**
     * getCycles()
     *
     * @this {CPUStateVAX}
     * @returns {number}
     */
    getCycles() { return this.nTotalCycles; }
}

CPUStateVAX.DISPATCH = DISPATCH;
CPUStateVAX.DISPATCH_OWNER = DISPATCH_OWNER;
CPUStateVAX.CIS_EMULATED = CIS_EMULATED;
CPUStateVAX.ROM_MAGIC_BYTE = ROM_MAGIC_BYTE;

export { CPUStateVAX, DISPATCH, DISPATCH_OWNER, CIS_EMULATED, VAXStop, VAXFault, ROM_MAGIC_BYTE };
