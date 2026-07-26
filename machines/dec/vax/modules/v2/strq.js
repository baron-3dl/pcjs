/**
 * @fileoverview VAX string, queue, INDEX, and PROBE instruction execution
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
 * SCOPE -- READ THIS BEFORE ADDING AN OPCODE
 * ============================================================================
 * This is the last slice of the Base Instruction Group (IG_BASE | IG_BSGFL | IG_BSDFL --
 * see decode.js's `instructionSet`) that cpu.js (pcjsvax-80d, integer/logical/bit-field),
 * control.js (pcjsvax-fab, branch/jump/call/stack) and fpa.js (pcjsvax-710, F/D/G floating) do
 * not implement.  The residual is COMPUTED, not hand-enumerated -- see
 * tests/base_group_residual.js, the re-runnable source of truth this file's scope was derived
 * from.  It implements exactly:
 *
 *   String:   MOVC3, MOVC5, CMPC3, CMPC5, LOCC, SKPC, SCANC, SPANC
 *   Queue:    INSQUE, REMQUE, INSQHI, INSQTI, REMQHI, REMQTI  (interlocked forms are trivially
 *             atomic here -- single-threaded JS has no concurrent access to race against)
 *   Other:    INDEX, PROBER, PROBEW, NOP
 *
 * NOT HERE, ON PURPOSE, AND WHY (all confirmed by tests/base_group_residual.js, not assumed):
 *
 *   - MTPR, MFPR.  Explicit coordination carve-out -- pcjsvax-e49 owns the IPR register file,
 *     running concurrently in a sibling worktree.  Do not implement; do not stub.
 *
 *   - CHMK, CHME, CHMS, CHMU.  Every one of them dispatches through op_chm() -> intexc(), i.e. a
 *     full SCB exception-frame push.  No item owns that mechanism yet (see the finding below) --
 *     deferred, same carve-out class as MTPR/MFPR.
 *
 *   - HALT, BPT, XFC, REI, LDPCTX, SVPCTX.  These showed up in the computed residual too, and are
 *     NOT string/queue/INDEX/PROBE opcodes -- they are exception/context-switch machinery that
 *     cpu.js's own file header optimistically assigned to "pcjsvax-fab" (control.js), but
 *     control.js's ACTUAL scope note explicitly does not claim them.  Investigated one at a time
 *     against vax_cpu.c / vax_cpu1.c:
 *       - BPT, XFC:            unconditionally call intexc() -- full SCB dispatch, same gap as
 *                               CHMx above.
 *       - REI:                 validates against PSL_MBZ/ASTLVL/STK[]-per-mode stack pointers and
 *                               ends with a PSL/PC swap that IS the return half of SCB dispatch --
 *                               meaningless without the mechanism CHMx needs.
 *       - LDPCTX, SVPCTX:      RSVD_INST_FAULT if not kernel mode (needs SCB), and their entire
 *                               body reads/writes KSP/ESP/SSP/USP/IS/PCBB/P0BR/P0LR/P1BR/P1LR/
 *                               ASTLVL -- the privileged IPR/process-context register file mmu.js's
 *                               header already assigns to "the IPR item" (pcjsvax-e49).
 *       - HALT:                RSVD_INST_FAULT if not kernel (needs SCB); the kernel-mode path
 *                               halts the SIMULATOR itself, which has no register-file-plus-PSL
 *                               comparison to grade against -- there is no "next state" to diff.
 *     None of the six is implementable without either the SCB dispatch mechanism or the IPR
 *     register file, both pcjsvax-e49's.  This is a FINDING, reported to the PM for a new item;
 *     it is not silently absorbed here and not silently left undocumented.  NOP, the seventh
 *     residual opcode near this cluster, has neither dependency (vax_cpu.c: `case NOP: break;`)
 *     and IS implemented below.
 *
 *   - MOVTC, MOVTUC, MATCHC, EDITPC, and packed decimal (ADDP4/ADDP6/SUBP4/SUBP6/MULP/DIVP/
 *     CVTPS/CVTSP/CVTPT/CVTTP/MOVP/CMPP3/CMPP4/CVTPL/CVTLP/ASHP/CRC).  These were flagged as
 *     "expected" by the dispatching prompt but tests/base_group_residual.js proves they are
 *     IG_EMONL ("emulated-only"), not IG_BASE/IG_BSGFL/IG_BSDFL -- vax_cis.c's op_cis() routes
 *     every one of them through cpu_emulate_exception() before its own switch even runs, on ANY
 *     VAX that lacks the optional CIS coprocessor (vax_defs.h: VAX_PACKED/VAX_EMONL gate them,
 *     and decode.js's `instructionSet` never sets those bits).  Genuinely out of the Base
 *     Instruction Group for this CPU model -- not deferred, not missing, simply not applicable.
 *
 * ============================================================================
 * NO CPU LOOP OWNS fault_PC -- WHAT THAT COSTS THIS FILE
 * ============================================================================
 * MOVC3/MOVC5/CMPC3/CMPC5/LOCC/SKPC/SCANC/SPANC are, architecturally, RESTARTABLE: SIMH sets
 * PSL<FPD> before the copy/scan loop and packs a resume state (fill/mask character, delta-PC,
 * remaining length) into R0 so that if the loop is interrupted (page fault, asynchronous
 * interrupt), the NEXT sim_instr() top-of-loop iteration re-enters with PSL<FPD> already set,
 * skips specifier re-resolution (decode.js's `decode(fFPD)` already has this branch built and
 * ready -- see its file header), and resumes via `SETPC(fault_PC + STR_GETDPC(R[0]))`.
 *
 * `fault_PC` -- the PC at the START of the current instruction, before decode -- is a variable
 * `sim_instr()`'s own top-of-loop maintains.  No item owns that loop yet (this file's dispatcher
 * was told as much, and this port confirms it firsthand): there is no fault_PC anywhere in this
 * codebase for this file to read.  Consequences, scoped precisely:
 *
 *   - The FORWARD path (PSL<FPD> clear at entry) is fully, faithfully ported below, including
 *     setting PSL<FPD> before the loop and clearing it after -- exactly SIMH's structure.  Every
 *     handler here runs its loop to completion in one synchronous call (JavaScript has no
 *     equivalent of SIMH's sim_interval yielding mid-instruction), so for any state this file's
 *     own randomized generator produces -- which, like every sibling differential in this
 *     project, never manufactures an execution-time fault -- PSL<FPD> is set and cleared inside
 *     the SAME call and is never externally observable.  This is the same "execution-time faults
 *     are out of scope" boundary cpu.js's and control.js's own headers already draw.
 *   - The RESUME path (PSL<FPD> set at entry) is structurally present -- reads STR_GETDPC/
 *     STR_GETCHR from R0, as SIMH does -- but has no correct fault_PC to add the delta to, so it
 *     leaves PC exactly where it was deposited rather than computing SIMH's resumed value.  This
 *     is a real, documented gap, not a silent one: tests/strqdiff.js's EHKAA phase detects any
 *     trace instance whose PRE-state already has PSL<FPD> set (a genuine mid-string-op resume in
 *     real EHKAA execution) and SKIPS it -- counted, reported by name, never silently dropped --
 *     using the exact same "did a trap or interrupt intervene" discipline intdiff.js's EHKAA
 *     phase already established for its own domain.  Fixing this for real needs fault_PC, i.e.
 *     needs the CPU loop.
 *
 * ============================================================================
 * INTERLOCKED QUEUE INSTRUCTIONS -- WHY NO INTERLOCK CODE IS HERE
 * ============================================================================
 * INSQHI/INSQTI/REMQHI/REMQTI exist on real hardware to let multiple CPUs (or a CPU and a
 * mid-interrupt copy of itself) mutate a queue without racing.  SIMH's own port has no interlock
 * either -- Test()-then-Write() is not atomic in C -- because a single-threaded simulator has no
 * concurrent access to race against.  The same is true here, more so: JavaScript is single-
 * threaded and this execute() call runs to completion before anything else touches `cpu`.  The
 * queue-header busy bit (bit 0 of the header longword) is still read/set/cleared exactly as SIMH
 * does, because it is architecturally VISIBLE state (a real OS checks it), not because this port
 * needs it for correctness.
 *
 * ============================================================================
 * DEFERRED TRAP -- INDEX's SET_TRAP(TRAP_SUBSCR)
 * ============================================================================
 * A subscript out of [low,high] does not fault immediately -- vax_cpu.c requests a trap
 * (TRAP_SUBSCR) that SIMH's dispatcher delivers at the top of the NEXT instruction, exactly like
 * the integer-overflow trap cpu.js's header already excludes from single-instruction grading for
 * the identical reason: a one-instruction register-file-plus-PSL comparison cannot observe a
 * dispatch that has not happened yet.  Ported anyway, following fpa.js's existing (also untested
 * against SIMH, also forward-compatible) `cpu.trpirq` convention, so a future CPU-loop owner has
 * something to drain.
 *
 * ============================================================================
 * ADDRESS / DATA CONVENTION -- see defines.js and decode.js's file header
 * ============================================================================
 * Same convention as every sibling execution module: `va`/pointer values are signed int32,
 * coerced with `>>> 0` only where MMUVAX requires it; longword data is signed int32.
 */

import { OP_MEM, VAXFAULT } from "./decode.js";
import { OPCODES } from "./drom.js";
import MMUVAX from "./mmu.js";

/* ------------------------------------------------------------------------------------------- *
 * Local constants -- each execution module in this project owns its own copy rather than         *
 * sharing across files (see control.js's/fpa.js's own local CC/SXT definitions).                  *
 * ------------------------------------------------------------------------------------------- */

const L_BYTE = 1, L_WORD = 2, L_LONG = 4;

const BSIGN = 0x80, WSIGN = 0x8000, LSIGN = 0x80000000 | 0;
const BMASK = 0xFF, WMASK = 0xFFFF;

const CC = { N: 0x8, Z: 0x4, V: 0x2, C: 0x1, MASK: 0xF };

const PSL_FPD    = 1 << 27;
const PSL_M_MODE = 0x3;
const PSL_V_PRV  = 22;
const PSL_M_IPL  = 0x1F;

/* vax_defs.h STR_* -- pack/unpack the FPD resume state into R0.  See the file header's "NO CPU
   LOOP OWNS fault_PC" note for why the delta-PC field this file PACKS is never correctly UNPACKED
   on resume; it is packed anyway for structural fidelity with SIMH and so the field is at least
   present (zeroed) rather than garbage. */
const STR_V_DPC  = 24, STR_M_DPC = 0xFF;
const STR_V_CHR  = 16, STR_M_CHR = 0xFF;
const STR_LNMASK = 0xFFFF;
function STR_GETDPC(x) { return (x >>> STR_V_DPC) & STR_M_DPC; }
function STR_GETCHR(x) { return (x >>> STR_V_CHR) & STR_M_CHR; }
function STR_PACK(fill, len) { return (((fill & STR_M_CHR) << STR_V_CHR) | (len & STR_LNMASK)) | 0; }

/* vax_defs.h TIR_V_TRAP/TRAP_SUBSCR -- see "DEFERRED TRAP" above. */
const TIR_V_TRAP   = 5;
const TRAP_SUBSCR  = 7 << TIR_V_TRAP;

function SXTB(x) { return (x & BSIGN) ? (x | ~BMASK) : (x & BMASK); }
function SXTW(x) { return (x & WSIGN) ? (x | ~WMASK) : (x & WMASK); }

function getCC(cpu) { return cpu.psl & CC.MASK; }
function setCC(cpu, cc) { cpu.psl = (cpu.psl & ~CC.MASK) | (cc & CC.MASK); }

/* CC_IIZZ_L -- vax_defs.h:737-739. */
function ccIIZZ_L(r) { return (r & LSIGN) ? CC.N : (r === 0) ? CC.Z : 0; }

/* CC_CMP_B/W/L -- vax_defs.h:813-826. */
function ccCmp_B(s1, s2) {
    let cc = (SXTB(s1) < SXTB(s2)) ? CC.N : (s1 === s2) ? CC.Z : 0;
    if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC.C;
    return cc;
}
function ccCmp_W(s1, s2) {
    let cc = (SXTW(s1) < SXTW(s2)) ? CC.N : (s1 === s2) ? CC.Z : 0;
    if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC.C;
    return cc;
}
function ccCmp_L(s1, s2) {
    let cc = (s1 < s2) ? CC.N : (s1 === s2) ? CC.Z : 0;
    if ((s1 >>> 0) < (s2 >>> 0)) cc |= CC.C;
    return cc;
}

/**
 * store2(cpu, opnd, j, value) -- the `w.bwlqo` two-slot write convention (decode.js contract
 * table): opnd[j] is OP_MEM or a register number, opnd[j+1] is the memory address (only valid
 * when opnd[j] === OP_MEM).
 *
 * @param {Object} cpu
 * @param {Int32Array|Array} opnd
 * @param {number} j
 * @param {number} value
 */
function store2(cpu, opnd, j, value)
{
    if (opnd[j] !== OP_MEM) {
        cpu.regs[opnd[j]] = value | 0;
    } else {
        cpu.mmu.writeData(opnd[j + 1], value | 0, L_LONG, cpu.accW());
    }
}

/* ------------------------------------------------------------------------------------------- *
 * NOP -- vax_cpu.c: `case NOP: break;`.  No operands (drom.js: 0 specifiers).                     *
 * ------------------------------------------------------------------------------------------- */

function opNop(cpu, opnd) {}

/* ------------------------------------------------------------------------------------------- *
 * INDEX -- vax_cpu.c:2704-2708.                                                                    *
 * opnd[0..4] = subscript.rl, low.rl, high.rl, size.rl, indexin.rl; opnd[5:6] = result.wl           *
 * ------------------------------------------------------------------------------------------- */

function opIndex(cpu, opnd)
{
    let subscript = opnd[0], low = opnd[1], high = opnd[2], size = opnd[3], indexin = opnd[4];
    if (subscript < low || subscript > high) {
        /* SET_TRAP(TRAP_SUBSCR) -- deferred, see file header. */
        cpu.trpirq = (cpu.trpirq & PSL_M_IPL) | TRAP_SUBSCR;
    }
    let r = Math.imul((subscript + indexin) | 0, size);
    store2(cpu, opnd, 5, r);
    setCC(cpu, ccIIZZ_L(r));
}

/* ------------------------------------------------------------------------------------------- *
 * PROBER, PROBEW -- vax_cpu1.c op_probe.  opnd[0]=mode.rb, opnd[1]=length.rw, opnd[2]=base.ab     *
 * ------------------------------------------------------------------------------------------- */

function opProbe(cpu, opnd, rw)
{
    let mode = opnd[0] & PSL_M_MODE;
    let length = opnd[1];
    let ba = opnd[2];
    let prv = (cpu.psl >>> PSL_V_PRV) & PSL_M_MODE;
    if (prv > mode) mode = prv;
    let acc = rw ? MMUVAX.accWrite(mode) : MMUVAX.accRead(mode);

    let stat = { code: 0 };
    let PR = MMUVAX.PR;

    /* op_probe() itself returns CC_Z on failure or 0 on success; the CALLER (vax_cpu.c:
       `cc = (cc & CC_C) | op_probe(opnd, opc & 1);`) is the one that preserves the incoming
       carry bit.  Ported as one step: preserved C, plus Z only on failure. */
    cpu.mmu.test(ba, acc, stat);
    if (stat.code === PR.PTNV) cpu.fault(VAXFAULT.TNV);
    if (stat.code !== PR.TNV && stat.code !== PR.OK) { setCC(cpu, (getCC(cpu) & CC.C) | CC.Z); return; }

    cpu.mmu.test((ba + length - 1) | 0, acc, stat);
    if (stat.code === PR.PTNV) cpu.fault(VAXFAULT.TNV);
    if (stat.code !== PR.TNV && stat.code !== PR.OK) { setCC(cpu, (getCC(cpu) & CC.C) | CC.Z); return; }

    setCC(cpu, getCC(cpu) & CC.C);
}

/* ------------------------------------------------------------------------------------------- *
 * INSQUE, REMQUE -- vax_cpu1.c:506-563.                                                            *
 * ------------------------------------------------------------------------------------------- */

function opInsque(cpu, opnd)
{
    let p = opnd[1], e = opnd[0];
    let RA = cpu.accR(), WA = cpu.accW();
    let s = cpu.mmu.readData(p, L_LONG, WA);
    cpu.mmu.readData((s + 4) | 0, L_LONG, WA);
    cpu.mmu.readData((e + 4) | 0, L_LONG, WA);
    cpu.mmu.writeData(e, s, L_LONG, WA);
    cpu.mmu.writeData((e + 4) | 0, p, L_LONG, WA);
    cpu.mmu.writeData((s + 4) | 0, e, L_LONG, WA);
    cpu.mmu.writeData(p, e, L_LONG, WA);
    setCC(cpu, ccCmp_L(s, p));
}

function opRemque(cpu, opnd)
{
    let e = opnd[0];
    let RA = cpu.accR(), WA = cpu.accW();
    let s = cpu.mmu.readData(e, L_LONG, RA);
    let p = cpu.mmu.readData((e + 4) | 0, L_LONG, RA);
    let cc = ccCmp_L(s, p);
    if (e !== p) {
        cpu.mmu.readData((s + 4) | 0, L_LONG, WA);
        if (opnd[1] === OP_MEM) cpu.mmu.readData(opnd[2], L_LONG, WA);
        cpu.mmu.writeData(p, s, L_LONG, WA);
        cpu.mmu.writeData((s + 4) | 0, p, L_LONG, WA);
    } else {
        cc |= CC.V;
    }
    store2(cpu, opnd, 1, e);
    setCC(cpu, cc);
}

/* ------------------------------------------------------------------------------------------- *
 * INSQHI, INSQTI -- vax_cpu1.c:598-663.  opnd[0]=entry.ab, opnd[1]=header.aq.                      *
 * See the file header's "INTERLOCKED QUEUE INSTRUCTIONS" note: the interlock bit is set/cleared    *
 * for architectural visibility, not because this single-threaded port needs mutual exclusion.       *
 * ------------------------------------------------------------------------------------------- */

function opInsqhi(cpu, opnd)
{
    let h = opnd[1], d = opnd[0];
    let WA = cpu.accW();
    if (h === d || ((h | d) & 7)) cpu.fault(VAXFAULT.RESOP);
    cpu.mmu.readData(d, L_BYTE, WA);
    let a = cpu.mmu.readData(h, L_LONG, WA);
    if (a & 6) cpu.fault(VAXFAULT.RESOP);
    if (a & 1) { setCC(cpu, CC.C); return; }
    cpu.mmu.writeData(h, (a | 1) | 0, L_LONG, WA);
    a = (a + h) | 0;
    let stat = { code: 0 };
    if (cpu.mmu.test(a, WA, stat) < 0) cpu.mmu.writeData(h, (a - h) | 0, L_LONG, WA);
    cpu.mmu.writeData((a + 4) | 0, (d - a) | 0, L_LONG, WA);
    cpu.mmu.writeData(d, (a - d) | 0, L_LONG, WA);
    cpu.mmu.writeData((d + 4) | 0, (h - d) | 0, L_LONG, WA);
    cpu.mmu.writeData(h, (d - h) | 0, L_LONG, WA);
    setCC(cpu, (a === h) ? CC.Z : 0);
}

function opInsqti(cpu, opnd)
{
    let h = opnd[1], d = opnd[0];
    let WA = cpu.accW(), RA = cpu.accR();
    if (h === d || ((h | d) & 7)) cpu.fault(VAXFAULT.RESOP);
    cpu.mmu.readData(d, L_BYTE, WA);
    let a = cpu.mmu.readData(h, L_LONG, WA);
    if (a === 0) { opInsqhi(cpu, opnd); return; }
    if (a & 6) cpu.fault(VAXFAULT.RESOP);
    if (a & 1) { setCC(cpu, CC.C); return; }
    cpu.mmu.writeData(h, (a | 1) | 0, L_LONG, WA);
    let c = (cpu.mmu.readData((h + 4) | 0, L_LONG, RA) + h) | 0;
    if (c & 7) {
        cpu.mmu.writeData(h, a, L_LONG, WA);
        cpu.fault(VAXFAULT.RESOP);
    }
    let stat = { code: 0 };
    if (cpu.mmu.test(c, WA, stat) < 0) cpu.mmu.writeData(h, a, L_LONG, WA);
    cpu.mmu.writeData(c, (d - c) | 0, L_LONG, WA);
    cpu.mmu.writeData(d, (h - d) | 0, L_LONG, WA);
    cpu.mmu.writeData((d + 4) | 0, (c - d) | 0, L_LONG, WA);
    cpu.mmu.writeData((h + 4) | 0, (d - h) | 0, L_LONG, WA);
    cpu.mmu.writeData(h, a, L_LONG, WA);
    setCC(cpu, 0);
}

/* ------------------------------------------------------------------------------------------- *
 * REMQHI, REMQTI -- vax_cpu1.c:683-762.  opnd[0]=header.aq, opnd[1:2]=dest.wl.                     *
 * ------------------------------------------------------------------------------------------- */

function opRemqhi(cpu, opnd)
{
    let h = opnd[0];
    let WA = cpu.accW(), RA = cpu.accR();
    if (h & 7) cpu.fault(VAXFAULT.RESOP);
    if (opnd[1] === OP_MEM) {
        if (h === opnd[2]) cpu.fault(VAXFAULT.RESOP);
        cpu.mmu.readData(opnd[2], L_LONG, WA);
    }
    let ar = cpu.mmu.readData(h, L_LONG, WA);
    if (ar & 6) cpu.fault(VAXFAULT.RESOP);
    if (ar & 1) { setCC(cpu, CC.V | CC.C); return; }
    let a = (ar + h) | 0;
    let b = 0;
    if (ar) {
        cpu.mmu.writeData(h, (ar | 1) | 0, L_LONG, WA);
        let stat = { code: 0 };
        if (cpu.mmu.test(a, RA, stat) < 0) cpu.mmu.writeData(h, ar, L_LONG, WA);
        b = (cpu.mmu.readData(a, L_LONG, RA) + a) | 0;
        if (b & 7) {
            cpu.mmu.writeData(h, ar, L_LONG, WA);
            cpu.fault(VAXFAULT.RESOP);
        }
        if (cpu.mmu.test(b, WA, stat) < 0) cpu.mmu.writeData(h, ar, L_LONG, WA);
        cpu.mmu.writeData((b + 4) | 0, (h - b) | 0, L_LONG, WA);
        cpu.mmu.writeData(h, (b - h) | 0, L_LONG, WA);
    }
    store2(cpu, opnd, 1, a);
    if (ar === 0) { setCC(cpu, CC.Z | CC.V); return; }
    setCC(cpu, (b === h) ? CC.Z : 0);
}

function opRemqti(cpu, opnd)
{
    let h = opnd[0];
    let WA = cpu.accW(), RA = cpu.accR();
    if (h & 7) cpu.fault(VAXFAULT.RESOP);
    if (opnd[1] === OP_MEM) {
        if (h === opnd[2]) cpu.fault(VAXFAULT.RESOP);
        cpu.mmu.readData(opnd[2], L_LONG, WA);
    }
    let ar = cpu.mmu.readData(h, L_LONG, WA);
    if (ar & 6) cpu.fault(VAXFAULT.RESOP);
    if (ar & 1) { setCC(cpu, CC.V | CC.C); return; }
    let c;
    if (ar) {
        cpu.mmu.writeData(h, (ar | 1) | 0, L_LONG, WA);
        c = cpu.mmu.readData((h + 4) | 0, L_LONG, RA);
        if (ar === c) {
            cpu.mmu.writeData(h, ar, L_LONG, WA);
            opRemqhi(cpu, opnd);
            return;
        }
        if (c & 7) {
            cpu.mmu.writeData(h, ar, L_LONG, WA);
            cpu.fault(VAXFAULT.RESOP);
        }
        c = (c + h) | 0;
        let stat = { code: 0 };
        if (cpu.mmu.test((c + 4) | 0, RA, stat) < 0) cpu.mmu.writeData(h, ar, L_LONG, WA);
        let b = (cpu.mmu.readData((c + 4) | 0, L_LONG, RA) + c) | 0;
        if (b & 7) {
            cpu.mmu.writeData(h, ar, L_LONG, WA);
            cpu.fault(VAXFAULT.RESOP);
        }
        if (cpu.mmu.test(b, WA, stat) < 0) cpu.mmu.writeData(h, ar, L_LONG, WA);
        cpu.mmu.writeData(b, (h - b) | 0, L_LONG, WA);
        cpu.mmu.writeData((h + 4) | 0, (b - h) | 0, L_LONG, WA);
        cpu.mmu.writeData(h, ar, L_LONG, WA);
    } else {
        c = h;
    }
    store2(cpu, opnd, 1, c);
    if (ar === 0) { setCC(cpu, CC.Z | CC.V); return; }
    setCC(cpu, 0);
}

/* ------------------------------------------------------------------------------------------- *
 * MOVC3, MOVC5 -- vax_cpu1.c:807-937 (op_movc).                                                    *
 * MOVC3 opnd: [0]=len.rw [1]=src.ab [2]=dst.ab.  MOVC5 opnd: [0]=srclen.rw [1]=src.ab [2]=fill.rb   *
 * [3]=dstlen.rw [4]=dst.ab.                                                                          *
 * ------------------------------------------------------------------------------------------- */

const MVC_FRWD = 0, MVC_BACK = 1, MVC_FILL = 3, MVC_M_STATE = 3, MVC_V_CC = 2;
const LOOPLNT = [L_BYTE, L_LONG, L_BYTE];

function opMovc(cpu, opnd, movc5)
{
    let regs = cpu.regs;
    let RA = cpu.accR(), WA = cpu.accW();
    let fill, cc = 0;

    if (cpu.psl & PSL_FPD) {
        /* Resume path -- see file header's "NO CPU LOOP OWNS fault_PC" note: PC is left as
           deposited rather than recomputed, because fault_PC does not exist anywhere in this
           codebase yet. */
        fill = STR_GETCHR(regs[0]);
        regs[2] = regs[2] & STR_LNMASK;
        if (regs[4] > 0) regs[4] = regs[4] & STR_LNMASK;
    } else {
        regs[1] = opnd[1];
        if (movc5) {
            regs[2] = (opnd[0] < opnd[3]) ? opnd[0] : opnd[3];
            regs[3] = opnd[4];
            regs[4] = (opnd[3] - opnd[0]) | 0;
            fill = opnd[2];
            cc = ccCmp_W(opnd[0], opnd[3]);
        } else {
            regs[2] = opnd[0];
            regs[3] = opnd[2];
            regs[4] = 0;
            fill = 0;
            cc = CC.Z;
        }
        regs[0] = STR_PACK(fill, regs[2]);
        if (regs[2]) {
            if ((regs[1] >>> 0) < (regs[3] >>> 0)) {
                regs[1] = (regs[1] + regs[2]) | 0;
                regs[3] = (regs[3] + regs[2]) | 0;
                regs[5] = MVC_BACK;
            } else {
                regs[5] = MVC_FRWD;
            }
        } else {
            regs[5] = MVC_FILL;
        }
        regs[5] = (regs[5] | (cc << MVC_V_CC)) | 0;
        cpu.psl |= PSL_FPD;
    }

    let state = regs[5] & MVC_M_STATE;
    if (state !== MVC_FRWD && state !== MVC_BACK && state !== MVC_FILL) {
        cpu.fault(VAXFAULT.RESOP);
    }
    if (state === MVC_FRWD || state === MVC_BACK) {
        let forward = (state === MVC_FRWD);
        let mlnt = [0, 0, 0];
        mlnt[0] = forward ? ((4 - regs[3]) & 3) : (regs[3] & 3);
        if (mlnt[0] > regs[2]) mlnt[0] = regs[2];
        mlnt[1] = (regs[2] - mlnt[0]) & ~3;
        mlnt[2] = regs[2] - mlnt[0] - mlnt[1];
        for (let i = 0; i < 3; i++) {
            let lnt = LOOPLNT[i];
            for (let j = 0; j < mlnt[i]; j += lnt) {
                if (forward) {
                    let wd = cpu.mmu.readData(regs[1], lnt, RA);
                    cpu.mmu.writeData(regs[3], wd, lnt, WA);
                    regs[1] = (regs[1] + lnt) | 0;
                    regs[3] = (regs[3] + lnt) | 0;
                } else {
                    let wd = cpu.mmu.readData((regs[1] - lnt) | 0, lnt, RA);
                    cpu.mmu.writeData((regs[3] - lnt) | 0, wd, lnt, WA);
                    regs[1] = (regs[1] - lnt) | 0;
                    regs[3] = (regs[3] - lnt) | 0;
                }
                regs[2] = (regs[2] - lnt) | 0;
            }
        }
        if (!forward) {
            regs[1] = (regs[1] + (regs[0] & STR_LNMASK)) | 0;
            regs[3] = (regs[3] + (regs[0] & STR_LNMASK)) | 0;
        }
    }

    /* FILL -- entered either by falling through from FRWD/BACK, or directly when state ===
       MVC_FILL. */
    if (regs[4] > 0) {
        regs[5] = (regs[5] | MVC_FILL) | 0;
        let mlnt = [0, 0, 0];
        mlnt[0] = (4 - regs[3]) & 3;
        if (mlnt[0] > regs[4]) mlnt[0] = regs[4];
        mlnt[1] = (regs[4] - mlnt[0]) & ~3;
        mlnt[2] = regs[4] - mlnt[0] - mlnt[1];
        for (let i = 0; i < 3; i++) {
            let lnt = LOOPLNT[i];
            let fillLnt = fill & BMASK;
            if (lnt === L_LONG) fillLnt = ((fillLnt << 24) | (fillLnt << 16) | (fillLnt << 8) | fillLnt) | 0;
            for (let j = 0; j < mlnt[i]; j += lnt) {
                cpu.mmu.writeData(regs[3], fillLnt, lnt, WA);
                regs[3] = (regs[3] + lnt) | 0;
                regs[4] = (regs[4] - lnt) | 0;
            }
        }
    }

    cpu.psl &= ~PSL_FPD;
    cc = (regs[5] >>> MVC_V_CC) & CC.MASK;
    regs[0] = (-regs[4]) | 0;
    regs[2] = regs[4] = regs[5] = 0;
    setCC(cpu, cc);
}

/* ------------------------------------------------------------------------------------------- *
 * CMPC3, CMPC5 -- vax_cpu1.c:953-1011 (op_cmpc).                                                   *
 * CMPC3 opnd: [0]=len.rw [1]=src1.ab [2]=src2.ab.  CMPC5 opnd: [0]=src1len.rw [1]=src1.ab           *
 * [2]=fill.rb [3]=src2len.rw [4]=src2.ab.                                                            *
 * ------------------------------------------------------------------------------------------- */

function opCmpc(cpu, opnd, cmpc5)
{
    let regs = cpu.regs;
    let RA = cpu.accR();
    let fill;

    if (cpu.psl & PSL_FPD) {
        fill = STR_GETCHR(regs[0]);
    } else {
        regs[1] = opnd[1];
        if (cmpc5) {
            regs[2] = opnd[3];
            regs[3] = opnd[4];
            fill = opnd[2];
        } else {
            regs[2] = opnd[0];
            regs[3] = opnd[2];
            fill = 0;
        }
        regs[0] = STR_PACK(fill, opnd[0]);
        cpu.psl |= PSL_FPD;
    }
    regs[2] = regs[2] & STR_LNMASK;

    let s1 = 0, s2 = 0;
    while (((regs[0] | regs[2]) & STR_LNMASK) !== 0) {
        if (regs[0] & STR_LNMASK) s1 = cpu.mmu.readData(regs[1], L_BYTE, RA);
        else s1 = fill;
        if (regs[2]) s2 = cpu.mmu.readData(regs[3], L_BYTE, RA);
        else s2 = fill;
        if (s1 !== s2) break;
        if (regs[0] & STR_LNMASK) {
            regs[0] = (regs[0] & ~STR_LNMASK) | ((regs[0] - 1) & STR_LNMASK);
            regs[1] = (regs[1] + 1) | 0;
        }
        if (regs[2]) {
            regs[2] = (regs[2] - 1) & STR_LNMASK;
            regs[3] = (regs[3] + 1) | 0;
        }
    }
    cpu.psl &= ~PSL_FPD;
    setCC(cpu, ccCmp_B(s1, s2));
    regs[0] = regs[0] & STR_LNMASK;
}

/* ------------------------------------------------------------------------------------------- *
 * LOCC, SKPC -- vax_cpu1.c:1013-1036 (op_locskp).  opnd[0]=match.rb [1]=len.rw [2]=src.ab.         *
 * ------------------------------------------------------------------------------------------- */

function opLocskp(cpu, opnd, skpc)
{
    let regs = cpu.regs;
    let RA = cpu.accR();
    let match;

    if (cpu.psl & PSL_FPD) {
        match = STR_GETCHR(regs[0]);
    } else {
        match = opnd[0];
        regs[0] = STR_PACK(match, opnd[1]);
        regs[1] = opnd[2];
        cpu.psl |= PSL_FPD;
    }
    while ((regs[0] & STR_LNMASK) !== 0) {
        let c = cpu.mmu.readData(regs[1], L_BYTE, RA);
        if ((c === match) !== !!skpc) break;
        regs[0] = (regs[0] & ~STR_LNMASK) | ((regs[0] - 1) & STR_LNMASK);
        regs[1] = (regs[1] + 1) | 0;
    }
    cpu.psl &= ~PSL_FPD;
    regs[0] = regs[0] & STR_LNMASK;
    setCC(cpu, regs[0] ? 0 : CC.Z);
}

/* ------------------------------------------------------------------------------------------- *
 * SCANC, SPANC -- vax_cpu1.c:1053-1077 (op_scnspn).                                                *
 * opnd[0]=len.rw [1]=src.ab [2]=table.ab [3]=mask.rb.                                               *
 * ------------------------------------------------------------------------------------------- */

function opScnspn(cpu, opnd, spanc)
{
    let regs = cpu.regs;
    let RA = cpu.accR();
    let mask;

    if (cpu.psl & PSL_FPD) {
        mask = STR_GETCHR(regs[0]);
    } else {
        regs[1] = opnd[1];
        regs[3] = opnd[2];
        mask = opnd[3];
        regs[0] = STR_PACK(mask, opnd[0]);
        cpu.psl |= PSL_FPD;
    }
    while ((regs[0] & STR_LNMASK) !== 0) {
        let c = cpu.mmu.readData(regs[1], L_BYTE, RA);
        let t = cpu.mmu.readData((regs[3] + c) | 0, L_BYTE, RA);
        if (((t & mask) !== 0) !== !!spanc) break;
        regs[0] = (regs[0] & ~STR_LNMASK) | ((regs[0] - 1) & STR_LNMASK);
        regs[1] = (regs[1] + 1) | 0;
    }
    cpu.psl &= ~PSL_FPD;
    regs[0] = regs[0] & STR_LNMASK;
    regs[2] = 0;
    setCC(cpu, regs[0] ? 0 : CC.Z);
}

/* ------------------------------------------------------------------------------------------- *
 * DISPATCH -- built positionally from drom.js's OPCODES, exactly cpu.js's convention.              *
 * ------------------------------------------------------------------------------------------- */

const H = {};
H.NOP    = (cpu, opnd) => opNop(cpu, opnd);
H.INDEX  = (cpu, opnd) => opIndex(cpu, opnd);
H.PROBER = (cpu, opnd) => opProbe(cpu, opnd, 0);
H.PROBEW = (cpu, opnd) => opProbe(cpu, opnd, 1);
H.INSQUE = (cpu, opnd) => opInsque(cpu, opnd);
H.REMQUE = (cpu, opnd) => opRemque(cpu, opnd);
H.INSQHI = (cpu, opnd) => opInsqhi(cpu, opnd);
H.INSQTI = (cpu, opnd) => opInsqti(cpu, opnd);
H.REMQHI = (cpu, opnd) => opRemqhi(cpu, opnd);
H.REMQTI = (cpu, opnd) => opRemqti(cpu, opnd);
H.MOVC3  = (cpu, opnd) => opMovc(cpu, opnd, 0);
H.MOVC5  = (cpu, opnd) => opMovc(cpu, opnd, 1);
H.CMPC3  = (cpu, opnd) => opCmpc(cpu, opnd, 0);
H.CMPC5  = (cpu, opnd) => opCmpc(cpu, opnd, 1);
H.LOCC   = (cpu, opnd) => opLocskp(cpu, opnd, 0);
H.SKPC   = (cpu, opnd) => opLocskp(cpu, opnd, 1);
H.SCANC  = (cpu, opnd) => opScnspn(cpu, opnd, 0);
H.SPANC  = (cpu, opnd) => opScnspn(cpu, opnd, 1);

const STRQ_OPCODES = {};
for (let name in H) {
    let opc = OPCODES.indexOf(name);
    if (opc < 0) throw new Error(`strq.js: opcode mnemonic "${name}" not found in drom.js OPCODES`);
    STRQ_OPCODES[opc] = H[name];
}

/** Mnemonics this file wires up, for a test harness to enumerate without re-deriving the list. */
const IMPLEMENTED = Object.keys(H);

/**
 * executeStrq(opc, decoder, cpu)
 *
 * @param {number} opc
 * @param {VAXDecoder} decoder
 * @param {Object} cpu
 * @returns {boolean} true if this module handled the opcode
 */
function executeStrq(opc, decoder, cpu)
{
    let fn = STRQ_OPCODES[opc];
    if (!fn) return false;
    fn(cpu, decoder.opnd);
    return true;
}

export default executeStrq;
export { executeStrq, STRQ_OPCODES, IMPLEMENTED, H as HANDLERS, CC, getCC, setCC, PSL_FPD, TRAP_SUBSCR };
