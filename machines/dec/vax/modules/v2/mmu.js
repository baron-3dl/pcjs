/**
 * @fileoverview VAX memory management -- virtual-to-physical translation and the software TLB
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
 *
 * ============================================================================
 * THE SIGNED-int32 HAZARD LIVES HERE.  READ THIS BEFORE CHANGING ANY LINE.
 * ============================================================================
 * defines.js states the convention for the whole machine; this is the module it was written for.
 * The physical bus is 30 bits wide, so `& VAX.PAMASK` rescues almost any sloppiness there.  A VAX
 * VIRTUAL address is a full 32 bits and the region select is bits <31:30> -- there is no mask that
 * makes an S0 address non-negative, and getting a shift wrong here does not throw, does not
 * produce an obviously bogus value, and does not fault.  It silently translates through the WRONG
 * PAGE TABLE.
 *
 * The one line where that actually bites is the page-table index in fill():
 *
 *      let ptidx = (va >>> 7) & ~0x03;     // CORRECT
 *      let ptidx = (va >> 7)  & ~0x03;     // SILENTLY WRONG for every S0 and P1 address
 *
 * `& ~0x03` does NOT rescue `>>`: it clears two bits at the bottom, while a signed shift
 * corrupts the top.  For va = 0x80001000 the correct index is 0x01000020 and the broken one is
 * -16777184, which (a) passes the `ptidx >= d_slr` length check that should have been evaluated
 * against a large positive number, and (b) addresses a completely different PTE once the sum is
 * masked with PAMASK.  Both page tables are real memory, so the wrong PTE is a perfectly
 * plausible-looking one.  mmudiff.js `--selfcheck` injects exactly this mutation.
 *
 * By contrast, several places that LOOK like the same hazard are provably safe, and are written
 * the way SIMH writes them rather than being defensively coerced:
 *
 *      va & VA_S0, va & VA_P1      masks -- ToInt32 first, so sign is irrelevant
 *      VA_GETVPN(x) = (x >>> 9) & VA_M_VPN
 *                                  the 22-bit mask discards the sign extension: `>>` and `>>>`
 *                                  are IDENTICAL here.  `>>>` is used anyway, because rule 3 in
 *                                  defines.js asks a reviewer to treat `>>` on an address as a
 *                                  defect without having to think about which case this is.
 *      PTE_GETACC(x) = (x >>> 27) & PTE_M_ACC
 *                                  same: the 4-bit mask discards the sign extension.
 *      (d_sbr + ptidx) & PAMASK    sum is a JS double, ToInt32 wraps it exactly as C's uint32
 *                                  arithmetic does, and the mask then clamps it.
 *
 * Sums that are NOT masked (`d_p0br + ptidx`, `d_p1br + ptidx`) are coerced with `| 0` at the
 * point of computation, because C computes them in 32 bits and the result is then tested against
 * VA_S0 -- carrying a JS double past 2^32 there would change the answer.
 * ============================================================================
 *
 * DESIGN CONSTRAINT: NO BLOCK-PER-PAGE PAGING
 * -------------------------------------------
 * PCx86 maps the 386 page table onto the Bus block array (`enablePageBlocks`), which works only
 * because an x86 page and a PCjs block are both 4KB.  A VAX page is 512 bytes; a block-per-page
 * array over the 4GB virtual space would be 8M entries.  So this module follows PDPjs's model --
 * explicit translation on every access -- accelerated by a direct-mapped software TLB that
 * mirrors SIMH's split `stlb[]`/`ptlb[]` exactly, including its size (4096 entries each) and its
 * index (VPN<11:0>).  The split matters: the same TB index can hold a system and a process entry
 * simultaneously, and TBIA/TBIS/MTPR-to-a-Px-register flush them on different schedules.
 *
 * WHAT THIS MODULE DOES NOT DO
 * ----------------------------
 *   - It does not dispatch exceptions.  A translation fault throws a VAXFault carrying the ACV or
 *     TNV SCB offset plus SIMH's p1/p2 fault parameters; pushing them on the appropriate stack is
 *     the CPU's exception item.
 *   - It does not decode MTPR/MFPR.  It publishes the register mutators the IPR item needs
 *     (setP0BR() and friends), each of which flushes exactly what SIMH's MTPR handler flushes.
 *   - It does not turn a non-existent-memory fault into a machine check.  The Bus records the
 *     fault (bus.checkFault()); mapping it onto SCB vector 0x04 belongs with the CPU.
 *   - It does not handle quadword or octaword references, exactly as SIMH's Read/Write do not.
 *     A consumer decomposes those into longwords and, when it must know up front whether BOTH
 *     halves are accessible, probes with test() first -- see PROBE/WRITE_Q below.
 *
 * THE PROBE SEAM (`test`) -- READ THIS IF YOU ARE WAITING ON THIS ITEM
 * -------------------------------------------------------------------
 * `test(va, acc, stat)` is Open SIMH's `Test()`: it returns the physical address for `va` with no
 * side effect other than filling the TB, and reports a failure through `stat` INSTEAD of faulting.
 * That is the primitive three separate consumers need:
 *
 *   - a quadword or octaword write that must not perform a partial store when its second page is
 *     inaccessible (SIMH's WRITE_Q, vax_cpu.c:222, probes `va + 7` then `va`);
 *   - PROBER/PROBEW, which are defined in terms of it;
 *   - the debugger's "show virtual" equivalent, which must not perturb the machine.
 *
 * Call it as `let stat = {code: 0}; let pa = mmu.test(va, acc, stat);` -- `pa < 0` means the
 * access is not permitted and `stat.code` is one of MMUVAX.PR.*.  Passing `stat` as null makes it
 * fault instead, which is what an instruction body wants.
 */

import { VAX } from "./defines.js";
/*
 * VAXFault is defined in decode.js rather than here because the decoder needed it first, and there
 * must be exactly ONE fault class: the CPU's catch handler has to treat a reserved-addressing
 * fault thrown during operand resolution and an ACV thrown by the translation of that same
 * operand identically, and it cannot do that if they are different types.  decode.js does not
 * import this module, so there is no cycle.
 */
import { VAXFault, VAXFAULT } from "./decode.js";

/*
 * Virtual address geometry (vax_defs.h:257-270).  A VAX page is 512 bytes.
 */
const VA_N_OFF      = 9;
const VA_PAGSIZE    = 1 << VA_N_OFF;            // 512
const VA_M_OFF      = VA_PAGSIZE - 1;           // 0x1FF
const VA_V_VPN      = VA_N_OFF;
const VA_M_VPN      = 0x3FFFFF;                 // 22 bits
const VA_S0         = 0x80000000;               // region select <31> -- system space
const VA_P1         = 0x40000000;               // region select <30> -- P1 (with <31> clear)
const VA_N_TBI      = 12;
const VA_TBSIZE     = 1 << VA_N_TBI;            // 4096 entries per TB, as on SIMH
const VA_M_TBI      = VA_TBSIZE - 1;

/*
 * Page table entry (vax_defs.h:274-281) and translation buffer entry (vax_defs.h:285-294).
 *
 * TLB_PFN is (PAWIDTH - VA_N_OFF) = 21 bits of page frame number, positioned at <29:9>, so a
 * translated physical address is always in 0..0x3FFFFFFF and therefore always non-negative.
 */
const PTE_V         = 0x80000000;               // valid
const PTE_V_ACC     = 27;
const PTE_M_ACC     = 0xF;
const PTE_M         = 1 << 26;                  // modified

const TLB_V_WACC    = 4;
const TLB_WACC      = 0xF << TLB_V_WACC;        // 0xF0
const TLB_M         = 1 << 8;
const TLB_N_PFN     = VAX.PAWIDTH - VA_N_OFF;   // 21
const TLB_PFN       = ((1 << TLB_N_PFN) - 1) << VA_V_VPN;   // 0x3FFFFE00

/*
 * Register masks (vax_defs.h:622-623).  ML_PXBR_TEST/ML_LR_TEST/ML_SBR_TEST are all EMPTY on the
 * KA655 (vaxmod_defs.h:249-252) -- only the 780 and the 8600 fault on a malformed base/length
 * register -- so this module masks and does not validate, exactly as the target hardware does.
 */
const BR_MASK       = 0xFFFFFFFC | 0;
const LR_MASK       = 0x003FFFFF;

/*
 * Access-mode encoding.  An access code is a BITMASK, not a mode number: bit <mode> set means
 * "read permitted in that mode", bit <mode+4> means "write permitted".  ACC_MASK(m) is SIMH's
 * `1 << m`, RA is the read form and WA the write form (vax_defs.h:716-721).
 */
const KERN = 0, EXEC = 1, SUPV = 2, USER = 3;

/*
 * PTE protection code -> TLB access mask (vax_mmu.c:62-84).  Transcribed from SIMH's cvtacc[]
 * with its TLB_ACCR/TLB_ACCW macros spelled out, so the transcription can be diffed by eye.
 */
const ACCR = (m) => 1 << m;
const ACCW = (m) => (1 << m) << TLB_V_WACC;
const CVTACC = new Int32Array([
    0,
    0,
    ACCW(KERN) + ACCR(KERN),
    ACCR(KERN),
    ACCW(KERN) + ACCW(EXEC) + ACCW(SUPV) + ACCW(USER) +
        ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV) + ACCR(USER),
    ACCW(KERN) + ACCW(EXEC) + ACCR(KERN) + ACCR(EXEC),
    ACCW(KERN) + ACCR(KERN) + ACCR(EXEC),
    ACCR(KERN) + ACCR(EXEC),
    ACCW(KERN) + ACCW(EXEC) + ACCW(SUPV) +
        ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV),
    ACCW(KERN) + ACCW(EXEC) +
        ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV),
    ACCW(KERN) + ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV),
    ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV),
    ACCW(KERN) + ACCW(EXEC) + ACCW(SUPV) +
        ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV) + ACCR(USER),
    ACCW(KERN) + ACCW(EXEC) +
        ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV) + ACCR(USER),
    ACCW(KERN) + ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV) + ACCR(USER),
    ACCR(KERN) + ACCR(EXEC) + ACCR(SUPV) + ACCR(USER)
]);

/*
 * Probe results / memory-management fault codes (vax_defs.h:569-577).
 *
 * PR_TNV is 4, i.e. a BIT: `code & PR_TNV` is what distinguishes a translation-not-valid abort
 * (SCB 0x24) from an access-control-violation abort (SCB 0x20), and it is true for PR_TNV and
 * PR_PTNV only.  Do not renumber these.
 */
const PR_ACV        = 0;
const PR_LNV        = 1;
const PR_PACV       = 2;                        // VAX-780 only; unreachable on a CVAX
const PR_PLNV       = 3;
const PR_TNV        = 4;
const PR_PTNV       = 6;
const PR_OK         = 7;

/*
 * insert[] (vax_mmu.h:66-68) -- the low-byte masks an unaligned fragment is trimmed with.
 */
const INSERT = new Int32Array([0x00000000, 0x000000FF, 0x0000FFFF, 0x00FFFFFF]);

const L_BYTE = 1, L_WORD = 2, L_LONG = 4;

/**
 * @class MMUVAX
 * @property {BusVAX} bus
 * @property {number} mapen      memory management enable (0 or 1)
 * @property {Int32Array} stlbTag  system TB tags (VPN, or -1 when invalid)
 * @property {Int32Array} stlbPte  system TB entries (access mask | TLB_M | PFN)
 * @property {Int32Array} ptlbTag  process TB tags
 * @property {Int32Array} ptlbPte  process TB entries
 */
export default class MMUVAX {
    /**
     * MMUVAX(bus)
     *
     * @param {BusVAX} [bus]
     */
    constructor(bus)
    {
        this.bus = bus || null;

        /*
         * Architectural map registers, as MTPR leaves them.
         */
        this.p0br = 0;
        this.p0lr = 0;
        this.p1br = 0;
        this.p1lr = 0;
        this.sbr = 0;
        this.slr = 0;
        this.mapen = 0;

        /*
         * Dynamic copies (vax_mmu.c:56-58).  set_map_reg() folds the region bias into the base and
         * length registers once, so fill() can index all three page tables with the same
         * `ptidx = (va >>> 7) & ~3` without re-deriving the region.  These are what fill() reads;
         * the architectural registers above are what MFPR returns.
         */
        this.d_p0br = 0;
        this.d_p0lr = 0;
        this.d_p1br = 0;
        this.d_p1lr = 0;
        this.d_sbr = 0;
        this.d_slr = 0;

        /*
         * The split translation buffer.  Direct-mapped, VA_TBSIZE entries each, indexed by
         * VPN<11:0>, tagged with the full 22-bit VPN.  Two parallel Int32Arrays rather than an
         * array of {tag, pte} objects: SIMH's TLBENT is two int32s, and a typed-array pair keeps
         * the whole TB in two flat allocations instead of 8192 heap objects.
         */
        this.stlbTag = new Int32Array(VA_TBSIZE);
        this.stlbPte = new Int32Array(VA_TBSIZE);
        this.ptlbTag = new Int32Array(VA_TBSIZE);
        this.ptlbPte = new Int32Array(VA_TBSIZE);

        /*
         * Fault parameters, mirroring SIMH's globals p1/p2 (vax_cpu.c:260).  The CPU's exception
         * dispatcher pushes these; they are also carried on the thrown VAXFault, which is the
         * interface a JS consumer should prefer.
         */
        this.p1 = 0;
        this.p2 = 0;

        /*
         * mchk_va (vax_cpu.c) -- the virtual address of the access in progress, recorded for the
         * machine-check parameter block.  Set on every Read/Write whether or not mapping is on.
         */
        this.mchkVA = 0;

        /*
         * The physical address(es) the last Read/Write/test resolved.  `pa1` is meaningful only
         * for an unaligned access, where it is the SECOND longword the access touches and is NOT
         * necessarily pa+4 -- that is the whole point of the cross-page path.  Both are diagnostic
         * (the debugger and mmudiff.js read them); nothing in the translation depends on them.
         */
        this.pa = 0;
        this.pa1 = 0;

        this.reset();
    }

    /**
     * setBus(bus)
     *
     * @this {MMUVAX}
     * @param {BusVAX} bus
     */
    setBus(bus)
    {
        this.bus = bus;
    }

    /**
     * reset()
     *
     * tlb_reset() (vax_mmu.c:283) invalidates both TBs by setting every tag AND every pte to -1.
     * A tag of -1 can never match a VPN (which is 22 bits, so 0..0x3FFFFF), and a pte of -1 has
     * every access bit set -- which is why SIMH must and does compare the tag as well as the
     * access bits.  The map registers are NOT cleared here: on real hardware they survive a
     * bus reset and are re-initialized by MTPR.
     *
     * @this {MMUVAX}
     */
    reset()
    {
        this.stlbTag.fill(-1);
        this.stlbPte.fill(-1);
        this.ptlbTag.fill(-1);
        this.ptlbPte.fill(-1);
    }

    /**
     * setMapReg()
     *
     * set_map_reg() (vax_mmu.c:206).  Folds each region's bias into its dynamic base/length copy:
     *
     *   ptidx for a virtual address is (va >>> 7) & ~3, i.e. 4 * VPN, and VPN carries the region
     *   bits.  So a P1 address contributes a constant 0x800000 (VA<30> >> 7) and an S0 address a
     *   constant 0x1000000 (VA<31> >> 7).  Subtracting those from the base registers, and adding
     *   them to the length registers, lets fill() use one index expression for all three regions.
     *
     * Must be called after ANY change to a base or length register.  The mutators below do it.
     *
     * @this {MMUVAX}
     */
    setMapReg()
    {
        this.d_p0br = this.p0br & ~0x03;
        this.d_p1br = (this.p1br - 0x800000) & ~0x03;       // VA<30> >> 7
        this.d_sbr  = (this.sbr - 0x1000000) & ~0x03;       // VA<31> >> 7
        this.d_p0lr = (this.p0lr << 2);
        this.d_p1lr = (this.p1lr << 2) + 0x800000;          // VA<30> >> 7
        this.d_slr  = (this.slr << 2) + 0x1000000;          // VA<31> >> 7
    }

    /**
     * zapTB(fSys)
     *
     * zap_tb() (vax_mmu.c:219).  Invalidates the process TB always, and the system TB as well when
     * fSys is true.  MTPR to P0BR/P0LR/P1BR/P1LR zaps process only; MTPR to SBR/SLR/MAPEN and
     * TBIA zap both.
     *
     * @this {MMUVAX}
     * @param {boolean|number} fSys
     */
    zapTB(fSys)
    {
        this.ptlbTag.fill(-1);
        this.ptlbPte.fill(-1);
        if (fSys) {
            this.stlbTag.fill(-1);
            this.stlbPte.fill(-1);
        }
    }

    /**
     * zapTBEnt(va)
     *
     * zap_tb_ent() (vax_mmu.c:233) -- MTPR to TBIS.  Invalidates the single direct-mapped entry
     * this virtual address maps to, in whichever of the two TBs its region selects.
     *
     * @this {MMUVAX}
     * @param {number} va
     */
    zapTBEnt(va)
    {
        let tbi = ((va >>> VA_V_VPN) & VA_M_VPN) & VA_M_TBI;
        if (va & VA_S0) {
            this.stlbTag[tbi] = this.stlbPte[tbi] = -1;
        } else {
            this.ptlbTag[tbi] = this.ptlbPte[tbi] = -1;
        }
    }

    /**
     * chkTBEnt(va)
     *
     * chk_tb_ent() (vax_mmu.c:245) -- MTPR to TBCHK, which sets PSL<V> when the entry is present.
     * Note that it tests the TAG ONLY: an entry whose access bits deny this mode still "hits".
     *
     * @this {MMUVAX}
     * @param {number} va
     * @returns {boolean}
     */
    chkTBEnt(va)
    {
        let vpn = (va >>> VA_V_VPN) & VA_M_VPN;
        let tbi = vpn & VA_M_TBI;
        return ((va & VA_S0)? this.stlbTag[tbi] : this.ptlbTag[tbi]) == vpn;
    }

    /**
     * setMAPEN(val)
     *
     * MTPR to MAPEN (vax_cpu1.c:1531).  Note the deliberate fall-through in SIMH: writing MAPEN
     * ALWAYS zaps the entire TB, whether it enables or disables mapping, and whether or not the
     * value changed.
     *
     * @this {MMUVAX}
     * @param {number} val
     */
    setMAPEN(val)
    {
        this.mapen = val & 1;
        this.zapTB(1);
    }

    /**
     * setP0BR(val)
     *
     * @this {MMUVAX}
     * @param {number} val
     */
    setP0BR(val)
    {
        this.p0br = val & BR_MASK;              // longword aligned
        this.zapTB(0);                          // process TB only
        this.setMapReg();
    }

    /**
     * setP0LR(val)
     *
     * @this {MMUVAX}
     * @param {number} val
     */
    setP0LR(val)
    {
        this.p0lr = val & LR_MASK;
        this.zapTB(0);
        this.setMapReg();
    }

    /**
     * setP1BR(val)
     *
     * @this {MMUVAX}
     * @param {number} val
     */
    setP1BR(val)
    {
        this.p1br = val & BR_MASK;
        this.zapTB(0);
        this.setMapReg();
    }

    /**
     * setP1LR(val)
     *
     * @this {MMUVAX}
     * @param {number} val
     */
    setP1LR(val)
    {
        this.p1lr = val & LR_MASK;
        this.zapTB(0);
        this.setMapReg();
    }

    /**
     * setSBR(val)
     *
     * @this {MMUVAX}
     * @param {number} val
     */
    setSBR(val)
    {
        this.sbr = val & BR_MASK;
        this.zapTB(1);                          // a system table move invalidates EVERYTHING
        this.setMapReg();
    }

    /**
     * setSLR(val)
     *
     * @this {MMUVAX}
     * @param {number} val
     */
    setSLR(val)
    {
        this.slr = val & LR_MASK;
        this.zapTB(1);
        this.setMapReg();
    }

    /**
     * mmErr(code, va, acc, stat)
     *
     * MM_ERR() (vax_mmu.c:130).  Two callers, two behaviors, and the difference is the whole
     * reason test() exists: when `stat` is supplied the error is REPORTED and fill() returns a
     * zero PTE, and when it is not, the error ABORTS -- SIMH longjmps, we throw.
     *
     * p1 is MM_PARAM(w, code): bit 2 set when the faulting access was a write, bits 1:0 the
     * fault code truncated to two bits (so PR_PTNV, 6, arrives as 2).  p2 is the virtual address.
     * Both are what the CPU pushes on the exception stack.
     *
     * @this {MMUVAX}
     * @param {number} code (PR_*)
     * @param {number} va
     * @param {number} acc
     * @param {Object|null} stat
     * @returns {number} 0 (the zero PTE), when stat was supplied
     * @throws {VAXFault} when stat was not
     */
    mmErr(code, va, acc, stat)
    {
        if (stat) {
            stat.code = code;
            return 0;
        }
        this.p1 = ((acc & TLB_WACC)? 4 : 0) | (code & 3);
        this.p2 = va;
        throw new VAXFault((code & PR_TNV)? VAXFAULT.TNV : VAXFAULT.ACV, this.p1, this.p2);
    }

    /**
     * fill(va, lnt, acc, stat)
     *
     * fill() (vax_mmu.c:136) -- the TB miss path, i.e. the actual page table walk.  Called on a
     * tag mismatch, on an access mismatch, or on a write to a page whose TB entry lacks TLB_M.
     * It loads the TB and returns the resulting entry; it does NOT return the physical address.
     *
     * The two-level walk is the part worth reading slowly.  A system (S0) page is one level: its
     * PTE lives at a PHYSICAL address, SBR + 4*VPN.  A process (P0/P1) page is two: its PTE lives
     * at a VIRTUAL address in S0 space, so that PTE address must itself be translated through the
     * system page table first -- which is why the system TB is consulted in the middle of a
     * process-space fill, and why zapping the system TB has to zap the process TB too.
     *
     * SIMH quirks preserved verbatim:
     *
     *   - the system-PTE index is `ptead >>> 7` with NO `& ~3`, unlike the first-level index.
     *     ptead is longword aligned, so bits <1:0> of the index come from ptead<8:7> and are
     *     non-zero for three quarters of all page-table pages.  It does not affect which longword
     *     is read (readLP() truncates, as SIMH's `M[pa >> 2]` does), but it DOES make the
     *     `ptidx >= d_slr` length check fire up to three longwords early.  Copied, not corrected.
     *   - the ACCESS check precedes the VALID check, so a PTE with protection code 0 reports ACV
     *     rather than TNV even though it is also invalid.  Software distinguishes them, so the
     *     order is architectural.
     *   - PR_PACV is unreachable: SIMH tests the system PTE's protection only `#if defined
     *     (VAX_780)`, and we are a CVAX.
     *
     * @this {MMUVAX}
     * @param {number} va
     * @param {number} lnt (unused, as in SIMH; kept for signature fidelity)
     * @param {number} acc
     * @param {Object|null} [stat]
     * @returns {number} the TB entry (access mask | TLB_M | PFN), or 0 on a reported error
     */
    fill(va, lnt, acc, stat)
    {
        /*
         * THE line.  `>>>`, never `>>`; see the header.  `& ~0x03` does not rescue a signed shift.
         */
        let ptidx = (va >>> 7) & ~0x03;
        let ptead, pte, tbi, vpn, tlbpte;

        if (va & VA_S0) {                               /* system space? */
            if (ptidx >= this.d_slr) {
                return this.mmErr(PR_LNV, va, acc, stat);
            }
            ptead = (this.d_sbr + ptidx) & VAX.PAMASK;
        } else {
            if (va & VA_P1) {                           /* P1? */
                if (ptidx < this.d_p1lr) {
                    return this.mmErr(PR_LNV, va, acc, stat);
                }
                ptead = (this.d_p1br + ptidx) | 0;
            } else {                                    /* P0 */
                if (ptidx >= this.d_p0lr) {
                    return this.mmErr(PR_LNV, va, acc, stat);
                }
                ptead = (this.d_p0br + ptidx) | 0;
            }
            if ((ptead & VA_S0) == 0) {
                /*
                 * STOP_PPTE (vax_defs.h:61).  A process page table that does not live in system
                 * space is not a program fault, it is an impossible machine state -- SIMH halts
                 * the simulator rather than raising an exception, and so do we.
                 */
                throw new VAXFault(VAXFAULT.PPTE, ptead, va);
            }
            vpn = (ptead >>> VA_V_VPN) & VA_M_VPN;
            tbi = vpn & VA_M_TBI;
            if (this.stlbTag[tbi] != vpn) {             /* system PTE not in TB? */
                ptidx = ptead >>> 7;                    /* SIMH quirk: no & ~3 here */
                if (ptidx >= this.d_slr) {
                    return this.mmErr(PR_PLNV, va, acc, stat);
                }
                pte = this.readLP((this.d_sbr + ptidx) & VAX.PAMASK);
                if ((pte & PTE_V) == 0) {
                    return this.mmErr(PR_PTNV, va, acc, stat);
                }
                this.stlbTag[tbi] = vpn;
                this.stlbPte[tbi] = (CVTACC[(pte >>> PTE_V_ACC) & PTE_M_ACC] |
                    ((pte << VA_N_OFF) & TLB_PFN)) | 0;
            }
            ptead = ((this.stlbPte[tbi] & TLB_PFN) | (ptead & VA_M_OFF)) | 0;
        }
        pte = this.readL(ptead);                        /* read the page's PTE */
        tlbpte = (CVTACC[(pte >>> PTE_V_ACC) & PTE_M_ACC] |
            ((pte << VA_N_OFF) & TLB_PFN)) | 0;
        if ((tlbpte & acc) == 0) {                      /* access permitted in this mode? */
            return this.mmErr(PR_ACV, va, acc, stat);
        }
        if ((pte & PTE_V) == 0) {                       /* valid? */
            return this.mmErr(PR_TNV, va, acc, stat);
        }
        if (acc & TLB_WACC) {                           /* write access? */
            if ((pte & PTE_M) == 0) {
                this.writeL(ptead, pte | PTE_M);        /* set M in memory, once */
            }
            tlbpte = tlbpte | TLB_M;
        }
        vpn = (va >>> VA_V_VPN) & VA_M_VPN;
        tbi = vpn & VA_M_TBI;
        if ((va & VA_S0) == 0) {                        /* process space? */
            this.ptlbTag[tbi] = vpn;
            this.ptlbPte[tbi] = tlbpte;
            return tlbpte;
        }
        this.stlbTag[tbi] = vpn;
        this.stlbPte[tbi] = tlbpte;
        return tlbpte;
    }

    /**
     * readData(va, lnt, acc)
     *
     * Read() (vax_mmu.h:121).  Three phases, exactly as SIMH's comment describes them:
     *
     *   1. translate through the TB, calling fill() on a tag, access or modify mismatch.  An
     *      invalid TB entry has access bits from a pte of -1 and a tag of -1, so it mismatches on
     *      the tag.  If the resulting physical address is naturally aligned, do one physical read.
     *   2. otherwise, decide whether the access crosses a page boundary.  If it does, translate
     *      the SECOND page too; if it does not, the second physical address is simply the next
     *      longword.
     *   3. stitch the value out of the two longwords.
     *
     * Byte and word results are zero-extended; a longword is a signed int32 (defines.js rule 5).
     *
     * @this {MMUVAX}
     * @param {number} va virtual address (full 32 bits, signed int32)
     * @param {number} lnt L_BYTE, L_WORD or L_LONG
     * @param {number} acc access mask (RA/WA form -- see accRead()/accWrite())
     * @returns {number}
     */
    readData(va, lnt, acc)
    {
        let vpn, off, tbi, pa, pa1, bo, sc, wl, wh, xpte;

        this.mchkVA = va;
        if (this.mapen) {
            vpn = (va >>> VA_V_VPN) & VA_M_VPN;
            off = va & VA_M_OFF;
            tbi = vpn & VA_M_TBI;
            xpte = (va & VA_S0)? this.stlbPte[tbi] : this.ptlbPte[tbi];
            if (((xpte & acc) == 0) || (((va & VA_S0)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                ((acc & TLB_WACC) && ((xpte & TLB_M) == 0))) {
                xpte = this.fill(va, lnt, acc, null);
            }
            pa = (xpte & TLB_PFN) | off;
        } else {
            pa = va & VAX.PAMASK;
            off = 0;
        }
        this.pa = pa;
        this.pa1 = pa;
        if ((pa & (lnt - 1)) == 0) {                    /* naturally aligned? */
            if (lnt >= L_LONG) return this.readL(pa);
            if (lnt == L_WORD) return this.readW(pa);
            return this.readB(pa);
        }
        if (this.mapen && (off + lnt) > VA_PAGSIZE) {   /* crosses a page boundary? */
            vpn = ((va + lnt) >>> VA_V_VPN) & VA_M_VPN;
            tbi = vpn & VA_M_TBI;
            xpte = (va & VA_S0)? this.stlbPte[tbi] : this.ptlbPte[tbi];
            if (((xpte & acc) == 0) || (((va & VA_S0)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                ((acc & TLB_WACC) && ((xpte & TLB_M) == 0))) {
                xpte = this.fill((va + lnt) | 0, lnt, acc, null);
            }
            pa1 = ((xpte & TLB_PFN) | ((va + 4) & VA_M_OFF)) & ~0x03;
        } else {
            pa1 = ((pa + 4) & VAX.PAMASK) & ~0x03;
        }
        this.pa1 = pa1;
        bo = pa & 3;
        if (lnt >= L_LONG) {                            /* unaligned longword */
            sc = bo << 3;
            wl = this.readU(pa, L_LONG - bo);
            wh = this.readU(pa1, bo);
            return (wl | (wh << (32 - sc))) | 0;
        } else if (bo == 1) {                           /* word wholly inside a longword */
            return this.readU(pa, L_WORD);
        } else {                                        /* word crossing a longword */
            wl = this.readU(pa, L_BYTE);
            wh = this.readU(pa1, L_BYTE);
            return wl | (wh << 8);
        }
    }

    /**
     * writeData(va, val, lnt, acc)
     *
     * Write() (vax_mmu.h:180).  Mirrors readData(), with two differences that are SIMH's and are
     * copied deliberately:
     *
     *   - the TB re-fill condition tests `(xpte & TLB_M) == 0` UNCONDITIONALLY, not only when the
     *     access code carries write permission.  A caller reaching Write() with a read access code
     *     would therefore re-fill on every access; no caller does.
     *   - the second page's VPN is computed from `va + 4`, not `va + lnt` as in Read().  For every
     *     length that can reach this path the two agree (a byte never crosses; a word only crosses
     *     at offset 511, where both land in the next page), so it is a cosmetic asymmetry -- but
     *     it is transcribed rather than tidied, because "tidied" is how transcription bugs enter.
     *
     * @this {MMUVAX}
     * @param {number} va
     * @param {number} val right-justified in a longword
     * @param {number} lnt
     * @param {number} acc
     */
    writeData(va, val, lnt, acc)
    {
        let vpn, off, tbi, pa, pa1, bo, sc, xpte;

        this.mchkVA = va;
        if (this.mapen) {
            vpn = (va >>> VA_V_VPN) & VA_M_VPN;
            off = va & VA_M_OFF;
            tbi = vpn & VA_M_TBI;
            xpte = (va & VA_S0)? this.stlbPte[tbi] : this.ptlbPte[tbi];
            if (((xpte & acc) == 0) || (((va & VA_S0)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                ((xpte & TLB_M) == 0)) {
                xpte = this.fill(va, lnt, acc, null);
            }
            pa = (xpte & TLB_PFN) | off;
        } else {
            pa = va & VAX.PAMASK;
            off = 0;
        }
        this.pa = pa;
        this.pa1 = pa;
        if ((pa & (lnt - 1)) == 0) {
            if (lnt >= L_LONG) this.writeL(pa, val);
            else if (lnt == L_WORD) this.writeW(pa, val);
            else this.writeB(pa, val);
            return;
        }
        if (this.mapen && (off + lnt) > VA_PAGSIZE) {
            vpn = ((va + 4) >>> VA_V_VPN) & VA_M_VPN;   /* SIMH quirk: va + 4, not va + lnt */
            tbi = vpn & VA_M_TBI;
            xpte = (va & VA_S0)? this.stlbPte[tbi] : this.ptlbPte[tbi];
            if (((xpte & acc) == 0) || (((va & VA_S0)? this.stlbTag[tbi] : this.ptlbTag[tbi]) != vpn) ||
                ((xpte & TLB_M) == 0)) {
                xpte = this.fill((va + lnt) | 0, lnt, acc, null);
            }
            pa1 = ((xpte & TLB_PFN) | ((va + 4) & VA_M_OFF)) & ~0x03;
        } else {
            pa1 = ((pa + 4) & VAX.PAMASK) & ~0x03;
        }
        this.pa1 = pa1;
        bo = pa & 3;
        if (lnt >= L_LONG) {
            sc = bo << 3;
            this.writeU(pa, val & INSERT[L_LONG - bo], L_LONG - bo);
            this.writeU(pa1, (val >>> (32 - sc)) & INSERT[bo], bo);
        } else if (bo == 1) {
            this.writeU(pa, val & 0xFFFF, L_WORD);
        } else {
            this.writeU(pa, val & 0xFF, L_BYTE);
            this.writeU(pa1, (val >>> 8) & 0xFF, L_BYTE);
        }
    }

    /**
     * test(va, acc, stat)
     *
     * Test() (vax_mmu.h:245) -- THE PROBE SEAM.  See the module header for who consumes it.
     *
     * Note what it is NOT: it is not a pure query.  A TB miss fills the TB, exactly as a real
     * access would, because that is what the hardware does and because PROBER/PROBEW are specified
     * in terms of the same microcode path.  What it does avoid is the FAULT: with `stat` supplied,
     * an inaccessible page returns -1 and sets stat.code instead of throwing.
     *
     * It probes a single BYTE.  A caller that needs to know whether a multi-byte access is wholly
     * accessible must probe both ends -- SIMH's WRITE_Q (vax_cpu.c:222) probes `va + 7` first and
     * `va` second, so that when both pages are bad the reported fault is the one for the FIRST
     * page, which is what the architecture requires.
     *
     * With mapping off, it returns `va & PAMASK` and never fails.
     *
     * @this {MMUVAX}
     * @param {number} va
     * @param {number} acc
     * @param {Object|null} [stat] receives {code: PR_*}; omit to fault instead
     * @returns {number} physical address, or -1 when `stat` was supplied and the access failed
     */
    test(va, acc, stat)
    {
        let vpn, off, tbi, xpte;

        if (stat) stat.code = PR_OK;
        if (this.mapen) {
            vpn = (va >>> VA_V_VPN) & VA_M_VPN;
            off = va & VA_M_OFF;
            tbi = vpn & VA_M_TBI;
            xpte = (va & VA_S0)? this.stlbPte[tbi] : this.ptlbPte[tbi];
            if ((xpte & acc) && (((va & VA_S0)? this.stlbTag[tbi] : this.ptlbTag[tbi]) == vpn)) {
                this.pa = (xpte & TLB_PFN) | off;
                return this.pa;
            }
            xpte = this.fill(va, L_BYTE, acc, stat);
            if (!stat || stat.code == PR_OK) {
                this.pa = (xpte & TLB_PFN) | off;
                return this.pa;
            }
            return -1;
        }
        this.pa = va & VAX.PAMASK;
        return this.pa;
    }

    /**
     * accRead(mode)
     *
     * SIMH's RA: the access code for a READ in `mode` (0=K, 1=E, 2=S, 3=U).
     *
     * @param {number} mode
     * @returns {number}
     */
    static accRead(mode)
    {
        return 1 << (mode & 3);
    }

    /**
     * accWrite(mode)
     *
     * SIMH's WA: the access code for a WRITE in `mode`.
     *
     * @param {number} mode
     * @returns {number}
     */
    static accWrite(mode)
    {
        return (1 << (mode & 3)) << TLB_V_WACC;
    }

    /*
     * ----------------------------------------------------------------------------------------
     * Physical access primitives (vax_mmu.h:283-453).
     *
     * These exist as MMU methods rather than as direct Bus calls because SIMH's inline versions
     * carry semantics the Bus does not: they read the LONGWORD CONTAINING the address (SIMH's
     * `M[pa >> 2]`) and shift the answer out, which is not the same as an unaligned Bus access,
     * and they distinguish a virtual-context reference from a physical-context one for the
     * machine-check reference type.  The aligned byte/word/longword forms are provably identical
     * to the corresponding Bus accessors -- which busdiff.js has already graded against SIMH over
     * 150,000 operations -- so those delegate; readU/writeU, which have no Bus equivalent, spell
     * SIMH's shift-and-mask out.
     *
     * Every `pa` reaching these has come out of (xpte & TLB_PFN) | off, or out of `va & PAMASK`,
     * so it is already in 0..0x3FFFFFFF and non-negative.  The Bus re-normalizes anyway.
     * ----------------------------------------------------------------------------------------
     */

    /**
     * readB(pa)
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @returns {number} 0x00-0xff
     */
    readB(pa)
    {
        return this.bus.getByte(pa);
    }

    /**
     * readW(pa)
     *
     * @this {MMUVAX}
     * @param {number} pa (even)
     * @returns {number} 0x0000-0xffff
     */
    readW(pa)
    {
        return this.bus.getWord(pa);
    }

    /**
     * readL(pa)
     *
     * SIMH's ReadL, in virtual context.  `pa & ~3` reproduces `M[pa >> 2]`: fill() can reach here
     * with a page-table entry address whose low bits are not zero, and SIMH reads the containing
     * longword rather than performing an unaligned access.  Callers in the aligned fast path pass
     * an aligned address, for which the mask is a no-op.
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @returns {number} signed int32
     */
    readL(pa)
    {
        return this.bus.getLong(pa & ~0x03);
    }

    /**
     * readLP(pa)
     *
     * SIMH's ReadLP -- the same read, in PHYSICAL context.  The distinction is not the address
     * (both are physical); it is which machine-check reference type a non-existent-memory failure
     * would report.  fill() uses it for the system PTE of a two-level walk.
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @returns {number} signed int32
     */
    readLP(pa)
    {
        return this.bus.getLong(pa & ~0x03);
    }

    /**
     * readU(pa, lnt)
     *
     * ReadU() (vax_mmu.h:390) -- read 1..3 bytes out of the longword CONTAINING pa, right
     * justified.  This is the primitive both unaligned fragments of a cross-page access go
     * through, so it is where a byte-order or shift error would live.
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @param {number} lnt 1, 2 or 3 (0 is possible for the high fragment of a longword at bo == 0,
     *                    which cannot occur, since bo == 0 is the aligned path)
     * @returns {number}
     */
    readU(pa, lnt)
    {
        let sc = (pa & 3) << 3;
        return (this.bus.getLong(pa & ~0x03) >>> sc) & INSERT[lnt];
    }

    /**
     * writeB(pa, val)
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @param {number} val
     */
    writeB(pa, val)
    {
        this.bus.setByte(pa, val);
    }

    /**
     * writeW(pa, val)
     *
     * @this {MMUVAX}
     * @param {number} pa (even)
     * @param {number} val
     */
    writeW(pa, val)
    {
        this.bus.setWord(pa, val);
    }

    /**
     * writeL(pa, val)
     *
     * SIMH's WriteL (vax_mmu.h:397) branches on ADDR_IS_IO(pa): ordinary memory is one `M[pa>>2]
     * = val`, but a Qbus reference goes through WriteIO(pa,val,L_LONG), which is NOT one op --
     * `vax_io.c`'s WriteIO splits an aligned longword into TWO 16-bit Qbus cycles,
     * `WriteQb(pa,...)` then `WriteQb(pa+2,...)`.  For an UNBACKED Qbus longword that matters
     * observably (pcjsvax-d22): each cycle independently reaches cpustate.js's onBusFault(), so a
     * genuine aligned-long Qbus write calls cqMerr() TWICE, and the second call is where DSER's
     * LST (lost-error) bit gets set (measured directly against the real oracle -- a virgin DSER
     * shows 0x88, not 0x80, after exactly this instruction).  Splitting into two setWord() calls
     * here is what reproduces that for free, without threading a size parameter through
     * onBusFault(): each setWord() that lands on nothing calls the SAME fault path a standalone
     * word write would.  A backed Qbus longword (a real device model, when one lands here) sees
     * two word-sized register writes instead of one long-sized one -- which is exactly the real
     * Qbus wire protocol; nothing decodes REG_BASE via writeL() today, so this is currently
     * unobservable outside the CQBIC probe this item grades.
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @param {number} val
     */
    writeL(pa, val)
    {
        if (VAX.isQbusAddr(pa)) {
            this.bus.setWord(pa, val & 0xFFFF);
            this.bus.setWord((pa + 2) >>> 0, (val >>> 16) & 0xFFFF);
            return;
        }
        this.bus.setLong(pa & ~0x03, val);
    }

    /**
     * writeLP(pa, val)
     *
     * SIMH's WriteLP -- physical context; see readLP().
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @param {number} val
     */
    writeLP(pa, val)
    {
        this.bus.setLong(pa & ~0x03, val);
    }

    /**
     * writeU(pa, val, lnt)
     *
     * WriteU() (vax_mmu.h:436) -- merge 1..3 bytes into the longword containing pa.  For ordinary
     * memory that is a read-modify-write, because the Bus has no sub-longword merge primitive that
     * takes a length.  SIMH's OWN WriteU branches on ADDR_IS_IO(pa) exactly like WriteL does (see
     * its doc comment): a Qbus reference does NOT read-modify-write at all -- it goes to
     * WriteIOU(pa,val,lnt) (vax_io.c:358), which issues DIRECT byte/word Qbus cycles from `val`
     * (already right-justified), with NO preceding read.
     *
     * This is not a cosmetic difference for an UNBACKED Qbus address (pcjsvax-d22): the ordinary
     * RAM path's `this.bus.getLong(addr)` is itself a READ, and a read to nothing throws a
     * SYNCHRONOUS machine check (cpustate.js's onBusFault(), fQbus branch) -- so before this fix,
     * EVERY unaligned Qbus write synchronously machine-checked via the read half of a
     * read-modify-write that real hardware never performs, misreporting a WRITE as if it were a
     * READ that faulted.  Measured directly: real SIMH takes no exception at all for an unaligned
     * write to an unbacked Qbus address (matching the aligned case), so the deferred-write path
     * is unreachable here without this branch.
     *
     * The op-count mirrors WriteIOU's own switch on `lnt` and `pa & 1` exactly, so cqMerr()'s LST
     * bit (set by a SECOND unbacked reference landing on an unresolved error) comes out right for
     * every alignment, the same way writeL()'s two-word split does for the aligned-long case --
     * see cqMerr()'s doc comment in exc.js.
     *
     * @this {MMUVAX}
     * @param {number} pa
     * @param {number} val right-justified: bits 0..(lnt*8-1) are the ones to write
     * @param {number} lnt 1, 2 or 3 (0 is possible for the high fragment of a longword at bo == 0,
     *                    which cannot occur, since bo == 0 is the aligned path)
     */
    writeU(pa, val, lnt)
    {
        if (VAX.isQbusAddr(pa)) {
            switch (lnt) {
            case 1:
                this.bus.setByte(pa, val & 0xFF);
                break;
            case 2:
                if (pa & 1) {
                    this.bus.setByte(pa, val & 0xFF);
                    this.bus.setByte((pa + 1) >>> 0, (val >>> 8) & 0xFF);
                } else {
                    this.bus.setWord(pa, val & 0xFFFF);
                }
                break;
            case 3:
                if (pa & 1) {
                    this.bus.setByte(pa, val & 0xFF);
                    this.bus.setWord((pa + 1) >>> 0, (val >>> 8) & 0xFFFF);
                } else {
                    this.bus.setWord(pa, val & 0xFFFF);
                    this.bus.setByte((pa + 2) >>> 0, (val >>> 16) & 0xFF);
                }
                break;
            }
            return;
        }
        let addr = pa & ~0x03;
        let sc = (pa & 3) << 3;
        let mask = INSERT[lnt] << sc;
        this.bus.setLong(addr, (this.bus.getLong(addr) & ~mask) | ((val & INSERT[lnt]) << sc));
    }
}

MMUVAX.VA_N_OFF     = VA_N_OFF;
MMUVAX.VA_PAGSIZE   = VA_PAGSIZE;
MMUVAX.VA_M_OFF     = VA_M_OFF;
MMUVAX.VA_V_VPN     = VA_V_VPN;
MMUVAX.VA_M_VPN     = VA_M_VPN;
MMUVAX.VA_S0        = VA_S0;
MMUVAX.VA_P1        = VA_P1;
MMUVAX.VA_TBSIZE    = VA_TBSIZE;
MMUVAX.VA_M_TBI     = VA_M_TBI;
MMUVAX.PTE_V        = PTE_V;
MMUVAX.PTE_M        = PTE_M;
MMUVAX.PTE_V_ACC    = PTE_V_ACC;
MMUVAX.TLB_M        = TLB_M;
MMUVAX.TLB_PFN      = TLB_PFN;
MMUVAX.TLB_WACC     = TLB_WACC;
MMUVAX.CVTACC       = CVTACC;
MMUVAX.BR_MASK      = BR_MASK;
MMUVAX.LR_MASK      = LR_MASK;
MMUVAX.L_BYTE       = L_BYTE;
MMUVAX.L_WORD       = L_WORD;
MMUVAX.L_LONG       = L_LONG;

/**
 * Access modes, in architectural order.  These are MODE NUMBERS; an access CODE is a bitmask
 * built from one with accRead()/accWrite().
 */
MMUVAX.MODE = {KERN: KERN, EXEC: EXEC, SUPV: SUPV, USER: USER};

/**
 * Probe results, as returned in stat.code by test() and fill().
 */
MMUVAX.PR = {
    ACV:  PR_ACV,
    LNV:  PR_LNV,
    PACV: PR_PACV,
    PLNV: PR_PLNV,
    TNV:  PR_TNV,
    PTNV: PR_PTNV,
    OK:   PR_OK
};

/**
 * Regions, for diagnostics and coverage accounting only -- translation itself selects with
 * `va & VA_S0` / `va & VA_P1`, exactly as SIMH does, because a mask is sign-safe and a shift is
 * the thing this module's header warns about.
 */
MMUVAX.REGION = {P0: 0, P1: 1, S0: 2, S1: 3};

/**
 * getRegion(va)
 *
 * @param {number} va
 * @returns {number} MMUVAX.REGION.*
 */
MMUVAX.getRegion = function(va)
{
    return va >>> 30;                   // `>>>`.  `>>` here yields 3 for every S0 address.
};

export { MMUVAX };
