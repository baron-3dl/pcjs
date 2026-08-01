/**
 * @fileoverview A multi-device MemoryVAX controller for VAX.PHYSMEM.REG_BASE's local-register span
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this
 * work.
 *
 * ============================================================================
 * WHAT THIS IS
 * ============================================================================
 * pcjsvax-bfb's own romdiff boundary-advance: REG_BASE (0x20080000, VAX.PHYSMEM.REG_LENGTH =
 * 0x80000 bytes -- 64 KA655 bus blocks at BusVAX.BLOCK_SIZE) is where vax_sysdev.c's `regtable[]`
 * puts SEVERAL named devices at fixed sub-offsets: CQBIC (REGBASE+0, cqbic.js), CMCTL
 * (REGBASE+0x100), the KA655 CACR/BDR pair (REGBASE+0x4000), CQMAP (REGBASE+0x8000), CQIPC
 * (REGBASE+0x1F40).  Each is its own device class (see cqbic.js, and whichever future item lands
 * CMCTL/KA/CQMAP/CQIPC); this module is only the DISPATCHER that makes ONE MemoryVAX controller,
 * installed once over the whole REG_LENGTH span (bus.js's addRegBlock()), answer for however many
 * of those sub-ranges have a device installed and fall through exactly like undecoded SSC registers
 * do (ssc.js) for the rest -- so tests/romdiff.js's boundary-advance keeps naming the NEXT undecoded
 * range by address, never a whole-block false negative.
 *
 * `devices` is a plain array of `{base, length, dev}`; `dev` exposes readLong/readWord/readByte/
 * writeLong/writeWord/writeByte(addr) exactly like SSCVAX/NVRVAX/CQBICVAX.  Checked in the order
 * given (the ranges are disjoint on real hardware, so order does not matter for correctness, only
 * for which one this file's own selfcheck exercises first).
 *
 * ============================================================================
 * THREE OUTCOMES, NOT TWO -- WHY `REG_MCHK` EXISTS (pcjsvax-622)
 * ============================================================================
 * A sub-device's read used to answer one of two ways: a number ("I decode this"), or `null` ("I do
 * not"), the latter routed to MemoryVAX.readNone() -> BusVAX.fault() -> CPUStateVAX.onBusFault(),
 * which reproduces vax_sysdev.c's ReadReg()/WriteReg() fall-through -- `ssc_bto |= SSCBTO_BTO |
 * SSCBTO_RWT` and THEN a machine check (:1031, :1070).
 *
 * vax_sysdev.c has a THIRD outcome that neither of those expresses: a handler that is reached, and
 * that raises `MACH_CHECK()` ITSELF, without the bus-timeout bits.  cmctl_rd()/cmctl_wr()'s
 * register 18 is one (cmctl.js); cqbic.js's header names two more it had to disclose as gaps for
 * want of this seam (cqmap_rd()'s read branch, and cqm_rd()/cqm_wr() -- wiring those is
 * pcjsvax-5c1's work, not this file's).  The difference is DIRECTLY OBSERVABLE to the ROM, which
 * can read the SSC bus-timeout register: measured on the live oracle, a read of REG+0x148
 * (register 18) machine-checks with BTO = 00000000, while a read of REG+0x14C -- one longword past
 * the end of CMCTL, genuinely undecoded -- machine-checks with BTO = C0000000.
 *
 * So a device may return `REG_MCHK` from any read or write, and this dispatcher raises the fault
 * through BusVAX.fault()'s `fNoBto` argument instead of falling through to readNone/writeNone.
 * `REG_MCHK` is a Symbol rather than a sentinel number so that it can never collide with a
 * register value, and rather than `null` so that "I do not decode this" keeps meaning exactly what
 * it meant before.
 */

import { VAX } from "./defines.js";

/**
 * The "reached the handler, and the handler raised MACH_CHECK() itself" answer -- see the file
 * header.  Exported so a sub-device (cmctl.js today) can return it from any read or write.
 */
export const REG_MCHK = Symbol("REG_MCHK");

/**
 * makeRegController(devices)
 *
 * @param {Array<{base: number, length: number, dev: Object}>} devices
 * @returns {Object}
 */
export function makeRegController(devices)
{
    let entries = devices.map((d) => ({base: d.base >>> 0, high: (d.base + d.length) >>> 0, dev: d.dev}));
    function find(addr) {
        for (let e of entries) if (addr >= e.base && addr < e.high) return e.dev;
        return null;
    }

    /**
     * mchk(blk, addr, access, dflt)
     *
     * The REG_MCHK path: BusVAX.fault() with `fNoBto` set, so CPUStateVAX.onBusFault() raises
     * SCB_MCHK WITHOUT the SSC bus-timeout bits.  `dflt` mirrors what the corresponding
     * MemoryVAX.readNone()/readWordNone()/readLongNone() returns; with a CPU attached the fault
     * throws and the value is never seen, and without one it keeps the undriven-CDAL convention.
     */
    function mchk(blk, addr, access, dflt) {
        blk.bus.fault(addr, access, true);
        return dflt;
    }

    /*
     * BUILT ONCE PER CONTROLLER AND SHARED BY EVERY BLOCK, not rebuilt per block.
     *
     * HANDOFF.md standing rule 14, and this file came within one item of repeating the failure that
     * earned it.  getControllerAccess() used to allocate a fresh six-closure array on every call --
     * i.e. once per bus block -- which is precisely the shape that took cdgdiff.js to 8.6 GB RSS
     * and OOM-killed the orchestrator and every sibling agent: "the test built a fresh machine per
     * case, and each construction registered a span whose controller handed back a newly-allocated
     * access table PER BLOCK."  It stayed cheap here only because addRegBlock() spans 64 blocks and
     * addIoPage() exactly one.  pcjsvax-5c1's addCqm() spans 512, and tests/timerdiff.js -- which
     * builds a machine per case -- was OOM-KILLED (rc=137) on the first gate run after it landed.
     *
     * Sharing is safe because none of this is per-block state: each function is invoked as a METHOD
     * on its own MemoryVAX, so `this` still binds to the right block at call time, and the only
     * values closed over (`entries`, `find`, `mchk`) belong to the controller, not to a block.
     */
    const access = [
                function readByte(off, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let v = dev.readByte(addr);
                        if (v === REG_MCHK) return mchk(this, addr, VAX.ACCESS.READ_BYTE, 0xFF);
                        if (v !== null) return v;
                    }
                    return this.readNone(off, addr);
                },
                function writeByte(off, b, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let r = dev.writeByte(addr, b);
                        if (r === REG_MCHK) { mchk(this, addr, VAX.ACCESS.WRITE_BYTE, 0); return; }
                        if (r) return;
                    }
                    this.writeNone(off, b, addr);
                },
                function readWord(off, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let v = dev.readWord(addr);
                        if (v === REG_MCHK) return mchk(this, addr, VAX.ACCESS.READ_WORD, 0xFFFF);
                        if (v !== null) return v;
                    }
                    return this.readWordNone(off, addr);
                },
                function writeWord(off, w, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let r = dev.writeWord(addr, w);
                        if (r === REG_MCHK) { mchk(this, addr, VAX.ACCESS.WRITE_WORD, 0); return; }
                        if (r) return;
                    }
                    this.writeNone(off, w, addr);
                },
                function readLong(off, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let v = dev.readLong(addr);
                        if (v === REG_MCHK) return mchk(this, addr, VAX.ACCESS.READ_LONG, -1);
                        if (v !== null) return v;
                    }
                    return this.readLongNone(off, addr);
                },
                function writeLong(off, l, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let r = dev.writeLong(addr, l);
                        if (r === REG_MCHK) { mchk(this, addr, VAX.ACCESS.WRITE_LONG, 0); return; }
                        if (r) return;
                    }
                    this.writeNone(off, l, addr);
                }
    ];

    return {
        getControllerBuffer(addr) { return [null, 0]; },
        getControllerAccess() { return access; }
    };
}
