/**
 * @fileoverview A multi-device MemoryVAX controller for VAX.PHYSMEM.REG_BASE's local-register span
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
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
 */

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

    return {
        getControllerBuffer(addr) { return [null, 0]; },
        getControllerAccess() {
            return [
                function readByte(off, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let v = dev.readByte(addr);
                        if (v !== null) return v;
                    }
                    return this.readNone(off, addr);
                },
                function writeByte(off, b, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev && dev.writeByte(addr, b)) return;
                    this.writeNone(off, b, addr);
                },
                function readWord(off, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let v = dev.readWord(addr);
                        if (v !== null) return v;
                    }
                    return this.readWordNone(off, addr);
                },
                function writeWord(off, w, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev && dev.writeWord(addr, w)) return;
                    this.writeNone(off, w, addr);
                },
                function readLong(off, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev) {
                        let v = dev.readLong(addr);
                        if (v !== null) return v;
                    }
                    return this.readLongNone(off, addr);
                },
                function writeLong(off, l, addr) {
                    addr = addr >>> 0;
                    let dev = find(addr);
                    if (dev && dev.writeLong(addr, l)) return;
                    this.writeNone(off, l, addr);
                }
            ];
        }
    };
}
