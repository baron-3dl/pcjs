/**
 * @fileoverview Implements the VAX Bus component
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * Portions adapted from the JavaScript PDP 11/70 Emulator written by Paul Nankervis
 * <paulnank@hotmail.com> at <http://skn.noip.me/pdp11/pdp11.html> with permission, by way of
 * PDPjs (machines/dec/pdp11/modules/v2/bus.js).
 *
 * ============================================================================
 * THE UNSIGNED ADDRESS CONVENTION -- the single most important rule in VAXjs
 * ============================================================================
 * VAX S0 system space starts at 0x80000000, which is NEGATIVE as a JavaScript int32.  Every public
 * entry point in this file normalizes its address on entry, first thing, no exceptions:
 *
 *      addr = (addr >>> 0) & this.nBusMask;     // nBusMask == VAX.PAMASK == 0x3FFFFFFF
 *
 * and every block index is computed with `>>>`, never `>>`.  Address arithmetic that could leave
 * the physical space (addr + 1 at the top of memory) is re-normalized by routing back through a
 * public entry point rather than being carried raw.  This mirrors SIMH exactly: cpu_ex()/cpu_dep()
 * do `addr = addr & PAMASK` on a uint32, so `examine 80001000` reads physical 0x1000 -- a behavior
 * this module's differential test (machines/dec/vax/tests/busdiff.js) checks against the real
 * microvax3900 binary.
 *
 * Read the full statement in defines.js before writing any other VAX module.  Two things in it are
 * not obvious from this file: (a) longword DATA is deliberately signed int32, unlike addresses,
 * and (b) the PAMASK here is what makes physical addressing forgiving -- VIRTUAL addresses are 32
 * bits with the region select in <31:30> and no mask can rescue them, so that is where this bug
 * will actually cost someone a day.
 * ============================================================================
 */

import MemoryVAX from "./memory.js";
import MESSAGE from "./message.js";
import Component from "../../../../modules/v2/component.js";
import State from "../../../../modules/v2/state.js";
import { DEBUGGER, VAX } from "./defines.js";

/**
 * @class BusVAX
 * @unrestricted
 */
export default class BusVAX extends Component {
    /**
     * BusVAX(parmsBus, cpu, dbg)
     *
     * The BusVAX component manages the VAX physical address space.  It is allocated like any other
     * PCjs component (by the Computer's init() handler, which then calls every other component's
     * initBus()), but unlike PDPjs's Bus it does NOT maintain two parallel block arrays: PDPjs needs
     * them because the PDP-11 IOPAGE moves within the CPU's address space as the MMU mode changes,
     * whereas the VAX Qbus I/O page lives at a FIXED physical address (0x20000000) and virtual-to-
     * physical translation happens above this layer (see the design note about PDPjs's explicit
     * mapVirtualToPhysical() model, which the VAX MMU will follow).
     *
     * cpu and dbg may both be null.  At this stage of the port there is no VAX CPU yet, and this
     * component is deliberately usable standalone so it can be graded against SIMH on its own.
     *
     * @param {Object} parmsBus
     * @param {CPUStateVAX} [cpu]
     * @param {DebuggerVAX} [dbg]
     */
    constructor(parmsBus, cpu, dbg)
    {
        super("Bus", parmsBus, MESSAGE.BUS);

        this.cpu = cpu || null;
        this.dbg = dbg || null;

        /*
         * DESIGN CONSTRAINT: the physical bus is capped at 30 bits (VAX.PAWIDTH), which is what the
         * CVAX CDAL bus actually implements and what SIMH's PAMASK enforces.  This is not cosmetic:
         * initMemory() eagerly allocates addrTotal/nBlockSize block slots, so a 32-bit bus would
         * allocate 4x the slots for address lines that do not exist.  (It would also break outright:
         * `1 << 32` is 1 in JavaScript.)
         */
        this.nBusWidth = +parmsBus['busWidth'] || VAX.PAWIDTH;
        if (this.nBusWidth > VAX.PAWIDTH) {
            this.printf(MESSAGE.ERROR, "busWidth %d exceeds VAX physical address width; using %d\n", this.nBusWidth, VAX.PAWIDTH);
            this.nBusWidth = VAX.PAWIDTH;
        }

        this.addrTotal = Math.pow(2, this.nBusWidth);
        this.nBusMask = (this.addrTotal - 1) >>> 0;
        this.nBlockSize = BusVAX.BLOCK_SIZE;
        this.nBlockShift = Math.log2(this.nBlockSize) | 0;
        this.nBlockLen = this.nBlockSize >> 2;
        this.nBlockLimit = this.nBlockSize - 1;
        this.nBlockTotal = (this.addrTotal / this.nBlockSize) | 0;
        this.nBlockMask = this.nBlockTotal - 1;

        /*
         * Fault state.  fFault is sticky until checkFault() clears it; nDisableFaults suppresses
         * only the escalation to the CPU (used by the "Direct" interfaces), never the flag itself,
         * exactly as PDPjs does it.
         *
         * How a non-existent-memory fault becomes a VAX machine check (SCB vector 0x04) is
         * deliberately NOT decided here -- that requires the CPU and the SCB, and belongs with
         * those items.  Until then a consumer installs a handler with setFaultHandler().
         */
        this.fFault = false;
        this.addrFault = 0;
        this.nFaults = 0;                   // monotonic counter, used to abort a multi-byte access
        this.nDisableFaults = 0;
        this.fnFault = null;

        /*
         * Array of RESET notification handlers registered by Device components.
         */
        this.afnReset = [];

        /*
         * Scratch pair used by getQuad() when the caller does not supply one.  A quadword is
         * returned as a [lo, hi] pair of longwords rather than a BigInt: BigInt is far too slow for
         * an emulator inner loop, and this matches the hi/lo approach the ported D/G float and
         * EMUL/EDIV code will need anyway.
         */
        this.aQuadTemp = [0, 0];

        this.aMemBlocks = [];
        this.initMemory();

        this.setReady();
    }

    /**
     * initMemory()
     *
     * Allocate enough (empty) Memory blocks to span the entire physical address space.  Every slot
     * initially points at ONE shared empty block, so this costs one pointer per block, not one
     * block per block.
     *
     * @this {BusVAX}
     */
    initMemory()
    {
        let block = new MemoryVAX(this);
        block.copyBreakpoints(this.dbg);
        this.aMemBlocks = new Array(this.nBlockTotal);
        for (let iBlock = 0; iBlock < this.nBlockTotal; iBlock++) {
            this.aMemBlocks[iBlock] = block;
        }
    }

    /**
     * getWidth()
     *
     * @this {BusVAX}
     * @returns {number}
     */
    getWidth()
    {
        return this.nBusWidth;
    }

    /**
     * isReserved(addr)
     *
     * The KA655 I/O, register, SSC, NVR and Qbus-memory ranges are RESERVED by this component but
     * deliberately NOT decoded: no handlers, no storage, no aliasing.  Accesses to them fault as
     * non-existent memory until the device items populate them.  This exists so that a later item
     * can assert "nobody has quietly mapped RAM on top of the I/O space".
     *
     * ROM_BASE is NOT in this list: pcjsvax-223 decodes it (see addRom()), so it is no longer
     * reserved-but-undecoded -- it is reserved-and-DECODED, like RAM.
     *
     * SSC_BASE and NVR_BASE were MISSING from BusVAX.RESERVED until pcjsvax-446's veracity review
     * caught the gap (standing rule 7: scope lives in code, not comments -- this comment already
     * claimed "SSC, NVR" were covered while the array below did not list them).  Both sit past
     * REG_BASE's span (REG_BASE+REG_LENGTH = 0x20100000; SSC_BASE = 0x20140000), so any code
     * deriving "what this bus reserves" from the array alone silently missed them -- and
     * pcjsvax-223 measured the ROM's FIRST absent-hardware probe as SSC+0x0.
     *
     * @this {BusVAX}
     * @param {number} addr (physical)
     * @returns {boolean}
     */
    isReserved(addr)
    {
        addr = (addr >>> 0) & this.nBusMask;
        for (let i = 0; i < BusVAX.RESERVED.length; i++) {
            let r = BusVAX.RESERVED[i];
            if (addr >= r[0] && addr < r[0] + r[1]) return true;
        }
        return false;
    }

    /**
     * reset()
     *
     * @this {BusVAX}
     */
    reset()
    {
        for (let i = 0; i < this.afnReset.length; i++) {
            this.afnReset[i]();
        }
    }

    /**
     * powerUp(data, fRepower)
     *
     * @this {BusVAX}
     * @param {Object|null} data
     * @param {boolean} [fRepower]
     * @returns {boolean} true if successful, false if failure
     */
    powerUp(data, fRepower)
    {
        if (!fRepower) {
            if (!data) {
                this.reset();
            } else {
                if (!this.restore(data)) return false;
            }
        }
        return true;
    }

    /**
     * powerDown(fSave, fShutdown)
     *
     * @this {BusVAX}
     * @param {boolean} [fSave]
     * @param {boolean} [fShutdown]
     * @returns {Object|boolean}
     */
    powerDown(fSave, fShutdown)
    {
        return fSave? this.save() : true;
    }

    /**
     * save()
     *
     * @this {BusVAX}
     * @returns {Object|null}
     */
    save()
    {
        let state = new State(this);
        state.set(0, this.saveMemory());
        return state.data();
    }

    /**
     * restore(data)
     *
     * @this {BusVAX}
     * @param {Object} data
     * @returns {boolean} true if restore successful, false if not
     */
    restore(data)
    {
        return this.restoreMemory(data[0]);
    }

    /**
     * addMemory(addr, size, type, controller)
     *
     * Adds new Memory blocks to the specified address range.  See PDPjs's addMemory() for the full
     * rationale; the only VAX-specific change is that iBlock is computed with `>>>` and the starting
     * address is normalized, so a request at (say) 0x80000000 lands where SIMH would put it.
     *
     * @this {BusVAX}
     * @param {number} addr is the starting physical address of the request
     * @param {number} size of the request, in bytes
     * @param {number} type is one of the MemoryVAX.TYPE constants
     * @param {Object} [controller] is an optional memory controller component
     * @returns {boolean} true if successful, false if not
     */
    addMemory(addr, size, type, controller)
    {
        let addrNext = (addr >>> 0) & this.nBusMask;
        let sizeLeft = size;
        let iBlock = addrNext >>> this.nBlockShift;

        while (sizeLeft > 0 && iBlock < this.aMemBlocks.length) {

            let block = this.aMemBlocks[iBlock];
            let addrBlock = iBlock * this.nBlockSize;
            let sizeBlock = this.nBlockSize - (addrNext - addrBlock);
            if (sizeBlock > sizeLeft) sizeBlock = sizeLeft;

            if (!controller && block && block.size) {
                if (block.type == type) {
                    if (addrNext + sizeLeft <= block.addr) {
                        block.used += (block.addr - addrNext);
                        block.addr = addrNext;
                        return true;
                    }
                    if (addrNext >= block.addr + block.used) {
                        let sizeAvail = block.size - (addrNext - addrBlock);
                        if (sizeAvail > sizeLeft) sizeAvail = sizeLeft;
                        block.used = addrNext - block.addr + sizeAvail;
                        addrNext = addrBlock + this.nBlockSize;
                        sizeLeft -= sizeAvail;
                        iBlock++;
                        continue;
                    }
                }
                return this.reportError(BusVAX.ERROR.RANGE_INUSE, addrNext, sizeLeft);
            }

            let blockNew = new MemoryVAX(this, addrNext, sizeBlock, this.nBlockSize, type, controller);
            blockNew.copyBreakpoints(this.dbg, block);
            this.aMemBlocks[iBlock++] = blockNew;

            addrNext = addrBlock + this.nBlockSize;
            sizeLeft -= sizeBlock;
        }

        if (sizeLeft <= 0) {
            this.printf(MESSAGE.STATUS, "Added %dKb %s at %#010x\n", (size >> 10), MemoryVAX.TYPE_NAMES[type], addr >>> 0);
            return true;
        }

        return this.reportError(BusVAX.ERROR.RANGE_INVALID, addr, size);
    }

    /**
     * addRom(bytes)
     *
     * Decodes the KA655 boot ROM at VAX.PHYSMEM.ROM_BASE for its full mirrored span
     * (VAX.PHYSMEM.ROM_LENGTH, twice ROM_SIZE).  vax_sysdev.c's regtable entry for ROMBASE covers
     * `[ROMBASE, ROMBASE+2*ROMSIZE)` with a SINGLE read routine (`rom_rd`) that masks the offset
     * with ROMAMASK (ROMSIZE-1, vax_sysdev.c:546-555), so the upper half is a TRUE ALIAS of the
     * lower half -- one underlying array, not two copies -- which matters because the one thing
     * ever written into it after this call (the boot-time model magic byte, see
     * CPUStateVAX.boot()) must be visible through the mirror too, exactly as it is on hardware.
     *
     * The primary half is ordinary ROM-typed blocks: read-only to the executing machine (writes
     * go to writeNone), writable only through the Direct accessors, which is how the ROM's own
     * content and the boot-time magic byte get in without ever making the ROM writable to a
     * running program.  The mirror half is this component's FIRST use of
     * MemoryVAX.TYPE.CONTROLLER (memory.js:87-93, never previously instantiated anywhere in this
     * tree): its controller resolves EACH mirror block's `getControllerBuffer(addr)` back to the
     * PRIMARY block already sitting at `addr - ROM_SIZE` and hands back that block's own `adw`, so
     * every mirror block -- addMemory() still creates one MemoryVAX per BLOCK_SIZE chunk, same as
     * any other region -- reads exactly what its primary counterpart holds, live, with no copy.
     * Proved, not assumed, by tests/romdiff.js, which checks the mirror against SIMH at several
     * offsets including the boundary, both before and after the magic byte is written.
     *
     * @this {BusVAX}
     * @param {Uint8Array} bytes exactly VAX.PHYSMEM.ROM_SIZE bytes long
     */
    addRom(bytes)
    {
        if (bytes.length != VAX.PHYSMEM.ROM_SIZE) {
            throw new Error(`BusVAX.addRom: expected ${VAX.PHYSMEM.ROM_SIZE} bytes, got ${bytes.length}`);
        }
        let base = VAX.PHYSMEM.ROM_BASE >>> 0;
        let size = VAX.PHYSMEM.ROM_SIZE;
        this.addMemory(base, size, MemoryVAX.TYPE.ROM);
        for (let i = 0; i < bytes.length; i++) {
            let addr = (base + i) >>> 0;
            this.aMemBlocks[addr >>> this.nBlockShift].writeByteDirect(addr & this.nBlockLimit, bytes[i], addr);
        }
        this.addMemory((base + size) >>> 0, size, MemoryVAX.TYPE.ROM, BusVAX.makeRomAliasController(this));
    }

    /**
     * makeRomAliasController(bus)
     *
     * A MemoryVAX controller (memory.js's constructor: `getControllerBuffer(addr)` ->
     * `[adw, offset]`, `getControllerAccess()` -> the 6-entry afn table) whose buffer, for a mirror
     * block being constructed at physical `addr`, IS the primary block already sitting at
     * `addr - ROM_SIZE`'s own `adw`, at offset 0.  Only the READ half of the afn table is supplied;
     * leaving the write entries `undefined` means both `writeByte` (already blocked by
     * `type == ROM`'s `fReadOnly`) and `writeByteDirect` (which normally bypasses `fReadOnly`, see
     * memory.js's setWriteAccess()) fall through to `writeNone` for the mirror specifically -- the
     * one and only write path into this ROM stays the primary half's Direct accessor, matching real
     * hardware, where there is only one array to write.
     *
     * @param {BusVAX} bus
     * @returns {Object}
     */
    static makeRomAliasController(bus)
    {
        return {
            getControllerBuffer(addr) {
                let primaryAddr = ((addr >>> 0) - VAX.PHYSMEM.ROM_SIZE) >>> 0;
                let block = bus.aMemBlocks[primaryAddr >>> bus.nBlockShift];
                return [block.adw, 0];
            },
            getControllerAccess() {
                return [
                    MemoryVAX.prototype.readByteMemory, undefined,
                    MemoryVAX.prototype.readWordMemory, undefined,
                    MemoryVAX.prototype.readLongMemory, undefined
                ];
            }
        };
    }

    /**
     * removeMemory(addr, size)
     *
     * @this {BusVAX}
     * @param {number} addr
     * @param {number} size
     * @returns {boolean} true if successful, false if not
     */
    removeMemory(addr, size)
    {
        addr = (addr >>> 0) & this.nBusMask;
        if (!(addr & this.nBlockLimit) && size && !(size & this.nBlockLimit)) {
            let iBlock = addr >>> this.nBlockShift;
            while (size > 0) {
                let blockOld = this.aMemBlocks[iBlock];
                let blockNew = new MemoryVAX(this, addr);
                blockNew.copyBreakpoints(this.dbg, blockOld);
                this.aMemBlocks[iBlock++] = blockNew;
                addr = iBlock * this.nBlockSize;
                size -= this.nBlockSize;
            }
            return true;
        }
        return this.reportError(BusVAX.ERROR.RANGE_INVALID, addr, size);
    }

    /**
     * zeroMemory(addr, size, pattern)
     *
     * @this {BusVAX}
     * @param {number} addr
     * @param {number} size
     * @param {number} [pattern]
     */
    zeroMemory(addr, size, pattern)
    {
        addr = (addr >>> 0) & this.nBusMask;
        let off = addr & this.nBlockLimit;
        let iBlock = addr >>> this.nBlockShift;
        while (size > 0 && iBlock < this.aMemBlocks.length) {
            this.aMemBlocks[iBlock].zero(off, size, pattern);
            size -= (this.nBlockSize - off);
            iBlock++;
            off = 0;
        }
    }

    /**
     * getMemoryBlocks(addr, size)
     *
     * @this {BusVAX}
     * @param {number} addr is the starting physical address
     * @param {number} size of the request, in bytes
     * @returns {Array} of Memory blocks
     */
    getMemoryBlocks(addr, size)
    {
        let aBlocks = [];
        let iBlock = ((addr >>> 0) & this.nBusMask) >>> this.nBlockShift;
        while (size > 0 && iBlock < this.aMemBlocks.length) {
            aBlocks.push(this.aMemBlocks[iBlock++]);
            size -= this.nBlockSize;
        }
        return aBlocks;
    }

    /**
     * setMemoryBlocks(addr, size, aBlocks, type)
     *
     * @this {BusVAX}
     * @param {number} addr is the starting physical address
     * @param {number} size of the request, in bytes
     * @param {Array} aBlocks as returned by getMemoryBlocks()
     * @param {number} [type] is one of the MemoryVAX.TYPE constants
     */
    setMemoryBlocks(addr, size, aBlocks, type)
    {
        let i = 0;
        addr = (addr >>> 0) & this.nBusMask;
        let iBlock = addr >>> this.nBlockShift;
        while (size > 0 && iBlock < this.aMemBlocks.length) {
            let block = aBlocks[i++];
            if (!block) break;
            if (type !== undefined) {
                let blockNew = new MemoryVAX(this, addr);
                blockNew.clone(block, type, this.dbg);
                block = blockNew;
            }
            this.aMemBlocks[iBlock++] = block;
            size -= this.nBlockSize;
            addr = iBlock * this.nBlockSize;
        }
    }

    /**
     * getBlock(addr)
     *
     * @this {BusVAX}
     * @param {number} addr (must ALREADY be normalized)
     * @returns {MemoryVAX}
     */
    getBlock(addr)
    {
        return this.aMemBlocks[addr >>> this.nBlockShift];
    }

    /**
     * getByte(addr)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @returns {number} byte (8-bit) value at that address (0x00-0xff)
     */
    getByte(addr)
    {
        addr = (addr >>> 0) & this.nBusMask;
        return this.aMemBlocks[addr >>> this.nBlockShift].readByte(addr & this.nBlockLimit, addr);
    }

    /**
     * getWord(addr)
     *
     * Unaligned word accesses are legal on a VAX.  If the word lies wholly inside one block, the
     * block's own accessor stitches across two longwords as needed; if it straddles the END of a
     * block (and therefore possibly the end of the physical address space), we fall back to byte
     * accesses, each of which re-normalizes its address -- so addr+1 at 0x3FFFFFFF wraps to 0,
     * exactly as SIMH's per-byte `addr & PAMASK` does.
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @returns {number} word (16-bit) value at that address (0x0000-0xffff)
     */
    getWord(addr)
    {
        addr = (addr >>> 0) & this.nBusMask;
        let off = addr & this.nBlockLimit;
        if (off < this.nBlockLimit) {
            return this.aMemBlocks[addr >>> this.nBlockShift].readWord(off, addr);
        }
        /*
         * Straddling fallback.  A non-existent-memory fault ABORTS the remainder of the access
         * (nFaults is a monotonic counter, so this costs nothing and perturbs no sticky state);
         * bytes not fetched read as zero.  Once fFault is set the value is meaningless anyway --
         * on hardware the access aborts into a machine check -- but making it deterministic means
         * it can be graded, and it happens to be exactly what SIMH's console reports for the same
         * access (scp.c get_aval() pre-zeroes sim_eval[] and breaks at the first SCPE_NXM).
         */
        let n = this.nFaults;
        let b0 = this.getByte(addr);
        if (this.nFaults != n) return 0;
        let b1 = this.getByte(addr + 1);
        if (this.nFaults != n) return b0;
        return b0 | (b1 << 8);
    }

    /**
     * getLong(addr)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @returns {number} longword value at that address, as a SIGNED int32 (convention rule 5)
     */
    getLong(addr)
    {
        addr = (addr >>> 0) & this.nBusMask;
        let off = addr & this.nBlockLimit;
        if (off < this.nBlockLimit - 2) {
            return this.aMemBlocks[addr >>> this.nBlockShift].readLong(off, addr);
        }
        let n = this.nFaults, l = 0;                // straddling fallback; see getWord()
        for (let i = 0; i < 4; i++) {
            let b = this.getByte(addr + i);
            if (this.nFaults != n) break;
            l |= (b << (i << 3));
        }
        return l;
    }

    /**
     * getQuad(addr, aQuad)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @param {Array} [aQuad] a caller-supplied 2-element array to fill (avoids allocation)
     * @returns {Array} [lo, hi], each a SIGNED int32 (convention rule 5)
     */
    getQuad(addr, aQuad)
    {
        aQuad = aQuad || this.aQuadTemp;
        aQuad[0] = this.getLong(addr);
        aQuad[1] = this.getLong((addr >>> 0) + 4);
        return aQuad;
    }

    /**
     * setByte(addr, b)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @param {number} b is the byte (8-bit) value to write
     */
    setByte(addr, b)
    {
        addr = (addr >>> 0) & this.nBusMask;
        this.aMemBlocks[addr >>> this.nBlockShift].writeByte(addr & this.nBlockLimit, b & 0xff, addr);
    }

    /**
     * setWord(addr, w)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @param {number} w is the word (16-bit) value to write
     */
    setWord(addr, w)
    {
        addr = (addr >>> 0) & this.nBusMask;
        let off = addr & this.nBlockLimit;
        if (off < this.nBlockLimit) {
            this.aMemBlocks[addr >>> this.nBlockShift].writeWord(off, w & 0xffff, addr);
            return;
        }
        let n = this.nFaults;                       // straddling fallback; a fault aborts the rest
        this.setByte(addr, w & 0xff);
        if (this.nFaults != n) return;
        this.setByte(addr + 1, (w >> 8) & 0xff);
    }

    /**
     * setLong(addr, l)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @param {number} l is the longword value to write
     */
    setLong(addr, l)
    {
        addr = (addr >>> 0) & this.nBusMask;
        let off = addr & this.nBlockLimit;
        if (off < this.nBlockLimit - 2) {
            this.aMemBlocks[addr >>> this.nBlockShift].writeLong(off, l | 0, addr);
            return;
        }
        let n = this.nFaults;                       // straddling fallback; a fault aborts the rest
        for (let i = 0; i < 4; i++) {
            this.setByte(addr + i, (l >>> (i << 3)) & 0xff);
            if (this.nFaults != n) return;
        }
    }

    /**
     * setQuad(addr, lo, hi)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @param {number} lo is the low longword
     * @param {number} hi is the high longword
     */
    setQuad(addr, lo, hi)
    {
        this.setLong(addr, lo);
        this.setLong((addr >>> 0) + 4, hi);
    }

    /**
     * getByteDirect(addr)
     *
     * The "Direct" interfaces are for device I/O and Debugger/console requests, not the CPU: they
     * bypass any "checked" (breakpoint) handlers and they suppress escalation of a fault to the CPU,
     * while still recording it so the caller can test checkFault().
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @returns {number} byte (8-bit) value at that address
     */
    getByteDirect(addr)
    {
        addr = (addr >>> 0) & this.nBusMask;
        this.nDisableFaults++;
        let b = this.aMemBlocks[addr >>> this.nBlockShift].readByteDirect(addr & this.nBlockLimit, addr);
        this.nDisableFaults--;
        return b;
    }

    /**
     * setByteDirect(addr, b)
     *
     * @this {BusVAX}
     * @param {number} addr is a physical address
     * @param {number} b is the byte (8-bit) value to write
     */
    setByteDirect(addr, b)
    {
        addr = (addr >>> 0) & this.nBusMask;
        this.nDisableFaults++;
        this.aMemBlocks[addr >>> this.nBlockShift].writeByteDirect(addr & this.nBlockLimit, b & 0xff, addr);
        this.nDisableFaults--;
    }

    /**
     * addMemBreak(addr, fWrite)
     *
     * @this {BusVAX}
     * @param {number} addr
     * @param {boolean} fWrite
     */
    addMemBreak(addr, fWrite)
    {
        if (DEBUGGER) {
            addr = (addr >>> 0) & this.nBusMask;
            this.aMemBlocks[addr >>> this.nBlockShift].addBreakpoint(addr & this.nBlockLimit, fWrite);
        }
    }

    /**
     * removeMemBreak(addr, fWrite)
     *
     * @this {BusVAX}
     * @param {number} addr
     * @param {boolean} fWrite
     */
    removeMemBreak(addr, fWrite)
    {
        if (DEBUGGER) {
            addr = (addr >>> 0) & this.nBusMask;
            this.aMemBlocks[addr >>> this.nBlockShift].removeBreakpoint(addr & this.nBlockLimit, fWrite);
        }
    }

    /**
     * saveMemory(fAll)
     *
     * @this {BusVAX}
     * @param {boolean} [fAll]
     * @returns {Array}
     */
    saveMemory(fAll)
    {
        let i = 0;
        let a = [];
        for (let iBlock = 0; iBlock < this.nBlockTotal; iBlock++) {
            let block = this.aMemBlocks[iBlock];
            if (fAll && block.type != MemoryVAX.TYPE.ROM || block.fDirty || block.fDirtyEver) {
                a[i++] = iBlock;
                a[i++] = State.compress(block.save());
            }
        }
        return a;
    }

    /**
     * restoreMemory(a)
     *
     * @this {BusVAX}
     * @param {Array} a
     * @returns {boolean} true if successful, false if not
     */
    restoreMemory(a)
    {
        for (let i = 0; i < a.length - 1; i += 2) {
            let iBlock = a[i];
            let adw = a[i+1];
            if (adw && adw.length < this.nBlockLen) {
                adw = State.decompress(adw, this.nBlockLen);
            }
            let block = this.aMemBlocks[iBlock];
            if (!block || !block.restore(adw)) {
                Component.error("Unable to restore memory block " + iBlock);
                return false;
            }
        }
        return true;
    }

    /**
     * getMemoryLimit(type)
     *
     * @this {BusVAX}
     * @param {number} type is one of the MemoryVAX.TYPE constants
     * @returns {number} (the limiting address of the specified memory type, zero if none)
     */
    getMemoryLimit(type)
    {
        let addr = 0;
        for (let iBlock = 0; iBlock < this.aMemBlocks.length; iBlock++) {
            let block = this.aMemBlocks[iBlock];
            if (block.type == type) {
                addr = block.addr + block.used;
            }
        }
        return addr;
    }

    /**
     * addResetHandler(fnReset)
     *
     * @this {BusVAX}
     * @param {function()} fnReset
     */
    addResetHandler(fnReset)
    {
        this.afnReset.push(fnReset);
    }

    /**
     * setFaultHandler(fn)
     *
     * @this {BusVAX}
     * @param {function(number,number)|null} fn
     */
    setFaultHandler(fn)
    {
        this.fnFault = fn || null;
    }

    /**
     * fault(addr, access)
     *
     * Bus interface for signaling non-existent memory.
     *
     * @this {BusVAX}
     * @param {number} addr
     * @param {number} [access] (a VAX.ACCESS value, for diagnostic purposes only)
     */
    fault(addr, access)
    {
        this.fFault = true;
        this.nFaults++;
        this.addrFault = addr >>> 0;
        if (!this.nDisableFaults) {
            if (DEBUGGER && this.dbg) {
                this.dbg.printf(MESSAGE.FAULT + MESSAGE.ADDR, "memory fault (%#x) on %#010x\n", access, addr >>> 0);
            }
            if (this.fnFault) this.fnFault(this.addrFault, access);
        }
    }

    /**
     * checkFault()
     *
     * This also serves as a clearFault() function.
     *
     * @this {BusVAX}
     * @returns {boolean}
     */
    checkFault()
    {
        let f = this.fFault;
        this.fFault = false;
        return f;
    }

    /**
     * reportError(errNum, addr, size, fQuiet)
     *
     * @this {BusVAX}
     * @param {number} errNum
     * @param {number} addr
     * @param {number} size
     * @param {boolean} [fQuiet]
     * @returns {boolean} false
     */
    reportError(errNum, addr, size, fQuiet)
    {
        this.printf(fQuiet? MESSAGE.NONE : MESSAGE.ERROR, "Memory block error (%d: %#x,%#x)\n", errNum, addr >>> 0, size);
        return false;
    }
}

/*
 * 8Kb blocks: the same size PDPjs uses, and the same size as the KA655 Qbus I/O page
 * (IOPAGESIZE, vaxmod_defs.h).  At a 30-bit bus that is 131072 block slots, all of which initially
 * share one empty block object.
 */
BusVAX.BLOCK_SIZE = 0x2000;

/*
 * Physical ranges this component reserves but does NOT decode.  See isReserved().
 */
BusVAX.RESERVED = [
    [VAX.PHYSMEM.CDG_BASE,    VAX.PHYSMEM.CDG_LENGTH],
    [VAX.PHYSMEM.IOPAGE_BASE, VAX.PHYSMEM.IOPAGE_LENGTH],
    [VAX.PHYSMEM.REG_BASE,    VAX.PHYSMEM.REG_LENGTH],
    [VAX.PHYSMEM.CQM_BASE,    VAX.PHYSMEM.CQM_LENGTH],
    [VAX.PHYSMEM.SSC_BASE,    VAX.PHYSMEM.SSC_LENGTH],   // added pcjsvax-446: was a gap (see isReserved())
    [VAX.PHYSMEM.NVR_BASE,    VAX.PHYSMEM.NVR_LENGTH]    // added pcjsvax-446: was a gap (see isReserved())
    /* ROM_BASE removed by pcjsvax-223: it is decoded now (see addRom()), not merely reserved. */
];

BusVAX.ERROR = {
    RANGE_INUSE:        1,
    RANGE_INVALID:      2,
    NO_CONTROLLER:      3
};
