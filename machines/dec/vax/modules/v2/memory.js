/**
 * @fileoverview Implements the VAX Memory component
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
 * Portions adapted from the JavaScript PDP 11/70 Emulator written by Paul Nankervis
 * <paulnank@hotmail.com> at <http://skn.noip.me/pdp11/pdp11.html> with permission, by way of
 * PDPjs (machines/dec/pdp11/modules/v2/memory.js).
 *
 * PDPjs (machines/dec/pdp11/modules/v2/memory.js).
 *
 * ADDRESS CONVENTION: addresses reaching this module have ALREADY been normalized by BusVAX
 * (`(addr >>> 0) & VAX.PAMASK`), and every offset is guaranteed to lie wholly within this block.
 * See the "UNSIGNED ADDRESS CONVENTION" block comment in defines.js -- read it before touching
 * anything in this file.  Within a block we use `off`, a 0..nBlockSize-1 value that is never
 * negative, so `>>` on an offset is safe; `>>` on an ADDRESS never is.
 */

import MESSAGE from "./message.js";
import Component from "../../../../modules/v2/component.js";
import { DEBUGGER, VAX } from "./defines.js";

/**
 * @class MemoryVAX
 * @property {number} id
 * @property {number} used
 * @property {number} size
 * @property {Int32Array} adw
 * @property {Object} controller
 */
export default class MemoryVAX {
    /**
     * MemoryVAX(bus, addr, used, size, type, controller)
     *
     * Every VAX memory block is backed by an Int32Array of LITTLE-ENDIAN LONGWORDS (adw), exactly
     * as PDPjs does it, and for the same reason: the VAX is a little-endian, longword-oriented
     * machine, and it permits unaligned byte/word/longword/quadword access.  The shift/mask
     * accessors below stitch across two adjacent longwords when an access is unaligned; accesses
     * that would straddle the END of this block are split by BusVAX before they ever get here, so
     * `adw[idw + 1]` is always in bounds.
     *
     * The Bus allocates one shared empty block (size 0) for the entire address space at init, so
     * every address routes through the same code path; unbacked addresses land on readNone/writeNone
     * and are reported to the Bus as non-existent memory.
     *
     * WARNING: Memory blocks are low-level objects with no UI requirements, so they do NOT inherit
     * from Component; use the Debugger's methods, not Component's, for assertions.
     *
     * @param {BusVAX} bus
     * @param {number|null} [addr] of lowest used address in block
     * @param {number} [used] portion of block in bytes (0 for none); must be a multiple of 4
     * @param {number} [size] of block's buffer in bytes (0 for none); must be a multiple of 4
     * @param {number} [type] is one of the MemoryVAX.TYPE constants (default is MemoryVAX.TYPE.NONE)
     * @param {Object} [controller] is an optional memory controller component
     */
    constructor(bus, addr, used, size, type, controller)
    {
        this.bus = bus;
        this.id = (MemoryVAX.idBlock += 2);
        this.adw = null;
        this.offset = 0;
        this.addr = addr;
        this.used = used;
        this.size = size || 0;
        this.type = type || MemoryVAX.TYPE.NONE;
        this.fReadOnly = (type == MemoryVAX.TYPE.ROM);
        this.controller = null;
        this.dbg = null;
        this.cReadBreakpoints = this.cWriteBreakpoints = 0;
        this.fDirty = this.fDirtyEver = false;

        this.readByte  = this.readByteDirect  = this.readNone;
        this.readWord  = this.readWordDirect  = this.readWordNone;
        this.readLong  = this.readLongDirect  = this.readLongNone;
        this.writeByte = this.writeByteDirect = this.writeNone;
        this.writeWord = this.writeWordDirect = this.writeNone;
        this.writeLong = this.writeLongDirect = this.writeNone;

        this.copyBreakpoints();

        if (!this.size) {
            this.setAccess();
            return;
        }

        if (controller) {
            this.controller = controller;
            let a = controller.getControllerBuffer(addr);
            this.adw = a[0];
            this.offset = a[1];
            this.setAccess(controller.getControllerAccess());
            return;
        }

        this.buffer = new ArrayBuffer(this.size);
        this.adw = new Int32Array(this.buffer, 0, this.size >> 2);
        this.setAccess(MemoryVAX.afnMemory);
    }

    /**
     * init(addr)
     *
     * @this {MemoryVAX}
     * @param {number} addr
     */
    init(addr)
    {
        this.addr = addr;
    }

    /**
     * clone(mem, type, dbg)
     *
     * @this {MemoryVAX}
     * @param {MemoryVAX} mem
     * @param {number} [type]
     * @param {DebuggerVAX} [dbg]
     */
    clone(mem, type, dbg)
    {
        this.id = mem.id | 0x1;
        this.used = mem.used;
        this.size = mem.size;
        if (type) {
            this.type = type;
            this.fReadOnly = (type == MemoryVAX.TYPE.ROM);
        }
        this.buffer = mem.buffer;
        this.adw = mem.adw;
        this.setAccess(MemoryVAX.afnMemory);
        this.copyBreakpoints(dbg, mem);
    }

    /**
     * save()
     *
     * Returns the block contents as an array of signed 32-bit longwords (see convention rule 5 in
     * defines.js: longword DATA is int32, and may therefore appear negative).
     *
     * @this {MemoryVAX}
     * @returns {Array|null}
     */
    save()
    {
        if (this.controller) return null;
        let adw = new Array(this.size >> 2);
        for (let i = 0; i < adw.length; i++) adw[i] = this.adw[i];
        return adw;
    }

    /**
     * restore(adw)
     *
     * @this {MemoryVAX}
     * @param {Array|null} adw
     * @returns {boolean} true if successful, false if block size mismatch
     */
    restore(adw)
    {
        if (this.controller) return (adw == null);
        Component.assert(adw != null);
        if (adw && this.size == adw.length << 2) {
            for (let i = 0; i < adw.length; i++) this.adw[i] = adw[i];
            this.fDirty = true;
            return true;
        }
        return false;
    }

    /**
     * zero(off, len, pattern)
     *
     * @this {MemoryVAX}
     * @param {number} [off] (optional starting byte offset within block)
     * @param {number} [len] (optional maximum number of bytes; default is the entire block)
     * @param {number} [pattern]
     */
    zero(off, len, pattern)
    {
        off = off || 0;
        pattern = (pattern || 0) & 0xff;
        if (len === undefined) len = this.size;
        if (!this.adw) return;
        let end = off + len;
        if (end > this.size) end = this.size;
        for (let i = off; i < end; i++) this.writeByteDirect(i, pattern, this.addr + i);
    }

    /**
     * setAccess(afn, fDirect)
     *
     * The afn parameter is a 6-entry function table: readByte, writeByte, readWord, writeWord,
     * readLong, writeLong.  (PDPjs uses 4; the VAX adds native longword accessors, which the CPU
     * will use far more often than either of the others.)  Quadword access is composed from two
     * longword accesses by the Bus, because a quadword can straddle a block boundary and because
     * JS has no native 64-bit integer that is cheap in an inner loop.
     *
     * @this {MemoryVAX}
     * @param {Array.<function()>} [afn] function table
     * @param {boolean} [fDirect] (true to update direct access functions as well; default is true)
     */
    setAccess(afn, fDirect)
    {
        if (!afn) {
            Component.assert(this.type == MemoryVAX.TYPE.NONE);
            afn = MemoryVAX.afnNone;
        }
        this.setReadAccess(afn, fDirect);
        this.setWriteAccess(afn, fDirect);
    }

    /**
     * setReadAccess(afn, fDirect)
     *
     * @this {MemoryVAX}
     * @param {Array.<function()>} afn
     * @param {boolean} [fDirect]
     */
    setReadAccess(afn, fDirect)
    {
        if (!fDirect || !this.cReadBreakpoints) {
            this.readByte = afn[0] || this.readNone;
            this.readWord = afn[2] || this.readWordNone;
            this.readLong = afn[4] || this.readLongNone;
        }
        if (fDirect || fDirect === undefined) {
            this.readByteDirect = afn[0] || this.readNone;
            this.readWordDirect = afn[2] || this.readWordNone;
            this.readLongDirect = afn[4] || this.readLongNone;
        }
    }

    /**
     * setWriteAccess(afn, fDirect)
     *
     * @this {MemoryVAX}
     * @param {Array.<function()>} afn
     * @param {boolean} [fDirect]
     */
    setWriteAccess(afn, fDirect)
    {
        if (!fDirect || !this.cWriteBreakpoints) {
            this.writeByte = !this.fReadOnly && afn[1] || this.writeNone;
            this.writeWord = !this.fReadOnly && afn[3] || this.writeNone;
            this.writeLong = !this.fReadOnly && afn[5] || this.writeNone;
        }
        if (fDirect || fDirect === undefined) {
            this.writeByteDirect = afn[1] || this.writeNone;
            this.writeWordDirect = afn[3] || this.writeNone;
            this.writeLongDirect = afn[5] || this.writeNone;
        }
    }

    /**
     * resetReadAccess()
     *
     * @this {MemoryVAX}
     */
    resetReadAccess()
    {
        this.readByte = this.readByteDirect;
        this.readWord = this.readWordDirect;
        this.readLong = this.readLongDirect;
    }

    /**
     * resetWriteAccess()
     *
     * @this {MemoryVAX}
     */
    resetWriteAccess()
    {
        this.writeByte = this.fReadOnly? this.writeNone : this.writeByteDirect;
        this.writeWord = this.fReadOnly? this.writeNone : this.writeWordDirect;
        this.writeLong = this.fReadOnly? this.writeNone : this.writeLongDirect;
    }

    /**
     * addBreakpoint(off, fWrite)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {boolean} fWrite
     */
    addBreakpoint(off, fWrite)
    {
        if (!fWrite) {
            if (this.cReadBreakpoints++ === 0) this.setReadAccess(MemoryVAX.afnChecked, false);
        } else {
            if (this.cWriteBreakpoints++ === 0) this.setWriteAccess(MemoryVAX.afnChecked, false);
        }
    }

    /**
     * removeBreakpoint(off, fWrite)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {boolean} fWrite
     */
    removeBreakpoint(off, fWrite)
    {
        if (!fWrite) {
            if (--this.cReadBreakpoints === 0) this.resetReadAccess();
            Component.assert(this.cReadBreakpoints >= 0);
        } else {
            if (--this.cWriteBreakpoints === 0) this.resetWriteAccess();
            Component.assert(this.cWriteBreakpoints >= 0);
        }
    }

    /**
     * copyBreakpoints(dbg, mem)
     *
     * @this {MemoryVAX}
     * @param {DebuggerVAX} [dbg]
     * @param {MemoryVAX} [mem]
     */
    copyBreakpoints(dbg, mem)
    {
        this.dbg = dbg;
        this.cReadBreakpoints = this.cWriteBreakpoints = 0;
        if (mem) {
            if ((this.cReadBreakpoints = mem.cReadBreakpoints)) {
                this.setReadAccess(MemoryVAX.afnChecked, false);
            }
            if ((this.cWriteBreakpoints = mem.cWriteBreakpoints)) {
                this.setWriteAccess(MemoryVAX.afnChecked, false);
            }
        }
    }

    /**
     * readNone(off, addr)
     *
     * Non-existent memory.  We report the fault to the Bus and return all bits set, which is what
     * an undriven CDAL bus looks like.  NOTE: the VALUE returned here is not something SIMH's
     * console can be differentially compared against, because SIMH's console refuses the access
     * outright (SCPE_NXM) rather than returning data; the observable, comparable behavior is the
     * FAULT, which BusVAX.checkFault() exposes.
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number}
     */
    readNone(off, addr)
    {
        if (DEBUGGER && this.dbg) {
            this.dbg.printf(MESSAGE.MEMORY, "attempt to read invalid address %#010x\n", addr);
        }
        this.bus.fault(addr, VAX.ACCESS.READ_BYTE);
        return 0xff;
    }

    /**
     * readWordNone(off, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number}
     */
    readWordNone(off, addr)
    {
        this.bus.fault(addr, VAX.ACCESS.READ_WORD);
        return 0xffff;
    }

    /**
     * readLongNone(off, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number}
     */
    readLongNone(off, addr)
    {
        this.bus.fault(addr, VAX.ACCESS.READ_LONG);
        return -1;                          // 0xffffffff as a signed int32 (see convention rule 5)
    }

    /**
     * writeNone(off, v, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} v
     * @param {number} addr
     */
    writeNone(off, v, addr)
    {
        if (DEBUGGER && this.dbg) {
            this.dbg.printf(MESSAGE.MEMORY, "attempt to write %#x to invalid address %#010x\n", v, addr);
        }
        this.bus.fault(addr, VAX.ACCESS.WRITE);
    }

    /**
     * readByteMemory(off, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number} (0x00-0xff)
     */
    readByteMemory(off, addr)
    {
        return (this.adw[off >> 2] >>> ((off & 0x3) << 3)) & 0xff;
    }

    /**
     * readWordMemory(off, addr)
     *
     * Unaligned words are legal on a VAX.  When the word begins in the last byte of a longword, we
     * stitch it together from two adjacent longwords -- the same trick PDPjs uses, generalized by
     * the fact that the VAX never faults on an unaligned access.
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number} (0x0000-0xffff)
     */
    readWordMemory(off, addr)
    {
        let idw = off >> 2;
        let nShift = (off & 0x3) << 3;
        if (nShift < 24) {
            return (this.adw[idw] >>> nShift) & 0xffff;
        }
        return (this.adw[idw] >>> 24) | ((this.adw[idw + 1] & 0xff) << 8);
    }

    /**
     * readLongMemory(off, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number} (signed int32; see convention rule 5 in defines.js)
     */
    readLongMemory(off, addr)
    {
        let idw = off >> 2;
        let nShift = (off & 0x3) << 3;
        if (!nShift) {
            return this.adw[idw];
        }
        return (this.adw[idw] >>> nShift) | (this.adw[idw + 1] << (32 - nShift));
    }

    /**
     * writeByteMemory(off, b, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} b
     * @param {number} addr
     */
    writeByteMemory(off, b, addr)
    {
        let idw = off >> 2;
        let nShift = (off & 0x3) << 3;
        this.adw[idw] = (this.adw[idw] & ~(0xff << nShift)) | ((b & 0xff) << nShift);
        this.fDirty = true;
    }

    /**
     * writeWordMemory(off, w, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} w
     * @param {number} addr
     */
    writeWordMemory(off, w, addr)
    {
        let idw = off >> 2;
        let nShift = (off & 0x3) << 3;
        w &= 0xffff;
        if (nShift < 24) {
            this.adw[idw] = (this.adw[idw] & ~(0xffff << nShift)) | (w << nShift);
        } else {
            this.adw[idw] = (this.adw[idw] & 0x00ffffff) | (w << 24);
            idw++;
            this.adw[idw] = (this.adw[idw] & ~0xff) | (w >>> 8);
        }
        this.fDirty = true;
    }

    /**
     * writeLongMemory(off, l, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} l
     * @param {number} addr
     */
    writeLongMemory(off, l, addr)
    {
        let idw = off >> 2;
        let nShift = (off & 0x3) << 3;
        if (!nShift) {
            this.adw[idw] = l;
        } else {
            /*
             * nShift is 8, 16 or 24 (never 0 -- the aligned case returned above, and never 32,
             * which JS would take mod 32 and turn into a no-op shift).
             *
             * The low (32 - nShift) bits of l occupy the HIGH (32 - nShift) bits of adw[idw]; the
             * remaining nShift bits of l occupy the LOW nShift bits of adw[idw+1].  Note that the
             * two "keep" masks are ((1 << nShift) - 1) and its complement -- the split point is the
             * same in both longwords, which is easy to get backwards, and getting it backwards is
             * invisible to any test that only ever writes ALIGNED longwords.
             */
            this.adw[idw] = (this.adw[idw] & ((1 << nShift) - 1)) | (l << nShift);
            idw++;
            this.adw[idw] = (this.adw[idw] & ~((1 << nShift) - 1)) | (l >>> (32 - nShift));
        }
        this.fDirty = true;
    }

    /**
     * readByteChecked(off, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number}
     */
    readByteChecked(off, addr)
    {
        if (DEBUGGER && this.dbg && this.addr != null) this.dbg.checkMemoryRead(this.addr + off, 1);
        return this.readByteDirect(off, addr);
    }

    /**
     * readWordChecked(off, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number}
     */
    readWordChecked(off, addr)
    {
        if (DEBUGGER && this.dbg && this.addr != null) this.dbg.checkMemoryRead(this.addr + off, 2);
        return this.readWordDirect(off, addr);
    }

    /**
     * readLongChecked(off, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} addr
     * @returns {number}
     */
    readLongChecked(off, addr)
    {
        if (DEBUGGER && this.dbg && this.addr != null) this.dbg.checkMemoryRead(this.addr + off, 4);
        return this.readLongDirect(off, addr);
    }

    /**
     * writeByteChecked(off, b, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} b
     * @param {number} addr
     */
    writeByteChecked(off, b, addr)
    {
        if (DEBUGGER && this.dbg && this.addr != null) this.dbg.checkMemoryWrite(this.addr + off, 1);
        if (this.fReadOnly) this.writeNone(off, b, addr); else this.writeByteDirect(off, b, addr);
    }

    /**
     * writeWordChecked(off, w, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} w
     * @param {number} addr
     */
    writeWordChecked(off, w, addr)
    {
        if (DEBUGGER && this.dbg && this.addr != null) this.dbg.checkMemoryWrite(this.addr + off, 2);
        if (this.fReadOnly) this.writeNone(off, w, addr); else this.writeWordDirect(off, w, addr);
    }

    /**
     * writeLongChecked(off, l, addr)
     *
     * @this {MemoryVAX}
     * @param {number} off
     * @param {number} l
     * @param {number} addr
     */
    writeLongChecked(off, l, addr)
    {
        if (DEBUGGER && this.dbg && this.addr != null) this.dbg.checkMemoryWrite(this.addr + off, 4);
        if (this.fReadOnly) this.writeNone(off, l, addr); else this.writeLongDirect(off, l, addr);
    }
}

/*
 * Basic memory types (see PDPjs for the rationale; the VAX set is the same minus VIDEO).
 */
MemoryVAX.TYPE = {
    NONE:       0,
    RAM:        1,
    ROM:        2,
    CONTROLLER: 3
};
MemoryVAX.TYPE_NAMES = ["NONE", "RAM", "ROM", "H/W"];

/*
 * Last used block ID (used for debugging only)
 */
MemoryVAX.idBlock = 0;

/*
 * Access function tables: [readByte, writeByte, readWord, writeWord, readLong, writeLong]
 */
MemoryVAX.afnNone = [];

MemoryVAX.afnMemory = [
    MemoryVAX.prototype.readByteMemory,
    MemoryVAX.prototype.writeByteMemory,
    MemoryVAX.prototype.readWordMemory,
    MemoryVAX.prototype.writeWordMemory,
    MemoryVAX.prototype.readLongMemory,
    MemoryVAX.prototype.writeLongMemory
];

MemoryVAX.afnChecked = [
    MemoryVAX.prototype.readByteChecked,
    MemoryVAX.prototype.writeByteChecked,
    MemoryVAX.prototype.readWordChecked,
    MemoryVAX.prototype.writeWordChecked,
    MemoryVAX.prototype.readLongChecked,
    MemoryVAX.prototype.writeLongChecked
];
