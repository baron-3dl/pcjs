/**
 * @fileoverview pcjsvax-ae1 -- rq.js's image-provider contract over a LAZY, READ-ONLY backing store
 *               plus a COPY-ON-WRITE overlay, so a browser `File` can be DUA0
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHAT THIS IS
 * ------------
 * tests/mscpharness.js's `fileImageProvider()` is the NODE half of rq.js's provider contract; this
 * is the BROWSER half.  rq.js's attach() takes `{byteLength, read(), [write()], [filesystemBytes]}`
 * and "cannot tell an `fs` descriptor from a browser `File`" -- its own header's words, and this
 * file is the thing that sentence was written for.
 *
 * It is deliberately BACKEND-AGNOSTIC: it takes a `readRaw(offset, length)` and knows nothing about
 * `File`, `Blob`, `XMLHttpRequest` or `fs`.  browser/vaxworker.js supplies the two browser backends
 * (FileReaderSync over `File.slice()`, and a ranged synchronous XHR); tests/overlayprovider.js
 * supplies an `fs` one and grades this file against `fileImageProvider()` over the SAME container.
 * That is the only reason the seam is here rather than inline in the worker -- a provider that only
 * exists inside a Worker cannot be graded by anything in this tree.
 *
 * THE THREE CONSTRAINTS IT EXISTS TO SATISFY
 * ------------------------------------------
 * 1. *** NEVER LOAD THE WHOLE IMAGE. ***  A real OpenVMS volume is ~1 GB (the one measured here is
 *    1,048,624,640 bytes) and reading it into a tab kills the tab.  Every read goes through a
 *    fixed-size chunk cache over `readRaw`, so the resident set is bounded by `cacheChunks *
 *    chunkBytes` NO MATTER HOW BIG THE CONTAINER IS.  A VMS boot touches tens of MB of a 1 GB
 *    volume and never has more than the cache resident.
 *
 * 2. *** WRITES MUST WORK, AND THE USER'S FILE MUST NOT BE TOUCHED. ***  rq.js's attach() reads the
 *    ABSENCE of `write` as sim_disk_attach_ex2()'s "rb+" -> "rb" fallback and forces UNIT_RO --
 *    and MEASURED 2026-07-30 (see tests/vmsbootprobe.js's header), a read-only DUA0 stops OpenVMS
 *    dead at `%SYSTEM-I-MOUNTVER, VAX1$DUA0: has been write-locked.`  A browser `File` is read-only
 *    by nature.  So writes land in an IN-MEMORY COPY-ON-WRITE OVERLAY keyed by 512-byte block: the
 *    unit is writable, mount verification completes, and the picked file is never modified.  The
 *    overlay is BOUNDED (`maxOverlayBlocks`) and throws by name when the bound is reached rather
 *    than growing until the tab dies -- HANDOFF.md standing rule 14, every bound absolute.
 *
 * 3. *** THE OVERLAY IS PART OF THE READ PATH, NOT A WRITE LOG. ***  A block that has been written
 *    must read back written, including a block PAST THE END OF THE CONTAINER: autosize routinely
 *    leaves the unit larger than the file (the volume measured here declares 1,505,766,912 bytes
 *    against a 1,048,624,640-byte container), rq.js's diskRead() treats a short read as EOF and
 *    zero-fills, and VMS writes into that region during mount verification.  So `read()` reports
 *    bytes DELIVERED as the larger of what the backing store had and what the overlay holds, and
 *    zero-fills the gap between them itself.
 *
 * WHAT IT IS NOT.  It is not a persistence layer.  Nothing survives a page reload; that is
 * `pcjsvax-367`'s job (IndexedDB / File System Access API), filed as follow-on work.
 */

import { ods2VolumeBytesFrom } from "../modules/v2/ods2.js";

/** rq.js's RQ_NUMBY -- the sector size every offset and length in this file is reasoned in. */
const SECTOR = 512;

/** 64 KiB.  Big enough that a 32 KiB MSCP transfer is one or two backing reads, small enough that
    the cache's granularity does not dominate its ceiling. */
const DEFAULT_CHUNK_BYTES = 64 * 1024;

/** 256 chunks x 64 KiB = 16 MiB resident, whatever the container's size. */
const DEFAULT_CACHE_CHUNKS = 256;

/** 262,144 blocks x 512 = 128 MiB of dirty data before the overlay refuses.  A boot to the login
    prompt writes a small fraction of this; the bound is here so that a RUNAWAY writer fails by
    name instead of taking the tab down. */
const DEFAULT_MAX_OVERLAY_BLOCKS = 262144;

/**
 * overlayImageProvider(opts)
 *
 * @param {Object} opts
 *   {number}   byteLength          the container's size in bytes
 *   {Function} readRaw             (offset, length) -> Uint8Array of 0..length bytes, SYNCHRONOUS.
 *                                  Short means EOF; it must never throw for an in-range request.
 *   {string}   [name]              for reporting only
 *   {boolean}  [readOnly]          omit `write` entirely, i.e. ask rq.js's attach() for UNIT_RO
 *   {number}   [chunkBytes]        backing-store read granularity
 *   {number}   [cacheChunks]       resident chunks; the cache ceiling is chunkBytes * cacheChunks
 *   {number}   [maxOverlayBlocks]  dirty-block ceiling
 * @returns {Object} an rq.js image provider
 */
export function overlayImageProvider(opts)
{
    const byteLength = opts.byteLength >>> 0 === opts.byteLength ? opts.byteLength : Number(opts.byteLength);
    const readRaw = opts.readRaw;
    const chunkBytes = opts.chunkBytes || DEFAULT_CHUNK_BYTES;
    const cacheChunks = opts.cacheChunks || DEFAULT_CACHE_CHUNKS;
    const maxOverlayBlocks = opts.maxOverlayBlocks || DEFAULT_MAX_OVERLAY_BLOCKS;
    if (typeof readRaw !== "function") throw new Error("overlayImageProvider: readRaw is required");
    if (!(byteLength >= 0)) throw new Error("overlayImageProvider: byteLength must be a number");

    /* Insertion-ordered, which is what makes the eviction below FIFO without a second structure:
       a Map's iterator yields keys in insertion order, so `keys().next().value` is the oldest. */
    let cache = new Map();
    /* block index -> Uint8Array(512).  The COW overlay; see constraint 2 in the file header. */
    let overlay = new Map();
    let stats = {rawReads: 0, rawBytes: 0, cacheHits: 0, cacheMisses: 0, evictions: 0,
                 reads: 0, writes: 0, overlayBlocks: 0, overlayHighBlock: -1};

    function chunkAt(ci)
    {
        let hit = cache.get(ci);
        if (hit !== undefined) { stats.cacheHits++; return hit; }
        stats.cacheMisses++;
        let off = ci * chunkBytes;
        let want = Math.min(chunkBytes, Math.max(0, byteLength - off));
        let got = want > 0 ? readRaw(off, want) : new Uint8Array(0);
        stats.rawReads++;
        stats.rawBytes += got.length;
        /* A short backing read is EOF, not an error (rq.js's diskRead() header says so of pread(2)
           and this provider must present the same shape).  The chunk is padded so that every cache
           entry is exactly `want` bytes and callers never have to re-check. */
        let buf = got.length === want ? got : (() => { let b = new Uint8Array(want); b.set(got.subarray(0, Math.min(got.length, want))); return b; })();
        cache.set(ci, buf);
        if (cache.size > cacheChunks) {
            let oldest = cache.keys().next().value;
            cache.delete(oldest);
            stats.evictions++;
        }
        return buf;
    }

    /**
     * baseRead(offset, length, dst)
     *
     * The BACKING STORE only -- no overlay.  Returns bytes delivered, 0..length, short at EOF.
     */
    function baseRead(offset, length, dst)
    {
        let avail = Math.max(0, Math.min(length, byteLength - offset));
        let done = 0;
        while (done < avail) {
            let pos = offset + done;
            let ci = Math.floor(pos / chunkBytes);
            let within = pos - ci * chunkBytes;
            let chunk = chunkAt(ci);
            let n = Math.min(chunk.length - within, avail - done);
            if (n <= 0) break;                              /* the chunk was short: real EOF */
            dst.set(chunk.subarray(within, within + n), done);
            done += n;
        }
        return done;
    }

    /**
     * overlayHigh(offset, length)
     *
     * The highest byte offset, RELATIVE to `offset`, that a dirty block reaches inside the request
     * -- 0 if none does.  Separate from applyOverlay() because the caller has to ZERO the gap
     * between the backing store's EOF and this point BEFORE the blocks are painted; see constraint
     * 3 in the file header, and rq.js's diskRead(), which only zero-fills from the returned count.
     */
    function overlayHigh(offset, length)
    {
        if (overlay.size === 0) return 0;
        let first = Math.floor(offset / SECTOR), last = Math.floor((offset + length - 1) / SECTOR);
        let high = 0;
        for (let b = first; b <= last; b++) {
            if (!overlay.has(b)) continue;
            let to = Math.min(offset + length, b * SECTOR + SECTOR);
            if (to - offset > high) high = to - offset;
        }
        return high;
    }

    /**
     * applyOverlay(offset, length, dst)
     *
     * Paint every dirty block that intersects [offset, offset+length) over `dst`.
     *
     * Iterating the REQUEST's blocks rather than the overlay's is what keeps this O(request)
     * instead of O(dirty set): a boot leaves tens of thousands of blocks dirty and every one of
     * them would otherwise be visited on every read.
     */
    function applyOverlay(offset, length, dst)
    {
        if (overlay.size === 0) return;
        let first = Math.floor(offset / SECTOR), last = Math.floor((offset + length - 1) / SECTOR);
        for (let b = first; b <= last; b++) {
            let blk = overlay.get(b);
            if (blk === undefined) continue;
            let blkStart = b * SECTOR;
            let from = Math.max(offset, blkStart);
            let to = Math.min(offset + length, blkStart + SECTOR);
            dst.set(blk.subarray(from - blkStart, to - blkStart), from - offset);
        }
    }

    let prov = {
        byteLength,
        path: opts.name || "(browser image)",
        writable: !opts.readOnly,
        filesystemBytes: undefined,                         /* filled in below, once `read` exists */

        /**
         * read(offset, length, dst)
         *
         * @returns {number} bytes DELIVERED, 0..length -- rq.js zero-fills whatever is left.
         */
        read(offset, length, dst) {
            stats.reads++;
            let n = baseRead(offset, length, dst);
            let high = overlayHigh(offset, length);
            if (high > n) {
                /* The overlay reached past what the backing store had.  rq.js only zero-fills from
                   the returned count onwards, so the hole between them is ours to zero -- and it
                   must be zeroed BEFORE the dirty blocks are painted over it. */
                dst.fill(0, n, high);
                n = high;
            }
            applyOverlay(offset, length, dst);
            return n;
        },

        /** Backing-store bytes only, ignoring the overlay -- tests/overlayprovider.js's control. */
        readBase: baseRead,

        stats() {
            return Object.assign({}, stats, {
                overlayBlocks: overlay.size,
                overlayBytes: overlay.size * SECTOR,
                overlayCeilingBytes: maxOverlayBlocks * SECTOR,
                cacheResidentBytes: cache.size * chunkBytes,
                cacheCeilingBytes: cacheChunks * chunkBytes
            });
        },

        close() { cache.clear(); overlay.clear(); }
    };

    /* *** PRESENT ONLY WHEN WRITABLE, because its ABSENCE is the signal rq.js's attach() reads. ***
       Exactly the rule tests/mscpharness.js's fileImageProvider() follows for the same reason. */
    if (!opts.readOnly) {
        /**
         * write(offset, length, src)
         *
         * Copy-on-write into the overlay.  Always stores the WHOLE request -- pwrite(2) extends a
         * regular file past its end and rq.js's diskWrite() throws RQUnimplemented on a short store,
         * so a provider that refused to grow would turn a legitimate VMS write into a crash.
         *
         * @returns {number} bytes stored; always `length`, or it throws
         */
        function write(offset, length, src) {
            stats.writes++;
            let first = Math.floor(offset / SECTOR), last = Math.floor((offset + length - 1) / SECTOR);
            /* Checked BEFORE the first block is stored, so the bound cannot be exceeded by the
               request that trips it. */
            let fresh = 0;
            for (let b = first; b <= last; b++) if (!overlay.has(b)) fresh++;
            if (overlay.size + fresh > maxOverlayBlocks) {
                throw new Error(`imageprovider: the copy-on-write overlay is full -- ` +
                    `${overlay.size} of ${maxOverlayBlocks} blocks (${(maxOverlayBlocks * SECTOR / (1 << 20)).toFixed(0)} MiB) ` +
                    `are dirty and this write needs ${fresh} more.  Raise maxOverlayBlocks or give ` +
                    `the machine a persistent backing store (pcjsvax-367)`);
            }
            for (let b = first; b <= last; b++) {
                let blk = overlay.get(b);
                if (blk === undefined) {
                    /* A PARTIAL block write must not lose the bytes it does not cover, so a fresh
                       overlay block is seeded from the backing store first. */
                    blk = new Uint8Array(SECTOR);
                    baseRead(b * SECTOR, SECTOR, blk);
                    overlay.set(b, blk);
                    if (b > stats.overlayHighBlock) stats.overlayHighBlock = b;
                }
                let blkStart = b * SECTOR;
                let from = Math.max(offset, blkStart);
                let to = Math.min(offset + length, blkStart + SECTOR);
                blk.set(src.subarray(from - offset, to - offset), from - blkStart);
            }
            return length;
        };
        prov.write = write;
    }

    /* get_filesystem_size(), through THIS provider's own read path -- so the parse is served by the
       same lazy chunk cache as everything else and a 1 GB container costs three 64 KiB reads to
       identify.  See modules/v2/ods2.js: it returns undefined for anything that is not ODS-2, and
       never throws, which is what makes an arbitrary user-picked file safe to hand it. */
    prov.filesystemBytes = ods2VolumeBytesFrom(byteLength, (lbn) => {
        let b = new Uint8Array(SECTOR);
        return baseRead(lbn * SECTOR, SECTOR, b) === SECTOR ? b : null;
    });

    return prov;
}

export { SECTOR, DEFAULT_CHUNK_BYTES, DEFAULT_CACHE_CHUNKS, DEFAULT_MAX_OVERLAY_BLOCKS };
export default overlayImageProvider;
