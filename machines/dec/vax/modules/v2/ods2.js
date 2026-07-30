/**
 * @fileoverview get_ods2_filesystem_size() (sim_disk.c:1257-1340), over an ABSTRACT block reader
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY THIS MOVED HERE (pcjsvax-ae1)
 * ---------------------------------
 * The walk below WAS tests/mscpharness.js's `ods2VolumeBytes(fd)`, and it was written against a Node
 * file descriptor: `fs.fstatSync`, `fs.readSync`, and Buffer's `readUInt16LE`/`readUInt32LE`.  None
 * of those exist in a browser, and a browser-supplied disk image needs the SAME number, because
 * rq.js's attach() takes `provider.filesystemBytes` and does sim_disk_attach_ex2()'s autosize
 * arithmetic on it -- see rq.js's attach() and the AUTOSIZE comment in it.  A unit attached without
 * it takes the OTHER arm and can end up a different size than the same container attached under
 * Node, which is a difference produced entirely by which harness opened the file.
 *
 * So the walk is here ONCE, parameterised by a `readBlock(lbn)` that returns 512 bytes or null, and
 * mscpharness.js's `ods2VolumeBytes(fd)` is now a four-line `fs` adapter over it.  HANDOFF.md
 * standing rule 7 -- a second hand-written copy of a parser is exactly the drift that rule exists
 * for, and here the two copies would have been in two languages of the same tree.
 *
 * IT STAYS OUT OF rq.js.  A disk controller does not parse volumes; sim_disk does, before the
 * controller ever sees the unit, and hands the result to autosize.  rq.js's own header records that
 * exclusion (pcjsvax-f52) and it is unchanged by this move.
 *
 * SECURITY NOTE (pcjsvax-1ad).  Every input to this function is attacker-controlled the moment a
 * user picks an arbitrary file in a browser.  The walk therefore treats EVERY field as hostile: the
 * three retrieval-pointer formats are bounds-checked, `readBlock()` is required to refuse an LBN
 * outside the container, and the final answer is sanity-bounded against the container size.  The
 * function's contract is that it returns `undefined` rather than throwing, for ANY input.
 */

/**
 * ods2VolumeBytesFrom(size, readBlock)
 *
 * get_ods2_filesystem_size() reduced to the one number it returns: `Scb.scb_l_volsize * 512`, the
 * size the VOLUME declares for itself.
 *
 * The walk is the C's: HOME BLOCK at LBN 1 -> the BITMAP.SYS file header at
 * `ibmaplbn + ibmapsize + 1` -> its first retrieval pointer -> the STORAGE CONTROL BLOCK.  The
 * validity tests are the C's too, minus the two ODS checksums: what they are here for is to REFUSE
 * a container that is not an ODS-2 volume, and a pattern-filled scratch file fails the structure
 * level, the cluster factor and the bitmap fields long before a checksum would matter.  Returning
 * undefined is the "unrecognised" answer and it selects autosize's OTHER arm, so a false positive
 * would change a unit's size -- which is why every field is checked.
 *
 * @param {number} size the container's size in BYTES
 * @param {function(number):(Uint8Array|null)} readBlock (lbn) -> 512 bytes, or null if unreadable
 * @returns {number|undefined} the volume's size in BYTES, or undefined if this is not ODS-2
 */
export function ods2VolumeBytesFrom(size, readBlock)
{
    try {
        const rd = (lbn) => {
            if (!Number.isFinite(lbn) || lbn < 0 || (lbn + 1) * 512 > size) return null;
            let b = readBlock(lbn);
            return (b && b.length === 512) ? new DataView(b.buffer, b.byteOffset, 512) : null;
        };
        let H = rd(1);
        if (!H) return undefined;
        let strucver = H.getUint8(12), struclev = H.getUint8(13), cluster = H.getUint16(14, true);
        let ibmapvbn = H.getUint16(22, true), ibmaplbn = H.getUint32(24, true);
        let maxfiles = H.getUint32(28, true), ibmapsize = H.getUint16(32, true);
        let resfiles = H.getUint16(34, true);
        if (H.getUint32(0, true) === 0 || H.getUint32(4, true) === 0 || H.getUint32(8, true) === 0) return undefined;
        if ((struclev !== 2 && struclev !== 5) || strucver === 0 || cluster === 0) return undefined;
        if (ibmapvbn === 0 || ibmaplbn === 0 || ibmapsize === 0) return undefined;
        if (resfiles < 5 || resfiles >= maxfiles) return undefined;
        let hdr = rd(ibmaplbn + ibmapsize + 1);
        if (!hdr) return undefined;
        let o = hdr.getUint8(1) * 2;                        /* fh2_b_mpoffset, in WORDS */
        if (o < 4 || o > 500) return undefined;
        let w0 = hdr.getUint16(o, true), fmt = (w0 >>> 14) & 3;
        if (fmt === 0) { o += 2; w0 = hdr.getUint16(o, true); fmt = (w0 >>> 14) & 3; }  /* placement */
        let scbLbn;
        if (fmt === 1) scbLbn = (((w0 >>> 8) & 0x3F) << 16) + hdr.getUint16(o + 2, true);
        else if (fmt === 2) scbLbn = (hdr.getUint16(o + 4, true) << 16) + hdr.getUint16(o + 2, true);
        else if (fmt === 3) scbLbn = hdr.getUint32(o + 4, true);
        else return undefined;
        let S = rd(scbLbn);
        if (!S) return undefined;
        if (S.getUint8(0) !== strucver || S.getUint8(1) !== struclev ||
            S.getUint16(2, true) !== cluster) return undefined;
        let volsize = S.getUint32(4, true);
        /* A volume smaller than one cluster, or wildly larger than the container, is a misparse. */
        if (volsize === 0 || volsize * 512 > size * 4) return undefined;
        return volsize * 512;
    } catch (e) {
        return undefined;
    }
}

export default ods2VolumeBytesFrom;
