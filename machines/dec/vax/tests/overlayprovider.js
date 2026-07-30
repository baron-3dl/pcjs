/**
 * @fileoverview pcjsvax-ae1 -- grade browser/imageprovider.js against tests/mscpharness.js's
 *               fileImageProvider() over the SAME container
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 *
 * WHY IT EXISTS
 * -------------
 * browser/imageprovider.js is the half of the disk path that only ever runs inside a Worker, and a
 * provider that only exists inside a Worker cannot be graded by anything in this tree -- so it is
 * written backend-agnostic (it takes a `readRaw`) and this file hands it an `fs` backend and
 * requires it to be INDISTINGUISHABLE from the Node provider rq.js already boots from.
 *
 * THE ORACLE IS fileImageProvider(), not a hand-written expectation.  Every read case asks both
 * providers for the same bytes at the same offset and requires the same delivered count and the
 * same bytes -- including the short-read-at-EOF case, which is the one a real OpenVMS volume
 * presents on every run (the volume declares more sectors than the container holds).
 *
 * The WRITE cases have no oracle in the same shape, because the whole point of the overlay is that
 * it does NOT do what pwrite(2) does: it must leave the user's file untouched.  So they are graded
 * as invariants -- read-back, partial-block preservation, past-EOF readability, the container's
 * mtime and contents unchanged, and the ceiling refusing by name.
 *
 * USAGE
 *   node machines/dec/vax/tests/overlayprovider.js [--volume PATH]
 *
 * Exits 0 only if every check passes.  With --volume it additionally requires the two providers to
 * agree on `filesystemBytes` for a REAL ODS-2 volume, which is the number rq.js's attach() does
 * autosize arithmetic on; without it that check is reported SKIPPED by name (standing rule 6) and
 * the run still fails if anything else fails.
 */

import fs from "fs";
import os from "os";
import path from "path";

import { fileImageProvider, ods2VolumeBytes } from "./mscpharness.js";
import { overlayImageProvider, SECTOR } from "../browser/imageprovider.js";

function getArg(name, def) { let i = process.argv.indexOf(name); return i >= 0 ? process.argv[i + 1] : def; }

let passed = 0, failed = 0;
function check(name, cond, detail)
{
    if (cond) { passed++; console.log(`  PASS  ${name}`); }
    else { failed++; console.log(`  FAIL  ${name}${detail ? " -- " + detail : ""}`); }
}
function same(a, b) { if (a.length !== b.length) return false; for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false; return true; }

/** An fs-backed `readRaw`, i.e. the shape browser/vaxworker.js's FileReaderSync backend has. */
function fsReadRaw(fd)
{
    return (offset, length) => {
        let b = Buffer.alloc(length);
        let n = fs.readSync(fd, b, 0, length, offset);
        return new Uint8Array(b.buffer, b.byteOffset, n);
    };
}

function main()
{
    let dir = fs.mkdtempSync(path.join(os.tmpdir(), "overlayprov-"));
    let scratch = path.join(dir, "scratch.dsk");
    /* 1000 sectors of a pattern that differs in every byte position of every block, so a misplaced
       read is a mismatch rather than a coincidence. */
    const NSECT = 1000;
    let img = Buffer.alloc(NSECT * SECTOR);
    for (let b = 0; b < NSECT; b++) {
        for (let i = 0; i < SECTOR; i++) img[b * SECTOR + i] = (b * 7 + i * 31 + (i >> 3)) & 0xFF;
    }
    fs.writeFileSync(scratch, img);

    console.log(`overlayprovider (pcjsvax-ae1) -- browser/imageprovider.js vs fileImageProvider()`);
    console.log(`  scratch   ${scratch} (${img.length} bytes, ${NSECT} sectors)`);

    let node = fileImageProvider(scratch);
    let fd = fs.openSync(scratch, "r");
    /* Deliberately TINY cache bounds, so the eviction path runs on a 500 KB container rather than
       only on a 1 GB one -- the bound is the thing being graded, and a cache big enough to hold the
       whole container would grade nothing. */
    let brow = overlayImageProvider({
        byteLength: fs.statSync(scratch).size, readRaw: fsReadRaw(fd), name: scratch,
        chunkBytes: 4096, cacheChunks: 8, maxOverlayBlocks: 16
    });

    check("byteLength agrees", node.byteLength === brow.byteLength, `${node.byteLength} vs ${brow.byteLength}`);
    check("filesystemBytes agrees on a non-ODS-2 container",
        node.filesystemBytes === brow.filesystemBytes && brow.filesystemBytes === undefined,
        `${node.filesystemBytes} vs ${brow.filesystemBytes}`);
    check("writable, so rq.js's attach() will NOT force UNIT_RO", brow.writable === true && typeof brow.write === "function");

    /* ---- READS: every case asks both providers and requires identical answers ---- */
    let cases = [
        {name: "sector 0", off: 0, len: SECTOR},
        {name: "one sector mid-container", off: 500 * SECTOR, len: SECTOR},
        {name: "64-sector transfer", off: 100 * SECTOR, len: 64 * SECTOR},
        {name: "a transfer spanning many cache chunks", off: 3 * SECTOR, len: 200 * SECTOR},
        {name: "the last full sector", off: (NSECT - 1) * SECTOR, len: SECTOR},
        {name: "a transfer STRADDLING the end of the container", off: (NSECT - 4) * SECTOR, len: 16 * SECTOR},
        {name: "a transfer wholly past the end of the container", off: (NSECT + 10) * SECTOR, len: 8 * SECTOR},
        {name: "an unaligned range", off: 12345, len: 6789}
    ];
    for (let c of cases) {
        let a = Buffer.alloc(c.len), b = new Uint8Array(c.len);
        a.fill(0xAA); b.fill(0xAA);
        let na = node.read(c.off, c.len, a);
        let nb = brow.read(c.off, c.len, b);
        check(`read: ${c.name} -- delivered count`, na === nb, `node ${na} vs browser ${nb}`);
        check(`read: ${c.name} -- bytes`, same(new Uint8Array(a.buffer, a.byteOffset, na), b.subarray(0, nb)));
    }

    /* Reading the whole container in one pass must not make the cache resident-set exceed its
       ceiling.  This is constraint 1 -- "never load the whole image" -- as an assertion. */
    {
        let scratchBuf = new Uint8Array(SECTOR);
        for (let b = 0; b < NSECT; b++) brow.read(b * SECTOR, SECTOR, scratchBuf);
        let s = brow.stats();
        check("cache resident set stays inside its ceiling after reading the WHOLE container",
            s.cacheResidentBytes <= s.cacheCeilingBytes,
            `${s.cacheResidentBytes} > ${s.cacheCeilingBytes}`);
        check("cache actually evicted (i.e. the bound was exercised, not merely large enough)",
            s.evictions > 0, `evictions=${s.evictions}`);
    }

    /* ---- WRITES ---- */
    let mtimeBefore = fs.statSync(scratch).mtimeMs;
    let sha = () => fs.readFileSync(scratch);
    let bytesBefore = sha();
    {
        let src = new Uint8Array(SECTOR).fill(0x5A);
        check("write of a whole sector reports the whole length", brow.write(7 * SECTOR, SECTOR, src) === SECTOR);
        let got = new Uint8Array(SECTOR);
        brow.read(7 * SECTOR, SECTOR, got);
        check("a written sector reads back written", same(got, src));
        let neighbour = new Uint8Array(SECTOR);
        brow.read(8 * SECTOR, SECTOR, neighbour);
        check("the NEXT sector is untouched", same(neighbour, new Uint8Array(img.subarray(8 * SECTOR, 9 * SECTOR))));
    }
    {
        /* A partial-block write must not lose the bytes it does not cover: the fresh overlay block
           is seeded from the backing store first. */
        let src = new Uint8Array(16).fill(0xC3);
        brow.write(20 * SECTOR + 100, 16, src);
        let got = new Uint8Array(SECTOR);
        brow.read(20 * SECTOR, SECTOR, got);
        let want = new Uint8Array(img.subarray(20 * SECTOR, 21 * SECTOR));
        want.set(src, 100);
        check("a PARTIAL-sector write preserves the rest of the sector", same(got, want));
    }
    {
        /* Autosize routinely leaves the unit larger than its container, and VMS writes there. */
        let off = (NSECT + 5) * SECTOR;
        let src = new Uint8Array(SECTOR).fill(0x99);
        check("a write PAST the end of the container is accepted in full", brow.write(off, SECTOR, src) === SECTOR);
        let got = new Uint8Array(2 * SECTOR).fill(0xAA);
        let n = brow.read(off - SECTOR, 2 * SECTOR, got);
        check("...and reads back, with the sector before it zero-filled",
            n === 2 * SECTOR && same(got.subarray(SECTOR), src) && got.subarray(0, SECTOR).every((v) => v === 0),
            `delivered ${n}`);
    }
    check("the user's file is byte-for-byte unchanged", same(new Uint8Array(sha()), new Uint8Array(bytesBefore)));
    check("the user's file's mtime is unchanged", fs.statSync(scratch).mtimeMs === mtimeBefore);
    {
        /* The bound must refuse BY NAME rather than growing until the tab dies. */
        let src = new Uint8Array(SECTOR).fill(1), threw = null;
        try { for (let b = 100; b < 200; b++) brow.write(b * SECTOR, SECTOR, src); }
        catch (e) { threw = String(e.message || e); }
        check("the overlay refuses past maxOverlayBlocks, by name",
            threw !== null && /copy-on-write overlay is full/.test(threw), threw || "(did not throw)");
        let s = brow.stats();
        check("...and the dirty set never exceeded the bound", s.overlayBlocks <= 16, `${s.overlayBlocks}`);
    }
    {
        /* readOnly omits `write` ENTIRELY -- the signal rq.js's attach() reads for UNIT_RO. */
        let ro = overlayImageProvider({byteLength: 512, readRaw: fsReadRaw(fd), readOnly: true});
        check("readOnly provider carries NO write member at all",
            ro.write === undefined && ro.writable === false);
    }

    /* ---- the real volume, when one is available ---- */
    let volume = getArg("--volume", null);
    if (!volume) {
        console.log(`  SKIP  filesystemBytes on a real ODS-2 volume -- pass --volume PATH to run it`);
    } else if (!fs.existsSync(volume)) {
        check(`the volume ${volume} exists`, false);
    } else {
        let vfd = fs.openSync(volume, "r");
        let want = ods2VolumeBytes(vfd);
        let vprov = overlayImageProvider({byteLength: fs.statSync(volume).size, readRaw: fsReadRaw(vfd), name: volume});
        check(`filesystemBytes agrees on the real volume (${want})`, vprov.filesystemBytes === want,
            `node ${want} vs browser ${vprov.filesystemBytes}`);
        check("the ODS-2 parse cost only a handful of backing reads, not a whole-image load",
            vprov.stats().rawBytes < 1 << 20, `${vprov.stats().rawBytes} bytes read`);
        fs.closeSync(vfd);
    }

    node.close(); brow.close(); fs.closeSync(fd);
    fs.rmSync(dir, {recursive: true, force: true});

    console.log(`\nPASSED ${passed} / FAILED ${failed}`);
    process.exit(failed ? 1 : 0);
}

main();
