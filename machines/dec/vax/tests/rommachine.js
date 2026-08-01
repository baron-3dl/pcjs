/**
 * @fileoverview The ROM boot-entry machine, built ONCE and shared by every differential that boots
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
 *               the KA655 console ROM
 *
 * WHAT THIS IS
 * ------------
 * pcjsvax-bfb.  This function's body was tests/romdiff.js's own `makeMachine()`, verbatim, and is
 * moved here UNCHANGED so that romdiff.js and tests/conoutdiff.js grade the SAME machine rather
 * than two hand-kept copies of it.  HANDOFF.md standing rule 7 -- "scope lives in code, not
 * comments" -- was earned by exactly this failure mode: two modules' idea of who owned what drifted
 * apart and six opcodes fell through the gap.  A second differential that boots the same ROM from
 * the same entry state against the same oracle must not carry its own transcription of which
 * devices are decoded; if it did, one file could gain a device and the other could keep reporting
 * on a machine that no longer exists.
 *
 * `devices` is the OTHER reason this is shared: it is built by the SAME statements that construct
 * the machine, so a census over it (tests/conoutdiff.js's DEVICE phase) cannot enumerate a device
 * that is not in the machine, and cannot miss one that is -- HANDOFF.md standing rule 5, "never
 * hand-enumerate a scope list; derive it programmatically."  The names are the JS class names,
 * taken from the constructor, not typed in.
 */

import BusVAX from "../modules/v2/bus.js";
import MemoryVAX from "../modules/v2/memory.js";
import { VAX } from "../modules/v2/defines.js";
import CPUStateVAX from "../modules/v2/cpustate.js";
import SSCVAX from "../modules/v2/ssc.js";
import NVRVAX from "../modules/v2/nvr.js";
import CQBICVAX, { CQMAPVAX, CQMVAX, CQMAP_BASE, CQMAPSIZE } from "../modules/v2/cqbic.js";
import KA655VAX from "../modules/v2/ka655.js";
import CMCTLVAX, { CMCTL_BASE, CMCTL_LENGTH } from "../modules/v2/cmctl.js";
import CQIPCVAX, { DBLVAX, CQIPC_BASE, CQIPC_SIZE, DBL_BASE, DBL_SIZE } from "../modules/v2/cqipc.js";
import CDGVAX from "../modules/v2/cdg.js";
import ConsoleVAX from "../modules/v2/console.js";
import ClkVAX, { IPL_CLK_ABS, INT_V_CLK } from "../modules/v2/clk.js";
import { makeIprDevice } from "../modules/v2/iprdevice.js";
import { SCB } from "../modules/v2/exc.js";

/** 16MB, the SIMH microvax3900 default -- the same size every other differential in this tree uses. */
const MEMSIZE = 0x01000000;

/**
 * makeRomMachine(romBytes, fOmitCdg, fOmitCqm)
 *
 * RAM at 0 (the KA655's own system memory, which the ROM's power-up self-tests size and probe --
 * without it the very first instructions would fault on RAM before ever reaching the ROM's own
 * device probing) plus the ROM itself, decoded via BusVAX.addRom(), plus the SSC base register,
 * decoded via BusVAX.addSsc() (pcjsvax-320), plus the REG_BASE sub-devices (BusVAX.addRegBlock(),
 * pcjsvax-bfb), the cache diagnostic space (BusVAX.addCdg(), pcjsvax-0b7) and the CQBIC doorbell's
 * two bytes of the Qbus I/O page (BusVAX.addIoPage(), pcjsvax-b8a).
 *
 * `fOmitCdg` (pcjsvax-fe7, romdiff.js's --no-cdg) leaves the cache-diagnostic space UNDECODED.  It
 * touches exactly one line of this function and nothing anywhere else -- deliberately, because
 * romdiff.js's regression floor uses it to hand the walk rule a REAL hardware gap and then requires
 * the walk to stop at it and name it.  A floor that reached into the walk rule itself would grade
 * nothing.
 *
 * `fOmitCqm` (pcjsvax-aa5) leaves the CQBIC scatter/gather map at CQMAPBASE and the Qbus memory
 * window at CQMBASE UNDECODED -- the exact state this machine was in before pcjsvax-5c1, and the
 * state MEASURED 2026-07-29 to put `?80` back into the ROM's console stream.  It exists for the
 * same reason `fOmitCdg` does: so a gate can hand the ROM a REAL missing decode and require an
 * instrument to notice.  conoutdiff.js's `cqm-window-undecoded` mutation is its only caller.
 *
 * `opts` (pcjsvax-319) is the ONE additive parameter, and it carries exactly two fields:
 *
 *   `memSize`   RAM size in bytes, defaulting to MEMSIZE.  It is a PARAMETER rather than a second
 *               constant because pcjsvax-59f fixed 16MB as the ROM SELF-TEST target while the
 *               OpenVMS oracle reference (pcjsvax-459) was captured at `set cpu 128m`, and a second
 *               hardcoded size is precisely the drift standing rule 7 was earned by.  Every
 *               existing caller passes nothing and gets 16MB.  Note it is threaded to ALL FOUR
 *               places the old constant was read -- addMemory(), CQBICVAX and CMCTLVAX both take it
 *               as "the size passed to addMemory()", and a machine whose CMCTL disagrees with its
 *               own RAM reports a memory size the ROM's self-test then contradicts.
 *
 *   `qbus`      a FUNCTION, called with {bus, cpu, cqbic, memSize}, returning
 *               {windows: [{base, length, dev}], tickDev}.  Its windows are appended to the ONE
 *               addIoPage() call below and its devices join the `devices` roster; `tickDev` becomes
 *               `cpu.qbus`, the per-instruction event hook cpustate.js already calls.  It is a
 *               CALLBACK rather than a device argument so that this file -- which every ROM
 *               differential imports -- does not gain an import of rq.js for the sake of one
 *               caller: tests/vmsbootprobe.js passes the RQVAX construction it takes from
 *               tests/mscpharness.js, and nothing else in the tree passes anything.
 *
 * @param {Uint8Array} romBytes
 * @param {boolean} [fOmitCdg]
 * @param {boolean} [fOmitCqm]
 * @param {Object} [opts] {memSize, qbus}
 * @returns {Object} {bus, cpu, consoleDev, clk, ssc, nvr, cqbic, cqipc, dbl, cmctl, ka655, cdg,
 *   cqmap, cqm, memSize, devices}
 */
function makeRomMachine(romBytes, fOmitCdg = false, fOmitCqm = false, opts = {})
{
    let memSize = (opts.memSize === undefined) ? MEMSIZE : (opts.memSize >>> 0);
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, memSize, MemoryVAX.TYPE.RAM);
    bus.addRom(romBytes);
    /* SSCVAX's REG_BTO and CQBICVAX's DSER/MEAR are the SAME state pcjsvax-446/d22 already track on
       cpu.exc (sscBto/cqDser/cqMear) -- see ssc.js's and cqbic.js's file headers -- so both devices
       are constructed with a `cpu.exc` reference.  CPUStateVAX's constructor builds `exc` before
       setBus() is ever called, so cpu can be constructed first here without a bus yet attached. */
    let cpu = new CPUStateVAX({id: "cpu"});
    /* ConsoleVAX (pcjsvax-bfb) models RXCS/RXDB/TXCS/TXDB ONCE and is wired into BOTH address
       paths: setIPRDevice() below (the IPR path) and SSCVAX's third constructor argument (the SSC
       mirror) -- see console.js's and ssc.js's file headers.
       ClkVAX (pcjsvax-954) owns MT.ICCS/MT.TODR.  exc.js's setIPRDevice() accepts only ONE device,
       so both are combined behind iprdevice.js's makeIprDevice() -- see that file's header for why
       clk is NOT also ticked through it (cpu.clk already ticks it once per instruction via
       cpustate.js's own hook; ticking it again here would double clk_svc's effective rate). */
    let consoleDev = new ConsoleVAX(cpu.exc);
    let clk = new ClkVAX(cpu.exc);
    /* SSCVAX (pcjsvax-055) self-wires T0/T1's interrupt sources in its own constructor (see
       ssc.js's file header) -- kept as a local so `cpu.tmr` below can reach it for the
       per-instruction tick hook, the same reason `clk` is kept as a local for `cpu.clk`. */
    let ssc = new SSCVAX(cpu.exc, consoleDev);
    let nvr = new NVRVAX();
    bus.addSsc(ssc, nvr);
    /* KA655VAX is kept as a local because CDGVAX needs the SAME instance: vax_sysdev.c's cdg_rd()
       writes the KA655 CACR as a side effect of every cache-diagnostic read (pcjsvax-0b7 -- see
       cdg.js's and ka655.js's headers, and tests/cdgdiff.js, which grades that side effect against
       the live oracle).  Two instances would leave the ROM reading a CACR that never saw the CDG
       traffic. */
    let ka655 = new KA655VAX();
    /* `bus` and MEMSIZE are REQUIRED for the map window and the Qbus memory window (pcjsvax-5c1):
       without them cqbic.js's requireBus() throws on every map access, and memSize 0 makes isMem()
       false for every address.  This machine passed one argument until pcjsvax-ee7 measured that
       the ROM's self-test 80 programs all 8,192 map entries and then walks CQMBASE. */
    let cqbic = new CQBICVAX(cpu.exc, bus, memSize);
    /* CMCTLVAX (pcjsvax-622) is the memory controller's register file.  It takes MEMSIZE because
       SIMH's cmctl_rd()/cmctl_wr() read `MEMSIZE` in two places -- register 18's KA655X test and
       the signature request's ADDR_IS_MEM() -- and MEMSIZE is `cpu_unit.capac`, i.e. exactly the
       size passed to addMemory() above.  Its base and length are IMPORTED, not restated: the
       length is a computation over the bank geometry (see cmctl.js's header), and a second
       hand-written copy of it here is precisely the drift standing rule 7 was earned by. */
    let cmctl = new CMCTLVAX(memSize);
    /* CQIPCVAX (pcjsvax-b8a) is `cq_ipc`, ONE register with TWO address paths -- the local register
       at CQIPCBASE and the Qbus I/O-page doorbell the QBA's DIB autoconfigures (see cqipc.js's
       header).  Both mounts below share this one instance for the same reason console.js's IPR and
       SSC paths share one ConsoleVAX: two models of a W1C register would drift, and the W1C bit is
       what makes the drift observable.  setIpc() wires cqbic_wr()'s DSER<SME> cross-clear, the one
       side effect on cq_ipc that lives outside cqipc.js. */
    let cqipc = new CQIPCVAX();
    cqbic.setIpc(cqipc);
    /* CQMAP (pcjsvax-ee7/aa5) and the Qbus MEMORY window at CQMBASE (pcjsvax-5c1).  regblock.js's
       header always listed CQMAP as one of the five regtable[] sub-devices; this machine simply
       never mounted it, so every ROM access to CQMAPBASE faulted and self-test 80 reported `?80`.
       The ROM programs all 8,192 map entries and then walks CQMBASE.

       Both are held in NAMED LOCALS rather than constructed inline in the mount lists, so that the
       `devices` roster below can carry them: they were the two devices the ROM drives hardest
       (57,536 calls over a 12,000,000-instruction walk, MEASURED 2026-07-29) and the ONLY two the
       roster omitted, which made a census structurally unable to see the decode whose absence
       produces `?80`. */
    let cqmap = null, cqm = null;
    let regBlocks = [
        {base: VAX.PHYSMEM.REG_BASE >>> 0, length: 0x14, dev: cqbic},
        {base: CMCTL_BASE, length: CMCTL_LENGTH, dev: cmctl},
        {base: (VAX.PHYSMEM.REG_BASE + 0x4000) >>> 0, length: 8, dev: ka655},
        {base: CQIPC_BASE, length: CQIPC_SIZE, dev: cqipc}
    ];
    if (!fOmitCqm) {
        cqmap = new CQMAPVAX(cqbic);
        regBlocks.push({base: CQMAP_BASE, length: CQMAPSIZE, dev: cqmap});
    }
    bus.addRegBlock(regBlocks);
    /* Decoded with regblock.js's dispatcher for the same reason addIoPage() reuses it: anything the
       device declines falls through to the identical unbacked path it took before. */
    if (!fOmitCqm) {
        cqm = new CQMVAX(cqbic);
        bus.addCqm([{base: VAX.PHYSMEM.CQM_BASE >>> 0, length: VAX.PHYSMEM.CQM_LENGTH, dev: cqm}]);
    }
    /* The Qbus I/O page, decoded for the FIRST time in this tree, and for exactly DBL_SIZE bytes.
       Everything else in the range keeps faulting through the identical path it took before -- see
       bus.js's addIoPage() and cqipc.js's SCOPE section. */
    let dbl = new DBLVAX(cqipc);
    /* `opts.qbus`'s windows join the SAME addIoPage() call: bus.js's addIoPage() installs ONE
       controller over the whole page and a second call would replace the first, so a caller that
       mounts a Qbus device cannot do it from outside this function. */
    let qbusExtra = opts.qbus ? opts.qbus({bus, cpu, cqbic, memSize}) : null;
    let qbusWindows = (qbusExtra && qbusExtra.windows) || [];
    bus.addIoPage([{base: DBL_BASE, length: DBL_SIZE, dev: dbl}, ...qbusWindows]);
    if (qbusExtra && qbusExtra.tickDev) cpu.qbus = qbusExtra.tickDev;
    let cdg = null;
    if (!fOmitCdg) { cdg = new CDGVAX(ka655); bus.addCdg(cdg); }
    cpu.setBus(bus);
    cpu.exc.setIPRDevice(makeIprDevice(clk, consoleDev));
    cpu.exc.addInterruptSource(IPL_CLK_ABS, INT_V_CLK, SCB.INTTIM);
    cpu.clk = clk;
    cpu.tmr = ssc;
    /* Derived from the constructions above, in this same function, so it can neither list a device
       the machine does not have nor omit one it does (see the file header).  `cdg`, `cqmap` and
       `cqm` are present only when they were actually decoded, which is what makes `fOmitCdg` and
       `fOmitCqm` visible to a census.

       That claim was FALSE until pcjsvax-aa5: this roster omitted CQMAPVAX and CQMVAX even when the
       machine had them, because both were constructed inline inside the mount lists.  A comment
       claiming more than the code did is HANDOFF.md standing rule 12, and the consequence was
       measurable -- deleting both mounts reintroduced `?80` and NOT ONE of cqmmapdiff, cqmerrdiff,
       qdmadiff, romdiff or conoutdiff failed. */
    let devices = [consoleDev, clk, ssc, nvr, cqbic, cqipc, dbl, cmctl, ka655, cdg, cqmap, cqm]
        .concat(qbusWindows.map((w) => w.dev))
        .filter((d) => d !== null)
        .map((d) => ({name: d.constructor.name, dev: d}));
    return {bus, cpu, consoleDev, clk, ssc, nvr, cqbic, cqipc, dbl, cmctl, ka655, cdg, cqmap, cqm,
            memSize, devices};
}

export { makeRomMachine, MEMSIZE };
export default makeRomMachine;
