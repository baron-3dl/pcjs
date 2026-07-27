/**
 * @fileoverview The ROM boot-entry machine, built ONCE and shared by every differential that boots
 *               the KA655 console ROM
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
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
import CQBICVAX from "../modules/v2/cqbic.js";
import KA655VAX from "../modules/v2/ka655.js";
import CDGVAX from "../modules/v2/cdg.js";
import ConsoleVAX from "../modules/v2/console.js";
import ClkVAX, { IPL_CLK_ABS, INT_V_CLK } from "../modules/v2/clk.js";
import { makeIprDevice } from "../modules/v2/iprdevice.js";
import { SCB } from "../modules/v2/exc.js";

/** 16MB, the SIMH microvax3900 default -- the same size every other differential in this tree uses. */
const MEMSIZE = 0x01000000;

/**
 * makeRomMachine(romBytes, fOmitCdg)
 *
 * RAM at 0 (the KA655's own system memory, which the ROM's power-up self-tests size and probe --
 * without it the very first instructions would fault on RAM before ever reaching the ROM's own
 * device probing) plus the ROM itself, decoded via BusVAX.addRom(), plus the SSC base register,
 * decoded via BusVAX.addSsc() (pcjsvax-320), plus the REG_BASE sub-devices (BusVAX.addRegBlock(),
 * pcjsvax-bfb) and the cache diagnostic space (BusVAX.addCdg(), pcjsvax-0b7).
 *
 * `fOmitCdg` (pcjsvax-fe7, romdiff.js's --no-cdg) leaves the cache-diagnostic space UNDECODED.  It
 * touches exactly one line of this function and nothing anywhere else -- deliberately, because
 * romdiff.js's regression floor uses it to hand the walk rule a REAL hardware gap and then requires
 * the walk to stop at it and name it.  A floor that reached into the walk rule itself would grade
 * nothing.
 *
 * @param {Uint8Array} romBytes
 * @param {boolean} [fOmitCdg]
 * @returns {Object} {bus, cpu, consoleDev, clk, ssc, nvr, cqbic, ka655, cdg, devices}
 */
function makeRomMachine(romBytes, fOmitCdg = false)
{
    let bus = new BusVAX({busWidth: VAX.PAWIDTH, id: "bus"}, null, null);
    bus.addMemory(0, MEMSIZE, MemoryVAX.TYPE.RAM);
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
    let cqbic = new CQBICVAX(cpu.exc);
    bus.addRegBlock([
        {base: VAX.PHYSMEM.REG_BASE >>> 0, length: 0x14, dev: cqbic},
        {base: (VAX.PHYSMEM.REG_BASE + 0x4000) >>> 0, length: 8, dev: ka655}
    ]);
    let cdg = null;
    if (!fOmitCdg) { cdg = new CDGVAX(ka655); bus.addCdg(cdg); }
    cpu.setBus(bus);
    cpu.exc.setIPRDevice(makeIprDevice(clk, consoleDev));
    cpu.exc.addInterruptSource(IPL_CLK_ABS, INT_V_CLK, SCB.INTTIM);
    cpu.clk = clk;
    cpu.tmr = ssc;
    /* Derived from the constructions above, in this same function, so it can neither list a device
       the machine does not have nor omit one it does (see the file header).  `cdg` is present only
       when it was actually decoded, which is what makes `fOmitCdg` visible to a census. */
    let devices = [consoleDev, clk, ssc, nvr, cqbic, ka655, cdg].filter((d) => d !== null)
        .map((d) => ({name: d.constructor.name, dev: d}));
    return {bus, cpu, consoleDev, clk, ssc, nvr, cqbic, ka655, cdg, devices};
}

export { makeRomMachine, MEMSIZE };
export default makeRomMachine;
