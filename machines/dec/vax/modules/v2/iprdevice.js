/**
 * @fileoverview Aggregates multiple off-chip IPR devices behind exc.js's single setIPRDevice() slot
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 * ============================================================================
 * WHAT THIS IS, AND WHY IT EXISTS
 * ============================================================================
 * pcjsvax-bfb, post-merge reconciliation with pcjsvax-954.  `exc.js`'s ReadIPR/WriteIPR dispatch
 * (readIPR()/writeIPR()) installs exactly ONE off-chip device via `setIPRDevice(dev)` -- there is
 * only one slot.  Two items each built a real device against that same slot: pcjsvax-954's
 * `ClkVAX` (clk.js -- MT.ICCS=24, MT.TODR=27) and this item's `ConsoleVAX` (console.js -- MT.RXCS=32,
 * MT.RXDB=33, MT.TXCS=34, MT.TXDB=35).  Installing either one alone would silently shadow the
 * other's registers back to exc.js's pre-device default (0 / dropped write) -- console.js's own
 * TODR sentinel did exactly that until this reconciliation, and it is now DELETED (see console.js's
 * history) in favor of ClkVAX's real, live-graded implementation (tests/timerdiff.js).
 *
 * This module is the resolution: a THIRD, trivial object that owns neither register set itself, and
 * routes `read(prn)`/`write(prn,val)` to whichever real device's registers the number belongs to --
 * console.js's FOUR are named explicitly (the ONLY prn set that must NOT fall through to clk, since
 * clk.js's own read()/write() already correctly default to 0/no-op for every prn it does not own,
 * including MT.NICR/MT.ICR, which clk.js's file header measures as genuinely inert on this SIMH
 * model -- so "not one of console's four" -> clk is the SAME behavior as a real, explicit
 * NICR/ICR/CSRS/CSRD/CSTS/CSTD/CADR/MSER/IORESET dispatch table would produce, without needing to
 * duplicate that list here).
 *
 * ============================================================================
 * tick(cpu) -- DELIBERATELY DOES NOT FORWARD TO clk
 * ============================================================================
 * exc.js's setIRQL() (this item's own per-instruction hook, added for console.js's TXCS-completion
 * timing -- see that file's doc comment) calls `this.iprDevice.tick(cpu)` once per instruction.
 * ClkVAX ALSO has a `tick(cpu)` method -- but it is ALREADY wired to fire once per instruction
 * through a COMPLETELY SEPARATE mechanism, cpustate.js's own `cpu.clk` slot (pcjsvax-954's hook,
 * stepCPU()'s loop, unrelated to exc.js).  Forwarding THIS aggregate's tick() to clk as well would
 * tick it TWICE per instruction retired -- doubling clk_svc's effective rate (todr_reg incrementing
 * twice as fast, CLK interrupts requested twice as often) with no corresponding change on the SIMH
 * side.  This class's own tick() therefore calls ONLY the device that has no dedicated cpustate.js
 * slot of its own (console) -- verified by cpudiff.js's full EHKAA run staying an exact
 * MATCH-over-335444-records after this reconciliation (a rate change of this kind would show up
 * immediately as a TODR-driven CC/timing divergence, exactly the class of bug this project's own
 * cpudiff.js phase exists to catch).
 *
 * @param {Object} clk a ClkVAX instance (clk.js) -- owns MT.ICCS/MT.TODR, defaults everything else
 * @param {Object} console_ a ConsoleVAX instance (console.js) -- owns MT.RXCS/RXDB/TXCS/TXDB
 * @returns {Object} {read(prn), write(prn,val), tick(cpu)} -- the setIPRDevice(dev) contract, plus
 *   the tick() extension exc.js's setIRQL() consults if present
 */
function makeIprDevice(clk, console_)
{
    const CONSOLE_PRNS = new Set([32, 33, 34, 35]);   // MT.RXCS, MT.RXDB, MT.TXCS, MT.TXDB

    return {
        read(prn) {
            return CONSOLE_PRNS.has(prn) ? console_.read(prn) : clk.read(prn);
        },
        write(prn, val) {
            if (CONSOLE_PRNS.has(prn)) console_.write(prn, val);
            else clk.write(prn, val);
        },
        tick(cpu) {
            /* clk is deliberately NOT ticked here -- see the file header. */
            if (console_.tick) console_.tick(cpu);
        }
    };
}

export { makeIprDevice };
