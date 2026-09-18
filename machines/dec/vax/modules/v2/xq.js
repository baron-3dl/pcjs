/**
 * @fileoverview The DELQA (DEQNA/DELQA-normal) Ethernet controller for the KA655 VAX
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT license.
 *
 * Portions adapted from the Open SIMH VAX simulator, Copyright © 1998-2019 Robert M Supnik,
 * used under the MIT license.  Robert M Supnik's name is not used to endorse or promote this work.
 *
 * ============================================================================
 * WHAT THIS IS  (item pcjsvax-1a45, epic pcjsvax-d1bf; port spec docs/design/delqa-port.md)
 * ============================================================================
 * xq.js is a differential port of Open SIMH's PDP11/pdp11_xq.c -- the DEQNA/DELQA-normal register
 * file, the xq_csr_set_clr interrupt state machine, device reset/power-up, and the Qbus wiring by
 * which the DELQA coexists with the RQDX3 on the KA655's one Qbus I/O page.  It is graded
 * bit-for-bit against a real microvax3900 oracle by tests/xqdiff.js -- ported against the C, not
 * against a manual and not as a plausible stub (acceptance bar pcjsvax-e05).
 *
 * SCOPE OF THIS RUNG (1a45).  Registers, the interrupt state machine, reset, and the dual-controller
 * Qbus wiring -- NO packet movement.  The TX descriptor walk (xq_process_xbdl) is pcjsvax-6b0 and
 * the RX/setup path (xq_process_rbdl / xq_process_setup) is pcjsvax-a7f; both are left as CLEAN
 * SEAMS here (dispatchRbdl/dispatchXbdl throw XQUnimplemented by name rather than inventing an
 * answer -- HANDOFF-style, so a differential can never green a transmit that does not happen).  The
 * EthernetLink transport seam (ethlink.js, item 549) and its hub backend (ethlink-hubport.js, item
 * ee9) are already built and proven; this device DRIVES that seam, it does not re-implement it.
 *
 * DEQNA/DELQA-NORMAL ONLY.  DELQA-T ("turbo"/DELQA-PLUS) and the PDP-11 boot ROM are out of scope
 * (port spec §Scope): VMS + DECnet on the VAX never selects turbo, and the boot ROM is PDP-11 binary
 * code.  `type` is XQ_T_DELQA and `mode` is XQ_T_DELQA after reset; a `mode` that ever becomes
 * XQ_T_DELQA_PLUS in this port is a defect.  The turbo-arm handshake at pdp11_xq.c:2513-2519 is
 * deliberately NOT implemented, and a write that would arm it is REPORTED (standing rule 6), never
 * silently accepted -- see wr().
 *
 * WHERE THE CONSTANTS LIVE.  As with rq.js's RQ_BASE/IPL_RQ (rq.js:975-991), the device's Qbus
 * address, interrupt slot and register masks are defined and exported HERE rather than in defines.js:
 * the tests (tests/xqdiff.js) and the machine builders import them from this module, exactly as
 * tests/mscpharness.js imports RQ_BASE from rq.js.  The item's "pick constants in defines.js" is a
 * suggestion predating that precedent; keeping them device-local is what the tree already does.
 *
 * DUAL-CONTROLLER QBUS WIRING (the structural gap this rung closes).  bus.addIoPage() already takes
 * a LIST of windows over the one I/O page, so the RQDX3's and the DELQA's register windows coexist
 * in the one controller with no change to bus.js.  What was single was the CPU's per-instruction
 * event-queue hook `cpu.qbus`: rq.js owns it (its RQ_QUEUE service).  Per port spec §7's
 * recommendation (b) -- explicit, no combined-tick coupling -- a SECOND hook `cpu.qbus2` is added in
 * cpustate.js (ticked, drained on HALT in exc.js, considered by idle.js), and the DELQA is installed
 * on it.  The two controllers' interrupt slots are distinct: RQ at (IPL 0x14, bit 0), XQ at
 * (IPL 0x14, bit 6) -- addInterruptSource() has no collision guard (pcjsvax-811), so the constructor
 * ASSERTS the XQ slot is empty before claiming it and never touches RQ's.
 *
 * NEVER-CRASH-A-PEER.  This card will carry real SCS frames to a real OpenVMS peer.  The frame path
 * (6b0/a7f) forwards bytes verbatim through ethlink.js and never mutates, truncates, reorders,
 * injects, or aliases a buffer; the seams here preserve that invariant by not touching frames at all.
 */

import { VAX } from "./defines.js";
import { VEC_SET } from "./exc.js";
import { LoopbackEthernetLink } from "./ethlink.js";

/* --------------------------------------------------------------------------------------------- *
 * §7 Qbus address, interrupt slot, vector -- VERIFIED against the oracle (docs/design/delqa-port.md)
 * --------------------------------------------------------------------------------------------- */

/** IOLN_XQ = 020 octal = 16 bytes = 8 words (pdp11_xq.h). */
export const IOLN_XQ = 16;

/** The DELQA CSR window, VERIFIED against the live oracle (2026-09-18): `set xq enable; show xq`
    reports `address=20001920-2000192F`, and `show qba iospace` lists XQ at 20001920-2000192F.  So
    XQ_OFFSET = 0x1920 and XQ_BASE = IOPAGE_BASE (0x20000000) + 0x1920.  Derived like rq.js's
    RQ_BASE but pinned to the MEASURED offset; tests/xqdiff.js re-reads `show qba iospace` from the
    oracle on every run and FAILS if the XQ row disagrees, the same discipline RQ_BASE is under. */
export const XQ_OFFSET = 0x1920;
export const XQ_BASE   = (VAX.PHYSMEM.IOPAGE_BASE + XQ_OFFSET) >>> 0;

/** The interrupt request slot.  vaxmod_defs.h: INT_V_XQ = 6, IPL_XQ = 0x14 (the IPL exc.js's
    raiseInterrupt()/clearInterrupt() take, not the IPL_HMIN-relative index).  RQ occupies
    (0x14, 0); XQ occupies (0x14, 6) -- a DISTINCT bit, so the two do not collide today, but the
    constructor still asserts the slot is free (pcjsvax-811 -- addInterruptSource has no guard). */
export const IPL_XQ   = 0x14;
export const INT_V_XQ = 6;

/** The autoconfigured DELQA interrupt vector.  On the VAX the hardware vector is 0120 octal, but the
    DIB's `vec` starts at 0 and is the DRIVER's echo of what it writes into VAR<IV> (xq_wr_var sets
    dib->vec = data & XQ_VEC_IV); the oracle's `show qba iospace` prints a blank XQ vector until the
    driver programs one, exactly as it does for RQ.  Recorded for the interrupt-delivery rungs; 1a45
    moves no packets and delivers no interrupt. */
export const XQ_VEC_FIXED = 0o120;

/* --------------------------------------------------------------------------------------------- *
 * §1 CSR / VAR register bits and access masks (pdp11_xq.h:324-361) -- transcribed verbatim
 * --------------------------------------------------------------------------------------------- */

export const XQ_CSR_RI = 0x8000, XQ_CSR_PE = 0x4000, XQ_CSR_CA = 0x2000, XQ_CSR_OK = 0x1000,
             XQ_CSR_RR = 0x0800, XQ_CSR_SE = 0x0400, XQ_CSR_EL = 0x0200, XQ_CSR_IL = 0x0100,
             XQ_CSR_XI = 0x0080, XQ_CSR_IE = 0x0040, XQ_CSR_RL = 0x0020, XQ_CSR_XL = 0x0010,
             XQ_CSR_BD = 0x0008, XQ_CSR_NI = 0x0004, XQ_CSR_SR = 0x0002, XQ_CSR_RE = 0x0001;
export const XQ_CSR_RO   = 0xF8B4;   /* read-only bits */
export const XQ_CSR_RW   = 0x074B;   /* read/write bits */
export const XQ_CSR_W1   = 0x8080;   /* write-one-to-clear bits (RI, XI) */
export const XQ_CSR_BP   = 0x0208;   /* boot PDP diagnostic ROM (out of scope) */
export const XQ_CSR_XIRI = 0x8080;   /* XI | RI */

export const XQ_VEC_MS = 0x8000, XQ_VEC_OS = 0x4000, XQ_VEC_RS = 0x2000, XQ_VEC_S3 = 0x1000,
             XQ_VEC_S2 = 0x0800, XQ_VEC_S1 = 0x0400, XQ_VEC_ST = 0x1C00, XQ_VEC_IV = 0x03FC,
             XQ_VEC_RR = 0x0002, XQ_VEC_ID = 0x0001;
export const XQ_VEC_RO = 0x5C02;     /* read-only bits */
export const XQ_VEC_RW = 0xA3FD;     /* read/write bits */

/** enum xq_type (pdp11_xq.h:99). */
export const XQ_T_DEQNA = 0, XQ_T_DELQA = 1, XQ_T_DELQA_PLUS = 2;

/** sanity-timer enable flags (pdp11_xq.h:103-104) and delays (pdp11_xq.h:92-94). */
export const XQ_SAN_HW_SW = 2, XQ_SAN_ENABLE = 1;
export const XQ_STARTUP_DELAY = 20;      /* instructions to delay before the receiver starts */
export const XQ_QUE_MAX = 500;           /* read-queue capacity, in packets */

/**
 * Thrown BY NAME rather than answered, exactly as rq.js's RQUnimplemented: the frame-movement paths
 * (TX = pcjsvax-6b0, RX/setup = pcjsvax-a7f) are seams in this rung, and a differential that reached
 * one must fail loudly rather than compare a stubbed transmit against a real one.
 */
export class XQUnimplemented extends Error {}

/* --------------------------------------------------------------------------------------------- *
 * The default MAC -- 08:00:2B:00:00:00, DEC's OUI (xq_reset's xq_setmac ".../24", pdp11_xq.c:2575).
 * The high byte of every MAC-ROM read floats high (0xFF); see rd().
 * --------------------------------------------------------------------------------------------- */
const DEFAULT_MAC = [0x08, 0x00, 0x2B, 0x00, 0x00, 0x00];

/**
 * XQVAX -- one DELQA controller.  A sibling of rq.js's RQVAX and installed the same way: constructed
 * with the CQBIC (for the interrupt seam it owns via cqbic.exc, and -- for 6b0/a7f -- for the DMA
 * map), placed on the I/O page by the machine builder, and hooked onto the CPU's second Qbus event
 * slot (cpu.qbus2).
 */
export default class XQVAX {
    /**
     * @param {Object} cqbic a CQBICVAX WITH a bus -- provides cqbic.exc for the interrupt seam and
     *   (for the later frame rungs) the scatter-gather DMA map.  A CQBIC built without an `exc`
     *   leaves this device recording irq and delivering nothing -- the pre-interrupt behaviour, not
     *   a crash -- mirroring rq.js.
     * @param {Object} [opts] {base, ethlink} -- `base` is the Qbus I/O-page address this instance
     *   answers for (defaults to the autoconfigured XQ_BASE); `ethlink` is an injected EthernetLink
     *   transport (ethlink.js).  With no `ethlink` the device has NO network attached (etherface
     *   null), which is the state the oracle is in under `set xq enable` with no `attach` -- the
     *   deterministic state tests/xqdiff.js grades.
     */
    constructor(cqbic, opts = {})
    {
        this.cqbic = cqbic;
        this.base = ((opts.base === undefined) ? XQ_BASE : opts.base) >>> 0;

        /* `type` is fixed DELQA in this port (never DEQNA-lock, never turbo).  `mode` tracks
           DELQA/DEQNA per VAR<MS> and is asserted DELQA after reset. */
        this.type = XQ_T_DELQA;
        this.mode = XQ_T_DELQA;
        this.lockmode = 0;                                  /* DEQNA-Lock OFF */
        this.sanity = { enabled: 0, quarter_secs: 0, timer: 0 };

        /* The register-file state (pdp11_xq.h:276-304, the fields this rung touches). */
        this.mac = DEFAULT_MAC.slice();                     /* the 6-byte station address */
        this.mac_checksum = [0, 0];                         /* xq_make_checksum output */
        this.csr = 0;
        this.var = 0;
        this.vec = 0;                                       /* dib->vec, the driver's VAR<IV> echo */
        this.irq = 0;

        /* The BDL register latches and their working copies -- present so the 6b0/a7f seams have
           somewhere to land; untouched by this rung except that a write to them is refused (wr()). */
        this.rbdl = [0, 0];
        this.xbdl = [0, 0];

        /* The injected transport.  `etherface` is the C's `xq->var->etherface`: null when nothing is
           attached.  A LoopbackEthernetLink passed in `opts.ethlink` is a concrete backend; it is
           only "attached" once attach() is called, matching the oracle's enable-vs-attach split. */
        this.ethlink = opts.ethlink || null;
        this.etherface = null;                              /* set by attach() */

        /* The receive FIFO the RX path (a7f) drains into guest memory.  Allocated once. */
        this.readQ = [];

        /* The event queue -- one deadline per pseudo-unit, in cpu.nTotalCycles units, or null for
           "not active" (sim_is_active).  UNIT 0 = receive poll (xq_svc), 1 = sanity timer
           (xq_tmrsvc), 2 = receiver startup (xq_startsvc).  1a45 attaches nothing, so none of these
           fire with observable effect; they exist so 6b0/a7f have a real scheduler to hang service
           routines on rather than inventing one. */
        this.units = [
            { due: null, svc: (cpu) => this.svc(cpu) },
            { due: null, svc: (cpu) => this.tmrSvc(cpu) },
            { due: null, svc: (cpu) => this.startSvc(cpu) }
        ];
        this.evSeq = 0;
        this.cpu = null;

        /* THE INTERRUPT SEAM.  Installed ONCE here (not in reset), like rq.js: cqbic.exc owns the
           CPU's VAXExc.  The XQ slot (0x14, 6) MUST be distinct from RQ's (0x14, 0); add
           InterruptSource has no collision guard (pcjsvax-811), so assert it is empty before
           claiming it -- a machine that wired two devices to one slot would otherwise lose one
           silently. */
        this.exc = (cqbic && cqbic.exc) || null;
        if (this.exc) this.exc.addInterruptSource(IPL_XQ, INT_V_XQ, () => this.inta(), VEC_SET);

        this.powerUp();
    }

    /* --------------------------------------------------------------------------------------- *
     * Attachment (the transport, not an image)                                                 *
     * --------------------------------------------------------------------------------------- */

    /**
     * attach() -- bind the injected EthernetLink and mark the etherface present, then re-reset so
     * the OK (transceiver power) bit and the reset side effects that depend on `etherface` take hold,
     * exactly as SIMH's xq_attach re-runs the etherface-dependent arm of xq_reset.  1a45's
     * differential does NOT call this (it grades the enabled-but-unattached state); it is here so the
     * later rungs and the browser machine can bring the card online.
     *
     * @param {Object} [ethlink] optional EthernetLink to bind now (else opts.ethlink from construct)
     */
    attach(ethlink)
    {
        if (ethlink) this.ethlink = ethlink;
        if (!this.ethlink) this.ethlink = new LoopbackEthernetLink();
        this.etherface = this.ethlink;
        this.etherface.setReceiveHandler((frameU8) => this.deliverReceive(frameU8));
        this.etherface.attach();
        /* program the receive filter to the ROM MAC and light OK (xq_reset's etherface arm) */
        this.etherface.setFilter([this.mac], false, false);
        this.csrSetClr(XQ_CSR_OK, 0);
        return this;
    }

    detach()
    {
        if (this.etherface) this.etherface.detach();
        this.etherface = null;
        this.csrSetClr(0, XQ_CSR_OK);
        return this;
    }

    /* --------------------------------------------------------------------------------------- *
     * §6.1 Reset and power-up -- TWO separate methods (like rq.js)                              *
     * --------------------------------------------------------------------------------------- */

    /**
     * powerUp() -- the `sim_switches & SWMASK('P')` arm of xq_reset (pdp11_xq.c:2629-2635): clear the
     * setup struct and turn on all three LEDs, then perform the ordinary reset.  Modelled as PCjs's
     * powerUp() distinct from reset(), the way rq.js splits them.
     */
    powerUp()
    {
        this.setup = { l1: 1, l2: 1, l3: 1, promiscuous: 0, multicast: 0, sanity_timer: 0 };
        this.reset();
    }

    /**
     * reset() -- xq_reset (pdp11_xq.c:2551-2637), the run-time (non-one-time) arm.  The one-time unit
     * naming and the initial xq_setmac are done in the constructor here.  Order, term for term:
     * xq_make_checksum; init VAR (DELQA: MS, plus OS if HW sanity) and mode; dib.vec = 0; CSR =
     * RL|XL via csr_set_clr(set, ~set); clrint; init+clear ReadQ; if etherface: filter+OK+sanity
     * timer+async off; cancel the receiver units; set HW sanity count.  (auto_config() is a
     * no-op here -- the base is fixed by the machine builder and checked against the oracle's own
     * autoconfiguration by xqdiff.js, mirroring rq.js.)
     */
    reset()
    {
        this.makeChecksum();

        /* init vector address register + mode.  type is DELQA, so the DELQA arm:
           var = (lockmode ? 0 : MS) | (HW-sanity ? OS : 0); mode = lockmode ? DEQNA : DELQA. */
        this.var = (this.lockmode ? 0 : XQ_VEC_MS) |
                   ((this.sanity.enabled & XQ_SAN_HW_SW) ? XQ_VEC_OS : 0);
        this.mode = this.lockmode ? XQ_T_DEQNA : XQ_T_DELQA;
        this.vec = 0;                                       /* dib->vec = 0 */

        /* init CSR: set RL|XL, clear everything else -- csr_set_clr(RL|XL, ~(RL|XL)) => CSR = RL|XL */
        const setBits = XQ_CSR_RL | XQ_CSR_XL;
        this.csrSetClr(setBits, (~setBits) & 0xFFFF);

        this.clrInt();                                      /* clear interrupts unconditionally */

        this.readQ.length = 0;                              /* ethq_init + ethq_clear */

        if (this.etherface) {
            this.etherface.setFilter([this.mac], false, false);   /* restore ROM-mac filter */
            this.csrSetClr(XQ_CSR_OK, 0);                         /* transceiver power on */
            this.activate(1, 250);                                /* 250ms sanity service (arbitrary units) */
        }

        /* sim_cancel(unit[0]) and sim_cancel(unit[2]) -- stop the receiver poll and startup. */
        this.units[0].due = null;
        this.units[2].due = null;

        if (this.sanity.enabled & XQ_SAN_HW_SW) {
            this.sanity.quarter_secs = 240 * 4;             /* XQ_HW_SANITY_SECS * 4 */
        }
    }

    /**
     * swReset() -- xq_sw_reset (pdp11_xq.c:2129-2156): the DRIVER-issued soft reset, taken when CSR
     * SR transitions from set to clear.  Re-asserts RL|XL (clearing the rest), lights OK if attached,
     * clears the interrupt unconditionally, and flushes the read queue.  DELQA-T-return and the
     * DEQNA/ULTRIX interrupt-enable quirks are out of scope for a fixed-DELQA controller.
     */
    swReset()
    {
        const setBits = XQ_CSR_XL | XQ_CSR_RL;
        this.csrSetClr(setBits, (~setBits) & 0xFFFF);
        if (this.etherface) this.csrSetClr(XQ_CSR_OK, 0);
        this.clrInt();
        this.readQ.length = 0;
    }

    /**
     * makeChecksum() -- xq_make_checksum (pdp11_xq.c:660-681).  A bespoke 16-bit end-around-carry sum
     * over the six MAC bytes (NOT eth_crc32 -- keep them separate, port spec §5.2), stored little
     * end first into mac_checksum[0..1].  Returned in external-loopback mode by rd() index 0/1.
     */
    makeChecksum()
    {
        let checksum = 0;
        const wmask = 0xFFFF;
        for (let i = 0; i < 6; i += 2) {
            checksum = (checksum << 1) >>> 0;
            if (checksum > wmask) checksum -= wmask;
            checksum += (this.mac[i] << 8) | this.mac[i + 1];
            if (checksum > wmask) checksum -= wmask;
        }
        if (checksum === wmask) checksum = 0;
        this.mac_checksum[0] = checksum & 0xFF;
        this.mac_checksum[1] = (checksum >>> 8) & 0xFF;
    }

    /**
     * setMac(mac) -- xq_setmac (pdp11_xq.c:683-696): set the station address and recompute the
     * checksum.  Accepts a 6-element byte array or an "aa:bb:cc:dd:ee:ff" string.
     */
    setMac(mac)
    {
        let bytes;
        if (typeof mac === "string") {
            bytes = mac.split(/[:\-.]/).map((h) => parseInt(h, 16) & 0xFF);
        } else {
            bytes = Array.from(mac).map((b) => b & 0xFF);
        }
        if (bytes.length !== 6 || bytes.some((b) => Number.isNaN(b))) {
            throw new Error("xq.js: setMac expects 6 bytes, got " + JSON.stringify(mac));
        }
        this.mac = bytes;
        this.makeChecksum();
        return this;
    }

    /* --------------------------------------------------------------------------------------- *
     * §3 The interrupt state machine -- PORTED VERBATIM (pdp11_xq.c:2976-3065)                  *
     * --------------------------------------------------------------------------------------- */

    /**
     * setInt() -- xq_setint (pdp11_xq.c:2976), DEQNA/DELQA-normal path only (the DELQA-PLUS icr guard
     * is out of scope).  Raise the controller's master interrupt.
     */
    setInt()
    {
        this.irq = 1;
        if (this.exc) this.exc.raiseInterrupt(IPL_XQ, INT_V_XQ);
    }

    /**
     * clrInt() -- xq_clrint (pdp11_xq.c:2993).  This port has exactly ONE XQ controller (each browser
     * VAX is its own emulator instance), so the C's walk over XQ_MAX_CONTROLLERS to keep the shared
     * master set for a sibling reduces to clearing the one request bit.
     */
    clrInt()
    {
        this.irq = 0;
        if (this.exc) this.exc.clearInterrupt(IPL_XQ, INT_V_XQ);
    }

    /**
     * inta() -- xq_int (pdp11_xq.c:3007): the acknowledge/vector-fetch handed to addInterruptSource.
     * Returns the vector and clears the interrupt if one is pending.
     */
    inta()
    {
        if (this.irq) {
            this.clrInt();
            return this.vec | 0;
        }
        return 0;
    }

    /**
     * csrSetClr(setBits, clearBits) -- xq_csr_set_clr (pdp11_xq.c:3020-3065).  PORTED EXACTLY.  Every
     * interrupt-relevant CSR mutation flows through this ONE function; nothing pokes an RI/XI/IE bit
     * around it.
     */
    csrSetClr(setBits, clearBits)
    {
        const saved = this.csr;
        this.csr = ((this.csr | setBits) & ~clearBits) & 0xFFFF;

        if ((saved ^ this.csr) & XQ_CSR_IE) {               /* IE is transitioning */
            if ((clearBits & XQ_CSR_IE) && this.irq) this.clrInt();
            if ((setBits & XQ_CSR_IE) && (this.csr & XQ_CSR_XIRI) && !this.irq) this.setInt();
        } else {                                            /* IE not transitioning */
            if (this.csr & XQ_CSR_IE) {                     /* interrupts enabled */
                if (((saved ^ this.csr) & (setBits & XQ_CSR_XIRI)) && !this.irq) {
                    this.setInt();
                } else if (((saved ^ this.csr) & (clearBits & XQ_CSR_XIRI)) &&
                           !(this.csr & XQ_CSR_XIRI) && this.irq) {
                    this.clrInt();
                }
            }
        }
    }

    /* --------------------------------------------------------------------------------------- *
     * §1 The register file -- xq_rd / xq_wr (DEQNA/DELQA-normal arm only)                        *
     * --------------------------------------------------------------------------------------- */

    /**
     * rd(pa) -- xq_rd (pdp11_xq.c:1056-1095), DEQNA/DELQA-normal.  Word index (pa>>1)&07:
     *   0,1 -> 0xFF00 | (csr&EL ? mac_checksum[i] : mac[i])   (checksum only in external loopback)
     *   2..5 -> 0xFF00 | mac[i]
     *   6 -> var        7 -> csr
     * The high byte of every MAC-ROM read is 0xFF (upper 8 bits float high on real hardware).
     *
     * @param {number} pa absolute physical address
     * @returns {number} the 16-bit word
     */
    rd(pa)
    {
        const index = (pa >>> 1) & 0o7;
        switch (index) {
        case 0:
        case 1:
            return (this.csr & XQ_CSR_EL) ? (0xFF00 | this.mac_checksum[index])
                                          : (0xFF00 | this.mac[index]);
        case 2:
        case 3:
        case 4:
        case 5:
            return 0xFF00 | this.mac[index];
        case 6:
            return this.var & 0xFFFF;                       /* DELQA-PLUS srr branch is out of scope */
        case 7:
        default:
            return this.csr & 0xFFFF;
        }
    }

    /**
     * wr(pa, data) -- xq_wr (pdp11_xq.c:2471-2547), the `default` (DEQNA/DELQA-normal) arm only.
     * Word index (pa>>1)&07:
     *   0,1 -> IBAL/IBAH: no-op in normal mode (only meaningful on DELQA-T -- NOT entered here).  A
     *          write that looks like the turbo-arm handshake (0x0BAF/0xFF00) is REPORTED, not honored.
     *   2   -> RBDL lo          3 -> RBDL hi (+ RX dispatch -- SEAM: pcjsvax-a7f)
     *   4   -> XBDL lo          5 -> XBDL hi (clears XL, + TX dispatch -- SEAM: pcjsvax-6b0)
     *   6   -> VAR (xq_wr_var)  7 -> CSR (xq_wr_csr)
     */
    wr(pa, data)
    {
        const index = (pa >>> 1) & 0o7;
        data &= 0xFFFF;
        switch (index) {
        case 0:
            /* IBAL: a no-op on a DELQA-normal card. */
            break;
        case 1:
            /* IBAH: the DELQA-T turbo-arm handshake lives here (iba==0x0BAF && data==0xFF00).  This
               port is fixed DELQA-normal and never enters turbo; surface any attempt rather than
               silently accepting it (standing rule 6). */
            if (((this.xbdl[0] & 0xFFFF) === 0x0BAF) && (data === 0xFF00)) {
                this.reportTurboArm(pa, data);
            }
            break;
        case 2:
            this.rbdl[0] = data;
            break;
        case 3:
            this.rbdl[1] = data;
            this.dispatchRbdl();                            /* SEAM: RX -- pcjsvax-a7f */
            break;
        case 4:
            this.xbdl[0] = data;
            break;
        case 5:
            this.xbdl[1] = data;
            this.csrSetClr(0, XQ_CSR_XL);                   /* clear XL, then transmit */
            this.dispatchXbdl();                            /* SEAM: TX -- pcjsvax-6b0 */
            break;
        case 6:
            this.wrVar(data);
            break;
        case 7:
        default:
            this.wrCsr(data);
            break;
        }
    }

    /**
     * reportTurboArm(pa, data) -- surface an out-of-scope DELQA-T turbo-arm write.  Not a crash: the
     * write is ignored (this card stays DELQA-normal), but it is reported so a guest or a differential
     * that unexpectedly drove turbo is not met with a silent accept.
     */
    reportTurboArm(pa, data)
    {
        this.turboArmSeen = (this.turboArmSeen || 0) + 1;
        if (typeof console !== "undefined" && console.warn) {
            console.warn("xq.js: DELQA-T turbo-arm write (pa=0x" + (pa >>> 0).toString(16) +
                " data=0x" + (data & 0xFFFF).toString(16) + ") ignored -- this port is DELQA-normal " +
                "only (port spec §Scope). NOT entering turbo mode.");
        }
    }

    /**
     * wrVar(data) -- xq_wr_var (pdp11_xq.c:2183-2229), DELQA arm.  var = data & XQ_VEC_RW (unless
     * DEQNA-lock, out of scope for a fixed-DELQA card).  If MS toggles, switch mode DELQA<->DEQNA and
     * mask VAR accordingly.  If Request-Self-test set, clear it and report: no etherface -> set S1
     * ("No Network Connection"); attached -> clear ST (success).  Mirror the vector: dib.vec =
     * data & XQ_VEC_IV.
     */
    wrVar(data)
    {
        const saveVar = this.var;
        if (this.lockmode) {
            this.var = data & (XQ_VEC_IV | XQ_VEC_ID);
        } else {
            this.var = data & XQ_VEC_RW;
        }

        if ((saveVar ^ this.var) & XQ_VEC_MS) {             /* DEQNA-Lock mode changing? */
            if (~this.var & XQ_VEC_MS) {
                this.mode = XQ_T_DEQNA;
                this.var &= ~(XQ_VEC_OS | XQ_VEC_RS | XQ_VEC_ST);
            } else {
                this.mode = XQ_T_DELQA;
            }
        }

        if (this.var & XQ_VEC_RS) {                         /* Request Self-Test */
            this.var &= ~XQ_VEC_RS;
            if (!this.etherface) this.var |= XQ_VEC_S1;     /* No Network Connection */
            else this.var &= ~XQ_VEC_ST;                    /* success */
        }

        this.vec = data & XQ_VEC_IV;                        /* dib->vec */
        this.var &= 0xFFFF;
    }

    /**
     * wrCsr(data) -- xq_wr_csr (pdp11_xq.c:2290-2334).  set_bits = data & RW; clr_bits = (RW bits the
     * write clears) | (W1-to-clear RI/XI) | (clearing XI also clears NI).  Then, in order: SR
     * transitioning-cleared -> swReset and RETURN; RE 0->1 -> schedule the receiver start; RE 1->0 ->
     * stop the receiver; apply via csr_set_clr.  The boot-ROM BP check is out of scope.
     */
    wrCsr(data)
    {
        data &= 0xFFFF;
        const setBits = data & XQ_CSR_RW;
        const clrBits = (((data ^ XQ_CSR_RW) & XQ_CSR_RW) |
                         (data & XQ_CSR_W1) |
                         ((data & XQ_CSR_XI) ? XQ_CSR_NI : 0)) & 0xFFFF;

        /* reset controller when SR transitions to cleared */
        if (this.csr & XQ_CSR_SR & ~data) {
            this.swReset();
            return;
        }

        /* start receiver when RE transitions to set */
        if ((~this.csr) & XQ_CSR_RE & data) {
            this.activate(2, this.startupDelay());          /* xq_startsvc after startup_delay */
        }

        /* stop receiver when RE transitions to clear */
        if (this.csr & XQ_CSR_RE & ~data) {
            this.stopReceiver();
        }

        this.csrSetClr(setBits, clrBits);

        /* boot/diagnostic ROM (XQ_CSR_BP) is a PDP-11-ism and out of scope -- deliberately NOT done. */
    }

    startupDelay() { return XQ_STARTUP_DELAY; }

    /* --------------------------------------------------------------------------------------- *
     * PCjs register contract (readByte/readWord/readLong/writeByte/writeWord/writeLong)         *
     * mirrors rq.js:2048-2110 -- addr is ABSOLUTE; return value/null on read, true/false on write *
     * --------------------------------------------------------------------------------------- */

    /**
     * readLong(addr) -- ReadIO(pa, L_LONG) is two Qbus cycles, and both land on this device because
     * the window is 16 bytes.  Mirrors rq.js.readLong.
     */
    readLong(addr)
    {
        addr = addr >>> 0;
        if (addr + 4 > this.base + IOLN_XQ) return null;
        const lo = this.rd(addr) & 0xFFFF;
        const hi = this.rd((addr + 2) >>> 0) & 0xFFFF;
        return ((hi << 16) | lo) | 0;
    }

    readWord(addr) { return this.rd(addr >>> 0) & 0xFFFF; }

    readByte(addr) { return (this.rd(addr >>> 0) >>> ((addr & 1) << 3)) & 0xFF; }

    /** writeLong -- mmu.js splits an aligned Qbus longword write into two setWord cycles, so this is
        not on any CPU path; return false (not decoded) rather than silently accept, like rq.js. */
    writeLong(addr, val) { return false; }

    writeWord(addr, val) { this.wr(addr >>> 0, val & 0xFFFF); return true; }

    /** writeByte -- WriteQb reaches xq_wr with the byte right-justified and unshifted; xq_wr uses it
        unmerged, exactly as rq.js documents for its own device.  Graded, not tidied. */
    writeByte(addr, val) { this.wr(addr >>> 0, val & 0xFF); return true; }

    /* --------------------------------------------------------------------------------------- *
     * §6.2 The event/service model on the PCjs per-instruction tick (NO wall-clock timer)       *
     * The frame-service bodies are SEAMS for 6b0/a7f; with nothing attached they are no-ops.    *
     * --------------------------------------------------------------------------------------- */

    /**
     * activate(unit, delayInstr) -- sim_activate(unit, delay): arm the unit's deadline `delayInstr`
     * instructions from now.  Uses the cpu captured by the last tick()/wr() path; if no cpu is known
     * yet (device driven before any instruction), the deadline is relative to 0 and fires on the
     * first tick.
     */
    activate(unit, delayInstr)
    {
        const now = this.cpu ? this.cpu.nTotalCycles : 0;
        this.units[unit].due = (now + (delayInstr | 0));
        this.units[unit].seq = this.evSeq++;
    }

    /** nextEvent() -- the earliest armed unit, or null. */
    nextEvent()
    {
        let best = null;
        for (const u of this.units) {
            if (u.due === null) continue;
            if (best === null || u.due < best.due || (u.due === best.due && u.seq < best.seq)) best = u;
        }
        return best;
    }

    /**
     * tick(cpu) -- the per-instruction hook cpu.qbus2 calls (cpustate.js).  Service every event whose
     * time has come, exactly as rq.js's tick services its queue.  1a45 arms nothing observable, so
     * this is a no-op in the differential; it is wired now so 6b0/a7f need not touch cpustate.js.
     */
    tick(cpu)
    {
        this.cpu = cpu;
        for (let n = 0; ; n++) {
            const e = this.nextEvent();
            if (e === null || cpu.nTotalCycles < e.due) break;
            if (n > XQ_QUE_MAX + 8) {
                throw new Error("xq.js: tick() serviced too many events without the queue going " +
                    "quiet -- a service is re-arming itself without progress.");
            }
            e.due = null;
            e.svc(cpu);
        }
    }

    /** idleAble -- SIMH's UNIT_IDLE; the DELQA's service units do not carry it, so idle.js will not
        sleep while one is outstanding (rq.js documents the same for its queue). */
    get idleAble() { return false; }

    instrsToEvent(cpu)
    {
        const e = this.nextEvent();
        if (e === null) return Infinity;
        const j = (e.due - cpu.nTotalCycles) + 1;
        return j > 0 ? j : 0;
    }

    /** drainOnHalt(cpu) -- the HALT instruction's event drain (exc.js), mirroring rq.js: run every
        remaining service to completion.  Bounded so a self-re-arming service is a named error, not
        a hang. */
    drainOnHalt(cpu)
    {
        this.cpu = cpu;
        for (let n = 0; ; n++) {
            const e = this.nextEvent();
            if (e === null) break;
            if (n > XQ_QUE_MAX + 8) {
                throw new Error("xq.js: drainOnHalt() ran too many services without the queue going " +
                    "idle -- a service is re-arming itself without progress.");
            }
            e.due = null;
            e.svc(cpu);
        }
    }

    /* ---- the service bodies (SEAMS) ---- */

    /** startSvc -- xq_startsvc/xq_start_receiver (pdp11_xq.c): with no etherface it returns
        immediately (the C's first line).  The attached RX path is pcjsvax-a7f. */
    startSvc(cpu)
    {
        if (!this.etherface) return;
        throw new XQUnimplemented("xq.js: attached receiver start is pcjsvax-a7f (RX path)");
    }

    /** svc -- xq_svc, the receive poll.  Frame movement is pcjsvax-a7f; unattached it is a no-op. */
    svc(cpu)
    {
        if (!this.etherface || this.readQ.length === 0) return;
        throw new XQUnimplemented("xq.js: receive-poll frame movement is pcjsvax-a7f (RX path)");
    }

    /** tmrSvc -- xq_tmrsvc, the 250ms sanity/system-id timer.  Off by default; a real sanity timeout
        (xq_boot_host) is a device reset and MUST be modelled once sanity is enabled (a7f/later).  For
        1a45 the sanity timer is off, so re-arm without effect. */
    tmrSvc(cpu)
    {
        /* re-arm to keep the C's cadence; no countdown effect while sanity is disabled */
        if (this.etherface) this.activate(1, 250);
    }

    stopReceiver()
    {
        this.units[0].due = null;
        this.units[2].due = null;
    }

    /* ---- the frame dispatch SEAMS (6b0/a7f) -- throw by name, never silently no-op ---- */

    /** dispatchRbdl -- xq_dispatch_rbdl: begin an RX descriptor walk.  pcjsvax-a7f. */
    dispatchRbdl()
    {
        throw new XQUnimplemented("xq.js: RX descriptor processing (xq_process_rbdl) is pcjsvax-a7f");
    }

    /** dispatchXbdl -- xq_dispatch_xbdl: begin a TX descriptor walk.  pcjsvax-6b0. */
    dispatchXbdl()
    {
        throw new XQUnimplemented("xq.js: TX descriptor processing (xq_process_xbdl) is pcjsvax-6b0");
    }

    /** deliverReceive -- the etherface's inbound callback; drains into readQ for xq_svc.  a7f. */
    deliverReceive(frameU8)
    {
        throw new XQUnimplemented("xq.js: inbound frame delivery is pcjsvax-a7f (RX path)");
    }
}
