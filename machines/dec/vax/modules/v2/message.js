/**
 * @fileoverview Defines VAX message categories
 * @author Jeff Parsons <Jeff@pcjs.org>
 * @copyright © 2012-2026 Jeff Parsons
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 */

import MESSAGE from "../../../../modules/v2/message.js";

MESSAGE.CPU         = 0x00000002;
MESSAGE.TRAP        = 0x00000004;
MESSAGE.FAULT       = 0x00000008;
MESSAGE.INT         = 0x00000010;
MESSAGE.BUS         = 0x00000020;
MESSAGE.MEMORY      = 0x00000040;
MESSAGE.MMU         = 0x00000080;
MESSAGE.ROM         = 0x00000100;
MESSAGE.DEVICE      = 0x00000200;
MESSAGE.COMPUTER    = 0x04000000;

MESSAGE.NAMES["cpu"]        = MESSAGE.CPU;
MESSAGE.NAMES["trap"]       = MESSAGE.TRAP;
MESSAGE.NAMES["fault"]      = MESSAGE.FAULT;
MESSAGE.NAMES["int"]        = MESSAGE.INT;
MESSAGE.NAMES["bus"]        = MESSAGE.BUS;
MESSAGE.NAMES["memory"]     = MESSAGE.MEMORY;
MESSAGE.NAMES["mmu"]        = MESSAGE.MMU;
MESSAGE.NAMES["rom"]        = MESSAGE.ROM;
MESSAGE.NAMES["device"]     = MESSAGE.DEVICE;
MESSAGE.NAMES["computer"]   = MESSAGE.COMPUTER;

export default MESSAGE;
