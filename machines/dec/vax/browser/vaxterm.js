/**
 * @fileoverview pcjsvax-de8 -- the terminal widget bound to console.js's txdbWr()/injectChar() seam
 * @author Chris Baron <baron@3dl.dev>
 * @copyright © 2026 Chris Baron
 * @license MIT <https://www.pcjs.org/LICENSE.txt>
 *
 * This file is part of PCjs, a computer emulation software project at <https://www.pcjs.org>.
 * PCjs is Copyright © 2012-2026 Jeff Parsons, and this file is distributed under its MIT
 * license.
 *
 * WHAT IT MODELS, AND WHAT IT DOES NOT
 * ------------------------------------
 * A GLASS TTY with scrollback, not a VT100.  The KA655 console and OpenVMS's `_OPA0:` both drive a
 * hardcopy-shaped device: the ROM's self-test countdown, SYSBOOT, the startup log and LOGINOUT's
 * prompt are all plain lines with CR/LF, backspace and the occasional tab.  So the model here is a
 * line array with a cursor column -- CR returns to column 0, LF opens the next line at the same
 * column, BS steps back, and a printable byte OVERWRITES at the column rather than appending, which
 * is what makes VMS's `\\r`-redraw of a partially typed line come out right.
 *
 * CSI sequences are PARSED AND MOSTLY DISCARDED, deliberately.  Two are honoured because the guest
 * really does use them and the screen is visibly wrong without them (`ESC[K` erase-to-end-of-line
 * and `ESC[2J` erase-screen); the rest are consumed so their parameter bytes never appear as text.
 * A full terminal emulator is a separate, much larger thing and is filed as pcjsvax-582 -- a DCL
 * session running EDT or SET TERM/INQUIRE will not paint correctly until it exists, and that is
 * stated rather than discovered.
 *
 * INPUT is the other half of console.js's seam.  Keystrokes become bytes and are posted to the
 * Worker, which hands them to VaxMachine.type() -> injectChar().  DELETE (0x7F), not BACKSPACE
 * (0x08), is what a VMS terminal driver treats as rubout, so that is what the Backspace key sends.
 */

const ESC = 0x1B;

/** Scrollback cap.  A full VMS boot is ~600 lines; 5,000 keeps a long session cheap to re-render. */
const MAX_LINES = 5000;

export class VaxTerminal
{
    /**
     * @param {Element} el a <pre> (or anything whose textContent can be replaced)
     * @param {function(string)} onInput called with the bytes-as-a-string the user typed
     */
    constructor(el, onInput)
    {
        this.el = el;
        this.onInput = onInput;
        this.lines = [""];
        this.row = 0;
        this.col = 0;
        this.esc = null;                                    /* the CSI sequence being collected */
        this.dirty = false;
        this.bytesIn = 0;
        this.hideRange = null;                              /* opt-in {start,end}: lines to NOT paint (see render) */

        el.tabIndex = 0;
        el.addEventListener("keydown", (e) => this.onKeyDown(e));
        el.addEventListener("paste", (e) => {
            e.preventDefault();
            let t = (e.clipboardData || window.clipboardData).getData("text");
            if (t) this.onInput(t.replace(/\n/g, "\r"));
        });
        /* Clicking the screen focuses it, so a user who clicks where they are looking can type. */
        el.addEventListener("mousedown", () => el.focus());
    }

    /** @param {Uint8Array} bytes console output, exactly as console.js's txdbWr() pushed it */
    write(bytes)
    {
        this.bytesIn += bytes.length;
        for (let b of bytes) this.putByte(b);
        this.dirty = true;
    }

    putByte(b)
    {
        if (this.esc !== null) { this.escByte(b); return; }
        switch (b) {
        case ESC:  this.esc = ""; return;
        case 0x0D: this.col = 0; return;                    /* CR */
        case 0x0A: this.newline(); return;                  /* LF */
        case 0x08: if (this.col > 0) this.col--; return;    /* BS */
        case 0x07: return;                                  /* BEL */
        case 0x00: return;
        case 0x09: {                                        /* HT */
            let n = 8 - (this.col % 8);
            for (let i = 0; i < n; i++) this.putPrintable(0x20);
            return;
        }
        default:
            if (b >= 0x20 && b < 0x7F) this.putPrintable(b);
            /* Everything else -- 8-bit and the remaining C0 controls -- is dropped rather than
               drawn, because a glass TTY that paints its own control codes is unreadable. */
            return;
        }
    }

    putPrintable(b)
    {
        let line = this.lines[this.row];
        if (line.length < this.col) line = line + " ".repeat(this.col - line.length);
        this.lines[this.row] = line.slice(0, this.col) + String.fromCharCode(b) + line.slice(this.col + 1);
        this.col++;
    }

    newline()
    {
        this.row++;
        if (this.row >= this.lines.length) this.lines.push("");
        if (this.lines.length > MAX_LINES) {
            let drop = this.lines.length - MAX_LINES;
            this.lines.splice(0, drop);
            this.row -= drop;
            if (this.hideRange) {                       /* keep the hide block anchored across the trim */
                this.hideRange.start = Math.max(0, this.hideRange.start - drop);
                if (this.hideRange.end !== Infinity) this.hideRange.end = Math.max(0, this.hideRange.end - drop);
            }
        }
    }

    /** Collect a CSI sequence.  Parameter and intermediate bytes are 0x20-0x3F; the FINAL byte is
        0x40-0x7E and ends it.  A non-CSI escape (`ESC(B`, `ESC=`) is ended by its own next byte. */
    escByte(b)
    {
        if (this.esc === "" && b !== 0x5B) { this.esc = null; return; }  /* not CSI: consume one */
        this.esc += String.fromCharCode(b);
        if (b >= 0x40 && b <= 0x7E && this.esc.length > 1) {
            let seq = this.esc;
            this.esc = null;
            let fin = seq[seq.length - 1], params = seq.slice(1, -1);
            if (fin === "K") {                              /* erase in line */
                let p = params === "" ? "0" : params;
                if (p === "0") this.lines[this.row] = this.lines[this.row].slice(0, this.col);
                else if (p === "1") this.lines[this.row] = " ".repeat(this.col) + this.lines[this.row].slice(this.col);
                else if (p === "2") this.lines[this.row] = "";
            } else if (fin === "J" && (params === "2" || params === "3")) {
                this.lines = [""]; this.row = 0; this.col = 0;
            }
            /* Everything else is consumed and ignored -- see the file header, and pcjsvax-582. */
        }
    }

    render()
    {
        if (!this.dirty) return;
        this.dirty = false;
        /* A block on the cursor's line, so the user can see the guest is waiting for them. */
        let out, curRow;
        if (this.hideRange) {
            /* pcjsvax demo (ovmx.html only): the NetBSD substrate boot -- kernel banner + device
               probe -- is the "firmware POST" layer a faithful VMS console never shows.  hideRange
               {start,end} drops that ONE contiguous block from what is PAINTED; this.lines stays
               whole, so the self-drive controller (which reads term.lines) still sees the raw
               stream and copy-from-screen never yields the hidden lines.  end===Infinity while the
               reveal marker has not appeared yet -- show only the faithful KA655/VMB head. */
            let s = this.hideRange.start, e = this.hideRange.end;
            if (e === Infinity || e > this.lines.length) {
                out = this.lines.slice(0, s);
                curRow = -1;                                /* cursor is inside the hidden block */
            } else {
                out = this.lines.slice(0, s).concat(this.lines.slice(e));
                curRow = this.row >= e ? s + (this.row - e) : (this.row < s ? this.row : -1);
            }
        } else {
            out = this.lines.slice();
            curRow = this.row;
        }
        if (curRow >= 0) {
            let cur = out[curRow] || "";
            out[curRow] = (cur.length < this.col ? cur + " ".repeat(this.col - cur.length) : cur.slice(0, this.col)) +
                "█" + cur.slice(this.col + 1);
        }
        /* pcjsvax-f23: browser/vax.html renders into a <pre>, but the PCjs machine XML's
           `<control type="textarea" binding="print"/>` renders into a <textarea>, whose displayed
           text is its VALUE and not its textContent once the browser has given it a value at all.
           One terminal implementation, two hosts -- so the target is chosen here rather than
           forked into a second widget. */
        let text = out.join("\n");
        if (this.el.tagName == "TEXTAREA") this.el.value = text; else this.el.textContent = text;
        this.el.scrollTop = this.el.scrollHeight;
    }

    clear() { this.lines = [""]; this.row = 0; this.col = 0; this.dirty = true; this.render(); }

    onKeyDown(e)
    {
        if (e.ctrlKey && (e.key === "c" || e.key === "v" || e.key === "C" || e.key === "V") && e.shiftKey) return;
        let s = null;
        if (e.key === "Enter") s = "\r";
        else if (e.key === "Backspace") s = "\x7F";          /* VMS rubout is DEL, not BS */
        else if (e.key === "Tab") s = "\t";
        else if (e.key === "Escape") s = "\x1B";
        else if (e.key === "ArrowUp") s = "\x1B[A";
        else if (e.key === "ArrowDown") s = "\x1B[B";
        else if (e.key === "ArrowRight") s = "\x1B[C";
        else if (e.key === "ArrowLeft") s = "\x1B[D";
        else if (e.ctrlKey && e.key.length === 1) {
            let c = e.key.toUpperCase().charCodeAt(0);
            if (c >= 0x40 && c < 0x60) s = String.fromCharCode(c & 0x1F);
        }
        else if (e.key.length === 1 && !e.metaKey) s = e.key;
        if (s === null) return;
        e.preventDefault();
        this.onInput(s);
    }
}

export default VaxTerminal;
