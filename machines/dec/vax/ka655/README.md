---
layout: page
title: DEC MicroVAX 3900 (KA655) with OpenVMS VAX
permalink: /machines/dec/vax/ka655/
machines:
  - id: vax3900
    type: vax
    name: DEC MicroVAX 3900 (KA655) with 16Mb, RQDX3 and OpenVMS VAX
    config: /machines/dec/vax/ka655/machine.xml
    unbundled: true
---

The machine below is a DEC **MicroVAX 3900**, the Q22-bus VAX built around the **KA655** CPU module: a
CVAX 78034 at 3.125 MHz with the CFPA floating-point accelerator, the CMCTL memory controller, the
CQBIC Qbus interface, and the KA655 console/diagnostic ROM at physical `0x20040000`.

Attached devices:

- [RQDX3 MSCP Disk Controller](/machines/dec/vax/rqdx3/), with the OpenVMS system disk auto-mounted on `DUA0:`
- The console serial line on `OPA0:`, as a glass TTY

The machine auto-boots `DUA0:` and answers OpenVMS's date prompt, so it arrives at a `Username:`
prompt on its own.  Set `autoBoot="false"` on the `<cpu>` element (or press Reset and type at the
console yourself) if you would rather stop at the KA655's `>>>` prompt.

> ### Log in with `SYSTEM` / `QUOKKA1953`
>
> The whole boot takes about a minute — the KA655's self-tests run first, then OpenVMS starts up, and
> the login prompt is the last thing to appear.  Click the terminal before typing.
>
> There is no secret here worth keeping: this is a throwaway system disk running in your own browser
> tab, the credentials are in this repository, and nothing you do reaches anything else.  Writes go
> to an in-memory copy-on-write overlay and vanish when you close the tab — the disk image itself is
> fetched read-only and is never modified.

### What works, and what does not

The console is a **glass TTY, not a VT100** — DCL, the boot stream and the login prompt all render
correctly, but screen programs (EDT, EVE, NOTES) will not paint, and OpenVMS answers
`%SET-W-NOTSET … UNKTERM`.  A real VT100 is separate work.

There is **no save/restore**.  The KA655, its bus, its memory and its Qbus devices execute inside a
**Web Worker**, not on the page's main thread, so PCjs's `State`/`localStorage` resume path is not
wired up.  That is not a stylistic choice: the RQDX3's `diskRead()` is a *synchronous* read issued
from inside the instruction stream, and the main thread has no synchronous way to read a slice of a
`File` — so a main-thread machine would have to hold the entire 159 MB system disk in memory.  In a
Worker, `FileReaderSync` over `File.slice()` exists, and a boot touches a measured 55.3 MiB of it.

### Running it locally

The system disk and the console ROM are **not in this repository** — they are DEC-copyright media,
and they are served from the same kind of per-class media host PCjs uses for `decdisks`,
`diskettes` and the rest.  For local development, put them in the gitignored `/disks/vaxdisks`
mirror described by `_developer.yml`:

```
disks/vaxdisks/ka655x.json5      the console ROM, from tools/rom2json5.mjs
disks/vaxdisks/vms55-rd54.dsk    the OpenVMS V5.5-2H4 system disk (159,334,400 bytes = exactly RD54)
```

`ka655x.json5` is produced from a raw KA655 ROM image with:

```bash
node machines/dec/vax/tools/rom2json5.mjs ka655x.bin disks/vaxdisks/ka655x.json5
```

JSON5 rather than the raw `.bin` because that is what a `<rom file="..."/>` can load without a
server: PCjs's ROM component sends any other extension to the `DumpAPI` converter, which does not
exist on a static file server or on GitHub Pages.

{% include machine.html id="vax3900" %}

### The machine XML

- [Machine XML](machine.xml)
- [RQDX3 Device XML](/machines/dec/vax/rqdx3/default.xml)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<?xml-stylesheet type="text/xsl" href="/machines/dec/vax/xsl/machine.xsl"?>
<machine id="vax3900" type="vax" border="1" pos="center" background="default">
  <name pos="center">DEC MicroVAX 3900 (KA655) with 16Mb, RQDX3 and OpenVMS VAX</name>
  <computer id="computer" busWidth="30"/>
  <cpu id="cpu" model="KA655" cycles="3900000" autoStart="true" autoBoot="true"/>
  <ram id="ram" addr="0x00000000" size="0x1000000"/>
  <rom id="ka655x" addr="0x20040000" size="0x20000" file="/vaxdisks/ka655x.json5"/>
  <serial id="console" adapter="0" binding="print"/>
  <device ref="/machines/dec/vax/rqdx3/default.xml" autoMount='{DUA0:{path:"/vaxdisks/vms55-rd54.dsk"}}'/>
</machine>
```

> **A note on opening `machine.xml` directly.**  Every PCjs machine XML carries an
> `<?xml-stylesheet?>` processing instruction so that the raw file can be opened in a browser and
> rendered by the browser's own XSLT engine.  **That no longer works in current Chrome**: as of
> Chrome 149, both XSLT processing instructions and the `XSLTProcessor` API have been removed
> (`XSLTProcessor is not defined`).  This is why PCjs pages load
> `/assets/js/xslt-polyfill.min.js` before the machine scripts — the page above works, the raw XML
> URL does not.  The processing instruction is kept because it still works in engines that have
> not dropped XSLT yet.
