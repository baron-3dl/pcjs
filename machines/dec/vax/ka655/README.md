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

> ### Log in with `SYSTEM` / `QUOKKA1953`
>
> The machine boots itself: the KA655's self-tests first, then OpenVMS, about a minute to the
> `Username:` prompt.  Click the terminal before typing.

The machine below is a DEC **MicroVAX 3900**, the Q22-bus VAX built around the **KA655** CPU module:
a CVAX 78034 at 3.125 MHz with the CFPA floating-point accelerator, the CMCTL memory controller, the
CQBIC Qbus interface, and the KA655 console/diagnostic ROM at physical `0x20040000`.

Attached devices:

- [RQDX3 MSCP Disk Controller](/machines/dec/vax/rqdx3/), with an OpenVMS VAX V5.5 system disk
  auto-mounted on `DUA0:`
- The console serial line on `OPA0:`, as a glass TTY — DCL and the boot stream render correctly, but
  screen programs (EDT, EVE, NOTES) do not paint

The disk image is fetched read-only; writes go to an in-memory copy-on-write overlay that vanishes
when you close the tab.  Set `autoBoot="false"` on the `<cpu>` element of
[machine.xml](machine.xml) if you would rather stop at the KA655's `>>>` prompt and boot by hand.

{% include machine.html id="vax3900" %}

The port, the media it loads, and how to run it locally are described on the
[DEC VAX](/machines/dec/vax/) page.
