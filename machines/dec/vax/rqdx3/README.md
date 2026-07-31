---
layout: page
title: RQDX3 MSCP Disk Controller
permalink: /machines/dec/vax/rqdx3/
---

The RQDX3 is DEC's Q22-bus MSCP disk controller.  On this machine it presents the OpenVMS system
volume as `DUA0:`, and the emulated controller is the [RQDX3
Component](/machines/dec/vax/modules/v2/rqdx3.js) driving
[modules/v2/rq.js](/machines/dec/vax/modules/v2/rq.js), a port of Open SIMH's `pdp11_rq.c`.

Machines containing the RQDX3 Component include:

- [MicroVAX 3900 with OpenVMS VAX](/machines/dec/vax/ka655/)

The disks those machines can mount are listed in the following RQDX3 Device XML file:

- [Default](/machines/dec/vax/rqdx3/default.xml)

which is typically referenced by a Machine XML file as:

```xml
<device ref="/machines/dec/vax/rqdx3/default.xml" autoMount='{DUA0:{path:"/vaxdisks/vms55-rd54.dsk"}}'/>
```

Device XML files not only configure a device, but also list all the resources the device will use,
and define UI elements used to control the device, such as choosing which disks should be
"auto-mounted" by the RQDX3 device.  For example:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<device id="rqdx3" type="rqdx3" autoMountExample='{DUA0:{path:"/vaxdisks/vms55-rd54.dsk"}}' pos="left" width="45%" padLeft="8px" padBottom="8px">
	<control type="container">
		<control type="list" binding="listDisks">
			<disk id="disk01" name="VAX/VMS V5.5-2H4 (RD54)" path="/vaxdisks/vms55-rd54.dsk"/>
			<disk id="disk02" name="VAX/VMS V5.5-2H4 (RD54, gzip)" path="/vaxdisks/vms55-rd54.dsk.gz"/>
		</control>
		<control type="button" binding="loadDisk">Load</control>
		<control type="description" binding="descDisk" padRight="8px"/>
		<control type="file" binding="mountDisk"/>
	</control>
</device>
```

### Two things about VAX-scale media

**The images are raw, not JSON.**  PCjs's own disk component already loads raw binary images by
`path` — `pcx86/modules/v2/disk.js` documents `*.img` paths, fetches them with `getResource()`, and
hands non-string responses straight to `buildDisk()`.  So no conversion step is involved here.  What
*is* unusual is the size: an OpenVMS system volume is 159,334,400 bytes, roughly eight times the
largest media PCjs otherwise references (`/harddisks/pcx86/20mb/*.json`).

**A `.gz` path is decompressed transparently.**  GitHub rejects any file over 100 MB, so a
repo-hosted deployment cannot serve the raw image at all; gzipped it is 20,038,241 bytes.  The
decode is a `DecompressionStream` step inside this device's own loader, before the bytes reach the
drive — the same code path either way.  The shipped `autoMount` uses the **raw** image, served from
the `vaxdisks` media host; the `.gz` entry is listed for deployments that cannot.

Either way the machine page takes **no query parameters**: the media is named by the device XML, not
by the URL.
