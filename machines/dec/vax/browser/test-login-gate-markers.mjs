// rd vms-5f6: ovmx-cluster.html's login-gate marker (GATE_READY, the console-advance nudge) and
// its authenticity marker (ACP_TELL_VMS_BANNER) each need to fire for ALL THREE node kinds the
// live demo boots: OVMX/VAX (Node B), a real V5.5 VAX/VMS volume, and a real V7.3 OpenVMS VAX
// volume. Before this fix:
//   - GATE_READY only matched the V5.5 string "Welcome to VAX/VMS", so an OVMX/VAX guest (which
//     prints "Welcome to the OpenVMX ... Operating System") never got the nudge and its console
//     stalled forever at "%RUN-S-PROC_ID, ..."; and a real V7.3 guest (which prints "Welcome to
//     OpenVMS ...", not "VAX/VMS" either) never got it either.
//   - ACP_TELL_VMS_BANNER only matched the V5.5-shaped "VAX/VMS Version Vn.n-xxx" banner, so a
//     real V7.3 volume (which identifies as "OpenVMS", not "VAX/VMS") never fired acp-ok.
//
// This test extracts the LIVE regex literals straight out of ovmx-cluster.html (so it fails if
// the source drifts from what's tested here) and checks them against verbatim excerpts of real
// captured consoles:
//   - OVMX/VAX Node B boot:  vms/tests/lab/captures/vms-e18e-cn3-browser-20260925/cn3-pass-CAB/OVMXB.console.log
//   - real V7.3 Node C boot: vms/tests/lab/captures/vms-e18e-cn3-browser-20260925/cn3-pass-CAB/VAXC.console.log
//                             (also vms/tests/lab/captures/vms-1ac-cn3-achieved-20260925/VAXC.console.log)
// V5.5 fixtures are hand-written from the ORIGINAL marker comments (no V5.5 capture ships in the
// vms repo's current tree any more -- Node C is real V7.3 now), just to prove the fix is additive
// and does not regress the original V5.5 match.

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const src = readFileSync(join(here, 'ovmx-cluster.html'), 'utf8');

function extractRegex(constName) {
  const m = src.match(new RegExp(`const ${constName}\\s*=\\s*(/.*/[a-z]*);`));
  if (!m) throw new Error(`could not find const ${constName} in ovmx-cluster.html`);
  // eslint-disable-next-line no-eval -- turning the source's own regex LITERAL text back into a RegExp
  return (0, eval)(m[1]);
}

const GATE_READY = extractRegex('GATE_READY');
const ACP_TELL_VMS_BANNER = extractRegex('ACP_TELL_VMS_BANNER');

// ---- verbatim excerpts (real captures, rd vms-5f6 / vms-e18e / vms-1ac) ----

// OVMX/VAX Node B, the login gate, from OVMXB.console.log (cn3-pass-CAB), lines 151-155:
const OVMXB_GATE = [
  '%RUN-S-PROC_ID, identification of created process is 0000006B',
  '',
  ' Welcome to the OpenVMX VAX Operating System, Version V0.7 (OpenVMS-compatible)',
  '',
  'Username: ',
].join('\r\n');

// real V7.3 Node C, the login gate, from VAXC.console.log (cn3-pass-CAB), lines 148-154:
const VAXC_V73_GATE = [
  '',
  ' Welcome to OpenVMS (TM) VAX Operating System, Version V7.3    ',
  '',
  'Username: ',
].join('\r\n');

// real V7.3 Node C, the early boot banner (fires well before login), line 23:
const VAXC_V73_BANNER = '   OpenVMS (TM) VAX Version V7.3     Major version id = 1 Minor version id = 0';

// V5.5 Node C, the ORIGINAL ground-truthed strings this marker was built for (rd vms-7ad,
// 2026-09-19) -- hand-written per the pre-fix marker comments, no V5.5 capture ships any more.
const VAXC_V55_GATE = ' Welcome to VAX/VMS\r\n\r\nUsername: ';
const VAXC_V55_BANNER = 'VAX/VMS Version V5.5-2H4';

// The OVMX welcome text must NEVER satisfy the real-VMS authenticity tell (that's the whole
// point of the tell being "real VMS only") -- otherwise a facade node could counterfeit it.
const OVMXB_WELCOME_ONLY = ' Welcome to the OpenVMX VAX Operating System, Version V0.7 (OpenVMS-compatible)';

const checks = [
  ['GATE_READY matches OVMX/VAX Node B (%RUN-S-PROC_ID) -- rd vms-5f6 fix',
    GATE_READY.test(OVMXB_GATE)],
  ['GATE_READY matches real V7.3 Node C ("Welcome to OpenVMS") -- rd vms-5f6 fix',
    GATE_READY.test(VAXC_V73_GATE)],
  ['GATE_READY still matches real V5.5 Node C ("Welcome to VAX/VMS") -- no regression',
    GATE_READY.test(VAXC_V55_GATE)],
  ['GATE_READY matches a bare JOB_CONTROL line (openvmx-site node.html parity)',
    GATE_READY.test('some prefix JOB_CONTROL suffix')],

  ['ACP_TELL_VMS_BANNER matches the real V7.3 banner -- rd vms-5f6 fix',
    ACP_TELL_VMS_BANNER.test(VAXC_V73_BANNER)],
  ['ACP_TELL_VMS_BANNER matches the real V7.3 welcome line too',
    ACP_TELL_VMS_BANNER.test(VAXC_V73_GATE)],
  ['ACP_TELL_VMS_BANNER still matches the V5.5-shaped banner -- no regression',
    ACP_TELL_VMS_BANNER.test(VAXC_V55_BANNER)],
  ['ACP_TELL_VMS_BANNER does NOT match the OVMX welcome banner (authenticity: no facade counterfeit)',
    !ACP_TELL_VMS_BANNER.test(OVMXB_WELCOME_ONLY)],
];

let pass = true;
console.log('=== rd vms-5f6: login-gate + authenticity marker checks ===');
for (const [name, ok] of checks) {
  console.log(`  ${ok ? 'PASS' : 'FAIL'}  ${name}`);
  if (!ok) pass = false;
}
console.log(pass ? '\nALL PASS' : '\nFAILURES');
process.exit(pass ? 0 : 1);
