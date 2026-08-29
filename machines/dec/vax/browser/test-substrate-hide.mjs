import { VaxTerminal } from './vaxterm.mjs';
// Minimal el stub (constructor only touches el.tabIndex/addEventListener; render sets textContent)
const el = { tagName:'PRE', textContent:'', value:'', scrollTop:0, scrollHeight:0,
  set tabIndex(_){}, addEventListener(){}, focus(){} };
const term = new VaxTerminal(el, ()=>{});

// Real boot transcript (from .boot-cache/lab-vax/demo-diag capture): KA655 POST -> VMB ->
// NetBSD secondary boot + kernel banner + device probe + SYSKRNL + kmod DEBUG -> OVMX banner -> login.
const boot = [
 "KA655-B V5.3, VMB 2.7",
 "Performing normal system tests.",
 "40..39..38..37..",
 "Tests completed.",
 ">>>B/R5:2 DUA0",
 "(BOOT/R5:2 DUA0)",
 "  2..",
 "-DUA0",
 "  1..0..",
 ">> NetBSD/vax boot [1.12 (Mon Dec 16 13:08:11 UTC 2024)] <<",
 ">> Press any key to abort autoboot 0",
 "> boot netbsd",
 "[   1.0000000] NetBSD 10.1 (OVMX) #0: Fri Aug 21 17:55:09 UTC 2026",
 "[   1.0000000] MicroVAX 3800/3900",
 "[   1.0000000] uda0 at uba0 csr 172150 vec 774 ipl 17",
 "[   1.0000000] mscpbus0 at uda0: version 3 model 3",
 "[   1.0900040] ra0 at mscpbus0 drive 0: RD54",
 "OVMX/NetBSD -- SYSKRNL (NetBSD kernel)",
 "[  10.4100040] DEBUG: module: Loading module from /stand/vax/10.1/modules/vms/vms.kmod",
 "[  12.6800040] DEBUG: module: module `vms' loaded successfully",
 "%OVMX-I-EXEC, VMS executive attached on /dev/vms",
 "",
 "    OpenVMX V0.5-8 - OpenVMS-compatible",
 "     1-JAN-2010 12:47:09.53",
 "%OVMX-I-MOUNTED, system disk DKA0: mounted",
 "Username: SYSTEM",
 "Password: ",
 "   Welcome to OpenVMX V0.5-8 - OpenVMS-compatible",
 "$ ",
];
// Feed as console bytes exactly as the worker would (LF between lines)
term.write(new TextEncoder().encode(boot.join("\r\n")));

// Replicate ovmx.html's marker logic
const RE_HIDE_START=/NetBSD/, RE_REVEAL=/^%OVMX-|OpenVMS-compatible/;
let hideStartIdx=-1, revealIdx=-1;
for(let i=0;i<term.lines.length;i++){
  if(hideStartIdx<0){ if(RE_HIDE_START.test(term.lines[i])) hideStartIdx=i; }
  else if(RE_REVEAL.test(term.lines[i])){ revealIdx=i; break; }
}
term.hideRange={start:hideStartIdx, end: revealIdx<0?Infinity:revealIdx};
term.dirty=true; term.render();
const rendered = el.textContent;

console.log('hideStartIdx=',hideStartIdx,'("'+term.lines[hideStartIdx]+'")');
console.log('revealIdx=',revealIdx,'("'+term.lines[revealIdx]+'")');
console.log('\n=== RENDERED (viewer sees) ===\n'+rendered);
const has=(s,re)=>re.test(s);
const checks=[
 ['hides NetBSD',                !has(rendered,/NetBSD/)],
 ['hides device probe',          !has(rendered,/mscpbus|RD54|uda0 at uba0/)],
 ['hides SYSKRNL line',          !has(rendered,/SYSKRNL/)],
 ['hides kmod DEBUG',            !has(rendered,/DEBUG: module/)],
 ['keeps KA655 POST',            has(rendered,/KA655-B V5\.3/)],
 ['keeps >>> BOOT',              has(rendered,/>>>B\/R5:2 DUA0/)],
 ['keeps VMB (BOOT/R5:2)',       has(rendered,/\(BOOT\/R5:2 DUA0\)/)],
 ['shows OVMX banner',           has(rendered,/OpenVMX V0\.5-8 - OpenVMS-compatible/)],
 ['shows %OVMX-I-EXEC',          has(rendered,/%OVMX-I-EXEC/)],
 ['shows Username:',             has(rendered,/Username:/)],
 ['shows $ prompt',              has(rendered,/\$ /)],
 ['MODEL retains NetBSD (detect intact)', has(term.lines.join("\n"),/NetBSD/)],
];
let pass=true; console.log('\n=== ASSERTIONS ===');
for(const [n,v] of checks){ console.log(`  ${v?'PASS':'FAIL'}  ${n}`); if(!v)pass=false; }
console.log(pass?'\nALL PASS':'\nFAILURES'); process.exit(pass?0:1);
