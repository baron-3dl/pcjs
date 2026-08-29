import { VaxTerminal } from './vaxterm.mjs';
const mk=()=>{const el={tagName:'PRE',textContent:'',set tabIndex(_){},addEventListener(){},focus(){},scrollTop:0,scrollHeight:0};return [new VaxTerminal(el,()=>{}),el];};
// boot to reveal, set hideRange, THEN the guest clears the screen (ESC[2J) and prints a DCL session
const [term,el]=mk();
const boot=["KA655-B V5.3","test..","(BOOT/R5:2 DUA0)",">> NetBSD/vax boot <<","mscpbus0 at uda0","ra0: RD54","%OVMX-I-EXEC, VMS executive attached on /dev/vms","    OpenVMX V0.5-8 - OpenVMS-compatible","Username: SYSTEM","$ "];
term.write(new TextEncoder().encode(boot.join("\r\n")));
let hs=-1,rv=-1; for(let i=0;i<term.lines.length;i++){if(hs<0){if(/NetBSD/.test(term.lines[i]))hs=i;}else if(/^%OVMX-|OpenVMS-compatible/.test(term.lines[i])){rv=i;break;}}
term.hideRange={start:hs,end:rv}; term.dirty=true; term.render();
console.log('before clear: hideRange=',JSON.stringify(term.hideRange),' rendered.len=',el.textContent.length);
// guest clears screen then prints DCL content
term.write(new TextEncoder().encode("\x1b[2J\r\n$ SHOW TIME\r\n  1-JAN-2010 12:54\r\n$ "));
term.render();
console.log('after ESC[2J: hideRange=',JSON.stringify(term.hideRange),' lines=',term.lines.length,' rendered.len=',el.textContent.length);
console.log('=== RENDERED after clear ===\n['+el.textContent+']');
console.log(el.textContent.replace(/[█\s]/g,'').length===0 ? '>>> REPRODUCED: empty/over-hidden after clear' : '>>> shows content');
