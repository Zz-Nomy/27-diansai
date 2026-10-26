"""Build BOTH boards locally with AC6. No flashing or hardware access."""
from pathlib import Path
import argparse
import concurrent.futures
import hashlib
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.stdout.reconfigure(encoding='utf-8')
HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--compiler-bin', default='C:/Users/28154/AppData/Local/Keil_v5/ARM/ARMCLANG/bin')
parser.add_argument('--board', choices=['flight', 'remote', 'all'], default='all')
args = parser.parse_args()
BIN = Path(args.compiler_bin)
reports = []
for board, directory, project in [('flight','02主控源码','mini.uvprojx'),
                                  ('remote','03遥控源码','remote.uvprojx')]:
    if args.board not in ('all', board): continue
    root = WORKSPACE/'02_飞控源码与说明'/directory
    out = HERE/'arm_build'/board
    out.mkdir(parents=True, exist_ok=True)
    flags = ['--target=arm-arm-none-eabi','-mcpu=cortex-m3','-mthumb','-std=gnu99',
             '-Oz','-g','-ffunction-sections','-fdata-sections','-DSTM32F10X_MD',
             '-DUSE_STDPERIPH_DRIVER','-Wno-nonportable-include-path']
    for folder in ['StartUp','Lib/inc','USER','MATH','CONFIG','HARDWARE','HAL','USB_hid','USB_library','APP']:
        if (root/folder).is_dir(): flags.append('-I'+folder)
    sources = [n.text.replace('\\','/') for n in ET.parse(root/project).findall('.//FilePath')
               if n.text.lower().endswith(('.c','.s'))]
    def compile_one(name):
        obj = out/(name.replace('./','').replace('/','_')+'.o')
        if name.lower().endswith('.s'):
            cmd = [str(BIN/'armasm.exe'),'--cpu','Cortex-M3','--apcs=interwork',
                   '--pd','__MICROLIB SETA 1','-o',str(obj),name]
        else:
            cmd = [str(BIN/'armclang.exe'),*flags,'-c',name,'-o',str(obj)]
        r = subprocess.run(cmd,cwd=root,capture_output=True,text=True,errors='replace')
        return {'source':name,'returncode':r.returncode,'diagnostics':r.stdout+r.stderr,'object':str(obj)}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        compiled = list(pool.map(compile_one,sources))
    (out/'compile_log.txt').write_text('\n'.join(r['source']+'\n'+r['diagnostics'] for r in compiled),encoding='utf-8')
    failed = [r for r in compiled if r['returncode']]
    report = {'board':board,'hardware_access':False,'sources':compiled,'compilation_passed':not failed,
              'source_sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in root.rglob('*') if p.is_file() and p.suffix.lower() in ('.c','.h','.s','.uvprojx')}}
    for r in failed: print(board,r['source'],r['diagnostics'])
    if not failed:
        scatter = out/'firmware.sct'
        scatter.write_text('''LR_IROM1 0x08000000 0x00010000 {
 ER_IROM1 0x08000000 0x00010000 { *.o (RESET, +First) *(InRoot$$Sections) .ANY (+RO) }
 RW_IRAM1 0x20000000 0x00005000 { .ANY (+RW +ZI) }
}
''',encoding='ascii')
        axf=out/(board+'_light.axf')
        cmd=[str(BIN/'armlink.exe'),'--cpu=Cortex-M3','--library_type=microlib','--scatter',str(scatter),
             '--entry=Reset_Handler','--map','--list',str(out/'firmware.map'),'--info=sizes',
             '--output',str(axf),*[r['object'] for r in compiled]]
        r=subprocess.run(cmd,cwd=root,capture_output=True,text=True,errors='replace')
        (out/'link_log.txt').write_text(r.stdout+r.stderr,encoding='utf-8')
        report['link_returncode']=r.returncode
        print(board,r.stdout+r.stderr)
        if not r.returncode:
            exports=[]
            for fmt,ext in [('--i32combined','hex'),('--bin','bin')]:
                filename=out/(board+'_light.'+ext)
                r=subprocess.run([str(BIN/'fromelf.exe'),fmt,'--output',str(filename),str(axf)],capture_output=True,text=True,errors='replace')
                exports.append({'path':str(filename),'returncode':r.returncode,'diagnostics':r.stdout+r.stderr,
                                'sha256':hashlib.sha256(filename.read_bytes()).hexdigest() if not r.returncode else None})
            report['exports']=exports
            report['passed']=all(r['returncode']==0 for r in exports)
    report.setdefault('passed',False)
    (out/'build_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(board, 'PASS' if report['passed'] else 'FAIL',len(compiled)-len(failed),'/',len(compiled),'sources')
    reports.append(report)
(HERE/'arm_build'/'build_result.json').write_text(json.dumps({'passed':all(r['passed'] for r in reports),'boards':reports},ensure_ascii=False,indent=2),encoding='utf-8')
raise SystemExit(0 if all(r['passed'] for r in reports) else 1)
