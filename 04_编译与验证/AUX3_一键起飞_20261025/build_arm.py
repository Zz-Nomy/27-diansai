"""Reproducible local AC6 Cortex-M3 build, no download or hardware access."""
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
OUT = Path(__file__).resolve().parent / 'arm_build'
OUT.mkdir(exist_ok=True)
ROOT = Path(__file__).resolve().parents[2] / '02_飞控源码与说明' / '02主控源码'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--compiler-bin',default=r'C:/Users/28154/AppData/Local/Keil_v5/ARM/ARMCLANG/bin')
args=parser.parse_args()
BIN = Path(args.compiler_bin)
FLAGS = ['--target=arm-arm-none-eabi', '-mcpu=cortex-m3', '-mthumb',
         '-std=gnu99', '-Oz', '-g', '-ffunction-sections', '-fdata-sections',
         '-DSTM32F10X_MD', '-DUSE_STDPERIPH_DRIVER',
         '-Wno-nonportable-include-path']
for directory in ('StartUp','Lib/inc','USER','MATH','CONFIG','HARDWARE','HAL','USB_hid','USB_library'):
    FLAGS.append('-I'+directory)
tree = ET.parse(ROOT / 'mini.uvprojx')
sources = [node.text.replace('\\','/') for node in tree.findall('.//FilePath')
           if node.text.lower().endswith(('.c', '.s'))]

def compile_one(name):
    output = OUT / (name.replace('./','').replace('/','_') + '.o')
    if name.lower().endswith('.s'):
        cmd = [str(BIN/'armasm.exe'), '--cpu', 'Cortex-M3', '--apcs=interwork',
               '--pd', '__MICROLIB SETA 1', '-o', str(output), name]
    else:
        cmd = [str(BIN/'armclang.exe'), *FLAGS, '-c', name, '-o', str(output)]
    try:
        r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, errors='replace')
    except OSError as e:
        return {'source':name, 'returncode':-1, 'diagnostics':str(e), 'object':str(output)}
    return {'source':name, 'returncode':r.returncode,
            'diagnostics':r.stdout+r.stderr, 'object':str(output)}

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    results = list(pool.map(compile_one, sources))
(OUT/'compile_log.txt').write_text('\n'.join(
    item['source']+'\n'+item['diagnostics'] for item in results), encoding='utf-8')
failures = [item for item in results if item['returncode']]
report = {'compiler':str(BIN/'armclang.exe'), 'sources':results,
          'compilation_passed':not failures, 'hardware_access':False,
          'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in sorted(ROOT.rglob('*'))
                           if p.is_file() and p.suffix.lower() in ('.c','.h','.s','.uvprojx')}}
for item in failures: print(item['source']+'\n'+item['diagnostics'])
if not failures:
    scatter = OUT/'mini.sct'
    scatter.write_text('''LR_IROM1 0x08000000 0x00010000 {
  ER_IROM1 0x08000000 0x00010000 {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00005000 { .ANY (+RW +ZI) }
}
''', encoding='ascii')
    axf = OUT/'mini_aux3.axf'
    cmd = [str(BIN/'armlink.exe'), '--cpu=Cortex-M3', '--library_type=microlib',
           '--scatter', str(scatter), '--entry=Reset_Handler', '--map',
           '--list', str(OUT/'mini_aux3.map'), '--info=sizes', '--output', str(axf)]
    cmd += [item['object'] for item in results]
    link = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, errors='replace')
    (OUT/'link_log.txt').write_text(link.stdout+link.stderr, encoding='utf-8')
    report['link_returncode'] = link.returncode
    print(link.stdout+link.stderr)
    if link.returncode == 0:
        for fmt, filename in (('--i32combined','mini_aux3.hex'),('--bin','mini_aux3.bin')):
            result = subprocess.run([str(BIN/'fromelf.exe'), fmt, '--output',str(OUT/filename),str(axf)],
                                    capture_output=True, text=True, errors='replace')
            report[filename] = {'returncode':result.returncode,'diagnostics':result.stdout+result.stderr}
            if result.returncode == 0:
                report[filename]['sha256'] = hashlib.sha256((OUT/filename).read_bytes()).hexdigest()
        report['exports_passed']=all(report[n]['returncode']==0 for n in ('mini_aux3.hex','mini_aux3.bin'))
        map_text=(OUT/'mini_aux3.map').read_text(errors='replace')
        report['memory_totals']={label:int(value) for label,value in re.findall(
            r'Total (RO|RW|ROM) Size[^\n]*?\s(\d+)\s+\(',map_text)}
        if report['exports_passed']:
            print('Full Cortex-M3 build, link and HEX/BIN export passed. This is not a flight validation.')
(OUT/'build_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(f'Compiled {len(sources)-len(failures)}/{len(sources)} source files.')
sys.exit(1 if failures or report.get('link_returncode') or not report.get('exports_passed') else 0)
