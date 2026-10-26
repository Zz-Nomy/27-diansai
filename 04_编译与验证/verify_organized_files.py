"""Recheck organization, original file preservation, local links and Keil references."""
from pathlib import Path
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET

sys.stdout.reconfigure(encoding='utf-8')
root = Path(__file__).resolve().parents[1]
archive = root / '无用'
receipt_path = root / '04_编译与验证/整理核验_20261025.json'
receipt = json.loads(receipt_path.read_text(encoding='utf-8-sig'))


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def markdown_targets(text):
    # Balanced parentheses also handle filenames such as 注意事项(1).txt.
    for match in re.finditer(r'\]\(', text):
        start = match.end()
        depth = 1
        for position in range(start, len(text)):
            if text[position] == '(':
                depth += 1
            elif text[position] == ')':
                depth -= 1
                if depth == 0:
                    yield text[start:position]
                    break
            elif text[position] == '\n':
                break


files = sorted(p for p in root.rglob('*') if p.is_file() and p != receipt_path)
hashes = {p: digest(p) for p in files}
available = set(hashes.values())
unpreserved = [item['path'] for item in receipt['original_manifest'] if item['sha256'] not in available]
active = [p for p in files if archive not in p.parents]
broken = []
for document in (p for p in active if p.suffix.lower() == '.md'):
    for target in markdown_targets(document.read_text(encoding='utf-8-sig')):
        if '://' in target or target.startswith('#'):
            continue
        target = re.sub(r':\d+$', '', target)
        if not (document.parent / target).exists():
            broken.append({'document': document.relative_to(root).as_posix(), 'target': target})

source = root / '02_飞控源码与说明/02主控源码'
tree = ET.parse(source / 'mini.uvprojx')
refs = [item.findtext('FilePath') for item in tree.findall('.//Groups/Group/Files/File')]
missing = [rel for rel in refs if not (source / rel.replace('\\', '/')).is_file()]
historical = json.loads((root / '04_编译与验证/历史验证记录/资料完整性核对.json').read_text(encoding='utf-8-sig'))
mismatches = [item['path'] for item in historical['source_manifest'] if digest(source / item['path']) != item['sha256']]

test_script = root / '04_编译与验证/verify_flow_reader.py'
test_text = test_script.read_text(encoding='utf-8-sig')
compile(test_text, str(test_script), 'exec')
assert 'workspace / "02_飞控源码与说明" / "02主控源码"' in test_text
assert len(refs) == 65 and len(historical['source_manifest']) == 134
assert not unpreserved and not broken and not missing and not mismatches, (unpreserved, broken, missing, mismatches)

receipt.update({
    'all_original_file_contents_preserved': True,
    'unpreserved_original_files': [],
    'active_files_excluding_receipt': len(active),
    'archived_files': len(files) - len(active),
    'project_references': len(refs),
    'missing_project_files': [],
    'supplier_source_hash_mismatches': [],
    'broken_local_markdown_links': [],
    'simulation_script_syntax_and_new_source_path': 'passed',
    'verification_status': 'passed',
    'verification_script': '04_编译与验证/verify_organized_files.py',
    'active_manifest': [
        {'path': p.relative_to(root).as_posix(), 'bytes': p.stat().st_size, 'sha256': hashes[p]}
        for p in active
    ],
})
receipt_path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n', encoding='utf-8-sig')
print(json.dumps({
    'status': 'passed',
    'original_files_preserved': len(receipt['original_manifest']),
    'active_files_including_receipt': len(active) + 1,
    'archived_files': len(files) - len(active),
    'source_files_unchanged': len(historical['source_manifest']),
    'project_references_present': len(refs),
    'broken_local_links': 0,
    'root_entries': [p.name for p in sorted(root.iterdir())],
}, ensure_ascii=False, indent=2))
