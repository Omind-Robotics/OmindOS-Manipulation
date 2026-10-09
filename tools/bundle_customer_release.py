"""Bundle only accepted customer artifacts for publication, without source files."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

cli = argparse.ArgumentParser(description=__doc__)
cli.add_argument('--candidate', type=Path, required=True)
args = cli.parse_args()
root = args.candidate.resolve(strict=True)
validation = json.loads((root/'VALIDATION.json').read_text())
assert validation['status'] == 'passed' and validation['package_revision'] == 'runtime.2'
names = ['OmindOS-Manipulation-0.2-client.tar.gz',
         'OmindOS-Manipulation-0.2-runtime-linux-amd64.tar.gz',
         'SHA256SUMS', 'manifest.json', 'VALIDATION.json']
names += [p.name for p in sorted(root.glob('*.log'))]
archive = root/'OmindOS-Manipulation-0.2-Customer-Release-Linux-amd64.zip'
with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_STORED) as out:
    for name in names:
        out.write(root/name, name)
with archive.open('rb') as stream:
    digest = hashlib.file_digest(stream, 'sha256').hexdigest()
(root/'SHA256SUMS-Customer-Release').write_text(f'{digest}  {archive.name}\n')
print(json.dumps(dict(file=archive.name, bytes=archive.stat().st_size, sha256=digest)))
