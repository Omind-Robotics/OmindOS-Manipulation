"""Package an already built frozen runtime and a small editable customer kit.

The output is a candidate until tools/verify_customer_release.py passes. No tag,
GitHub Release or deployment is changed by this tool.
"""
import argparse
import gzip
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]
RUNTIME_NAME = 'OmindOS-Manipulation-0.2-runtime-linux-amd64.tar.gz'
CLIENT_NAME = 'OmindOS-Manipulation-0.2-client.tar.gz'


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def archive(source, path):
    with tarfile.open(path, 'w:gz', compresslevel=6) as out:
        out.add(source, arcname=source.name)


def audit(runtime):
    core = ('community', 'motion', 'modeling', 'robot_sim', 'basis', 'visualization')
    leaked = [str(p.relative_to(runtime)) for p in runtime.rglob('*')
              if p.is_file() and p.suffix in ('.py', '.pyx', '.c', '.cpp')
              and any(part in core for part in p.relative_to(runtime).parts)]
    if leaked:
        raise RuntimeError('Core plaintext source in runtime: ' + ', '.join(leaked[:10]))
    if list(runtime.rglob('.git')):
        raise RuntimeError('Git history must not enter the runtime')
    return dict(status='passed', core_plaintext_files=0,
                packaging='PyInstaller frozen Python bytecode and shared libraries; not encryption',
                runtime_files=sum(p.is_file() for p in runtime.rglob('*')))


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--runtime', type=Path, required=True)
    cli.add_argument('--output', type=Path, required=True)
    args = cli.parse_args()
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise SystemExit('This candidate is Linux x86_64 only')
    runtime_input = args.runtime.resolve(strict=True)
    if not (runtime_input/'omindos-runtime').is_file():
        raise SystemExit('Missing frozen omindos-runtime executable')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    runtime = output/'staging/runtime'
    shutil.copytree(runtime_input, runtime)
    licenses = runtime/'licenses'
    licenses.mkdir()
    shutil.copy2(ROOT/'LICENSE', licenses/'WRS-MIT.txt')
    shutil.copy2(ROOT/'ATTRIBUTION.md', licenses/'ATTRIBUTION.md')
    inventory = []
    for dist in metadata.distributions():
        name = dist.metadata.get('Name', 'unknown')
        files = [p for p in (dist.files or []) if any(
            x.lower().startswith(('license', 'copying', 'notice', 'copyright')) for x in p.parts)]
        inventory.append(dict(name=name, version=dist.version,
                              license_metadata=dist.metadata.get('License'),
                              license_files=[str(p) for p in files]))
        for path in files:
            source = Path(dist.locate_file(path))
            if source.is_file():
                destination = licenses/'dependencies'/name/str(path)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
    (licenses/'build-environment.json').write_text(json.dumps(inventory, indent=2)+'\n')
    # These are build inputs, not a claim that every listed distribution is used
    # by the executable. PyInstaller's own TOC is retained in developer evidence.
    audit_result = audit(runtime)
    client = output/'staging/OmindOS-Manipulation-0.2'
    (client/'sdk').mkdir(parents=True)
    for path in ('__init__.py', 'omindos_client.py'):
        shutil.copy2(ROOT/'sdk'/path, client/'sdk'/path)
    (client/'examples').mkdir()
    shutil.copy2(ROOT/'examples/customer/mobile_dual_api.py', client/'examples/mobile_dual_api.py')
    shutil.copytree(ROOT/'community/models/realman_rm75', client/'robot/arm-model')
    # Uncompressed mesh copies make the provided manufacturer URDF usable by
    # ordinary external URDF viewers as well as the frozen runtime.
    for path in (client/'robot/arm-model/meshes').glob('*.gz'):
        path.with_suffix('').write_bytes(gzip.decompress(path.read_bytes()))
        path.unlink()
    arms = json.loads((ROOT/'configs/simulation/dual-rm75.reference.json').read_text())
    for side in ('left', 'right'):
        arms[side]['urdf_path'] = 'arm-model/RM75-B.urdf'
    (client/'robot/arms.json').write_text(json.dumps(arms, indent=2)+'\n')
    shutil.copy2(ROOT/'configs/simulation/rm75-waist.reference.json', client/'robot/waist.json')
    shutil.copy2(ROOT/'configs/simulation/rm75-navigation.reference.json', client/'robot/navigation.json')
    shutil.copy2(ROOT/'docs/CUSTOMER_RELEASE.zh-CN.md', client/'README.md')
    shutil.copy2(ROOT/'LICENSE', client/'LICENSE')
    shutil.copy2(ROOT/'ATTRIBUTION.md', client/'ATTRIBUTION.md')
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip())
    manifest = dict(product='OmindOS Manipulation', version='0.2', package_revision='runtime.1',
                    base_commit=revision, uncommitted_changes=dirty, simulation_only=True,
                    platform=platform.platform(), machine=platform.machine(), source_audit=audit_result)
    (client/'VERSION.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    archive(runtime, output/RUNTIME_NAME)
    runtime_hash = sha(output/RUNTIME_NAME)
    install = '''#!/bin/sh
set -eu
cd "$(dirname "$0")"
test "$(uname -s)" = Linux && test "$(uname -m)" = x86_64 || { echo 'Linux x86_64 required'; exit 1; }
command -v python3 >/dev/null
test ! -e runtime || { echo 'runtime already exists; use a fresh release directory'; exit 1; }
printf '%s  %s\\n' 'RUNTIME_HASH' '../RUNTIME_NAME' | sha256sum -c -
tar -xzf ../RUNTIME_NAME
./runtime/omindos-runtime --version
echo 'Installed. Run: python3 examples/mobile_dual_api.py'
'''.replace('RUNTIME_HASH', runtime_hash).replace('RUNTIME_NAME', RUNTIME_NAME)
    (client/'install.sh').write_text(install)
    (client/'install.sh').chmod(0o755)
    archive(client, output/CLIENT_NAME)
    (output/'SHA256SUMS').write_text(''.join(f'{sha(output/name)}  {name}\n' for name in (CLIENT_NAME, RUNTIME_NAME)))
    print(json.dumps(dict(output=str(output), files=[CLIENT_NAME, RUNTIME_NAME, 'SHA256SUMS'], audit=audit_result)))


if __name__ == '__main__':
    main()
