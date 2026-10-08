# Build and verify the OmindOS Manipulation 0.2 customer release

The development checkout keeps full source and tests. The customer kit is built
from an explicit allowlist: standard-library SDK, one editable task example,
manufacturer arm URDF/meshes, arm/waist/chassis scene settings and documentation.
The frozen runtime embeds Python bytecode and native dependencies. It does not
ship the core `.py` files and is not encryption or an anti-reverse-engineering
guarantee. Previously distributed copies retain their applicable licenses; this build
does not change repository visibility. The historical package namespace `community` stays for compatibility.

Customer downloads and installation instructions live in the [public release repository](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release).
The old Community repository is retained only for authorized historical access.
Builds use this private checkout and must not clone or download from the old repository.

## Build

Use a clean Linux x86_64 machine with Python 3.12 and a recorded dependency set.
Build on Ubuntu 24.04 for that target; frozen Linux programs still depend on
compatible system glibc/OpenGL/X11 libraries. This is not an ARM64/macOS/Windows
artifact and must be built and tested separately for those targets.

```bash
python3.12 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements-community.txt pyinstaller==6.11.1
python -m unittest community.test_contract community.test_api community.test_pitch community.test_dual7 community.test_rm75 community.test_navigation community.test_customer_api
python -m PyInstaller --noconfirm --clean tools/runtime.spec
python tools/package_customer_release.py --runtime dist/omindos-runtime --output /tmp/manipulation-candidate
python tools/verify_customer_release.py --candidate /tmp/manipulation-candidate
```

Use a new output directory for each candidate. Commit source first for a clean
release record; a working-tree candidate is explicitly marked as such in the
manifest. Capture the build log, frozen dependency TOC, environment inventory,
source audit and validation report. Run again on the actual target computer
before declaring that platform accepted. Reuse the existing Navigation v0.1 trace
fixture only as recorded evidence, not as a claim of a new live ROS2 run.

Two customer artifacts are produced: `OmindOS-Manipulation-0.2-client.tar.gz`
and `OmindOS-Manipulation-0.2-runtime-linux-amd64.tar.gz`, with SHA256 checksums.
The client install script checks the expected runtime checksum before extraction.
The standalone check extracts both into a fresh temporary directory, uses system
Python for the SDK, removes Python source search paths and runs the customer API
example through the frozen executable. It checks requested endpoints, phase
ordering, blocked-scene failure, model changes, cancellation and the core source
allowlist. Success must come from the extracted package, not the checkout.

## Distribution scope

The first release supports two seven-revolute-joint arm URDFs plus a configurable
pitch waist/chassis geometry. Unsupported model topology is rejected. This is
not a general whole-robot URDF loader. Metadata, fixed-base 15-joint motion and
navigation-trace/scene tasks all use the same external arm/waist settings.
Do not label sampled paths as controller trajectories or task cancellation as
hardware emergency stop. The runtime imports navigation evidence; it does not
publish live robot goals. Existing ROS2 collection tools remain developer tools.

The build includes WRS MIT, RM75 Apache-2.0 notices and dependency license files.
`licenses/build-environment.json` lists build distributions, which is deliberately
not presented as a complete runtime SBOM. No Navigation runtime or GPL FAST-LIO
component is copied into this package. A Docker wrapper, if later needed, must
use the tested candidate as its input rather than copy the development tree.

Primary packaging references:
[runtime/data paths](https://pyinstaller.org/en/stable/runtime-information.html),
[embedded bytecode](https://pyinstaller.org/en/stable/advanced-topics.html).
These tools only create local candidates. Publishing a tag or updating a release
is a separate operation; do not modify the existing `v0.2` tag/assets.
