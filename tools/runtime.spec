# Build from the repository root with Python 3.12 / PyInstaller 6.11.1.
from pathlib import Path
from PyInstaller.utils.hooks import collect_data_files, collect_dynamic_libs

root = Path(SPECPATH).parent
demo_names = ('quick_start', 'ur3e_single_arm', 'grasping', 'motion_planning',
              'dual_arm', 'mobile_dual_arm', 'integrated_task', 'mobile_dual_pitch', 'rm75_navigation', 'dual_task_motion')
datas = [(str(root / 'configs/simulation'), 'configs/simulation'),
         (str(root / 'community/models'), 'community/models')]
datas += collect_data_files('robot_sim', includes=['**/*.stl', '**/*.STL', '**/*.dae', '**/*.obj'])
datas += collect_data_files('basis.trimesh', includes=['**/*.json', '**/*.template'])
binaries = collect_dynamic_libs('panda3d')
a = Analysis([str(root / 'community/runtime_entry.py')], pathex=[str(root)],
             binaries=binaries, datas=datas,
             hiddenimports=['community.api_worker'] + ['community.' + name for name in demo_names],
             hookspath=[], hooksconfig={}, runtime_hooks=[],
             excludes=['torch', 'tensorflow', 'IPython', 'jupyter', 'pytest', 'notebook',
                       'open3d.cuda', 'open3d.visualization.tensorboard_plugin'],
             noarchive=False)
# This release uses CPU geometric planning only; do not ship the unused CUDA runtime.
a.binaries = [entry for entry in a.binaries if not entry[0].startswith('open3d/cuda/')]
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name='omindos-runtime',
          debug=False, bootloader_ignore_signals=False, strip=False, upx=False, console=True)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name='omindos-runtime')
