"""RM75-B configuration and pinned manufacturer URDF data; stdlib only, no ROS."""
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
from community.waist_config import vector
from community.urdf_model import load_model

MODEL_DIR = Path(__file__).resolve().parent / 'models/realman_rm75'
MODEL_COMMIT = 'b63dddb20b7620d14cfff32678de60bb3a201ebc'


@lru_cache(maxsize=1)
def rm75_joints():
    root = ET.parse(MODEL_DIR / 'RM75-B.urdf').getroot()
    joints = []
    for i in range(1, 8):
        joint = root.find(f"joint[@name='joint_{i}']")
        origin, limit = joint.find('origin'), joint.find('limit')
        joints.append(dict(
            xyz=tuple(map(float, origin.get('xyz').split())),
            rpy=tuple(map(float, origin.get('rpy').split())),
            axis=tuple(map(float, joint.find('axis').get('xyz').split())),
            limits=(float(limit.get('lower')), float(limit.get('upper')))))
    return tuple(joints)


@dataclass(frozen=True)
class RM75Config:
    mount_m: tuple
    mount_rpy: tuple
    home_rad: tuple
    model: str = 'RM75-B'
    urdf_path: str | None = None
    _model_data: dict = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        if self.model not in ('RM75-B', 'URDF-7R'):
            raise ValueError('This pinned model supports RM75-B; other flange variants need their own model')
        if self.model == 'URDF-7R' and not self.urdf_path:
            raise ValueError('URDF-7R requires an explicit local urdf_path')
        if self.urdf_path is not None:
            object.__setattr__(self, 'urdf_path', str(Path(self.urdf_path).resolve(strict=True)))
        object.__setattr__(self, '_model_data', load_model(self.urdf_path or MODEL_DIR / 'RM75-B.urdf'))
        for name, length in [('mount_m', 3), ('mount_rpy', 3), ('home_rad', 7)]:
            object.__setattr__(self, name, vector(getattr(self, name), length, name))
        if any(not lo <= q <= hi for q, (lo, hi) in zip(self.home_rad, self.limits_rad)):
            raise ValueError('RM75 home configuration exceeds manufacturer URDF limits')

    @property
    def limits_rad(self):
        return tuple(j['limits'] for j in self._model_data['joints'])

    def to_dict(self):
        data = dict(mount_m=self.mount_m, mount_rpy=self.mount_rpy, home_rad=self.home_rad, model=self.model)
        if self.urdf_path is not None:
            data['urdf_path'] = self.urdf_path
        return data
