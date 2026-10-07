"""Validated simulation-only seven-axis reference geometry; no ROS dependencies."""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from community.waist_config import vector
from community.rm75_config import RM75Config


@dataclass(frozen=True)
class SevenAxisConfig:
    mount_m: tuple
    mount_rpy: tuple
    lengths_m: tuple
    axes: tuple
    limits_rad: tuple
    home_rad: tuple
    link_width_m: float = .05

    def __post_init__(self):
        for name, count in [('mount_m', 3), ('mount_rpy', 3), ('lengths_m', 7), ('home_rad', 7)]:
            object.__setattr__(self, name, vector(getattr(self, name), count, name))
        if min(self.lengths_m) <= 0:
            raise ValueError('All seven link lengths must be positive')
        if len(self.axes) != 7 or len(self.limits_rad) != 7:
            raise ValueError('Exactly seven axes and seven joint limits are required')
        axes = tuple(vector(axis, 3, 'axis') for axis in self.axes)
        if any(abs(sum(v*v for v in axis) - 1.) > 1e-8 for axis in axes):
            raise ValueError('Each motion axis must be a unit vector')
        limits = tuple(vector(limit, 2, 'joint limits') for limit in self.limits_rad)
        if any(not -2*math.pi <= lo < hi <= 2*math.pi for lo, hi in limits):
            raise ValueError('Joint limits must be ordered within [-2pi, 2pi]')
        if any(not lo <= q <= hi for q, (lo, hi) in zip(self.home_rad, limits)):
            raise ValueError('Home configuration exceeds joint limits')
        width = vector([self.link_width_m], 1, 'link width')[0]
        if not 0 < width < min(self.lengths_m):
            raise ValueError('Link width must be positive and smaller than each link length')
        object.__setattr__(self, 'axes', axes)
        object.__setattr__(self, 'limits_rad', limits)
        object.__setattr__(self, 'link_width_m', width)

    def to_dict(self):
        return asdict(self)


def validate_arm_config(data):
    if data.get('schema') == 'omindos.community.dual_rm75.v1' and data.get('simulation_only') is True:
        return {side: RM75Config(**data[side]) for side in ('left', 'right')}
    if (data.get('schema') != 'omindos.community.dual_seven_axis.v1'
            or data.get('simulation_only') is not True):
        raise ValueError('Only simulation-only dual seven-axis geometry is supported')
    return {side: SevenAxisConfig(**data[side]) for side in ('left', 'right')}


def load_arm_config(path=None):
    if path is None:
        path = Path(__file__).resolve().parents[1] / 'configs/simulation/dual-rm75.reference.json'
    return validate_arm_config(json.loads(Path(path).read_text(encoding='utf-8')))


def arm_config_dict(configs):
    schema = ('omindos.community.dual_rm75.v1' if all(isinstance(c, RM75Config) for c in configs.values())
              else 'omindos.community.dual_seven_axis.v1')
    return dict(schema=schema, simulation_only=True,
                **{side: configs[side].to_dict() for side in ('left', 'right')})
