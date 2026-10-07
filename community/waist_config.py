"""Validated reference geometry for the simulation-only pitch-waist model."""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


def vector(value, length, name):
    try:
        values = tuple(value)
        if len(values) != length or any(isinstance(v, (bool, str, bytes)) for v in values):
            raise ValueError
        values = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in values):
            raise ValueError
        return values
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} requires {length} finite numbers') from exc


@dataclass(frozen=True)
class WaistConfig:
    # These are illustrative geometry values, not measurements of a customer's robot.
    pivot_m: tuple
    limits_rad: tuple
    home_rad: float
    transport_rad: float
    chassis_center_m: tuple
    chassis_size_m: tuple
    torso_center_m: tuple
    torso_size_m: tuple
    schema: str = 'omindos.community.waist_pitch.v1'
    simulation_only: bool = True

    def __post_init__(self):
        if self.schema != 'omindos.community.waist_pitch.v1' or self.simulation_only is not True:
            raise ValueError('Only the simulation-only waist configuration is supported')
        for name in ('pivot_m', 'chassis_center_m', 'chassis_size_m', 'torso_center_m', 'torso_size_m'):
            values = vector(getattr(self, name), 3, name)
            if name.endswith('size_m') and min(values) <= 0:
                raise ValueError(f'{name} must be positive')
            object.__setattr__(self, name, values)
        limits = vector(self.limits_rad, 2, 'limits_rad')
        if not -math.pi <= limits[0] < limits[1] <= math.pi:
            raise ValueError('Waist limits must be ordered within [-pi, pi]')
        object.__setattr__(self, 'limits_rad', limits)
        for name in ('home_rad', 'transport_rad'):
            value = vector([getattr(self, name)], 1, name)[0]
            if not limits[0] <= value <= limits[1]:
                raise ValueError(f'{name} exceeds waist limits')
            object.__setattr__(self, name, value)

    def to_dict(self):
        return asdict(self)


def load_waist_config(path=None):
    if path is None:
        path = Path(__file__).resolve().parents[1] / 'configs/simulation/mobile-dual-pitch.reference.json'
    return WaistConfig(**json.loads(Path(path).read_text(encoding='utf-8')))
