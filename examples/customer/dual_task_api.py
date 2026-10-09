"""Customer example: coupled waist and dual RM75 task-space path, standard library only."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sdk.omindos_client import ManipulationClient


def identity():
    return [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]


def finish(client, task):
    result = client.wait(task, timeout=120)
    if result['status'] != 'succeeded':
        raise RuntimeError(result['error'])
    return result['result']


def main():
    with ManipulationClient() as client:
        info = finish(client, client.get_mobile_model(
            arm_config='robot/arms.json', waist_config='robot/waist.json'))
        origin, moved = identity(), identity()
        moved[0][3], moved[2][3] = .015, .01
        plan = finish(client, client.plan_mobile_object_motion(
            info['home_positions_rad'], [origin, moved, origin], info['home_tcp_poses'],
            arm_config='robot/arms.json', waist_config='robot/waist.json'))
        assert plan['dof'] == 15 and not plan['timed'] and not plan['hardware_executed']
        assert plan['validated_samples'] > 2
        output = Path('results/dual-task-api.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(plan, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        print(json.dumps({k: plan[k] for k in ('status', 'dof', 'validated_samples',
              'max_tcp_position_error_m', 'max_tcp_rotation_error_rad',
              'max_relative_position_error_m', 'simulation_only')}, indent=2))


if __name__ == '__main__':
    main()
