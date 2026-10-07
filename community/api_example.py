"""Run the public Python API examples; save results for inspection."""
import json
from pathlib import Path

from community.api import ManipulationClient


def main():
    with ManipulationClient() as client:
        motion = client.plan_joint_motion(
            start=[0., -.7, .9, .3, .6, 0.],
            goal=[.45, -.8, 1.1, .45, .8, -.15],
            obstacles=[dict(size_m=[.12, .12, .3], position_m=[.25, -.25, .15])])
        motion_result = client.wait(motion)
        grasp = client.plan_grasps([.02, .06, .04], jaw_width_m=.065)
        grasp_result = client.wait(grasp)
    results = dict(motion=motion_result, grasp=grasp_result)
    destination = Path('community-results/api-example.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(results, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for name, result in results.items():
        print(f"{name}: {result['status']}")
    print(destination)
    return 0 if all(r['status'] == 'succeeded' for r in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
