"""Customer API example: chassis navigation, stop gate, then dual arms + waist."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
# Works both in the extracted release and in the full development checkout.
if not (ROOT / 'sdk').is_dir():
    ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
from sdk.omindos_client import ManipulationClient


def main():
    robot = ROOT / 'robot'
    if robot.is_dir():
        arms, waist, scene_path = robot/'arms.json', robot/'waist.json', robot/'navigation.json'
    else:
        arms = ROOT/'configs/simulation/dual-rm75.reference.json'
        waist = ROOT/'configs/simulation/rm75-waist.reference.json'
        scene_path = ROOT/'configs/simulation/rm75-navigation.reference.json'
    with ManipulationClient() as client:
        metadata = client.wait(client.get_mobile_model(arm_config=arms, waist_config=waist))
        if metadata['status'] != 'succeeded':
            raise RuntimeError(metadata['error'])
        start = metadata['result']['home_positions_rad']
        goal = start.copy()
        goal[0], goal[7], goal[14] = .1, .2, -.2
        joint_result = client.wait(client.plan_mobile_joint_motion(
            start, goal, arm_config=arms, waist_config=waist))
        if joint_result['status'] != 'succeeded':
            raise RuntimeError(joint_result['error'])
        scene = json.loads(scene_path.read_text(encoding='utf-8'))
        mobile_result = client.wait(client.plan_mobile_task(
            scene=scene, target_positions=goal, arm_config=arms, waist_config=waist))
        if mobile_result['status'] != 'succeeded':
            raise RuntimeError(mobile_result['error'])
        output = dict(model=metadata, joint_motion=joint_result, mobile_task=mobile_result)
    destination = ROOT/'results/customer-api.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    plan = mobile_result['result']
    print(json.dumps(dict(status='passed', dof=15,
                         base_final_pose=plan['navigation']['samples'][-1]['pose'],
                         phase_history=plan['phase_history'],
                         joint_samples=plan['validated_joint_samples']), ensure_ascii=False))


if __name__ == '__main__':
    main()
