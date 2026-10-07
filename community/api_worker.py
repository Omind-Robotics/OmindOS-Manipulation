"""Internal offline worker for community.api; invoke the public client instead."""
import json
from pathlib import Path
import sys


def joint_motion(request):
    import numpy as np
    import modeling.collision_model as cm
    from community.common import plan_path, require
    from community.interface import JOINTS
    from community.robots import SingleArm

    robot = SingleArm()
    start, goal = np.array(request['start']), np.array(request['goal'])
    limits = np.asarray(robot.get_jnt_ranges('arm'))
    for label, q in [('start', start), ('goal', goal)]:
        require(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]),
                label + ' is outside UR3e joint limits')
    obstacles = []
    for box in request['obstacles']:
        obstacle = cm.gen_box(extent=box['size_m'])
        obstacle.set_pos(np.array(box['position_m']))
        obstacles.append(obstacle)
    path = plan_path(robot, 'arm', start, goal, obstacles)
    return dict(robot='UR3e', frame_id='arm_base', joint_names=list(JOINTS),
                positions_rad=[q.tolist() for q in path], validated_samples=len(path),
                edge_step_rad=.025, obstacles=request['obstacles'],
                scope='Bare-arm geometric path; no tool, payload, time parameterization or execution')


def grasps(request):
    import modeling.collision_model as cm
    from community.common import require
    from community.grasping import candidates
    from robot_sim.end_effectors.grippers.robotiq85.robotiq85 import Robotiq85

    obj = cm.gen_box(extent=request['size_m'])
    gripper = Robotiq85()
    found = candidates(gripper, obj, width=request['jaw_width_m'])
    require(len(found) > 0, 'No collision-free grasp candidates for this box and jaw width')
    result = []
    for width, jaw_pos, jaw_rot, hand_pos, hand_rot in found:
        gripper.fix_to(hand_pos, hand_rot)
        gripper.jaw_to(width)
        require(not gripper.is_mesh_collided([obj]), 'Grasp intersects object geometry')
        result.append(dict(jaw_width_m=float(width), jaw_center_position_m=jaw_pos.tolist(),
                           jaw_center_rotation=jaw_rot.tolist(),
                           gripper_position_m=hand_pos.tolist(), gripper_rotation=hand_rot.tolist()))
    return dict(gripper='Robotiq85', frame_id='object', box_size_m=request['size_m'],
                candidates=result, candidate_count=len(result),
                scope='Geometric candidates; no contact, force closure, reachability or approach validation')


def mobile_robot(request):
    from community.pitch_robot import MobileDualPitch
    from community.seven_axis_config import validate_arm_config
    from community.waist_config import WaistConfig
    arms = validate_arm_config(request['arm_config']) if request.get('arm_config') else None
    waist = WaistConfig(**request['waist_config']) if request.get('waist_config') else None
    return MobileDualPitch(config=waist, arm_configs=arms)


def mobile_model(request):
    from community.mobile_dual_pitch import JOINT_NAMES
    robot = mobile_robot(request)
    return dict(dof=15, joint_names=JOINT_NAMES,
                home_positions_rad=robot.get_jnt_values(robot.component).tolist(),
                limits_rad=robot.get_jnt_ranges(robot.component).tolist(),
                arm_urdf_joint_names={side: [j['name'] for j in cfg._model_data['joints']]
                                     for side, cfg in robot.arm_configs.items() if hasattr(cfg, '_model_data')},
                base_coordinates=['x_m', 'y_m', 'yaw_rad'],
                scope='Model metadata only; reference body/tool geometry requires calibration')


def mobile_joint_motion(request):
    import numpy as np
    import modeling.collision_model as cm
    from community.common import plan_path
    from community.mobile_dual_pitch import JOINT_NAMES
    from community.rm75_navigation import set_pose
    robot = mobile_robot(request)
    set_pose(robot, request['base_pose'])
    start, goal = np.array(request['start']), np.array(request['goal'])
    robot.fk(robot.component, goal)
    robot.fk(robot.component, start)
    obstacles = []
    for box in request['obstacles']:
        obstacle = cm.gen_box(extent=box['size_m'])
        obstacle.set_pos(np.array(box['position_m']))
        obstacles.append(obstacle)
    path = plan_path(robot, robot.component, start, goal, obstacles)
    return dict(dof=15, joint_names=JOINT_NAMES, frame_id='map', base_pose=request['base_pose'],
                positions_rad=[q.tolist() for q in path], validated_samples=len(path), edge_step_rad=.025,
                obstacles=request['obstacles'],
                scope='Fixed-base untimed geometric path; sampled collision validation; no hardware execution')


def mobile_task(request):
    from community.rm75_navigation import run_task
    from community.seven_axis_config import validate_arm_config
    from community.waist_config import WaistConfig
    arms = validate_arm_config(request['arm_config']) if request.get('arm_config') else None
    waist = WaistConfig(**request['waist_config']) if request.get('waist_config') else None
    _, _, result = run_task(request['scene'], request['navigation_trace'], arm_configs=arms,
                            waist_config=waist, target_positions=request['target_positions'])
    return result


def main():
    from community.common import seed_rng
    request = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
    seed_rng(request['seed'])
    operation = request['operation']
    result = {'joint_motion': joint_motion, 'grasps': grasps, 'mobile_model': mobile_model,
              'mobile_joint_motion': mobile_joint_motion, 'mobile_task': mobile_task}[operation](request)
    payload = dict(result)
    payload.update(schema='omindos.manipulation.planning.v0.2', status='passed',
                   mode='simulation', simulation_only=True, seed=request['seed'],
                   operation=operation)
    Path(sys.argv[2]).write_text(json.dumps(payload, indent=2, allow_nan=False) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
