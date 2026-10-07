"""Demo 08: standalone 15-DoF kinematics (7 left + 7 right + pitch waist), no ROS."""
import json
from pathlib import Path
import numpy as np
from community.common import parser, seed_rng, require, plan_path, report, show_robot
from community.pitch_robot import MobileDualPitch
from community.seven_axis_config import load_arm_config, arm_config_dict
from community.waist_config import load_waist_config

JOINT_NAMES = (['waist_pitch_joint'] + [f'left_joint_{i}' for i in range(1, 8)]
               + [f'right_joint_{i}' for i in range(1, 8)])


def main():
    cli = parser(__doc__)
    cli.add_argument('--waist-config', type=Path, help='Uncalibrated simulation waist geometry JSON')
    cli.add_argument('--arm-config', type=Path, help='Dual arm configuration JSON (default: manufacturer RM75-B)')
    cli.add_argument('--pitch-rad', type=float, default=.15, help='Final waist angle in radians')
    cli.add_argument('--plan-output', type=Path, help='Export offline kinematic samples; not a ROS2/controller trajectory')
    args = cli.parse_args()
    seed_rng(args.seed)
    config = load_waist_config(args.waist_config) if args.waist_config else None
    arms = load_arm_config(args.arm_config)
    robot = MobileDualPitch(config, arm_configs=arms)
    config = robot.waist_config
    start = robot.get_jnt_values(robot.component)
    goal = start.copy()
    goal[0] = args.pitch_rad
    goal[1:8] += np.array([.12, -.10, .08, .12, -.08, .10, .20])
    goal[8:] += np.array([-.12, -.10, -.08, .12, .08, .10, -.20])
    # Reject invalid input before entering RRT; all fifteen joints participate.
    robot.fk(robot.component, goal)
    robot.fk(robot.component, start)
    base_before = robot.pos.copy(), robot.rotmat.copy()
    states = plan_path(robot, robot.component, start, goal)
    require(np.allclose(robot.pos, base_before[0]) and np.allclose(robot.rotmat, base_before[1]),
            'Waist planning moved the fixed chassis')
    if args.plan_output:
        plan = dict(schema='omindos.community.kinematics.dual7_pitch.v1', simulation_only=True,
                    dof=15, joint_names=JOINT_NAMES, waist_config=config.to_dict(),
                    arm_config=arm_config_dict(arms),
                    base_position_m=robot.pos.tolist(), base_rotation=robot.rotmat.tolist(),
                    positions_rad=[q.tolist() for q in states])
        args.plan_output.parent.mkdir(parents=True, exist_ok=True)
        args.plan_output.write_text(json.dumps(plan, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    report(args, '08-mobile-dual-pitch', joints=15, arm_dof=[7, 7], waist_axis='base +Y',
           waist_start_rad=float(start[0]), waist_goal_rad=float(goal[0]),
           validated_joint_samples=len(states), ros_required=False,
           arm_models=[getattr(arms[side], 'model', 'generic-seven-axis') for side in ('left', 'right')],
           geometry='manufacturer RM75-B arms or configured reference chain; uncalibrated body/mounts and reference grippers',
           scope='Standalone kinematics and sampled collision checks; no dynamics or hardware execution')
    if not args.headless:
        show_robot(robot, states + states[-2:0:-1], lambda q: robot.fk(robot.component, q), lookat=(.5, 0, 1))


if __name__ == '__main__':
    main()
