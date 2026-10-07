"""Legacy UR3e example: mock base relocation and 13-joint geometric planning."""
import json
from pathlib import Path
import numpy as np
from community.common import parser, seed_rng, require, plan_path, report, show_robot
from community.interface import BaseGoal, TaskGate, trajectory_joint_names, validate_playback
from community.mobile_dual_arm import navigate
from community.legacy_pitch_robot import MobileDualPitch
from community.waist_config import load_waist_config


def main():
    cli = parser(__doc__)
    cli.add_argument('--waist-config', type=Path, help='Simulation geometry JSON; default is an uncalibrated reference')
    cli.add_argument('--pitch-rad', type=float, default=.15, help='Simulated final waist angle in radians')
    cli.add_argument('--plan-output', type=Path)
    args = cli.parse_args()
    seed_rng(args.seed)
    config = load_waist_config(args.waist_config)
    robot = MobileDualPitch(config)
    robot.set_waist_pitch(config.transport_rad)
    gate = TaskGate()
    gate.start_navigation()
    base_goal = BaseGoal(.35, .1, .15)
    route = navigate(robot, base_goal)
    gate.navigation_result(True, True)
    gate.start_manipulation()
    start = robot.get_jnt_values(robot.component)
    goal = start.copy()
    goal[0] = args.pitch_rad
    goal[1:7] += np.array([.04, -.03, .03, 0, .02, 0])
    goal[7:] -= np.array([.04, -.03, .03, 0, .02, 0])
    # Reject invalid input before entering the original RRT implementation.
    robot.fk(robot.component, goal)
    robot.fk(robot.component, start)
    base_before = robot.pos.copy(), robot.rotmat.copy()
    states = plan_path(robot, robot.component, start, goal)
    require(np.allclose(robot.pos, base_before[0]) and np.allclose(robot.rotmat, base_before[1]),
            'Waist planning moved the fixed chassis')
    gate.manipulation_result(True)
    plan = dict(schema='omindos.community.playback.waist_pitch.v1', simulation_only=True,
                base_goal=vars(base_goal), waist_config=config.to_dict(),
                joint_names=trajectory_joint_names(True),
                points=[dict(time_s=round(i * .2, 6), positions=q.tolist()) for i, q in enumerate(states)])
    validate_playback(plan)
    if args.plan_output:
        args.plan_output.parent.mkdir(parents=True, exist_ok=True)
        args.plan_output.write_text(json.dumps(plan, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    report(args, '08-mobile-dual-pitch', joints=13, waist_axis='base +Y',
           waist_start_rad=float(start[0]), waist_goal_rad=float(goal[0]),
           base_samples=len(route), validated_joint_samples=len(states), task_history=gate.history,
           geometry='uncalibrated configurable chassis/torso boxes and upstream UR3e arms',
           scope='Mock navigation and sampled geometric planning; no stability, dynamics or hardware execution')
    if not args.headless:
        show_robot(robot, states, lambda q: robot.fk(robot.component, q), lookat=(.7, 0, 1))


if __name__ == '__main__':
    main()
