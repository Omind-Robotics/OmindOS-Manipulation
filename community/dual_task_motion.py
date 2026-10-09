"""Demo: fixed-base, shared-waist and dual RM75 rigid-grasp task-space tracking."""
import json
import numpy as np
from community.common import parser, report, show_robot
from community.pitch_robot import MobileDualPitch
from community.dual_task import plan_robot_object_motion


def run_task():
    robot = MobileDualPitch()
    start = robot.get_jnt_values(robot.component)
    # Virtual object frame at the TCP midpoint. No object/payload is fabricated.
    grasps = []
    for arm in ('lft_arm', 'rgt_arm'):
        p, r = robot.get_gl_tcp(arm)
        t = np.eye(4); t[:3, :3], t[:3, 3] = r, p
        grasps.append(t)
    origin = np.eye(4)
    origin[:3, 3] = (grasps[0][:3, 3] + grasps[1][:3, 3]) / 2
    for t in grasps:
        t[:3, 3] -= origin[:3, 3]
    raised = origin.copy(); raised[:3, 3] += [.015, 0, .01]
    plan = plan_robot_object_motion(robot, start, [origin, raised, origin], grasps)
    return robot, plan


def main():
    cli = parser(__doc__)
    from pathlib import Path
    cli.add_argument('--plan-output', type=Path)
    args = cli.parse_args()
    for path in (args.output, args.plan_output):
        if path:
            path.unlink(missing_ok=True)
    robot, plan = run_task()
    if args.plan_output:
        args.plan_output.parent.mkdir(parents=True, exist_ok=True)
        args.plan_output.write_text(json.dumps(plan, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    report(args, 'dual-task-rm75', joints=15,
           validated_samples=plan['validated_samples'],
           max_tcp_position_error_m=plan['max_tcp_position_error_m'],
           max_tcp_rotation_error_rad=plan['max_tcp_rotation_error_rad'],
           max_relative_position_error_m=plan['max_relative_position_error_m'],
           scope='Fixed-base sampled geometric tracking; virtual grasp frame, no payload/dynamics/hardware')
    if not args.headless:
        show_robot(robot, plan['positions_rad'], lambda q: robot.fk(robot.component, q), lookat=(.5, 0, 1))


if __name__ == '__main__':
    main()
