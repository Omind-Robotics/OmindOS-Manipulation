"""Demo 09: chassis navigation, verified stop, then dual RM75 + pitch waist."""
import itertools
import json
import math
from pathlib import Path
import numpy as np
import basis.robot_math as rm
import modeling.collision_model as cm
from community.base_navigation import Grid, kinematic_trace
from community.navigation_contract import validate_trace, angle_delta, finite_vector
from community.common import parser, seed_rng, require, report, plan_path, show_robot
from community.interface import TaskGate
from community.pitch_robot import MobileDualPitch
from community.mobile_dual_pitch import JOINT_NAMES
from community.seven_axis_config import arm_config_dict

DEFAULT_SCENE = Path(__file__).resolve().parents[1] / 'configs/simulation/rm75-navigation.reference.json'


def transport_radius(robot, margin=.05):
    """Project every registered collision box, including hands, into base XY."""
    require(not robot.lft_oih_infos and not robot.rgt_oih_infos, 'Navigation requires empty grippers')
    radius = 0.
    for link in robot.cc.all_cdelements:
        for solid in link['collisionmodel'].cdnp.node().getSolids():
            require(hasattr(solid, 'getMin'), 'Transport envelope currently requires box collision primitives')
            corners = np.array(list(itertools.product(*zip(solid.getMin(), solid.getMax()))))
            world = corners @ link['gl_rotmat'].T + link['gl_pos']
            base = (world - robot.pos) @ robot.rotmat
            radius = max(radius, float(np.linalg.norm(base[:, :2], axis=1).max()))
    return radius + margin


def scene_grid(scene):
    if scene.get('schema') != 'omindos.community.navigation_scene.v1' or scene.get('frame_id') != 'map':
        raise ValueError('Expected map-frame navigation scene')
    grid = Grid(np.zeros((scene['height'], scene['width']), dtype=int), scene['resolution'], scene['origin'])
    obstacles = []
    yy, xx = np.indices(grid.data.shape)
    low_x, low_y = grid.origin[0] + xx*grid.resolution, grid.origin[1] + yy*grid.resolution
    for spec in scene['obstacles']:
        center = np.array(finite_vector(spec['center_m'], 3, 'obstacle center'))
        size = np.array(finite_vector(spec['size_m'], 3, 'obstacle size'))
        if np.any(size <= 0):
            raise ValueError('Obstacle dimensions must be positive')
        # Closed-cell overlap, not only cell centres, conservatively rasterizes boxes.
        mask = ((low_x <= center[0]+size[0]/2) & (low_x+grid.resolution >= center[0]-size[0]/2)
                & (low_y <= center[1]+size[1]/2) & (low_y+grid.resolution >= center[1]-size[1]/2))
        grid.data[mask] = 100
        obstacle = cm.gen_box(extent=size, rgba=[.7, .36, .17, 1])
        obstacle.set_pos(center)
        obstacles.append(obstacle)
    return Grid(grid.data, grid.resolution, grid.origin), obstacles


def map_obstacles(grid):
    """Extrude occupied AND unknown cells into conservative 2 m columns."""
    obstacles = []
    for y, row in enumerate(grid.blocked):
        x = 0
        while x < grid.width:
            if not row[x]:
                x += 1
                continue
            begin = x
            while x < grid.width and row[x]:
                x += 1
            obstacle = cm.gen_box(extent=[(x-begin)*grid.resolution, grid.resolution, 2.], rgba=[.65,.4,.2,1])
            obstacle.set_pos([grid.origin[0]+(x+begin)*grid.resolution/2,
                              grid.origin[1]+(y+.5)*grid.resolution, 1.])
            obstacles.append(obstacle)
    return obstacles


def set_pose(robot, pose):
    robot.set_base(np.array([pose[0], pose[1], 0.]), rm.rotmat_from_axangle([0, 0, 1], pose[2]))


def validate_base_motion(robot, trace, grid, obstacles):
    validate_trace(trace)
    transport = robot.get_jnt_values(robot.component).copy()
    require(abs(transport[0]-robot.waist_config.transport_rad) < 1e-9, 'Waist must be in transport pose')
    inflated = grid.inflate(transport_radius(robot))
    poses = [s['pose'] for s in trace['samples']]
    checked = 0
    for a, b in zip([poses[0]] + poses[:-1], poses):
        require(inflated.segment_free(a[:2], b[:2]), 'Navigation route has insufficient whole-robot map clearance')
        yaw = angle_delta(b[2], a[2])
        count = max(1, math.ceil(math.dist(a[:2], b[:2])/.025), math.ceil(abs(yaw)/.025))
        for t in np.linspace(0., 1., count+1)[1:]:
            pose = [a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), a[2]+t*yaw]
            set_pose(robot, pose)
            require(np.allclose(robot.get_jnt_values(robot.component), transport), 'Upper body moved during navigation')
            require(not robot.is_collided(obstacle_list=obstacles), 'Whole-robot collision during navigation')
            checked += 1
    return checked


def run_task(scene=None, navigation_trace=None, *, arm_configs=None, waist_config=None, target_positions=None):
    robot, gate = MobileDualPitch(config=waist_config, arm_configs=arm_configs), TaskGate()
    robot.set_waist_pitch(robot.waist_config.transport_rad)
    transport = robot.get_jnt_values(robot.component).copy()
    radius = transport_radius(robot)
    gate.start_navigation()
    try:
        if navigation_trace is None:
            grid, obstacles = scene_grid(scene)
            start = finite_vector(scene['start'], 3, 'start x,y,yaw')
            goal = finite_vector(scene['goal'], 3, 'goal x,y,yaw')
            points = grid.inflate(radius).plan(start[:2], goal[:2])
            trace = kinematic_trace(points, start[2], goal[2])
            trace['map'] = grid.to_dict()
        else:
            trace = validate_trace(navigation_trace)
            grid = Grid(**trace['map'])
            obstacles = map_obstacles(grid)
        checked = validate_base_motion(robot, trace, grid, obstacles)
        gate.navigation_result(success=True, stopped=True)
        gate.start_manipulation()
        base_pos, base_rot = robot.pos.copy(), robot.rotmat.copy()
        if target_positions is None:
            target = transport.copy()
            target[0] = .15
            target[1:8] += [.12, -.10, .08, .12, -.08, .10, .20]
            target[8:] += [-.12, -.10, -.08, .12, .08, .10, -.20]
        else:
            target = np.asarray(target_positions, dtype=float)
            robot.fk(robot.component, target)
            robot.fk(robot.component, transport)
        states = plan_path(robot, robot.component, transport, target, obstacles)
        require(np.allclose(robot.pos, base_pos) and np.allclose(robot.rotmat, base_rot), 'Base moved during manipulation')
        gate.manipulation_result(True)
    except Exception:
        gate.fail()
        raise
    plan = dict(schema='omindos.community.mobile_manipulation.v0.2', simulation_only=True,
                version='0.2', dof=15, base_coordinates=['x_m', 'y_m', 'yaw_rad'],
                joint_names=JOINT_NAMES, navigation=trace, transport_positions_rad=transport.tolist(),
                manipulation_positions_rad=[q.tolist() for q in states],
                waist_config=robot.waist_config.to_dict(), arm_config=arm_config_dict(robot.arm_configs),
                phase_history=gate.history, footprint_radius_m=radius,
                validated_base_samples=checked, validated_joint_samples=len(states))
    return robot, obstacles, plan


def main():
    cli = parser(__doc__)
    sources = cli.add_mutually_exclusive_group()
    sources.add_argument('--scene', type=Path, default=None)
    sources.add_argument('--navigation-log', type=Path, help='Actual Navigation v0.1 topic trace with occupancy map')
    cli.add_argument('--plan-output', type=Path)
    args = cli.parse_args()
    seed_rng(args.seed)
    # Delete stale result/plan before execution so a failed attempt cannot look successful.
    for path in (args.output, args.plan_output):
        if path:
            path.unlink(missing_ok=True)
    trace = json.loads(args.navigation_log.read_text()) if args.navigation_log else None
    scene = json.loads((args.scene or DEFAULT_SCENE).read_text()) if trace is None else None
    robot, obstacles, plan = run_task(scene, trace)
    if args.plan_output:
        args.plan_output.parent.mkdir(parents=True, exist_ok=True)
        args.plan_output.write_text(json.dumps(plan, indent=2, allow_nan=False)+'\n')
    report(args, '09-rm75-navigation', version='0.2', joints=15, arm_dof=[7, 7],
           navigation_backend=plan['navigation']['backend'], phase_history=plan['phase_history'],
           footprint_radius_m=plan['footprint_radius_m'],
           base_final_pose=plan['navigation']['samples'][-1]['pose'],
           validated_base_samples=plan['validated_base_samples'],
           validated_joint_samples=plan['validated_joint_samples'], ros_required=False,
           scope='Kinematics and sampled collision validation; uncalibrated chassis/mounts; no hardware execution')
    if not args.headless:
        poses = [s['pose'] for s in plan['navigation']['samples']]
        frames = [(p, plan['transport_positions_rad']) for p in poses]
        frames += [(poses[-1], q) for q in plan['manipulation_positions_rad']]
        def apply(frame):
            set_pose(robot, frame[0])
            robot.fk(robot.component, frame[1])
        import modeling.geometric_model as gm
        xy = np.array(poses)[:, :2]
        center = (xy.min(axis=0)+xy.max(axis=0))/2
        span = max(3., float(np.ptp(xy, axis=0).max()))
        route_points = [[p[0], p[1], .02] for p in poses]
        route = gm.gen_linesegs(list(zip(route_points, route_points[1:])), thickness=.0008, rgba=[.1,.65,.8,1])
        markers = [gm.gen_sphere(pos=np.array([p[0], p[1], .04]), radius=.08, rgba=color)
                   for p, color in ((poses[0], [.1,.7,.2,1]), (poses[-1], [.9,.2,.2,1]))]
        show_robot(robot, frames, apply, obstacles+[route]+markers, lookat=(*center, .7),
                   cam_pos=[center[0]+span, center[1]-1.35*span, span+.7])


if __name__ == '__main__':
    main()
