# OmindOS Manipulation 0.2

Simulation examples for learning UR3e kinematics, grasping, collision checking,
RRT-Connect, fixed dual arms and mobile manipulation task composition.

**Version 0.2:** standalone chassis navigation, dual RM75-B arms and a pitch waist.
Run `python -m community.rm75_navigation` for navigation, verified stopping, and
15-joint manipulation. ROS is optional. The historical v0.1 release stays immutable.

[Customer guide (Chinese)](docs/CUSTOMER_GUIDE.zh-CN.md) · [Chinese overview](README.md) · [Product](docs/PRODUCT_MANUAL.zh-CN.md) ·
[Install](docs/INSTALL.zh-CN.md) · [API](docs/API.zh-CN.md) · [Changes](CHANGELOG.md)

The seven demos and the ROS2 mock transport were checked in the original v0.1
release. See [historical results](docs/COMMUNITY_VALIDATION.md) and
[current CI](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/actions/workflows/community.yml).
Physical robots, contact dynamics and production deployment remain outside the
verified scope.

## Customer runtime and development checkout

The full checkout is the development edition. The customer release contains a
frozen runtime plus editable URDF assets/configuration, a standard-library Python
SDK and examples. New APIs accept requested 15-joint targets and chassis scenes.
See [customer release guide](docs/CUSTOMER_RELEASE.zh-CN.md) and
[build and independent verification](docs/BUILD_CUSTOMER_RELEASE.md).
A candidate is not a verified release until its standalone check passes.

## Development setup

```bash
git clone --branch main --single-branch https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community.git
cd OmindOS-Manipulation-Community
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-community.txt
python -m community.verify
python -m unittest community.test_contract community.test_api community.test_pitch community.test_dual7 community.test_rm75 community.test_navigation
python -m community.api_example
```

The verified reference environment is Ubuntu 24.04, CPython 3.12, NumPy 1.26.4
and Panda3D 1.10.15. `requirements-community.txt` is for these demos;
`requirements.txt` preserves the historical application environment.
Windows and macOS have not been verified. See [environment notes](docs/ENVIRONMENT.md).

To open a visualization window on a desktop:

```bash
python -m community.ur3e_single_arm
```

Add `--headless` to compute and check results without a display. Each demo supports
`--seed 7` and `--output community-results/result.json`. A failure exits nonzero.
Run the module commands from the repository root; the original
`python community/quick_start.py --headless` entry point also remains available.

## Community Demos

| Demo | Command after `python -m community.` | What it actually checks | Offline status |
|---|---|---|---|
| 01 — Quick Start | `quick_start --headless` | Joint chain, target → IK → FK residual | Verified |
| 02 — UR3e Single Arm | `ur3e_single_arm --headless` | UR3e meshes, six joints, TCP, IK and self collision | Verified |
| 03 — Grasping | `grasping --headless` | Robotiq85 grasp candidates on a generated box, mesh collision filtering | Verified |
| 04 — RRT-Connect | `motion_planning --headless` | Original planner, obstacle detection, endpoints, joint limits and densely sampled edges | Verified |
| 05 — Fixed Dual Arm | `dual_arm --headless` | Twelve-joint coordinated motion and original dual-arm collision model | Verified |
| 06 — Mobile Dual Arm | `mobile_dual_arm --headless` | Simulated base route, rigid transforms, collision checks, navigation result gate | Verified; mock navigation |
| 07 — Integrated Task | `integrated_task --headless` | Simulated navigation, geometric grasps, two sequential arm plans, attached objects, release | Verified; mock navigation |
| 08 — Dual RM75 + pitch | `mobile_dual_pitch --headless` | Manufacturer arm kinematics, 15-joint planning and collision checks | Offline |
| 09 — Navigation + RM75 | `rm75_navigation --headless` | Chassis route, complete transport footprint, stop gate and subsequent 15-joint plan | Offline + optional real runtime trace |

The mobile example relocates the existing dual-arm rig as a kinematic assembly;
it does not add a wheel model, localization, or a mobile-base dynamics engine.
The integrated task begins with boxes at known reachable grasp poses. It does not
claim perception, approach planning, contact-force simulation, or a dual-arm handover.

Follow the [demo walkthrough](community/README.md) and [learning path](docs/LEARNING_PATH.md).

## Navigation integration

The default `rm75_navigation` demo uses an independent grid A* planner, a whole-robot
transport envelope and planar kinematics without ROS. It exports base coordinates
separately from the 15 upper-body joints. Unknown occupancy cells are blocked.

The optional adapter runs the actual published `omindos/navigation:0.1` simulator:
`/goal_pose`, `/plan`, `/localization/pose`, `/cmd_vel` and `/map`. This release uses
Python `nav3d_native` nodes, not a Nav2 Action server. After matching a post-goal path
and observing fresh, stable stopping evidence, its JSON trace can be replayed through
full-robot collision validation before arm planning. Actual terminal yaw is retained.
See the [0.2 integration guide](docs/NAVIGATION_v0.2.zh-CN.md) for Docker commands.

The original 12/13-joint ROS2 Action bridge remains an isolated mock transport example;
it is not the interface to this Navigation release. Neither adapter executes hardware.
CI separately checks the real Navigation image (pinned SHA256), topic capture,
RM75 trace replay, all nine offline demos and the legacy mock Action transport.

## Repository map

```text
community/          Public planning API, demos, validation and ROS2 mock bridge
basis/              Original mathematical and geometry utilities
robot_sim/          Original robot, joint-chain and gripper models
modeling/           Geometric and collision models
grasping/           Grasp annotation and planning
motion/             Current motion/planning modules used by Community demos
motionplanning/     Historical motion-planning modules
visualization/      Panda3D visualization
0000_examples/      Historical examples
0000_book/          Historical teaching examples
app/                Integrated research/hardware applications
docs/               Installation, learning, interface and validation notes
```

Historical examples are retained and are **not all verified**. Community does not
rename the upstream source tree or import hardware drivers. The original research
IK, geometry and RRT algorithms remain in place; small Python/NumPy compatibility
edits are itemized in [the release notes](docs/RELEASE_v0.1.md).

## Real robots

Start in simulation. This release is not a physical-robot safety controller.
Hardware integration needs its own driver mapping, joint limits, tool/payload
calibration, collision validation and manufacturer-required safety checks.

## Project origin and attribution

This repository is derived from the **Universal Robots UR3e Planning & Control System**
and originates from work based on / inspired by the **WRS** robotics software developed
by **Weiwei Wan and collaborators at Osaka University**.

The original repository license, source author comments and attribution are preserved.
Please retain the applicable notices when redistributing derived work. Original WRS
and UR3e work is not reattributed to OriginMind.

## License and contributing

This repository is distributed under the **MIT License**. See [LICENSE](LICENSE):

> Copyright (c) 2020 Weiwei Wan

Community documentation and later contributions do not remove or replace this notice.
OmindOS Community additions are maintained by OriginMind under the same MIT license.

Contributions should include a reproducible command, environment details and the
expected output. Run `python -m community.verify` and
`python -m unittest community.test_contract community.test_api community.test_pitch community.test_dual7 community.test_rm75 community.test_navigation` before proposing changes to a demo.

## Standalone 15-DoF mobile dual arms

`python -m community.mobile_dual_pitch` opens the 15-joint kinematic animation:
seven joints per arm plus one pitch waist. Add `--headless` for offline verification.
**ROS and Gazebo are not required.** Both seventh joints are active revolute joints,
with FK, single-arm IK, joint-space planning and collision checking.

The default arms now use the manufacturer's RM75-B URDF joint origins, axes, limits
and link meshes, bundled under Apache-2.0. Shoulder mounts, waist, body collision boxes
and grippers remain uncalibrated reference parameters. Only the standard RM75-B flange
is enabled; other variants must be checked against their corresponding model. The fixed chassis does not
pitch with the upper body. Wheel motion and gripper opening are outside the 15 DoFs.
See the [configuration and Python interface guide](docs/MOBILE_DUAL_PITCH.zh-CN.md).

Original UR3e single-arm and twelve-joint demos remain available. The old thirteen-joint
waist demo has moved to `community.legacy_mobile_dual_pitch`; the existing ROS2 bridge
retains its original 12/13-joint protocols. New 15-joint exports are untimed offline
kinematic samples, not ROS or hardware controller trajectories.
