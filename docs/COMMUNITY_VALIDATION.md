# Community v0.1 verification

Recorded 2026-10-06 (Asia/Shanghai). Starting branch commit:
`30ce37feb53c2ff5e77faeeccd2d3e95b0accc8a`.

Reference host: Ubuntu 24.04, CPython 3.12.14, Linux x86_64, fresh `.venv` with
Community pins. Commands: `python -m community.verify`,
`python -m unittest community.test_contract`, `python -m pip check`.

| Check | Measured result |
|---|---|
| Quick Start | Pass; position error 0.00112234 m, rotation-matrix error 0.00099629 |
| UR3e single arm | Pass; position error 0.00090410 m, rotation-matrix error 0.00244977; 7 active collision links |
| Grasping | Pass; 6 collision-filtered box grasp candidates |
| RRT-Connect | Pass; 283 edge samples, valid endpoints/limits, blocking-obstacle positive control |
| Fixed dual arm | Pass; 5 checked 12-joint configurations |
| Mobile dual arm | Pass; 31 base transforms, consistent TCP transform, `ready` gate |
| Integrated task | Pass; arm paths with 389 and 402 checked samples, 2 released objects, successful task state |
| Contract regression checks | 5 passed: state order, failure/stopped-base gate, cancellation, invalid base goal and invalid trajectory |
| Dependency integrity | `pip check` reports no broken requirements |
| Geometry rendering | Panda3D TinyOffscreen rendered the full dual-arm mesh successfully; image visually inspected |
| Desktop window/input | Not tested: this host has no desktop display |
| ROS2 action transport, same-host IPC | Pass; 3/3 actual Action checks on RoboStack Humble / Python 3.14.7 / Fast DDS 2.6.11, with shared-memory-only transport |
| Independent simulation CI | Pass; Ubuntu 24.04 / Python 3.12, 7/7 demos and 5 contract tests |
| Standard localhost ROS2 DDS/action CI | Pass; Ubuntu 22.04 / ROS2 Humble container, 3/3 Action cases and 5 contract tests |
| Cross-machine ROS2 networking | Not tested |
| Physical hardware / C++ Navigation stack | Not tested |

Machine-readable results: [Linux report](validation/linux-py312.json).
Observed package versions: [environment snapshot](validation/pip-freeze-linux-py312.txt).
ROS2 results: [three Action cases](validation/ros2-humble-shm.json).
ROS2 package snapshot: [RoboStack environment](validation/ros2-humble-packages.json).
The CI workflow preserves per-demo logs and JSON plus ROS2 results as build artifacts.

## Independent GitHub CI

[Run 37375913047](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/actions/runs/37375913047)
completed successfully on 2026-10-05 at 21:29 UTC for source commit
`44c3e9c85861ada88b0892c93888a6ca760330f5`. Both jobs passed:

- `simulation`: installs Community dependencies, runs the five contract tests and
  all seven headless demos, and exports the integrated playback plan.
- `ros2-transport`: consumes that plan in `ros:humble-ros-base` on Ubuntu 22.04,
  repeats the five contract tests and passes all three separate-process Action cases
  with the default verifier's localhost DDS configuration.

Artifacts are `community-simulation` and `community-ros2`. Run, job and artifact
identifiers and digests are recorded in [the CI evidence](validation/github-actions.json).
The release-status documentation update follows this tested source commit and
does not change executable code, dependencies or workflow configuration.

This confirms standard same-host discovery and Action communication with mock
servers. The CI run does not determine whether each DDS payload used shared memory
or UDP, and does not exercise cross-machine networking or the actual C++ Navigation stack.

## ROS2 follow-up

The normal local-only UDP discovery attempt failed with `getifaddrs: Operation not
permitted` before the client could discover the server. An isolated RoboStack Humble
environment was then tested with Fast DDS's explicit shared-memory-only XML profile:
no UDP/TCP transport is configured. Three separate-process tests passed:

1. Navigation succeeds, a new stopped-base observation arrives, and the arm action succeeds.
2. Navigation aborts and the task fails without starting manipulation.
3. Manipulation aborts and the task reports failure.

This verifies real ROS2 serialization, discovery, actions, feedback and results over
same-host IPC against mock capability servers. It does not validate UDP networking,
physical devices, or the actual C++ Navigation stack. The shared-memory JSON report
preserves the observation made before CI completed; its queued-CI note is historical.
The successful independent system-ROS2 CI run is recorded separately above.

## Issues found and handled

- Direct Quick Start execution did not put the repository root on the import path.
  The entry point now resolves its own source root; module entry points are preferred.
- Missing optional QPanda3D blocked every geometry import. Qt embedding imports are lazy.
- Removed NumPy aliases in the vendored `basis/trimesh` layer are replaced with their
  equivalent builtin types; integer RRT node-ID bounds work on Python 3.12.
- The original UR3E constructor creates an empty collision checker. The Community
  single-arm adapter explicitly calls the existing UR3E collision setup, and a
  deliberately blocking box confirms detection is active.
- The historical RRT smoother changed the seed-7 terminal configuration. Community
  disables smoothing and validates every returned edge; original algorithms are retained.
- The historical mobile `move_to` uses a removed pose method. The Community adapter
  relocates existing kinematic chains and calls original FK to update grippers and objects.

## Interpretation

Passing these checks demonstrates the stated simulation exercises on the recorded
host, not hardware readiness or continuous collision safety. The original planner
is probabilistic; other seeds/scenes may fail. A failure remains a nonzero exit,
not an automatically accepted motion. The integrated task is a geometric transfer
with known object poses and abstract release, not a contact-physics pick-and-place benchmark.


## Standalone 15-DoF update (2026-10-07)

The default `mobile_dual_pitch` now uses independent seven-axis arms plus one pitch
waist. It runs directly in Python without ROS. Its local verification on Linux
x86_64 / CPython 3.12.14 / NumPy 1.26.4 / Panda3D 1.10.15 passed:

- All eight default headless demos; the new demo validates 219 sampled 15-joint states with seed 7.
- The 20 existing contract/API/legacy-waist tests, plus nine new 15-DoF regressions.
- Independent matrix FK for both arms and the waist, all fourteen arm joints affecting TCP,
  seven-axis IK, fifteen-dimensional planning, atomic dimension/limit rejection,
  seventh-link obstacle detection, arm-arm/self collision, held-object transforms and repeated release.
- The explicitly named `legacy_mobile_dual_pitch` still produces a valid 13-joint fixture.
- Offscreen rendering of the new geometry, visually inspected; [reference image](images/mobile-dual-15dof.png).

Desktop window input was not exercised on this headless host. The new geometry is
an uncalibrated reference chain, not the physical robot's verified model. No dynamics,
ROS integration for the 15-joint model, or hardware motion was tested or added.
The original 12/13-joint ROS transport remains separate and unchanged.

## Manufacturer RM75-B follow-up (2026-10-07)

The default arms now load the pinned official RM75-B URDF and eight STL link meshes.
The previous generic seven-axis configuration remains available explicitly.

- All 37 contract/API/legacy-waist/generic-seven-axis/RM75 tests passed locally.
- All eight headless demos passed. The RM75 15-joint demo with seed 7 checked 568
  joint-space samples, including limits and conservative collision boxes.
- Independent official modified-DH FK agreed for 40 random arm poses with base,
  waist and mount rotations (position tolerance 10 micrometres; matrix-entry
  tolerance 3e-5, allowing the source URDF's rounded fixed angles).
- All eight collision links on each arm, including the fixed base and J7 link,
  detected deliberately inserted obstacles. Cross-arm and folded-arm positive
  controls, seven-axis IK, model copying, held-object motion and release passed.
- Compressed meshes were checked against SHA256 hashes of the original upstream
  STL bytes. No runtime model download or ROS dependency was introduced.
- [Actual offscreen render](images/mobile-dual-rm75.png) visually inspected.

Only RM75-B standard flange is implemented. Whole-robot mounting transforms,
waist range, body geometry and gripper TCP remain uncalibrated reference values.
This does not validate a physical robot, dynamics, precise mesh contact or desktop input.
The manufacturer data and independent DH source are documented in the
[asset provenance](../community/models/realman_rm75/NOTICE.md).

## 0.2 chassis navigation and RM75 integration (2026-10-07)

Local CPython 3.12.14 / NumPy 1.26.4 / Panda3D 1.10.15:

- 51 regression tests, including 14 new navigation tests, passed.
- The default standalone route uses a 0.790582 m whole-robot transport radius,
  validates 714 base samples, then 568 upper-body joint samples with seed 7.
- Negative tests cover unknown cells, no route, narrow passages, grid corners,
  missing/wrong post-goal paths, stale observations, nonzero velocity on every
  Twist axis, unstable heading, discontinuous traces and map clearance rejection.
- An actual Navigation v0.1 capture (44 observations) passed local full-robot
  replay: 102 resampled base poses and 568 joint poses. The final estimated yaw
  was retained. The initial CI attempt exposed and prompted a fix for an
  unbounded simulator shutdown wait; release still requires a fully passing run.
- Failed navigation is checked not to call the arm planner. The base transform
  moves the entire assembly rigidly and stays fixed during manipulation.

Release gates in `community.yml` also require all nine demos, API examples,
legacy ROS2 Action checks, actual Navigation v0.1 topic capture from its pinned
release image, and full-robot replay of that captured trace. The validation ZIP
on the 0.2 release contains the exact workflow's reports and samples. A missing
or failed gate prevents automatic publication; synthetic traces are not used as
substitutes for the runtime integration gate.

This verifies simulation kinematics and sampled conservative collision checks.
It does not certify actual chassis dimensions, terminal yaw control, physical
stopping, dynamic feasibility, calibration or hardware safety.
