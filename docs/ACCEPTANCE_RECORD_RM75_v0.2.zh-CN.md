# OmindOS Manipulation 0.2 — RM75-B 客户验收记录

> 历史记录说明：本页指向旧 Community 的 Release、tag 或 CI 链接用于追溯，私有化后需要旧仓权限；不作为客户安装依赖。当前客户入口为[公开发行仓](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release)，参见[迁移说明](REPOSITORY_MIGRATION.zh-CN.md)。

模板版本：v0.2；软件基线：`v0.2` / `6726ddfdf84bf9eb44a36671566f5199856cbe32`。
目标模型：左 RM75-B 7 轴＋右 RM75-B 7 轴＋pitch 腰部 1 轴；底盘 `x/y/yaw` 单独记录。

此文件是待填写模板，不代表客户或目标机器人已通过验收。复制后填写项目结果，保留原模板与历史记录。
每项结果填写“通过 / 未通过 / 未测试 / 不适用”；“不适用”必须说明范围与原因。
证据栏中的路径是核对位置或建议保存位置，不表示文件已经在客户环境生成。
发布端已存在的证据仅列于附录，不得直接填入客户结果栏。

## 1. 验收层级与范围

| 层级 | 本次包含与否 | 可以证明什么 | 不能据此认定什么 | 本次结论 |
|---|---|---|---|---|
| A：离线软件验证 | 待填写 | 固定源码、模型、九项示例、51 项回归、采样碰撞及规划 | 目标机已连接、实际运动或安全验收 | 待验收 |
| B：Navigation 实际消息采集与离线回放 | 待填写 | 发布版导航节点在隔离仿真中产生的 Topic、到达证据、整机回放 | 实车导航、实测停车、硬件互锁有效 | 待验收 |
| C：客户现场只读连接 | 待填写 | 本项目真实设备身份、反馈、坐标和配置核对 | 已执行动作或具备安全运行条件 | 未测试 |
| D：真机运动执行与现场验收 | 待填写 | 单独适配后的实际运动、性能和现场停止行为 | 不能由 A/B/C 自动推定 | 未测试 |

B 的“实际消息”指 `omindos/navigation:0.1` 中实际运行的 `nav3d_native` Python 节点，仍是仿真；
示例建图使用模拟真值。v0.2 不含 RM75、腰部、底盘和夹爪的真机运动执行适配。
到达停稳门禁只允许进入**离线机械臂规划**，不是急停、制动或安全认证。

## 2. 项目、版本与模型资料

| 项目 | 本次记录 |
|---|---|
| 客户 / 项目 / 记录编号 | 待填写 |
| 日期、时区、地点；操作人 / 复核人 | 待填写 |
| 源码来源、实际完整提交号、工作区改动与补丁 | 待填写；不得仅写 main |
| 验收模板版本 / 模板所在提交号 | 待填写；模板修订提交与软件发布提交分别记录 |
| OS、CPU 架构、Python、依赖清单及 `pip check` | 待填写 |
| 可选 Docker、Navigation 镜像、运行时压缩包 SHA256 | 待填写 |
| 左 / 右臂型号、标准版或其他末端版本、序列号 | 待填写；默认模型仅 RM75-B 标准版 |
| 左 / 右控制器与固件、SDK / 驱动及连接方式 | 待填写；未连接则记未测试 |
| 腰部执行器、轴线、零位、限位、驱动 / 制动配置 | 待填写 |
| 底盘型号、驱动方式、外形、坐标系与定位来源 | 待填写；仿真不声明实际轮型或动力学 |
| 左 / 右夹爪、工具 TCP、开口、负载及重心 | 待填写；参考夹爪不是实机标定 |
| 双臂安装变换、腰轴 / 底盘 / 工具标定文件与日期 | 待填写 |
| 场景 / 地图 / 工装 / 障碍物版本与 SHA256 | 待填写 |
| 证据根目录、运行命令 / 参数、种子、退出码 | 待填写；同次运行使用独立目录，保留失败日志 |

## 3. 证据路径索引

下表左列是按发布工作流在仓库根目录运行时的默认路径；右列是已发布
`OmindOS-Manipulation-0.2-validation.zip` 解压后的真实路径。二者不能混用。
`<demo>` 使用第 6 节模块名；`<本次证据目录>` 由项目填写，不是仓库内置目录。

| 证据 | 本次复验默认输出 / 建议保存位置 | 发布 ZIP 内路径 |
|---|---|---|
| 九项示例汇总 | `community-results/summary.json` | `release-artifacts/community-simulation/summary.json` |
| 每项示例与日志 | `community-results/<demo>.json`、`<demo>.log` | `release-artifacts/community-simulation/<demo>.json`、`<demo>.log` |
| 固定底盘十五关节采样 | `community-results/kinematics-15dof-playback.json` | `release-artifacts/community-simulation/kinematics-15dof-playback.json` |
| 独立 A* 导航联动采样 | `community-results/mobile-manipulation-0.2.json` | `release-artifacts/community-simulation/mobile-manipulation-0.2.json` |
| Python API / CLI | `community-results/api-example.json`、`api-cli.json` | `release-artifacts/community-simulation/api-example.json`、`api-cli.json` |
| 51 项回归 | `<本次证据目录>/regression.log`（自行保存） | ZIP 无此日志；见发布工作流 `simulation` 的 unittest 步骤 |
| Navigation Topic 采集 | `community-results/navigation/navigation-trace.json`、`navigation-runtime.log` | `release-artifacts/navigation-runtime/navigation-trace.json`、`navigation-runtime.log` |
| 导入轨迹后的结果 / 规划 | `community-results/navigation-integration.json`、`navigation-integration-plan.json` | `release-artifacts/mobile-integration/navigation-integration.json`、`navigation-integration-plan.json` |
| legacy 12/13 关节传输 | `community-results/ros2/`、`community-results/ros2-waist/` | `release-artifacts/community-ros2/ros2/`、`release-artifacts/community-ros2/ros2-waist/` |
| 现场连接 / 真机执行 | `<本次证据目录>/site/`、`<本次证据目录>/motion/` | 无；未做现场验收 |

软件验收以固定提交的[发布工作流](../.github/workflows/community.yml)、[发布说明](RELEASE_v0.2.md)、
[导航指南](NAVIGATION_v0.2.zh-CN.md)和实际 JSON 一起核对。
`docs/validation/rm75-linux-py312.json` 等旧报告保留其原有范围；其中八项示例记录不能替代 v0.2 的九项证据。

## 4. 复验命令与日志

按[安装指南](INSTALL.zh-CN.md)准备 Ubuntu 24.04 x86_64 / Python 3.12 环境，
从固定源码根目录运行。先归档已有 `community-results/`；不要用上一次成功文件代替本次输出。
以下 Bash 命令保存退出状态；任一步失败即停止。它们不连接或驱动真实机器人。

```bash
set -euo pipefail
mkdir -p community-results/acceptance
git rev-parse HEAD > community-results/acceptance/source-commit.txt
git status --short > community-results/acceptance/worktree.txt
python --version > community-results/acceptance/python-version.txt
python -m pip freeze > community-results/acceptance/pip-freeze.txt
python -m pip check 2>&1 | tee community-results/acceptance/pip-check.log
python -m unittest -v community.test_contract community.test_api community.test_pitch \
  community.test_dual7 community.test_rm75 community.test_navigation \
  2>&1 | tee community-results/acceptance/regression.log
python -m community.verify 2>&1 | tee community-results/acceptance/verify.log
python -m community.api_example 2>&1 | tee community-results/acceptance/api-example.log
python -m community.api quick_start --output community-results/api-cli.json \
  2>&1 | tee community-results/acceptance/api-cli.log
```

这里的 `-v` 只增加逐测试名称日志，测试集合与发布工作流相同。
记录每条命令的实际退出码；失败时保留日志并填写“未通过”，未执行的后续步骤填“未测试”。
环境信息和本次源码若与发布不同，记录差异，不能仍声称复现固定发布环境。

B 层按[导航指南的镜像加载与隔离采集命令](NAVIGATION_v0.2.zh-CN.md#使用-navigation-仓库的实际发布版)操作，
镜像压缩包核对值为 `9a2fbed8baeb53b33586c5f76674dead3a6b29d3973fe508f4928e3928f9f491`。
采集使用 `--network none` 的模拟容器，不将该脚本连接客户机器人。随后离线导入：

```bash
python -m community.rm75_navigation --headless \
  --navigation-log community-results/navigation/navigation-trace.json \
  --output community-results/navigation-integration.json \
  --plan-output community-results/navigation-integration-plan.json \
  2>&1 | tee community-results/acceptance/navigation-integration.log
```

只回放发布 ZIP 中的已有 trace 时，应将输入改为其解压路径，填写“发布记录离线回放”，
不能填写“本次重新采集”或“现场导航”。保存输入 trace 的 SHA256 与输出，区分各次运行。

## 5. 51 项回归与环境检查（A）

51 是 v0.2 固定提交中六个模块的测试方法总数，不是 51 次真机实验，也不等于九项示例数量。
核对 `Ran 51 tests`、`OK`、进程退出码 0 和各方法结果；不能以跳过测试凑成通过。
新版源码增减测试时记录实际数量和差异，不改写历史 51 项记录。

| 模块 / 检查 | v0.2 数量 | 覆盖范围与核对依据 | 本次结果 | 本次证据路径 |
|---|---:|---|---|---|
| 依赖与环境 | — | `pip check` 无冲突；版本及依赖与记录相符 | 未测试 | 待填写：`acceptance/pip-check.log`、环境记录 |
| `community.test_contract` | 7 | 阶段顺序、失败 / 取消、坐标及旧轨迹协议；含 legacy 13 关节约束 | 未测试 | 待填写：`acceptance/regression.log` |
| `community.test_api` | 8 | 六轴 UR3e API 的端点、避障、限位、抓取、参数、取消、超时、关闭；reference | 未测试 | 待填写：同上 |
| `community.test_pitch` | 5 | 腰轴变换、附着物、身体碰撞、运输姿态与限位、复制；参考模型 | 未测试 | 待填写：同上 |
| `community.test_dual7` | 9 | 通用七轴 FK、各关节作用、维度 / 限位、碰撞、附着物、复制 / IK / 规划及配置；reference | 未测试 | 待填写：同上 |
| `community.test_rm75` | 8 | RM75-B 官方 MDH 交叉核验、资产完整性、每轴 TCP 作用、限位、IK / 规划、互撞 / 自碰撞、附着物、机型配置 | 未测试 | 待填写：同上 |
| `community.test_navigation` | 14 | 到达 / 六分量速度 / 连续停稳、数据拒绝、地图净空、整机运输与规划门禁；见第 8 节 | 未测试 | 待填写：同上 |
| 总数与执行结果 | 51 | 7＋8＋5＋9＋8＋14；无失败、错误或跳过，退出码 0 | 未测试 | 待填写：完整日志及退出码 |
| Python API 示例 / CLI | — | `api-example.json` 的 `motion.status`、`grasp.status` 及 `api-cli.json.status` 为 `succeeded`；均为模拟 | 未测试 | 待填写：第 3 节 API 文件 |

计数依据为各 `community/test_*.py` 中 unittest 测试方法；循环内的多次输入不单独计数。
API 的 `plan_joint_motion` 仍是六轴 UR3e，不接受 RM75 十五关节目标；十五关节规划见模型接口与第 7 节。

## 6. 九项无窗口示例（A）

`community.verify` 的 `summary.json.status` 及九个 `results[].status` 应为 `passed`；
逐一核对 `demo` 身份、`mode: simulation`、对应 JSON / 日志与退出码，`hardware` 应仍为 `not tested`。
下列附加字段用于识别能力范围，不将发布时的随机规划采样数量作为通用性能阈值。

| 模块名 / JSON 文件名 | `demo` 标识 | 检查项及范围 | 本次结果 | 本次证据路径 |
|---|---|---|---|---|
| `quick_start` | `01-quick-start` | FK / IK 误差字段；UR3e reference | 未测试 | 待填写：`community-results/quick_start.json`、`.log` |
| `ur3e_single_arm` | `02-ur3e` | `joints_rad` 六维、误差与 `collision_links`；legacy UR3e | 未测试 | 待填写：`community-results/ur3e_single_arm.json`、`.log` |
| `grasping` | `03-grasping` | `candidates`、`jaw_width_m`；Robotiq85 几何候选 reference，无接触力验证 | 未测试 | 待填写：`community-results/grasping.json`、`.log` |
| `motion_planning` | `04-rrt-connect` | `validated_samples`、`edge_step_rad: 0.025`、`blocking_obstacle_detected: true`；UR3e reference | 未测试 | 待填写：`community-results/motion_planning.json`、`.log` |
| `dual_arm` | `05-fixed-dual-arm` | `joints: 12` 与采样检查；legacy 双 UR3e | 未测试 | 待填写：`community-results/dual_arm.json`、`.log` |
| `mobile_dual_arm` | `06-mobile-dual-arm` | `navigation: mock`、`transport: in-process`、`task_history`；legacy 底盘参考 | 未测试 | 待填写：`community-results/mobile_dual_arm.json`、`.log` |
| `integrated_task` | `07-integrated-task` | `navigation: mock`、`arm_path_samples`、`objects_released`、阶段历史；legacy 几何搬运 | 未测试 | 待填写：`community-results/integrated_task.json`、`.log`、`ros2-playback.json` |
| `mobile_dual_pitch` | `08-mobile-dual-pitch` | `joints: 15`、`arm_dof: [7,7]`、`arm_models: ["RM75-B","RM75-B"]`；RM75 目标模型 | 未测试 | 待填写：`community-results/mobile_dual_pitch.json`、`.log`、`kinematics-15dof-playback.json` |
| `rm75_navigation` | `09-rm75-navigation` | `joints: 15`、`navigation_backend: standalone_astar`、阶段与采样；RM75 离线联动 | 未测试 | 待填写：`community-results/rm75_navigation.json`、`.log`、`mobile-manipulation-0.2.json` |

九项共同构成发行软件检查；前七项通过不能作为 RM75 模型、连接或真机通过的替代依据。
示例 09 默认 A* 输出与 B 层 Navigation Topic 导入输出应分别保存、分别验收。

## 7. RM75-B 模型、限位、整机碰撞与规划（A/B）

角度用 rad、长度用 m。`joint_names` 必须依次为 `waist_pitch_joint`、`left_joint_1` 至
`left_joint_7`、`right_joint_1` 至 `right_joint_7`，每个关节采样长度为 15。
底盘、车轮与夹爪开合不插入该数组。

| 部位 | v0.2 默认位置限位 | 来源与现场状态 |
|---|---|---|
| 左右臂 J1 / J3 / J5 | [-3.106, 3.106] | [厂家 URDF](../community/models/realman_rm75/RM75-B.urdf)；现场匹配待核对 |
| 左右臂 J2 | [-2.2689, 2.2689] | 同上 |
| 左右臂 J4 | [-2.356, 2.356] | 同上 |
| 左右臂 J6 | [-2.234, 2.234] | 同上 |
| 左右臂 J7 | [-6.28, 6.28] | 同上；不扩写成无限旋转 |
| 腰 pitch | [-0.35, 0.35]；运输位 0 | [参考腰配置](../configs/simulation/rm75-waist.reference.json)；绕底盘 +Y，未实机标定 |

模型上游提交 `b63dddb20b7620d14cfff32678de60bb3a201ebc`，来源与许可证见
[NOTICE](../community/models/realman_rm75/NOTICE.md)。限位不等于速度、力矩或现场可运行范围。

| 检查项 | 通过依据 / 必须记录的内容 | 本次结果 | 本次证据路径 |
|---|---|---|---|
| 模型身份与资产 | 两侧 `arm_config.*.model` 均为 RM75-B；URDF 及解压 STL 对照 `mesh-sha256.json`；未误用通用七轴或其他末端版 | 未测试 | 待填写：配置、模型摘要、`test_rm75` 日志 |
| 十五关节顺序、维度与限位 | 所有采样为有限数、顺序正确、逐轴在上述限位；越限 / 错长度 / 非有限值在修改姿态前被拒绝 | 未测试 | 待填写：两类 plan JSON、回归日志 |
| 正运动学与第七轴 | RM75 MDH 独立核对左右共 40 个随机姿态；零位法兰轴向距离 0.8505 m；各轴含 J7 改变本臂 TCP | 未测试 | 待填写：`test_rm75` 日志；不将标称工作半径 0.610 m 当该轴向距离 |
| 安装、腰部与工具配置 | 核对 `arm_config`、`waist_config`；参考底盘 / 肩部 / 腰部 / RobotiqHE 夹爪与实机标定分开记录 | 未测试 | 待填写：输入配置、输出配置及现场差异 |
| 整机碰撞覆盖 | 两侧固定底座及七段连杆、夹爪、躯干、底盘、外部障碍；含互臂、非相邻自碰撞与附着物回归 | 未测试 | 待填写：`test_rm75`、`test_dual7`、`test_pitch` 日志及场景 |
| 固定底盘十五关节规划 | `kinematics-15dof-playback.json` 的 `schema: omindos.community.kinematics.dual7_pitch.v1`、`simulation_only: true`、`dof: 15`、`positions_rad`；起终点及 `base_position_m` / `base_rotation` 相符 | 未测试 | 待填写：示例 08 结果 / plan / 日志 |
| 导航后规划 | `schema: omindos.community.mobile_manipulation.v0.2`、`version: "0.2"`、`simulation_only: true`、`dof: 15`；`transport_positions_rad` 为起点，`manipulation_positions_rad` 为规划采样 | 未测试 | 待填写：独立联动和导入联动两份 plan |
| 规划端点与底盘固定 | 默认目标腰 0.15；左臂相对运输位增量 [0.12,-0.10,0.08,0.12,-0.08,0.10,0.20]，右臂 [-0.12,-0.10,-0.08,0.12,0.08,0.10,-0.20]；规划中底盘固定在导航最终估计姿态 | 未测试 | 待填写：实际参数、plan 首末点、回归 / 执行日志 |
| 采样质量 | 关节段最大增量 0.025 rad；端点、限位、逐样本碰撞检查通过；`validated_joint_samples` 与采样条数相符且非空 | 未测试 | 待填写：结果与 plan、碰撞检查日志 |

碰撞几何为 RM75 STL 的局部包围盒外扩 1 mm；相邻连杆、底座与同肩 J1/J2、
底座与相连支架 / 躯干等安装配对被排除，详见[模型指南](MOBILE_DUAL_PITCH.zh-CN.md)。
这是几何加密采样，不是连续碰撞证明、接触力学、抗倾覆或硬件安全验证。
输出没有控制时间参数、关节速度或加速度，不能直接作为电机 / ROS2 执行轨迹。
`status` 在结果 JSON 中；plan JSON 不提供独立 `status` 字段，应与同次结果和日志配对核对。

## 8. Navigation 轨迹、到达停稳与整机净空（B）

| 接口 | 实际消息类型 | 证据用途 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | 单次目标 XY，map 坐标 |
| `/plan` | `nav_msgs/Path` | 目标发布后路径及终点匹配 |
| `/localization/pose` | `geometry_msgs/PoseStamped` | 估计位姿与朝向 |
| `/cmd_vel` | `geometry_msgs/Twist` | 六分量命令近零；不是实测轮速或机身速度 |
| `/map` | `nav_msgs/OccupancyGrid` | 地图与整机净空回放 |

`/plan` 和 `/map` 的 QoS 为 reliable / transient_local / depth 1。
真实消息采集链路使用上述 Topic；旧 Nav2 Action mock 的成功结果不能替代此项。

| 检查项 | 字段、阈值与判据 | 本次结果 | 本次证据路径 |
|---|---|---|---|
| 输入来源 | 固定 Navigation v0.1 压缩包 SHA256、镜像、脚本和采集命令；注明本次采集或发布记录回放 | 未测试 | 待填写：镜像核对日志、`navigation-runtime.log`、输入摘要 |
| trace 身份与地图 | `schema: omindos.community.navigation_trace.v1`、`simulation_only: true`、`frame_id: map`、`backend: omindos_navigation_0.1_topics`、`runtime: omindos/navigation:0.1`；地图 `resolution/origin/data` 有效 | 未测试 | 待填写：`navigation-trace.json` |
| 时间与记录完整性 | `samples` 非空且至多 20000；`time_s` 相对目标发布单调递增；相邻间隔不大于 0.5 s；非有限值与异常位姿跳变被拒绝 | 未测试 | 待填写：trace、`test_corrupt_trace_rejected` 日志 |
| 目标与路径 | 收集器检查路径 header 不早于目标发布；sample 的 `path_time_s` 在 [0, time_s]，`path_end` 到 `goal_xy` 不超过 0.25 m | 未测试 | 待填写：trace、收集器版本与运行日志、错误 / 发布前路径回归 |
| 到达与新鲜度 | `pose[:2]` 到 `goal_xy` 不超过 0.25 m；`pose_time_s`、`cmd_time_s` 非负且不晚于当前样本，年龄各不超过 0.5 s | 未测试 | 待填写：trace、陈旧定位 / 命令回归 |
| 连续停稳 | `twist` 顺序为 linear xyz、angular xyz，六分量绝对值各不超过 0.001（m/s 或 rad/s）；连续至少 0.6 s，位置相对稳定锚点漂移不超过 0.01 m、yaw 漂移不超过 0.02 rad | 未测试 | 待填写：末段样本 / 时间区间，六轴与漂移回归 |
| 观测中断与最终状态 | 超过 0.5 s 中断重置稳定计时；导入含数据间断的整条 trace 被拒绝；末样本门禁通过且 `status: arrived` 才放行 | 未测试 | 待填写：trace、中断 / failed 状态回归 |
| 导航终点朝向 | 保留 `samples[-1].pose[2]`；结果 `base_final_pose` 与末样本一致，不强行设置用户目标 yaw | 未测试 | 待填写：trace、结果与朝向回归 |
| 运输姿态与包络 | 腰为 `transport_rad`，双臂保持运输姿态、夹爪不持物；底盘、肩、双臂和夹爪碰撞盒 XY 最大半径加 0.05 m；保存 `footprint_radius_m`（默认约 0.790582 m） | 未测试 | 待填写：plan 的运输位、配置、半径和运输回归 |
| 地图净空 | 整机包络膨胀后每段可通行；占用 / 未知格与边界保守处理、不斜穿格角；回放地图障碍拉伸为 2 m 柱体 | 未测试 | 待填写：输入地图、plan 与净空 / 窄通道回归 |
| 底盘整机碰撞重查 | 底盘插值最大间隔 0.025 m / 0.025 rad；运输位不变、每个样本无碰撞；`validated_base_samples` 记录实际检查数 | 未测试 | 待填写：`navigation-integration.json`、plan 与执行日志 |
| 导入后规划与阶段 | plan 中 `navigation` 对应输入 trace；`base_coordinates: ["x_m","y_m","yaw_rad"]`；`phase_history` 为 idle → navigating → ready → manipulating → succeeded；再核第 7 节十五关节结果 | 未测试 | 待填写：trace、`navigation-integration-plan.json` 与结果 |
| 拒绝场景 | 错 frame、缺路径 / 错目标 / 发布前路径、陈旧数据、未停稳、漂移、NaN、间断 / 跳变、导航失败、无路 / 窄路 / 碰撞不得完成联动；无路和导入净空失败回归验证未调用臂规划器 | 未测试 | 待填写：`community.test_navigation` 14 项逐项日志 |
| 失败输出隔离 | `rm75_navigation` 失败非零退出；指定结果 / plan 路径在运行前清理，不能用残留文件判成功；保留失败命令与日志 | 未测试 | 待填写：失败复验日志、退出码与输出目录记录 |

净空验证输出是通过/拒绝与包络、采样计数；没有“实测最小净空”字段。
若项目要求数值最小净空，应另行测量并标注方法，不得把 `footprint_radius_m` 当作最小间隙。
`validated_base_samples` 是插值检查数，不必等于原始 `samples` 条数。
JSON 保存的是接收相对时间和派生路径终点，不是完整 rosbag 或原始 header 档案；原始消息审计需另行采集。

Navigation v0.1 没有任务 ID、接受确认、可靠取消 / 抢占；无路可能保留旧路径，数据过期也不保证实车停车。
本表不把命令近零、到达字段或门禁通过解释为真实底盘已安全停稳。

## 9. Legacy / reference 范围

保留原[v0.1.1 验收模板](ACCEPTANCE_RECORD.zh-CN.md)作为历史参考，不修改旧验收结论。
下列项目即使通过，也不能作为 RM75 目标机通过依据。

| 旧能力 / 字段 | 本次处理方式 | 本次结果 | 证据 / 不适用原因 |
|---|---|---|---|
| UR3e、PolyScope、External Control、URCap、Dashboard | 仅实际 UR3e 项目填写；RM75 控制器资料填第 2 / 10 节 | 未测试 | 待填写 |
| `ur3e_preflight`、`ur_dashboard_probe`、Hand-E / 2F-85 模板 | UR3e 只读接入参考；`observed` 和固定 `ready_for_motion: false` 不代表目标机可运动 | 未测试 | 待填写 |
| 六轴 `plan_joint_motion`、前七项示例 | 软件兼容性 / 教学参考，不能代替 RM75 十五关节规划 | 未测试 | 待填写 |
| `community.legacy_mobile_dual_pitch` | 原 13 关节模拟模型 / `ros2-waist-playback.json` | 未测试 | 待填写 |
| 原 12/13 关节 ROS2 Action 通信 | 按[ROS2 接口说明](ROS2_INTERFACE.md)独立运行；不是 RM75 15 关节驱动，也不是 Navigation 实际 Topic 接口 | 未测试 | 待填写：两套 ROS2 summary / client / server 日志 |

ROS2 summary 是案例数组：success 案例应为 `passed`；`--fail-navigation`、`--fail-manipulation`
案例应为预期 `failed` 并具有相应终止阶段。不能要求数组中所有 `status` 都是 `passed`。
该兼容性检查在发布工作流单独执行，不计入 51 个测试方法或九项示例。

## 10. 客户现场连接与真机执行（C/D；无证据保持未测试）

### C：只读连接与配置核对

| 检查项 | 现场应保留的证据 | 本次结果 | 本次证据路径 |
|---|---|---|---|
| 左右 RM75-B 身份、末端版本、控制器 / 固件 / SDK | 设备实际身份与项目配置对应；型号不符则先更换模型 / 适配 | 未测试 | 待填写 |
| 15 关节映射、单位、正方向、零位、实际限位 | 实测反馈与模型顺序逐轴对照；不能从模拟关节消息推定 | 未测试 | 待填写 |
| 腰轴、双臂安装、底盘 map / base、TCP 标定 | 标定文件、版本、日期、测量方法及误差 | 未测试 | 待填写 |
| 工具、负载、重心、底盘与环境几何 | 实物配置与碰撞模型差异记录 | 未测试 | 待填写 |
| 状态、故障、驱动与停止反馈可观测性 | 实际通信日志与状态语义；只读观察不执行运动 | 未测试 | 待填写 |

### D：完成项目执行适配后另行验收

v0.2 离线样本不能直接下发；先记录实际适配程序、提交、驱动和时间参数化，
由项目双方确定执行与现场停止方案。此节不是自动执行指令。

| 测试前约定 | 本次记录 |
|---|---|
| 适配程序、控制器、驱动、15 关节映射与输入校验 | 待填写 |
| 速度、加速度、负载 / 重心、底盘与腰部运行条件 | 待填写 |
| 作业区域、障碍物、工具 / 工件、抓取与放置坐标 | 待填写 |
| 到达和实测停稳的测量方法、互锁、急停 / 制动与恢复行为 | 待填写 |
| 有效尝试定义、次数、成功标准、误差、周期时间指标 | 待填写 |
| 失败、超时、取消、通信丢失与人工干预的计数方法 | 待填写 |

| 现场检查 / 执行项 | 本次结果 | 实测值 / 异常 | 证据路径 |
|---|---|---|---|
| 单臂 / 双臂 / 腰部动作与轨迹跟踪 | 未测试 | 待填写 | 待填写 |
| 真实导航到达、实测停稳、再启动机械臂的互锁 | 未测试 | 待填写 | 待填写 |
| 实际抓持、搬运、释放、放置误差及周期 | 未测试 | 待填写 | 待填写 |
| 无规划、夹持失败、控制器拒绝、超时 / 通信丢失 | 未测试 | 待填写：不推进后续动作与恢复行为 | 待填写 |
| 操作员停止、取消、急停 / 制动与恢复后行为 | 未测试 | 待填写：实际停止反馈；不能以离线门禁替代 | 待填写 |

逐次执行记录（按实际试次追加，不预填成功）：

| 次数 / 时间 | 动作与配置 | 实际反馈 / 结果 | 误差 / 周期 | 异常 / 人工干预 | 证据 |
|---|---|---|---|---|---|
| 待填写 | 待填写 | 未测试 | 待填写 | 待填写 | 待填写 |

## 11. 分层结论与签署

| 项目 | 本次结论 | 证据索引 / 未完成原因 / 范围限制 |
|---|---|---|
| A：软件与 RM75 离线验证 | 待验收 | 待填写 |
| B：Navigation 实际消息回放联动 | 待验收 | 待填写：重新采集或复用发布记录 |
| C：客户现场只读连接 | 未测试 | 待填写 |
| D：真机运动、性能及现场停止验收 | 未测试 | 待填写 |
| 遗留问题、负责人、计划日期、复验条件 | 待填写 | 待填写 |
| 客户确认人 / 日期；实施方确认人 / 日期 | 待填写 | 待填写 |

不得将 A/B 通过汇总为“整机安全验收通过”。签署时明确本次包含层级、失败或未实施项，
并随记录保留完整提交号、配置、命令、日志、JSON、证据摘要与现场记录。

## 附录：已发布 v0.2 的只读核对索引（非客户验收结果）

核对日期：2026-10-08（北京时间）。以下只描述原发布证据，不重写发布结果或预填上述表格。

- [v0.2 Release](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/releases/tag/v0.2)：2026-10-07 发布，目标提交 `6726ddfdf84bf9eb44a36671566f5199856cbe32`。
- [Community verification run 37644142421](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/actions/runs/37644142421)：`simulation`、`ros2-transport`、`navigation-runtime`、`mobile-integration` 为发布前四道检查；随后 `release-v02` 打包。
- [simulation job 112870377655](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/actions/runs/37644142421/job/112870377655)：unittest 日志记录 `Ran 51 tests` 和 `OK`；该日志不在验证 ZIP 内。
- `OmindOS-Manipulation-0.2-validation.zip` 的 SHA256：`1b55e48566e38bd5190468c471ea346183e21a2027817adf3390a6a30787f97b`；Release 同时附 `SHA256SUMS-v0.2`，下载后自行核对。

| 原发布文件（相对 `release-artifacts/`） | 已有证据中可核对的值；仅作来源定位 |
|---|---|
| `community-simulation/summary.json` | Python 3.12.15；九项 `passed`；`hardware: not tested` |
| `community-simulation/mobile_dual_pitch.json` 与 `kinematics-15dof-playback.json` | 双 RM75-B、15 关节、568 个关节采样 |
| `community-simulation/rm75_navigation.json` 与 `mobile-manipulation-0.2.json` | `standalone_astar`；714 个底盘检查样本、568 个关节采样 |
| `navigation-runtime/navigation-trace.json` | `omindos_navigation_0.1_topics`；43 条采集样本；目标 [-1.6,-2.5]；`arrived`；`simulation_only: true` |
| `mobile-integration/navigation-integration.json` 与 `navigation-integration-plan.json` | 101 个底盘检查样本、568 个关节采样；包络 0.7905822201686025 m；离线阶段至 succeeded |

这些采样数量是该次发布的观测值，复验因路径、配置、运行时与采集时序变化可以不同。
发布资产、标签、原 `docs/validation/` 报告和旧验收记录均作为历史证据保留。
