# UR3e 与夹爪接入手册

适用软件：OmindOS Manipulation 社区版 v0.1.1 主线交付修订
设备范围：UR3e；Robotiq Hand-E 或 2F-85；通过客户现场网络连接

## 1. 本手册的使用范围

本手册指导客户准备设备配置、读取机器人软件版本、安装官方 UR 驱动、
检查 ROS2 数据，并为首次抓放建立调试与验收流程。
当前软件包已提供只读诊断工具和两款夹爪的配置模板；真实机械臂运动、夹爪执行适配和
自动抓放程序需要项目集成后交付。下文把可直接执行的检查命令与后续现场实施步骤分开列明。

一台机械臂配一款夹爪即可开始。双臂设备先分别完成单臂检查，记录所选手臂的命名空间和工具配置。
Hand-E、2F-85 分别维护工具坐标、开口标定与负载参数。

## 2. 准备设备与现场电脑

| 项目 | 客户需要准备的内容 |
|---|---|
| 机器人 | UR3e 控制箱、示教器、实际机器人 IP、PolyScope 版本 |
| 夹爪 | 实际型号、安装连接件、供电、通信接线、适配驱动或 URCap |
| 电脑 | 网口、Python 3；使用 ROS2 检查时准备 Ubuntu 22.04 / ROS2 Humble |
| 网络 | 电脑与机器人互通，按现场网段分配不同地址，避免地址冲突 |
| 标定 | 该机器人的出厂运动学参数、工具中心点、实际工具及物体负载 |
| 工作区 | 安装支架、工作台、工装、物体和其他机械臂的位置与几何 |

TCP/IP 是电脑与 UR3e 的通信路径。夹爪可能通过 URCap、末端接口、独立控制器或串行转接连接，
其协议和反馈单位按实际产品确定，不能根据机器人使用 TCP/IP 就自动套用同一协议。

## 3. 创建客户配置记录

从仓库根目录运行以下命令，按实际夹爪选择其中一个模板：

```bash
mkdir -p configs/hardware/local
# Hand-E：
cp configs/hardware/ur3e-hande.template.json configs/hardware/local/ur3e-site.json
```

使用 2F-85 时，将源文件换成 `configs/hardware/ur3e-2f85.template.json`。
Windows 用户可直接新建 `local` 文件夹并复制、重命名文件。

| 配置项 | 填写方法 |
|---|---|
| `robot.robot_ip` | 示教器中显示的机器人实际地址 |
| `robot.polyscope_version` | 诊断报告中的版本；读取失败时查看示教器版本页 |
| `robot.calibration_file` | 此机器人导出的标定文件绝对路径 |
| `host.os_version` / `host.ros_distribution` | 实际电脑环境 |
| `gripper.model` / `gripper.connection` | 实际夹爪及通信路径 |
| `gripper.opening_calibration_record` | 开口值与反馈单位的标定记录路径 |
| `setup.single_arm_side` / `setup.joint_prefix` | 单臂或所选左、右臂及真实关节前缀 |
| `setup.tcp_xyz_m_rotation_vector_rad` | 工具中心点的平移和旋转向量，按字段注明 m、rad |
| `setup.tool_and_part_payload_kg` | 实际工具与抓持物体的负载记录 |
| `setup.tool_and_part_center_of_gravity_m` | 与该负载对应的重心记录 |
| `task.known_pick_pose_record` / `task.known_place_pose_record` | 实际取放位置及坐标系的记录路径 |

所有模板都是现场资料表。诊断命令通过命令行参数读取配置，不会自动加载此 JSON；
模板中填入坐标也不会触发运动。`null` 代表尚未填写，不代表零值或默认设备参数。
`configs/hardware/local/` 已被 Git 忽略，客户资料保存在自己的交付记录中。

## 4. TCP/IP 连接与版本检查

在同网段现场电脑上下载最新 `main` 源码并进入仓库根目录。
以下命令中的 `机器人IP` 是占位符，必须替换为实际地址：

```text
python3 -m community.hardware.ur_dashboard_probe --host 机器人IP
```

Windows 中可使用 `python`。此工具只用 Python 标准库，无需安装 ROS2 或仿真依赖。

| 内容 | 说明 |
|---|---|
| 目标端口 | 默认 TCP 29999；仅连接明确提供的单个地址 |
| 查询内容 | 软件版本、型号回复、机器人模式、安全模式、程序运行状态、远程模式 |
| 结果文件 | `community-results/hardware/tcp-report.json` |
| 超时 | 默认每次响应 3 秒，可用 `--timeout` 修改 |
| 操作范围 | 只读；不启停程序、不上电、不解锁、不发送运动或夹爪指令 |

`status: observed` 表示识别到了软件版本；`status: incomplete` 表示连接或查询未完成，
详情在 `error` 与 `responses` 中。型号回复中的 `UR3` 需要结合铭牌确认，不能单独用来区分 UR3 与 UR3e。

此工具针对 PolyScope 5 Dashboard。PolyScope X 使用另一套 Robot API；
29999 连接失败时应核查接口与实际版本，不自动推断软件版本。
连接报告始终不作为运动许可或真机完成证明。

## 5. 安装官方 UR 驱动并加载标定

本节在已安装 ROS2 Humble、已配置 ROS2 软件源的 Ubuntu 22.04 上执行。
全新 ROS2 主机先按 [UR Humble 官方安装说明](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
链接中的 ROS2 / 驱动环境要求完成准备。不要在仿真的 Python 3.12 虚拟环境中执行本节。

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ur ros-humble-ros2controlcli \
  ros-humble-sensor-msgs ros-humble-control-msgs ros-humble-controller-manager-msgs
source /opt/ros/humble/setup.bash
```

按实际 PolyScope 版本安装 External Control URCap，并在其设置中填写现场电脑地址。
机器人与电脑的启动流程以[官方机器人设置说明](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html)为准。

导出机器人标定时，控制器需按官方要求上电。以下为命令格式，占位符由客户替换：

```text
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=机器人IP \
  target_filename:=标定文件绝对路径
```

启动驱动时加载同一台机器人的标定文件：

```text
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=机器人IP \
  kinematics_params_file:=标定文件绝对路径 \
  headless_mode:=false
```

核对驱动日志中的标定检查结果，由现场操作人员在示教器上启动对应的 External Control 程序。
使用这里的非 headless 启动方式保留示教器的程序启动步骤。
驱动连接建立后继续做只读检查，此时不播放 Community 模拟轨迹。

## 6. ROS2 关节与控制器检查

保持驱动运行，在第二个终端进入仓库根目录：

```bash
source /opt/ros/humble/setup.bash
python3 -m community.hardware.ur3e_preflight
```

| 默认检查项 | 值 |
|---|---|
| 关节状态 | `/joint_states` |
| 六关节前缀 | 空字符串 |
| 轨迹 Action | `/scaled_joint_trajectory_controller/follow_joint_trajectory` |
| 控制器管理器 | `/controller_manager` |
| 控制器 | `scaled_joint_trajectory_controller` |
| 最长观察时间 | 15 秒 |

自定义命名空间或双臂设备使用以下参数替换默认值：`--joint-topic`、`--joint-prefix`、
`--arm-action`、`--controller-manager`、`--controller`。参数值应从实际驱动配置读取。

结果位于 `community-results/hardware/ur3e-observation.json`。

| 结果 | 含义 |
|---|---|
| `observed`，退出码 0 | 收到多个有效时间戳样本，发现 Action，并查询到活动控制器 |
| `incomplete`，退出码 1 | 缺少数据、数据过期、控制器未活动或观察被中断 |
| 退出码 2 | 参数或 ROS2 依赖错误 |
| `stationary_sample: true` | 最近一次样本的六关节速度绝对值均小于 0.001 rad/s |
| `stationary_sample: null` | 未收到速度数组，停稳状态未知 |

工具检查关节名称、数值与时间戳；`observed` 仅证明接口和数据被观察到。
它不确认工具标定、抓持状态或运动安全，因此 `ready_for_motion` 与 `physical_robot_verified` 保持 `false`。
ROS2 时钟需与数据源一致，当前工具不使用模拟时间。

## 7. 夹爪接入与执行适配

先根据夹爪接线选择厂商支持的控制方式，记录驱动 / URCap 版本，再分别标定 Hand-E 或 2F-85。
本软件包未提供可直接执行的 `open_gripper`、`close_gripper` 或自动抓放命令；
本节是执行适配的交付要求，不能将模板存在解释为驱动已经可用。

| 适配项目 | 交付要求 |
|---|---|
| 开口与反馈 | 明确单位和标定，区分发出指令、到位、抓住物体、故障 |
| 工具与负载 | 使用实际安装的夹爪、连接件和物体参数 |
| 运动输入 | 校验关节顺序、限位、实际起点及包含工具的碰撞场景 |
| 执行轨迹 | 提供时间、速度和加速度约束；按现场控制器接口发送 |
| 结果判断 | 结合控制器返回和实测反馈，失败时停止后续步骤 |
| 取消与异常 | 验证取消、超时和失联后的处理，恢复连接后不自动续跑 |

`community.api` 目前返回裸臂几何路径；`plan_grasps` 使用 Robotiq85 模型。
Hand-E 的夹爪几何、标定和执行需独立适配。离线十二关节回放同样不能直接重映射到真实控制器。

## 8. 首次已知位置抓放流程

以下流程在执行适配、工具标定与现场路径验证完成后实施。
取放位置由现场示教或经标定的规划得到，不在产品文档中预设客户坐标。

| 顺序 | 操作 | 记录的完成证据 |
|---|---|---|
| 1 | 读取机器人与夹爪状态 | 设备身份、关节顺序、反馈正常 |
| 2 | 独立测试夹爪开合 | 开口单位、实际行程、到位与故障反馈正确 |
| 3 | 空载到抓取接近点 | 规划与实际轨迹一致，关节到位 |
| 4 | 接近物体 | 工具与工装、支架、其他手臂无非预期接触 |
| 5 | 闭合夹爪 | 夹持反馈有效，不仅是指令已发送 |
| 6 | 抬升和搬运 | 物体随夹爪移动，无滑落 |
| 7 | 到放置点并释放 | 物体获得支撑，释放反馈有效 |
| 8 | 退回接近点 | 任务结束，保存周期、误差与异常记录 |

首次运行使用现场评估确定的低速与负载设置，由现场操作人员监控。
任一步异常时停止推进，不在通信恢复或保护停止解除后自动继续。
先验收一次完整抓放，再按照项目约定进行重复测试。

## 9. 故障处理

| 现象 | 优先检查 | 处理结果应记录 |
|---|---|---|
| TCP 连接超时或拒绝 | IP、网段、网线、目标服务是否开放、实际 PolyScope 类型 | 地址配置与原始报错 |
| 软件版本无法识别 | `responses.software`、示教器版本页、是否为 PolyScope X | 实际版本与接口类型 |
| `No module named community` | 是否从完整仓库根目录运行 | 使用的目录与源码提交号 |
| `No module named rclpy` | ROS2 环境是否加载、是否误用 Python 3.12 虚拟环境 | Python 与 ROS2 环境 |
| ROS2 没有关节数据 | 驱动进程、话题、命名空间、DDS 通信与时钟 | 话题名与诊断报告 |
| 缺少或重复关节 | 单臂前缀与实际 `joint_states` 是否匹配 | 六关节映射 |
| 控制器未活动 | `ros2 control list_controllers`、External Control 程序和驱动日志 | 控制器状态及日志 |
| 收到位置但没有速度 | 反馈数组是否包含速度 | 停稳状态按未知处理 |
| 夹爪动作或开口不符 | 实际型号、单位、接线、标定与驱动版本 | 夹爪配置和反馈 |
| 几何路径成功但不能执行 | 轨迹时序、工具、负载及执行适配是否完成 | 缺失的执行条件 |

## 10. 验收与资料归档

使用[验收记录模板](ACCEPTANCE_RECORD.zh-CN.md)分别记录软件自检、只读连接和真机抓放。
未执行的检查填写“未测试”；自动测试、模拟服务与现场实物结果分别保存。
客户资料、标定文件与原始日志保存在本地项目记录中，公开提交问题前按需脱敏。

## 参考文档

- [UR Humble 驱动及安装包](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
- [UR 标定参数提取](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/robot_setup.html)
- [UR 驱动启动与标定检查](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/startup.html)
- [Dashboard TCP/IP 协议](https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/)
- [Dashboard 查询命令表](https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-dashboard/Dashboard_table.htm)
- [PolyScope X 接口区别](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_client_library/doc/examples/dashboard_client.html)
