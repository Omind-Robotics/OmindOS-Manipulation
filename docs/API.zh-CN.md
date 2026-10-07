# Python API 使用手册

十五关节、外部 URDF 和移动底盘的客户 API 见 [0.2 发行版指南](CUSTOMER_RELEASE.zh-CN.md)。
本文保留六轴 UR3e 与旧通信接口的兼容说明；`community.api.ManipulationClient`
与 `sdk.omindos_client.ManipulationClient` 是同一个客户类。

0.2 新增底盘导航、到达停稳门禁和双 RM75＋腰部联动，入口：`python -m community.rm75_navigation`。
详见[导航联动指南](NAVIGATION_v0.2.zh-CN.md)和[0.2 发布说明](RELEASE_v0.2.md)。

适用版本：OmindOS Manipulation 0.2。入口：`community.api.ManipulationClient`。
本文中的接口均为本地离线计算，不连接真实机器人。ROS2 通信接口见本文末尾。

## 1. 运行准备

按[安装指南](INSTALL.zh-CN.md)完成环境配置，在仓库根目录激活 `.venv`。
保存自己的脚本到仓库根目录后，用 `python 文件名.py` 运行；也可以直接进入 `python` 交互环境。

完整可运行例子已提供：

```bash
python -m community.api_example
```

输出保存到 `community-results/api-example.json`。

## 2. 调用方式

```python
from community.api import ManipulationClient

with ManipulationClient() as client:
    task_id = client.plan_joint_motion(
        start=[0.0, -0.7, 0.9, 0.3, 0.6, 0.0],
        goal=[0.45, -0.8, 1.1, 0.45, 0.8, -0.15],
        obstacles=[{
            "size_m": [0.12, 0.12, 0.30],
            "position_m": [0.25, -0.25, 0.15],
        }],
        seed=7,
    )
    print(client.get_status(task_id)["status"])
    response = client.wait(task_id, timeout=120)
    if response["status"] != "succeeded":
        raise RuntimeError(response["error"])
    path = response["result"]["positions_rad"]
    print("起点：", path[0])
    print("终点：", path[-1])
```

每次提交都会启动一个独立进程并立即返回 `task_id`。使用 `with`，退出时会终止未完成的计算、
回收进程并删除临时文件。结果需要留存时，请在自己的程序中保存返回的字典。
任务编号只在当前客户端实例中有效，不是服务器上的持久任务；客户端不支持多线程同时调用。

## 3. 关节运动规划

```python
task_id = client.plan_joint_motion(start, goal, obstacles=(), seed=7)
```

| 参数 | 类型与单位 | 说明 |
|---|---|---|
| `start` | 六个有限数值，rad | 起点关节角 |
| `goal` | 六个有限数值，rad | 目标关节角 |
| `obstacles` | 障碍物字典列表，可为空 | 每项必须恰好包含 `size_m` 与 `position_m` |
| `size_m` | 三个正数，m | 轴对齐长方体沿 x、y、z 的完整尺寸，不是半尺寸 |
| `position_m` | 三个有限数值，m | 障碍物中心在机械臂基座坐标系中的位置 |
| `seed` | 0 至 2³²−1 的整数 | 随机种子，默认 7；便于固定环境中复现 |

六关节顺序：

| 序号 | 名称 |
|---|---|
| 1 | `shoulder_pan_joint` |
| 2 | `shoulder_lift_joint` |
| 3 | `elbow_joint` |
| 4 | `wrist_1_joint` |
| 5 | `wrist_2_joint` |
| 6 | `wrist_3_joint` |

API 调用原有 UR3e 模型和 RRT-Connect，检查关节限位、自碰撞与传入的障碍物。
返回路径再次加密采样，最大相邻关节增量为 0.025 rad。

成功结果 `response["result"]` 的主要字段：

| 字段 | 含义 |
|---|---|
| `positions_rad` | N×6 路径点列表，首尾对应请求起终点 |
| `joint_names` | 六关节名称及顺序 |
| `frame_id` | `arm_base`，障碍物所在的机械臂基座坐标系 |
| `validated_samples` | 已检查的路径采样点数 |
| `edge_step_rad` | 路径检查的最大关节采样间隔 |
| `obstacles` | 本次请求的障碍物 |
| `scope` | 当前计算范围说明 |

该路径没有时间戳、速度、加速度或负载约束，不能直接作为控制器执行轨迹。
规划对象是裸臂，不含夹爪、安装支架和抓持物体。采样检查不等同于连续碰撞检测。
路径搜索有时间和迭代上限，失败不必然说明数学上不存在路径。

## 4. 抓取候选

```python
with ManipulationClient() as client:
    task_id = client.plan_grasps([0.02, 0.06, 0.04], jaw_width_m=0.065)
    response = client.wait(task_id)
    if response["status"] != "succeeded":
        raise RuntimeError(response["error"])
    candidates = response["result"]["candidates"]
    print("候选数量：", len(candidates))
```

| 参数 | 要求 |
|---|---|
| `size_m` | 长方体完整尺寸 `[x, y, z]`，三个正数，单位 m |
| `jaw_width_m` | 夹爪开口，0 < 宽度 ≤ 0.085 m，默认 0.065 m |
| `seed` | 与运动规划相同，默认 7 |

物体中心位于原点，轴与物体坐标系对齐。使用 Robotiq85 模型，在预设的夹爪旋转候选中进行网格碰撞过滤。
这不是任意点云或任意形状物体的通用抓取识别接口。

每个候选包含：

- `jaw_width_m`：夹爪开口。
- `jaw_center_position_m`、`jaw_center_rotation`：夹爪中心在物体坐标系下的位置及 3×3 旋转矩阵。
- `gripper_position_m`、`gripper_rotation`：夹爪模型基座在物体坐标系下的位置及 3×3 旋转矩阵。

旋转矩阵表示从相应局部坐标系到物体坐标系的旋转；点坐标满足 `p_object = R @ p_local + t`。
没有候选时任务返回失败，不会返回伪成功。候选仍需接触、力闭合、机械臂可达性和接近路径验证。

## 5. 运行现有示例

```python
with ManipulationClient() as client:
    task_id = client.run_demo("rm75_navigation", seed=7)
    response = client.wait(task_id)
    print(response["status"], response["result"])
```

允许的名称：`quick_start`、`ur3e_single_arm`、`grasping`、`motion_planning`、
`dual_arm`、`mobile_dual_arm`、`integrated_task`、`mobile_dual_pitch`、`rm75_navigation`。
统一入口以无窗口方式运行；成功结果保留原示例的指标。

命令行也可使用：

```bash
python -m community.api dual_arm --timeout 120 --output community-results/dual-arm-api.json
```

命令行成功返回 0，失败或取消返回非 0；按 Ctrl+C 会取消本地计算。

## 6. 状态、等待与停止

```python
status = client.get_status(task_id)
response = client.wait(task_id, timeout=120)
canceled_or_finished = client.stop(task_id)
```

| `status` | 含义 | `result` |
|---|---|---|
| `running` | 本地计算尚未结束 | `None` |
| `succeeded` | 工作进程正常退出且产生通过检查的报告 | 对应结果字典 |
| `failed` | 算法或依赖错误、无有效结果、等待超时 | `None` |
| `canceled` | 用户取消了本地计算 | `None` |

统一返回字典包含 `task_id`、`operation`、`status`、`simulation_only`、`result`、`error`。
`operation` 为 `joint_motion`、`grasps` 或 `demo`，`simulation_only` 始终为 `True`。
内层计算报告使用 `status: passed`；外层任务状态使用 `status: succeeded`，两者含义不同。

`get_status` 不阻塞等待完成，也不查询物理机器人。
`wait` 从调用时开始计时，默认 120 秒；超时会终止工作进程并报告 `failed` / `timeout`。
终止进程可能额外需要最多约 6 秒。没有调用 `wait` 时不存在后台超时看门狗。
请主动查询状态，或通过 `with` / `close()` 回收任务。

停止已经完成的任务会保留原终态，不会把成功改成取消。
`stop()` 是计算取消，不能用来代替机器人控制器或硬件急停。

## 7. 错误处理

| 情况 | 表现 | 建议 |
|---|---|---|
| 向量长度不对、NaN、字符串、非法宽度或种子 | 提交时抛出 `ValueError` | 修正参数，不会启动计算 |
| 已关闭客户端再次提交 | `RuntimeError` | 新建客户端 |
| 不属于本客户端的任务编号 | `KeyError` | 保存并使用原实例返回的编号 |
| 关节超限、碰撞阻挡、找不到候选、依赖缺失 | `failed`，`error.code = worker_failed` | 阅读 `error.details`，最后最多 2000 字符的工作日志 |
| 等待超时 | `failed`，`error.code = timeout` | 检查场景复杂度、环境和时限 |
| 主动取消 | `canceled`，`error.code = canceled` | 根据业务需要重新提交 |

成功的 `error` 为 `None`，失败时不要继续把空结果送往下游。

保存结果示例：

```python
import json
from pathlib import Path
Path("my-result.json").write_text(
    json.dumps(response, indent=2, ensure_ascii=False, allow_nan=False),
    encoding="utf-8",
)
```

## 8. ROS2 与其他语言

Python API 用于当前进程发起离线规划。C++ 程序与 Navigation 的跨进程集成继续走 ROS2 标准接口，
不需要把规划算法重写为 C++。

| 通信接口 | 当前名称 | 内容 |
|---|---|---|
| `NavigateToPose` | `/omindos/community/navigate_to_pose` | `map` 中的导航目标与结果 |
| `FollowJointTrajectory` | `/omindos/community/follow_joint_trajectory` | 按文档排序的十二关节模拟回放 |
| `Odometry` | `/omindos/community/odom` | 模拟底座位姿及停稳判断 |

本节 ROS2 服务仍为模拟实现。单臂 API 的六关节无时序路径，不能直接作为十二关节回放输入。
请使用 `community.integrated_task --plan-output ...` 生成现有桥接所需格式，参见
[ROS2_INTERFACE.md](ROS2_INTERFACE.md)。真实硬件适配在后续工作中完成。

## 9. 客户设备接入

本手册中的 `plan_grasps` 使用 Robotiq85 几何模型，现场 Hand-E 模板不会自动切换该算法模型。
真实执行适配、工具标定、夹爪接线与连接诊断见[设备接入手册](UR3E_COMMISSIONING.zh-CN.md)。
交付流程从[客户使用手册](CUSTOMER_GUIDE.zh-CN.md)进入，项目验收使用[记录模板](ACCEPTANCE_RECORD.zh-CN.md)。

## 腰部 pitch 与双臂

默认十五关节运动学示例（无需 ROS）可用 `client.run_demo("mobile_dual_pitch")` 调用。
自定义配置、腰部目标和十五关节模型见[移动双臂与腰部俯仰](MOBILE_DUAL_PITCH.zh-CN.md)。
`plan_joint_motion` 仍只接受六关节裸臂请求，不会自动转为十五关节规划。
