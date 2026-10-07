# OmindOS Manipulation 0.2 发行版

OriginMind 源灵智能维护的移动双臂规划软件。默认整机是移动底盘、一个俯仰腰关节、
左右各七轴 RM75-B 机械臂。上半身共十五关节；底盘单独使用 `x、y、yaw` 三个位姿量。
当前发行包用于离线几何规划、碰撞检查和仿真记录联动，不发送真机运动命令。

## 安装和首次运行

本包面向 Linux x86_64。客户端需要 Python 3.9 或更新版本；算法运行包已携带 Python
及所需库，客户端不需要安装完整仿真依赖。图形显示和不同 Linux 发行版需在目标机验证。
开发参考环境是 Ubuntu 24.04 / Python 3.12；实际构建与验证环境记录在 `VERSION.json`。

把两个命名压缩包和 `SHA256SUMS` 下载到同一目录。GitHub 自动生成的 Source code
是开发源码快照，不能代替下面的运行包。

```bash
sha256sum -c SHA256SUMS
tar -xzf OmindOS-Manipulation-0.2-client.tar.gz
cd OmindOS-Manipulation-0.2
./install.sh
python3 examples/mobile_dual_api.py
```

首次运行的成功输出必须有 `status: passed`，并生成 `results/customer-api.json`。
安装脚本只校验并解压运行包；无需管理员权限。目标机可能需安装 OpenGL/X11 系统库
（Ubuntu 可用 `sudo apt-get install libgl1 libgomp1`）。不需要 ROS 即可运行离线示例。

## 客户需要保留和修改的内容

| 位置 | 用途 |
|---|---|
| `robot/arm-model/RM75-B.urdf` 与 `meshes/` | 两臂共用的默认关节链、限位和模型文件 |
| `robot/arms.json` | 左右安装位置、安装姿态、初始关节角与 URDF 路径 |
| `robot/waist.json` | 腰部轴心、角度范围、运输姿态与底盘/躯干参考几何 |
| `robot/navigation.json` | 地图范围、分辨率、起终点底盘位姿和场景障碍物 |
| `sdk/omindos_client.py` | 标准库 Python 客户端；客户程序通过它提交任务 |
| `examples/mobile_dual_api.py` | 十五关节规划和底盘到位后双臂规划的完整调用例子 |
| `runtime/` | 封装后的算法程序、依赖、内部默认模型和许可证，不需要修改 |

URDF 引用的网格必须随模型保留。`package://名称/meshes/文件.STL` 在本包中按 URDF
所在目录解析，不依赖 ROS 包查找；也支持本地相对/绝对 STL 路径。模型文件使用米。
`urdf_path` 相对 `arms.json` 所在目录解析。修改模型后请重新验收，不沿用旧结果。

第一版支持每臂八个连杆、七个独立 revolute 关节构成的一条链，碰撞几何为每连杆一个
STL 网格或 box。加载 joint origin、axis、位置限位、collision origin 和 mesh scale；
不支持的 fixed/mimic/prismatic 关节、分支或多碰撞几何会明确报错，不会偷偷换回默认模型。
这是两臂 URDF＋腰部/底盘参数的配置方式，尚不是任意整机 URDF 的通用加载器。
自定义七轴臂用 `model: URDF-7R` 并填写 `urdf_path`；不能把未经验证的型号标为 RM75-B。
URDF 惯量和速度字段目前不用于动力学或时间参数化。夹爪仍是参考模型，不是任意工具适配。

## 客户 API

```python
from sdk.omindos_client import ManipulationClient

with ManipulationClient() as client:
    info = client.wait(client.get_mobile_model(
        arm_config="robot/arms.json", waist_config="robot/waist.json"))
    if info["status"] != "succeeded":
        raise RuntimeError(info["error"])
    start = info["result"]["home_positions_rad"]
    goal = start.copy()
    goal[0] = 0.10             # 腰部 pitch
    goal[7] = 0.20             # 左臂第七关节
    goal[14] = -0.20           # 右臂第七关节
    task = client.plan_mobile_joint_motion(
        start, goal, base_pose=[0.0, 0.0, 0.0],
        arm_config="robot/arms.json", waist_config="robot/waist.json")
    result = client.wait(task, timeout=120)
    if result["status"] != "succeeded":
        raise RuntimeError(result["error"])
    positions = result["result"]["positions_rad"]
```

| 操作 | 入口 | 输入与结果 |
|---|---|---|
| 查询配置后的整机模型 | `get_mobile_model(arm_config=..., waist_config=...)` | 返回十五关节顺序、初始角、位置限位和源 URDF 关节名 |
| 底盘固定时规划上半身 | `plan_mobile_joint_motion(start, goal, base_pose=..., obstacles=...)` | 自定义十五关节目标；返回密集采样路径，底盘位姿保持不变 |
| 底盘导航＋双臂腰部任务 | `plan_mobile_task(scene=..., target_positions=...)` | 先离线规划并验证底盘全程，再通过到达/连续停稳门禁，随后规划上半身 |
| 导入已采集的导航记录 | `plan_mobile_task(navigation_trace=..., target_positions=...)` | 按既有 Navigation v0.1 仿真话题记录验收底盘碰撞与停稳，再规划上半身 |
| 原有单臂与抓取 | `plan_joint_motion(...)`、`plan_grasps(...)` | 六轴 UR3e 几何路径与 Robotiq85 长方体抓取候选，模型范围不变 |
| 任务管理 | `get_status(task)`、`wait(task, timeout=120)`、`stop(task)` | 查询、等待和取消本地计算；不是硬件急停 |
| 内置示例 | `run_demo(name)` | 九个固定无窗口示例；自定义配置用上述规划入口 |

所有提交返回 `task_id`；通过 `wait` 得到状态与结果。关节顺序为腰部、左臂七关节、
右臂七关节，角度为弧度，距离为米。固定底盘规划的障碍物中心在 `map` 坐标系。
`scene` 与 `navigation_trace` 必须二选一；导航门禁失败不会继续双臂规划。
省略配置使用内部默认模型，正式客户调用建议始终指定自己的配置文件。

底盘任务返回 `navigation.samples`、`transport_positions_rad`、
`manipulation_positions_rad`、`phase_history` 和校验数量。阶段顺序为
`idle → navigating → ready → manipulating → succeeded`。
规划结果没有执行时序、速度、加速度或负载保证，不能直接送控制器执行。

## 开发版与发行版

开发版保留完整源码、模型、测试和构建工具；发行版提供稳定调用入口及封装运行程序。
算法模块以 Python 编译字节码和依赖库封装，不随包附送核心 `.py` 源码。
这不是源码加密，也不是不可逆向的保护方案；公开仓库中已有代码不会因为打包消失。
许可证和来源声明随包保留，不改变 WRS MIT 或 RM75 模型 Apache-2.0 的许可。

Optional Navigation v0.1 是独立运行环境，本包没有复制或重新分发其镜像。
已采集记录的导入不等于实时底盘驱动。真机需要底盘/机械臂驱动、工具与整机标定、
时间参数化、控制器执行和现场验收；本版不声明已完成这些工作。
