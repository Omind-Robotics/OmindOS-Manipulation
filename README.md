# OmindOS Manipulation 0.2 开发版

本仓库为私有开发源码，仅授权团队可访问。客户统一使用
[公开发行仓与安装说明](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release)，无需访问开发仓或旧 Community 仓。
仓库分工、旧克隆迁移与历史记录权限见[迁移说明](docs/REPOSITORY_MIGRATION.zh-CN.md)。

机械臂运动与抓取规划软件，由 OriginMind 源灵智能维护。
基于原有 UR3e / WRS 工程，提供单臂、固定双臂和移动操作示例，以及可传入自定义参数的 Python 规划接口。

**移动双臂默认使用两只睿尔曼 RM75-B：左臂 7 轴＋右臂 7 轴＋腰部 pitch，共 15 自由度。直接运行运动学仿真，不需要 ROS。**
安装后执行 `python -m community.rm75_navigation` 打开底盘绕障、停车、双臂和腰部联动的三维动画；加 `--headless` 可无窗口验证。
原地十五关节演示保留为 `python -m community.mobile_dual_pitch`。

**当前版本：OmindOS Manipulation 0.2。** 适用于离线几何规划、仿真示例和接口开发。
已发布的 [v0.1 历史源码快照（需旧仓权限）](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/releases/tag/v0.1) 保持不变；
[0.2 发布说明](docs/RELEASE_v0.2.md)包含底盘导航联动、统一 API 与中文手册。`main` 随后续修订更新，复现实验时请记录提交号。

**客户从这里开始：[0.2 runtime.2 公开发行包与安装说明](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release/releases/tag/v0.2-runtime.2)**

[发行包与移动双臂 API 文档](docs/CUSTOMER_RELEASE.zh-CN.md)

开发版保留完整工程。客户发行包保留 URDF/网格、腰部与底盘参数、标准库 SDK、
调用示例和文档，算法以封装运行程序提供。新接口支持客户指定十五关节目标和底盘导航场景。
构建工具和独立运行检查见 [发行包构建](docs/BUILD_CUSTOMER_RELEASE.md)。
发行包候选需通过独立检查才可交付；本说明不更改已发布的 `v0.2` tag 或旧资产。

[原有客户流程与设备接入](docs/CUSTOMER_GUIDE.zh-CN.md)

新增[双手任务空间与共享腰关节算法](docs/DUAL_TASK_ALGORITHM.zh-CN.md)：两手目标位姿联合逆解、固定抓持关系的协同路径及采样碰撞检查。开发入口 `python -m community.dual_task_motion --headless`；runtime.2 客户 SDK 入口 `plan_mobile_object_motion`。

[产品说明](docs/PRODUCT_MANUAL.zh-CN.md) · [安装指南](docs/INSTALL.zh-CN.md) ·
[API 使用手册](docs/API.zh-CN.md) · [UR3e 接入参考](docs/UR3E_COMMISSIONING.zh-CN.md) ·
[RM75-B 0.2 验收记录](docs/ACCEPTANCE_RECORD_RM75_v0.2.zh-CN.md) ·
[旧版验收参考](docs/ACCEPTANCE_RECORD.zh-CN.md) · [更新记录](CHANGELOG.md) · [English](README.en.md)

## 开发源码安装与第一次运行

以下步骤需要本私有仓库的读取权限。普通客户按上方公开发行包说明安装。

参考环境：**Ubuntu 24.04 x86_64、Python 3.12**。无需连接机器人即可运行。

```bash
sudo apt-get update
sudo apt-get install -y git python3.12-venv libgl1 libgomp1

git clone --branch main --single-branch https://github.com/maganrobotics-boop/OmindOS-Manipulation.git
cd OmindOS-Manipulation
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-community.txt
python -m pip check
python -m community.verify
```

九项示例全部显示 `passed`，且命令退出码为 0，表示离线示例检查通过。
报告位于 `community-results/summary.json`。模型、碰撞检测与依赖在无窗口模式下仍然需要完整安装。

已有旧版仓库的用户请按[升级说明](docs/INSTALL.zh-CN.md#已有仓库升级)操作，保留自己的修改。

## 统一 Python 接口

```python
from community.api import ManipulationClient

with ManipulationClient() as client:
    task_id = client.plan_joint_motion(
        start=[0.0, -0.7, 0.9, 0.3, 0.6, 0.0],
        goal=[0.45, -0.8, 1.1, 0.45, 0.8, -0.15],
    )
    result = client.wait(task_id, timeout=120)
    if result["status"] != "succeeded":
        raise RuntimeError(result["error"])
    path = result["result"]["positions_rad"]
    print("路径采样点：", len(path))
```

在仓库根目录运行 Python。角度为弧度，距离为米。
此调用返回经过采样碰撞检查的 UR3e 裸臂几何路径，**不会驱动机械臂**，也不包含速度、加速度与执行时序。

| 操作 | 接口 | 返回内容 |
|---|---|---|
| 关节运动规划 | `plan_joint_motion(start, goal, obstacles=...)` | 六关节路径与验证信息 |
| 抓取候选规划 | `plan_grasps(size_m, jaw_width_m=...)` | Robotiq85 对长方体的几何抓取候选 |
| 运行示例 | `run_demo(name)` | 对应示例的检查结果 |
| 查询任务 | `get_status(task_id)` | 运行中、成功、失败或已取消 |
| 等待完成 | `wait(task_id, timeout=120)` | 最终结果；超时终止本地计算 |
| 取消计算 | `stop(task_id)` | 终止本地工作进程；不等于真机急停 |

直接运行完整调用例子：

```bash
python -m community.api_example
python -m community.api quick_start --output community-results/api-cli.json
```

## 九个示例

| 示例 | 无窗口运行命令 | 内容 |
|---|---|---|
| 快速入门 | `python -m community.quick_start --headless` | 关节链、逆解与正解误差 |
| UR3e 单臂 | `python -m community.ur3e_single_arm --headless` | 模型、逆运动学、自碰撞 |
| 抓取规划 | `python -m community.grasping --headless` | 夹爪候选及网格碰撞过滤 |
| 避障规划 | `python -m community.motion_planning --headless` | RRT-Connect 与路径采样检查 |
| 固定双臂 | `python -m community.dual_arm --headless` | 双臂十二关节与碰撞模型 |
| 移动双臂 | `python -m community.mobile_dual_arm --headless` | 模拟底座位移与任务顺序 |
| 综合任务 | `python -m community.integrated_task --headless` | 模拟导航、双臂依次搬运、几何附着与释放 |
| 15 自由度移动双臂 | `python -m community.mobile_dual_pitch --headless` | 一个 pitch 腰关节＋两只七轴臂，共 15 关节的运动学与几何规划；无需 ROS |
| 底盘导航＋RM75 联动 | `python -m community.rm75_navigation --headless` | A* 绕障、整机净空、停车门禁、随后执行十五关节规划；支持实际导航记录导入 |

有图形桌面时，单个示例去掉 `--headless` 可打开可视化窗口。
无窗口检查不代表图形交互已经验收。历史应用保留在原目录，未全部验证。

腰部 pitch 版本见[移动双臂与腰部俯仰指南](docs/MOBILE_DUAL_PITCH.zh-CN.md)。
双臂采用厂家 RM75-B 的七轴运动链、关节限位和连杆外形；安装位置、腰部与夹爪仍为待标定参考配置。
默认按标准版 RM75-B 建模；六维力等其他末端版本需核对相应模型。万向轮与夹爪开合不计入 15 自由度。
旧十三关节 UR3e 带腰模型保留在 `community.legacy_mobile_dual_pitch`，与新的十五关节入口分开。

## 与 Navigation 连接

0.2 提供两条入口：默认纯 Python 的栅格规划与平面运动学；可选对接
[OmindOS Navigation Community](https://github.com/maganrobotics-boop/OmindOS-Navigation-Community)
实际发布的 `omindos/navigation:0.1` Topic 接口。

后者采集 `/goal_pose`、`/plan`、`/localization/pose`、`/cmd_vel` 和 `/map`，
将真实导航运行时的仿真记录导入整机回放，检查到达、停稳和碰撞后，再规划双臂与腰部。
导航发布版没有 Nav2 Action；原 `community.ros2_bridge` 保留为独立的 12/13 关节模拟 Action 示例。

[0.2 导航联动安装、配置与限制](docs/NAVIGATION_v0.2.zh-CN.md)。
默认模式不需要 ROS；可选导航后端运行在 ROS2 Humble 容器，宿主仍使用 Python 3.12。

## UR3e 与夹爪接入

已提供[客户设备接入手册](docs/UR3E_COMMISSIONING.zh-CN.md)、设备信息模板及
`community.hardware.ur3e_preflight` 只读连接检查。它检查关节数据与控制器接口，
不会发送运动指令。另提供无需 ROS2 的 TCP/IP 版本读取工具和 Hand-E / 2F-85 两套现场模板：

```text
python3 -m community.hardware.ur_dashboard_probe --host 机器人IP
```

从同网段现场电脑运行，将占位符替换为实际 IP。输出连接报告后，再进行夹爪适配和真机抓放。

## 验证与下一步

```bash
python -m unittest community.test_contract community.test_api community.test_pitch community.test_dual7 community.test_rm75 community.test_navigation
python -m community.verify
```

[历史 v0.1 验证记录](docs/COMMUNITY_VALIDATION.md) ·
[当前自动检查](https://github.com/maganrobotics-boop/OmindOS-Manipulation/actions/workflows/community.yml)

0.2 已提供发布版 Navigation 仿真话题对接与移动双臂运动学联动。真机驱动、标定及实际抓放仍需独立联调。
硬件驱动、接触力学、感知识别和整机实测不属于本次交付修订的已完成功能。

## 来源与许可

本项目来源于 Universal Robots UR3e Planning & Control System，继承了
[Weiwei Wan 及合作者的 WRS](https://github.com/wanweiwei07/wrs) 工作。
保留原始算法、作者注释、[MIT 许可证](LICENSE)与[来源说明](ATTRIBUTION.md)。

`Copyright (c) 2020 Weiwei Wan`

OriginMind 维护的 Community 新增内容同样按 MIT 许可证提供。
