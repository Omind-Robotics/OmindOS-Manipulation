# OmindOS Manipulation 客户使用手册

0.2 新增底盘导航、到达停稳门禁和双 RM75＋腰部联动，入口：`python -m community.rm75_navigation`。
详见[导航联动指南](NAVIGATION_v0.2.zh-CN.md)和[0.2 发布说明](RELEASE_v0.2.md)。

版本：v0.2 客户验收文档修订
维护方：OriginMind 源灵智能
适用对象：机械臂应用开发、系统集成、教学与科研用户

客户封装运行包的安装、URDF/参数与新十五关节/底盘 API，见 [发行版指南](CUSTOMER_RELEASE.zh-CN.md)。
下面的源码安装步骤属于开发版；不是精简发行包的安装步骤。

## 1. 产品用途与交付范围

OmindOS Manipulation 提供机械臂运动规划、抓取候选生成、碰撞检查、
单臂与双臂示例，以及与外部系统连接的接口参考。用户可先安装软件完成离线验证，
再按照本机型号、夹爪和现场配置开展设备集成。

当前版本交付源码、模型、离线规划 API、模拟 ROS2 通信示例、只读设备诊断工具及文档。
**RM75、腰部、底盘及夹爪的真机运动执行适配，以及真实 UR3e 自动抓放适配，均未包含在本版本中。**
设备接入章节用于连接检查和后续集成验收，不代表所有客户设备均可安装后直接自动抓放。

| 使用目标 | 当前可以做的事 | 对应入口 |
|---|---|---|
| 了解产品 | 查看功能、环境和适用范围 | [产品说明](PRODUCT_MANUAL.zh-CN.md) |
| 在新电脑安装 | 安装固定依赖，运行九个示例 | [安装指南](INSTALL.zh-CN.md) |
| 在自己的程序中调用 | 传入关节目标、障碍物、物体尺寸，查询或取消计算 | [API 手册](API.zh-CN.md) |
| 检查 UR3e 连接（legacy/reference） | 读取 Dashboard 版本及 ROS2 状态；不能作为 RM75 目标机通过依据 | [UR3e 与夹爪接入手册](UR3E_COMMISSIONING.zh-CN.md) |
| 对接 Navigation v0.1 | 实际发布节点的 Topic 采集、整机离线回放与停稳门禁 | [0.2 导航联动指南](NAVIGATION_v0.2.zh-CN.md) |
| 检查旧 Action 通信（legacy/reference） | 原 12/13 关节模拟通信；不代表 RM75 驱动 | [ROS2 接口说明](ROS2_INTERFACE.md) |
| 记录 RM75-B 0.2 交付结果 | 分别记录离线验证、导航消息回放、现场连接和真机执行 | [RM75-B 0.2 客户验收记录](ACCEPTANCE_RECORD_RM75_v0.2.zh-CN.md) |
| 查阅旧项目验收格式 | v0.1.1、UR3e / PolyScope 等历史字段 | [旧模板（legacy/reference）](ACCEPTANCE_RECORD.zh-CN.md) |

## 2. 获取软件与确认版本

仓库地址：<https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community>

最新维护内容位于 `main`。`v0.2` 是固定发布快照，对应
`6726ddfdf84bf9eb44a36671566f5199856cbe32`；新增验收表通过主线文档提供，
不回写该标签或发行资产。原 `v0.1` Release 保留为历史版本。
使用当前主线时记录完整提交号；按 v0.2 复验时另行记录软件发布提交与验收模板提交。

```bash
git clone --branch main --single-branch https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community.git
cd OmindOS-Manipulation-Community
git rev-parse HEAD
```

也可以在 GitHub 仓库首页选择 `main`，使用 **Code → Download ZIP** 下载并完整解压。
ZIP 用户应同时记录下载日期和仓库对应提交号。源码中的机器人模型和资源文件需要一并保留。

这里介绍的是完整开发版的源码安装，依赖从各自软件源安装。精简运行包使用上面的发行版指南；两种方式均不附带已经标定的硬件控制器配置。

## 3. 选择运行环境

| 用途 | 环境 | 说明 |
|---|---|---|
| 离线规划、API 与九个示例 | Ubuntu 24.04 x86_64，Python 3.12 虚拟环境 | 本版本的仿真参考环境 |
| UR3e TCP/IP 版本诊断 | Python 3，标准库 | 无需 ROS2 或仿真依赖；实际连接须在客户网络检查 |
| ROS2 模拟通信与只读关节诊断 | Ubuntu 22.04，ROS2 Humble，系统 Python | 使用独立于仿真的 Python 环境 |
| 图形演示 | 仿真环境加可用桌面和 OpenGL | 图形显示需在目标电脑检查 |

单独的 TCP/IP 诊断脚本可按说明在 Windows Python 中运行；这不代表整个仿真软件已完成 Windows 验收。
不要将 ROS2 Humble 的系统 Python 二进制模块直接混装到 Python 3.12 仿真虚拟环境。

## 4. 安装并完成软件自检

在 Ubuntu 24.04 上，从仓库根目录执行：

```bash
sudo apt-get update
sudo apt-get install -y python3.12-venv libgl1 libgomp1
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-community.txt
python -m pip check
python -m community.verify
python -m unittest community.test_contract community.test_api community.test_pitch community.test_dual7 community.test_rm75 community.test_navigation
python -m community.api_example
```

| 命令 | 完成标志 | 结果位置 |
|---|---|---|
| `pip check` | 无依赖冲突 | 终端输出 |
| `community.verify` | 九项均显示 `passed`，退出码为 0 | `community-results/summary.json` 与各示例日志 |
| `unittest` | 固定 v0.2 为 `Ran 51 tests`、`OK`，退出码 0；新版记录实际数量 | 终端输出；按验收表保存完整日志 |
| `community.api_example` | `motion`、`grasp` 均显示 `succeeded` | `community-results/api-example.json` |

此步骤无需连接机器人。安装或运行遇到问题，先按[安装指南的故障表](INSTALL.zh-CN.md#常见问题)处理。

## 5. 第一次调用规划接口（UR3e reference）

本节六轴 API 示例不作为 RM75 验收依据。RM75 十五关节使用
[模型接口与规划说明](MOBILE_DUAL_PITCH.zh-CN.md#直接调用运动学模型)。

下面的程序接收六关节起点和终点，返回 UR3e 几何路径。角度使用 rad，长度使用 m。
在仓库根目录保存脚本并使用已激活的仿真 Python 运行：

```python
from community.api import ManipulationClient

with ManipulationClient() as client:
    task_id = client.plan_joint_motion(
        start=[0.0, -0.7, 0.9, 0.3, 0.6, 0.0],
        goal=[0.45, -0.8, 1.1, 0.45, 0.8, -0.15],
    )
    response = client.wait(task_id, timeout=120)
    if response["status"] == "succeeded":
        print(response["result"]["positions_rad"])
    else:
        print(response["error"])
```

这些数值只用于离线示例。当前结果不含执行时间、速度、加速度、实际工具与负载信息，
不直接下发至真实机器人。抓取候选、障碍物、状态与取消的完整用法见 [API 手册](API.zh-CN.md)。

`plan_grasps` 当前使用 Robotiq85 模型；Hand-E 现场配置模板不会自动改变该函数的规划模型。

## 6. 客户设备配置

RM75-B 项目按[0.2 验收记录](ACCEPTANCE_RECORD_RM75_v0.2.zh-CN.md)填写左右臂、控制器、
腰部、底盘、夹爪及标定资料；默认仿真配置不能当作客户实机参数。
以下 UR3e / PolyScope / URCap 资料仅为 **legacy/reference**，不用于判定 RM75 目标机通过。

客户自行填写机器人地址、软件版本、夹爪、工具坐标和负载；软件包不预设客户的 IP、
抓取位置或配件组合。支持准备两类常见夹爪的独立现场资料：

| 夹爪 | 配置模板 |
|---|---|
| Robotiq Hand-E | [ur3e-hande.template.json](../configs/hardware/ur3e-hande.template.json) |
| Robotiq 2F-85 | [ur3e-2f85.template.json](../configs/hardware/ur3e-2f85.template.json) |
| 尚未选择夹爪 | [ur3e-site.template.json](../configs/hardware/ur3e-site.template.json) |

模板是设备资料表，不是机器人执行文件；其具体填写方法、TCP/IP 检测命令、
官方驱动设置和首次抓放流程都在 [UR3e 与夹爪接入手册](UR3E_COMMISSIONING.zh-CN.md) 中。

## 7. 交付与验收

RM75-B 0.2 使用[新版客户验收记录](ACCEPTANCE_RECORD_RM75_v0.2.zh-CN.md)，依次区分：
离线软件验证、Navigation 实际仿真消息回放、客户现场只读连接、真机执行与现场验收。
表内包含九项示例、51 项回归、十五关节模型 / 限位、整机碰撞、导航证据、连续停稳门禁和规划输出路径。
未实施的项目填写“未测试”；没有现场证据，不填写现场通过结论。

旧[v0.1.1 验收记录](ACCEPTANCE_RECORD.zh-CN.md)保留为 legacy/reference。
UR3e、PolyScope 和原 12/13 关节模拟通信通过，不能替代 RM75 目标机验收。

软件包中的自动测试与模拟服务结果不能代填现场结果。真实抓放的成功率、周期时间和放置误差，
由项目双方在测试前约定对象、次数、测量方法和指标；本版本不预设一组适用于所有机器人的数值。

## 8. 升级、复现与问题处理

`main` 会继续更新。项目开始时保留源代码提交号、配置文件和测试结果，
在更新前先保存客户自己的程序与配置。升级方法见[安装指南](INSTALL.zh-CN.md#已有仓库升级)。

出现问题时，记录以下内容再提交至[仓库 Issues](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/issues)：
软件提交号、系统与 Python / ROS2 版本、问题所属阶段、最小复现命令、退出码与相关日志。
报告中涉及机器人地址、序列号、人员信息和现场数据时，公开发布前先删除或替换这些内容。

## 9. 许可证与来源

软件保留 WRS / UR3e 原工程来源与 MIT 许可证。详见 [LICENSE](../LICENSE) 和
[ATTRIBUTION.md](../ATTRIBUTION.md)。OriginMind 维护的 Community 新增部分也按 MIT 许可证提供。

## 带腰关节的移动双臂

默认模型为 **左臂 7 轴＋右臂 7 轴＋腰部 pitch，共 15 自由度**，直接进行运动学仿真，无需 ROS。
运行 `python -m community.mobile_dual_pitch` 打开三维动画；加 `--headless` 可无窗口检查。
参见[十五自由度配置与运行指南](MOBILE_DUAL_PITCH.zh-CN.md)。双臂已采用官方 RM75-B 运动链与外形；整机安装、腰部与夹爪参数仍需标定。
旧十三关节 UR3e 示例保留在 `community.legacy_mobile_dual_pitch`。
