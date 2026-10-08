# 开发源码安装与运行指南

本页仅供有私有开发仓读取权限的团队成员。客户请使用
[公开发行仓的安装说明](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release)；完整运行 ZIP 不依赖旧 Community 仓。

0.2 新增底盘导航、到达停稳门禁和双 RM75＋腰部联动，入口：`python -m community.rm75_navigation`。
详见[导航联动指南](NAVIGATION_v0.2.zh-CN.md)和[0.2 发布说明](RELEASE_v0.2.md)。

适用版本：OmindOS Manipulation 0.2。旧 `v0.1` 标签不包含新增统一 API。

## 选择环境

| 用途 | 参考环境 | 是否需要机器人 |
|---|---|---|
| 几何规划、九项示例、统一 API | Ubuntu 24.04 x86_64，Python 3.12，独立虚拟环境 | 不需要 |
| ROS2 Action 通信示例 | Ubuntu 22.04，ROS2 Humble，ROS2 对应的系统 Python | 不需要，服务为模拟实现 |
| 图形展示 | 仿真环境加可用的桌面与 OpenGL | 不需要 |
| 真机操作 | 设备驱动与现场集成环境 | 需要，尚未验收 |

Windows、macOS、ARM 平台尚未作为此版验收环境。使用 Windows 电脑的用户可先准备上述 Ubuntu 环境；
WSL 或虚拟机尤其需要另行检查图形显示，不能直接视为已验证。

## 全新安装

打开 Ubuntu 24.04 终端，执行：

```bash
sudo apt-get update
sudo apt-get install -y git python3.12-venv libgl1 libgomp1

git clone --branch main --single-branch https://github.com/maganrobotics-boop/OmindOS-Manipulation.git
cd OmindOS-Manipulation
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-community.txt
python -m pip check
```

Open3D 等依赖体积较大，请为完整源码、虚拟环境和安装缓存预留数 GB 空间。
使用 `requirements-community.txt`，原 `requirements.txt` 属于历史应用环境。
无需为 Community 示例安装 QPanda3D 或 PyQt5。

## 第一次验证

```bash
python -m community.verify
python -m unittest community.test_contract community.test_api community.test_pitch community.test_dual7 community.test_rm75 community.test_navigation
python -m community.api_example
```

预期：九项示例全部显示 `passed`；单元测试显示 `OK`；API 例子中的 `motion` 和 `grasp` 均显示 `succeeded`。
如果失败，查看 `community-results/` 中对应的 JSON 和日志。API 例子的错误详情在 `api-example.json` 中。

验证报告不代表真机动作完成。程序退出码为 0 表示检查成功，非 0 表示失败。

## 打开窗口

在有图形桌面的终端运行：

```bash
python -m community.ur3e_single_arm
python -m community.grasping
```

关闭窗口后程序退出。通过 SSH 或在没有显示器的环境中，请加 `--headless`。
API 默认始终使用无窗口计算；需要观看动画时运行相应示例。

## 每次重新进入

```bash
cd OmindOS-Manipulation
source .venv/bin/activate
python -m community.api quick_start --output community-results/api-cli.json
```

所有命令从仓库根目录运行。源码模型和包结构是运行所需内容，不要只复制一个 `.py` 文件。
离开环境执行 `deactivate`。

## 已有仓库升级

旧 Community 与新开发仓采用不同历史基线。保留旧目录和自己的修改，不要直接改远程地址后合并，也不要强制覆盖。
在旧目录执行 `git status` 并保存本地工作后，在旁边新建开发仓克隆：

```bash
cd ..
git clone --branch main --single-branch https://github.com/maganrobotics-boop/OmindOS-Manipulation.git OmindOS-Manipulation-dev
cd OmindOS-Manipulation-dev
```

然后按上方“全新安装”创建虚拟环境和安装依赖。逐项迁移自己的代码、URDF 和参数并重新验证。
没有开发仓权限的客户直接下载公开发行包，不需要申请旧仓权限。
已是新开发仓的干净工作区可执行 `git pull --ff-only`；若失败，先检查本地修改，不强制覆盖。
旧仓 `v0.1`、`v0.2` 标签和发行资产保留为历史记录，仅授权用户可访问。
保存 `git rev-parse HEAD` 的结果用于复现。详见[迁移说明](REPOSITORY_MIGRATION.zh-CN.md)。

## ROS2 通信运行

仿真环境先导出已检查的示例轨迹：

```bash
python -m community.integrated_task --headless --plan-output community-results/ros2-playback.json
```

将完整仓库和该 JSON 准备到 ROS2 Humble 环境。在已安装 ROS2 的 Ubuntu 22.04 中，
退出仿真虚拟环境并执行：

```bash
sudo apt-get install -y ros-humble-nav2-msgs ros-humble-control-msgs
source /opt/ros/humble/setup.bash
python3 -m community.ros2_verify --plan community-results/ros2-playback.json
```

这会检查成功、导航失败阻止机械臂、机械臂失败反馈三个同机通信场景。
详细说明见 [ROS2_INTERFACE.md](ROS2_INTERFACE.md)。
Ubuntu 22.04 / Humble 的 `rclpy` 使用匹配的系统 Python；不要与 Python 3.12 仿真环境混装。

## 设备连接检查

UR3e 连接、软件版本读取、官方驱动与两款夹爪配置，见
[UR3e 与夹爪接入手册](UR3E_COMMISSIONING.zh-CN.md)。
TCP/IP 版本诊断仅需 Python 标准库；ROS2 观察工具使用上述独立 ROS2 环境。

## 常见问题

| 现象 | 处理 |
|---|---|
| 找不到 `community` 或 `basis` | 回到仓库根目录，确认下载完整源码 |
| 找不到 `community.api` | 检查是否仍处于旧 `v0.1`；新增 API 在主线修订中 |
| 找不到 STL / DAE 模型 | 重新获取完整仓库，不要仅复制 Python 文件 |
| Panda3D 无法创建窗口 | 在图形桌面运行；仅检查计算可使用 `--headless` |
| NumPy 或 `pkg_resources` 错误 | 新建虚拟环境，按固定依赖安装，避免混用历史环境 |
| `No module named rclpy` | 在独立 ROS2 终端加载 `setup.bash` 并使用其系统 Python |
| 规划任务 `failed` | 读取 `error.details`；检查限位、障碍物、依赖和路径是否存在 |
| 网络下载失败 | 检查 GitHub 与 Python 包源连接；源码依赖仍需下载；客户可用公开发行仓的封装运行包，系统库另行准备 |

固定直接依赖和历史环境记录见 [ENVIRONMENT.md](ENVIRONMENT.md)。完整间接依赖并非跨平台锁文件。
