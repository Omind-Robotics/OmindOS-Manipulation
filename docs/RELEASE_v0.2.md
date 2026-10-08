# OmindOS Manipulation 0.2

> 历史记录说明：本页指向旧 Community 的 Release、tag 或 CI 链接用于追溯，私有化后需要旧仓权限；不作为客户安装依赖。当前客户入口为[公开发行仓](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release)，参见[迁移说明](REPOSITORY_MIGRATION.zh-CN.md)。

底盘导航＋双睿尔曼 RM75-B＋pitch 腰部的联动运动学仿真。

- 左臂 7 轴、右臂 7 轴、腰部 1 轴，共 15 个上半身关节；底盘另外使用 `x / y / yaw` 三个平面坐标。
- 新增 `community.rm75_navigation`：栅格 A* 绕障、完整运输姿态包络、平面运动学、停车门禁、固定底盘后的十五关节规划。
- 默认仿真只需 Python，不需要 ROS。配置、地图、底盘记录、关节采样和阶段历史可导出 JSON。
- 可选对接已发布的 `omindos/navigation:0.1`：真实 `/goal_pose`、`/plan`、`/localization/pose`、`/cmd_vel`、`/map` 消息采集；导航记录回放通过整机碰撞检查后，再计算双臂和腰部动作。
- 到达确认要求目标匹配、发布目标后的路径、新鲜定位与速度、连续停稳；失败、无路径、错误坐标系或过期记录不会放行机械臂。
- 保留 UR3e 示例、统一 Python API 和旧 12/13 关节 ROS2 模拟协议；原 v0.1 发布不变。

## 开始运行

Ubuntu 24.04 x86_64 / Python 3.12，在仓库目录执行：

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-community.txt
python -m community.rm75_navigation
```

无图形桌面时增加 `--headless`。完整安装与可选导航对接见
[导航联动指南](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/blob/v0.2/docs/NAVIGATION_v0.2.zh-CN.md) 和 [安装说明](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Community/blob/v0.2/docs/INSTALL.zh-CN.md)。

## 验证与下载

发布由 `main` 上的 Community verification 工作流生成，必须先通过：

1. 51 项回归检查、九个无窗口示例及 Python API 调用。
2. 原 12/13 关节 ROS2 模拟消息传输回归。
3. 固定 SHA256 的 Navigation v0.1 发布镜像，在禁用外网的容器内运行实际导航节点和消息采集。
4. 实际导航轨迹导入 RM75 整机、碰撞检查和导航后十五关节规划。

源代码由本发布的 GitHub Source code 下载提供。`OmindOS-Manipulation-0.2-validation.zip`
包含本次工作流报告、导航记录和离线联动采样；另附 SHA256 校验文件。

## 范围

这是运动学与采样碰撞验证，不包含真机运动控制、动力学、接触、轮胎打滑或整机稳定性验证。
RM75-B 运动链和外形来自厂家模型；底盘、肩部安装、腰部及夹爪仍为未标定参考配置。
Navigation v0.1 实际发布的是 `nav3d_native` Python 节点，没有 Nav2 Action、任务 ID 或可靠取消接口；本次适配不会把旧模拟 Action 桥接冒充实际接口。
导航终点使用实际估计朝向，不宣称目标 yaw 已受控。导航镜像的示例建图使用模拟真值。
完整运输包络比导航运行时自身的规划半径更大，狭窄路线会被回放校验拒绝。
停车门禁只决定是否进入离线机械臂规划，不是硬件急停。
