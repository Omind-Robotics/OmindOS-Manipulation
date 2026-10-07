# 0.2 底盘导航与双 RM75 联动

交付时填写[RM75-B 0.2 客户验收记录](ACCEPTANCE_RECORD_RM75_v0.2.zh-CN.md)，
分别保存离线验证、实际导航消息回放、现场连接与真机执行结果及证据。

## 直接运行，不需要 ROS

```bash
python -m community.rm75_navigation
python -m community.rm75_navigation --headless \
  --output community-results/navigation.json \
  --plan-output community-results/mobile-manipulation-0.2.json
```

默认场景从 `(-1.0, -1.5, 0)` 移动到 `(4.8, 1.5, 0)`，绕过中间障碍物。
运输时腰部保持配置的 `transport_rad`，双臂保持紧凑初始姿态、夹爪不持物。
到达并停稳后，底盘保持不动，规划左臂 7 轴＋右臂 7 轴＋pitch 腰部 1 轴。

底盘坐标使用地图 `map`：米、弧度，x 前、y 左、z 上。
15 自由度指上半身关节，底盘的 `x/y/yaw` 独立保存，不混入关节数组。
底盘按可原地转向的平面运动学参考模型运动；未声明实际轮型、驱动轮数量或动力学。

编辑 `configs/simulation/rm75-navigation.reference.json` 可设置地图范围、分辨率、起终点和长方体障碍物：

```bash
python -m community.rm75_navigation --scene /path/to/scene.json --headless
```

新规划器独立实现栅格 A*，不复制 Navigation 的内部算法。它以全部运输姿态碰撞盒
（含底盘、肩部、手臂和夹爪）的 XY 投影最大半径加 5 cm 作为包络；当前参考值约 0.791 m。
占用格、未知格和地图边界均保守处理，禁止斜穿格角；每条路径段及整机姿态重新检查。
导航路径不存在、起终点过近障碍物或整机碰撞时，以非零状态退出。

## 使用 Navigation 仓库的实际发布版

来源：[OmindOS-Navigation-Community](https://github.com/maganrobotics-boop/OmindOS-Navigation-Community)，
接口核对版本 `ab456fdde986e7f3904d47fcfc87dc944a6bb719`；运行时固定为 `v0.1` 的
`omindos/navigation:0.1` Linux amd64 镜像。

| 接口 | 消息 | 适配用途 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | 一次发布目标 XY，frame 为 map |
| `/localization/pose` | `geometry_msgs/PoseStamped` | 实际估计位置和朝向 |
| `/plan` | `nav_msgs/Path` | 目标发布后的路径与终点匹配 |
| `/cmd_vel` | `geometry_msgs/Twist` | 六个分量均接近零的停车证据 |
| `/map` | `nav_msgs/OccupancyGrid` | 回放中的整机净空检查 |

`/map` 和 `/plan` 使用 reliable、transient_local、depth 1。
这些是真实发布版 Topic，**不是 Nav2 `NavigateToPose` Action**。
旧 `community.ros2_bridge` 仍是独立的模拟 Action 教学例子。

需要 Docker 的步骤只用于运行可选导航后端；宿主仿真仍使用 Python 3.12，无需安装 ROS：

```bash
curl -fL --retry 3 \
  https://github.com/maganrobotics-boop/OmindOS-Navigation-Community/releases/download/v0.1/omindos-navigation-v0.1-runtime-amd64.tar.gz \
  -o navigation-v0.1.tar.gz
echo '9a2fbed8baeb53b33586c5f76674dead3a6b29d3973fe508f4928e3928f9f491  navigation-v0.1.tar.gz' | sha256sum -c -
docker load -i navigation-v0.1.tar.gz
mkdir -p community-results/navigation
docker run --rm --network none \
  -v "$PWD:/community:ro" -v "$PWD/community-results/navigation:/artifacts" \
  --entrypoint /bin/bash omindos/navigation:0.1 /community/examples/navigation_runtime.sh
python -m community.rm75_navigation --headless \
  --navigation-log community-results/navigation/navigation-trace.json \
  --output community-results/navigation-integration.json \
  --plan-output community-results/navigation-integration-plan.json
```

去掉最后一步的 `--headless`，即可看实际估计底盘轨迹和随后的机械臂运动。
容器默认演示目标 `(-1.6, -2.5)`，位于发布版地图的开阔区域。
可在容器命令最后加 `--goal X Y`。底盘路径必须适合整机包络，原导航节点的较小规划半径不代表双臂整机一定可通行。
运行脚本省略会反复覆盖目标的 `demo_supervisor`，退出时结束该次模拟导航进程。
不要把 Python 3.10 的 ROS 包装入宿主的 Python 3.12 虚拟环境。

## 停车与失败语义

顺序为 `idle → navigating → ready → manipulating → succeeded`。
适配器要求 map 坐标、目标发布后的匹配路径、最近 0.5 秒内的定位和速度、
位置误差不超过 0.25 m、速度六分量绝对值不超过 0.001，且连续至少 0.6 秒位置漂移
不超过 0.01 m、朝向漂移不超过 0.02 rad。观测中断会重新计时。
保留实际最终朝向；不将用户目标 yaw 强行赋给机器人。

JSON 导入再次验证证据、单调时间、数据间断、非有限数和异常位姿跳变。
随后按最多 0.025 m / 0.025 rad 间隔检查底盘运动，并在终点固定底盘规划关节。
失败记录不会进入机械臂计算；输出路径会先清理旧结果，防止误读上次成功文件。
记录中的时间仅描述模拟导航采集，不构成速度规划后的机械臂控制轨迹。

发布版没有任务 ID、接受确认、取消和可靠抢占；无路时可能继续保留旧路径，输入过期也不保证底盘自动停车。
本适配器因此仅用于隔离仿真与离线回放，不发送真机机械臂指令，门禁不等于物理安全停机。
栅格图是二维信息，回放保守拉伸为 2 m 高障碍柱；不提供三维感知或动态障碍预测。
