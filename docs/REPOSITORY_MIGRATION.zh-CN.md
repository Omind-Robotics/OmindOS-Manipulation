# Manipulation 三仓迁移说明

2026-10-08。当前维护与安装入口按下表区分。

| 仓库 | 用途与访问方式 |
|---|---|
| [OmindOS-Manipulation-Release](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release) | 公开客户发行入口：完整运行 ZIP、SHA256、安装文档与离线验证记录 |
| [OmindOS-Manipulation](https://github.com/maganrobotics-boop/OmindOS-Manipulation) | 私有开发源码：仅授权团队使用，后续开发基线为 `main` |
| OmindOS-Manipulation-Community | 历史仓：已设为私有并归档；保留提交、分支、标签、Release 及其资产 |

## 客户安装

从公开发行仓 [v0.2-runtime.1](https://github.com/maganrobotics-boop/OmindOS-Manipulation-Release/releases/tag/v0.2-runtime.1)
下载完整 ZIP 和 `SHA256SUMS-Customer-Release`，按其 README 校验、解压、安装并运行。
无需克隆、登录或读取任何私有源码仓。GitHub 自动生成的 Source code 仅是公开文档快照，不能替代运行包。

本次从公开下载 URL 重新获取 290,667,300 字节 ZIP，SHA256 为
`3cb2895821526cb0f1d9816b5f3fb7f8e89ba664859f74086838f6bdc569b770`。
ZIP、客户端和运行包的校验均通过。在源码目录之外解压，执行 `install.sh`，再以系统 Python、
清除 `PYTHONPATH` 并使用 `-I` 运行客户 API 示例，结果为 `status: passed`、15 关节、570 个路径采样。
阶段为 `idle → navigating → ready → manipulating → succeeded`，底盘终点约为 `[4.8, 1.5, 0]`。
包内安装脚本、SDK、示例和文档未发现旧仓下载或克隆依赖。

此核验为 Linux x86_64 / glibc 2.39 的离线安装与规划检查；`hardware: not tested` 保持不变。
不表示完成真机执行、标定、动力学或其他操作系统验收。现有 tag 与发行资产不重打包、不覆盖。

## 开发者迁移

开发仓以独立源码快照起步，与旧 Community 历史不同。保存旧工作区，再在相邻目录重新克隆
私有开发仓；不要将旧工作区直接改远程后强行合并。具体命令见[源码安装指南](INSTALL.zh-CN.md#已有仓库升级)。
保持 Python 模块名 `community` 和 `requirements-community.txt`，以免破坏程序导入兼容性。

开发仓工作流均检出当前仓库。`customer-runtime.yml` 构建候选并验证；历史 `release-v02` 发布任务
仅允许原 Community 仓执行，避免复制的工作流在开发仓重新生成旧 `v0.2`。
可选 Navigation 运行包仍来自独立公开的 Navigation 仓；客户默认离线示例不依赖 ROS 或该运行包。

## 历史记录

旧仓 `v0.1` 指向 `8e57470696be60bfef3f018384d2f4a155b794d8`；
旧仓 `v0.2` 指向 `6726ddfdf84bf9eb44a36671566f5199856cbe32`。
历史 Release/CI 引用仍指向真实旧仓，不能将其 URL 机械替换成新仓而伪造迁移后的运行记录。
旧的 `docs/validation/github-actions.json` 保存当时仓库名与原始验证信息，不改写历史证据。
这些记录在私有化后需要旧仓权限，不作为客户安装前置条件。

GitHub Fork 需要先使用 “Leave fork network” 成为独立仓库，才能修改为私有；这会永久解除上游关系。
归档用于保留历史并停止写入，不执行仓库删除、标签重写或资产删除。

## 完成复核（2026-10-08，北京时间）

- 公开发行仓：`private=false`、`archived=false`；私有开发仓：`private=true`、`archived=false`；旧 Community：`private=true`、`archived=true`、`fork=false`。
- 旧仓保留原 repository ID `1406302408`；归档前后 11 个分支的提交号、2 个 tag 的目标提交均完全一致，`main` 仍为 `a0672c645dcd9db7db9abce6ca4ccd1b19f9ea22`。
- 旧仓 v0.1 / v0.2 两条 Release 的 ID 与资产清单不变；v0.2 两个资产的 ID、字节数、SHA256 均不变。公开发行仓 runtime.1 的资产也未修改。
- 私有化后匿名检查发行仓首页、runtime.1 发布页、完整 ZIP 和外层 SHA256 文件，均返回 HTTP 200。
- Navigation 公开首页的旧安装入口已替换为公开发行仓（提交 `67d6f6b99655ef7516e34208ef1438441c3f7ae1`）。已核对官网中英文 OmindOS 与双臂产品入口源码，未发现旧仓链接。
- 开发仓迁移提交 `4efe0eb0fdafe59c6f2c603cbba2f14612569f7b`：`Customer runtime package`（run `37706588521`）与 `Community verification`（run `37706588442`）均成功。
- 本次没有执行真机运动，`hardware: not tested` 继续保留。
