# Git 与 Codex 并行开发工作流

本文说明本项目后续如何建立 GitHub 远端，以及如何让多个 Codex 会话并行工作而不互相覆盖。

## 当前集成合同

- 科目二优先，当前传感器链为 Horizon PointCloud2 + LIO，
  `ScanRegistration.msg_type=1`；
- 车辆是两侧独立驱动的差速履带底盘；
- 科目二临时发布 identity `map→odom`，仅用于坐标已对齐的封闭场路径；
- 科目二最终输出是 `/cmd_vel` `geometry_msgs/msg/Twist`，单位为 m/s 与 rad/s；
- 科目一已实现机体系最小闭环：其他团队提供
  `/subject1/nominal_cmd_vel` 与 `/subject1/next_waypoint_base`，本仓库在同一
  timer 内原子选择巡航或避障并唯一发布 `/cmd_vel`；
- 科目一的 `map_odom_manager` 初始无有效变换；合作方全局位姿语义确认前
  不发布 `map→odom`，局部闭环只依赖 `base_link` 输入；
- 上游来源/传输/部署仍未知，禁止假定。

跨包改动必须先核对 [`ARCHITECTURE.md`](ARCHITECTURE.md) 和
[`INTERFACES.md`](INTERFACES.md)，不得让并行分支重新引入已废弃的传感器、自动起点对齐或旧最终命令 topic/type 作为当前科目二合同。

## 仓库边界

代码仓库固定为：

```text
/home/l1u/labs/越野车规划控制/ugv_planning_control
```

不要把上一级 `越野车规划控制/` 直接作为 Git 仓库。上一级目录包含历史任务、原始材料和外部参考工程；把它们全部纳入代码仓库会：

- 混淆“产品代码”和“只读参考资料”的边界；
- 扩大 Git 变更、搜索结果和 Codex 会话上下文；
- 增加误改雷达驱动、LIO 工程或历史文档的风险；
- 让构建产物、测试证据和版本标签难以对应到一份明确源码。

需要历史信息时，按 `docs/REFERENCE_MATERIAL.md` 指向的位置只读查阅。不要复制或 vendor `livox_ros2_driver`、`LIO_Livox_ROS2`。

## 当前 Git 状态

- 本目录已经初始化为 Git 仓库；
- 集成分支为 `main`；
- 当前没有配置 GitHub remote；
- 在确认 GitHub owner 和仓库 visibility 前，不创建远端。

以下命令可复核本地状态：

```bash
cd /home/l1u/labs/越野车规划控制/ugv_planning_control
git rev-parse --show-toplevel
git branch --show-current
git remote -v
```

`git remote -v` 当前应无输出。

## 确认 owner 和 visibility 后创建 GitHub 仓库

先确认两项信息：

1. `<OWNER>`：GitHub 用户名或组织名；
2. visibility：`--private`、`--internal` 或 `--public`。

本项目包含现场集成信息，默认建议使用私有仓库：

```bash
cd /home/l1u/labs/越野车规划控制/ugv_planning_control
gh auth status
gh repo create <OWNER>/ugv_planning_control \
  --private \
  --source=. \
  --remote=origin \
  --push
git remote -v
git branch -vv
```

如 owner 要求组织内部可见或公开，将 `--private` 替换为 `--internal` 或 `--public`。上述创建命令尚未执行，因为 owner 和 visibility 仍未确认；其参数已按本机 `gh repo create --help` 核对。

## 一个 Codex 会话对应一个 worktree

不要在同一工作目录同时运行多个可写 Codex 会话。每个会话使用独立分支和独立 worktree，并且只拥有一个 package 及其对应的 `workstreams/` 目录。

例如，为 Subject 2 调参任务创建工作区：

```bash
cd /home/l1u/labs/越野车规划控制/ugv_planning_control
mkdir -p ../ugv_worktrees
git worktree add \
  -b feat/subject2-tuning \
  ../ugv_worktrees/subject2-tuning \
  main
cd ../ugv_worktrees/subject2-tuning
git status --short --branch
```

之后从 `../ugv_worktrees/subject2-tuning` 启动该 Codex 会话。任务完成并合并后，由集成负责人清理：

```bash
cd /home/l1u/labs/越野车规划控制/ugv_planning_control
git worktree remove ../ugv_worktrees/subject2-tuning
git branch -d feat/subject2-tuning
```

只有在分支已经合并且工作区无未提交变更时执行清理。

## 所有权划分

并行会话必须使用互斥写入范围：

| 会话 | 唯一写入范围 |
| --- | --- |
| 定位/TF | `src/ugv_localization_mvp/**`、`workstreams/01_localization/**` |
| 科目二控制 | `src/ugv_subject2_mvp/**`、`workstreams/02_subject2_control/**` |
| 科目一感知 | `src/ugv_subject1_perception_mvp/**`、`workstreams/03_subject1_perception/**` |
| 科目一避障 | `src/ugv_subject1_avoidance_mvp/**`、`workstreams/04_subject1_avoidance/**` |
| GPS/LIO 恢复 | `workstreams/05_gps_lio_recovery/**`；接口确认前不实现自动恢复 |
| RDK 测试调参 | `workstreams/06_rdk_test_tuning/**` 和任务明确授权的测试记录 |
| 开发 fixture | `src/ugv_mvp_tools/**`、`workstreams/07_dev_tools/**` |
| bringup | `src/ugv_mvp_bringup/**`、`workstreams/08_bringup/**` |
| 集成负责人 | 根目录治理、`docs/**`、`.github/**`、`scripts/**` 和跨包接口 |

一个会话不得顺手修改另一个会话拥有的 package。发现跨包问题时，记录复现步骤和建议修复位置，交给对应 owner 或集成负责人。

## 分支交付与合并

每个分支在交付前：

1. 先运行所属 package 的窄测试；
2. 再运行任务要求的完整构建、测试或 RDK smoke；
3. 按根目录 `AGENTS.md` 的 Lore 格式提交；
4. 报告 commit、修改文件、验证命令和仍未验证的实车项。

集成负责人按以下顺序合并：

1. 定位与 TF；
2. 科目二控制；
3. 科目二 bringup 与 RDK 证据；
4. 科目一感知；
5. 科目一避障；
6. 只有在接口确认后才合并 GPS/LIO 自动恢复。

示例合并流程：

```bash
cd /home/l1u/labs/越野车规划控制/ugv_planning_control
git switch main
git status --short
git merge --no-ff feat/subject2-tuning
python3 scripts/verify_repository.py
```

存在 GitHub remote 后，在合并前执行 `git pull --ff-only origin main`，合并并验证后执行 `git push origin main`。不要从未验证的多个分支一次性拼接后直接部署 RDK。

## 后续任务优先级

### P0：先让科目二具备实车低速测试条件

- 回填并审批 Horizon 的 `base_link→livox_frame` 独立 6DoF、误差和测量记录编号；
- 回填车辆速度、偏航角速度、加减速度和首次联调安全速度；
- 解决 RDK 上雷达驱动/LIO 重力缩放问题，确认实际 LIO 参数；
- 用真实 Horizon PointCloud2/LIO `msg_type=1` 话题验证 canonical odom、identity `map→odom` 和 TF，执行器保持断开；
- 使用人工复核的封闭场局部路径，确认其原点/轴向与临时 identity `map→odom` 假设一致；未知上游数据不得用于实车；
- 依次完成直线、左/右弧线和 odom 断流/时间戳/跳变停车测试。

### P1：科目一真实点云与最终命令联调

- 回填并审批 Horizon 的独立静态外参、车辆 footprint 和安全膨胀；
- 采集真实 Horizon PointCloud2 bag，验证机体轴向、地面高度带和自车裁剪；
- 用已知前/左/右软障碍调节点云栅格与曲率候选参数；
- 由其他团队持续发布平面巡航命令 `/subject1/nominal_cmd_vel` 和已经转换到
  `base_link` 的 `/subject1/next_waypoint_base`；
- 验证无障碍透传、障碍安全轨迹接管、障碍解除恢复透传，以及所有
  blocked/stale/invalid 分支最终 `/cmd_vel` 为零；
- 确认科目一运行图中只有本仓库的 local avoidance selector 发布 `/cmd_vel`；
- 先断开执行器验证命令方向，再进行批准速度下的封闭场地测试。

### P2：接口确认后的集成

- 只有拿到 GPS/global pose 的话题、消息、坐标系、时间和有效性定义后，才实现 GPS 辅助恢复；
- 只有拿到明确的 LIO supervisor 重启接口和重启后有效判据后，才实现自动重启；
- 只有上游程序及部署形式稳定后，才开发数据桥接包；
- 下游接收 `/cmd_vel` `geometry_msgs/msg/Twist`：`linear.x` m/s、`angular.z` rad/s；左右履带执行器换算和独立 watchdog 由下游团队负责。

阻塞项的完整清单见 `docs/KNOWN_GAPS.md`。

## Codex 会话提示词模板

为每个 worktree 新开 Codex 会话时，使用下面的模板，并替换尖括号内容：

```text
你在 <WORKTREE_ABSOLUTE_PATH> 工作。

先读：
1. AGENTS.md
2. <WORKSTREAM_PATH>/SESSION_ROADMAP.md
3. 与本任务直接相关的 docs 文件

目标：
<ONE_CONCRETE_RESULT>

唯一允许写入：
- <OWNED_PACKAGE_OR_DOC_PATH>
- <WORKSTREAM_PATH>

禁止：
- 不修改 livox_ros2_driver 或 LIO_Livox_ROS2
- 不修改其他会话拥有的 package
- 不假定未知上游、GPS、LIO 重启或车辆参数
- 不删除 Subject 1 的 active/candidate/trajectory 诊断接口，不引入第二个
  `/cmd_vel` 发布者，不在全局位姿合同未知时为 Subject 1 猜测 `map→odom`
- 不用 fixture/bag/unit test 结果宣称实车通过

验收：
- <TARGETED_TEST>
- <INTEGRATION_OR_RDK_CHECK>
- 报告 commit、修改文件、实际输出和未验证项
- 提交信息遵守 AGENTS.md 的 Lore Commit Protocol

遇到跨包问题时只记录证据并交给集成负责人，不扩大写入范围。
```

每个提示词只给一个明确结果和一组可执行验收条件。不要让多个廉价模型同时“完善整个系统”。
