# 科目二最小路径跟踪：后续会话路线图

## 本会话已经交付

`ugv_subject2_mvp` 是一个无 Nav2 运行时依赖的 ROS 2 Humble C++17 包。当前
P0 目标是 Horizon + LIO 定位的两轮差速履带车：

- 输入 `/localization/trusted_odom`、`/localization/odom_valid`、`/subject2/path`。
- 通过 TF 把 odom 当前位姿转换到 Path 坐标系；当前明确假设物理起点与全局
  路径起点一致，因此定位包发布 identity `map→odom`。
- 以 20 Hz 发布 `/cmd_vel`，消息类型为 `geometry_msgs/msg/Twist`。
  `linear.x` 表示车体中心沿 `base_link +x` 的前进速度，单位 m/s；
  `angular.z` 表示绕 `base_link +z` 的偏航角速度，单位 rad/s，左转为正。
  其他 Twist 分量必须为零。履带左右轮速换算、油门、转向和执行器安全由下游
  团队适配，本仓库不实现。
- 发布 `/subject2/target_point` 供 RViz 和调参观察。
- 采用路径最近进度、固定/速度缩放前视、`κ=2y/L²`、曲率/速度/角速度限幅和终点减速；前视点落在车后时保持零命令。
- odom/path/valid 超时、invalid、空路径、非有限数或 TF 失败时，每个控制周期持续发布零命令。

本包刻意不实现 GPS、LIO 重启、上游 gateway、车辆执行器适配或复杂状态机。

`/subject2/path` 是本仓库内部控制器的标准输入，不是对上游团队实现形式的
承诺。上游实际能提供文件、网络报文、函数调用还是其他数据尚未确认；在拿到
真实程序/样例之前，不得提前开发桥接包，也不得让核心控制器依赖猜测出来的接口。

## 便宜模型可直接执行的任务（按优先级）

只修改 `src/ugv_subject2_mvp/**` 和本目录。开始前阅读根 `AGENTS.md`、
`docs/ARCHITECTURE.md`、`docs/INTERFACES.md`。不要引入 Nav2。

### P0-A：保持软件回归全绿

不改算法，只在当前源码上执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_subject2_mvp --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select ugv_subject2_mvp --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2
ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_fault
ROS_DOMAIN_ID=<unused> scripts/run_fixture_smoke.sh subject2_jump
```

验收：全部命令成功，nominal 模式 `/cmd_vel` 非零，stale/jump 后持续为零且
last-good snapshot 与故障前最后可信 odom 一致。

### P0-B：用真车 Horizon/LIO bag 调参

一次只改一个参数组；保存 bag、参数 YAML、Git commit 和误差统计。不得使用
合成 fixture 结论代替真车效果。验收至少记录：

```bash
ros2 topic hz /livox_odometry_mapped
ros2 topic hz /localization/trusted_odom
ros2 topic hz /cmd_vel
ros2 topic echo --once /cmd_vel
ros2 run tf2_ros tf2_echo map odom
```

同时给出最大/均方横向误差、终点误差、最大角速度、停车距离和人工急停记录。

### P1：上游确认后的最薄适配

只有拿到上游真实输出样例与更新/结束语义后才开始。新适配器只负责转换为
`nav_msgs/msg/Path` 并发布 `/subject2/path`；不得把解析逻辑塞入控制器。
验收需包含真实样例解析测试、畸形输入 fail-closed 测试以及：

```bash
ros2 topic echo --once /subject2/path
ros2 topic hz /subject2/path
```

### 暂缓：GPS/LIO 恢复

接口未确认前不开发。odom fault 后唯一正确行为是保持 `/cmd_vel` 为零。

## 调参顺序（先低速直线，再弯道）

1. **安全限幅**：从 `nominal_speed=0.1`、`max_speed=0.2`、
   `max_yaw_rate=0.3` 开始；按车辆团队给出的硬限制逐步增加。
2. **固定前视**：先令 `use_speed_scaled_lookahead=false`，在直线与缓弯调
   `lookahead_distance`。振荡则增大，切弯过慢则减小。
3. **速度缩放前视**：打开 `use_speed_scaled_lookahead`，调整
   `lookahead_speed_gain/min_lookahead/max_lookahead`。
4. **曲率限制**：根据最小转弯半径设置 `max_curvature≈1/R_min`，再限制
   `max_yaw_rate`。不要用参数掩盖坐标轴或 TF 错误。
5. **终点**：调整 `slowdown_distance` 与 `goal_tolerance`；检查接近终点时速度单调下降，
   且车辆真实停车距离不越界。
6. **超时**：按实测发布周期和 99 百分位抖动设置
   `odom_timeout_sec/path_timeout_sec/valid_timeout_sec`，但 invalid 必须立即停车。

每次真车测试保存参数 YAML、rosbag、Git commit、场地/载荷、最大横向误差、停车距离和操作者结论。

## 本地/RDK 构建

仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_subject2_mvp --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select ugv_subject2_mvp
colcon test-result --verbose
```

RDK 使用专用工作区，不覆盖已安装的 Livox/LIO 工作区。同步仓库后运行同样命令。

## 无执行器测试方法

1. 启动定位包，确认 TF 链 `map→odom→base_link` 唯一且连续。
2. 启动本节点并发布小型 `nav_msgs/Path`。
3. 用 `ros2 topic hz /cmd_vel` 确认约 20 Hz；用 `echo` 检查只有
   `linear.x/angular.z` 非零。
4. 直线路径应 `angular.z≈0`；左/右弯符号分别为正/负。
5. 停止 trusted odom、发布 `odom_valid=false`、删除 TF、停止 path 发布，分别确认下一个周期起持续零。
6. bag 回放只证明软件行为，不证明真车安全或闭环效果。

## 上车测试门槛

- 车辆团队确认速度、角速度、曲率、减速度与停车距离上限。
- 实测并审核 Horizon 静态 TF；TF authority 无冲突。
- 下游适配器具备独立硬件急停/看门狗，并先架空轮或断开动力验证符号。
- 首次测试清空场地、限速、人工急停就位；先直线，再大半径弯，最后完整路径。

## 尚缺、不得猜测的恢复接口

后续恢复会话必须先获得：

- GPS 输出类型、坐标基准、完整位置和航向有效性/时间戳语义。
- LIO 启动、停止、健康检查、重启完成判据和超时。
- 重启后 raw LIO 世界坐标系的重置行为。
- 利用“最后可信 odom + GPS 全局位姿”更新 `map→odom` 的明确数学约定。
- 恢复成功后谁调用 `/localization/reset_odom_fault`，以及失败时退赛/保持停车的责任边界。

这些接口确认前，odom fault 后的唯一正确行为是保持零命令。
