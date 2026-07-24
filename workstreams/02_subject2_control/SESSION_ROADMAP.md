# 科目二最小路径跟踪：后续会话路线图

## 本会话已经交付

`ugv_subject2_mvp` 是一个无 Nav2 运行时依赖的 ROS 2 Humble C++17 包：

- 输入 `/localization/trusted_odom`、`/localization/odom_valid`、`/subject2/path`。
- 通过 TF 把 odom 当前位姿转换到 Path 坐标系；`map→odom` 由定位包负责。
- 以 20 Hz 发布 `/control/cmd_vel`，仅使用 `linear.x` 与 `angular.z`。
- 发布 `/subject2/target_point` 供 RViz 和调参观察。
- 采用路径最近进度、固定/速度缩放前视、`κ=2y/L²`、曲率/速度/角速度限幅和终点减速。
- odom/path/valid 超时、invalid、空路径、非有限数或 TF 失败时，每个控制周期持续发布零命令。

本包刻意不实现 GPS、LIO 重启、上游 gateway、车辆执行器适配或复杂状态机。

## 便宜模型下一会话的工作边界

只修改 `src/ugv_subject2_mvp/**` 和本目录。开始前阅读根 `AGENTS.md`、
`docs/ARCHITECTURE.md`、`docs/INTERFACES.md`。不要引入 Nav2。

建议提示词：

```text
对 ugv_subject2_mvp 做一次真车数据驱动的小步调参/修复。先读取最新 rosbag 和测试记录，
一次只修改一个参数组或一个明确缺陷；运行包级 build/test 后再提交。不得实现尚未确认的
GPS/LIO 恢复接口，不得把仿真或 bag 回放称为真车验证。
```

## 调参顺序（先低速直线，再弯道）

1. **安全限幅**：从 `nominal_speed=0.2`、`max_speed=0.3`、
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
3. 用 `ros2 topic hz /control/cmd_vel` 确认约 20 Hz；用 `echo` 检查只有
   `linear.x/angular.z` 非零。
4. 直线路径应 `angular.z≈0`；左/右弯符号分别为正/负。
5. 停止 trusted odom、发布 `odom_valid=false`、删除 TF、停止 path 发布，分别确认下一个周期起持续零。
6. bag 回放只证明软件行为，不证明真车安全或闭环效果。

## 上车测试门槛

- 车辆团队确认速度、角速度、曲率、减速度与停车距离上限。
- 实测并审核 Avia 静态 TF；TF authority 无冲突。
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
