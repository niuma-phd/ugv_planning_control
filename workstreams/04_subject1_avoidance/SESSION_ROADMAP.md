# 科目一机体系局部避障：会话路线图

## 1. 这个会话只做什么

只维护 `src/ugv_subject1_avoidance_mvp/` 和本目录。目标是一个容易理解、容易调参的 MVP：

- 障碍物与下一航点都已经位于 `base_link`。
- 障碍存在时，固定低速采样多条常曲率前向轨迹。
- 用膨胀后的矩形车体逐点碰撞检查，选择代价最低的安全轨迹。
- 发布候选 `v/omega` 和 `avoidance_active`，但**不读取、接管、混合或转发其他团队的巡航命令**。
- 无障碍时 `active=false` 且候选命令为零；输入超时或无安全轨迹时 `active=true` 且命令为零。

这是受 Nav2 DWB“采样—评分”结构启发的独立小实现，不依赖 Nav2/PCL，也没有复制第三方源码。

## 2. 已实现接口

输入：

- `/subject1/obstacles`，`geometry_msgs/msg/PoseArray`：障碍栅格中心，坐标系应为 `base_link`。
- `/subject1/next_waypoint_base`，`geometry_msgs/msg/PointStamped`：下一航点，坐标系应为 `base_link`。

输出：

- `/subject1/avoidance_active`，`std_msgs/msg/Bool`。
- `/subject1/avoid_cmd_vel`，`geometry_msgs/msg/TwistStamped`：只使用 `linear.x` 和 `angular.z`。
- `/subject1/selected_trajectory`，`nav_msgs/msg/Path`：调试用的选中 rollout。

当前节点按本机**接收时间**检查新鲜度，不依赖不同设备间尚未验证的时钟同步。

## 3. 算法与参数

常曲率 `k`、弧长 `s`：

```text
x = sin(k*s)/k
y = (1-cos(k*s))/k
yaw = k*s
omega = v*k
```

`k≈0` 时采用直线。评分项为终点到目标距离、末端朝向误差、曲率绝对值和最小净空；发生碰撞的候选直接丢弃。

主要参数：

| 参数 | 含义 | 默认值 |
|---|---|---:|
| `speed_mps` | 避障固定前进速度 | 0.10 |
| `max_curvature` | 最大曲率绝对值 | 1.2 1/m |
| `curvature_samples` | 曲率采样数 | 25 |
| `horizon_m` | rollout 弧长 | 3.0 m |
| `step_m` | 碰撞检查步长 | 0.10 m |
| `footprint_half_length_m` | 车体矩形半长 | 0.55 m |
| `footprint_half_width_m` | 车体矩形半宽 | 0.40 m |
| `inflation_m` | 额外安全膨胀 | 0.20 m |
| `input_timeout_s` | 任一输入超时阈值 | 0.30 s |
| `goal_distance_weight` | 目标距离权重 | 1.0 |
| `heading_weight` | 航向误差权重 | 0.7 |
| `curvature_weight` | 转弯幅度惩罚 | 0.15 |
| `clearance_weight` | 净空奖励 | 0.35 |

## 4. 调参顺序（不要同时乱调）

1. **先测车体**：回填半长、半宽；先取比实测更大的值。
2. **低速安全值**：架空或断开执行器验证符号，再将 `speed_mps` 从 0.10 m/s 起步。
3. **几何可达性**：按车辆最小转弯半径设置 `max_curvature <= 1/R_min`；确认正角速度左转。
4. **碰撞余量**：从 `inflation_m=0.30` 开始，在静态纸箱旁逐步减小，绝不先减 footprint。
5. **视距与离散度**：保证 `horizon_m` 能看到绕行出口；CPU 足够时增加采样数，防漏碰时减小 `step_m`。
6. **先目标后平滑**：先调 `goal_distance_weight`/`heading_weight` 能绕回航点，再增加 `curvature_weight` 抑制大转弯。
7. **最后调净空**：逐步增加 `clearance_weight`，若车辆为追求净空偏离目标过多则回退。
8. 每次只改一个参数，保存 rosbag、参数文件、选中轨迹与结果。

## 5. 本地/RDK构建和测试

仓库根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_subject1_avoidance_mvp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select ugv_subject1_avoidance_mvp
colcon test-result --verbose
```

RDK 使用 `ssh-sunrise-mercury`，将仓库同步到独立工作区后执行相同命令。不要覆盖设备上 Livox/LIO 的现有工作区。

手工启动：

```bash
source install/setup.bash
ros2 run ugv_subject1_avoidance_mvp local_avoidance_node --ros-args \
  -p speed_mps:=0.10 -p inflation_m:=0.30
```

## 6. 真实测试步骤

1. **执行器断开**：静态发布空 `PoseArray`，验证 active=false、命令恒零。
2. **超时**：停止任一输入，最多 `input_timeout_s + 1/publish_rate_hz` 后应 active=true、命令为零。
3. **桌面合成障碍**：分别发布左、右、中央障碍；在 RViz 检查轨迹方向和 `omega` 符号。
4. **实车静止**：接入真实感知，确认障碍中心与车体方向一致，没有 self points；输出仍不连接执行器。
5. **封闭低速场地**：软障碍、急停人员、0.10 m/s 开始；逐步验证直通、左绕、右绕、封堵停车。
6. **交接测试**：由集成方验证 active 的边沿和下游仲裁。该包本身不订阅巡航命令。
7. 记录 `/subject1/obstacles`、航点、active、候选命令、选中轨迹和 TF；测试后复盘最小净空。

## 7. 下一会话提示模板

```text
只修改 src/ugv_subject1_avoidance_mvp 和 workstreams/04_subject1_avoidance。
先读根 AGENTS.md、docs/ARCHITECTURE.md、docs/INTERFACES.md 和本路线图。
当前目标：[填写一个可验证目标]
必须保持：无障碍 inactive+零命令；超时/封堵 active+零命令；不接触巡航命令。
先运行包级测试，修改后再跑仓库规定的完整检查。不要声称合成测试等于真车成功。
```

## 8. 明确 TODO / 现场缺口

- [ ] 回填实测车体半长、半宽和允许的安全膨胀。
- [ ] 回填最低可靠速度、最大允许曲率及 `omega` 符号的执行器验证记录。
- [ ] 用真实 Horizon 障碍输出确定输入频率，随后冻结 timeout。
- [ ] 确认感知 PoseArray 中单点代表栅格中心还是更大目标；当前碰撞模型把点当占据中心并仅用车辆膨胀。
- [ ] 坡地、沟壑、动态障碍不在 MVP 保证范围；真实赛道失败证据出现后再决定是否增加模型。
- [ ] 下游团队/集成包负责仲裁与硬件 watchdog；本包不得自行扩张该职责。
