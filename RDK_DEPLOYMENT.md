# RDK 科目二部署与启动

本文对应当前“LIO odom 直连 + CSV 严格顺序确认 + 标准 Pure Pursuit 动态前视”版本。生产 launch 不再启动 `odom_guard` 或自动恢复协调器。

## 1. 获取用户指定版本

在 RDK 上将 `DEPLOY_SHA` 替换为用户给出的完整 40 位提交：

```bash
REPO_URL='https://github.com/niuma-phd/ugv_planning_control.git'
DEPLOY_SHA='<完整40位提交>'
WS='/home/sunrise/ugv_planning_control_ws_lio_guidance'

test "${#DEPLOY_SHA}" -eq 40
if [ ! -d "$WS/.git" ]; then
  git clone "$REPO_URL" "$WS"
fi
cd "$WS"
git fetch --prune origin
git checkout --detach "$DEPLOY_SHA"
test "$(git rev-parse HEAD)" = "$DEPLOY_SHA"
```

若工作区已有未保存修改，先停止，不要覆盖现场文件。

## 2. 构建

```bash
cd /home/sunrise/ugv_planning_control_ws_lio_guidance
source /opt/ros/humble/setup.bash
scripts/build_subject2.sh --clean
source /home/sunrise/ugv_planning_control_ws_lio_guidance/install_subject2/setup.bash
python3 scripts/verify_repository.py
```

仓库通用默认配置仍保留低速开发值。当前实车验证参数单独保存在
`src/ugv_subject2_bringup/config/subject2_field_verified.yaml`，不要把它误当成
其他车辆可直接套用的安全配置。需要复现实车基线时，人工核对差异后再复制：

```bash
cd /home/sunrise/ugv_planning_control_ws_lio_guidance
diff -u /home/sunrise/subject2_field.yaml \
  src/ugv_subject2_bringup/config/subject2_field_verified.yaml || true
cp src/ugv_subject2_bringup/config/subject2_field_verified.yaml \
  /home/sunrise/subject2_field.yaml
```

实车验证控制参数为：

```yaml
nominal_speed: 1.5
max_speed: 1.5
max_yaw_rate: 2.0
max_curvature: 0.10
enhanced_tracking_enabled: true
minimum_linear_speed: 1.0
minimum_tracking_yaw_rate: 0.10
minimum_turning_yaw_rate: 2.0
lookahead_min_m: 8.0
lookahead_max_m: 12.0
lookahead_speed_gain: 2.0
turning_motion_threshold_rad: 0.05
turn_in_place_threshold_rad: 1.50
turn_in_place_exit_threshold_rad: 0.70
tracking_omega_enter_threshold_rad_s: 0.075
tracking_omega_exit_threshold_rad_s: 0.040
slowdown_distance: 5.0
waypoint_tolerance: 2.0
goal_tolerance: 2.0
command_publish_rate_hz: 0.5
command_hold_timeout_sec: 4.10
tracking_yaw_pulse_enabled: false
tracking_yaw_min_pulse_sec: 2.00
tracking_yaw_demand_gain: 1.0
stall_boost_enabled: false
```

约束：`command_hold_timeout_sec` 必须严格大于
`2 / command_publish_rate_hz`，因此 `0.5 Hz` 使用 `4.10 s`。如果继续维护
`/home/sunrise/subject2_field.yaml`，只保留当前节点参数和真实设备已验证的
GPS 配置。所有运动参数只在启动时读取，修改后必须重启控制 launch。

全局航点应先在本地电脑按
[WGS84 航点转换说明](docs/WGS84航点转换说明.md)生成并核对，再上传：

```powershell
python .\scripts\convert_wgs84_waypoints.py --heading E --force
scp .\watpoints_odom.csv sunrise@192.168.188.227:/home/sunrise/waypoints.csv
```

## 3. 从全部程序未启动开始

终端 A：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
/home/sunrise/ws_livox/rdk_lio_tmux.sh start
```

该脚本由用户确认会同时启动 Horizon 雷达驱动和 LIO；不要再启动第二份驱动或 LIO。

终端 B：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/ugv_planning_control_ws_lio_guidance/install_subject2/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch ugv_subject2_bringup subject2.launch.py \
  config_file:=/home/sunrise/subject2_field.yaml \
  waypoint_file:=/home/sunrise/waypoints.csv \
  lidar_extrinsics_valid:=true \
  publish_lidar_static_tf:=false \
  lidar_extrinsics_provenance:=user-confirmed-zero-extrinsics \
  base_to_lidar_x:=0.0 \
  base_to_lidar_y:=0.0 \
  base_to_lidar_z:=0.0 \
  base_to_lidar_roll:=0.0 \
  base_to_lidar_pitch:=0.0 \
  base_to_lidar_yaw:=0.0
```

控制终端会显示当前正在前往、已到达/通过和最终锁停的航点编号，并约每秒显示当前位姿、目标距离及输出 `v/omega`。

启动后先核对，不要同时保留第二个 `/cmd_vel` 发布者：

```bash
ros2 topic info /cmd_vel -v
ros2 topic hz /cmd_vel
ros2 topic hz /localization/odom
```

实车 profile 的 `/cmd_vel` 目标频率约为 `0.5 Hz`。

## 4. 当前边界

- 控制输入为 adapter 发布的 `/localization/odom`，不是 `/localization/trusted_odom`。
- 不检查 odom 时间戳、新鲜度或时间顺序，也不使用 `odom_valid/navigation_enabled`。
- LIO 不再产生新控制结果时，缓存命令租约过期后由发布 timer 置零；按
  `0.5 Hz + 4.10 s` 配置，最坏响应可能接近 6 秒，不能替代底盘独立的
  `/cmd_vel` 接收超时停车看门狗或现场急停。
- Ctrl+C 或进程崩溃不保证最后一帧零速；停止前必须先确保车辆已停稳。
- GPS 不参与逐帧控制；串口接入见 [GPGGA 串口接入说明](docs/GGA串口接入说明.md)。
- 航点固定使用 `/home/sunrise/waypoints.csv`，代码不会创建或修改该文件。
- 六轴全 0 外参来自用户明确确认；若机械安装改变，必须重新确认后更新命令。
- 当前车辆没有避障，只能在清空障碍物的封闭场地运行。
- GPS 接续定位和 LIO 跑飞自动恢复尚未完成，`auto_align_from_path=false`。

完整算法、参数影响与调参顺序见[科目二顺序航点控制与现场调参说明](docs/科目二_自主导航使用说明.md)。
