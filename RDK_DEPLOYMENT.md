# RDK 科目二部署与启动

本文对应当前“LIO odom 直连 + CSV 严格顺序航点”版本。生产 launch 不再启动 `odom_guard` 或自动恢复协调器。

## 1. 获取用户指定版本

在 RDK 上将 `DEPLOY_SHA` 替换为用户给出的完整 40 位提交：

```bash
REPO_URL='https://github.com/niuma-phd/ugv_planning_control.git'
DEPLOY_SHA='<完整40位提交>'
WS='/home/sunrise/ugv_planning_control_ws'

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
cd /home/sunrise/ugv_planning_control_ws
source /opt/ros/humble/setup.bash
scripts/build_subject2.sh --clean
source /home/sunrise/ugv_planning_control_ws/install_subject2/setup.bash
python3 scripts/verify_repository.py
```

仓库默认控制参数为：

```yaml
nominal_speed: 0.50
max_speed: 1.00
max_yaw_rate: 0.40
max_curvature: 1.00
turn_in_place_threshold_rad: 1.0472
slowdown_distance: 1.20
waypoint_tolerance: 0.30
goal_tolerance: 0.30
```

如果继续使用 `/home/sunrise/subject2_field.yaml`，只保留当前参数名和已经由真实设备验证的 GPS 配置。运动参数只在启动时读取，修改后应重启控制 launch。

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
source /home/sunrise/ugv_planning_control_ws/install_subject2/setup.bash
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

## 4. 当前边界

- 控制输入为 adapter 发布的 `/localization/odom`，不是 `/localization/trusted_odom`。
- 不检查 odom 时间戳、新鲜度或时间顺序，也不使用 `odom_valid/navigation_enabled`。
- LIO 不再发布时控制器不会产生新命令；底盘必须有独立 `/cmd_vel` 接收超时停车看门狗。
- GPS 不参与逐帧控制；串口接入见 [GPGGA 串口接入说明](docs/GGA串口接入说明.md)。
- 航点固定使用 `/home/sunrise/waypoints.csv`，代码不会创建或修改该文件。
- 六轴全 0 外参来自用户明确确认；若机械安装改变，必须重新确认后更新命令。

完整算法、参数影响与调参顺序见[科目二顺序航点控制与现场调参说明](docs/科目二_自主导航使用说明.md)。
