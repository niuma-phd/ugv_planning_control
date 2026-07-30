# GPS 单点定位顺序压点说明

本文对应独立分支 `codex/gps-direct-waypoint`。它保留原来的顺序航点控制核心，
但把车辆平面位置改为 GPS GGA，不启动或读取 LIO。

## 数据链

```text
/dev/ttyUSB0 (115200, 8N1)
  -> gga_serial_node
  -> /gps/validated_fix
  -> gps_waypoint_controller_node
  -> /cmd_vel
```

RDK 实读确认接收机输出约 1 Hz 的 `GNGGA`，8 秒窗口内 8 条 GGA 校验和全部
正确。`GNTXT` 和其他 NMEA 类型会被静默忽略，不会把一个已经有效的 GGA 状态
打成无效。只有句型、校验和、UTC 单调性、fix quality、卫星数和 HDOP 全部通过的
同一帧才发布到 `/gps/validated_fix`。

当前配置只接受单点解 `fix quality=1`，并以实读样本固化
`satellites >= 12`、`HDOP <= 0.80`。这是现场首版的保守门限；若实际环境长期不满足，
必须先记录真实 GGA，再修改配置，不能绕过验证直接放行。

## 航点文件与执行顺序

启动时直接读取仓库根目录下的 `scripts/track_output.txt`。格式必须为 UTF-8：

```text
序号;经度;纬度;高程
1;118.81295711;32.09341467;0
```

- 序号必须严格递增，程序不排序、不跳到“最近的未来点”。
- 文件第一点被保留为第一个目标，不会被丢掉。
- 程序以第一点建立局部 ENU 投影：`x=东`、`y=北`，实时 GPS 与全部航点使用同一投影。
- 当前控制把 GPS 天线位置当作车辆位置；天线到车辆控制中心的平移外参尚未提供，
  因此到点含义是“天线进入容差圆”。
- 单点 GPS 的绝对误差通常大于 LIO；`waypoint_tolerance` 与 `goal_tolerance` 当前均为
  1.5 m，只是待现场验证的初值。

## 初始航向

`initial_heading` 必须由现场按车头方向填写：

| 参数 | 车头方向 | ENU yaw |
|---|---|---:|
| `EAST` | 正东 | `0` |
| `NORTH` | 正北 | `+pi/2` |
| `WEST` | 正西 | `pi` |
| `SOUTH` | 正南 | `-pi/2` |

启动时先使用该航向。车辆已经收到正向速度命令后，累计 GPS 位移达到 5 m 才会用
移动轨迹更新航向，逐帧的小抖动不会更新航向。

车辆已确认不会超过 3 m/s。两条已接受定位之间若出现更高的表观速度，控制节点不会
把该坐标交给航点控制器，而是锁存零速并要求重启 launch，防止坐标跳变越序推进航点。
串口失效或定位超时后会清空移动航向基线，恢复时从新样本重新建立基线。

单天线 GGA 无法观测原地转动。若初始方向与目标方向相差过大，原控制器会请求原地
转向；本 GPS 版本会立即锁存零速并提示 `HEADING_UNOBSERVABLE_STOP`，不会开环盲转。
因此测试前必须把车头对到与首段路线大致一致的东/南/西/北方向。当前文件首段约向
东偏南 10.6 度，使用 `initial_heading:=EAST`。

## RDK 独立工作区

源码目录：

```text
/home/sunrise/ws_gps_control/src/ugv_planning_control
```

源码部署完成后，在 RDK 独立工作区构建：

```bash
cd /home/sunrise/ws_gps_control/src/ugv_planning_control
chmod +x scripts/build_gps_subject2.sh
./scripts/build_gps_subject2.sh --clean
source install_gps/setup.bash
```

先做零速观察：

```bash
ros2 launch ugv_gps_waypoint_control gps_subject2.launch.py \
  track_file:=/home/sunrise/ws_gps_control/src/ugv_planning_control/scripts/track_output.txt \
  initial_heading:=EAST \
  motion_enabled:=false
```

确认 `/cmd_vel` 只有该控制节点一个发布者、GPS 状态稳定且现场允许后，才把最后一项
改为 `motion_enabled:=true`。该 launch 只启动 GGA 串口节点和 GPS 航点控制节点，
不要同时启动原 LIO 航点控制 launch。

终端会输出 `going to sequence=...`、当前位置、当前命令和最终点到达信息；也可查看：

```bash
ros2 topic echo /gps_control/status
ros2 topic echo /gps/gga_position_valid
ros2 topic hz /gps/validated_fix
```
