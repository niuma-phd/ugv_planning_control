# GPGGA 串口接入说明

> GPS 节点当前只做独立 position-only 采集和质量观察。航点控制直接消费
> LIO 适配后的 `/localization/odom`，不再经过 `/localization/trusted_odom`
> 或自动恢复协调器。本文若出现旧控制链描述，以
> [科目二顺序航点控制与现场调参说明](科目二_自主导航使用说明.md)为准。

## 1. 本轮能力边界

`指令参考GGA.docx` 定义的是标准 NMEA ASCII `$GPGGA` 报文。它包含 UTC
时分秒、经纬度、定位质量、参与解算卫星数、HDOP、海拔和大地水准面高程异常，**不包含车辆航向**。

因此当前数据流是：

```text
显式串口设备 → gga_serial_node → /gps/gga_sentence
                                → /gps/fix
                                → /gps/gga_position_valid

LIO → /localization/odom（位置和航向）→ 顺序航点控制 → /cmd_vel
```

- `/gps/fix` 是 `sensor_msgs/msg/NavSatFix`，`frame_id=gps_link` 表示天线相位中心。
- GGA 海拔是平均海平面高；节点按 `椭球高 = 海平面高 + 高程异常` 填写
  `NavSatFix.altitude`。
- GGA 没有经审核的误差模型，`position_covariance_type` 保持 `UNKNOWN`；HDOP
  不会被伪装成 `m²`。
- `header.stamp` 是主机收到完整校验通过报文的 ROS 时刻。GGA 只有 UTC 日内时间、没有日期，
  所以 UTC 字段只用于拒绝重复/倒退历元；该时间方案不满足自动恢复的测量时刻契约。
- 节点不发布 `/localization/gps_pose`，也不会把 position-only 数据标成控制输入。

健康行驶的控制位置和航向来自 LIO，因此没有 GPS 航向不会改变当前顺序航点控制。

## 2. 串口为何保持为空

参考文档没有给出设备路径、波特率、数据位、校验位或停止位。生产 launch 的默认值因此是：

```text
gps_serial_device:=""
gps_serial_baud_rate:=""
gps_serial_data_bits:=""
gps_serial_parity:=""
gps_serial_stop_bits:=""
```

设备路径为空时 launch **不会创建** `gga_serial_node`，也不会扫描 `/dev`、自动猜波特率或探测
RDK。设备路径非空时，上述五项必须全部显式填写；节点只会打开指定的绝对路径。

## 3. 在实际连接 GPS 的机器上查串口

本轮 RDK 未上线，不运行这些命令。GPS 接到目标机器后，按以下顺序取证。

### 3.1 首选稳定的 by-id 路径

插拔 GPS 前后分别执行：

```bash
ls -l /dev/serial/by-id/ 2>/dev/null || true
ls -l /dev/serial/by-path/ 2>/dev/null || true
```

新出现的 `/dev/serial/by-id/...` 通常比 `/dev/ttyUSB0` 或 `/dev/ttyACM0` 稳定，应优先填入
`gps_serial_device`。若没有 `by-id`，可同时观察 udev 事件，再插入设备：

```bash
udevadm monitor --kernel --udev --property
```

另一个终端查看候选设备及属性：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyS* 2>/dev/null
udevadm info --query=property --name=/dev/ttyUSB0
```

不要把示例 `/dev/ttyUSB0` 直接当成答案；以本机插拔事件、USB 序列号和供应商/产品 ID
交叉确认。还要检查占用和权限：

```bash
GPS_DEV=/dev/serial/by-id/替换为实际名称
readlink -f "$GPS_DEV"
stat -c 'mode=%A owner=%U group=%G path=%n' "$GPS_DEV"
id
fuser -v "$GPS_DEV" || true
lsof "$GPS_DEV" 2>/dev/null || true
```

常见串口组是 `dialout`。若账号不在设备所属组，需要管理员按现场权限策略加入相应组并重新
登录；不要用长期 `chmod 777` 绕过权限。

### 3.2 确认波特率和 8N1 等帧格式

`GGA.docx` 没有这些信息，必须从 GPS 配置软件、供应商手册或设备当前配置中确认。不要自动
轮询多个波特率，因为乱码偶然含 `$` 不能证明配置正确，还可能与正在占用串口的程序冲突。

确认参数后，可先只读检查 ASCII 报文。下例中的值只是命令格式，必须替换成已确认值：

```bash
GPS_DEV=/dev/serial/by-id/替换为实际名称
GPS_BAUD=替换为已确认波特率

stty -F "$GPS_DEV" "$GPS_BAUD" cs8 -cstopb -parenb raw -echo
timeout 5 cat "$GPS_DEV"
```

应看到以 `$GPGGA,` 开头、以 `*HH` 和 CR/LF 结束的完整 ASCII 行。记录至少 30 秒原始输出，
检查更新频率、fix 降级、断流、午夜 UTC 递增和差分龄期的真实表现。

## 4. 先单独启动 GPS 节点

本地构建后，可不启动雷达、LIO、控制器或底盘，仅运行 GPS 适配器：

```bash
source /opt/ros/humble/setup.bash
source install_subject2/setup.bash

ros2 run ugv_localization_mvp gga_serial_node --ros-args \
  --params-file src/ugv_subject2_bringup/config/subject2.yaml \
  -p device:="$GPS_DEV" \
  -p baud_rate:="$GPS_BAUD" \
  -p data_bits:=8 \
  -p parity:=none \
  -p stop_bits:=1
```

仅当设备确认为 8N1 时才可照抄后三项。另开终端观察：

```bash
ros2 topic echo /gps/gga_sentence
ros2 topic echo /gps/fix
ros2 topic echo /gps/gga_position_valid
```

默认 `quality_profile_valid=false`，因此即使经纬度可见，质量 Bool 仍保持 false。取得真实数据
统计并审核 fix quality 白名单、最少卫星数和最大 HDOP 后，才在现场参数文件中同时修改：

```yaml
gga_serial:
  ros__parameters:
    quality_profile_valid: true
    accepted_fix_qualities: [4]  # 示例占位；必须由实际接收机状态定义审核
    minimum_satellites: 10       # 示例占位；必须由现场统计审核
    maximum_hdop: 2.0            # 示例占位；必须由现场统计审核
```

这些质量参数只控制 position-only 的 `/gps/gga_position_valid`，不会开启自动恢复。

## 5. 纳入科目二 launch

串口和帧格式全部确认后，在原有实车启动参数之外补充：

```bash
gps_serial_device:="$GPS_DEV" \
gps_serial_baud_rate:="$GPS_BAUD" \
gps_serial_data_bits:=8 \
gps_serial_parity:=none \
gps_serial_stop_bits:=1
```

例如这些参数可追加到 `ros2 launch ugv_subject2_bringup subject2.launch.py ...` 或
`subject2_horizon.launch.py ...`。它们只启动 GPS 采集，不改变控制输入。

## 6. 参考样例的校验矛盾

参考文档样例尾部写的是 `*55`，但按 NMEA 规则对 `$` 与 `*` 之间全部 ASCII 字节做 XOR，
该行结果是 `0x65`。解析器会拒绝原样 `*55`；把同一 payload 的尾部改成 `*65` 后才会通过。
生产代码不会为迁就样例而关闭校验。

参考文档还将基站 ID 描述为 `0000-1023`，但示例值是 `AAAA`。当前解析器因此只对该字段执行
“最多 4 位字母或数字”的语法校验；它不参与定位有效性、控制或恢复决策。实车使用前应向接收机供应方确认该字段的真实约束。
