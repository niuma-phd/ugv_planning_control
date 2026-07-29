# Win10 Codex：RDK GPS 与航点命令

## 直接复制给 Win10 Codex 的首轮提示词

```text
目标仓库是 https://github.com/niuma-phd/ugv_planning_control.git。先让我提供已推送的精确 40 位 SHA，在 Win10 和 RDK 都检出该 SHA并核对 HEAD；RDK 的 SSH 命令和密码随后由我提供，密码只由我在 Windows OpenSSH 交互提示中输入，禁止写入提示词、命令、文件或日志。连接后严格按 RDK_DEPLOYMENT.md 快速做三件事：①确认 GPS 稳定串口和供应商确认的帧格式，采样约30秒有效GGA；②仅在仓库外 field YAML 固化真实数据验证过的稳定GPS配置并复测，然后完整构建；③向我索取已设置好的航点CSV绝对路径和必需实车雷达外参，最后只输出一条填写完整的 subject2.launch.py 测试命令，不执行。不要猜串口、波特率、帧格式、外参或航点，不编辑航点，不启动控制，不发布运动命令；速度保持0.5m/s，automatic_recovery_enabled=false。
```

本文编写时未连接 RDK；设备值只能来自用户、供应商资料和现场数据。

## 准备：精确 SHA 与 SSH

用户提供已推送的 40 位 `DEPLOY_SHA`。Win10 PowerShell：

```powershell
$RepoUrl='https://github.com/niuma-phd/ugv_planning_control.git'
$DeploySha='<用户提供的40位SHA>'; $Workspace=Join-Path $HOME 'ugv_planning_control'
if ($DeploySha -notmatch '^[0-9a-fA-F]{40}$') { throw '需要精确40位SHA' }
if (-not (Test-Path $Workspace)) { git clone $RepoUrl $Workspace }
Set-Location $Workspace
if (git status --porcelain) { throw 'Win10工作区不干净' }
git fetch --prune --tags origin '+refs/heads/*:refs/remotes/origin/*'
git checkout --detach $DeploySha
if ((git rev-parse HEAD).Trim() -ne $DeploySha) { throw 'SHA不一致' }
```

再执行用户提供的 SSH 命令；密码仅由用户交互输入。RDK 上：

```bash
REPO_URL='https://github.com/niuma-phd/ugv_planning_control.git'
DEPLOY_SHA='<同一40位SHA>'; WS="$HOME/ugv_planning_control_ws"
[[ "$DEPLOY_SHA" =~ ^[0-9a-fA-F]{40}$ ]] || exit 2
if [ ! -d "$WS/.git" ]; then git clone "$REPO_URL" "$WS"; fi
cd "$WS"; test -z "$(git status --porcelain)" || exit 2
git fetch --prune --tags origin '+refs/heads/*:refs/remotes/origin/*'
git checkout --detach "$DEPLOY_SHA"
test "$(git rev-parse HEAD)" = "$DEPLOY_SHA"
```

## 1. 确认 GPS 并采样约 30 秒 GGA

不得扫描或猜波特率。先取得供应商/配置工具确认的 `GPS_BAUD`、
`GPS_DATA_BITS`、`GPS_PARITY`（`none/even/odd`）、`GPS_STOP_BITS`（`1/2`）。
插拔前后比较并选稳定路径：

```bash
ls -l /dev/serial/by-id/ 2>/dev/null || true
ls -l /dev/serial/by-path/ 2>/dev/null || true
GPS_DEV='<现场确认的/dev/serial/by-id/...绝对路径>'
readlink -f "$GPS_DEV"; fuser -v "$GPS_DEV" || true
```

只启动 GPS 节点，不启动 `subject2.launch.py`：

```bash
cd "$WS"; source /opt/ros/humble/setup.bash
colcon --log-base log_gps build --build-base build_gps --install-base install_gps \
  --packages-select ugv_localization_mvp --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$WS/install_gps/setup.bash"
FIELD_CONFIG="$HOME/subject2_field.yaml"
if [ ! -e "$FIELD_CONFIG" ]; then cp src/ugv_subject2_bringup/config/subject2.yaml "$FIELD_CONFIG"; fi
GPS_BAUD='<确认值>'; GPS_DATA_BITS='<确认值>'; GPS_PARITY='<none/even/odd>'; GPS_STOP_BITS='<1或2>'
ros2 run ugv_localization_mvp gga_serial_node --ros-args --params-file "$FIELD_CONFIG" \
  -p device:="$GPS_DEV" -p baud_rate:="$GPS_BAUD" -p data_bits:="$GPS_DATA_BITS" \
  -p parity:="$GPS_PARITY" -p stop_bits:="$GPS_STOP_BITS"
```

保持该终端运行；第二个 SSH 终端采样：

```bash
source /opt/ros/humble/setup.bash; source "$WS/install_gps/setup.bash"
GGA_SAMPLE="$HOME/gga_$(date -u +%Y%m%dT%H%M%SZ)_30s.txt"
timeout --signal=INT 30s ros2 topic echo /gps/gga_sentence > "$GGA_SAMPLE"
RC=$?; test "$RC" -eq 0 -o "$RC" -eq 124; test -s "$GGA_SAMPLE"
awk -F, '/GGA,/ {print "quality=" $7, "satellites=" $8, "hdop=" $9}' "$GGA_SAMPLE"
ros2 topic echo /gps/fix --once
```

连续收到校验通过、UTC 推进且位置合理的 GGA 才算确认。空样本、校验错误
或断流时停止，不得关闭校验或放宽门禁。

## 2. 固化稳定配置、复测并完整构建

根据 30 秒样本、供应商对 `fix_quality` 的定义和用户认可状态，只修改
仓库外的 `$FIELD_CONFIG`，在 `gga_serial.ros__parameters` 固化：

```yaml
device: /dev/serial/by-id/<确认设备>
baud_rate: <确认值>
data_bits: <确认值>
parity: <none/even/odd>
stop_bits: <1或2>
quality_profile_valid: true
accepted_fix_qualities: [<已确认质量码>]
minimum_satellites: <样本支持的正整数>
maximum_hdop: <样本支持的正数>
```

无有效定位或供应商定义时保持 `quality_profile_valid: false` 并停止。保存后
`Ctrl-C` 停旧节点，再仅用 field YAML 重启：

```bash
ros2 run ugv_localization_mvp gga_serial_node --ros-args --params-file "$FIELD_CONFIG"
# 第二终端：
VALID_SAMPLE="$HOME/gga_valid_$(date -u +%Y%m%dT%H%M%SZ)_30s.txt"
timeout --signal=INT 30s ros2 topic echo /gps/gga_position_valid > "$VALID_SAMPLE"
RC=$?; test "$RC" -eq 0 -o "$RC" -eq 124
printf 'true=%s false=%s\n' "$(grep -c '^data: true' "$VALID_SAMPLE" || true)" \
  "$(grep -c '^data: false' "$VALID_SAMPLE" || true)"
```

记录计数和日志，`Ctrl-C` 停节点，然后完整构建：
同时确保 field YAML 中 `nominal_speed: 0.50`、`max_speed: 0.50`、
`automatic_recovery_enabled: false` 和 `auto_align_from_path: false`；若是旧配置，
将这四项更新为当前值后再构建检查。

```bash
cd "$WS"; source /opt/ros/humble/setup.bash
scripts/build_subject2.sh --clean
source "$WS/install_subject2/setup.bash"
python3 scripts/verify_repository.py
ros2 launch ugv_subject2_bringup subject2.launch.py --show-args
grep -E 'nominal_speed: 0\.50|max_speed: 0\.50|automatic_recovery_enabled: false' "$FIELD_CONFIG"
```

## 3. 只输出航点测试命令

构建成功后向用户索取：航点 CSV 绝对路径；实车 `base_link → livox_frame`
的 `x/y/z`（米）、`roll/pitch/yaw`（弧度）及来源编号；是否由外部 LIO
发布雷达 TF；再次确认 GPS 串口和帧格式。不得创建或编辑航点。

外部 LIO 模式用 `publish_lidar_static_tf:=false` 避免 TF 双父。信息齐全后，
提醒用户先确认外部 LIO `status` 通过，再只输出一条已替换所有
尖括号占位符的命令，**不得执行**：

```bash
ros2 launch ugv_subject2_bringup subject2.launch.py \
  config_file:="$HOME/subject2_field.yaml" waypoint_file:='<航点CSV绝对路径>' \
  odom_snapshot_directory:="$HOME/.ros/ugv_mvp" \
  lidar_extrinsics_valid:=true publish_lidar_static_tf:=false \
  lidar_extrinsics_provenance:='<外参来源编号>' \
  base_to_lidar_x:='<米>' base_to_lidar_y:='<米>' base_to_lidar_z:='<米>' \
  base_to_lidar_roll:='<弧度>' base_to_lidar_pitch:='<弧度>' base_to_lidar_yaw:='<弧度>' \
  gps_serial_device:='<稳定绝对串口>' gps_serial_baud_rate:='<波特率>' \
  gps_serial_data_bits:='<数据位>' gps_serial_parity:='<none/even/odd>' \
  gps_serial_stop_bits:='<1或2>' automatic_recovery_enabled:=false
```

最终仅回报：相同 SHA、GPS 样本摘要、field YAML 路径及复测计数、构建结果、
已填完整但未执行的 launch 命令，并声明未发布运动命令。
