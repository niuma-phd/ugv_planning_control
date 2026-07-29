# 越野车规划控制 MVP

面向 ROS 2 Humble 与两轮差速履带车的最小可运行规划控制仓库。当前主分支仅维护科目二自主导航闭环。仓库外层的 `越野车规划控制/` 目录仅作为背景资料，不属于本代码仓库。

科目一已经退出主分支，最终状态封存在 Git 标签 `subject1-final-20260725`；除非明确决定恢复该方向，否则不要把科目一代码重新合入主分支。

## 当前闭环

### 科目二：自主导航

`Horizon + IMU → LIO → 里程计坐标适配 → 顺序航点控制 → /cmd_vel`

- 定位使用现有 LIO；适配器把 `/livox_odometry_mapped` 转为规范的
  `/localization/odom`，控制器逐帧直接消费该话题。
- 生产 `waypoint_controller_node` 在启动时通过 `waypoint_file`
  直接完整加载用户指定的绝对 CSV 航点文件；不再依赖 `/subject2/path`
  或 `waypoint_file_publisher_node`。
- CSV 严格按行顺序执行，当前点确认后才追踪下一点；目标位于后方或
  航向偏差过大时先原地对准，最终点到达后零速锁存。
- 明日实车使用外部 tmux launcher 的 Horizon CustomMsg LIO；仓库另保留 PointCloud2 受管启动模式，两者禁止同时运行。
- 当前起步假设 `map` 与 `odom` 重合。生产 launch 不再启动
  `odom_guard` 或恢复协调器，也不使用 odom 时间戳、新鲜度、
  `odom_valid` 或 `navigation_enabled` 门禁。
- 最终命令为 `geometry_msgs/msg/Twist`：`linear.x` 单位 m/s，`angular.z` 单位 rad/s。
- 生产配置当前 `nominal_speed=0.5 m/s`、`max_speed=1.0 m/s`；上限放宽
  不会把巡航速度自动提高到 1.0 m/s。
- 生产 launch 已支持显式串口的 GPGGA position-only 接入，发布 `/gps/fix` 与质量诊断；
  GGA 没有航向，所以正常控制仍完全使用 LIO，默认不启用 GPS/LIO 自动恢复。

详见[科目二自主导航使用说明](docs/科目二_自主导航使用说明.md)。
旧 guard/recovery 测试记录保留在[科目二航点文件接入与测试手册](docs/科目二_上游接入与测试手册.md)，
不要把其中旧启动参数用于当前生产链。
另一台设备或 AI 从 GitHub 取得精确版本并部署到 RDK 时，使用
[`RDK_DEPLOYMENT.md`](RDK_DEPLOYMENT.md)作为入口，不要使用临时会话交接文件。

## 构建

科目二生产构建不包含测试工具包 `ugv_mvp_tools`。

```bash
# 科目二最小包：
# ugv_localization_mvp ugv_subject2_mvp ugv_subject2_bringup
scripts/build_subject2.sh --clean
source install_subject2/setup.bash
```

构建产物位于 `build_subject2/`、`install_subject2/`、`log_subject2/`。

具体启动命令、航点文件格式、参数、TF 检查与调试步骤见科目二使用说明。

## 仓库结构

```text
src/           ROS 2 功能包
scripts/       独立构建、仓库校验和离线夹具脚本
docs/          科目二使用说明与实车接口/待办
dependencies/  外部源码版本记录
artifacts/     外部大文件的记录规则
```

## 开发验证

```bash
python3 scripts/verify_repository.py

source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose

# fixture 只发布到 /fixture/cmd_vel，不授权真实车辆输出
ROS_DOMAIN_ID=171 scripts/run_fixture_smoke.sh subject2
```

航点 CSV 只在控制节点启动时读取一次，不热更新。修改路线时必须先安全停车，再用新文件重启控制 launch。
文件最大 `2 MiB`、单行最大 `4096` 字节、最多 `10000` 个航点；任一上限超出时控制节点拒绝启动。
航点坐标、点序、场地一致性和最终文件路径由用户负责提供与审核。

## 文档

- [科目二自主导航：代码逻辑、构建、启动与调试](docs/科目二_自主导航使用说明.md)
- [旧 guard/recovery 航点测试记录](docs/科目二_上游接入与测试手册.md)
- [实车接口、TF、测量回填与剩余待办](docs/实车接口与待办.md)
- [GPGGA 串口接入与串口查询](docs/GGA串口接入说明.md)
- [从 GitHub 取得精确版本并部署到 RDK](RDK_DEPLOYMENT.md)

## 安全边界

本仓库完成的是软件 MVP 与离线/断执行器验证，不等于实车放行。以下条件未满足时，禁止发送非零实车控制：

1. 雷达到 `base_link` 的实测六轴外参及记录编号已经审核；外部 tmux LIO 模式不另发静态 TF，避免 `livox_frame` 双父；
2. 车辆尺寸、速度、转向、制动距离和命令延迟已测量并回填；
3. `/cmd_vel` 只有一个最终发布者，底盘侧独立看门狗有效；
4. 非法数值、TF 丢失和急停测试均在断执行器条件下通过，且底盘侧独立
   `/cmd_vel` 接收超时看门狗已验证；
5. 获得现场负责人批准后，才可按限速参数进行低速测试。

未闭合项和合作方输入要求统一记录在[实车接口与待办](docs/实车接口与待办.md)。

> **人机责任边界**：真实上车的直线、转弯、制动和航点跟踪测试均由用户本人
> 现场操作。AI 只能在执行机构物理断开，或车辆可靠架起且车轮/履带离地时
> 辅助静态验证、分析日志和调整配置；不得自行发布非零运动命令或发起实车运动。

> **本轮验证边界**：RDK 已掉线，本轮不做当前 RDK 或实车验证。GPGGA 串口接入已实现为
> position-only 软件路径，但具体串口、帧格式和真实接收机尚未验证；无可信航向时 GPS/LIO
> 当前控制直接消费 LIO odom，不启用自动恢复。底盘命令超时看门狗和实车参数仍需现场闭合。
