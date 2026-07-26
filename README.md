# 越野车规划控制 MVP

面向 ROS 2 Humble 与两轮差速履带车的最小可运行规划控制仓库。当前主分支仅维护科目二自主导航闭环。仓库外层的 `越野车规划控制/` 目录仅作为背景资料，不属于本代码仓库。

科目一已经退出主分支，最终状态封存在 Git 标签 `subject1-final-20260725`；除非明确决定恢复该方向，否则不要把科目一代码重新合入主分支。

## 当前闭环

### 科目二：自主导航

`Horizon PointCloud2 + IMU → 受管 LIO → 里程计适配/保护 → Pure Pursuit → /cmd_vel`

- 定位使用现有 LIO，保护后的里程计为 `/localization/trusted_odom`。
- 全局路径输入为 `/subject2/path`；开发测试可用 CSV 航点文件通过独立工具节点以 1 Hz 发布。
- 当前起步假设 `map` 与 `odom` 重合；LIO 故障后，恢复协调器只有在新 GPS 全局位姿和重启后 LIO 里程计都稳定、`map→odom` 更新回显一致且保护器复位验证通过后，才重新允许行驶。
- 恢复失败或超时进入 `ABORTED`，持续禁止导航；默认每个 `recovery_coordinator` 进程生命周期最多尝试 1 次。
- 最终命令为 `geometry_msgs/msg/Twist`：`linear.x` 单位 m/s，`angular.z` 单位 rad/s。

详见[科目二自主导航使用说明](docs/科目二_自主导航使用说明.md)。
上游数据接入和逐项测试见[科目二上游接入与测试手册](docs/科目二_上游接入与测试手册.md)。

## 构建

科目二生产构建不包含测试工具包 `ugv_mvp_tools`。

```bash
# 科目二最小包：
# ugv_localization_mvp ugv_subject2_mvp ugv_subject2_bringup
scripts/build_subject2.sh --clean
source install_subject2/setup.bash
```

构建产物位于 `build_subject2/`、`install_subject2/`、`log_subject2/`。

具体启动命令、参数、话题检查、TF 检查与调试步骤见科目二使用说明。

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

CSV 航点发布器属于开发工具包 `ugv_mvp_tools`，不在生产 launch 中默认启动。完整编辑、启动、观察和停止命令见科目二使用说明。

## 文档

- [科目二自主导航：代码逻辑、构建、启动与调试](docs/科目二_自主导航使用说明.md)
- [科目二上游接入与测试手册](docs/科目二_上游接入与测试手册.md)
- [实车接口、TF、测量回填与剩余待办](docs/实车接口与待办.md)

## 安全边界

本仓库完成的是软件 MVP 与离线/断执行器验证，不等于实车放行。以下条件未满足时，禁止发送非零实车控制：

1. 雷达白名单、雷达到 `base_link` 的实测静态 TF 及记录编号已经审核；
2. 车辆尺寸、速度、转向、制动距离和命令延迟已测量并回填；
3. `/cmd_vel` 只有一个最终发布者，底盘侧独立看门狗有效；
4. 断定位、断点云、输入超时、非法数值和急停测试均在断执行器条件下通过；
5. 获得现场负责人批准后，才可按限速参数进行低速测试。

未闭合项和合作方输入要求统一记录在[实车接口与待办](docs/实车接口与待办.md)。

> **本轮验证边界**：RDK 离线，本轮未访问、未编译、未测试 RDK，也未做实车验收。GPS/LIO 恢复软件链和合成 fixture 已实现；真实 GPS 适配、真实 LIO/RDK 进程管理和实车参数仍待验证。
