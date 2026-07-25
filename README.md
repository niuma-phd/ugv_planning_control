# 越野车规划控制 MVP

面向 ROS 2 Humble 与两轮差速履带车的最小可运行规划控制仓库。当前优先保障科目二闭环，同时保留科目一基于 Horizon 点云的局部避障闭环。仓库外层的 `越野车规划控制/` 目录仅作为背景资料，不属于本代码仓库。

## 当前闭环

### 科目二：自主导航

`Horizon PointCloud2 + IMU → LIO → 里程计适配/保护 → Pure Pursuit → /cmd_vel`

- 定位使用现有 LIO，保护后的里程计为 `/localization/trusted_odom`。
- 全局路径输入为 `/subject2/path`。
- 当前起步假设是 `map` 与 `odom` 重合，仅适用于已确认路径原点和坐标轴一致的场地。
- 最终命令为 `geometry_msgs/msg/Twist`：`linear.x` 单位 m/s，`angular.z` 单位 rad/s。

详见[科目二自主导航使用说明](docs/科目二_自主导航使用说明.md)。

### 科目一：局部避障

`Horizon PointCloud2 → 机体系障碍物 → 局部轨迹搜索 → 名义命令/避障命令仲裁 → /cmd_vel`

- 无相关障碍物时透传合作方的 `/subject1/nominal_cmd_vel`。
- 有相关障碍物时，由局部避障取得控制权。
- 障碍物阻塞、输入超时或数据非法时，最终命令归零。
- 科目一启动时不默认发布 `map→odom`，必须在获得经审核的对齐结果后显式更新。

详见[科目一局部避障使用说明](docs/科目一_局部避障使用说明.md)。

## 独立构建

两个科目使用独立的 `build/install/log` 目录，互不覆盖；生产构建不包含测试工具包 `ugv_mvp_tools`。

```bash
# 科目二最小包：
# ugv_localization_mvp ugv_subject2_mvp ugv_subject2_bringup
scripts/build_subject2.sh --clean
source install_subject2/setup.bash

# 科目一最小包：
# ugv_localization_mvp ugv_subject1_perception_mvp
# ugv_subject1_avoidance_mvp ugv_subject1_bringup
scripts/build_subject1.sh --clean
source install_subject1/setup.bash
```

构建产物分别位于：

- 科目二：`build_subject2/`、`install_subject2/`、`log_subject2/`
- 科目一：`build_subject1/`、`install_subject1/`、`log_subject1/`

具体启动命令、参数、话题检查、TF 检查与调试步骤都放在对应科目的使用说明中。

## 仓库结构

```text
src/           ROS 2 功能包
scripts/       独立构建、仓库校验和离线夹具脚本
docs/          两科使用说明与实车接口/待办
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

# 夹具只发布到 /fixture/cmd_vel，不授权真实车辆输出
ROS_DOMAIN_ID=171 scripts/run_fixture_smoke.sh subject2
ROS_DOMAIN_ID=174 scripts/run_fixture_smoke.sh subject1
```

## 文档

- [科目二自主导航：代码逻辑、构建、启动与调试](docs/科目二_自主导航使用说明.md)
- [科目一局部避障：代码逻辑、构建、启动与调试](docs/科目一_局部避障使用说明.md)
- [实车接口、TF、测量回填与剩余待办](docs/实车接口与待办.md)

## 安全边界

本仓库完成的是软件 MVP 与离线/断执行器验证，不等于实车放行。以下条件未满足时，禁止发送非零实车控制：

1. 雷达白名单、雷达到 `base_link` 的实测静态 TF 及记录编号已经审核；
2. 车辆尺寸、速度、转向、制动距离和命令延迟已测量并回填；
3. `/cmd_vel` 只有一个最终发布者，底盘侧独立看门狗有效；
4. 断定位、断点云、输入超时、非法数值和急停测试均在断执行器条件下通过；
5. 获得现场负责人批准后，才可按限速参数进行低速测试。

未闭合项和合作方输入要求统一记录在[实车接口与待办](docs/实车接口与待办.md)。
