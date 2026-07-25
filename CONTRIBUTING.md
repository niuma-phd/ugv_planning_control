# 贡献指南

## 分支与范围

- `main` 只接收完成新鲜验证的集成结果。
- 使用短期分支，例如 `feat/subject2-controller` 或 `fix/subject1-timeout`。
- 每次改动限定到明确的功能包或文档，避免同时重构无关代码。
- 不提交 rosbag、点云、视频、大日志、设备密钥或真实雷达识别码。

## 提交前验证

先运行仓库结构检查：

```bash
python3 scripts/verify_repository.py
```

改动仅涉及一个科目时，先做对应的独立 Release 构建：

```bash
scripts/build_subject2.sh --clean  # 科目二
scripts/build_subject1.sh --clean  # 科目一
```

涉及共享定位、任一科目的启动包、接口或发布前集成时，再运行全仓构建和测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

按改动范围补充离线闭环检查：

```bash
ROS_DOMAIN_ID=171 scripts/run_fixture_smoke.sh subject2
ROS_DOMAIN_ID=174 scripts/run_fixture_smoke.sh subject1
```

夹具命令被重映射到 `/fixture/cmd_vel`，不能把夹具、回放或断执行器结果描述成实车测试通过。

## 提交说明

提交或 PR 必须写明：

- 目标、修改范围和基线提交；
- 新增或调整的参数；
- 实际执行的构建、测试命令与平台；
- 是否使用真实数据、是否连接执行器；
- 剩余风险和实车门禁状态。

启动、调试和接口约束以以下三份文档为准：

- [科目二自主导航使用说明](docs/科目二_自主导航使用说明.md)
- [科目一局部避障使用说明](docs/科目一_局部避障使用说明.md)
- [实车接口与待办](docs/实车接口与待办.md)
