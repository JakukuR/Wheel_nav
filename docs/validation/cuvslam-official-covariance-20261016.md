# cuVSLAM 协方差与 Track 异常按官方方式处理 — 2026-10-16

## 问题

`nav-20261015-170936-2391013` 在 17:49:39 返回负协方差。
接入层在记录 `invalid_sdk_covariance` 后主动抛异常，外层 catch 设置
`stopped_=true`，导致整个处理循环退出。该停止行为不是官方 SDK 的返回策略。

## 改动与来源

- SDK 协方差异常只写既有诊断日志，不抛异常、不跳帧、不更改健康状态。
- `Track()` 的异常在当前帧内捕获，打印警告并继续下一帧，保留 tracker；
  对应官方 ROS `UpdatePose()` 的 catch + return 行为。
- Odometry 的 pose/twist covariance 使用官方 `PoseCache/VelocityCache`，
  缓存少于 10 个样本时按官方实现输出单位矩阵。SDK 原始协方差不进入 odom。
- SDK 返回空位姿时按官方处理清空 pose cache；正常帧恢复后重新积累。
- 保留现有 body-frame 速度估计，将实际发布的 twist 输入官方速度缓存。
  本次仅迁移协方差与 Track 异常处理，不宣称整个 ROS 接入层等同官方节点。

直接复用 NVIDIA-ISAAC-ROS/isaac_ros_visual_slam 提交
`9058506efd0b967ac894a717bdbe98df017637e5` 的原始文件，未修改其算法：

- `pose_cache.cpp`: SHA-256 `37f7ab6438081f995c16d7d51d07e911c22272a43d6bcfce0f90a56e1017db0c`
- `pose_cache.hpp`: SHA-256 `e88a67835baafe2943be2012fd48983c9fd7e2281690c6bfe11fe33f2f935d93`

源码与 Apache-2.0 许可证存放在包内 `third_party/isaac_ros_cuvslam/`。
SDK 对照版本：nvidia-isaac/cuVSLAM `2a0882cfada6448632f37182a4b8a270adb2ec3d`。
ROS 与 SDK 接口版本不同；引入的缓存代码没有 SDK 依赖。

## 验证与安装

- 在 Orin ROS 2 Jazzy 环境编译 `wla_cuvslam_navigation` 成功。
- CTest 5/5 通过，包括原有 visual_pose_evidence、health_evidence、
  odometry_contract、initialization_sampling。
- 新增回归 harness 编译生产源码中的实际 Track catch、协方差诊断、
  odom 协方差赋值及空位姿处理片段，链接未修改的官方缓存实现。
  验证第一次 Track 抛异常后第二帧仍被处理；捕获的负对角矩阵和 NaN
  不终止处理；前 9 帧单位矩阵；第 10 帧使用可解析的历史样本方差；
  静止样本协方差为零；空位姿清空 pose cache。
- 此 harness 使用假的 SDK 返回值，不替代真实 SDK 遮挡恢复或实车验收。
- 安装到 `r680_nav.sh` 引用的 `.runtime/cuvslam_navigation/install`，
  下次启动导航加载。没有重启底盘、启动导航或进行运动测试。
- 日志：`.runtime/cuvslam_navigation/official-covariance-build.log`、
  `.runtime/cuvslam_navigation/official-covariance-test.log`。

SDK 本身的数值异常根因与 watchdog 恢复期间再故障的行为不属于本次修改。
工作区已有的 Agent gateway、OpenVINS launch/config 改动保持独立，不纳入本次提交。
