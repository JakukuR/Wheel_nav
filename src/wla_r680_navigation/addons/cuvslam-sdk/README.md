# cuVSLAM v17 视觉位姿来源补丁

导航前端使用新增的 `Odometry::IsLastPoseInertialOnly()`，区分视觉解算与纯 IMU 推算。必须先应用本补丁并重新构建 SDK，再构建 `wla_cuvslam_navigation`。未修补的 SDK 无此符号，不能与本前端混用。

`visual-pose-source.patch` 包含 C++ API、Python 绑定源码及两项 API 回归；`sdk-base-sha256.json` 保存本次使用的 SDK 原始文件校验值。新增文件的原始校验值为 null。SDK 基于 Orin 上的 cuVSLAM v17 源码包，没有可记录的 Git 提交号。这里只归档补丁，不包含 SDK 二进制、测试数据或完整 SDK 源码。

在对应 SDK 源码根目录中检查并应用：

```bash
git apply --check /path/to/wla_r680_navigation/addons/cuvslam-sdk/visual-pose-source.patch
git apply /path/to/wla_r680_navigation/addons/cuvslam-sdk/visual-pose-source.patch
cmake --build /path/to/sdk_build --target cuvslam -j2
```

重新构建时沿用已验证的 SDK 配置与构建类型。SDK 的公共头文件和运行时 `libcuvslam.so` 必须来自同一次修补构建。Python 绑定源码同步更新，但本轮未重装 Python 环境。已在 Orin 运行两项 API 测试；它们验证初始化与非惯性模式分支，不能替代真实 PnP 退化场景验收。
