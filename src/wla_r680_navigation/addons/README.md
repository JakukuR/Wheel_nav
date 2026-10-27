# cuVSLAM SDK 补丁

- `cuvslam-sdk/`：`src/wla_cuvslam_navigation` 前端所需的 cuVSLAM v17 API 补丁与原始文件校验清单。

cuVSLAM 模式需先按 [SDK 补丁说明](cuvslam-sdk/README.md) 修补、重建 SDK，再按
[前端说明](../../wla_cuvslam_navigation/README.md) 构建前端与导航包；MPC 插件见
[wla_diff_mpc](../../wla_diff_mpc/README.md)。

2026-10-27 起这里不再保存 `wla_diff_mpc`、`wla_cuvslam_navigation` 的副本，两包源码只在
工作区 `src/` 下维护。`COLCON_IGNORE` 保留，避免 colcon 扫描本目录。
