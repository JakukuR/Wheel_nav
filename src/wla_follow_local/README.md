# 当前深度与地图查询数值扩展

只构建 `_wla_sim_geometry`：深度反投影、地面/孤立簇过滤、射线槽及距离场查询。
当前 tracker 与 MPC 位于 wla_person_follow；已删除旧 `_wla_follow_local` 的局部规划器、
TargetKalman 和 object_close，避免并存两套跟随控制链。
