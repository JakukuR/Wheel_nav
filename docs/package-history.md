# 原源码包提交历史

2026-10-09，将 6 个独立源码仓库的 24 条原提交接入工作区 `main`。采用保留当前文件树的历史合并，原提交号、作者、日期、父提交及历史文件内容均保留；没有改写已发布历史或强推。

导航包原有历史此前已被工作区继承。`wla_person_follow` 原本没有独立 Git 仓库。

| 当前路径 | 原分支 | 原提交数 | 原仓库最后提交 |
| --- | --- | --- | --- |
| `src/explore` | `main` | 1 | [33147e8](https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav/commit/33147e849ab1dab82ecc5d0d448c51e196019fdf) |
| `src/explore_lite_msgs` | `main` | 1 | [3ad27e7](https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav/commit/3ad27e780164061dfe9ed8078d5e18dc8fdc86e1) |
| `src/gamepad_contorl` | `main` | 2 | [df6ab76](https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav/commit/df6ab7608ef95765e2e5d2726b940b80b3ef6dfc) |
| `src/wla_cuvslam_navigation` | `main` | 8 | [b8c50d1](https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav/commit/b8c50d15f3d4ecb4476b03290cc1b2ecfc712dfb) |
| `src/wla_cuvslam_validation` | `master` | 5 | [3ccfa19](https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav/commit/3ccfa19f72316279179de5e603650e57169c90e6) |
| `src/wla_diff_mpc` | `main` | 7 | [caece7f](https://github.com/OnanaRobotics-MMAgent/Wheel_Legged_Nav/commit/caece7f8d76521b3bde04d2ec15a79f33ac37888) |

原历史中的文件路径仍按原包根目录记录，当前源码位于工作区 `src/<包名>/`。保留原提交号意味着历史路径也保留；只按当前子目录查询的文件历史不一定显示全部原记录。查看原仓库历史时，使用表中的原提交号：

```bash
cd ~/ros2_ws
git log --oneline caece7f8d76521b3bde04d2ec15a79f33ac37888
git show caece7f8d76521b3bde04d2ec15a79f33ac37888:README.md
git log --graph --oneline --all
```

每个历史合并提交记录 `Package path`、`Original branch` 和 `Original tip`，可从工作区提交图定位。原包的 `.git` 和未提交工作仍保留在 Orin；后续统一提交请在 `~/ros2_ws` 根目录操作。

历史合并的第一父提交文件树完全相同；仅随后新增本索引及工作区 README 的链接，没有重新部署或执行运动测试。
