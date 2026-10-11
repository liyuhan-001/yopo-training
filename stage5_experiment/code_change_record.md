# 第五阶段代码与参数记录

## 本次两轮汇总更新（2026-10-11）

1. 保留此前的 `stage5_logger.py`、`test_yopo_ros.py`、`config.yaml`、`traj_opt.yaml` 四份原始快照，不修改运行逻辑。
2. 增加 15 份 trial02 轨迹，汇总扩展为 30 次；保留全部 trial01。
3. 新增 `analysis/analyze_stage5.py`，核对实验组合、名称与参数、起终点、单调时间戳、每点速度和累计三维路径，并重算全记录期平均速度及目标误差。
4. 更新三组的均值和样本标准差；增加两轮配对展示、时间拆分与轨迹图。0.1 m 分界仅用于离线诊断，未回写或改变记录器的计时标准。

## 既有代码快照中可确认的功能

| 文件 | 可确认的内容 |
|---|---|
| stage5_logger.py | 订阅里程计、目标、点云和推理耗时；收到目标后开始记录；输出轨迹 CSV 并追加 summary；默认 success_radius=5 m |
| stage5_logger.py | run_id、tree_dist、seed、speed 来自启动参数。名称不会自动改变地图 seed，也不会自动更改 YAML |
| test_yopo_ros.py | 目标初值为 [30,0,2]，支持目标话题；发布 /yopo/inference_ms；前向计时使用 perf_counter，CUDA 路径在前后同步 |
| config.yaml | 当前保留的最终设置为 random_map=true、maze_type=5、tree_dist=6、seed=5 |
| traj_opt.yaml | velocity=6.0，训练速度和加速度上限设置均为 6.0 |

本表描述实际提供的代码内容，不把未提供基线的内容虚构成逐行修改记录。规划器 `--trial=1 --epoch=50` 选择权重；记录器名称中的 trial01/trial02 表示实验重复轮次，两者含义不同。

## 需要保留的实验边界

四份快照来自第一次提交，第二轮压缩包只含 CSV。可以确认它们在本次打包时未改变，不能由此独立证明第二轮 Ubuntu 实际使用的文件版本。后续运行宜同时保存配置哈希、地图和代码提交号。
