# YOPO 第五阶段：两轮随机森林树间距实验

2026-10-11 更新。包含 trial01 与 trial02 的 30 次运行：tree_dist 为 3、4、6 m，每种间距 5 个 seed，每个 seed 重复 2 次。

30 次记录均进入目标点周围 **5 m 判定区域**。原始总耗时包含起点附近的等待阶段；不能直接用它比较纯飞行效率。完整分析见[实验报告](analysis/stage5_submission_text.md)。本阶段不进行碰撞次数分析。

| 提交材料 | 文件 |
|---|---|
| 可复制的实验前填写、实验后回答、分析和个人总结 | [完整报告](analysis/stage5_submission_text.md) |
| 实验设计表 | [experiment_design.md](experiment_design.md) |
| 原始记录 | [summary.csv](summary.csv) 与 [raw/](raw/) 下 30 份轨迹 |
| 参数和运行代码快照 | [current_config_snapshot/](current_config_snapshot/) |
| 代码修改说明 | [code_change_record.md](code_change_record.md) |
| 结果图表 | [总览](analysis/stage5_density_comparison.png)、[时间拆分](analysis/stage5_time_decomposition.png)、[轨迹投影](analysis/stage5_trajectories.png) |
| 统计明细 | [组统计](analysis/group_statistics.csv)、[单次指标](analysis/run_metrics.csv)、[时间诊断](analysis/timing_diagnostics.csv) |
| 数据核对及文件哈希 | [validation.json](analysis/validation.json)、[source_manifest.json](source_manifest.json) |
| GitHub 更新记录 | [github_update_record.md](github_update_record.md) |

![结果总览](analysis/stage5_density_comparison.png)

## 数据与统计口径

- 新数据来自用户提供的 `stage5_experiment.zip`；原始 CSV 按字节保留，未重新命名或更改测量值。15 份 trial01 与上次提交一致。
- 组统计是每种树间距 10 次运行的等权均值及样本标准差，包含 5 对同 seed 重复测量。不是 10 个独立随机地图；未作显著性检验。
- 运行代码及 YAML 沿用 2026-10-05 的四份快照，本次数据包未提供新代码。最终 config.yaml 为 td6/seed5，不能代替每次运行的地图配置证明。
- 统计表保留原始 flight_time_s；新增时间诊断以相对起点首次位移超过 0.1 m 为分界，仅供解释等待影响。
- 原汇总中的碰撞相关列保持原样，但不参与本次结果评价，也不据此宣称零碰撞。

## 复现报告与图表

在已具备 Python 3.8+、numpy、matplotlib 的环境中，从本目录执行：

```bash
python analysis/analyze_stage5.py
```

脚本检查 30 条汇总及全部轨迹采样，生成统计、时间诊断、报告和三张图，不修改 summary.csv 或 raw/。中文图需要 Microsoft YaHei、SimHei、Noto Sans CJK SC 或 WenQuanYi Micro Hei 中的一种字体。更换数据集或追加第三轮时，需要明确更新脚本的实验组合检查和报告范围。

这份分析不重新运行 ROS 仿真，也不更改控制器和规划器。本阶段更新发布在 [dev 分支](https://github.com/liyuhan-001/yopo-training/tree/dev/stage5_experiment)。
