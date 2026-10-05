# YOPO 第五阶段：随机森林树间距实验

本目录保存本阶段的实验设计、数据、运行代码快照和结果分析。实验于 2026-10-05 整理，数据来自提交的 `stage5_submission_20261005.tar.gz.zip`。

## 内容

- `summary.csv`：15 次运行的汇总记录。
- `raw/`：15 份原始里程计轨迹，原样保留。
- `current_config_snapshot/`：实验脚本与两份配置文件的快照。配置文件展示最后一次实验设置；逐次运行的 seed 与 tree distance 以 `summary.csv` 为准。
- `analysis/stage5_density_comparison.png`：每个 seed 的指标点及均值和样本标准差。
- `analysis/stage5_submission_text.md`：实验开始前填写与实验完成后回答的可复制文字。

## 实验设置

随机森林场景（`maze_type=5`），`tree_dist` 为 3、4、6 m，每种间距各运行 seed 1–5；目标速度为 6 m/s，起点约为 `[0, 0, 2.008]`，目标为 `[30, 0, 2]`。记录器使用 5 m 成功半径，因此报告中的成功表示进入目标判定区域。

汇总表与全部 15 份原始轨迹已交叉检查。每组 5 次运行均被记录为成功，最终误差范围约为 4.954–4.998 m。
