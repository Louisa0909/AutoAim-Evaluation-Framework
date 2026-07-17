# 单次运行完整可视化

1. 打开 `visualize_run.py`，把顶部的 `RUN_DIR` 改成需要分析的 `offline_test/output/run_*` 目录。
2. 安装依赖：`python -m pip install -r offline_test/visual/requirements.txt`
3. 运行：`python offline_test/visual/visualize_run.py`

结果写入指定运行目录下的 `visual_report/`。其中 `README.md` 解释每张图的横轴、纵轴、单位和含义。

脚本故意不自动选择“最新运行”，以保证同一份代码始终对应明确的一次测试。
