# 单次运行完整可视化

## 概述

1. 安装依赖：`python -m pip install -r offline_test/visual/requirements.txt`
2. 先完成一次新版 Pipeline 运行，使 Evaluator 生成按模块拆分的 `*_comparison.csv`。
3. 显式传入运行目录：

   `python offline_test/visual/visualize_run.py offline_test/output/run_...`

结果写入指定运行目录下的 `visual_report/`。其中 `README.md` 解释每张图的横轴、纵轴、单位和含义。

核心报告按 Observation、Solver、Tracker、Aimer、Command 和物理命中组织；运行时间、状态机、装甲板关联等放在调试附录。脚本不自动选择“最新运行”，以保证报告可复现。


## 每张图含义

• 新版 visualize_run.py 一共生成 10 张图片。整体分为两部分：

  - 01～09：核心性能报告
  - 10：调试诊断附录

  所有时序图的横轴都是“算法产生当前输出的时间”，单位为秒。Aimer 和 Command 虽然横轴是当前时
  间，但它们的 Ground Truth 取自预测命中时刻。

### 01_observation.png

  用于观察进入 Solver 之前的二维装甲板检测质量，共四个子图。

#### Observation corner error

  四个装甲板角点的像素 RMSE：

  corner_rmse_px

  它综合反映 8 个分量，也就是 4 个角点的 u/v 偏差。越接近 0，二维角点观测越准确。

#### Observation center signed error

  装甲板中心的像素有符号误差：

  - u：水平方向
  - v：竖直方向

  正负号可以帮助判断是否存在固定方向的系统性偏移。例如 u 长期大于 0，说明观测中心长期偏向理想
  位置的一侧。

#### Observation size signed error

  观测边界框相对理想边界框的：

  - 宽度误差
  - 高度误差

  如果宽度误差长期为正，表示检测框通常偏宽。这个误差会影响由二维尺寸估计深度的算法。

#### Observation detection state

  每块本应可见的装甲板是否被检测到：

  - 1：检测到
  - 0：漏检

  这个子图主要用于定位观测丢失时刻。

  ———

### 02_solver_position.png

  展示 Solver 对“当前装甲板中心位置”的估计，共 3×2 个子图。

  每一行对应一个坐标轴：

  - 第一行：x
  - 第二行：y
  - 第三行：z

  左列叠加：

  - Solver 估计值
  - 当前装甲板 Ground Truth

  右列展示有符号误差：

  error = estimate - truth

  坐标含义是：

  - x：前方距离
  - y：左右方向
  - z：竖直方向

  这张图适合判断 PnP、相机参数、坐标变换等环节带来的误差。例如：

  - x 误差主要可能表现为深度估计问题
  - y 长期偏正或偏负可能与外参或水平像素偏差有关
  - z 偏差可能与相机安装姿态、竖直像素误差有关

  ———

### 03_solver_yaw.png

  展示 Solver 对当前装甲板 yaw 的估计。

  左图：

  - Solver 估计的装甲板 yaw
  - 装甲板真实 yaw

  右图：

  - yaw 有符号误差

  单位是弧度。

  yaw 属于周期角，因此 Evaluator 中使用的是经过角度环绕处理的误差，避免在 -π 和 +π 附近产生接
  近 2π 的虚假误差。

  ———

### 04_tracker_position.png

  展示 Tracker 对“当前目标旋转中心”的位置估计。

  结构同样是 3×2：

  x estimate vs truth | x signed error
  y estimate vs truth | y signed error
  z estimate vs truth | z signed error

  注意它与 Solver 图的物理对象不同：

  - Solver 比较当前装甲板中心
  - Tracker 比较目标旋转中心

  这张图主要回答两个问题：

  1. Tracker 是否正确恢复了目标中心？
  2. Solver 的观测误差经过跟踪滤波后，是被抑制了还是被放大了？

  例如 Solver 的 y 偏差和 Tracker 的 y 偏差基本一致，说明误差直接传播到了跟踪层。

  ———

### 05_tracker_velocity.png

  展示 Tracker 对目标中心三轴速度的估计。

  共 3×2 个子图：

  vx estimate vs truth | vx error
  vy estimate vs truth | vy error
  vz estimate vs truth | vz error

  单位均为 m/s。

  它将速度和位置拆开绘制，避免把 m 和 m/s 混在同一个坐标轴中。

  这张图适合观察：

  - Tracker 速度收敛时间
  - 静止目标是否出现虚假速度
  - 匀速目标能否稳定跟随
  - 观测噪声是否导致速度剧烈振荡
  - 切换装甲板后速度是否发生跳变

  对于静止目标，三轴真值通常都是 0；估计曲线越稳定地贴近 0 越好。

  ———

### 06_tracker_rotation_model.png

  展示 Tracker 的旋转状态和目标模型参数，共五个子图。

#### Yaw

  目标旋转角：

  - Tracker 估计 yaw
  - 真实 yaw

#### Yaw rate

  目标旋转角速度：

  - 估计 yaw rate
  - 真实 yaw rate

  单位是 rad/s。

  小陀螺场景中，这个量直接决定未来装甲板位置预测是否准确。

#### Radius

  目标中心到主要装甲板的旋转半径。

  如果半径估计错误，即使中心位置和 yaw 正确，重建出的装甲板位置仍然会偏移。

#### Radius delta

  模型中两组装甲板的半径差异。

  通常用于描述不同装甲板组并非处于完全相同的旋转半径上。

#### Height delta

  不同装甲板组之间的高度差。

  如果目标模型具有交替高度的装甲板，这个参数会影响装甲板 z 坐标的预测。

  这张图目前主要叠加估计值与真值，没有单独设置误差列，目的是保持参数总览紧凑。

  ———

### 07_aimer_future_position.png

  展示 Aimer 预测的“未来命中时刻装甲板位置”。

  结构是 3×2：

  predicted x vs future GT | x error
  predicted y vs future GT | y error
  predicted z vs future GT | z error

  这里的两个时间非常重要：

  - 横轴：Aimer 产生预测的当前时间
  - GT 参考时间：预测的子弹命中时间

  也就是说，某个横坐标为 t 的数据点比较的是：

  t 时刻 Aimer 给出的未来位置
  vs
  impact_timestamp_ns 时刻的真实装甲板位置

  它不是拿未来预测位置和当前装甲板位置比较。

  这张图直接反映：

  - Tracker 状态误差
  - 速度和转速估计误差
  - 装甲板模型参数误差
  - 弹丸飞行时间估计误差
  - 装甲板选择或切板错误

  最终如何传播到瞄准点。

  ———

### 08_command.png

展示最终下发的云台角度与物理念想角，共两行。

#### Command yaw

  左图：

  - 实际命令 yaw
  - 根据未来真实装甲板位置计算的理想 yaw

  右图：

  - yaw 有符号误差

#### Ballistic command pitch

  左图：

  - 实际命令 pitch
  - 根据未来装甲板位置和弹道模型算出的理想 pitch

  右图：

  - pitch 有符号误差

  这里比较的是弹道理想 pitch，不是简单的视线 LOS pitch。

  区别是：

  - LOS pitch：仅对准装甲板几何位置
  - 弹道 pitch：考虑重力、空气阻力、弹速和枪口位置

  因此弹道 pitch 才是判断实际命令是否能命中的主要参考。

  ———

### 09_physical_hits.png

  展示实际发射事件的物理命中结果，共两个子图。

#### Physical shots

  每个散点代表一发实际开火：

  - 横轴：命令产生时间
  - 纵轴：子弹轨迹与预期装甲板之间的最小距离
  - 绿色：判定命中
  - 红色：判定未命中

  纵轴越小，说明弹丸轨迹越接近装甲板。

  它不仅检查角度误差，还把以下因素组合到一次物理仿真中：

  - yaw/pitch 命令
  - 子弹速度
  - 枪口延迟
  - 重力与阻力
  - 装甲板未来运动
  - 装甲板实际几何尺寸

#### Shot outcome counts

  柱状图统计：

  - Hit：命中
  - Miss：可评价但未命中
  - Unevaluable：因真值时间范围等原因无法可靠评价

  它用于快速观察总体开火结果，但不能替代逐发脱靶距离分析。

  ———

### 10_debug_appendix.png

  这张图用于调试，不作为主要性能结论。

#### Module runtime

  展示每帧各模块耗时：

  - Solver + Tracker
  - Aimer
  - Shooter
  - Total

  单位为毫秒，用于发现耗时尖峰和性能瓶颈。

#### Tracker state

  设计意图是展示 Tracker 状态机：

  - lost
  - detecting
  - tracking
  - temp_lost

  不过当前实现直接尝试从字符串状态绘制数值曲线，这个子图还需要补一层“状态名称到整数编码”的转
  换，否则可能为空。这是当前脚本中需要继续修正的地方。

#### Armor association audit

  比较：

  - Aimer 内部装甲板 ID
  - Evaluator 匹配到的物理 Ground Truth 装甲板 ID

  它用于排查：

  - 切板时刻是否合理
  - 内部编号与物理编号是否存在相位差
  - Aimer 是否选择了错误的未来装甲板

  两条 ID 曲线不一致不一定代表错误，因为内部编号和物理 GT 编号可能采用不同起始相位。

#### Pitch reference diagnostic

  比较：

  - 几何 LOS pitch
  - 弹道理想 pitch

  两条曲线之间的差异表示为了补偿弹道下坠而需要增加的 pitch 提前量。

  它主要用于验证弹道模型和符号约定，不应直接当作算法误差。

  总体上，这套图可以沿着下面的链路排查误差：

  二维像素观测
  → Solver 当前装甲板位置
  → Tracker 当前目标中心和运动状态
  → Aimer 未来装甲板位置
  → Command 控制角
  → 子弹物理命中

如果前一层已经出现明显的固定偏差，而后续层出现相似偏差，通常说明误差在传播；如果前一层正常、下一层突然恶化，则更可能是下一模块自身的模型或时间对齐问题。
# 六阶段误差传播分析

运行：

```bash
python3 offline_test/visual_analyse/analyse_error_propagation.py \
  offline_test/output/run_...
```

如果当前目录已经是 `offline_test`：

```bash
python3 visual_analyse/analyse_error_propagation.py output/run_...
```

脚本会在该次运行的 `error_propagation_report/` 下生成六个距离指标的 CSV、分子图时序图和按同一 Solver 输入帧对齐的叠加图。弹道层对每个有效 Aimer 帧执行一次虚拟发弹，不要求 Shooter 实际开火。
