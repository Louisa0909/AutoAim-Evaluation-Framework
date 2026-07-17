# 数据接口规范 1.0

本目录的规范是测评系统与被测自瞄程序之间的稳定边界。动态记录使用 JSONL（一行一个 JSON 对象），静态元信息使用 JSON 兼容的 YAML 1.2。所有文件均为 UTF-8。

## 全局约定

- 时间戳：`int64` 纳秒。
- 距离：米；速度：米每秒。
- 角度：弧度；角速度：弧度每秒。
- 世界坐标：右手系，x 向前、y 向左、z 向上。
- 控制指令沿用现有 Aimer：yaw 向左为正，pitch 向下为正（抬头为负）。
- 四元数顺序：`[w,x,y,z]`，表示 gimbal 到 world 的旋转。
- 装甲板角点：左上、右上、右下、左下。
- 真值只读，算法输出不得写入 `ground_truth/`。
- `target_hint_id`、`armor_hint_id` 只供评价关联，不得用于算法决策。
- C++ backend 只能读取 `metadata.yaml`、`frames.jsonl`、`observations.jsonl`，不得读取 `ground_truth/`。

运行时 Pipeline 会把上述三个公开文件复制到 `output/run_xxx/algorithm_input/`，backend 只接收这个目录。`target_hint_id` 和 `armor_hint_id` 可以原样写入节点输出供 Evaluator 关联，但 `make_armor()`、Solver、Tracker、Aimer 和 Shooter 不得读取它们作决策。

## 数据集布局

```text
case_name/
├── metadata.yaml
├── frames.jsonl
├── observations.jsonl
└── ground_truth/
    ├── target_states.jsonl
    ├── armor_states.jsonl
    ├── gimbal_states.jsonl
    └── shots.jsonl
```

## `frames.jsonl`

每个算法周期一行，即使没有观测也必须存在。

```json
{"frame_id":0,"timestamp_ns":0,"image_path":null,"imu_q_wxyz":[1,0,0,0],"bullet_speed":27.0,"valid":true}
```

必需字段：`frame_id`、`timestamp_ns`、`imu_q_wxyz`、`bullet_speed`、`valid`。`image_path` 可为空。

## `observations.jsonl`

每帧一行；漏检帧写 `"armors":[]`，不能删除该帧。

```json
{"frame_id":0,"timestamp_ns":0,"armors":[{"observation_id":0,"target_hint_id":1,"armor_hint_id":0,"class_id":6,"color":"blue","name":"two","armor_type":"small","confidence":1.0,"corners_px":[[710,486],[781,486],[781,516],[710,516]],"bbox_xywh":[710,486,71,30],"visible":true,"valid":true}]}
```

## `target_states.jsonl`

每个目标、每个真值时刻一行。目标状态与当前 EKF 对齐：

```json
{"timestamp_ns":0,"target_id":1,"position":[5,0,0.5],"velocity":[0,0,0],"yaw":0,"yaw_rate":2,"radius":0.2,"radius_delta":0.03,"height_delta":0.02,"armor_count":4,"valid":true,"source":"generated","confidence":1.0}
```

EKF 映射为：`x,vx,y,vy,z,vz,yaw,yaw_rate,radius,radius_delta,height_delta`。

## `armor_states.jsonl`

每块装甲板、每个真值时刻一行。

```json
{"timestamp_ns":0,"target_id":1,"armor_id":0,"position":[4.8,0,0.5],"yaw":0,"armor_type":"small","name":"two","visible":true,"attackable":true,"valid":true}
```

`visible` 表示成像可见，`attackable` 表示当前朝向适合击打，两者不能混用。

## C++ 算法输出

- `algorithm_output.jsonl`：每帧 Tracker、EKF、Aimer、Command 和耗时。
- `solver_output.jsonl`：每个观测对应的 Solver 三维位置和 yaw。
- `shots.jsonl`：每次 Shooter 开火事件及命中结果。

完整样例由 `python run_pipeline.py --dataset-only` 自动生成，生成文件就是规范的可执行示例。

## Python 评价输出

- `frame_errors.csv`：Tracker、EKF、Aimer 的逐帧误差。
- `solver_errors.csv`：逐装甲板 Solver 误差。
- `summary.json`：Solver、Tracker、Aimer、Shooter、命中率和时延统计。
- `position_error.svg`：Tracker 位置误差曲线。

## Aimer与物理命中指标

- `aim_point_error`：Aimer预测未来装甲板位置与对应未来真值的三维误差。
- `command_yaw_error`：命令 yaw 与未来目标方向的环绕角误差。
- `line_of_sight_pitch_difference`：命令 pitch 相对纯几何视线角的差值，仅作诊断，不代表算法错误。
- `ideal_ballistic_pitch`：Evaluator 使用独立 RK4 数值积分、重力和等效二次阻力求出的参考 pitch。
- `ballistic_pitch_error`：命令 pitch 与独立弹道参考 pitch 的环绕角误差。
- `physical_miss_distance`、`hit_rate`：从真实/配置枪口状态按命令发射，并与运动装甲板平面求交得到。真值没有覆盖完整飞行区间的射击记为 `unevaluable_shots`，不进入命中率分母。

参考模型不得调用被测 Aimer 内部的 `Trajectory`。当前生成数据使用配置中的理想云台；真实数据应补充实际出膛时间、实测弹速、实际云台角和枪口位置。

## 合成图像投影

合成观测必须使用与被测 Solver 相同的 `camera_matrix`、五参数畸变、`R_gimbal2imubody`、`R_camera2gimbal` 和 `t_camera2gimbal`。Python 生成器独立复现 `Solver::reproject_armor()` 的正向几何，包括普通装甲板 15° pitch（前哨站为 -15°），但 C++ backend 仍只读取最终的 `observations.jsonl`，不会读取真值。
