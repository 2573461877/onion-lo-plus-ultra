# Onion GPS Trajectory Registration and Evaluation

该包是独立的 C++/ROS1 旁路工具，不修改 Onion 建图、固定地图定位、全局重定位或
自动驾驶 GPS 驱动。包内不再用一个 `mode` 参数切换全部逻辑，而是用三个名字和职责
明确的节点、三个 launch 分别完成轨迹配准、固定配准关系下的定位评估，以及单段连续
数据的分段配准并评估。

## 1. 三项功能及其边界

### 1.1 轨迹配准

输入 GPS 与 Onion 在同一时段、同一路径上的位姿，按时间插值匹配后，用
RANSAC + SVD 计算固定的平面刚体变换和高度偏移：

```text
p_gps = Rz(yaw_offset) * p_onion + [tx, ty, tz]
```

结果保存为 `map_to_gps_alignment.json`。它描述 Onion 地图坐标到 UTM（或局部 ENU）
的固定关系，不是逐帧轨迹，也不是 GPS 硬件外参。配准会消除地图任意原点和朝向；
同一批配准点的残差只能检查配准质量，不能作为独立定位精度。

对应文件：

- `src/trajectory_registration_node.cpp`
- `launch/trajectory_registration.launch`

启动：

```bash
roslaunch onion_gps_evaluation trajectory_registration.launch
```

bag 播放结束后生成结果：

```bash
rosservice call /onion_gps_trajectory_registration/finalize "{}"
```

默认输出：

```text
$(find onion_lo_plus)/results/gps_evaluation/registration/
  map_to_gps_alignment.json
  matched_trajectory.csv
  summary.json
  summary.csv
  relocalization_events.csv
  status_events.csv
```

如需指定路径：

```bash
roslaunch onion_gps_evaluation trajectory_registration.launch \
  output_directory:=/data/gps_eval/site_registration
```

### 1.2 固定地图定位精度评估

加载已经生成并冻结的 `map_to_gps_alignment.json`，在已有点云地图中运行 Onion
定位，将 Onion 位姿变换到 GPS 坐标系，然后逐帧与 GPS 比较。评估阶段不会重新拟合
或更新变换，输出水平、垂直、三维、航向、1 s RPE、匹配率、阈值通过率以及重定位
收敛指标。

评估 bag 应来自相同物理地图覆盖区域，但不能参与本次加载文件的配准。它可以是同一
道路的另一次采集、反向行驶，或地图覆盖范围内的另一段路线。

对应文件：

- `src/localization_accuracy_evaluator_node.cpp`
- `launch/localization_accuracy_evaluation.launch`

启动：

```bash
roslaunch onion_gps_evaluation localization_accuracy_evaluation.launch
```

launch 默认加载：

```text
$(find onion_lo_plus)/results/gps_evaluation/registration/map_to_gps_alignment.json
```

使用其他配准文件：

```bash
roslaunch onion_gps_evaluation localization_accuracy_evaluation.launch \
  alignment_input_path:=/data/gps_eval/site_registration/map_to_gps_alignment.json \
  output_directory:=/data/gps_eval/localization_run_01
```

bag 播放结束后：

```bash
rosservice call /onion_localization_accuracy_evaluation/finalize "{}"
```

### 1.3 单段连续轨迹分段配准并评估

适用于只有一个连续 rosbag 的联调场景。节点先按时间顺序匹配完整轨迹，然后：

1. 前 `registration_fraction` 部分只用于估计固定变换。
2. 严格晚于配准结束时刻的剩余部分只用于计算定位误差。
3. 后段评估点不会反过来参与配准。

默认前 30% 配准、后 70% 评估。也可以设置正数
`registration_duration_sec`，此时以开始后的固定秒数作为配准段并优先于比例。

对应文件：

- `src/segmented_registration_evaluator_node.cpp`
- `launch/segmented_registration_evaluation.launch`

启动：

```bash
roslaunch onion_gps_evaluation segmented_registration_evaluation.launch
```

修改切分方式：

```bash
# 前 40% 配准，后 60% 评估
roslaunch onion_gps_evaluation segmented_registration_evaluation.launch \
  registration_fraction:=0.40

# 前 60 秒配准，其余时间评估
roslaunch onion_gps_evaluation segmented_registration_evaluation.launch \
  registration_duration_sec:=60.0
```

bag 播放结束后：

```bash
rosservice call /onion_segmented_registration_evaluation/finalize "{}"
```

该方式可验证整条处理链和检查后段误差，但配准段与评估段仍来自同一次采集，不替代
独立路线或测量控制点验收。

## 2. 文件与职责

| 文件 | 职责 |
| --- | --- |
| `include/onion_gps_evaluation/trajectory_alignment.hpp` | 与 ROS 解耦的位姿、匹配、配准和指标类型/API |
| `src/trajectory_alignment.cpp` | 时间插值、WGS84→ENU、RANSAC+SVD、ATE 类误差、RPE |
| `include/onion_gps_evaluation/evaluation_runner.hpp` | 三个工作流共享的 ROS 数据采集和报告接口 |
| `src/evaluation_runner.cpp` | 订阅、有效性门控、切分、JSON/CSV、Path 和重定位统计 |
| `src/trajectory_registration_node.cpp` | 仅执行轨迹配准 |
| `src/localization_accuracy_evaluator_node.cpp` | 仅加载固定变换并评估 |
| `src/segmented_registration_evaluator_node.cpp` | 同一连续数据前段配准、后段评估 |
| `src/synthetic_trajectory_publisher_node.cpp` | 发布确定性模拟轨迹，供无 GPS 硬件编译后回归 |
| `scripts/play_bag_and_finalize.sh` | 播放完整 rosbag，等待回调队列后按工作流自动调用 finalize |
| `config/gps_trajectory_evaluation.yaml` | 默认话题、时间门限、配准和指标参数 |
| `test/trajectory_alignment_test.cpp` | 纯 C++ gtest 单元测试 |

## 3. 默认输入和坐标约定

默认话题在 `config/gps_trajectory_evaluation.yaml` 中修改，launch 命令无需重复传入：

```text
/onion_lo_plus_node/odometry
/gps/evaluation/odom_utm
/gps/evaluation/fix
/gps/evaluation/position_valid
/gps/evaluation/heading_valid
/initialpose
/onion_scancontext_relocalization/status
```

默认 `gps_position_source: odom_utm`。GPS 驱动发布的位置已经是车辆后轴中心，本包
直接使用，不再次进行天线杆臂或车辆坐标平移。`NavSatFix` 只为逐帧 CSV 附加经纬高；
如果改成 `navsat_fix`，节点才会把 WGS84 转为局部 ENU。

`/gps/evaluation/position_valid` 和 `heading_valid` 的 ROS 消息均为
`std_msgs/Bool`，线上只可能出现 `true/false`。节点启动后、收到第一条 Bool 前，
C++ 内部状态名为 `not_received`；这不是话题值。报告用以下英文计数区分它与明确
的 `false`：

```text
dropped_before_position_validity_count
dropped_invalid_gps_count
suppressed_before_heading_validity_count
suppressed_invalid_gps_yaw_count
```

位置未确认有效时整帧 GPS 不参加匹配；位置有效但航向未确认有效时保留位置并将该帧
GPS yaw 设为不可用，因此不会污染航向误差。

## 4. 推荐实际流程

### 4.1 建图轨迹与 GPS 配准

1. 启动 GPS 解析和 Onion 建图/里程计。
2. 启动 `trajectory_registration.launch`。
3. 播放或采集同一时段的点云、IMU、GPS、TF。
4. 调用配准节点的 `finalize`。
5. 检查配准基线、内点数、配准 RMSE、GPS 解状态和时间同步。
6. 冻结并保存该地图对应的 `map_to_gps_alignment.json`。

配准使用整段轨迹的鲁棒拟合，不假设只靠第一帧确定朝向。车辆起点静止且认为两者位置
重合只能可靠约束平移；yaw 仍需要有效 GPS 航向、足够运动基线或测量控制点。

### 4.2 已有地图定位评估

1. 加载同一张点云地图和该地图对应的固定配准文件。
2. 启动 Onion 固定地图定位/重定位。
3. 启动 `localization_accuracy_evaluation.launch`。
4. 播放未参与配准的评估 bag。
5. 调用评估节点的 `finalize`，读取 `summary.json` 和逐帧 CSV。

禁止逐帧重新对齐，也不要对每次误差较大的路段重新拟合。否则地图全局平移/yaw
误差会被评估数据本身吸收，不能反映真实定位表现。

## 5. 输出说明

`map_to_gps_alignment.json` 主要包含：

```text
schema_version
transform_name = gps_from_onion_map
source_frame / target_frame
gps_position_source
transform.yaw_rad / yaw_deg
transform.translation_x_m / y_m / z_m
registration.matched_pose_count / inlier_count
registration.source_baseline_m
registration.horizontal_rmse_m
registration.start_stamp_sec / end_stamp_sec
navsat_reference（仅局部 ENU 需要）
```

`summary.json` 另外包含：

- 输入数量和有效性门控计数。
- 水平、垂直、3D 和航向的最小值、均值、中位数、P95、最大值和 RMSE。
- 5/10/20 cm 水平误差通过率。
- 时间匹配可用率与 1 s RPE。
- `/initialpose` 事件的首次输出延迟、初始误差、稳定收敛时间和收敛后误差。
- `accuracy_claim`，明确区分仅配准、固定变换评估和同 bag 后段留出评估。

## 6. 无硬件回归

```bash
roslaunch onion_gps_evaluation synthetic_segmented_evaluation.launch
```

它启动 C++ 分段节点和 C++ 模拟发布器，自动调用 finalize，并在以下目录生成报告：

```text
/tmp/onion_gps_evaluation_cpp_synthetic
```

模拟测试只验证编译、时间匹配、轨迹切分、配准、评估和报告链路，不代表实车定位精度。

## 7. rosbag 播放完成后自动生成报告

正常评估节点不会自行判断 rosbag 是否播放结束。推荐让
`play_bag_and_finalize.sh` 负责完整播放流程：播放前确认对应 finalize 服务已存在，
等待 `rosbag play` 正常退出，再等待默认 2 秒使 ROS 订阅回调队列处理完成，最后调用
服务。

脚本使用 `--workflow` 明确选择服务，映射关系固定为：

| `--workflow` | 必须预先启动的 launch | 自动调用的服务 |
| --- | --- | --- |
| `registration` | `trajectory_registration.launch` | `/onion_gps_trajectory_registration/finalize` |
| `evaluation` | `localization_accuracy_evaluation.launch` | `/onion_localization_accuracy_evaluation/finalize` |
| `segmented` | `segmented_registration_evaluation.launch` | `/onion_segmented_registration_evaluation/finalize` |

### 7.1 轨迹配准

终端 1：

```bash
roslaunch onion_gps_evaluation trajectory_registration.launch
```

终端 2：

```bash
rosrun onion_gps_evaluation play_bag_and_finalize.sh \
  --bag /data/registration.bag \
  --workflow registration
```

### 7.2 已有地图定位评估

终端 1：

```bash
roslaunch onion_gps_evaluation localization_accuracy_evaluation.launch \
  alignment_input_path:=/data/site/map_to_gps_alignment.json
```

终端 2：

```bash
rosrun onion_gps_evaluation play_bag_and_finalize.sh \
  --bag /data/localization_evaluation.bag \
  --workflow evaluation
```

### 7.3 同一连续 bag 分段配准并评估

终端 1：

```bash
roslaunch onion_gps_evaluation segmented_registration_evaluation.launch
```

终端 2：

```bash
rosrun onion_gps_evaluation play_bag_and_finalize.sh \
  --bag /data/continuous_run.bag \
  --workflow segmented
```

### 7.4 播放参数和队列等待时间

`--` 后面的内容会原样传给 `rosbag play`：

```bash
rosrun onion_gps_evaluation play_bag_and_finalize.sh \
  --bag /data/continuous_run.bag \
  --workflow segmented \
  --drain-sec 5 \
  -- --clock -r 0.5
```

参数说明：

```text
--bag                   必填，rosbag 路径
--workflow              必填，registration/evaluation/segmented
--drain-sec             可选，播放结束后的回调等待时间，默认 2 秒
--service-wait-sec      可选，启动前等待 finalize 服务的时间，默认 30 秒
--                      后续参数传给 rosbag play
```

脚本把 `rosbag play` 的标准输入重定向到 `/dev/null`，防止无人值守播放时空格键误暂停。
如果播放被中断、rosbag 返回非零状态、服务不存在或服务返回 `success=false`，脚本都会
返回失败，并且不会把异常播放主动标记为成功报告。

2 秒仅适用于 Onion 位姿没有明显处理积压的情况。如果 bag 结束后 Onion 仍在持续发布
积压位姿，应增大 `--drain-sec`，同时检查处理耗时与播放速率；等待时间不能修复持续
处理速度低于输入速度的问题。
