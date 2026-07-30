# Onion-LO++ 建图、全局重定位与连续定位开发交接

- 更新日期：2026-07-28
- 代码基线：`d2ca54acd7ac9b3ca14139e9a585305263cd0ddb`
- Git 分支：`main`
- 远端仓库：`https://github.com/2573461877/onion-lo-plus-ultra.git`

## 1. 阅读顺序和重要边界

接手此项目的智能体应先完整阅读本文件，然后依次阅读：

1. 仓库根目录 `README.md`。
2. `onion_relocalization/README.md`。
3. `onion-lo++/config/halo_outdoor.yaml`。
4. `onion_relocalization/config/halo_outdoor.yaml`。
5. 本地测试报告
   `linux-diagnostics/onion-export-relocal-vehicle-v7-ratio-final-20260727/VEHICLE_RELOCALIZATION_EVALUATION.md`。

必须正确理解以下结论：

- 当前代码已经具备 MID-360/Halo 建图、PCD 保存与载入、HDL 全局重定位、
  Scan Context + KISS-Matcher 全局重定位和 Onion 连续 scan-to-map 定位。
- 当前 Scan Context 三点测试的**初始重定位重复性误差**为
  `0.003–0.014 m`，但这不是独立真值精度。
- Onion 接管后的尾段 3D P95 为 `0.078–0.117 m`，因此当前整套系统
  **尚未达到持续 5 cm 定位要求**。
- 同一 bag 的 Onion 建图轨迹只能用于重复性回归，不能称为车辆真实位姿。
  正式 5 cm 验收必须使用 RTK/INS、测量控制点、全站仪或其他独立真值。
- 生产中的 Scan Context 1.5 s 移动点云必须由轮速+IMU、INS 或独立局部
  LiDAR 里程计提供相对运动。不能让等待 `/initialpose` 的 Onion 同时充当
  初始化前去畸变里程计，否则会形成循环依赖。

## 2. 当前状态快照

| 项目 | 当前状态 |
|---|---|
| Windows 本地代码 | `E:\Onion-LO-Plus` |
| 本地分支 | `main` |
| 本地/GitHub/设备提交 | 均为 `d2ca54a`，本文件创建前已核对一致 |
| Linux 主机 | `EAORA04`，ARM64，Ubuntu 22.04 |
| SSH | `nvidia@192.168.2.44:22`，密码只能交互输入，禁止写进命令或文档 |
| ROS 环境 | Docker `sydl-noetic-final`（镜像 `sydl/noetic-dev:eaora04`）内的 ROS1 Noetic |
| Docker 状态 | 2026-07-28 核对为运行中 |
| Catkin 工作空间 | `/home/nvidia/test-onion-ws` |
| Linux 仓库 | `/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra` |
| Git 状态 | 2026-07-28 核对为 `main...origin/main` 且干净 |
| Release 编译 | 已通过 |
| HDL/Scan Context launch | `roslaunch --nodes` 均解析通过 |
| 地图/数据库 | 存在于设备 `onion-lo++/results`，不受 Git 管理 |
| 当前推荐全局初始化 | Scan Context + KISS-Matcher；HDL 保留为可切换备选 |
| 生产 5 cm 验收 | 未完成 |

本文件是新生成的本地交接文档。如果它尚未提交，`git status` 会显示该文件为
未跟踪文件；这不表示算法源码产生了漂移。

## 3. 存储位置

### 3.1 Windows 本地

| 内容 | 路径 |
|---|---|
| 源码仓库 | `E:\Onion-LO-Plus` |
| MID-360/Halo 主包 | `E:\Onion-LO-Plus\onion-lo++` |
| 统一点云预处理包 | `E:\Onion-LO-Plus\onion_cloud_preprocessing` |
| HDL/Scan Context 统一重定位包 | `E:\Onion-LO-Plus\onion_relocalization` |
| 内置 HDL 引擎 | `E:\Onion-LO-Plus\hdl_global_localization` |
| 本地忽略的 Livox 驱动源码 | `E:\Onion-LO-Plus\livox_ros_driver2` |
| 当前重定位评估 | `E:\Onion-LO-Plus\linux-diagnostics\onion-export-relocal-vehicle-v7-ratio-final-20260727` |
| 当前车体裁剪地图及预览 | `E:\Onion-LO-Plus\linux-diagnostics\onion-export-vehicle-crop-20260727` |
| Halo 产品手册 | `C:\Users\SYDL\Downloads\Halo 产品手册 V0.3.pdf` |

本地已清除旧 `linux-maps`、早期诊断和临时目录。当前有效本地地图副本为：

```text
linux-diagnostics/onion-export-vehicle-crop-20260727/
  tanwei_tw360_halo_outdoor_vehicle_crop_20260727.pcd
  tanwei_tw360_halo_outdoor_vehicle_crop_20260727_binary.pcd
  map_topdown_density_and_trajectory.png
  map_topdown_height.png
  vehicle_crop_mapping_20260727.csv
  vehicle_crop_mapping_20260727.log
```

其中压缩 PCD：

```text
POINTS 568023
FIELDS x y z intensity label
DATA binary_compressed
SHA-256 b25101ec1836fca3da6591d9012ccb65ab978b16e1107cdcc9f5d6944c2d8b9b
```

### 3.2 Linux 调试端

| 内容 | 路径 |
|---|---|
| ROS1 工作空间 | 主机 `/home/nvidia/test-onion-ws`；新容器 `/workspaces/test-onion-ws` |
| 项目仓库 | `/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra` |
| Livox 驱动 Catkin 包 | `/home/nvidia/test-onion-ws/src/livox_ros_driver2` |
| 结果目录 | `/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results` |
| MID-360 bag | `/home/nvidia/mid360-20260723.bag` |
| 当前可用 Halo bag | `/home/nvidia/ssd-data/bags/tanwei_tw360.bag` |
| 当前车体裁剪地图 | `.../onion-lo++/results/tanwei_tw360_halo_outdoor_vehicle_crop_20260727.pcd` |
| 旧的未统一裁剪 Halo 地图 | `.../onion-lo++/results/tanwei_tw360_halo_outdoor_map.pcd` |
| HDL 粗地图 | `.../onion-lo++/results/tanwei_tw360_halo_outdoor_map_voxel_2m.pcd` |
| MID-360 地图 | `.../onion-lo++/results/onion_map.pcd` |
| Scan Context 数据库 | `.../onion-lo++/results/scancontext_database_vehicle_v3` |
| 当前建图轨迹/指标 | `.../onion-lo++/results/diagnostics/halo_outdoor/vehicle_crop_mapping_20260727.csv` |
| 三点重定位指标 | `.../onion-lo++/results/diagnostics/halo_relocal_vehicle_v7_ratio_final` |

`...` 代表：

```text
/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra
```

2026-07-28 实机核对结果：

```text
Halo bag:
  path: /home/nvidia/ssd-data/bags/tanwei_tw360.bag
  duration: 76 s
  size: 1.7 GB
  /fusion_rslidar_points: 766 sensor_msgs/PointCloud2
  /tanwaylidar_imu: 15334 sensor_msgs/Imu

vehicle crop map:
  size: 9841268 bytes
  SHA-256:
  b25101ec1836fca3da6591d9012ccb65ab978b16e1107cdcc9f5d6944c2d8b9b

old Halo map:
  size: 10440632 bytes
  SHA-256:
  488ca41c4e17e8d930513adb9569321dfc6d300f913f69ccb6a526894a66ed68

MID-360 map:
  size: 1853282 bytes
  SHA-256:
  45d66f86494ddec389d72b1b46b932d5009a0be05eb83a8cc6d93cb20aecd251

Scan Context vehicle_v3:
  38 keyframes
  38 persisted descriptors
  manifest SHA-256:
  4253077727c81869c1cc0cd1a0020cddebed5bc53146311842a56cf0d4335b14
```

### 3.3 当前已知路径漂移

以下 launch 的默认 Halo bag 仍写成：

```text
/home/nvidia/tanwei_tw360.bag
```

但设备上的实际文件是：

```text
/home/nvidia/ssd-data/bags/tanwei_tw360.bag
```

因此运行以下 launch 时必须显式传入正确的 `bagfile` 或 `bag_path`：

- `onion-lo++/launch/halo_outdoor.launch`
- `onion_relocalization/launch/halo_scancontext_evaluation.launch`
- `onion_relocalization/launch/halo_build_scancontext_database.launch`

在修正默认值前，不要直接依赖 launch 默认 bag 路径。

## 4. 当前功能包与职责

### 4.1 `onion-lo++`

主要功能：

- 读取 `sensor_msgs/PointCloud2`。
- 按逐点时间进行连续时间 LiDAR 里程计。
- Onion 特征分类、滑窗优化、局部体素地图配准。
- 建立并保存持久化全局 PCD。
- 载入已有 PCD 后做 local scan-to-map 定位。
- 接收 `/initialpose`，重置局部定位并回放初始化期间缓存的点云。
- 发布位姿、里程计、轨迹、局部地图、全局地图和 TF。
- 在 tracking failure 时保存上下文并保护最后一份有效地图。

核心文件：

```text
onion-lo++/src/ros_main.cpp
onion-lo++/src/ros_main.hpp
onion-lo++/src/traj_odom.cpp
onion-lo++/src/map_manager.cpp
onion-lo++/src/Save_Map.hpp
onion-lo++/config/livox.yaml
onion-lo++/config/halo_outdoor.yaml
```

### 4.2 `onion_cloud_preprocessing`

主要功能：

- 只执行一次共享车辆自身点云裁剪。
- 在 `vehicle_link` 中判断车体包围盒。
- 同时输出保留原传感器坐标和转换到 `vehicle_link` 的点云。
- 保留 PointCloud2 的逐点时间等原始字段。
- TF 不可用时默认失败退出，防止静默跳过滤波污染地图。

数据流：

```text
/fusion_rslidar_points
  -> /onion/points_filtered_sensor  [rslidar_top，给 Onion]
  -> /onion/points_filtered_vehicle [vehicle_link，给全局重定位]
```

### 4.3 `onion_relocalization`

该包已经合并原来的：

```text
onion_global_relocalization
onion_scancontext_relocalization
```

当前包含三个可执行程序：

```text
onion_hdl_relocalization_node
onion_scancontext_relocalization_node
onion_scancontext_database_builder
```

通过以下 launch 切换算法：

```bash
roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=hdl

roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=scancontext
```

### 4.4 `hdl_global_localization`

- HDL 源码已作为普通 Catkin 包内置到本仓库。
- 已移除 Git 子模块、`.gitmodules`、编译时在线下载、TEASER 代码和无效配置。
- 当前可用引擎为 `BBS` 和 `FPFH_RANSAC`。
- Onion 适配默认使用 CPU `FPFH_RANSAC`。
- 必须保留 `hdl_global_localization/LICENSE`。

不要重新引入原 HDL 子模块，也不要因“只用于测试”而删除 BSD 或其他第三方
许可证。非商业用途同样需要遵守再分发条款。

## 5. 坐标系和 TF

车辆目标输出统一为：

```text
odom -> vehicle_link
```

内部跟踪关系：

```text
Onion 内部跟踪: odom -> rslidar_top
公开输出:       odom -> vehicle_link
```

静态 TF 位于：

```text
onion_cloud_preprocessing/launch/halo_vehicle_tf.launch
```

当前关系：

```text
vehicle_link -> gps_link:
  xyz = [0, 0, 0]
  yaw pitch roll = [0, 0, 0]

vehicle_link -> fusion_link:
  xyz = [1.6, 0, 0]
  yaw pitch roll = [0, 0, 0]

fusion_link -> rslidar_top:
  xyz = [-0.15, 0, 0.9]
  yaw pitch roll = [-2.3562, 0.2618, 3.1415926]

fusion_link -> camera_link:
  xyz = [-0.15, 0, 1]
  yaw pitch roll = [0, 0, 0]
```

注意：ROS1 `tf/static_transform_publisher` 此处参数顺序是：

```text
x y z yaw pitch roll parent child period_ms
```

不要误写成 roll/pitch/yaw，也不要把父子坐标系反转。

`/initialpose` 的语义必须是：

```text
odom -> vehicle_link
```

Onion 内部会通过静态 TF 转换成：

```text
odom -> rslidar_top
```

不要把 LiDAR 位姿直接伪装成 `vehicle_link` 位姿发布到 `/initialpose`。

## 6. 车辆自身点云裁剪

当前 Halo 车辆裁剪范围在 `vehicle_link` 中定义：

```yaml
min_x: -0.6
max_x: 1.65
min_y: -0.7
max_y: 0.7
min_z: -0.5
max_z: 2.5
```

建图、HDL、Scan Context 和 Onion 定位都使用同一个
`onion_vehicle_crop_filter`。不要再在后端额外开启第二次裁剪，否则可能：

- 让建图点集与定位点集不一致。
- 重复 TF 和序列化开销。
- 破坏逐点时间字段。
- 使调参时无法判断是哪一级滤波产生差异。

`halo_outdoor.launch` 已把 Onion 内部备用裁剪关闭。Onion 自带裁剪参数只用于
绕过共享预处理器直接启动节点时的后备方案。

当前仅实现车辆自身包围盒裁剪；没有实现经过验证的动态车辆/行人时序剔除。
Onion 的标签、体素下采样、鲁棒核或点到平面残差都不能等价为动态目标滤波。

## 7. 点云时间字段与扫描窗口

### 7.1 Tanway Halo bag

当前 bag 的 PointCloud2 逐点绝对时间为：

```text
t_sec + t_usec * 1e-6
```

配置：

```yaml
point_time_field: t_sec
point_time_scale: 1.0
point_time_secondary_field: t_usec
point_time_secondary_scale: 1.0e-6
point_time_is_offset: false
```

Halo 是约 10 Hz 的旋转式 360° LiDAR，应使用：

```yaml
Traj/seg_interval: 100
```

旧的 20 ms 参数只覆盖约 72° 扇区，室外容易欠约束并出现少于 20 个配准内点。

### 7.2 Livox MID-360

Livox Driver2 `PointCloud2` 使用：

```text
x, y, z, intensity, tag, line, timestamp
```

时间配置：

```yaml
point_time_field: timestamp
point_time_scale: 1.0e-9
point_time_is_offset: false
```

不要只凭话题名判断消息类型。必须先执行：

```bash
rostopic type /livox/lidar
rostopic info /livox/lidar
rostopic echo -n 1 /livox/lidar/fields
```

确认实际是 `sensor_msgs/PointCloud2` 后再使用当前配置；不要误改成
`livox_ros_driver2/CustomMsg`。

## 8. 地图、体素和载入逻辑

### 8.1 持久 PCD

当前持久地图保存：

```text
x y z intensity label
```

Halo 默认持久地图体素为 `0.20 m`。每个“空间体素 + Onion label”保存所有
落入点的 XYZ 和 intensity 均值，即质心代表点，而不是直接保留第一个点。

体素索引使用：

```cpp
floor(x / voxel_size)
floor(y / voxel_size)
floor(z / voxel_size)
```

因此负坐标不会再使用向零截断。

### 8.2 载入地图

载入地图后，Onion 建立用于局部配准的体素表。每个体素超过
`loaded_map_max_points_per_voxel`（默认 150）时：

1. 先稳定排序，避免依赖 PCD 文件点顺序。
2. 每种 label 至少保留一个靠近体素中心的代表点。
3. 其余容量使用最远点采样填充，以覆盖体素内的空间几何。

这已经修复“每体素直接截取前 150 点”的问题。

普通 XYZ/XYZI PCD 仍可载入，但没有 `label` 时会统一按 label 0 处理，失去
Onion 标签一致性约束。给全局重定位模块使用时 XYZ 足够做纯几何配准，但
Onion 连续定位最好使用本项目输出的 XYZIL 地图。

`Save_Map.hpp` 中旧的 `Save_VoxelHashMap` 使用 `round()`，它服务于传统
OctoMap/可视化缓存，不是当前持久 PCD 的 `GlobalMapVoxelCache`。不要再次
把旧辅助类当成 PCD 保存实现。

### 8.3 地图保护

当前代码会拒绝用以下地图覆盖最后一份有效 PCD：

- 点数少于阈值。
- 明显退化成直线。
- tracking failure 后被污染。
- 包含无效字段或无法重新读回验证。

调试失败时不要手工覆盖已验证地图。新地图应使用新文件名，完成点数、范围、
PCD 字段、哈希和可视化检查后再替换正式路径。

## 9. 定位流程

### 9.1 Scan Context 路径

数据库构建：

```text
原始 Halo bag
  -> TF 到 vehicle_link
  -> 车辆自身点云裁剪
  -> 使用建图轨迹对逐点时间去畸变
  -> 每 2 s 建立关键帧，前后各累积约 1 s
  -> 保存 fine/coarse PCD
  -> 保存 Polar Scan Context 描述子
  -> 保存关键帧时间、vehicle_link 位姿、帧号和文件路径到 manifest.csv
```

当前 `vehicle_v3` 数据库：

- 38 个关键帧。
- 38 个描述子。
- 765 帧接受，1 帧在去畸变质量检查中拒绝。
- 数据库 anchor frame 为 `vehicle_link`。
- 车体裁剪删除约 46.64% 的有限原始点。

运行时：

```text
实时点云
  -> vehicle_link 裁剪
  -> 外部相对里程计逐点去畸变
  -> 累积 1.5 s，至少 8 帧
  -> Polar Scan Context Top-3 检索和粗 yaw
  -> KISS-Matcher/Quatro 6DoF 粗配准
  -> GICP 精配准
  -> inlier/RMSE/几何一致性验证
  -> 发布 odom -> vehicle_link 到 /initialpose
  -> Onion 转换到 rslidar_top 并回放缓存点云
  -> 连续 scan-to-map 定位
```

主要默认阈值：

```yaml
accumulation_sec: 1.5
minimum_accumulated_frames: 8
top_k: 3
kiss_voxel_size: 0.60
kiss_max_correspondences: 3000
kiss_use_quatro: true
kiss_use_ratio_test: true
gicp_max_correspondence_distance: 0.80
minimum_kiss_inliers: 100
validation_distance_m: 0.30
minimum_inlier_fraction: 0.45
maximum_rmse_m: 0.18
```

保持 KISS-Matcher ratio test 开启。之前随机截断大量对应点会造成厘米级结果
抖动；ratio test 会稳定保留更优特征比值。

### 9.2 HDL 路径

```text
载入全局 PCD
  -> 1.0 m 下采样
  -> FPFH 特征
  -> RANSAC Top-K 全局候选
  -> 候选附近裁剪密集子地图
  -> NDT + ICP 精配准
  -> inlier/RMSE/方位覆盖/修正幅度验证
  -> 发布 odom -> vehicle_link 到 /initialpose
  -> Onion 连续定位
```

HDL 默认的 `FPFH_RANSAC` 在 ARM64 上比上游密集默认参数更保守。它不需要
Scan Context 描述子数据库，但全局 FPFH 初始化对大地图更耗时，对重复室外
结构也更容易出现 Top-K 召回不足。密集 NDT/ICP 只能修正候选，无法找回没有
进入 Top-K 的真实位置。

当前保留证据确认 HDL 节点可编译、launch 可解析、接口可接入 Onion；当前
保留的三点厘米级报告针对 Scan Context 路径，不要把该结果写成 HDL 三点结果。

### 9.3 `/initialpose` 缓存回放

全局匹配耗时期间，Onion 会缓存输入点云：

```yaml
initial_pose_cloud_buffer_size: 50
initial_pose_replay_tolerance_sec: 0.05
```

收到 `/initialpose` 后，从该时间戳附近开始回放缓存，再追到当前帧。

这不是可选优化。旧实现会在全局配准期间丢弃点云，使初始位姿对应的扫描和
Onion 接管扫描相差约 0.5–0.6 s。在 35 s 测点中：

```text
修复前：
  初值误差约 0.009 m
  Onion 尾段 3D P95 约 10.825 m

修复后：
  初值误差约 0.014 m
  Onion 尾段 3D P95 约 0.117 m
```

不要删除时间戳回放，也不要在初始化完成前直接把缓存清空。

## 10. 编译与启动

### 10.1 连接设备

Windows PowerShell：

```powershell
Test-NetConnection 192.168.2.44 -Port 22 -InformationLevel Quiet
ssh -tt -o StrictHostKeyChecking=accept-new nvidia@192.168.2.44
```

密码只在交互提示中输入。禁止使用 `sshpass`、命令行参数、脚本或文档明文。

进入 ROS1 容器：

```bash
sudo docker inspect -f '{{.State.Running}}' sydl-noetic-final
sudo docker exec -it sydl-noetic-final /bin/bash
```

Linux 宿主机有 ROS2 Humble；本项目必须在 `sydl-noetic-final` 的 ROS1 Noetic 中
编译和运行。

### 10.2 Release 编译

容器内：

```bash
source /opt/ros/noetic/setup.bash
cd /home/nvidia/test-onion-ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source /home/nvidia/test-onion-ws/devel/setup.bash
```

必须要求退出码为 0，并确认以下目标存在：

```text
onion_lo_plus_node
vehicle_crop_filter_node
hdl_global_localization_node
onion_hdl_relocalization_node
onion_scancontext_relocalization_node
onion_scancontext_database_builder
```

缺少可选 OpenNI/libusb/VTK 工具和 TBB deprecated 信息是现有容器警告。
只有在最终构建退出码为 0、目标全部生成时才可忽略。

### 10.3 检查 Halo bag

```bash
source /opt/ros/noetic/setup.bash
rosbag info /home/nvidia/ssd-data/bags/tanwei_tw360.bag

source /home/nvidia/test-onion-ws/devel/setup.bash
rosrun onion_lo_plus check_pointcloud_bag.py \
  /home/nvidia/ssd-data/bags/tanwei_tw360.bag \
  --topic /fusion_rslidar_points
```

当前可用修复包应为 766 条 PointCloud2。不要重新使用已删除的原始损坏包。

### 10.4 Halo 离线建图

正确性优先：

```bash
roslaunch onion_lo_plus halo_outdoor.launch \
  bagfile:=/home/nvidia/ssd-data/bags/tanwei_tw360.bag \
  bag_rate:=0.25 \
  visualize:=false \
  map_path:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/halo_new_map.pcd \
  metrics_output_path:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/diagnostics/halo_outdoor/halo_new_metrics.csv
```

此前完整 766 帧在 `bag_rate:=0.5` 也完成过，但新改动第一次回归应从 0.25
开始。确认单帧处理持续低于有效输入周期、无积压后再提高。

手动保存：

```bash
rosservice call /onion_lo_plus_node/save_map "{}"
```

不要直接覆盖已验证的
`tanwei_tw360_halo_outdoor_vehicle_crop_20260727.pcd`。

### 10.5 构建 Scan Context 数据库

```bash
roslaunch onion_relocalization halo_build_scancontext_database.launch \
  bag_path:=/home/nvidia/ssd-data/bags/tanwei_tw360.bag \
  reference_poses_csv:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/diagnostics/halo_outdoor/vehicle_crop_mapping_20260727.csv \
  output_directory:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/scancontext_database_vehicle_v3
```

数据库必须和以下内容成套更新：

- 同一版本的裁剪地图。
- 同一次建图轨迹。
- 相同 TF。
- 相同车辆裁剪范围。
- 相同逐点时间解释。

不要只从一张全局 PCD 人为切块后声称得到准确关键帧数据库。

### 10.6 Scan Context 离线回归

测试专用、使用同 bag 建图轨迹提供相对运动：

```bash
roslaunch onion_relocalization halo_scancontext_evaluation.launch \
  bagfile:=/home/nvidia/ssd-data/bags/tanwei_tw360.bag \
  bag_start:=10.0 \
  bag_rate:=0.5 \
  map_path:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/tanwei_tw360_halo_outdoor_vehicle_crop_20260727.pcd \
  database_manifest:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/scancontext_database_vehicle_v3/manifest.csv
```

依次使用 `bag_start:=10.0`、`35.0`、`60.0` 做差异位置回归。必须给每次运行
设置不同的指标输出文件，避免追加到旧 CSV 后误统计。

`halo_scancontext_evaluation.launch` 中的参考里程计只用于测试。生产时禁止使用
录制后生成的 Onion 建图轨迹。

### 10.7 Scan Context 实时运行

先确保实际雷达驱动发布：

```text
/fusion_rslidar_points
sensor_msgs/PointCloud2
frame_id = rslidar_top
```

并确保外部相对里程计已经发布，例如 `/local_odometry`：

```bash
roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=scancontext \
  map_path:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/tanwei_tw360_halo_outdoor_vehicle_crop_20260727.pcd \
  database_manifest:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/scancontext_database_vehicle_v3/manifest.csv \
  relative_odom_topic:=/local_odometry \
  visualize:=false
```

如果 `relative_odom_topic` 为空，launch 会进入“静止测试”模式并关闭强制去畸变。
这只适用于车辆静止的初始测试，不能用于 5 m/s 移动车辆。

### 10.8 HDL 回归

```bash
roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=hdl \
  map_path:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/tanwei_tw360_halo_outdoor_vehicle_crop_20260727.pcd \
  bagfile:=/home/nvidia/ssd-data/bags/tanwei_tw360.bag \
  bag_start:=10.0 \
  bag_rate:=0.25 \
  visualize:=false
```

需要重新触发时：

```bash
rosservice call /onion_hdl_relocalization/relocalize "{}"
```

Scan Context：

```bash
rosservice call /onion_scancontext_relocalization/relocalize "{}"
```

## 11. 关键话题、服务和输出

### 输入

```text
/fusion_rslidar_points                  Halo 原始 PointCloud2
/livox/lidar                            MID-360 PointCloud2
/local_odometry                         生产 Scan Context 外部相对运动示例
/initialpose                            全局初始化或人工初始化输入
```

### 预处理

```text
/onion/points_filtered_sensor           rslidar_top
/onion/points_filtered_vehicle          vehicle_link
```

### Onion 输出

```text
/onion_lo_plus_node/odometry            nav_msgs/Odometry
/onion_lo_plus_node/pose                geometry_msgs/PoseStamped
/onion_lo_plus_node/trajectory          nav_msgs/Path
/onion_lo_plus_node/frame               当前去畸变点云
/onion_lo_plus_node/local_map           局部配准地图
/onion_lo_plus_node/global_map          持久地图预览
/onion_lo_plus_node/octomap_binary      旧 OctoMap 输出
TF: odom -> vehicle_link
```

### 重定位

```text
/initialpose
/onion_hdl_relocalization/candidate_pose
/onion_scancontext_relocalization/status
/onion_hdl_relocalization/relocalize
/onion_scancontext_relocalization/relocalize
```

### 地图

```text
/onion_lo_plus_node/save_map
```

## 12. 当前测试结果

### 12.1 Scan Context 三点初始重定位

| bag 起点 | Top-1 | 初始 3D 差异 | 初始姿态差异 | 配准计算耗时 |
|---:|:---:|---:|---:|---:|
| 10 s | 是 | 0.003 m | 0.002° | 859.6 ms |
| 35 s | 是 | 0.014 m | 0.177° | 1082.5 ms |
| 60 s | 是 | 0.007 m | 0.009° | 1002.0 ms |

计算耗时不包含 1.5 s 点云累积。按实时点云估计，空启动到首次初始位姿约
`2.36–2.58 s`。

### 12.2 Onion 连续定位

| bag 起点 | Onion 单帧 P95 | 尾段 3D P95 | 尾段 5 cm 通过率 |
|---:|---:|---:|---:|
| 10 s | 106.9 ms | 0.078 m | 3.8% |
| 35 s | 120.0 ms | 0.117 m | 46.7% |
| 60 s | 113.8 ms | 0.103 m | 1.9% |

最初约 1 s 有 `0.25–0.49 m` 初始化瞬态。控制器不能在第一条 Onion 位姿到达
时立即闭环，应等待：

- `/initialpose` 已接受。
- 缓存回放追平。
- 连续残差和处理时延稳定。
- 独立定位质量检查通过。

### 12.3 资源占用

35 s 组合运行：

| 组件 | CPU 均值 | CPU 峰值 | RSS 峰值 | 温度峰值 |
|---|---:|---:|---:|---:|
| Scan Context | 7.6% | 159.9% | 94.4 MB | 48.5°C |
| Onion | 213.5% | 349.3% | 158.5 MB | 48.5°C |
| 预处理 | 4.0% | 25.7% | 36.1 MB | 48.6°C |

Onion 在 10 Hz 输入下已接近或略超 100 ms 周期。5 m/s 车辆部署前仍需降低
P95 时延、避免队列增长，并增加控制器就绪门控。

## 13. 历史错误、根因和防回归规则

| 现象/错误 | 根因 | 已采用修正 | 后续禁止事项 |
|---|---|---|---|
| MID-360 话题名正确但节点无点云 | 把 PointCloud2 当成 CustomMsg，或字段配置不匹配 | 按实际 PointCloud2 字段解析，时间用 `timestamp * 1e-9` | 不检查 `rostopic type/fields` 就改消息类型 |
| Halo 室外配准内点少、地图像扇形 | 沿用 MID-360 的 20 ms 段，只看到旋转雷达局部扇区 | Halo `Traj/seg_interval=100 ms` | 把固态雷达扫描窗直接套到旋转雷达 |
| PointCloud2 去畸变异常 | 把 `t_sec/t_usec` 当相对时间，或单位错误 | 使用绝对 `t_sec + t_usec*1e-6` | 仅按产品手册新版字段猜测录制 bag 的实际字段 |
| rosbag reindex 后仍出现异常点 | reindex 只能修索引，不能修 PointCloud2 payload | 确认第 767 帧从点 23788 起损坏，只保留前 766 帧 | 把“rosbag info 可读”当成 payload 完整证明 |
| 建图 1.0 倍速时队列积压 | 处理 130–156 ms，超过 10 Hz 输入周期 | 正确性优先降低 bag rate，增大输入缓冲仅吸收短暂抖动 | 用增大输出队列掩盖输入计算不足 |
| 失败运行覆盖好地图 | tracking failure 后仍保存持久地图 | tracking failure 锁存、最小点数和地图形状验证 | 失败后直接覆盖正式 PCD |
| 负坐标进入错误体素 | 使用 C++ 向零截断 | 当前体素键统一使用 `floor()` | 恢复 `static_cast<int>(x/voxel)` |
| 载入地图每体素只保留文件前 150 点 | 点顺序偏置，空间覆盖差 | label 代表点 + 稳定最远点采样 | 恢复直接 `resize(150)` |
| 建图有车体反射，定位点集却不同 | 建图/定位/重定位裁剪不统一 | 单一共享预处理包，同时输出传感器帧和车辆帧 | 每个功能包各自再裁剪一次 |
| TF 缺失时地图被车辆点污染 | 裁剪节点静默降级为不过滤 | `fail_if_tf_unavailable=true` | 为了“先运行”而静默跳过 TF |
| 重定位输出在 LiDAR 原点而车辆需要后轴中心 | `/initialpose` 和 Onion 输出 frame 语义不统一 | 公开统一 `odom -> vehicle_link`，内部转换到 `rslidar_top` | 把 sensor pose 直接标成 vehicle pose |
| 初值很好但 Onion 随后漂移约 10 m | 全局配准耗时期间点云丢失，初值与接管扫描时间错位 | 按 `/initialpose` 时间戳回放 6–7 帧缓存 | 删除缓存回放或用最新扫描直接套旧初值 |
| 移动车辆 Scan Context 1.5 s 累积变形 | 没有独立相对运动，无法逐点/逐帧去畸变 | 接入外部轮速+IMU/INS/local LIO | 用等待初值的 Onion 反过来给自身初始化去畸变 |
| Scan Context 同一位置结果有厘米级随机波动 | KISS 对应点随机截断 | 开启 ratio test，稳定选择优质对应 | 关闭 ratio test 后只比较单次结果 |
| HDL 大地图初始化很慢 | 上游 0.5 m 密集 FPFH 参数不适合 ARM64 | 默认 1.0 m 全局下采样并限制 RANSAC budget | 不做资源监控就恢复上游密集参数 |
| HDL 改为内置后“本地应失败、设备却编过” | 设备旧子模块目录残留 TEASER 头文件，形成假通过 | 清空精确 HDL 目录后复制干净源码并重编；最终 Git 对齐 | 在脏子模块目录上验证移除文件的改动 |
| Git reset 后旧验证文件仍出现 | `reset --hard` 不删除未跟踪文件 | 只删除已确认的精确旧文件，再核对 `git status` | 使用宽泛 `git clean -fdx` |
| Git 报 dubious ownership | Linux 宿主/容器所有权不同 | 只添加精确 safe.directory | 使用 `safe.directory=*` |
| Windows README 显示修改但内容未变 | CRLF/索引状态变化 | 比较 blob/hash 和实际 diff | 看到 `M` 就盲目覆盖用户文件 |
| 编译输出有 VTK/OpenNI/TBB 警告 | 容器缺少未使用的可选工具或上游弃用提示 | 以退出码和目标生成结果判断 | 把 warning 当成功，也不要把非零退出当 warning |
| 三点误差小就宣称道路绝对 5 cm | 参考轨迹与被测系统同源 | 明确称为重复性回归，引入 RTK/INS 独立真值 | 用同 bag Onion 轨迹做正式精度验收 |
| launch 找不到 Halo bag | 默认仍指向 `/home/nvidia/tanwei_tw360.bag` | 当前命令显式传 SSD 路径 | 在未修默认值前省略 `bagfile/bag_path` |

## 14. 当前限制与下一阶段建议

优先级从高到低：

1. **建立独立真值**
   同步录制 `/fusion_rslidar_points`、IMU、RTK/INS 或 `/gps_odom`。
   `gps_link` 当前与 `vehicle_link` 重合，但仍需确认 GPS 数据的坐标定义、
   ENU/UTM 转换、时间同步、固定解状态和杆臂标定。不要直接把未经检查的
   GPS odometry 当厘米级真值。

2. **接入初始化前局部运动源**
   生产 Scan Context 需要轮速+IMU、INS 或独立 local LIO，为 1.5 s 查询
   点云提供逐点去畸变和帧间累积。

3. **降低 Onion 连续跟踪误差**
   当前全局初值已明显优于连续尾段。下一步应针对地图质量、时延、动态物体、
   地面约束、标定和 scan-to-map 参数进行独立 A/B，而不是继续只优化初值。

4. **使 10 Hz 实时周期稳定**
   Onion 单帧 P95 为 107–120 ms。需要让稳定工况 P95 明显低于 100 ms，
   同时监控 ROS 队列、CPU、温度和首次定位总延迟。

5. **动态物体地图清理**
   当前只有车体裁剪，没有经过验证的跨帧动态物体剔除。建议先保留地面用于
   里程计约束，针对持久地图增加跨帧稳定性过滤，而不是把所有非平面点都视为
   动态点。

6. **控制器侧定位就绪状态**
   需要发布明确的状态：全局初始化接受、缓存追平、连续残差稳定、时延正常、
   独立传感器健康。车辆控制不能只监听第一条 odometry。

7. **修复 launch 默认 Halo bag 路径**
   若设备 SSD 路径确定长期不变，可将默认值改为
   `/home/nvidia/ssd-data/bags/tanwei_tw360.bag`，修改后仍需在设备编译和
   `roslaunch --nodes` 验证。

## 15. Git、同步和发布规则

Windows 仓库是源码真值：

```powershell
Set-Location 'E:\Onion-LO-Plus'
git status --short --branch
git diff --check
```

修改只在本地完成，然后传精确文件到 Linux 临时目录。不要直接 SCP 到
root-owned 仓库。每个文件使用 SHA-256 对比。

不要提交：

- bag。
- PCD/地图。
- `linux-diagnostics`。
- `build/devel/install`。
- 本地 Livox 驱动源码。
- SSH 传输暂存目录。
- 密码或设备凭据。

发布流程：

```text
Windows 修改
  -> Linux sydl-noetic-final 编译/最小运行验证
  -> Windows 精确暂存
  -> commit 到 main
  -> push origin main
  -> Linux git fetch
  -> Linux 精确检查后 reset --hard origin/main
  -> Linux 再编译和最小验证
```

只有用户明确授权时才能推送或覆盖 Linux 工作树。禁止宽泛清理目录、工作空间
根目录或 `/home/nvidia`。

## 16. 新智能体首次接手检查清单

```text
[ ] 阅读本文件、根 README、onion_relocalization README
[ ] Windows git status，确认当前提交和用户未提交改动
[ ] Test-NetConnection 192.168.2.44:22
[ ] 交互 SSH，确认 EAORA04
[ ] 确认 sydl-noetic-final 正在运行
[ ] 确认 Linux 仓库路径和 git status
[ ] 确认实际 Halo bag 位于 SSD 路径
[ ] rosbag info 确认 766 条 PointCloud2
[ ] rostopic type/fields 确认消息和逐点时间字段
[ ] TF 检查 vehicle_link、fusion_link、rslidar_top
[ ] Release 编译退出码 0
[ ] roslaunch --nodes 分别检查 hdl 和 scancontext
[ ] 新实验使用新地图/CSV/日志文件名
[ ] 记录 bag start/rate、提交、参数、地图/数据库哈希
[ ] 同时记录 CPU、RSS、温度、单帧处理 P95 和初次定位总耗时
[ ] 区分重复性参考与独立物理真值
[ ] 不删除第三方许可证
```

## 17. 当前证据入口

本地：

```text
linux-diagnostics/
  onion-export-vehicle-crop-20260727/
    map_topdown_density_and_trajectory.png
    map_topdown_height.png
    tanwei_tw360_halo_outdoor_vehicle_crop_20260727.pcd
    vehicle_crop_mapping_20260727.csv
    vehicle_crop_mapping_20260727.log

  onion-export-relocal-vehicle-v7-ratio-final-20260727/
    VEHICLE_RELOCALIZATION_EVALUATION.md
    vehicle_relocalization_summary.csv
    vehicle_relocalization_summary.json
    vehicle_relocalization_resources_summary.csv
    vehicle_relocalization_summary.png
    vehicle_tracking_error_timeseries.png
    vehicle_relocalization_resources.png
    start10/
    start35/
    start60/
    resources/
```

关键提交：

```text
d2ca54a Vendor HDL localization and clean workspace metadata
82d289e feat: unify vehicle-frame relocalization pipelines
fd2f345 Improve global relocalization validation and plane residuals
9f1a618 Support coarse global relocalization maps
17aeb4d Add Tanway Halo outdoor mapping support
eb10e69 Add HDL global relocalization adapter
cac9595 Improve loaded map voxel sampling
02c8c18 Benchmark and tune map localization
3769c0e Add PointCloud2 bag continuity checker
b231a42 Capture first odometry divergence diagnostics
74a8946 Optimize indoor mapping and validated PCD export
```

后续每次重要调试都应在新的诊断目录中记录：

- Git 提交。
- 完整 launch 命令。
- 地图和数据库 SHA-256。
- bag 路径、消息数、起点和播放速率。
- 参数快照。
- 成功和失败日志。
- 精度、时延、资源、温度。
- 参考轨迹的来源及其是否独立。

## 18. 自动驾驶 GPS 真值候选接口（2026-07-28）

### 18.1 新容器和代码位置

后续 Onion 与自动驾驶 GPS 的编译、联调改用：

```text
container: sydl-noetic-final
image: sydl/noetic-dev:eaora04
ROS: Noetic
Onion workspace: /workspaces/test-onion-ws
autonomy workspace: /workspaces/sydl-autonomy
GPS repository:
  /workspaces/sydl-autonomy/repos/sydl-autonomy-perception
GPS package:
  .../src/gps_driver/gongji
```

`ruimove` 仍存在，但不再是本项目后续编译和调试目标。新容器把主机
`/home/nvidia/test-onion-ws` 和 `/home/nvidia/sydl-autonomy` 分别挂载到上述
`/workspaces` 路径。

设备上的自动驾驶工作区当前只有感知仓和决策仓；控制台仓、知识库仓尚未拉取。Windows
总工作区仍是四仓源码和文档真值：

```text
E:\_AAAA\origin-projects\sydl-autonomy
```

### 18.2 原 GPS 链路和兼容边界

整车 `smartcar/launch/driverless.launch` include `gongji/launch/gongji.launch`，
串口原始包发布 `/gps`，UTM 位姿发布 `/gps_odom`。

`/gps_odom.pose.covariance[0..4]` 已被决策、路径录制和上位机分别当作 yaw、经度、
纬度、卫星数和旧有效性读取，并不是标准协方差。本次不改变这些 topic 和字段语义，
避免影响原自动驾驶代码。

### 18.3 新增评估输出

新增独立 `gongji_evaluation` 节点，订阅 `/gps` 并发布：

```text
/gps/evaluation/fix
/gps/evaluation/odom_utm
/gps/evaluation/velocity_enu
/gps/evaluation/imu
/gps/evaluation/time_reference
/gps/evaluation/position_valid
/gps/evaluation/heading_valid
/gps/evaluation/valid
/gps/evaluation/diagnostics
```

使用：

```bash
roslaunch gongji gongji_evaluation.launch
```

回放已含 `/gps` 的 rosbag 时：

```bash
roslaunch gongji gongji_evaluation.launch start_driver:=false
```

新 launch 才开启严格 CRC；原 `gongji.launch` 默认保持历史兼容。评估节点沿用实车验证
解析，把北向为 0、北偏西为正的设备航向转换为 ENU `yaw = pi/2 + heading`，并把位置、速度、姿态
精度写入标准协方差，同时提供 GPS 周/周内时的 UTC 时间参考。

完整参数、单位、话题和硬件核对清单见：

```text
E:\_AAAA\origin-projects\sydl-autonomy\
  repos\sydl-autonomy-perception\
  src\gps_driver\gongji\GPS_EVALUATION_CN.md
```

### 18.4 当前验证结果和限制

2026-07-28 在 `sydl-noetic-final` 中完成：

```text
Release 全量 catkin_make: 通过
gongji: [100%] Built target
gongji_evaluation: [100%] Built target
source 感知后决策仓 Release 编译: 通过
转换单测: 5/5 通过
catkin_test_results: 0 errors, 0 failures
launch start_driver=false/true: 均解析通过
模拟 /gps 冒烟: 发布 NavSatFix，evaluation/valid=false
```

设备未连接 GPS，本轮没有验证串口、真实 CRC 通过率、状态码、时间单位、IMU 比例、
天线杆臂、topic 频率或实际定位精度。

`config/evaluation.yaml` 的 `accepted_gnss_statuses` 故意留空；仓库中没有该接收机的
状态码说明，填入准确的 RTK fixed 状态码前 `/gps/evaluation/valid` 保持 false。
接收机上报精度也不能单独证明 5 cm 绝对误差。

### 18.5 新设备硬件测试顺序

1. 核对 Gongji 型号协议、CRC、大小端和全部比例。
2. 记录原 `/gps`，确认 `GNSS_Status`、`INS_Status` 在单点、浮点、固定解和失锁时的
   实际数值。
3. 核对 GPS 周内时单位、闰秒、系统时钟和 LiDAR 时间基准。
4. 静止测试 IMU 重力方向、速度零偏、UTM 抖动和航向。
5. 测量 `gps_link` 到 `vehicle_link`、LiDAR frame 的杆臂和旋转外参。
6. 位置指标只在 `/gps/evaluation/position_valid=true` 的连续区间评分；航向指标另要求
   `/gps/evaluation/heading_valid=true`。把 UTM 轨迹一次性刚体对齐到 Onion 地图系，
   禁止逐帧对齐。
7. 分别报告水平、垂直、航向、3D 的中位数、P95、最大值、连续超限时长和有效样本比例。

## 19. Onion GPS C++ 轨迹配准与独立评估旁路（2026-07-30）

新增独立 Catkin 包：

```text
E:\Onion-LO-Plus\onion_gps_evaluation
```

该包不修改 `onion-lo++`、`onion_relocalization` 或自动驾驶 GPS 节点，只订阅：

```text
/onion_lo_plus_node/odometry
/gps/evaluation/odom_utm
/gps/evaluation/fix
/gps/evaluation/position_valid
/gps/evaluation/heading_valid
/initialpose
/onion_scancontext_relocalization/status
```

Python 原型已重构为 C++17/roscpp/Eigen 架构，运行时不再依赖 Python 或 NumPy。三个
用途分别对应独立节点和 launch，不再使用 `mode` 字符串在一个节点中切换：

```text
轨迹配准:
  src/trajectory_registration_node.cpp
  launch/trajectory_registration.launch

已有固定配准关系的定位精度评估:
  src/localization_accuracy_evaluator_node.cpp
  launch/localization_accuracy_evaluation.launch

同一连续 bag 前段配准、后段评估:
  src/segmented_registration_evaluator_node.cpp
  launch/segmented_registration_evaluation.launch
```

共享实现位于：

```text
include/onion_gps_evaluation/trajectory_alignment.hpp
src/trajectory_alignment.cpp
include/onion_gps_evaluation/evaluation_runner.hpp
src/evaluation_runner.cpp
```

三种启动方式：

```bash
roslaunch onion_gps_evaluation trajectory_registration.launch
roslaunch onion_gps_evaluation localization_accuracy_evaluation.launch
roslaunch onion_gps_evaluation segmented_registration_evaluation.launch
```

三个 finalize 服务：

```bash
rosservice call /onion_gps_trajectory_registration/finalize "{}"
rosservice call /onion_localization_accuracy_evaluation/finalize "{}"
rosservice call /onion_segmented_registration_evaluation/finalize "{}"
```

配准输出改名为 `map_to_gps_alignment.json`，保存固定
`GPS_from_Onion_map` 的 yaw 和 xyz 平移、坐标系、GPS 来源、配准样本/内点、基线、
残差和时间范围。它不是轨迹文件。同一次连续 bag 的分段模式默认前 30% 配准、后 70%
评估，两个样本集合严格不重叠；它适合联调和回归，但不能替代固定配准文件下的独立
采集路线验收。

主要功能保持：

1. 时间插值匹配 Onion 与车辆后轴中心 GPS 位姿。
2. RANSAC + SVD 估计地图到 UTM/ENU 的固定平面刚体变换和高度偏移。
3. 输出水平、垂直、3D、航向、1 s RPE、匹配率和 5/10/20 cm 通过率。
4. 以 `/initialpose` 为事件计算初始误差、首次输出延迟、稳定收敛时间和收敛后误差。
5. 保存逐帧 CSV、事件 CSV、JSON/CSV 汇总并发布 RViz Path；不发布 TF、
   `/initialpose` 或控制指令。

C++ 无硬件端到端回归：

```bash
roslaunch onion_gps_evaluation synthetic_segmented_evaluation.launch
```

详细文件职责、参数和流程以 `onion_gps_evaluation/README.md` 为准。模拟测试只验证
时间匹配、切分、配准、指标和报告链路，不代表实车定位精度。

### 19.1 C++ 重构验证结果

2026-07-30 已在 `sydl-noetic-final`、ROS Noetic、
`/home/nvidia/test-onion-ws` 完成：

```text
全工作空间 Release catkin_make: 通过
onion_gps_evaluation 四个 C++ 节点和公共库: 通过
trajectory_alignment gtest: 4/4 通过
catkin_test_results: 0 errors, 0 failures
四个 launch roslaunch --nodes: 全部解析通过
源目录残留 Python 运行文件: 0
```

分段合成回归：

```text
完整匹配: 241
前段配准: 73（内点 73，基线 6.032975 m）
后段评估: 168
水平 RMSE: 0.203441 m
水平 P95: 0.537386 m
航向 RMSE: 1.001397 deg
重定位稳定收敛: 1/1，1.35 s
accuracy_claim: post_registration_holdout_evaluation
```

模拟器故意在重定位事件后注入衰减位置和航向误差，所以上述误差不应为零。另已顺序
验证独立 `trajectory_registration.launch` 写入配准文件，再由
`localization_accuracy_evaluation.launch` 读取同一文件并完成 241 帧评分。

模拟回归生成的 CSV、JSON 和日志属于过程诊断产物，关键指标已记录在上文；这些产物
在提交前清理，不纳入 Git 仓库。

设备仍未连接 GPS 硬件，本轮没有验证真实 RTK 状态、时间同步、协方差、UTM 分区或
实车绝对精度。

### 19.2 rosbag 播放完成后自动 finalize

新增：

```text
onion_gps_evaluation/scripts/play_bag_and_finalize.sh
```

脚本在播放前确认目标服务存在，以标准输入断开的方式执行 `rosbag play`，只在播放
正常结束后等待回调队列（默认 2 秒）并调用服务。`--workflow` 与服务映射：

```text
registration -> /onion_gps_trajectory_registration/finalize
evaluation   -> /onion_localization_accuracy_evaluation/finalize
segmented    -> /onion_segmented_registration_evaluation/finalize
```

使用示例：

```bash
rosrun onion_gps_evaluation play_bag_and_finalize.sh \
  --bag /data/continuous_run.bag \
  --workflow segmented \
  --drain-sec 2 \
  -- --clock -r 0.5
```

对应评估 launch 必须先启动。播放被中断、返回非零状态、服务不存在或 finalize 返回
失败时，脚本返回非零状态，不主动把不完整数据标记为成功报告。完整参数和三种启动
组合见 `onion_gps_evaluation/README.md` 第 7 节。
