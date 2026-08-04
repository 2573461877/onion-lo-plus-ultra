# FAST-LIO Humanoid 建图与定位功能包

本目录是一个独立 ROS1 Catkin 功能包，将
[`deepglint/FAST_LIO_LOCALIZATION_HUMANOID`](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)
的 FAST-LIO 建图代码和 Open3D 固定地图定位代码整合到当前总项目中。
它不是 Git 子模块，也不包含上游 `.git`、示例地图、视频、图片、PDF、
运行日志或上游重复的 Livox 驱动。

## 功能边界

- `fast_lio_humanoid_mapping`：LiDAR/IMU 预处理、IMU 前向传播、
  迭代误差状态卡尔曼滤波、ikd-Tree 扫描到地图匹配、里程计/点云/轨迹
  发布和 PCD 保存。
- `fast_lio_humanoid_localization`：加载 PCD/PLY 固定地图，使用初始位姿、
  Open3D 粗细两级点云配准估计 `map -> camera_init`，发布定位位姿、
  置信度和延迟。
- 两个节点使用 `/fast_lio_humanoid/*` 输出，能够与 Onion-LO++ 分开启动，
  不覆盖其默认输出话题。

上游定位仍然需要大致正确的初始位姿；它不是不依赖先验的任意位置全局
重定位。运行中可通过 `/initialpose` 更新初始估计。

## Livox 驱动适配结论

当前总项目已有并已实机验证的 `livox_ros_driver2`，本功能包直接依赖它，
不会复制或修改该目录。

逐文件比较结果：

- 两份 Driver2 有 84 个普通文件 SHA-256 完全一致。
- DeepGlint 版本有 13 个不同文件和 1 个额外生成的 `package.xml`；
  当前本地版本另有 6 个 MID360s/仓库辅助文件。
- 实质差异位于 `src/comm/pub_handler.*` 等文件：DeepGlint 为 Unitree G1
  倒装 MID-360，将 JSON 点云外参旋转同时应用到 IMU 角速度和加速度。
- 当前本地驱动保留其已经验证的行为；倒装修正不合并。安装姿态通过本
  功能包的 LiDAR-IMU 外参与车体 TF 明确配置。

完整的逐文件差异列表和适配边界见 `DRIVER_COMPATIBILITY.md`。

原上游 FAST-LIO 仍依赖旧 `livox_ros_driver/CustomMsg`。本次适配已：

- 改为使用现有 `livox_ros_driver2/CustomMsg`；
- 增加 `lidar_type: 5` 的 Driver2 `sensor_msgs/PointCloud2` 路径；
- 解析 `x/y/z/intensity/tag/line/timestamp`；
- 对 Driver2 `FLOAT64` 绝对纳秒时间戳减去首点时间，并换算为 FAST-LIO
  去畸变需要的相对毫秒；
- 对 Driver2 包聚合产生的轻微帧内时间回退稳定排序，确保反向去畸变
  遍历使用单调点时间。

## 目录与数据流

```text
/livox/lidar (Driver2 PointCloud2) ─┐
                                    ├─> preprocess
/livox/imu (sensor_msgs/Imu) ───────┘
  -> IMU propagation / undistortion
  -> iterated EKF scan-to-map update
  -> ikd-Tree incremental map
  -> /fast_lio_humanoid/odometry
  -> /fast_lio_humanoid/cloud_registered
  -> output_directory/scans.pcd

fixed PCD/PLY + FAST-LIO odometry/registered cloud + /initialpose
  -> Open3D coarse/fine registration
  -> map -> camera_init
  -> /fast_lio_humanoid/localization
  -> /fast_lio_humanoid/localization_confidence
```

核心代码：

- `src/fast_lio/laserMapping.cpp`：ROS 接口、同步、EKF/地图主循环、输出。
- `src/fast_lio/preprocess.cpp`：Driver2 CustomMsg/PointCloud2 预处理。
- `src/fast_lio/IMU_Processing.hpp`：IMU 初始化、传播和点云去畸变。
- `include/fast_lio/ikd-Tree/`：增量 ikd-Tree。
- `src/open3d_loc/global_localization.cpp`：固定地图加载、初始位姿和定位循环。
- `src/open3d_loc/open3d_registration/`：Open3D 配准。

## 依赖与编译

基础建图依赖 ROS Noetic、PCL、Eigen、Boost、OpenMP 和当前工作空间中的
`livox_ros_driver2`。固定地图定位还依赖 ROS `urdf` 和与目标 ARM64 系统
ABI 兼容的 Open3D C++ SDK。EAORA04 已验证的组合为 Open3D 0.14.1、
C++11 ABI、共享库、CPU-only；Open3D 自身使用 Eigen 3.4.0，不能在构建
Open3D 时改用 Ubuntu 20.04 的 Eigen 3.3.7。

```bash
cd /home/nvidia/test-onion-ws
source /opt/ros/noetic/setup.bash

# EAORA04 新镜像中的已验证路径：
export PATH=/opt/cmake-3.22.6-linux-aarch64/bin:$PATH
export Open3D_DIR=/opt/open3d-0.14.1/lib/cmake/Open3D

catkin_make --pkg fast_lio_localization_humanoid \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpen3D_DIR="$Open3D_DIR" \
  -DBUILD_OPEN3D_LOCALIZATION=ON
source devel/setup.bash
rospack find fast_lio_localization_humanoid
```

找不到 Open3D 时，CMake 会明确警告并只构建建图节点。安装/指定 Open3D
后重新编译，才会生成 `fast_lio_humanoid_localization`。

## EAORA04 实测结果

2026-08-01 在 EAORA04 ARM64、ROS1 Noetic、新镜像
`sydl/noetic-dev:eaora04` 的持久容器 `sydl-noetic-final` 中完成 Release
编译、MID-360 rosbag 建图和固定地图加载测试。本轮未使用 `ruimove`：

- `fast_lio_humanoid_mapping` 编译成功，生成 831,264 字节可执行文件；
- ARM64 Open3D 0.14.1 安装于 `/opt/open3d-0.14.1`，PCD 读写和 ICP
  最小 C++ 测试通过；
- `fast_lio_humanoid_localization` 编译成功，生成 544,464 字节可执行文件；
- 测试包为 `/home/nvidia/mid360-20260723.bag`，SHA-256 为
  `00b16cfa828be86bc23856a2f342e0ac62d6e7a1df8abd9867a0d5f23d7b2374`，
  时长 33.791 秒，包含 337 帧 PointCloud2 和 6,759 帧 IMU；
- 以 `0.5` 倍速播放后输出 334 条单调里程计，覆盖 33.300 秒；
- 保存二进制 PCD：1,440,009 点、46,080,541 字节，所有 XYZ 均为有限值；
- 地图 XYZ 范围约为
  `[-6.146, -22.245, -1.179]` 到 `[19.572, 10.219, 3.459]`；
- 地图 SHA-256 为
  `690ca8a97288c8a3264486c0576bb7be3ed8948ada950ffbba6e690709b14a52`；
- 测试日志 `FATAL=0`、`ERROR=0`，无同名节点冲突；
- 定位节点用该 PCD 完成加载、降采样、法向量计算并进入等待里程计状态，
  烟测日志 `FATAL=0`、`ERROR=0`。

`0.5` 倍速结果证明当前 Driver2 消息适配和建图链路可用，不等同于已经
证明原速实时性能。定位测试证明 Open3D/ROS 链接和地图初始化可用，尚不
等同于完成了带 `/initialpose` 的全流程定位精度评估。

## MID-360 建图

先核对实际消息，不要只按话题名判断：

```bash
rostopic type /livox/lidar
rostopic type /livox/imu
rostopic echo -n 1 /livox/lidar/fields
```

实时建图：

```bash
roslaunch fast_lio_localization_humanoid mapping_mid360.launch \
  save_map:=true \
  output_directory:=/tmp/fast_lio_localization_humanoid
```

rosbag 建图分两个终端运行。后台或 SSH 无人值守时必须让 rosbag 不读取
终端输入：

```bash
roslaunch fast_lio_localization_humanoid mapping_mid360.launch \
  use_sim_time:=true \
  save_map:=true \
  output_directory:=/tmp/fast_lio_localization_humanoid

rosbag play /home/nvidia/mid360-20260723.bag \
  --clock -r 0.5 < /dev/null
```

播放结束后向建图节点发送 `SIGINT`，最终地图为
`/tmp/fast_lio_localization_humanoid/scans.pcd`。`0.5` 倍速是验证正确性
的保守起点，不代表已经证明 10 Hz 实时性能。

本包使用 roscpp 默认 SIGINT 关闭流程，保证主循环先退出再写入 PCD；
不要用 `SIGKILL` 终止节点，否则无法执行最终地图保存。

## Tanway TW360/Halo rosbag 建图

`mapping_tanway_tw360.launch` 支持本项目既有 Halo bag 的实际字段布局：

```text
/fusion_rslidar_points: FLOAT32 x/y/z/intensity, INT32 channel, UINT32 t_sec/t_usec
/tanwaylidar_imu:       sensor_msgs/Imu
point time:             t_sec + t_usec * 1e-6
```

处理器以点云头时间为扫描起点，把绝对逐点时间换算为相对毫秒，并按逐点
时间稳定排序，供 FAST-LIO IMU 去畸变使用。离线运行示例：

```bash
roslaunch fast_lio_localization_humanoid mapping_tanway_tw360.launch \
  use_sim_time:=true \
  save_map:=true \
  output_directory:=/tmp/fast_lio_tanway

rosbag play /home/nvidia/ssd-data/bags/tanwei_tw360.bag \
  --clock -r 0.25 < /dev/null
```

该 bag 没有提供 `TanwayIMU` 到 `rslidar_top` 的 TF/标定话题，因此专用
配置中的 LiDAR-IMU 外参暂以单位阵作为测试起点。正式部署前必须使用实测
外参替换 `config/tanway_tw360.yaml` 中的 `extrinsic_T/R`；地图成功生成
本身不能证明外参或轨迹精度正确。

2026-08-01 在 EAORA04 的 `sydl-noetic-final` 中以 `0.25` 倍速完整播放
`/home/nvidia/ssd-data/bags/tanwei_tw360.bag`。该 bag 的 SHA-256 为
`bfa5966639ff353bc35346b7b49ceed4ef3a14f1c04dd9f2d904c985fbf29df4`，
包含 766 帧点云和 15,334 帧 IMU。实测输出 762 条时间单调的里程计，
`FATAL=0`、`ERROR=0`，保存 6,606,511 点、211,408,605 字节的二进制
PCD；地图 SHA-256 为
`9acc0dc6372dfef40415ce8b0358fd48045d3d82d1c00af1f48b02b4189cd84f`。
地图所有 XYZ 均为有限值，范围约为
`[-150.712, -89.471, -16.848]` 到 `[5.139, 69.455, 21.940]`。

原始 IMU 时间戳有 3 次 1.553 ms 的轻微回退；当前 FAST-LIO 检测到回退
后清空瞬时 IMU 缓冲，但对应里程计最大单帧位移仍为 0.248 m，没有出现
轨迹跳变。该结果证明字段适配和离线建图链路可用，外参准确性及真实轨迹
精度仍需带标定/真值的测试确认。

## 固定地图定位

```bash
roslaunch fast_lio_localization_humanoid localization_mid360.launch \
  map:=/absolute/path/map.pcd
```

定位所用地图、FAST-LIO 里程计和初始位姿必须在同一坐标定义下。默认
LiDAR-IMU 外参来自上游 MID-360 配置，只能作为起点；换雷达、改变安装
方式或使用车体坐标时必须重新标定并同步修改：

- `config/mid360.yaml` 的 `mapping/extrinsic_T`、`extrinsic_R`；
- `launch/localization_mid360.launch` 的静态 TF；
- `config/localization.yaml` 的 frame/topic/体素与阈值参数。

## 输出话题

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/fast_lio_humanoid/odometry` | `nav_msgs/Odometry` | 运行起点局部坐标中的 LIO 位姿 |
| `/fast_lio_humanoid/cloud_registered` | `sensor_msgs/PointCloud2` | 去畸变并变换到 LIO 里程计坐标的当前帧 |
| `/fast_lio_humanoid/path` | `nav_msgs/Path` | LIO 轨迹 |
| `/fast_lio_humanoid/localization` | `geometry_msgs/PoseStamped` | 固定地图坐标中的定位结果 |
| `/fast_lio_humanoid/localization_confidence` | `std_msgs/Float32` | Open3D 配准 fitness |
| `/fast_lio_humanoid/localization_delay_ms` | `std_msgs/Float32` | 定位结果相对里程计时间戳的延迟 |

## 已知限制

- Open3D 的 ARM64 可用性和速度取决于实际安装版本，不能用上游 x86
  30 ms 数据代替目标设备实测。
- PCD 全量累计会占用内存；长距离建图应设置正数 `pcd_save/interval`
  分段保存。
- 精确建图依赖正确 LiDAR-IMU 外参和同步；默认外参不是所有车辆安装的
  通用标定。
- 本包保留上游算法代码及原始版权声明。来源、裁剪范围和许可证说明见
  `UPSTREAM.md` 与 `LICENSES/`；驱动差异证据见
  `DRIVER_COMPATIBILITY.md`。
