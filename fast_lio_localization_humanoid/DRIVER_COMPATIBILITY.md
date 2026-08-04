# Livox ROS Driver2 兼容性审计

审计对象：

- 当前项目：`livox_ros_driver2/`
- 上游项目：`FAST_LIO_LOCALIZATION_HUMANOID/livox_ros_driver2/`
- 上游代码基线：`e4234d4e559548bfe0afdf407990bfb206e183c0`
- 比较方法：对两目录的普通文件按相对路径计算 SHA-256；未跟随链接，
  未比较构建产物。

## 结论

当前项目中的 Driver2 版本标识为 1.2.6，上游副本为 1.2.4。两者共有
84 个内容完全相同的文件，包含消息定义，因此 Driver2 `CustomMsg` 消息
结构兼容。本功能包复用当前项目中的驱动，不复制上游副本，也不修改
`livox_ros_driver2/`。

上游副本的关键行为变化是：将配置中的 LiDAR 外参旋转同时应用到 IMU
角速度与线加速度，用来适配 Unitree G1 上倒装的 MID-360。当前项目版本
包含其已验证的 MID360s 支持，不能直接被较旧上游副本替换。倒装或其他
安装姿态应在本功能包的 LiDAR-IMU 外参及车体 TF 中显式表达。

## 内容不同的共有文件

1. `build.sh`
2. `CHANGELOG.md`
3. `CMakeLists.txt`
4. `config/MID360_config.json`
5. `README.md`
6. `src/call_back/livox_lidar_callback.cpp`
7. `src/call_back/livox_lidar_callback.h`
8. `src/comm/pub_handler.cpp`
9. `src/comm/pub_handler.h`
10. `src/include/livox_ros_driver2.h`
11. `src/lds_lidar.cpp`
12. `src/lds_lidar.h`
13. `src/parse_cfg_file/parse_livox_lidar_cfg.h`

## 仅当前项目存在

1. `.gitignore`
2. `config/MID360s_config.json`
3. `launch_ROS1/msg_MID360s.launch`
4. `launch_ROS1/rviz_MID360s.launch`
5. `launch_ROS2/msg_MID360s_launch.py`
6. `launch_ROS2/rviz_MID360s_launch.py`

## 仅上游副本存在

1. `package.xml`

上游 `package.xml` 属于其工作区封装差异，不需要引入；当前项目的
Driver2 已有自己的构建与运行方式。

## 本功能包的适配点

- 旧的 `livox_ros_driver/CustomMsg` include 和命名空间改为
  `livox_ros_driver2/CustomMsg`。
- 为 Driver2 `sensor_msgs/PointCloud2` 增加 MID-360 预处理分支。
- 读取 `x/y/z/intensity/tag/line/timestamp` 字段。
- 将 Driver2 的 `FLOAT64` 绝对纳秒时间戳转换为每帧相对毫秒，供
  FAST-LIO 点级去畸变使用。
- 对 Driver2 聚合时出现的轻微帧内时间回退进行稳定排序，保证 FAST-LIO
  反向去畸变遍历使用单调点时间。
- 所有雷达、IMU 话题和 LiDAR-IMU 外参通过 YAML/launch 配置，不依赖
  上游驱动的 G1 专用 IMU 旋转补丁。
