# Onion-LO-Plus Ultra

ROS 1 continuous-time LiDAR odometry, mapping, and map-based localization for
Livox MID-360.

This derivative adds:

- `livox_ros_driver2` `sensor_msgs/PointCloud2` input;
- ARM-safe packed PointCloud2 timestamp parsing;
- compressed PCD global-map saving with `x/y/z/intensity/label`;
- immutable PCD loading for scan-to-map localization;
- pose, odometry, trajectory, global-map, and TF output.

## Platform

- Ubuntu 20.04
- ROS Noetic
- C++17
- Livox SDK2
- PCL, Eigen, Sophus, Ceres, TBB, fmt, and OctoMap

## Clone

```bash
mkdir -p ~/onion_ws/src
cd ~/onion_ws/src
git clone https://github.com/2573461877/onion-lo-plus-ultra.git
```

Copy Livox ROS Driver 2 into the repository root before building:

```bash
cp -a /path/to/livox_ros_driver2 \
  ~/onion_ws/src/onion-lo-plus-ultra/
```

## Build

Prepare the locally installed driver's ROS 1 manifest and build from the
workspace root:

```bash
cd ~/onion_ws/src/onion-lo-plus-ultra/livox_ros_driver2
cp package_ROS1.xml package.xml

cd ~/onion_ws
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Run

Start the MID-360 driver:

```bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

Start Onion-LO++ in mapping mode:

```bash
roslaunch onion_lo_plus livox.launch
```

Save the global PCD map:

```bash
rosservice call /onion_lo_plus_node/save_map
```

Load a map for localization:

```bash
roslaunch onion_lo_plus livox.launch \
  localization_mode:=true \
  map_path:=/absolute/path/to/onion_map.pcd
```

Pose outputs:

```text
/onion_lo_plus_node/pose
/onion_lo_plus_node/odometry
/onion_lo_plus_node/trajectory
/onion_lo_plus_node/global_map
odom -> base_link TF
```

## Attribution

Based on [huashu996/Onion-LO-Plus](https://github.com/huashu996/Onion-LO-Plus).
Original copyright and license notices are retained in the source files.

Livox ROS Driver 2 is not included in this repository. Obtain it separately
from [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2).

The original project also acknowledges KISS-ICP and Traj-LO.
