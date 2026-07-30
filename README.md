# Onion-LO-Plus Ultra

ROS 1 continuous-time LiDAR odometry, mapping, map localization, and
replaceable global relocalization for ARM64 deployment.

## Packages

- `onion-lo++`: PointCloud2 odometry, mapping, PCD loading, and continuous
  scan-to-map localization.
- `onion_cloud_preprocessing`: shared TF-aware vehicle-body crop. It publishes
  both sensor-frame and `vehicle_link` point clouds without losing per-point
  timestamps.
- `onion_relocalization`: one package containing the HDL and Scan Context +
  KISS-Matcher global initialization paths.
- `onion_gps_evaluation`: C++ GPS/Onion trajectory registration, fixed-map
  localization accuracy evaluation, segmented holdout evaluation, and
  relocalization convergence reports.
- `hdl_global_localization`: the HDL engine vendored as a normal Catkin
  package. It is no longer a Git submodule and performs no build-time source
  download.

The deployment workspace supplies the ROS 1 LiDAR driver locally. Bags, maps,
diagnostics, generated Scan Context databases, build output, and driver source
are intentionally not committed.

## Platform

- Ubuntu 20.04
- ROS Noetic
- C++17
- PCL, Eigen, Sophus, Ceres, TBB, fmt, OpenCV, and OctoMap

## Build

Place this repository and the required ROS 1 LiDAR driver in a Catkin
workspace, then build:

```bash
cd ~/onion_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Mapping

MID-360:

```bash
roslaunch onion_lo_plus livox.launch
```

Tanway Halo with the shared `vehicle_link` crop:

```bash
roslaunch onion_lo_plus halo_outdoor.launch
```

Save the current global map:

```bash
rosservice call /onion_lo_plus_node/save_map
```

## Global relocalization

The Halo selector switches implementations without changing the shared
preprocessing or Onion handoff:

```bash
roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=scancontext

roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=hdl
```

Both paths publish `odom -> vehicle_link` on `/initialpose`. Onion converts
that public pose to its internal LiDAR tracking frame, replays buffered scans
from the initial-pose timestamp, and continues publishing the vehicle pose.

Important parameters, including map/database paths, topics, crop bounds,
initial-pose buffer size, replay tolerance, and registration voxel size, are
available through YAML and launch arguments.

## GPS-based evaluation

The evaluation package is a read-only C++ sidecar. It does not modify Onion,
publish `/initialpose`, or control the vehicle. Its three workflows use
separate nodes and launch files.

Register one GPS/Onion trajectory and save the fixed map-to-GPS transform:

```bash
roslaunch onion_gps_evaluation trajectory_registration.launch
```

Evaluate a later localization run without refitting the transform:

```bash
roslaunch onion_gps_evaluation localization_accuracy_evaluation.launch
```

When only one continuous bag is available, use its earlier portion for
registration and its strictly later portion for evaluation:

```bash
roslaunch onion_gps_evaluation segmented_registration_evaluation.launch
```

The package consumes `/onion_lo_plus_node/odometry`,
`/gps/evaluation/odom_utm`, `/gps/evaluation/fix`, and `/initialpose`.
Each node has a private `finalize` service documented in
`onion_gps_evaluation/README.md`, together with file responsibilities, output
artifacts, coordinate conventions, and the C++ synthetic regression launch.

For unattended offline evaluation, the package also provides a wrapper that
plays one bag to completion and calls the selected workflow's finalize service:

```bash
rosrun onion_gps_evaluation play_bag_and_finalize.sh \
  --bag /data/evaluation.bag \
  --workflow evaluation
```

## Accuracy boundary

The current same-bag regression tests verify successful centimeter-level
global initialization. They do not establish independent absolute accuracy.
Road deployment still requires an independent RTK/INS or surveyed reference,
and a controller-side readiness gate after the initial Onion handoff.

## Licensing

Required third-party copyright and license texts are retained beside vendored
source code. They must remain present while those sources are redistributed.
