# Onion global relocalization

`onion_relocalization` is one replaceable ROS1 package containing two
independent global-initialization implementations for Onion-LO++:

- `onion_hdl_relocalization_node`: HDL global localization with dense-map
  NDT/ICP refinement and geometry validation.
- `onion_scancontext_relocalization_node`: official Polar Scan Context C++
  Top-K retrieval, KISS-Matcher 6DoF registration, GICP refinement and
  geometry validation.
- `onion_scancontext_database_builder`: persistent keyframe/descriptor
  database builder for the Scan Context path.

The algorithms remain separate executables. Topics, map/database paths,
thresholds, point timing, crop bounds and algorithm selection are launch/YAML
parameters.

## Shared vehicle-frame pipeline

The Halo launch files use `onion_cloud_preprocessing` once:

```text
/fusion_rslidar_points
  -> crop in vehicle_link
  -> /onion/points_filtered_sensor  (rslidar_top; Onion internal tracking)
  -> /onion/points_filtered_vehicle (vehicle_link; HDL/Scan Context)
```

Both relocalizers therefore publish `/initialpose` as
`odom -> vehicle_link`. Onion converts that public vehicle pose to its
internal `odom -> rslidar_top` pose before scan-to-map tracking, and publishes
continuous `/odometry`, `/pose`, `/trajectory` and TF back in
`vehicle_link`. Scans received while global matching is running are buffered
and replayed from the `/initialpose` timestamp, preventing a stale global pose
from being applied directly to a newer moving-vehicle scan.

## Build the Scan Context database

The mapping metrics CSV used by the Halo pipeline contains
`odom -> vehicle_link`. The database builder applies the same TF vehicle crop,
preserves point timestamps, converts raw bag points to `vehicle_link`, then
deskews and aggregates them in a vehicle-frame keyframe:

```bash
roslaunch onion_relocalization halo_build_scancontext_database.launch \
  bag_path:=/path/data.bag \
  reference_poses_csv:=/path/reference_mapping_metrics.csv \
  output_directory:=/path/scancontext_database_vehicle_v3
```

Each keyframe stores its Scan Context descriptor in `descriptors/*.scd`.
At runtime descriptors are loaded immediately and only Top-K candidate PCDs
are loaded. Build this database once for a stable map and reuse it.

Only the classic Polar Scan Context C++ source released by
`gisbi-kim/scancontext_tro` is included. MATLAB-only experimental variants
are not included.

## Select HDL or Scan Context

For the current Halo vehicle:

```bash
roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=scancontext ...

roslaunch onion_relocalization halo_relocalization_selector.launch \
  method:=hdl ...
```

The explicit launch files remain available:

```bash
roslaunch onion_relocalization \
  halo_scancontext_kiss_onion_relocalization.launch ...

roslaunch onion_relocalization halo_hdl_onion_relocalization.launch ...
```

Common Onion handoff parameters (`initial_pose_cloud_buffer_size`,
`initial_pose_replay_tolerance_sec` and `traj_voxel_size`) are exposed by both
method launch files and by the selector. The Scan Context path keeps
KISS-Matcher's ratio test enabled to avoid randomized correspondence
truncation.

For a moving Scan Context query, pass a time-aligned wheel/INS/local-LiDAR
odometry topic:

```bash
roslaunch onion_relocalization \
  halo_scancontext_kiss_onion_relocalization.launch \
  relative_odom_topic:=/local_odometry ...
```

That source must exist before global relocalization; Onion cannot be its own
pre-initialization odometry because Onion waits for `/initialpose`. An empty
topic is suitable only for a stationary test.

## Evaluation reference

An Onion mapping trajectory from the same bag is useful as a repeatability
reference, not physical ground truth. It shares the same LiDAR, calibration,
map and drift. Use RTK/INS, surveyed landmarks, total station or motion
capture for an independent 5 cm accuracy claim.
