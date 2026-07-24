# Onion global relocalization adapter

This ROS1 package keeps global initialization separate from Onion-LO++ local
scan-to-map tracking. The first implementation uses the upstream
`hdl_global_localization` package pinned as a Git submodule.

## Runtime flow

1. Load the persistent PCD, or an optional coarse voxelized copy of it, for
   global feature matching.
2. Send its XYZ fields to `/hdl_global_localization/set_global_map`.
3. Query `/hdl_global_localization/query` with a live LiDAR frame and retain
   its Top-K coarse candidates.
4. Crop a local submap around every candidate, run NDT followed by ICP
   against the dense Onion map, and reject candidates that fail dense
   inlier, RMSE, azimuth-coverage, or correction-magnitude checks.
5. Publish only the best accepted refined pose on the standard
   `/initialpose` topic.
6. Onion starts local tracking only after receiving that pose.

The only Onion-specific boundary is `/initialpose`, so another global
relocalization implementation can replace this adapter without changing the
estimator.

## Build

Initialize the pinned upstream package after cloning:

```bash
git submodule update --init --recursive
catkin_make -DCMAKE_BUILD_TYPE=Release
```

TEASER++ is intentionally disabled. The default engine is the CPU-only
`FPFH_RANSAC` implementation. The launch defaults downsample the dense Onion
centroid map and query cloud to 1.0 m before feature extraction; the original
upstream 0.5 m/large-neighborhood settings are too expensive for this map on
the Jetson CPU. All feature and RANSAC limits remain launch arguments for
later tuning.

## Launch

```bash
roslaunch onion_global_relocalization hdl_onion_relocalization.launch \
  map_path:=/absolute/path/to/onion_map.pcd
```

For a large dense map, keep `map_path` and `refinement_map_path` pointed at the
full-resolution map used by Onion and pass a separately voxelized copy through
`global_map_path`. This reduces one-time FPFH map initialization without
lowering either local tracking or candidate-refinement resolution:

```bash
roslaunch onion_global_relocalization hdl_onion_relocalization.launch \
  map_path:=/absolute/path/to/onion_map.pcd \
  global_map_path:=/absolute/path/to/onion_map_voxel_2m.pcd \
  refinement_map_path:=/absolute/path/to/onion_map.pcd
```

The production defaults deliberately reject ambiguous outdoor matches:
validation requires at least 45% of the 0.2 m query voxels to have a dense-map
neighbor within 0.3 m, no more than 0.18 m inlier RMSE, and support from at
least 6 of 12 azimuth sectors. These values are launch arguments, but lowering
them can turn repeated road geometry into a confident false positive. A
rejected candidate does not reset Onion.

Dense refinement improves candidate precision and failure detection; it
cannot recover the true position when FPFH-RANSAC fails to include it in
Top-K. The adapter logs every coarse and refined candidate so retrieval recall
and refinement quality can be evaluated separately.

For the existing diagnostic bag:

```bash
roslaunch onion_global_relocalization hdl_onion_relocalization.launch \
  map_path:=/home/nvidia/test-onion-ws/src/onion-lo-plus-ultra/onion-lo++/results/onion_map.pcd \
  bagfile:=/home/nvidia/mid360-20260723.bag \
  bag_rate:=0.05 \
  visualize:=false
```

To request another solution after startup:

```bash
rosservice call /onion_global_relocalization/relocalize
```

The selected HDL candidate is also published as
`/onion_global_relocalization/candidate_pose` for inspection.

## Upstream

- Repository: <https://github.com/koide3/hdl_global_localization>
- Pinned commit: `a6a6b69929ac371d9531d0dcdef3c1593fd49245`
- License: BSD, retained in the upstream submodule
