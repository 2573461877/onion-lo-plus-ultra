# Device verification — 2026-07-27

Target:

- host: `nvidia@192.168.2.44`
- Docker: `ruimove`
- ROS: Noetic on ARM64
- workspace: `/home/nvidia/test-onion-ws`

## Build

The merged workspace completed a Release build:

```text
[100%] Built target vehicle_crop_filter_node
[100%] Built target onion_lo_plus_node
[100%] Built target onion_hdl_relocalization_node
[100%] Built target onion_scancontext_database_builder
[100%] Built target onion_scancontext_relocalization_node
```

The package contains only the released Polar Scan Context C++ module; no
MATLAB implementation is included. KISS-Matcher, ROBIN, PMC and Xenium are
isolated inside `onion_relocalization`.

## Current vehicle-frame database

- bag: `/home/nvidia/tanwei_tw360.bag`
- raw topic: `/fusion_rslidar_points`
- output: `onion-lo++/results/scancontext_database_vehicle_v3`
- anchor frame: `vehicle_link`
- 38 keyframes from 765 accepted frames
- one frame rejected during deskew
- 46.64% of finite raw points removed by the configured vehicle crop

## Final three-position test

The test used the same-bag Onion mapping trajectory for relative motion and
repeatability comparison. It is not independent physical ground truth.

| bag start | selected rank | initial 3D difference | matching time | Onion tail 3D P95 |
|---:|---:|---:|---:|---:|
| 10 s | Top-1 | 0.003 m | 859.6 ms | 0.078 m |
| 35 s | Top-1 | 0.014 m | 1082.5 ms | 0.117 m |
| 60 s | Top-1 | 0.007 m | 1002.0 ms | 0.103 m |

The `/initialpose` messages describe `odom -> vehicle_link`. Onion logged the
conversion to `rslidar_top` and replayed 6–7 buffered scans from the initial
pose timestamp. No tracking failure was emitted. The replay fixed the previous
35 s runaway result (tail P95 about 10.825 m before, 0.117 m after).

The initial relocalization result meets 5 cm in this repeatability test.
Continuous Onion tracking does not: its tail P95 remains 7.8–11.7 cm, and the
first roughly one second after handoff contains a larger initialization
transient. RTK/INS or another independent reference is required before any
absolute 5 cm claim.

Launch selection parses to distinct implementations:

```text
method:=scancontext
  /onion_scancontext_relocalization
  /onion_lo_plus_node

method:=hdl
  /hdl_global_localization
  /onion_hdl_relocalization
  /onion_lo_plus_node
```
