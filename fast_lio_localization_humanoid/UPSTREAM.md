# Upstream provenance

This package vendors and integrates source code from:

- `deepglint/FAST_LIO_LOCALIZATION_HUMANOID`
- upstream commit `e4234d4e559548bfe0afdf407990bfb206e183c0`
- source URL:
  <https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID>

Imported code is limited to the FAST-LIO and Open3D localization sources,
headers, and the FAST-LIO message definition. The upstream `data`, `doc`,
`Log`, `PCD`, RViz media, videos, images, sample maps, and bundled
`livox_ros_driver2` directory are intentionally not included.

The workspace's existing `livox_ros_driver2` package is used instead. The
DeepGlint driver fork modifies Driver2 so the configured point-cloud rotation
is also applied to IMU measurements for an upside-down Unitree G1 MID-360.
That hardware-specific driver change is not imported. Sensor mounting and
LiDAR-to-IMU calibration remain explicit runtime configuration.

FAST-LIO source files retain their original notices. The upstream FAST-LIO
repository ships the GPL-2.0 text in `LICENSES/GPL-2.0.txt`; individual source
files also contain BSD-3-Clause notices. The imported Open3D conversion code
contains Apache-2.0 notices. Review all upstream license obligations before
redistribution.
