/**
MIT License

Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TRAJLO_CONFIG_H
#define TRAJLO_CONFIG_H

#include <string>
#include <vector>

#include <sophus/se3.hpp>
namespace traj {
struct TrajConfig {
  TrajConfig() = default;
  void load(const std::string& filename);
  // dataset
  std::string type;
  bool save_pose = false;
  int exp_key_num = 3000;
  double Resolution_v = 1.0;
  double Resolution_h = 1.0;
  std::string pose_file_path;
  int frame_num = 0;
  int point_num = 0;

  // calibration:
  double time_offset = 0.0;
  Sophus::SE3d T_body_lidar;
  Sophus::SE3d T_body_gt;
  Sophus::SE3d T_vis_lidar;

  // trajectory
  double init_interval = 200.0;
  double seg_interval = 20.0;
  int seg_num = 3;
  float kinematic_constrain = 2.0F;
  double init_pose_weight = 1e9;
  double converge_thresh = 0.001;
  int max_iterations = 20;
  // mapping
  double voxel_size = 1.0;
  double planer_thresh = 0.1;
  double raw_point_num = 30000.0;
  // Hard upper bound for the adaptive registration-map voxel capacity.
  // Keeping this independent from loaded_map_max_points_per_voxel prevents
  // the online mapper from silently growing each voxel to 500 points.
  int max_points_per_voxel = 80;
  // A scan is excluded from the persistent map when any optimized segment
  // has fewer correspondences than this value.
  int min_registration_inliers = 20;
  // Numerical safeguards for the direct normal-equation optimizer.
  double optimizer_damping = 1e-6;
  double max_optimizer_translation_increment = 0.50;
  double max_optimizer_rotation_increment_deg = 20.0;
  double max_optimizer_translation_deviation = 1.0;
  double max_optimizer_rotation_deviation_deg = 30.0;

  // MID-360 PointCloud2 input. The official livox_ros_driver2
  // xfer_format=0 path publishes an absolute FLOAT64 timestamp in ns.
  std::string point_time_field = "timestamp";
  double point_time_scale = 1e-9;
  bool point_time_is_offset = false;

  // Persistent-map / localization mode.
  bool localization_mode = false;
  bool update_loaded_map = false;
  std::string map_path;
  // The validated localization default bounds neighbor-search cost while
  // retaining enough local geometry. Set 0 to keep every finite PCD point.
  int loaded_map_max_points_per_voxel = 150;
  std::vector<double> initial_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
};
}  // namespace traj

#endif  // TRAJLO_CONFIG_H
