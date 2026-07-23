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

#ifndef TRAJLO_ODOMETRY_H
#define TRAJLO_ODOMETRY_H

#include <atomic>
#include <memory>
#include <thread>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <pcl/point_cloud.h>
#include <tbb/concurrent_queue.h>
#include <Eigen/Eigen>

#include <trajlo/common_type.h>
#include <trajlo/config.h>
#include <trajlo/pose_type.h>
#include <trajlo/eigen_utils.hpp>
#include "map_manager.hpp"
#include <onion/onion.hpp>
using namespace std;
namespace traj {
class TrajLOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<TrajLOdometry>;

  explicit TrajLOdometry(const TrajConfig& config);

  void Start(const Scan::Ptr curr_points);
  bool SetInitialPose(const Sophus::SE3d& pose);
  pcl::PointCloud<MapPointXYZIL>::Ptr ExportRegistrationMap() const;
  bool Optimize();
  void Marginalize();
  void PointCloudSegment(Scan::Ptr scan, Measurement::Ptr measure);
  void RangeFilter(Measurement::Ptr measure, std::vector<Vector6d>& points, int scan_num);
  void UndistortRawPoints(std::vector<PointXYZIT>& pc_in,
                                       std::vector<PointXYZI>& pc_out,
                                       const posePair& pp,
                                        Sophus::SE3d curr_pose,
                                       std::vector<PointXYZI>& deskew_points);
  tbb::concurrent_bounded_queue<Scan::Ptr> laser_data_queue;
  bool isFinish = false;  // 由主线程控制退出
  Sophus::SE3d T_wc_curr;
  Sophus::SE3d current_pose;
  std::vector<PointXYZI> deskew_points;
  inline bool IsLocalizationMode() const { return localization_mode_; }
  inline bool IsTrackingHealthy() const { return tracking_healthy_; }
  inline double LastRegistrationInliers() const {
    return last_registration_inliers_;
  }
  inline const std::string& LastFailureReason() const {
    return failure_reason_;
  }
  inline const std::string& LastFailureReport() const {
    return failure_report_;
  }
 private:
  bool FailOptimization(const std::string& reason);
  // 与地图、位姿、缓存相关的成员不变
  MapManager::Ptr map_;
  bool isMove_ = false;
  int scan_num=0;

  Sophus::SE3d T_prior;

  Eigen::aligned_map<tStampPair, Measurement::Ptr> measurements;
  std::deque<Measurement::Ptr> measure_cache;
  std::vector<PointXYZIT> points_cache;

  Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses_;
  using tumPose = std::pair<int64_t, Sophus::SE3d>;
  std::vector<tumPose> trajectory_;

  int64_t last_begin_t_ns_;
  int64_t last_end_t_ns_;
  bool first_scan_ = true;
  size_t plane_cnt_cache_ = 0;
  //Traj
  double converge_thresh_ = 0.01;
  int64_t window_interval_ = 4e7;
  int64_t init_interval_ = 3e8;
  size_t max_frames_ = 4;
  double voxel_size = 1;
  double kinematic_constrain = 2;
  int max_iterations = 20;
  double RAM_NUM;
  double init_pose_weight_ = 1e9;
  bool localization_mode_ = false;
  Sophus::SE3d initial_pose_;
  bool tracking_healthy_ = true;
  double last_registration_inliers_ = 0.0;
  std::string failure_reason_;
  std::string failure_report_;
  std::ostringstream optimization_trace_;
  std::size_t diagnostic_scan_index_ = 0;
  std::size_t diagnostic_raw_points_ = 0;
  std::size_t diagnostic_classified_points_ = 0;
  int diagnostic_segment_index_ = -1;
  std::size_t diagnostic_segment_points_ = 0;
  double diagnostic_segment_plane_ratio_ = 0.0;
  double diagnostic_onion_factor_ = 0.0;
  double diagnostic_onion_plane_ratio_ = 0.0;

  //Onion
  double Resolution_v;
  double Resolution_h;
  int exp_key_num;
  // 边缘化参数
  AbsOrderMap marg_order;
  Eigen::MatrixXd marg_H;
  Eigen::VectorXd marg_b;
  Onion onion;
  TrajConfig config_;
};


}  // namespace traj

#endif  // TRAJLO_ODOMETRY_H
