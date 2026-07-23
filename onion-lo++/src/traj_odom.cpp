#include "traj_odom.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
std::mutex print_mutex;
bool First_pose{true};
double first_scan_time;
double scan_time;
double total_ONION_ms =  0;
double scan_num =0;
uint64_t getCurrTime() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}
namespace traj {
TrajLOdometry::TrajLOdometry(const TrajConfig& config)
    : config_(config) {

  laser_data_queue.set_capacity(100);
  //Onion
  Resolution_v = config_.Resolution_v;
  Resolution_h = config_.Resolution_h;
  exp_key_num = config_.exp_key_num;
  converge_thresh_ = config.converge_thresh;
  //Traj
  init_interval_ = 1e06*config.init_interval;
  max_frames_ = config.seg_num;
  kinematic_constrain = config_.kinematic_constrain;
  init_pose_weight_ = config.init_pose_weight;
  max_iterations = config_.max_iterations;
  voxel_size = config.voxel_size;
  double planer_thresh = 0.1;
  RAM_NUM = config.raw_point_num;
  localization_mode_ = config.localization_mode;

  if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
    throw std::invalid_argument(
        "Traj/voxel_size must be greater than zero");
  }
  if (config_.max_points_per_voxel <= 0) {
    throw std::invalid_argument(
        "Traj/max_points_per_voxel must be greater than zero");
  }
  if (config_.min_registration_inliers < 0) {
    throw std::invalid_argument(
        "Traj/min_registration_inliers must not be negative");
  }

  Eigen::Vector3d initial_translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d initial_rotation = Eigen::Matrix3d::Identity();
  if (config.initial_pose.size() == 6) {
    initial_translation =
        Eigen::Vector3d(config.initial_pose[0], config.initial_pose[1],
                        config.initial_pose[2]);
    const Eigen::AngleAxisd roll(config.initial_pose[3],
                                 Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(config.initial_pose[4],
                                  Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(config.initial_pose[5],
                                Eigen::Vector3d::UnitZ());
    initial_rotation = (yaw * pitch * roll).toRotationMatrix();
  }
  initial_pose_ = Sophus::SE3d(initial_rotation, initial_translation);
  T_wc_curr = initial_pose_;
  current_pose = initial_pose_;
  T_prior = Sophus::SE3d();
  
  map_.reset(new MapManager(voxel_size, planer_thresh));
  map_->max_points_per_voxel_ =
      localization_mode_ ? config.loaded_map_max_points_per_voxel
                         : config.max_points_per_voxel;
  if (localization_mode_) {
    std::string map_error;
    if (!map_->LoadGlobalMap(config.map_path, !config.update_loaded_map,
                             &map_error)) {
      throw std::runtime_error(map_error);
    }
    std::cout << "\033[32m[Localization] Loaded " << map_->MapPointCount()
              << " map points from " << config.map_path
              << (map_->HasMapLabels() ? " (with Onion labels)"
                                       : " (geometry-only map)")
              << "\033[0m" << std::endl;
  }
  // setup marginalization
  marg_H.setZero(POSE_SIZE, POSE_SIZE);
  marg_b.setZero(POSE_SIZE);

  marg_H.diagonal().setConstant(init_pose_weight_);
}

bool TrajLOdometry::SetInitialPose(const Sophus::SE3d& pose) {
  if (!localization_mode_) return false;

  initial_pose_ = pose;
  T_wc_curr = pose;
  current_pose = pose;
  T_prior = Sophus::SE3d();

  measurements.clear();
  measure_cache.clear();
  points_cache.clear();
  frame_poses_.clear();
  trajectory_.clear();
  deskew_points.clear();
  first_scan_ = true;
  plane_cnt_cache_ = 0;
  marg_H.setZero(POSE_SIZE, POSE_SIZE);
  marg_b.setZero(POSE_SIZE);
  marg_H.diagonal().setConstant(init_pose_weight_);
  return true;
}

pcl::PointCloud<MapPointXYZIL>::Ptr
TrajLOdometry::ExportRegistrationMap() const {
  return map_->ExportMapCloud();
}
Vector6dVector ConvertToVector6d(const Scan::Ptr& curr_points) {
    Vector6dVector result;
    result.reserve(curr_points->points.size());
    for (const auto& pt : curr_points->points) {
        Eigen::Matrix<double,6,1> vec;
        vec << static_cast<double>(pt.x), static_cast<double>(pt.y), static_cast<double>(pt.z), pt.ts, static_cast<double>(pt.intensity), static_cast<double>(pt.label);
        result.push_back(vec);
    }
    return result;
}
std::vector<PointXYZIT> ConvertToPoints(const Vector6dVector& vec6d_list) {
    std::vector<PointXYZIT> points;
    points.reserve(vec6d_list.size());
    for (const auto& vec : vec6d_list) {
        PointXYZIT pt;
        pt.x = static_cast<float>(vec[0]);
        pt.y = static_cast<float>(vec[1]);
        pt.z = static_cast<float>(vec[2]);
        pt.ts = vec[3];
        pt.intensity = static_cast<float>(vec[4]);
        pt.label = static_cast<double>(vec[5]);
        points.push_back(pt);
    }
    std::sort(points.begin(), points.end(), [](const PointXYZIT& a, const PointXYZIT& b) {
        return a.ts < b.ts;
    });
    return points;
}
void TrajLOdometry::Start(const Scan::Ptr curr_points) {
  if (!curr_points.get()) return;
  tracking_healthy_ = true;
  last_registration_inliers_ = 0.0;
  bool has_registration_result = false;
  if (first_scan_) {
    last_begin_t_ns_ = curr_points->timestamp;

    last_end_t_ns_ = last_begin_t_ns_ + init_interval_;
    first_scan_ = false;
  }
  scan_num++;
  Measurement::Ptr measure;
  int point_count = curr_points->points.size();
  Vector6dVector v6d_scan;
	Vector6dVector seg_scan;
	v6d_scan.reserve(point_count);
	seg_scan.reserve(point_count);
	int frame_id = 0;
  v6d_scan = ConvertToVector6d(curr_points);
  //-------Onion-----------
  auto t1 = std::chrono::high_resolution_clock::now();
  onion.Create_Onion(v6d_scan, Resolution_v, Resolution_h, exp_key_num);
  seg_scan = onion.Classifier();
  onion.clear();
  auto t2 = std::chrono::high_resolution_clock::now();
  double duration_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
  total_ONION_ms += duration_ms;
  double avg_onion_time = total_ONION_ms/scan_num;
	std::cout << "\033[33mOnion time: " 
          << std::fixed << std::setprecision(3) 
          << avg_onion_time 
          << " ms\033[0m" << std::endl;
  //-------Onion-----------
	auto seg_scan_ = ConvertToPoints(seg_scan);
	curr_points->points.clear();
	curr_points->points = seg_scan_;
	curr_points->size = seg_scan_.size();
  const double safe_onion_factor =
      std::isfinite(onion.Onion_Factor) && onion.Onion_Factor > 0.0
          ? onion.Onion_Factor
          : 1e-3;
  const double adaptive_capacity =
      std::pow(voxel_size / (safe_onion_factor / 3.0), 3.0);
  const int adaptive_max = static_cast<int>(std::clamp(
      adaptive_capacity, 1.0,
      static_cast<double>(std::numeric_limits<int>::max())));
  if (!localization_mode_) {
    map_->max_points_per_voxel_ =
        std::min(adaptive_max, config_.max_points_per_voxel);
  }
	map_->reg_thresh_ = safe_onion_factor;
	map_->planer_threshold_ = safe_onion_factor/5.0;
  cout<<"onion.Onion_Factor---"<<onion.Onion_Factor<<endl;
  cout<<"onion.Plane_rotio---"<<onion.Plane_rotio<<endl;
  cout<<"map.max_points_per_voxel_---"<<map_->max_points_per_voxel_<<endl;
	window_interval_ = 1e06*config_.seg_interval;
	if (curr_points->points.size()< 0.4*RAM_NUM){
		window_interval_ = 2.0*window_interval_;
		std::cout << "\033[31m[Degenerate Scene] !!!\033[0m" << std::endl;
	}
	PointCloudSegment(curr_points, measure);
	scan_num++;
  while (!measure_cache.empty()) {
    measure = measure_cache.front();
    measure_cache.pop_front();
    bool Degenerate = (measure->plane_ratio > 0.80 || onion.Plane_rotio> 0.80);
    auto NUM = measure->points.size();
    std::vector<Vector6d> points;
    RangeFilter(measure, points, scan_num);
    const auto& tp = measure->tp;

    if (map_->IsInit() && frame_poses_.empty()) {
      T_wc_curr = initial_pose_;
      current_pose = initial_pose_;
      frame_poses_[tp.first] =
          PoseStateWithLin<double>(tp.first, initial_pose_, true);
      T_prior = Sophus::SE3d();
    }
    
    if (!map_->IsInit()) {
      T_wc_curr = Sophus::SE3d();
      map_->MapInit(points);
      frame_poses_[tp.second] = PoseStateWithLin<double>(tp.second, T_wc_curr, true);
      trajectory_.emplace_back(tp.first, T_wc_curr);
      map_->SetInit();
      T_prior = Sophus::SE3d();
      for (const auto& point : measure->points) {
        PointXYZI output;
        output.x = point.x;
        output.y = point.y;
        output.z = point.z;
        output.intensity = point.intensity;
        output.label = static_cast<float>(point.label);
        deskew_points.emplace_back(output);
      }
    } else {
      measure->pseudoPrior = T_prior;
      measurements[tp] = measure;
      Sophus::SE3d T_w_pred = frame_poses_[tp.first].getPose() * T_prior;
      frame_poses_[tp.second] = PoseStateWithLin<double>(tp.second, T_w_pred);
      map_->PreProcess(points, tp, safe_onion_factor, Degenerate);
      Optimize();
      T_wc_curr = frame_poses_[tp.second].getPose();
      if (!has_registration_result) {
        last_registration_inliers_ = measure->lastInliers;
        has_registration_result = true;
      } else {
        last_registration_inliers_ =
            std::min(last_registration_inliers_, measure->lastInliers);
      }
      if (!T_wc_curr.matrix().allFinite() ||
          !std::isfinite(measure->lastInliers) ||
          measure->lastInliers <
              static_cast<double>(config_.min_registration_inliers)) {
        tracking_healthy_ = false;
      }
      Sophus::SE3d model_deviation = T_w_pred.inverse() * T_wc_curr;
      T_prior = frame_poses_[tp.first].getPose().inverse() * T_wc_curr;
      Marginalize();
			ScanVisData::Ptr visData(new ScanVisData);
			posePair pp{frame_poses_[tp.first].getPose(),
										frame_poses_[tp.second].getPose()};
			visData->T_w = pp.first;
			UndistortRawPoints(measure->points, visData->data, pp, visData->T_w, deskew_points);
    }
    frame_id++;
  }
  if (!frame_poses_.empty()) {
    auto iter = std::prev(frame_poses_.end());
    current_pose = iter->second.getPose();
  }
  if (!current_pose.matrix().allFinite()) {
    tracking_healthy_ = false;
  }
}

/*
 * The analytic Jacobians in the paper are derived in SE(3) form. For the
 * efficiency in our implementation, we instead update poses in SO(3)+R3
 * form. The connection between them has been discussed in
 * https://gitlab.com/VladyslavUsenko/basalt/-/issues/37
 * */
void TrajLOdometry::Optimize() {
  AbsOrderMap aom;
  for (const auto& kv : frame_poses_) {
    aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);
    aom.total_size += POSE_SIZE;
    aom.items++;
  }

  Eigen::MatrixXd abs_H;
  Eigen::VectorXd abs_b;

  for (int iter = 0; iter < max_iterations; iter++) {
    abs_H.setZero(aom.total_size, aom.total_size);
    abs_b.setZero(aom.total_size);
		std::size_t count = measurements.size();
		//std::cout << "Number of measurements: " << count << std::endl;
    for (auto& m : measurements) {
			/*
			struct Measurement {
				using Ptr = std::shared_ptr<Measurement>;
				tStampPair tp;
				std::vector<PointXYZIT> points;  // for visualization
				Sophus::SE3d pseudoPrior;

				Eigen::Matrix<double, 12, 12> delta_H;
				Eigen::Matrix<double, 12, 1> delta_b;
				double lastError = 0;
				double lastInliers = 0;
			};
			*/
      int64_t idx_prev = m.first.first;
      int64_t idx_curr = m.first.second;

      const auto& prev = frame_poses_[idx_prev];
      const auto& curr = frame_poses_[idx_curr];

      posePair pp{prev.getPose(), curr.getPose()};
      const tStampPair& tp = m.second->tp;  //{idx_prev,idx_curr};

      // 1. Geometric constrains from lidar point cloud.
      map_->PointRegistrationNormal({prev, curr}, tp, m.second->delta_H,
                                    m.second->delta_b, m.second->lastError,
                                    m.second->lastInliers);

      // 2. Motion constrains behind continuous movement.
      // Log(Tbe)-Log(prior) Equ.(6)
      {
        Sophus::SE3d T_be = pp.first.inverse() * pp.second;
        Sophus::Vector6d tau = Sophus::se3_logd(T_be);
        Sophus::Vector6d res = tau - Sophus::se3_logd(m.second->pseudoPrior);

        Sophus::Matrix6d J_T_w_b;
        Sophus::Matrix6d J_T_w_e;
        Sophus::Matrix6d rr_b;
        Sophus::Matrix6d rr_e;

        if (prev.isLinearized() || curr.isLinearized()) {
          pp = std::make_pair(prev.getPoseLin(), curr.getPoseLin());
          T_be = pp.first.inverse() * pp.second;
          tau = Sophus::se3_logd(T_be);
        }

        Sophus::rightJacobianInvSE3Decoupled(tau, J_T_w_e);
        J_T_w_b = -J_T_w_e * (T_be.inverse()).Adj();

        rr_b.setIdentity();
        rr_b.topLeftCorner<3, 3>() = pp.first.rotationMatrix().transpose();
        rr_e.setIdentity();
        rr_e.topLeftCorner<3, 3>() = pp.second.rotationMatrix().transpose();

        Eigen::Matrix<double, 6, 12> J_be;
        J_be.topLeftCorner<6, 6>() = J_T_w_b * rr_b;
        J_be.topRightCorner<6, 6>() = J_T_w_e * rr_e;

        double alpha_e = kinematic_constrain * m.second->lastInliers;
        m.second->delta_H += alpha_e * J_be.transpose() * J_be;
        m.second->delta_b -= alpha_e * J_be.transpose() * res;
      }

      int abs_id = aom.abs_order_map.at(idx_prev).first;
      abs_H.block<POSE_SIZE * 2, POSE_SIZE * 2>(abs_id, abs_id) +=
          m.second->delta_H;
      abs_b.segment<POSE_SIZE * 2>(abs_id) += m.second->delta_b;
    }

    // Marginalization Error Term
    // reference: Square Root Marginalization for Sliding-Window Bundle
    // Adjustment (N Demmel, D Schubert, C Sommer, D Cremers and V Usenko)
    // https://arxiv.org/abs/2109.02182
    Eigen::VectorXd delta;
    for (const auto& p : frame_poses_) {
      if (p.second.isLinearized()) {
        delta = p.second.getDelta();
      }
    }
    abs_H.block<POSE_SIZE, POSE_SIZE>(0, 0) += marg_H;
    abs_b.head<POSE_SIZE>() -= marg_b;
    abs_b.head<POSE_SIZE>() -= (marg_H * delta);

    Eigen::VectorXd update = abs_H.ldlt().solve(abs_b);
    double max_inc = update.array().abs().maxCoeff();

    if (max_inc < converge_thresh_) {
      break;
    }

    for (auto& kv : frame_poses_) {
      int idx = aom.abs_order_map.at(kv.first).first;
      kv.second.applyInc(update.segment<POSE_SIZE>(idx));
    }
  }

  // update pseudo motion prior after each optimization
  int64_t begin_t = measurements.begin()->first.first;
  int64_t end_t = measurements.begin()->first.second;
  auto begin = frame_poses_[begin_t];
  auto end = frame_poses_[end_t];

  const int64_t m0_t = begin_t;
  for (auto m : measurements) {
    if (m.first.first == m0_t) continue;
    m.second->pseudoPrior = begin.getPose().inverse() * end.getPose();
    begin = frame_poses_[m.first.first];
    end = frame_poses_[m.first.second];
  }
}

void TrajLOdometry::Marginalize() {
  // remove pose with minimal timestamp
  if (measurements.size() >= max_frames_) {
    const auto& tp = measurements.begin()->first;
    const posePair pp{frame_poses_[tp.first].getPose(),
                      frame_poses_[tp.second].getPose()};
    map_->Update(pp, tp);

    Eigen::VectorXd delta = frame_poses_[tp.first].getDelta();

    Eigen::Matrix<double, 12, 12> marg_H_new =
        measurements.begin()->second->delta_H;
    Eigen::Matrix<double, 12, 1> marg_b_new =
        measurements.begin()->second->delta_b;
    marg_H_new.topLeftCorner<POSE_SIZE, POSE_SIZE>() += marg_H;

    marg_b_new.head<POSE_SIZE>() -= marg_b;
    marg_b_new.head<POSE_SIZE>() -= (marg_H * delta);

    Eigen::MatrixXd H_mm_inv =
        marg_H_new.topLeftCorner<6, 6>().fullPivLu().solve(
            Eigen::MatrixXd::Identity(6, 6));
    marg_H_new.bottomLeftCorner<6, 6>() *= H_mm_inv;

    marg_H = marg_H_new.bottomRightCorner<6, 6>();
    marg_b = marg_b_new.tail<6>();
    marg_H -=
        marg_H_new.bottomLeftCorner<6, 6>() * marg_H_new.topRightCorner<6, 6>();
    marg_b -= marg_H_new.bottomLeftCorner<6, 6>() * marg_b_new.head<6>();


    // erase
    frame_poses_.erase(tp.first);
    measurements.erase(tp);

    trajectory_.emplace_back(tp.first, pp.first);
    frame_poses_[tp.second].setLinTrue();
  }
}

void TrajLOdometry::PointCloudSegment(Scan::Ptr scan, Measurement::Ptr measure) {
  for (size_t i = 0; i < scan->size; i++) {
    const auto& p = scan->points[i];
    int64_t ts_ns = static_cast<int64_t>(p.ts * 1e9);
    if (ts_ns < last_end_t_ns_) {
      points_cache.emplace_back(p);
      if (p.label != 2.0) ++plane_cnt_cache_;
    } else {
        if (points_cache.empty()) {
          last_begin_t_ns_ = last_end_t_ns_;
          last_end_t_ns_ = last_begin_t_ns_ + window_interval_;
          if (ts_ns < last_end_t_ns_) {
            points_cache.emplace_back(p);
            if (p.label != 2.0) ++plane_cnt_cache_;
          }
          continue;
        }
        measure.reset(new Measurement);
        measure->tp = {last_begin_t_ns_, last_end_t_ns_};
        measure->points = points_cache;
        double plane_ratio = static_cast<double>(plane_cnt_cache_) /
                             static_cast<double>(points_cache.size());
        /*
        std::cout << "[Segment] tp=[" << last_begin_t_ns_ << ", " << last_end_t_ns_
                  << "] size=" << points_cache.size()
                  << " plane_ratio=" << plane_ratio << std::endl;
        */
        measure->plane_ratio = plane_ratio;
        measure_cache.push_back(measure);
        plane_cnt_cache_ = 0;
        points_cache.clear();
      last_begin_t_ns_ = last_end_t_ns_;
      last_end_t_ns_ = last_begin_t_ns_ + window_interval_;
      if (ts_ns < last_end_t_ns_) {
        points_cache.emplace_back(p);
        if (p.label  != 2.0) ++plane_cnt_cache_;
      }
    }
  }
}

void TrajLOdometry::RangeFilter(Measurement::Ptr measure,
                                std::vector<Vector6d>& points,
                                int scan_num) {
  points.reserve(measure->points.size());
  const auto& tp = measure->tp;
  double interv = (tp.second - tp.first) * 1e-9;
  double begin_ts = tp.first * 1e-9;
  for (const auto& p : measure->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z)) {
      continue;
    }
    double alpha = (p.ts - begin_ts) / interv;
    //double intensity = p.intensity;
    double label = p.label;
		Vector6d pt;
		pt << static_cast<double>(p.x),
				  static_cast<double>(p.y),
				  static_cast<double>(p.z),
				  alpha,
				  static_cast<double>(scan_num),
				  static_cast<double>(label);

		points.emplace_back(pt);
  }
}

void TrajLOdometry::UndistortRawPoints(
    std::vector<PointXYZIT>& pc_in,
    std::vector<PointXYZI>& pc_out,
    const posePair& pp,
    Sophus::SE3d curr_pose,
    std::vector<PointXYZI>& deskew_points)
{
    if (pc_in.empty()) return;
    const double begin_ts = pc_in.front().ts;
    const double end_ts   = pc_in.back().ts;
    double interv = end_ts - begin_ts;
    if (interv <= 1e-6) {
        interv = 1e-6;
    }
    pc_out.reserve(pc_in.size());
    deskew_points.reserve(pc_in.size());
    Sophus::Vector6d tau;
    {
        Sophus::SE3d T_rel = pp.first.inverse() * pp.second;
        if (!T_rel.matrix().allFinite()) {
            return;
        }
        Eigen::Matrix3d R = T_rel.so3().matrix();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();

        T_rel = Sophus::SE3d(R, T_rel.translation());
        tau = Sophus::se3_logd(T_rel);
    }
    for (const auto& p : pc_in) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        double alpha = (p.ts - begin_ts) / interv;
        alpha = std::clamp(alpha, 0.0, 1.0);
        Sophus::SE3d T_b_i = Sophus::se3_expd(alpha * tau);
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = curr_pose * T_b_i * point;
        PointXYZI po;
        po.x = point.x();
        po.y = point.y();
        po.z = point.z();
        po.intensity = p.intensity;
        po.label = static_cast<float>(p.label);
        pc_out.emplace_back(po);
        deskew_points.emplace_back(po);
    }
}
}  // namespace traj
