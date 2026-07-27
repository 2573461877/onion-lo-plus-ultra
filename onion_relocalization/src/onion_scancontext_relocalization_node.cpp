#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kiss_matcher/KISSMatcher.hpp>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "Scancontext.h"
#include "onion_relocalization/descriptor_io.hpp"
#include "onion_relocalization/pointcloud_deskew.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;
using onion_relocalization::PointTimeConfig;
using onion_relocalization::TimedPose;

std::string normalizeFrameId(const std::string& frame_id) {
  const auto first_character = frame_id.find_first_not_of('/');
  if (first_character == std::string::npos) return {};
  return frame_id.substr(first_character);
}

struct DatabaseEntry {
  std::string id;
  double stamp_sec = 0.0;
  Eigen::Isometry3d map_from_anchor = Eigen::Isometry3d::Identity();
  std::string coarse_path;
  std::string fine_path;
  std::string anchor_frame;
  CloudPtr coarse;
  CloudPtr fine;
  Eigen::MatrixXd descriptor;
};

struct OdomSample {
  ros::Time stamp;
  Eigen::Isometry3d odom_from_sensor = Eigen::Isometry3d::Identity();
};

struct CloudFrame {
  ros::Time stamp;
  CloudPtr fine{new Cloud};
  bool has_motion_pose = false;
  Eigen::Isometry3d motion_from_sensor = Eigen::Isometry3d::Identity();
};

struct Candidate {
  std::size_t database_index = 0;
  double scancontext_distance = std::numeric_limits<double>::infinity();
  int sector_shift = 0;
};

struct Validation {
  double inlier_fraction = 0.0;
  double rmse_m = std::numeric_limits<double>::infinity();
};

struct EvaluatedCandidate {
  bool valid = false;
  bool accepted = false;
  std::size_t database_index = 0;
  std::size_t rank = 0;
  std::size_t kiss_inliers = 0;
  bool gicp_converged = false;
  double kiss_ms = 0.0;
  double gicp_ms = 0.0;
  Validation validation;
  Eigen::Isometry3d map_from_query = Eigen::Isometry3d::Identity();
  double score = std::numeric_limits<double>::infinity();
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    values.push_back(value);
  }
  return values;
}

Eigen::Isometry3d poseFromValues(const std::vector<std::string>& values,
                                std::size_t offset) {
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.translation() =
      Eigen::Vector3d(std::stod(values.at(offset)),
                      std::stod(values.at(offset + 1)),
                      std::stod(values.at(offset + 2)));
  Eigen::Quaterniond quaternion(std::stod(values.at(offset + 6)),
                               std::stod(values.at(offset + 3)),
                               std::stod(values.at(offset + 4)),
                               std::stod(values.at(offset + 5)));
  quaternion.normalize();
  output.linear() = quaternion.toRotationMatrix();
  return output;
}

Eigen::Isometry3d poseFromOdometry(const nav_msgs::Odometry& message) {
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.translation() =
      Eigen::Vector3d(message.pose.pose.position.x,
                      message.pose.pose.position.y,
                      message.pose.pose.position.z);
  Eigen::Quaterniond quaternion(message.pose.pose.orientation.w,
                               message.pose.pose.orientation.x,
                               message.pose.pose.orientation.y,
                               message.pose.pose.orientation.z);
  if (quaternion.norm() < 1e-8) {
    quaternion = Eigen::Quaterniond::Identity();
  }
  quaternion.normalize();
  output.linear() = quaternion.toRotationMatrix();
  return output;
}

std::vector<Eigen::Vector3f> toEigenPoints(const Cloud& cloud) {
  std::vector<Eigen::Vector3f> output;
  output.reserve(cloud.size());
  for (const auto& point : cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      output.emplace_back(point.x, point.y, point.z);
    }
  }
  return output;
}

pcl::PointCloud<SCPointType> toScanContextCloud(const Cloud& cloud) {
  pcl::PointCloud<SCPointType> output;
  output.reserve(cloud.size());
  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    SCPointType converted;
    converted.x = point.x;
    converted.y = point.y;
    converted.z = point.z;
    converted.intensity = 0.0F;
    output.push_back(converted);
  }
  return output;
}

CloudPtr filterAndDownsample(const CloudPtr& input,
                             float leaf_size,
                             float minimum_range,
                             float maximum_range,
                             float minimum_z,
                             float maximum_z) {
  CloudPtr filtered(new Cloud);
  filtered->reserve(input->size());
  const float minimum_range_sq = minimum_range * minimum_range;
  const float maximum_range_sq = maximum_range * maximum_range;
  for (const auto& point : input->points) {
    const float radius_sq = point.x * point.x + point.y * point.y;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || radius_sq < minimum_range_sq ||
        radius_sq > maximum_range_sq || point.z < minimum_z ||
        point.z > maximum_z) {
      continue;
    }
    filtered->push_back(point);
  }

  if (leaf_size <= 0.0F) {
    return filtered;
  }
  CloudPtr downsampled(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(filtered);
  voxel.filter(*downsampled);
  return downsampled;
}

Validation validateGeometry(const CloudPtr& query,
                            const CloudPtr& target,
                            const Eigen::Matrix4f& target_from_query,
                            double threshold_m) {
  Validation output;
  if (query->empty() || target->empty()) {
    return output;
  }
  Cloud transformed;
  pcl::transformPointCloud(*query, transformed, target_from_query);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_distance_sq(1);
  const double threshold_sq = threshold_m * threshold_m;
  std::size_t evaluated = 0;
  std::size_t inliers = 0;
  double error_sum = 0.0;
  for (std::size_t index = 0; index < transformed.size(); index += 2) {
    ++evaluated;
    if (tree.nearestKSearch(transformed[index], 1, nearest_index,
                            nearest_distance_sq) > 0 &&
        nearest_distance_sq[0] <= threshold_sq) {
      ++inliers;
      error_sum += nearest_distance_sq[0];
    }
  }
  output.inlier_fraction =
      evaluated > 0 ? static_cast<double>(inliers) / evaluated : 0.0;
  if (inliers > 0) {
    output.rmse_m = std::sqrt(error_sum / inliers);
  }
  return output;
}

long residentSetSizeKiB() {
  std::ifstream input("/proc/self/status");
  std::string key;
  while (input >> key) {
    if (key == "VmRSS:") {
      long value = 0;
      input >> value;
      return value;
    }
    std::string remainder;
    std::getline(input, remainder);
  }
  return -1;
}

}  // namespace

class OnionScanContextRelocalization {
 public:
  OnionScanContextRelocalization()
      : private_node_("~") {
    loadParameters();
    loadDatabase();

    initial_pose_publisher_ =
        node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            initial_pose_topic_, 1, true);
    status_publisher_ =
        private_node_.advertise<std_msgs::String>("status", 10, true);
    cloud_subscriber_ =
        node_.subscribe(lidar_topic_, cloud_queue_size_,
                        &OnionScanContextRelocalization::cloudCallback, this);
    if (!odom_topic_.empty()) {
      odom_subscriber_ =
          node_.subscribe(odom_topic_, 200,
                          &OnionScanContextRelocalization::odomCallback, this);
    }
    if (point_time_config_.enabled && odom_topic_.empty()) {
      if (point_time_config_.required) {
        throw std::runtime_error(
            "point deskew requires a time-aligned ~odom_topic");
      }
      ROS_WARN(
          "Point deskew is enabled but ~odom_topic is empty; raw point "
          "clouds will be used");
    }
    trigger_service_ =
        private_node_.advertiseService(
            "relocalize",
            &OnionScanContextRelocalization::triggerCallback, this);

    if (!metrics_output_path_.empty()) {
      const boost::filesystem::path path(metrics_output_path_);
      boost::filesystem::create_directories(path.parent_path());
      metrics_output_.open(path.string(), std::ios::out | std::ios::app);
      if (metrics_output_.tellp() == 0) {
          metrics_output_
              << "stamp_sec,query_points,database_entries,top_k,"
                 "descriptor_ms,kiss_ms,gicp_ms,total_ms,selected_rank,"
                 "scancontext_distance,kiss_inliers,inlier_fraction,"
                 "rmse_m,accepted,pose_x,pose_y,pose_z,"
                 "pose_qx,pose_qy,pose_qz,pose_qw,rss_kib\n";
      }
    }

    publishStatus("ready database_entries=" +
                  std::to_string(database_.size()));
    ROS_INFO_STREAM(
        "Scan Context C++ relocalizer ready: database=" << database_.size()
        << ", lidar_topic=" << lidar_topic_
        << ", query/output child="
        << (query_frame_.empty() ? "<cloud frame>" : query_frame_)
        << ", odom_topic="
        << (odom_topic_.empty() ? "<disabled>" : odom_topic_)
        << ", accumulation=" << accumulation_sec_ << " s"
        << ", Top-K=" << top_k_
        << ", point_deskew="
        << (point_time_config_.enabled ? "enabled" : "disabled"));
  }

 private:
  void loadParameters() {
    private_node_.param<std::string>("database_manifest",
                                     database_manifest_, "");
    private_node_.param<std::string>("lidar_topic", lidar_topic_,
                                     "/livox/lidar");
    private_node_.param<std::string>("query_frame", query_frame_, "");
    private_node_.param<std::string>("odom_topic", odom_topic_, "");
    private_node_.param<std::string>("map_frame", map_frame_, "odom");
    private_node_.param<std::string>("initial_pose_topic",
                                     initial_pose_topic_, "/initialpose");
    private_node_.param("cloud_queue_size", cloud_queue_size_, 100);
    private_node_.param("auto_relocalize", auto_relocalize_, true);
    private_node_.param("relocalize_once", relocalize_once_, true);
    private_node_.param("accumulation_sec", accumulation_sec_, 1.5);
    private_node_.param("minimum_accumulated_frames",
                        minimum_accumulated_frames_, 5);
    private_node_.param("require_motion_compensation",
                        require_motion_compensation_, false);
    private_node_.param("assume_static_without_odometry",
                        assume_static_without_odometry_, true);
    private_node_.param("maximum_odom_time_error_sec",
                        maximum_odom_time_error_sec_, 0.10);
    private_node_.param("enable_point_deskew",
                        point_time_config_.enabled, false);
    private_node_.param("require_point_deskew",
                        point_time_config_.required, false);
    private_node_.param<std::string>(
        "point_time_field", point_time_config_.primary_field, "t_sec");
    private_node_.param("point_time_scale",
                        point_time_config_.primary_scale, 1.0);
    private_node_.param<std::string>(
        "point_time_secondary_field",
        point_time_config_.secondary_field, "t_usec");
    private_node_.param("point_time_secondary_scale",
                        point_time_config_.secondary_scale, 1.0e-6);
    private_node_.param("point_time_is_offset",
                        point_time_config_.time_is_offset, false);
    private_node_.param("maximum_pose_interpolation_gap_sec",
                        point_time_config_.maximum_pose_gap_sec, 0.15);
    private_node_.param(
        "maximum_point_time_distance_sec",
        point_time_config_.maximum_point_time_distance_sec, 1.0);
    private_node_.param("minimum_deskewed_fraction",
                        minimum_deskewed_fraction_, 0.95);
    private_node_.param("pending_cloud_limit",
                        pending_cloud_limit_, 50);
    private_node_.param("retry_period_sec", retry_period_sec_, 2.0);
    private_node_.param("top_k", top_k_, 3);
    private_node_.param("fine_leaf_size", fine_leaf_size_, 0.20);
    private_node_.param("coarse_leaf_size", coarse_leaf_size_, 0.60);
    private_node_.param("minimum_range_m", minimum_range_m_, 2.0);
    private_node_.param("maximum_range_m", maximum_range_m_, 80.0);
    private_node_.param("minimum_z_m", minimum_z_m_, -4.0);
    private_node_.param("maximum_z_m", maximum_z_m_, 12.0);
    private_node_.param("kiss_voxel_size", kiss_voxel_size_, 0.60);
    private_node_.param("kiss_max_correspondences",
                        kiss_max_correspondences_, 3000);
    private_node_.param("kiss_use_quatro", kiss_use_quatro_, true);
    private_node_.param("kiss_use_ratio_test",
                        kiss_use_ratio_test_, true);
    private_node_.param("gicp_max_iterations", gicp_max_iterations_, 30);
    private_node_.param("gicp_max_correspondence_distance",
                        gicp_max_correspondence_distance_, 0.80);
    private_node_.param("validation_distance_m",
                        validation_distance_m_, 0.30);
    private_node_.param("minimum_kiss_inliers",
                        minimum_kiss_inliers_, 100);
    private_node_.param("minimum_inlier_fraction",
                        minimum_inlier_fraction_, 0.45);
    private_node_.param("maximum_rmse_m", maximum_rmse_m_, 0.18);
    query_frame_ = normalizeFrameId(query_frame_);
    private_node_.param<std::string>("metrics_output_path",
                                     metrics_output_path_, "");

    if (database_manifest_.empty()) {
      throw std::runtime_error("~database_manifest is required");
    }
    top_k_ = std::max(1, top_k_);
    pending_cloud_limit_ = std::max(2, pending_cloud_limit_);
    minimum_deskewed_fraction_ =
        std::clamp(minimum_deskewed_fraction_, 0.0, 1.0);
    if (point_time_config_.required &&
        !point_time_config_.enabled) {
      throw std::runtime_error(
          "require_point_deskew=true but point deskew is disabled");
    }
  }

  void loadDatabase() {
    const boost::filesystem::path manifest_path(database_manifest_);
    std::ifstream input(manifest_path.string());
    if (!input) {
      throw std::runtime_error("cannot open database manifest: " +
                               database_manifest_);
    }
    const boost::filesystem::path database_root =
        manifest_path.parent_path();
    std::string line;
    std::getline(input, line);
    std::size_t persisted_descriptors = 0;
    std::size_t legacy_descriptors = 0;
    while (std::getline(input, line)) {
      if (line.empty() || line.front() == '#') {
        continue;
      }
      const auto values = splitCsv(line);
      if (values.size() < 12) {
        throw std::runtime_error("malformed database manifest row: " +
                                 line);
      }
      DatabaseEntry entry;
      entry.id = values[0];
      entry.stamp_sec = std::stod(values[1]);
      entry.map_from_anchor = poseFromValues(values, 2);
      entry.coarse_path =
          (database_root / values[9]).string();
      entry.fine_path =
          (database_root / values[10]).string();
      if (values.size() >= 15) {
        entry.anchor_frame = normalizeFrameId(values[14]);
      }
      if (!query_frame_.empty() && !entry.anchor_frame.empty() &&
          entry.anchor_frame != query_frame_) {
        throw std::runtime_error(
            "database entry " + entry.id + " uses anchor frame '" +
            entry.anchor_frame + "' but runtime query_frame is '" +
            query_frame_ + "'");
      }
      if (values.size() >= 13 && !values[12].empty()) {
        const auto descriptor_path =
            (database_root / values[12]).string();
        entry.descriptor =
            onion_relocalization::loadDescriptor(
                descriptor_path);
        ++persisted_descriptors;
      } else {
        entry.coarse.reset(new Cloud);
        if (pcl::io::loadPCDFile(entry.coarse_path,
                                 *entry.coarse) != 0) {
          throw std::runtime_error(
              "cannot load legacy coarse point cloud for entry " +
              entry.id);
        }
        auto context_cloud = toScanContextCloud(*entry.coarse);
        entry.descriptor =
            scan_context_.makeScancontext(context_cloud);
        ++legacy_descriptors;
      }
      database_.push_back(std::move(entry));
    }
    if (database_.empty()) {
      throw std::runtime_error("database manifest contains no entries");
    }
    ROS_INFO(
        "database descriptor loading: persisted=%zu legacy_computed=%zu; "
        "candidate PCDs are loaded lazily",
        persisted_descriptors, legacy_descriptors);
  }

  bool ensurePointCloudsLoaded(DatabaseEntry* entry) {
    if (!entry->coarse) {
      entry->coarse.reset(new Cloud);
      if (pcl::io::loadPCDFile(entry->coarse_path,
                               *entry->coarse) != 0) {
        entry->coarse.reset();
        return false;
      }
    }
    if (!entry->fine) {
      entry->fine.reset(new Cloud);
      if (pcl::io::loadPCDFile(entry->fine_path, *entry->fine) != 0) {
        entry->fine.reset();
        return false;
      }
    }
    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    if (!odometry_.empty() &&
        (message->header.stamp - odometry_.back().stamp).toSec() <
            -1.0) {
      ROS_WARN(
          "Odometry time moved backwards; clearing relocalization motion "
          "buffers for a new bag/live epoch");
      odometry_.clear();
      pending_clouds_.clear();
      frames_.clear();
      last_attempt_ = ros::Time();
      deskewed_clouds_ = 0;
      missing_earliest_odom_clouds_ = 0;
    }
    OdomSample sample;
    sample.stamp = message->header.stamp;
    sample.odom_from_sensor = poseFromOdometry(*message);
    odometry_.push_back(std::move(sample));
    if (odometry_.size() >= 2 &&
        odometry_[odometry_.size() - 2].stamp >
            odometry_.back().stamp) {
      std::sort(odometry_.begin(), odometry_.end(),
                [](const OdomSample& left,
                   const OdomSample& right) {
                  return left.stamp < right.stamp;
                });
    }
    const ros::Time oldest =
        odometry_.back().stamp -
        ros::Duration(accumulation_sec_ + 3.0);
    while (!odometry_.empty() && odometry_.front().stamp < oldest) {
      odometry_.pop_front();
    }
    drainPendingClouds();
  }

  std::vector<TimedPose> motionPoses() const {
    std::vector<TimedPose> output;
    output.reserve(odometry_.size());
    for (const auto& sample : odometry_) {
      TimedPose timed_pose;
      timed_pose.stamp_sec = sample.stamp.toSec();
      timed_pose.pose = sample.odom_from_sensor;
      output.push_back(std::move(timed_pose));
    }
    return output;
  }

  bool motionPose(const ros::Time& stamp,
                  Eigen::Isometry3d* pose) const {
    const auto poses = motionPoses();
    if (onion_relocalization::interpolatePose(
            poses, stamp.toSec(),
            point_time_config_.maximum_pose_gap_sec, pose)) {
      return true;
    }
    if (odometry_.empty()) {
      return false;
    }
    auto best = odometry_.begin();
    double best_error = std::abs((best->stamp - stamp).toSec());
    for (auto iterator = std::next(odometry_.begin());
         iterator != odometry_.end(); ++iterator) {
      const double error = std::abs((iterator->stamp - stamp).toSec());
      if (error < best_error) {
        best = iterator;
        best_error = error;
      }
    }
    if (best_error > maximum_odom_time_error_sec_) {
      return false;
    }
    *pose = best->odom_from_sensor;
    return true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    if (succeeded_ && relocalize_once_) {
      return;
    }
    if (point_time_config_.enabled && !odom_topic_.empty()) {
      pending_clouds_.push_back(message);
      if (pending_clouds_.size() >
          static_cast<std::size_t>(pending_cloud_limit_)) {
        const auto oldest = pending_clouds_.front();
        pending_clouds_.pop_front();
        if (point_time_config_.required) {
          ROS_WARN_THROTTLE(
              1.0,
              "Dropping a point cloud because deskew odometry did not "
              "arrive before the pending queue filled");
        } else {
          processCloud(oldest, CloudPtr());
        }
      }
      drainPendingClouds();
      return;
    }
    processCloud(message, CloudPtr());
  }

  void drainPendingClouds() {
    if (draining_pending_clouds_ || pending_clouds_.empty()) {
      return;
    }
    draining_pending_clouds_ = true;
    while (!pending_clouds_.empty()) {
      const auto message = pending_clouds_.front();
      double minimum_stamp = 0.0;
      double maximum_stamp = 0.0;
      std::string time_error;
      if (!onion_relocalization::pointTimeRange(
              *message, point_time_config_, &minimum_stamp,
              &maximum_stamp, &time_error)) {
        pending_clouds_.pop_front();
        if (point_time_config_.required) {
          ROS_WARN_THROTTLE(
              1.0, "Point deskew rejected a cloud: %s",
              time_error.c_str());
        } else {
          ROS_WARN_THROTTLE(
              1.0, "Point deskew fallback to raw cloud: %s",
              time_error.c_str());
          processCloud(message, CloudPtr());
        }
        continue;
      }
      if (odometry_.empty() ||
          odometry_.back().stamp.toSec() + 1.0e-6 <
              maximum_stamp) {
        break;
      }
      pending_clouds_.pop_front();
      if (odometry_.front().stamp.toSec() - 1.0e-6 >
          minimum_stamp) {
        ++missing_earliest_odom_clouds_;
        ROS_WARN_THROTTLE(
            1.0,
            "Point deskew rejected a cloud because its earliest odometry "
            "sample is missing (missing=%.6f s, odom=%zu, pending=%zu)",
            odometry_.front().stamp.toSec() - minimum_stamp,
            odometry_.size(), pending_clouds_.size());
        if (!point_time_config_.required) {
          processCloud(message, CloudPtr());
        }
        continue;
      }

      const auto deskewed =
          onion_relocalization::deskewPointCloud(
              *message, point_time_config_, motionPoses());
      const std::size_t total =
          deskewed.valid_points + deskewed.invalid_points;
      const double valid_fraction =
          total > 0
              ? static_cast<double>(deskewed.valid_points) /
                    static_cast<double>(total)
              : 0.0;
      if (!deskewed.ok() ||
          valid_fraction < minimum_deskewed_fraction_) {
        ROS_WARN_THROTTLE(
            1.0,
            "Point deskew rejected a cloud: %s valid_fraction=%.3f",
            deskewed.error.empty() ? "insufficient pose coverage"
                                  : deskewed.error.c_str(),
            valid_fraction);
        if (!point_time_config_.required) {
          processCloud(message, CloudPtr());
        }
        continue;
      }
      ++deskewed_clouds_;
      if (deskewed_clouds_ == 1 || deskewed_clouds_ % 10 == 0) {
        ROS_INFO(
            "Point deskew accepted cloud=%zu valid_fraction=%.3f "
            "point_span=%.6f s",
            deskewed_clouds_, valid_fraction,
            deskewed.maximum_point_stamp_sec -
                deskewed.minimum_point_stamp_sec);
      }
      processCloud(message, deskewed.cloud);
    }
    draining_pending_clouds_ = false;
  }

  void processCloud(
      const sensor_msgs::PointCloud2::ConstPtr& message,
      const CloudPtr& deskewed_cloud) {
    const std::string cloud_frame =
        normalizeFrameId(message->header.frame_id);
    if (cloud_frame.empty() ||
        (!query_frame_.empty() && cloud_frame != query_frame_)) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Scan Context rejected query frame '"
                   << message->header.frame_id << "'; expected '"
                   << query_frame_ << "'");
      return;
    }
    CloudPtr raw = deskewed_cloud;
    if (!raw) {
      raw.reset(new Cloud);
      pcl::fromROSMsg(*message, *raw);
    }
    CloudFrame frame;
    frame.stamp = message->header.stamp.isZero()
                      ? ros::Time::now()
                      : message->header.stamp;
    frame.fine = filterAndDownsample(
        raw, static_cast<float>(fine_leaf_size_),
        static_cast<float>(minimum_range_m_),
        static_cast<float>(maximum_range_m_),
        static_cast<float>(minimum_z_m_),
        static_cast<float>(maximum_z_m_));
    frame.has_motion_pose =
        motionPose(frame.stamp, &frame.motion_from_sensor);
    if (require_motion_compensation_ && !frame.has_motion_pose) {
      ROS_WARN_THROTTLE(
          1.0,
          "Scan Context is waiting for time-aligned relative odometry");
      return;
    }
    frames_.push_back(std::move(frame));
    const ros::Time oldest =
        frames_.back().stamp - ros::Duration(accumulation_sec_ + 0.5);
    while (!frames_.empty() && frames_.front().stamp < oldest) {
      frames_.pop_front();
    }

    if (!auto_relocalize_ && !manual_request_) {
      return;
    }
    if (!last_attempt_.isZero() &&
        (frames_.back().stamp - last_attempt_).toSec() <
            retry_period_sec_) {
      return;
    }
    if (frames_.size() <
        static_cast<std::size_t>(minimum_accumulated_frames_)) {
      return;
    }
    const double span =
        (frames_.back().stamp - frames_.front().stamp).toSec();
    if (span + 0.05 < accumulation_sec_) {
      return;
    }
    last_attempt_ = frames_.back().stamp;
    manual_request_ = false;
    attemptRelocalization();
  }

  std::pair<CloudPtr, CloudPtr> aggregateQuery() const {
    CloudPtr accumulated(new Cloud);
    const auto& anchor = frames_.back();
    const bool can_compensate =
        std::all_of(frames_.begin(), frames_.end(),
                    [](const CloudFrame& frame) {
                      return frame.has_motion_pose;
                    });
    if (!can_compensate && !assume_static_without_odometry_) {
      *accumulated = *anchor.fine;
    } else {
      for (const auto& frame : frames_) {
        if (can_compensate) {
          const Eigen::Matrix4f anchor_from_frame =
              (anchor.motion_from_sensor.inverse() *
               frame.motion_from_sensor)
                  .matrix()
                  .cast<float>();
          Cloud transformed;
          pcl::transformPointCloud(*frame.fine, transformed,
                                   anchor_from_frame);
          *accumulated += transformed;
        } else {
          *accumulated += *frame.fine;
        }
      }
    }
    CloudPtr fine = filterAndDownsample(
        accumulated, static_cast<float>(fine_leaf_size_),
        static_cast<float>(minimum_range_m_),
        static_cast<float>(maximum_range_m_),
        static_cast<float>(minimum_z_m_),
        static_cast<float>(maximum_z_m_));
    CloudPtr coarse = filterAndDownsample(
        fine, static_cast<float>(coarse_leaf_size_),
        static_cast<float>(minimum_range_m_),
        static_cast<float>(maximum_range_m_),
        static_cast<float>(minimum_z_m_),
        static_cast<float>(maximum_z_m_));
    return {coarse, fine};
  }

  std::vector<Candidate> retrieveCandidates(
      const Eigen::MatrixXd& query) {
    std::vector<Candidate> candidates;
    candidates.reserve(database_.size());
    for (std::size_t index = 0; index < database_.size(); ++index) {
      Candidate candidate;
      candidate.database_index = index;
      Eigen::MatrixXd query_copy = query;
      Eigen::MatrixXd database_copy = database_[index].descriptor;
      const auto distance = scan_context_.distanceBtnScanContext(
          query_copy, database_copy);
      candidate.scancontext_distance = distance.first;
      candidate.sector_shift = distance.second;
      candidates.push_back(std::move(candidate));
    }
    const std::size_t keep =
        std::min<std::size_t>(top_k_, candidates.size());
    std::partial_sort(
        candidates.begin(), candidates.begin() + keep, candidates.end(),
        [](const Candidate& left, const Candidate& right) {
          return left.scancontext_distance <
                 right.scancontext_distance;
        });
    candidates.resize(keep);
    return candidates;
  }

  EvaluatedCandidate evaluateCandidate(
      const Candidate& candidate,
      std::size_t rank,
      const CloudPtr& coarse_query,
      const CloudPtr& fine_query) {
    EvaluatedCandidate output;
    output.database_index = candidate.database_index;
    output.rank = rank;
    auto& entry = database_[candidate.database_index];
    try {
      if (!ensurePointCloudsLoaded(&entry)) {
        throw std::runtime_error(
            "cannot lazily load candidate point clouds");
      }
      kiss_matcher::KISSMatcherConfig config(
          static_cast<float>(kiss_voxel_size_));
      config.use_quatro_ = kiss_use_quatro_;
      config.use_ratio_test_ = kiss_use_ratio_test_;
      config.num_max_corr_ = kiss_max_correspondences_;
      kiss_matcher::KISSMatcher matcher(config);

      const auto kiss_begin = Clock::now();
      const auto solution =
          matcher.estimate(toEigenPoints(*coarse_query),
                           toEigenPoints(*entry.coarse));
      output.kiss_ms =
          std::chrono::duration<double, std::milli>(
              Clock::now() - kiss_begin)
              .count();
      output.kiss_inliers = matcher.getNumFinalInliers();

      Eigen::Matrix4f target_from_query = Eigen::Matrix4f::Identity();
      target_from_query.block<3, 3>(0, 0) =
          solution.rotation.cast<float>();
      target_from_query.block<3, 1>(0, 3) =
          solution.translation.cast<float>();

      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,
                                            pcl::PointXYZ>
          gicp;
      gicp.setInputSource(fine_query);
      gicp.setInputTarget(entry.fine);
      gicp.setMaximumIterations(gicp_max_iterations_);
      gicp.setMaxCorrespondenceDistance(
          gicp_max_correspondence_distance_);
      gicp.setTransformationEpsilon(1e-4);
      gicp.setEuclideanFitnessEpsilon(1e-5);
      Cloud aligned;
      const auto gicp_begin = Clock::now();
      gicp.align(aligned, target_from_query);
      output.gicp_ms =
          std::chrono::duration<double, std::milli>(
              Clock::now() - gicp_begin)
              .count();
      output.gicp_converged = gicp.hasConverged();
      if (output.gicp_converged) {
        target_from_query = gicp.getFinalTransformation();
      }
      output.validation =
          validateGeometry(fine_query, entry.fine, target_from_query,
                           validation_distance_m_);

      Eigen::Isometry3d target_transform =
          Eigen::Isometry3d::Identity();
      target_transform.matrix() = target_from_query.cast<double>();
      output.map_from_query =
          entry.map_from_anchor * target_transform;
      output.accepted =
          output.gicp_converged &&
          output.kiss_inliers >=
              static_cast<std::size_t>(minimum_kiss_inliers_) &&
          output.validation.inlier_fraction >= minimum_inlier_fraction_ &&
          output.validation.rmse_m <= maximum_rmse_m_;
      output.score =
          output.validation.rmse_m -
          0.35 * output.validation.inlier_fraction +
          (output.accepted ? 0.0 : 10.0);
      output.valid = true;
    } catch (const std::exception& error) {
      ROS_WARN("KISS candidate %s failed: %s", entry.id.c_str(),
               error.what());
    }
    return output;
  }

  void attemptRelocalization() {
    const auto total_begin = Clock::now();
    publishStatus("matching");
    const auto query_clouds = aggregateQuery();
    if (query_clouds.first->size() < 200) {
      ROS_WARN("Relocalization query has only %zu coarse points",
               query_clouds.first->size());
      publishStatus("rejected too_few_query_points");
      return;
    }

    const auto descriptor_begin = Clock::now();
    auto descriptor_cloud = toScanContextCloud(*query_clouds.first);
    const auto descriptor =
        scan_context_.makeScancontext(descriptor_cloud);
    const auto candidates = retrieveCandidates(descriptor);
    const double descriptor_ms =
        std::chrono::duration<double, std::milli>(
            Clock::now() - descriptor_begin)
            .count();

    EvaluatedCandidate selected;
    for (std::size_t rank = 0; rank < candidates.size(); ++rank) {
      const auto result =
          evaluateCandidate(candidates[rank], rank + 1,
                            query_clouds.first, query_clouds.second);
      if (result.valid &&
          (!selected.valid || result.score < selected.score)) {
        selected = result;
      }
    }
    const double total_ms =
        std::chrono::duration<double, std::milli>(
            Clock::now() - total_begin)
            .count();

    const Candidate* selected_context = nullptr;
    if (selected.valid && selected.rank > 0 &&
        selected.rank <= candidates.size()) {
      selected_context = &candidates[selected.rank - 1];
    }
    if (selected.accepted) {
      publishInitialPose(selected.map_from_query, frames_.back().stamp);
      succeeded_ = true;
      publishStatus("accepted entry=" +
                    database_[selected.database_index].id);
    } else {
      publishStatus("rejected geometry_validation");
    }

    if (selected.valid && selected_context) {
      ROS_INFO_STREAM(
          "Scan Context relocalization " <<
          (selected.accepted ? "ACCEPTED" : "REJECTED")
          << ": entry=" << database_[selected.database_index].id
          << ", rank=" << selected.rank
          << ", SC="
          << selected_context->scancontext_distance
          << ", yaw_hint_deg="
          << selected_context->sector_shift *
                 scan_context_.PC_UNIT_SECTORANGLE
          << ", KISS_inliers=" << selected.kiss_inliers
          << ", geom=" << selected.validation.inlier_fraction
          << ", rmse=" << selected.validation.rmse_m
          << " m, total=" << total_ms << " ms");
    } else {
      ROS_WARN("All Scan Context Top-K candidates failed");
    }

    if (metrics_output_.is_open()) {
      const double missing_pose =
          std::numeric_limits<double>::quiet_NaN();
      Eigen::Quaterniond selected_quaternion =
          Eigen::Quaterniond::Identity();
      if (selected.valid) {
        selected_quaternion =
            Eigen::Quaterniond(selected.map_from_query.linear());
        selected_quaternion.normalize();
      }
      metrics_output_ << std::fixed << std::setprecision(6)
                      << frames_.back().stamp.toSec() << ","
                      << query_clouds.first->size() << ","
                      << database_.size() << "," << candidates.size()
                      << "," << descriptor_ms << ","
                      << (selected.valid ? selected.kiss_ms : -1.0)
                      << ","
                      << (selected.valid ? selected.gicp_ms : -1.0)
                      << "," << total_ms << ","
                      << (selected.valid ? selected.rank : 0) << ","
                      << (selected_context
                              ? selected_context->scancontext_distance
                              : -1.0)
                      << ","
                      << (selected.valid ? selected.kiss_inliers : 0)
                      << ","
                      << (selected.valid
                              ? selected.validation.inlier_fraction
                              : 0.0)
                      << ","
                       << (selected.valid ? selected.validation.rmse_m
                                          : -1.0)
                       << "," << selected.accepted << ","
                       << (selected.valid
                               ? selected.map_from_query.translation().x()
                               : missing_pose)
                       << ","
                       << (selected.valid
                               ? selected.map_from_query.translation().y()
                               : missing_pose)
                       << ","
                       << (selected.valid
                               ? selected.map_from_query.translation().z()
                               : missing_pose)
                       << ","
                       << (selected.valid ? selected_quaternion.x()
                                          : missing_pose)
                       << ","
                       << (selected.valid ? selected_quaternion.y()
                                          : missing_pose)
                       << ","
                       << (selected.valid ? selected_quaternion.z()
                                          : missing_pose)
                       << ","
                       << (selected.valid ? selected_quaternion.w()
                                          : missing_pose)
                       << ","
                       << residentSetSizeKiB() << "\n";
      metrics_output_.flush();
    }
  }

  void publishInitialPose(const Eigen::Isometry3d& pose,
                          const ros::Time& stamp) {
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.pose.pose.position.x = pose.translation().x();
    message.pose.pose.position.y = pose.translation().y();
    message.pose.pose.position.z = pose.translation().z();
    Eigen::Quaterniond quaternion(pose.linear());
    quaternion.normalize();
    message.pose.pose.orientation.x = quaternion.x();
    message.pose.pose.orientation.y = quaternion.y();
    message.pose.pose.orientation.z = quaternion.z();
    message.pose.pose.orientation.w = quaternion.w();
    message.pose.covariance.fill(0.0);
    message.pose.covariance[0] = 0.05 * 0.05;
    message.pose.covariance[7] = 0.05 * 0.05;
    message.pose.covariance[14] = 0.10 * 0.10;
    message.pose.covariance[21] = 0.02 * 0.02;
    message.pose.covariance[28] = 0.02 * 0.02;
    message.pose.covariance[35] = 0.03 * 0.03;
    initial_pose_publisher_.publish(message);
    ROS_INFO_STREAM(
        "Published Scan Context /initialpose for child '"
        << (query_frame_.empty() ? "<cloud frame>" : query_frame_)
        << "' at xyz=(" << pose.translation().x() << ", "
        << pose.translation().y() << ", " << pose.translation().z()
        << ")");
  }

  bool triggerCallback(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& response) {
    manual_request_ = true;
    succeeded_ = false;
    response.success = true;
    response.message =
        "relocalization scheduled after the accumulation window is ready";
    return true;
  }

  void publishStatus(const std::string& value) {
    std_msgs::String message;
    message.data = value;
    status_publisher_.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  SCManager scan_context_;
  std::vector<DatabaseEntry> database_;
  std::deque<OdomSample> odometry_;
  std::deque<CloudFrame> frames_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> pending_clouds_;

  ros::Subscriber cloud_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher initial_pose_publisher_;
  ros::Publisher status_publisher_;
  ros::ServiceServer trigger_service_;

  std::string database_manifest_;
  std::string lidar_topic_;
  std::string query_frame_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string initial_pose_topic_;
  std::string metrics_output_path_;
  int cloud_queue_size_ = 100;
  bool auto_relocalize_ = true;
  bool relocalize_once_ = true;
  bool require_motion_compensation_ = false;
  bool assume_static_without_odometry_ = true;
  double accumulation_sec_ = 1.5;
  int minimum_accumulated_frames_ = 5;
  double maximum_odom_time_error_sec_ = 0.10;
  PointTimeConfig point_time_config_;
  double minimum_deskewed_fraction_ = 0.95;
  int pending_cloud_limit_ = 50;
  double retry_period_sec_ = 2.0;
  int top_k_ = 3;
  double fine_leaf_size_ = 0.20;
  double coarse_leaf_size_ = 0.60;
  double minimum_range_m_ = 2.0;
  double maximum_range_m_ = 80.0;
  double minimum_z_m_ = -4.0;
  double maximum_z_m_ = 12.0;
  double kiss_voxel_size_ = 0.60;
  int kiss_max_correspondences_ = 3000;
  bool kiss_use_quatro_ = true;
  bool kiss_use_ratio_test_ = true;
  int gicp_max_iterations_ = 30;
  double gicp_max_correspondence_distance_ = 0.80;
  double validation_distance_m_ = 0.30;
  int minimum_kiss_inliers_ = 100;
  double minimum_inlier_fraction_ = 0.45;
  double maximum_rmse_m_ = 0.18;
  bool manual_request_ = false;
  bool succeeded_ = false;
  bool draining_pending_clouds_ = false;
  std::size_t deskewed_clouds_ = 0;
  std::size_t missing_earliest_odom_clouds_ = 0;
  ros::Time last_attempt_;
  std::ofstream metrics_output_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "onion_scancontext_relocalization");
  try {
    OnionScanContextRelocalization node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Scan Context++ relocalizer failed: %s", error.what());
    return 2;
  }
  return 0;
}
